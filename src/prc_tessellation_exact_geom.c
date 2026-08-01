/* Copyright (C) 2023-2026 CascadiaVoxel LLC

    nanoPRC is free software: you can redistribute it and/or modify it under
    the terms of the GNU Affero General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
    License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.
*/

/* Methods to create wire and tessellation structures from exact geometry */

#include "prc_data.h"
#include <stdio.h>
#include <math.h>

#define CURVE_SAMPLES 256
#define CURVE_PRECISION 1e-6

/* Evaluate parabola at a single point using the modified spec formula (spec has an error) */
static prc_vec3
prc_evaluate_parabola(prc_context *ctx, prc_crv_parabola *parabola, double input)
{
    prc_vec3 output;
    double focal_length = parabola->focal_length;

    output.z = 0;

    if (parabola->type == 0)
    {
        double param2 = input * input;
        double p2 = param2 / sqrt(16.0 * focal_length * focal_length + param2);
        output.x = p2;
        output.y = 2.0 * focal_length * sqrt(p2 / focal_length);
        if (input < 0.0)
        {
            output.y = -output.y;
        }
    }
    else
    {
        output.x = focal_length * input * input;
        output.y = 2.0 * focal_length * input;
    }

    return output;
}

static int
prc_sample_curve(prc_context *ctx, prc_data *data, prc_content_wire_edge *curve)
{
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;

    /* Only doing parabola to start... */
    switch (curve->ptr_curve.curve_type)
    {
        case PRC_TYPE_CRV_Parabola:
        {
            prc_crv_parabola *parabola = curve->ptr_curve.crv_parabola;
            prc_parameterization params = parabola->parameterization;
            double focal_length = parabola->focal_length;
            uint8_t type = parabola->type;
            uint8_t curve_approx_good = 0;

            /* Create a set of samples across the range, evaluate the curve
             * then evaluate at the midpoint of each segment and see if the distance is within tolerance.
             * If not, subdivide the segment and repeat until we have a good set of samples */
            uint32_t num_samples = CURVE_SAMPLES;
            double start = params.interval.min_value;
            double end = params.interval.max_value;

            while (!curve_approx_good)
            {
                curve_approx_good = 1;

                for (uint32_t i = 0; i < num_samples - 1; i++)
                {
                    double t0 = start + (end - start) * ((double)i / (double)(num_samples - 1));
                    double t1 = start + (end - start) * ((double)(i + 1) / (double)(num_samples - 1));
                    prc_vec3 p0 = prc_evaluate_parabola(ctx, parabola, t0);
                    prc_vec3 p1 = prc_evaluate_parabola(ctx, parabola, t1);
                    prc_vec3 mid = prc_evaluate_parabola(ctx, parabola, (t0 + t1) / 2.0);

                    /* Evaluate the midpoint of the segment */
                    prc_vec3 seg_mid;
                    seg_mid.x = (p0.x + p1.x) / 2.0;
                    seg_mid.y = (p0.y + p1.y) / 2.0;
                    seg_mid.z = (p0.z + p1.z) / 2.0;

                    /* Calculate the distance from the midpoint to the curve */
                    double dist = sqrt((mid.x - seg_mid.x) * (mid.x - seg_mid.x) +
                        (mid.y - seg_mid.y) * (mid.y - seg_mid.y) +
                        (mid.z - seg_mid.z) * (mid.z - seg_mid.z));
                    if (dist > CURVE_PRECISION)
                    {
                        curve_approx_good = 0;
                        break;
                    }
                }
                if (!curve_approx_good)
                {
                    num_samples *= 2;
                }
            }

            /* We now have a sufficient precision on the curve. Lets generate the
               XYZ sample points and store them */
            data->exact_geom_tess[geom_count].wire_data =
                (prc_exact_geom_wire_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_wire_data));
            if (data->exact_geom_tess[geom_count].wire_data == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data in prc_sample_curve\n");
                return PRC_ERROR_MEMORY;
            }

            data->exact_geom_tess[geom_count].wire_data->number_of_points = num_samples;
            data->exact_geom_tess[geom_count].wire_data->points = (prc_vec3 *)prc_calloc(ctx, num_samples, sizeof(prc_vec3));
            if (data->exact_geom_tess[geom_count].wire_data->points == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data points in prc_sample_curve\n");
                return PRC_ERROR_MEMORY;
            }

            for (uint32_t i = 0; i < num_samples; i++)
            {
                double t = start + (end - start) * ((double)i / (double)(num_samples - 1));
                data->exact_geom_tess[geom_count].wire_data->points[i] = prc_evaluate_parabola(ctx, parabola, t);
            }

#if 0
            /* Lets print the points for a sanity check */
            for (uint32_t i = 0; i < num_samples; i++)
            {
                prc_vec3 p = data->exact_geom_tess[geom_count].wire_data->points[i];
                printf("prc_sample_curve: point[%u] = (%f, %f, %f)\n", i, p.x, p.y, p.z);
            }
#endif
            break;
        }

        default:
            data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;

    }
    return 0;
}

int
prc_approximate_exact_geom(prc_context *ctx, prc_api_data data_in)
{
    prc_data *data = (prc_data *)data_in;
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;
    int code;

    /* Lets figure out if we are doing a curve or a triangle tessellation */
    prc_topo *topo = &data->file_struct[file_index].geometry->exact_geometry.topo_contexts[topo_index].bodies[body_index];

    switch (topo->tag)
    {

    /* A simple case to start out */
    case PRC_TYPE_TOPO_SingleWireBody:
    {
        data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_WIRE;

        prc_topo_single_wire_body *body = topo->topo_single_wire_body;
        if (body->wire_body.is_stored == 1)
        {
            /* We have to find this one. For now we skip this case */
            data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;
        }

        /* Get the type of topo that this is. For now we just do wire edge.. */
        switch (body->wire_body.topo->tag)
        {
        case PRC_TYPE_TOPO_WireEdge:
        {
            prc_topo_wire_edge *wire_edge = body->wire_body.topo->topo_wire_edge;
            code = prc_sample_curve(ctx, data, &wire_edge->curve);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_sample_curve\n");
                return code;
            }
            break;
        }
        default:
            data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;
        }

        break;
    }
    case PRC_TYPE_TOPO_WireEdge:
    case PRC_TYPE_TOPO_Edge:
    case PRC_TYPE_TOPO_CoEdge:
    case PRC_TYPE_TOPO_Loop:
    case PRC_TYPE_TOPO_SingleWireBodyCompress:
    case PRC_TYPE_TOPO_WireBody:
        data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
        break;

    case PRC_TYPE_TOPO_Body:
    case PRC_TYPE_TOPO_BrepData:
    case PRC_TYPE_TOPO_BrepDataCompress:
    case PRC_TYPE_TOPO_Face:
        data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
        break;

    default:
        data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
    }

    return 0;
}