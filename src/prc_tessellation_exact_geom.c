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

/* A standard type for curve sampling */
typedef prc_vec3 (*curve_func)(prc_context *ctx, void *params, double input);

/* Evaluate parabola at a single point using the modified spec formula (spec has an error) */
static prc_vec3
prc_evaluate_parabola(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_parabola *parabola = (prc_crv_parabola *)params;
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

/* Evaluate line at a single point */
static prc_vec3
prc_evaluate_line(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;

    output.x = input;
    output.y = 0;
    output.z = 0;

    return output;
}

/* Evaluate hyperbola at a single point */
static prc_vec3
prc_evaluate_hyperbola(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_hyperbola *hyperbola = (prc_crv_hyperbola *)params;
    double semi_axis_image = hyperbola->semi_axis_image;
    double semi_axis = hyperbola->semi_axis;
    double temp;

    output.z = 0;

    if (hyperbola->type == 0)
    {
        if (semi_axis_image == 0.0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid hyperbola semi_axis_image in prc_evaluate_hyperbola\n");
            return output;
        }
        temp = input / semi_axis;
        output.x = semi_axis_image * sqrt(1.0 + temp * temp);
        output.y = input;
    }
    else
    {
        output.x = semi_axis_image * cosh(input);
        output.y = semi_axis * sinh(input);
    }

    return output;
}

/* Evaluate circle at a single point */
static prc_vec3
prc_evaluate_circle(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_circle *circle = (prc_crv_circle *)params;
    double radius = circle->radius;

    output.z = 0;
    output.x = radius * cos(input);
    output.y = radius * sin(input);

    return output;
}

/* Evaluate an ellipse at a single point */
static prc_vec3
prc_evaluate_ellipse(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_ellipse *ellipse = (prc_crv_ellipse *)params;
    double rx = ellipse->rx;
    double ry = ellipse->ry;

    output.z = 0;
    output.x = rx * cos(input);
    output.y = ry * sin(input);

    return output;
}

/* Evaluate a helix at a single point */
#if 0
static prc_vec3
prc_evaluate_helix(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_helix01 *helix = (prc_crv_helix01 *)params;
    uint8_t type = helix->type;

    if (type == 0)
    {
        double radius = helix->type0_helix.radius;
        double pitch = helix->type0_helix.pitch;
        prc_vec3 origin;

        origin.x = helix->type0_helix.origin_0;
        origin.y = helix->type0_helix.origin_1;
        origin.z = helix->type0_helix.origin_2;



    }
    else
    {

    }

}
#endif

static int
prc_sample_curve(prc_context *ctx, prc_data *data, prc_content_wire_edge *curve)
{
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;
    uint8_t curve_approx_good = 0;
    void *curve_params = NULL;
    curve_func curve_eval_func = NULL;
    double start;
    double end;
    uint32_t i;
    double t, t0, t1, dist;
    prc_vec3 p0, p1, mid, seg_mid;
    uint32_t num_samples;

    switch (curve->ptr_curve.curve_type)
    {
        case PRC_TYPE_CRV_Parabola:
        {
            prc_crv_parabola *parabola = curve->ptr_curve.crv_parabola;
            prc_parameterization params = parabola->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = (void *)parabola;
            curve_eval_func = prc_evaluate_parabola;
            num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Line:
        {
            prc_crv_line *line = curve->ptr_curve.crv_line;
            prc_parameterization params = line->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = NULL;
            curve_eval_func = prc_evaluate_line;
            num_samples = 2;
            break;
        }

        case PRC_TYPE_CRV_Hyperbola:
        {
            prc_crv_hyperbola *hyperbola = curve->ptr_curve.crv_hyperbola;
            prc_parameterization params = hyperbola->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = (void *)hyperbola;
            curve_eval_func = prc_evaluate_hyperbola;
            num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Circle:
        {
            prc_crv_circle *circle = curve->ptr_curve.crv_circle;
            prc_parameterization params = circle->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = (void *)circle;
            curve_eval_func = prc_evaluate_circle;
            num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Ellipse:
        {
            prc_crv_ellipse *ellipse = curve->ptr_curve.crv_ellipse;
            prc_parameterization params = ellipse->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = (void *)ellipse;
            curve_eval_func = prc_evaluate_ellipse;
            num_samples = CURVE_SAMPLES;
            break;
        }

        /* Disable this one for now */
#if 0
        case PRC_TYPE_CRV_Helix01:
        {
            prc_crv_helix01 *helix = curve->ptr_curve.crv_helix01;
            prc_parameterization params = helix->parameterization;
            start = params.interval.min_value;
            end = params.interval.max_value;
            curve_params = (void *)helix;
            curve_eval_func = prc_evaluate_helix;
            num_samples = CURVE_SAMPLES;
            break;
        }
#endif
        default:
            data->exact_geom_tess[geom_count].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;
    }

    /* Create a set of samples across the range, evaluate the curve
     * then evaluate at the midpoint of each segment and see if the distance is within tolerance.
     * If not, subdivide the segment and repeat until we have a good set of samples */

    if (curve_eval_func == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid curve evaluation function in prc_sample_curve\n");
        return PRC_ERROR_INTERNAL;
    }
    while (!curve_approx_good)
    {
        curve_approx_good = 1;

        for (i = 0; i < num_samples - 1; i++)
        {
            t0 = start + (end - start) * ((double)i / (double)(num_samples - 1));
            t1 = start + (end - start) * ((double)(i + 1) / (double)(num_samples - 1));
            p0 = curve_eval_func(ctx, curve_params, t0);
            p1 = curve_eval_func(ctx, curve_params, t1);
            mid = curve_eval_func(ctx, curve_params, (t0 + t1) / 2.0);

            /* Evaluate the midpoint of the segment */
            seg_mid;
            seg_mid.x = (p0.x + p1.x) / 2.0;
            seg_mid.y = (p0.y + p1.y) / 2.0;
            seg_mid.z = (p0.z + p1.z) / 2.0;

            /* Calculate the distance from the midpoint to the curve */
            dist = sqrt((mid.x - seg_mid.x) * (mid.x - seg_mid.x) +
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

    for (i = 0; i < num_samples; i++)
    {
        t = start + (end - start) * ((double)i / (double)(num_samples - 1));
        data->exact_geom_tess[geom_count].wire_data->points[i] = curve_eval_func(ctx, curve_params, t);
    }

#if 0
    /* Lets print the points for a sanity check */
    for (uint32_t i = 0; i < num_samples; i++)
    {
        prc_vec3 p = data->exact_geom_tess[geom_count].wire_data->points[i];
        printf("prc_sample_curve: point[%u] = (%f, %f, %f)\n", i, p.x, p.y, p.z);
    }
#endif

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