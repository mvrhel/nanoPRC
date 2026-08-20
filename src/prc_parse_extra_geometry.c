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

#include <stdlib.h>
#include <string.h>
#include "prc_parse_extra_geometry.h"
#include "prc_parse_common.h"
#include "debug.h"

#define TRIM_LOOP_INITIAL_SIZE 10
#define CURVES_PERLOOP_INITIAL_SIZE 10
#define BREP_VERTEX_INITIAL_SIZE 1000
#define BREP_EDGE_INITIAL_SIZE 1000
/* Generous upper bound on a NURBS surface's control-point grid (u*v). No
   legitimate CAD model approaches this; it exists to reject file-supplied
   counts before their product is used for allocation sizing/loop bounds. */
#define PRC_MAX_NURBS_CONTROL_POINTS 16000000ULL

/* Forward declarations due to circular dependencies */
static int prc_parse_ptr_curve(prc_context *ctx, prc_bit_state *bit_state,
    prc_ptr_curve *data);
static int prc_parse_ptr_surface(prc_context *ctx, prc_bit_state *bit_state,
    prc_ptr_surface *data);
static int prc_parse_compressed_point(prc_context *ctx, prc_bit_state *bit_state,
    prc_compressed_point *data, double tolerance);
static int prc_parse_unique_vertex(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_unique_vertex *data, uint8_t read_tag);
static int prc_parse_base_topology(prc_context *ctx, prc_bit_state *bit_state,
    prc_base_topology *data);
static int prc_parse_content_body(prc_context *ctx, prc_bit_state *bit_state,
    prc_content_body *data);
#define PRC_TOPO_BODY_RECURSION_MAX 256
static int prc_parse_topo(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo *data, int depth);
static int prc_parse_surf(prc_context *ctx, prc_bit_state *bit_state,
    prc_type_surf *data);
void prc_release_compressed_curve(prc_context *ctx, prc_compressed_curve *data);

/* Table 24 � UVParameterization */
static void
prc_parse_uv_parameterization(prc_context *ctx, prc_bit_state *bit_state,
    prc_uv_parameterization *data)
{
    data->swap_uv = prc_bitread_bit(ctx, bit_state);
    data->surface_domain = prc_parse_domain(ctx, bit_state);

    data->u_param_coeff_a = prc_bitread_double(ctx, bit_state);
    data->v_param_coeff_a = prc_bitread_double(ctx, bit_state);
    data->u_param_coeff_b = prc_bitread_double(ctx, bit_state);
    data->v_param_coeff_b = prc_bitread_double(ctx, bit_state);
}

/* Table 239 CompressedVertex */
static int
prc_parse_compressed_vertex(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data,
    prc_compressed_vertex *data)
{
    int code;

    data->not_already_stored = prc_bitread_bit(ctx, bit_state);

    if (!data->not_already_stored)
    {
        /* Just get the index to the vertex */
        data->point_index = prc_bitread_uint_variable_bit(ctx, bit_state,
            compressed_data->number_bits_for_encoding);
    }
    else
    {
        /* We need to parse the compressed vertex */
        code = prc_parse_compressed_point(ctx, bit_state, &data->point_data,
                                          compressed_data->tolerance);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex_data\n");
            return code;
        }
        /* Store the vertex in the compressed data */
        compressed_data->vertices[compressed_data->current_vertex_index] = *data;
        compressed_data->current_vertex_index++;
        if (compressed_data->current_vertex_index >= compressed_data->vertices_capacity)
        {
            prc_compressed_vertex *new_vertices;
            /* Need to reallocate */
            compressed_data->vertices_capacity *= 2;
            new_vertices = (prc_compressed_vertex *)prc_realloc(ctx,
                compressed_data->vertices,
                compressed_data->vertices_capacity * sizeof(prc_compressed_vertex));
            if (new_vertices == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to reallocate compressed_data->vertices\n");
                return PRC_ERROR_MEMORY;
            }
            compressed_data->vertices = new_vertices;
        }
    }

    return 0;
}

/* Table 240 CompressedPoint */
static int
prc_parse_compressed_point(prc_context *ctx, prc_bit_state *bit_state,
    prc_compressed_point *data, double tolerance)
{
    int code;
    prc_topo_unique_vertex unique_vertex = {0};

    data->uNbBits = prc_bitread_uint_variable_bit(ctx, bit_state, 6);

    if (data->uNbBits > 30 || data->uNbBits == 0)
    {
        /* Search for PRC_TYPE_TOPO_UniqueVetex tag */
#if 0
        prc_debug_stream(ctx, bit_state);
        code = prc_parse_unique_vertex(ctx, bit_state, &unique_vertex, READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_unique_vertex\n");
            return code;
        }
        data->point.x = unique_vertex.vertex.x;
        data->point.y = unique_vertex.vertex.y;
        data->point.z = unique_vertex.vertex.z;
#endif
        /* Spec seems to be wrong about this. At least if we are coming from
           isonurbs and the uNbBits is 31 we just do 3 double reads it seems */

        if (data->uNbBits == 31)
        {
            data->point.x = prc_bitread_double(ctx, bit_state);
            data->point.y = prc_bitread_double(ctx, bit_state);
            data->point.z = prc_bitread_double(ctx, bit_state);
        }
        else
        {
            code = prc_parse_unique_vertex(ctx, bit_state, &unique_vertex, READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_unique_vertex\n");
                return code;
            }
            data->point.x = unique_vertex.vertex.x;
            data->point.y = unique_vertex.vertex.y;
            data->point.z = unique_vertex.vertex.z;
        }
    }
    else
    {
        data->point.x = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->uNbBits, tolerance);
        data->point.y = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->uNbBits, tolerance);
        data->point.z = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->uNbBits, tolerance);
    }
    return 0;
}

/* Table 239 StartEndData */
static int
prc_parse_start_end_data(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_start_end_data *data)
{
    int code;

    if (!compressed_data->compressed_iso_spline)
    {
        if (compressed_data->is_a_compressed_face)
        {
            data->is_vertex = 1;
            code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
                &data->start_vertex);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
                return code;
            }
            code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
                &data->end_vertex);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
                return code;
            }
        }
        else
        {
            data->is_vertex = 0;
            code = prc_parse_compressed_point(ctx, bit_state, &data->start_point,
                compressed_data->tolerance);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
                return code;
            }
            code = prc_parse_compressed_point(ctx, bit_state, &data->end_point,
                compressed_data->tolerance);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
                return code;
            }
        }
    }

    return 0;
}

/* Table 235 ParticularCircle */
static int
prc_parse_particular_circle(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_particular_circle *data)
{
    int code;

    data->full_circle = prc_bitread_bit(ctx, bit_state);

    if (!compressed_data->compressed_iso_spline)
    {
        code = prc_parse_start_end_data(ctx, bit_state, compressed_data,
            &data->start_end_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_start_end_data\n");
            return code;
        }
    }

    if (data->full_circle)
    {
        code = prc_parse_compressed_point(ctx, bit_state, &data->center,
            compressed_data->tolerance);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
            return code;
        }
        code = prc_parse_compressed_point(ctx, bit_state, &data->normal_plane,
            compressed_data->tolerance);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
            return code;
        }
    }
    else
    {
        code = prc_parse_compressed_point(ctx, bit_state, &data->middle_of_arc,
            compressed_data->tolerance);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
            return code;
        }
    }
    return 0;
}

/* Table 236 GeneralCircle */
static int
prc_parse_general_circle(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_general_circle *data)
{
    int code;

    if (!compressed_data->compressed_iso_spline)
    {
        code = prc_parse_start_end_data(ctx, bit_state, compressed_data,
            &data->start_end_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_start_end_data\n");
            return code;
        }
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->center,
        compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    data->circle_angle = prc_bitread_bit(ctx, bit_state);

    return 0;
}

/* Table 238 PRC_HCG_BsplineHermintCurve */
static int
prc_parse_hcg_bspline_hermite_curve(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_bspline_hermite_curve *data,
    uint8_t read_tag)
{
    int code;
    uint32_t k;
    uint32_t number_points;

    if (read_tag)
    {
        data->type = prc_bitread_uint32(ctx, bit_state);
        if (data->type != PRC_HCG_BsplineHermiteCurve)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_hcg_circle\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->type = PRC_HCG_BsplineHermiteCurve; /* Preread for abstract switch */
    }

    code = prc_parse_start_end_data(ctx, bit_state, compressed_data,
        &data->start_end_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_start_end_data\n");
        return code;
    }

    /* For storing number_point */
    data->number_bits = prc_bitread_uint_variable_bit(ctx, bit_state, 4);
    data->number_points = prc_bitread_uint_variable_bit(ctx, bit_state, data->number_bits);
    data->point_number_bits = prc_bitread_uint_variable_bit(ctx, bit_state, 6);
    number_points = data->number_points - 2;

    if (number_points > 0)
    {
        data->points = (prc_vec3*) prc_calloc(ctx, number_points,
            sizeof(prc_vec3));
        if (data->points == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate data->points\n");
            return PRC_ERROR_MEMORY;
        }

        if (data->point_number_bits > 30)
        {
            for (k = 0; k < number_points; k++)
            {
                data->points[k] = prc_parse_3d_vector(ctx, bit_state);
            }
        }
        else
        {
            for (k = 0; k < number_points; k++)
            {
                data->points[k].x = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->point_number_bits,
                                            compressed_data->tolerance);
                data->points[k].y = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->point_number_bits,
                                            compressed_data->tolerance);
                data->points[k].z = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->point_number_bits,
                    compressed_data->tolerance);
            }
        }
    }

    data->tangent_number_bits = prc_bitread_uint_variable_bit(ctx, bit_state, 6);
    number_points = data->number_points;
    if (number_points > 0)
    {
        data->tangents = (prc_vec3*)prc_calloc(ctx, number_points,
            sizeof(prc_vec3));
        if (data->tangents == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate data->tangents\n");
            return PRC_ERROR_MEMORY;
        }
        if (data->tangent_number_bits > 30)
        {
            for (k = 0; k < number_points; k++)
            {
                data->tangents[k] = prc_parse_3d_vector(ctx, bit_state);
            }
        }
        else
        {
            for (k = 0; k < number_points; k++)
            {
                data->tangents[k].x = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->tangent_number_bits,
                                            compressed_data->tolerance);
                data->tangents[k].y = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->tangent_number_bits,
                                            compressed_data->tolerance);
                data->tangents[k].z = prc_bitread_double_with_variable_bit_number(ctx,
                                            bit_state, data->tangent_number_bits,
                    compressed_data->tolerance);
            }
        }
    }
    return 0;
}

/* Table 234 PRC_HCG_Circle */
static int
prc_parse_hcg_circle(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_circle *data,
    uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->type = prc_bitread_uint32(ctx, bit_state);
        if (data->type != PRC_HCG_Circle)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_hcg_circle\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->type = PRC_HCG_Circle; /* Preread for abstract switch */
    }

    data->is_particular_circle = prc_bitread_bit(ctx, bit_state);

    if (data->is_particular_circle)
    {
        code = prc_parse_particular_circle(ctx, bit_state, compressed_data,
                                           &data->particular_circle);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_particular_circle\n");
            return code;
        }
    }
    else
    {
        code = prc_parse_general_circle(ctx, bit_state, compressed_data,
                                        &data->general_circle);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_general_circle\n");
            return code;
        }
    }
    return 0;
}

static int
prc_parse_hcg_line(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_line *data,
    uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->type = prc_bitread_uint32(ctx, bit_state);
        if (data->type != PRC_HCG_Line)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_hcg_line\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->type = PRC_HCG_Line;
    }

    code = prc_parse_start_end_data(ctx, bit_state, compressed_data,
                                    &data->start_end_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_start_end_data\n");
        return code;
    }
    return 0;
}

/* Section 7.9.21.9  Compressed Curve */
static int
prc_parse_compressed_curve(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_compressed_curve *data)
{
    uint32_t entity_type;
    uint8_t is_curve;
    int code;

    /* First get the entity type */
    code = prc_bitread_compressed_entity_type(ctx, bit_state, &is_curve, &entity_type);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_bitread_compressed_entity_type\n");
        return code;
    }

    if (!is_curve)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_curve\n");
        return code;
    }
    else
    {
        data->curve_type = entity_type; /* Set the type */
        switch (entity_type)
        {
        case PRC_HCG_Line:
            code = prc_parse_hcg_line(ctx, bit_state, compressed_data,
                                      &data->hcg_line, 0);
            break;

        case PRC_HCG_Circle:
            code = prc_parse_hcg_circle(ctx, bit_state, compressed_data,
                                        &data->hcg_circle, 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_circle\n");
                return code;
            }
            break;

        case PRC_HCG_BsplineHermiteCurve:
           code = prc_parse_hcg_bspline_hermite_curve(ctx, bit_state, compressed_data,
                &data->hcg_bspline_hermite_curve, 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_bspline_hermite_curve\n");
                return code;
            }
            break;

        case PRC_HCG_CompositeCurve:
          //  code = prc_parse_hcg_composite_curve(ctx, bit_state, compressed_data,
          //      &data->hcg_composite_curve);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_composite_curve\n");
                return code;
            }
            break;
        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_parse_compressed_curve\n", entity_type);
            return PRC_ERROR_PARSE;
        }
    }
    return 0;
}

/* Table 182 PRC_TYPE_TOPO_UniqueVertex */
static int
prc_parse_unique_vertex(prc_context *ctx, prc_bit_state *bit_state,
                        prc_topo_unique_vertex *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_UniqueVertex)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_unique_vertex\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_UniqueVertex;
    }

    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }
    data->vertex = prc_parse_3d_vector(ctx, bit_state);

    data->has_tolerance = prc_bitread_bit(ctx, bit_state);
    if (data->has_tolerance)
    {
        data->tolerance = prc_bitread_double(ctx, bit_state);
    }
    else
    {
        data->tolerance = 0.0; /* Default value */
    }

    return 0;
}

/* Table 232 RefOrCompressedCurve */
static int
prc_parse_ref_or_compressed_curve(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_ref_or_compressed_curve *data)
{
    int code;
    prc_compressed_curve *nano_data = &compressed_data->curves[compressed_data->current_curve_index];

    data->curve_is_not_already_stored = prc_bitread_bit(ctx, bit_state);
    if (data->curve_is_not_already_stored)
    {
        /* Then we need to parse the compressed curve. Also set the reference
           number to what we are using */
        data->index_compressed_curve = compressed_data->current_curve_index;
        code = prc_parse_compressed_curve(ctx, bit_state, compressed_data,
            nano_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_curve\n");
            return code;
        }
        data->compressed_curve = nano_data;
        compressed_data->current_curve_index++;

        if (compressed_data->current_curve_index >= compressed_data->curves_capacity)
        {
            prc_compressed_curve *new_curves;
            /* Need to reallocate */
            compressed_data->curves_capacity *= 2;
            new_curves = (prc_compressed_curve *)prc_realloc(ctx,
                compressed_data->curves,
                compressed_data->curves_capacity * sizeof(prc_compressed_curve));
            if (new_curves == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to reallocate compressed_data->curves\n");
                return PRC_ERROR_MEMORY;
            }
            compressed_data->curves = new_curves;
        }
    }
    else
    {
        /* Just get the index to the curve */
        data->index_compressed_curve = prc_bitread_uint_variable_bit(ctx,
                          bit_state, compressed_data->number_bits_for_encoding);
    }
    return 0;
}

/* Table 231 � AnaFaceTrimLoop */
static int
prc_parse_ana_face_trim_loop(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, uint32_t *trim_loop_count,
    uint32_t *trim_loop_capacity, prc_ana_face_trim_loop **data_ptr)
{
    int code;
    uint8_t done = 0;
    prc_ana_face_trim_loop *data = *data_ptr;
    prc_ana_face_trim_loop *current_loop = data;
    uint32_t curve_capacity;
    prc_ref_or_compressed_curve *current_curve;
    uint32_t curve_index;
    uint32_t entity_type;
    uint8_t is_curve;

    while (!done)
    {
        curve_capacity = CURVES_PERLOOP_INITIAL_SIZE;

        /* Initialize the curves array to its capacity */
        current_loop->trim_curves = (prc_ref_or_compressed_curve *)prc_calloc(ctx,
            curve_capacity, sizeof(prc_ref_or_compressed_curve));
        if (current_loop->trim_curves == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate current_loop->trim_curves\n");
            return PRC_ERROR_MEMORY;
        }

        /* Get this loops orientation */
        current_loop->loop_surface_orientation = prc_bitread_bit(ctx, bit_state);
        curve_index = 0;

        current_loop->curve_count = 0;

        /* Now loop through the curves. */
        do
        {
            current_curve = &current_loop->trim_curves[curve_index];
            current_curve->curve_is_not_already_stored = prc_bitread_bit(ctx, bit_state);

            if (current_curve->curve_is_not_already_stored)
            {
                /* We need to check the curve type. If it is PRC_HCG_NewLoop or
                   PRC_HCG_EndLoop then we are at the end of the curves and need
                   to start a new loop */
                code = prc_bitread_compressed_entity_type(ctx, bit_state, &is_curve,
                                                          &entity_type);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_bitread_compressed_entity_type\n");
                    return code;
                }

                if ((entity_type == PRC_HCG_NewLoop || entity_type == PRC_HCG_EndLoop) && !is_curve)
                {
                    /* We are done with this loop */
                    current_loop->type = entity_type;
                    break;
                }
                else if (!is_curve)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Expected curve in prc_parse_ana_face_trim_loop\n");
                    return PRC_ERROR_PARSE;
                }
                else
                {
                    current_loop->curve_count += 1;

                    /* Need to parse the compressed curve */
                    current_curve->compressed_curve = (prc_compressed_curve *)prc_calloc(ctx,
                        1, sizeof(prc_compressed_curve));
                    if (current_curve->compressed_curve == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate current_curve->compressed_curve\n");
                        return PRC_ERROR_MEMORY;
                    }

                    current_curve->compressed_curve->curve_type = entity_type;
                    switch (entity_type)
                    {
                    case PRC_HCG_Line:
                        code = prc_parse_hcg_line(ctx, bit_state, compressed_data,
                            &current_curve->compressed_curve->hcg_line, 0);
                        break;

                    case PRC_HCG_Circle:
                        code = prc_parse_hcg_circle(ctx, bit_state, compressed_data,
                            &current_curve->compressed_curve->hcg_circle, 0);
                        if (code < 0)
                        {
                            prc_error(ctx, code, "Failed in prc_parse_hcg_circle\n");
                            return code;
                        }
                        break;

                    case PRC_HCG_BsplineHermiteCurve:
                        code = prc_parse_hcg_bspline_hermite_curve(ctx, bit_state, compressed_data,
                                    &current_curve->compressed_curve->hcg_bspline_hermite_curve, 0);
                        if (code < 0)
                        {
                            prc_error(ctx, code, "Failed in prc_parse_hcg_bspline_hermite_curve\n");
                            return code;
                        }
                        break;

                    case PRC_HCG_CompositeCurve:
                        //  code = prc_parse_hcg_composite_curve(ctx, bit_state, compressed_data,
                        //      &current_curve->compressed_curve->hcg_composite_curve);
                        if (code < 0)
                        {
                            prc_error(ctx, code, "Failed in prc_parse_hcg_composite_curve\n");
                            return code;
                        }
                        break;
                    default:
                        prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_parse_compressed_curve\n", entity_type);
                        return PRC_ERROR_PARSE;
                    }
                }
            }
            else
            {
                current_loop->curve_count += 1;

                /* Just get the index to the curve */
                current_curve->index_compressed_curve = prc_bitread_uint_variable_bit(ctx,
                    bit_state, compressed_data->number_bits_for_encoding);
            }
            curve_index++;

            if (curve_index >= curve_capacity)
            {
                prc_ref_or_compressed_curve *new_trim_curves;
                /* Need to reallocate. This is not updating the output structure... */
                curve_capacity *= 2;
                new_trim_curves = (prc_ref_or_compressed_curve *)prc_realloc(ctx,
                    current_loop->trim_curves,
                    curve_capacity * sizeof(prc_ref_or_compressed_curve));
                if (new_trim_curves == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to reallocate current_loop->trim_curves\n");
                    return PRC_ERROR_MEMORY;
                }
                current_loop->trim_curves = new_trim_curves;
                /* Zero allocate the new area */
                memset(&current_loop->trim_curves[curve_index], 0,
                       (curve_capacity - curve_index) *
                    sizeof(prc_ref_or_compressed_curve));
            }
        }
        while (1);

        (*trim_loop_count)++;
        if (current_loop->type == PRC_HCG_NewLoop)
        {
            /* We will have another loop. Check that we have not reach capacity
               and deal with it if we have */

            if (*trim_loop_count >= *trim_loop_capacity)
            {
                prc_ana_face_trim_loop *new_data_ptr;
                /* Need to reallocate.   This is not updating the output structure... */
                *trim_loop_capacity *= 2;
                new_data_ptr = (prc_ana_face_trim_loop *)prc_realloc(ctx, *data_ptr,
                    (*trim_loop_capacity) * sizeof(prc_ana_face_trim_loop));
                if (new_data_ptr == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to reallocate data->trim_loop\n");
                    return PRC_ERROR_MEMORY;
                }
                *data_ptr = new_data_ptr;
                /* Zero allocate the new area */
                memset(&(*data_ptr)[*trim_loop_count], 0,
                       (*trim_loop_capacity - *trim_loop_count) *
                    sizeof(prc_ana_face_trim_loop));

                /* And set our pointer to the new location */
                data = *data_ptr;

                /* Set current loop to the new location */
                current_loop = &data[*trim_loop_count];
            }
            else
            {
                current_loop++;
            }
        }
        else if (current_loop->type == PRC_HCG_EndLoop)
        {
            done = 1;
        }
        else
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown type in prc_parse_ana_face_trim_loop\n");
            return PRC_ERROR_PARSE;
        }
    } /* End while on trim loops */

    return 0;
}

/* Table 230 ContentCompressedAnaFace */
static int
prc_parse_content_compressed_ana_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_content_compressed_ana_face *data)
{
    int code;

    uint32_t trim_loop_count = 0;
    uint32_t trim_loop_capacity = TRIM_LOOP_INITIAL_SIZE;

    data->is_trimmed = prc_bitread_bit(ctx, bit_state);

    if (data->is_trimmed)
    {
        /* This array is terrible as we can't preallocate for storage. */
        data->trim_loop = (prc_ana_face_trim_loop *)prc_calloc(ctx, TRIM_LOOP_INITIAL_SIZE,
            sizeof(prc_ana_face_trim_loop));
        if (data->trim_loop == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate data->trim_loop\n");
            return PRC_ERROR_MEMORY;
        }

        /* Have to send ref to data->trim_loop since me may need to realloc it */
        code = prc_parse_ana_face_trim_loop(ctx, bit_state, compressed_data,
            &trim_loop_count, &trim_loop_capacity, &data->trim_loop);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_ana_trim_loop\n");
            prc_free(ctx, data->trim_loop);
            return code;
        }

        /* TODO all_loops_are_vertex_loops needs to be handled */
        if (compressed_data->all_loops_are_vertex_loops &&
            compressed_data->surface_type_is_prc_hcg_ana_torus)
        {
            code = prc_parse_compressed_point(ctx, bit_state, &data->compressed_point,
                compressed_data->tolerance);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
                prc_free(ctx, data->trim_loop);
                return code;
            }
        }
    }
    data->trim_loop_count = trim_loop_count;
    return 0;
}

/* Table 229 ContentCompressedIsoFace. This has some serious flaws in the
   specification. */
static int
prc_parse_content_compressed_iso_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_content_compressed_iso_face *data)
{
    int code;

    data->orientation_loop_with_surface = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_ref_or_compressed_curve(ctx, bit_state, compressed_data,
                                             &data->first_trim_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_ref_or_compressed_curve\n");
        return code;
    }
    code = prc_parse_ref_or_compressed_curve(ctx, bit_state, compressed_data,
                                             &data->second_trim_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_ref_or_compressed_curve\n");
        return code;
    }

    /* If we are in an ISO case, then the third and fourth curves are implicit
       and only rely upon a common third fourth vertex for reconstruction.
       Need to understand if this is different if it is not an ISO curve */

    data->third_trim_curve_is_not_yet_saved = prc_bitread_bit(ctx, bit_state);
    if (!data->third_trim_curve_is_not_yet_saved)
    {
        data->third_trim_curve = prc_bitread_uint_variable_bit(ctx, bit_state,
            compressed_data->number_bits_for_encoding);
    }

    data->fourth_trim_curve_is_not_yet_saved = prc_bitread_bit(ctx, bit_state);
    if (!data->fourth_trim_curve_is_not_yet_saved)
    {
        data->fourth_trim_curve = prc_bitread_uint_variable_bit(ctx, bit_state,
            compressed_data->number_bits_for_encoding);

    }

    if (data->fourth_trim_curve_is_not_yet_saved &&
        data->third_trim_curve_is_not_yet_saved)
    {
        code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
            &data->common_third_fourth_vertex);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
            return code;
        }
    }

    return 0;
}

/* Table 228 ContentCompressedFace */
static int
prc_parse_content_compressed_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_content_compressed_face *data,
    uint8_t is_an_iso_face)
{
    int code;

    data->orientation_surface_with_shell = prc_bitread_bit(ctx, bit_state);
    data->is_an_iso_face = is_an_iso_face;

    if (data->is_an_iso_face)
    {
        code = prc_parse_content_compressed_iso_face(ctx, bit_state,
                                            compressed_data, &data->iso_face);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_compressed_iso_face\n");
            return code;
        }
    }
    else
    {
        code = prc_parse_content_compressed_ana_face(ctx, bit_state,
            compressed_data, &data->ana_face);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_compressed_ana_face\n");
            return code;
        }
    }
    return 0;
}

/* Table 204 PRC_HCG_IsoPlane */
static int
prc_parse_hcg_iso_plane(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_plane *data)
{
    int code;

    data->tag = PRC_HCG_IsoPlane; /* Preread for abstract switch */

    data->x = prc_bitread_double(ctx, bit_state);
    data->y = prc_bitread_double(ctx, bit_state);
    data->positive_z = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, true);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}

/* Table 205 PRC_HCG_IsoCylinder */
static int
prc_parse_hcg_iso_cylinder(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_cylinder *data)
{
    int code;

    data->tag = PRC_HCG_IsoCylinder; /* Preread for abstract switch */
    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, true);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}

/* Table 206 PRC_HCG_IsoTorus */
static int
prc_parse_hcg_iso_torus(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_torus *data)
{
    int code;

    data->tag = PRC_HCG_IsoTorus; /* Preread for abstract switch */
    data->is_major_radius = prc_bitread_bit(ctx, bit_state);
    code = prc_parse_content_compressed_face(ctx, bit_state,
                                             compressed_data, &data->face, true);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}


/* Table 207 PRC_HCG_Sphere */
static int
prc_parse_hcg_iso_sphere(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_sphere *data)
{
    int code;

    data->tag = PRC_HCG_IsoSphere; /* Preread for abstract switch */
    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, true);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}

/* Table 208 PRC_HCG_IsoCone */
static int
prc_parse_hcg_iso_cone(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_cone *data)
{
    int code;

    data->tag = PRC_HCG_IsoCone; /* Preread for abstract switch */
    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, true);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}

/* Table 217 � IsoNURBSTrimCrv */
static int
prc_parse_iso_nurbs_trim_crv(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data,
    prc_iso_nurbs_trim_crv *data)
{
    int code;

    data->iso_boundary = prc_bitread_bit(ctx, bit_state);

    if (!data->iso_boundary)
    {
        data->is_a_circle = prc_bitread_bit(ctx, bit_state);
        if (data->is_a_circle)
        {
            code = prc_parse_hcg_circle(ctx, bit_state, compressed_data,
                                        &data->compressed_circle, 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_circle\n");
                return code;
            }
        }
        else
        {
            code = prc_parse_hcg_line(ctx, bit_state, compressed_data,
                                             &data->compressed_line, 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_nurbs\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 228 � ContentCompressedFace */
static int
prc_parse_compressed_weights(prc_context *ctx, prc_bit_state *bit_state,
    uint32_t num_points, prc_compressed_weights *data)
{
    uint32_t i;

    data->number_bit_weight = prc_bitread_uint_variable_bit(ctx, bit_state, 6);

    if (data->number_bit_weight == 0)
        return 0;

    data->compressed_weights = (double *)prc_calloc(ctx, num_points, sizeof(double));
    if (!data->compressed_weights)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_weights\n");
        return PRC_ERROR_MEMORY;
    }

    if (data->number_bit_weight <= 30)
    {
        data->tolerance = prc_bitread_double(ctx, bit_state);
        for (i = 0; i < num_points; i++)
        {
            data->compressed_weights[i] = prc_bitread_double_with_variable_bit_number(ctx,
                                                bit_state, data->number_bit_weight + 1, 1.0);
        }
    }
    else
    {
        for (i = 0; i < num_points; i++)
        {
            data->compressed_weights[i] = prc_bitread_double(ctx, bit_state);
        }
    }
    return 0;
}

/* Table 225 CompressedKnots */
static int
prc_parse_compressed_knots(prc_context *ctx, prc_bit_state *bit_state,
    uint32_t num_knots, prc_compressed_knots *data)
{
    int code;
    uint32_t i;

    /* type_param can have one of the following values:
        0 for uniform parameterization (in which case we would not be here).
        1 for non-uniform parameterization (is_unknown_form is true).
        2 for pseudo-uniform parameterization, meaning that the parameterization
          is uniform except for extremities, mostly coming from a trim applied
          uniformly. (is_pseudo_uniform is true) Then I think we just encode
          two points */

    data->is_unknown_form = prc_bitread_bit(ctx, bit_state);
    if (!data->is_unknown_form)
    {
        data->is_pseudo_uniform = prc_bitread_bit(ctx, bit_state);

        if (data->is_pseudo_uniform)
        {
            num_knots = 2; /* Only two knots to read */
        }
    }

    data->number_bit_parameter = prc_bitread_uint_variable_bit(ctx, bit_state, 6);

    data->compressed_knots = (prc_compressed_knot *)prc_calloc(ctx, num_knots, sizeof(prc_compressed_knot));
    if (!data->compressed_knots)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_knots\n");
        return PRC_ERROR_MEMORY;
    }

    for (i = 0; i < num_knots; i++)
    {
        if (data->number_bit_parameter > 30)
        {
            data->compressed_knots[i].knot = prc_bitread_double(ctx, bit_state);
        }
        else
        {
            data->compressed_knots[i].knot = prc_bitread_double_with_variable_bit_number(ctx,
                                                bit_state, data->number_bit_parameter + 1, 0.0);
        }
    }

    return 0;
}

/* Table 224 CompressedKnotVector */
/* This structure has serious issues.  For now, assume that it is a single bit
   which is the is_uniform setting */
static int
prc_parse_compressed_knot_vector(prc_context *ctx, prc_bit_state *bit_state,
    uint32_t num_knots, prc_compressed_knot_vector *data)
{
    int code;

    data->is_uniform = prc_bitread_bit(ctx, bit_state);

    if (!data->is_uniform)
    {
        code = prc_parse_compressed_knots(ctx, bit_state, num_knots , &data->knots);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_knots\n");
            return code;
        }
    }
    return 0;
}

/* Table 221 � CompressedMultiplicities */
static int
prc_parse_compressed_multiplicities(prc_context *ctx, prc_bit_state *bit_state,
    uint32_t num_bits, uint32_t index, prc_compressed_multiplicities *data)
{
    prc_compressed_multiplicities *curr_data = &data[index];
    curr_data->multiplicity_is_stored = prc_bitread_bit(ctx, bit_state);

    /* Note that this naming of this variable is wrong in the spec. This should
       be multiplicity_is_not_stored */
    if (!curr_data->multiplicity_is_stored)
    {
        curr_data->multiplicity = prc_bitread_uint_variable_bit(ctx, bit_state, num_bits);
    }
    else
    {
        if (index == 0)
        {
            curr_data->multiplicity = 1; /* First one must be one */
        }
        else
        {
            curr_data->multiplicity = data[index - 1].multiplicity;
        }
    }
    return 0;
}

/* Table 223 � InteriorCompressedControlPoints */
static int
prc_parse_interior_compressed_control_points(prc_context *ctx, prc_bit_state *bit_state,
    uint32_t num_bits, prc_interior_compressed_control_points *data)
{
    data->type = prc_bitread_uint_variable_bit(ctx, bit_state, 2);

    if (data->type == 1)
    {
        data->Pijx = 0;
        data->Pijy = 0;
        data->Pijz = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
    }
    else if (data->type == 2)
    {
        data->Pijx = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
        data->Pijy = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
        data->Pijz = 0;
    }
    else if (data->type == 3)
    {
        data->Pijx = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
        data->Pijy = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
        data->Pijz = prc_bitread_int_variable_bit(ctx, bit_state, num_bits);
    }
    else if (data->type == 0)
    {
        /* No data stored as it must have been less than the nurbs_tolerance */
        data->Pijx = 0;
        data->Pijy = 0;
        data->Pijz = 0;
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_interior_compressed_control_points\n");
        return PRC_ERROR_PARSE;
    }

    return 0;
}

/* Table 222 � CompressedControlPoints */
static int
prc_parse_compressed_control_points(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, uint32_t num_bits_for_isomin,
    uint32_t num_bits_for_rest, uint32_t num_control_points_u,
    uint32_t num_control_points_v, prc_compressed_control_points *data)
{
    /* num_control_points_u/v are derived from file-controlled knot
       multiplicities and can be 0 (making num_control_points_u - 1 wrap to
       UINT32_MAX) or large enough that (num_control_points_u - 1) *
       (num_control_points_v - 1), computed in 32-bit arithmetic, overflows
       and wraps to a small value before ever reaching prc_calloc -- which
       would then allocate a small buffer while the nested loop below still
       writes the full, un-wrapped number of interior points (heap OOB
       write). Reject invalid counts and do the multiplication in 64-bit. */
    if (num_control_points_u == 0 || num_control_points_v == 0 ||
        (uint64_t)(num_control_points_u - 1) * (uint64_t)(num_control_points_v - 1) >
            PRC_MAX_NURBS_CONTROL_POINTS)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Invalid NURBS control point counts\n");
        return PRC_ERROR_PARSE;
    }

    /* Here is p[0][0] */
    data->p00 = prc_parse_3d_vector(ctx, bit_state);

    /* First get the compressed control v points [0][j] */
    data->ccpt_in_v = (prc_vec3 *)prc_calloc(ctx, num_control_points_v - 1, sizeof(prc_vec3));
    if (!data->ccpt_in_v)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_control_points\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t j = 0; j < num_control_points_v - 1; j++)
    {
        data->ccpt_in_v[j].x = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
        data->ccpt_in_v[j].y = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
        data->ccpt_in_v[j].z = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
    }

    /* Now get the compressed control u points [i][0] */
    data->ccpt_in_u = (prc_vec3 *)prc_calloc(ctx, num_control_points_u - 1, sizeof(prc_vec3));
    if (!data->ccpt_in_u)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_control_points\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t j = 0; j < num_control_points_u - 1; j++)
    {
        data->ccpt_in_u[j].x = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
        data->ccpt_in_u[j].y = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
        data->ccpt_in_u[j].z = prc_bitread_int_variable_bit(ctx, bit_state,
            num_bits_for_isomin);
    }

    /* Now the interior points */
    data->ccpt_interior = (prc_interior_compressed_control_points *)prc_calloc(
        ctx, (num_control_points_u - 1) * (num_control_points_v - 1),
        sizeof(prc_interior_compressed_control_points));
    if (!data->ccpt_interior)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_control_points\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t i = 0; i < num_control_points_u - 1; i++)
    {
        for (uint32_t j = 0; j < num_control_points_v - 1; j++)
        {
            uint32_t index = i * (num_control_points_v - 1) + j;
            int code = prc_parse_interior_compressed_control_points(ctx, bit_state,
                num_bits_for_rest, &data->ccpt_interior[index]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_interior_compressed_control_points\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 220 � CompressedNURBS */
static int
prc_parse_compressed_nurbs(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data,
    prc_compressed_nurbs *data)

{
    int code;
    uint32_t num_bits_u, num_bits_v;
    uint32_t number_knots_in_u, number_knots_in_v;
    uint32_t number_control_points_u, number_control_points_v;
    double nurbs_tolerance = compressed_data->tolerance / 5.0;
    double v_tolerance, u_tolerance;
    uint32_t i;
    uint32_t sum_multiplicities_u = 0;
    uint32_t sum_multiplicities_v = 0;

    data->degree_in_u = prc_bitread_uint_variable_bit(ctx, bit_state, 5);
    data->degree_in_v = prc_bitread_uint_variable_bit(ctx, bit_state, 5);

    num_bits_u = (data->degree_in_u ? ceil(log(data->degree_in_u + 2) / log(2)) : 2);
    num_bits_v = (data->degree_in_v ? ceil(log(data->degree_in_v + 2) / log(2)) : 2);

    v_tolerance = 1.0 / pow(2, num_bits_v - 1);
    u_tolerance = 1.0 / pow(2, num_bits_u - 1);

    data->number_stored_knots_in_u = prc_bitread_uint_variable_bit(ctx, bit_state, 16);
    if (data->number_stored_knots_in_u > 0)
    {
        data->mult_u = (prc_compressed_multiplicities *)prc_calloc(ctx, data->number_stored_knots_in_u,
            sizeof(prc_compressed_multiplicities));
        if (!data->mult_u)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_nurbs\n");
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < data->number_stored_knots_in_u; i++)
        {
            code = prc_parse_compressed_multiplicities(ctx, bit_state, num_bits_u,
                i, data->mult_u);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_multiplicities\n");
                return code;
            }
            sum_multiplicities_u += data->mult_u[i].multiplicity;
        }
    }

    data->number_stored_knots_in_v = prc_bitread_uint_variable_bit(ctx, bit_state, 16);
    if (data->number_stored_knots_in_v > 0)
    {
        data->mult_v = (prc_compressed_multiplicities *)prc_calloc(ctx, data->number_stored_knots_in_v,
            sizeof(prc_compressed_multiplicities));
        if (!data->mult_v)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_compressed_nurbs\n");
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < data->number_stored_knots_in_v; i++)
        {
            code = prc_parse_compressed_multiplicities(ctx, bit_state, num_bits_v,
                i, data->mult_v);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_multiplicities\n");
                return code;
            }
            sum_multiplicities_v += data->mult_v[i].multiplicity;
        }
    }

    data->is_closed_in_u = prc_bitread_bit(ctx, bit_state);
    data->is_closed_in_v = prc_bitread_bit(ctx, bit_state);
    data->number_bits_for_isomin = prc_bitread_uint_variable_bit(ctx, bit_state, 20);
    data->number_bits_for_rest = prc_bitread_uint_variable_bit(ctx, bit_state, 20);

    /* From PDF Association issues */
    number_control_points_u = sum_multiplicities_u - data->degree_in_u - 1;
    number_control_points_v = sum_multiplicities_v - data->degree_in_v - 1;

    code = prc_parse_compressed_control_points(ctx, bit_state, compressed_data,
        data->number_bits_for_isomin + 1, data->number_bits_for_rest + 1,
        number_control_points_u, number_control_points_v,
        &data->compressed_control_points);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_control_points\n");
        return code;
    }

    /* Confusing how described in text. Arrived at by trial and error */
    number_knots_in_u = data->number_stored_knots_in_u - 2;
    number_knots_in_v = data->number_stored_knots_in_v - 2;

    code = prc_parse_compressed_knot_vector(ctx, bit_state, number_knots_in_u,
        &data->knot_vector_u);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_knot_vector\n");
        return code;
    }

    code = prc_parse_compressed_knot_vector(ctx, bit_state, number_knots_in_v,
        &data->knot_vector_v);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_knot_vector\n");
        return code;
    }

    data->is_rational = prc_bitread_bit(ctx, bit_state);

    /* Need to encounter still */
    if (data->is_rational)
    {
        uint32_t num_points = number_control_points_u * number_control_points_v;
        code = prc_parse_compressed_weights(ctx, bit_state, num_points,
                                            &data->compressed_weights);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_weights\n");
            return code;
        }
    }
    return 0;
}

/* Table 216 � IsoNURBSTrimCurve */
static int
prc_parse_iso_nurbs_trim_curve(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data,
    prc_iso_nurbs_trim_curve *data)
{
    int code;

    data->is_referenced = prc_bitread_bit(ctx, bit_state);
    if (!data->is_referenced)
    {
        code = prc_parse_iso_nurbs_trim_crv(ctx, bit_state, compressed_data,
                                          &data->trim_curve);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_curve\n");
            return code;
        }
    }
    else
    {   data->trim_curve_index = prc_bitread_uint_variable_bit(ctx,
                        bit_state, compressed_data->number_bits_for_encoding);
    }
    return 0;
}

/* Table 215 � PRC_HCG_IsoNURBS */
static int
prc_parse_hcg_iso_nurbs(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_iso_nurbs *data)
{
    int code;

    data->tag = PRC_HCG_IsoNURBS; /* Preread for abstract switch */
    data->orientation_surface_with_shell = prc_bitread_bit(ctx, bit_state);
    data->orientation_loop_with_surface = prc_bitread_bit(ctx, bit_state);

    for (uint32_t i = 0; i < 3; i++)
    {
        data->sense_array[i] = prc_bitread_bit(ctx, bit_state);
    }

    code = prc_parse_compressed_nurbs(ctx, bit_state, compressed_data,
                                      &data->surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_nurbs\n");
        return code;
    }

    for (uint32_t i = 0; i < 4; i++)
    {
        code = prc_parse_iso_nurbs_trim_curve(ctx, bit_state, compressed_data,
                                              &data->curves[i]);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_ref_or_compressed_curve\n");
            return code;
        }
    }

    if (!data->curves[0].is_referenced && !data->curves[3].is_referenced)
    {
        code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
                                           &data->loop_vertex3);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
            return code;
        }
    }

    if (!data->curves[0].is_referenced && !data->curves[1].is_referenced)
    {
        code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
            &data->loop_vertex0);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
            return code;
        }
    }

    if (!data->curves[1].is_referenced && !data->curves[2].is_referenced)
    {
        code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
            &data->loop_vertex1);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
            return code;
        }
    }

    if (!data->curves[2].is_referenced && !data->curves[3].is_referenced)
    {
        code = prc_parse_compressed_vertex(ctx, bit_state, compressed_data,
            &data->loop_vertex2);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_vertex\n");
            return code;
        }
    }

    return 0;
}

/* Table 209 PRC_HCG_AnaPlane */
static int
prc_parse_hcg_ana_plane(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_plane *data)
{
    int code;

    data->tag = PRC_HCG_AnaPlane; /* Preread for abstract switch */

    data->x = prc_bitread_double(ctx, bit_state);
    data->y = prc_bitread_double(ctx, bit_state);
    data->positive_z = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }
    return 0;
}

/* Table 210 PRC_HCG_AnaCylinder */
static int
prc_parse_hcg_ana_cylinder(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_cylinder *data)
{
    int code;

    data->tag = PRC_HCG_AnaCylinder; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->point,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->direction,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    return 0;
}

/* Table 211 PRC_HCG_AnaTorus */
static int
prc_parse_hcg_ana_torus(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_torus *data)
{
    int code;

    data->tag = PRC_HCG_AnaTorus; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->center,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->x_axis,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    data->y_axis = prc_parse_3d_vector(ctx, bit_state);

    return 0;
}

/* Table 212 PRC_HCG_AnaSphere */
static int
prc_parse_hcg_ana_sphere(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_sphere *data)
{
    int code;

    data->tag = PRC_HCG_AnaSphere; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->sphere_center,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    return 0;
}

/* Table 213 PRC_HCG_AnaCone */
static int
prc_parse_hcg_ana_cone(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_cone *data)
{
    int code;

    data->tag = PRC_HCG_AnaCone; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->axis_point,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }

    code = prc_parse_compressed_point(ctx, bit_state, &data->apex_point,
                                      compressed_data->tolerance);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_point\n");
        return code;
    }
    return 0;
}

/* Table 214 PRC_HCG_AnaGenericFace */
static int
prc_parse_hcg_ana_generic_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_generic_face *data)
{
    int code;

    data->tag = PRC_HCG_AnaGenericFace; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_surf(ctx, bit_state, &data->surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_surf for surface\n");
        return code;
    }
    return 0;
}

/* Table 218 � PRC_HCG_AnaNURBS */
static int
prc_parse_hcg_ana_nurbs(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_hcg_ana_nurbs *data)
{
    int code;

    data->tag = PRC_HCG_AnaNURBS; /* Preread for abstract switch */

    code = prc_parse_content_compressed_face(ctx, bit_state,
        compressed_data, &data->face, false);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_hcg_iso_surface\n");
        return code;
    }

    code = prc_parse_compressed_nurbs(ctx, bit_state,
                                compressed_data, &data->compressed_surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_nurbs\n");
        return code;
    }
    return 0;
}

/* Section 7.9.21.5  Compressed Face */
static int
prc_parse_compressed_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_compressed_face *data)
{
    uint32_t entity_type;
    uint8_t is_curve;
    int code;

    compressed_data->compressed_iso_spline = false;
    compressed_data->is_iso_type = true;

    /* First get the entity type */
    code = prc_bitread_compressed_entity_type(ctx, bit_state, &is_curve, &entity_type);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_bitread_compressed_entity_type\n");
        return code;
    }

    if (!is_curve)
    {
        data->tag = entity_type;

        switch (entity_type)
        {
        case PRC_HCG_NewLoop:
            //code = prc_parse_hcg_newloop(ctx, bit_state, &data->)
            prc_error(ctx, PRC_ERROR_NOT_IMPLEMENTED,
                "PRC_HCG_NewLoop not implemented yet\n");
        break;

        case PRC_HCG_EndLoop:
            //code = prc_parse_hcg_endloop(ctx, bit_state, &data->)
            prc_error(ctx, PRC_ERROR_NOT_IMPLEMENTED,
                "PRC_HCG_EndLoop not implemented yet\n");
        break;

        case PRC_HCG_IsoPlane:
            code = prc_parse_hcg_iso_plane(ctx, bit_state, compressed_data,
                &data->hcg_iso_plane);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_plane\n");
                return code;
            }
        break;

        case PRC_HCG_IsoCylinder:
            code = prc_parse_hcg_iso_cylinder(ctx, bit_state, compressed_data,
                &data->hcg_iso_cylinder);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_cylinder\n");
                return code;
            }
        break;

        case PRC_HCG_IsoTorus:
            code = prc_parse_hcg_iso_torus(ctx, bit_state, compressed_data,
                                           &data->hcg_iso_torus);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_torus\n");
                return code;
            }
        break;

        case PRC_HCG_IsoSphere:
            code = prc_parse_hcg_iso_sphere(ctx, bit_state, compressed_data,
                &data->hcg_iso_sphere);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_sphere\n");
                return code;
            }
        break;

        case PRC_HCG_IsoCone:
            code = prc_parse_hcg_iso_cone(ctx, bit_state, compressed_data,
                &data->hcg_iso_cone);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_cone\n");
                return code;
            }
        break;

        case PRC_HCG_IsoNURBS:
            compressed_data->compressed_iso_spline = true;
            code = prc_parse_hcg_iso_nurbs(ctx, bit_state,
                compressed_data, &data->hcg_iso_nurbs);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_iso_nurbs\n");
                return code;
            }
        break;

        case PRC_HCG_AnaPlane:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_plane(ctx, bit_state,
                compressed_data, &data->hcg_ana_plane);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_plane\n");
                return code;
            }
        break;

        case PRC_HCG_AnaCylinder:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_cylinder(ctx, bit_state,
                compressed_data, &data->hcg_ana_cylinder);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_cylinder\n");
                return code;
            }
        break;

        case PRC_HCG_AnaTorus:
            compressed_data->is_iso_type = false;
            compressed_data->surface_type_is_prc_hcg_ana_torus = true;
            code = prc_parse_hcg_ana_torus(ctx, bit_state,
                compressed_data, &data->hcg_ana_torus);
            compressed_data->surface_type_is_prc_hcg_ana_torus = false;
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_torus\n");
                return code;
            }
        break;

        case PRC_HCG_AnaSphere:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_sphere(ctx, bit_state,
                compressed_data, &data->hcg_ana_sphere);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_sphere\n");
                return code;
            }
        break;

        case PRC_HCG_AnaCone:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_cone(ctx, bit_state,
                compressed_data, &data->hcg_ana_cone);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_cone\n");
                return code;
            }
        break;

        case PRC_HCG_AnaNURBS:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_nurbs(ctx, bit_state,
                compressed_data, &data->hcg_ana_nurbs);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_nurbs\n");
                return code;
            }
        break;

        case PRC_HCG_AnaGenericFace:
            compressed_data->is_iso_type = false;
            code = prc_parse_hcg_ana_generic_face(ctx, bit_state,
                compressed_data, &data->hcg_ana_generic_face);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_hcg_ana_generic_face\n");
                return code;
            }
        break;
        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type in prc_parse_compressed_face: %u\n", entity_type);
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        prc_error(ctx, code, "Failed in prc_parse_compressed_face\n");
        return code;
    }

    return 0;
}

/* Table 201 */
static int
prc_parse_compressed_shell(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_compressed_shell *data)
{
    int code;
    uint32_t k;

    data->single_face = prc_bitread_bit(ctx, bit_state);
    if (!data->single_face)
    {
        data->number_of_faces = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
    }
    else
    {
        data->number_of_faces = 1;
    }

    if (data->number_of_faces > 0)
    {
        data->faces = (prc_compressed_face *)prc_calloc(ctx, data->number_of_faces, sizeof(prc_compressed_face));
        if (data->faces == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_compressed_shell\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_faces; k++)
        {
            code = prc_parse_compressed_face(ctx, bit_state,
                compressed_data, &data->faces[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_face\n");
                return code;
            }
        }

        data->is_iso_face = (uint8_t *)prc_calloc(ctx, data->number_of_faces, sizeof(uint8_t));
        if (data->is_iso_face == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_compressed_shell\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_faces; k++)
        {
            data->is_iso_face[k] = prc_bitread_bit(ctx, bit_state);
        }
    }
    return 0;
}

/* Table 244 PtrTopology */
static int
prc_parse_ptr_topology(prc_context *ctx, prc_bit_state *bit_state, prc_ptr_topology *data)
{
    int code;

    data->is_stored = prc_bitread_bit(ctx, bit_state);

    if (!data->is_stored)
    {
        data->topo = (prc_topo *)prc_calloc(ctx, 1, sizeof(prc_topo));
        if (data->topo == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_topology\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_topo(ctx, bit_state, data->topo, 0);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_topo\n");
            return code;
        }
    }
    else
    {
        data->topo_identifier = prc_bitread_uint32(ctx, bit_state);
    }
    return 0;
}

/* Table 246 ContentCurve */
static int
prc_parse_content_curve(prc_context *ctx, prc_bit_state *bit_state,
                        prc_content_curve *data)
{
    int code;

    data->has_base_geometry = prc_bitread_bit(ctx, bit_state);
    if (data->has_base_geometry)
    {
        code = prc_parse_attribute_data(ctx, bit_state, &data->attribute_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_attribute_data\n");
            return code;
        }

        code = prc_parse_name(ctx, bit_state, &data->name);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_name\n");
            return code;
        }

        data->id = prc_bitread_uint32(ctx, bit_state);
    }

    data->extension_type = prc_bitread_uint32(ctx, bit_state);
    data->is_3d_flag = prc_bitread_bit(ctx, bit_state);

    return 0;
}

/* Table 248 PRC_TYPE_CRV_Blend02Boundary */
static int
prc_parse_crv_blend02_boundary(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_blend02_boundary *data, uint8_t read_tag)
{
    int code;
    uint32_t i;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Blend02Boundary)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_blend02_boundary\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Blend02Boundary;
    }

    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));

    data->parameterization = prc_parse_parameterization(ctx, bit_state);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface\n");
        return code;
    }

    data->bound = prc_bitread_int32(ctx, bit_state);

    data->number_of_crossing_points = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_crossing_points > 0)
    {
        data->crossing_points = (prc_vec3 *)prc_calloc(ctx, data->number_of_crossing_points,
            sizeof(prc_vec3));
        if (!data->crossing_points)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_crv_blend02_boundary\n");
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < data->number_of_crossing_points; i++)
        {
            data->crossing_points[i].x = prc_bitread_double(ctx, bit_state);
            data->crossing_points[i].y = prc_bitread_double(ctx, bit_state);
            data->crossing_points[i].z = prc_bitread_double(ctx, bit_state);
        }
    }

    data->chord_error = prc_bitread_double(ctx, bit_state);
    data->angle_error = prc_bitread_double(ctx, bit_state);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->bound_surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_prt_surface\n");
        return code;
    }

    data->sense_bound_surface = prc_bitread_bit(ctx, bit_state);
    data->intersection_order = prc_bitread_bit(ctx, bit_state);
    data->sense_intersection_order = prc_bitread_bit(ctx, bit_state);
    data->base_parameter = prc_bitread_double(ctx, bit_state);
    data->base_scale = prc_bitread_double(ctx, bit_state);

    data->start_limit_point = prc_parse_3d_vector(ctx, bit_state);
    data->start_limit_type = prc_bitread_uint32(ctx, bit_state);
    data->end_limit_point = prc_parse_3d_vector(ctx, bit_state);
    data->end_limit_type = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

static int
prc_parse_control_points_nurbs_crv(prc_context *ctx, prc_bit_state *bit_state,
    prc_control_points_nurbs_crv *data, uint8_t is_rational, uint8_t is_3d)
{
    data->x = prc_bitread_double(ctx, bit_state);
    data->y = prc_bitread_double(ctx, bit_state);

    if (is_3d)
    {
        data->z = prc_bitread_double(ctx, bit_state);
    }

    if (is_rational)
    {
        data->w = prc_bitread_double(ctx, bit_state);
    }

    return 0;
}

/* Table 249 PRC_TYPE_CRV_NURBS */
static int
prc_parse_crv_nurbs(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_nurbs *data, uint8_t read_tag)
{
    int code;
    uint32_t np, nu, i;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_NURBS)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_nurbs\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_NURBS;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->is_rational = prc_bitread_bit(ctx, bit_state);
    data->d = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_control_points = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_knots = prc_bitread_uint32(ctx, bit_state);

    np = data->highest_index_of_control_points + 1;
    nu = data->highest_index_of_knots + 1;

    data->p = (prc_control_points_nurbs_crv *)prc_calloc(ctx, np, sizeof(prc_control_points_nurbs_crv));
    if (!data->p)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_crv_nurbs\n");
        return PRC_ERROR_MEMORY;
    }
    for (i = 0; i < np; i++)
    {
        code = prc_parse_control_points_nurbs_crv(ctx, bit_state, &data->p[i],
            data->is_rational, data->curve_data.is_3d_flag);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_control_points_nurbs_crv\n");
            return code;
        }
    }

    data->u = (double *)prc_calloc(ctx, nu, sizeof(double));
    if (!data->u)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_crv_nurbs\n");
        return PRC_ERROR_MEMORY;
    }
    for (i = 0; i < nu; i++)
    {
        data->u[i] = prc_bitread_double(ctx, bit_state);
    }

    data->knot_type = prc_bitread_uint32(ctx, bit_state);
    data->curve_form = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 255 PRC_TYPE_CRV_Circle */
static int
prc_parse_crv_circle(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_circle *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Circle)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_circle\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Circle;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));

    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->radius = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 257 PRC_TYPE_CRV_Composite */
static int
prc_parse_crv_composite(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_composite *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Composite)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_composite\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Composite;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    return 0;
}

/* Table 260 PRC_TYPE_CRV_OnSurf */
static int
prc_parse_crv_onsurf(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_onsurf *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_OnSurf)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_onsurf\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_OnSurf;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    return 0;
}

/* Table 262 PRC_TYPE_CRV_Ellipse */
static int
prc_parse_crv_ellipse(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_ellipse *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Ellipse)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_ellipse\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Ellipse;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));

    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->rx = prc_bitread_double(ctx, bit_state);
    data->ry = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 264 PRC_TYPE_CRV_Equation */
static int
prc_parse_crv_equation(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_equation *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Equation)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_equation\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Equation;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    return 0;
}

/* Table 268 Type0HelixData */
static void
prc_parse_crv_type0_helix(prc_context *ctx, prc_bit_state *bit_state, prc_type0_helix_data *data)
{
    data->origin_0 = prc_bitread_double(ctx, bit_state);
    data->direction_0 = prc_bitread_double(ctx, bit_state);
    data->origin_1 = prc_bitread_double(ctx, bit_state);
    data->direction_1 = prc_bitread_double(ctx, bit_state);
    data->origin_2 = prc_bitread_double(ctx, bit_state);
    data->direction_2 = prc_bitread_double(ctx, bit_state);
    data->pitch = prc_bitread_double(ctx, bit_state);
    data->radius = prc_bitread_double(ctx, bit_state);
}

/* Table 269 TypeaHelixData */
static int
prc_parse_crv_type1_helix(prc_context *ctx, prc_bit_state *bit_state, prc_type1_helix_data *data)
{
    int code;

    data->unit_z0 = prc_bitread_double(ctx, bit_state);
    data->unit_u0 = prc_bitread_double(ctx, bit_state);
    data->unit_z1 = prc_bitread_double(ctx, bit_state);
    data->unit_u1 = prc_bitread_double(ctx, bit_state);
    data->unit_z2 = prc_bitread_double(ctx, bit_state);
    data->unit_u2 = prc_bitread_double(ctx, bit_state);
    data->reserved_double_0 = prc_bitread_double(ctx, bit_state);
    data->reserved_double_1 = prc_bitread_double(ctx, bit_state);
    data->reserved_double_2 = prc_bitread_double(ctx, bit_state);
    data->reserved_double_3 = prc_bitread_double(ctx, bit_state);

    code = prc_parse_math_fct_1d(ctx, bit_state, &data->radius_law);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_math_fct_1d\n");
        return code;
    }

    code = prc_parse_math_fct_1d(ctx, bit_state, &data->z_law);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_math_fct_1d\n");
        return code;
    }

    code = prc_parse_math_fct_1d(ctx, bit_state, &data->theta_law);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_math_fct_1d\n");
        return code;
    }
    return 0;
}

/* Table 266 PRC_TYPE_CRV_Helix01 */
static int
prc_parse_crv_helix01(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_helix01 *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Helix01)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_helix01\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Helix01;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }
    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));

    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->type = prc_bitread_uint8(ctx, bit_state);
    data->orientation = prc_bitread_bit(ctx, bit_state);
    data->start = prc_parse_3d_vector(ctx, bit_state);

    if (data->type == 0)
    {
        prc_parse_crv_type0_helix(ctx, bit_state, &data->type0_helix);
    }
    else
    {
        code = prc_parse_crv_type1_helix(ctx, bit_state, &data->type1_helix);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_crv_type1_helix\n");
            return code;
        }
    }

    return 0;
}

/* Table 269 PRC_TYPE_CRV_Hyperbola */
static int
prc_parse_crv_hyperbola(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_hyperbola *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Hyperbola)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_hyperbola\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Hyperbola;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));

    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->semi_axis_image = prc_bitread_double(ctx, bit_state); /* Different than spec */
    data->semi_axis = prc_bitread_double(ctx, bit_state);
    data->type = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 274  CrossingPointsCrvIntersection */
static void
prc_parse_crossing_points_intersection(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_crossing_points_intersection *data)
{
    data->position = prc_parse_3d_vector(ctx, bit_state);
    data->uv_surface_1 = prc_parse_2d_vector(ctx, bit_state);
    data->uv_surface_2 = prc_parse_2d_vector(ctx, bit_state);
    data->tangent = prc_parse_3d_vector(ctx, bit_state);
    data->parameter = prc_bitread_double(ctx, bit_state);
    data->scale = prc_bitread_double(ctx, bit_state);
    data->flags = prc_bitread_uint8(ctx, bit_state);
}

/* Table 272 PRC_TYPE_CRV_Intersection */
static int
prc_parse_crv_intersection(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_intersection *data, uint8_t read_tag)
{
    int code;
    uint32_t i;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Intersection)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_intersection\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Intersection;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    data->parameterization = prc_parse_parameterization(ctx, bit_state);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->surface1);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface\n");
        return code;
    }
    code = prc_parse_ptr_surface(ctx, bit_state, &data->surface2);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface\n");
        return code;
    }

    data->sense_1 = prc_bitread_bit(ctx, bit_state);
    data->sense_2 = prc_bitread_bit(ctx, bit_state);
    data->sense_cross = prc_bitread_bit(ctx, bit_state);

    data->number_of_crossings = prc_bitread_uint32(ctx, bit_state);

    data->crossing_points = (prc_crv_crossing_points_intersection*)prc_calloc(ctx,
        data->number_of_crossings, sizeof(prc_crv_crossing_points_intersection));
    if (!data->crossing_points)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Memory allocation failed in prc_parse_crv_intersection\n");
        return PRC_ERROR_MEMORY;
    }
    for (i = 0; i < data->number_of_crossings; i++)
    {
        prc_parse_crossing_points_intersection(ctx, bit_state, &data->crossing_points[i]);
    }

    data->start_limit = prc_parse_3d_vector(ctx, bit_state);
    data->start_limit_type = prc_bitread_uint32(ctx, bit_state);

    data->end_limit = prc_parse_3d_vector(ctx, bit_state);
    data->end_limit_type = prc_bitread_uint32(ctx, bit_state);

    data->chord_error = prc_bitread_double(ctx, bit_state);
    data->angle_error = prc_bitread_double(ctx, bit_state);

    data->param_respected = prc_bitread_bit(ctx, bit_state);

    return 0;
}

/* Table 275 PRC_TYPE_CRV_Line */
static int
prc_parse_crv_line(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_line *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Line)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_line\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Line;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    data->parameterization = prc_parse_parameterization(ctx, bit_state);

    return 0;
}

/* Table 277 PRC_TYPE_CRV_Offset */
static int
prc_parse_crv_offset(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_offset *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Offset)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_offset\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Offset;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    code = prc_parse_ptr_curve(ctx, bit_state, &data->base_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for base_curve\n");
        return code;
    }

    data->offset_plane_normal = prc_parse_3d_vector(ctx, bit_state);
    data->offset = prc_bitread_double(ctx, bit_state);
    data->base_params = NULL;
    data->base_func = NULL;
    return 0;
}

/* Table 279 PRC_TYPE_CRV_Parabola */
static int
prc_parse_crv_parabola(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_parabola *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Parabola)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_parabola\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Parabola;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->focal_length = prc_bitread_double(ctx, bit_state);
    data->type = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 282 PolyLinePoint */
static void
prc_parse_polyline_point(prc_context *ctx, prc_bit_state *bit_state,
    uint8_t is_3d, prc_polyline_point *data)
{
    if (is_3d)
    {
        data->point_3d = prc_parse_3d_vector(ctx, bit_state);
    }
    else
    {
        data->point_2d.x = prc_bitread_double(ctx, bit_state);
    }
}

/* Table 281 PRC_TYPE_CRV_Polyline */
static int
prc_parse_crv_polyline(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_polyline *data, uint8_t read_tag)
{
    int code;
    uint32_t i;
    uint8_t is_3d;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_PolyLine)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_polyline\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_PolyLine;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    data->parameterization = prc_parse_parameterization(ctx, bit_state);
    data->number_of_points = prc_bitread_uint32(ctx, bit_state);
    is_3d = data->curve_data.is_3d_flag;

    if (data->number_of_points > 0)
    {
        data->points = (prc_polyline_point *)prc_calloc(ctx, data->number_of_points, sizeof(prc_polyline_point));
        if (data->points == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_crv_polyline\n");
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < data->number_of_points; i++)
        {
            prc_parse_polyline_point(ctx, bit_state, is_3d, &data->points[i]);
        }
    }

    return 0;
}

/* Table 283 PRC_TYPE_CRV_Transform */
static int
prc_parse_crv_transform(prc_context *ctx, prc_bit_state *bit_state,
    prc_crv_transform *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_CRV_Transform)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_crv_transform\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_CRV_Transform;
    }
    code = prc_parse_content_curve(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_curve\n");
        return code;
    }

    return 0;
}

/* Table 242 � PtrCurve */
static int
prc_parse_ptr_curve(prc_context *ctx, prc_bit_state *bit_state, prc_ptr_curve *data)
{
    int code;

    data->is_referenced = prc_bitread_bit(ctx, bit_state);

    if (data->is_referenced)
    {
        data->curve_identifier = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        /* First get the curve type */
        data->curve_type = prc_bitread_uint32(ctx, bit_state);

        switch (data->curve_type)
        {
        case PRC_TYPE_ROOT:
            /* PRC_TYPE_ROOT (0) is used to indicate that the entity corresponds
               to a NULL pointer and no additional data is saved. Otherwise, the
               integer shall be one of the subtypes of curve, surface, or topology.*/
            break;
        case PRC_TYPE_CRV_Blend02Boundary:
            data->crv_blend02_boundary = (prc_crv_blend02_boundary *)prc_calloc(ctx, 1, sizeof(prc_crv_blend02_boundary));
            if (data->crv_blend02_boundary == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_blend02_boundary(ctx, bit_state, data->crv_blend02_boundary, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_blend_02_boundary\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_NURBS:
            data->crv_nurbs = (prc_crv_nurbs *)prc_calloc(ctx, 1, sizeof(prc_crv_nurbs));
            if (data->crv_nurbs == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_nurbs(ctx, bit_state, data->crv_nurbs, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_nurbs\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Circle:
            data->crv_circle = (prc_crv_circle *)prc_calloc(ctx, 1, sizeof(prc_crv_circle));
            if (data->crv_circle == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_circle(ctx, bit_state, data->crv_circle, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_circle\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Composite:
            data->crv_composite = (prc_crv_composite *)prc_calloc(ctx, 1, sizeof(prc_crv_composite));
            if (data->crv_composite == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_composite(ctx, bit_state, data->crv_composite, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_composite\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_OnSurf:
            data->crv_onsurf = (prc_crv_onsurf *)prc_calloc(ctx, 1, sizeof(prc_crv_onsurf));
            if (data->crv_onsurf == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_onsurf(ctx, bit_state, data->crv_onsurf, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_on_surf\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Ellipse:
            data->crv_ellipse = (prc_crv_ellipse *)prc_calloc(ctx, 1, sizeof(prc_crv_ellipse));
            if (data->crv_ellipse == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_ellipse(ctx, bit_state, data->crv_ellipse, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_ellipse\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Equation:
            data->crv_equation = (prc_crv_equation *)prc_calloc(ctx, 1, sizeof(prc_crv_equation));
            if (data->crv_equation == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_equation(ctx, bit_state, data->crv_equation, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_equation\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Helix01:
            data->crv_helix01 = (prc_crv_helix01 *)prc_calloc(ctx, 1, sizeof(prc_crv_helix01));
            if (data->crv_helix01 == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_helix01(ctx, bit_state, data->crv_helix01, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_helix01\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Hyperbola:
            data->crv_hyperbola = (prc_crv_hyperbola *)prc_calloc(ctx, 1, sizeof(prc_crv_hyperbola));
            if (data->crv_hyperbola == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_hyperbola(ctx, bit_state, data->crv_hyperbola, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_hyperbola\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Intersection:
            data->crv_intersection = (prc_crv_intersection *)prc_calloc(ctx, 1, sizeof(prc_crv_intersection));
            if (data->crv_intersection == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_intersection(ctx, bit_state, data->crv_intersection, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_intersection\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Line:
            data->crv_line = (prc_crv_line *)prc_calloc(ctx, 1, sizeof(prc_crv_line));
            if (data->crv_line == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_line(ctx, bit_state, data->crv_line, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_line\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Offset:
            data->crv_offset = (prc_crv_offset *)prc_calloc(ctx, 1, sizeof(prc_crv_offset));
            if (data->crv_offset == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_offset(ctx, bit_state, data->crv_offset, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_offset\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Parabola:
            data->crv_parabola = (prc_crv_parabola *)prc_calloc(ctx, 1, sizeof(prc_crv_parabola));
            if (data->crv_parabola == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_parabola(ctx, bit_state, data->crv_parabola, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_parabola\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_PolyLine:
            data->crv_polyline = (prc_crv_polyline *)prc_calloc(ctx, 1, sizeof(prc_crv_polyline));
            if (data->crv_polyline == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_polyline(ctx, bit_state, data->crv_polyline, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_polyline\n");
                return code;
            }
            break;

        case PRC_TYPE_CRV_Transform:
            data->crv_transform = (prc_crv_transform *)prc_calloc(ctx, 1, sizeof(prc_crv_transform));
            if (data->crv_transform == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_curve\n");
                return PRC_ERROR_MEMORY;
            }
            code = prc_parse_crv_transform(ctx, bit_state, data->crv_transform, DONT_READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_crv_transform\n");
                return code;
            }
            break;

        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown curve type in prc_parse_ptr_curve: %u\n", data->curve_type);
            return PRC_ERROR_PARSE;
        }
    }
    return 0;
}

/* Table 194 � ContentWireEdge */
static int
prc_parse_content_wire_edge(prc_context *ctx, prc_bit_state *bit_state, prc_content_wire_edge *data)
{
    int code;

    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->ptr_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve\n");
        return code;
    }

    data->is_trimmed = prc_bitread_bit(ctx, bit_state);

    if (data->is_trimmed)
    {
        data->trim_interval = prc_parse_interval(ctx, bit_state);
    }

    return 0;
}

/* Table 183 � PRC_TYPE_TOPO_WireEdge */
static int
prc_parse_topo_wire_edge(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_wire_edge *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_WireEdge)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_topo_wire_edge\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_WireEdge;
    }

    code = prc_parse_content_wire_edge(ctx, bit_state, &data->curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_wire_edge\n");
        return code;
    }

    return 0;
}

/* Table 184 PRC_TYPE_TOPO_Edge */
static int
prc_parse_edge(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_edge *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_Edge)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_edge\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Edge;
    }

    code = prc_parse_content_wire_edge(ctx, bit_state, &data->wire_edge);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_wire_edge\n");
        return code;
    }

    code = prc_parse_ptr_topology(ctx, bit_state, &data->start_vertex);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology for start_vertex\n");
        return code;
    }

    code = prc_parse_ptr_topology(ctx, bit_state, &data->end_vertex);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology for end_vertex\n");
        return code;
    }

    data->has_tolerance = prc_bitread_bit(ctx, bit_state);

    if (data->has_tolerance)
    {
        data->tolerance = prc_bitread_double(ctx, bit_state);
    }

    return 0;
}

/* Table 195 PRC_TYPE_TOPO_SingleWireBody */
static int
prc_parse_topo_single_wire_body(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_single_wire_body *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_SingleWireBody)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Error in prc_parse_topo_single_wire_body\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_SingleWireBody;
    }

    /* They all have Table 193 ContentBody */
    code = prc_parse_content_body(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_body\n");
        return code;
    }

    code = prc_parse_ptr_topology(ctx, bit_state, &data->wire_body);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology\n");
        return code;
    }
    return 0;
}

/* Table 191 PRC_TYPE_TOPO_Connex */
static int
prc_parse_topo_connex(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_connex *data, uint8_t read_tag)
{
    int code;
    uint32_t k;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_Connex)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_topo_connex\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Connex;
    }

    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    data->number_of_shells = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_shells > 0)
    {
        data->shells = (prc_ptr_topology*) prc_calloc(ctx, data->number_of_shells, sizeof(prc_ptr_topology));
        if (data->shells == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo_connex\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_shells; k++)
        {
            code = prc_parse_ptr_topology(ctx, bit_state, &data->shells[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ptr_topology\n");
                return code;
            }
        }
    }

    return 0;
}

/* Table 196 PRC_TYPE_TOPO_BrepData */
static int
prc_parse_brep_data(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_brep_data *data, uint8_t read_tag)
{
    int code;
    uint32_t k;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_BrepData)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_brep_data\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_BrepData;
    }

    /* They all have Table 193 ContentBody */
    code = prc_parse_content_body(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_body\n");
        return code;
    }

   // prc_debug_stream(ctx, bit_state);
    /* Number of bits then unsigned integer for this count */
    //data->number_of_connex = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);

    //data->number_of_connex = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
    data->number_of_connex = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_connex > 0)
    {
        data->connex = (prc_ptr_topology *)prc_malloc(ctx, data->number_of_connex * sizeof(prc_ptr_topology));
        if (data->connex == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_brep_data\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_connex; k++)
        {
            code = prc_parse_ptr_topology(ctx, bit_state, &data->connex[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ptr_topology\n");
                return code;
            }
        }
    }

    if ((data->base.bounding_box_behavior & (PRC_BODY_BBOX_Evaluation | PRC_BODY_BBOX_Precise)) != 0)
    {
        code = prc_parse_bound_box(ctx, bit_state, &data->bounding_box);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_bound_box\n");
            return code;
        }
    }

    return 0;
}

static int
prc_parse_single_wire_body_compress(prc_context *ctx, prc_bit_state *bit_state,
                                    prc_topo_single_wire_compress *data,
                                    uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_SingleWireBodyCompress)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Error in prc_parse_topo_single_wire_body_compress\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_SingleWireBodyCompress;
    }

    /* They all have Table 193 ContentBody */
    code = prc_parse_content_body(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_body\n");
        return code;
    }

    return 0;
}

/* Table 178 Basetopology */
static int
prc_parse_base_topology(prc_context *ctx, prc_bit_state *bit_state, prc_base_topology *data)
{
    int code;

    data->has_base = prc_bitread_bit(ctx, bit_state);

    if (data->has_base)
    {
        code = prc_parse_attribute_data(ctx, bit_state, &data->attribute_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_attribute_data\n");
            return code;
        }

        code = prc_parse_name(ctx, bit_state, &data->name);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_name\n");
            return code;
        }

        data->id = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        memset(data, 0, sizeof(prc_base_topology));
    }

    return 0;
}

/* Table 201 CompressedConnex */
static int
prc_parse_compressed_connex(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, uint32_t *num_faces,
    prc_compressed_connex *data)
{
    int code;
    uint32_t k;

    data->number_of_shells = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
    if (data->number_of_shells > 0)
    {
        data->shells = (prc_compressed_shell*) prc_calloc(ctx, data->number_of_shells, sizeof(prc_compressed_shell));
        if (data->shells == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_compressed_connex\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_shells; k++)
        {
            code = prc_parse_compressed_shell(ctx, bit_state, compressed_data,
                                              &data->shells[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_shell\n");
                return code;
            }
            *num_faces += data->shells[k].number_of_faces;
        }
    }
    return 0;
}

/* Table 200 MultipleCompressedConnex */
static int
prc_parse_multiple_compressed_connex(prc_context *ctx, prc_bit_state *bit_state,
    prc_nano_brep_compressed_data *compressed_data, prc_multi_compressed_connex *data)
{
    int code;
    uint32_t k;

    data->number_of_faces = 0;
    data->number_of_connex = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
    if (data->number_of_connex > 0)
    {
        data->connex = (prc_compressed_connex *)prc_calloc(ctx, data->number_of_connex, sizeof(prc_compressed_connex));
        if (data->connex == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_multiple_compressed_connex\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_connex; k++)
        {
            code = prc_parse_compressed_connex(ctx, bit_state, compressed_data,
                                               &data->number_of_faces, &data->connex[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_compressed_connex\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 198 PRC_TYPE_TOPO_BrepDataCompress */
static int
prc_parse_brep_data_compress(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_brep_data_compress *data, uint8_t read_tag)
{
    int code;
    prc_nano_brep_compressed_data *compressed_data;
    uint32_t number_of_faces;
    uint32_t k;

    if (ctx->internal.nano_brep_data == NULL)
    {
        ctx->internal.nano_brep_data = (prc_nano_brep_compressed_data *)prc_calloc(ctx, 1, sizeof(prc_nano_brep_compressed_data));
        if (ctx->internal.nano_brep_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_brep_data_compress\n");
            return PRC_ERROR_MEMORY;
        }
    }
    compressed_data = ctx->internal.nano_brep_data;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_BrepDataCompress)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_brep_data_compress\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_BrepDataCompress;
    }

    /* They all have Table 193 ContentBody */
    code = prc_parse_content_body(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_body\n");
        return code;
    }

    data->brep_data_compressed_tolerance = prc_bitread_double(ctx, bit_state);
    data->number_of_bits_to_store_ref = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
    data->number_of_vertex_refs = prc_bitread_uint_variable_bit(ctx, bit_state, data->number_of_bits_to_store_ref);
    data->number_of_edge_refs = prc_bitread_uint_variable_bit(ctx, bit_state, data->number_of_bits_to_store_ref);
    data->single_connex_test = prc_bitread_bit(ctx, bit_state);

    /* Initialize the compressed_data that has the curves and vertices so that
       we can find them from their stored indices */
    compressed_data->number_bits_for_encoding = data->number_of_bits_to_store_ref;
    compressed_data->number_of_vertex_refs = data->number_of_vertex_refs;
    compressed_data->number_of_edge_refs = data->number_of_edge_refs;
    compressed_data->is_a_compressed_face = true;

    compressed_data->vertices = (prc_compressed_vertex *)prc_calloc(ctx,
        BREP_VERTEX_INITIAL_SIZE, sizeof(prc_compressed_vertex));
    if (compressed_data->vertices == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_brep_data_compress\n");
        return PRC_ERROR_MEMORY;
    }
    compressed_data->vertices_capacity = BREP_VERTEX_INITIAL_SIZE;

    compressed_data->curves = (prc_compressed_curve *)prc_calloc(ctx,
        BREP_EDGE_INITIAL_SIZE, sizeof(prc_compressed_curve));
    if (compressed_data->curves == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_brep_data_compress\n");
        prc_free(ctx, compressed_data->vertices);
        compressed_data->vertices = NULL;
        return PRC_ERROR_MEMORY;
    }
    compressed_data->curves_capacity = BREP_EDGE_INITIAL_SIZE;

    compressed_data->current_curve_index = 0;
    compressed_data->current_vertex_index = 0;

    /* Section 7.9.21.11 */
    compressed_data->tolerance = data->brep_data_compressed_tolerance / 100.0;

    /* True if PRC_TYPE_TOPO_BrepDataCompress False if
       PRC_TYPE_TOPO_SingleWireBodyCompress */
    compressed_data->curve_trimming_face = 1;

    if (data->single_connex_test)
    {
        /* Read a single compressed shell */
        code = prc_parse_compressed_shell(ctx, bit_state, compressed_data,
            &data->single_connex);
        number_of_faces = data->single_connex.number_of_faces;

        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_compressed_shell\n");
            return code;
        }
    }
    else
    {
        /* Multiple compressed connex stored in file*/
        code = prc_parse_multiple_compressed_connex(ctx, bit_state, compressed_data,
            &data->multi_connex);
        number_of_faces = data->multi_connex.number_of_faces;

        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_multiple_compressed_connex\n");
            return code;
        }
    }

    /* Get the base topology data. Order of these corresponds
       to the order in which the faces are encountered in the
       scanning of the connex/shell data above. */
    data->number_of_faces = number_of_faces;
    if (number_of_faces > 0)
    {
        data->base_topology = (prc_base_topology *)prc_calloc(ctx,
            number_of_faces, sizeof(prc_base_topology));
        if (data->base_topology == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_brep_data_compress\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < number_of_faces; k++)
        {
            code = prc_parse_base_topology(ctx, bit_state, &data->base_topology[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_base_topology for face base topology\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 193 ContentBody */
static int
prc_parse_content_body(prc_context *ctx, prc_bit_state *bit_state, prc_content_body *data)
{
    int code;

    code = prc_parse_base_topology(ctx, bit_state, &data->base_topology);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    data->bounding_box_behavior = prc_bitread_uint8(ctx, bit_state);

    return 0;
}

/* Table 181 � PRC_TYPE_TOPO_MultipleVertex */
static int
prc_parse_multiple_vertex(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_multiple_vertex *data, uint8_t read_tag)
{
    int code;
    size_t k;
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_MultipleVertex)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_multiple_vertex\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_MultipleVertex;
    }
    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    data->number_of_points = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_points > 0)
    {
        data->points = (prc_vec3 *)prc_calloc(ctx, data->number_of_points, sizeof(prc_vec3));
        if (data->points == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_multiple_vertex\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_points; k++)
        {
            data->points[k] = prc_parse_3d_vector(ctx, bit_state);
        }
    }

    return 0;
}

/* Table 185 � PRC_TYPE_TOPO_CoEdge */
static int
prc_parse_coedge(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_coedge *data, uint8_t read_tag)
{
    int code;
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_CoEdge)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_coedge\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_CoEdge;
    }
    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }
    code = prc_parse_ptr_topology(ctx, bit_state, &data->ptr_topology);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology for edge\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->ptr_curves);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for ptr_curves\n");
        return code;
    }

    data->coedge_orientation = prc_bitread_uint8(ctx, bit_state);
    data->uv_orientation = prc_bitread_uint8(ctx, bit_state);
    return 0;
}

/* Table 187 � CoedgeInLoop */
static int
prc_parse_coedge_in_loop(prc_context *ctx, prc_bit_state *bit_state,
    prc_coedge_in_loop *data)
{
    int code;

    code = prc_parse_ptr_topology(ctx, bit_state, &data->next_coedge);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology for coedge\n");
        return code;
    }

    data->neighbor_index = prc_bitread_uint32(ctx, bit_state);
    return 0;
}

/* Table 186 � PRC_TYPE_TOPO_Loop */
static int
prc_parse_loop(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_loop *data, uint8_t read_tag)
{
    int code;
    size_t k;
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_Loop)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_loop\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Loop;
    }
    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    data->loop_orientation = prc_bitread_uint8(ctx, bit_state);
    data->number_of_coedges = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_coedges > 0)
    {
        data->coedge = (prc_coedge_in_loop *)prc_calloc(ctx, data->number_of_coedges, sizeof(prc_coedge_in_loop));
        if (data->coedge == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_loop\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_coedges; k++)
        {
            code = prc_parse_coedge_in_loop(ctx, bit_state, &data->coedge[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ptr_topology\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 286 � ContentSurface */
static int
prc_parse_content_surface(prc_context *ctx, prc_bit_state *bit_state,
    prc_content_surface *data)
{
    int code;

    data->has_base_geometry = prc_bitread_bit(ctx, bit_state);

    if (data->has_base_geometry)
    {
        code = prc_parse_attribute_data(ctx, bit_state, &data->attribute_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_attribute_data\n");
            return code;
        }
        code = prc_parse_name(ctx, bit_state, &data->name);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_name\n");
            return code;
        }
        data->id = prc_bitread_uint32(ctx, bit_state);
    }

    data->extension_type = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 288 PRC_TYPE_SURF_Blend01 */
static int
prc_parse_surf_blend01(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_blend01 *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Blend01)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_blend01\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Blend01;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for curve_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->center_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for center_curve\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->origin_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for origin_curve\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->tangent_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for tangent_curve\n");
        return code;
    }

    return 0;
}

/* Table 290 � PRC_TYPE_SURF_Blend02 */
static int
prc_parse_surf_blend02(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_blend02 *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Blend02)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_blend02\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Blend02;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for curve_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->bound_surface0);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for bound_surface0\n");
        return code;
    }
    code = prc_parse_ptr_curve(ctx, bit_state, &data->bound_curve0);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for bound_curve0\n");
        return code;
    }

    code = prc_parse_ptr_surface(ctx, bit_state, &data->bound_surface1);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for bound_surface1\n");
        return code;
    }
    code = prc_parse_ptr_curve(ctx, bit_state, &data->bound_curve1);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for bound_curve1\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->center_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for center_curve\n");
        return code;
    }

    data->center_curve_sense = prc_bitread_bit(ctx, bit_state);
    data->bound_surface0_sense = prc_bitread_bit(ctx, bit_state);
    data->bound_surface1_sense = prc_bitread_bit(ctx, bit_state);

    data->radius0 = prc_bitread_double(ctx, bit_state);
    data->radius1 = prc_bitread_double(ctx, bit_state);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->cliff_surface0);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for cliff_surface0\n");
        return code;
    }
    code = prc_parse_ptr_surface(ctx, bit_state, &data->cliff_surface1);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for cliff_surface1\n");
        return code;
    }

    data->parameterization_type = prc_bitread_uint8(ctx, bit_state);

    return 0;
}
/* Table 292 � PRC_TYPE_SURF_Blend03 */
static int
prc_parse_surf_blend03(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_blend03 *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Blend03)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_blend03\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Blend03;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for curve_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    data->number_of_elements = prc_bitread_int32(ctx, bit_state); /* Unsigned int ?? */

    if (data->number_of_elements > 0)
    {
        data->parameters = (double *)prc_calloc(ctx, data->number_of_elements, sizeof(double));
        if (data->parameters == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements; k++)
        {
            data->parameters[k] = prc_bitread_double(ctx, bit_state);
        }

        data->multiplicities = (int32_t *)prc_calloc(ctx, data->number_of_elements, sizeof(int32_t));
        if (data->multiplicities == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements; k++)
        {
            data->multiplicities[k] = prc_bitread_int32(ctx, bit_state);
        }

        data->points = (prc_vec3 *)prc_calloc(ctx, data->number_of_elements * 3, sizeof(prc_vec3));
        if (data->points == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements * 3; k++)
        {
            data->points[k] = prc_parse_3d_vector(ctx, bit_state);
        }

        data->rail_2_angles_v = (double *)prc_calloc(ctx, data->number_of_elements, sizeof(double));
        if (data->rail_2_angles_v == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements; k++)
        {
            data->rail_2_angles_v[k] = prc_bitread_double(ctx, bit_state);
        }

        data->tangents = (prc_vec3 *)prc_calloc(ctx, data->number_of_elements * 3, sizeof(prc_vec3));
        if (data->tangents == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements * 3; k++)
        {
            data->tangents[k] = prc_parse_3d_vector(ctx, bit_state);
        }

        data->rail_2_derivatives_v = (double *)prc_calloc(ctx, data->number_of_elements, sizeof(double));
        if (data->rail_2_derivatives_v == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements; k++)
        {
            data->rail_2_derivatives_v[k] = prc_bitread_double(ctx, bit_state);
        }

        data->second_derivatives = (prc_vec3 *)prc_calloc(ctx, data->number_of_elements * 3, sizeof(prc_vec3));
        if (data->second_derivatives == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements * 3; k++)
        {
            data->second_derivatives[k] = prc_parse_3d_vector(ctx, bit_state);
        }

        data->rail_2_second_derivatives = (double *)prc_calloc(ctx, data->number_of_elements, sizeof(double));
        if (data->rail_2_second_derivatives == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_elements; k++)
        {
            data->rail_2_second_derivatives[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    data->rail_2_parameter_v = prc_bitread_double(ctx, bit_state);
    data->trim_v_min = prc_bitread_double(ctx, bit_state);
    data->trim_v_max = prc_bitread_double(ctx, bit_state);

    for (int32_t k = 0; k < 6; k++)
    {
        data->reserved_int[k] = prc_bitread_int32(ctx, bit_state);
    }

    data->reserved_char_0 = prc_bitread_uint8(ctx, bit_state);
    data->reserved_char_1 = prc_bitread_uint8(ctx, bit_state);
    data->reserved_char_2 = prc_bitread_uint8(ctx, bit_state);
    data->number_of_supplemental_doubles = prc_bitread_int32(ctx, bit_state);
    if (data->number_of_supplemental_doubles > 0)
    {
        data->supplemental_doubles = (double *)prc_calloc(ctx, data->number_of_supplemental_doubles, sizeof(double));
        if (data->supplemental_doubles == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_blend03\n");
            return PRC_ERROR_MEMORY;
        }
        for (int32_t k = 0; k < data->number_of_supplemental_doubles; k++)
        {
            data->supplemental_doubles[k] = prc_bitread_double(ctx, bit_state);
        }
    }
     return 0;
}

/* Table 294 � ControlPointsNURBSSurf */
static void
prc_parse_control_points_nurbs_surf(prc_context *ctx, prc_bit_state *bit_state,
    uint8_t is_rational, prc_control_points_nurbs_surf *data)
{
    data->x = prc_bitread_double(ctx, bit_state);
    data->y = prc_bitread_double(ctx, bit_state);
    data->z = prc_bitread_double(ctx, bit_state);

    if (is_rational)
    {
        data->w = prc_bitread_double(ctx, bit_state);
    }
    else
    {
        data->w = 1.0;
    }
}

/* Table 293 � PRC_TYPE_SURF_NURBS */
static int
prc_parse_surf_nurbs(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_nurbs *data, uint8_t read_tag)
{
    int code;
    uint32_t array_size;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_NURBS)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_nurbs\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_NURBS;
    }
    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->is_rational = prc_bitread_bit(ctx, bit_state);
    data->du = prc_bitread_uint32(ctx, bit_state);
    data->dv = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_control_points_u = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_control_points_v = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_knots_u = prc_bitread_uint32(ctx, bit_state);
    data->highest_index_of_knots_v = prc_bitread_uint32(ctx, bit_state);

    array_size = (data->highest_index_of_control_points_u + 1) *
        (data->highest_index_of_control_points_v + 1);
    data->p = (prc_control_points_nurbs_surf *)prc_calloc(ctx, array_size,
        sizeof(prc_control_points_nurbs_surf));
    if (data->p == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_nurbs\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t k = 0; k < array_size; k++)
    {
        prc_parse_control_points_nurbs_surf(ctx, bit_state, data->is_rational, &data->p[k]);
    }

    array_size = data->highest_index_of_knots_u + 1;
    data->knot_vector_u = (double *)prc_calloc(ctx, array_size, sizeof(double));
    if (data->knot_vector_u == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_nurbs\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t k = 0; k < array_size; k++)
    {
        data->knot_vector_u[k] = prc_bitread_double(ctx, bit_state);
    }
    array_size = data->highest_index_of_knots_v + 1;
    data->knot_vector_v = (double *)prc_calloc(ctx, array_size, sizeof(double));
    if (data->knot_vector_v == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_surf_nurbs\n");
        return PRC_ERROR_MEMORY;
    }
    for (uint32_t k = 0; k < array_size; k++)
    {
        data->knot_vector_v[k] = prc_bitread_double(ctx, bit_state);
    }

    data->knot_type = prc_bitread_uint32(ctx, bit_state);
    data->surface_form = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 297 � PRC_TYPE_SURF_Cone */
static int
prc_parse_surf_cone(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_cone *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Cone)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_cone\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Cone;
    }
    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }
    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);
    data->radius = prc_bitread_double(ctx, bit_state);
    data->semi_angle = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 299 � PRC_TYPE_SURF_Cylinder */
static int
prc_parse_surf_cylinder(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_cylinder *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Cylinder)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_cylinder\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Cylinder;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }
    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);
    data->radius = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 301 � PRC_TYPE_SURF_Cylindrical */
static int
prc_parse_surf_cylindrical(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_cylindrical *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Cylindrical)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_cylindrical\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Cylindrical;
    }
    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->base_surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for base_surface\n");
        return code;
    }

    data->tolerance = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 303 PRC_TYPE_SURF_Offset */
static int
prc_parse_surf_offset(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_offset *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Offset)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_offset\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Offset;
    }
    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->base_surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for base_surface\n");
        return code;
    }

    data->offset_distance = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 304 � PRC_TYPE_SURF_Pipe */
static int
prc_parse_surf_pipe(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_pipe *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Pipe)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_pipe\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Pipe;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    prc_parse_3d_transform(ctx, bit_state, &data->transform);
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->center_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for center_curve\n");
        return code;
    }

    code = prc_parse_ptr_curve(ctx, bit_state, &data->origin_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for origin_curve\n");
        return code;
    }

    data->radius = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* This type is used in the tree structures.. Need to check if it should
   be read as this or a SURF_PTR from those types */
/* Table 306 � PRC_TYPE_SURF_Plane */
int
prc_parse_surf_plane(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_plane *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Plane)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_plane\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Plane;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    /* Does not have a has_transform check */
    prc_parse_3d_transform(ctx, bit_state, &data->transform);

    /* Note to self.  -12345 is -inf and 12345 is +inf */
    data->domain = prc_parse_domain(ctx, bit_state);

    data->u_parameter_coeff_a = prc_bitread_double(ctx, bit_state);
    data->v_parameter_coeff_a = prc_bitread_double(ctx, bit_state);
    data->u_parameter_coeff_b = prc_bitread_double(ctx, bit_state);
    data->v_parameter_coeff_b = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 308 � PRC_TYPE_SURF_Ruled */
static int
prc_parse_surf_ruled(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_ruled *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Ruled)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_ruled\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Ruled;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }
    prc_parse_3d_transform(ctx, bit_state, &data->transform);
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->first_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for first_curve\n");
        return code;
    }
    code = prc_parse_ptr_curve(ctx, bit_state, &data->second_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for second_curve\n");
        return code;
    }

    return 0;
}

/* Table 310 � PRC_TYPE_SURF_Sphere */
static int
prc_parse_surf_sphere(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_sphere *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Sphere)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_sphere\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Sphere;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    data->radius = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 312 � PRC_TYPE_SURF_Revolution */
static int
prc_parse_surf_revolution(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_revolution *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Revolution)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_revolution\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Revolution;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);
    data->tolerance = prc_bitread_double(ctx, bit_state);

    data->origin = prc_parse_3d_vector(ctx, bit_state);
    data->x_axis = prc_parse_3d_vector(ctx, bit_state);
    data->y_axis = prc_parse_3d_vector(ctx, bit_state);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->base_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for base_curve\n");
        return code;
    }

    return 0;
}

/* Table 314 � PRC_TYPE_SURF_Extrusion */
static int
prc_parse_surf_extrusion(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_extrusion *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Extrusion)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_extrusion\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Extrusion;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);
    data->sweep_vector = prc_parse_3d_vector(ctx, bit_state);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->base_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for base_curve\n");
        return code;
    }

    return 0;
}

/* Table 316 � PRC_TYPE_SURF_FromCurves */
static int
prc_parse_surf_from_curves(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_fromcurves *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_FromCurves)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_from_curves\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_FromCurves;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }
    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    data->origin = prc_parse_3d_vector(ctx, bit_state);

    code = prc_parse_ptr_curve(ctx, bit_state, &data->first_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for first_curve\n");
        return code;
    }
    code = prc_parse_ptr_curve(ctx, bit_state, &data->second_curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_curve for second_curve\n");
        return code;
    }

    return 0;
}

/* Table 318 � PRC_TYPE_SURF_Torus */
static int
prc_parse_surf_torus(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_torus *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Torus)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_torus\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Torus;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform);
    }
    memset(&data->exact_geom_transform, 0, sizeof(data->exact_geom_transform));
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);
    data->major_radius = prc_bitread_double(ctx, bit_state);
    data->minor_radius = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 320 � PRC_TYPE_SURF_Transform */
static int
prc_parse_surf_transform(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_transform *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_SURF_Transform)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_surf_transform\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_SURF_Transform;
    }

    code = prc_parse_content_surface(ctx, bit_state, &data->curve_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_content_surface for surface_data\n");
        return code;
    }

    prc_parse_3d_transform(ctx, bit_state, &data->transform);
    prc_parse_uv_parameterization(ctx, bit_state, &data->parameterization);

    code = prc_parse_ptr_surface(ctx, bit_state, &data->base_surface);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for base_surface\n");
        return code;
    }

    code = prc_parse_math_fct_3d(ctx, bit_state, &data->math_transform);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_math_fct_3d for math_transform\n");
        return code;
    }

    return 0;
}

static int
prc_parse_surf(prc_context *ctx, prc_bit_state *bit_state, prc_type_surf *data)
{
    int code = 0;

    /* First get the surface type */
    data->surface_type = prc_bitread_uint32(ctx, bit_state);

    switch (data->surface_type)
    {
    case PRC_TYPE_ROOT:
        /* Null case */
        break;
    case PRC_TYPE_SURF_Blend01:
        data->surf_blend01 = (prc_surf_blend01 *)prc_calloc(ctx, 1, sizeof(prc_surf_blend01));
        if (data->surf_blend01 == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_blend01(ctx, bit_state, data->surf_blend01, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_blend01\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Blend02:
        data->surf_blend02 = (prc_surf_blend02 *)prc_calloc(ctx, 1, sizeof(prc_surf_blend02));
        if (data->surf_blend02 == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_blend02(ctx, bit_state, data->surf_blend02, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_blend02\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Blend03:
        data->surf_blend03 = (prc_surf_blend03 *)prc_calloc(ctx, 1, sizeof(prc_surf_blend03));
        if (data->surf_blend03 == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_blend03(ctx, bit_state, data->surf_blend03, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_blend03\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_NURBS:
        data->surf_nurbs = (prc_surf_nurbs *)prc_calloc(ctx, 1, sizeof(prc_surf_nurbs));
        if (data->surf_nurbs == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_nurbs(ctx, bit_state, data->surf_nurbs, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_nurbs\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Cone:
        data->surf_cone = (prc_surf_cone *)prc_calloc(ctx, 1, sizeof(prc_surf_cone));
        if (data->surf_cone == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_cone(ctx, bit_state, data->surf_cone, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_cone\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Cylinder:
        data->surf_cylinder = (prc_surf_cylinder *)prc_calloc(ctx, 1, sizeof(prc_surf_cylinder));
        if (data->surf_cylinder == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_cylinder(ctx, bit_state, data->surf_cylinder, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_cylinder\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Cylindrical:
        data->surf_cylindrical = (prc_surf_cylindrical *)prc_calloc(ctx, 1, sizeof(prc_surf_cylindrical));
        if (data->surf_cylindrical == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_cylindrical(ctx, bit_state, data->surf_cylindrical, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_cyldrical\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Offset:
        data->surf_offset = (prc_surf_offset *)prc_calloc(ctx, 1, sizeof(prc_surf_offset));
        if (data->surf_offset == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_offset(ctx, bit_state, data->surf_offset, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_offset\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Pipe:
        data->surf_pipe = (prc_surf_pipe *)prc_calloc(ctx, 1, sizeof(prc_surf_pipe));
        if (data->surf_pipe == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_pipe(ctx, bit_state, data->surf_pipe, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_pipe\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Plane:
        data->surf_plane = (prc_surf_plane *)prc_calloc(ctx, 1, sizeof(prc_surf_plane));
        if (data->surf_plane == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_plane(ctx, bit_state, data->surf_plane, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_plane\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Ruled:
        data->surf_ruled = (prc_surf_ruled *)prc_calloc(ctx, 1, sizeof(prc_surf_ruled));
        if (data->surf_ruled == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_ruled(ctx, bit_state, data->surf_ruled, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_ruled\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Sphere:
        data->surf_sphere = (prc_surf_sphere *)prc_calloc(ctx, 1, sizeof(prc_surf_sphere));
        if (data->surf_sphere == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_sphere(ctx, bit_state, data->surf_sphere, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_sphere\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Revolution:
        data->surf_revolution = (prc_surf_revolution *)prc_calloc(ctx, 1, sizeof(prc_surf_revolution));
        if (data->surf_revolution == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_revolution(ctx, bit_state, data->surf_revolution, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_revolution\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Extrusion:
        data->surf_extrusion = (prc_surf_extrusion *)prc_calloc(ctx, 1, sizeof(prc_surf_extrusion));
        if (data->surf_extrusion == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_extrusion(ctx, bit_state, data->surf_extrusion, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_extrusion\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_FromCurves:
        data->surf_fromcurves = (prc_surf_fromcurves *)prc_calloc(ctx, 1, sizeof(prc_surf_fromcurves));
        if (data->surf_fromcurves == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_from_curves(ctx, bit_state, data->surf_fromcurves, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_from_curves\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Torus:
        data->surf_torus = (prc_surf_torus *)prc_calloc(ctx, 1, sizeof(prc_surf_torus));
        if (data->surf_torus == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_torus(ctx, bit_state, data->surf_torus, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_torus\n");
            return code;
        }
        break;

    case PRC_TYPE_SURF_Transform:
        data->surf_transform = (prc_surf_transform *)prc_calloc(ctx, 1, sizeof(prc_surf_transform));
        if (data->surf_transform == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ptr_surface\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_surf_transform(ctx, bit_state, data->surf_transform, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf_transform\n");
            return code;
        }
        break;

    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Unknown surface type in prc_parse_ptr_surface\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 243 � PtrSurface */
static int
prc_parse_ptr_surface(prc_context *ctx, prc_bit_state *bit_state, prc_ptr_surface *data)
{
    int code = 0;

    data->is_referenced = prc_bitread_bit(ctx, bit_state);

    if (data->is_referenced)
    {
        data->surface_identifier = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        code = prc_parse_surf(ctx, bit_state, &data->surface);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_surf for surface\n");
            return code;
        }
    }
    return code;
}

/* Table 188 � PRC_TYPE_TOPO_Face */
static int
prc_parse_face(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_face *data, uint8_t read_tag)
{
    int code;
    size_t k;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_Face)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_face\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Face;
    }

    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    code = prc_parse_ptr_surface(ctx, bit_state, &data->surface_geometry);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_surface for ptr_surface\n");
        return code;
    }

    data->is_trimmed = prc_bitread_bit(ctx, bit_state);
    if (data->is_trimmed)
    {
        data->trimmed_surface = prc_parse_domain(ctx, bit_state);
    }

    data->has_tolerance = prc_bitread_bit(ctx, bit_state);
    if (data->has_tolerance)
    {
        data->tolerance = prc_bitread_double(ctx, bit_state);
    }

    data->number_of_loops = prc_bitread_uint32(ctx, bit_state);

    /* This should be set to -1 if it is not defined */
    data->index_of_outer_loop = prc_bitread_int32(ctx, bit_state);

    if (data->number_of_loops > 0)
    {
        data->loops = (prc_ptr_topology *)prc_calloc(ctx, data->number_of_loops, sizeof(prc_ptr_topology));
        if (data->loops == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_face\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_loops; k++)
        {
            code = prc_parse_ptr_topology(ctx, bit_state, &data->loops[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ptr_topology\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 190 � FacesInShell */
static int
prc_parse_faces_in_shell(prc_context *ctx, prc_bit_state *bit_state,
    prc_faces_in_shell *data)
{
    int code;

    code = prc_parse_ptr_topology(ctx, bit_state, &data->face);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ptr_topology for face\n");
        return code;
    }

    data->orientation = prc_bitread_uint8(ctx, bit_state);
    return 0;
}

/* Table 189 � PRC_TYPE_TOPO_Shell */
static int
prc_parse_shell(prc_context *ctx, prc_bit_state *bit_state,
    prc_topo_shell *data, uint8_t read_tag)
{
    int code;
    size_t k;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
        if (data->tag != PRC_TYPE_TOPO_Shell)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_shell\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Shell;
    }

    code = prc_parse_base_topology(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_base_topology\n");
        return code;
    }

    data->is_closed = prc_bitread_bit(ctx, bit_state);

    data->number_of_faces = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_faces > 0)
    {
        data->faces = (prc_faces_in_shell *)prc_calloc(ctx, data->number_of_faces, sizeof(prc_faces_in_shell));
        if (data->faces == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_shell\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_faces; k++)
        {
            code = prc_parse_faces_in_shell(ctx, bit_state, &data->faces[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ptr_topology\n");
                return code;
            }
        }
    }
    return 0;
}

/* Abstract type */
static int
prc_parse_topo(prc_context *ctx, prc_bit_state *bit_state, prc_topo *data, int depth)
{
    int code = 0;

    /* PRC_TYPE_TOPO_Body self-recurses with a tag pair supplied entirely by the
       file (a few bytes per level), so an attacker can drive recursion depth
       proportional to file size. Cap it to avoid stack exhaustion. */
    if (depth > PRC_TOPO_BODY_RECURSION_MAX)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "prc_parse_topo recursion depth exceeded\n");
        return PRC_ERROR_PARSE;
    }

    data->tag = prc_bitread_uint32(ctx, bit_state);

    switch (data->tag)
    {
    case PRC_TYPE_ROOT:
        /* Null case */
        break;
    case PRC_TYPE_TOPO_MultipleVertex:
        data->topo_multiple_vertex = (prc_topo_multiple_vertex *)prc_calloc(ctx, 1, sizeof(prc_topo_multiple_vertex));
        if (data->topo_multiple_vertex == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_multiple_vertex(ctx, bit_state, data->topo_multiple_vertex, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_UniqueVertex:
        data->topo_unique_vertex = (prc_topo_unique_vertex *)prc_calloc(ctx, 1, sizeof(prc_topo_unique_vertex));
        if (data->topo_unique_vertex == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_unique_vertex(ctx, bit_state, data->topo_unique_vertex, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_WireEdge:
        data->topo_wire_edge = (prc_topo_wire_edge *)prc_calloc(ctx, 1, sizeof(prc_topo_wire_edge));
        if (data->topo_wire_edge == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_topo_wire_edge(ctx, bit_state, data->topo_wire_edge, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Edge:
        data->topo_edge = (prc_topo_edge *)prc_calloc(ctx, 1, sizeof(prc_topo_edge));
        if (data->topo_edge == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_edge(ctx, bit_state, data->topo_edge, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_CoEdge:
        data->topo_coedge = (prc_topo_coedge *)prc_calloc(ctx, 1, sizeof(prc_topo_coedge));
        if (data->topo_coedge == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_coedge(ctx, bit_state, data->topo_coedge, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Loop:
        data->topo_loop = (prc_topo_loop *)prc_calloc(ctx, 1, sizeof(prc_topo_loop));
        if (data->topo_loop == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_loop(ctx, bit_state, data->topo_loop, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Face:
        data->topo_face = (prc_topo_face *)prc_calloc(ctx, 1, sizeof(prc_topo_face));
        if (data->topo_face == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_face(ctx, bit_state, data->topo_face, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Shell:
        data->topo_shell = (prc_topo_shell *)prc_calloc(ctx, 1, sizeof(prc_topo_shell));
        if (data->topo_shell == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_shell(ctx, bit_state, data->topo_shell, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Connex:
        data->topo_connex = (prc_topo_connex *)prc_calloc(ctx, 1, sizeof(prc_topo_connex));
        if (data->topo_connex == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_topo_connex(ctx, bit_state, data->topo_connex, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_Body:
        data->topo_body = (prc_topo *)prc_calloc(ctx, 1, sizeof(prc_topo));
        if (data->topo_body == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        /* Note the recursion */
        code = prc_parse_topo(ctx, bit_state, data->topo_body, depth + 1);
        break;

    case PRC_TYPE_TOPO_SingleWireBody:
        data->topo_single_wire_body = (prc_topo_single_wire_body *)prc_calloc(ctx, 1, sizeof(prc_topo_single_wire_body));
        if (data->topo_single_wire_body == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_topo_single_wire_body(ctx, bit_state, data->topo_single_wire_body, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_BrepData:
        data->topo_brep_data = (prc_topo_brep_data *)prc_calloc(ctx, 1, sizeof(prc_topo_brep_data));
        if (data->topo_brep_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_brep_data(ctx, bit_state, data->topo_brep_data, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_SingleWireBodyCompress:
        data->topo_single_wire_compress = (prc_topo_single_wire_compress *)prc_calloc(ctx, 1, sizeof(prc_topo_single_wire_compress));
        if (data->topo_single_wire_compress == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_single_wire_body_compress(ctx, bit_state, data->topo_single_wire_compress, DONT_READ_TAG);
        break;

    case PRC_TYPE_TOPO_BrepDataCompress:
        data->topo_brep_data_compress = (prc_topo_brep_data_compress *)prc_calloc(ctx, 1, sizeof(prc_topo_brep_data_compress));
        if (data->topo_brep_data_compress == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_brep_data_compress(ctx, bit_state, data->topo_brep_data_compress, DONT_READ_TAG);

        /* For now, done with this data. Release it */
        prc_nano_brep_compressed_data *compressed_data = ctx->internal.nano_brep_data;
        if (compressed_data != NULL)
        {
            if (compressed_data->vertices != NULL)
            {
                prc_free(ctx, compressed_data->vertices);
            }
            if (compressed_data->curves != NULL)
            {
                for (uint32_t k = 0; k < compressed_data->current_curve_index; k++)
                {
                    prc_release_compressed_curve(ctx, &compressed_data->curves[k]);
                }
                prc_free(ctx, compressed_data->curves);
            }
            prc_free(ctx, compressed_data);
            ctx->internal.nano_brep_data = NULL;
        }
        break;

    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_topo\n");
        return PRC_ERROR_PARSE;
    }
    return code;
}

/* Table 180 */
static int
prc_parse_topo_contexts(prc_context *ctx, prc_bit_state *bit_state,
                        prc_topo_context *data, uint8_t read_tag)
{
    int code;
    size_t k;

    if (read_tag)
    {
        code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_TOPO_Context, &data->tag);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_read_check_tag\n");
            return code;
        }
    }
    else
    {
        data->tag = PRC_TYPE_TOPO_Context;
    }

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    data->behavior = prc_bitread_uint8(ctx, bit_state);
    data->grandularity = prc_bitread_double(ctx, bit_state);
    data->tolerance = prc_bitread_double(ctx, bit_state);
    data->has_face_thickness = prc_bitread_bit(ctx, bit_state);
    if (data->has_face_thickness)
    {
        data->smallest_face_thickness = prc_bitread_double(ctx, bit_state);
    }
    data->has_scale = prc_bitread_bit(ctx, bit_state);
    if (data->has_scale)
    {
        data->has_scale_option_undefined_in_spec = prc_bitread_double(ctx, bit_state);
    }
    data->number_of_bodies = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_bodies > 0)
    {
        data->bodies = (prc_topo *)prc_calloc(ctx, data->number_of_bodies, sizeof(prc_topo));
        if (data->bodies == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_topo_contexts\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_bodies; k++)
        {
            code = prc_parse_topo(ctx, bit_state, &data->bodies[k], 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_topo\n");
                return code;
            }
        }
    }

    return 0;
}

/* Table 50 FileStructureExactGeometry */
int
prc_parse_exact_geometry(prc_context *ctx, prc_bit_state *bit_state, prc_file_structure_exact_geometry *data)
{
    int code = 0;
    uint32_t k;

    data->topo_context_count = prc_bitread_uint32(ctx, bit_state);
    if (data->topo_context_count > 0)
    {
        data->topo_contexts = (prc_topo_context *)prc_calloc(ctx, data->topo_context_count, sizeof(prc_topo_context));
        if (data->topo_contexts == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_exact_geometry\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->topo_context_count; k++)
        {
            code = prc_parse_topo_contexts(ctx, bit_state, &data->topo_contexts[k], READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_topo_contexts\n");
                return code;
            }
        }
    }
    return code;
}

/* Table 54 Body Information */
static int
prc_parse_body_information(prc_context *ctx, prc_bit_state *bit_state, prc_body_information *data)
{
    int code = 0;

    data->body_serial_type = prc_bitread_uint32(ctx, bit_state);

    if (data->body_serial_type == PRC_TYPE_TOPO_BrepDataCompress ||
        data->body_serial_type == PRC_TYPE_TOPO_SingleWireBodyCompress ||
        data->body_serial_type == PRC_TYPE_TESS_3D_Compressed)
    {
        data->tolerance = prc_bitread_double(ctx, bit_state);
    }

    return code;
}

/* Table 53 Geometry Summary */
static int
prc_parse_geometry_summary(prc_context *ctx, prc_bit_state *bit_state, prc_geometry_summary *data)
{
    int code = 0;
    uint32_t k;

    data->number_of_bodies = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_bodies > 0)
    {
        data->bodies = (prc_body_information *)prc_malloc(ctx, data->number_of_bodies * sizeof(prc_body_information));
        if (data->bodies == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_geometry_summary\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_bodies; k++)
        {
            code = prc_parse_body_information(ctx, bit_state, &data->bodies[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_body_information\n");
                return code;
            }
        }
    }
    return code;
}

/* Table 58 ElementGraphicsBehavior */
static int
prc_parse_element_graphics_behavior(prc_context *ctx, prc_bit_state *bit_state, prc_element_graphics_behavior *data)
{
    int code = 0;

    data->use_context = prc_bitread_bit(ctx, bit_state);

    /* Per the spec.  If use_context is false the read in these indexes to the FilterStructureInternalGlobalData */
    if (data->use_context == 0)
    {
        data->biased_layer_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_index_of_line_style = prc_bitread_uint32(ctx, bit_state);

        /* And this bit.. */
        data->behavior_bit_field[0] = prc_bitread_uint8(ctx, bit_state);
        data->behavior_bit_field[1] = prc_bitread_uint8(ctx, bit_state);
    }
    return code;
}

/* Table 57 ElementInformation */
static int
prc_parse_element_information(prc_context *ctx, prc_bit_state *bit_state, prc_element_information *data)
{
    int code = 0;

    data->has_graphics = prc_bitread_bit(ctx, bit_state);
    if (data->has_graphics)
    {
        code = prc_parse_element_graphics_behavior(ctx, bit_state, &data->graphic_behavoir);
    }
    return code;
}

/* Table 56 GraphicsInformation */
static int
prc_parse_graphics_information(prc_context *ctx, prc_bit_state *bit_state, prc_graphics_information *data)
{
    int code = 0;
    uint32_t k;

    data->element_type = prc_bitread_uint32(ctx, bit_state);
    data->number_of_element = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_element > 0)
    {
        data->element_information = (prc_element_information *)prc_malloc(ctx, data->number_of_element * sizeof(prc_element_information));
        if (data->element_information == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_graphics_information\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_element; k++)
        {
            code = prc_parse_element_information(ctx, bit_state, &data->element_information[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_element_information\n");
                return code;
            }
        }
    }
    return code;
}

/* Table 55 ContextGraphics */
static int
prc_parse_context_graphics(prc_context *ctx, prc_bit_state *bit_state, prc_context_graphics *data)
{
    int code = 0;
    uint32_t k;

    data->number_of_treat_type = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_treat_type > 0)
    {
        /* Zero-initialized: if prc_parse_graphics_information fails partway
           through the loop below, this function returns early and leaves the
           remaining elements untouched. number_of_treat_type still reports
           the full count, so release code later walks past the populated
           elements -- calloc keeps their pointer fields NULL (instead of
           uninitialized/malloc garbage) so that walk is safe. */
        data->treat_types = (prc_graphics_information *)prc_calloc(ctx, data->number_of_treat_type, sizeof(prc_graphics_information));
        if (data->treat_types == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_context_graphics\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_treat_type; k++)
        {
            code = prc_parse_graphics_information(ctx, bit_state, &data->treat_types[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_graphics_information\n");
                return code;
            }
        }
    }
    return code;
}

/* Table 52 ExtraGeometry (Mislabeled in spec as ExactGeometry) */
int
prc_parse_extra_geometry(prc_context *ctx, prc_bit_state *bit_state, prc_extra_geometry *data)
{
    int code;

    code = prc_parse_geometry_summary(ctx, bit_state, &data->summary);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_geometry_summary\n");
        return code;
    }
    code = prc_parse_context_graphics(ctx, bit_state, &data->context_graphics);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_context_graphics\n");
    }
    return code;
}
