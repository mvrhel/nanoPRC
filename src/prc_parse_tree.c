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

#include <stdio.h>
#include <stdlib.h>
#include "prc_parse_tree.h"
#include "prc_parse_common.h"
#include "prc_schema.h"
#include "debug.h"
#include "prc_diag_env.h"

#define DEBUG_TREE 0

/* prc_parse_ri and prc_parse_ri_set are mutually recursive (a RI can wrap a
   nested RI, and an RI_Set contains an array of RIs, each of which can again
   be an RI_Set). Both nestings are attacker-controlled via the file, so cap
   the depth to avoid unbounded C-stack recursion. */
#define PRC_RI_RECURSION_MAX 256
/* prc_parse_mkp_annotation_set_notag self-recurses on nested annotation
   sets, also attacker-controlled nesting depth. */
#define PRC_MKP_ANNOTATION_SET_RECURSION_MAX 256

static int prc_parse_ri(prc_context *ctx, prc_bit_state *bit_state, prc_ri *data, int depth);

/* Table 117 */
static int
prc_parse_ri_brep_model(prc_context *ctx, prc_bit_state *bit_state, prc_ri_brep_model *data)
{
    int code;

    data->exact_geometry = prc_bitread_bit(ctx, bit_state);
    if (data->exact_geometry)
    {
        data->index_topological_context = prc_bitread_uint32(ctx, bit_state);
        data->index_body = prc_bitread_uint32(ctx, bit_state);
    }
    data->is_closed = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    return 0;
}

/* Table 118 */
static int
prc_parse_ri_curve(prc_context *ctx, prc_bit_state *bit_state, prc_ri_curve *data)
{
    int code;

    data->exact_geometry = prc_bitread_bit(ctx, bit_state);
    if (data->exact_geometry)
    {
        data->index_topological_context = prc_bitread_uint32(ctx, bit_state);
        data->index_body = prc_bitread_uint32(ctx, bit_state);
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 119 */
static int
prc_parse_ri_direction(prc_context *ctx, prc_bit_state *bit_state, prc_ri_direction *data)
{
    int code;

    /* The spec does not include the curve, but it is there */
    code = prc_parse_ri_curve(ctx, bit_state, &data->curve);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_ri_curve\n");
        return code;
    }

    data->has_origin = prc_bitread_bit(ctx, bit_state);
    if (data->has_origin)
    {
        data->origin = prc_parse_3d_vector(ctx, bit_state);
    }
    data->direction = prc_parse_3d_vector(ctx, bit_state);

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 120 */
static int
prc_parse_ri_plane(prc_context *ctx, prc_bit_state *bit_state, prc_ri_plane *data)
{
    int code;

    data->exact_geometry = prc_bitread_bit(ctx, bit_state);
    if (data->exact_geometry)
    {
        data->index_topological_context = prc_bitread_uint32(ctx, bit_state);
        data->index_body = prc_bitread_uint32(ctx, bit_state);
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 121 */
static int
prc_parse_ri_point_set(prc_context *ctx, prc_bit_state *bit_state, prc_ri_point_set *data)
{
    int code;
    uint32_t k;

    data->number_of_points = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_points > 0)
    {
        data->points = (prc_vec3 *)prc_calloc(ctx, data->number_of_points, sizeof(prc_vec3));
        if (data->points == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ri_point_set_notag\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_points; k++)
        {
            data->points[k] = prc_parse_3d_vector(ctx, bit_state);
        }
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    return 0;
}

/* Table 122 */
static int
prc_parse_ri_poly_brep_model(prc_context *ctx, prc_bit_state *bit_state, prc_ri_poly_brep_model *data)
{
    int code;

    data->is_closed = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 123 */
static int
prc_parse_ri_poly_wire(prc_context *ctx, prc_bit_state *bit_state, prc_ri_poly_wire *data)
{
    int code;

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 124 */
static int
prc_parse_ri_set(prc_context *ctx, prc_bit_state *bit_state, prc_ri_set *data, int depth)
{
    uint32_t k;
    int code;

    data->number_of_items = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_items > 0)
    {
        data->rep_items = (prc_ri *)prc_calloc(ctx,
                                        data->number_of_items, sizeof(prc_ri));
        if (data->rep_items == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_ri_set\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_items; k++)
        {
            code = prc_parse_ri(ctx, bit_state, &data->rep_items[k], depth + 1);
            if (code < 0)
            {
                prc_error(ctx, code, "Parsing error in prc_parse_ri_set\n");
                return code;
            }
        }
    }
    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 125 */
static int
prc_parse_ri_coordinate_system(prc_context *ctx, prc_bit_state *bit_state, prc_ri_coordinate_system *data)
{
    int code;

    /* Get the tag for the transformation type */
    data->transform.transformation_type = prc_bitread_uint32(ctx, bit_state);

    if (data->transform.transformation_type == PRC_TYPE_MISC_CartesianTransformation)
    {
        prc_parse_3d_transform(ctx, bit_state, &data->transform.cartesian);
    }
    else if (data->transform.transformation_type == PRC_TYPE_MISC_GeneralTransformation)
    {
        prc_parse_general_transform(ctx, bit_state, &data->transform.general_transformation);
    }
    else
    {
        return PRC_ERROR_PARSE;
    }
    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* TODO prc_execute_schema_instruction on these... */
static int
prc_parse_ri(prc_context *ctx, prc_bit_state *bit_state, prc_ri *data, int depth)
{
    int code;

    /* prc_parse_ri and prc_parse_ri_set are mutually recursive on file-
       controlled nesting (a nested RepresentationalItem, or an RI_Set whose
       items can themselves be RI_Sets); cap the depth instead of letting it
       grow unbounded. */
    if (depth > PRC_RI_RECURSION_MAX)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "prc_parse_ri recursion depth exceeded\n");
        return PRC_ERROR_PARSE;
    }

    /* Read the tag to see what we have */
    data->representation_type = prc_bitread_uint32(ctx, bit_state);

    /* These all have representation item content types */
    code = prc_parse_representation_item_content(ctx, bit_state, &data->item_content);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_representation_item_content\n");
        return code;
    }

#if DEBUG_TREE
    DEBUG_LOG("RepresentationItem: representation_type=%u biased_index_local_coordinate_system=%u "
        "biased_index_tessellation=%u biased_index_of_line_style=%u name.same=%u name=\"%s\"\n",
        data->representation_type, data->item_content.biased_index_local_coordinate_system,
        data->item_content.biased_index_tessellation,
        data->item_content.base.graphics_content.biased_index_of_line_style,
        data->item_content.base.base.name.same,
        data->item_content.base.base.name.same ? "" : (data->item_content.base.base.name.name.string ? data->item_content.base.base.name.name.string : "(null)"));
#endif

    switch (data->representation_type)
    {
    case PRC_TYPE_RI_RepresentationalItem:
        /* Hmm this is getting recursive if we end up here */
        data->ri_rep_item = (prc_ri*)prc_malloc(ctx, sizeof(prc_ri));
        if (data->ri_rep_item == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_ri(ctx, bit_state, data->ri_rep_item, depth + 1);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri\n");
            return code;
        }

        break;
    case PRC_TYPE_RI_BrepModel:
        data->ri_brep_model = (prc_ri_brep_model*)prc_malloc(ctx, sizeof(prc_ri_brep_model));
        if (data->ri_brep_model == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_brep_model->tag = data->representation_type;
        data->ri_brep_model->item_content = data->item_content;
        code = prc_parse_ri_brep_model(ctx, bit_state, data->ri_brep_model);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_ri_brep_model\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_Curve:
        data->ri_curve = (prc_ri_curve*)prc_malloc(ctx, sizeof(prc_ri_curve));
        if (data->ri_curve == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_curve->tag = data->representation_type;
        data->ri_curve->item_content = data->item_content;
        code = prc_parse_ri_curve(ctx, bit_state, data->ri_curve);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_ri_curve\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_Direction:
        data->ri_direction = (prc_ri_direction*)prc_malloc(ctx, sizeof(prc_ri_direction));
        if (data->ri_direction == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_direction->tag = data->representation_type;
        data->ri_direction->item_content = data->item_content;
        code = prc_parse_ri_direction(ctx, bit_state, data->ri_direction);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_ri_direction\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_Plane:
        data->ri_plane = (prc_ri_plane*)prc_malloc(ctx, sizeof(prc_ri_plane));
        if (data->ri_plane == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Memory error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_plane->tag = data->representation_type;
        data->ri_plane->item_content = data->item_content;
        code = prc_parse_ri_plane(ctx, bit_state, data->ri_plane);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_ri_plane\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_PointSet:
        data->ri_point_set = (prc_ri_point_set*)prc_malloc(ctx, sizeof(prc_ri_point_set));
        if (data->ri_point_set == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_point_set->tag = data->representation_type;
        data->ri_point_set->item_content = data->item_content;
        code = prc_parse_ri_point_set(ctx, bit_state, data->ri_point_set);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_PolyBrepModel:
        data->ri_poly_brep_model = (prc_ri_poly_brep_model*)prc_malloc(ctx, sizeof(prc_ri_poly_brep_model));
        if (data->ri_poly_brep_model == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_poly_brep_model->tag = data->representation_type;
        data->ri_poly_brep_model->item_content = data->item_content;
        code = prc_parse_ri_poly_brep_model(ctx, bit_state, data->ri_poly_brep_model);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri_poly_brep_model\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_PolyWire:
        data->ri_poly_wire = (prc_ri_poly_wire*)prc_malloc(ctx, sizeof(prc_ri_poly_wire));
        if (data->ri_poly_wire == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_poly_wire->tag = data->representation_type;
        data->ri_poly_wire->item_content = data->item_content;
        code = prc_parse_ri_poly_wire(ctx, bit_state, data->ri_poly_wire);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri_poly_wire\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_Set:
        data->ri_set = (prc_ri_set*)prc_malloc(ctx, sizeof(prc_ri_set));
        if (data->ri_set == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_set->tag = data->representation_type;
        data->ri_set->item_content = data->item_content;
        code = prc_parse_ri_set(ctx, bit_state, data->ri_set, depth + 1);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri_set\n");
            return code;
        }
        break;
    case PRC_TYPE_RI_CoordinateSystem:
        data->ri_coordinate_system = (prc_ri_coordinate_system*)prc_malloc(ctx, sizeof(prc_ri_coordinate_system));
        if (data->ri_coordinate_system == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Parsing error in prc_parse_ri\n");
            return PRC_ERROR_MEMORY;
        }
        data->ri_coordinate_system->tag = data->representation_type;
        data->ri_coordinate_system->item_content = data->item_content;
        code = prc_parse_ri_coordinate_system(ctx, bit_state, data->ri_coordinate_system);
        if (code < 0)
        {
            prc_error(ctx, code, "Parsing error in prc_parse_ri_coordinate_system\n");
            return code;
        }
        break;
    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_ri\n");
        return 0; /* Swallow it.  Many files with issues here */
    }
    return 0;
}

/* Table 128  This is really a PRC_TYPE_MISC_ReferenceOnPRCBase (Table 80 in updated spec) */
static int
prc_parse_reference_unique_identifier(prc_context *ctx, prc_bit_state *bit_state, prc_misc_reference_on_prcbase *data)
{
    int code = 0;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_MISC_ReferenceOnPRCBase, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data->type_of_entity = prc_bitread_uint32(ctx, bit_state);
    data->flag = prc_bitread_bit(ctx, bit_state);

    if (!data->flag) /* Note original spec was wrong on this */
    {
        data->different_unique_id = prc_get_compressed_unique_id(ctx, bit_state);
    }

    data->unique_id = prc_bitread_uint32(ctx, bit_state);

    return code;
}

/* Table 132 */
static int
prc_parse_mkp_leader(prc_context *ctx, prc_bit_state *bit_state, prc_mkp_leader *data)
{
    int code = 0;
    uint32_t type;

    type = prc_bitread_uint32(ctx, bit_state);
    if (type != PRC_TYPE_MKP_Leader)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_mkp_leader\n");
        return PRC_ERROR_PARSE;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_base_with_graphics\n");
    }

    data->is_first_linked_item = prc_bitread_bit(ctx, bit_state);

    if (data->is_first_linked_item)
    {
        code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->first_linked_item);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifier\n");
            return code;
        }
    }

    data->is_second_linked_item = prc_bitread_bit(ctx, bit_state);
    if (data->is_second_linked_item)
    {
        code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->second_linked_item);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifier\n");
            return code;
        }
    }

    data->biased_index_tessallation = prc_bitread_uint32(ctx, bit_state);

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);

    return code;
}

/* Table 82 */
static int
prc_parse_additional_target_data(prc_context *ctx, prc_bit_state *bit_state, prc_additional_target_data *data)
{
    int code = 0;
    uint32_t k;

    data->flag = prc_bitread_bit(ctx, bit_state);

    if (!data->flag)
    {
        data->unique_id = prc_get_compressed_unique_id(ctx, bit_state);
    }
    data->index_of_topological_index = prc_bitread_uint32(ctx, bit_state);
    data->index_of_body = prc_bitread_uint32(ctx, bit_state);
    data->number_of_indices = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_indices > 0)
    {
        data->indices = (prc_unsigned_int *)prc_calloc(ctx, data->number_of_indices, sizeof(prc_unsigned_int));
        if (data->indices == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_additional_target_data\n");
            return PRC_ERROR_MEMORY;
        }

        /* This is the face indice */
        for (k = 0; k < data->number_of_indices; k++)
        {
            data->indices[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    return code;
}

/* Table 81 */
static int
prc_parse_misc_reference_on_topology_no_tag(prc_context *ctx, prc_bit_state *bit_state, prc_misc_reference_on_topology *data)
{
    int code = 0;

    data->type = prc_bitread_uint32(ctx, bit_state);  /* Should be PRC_TYPE_TOPO_Face (149) */
    data->flag = prc_bitread_bit(ctx, bit_state);

    if (data->flag)
    {
        code = prc_parse_additional_target_data(ctx, bit_state, &data->data);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_additional_target_data\n");
            return PRC_ERROR_PARSE;
        }
    }
    return code;
}

/* Table 80 */
static int
prc_parse_misc_reference_on_prcbase_no_tag(prc_context *ctx, prc_bit_state *bit_state, prc_misc_reference_on_prcbase *data)
{
    data->type_of_entity = prc_bitread_uint32(ctx, bit_state);
    data->flag = prc_bitread_bit(ctx, bit_state);
    if (!data->flag)
    {
        data->different_unique_id = prc_get_compressed_unique_id(ctx, bit_state);
    }
    data->unique_id = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 85 */
static int
prc_parse_reference_data(prc_context *ctx, prc_bit_state *bit_state, prc_reference_data *data)
{
    int code;

    /* Read in the tag first to decide which one we have */
    data->ref_type = prc_bitread_uint32(ctx, bit_state);

    if (data->ref_type == PRC_TYPE_MISC_ReferenceOnTopology)
    {
        data->topo_reference.tag = data->ref_type;
        code = prc_parse_misc_reference_on_topology_no_tag(ctx, bit_state, &data->topo_reference);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_misc_reference_on_topology_no_tag\n");
            return PRC_ERROR_PARSE;
        }
    }
    else if (data->ref_type == PRC_TYPE_MISC_ReferenceOnPRCBase)
    {
        data->non_topo_reference.tag = data->ref_type;
        code = prc_parse_misc_reference_on_prcbase_no_tag(ctx, bit_state, &data->non_topo_reference);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_misc_reference_on_prcbase_no_tag\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_reference_data\n");
        return PRC_ERROR_PARSE;
    }
    return code;
}

/* Table 84 ContentEntityReference */
static int
prc_parse_content_entity_reference(prc_context *ctx, prc_bit_state *bit_state, prc_content_entity_reference *data)
{
    int code = 0;

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_base_with_graphics\n");
        return PRC_ERROR_PARSE;
    }

    data->index_of_local_coordinate = prc_bitread_uint32(ctx, bit_state);
    data->flag = prc_bitread_bit(ctx, bit_state);
    if (data->flag)
    {
        code = prc_parse_reference_data(ctx, bit_state, &data->reference_data);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_reference_data\n");
            return PRC_ERROR_PARSE;
        }
    }
    return code;
}

/* Table 79 */
static int
prc_parse_content_extended_entity_reference(prc_context *ctx, prc_bit_state *bit_state, prc_content_extended_entity_ref *data)
{
    int code = 0;

    code = prc_parse_content_entity_reference(ctx, bit_state, &data->content_entity_ref);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_content_entity_reference\n");
        return PRC_ERROR_PARSE;
    }

    data->has_reference_data = prc_bitread_bit(ctx, bit_state);
    if (data->has_reference_data)
    {
        code = prc_parse_reference_data(ctx, bit_state, &data->reference_data);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_reference_data\n");
            return PRC_ERROR_PARSE;
        }
    }
    return code;
}

/* Table 78 */
static int
prc_parse_misc_markup_linked_item(prc_context *ctx, prc_bit_state *bit_state, prc_misc_markup_linked_item *data)
{
    int code = 0;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_MISC_MarkupLinkedItem, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_extended_entity_reference(ctx, bit_state, &data->content_entity_ref);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_content_extended_reference\n");
        return PRC_ERROR_PARSE;
    }

    data->show_markup = prc_bitread_bit(ctx, bit_state);
    data->delete_markup = prc_bitread_bit(ctx, bit_state);
    data->show_leader = prc_bitread_bit(ctx, bit_state);
    data->delete_leader = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);

    return code;
}

/* Table 133 */
static int
prc_parse_mkp_annotation_item_notag(prc_context *ctx, prc_bit_state *bit_state, prc_type_mkp_annotation_item *data)
{
    int code;

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_base_with_graphics\n");
        return PRC_ERROR_PARSE;
    }

    code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->unique_id);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_reference_unique_identifier\n");
        return PRC_ERROR_PARSE;
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_user_data\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 135 */
static int
prc_parse_mkp_annotation_ref_notag(prc_context *ctx, prc_bit_state *bit_state, prc_type_mkp_annotation_reference *data)
{
    int code;
    uint32_t k;

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_base_with_graphics\n");
        return PRC_ERROR_PARSE;
    }

    data->number_of_linked_items = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_linked_items > 0)
    {
        data->linked_items = (prc_misc_reference_on_prcbase *)prc_calloc(ctx, data->number_of_linked_items, sizeof(prc_misc_reference_on_prcbase));
        if (data->linked_items == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_annotation_ref_notag\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_linked_items; k++)
        {
            code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->linked_items[k]);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_reference_unique_identifier\n");
                return PRC_ERROR_PARSE;
            }
        }
    }
    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    return code;
}

/* Table 134 */
static int
prc_parse_mkp_annotation_set_notag(prc_context *ctx, prc_bit_state *bit_state,
    prc_type_mkp_annotation_set *data, int depth)
{
    int code;
    uint32_t k;

    /* Nested annotation sets are attacker-controlled via the file; cap the
       depth instead of letting it grow unbounded. */
    if (depth > PRC_MKP_ANNOTATION_SET_RECURSION_MAX)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "prc_parse_mkp_annotation_set_notag recursion depth exceeded\n");
        return PRC_ERROR_PARSE;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_base_with_graphics\n");
        return PRC_ERROR_PARSE;
    }

    data->number_of_annotations = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_annotations > 0)
    {
        data->annotations = (prc_annotation_entity *)prc_calloc(ctx, data->number_of_annotations, sizeof(prc_annotation_entity));
        if (data->annotations == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_annotation_set_notag\n");
            return PRC_ERROR_MEMORY;
        }

        /* This is crazy as we have annotations in annotations possibly. I guess that is what an annotation set is */
        for (k = 0; k < data->number_of_annotations; k++)
        {
            /* Get the annotation type */
            prc_unsigned_int tag = prc_bitread_uint32(ctx, bit_state);
            data->annotations[k].tag = tag;

            switch (tag)
            {
            case PRC_TYPE_MKP_AnnotationItem:
                data->annotations[k].item.tag = tag;
                code = prc_parse_mkp_annotation_item_notag(ctx, bit_state, &data->annotations[k].item);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_item_notag\n");
                    return code;
                }
                break;

            case PRC_TYPE_MKP_AnnotationSet:
                data->annotations[k].set.tag = tag;
                code = prc_parse_mkp_annotation_set_notag(ctx, bit_state, &data->annotations[k].set, depth + 1);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_set_notag\n");
                    return code;
                }
                break;

            case PRC_TYPE_MKP_AnnotationReference:
                data->annotations[k].ref.tag = tag;
                code = prc_parse_mkp_annotation_ref_notag(ctx, bit_state, &data->annotations[k].ref);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_ref_notag\n");
                    return code;
                }
                break;

            default:
                prc_error(ctx, code, "Parsing error in prc_parse_markup_data\n");
                return code;
            }
        }
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_user_data\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 131 */
static int
prc_parse_mkp_markups(prc_context *ctx, prc_bit_state *bit_state, prc_mkp_markup *data)
{
    int code;
    uint32_t k;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_MKP_Markup, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_base_with_graphics\n");
        return PRC_ERROR_PARSE;
    }

    data->markup_type = prc_bitread_uint32(ctx, bit_state);
    data->markup_subtype = prc_bitread_uint32(ctx, bit_state);
    data->number_of_linked_items = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_linked_items > 0)
    {
        data->linked_items = (prc_misc_reference_on_prcbase *)prc_calloc(ctx, data->number_of_linked_items, sizeof(prc_misc_reference_on_prcbase));
        if (data->linked_items == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_markups\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_linked_items; k++)
        {
            code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->linked_items[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifiers\n");
                return code;
            }
        }
    }

    data->number_of_leaders = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_leaders > 0)
    {
        data->leaders = (prc_misc_reference_on_prcbase *)prc_calloc(ctx, data->number_of_leaders, sizeof(prc_misc_reference_on_prcbase));
        if (data->leaders == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_markups\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_leaders; k++)
        {
            code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->leaders[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifiers\n");
                return code;
            }
        }
    }
    data->biased_index_tessellation = prc_bitread_uint32(ctx, bit_state);
    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_user_data\n");
        return code;
    }

    return code;
}

/* Table 65 */
static int
prc_parse_markup_data(prc_context *ctx, prc_bit_state *bit_state, prc_markup_data *data)
{
    int code = 0;
    uint32_t k;

    data->number_of_linked_items = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_linked_items > 0)
    {
        data->linked_items = (prc_misc_markup_linked_item *)prc_calloc(ctx, data->number_of_linked_items, sizeof(prc_misc_markup_linked_item));
        if (data->linked_items == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_markup_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_linked_items; k++)
        {
            code = prc_parse_misc_markup_linked_item(ctx, bit_state, &data->linked_items[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_misc_markup_linked_item\n");
                return code;
            }
        }
    }

    data->number_of_leaders = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_leaders > 0)
    {
        data->leaders = (prc_mkp_leader *)prc_calloc(ctx, data->number_of_leaders, sizeof(prc_mkp_leader));
        if (data->leaders == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_markup_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_leaders; k++)
        {
            code = prc_parse_mkp_leader(ctx, bit_state, &data->leaders[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_mkp_leader\n");
                return code;
            }
        }
    }

    data->number_of_markups = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_markups > 0)
    {
        data->markups = (prc_mkp_markup *)prc_calloc(ctx, data->number_of_markups, sizeof(prc_mkp_markup));
        if (data->markups == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_markup_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_markups; k++)
        {
            code = prc_parse_mkp_markups(ctx, bit_state, &data->markups[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_mkp_markups\n");
                return code;
            }
        }
    }

    data->number_of_annotation_entities = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_annotation_entities != 0)
    {
        data->annotation_entities = (prc_annotation_entity *)prc_calloc(ctx, data->number_of_annotation_entities, sizeof(prc_annotation_entity));
        if (data->annotation_entities == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_markup_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_annotation_entities; k++)
        {
            /* Grab the tag */
            data->annotation_entities[k].tag = prc_bitread_uint32(ctx, bit_state);
            switch (data->annotation_entities[k].tag)
            {
            case PRC_TYPE_MKP_AnnotationItem:
                data->annotation_entities[k].item.tag = data->annotation_entities[k].tag;
                code = prc_parse_mkp_annotation_item_notag(ctx, bit_state, &data->annotation_entities[k].item);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_item_notag\n");
                    return code;
                }
                break;

            case PRC_TYPE_MKP_AnnotationSet:
                data->annotation_entities[k].set.tag = data->annotation_entities[k].tag;
                code = prc_parse_mkp_annotation_set_notag(ctx, bit_state, &data->annotation_entities[k].set, 0);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_set_notag\n");
                    return code;
                }
                break;

            case PRC_TYPE_MKP_AnnotationReference:
                data->annotation_entities[k].ref.tag = data->annotation_entities[k].tag;
                code = prc_parse_mkp_annotation_ref_notag(ctx, bit_state, &data->annotation_entities[k].ref);
                if (code < 0)
                {
                    prc_error(ctx, code, "Parsing error in prc_parse_mkp_annotation_ref_notag\n");
                    return code;
                }
                break;

            default:
                prc_error(ctx, code, "Parsing error in prc_parse_markup_data\n");
                return 0; /* Swallow it */
            }
        }
    }

    return 0;
}

/* Table 285 ContentSurface */
static int
prc_parse_content_surface(prc_context *ctx, prc_bit_state *bit_state, prc_content_surface *data)
{
    int code;

    data->has_base_geometry = prc_bitread_bit(ctx, bit_state);

    if (data->has_base_geometry)
    {
        code = prc_parse_attribute_data(ctx, bit_state, &data->attribute_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_attribute_data\n");
            return code;
        }

        code = prc_parse_name(ctx, bit_state, &data->name);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_name\n");
            return code;
        }

        data->id = prc_bitread_uint32(ctx, bit_state);
    }
    data->extension_type = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* Table 114 PRC_TYPE_GRAPH_Camera */
static int
prc_parse_graph_camera(prc_context *ctx, prc_bit_state *bit_state, prc_graph_camera *data)
{
    int code = 0;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_Camera, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->is_orthographic = prc_bitread_bit(ctx, bit_state);
    data->position = prc_parse_3d_vector(ctx, bit_state);
    data->look = prc_parse_3d_vector(ctx, bit_state);
    data->up = prc_parse_3d_vector(ctx, bit_state);

    data->x = prc_bitread_double(ctx, bit_state);
    data->y = prc_bitread_double(ctx, bit_state);
    data->ratio = prc_bitread_double(ctx, bit_state);
    data->clip_near = prc_bitread_double(ctx, bit_state);
    data->clip_far = prc_bitread_double(ctx, bit_state);
    data->zoom = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Tables 109 - 112 light object abstract type */
static int
prc_parse_graph_light_object(prc_context *ctx, prc_bit_state *bit_state, prc_graph_light_object *data)
{
    int code;

    data->tag = prc_bitread_uint32(ctx, bit_state);
    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_prc_ref_base\n");
        return code;
    }
    data->biased_ambiant_index = prc_bitread_uint32(ctx, bit_state);
    data->biased_diffuse_index = prc_bitread_uint32(ctx, bit_state);
    data->biased_emmissive_index = prc_bitread_uint32(ctx, bit_state);
    data->biased_specular_index = prc_bitread_uint32(ctx, bit_state);

    switch (data->tag)
    {
    case PRC_TYPE_GRAPH_AmbientLight:
        return 0;

    case PRC_TYPE_GRAPH_PointLight:
        data->location = prc_parse_3d_vector(ctx, bit_state);
        data->constant_attenuation_factor = prc_bitread_double(ctx, bit_state);
        data->linear_attenuation_factor = prc_bitread_double(ctx, bit_state);
        data->quadratic_attenuation_factor = prc_bitread_double(ctx, bit_state);
        break;

    case PRC_TYPE_GRAPH_DirectionalLight:
        data->direction = prc_parse_3d_vector(ctx, bit_state);
        /* VERSION GATE (ISO/CD 14739-1, PRC_TYPE_GRAPH_DirectionalLight):
           `intensity` is present only when the file's authoring version is
           >= 8030. The authoring version is the SOLE mechanism for detecting
           its presence -- there is no secondary flag -- and a reader meeting
           an older file "shall treat intensity as 1.0 and shall not attempt
           to read intensity from the bitstream".

           Reading it unconditionally (this parser's behaviour until now) does
           not merely produce a wrong intensity on a pre-8030 file: it consumes
           8 bytes that are not there and desyncs the bitstream for everything
           parsed after the light. PRC_TYPE_GRAPH_PointLight has no intensity
           field at all in any version, so the gate belongs here only.

           ctx->source_file_version carries the file's own authoring_version
           (set in prc_parse_main.c from the FileStructure header), the same
           field the >= 7046 / 7309 / 8016 gates elsewhere in this file use. */
        if (ctx->source_file_version >= 8030)
            data->intensity = prc_bitread_double(ctx, bit_state);
        else
            data->intensity = 1.0;
        break;

    case PRC_TYPE_GRAPH_SpotLight:
        data->location = prc_parse_3d_vector(ctx, bit_state);
        data->constant_attenuation_factor = prc_bitread_double(ctx, bit_state);
        data->linear_attenuation_factor = prc_bitread_double(ctx, bit_state);
        data->quadratic_attenuation_factor = prc_bitread_double(ctx, bit_state);
        data->direction = prc_parse_3d_vector(ctx, bit_state);
        data->fall_off_angle = prc_bitread_double(ctx, bit_state);
        data->fall_off_exponenent = prc_bitread_double(ctx, bit_state);
        break;

    default:
        return PRC_ERROR_PARSE;
    }

    return 0;
}

/* Table 113 PRC_TYPE_GRAPH_SceneDisplayParameters */
static int
prc_parse_scene_display_parameters(prc_context *ctx, prc_bit_state *bit_state, prc_scene_display_parameters *data)
{
    int code = 0;
    uint32_t k;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_SceneDisplayParameters, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->is_active = prc_bitread_bit(ctx, bit_state);

    data->number_of_lights = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_lights > 0)
    {
        data->lights = (prc_graph_light_object *)prc_calloc(ctx, data->number_of_lights, sizeof(prc_graph_light_object));
        if (data->lights == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_scene_display_parameters\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_lights; k++)
        {
            code = prc_parse_graph_light_object(ctx, bit_state, &data->lights[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_graph_light_object\n");
                return code;
            }
        }
    }

    data->camera_defined = prc_bitread_bit(ctx, bit_state);
    if (data->camera_defined)
    {
        code = prc_parse_graph_camera(ctx, bit_state, &data->camera);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_graph_camera\n");
            return code;
        }
    }

    data->rotation_center_defined = prc_bitread_bit(ctx, bit_state);
    if (data->rotation_center_defined)
    {
        data->rotation_center = prc_parse_3d_vector(ctx, bit_state);
    }

    data->number_of_clipping_planes = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_clipping_planes > 0)
    {
        data->clipping_planes = (prc_surf_plane *)prc_calloc(ctx, data->number_of_clipping_planes, sizeof(prc_surf_plane));
        if (data->clipping_planes == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_scene_display_parameters\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_clipping_planes; k++)
        {
            /* CHECK IF THIS SHOULD BE READING SURF_PTR TYPE */
            code = prc_parse_surf_plane(ctx, bit_state, &data->clipping_planes[k], READ_TAG);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_graph_light_object\n");
                return code;
            }
        }
    }

    data->index_of_line_style_background = prc_bitread_uint32(ctx, bit_state);
    data->index_of_line_style_default = prc_bitread_uint32(ctx, bit_state);
    data->number_default_styles = prc_bitread_uint32(ctx, bit_state);
    if (data->number_default_styles > 0)
    {
        data->styles = (prc_unsigned_int *)prc_calloc(ctx, data->number_default_styles, sizeof(prc_unsigned_int) * 2);
        if (data->styles == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_scene_display_parameters\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_default_styles * 2; k++)
        {
            data->styles[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->is_absolute = prc_bitread_bit(ctx, bit_state);

    return 0;
}

/* Table 68 ContentLayerFilterItems */
static int
prc_parse_content_layer_filter_item(prc_context *ctx, prc_bit_state *bit_state, prc_content_layer_filter_item *data)
{
    uint32_t k;

    data->b_is_inclusive = prc_bitread_bit(ctx, bit_state);
    data->number_of_layers = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_layers > 0)
    {
        data->layers = (prc_unsigned_int *)prc_calloc(ctx, data->number_of_layers, sizeof(prc_unsigned_int));
        if (data->layers == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_layer_filter_items\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_layers; k++)
        {
            data->layers[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    return 0;
}

/* Table 77 PRC_TYPE_MISC_EntityReference */
static int
prc_parse_misc_entity_reference(prc_context *ctx, prc_bit_state *bit_state, prc_misc_entity_reference *data)
{
    int code;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_MISC_EntityReference, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_entity_reference(ctx, bit_state, &data->content_entity_ref);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_entity_reference\n");
        return code;
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    return 0;
}

/* Table 69 ContentEntityFilterItems */
static int
prc_parse_content_entity_filter_items(prc_context *ctx, prc_bit_state *bit_state, prc_content_entity_filter_item *data)
{
    uint32_t k;
    int code;

    data->b_is_inclusive = prc_bitread_bit(ctx, bit_state);
    data->number_of_entities = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_entities > 0)
    {
        data->entities = (prc_misc_entity_reference *)prc_calloc(ctx, data->number_of_entities, sizeof(prc_misc_entity_reference));
        if (data->entities == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_entity_filter_items\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->number_of_entities; k++)
        {
            code = prc_parse_misc_entity_reference(ctx, bit_state, &data->entities[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_misc_entity_reference\n");
                return code;
            }
        }
    }

    return 0;
}


/* Table 67 PRC_TYPE_ASM_Filter */
static int
prc_parse_asm_filter(prc_context *ctx, prc_bit_state *bit_state, prc_asm_filter *data)
{
    int code = 0;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_ASM_Filter, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_prc_base\n");
        return code;
    }

    data->is_active = prc_bitread_bit(ctx, bit_state);

    code = prc_parse_content_layer_filter_item(ctx, bit_state, &data->layer_filter);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_layer_filter_items\n");
        return code;
    }

    code = prc_parse_content_entity_filter_items(ctx, bit_state, &data->entity_filter);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_layer_filter_items\n");
        return code;
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }
    return 0;
}

/* Table 127 PRC_TYPE_MKP_View */
static int
prc_parse_mkp_view(prc_context *ctx, prc_bit_state *bit_state, prc_mkp_view *data)
{
    int code = 0;
    uint32_t k;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_MKP_View, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->number_of_annotations = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_annotations > 0)
    {
        data->annotations = (prc_misc_reference_on_prcbase *)prc_calloc(ctx, data->number_of_annotations, sizeof(prc_misc_reference_on_prcbase));
        if (data->annotations == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_view\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_annotations; k++)
        {
            code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->annotations[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifier\n");
                return code;
            }
        }
    }

    /* CHECK IF THIS SHOULD BE READING SURF_PTR TYPE -- Verified it should not */
    code = prc_parse_surf_plane(ctx, bit_state, &data->annotation_plane, READ_TAG);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_surf_plane\n");
        return code;
    }

    if (ctx->source_file_version >= 7046)
    {
        data->has_parameters = prc_bitread_bit(ctx, bit_state);
        if (data->has_parameters)
        {
            code = prc_parse_scene_display_parameters(ctx, bit_state, &data->scene_display_parameters);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_scene_display_parameters\n");
                return code;
            }
        }
    }

    if (ctx->source_file_version >= 7309)
    {
        data->is_annotation_view = prc_bitread_bit(ctx, bit_state);
    }

    if (ctx->source_file_version >= 8016)
    {
        data->is_default_view = prc_bitread_bit(ctx, bit_state);
        data->is_direction = prc_bitread_bit(ctx, bit_state);

        data->number_of_linked_items = prc_bitread_uint32(ctx, bit_state);
        if (data->number_of_linked_items > 0)
        {
            data->linked_items = (prc_misc_reference_on_prcbase *)prc_calloc(ctx, data->number_of_linked_items, sizeof(prc_misc_reference_on_prcbase));
            if (data->linked_items == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_view\n");
                return PRC_ERROR_MEMORY;
            }

            for (k = 0; k < data->number_of_linked_items; k++)
            {
                code = prc_parse_reference_unique_identifier(ctx, bit_state, &data->linked_items[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_parse_reference_unique_identifier\n");
                    return code;
                }
            }
        }

        data->number_of_filters = prc_bitread_uint32(ctx, bit_state);
        if (data->number_of_filters > 0)
        {
            data->filters = (prc_asm_filter *)prc_calloc(ctx, data->number_of_filters, sizeof(prc_asm_filter));
            if (data->filters == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_mkp_view\n");
                return PRC_ERROR_MEMORY;
            }

            for (k = 0; k < data->number_of_filters; k++)
            {
                code = prc_parse_asm_filter(ctx, bit_state, &data->filters[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_parse_asm_filter\n");
                    return code;
                }
            }
        }
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    /* Need to check if PRC_TYPE_MKP_View is in the schema */
    code = prc_check_for_schema(ctx, PRC_TYPE_MKP_View);
    if (code > 0)
    {
        int recursion = 0;
        code = prc_execute_schema(ctx, bit_state, prc_get_schema(ctx, code - 1), &recursion);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_execute_schema\n");
            return code;
        }
    }

    return 0;
}

/* Table 62 */
static int
prc_parse_product_information(prc_context *ctx, prc_bit_state *bit_state, prc_product_information *data)
{
    data->product_information_flags = prc_bitread_bit(ctx, bit_state);
    data->unit = prc_bitread_double(ctx, bit_state);
    data->product_information_flags = prc_bitread_uint8(ctx, bit_state);
    data->product_load_status = prc_bitread_int32(ctx, bit_state);
    return 0;
}

/* Table 60 ReferencesOfProductOccurrence */
static int
prc_parse_references_of_product_occurrence(prc_context *ctx, prc_bit_state *bit_state, prc_references_of_product_occurrence *data)
{
    uint32_t k;

    data->biased_index_part = prc_bitread_uint32(ctx, bit_state);
    data->biased_index_prototype = prc_bitread_uint32(ctx, bit_state);
    if (data->biased_index_prototype != 0)
    {
        prc_parse_file_indentifier(ctx, bit_state, &data->prototype_in_same_file_structure);
    }
    data->biased_index_external_data = prc_bitread_uint32(ctx, bit_state);
    if (data->biased_index_external_data != 0)
    {
        prc_parse_file_indentifier(ctx, bit_state, &data->external_data_in_same_file_structure);
    }
    /* The above file indentifiers are used to search through the other files and get the appropriate parts. */

    data->number_of_child_product_occurrences = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_child_product_occurrences > 0)
    {
        data->index_child_occurrence = (prc_unsigned_int *)prc_calloc(ctx,
            data->number_of_child_product_occurrences, sizeof(prc_unsigned_int));
        if (data->index_child_occurrence == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_references_of_product_occurrence\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_child_product_occurrences; k++)
        {
            data->index_child_occurrence[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

#if DEBUG_TREE
    DEBUG_LOG2("*** biased_index_part = %u\n", data->biased_index_part);
    DEBUG_LOG2("         biased_index_prototype = %u\n", data->biased_index_prototype);
    if (data->biased_index_prototype != 0)
    {
        DEBUG_LOG2("        Entity exists in same FileStructure = %d\n", data->prototype_in_same_file_structure.flag);
        if (data->prototype_in_same_file_structure.flag == 0)
        {
            DEBUG_LOG2("         File id = [%u %u %u %u]\n",
                data->prototype_in_same_file_structure.unique_id.unique_id0,
                data->prototype_in_same_file_structure.unique_id.unique_id1,
                data->prototype_in_same_file_structure.unique_id.unique_id2,
                data->prototype_in_same_file_structure.unique_id.unique_id3);
        }
    }
    DEBUG_LOG2("         biased_index_external_data = %u\n", data->biased_index_external_data);
    if (data->biased_index_external_data != 0)
    {
        DEBUG_LOG2("         Entity exists in same FileStructure = %d\n", data->external_data_in_same_file_structure.flag);
        if (data->external_data_in_same_file_structure.flag == 0)
        {
            DEBUG_LOG2("         File id = [%u %u %u %u]\n",
                data->external_data_in_same_file_structure.unique_id.unique_id0,
                data->external_data_in_same_file_structure.unique_id.unique_id1,
                data->external_data_in_same_file_structure.unique_id.unique_id2,
                data->external_data_in_same_file_structure.unique_id.unique_id3);
        }
    }
    DEBUG_LOG2("         number_of_child_product_occurrences = %d\n", data->number_of_child_product_occurrences);
    for (k = 0; k < data->number_of_child_product_occurrences; k++)
    {
        DEBUG_LOG2("         index_child_occurrence[%d] = %d\n", k, data->index_child_occurrence[k]);
    }
#endif
    return 0;
}

/* Table 59 PRC_TYPE_ASM_ProductOccurrence */
int
prc_parse_product_occurrence(prc_context *ctx, prc_bit_state *bit_state, prc_asm_product_occurrence *data)
{
    int code;
    uint32_t k;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_ASM_ProductOccurrence, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_base_with_graphics\n");
        return code;
    }

    code = prc_parse_references_of_product_occurrence(ctx, bit_state, &data->references_product_occurrence);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_references_of_product_occurrence\n");
        return code;
    }

    data->product_behavior = prc_bitread_uint8(ctx, bit_state);

    code = prc_parse_product_information(ctx, bit_state, &data->product_information);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_product_information\n");
        return code;
    }

    data->has_transform = prc_bitread_bit(ctx, bit_state);
    if (data->has_transform)
    {
        /* This is ill-defined in the specification.  What type of transformation? */
        /* Apparently we are supposed to read the next byte to deteremine the type */
        code = prc_parse_misc_transform(ctx, bit_state, &data->location);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_misc_transform\n");
            return code;
        }
    }

    data->entity_ref_count = prc_bitread_uint32(ctx, bit_state);
    if (data->entity_ref_count > 0)
    {
        data->entity_reference = (prc_misc_entity_reference *)prc_calloc(ctx, data->entity_ref_count, sizeof(prc_misc_entity_reference));
        if (data->entity_reference == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_product_occurrence\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->entity_ref_count; k++)
        {
            code = prc_parse_misc_entity_reference(ctx, bit_state, &data->entity_reference[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_parse_entity_reference\n");
                return code;
            }
        }
    }

    code = prc_parse_markup_data(ctx, bit_state, &data->markups);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_markup_data\n");
        return code;
    }

    data->number_of_views = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_views > 0)
    {
        data->views = (prc_mkp_view *)prc_calloc(ctx, data->number_of_views, sizeof(prc_mkp_view));
        if (data->views == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_product_occurrence\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_views; k++)
        {
            code = prc_parse_mkp_view(ctx, bit_state, &data->views[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_parse_mkp_view\n");
                return code;
            }
        }
    }

    data->has_filter = prc_bitread_bit(ctx, bit_state);
    if (data->has_filter)
    {
        code = prc_parse_asm_filter(ctx, bit_state, &data->entity_filter);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_asm_filter\n");
            return code;
        }
    }

    data->number_of_display_filters = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_display_filters > 0)
    {
        data->display_filters = (prc_asm_filter *)prc_calloc(ctx, data->number_of_display_filters, sizeof(prc_asm_filter));
        if (data->display_filters == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_product_occurrence\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_display_filters; k++)
        {
            code = prc_parse_asm_filter(ctx, bit_state, &data->display_filters[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_parse_asm_filter\n");
                return code;
            }
        }
    }

    data->number_of_scene_parameters = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_scene_parameters > 0)
    {
        data->scene_display_parameters = (prc_scene_display_parameters *)prc_calloc(ctx, data->number_of_scene_parameters, sizeof(prc_scene_display_parameters));
        if (data->scene_display_parameters == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_product_occurrence\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_scene_parameters; k++)
        {
            code = prc_parse_scene_display_parameters(ctx, bit_state, &data->scene_display_parameters[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_parse_scene_display_parameters\n");
                return code;
            }
        }
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    data->style_inheritance.has_any_inheritance = 0;

#if DEBUG_TREE
    DEBUG_LOG("ProductOccurrence: attribute_count=%u name.same=%u name=\"%s\" "
        "biased_layer_index=%u biased_index_of_line_style=%u has_entity_ref=%u "
        "biased_index_part=%u biased_index_prototype=%u "
        "num_child_occ=%u product_behavior=%u has_transform=%u entity_ref_count=%u "
        "number_of_views=%u\n",
        data->base.base.attribute_data.attribute_count, data->base.base.name.same,
        data->base.base.name.same ? "" : (data->base.base.name.name.string ? data->base.base.name.name.string : "(null)"),
        data->base.graphics_content.biased_layer_index, data->base.graphics_content.biased_index_of_line_style,
        data->base.graphics_content.has_entity_ref,
        data->references_product_occurrence.biased_index_part, data->references_product_occurrence.biased_index_prototype,
        data->references_product_occurrence.number_of_child_product_occurrences,
        data->product_behavior, data->has_transform, data->entity_ref_count, data->number_of_views);
#endif

    if (prc_diag_getenv("PRC_DIAG_DUMP_TREE") != NULL)
    {
        uint32_t ci;
        fprintf(stderr, "PRC_DIAG_DUMP_TREE: ProductOccurrence unique_id=%u name=\"%s\" "
            "biased_layer_index=%u biased_index_of_line_style=%u behavior1=%u behavior2=%u "
            "biased_index_part=%u biased_index_prototype=%u num_child_occ=%u "
            "has_transform=%u entity_ref_count=%u number_of_views=%u children=[",
            data->base.base.unique_id, data->base.base.name.same ? "" :
                (data->base.base.name.name.string ? (const char *)data->base.base.name.name.string : "(null)"),
            data->base.graphics_content.biased_layer_index, data->base.graphics_content.biased_index_of_line_style,
            (unsigned)data->base.graphics_content.behavior_bit_field1, (unsigned)data->base.graphics_content.behavior_bit_field2,
            data->references_product_occurrence.biased_index_part, data->references_product_occurrence.biased_index_prototype,
            data->references_product_occurrence.number_of_child_product_occurrences,
            data->has_transform, data->entity_ref_count, data->number_of_views);
        for (ci = 0; ci < data->references_product_occurrence.number_of_child_product_occurrences; ci++)
            fprintf(stderr, "%s%u", ci ? "," : "", data->references_product_occurrence.index_child_occurrence[ci]);
        fprintf(stderr, "]\n");
    }

    return 0;
}

/* Table 67 PRC_TYPE_ASM_PartDefinition */
int
prc_parse_parts(prc_context *ctx, prc_bit_state *bit_state, prc_asm_parts_definition *data)
{
    int code;
    uint32_t k;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_ASM_PartDefinition, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_base_with_graphics(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_base_with_graphics\n");
        return code;
    }

    code = prc_parse_bound_box(ctx, bit_state, &data->bounding_box);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_bound_box\n");
        return code;
    }

    data->num_rep_items = prc_bitread_uint32(ctx, bit_state);

    /* A zero-volume bounding box on a PartDefinition -- most often seen on
       an otherwise-empty wrapper node (num_rep_items == 0) inserted purely
       for tree structure -- was confirmed real-Acrobat-causal for a
       silently blanked model tree beneath that node, even when the actual
       geometry a level or two further down is completely valid (see
       demos/stl_import/src/stl_import.c's has_empty_part fix). This is a
       READ-side warning only: nanoPRC's own decoder tolerates it fine (that
       tolerance is exactly why the bug went unnoticed for so long), so this
       does not fail the parse -- it just surfaces a real, previously-silent
       risk factor for anyone debugging a blank-tree report against a file
       from a different writer. */
    if (data->bounding_box.minimum_corner.x == data->bounding_box.maximum_corner.x &&
        data->bounding_box.minimum_corner.y == data->bounding_box.maximum_corner.y &&
        data->bounding_box.minimum_corner.z == data->bounding_box.maximum_corner.z)
    {
        fprintf(stderr, "Warning: PartDefinition (num_rep_items=%u) has a zero-volume "
            "bounding box (%.6f,%.6f,%.6f)-(%.6f,%.6f,%.6f) -- known to cause real Adobe "
            "Acrobat to silently blank the model tree beneath this node, even when its own "
            "geometry (or a descendant's) is otherwise valid.\n",
            data->num_rep_items,
            data->bounding_box.minimum_corner.x, data->bounding_box.minimum_corner.y,
            data->bounding_box.minimum_corner.z, data->bounding_box.maximum_corner.x,
            data->bounding_box.maximum_corner.y, data->bounding_box.maximum_corner.z);
    }

    if (data->num_rep_items > 0)
    {
        data->rep_items = (prc_ri *)prc_calloc(ctx, data->num_rep_items, sizeof(prc_ri));
        if (data->rep_items == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_parts\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->num_rep_items; k++)
        {
            code = prc_parse_ri(ctx, bit_state, &data->rep_items[k], 0);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_ri\n");
                return code;
            }
        }
    }

    code = prc_parse_markup_data(ctx, bit_state, &data->markups);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_markup_data\n");
        return code;
    }

    data->number_views = prc_bitread_uint32(ctx, bit_state);
    if (data->number_views > 0)
    {
        data->views = (prc_mkp_view *)prc_calloc(ctx, data->number_views, sizeof(prc_mkp_view));
        if (data->views == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_parts\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_views; k++)
        {
            code = prc_parse_mkp_view(ctx, bit_state, &data->views[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_mkp_view\n");
                return code;
            }
        }
    }

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    data->style_inheritance.has_any_inheritance = 0;

#if DEBUG_TREE
    DEBUG_LOG("PartDefinition: attribute_count=%u name.same=%u name=\"%s\" "
        "biased_layer_index=%u biased_index_of_line_style=%u has_entity_ref=%u "
        "bbox_min=(%f,%f,%f) bbox_max=(%f,%f,%f) num_rep_items=%u number_views=%u\n",
        data->base.base.attribute_data.attribute_count, data->base.base.name.same,
        data->base.base.name.same ? "" : (data->base.base.name.name.string ? data->base.base.name.name.string : "(null)"),
        data->base.graphics_content.biased_layer_index, data->base.graphics_content.biased_index_of_line_style,
        data->base.graphics_content.has_entity_ref,
        data->bounding_box.minimum_corner.x, data->bounding_box.minimum_corner.y, data->bounding_box.minimum_corner.z,
        data->bounding_box.maximum_corner.x, data->bounding_box.maximum_corner.y, data->bounding_box.maximum_corner.z,
        data->num_rep_items, data->number_views);
#endif

    if (prc_diag_getenv("PRC_DIAG_DUMP_TREE") != NULL)
    {
        fprintf(stderr, "PRC_DIAG_DUMP_TREE: PartDefinition unique_id=%u name=\"%s\" "
            "biased_index_of_line_style=%u bbox_min=(%f,%f,%f) bbox_max=(%f,%f,%f) "
            "num_rep_items=%u number_views=%u\n",
            data->base.base.unique_id, data->base.base.name.same ? "" :
                (data->base.base.name.name.string ? (const char *)data->base.base.name.name.string : "(null)"),
            data->base.graphics_content.biased_index_of_line_style,
            data->bounding_box.minimum_corner.x, data->bounding_box.minimum_corner.y, data->bounding_box.minimum_corner.z,
            data->bounding_box.maximum_corner.x, data->bounding_box.maximum_corner.y, data->bounding_box.maximum_corner.z,
            data->num_rep_items, data->number_views);
    }

    return 0;
}
