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

#include <string.h>
#include <stdlib.h>
#include "prc_parse_global.h"
#include "prc_parse_common.h"
#include "debug.h"

#define STB_IMAGE_IMPLEMENTATION
/* Need to figure out how to pass the ctx here */
//#define STBI_MALLOC(sz)           prc_malloc(sz)
//#define STBI_REALLOC(p,newsz)     realloc(p,newsz)
//#define STBI_FREE(p)              free(p)
#include "stb_image.h"

#define PRC_DEBUG_DUMP_IMAGES 0

static int
prc_parse_transformation(prc_context *ctx, prc_bit_state *bit_state, prc_misc_transformation *data)
{
    int code = 0;
    uint32_t k;

    data->transformation_type = prc_bitread_uint32(ctx, bit_state);
    if (data->transformation_type == PRC_TYPE_MISC_GeneralTransformation)
    {
        for (k = 0; k < 16; k++)
        {
            data->general_transformation.general_transform[k] = prc_bitread_double(ctx, bit_state);
        }
    }
    else if (data->transformation_type == PRC_TYPE_MISC_CartesianTransformation)
    {
        /* Not clear what size read this should be. Likely 8 bits though */
        prc_parse_3d_transform(ctx, bit_state, &data->cartesian);
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_transformation\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 125 prc_ri_coordinate_system */
static int
prc_parse_type_ri_coordinatesystem(prc_context *ctx, prc_bit_state *bit_state, prc_ri_coordinate_system *data)
{
    int code = 0;
    int schema_code;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_RI_CoordinateSystem, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_representation_item_content(ctx, bit_state, &data->item_content);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_representation_item_content\n");
        return code;
    }

    code = prc_parse_transformation(ctx, bit_state, &data->transform);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_transformation\n");
        return code;
    }

    code = prc_consume_schema_extension(ctx, bit_state, schema_code);
    if (code < 0)
        return code;

    code = prc_parse_user_data(ctx, bit_state, &data->user_data);

    return code;
}

#define MAX_BYTES_SKIP 4096
static int
prc_skip_to(prc_context *ctx, prc_bit_state *bit_state, uint32_t find)
{
    prc_bit_state temp;
    uint32_t val_read;
    size_t start_pos = (size_t)bit_state->ptr;
    size_t size = 0;
    uint8_t bit;
    size_t num_bits_back;
    size_t bytes_back;
    size_t rem_bits_back;
    size_t k;
    size_t new_mask;

    temp.ptr = bit_state->ptr;
    temp.bitmask = bit_state->bitmask;

    while (size < MAX_BYTES_SKIP)
    {
        val_read = prc_bitread_uint32(ctx, bit_state);
        size = (size_t)bit_state->ptr - start_pos;
        if (val_read == find)
        {
            /* Back up for the key */
            if (find & 0xff00)
            {
                num_bits_back = 19; /* 1 xxxx xxxx 1 xxxx xxxx 0 */
            }
            else
            {
                num_bits_back = 10; /* 1 xxxx xxxx 0 */
            }

            /* One back for the number of entries. Assume less than 256 */
            num_bits_back = num_bits_back + 19;

            /* Now move back that many bits */
            bytes_back = num_bits_back >> 3;
            bit_state->ptr -= bytes_back;

            rem_bits_back = num_bits_back - bytes_back * 8;

            new_mask = bit_state->bitmask;
            for (k = 0; k < rem_bits_back; k++)
            {
                new_mask = new_mask << (size_t)1;
                if (new_mask > 0xff)
                {
                    bit_state->ptr--;
                    new_mask = 0x01;
                }
            }
            bit_state->bitmask = (uint8_t)new_mask;
            return 0;
        }

        bit = prc_bitread_bit(ctx, &temp);
        bit_state->bitmask = temp.bitmask;
        bit_state->ptr = temp.ptr;
    }
    return PRC_ERROR_KEY_NOT_FOUND;
}

/* Table 147 prc_tess_markup */
static int
prc_parse_tess_markup(prc_context *ctx, prc_bit_state *bit_state, prc_tess_markup *data)
{
    int code;
    int schema_code;
    uint32_t k;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_TESS_MarkUp, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_content_base_tess_data(ctx, bit_state, &data->tessellation_coordinates);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_base_tess_data\n");
        return code;
    }

    data->number_of_codes = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_codes > 0)
    {
        data->code_numbers = (uint32_t *)prc_calloc(ctx, data->number_of_codes, sizeof(uint32_t));
        if (data->code_numbers == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_markup\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_codes; k++)
        {
            data->code_numbers[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->number_of_text_strings = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_text_strings > 0)
    {
        data->text_strings = (prc_string *)prc_calloc(ctx, data->number_of_text_strings, sizeof(prc_string));
        if (data->code_numbers == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_markup\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_text_strings; k++)
        {
            code = prc_bitread_string(ctx, bit_state, &data->text_strings[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_bitread_string\n");
                return code;
            }
        }
    }

    code = prc_bitread_string(ctx, bit_state, &data->tessellation_label);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_bitread_string\n");
        return code;
    }

    data->behavior = prc_bitread_uint8(ctx, bit_state);

    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

/* 8.5.10 prc_graph_fill_pattern*/
static int
prc_parse_fills(prc_context *ctx, prc_bit_state *bit_state, prc_graph_fill_pattern *data)
{
    int code;
    uint32_t k;

    data->fill_pattern_type = prc_bitread_uint32(ctx, bit_state);
    if (data->fill_pattern_type == PRC_TYPE_GRAPH_DottingPattern)
    {
        prc_graph_dotting_pattern *pat_data;
        pat_data = (prc_graph_dotting_pattern *)prc_calloc(ctx, 1, sizeof(prc_graph_dotting_pattern));
        if (pat_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_fills\n");
            return PRC_ERROR_MEMORY;
        }
        data->dotting_pattern = pat_data;

        code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &pat_data->base);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
            return code;
        }

        pat_data->biased_next_pattern_index = prc_bitread_uint32(ctx, bit_state);
        pat_data->pitch = prc_bitread_double(ctx, bit_state);
        pat_data->is_offset = prc_bitread_bit(ctx, bit_state);
        pat_data->biased_color_index = prc_bitread_int32(ctx, bit_state);
    }
    else if (data->fill_pattern_type == PRC_TYPE_GRAPH_HatchingPattern)
    {
        prc_graph_hatching_pattern *pat_data;
        pat_data = (prc_graph_hatching_pattern *)prc_calloc(ctx, 1, sizeof(prc_graph_hatching_pattern));
        if (pat_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_fills\n");
            return PRC_ERROR_MEMORY;
        }
        data->hatching_pattern = pat_data;

        code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &pat_data->base);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
            return code;
        }

        pat_data->biased_next_pattern_index = prc_bitread_uint32(ctx, bit_state);
        pat_data->number_of_hatching_lines = prc_bitread_uint32(ctx, bit_state);
        if (pat_data->number_of_hatching_lines > 0)
        {
            pat_data->hatch = (prc_hatch *)prc_calloc(ctx, pat_data->number_of_hatching_lines, sizeof(prc_hatch));
            if (pat_data->hatch == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_fills\n");
                return PRC_ERROR_MEMORY;
            }

            for (k = 0; k < pat_data->number_of_hatching_lines; k++)
            {
                pat_data->hatch[k].startpoint_x = prc_bitread_double(ctx, bit_state);
                pat_data->hatch[k].endpoint_x = prc_bitread_double(ctx, bit_state);
                pat_data->hatch[k].startpoint_y = prc_bitread_double(ctx, bit_state);
                pat_data->hatch[k].endpoint_y = prc_bitread_double(ctx, bit_state);
                pat_data->hatch[k].angle = prc_bitread_double(ctx, bit_state);
                pat_data->hatch[k].style_index = prc_bitread_uint32(ctx, bit_state);
            }
        }
    }
    else if (data->fill_pattern_type == PRC_TYPE_GRAPH_SolidPattern)
    {
        prc_graph_solid_pattern *pat_data;
        pat_data = (prc_graph_solid_pattern *)prc_calloc(ctx, 1, sizeof(prc_graph_solid_pattern));
        if (pat_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_fills\n");
            return PRC_ERROR_MEMORY;
        }
        data->solid_pattern = pat_data;

        code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &pat_data->base);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
            return code;
        }

        pat_data->biased_next_pattern_index = prc_bitread_uint32(ctx, bit_state);
        pat_data->is_material = prc_bitread_bit(ctx, bit_state);
        if (pat_data->is_material)
        {
            pat_data->biased_material_index = prc_bitread_int32(ctx, bit_state);
        }
        else
        {
            pat_data->biased_color_index = prc_bitread_int32(ctx, bit_state);
        }
    }
    else if (data->fill_pattern_type == PRC_TYPE_GRAPH_VpicturePattern)
    {
        prc_graph_vpicture_pattern *pat_data;
        pat_data = (prc_graph_vpicture_pattern *)prc_calloc(ctx, 1, sizeof(prc_graph_vpicture_pattern));
        if (pat_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_fills\n");
            return PRC_ERROR_MEMORY;
        }
        data->picture_pattern = pat_data;

        code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &pat_data->base);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
            return code;
        }

        pat_data->biased_next_pattern_index = prc_bitread_uint32(ctx, bit_state);
        pat_data->pattern_dimensions[0] = prc_bitread_double(ctx, bit_state);
        pat_data->pattern_dimensions[1] = prc_bitread_double(ctx, bit_state);

        code = prc_parse_tess_markup(ctx, bit_state, &pat_data->markup);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_tess_markup\n");
            return code;
        }
    }
    else if (data->fill_pattern_type == PRC_TYPE_GRAPH_FillPattern)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_fills\n");
        return PRC_ERROR_PARSE;
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_fills\n");
        return PRC_ERROR_PARSE;
    }

    return 0;
}

static inline void
prc_copy_bit_state(prc_context *ctx, prc_bit_state *state_to, prc_bit_state *state_from)
{
    state_to->bitmask = state_from->bitmask;
    state_to->ptr = state_from->ptr;

    /* ToDo proper handling of the name */
}

/* Table 90 PRC_TYPE_GRAPH_STYLE */
static int
prc_parse_styles(prc_context *ctx, prc_bit_state *bit_state, prc_graph_style *data)
{
    int code;
    int schema_code;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_Style, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->line_width = prc_bitread_double(ctx, bit_state);
    data->is_vpicture = prc_bitread_bit(ctx, bit_state);
    data->biased_pattern_index = prc_bitread_uint32(ctx, bit_state);
    data->is_material = prc_bitread_bit(ctx, bit_state);
    data->biased_color_index = prc_bitread_uint32(ctx, bit_state);
    data->is_transparency = prc_bitread_bit(ctx, bit_state);
    if (data->is_transparency)
        data->transparency = prc_bitread_uint8(ctx, bit_state);
    data->is_rendering_parameters = prc_bitread_bit(ctx, bit_state);
    if (data->is_rendering_parameters)
        data->rendering_parameters = prc_bitread_uint8(ctx, bit_state);
    data->is_rendering_parameters2 = prc_bitread_bit(ctx, bit_state);
    if (data->is_rendering_parameters2)
        data->rendering_parameters2 = prc_bitread_uint8(ctx, bit_state);
    data->is_rendering_parameters3 = prc_bitread_bit(ctx, bit_state);
    if (data->is_rendering_parameters3)
        data->rendering_parameters3 = prc_bitread_uint8(ctx, bit_state);
    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

/* Table 104 prc_graph_line_pattern */
static int
prc_parse_line_patterns(prc_context *ctx, prc_bit_state *bit_state, prc_graph_line_pattern *data)
{
    int code;
    int schema_code;
    uint32_t k;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_LinePattern, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->number_of_elements = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_elements > 0)
    {
        data->lengths = (double *)prc_calloc(ctx, data->number_of_elements, sizeof(double));
        if (data->lengths == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_line_patterns\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_elements; k++)
        {
            data->lengths[k] = prc_bitread_double(ctx, bit_state);
        }
    }
    data->start_offset = prc_bitread_double(ctx, bit_state);
    data->scale = prc_bitread_bit(ctx, bit_state);
    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

/* prc_graph_material.  This apparently can be different types as defined in Table 89 */
/* That was not clear in the spec */
static int
prc_parse_material(prc_context *ctx, prc_bit_state *bit_state, prc_graph_material *data)
{
    int code;

    data->tag = prc_bitread_uint32(ctx, bit_state);

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    if (data->tag == PRC_TYPE_GRAPH_Material)
    {
        data->biased_ambient_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_diffuse_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_emissive_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_specular_index = prc_bitread_uint32(ctx, bit_state);

        data->shininess = prc_bitread_double(ctx, bit_state);
        data->ambient_alpha = prc_bitread_double(ctx, bit_state);
        data->diffuse_alpha = prc_bitread_double(ctx, bit_state);
        data->emissive_alpha = prc_bitread_double(ctx, bit_state);
        data->specular_alpha = prc_bitread_double(ctx, bit_state);
    }
    else if (data->tag == PRC_TYPE_GRAPH_TextureApplication)
    {
        data->biased_material_generic_index = prc_bitread_uint32(ctx, bit_state);  /* Biased index to PRC_TYPE_GRAPH_Material entry */
        data->biased_texture_definition_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_next_texture_index = prc_bitread_uint32(ctx, bit_state);
        data->biased_uv_coordinates_index = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_material\n");
        return PRC_ERROR_PARSE;
    }

    return 0;
}

/* Table 88.  There are issues with the spec and this */
static prc_cartesian_trans_2d
prc_parse_cartesian_transformation_2d(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_cartesian_trans_2d data = { 0 };

    data.behavior = prc_bitread_uint8(ctx, bit_state);
    if (data.behavior & PRC_TRANSFORMATION_Translate)
    {
        data.translation = prc_parse_2d_vector(ctx, bit_state);
    }
    if (data.behavior & PRC_TRANSFORMATION_Rotate)
    {
        data.rotation[0] = prc_parse_2d_vector(ctx, bit_state);
        data.rotation[1] = prc_parse_2d_vector(ctx, bit_state);
    }
    if (data.behavior & PRC_TRANSFORMATION_Scale)
    {
        data.scale = prc_bitread_double(ctx, bit_state);
    }
    if (data.behavior & PRC_TRANSFORMATION_NonUniformScale)
    {
        data.non_uniform_scale = prc_parse_2d_vector(ctx, bit_state);
    }
    if (data.behavior & PRC_TRANSFORMATION_NonOrtho)
    {
        data.non_ortho_matrix[0] = prc_parse_2d_vector(ctx, bit_state);
        data.non_ortho_matrix[1] = prc_parse_2d_vector(ctx, bit_state);
    }
    if (data.behavior & PRC_TRANSFORMATION_Homogeneous)
    {
        data.homogeneous[0] = prc_bitread_double(ctx, bit_state);
        data.homogeneous[1] = prc_bitread_double(ctx, bit_state);
        data.homogeneous[2] = prc_bitread_double(ctx, bit_state);
    }
    return data;
}

static int
prc_parse_texture_trans(prc_context *ctx, prc_bit_state *bit_state, prc_graph_texture_transformation *data)
{
    int code;
    int schema_code;
    uint32_t tag;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_TextureTransformation, &tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    data->tag = tag;

    /* Not clear where these are applied */
    data->invert_s = prc_bitread_bit(ctx, bit_state);
    data->invert_t = prc_bitread_bit(ctx, bit_state);

    data->transform_2d = prc_bitread_bit(ctx, bit_state);

    data->transform = prc_parse_cartesian_transformation_2d(ctx, bit_state);
    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

/* Table 96 prc_graph_texture_definition */
static int
prc_parse_graph_textures(prc_context *ctx, prc_bit_state *bit_state, prc_graph_texture_definition *data)
{
    int schema_code;
    int code = 0;
    uint32_t k;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_TextureDefinition, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_content_prc_ref_base(ctx, bit_state, false, NULL, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->biased_picture_index = prc_bitread_uint32(ctx, bit_state);
    data->texture_dimension = prc_bitread_uint8(ctx, bit_state);

    data->texture_mapping_type = prc_bitread_int32(ctx, bit_state);

    if (data->texture_mapping_type == PRC_texture_mapping_retrieve_UV)
    {
        data->texture_mapping_operator = prc_bitread_int32(ctx, bit_state);

        data->has_transformation = prc_bitread_bit(ctx, bit_state);
        if (data->has_transformation)
        {
            code = prc_parse_cart_trans(ctx, bit_state, &data->transformation);
        }
    }

    data->texture_mapping_attributes = prc_bitread_uint32(ctx, bit_state);

    data->number_texture_mapping_attributes_intensities = prc_bitread_uint32(ctx, bit_state);
    if (data->number_texture_mapping_attributes_intensities > 0)
    {
        data->texture_mapping_attributes_intensities =
            (double *)prc_calloc(ctx, data->number_texture_mapping_attributes_intensities, sizeof(double));
        if (data->texture_mapping_attributes_intensities == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_graph_textures\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_texture_mapping_attributes_intensities; k++)
        {
            data->texture_mapping_attributes_intensities[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    data->number_of_texture_mapping_attributes_components = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_texture_mapping_attributes_components > 0)
    {
        data->texture_mapping_attributes_components =
            (uint8_t *)prc_calloc(ctx, data->number_of_texture_mapping_attributes_components, sizeof(uint8_t));
        if (data->texture_mapping_attributes_components == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_graph_textures\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_texture_mapping_attributes_components; k++)
        {
            data->texture_mapping_attributes_components[k] = prc_bitread_uint8(ctx, bit_state);
        }
    }

    data->texture_function = prc_bitread_int32(ctx, bit_state);
    if (data->texture_function == PRC_texture_function_blend)
    {
        for (k = 0; k < 4; k++)
        {
            data->blend_src[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    /* blend_des_rgb and blend_des_alpha are mentioned above the table but not in
       the table.  Turns out there is a dependency on the read */
    data->blend_src_rgb = prc_bitread_int32(ctx, bit_state);
    if (data->blend_src_rgb != 0)
        data->blend_des_rgb = prc_bitread_int32(ctx, bit_state);
    data->blend_src_alpha = prc_bitread_int32(ctx, bit_state);
    if (data->blend_src_alpha != 0)
        data->blend_des_alpha = prc_bitread_int32(ctx, bit_state);

    data->texture_application_mode = prc_bitread_uint8(ctx, bit_state);
    if (data->texture_application_mode & PRC_texture_application_alpha)
    {
        data->alpha_test = prc_bitread_int32(ctx, bit_state);
        data->alpha_test_reference = prc_bitread_double(ctx, bit_state);
    }

   /* Does not exist and should be removed from spec */
   // data->texture_wrapping_mode = prc_bitread_uint8(ctx, bit_state);
    data->texture_wrapping_mode_s = prc_bitread_int32(ctx, bit_state);

    if (data->texture_dimension > 1)
    {
        data->texture_wrapping_mode_t = prc_bitread_int32(ctx, bit_state);
    }
    if (data->texture_dimension > 2)
    {
        data->texture_wrapping_mode_r = prc_bitread_int32(ctx, bit_state);
    }

    data->has_texture_transformation = prc_bitread_bit(ctx, bit_state);

    if (data->has_texture_transformation)
    {
        code = prc_parse_texture_trans(ctx, bit_state, &data->texture_transformation);
    }
    if (code < 0)
        return code;
    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

/* Table 93 parsing */
static int
prc_parse_graph_picture(prc_context *ctx, prc_bit_state *bit_state, prc_graph_picture *data, prc_file_structure_header *header)
{
    int schema_code;
    int code = 0;
    unsigned char *stb_output;
    uint32_t k;

    schema_code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_GRAPH_Picture, &data->tag);
    if (schema_code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return schema_code;
    }

    code = prc_parse_content_prc_base(ctx, bit_state, &data->base);
    data->format = prc_bitread_uint32(ctx, bit_state);  /* Spec was not clear on this encoding.  Stuart said unsigned int */

    data->biased_uncompressed_file_index = prc_bitread_uint32(ctx, bit_state);
    k = data->biased_uncompressed_file_index - 1;
    data->pixel_width = prc_bitread_uint32(ctx, bit_state);
    data->pixel_height = prc_bitread_uint32(ctx, bit_state);

    /* If the data type is PNG or JPG lets do the decode now */
    if ((data->format == KEPRCPicture_JPG || data->format == KEPRCPicture_PNG) && k >= 0 && k < header->file_count &&
        header->files[k].raw_image == NULL)
    {

#if PRC_DEBUG_DUMP_IMAGES
        FILE *f;
        char filename[256];

        if (data->format == KEPRCPicture_JPG)
            sprintf(filename, "image_%d.jpg", k);
        else
            sprintf(filename, "image_%d.png", k);
        f = fopen(filename, "wb");
        fwrite(header->files[k].block, 1, header->files[k].block_size, f);
        fclose(f);
#endif

        stb_output = stbi_load_from_memory(header->files[k].block, header->files[k].block_size,
            &header->files[k].x, &header->files[k].y, &header->files[k].n, 0);
        if (stb_output != NULL)
        {
            size_t image_size;

            if (header->files[k].x <= 0 || header->files[k].y <= 0 || header->files[k].n <= 0 ||
                (size_t)header->files[k].x > SIZE_MAX / (size_t)header->files[k].y)
            {
                stbi_image_free(stb_output);
                prc_error(ctx, PRC_ERROR_MEMORY, "Image dimensions overflow in prc_parse_uncomp_file\n");
                return PRC_ERROR_MEMORY;
            }
            image_size = (size_t)header->files[k].x * (size_t)header->files[k].y;
            if (image_size > SIZE_MAX / (size_t)header->files[k].n)
            {
                stbi_image_free(stb_output);
                prc_error(ctx, PRC_ERROR_MEMORY, "Image dimensions overflow in prc_parse_uncomp_file\n");
                return PRC_ERROR_MEMORY;
            }
            image_size *= (size_t)header->files[k].n;

            header->files[k].raw_image = prc_malloc(ctx, image_size);
            if (header->files[k].raw_image == NULL)
            {
                stbi_image_free(stb_output);
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_parse_uncomp_file\n");
                return PRC_ERROR_MEMORY;
            }
            memcpy(header->files[k].raw_image, stb_output, image_size);
            data->num_elements_per_pixel = header->files[k].n;
            stbi_image_free(stb_output);
        }
        else
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_parse_uncomp_file\n");
            return PRC_ERROR_MEMORY;
        }
    }
    if (code < 0)
        return code;
    return prc_consume_schema_extension(ctx, bit_state, schema_code);
}

static prc_font_key
prc_parse_font_key(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_font_key key;

    key.font_size = prc_bitread_uint32(ctx, bit_state);
    key.font_attributes = prc_bitread_uint8(ctx, bit_state);

    return key;
}

static int
prc_parse_font_keys_same_font(prc_context *ctx, prc_bit_state *bit_state, prc_font_keys_same_font *data)
{
    int code = 0;
    uint32_t k;

    code = prc_bitread_string(ctx, bit_state, &data->font_name);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_bitread_string\n");
        return code;
    }

    data->character_set = prc_bitread_uint32(ctx, bit_state);
    data->key_count = prc_bitread_uint32(ctx, bit_state);

    if (data->key_count > 0)
    {
        data->font_key_list = (prc_font_key *)prc_calloc(ctx, data->key_count, sizeof(prc_font_key));
        if (data->font_key_list == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_font_keys_same_font\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->key_count; k++)
        {
            data->font_key_list[k] = prc_parse_font_key(ctx, bit_state);
        }
    }
    return code;
}

int
prc_parse_serialize_help(prc_context *ctx, prc_bit_state *bit_state, prc_markup_serialization_helper *data)
{
    int code = 0;
    uint32_t k;

    code = prc_bitread_string(ctx, bit_state, &data->default_font_family_name);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_bitread_string\n");
        return code;
    }

    data->font_keys_count = prc_bitread_uint32(ctx, bit_state);
    if (data->font_keys_count > 0)
    {
        data->font_keys_of_font = (prc_font_keys_same_font *)prc_calloc(ctx, data->font_keys_count, sizeof(prc_font_keys_same_font));
        if (data->font_keys_of_font == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_serialize_help\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->font_keys_count; k++)
        {
            code = prc_parse_font_keys_same_font(ctx, bit_state, &data->font_keys_of_font[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_font_keys_same_font\n");
                return code;
            }
        }
    }
    return code;
}

/* Table 40 parsing */
int prc_parse_global_data(prc_context *ctx, prc_bit_state *bit_state, prc_file_struct_internal_global_data *data,
                          prc_file_structure_header *header)
{
    int code = 0;
    uint32_t k;
    prc_bit_state pos;

    data->tess_chord = prc_bitread_double(ctx, bit_state);
    data->tess_angle = prc_bitread_double(ctx, bit_state);
    DEBUG_LOG2("global tess_chord=%.17g tess_angle=%.17g\n", data->tess_chord, data->tess_angle);

    code = prc_parse_serialize_help(ctx, bit_state, &data->serialize_help);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_serialize_help\n");
        return code;
    }

    data->color_count = prc_bitread_uint32(ctx, bit_state);
    if (data->color_count > 0)
    {
        data->colors = (prc_rgb_color *)prc_calloc(ctx, data->color_count, sizeof(prc_rgb_color));
        if (data->colors == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->color_count; k++)
        {
            data->colors[k] = prc_parse_rgb(ctx, bit_state, false);
        }
    }

    data->picture_count = prc_bitread_uint32(ctx, bit_state);
    if (data->picture_count > 0)
    {
        data->pictures = (prc_graph_picture *)prc_calloc(ctx, data->picture_count, sizeof(prc_graph_picture));
        if (data->pictures == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->picture_count; k++)
        {
            code = prc_parse_graph_picture(ctx, bit_state, &data->pictures[k], header);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_graph_picture\n");
                return code;
            }
        }
    }

    data->texture_count = prc_bitread_uint32(ctx, bit_state);
    if (data->texture_count > 0)
    {
        data->textures = (prc_graph_texture_definition *)prc_calloc(ctx, data->texture_count, sizeof(prc_graph_texture_definition));
        if (data->textures == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->texture_count; k++)
        {
            code = prc_parse_graph_textures(ctx, bit_state, &data->textures[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_graph_textures\n");
                return code;
            }
        }
    }

    data->material_count = prc_bitread_uint32(ctx, bit_state);
    if (data->material_count > 0)
    {
        data->materials = (prc_graph_material *)prc_calloc(ctx, data->material_count, sizeof(prc_graph_material));
        if (data->materials == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->material_count; k++)
        {
            code = prc_parse_material(ctx, bit_state, &data->materials[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_material\n");
                return code;
            }
        }
    }

    data->line_pattern_count = prc_bitread_uint32(ctx, bit_state);
    if (data->line_pattern_count > 0)
    {
        data->line_patterns = (prc_graph_line_pattern *)prc_calloc(ctx, data->line_pattern_count, sizeof(prc_graph_line_pattern));
        if (data->line_patterns == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->line_pattern_count; k++)
        {
            code = prc_parse_line_patterns(ctx, bit_state, &data->line_patterns[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_line_patterns\n");
                return code;
            }
        }
    }

    data->style_count = prc_bitread_uint32(ctx, bit_state);
    if (data->style_count > 0)
    {
        data->styles = (prc_graph_style *)prc_calloc(ctx, data->style_count, sizeof(prc_graph_style));
        if (data->styles == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->style_count; k++)
        {
            code = prc_parse_styles(ctx, bit_state, &data->styles[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_styles\n");
                return code;
            }
        }
    }

    data->fill_count = prc_bitread_uint32(ctx, bit_state);
    if (data->fill_count > 0)
    {
        data->fills = (prc_graph_fill_pattern *)prc_calloc(ctx, data->fill_count, sizeof(prc_graph_fill_pattern));
        if (data->fills == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->fill_count; k++)
        {
            prc_copy_bit_state(ctx, &pos, bit_state);
            code = prc_parse_fills(ctx, bit_state, &data->fills[k]);
            if (code == PRC_ERROR_MEMORY)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
                return PRC_ERROR_MEMORY;
            }
            if (code == PRC_ERROR_PARSE)
            {
                prc_copy_bit_state(ctx, bit_state, &pos);
                code = prc_skip_to(ctx, bit_state, PRC_TYPE_RI_CoordinateSystem);
                if (code == PRC_ERROR_KEY_NOT_FOUND)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_global_data\n");
                    return PRC_ERROR_PARSE;
                }
                else
                    break;
            }
        }
    }

    data->ref_coord_count = prc_bitread_uint32(ctx, bit_state);
    if (data->ref_coord_count > 0)
    {
        data->ref_coords = (prc_ri_coordinate_system *)prc_calloc(ctx, data->ref_coord_count, sizeof(prc_ri_coordinate_system));
        if (data->ref_coords == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_global_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->ref_coord_count; k++)
        {
            code = prc_parse_type_ri_coordinatesystem(ctx, bit_state, &data->ref_coords[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_type_ri_coordinatesystem\n");
                return code;
            }
        }
    }

    return code;
}
