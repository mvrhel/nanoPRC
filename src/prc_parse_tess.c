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
#include <stdio.h>
#include <string.h>
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "debug.h"
#include "prc_decode_compressed_tess.h"
#include "prc_decode_markup_tess.h"

#define MATLAB_DEBUG 0

/* Table 176 PRC_TYPE_BINARY_TEXTURE_DATA.  This object is stored in a unique way.
   The entire structure is packed int an unsigned integer array that is written
   byte by byte via MakePortable32BitsUnsigned */
static prc_binary_texture_data *
prc_parse_binary_texture_data(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_binary_texture_data *data;
    uint32_t k;
    uint32_t num_bytes;

    /* The first unsigned int is the number of 32 bit chunks in the bit stream
       that follows. These are stored in 8 bit parts so that we are platform
       independent */
    data = (prc_binary_texture_data *)prc_calloc(ctx, 1, sizeof(prc_binary_texture_data));
    if (data == NULL)
    {
        prc_free(ctx, data);
        return NULL;
    }
    data->texture_binary_data_size = prc_bitread_uint32(ctx, bit_state);
    num_bytes = data->texture_binary_data_size * 4;

    data->texture_binary_data = (uint8_t *)prc_calloc(ctx, num_bytes, sizeof(uint8_t));
    if (data->texture_binary_data == NULL)
    {
        prc_free(ctx, data);
        return NULL;
    }

    for (k = 0; k < num_bytes; k++)
    {
        data->texture_binary_data[k] = prc_bitread_uint8(ctx, bit_state);
    }

    data->last_integer_used_bit_number = prc_bitread_uint32(ctx, bit_state);

    return data;
}

/* Table 175 PRC_TYPE_COMPRESSED_TEXTURE_PARAMETER.  */
static prc_compressed_texture_parameter *
prc_parse_texture_data(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_compressed_texture_parameter *data;
    uint32_t k;

    data = (prc_compressed_texture_parameter *)prc_calloc(ctx, 1, sizeof(prc_compressed_texture_parameter));
    if (data == NULL)
        return NULL;

    data->binary_texture_data = prc_parse_binary_texture_data(ctx, bit_state);
    if (data->binary_texture_data == NULL)
    {
        prc_free(ctx, data);
        return NULL;
    }

    data->reference_array_size = prc_bitread_uint32(ctx, bit_state);
    if (data->reference_array_size > 0)
    {
        data->reference_array = (uint32_t *)prc_calloc(ctx, data->reference_array_size, sizeof(uint32_t));
        if (data->reference_array == NULL)
        {
            prc_free(ctx, data->binary_texture_data);
            prc_free(ctx, data);
            return NULL;
        }

        for (k = 0; k < data->reference_array_size; k++)
        {
            /* Stored as NumberOfBitsThenUnsignedInteger. */
            data->reference_array[k] = prc_bitread_number_of_bits_then_unsigned_int(ctx, bit_state);
        }
    }

    data->texture_parameters_tolerance = prc_bitread_double(ctx, bit_state);
    data->texture_parameters_size = prc_bitread_uint32(ctx, bit_state);
    if (data->texture_parameters_size > 0)
    {
        data->texture_parameters = (float *)prc_calloc(ctx, data->texture_parameters_size, sizeof(float));
        if (data->texture_parameters == NULL)
        {
            prc_free(ctx, data->binary_texture_data);
            prc_free(ctx, data->reference_array);
            prc_free(ctx, data);
            return NULL;
        }

        for (k = 0; k < data->texture_parameters_size; k++)
        {
            data->texture_parameters[k] = prc_bitread_float(ctx, bit_state);
        }
    }
    return data;
}

/* Table 143 VertexColors */
static int
prc_parse_vertexcolors(prc_context *ctx, prc_bit_state *bit_state, prc_vertex_colors *data,
    uint32_t number_colors, uint8_t is_face)
{
    uint32_t k;
    prc_rgb_color last_value;

    data->is_rgba = prc_bitread_bit(ctx, bit_state);

    if (!is_face)
    {
        data->is_segment_color = prc_bitread_bit(ctx, bit_state);
    }
    else
    {
        data->is_segment_color = 0;
    }
    data->b_optimized = prc_bitread_bit(ctx, bit_state); /* Should always be false */

    if (!data->b_optimized)
    {
        data->color_data.remaining_vertices = prc_calloc(ctx, (number_colors - 1), sizeof(prc_color_data_remainder));
        if (data->color_data.remaining_vertices == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_vertexcolors\n");
            return PRC_ERROR_MEMORY;
        }

        data->color_data.first_vertex = prc_parse_rgb8(ctx, bit_state, data->is_rgba);
        last_value = data->color_data.first_vertex;

        for (k = 0; k < number_colors - 1; k++)
        {
            data->color_data.remaining_vertices[k].is_same = prc_bitread_bit(ctx, bit_state);
            if (data->color_data.remaining_vertices[k].is_same)
            {
                data->color_data.remaining_vertices[k].color = last_value;
            }
            else
            {
                data->color_data.remaining_vertices[k].color = prc_parse_rgb8(ctx, bit_state, data->is_rgba);
                last_value = data->color_data.remaining_vertices[k].color;
            }
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_vertexcolors\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 140 PRC_TYPE_TESS_Face */
static int
prc_parse_tess_face(prc_context *ctx, prc_bit_state *bit_state, uint8_t must_calculate_normals,
                    uint32_t face_number, prc_tess_face *data)
{
    uint32_t k;
    int code;
    uint32_t normal_single = 0;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_TESS_Face, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data->size_of_line_attributes = prc_bitread_uint32(ctx, bit_state);
    if (data->size_of_line_attributes > 0)
    {
        data->line_attributes = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), data->size_of_line_attributes);
        if (data->line_attributes == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_face\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->size_of_line_attributes; k++)
        {
            data->line_attributes[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->start_of_wire_data = prc_bitread_uint32(ctx, bit_state);

    data->size_of_sizes_wire = prc_bitread_uint32(ctx, bit_state);
    if (data->size_of_sizes_wire > 0)
    {
        data->sizes_wire = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), data->size_of_sizes_wire);
        if (data->sizes_wire == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_face\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->size_of_sizes_wire; k++)
        {
            data->sizes_wire[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->used_entities_flag = prc_bitread_uint32(ctx, bit_state);
    data->start_triangulated = prc_bitread_uint32(ctx, bit_state);

    data->size_of_triangulateddata = prc_bitread_uint32(ctx, bit_state);
    if (data->size_of_triangulateddata > 0)
    {
        data->triangulateddata = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), data->size_of_triangulateddata);
        if (data->triangulateddata == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_face\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->size_of_triangulateddata; k++)
        {
            data->triangulateddata[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->number_of_textured_coordinate_indexes = prc_bitread_uint32(ctx, bit_state);
    data->has_vertex_colors = prc_bitread_bit(ctx, bit_state);

    if (data->has_vertex_colors)
    {
        /* data->triangulateddata[0] is number of triangles. Multiply that by
           3 to get number of vertices for this. TODO: This could be an issue
           if we have strips or fans or multiple object types */
        code = prc_parse_vertexcolors(ctx, bit_state, &data->vertex_colors,
            data->triangulateddata[0] * 3, true);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_vertexcolors\n");
            return code;
        }
    }

    if (data->size_of_line_attributes > 0)
        data->behavior = prc_bitread_uint32(ctx, bit_state);

    return 0;
}

/* This is only called if normals must be calculated. In that case, the
   normals are not present in the data but the indexing assumes they are
   and so we need to adjust. Crazy that this has to be dealt with on the
   parser side */
static int
prc_adjust_offsets(prc_context *ctx, prc_tess_3d *data)
{
    uint32_t k;
    uint32_t texture_offset = 0;
    uint32_t number_triangles_multi_normal;
    uint32_t number_fan_indices_multi_normal;
    uint32_t number_strip_indices_multi_normal;
    uint32_t number_triangles_one_normal;
    uint32_t number_fan_indices_one_normal;
    uint32_t number_strip_indices_one_normal;
    uint32_t number_triangles_multi_normal_texture;
    uint32_t number_fan_indices_multi_normal_texture;
    uint32_t number_strip_indices_multi_normal_texture;
    uint32_t number_triangles_one_normal_texture;
    uint32_t number_fan_indices_one_normal_texture;
    uint32_t number_strip_indices_one_normal_texture;
    uint32_t position;
    uint32_t fan_count_multi_norm;
    uint32_t fan_count_one_norm;
    uint32_t fan_count_multi_norm_texture;
    uint32_t fan_count_one_norm_texture;
    uint32_t strip_count_multi_norm;
    uint32_t strip_count_one_norm;
    uint32_t strip_count_multi_norm_texture;
    uint32_t strip_count_one_norm_texture;
    uint32_t j;
    uint32_t current_offset = 0;

    for (k = 0; k < data->number_of_face_tessellation; k++)
    {
        prc_tess_face *face = &data->face_tessellation_data[k];
        uint32_t used_entities_flag = face->used_entities_flag;
        uint32_t size_of_triangulateddata = face->size_of_triangulateddata;

        if (size_of_triangulateddata == 0)
        {
            continue; /* No data to adjust */
        }

        if (data->number_of_texture_coordinates > 0)
        {
            texture_offset = face->number_of_textured_coordinate_indexes;
        }
        else
        {
            texture_offset = 0;
        }

        /* Lets determine the number of each type so that we can adjust the index */
        number_triangles_multi_normal = 0;
        number_fan_indices_multi_normal = 0;
        number_strip_indices_multi_normal = 0;
        number_triangles_one_normal = 0;
        number_fan_indices_one_normal = 0;
        number_strip_indices_one_normal = 0;
        number_triangles_multi_normal_texture = 0;
        number_fan_indices_multi_normal_texture = 0;
        number_strip_indices_multi_normal_texture = 0;
        number_triangles_one_normal_texture = 0;
        number_fan_indices_one_normal_texture = 0;
        number_strip_indices_one_normal_texture = 0;
        position = 0;

        if (used_entities_flag & PRC_FACETESSDATA_Triangle)
        {
            number_triangles_multi_normal = face->triangulateddata[position];
            position++;
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleFan)
        {
            fan_count_multi_norm = face->triangulateddata[position];
            position++;
            for (j = 0; j < fan_count_multi_norm; j++)
            {
                number_fan_indices_multi_normal += face->triangulateddata[position + j];
            }
            position += fan_count_multi_norm; /* Skip fan offsets */
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleStripe)
        {
            strip_count_multi_norm = face->triangulateddata[position];
            position++;
            for (j = 0; j < strip_count_multi_norm; j++)
            {
                number_strip_indices_multi_normal += face->triangulateddata[position + j];
            }
            position += strip_count_multi_norm; /* Skip strip offsets */
        }

        /* I suspect we can't mix one normals with multinormals and non-textures
           with textures but we will assume that this can occur. The unknown
           is how they are ordred. Maybe you can have triangle multi norm,
           and fan one norm with texture an a strip with one norm no texture but
           you can't mix triangle multinorm with triangle single norm as it would
           not be clear what would come first. */
        if (used_entities_flag & PRC_FACETESSDATA_TriangleOneNormal)
        {
            number_triangles_one_normal = face->triangulateddata[position];
            position++;
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleFanOneNormal)
        {
            fan_count_one_norm = face->triangulateddata[position];
            position++;
            for (j = 0; j < fan_count_one_norm; j++)
            {
                /* And there is a + 1 for the normal but I don't know how
                   that works if the normal is not there. */
                uint32_t unmasked_value = face->triangulateddata[position + j] & ~PRC_FACETESSDATA_NORMAL_Single;
                number_fan_indices_one_normal += unmasked_value;
            }
            position += fan_count_one_norm; /* Skip fan offsets */
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleStripeOneNormal)
        {
            strip_count_one_norm = face->triangulateddata[position];
            position++;
            for (j = 0; j < strip_count_one_norm; j++)
            {
                /* And there is a + 1 for the normal but I don't know how
                   that works if the normal is not there. */
                uint32_t unmasked_value = face->triangulateddata[position + j] & ~PRC_FACETESSDATA_NORMAL_Single;
                number_strip_indices_one_normal += unmasked_value;
            }
            position += strip_count_one_norm; /* Skip strip offsets */
        }

        if (used_entities_flag & PRC_FACETESSDATA_TriangleTextured)
        {
            number_triangles_multi_normal_texture = face->triangulateddata[position];
            position++;
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleFanTextured)
        {
            fan_count_multi_norm_texture = face->triangulateddata[position];
            position++;
            for (j = 0; j < fan_count_multi_norm_texture; j++)
            {
                number_fan_indices_multi_normal_texture += face->triangulateddata[position + j];
            }
            position += fan_count_multi_norm_texture; /* Skip fan offsets */
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleStripeTextured)
        {
            strip_count_multi_norm_texture = face->triangulateddata[position];
            position++;
            for (j = 0; j < strip_count_multi_norm_texture; j++)
            {
                number_strip_indices_multi_normal_texture += face->triangulateddata[position + j];
            }
            position += strip_count_multi_norm_texture; /* Skip strip offsets */
        }

        if (used_entities_flag & PRC_FACETESSDATA_TriangleOneNormalTextured)
        {
            number_triangles_one_normal_texture = face->triangulateddata[position];
            position++;
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleFanOneNormalTextured)
        {
            fan_count_one_norm_texture = face->triangulateddata[position];
            position++;
            for (j = 0; j < fan_count_one_norm_texture; j++)
            {
                uint32_t unmasked_value = face->triangulateddata[position + j] & ~PRC_FACETESSDATA_NORMAL_Single;
                number_fan_indices_one_normal_texture += unmasked_value;
            }
            position += fan_count_one_norm_texture; /* Skip fan offsets */
        }
        if (used_entities_flag & PRC_FACETESSDATA_TriangleStripeOneNormalTextured)
        {
            strip_count_one_norm_texture = face->triangulateddata[position];
            position++;
            for (j = 0; j < strip_count_one_norm_texture; j++)
            {
                uint32_t unmasked_value = face->triangulateddata[position + j] & ~PRC_FACETESSDATA_NORMAL_Single;
                number_strip_indices_one_normal_texture += unmasked_value;
            }
            position += strip_count_one_norm_texture; /* Skip strip offsets */
        }

        /* Now we have all the counts we should be able to adjust the offset. Note
         * that the computation of the offset here affects the offset for the next
         * face, not this one. So we use the current offset now */
        face->start_triangulated = current_offset;

        current_offset += number_triangles_multi_normal * 3; /* 3 positions per triangle */
        current_offset += number_fan_indices_multi_normal; /* One position for each indice */
        current_offset += number_strip_indices_multi_normal; /* One position for each indice */

        /* It is not clear if we would even be in here in the one normal case.
           What does it mean if we have to calculate normals and we have a normal
           for the whole face? For now, assume its the same as above. Probably
           should put and assert here to catch this case */
        current_offset += number_triangles_one_normal * 3; /* 3 positions per triangle */
        current_offset += number_fan_indices_one_normal; /* One position for each indice */
        current_offset += number_strip_indices_one_normal; /* One position for each indice */

        current_offset += number_triangles_multi_normal_texture * 3 * (1 + texture_offset); /* tvtvtv  per triangle */
        current_offset += number_fan_indices_multi_normal_texture * (1 + texture_offset); /* tv each indice */
        current_offset += number_strip_indices_multi_normal_texture * (1 + texture_offset); /* tv each indice */

        /* Same as before.  We are in one normal case here */
        current_offset += number_triangles_one_normal_texture * 3 * (1 + texture_offset); /* tvtvtv  per triangle */
        current_offset += number_fan_indices_one_normal_texture * (1 + texture_offset); /* tv each indice */
        current_offset += number_strip_indices_one_normal_texture * (1 + texture_offset); /* tv each indice */
    }
     return 0;
}

/* Table 138 PRC_TYPE_TESS_3D */
static int
prc_parse_tess_3d(prc_context *ctx, prc_bit_state *bit_state, prc_tess_3d **data_in)
{
    prc_tess_3d *data;
    uint32_t k;
    int code;

    data = *data_in = (prc_tess_3d *)prc_calloc(ctx, 1, sizeof(prc_tess_3d));
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
        return PRC_ERROR_MEMORY;
    }

    /* We have already read the tag if we are here */
    data->tag = PRC_TYPE_TESS_3D;

    code = prc_parse_content_base_tess_data(ctx, bit_state, &data->tessellation_coordinates);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_base_tess_data\n");
        return code;
    }

    data->has_faces = prc_bitread_bit(ctx, bit_state);
    data->has_loops = prc_bitread_bit(ctx, bit_state);
    data->must_calculate_normals = prc_bitread_bit(ctx, bit_state);

    if (data->must_calculate_normals)
    {
        data->normal_recalculation_flags = prc_bitread_uint8(ctx, bit_state);
        data->crease_angle = prc_bitread_double(ctx, bit_state) * PRC_PI / 180.0;
    }

    data->number_of_normal_coordinates = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_normal_coordinates > 0)
    {
        data->normal_coordinates = (double *)prc_calloc(ctx, sizeof(double), data->number_of_normal_coordinates);
        if (data->normal_coordinates == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_normal_coordinates; k++)
        {
            data->normal_coordinates[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    data->number_of_wire_indices = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_wire_indices > 0)
    {
        data->wire_indices = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), data->number_of_wire_indices);
        if (data->wire_indices == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_wire_indices; k++)
        {
            data->wire_indices[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    /* This is the number of values that we have of indices that are for example
       n,v,n,v,n,v...  or n,t,v,n,t,v,n,t,v...  where n is normal indice, t is texture
       indice, v is vertex indice the exact layout is specified in the face */
    data->number_of_triangulated_indicies = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_triangulated_indicies > 0)
    {
        data->triangulated_index_array = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), data->number_of_triangulated_indicies);
        if (data->triangulated_index_array == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_triangulated_indicies; k++)
        {
            data->triangulated_index_array[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }

    data->number_of_face_tessellation = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_face_tessellation > 0)
    {
        data->face_tessellation_data = (prc_tess_face *)prc_calloc(ctx, sizeof(prc_tess_face), data->number_of_face_tessellation);
        if (data->face_tessellation_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_face_tessellation; k++)
        {
            code = prc_parse_tess_face(ctx, bit_state, data->must_calculate_normals,
                                       k, &data->face_tessellation_data[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_tess_face\n");
                return code;
            }
        }
    }

    data->number_of_texture_coordinates = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_texture_coordinates > 0)
    {
        data->texture_coordinates = (double *)prc_calloc(ctx, sizeof(double), data->number_of_texture_coordinates);
        if (data->texture_coordinates == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_texture_coordinates; k++)
        {
            data->texture_coordinates[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    /* Offsets assume normals are present, but they actually are not present
       if must_calculate_normals is true */
    if (data->must_calculate_normals)
    {
        code = prc_adjust_offsets(ctx, data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_adjust_offsets\n");
            return code;
        }
    }
    return 0;
}

/* Table 142 PRC_TYPE_TESS_3D_Wire */
static int
prc_parse_tess_3d_wire(prc_context *ctx, prc_bit_state *bit_state, prc_tess_3d_wire **data_in)
{
    uint32_t vertex_color_count = 0;
    prc_tess_3d_wire *data;
    int code;

    data = *data_in = (prc_tess_3d_wire *)prc_calloc(ctx, 1, sizeof(prc_tess_3d_wire));
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_test_3d_wire\n");
        return PRC_ERROR_MEMORY;
    }

    /* We have already read the tag if we are here */
    data->tag = PRC_TYPE_TESS_3D_Wire;

    code = prc_parse_content_base_tess_data(ctx, bit_state, &data->tessellation_coordinates);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_base_tess_data\n");
        return code;
    }

    data->number_of_wire_indexes = prc_bitread_uint32(ctx, bit_state);

    /* If data->number_of_wire_indexes == 0 we just have one wire defined by the prc_parse_content_base_tess_data */
    /* If data->number_of_wire_indexex, then for each one of these we have an array of values. The first point
       encoded tells us the number of indices that follow.  */
    if (data->number_of_wire_indexes > 0)
    {
        data->wire_elements = (prc_tess_3d_wire_element*)prc_calloc(ctx, data->number_of_wire_indexes, sizeof(prc_tess_3d_wire_element));
        if (data->wire_elements == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_test_3d_wire\n");
            return PRC_ERROR_MEMORY;
        }
        /* We can read in a total of data->number_of_wire_indexes values.  We also
           want to compute how many vertex colors this would create if we actually
           have vertex colors */
        uint32_t count = 0;
        uint32_t num_wires = 0;

        while (count < data->number_of_wire_indexes)
        {
            /* The first value gives us the number */
            uint32_t size = prc_bitread_uint32(ctx, bit_state);
            data->wire_elements[num_wires].number_of_wire_indexes = size & 0x0FFFFFFF;
            data->wire_elements[num_wires].is_connected = size & 0x30000000;
            data->wire_elements[num_wires].wire_indexes = (uint32_t *)prc_calloc(ctx, data->wire_elements[num_wires].number_of_wire_indexes, sizeof(uint32_t));
            if (data->wire_elements[num_wires].wire_indexes == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_test_3d_wire\n");
                return PRC_ERROR_MEMORY;
            }
            for (uint32_t j = 0; j < data->wire_elements[num_wires].number_of_wire_indexes; j++)
            {
                data->wire_elements[num_wires].wire_indexes[j] = prc_bitread_uint32(ctx, bit_state);
            }

            vertex_color_count += data->wire_elements[num_wires].number_of_wire_indexes;

            /* If this one is closing then we have to add another color count */
            if (data->wire_elements[num_wires].is_connected & PRC_3DWIRETESSDATA_IsClosing)
            {
                vertex_color_count += 1;
            }

            count = count + 1 + data->wire_elements[num_wires].number_of_wire_indexes;
            num_wires++;
        }
        data->number_of_wire_elements = num_wires;
    }
    else
    {
        data->number_of_wire_elements = 1;
    }

    data->vertex_color_count = vertex_color_count;
    data->has_vertex_colors = prc_bitread_bit(ctx, bit_state);
    if (data->has_vertex_colors)
    {
        code = prc_parse_vertexcolors(ctx, bit_state, &data->vertex_color_data,
            vertex_color_count, false);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_vertexcolors\n");
            return code;
        }
    }

    return 0;
}

/* Table 147 PRC_TYPE_TESS_Markup */
static int
prc_parse_tess_markup(prc_context *ctx, prc_bit_state *bit_state, prc_tess_markup **data_in)
{
    prc_tess_markup *data;
    uint32_t k;
    int code;

    data = *data_in = (prc_tess_markup *)prc_calloc(ctx, 1, sizeof(prc_tess_markup));
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_markup\n");
        return PRC_ERROR_MEMORY;
    }

    /* We have already read the tag if we are here */
    data->tag = PRC_TYPE_TESS_MarkUp;

    code = prc_parse_content_base_tess_data(ctx, bit_state, &data->tessellation_coordinates);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_base_tess_data\n");
        return code;
    }

    data->number_of_codes = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_codes > 0)
    {
        data->code_numbers = (prc_unsigned_int *)prc_calloc(ctx, data->number_of_codes, sizeof(prc_unsigned_int));
        if (data->code_numbers == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_base_tess_data\n");
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
        if (data->text_strings == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_base_tess_data\n");
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

    if (data->number_of_codes == 0 && data->number_of_text_strings == 0)
    {
        return 0; /* Nothing more to do */
    }

    /* Process the markup tessellation data into something useful that we can push
       through the API */
    code = prc_decode_markup_tess(ctx, data);

    return code;
}

#define DEBUG_COMPRESSED_TESS 0

/* Table 174 PRC_TYPE_TESS_3D_COMPRESSED. To compute the compressed point mesh we need
   tolerance, point_array, edge_status_array, point_reference_array, reference_array_size,
   and point_is_a_reference */
int
prc_parse_tess_3d_compressed(prc_context *ctx, prc_bit_state *bit_state, prc_tess_3d_compressed **data_in, uint8_t debug_tess)
{
    prc_tess_3d_compressed *data;
    uint32_t k;
    int code;
    int has_left_right_data = 0;
    uint32_t number_ref_points = 0;

    data = *data_in = (prc_tess_3d_compressed *)prc_calloc(ctx, 1, sizeof(prc_tess_3d_compressed));
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }

    /* We have already read the tag if we are here */
    data->tag = PRC_TYPE_TESS_3D_Compressed;

    data->is_calculated = prc_bitread_bit(ctx, bit_state);
    data->has_faces = prc_bitread_bit(ctx, bit_state);
    data->tolerance = prc_bitread_double(ctx, bit_state);

    data->origin_array.x = prc_bitread_float(ctx, bit_state);
    data->origin_array.y = prc_bitread_float(ctx, bit_state);
    data->origin_array.z = prc_bitread_float(ctx, bit_state);

#if DEBUG_COMPRESSED_TESS
    DEBUG_LOG("data->origin_array.x = %lf\n", data->origin_array.x);
    DEBUG_LOG("data->origin_array.y = %lf\n", data->origin_array.y);
    DEBUG_LOG("data->origin_array.z = %lf\n", data->origin_array.z);
#endif

    data->point_array = (int32_t *)prc_bitread_compressed_integer_array(ctx, bit_state, &data->point_array_size);
    if (data->point_array == NULL && data->point_array_size != 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }

    /* edge_status_array (0 - no neighbour, 1 - has right neighbour, 2 - has left neighbour, 3 - has both) */
    data->edge_status_array = prc_bitread_character_array(ctx, bit_state, &data->edge_status_array_size,
                                                          2, true, 0);
    if (data->edge_status_array == NULL && data->edge_status_array_size != 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }

#if DEBUG_COMPRESSED_TESS
    DEBUG_LOG("\nEdge status array\n");
    for (k = 0; k < data->edge_status_array_size; k++)
    {
#if MATLAB_DEBUG
        DEBUG_LOG("%d, ",data->edge_status_array[k]);
#else
        DEBUG_LOG("data->edge_status_array[%d] = %d\n", k, data->edge_status_array[k]);
#endif
    }
#endif

    /* Spec states data->face_number is derived from maximum value in triangle_face_array */
    data->triangle_face_array = (uint32_t *)prc_bitread_compressed_indice_array(ctx,
                                bit_state, &data->triangle_face_array_size, true, 0);
    data->face_number = 0;
    DEBUG_LOG("\nTriangle face array\n");
    for (k = 0; k < data->triangle_face_array_size; k++)
    {
        if (data->face_number < data->triangle_face_array[k])
            data->face_number = data->triangle_face_array[k];
#if DEBUG_COMPRESSED_TESS
#if MATLAB_DEBUG
        DEBUG_LOG("%d, ",data->triangle_face_array[k]);
#else
        DEBUG_LOG("data->triangle_face_array[%d] = %d\n", k, data->triangle_face_array[k]);
#endif
#endif
    }
    data->face_number += 1; /* Change this from a max index to a number of faces */
#if DEBUG_COMPRESSED_TESS
    DEBUG_LOG("\ndata->face_number = %d\n", data->face_number);
#endif

    if (data->triangle_face_array == NULL && data->triangle_face_array_size != 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }

    data->reference_array_size = prc_bitread_uint32(ctx, bit_state);

    if (data->reference_array_size > 0)
    {
        data->points_is_reference_array = (uint8_t *)prc_calloc(ctx, data->reference_array_size, sizeof(uint8_t));
        if (data->points_is_reference_array == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }

#if DEBUG_COMPRESSED_TESS
        DEBUG_LOG("points_is_reference_array\n");
#endif
        for (k = 0; k < data->reference_array_size; k++)
        {
            data->points_is_reference_array[k] = prc_bitread_bit(ctx, bit_state);
            if (data->points_is_reference_array[k])
                number_ref_points++;
#if DEBUG_COMPRESSED_TESS
#if MATLAB_DEBUG
            DEBUG_LOG("%d, ",data->points_is_reference_array[k]);
#else
            DEBUG_LOG("data->points_is_reference_array[%d] = %d\n", k, data->points_is_reference_array[k]);
#endif
#endif
        }
    }

    /* This is just goofy to do this with this one particular data type. They refer to number_of_reference_points which
       seems to not be defined. */
    /* If the number of reference points is greater than 3 then the point_reference_array is compressed */
    /* HTML doc has < 3 but <= 3 seems to be uncompressed */
    data->point_reference_array = prc_bitread_compressed_indice_array(ctx, bit_state,
                        &data->point_reference_array_size, false, number_ref_points);
    if (data->point_reference_array == NULL && data->point_reference_array_size != 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }
#if DEBUG_COMPRESSED_TESS
    DEBUG_LOG("Point reference array\n");
    for (k = 0; k < data->point_reference_array_size; k++)
    {
#if MATLAB_DEBUG
        DEBUG_LOG("%d, ",data->point_reference_array[k]);
#else
        DEBUG_LOG("data->point_reference_array[%d] = %d\n", k, data->point_reference_array[k]);
#endif
    }
    DEBUG_LOG("\n");
#endif

    data->must_recalculate_normals = prc_bitread_bit(ctx, bit_state);
    if (data->must_recalculate_normals)
    {
        if (data->triangle_face_array_size > 0)
        {
            data->normal_is_reversed = (uint8_t *)prc_calloc(ctx, data->triangle_face_array_size, sizeof(uint8_t));
            if (data->normal_is_reversed == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
                return PRC_ERROR_MEMORY;
            }

            for (k = 0; k < data->triangle_face_array_size; k++)
            {
                data->normal_is_reversed[k] = prc_bitread_bit(ctx, bit_state);
            }
        }

#if DEBUG_COMPRESSED_TESS
        DEBUG_LOG("\nNormal is reversed array\n");
        for (k = 0; k < data->triangle_face_array_size; k++)
        {
        #if MATLAB_DEBUG
            DEBUG_LOG("%d, ",data->normal_is_reversed[k]);
        #else
            DEBUG_LOG("data->normal_is_reversed[%d] = %d\n", k, data->normal_is_reversed[k]);
        #endif
        }
#endif

        data->crease_angle = prc_bitread_double(ctx, bit_state);

        /* Put into radians */
        data->crease_angle = data->crease_angle * PRC_PI / 180.0;
        data->normal_recalculation_flags = prc_bitread_uint8(ctx, bit_state);
    }
    else
    {   /* must_recalculate_normals false */
#if DEBUG_COMPRESSED_TESS
        DEBUG_LOG("Do not need to recalculate normals\n");
#endif

        data->normal_angle_number_of_bits = prc_bitread_uint8(ctx, bit_state);
        data->normal_binary_data_size = prc_bitread_uint32(ctx, bit_state);
        if (data->normal_binary_data_size > 0)
        {
            data->normal_binary_data = (uint8_t *)prc_calloc(ctx, data->normal_binary_data_size, sizeof(uint8_t));
            if (data->normal_binary_data == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
                return PRC_ERROR_MEMORY;
            }

            DEBUG_LOG("Normal binary data\n");
            for (k = 0; k < data->normal_binary_data_size; k++)
            {
                data->normal_binary_data[k] = prc_bitread_bit(ctx, bit_state);
#if DEBUG_COMPRESSED_TESS
#if MATLAB_DEBUG
                DEBUG_LOG("%d, ",data->normal_binary_data[k]);
#else
                DEBUG_LOG("data->normal_binary_data[%d] = 0x%x\n", k, data->normal_binary_data[k]);
#endif
#endif
            }
#if DEBUG_COMPRESSED_TESS
            DEBUG_LOG("\n");
#endif
        }

        /* Huffman encoded shortArray */
        data->normal_angle_array = prc_bitread_short_array(ctx, bit_state,
            &data->normal_angle_array_size, true, data->normal_angle_number_of_bits);
        if (data->normal_angle_array == NULL && data->normal_angle_array_size != 0)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }
#if DEBUG_COMPRESSED_TESS
        DEBUG_LOG("Normal angle array\n");
        for (k = 0; k < data->normal_angle_array_size; k++)
        {
#if MATLAB_DEBUG
            DEBUG_LOG("%d, ",data->normal_angle_array[k]);
#else
            DEBUG_LOG("data->normal_angle_array[%d] = %d\n", k, data->normal_angle_array[k]);
#endif
        }
#endif
        DEBUG_LOG("\n");

        data->has_faces = 1; /* Must be true if we have normal data like this */
        /* Spec not clear on the size of this array, I suspect data->face_number */
        data->is_face_planar = (uint8_t *)prc_calloc(ctx, data->face_number, sizeof(uint8_t));
        if (data->is_face_planar == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->face_number; k++)
        {
            data->is_face_planar[k] = prc_bitread_bit(ctx, bit_state);
#if DEBUG_COMPRESSED_TESS
            DEBUG_LOG("data->is_face_planar[%d] = %d\n", k, data->is_face_planar[k]);
#endif
        }
    }

    data->is_point_color = prc_bitread_bit(ctx, bit_state);
    if (data->is_point_color)
    {
        /* Spec not clear on the size of this array */
        data->is_point_color_on_face = (uint8_t *)prc_calloc(ctx, data->face_number, sizeof(uint8_t));
        if (data->is_point_color_on_face == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->face_number; k++)
        {
            data->is_point_color_on_face[k] = prc_bitread_bit(ctx, bit_state);
        }

        data->point_color_array = prc_bitread_character_array(ctx, bit_state,
                                        &data->point_color_array_size, 8, true, 0);
        if (data->point_color_array == NULL && data->point_color_array_size != 0)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }

        /* See table 173 for a description of the point color array. It has a special
           encoding telling if alpha is included. 5 characters are used for each
           vertex. First character says if alpha was included.  The next three are
           RGB the last one is alpha or should be ignored depending upon the first
           character */
        data->decoded_point_color_array_size = data->point_color_array_size * 4 / 5;
        data->decoded_point_color_array = (float *)prc_calloc(ctx,
            data->decoded_point_color_array_size, sizeof(float));
        if (data->decoded_point_color_array == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }
        uint32_t num_colors = data->point_color_array_size / 5;
        for (k = 0; k < num_colors; k++)
        {
            uint8_t alpha_included = data->point_color_array[k * 5];
            data->decoded_point_color_array[k * 4 + 0] = (float)data->point_color_array[k * 5 + 1] / 255.0f;
            data->decoded_point_color_array[k * 4 + 1] = (float)data->point_color_array[k * 5 + 2] / 255.0f;
            data->decoded_point_color_array[k * 4 + 2] = (float)data->point_color_array[k * 5 + 3] / 255.0f;
            if (alpha_included)
                data->decoded_point_color_array[k * 4 + 3] = (float)data->point_color_array[k * 5 + 4] / 255.0f;
            else
                data->decoded_point_color_array[k * 4 + 3] = 1.0f;
        }
    }
    else
    {
        data->decoded_point_color_array = NULL;
        data->decoded_point_color_array_size = 0;
    }

    data->is_multiple_line_attribute = prc_bitread_bit(ctx, bit_state);
    if (data->is_multiple_line_attribute)
    {
        /* Spec not clear on the size of this array */
        data->is_multiple_line_attribute_on_face = (uint8_t *)prc_calloc(ctx, data->face_number, sizeof(uint8_t));
        if (data->is_multiple_line_attribute_on_face == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->face_number; k++)
        {
            data->is_multiple_line_attribute_on_face[k] = prc_bitread_bit(ctx, bit_state);
        }
    }

    data->line_attribute_array = prc_bitread_short_array(ctx, bit_state, &data->line_attribute_array_size, true, 16);
    if (data->line_attribute_array == NULL && data->line_attribute_array_size != 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
        return PRC_ERROR_MEMORY;
    }

    data->no_texture = prc_bitread_bit(ctx, bit_state);
    if (!data->no_texture)
    {
        /* Read in the texture data.  First the CompressedTextureParameter */
        data->texture_data = prc_parse_texture_data(ctx, bit_state);
        if (data->texture_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }

        data->all_faces_have_texture = prc_bitread_bit(ctx, bit_state);

        if (!data->all_faces_have_texture)
        {
            /* Spec not clear on the size of this array */
            data->face_has_texture = (uint8_t *)prc_calloc(ctx, data->face_number, sizeof(uint8_t));
            if (data->face_has_texture == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
                return PRC_ERROR_MEMORY;
            }
            for (k = 0; k < data->face_number; k++)
            {
                data->face_has_texture[k] = prc_bitread_bit(ctx, bit_state);
            }
        }
    }

    data->has_behaviors = prc_bitread_bit(ctx, bit_state);
    if (data->has_behaviors)
    {
        data->behaviors_array = prc_bitread_character_array(ctx, bit_state,
                                            &data->behaviors_array_size, 8, true,
                                            0);
        if (data->behaviors_array == NULL && data->behaviors_array_size != 0)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
            return PRC_ERROR_MEMORY;
        }
    }

    /* Lets see if there is ANY left right data in here */
    for (k = 0; k < data->edge_status_array_size; k++)
    {
        if (data->edge_status_array[k] != 0)
        {
            has_left_right_data = 1;
            break;
        }
    }

    if (data->point_array_size > 0)
    {
        /* Lets go ahead and get the vertices, the normals, the triangle vertex indices */
        /* Right now we are going to assume that if we have no left right data
           that we must compute the normals, as that is the only file that I have
           that is that case */
        code = prc_decode_compressed_tess(ctx, data, debug_tess);
        if (code != 0)
            return code;

        /* Don't do this if the normal angles are encoded OR if we have
           vertex colors */
        if (data->must_recalculate_normals && data->decoded_point_color_array == NULL)
        {
            code = prc_compressed_tess_apply_crease_angle(ctx, data);
            if (code != 0)
                return code;
        }

        /* Lets compute edges for this tessellation. These are computed by
          looking at the triangle normals and those that are greater than a
          particular theshold will define a line */
        code = prc_compute_edges_compressed_tess(ctx, data);
        if (code != 0)
            return code;

        /* If we have texture coordinates for the vertices, then copy them over now */
        if (!data->no_texture && data->texture_data->texture_parameters != NULL)
        {
            data->uv_coordinates_3d = (float *)prc_calloc(ctx,
                data->texture_data->texture_parameters_size, sizeof(float));
            if (data->uv_coordinates_3d == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_tess_3d_compressed\n");
                return PRC_ERROR_MEMORY;
            }
            memcpy(data->uv_coordinates_3d, data->texture_data->texture_parameters,
                data->texture_data->texture_parameters_size * sizeof(float));
        }
        return code;
    }
    else
    {
        /* We have no vertices so we should skip this face. That is handled
           later during the API processing */
        return 0;
    }
}

/* Section 8.8.2 PRC_TYPE_TESS Abstract type */
int
prc_parse_tess(prc_context *ctx, prc_bit_state *bit_state, prc_tess *data, uint8_t debug_tess)
{
    int code = 0;

    data->tess_type = prc_bitread_uint32(ctx, bit_state);

    switch (data->tess_type)
    {
    case PRC_TYPE_TESS_3D:
        code = prc_parse_tess_3d(ctx, bit_state, &data->tess_3d);
        break;
    case PRC_TYPE_TESS_3D_Compressed:
        code = prc_parse_tess_3d_compressed(ctx, bit_state, &data->tess_3d_compressed, debug_tess);
        break;
    case PRC_TYPE_TESS_3D_Wire:
        code = prc_parse_tess_3d_wire(ctx, bit_state, &data->tess_3d_wire);
        break;
    case PRC_TYPE_TESS_MarkUp:
        code = prc_parse_tess_markup(ctx, bit_state, &data->tess_markup);
        break;
    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Unknown Tessellation type\n");
        return PRC_ERROR_PARSE;
    }
    return code;
}
