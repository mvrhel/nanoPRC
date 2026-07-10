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
#include "prc_tri_primitives_helper_api.h"
#include "prc_internal_proto_api.h"

void
prc_api_helper_set_vertex_position(prc_context *ctx,
	prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos,
	prc_tess_3d_compressed *tess, uint32_t vertex_in_pos)
{
    vertex_out->vertices[vertex_out_pos].position[0] =
        (float)tess->vertices_prc_compressed_3d[vertex_in_pos];
    vertex_out->vertices[vertex_out_pos].position[1] =
        (float)tess->vertices_prc_compressed_3d[vertex_in_pos + 1];
    vertex_out->vertices[vertex_out_pos].position[2] =
        (float)tess->vertices_prc_compressed_3d[vertex_in_pos + 2];
}

void
prc_api_helper_set_texture_coordinates(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos,
    prc_tess_3d_compressed *tess,
    prc_internal_api_position_normal_pair *position_normal_pair)
{
    vertex_out->vertices[vertex_out_pos].uv[0] = tess->uv_coordinates_3d[2 * vertex_out_pos];
    vertex_out->vertices[vertex_out_pos].uv[1] = 1 - tess->uv_coordinates_3d[2 * vertex_out_pos + 1];
    vertex_out->vertices[vertex_out_pos].color[0] = 1.0;
    vertex_out->vertices[vertex_out_pos].color[1] = 1.0;
    vertex_out->vertices[vertex_out_pos].color[2] = 1.0;
    vertex_out->vertices[vertex_out_pos].color[3] = 1.0;
    position_normal_pair[vertex_out_pos].texture_set = PRC_INTERNAL_API_TEXTURE_SET;
}

void
prc_api_helper_set_face_color(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos,
    float *face_color, prc_internal_api_position_normal_pair *position_normal_pair)
{
	memcpy(vertex_out->vertices[vertex_out_pos].color, face_color, sizeof(float) * 4);
    position_normal_pair[vertex_out_pos].texture_set = PRC_INTERNAL_API_NO_TEXTURE;
}

void
prc_api_helper_intialize_position_normal_pair(prc_context *ctx,
    prc_internal_api_position_normal_pair *position_normal_pair,
    uint32_t vertex_out_pos, prc_internal_api_color_state_t color_state,
    prc_internal_api_style_state_t style_state, int32_t face_style,
    int32_t face_style_file_index)
{
    position_normal_pair[vertex_out_pos].color_set = color_state;
	position_normal_pair[vertex_out_pos].style_set = style_state;
    position_normal_pair[vertex_out_pos].face_style_file_index = face_style_file_index;
    position_normal_pair[vertex_out_pos].api_vertex_index = (uint32_t)vertex_out_pos;
    position_normal_pair[vertex_out_pos].prc_position_index = (uint32_t)vertex_out_pos;
    position_normal_pair[vertex_out_pos].prc_normal_index = 0;
    position_normal_pair[vertex_out_pos].style_index = face_style;
    position_normal_pair[vertex_out_pos].next = NULL;
    position_normal_pair[vertex_out_pos].normal_set = PRC_INTERNAL_API_NORM_NOT_SET;
}

void
prc_api_helper_set_normal(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out,
    uint32_t vertex_out_pos, prc_tess_3d_compressed *tess,
    prc_internal_api_position_normal_pair *position_normal_pair, uint32_t normal_index)
{
    /* No normals assigned yet, so just add this one. Other members
       of this structure should already be set */
    position_normal_pair[vertex_out_pos].prc_normal_index = normal_index;
    position_normal_pair[vertex_out_pos].normal_set = PRC_INTERNAL_API_NORM_SET;

    /* Add the normal information to the vertex_out data */
    vertex_out->vertices[vertex_out_pos].normal[0] =
        tess->normals_prc_compressed_3d[normal_index * 3];
    vertex_out->vertices[vertex_out_pos].normal[1] =
        tess->normals_prc_compressed_3d[normal_index * 3 + 1];
    vertex_out->vertices[vertex_out_pos].normal[2] =
        tess->normals_prc_compressed_3d[normal_index * 3 + 2];
}

void
prc_api_helper_set_vertex_color(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos,
    prc_tess_3d_compressed *tess, uint32_t vertex_in_pos,
    prc_internal_api_position_normal_pair *position_normal_pair)
{
    position_normal_pair->color_set = PRC_INTERNAL_API_COLOR_SET;
    vertex_out->vertices[vertex_out_pos].color[0] =
        tess->decoded_point_color_array[vertex_in_pos * 4];
    vertex_out->vertices[vertex_out_pos].color[1] =
        tess->decoded_point_color_array[vertex_in_pos * 4 + 1];
    vertex_out->vertices[vertex_out_pos].color[2] =
        tess->decoded_point_color_array[vertex_in_pos * 4 + 2];
    vertex_out->vertices[vertex_out_pos].color[3] =
        tess->decoded_point_color_array[vertex_in_pos * 4 + 3];
}

uint8_t
prc_api_helper_color_mismatch(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos,
    prc_tess_3d_compressed *tess, uint32_t vertex_in_pos)
{
    if (vertex_out->vertices[vertex_out_pos].color[0] !=
        tess->decoded_point_color_array[vertex_in_pos * 4] ||
        vertex_out->vertices[vertex_out_pos].color[1] !=
        tess->decoded_point_color_array[vertex_in_pos * 4 + 1] ||
        vertex_out->vertices[vertex_out_pos].color[2] !=
        tess->decoded_point_color_array[vertex_in_pos * 4 + 2] ||
        vertex_out->vertices[vertex_out_pos].color[3] !=
        tess->decoded_point_color_array[vertex_in_pos * 4 + 3])
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int
prc_api_helper_set_vertex_style(prc_context *ctx,
    prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, prc_api_tess_vertex_buffer *vertex_out,
    uint32_t vertex_out_pos, int32_t unbiased_style_index)
{
    int code;
    uint32_t face_index;
    float alpha = 1.0f;

    /* Get the style information from the global structure */
    prc_internal_graph_style style;
    uint8_t is_material, is_texture;
    prc_api_material material;
    float style_color[4];

    code = prc_internal_api_get_style(ctx, global_data, header,
        unbiased_style_index, &is_material, &is_texture,
        &material, &style, style_color, alpha, false);
    if (code < 0)
        return code;

    memcpy(vertex_out->vertices[vertex_out_pos].color, style_color, 4 * sizeof(float));
    vertex_out->vertices[vertex_out_pos].style_index = unbiased_style_index;
    vertex_out->vertices[vertex_out_pos].tri_has_material = is_material;

    if (is_material)
    {
        /* This is a material, so set the values accordingly */
        vertex_out->vertices[vertex_out_pos].diffuse[0] = material.diffuse[0];
        vertex_out->vertices[vertex_out_pos].diffuse[1] = material.diffuse[1];
        vertex_out->vertices[vertex_out_pos].diffuse[2] = material.diffuse[2];
        vertex_out->vertices[vertex_out_pos].tint[0] = material.ambient[0];
        vertex_out->vertices[vertex_out_pos].tint[1] = material.ambient[1];
        vertex_out->vertices[vertex_out_pos].tint[2] = material.ambient[2];
        vertex_out->vertices[vertex_out_pos].specular[0] = material.specular[0];
        vertex_out->vertices[vertex_out_pos].specular[1] = material.specular[1];
        vertex_out->vertices[vertex_out_pos].specular[2] = material.specular[2];
        vertex_out->vertices[vertex_out_pos].emissive[0] = material.emissive[0];
        vertex_out->vertices[vertex_out_pos].emissive[1] = material.emissive[1];
        vertex_out->vertices[vertex_out_pos].emissive[2] = material.emissive[2];
        vertex_out->vertices[vertex_out_pos].shininess = material.shininess;
        vertex_out->vertices[vertex_out_pos].alpha = material.diffuse_alpha;
    }
    else
    {
        /* This is a color, just use the diffuse color */
        vertex_out->vertices[vertex_out_pos].diffuse[0] = style_color[0];
        vertex_out->vertices[vertex_out_pos].diffuse[1] = style_color[1];
        vertex_out->vertices[vertex_out_pos].diffuse[2] = style_color[2];

        vertex_out->vertices[vertex_out_pos].tint[0] = 1;
        vertex_out->vertices[vertex_out_pos].tint[1] = 1;
        vertex_out->vertices[vertex_out_pos].tint[2] = 1;
        vertex_out->vertices[vertex_out_pos].specular[0] = 0;
        vertex_out->vertices[vertex_out_pos].specular[1] = 0;
        vertex_out->vertices[vertex_out_pos].specular[2] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[1] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[2] = 0;
        vertex_out->vertices[vertex_out_pos].shininess = 0.2f;
        vertex_out->vertices[vertex_out_pos].alpha = 1.0f;
    }
    return 0;
}

/* We already parsed all the style data from the ref data and stored in in
   face_out_reserved. Here we just populate it for the vertex.  This
   should be called for the state PRC_INTERNAL_API_STYLE_SET_FROM_REF_DATA
   This would handle colors and materials */
int
prc_api_helper_set_vertex_style_from_face_ref(prc_context *ctx,
    uint8_t vertices_have_style, prc_api_tess_vertex_buffer *vertex_out,
    uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t triangle_count,
    prc_internal_api_position_normal_pair *position_normal_pair,
    prc_internal_api_face *face_out_reserved)
{
    uint32_t face_index = tess->triangle_face_array[triangle_count];
    prc_internal_graph_style *style = &face_out_reserved->style[face_index];
    uint8_t is_material = style->is_material;

    vertex_out->vertices[vertex_out_pos].uv_set = 0; /* We dont use the texture for this */

    position_normal_pair->style_set = PRC_INTERNAL_API_STYLE_SET_FROM_REF_DATA;
    position_normal_pair->style_index = face_out_reserved->style[face_index].face_style_index;
    position_normal_pair->face_style_file_index = face_out_reserved->style[face_index].face_style_file_index;

    if (is_material)
    {
        /* This is a material, so set the values accordingly */
        memcpy(vertex_out->vertices[vertex_out_pos].tint, &style->ambient_color, 3 * sizeof(float));
        memcpy(vertex_out->vertices[vertex_out_pos].diffuse, &style->diffuse_color, 3 * sizeof(float));
        memcpy(vertex_out->vertices[vertex_out_pos].specular, &style->specular_color, 3 * sizeof(float));
        memcpy(vertex_out->vertices[vertex_out_pos].emissive, &style->emissive_color, 3 * sizeof(float));
        vertex_out->vertices[vertex_out_pos].shininess = style->shininess;
        vertex_out->vertices[vertex_out_pos].alpha = style->diffuse_alpha;
        vertex_out->vertices[vertex_out_pos].tri_has_material = 1;
        memset(&vertex_out->vertices[vertex_out_pos].color, 0, 4 * sizeof(float)); /* We dont use the color for materials */
        vertex_out->vertices[vertex_out_pos].style_index = face_out_reserved->style[face_index].face_style_index;
        vertex_out->vertices[vertex_out_pos].style_file_index = face_out_reserved->style[face_index].face_style_file_index;
    }
    else
    {
        if (!vertices_have_style)
        {
            /* This is a color, just the color.  However, this method is only
               used in the compressed tessellation case.  We force the vertices
               to have the more complex style in this case as it is easier with
               the way faces are done.  At this point, the color is in the tint
               so set everything accordingly */
            vertex_out->vertices[vertex_out_pos].tri_has_material = 1;

            /* We need to stash the color into the vertex as a material */
            memcpy(vertex_out->vertices[vertex_out_pos].diffuse, style->tint, 3 * sizeof(float));
            vertex_out->vertices[vertex_out_pos].specular[0] = 1.0f;
            vertex_out->vertices[vertex_out_pos].specular[1] = 1.0f;
            vertex_out->vertices[vertex_out_pos].specular[2] = 1.0f;
            vertex_out->vertices[vertex_out_pos].tint[0] = 1;
            vertex_out->vertices[vertex_out_pos].tint[1] = 1;
            vertex_out->vertices[vertex_out_pos].tint[2] = 1;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;

            //memset(vertex_out->vertices[vertex_out_pos].emissive, 0, 3 * sizeof(float));
            vertex_out->vertices[vertex_out_pos].shininess = 0.2f;
            vertex_out->vertices[vertex_out_pos].alpha = 1.0f;
            vertex_out->vertices[vertex_out_pos].style_index = face_out_reserved->style[face_index].face_style_index;
            vertex_out->vertices[vertex_out_pos].style_file_index = face_out_reserved->style[face_index].face_style_file_index;

            vertex_out->vertices[vertex_out_pos].color[0] = 1;
            vertex_out->vertices[vertex_out_pos].color[1] = 1;
            vertex_out->vertices[vertex_out_pos].color[2] = 1;
            vertex_out->vertices[vertex_out_pos].color[3] = 1;
        }
        else
        {
            /* We need to stash the color into the vertex as a material */
            memcpy(vertex_out->vertices[vertex_out_pos].diffuse, style->diffuse_color, 3 * sizeof(float));
            vertex_out->vertices[vertex_out_pos].specular[0] = 1.0f;
            vertex_out->vertices[vertex_out_pos].specular[1] = 1.0f;
            vertex_out->vertices[vertex_out_pos].specular[2] = 1.0f;
            vertex_out->vertices[vertex_out_pos].tint[0] = 1;
            vertex_out->vertices[vertex_out_pos].tint[1] = 1;
            vertex_out->vertices[vertex_out_pos].tint[2] = 1;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
            vertex_out->vertices[vertex_out_pos].emissive[0] = 0;

            //memset(vertex_out->vertices[vertex_out_pos].emissive, 0, 3 * sizeof(float));
            vertex_out->vertices[vertex_out_pos].shininess = 0.2f;
            vertex_out->vertices[vertex_out_pos].alpha = 1.0f;
            vertex_out->vertices[vertex_out_pos].style_index = face_out_reserved->style[face_index].face_style_index;
            vertex_out->vertices[vertex_out_pos].style_file_index = face_out_reserved->style[face_index].face_style_file_index;

            vertex_out->vertices[vertex_out_pos].color[0] = 1;
            vertex_out->vertices[vertex_out_pos].color[1] = 1;
            vertex_out->vertices[vertex_out_pos].color[2] = 1;
            vertex_out->vertices[vertex_out_pos].color[3] = 1;
        }
    }

    return 0;
}

int
prc_api_helper_set_tri_style(prc_context *ctx,
    prc_file_struct_internal_global_data *global_data, uint32_t tess_file_style_index,
    prc_file_structure_header *header, prc_api_tess_vertex_buffer *vertex_out,
    uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t triangle_count,
    prc_internal_api_position_normal_pair *position_normal_pair,
    prc_internal_api_face *face_out_reserved, prc_internal_api_style_state_t new_state)
{
    int line_style_index;
    uint32_t file_style_index;
    int code;
    uint32_t face_index;

    /* Get the style information from the global structure */
    prc_internal_graph_style style;
    uint8_t is_material, is_texture;
    prc_api_material material;
    float style_color[4];
    float alpha = 1.0f;

    if (new_state == PRC_INTERNAL_API_STYLE_SET_FROM_TESS)
    {
        line_style_index = tess->triangle_styles[triangle_count];
        file_style_index = tess_file_style_index;
    }
    else if (new_state == PRC_INTERNAL_API_STYLE_SET_FROM_REF_DATA)
    {
        /* This is related to the face */
        face_index = tess->triangle_face_array[triangle_count];
        line_style_index = face_out_reserved->style[face_index].face_style_index;
        file_style_index = face_out_reserved->style[face_index].face_style_file_index;
    }
    else
    {
        return PRC_ERROR_PARSE; /* Invalid state to set the style from */
    }

    code = prc_internal_api_get_style(ctx, global_data, header, line_style_index,
        &is_material, &is_texture, &material, &style, style_color, alpha, false);
    if (code < 0)
        return code;

    position_normal_pair->style_set = new_state;
    position_normal_pair->style_index = line_style_index; /* Set the style index in the position normal pair */
    memcpy(vertex_out->vertices[vertex_out_pos].color, style_color, 4 * sizeof(float));
	vertex_out->vertices[vertex_out_pos].style_index = line_style_index;
    vertex_out->vertices[vertex_out_pos].tri_has_material = is_material;
    vertex_out->vertices[vertex_out_pos].style_file_index = file_style_index;


    if (is_material)
    {
        /* This is a material, so set the values accordingly */
        vertex_out->vertices[vertex_out_pos].diffuse[0] = material.diffuse[0];
        vertex_out->vertices[vertex_out_pos].diffuse[1] = material.diffuse[1];
        vertex_out->vertices[vertex_out_pos].diffuse[2] = material.diffuse[2];
        vertex_out->vertices[vertex_out_pos].tint[0] = material.ambient[0];
        vertex_out->vertices[vertex_out_pos].tint[1] = material.ambient[1];
        vertex_out->vertices[vertex_out_pos].tint[2] = material.ambient[2];
        vertex_out->vertices[vertex_out_pos].specular[0] = material.specular[0];
        vertex_out->vertices[vertex_out_pos].specular[1] = material.specular[1];
        vertex_out->vertices[vertex_out_pos].specular[2] = material.specular[2];
        vertex_out->vertices[vertex_out_pos].emissive[0] = material.emissive[0];
        vertex_out->vertices[vertex_out_pos].emissive[1] = material.emissive[1];
        vertex_out->vertices[vertex_out_pos].emissive[2] = material.emissive[2];
        vertex_out->vertices[vertex_out_pos].shininess = material.shininess;
        vertex_out->vertices[vertex_out_pos].alpha = 1.0;
    }
    else
    {
        /* This is a color, just use the diffuse color */
        vertex_out->vertices[vertex_out_pos].diffuse[0] = style_color[0];
        vertex_out->vertices[vertex_out_pos].diffuse[1] = style_color[1];
        vertex_out->vertices[vertex_out_pos].diffuse[2] = style_color[2];

        vertex_out->vertices[vertex_out_pos].tint[0] = 1;
        vertex_out->vertices[vertex_out_pos].tint[1] = 1;
        vertex_out->vertices[vertex_out_pos].tint[2] = 1;
        vertex_out->vertices[vertex_out_pos].specular[0] = 0;
        vertex_out->vertices[vertex_out_pos].specular[1] = 0;
        vertex_out->vertices[vertex_out_pos].specular[2] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[0] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[1] = 0;
        vertex_out->vertices[vertex_out_pos].emissive[2] = 0;
        vertex_out->vertices[vertex_out_pos].shininess = 0.2f;
        vertex_out->vertices[vertex_out_pos].alpha = 1.0f;
    }
	return 0;
}
