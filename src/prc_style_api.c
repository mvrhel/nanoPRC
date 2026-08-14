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
#include <string.h>
#include "../include/prc_api.h"
#include "prc_internal_api.h"
#include "prc_data.h"


void prc_internal_api_initialize_vertex(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_buffer)
{
    uint32_t k;
    uint32_t j;
    uint32_t start_index = vertex_buffer->num_vertices;
    uint32_t end_index = vertex_buffer->capacity;
    prc_api_vertex *vertices = vertex_buffer->vertices;

    /* Initialize the vertex buffer */
    for (k = start_index; k < end_index; k++)
    {
        for (j = 0; j < 3; j++)
        {
            vertices[k].position[j] = 0.0f;
            vertices[k].normal[j] = 0.0f;
            vertices[k].color[j] = 1.0f;
            vertices[k].diffuse[j] = 0.0f;
            vertices[k].tint[j] = 0.0f;
            vertices[k].specular[j] = 0.0f;
            vertices[k].emissive[j] = 0.0f;
        }

        vertices[k].uv[0] = 0.0f;
        vertices[k].uv[1] = 0.0f;
        vertices[k].color[3] = 1.0f;

        vertices[k].tri_has_material = 0;
        vertices[k].shininess = 0;
        vertices[k].alpha = 1.0f;

        /* This region comes from prc_realloc, which -- unlike prc_calloc,
           used for the buffer's initial allocation -- does not zero new
           memory. Without explicitly clearing these here, they retain
           whatever the reused heap memory previously held (a genuine
           uninitialized-read bug, not merely a missing "nice to have"
           default): normal_set/uv_set false-positive as "set" at random,
           and style_index/style_file_index can come up as an arbitrary
           value instead of the "no style assigned yet" default every
           freshly prc_calloc'd vertex gets implicitly. Match that default
           (all-zero) here so a vertex's observable state does not depend
           on whether it happened to need this buffer to grow. */
        vertices[k].normal_set = 0;
        vertices[k].uv_set = 0;
        vertices[k].style_index = 0;
        vertices[k].style_file_index = 0;
    }
}

void
prc_api_initialize_face(prc_context *ctx, prc_api_face *face)
{
    face->has_transparency = 0;
    face->is_material = 0;
    face->is_texture = 0;
    face->num_graphic_primitives = 0;
    face->vertices_have_style = 0;
    face->face_has_single_style = 0;
    face->disable_face = 0;
    face->reserved = NULL;
}

void
prc_api_initialize_texture(prc_context *ctx, prc_api_texture *texture)
{
    texture->data = NULL;
    texture->height = 0;
    texture->num_channels = 0;
    texture->width = 0;
    texture->has_transform = 0;
}

void
prc_internal_api_initialize_texture(prc_context *ctx, prc_internal_texture *texture)
{
    texture->next_texture_index = -1;
    texture->picture_index = -1;
    texture->texture_definition_index = -1;
    texture->uv_coordinates_index = -1;
    texture->wrapping_u = 0;
    texture->wrapping_v = 0;
    texture->material_generic_index = -1;
    texture->num_elements_per_pixel = 0;
    texture->picture_data = NULL;
    texture->picture_height = 0;
    texture->picture_width = 0;
    texture->has_texture_transform = 0;
    memset(texture->texture_transform, 0, sizeof(double) * 9);
    texture->next_texture = NULL;
}

void
prc_internal_api_initialize_style(prc_context *ctx, prc_internal_graph_style *style)
{
    style->ambient_alpha = 0.0f;
    style->ambient_index = -1;
    style->diffuse_alpha = 0.0f;
    style->diffuse_index = -1;
    style->emissive_alpha = 0.0f;
    style->emissive_index = -1;
    style->shininess = 0.0f;
    style->specular_alpha = 0.0f;
    style->specular_index = -1;
    style->is_material = 0;
    style->is_transparency = 0;
    style->is_vpicture = 0;
    style->biased_pattern_index = -1;
    style->line_width = 1.0f;
    style->face_style_index = -1;
    style->face_style_file_index = -1;
    memset(style->tint, 0, sizeof(float) * 4);
    prc_internal_api_initialize_texture(ctx, &style->texture);
}

int
prc_internal_api_get_color(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    int32_t color_index, float *color_out)
{
    prc_rgb_color rgb_color;

    if (global_data->colors != NULL)
    {
        if (color_index < global_data->color_count && color_index > -1)
        {
            rgb_color = global_data->colors[color_index];
            color_out[0] = rgb_color.red;
            color_out[1] = rgb_color.green;
            color_out[2] = rgb_color.blue;
        }
    }
    return 0;
}

/* Note style index is the unbiased index here */
int
prc_internal_api_get_color_from_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    int32_t style_index, float *color_out)
{
    prc_rgb_color rgb_color;
    uint32_t color_material_index = 0;
    prc_graph_style graph_style;
    prc_graph_material graph_material;
    uint32_t rgb_material_index;

    if (global_data->styles != NULL)
    {
        if (style_index < global_data->style_count && style_index > -1)
        {
            color_material_index = global_data->styles[style_index].biased_color_index;
        }
        else
        {
            color_out[0] = 0.8f;
            color_out[1] = 0.8f;
            color_out[2] = 0.8f;
            color_out[3] = 1.0f;
            return 0;
        }

        if (color_material_index == 0)
        {
            color_out[0] = 0.8f;
            color_out[1] = 0.8f;
            color_out[2] = 0.8f;
            color_out[3] = 1.0f;
            return 0;
        }
        else
            color_material_index--;  /* Unbias this */

        graph_style = global_data->styles[style_index];

        if (graph_style.is_material)
        {
            /* Is a texture or a material. Just return white color */
            color_out[0] = 1.0f;
            color_out[1] = 1.0f;
            color_out[2] = 1.0f;
            color_out[3] = 1.0f;
        }
        else
        {
            /* This is a color */
            if (color_material_index < global_data->color_count)
            {
                rgb_color = global_data->colors[color_material_index];
                color_out[0] = rgb_color.red;
                color_out[1] = rgb_color.green;
                color_out[2] = rgb_color.blue;
                color_out[3] = rgb_color.alpha;
            }
        }
    }
    return 0;
}

int
prc_internal_get_surface_material(prc_context *ctx,
    prc_file_struct_internal_global_data *global_data, int32_t index,
    prc_api_material *material)
{
    int32_t color_index;
    prc_rgb_color rgb_color;

    if (global_data->materials == NULL)
    {
        return PRC_ERROR_PARSE;
    }

    if (index < global_data->material_count)
    {
        prc_graph_material graph_material = global_data->materials[index];
        material->ambient_alpha = (float) graph_material.ambient_alpha;
        material->diffuse_alpha = (float) graph_material.diffuse_alpha;
        material->emissive_alpha = (float) graph_material.emissive_alpha;
        material->specular_alpha = (float) graph_material.specular_alpha;
        material->shininess = (float) graph_material.shininess;

        if (graph_material.biased_ambient_index <= 0 ||
            graph_material.biased_diffuse_index <= 0 ||
            graph_material.biased_emissive_index <= 0 ||
            graph_material.biased_specular_index <= 0)
        {
            return PRC_ERROR_PARSE;
        }

        color_index = (graph_material.biased_ambient_index - 1) /3;
        if (color_index < global_data->color_count)
        {
            rgb_color = global_data->colors[color_index];
            material->ambient[0] = (float)rgb_color.red;
            material->ambient[1] = (float)rgb_color.green;
            material->ambient[2] = (float)rgb_color.blue;
        }
        else
        {
            return PRC_ERROR_PARSE;
        }

        color_index = (graph_material.biased_diffuse_index - 1) / 3;
        if (color_index < global_data->color_count)
        {
            rgb_color = global_data->colors[color_index];
            material->diffuse[0] = (float)rgb_color.red;
            material->diffuse[1] = (float)rgb_color.green;
            material->diffuse[2] = (float)rgb_color.blue;
        }
        else
        {
            return PRC_ERROR_PARSE;
        }

        color_index = (graph_material.biased_emissive_index - 1) / 3;
        if (color_index < global_data->color_count)
        {
            rgb_color = global_data->colors[color_index];
            material->emissive[0] = (float)rgb_color.red;
            material->emissive[1] = (float)rgb_color.green;
            material->emissive[2] = (float)rgb_color.blue;
        }
        else
        {
            return PRC_ERROR_PARSE;
        }

        color_index = (graph_material.biased_specular_index - 1) / 3;
        if (color_index < global_data->color_count)
        {
            rgb_color = global_data->colors[color_index];
            material->specular[0] = (float)rgb_color.red;
            material->specular[1] = (float)rgb_color.green;
            material->specular[2] = (float)rgb_color.blue;
        }
        else
        {
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Matrix elements are stored in column first order assuming we are multiplying
   on the right of the matrix by the column position vector.

   M00 M01 M02
   M10 M11 M12
   0   0   1

   where the element indices are as given in the PRC spec. So data is stored
   in memory as M00, M10, 0, M01, M11, ... */

static int
prc_internal_set_texture_transform(prc_context *ctx,
    prc_graph_texture_transformation *prc_texture_trans, double *texture_matrix)
{
    prc_cartesian_trans_2d cart = prc_texture_trans->transform;
    uint32_t k;

    memset(texture_matrix, 0, 9 * sizeof(double));
    texture_matrix[0] = 1.0;
    texture_matrix[4] = 1.0;
    texture_matrix[8] = 1.0;

    if (cart.behavior & PRC_TRANSFORMATION_Translate)
    {
        texture_matrix[2] = cart.translation.x;
        texture_matrix[5] = cart.translation.y;
    }
    if (cart.behavior & PRC_TRANSFORMATION_NonOrtho)
    {
        texture_matrix[0] = cart.non_ortho_matrix[0].x;
        texture_matrix[1] = cart.non_ortho_matrix[0].y;
        texture_matrix[3] = cart.non_ortho_matrix[1].x;
        texture_matrix[4] = cart.non_ortho_matrix[1].y;
    }
    if (cart.behavior & PRC_TRANSFORMATION_Rotate)
    {
        texture_matrix[0] = cart.rotation[0].x;
        texture_matrix[1] = cart.rotation[0].y;
        texture_matrix[3] = cart.rotation[1].x;
        texture_matrix[4] = cart.rotation[1].y;
    }
    if (cart.behavior & PRC_TRANSFORMATION_NonUniformScale)
    {
        texture_matrix[0] = texture_matrix[0] * cart.non_uniform_scale.x;
        texture_matrix[1] = texture_matrix[1] * cart.non_uniform_scale.x;
        texture_matrix[3] = texture_matrix[3] * cart.non_uniform_scale.y;
        texture_matrix[4] = texture_matrix[4] * cart.non_uniform_scale.y;
    }
    if (cart.behavior & PRC_TRANSFORMATION_Scale)
    {
        texture_matrix[0] = texture_matrix[0] * cart.scale;
        texture_matrix[1] = texture_matrix[1] * cart.scale;
        texture_matrix[3] = texture_matrix[3] * cart.scale;
        texture_matrix[4] = texture_matrix[4] * cart.scale;
    }

    if (cart.behavior & PRC_TRANSFORMATION_Homogeneous)
    {
        /* This is an odd one. */
        texture_matrix[2] = cart.homogeneous[0];
        texture_matrix[5] = cart.homogeneous[1];
        texture_matrix[8] = cart.homogeneous[2];
    }
    return 0;
}

/* Iterative (not recursive): the texture-application pipeline
   (next_texture_index chaining one texture to the next) is attacker-
   controlled via the file and was previously handled by tail recursion; walk
   it in a loop instead to avoid unbounded C-stack growth. current_graph_material
   is a by-value copy (not a pointer) precisely so it survives across loop
   iterations without depending on any per-iteration stack storage. */
int
prc_internal_set_texture_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, const prc_graph_material *graph_material,
    prc_internal_graph_style *style, prc_internal_texture *internal_texture)
{
    prc_graph_material current_graph_material = *graph_material;
    prc_internal_texture *current_internal_texture = internal_texture;
    prc_internal_graph_style *current_style = style;
    prc_internal_graph_style dummy_style;

    for (;;)
    {
        int code;
        prc_graph_texture_definition texture_def;
        int32_t texture_index;

        /* A texture can still set the material values though... */
        current_style->material_type = PRC_TYPE_GRAPH_TextureApplication;
        texture_index = current_graph_material.biased_texture_definition_index - 1;

        if (texture_index < global_data->texture_count)
        {
            texture_def = global_data->textures[texture_index];
            current_internal_texture->texture_definition_index = texture_index;
            current_internal_texture->next_texture_index = current_graph_material.biased_next_texture_index - 1;
            current_internal_texture->uv_coordinates_index = current_graph_material.biased_uv_coordinates_index - 1;
            current_internal_texture->material_generic_index = current_graph_material.biased_material_generic_index - 1;
            current_internal_texture->picture_index = texture_def.biased_picture_index - 1;
            current_internal_texture->wrapping_u = texture_def.texture_wrapping_mode_s; /* Repeat in u */
            current_internal_texture->wrapping_v = texture_def.texture_wrapping_mode_t; /* Repeat in v */
            current_internal_texture->has_texture_transform = texture_def.has_texture_transformation;
            if (current_internal_texture->has_texture_transform)
            {
                code = prc_internal_set_texture_transform(ctx, &texture_def.texture_transformation,
                    current_internal_texture->texture_transform);
                if (code != 0)
                {
                    return code;
                }
            }

            /* Get the pointer to the image data. We probably need a image type. e.g. RGB RGBA etc */
            if (global_data->pictures != NULL && current_internal_texture->picture_index > -1 &&
                current_internal_texture->picture_index < global_data->picture_count)
            {
                current_internal_texture->picture_width = header->files[current_internal_texture->picture_index].x;
                current_internal_texture->picture_height = header->files[current_internal_texture->picture_index].y;
                current_internal_texture->num_elements_per_pixel = header->files[current_internal_texture->picture_index].n;

                int32_t file_index = global_data->pictures[current_internal_texture->picture_index].biased_uncompressed_file_index - 1;
                if (file_index > -1 && file_index < header->file_count && header->files[file_index].raw_image != NULL)
                {
                    current_internal_texture->picture_data = header->files[file_index].raw_image;
                }
            }

            /* Deal with the generic material */
            if (current_internal_texture->material_generic_index > -1 &&
                current_internal_texture->material_generic_index < global_data->material_count)
            {
                prc_api_material generic_material;
                code = prc_internal_get_surface_material(ctx, global_data,
                    current_internal_texture->material_generic_index, &generic_material);
                if (code != 0)
                {
                    return code;
                }
                current_style->ambient_alpha = generic_material.ambient_alpha;
                current_style->diffuse_alpha = generic_material.diffuse_alpha;
                current_style->emissive_alpha = generic_material.emissive_alpha;
                current_style->specular_alpha = generic_material.specular_alpha;
                current_style->shininess = generic_material.shininess;

                memcpy(current_style->ambient_color, generic_material.ambient, 3 * sizeof(float));
                memcpy(current_style->diffuse_color, generic_material.diffuse, 3 * sizeof(float));
                memcpy(current_style->emissive_color, generic_material.emissive, 3 * sizeof(float));
                memcpy(current_style->specular_color, generic_material.specular, 3 * sizeof(float));
            }
            else
            {
                return PRC_ERROR_PARSE;
            }
        }
        else
        {
            return PRC_ERROR_PARSE;
        }

        /* We may have a pipeline going on here. */
        if (current_internal_texture->next_texture_index < 0)
        {
            return 0;
        }

        /* Multiple texture case. We are setup right now to
           just deal with two of these */
        current_internal_texture->next_texture = (prc_internal_texture *)prc_calloc(ctx, 1, sizeof(prc_internal_texture));
        if (current_internal_texture->next_texture == NULL)
        {
            return PRC_API_ERROR_MEMORY;
        }

        current_graph_material = global_data->materials[current_internal_texture->next_texture_index];
        current_internal_texture = current_internal_texture->next_texture;
        current_style = &dummy_style;
    }
}

int
prc_internal_set_internal_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, prc_graph_style *graph_style, uint8_t is_material,
    uint8_t is_texture, prc_internal_graph_style *style)
{
    int code;

    style->line_width = graph_style->line_width;
    style->is_vpicture = graph_style->is_vpicture;
    style->biased_color_index = graph_style->biased_color_index;
    style->biased_pattern_index = graph_style->biased_pattern_index;
    style->is_material = graph_style->is_material; /* True for texture OR material */
    style->is_transparency = graph_style->is_transparency;
    style->tint[0] = 1.0f;
    style->tint[1] = 1.0f;
    style->tint[2] = 1.0f;
    style->tint[3] = 1.0f;

    int32_t material_index = graph_style->biased_color_index - 1;
    prc_graph_material graph_material = global_data->materials[material_index];

    if (!is_material && is_texture)
    {
        code = prc_internal_set_texture_style(ctx, global_data, header, &graph_material,
                                              style, &style->texture);
        if (code != 0)
        {
            return code;
        }
    }
    else if (is_material && !is_texture)
    {
        /* This is a material */
        style->material_type = PRC_TYPE_GRAPH_Material;

        if (material_index < global_data->material_count)
        {
            style->ambient_alpha = graph_material.ambient_alpha;
            style->diffuse_alpha = graph_material.diffuse_alpha;
            style->emissive_alpha = graph_material.emissive_alpha;
            style->specular_alpha = graph_material.specular_alpha;
            style->shininess = graph_material.shininess;

            if (graph_material.biased_ambient_index <= 0 ||
                graph_material.biased_diffuse_index <= 0 ||
                graph_material.biased_emissive_index <= 0 ||
                graph_material.biased_specular_index <= 0)
            {
                return PRC_ERROR_PARSE;
            }

            int32_t color_index = (graph_material.biased_ambient_index - 1) / 3;
            if (color_index < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index];
                style->ambient_color[0] = rgb_color.red;
                style->ambient_color[1] = rgb_color.green;
                style->ambient_color[2] = rgb_color.blue;
            }
            else
            {
                return PRC_ERROR_PARSE;
            }

            color_index = (graph_material.biased_diffuse_index - 1) / 3;
            if (color_index < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index];
                style->diffuse_color[0] = rgb_color.red;
                style->diffuse_color[1] = rgb_color.green;
                style->diffuse_color[2] = rgb_color.blue;
            }
            else
            {
                return PRC_ERROR_PARSE;
            }

            color_index = (graph_material.biased_emissive_index - 1) / 3;
            if (color_index < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index];
                style->emissive_color[0] = rgb_color.red;
                style->emissive_color[1] = rgb_color.green;
                style->emissive_color[2] = rgb_color.blue;
            }
            else
            {
                return PRC_ERROR_PARSE;
            }

            color_index = (graph_material.biased_specular_index - 1) / 3;
            if (color_index < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index];
                style->specular_color[0] = rgb_color.red;
                style->specular_color[1] = rgb_color.green;
                style->specular_color[2] = rgb_color.blue;
            }
            else
            {
                return PRC_ERROR_PARSE;
            }

            /* Make sure the texture related values are set */
            style->texture.next_texture_index = graph_material.biased_next_texture_index - 1;
            style->texture.texture_definition_index = graph_material.biased_texture_definition_index -1;
            style->texture.uv_coordinates_index = graph_material.biased_uv_coordinates_index - 1;
            style->texture.material_generic_index = graph_material.biased_material_generic_index - 1;
            style->texture.uv_coordinates_index = graph_material.biased_uv_coordinates_index - 1;
            style->texture.picture_data = NULL;
            style->texture.picture_height = 0;
            style->texture.picture_width = 0;
            style->texture.picture_index = -1;
            style->texture.num_elements_per_pixel = 0;
        }
        else
        {
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* This gets the tess or face color style AND returns the color we should use for the vertex.
   This has some redundancy in it. TODO. Simplify the structures a bit.  style_index
   dont_allow_texture is for the case when we are doing uncompressed with no texture entities.
   In this case, we want to ignore the texture part of the style and just use the material part.
*/
int
prc_internal_api_get_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, int32_t style_index, uint8_t *is_material, uint8_t *is_texture,
    prc_api_material *material, prc_internal_graph_style *style, float *color,
    float alpha, uint8_t dont_allow_texture)
{
    int32_t code;
    prc_graph_material prc_material;

    if (global_data->styles == NULL || style_index < 0)
    {
        /* No styles in global. Use a default
           diffuse: 192, 192, 192
           specular: 192, 192, 192
           ambient: 30, 30, 30
           emissive: 0, 0, 0
           shininess: 0.2
           opacity: 1.0       */
        color[0] = 192.0f / 255.0f;
        color[1] = 192.0f / 255.0f;
        color[2] = 192.0f / 255.0f;
        color[3] = 1.0f;
        style->ambient_alpha = 1.0f;
        style->diffuse_alpha = alpha;
        style->emissive_alpha = 1.0f;
        style->specular_alpha = 1.0f;
        style->shininess = 0.2f;
        style->ambient_color[0] = 30.0f / 255.0f;
        style->ambient_color[1] = 30.0f / 255.0f;
        style->ambient_color[2] = 30.0f / 255.0f;
        style->diffuse_color[0] = 192.0f / 255.0f;
        style->diffuse_color[1] = 192.0f / 255.0f;
        style->diffuse_color[2] = 192.0f / 255.0f;
        style->emissive_color[0] = 0.0f;
        style->emissive_color[1] = 0.0f;
        style->emissive_color[2] = 0.0f;
        style->specular_color[0] = 192.0f / 255.0f;
        style->specular_color[1] = 192.0f / 255.0f;
        style->specular_color[2] = 192.0f / 255.0f;
        *is_material = 1;
        *is_texture = 0;

        material->ambient_alpha = style->ambient_alpha;
        material->diffuse_alpha = style->diffuse_alpha * alpha;
        material->emissive_alpha = style->emissive_alpha;
        material->specular_alpha = style->specular_alpha;
        material->shininess = style->shininess;
        material->ambient[0] = style->ambient_color[0];
        material->ambient[1] = style->ambient_color[1];
        material->ambient[2] = style->ambient_color[2];
        material->diffuse[0] = style->diffuse_color[0];
        material->diffuse[1] = style->diffuse_color[1];
        material->diffuse[2] = style->diffuse_color[2];
        material->emissive[0] = style->emissive_color[0];
        material->emissive[1] = style->emissive_color[1];
        material->emissive[2] = style->emissive_color[2];
        material->specular[0] = style->specular_color[0];
        material->specular[1] = style->specular_color[1];
        material->specular[2] = style->specular_color[2];

        return 0;
    }
    /* style_index_in is biased */
    if (style_index < global_data->style_count && style_index > -1)
    {
        prc_graph_style graph_style = global_data->styles[style_index];

        if (global_data->styles[style_index].biased_color_index == 0)
        {
            return PRC_ERROR_PARSE;
        }

        int32_t color_index_unbiased = global_data->styles[style_index].biased_color_index - 1;

        if (graph_style.is_material)
        {
            /* Is a texture or a material. Just return white color */
            color[0] = 1.0f;
            color[1] = 1.0f;
            color[2] = 1.0f;
            color[3] = 1.0f;

            /* At this point, we have to get the */

            /* If this is a material then set the values in tessellation OR the
               face */
            if (graph_style.tag == PRC_TYPE_GRAPH_Material)
            {
                /* This is the surface materials case */
                *is_material = 1;
                *is_texture = 0;

                code = prc_internal_get_surface_material(ctx, global_data,
                    color_index_unbiased, material);
                if (code < 0)
                    return code;

                /* Make sure the style is set properly too */
                code = prc_internal_set_internal_style(ctx, global_data, header,
                    &graph_style, *is_material, *is_texture, style);
                if (code < 0)
                    return code;
            }
            else if (graph_style.tag == PRC_TYPE_GRAPH_TextureApplication)
            {
                /* This is a texture */
                if (dont_allow_texture)
                {
                    /* Use the generic material */
                    int32_t material_index = graph_style.biased_color_index - 1;
                    prc_graph_material graph_material;
                    if (material_index < 0 || material_index >= (int32_t)global_data->material_count)
                    {
                        return PRC_ERROR_PARSE;
                    }
                    graph_material = global_data->materials[material_index];
                    material_index = graph_material.biased_material_generic_index - 1;

                    /* This is the surface materials case */
                    *is_material = 1;
                    *is_texture = 0;

                    code = prc_internal_get_surface_material(ctx, global_data,
                                                             material_index, material);
                    if (code < 0)
                        return code;

                    /* Set the style values based upon what is in the material */
                    style->ambient_alpha = material->ambient_alpha;
                    style->diffuse_alpha = material->diffuse_alpha;
                    style->emissive_alpha = material->emissive_alpha;
                    style->specular_alpha = material->specular_alpha;
                    style->shininess = material->shininess;
                    memcpy(style->ambient_color, material->ambient, 3 * sizeof(float));
                    memcpy(style->diffuse_color, material->diffuse, 3 * sizeof(float));
                    memcpy(style->emissive_color, material->emissive, 3 * sizeof(float));
                    memcpy(style->specular_color, material->specular, 3 * sizeof(float));
                }
                else
                {
                    *is_material = 0;
                    *is_texture = 1;

                    code = prc_internal_set_internal_style(ctx, global_data, header,
                        &graph_style, *is_material, *is_texture, style);
                    if (code < 0)
                        return code;
                }
            }
            else if (graph_style.tag == PRC_TYPE_GRAPH_Style)
            {
                /* We don't know yet if it is something with material properites
                   or has a texture. We have to dig deeper. Get the material */
                if (color_index_unbiased < 0 || color_index_unbiased >= (int32_t)global_data->material_count)
                {
                    return PRC_ERROR_PARSE;
                }
                prc_material = global_data->materials[color_index_unbiased];
                if (prc_material.biased_texture_definition_index > 0)
                {

                    /* This is a texture */
                    if (dont_allow_texture)
                    {
                        /* Use the generic material */
                        int32_t material_index = prc_material.biased_material_generic_index - 1;

                        *is_material = 1;
                        *is_texture = 0;

                        code = prc_internal_get_surface_material(ctx, global_data,
                                                                 material_index, material);
                        if (code < 0)
                            return code;

                        /* Set the style values based upon what is in the material */
                        style->ambient_alpha = material->ambient_alpha;
                        style->diffuse_alpha = material->diffuse_alpha;
                        style->emissive_alpha = material->emissive_alpha;
                        style->specular_alpha = material->specular_alpha;
                        style->shininess = material->shininess;
                        memcpy(style->ambient_color, material->ambient, 3 * sizeof(float));
                        memcpy(style->diffuse_color, material->diffuse, 3 * sizeof(float));
                        memcpy(style->emissive_color, material->emissive, 3 * sizeof(float));
                        memcpy(style->specular_color, material->specular, 3 * sizeof(float));
                    }
                    else
                    {

                        *is_material = 0;
                        *is_texture = 1;

                        code = prc_internal_set_texture_style(ctx, global_data,
                            header, &prc_material, style, &style->texture);
                        if (code < 0)
                            return code;
                    }
                }
                else
                {
                    /* This is a material */
                    *is_material = 1;
                    *is_texture = 0;
                    code = prc_internal_get_surface_material(ctx, global_data,
                        color_index_unbiased, material);
                    if (code < 0)
                        return code;
                    /* Make sure the style is set properly too */
                    style->ambient_alpha = material->ambient_alpha;
                    style->diffuse_alpha = material->diffuse_alpha;
                    style->emissive_alpha = material->emissive_alpha;
                    style->specular_alpha = material->specular_alpha;
                    style->shininess = material->shininess;

                    memcpy(style->ambient_color, material->ambient, 3 * sizeof(float));
                    memcpy(style->diffuse_color, material->diffuse, 3 * sizeof(float));
                    memcpy(style->emissive_color, material->emissive, 3 * sizeof(float));
                    memcpy(style->specular_color, material->specular, 3 * sizeof(float));
                }
            }
        }
        else
        {
            /* This is a color */
            *is_material = 0;
            *is_texture = 0;
            color_index_unbiased = color_index_unbiased / 3;
            if (color_index_unbiased < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index_unbiased];
                color[0] = (float) rgb_color.red;
                color[1] = (float) rgb_color.green;
                color[2] = (float) rgb_color.blue;
                color[3] = (float) rgb_color.alpha;

                style->tint[0] = color[0];
                style->tint[1] = color[1];
                style->tint[2] = color[2];
                style->tint[3] = color[3];
            }
        }
        style->face_style_index = style_index;
    }
    else
    {
        return PRC_ERROR_PARSE;
    }
    return 0;
}

void
prc_copy_internal_graph_style_to_api(prc_context *ctx, prc_internal_graph_style *wire_style,
    prc_api_material *api_material)
{
    api_material->ambient_alpha = wire_style->ambient_alpha;
    api_material->diffuse_alpha = wire_style->diffuse_alpha;
    api_material->emissive_alpha = wire_style->emissive_alpha;
    api_material->specular_alpha = wire_style->specular_alpha;
    api_material->shininess = wire_style->shininess;
    api_material->ambient[0] = wire_style->ambient_color[0];
    api_material->ambient[1] = wire_style->ambient_color[1];
    api_material->ambient[2] = wire_style->ambient_color[2];
    api_material->emissive[0] = wire_style->emissive_color[0];
    api_material->emissive[1] = wire_style->emissive_color[1];
    api_material->emissive[2] = wire_style->emissive_color[2];
    api_material->diffuse[0] = wire_style->diffuse_color[0];
    api_material->diffuse[1] = wire_style->diffuse_color[1];
    api_material->diffuse[2] = wire_style->diffuse_color[2];
    api_material->specular[0] = wire_style->specular_color[0];
    api_material->specular[1] = wire_style->specular_color[1];
    api_material->specular[2] = wire_style->specular_color[2];
}
