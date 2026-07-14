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
#include <stdio.h>
#include <math.h>
#include "../include/prc_api.h"
#include "prc_internal_api.h"
#include "prc_internal_proto_api.h"

static int
prc_internal_api_face_has_vertex_colors(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data, uint32_t face_number)
{
    /* Make sure we are in a 3D tess type */
    if (uncompressed_data->tess->tess_type != PRC_TYPE_TESS_3D)
        return false;

    if (uncompressed_data->tess->tess_3d == NULL ||
        uncompressed_data->tess->tess_3d->face_tessellation_data == NULL)
        return false;

    if (face_number >= uncompressed_data->tess->tess_3d->number_of_face_tessellation)
        return false;

    /* Check if the face has vertex colors */
    if (uncompressed_data->tess->tess_3d->face_tessellation_data[face_number].has_vertex_colors)
        return true;

    return false;
}

/* After we have extracted the vertices from the uncompressed encoding list, we can
   finally compute the normals if needed. Right now, we only support this for
   the case of triangles, NOT fans or strips. */
int
prc_internal_api_calculate_normals_triangles(prc_context *ctx, uint32_t num_triangles,
                            prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    int code;
    uint32_t k;

    /* First we need to build the edge list */
    code = prc_internal_api_build_edge_list(ctx, num_triangles, uncompressed_data);
    if (code < 0)
        return code;

#if 0
    /* DEBUG. Print out all the vertex positions, normals and diffuse color */
    for (k = 0; k < uncompressed_data->vertex_out->num_vertices; k++)
    {
        printf("Vertex %d: Pos(%f, %f, %f) Normal(%f, %f, %f) Diffuse(%f, %f, %f) Color(%f, %f, %f, %f) Style(face_index=%d file_index=%d) tri_has_material = %d\n",
            k,
            uncompressed_data->vertex_out->vertices[k].position[0], uncompressed_data->vertex_out->vertices[k].position[1], uncompressed_data->vertex_out->vertices[k].position[2],
            uncompressed_data->vertex_out->vertices[k].normal[0], uncompressed_data->vertex_out->vertices[k].normal[1], uncompressed_data->vertex_out->vertices[k].normal[2],
            uncompressed_data->vertex_out->vertices[k].diffuse[0], uncompressed_data->vertex_out->vertices[k].diffuse[1], uncompressed_data->vertex_out->vertices[k].diffuse[2],
            uncompressed_data->vertex_out->vertices[k].color[0], uncompressed_data->vertex_out->vertices[k].color[1], uncompressed_data->vertex_out->vertices[k].color[2], uncompressed_data->vertex_out->vertices[k].color[3],
            uncompressed_data->vertex_out->vertices[k].style_index, uncompressed_data->vertex_out->vertices[k].style_file_index,
            uncompressed_data->vertex_out->vertices[k].tri_has_material);
    }

    /* And now print out the indices */
    for (k = 0; k < uncompressed_data->face_out_reserved->num_indices; k++)
    {
        printf("Index %d: %d\n", k, uncompressed_data->face_out_reserved->vertex_indices[k]);
    }
#endif

    /* Now we can compute the normals */
    code = prc_internal_api_compute_normals(ctx, uncompressed_data);
    if (code < 0)
        return code;

    prc_free(ctx, uncompressed_data->edge_list->edge);
    prc_free(ctx, uncompressed_data->edge_list);

    return 0;
}

static uint32_t*
prc_internal_api_get_normal_texture_position_index(uint32_t *index, int32_t *normal_index,
                                                   uint32_t *texture_indices,
                                                   uint32_t *position_index,
                                                   uint32_t number_of_texture_indices,
                                                   uint8_t must_calc_normals,
                                                   prc_multi_single_norm_type_t multi_single_norm_type)
{
    if (multi_single_norm_type == PRC_INTERNAL_MULTI_NORM)
    {
        /* Divide by 3.  As these indices are given as multiples of three */
        if (must_calc_normals)
        {
            *normal_index = -1;

            /* Not sure about the texture indices though. For now, don't divide by three as
               they are not x,y,z locations */
            for (uint32_t k = 0; k < number_of_texture_indices; k++)
            {
                texture_indices[k] = index[k];
            }
            *position_index = index[number_of_texture_indices] / 3;
            return index + number_of_texture_indices + 1;
        }
        else
        {
            *normal_index = index[0] / 3;

            /* Not sure about the texture indices though. For now, don't divide by three as
               they are not x,y,z locations */
            for (uint32_t k = 0; k < number_of_texture_indices; k++)
            {
                texture_indices[k] = index[k + 1];
            }
            *position_index = index[number_of_texture_indices + 1] / 3;
            return index + number_of_texture_indices + 2;
        }
    }
    else if (multi_single_norm_type == PRC_INTERNAL_SINGLE_NORM_INITIAL)
    {
        /* ONLY THIS VERY FIRST POINT HAS THE NORMAL */
        /* Divide by 3.  As these indices are given as multiples of three */
        *normal_index = index[0] / 3;

        /* Not sure about the texture indices though. For now, don't divide by three as
           they are not x,y,z locations */
        for (uint32_t k = 0; k < number_of_texture_indices; k++)
        {
            texture_indices[k] = index[k + 1];
        }
        *position_index = index[number_of_texture_indices + 1] / 3;
        return index + number_of_texture_indices + 2;
    }
    else
    {
        /* SINGLE_NORM_SUBSEQUENT - the normal is the same as the initial normal, so we just ignore it */
        /* Divide by 3.  As these indices are given as multiples of three */
        *normal_index = -1;
        /* Not sure about the texture indices though. For now, don't divide by three as
           they are not x,y,z locations */
        for (uint32_t k = 0; k < number_of_texture_indices; k++)
        {
            texture_indices[k] = index[k];
        }
        *position_index = index[number_of_texture_indices] / 3;
        return index + number_of_texture_indices + 1;
    }
}

static uint32_t*
prc_internal_api_get_normal_position_index(uint32_t *index, int32_t *normal_index,
                            uint32_t *position_index, uint8_t must_calc_normals,
                            prc_multi_single_norm_type_t multi_single_norm_type)
{
    if (multi_single_norm_type == PRC_INTERNAL_MULTI_NORM)
    {
        /* Divide by 3.  As these indices are given as multiples of three */
        if (must_calc_normals)
        {
            /* Per the spec, the normal is not stored */
            *normal_index = -1;
            *position_index = index[0] / 3;
            return index + 1;
        }
        else
        {
            *normal_index = index[0] / 3;
            *position_index = index[1] / 3;
            return index + 2;
        }
    }
    else if (multi_single_norm_type == PRC_INTERNAL_SINGLE_NORM_INITIAL)
    {
        /* ONLY THIS VERY FIRST POINT HAS THE NORMAL */
        /* Divide by 3.  As these indices are given as multiples of three */
        *normal_index = index[0] / 3;
        *position_index = index[1] / 3;
        return index + 2;
    }
    else
    {
        /* SINGLE_NORM_SUBSEQUENT - the normal is the same as the initial normal, so we just ignore it */
        /* Divide by 3.  As these indices are given as multiples of three */
        *normal_index = -1;
        *position_index = index[0] / 3;
        return index + 1;
    }
}

/* A helper for comparing textures in vertices */
static uint8_t
prc_internal_api_compare_texture_indices(uint32_t *index, uint32_t *texture_indices,
    uint32_t number_of_texture_indices)
{
    for (uint32_t k = 0; k < number_of_texture_indices; k++)
    {
        if (index[k] != texture_indices[k])
            return false;
    }
    return true;
}

/* A helper for comparing textures in vertices */
static uint8_t
prc_internal_api_check_texture(uint32_t *texture_indices,
    prc_internal_api_position_normal_pair *position_normal_pair,
    uint32_t num_text_indices, uint8_t has_texture)
{
    if (has_texture)
    {
        if (position_normal_pair->texture_set == PRC_INTERNAL_API_TEXTURE_SET)
        {
            uint8_t match = prc_internal_api_compare_texture_indices(texture_indices,
                position_normal_pair->prc_texture_index, num_text_indices);

            return match;
        }
    }
    return true;
}

/* Search through and see if we have an existing vertex that has this normal, position,
   color, texture etc. If not, add it as a new unique vertex. This vertex has a texture
   if *texture_indices is not NULL.  Note that we may need to compute the normals
   after this method is used. That will involve building an edge list and doing
   vertex splitting. We will be ignoring the normals here is they are not provided.
   Normal computation is required if normal_index = -1, *normals == NULL and
   uncompressed_data->must_calc_normals is true (All three of these should be true
   -- if not then something has gone wrong) */
static int
prc_internal_api_get_vertex_index(prc_context *ctx, size_t *vertex_index,
    uint32_t *texture_indices, uint32_t num_text_indices,
    prc_internal_api_uncomm_tess_data *uncompressed_data,
    uint32_t indice_count)
{
    prc_internal_api_face *face_out_reserved = uncompressed_data->face_out_reserved;
    prc_file_struct_internal_global_data *global_data = uncompressed_data->global_data;
    prc_internal_api_position_normal_pair *position_normal_pair;
    prc_internal_api_position_normal_lookup *lut = &uncompressed_data->position_normal_lut;
    float *face_initial_positions = uncompressed_data->face_initial_positions;
    float *face_initial_normals = uncompressed_data->face_initial_normals;
    float *face_initial_texture_coords = uncompressed_data->face_initial_texture_coords;
    uint32_t *face_position_indices = uncompressed_data->face_position_indices;
    uint32_t *face_normal_indices = uncompressed_data->face_normal_indices;
    uint32_t *face_texture_indices = uncompressed_data->face_texture_indices;
    uint32_t *face_vertex_color_indices = uncompressed_data->face_vertex_color_indices;
    uint8_t has_normals = face_initial_normals != NULL;
    uint8_t has_texture = face_initial_texture_coords != NULL;
    prc_api_tess_vertex_buffer *vertex_out = uncompressed_data->vertex_out;
    uint8_t has_vertex_colors = (uncompressed_data->decoded_colors != NULL) ? true : false;
    int code;
    uint32_t position_index;
    uint32_t normal_index = 0;
    uint32_t texture_index = 0;
    uint32_t decode_color_offset;
    uint8_t has_pure_color = uncompressed_data->has_pure_color;

    position_index = face_position_indices[indice_count];
    position_normal_pair = &lut->position_normal_pair[position_index];

    /* Check if this is the FIRST time encountering this position */
    if (position_normal_pair->api_vertex_index == UINT32_MAX)
    {
        /* This PRC position has NOT been encountered yet.
           Create a new vertex at vertex_out->num_vertices. */
        if (vertex_out->num_vertices >= vertex_out->capacity)
        {
            prc_api_vertex *new_vertices;
            vertex_out->capacity = vertex_out->capacity * 2;
            new_vertices = (prc_api_vertex *)prc_realloc(ctx,
                vertex_out->vertices, vertex_out->capacity * sizeof(prc_api_vertex));
            if (new_vertices == NULL)
                return PRC_API_ERROR_MEMORY;
            vertex_out->vertices = new_vertices;

            prc_internal_api_initialize_vertex(ctx, vertex_out);
        }

        /* Create new API vertex at current num_vertices */
        uint32_t new_api_idx = (uint32_t)vertex_out->num_vertices;

        /* Copy position from face_initial_positions */
        vertex_out->vertices[new_api_idx].position[0] = face_initial_positions[position_index * 3];
        vertex_out->vertices[new_api_idx].position[1] = face_initial_positions[position_index * 3 + 1];
        vertex_out->vertices[new_api_idx].position[2] = face_initial_positions[position_index * 3 + 2];

        /* Set normal if available, otherwise mark as needing computation */
        if (has_normals)
        {
            normal_index = face_normal_indices[indice_count];
            vertex_out->vertices[new_api_idx].normal[0] = face_initial_normals[normal_index * 3];
            vertex_out->vertices[new_api_idx].normal[1] = face_initial_normals[normal_index * 3 + 1];
            vertex_out->vertices[new_api_idx].normal[2] = face_initial_normals[normal_index * 3 + 2];
            vertex_out->vertices[new_api_idx].normal_set = true;
            position_normal_pair->normal_set = PRC_INTERNAL_API_NORM_SET;
        }
        else
        {
            vertex_out->vertices[new_api_idx].normal_set = false;
            position_normal_pair->normal_set = PRC_INTERNAL_API_NORM_MUST_BE_COMPUTED;
        }

        /* Set texture coordinates if present. Warning may be an issue here
           if num_text_indices > 1 */
        if (num_text_indices > 1)
            return PRC_API_ERROR_PARSER;

        if (has_texture)
        {
            texture_index = face_texture_indices[indice_count];

            double raw_u = face_initial_texture_coords[texture_index * 2];
            double raw_v = face_initial_texture_coords[texture_index * 2 + 1];

            prc_internal_api_set_vertex_texture_coords(ctx, &vertex_out->vertices[new_api_idx],
                raw_u, raw_v, uncompressed_data->texture_transform,
                uncompressed_data->has_texture_transform);

            /* Store texture indices in LUT */
            for (uint32_t k = 0; k < num_text_indices; k++)
            {
                position_normal_pair->prc_texture_index[k] = texture_indices[k];
            }
            position_normal_pair->texture_set = PRC_INTERNAL_API_TEXTURE_SET;
            position_normal_pair->prc_num_texture_indices = num_text_indices;

            /* Textured vertices should be white */
            vertex_out->vertices[new_api_idx].color[0] = 1.0f;
            vertex_out->vertices[new_api_idx].color[1] = 1.0f;
            vertex_out->vertices[new_api_idx].color[2] = 1.0f;
            vertex_out->vertices[new_api_idx].color[3] = 1.0f;
        }
        else
        {
            position_normal_pair->texture_set = PRC_INTERNAL_API_NO_TEXTURE;

            /* Set color based on material/style */
            if (uncompressed_data->is_material && !has_vertex_colors)
            {
                /* Material-only faces: white color so material diffuse shows through */
                vertex_out->vertices[new_api_idx].color[0] = 1.0f;
                vertex_out->vertices[new_api_idx].color[1] = 1.0f;
                vertex_out->vertices[new_api_idx].color[2] = 1.0f;
                vertex_out->vertices[new_api_idx].color[3] = 1.0f;
            }
            else if (has_vertex_colors)
            {
                decode_color_offset = indice_count * 4;
                memcpy(vertex_out->vertices[new_api_idx].color,
                    &uncompressed_data->decoded_colors[decode_color_offset],
                    4 * sizeof(float));
                position_normal_pair->prc_vertex_color_index = face_vertex_color_indices[indice_count];

                /* To get pure color with how I currently have the generic.frag
                   set up do the following */
                vertex_out->vertices[new_api_idx].diffuse[0] = 1.0;
                vertex_out->vertices[new_api_idx].diffuse[1] = 1.0;
                vertex_out->vertices[new_api_idx].diffuse[2] = 1.0;

                vertex_out->vertices[new_api_idx].emissive[0] = 0.0;
                vertex_out->vertices[new_api_idx].emissive[1] = 0.0;
                vertex_out->vertices[new_api_idx].emissive[2] = 0.0;

                vertex_out->vertices[new_api_idx].tint[0] = 0.0;
                vertex_out->vertices[new_api_idx].tint[1] = 0.0;
                vertex_out->vertices[new_api_idx].tint[2] = 0.0;

                vertex_out->vertices[new_api_idx].specular[0] = 0.0;
                vertex_out->vertices[new_api_idx].specular[1] = 0.0;
                vertex_out->vertices[new_api_idx].specular[2] = 0.0;
            }
            else
            {
                if (!has_pure_color)
                {
                    /* If we don't have a pure color, then we will use the style color */
                    /* Get color from style */
                    code = prc_internal_api_get_color_from_style(ctx, global_data,
                        face_out_reserved->style->face_style_index,
                        vertex_out->vertices[new_api_idx].color);
                    if (code < 0)
                        return code;
                }
                else
                {
                    /* If we have a pure color, then we will use that */
                    vertex_out->vertices[new_api_idx].color[0] = uncompressed_data->pure_color[0];
                    vertex_out->vertices[new_api_idx].color[1] = uncompressed_data->pure_color[1];
                    vertex_out->vertices[new_api_idx].color[2] = uncompressed_data->pure_color[2];
                    vertex_out->vertices[new_api_idx].color[3] = uncompressed_data->pure_color[3];
                }
            }
        }

        /* Initialize LUT entry for this position */
        position_normal_pair->api_vertex_index = new_api_idx;
        position_normal_pair->prc_position_index = position_index;
        position_normal_pair->prc_normal_index = normal_index;
        position_normal_pair->style_index = face_out_reserved->style->face_style_index;
        position_normal_pair->next = NULL;

        vertex_out->num_vertices++;

        /* Return the newly created vertex index */
        *vertex_index = new_api_idx;
        return 0;
    }

    /* The PRC position has been encountered before. Check if we need a duplicate
       based on normal/texture/color/style differences. */
    if (has_normals || has_texture || has_vertex_colors)
    {
        /* Search through this current linked list for a matching vertex */
        prc_internal_api_position_normal_pair *current_list = position_normal_pair;
        int got_hit = false;

        while (!got_hit)
        {
            int got_texture_hit = prc_internal_api_check_texture(texture_indices,
                current_list, num_text_indices, has_texture);

            int got_normal_hit;
            if (has_normals)
            {
                got_normal_hit = (current_list->prc_normal_index == face_normal_indices[indice_count]);
            }
            else
            {
                got_normal_hit = true; /* Normal not an issue in matching */
            }

            int got_color_hit = true;
            if (has_vertex_colors)
            {
                /* All the color vertices have a color assigned, so we can't use this for matching */
                got_color_hit = false;
            }

            if (got_normal_hit && got_texture_hit && got_color_hit)
            {
                /* Found a match - reuse existing vertex */
                *vertex_index = current_list->api_vertex_index;
                return 0;
            }

            if (current_list->next == NULL)
                break; /* End of list, need to create duplicate */

            /* Move to the next vertex in our list and check that one */
            current_list = current_list->next;
        }

        /* No match found - create a new vertex for this position */
        if (vertex_out->num_vertices >= vertex_out->capacity)
        {
            prc_api_vertex *new_vertices;
            vertex_out->capacity = vertex_out->capacity * 2;
            new_vertices = (prc_api_vertex *)prc_realloc(ctx,
                vertex_out->vertices, vertex_out->capacity * sizeof(prc_api_vertex));
            if (new_vertices == NULL)
                return PRC_API_ERROR_MEMORY;
            vertex_out->vertices = new_vertices;

            prc_internal_api_initialize_vertex(ctx, vertex_out);
        }

        /* Create a new index with that position location */
        uint32_t new_dup_idx = (uint32_t)vertex_out->num_vertices;
        uint32_t base_api_idx = current_list->api_vertex_index;

        /* Set the position */
        vertex_out->vertices[new_dup_idx].position[0] = face_initial_positions[position_index * 3];
        vertex_out->vertices[new_dup_idx].position[1] = face_initial_positions[position_index * 3 + 1];
        vertex_out->vertices[new_dup_idx].position[2] = face_initial_positions[position_index * 3 + 2];

        /* Set normal for duplicate if we already have the normals */
        if (has_normals)
        {
            normal_index = face_normal_indices[indice_count];
            vertex_out->vertices[new_dup_idx].normal[0] = face_initial_normals[normal_index * 3];
            vertex_out->vertices[new_dup_idx].normal[1] = face_initial_normals[normal_index * 3 + 1];
            vertex_out->vertices[new_dup_idx].normal[2] = face_initial_normals[normal_index * 3 + 2];
            vertex_out->vertices[new_dup_idx].normal_set = true;
        }

        /* Set texture/color for the duplicate */
        if (has_texture)
        {
            texture_index = face_texture_indices[indice_count];
            double raw_u = face_initial_texture_coords[texture_index * 2];
            double raw_v = face_initial_texture_coords[texture_index * 2 + 1];

            prc_internal_api_set_vertex_texture_coords(ctx, &vertex_out->vertices[new_dup_idx],
                raw_u, raw_v, uncompressed_data->texture_transform,
                uncompressed_data->has_texture_transform);

            /* Textured vertices should be white */
            vertex_out->vertices[new_dup_idx].color[0] = 1.0f;
            vertex_out->vertices[new_dup_idx].color[1] = 1.0f;
            vertex_out->vertices[new_dup_idx].color[2] = 1.0f;
            vertex_out->vertices[new_dup_idx].color[3] = 1.0f;
        }
        else
        {
            /* Copy color from base or set from style */
            if (has_vertex_colors)
            {
                decode_color_offset = indice_count * 4;
                memcpy(vertex_out->vertices[new_dup_idx].color,
                    &uncompressed_data->decoded_colors[decode_color_offset],
                    4 * sizeof(float));
            }
            else
            {
                if (!has_pure_color)
                {
                    /* If we don't have a pure color, then we will use the style color */
                    /* Get color from style */
                    code = prc_internal_api_get_color_from_style(ctx, global_data,
                        face_out_reserved->style->face_style_index,
                        vertex_out->vertices[new_dup_idx].color);
                    if (code < 0)
                        return code;
                }
                else
                {
                    /* If we have a pure color, then we will use that */
                    vertex_out->vertices[new_dup_idx].color[0] = uncompressed_data->pure_color[0];
                    vertex_out->vertices[new_dup_idx].color[1] = uncompressed_data->pure_color[1];
                    vertex_out->vertices[new_dup_idx].color[2] = uncompressed_data->pure_color[2];
                    vertex_out->vertices[new_dup_idx].color[3] = uncompressed_data->pure_color[3];
                }
            }
        }

        vertex_out->num_vertices++;

        /* Add the new item to our linked list */
        current_list->next = (prc_internal_api_position_normal_pair *)prc_calloc(ctx, 1,
            sizeof(prc_internal_api_position_normal_pair));
        if (current_list->next == NULL)
            return PRC_API_ERROR_MEMORY;

        current_list->next->api_vertex_index = new_dup_idx;
        current_list->next->prc_position_index = position_index;
        current_list->next->prc_normal_index = normal_index;
        current_list->next->normal_set = has_normals ? PRC_INTERNAL_API_NORM_SET : PRC_INTERNAL_API_NORM_MUST_BE_COMPUTED;
        current_list->next->style_index = face_out_reserved->style->face_style_index;
        current_list->next->next = NULL;

        /* And set the texture indices if we have them */
        if (has_texture)
        {
            for (uint32_t k = 0; k < num_text_indices; k++)
            {
                current_list->next->prc_texture_index[k] = texture_indices[k];
            }
            current_list->next->texture_set = PRC_INTERNAL_API_TEXTURE_SET;
        }

        *vertex_index = new_dup_idx;
        return 0;
    }

    /* Simpler case: first normal/texture assignment */
    *vertex_index = position_normal_pair->api_vertex_index;
    return 0;
}

/* This function gets the normal and position indices of the 3D uncompressed
   tessellation data for a face and maps it into a new list of vertices and
   indices.  This is operating on one point, updating the position and normal
   vectors as well as the indice value. If we are in a single norm situation,
   then we pass a flag and that initial norm index that was returned the
   first time the function was called.  The flag lets us know what the situation
   is (e.g MULTI_NORM, SINGLE_NORM_INITIAL, SINGLE_NORM_SUBSEQUENT) */
static int
prc_internal_api_remap_indices(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data,
    uint8_t has_texture_indices, uint32_t indice_count,
    prc_multi_single_norm_type_t multi_single_norm_type,
    int32_t *single_norm_initial_index)
{
    int32_t normal_index;
    uint32_t position_index;
    uint8_t must_calculate_normals = uncompressed_data->must_calculate_normals;
    uint32_t *prc_vertex_indice_to_api_vertex_indice = uncompressed_data->prc_vertex_indice_to_api_vertex_indice;
    uint32_t *prc_normal_indice_to_api_normal_indice = uncompressed_data->prc_normal_indice_to_api_normal_indice;
    uint32_t *prc_texture_indice_to_api_texture_indice = uncompressed_data->prc_texture_indice_to_api_texture_indice;
    float *face_initial_positions = uncompressed_data->face_initial_positions;
    float *face_initial_normals = uncompressed_data->face_initial_normals;
    float *face_initial_texture_coords = uncompressed_data->face_initial_texture_coords;
    uint32_t point_indice_pos;
    uint32_t normal_indice_pos;
    uint32_t texture_indice_pos;
    uint32_t *position_count = &uncompressed_data->face_num_initial_positions;
    uint32_t *normal_count = &uncompressed_data->face_num_initial_normals;
    uint32_t *texture_count = &uncompressed_data->face_num_initial_texture_coords;
    uint32_t k;
    uint32_t texture_indices[MAX_NUM_TEXTURE_COORDS];
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t num_vertices_prc = uncompressed_data->tess->num_vertices_internal;
    uint32_t num_normals_prc = uncompressed_data->tess->num_normals_internal;

    /* This just gets the indices for the normal, position, and texture */
    if (has_texture_indices)
    {
        *(uncompressed_data->src_index_data) =
            prc_internal_api_get_normal_texture_position_index(*(uncompressed_data->src_index_data),
                &normal_index, texture_indices, &position_index, num_text_coord,
                must_calculate_normals, multi_single_norm_type);
    }
    else
    {
        *(uncompressed_data->src_index_data) =
            prc_internal_api_get_normal_position_index(*(uncompressed_data->src_index_data),
                &normal_index, &position_index, must_calculate_normals, multi_single_norm_type);
    }

    /* Sanity check for malicious file data */
    if (position_index >= num_vertices_prc)
        return PRC_API_ERROR_PARSER;

    if (normal_index != -1 && normal_index >= num_normals_prc)
        return PRC_API_ERROR_PARSER;

    /* Now see if these indices are already mapped */
    if (prc_vertex_indice_to_api_vertex_indice[position_index] == UINT32_MAX)
    {
        prc_vertex_indice_to_api_vertex_indice[position_index] = *position_count;
        point_indice_pos = *position_count;
        /* Add the new position that we have not had before */
        for (k = 0; k < 3; k++)
        {
            face_initial_positions[point_indice_pos * 3 + k] =
                uncompressed_data->tess->vertices_internal[position_index].position[k];
        }
        *position_count = *position_count + 1;
    }
    else
    {
        /* It already has a position. Get the indice for this in the new data */
        point_indice_pos = prc_vertex_indice_to_api_vertex_indice[position_index];
    }
    /* Add the index into the new data */
    uncompressed_data->face_position_indices[indice_count] = point_indice_pos;


    /* Normal handling - only when normals are present in input (i.e. we do not
       need to compute them later) */
    if (!must_calculate_normals)
    {
        if (multi_single_norm_type == PRC_INTERNAL_MULTI_NORM ||
            multi_single_norm_type == PRC_INTERNAL_SINGLE_NORM_INITIAL)
        {
            if (prc_normal_indice_to_api_normal_indice[normal_index] == UINT32_MAX)
            {
                /* New normal: assign next available normal index and copy values */
                prc_normal_indice_to_api_normal_indice[normal_index] = *normal_count;
                normal_indice_pos = *normal_count;
                for (k = 0; k < 3; k++)
                {
                    face_initial_normals[normal_indice_pos * 3 + k] =
                        uncompressed_data->normals_internal[normal_index].normal[k];
                }
                (*normal_count)++;
            }
            else
            {
                /* Existing normal mapping */
                normal_indice_pos = prc_normal_indice_to_api_normal_indice[normal_index];
            }

            /* Add the index into the new data */
            uncompressed_data->face_normal_indices[indice_count] = normal_indice_pos;

            if (multi_single_norm_type == PRC_INTERNAL_SINGLE_NORM_INITIAL)
            {
                /* Stash the index of our one normal */
                *single_norm_initial_index = normal_index;
            }
        }
        else
        {
            /* Use the first normal index which we stashed from the first decode */
            normal_indice_pos = prc_normal_indice_to_api_normal_indice[*single_norm_initial_index];
            uncompressed_data->face_normal_indices[indice_count] = normal_indice_pos;
        }
    }

    /* It is possible that we have texture indices but no texture! The
       auto engine has examples like this. In this case we skip the texture */
    if (uncompressed_data->is_texture && has_texture_indices)
    {
        /* There may be an issue here if we have multiple texture coordinates
           Waiting to find a file with this. For now, we are just using the first coordinate */
        uint32_t texture_index = texture_indices[0] / 2;
        uint32_t num_texture_coords_prc = uncompressed_data->tess->tess_3d->number_of_texture_coordinates / 2;

        /* Sanity check for malicious file data. prc_texture_indice_to_api_texture_indice
           and texture_coordinates are both sized off number_of_texture_coordinates/2, but
           texture_index comes straight from the file's index stream with no relation to
           that count, so validate it here (mirrors the position_index/normal_index checks
           above). */
        if (texture_index >= num_texture_coords_prc)
            return PRC_API_ERROR_PARSER;

        if (prc_texture_indice_to_api_texture_indice[texture_index] == UINT32_MAX)
        {
            prc_texture_indice_to_api_texture_indice[texture_index] = *texture_count;
            texture_indice_pos = *texture_count;

            /* Add the new texture coordinate that we have not had before */
            for (k = 0; k < 2; k++)
            {
                face_initial_texture_coords[texture_indice_pos * 2 + k] =
                    uncompressed_data->tess->tess_3d->texture_coordinates[texture_index * 2 + k];
            }
            *texture_count = *texture_count + 1;
        }
        else
        {
            /* It already has texture coordinates. Get the indice for this in the new data */
            texture_indice_pos = prc_texture_indice_to_api_texture_indice[texture_index];
        }
        uncompressed_data->face_texture_indices[indice_count] = texture_indice_pos;
    }

    return 0;
}

/* This has to be done after we have added ALL the different types of
   triangles, stripes and fans. I used to do this with each one but that
   will not work. */
int
prc_internal_uncompressed_create_vertices(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    uint32_t indice_count = uncompressed_data->index_count;
    int code;
    uint32_t k;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;

    /* Now we want to create the vertices (which include the location, texture,
       and normal) and the single list of indices to those vertices. */
    for (k = 0; k < indice_count; k++)
    {
        code = prc_internal_api_get_vertex_index(ctx, &vertex_index,
            uncompressed_data->face_texture_indices, num_text_coord,
            uncompressed_data, k);
        if (code < 0)
            return code;

        vertex_indices[k] = (uint32_t)vertex_index;
    }
    return 0;
}

/* PRC_FACETESSDATA_Triangle and PRC_FACETESSDATA_TriangleTextured */
int
prc_internal_api_vertex_triangle_multinorm(prc_context *ctx, size_t num_triangles,
                uint8_t has_texture, prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;

    if (num_triangles == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
       a new set that are specific for this face */
    for (k = 0; k < num_triangles; k++)
    {
        /* Sets of three vertices */
        for (j = 0; j < 3; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx,
                    uncompressed_data, has_texture, indice_count,
                    PRC_INTERNAL_MULTI_NORM, NULL);
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* PRC_FACETESSDATA_TriangleFan */
/* PRC_FACETESSDATA_TriangleFanTextured */
int
prc_internal_api_vertex_fan_multinorm(prc_context *ctx, size_t num_fans,
    size_t *fan_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;

    if (num_fans == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
   a new set that are specific for this face */
    for (k = 0; k < num_fans; k++)
    {
        size_t num_points = fan_offsets[2 * k + 1];

        /* Sets of three vertices */
        for (j = 0; j < num_points; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx, uncompressed_data,
                    has_texture, indice_count, PRC_INTERNAL_MULTI_NORM, NULL);
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* PRC_FACETESSDATA_TriangleStripe */
/* PRC_FACETESSDATA_TriangleStripeTextured */
int
prc_internal_api_vertex_strip_multinorm(prc_context *ctx, size_t num_strips,
    size_t *strip_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;

    if (num_strips == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
       a new set that are specific for this face */
    for (k = 0; k < num_strips; k++)
    {
        size_t num_points = strip_offsets[2 * k + 1];

        /* Sets of three vertices */
        for (j = 0; j < num_points; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx, uncompressed_data, has_texture,
                    indice_count, PRC_INTERNAL_MULTI_NORM, NULL);
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* One normal for face. with or without texture */
/* PRC_FACETESSDATA_TriangleOneNormal */
int
prc_internal_api_vertex_triangle_one_norm(prc_context *ctx,
    size_t num_triangles, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    /* In this case we first read a single normal indice and then all the
       triangle indice */
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;
    prc_multi_single_norm_type_t single_norm_state;
    int32_t single_norm_initial_index = -1;

    if (num_triangles == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
       a new set that are specific for this face */
    for (k = 0; k < num_triangles; k++)
    {
        single_norm_state = PRC_INTERNAL_SINGLE_NORM_INITIAL;

        /* Sets of three vertices each with the same norm */
        for (j = 0; j < 3; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx,
                    uncompressed_data, has_texture, indice_count,
                    single_norm_state, &single_norm_initial_index);
            single_norm_state = PRC_INTERNAL_SINGLE_NORM_SUBSEQUENT;
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* PRC_FACETESSDATA_TriangleFanOneNormal. Might not really have one normal
   per face */
int
prc_internal_api_vertex_fan_one_norm(prc_context *ctx,
    size_t num_fans, size_t *fan_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;
    prc_multi_single_norm_type_t single_norm_state;
    int32_t single_norm_initial_index = -1;

    if (num_fans == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
       a new set that are specific for this face */
    for (k = 0; k < num_fans; k++)
    {
        /* First read gets 2 and then the rest each get 1 */
        size_t num_points = fan_offsets[2 * k + 1];
        single_norm_state = PRC_INTERNAL_SINGLE_NORM_INITIAL;

        for (j = 0; j < num_points; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx, uncompressed_data,
                    has_texture, indice_count, single_norm_state,
                    &single_norm_initial_index);
            single_norm_state = PRC_INTERNAL_SINGLE_NORM_SUBSEQUENT;
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* PRC_FACETESSDATA_TriangleStripeOneNormal  Might not really have one normal
   per face */
int
prc_internal_api_vertex_strip_one_norm(prc_context *ctx,
    size_t num_strips, size_t *strip_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    size_t k, j;
    int code;
    uint32_t indice_count = uncompressed_data->index_count;
    size_t vertex_index;
    uint32_t num_text_coord = uncompressed_data->face_out_reserved->num_texture_coords;
    uint32_t *vertex_indices = uncompressed_data->face_out_reserved->vertex_indices;
    prc_multi_single_norm_type_t single_norm_state;
    int32_t single_norm_initial_index = -1;

    if (num_strips == 0)
        return 0;

    /* First go through all our positions and normals and remap them to
       a new set that are specific for this face */
    for (k = 0; k < num_strips; k++)
    {
        /* First read gets 2 and then the rest each get 1 */
        size_t num_points = strip_offsets[2 * k + 1];
        single_norm_state = PRC_INTERNAL_SINGLE_NORM_INITIAL;

        for (j = 0; j < num_points; j++)
        {
            /* Remap normal and point to new structure for easier splitting etc. */
            code =
                prc_internal_api_remap_indices(ctx, uncompressed_data, has_texture,
                    indice_count, single_norm_state, &single_norm_initial_index);
            single_norm_state = PRC_INTERNAL_SINGLE_NORM_SUBSEQUENT;
            if (code < 0)
                return code;
            indice_count++;
        }
    }
    uncompressed_data->index_count = indice_count;

    return 0;
}

/* Deal with the fans on a single face. This gets the indices into the triangulation data */
int
prc_internal_api_set_fans(prc_context *ctx, prc_tess_face face,
                          uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
                          size_t *face_tessellation_index, size_t *num_indices)
{
    size_t k;

    entities->num_fans = face.triangulateddata[*face_tessellation_index];
    *face_tessellation_index = *face_tessellation_index + 1;

    if (entities->num_fans > 0)
    {
        /* Fan offset into the index array followed by the number of indices for that fan. */
        entities->fan_offsets = (size_t *)prc_calloc(ctx, entities->num_fans * 2, sizeof(size_t));
        if (entities->fan_offsets == NULL)
            return PRC_API_ERROR_MEMORY;

        for (k = 0; k < entities->num_fans; k++)
        {
            /* The offset for this fan in the indices */
            entities->fan_offsets[2 * k] = *num_indices;

            /* The number of indices in the fan */
            if (is_single_norm)
            {
                /* Apply the crazy mask */
                uint32_t num_indices_for_fan = (face.triangulateddata[*face_tessellation_index] & ~PRC_FACETESSDATA_NORMAL_Single);
                entities->fan_offsets[2 * k + 1] = num_indices_for_fan;
                *num_indices = *num_indices + num_indices_for_fan;
            }
            else
            {
                entities->fan_offsets[2 * k + 1] = face.triangulateddata[*face_tessellation_index];
                *num_indices = *num_indices + face.triangulateddata[*face_tessellation_index];
            }
            *face_tessellation_index = *face_tessellation_index + 1;
        }
    }
    return 0;
}

/* Deal with the strips on a single face. This gets the indices into the triangulation data */
int
prc_internal_api_set_strips(prc_context *ctx, prc_tess_face face,
                    uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
                            size_t *face_tessellation_index, size_t *num_indices)
{
    size_t k;

    entities->num_strips = face.triangulateddata[*face_tessellation_index];
    *face_tessellation_index = *face_tessellation_index + 1;

    if (entities->num_strips > 0)
    {
        /* Strip offset into the index array followed by the number of indices for that strip. */
        entities->strip_offsets = (size_t *)prc_calloc(ctx, entities->num_strips * 2, sizeof(size_t));
        if (entities->strip_offsets == NULL)
            return PRC_API_ERROR_MEMORY;

        for (k = 0; k < entities->num_strips; k++)
        {
            /* The offset for this strip in the indices */
            entities->strip_offsets[2 * k] = *num_indices;

            /* The number of indices in the strip */
            if (is_single_norm)
            {
                /* Deal with the crazy mask */
                uint32_t num_indices_for_strip = (face.triangulateddata[*face_tessellation_index] & ~PRC_FACETESSDATA_NORMAL_Single);
                entities->strip_offsets[2 * k + 1] = num_indices_for_strip;
                *num_indices = *num_indices + num_indices_for_strip;
            }
            else
            {
                entities->strip_offsets[2 * k + 1] = face.triangulateddata[*face_tessellation_index];
                *num_indices = *num_indices + face.triangulateddata[*face_tessellation_index];
            }
            *face_tessellation_index = *face_tessellation_index + 1;
        }
    }
    return 0;
}

/* Deal with the triangles on a single face. This gets the indices into the
   triangulation data */
void
prc_internal_api_set_triangles(prc_context *ctx, prc_tess_face face,
                    uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
                               size_t *face_tessellation_index, size_t *num_indices)
{
	entities->num_triangles = face.triangulateddata[0];
	*num_indices = entities->num_triangles * 3; /* 3 indices per triangle */
 	*face_tessellation_index = *face_tessellation_index + 1;
}
