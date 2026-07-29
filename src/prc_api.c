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

#include "prc_internal_api.h"
#include "prc_api_debug.h"
#include "prc_data.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h> /* for UINT32_MAX */

/* Reasonable guard to avoid unbounded allocations. Tune as needed. */
#define PRC_STYLE_POOL_MAX_CAPACITY (1000000u)


#define PARTS_DETAIL_INIT_SIZE 100

/* Iterative (not recursive): the style tree's depth follows the assembly/RI
   tree nesting in the file, which is attacker-controlled and could otherwise
   drive unbounded C-stack recursion. Uses an explicit heap-allocated work
   list instead of the call stack. Visit order doesn't matter here -- each
   node only frees its own children array, independent of sibling/descendant
   state -- so a simple stack (LIFO) work list is sufficient. */
void
prc_api_release_style_tree(prc_context *ctx, prc_api_object_style *node)
{
    prc_api_object_style **worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity;

    if (node == NULL)
        return;

    worklist_capacity = 64;
    worklist = (prc_api_object_style **)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_object_style *));
    if (worklist == NULL)
        return;
    worklist[worklist_size++] = node;

    while (worklist_size > 0)
    {
        prc_api_object_style *curr;
        uint32_t k;

        curr = worklist[--worklist_size];
        if (curr == NULL)
            continue;

        for (k = 0; k < curr->num_children; k++)
        {
            if (worklist_size >= worklist_capacity)
            {
                prc_api_object_style **new_worklist;
                worklist_capacity *= 2;
                new_worklist = (prc_api_object_style **)prc_realloc(ctx, worklist,
                    worklist_capacity * sizeof(prc_api_object_style *));
                if (new_worklist == NULL)
                {
                    prc_free(ctx, worklist);
                    return;
                }
                worklist = new_worklist;
            }
            worklist[worklist_size++] = curr->children[k];
        }

        if (curr->children != NULL)
        {
            prc_free(ctx, curr->children);
            curr->children = NULL;
        }
        curr->num_children = 0;
    }

    prc_free(ctx, worklist);
}

PRC_EXPORT prc_context*
prc_api_new_context(const prc_hooks *hooks)
{
    return prc_new_context(hooks);
}

PRC_EXPORT int
prc_api_release_context(prc_context *ctx)
{
    int release_code = 0;
    prc_release_context(ctx);
#if PRC_DEBUG_MEMORY
    if (ctx != NULL)
    {
        uint8_t has_leak = 0;
        size_t k;
        for (k = 0; k < ctx->current_memory_index; k++)
        {
            if (ctx->debug_memory[k].is_free == 0)
            {
                has_leak = 1;
                fprintf(stderr, "Memory leak at debug_index=%zu alloc_index=%zu ptr=%p\n",
                    k,
                    ctx->debug_memory[k].index,
                    ctx->debug_memory[k].data);
            }
        }

        if (ctx->debug_memory_untracked_alloc_count > 0)
        {
            fprintf(stderr,
                "Warning: PRC_DEBUG_MEMORY table saturated; %zu allocation(s) were untracked. "
                "Leak scan is incomplete.\n",
                ctx->debug_memory_untracked_alloc_count);
        }

        if (has_leak)
            release_code = PRC_API_MEMORY_LEAK_DETECTED;
    }
#endif
     return release_code;
}

PRC_EXPORT prc_api_data
prc_api_open_contents(prc_context *ctx, const char *infile)
{
    return prc_open_contents(ctx, infile);
}

static void
prc_api_helper_release_attributes(prc_context *ctx, prc_api_attributes *attribute)
{
    uint32_t k, j;

    if (attribute->num_base_attributes == 0)
    {
        return;
    }

    for (k = 0; k < attribute->num_base_attributes; k++)
    {
        prc_api_attribute_base *attr = &attribute->base_attributes[k];

        if (attr->attribute_base_title != NULL)
        {
            prc_free(ctx, attr->attribute_base_title);
            attr->attribute_base_title = NULL;
        }

        for (j = 0; j < attr->num_attributes; j++)
        {
            prc_api_attribute_entry *entry = &attr->attributes[j];
            if (entry->entry_title != NULL)
                prc_free(ctx, entry->entry_title);
            if (entry->type == PRC_API_STRING_ATTRIBUTE)
            {
                if (entry->value_string != NULL)
                    prc_free(ctx, entry->value_string);
            }
        }
        prc_free(ctx, attr->attributes);
    }
    prc_free(ctx, attribute->base_attributes);
}

/* Here we release the API visible data that was created. Not the parsed objects. */
PRC_EXPORT void
prc_api_release_data(prc_context *ctx, prc_api_data data_in, prc_api_tess *tess_in,
    uint32_t num_tess, prc_api_tess *line_tess, uint32_t num_line_tess,
    prc_api_product *product_tree)
{
    prc_data *data = (prc_data *)data_in;
    prc_tess *tess;
    uint32_t k, j;
    prc_api_child_reserve *reserve;

    if (product_tree != NULL)
    {
        if (product_tree->reserved != NULL)
        {
            reserve = (prc_api_child_reserve *)product_tree->reserved;

            if (reserve->style_tree_root != NULL)
            {
                prc_api_release_style_tree(ctx, reserve->style_tree_root);
                reserve->style_tree_root = NULL;
            }

            if (reserve->style_pool != NULL)
            {
                prc_free(ctx, reserve->style_pool);
                reserve->style_pool = NULL;
            }
            reserve->style_pool_capacity = 0;
            reserve->style_pool_index = 0;

            for (k = 0; k < reserve->num_parts; k++)
            {
                if (reserve->parts[k].name != NULL)
                {
                    prc_free(ctx, reserve->parts[k].name);
                }
                prc_api_helper_release_attributes(ctx,
                    &reserve->parts[k].attributes);
            }
            if (reserve)
                prc_free(ctx, reserve->parts);

            for (k = 0; k < reserve->num_products; k++)
            {
                if (reserve->products[k].name != NULL)
                {
                    prc_free(ctx, reserve->products[k].name);
                }
                prc_api_helper_release_attributes(ctx,
                    &reserve->products[k].attributes);
            }
            if (reserve->products != NULL)
                prc_free(ctx, reserve->products);

            for (k = 0; k < reserve->num_markups; k++)
            {
                if (reserve->markups[k].name != NULL)
                    prc_free(ctx, reserve->markups[k].name);
            }
            if (reserve->markups != NULL)
                prc_free(ctx, reserve->markups);
            prc_free(ctx, reserve);
        }
    }

    for (j = 0; j < data->file_structure_count; j++)
    {
        prc_filestructure *file_struct = &data->file_struct[j];
        uint32_t num_tess_prc = file_struct->tessellation->tess_count;

        for (k = 0; k < num_tess_prc; k++)
        {
            tess = &file_struct->tessellation->tess[k];

            if (tess->tess_type == PRC_TYPE_TESS_3D ||
                tess->tess_type == PRC_TYPE_TESS_3D_Compressed)
            {
                if (tess->vertices_internal != NULL)
                {
                    prc_free(ctx, tess->vertices_internal);
                    tess->vertices_internal = NULL;
                }
                if (tess->normals_internal != NULL)
                {
                    prc_free(ctx, tess->normals_internal);
                    tess->normals_internal = NULL;
                }
                /* tess->position_normal_lut is used only by the uncompressed
                   (PRC_TYPE_TESS_3D) branch of prc_api_get_tessellation_vertices,
                   which builds and frees its own use of this field within
                   every call -- so this is normally already NULL by the
                   time release runs; kept as a defensive catch-all in case
                   a call fails partway through. The compressed-tessellation
                   branch's equivalent cache is scoped per api_tess instance
                   (api_tess->reserved, see prc_tess_internal_position_normal_lookup
                   in prc_internal_api.h) and freed below, in the per-api_tess
                   loop -- NOT here, since this prc_tess-level struct is
                   shared by every api_tess instance that references the
                   same underlying tessellation (e.g. reused/instanced
                   hardware in an assembly), and each such instance owns an
                   independent copy of its own dedup cache. */
                if (tess->position_normal_lut.position_normal_pair != NULL)
                {
                    size_t pn;
                    for (pn = 0; pn < tess->position_normal_lut.number_values; pn++)
                    {
                        prc_internal_api_position_normal_pair *pn_current =
                            tess->position_normal_lut.position_normal_pair[pn].next;
                        while (pn_current != NULL)
                        {
                            prc_internal_api_position_normal_pair *pn_temp = pn_current;
                            pn_current = pn_current->next;
                            prc_free(ctx, pn_temp);
                        }
                    }
                    prc_free(ctx, tess->position_normal_lut.position_normal_pair);
                    tess->position_normal_lut.position_normal_pair = NULL;
                    tess->position_normal_lut.number_values = 0;
                }
            }
            if (tess->tess_type == PRC_TYPE_TESS_3D_Wire ||
                tess->tess_type == PRC_TYPE_TESS_MarkUp)
            {
                if (tess->vertices_internal != NULL)
                    prc_free(ctx, tess->vertices_internal);
            }
        }
    }

    for (k = 0; k < num_line_tess; k++)
    {
        if (line_tess[k].tess_faces != NULL)
        {
            for (j = 0; j < line_tess[k].num_faces; j++)
            {
                if (line_tess[k].tess_faces[j].face_vertices.vertices != NULL)
                    prc_free(ctx, line_tess[k].tess_faces[j].face_vertices.vertices);
                prc_internal_api_wire *wire = prc_face_internal_wire(&line_tess[k].tess_faces[j]);
                if (wire != NULL)
                {
                    uint32_t num_graphic_primitives = line_tess[k].tess_faces[j].num_graphic_primitives;
                    for (int i = 0; i < num_graphic_primitives; i++)
                    {
                        if (wire[i].vertex_indices != NULL)
                            prc_free(ctx, wire[i].vertex_indices);
                    }
                    prc_free(ctx, wire);
                }
            }
            //prc_free(ctx, line_tess[k].tess_faces);
        }
    }

    for (k = 0; k < num_tess; k++)
    {
        if (tess_in[k].type == PRC_API_TESS_3D ||
            tess_in[k].type == PRC_API_TESS_3D_Compressed)
        {
            if (tess_in[k].tess_faces != NULL)
            {
                for (j = 0; j < tess_in[k].num_faces; j++)
                {
                    if (tess_in[k].tess_faces[j].reserved != NULL)
                    {
                        prc_internal_api_face *face_out_reserved =
                            prc_face_internal_face(&tess_in[k].tess_faces[j]);
                        if (face_out_reserved->vertex_indices != NULL)
                            prc_free(ctx, face_out_reserved->vertex_indices);
                        if (face_out_reserved->single_norm.fan_offsets != NULL)
                            prc_free(ctx, face_out_reserved->single_norm.fan_offsets);
                        if (face_out_reserved->single_norm.strip_offsets != NULL)
                            prc_free(ctx, face_out_reserved->single_norm.strip_offsets);
                        if (face_out_reserved->multi_norm.fan_offsets != NULL)
                            prc_free(ctx, face_out_reserved->multi_norm.fan_offsets);
                        if (face_out_reserved->multi_norm.strip_offsets != NULL)
                            prc_free(ctx, face_out_reserved->multi_norm.strip_offsets);
                        if (face_out_reserved->texture_single_norm.fan_offsets != NULL)
                            prc_free(ctx, face_out_reserved->texture_single_norm.fan_offsets);
                        if (face_out_reserved->texture_single_norm.strip_offsets != NULL)
                            prc_free(ctx, face_out_reserved->texture_single_norm.strip_offsets);
                        if (face_out_reserved->texture_multi_norm.fan_offsets != NULL)
                            prc_free(ctx, face_out_reserved->texture_multi_norm.fan_offsets);
                        if (face_out_reserved->texture_multi_norm.strip_offsets != NULL)
                            prc_free(ctx, face_out_reserved->texture_multi_norm.strip_offsets);
                        /* owns_style == 0 means `style` is a borrowed pointer
                           into a per-api_tess-instance cache shared by every
                           face of a compressed tessellation (see the
                           face_style_cache cleanup below, near the
                           tess_in[k].reserved release) -- freeing it here,
                           once per face, would double-free after the first
                           face. */
                        if (face_out_reserved->style != NULL && face_out_reserved->owns_style)
                        {
                            /* Deal with any texture pipelines */
                            uint8_t done = 0;
                            prc_internal_texture *next = face_out_reserved->style->texture.next_texture;
                            while (!done)
                            {
                                prc_internal_texture *next2;
                                if (next != NULL)
                                {
                                    next2 = next->next_texture;
                                    prc_free(ctx, next);
                                    next = next2;
                                }
                                else
                                {
                                    done = 1;
                                }
                            }
                            prc_free(ctx, face_out_reserved->style);
                        }
                        prc_free(ctx, face_out_reserved);
                    }
                    if (tess_in[k].type == PRC_API_TESS_3D)
                    {
                        if (tess_in[k].tess_faces[j].face_vertices.vertices != NULL)
                            prc_free(ctx, tess_in[k].tess_faces[j].face_vertices.vertices);
                    }
                }
                //prc_free(ctx, tess_in[k].tess_faces);
            }
            if (tess_in[k].type == PRC_API_TESS_3D_Compressed &&
                tess_in[k].reserved != NULL)
            {
                /* Per-instance dedup cache built by the compressed branch
                   of prc_api_get_tessellation_vertices, scoped to THIS
                   api_tess (see prc_tess_internal_position_normal_lookup
                   in prc_internal_api.h) -- not shared with any other
                   instance that may reference the same underlying
                   tess_index, so it is freed here, once per instance,
                   rather than once per underlying tessellation. */
                prc_internal_api_position_normal_lookup *tess_cache =
                    prc_tess_internal_position_normal_lookup(&tess_in[k]);
                if (tess_cache->position_normal_pair != NULL)
                {
                    size_t pn;
                    for (pn = 0; pn < tess_cache->number_values; pn++)
                    {
                        prc_internal_api_position_normal_pair *pn_current =
                            tess_cache->position_normal_pair[pn].next;
                        while (pn_current != NULL)
                        {
                            prc_internal_api_position_normal_pair *pn_temp = pn_current;
                            pn_current = pn_current->next;
                            prc_free(ctx, pn_temp);
                        }
                    }
                    prc_free(ctx, tess_cache->position_normal_pair);
                }
                /* Per-face style table (face_style_cache_count entries),
                   shared by every face_out_reserved->style in this
                   instance's tess_faces (owns_style == 0 there, so none of
                   them free it themselves -- see the per-face release loop
                   above). Each entry can carry its own texture pipeline,
                   same as the single-entry uncompressed case. */
                if (tess_cache->face_style_cache != NULL)
                {
                    uint32_t si;
                    for (si = 0; si < tess_cache->face_style_cache_count; si++)
                    {
                        prc_internal_texture *next = tess_cache->face_style_cache[si].texture.next_texture;
                        while (next != NULL)
                        {
                            prc_internal_texture *next2 = next->next_texture;
                            prc_free(ctx, next);
                            next = next2;
                        }
                    }
                    prc_free(ctx, tess_cache->face_style_cache);
                }
                prc_free(ctx, tess_cache);
                tess_in[k].reserved = NULL;
            }
            if (tess_in[k].tess_vertices.vertices != NULL)
                prc_free(ctx, tess_in[k].tess_vertices.vertices);
        }

        if (tess_in[k].type == PRC_API_TESS_3D_Wire)
        {
            prc_internal_api_wire *wire = prc_tess_internal_wire(&tess_in[k]);

            if (wire != NULL)
            {
                for (j = 0; j < tess_in[k].num_line_primitives; j++)
                {
                    if (wire[j].vertex_indices != NULL)
                        prc_free(ctx, wire[j].vertex_indices);
                }
                prc_free(ctx, wire);
            }
            if (tess_in[k].tess_vertices.vertices != NULL)
                prc_free(ctx, tess_in[k].tess_vertices.vertices);
        }
        if (tess_in[k].type == PRC_API_TESS_MarkUp)
        {
            prc_internal_api_wire *markup = prc_tess_internal_wire(&tess_in[k]);
            if (markup != NULL)
            {
                for (j = 0; j < tess_in[k].num_line_primitives; j++)
                {
                    if (markup[j].vertex_indices != NULL)
                        prc_free(ctx, markup[j].vertex_indices);
                }
                prc_free(ctx, markup);
            }
            if (tess_in[k].tess_vertices.vertices != NULL)
                prc_free(ctx, tess_in[k].tess_vertices.vertices);
        }
        if (tess_in[k].reserved2 != NULL)
        {
            prc_free(ctx, tess_in[k].reserved2);
        }
        if (tess_in[k].text_primitives != NULL)
        {
            prc_free(ctx, tess_in[k].text_primitives);
        }
    }

    if (data->part_details != NULL)
    {
        prc_free(ctx, data->part_details);
        data->part_details = NULL;
    }

    if (data->markup_details != NULL)
    {
        prc_free(ctx, data->markup_details);
        data->markup_details = NULL;
    }
    prc_release_data(ctx, (prc_data *)data);
}

static void
prc_api_matrix_identity_check(prc_context *ctx, prc_api_transform *transform)
{
    if (transform->matrix[0] == 1.0 &&
        transform->matrix[1] == 0.0 &&
        transform->matrix[2] == 0.0 &&
        transform->matrix[3] == 0.0 &&
        transform->matrix[4] == 0.0 &&
        transform->matrix[5] == 1.0 &&
        transform->matrix[6] == 0.0 &&
        transform->matrix[7] == 0.0 &&
        transform->matrix[8] == 0.0 &&
        transform->matrix[9] == 0.0 &&
        transform->matrix[10] == 1.0 &&
        transform->matrix[11] == 0.0 &&
        transform->matrix[12] == 0.0 &&
        transform->matrix[13] == 0.0 &&
        transform->matrix[14] == 0.0 &&
        transform->matrix[15] == 1.0)
    {
        transform->is_identity = 1;
    }
}

/* Matrix elements are stored in column first order assuming we are multiplying
   on the right of the matrix by the column position vector.

   M00 M01 M02 M03
   M10 M11 M12 M13
   M20 M21 M22 M23
   0   0   0   1

   where the element indices are as given in the PRC spec. So data is stored
   in memory as M00, M10, M20, 0, M01, M11, ... */

static int
prc_api_set_transform(prc_context *ctx, prc_api_transform *location,
    const prc_misc_transformation *prc_trans)
{
    if (location == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "location is NULL in prc_api_set_transform\n");
        return PRC_ERROR_MEMORY;
    }

    uint8_t other_flags_set = 0;
    double *matrix = location->matrix;
    location->is_identity = 0;

    if (prc_trans->transformation_type == PRC_TYPE_MISC_GeneralTransformation)
    {
        memcpy(matrix, &prc_trans->general_transformation.general_transform,
               sizeof(double) * 16);
    }
    else if (prc_trans->transformation_type == PRC_TYPE_MISC_CartesianTransformation)
    {
        char behavior = prc_trans->cartesian.behavior;
        memset(matrix, 0, sizeof(double) * 16);

        /* Identity */
        matrix[0] = 1.0;
        matrix[5] = 1.0;
        matrix[10] = 1.0;
        matrix[15] = 1.0;

        if (behavior == 0)
        {
            prc_api_matrix_identity_check(ctx, location);
            return 0;
        }

        if (behavior & PRC_TRANSFORMATION_Translate)
        {
            matrix[12] = prc_trans->cartesian.translation.x;
            matrix[13] = prc_trans->cartesian.translation.y;
            matrix[14] = prc_trans->cartesian.translation.z;
        }

        if (behavior & PRC_TRANSFORMATION_NonOrtho)
        {
            /* TODO Check this. */
            matrix[0] = prc_trans->cartesian.non_ortho_matrix[0].x;
            matrix[1] = prc_trans->cartesian.non_ortho_matrix[0].y;
            matrix[2] = prc_trans->cartesian.non_ortho_matrix[0].z;

            matrix[4] = prc_trans->cartesian.non_ortho_matrix[1].x;
            matrix[5] = prc_trans->cartesian.non_ortho_matrix[1].y;
            matrix[6] = prc_trans->cartesian.non_ortho_matrix[1].z;

            matrix[8] = prc_trans->cartesian.non_ortho_matrix[2].x;
            matrix[9] = prc_trans->cartesian.non_ortho_matrix[2].y;
            matrix[10] = prc_trans->cartesian.non_ortho_matrix[2].z;

            other_flags_set = 1;
        }

        if (behavior & PRC_TRANSFORMATION_Rotate)
        {
            /* TODO. Check this. Table 87 -- 3D Transformation */
            prc_vec3 z_axis;

            if (behavior & PRC_TRANSFORMATION_Mirror)
            {
                /* Z is cross product of Y and X */
                prc_vec_cross(prc_trans->cartesian.rotation[1],
                    prc_trans->cartesian.rotation[0], &z_axis);
            }
            else
            {
                prc_vec_cross(prc_trans->cartesian.rotation[0],
                    prc_trans->cartesian.rotation[1], &z_axis);
            }
            /* Now load the matrix */
            matrix[0] = prc_trans->cartesian.rotation[0].x;
            matrix[1] = prc_trans->cartesian.rotation[0].y;
            matrix[2] = prc_trans->cartesian.rotation[0].z;
            matrix[4] = prc_trans->cartesian.rotation[1].x;
            matrix[5] = prc_trans->cartesian.rotation[1].y;
            matrix[6] = prc_trans->cartesian.rotation[1].z;
            matrix[8] = z_axis.x;
            matrix[9] = z_axis.y;
            matrix[10] = z_axis.z;

            other_flags_set = 1;
        }

        if (behavior & PRC_TRANSFORMATION_NonUniformScale)
        {
            if (other_flags_set)
            {
                matrix[0] = matrix[0] * prc_trans->cartesian.non_uniform_scale.x;
                matrix[5] = matrix[5] * prc_trans->cartesian.non_uniform_scale.y;
                matrix[10] = matrix[10] * prc_trans->cartesian.non_uniform_scale.z;
            }
            else
            {
                matrix[0] = prc_trans->cartesian.non_uniform_scale.x;
                matrix[5] = prc_trans->cartesian.non_uniform_scale.y;
                matrix[10] = prc_trans->cartesian.non_uniform_scale.z;
                other_flags_set = 1;
            }
        }

        if (behavior & PRC_TRANSFORMATION_Homogeneous)
        {
            /* This is an odd one */
            matrix[3] = prc_trans->cartesian.homogeneous[0];
            matrix[7] = prc_trans->cartesian.homogeneous[1];
            matrix[11] = prc_trans->cartesian.homogeneous[2];
            matrix[15] = prc_trans->cartesian.homogeneous[3];
        }

        if (behavior & PRC_TRANSFORMATION_Scale)
        {
            for (int i = 0; i < 11; i++)
            {
                matrix[i] = matrix[i] * prc_trans->cartesian.scale;
            }
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_api_set_transform\n");
        return PRC_ERROR_PARSE;
    }

    prc_api_matrix_identity_check(ctx, location);

    return 0;
}

static int
prc_api_set_transform_to_identity(prc_context *ctx, prc_api_transform *transform)
{
    if (transform == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "transform is NULL in prc_api_set_transform_to_identity\n");
        return PRC_ERROR_MEMORY;
    }

    double *matrix = transform->matrix;
    transform->is_identity = 1;

    matrix[0] = 1.0;
    matrix[1] = 0.0;
    matrix[2] = 0.0;
    matrix[3] = 0.0;
    matrix[4] = 0.0;
    matrix[5] = 1.0;
    matrix[6] = 0.0;
    matrix[7] = 0.0;
    matrix[8] = 0.0;
    matrix[9] = 0.0;
    matrix[10] = 1.0;
    matrix[11] = 0.0;
    matrix[12] = 0.0;
    matrix[13] = 0.0;
    matrix[14] = 0.0;
    matrix[15] = 1.0;

    return 0;
}

static int
prc_api_allocate_reserve(prc_context *ctx, uint32_t num_parts, uint32_t num_products,
    uint32_t num_markups, prc_api_child_reserve *reserve)
{
    if (num_parts > 0)
    {
        reserve->parts = (prc_api_part *)prc_calloc(ctx, sizeof(prc_api_part), num_parts);
        if (reserve->parts == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_allocate_reserve\n");
            return PRC_ERROR_MEMORY;
        }
    }
    else
    {
        reserve->parts = NULL;
    }

    if (num_products > 0)
    {
        reserve->products = (prc_api_product *)prc_calloc(ctx, sizeof(prc_api_product), num_products);
        if (reserve->products == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_allocate_reserve\n");
            return PRC_ERROR_MEMORY;
        }
    }
    else
    {
        reserve->products = NULL;
    }

    if (num_markups > 0)
    {
        reserve->markups = (prc_api_markup *)prc_calloc(ctx, sizeof(prc_api_markup), num_markups);
        if (reserve->markups == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_allocate_reserve\n");
            return PRC_ERROR_MEMORY;
        }
    }
    else
    {
        reserve->markups = NULL;
    }

    reserve->num_parts = num_parts;
    reserve->num_products = num_products;
    reserve->num_markups = num_markups;
    reserve->part_index = 0;
    reserve->product_index = 0;
    reserve->markup_index = 0;

    reserve->style_tree_root = NULL;

    return 0;
}

static int
prc_api_set_rep_item_name(prc_context *ctx, prc_api_part *rep_item,
    unsigned char *rep_item_name)
{
    unsigned char name_rep_item[] = "node";

    if (rep_item_name != NULL)
    {
        rep_item->name = (char*)prc_calloc(ctx,
            strlen((const char*) rep_item_name) + 1, sizeof(unsigned char));
        if (rep_item->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_rep_item_name\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(rep_item->name, rep_item_name, strlen((const char *)rep_item_name));
    }
    else
    {   /* Set name to name_rep_item */
        rep_item->name = (char*)prc_calloc(ctx,
            strlen((const char *)name_rep_item) + 1, sizeof(unsigned char));
        if (rep_item->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_rep_item_name\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(rep_item->name, name_rep_item, strlen((const char *)name_rep_item));
    }
    return 0;
}

static int
prc_api_set_part_name(prc_context *ctx, prc_api_part *part,
                      const unsigned char *name_in)
{
    unsigned char name_part[] = "part";

    if (name_in != NULL)
    {
        part->name = (char*)prc_calloc(ctx,
            strlen((const char*) name_in) + 1, sizeof(unsigned char));
        if (part->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_part_name\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(part->name, name_in, strlen((const char*) name_in));
    }
    else
    {   /* Set name to name_part */
        part->name = (char*)prc_calloc(ctx, strlen((const char *)name_part) + 1, sizeof(unsigned char));
        if (part->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_part_name\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(part->name, name_part, strlen((const char *)name_part));
    }
    return 0;
}

static uint8_t
prc_api_helper_ri_items_all_brep(prc_context *ctx, uint32_t num_refs,
    prc_ri *rep_items)
{
    uint32_t k;

    for (k = 0; k < num_refs; k++)
    {
        if (!(rep_items[k].representation_type == PRC_TYPE_RI_BrepModel
            || rep_items[k].representation_type == PRC_TYPE_RI_PolyBrepModel))
            return 0;
    }
    return 1;
}

typedef struct
{
    uint32_t num_rep_items;
    prc_ri_set *ri_set;
    prc_api_part *api_rep;
} prc_api_add_ri_to_ri_work;

static int
prc_api_add_ri_to_ri_push(prc_context *ctx, prc_api_add_ri_to_ri_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint32_t num_rep_items,
    prc_ri_set *ri_set, prc_api_part *api_rep)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_add_ri_to_ri_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_add_ri_to_ri_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_add_ri_to_ri_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].num_rep_items = num_rep_items;
    (*worklist)[*worklist_size].ri_set = ri_set;
    (*worklist)[*worklist_size].api_rep = api_rep;
    (*worklist_size)++;
    return 0;
}

/* Adds RIs to RIs. Only valid if parent is PRC_TYPE_RI_Set. Iterative (not
   recursive): RI_Set nesting follows the file's structure, which is
   attacker-controlled and could otherwise drive unbounded C-stack recursion.
   Uses an explicit heap-allocated work list instead. */
static int32_t
prc_api_helper_add_ri_to_ri(prc_context *ctx,
    prc_api_tess *tessellations, prc_api_child_reserve *reserve,
    uint32_t num_rep_items, prc_ri_set *ri_set, prc_api_part *api_rep)
{
    prc_api_add_ri_to_ri_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    int32_t result = 0;

    worklist = (prc_api_add_ri_to_ri_work *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_add_ri_to_ri_work));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri_to_ri\n");
        return PRC_ERROR_MEMORY;
    }
    worklist[worklist_size].num_rep_items = num_rep_items;
    worklist[worklist_size].ri_set = ri_set;
    worklist[worklist_size].api_rep = api_rep;
    worklist_size++;

    while (worklist_size > 0)
    {
        prc_api_add_ri_to_ri_work item = worklist[--worklist_size];
        uint32_t k;

        for (k = 0; k < item.num_rep_items; k++)
        {
            if (item.ri_set->rep_items[k].representation_type == PRC_TYPE_RI_BrepModel ||
                item.ri_set->rep_items[k].representation_type == PRC_TYPE_RI_PolyBrepModel)
            {
                uint32_t tess_index_ri = item.ri_set->rep_items[k].item_content.biased_index_tessellation;
                if (tess_index_ri > 0)
                {
                    int32_t code;

                    item.api_rep[k].tess = &tessellations[tess_index_ri - 1];
                    code = prc_api_set_rep_item_name(ctx, &item.api_rep[k],
                        item.ri_set->rep_items[k].item_content.base.base.name.name.string);
                    if (code < 0)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri_items\n");
                        result = PRC_ERROR_MEMORY;
                        goto cleanup;
                    }
                    /* This one has no children */
                    item.api_rep[k].num_rep_items = 0;
                    item.api_rep[k].rep_items = 0;
                }
            }
            else if (item.ri_set->rep_items[k].representation_type == PRC_TYPE_RI_Set)
            {
                /* In this case we need to add a rep type to a rep type */
                item.api_rep[k].num_rep_items = item.ri_set->rep_items[k].ri_set->number_of_items;
                item.api_rep[k].rep_items = &reserve->parts[reserve->part_index];
                reserve->part_index += item.api_rep[k].num_rep_items;

                if (prc_api_add_ri_to_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        item.api_rep[k].num_rep_items, item.ri_set->rep_items[k].ri_set,
                        &item.api_rep[k]) < 0)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri_items\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
            }
        }
    }

cleanup:
    prc_free(ctx, worklist);
    return result;
}

typedef struct
{
    uint32_t num_rep_items;
    prc_ri *rep_items;
    prc_api_part *parent;
} prc_api_add_ri_items_work;

static int
prc_api_add_ri_items_push(prc_context *ctx, prc_api_add_ri_items_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint32_t num_rep_items,
    prc_ri *rep_items, prc_api_part *parent)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_add_ri_items_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_add_ri_items_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_add_ri_items_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].num_rep_items = num_rep_items;
    (*worklist)[*worklist_size].rep_items = rep_items;
    (*worklist)[*worklist_size].parent = parent;
    (*worklist_size)++;
    return 0;
}

/* Adds the representative items. Iterative (not recursive): RI_Set nesting
   follows the file's structure, which is attacker-controlled and could
   otherwise drive unbounded C-stack recursion. Uses an explicit
   heap-allocated work list instead. */
static int32_t
prc_api_helper_add_ri_items(prc_context *ctx,prc_api_child_reserve *reserve,
    uint32_t num_rep_items, prc_ri *rep_items, prc_api_part *parent)
{
    prc_api_add_ri_items_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    int32_t result = 0;

    worklist = (prc_api_add_ri_items_work *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_add_ri_items_work));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri_items\n");
        return PRC_ERROR_MEMORY;
    }
    worklist[worklist_size].num_rep_items = num_rep_items;
    worklist[worklist_size].rep_items = rep_items;
    worklist[worklist_size].parent = parent;
    worklist_size++;

    while (worklist_size > 0)
    {
        prc_api_add_ri_items_work item = worklist[--worklist_size];
        uint32_t k;

        item.parent->rep_items = &reserve->parts[reserve->part_index];
        item.parent->num_rep_items = item.num_rep_items;
        reserve->part_index += item.num_rep_items;

        for (k = 0; k < item.num_rep_items; k++)
        {
            if (item.rep_items[k].representation_type == PRC_TYPE_RI_BrepModel ||
                item.rep_items[k].representation_type == PRC_TYPE_RI_PolyBrepModel ||
                item.rep_items[k].representation_type == PRC_TYPE_RI_PointSet)
            {
                uint32_t tess_index_ri = item.rep_items[k].item_content.biased_index_tessellation;
                if (tess_index_ri > 0)
                {
                    int32_t code;

                    /* Add a NULL check for 'part->rep_items' before dereferencing it */
                    if (item.parent->rep_items == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "rep_items is NULL in prc_api_initialize_node\n");
                        result = PRC_ERROR_MEMORY;
                        goto cleanup;
                    }
                    code = prc_api_set_rep_item_name(ctx, &item.parent->rep_items[k],
                        item.rep_items[k].item_content.base.base.name.name.string);
                    if (code < 0)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_rep_item_name\n");
                        result = PRC_ERROR_MEMORY;
                        goto cleanup;
                    }
                    /* This one has no children (as it has no sets) */
                    item.parent->rep_items[k].num_rep_items = 0;
                    item.parent->rep_items[k].rep_items = NULL;
                    item.parent->rep_items[k].biased_tess_index = tess_index_ri;
                    item.parent->rep_items[k].biased_style_index =
                        item.rep_items[k].item_content.base.graphics_content.biased_index_of_line_style;
                }
            }
            else if (item.rep_items[k].representation_type == PRC_TYPE_RI_Set)
            {
                /* In this case we need to add a rep type to a rep type */
                prc_ri_set *ri_set = (item.rep_items[k]).ri_set;
                uint32_t set_num_rep_items = ri_set->number_of_items;
                prc_ri *set_ri_items = ri_set->rep_items;
                int32_t code;

                /* First we need to add the name of the RI set */
                code = prc_api_set_rep_item_name(ctx, &item.parent->rep_items[k],
                    item.rep_items[k].item_content.base.base.name.name.string);
                if (code < 0)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_set_rep_item_name\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
                if (prc_api_add_ri_items_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        set_num_rep_items, set_ri_items, &item.parent->rep_items[k]) < 0)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri_items\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
            }
        }
    }

cleanup:
    prc_free(ctx, worklist);
    return result;
 }

/* A recursive method to set the nodes in the product/part tree */
static int
prc_api_initialize_node(prc_context *ctx, prc_data *data, prc_api_product *product,
    const unsigned char *name_in, uint32_t biased_part_index_in,
    uint32_t number_children, uint32_t *children_indices, prc_api_child_reserve *reserve,
    uint32_t *num_models, uint32_t *num_nodes, uint8_t needs_3d_pmi_child,
    prc_asm_product_occurrence *prc_product_parent, uint32_t file_index)
{
    int code;
    uint32_t len_name = 0;
    uint32_t k, j;
    prc_asm_file_structure_tree *tree_in = data->file_struct[file_index].tree;
    prc_type_asm_modelfile *model_in = data->file_struct[file_index].model;
    prc_asm_parts_definition *prc_part;
    uint32_t num_rep_items;
    uint32_t tess_index_ri;
    prc_asm_product_occurrence *prc_product;
    uint32_t num_refs;
    prc_unsigned_int *indices;
    prc_references_of_product_occurrence product_refs;
    uint32_t num_markups = 0;
    uint32_t biased_part_index;
    unsigned char *name;
    unsigned char name_3d_pmi[] = "3D PMI";
    uint8_t all_brep;
    unsigned char *model_name = NULL;

    if (*num_nodes == 0)
    {
        model_name = model_in->base.name.name.string;
    }

    *num_nodes += 1;

    if (needs_3d_pmi_child)
        *num_nodes += 1;

    if (name_in != NULL)
    {
        len_name = strlen((const char *)name_in);
        product->name = (char*)prc_calloc(ctx, len_name + 1, sizeof(char));
        if (product->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(product->name, name_in, len_name);
    }
    else
    {
        product->name = NULL;
    }

    if (biased_part_index_in == 0)
    {
        product->part = NULL;
    }
    else
    {
        product->part = &reserve->parts[reserve->part_index];
        reserve->part_index++;
        prc_part = &tree_in->parts[biased_part_index_in - 1];
        num_rep_items = prc_part->num_rep_items;
        product->part->tess = NULL;

        /* Set the name of the part. A product can have at most one part.
           Note that it could be the same as the
           product name and the product may only have this single part. We
           will let the application decide how it wants to deal with that */
        code = prc_api_set_rep_item_name(ctx, product->part,
                                        prc_part->base.base.name.name.string);

        if (num_rep_items > 0)
        {
            /* Check if all the rep items are PRC_TYPE_RI_BrepModel type */
            all_brep = prc_api_helper_ri_items_all_brep(ctx, num_rep_items,
                prc_part->rep_items);

            /* If there is just one rep item AND it is of type
               PRC_TYPE_RI_BrepModel then we just add the part
               to the tree. If there are multiple rep items, then we have to
               add these as children of the part -- if they have an associated
               tessellation. */
            if (num_rep_items == 1 && all_brep)
            {
                tess_index_ri = prc_part->rep_items[0].item_content.biased_index_tessellation;

                if (product->part->name != NULL)
                {
                    prc_free(ctx, product->part->name);
                    product->part->name = NULL;
                }
                code = prc_api_set_part_name(ctx, product->part,
                    prc_part->rep_items[0].item_content.base.base.name.name.string);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_set_part_name\n");
                    return code;
                }

                /* Also set the tessellation index and the style index */
                product->part->biased_tess_index = tess_index_ri;
                product->part->biased_style_index =
                    prc_part->rep_items[0].item_content.base.graphics_content.biased_index_of_line_style;
            }
            else if (all_brep)
            {   /* If all the rep items are of type PRC_TYPE_RI_BrepModel then */
                /* We have to add rep item children to the part */
                /* First figure out how many here have a tessellation associated
                   with them */
                for (j = 0; j < num_rep_items; j++)
                {
                    tess_index_ri = prc_part->rep_items[j].item_content.biased_index_tessellation;
                    if (tess_index_ri > 0)
                    {
                        product->part->num_rep_items++;
                    }
                }
                /* Get the already allocated items from the reserve */
                product->part->rep_items = &reserve->parts[reserve->part_index];
                reserve->part_index += product->part->num_rep_items;

                /* Now go through and set the names */
                int rep_index = 0;
                for (j = 0; j < num_rep_items; j++)
                {
                    tess_index_ri = prc_part->rep_items[j].item_content.biased_index_tessellation;
                    if (tess_index_ri > 0)
                    {
                        code = prc_api_set_rep_item_name(ctx, &product->part->rep_items[rep_index],
                            prc_part->rep_items[j].item_content.base.base.name.name.string);
                        if (code < 0)
                        {
                            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
                            return PRC_ERROR_MEMORY;
                        }
                        /* Also set the tessellation index and the style index */
                        product->part->rep_items[rep_index].biased_tess_index = tess_index_ri;
                        product->part->rep_items[rep_index].biased_style_index =
                            prc_part->rep_items[j].item_content.base.graphics_content.biased_index_of_line_style;
                        rep_index++;
                    }
                }
                *num_models += product->part->num_rep_items;
            }
            else
            {
                /* There must be non-brep data in here. In this case we have to
                   approach this with a recursive function, since an RI set
                   has multiple items, and one of those could be set... */
                code = prc_api_helper_add_ri_items(ctx, reserve,
                    num_rep_items, prc_part->rep_items, product->part);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_helper_add_ri_items\n");
                    return code;
                }
            }
        }
    }

    /* Note that a product can have a part AND have children products */
    if (product->part != NULL && product->part->biased_tess_index > 0)
    {
        if (number_children != 0)
        {
            product->type = PRC_API_NODE_PRODUCT_WITH_PART;
        }
        else
        {
            product->type = PRC_API_NODE_PART;
        }
        *num_models += 1;
    }
    else
    {
        product->type = PRC_API_NODE_PRODUCT;
    }

    product->num_children = number_children + needs_3d_pmi_child;
    if (number_children > 0)
    {
        product->children = &reserve->products[reserve->product_index];
        reserve->product_index += (number_children + needs_3d_pmi_child);
    }
    else
    {
        product->children = NULL;
    }

    if (model_name != NULL)
    {
        code = prc_api_set_transform_to_identity(ctx, &product->children[0].location);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_api_initialize_node\n");
            return code;
        }

        product->children[0].is_model = 1;
        code = prc_api_initialize_node(ctx, data, product->children,
            model_name, 0, 1, children_indices, reserve, num_models, num_nodes,
            0, NULL, file_index);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_api_initialize_node\n");
            return code;
        }
    }
    else
    {
        for (k = 0; k < number_children; k++)
        {
            prc_product = &tree_in->products[children_indices[k]];
            num_markups = prc_product->markups.number_of_markups;
            product_refs = prc_product->references_product_occurrence;
            num_refs = product_refs.number_of_child_product_occurrences;
            indices = product_refs.index_child_occurrence;
            biased_part_index = product_refs.biased_index_part;
            name = prc_product->base.base.name.name.string;

            if (prc_product->has_transform)
            {
                code = prc_api_set_transform(ctx, &product->children[k].location,
                    &prc_product->location);
            }
            else
            {
                code = prc_api_set_transform_to_identity(ctx, &product->children[k].location);
            }
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_api_initialize_node\n");
                return code;
            }
            product->children[k].is_model = 0;
            code = prc_api_initialize_node(ctx, data, &product->children[k], name,
                biased_part_index, num_refs, indices, reserve, num_models, num_nodes,
                num_markups > 0, prc_product, file_index);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_api_initialize_node\n");
                return code;
            }
        }

        /* Take care of the markup node at this point */
        if (needs_3d_pmi_child)
        {
            prc_api_product *pmi_3d_node = &product->children[product->num_children - 1];
            uint32_t num_markups;

            pmi_3d_node->type = PRC_API_NODE_MARKUP;
            pmi_3d_node->num_children = 0;

            /* Set the transformation to identity */
            code = prc_api_set_transform_to_identity(ctx, &pmi_3d_node->location);

            /* Set name to name_3d_pmi */
            pmi_3d_node->name = (char *)prc_calloc(ctx, strlen((const char *)name_3d_pmi) + 1, sizeof(char));
            if (pmi_3d_node->name == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
                return PRC_ERROR_MEMORY;
            }
            memcpy(pmi_3d_node->name, name_3d_pmi, strlen((const char *)name_3d_pmi));

            /* Add the markup children to this node */
            pmi_3d_node->num_markups = prc_product_parent->markups.number_of_markups;
            pmi_3d_node->markup = &reserve->markups[reserve->markup_index];
            reserve->markup_index += prc_product_parent->markups.number_of_markups;

            for (k = 0; k < prc_product_parent->markups.number_of_markups; k++)
            {
                prc_api_markup *markup = &pmi_3d_node->markup[k];

                markup->name = (char *)prc_calloc(ctx,
                    strlen((const char*) prc_product_parent->markups.markups[k].base.base.name.name.string) + 1,
                    sizeof(char));
                if (markup->name == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
                    return PRC_ERROR_MEMORY;
                }
                memcpy(markup->name, prc_product_parent->markups.markups[k].base.base.name.name.string,
                    strlen((const char *)prc_product_parent->markups.markups[k].base.base.name.name.string));

                markup->biased_style_index =
                    prc_product_parent->markups.markups[k].base.graphics_content.biased_index_of_line_style;
                markup->biased_tess_index =
                    prc_product_parent->markups.markups[k].biased_index_tessellation;
               // markup->tess = &tessellations[prc_product_parent->markups.markups[k].biased_index_tessellation - 1];
            }
        }
    }

    return 0;
}

PRC_EXPORT uint32_t
prc_api_model_item_number_of_markups(prc_context *ctx, prc_api_product *api_product)
{
    return api_product->num_markups;
}

PRC_EXPORT uint8_t
prc_api_model_item_is_part(prc_context* ctx, prc_api_product* product)
{
    if (product->part == NULL)
        return 0;

    if (product->part->tess != NULL)
    {
        if (product->part->tess->type == PRC_API_TESS_3D ||
            product->part->tess->type == PRC_API_TESS_3D_Compressed ||
            product->part->tess->type == PRC_API_TESS_3D_Wire)
        {
            return 1;
        }
    }
    return 0;
}

PRC_EXPORT uint32_t
prc_api_get_number_of_view(prc_context *ctx, prc_api_data data)
{
    prc_data *data_in = (prc_data *)data;

    if (data == NULL)
    {
        return 0;
    }
    return data_in->view_count;
}

PRC_EXPORT int
prc_api_get_view(prc_context *ctx, prc_api_data data, uint32_t view_index, char **name, double **matrix, double *center_orbit_z)
{
    prc_data *data_in = (prc_data *)data;

    if (view_index >= data_in->view_count)
    {
        prc_error(ctx, PRC_API_INDEX, "view_index is out of range in prc_api_get_view\n");
        return PRC_API_INDEX;
    }
    *name = data_in->views[view_index].external_name;
    *matrix = data_in->views[view_index].matrix;
    *center_orbit_z = data_in->views[view_index].center_orbit_z;
    return 0;
}

PRC_EXPORT void
prc_api_set_transform_identity(prc_context* ctx, prc_api_transform* transform)
{
    prc_api_set_transform_to_identity(ctx, transform);
}

PRC_EXPORT void
prc_api_update_transform(prc_context *ctx, prc_api_transform *concate_transform, prc_api_transform *new_transform)
{
    uint32_t j, k;
    double *left_matrix;
    double right_matrix[16];

    if (new_transform->is_identity)
        return;

    if (concate_transform->is_identity && !new_transform->is_identity)
    {
        memcpy(concate_transform->matrix, new_transform->matrix, sizeof(double) * 16);
        concate_transform->is_identity = false;
    }
    return;
}

PRC_EXPORT prc_api_tess*
prc_api_get_model_tessellation(prc_context *ctx, prc_api_product* product)
{
    return product->part->tess;
}

PRC_EXPORT prc_api_tess*
prc_api_get_ri_tessellation(prc_context *ctx, prc_api_part *part,
                            uint32_t rep_item_index)
{
    return part->rep_items[rep_item_index].tess;
}

PRC_EXPORT prc_api_tess *
prc_api_get_ri_line_tessellation(prc_context *ctx, prc_api_part *part,
    uint32_t rep_item_index)
{
    return part->rep_items[rep_item_index].tess_line;
}

PRC_EXPORT prc_api_tess*
prc_api_get_markup_tessellation(prc_context *ctx, prc_api_product *product, uint32_t markup_index)
{
    return product->markup[markup_index].tess;
}

static void
prc_api_initialize_model_tree_parent(prc_context *ctx, prc_api_product *parent,
    uint32_t file_index)
{
    parent->is_model = 0;
    parent->reserved = NULL;
    parent->name = NULL;
    parent->type = PRC_API_NODE_PRODUCT;
    parent->num_children = 0;
    parent->children = NULL;
    parent->part = NULL;
    parent->num_markups = 0;
    parent->markup = NULL;
    parent->location.is_identity = 1;
    parent->file_index = file_index;
}

static void
prc_api_copy_node(prc_context *ctx, prc_api_product *des, prc_api_product *src)
{
    des->name = src->name;
    des->type = src->type;
    des->num_children = src->num_children;
    des->children = src->children;
    des->part = src->part;
    des->num_markups = src->num_markups;
    des->markup = src->markup;
    des->location.is_identity = src->location.is_identity;
    memcpy(des->location.matrix, src->location.matrix, sizeof(double) * 16);
    des->file_index = src->file_index;
    des->is_model = src->is_model;
}

/* Work-list entry for the iterative has-tessellations walk below: either a
   product-tree node or an RI (part) node, tagged so both can share one
   heap-allocated stack. */
typedef struct
{
    uint8_t is_product; /* 1 = node is prc_api_product*, 0 = node is prc_api_part* */
    void *node;
} prc_api_tess_check_item;

static int
prc_api_tess_check_push(prc_context *ctx, prc_api_tess_check_item **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint8_t is_product, void *node)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_tess_check_item *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_tess_check_item *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_tess_check_item));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].is_product = is_product;
    (*worklist)[*worklist_size].node = node;
    (*worklist_size)++;
    return 0;
}

/* Searches the product/RI tree for any tessellation. Iterative (not
   recursive): both the assembly tree and RI (representation item) nesting
   follow the file's structure, which is attacker-controlled and could
   otherwise drive unbounded C-stack recursion. Uses an explicit
   heap-allocated work list instead; short-circuits as soon as one is found,
   same as the original recursive search. */
static uint8_t
prc_api_tree_has_tessellations(prc_context *ctx, prc_api_product *tree)
{
    prc_api_tess_check_item *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    uint8_t found = 0;

    worklist = (prc_api_tess_check_item *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_tess_check_item));
    if (worklist == NULL)
        return 0;
    worklist[worklist_size].is_product = 1;
    worklist[worklist_size].node = tree;
    worklist_size++;

    while (worklist_size > 0 && !found)
    {
        prc_api_tess_check_item item = worklist[--worklist_size];
        uint32_t k;

        if (item.is_product)
        {
            prc_api_product *node = (prc_api_product *)item.node;

            if (node->part != NULL && node->part->tess != NULL)
            {
                found = 1;
                break;
            }

            if (node->markup != NULL)
            {
                for (k = 0; k < node->num_markups; k++)
                {
                    if (node->markup[k].tess != NULL)
                    {
                        found = 1;
                        break;
                    }
                }
                if (found)
                    break;
            }

            if (node->part != NULL && node->part->rep_items != NULL)
            {
                for (k = 0; k < node->part->num_rep_items; k++)
                {
                    if (prc_api_tess_check_push(ctx, &worklist, &worklist_size,
                            &worklist_capacity, 0, &node->part->rep_items[k]) < 0)
                    {
                        prc_free(ctx, worklist);
                        return 0;
                    }
                }
            }

            if (node->num_children > 0)
            {
                for (k = 0; k < node->num_children; k++)
                {
                    if (prc_api_tess_check_push(ctx, &worklist, &worklist_size,
                            &worklist_capacity, 1, &node->children[k]) < 0)
                    {
                        prc_free(ctx, worklist);
                        return 0;
                    }
                }
            }
        }
        else
        {
            prc_api_part *ri = (prc_api_part *)item.node;

            if (ri->tess != NULL)
            {
                found = 1;
                break;
            }

            if (ri->rep_items != NULL)
            {
                for (k = 0; k < ri->num_rep_items; k++)
                {
                    if (prc_api_tess_check_push(ctx, &worklist, &worklist_size,
                            &worklist_capacity, 0, &ri->rep_items[k]) < 0)
                    {
                        prc_free(ctx, worklist);
                        return 0;
                    }
                }
            }
        }
    }

    prc_free(ctx, worklist);
    return found;
}

static void
prc_api_helper_init_unique_id(prc_context *ctx, prc_unique_id *unique_id)
{
    unique_id->unique_id0 = 0;
    unique_id->unique_id1 = 0;
    unique_id->unique_id2 = 0;
    unique_id->unique_id3 = 0;
}

uint32_t
prc_api_helper_get_biased_file_index_from_unique_id(prc_context *ctx, prc_api_data data,
                                                    prc_unique_id unique_id)
{
    prc_data *data_in = (prc_data *)data;
    uint32_t k;

    for (k = 0; k < data_in->file_structure_count; k++)
    {
        if (data_in->file_struct[k].header->unique_id_file.unique_id0 == unique_id.unique_id0 &&
            data_in->file_struct[k].header->unique_id_file.unique_id1 == unique_id.unique_id1 &&
            data_in->file_struct[k].header->unique_id_file.unique_id2 == unique_id.unique_id2 &&
            data_in->file_struct[k].header->unique_id_file.unique_id3 == unique_id.unique_id3)
        {
            return k + 1;
        }
    }
    return 0;
}

/*
 * Style pool helpers - allocate a contiguous pool of prc_api_object_style
 * nodes, hand out nodes by index, and free the pool.
 *
 * Design:
 *  - Capacity = num_products + num_parts + num_markups (clamped by a safe cap).
 *  - prc_internal_api_style_pool_alloc returns NULL on OOM or if the pool
 *    is exhausted.
 *  - Pool memory is owned by the reserve and freed with prc_internal_api_style_pool_free.
 *
 * Uses existing allocator wrappers: prc_calloc / prc_realloc / prc_free.
 */

static int
prc_internal_api_style_pool_init(prc_context *ctx, prc_api_child_reserve *reserve,
    uint32_t num_products, uint32_t num_parts, uint32_t num_markups)
{
    uint64_t requested = (uint64_t)num_products + (uint64_t)num_parts + (uint64_t)num_markups;
    uint32_t capacity;

    if (reserve == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "reserve is NULL in prc_internal_api_style_pool_init\n");
        return PRC_ERROR_INTERNAL;
    }

    if (requested == 0)
    {
        /* ensure a small pool so callers may still allocate a node */
        capacity = 1;
    }
    else if (requested > PRC_STYLE_POOL_MAX_CAPACITY)
    {
        capacity = PRC_STYLE_POOL_MAX_CAPACITY;
    }
    else if (requested > UINT32_MAX)
    {
        capacity = PRC_STYLE_POOL_MAX_CAPACITY;
    }
    else
    {
        capacity = (uint32_t)requested;
    }

    reserve->style_pool = (prc_api_object_style *)prc_calloc(ctx, sizeof(prc_api_object_style), capacity);
    if (reserve->style_pool == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_internal_api_style_pool_init\n");
        reserve->style_pool_capacity = 0;
        reserve->style_pool_index = 0;
        return PRC_ERROR_MEMORY;
    }

    reserve->style_pool_capacity = capacity;
    reserve->style_pool_index = 0;

    return 0;
}

static prc_api_object_style *
prc_internal_api_style_pool_alloc(prc_context *ctx, prc_api_child_reserve *reserve)
{
    prc_api_object_style *node;

    if (reserve == NULL || reserve->style_pool == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "reserve or style_pool is NULL in prc_internal_api_style_pool_alloc\n");
        return NULL;
    }

    if (reserve->style_pool_index >= reserve->style_pool_capacity)
    {
        /* pool exhausted */
        prc_error(ctx, PRC_ERROR_INTERNAL, "Style pool exhausted (capacity=%u)\n", reserve->style_pool_capacity);
        return NULL;
    }

    node = &reserve->style_pool[reserve->style_pool_index++];
    /* Clear/initialize fields to safe defaults in case node was reused */
    node->file_index = UINT32_MAX;
    node->base_with_graphics = NULL;
    node->num_entity_references = 0;
    node->entity_references = NULL;
    node->parent = NULL;
    node->children = NULL;
    node->num_children = 0;

    return node;
}

static void
prc_api_helper_set_inheritance_detail(prc_context *ctx,
    prc_internal_detail_inheritance *inheritance, uint32_t biased_index, uint32_t file_index)
{
    inheritance->has_inheritance = 1;
    inheritance->inherited_file_index = file_index;
    inheritance->inherited_value = biased_index;
}

/* Deal with the inheritence, product to product, product to part, part to RI,
   RI to RI */
static void
prc_api_helper_set_inheritance_data(prc_context *ctx,
    prc_internal_inheritance *parent_inherit, prc_graphics_content *parent_graphics,
    prc_internal_inheritance *child_inherit, uint32_t file_index)
{
    /* We deal with each of these separately since they could trickle through
       on their own with transparency not being inherited but color being
       inherited for example. We also have to deal with inheriting the current
       value vs something that was previously inherited */
    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritColor)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->color,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->color.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->color,
            parent_inherit->color.inherited_value,
            parent_inherit->color.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }

    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritTransparency)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->transparency,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->transparency.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->transparency,
            parent_inherit->transparency.inherited_value,
            parent_inherit->transparency.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }

    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritLinePattern)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->line_pattern,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->line_pattern.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->line_pattern,
            parent_inherit->line_pattern.inherited_value,
            parent_inherit->line_pattern.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }

    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritLineWidth)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->line_width,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->line_width.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->line_width,
            parent_inherit->line_width.inherited_value,
            parent_inherit->line_width.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }

    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritShow)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->show,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->show.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->show,
            parent_inherit->show.inherited_value,
            parent_inherit->show.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }

    if (parent_graphics->behavior_bit_field1 & PRC_GRAPHICS_ChildHeritLayer)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->layer,
            parent_graphics->biased_index_of_line_style, file_index);
        child_inherit->has_any_inheritance = 1;
    }
    else if (parent_inherit->has_any_inheritance && parent_inherit->layer.has_inheritance)
    {
        prc_api_helper_set_inheritance_detail(ctx, &child_inherit->layer,
            parent_inherit->layer.inherited_value,
            parent_inherit->layer.inherited_file_index);
        child_inherit->has_any_inheritance = 1;
    }
}

/* Counts the item types in the representation items. Iterative (not
   recursive): RI_Set nesting follows the file's structure, which is
   attacker-controlled and could otherwise drive unbounded C-stack recursion.
   Uses an explicit heap-allocated work list instead. Returns 0 on success,
   PRC_ERROR_MEMORY if the work list couldn't grow -- unlike a void return,
   this lets the caller (which uses the resulting count to size later
   allocations) abort rather than silently proceed with an undercount. */
static int
prc_api_helper_get_ri_items(prc_context *ctx, prc_ri *ri, uint32_t *num_rep_items,
                            uint32_t file_index)
{
    prc_ri **worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;

    worklist = (prc_ri **)prc_malloc(ctx, worklist_capacity * sizeof(prc_ri *));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_get_ri_items\n");
        return PRC_ERROR_MEMORY;
    }
    worklist[worklist_size++] = ri;

    while (worklist_size > 0)
    {
        prc_ri *curr = worklist[--worklist_size];

        if (curr->representation_type == PRC_TYPE_RI_Set)
        {
            uint32_t k;
            *num_rep_items += 1;
            for (k = 0; k < curr->ri_set->number_of_items; k++)
            {
                prc_api_helper_set_inheritance_data(ctx, &curr->item_content.style_inheritance,
                    &curr->item_content.base.graphics_content,
                    &curr->ri_set->rep_items[k].item_content.style_inheritance, file_index);

                if (worklist_size >= worklist_capacity)
                {
                    prc_ri **new_worklist;
                    worklist_capacity *= 2;
                    new_worklist = (prc_ri **)prc_realloc(ctx, worklist,
                        worklist_capacity * sizeof(prc_ri *));
                    if (new_worklist == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_get_ri_items\n");
                        prc_free(ctx, worklist);
                        return PRC_ERROR_MEMORY;
                    }
                    worklist = new_worklist;
                }
                worklist[worklist_size++] = &curr->ri_set->rep_items[k];
            }
        }
        else
        {
            *num_rep_items += 1;
        }
    }

    prc_free(ctx, worklist);
    return 0;
}

/* A recusive method that goes through all the trees and figures out what we
   need to allocate node wise. It also looks at the bit behavior fields as
   we traverse the tree, looking at the ChildHerit settings to catch cases
   where the RI leaves (children) inherit the style, transparency of a parent.
   The spec is not clear on this. There is also a ParentHerit case which makes
   no sense and would involve going through the tree backwards. We will ignore
   that for now. */
typedef struct
{
    uint32_t file_index;
    prc_asm_product_occurrence *product;
} prc_api_count_items_work;

static int
prc_api_count_items_push(prc_context *ctx, prc_api_count_items_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint32_t file_index,
    prc_asm_product_occurrence *product)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_count_items_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_count_items_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_count_items_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].file_index = file_index;
    (*worklist)[*worklist_size].product = product;
    (*worklist_size)++;
    return 0;
}

/* A method that goes through all the trees and figures out what we need to
   allocate node wise. It also looks at the bit behavior fields as we traverse
   the tree, looking at the ChildHerit settings to catch cases where the RI
   leaves (children) inherit the style, transparency of a parent. The spec is
   not clear on this. There is also a ParentHerit case which makes no sense
   and would involve going through the tree backwards. We will ignore that
   for now.

   Iterative (not recursive): the product-occurrence tree (prototypes and
   children) follows the file's assembly structure, which is
   attacker-controlled and could otherwise drive unbounded C-stack recursion.
   Uses an explicit heap-allocated work list instead. Traversal order differs
   from the original strict pre-order recursion (this is a LIFO stack), but
   every node is still visited exactly once and each style_inheritance write
   still happens on the parent before the child is ever popped/processed, so
   the resulting counts and inheritance data are unaffected. */
static int
prc_api_helper_count_items(prc_context *ctx, prc_api_data data, uint32_t num_files,
    uint32_t curr_file_index, prc_asm_product_occurrence *product, uint32_t *num_parts,
    uint32_t *num_products, uint32_t *num_markups)
{
    prc_data *data_in = (prc_data *)data;
    prc_api_count_items_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    int result = 0;

    worklist = (prc_api_count_items_work *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_count_items_work));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_count_items\n");
        return PRC_ERROR_MEMORY;
    }
    worklist[worklist_size].file_index = curr_file_index;
    worklist[worklist_size].product = product;
    worklist_size++;

    while (worklist_size > 0)
    {
        prc_api_count_items_work item = worklist[--worklist_size];
        uint32_t work_curr_file_index = item.file_index;
        prc_asm_product_occurrence *work_product = item.product;
        uint32_t number_of_child_product_occurrences =
            work_product->references_product_occurrence.number_of_child_product_occurrences;
        prc_references_of_product_occurrence *product_refs =
            &work_product->references_product_occurrence;
        prc_unique_id file_id;
        uint32_t k;
        prc_asm_product_occurrence *child_product;
        int code;
        prc_asm_parts_definition *part;
        uint32_t biased_file_index;
        uint32_t file_index = work_curr_file_index;
        prc_asm_product_occurrence *prototype_product;
        uint8_t wants_cross_file_ref = 0;

        prc_api_helper_init_unique_id(ctx, &file_id);

        /* Include this product */
        *num_products += 1;

        /* Check if it has any markups */
        if (work_product->markups.number_of_markups > 0)
        {
            *num_products += 1; /* For the 3D PMI child that contains all the markups */
            *num_markups += work_product->markups.number_of_markups;
        }

        if (product_refs->biased_index_prototype != 0 && product_refs->prototype_in_same_file_structure.flag == 0)
        {
            file_id = product_refs->prototype_in_same_file_structure.unique_id;
            wants_cross_file_ref = 1;
        }

        if (product_refs->biased_index_external_data != 0 && product_refs->external_data_in_same_file_structure.flag == 0)
        {
            file_id = product_refs->external_data_in_same_file_structure.unique_id;
            wants_cross_file_ref = 1;
        }

        biased_file_index = wants_cross_file_ref
            ? prc_api_helper_get_biased_file_index_from_unique_id(ctx, data, file_id) : 0;
        if (biased_file_index != 0)
        {
            file_index = biased_file_index - 1;
        }

        /* biased_index_part/biased_index_prototype below are indices into
           the file structure named by file_id, not necessarily the current
           product's own file. If that file structure was requested but
           isn't among the ones currently loaded (e.g. a deliberately-
           isolated single-file-structure slice of a larger assembly, or any
           file missing an external reference target), file_index is still
           whatever it defaulted to above (the current product's own file) --
           using it would walk that file's part/product arrays with an index
           meant for a different, absent file. Skip both blocks below rather
           than reading out of range. */
        if (wants_cross_file_ref && biased_file_index == 0)
        {
            /* leave part/prototype unprocessed for this product */
        }
        else if (product_refs->biased_index_part != 0)
        {
            part = &data_in->file_struct[file_index].tree->parts[product_refs->biased_index_part - 1];

            /* Include this part */
            *num_parts += 1;

            /* Do any inheritence */
            prc_api_helper_set_inheritance_data(ctx, &work_product->style_inheritance,
                &work_product->base.graphics_content, &part->style_inheritance, file_index);

            /* Now count through all its rep items. These effect the parts count */
            for (k = 0; k < part->num_rep_items; k++)
            {
                prc_api_helper_set_inheritance_data(ctx, &part->style_inheritance,
                    &part->base.graphics_content, &part->rep_items[k].item_content.style_inheritance,
                    file_index);
                code = prc_api_helper_get_ri_items(ctx, &part->rep_items[k], num_parts, file_index);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_helper_get_ri_items\n");
                    result = code;
                    goto cleanup;
                }
            }
        }

        /* These are refs to the product. Continue to drill. Product is added in
            this call so don't add it here. Guarded by the same
            wants_cross_file_ref/biased_file_index check as the part block
            above -- see its comment. */
        if (!(wants_cross_file_ref && biased_file_index == 0) && product_refs->biased_index_prototype != 0)
        {
            prototype_product =
                &data_in->file_struct[file_index].tree->products[product_refs->biased_index_prototype - 1];
            prc_api_helper_set_inheritance_data(ctx, &work_product->style_inheritance,
                &prototype_product->base.graphics_content, &prototype_product->style_inheritance,
                file_index);
            if (prc_api_count_items_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                    file_index, prototype_product) < 0)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_count_items\n");
                result = PRC_ERROR_MEMORY;
                goto cleanup;
            }
        }

        if (number_of_child_product_occurrences > 0)
        {
            /* Note that these are in the curr_file_index. It is a little confusing
               because the parent may be referencing another file. But we treat
               this parents children as being in the current file. */
            for (k = 0; k < number_of_child_product_occurrences; k++)
            {
                if (product_refs->index_child_occurrence[k] >= (data_in->file_struct[work_curr_file_index].tree->product_count - 1))
                {
                    prc_error(ctx, PRC_API_INDEX, "Child reference index out of range in prc_api_helper_add_items\n");
                    result = PRC_API_INDEX;
                    goto cleanup;
                }
                child_product = &data_in->file_struct[work_curr_file_index].tree->products[product_refs->index_child_occurrence[k]];

                /* Check if we need to set any of the inheritance data in the child
                   based upon what is in the parent */
                prc_api_helper_set_inheritance_data(ctx, &work_product->style_inheritance,
                    &work_product->base.graphics_content, &child_product->style_inheritance,
                    file_index);

                /* Product is added in this call, so don't do it here */
                if (prc_api_count_items_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        work_curr_file_index, child_product) < 0)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_count_items\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
            }
        }
    }

cleanup:
    prc_free(ctx, worklist);
    return result;
}

/* Reworked version which starts out with the last product in the last file and
   recursively works its way through the data. This is the first call in
   and we have to start at a special place (last product in last file). */
PRC_EXPORT int
prc_api_prep_model_tree(prc_context *ctx, prc_api_data data, uint32_t *num_parts,
                        uint32_t *num_products, uint32_t *num_markups)
{
    uint32_t last_file_index;
    prc_data *data_in = (prc_data *)data;
    prc_asm_file_structure_tree *last_tree;
    prc_asm_product_occurrence *prc_product;
    int code;
    uint32_t num_files;

    num_files = data_in->file_structure_count;
    last_file_index = num_files - 1;
    last_tree = data_in->file_struct[last_file_index].tree;
    prc_product = &last_tree->products[last_tree->product_count - 1];

    *num_parts = 0;
    *num_markups = 0;
    *num_products = 1;
    /* We start with 1 because we have to account for the root node which is the
       last product in the last file */

   /* This is a recursive method to count the parts, products and markups
      so that we can allocate the proper amount of space. */
    code = prc_api_helper_count_items(ctx, data, num_files, last_file_index,
                                      prc_product, num_parts, num_products,
                                      num_markups);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_add_items\n");
        return code;
    }
    return 0;
}

/* Get the attributes and copy them into the api product type which will be
   in the model tree. */
static int
prc_api_helper_get_attributes(prc_context *ctx, prc_api_attributes *api_attributes,
    prc_attribute_data *attribute_data_in)
{
    uint32_t k, j;
    uint32_t num_base_attributes = attribute_data_in->attribute_count;

    if (num_base_attributes > 0)
    {
        api_attributes->num_base_attributes = num_base_attributes;
        api_attributes->base_attributes = (prc_api_attribute_base *)prc_calloc(ctx,
            num_base_attributes, sizeof(prc_api_attribute_base));
        if (api_attributes->base_attributes == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
            return PRC_ERROR_MEMORY;
        }

        for (j = 0; j < num_base_attributes; j++)
        {
            prc_misc_attribute *attribute_data = &attribute_data_in->attributes[j];
            prc_api_attribute_base *api_base_attr = &api_attributes->base_attributes[j];

            /* Deal with the title for the base attributes */
            if (attribute_data->attribute_title.string_title.string != NULL)
            {
                api_base_attr->attribute_base_title = (char *)prc_calloc(ctx,
                    strlen((const char *)attribute_data->attribute_title.string_title.string) + 1, sizeof(char));
                if (api_base_attr->attribute_base_title == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                    return PRC_ERROR_MEMORY;
                }
                memcpy(api_base_attr->attribute_base_title,
                    attribute_data->attribute_title.string_title.string,
                    strlen((const char *)attribute_data->attribute_title.string_title.string));
            }
            else
            {
                /* It may be one of the integer type titles */
                uint32_t title_int = attribute_data->attribute_title.integer_title;
                if (title_int >= prc_misc_attribute_TITLE && title_int < prc_misc_attribute_MAX)
                {
                    api_base_attr->attribute_base_title = (char *)prc_calloc(ctx,
                        strlen(prc_misc_attribute_NAMES[title_int]) + 1, sizeof(char));
                    if (api_base_attr->attribute_base_title == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                        return PRC_ERROR_MEMORY;
                    }
                    memcpy(api_base_attr->attribute_base_title,
                        prc_misc_attribute_NAMES[title_int],
                        strlen(prc_misc_attribute_NAMES[title_int]) + 1);
                }
                else
                    api_base_attr->attribute_base_title = NULL;
            }

            /* Now all the sub attributes of this one. This is almost always 1 */
            if (attribute_data->number_attributes > 0)
            {
                api_base_attr->num_attributes = attribute_data->number_attributes;
                api_base_attr->attributes = (prc_api_attribute_entry *)prc_calloc(ctx,
                    attribute_data->number_attributes, sizeof(prc_api_attribute_entry));
                if (api_base_attr->attributes == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                    return PRC_ERROR_MEMORY;
                }

                /* Now step through all the sub attributes */
                for (k = 0; k < attribute_data->number_attributes; k++)
                {
                    prc_api_attribute_entry *api_attr = &api_base_attr->attributes[k];
                    prc_attribute_key_value *prc_attr = &attribute_data->attributes[k];

                    if (prc_attr->title.string_title.string != NULL)
                    {
                        api_attr->entry_title = (char *)prc_calloc(ctx,
                            strlen((const char *)prc_attr->title.string_title.string) + 1, sizeof(char));
                        if (api_attr->entry_title == NULL)
                        {
                            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                            return PRC_ERROR_MEMORY;
                        }
                        memcpy(api_attr->entry_title, prc_attr->title.string_title.string,
                            strlen((const char *)prc_attr->title.string_title.string));
                    }
                    else
                    {
                        api_attr->entry_title = NULL;
                    }

                    switch (prc_attr->type)
                    {
                    case PRC_ATTRIBUTE_TYPE_INT:
                        api_attr->type = PRC_API_INTEGER_ATTRIBUTE;
                        api_attr->value_integer = prc_attr->value_integer;
                        break;
                    case PRC_ATTRIBUTE_TYPE_DOUBLE:
                        api_attr->type = PRC_API_DOUBLE_ATTRIBUTE;
                        api_attr->value_double = prc_attr->value_double;
                        break;
                    case PRC_ATTRIBUTE_TYPE_TIME32:
                        api_attr->type = PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE;
                        api_attr->value_secs_integer = prc_attr->value_secs_integer;
                        break;
                    case PRC_ATTRIBUTE_TYPE_CHAR_UTF8:
                        api_attr->type = PRC_API_STRING_ATTRIBUTE;
                        /* val_string.string is NULL whenever null_flag is set
                           (no string value at all, e.g. an empty CHAR_UTF8
                           attribute) -- confirmed the hard way via a crash
                           (strlen on NULL) reading a real-world file with
                           exactly this attribute shape. Produce an empty
                           string instead of dereferencing NULL. */
                        if (prc_attr->val_string.null_flag || prc_attr->val_string.string == NULL)
                        {
                            api_attr->value_string = (char *)prc_calloc(ctx, 1, sizeof(char));
                            if (api_attr->value_string == NULL)
                            {
                                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                                return PRC_ERROR_MEMORY;
                            }
                            break;
                        }
                        api_attr->value_string = (char *)prc_calloc(ctx,
                            strlen((const char *)prc_attr->val_string.string) + 1, sizeof(char));
                        if (api_attr->value_string == NULL)
                        {
                            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper _get_attributes\n");
                            return PRC_ERROR_MEMORY;
                        }
                        memcpy(api_attr->value_string, prc_attr->val_string.string,
                            strlen((const char *)prc_attr->val_string.string));
                        break;

                    case PRC_ATTRIBUTE_TYPE_TIME64:
                        api_attr->type = PRC_API_VALUE_TIME_ATTRIBUTE;
                        api_attr->value_time = ((uint64_t)prc_attr->value_time_msp) << 32 | prc_attr->value_time_lsp;
                        break;

                    default:
                        prc_error(ctx, PRC_ERROR_INTERNAL, "Unknown attribute type in prc_api_helper _get_attributes\n");
                        return PRC_ERROR_INTERNAL;
                    }
                }
            }
            else
            {
                api_base_attr->num_attributes = 0;
                api_base_attr->attributes = NULL;
            }
        }
    }
    else
    {
        api_attributes->num_base_attributes = 0;
        api_attributes->base_attributes = NULL;
    }
    return 0;
}

static int
prc_api_helper_init_model_node(prc_context *ctx, prc_api_product *product,
    const prc_type_asm_modelfile *model_in)
{
    static const char default_model_name[] = "Model";
    const unsigned char *file_name = NULL;
    size_t len_name;

    /* Prefer the name actually stored in the PRC_TYPE_ASM_ModelFile's own
       ContentPRCBase (same == 0 means a name follows), falling back to the
       generic "Model" placeholder only when the file carries none -- most
       PRC producers never set this field, hence the fallback. --
       Oddly Adobe always seems to set this to Model so lets do the same */
    if (model_in != NULL && model_in->base.name.same == 0 &&
        model_in->base.name.name.string != NULL && 0)
    {
        file_name = model_in->base.name.name.string;
    }

    product->is_model = 1;
    len_name = (file_name != NULL) ? strlen((const char *)file_name)
                                    : strlen(default_model_name);
    product->name = (char *)prc_calloc(ctx, len_name + 1, sizeof(char));
    if (product->name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(product->name, (file_name != NULL) ? (const char *)file_name : default_model_name, len_name);
    product->type = PRC_API_NODE_PRODUCT;
    product->num_children = 1;
    product->part = NULL;
    product->num_markups = 0;
    product->markup = NULL;
    product->file_index = 0;
    product->reserved = NULL;
    product->location.is_identity = 1;

    return 0;
}

/* Modified: create and attach style nodes for parts and RIs.
   - prc_api_helper_add_part now accepts reserve and parent_style.
   - prc_api_helper_add_ri now accepts parent_style and creates an RI style node,
     recursing with that node as parent so RI->RI chains become style children.
   - Caller updated in prc_api_helper_add_product to pass reserve and product_style.
*/

/* Forward declaration so callers above the definition don't get an implicit-int.
   This prevents MSVC treating prc_add_style_data as returning int. */
static prc_api_object_style *
prc_add_style_data(prc_context *ctx, prc_api_child_reserve *reserve,
    void *product_part_RI, prc_internal_api_object_type object_type, uint32_t file_index,
    prc_api_object_style *parent);

/* Adjusted: return the created style pointer so callers can pass it to RIs.
   Returns NULL on error. */
static prc_api_object_style *
prc_api_helper_add_part(prc_context *ctx, char *parent_name, int32_t file_index,
    prc_api_part *api_part, prc_asm_parts_definition *prc_part,
    uint32_t parent_biased_style_index,
    uint32_t parent_biased_style_file_index,
    prc_api_child_reserve *reserve,
    prc_api_object_style *parent_style)
{
    char part_name[] = "Unnamed Part";
    char *name;
    int code;
    prc_api_object_style *part_style = NULL;

    if (prc_part->base.base.name.name.string == NULL)
    {
        api_part->name_same_as_product = 1;
        if (parent_name != NULL)
        {
            name = parent_name;
        }
        else
        {
            name = part_name;
        }
    }
    else
    {
        api_part->name_same_as_product = 0;
        name = (char*) prc_part->base.base.name.name.string;
    }

    api_part->name = (char *)prc_calloc(ctx, strlen((const char*) name) + 1, sizeof(char));
    if (api_part->name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_part\n");
        return NULL;
    }
    memcpy(api_part->name, name, strlen(name));

    api_part->num_rep_items = prc_part->num_rep_items;
    api_part->rep_items = NULL;
    api_part->biased_style_index = prc_part->base.graphics_content.biased_index_of_line_style;
    api_part->tess = NULL;
    api_part->tess_file_index = file_index;

    code = prc_api_helper_get_attributes(ctx, &api_part->attributes,
        &prc_part->base.base.attribute_data);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_api_helper_get_attributes\n");
        return NULL;
    }

    /* Create style node for this part and attach to parent_style (if provided). */
    if (reserve != NULL)
    {
        part_style = prc_add_style_data(ctx, reserve, prc_part,
            PRC_INTERNAL_API_OBJECT_TYPE_PART, file_index, parent_style);
        if (part_style == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate style node for part in prc_api_helper_add_part\n");
            return NULL;
        }
    }

    return part_style;
}
typedef struct
{
    uint32_t file_index;
    prc_api_part *api_part;
    prc_ri *prc_ri;
    uint32_t parent_biased_style_index;
    uint32_t parent_biased_style_file_index;
    prc_api_object_style *parent_style;
} prc_api_add_ri_work;

static int
prc_api_add_ri_push(prc_context *ctx, prc_api_add_ri_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint32_t file_index,
    prc_api_part *api_part, prc_ri *prc_ri, uint32_t parent_biased_style_index,
    uint32_t parent_biased_style_file_index, prc_api_object_style *parent_style)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_add_ri_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_add_ri_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_add_ri_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].file_index = file_index;
    (*worklist)[*worklist_size].api_part = api_part;
    (*worklist)[*worklist_size].prc_ri = prc_ri;
    (*worklist)[*worklist_size].parent_biased_style_index = parent_biased_style_index;
    (*worklist)[*worklist_size].parent_biased_style_file_index = parent_biased_style_file_index;
    (*worklist)[*worklist_size].parent_style = parent_style;
    (*worklist_size)++;
    return 0;
}

/* It accepts parent_style and passes it down when recursing so RI->RI style
   parent/child relationships are created.

   Iterative (not recursive): RI_Set nesting follows the file's structure,
   which is attacker-controlled and could otherwise drive unbounded C-stack
   recursion. Uses an explicit heap-allocated work list instead. Each node's
   field assignments only depend on its own prc_ri/parent parameters (not on
   whether its children have been processed yet), so processing order across
   the tree doesn't affect the result. */
static int
prc_api_helper_add_ri(prc_context *ctx, uint32_t file_index, prc_api_part *api_part,
    prc_ri *prc_ri, uint32_t parent_biased_style_index,
    uint32_t parent_biased_style_file_index,
    prc_api_child_reserve *reserve,
    prc_api_object_style *parent_style)
{
    prc_api_add_ri_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    int result = 0;
    int code;

    worklist = (prc_api_add_ri_work *)prc_malloc(ctx, worklist_capacity * sizeof(prc_api_add_ri_work));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri\n");
        return PRC_ERROR_MEMORY;
    }
    if (prc_api_add_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity, file_index,
            api_part, prc_ri, parent_biased_style_index, parent_biased_style_file_index,
            parent_style) < 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri\n");
        prc_free(ctx, worklist);
        return PRC_ERROR_MEMORY;
    }

    while (worklist_size > 0)
    {
        prc_api_add_ri_work item = worklist[--worklist_size];
        char node_name[] = "node";
        char *name;
        prc_api_object_style *ri_style = NULL;

        if (item.prc_ri->item_content.base.base.name.name.string == NULL)
        {
            name = node_name;
        }
        else
        {
            name = (char*) item.prc_ri->item_content.base.base.name.name.string;
        }

        item.api_part->name = (char *)prc_calloc(ctx, strlen((const char*) name) + 1, sizeof(char));
        if (item.api_part->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri\n");
            result = PRC_ERROR_MEMORY;
            goto cleanup;
        }
        memcpy(item.api_part->name, name, strlen(name));

        /* Create style node for this RI and attach under parent_style */
        ri_style = prc_add_style_data(ctx, reserve, item.prc_ri,
            PRC_INTERNAL_API_OBJECT_TYPE_RI, item.file_index, item.parent_style);
        if (ri_style == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate style node for RI in prc_api_helper_add_ri\n");
            result = PRC_ERROR_MEMORY;
            goto cleanup;
        }

        /* Deal with any attributes that may exist for the RI item */
        code = prc_api_helper_get_attributes(ctx, &item.api_part->attributes,
            &item.prc_ri->item_content.base.base.attribute_data);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_api_helper_get_attributes\n");
            result = PRC_ERROR_MEMORY;
            goto cleanup;
        }

        if (item.prc_ri->representation_type == PRC_TYPE_RI_BrepModel ||
            item.prc_ri->representation_type == PRC_TYPE_RI_PolyBrepModel ||
            item.prc_ri->representation_type == PRC_TYPE_RI_PointSet)
        {
            item.api_part->num_rep_items = 0;
            item.api_part->rep_items = NULL;
        }
        else if (item.prc_ri->representation_type == PRC_TYPE_RI_Set)
        {
            uint32_t k;

            item.api_part->num_rep_items = item.prc_ri->ri_set->number_of_items;
            item.api_part->rep_items = &reserve->parts[reserve->part_index];
            reserve->part_index += item.prc_ri->ri_set->number_of_items;
            for (k = 0; k < item.prc_ri->ri_set->number_of_items; k++)
            {
                /* Pass ri_style as parent_style so nested RIs become children of this RI style */
                if (prc_api_add_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        item.file_index, &item.api_part->rep_items[k],
                        &item.prc_ri->ri_set->rep_items[k], item.parent_biased_style_index,
                        item.parent_biased_style_file_index, ri_style) < 0)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_ri\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
            }
        }

        item.api_part->RI_item_style_node = ri_style;
        item.api_part->biased_local_coordinate_index = item.prc_ri->item_content.biased_index_local_coordinate_system;
        item.api_part->biased_style_index = item.prc_ri->item_content.base.graphics_content.biased_index_of_line_style;
        item.api_part->biased_tess_index = item.prc_ri->item_content.biased_index_tessellation;
        item.api_part->tess_file_index = item.file_index;
        item.api_part->biased_layer_index = item.prc_ri->item_content.base.graphics_content.biased_layer_index;
        item.api_part->behavior_bit_field1 = item.prc_ri->item_content.base.graphics_content.behavior_bit_field1;
        item.api_part->behavior_bit_field2 = item.prc_ri->item_content.base.graphics_content.behavior_bit_field2;
        item.api_part->has_inherited_style = item.prc_ri->item_content.style_inheritance.has_any_inheritance;
        item.api_part->color.has_inheritance = item.prc_ri->item_content.style_inheritance.color.has_inheritance;
        item.api_part->color.inherited_value = item.prc_ri->item_content.style_inheritance.color.inherited_value;
        item.api_part->color.inherited_file_index = item.prc_ri->item_content.style_inheritance.color.inherited_file_index;
        item.api_part->transparency.has_inheritance = item.prc_ri->item_content.style_inheritance.transparency.has_inheritance;
        item.api_part->transparency.inherited_value = item.prc_ri->item_content.style_inheritance.transparency.inherited_value;
        item.api_part->transparency.inherited_file_index = item.prc_ri->item_content.style_inheritance.transparency.inherited_file_index;
        item.api_part->tess = NULL; /* This gets set when we do the tessellations */
        item.api_part->has_entity_ref = item.prc_ri->item_content.base.graphics_content.has_entity_ref;
        if (item.api_part->has_entity_ref)
        {
            item.api_part->entity_ref.behavior_bit_field1 = item.prc_ri->item_content.base.graphics_content.entity_ref.behavior_bit_field1;
            item.api_part->entity_ref.behavior_bit_field2 = item.prc_ri->item_content.base.graphics_content.entity_ref.behavior_bit_field2;
            item.api_part->entity_ref.biased_index_of_line_style = item.prc_ri->item_content.base.graphics_content.entity_ref.biased_index_of_line_style;
            item.api_part->entity_ref.biased_layer_index = item.prc_ri->item_content.base.graphics_content.entity_ref.biased_layer_index;
            item.api_part->entity_ref.file_index = item.prc_ri->item_content.base.graphics_content.entity_ref.file_index;
        }

        /* We can have this pseudo inheritance... */
        if (item.api_part->biased_style_index == 0 && item.api_part->has_inherited_style == 0 &&
            item.parent_biased_style_index != 0)
        {
            item.api_part->has_inherited_style = 1;
            item.api_part->color.has_inheritance = 1;
            item.api_part->color.inherited_value = item.parent_biased_style_index;
            item.api_part->color.inherited_file_index = item.parent_biased_style_file_index;
            item.api_part->transparency.has_inheritance = 0;
        }
    }

cleanup:
    prc_free(ctx, worklist);
    return result;
}


/* Incoming matrix has all the prototype transforms concatenated */
static int
prc_api_helper_copy_product_details(prc_context *ctx, prc_api_data data,
    int32_t file_index, char *base_name, prc_api_transform incoming_matrix,
    prc_asm_product_occurrence *product, prc_api_product *product_tree)
{
    char product_name[] = "Unnamed Product";
    char *name;
    int code;
    uint32_t k, j;
    uint32_t biased_file_index;
    uint32_t entity_ref_file_index;

    if (base_name == NULL)
    {
        name = product_name;
    }
    else
    {
        name = base_name;
    }

    product_tree->name = (char*)prc_calloc(ctx, strlen((const char*) name) + 1, sizeof(unsigned char));
    if (product_tree->name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_copy_product_details\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(product_tree->name, name, strlen(name));

#if 0
    /* Handy for debugging tree issues */
    if (name != NULL)
    {
        char temp_str[] = "MixingCover-01-1 (Default)\0";
        if (strncmp(name, temp_str, strlen(temp_str) - 1) == 0)
        {
            int zz = 1;
        }
    }
#endif

    memcpy(&product_tree->location, &incoming_matrix, sizeof(prc_api_transform));
    product_tree->is_model = 0;
    product_tree->type = PRC_API_NODE_PRODUCT;
    product_tree->num_children = 0;
    product_tree->children = NULL;
    product_tree->part = NULL;
    product_tree->num_markups = 0;
    product_tree->markup = NULL;
    product_tree->file_index = file_index;
    product_tree->reserved = NULL;

    code = prc_api_helper_get_attributes(ctx, &product_tree->attributes,
                                         &product->base.base.attribute_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_copy_product_details\n");
        return code;
    }

    /* Process the entity refs at this time, blowing away the current information
       that a part or RI may have in terms of style and location */
    if (product->entity_ref_count > 0)
    {
        for (k = 0; k < product->entity_ref_count; k++)
        {
            if (product->entity_reference->content_entity_ref.reference_data.ref_type == PRC_TYPE_MISC_ReferenceOnPRCBase)
            {
                if (!product->entity_reference->content_entity_ref.reference_data.non_topo_reference.flag)
                {
                    biased_file_index = prc_api_helper_get_biased_file_index_from_unique_id(ctx, data,
                        product->entity_reference->content_entity_ref.reference_data.non_topo_reference.different_unique_id);
                    if (biased_file_index != 0)
                    {
                        entity_ref_file_index = biased_file_index - 1;
                    }
                    else
                    {
                        /* Something wrong here. Just get out of it */
                        break;
                    }
                }
                else
                {
                    entity_ref_file_index = file_index;
                }

                uint32_t ref_index = product->entity_reference->content_entity_ref.reference_data.non_topo_reference.unique_id;

                /* Search through the reference items in ctx and see if we
                   need to update the style or location information based upon the reference item.
                   and its values */
                for (j = 0; j < ctx->graphics_content.ref_base_count; j++)
                {
                    prc_content_prc_ref_base *ref_base = (prc_content_prc_ref_base *)ctx->graphics_content.ref_base_ptr[j];

                    if (ref_index == ref_base->unique_id && file_index == ref_base->file_index && ref_base->parent_is_RI)
                    {
                        /* We have a match. Update the location and style information */
                        prc_representation_item_content *ri_item = (prc_representation_item_content*) ref_base->parent;

                        /* Update the style information based upon the reference. */
                        ri_item->base.graphics_content.has_entity_ref = 1;
                        ri_item->base.graphics_content.entity_ref.biased_index_of_line_style =
                            product->entity_reference->content_entity_ref.base.graphics_content.biased_index_of_line_style;
                        ri_item->base.graphics_content.entity_ref.biased_layer_index =
                            product->entity_reference->content_entity_ref.base.graphics_content.biased_layer_index;
                        ri_item->base.graphics_content.entity_ref.behavior_bit_field1 =
                            product->entity_reference->content_entity_ref.base.graphics_content.behavior_bit_field1;
                        ri_item->base.graphics_content.entity_ref.behavior_bit_field2 =
                            product->entity_reference->content_entity_ref.base.graphics_content.behavior_bit_field2;
                        ri_item->base.graphics_content.entity_ref.file_index = entity_ref_file_index;

                        /* TODO deal with changes in local references too */
                        if (product->entity_reference->content_entity_ref.index_of_local_coordinate != 0)
                        {
                            /* Throw and error for now so we catch this */
                            prc_error(ctx, PRC_ERROR_INTERNAL, "Local coordinate system reference in entity reference is not supported in prc_api_helper_copy_product_details\n");
                            return PRC_ERROR_INTERNAL;
                        }
                        break;
                    }
                }
            }
            else
            {
                /* TODO Brep case */
            }
        }
    }
    return 0;
}

static int
prc_api_helper_add_markups(prc_context *ctx, prc_api_data data, uint32_t num_files,
    uint32_t curr_file_index, prc_asm_product_occurrence *product,
    prc_api_child_reserve *reserve, prc_api_product *product_tree)
{
    int code;
    uint32_t k;
    char name_3d_pmi[] = "3D PMI";
    char name_markup[] = "Unnamed Markup";
    char *name;
    uint32_t num_markups = product->markups.number_of_markups;
    /* Grab the end */
    prc_api_product *pmi_3d_node = &product_tree->children[product_tree->num_children - 1];

    pmi_3d_node->type = PRC_API_NODE_MARKUP;
    pmi_3d_node->num_children = 0;
    pmi_3d_node->children = NULL;
    pmi_3d_node->part = NULL;
    pmi_3d_node->reserved = NULL;

    /* Set the transformation to identity */
    code = prc_api_set_transform_to_identity(ctx, &pmi_3d_node->location);

    /* Set name to name_3d_pmi */
    pmi_3d_node->name = (char*)prc_calloc(ctx, strlen((const char *)name_3d_pmi) + 1, sizeof(char));
    if (pmi_3d_node->name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(pmi_3d_node->name, name_3d_pmi, strlen((const char *)name_3d_pmi));

    /* Add the markup children to this node */
    pmi_3d_node->num_markups = num_markups;
    pmi_3d_node->markup = &reserve->markups[reserve->markup_index];
    reserve->markup_index += num_markups;

    for (k = 0; k < num_markups; k++)
    {
        prc_api_markup *markup = &pmi_3d_node->markup[k];

        if (product->markups.markups[k].base.base.name.name.string == NULL)
        {
            name = name_markup;
        }
        else
        {
            name = (char*) product->markups.markups[k].base.base.name.name.string;
        }

        markup->name = (char*)prc_calloc(ctx, strlen((const char*) name) + 1,
                                                        sizeof(char));
        if (markup->name == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_initialize_node\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(markup->name, name, strlen(name));

        markup->biased_style_index =
            product->markups.markups[k].base.graphics_content.biased_index_of_line_style;
        markup->biased_tess_index =
            product->markups.markups[k].biased_index_tessellation;
        markup->tess = NULL;
        markup->file_index = curr_file_index;
    }
    return 0;
}

static void
prc_api_helper_update_parent_style(prc_context *ctx, prc_asm_product_occurrence *product,
    uint32_t current_file_index, uint32_t *style_index, uint32_t *style_file_index)
{
    if (product->base.graphics_content.biased_index_of_line_style != 0)
    {
        *style_index = product->base.graphics_content.biased_index_of_line_style;
        *style_file_index = current_file_index;
    }
}

static prc_api_object_style *
prc_add_style_data(prc_context *ctx, prc_api_child_reserve *reserve,
    void *product_part_RI, prc_internal_api_object_type object_type, uint32_t file_index,
    prc_api_object_style *parent)
{
    prc_api_object_style *root;
    prc_api_object_style *node;

    if (reserve == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "reserve is NULL in prc_add_style_data\n");
        return NULL;
    }

    /* Ensure style pool exists (lazy init as before) */
    if (reserve->style_pool == NULL)
    {
        int rc = prc_internal_api_style_pool_init(ctx, reserve,
            reserve->num_products, reserve->num_parts, reserve->num_markups);
        if (rc < 0)
        {
            prc_error(ctx, rc, "Failed to initialize style pool in prc_add_style_data\n");
            return NULL;
        }
    }

    node = prc_internal_api_style_pool_alloc(ctx, reserve);
    if (node == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Style pool allocation failed in prc_add_style_data\n");
        return NULL;
    }

    /* initialize node */
    node->file_index = file_index;
    node->base_with_graphics = NULL;
    node->num_entity_references = 0;
    node->entity_references = NULL;
    node->parent = parent;
    node->children = NULL;
    node->num_children = 0;

    /* Populate styles depending on type */
    switch (object_type)
    {
    case PRC_INTERNAL_API_OBJECT_TYPE_PRODUCT:
        node->base_with_graphics = &((prc_asm_product_occurrence *)product_part_RI)->base;
        node->num_entity_references = ((prc_asm_product_occurrence *)product_part_RI)->entity_ref_count;
        if (node->num_entity_references > 0)
            node->entity_references = ((prc_asm_product_occurrence *)product_part_RI)->entity_reference;
        break;
    case PRC_INTERNAL_API_OBJECT_TYPE_PART:
        node->base_with_graphics = &((prc_asm_parts_definition *)product_part_RI)->base;
        break;
    case PRC_INTERNAL_API_OBJECT_TYPE_RI:
    {
        prc_ri *prc_rep_item = (prc_ri *)product_part_RI;
        node->base_with_graphics = &prc_rep_item->item_content.base;
        node->brep_type = prc_rep_item->representation_type;
        if (prc_rep_item->representation_type == PRC_TYPE_RI_BrepModel)
        {
            node->index_of_topological_index = prc_rep_item->ri_brep_model->index_topological_context;
            node->index_of_body = prc_rep_item->ri_brep_model->index_body;
        }
    }
        break;
    default:
        prc_error(ctx, PRC_ERROR_INTERNAL, "Unknown object type in prc_add_style_data\n");
        return NULL;
    }

    /* Attach into tree:
       - if parent == NULL and root not set => node becomes root
       - if parent != NULL => append into parent's children array
       - if parent == NULL and root already set => append to root */
    if (parent == NULL)
    {
        if (reserve->style_tree_root == NULL)
        {
            reserve->style_tree_root = node;
            return node;
        }
        /* attach to root if no explicit parent */
        parent = reserve->style_tree_root;
    }

    /* grow parent's children array by one */
    prc_api_object_style **new_children =
        (prc_api_object_style **)prc_realloc(ctx, parent->children,
            sizeof(prc_api_object_style *) * (parent->num_children + 1));
    if (new_children == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow style children array in prc_add_style_data\n");
        return NULL;
    }
    new_children[parent->num_children] = node;
    parent->children = new_children;
    node->parent = parent;
    parent->num_children += 1;

    return node;
}

typedef struct
{
    uint32_t curr_file_index;
    unsigned char *base_product_name;
    prc_api_transform incoming_transform;
    prc_asm_product_occurrence *product;
    uint32_t parent_biased_style_index;
    uint32_t parent_biased_style_file_index;
    prc_api_product *product_tree;
    prc_api_object_style *parent_style;
} prc_api_add_product_work;

static int
prc_api_add_product_push(prc_context *ctx, prc_api_add_product_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity,
    uint32_t curr_file_index, unsigned char *base_product_name,
    prc_api_transform incoming_transform, prc_asm_product_occurrence *product,
    uint32_t parent_biased_style_index, uint32_t parent_biased_style_file_index,
    prc_api_product *product_tree, prc_api_object_style *parent_style)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_api_add_product_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_api_add_product_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_api_add_product_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }

#if 0
    /* Handy for debugging tree issues */
    if (base_product_name != NULL)
    {
        char temp_str[] = "MixingCover-01-1 (Default)\0";
        if (strncmp(base_product_name, temp_str, strlen(temp_str) - 1) == 0)
        {
            int zz = 1;
        }
    }
#endif

    (*worklist)[*worklist_size].curr_file_index = curr_file_index;
    (*worklist)[*worklist_size].base_product_name = base_product_name;
    (*worklist)[*worklist_size].incoming_transform = incoming_transform;
    (*worklist)[*worklist_size].product = product;
    (*worklist)[*worklist_size].parent_biased_style_index = parent_biased_style_index;
    (*worklist)[*worklist_size].parent_biased_style_file_index = parent_biased_style_file_index;
    (*worklist)[*worklist_size].product_tree = product_tree;
    (*worklist)[*worklist_size].parent_style = parent_style;
    (*worklist_size)++;
    return 0;
}

/* Capture the part style and pass it as parent to RI creation.

   Iterative (not recursive): the product-occurrence tree (prototype chains
   and child occurrences) follows the file's assembly structure, which is
   attacker-controlled and could otherwise drive unbounded C-stack recursion.
   Uses an explicit heap-allocated work list instead.

   Two recursion shapes existed in the original: (1) a pure prototype-chain
   "tail" case (product has a prototype and no children of its own) that just
   delegated entirely to the prototype, writing into the SAME product_tree
   node -- converted to a plain inner loop that reassigns its working state
   instead of recursing; (2) genuine branching over child product
   occurrences, each getting its own product_tree node -- converted to
   pushing onto the outer work list. */
static int
prc_api_helper_add_product(prc_context *ctx, prc_api_data data, uint32_t num_files,
    uint32_t curr_file_index, unsigned char *base_product_name,
    prc_api_transform incoming_transform,
    prc_asm_product_occurrence *product,
    uint32_t parent_biased_style_index,
    uint32_t parent_biased_style_file_index,
    prc_api_product *product_tree, prc_api_child_reserve *reserve,
    prc_api_object_style *parent_style)
{
    prc_data *data_in = (prc_data *)data;
    prc_api_add_product_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;
    int result = 0;

    worklist = (prc_api_add_product_work *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_api_add_product_work));
    if (worklist == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_product\n");
        return PRC_ERROR_MEMORY;
    }
    if (prc_api_add_product_push(ctx, &worklist, &worklist_size, &worklist_capacity,
            curr_file_index, base_product_name, incoming_transform, product,
            parent_biased_style_index, parent_biased_style_file_index,
            product_tree, parent_style) < 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_product\n");
        prc_free(ctx, worklist);
        return PRC_ERROR_MEMORY;
    }

    while (worklist_size > 0)
    {
        prc_api_add_product_work item = worklist[--worklist_size];
        uint32_t work_curr_file_index = item.curr_file_index;
        unsigned char *work_base_product_name = item.base_product_name;
        prc_api_transform work_incoming_transform = item.incoming_transform;
        prc_asm_product_occurrence *work_product = item.product;
        uint32_t work_parent_biased_style_index = item.parent_biased_style_index;
        uint32_t work_parent_biased_style_file_index = item.parent_biased_style_file_index;
        prc_api_product *work_product_tree = item.product_tree;
        prc_api_object_style *work_parent_style = item.parent_style;

        for (;;)
        {
            uint32_t number_of_child_product_occurrences =
                work_product->references_product_occurrence.number_of_child_product_occurrences;
            prc_references_of_product_occurrence *product_refs =
                &work_product->references_product_occurrence;
            prc_unique_id file_id;
            uint32_t k;
            prc_asm_product_occurrence *child_product;
            int code;
            prc_asm_parts_definition *part;
            uint32_t biased_file_index;
            uint32_t file_index = work_curr_file_index;
            prc_asm_product_occurrence *prototype_product;
            prc_api_transform new_transform;
            uint8_t wants_cross_file_ref = 0;
            char *base_name;
            prc_api_object_style *product_style;

            if (work_base_product_name != NULL)
            {
                base_name = (char *)work_base_product_name;
            }
            else
            {
                base_name = (char *)work_product->base.base.name.name.string;
            }

#if 0
            /* Handy for debugging tree issues */
            if (base_name != NULL)
            {
                char temp_str[] = "MixingCover-01-1 (Default)\0";
                if (strncmp(base_name, temp_str, strlen(temp_str) - 1) == 0)
                {
                    int zz = 1;
                }
            }
#endif

            prc_api_helper_init_unique_id(ctx, &file_id);

            if (product_refs->biased_index_prototype != 0 && product_refs->prototype_in_same_file_structure.flag == 0)
            {
                file_id = product_refs->prototype_in_same_file_structure.unique_id;
                wants_cross_file_ref = 1;
            }

            if (product_refs->biased_index_external_data != 0 && product_refs->external_data_in_same_file_structure.flag == 0)
            {
                file_id = product_refs->external_data_in_same_file_structure.unique_id;
                wants_cross_file_ref = 1;
            }

            biased_file_index = wants_cross_file_ref
                ? prc_api_helper_get_biased_file_index_from_unique_id(ctx, data, file_id) : 0;
            if (biased_file_index != 0)
            {
                file_index = biased_file_index - 1;
            }

            /* create style node for this product and keep pointer for children to use */
            product_style = prc_add_style_data(ctx, reserve, work_product,
                PRC_INTERNAL_API_OBJECT_TYPE_PRODUCT, file_index, work_parent_style);
            if (product_style == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_add_style_data\n");
                result = PRC_ERROR_MEMORY;
                goto cleanup;
            }

            /* There is a prototypes to this product. Continue to drill but concatenate
               any transform as we go. */
            if (work_product->has_transform)
            {
                code = prc_api_set_transform(ctx, &new_transform, &work_product->location);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_helper_add_items\n");
                    result = code;
                    goto cleanup;
                }
                prc_api_update_transform(ctx, &work_incoming_transform, &new_transform);
            }

            /* !(wants_cross_file_ref && biased_file_index == 0): don't follow
               a prototype reference whose target file structure couldn't be
               resolved (see the identical guard/comment in
               prc_api_helper_count_items) -- file_index would still be this
               product's own file, not the one biased_index_prototype below
               is an index into. Fall through to the "last prototype" case
               instead of reading out of range. */
            if (!(wants_cross_file_ref && biased_file_index == 0) &&
                product_refs->biased_index_prototype != 0 &&
                product_refs->number_of_child_product_occurrences == 0)
            {
                /* IMPORTANT: pass product_style as the parent for the prototype
                   chain so it builds as a single-child lineage under this node. */
                prototype_product =
                    &data_in->file_struct[file_index].tree->products[product_refs->biased_index_prototype - 1];
                prc_api_helper_update_parent_style(ctx, prototype_product, file_index,
                    &work_parent_biased_style_index, &work_parent_biased_style_file_index);

                /* Tail case: same product_tree, follow the prototype chain
                   in place instead of recursing. */
                work_curr_file_index = file_index;
                work_base_product_name = (unsigned char *)base_name;
                work_product = prototype_product;
                work_parent_style = product_style;
                continue;
            }
            else
            {
                /* At last prototype. Copy over the product details */
                prc_api_helper_update_parent_style(ctx, work_product, file_index,
                    &work_parent_biased_style_index, &work_parent_biased_style_file_index);
                code = prc_api_helper_copy_product_details(ctx, data, file_index,
                    base_name, work_incoming_transform, work_product, work_product_tree);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_helper_add_product_details\n");
                    result = code;
                    goto cleanup;
                }
            }

            /* We should be at the last prototype. At this point work on the child
               product occurrences AND add the part if there is one. The carburetor
               for example has a tree that has this occur (which is a part and
               child product occurrences) */
            if (product_refs->biased_index_part != 0)
            {
                part = &data_in->file_struct[file_index].tree->parts[product_refs->biased_index_part - 1];
                work_product_tree->part = &reserve->parts[reserve->part_index];

                /* We may want to avoid adding this part if its name is null.
                   Capture the created part style so we can pass it to RIs. */
                prc_api_object_style *part_style = prc_api_helper_add_part(ctx,
                    work_product_tree->name, file_index,
                    work_product_tree->part, part, work_parent_biased_style_index,
                    work_parent_biased_style_file_index, reserve, product_style);
                if (part_style == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_api_helper_add_part\n");
                    result = PRC_ERROR_MEMORY;
                    goto cleanup;
                }
                reserve->part_index += 1;

                /* Now add the rep items as children of the part.
                    These effect the parts count */
                if (part->num_rep_items > 0)
                {
                    work_product_tree->part->rep_items = &reserve->parts[reserve->part_index];
                    reserve->part_index += part->num_rep_items;

                    for (k = 0; k < part->num_rep_items; k++)
                    {
                        code = prc_api_helper_add_ri(ctx, file_index,
                            &work_product_tree->part->rep_items[k],
                            &part->rep_items[k], work_parent_biased_style_index,
                            work_parent_biased_style_file_index, reserve, part_style);
                        if (code < 0)
                        {
                            prc_error(ctx, code, "Failed in prc_api_helper_add_part\n");
                            result = code;
                            goto cleanup;
                        }
                    }
                }
                else
                {
                    work_product_tree->part->rep_items = NULL;
                    work_product_tree->part->num_rep_items = 0;
                }
            }

            /* If we have any children deal with those now */
            if (number_of_child_product_occurrences > 0)
            {
                /* Grab number_of_child_product_occurrences from the reserve. Also,
                   update the children count if we have any markups */
                uint8_t has_markups = work_product->markups.number_of_markups > 0 ? 1 : 0;
                work_product_tree->num_children = number_of_child_product_occurrences + has_markups;
                work_product_tree->children = &reserve->products[reserve->product_index];
                reserve->product_index += number_of_child_product_occurrences + has_markups;

                for (k = 0; k < number_of_child_product_occurrences; k++)
                {
                    if (product_refs->index_child_occurrence[k] >= (data_in->file_struct[work_curr_file_index].tree->product_count - 1))
                    {
                        prc_error(ctx, PRC_API_INDEX, "Child reference index out of range in prc_api_helper_add_items\n");
                        result = PRC_API_INDEX;
                        goto cleanup;
                    }
                    child_product = &data_in->file_struct[work_curr_file_index].tree->products[product_refs->index_child_occurrence[k]];

                    /* Process the product child but with the identity transform */
                    code = prc_api_set_transform_to_identity(ctx, &new_transform);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed in prc_api_set_transform_to_identity\n");
                        result = code;
                        goto cleanup;
                    }
                    prc_api_helper_update_parent_style(ctx, child_product, file_index,
                        &work_parent_biased_style_index, &work_parent_biased_style_file_index);
                    /* pass product_style so child style nodes attach under this product's style */
                    if (prc_api_add_product_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                            work_curr_file_index, NULL, new_transform, child_product,
                            work_parent_biased_style_index, work_parent_biased_style_file_index,
                            &work_product_tree->children[k], product_style) < 0)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_helper_add_product\n");
                        result = PRC_ERROR_MEMORY;
                        goto cleanup;
                    }
                }
            }

            /* Add the markups */
            if (work_product->markups.number_of_markups > 0)
            {
                if (number_of_child_product_occurrences == 0)
                {
                    /* We didn't have any child product occurrences but we have markups.
                       So we need to add a child for the 3D PMI and then add the markups
                       under that. */
                    work_product_tree->num_children = 1;
                    work_product_tree->children = &reserve->products[reserve->product_index];
                    reserve->product_index += 1;
                }

                /* Add the base 3D PMI product and then the markups. We already grabbed
                   an extra node for the 3D PMI entry. It goes as the end */
                code = prc_api_helper_add_markups(ctx, data, num_files, work_curr_file_index,
                    work_product, reserve, work_product_tree);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_api_helper_add_markups\n");
                    result = code;
                    goto cleanup;
                }
            }
            break;
        }
    }

cleanup:
    prc_free(ctx, worklist);
    return result;
}

/* A special handling of the first node prior to the recursion */
static int
prc_api_helper_first_node(prc_context *ctx, prc_api_data data, uint32_t num_files,
    prc_api_child_reserve *reserve, prc_api_product *product_tree)
{
    uint32_t last_file_index = num_files - 1;
    prc_data *data_in = (prc_data *)data;
    prc_asm_file_structure_tree *last_tree = data_in->file_struct[last_file_index].tree;
    prc_filestructure *files = data_in->file_struct;
    prc_asm_product_occurrence *prc_product = &last_tree->products[last_tree->product_count - 1];
    prc_references_of_product_occurrence product_refs = prc_product->references_product_occurrence;
    uint32_t num_children = product_refs.number_of_child_product_occurrences;
    uint32_t k;
    prc_api_transform product_transform; /* This keeps track of transforms through the prototypes */
    int code;

    /* Initialize our model node */
    code = prc_api_helper_init_model_node(ctx, product_tree, files[last_file_index].model);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_init_model_node\n");
        return code;
    }

    /* Special handling for the first case */
    reserve->product_index += 1;
    product_tree->children = &reserve->products[reserve->product_index];
    reserve->product_index += 1;

    /* Start the recusion starting from the last product in the last file */
    code = prc_api_set_transform_to_identity(ctx, &product_transform);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_copy_product_details\n");
        return code;
    }

    /* pass NULL as parent_style for the top-level call */
    prc_api_helper_add_product(ctx, data, num_files, last_file_index, NULL,
        product_transform, &last_tree->products[last_tree->product_count - 1],
        0, 0, &product_tree->children[0], reserve, NULL);

    return 0;
}

PRC_EXPORT int
prc_api_create_model_tree(prc_context *ctx, prc_api_data data,
    prc_api_product **product_tree_in, uint32_t num_parts, uint32_t num_products,
    uint32_t num_markups)
{
    prc_api_child_reserve *reserve;
    prc_api_product *product_tree;
    int code;
    uint32_t num_files;
    prc_data *data_in = (prc_data *)data;

    num_files = data_in->file_structure_count;

    /* Create the reserve based upon the counts */
    reserve = (prc_api_child_reserve *)prc_calloc(ctx, sizeof(prc_api_child_reserve), 1);
    if (reserve == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_create_model_tree\n");
        return PRC_ERROR_MEMORY;
    }
    code = prc_api_allocate_reserve(ctx, num_parts, num_products, num_markups,
                                    reserve);
    if (code < 0)
    {   prc_error(ctx, code, "Failed in prc_api_allocate_reserve\n");
        return code;
    }

    product_tree = &reserve->products[reserve->product_index];
    code = prc_api_helper_first_node(ctx, data, num_files, reserve, product_tree);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_first_node\n");
        return code;
    }

    product_tree->reserved = (void *)reserve;
    *product_tree_in = product_tree;

#if 0
    prc_api_print_tree(ctx, product_tree, 0);
#endif
    return 0;
}

PRC_EXPORT void
prc_api_release_tessellation_vertices(prc_context *ctx, prc_api_tess *tess)
{
    if (tess->tess_vertices.vertices != NULL)
    {
        prc_free(ctx, tess->tess_vertices.vertices);
        tess->tess_vertices.vertices = NULL;
    }
    tess->tess_vertices.num_vertices = 0;
    tess->tess_vertices.capacity = 0;
}

static int
prc_api_helper(prc_context *ctx, prc_data *data_in, uint32_t file_index,
    uint32_t tess_index, uint32_t part_index, uint32_t style_index,
    uint8_t has_inherited_style,
    uint32_t inherited_color_index, uint32_t inherited_color_file_index,
    uint8_t has_inherited_color, uint32_t inherited_trans_index,
    uint32_t inherited_trans_file_index, uint8_t has_inherited_trans)
{
    prc_data *data = (prc_data *)data_in;
    tess_style_file_part *part_details = data->part_details;
    uint32_t num_details = data->unique_part_count;
    uint32_t i;
    uint8_t found = 0;

    for (i = 0; i < num_details; i++)
    {
        if (part_details[i].tess_file_index == file_index &&
            part_details[i].tessellation_index == tess_index &&
            part_details[i].style_biased_index == style_index &&
            part_details[i].has_inherited_style == has_inherited_style &&
            part_details[i].inherited_color_index == inherited_color_index &&
            part_details[i].inherited_color_file_index == inherited_color_file_index &&
            part_details[i].has_inherited_color == has_inherited_color &&
            part_details[i].inherited_trans_index == inherited_trans_index &&
            part_details[i].inherited_trans_file_index == inherited_trans_file_index &&
            part_details[i].has_inherited_trans == has_inherited_trans)
        {
            return i;
        }
    }
    return -1;
}

/* Gets number of tessellations in the file AND build our table relating the
   tessellations with the styles and the parts across the files. We also have
   to add in the Markup tessellations */
PRC_EXPORT int
prc_api_get_number_tessellations(prc_context *ctx, prc_api_data data_in,
                                 prc_api_product *model_tree, uint32_t *num_tess,
                                 uint32_t *num_line_tess)
{
    prc_data *data = (prc_data *)data_in;
    uint32_t num_files;
    uint32_t num_part_tessellations = 0;
    uint32_t num_markup_tessellations = 0;
    uint32_t num_tess_in_file;
    uint32_t i, j, k ,m;
    prc_api_child_reserve *reserve = (prc_api_child_reserve *)model_tree->reserved;
    uint32_t num_parts = reserve->num_parts;
    uint32_t num_markups = reserve->num_markups;
    uint32_t num_details = PARTS_DETAIL_INIT_SIZE;
    prc_api_part *part;
    uint32_t style_index;
    uint8_t has_inherited_style;
    uint32_t inherited_color_index;
    uint32_t inherited_color_file_index;
    uint32_t inherited_trans_index;
    uint32_t inherited_trans_file_index;
    uint8_t has_inherited_color;
    uint8_t has_inherited_trans;
    int32_t part_detail_index;
    tess_style_file_part *part_detail;
    uint32_t found_part_match;
    uint32_t markup_index = 0;
    prc_filestructure *file_struct;
    prc_tess *tess;
    prc_tesslation_t tess_type;

    *num_line_tess = 0;

    data->part_details = (tess_style_file_part *)prc_calloc(ctx, sizeof(tess_style_file_part), PARTS_DETAIL_INIT_SIZE);
    if (data->part_details == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_get_number_tessellations\n");
        return PRC_ERROR_MEMORY;
    }

    if (num_markups > 0)
    {
        data->markup_details = (tess_style_file_markup *)prc_calloc(ctx, sizeof(tess_style_file_markup), num_markups);
        if (data->markup_details == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_get_number_tessellations\n");
            return PRC_ERROR_MEMORY;
        }
    }

    num_files = data->file_structure_count;

    for (i = 0; i < num_files; i++)
    {
        num_tess_in_file = data->file_struct[i].tessellation->tess_count;
        for (j = 0; j < num_tess_in_file; j++)
        {
            found_part_match = 0;

            /* Search the reserve for parts that have this tessellation and file.
               Note that the tessellation could be for a markup so we need
               to be prepared for that case */
            for (k = 0; k < num_parts; k++)
            {
                part = &reserve->parts[k];

                if (part->name != NULL)
                {
                    if (strncmp((const char *)part->name, "FDD-58098.ipt:2", 15) == 0)
                    {
                        int debug = 1;
                    }
                }

                if (part->biased_tess_index > 0 && part->biased_tess_index - 1 == j &&
                    part->tess_file_index == i)
                {
                    style_index = part->biased_style_index;
                    found_part_match = 1;

                    /* Get the style information */
                    if (part->has_inherited_style)
                    {
                        has_inherited_style = true;
                        if (part->color.has_inheritance)
                        {
                            inherited_color_index = part->color.inherited_value;
                            inherited_color_file_index = part->color.inherited_file_index;
                            has_inherited_color = 1;
                        }
                        else
                        {
                            inherited_color_index = 0;
                            inherited_color_file_index = 0;
                            has_inherited_color = 0;

                        }
                        if (part->transparency.has_inheritance)
                        {
                            inherited_trans_index = part->transparency.inherited_value;
                            inherited_trans_file_index = part->transparency.inherited_file_index;
                            has_inherited_trans = 1;
                        }
                        else
                        {
                            inherited_trans_index = 0;
                            inherited_trans_file_index = 0;
                            has_inherited_trans = 0;
                        }
                    }
                    else
                    {
                        has_inherited_style = false;
                        inherited_color_index = 0;
                        inherited_color_file_index = 0;
                        inherited_trans_index = 0;
                        inherited_trans_file_index = 0;
                        has_inherited_color = 0;
                        has_inherited_trans = 0;
                    }

                    /* Check for existence of this one already */
                    part_detail_index = prc_api_helper(ctx, data, i, j, k,
                        style_index, has_inherited_style,
                        inherited_color_index, inherited_color_file_index,
                        has_inherited_color, inherited_trans_index,
                        inherited_trans_file_index, has_inherited_trans);

                    if (part_detail_index >= 0)
                    {
                        /* We found this combination. Assign the part detail
                        *  index to this part and continue to the next part.
                        *  We do not need to add a new tessellation as this is
                        *  shared amongst multiple parts (likely with a spatial
                        *  location transform) */
                        reserve->parts[k].part_detail_index = part_detail_index;
                        continue;
                    }

                    /* We need to add a new one to the part details */
                    if (num_part_tessellations >= num_details)
                    {
                        tess_style_file_part *new_part_details;
                        num_details *= 2;
                        new_part_details = (tess_style_file_part *)prc_realloc(ctx,
                            data->part_details,
                            sizeof(tess_style_file_part) * num_details);
                        if (new_part_details == NULL)
                        {
                            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_api_get_number_tessellations\n");
                            return PRC_ERROR_MEMORY;
                        }
                        data->part_details = new_part_details;
                    }

                    part_detail = &data->part_details[num_part_tessellations];
                    part_detail->style_leaf = (void *)reserve->parts[k].RI_item_style_node;
                    part_detail->part_reserve_index = k;
                    part_detail->tessellation_index = j;
                    part_detail->tess_file_index = i;
                    part_detail->tessellation = NULL; /* This gets set later when we do the tessellations */
                    part_detail->wire_tessellation = NULL; /* Only added if we have this data */
                    part_detail->style_biased_index = style_index; /* Note that the file index for this has to be the same as the tessellation */
                    part_detail->has_inherited_style = has_inherited_style; /* These inherited styles can be different files */
                    part_detail->has_inherited_color = has_inherited_color;
                    part_detail->has_inherited_trans = has_inherited_trans;
                    part_detail->inherited_color_index = inherited_color_index;
                    part_detail->inherited_trans_index = inherited_trans_index;
                    part_detail->inherited_color_file_index = inherited_color_file_index;
                    part_detail->inherited_trans_file_index = inherited_trans_file_index;
                    if (part->has_entity_ref)
                    {
                        part_detail->has_entity_ref = 1;
                        part_detail->entity_ref = part->entity_ref;
                    }
                    else
                    {
                        part_detail->has_entity_ref = 0;
                    }

                    num_part_tessellations++;

                    /* Grab that tessellation and have a look to see if it has
                       any wire information and is a 3D type.  If yes, then we
                       are going to add a special wire structure for this */
                    file_struct = &data->file_struct[i];
                    tess = &file_struct->tessellation->tess[j];
                    tess_type = tess->tess_type;

                    if (tess_type == PRC_TYPE_TESS_3D)
                    {
                        prc_tess_3d *tess3d = tess->tess_3d;
                        if (tess3d->number_of_wire_indices > 0)
                        {
                            (*num_line_tess)++;
                        }
                    }
                    else if (tess_type == PRC_TYPE_TESS_3D_Compressed)
                    {
                        prc_tess_3d_compressed *tess_compressed = tess->tess_3d_compressed;

                        if (tess_compressed->number_of_edges > 0)
                        {
                            (*num_line_tess)++;
                        }
                    }
                }
            }
            if (!found_part_match)
            {
                /* We didn't find a part that references this tessellation.
                   However we could still have a markup that references
                   this tessellation. In fact, there can be multiple references
                   that use this same markup tessellation, so we need to go
                   through all of the markups and not break after finding one. */
                for (k = 0; k < num_markups; k++)
                {
                    prc_api_markup *markup = &reserve->markups[k];
                    if (markup->biased_tess_index > 0 && markup->biased_tess_index - 1 == j &&
                        markup->file_index == i)
                    {
                        data->markup_details[markup_index].markup_reserve_index = k;
                        data->markup_details[markup_index].tessellation_index = j;
                        data->markup_details[markup_index].tess_file_index = i;
                        data->markup_details[markup_index].style_biased_index = markup->biased_style_index;
                        data->markup_details[markup_index].tessellation = NULL; /* This gets set later when we do the tessellations */
                        markup_index++;
                        num_markup_tessellations++;
                    }
                }
            }
        }
    }
    data->unique_part_count = num_part_tessellations;
    data->unique_markup_count = num_markup_tessellations;
    *num_tess = num_part_tessellations + num_markup_tessellations;
    return 0;
}

/* Here we are looking for multiple styles for a tessellation. For counting
   purposes.  For example same tessellations but different styles (or inherited
   values) */
static uint32_t
prc_internal_api_search_for_multiple_styles(prc_context *ctx,
    prc_api_product *api_tree, uint32_t file_index, uint32_t tess_index)
{
    prc_api_child_reserve *reserve = (prc_api_child_reserve *)api_tree->reserved;
    uint32_t num_parts = reserve->num_parts;
    uint32_t num_markups = reserve->num_markups;
    uint32_t prior_biased_style_index = 0;
    uint8_t prior_set = false;
    uint32_t style_count = 0;
    uint32_t biased_style_index;
    uint32_t file_style_index; /* Only used in inherited case */

    for (uint32_t k = 0; k < num_parts; k++)
    {
        if (reserve->parts[k].biased_tess_index > 0)
        {
            if (reserve->parts[k].biased_tess_index - 1 == tess_index &&
                reserve->parts[k].tess_file_index == file_index)
            {
                /* We are at the tessellation that is in this file. Now we
                   have to be careful. The style could be defined or inherited */
                biased_style_index = reserve->parts[k].biased_style_index;

                if (prior_set)
                {
                    style_count++;
                    if (prior_biased_style_index != biased_style_index)
                    {
                        prc_error(ctx, PRC_ERROR_PARSE, "Tessellation %d has multiple styles in product tree\n", tess_index);
                    }
                }
                else
                {
                    /* This is the first time we have found this tessellation. Set the style index and continue on */
                    prior_biased_style_index = biased_style_index;
                    prior_set = true;
                    style_count = 1;
                }
            }
        }
    }
    return style_count;
}

/* Gets number of unique styles for that tessellation */
PRC_EXPORT uint32_t
prc_api_get_number_styles_for_tessellation(prc_context *ctx, prc_api_data data_in,
    uint32_t file_index)
{
    prc_data *data = (prc_data *)data_in;

    if (file_index >= data->file_structure_count)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "File index out of range in prc_api_get_number_tessellations\n");
        return 0;
    }

    return data->file_struct[file_index].tessellation->tess_count;
}

static int
prc_api_helper_get_tess_and_file_index(prc_context *ctx, prc_api_data data_in,
    uint32_t tess_index, uint32_t *file_index_out, uint32_t *tess_index_out)
{
    prc_data *data = (prc_data *)data_in;
    tess_style_file_part *part_details = data->part_details;
    tess_style_file_markup *markup_details = data->markup_details;

    if (tess_index >= data->unique_part_count + data->unique_markup_count)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "Tessellation index out of range in prc_api_helper_get_tess_and_file_index\n");
        return PRC_API_ERROR_PARAMETER;
    }
    if (tess_index >= data->unique_part_count)
    {
        /* This is a markup tessellation. They occur at the end */
        tess_index = tess_index - data->unique_part_count;
        if (tess_index > data->unique_markup_count - 1)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER, "Tessellation index out of range in prc_api_helper_get_tess_and_file_index\n");
            return PRC_API_ERROR_PARAMETER;
        }
        *file_index_out = markup_details[tess_index].tess_file_index;
        *tess_index_out = markup_details[tess_index].tessellation_index;
    }
    else
    {
        /* This is a part tessellation */
        *file_index_out = part_details[tess_index].tess_file_index;
        *tess_index_out = part_details[tess_index].tessellation_index;
    }
    return 0;
}

/* Tess index is the index into either part_details or markup_details in prc_data
   This populates the tessellation with the needed style information and sets
   the api_tess to its type */
PRC_EXPORT int
prc_api_initialize_tessellation(prc_context *ctx, prc_api_data data_in,
                                prc_api_product *model_tree, uint32_t tess_index,
                                prc_api_tess *api_tess, prc_api_tess *api_tess_line,
                                uint8_t *has_line)
{
    prc_data *data = (prc_data*)data_in;
    int code;
    uint32_t file_index = 0;
    uint32_t tessellation_index = 0;
    prc_tess *tessellation;
    tess_style_file_markup *markup_detail = NULL;
    tess_style_file_part *part_detail = NULL;
    prc_api_child_reserve *reserve;

    *has_line = 0;

    if (model_tree == NULL)
        return PRC_API_ERROR;

    reserve = (prc_api_child_reserve *)model_tree->reserved;

    code = prc_api_helper_get_tess_and_file_index(ctx, data_in, tess_index,
                                                  &file_index, &tessellation_index);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_get_tess_and_file_index\n");
        return 0;
    }

    if (tess_index >= data->unique_part_count)
    {
        /* This means this is a markup tessellation. We need to adjust the
           index to look in the markup details and not the part details */
        tess_index = tess_index - data->unique_part_count;
        if (tess_index >= data->unique_markup_count)
        {
            prc_error(ctx, PRC_API_INDEX, "Tessellation index out of range in prc_api_initialize_tessellation\n");
            return 0;
        }
        markup_detail = &data->markup_details[tess_index];
    }
    else
    {
        part_detail = &data->part_details[tess_index];
    }

    api_tess->tess_vertices.num_vertices = 0;
    api_tess->tess_vertices.vertices = NULL;
    api_tess->tess_vertices.capacity = 0;
    api_tess->num_line_primitives = 0;
    api_tess->num_text_primitives = 0;
    api_tess->name = NULL;
    api_tess->tess_index = tess_index;
    api_tess->part_index = -1;
    api_tess->product_index = -1;
    api_tess->mark_up_index = -1;
    api_tess->reserved = NULL;
    api_tess->reserved2 = NULL;
    api_tess->text_primitives = NULL;

    tessellation = &data->file_struct[file_index].tessellation->tess[tessellation_index];

    switch (tessellation->tess_type)
    {
        case PRC_TYPE_TESS_3D:
            api_tess->type = PRC_API_TESS_3D;
            prc_tess_3d *tess_3d = tessellation->tess_3d;
            if (tess_3d->number_of_wire_indices > 0)
            {
                *has_line = 1;
            }
            break;
        case PRC_TYPE_TESS_3D_Compressed:
            api_tess->type = PRC_API_TESS_3D_Compressed;
            prc_tess_3d_compressed *tess_3d_compressed = tessellation->tess_3d_compressed;
            if (tess_3d_compressed->number_of_edges > 0)
            {
                *has_line = 1;
            }
            break;
        case PRC_TYPE_TESS_3D_Wire:
            api_tess->type = PRC_API_TESS_3D_Wire;
            break;
        case PRC_TYPE_TESS_MarkUp:
            api_tess->type = PRC_API_TESS_MarkUp;
            break;
        default:
            api_tess->type = PRC_API_TESS_UNKNOWN;
            return 0;
    }

    /* We are going to set the default style to what ever this particular
       case is (as we are stepping through the unique cases that encompass
       tessellations and styles). That will create the vertices for the parts
       associated with it and we will assign it to those parts later. */
    if (part_detail != NULL)
    {
        api_tess->style_leaf = part_detail->style_leaf;
        tessellation->biased_style_index = part_detail->style_biased_index;

        if (part_detail->has_entity_ref)
        {
            tessellation->biased_style_index = part_detail->entity_ref.biased_index_of_line_style;
            tessellation->style_file_index = part_detail->entity_ref.file_index;
        }

        if (part_detail->has_inherited_color)
        {
            tessellation->has_inherited_color = 1;
            tessellation->inherited_color_index = part_detail->inherited_color_index;
            tessellation->inherited_color_file_index = part_detail->inherited_color_file_index;
        }
        else
        {
            tessellation->has_inherited_color = 0;
        }

        if (part_detail->has_inherited_trans)
        {
            tessellation->inherited_trans_index = part_detail->inherited_trans_index;
            tessellation->inherited_trans_file_index = part_detail->inherited_trans_file_index;
            tessellation->has_inherited_trans = 1;
        }
        else
        {
            tessellation->has_inherited_trans = 0;
        }

        /* Associate this api_tess with the part now */
        reserve->parts[part_detail->part_reserve_index].tess = api_tess;

        if (*has_line && api_tess_line != NULL)
        {
            api_tess_line->tess_vertices.num_vertices = 0;
            api_tess_line->tess_vertices.vertices = NULL;
            api_tess_line->tess_vertices.capacity = 0;
            api_tess_line->num_line_primitives = 0;
            api_tess_line->num_text_primitives = 0;
            api_tess_line->name = NULL;
            api_tess_line->tess_index = tess_index;
            api_tess_line->part_index = -1;
            api_tess_line->product_index = -1;
            api_tess_line->mark_up_index = -1;
            api_tess_line->reserved = NULL;
            api_tess_line->reserved2 = NULL;
            api_tess_line->text_primitives = NULL;
            reserve->parts[part_detail->part_reserve_index].tess_line = api_tess_line;
        }
    }
    else
    {
        /* Markup case */
        tessellation->biased_style_index = markup_detail->style_biased_index;
        tessellation->style_file_index = markup_detail->tess_file_index;
        tessellation->has_inherited_trans = 0;
        tessellation->has_inherited_color = 0;
        /* Associate this api_tess with the markup now */
        reserve->markups[markup_detail->markup_reserve_index].tess = api_tess;
    }
    return 0;
}

/* Get the type of tessellation from the parsed PRC data */
PRC_EXPORT prc_api_tess_type_t
prc_api_get_tessellation_type(prc_context *ctx, prc_api_data data, uint32_t tess_index)
{
    prc_data *data_in = (prc_data *)data;
    uint32_t file_index;
    uint32_t tessellation_index;
    int code;

    code = prc_api_helper_get_tess_and_file_index(ctx, data_in, tess_index,
                                                  &file_index, &tessellation_index);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_get_tess_and_file_index\n");
        return PRC_API_TESS_UNKNOWN;
    }

    switch (data_in->file_struct[file_index].tessellation->tess[tessellation_index].tess_type)
    {
        case PRC_TYPE_TESS_3D:
            return PRC_API_TESS_3D;
        case PRC_TYPE_TESS_3D_Compressed:
            return PRC_API_TESS_3D_Compressed;
        case PRC_TYPE_TESS_3D_Wire:
            return PRC_API_TESS_3D_Wire;
        case PRC_TYPE_TESS_MarkUp:
            return PRC_API_TESS_MarkUp;
        default:
            return PRC_API_TESS_UNKNOWN;
    }
}

/* The tess index in this case is an index into either part_details
   or markup details */
PRC_EXPORT uint32_t
prc_api_get_number_faces(prc_context *ctx, prc_api_data data_in,
    uint32_t tess_index)
{
    prc_data *data = (prc_data *)data_in;
    tess_style_file_part *part_details = data->part_details;
    tess_style_file_markup *markup_details = data->markup_details;
    uint32_t file_index;
    prc_filestructure *files = data->file_struct;
    prc_tess *tessellation;
    uint32_t tessellation_index; /* Tess index in the file */
    int code;

    code = prc_api_helper_get_tess_and_file_index(ctx, data_in, tess_index,
                                                  &file_index, &tessellation_index);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_api_helper_get_tess_and_file_index\n");
        return 0;
    }

    /* Get the tessellation */
    tessellation = &files[file_index].tessellation->tess[tessellation_index];

    switch (tessellation->tess_type)
    {
    case PRC_TYPE_TESS_3D:
        return tessellation->tess_3d->number_of_face_tessellation;
    case PRC_TYPE_TESS_3D_Compressed:
        /* 3D Compressed parts are all triangles and while they can have faces
           with different styles, we will treat this as a single face as the
           faces as they are indexed in the compressed structure can be scattered
           about. Really the face index is only used to express a style that those
           triangles have.

           Gated on is_multiple_line_attribute (real per-face/per-triangle
           style variation), NOT merely line_attribute_array != NULL: every
           compressed tessellation carries a line_attribute_array (Table 174
           requires it be read unconditionally), including a single no-style
           placeholder entry when there is no real per-face variation at all
           -- checking mere non-NULLness here collapsed every compressed
           tessellation's face count to 1 once nanoPRC's own write facility
           started always emitting that placeholder entry (previously it
           only ever emitted one for the supplied-normals/C2 path, so this
           branch was accidentally never exercised by nanoPRC's own C1
           output). */
        if (tessellation->tess_3d_compressed->is_multiple_line_attribute ||
            (tessellation->tess_3d_compressed->decoded_point_color_array != NULL))
        {
            /* We may need to look at the content of the line_attribute array
               and see that they are all the same */
			return 1;
		}
		else
		{
			return tessellation->tess_3d_compressed->face_number;
		}
    case PRC_TYPE_TESS_Face:
        return PRC_API_ERROR_PARSER;
    case PRC_TYPE_TESS_3D_Wire:
        return 0;
    case PRC_TYPE_TESS_MarkUp:
        return 0;
    default:
        return PRC_API_ERROR_PARSER;
    }
}

PRC_EXPORT uint8_t
prc_api_vertices_have_material(prc_context *ctx, const prc_api_tess *api_tess,
    uint32_t face_index)
{
    if (api_tess->type != PRC_API_TESS_3D_Compressed)
    {
        return 0;
    }

    return api_tess->tess_faces[face_index].vertices_have_style;
}

PRC_EXPORT uint8_t
prc_api_skip_face(prc_context *ctx, const prc_api_tess *api_tess,
    uint32_t face_index)
{
    if (api_tess->type == PRC_API_TESS_3D && face_index < api_tess->num_faces)
    {
        return api_tess->tess_faces[face_index].disable_face;
    }
    if (api_tess->type == PRC_API_TESS_3D_Compressed)
    {
        return api_tess->tess_faces[0].disable_face;
    }
    if (api_tess->type == PRC_API_TESS_3D_Wire_Extra && face_index < api_tess->num_faces)
    {
        return api_tess->tess_faces[face_index].disable_face;
    }
    return 0;
}

/* Will need to do this per face at some point */
PRC_EXPORT int
prc_api_face_is_material(prc_context *ctx, const prc_api_tess *api_tess,
                         uint32_t face_index)
{
    if (api_tess->type != PRC_API_TESS_3D)
    {
        if (api_tess->type == PRC_API_TESS_3D_Compressed)
        {
            if (api_tess->tess_faces[face_index].face_has_single_style &&
                api_tess->is_material)
            {
                return 1;
            }
        }
        else
        {
            return 0;
        }
    }
    else
    {
        if (api_tess->num_faces == 1 && api_tess->is_material)
        {
            return 1;
        }
        if (api_tess->num_faces > 1)
        {
            if (face_index > api_tess->num_faces - 1)
            {
                return PRC_API_ERROR_PARAMETER;
            }
            else
            {
                if (api_tess->tess_faces[face_index].is_material)
                {
                    return 1;
                }
            }
        }
    }
    return 0;
}

PRC_EXPORT void
prc_api_get_face_material(prc_context *ctx, const prc_api_tess *api_tess,
    prc_api_material *material, uint32_t face_index)
{
    if (api_tess->type == PRC_API_TESS_3D)
    {
        if (api_tess->num_faces == 0)
        {
            return;
        }

        prc_internal_api_face *face_reserved =
            prc_face_internal_face(&api_tess->tess_faces[face_index]);
        if (face_reserved == NULL)
        {
            return;
        }

        material->ambient_alpha = (float) face_reserved->style->ambient_alpha;
        material->emissive_alpha = (float) face_reserved->style->emissive_alpha;
        material->diffuse_alpha = (float) face_reserved->style->diffuse_alpha;
        material->specular_alpha = (float)  face_reserved->style->specular_alpha;

        material->ambient[0] = (float)face_reserved->style->ambient_color[0];
        material->ambient[1] = (float)face_reserved->style->ambient_color[1];
        material->ambient[2] = (float)face_reserved->style->ambient_color[2];

        material->emissive[0] = (float)face_reserved->style->emissive_color[0];
        material->emissive[1] = (float)face_reserved->style->emissive_color[1];
        material->emissive[2] = (float)face_reserved->style->emissive_color[2];

        material->diffuse[0] = (float)face_reserved->style->diffuse_color[0];
        material->diffuse[1] = (float)face_reserved->style->diffuse_color[1];
        material->diffuse[2] = (float)face_reserved->style->diffuse_color[2];

        material->specular[0] = (float)face_reserved->style->specular_color[0];
        material->specular[1] = (float)face_reserved->style->specular_color[1];
        material->specular[2] = (float)face_reserved->style->specular_color[2];

        material->shininess = (float)face_reserved->style->shininess;

    }
    else if (api_tess->type == PRC_API_TESS_3D_Wire)
    {
        memccpy(material, &api_tess->tess_material, 1, sizeof(prc_api_material));
    }
    else if (api_tess->type == PRC_API_TESS_3D_Compressed &&
        api_tess->tess_faces->face_has_single_style && api_tess->is_material)
    {
        prc_api_material *material_in = &api_tess->tess_faces[0].material;
        material->ambient_alpha = material_in->ambient_alpha;
        material->diffuse_alpha = material_in->diffuse_alpha;
        material->specular_alpha = material_in->specular_alpha;
        material->shininess = material_in->shininess;
        material->ambient[0] = material_in->ambient[0];
        material->ambient[1] = material_in->ambient[1];
        material->ambient[2] = material_in->ambient[2];
        material->emissive[0] = material_in->emissive[0];
        material->emissive[1] = material_in->emissive[1];
        material->emissive[2] = material_in->emissive[2];
        material->diffuse[0] = material_in->diffuse[0];
        material->diffuse[1] = material_in->diffuse[1];
        material->diffuse[2] = material_in->diffuse[2];
        material->specular[0] = material_in->specular[0];
        material->specular[1] = material_in->specular[1];
        material->specular[2] = material_in->specular[2];
        material->ambient_alpha = material_in->ambient_alpha;
        material->diffuse_alpha = material_in->diffuse_alpha;
        material->specular_alpha = material_in->specular_alpha;
        material->shininess = material_in->shininess;
    }
    else
    {
        /* For Markup and other types, there is no material to get. */
        memset(material, 0, sizeof(prc_api_material));
        material->ambient_alpha = 1.0f; /* Default values */
        material->diffuse_alpha = 1.0f;
        material->specular_alpha = 1.0f;
        material->shininess = 0.0f;
    }
}
