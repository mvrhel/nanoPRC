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
#define DEBUG_TEXTURES 0

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../include/prc_api.h"
#include "prc_internal_api.h"
#include "prc_internal_proto_api.h"
#include "prc_tri_primitives_helper_api.h"
#include "debug.h"

#if DEBUG_TEXTURES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#endif

#define DEBUG_NORM_VERT_PAIR 0
// Uncompressed tess debug
#define DEBUG_TEXTURE_COORDS 1
#define DEBUG_TARGET_TESS_INDEX 6
#define DEBUG_TARGET_FACE_INDEX 0

/* Targeted compressed tess index mapping debug.
    Prints only when the remapped API index matches one of the watched values. */
#define DEBUG_COMPRESSED_API_INDEX_MAP 0
#define DEBUG_COMPRESSED_API_INDEX_A 2392u
#define DEBUG_COMPRESSED_API_INDEX_B 854u
#define DEBUG_COMPRESSED_API_INDEX_C 2393u

uint32_t
prc_api_helper_get_biased_file_index_from_unique_id(prc_context *ctx, prc_api_data data,
    prc_unique_id unique_id);

/* Hashed edge lookup so that prc_internal_api_build_edge_list is O(n) instead of
   O(n^2) for large meshes. Mirrors prc_compressed_tess_hash_edge_key/
   prc_compressed_tess_find_open_edge_hashed in prc_decode_compressed_tess.c. */
static uint32_t
prc_internal_api_hash_edge_key(uint32_t index1, uint32_t index2, uint32_t mask)
{
    uint32_t min_index = (index1 < index2) ? index1 : index2;
    uint32_t max_index = (index1 < index2) ? index2 : index1;
    uint32_t hash = (min_index * 73856093u) ^ (max_index * 19349663u);
    return hash & mask;
}

static void
prc_internal_api_hash_insert_edge(uint32_t *hash_heads, uint32_t *hash_next,
    uint32_t hash_mask, uint32_t index1, uint32_t index2, uint32_t edge_index)
{
    uint32_t bucket = prc_internal_api_hash_edge_key(index1, index2, hash_mask);
    hash_next[edge_index] = hash_heads[bucket];
    hash_heads[bucket] = edge_index;
}

/* Find an unmatched edge candidate (num_triangles == 1) in the hashed edge buckets.
   We only search the triangle one indices obviously */
static prc_internal_api_edge*
prc_internal_api_find_open_edge_hashed(prc_internal_api_edge_list *edge_list,
    uint32_t *hash_heads, uint32_t *hash_next, uint32_t hash_mask,
    uint32_t index1, uint32_t index2)
{
    uint32_t bucket = prc_internal_api_hash_edge_key(index1, index2, hash_mask);
    uint32_t edge_index = hash_heads[bucket];

    while (edge_index != UINT32_MAX)
    {
        prc_internal_api_edge *edge = &edge_list->edge[edge_index];

        if (edge->num_triangles != 2)
        {
            if ((edge->tri_one_edge_indices[0] == index1 &&
                 edge->tri_one_edge_indices[1] == index2) ||
                (edge->tri_one_edge_indices[0] == index2 &&
                 edge->tri_one_edge_indices[1] == index1))
            {
                return edge;
            }
        }

        edge_index = hash_next[edge_index];
    }
    return NULL;
}

/* Another helper function for the edge list creation */
static void
prc_internal_api_set_edge(prc_internal_api_edge_list *edge_list,
    prc_internal_api_edge *edge, uint32_t *indices,
    prc_internal_api_edge_check_case_t edge_case, uint32_t indices_offset)
{
    if (edge == NULL)
    {
        /* Grab the next one in our capacity */
        uint32_t num_edges = edge_list->num_edges;
        edge = &edge_list->edge[num_edges];

        edge->tri_one_full_indices[0] = indices[0];
        edge->tri_one_full_indices[1] = indices[1];
        edge->tri_one_full_indices[2] = indices[2];

        edge->tri_one_vertex_indices_offset = indices_offset;

        edge->tri_one_edge_case = edge_case;

        switch (edge_case)
        {
            case PRC_INTERNAL_API_EDGE_01:
                edge->tri_one_edge_indices[0] = indices[0];
                edge->tri_one_edge_indices[1] = indices[1];
                break;

            case PRC_INTERNAL_API_EDGE_02:
                edge->tri_one_edge_indices[0] = indices[0];
                edge->tri_one_edge_indices[1] = indices[2];
                break;

            case PRC_INTERNAL_API_EDGE_12:
                edge->tri_one_edge_indices[0] = indices[1];
                edge->tri_one_edge_indices[1] = indices[2];
                break;
        }
        edge->num_triangles = 1;
        edge_list->num_edges++;
    }
    else
    {
        edge->tri_two_full_indices[0] = indices[0];
        edge->tri_two_full_indices[1] = indices[1];
        edge->tri_two_full_indices[2] = indices[2];

        edge->tri_two_vertex_indices_offset = indices_offset;

        edge->tri_two_edge_case = edge_case;

        switch (edge_case)
        {
            case PRC_INTERNAL_API_EDGE_01:
                edge->tri_two_edge_indices[0] = indices[0];
                edge->tri_two_edge_indices[1] = indices[1];
                break;
            case PRC_INTERNAL_API_EDGE_02:
                edge->tri_two_edge_indices[0] = indices[0];
                edge->tri_two_edge_indices[1] = indices[2];
                break;
            case PRC_INTERNAL_API_EDGE_12:
                edge->tri_two_edge_indices[0] = indices[1];
                edge->tri_two_edge_indices[1] = indices[2];
                break;
        }
        edge->num_triangles = 2;
    }
}

void
prc_internal_api_process_vertex_color_data(prc_context *ctx, uint32_t num_vertices,
    prc_color_data *color_data, uint8_t has_alpha, float *decoded_colors)
{
    uint32_t k;
    uint32_t array_offset;

    for (k = 0; k < num_vertices; k++)
    {
        if (k == 0)
        {
            /* First one is encoded special */
            decoded_colors[0] = color_data->first_vertex.red / 255.0f;
            decoded_colors[1] = color_data->first_vertex.green / 255.0f;
            decoded_colors[2] = color_data->first_vertex.blue / 255.0f;

            if (has_alpha)
            {
                decoded_colors[3] = color_data->first_vertex.alpha;
            }
            else
            {
                decoded_colors[3] = 1.0;
            }
        }
        else
        {
            /* We already applied the "same as" and copied the color when we read the array */
            array_offset = k * 4;
            decoded_colors[array_offset] = color_data->remaining_vertices[k - 1].color.red / 255.0f;
            decoded_colors[array_offset + 1] = color_data->remaining_vertices[k - 1].color.green / 255.0;
            decoded_colors[array_offset + 2] = color_data->remaining_vertices[k - 1].color.blue / 255.0;
            if (has_alpha)
            {
                decoded_colors[array_offset + 3] = color_data->remaining_vertices[k - 1].color.alpha;
            }
            else
            {
                decoded_colors[array_offset + 3] = 1.0;
            }
        }
    }
}

/* In this function we build the edge list from the triangle list. The edge list
   consists of a list of edges. The members of each edge will be the list of vertices
   for each of the two triangles that share that edge. We use the edge list to step
   through the tessellation and compute the normals.  Making use of the crease angle
   to determine if we need to split the vertices

   IMPORTANT: This function must be called AFTER the vertex indices have been
   converted from PRC position indices to API vertex indices. The indices in
   face_out_reserved->vertex_indices must already be valid API vertex indices
   that reference entries in vertex_out->vertices.
*/
int
prc_internal_api_build_edge_list(prc_context *ctx, uint32_t num_triangles,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    uint32_t k;
    uint32_t indices[3];
    prc_internal_api_edge *edge;
    prc_internal_api_edge_list *edge_list;
    uint32_t indices_offset; /* Offset into the indices array for the triangle */
    uint32_t *hash_heads = NULL;
    uint32_t *hash_next = NULL;
    uint32_t hash_capacity = 1;
    uint32_t hash_mask;

    /* indices_offset is needed so that when a vertex split occurs we can find
       where in the indices the triangle occurred and update its indice so that
       it points to the new vertex that we had to create */

       /* We could be in a case where we have triangles of different types in the
          list.  (e.g. multi-norm and single_norm. If so, we will need to create
          different edge lists for each of the possible cases. This is a safety
          check for that condition */
    if (uncompressed_data->edge_list != NULL)
        return PRC_API_ERROR_UNSUPPORTED;

    edge_list = (prc_internal_api_edge_list *)prc_calloc(ctx, 1,
        sizeof(prc_internal_api_edge_list));
    if (edge_list == NULL)
        return PRC_API_ERROR_MEMORY;

    /* We will definitely need less than this */
    edge_list->capacity = num_triangles * 3;
    edge_list->num_edges = 0;
    edge_list->edge = (prc_internal_api_edge *)prc_calloc(ctx,
        edge_list->capacity, sizeof(prc_internal_api_edge));
    if (edge_list->edge == NULL)
    {
        prc_free(ctx, edge_list);
        return PRC_API_ERROR_MEMORY;
    }

    /* Hash table over edge keys so edge lookups below are O(1) average instead
       of an O(num_edges) linear scan (which made this function O(num_triangles^2)
       on large meshes). hash_heads[bucket] is the index of the most recently
       inserted edge in that bucket, chained via hash_next[]. */
    while (hash_capacity < (num_triangles * 6 + 1))
    {
        hash_capacity <<= 1;
        if (hash_capacity == 0)
        {
            hash_capacity = 1;
            break;
        }
    }
    hash_mask = hash_capacity - 1;

    hash_heads = (uint32_t *)prc_calloc(ctx, hash_capacity, sizeof(uint32_t));
    hash_next = (uint32_t *)prc_calloc(ctx, edge_list->capacity, sizeof(uint32_t));
    if (hash_heads == NULL || hash_next == NULL)
    {
        prc_free(ctx, hash_heads);
        prc_free(ctx, hash_next);
        prc_free(ctx, edge_list->edge);
        prc_free(ctx, edge_list);
        return PRC_API_ERROR_MEMORY;
    }
    for (k = 0; k < hash_capacity; k++)
        hash_heads[k] = UINT32_MAX;

#if 0
    /* Verify that all indices are within the valid API vertex range.
       This catches cases where the edge list is being built with PRC indices instead of
       API indices. */
    printf("Debug: build_edge_list - Validating %u triangles, vertex_out->num_vertices=%zu\n",
        num_triangles, uncompressed_data->vertex_out->num_vertices);

    for (k = 0; k < num_triangles; k++)
    {
        uint32_t idx0 = uncompressed_data->face_out_reserved->vertex_indices[k * 3];
        uint32_t idx1 = uncompressed_data->face_out_reserved->vertex_indices[k * 3 + 1];
        uint32_t idx2 = uncompressed_data->face_out_reserved->vertex_indices[k * 3 + 2];

        if (idx0 >= uncompressed_data->vertex_out->num_vertices ||
            idx1 >= uncompressed_data->vertex_out->num_vertices ||
            idx2 >= uncompressed_data->vertex_out->num_vertices)
        {
            printf("ERROR: build_edge_list called with invalid indices!\n");
            printf("       Triangle %u has indices [%u,%u,%u], max valid=%zu\n",
                k, idx0, idx1, idx2, uncompressed_data->vertex_out->num_vertices);
            printf("       This indicates edge list is being built BEFORE vertex processing.\n");
            printf("       Edge list must be built AFTER indices are converted to API indices.\n");

            prc_free(ctx, edge_list->edge);
            prc_free(ctx, edge_list);
            return PRC_API_ERROR_PARSER;
        }
    }
    printf("Debug: build_edge_list - All indices validated successfully\n");

#endif

    for (k = 0; k < num_triangles; k++)
    {
        /* Grab the three indices for the triangle - these MUST be valid API vertex indices */
        indices[0] = uncompressed_data->face_out_reserved->vertex_indices[k * 3];
        indices[1] = uncompressed_data->face_out_reserved->vertex_indices[k * 3 + 1];
        indices[2] = uncompressed_data->face_out_reserved->vertex_indices[k * 3 + 2];

        indices_offset = k * 3;

        /* Now search through the hashed edge buckets to see if any of these
           edges are already present */
        /* Edge 0-1 */
        edge = prc_internal_api_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[0], indices[1]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            prc_internal_api_set_edge(edge_list, NULL, indices, PRC_INTERNAL_API_EDGE_01, indices_offset);
            prc_internal_api_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[0], indices[1], new_edge_index);
        }
        else
        {
            prc_internal_api_set_edge(edge_list, edge, indices, PRC_INTERNAL_API_EDGE_01, indices_offset);
        }

        /* Edge 0-2 */
        edge = prc_internal_api_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[0], indices[2]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            prc_internal_api_set_edge(edge_list, NULL, indices, PRC_INTERNAL_API_EDGE_02, indices_offset);
            prc_internal_api_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[0], indices[2], new_edge_index);
        }
        else
        {
            prc_internal_api_set_edge(edge_list, edge, indices, PRC_INTERNAL_API_EDGE_02, indices_offset);
        }

        /* Edge 1-2 */
        edge = prc_internal_api_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[1], indices[2]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            prc_internal_api_set_edge(edge_list, NULL, indices, PRC_INTERNAL_API_EDGE_12, indices_offset);
            prc_internal_api_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[1], indices[2], new_edge_index);
        }
        else
        {
            prc_internal_api_set_edge(edge_list, edge, indices, PRC_INTERNAL_API_EDGE_12, indices_offset);
        }
    }
    prc_free(ctx, hash_heads);
    prc_free(ctx, hash_next);
    uncompressed_data->edge_list = edge_list;
    return 0;
}

static int
prc_internal_api_compute_normal(prc_api_vertex *vertices, prc_vec3 *normal)
{
    /* Compute triangle normal using edge cross product:
       normal = normalize( (v1 - v0) x (v2 - v0) ).
       If normals appear inverted for your mesh, swap the cross arguments
       (i.e. cross(edge1, edge0)) to flip the sign. */
    prc_vec3 edge0, edge1;
    int code = 0;

    edge0.x = vertices[1].position[0] - vertices[0].position[0];
    edge0.y = vertices[1].position[1] - vertices[0].position[1];
    edge0.z = vertices[1].position[2] - vertices[0].position[2];

    edge1.x = vertices[2].position[0] - vertices[0].position[0];
    edge1.y = vertices[2].position[1] - vertices[0].position[1];
    edge1.z = vertices[2].position[2] - vertices[0].position[2];

    prc_vec_cross(edge0, edge1, normal);

    code = prc_vec_normalize(normal);

    return code;
}

static void
prc_internal_api_get_normals_state(prc_context *ctx, prc_internal_api_edge *edge,
                                  prc_api_vertex *vertices, uint8_t *normal0_set,
                                   uint8_t *normal1_set, uint8_t triangle_index)
{
    prc_internal_api_edge_check_case_t edge_case;
    *normal0_set = false;
    *normal1_set = false;

    if (triangle_index == 1)
    {
        edge_case = edge->tri_one_edge_case;
    }
    else
    {
        edge_case = edge->tri_two_edge_case;
    }

    if (edge_case == PRC_INTERNAL_API_EDGE_01)
    {
        if (vertices[0].normal_set)
            *normal0_set = true;
        if (vertices[1].normal_set)
            *normal1_set = true;
    }
    else if (edge_case == PRC_INTERNAL_API_EDGE_02)
    {
        if (vertices[0].normal_set)
            *normal0_set = true;
        if (vertices[2].normal_set)
            *normal1_set = true;
    }
    else if (edge_case == PRC_INTERNAL_API_EDGE_12)
    {
        if (vertices[1].normal_set)
            *normal0_set = true;
        if (vertices[2].normal_set)
            *normal1_set = true;
    }
}

static uint8_t
prc_internal_api_angle_less_than_crease_angle(prc_context *ctx, prc_vec3 normal1,
                                              prc_vec3 normal2, double crease_angle)
{
    int code;
    double angle;

    code = prc_vec_angle_between_vectors(normal1, normal2, &angle);
    if (code < 0)
        return false;

    if (angle < crease_angle)
        return true;

    return false;
}

static void
prc_internal_api_get_existing_normal(prc_context *ctx, prc_internal_api_edge *edge,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t edge_index, uint32_t triangle_index,
    prc_vec3 *normal)
{
    if (triangle_index == 1)
    {
        normal->x = vertex_out->vertices[edge->tri_one_edge_indices[edge_index]].normal[0];
        normal->y = vertex_out->vertices[edge->tri_one_edge_indices[edge_index]].normal[1];
        normal->z = vertex_out->vertices[edge->tri_one_edge_indices[edge_index]].normal[2];
    }
    else
    {
        normal->x = vertex_out->vertices[edge->tri_two_edge_indices[edge_index]].normal[0];
        normal->y = vertex_out->vertices[edge->tri_two_edge_indices[edge_index]].normal[1];
        normal->z = vertex_out->vertices[edge->tri_two_edge_indices[edge_index]].normal[2];
    }
}

static void
prc_internal_api_get_edge_data(prc_context *ctx, prc_internal_api_edge *edge,
    uint32_t *indices, prc_api_vertex *vertices, prc_api_tess_vertex_buffer *vertex_buff,
    uint32_t *vertex_indices_offset, prc_internal_api_edge_check_case_t *edge_case,
    uint8_t triangle_number)
{
    if (triangle_number == 1)
    {
        indices[0] = edge->tri_one_full_indices[0];
        indices[1] = edge->tri_one_full_indices[1];
        indices[2] = edge->tri_one_full_indices[2];
        vertices[0] = vertex_buff->vertices[indices[0]];
        vertices[1] = vertex_buff->vertices[indices[1]];
        vertices[2] = vertex_buff->vertices[indices[2]];
        *vertex_indices_offset = edge->tri_one_vertex_indices_offset;
        *edge_case = edge->tri_one_edge_case;
    }
    else
    {
        indices[0] = edge->tri_two_full_indices[0];
        indices[1] = edge->tri_two_full_indices[1];
        indices[2] = edge->tri_two_full_indices[2];
        vertices[0] = vertex_buff->vertices[indices[0]];
        vertices[1] = vertex_buff->vertices[indices[1]];
        vertices[2] = vertex_buff->vertices[indices[2]];
        *vertex_indices_offset = edge->tri_two_vertex_indices_offset;
        *edge_case = edge->tri_two_edge_case;
    }
}

/* Compact vertex buffer to only the vertices referenced by the face index buffer.
   - Allocates a new vertex array sized to the number of unique referenced indices.
   - Builds a remap table old_index -> new_index and copies referenced vertices once.
   - Updates face_out_reserved->vertex_indices in-place to the new indices.
   - Frees the old vertex array and updates vertex_out->num_vertices and capacity.
   Returns 0 on success or PRC_API_ERROR_MEMORY on allocation failure.
*/
static int
prc_internal_api_compact_vertices(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    prc_api_tess_vertex_buffer *vertex_out = uncompressed_data->vertex_out;
    prc_internal_api_face *face = uncompressed_data->face_out_reserved;
    if (!vertex_out || !face || face->vertex_indices == NULL || face->num_indices == 0)
        return 0; /* nothing to do */

    size_t old_nv = vertex_out->num_vertices;
    if (old_nv == 0)
        return 0;

    /* Mark seen vertices */
    uint8_t *seen = (uint8_t *)prc_calloc(ctx, old_nv, sizeof(uint8_t));
    if (seen == NULL)
        return PRC_API_ERROR_MEMORY;

    size_t unique_count = 0;
    for (size_t i = 0; i < face->num_indices; ++i)
    {
        uint32_t idx = face->vertex_indices[i];
        if (idx < old_nv && !seen[idx])
        {
            seen[idx] = 1;
            unique_count++;
        }
    }

    /* If all vertices are referenced, nothing to do */
    if (unique_count == old_nv)
    {
        prc_free(ctx, seen);
        return 0;
    }

    /* Allocate remap table and new vertex array */
    uint32_t *remap = (uint32_t *)prc_calloc(ctx, old_nv, sizeof(uint32_t));
    if (remap == NULL)
    {
        prc_free(ctx, seen);
        return PRC_API_ERROR_MEMORY;
    }
    for (size_t i = 0; i < old_nv; ++i) remap[i] = UINT32_MAX;

    prc_api_vertex *new_vertices = (prc_api_vertex *)prc_calloc(ctx, unique_count, sizeof(prc_api_vertex));
    if (new_vertices == NULL)
    {
        prc_free(ctx, seen);
        prc_free(ctx, remap);
        return PRC_API_ERROR_MEMORY;
    }

    /* Populate new_vertices in order of first appearance in index buffer */
    uint32_t next = 0;
    for (size_t i = 0; i < face->num_indices; ++i)
    {
        uint32_t old = face->vertex_indices[i];
        if (old >= old_nv)
            continue; /* defensive; should not happen */

        if (remap[old] == UINT32_MAX)
        {
            remap[old] = next;
            new_vertices[next] = vertex_out->vertices[old];
            next++;
        }
        /* Update index immediately */
        face->vertex_indices[i] = remap[old];
    }

    /* Replace old vertex array */
    prc_free(ctx, vertex_out->vertices);
    vertex_out->vertices = new_vertices;
    vertex_out->num_vertices = next;
    vertex_out->capacity = next;

    prc_free(ctx, seen);
    prc_free(ctx, remap);

    return 0;
}

/*
Reworked: prc_internal_api_split_vertex

Purpose:
- When splitting a single vertex we want to avoid reusing a duplicate that already
  has a normal assigned (that would force two triangles to share the same API
  vertex with different normals resulting in mismatched triangle vertex normals).
- Reuse the stored first-duplicate only if that duplicate exists AND its normal
  has not yet been set. Otherwise allocate a new duplicate.
- Uses vertex_to_original for O(1) lookup of original vertex index.
- All original index/edge bookkeeping updates are preserved.
*/
static int
prc_internal_api_split_vertex(prc_context *ctx, uint32_t *vertex_indices_list,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t *indices,
    prc_internal_api_edge *edge, uint32_t edge_index, uint32_t triangle_index,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    prc_api_vertex old_vertex;
    uint32_t indices_offset;
    size_t initial_count = uncompressed_data->position_normal_lut.number_values;
    uint32_t api_idx_to_duplicate;
    uint32_t orig_index;
    uint32_t reused_dup;

    if (triangle_index == 1)
    {
        indices_offset = edge->tri_one_vertex_indices_offset;
        api_idx_to_duplicate = edge->tri_one_edge_indices[edge_index];
    }
    else
    {
        indices_offset = edge->tri_two_vertex_indices_offset;
        api_idx_to_duplicate = edge->tri_two_edge_indices[edge_index];
    }

    /* Use vertex_to_original for O(1) lookup of the original vertex index */
    if (uncompressed_data->vertex_to_original == NULL ||
        api_idx_to_duplicate >= uncompressed_data->vertex_to_original_capacity)
    {
        return PRC_API_ERROR_PARSER;
    }

    orig_index = uncompressed_data->vertex_to_original[api_idx_to_duplicate];

    /* Sanity check: original must be within or equal to initial_count */
    if (orig_index >= initial_count)
    {
        printf("ERROR: orig_index %u > initial_count %zu\n", orig_index, initial_count);
        return PRC_API_ERROR_PARSER;
    }

    /* If a duplicate already exists for this original AND the duplicate does not yet
       have a normal assigned, reuse it. Otherwise allocate a fresh duplicate. */
    reused_dup = uncompressed_data->first_duplicate_for_original[orig_index];
    if (reused_dup != UINT32_MAX &&
        reused_dup < (uint32_t)vertex_out->num_vertices &&
        vertex_out->vertices[reused_dup].normal_set == false)
    {
        /* Update vertex indices and edge records according to the edge case */
        if (triangle_index == 1)
        {
            if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_01)
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 1] = reused_dup;
            }
            else if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_02)
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 2] = reused_dup;
            }
            else /* EDGE_12 */
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset + 1] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 2] = reused_dup;
            }

            /* Update edge bookkeeping */
            edge->split = true;
            edge->tri_one_edge_indices[edge_index] = reused_dup;
            if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_01)
            {
                if (edge_index == 0) edge->tri_one_full_indices[0] = reused_dup;
                else edge->tri_one_full_indices[1] = reused_dup;
            }
            else if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_02)
            {
                if (edge_index == 0) edge->tri_one_full_indices[0] = reused_dup;
                else edge->tri_one_full_indices[2] = reused_dup;
            }
            else
            {
                if (edge_index == 0) edge->tri_one_full_indices[1] = reused_dup;
                else edge->tri_one_full_indices[2] = reused_dup;
            }
        }
        else
        {
            if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_01)
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 1] = reused_dup;
            }
            else if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_02)
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 2] = reused_dup;
            }
            else /* EDGE_12 */
            {
                if (edge_index == 0)
                    vertex_indices_list[indices_offset + 1] = reused_dup;
                else
                    vertex_indices_list[indices_offset + 2] = reused_dup;
            }

            edge->split = true;
            edge->tri_two_edge_indices[edge_index] = reused_dup;
            if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_01)
            {
                if (edge_index == 0) edge->tri_two_full_indices[0] = reused_dup;
                else edge->tri_two_full_indices[1] = reused_dup;
            }
            else if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_02)
            {
                if (edge_index == 0) edge->tri_two_full_indices[0] = reused_dup;
                else edge->tri_two_full_indices[2] = reused_dup;
            }
            else
            {
                if (edge_index == 0) edge->tri_two_full_indices[1] = reused_dup;
                else edge->tri_two_full_indices[2] = reused_dup;
            }
        }
        return 0;
    }

    /* No suitable existing duplicate � proceed to allocate a new API vertex */
    if (triangle_index == 1)
    {
        old_vertex = vertex_out->vertices[edge->tri_one_edge_indices[edge_index]];
    }
    else
    {
        old_vertex = vertex_out->vertices[edge->tri_two_edge_indices[edge_index]];
    }

    /* Mark normal unset on duplicate */
    old_vertex.normal_set = false;

    /* Ensure capacity */
    if (vertex_out->num_vertices + 1 >= vertex_out->capacity)
    {
        prc_api_vertex *new_vertices;
        size_t new_capacity = vertex_out->capacity * 2;
        new_vertices = (prc_api_vertex *)prc_realloc(ctx,
            vertex_out->vertices, new_capacity * sizeof(prc_api_vertex));
        if (new_vertices == NULL)
            return PRC_API_ERROR_MEMORY;
        vertex_out->vertices = new_vertices;

        /* CRITICAL FIX: Also grow vertex_to_original to match */
        if (uncompressed_data->vertex_to_original != NULL)
        {
            uint32_t *new_map = (uint32_t *)prc_realloc(ctx,
                uncompressed_data->vertex_to_original, new_capacity * sizeof(uint32_t));
            if (new_map == NULL)
                return PRC_API_ERROR_MEMORY;
            uncompressed_data->vertex_to_original = new_map;
            uncompressed_data->vertex_to_original_capacity = new_capacity;
        }

        vertex_out->capacity = new_capacity;
        uncompressed_data->vertex_to_original_capacity = new_capacity;
        prc_internal_api_initialize_vertex(ctx, vertex_out);
    }

    /* Append new vertex */
    vertex_out->vertices[vertex_out->num_vertices] = old_vertex;

    /* Record the mapping from new vertex to original */
    uncompressed_data->vertex_to_original[vertex_out->num_vertices] = orig_index;

    vertex_out->num_vertices++;

    /* Record first duplicate for the original if none was recorded yet */
    if (uncompressed_data->first_duplicate_for_original[orig_index] == UINT32_MAX)
        uncompressed_data->first_duplicate_for_original[orig_index] = (uint32_t)(vertex_out->num_vertices - 1);

    /* Update indices and edge bookkeeping */
    if (triangle_index == 1)
    {
        if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_01)
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[0] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 1] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[1] = vertex_out->num_vertices - 1;
            }
        }
        else if (edge->tri_one_edge_case == PRC_INTERNAL_API_EDGE_02)
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[0] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 2] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[2] = vertex_out->num_vertices - 1;
            }
        }
        else /* EDGE_12 */
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset + 1] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[1] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 2] = vertex_out->num_vertices - 1;
                edge->tri_one_full_indices[2] = vertex_out->num_vertices - 1;
            }
        }
        edge->split = true;
        edge->tri_one_edge_indices[edge_index] = vertex_out->num_vertices - 1;
    }
    else
    {
        if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_01)
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[0] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 1] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[1] = vertex_out->num_vertices - 1;
            }
        }
        else if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_02)
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[0] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 2] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[2] = vertex_out->num_vertices - 1;
            }
        }
        else /* EDGE_12 */
        {
            if (edge_index == 0)
            {
                vertex_indices_list[indices_offset + 1] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[1] = vertex_out->num_vertices - 1;
            }
            else
            {
                vertex_indices_list[indices_offset + 2] = vertex_out->num_vertices - 1;
                edge->tri_two_full_indices[2] = vertex_out->num_vertices - 1;
            }
        }
        edge->split = true;
        edge->tri_two_edge_indices[edge_index] = vertex_out->num_vertices - 1;
    }
    return 0;
}

/*
Reworked: prc_internal_api_split_edge

Purpose:
- Split both vertices for triangle2's edge, but avoid reusing an existing
  duplicate that already has a normal assigned. Reuse only duplicates that
  exist and have normal_set == false.
- If a candidate duplicate is already initialized (normal_set == true),
  allocate a fresh duplicate to avoid mixing normals across triangles.
- Update vertex index list and edge bookkeeping accordingly.
- Uses vertex_to_original for O(1) lookup of original vertex index.
*/
static int
prc_internal_api_split_edge(prc_context *ctx, uint32_t *vertex_indices_list,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t *indices,
    prc_internal_api_edge *edge, prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    prc_api_vertex old_vertex1, old_vertex2;
    uint32_t indices_offset = edge->tri_two_vertex_indices_offset;
    size_t initial_count = uncompressed_data->position_normal_lut.number_values;
    uint32_t api0 = edge->tri_two_edge_indices[0];
    uint32_t api1 = edge->tri_two_edge_indices[1];
    uint32_t orig0, orig1;
    uint32_t dup0 = UINT32_MAX, dup1 = UINT32_MAX;

    /* Use vertex_to_original for O(1) lookup of the original vertex index.
       This handles both original vertices and any level of duplicates. */
    if (uncompressed_data->vertex_to_original == NULL)
        return PRC_API_ERROR_PARSER;

    if (api0 >= uncompressed_data->vertex_to_original_capacity ||
        api1 >= uncompressed_data->vertex_to_original_capacity)
        return PRC_API_ERROR_PARSER;

    orig0 = uncompressed_data->vertex_to_original[api0];
    orig1 = uncompressed_data->vertex_to_original[api1];

    /* Sanity check: originals must be within initial_count */
    if (orig0 >= initial_count || orig1 >= initial_count)
        return PRC_API_ERROR_PARSER;

    /* Candidate duplicates stored earlier */
    dup0 = uncompressed_data->first_duplicate_for_original[orig0];
    dup1 = uncompressed_data->first_duplicate_for_original[orig1];

    /* If both duplicates already exist and both are still un-initialized (no normal),
       simply update indices and bookkeeping. Otherwise allocate new ones as needed. */
    if (dup0 != UINT32_MAX && dup0 < (uint32_t)vertex_out->num_vertices &&
        dup1 != UINT32_MAX && dup1 < (uint32_t)vertex_out->num_vertices &&
        vertex_out->vertices[dup0].normal_set == false &&
        vertex_out->vertices[dup1].normal_set == false)
    {
        if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_01)
        {
            vertex_indices_list[indices_offset] = dup0;
            vertex_indices_list[indices_offset + 1] = dup1;
            edge->tri_two_full_indices[0] = dup0;
            edge->tri_two_full_indices[1] = dup1;
        }
        else if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_02)
        {
            vertex_indices_list[indices_offset] = dup0;
            vertex_indices_list[indices_offset + 2] = dup1;
            edge->tri_two_full_indices[0] = dup0;
            edge->tri_two_full_indices[2] = dup1;
        }
        else /* EDGE_12 */
        {
            vertex_indices_list[indices_offset + 1] = dup0;
            vertex_indices_list[indices_offset + 2] = dup1;
            edge->tri_two_full_indices[1] = dup0;
            edge->tri_two_full_indices[2] = dup1;
        }

        edge->split = true;
        edge->tri_two_edge_indices[0] = dup0;
        edge->tri_two_edge_indices[1] = dup1;
        return 0;
    }

    /* Ensure capacity for up to two new vertices */
    if (vertex_out->num_vertices + 2 > vertex_out->capacity)
    {
        prc_api_vertex *new_vertices;
        size_t new_capacity = vertex_out->capacity * 2;
        new_vertices = (prc_api_vertex *)prc_realloc(ctx,
            vertex_out->vertices, new_capacity * sizeof(prc_api_vertex));
        if (new_vertices == NULL)
            return PRC_API_ERROR_MEMORY;
        vertex_out->vertices = new_vertices;

        /* CRITICAL FIX: Also grow vertex_to_original to match */
        if (uncompressed_data->vertex_to_original != NULL)
        {
            uint32_t *new_map = (uint32_t *)prc_realloc(ctx,
                uncompressed_data->vertex_to_original, new_capacity * sizeof(uint32_t));
            if (new_map == NULL)
                return PRC_API_ERROR_MEMORY;
            uncompressed_data->vertex_to_original = new_map;
            uncompressed_data->vertex_to_original_capacity = new_capacity;

            /* Note: vertex_to_original is now per-face, no persistent tess storage needed */
        }

        vertex_out->capacity = new_capacity;
        prc_internal_api_initialize_vertex(ctx, vertex_out);
    }

    /* Duplicate first vertex if needed or if candidate exists but is already initialized */
    if (!(dup0 != UINT32_MAX && dup0 < (uint32_t)vertex_out->num_vertices && vertex_out->vertices[dup0].normal_set == false))
    {
        old_vertex1 = vertex_out->vertices[api0];
        old_vertex1.normal_set = false;
        vertex_out->vertices[vertex_out->num_vertices] = old_vertex1;
        dup0 = (uint32_t)vertex_out->num_vertices;

        /* Record the mapping from new vertex to original */
        uncompressed_data->vertex_to_original[dup0] = orig0;

        vertex_out->num_vertices++;
        /* Record mapping only if none existed before */
        if (uncompressed_data->first_duplicate_for_original[orig0] == UINT32_MAX)
            uncompressed_data->first_duplicate_for_original[orig0] = dup0;
    }

    /* Duplicate second vertex if needed or if candidate exists but is already initialized */
    if (!(dup1 != UINT32_MAX && dup1 < (uint32_t)vertex_out->num_vertices && vertex_out->vertices[dup1].normal_set == false))
    {
        old_vertex2 = vertex_out->vertices[api1];
        old_vertex2.normal_set = false;
        vertex_out->vertices[vertex_out->num_vertices] = old_vertex2;
        dup1 = (uint32_t)vertex_out->num_vertices;

        /* Record the mapping from new vertex to original */
        uncompressed_data->vertex_to_original[dup1] = orig1;

        vertex_out->num_vertices++;
        if (uncompressed_data->first_duplicate_for_original[orig1] == UINT32_MAX)
            uncompressed_data->first_duplicate_for_original[orig1] = dup1;
    }

    /* Update the vertex index list and tri_two_full_indices according to edge case */
    if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_01)
    {
        vertex_indices_list[indices_offset] = dup0;
        vertex_indices_list[indices_offset + 1] = dup1;
        edge->tri_two_full_indices[0] = dup0;
        edge->tri_two_full_indices[1] = dup1;
    }
    else if (edge->tri_two_edge_case == PRC_INTERNAL_API_EDGE_02)
    {
        vertex_indices_list[indices_offset] = dup0;
        vertex_indices_list[indices_offset + 2] = dup1;
        edge->tri_two_full_indices[0] = dup0;
        edge->tri_two_full_indices[2] = dup1;
    }
    else /* EDGE_12 */
    {
        vertex_indices_list[indices_offset + 1] = dup0;
        vertex_indices_list[indices_offset + 2] = dup1;
        edge->tri_two_full_indices[1] = dup0;
        edge->tri_two_full_indices[2] = dup1;
    }

    /* Update edge bookkeeping */
    edge->split = true;
    edge->tri_two_edge_indices[0] = dup0;
    edge->tri_two_edge_indices[1] = dup1;

    return 0;
}

static void
prc_internal_api_assign_normal_to_edge(prc_context *ctx,
                            prc_api_tess_vertex_buffer *vertex_out, prc_vec3 normal,
                            prc_internal_api_edge *edge, uint8_t triangle_number,
                            prc_internal_api_edge_normal_set_t edge_set_case)
{

    if (triangle_number == 1)
    {
        if (edge_set_case == PRC_INTERNAL_API_SET_BOTH_NORMALS_OF_EDGE ||
            edge_set_case == PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE)
        {
            vertex_out->vertices[edge->tri_one_edge_indices[0]].normal[0] = normal.x;
            vertex_out->vertices[edge->tri_one_edge_indices[0]].normal[1] = normal.y;
            vertex_out->vertices[edge->tri_one_edge_indices[0]].normal[2] = normal.z;
            vertex_out->vertices[edge->tri_one_edge_indices[0]].normal_set = true;
        }

        if (edge_set_case == PRC_INTERNAL_API_SET_BOTH_NORMALS_OF_EDGE ||
            edge_set_case == PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE)
        {
            vertex_out->vertices[edge->tri_one_edge_indices[1]].normal[0] = normal.x;
            vertex_out->vertices[edge->tri_one_edge_indices[1]].normal[1] = normal.y;
            vertex_out->vertices[edge->tri_one_edge_indices[1]].normal[2] = normal.z;
            vertex_out->vertices[edge->tri_one_edge_indices[1]].normal_set = true;
        }
    }
    else
    {
        if (edge_set_case == PRC_INTERNAL_API_SET_BOTH_NORMALS_OF_EDGE ||
            edge_set_case == PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE)
        {
            vertex_out->vertices[edge->tri_two_edge_indices[0]].normal[0] = normal.x;
            vertex_out->vertices[edge->tri_two_edge_indices[0]].normal[1] = normal.y;
            vertex_out->vertices[edge->tri_two_edge_indices[0]].normal[2] = normal.z;
            vertex_out->vertices[edge->tri_two_edge_indices[0]].normal_set = true;
        }
        if (edge_set_case == PRC_INTERNAL_API_SET_BOTH_NORMALS_OF_EDGE ||
            edge_set_case == PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE)
        {
            vertex_out->vertices[edge->tri_two_edge_indices[1]].normal[0] = normal.x;
            vertex_out->vertices[edge->tri_two_edge_indices[1]].normal[1] = normal.y;
            vertex_out->vertices[edge->tri_two_edge_indices[1]].normal[2] = normal.z;
            vertex_out->vertices[edge->tri_two_edge_indices[1]].normal_set = true;
        }
    }
}

/*
Reworked: prc_internal_api_handle_vertex_normal

Purpose:
- This wrapper was updated to accept uncompressed_data so that when a split is
  required it can call prc_internal_api_split_vertex with the mapping information.
- No algorithmic change beyond passing the uncompressed_data pointer.
*/
static int
prc_internal_api_handle_vertex_normal(prc_context *ctx, prc_internal_api_edge *edge,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t *vertex_indices_list,
    prc_vec3 new_normal, uint32_t *indices, double crease_angle, uint32_t edge_index,
    uint32_t triangle_index, prc_internal_api_edge_normal_set_t normal_set_case,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    prc_vec3 existing_normal;
    int code;

    prc_internal_api_get_existing_normal(ctx, edge, vertex_out, edge_index,
        triangle_index, &existing_normal);
    if (!prc_internal_api_angle_less_than_crease_angle(ctx, new_normal,
        existing_normal, crease_angle))
    {
        code = prc_internal_api_split_vertex(ctx, vertex_indices_list,
            vertex_out, indices, edge, edge_index, triangle_index,
            uncompressed_data);
        if (code < 0)
        {
            return code;
        }

        /* Assign new normal to the split vertex */
        prc_internal_api_assign_normal_to_edge(ctx, vertex_out, new_normal,
            edge, triangle_index, normal_set_case);
    }
    return 0;
}

/*
Reworked: prc_internal_api_vertex_edge_normal

Purpose:
- Accepts uncompressed_data to propagate the duplicate-mapping through to
  prc_internal_api_handle_vertex_normal / prc_internal_api_split_vertex.
- Behavior:
    - If the point normal is not set, assign directly.
    - Otherwise call handler that may split (and will now reuse first duplicates).
*/
static int
prc_internal_api_vertex_edge_normal(prc_context *ctx, prc_internal_api_edge *edge,
    prc_api_tess_vertex_buffer *vertex_out, uint32_t *vertex_indices_list,
    prc_vec3 new_normal, uint32_t *indices, double crease_angle, uint32_t edge_index,
    uint32_t triangle_index, uint8_t edge_point_normal_set,
    prc_internal_api_edge_normal_set_t normal_set_case,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    int code;

    /* If normal for point0 is not set, then do that now */
    if (!edge_point_normal_set)
    {
        /* Normal was not yet set. Go ahead and assign a normal */
        prc_internal_api_assign_normal_to_edge(ctx, vertex_out, new_normal,
            edge, triangle_index, normal_set_case);
    }
    else
    {
        code = prc_internal_api_handle_vertex_normal(ctx, edge,
            vertex_out, vertex_indices_list, new_normal, indices, crease_angle,
            edge_index, triangle_index, normal_set_case, uncompressed_data);
        if (code < 0)
        {
            return code;
        }
    }
    return 0;
}

/*
Reworked: prc_internal_api_compute_normals

Purpose:
- Allocate per-face local tracking arrays for vertex splitting (NOT persistent)
- vertex_to_original: Maps any vertex index back to its original position
- first_duplicate_for_original: Tracks first duplicate created for each original
- Use updated split/normal functions that accept uncompressed_data
- Free all local allocations at function end (no cross-face state)

Per-Face Design:
- Each face gets fresh allocations (no shared state between faces)
- Eliminates heap corruption from cross-face buffer reuse
- Simplifies memory management (no persistent pointers to track)
*/
int
prc_internal_api_compute_normals(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data)
{
    uint32_t k;
    uint32_t indices_tri1[3], indices_tri2[3];
    prc_internal_api_edge *edge;
    prc_api_tess_vertex_buffer *vertex_out = uncompressed_data->vertex_out;
    prc_internal_api_edge_list *edge_list = uncompressed_data->edge_list;
    prc_api_vertex vertices_tri1[3], vertices_tri2[3];
    prc_vec3 normal1, normal2, normal_avg, existing_normal;
    int code, code1, code2;
    uint8_t edge_point0_tri1_set, edge_point1_tri1_set;
    uint8_t edge_point0_tri2_set, edge_point1_tri2_set;
    double crease_angle = uncompressed_data->crease_angle;
    uint32_t *vertex_indices = uncompressed_data->vertex_indices;
    uint32_t vertex_indices_offset;
    prc_internal_api_edge_check_case_t edge_case;
    uint8_t edge_index, triangle_index;
    uint8_t normal_not_set;
    size_t num_pos = uncompressed_data->position_normal_lut.number_values;
    uint8_t had_failure = 0;

    /* Per-face scratch mappings must be either fully built or fully released. */
    uncompressed_data->first_duplicate_for_original = NULL;
    uncompressed_data->vertex_to_original = NULL;
    uncompressed_data->vertex_to_original_capacity = 0;

#if 0
    /* Check all edge indices are within bounds */
    for (k = 0; k < edge_list->num_edges; k++)
    {
        edge = &edge_list->edge[k];

        /* Validate triangle 1 indices */
        if (edge->tri_one_edge_indices[0] >= vertex_out->num_vertices ||
            edge->tri_one_edge_indices[1] >= vertex_out->num_vertices)
        {
            printf("ERROR: Edge %u has out-of-bounds tri_one indices: [%u,%u], max=%zu\n",
                k, edge->tri_one_edge_indices[0], edge->tri_one_edge_indices[1],
                vertex_out->num_vertices);
            printf("       This indicates the edge list was built with PRC indices instead of API indices\n");
            return PRC_API_ERROR_PARSER;
        }

        /* Validate triangle 2 indices if present */
        if (edge->num_triangles == 2)
        {
            if (edge->tri_two_edge_indices[0] >= vertex_out->num_vertices ||
                edge->tri_two_edge_indices[1] >= vertex_out->num_vertices)
            {
                printf("ERROR: Edge %u has out-of-bounds tri_two indices: [%u,%u], max=%zu\n",
                    k, edge->tri_two_edge_indices[0], edge->tri_two_edge_indices[1],
                    vertex_out->num_vertices);
                return PRC_API_ERROR_PARSER;
            }
        }
    }
    printf("Debug: All edge indices validated successfully\n");
#endif

    /* Allocate first_duplicate_for_original - fresh for THIS face only */
    uncompressed_data->first_duplicate_for_original =
        (uint32_t *)prc_calloc(ctx, num_pos, sizeof(uint32_t));
    if (uncompressed_data->first_duplicate_for_original == NULL)
    {
        code = PRC_API_ERROR_MEMORY;
        prc_error(ctx, code,
            "Failed to allocate first_duplicate_for_original in prc_internal_api_compute_normals\n");
        goto failure;
    }

    /* Initialize all to UINT32_MAX (no duplicates yet) */
    for (size_t i = 0; i < num_pos; ++i)
        uncompressed_data->first_duplicate_for_original[i] = UINT32_MAX;

    /* Allocate vertex_to_original - fresh for THIS face only
       Allocate 4x capacity to handle vertex splitting during normal computation */
    size_t initial_capacity = vertex_out->capacity * 4;
    uncompressed_data->vertex_to_original =
        (uint32_t *)prc_calloc(ctx, initial_capacity, sizeof(uint32_t));
    if (uncompressed_data->vertex_to_original == NULL)
    {
        code = PRC_API_ERROR_MEMORY;
        prc_error(ctx, code,
            "Failed to allocate vertex_to_original in prc_internal_api_compute_normals\n");
        goto failure;
    }
    uncompressed_data->vertex_to_original_capacity = initial_capacity;

    /* Initialize ALL entries to UINT32_MAX, not just the first N */
    for (size_t i = 0; i < initial_capacity; ++i)
        uncompressed_data->vertex_to_original[i] = UINT32_MAX;

    /* Then set identity mapping for the initial vertices */
    for (size_t i = 0; i < vertex_out->num_vertices; ++i)
        uncompressed_data->vertex_to_original[i] = (uint32_t)i;

    /* ===== NORMAL COMPUTATION AND EDGE PROCESSING ===== */

    /* Process edges and compute/set/split normals as needed */
    for (k = 0; k < edge_list->num_edges; k++)
    {
        edge = &edge_list->edge[k];
        triangle_index = 1;

        prc_internal_api_get_edge_data(ctx, edge, indices_tri1, vertices_tri1, vertex_out,
            &vertex_indices_offset, &edge_case, triangle_index);

        if (edge->num_triangles == 1)
        {
            triangle_index = 1;
            prc_internal_api_get_normals_state(ctx, edge, vertices_tri1, &edge_point0_tri1_set,
                &edge_point1_tri1_set, triangle_index);

            code = prc_internal_api_compute_normal(vertices_tri1, &normal1);
            if (code < 0)
            {
                continue;
            }

            edge_index = 0;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal1, indices_tri1, crease_angle,
                edge_index, triangle_index, edge_point0_tri1_set,
                PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (triangle1 edge0) in prc_internal_api_compute_normals\n");
                goto failure;
            }

            edge_index = 1;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal1, indices_tri1, crease_angle,
                edge_index, triangle_index, edge_point1_tri1_set,
                PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (triangle1 edge1) in prc_internal_api_compute_normals\n");
                goto failure;
            }
        }
        else
        {
            /* Two triangles sharing the edge */
            code1 = prc_internal_api_compute_normal(vertices_tri1, &normal1);
            prc_internal_api_get_normals_state(ctx, edge, vertices_tri1, &edge_point0_tri1_set,
                &edge_point1_tri1_set, triangle_index);

            triangle_index = 2;
            prc_internal_api_get_edge_data(ctx, edge, indices_tri2, vertices_tri2, vertex_out,
                &vertex_indices_offset, &edge_case, triangle_index);
            code2 = prc_internal_api_compute_normal(vertices_tri2, &normal2);
            prc_internal_api_get_normals_state(ctx, edge, vertices_tri2, &edge_point0_tri2_set,
                &edge_point1_tri2_set, triangle_index);

            if (code1 == 0 && code2 == 0)
            {
                if (prc_internal_api_angle_less_than_crease_angle(ctx, normal1, normal2, crease_angle))
                {
                    prc_vec_avg(normal1, normal2, &normal_avg);
                }
                else
                {
                    code = prc_internal_api_split_edge(ctx, vertex_indices,
                        vertex_out, indices_tri1, edge, uncompressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code,
                            "Failed in prc_internal_api_split_edge in prc_internal_api_compute_normals\n");
                        goto failure;
                    }

                    /* Assign normals for triangle1 (may cause additional vertex splits) */
                    triangle_index = 1;
                    edge_index = 0;
                    code = prc_internal_api_vertex_edge_normal(ctx, edge,
                        vertex_out, vertex_indices, normal1, indices_tri1,
                        crease_angle, edge_index, triangle_index, edge_point0_tri1_set,
                        PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE, uncompressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code,
                            "Failed in prc_internal_api_vertex_edge_normal (split triangle1 edge0) in prc_internal_api_compute_normals\n");
                        goto failure;
                    }

                    edge_index = 1;
                    code = prc_internal_api_vertex_edge_normal(ctx, edge,
                        vertex_out, vertex_indices, normal1, indices_tri1,
                        crease_angle, edge_index, triangle_index, edge_point1_tri1_set,
                        PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE, uncompressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code,
                            "Failed in prc_internal_api_vertex_edge_normal (split triangle1 edge1) in prc_internal_api_compute_normals\n");
                        goto failure;
                    }

                    /* Assign normals to triangle2 (new duplicates) */
                    triangle_index = 2;
                    prc_internal_api_get_edge_data(ctx, edge, indices_tri2, vertices_tri2, vertex_out,
                        &vertex_indices_offset, &edge_case, triangle_index);

                    edge_index = 0;
                    normal_not_set = 0;
                    code = prc_internal_api_vertex_edge_normal(ctx, edge,
                        vertex_out, vertex_indices, normal2, indices_tri2,
                        crease_angle, edge_index, triangle_index, normal_not_set,
                        PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE, uncompressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code,
                            "Failed in prc_internal_api_vertex_edge_normal (split triangle2 edge0) in prc_internal_api_compute_normals\n");
                        goto failure;
                    }

                    edge_index = 1;
                    code = prc_internal_api_vertex_edge_normal(ctx, edge,
                        vertex_out, vertex_indices, normal2, indices_tri2,
                        crease_angle, edge_index, triangle_index, normal_not_set,
                        PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE, uncompressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code,
                            "Failed in prc_internal_api_vertex_edge_normal (split triangle2 edge1) in prc_internal_api_compute_normals\n");
                        goto failure;
                    }

                    continue;
                }
            }
            else
            {
                if (code1 == 0)
                    prc_vec_copy(normal1, &normal_avg, 0);
                else if (code2 == 0)
                    prc_vec_copy(normal2, &normal_avg, 0);
                else
                    continue;
            }

            /* Assign averaged normal to both triangles' edge vertices */
            edge_index = 0;
            triangle_index = 1;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal_avg, indices_tri1,
                crease_angle, edge_index, triangle_index, edge_point0_tri1_set,
                PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (avg triangle1 edge0) in prc_internal_api_compute_normals\n");
                goto failure;
            }

            edge_index = 1;
            triangle_index = 1;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal_avg, indices_tri1,
                crease_angle, edge_index, triangle_index, edge_point1_tri1_set,
                PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (avg triangle1 edge1) in prc_internal_api_compute_normals\n");
                goto failure;
            }

            edge_index = 0;
            triangle_index = 2;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal_avg, indices_tri2,
                crease_angle, edge_index, triangle_index, edge_point0_tri2_set,
                PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (avg triangle2 edge0) in prc_internal_api_compute_normals\n");
                goto failure;
            }

            edge_index = 1;
            triangle_index = 2;
            code = prc_internal_api_vertex_edge_normal(ctx, edge,
                vertex_out, vertex_indices, normal_avg, indices_tri2,
                crease_angle, edge_index, triangle_index, edge_point1_tri2_set,
                PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE, uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_vertex_edge_normal (avg triangle2 edge1) in prc_internal_api_compute_normals\n");
                goto failure;
            }
        }
    }

    /* ===== FREE PER-FACE LOCAL ALLOCATIONS ===== */
success_cleanup:
    if (uncompressed_data->vertex_to_original != NULL)
    {
        prc_free(ctx, uncompressed_data->vertex_to_original);
        uncompressed_data->vertex_to_original = NULL;
        uncompressed_data->vertex_to_original_capacity = 0;
    }

    if (uncompressed_data->first_duplicate_for_original != NULL)
    {
        prc_free(ctx, uncompressed_data->first_duplicate_for_original);
        uncompressed_data->first_duplicate_for_original = NULL;
    }

    if (had_failure)
        return code;

    /* Compact vertex buffer so only referenced vertices remain */
    code = prc_internal_api_compact_vertices(ctx, uncompressed_data);
    if (code < 0)
    {
        prc_error(ctx, code,
            "Failed in prc_internal_api_compact_vertices in prc_internal_api_compute_normals\n");
        return code;
    }

    return 0;

failure:
    had_failure = 1;
    goto success_cleanup;
}

static void
prc_internal_api_find_bounding_box(prc_context *ctx, prc_tess *tess, prc_api_tess *api_tess)
{
    if (tess->tess_type != PRC_TYPE_TESS_3D && tess->tess_type != PRC_TYPE_TESS_3D_Compressed &&
        tess->tess_type != PRC_TYPE_TESS_3D_Wire && tess->tess_type != PRC_TYPE_TESS_MarkUp)
        return;

    uint32_t num_vertices = tess->num_vertices_internal;
    prc_internal_api_vertex *vertices = tess->vertices_internal;

    if (num_vertices == 0 || vertices == NULL)
    {
        tess->bounding_box_min[0] = 0;
        tess->bounding_box_min[1] = 0;
        tess->bounding_box_min[2] = 0;

        tess->bounding_box_max[0] = 0;
        tess->bounding_box_max[1] = 0;
        tess->bounding_box_max[2] = 0;
        return;
    }

    tess->bounding_box_min[0] = vertices[0].position[0];
    tess->bounding_box_min[1] = vertices[0].position[1];
    tess->bounding_box_min[2] = vertices[0].position[2];

    tess->bounding_box_max[0] = vertices[0].position[0];
    tess->bounding_box_max[1] = vertices[0].position[1];
    tess->bounding_box_max[2] = vertices[0].position[2];

    for (uint32_t k = 1; k < num_vertices; k++)
    {
        if (vertices[k].position[0] < tess->bounding_box_min[0])
            tess->bounding_box_min[0] = vertices[k].position[0];
        if (vertices[k].position[1] < tess->bounding_box_min[1])
            tess->bounding_box_min[1] = vertices[k].position[1];
        if (vertices[k].position[2] < tess->bounding_box_min[2])
            tess->bounding_box_min[2] = vertices[k].position[2];

        if (vertices[k].position[0] > tess->bounding_box_max[0])
            tess->bounding_box_max[0] = vertices[k].position[0];
        if (vertices[k].position[1] > tess->bounding_box_max[1])
            tess->bounding_box_max[1] = vertices[k].position[1];
        if (vertices[k].position[2] > tess->bounding_box_max[2])
            tess->bounding_box_max[2] = vertices[k].position[2];
    }

    api_tess->bounding_box_min[0] = tess->bounding_box_min[0];
    api_tess->bounding_box_min[1] = tess->bounding_box_min[1];
    api_tess->bounding_box_min[2] = tess->bounding_box_min[2];

    api_tess->bounding_box_max[0] = tess->bounding_box_max[0];
    api_tess->bounding_box_max[1] = tess->bounding_box_max[1];
    api_tess->bounding_box_max[2] = tess->bounding_box_max[2];
}

static int
prc_internal_api_get_vertices(prc_context *ctx, prc_api_data data_in,
                uint32_t file_index, uint32_t tess_index,
                prc_internal_api_vertex **vertex_buffer_out,
                size_t *num_vertices_out)
{
    prc_data *data = (prc_data *)data_in;
    prc_tesslation_t tess_type;
    size_t k;
    int code = 0;
    size_t num_vertices = 0;
    prc_filestructure *file_struct = &data->file_struct[file_index];
    prc_tess *tess_entry;
    prc_internal_api_vertex *vertex_buffer = NULL;

    *num_vertices_out = 0;
    *vertex_buffer_out = NULL;

    if (tess_index > file_struct->tessellation->tess_count - 1)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER,
            "Tessellation index out of range in prc_internal_api_get_vertices\n");
        return PRC_API_ERROR_PARAMETER;
    }

    tess_entry = &file_struct->tessellation->tess[tess_index];
    tess_type = tess_entry->tess_type;

    switch (tess_type)
    {
    case PRC_TYPE_TESS_3D:
    {
        prc_tess_3d *tess = tess_entry->tess_3d;
        num_vertices = (size_t)tess->tessellation_coordinates.number_of_coordinates / 3;
        if (num_vertices == 0)
        {
            code = PRC_API_ERROR_PARSER;
            prc_error(ctx, code,
                "No uncompressed 3D vertices available in prc_internal_api_get_vertices\n");
            goto failure;
        }

        vertex_buffer =
            (prc_internal_api_vertex*) prc_calloc(ctx, num_vertices,
                                                  sizeof(prc_internal_api_vertex));
        if (vertex_buffer == NULL)
        {
            code = PRC_API_ERROR_MEMORY;
            prc_error(ctx, code,
                "Memory allocation failed for uncompressed 3D vertex buffer in prc_internal_api_get_vertices\n");
            goto failure;
        }

        for (k = 0; k < num_vertices; k++)
        {
            vertex_buffer[k].position[0] = (float)tess->tessellation_coordinates.coordinates[k * 3];
            vertex_buffer[k].position[1] = (float)tess->tessellation_coordinates.coordinates[k * 3 + 1];
            vertex_buffer[k].position[2] = (float)tess->tessellation_coordinates.coordinates[k * 3 + 2];
        }
        break;
    }
    case PRC_TYPE_TESS_3D_Compressed:
    {
        prc_tess_3d_compressed *tess = tess_entry->tess_3d_compressed;
        num_vertices = (size_t)tess->num_vertices_prc_compressed_3d;
        if (num_vertices == 0)
        {
            code = PRC_API_ERROR_PARSER;
            prc_error(ctx, code,
                "No compressed 3D vertices available in prc_internal_api_get_vertices\n");
            goto failure;
        }

        vertex_buffer =
            (prc_internal_api_vertex*) prc_calloc(ctx, num_vertices,
                                                  sizeof(prc_internal_api_vertex));
        if (vertex_buffer == NULL)
        {
            code = PRC_API_ERROR_MEMORY;
            prc_error(ctx, code,
                "Memory allocation failed for compressed 3D vertex buffer in prc_internal_api_get_vertices\n");
            goto failure;
        }

        for (k = 0; k < num_vertices; k++)
        {
            vertex_buffer[k].position[0] =
                (float)tess->vertices_prc_compressed_3d[k * 3];
            vertex_buffer[k].position[1] =
                (float)tess->vertices_prc_compressed_3d[k * 3 + 1];
            vertex_buffer[k].position[2] =
                (float)tess->vertices_prc_compressed_3d[k * 3 + 2];
        }
        break;
    }
    case PRC_TYPE_TESS_Face:
        code = PRC_API_ERROR_PARSER;
        prc_error(ctx, code,
            "PRC_TYPE_TESS_Face does not provide vertices in prc_internal_api_get_vertices\n");
        goto failure;
    case PRC_TYPE_TESS_3D_Wire:
    {
        prc_tess_3d_wire *tess = tess_entry->tess_3d_wire;

        num_vertices = (size_t)tess->tessellation_coordinates.number_of_coordinates / 3;
        if (num_vertices == 0)
        {
            code = PRC_API_ERROR_PARSER;
            prc_error(ctx, code,
                "No wire vertices available in prc_internal_api_get_vertices\n");
            goto failure;
        }

        vertex_buffer =
            (prc_internal_api_vertex *)prc_calloc(ctx, num_vertices,
                sizeof(prc_internal_api_vertex));
        if (vertex_buffer == NULL)
        {
            code = PRC_API_ERROR_MEMORY;
            prc_error(ctx, code,
                "Memory allocation failed for wire vertex buffer in prc_internal_api_get_vertices\n");
            goto failure;
        }

        for (k = 0; k < num_vertices; k++)
        {
            vertex_buffer[k].position[0] = (float)tess->tessellation_coordinates.coordinates[k * 3];
            vertex_buffer[k].position[1] = (float)tess->tessellation_coordinates.coordinates[k * 3 + 1];
            vertex_buffer[k].position[2] = (float)tess->tessellation_coordinates.coordinates[k * 3 + 2];
        }
        break;
    }
    case PRC_TYPE_TESS_MarkUp:
    {
        prc_tess_markup *tess = tess_entry->tess_markup;
        num_vertices = tess->decode_num_vertices;
        if (num_vertices == 0)
        {
            *num_vertices_out = 0;
            *vertex_buffer_out = NULL;
            return 0;
        }

        vertex_buffer =
            (prc_internal_api_vertex *)prc_calloc(ctx, num_vertices,
                sizeof(prc_internal_api_vertex));
        if (vertex_buffer == NULL)
        {
            code = PRC_API_ERROR_MEMORY;
            prc_error(ctx, code,
                "Memory allocation failed for markup vertex buffer in prc_internal_api_get_vertices\n");
            goto failure;
        }

        for (k = 0; k < num_vertices; k++)
        {
            vertex_buffer[k].position[0] = (float)tess->decode_vertices[k].x;
            vertex_buffer[k].position[1] = (float)tess->decode_vertices[k].y;
            vertex_buffer[k].position[2] = (float)tess->decode_vertices[k].z;
        }
        break;
    }

    default:
        code = PRC_API_ERROR_PARSER;
        prc_error(ctx, code,
            "Unsupported tessellation type in prc_internal_api_get_vertices\n");
        goto failure;
    }

    tess_entry->vertices_internal = vertex_buffer;
    *num_vertices_out = num_vertices;
    *vertex_buffer_out = vertex_buffer;
    return 0;

failure:
    if (vertex_buffer != NULL)
    {
        prc_free(ctx, vertex_buffer);
        vertex_buffer = NULL;
    }
    if (tess_entry != NULL)
    {
        tess_entry->vertices_internal = NULL;
    }
    *num_vertices_out = 0;
    *vertex_buffer_out = NULL;
    return code;
}

static int
prc_internal_api_get_normals(prc_context *ctx, prc_api_data data_in,
                             uint32_t file_index, uint32_t tess_index,
                             prc_internal_api_normal **normals_buffer_out,
                             size_t *num_normals_out)
{
    prc_data *data = (prc_data *)data_in;
    prc_tesslation_t tess_type;
    size_t k;
    size_t num_normals = 0;
    prc_filestructure *file_struct = &data->file_struct[file_index];

    *num_normals_out = 0;
    *normals_buffer_out = NULL;

    if (tess_index > file_struct->tessellation->tess_count - 1)
    {
        return PRC_API_ERROR_PARAMETER;
    }

    tess_type = file_struct->tessellation->tess[tess_index].tess_type;

    switch (tess_type)
    {
    case PRC_TYPE_TESS_3D:
    {
        prc_tess_3d *tess = file_struct->tessellation->tess[tess_index].tess_3d;

        if (tess->must_calculate_normals)
            return 0;

        num_normals = (size_t)tess->number_of_normal_coordinates / 3;
        if (num_normals == 0)
            return PRC_API_ERROR_PARSER;

        *num_normals_out = num_normals;
        prc_internal_api_normal *normals_buffer =
            (prc_internal_api_normal*) prc_calloc(ctx, num_normals,
                                                  sizeof(prc_internal_api_normal));
        file_struct->tessellation->tess[tess_index].normals_internal = normals_buffer;
        if (normals_buffer == NULL)
            return PRC_API_ERROR_MEMORY;

        for (k = 0; k < num_normals; k++)
        {
            normals_buffer[k].normal[0] = (float)tess->normal_coordinates[k * 3];
            normals_buffer[k].normal[1] = (float)tess->normal_coordinates[k * 3 + 1];
            normals_buffer[k].normal[2] = (float)tess->normal_coordinates[k * 3 + 2];
        }
        *normals_buffer_out = normals_buffer;
        break;
    }
    case PRC_TYPE_TESS_3D_Compressed:
    {
        prc_tess_3d_compressed *tess =
            file_struct->tessellation->tess[tess_index].tess_3d_compressed;
        num_normals = (size_t)tess->num_normals_prc_compressed_3d;
        if (num_normals == 0)
            return PRC_API_ERROR_PARSER;

        *num_normals_out = num_normals;
        prc_internal_api_normal *normals_buffer =
            (prc_internal_api_normal*) prc_calloc(ctx, num_normals,
                                                  sizeof(prc_internal_api_normal));
        file_struct->tessellation->tess[tess_index].normals_internal = normals_buffer;
        if (normals_buffer == NULL)
            return PRC_API_ERROR_MEMORY;

        for (k = 0; k < num_normals; k++)
        {
            normals_buffer[k].normal[0] =
                (float)tess->normals_prc_compressed_3d[k * 3];
            normals_buffer[k].normal[1] =
                (float)tess->normals_prc_compressed_3d[k * 3 + 1];
            normals_buffer[k].normal[2] =
                (float)tess->normals_prc_compressed_3d[k * 3 + 2];
        }
        *normals_buffer_out = normals_buffer;
        break;
    }
    case PRC_TYPE_TESS_Face:
        break;
    case PRC_TYPE_TESS_3D_Wire:
        break;
    case PRC_TYPE_TESS_MarkUp:
        break;
    default:
        return PRC_API_ERROR_PARSER;
    }
    return 0;
}

static void
prc_api_helper_set_default_material(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out,
    uint32_t pos)
{
    return;

    vertex_out->vertices[pos].color[0] = 192.0f / 255.0f;
    vertex_out->vertices[pos].color[1] = 192.0f / 255.0f;
    vertex_out->vertices[pos].color[2] = 192.0f / 255.0f;
    vertex_out->vertices[pos].color[3] = 1.0f;
    vertex_out->vertices[pos].alpha = 1.0f;
    vertex_out->vertices[pos].shininess = 0.2f;
    vertex_out->vertices[pos].tint[0] = 30.0f / 255.0f;
    vertex_out->vertices[pos].tint[1] = 30.0f / 255.0f;
    vertex_out->vertices[pos].tint[2] = 30.0f / 255.0f;
    vertex_out->vertices[pos].diffuse[0] = 192.0f / 255.0f;
    vertex_out->vertices[pos].diffuse[1] = 192.0f / 255.0f;
    vertex_out->vertices[pos].diffuse[2] = 192.0f / 255.0f;
    vertex_out->vertices[pos].emissive[0] = 0.0f;
    vertex_out->vertices[pos].emissive[1] = 0.0f;
    vertex_out->vertices[pos].emissive[2] = 0.0f;
    vertex_out->vertices[pos].specular[0] = 192.0f / 255.0f;
    vertex_out->vertices[pos].specular[1] = 192.0f / 255.0f;
    vertex_out->vertices[pos].specular[2] = 192.0f / 255.0f;
    vertex_out->vertices[pos].tri_has_material = false;
}

static void
prc_internal_api_apply_texture_transform(prc_context *ctx, prc_api_texture *api_texture,
    prc_api_tess_vertex_buffer *vertex, uint32_t index)
{
    double vector[3] = { vertex->vertices[index].uv[0],
                         vertex->vertices[index].uv[1], 1};
    double result[3];
    uint32_t k, j;

    for (k = 0; k < 3; k++)
    {
        result[k] = 0;
        for (j = 0; j < 3; j++)
        {
            result[k] += api_texture->transform[j * 3 + k] * vector[j];
        }
    }
    vertex->vertices[index].uv[0] = (float)result[0];
    vertex->vertices[index].uv[1] = (float)result[1];
}

static int
prc_api_helper_get_tess_and_file_index2(prc_context *ctx, prc_api_data data_in,
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

static int
prc_api_helper_get_alpha_value(prc_context *ctx, prc_api_data data_in,
                               uint32_t file_index, uint32_t style_index,
                               float *alpha_out)
{
    prc_data *data = (prc_data *)data_in;
    prc_filestructure *file_struct = &data->file_struct[file_index];
    prc_file_struct_internal_global_data *global_data = &file_struct->globals->global_data;
    prc_file_structure_header *header = file_struct->header;
    if (style_index >= global_data->style_count)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "Style index out of range in prc_api_helper_get_alpha_value\n");
        return PRC_API_ERROR_PARAMETER;
    }
    prc_graph_style *style = &global_data->styles[style_index];

    if (style->is_transparency)
    {
        *alpha_out = style->transparency / 255.0;
    }
    else
    {
        *alpha_out = 1.0f;
    }
    return 0;
}

#if DEBUG_TEXTURES
static int
prc_helper_write_png(char filename[], unsigned char *data, uint32_t width,
    uint32_t height, uint32_t num_channels)
{
    /* Lets use stb_image  */
    int stride_in_bytes = width * num_channels;
    int result = stbi_write_png(filename, width, height, num_channels, data, stride_in_bytes);
    if (result == 0)
    {
        return PRC_API_ERROR_UNSUPPORTED;
    }
    return 0;
}
#endif

static int
prc_api_helper_get_material_from_style_index(prc_context *ctx, prc_api_data data_in,
    uint32_t file_index, uint32_t style_index, float alpha, uint8_t dont_allow_texture,
    uint8_t *is_material, uint8_t *is_texture, uint8_t *is_pure_color,
    prc_api_material *material, prc_internal_graph_style *style, float *color,
    uint32_t face_index, uint32_t tess_index, uint8_t *had_defined_style)
{
    prc_data *data = (prc_data *)data_in;
    prc_filestructure *file_struct;
    prc_file_struct_internal_global_data *global_data;
    prc_file_structure_header *header;
    int code;
    prc_graph_material prc_material;
    *had_defined_style = 0;
    *is_pure_color = 0;

    if (file_index >= data->file_structure_count)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "File index out of range in prc_api_helper_get_material_from_style_index\n");
        return PRC_API_ERROR_PARAMETER;
    }

    file_struct = &data->file_struct[file_index];
    global_data = &file_struct->globals->global_data;
    header = file_struct->header;

    prc_internal_api_initialize_style(ctx, style);
    style->face_style_file_index = file_index;
    style->face_style_index = style_index;

    if (global_data->styles == NULL || style_index < 0 ||
        file_index >= data->file_structure_count ||
        style_index >= data->file_struct[file_index].globals->global_data.style_count)
    {
        /* No styles in global. Or error. Use a default
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

    *had_defined_style = 1;

    /* style_index_in is unbiased */
    if (style_index < global_data->style_count)
    {
        prc_graph_style graph_style = global_data->styles[style_index];

        if (graph_style.is_transparency)
        {
            alpha = graph_style.transparency / 255.0f;
        }
        else
        {
            alpha = 1.0;
        }

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
                /* This is a texture. It may be that we don't have
                   texture coordinates in the uncompressed tessellation case.
                   In that case we want to return the material properties of the
                   generic material.
                   */
                if (dont_allow_texture)
                {
                    /* Use the generic material */
                    int32_t material_index = graph_style.biased_color_index - 1;
                    prc_graph_material graph_material = global_data->materials[material_index];
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

                    style->diffuse_alpha = 1.0;
                    style->ambient_alpha = 1.0;
                    style->specular_alpha = 1.0;
                    style->emissive_alpha = 1.0;

                    // **FIX**: Copy material values from style (which has generic material) to output material
                    material->ambient_alpha = (float)style->ambient_alpha;
                    material->diffuse_alpha = (float)style->diffuse_alpha;
                    material->emissive_alpha = (float)style->emissive_alpha;
                    material->specular_alpha = (float)style->specular_alpha;
                    material->shininess = (float)style->shininess;
                    memcpy(material->ambient, style->ambient_color, 3 * sizeof(float));
                    memcpy(material->diffuse, style->diffuse_color, 3 * sizeof(float));
                    memcpy(material->emissive, style->emissive_color, 3 * sizeof(float));
                    memcpy(material->specular, style->specular_color, 3 * sizeof(float));

                    // Make sure material has alpha = 1.0 for textured objects (override what we just copied)
                    material->diffuse_alpha = 1.0f;
                    material->ambient_alpha = 1.0f;
                    material->specular_alpha = 1.0f;
                    material->emissive_alpha = 1.0f;
                }
            }
            else if (graph_style.tag == PRC_TYPE_GRAPH_Style)
            {
                /* We don't know yet if it is something with material properties
                   or has a texture. We have to dig deeper. Get the material */
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

                        style->diffuse_alpha = 1.0;
                        style->ambient_alpha = 1.0;
                        style->specular_alpha = 1.0;
                        style->emissive_alpha = 1.0;

                        // **FIX**: Copy material values from style (which has generic material) to output material
                        material->ambient_alpha = (float)style->ambient_alpha;
                        material->diffuse_alpha = (float)style->diffuse_alpha;
                        material->emissive_alpha = (float)style->emissive_alpha;
                        material->specular_alpha = (float)style->specular_alpha;
                        material->shininess = (float)style->shininess;
                        memcpy(material->ambient, style->ambient_color, 3 * sizeof(float));
                        memcpy(material->diffuse, style->diffuse_color, 3 * sizeof(float));
                        memcpy(material->emissive, style->emissive_color, 3 * sizeof(float));
                        memcpy(material->specular, style->specular_color, 3 * sizeof(float));

                        // Make sure material has alpha = 1.0 for textured objects (override what we just copied)
                        material->diffuse_alpha = 1.0f;
                        material->ambient_alpha = 1.0f;
                        material->specular_alpha = 1.0f;
                        material->emissive_alpha = 1.0f;
                    }
                }
                else
                {
                    /* This is a material */
                    style->is_material = 1;
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

                    /* Need to investigate why these are set to zero sometimes */
                    style->ambient_alpha = 1;
                    style->diffuse_alpha = alpha;
                    style->emissive_alpha = 1;
                    style->specular_alpha = 1;
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
            *is_pure_color = 1;
            color_index_unbiased = color_index_unbiased / 3;
            if (color_index_unbiased < global_data->color_count)
            {
                prc_rgb_color rgb_color = global_data->colors[color_index_unbiased];
                color[0] = (float)rgb_color.red;
                color[1] = (float)rgb_color.green;
                color[2] = (float)rgb_color.blue;
                color[3] = (float)rgb_color.alpha;

                style->tint[0] = color[0];
                style->tint[1] = color[1];
                style->tint[2] = color[2];
                style->tint[3] = color[3];
            }
        }
    }
    else
    {
        return PRC_ERROR_PARSE;
    }
    return 0;
}

static int
prc_api_helper_get_face_style(prc_context *ctx, prc_api_data data_in,
    uint32_t file_index, uint32_t style_index, float alpha,
    uint8_t dont_allow_texture, uint8_t *is_material, uint8_t *is_texture,
    uint8_t *is_pure_color, prc_api_material *material,
    prc_internal_graph_style *style, float *color, prc_api_tess *api_tess,
    prc_api_face *face_out, uint32_t face_index, uint32_t tess_index,
    uint8_t *had_defined_style)
{
    int32_t code;
    prc_data *data = (prc_data *)data_in;
    prc_filestructure *file_struct = &data->file_struct[file_index];
    prc_file_structure_header *style_header = data->file_struct[file_index].header;

    code = prc_api_helper_get_material_from_style_index(ctx, data_in,
        file_index, style_index, alpha, dont_allow_texture,
        is_material, is_texture, is_pure_color, material, style, color,
        face_index, tess_index, had_defined_style);
    if (code < 0)
        return code;

    face_out->is_texture = *is_texture;
    face_out->is_material = *is_material;
    api_tess->is_material = *is_material;

    if (face_out->is_texture)
    {
        style->texture.picture_data = style_header->files[style->texture.picture_index].raw_image;
        face_out->texture.data = style_header->files[style->texture.picture_index].raw_image;
        face_out->texture.width = style->texture.picture_width;
        face_out->texture.height = style->texture.picture_height;
        face_out->texture.num_channels = style->texture.num_elements_per_pixel;
        face_out->texture.has_transform = style->texture.has_texture_transform;
        if (style->texture.has_texture_transform)
        {
            memcpy(face_out->texture.transform, style->texture.texture_transform, 9 * sizeof(double));
        }
    }

    if (face_out->is_material && !face_out->is_texture)
    {
        // Copy material colors from style to face_out
        memcpy(face_out->material.diffuse, style->diffuse_color, 3 * sizeof(float));
        memcpy(face_out->material.ambient, style->ambient_color, 3 * sizeof(float));
        memcpy(face_out->material.specular, style->specular_color, 3 * sizeof(float));
        memcpy(face_out->material.emissive, style->emissive_color, 3 * sizeof(float));
        face_out->material.shininess = (float)style->shininess;
        face_out->material.ambient_alpha = (float)style->ambient_alpha;
        face_out->material.diffuse_alpha = (float)style->diffuse_alpha;
        face_out->material.specular_alpha = (float)style->specular_alpha;
        face_out->material.emissive_alpha = (float)style->emissive_alpha;
    }
    return 0;
}

static int
prc_api_helper_get_style_index_from_leaf(prc_context *ctx, prc_data *data,
    uint32_t face_index, prc_api_object_style *style_in,
    int32_t *style_unbiased_index, uint32_t *file_index)
{
    uint32_t done = 0;
    *style_unbiased_index = -1;
    *file_index = 0;
    prc_misc_entity_reference *entity_reference = NULL;
    uint32_t biased_file_index = 0;
    int32_t entity_ref_file_index = -1;
    uint32_t ref_index;
    int j;
    uint32_t k;
    uint32_t i;
    uint32_t leaf_uid;
    uint32_t leaf_file_index;
    prc_api_object_style *style = style_in;

    if (style == NULL)
    {
        return 0;
    }

    /* Let start with the leaf style and update as we work backward through
       the parents. The parent may override the style of multiple children
       as well as setting styles of specific faces of its children. */

    /* Get the leaf information in case there is an entity reference to it
       from one of the products */
    leaf_file_index = style->file_index;
    leaf_uid = style->base_with_graphics->base.unique_id;

    while (!done)
    {
#if 0
        /* Handy for debugging style issues */
        if (style->base_with_graphics->base.name.name.size > 0 &&
            style->base_with_graphics->base.name.name.string != NULL)
        {
            char temp_str[] = "MBD_19207_7012761\0";
            if (strncmp(style->base_with_graphics->base.name.name.string, temp_str, strlen(temp_str) - 1) == 0)
            {
                int zz = 1;
            }
        }
#endif

        if (style->base_with_graphics->graphics_content.biased_index_of_line_style > 0)
        {
            *file_index = style->base_with_graphics->base.file_index;
            *style_unbiased_index = style->base_with_graphics->graphics_content.biased_index_of_line_style - 1;
        }

        /* Check on the references */
        if (style->num_entity_references > 0)
        {
            for (k = 0; k < style->num_entity_references; k++)
            {
                entity_reference = &style->entity_references[k];
                prc_reference_data *reference_data = &entity_reference->content_entity_ref.reference_data;
                if (reference_data->ref_type == PRC_TYPE_MISC_ReferenceOnPRCBase)
                {
                    /* Flashlight uses this */
                    prc_misc_reference_on_prcbase *nontopo_ref = &reference_data->non_topo_reference;
                    if (!nontopo_ref->flag)
                    {
                        biased_file_index =
                            prc_api_helper_get_biased_file_index_from_unique_id(ctx, data,
                                                nontopo_ref->different_unique_id);
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
                        entity_ref_file_index = *file_index;
                    }

                    ref_index = nontopo_ref->unique_id;

                    /* Check if this is the leaf */
                    if (ref_index == leaf_uid && entity_ref_file_index == leaf_file_index)
                    {
                        /* We have a match. Update the location and style information */
                        *file_index = entity_reference->content_entity_ref.base.base.file_index;
                        *style_unbiased_index = entity_reference->content_entity_ref.base.graphics_content.biased_index_of_line_style - 1;

                        /* TODO deal with changes in local references too */
                        if (entity_reference->content_entity_ref.index_of_local_coordinate != 0)
                        {
                            /* Throw an error for now so we catch this */
                            prc_error(ctx, PRC_ERROR_INTERNAL, "Local coordinate system reference in entity reference is not supported in prc_api_helper_copy_product_details\n");
                            return PRC_ERROR_INTERNAL;
                        }
                        break;
                    }
                }
                else
                {
                    /* PRC_TYPE_MISC_ReferenceOnPRCTopology case, which can
                       include face references.The woodshed file uses this */
                    prc_misc_reference_on_topology *topo_ref = &reference_data->topo_reference;
                    if (topo_ref->flag && topo_ref->type == PRC_TYPE_TOPO_Face &&
                        style_in->brep_type == PRC_TYPE_RI_BrepModel)
                    {
                        prc_additional_target_data *target_data = &topo_ref->data;
                        uint32_t index_of_topological_index = target_data->index_of_topological_index;
                        uint32_t index_of_body = target_data->index_of_body;

                        /* Check if we match the Brep references. If yes, then this may specify
                           face specific styles. */
                        if (index_of_topological_index == style_in->index_of_topological_index &&
                            index_of_body == style_in->index_of_body)
                        {
                            for (i = 0; i < target_data->number_of_indices; i++)
                            {
                                if (target_data->indices[i] == face_index)
                                {
                                    /* We have a match. Update the location and style information
                                       The style comes from the entity reference base */
                                    *file_index = entity_reference->content_entity_ref.base.base.file_index;
                                    *style_unbiased_index = entity_reference->content_entity_ref.base.graphics_content.biased_index_of_line_style - 1;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (style->parent == NULL)
        {
            done = 1;
        }
        else
        {
            style = style->parent;
        }
    }

    return 0;
}

/* Apply texture coordinates to a vertex, including transform and V-flip
   The V-flip (1-v) must be applied AFTER the transform, not before */
void
prc_internal_api_set_vertex_texture_coords(prc_context *ctx,
    prc_api_vertex *vertex,
    double raw_u,
    double raw_v,
    double *transform_matrix,  // Changed: pass matrix directly instead of texture struct
    uint8_t has_transform)
{
    vertex->uv[0] = (float)raw_u;
    vertex->uv[1] = (float)raw_v;
    vertex->uv_set = true;

    if (has_transform)
    {
        double u = raw_u;
        double v = raw_v;

        vertex->uv[0] = (float)(transform_matrix[0] * u +
            transform_matrix[1] * v + transform_matrix[2]);
        vertex->uv[1] = (float)(transform_matrix[3] * u +
            transform_matrix[4] * v + transform_matrix[5]);
    }

    /* V-flip must happen AFTER transform */
    vertex->uv[1] = 1 - vertex->uv[1];
}

/* Special handling for the wire data that may be embedded in 3D tessellation
   and that we create for the 3D compressed data */
PRC_EXPORT int
prc_api_get_line_tessellation_vertices(prc_context *ctx, prc_api_data data_in,
    prc_api_product *api_tree, uint32_t tess_index_in, uint32_t face_index,
    prc_api_face *face_out, prc_api_tess *tess_line)
{
    prc_data *data = (prc_data *)data_in;
    prc_tesslation_t tess_type;
    size_t k, j;
    int code;
    prc_tess *tess;
    int32_t default_style_index_unbiased = -1;
    uint32_t default_style_file_index = 0;
    prc_api_tess_vertex_buffer *vertex_out = &face_out->face_vertices;
    prc_filestructure *file_struct;
    prc_file_struct_internal_global_data *global_data;
    prc_file_structure_header *header;
    uint32_t file_index;
    uint32_t tess_index; /* This is the actual tess index into file_index */
    uint32_t used_vertex_count = 0;
    uint32_t *vertex_remap = NULL;

    face_out->num_graphic_primitives = 0;
    face_out->is_material = 0;
    face_out->has_transparency = 0;
    face_out->is_texture = 0;
    face_out->vertices_have_style = 0;
    face_out->face_has_single_style = 0;
    face_out->single_style_index = 0;
    face_out->reserved = NULL;
    face_out->disable_face = 0;
    face_out->texture.data = NULL;

    vertex_out->num_vertices = 0;
    vertex_out->capacity = 0;
    vertex_out->vertices = NULL;

    code = prc_api_helper_get_tess_and_file_index2(ctx, data_in, tess_index_in,
        &file_index, &tess_index);
    if (code < 0)
    {
        prc_error(ctx, code,
            "Failed in prc_api_helper_get_tess_and_file_index2 in prc_api_get_line_tessellation_vertices\n");
        return code;
    }

    file_struct = &data->file_struct[file_index];
    global_data = &file_struct->globals->global_data;
    header = file_struct->header;

    if (tess_index > file_struct->tessellation->tess_count - 1)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER,
            "Tessellation index out of range in prc_api_get_line_tessellation_vertices\n");
        return PRC_API_ERROR_PARAMETER;
    }
    tess = &file_struct->tessellation->tess[tess_index];
    tess_type = tess->tess_type;

    if (file_struct == NULL || file_index >= data->file_structure_count)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER,
            "File index out of range in prc_api_get_line_tessellation_vertices\n");
        return PRC_API_ERROR_PARAMETER;
    }

    if (tess_type == PRC_TYPE_TESS_3D_Compressed)
    {
        prc_tess_3d_compressed *tess3d_compressed = tess->tess_3d_compressed;
        uint32_t num_of_edges = tess3d_compressed->number_of_edges;
        uint32_t num_vertices = (uint32_t)tess3d_compressed->num_vertices_prc_compressed_3d;
        prc_internal_api_wire *wire = NULL;

        if (num_of_edges == 0)
        {
            /* Nothing to export for this tessellation. */
            face_out->num_graphic_primitives = 0;
            face_out->disable_face = 1;
            return 0;
        }

        if (tess3d_compressed->edge_indices == NULL ||
            tess3d_compressed->edge_vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "Compressed wire edge arrays are NULL in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        vertex_remap = (uint32_t *)prc_malloc(ctx, num_vertices * sizeof(uint32_t));
        if (vertex_remap == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for vertex_remap (compressed wire) in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_MEMORY;
        }

        for (k = 0; k < num_vertices; k++)
        {
            vertex_remap[k] = (uint32_t)-1;
        }

        /* First pass: build old->new index remap for edge endpoints only. */
        for (k = 0; k < num_of_edges; k++)
        {
            for (j = 0; j < 2; j++)
            {
                uint32_t old_index = tess3d_compressed->edge_indices[k * 2 + j];
                if (old_index >= num_vertices)
                {
                    prc_error(ctx, PRC_API_ERROR_PARAMETER,
                        "Compressed wire edge index out of range in prc_api_get_line_tessellation_vertices\n");
                    prc_free(ctx, vertex_remap);
                    return PRC_API_ERROR_PARAMETER;
                }
                if (vertex_remap[old_index] == (uint32_t)-1)
                {
                    vertex_remap[old_index] = used_vertex_count;
                    used_vertex_count++;
                }
            }
        }

        vertex_out->num_vertices = used_vertex_count;
        vertex_out->capacity = used_vertex_count;
        vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx, used_vertex_count,
            sizeof(prc_api_vertex));
        if (vertex_out->vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for compressed wire vertex buffer in prc_api_get_line_tessellation_vertices\n");
            prc_free(ctx, vertex_remap);
            return PRC_API_ERROR_MEMORY;
        }

        /* Materialize only used vertices into compact vertex buffer. */
        for (k = 0; k < num_vertices; k++)
        {
            uint32_t new_index = vertex_remap[k];
            if (new_index == (uint32_t)-1)
                continue;

            vertex_out->vertices[new_index].position[0] = tess3d_compressed->edge_vertices[k * 3];
            vertex_out->vertices[new_index].position[1] = tess3d_compressed->edge_vertices[k * 3 + 1];
            vertex_out->vertices[new_index].position[2] = tess3d_compressed->edge_vertices[k * 3 + 2];

            vertex_out->vertices[new_index].color[0] = 0;
            vertex_out->vertices[new_index].color[1] = 0;
            vertex_out->vertices[new_index].color[2] = 0;
            vertex_out->vertices[new_index].color[3] = 1;
        }

        wire = (prc_internal_api_wire *)prc_calloc(ctx, num_of_edges,
            sizeof(prc_internal_api_wire));
        if (wire == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for compressed wire primitive array in prc_api_get_line_tessellation_vertices\n");
            prc_free(ctx, vertex_out->vertices);
            vertex_out->vertices = NULL;
            vertex_out->num_vertices = 0;
            vertex_out->capacity = 0;
            prc_free(ctx, vertex_remap);
            return PRC_API_ERROR_MEMORY;
        }

        for (k = 0; k < num_of_edges; k++)
        {
            wire[k].num_indices = 2;
            wire[k].capacity = 2;
            wire[k].vertex_indices = (uint32_t *)prc_calloc(ctx, 2, sizeof(uint32_t));
            if (wire[k].vertex_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for compressed wire primitive indices in prc_api_get_line_tessellation_vertices\n");
                for (j = 0; j < k; j++)
                {
                    prc_free(ctx, wire[j].vertex_indices);
                }
                prc_free(ctx, wire);
                prc_free(ctx, vertex_out->vertices);
                vertex_out->vertices = NULL;
                vertex_out->num_vertices = 0;
                vertex_out->capacity = 0;
                prc_free(ctx, vertex_remap);
                return PRC_API_ERROR_MEMORY;
            }

            wire[k].vertex_indices[0] = vertex_remap[tess3d_compressed->edge_indices[k * 2]];
            wire[k].vertex_indices[1] = vertex_remap[tess3d_compressed->edge_indices[k * 2 + 1]];
            wire[k].type = PRC_API_LINE;
        }

        prc_free(ctx, vertex_remap);

        /* Set the reserved data */
        face_out->num_graphic_primitives = num_of_edges;
        face_out->reserved = (void *)wire;
    }
    else if (tess_type == PRC_TYPE_TESS_3D)
    {
        prc_tess_3d *tess3d = tess->tess_3d;
        uint32_t number_of_wire_indices = tess3d->number_of_wire_indices;
        uint32_t *wire_indices = tess3d->wire_indices;
        prc_tess_face *prc_face;
        uint32_t start_of_wire_data = 0;
        uint32_t size_of_sizes_wire = 0;
        uint32_t *sizes_wire = NULL;
        prc_internal_api_wire *wire = NULL;
        uint32_t prc_num_vertices = tess3d->tessellation_coordinates.number_of_coordinates / 3;
        uint32_t num_vertices = 0;
        uint32_t *wire_indices_face = NULL;
        uint32_t total_face_wire_indices = 0;

        if (tess3d->face_tessellation_data == NULL ||
            face_index >= tess3d->number_of_face_tessellation)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "Face index out of range for uncompressed wire extraction in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        prc_face = &tess3d->face_tessellation_data[face_index];
        start_of_wire_data = prc_face->start_of_wire_data;
        size_of_sizes_wire = prc_face->size_of_sizes_wire;

        if (size_of_sizes_wire == 0)
        {
            /* No wire data for this face. Just return. */
            face_out->num_graphic_primitives = 0;
            face_out->disable_face = 1;
            return 0;
        }

        sizes_wire = prc_face->sizes_wire;

        if (size_of_sizes_wire > 0 && sizes_wire == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "sizes_wire is NULL for uncompressed wire extraction in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        if (number_of_wire_indices > 0 && wire_indices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "wire_indices is NULL for uncompressed wire extraction in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        if (start_of_wire_data > number_of_wire_indices)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "start_of_wire_data out of range in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        for (k = 0; k < size_of_sizes_wire; k++)
        {
            uint32_t num_indices = sizes_wire[k] & 0x3FFF;
            if (num_indices > number_of_wire_indices - total_face_wire_indices)
            {
                prc_error(ctx, PRC_API_ERROR_PARAMETER,
                    "Wire index count exceeds available indices in prc_api_get_line_tessellation_vertices\n");
                return PRC_API_ERROR_PARAMETER;
            }
            total_face_wire_indices += num_indices;
        }

        if (total_face_wire_indices > number_of_wire_indices - start_of_wire_data)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "Computed total face wire indices out of range in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_PARAMETER;
        }

        wire_indices_face = &wire_indices[start_of_wire_data];

        vertex_out->num_vertices = 0;
        vertex_out->capacity = 0;
        vertex_out->vertices = NULL;

        vertex_remap = (uint32_t *)prc_malloc(ctx, prc_num_vertices * sizeof(uint32_t));
        if (vertex_remap == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for vertex_remap (uncompressed wire) in prc_api_get_line_tessellation_vertices\n");
            return PRC_API_ERROR_MEMORY;
        }

        for (k = 0; k < prc_num_vertices; k++)
        {
            vertex_remap[k] = (uint32_t)-1;
        }

        /* First pass: build old->new index remap for each of the points */
        uint32_t wire_index_offset = 0;
        for (k = 0; k < size_of_sizes_wire; k++)
        {
            uint32_t num_indices = sizes_wire[k] & 0x3FFF;
            for (j = 0; j < num_indices; j++)
            {
                uint32_t old_index = wire_indices_face[wire_index_offset] / 3;
                wire_index_offset++;
                if (old_index >= prc_num_vertices)
                {
                    prc_error(ctx, PRC_API_ERROR_PARAMETER,
                        "Uncompressed wire index out of range in prc_api_get_line_tessellation_vertices\n");
                    prc_free(ctx, vertex_remap);
                    return PRC_API_ERROR_PARAMETER;
                }
                if (vertex_remap[old_index] == (uint32_t)-1)
                {
                    vertex_remap[old_index] = used_vertex_count;
                    used_vertex_count++;
                }
            }
        }

        vertex_out->num_vertices = used_vertex_count;
        vertex_out->capacity = used_vertex_count;
        vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx, used_vertex_count,
                                                    sizeof(prc_api_vertex));
        if (vertex_out->vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for uncompressed wire vertex buffer in prc_api_get_line_tessellation_vertices\n");
            prc_free(ctx, vertex_remap);
            return PRC_API_ERROR_MEMORY;
        }

        /* Materialize only used vertices into compact vertex buffer. */
        for (k = 0; k < prc_num_vertices; k++)
        {
            uint32_t new_index = vertex_remap[k];
            if (new_index == (uint32_t)-1)
                continue;

            vertex_out->vertices[new_index].position[0] = tess3d->tessellation_coordinates.coordinates[k * 3];
            vertex_out->vertices[new_index].position[1] = tess3d->tessellation_coordinates.coordinates[k * 3 + 1];
            vertex_out->vertices[new_index].position[2] = tess3d->tessellation_coordinates.coordinates[k * 3 + 2];

            vertex_out->vertices[new_index].color[0] = 0;
            vertex_out->vertices[new_index].color[1] = 0;
            vertex_out->vertices[new_index].color[2] = 0;
            vertex_out->vertices[new_index].color[3] = 1;
        }

        wire = (prc_internal_api_wire *)prc_calloc(ctx, size_of_sizes_wire,
            sizeof(prc_internal_api_wire));
        if (wire == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for uncompressed wire primitive array in prc_api_get_line_tessellation_vertices\n");
            prc_free(ctx, vertex_out->vertices);
            vertex_out->vertices = NULL;
            vertex_out->num_vertices = 0;
            vertex_out->capacity = 0;
            prc_free(ctx, vertex_remap);
            return PRC_API_ERROR_MEMORY;
        }

        wire_index_offset = 0;
        for (k = 0; k < size_of_sizes_wire; k++)
        {
            wire[k].num_indices = sizes_wire[k] & 0x3FFF;
            wire[k].capacity = sizes_wire[k] & 0x3FFF;
            wire[k].vertex_indices = (uint32_t *)prc_calloc(ctx, wire[k].num_indices, sizeof(uint32_t));
            if (wire[k].vertex_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for uncompressed wire primitive indices in prc_api_get_line_tessellation_vertices\n");
                for (j = 0; j < k; j++)
                {
                    prc_free(ctx, wire[j].vertex_indices);
                }
                prc_free(ctx, wire);
                prc_free(ctx, vertex_out->vertices);
                vertex_out->vertices = NULL;
                vertex_out->num_vertices = 0;
                vertex_out->capacity = 0;
                prc_free(ctx, vertex_remap);
                return PRC_API_ERROR_MEMORY;
            }

            for (j = 0; j < wire[k].num_indices; j++)
            {
                wire[k].vertex_indices[j] = vertex_remap[wire_indices_face[wire_index_offset] / 3];
                wire_index_offset++;
            }

            wire[k].type = PRC_API_LINE;
        }
        prc_free(ctx, vertex_remap);

        /* Set the reserved data */
        face_out->num_graphic_primitives = size_of_sizes_wire;
        face_out->reserved = (void *)wire;
    }

    tess_line->type = PRC_API_TESS_3D_Wire_Extra;
    return 0;
}

/* tess_index_in is an index into the tessellations in part_details or
   markup_details in prc_data */
PRC_EXPORT int
prc_api_get_tessellation_vertices(prc_context *ctx, prc_api_data data_in,
    prc_api_product *api_tree, uint32_t tess_index_in, uint32_t face_index,
    prc_api_face *face_out, prc_api_tess *api_tess)
{
    prc_data *data = (prc_data *)data_in;
    prc_tesslation_t tess_type;
    size_t k, j;
    int code;
    prc_tess *tess;
    prc_internal_api_face *face_out_reserved;
    float face_color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    int32_t face_style_index = -1;
    int32_t default_style_index_unbiased = -1;
    uint32_t default_style_file_index = 0;
    prc_internal_api_normal_state_t normal_state;
    prc_internal_api_texture_state_t texture_state;
    uint8_t has_texture;
    uint8_t is_pure_color = 0;
    prc_api_tess_vertex_buffer *vertex_out = &api_tess->tess_vertices;
    uint8_t has_face = (face_out != NULL) ? true : false;
    uint8_t has_normals = (api_tess->type != PRC_API_TESS_3D_Wire && api_tess->type != PRC_API_TESS_MarkUp) ? true : false;
    uint32_t *compressed_tess_face_style = NULL;
    prc_filestructure *file_struct;
    prc_file_struct_internal_global_data *global_data;
    prc_file_structure_header *header;
    uint32_t file_index;
    uint32_t tess_index; /* This is the actual tess index into file_index */
    float alpha = 1.0f;
    int leaf_style_unbiased_index = -1;
    uint32_t leaf_style_file_index = 0;
    prc_internal_graph_style graph_style;
    uint8_t has_more_than_one_ref_style = false;
    uint8_t has_ref_style_defined = false;
    uint8_t external_style_defined = true;
    uint8_t is_uncompressed_with_no_texture_entities = false;

#define PRC_API_FAIL(_code, _msg)            \
    do {                                     \
        prc_error(ctx, (_code), (_msg));     \
        code = (_code);                      \
        goto failure;                        \
    } while (0)

    api_tess->has_transparency = 0;

    code = prc_api_helper_get_tess_and_file_index2(ctx, data_in, tess_index_in,
        &file_index, &tess_index);
    if (code < 0)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "Failed in prc_api_helper_get_tess_and_file_index2\n");
        return code;
    }

    file_struct = &data->file_struct[file_index];
    global_data = &file_struct->globals->global_data;
    header = file_struct->header;

    if (tess_index > file_struct->tessellation->tess_count - 1)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "Tess index larger than available tessellations\n");
        return PRC_API_ERROR_PARAMETER;
    }
    tess = &file_struct->tessellation->tess[tess_index];
    tess_type = tess->tess_type;

    if (file_struct == NULL || file_index >= data->file_structure_count)
    {
        prc_error(ctx, PRC_API_ERROR_PARAMETER, "File index larger than available file structure\n");
        return PRC_API_ERROR_PARAMETER;
    }

    if (has_face)
    {
        prc_api_initialize_face(ctx, face_out);
        prc_api_initialize_texture(ctx, &face_out->texture);

        alpha = 1.0;

        face_out->reserved = (void *)prc_calloc(ctx, 1, sizeof(prc_internal_api_face));
        if (face_out->reserved == NULL)
            PRC_API_FAIL(PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_out->reserved in prc_api_get_tessellation_vertices\n");
        face_out_reserved = prc_face_internal_face(face_out);

        if (tess_type == PRC_TYPE_TESS_3D)
        {
            prc_tess_3d *tess3d = tess->tess_3d;

            if (face_index > tess3d->number_of_face_tessellation - 1)
                PRC_API_FAIL(PRC_API_ERROR_PARAMETER,
                    "Face index out of range for uncompressed tessellation in prc_api_get_tessellation_vertices\n");

            prc_tess_face face = tess3d->face_tessellation_data[face_index];

            if (face.used_entities_flag < PRC_FACETESSDATA_TriangleTextured)
            {
                /* This means we have no textured entities. */
                is_uncompressed_with_no_texture_entities = true;
            }

            face_out_reserved->style = (prc_internal_graph_style *)prc_calloc(ctx,
                1, sizeof(prc_internal_graph_style));
            if (face_out_reserved->style == NULL)
            {
                PRC_API_FAIL(PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_out_reserved->style in prc_api_get_tessellation_vertices\n");
            }
            face_out_reserved->owns_style = 1; /* this face's own allocation, not a shared cache */

            /* Lets get the style information for this face. We will traverse
               backwards from the leaf (which is the RI) and forward from the product.
               dealing with inheritance etc along the way */
            code = prc_api_helper_get_style_index_from_leaf(ctx, data, face_index,
                (prc_api_object_style *)api_tess->style_leaf,
                &leaf_style_unbiased_index, &leaf_style_file_index);
            if (code < 0)
            {
                PRC_API_FAIL(code,
                    "Failed to get style from leaf in prc_api_get_tessellation_vertices\n");
            }

            code = prc_api_helper_get_face_style(ctx, data_in, leaf_style_file_index,
                leaf_style_unbiased_index, alpha, is_uncompressed_with_no_texture_entities,
                &api_tess->is_material, &has_texture, &is_pure_color, &api_tess->tess_material,
                face_out_reserved->style, face_color, api_tess, face_out, face_index,
                tess_index_in, &has_ref_style_defined);
            if (code < 0)
            {
                PRC_API_FAIL(code,
                    "Failed to get face style in prc_api_get_tessellation_vertices\n");
            }

            if (leaf_style_unbiased_index == -1 && !has_ref_style_defined)
            {
                /* In this case there was no style information externally
                   defined. We currently are set to use default settings. If
                   the face has some line styles or vertex colors then we will
                   use that. */
                external_style_defined = false;
            }
        }
        else
        {
            /* Compressed case */
            /* The style can be defined by reference for the faces OR it
               can be defined in the tessellation via the line styles or
               even by the vertex color. Question what do you do when you
               have both. */
            prc_tess_3d_compressed *tess_compressed = tess->tess_3d_compressed;
            prc_internal_api_position_normal_lookup *style_cache;
            if (tess_compressed == NULL)
                PRC_API_FAIL(PRC_API_ERROR_PARSER,
                    "Compressed tessellation payload is NULL in prc_api_get_tessellation_vertices\n");

            /* Defines Triangle, Fan, and Strips locations */
            if (tess_compressed->point_array_size == 0)
            {
                /* In the helicopter we end up here. That is because there is wire
                   data packed into this not triangulated data. We need to process
                   the wire data.  This also happens in 3dpdf-Industrial-Grinder TODO!
                   For now, we will set this tessellation as one to be skipped so
                   that we at least draw most of it. Not sure why they don't use
                   the PRC_TYPE_TESS_3D_Wire type of tessellation. */

                printf("Warning: Tess %u has no triangulated data.\n", tess_index);
                face_out->disable_face = 1;
                return 0;
            }

            /* This table (and has_more_than_one_ref_style, below) is
               identical across every face_index call for this SAME
               api_tess instance -- cache it on api_tess->reserved (a
               per-instance extension slot; see the vertex/normal cache
               built further down this function for the established
               pattern and why it must be per-instance, not shared across
               instances of the same underlying tess_index) instead of
               rebuilding it from scratch on every single call. Before
               this cache existed, an O(face_number) table was rebuilt on
               every one of the face_number calls the standard one-call-
               per-face walk (demos/quick_start's pattern, used by every
               demo/binding) makes, costing O(face_number^2) time and,
               since nothing freed a face's own copy until the whole walk
               finished, O(face_number^2) peak memory too -- confirmed
               against a real ~26000-face compressed tessellation, where
               the uncached version reached tens of GB before being
               killed. face_out_reserved->style below is a BORROWED
               pointer into this cache (owns_style left at its calloc'd
               default of 0), not its own allocation -- freed once, with
               the cache, in prc_api_release_data, not per-face. */
            style_cache = (prc_internal_api_position_normal_lookup *)api_tess->reserved;
            if (style_cache == NULL)
            {
                style_cache = (prc_internal_api_position_normal_lookup *)prc_calloc(ctx,
                    1, sizeof(prc_internal_api_position_normal_lookup));
                if (style_cache == NULL)
                    PRC_API_FAIL(PRC_API_ERROR_MEMORY,
                        "Memory allocation failed for style cache in prc_api_get_tessellation_vertices\n");
                api_tess->reserved = style_cache;
            }

            if (!style_cache->face_style_cache_valid)
            {
                uint32_t leaf_style_face_0 = 0;

                style_cache->face_style_cache = (prc_internal_graph_style *)prc_calloc(ctx,
                    tess_compressed->face_number, sizeof(prc_internal_graph_style));
                if (style_cache->face_style_cache == NULL)
                {
                    PRC_API_FAIL(PRC_API_ERROR_MEMORY,
                        "Memory allocation failed for compressed style table in prc_api_get_tessellation_vertices\n");
                }
                style_cache->face_style_cache_count = tess_compressed->face_number;

                /* We also want to know if all the faces have the same style */
                for (k = 0; k < tess_compressed->face_number; k++)
                {
                    /* Get the face style index for each face in the compressed tessellation.
                       We have to go from the leaf again, checking for each face in
                       this compressed case, as we deal with all the faces now when
                       working with compressed data */
                    /* Face index is k here */
                    code = prc_api_helper_get_style_index_from_leaf(ctx, data, k,
                        (prc_api_object_style *)api_tess->style_leaf,
                        &leaf_style_unbiased_index, &leaf_style_file_index);
                    if (code < 0)
                    {
                        PRC_API_FAIL(code,
                            "Failed to get style from leaf in prc_api_get_tessellation_vertices\n");
                    }

                    if (k == 0)
                    {
                        leaf_style_face_0 = leaf_style_unbiased_index;
                    }
                    else
                    {
                        if (leaf_style_unbiased_index != leaf_style_face_0)
                        {
                            /* We have different styles for different faces. We won't
                               be able to use the optimization of just looking at the
                               first face's style and applying it to all the faces. */
                            has_more_than_one_ref_style = true;
                        }
                    }
                    code = prc_api_helper_get_face_style(ctx, data_in, leaf_style_file_index,
                        leaf_style_unbiased_index, alpha, is_uncompressed_with_no_texture_entities,
                        &api_tess->is_material, &has_texture, &is_pure_color, &api_tess->tess_material,
                        &style_cache->face_style_cache[k], face_color, api_tess, face_out, k,
                        tess_index_in, &has_ref_style_defined);
                    if (code < 0)
                    {
                        PRC_API_FAIL(code,
                            "Failed to get face style in prc_api_get_tessellation_vertices\n");
                    }
                }
                style_cache->face_style_has_more_than_one_ref_style = has_more_than_one_ref_style;
                style_cache->face_style_cache_valid = 1;
            }
            else
            {
                uint32_t last_k = style_cache->face_style_cache_count - 1;

                has_more_than_one_ref_style = style_cache->face_style_has_more_than_one_ref_style;

                /* The loop above leaves face_out/api_tess mutated from its
                   LAST iteration (face_style_cache_count - 1), regardless
                   of which face_index the outer call actually asked for
                   (prc_api_helper_get_face_style's side effects on
                   face_out->is_texture/material and api_tess->is_material
                   aren't face_index-specific). Reproduce just that one
                   iteration's side effects here on a cache hit, instead of
                   the full O(face_number) loop -- this is why the cache
                   still costs O(1), not O(0), per call. */
                code = prc_api_helper_get_style_index_from_leaf(ctx, data, last_k,
                    (prc_api_object_style *)api_tess->style_leaf,
                    &leaf_style_unbiased_index, &leaf_style_file_index);
                if (code < 0)
                {
                    PRC_API_FAIL(code,
                        "Failed to get style from leaf in prc_api_get_tessellation_vertices\n");
                }
                code = prc_api_helper_get_face_style(ctx, data_in, leaf_style_file_index,
                    leaf_style_unbiased_index, alpha, is_uncompressed_with_no_texture_entities,
                    &api_tess->is_material, &has_texture, &is_pure_color, &api_tess->tess_material,
                    &style_cache->face_style_cache[last_k], face_color, api_tess, face_out, last_k,
                    tess_index_in, &has_ref_style_defined);
                if (code < 0)
                {
                    PRC_API_FAIL(code,
                        "Failed to get face style in prc_api_get_tessellation_vertices\n");
                }
            }

            face_out_reserved->style = style_cache->face_style_cache;
            face_out_reserved->number_of_styles = style_cache->face_style_cache_count;
        }
    }

    /* Get the tessalation vertices if we have not yet got them */
    if (tess->num_vertices_internal == 0)
    {
        code = prc_internal_api_get_vertices(ctx, data_in, file_index, tess_index,
            &tess->vertices_internal, &tess->num_vertices_internal);
        if (code < 0)
        {
            PRC_API_FAIL(code,
                "Failed to decode tessellation vertices in prc_api_get_tessellation_vertices\n");
        }
    }

    /* From the vertices compute a bounding box for this tessellation. This can be
       used for the default views */
    prc_internal_api_find_bounding_box(ctx, tess, api_tess);

    /* Only 3D and 3D compressed have normals */
    if (has_normals)
    {
        /* Get the tessalation normals if we have not yet got them AND they are already
           there.  We may need to calculate the normals from the triangles. */
        if (tess->num_normals_internal == 0)
        {
            code = prc_internal_api_get_normals(ctx, data_in, file_index,
                tess_index, &tess->normals_internal, &tess->num_normals_internal);
            if (code < 0)
            {
                PRC_API_FAIL(code,
                    "Failed to decode tessellation normals in prc_api_get_tessellation_vertices\n");
            }
        }
    }

    switch (tess_type)
    {
    case PRC_TYPE_TESS_3D:
    {
        if (!has_face)
            return PRC_API_ERROR_PARSER;

        prc_internal_api_tess_entities *entities_multiple_norms = &face_out_reserved->multi_norm;
        prc_internal_api_tess_entities *entities_one_norm = &face_out_reserved->single_norm;
        prc_internal_api_tess_entities *entities_textured_multiple_norms = &face_out_reserved->texture_multi_norm;
        prc_internal_api_tess_entities *entities_textured_one_norm = &face_out_reserved->texture_single_norm;
        prc_internal_api_uncomm_tess_data uncompressed_data;
        prc_api_tess_vertex_buffer *face_vertex_out = &face_out->face_vertices;
        size_t face_tessellation_index = 0;
        size_t num_indices = 0;
        prc_tess_3d *tess3d = tess->tess_3d;
        size_t index_count = 0;
        uint32_t *prc_vertex_indice_to_api_vertex_indice = NULL;
        uint32_t *prc_normal_indice_to_api_normal_indice = NULL;
        uint32_t *prc_texture_indice_to_api_texture_indice = NULL;
        float *face_initial_normals = NULL;
        float *face_initial_texture_coords = NULL;
        float *face_initial_positions = NULL;
        uint32_t *face_position_indices = NULL;
        uint32_t *face_normal_indices = NULL;
        uint32_t *face_texture_indices = NULL;
        uint32_t *face_vertex_color_indices = NULL;
        uint32_t single_normal_set;

        /* Set to avoid issues during release if this is skipped */
        face_vertex_out->vertices = NULL;

        memset(&uncompressed_data, 0, sizeof(uncompressed_data));

        uncompressed_data.has_pure_color = is_pure_color;
        if (is_pure_color)
        {
            for (k = 0; k < 4; k++)
            {
                uncompressed_data.pure_color[k] = face_color[k];
            }
        }

        if (face_index > tess3d->number_of_face_tessellation - 1)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "Face index out of range in uncompressed branch of prc_api_get_tessellation_vertices\n");
            code = PRC_API_ERROR_PARAMETER;
            goto uncompressed_failure;
        }

        prc_tess_face face = tess3d->face_tessellation_data[face_index];
        /* Different entitities in the face can have different line attributes
           That is what the size relates to. TODO. Add that feature here
           where we use different styles for the different entities. At this
           point we assume the case where this is just the same one for the
           entire face. */
        if (face.size_of_line_attributes != 0 && face.line_attributes[0] > 0)
        {
            /* line_attributes is biased */
            face_out_reserved->style->face_style_index = face.line_attributes[0] - 1;

            /* Use the tessellation file index */
            code = prc_api_helper_get_face_style(ctx, data_in, file_index,
                face_out_reserved->style->face_style_index, 1.0,
                is_uncompressed_with_no_texture_entities,
                &api_tess->is_material, &has_texture, &is_pure_color,
                &api_tess->tess_material, face_out_reserved->style, face_color,
                api_tess, face_out, face_index, tess_index_in,
                &has_ref_style_defined);
            if (code < 0)
            {
                prc_error(ctx, PRC_API_ERROR_PARAMETER, "Failed to get face style in prc_api_get_tessellation_vertices\n");
                goto uncompressed_failure;
            }
        }

        /* Get the index into the array of vertex indices for this face */
        uint32_t *src_index_data = &tess3d->triangulated_index_array[tess3d->face_tessellation_data[face_index].start_triangulated];

        if (face_out == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "face_out is NULL in uncompressed branch of prc_api_get_tessellation_vertices\n");
            code = PRC_API_ERROR_PARAMETER;
            goto uncompressed_failure;
        }

        /* This gets set even if we don't have a texture. In some files the style
           will not have a texture but the 3D tessellation data still has texture
           indices! */
        face_out_reserved->num_texture_coords = face.number_of_textured_coordinate_indexes;

        /* Defines Triangle, Fan, and Strips locations */
        if (face.size_of_triangulateddata == 0)
        {
            /* In the helicopter we end up here. That is because there is wire
               data packed into this not triangulated data. We need to process
               the wire data.  This also happens in 3dpdf-Industrial-Grinder TODO!
               For now, we will set this tessellation as one to be skipped so
               that we at least draw most of it. Not sure why they don't use
               the PRC_TYPE_TESS_3D_Wire type of tessellation. */

            printf("Warning: Face %u has no triangulated data.\n", face_index);
            face_out->disable_face = 1;
            return 0;
        }

        /* Data is stored such that a face can have a number of triangles, fans,
           and stripes. */
        /* The exact number is stored in the triangulateddata array. The types
           stored in there is defined by used_entities_flag.  The order is
           defined by Table 139. They may have multiple normals packed in the data,
           and they may have texture coordinates packed in the data */

        /* Third parameter on these next sets of functions indicates if we are
         * in a single or multiple normal set. */

           /* First we must check the multiple norm objects */
        if (face.used_entities_flag & PRC_FACETESSDATA_Triangle)
        {
            /* Read number of triangles with multiple norms*/
            prc_internal_api_set_triangles(ctx, face, 0, entities_multiple_norms,
                                        &face_tessellation_index, &num_indices);
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleFan)
        {
            /* Read number of fans with multiple norms */
            code = prc_internal_api_set_fans(ctx, face, 0, entities_multiple_norms,
                                        &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleFan entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleStripe)
        {
            /* Read number of strips with multiple norms */
            code = prc_internal_api_set_strips(ctx, face, 0, entities_multiple_norms,
                                            &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleStripe entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }

        /* Now we must check the single norm objects */
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleOneNormal)
        {
            /* Read number of triangles with one norm */
            prc_internal_api_set_triangles(ctx, face, 1, entities_one_norm,
                                        &face_tessellation_index, &num_indices);
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleFanOneNormal)
        {
            /* Read number of fans with one norm */
            code = prc_internal_api_set_fans(ctx, face, 1, entities_one_norm,
                                        &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleFanOneNormal entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleStripeOneNormal)
        {
            /* Read number of strips with one norm */
            code = prc_internal_api_set_strips(ctx, face, 1, entities_one_norm,
                                        &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleStripeOneNormal entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }

        /* Now we must check the multiple norm textured objects */
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleTextured)
        {
            /* Read number of triangles with multiple norms and texture */
            prc_internal_api_set_triangles(ctx, face, 0, entities_textured_multiple_norms,
                                        &face_tessellation_index, &num_indices);
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleFanTextured)
        {
            /* Read number of fans with multiple norms and texture */
            code = prc_internal_api_set_fans(ctx, face, 0, entities_textured_multiple_norms,
                                            &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleFanTextured entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleStripeTextured)
        {
            /* Read number of strips with multiple norms and texture */
            code = prc_internal_api_set_strips(ctx, face, 0,
                        entities_textured_multiple_norms, &face_tessellation_index,
                        &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleStripeTextured entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }

        /* Now we must check the single norm textured objects */
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleOneNormalTextured)
        {
            code = PRC_ERROR_NOT_IMPLEMENTED;
            prc_error(ctx, PRC_ERROR_NOT_IMPLEMENTED,
                "TriangleOneNormalTextured is not implemented in prc_api_get_tessellation_vertices\n");
            goto uncompressed_failure;
            /* Read number of triangles with one norm and texture */
            prc_internal_api_set_triangles(ctx, face, 1, entities_textured_one_norm,
                                        &face_tessellation_index, &num_indices);
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleFanOneNormalTextured)
        {
            /* Read number of fans with one norm and texture */
            code = prc_internal_api_set_fans(ctx, face, 1, entities_textured_one_norm,
                                        &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleFanOneNormalTextured entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }
        if (face.used_entities_flag & PRC_FACETESSDATA_TriangleStripeOneNormalTextured)
        {
            /* Read number of strips with one norm and texture */
            code = prc_internal_api_set_strips(ctx, face, 1, entities_textured_one_norm,
                                        &face_tessellation_index, &num_indices);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed to parse TriangleStripeOneNormalTextured entities in uncompressed PRC_TYPE_TESS_3D branch\n");
                goto uncompressed_failure;
            }
        }

        if (num_indices == 0)
        {
            /* We have no triangles, fans, or strips. That means we have no
               indices. We won't have any vertices to draw for this face. */
            face_out->disable_face = 1;
            return 0;
        }

        /* Total number of indices for all the triangles, fans, and strips for
           this face are counted in num_indices. We will use this for our
           initial allocations. */

        /* We have to combine the normal vector, position vector, color and texture
           coordinates into a vertex. For each unique position vector / normal
           vector / color / texture combination make a new vertex and add its
           position to the vertex indices. For now though we just create the minimum
           number of vertex indices that we may need and then we will realloc as
           the need comes up. */
        uint32_t texture_indice_num_per_position = face.number_of_textured_coordinate_indexes;

        face_out_reserved->vertex_indices =
            (uint32_t *)prc_calloc(ctx, num_indices, sizeof(uint32_t));
        if (face_out_reserved->vertex_indices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_out_reserved->vertex_indices\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }

        /* For the mapping tables we have to take the max of the indices and the
           number of vertices */
        prc_vertex_indice_to_api_vertex_indice =
            (uint32_t *)prc_calloc(ctx, tess->num_vertices_internal, sizeof(uint32_t));
        if (prc_vertex_indice_to_api_vertex_indice == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for prc_vertex_indice_to_api_vertex_indice\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }
        if (tess->num_normals_internal > 0)
        {
            prc_normal_indice_to_api_normal_indice =
                (uint32_t *)prc_calloc(ctx, tess->num_normals_internal, sizeof(uint32_t));
            if (prc_normal_indice_to_api_normal_indice == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for prc_normal_indice_to_api_normal_indice\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            prc_normal_indice_to_api_normal_indice = NULL;
        }

        if (has_texture)
        {
            /* Number of texture coordinates / 2 */
            prc_texture_indice_to_api_texture_indice =
                (uint32_t *) prc_calloc(ctx, tess->tess_3d->number_of_texture_coordinates / 2, sizeof(uint32_t));
            if (prc_texture_indice_to_api_texture_indice == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for prc_texture_indice_to_api_texture_indice\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            prc_texture_indice_to_api_texture_indice = NULL;
        }

        /* Initialize prc_vertex_indice_to_api_vertex_indice, and
           prc_normal_indice_to_api_normal_indice to UINT32MAX */
        for (k = 0; k < tess->num_vertices_internal; k++)
        {
            prc_vertex_indice_to_api_vertex_indice[k] = UINT32_MAX;
        }
        if (prc_normal_indice_to_api_normal_indice != NULL)
        {
            for (k = 0; k < tess->num_normals_internal; k++)
            {
                prc_normal_indice_to_api_normal_indice[k] = UINT32_MAX;
            }
        }
        if (has_texture)
        {
            for (j = 0; j < tess->tess_3d->number_of_texture_coordinates / 2; j++)
            {
                prc_texture_indice_to_api_texture_indice[j] = UINT32_MAX;
            }
        }
        /* Also make some temp structures to hold the vertex positions,
           the normal vectors, and the texture positions prior to any splitting. */
        if (tess->num_normals_internal > 0)
        {
            face_initial_normals = (float *)prc_calloc(ctx, num_indices * 3, sizeof(float));
            if (face_initial_normals == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_initial_normals\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            face_initial_normals = NULL;
        }

        face_initial_positions = (float *)prc_calloc(ctx, num_indices * 3, sizeof(float));
        if (face_initial_positions == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_initial_positions\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }

        if (has_texture)
        {
            /* This buffer is consumed as a single contiguous array in downstream
               decode helpers; allocating once avoids overwrite leaks. */
            (void)texture_indice_num_per_position;
            face_initial_texture_coords = (float *)prc_calloc(ctx, num_indices * 2, sizeof(float));
            if (face_initial_texture_coords == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_initial_texture_coords\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            face_initial_texture_coords = NULL;
        }

        /* Finally create the arrays to hold the new indices that will
           index into the above arrays */
        face_position_indices = (uint32_t *)prc_calloc(ctx, num_indices, sizeof(uint32_t));
        if (face_position_indices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_position_indices\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }
        if (tess->num_normals_internal > 0)
        {
            face_normal_indices = (uint32_t *)prc_calloc(ctx, num_indices, sizeof(uint32_t));
            if (face_normal_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_normal_indices\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            face_normal_indices = NULL;
        }
        if (has_texture)
        {
            face_texture_indices = (uint32_t *)prc_calloc(ctx, num_indices, sizeof(uint32_t));
            if (face_texture_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_texture_indices\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            face_texture_indices = NULL;
        }

        /* And then the special case where we have vertex colors defined */
        if (face.has_vertex_colors)
        {
            face_vertex_color_indices = (uint32_t *)prc_calloc(ctx, num_indices, sizeof(uint32_t));
            if (face_vertex_color_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for face_vertex_color_indices\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }
        }
        else
        {
            face_vertex_color_indices = NULL;
        }

        /* Create the table that will be used in finding common vertices
           as we create the single indice list that will reference those vertices.
           We build a list of lists where the lists on each list will store
           the vertex flavor (e.g. normal, color, texture) that exists at
           a particular location.  We can then search through that list for
           a match to one that we are wanting to add to our vertices. If we find
           it, then we use that one. Otherwise we add a new one to the list */
        tess->position_normal_lut.number_values = num_indices;
        tess->position_normal_lut.position_normal_pair =
            (prc_internal_api_position_normal_pair *)prc_calloc(ctx, num_indices,
            sizeof(prc_internal_api_position_normal_pair));
        if (tess->position_normal_lut.position_normal_pair == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for tess->position_normal_lut.position_normal_pair\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }

        /* Get position_normal_lut initialized */
        for (k = 0; k < num_indices; k++)
        {
            tess->position_normal_lut.position_normal_pair[k].api_vertex_index = UINT32_MAX;
            tess->position_normal_lut.position_normal_pair[k].prc_position_index = UINT32_MAX;
            tess->position_normal_lut.position_normal_pair[k].prc_normal_index = UINT32_MAX;
            tess->position_normal_lut.position_normal_pair[k].style_index = face_style_index;
            tess->position_normal_lut.position_normal_pair[k].next = NULL;
            tess->position_normal_lut.position_normal_pair[k].normal_set = PRC_INTERNAL_API_NORM_NOT_SET;
            tess->position_normal_lut.position_normal_pair[k].texture_set = PRC_INTERNAL_API_TEXTURE_NOT_SET;
        }

        face_vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx, num_indices,
            sizeof(prc_api_vertex));
        if (face_vertex_out->vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_vertex_out->vertices\n");
            code = PRC_API_ERROR_MEMORY;
            goto uncompressed_failure;
        }

        face_vertex_out->num_vertices = 0;
        face_vertex_out->capacity = num_indices;

        /* Go ahead and populate the vertex object based upon our parsing */
        face_out_reserved->num_indices = num_indices;
        face_out_reserved->capacity = num_indices;
        face_out->num_graphic_primitives = (entities_multiple_norms->num_triangles > 0) +
            entities_multiple_norms->num_fans + entities_multiple_norms->num_strips +
            (entities_one_norm->num_triangles > 0) + entities_one_norm->num_fans +
            entities_one_norm->num_strips + (entities_textured_multiple_norms->num_triangles > 0) +
            entities_textured_multiple_norms->num_fans + entities_textured_multiple_norms->num_strips +
            (entities_textured_one_norm->num_triangles > 0) +
            entities_textured_one_norm->num_fans + entities_textured_one_norm->num_strips;

        /* Do a sanity check here. Right now if we have to compute the normals,
           we only support that case where we have pure triangles. No strips or fans.
           Not sure if those even occur in a file. Per the spec it seems they could.
           TODO. Find out if this ever occurs */
        if (tess3d->must_calculate_normals)
        {
            if (entities_multiple_norms->num_fans > 0 || entities_multiple_norms->num_strips > 0 ||
                entities_textured_one_norm->num_fans > 0 || entities_textured_one_norm->num_strips > 0 ||
                entities_textured_multiple_norms->num_fans > 0 || entities_textured_multiple_norms->num_strips > 0 ||
                entities_one_norm->num_fans > 0 || entities_one_norm->num_strips > 0)
            {
                prc_error(ctx, PRC_API_ERROR_UNSUPPORTED,
                    "Normal recompute only supports pure triangles in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_UNSUPPORTED;
                goto uncompressed_failure;
            }
        }

        /* Also make sure that if we are in one of the texture types, that yes indeed
           we have a texture. If not, then the style is defined by biased_material_generic_index
           which we unbiased already and set it  in style->material_generic_index*/
        if (face_out->is_texture)
        {
            if (entities_multiple_norms->num_triangles > 0 || entities_multiple_norms->num_fans > 0 ||
                entities_multiple_norms->num_strips > 0 || entities_one_norm->num_triangles > 0 ||
                entities_one_norm->num_fans > 0 || entities_one_norm->num_strips > 0)
            {
                prc_error(ctx, PRC_API_ERROR_UNSUPPORTED,
                    "Mixed textured/non-textured entities are unsupported in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_UNSUPPORTED;
                goto uncompressed_failure;
            }
        }

        /* Now we must go through the entities and set the rest of the vertex data
           In the multi-normal case, we have a normal index followed by a position
           index for each face_out_p->num_indices. src_index_data points to the
           triangulated data at location [0].  We have offsets into that data stored
           in the entities so the order that we do these should not matter here */

           /* First pack the data of interest into a structure to make life a bit
              easier */
        uncompressed_data.src_index_data = &src_index_data;
        uncompressed_data.vertex_out = &face_out->face_vertices;  /* Use face-specific buffer */
        uncompressed_data.vertex_indices = face_out_reserved->vertex_indices;
        uncompressed_data.normals_internal = tess->normals_internal;
        uncompressed_data.position_normal_lut = tess->position_normal_lut;
        uncompressed_data.tess = tess;
        uncompressed_data.face_out_reserved = face_out_reserved;
        uncompressed_data.global_data = global_data;
        uncompressed_data.must_calculate_normals = tess3d->must_calculate_normals;
        uncompressed_data.is_texture = face_out->is_texture;
        uncompressed_data.is_material = face_out->is_material;
        uncompressed_data.edge_list = NULL;
        uncompressed_data.crease_angle = tess3d->crease_angle; /* Used in calc of normals */
        uncompressed_data.has_texture_transform = face_out->texture.has_transform;
        if (uncompressed_data.has_texture_transform)
        {
            memcpy(uncompressed_data.texture_transform, face_out->texture.transform, 9 * sizeof(double));
        }

        uncompressed_data.prc_vertex_indice_to_api_vertex_indice = prc_vertex_indice_to_api_vertex_indice;
        uncompressed_data.prc_normal_indice_to_api_normal_indice = prc_normal_indice_to_api_normal_indice;
        uncompressed_data.prc_texture_indice_to_api_texture_indice = prc_texture_indice_to_api_texture_indice;
        uncompressed_data.face_initial_normals = face_initial_normals;
        uncompressed_data.face_initial_texture_coords = face_initial_texture_coords;
        uncompressed_data.face_initial_positions = face_initial_positions;
        uncompressed_data.face_num_initial_positions = 0;
        uncompressed_data.face_num_initial_normals = 0;
        uncompressed_data.face_num_initial_texture_coords = 0;
        uncompressed_data.debug_tess_index = tess_index_in;
        uncompressed_data.debug_face_index = face_index;
        uncompressed_data.face_position_indices = face_position_indices;
        uncompressed_data.face_normal_indices = face_normal_indices;
        uncompressed_data.face_texture_indices = face_texture_indices;
        uncompressed_data.face_vertex_color_indices = face_vertex_color_indices;

        /* And decode the vertex colors if there are any */
        if (face.has_vertex_colors)
        {
            uint8_t has_alpha = face.vertex_colors.is_rgba;
            float *decoded_colors;
            uint32_t total_color_values = 0;
            uint32_t k;

            for (k = 0; k < face.size_of_triangulateddata; k++)
            {
                total_color_values += face.triangulateddata[k];
            }
            total_color_values = total_color_values * 3;  /* Three vertices per triangle */
            decoded_colors = (float *)prc_malloc(ctx, total_color_values * 4 * sizeof(float));
            if (decoded_colors == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for decoded vertex colors\n");
                code = PRC_API_ERROR_MEMORY;
                goto uncompressed_failure;
            }

            prc_internal_api_process_vertex_color_data(ctx, total_color_values,
                &face.vertex_colors.color_data, has_alpha, decoded_colors);
            uncompressed_data.decoded_colors = decoded_colors;
        }
        else
        {
            uncompressed_data.decoded_colors = NULL;
        }

        single_normal_set = 0;
        /* When we have mixed types (e.g. fan, triangle, strips we have to keep a running count of the index position */
        uncompressed_data.index_count = 0;

        /* This section is where all heavy work happens */
        /* Multiple norm non-textured cases */
        code = prc_internal_api_vertex_triangle_multinorm(ctx,
                            entities_multiple_norms->num_triangles, false,
                            &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_triangle_multinorm (non-textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_fan_multinorm(ctx,
            entities_multiple_norms->num_fans, entities_multiple_norms->fan_offsets,
            false, &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_fan_multinorm (non-textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_strip_multinorm(ctx,
            entities_multiple_norms->num_strips, entities_multiple_norms->strip_offsets,
            false, &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_strip_multinorm (non-textured)\n");
            goto uncompressed_failure;
        }

        /* Single norm case no texture */
        code = prc_internal_api_vertex_triangle_one_norm(ctx,
                                entities_one_norm->num_triangles, false,
                                &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_triangle_one_norm (non-textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_fan_one_norm(ctx,
                                    entities_one_norm->num_fans,
                                    entities_one_norm->fan_offsets, false,
                                    &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_fan_one_norm (non-textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_strip_one_norm(ctx,
                                entities_one_norm->num_strips,
                                entities_one_norm->strip_offsets, false,
                                &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_strip_one_norm (non-textured)\n");
            goto uncompressed_failure;
        }

        /* Multiple norm textured cases */
        code = prc_internal_api_vertex_triangle_multinorm(ctx,
            entities_textured_multiple_norms->num_triangles,
            true, &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_triangle_multinorm (textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_fan_multinorm(ctx,
            entities_textured_multiple_norms->num_fans,
            entities_textured_multiple_norms->fan_offsets,
            true, &uncompressed_data);

        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_fan_multinorm (textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_strip_multinorm(ctx,
            entities_textured_multiple_norms->num_strips,
            entities_textured_multiple_norms->strip_offsets,
            true, &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_strip_multinorm (textured)\n");
            goto uncompressed_failure;
        }

        /* Single norm textured case */
        code = prc_internal_api_vertex_triangle_one_norm(ctx,
            entities_textured_one_norm->num_triangles, true,
            &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_triangle_one_norm (textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_fan_one_norm(ctx,
            entities_textured_one_norm->num_fans,
            entities_textured_one_norm->fan_offsets, true,
            &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_fan_one_norm (textured)\n");
            goto uncompressed_failure;
        }

        code = prc_internal_api_vertex_strip_one_norm(ctx,
            entities_textured_one_norm->num_strips,
            entities_textured_one_norm->strip_offsets, true,
            &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_api_vertex_strip_one_norm (textured)\n");
            goto uncompressed_failure;
        }

        /* NOW we can deal with cleaning up and creating the vertices. We
           have to wait until we get through all the types */
        code = prc_internal_uncompressed_create_vertices(ctx, &uncompressed_data);
        if (code < 0)
        {
            prc_error(ctx, code,
                "Failed in prc_internal_uncompressed_create_vertices\n");
            goto uncompressed_failure;
        }

        /* If we have to compute the normals, do that now. This will end up doing
           vertex splitting as we encounter different normals for triangles that
           share an edge (assuming the edge angle is greater than the specified
           crease angle).  WARNING we really should only be here in the case
           where face.used_entities_flag has only triangle types (no fans or strips) */
        if (uncompressed_data.must_calculate_normals &&
            (face.used_entities_flag == PRC_FACETESSDATA_Triangle ||
             face.used_entities_flag == PRC_FACETESSDATA_TriangleTextured))
        {
            size_t num_triangles;

            if (face.used_entities_flag == PRC_FACETESSDATA_Triangle)
            {
                num_triangles = entities_multiple_norms->num_triangles;
            }
            else
            {
                num_triangles = entities_textured_multiple_norms->num_triangles;
            }
            code = prc_internal_api_calculate_normals_triangles(ctx, num_triangles,
                                                                &uncompressed_data);
            if (code < 0)
            {
                prc_error(ctx, code,
                    "Failed in prc_internal_api_calculate_normals_triangles\n");
                goto uncompressed_failure;
            }
        }
#if 0
        /* DEBUG. Print out all the vertex positions, normals and diffuse color */
        for (k = 0; k < face_vertex_out->num_vertices; k++)
        {
            printf("Vertex %d: Pos(%f, %f, %f) Normal(%f, %f, %f) Diffuse(%f, %f, %f) Color(%f, %f, %f, %f) Style(face_index=%d file_index=%d) tri_has_material = %d\n",
                k,
                face_vertex_out->vertices[k].position[0], face_vertex_out->vertices[k].position[1], face_vertex_out->vertices[k].position[2],
                face_vertex_out->vertices[k].normal[0], face_vertex_out->vertices[k].normal[1], face_vertex_out->vertices[k].normal[2],
                face_vertex_out->vertices[k].diffuse[0], face_vertex_out->vertices[k].diffuse[1], face_vertex_out->vertices[k].diffuse[2],
                face_vertex_out->vertices[k].color[0], face_vertex_out->vertices[k].color[1], face_vertex_out->vertices[k].color[2], face_vertex_out->vertices[k].color[3],
                face_vertex_out->vertices[k].style_index, face_vertex_out->vertices[k].style_file_index,
                face_vertex_out->vertices[k].tri_has_material);

            if (face_out->is_texture)
            {
                printf("    UV(%f, %f)\n", face_vertex_out->vertices[k].uv[0], face_vertex_out->vertices[k].uv[1]);
                printf(" Color = (%f, %f, %f)\n", face_vertex_out->vertices[k].color[0], face_vertex_out->vertices[k].color[1], face_vertex_out->vertices[k].color[2]);
            }
        }

        /* And now print out the indices */
        for (k = 0; k < face_out_reserved->num_indices; k++)
        {
            printf("Index %d: %d\n", k, face_out_reserved->vertex_indices[k]);
        }
#endif

        /* Free many items. */
        if (uncompressed_data.decoded_colors != NULL)
            prc_free(ctx, uncompressed_data.decoded_colors);
        if (prc_vertex_indice_to_api_vertex_indice != NULL)
            prc_free(ctx, prc_vertex_indice_to_api_vertex_indice);
        if (prc_normal_indice_to_api_normal_indice != NULL)
            prc_free(ctx, prc_normal_indice_to_api_normal_indice);
        if (prc_texture_indice_to_api_texture_indice != NULL)
            prc_free(ctx, prc_texture_indice_to_api_texture_indice);
        if (face_initial_normals != NULL)
            prc_free(ctx, face_initial_normals);
        if (face_initial_texture_coords != NULL)
            prc_free(ctx, face_initial_texture_coords);
        if (face_initial_positions != NULL)
            prc_free(ctx, face_initial_positions);
        if (face_position_indices != NULL)
            prc_free(ctx, face_position_indices);
        if (face_normal_indices != NULL)
            prc_free(ctx, face_normal_indices);
        if (face_texture_indices != NULL)
            prc_free(ctx, face_texture_indices);
        if (face_vertex_color_indices != NULL)
            prc_free(ctx, face_vertex_color_indices);
        if (tess->position_normal_lut.position_normal_pair != NULL)
        {
            for (j = 0; j < tess->position_normal_lut.number_values; j++)
            {
                prc_internal_api_position_normal_pair *curr =
                    tess->position_normal_lut.position_normal_pair[j].next;
                while (curr != NULL)
                {
                    prc_internal_api_position_normal_pair *next = curr->next;
                    prc_free(ctx, curr);
                    curr = next;
                }
            }
            prc_free(ctx, tess->position_normal_lut.position_normal_pair);
            tess->position_normal_lut.position_normal_pair = NULL;
        }

        goto uncompressed_done;

uncompressed_failure:
        if (code == 0)
            code = PRC_API_ERROR_PARSER;
        if (code == PRC_API_ERROR_MEMORY || code == PRC_ERROR_MEMORY)
        {
            if (face_vertex_out->vertices != NULL)
            {
                prc_free(ctx, face_vertex_out->vertices);
                face_vertex_out->vertices = NULL;
                face_vertex_out->num_vertices = 0;
                face_vertex_out->capacity = 0;
            }
            if (face_out_reserved != NULL && face_out_reserved->vertex_indices != NULL)
            {
                prc_free(ctx, face_out_reserved->vertex_indices);
                face_out_reserved->vertex_indices = NULL;
                face_out_reserved->num_indices = 0;
                face_out_reserved->capacity = 0;
            }
        }

        if (uncompressed_data.decoded_colors != NULL)
            prc_free(ctx, uncompressed_data.decoded_colors);
        if (prc_vertex_indice_to_api_vertex_indice != NULL)
            prc_free(ctx, prc_vertex_indice_to_api_vertex_indice);
        if (prc_normal_indice_to_api_normal_indice != NULL)
            prc_free(ctx, prc_normal_indice_to_api_normal_indice);
        if (prc_texture_indice_to_api_texture_indice != NULL)
            prc_free(ctx, prc_texture_indice_to_api_texture_indice);
        if (face_initial_normals != NULL)
            prc_free(ctx, face_initial_normals);
        if (face_initial_texture_coords != NULL)
            prc_free(ctx, face_initial_texture_coords);
        if (face_initial_positions != NULL)
            prc_free(ctx, face_initial_positions);
        if (face_position_indices != NULL)
            prc_free(ctx, face_position_indices);
        if (face_normal_indices != NULL)
            prc_free(ctx, face_normal_indices);
        if (face_texture_indices != NULL)
            prc_free(ctx, face_texture_indices);
        if (face_vertex_color_indices != NULL)
            prc_free(ctx, face_vertex_color_indices);
        if (tess->position_normal_lut.position_normal_pair != NULL)
        {
            for (j = 0; j < tess->position_normal_lut.number_values; j++)
            {
                prc_internal_api_position_normal_pair *curr =
                    tess->position_normal_lut.position_normal_pair[j].next;
                while (curr != NULL)
                {
                    prc_internal_api_position_normal_pair *next = curr->next;
                    prc_free(ctx, curr);
                    curr = next;
                }
            }
            prc_free(ctx, tess->position_normal_lut.position_normal_pair);
            tess->position_normal_lut.position_normal_pair = NULL;
        }
        prc_error(ctx, code,
            "Failure in uncompressed branch of prc_api_get_tessellation_vertices\n");
        goto failure;

uncompressed_done:
        ;
    }
    break;

    case PRC_TYPE_TESS_3D_Compressed:
    {
        /* If we are dealing with multiple faces in the compressed case, we want
           to do this a little different and just hand vertices that are relevant
           for this face and not all of the vertices. So lets do that as a special
           case */
        prc_tess_3d_compressed *tess =
            file_struct->tessellation->tess[tess_index].tess_3d_compressed;
        int num_prc_indices = tess->num_triangle_indices_prc_compressed_3d;
        uint8_t has_vertex_colors =
            (tess->decoded_point_color_array != NULL) ? true : false;
        /* Gated on is_multiple_line_attribute (real per-face/per-triangle
           style variation), not mere line_attribute_array != NULL -- see
           the identical fix/comment in prc_api_get_number_faces. A lone
           no-style placeholder entry (line_attribute_array_size == 1,
           value 0) must NOT be treated as "has a real style": doing so
           makes face_style_index resolve to a real style-table lookup at
           index 0 below instead of staying -1 ("no style, use the
           built-in default material" in prc_internal_api_get_style, which
           only special-cases style_index < 0, not == 0). */
        uint8_t has_style =
            tess->is_multiple_line_attribute ? true : false;
        uint8_t has_more_than_one_tri_style = false;
        uint32_t num_triangles = tess->triangle_face_array_size;
        prc_internal_api_color_state_t initial_color_state;
        prc_internal_api_style_state_t initial_style_state;
        uint32_t *prc_vertex_indices = tess->triangle_indices_prc_compressed_3d;
        uint32_t *prc_normal_indices = tess->normal_indices_prc_compressed_3d;
        int num_prc_vertices = tess->num_vertices_prc_compressed_3d;
        int num_prc_normals = tess->num_normals_prc_compressed_3d;
        prc_internal_api_position_normal_pair *position_normal_pair = NULL, *current;
        prc_internal_api_position_normal_pair *prev = NULL;
        prc_internal_api_position_normal_lookup *cache = NULL;
        size_t count = 0;
        int j, k;
        int got_color_match;
        int got_style_match;
        uint32_t vertex_index;
        uint32_t triangle_count = 0;
        uint32_t face_ref_index;
        uint32_t prc_position_index;
        uint32_t prc_normal_index;
        uint8_t all_tess_has_single_style = false;
        int32_t face_style_file_index;

        /* If the global data of this tessellation does not have any styles
           then we don't have any in the tessellation data. We can have
           reference data from another file (or even this one) */
        if (global_data->style_count == 0)
        {
            has_style = false;
        }

        if (has_style)
        {
            /* Check if we have more than one style */
            for (k = 1; k < num_triangles; k++)
            {
                if (tess->triangle_styles[k - 1] != tess->triangle_styles[k])
                {
                    has_more_than_one_tri_style = true;
                    break;
                }
            }
        }

        /* Note. If has_style == TRUE && has_more_than_one_tri_style == FALSE,
           then all the triangles have the same style! That is the easy case.
           Just set that as the DEFAULT style for every thing */

        /* The compressed tessellation contains only triangles. It does have
           the concept of faces, but they are not neccesarily contiguous in the
           encoded triangles which makes them some what useless for what we
           need here. Instead we will consider the compressed tessellation
           to have a single face */

        /* We are dealing with what may be one face of many.  Each face
           has its list of normal indices and vertex indices and potentially
           colors for each vertex OR style for each triangle. */

        /* The entire tessellation shares a set of vertex points and normal points
           that we currently index. We have to construct new API vertices
           that include the color, the normal, the style, and the spatial
           position though */

        if (face_out == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_PARAMETER,
                "face_out is NULL in compressed branch of prc_api_get_tessellation_vertices\n");
            code = PRC_API_ERROR_PARAMETER;
            goto compressed_failure;
        }

        /* We will be returning triangles, no other types of primatives */
        face_out->num_graphic_primitives = 1;

        /* We are not dealing with transparency at this stage */
        face_out->has_transparency = false;

        /* Get the vertices. We have to make our way through the triangle and normal
           indices, creating new indices for vertices that have a normal vector,
           position vector, and color (or texture), and style. We don't know how
           many that will take so we take a guess and increase the size as we
           work our way through the data */

        /* To do this, lets create a list of triangle indices and the normal indices
           that go with that position

            position(0) -> normal(3), normal(2), normal(5)
            position(1) -> normal(3), normal(0)
            etc.

            Then we will make our way through the list, creating a new vertex for
            each unique position/normal/color/style and an appropriate triangle indice.
        */

        /* Allocate the new indices. Same length as current one. This is
           per-face-owned (each face's tess_faces[j].reserved is a separate
           struct, freed individually in prc_api_release_data), so unlike
           the cache-guarded block below it must be (re)allocated on every
           call, not just the first. */
        face_out_reserved->vertex_indices = (uint32_t *)prc_calloc(ctx,
                                                num_prc_indices, sizeof(uint32_t));
        if (face_out_reserved->vertex_indices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for face_out_reserved->vertex_indices in compressed branch\n");
            code = PRC_API_ERROR_MEMORY;
            goto compressed_failure;
        }
        face_out_reserved->num_indices = num_prc_indices;
        face_out_reserved->capacity = num_prc_indices;

        /* Set a the color state and style state */
        if (has_vertex_colors)
        {
            initial_color_state = PRC_INTERNAL_API_COLOR_NOT_SET;
        }
        else
        {
            initial_color_state = PRC_INTERNAL_API_NO_COLOR;
        }

        /* Everything has the same style. This means no need to check,
           but we will set it to the face/ref style which should at
           least be the default */
        initial_style_state = PRC_INTERNAL_API_NO_STYLE;

        face_style_index = -1;
        face_style_file_index = -1;

        /* Style is coming directly from the tessellation data */
        if (has_more_than_one_tri_style)
        {
            initial_style_state = PRC_INTERNAL_API_STYLE_NOT_SET_FROM_TESS;
        }
        else
        {
            if (has_ref_style_defined)
            {
                face_style_index = face_out_reserved->style[0].face_style_index;
                face_style_file_index = face_out_reserved->style[0].face_style_file_index;
            }
            else if (has_style)
            {
                /* has_style (line_attribute_array != NULL) is what gates
                   the has_more_than_one_tri_style scan above that actually
                   reads triangle_styles -- this access needs the same
                   guard, or a compressed tessellation with no per-triangle
                   style data (line_attribute_array/triangle_styles both
                   NULL, e.g. every file this write facility produces,
                   which never emits per-triangle styles) reads
                   triangle_styles[0] out of a NULL array. face_style_index/
                   face_style_file_index are already initialized to -1
                   ("no style") above, so there is nothing to do in the
                   no-style case. */
                face_style_index = tess->triangle_styles[0];
                face_style_file_index = file_index;
            }
        }

        /* Style is coming directly from the reference data. This overrides
           the tessellation settings */
        if (has_more_than_one_ref_style)
        {
            initial_style_state = PRC_INTERNAL_API_STYLE_NOT_SET_FROM_REF_DATA;
        }

        if (has_more_than_one_ref_style || has_more_than_one_tri_style)
        {
            /* In this case all the triangles will need to have a style.
               To make the GPU code more efficient we will set a flag
               that will let the shader code to know to use the vertex
               style information */
            face_out->vertices_have_style = true;
        }
        else
        {
            face_out->vertices_have_style = false;
        }

        /* The decoded position/normal/color/style data built in this block is
           identical across every face_index of a GIVEN api_tess instance's
           compressed tessellation -- the compressed format doesn't cleanly
           separate per-face data, so the (uncached, always-run) loop below
           this one processes the whole tessellation's indices on every
           call regardless of face_index. It is NOT necessarily identical
           across DIFFERENT api_tess instances that happen to reference the
           same underlying PRC tess_index: style can be assigned by
           reference at the part/product-instance level
           (api_tess->style_leaf, consumed above via has_ref_style_defined/
           face_style_index), so two different instances of the same
           tessellation are free to carry different styles -- see the
           comment on prc_api_tess itself: "it turns out that we can have
           different realizations of the same tessellation with different
           styles". Cache this build on api_tess->reserved (a per-instance
           extension slot, already zeroed by prc_api_initialize_tessellation
           and otherwise unused by the compressed-tessellation type --
           reserved is only used by the mutually-exclusive wire/markup
           branches) so a later face_index call for the SAME instance reuses
           it, but a DIFFERENT instance referencing the same tess_index
           builds its own, independent copy.

           An earlier version of this optimization cached on the shared
           prc_tess struct instead (file_struct->tessellation->
           tess[tess_index].position_normal_lut), keyed by tess_index alone.
           That is the wrong scope: it is shared by every instance
           referencing the tessellation, so any instance after the first
           got an empty vertex_out->tess_vertices (this loop never ran for
           it) while its own face_out_reserved->vertex_indices was still
           fully populated by the loop below -- indices into an empty
           buffer. Reported as real parts rendering in the wrong location
           or not at all on assemblies with reused/instanced hardware
           (e.g. bolts/washers shared across many positions); see project
           history (branch disc-brake-rendering-regression). */
        /* api_tess->reserved may already be allocated (by the style-table
           cache earlier in this same function, on this same instance's
           first call) without the position/normal portion having been
           built yet -- position_normal_cache_valid (not merely `cache ==
           NULL`) is what actually signals that. See prc_internal_api_
           position_normal_lookup's own comment on that field for why. */
        cache = (prc_internal_api_position_normal_lookup *)api_tess->reserved;
        if (cache == NULL || !cache->position_normal_cache_valid)
        {
            /* Only free `cache` itself on a failure below if THIS block is
               the one that allocated it -- if it already existed (the
               style-table cache built it first, on this same instance's
               earlier/this call), it's still reachable from api_tess->
               reserved and owned by that cache's own lifetime, not this
               block's. */
            uint8_t cache_allocated_here = (cache == NULL);

            if (cache == NULL)
            {
                cache = (prc_internal_api_position_normal_lookup *)prc_calloc(ctx, 1,
                                            sizeof(prc_internal_api_position_normal_lookup));
                if (cache == NULL)
                {
                    prc_error(ctx, PRC_API_ERROR_MEMORY,
                        "Memory allocation failed for position_normal_lut cache in compressed branch\n");
                    code = PRC_API_ERROR_MEMORY;
                    goto compressed_failure;
                }
            }

            /* Create one of these for each vertex. Then each vertex will
               contain multiple normals in a list */
            position_normal_pair =
                (prc_internal_api_position_normal_pair *)prc_calloc(ctx, num_prc_vertices,
                                            sizeof(prc_internal_api_position_normal_pair));
            if (position_normal_pair == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for position_normal_pair in compressed branch\n");
                if (cache_allocated_here) prc_free(ctx, cache);
                code = PRC_API_ERROR_MEMORY;
                goto compressed_failure;
            }

            /* Allocate twice as many for the possible multiple normal, multiple color,
                situation. We may need to realloc this as we go along */
            vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx,
                                            num_prc_vertices * 2, sizeof(prc_api_vertex));
            if (vertex_out->vertices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for vertex_out->vertices in compressed branch\n");
                prc_free(ctx, position_normal_pair);
                if (cache_allocated_here) prc_free(ctx, cache);
                code = PRC_API_ERROR_MEMORY;
                goto compressed_failure;
            }

            vertex_out->num_vertices = num_prc_vertices;
            vertex_out->capacity = num_prc_vertices * 2;

            /* We will leave the normals unset. As we encounter a position, we will set
               the normal. If the normal is already set and different than the current
               one, we will add a new vertex and update the vertex array information.
               The assumption here in the compressed tessellation case, is that we have
               already computed the normals with the appropriate crease angle. This is
               in contrast to the uncompressed case where we have to still decode the
               indices and build the triangles to be able to compute the normals */
            for (k = 0; k < num_prc_vertices; k++)
            {
                prc_api_helper_set_vertex_position(ctx, vertex_out, k, tess, count);
                count += 3;

                // Set UV coordinates if texture data is available
                // (regardless of face_out->is_texture which may not be set yet)
                if (tess->uv_coordinates_3d != NULL)
                {
                    // Get raw UV coordinates from compressed tessellation data
                    double raw_u = tess->uv_coordinates_3d[k * 2];
                    double raw_v = tess->uv_coordinates_3d[k * 2 + 1];

                    // Apply transform and V-flip using helper function (correct order: raw -> transform -> flip)
                    // Note: face_out->texture may not be fully initialized yet, but transform should be available
                    prc_internal_api_set_vertex_texture_coords(ctx, &vertex_out->vertices[k], raw_u, raw_v,
                        (face_out && face_out->texture.has_transform) ? face_out->texture.transform : NULL,
                        (face_out && face_out->texture.has_transform) ? face_out->texture.has_transform : 0);

                    // Set color to white for textured vertices
                    vertex_out->vertices[k].color[0] = 1.0;
                    vertex_out->vertices[k].color[1] = 1.0;
                    vertex_out->vertices[k].color[2] = 1.0;
                    vertex_out->vertices[k].color[3] = 1.0;

                    // Store texture state in position_normal_pair
                    position_normal_pair[k].texture_set = PRC_INTERNAL_API_TEXTURE_SET;
                }
                else
                {
                    prc_api_helper_set_face_color(ctx, vertex_out, k, face_color,
                        position_normal_pair);
                }

                prc_api_helper_intialize_position_normal_pair(ctx,
                    position_normal_pair, k, initial_color_state, initial_style_state,
                    face_style_index, face_style_file_index);
            }

            /* Publish for reuse by later face_index calls for THIS SAME
               api_tess instance; freed once in prc_api_release_data. */
            cache->position_normal_pair = position_normal_pair;
            cache->number_values = (size_t)num_prc_vertices;
            cache->position_normal_cache_valid = 1;
            api_tess->reserved = (void *)cache;
        }
        else
        {
            /* Already built by an earlier face_index call for this SAME
               api_tess instance; vertex_out->vertices (api_tess->
               tess_vertices) was published then too and is
               tessellation-wide (for this instance), not per-face, so
               nothing else to rebuild here. */
            position_normal_pair = cache->position_normal_pair;
        }

        /* Now lets start going through the prc indices, assigning multiple normals to
           vertex positions and creating new vertex data that includes position,
           normal, and color (texture or vertex color) */
        for (k = 0; k < num_prc_indices; k++)
        {
            /* Get the position index in the PRC data */
            prc_position_index = prc_vertex_indices[k];

            /* Get the normal index in the PRC data */
            prc_normal_index = prc_normal_indices[k];

            /* Calculate which triangle we're in */
            uint32_t current_triangle = k / 3;  // Integer division

            /* Ensure triangle index is valid */
            if (current_triangle >= tess->triangle_face_array_size)
            {
                prc_error(ctx, PRC_API_ERROR_PARSER,
                    "triangle index exceeds triangle_face_array_size in compressed branch\n");
                printf("ERROR: triangle_count %u exceeds triangle_face_array_size %u\n",
                    current_triangle, tess->triangle_face_array_size);
                code = PRC_API_ERROR_PARSER;
                goto compressed_failure;
            }

            face_ref_index = tess->triangle_face_array[current_triangle];

            /* Validate face_ref_index against number_of_styles */
            if (face_ref_index >= face_out_reserved->number_of_styles)
            {
                printf("ERROR: face_ref_index %u exceeds number_of_styles %zu (triangle %u)\n",
                    face_ref_index, face_out_reserved->number_of_styles, current_triangle);
                printf("       num_prc_indices=%d, k=%d\n", num_prc_indices, k);
                /* Use default style or first style as fallback */
                face_ref_index = 0;
            }

            /* Check if a normal is already assigned to this position */
            if (position_normal_pair[prc_position_index].normal_set ==
                PRC_INTERNAL_API_NORM_NOT_SET)
            {
                /* No normals assigned yet, so just add this one. Other members
                   of this structure should already be set */
                prc_api_helper_set_normal(ctx, vertex_out, prc_position_index,
                    tess, position_normal_pair, prc_normal_index);

                /* If the color data is not set, do that now too */
                if (has_vertex_colors)
                {
                    if (position_normal_pair[prc_position_index].color_set ==
                        PRC_INTERNAL_API_COLOR_NOT_SET)
                    {
                        prc_api_helper_set_vertex_color(ctx, vertex_out,
                            prc_position_index, tess, k,
                            &position_normal_pair[prc_position_index]);
                    }
                }
                if (has_more_than_one_tri_style)
                {
                    /* In this case we are getting the style from the tessellation
                       data itself. */
                    if (position_normal_pair[prc_position_index].style_set ==
                        PRC_INTERNAL_API_STYLE_NOT_SET_FROM_TESS)
                    {
                        prc_api_helper_set_tri_style(ctx, global_data, file_index, header,
                            vertex_out, prc_position_index, tess, triangle_count,
                            &position_normal_pair[prc_position_index],
                            face_out_reserved, PRC_INTERNAL_API_STYLE_SET_FROM_TESS);
                    }
                }
                if (has_more_than_one_ref_style)
                {
                    /* In this case, each of the faces may have different styles
                       and we need to get it from those.  */
                    if (position_normal_pair[prc_position_index].style_set ==
                        PRC_INTERNAL_API_STYLE_NOT_SET_FROM_REF_DATA)
                    {
                        prc_api_helper_set_vertex_style_from_face_ref(ctx,
                            face_out->vertices_have_style,
                            vertex_out, prc_position_index, tess, triangle_count,
                            &position_normal_pair[prc_position_index],
                            face_out_reserved);
                    }
                }
                /* Nothing has been set yet. We should go ahead and use the
                   0th style that we parsed which will be the default color
                   if it had not style at all. If it does have a style then that
                   must be the same style for all the faces */
                if (position_normal_pair[prc_position_index].style_set == PRC_INTERNAL_API_NO_STYLE &&
                    position_normal_pair[prc_position_index].color_set == PRC_INTERNAL_API_NO_COLOR)
                {
                    if (has_ref_style_defined)
                    {
                        /* Use the style for this entire face get it from
                           the tessellation data. global data is the
                           proper place */
                        prc_api_helper_set_vertex_style_from_face_ref(ctx,
                            face_out->vertices_have_style, vertex_out,
                            prc_position_index, tess, triangle_count,
                            &position_normal_pair[prc_position_index],
                            face_out_reserved);
                    }
                    else
                    {
                        prc_api_helper_set_tri_style(ctx, global_data, file_index, header,
                            vertex_out, prc_position_index, tess, triangle_count,
                            &position_normal_pair[prc_position_index],
                            face_out_reserved, PRC_INTERNAL_API_STYLE_SET_FROM_TESS);
                    }
                }
            }
            else
            {
                /* There is at least one assigned there with a normal, check
                   through the list to see if we have a matching normal and if
                   we match on any of the other traits (e.g. color, style) */
                got_color_match = 0;
                got_style_match = 0;
                current = &position_normal_pair[prc_position_index];

                /* Check all the others in the list */
                while (current != NULL)
                {
                    if (current->prc_normal_index == prc_normal_index &&
                        current->prc_position_index == prc_position_index)
                    {
                        uint32_t api_vertex_index = current->api_vertex_index;

                        /* We have a match with the normal.  Check the color/style  */
                        if (has_vertex_colors)
                        {
                            if (current->color_set == PRC_INTERNAL_API_COLOR_NOT_SET)
                            {
                                /* Has not had its color set. We can grab this one
                                    and set its color now */
                                prc_api_helper_set_vertex_color(ctx, vertex_out,
                                    api_vertex_index, tess, k, current);
                                got_color_match = 1;
                            }
                            else
                            {
                                if (current->color_set != PRC_INTERNAL_API_COLOR_SET)
                                {
                                    prc_error(ctx, PRC_API_ERROR_PARSER,
                                        "Unexpected color_set state in compressed branch\n");
                                    code = PRC_API_ERROR_PARSER; /* Sanity check */
                                    goto compressed_failure;
                                }
                                /* Compare the color */
                                if (prc_api_helper_color_mismatch(ctx,
                                    vertex_out, api_vertex_index,
                                    tess, k))
                                {
                                    /* We have a color mismatch. Need to add a new vertex */
                                    got_color_match = 0;
                                }
                                else
                                {
                                    /* We have a match with the first one in the list */
                                    got_color_match = 1;
                                }
                            }
                        }
                        else
                        {
                            /* We have a match as there are no vertex colors */
                            got_color_match = 1;
                        }

                        /* Now lets deal with the style and see if we have a match */
                        if (has_more_than_one_tri_style)
                        {
                            if (current->style_set == PRC_INTERNAL_API_STYLE_NOT_SET_FROM_TESS)
                            {
                                prc_api_helper_set_tri_style(ctx, global_data, file_index,
                                    header, vertex_out, api_vertex_index, tess,
                                    triangle_count, current, face_out_reserved,
                                    PRC_INTERNAL_API_STYLE_SET_FROM_TESS);
                                got_style_match = 1;
                            }
                            else
                            {
                                if (current->style_set != PRC_INTERNAL_API_STYLE_SET_FROM_TESS)
                                {
                                    prc_error(ctx, PRC_API_ERROR_PARSER,
                                        "Unexpected style_set (tess) state in compressed branch\n");
                                    code = PRC_API_ERROR_PARSER; /* Sanity check */
                                    goto compressed_failure;
                                }
                                /* Compare the style */
                                if (tess->triangle_styles[triangle_count] !=
                                    vertex_out->vertices[api_vertex_index].style_index)
                                {
                                    /* We have a style mismatch. Need to add a new vertex */
                                    got_style_match = 0;
                                }
                                else
                                {
                                    /* We have a match with the first one in the list */
                                    got_style_match = 1;
                                }
                            }
                        }
                        if (has_more_than_one_ref_style)
                        {
                            if (current->style_set == PRC_INTERNAL_API_STYLE_NOT_SET_FROM_REF_DATA)
                            {
                                prc_api_helper_set_vertex_style_from_face_ref(ctx,
                                    face_out->vertices_have_style, vertex_out,
                                    api_vertex_index, tess, triangle_count,
                                    current, face_out_reserved);
                                got_style_match = 1;
                            }
                            else
                            {
                                if (current->style_set != PRC_INTERNAL_API_STYLE_SET_FROM_REF_DATA)
                                {
                                    prc_error(ctx, PRC_API_ERROR_PARSER,
                                        "Unexpected style_set (ref) state in compressed branch\n");
                                    code = PRC_API_ERROR_PARSER; /* Sanity check */
                                    goto compressed_failure;
                                }
                                /* Compare the style */
                                face_ref_index = tess->triangle_face_array[triangle_count];
                                if (face_out_reserved->style[face_ref_index].face_style_index !=
                                    vertex_out->vertices[api_vertex_index].style_index ||
                                    face_out_reserved->style[face_ref_index].face_style_file_index !=
                                    vertex_out->vertices[api_vertex_index].style_file_index)
                                {
                                    /* We have a style mismatch. Need to add a new vertex */
                                    got_style_match = 0;
                                }
                                else
                                {
                                    /* We have a match with the first one in the list */
                                    got_style_match = 1;
                                }
                            }
                        }
                        else
                        {
                            /* We have a match as there is only one style */
                            got_style_match = 1;
                        }

                        if (got_color_match && got_style_match)
                        {
                            /* We have a match on both color and style. No need to add a new vertex */
                            break; /* Break out of the while loop */
                        }
                    }
                    prev = current; /* Save the previous pointer */
                    current = current->next;
                }

                /* No match found. */
                if (!(got_color_match && got_style_match))
                {
                    /* We need to add this normal to the linked list. prev
                       already should be pointing to the end of the list */
                       /* Need to create a new one and add to the list */
                    prev->next =
                        (prc_internal_api_position_normal_pair *)prc_calloc(ctx, 1,
                            sizeof(prc_internal_api_position_normal_pair));
                    if (prev->next == NULL)
                    {
                        prc_error(ctx, PRC_API_ERROR_MEMORY,
                            "Memory allocation failed for position_normal_pair linked node\n");
                        code = PRC_API_ERROR_MEMORY;
                        goto compressed_failure;
                    }
                    prev->next->next = NULL;
                    prev->next->api_vertex_index = (uint32_t)vertex_out->num_vertices;
                    prev->next->normal_set = PRC_INTERNAL_API_NORM_SET;
                    prev->next->prc_normal_index = prc_normal_index;
                    prev->next->prc_position_index = prc_position_index;
                    face_ref_index = tess->triangle_face_array[triangle_count];

                    if (face_ref_index >= face_out_reserved->number_of_styles)
                    {
                        printf("WARNING: face_ref_index %u >= number_of_styles %zu, using 0\n",
                            face_ref_index, face_out_reserved->number_of_styles);
                        face_ref_index = 0;
                    }

                    prev->next->style_index =
                        face_out_reserved->style[face_ref_index].face_style_index;
                    prev->next->face_style_file_index =
                        face_out_reserved->style[face_ref_index].face_style_file_index;

                    /* Check if we need to realloc the vertex_out array */
                    if (vertex_out->num_vertices + 1 > vertex_out->capacity)
                    {
                        prc_api_vertex *new_vertices;
                        /* Double it */
                        vertex_out->capacity = vertex_out->capacity * 2;
                        new_vertices = (prc_api_vertex *)prc_realloc(ctx,
                            vertex_out->vertices,
                            vertex_out->capacity * sizeof(prc_api_vertex));
                        if (new_vertices == NULL)
                        {
                            prc_error(ctx, PRC_API_ERROR_MEMORY,
                                "Memory reallocation failed for vertex_out->vertices in compressed branch\n");
                            code = PRC_API_ERROR_MEMORY;
                            goto compressed_failure;
                        }
                        vertex_out->vertices = new_vertices;

                        /* Clear out any of the new items */
                        prc_internal_api_initialize_vertex(ctx, vertex_out);
                    }

                    /* Add the new vertex */
                    vertex_index = vertex_out->num_vertices;
                    vertex_out->num_vertices++;

                    vertex_out->vertices[vertex_index].position[0] = vertex_out->vertices[prc_position_index].position[0];
                    vertex_out->vertices[vertex_index].position[1] = vertex_out->vertices[prc_position_index].position[1];
                    vertex_out->vertices[vertex_index].position[2] = vertex_out->vertices[prc_position_index].position[2];
                    vertex_out->vertices[vertex_index].normal[0] = tess->normals_prc_compressed_3d[prc_normal_index * 3];
                    vertex_out->vertices[vertex_index].normal[1] = tess->normals_prc_compressed_3d[prc_normal_index * 3 + 1];
                    vertex_out->vertices[vertex_index].normal[2] = tess->normals_prc_compressed_3d[prc_normal_index * 3 + 2];

                    /* Either set the color for the vertex or use a texture or set a style */
                    if (face_out->is_texture)
                    {
                        // Get fresh raw UV coordinates for this duplicate
                        double raw_u = tess->uv_coordinates_3d[prc_position_index * 2];
                        double raw_v = tess->uv_coordinates_3d[prc_position_index * 2 + 1];

                        prc_internal_api_set_vertex_texture_coords(ctx, &vertex_out->vertices[vertex_index],
                            raw_u, raw_v, face_out->texture.transform, face_out->texture.has_transform);

                        vertex_out->vertices[vertex_index].color[0] = 1.0;
                        vertex_out->vertices[vertex_index].color[1] = 1.0;
                        vertex_out->vertices[vertex_index].color[2] = 1.0;
                        vertex_out->vertices[vertex_index].color[3] = 1.0;
                    }
                    else
                    {
                        if (has_vertex_colors)
                        {
                            vertex_out->vertices[vertex_index].color[0] = tess->decoded_point_color_array[k * 4];
                            vertex_out->vertices[vertex_index].color[1] = tess->decoded_point_color_array[k * 4 + 1];
                            vertex_out->vertices[vertex_index].color[2] = tess->decoded_point_color_array[k * 4 + 2];
                            vertex_out->vertices[vertex_index].color[3] = tess->decoded_point_color_array[k * 4 + 3];
                            prev->next->vertex_color[0] = tess->decoded_point_color_array[k * 4];
                            prev->next->vertex_color[1] = tess->decoded_point_color_array[k * 4 + 1];
                            prev->next->vertex_color[2] = tess->decoded_point_color_array[k * 4 + 2];
                            prev->next->vertex_color[3] = tess->decoded_point_color_array[k * 4 + 3];
                            prev->next->color_set = PRC_INTERNAL_API_COLOR_SET;
                        }
                        else
                        {
                            if (has_more_than_one_tri_style)
                            {
                                /* Set the style */
                                prc_api_helper_set_tri_style(ctx, global_data, file_index, header,
                                    vertex_out, vertex_index, tess, triangle_count,
                                    prev->next, face_out_reserved,
                                    PRC_INTERNAL_API_STYLE_SET_FROM_TESS);
                            }
                            else if (has_more_than_one_ref_style)
                            {
                                /* Set the style from the reference data */
                                prc_api_helper_set_vertex_style_from_face_ref(ctx,
                                    face_out->vertices_have_style, vertex_out,
                                    vertex_index, tess, triangle_count,
                                    prev->next, face_out_reserved);
                            }
                            else
                            {
                                if (has_ref_style_defined)
                                {
                                    /* Use the style for this entire face get it from
                                       the tessellation data. global data is the
                                       proper place */
                                       /* Set the face color */
                                      // vertex_out->vertices[vertex_index].color[0] = face_color[0];
                                     //  vertex_out->vertices[vertex_index].color[1] = face_color[1];
                                     //  vertex_out->vertices[vertex_index].color[2] = face_color[2];
                                    //   vertex_out->vertices[vertex_index].color[3] = face_color[3];

                                    prc_api_helper_set_vertex_style_from_face_ref(ctx,
                                        face_out->vertices_have_style, vertex_out,
                                        vertex_index, tess, triangle_count,
                                        prev->next, face_out_reserved);
                                }
                                else
                                {
                                    prc_api_helper_set_tri_style(ctx, global_data, file_index, header,
                                        vertex_out, vertex_index, tess, triangle_count,
                                        prev->next, face_out_reserved, PRC_INTERNAL_API_STYLE_SET_FROM_TESS);
                                }
                            }
                        }
                    }
                }
            }

            /* If we are on a multiple of 3, then increment triangle_count */
            if ((k + 1) % 3 == 0)
            {
                triangle_count++;
            }
        }

        /* So now that we have all the normal and position variants contained in
           position_normal_pair and all the vertex_out data set, lets step through
           the indices from the prc data and create new indices that go to the
           vertex_out values */
        for (k = 0; k < num_prc_indices; k++)
        {
            /* Get the position index in the PRC data */
            uint32_t prc_position_index = prc_vertex_indices[k];

            /* Get the normal index in the PRC data */
            uint32_t prc_normal_index = prc_normal_indices[k];

            /* Find the position in the position_normal_pair list */
            current = &position_normal_pair[prc_position_index];
            while (current->prc_normal_index != prc_normal_index)
            {
                current = current->next;
            }

            /* Now we have the correct position and normal index.  We can get the
                vertex index */
            face_out_reserved->vertex_indices[k] = current->api_vertex_index;

#if DEBUG_COMPRESSED_API_INDEX_MAP
            {
                uint32_t api_index = face_out_reserved->vertex_indices[k];
                if (api_index == DEBUG_COMPRESSED_API_INDEX_A ||
                    api_index == DEBUG_COMPRESSED_API_INDEX_C)
                {
                    printf("[CompressedMap] k=%d tri=%u prc_pos=%u prc_norm=%u -> api=%u (num_prc_vertices=%d num_api_vertices=%zu)\n",
                        k, (uint32_t)(k / 3), prc_position_index, prc_normal_index,
                        api_index, num_prc_vertices, vertex_out->num_vertices);
                }
            }
#endif
        }
#if 0
        /* DEBUG. Print out all the vertex positions, normals and diffuse color */
        for (k = 0; k < vertex_out->num_vertices; k++)
        {
            printf("Vertex %d: Pos(%f, %f, %f) Normal(%f, %f, %f) Diffuse(%f, %f, %f) Color(%f, %f, %f, %f) Style(face_index=%d file_index=%d) tri_has_material = %d\n",
                k,
                vertex_out->vertices[k].position[0], vertex_out->vertices[k].position[1], vertex_out->vertices[k].position[2],
                vertex_out->vertices[k].normal[0], vertex_out->vertices[k].normal[1], vertex_out->vertices[k].normal[2],
                vertex_out->vertices[k].diffuse[0], vertex_out->vertices[k].diffuse[1], vertex_out->vertices[k].diffuse[2],
                vertex_out->vertices[k].color[0], vertex_out->vertices[k].color[1], vertex_out->vertices[k].color[2], vertex_out->vertices[k].color[3],
                vertex_out->vertices[k].style_index, vertex_out->vertices[k].style_file_index,
                vertex_out->vertices[k].tri_has_material);

            if (face_out->is_texture)
            {
                printf("    UV(%f, %f)\n", vertex_out->vertices[k].uv[0], vertex_out->vertices[k].uv[1]);
                printf(" Color = (%f, %f, %f)\n", vertex_out->vertices[k].color[0], vertex_out->vertices[k].color[1], vertex_out->vertices[k].color[2]);
            }
        }

        /* And now print out the indices */
        for (k = 0; k < face_out_reserved->num_indices; k++)
        {
            printf("Index %d: %d\n", k, face_out_reserved->vertex_indices[k]);
        }
#endif

        /* Always the case for the compressed face when we do not have a texture */
        if (!face_out->is_texture && !has_vertex_colors)
        {
            face_out->vertices_have_style = true;
        }

        /* position_normal_pair is now owned by cache (either just published
           above, or reused from an earlier face_index call on this SAME
           api_tess instance) and persists across all of this instance's
           per-face calls -- freed once in prc_api_release_data, not here. */

        break;

compressed_failure:
        /* api_tess->reserved (if any) is unconditionally invalidated here,
           regardless of whether THIS call built it fresh or merely reused
           it from an earlier face_index call on this same instance: a
           failure partway through processing one face leaves no guarantee
           the cached state is consistent for any other face still to be
           processed on this instance. Use cache->number_values (not
           num_prc_vertices) for the cleanup bound in case this failure
           happens on a cache-hit call before num_prc_vertices for this
           call has been confirmed to match what built the cache -- for a
           conforming file the two always agree, but the cleanup should not
           assume that under an error condition. */
        if (api_tess->reserved != NULL)
        {
            prc_internal_api_position_normal_lookup *fail_cache =
                prc_tess_internal_position_normal_lookup(api_tess);

            if (fail_cache->position_normal_pair != NULL)
            {
                size_t pn;
                for (pn = 0; pn < fail_cache->number_values; pn++)
                {
                    prc_internal_api_position_normal_pair *temp;
                    current = fail_cache->position_normal_pair[pn].next;
                    while (current != NULL)
                    {
                        temp = current;
                        current = current->next;
                        prc_free(ctx, temp);
                    }
                }
                prc_free(ctx, fail_cache->position_normal_pair);
            }
            prc_free(ctx, fail_cache);
            api_tess->reserved = NULL;
        }
        position_normal_pair = NULL;
        goto failure;
    }
    case PRC_TYPE_TESS_Face:
        return PRC_API_ERROR_PARSER;
        break;
    case PRC_TYPE_TESS_3D_Wire:
    {
        prc_tess_3d_wire *tess =
            file_struct->tessellation->tess[tess_index].tess_3d_wire;
        uint32_t num_vertices = tess->tessellation_coordinates.number_of_coordinates / 3;
        float *vertex_colors = NULL;
        prc_internal_api_wire *wire = NULL;
        uint32_t wire_capacity = 0;
        uint32_t num_vertex_colors = 0;

        /* Lets allocate the space for the vertices */
        vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx, num_vertices, sizeof(prc_api_vertex));
        if (vertex_out->vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for wire vertex buffer in prc_api_get_tessellation_vertices\n");
            code = PRC_API_ERROR_MEMORY;
            goto wire_failure;
        }

        vertex_out->num_vertices = num_vertices;
        vertex_out->capacity = num_vertices;

        /* Set the base style if there is one */
        prc_internal_graph_style *wire_style = (prc_internal_graph_style*) api_tess->reserved2;
        if (wire_style != NULL)
        {
            prc_copy_internal_graph_style_to_api(ctx, wire_style, &api_tess->tess_material);
        }

        if (tess->has_vertex_colors)
        {
            num_vertex_colors = tess->vertex_color_count;
            vertex_colors = (float *)prc_calloc(ctx, num_vertex_colors * 4, sizeof(float));
            if (vertex_colors == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for wire vertex colors in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_MEMORY;
                goto wire_failure;
            }

            prc_internal_api_process_vertex_color_data(ctx, num_vertex_colors,
                &tess->vertex_color_data.color_data, tess->vertex_color_data.is_rgba,
                vertex_colors);
        }

        /* Positions, and a default color for every vertex (the same fallback
           used when there's no per-vertex color data at all: the wire's own
           style tint, or white). tess->vertex_color_count is a count of
           *wire-index entries actually visited by wire_elements* (see
           prc_parse_tess_3d_wire), not of raw positions -- a closed or
           multi-element wire can revisit the same position more than once,
           or a position can be unreferenced by any element, so vertex_colors
           cannot be indexed by raw position index k. Filling every vertex
           with this default first, then overwriting only the positions
           actually referenced by wire_elements below, keeps any unreferenced
           position sane instead of reading uninitialized/out-of-bounds data. */
        for (k = 0; k < num_vertices; k++)
        {
            vertex_out->vertices[k].position[0] = tess->tessellation_coordinates.coordinates[k * 3];
            vertex_out->vertices[k].position[1] = tess->tessellation_coordinates.coordinates[k * 3 + 1];
            vertex_out->vertices[k].position[2] = tess->tessellation_coordinates.coordinates[k * 3 + 2];

            if (wire_style != NULL)
            {
                vertex_out->vertices[k].color[0] = wire_style->tint[0];
                vertex_out->vertices[k].color[1] = wire_style->tint[1];
                vertex_out->vertices[k].color[2] = wire_style->tint[2];
                vertex_out->vertices[k].color[3] = wire_style->tint[3];
            }
            else
            {
                /* Default to white wires */
                vertex_out->vertices[k].color[0] = 1.0;
                vertex_out->vertices[k].color[1] = 1.0;
                vertex_out->vertices[k].color[2] = 1.0;
                vertex_out->vertices[k].color[3] = 1.0;
            }
        }

        /* Distribute the decoded colors to the vertices they actually belong
           to, walking wire_elements in exactly the order prc_parse_tess_3d_wire
           counted them in (per-element, per-index, plus one extra trailing
           entry per closing element) -- the only order that keeps color_idx
           aligned with vertex_colors. wire_indexes[] values are coordinate
           offsets (already seen divided by 3 elsewhere in this function, e.g.
           the wire-primitive index copies below), not vertex indices
           directly. A closing element's extra color entry has no
           corresponding wire index -- it's the color of the implicit closing
           segment back to the element's start, which has no home in a
           per-vertex color model, so it's applied to that start vertex (the
           closest reasonable per-vertex approximation) and otherwise just
           consumed to keep later elements' colors aligned. */
        if (tess->has_vertex_colors && tess->number_of_wire_indexes > 0)
        {
            uint32_t e, color_idx = 0;

            for (e = 0; e < tess->number_of_wire_elements; e++)
            {
                prc_tess_3d_wire_element *elem = &tess->wire_elements[e];

                for (j = 0; j < elem->number_of_wire_indexes; j++)
                {
                    uint32_t vidx = elem->wire_indexes[j] / 3;

                    if (color_idx < num_vertex_colors && vidx < num_vertices)
                    {
                        vertex_out->vertices[vidx].color[0] = vertex_colors[color_idx * 4];
                        vertex_out->vertices[vidx].color[1] = vertex_colors[color_idx * 4 + 1];
                        vertex_out->vertices[vidx].color[2] = vertex_colors[color_idx * 4 + 2];
                        vertex_out->vertices[vidx].color[3] = vertex_colors[color_idx * 4 + 3];
                    }
                    color_idx++;
                }

                if (elem->is_connected & PRC_3DWIRETESSDATA_IsClosing)
                {
                    if (color_idx < num_vertex_colors && elem->number_of_wire_indexes > 0)
                    {
                        uint32_t start_vidx = elem->wire_indexes[0] / 3;

                        if (start_vidx < num_vertices)
                        {
                            vertex_out->vertices[start_vidx].color[0] = vertex_colors[color_idx * 4];
                            vertex_out->vertices[start_vidx].color[1] = vertex_colors[color_idx * 4 + 1];
                            vertex_out->vertices[start_vidx].color[2] = vertex_colors[color_idx * 4 + 2];
                            vertex_out->vertices[start_vidx].color[3] = vertex_colors[color_idx * 4 + 3];
                        }
                    }
                    color_idx++;
                }
            }
        }

        if (vertex_colors != NULL)
            prc_free(ctx, vertex_colors);

        /* If number_of_wire_indexes is zero the tessellation coordinates represent
           a single wire edge. If not, we then have a sequence that is the number of
           indices per wire followed by the indices for that wire. The indices
           index into the the tessellation vertices. For each wire, we also
           have a flag that indicates if the wire is closed, or is linked to the
           preceeding wire. This flag will set up our primitive type */

         /* The exact number of primitives will depend upon if the wire is a
            continuation of the previous one */

        /* First deal with the case where we just use the vertices that we have
           directly. */
        if (tess->number_of_wire_indexes == 0)
        {
            api_tess->num_line_primitives = 1;

            /* We have a single wire edge */
            wire_capacity = 1;
            wire = (prc_internal_api_wire *)prc_calloc(ctx, 1,
                                            sizeof(prc_internal_api_wire));
            if (wire == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for wire primitive array in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_MEMORY;
                goto wire_failure;
            }

            wire->num_indices = tess->tessellation_coordinates.number_of_coordinates / 3;
            wire->vertex_indices = (uint32_t *)prc_calloc(ctx, wire->num_indices, sizeof(uint32_t));
            if (wire->vertex_indices == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for wire->vertex_indices in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_MEMORY;
                goto wire_failure;
            }

            /* Copy the indices */
            for (k = 0; k < wire->num_indices; k++)
            {
                wire->vertex_indices[k] = k;
            }

            /* Set the type */
            if (wire->num_indices == 2)
                wire->type = PRC_API_LINE;
            else
                wire->type = PRC_API_LINE_STRIP;

            /* Set the reserved data */
            api_tess->reserved = (void *)wire;
        }
        else
        {
            /* We have multiple wires. We will have to go through the indices and
               determine if the wire is a continuation of the previous one or if any
               are loops */
            uint32_t wire_count = 0;

            /* For now allocate enough to hold the twice the number of wire indexes.
               This is sufficient to hold all the primitives if they are all connected */
            wire_capacity = 2 * tess->number_of_wire_indexes;
            wire = (prc_internal_api_wire *)prc_calloc(ctx,
                                                    2 * tess->number_of_wire_indexes,
                                                    sizeof(prc_internal_api_wire));
            if (wire == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for wire primitive array in multi-wire case\n");
                code = PRC_API_ERROR_MEMORY;
                goto wire_failure;
            }

            for (k = 0; k < tess->number_of_wire_elements; k++)
            {
                prc_tess_3d_wire_element *wire_elements = &tess->wire_elements[k];
                uint32_t num_indices = wire_elements->number_of_wire_indexes;

                if (wire_elements->is_connected & PRC_3DWIRETESSDATA_IsClosing)
                {
                    wire[wire_count].num_indices = num_indices;
                    wire[wire_count].vertex_indices = (uint32_t *)prc_calloc(ctx,
                                                    num_indices, sizeof(uint32_t));
                    if (wire[wire_count].vertex_indices == NULL)
                    {
                        prc_error(ctx, PRC_API_ERROR_MEMORY,
                            "Memory allocation failed for closing wire indices\n");
                        code = PRC_API_ERROR_MEMORY;
                        goto wire_failure;
                    }

                    /* Copy the indices and divide them each by 3 */
                    for (j = 0; j < num_indices; j++)
                    {
                        wire[wire_count].vertex_indices[j] = wire_elements->wire_indexes[j] / 3;
                    }

                    /* Set the type */
                    wire[wire_count].type = PRC_API_LINE_LOOP;
                }
                else if (wire_elements->is_connected & PRC_3DWIRETESSDATA_IsContinuous)
                {
                    /* In addition to the current line, we need to add an additional
                       one to connect the to the previous line */
                    wire[wire_count].num_indices = num_indices;
                    wire[wire_count].capacity = num_indices;
                    wire[wire_count].vertex_indices = (uint32_t *)prc_calloc(ctx,
                                                    num_indices, sizeof(uint32_t));
                    if (wire[wire_count].vertex_indices == NULL)
                    {
                        prc_error(ctx, PRC_API_ERROR_MEMORY,
                            "Memory allocation failed for continuous wire indices\n");
                        code = PRC_API_ERROR_MEMORY;
                        goto wire_failure;
                    }

                    /* Copy the indices and divide them each by 3 */
                    for (j = 0; j < num_indices; j++)
                    {
                        wire[wire_count].vertex_indices[j] = wire_elements->wire_indexes[j] / 3;
                    }

                    /* Set the type */
                    if (num_indices == 2)
                        wire[wire_count].type = PRC_API_LINE;
                    else
                        wire[wire_count].type = PRC_API_LINE_STRIP;

                    /* Add the additional line */
                    wire_count++;
                    wire[wire_count].num_indices = 2;
                    wire[wire_count].capacity = 2;
                    wire[wire_count].vertex_indices = (uint32_t *)prc_calloc(ctx,
                                                                2, sizeof(uint32_t));
                    if (wire[wire_count].vertex_indices == NULL)
                    {
                        prc_error(ctx, PRC_API_ERROR_MEMORY,
                            "Memory allocation failed for connecting wire indices\n");
                        code = PRC_API_ERROR_MEMORY;
                        goto wire_failure;
                    }

                    /* Copy the indices */
                    prc_tess_3d_wire_element *prev_wire_elements = &tess->wire_elements[k - 1];
                    uint32_t prev_num_indices = prev_wire_elements->number_of_wire_indexes;
                    wire[wire_count].vertex_indices[0] = prev_wire_elements->wire_indexes[prev_num_indices - 1];
                    wire[wire_count].vertex_indices[1] = wire[wire_count - 1].vertex_indices[0];

                    /* Set the type */
                    wire[wire_count].type = PRC_API_LINE;
                }
                else
                {
                    /* Nothing special here. If it is two points only then do the line
                       type, else do the strip type */
                    wire[wire_count].num_indices = num_indices;
                    wire[wire_count].capacity = num_indices;
                    wire[wire_count].vertex_indices = (uint32_t *)prc_calloc(ctx,
                                                    num_indices, sizeof(uint32_t));
                    if (wire[wire_count].vertex_indices == NULL)
                    {
                        prc_error(ctx, PRC_API_ERROR_MEMORY,
                            "Memory allocation failed for wire strip indices\n");
                        code = PRC_API_ERROR_MEMORY;
                        goto wire_failure;
                    }

                    /* Copy the indices and divide them each by 3 */
                    for (j = 0; j < num_indices; j++)
                    {
                        wire[wire_count].vertex_indices[j] = wire_elements->wire_indexes[j] / 3;
                    }

                    /* Set the type */
                    if (num_indices == 2)
                        wire[wire_count].type = PRC_API_LINE;
                    else
                        wire[wire_count].type = PRC_API_LINE_STRIP;
                }
                wire_count++;
            }
            /* Set the reserved data */
            api_tess->num_line_primitives = wire_count;
            api_tess->reserved = (void *)wire;
        }
        break;

wire_failure:
        if (vertex_colors != NULL)
        {
            prc_free(ctx, vertex_colors);
            vertex_colors = NULL;
        }

        if (wire != NULL)
        {
            for (k = 0; k < wire_capacity; k++)
            {
                if (wire[k].vertex_indices != NULL)
                {
                    prc_free(ctx, wire[k].vertex_indices);
                    wire[k].vertex_indices = NULL;
                }
            }
            prc_free(ctx, wire);
            wire = NULL;
        }

        if (vertex_out != NULL && vertex_out->vertices != NULL)
        {
            prc_free(ctx, vertex_out->vertices);
            vertex_out->vertices = NULL;
            vertex_out->num_vertices = 0;
            vertex_out->capacity = 0;
        }

        api_tess->num_line_primitives = 0;
        api_tess->reserved = NULL;
        goto failure;
    }
    case PRC_TYPE_TESS_MarkUp:
    {
        /* We already created the primitives for this in prc_decode_markup_tess.c
           We just need to convert to the API primitives */
        prc_tess_markup *tess = file_struct->tessellation->tess[tess_index].tess_markup;
        uint32_t num_vertices = tess->decode_num_vertices;
        prc_internal_api_wire *wire = NULL;
        uint32_t line_prim_count = 0;
        uint32_t text_prim_count = 0;
        float markup_color[4] = { 0.0, 0.0, 0.0, 1.0 };

        if (num_vertices == 0)
        {
            return 0;
        }

        /* Lets allocate the space for the vertices */
        vertex_out->vertices = (prc_api_vertex *)prc_calloc(ctx, num_vertices, sizeof(prc_api_vertex));
        if (vertex_out->vertices == NULL)
        {
            prc_error(ctx, PRC_API_ERROR_MEMORY,
                "Memory allocation failed for markup vertex buffer in prc_api_get_tessellation_vertices\n");
            code = PRC_API_ERROR_MEMORY;
            goto markup_failure;
        }

        vertex_out->num_vertices = num_vertices;
        vertex_out->capacity = num_vertices;

        api_tess->num_line_primitives = 0;
         /* We can have line primitives or a text */
        for (k = 0; k < tess->decode_number_primitives; k++)
        {
            if (tess->decode_primitives[k].primitive_type != MARKUP_TEXT)
            {
                api_tess->num_line_primitives++;
            }
        }
        api_tess->num_text_primitives = tess->decode_number_primitives - api_tess->num_line_primitives;

        if (api_tess->num_text_primitives > 0)
        {
            api_tess->text_primitives = (prc_api_text_primitive *)prc_calloc(ctx,
                api_tess->num_text_primitives, sizeof(prc_api_text_primitive));
            if (api_tess->text_primitives == NULL)
            {
                prc_error(ctx, PRC_API_ERROR_MEMORY,
                    "Memory allocation failed for markup text primitives in prc_api_get_tessellation_vertices\n");
                code = PRC_API_ERROR_MEMORY;
                goto markup_failure;
            }
        }

        if (api_tess->num_line_primitives)
        {
            /* Allocate the space for the primitives */
             wire = (prc_internal_api_wire *)prc_calloc(ctx,
                 tess->decode_number_primitives,
                 sizeof(prc_internal_api_wire));
             if (wire == NULL)
             {
                 prc_error(ctx, PRC_API_ERROR_MEMORY,
                     "Memory allocation failed for markup wire primitives in prc_api_get_tessellation_vertices\n");
                 code = PRC_API_ERROR_MEMORY;
                 goto markup_failure;
             }
        }

        /* At this point, we are going to assume that all these primitives have
           the same color. That may not be true, and we will need to do
           some vertex splitting if that ends up being an issue. For now, lets
           go through the line primitives and get the color and see if there
           are multiple ones. I suspect that is very rare. Generally a markup
           will just be a single solid color */
        for (k = 0; k < tess->decode_number_primitives; k++)
        {
            if (tess->decode_primitives[k].primitive_type != MARKUP_TEXT)
            {
                uint8_t found_color = 0;

                code = prc_internal_api_get_color(ctx, global_data,
                    ((int32_t)tess->decode_primitives[k].biased_color_index) - 1,
                    markup_color);
                if (code < 0)
                {
                    prc_error(ctx, PRC_API_ERROR_PARAMETER, "Failed to get face style in prc_api_get_tessellation_vertices\n");
                    goto markup_failure;
                }
            }
        }

        /* Lets set the vertex locations and color */
        for (k = 0; k < num_vertices; k++)
        {
            vertex_out->vertices[k].position[0] = tess->decode_vertices[k].x;
            vertex_out->vertices[k].position[1] = tess->decode_vertices[k].y;
            vertex_out->vertices[k].position[2] = tess->decode_vertices[k].z;

            vertex_out->vertices[k].color[0] = markup_color[0];
            vertex_out->vertices[k].color[1] = markup_color[1];
            vertex_out->vertices[k].color[2] = markup_color[2];
            vertex_out->vertices[k].color[3] = 1.0;
        }

        for (k = 0; k < tess->decode_number_primitives; k++)
        {
            prc_markup_serialization_helper serialize_help = global_data->serialize_help;

            if (tess->decode_primitives[k].primitive_type == MARKUP_TEXT)
            {
                /* Get the string and the color */
                api_tess->text_primitives[text_prim_count].text =
                    tess->decode_primitives[k].text;

                if (tess->decode_primitives[k].biased_color_index == 0)
                {
                    api_tess->text_primitives[text_prim_count].color[0] = 0.0;
                    api_tess->text_primitives[text_prim_count].color[1] = 0.0;
                    api_tess->text_primitives[text_prim_count].color[2] = 0.0;
                }
                else
                {
                    uint32_t color_index = (tess->decode_primitives[k].biased_color_index - 1)/3;
                    if (color_index > global_data->color_count)
                    {
                        prc_error(ctx, PRC_API_ERROR_PARSER,
                            "Markup text color index out of range in prc_api_get_tessellation_vertices\n");
                        code = PRC_API_ERROR_PARSER;
                        goto markup_failure;
                    }
                    api_tess->text_primitives[text_prim_count].color[0] =
                        global_data->colors[color_index].red;
                    api_tess->text_primitives[text_prim_count].color[1] =
                        global_data->colors[color_index].green;
                    api_tess->text_primitives[text_prim_count].color[2] =
                        global_data->colors[color_index].blue;
                }

                /* Deal with dimensions */
                api_tess->text_primitives[text_prim_count].origin[0] =
                    tess->decode_primitives[k].face_frame_draw_origin.x;
                api_tess->text_primitives[text_prim_count].origin[1] =
                    tess->decode_primitives[k].face_frame_draw_origin.y;
                api_tess->text_primitives[text_prim_count].origin[2] =
                    tess->decode_primitives[k].face_frame_draw_origin.z;
                api_tess->text_primitives[text_prim_count].text_width =
                    tess->decode_primitives[k].text_width;
                api_tess->text_primitives[text_prim_count].text_height =
                    tess->decode_primitives[k].text_height;
                api_tess->text_primitives[text_prim_count].mode =
                    (prc_api_text_block_mode_t) tess->decode_primitives[k].block_mode;

                /* Now the font */

                text_prim_count++;
            }
            else
            {
                wire[line_prim_count].num_indices = tess->decode_primitives[k].number_indices;
                wire[line_prim_count].vertex_indices = (uint32_t *)prc_calloc(ctx,
                    wire[line_prim_count].num_indices, sizeof(uint32_t));
                if (wire[line_prim_count].vertex_indices == NULL)
                {
                    prc_error(ctx, PRC_API_ERROR_MEMORY,
                        "Memory allocation failed for markup line primitive indices\n");
                    code = PRC_API_ERROR_MEMORY;
                    goto markup_failure;
                }

                memcpy(wire[line_prim_count].vertex_indices, tess->decode_primitives[k].indices,
                    wire[line_prim_count].num_indices * sizeof(uint32_t));

                /* Set the type */
                if (wire[line_prim_count].num_indices == 2)
                    wire[line_prim_count].type = PRC_API_LINE_STRIP;
                else
                {
                    if (tess->decode_primitives[k].primitive_type == MARKUP_LINE_LOOP)
                    {
                        wire[line_prim_count].type = PRC_API_LINE_LOOP;
                    }
                    else if (tess->decode_primitives[k].primitive_type == MARKUP_TRIANGLE)
                    {
                        wire[line_prim_count].type = PRC_API_TRIANGLES;
                    }
                    else
                    {
                        wire[line_prim_count].type = PRC_API_LINE_STRIP;
                    }
                }
                line_prim_count++;
            }
        }
        /* Set the reserved data */
        api_tess->reserved = (void *)wire;

        break;

markup_failure:
        if (wire != NULL)
        {
            for (j = 0; j < tess->decode_number_primitives; j++)
            {
                if (wire[j].vertex_indices != NULL)
                {
                    prc_free(ctx, wire[j].vertex_indices);
                    wire[j].vertex_indices = NULL;
                }
            }
            prc_free(ctx, wire);
            wire = NULL;
        }

        if (api_tess->text_primitives != NULL)
        {
            prc_free(ctx, api_tess->text_primitives);
            api_tess->text_primitives = NULL;
        }
        api_tess->num_text_primitives = 0;

        if (vertex_out != NULL && vertex_out->vertices != NULL)
        {
            prc_free(ctx, vertex_out->vertices);
            vertex_out->vertices = NULL;
            vertex_out->num_vertices = 0;
            vertex_out->capacity = 0;
        }

        api_tess->num_line_primitives = 0;
        api_tess->reserved = NULL;
        goto failure;
    }

    default:
        PRC_API_FAIL(PRC_API_ERROR_PARSER,
            "Unsupported tessellation type in prc_api_get_tessellation_vertices\n");
    }

#undef PRC_API_FAIL
    return 0;

failure:
    if (has_face && face_out != NULL)
    {
        if (face_out->reserved != NULL)
        {
            face_out_reserved = prc_face_internal_face(face_out);
            if (face_out_reserved != NULL)
            {
                /* owns_style == 0 means `style` is a borrowed pointer into
                   the per-instance style cache (api_tess->reserved) --
                   freeing it here would corrupt that cache for any other
                   face_index call still to come on this same instance, or
                   double-free if it's already been published there. It's
                   freed once, with the cache, in prc_api_release_data. */
                if (face_out_reserved->style != NULL && face_out_reserved->owns_style)
                {
                    prc_free(ctx, face_out_reserved->style);
                    face_out_reserved->style = NULL;
                }
            }
            prc_free(ctx, face_out->reserved);
            face_out->reserved = NULL;
        }

        if (face_out->face_vertices.vertices != NULL)
        {
            prc_free(ctx, face_out->face_vertices.vertices);
            face_out->face_vertices.vertices = NULL;
            face_out->face_vertices.num_vertices = 0;
            face_out->face_vertices.capacity = 0;
        }
    }

    if (vertex_out != NULL && vertex_out->vertices != NULL)
    {
        prc_free(ctx, vertex_out->vertices);
        vertex_out->vertices = NULL;
        vertex_out->num_vertices = 0;
        vertex_out->capacity = 0;
    }

    return code;
}

PRC_EXPORT uint32_t
prc_api_number_of_materials(prc_context *ctx, prc_api_data data_in, const prc_api_tess *tess)
{
    uint32_t num_faces = tess->num_faces;
    prc_api_tess_type_t tess_type;

    tess_type = tess->type;

    if (tess_type == PRC_API_TESS_3D_Wire || tess_type == PRC_API_TESS_MarkUp)
    {
        return 0;
    }

    /* Just return the number of faces. We will either have the default material
       or the encoded material */
    return num_faces;
}

PRC_EXPORT size_t
prc_api_get_num_graphics_primitives(prc_context *ctx, prc_api_data data_in,
                                    const prc_api_tess *tess)
{
    prc_data *data = (prc_data *)data_in;
    prc_api_face face;
    uint32_t num_faces = tess->num_faces;
    uint32_t k;
    size_t num_objects = 0;
    prc_api_tess_type_t tess_type;

    /* If this is a compressed tessellation we only have one object which is
       triangles */
    tess_type = tess->type;

    if (tess_type == PRC_API_TESS_3D_Compressed)
    {
        return 1;
    }

    if (tess_type == PRC_API_TESS_3D_Wire || tess_type == PRC_API_TESS_MarkUp)
    {
        //return tess->num_line_primitives + tess->num_text_primitives;
        return tess->num_line_primitives;
    }

    for (k = 0; k < num_faces; k++)
    {
        face = tess->tess_faces[k];
        num_objects += face.num_graphic_primitives;
    }
    return num_objects;
}

PRC_EXPORT int
prc_api_get_text_primitive(prc_context *ctx, prc_api_data data,
    const prc_api_tess *tess, uint32_t text_index,
    prc_api_text_primitive *text_primitive)
{
    if (tess->type != PRC_API_TESS_MarkUp)
        return PRC_API_ERROR_PARAMETER;

    if (text_index > tess->num_text_primitives - 1)
        return PRC_API_ERROR_PARAMETER;

    memcpy(text_primitive, &tess->text_primitives[text_index],
                                                    sizeof(prc_api_text_primitive));
    return 0;
}

PRC_EXPORT int
prc_api_get_face_vertices(prc_context *ctx, const prc_api_tess *tess,
    uint32_t face_index, uint32_t *vertex_count, prc_api_vertex **vertices)
{
    prc_api_tess_type_t tess_type;

    tess_type = tess->type;

    if (face_index > tess->num_faces - 1)
        return PRC_API_ERROR_PARAMETER;

    if (tess_type == PRC_API_TESS_3D_Compressed ||
        tess_type == PRC_API_TESS_3D_Wire ||
        tess_type == PRC_API_TESS_MarkUp)
    {
        *vertices = tess->tess_vertices.vertices;
        *vertex_count = tess->tess_vertices.num_vertices;
    }
    else if (tess_type == PRC_API_TESS_3D ||
        tess_type == PRC_API_TESS_3D_Wire_Extra)
    {
        *vertices = tess->tess_faces[face_index].face_vertices.vertices;
        *vertex_count = tess->tess_faces[face_index].face_vertices.num_vertices;
    }
    else
    {
        return PRC_API_ERROR_PARSER;
    }
    return 0;
}

PRC_EXPORT int
prc_api_get_graphics_primitive(prc_context *ctx, prc_api_data data_in,
        const prc_api_tess *tess, uint32_t face_index, size_t graphics_index,
        prc_api_graphic_primitive *graphics_object)
{
    prc_data *data = (prc_data *)data_in;
    prc_internal_api_face *face;
    size_t k;
    prc_internal_api_tess_entities *entity;
    prc_api_tess_type_t tess_type;

    /* If this is a compressed tessellation we only have one object which is triangles */
    tess_type = tess->type;

    if (tess_type == PRC_API_TESS_3D_Compressed)
    {
        face = prc_face_internal_face(&tess->tess_faces[face_index]);

        graphics_object->type = PRC_API_TRIANGLES;
        graphics_object->num_indices = face->num_indices;
        graphics_object->indices = face->vertex_indices;
        return 0;
    }

    if (tess_type == PRC_API_TESS_3D_Wire || tess_type == PRC_API_TESS_MarkUp)
    {
        prc_internal_api_wire *wire = prc_tess_internal_wire(tess);
        if (graphics_index > tess->num_line_primitives - 1)
            return PRC_API_ERROR_PARAMETER;
        graphics_object->type = wire[graphics_index].type;
        graphics_object->num_indices = wire[graphics_index].num_indices;
        graphics_object->indices = wire[graphics_index].vertex_indices;
        return 0;
    }

    if (tess_type == PRC_API_TESS_3D_Wire_Extra)
    {
        prc_internal_api_wire *wire = prc_face_internal_wire(&tess->tess_faces[face_index]);
        if (graphics_index > tess->num_line_primitives - 1)
            return PRC_API_ERROR_PARAMETER;
        graphics_object->type = wire[graphics_index].type;
        graphics_object->num_indices = wire[graphics_index].num_indices;
        graphics_object->indices = wire[graphics_index].vertex_indices;
        return 0;
    }

    if (graphics_index > tess->tess_faces[face_index].num_graphic_primitives - 1)
        return PRC_API_ERROR_PARAMETER;

    face = prc_face_internal_face(&tess->tess_faces[face_index]);

    /* Run through each of the entity types */
    for (k = 0; k < PRC_INTERNAL_API_MAX; k++)
    {
        switch (k)
        {
            case PRC_INTERNAL_API_MULTINORM:
            entity = &face->multi_norm;
            break;

            case PRC_INTERNAL_API_SINGLENORM:
            entity = &face->single_norm;
            break;

            case PRC_INTERNAL_API_SINGLENORM_TEXTURE:
            entity = &face->texture_single_norm;
            break;

            case PRC_INTERNAL_API_MULTINORM_TEXTURE:
            entity = &face->texture_multi_norm;
            break;

            default:
            return PRC_API_ERROR_PARSER;
        }

        /* Have triangles */
        if (entity->num_triangles > 0)
        {
            if (graphics_index == 0)
            {
                /* Return triangles */
                graphics_object->type = PRC_API_TRIANGLES;
                graphics_object->num_indices = entity->num_triangles * 3;
                graphics_object->indices = face->vertex_indices;
                return 0;
            }
            /* Make indexing easier into fan or strips */
            graphics_index = graphics_index - 1;
            if (entity->num_fans > 0)
            {
                /* Have Fans */
                if (graphics_index < entity->num_fans)
                {
                    /* Return a fan */
                    graphics_object->type = PRC_API_FAN;
                    graphics_object->num_indices =
                        entity->fan_offsets[2 * graphics_index + 1];
                    graphics_object->indices =
                        &face->vertex_indices[entity->fan_offsets[2 * graphics_index]];
                    return 0;
                }
                else
                {
                    graphics_index = graphics_index - entity->num_fans;
                    graphics_object->type = PRC_API_STRIP;
                    graphics_object->num_indices =
                        entity->strip_offsets[2 * graphics_index + 1];
                    graphics_object->indices =
                        &face->vertex_indices[entity->strip_offsets[2 * graphics_index]];
                    return 0;
                }
            }
            else
            {
                /* Return a strip */
                graphics_object->type = PRC_API_STRIP;
                graphics_object->num_indices =
                    entity->strip_offsets[2 * graphics_index + 1];
                graphics_object->indices =
                    &face->vertex_indices[entity->strip_offsets[2 * graphics_index]];
                return 0;
            }
        }
        else
        {
            /* No Triangles */
            if (entity->num_fans > 0)
            {
                /* Have fans */
                if (graphics_index < entity->num_fans)
                {
                    /* Return a fan */
                    graphics_object->type = PRC_API_FAN;
                    graphics_object->num_indices =
                        entity->fan_offsets[2 * graphics_index + 1];
                    graphics_object->indices =
                        &face->vertex_indices[entity->fan_offsets[2 * graphics_index]];
                    return 0;
                }
                else
                {
                    graphics_index = graphics_index - entity->num_fans;
                    /* Return a strip */
                    graphics_object->type = PRC_API_STRIP;
                    graphics_object->num_indices =
                        entity->strip_offsets[2 * graphics_index + 1];
                    graphics_object->indices =
                        &face->vertex_indices[entity->strip_offsets[2 * graphics_index]];
                    return 0;
                }
            }
            else if (entity->num_strips > 0)
            {
                /* Return a strip */
                graphics_object->type = PRC_API_STRIP;
                graphics_object->num_indices =
                    entity->strip_offsets[2 * graphics_index + 1];
                graphics_object->indices =
                    &face->vertex_indices[entity->strip_offsets[2 * graphics_index]];
                return 0;
            }
        }
    }
    return PRC_API_ERROR_PARAMETER;
}
