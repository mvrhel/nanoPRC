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

#ifndef PRC_API_INTERNAL_PROTO_H
#define PRC_API_INTERNAL_PROTO_H

#include <stdint.h>
#include "prc_data.h"

/**
 * @file prc_internal_proto_api.h
 * @brief Internal tessellation/API conversion helpers used by nanoPRC.
 *
 * This header declares non-public helpers that convert PRC tessellation data
 * into API-facing buffers and styles.
 */

/** @defgroup internal_api Internal API Conversion Helpers
 *  @brief Internal helpers for tessellation conversion, edge processing, and style mapping.
 *  @{
 */

/**
 * @brief Normal interpretation mode for triangle conversion.
 */
typedef enum {
    PRC_INTERNAL_MULTI_NORM = 0,
    PRC_INTERNAL_SINGLE_NORM_INITIAL,
    PRC_INTERNAL_SINGLE_NORM_SUBSEQUENT
} prc_multi_single_norm_type_t;


typedef enum {
    PRC_INTERNAL_API_EDGE_01 = 0,
    PRC_INTERNAL_API_EDGE_02,
    PRC_INTERNAL_API_EDGE_12
} prc_internal_api_edge_check_case_t;

typedef enum {
    PRC_INTERNAL_API_SET_BOTH_NORMALS_OF_EDGE = 0,
    PRC_INTERNAL_API_SET_FIRST_NORMAL_OF_EDGE,
    PRC_INTERNAL_API_SET_SECOND_NORMAL_OF_EDGE
} prc_internal_api_edge_normal_set_t;

/**
 * @brief One edge and up to two incident triangles during edge-list construction.
 */
typedef struct prc_internal_api_edge_s
{
    uint32_t tri_one_full_indices[3];
    uint32_t tri_two_full_indices[3];
    uint32_t tri_one_edge_indices[2];
    uint32_t tri_two_edge_indices[2];
    prc_internal_api_edge_check_case_t tri_one_edge_case;
    prc_internal_api_edge_check_case_t tri_two_edge_case;
    uint32_t tri_one_vertex_indices_offset;
    uint32_t tri_two_vertex_indices_offset;
    uint32_t num_triangles;
    uint8_t split;
} prc_internal_api_edge;

/**
 * @brief Dynamic list of internal edges used during smoothing/splitting.
 */
typedef struct prc_internal_api_edge_list_s
{
    size_t num_edges;
    size_t capacity;
    prc_internal_api_edge *edge;
} prc_internal_api_edge_list;

/* This is just a convenience structure to pack in stuff into a single object
   that we can readily pass around instead of a pile of individual parts */
typedef struct prc_internal_api_uncomm_tess_data_s
{
    uint32_t **src_index_data;
    prc_api_tess_vertex_buffer *vertex_out;
    uint32_t *vertex_indices;
    prc_internal_api_normal *normals_internal;
    prc_internal_api_position_normal_lookup position_normal_lut;
    prc_tess *tess;
    prc_internal_api_face *face_out_reserved;
    uint8_t must_calculate_normals;
    uint8_t is_texture;
    uint8_t is_material;
    prc_internal_api_edge_list *edge_list;
    double crease_angle;
    prc_file_struct_internal_global_data *global_data;
    float *decoded_colors;
    uint8_t has_texture_transform;
    double texture_transform[9];
    uint8_t has_pure_color;
    float pure_color[4];

    /* Mapping from an original API vertex index (index into the initial
       vertex_out array / position_normal_lut) to the first duplicate created
       for that original during splitting.
       - Size should be initialized to position_normal_lut.number_values.
       - Entries initialized to UINT32_MAX to indicate "no duplicate yet".
       - When a split would create a duplicate of original O, the code should:
           if (first_duplicate_for_original[O] != UINT32_MAX)
               reuse that index instead of creating another duplicate;
           else
               create duplicate and store it here.
       This prevents creating multiple duplicate-of-duplicate vertices
       when several edges reference the same original vertex.
    */
    uint32_t *first_duplicate_for_original;

    /* Mapping from ANY API vertex index back to its original position index.
       - For indices < initial_count: vertex_to_original[i] = i (identity)
       - For duplicates: vertex_to_original[dup_idx] = original_position_index
       - Size/capacity managed alongside vertex_out (grows when vertex_out grows)
       - This provides O(1) lookup of which original a duplicate came from,
         avoiding O(n) searches through first_duplicate_for_original.
    */
    uint32_t *vertex_to_original;
    size_t vertex_to_original_capacity;

    uint32_t *prc_vertex_indice_to_api_vertex_indice;
    uint32_t *prc_normal_indice_to_api_normal_indice;
    uint32_t *prc_texture_indice_to_api_texture_indice;

    float *face_initial_normals;
    float *face_initial_texture_coords;
    float *face_initial_positions;
    uint32_t face_num_initial_positions;
    uint32_t face_num_initial_normals;
    uint32_t face_num_initial_texture_coords;

    uint32_t *face_position_indices;
    uint32_t *face_normal_indices;
    uint32_t *face_texture_indices;
    uint32_t *face_vertex_color_indices;

    uint32_t debug_tess_index;
    uint32_t debug_face_index;

    uint32_t index_count;
} prc_internal_api_uncomm_tess_data;

/**
 * @brief Initialize newly-added vertices after a vertex buffer growth/reallocation.
 *
 * @param ctx Parser/context object.
 * @param vertex_buffer Vertex buffer whose newly appended entries must be initialized.
 */
void prc_internal_api_initialize_vertex(prc_context *ctx,
    prc_api_tess_vertex_buffer *vertex_buffer);

/**
 * @brief Resolve style color values from a style index.
 *
 * @param ctx Parser/context object.
 * @param global_data Global file data used for style lookup.
 * @param style_index Unbiased style index.
 * @param color_out Output RGBA (or RGB + implied alpha depending on caller usage).
 * @return 0 on success, negative error code on failure.
 */
int prc_internal_api_get_color_from_style(prc_context *ctx,
    prc_file_struct_internal_global_data *global_data,
    int32_t style_index, float *color_out);

/**
 * @brief Get color from global data.
 *
 * @param ctx Parser/context object.
 * @param global_data Global file data used for style lookup.
 * @param color_index Unbiased color index.
 * @param color_out Output RGB (or RGB + implied alpha depending on caller usage).
 * @return 0 on success, negative error code on failure.
 */
int
prc_internal_api_get_color(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    int32_t color_index, float *color_out);

/**
 * @brief Build a 4x4 transform from a PRC location transform.
 *
 * @param ctx Parser/context object.
 * @param location PRC transform source.
 * @param transform Output transform matrix buffer.
 * @param is_identity Output flag indicating whether the transform is identity.
 * @return 0 on success, negative error code on failure.
 */
int prc_internal_api_set_transform(prc_context *ctx, const prc_misc_transformation *location,
    double *transform, uint8_t *is_identity);

/**
 * @brief Initialize a face output container to a safe default state.
 *
 * @param ctx Parser/context object.
 * @param face_out Face container to initialize.
 */
void prc_api_initialize_face(prc_context *ctx, prc_api_face *face_out);

/**
 * @brief Initialize an API texture container to a safe default state.
 *
 * @param ctx Parser/context object.
 * @param texture Texture container to initialize.
 */
void prc_api_initialize_texture(prc_context *ctx, prc_api_texture *texture);

/**
 * @brief Initialize an internal style structure to defaults.
 *
 * @param ctx Parser/context object.
 * @param style Style structure to initialize.
 */
void prc_internal_api_initialize_style(prc_context *ctx, prc_internal_graph_style *style);

int prc_internal_api_set_fans(prc_context *ctx, prc_tess_face face,
    uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
    size_t *face_tessellation_index, size_t *num_indices);

int prc_internal_api_set_strips(prc_context *ctx, prc_tess_face face,
    uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
    size_t *face_tessellation_index, size_t *num_indices);

void prc_internal_api_set_triangles(prc_context *ctx, prc_tess_face face,
    uint8_t is_single_norm, prc_internal_api_tess_entities *entities,
    size_t *face_tessellation_index, size_t *num_indices);

int prc_internal_api_vertex_triangle_multinorm(prc_context *ctx, size_t num_triangles,
    uint8_t has_texture, prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_vertex_fan_multinorm(prc_context *ctx, size_t num_fans,
    size_t *fan_offsets, uint8_t has_texture, prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_vertex_strip_multinorm(prc_context *ctx, size_t num_strips,
    size_t *strip_offsets, uint8_t has_texture, prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_vertex_triangle_one_norm(prc_context *ctx,
    size_t num_triangles, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_vertex_fan_one_norm(prc_context *ctx,
    size_t num_fans, size_t *fan_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_vertex_strip_one_norm(prc_context *ctx,
    size_t num_strips, size_t *strip_offsets, uint8_t has_texture,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_build_edge_list(prc_context *ctx, uint32_t num_triangles,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_compute_normals(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

void prc_copy_internal_graph_style_to_api(prc_context *ctx,
    prc_internal_graph_style *wire_style, prc_api_material *api_material);

int prc_internal_api_get_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, int32_t style_index, uint8_t *is_material, uint8_t *is_texture,
    prc_api_material *material, prc_internal_graph_style *style, float *color,
    float alpha, uint8_t dont_allow_texture);

int prc_internal_set_texture_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, const prc_graph_material *graph_material,
    prc_internal_graph_style *style, prc_internal_texture *internal_texture);

int prc_internal_set_internal_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data,
    prc_file_structure_header *header, prc_graph_style *graph_style, uint8_t is_material,
    uint8_t is_texture, prc_internal_graph_style *style);

void prc_internal_api_process_vertex_color_data(prc_context *ctx, uint32_t num_vertices,
    prc_color_data *color_data, uint8_t has_alpha, float *decoded_colors);

void prc_internal_api_set_vertex_texture_coords(prc_context *ctx, prc_api_vertex *vertex,
    double raw_u, double raw_v, double *transform_matrix, uint8_t has_transform);

int prc_internal_get_surface_material(prc_context *ctx,
    prc_file_struct_internal_global_data *global_data, int32_t index,
    prc_api_material *material);

int prc_internal_uncompressed_create_vertices(prc_context *ctx,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

int prc_internal_api_calculate_normals_triangles(prc_context *ctx, uint32_t num_triangles,
    prc_internal_api_uncomm_tess_data *uncompressed_data);

/** @} */

#endif
