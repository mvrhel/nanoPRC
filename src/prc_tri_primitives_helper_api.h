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

#include "prc_data.h"

#ifndef PRC_PRC_TRI_PRIMITIVES_HELPER_API_H
#define PRC_PRC_TRI_PRIMITIVES_HELPER_API_H

void prc_api_helper_set_vertex_position(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t vertex_in_pos);
void prc_api_helper_set_face_color(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, float *face_color, prc_internal_api_position_normal_pair *position_normal_pair);
void prc_api_helper_intialize_position_normal_pair(prc_context *ctx, prc_internal_api_position_normal_pair *position_normal_pair, uint32_t vertex_out_pos, prc_internal_api_color_state_t color_state, prc_internal_api_style_state_t style_state, int32_t face_style, int32_t face_style_file_index);
void prc_api_helper_set_normal(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, prc_internal_api_position_normal_pair *position_normal_pair, uint32_t normal_index);
void prc_api_helper_set_vertex_color(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t vertex_in_pos, prc_internal_api_position_normal_pair *position_normal_pair);
int prc_api_helper_set_tri_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data, uint32_t tess_file_index, prc_file_structure_header *header, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t triangle_count, prc_internal_api_position_normal_pair *position_normal_pair, prc_internal_api_face *face_out_reserved, prc_internal_api_style_state_t new_state);
uint8_t prc_api_helper_color_mismatch(prc_context *ctx, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t vertex_in_pos);
int prc_api_helper_set_vertex_style(prc_context *ctx, prc_file_struct_internal_global_data *global_data, prc_file_structure_header *header, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, int32_t unbiased_style_index);
int prc_api_helper_set_vertex_style_from_face_ref(prc_context *ctx, uint8_t vertices_have_style, prc_api_tess_vertex_buffer *vertex_out, uint32_t vertex_out_pos, prc_tess_3d_compressed *tess, uint32_t triangle_count, prc_internal_api_position_normal_pair *position_normal_pair, prc_internal_api_face *face_out_reserved);
#endif
