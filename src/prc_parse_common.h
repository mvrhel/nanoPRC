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

#ifndef PRC_PARSE_COMMON_H
#define PRC_PARSE_COMMON_H

#include "prc_data.h"
#include "prc_bit.h"
#include <math.h>

#define PRC_PI 0x1.921fb54442d18p1
#define PRC_HALF_PI 0x1.921fb54442d18p0
#define PRC_PI_OVER_4 0x1.921fb54442d18p-1

int prc_parse_content_prc_base(prc_context *ctx, prc_bit_state *bit_state, prc_content_prc_base *base);
int prc_parse_name(prc_context *ctx, prc_bit_state *bit_state, prc_name *val);
int prc_parse_attribute_data(prc_context *ctx, prc_bit_state *bit_state, prc_attribute_data *data);
int prc_get_attribute_title(prc_context *ctx, prc_bit_state *bit_state, prc_attribute_entry *title);
void debug_prc_unsignedint_bitstream(prc_context *ctx, prc_bit_state *bit_state, uint32_t find, size_t byte_length);
prc_rgb_color prc_parse_rgb(prc_context *ctx, prc_bit_state *bit_state, uint8_t has_alpha);
prc_rgb_color prc_parse_rgb8(prc_context *ctx, prc_bit_state *bit_state, uint8_t has_alpha);
int prc_parse_content_prc_ref_base(prc_context *ctx, prc_bit_state *bit_state, uint8_t parent_is_RI, void *parent, prc_content_prc_ref_base *data);
prc_vec3 prc_parse_3d_vector(prc_context *ctx, prc_bit_state *bit_state);
prc_vec2 prc_parse_2d_vector(prc_context *ctx, prc_bit_state *bit_state);
int prc_parse_content_base_tess_data(prc_context *ctx, prc_bit_state *bit_state, prc_content_base_tess_data *data);
int prc_parse_user_data(prc_context *ctx, prc_bit_state *bit_state, prc_user_data *data);
int prc_parse_cart_trans(prc_context *ctx, prc_bit_state *bit_state, prc_cart_transformation *data);
void prc_parse_3d_transform(prc_context* ctx, prc_bit_state* bit_state, prc_trans_3d *data);
void prc_parse_general_transform(prc_context *ctx, prc_bit_state *bit_state, prc_misc_general_trans *data);
int prc_parse_misc_transform(prc_context *ctx, prc_bit_state *bit_state, prc_misc_transformation *data);
int prc_parse_base_with_graphics(prc_context *ctx, prc_bit_state *bit_state, uint8_t parent_is_RI, void *RIparent, prc_base_with_graphics *data);
prc_unique_id prc_get_compressed_unique_id(prc_context *ctx, prc_bit_state *bit_state);
int prc_parse_representation_item_content(prc_context *ctx, prc_bit_state *bit_state, prc_representation_item_content *data);
int prc_read_check_tag(prc_context *ctx, prc_bit_state *bit_state, prc_unsigned_int expected_tag, prc_unsigned_int *read_tag);
int prc_consume_schema_extension(prc_context *ctx, prc_bit_state *bit_state,
    int schema_code);
int prc_check_for_schema(prc_context *ctx, prc_unsigned_int expected_tag);
prc_domain prc_parse_domain(prc_context *ctx, prc_bit_state *bit_state);
prc_parameterization prc_parse_parameterization(prc_context *ctx, prc_bit_state *bit_state);
prc_interval prc_parse_interval(prc_context *ctx, prc_bit_state *bit_state);
int prc_parse_bound_box(prc_context *ctx, prc_bit_state *bit_state, prc_bounding_box *data);
void prc_parse_file_indentifier(prc_context *ctx, prc_bit_state *bit_state, prc_file_identifier *data);
int prc_parse_math_fct_3d(prc_context *ctx, prc_bit_state *bit_state, prc_math_fct_3d *data);
int prc_parse_math_fct_1d(prc_context *ctx, prc_bit_state *bit_state, prc_math_fct_1d *data);

/* In prc_parse_extra_geometry */
int prc_parse_surf_plane(prc_context *ctx, prc_bit_state *bit_state,
    prc_surf_plane *data, uint8_t read_tag);

/* In prc_release.c */
void prc_release_header(prc_context *ctx, prc_header *header);
#endif

