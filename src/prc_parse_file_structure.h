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

#ifndef PRC_PARSE_FILE_STRUCTURE_H
#define PRC_PARSE_FILE_STRUCTURE_H

#include "prc_data.h"
#include "prc_bit.h"   /* prc_bit_state, for the consumption check below */

/* Whether a section walk consumed the buffer it was given: 0 if it ended within
   a byte of the end or left nothing but zero bits, PRC_ERROR_PARSE otherwise.
   Exposed rather than static so the rule can be tested directly on hand-built
   buffers -- the file that motivated it is a third-party fixture that cannot be
   redistributed here. */
int prc_check_section_consumed(prc_context *ctx, const prc_bit_state *bit_state,
    const uint8_t *buffer, uint32_t size_in_bytes, const char *section_name);

int prc_parse_file_extra_geometry(prc_context *ctx, prc_filestructure *file_struct);
int prc_parse_file_tessellation(prc_context *ctx, prc_filestructure *file_struct, uint8_t debug_tess);
int prc_parse_file_geometry(prc_context *ctx, prc_filestructure *file_struct);
int prc_parse_file_schema_and_global(prc_context *ctx, prc_filestructure *file_struct);
int prc_parse_file_tree(prc_context *ctx, prc_filestructure *file_struct);
int prc_parse_model_file(prc_context *ctx, prc_filestructure *file_struct, uint32_t file_struct_count, uint32_t index);
#endif
