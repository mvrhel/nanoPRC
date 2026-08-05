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

#ifndef PRC_BIT_DEBUG_H
#define PRC_BIT_DEBUG_H

/* Manual, breakpoint-only bitstream search aids -- not called anywhere in
   the normal parse path. See prc_bit_debug.c for usage. */
void prc_debug_stream(prc_context *ctx, prc_bit_state *state);
void prc_debug_view_stream_bounding_box_search(prc_context *ctx, prc_bit_state *state);
void prc_debug_view_stream_on_byte_boundary(prc_context *ctx, prc_bit_state *state);

#endif
