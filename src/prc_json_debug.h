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

#ifndef PRC_CONTEXT_H
#define PRC_CONTEXT_H

int debug_prc_read_vertices_from_JSON_file(prc_context *ctx, const char *file_name, prc_vec3 **vertices, uint32_t num_points);
int debug_prc_read_indices_from_JSON_file(prc_context *ctx, const char *file_name, int **indices, uint32_t num_points);

#endif
