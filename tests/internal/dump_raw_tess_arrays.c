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

/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Dumps the raw, still-encoded compressed-tessellation arrays
   (edge_status_array, points_is_reference_array, point_reference_array,
   normal_is_reversed, point_array, origin, tolerance) of a single-file-
   structure raw .prc file's first PRC_TYPE_TESS_3D_Compressed entry as
   plain text, one array per line. Built to feed an independent, from-
   scratch reimplementation of the gate-traversal topology (not reusing
   nanoPRC's own decode functions) so its trace can be diffed against
   nanoPRC's own decode trace to isolate a genuine implementation bug from
   a merely-different-but-valid convention -- see the turbine outer-shell
   (tess 875 / fs13) gate-order divergence investigation.

   HOW: usage: dump_raw_tess_arrays.exe <single_fs_input.prc> <output.txt> */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "zlib.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid, *out;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count = 0;
    uint32_t *section_offsets = NULL;
    uint32_t fs_count;
    uint32_t i;
    uint8_t *tess_inflated = NULL;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_tess_3d_compressed *parsed = NULL;
    int code;

    if (argc < 3) { printf("usage: %s <single_fs_input.prc> <output.txt>\n", argv[0]); return 2; }

    fid = fopen(argv[1], "rb");
    if (fid == NULL) { printf("failed to open %s\n", argv[1]); return 1; }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize) { printf("short read\n"); return 1; }
    if (memcmp(buf, "PRC", 3) != 0) { printf("not a raw PRC file\n"); return 1; }

    /* A PRC file holds "one or more FileStructures" (ISO 14739 4.3.2, "File
       structure") -- assembly component units, ordered parts first, then
       subassemblies, then the root assembly. Each carries its own complete set
       of sections, so a real assembly's tessellation is spread across all of
       them and dumping only the first sees a fraction of the model.

       Header layout, per file structure in turn: uid(16) + reserved(4) +
       section_count(4) + section_count * section_offset(4). Select which one to
       dump with PRC_DIAG_FS_INDEX (default 0). */
    p = buf + 3 + 4 + 4 + 16 + 16;
    fs_count = read_le_u32(p); p += 4;
    if (fs_count == 0) { printf("file_structure_count=0\n"); return 1; }
    {
        const char *fs_env = getenv("PRC_DIAG_FS_INDEX");
        uint32_t want_fs = (fs_env != NULL) ? (uint32_t)atoi(fs_env) : 0;
        uint32_t fs;

        if (want_fs >= fs_count)
        {
            printf("file_structure_count=%u, requested index %u out of range\n", fs_count, want_fs);
            return 1;
        }
        for (fs = 0; fs < fs_count; fs++)
        {
            p += 16;    /* file structure unique id */
            p += 4;     /* reserved */
            section_count = read_le_u32(p); p += 4;
            if (fs == want_fs)
            {
                section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
                for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
                break;
            }
            p += 4 * section_count;   /* skip this file structure's offset table */
        }
        fprintf(stderr, "file_structure_count=%u, dumping index %u\n", fs_count, want_fs);
    }
    if (section_count < 5) { printf("unexpected section_count=%u\n", section_count); return 1; }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uLongf dest_len;
        uint32_t comp_len = section_offsets[4] - section_offsets[3];
        dest_len = comp_len * 40u + 4096u;
        tess_inflated = (uint8_t *)malloc(dest_len);
        for (;;)
        {
            uLongf try_len = dest_len;
            int zret = uncompress(tess_inflated, &try_len, buf + section_offsets[3], comp_len);
            if (zret == Z_OK) { dest_len = try_len; break; }
            if (zret == Z_BUF_ERROR) { dest_len *= 2; free(tess_inflated); tess_inflated = (uint8_t *)malloc(dest_len); continue; }
            printf("zlib uncompress failed: %d\n", zret);
            return 1;
        }
        prc_init_bit_state(ctx, &bit_state, tess_inflated, (size_t)dest_len);
    }

    wrapper_tag = prc_bitread_uint32(ctx, &bit_state);
    if (wrapper_tag != PRC_TYPE_ASM_FileStructureTessellation) { printf("unexpected wrapper tag %u\n", wrapper_tag); return 1; }
    {
        prc_content_prc_base wrapper_base;
        memset(&wrapper_base, 0, sizeof(wrapper_base));
        code = prc_parse_content_prc_base(ctx, &bit_state, &wrapper_base);
        if (code < 0) { printf("prc_parse_content_prc_base failed: %d\n", code); return 1; }
    }
    tess_count = prc_bitread_uint32(ctx, &bit_state);
    {
        uint32_t want_index = 0;
        uint32_t k;
        const char *idx_env = getenv("PRC_DIAG_TESS_INDEX");
        if (idx_env != NULL)
            want_index = (uint32_t)atoi(idx_env);
        if (want_index >= tess_count)
        {
            printf("tess_count=%u, requested index %u out of range\n", tess_count, want_index);
            return 1;
        }
        for (k = 0; k < tess_count; k++)
        {
            prc_tess_3d_compressed *this_parsed = NULL;
            entry_tag = prc_bitread_uint32(ctx, &bit_state);
            if (entry_tag != PRC_TYPE_TESS_3D_Compressed)
            {
                printf("tess %u: entry tag=%u (not COMPRESSED) -- cannot skip past unknown entry type, stopping\n", k, entry_tag);
                return 1;
            }
            code = prc_parse_tess_3d_compressed(ctx, &bit_state, &this_parsed, 0);
            if (code < 0) { printf("tess %u: prc_parse_tess_3d_compressed failed: %d\n", k, code); prc_api_print_error_stack(ctx); return 1; }
            if (k == want_index)
                parsed = this_parsed;
        }
    }
    if (parsed == NULL) { printf("requested tessellation not found\n"); return 1; }

    out = fopen(argv[2], "w");
    if (out == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }

    fprintf(out, "T %u\n", parsed->triangle_face_array_size);
    fprintf(out, "reference_array_size %u\n", parsed->reference_array_size);
    fprintf(out, "point_reference_array_size %u\n", parsed->point_reference_array_size);
    fprintf(out, "point_array_size %u\n", parsed->point_array_size);
    fprintf(out, "tolerance %.17g\n", parsed->tolerance);
    fprintf(out, "origin %.17g %.17g %.17g\n", parsed->origin_array.x, parsed->origin_array.y, parsed->origin_array.z);
    fprintf(out, "must_recalculate_normals %u\n", parsed->must_recalculate_normals);
    fprintf(out, "crease_angle %.17g\n", parsed->crease_angle);
    fprintf(out, "normal_recalculation_flags %u\n", parsed->normal_recalculation_flags);
    fprintf(out, "face_number %u\n", parsed->face_number);
    fprintf(out, "is_point_color %u\n", parsed->is_point_color);
    fprintf(out, "is_multiple_line_attribute %u\n", parsed->is_multiple_line_attribute);
    fprintf(out, "no_texture %u\n", parsed->no_texture);
    fprintf(out, "has_behaviors %u\n", parsed->has_behaviors);
    fprintf(out, "line_attribute_array_size %u\n", parsed->line_attribute_array_size);
    fprintf(out, "line_attribute_array");
    for (i = 0; i < parsed->line_attribute_array_size; i++) fprintf(out, " %d", parsed->line_attribute_array[i]);
    fprintf(out, "\n");
    fprintf(out, "is_face_planar");
    if (parsed->is_face_planar != NULL)
        for (i = 0; i < parsed->face_number; i++) fprintf(out, " %u", parsed->is_face_planar[i]);
    fprintf(out, "\n");

    fprintf(out, "edge_status_array");
    for (i = 0; i < parsed->triangle_face_array_size; i++) fprintf(out, " %u", parsed->edge_status_array[i]);
    fprintf(out, "\n");

    /* triangle -> face id, one entry per triangle. This is the only place the
       per-face grouping of a compressed tessellation is available: the public
       API returns one aggregate primitive per compressed tessellation and
       ignores the face index, so is_face_planar[] cannot otherwise be checked
       against the geometry of the face it describes. */
    fprintf(out, "triangle_face_array_size %u\n", parsed->triangle_face_array_size);
    fprintf(out, "triangle_face_array");
    if (parsed->triangle_face_array != NULL)
        for (i = 0; i < parsed->triangle_face_array_size; i++)
            fprintf(out, " %u", parsed->triangle_face_array[i]);
    fprintf(out, "\n");

    fprintf(out, "points_is_reference_array");
    for (i = 0; i < parsed->reference_array_size; i++) fprintf(out, " %u", parsed->points_is_reference_array[i]);
    fprintf(out, "\n");

    fprintf(out, "point_reference_array");
    for (i = 0; i < parsed->point_reference_array_size; i++) fprintf(out, " %d", parsed->point_reference_array[i]);
    fprintf(out, "\n");

    fprintf(out, "normal_is_reversed");
    if (parsed->must_recalculate_normals)
        for (i = 0; i < parsed->triangle_face_array_size; i++) fprintf(out, " %u", parsed->normal_is_reversed[i]);
    fprintf(out, "\n");

    fprintf(out, "normal_angle_number_of_bits %u\n", parsed->normal_angle_number_of_bits);
    fprintf(out, "normal_binary_data_size %u\n", parsed->normal_binary_data_size);
    fprintf(out, "normal_binary_data");
    if (!parsed->must_recalculate_normals && parsed->normal_binary_data != NULL)
        for (i = 0; i < parsed->normal_binary_data_size; i++) fprintf(out, " %u", parsed->normal_binary_data[i]);
    fprintf(out, "\n");

    fprintf(out, "normal_angle_array_size %u\n", parsed->normal_angle_array_size);
    fprintf(out, "normal_angle_array");
    if (!parsed->must_recalculate_normals && parsed->normal_angle_array != NULL)
        for (i = 0; i < parsed->normal_angle_array_size; i++) fprintf(out, " %d", parsed->normal_angle_array[i]);
    fprintf(out, "\n");

    fprintf(out, "point_array");
    for (i = 0; i < parsed->point_array_size; i++) fprintf(out, " %d", parsed->point_array[i]);
    fprintf(out, "\n");

    fclose(out);
    printf("Wrote %s\n", argv[2]);

    free(section_offsets);
    free(tess_inflated);
    free(buf);
    prc_api_release_context(ctx);
    return 0;
}
