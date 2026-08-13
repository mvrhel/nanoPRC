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

   WHAT: Dumps every field of a single-file-structure raw .prc file's first
   PRC_TYPE_TESS_3D (uncompressed) tessellation entry -- the top-level
   has_faces/has_loops/must_calculate_normals bits plus every per-face
   record's tag/size_of_line_attributes/used_entities_flag/
   start_triangulated/size_of_triangulateddata/triangulateddata contents/
   number_of_textured_coordinate_indexes/has_vertex_colors. Built to compare
   against real-world, independently-produced uncompressed-PRC files (e.g.
   the *_ePRC.pdf test meshes) field-by-field against what
   prc_write_tess_3d.c emits, after two evidence-based guesses (tree shape,
   has_faces) both failed to fix an Acrobat-blank-model-tree symptom on
   this write facility's own TRIANGLES output -- see reauthor_tess_pdf.c's
   header comment for that history.

   Later extended (still read-only, still one PRC_TYPE_TESS_3D entry) for
   tracing a specific known-bad decoded vertex (from detect_tess_spikes.c's
   flagged output) back to its raw position in the file, to find which
   face/primitive produced it -- see the turbine Outer_Engine_Cover
   investigation:
     - [range_start range_count]: prints that exact window of
       triangulated_index_array (the default dump only shows the first 60
       of what can be tens of thousands of values).
     - --find x y z [tol=0.01]: scans the raw position-coordinate array
       (tessellation_coordinates.coordinates) for any position within tol
       of the target, printing its position_index and raw_index (=
       position_index*3, the value that actually appears in
       triangulated_index_array for a position reference). Skips the rest
       of the normal dump when used.
     - --allcoords: prints every tessellation_coordinates value at full
       (%.17g, bit-exact-roundtrippable) precision, one per line, followed
       by every triangulated_index_array value, one per line -- for a
       value-by-value diff between two files' decoded content (as opposed
       to their raw bits), added for the 2026-07-16 bit-position-tracing
       phase of the Acrobat blank-tree investigation (see ISO-SPEC/
       blanktree-matrix-evidence-table.md row 31) to check whether the
       decoded VALUES genuinely match between two independently-encoded
       files of the same nominal geometry, before tracing bit-level
       encoding choices. Skips the rest of the normal dump when used.

   HOW: usage: dump_uncompressed_tess_fields.exe <single_fs_input.prc>
   [range_start range_count] | [--find x y z [tol=0.01]] | [--allcoords]
   Extract a raw .prc first with extract_raw_prc.exe if starting from a
   PDF. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count;
    uint32_t *section_offsets;
    uint32_t i, f, k;
    uint8_t *tess_inflated = NULL;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_tess_3d *parsed = NULL;
    int code;
    int found = 0;

    uint32_t range_start = 0, range_count = 0; /* optional: print this exact window of triangulated_index_array instead of the default first-60 */
    int find_mode = 0;
    int allcoords_mode = 0;
    double find_xyz[3] = { 0, 0, 0 };
    double find_tol = 0.01;
    if (argc < 2) { printf("usage: %s <single_fs_input.prc> [range_start range_count] | [--find x y z [tol=0.01]]\n", argv[0]); return 2; }
    if (argc >= 3 && strcmp(argv[2], "--find") == 0)
    {
        find_mode = 1;
        if (argc < 6) { printf("--find needs x y z [tol]\n"); return 2; }
        find_xyz[0] = atof(argv[3]); find_xyz[1] = atof(argv[4]); find_xyz[2] = atof(argv[5]);
        if (argc >= 7) find_tol = atof(argv[6]);
    }
    else if (argc >= 3 && strcmp(argv[2], "--allcoords") == 0) { allcoords_mode = 1; }
    else if (argc >= 4) { range_start = (uint32_t)strtoul(argv[2], NULL, 10); range_count = (uint32_t)strtoul(argv[3], NULL, 10); }

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

    p = buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("only single-file-structure inputs supported\n"); return 1; }
    p += 4;
    p += 16;
    p += 4;
    section_count = read_le_u32(p); p += 4;
    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
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
        printf("compressed_size %u inflated_size %lu\n", comp_len, (unsigned long)dest_len);
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
    printf("tess_count %u\n", tess_count);

    for (i = 0; i < tess_count && !found; i++)
    {
        entry_tag = prc_bitread_uint32(ctx, &bit_state);
        printf("entry %u tag=%u\n", i, entry_tag);
        if (entry_tag != PRC_TYPE_TESS_3D)
        {
            printf("  (not PRC_TYPE_TESS_3D, this tool cannot skip past it -- stopping)\n");
            break;
        }

        code = prc_parse_tess_3d(ctx, &bit_state, &parsed);
        if (code < 0) { printf("prc_parse_tess_3d failed: %d\n", code); prc_api_print_error_stack(ctx); return 1; }
        found = 1;

        if (find_mode)
        {
            /* Scan the raw position-coordinate array (tessellation_coordinates.
               coordinates, triples of x,y,z) for anything within find_tol of
               the target -- used to locate a known-bad decoded vertex's raw
               position index, so its consuming face/strip/fan can be traced. */
            uint32_t n = parsed->tessellation_coordinates.number_of_coordinates / 3;
            uint32_t vi;
            uint32_t nmatches = 0;
            printf("searching %u positions for (%.6g,%.6g,%.6g) tol=%.6g\n", n, find_xyz[0], find_xyz[1], find_xyz[2], find_tol);
            for (vi = 0; vi < n; vi++)
            {
                double x = parsed->tessellation_coordinates.coordinates[vi*3+0];
                double y = parsed->tessellation_coordinates.coordinates[vi*3+1];
                double z = parsed->tessellation_coordinates.coordinates[vi*3+2];
                double dx = x - find_xyz[0], dy = y - find_xyz[1], dz = z - find_xyz[2];
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < find_tol)
                {
                    printf("MATCH position_index=%u raw_index(=idx*3)=%u pos=(%.6g,%.6g,%.6g) dist=%.6g\n",
                        vi, vi*3, x, y, z, dist);
                    nmatches++;
                }
            }
            printf("total matches: %u\n", nmatches);
            printf("RESULT OK\n");
            free(section_offsets);
            free(tess_inflated);
            free(buf);
            prc_api_release_context(ctx);
            return 0;
        }

        if (allcoords_mode)
        {
            uint32_t nc = parsed->tessellation_coordinates.number_of_coordinates;
            for (k = 0; k < nc; k++)
                printf("%.17g\n", parsed->tessellation_coordinates.coordinates[k]);
            for (k = 0; k < parsed->number_of_triangulated_indicies; k++)
                printf("%u\n", parsed->triangulated_index_array[k]);
            free(section_offsets);
            free(tess_inflated);
            free(buf);
            prc_api_release_context(ctx);
            return 0;
        }

        printf("has_faces %u\n", parsed->has_faces);
        printf("has_loops %u\n", parsed->has_loops);
        printf("must_calculate_normals %u\n", parsed->must_calculate_normals);
        if (parsed->must_calculate_normals)
        {
            printf("normal_recalculation_flags %u\n", parsed->normal_recalculation_flags);
            printf("crease_angle %.17g\n", parsed->crease_angle);
        }
        printf("number_of_normal_coordinates %u\n", parsed->number_of_normal_coordinates);
        printf("number_of_wire_indices %u\n", parsed->number_of_wire_indices);
        printf("number_of_triangulated_indicies %u\n", parsed->number_of_triangulated_indicies);
        if (range_count > 0)
        {
            uint32_t end = range_start + range_count;
            if (end > parsed->number_of_triangulated_indicies) end = parsed->number_of_triangulated_indicies;
            printf("triangulated_index_array[%u..%u)", range_start, end);
            for (k = range_start; k < end; k++)
                printf(" %u", parsed->triangulated_index_array[k]);
            printf("\n");
        }
        else
        {
            printf("triangulated_index_array");
            for (k = 0; k < parsed->number_of_triangulated_indicies && k < 60; k++)
                printf(" %u", parsed->triangulated_index_array[k]);
            if (parsed->number_of_triangulated_indicies > 60) printf(" ...(%u total)", parsed->number_of_triangulated_indicies);
            printf("\n");
        }
        printf("number_of_face_tessellation %u\n", parsed->number_of_face_tessellation);

        for (f = 0; f < parsed->number_of_face_tessellation; f++)
        {
            prc_tess_face *face = &parsed->face_tessellation_data[f];
            printf("face[%u] tag=%u size_of_line_attributes=%u start_of_wire_data=%u size_of_sizes_wire=%u "
                   "used_entities_flag=%u start_triangulated=%u size_of_triangulateddata=%u "
                   "number_of_textured_coordinate_indexes=%u has_vertex_colors=%u\n",
                f, face->tag, face->size_of_line_attributes, face->start_of_wire_data, face->size_of_sizes_wire,
                face->used_entities_flag, face->start_triangulated, face->size_of_triangulateddata,
                face->number_of_textured_coordinate_indexes, face->has_vertex_colors);
            printf("  line_attributes");
            for (k = 0; k < face->size_of_line_attributes; k++) printf(" %u", face->line_attributes[k]);
            printf("\n  triangulateddata");
            for (k = 0; k < face->size_of_triangulateddata && k < 30; k++) printf(" %u", face->triangulateddata[k]);
            if (face->size_of_triangulateddata > 30) printf(" ...(%u total)", face->size_of_triangulateddata);
            printf("\n");
            if (face->size_of_line_attributes > 0)
                printf("  behavior %u\n", face->behavior);
        }
        printf("number_of_texture_coordinates %u\n", parsed->number_of_texture_coordinates);
    }

    if (!found) printf("RESULT NO_TESS_3D_ENTRY_FOUND\n");
    else printf("RESULT OK\n");

    free(section_offsets);
    free(tess_inflated);
    free(buf);
    prc_api_release_context(ctx);
    return 0;
}
