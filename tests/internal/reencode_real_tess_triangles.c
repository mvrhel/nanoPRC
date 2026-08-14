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

   WHAT: The uncompressed-tessellation ("TRIANGLES" write kind) sibling of
   reencode_real_tess.c. Takes a REAL, complete, independently-produced raw
   .prc file (any number of file structures) and for every file structure
   whose tessellation section is a single (tess_count==1) PRC_TYPE_TESS_3D
   entry, with must_calculate_normals==1 and every face's
   used_entities_flag exactly PRC_FACETESSDATA_Triangle (no Fan/Strip/
   OneNormal mixing -- the shape this tool's five real reference files
   this session's Acrobat-blank-tree investigation identified actually
   use), fully DECODES the raw position/triangle data with nanoPRC's own
   parser, then RE-ENCODES it through nanoPRC's own writer
   (prc_write_tessellation_section_to_stream, PRC_API_WRITE_TESS_KIND_
   TRIANGLES) -- reusing the real production encoder, fed 100% real
   decoded positions/topology instead of a synthetic mesh. File structures
   whose tessellation section doesn't match that shape are left byte-for-
   byte untouched and reported as skipped. Everything else (schema, tree,
   geometry, extra_geometry, model-file) is left byte-for-byte untouched
   real content.

   HOW: usage: reencode_real_tess_triangles.exe <real_input.prc>
   <output.prc>.

   CONTAINER FORMAT: identical to reencode_real_tess.c -- see that file's
   own header comment for the full byte-layout description this tool's
   splice logic (below, copied near-verbatim) depends on.

   WHY IT EXISTS: this session's Acrobat-blank-model-tree investigation for
   uncompressed (TRIANGLES-kind) output tried three independent, real-file-
   comparison-motivated fixes at the PDF-wrapper/tree-shape layer (an empty
   part on every intermediate product level, matching a real /3DA
   dictionary exactly, adding /Resources<</Names[]>> to the /Type/3D
   stream dict) without success, then a synthetic-single-triangle splice
   test (schema+tree+tessellation ours, geometry/model-file real) that also
   failed in both Acrobat AND PDF-XChange even with an explicit default
   view added -- narrowing suspicion to this write facility's tessellation-
   section encoding itself, mirroring what reencode_real_tess.c already
   found for the COMPRESSED write kind. But that synthetic test used a
   degenerate 1-triangle mesh, leaving open whether the failure is a real
   encoding bug or an artifact of the synthetic case. This closes that gap
   by testing with real, complete, non-degenerate position/topology data
   from an actual working file instead.

   LIMITATIONS: Skips (rather than re-encodes) any file structure whose
   tessellation section isn't exactly one PRC_TYPE_TESS_3D entry with
   must_calculate_normals==1 and every face using pure
   PRC_FACETESSDATA_Triangle (no Fan/Strip/OneNormal, no per-vertex stored
   normals, no texture coordinates) -- deliberately scoped to the simple
   case actually observed in this investigation's five real reference
   files (ElevationMeshIS_ePRC.pdf, xml-sample-{wrl,iv,3ds}_ePRC.pdf), not
   a general uncompressed-tessellation reencoder. Vertex colors, if
   present, are intentionally dropped (prc_write_tess_3d has no color
   parameters at all; not what this test checks). Re-encodes with
   normals=NULL (letting prc_write_tess_3d recompute one flat per-face
   normal via PRC_FACETESSDATA_TriangleOneNormal)
   rather than exactly replicating the original's own must_calculate_normals
   convention bit-for-bit -- a real, known difference from the source file,
   but position/topology (the primary thing under test) is unaffected. */
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
#include "prc_write_tess_3d.h"
#include "prc_write_file_structure.h"
#include "prc_write_common.h"
#include "zlib.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void write_le_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v&0xff); p[1]=(uint8_t)((v>>8)&0xff); p[2]=(uint8_t)((v>>16)&0xff); p[3]=(uint8_t)((v>>24)&0xff); }

typedef struct
{
    uint32_t section_count;
    uint32_t *section_offset;
} fs_info;

static int
try_reencode_fs_tess(prc_context *ctx, const uint8_t *buf, uint32_t k, const fs_info *fs,
    uint8_t **out_bytes, size_t *out_len, double bbox_min_out[3], double bbox_max_out[3])
{
    uint32_t sec_start, sec_end, comp_len;
    uint8_t *inflated = NULL;
    uLongf dest_len;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_content_prc_base wrapper_base;
    prc_tess_3d *parsed = NULL;
    prc_bit_write_state wrapper_s;
    uint8_t *wrapper_comp = NULL;
    size_t wrapper_comp_len = 0;
    double *positions = NULL;
    uint32_t *tri_indices = NULL;
    uint32_t *face_tri_counts = NULL;
    uint32_t num_positions, num_faces, total_triangles, tri_cursor;
    uint32_t f;
    int code;
    int ok = 0;

    if (fs->section_count < 5)
    {
        printf("fs %u: section_count=%u (< 5), no tessellation section, skipping\n", k, fs->section_count);
        return 0;
    }
    sec_start = fs->section_offset[3];
    sec_end = fs->section_offset[4];
    if (sec_end <= sec_start)
    {
        printf("fs %u: empty tessellation section, skipping\n", k);
        return 0;
    }
    comp_len = sec_end - sec_start;

    dest_len = comp_len * 40u + 4096u;
    inflated = (uint8_t *)malloc(dest_len);
    for (;;)
    {
        uLongf try_len = dest_len;
        int zret = uncompress(inflated, &try_len, buf + sec_start, comp_len);
        if (zret == Z_OK) { dest_len = try_len; break; }
        if (zret == Z_BUF_ERROR) { dest_len *= 2; free(inflated); inflated = (uint8_t *)malloc(dest_len); continue; }
        printf("fs %u: zlib uncompress failed (%d), skipping\n", k, zret);
        free(inflated);
        return 0;
    }

    prc_init_bit_state(ctx, &bit_state, inflated, (size_t)dest_len);
    wrapper_tag = prc_bitread_uint32(ctx, &bit_state);
    if (wrapper_tag != PRC_TYPE_ASM_FileStructureTessellation)
    {
        printf("fs %u: unexpected wrapper tag %u, skipping\n", k, wrapper_tag);
        free(inflated);
        return 0;
    }
    memset(&wrapper_base, 0, sizeof(wrapper_base));
    code = prc_parse_content_prc_base(ctx, &bit_state, &wrapper_base);
    if (code < 0)
    {
        printf("fs %u: prc_parse_content_prc_base failed (%d), skipping\n", k, code);
        free(inflated);
        return 0;
    }
    tess_count = prc_bitread_uint32(ctx, &bit_state);
    if (tess_count != 1)
    {
        printf("fs %u: tess_count=%u (not 1), skipping\n", k, tess_count);
        free(inflated);
        return 0;
    }
    entry_tag = prc_bitread_uint32(ctx, &bit_state);
    if (entry_tag != PRC_TYPE_TESS_3D)
    {
        printf("fs %u: first tessellation entry tag=%u (not PRC_TYPE_TESS_3D), skipping\n", k, entry_tag);
        free(inflated);
        return 0;
    }

    code = prc_parse_tess_3d(ctx, &bit_state, &parsed);
    if (code < 0)
    {
        printf("fs %u: prc_parse_tess_3d failed (%d), skipping\n", k, code);
        prc_api_print_error_stack(ctx);
        free(inflated);
        return 0;
    }

    if (!parsed->must_calculate_normals)
    {
        printf("fs %u: must_calculate_normals=0 (stored normals), out of scope, skipping\n", k);
        free(inflated);
        return 0;
    }
    for (f = 0; f < parsed->number_of_face_tessellation; f++)
    {
        if (parsed->face_tessellation_data[f].used_entities_flag != PRC_FACETESSDATA_Triangle)
        {
            printf("fs %u: face %u used_entities_flag=%u (not pure Triangle), out of scope, skipping\n",
                k, f, parsed->face_tessellation_data[f].used_entities_flag);
            free(inflated);
            return 0;
        }
        /* Vertex colors, if present, are intentionally dropped during
           re-encode -- prc_write_tess_3d has no color parameters at all,
           and color preservation isn't what this test is checking
           (position/topology bit-encoding acceptance is). */
    }

    num_positions = parsed->tessellation_coordinates.number_of_coordinates / 3;
    num_faces = parsed->number_of_face_tessellation;
    positions = (double *)malloc(sizeof(double) * 3 * (num_positions ? num_positions : 1));
    memcpy(positions, parsed->tessellation_coordinates.coordinates, sizeof(double) * 3 * num_positions);

    if (num_positions > 0)
    {
        uint32_t a, v;
        for (a = 0; a < 3; a++) { bbox_min_out[a] = positions[a]; bbox_max_out[a] = positions[a]; }
        for (v = 1; v < num_positions; v++)
            for (a = 0; a < 3; a++)
            {
                double val = positions[v * 3 + a];
                if (val < bbox_min_out[a]) bbox_min_out[a] = val;
                if (val > bbox_max_out[a]) bbox_max_out[a] = val;
            }
    }
    else
    {
        uint32_t a;
        for (a = 0; a < 3; a++) { bbox_min_out[a] = 0.0; bbox_max_out[a] = 0.0; }
    }

    face_tri_counts = (uint32_t *)malloc(sizeof(uint32_t) * (num_faces ? num_faces : 1));
    total_triangles = 0;
    for (f = 0; f < num_faces; f++)
    {
        uint32_t n_tri = parsed->face_tessellation_data[f].triangulateddata[0];
        face_tri_counts[f] = n_tri;
        total_triangles += n_tri;
    }

    tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * (total_triangles ? total_triangles : 1));
    tri_cursor = 0;
    for (f = 0; f < num_faces; f++)
    {
        prc_tess_face *face = &parsed->face_tessellation_data[f];
        uint32_t n_corners = face_tri_counts[f] * 3;
        uint32_t c;
        for (c = 0; c < n_corners; c++)
        {
            uint32_t raw = parsed->triangulated_index_array[face->start_triangulated + c];
            tri_indices[tri_cursor * 3 + (c % 3)] = raw / 3;
            if ((c % 3) == 2) tri_cursor++;
        }
    }

    {
        prc_api_write_tessellation tess;

        memset(&tess, 0, sizeof(tess));
        tess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
        tess.positions = positions;
        tess.num_positions = num_positions;
        tess.normals = NULL;
        tess.num_normals = 0;
        tess.tri_indices = tri_indices;
        tess.norm_indices = NULL;
        tess.num_triangles = total_triangles;
        tess.face_tri_counts = face_tri_counts;
        tess.num_faces = num_faces;

        memset(&wrapper_s, 0, sizeof(wrapper_s));
        if (prc_bitwrite_init(ctx, &wrapper_s, 4096) != 0) { printf("fs %u: wrapper init failed, skipping\n", k); goto cleanup; }
        code = prc_write_tessellation_section_to_stream(ctx, &wrapper_s, &tess, 1);
        if (code != 0)
        {
            printf("fs %u: prc_write_tessellation_section_to_stream failed (%d), skipping\n", k, code);
            prc_api_print_error_stack(ctx);
            goto cleanup;
        }
        if (prc_bitwrite_flush(ctx, &wrapper_s) != 0) { printf("fs %u: wrapper flush failed, skipping\n", k); goto cleanup; }

        if (prc_write_deflate(ctx, wrapper_s.buf, wrapper_s.byte_pos, &wrapper_comp, &wrapper_comp_len) != 0)
        {
            printf("fs %u: deflate failed, skipping\n", k);
            goto cleanup;
        }

        printf("fs %u: tess reencoded (%u positions, %u faces, %u triangles), %u -> %zu bytes (compressed)\n",
            k, num_positions, num_faces, total_triangles, comp_len, wrapper_comp_len);
        *out_bytes = wrapper_comp;
        *out_len = wrapper_comp_len;
        ok = 1;
        wrapper_comp = NULL;
    }

cleanup:
    free(positions);
    free(tri_indices);
    free(face_tri_counts);
    prc_bitwrite_release(ctx, &wrapper_s);
    if (wrapper_comp != NULL) prc_free(ctx, wrapper_comp);
    free(inflated);
    return ok;
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t file_structure_count;
    fs_info *fs;
    uint32_t k, j;
    uint32_t start_offset, end_offset, file_count;
    size_t header_len;
    size_t total_bounds, bi;
    uint32_t *bounds;
    uint32_t *header_bytepos;
    int32_t *owner_k, *owner_i;
    uint8_t **new_tess_bytes;
    size_t *new_tess_len;
    uint8_t *fs_processed;
    uint32_t n_processed = 0;
    int64_t *chunk_delta;
    uint32_t *new_bounds;
    int64_t running;
    size_t m;
    uint8_t *out_buf;
    size_t new_total_size;
    FILE *out_fid;
    double g_bbox_min[3] = { 0.0, 0.0, 0.0 };
    double g_bbox_max[3] = { 0.0, 0.0, 0.0 };
    uint8_t g_have_bbox = 0;

    if (argc < 3)
    {
        printf("usage: %s <real_input.prc> <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }

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
    file_structure_count = read_le_u32(p); p += 4;
    if (file_structure_count == 0) { printf("file_structure_count=0\n"); return 1; }

    fs = (fs_info *)calloc(file_structure_count, sizeof(fs_info));

    total_bounds = 2;
    {
        const uint8_t *pp = p;
        for (k = 0; k < file_structure_count; k++)
        {
            uint32_t sc;
            pp += 16 + 4;
            sc = read_le_u32(pp); pp += 4;
            pp += (size_t)sc * 4;
            fs[k].section_count = sc;
            total_bounds += sc;
        }
    }

    bounds = (uint32_t *)malloc(sizeof(uint32_t) * total_bounds);
    header_bytepos = (uint32_t *)malloc(sizeof(uint32_t) * total_bounds);
    owner_k = (int32_t *)malloc(sizeof(int32_t) * total_bounds);
    owner_i = (int32_t *)malloc(sizeof(int32_t) * total_bounds);

    bi = 0;
    for (k = 0; k < file_structure_count; k++)
    {
        p += 16;
        p += 4;
        p += 4;
        fs[k].section_offset = (uint32_t *)malloc(sizeof(uint32_t) * fs[k].section_count);
        for (j = 0; j < fs[k].section_count; j++)
        {
            header_bytepos[bi] = (uint32_t)(p - buf);
            fs[k].section_offset[j] = read_le_u32(p); p += 4;
            bounds[bi] = fs[k].section_offset[j];
            owner_k[bi] = (int32_t)k;
            owner_i[bi] = (int32_t)j;
            bi++;
        }
    }
    header_bytepos[bi] = (uint32_t)(p - buf);
    start_offset = read_le_u32(p); p += 4;
    bounds[bi] = start_offset; owner_k[bi] = -1; owner_i[bi] = 0; bi++;
    header_bytepos[bi] = (uint32_t)(p - buf);
    end_offset = read_le_u32(p); p += 4;
    bounds[bi] = end_offset; owner_k[bi] = -1; owner_i[bi] = 1; bi++;
    file_count = read_le_u32(p); p += 4;
    if (file_count != 0) { printf("external file references (file_count=%u) not supported\n", file_count); return 1; }
    if (bi != total_bounds) { printf("internal error: bound count mismatch (%zu vs %zu)\n", bi, total_bounds); return 1; }
    header_len = (size_t)(p - buf);

    for (m = 1; m < total_bounds; m++)
    {
        if (bounds[m] < bounds[m-1])
        {
            printf("section offsets are not monotonically increasing at index %zu (%u < %u) -- "
                "container layout differs from what this tool assumes, aborting\n", m, bounds[m], bounds[m-1]);
            return 1;
        }
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    printf("file_structure_count=%u\n", file_structure_count);

    new_tess_bytes = (uint8_t **)calloc(file_structure_count, sizeof(uint8_t *));
    new_tess_len = (size_t *)calloc(file_structure_count, sizeof(size_t));
    fs_processed = (uint8_t *)calloc(file_structure_count, 1);

    {
        double union_bbox_min[3] = { 0.0, 0.0, 0.0 };
        double union_bbox_max[3] = { 0.0, 0.0, 0.0 };
        uint8_t have_bbox = 0;

        for (k = 0; k < file_structure_count; k++)
        {
            uint8_t *ob = NULL;
            size_t ol = 0;
            double fbmin[3], fbmax[3];
            if (try_reencode_fs_tess(ctx, buf, k, &fs[k], &ob, &ol, fbmin, fbmax))
            {
                uint32_t a;
                new_tess_bytes[k] = ob;
                new_tess_len[k] = ol;
                fs_processed[k] = 1;
                n_processed++;
                if (!have_bbox)
                {
                    for (a = 0; a < 3; a++) { union_bbox_min[a] = fbmin[a]; union_bbox_max[a] = fbmax[a]; }
                    have_bbox = 1;
                }
                else
                {
                    for (a = 0; a < 3; a++)
                    {
                        if (fbmin[a] < union_bbox_min[a]) union_bbox_min[a] = fbmin[a];
                        if (fbmax[a] > union_bbox_max[a]) union_bbox_max[a] = fbmax[a];
                    }
                }
            }
        }
        memcpy(g_bbox_min, union_bbox_min, sizeof(g_bbox_min));
        memcpy(g_bbox_max, union_bbox_max, sizeof(g_bbox_max));
        g_have_bbox = have_bbox;
    }

    printf("Re-encoded %u of %u file structures' tessellation sections\n", n_processed, file_structure_count);
    if (n_processed == 0)
    {
        printf("Nothing to re-encode, not writing output\n");
        return 1;
    }

    chunk_delta = (int64_t *)calloc(total_bounds > 0 ? total_bounds - 1 : 0, sizeof(int64_t));
    for (m = 0; m + 1 < total_bounds; m++)
    {
        uint32_t old_len = bounds[m+1] - bounds[m];
        int is_replace = (owner_k[m] >= 0 && owner_i[m] == 3 && fs_processed[(uint32_t)owner_k[m]]);
        uint32_t new_len = is_replace ? (uint32_t)new_tess_len[(uint32_t)owner_k[m]] : old_len;
        chunk_delta[m] = (int64_t)new_len - (int64_t)old_len;
    }

    new_bounds = (uint32_t *)malloc(sizeof(uint32_t) * total_bounds);
    running = 0;
    for (m = 0; m < total_bounds; m++)
    {
        new_bounds[m] = (uint32_t)((int64_t)bounds[m] + running);
        if (m + 1 < total_bounds) running += chunk_delta[m];
    }

    new_total_size = (size_t)new_bounds[total_bounds - 1];
    out_buf = (uint8_t *)malloc(new_total_size);

    memcpy(out_buf, buf, header_len);
    for (m = 0; m < total_bounds; m++)
        write_le_u32(out_buf + header_bytepos[m], new_bounds[m]);

    for (m = 0; m + 1 < total_bounds; m++)
    {
        uint32_t old_start = bounds[m];
        uint32_t old_len = bounds[m+1] - bounds[m];
        uint32_t new_start = new_bounds[m];
        int is_replace = (owner_k[m] >= 0 && owner_i[m] == 3 && fs_processed[(uint32_t)owner_k[m]]);
        if (is_replace)
        {
            uint32_t fk = (uint32_t)owner_k[m];
            memcpy(out_buf + new_start, new_tess_bytes[fk], new_tess_len[fk]);
        }
        else
        {
            memcpy(out_buf + new_start, buf + old_start, old_len);
        }
    }

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes, was %ld)\n", argv[2], new_total_size, fsize);

    /* Optional PDF embed with an explicit Default view framing the real
       decoded bbox (union across every re-encoded file structure) --
       prc_to_pdf.exe's NULL-options embed omits /3DV entirely, which a
       synthetic-triangle splice test earlier in this investigation showed
       matters (ruling that out is the whole point of using real geometry
       here instead). */
    if (argc >= 4)
    {
        prc_pdf_view_spec view;
        prc_pdf_write_options pdf_opts;
        double center[3], extent[3], diag_len;
        int a;
        int code;

        if (!g_have_bbox)
        {
            printf("no bbox available (no file structure re-encoded successfully?), embedding without a view\n");
            memset(&pdf_opts, 0, sizeof(pdf_opts));
            code = prc_api_pdf_embed_prc(ctx, argv[3], out_buf, new_total_size, &pdf_opts);
        }
        else
        {
            for (a = 0; a < 3; a++) center[a] = 0.5 * (g_bbox_min[a] + g_bbox_max[a]);
            for (a = 0; a < 3; a++) extent[a] = g_bbox_max[a] - g_bbox_min[a];
            diag_len = sqrt(extent[0] * extent[0] + extent[1] * extent[1] + extent[2] * extent[2]);
            if (diag_len < 1e-6) diag_len = 1.0;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = center[0] + diag_len * 1.0;
            view.eye[1] = center[1] - diag_len * 1.3;
            view.eye[2] = center[2] + diag_len * 0.7;
            view.target[0] = center[0];
            view.target[1] = center[1];
            view.target[2] = center[2];
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;
            code = prc_api_pdf_embed_prc(ctx, argv[3], out_buf, new_total_size, &pdf_opts);
        }
        if (code < 0)
        {
            printf("prc_api_pdf_embed_prc failed: %d\n", code);
            prc_api_print_error_stack(ctx);
        }
        else
        {
            printf("Wrote %s (with explicit Default view)\n", argv[3]);
        }
    }

    for (k = 0; k < file_structure_count; k++)
    {
        free(fs[k].section_offset);
        if (new_tess_bytes[k] != NULL) prc_free(ctx, new_tess_bytes[k]);
    }
    free(fs);
    free(new_tess_bytes);
    free(new_tess_len);
    free(fs_processed);
    free(bounds);
    free(header_bytepos);
    free(owner_k);
    free(owner_i);
    free(chunk_delta);
    free(new_bounds);
    free(out_buf);
    free(buf);
    prc_api_release_context(ctx);
    return 0;
}
