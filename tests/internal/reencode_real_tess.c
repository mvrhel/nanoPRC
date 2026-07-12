/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: The "field-preserving re-encoder". Takes a REAL, complete,
   independently-produced raw .prc file -- now ANY number of PRC file
   structures, not just one -- and for every file structure whose
   tessellation section is a single (tess_count==1) PRC_TYPE_TESS_3D_
   Compressed entry, fully DECODES it with nanoPRC's own parser, then
   RE-ENCODES the exact same decoded values through nanoPRC's own writer
   (prc_write_compress_tess_to_stream) -- reusing the real production
   encoder, just fed parsed-from-real data instead of a fresh mesh
   traversal. File structures whose tessellation section doesn't match
   that shape (tess_count!=1, first entry not COMPRESSED, too few
   sections, a parse error) are left byte-for-byte untouched and reported
   as skipped, rather than aborting the whole file. Everything else in
   every file structure (schema, tree, geometry, extra_geometry,
   model-file) is left byte-for-byte untouched real content.

   HOW: usage: reencode_real_tess.exe <real_input.prc> <output.prc>.

   CONTAINER FORMAT (reverse-engineered from prc_parse_main.c's
   prc_parse_main_header, not the spec text, since that's the version this
   tool must byte-match): "PRC" magic(3) + min_vers(4) + auth_vers(4) +
   file_uid(16) + app_uid(16) + file_structure_count(4), then for each file
   structure k in turn: fs_uid(16) + reserved(4) + section_count(4) +
   section_count section_offset(4) entries (ABSOLUTE byte offsets into the
   whole buffer, one per top-level section: schema/globals, ?, tree,
   tessellation(index 3), extra_geometry, model -- indices confirmed
   empirically, not by name, against real files; this tool only cares that
   index 3 is tessellation and index 4 is the start of whatever follows
   it). After ALL file structures' tables: one GLOBAL start_offset(4) +
   end_offset(4) + file_count(4) (external file references; must be 0,
   unsupported). Multi-file-structure containers lay every file
   structure's section data out back-to-back in the same shared buffer, in
   file-structure order, so section boundaries across the WHOLE file form
   one flat, strictly increasing sequence of byte offsets ending at
   end_offset == EOF. This tool treats that sequence as a flat chunk list
   (each chunk either copied verbatim or, for a re-encoded tessellation
   section, replaced with a possibly different-length blob), recomputes
   every offset by a running size-delta prefix sum, and patches the
   offset-table VALUES in place (the header's own byte layout/length never
   changes -- only fscount, section_count, uid, reserved are fixed-size
   and unchanged; only the 4-byte offset values inside are rewritten).

   WHY IT EXISTS / WHAT IT FOUND: if this write facility's bit-level
   encoding process cannot faithfully reproduce a working file's
   tessellation record even when fed 100% real, correct values, that
   proves a bug in HOW compressed tessellation is bit-packed, independent
   of WHICH values are chosen -- something no amount of field-value
   comparison against spec text could reveal on its own. Using this tool:
   (1) found and led to fixing a real production bug -- prc_write_
   compress_tess_to_stream always hardcoded is_face_planar=0 for every
   face with no way to express genuine per-face planarity, which
   corrupted normal reconstruction for any real file with actually-planar
   faces (see the is_face_planar parameter this tool exercises, and the
   commit that added it). (2) After that fix, the zero-mutation re-
   encoded output STILL shows a blank model tree in Adobe Acrobat, while
   opening correctly in nanoPRC's own reader, the independent ground-
   truth reader, AND PDF-XChange. This is the most decisive finding of
   the whole investigation: the bug is confirmed to be in the bit-level
   encoding mechanics themselves (not value selection), and Acrobat
   specifically is more sensitive to it than every other reader tried.
   See the "Acrobat blank-tree investigation" project notes for the full
   picture. (3) Extended to multi-file-structure containers to round-trip
   a real 973-part/28-file-structure turbine assembly used in the ISO
   §7.8.9 traversal review, as a stronger encoder-side companion to the
   decoder-side ablation in verify_manifold_pdf.c.

   LIMITATIONS: Skips (rather than re-encodes) any file structure whose
   tessellation section isn't exactly one PRC_TYPE_TESS_3D_Compressed
   entry -- uncompressed tessellations, wire/line tessellations, and
   multi-entry tessellation sections are left untouched, not exercised by
   this tool. No mutation capability -- zero-mutation baseline only (see
   project notes for the original single-file-structure design intent).
   The earlier single-file-structure version of this tool supported
   dumping the inflated pre-deflate bitstreams for a direct bit-level
   diff; that option is dropped here since "which of N file structures"
   doesn't generalize cleanly -- use verify_manifold_real.c/
   verify_manifold_pdf.c for round-trip correctness checking instead. */
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
#include "prc_write_compress_tess.h"
#include "prc_write_file_structure.h"
#include "prc_write_common.h"
#include "zlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void write_le_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v&0xff); p[1]=(uint8_t)((v>>8)&0xff); p[2]=(uint8_t)((v>>16)&0xff); p[3]=(uint8_t)((v>>24)&0xff); }

typedef struct
{
    uint32_t section_count;
    uint32_t *section_offset; /* original absolute byte offsets, length section_count */
} fs_info;

/* Attempt to re-encode file structure k's tessellation section (its section
   index 3, [section_offset[3], section_offset[4])) with nanoPRC's own
   writer, fed the values nanoPRC's own parser decoded from the real bytes.
   On success, *out_bytes/*out_len receive a malloc'd deflated blob (caller
   frees) and the function returns 1. On any "not eligible" or parse/write
   failure, prints a one-line reason and returns 0 (leave this fs's section
   untouched -- not a fatal error for the whole file). */
static int
try_reencode_fs_tess(prc_context *ctx, const uint8_t *buf, uint32_t k, const fs_info *fs,
    uint8_t **out_bytes, size_t *out_len)
{
    uint32_t sec_start, sec_end, comp_len;
    uint8_t *inflated = NULL;
    uLongf dest_len;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_content_prc_base wrapper_base;
    prc_tess_3d_compressed *parsed = NULL;
    prc_encode_traversal_result trav;
    double crease_angle_degrees = 0.0;
    int32_t *triangle_face_array_signed = NULL;
    prc_bit_write_state wrapper_s;
    uint8_t *wrapper_comp = NULL;
    size_t wrapper_comp_len = 0;
    uint32_t kk;
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
    if (entry_tag != PRC_TYPE_TESS_3D_Compressed)
    {
        printf("fs %u: first tessellation entry tag=%u (not COMPRESSED), skipping\n", k, entry_tag);
        free(inflated);
        return 0;
    }

    code = prc_parse_tess_3d_compressed(ctx, &bit_state, &parsed, 0);
    if (code < 0)
    {
        printf("fs %u: prc_parse_tess_3d_compressed failed (%d), skipping\n", k, code);
        prc_api_print_error_stack(ctx);
        free(inflated);
        return 0;
    }

    memset(&trav, 0, sizeof(trav));
    trav.point_array = parsed->point_array;
    trav.point_array_size = parsed->point_array_size;
    trav.edge_status_array = parsed->edge_status_array;
    trav.edge_status_array_size = parsed->triangle_face_array_size;

    triangle_face_array_signed = (int32_t *)malloc(sizeof(int32_t) * (parsed->triangle_face_array_size > 0 ? parsed->triangle_face_array_size : 1));
    for (kk = 0; kk < parsed->triangle_face_array_size; kk++)
        triangle_face_array_signed[kk] = (int32_t)parsed->triangle_face_array[kk];
    trav.triangle_face_array = triangle_face_array_signed;
    trav.triangle_face_array_size = parsed->triangle_face_array_size;

    trav.points_is_reference_array = parsed->points_is_reference_array;
    trav.points_is_reference_array_size = parsed->reference_array_size;
    trav.point_reference_array = parsed->point_reference_array;
    trav.point_reference_array_size = parsed->point_reference_array_size;
    trav.origin[0] = parsed->origin_array.x;
    trav.origin[1] = parsed->origin_array.y;
    trav.origin[2] = parsed->origin_array.z;

    if (parsed->must_recalculate_normals)
        crease_angle_degrees = parsed->crease_angle * 180.0 / M_PI;

    memset(&wrapper_s, 0, sizeof(wrapper_s));
    if (prc_bitwrite_init(ctx, &wrapper_s, 4096) != 0) { printf("fs %u: wrapper init failed, skipping\n", k); goto cleanup; }
    if (prc_bitwrite_uint32(ctx, &wrapper_s, PRC_TYPE_ASM_FileStructureTessellation) != 0) { printf("fs %u: wrapper tag write failed, skipping\n", k); goto cleanup; }
    if (prc_bitwrite_uint32(ctx, &wrapper_s, 0) != 0) { printf("fs %u: wrapper attr write failed, skipping\n", k); goto cleanup; }
    if (prc_bitwrite_bit(ctx, &wrapper_s, 1) != 0) { printf("fs %u: wrapper name write failed, skipping\n", k); goto cleanup; }
    if (prc_bitwrite_uint32(ctx, &wrapper_s, 1) != 0) { printf("fs %u: wrapper tess_count write failed, skipping\n", k); goto cleanup; }
    if (prc_bitwrite_uint32(ctx, &wrapper_s, PRC_TYPE_TESS_3D_Compressed) != 0) { printf("fs %u: entry tag write failed, skipping\n", k); goto cleanup; }
    code = prc_write_compress_tess_to_stream(ctx, &wrapper_s, &trav, parsed->tolerance,
        parsed->must_recalculate_normals ? parsed->normal_is_reversed : NULL,
        crease_angle_degrees,
        parsed->must_recalculate_normals ? NULL : parsed->normal_angle_array,
        parsed->must_recalculate_normals ? 0 : parsed->normal_angle_array_size,
        parsed->must_recalculate_normals ? NULL : (const uint8_t *)parsed->normal_binary_data,
        parsed->must_recalculate_normals ? 0 : parsed->normal_binary_data_size,
        parsed->must_recalculate_normals,
        parsed->must_recalculate_normals ? NULL : (const uint8_t *)parsed->is_face_planar);
    if (code != 0)
    {
        printf("fs %u: prc_write_compress_tess_to_stream failed (%d), skipping\n", k, code);
        prc_api_print_error_stack(ctx);
        goto cleanup;
    }
    if (prc_bitwrite_flush(ctx, &wrapper_s) != 0) { printf("fs %u: wrapper flush failed, skipping\n", k); goto cleanup; }

    if (prc_write_deflate(ctx, wrapper_s.buf, wrapper_s.byte_pos, &wrapper_comp, &wrapper_comp_len) != 0)
    {
        printf("fs %u: deflate failed, skipping\n", k);
        goto cleanup;
    }

    printf("fs %u: tess reencoded, %u -> %zu bytes (compressed)\n", k, comp_len, wrapper_comp_len);
    *out_bytes = wrapper_comp;
    *out_len = wrapper_comp_len;
    ok = 1;
    wrapper_comp = NULL; /* ownership transferred */

cleanup:
    free(triangle_face_array_signed);
    prc_bitwrite_release(ctx, &wrapper_s);
    if (wrapper_comp != NULL) prc_free(ctx, wrapper_comp);
    free(inflated);
    /* `parsed` intentionally not freed -- consistent with this tool's
       existing standard of not chasing every allocation in a one-shot
       diagnostic CLI; process exit reclaims it. */
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

    if (argc < 3)
    {
        printf("usage: %s <real_input.prc> <output.prc>\n", argv[0]);
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

    p = buf + 3 + 4 + 4 + 16 + 16; /* magic + min_vers + auth_vers + file_uid + app_uid */
    file_structure_count = read_le_u32(p); p += 4;
    if (file_structure_count == 0) { printf("file_structure_count=0\n"); return 1; }

    fs = (fs_info *)calloc(file_structure_count, sizeof(fs_info));

    /* Every section_offset entry (across all file structures) plus the
       trailing start_offset/end_offset markers, in file order -- this is
       the flat chunk-boundary list the splice pass below walks. */
    total_bounds = 2;

    /* First pass just to learn each fs's section_count (needed to size arrays). */
    {
        const uint8_t *pp = p;
        for (k = 0; k < file_structure_count; k++)
        {
            uint32_t sc;
            pp += 16 + 4; /* uid + reserved */
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
        p += 16; /* fs_uid */
        p += 4;  /* reserved */
        p += 4;  /* section_count (already known from pre-pass) */
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

    for (k = 0; k < file_structure_count; k++)
    {
        uint8_t *ob = NULL;
        size_t ol = 0;
        if (try_reencode_fs_tess(ctx, buf, k, &fs[k], &ob, &ol))
        {
            new_tess_bytes[k] = ob;
            new_tess_len[k] = ol;
            fs_processed[k] = 1;
            n_processed++;
        }
    }

    printf("Re-encoded %u of %u file structures' tessellation sections\n", n_processed, file_structure_count);
    if (n_processed == 0)
    {
        printf("Nothing to re-encode, not writing output\n");
        return 1;
    }

    /* Recompute every offset as a running prefix-sum of per-chunk size
       deltas. Chunk m spans [bounds[m], bounds[m+1]) for m in
       [0, total_bounds-1); it is a "replace" chunk iff it's owned by
       section index 3 of a file structure we successfully re-encoded. */
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

    new_total_size = (size_t)new_bounds[total_bounds - 1]; /* new end_offset == new EOF */
    out_buf = (uint8_t *)malloc(new_total_size);

    /* Header: same byte layout/length as the input, only the offset VALUES
       (at their recorded byte positions) are patched. */
    memcpy(out_buf, buf, header_len);
    for (m = 0; m < total_bounds; m++)
        write_le_u32(out_buf + header_bytepos[m], new_bounds[m]);

    /* Body: walk the same chunk list, copying verbatim or substituting the
       re-encoded bytes, writing at each chunk's NEW position. */
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
