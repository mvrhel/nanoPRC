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

   WHAT: Takes a REAL, independently-produced, complete raw .prc file and
   rebuilds it keeping every section (file-struct-header, schema/globals,
   tree, tessellation, geometry, extra_geometry) byte-for-byte identical
   EXCEPT the model-file section, which is regenerated using this write
   facility's OWN encoder (prc_write_model_file_to_stream_ex), pointed at
   the real file structure's own unique_id/root_index so the reference
   still resolves correctly against the real, untouched tree section.

   HOW: usage: splice_model_section.exe <real_input.prc> <root_biased_index>
   <output.prc>. root_biased_index must be the real file's own model-file
   root_index value (readable via scan_prc's DEBUG2 trace, or by trial:
   it's usually the real file's total product count, since the write
   facility's own tree encoder always places the root product last).
   Hand-parses the raw (non-bit-packed) PRC main header to find each
   section's byte offset, regenerates only the model-file section's bits,
   and rewrites the main header's section_offset/start_offset/end_offset
   table to match the new total layout -- every other section's bytes are
   memcpy'd verbatim from the input.

   WHY IT EXISTS: This isolates whether a reader's problem with this
   write facility's output is specifically the model-file section's bit
   encoding, independent of any content differences -- everything else in
   the output file is untouched, real, proven-working bytes. This was the
   FIRST section-splice test in the Acrobat blank-model-tree investigation
   and the first to definitively clear a whole section (model-file) as a
   suspect: real content + this write facility's own model-file section
   opened correctly in Acrobat, proving the model-file encoder was not
   the bug.

   LIMITATIONS: Only handles single-file-structure inputs (fs_count == 1,
   true of every real-world file tested so far). Assumes the real input's
   own root_biased_index is already known by the caller -- does not
   compute or verify it. No sanity check that the given root_biased_index
   actually matches the real tree section's product count; a wrong value
   produces a file that will fail to resolve its root reference, not a
   clear error at splice time. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_write_model.h"
#include "prc_write_common.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t fs_count, fs_uid[4], reserved, section_count;
    uint32_t *section_offsets;
    uint32_t start_offset, end_offset;
    uint32_t root_biased_index;
    prc_bit_write_state model_s;
    uint8_t *model_comp = NULL;
    size_t model_comp_len = 0;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    size_t out_pos;
    uint32_t i;
    FILE *out_fid;

    if (argc < 4)
    {
        printf("usage: %s <real_input.prc> <root_biased_index> <output.prc>\n", argv[0]);
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

    p = buf + 3;
    p += 4; /* min_vers_for_read */
    p += 4; /* auth_vers */
    p += 16; /* main unique_id_file */
    p += 16; /* main unique_id_application */
    fs_count = read_le_u32(p); p += 4;
    if (fs_count != 1) { printf("only single-file-structure inputs supported, got %u\n", fs_count); return 1; }

    fs_uid[0] = read_le_u32(p + 0);
    fs_uid[1] = read_le_u32(p + 4);
    fs_uid[2] = read_le_u32(p + 8);
    fs_uid[3] = read_le_u32(p + 12);
    p += 16;
    reserved = read_le_u32(p); p += 4;
    (void)reserved;
    section_count = read_le_u32(p); p += 4;

    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
    start_offset = read_le_u32(p); p += 4;
    end_offset = read_le_u32(p); p += 4;
    /* file_count field follows, unused here */

    printf("Parsed real file: fs_uid=[%u %u %u %u] section_count=%u start_offset=%u end_offset=%u total_size=%ld\n",
        fs_uid[0], fs_uid[1], fs_uid[2], fs_uid[3], section_count, start_offset, end_offset, fsize);

    root_biased_index = (uint32_t)atoi(argv[2]);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    memset(&model_s, 0, sizeof(model_s));
    if (prc_bitwrite_init(ctx, &model_s, 256) != 0) { printf("bitwrite_init failed\n"); return 1; }
    if (prc_write_model_file_to_stream_ex(ctx, &model_s, NULL, root_biased_index, 1, fs_uid) != 0)
    {
        printf("prc_write_model_file_to_stream_ex failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }
    if (prc_bitwrite_flush(ctx, &model_s) != 0) { printf("bitwrite_flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, model_s.buf, model_s.byte_pos, &model_comp, &model_comp_len) != 0)
    {
        printf("prc_write_deflate failed\n");
        return 1;
    }
    printf("Generated replacement model-file section: %zu bytes compressed (real was %u bytes)\n",
        model_comp_len, end_offset - start_offset);

    /* Rebuild: keep every byte up to start_offset identical (main header +
       file-struct-header + schema + tree + tess + geometry + extra_geometry
       sections, all real), then append OUR model-file section, then patch
       only the start_offset/end_offset dwords (section_offsets[] are
       unchanged -- nothing before start_offset moved). */
    new_section_offsets = section_offsets; /* unchanged */
    new_start_offset = start_offset; /* unchanged: everything before it is untouched */
    new_end_offset = new_start_offset + (uint32_t)model_comp_len;
    new_total_size = (size_t)new_end_offset;

    out_buf = (uint8_t *)malloc(new_total_size);
    memcpy(out_buf, buf, start_offset); /* everything up to and including all real sections */
    memcpy(out_buf + new_start_offset, model_comp, model_comp_len);

    /* Patch header's start_offset/end_offset fields in place (same byte
       positions as the original -- section_offsets[] table size/position
       didn't change, only what comes after start_offset). Recompute their
       byte offset the same way the header was laid out: 3+4+4+16+16+4
       (=47) + 16+4+4 (=fs_uid+reserved+section_count, =71) + 4*section_count,
       then start_offset dword, then end_offset dword. */
    {
        size_t off = 3 + 4 + 4 + 16 + 16 + 4 + 16 + 4 + 4 + (size_t)4 * section_count;
        out_buf[off + 0] = (uint8_t)(new_start_offset & 0xff);
        out_buf[off + 1] = (uint8_t)((new_start_offset >> 8) & 0xff);
        out_buf[off + 2] = (uint8_t)((new_start_offset >> 16) & 0xff);
        out_buf[off + 3] = (uint8_t)((new_start_offset >> 24) & 0xff);
        out_buf[off + 4] = (uint8_t)(new_end_offset & 0xff);
        out_buf[off + 5] = (uint8_t)((new_end_offset >> 8) & 0xff);
        out_buf[off + 6] = (uint8_t)((new_end_offset >> 16) & 0xff);
        out_buf[off + 7] = (uint8_t)((new_end_offset >> 24) & 0xff);
    }
    (void)new_section_offsets;

    out_fid = fopen(argv[3], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[3]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes)\n", argv[3], new_total_size);

    prc_free(ctx, model_comp);
    prc_bitwrite_release(ctx, &model_s);
    prc_api_release_context(ctx);
    free(section_offsets);
    free(out_buf);
    free(buf);
    return 0;
}
