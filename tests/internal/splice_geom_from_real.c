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

   WHAT: Takes OUR OWN complete, self-consistent raw .prc file (schema+
   tree+tessellation+geometry+extra_geometry+model-file, all written by
   this write facility) and replaces ONLY the geometry and extra_geometry
   sections with a REAL, independently-produced file's own bytes for
   those two sections, keeping everything else (schema, tree,
   tessellation, model-file) exactly as this write facility's own writer
   produced it.

   HOW: usage: splice_geom_from_real.exe <ours.prc> <real.prc>
   <output.prc>. Pure byte-level section splicer -- does NOT link against
   nanoPRC at all (no prc_static dependency; hand-parses the raw main-
   header/section-offset table directly), since geometry/extra_geometry
   are always trivially empty stubs (topo_context_count / extra_geom_
   count == 0) for both this write facility and real producers, so no
   cross-referencing indices are at stake and no re-encoding is needed --
   just a byte copy plus offset-table patching.

   WHY IT EXISTS: This was the last section never yet substituted with
   real content in the splice-test investigation -- isolates whether this
   write facility's own encoding of these two nominally-empty sections
   (as opposed to their real counterparts) contributes to the Acrobat
   blank-model-tree bug. Result: no effect either way, ruling geometry/
   extra_geometry out as a suspect (real bytes here did not fix an
   otherwise-failing all-our-own-content file).

   LIMITATIONS: Only handles single-file-structure inputs on both sides.
   Assumes both inputs' geometry+extra_geometry sections are genuinely
   trivial/empty -- would silently produce a broken file if either input
   had real exact-B-Rep geometry content with actual cross-references
   into other sections (not a concern for any file used in this
   investigation, all tessellation-only). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void write_le_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v&0xff); p[1]=(uint8_t)((v>>8)&0xff); p[2]=(uint8_t)((v>>16)&0xff); p[3]=(uint8_t)((v>>24)&0xff); }

typedef struct {
    uint8_t *buf;
    long size;
    uint32_t section_count;
    uint32_t *section_offsets;
    uint32_t start_offset, end_offset;
    size_t header_prefix_len;
    size_t section_off_table_byte_pos;
} prc_file_info;

static int load_prc(const char *path, prc_file_info *info)
{
    FILE *fid;
    size_t read_size;
    const uint8_t *p;
    uint32_t i;

    fid = fopen(path, "rb");
    if (fid == NULL) { printf("failed to open %s\n", path); return -1; }
    fseek(fid, 0L, SEEK_END);
    info->size = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    info->buf = (uint8_t *)malloc((size_t)info->size);
    read_size = fread(info->buf, 1, (size_t)info->size, fid);
    fclose(fid);
    if (read_size != (size_t)info->size) { printf("short read on %s\n", path); return -1; }
    if (memcmp(info->buf, "PRC", 3) != 0) { printf("%s: not a raw PRC file\n", path); return -1; }

    p = info->buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("%s: only single-file-structure inputs supported\n", path); return -1; }
    p += 4;
    p += 16; /* fs_uid */
    p += 4;  /* reserved */
    info->section_off_table_byte_pos = (size_t)(p - info->buf) + 4;
    info->section_count = read_le_u32(p); p += 4;
    info->header_prefix_len = (size_t)(p - info->buf);

    info->section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * info->section_count);
    for (i = 0; i < info->section_count; i++) { info->section_offsets[i] = read_le_u32(p); p += 4; }
    info->start_offset = read_le_u32(p); p += 4;
    info->end_offset = read_le_u32(p); p += 4;
    return 0;
}

int main(int argc, char **argv)
{
    prc_file_info ours, real;
    uint32_t i;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    FILE *out_fid;
    uint32_t real_geom_len, real_extra_geom_len;

    if (argc < 4)
    {
        printf("usage: %s <ours.prc> <real.prc> <output.prc>\n", argv[0]);
        return 2;
    }

    if (load_prc(argv[1], &ours) != 0) return 1;
    if (load_prc(argv[2], &real) != 0) return 1;

    if (ours.section_count < 6 || real.section_count < 6)
    {
        printf("expected section_count>=6 in both files (ours=%u real=%u)\n", ours.section_count, real.section_count);
        return 1;
    }

    real_geom_len = real.section_offsets[5] - real.section_offsets[4];
    real_extra_geom_len = real.start_offset - real.section_offsets[5];
    printf("Ours: geometry=%u bytes, extra_geometry=%u bytes\n",
        ours.section_offsets[5] - ours.section_offsets[4], ours.start_offset - ours.section_offsets[5]);
    printf("Real: geometry=%u bytes, extra_geometry=%u bytes\n", real_geom_len, real_extra_geom_len);

    /* Rebuild ours, keeping [0..4) (file-struct-header, schema, tree,
       tessellation) unchanged, substituting real's geometry+extra_geometry
       bytes at [4..start_offset), then our own model-file section
       unchanged after that (shifted by the size delta). */
    new_section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * ours.section_count);
    for (i = 0; i <= 4; i++) new_section_offsets[i] = ours.section_offsets[i]; /* [0..4] unchanged (geometry START doesn't move) */
    new_section_offsets[5] = ours.section_offsets[4] + real_geom_len;
    new_start_offset = new_section_offsets[5] + real_extra_geom_len;
    new_end_offset = new_start_offset + (ours.end_offset - ours.start_offset); /* our model-file section length, unchanged */
    new_total_size = (size_t)new_end_offset;

    out_buf = (uint8_t *)malloc(new_total_size);
    memcpy(out_buf, ours.buf, ours.header_prefix_len);
    {
        uint8_t *q = out_buf + ours.section_off_table_byte_pos;
        size_t real_trailer_pos = ours.section_off_table_byte_pos + (size_t)4 * ours.section_count + 4 + 4;
        for (i = 0; i < ours.section_count; i++) { write_le_u32(q, new_section_offsets[i]); q += 4; }
        write_le_u32(q, new_start_offset); q += 4;
        write_le_u32(q, new_end_offset); q += 4;
        memcpy(q, ours.buf + real_trailer_pos, 4);
    }
    /* our file-struct-header + schema + tree + tessellation (sections 0..3), unchanged */
    memcpy(out_buf + new_section_offsets[0], ours.buf + ours.section_offsets[0], ours.section_offsets[4] - ours.section_offsets[0]);
    /* real geometry section bytes */
    memcpy(out_buf + new_section_offsets[4], real.buf + real.section_offsets[4], real_geom_len);
    /* real extra_geometry section bytes */
    memcpy(out_buf + new_section_offsets[5], real.buf + real.section_offsets[5], real_extra_geom_len);
    /* our model-file section, unchanged content, shifted position */
    memcpy(out_buf + new_start_offset, ours.buf + ours.start_offset, ours.end_offset - ours.start_offset);

    out_fid = fopen(argv[3], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[3]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes)\n", argv[3], new_total_size);

    free(ours.buf); free(ours.section_offsets);
    free(real.buf); free(real.section_offsets);
    free(new_section_offsets);
    free(out_buf);
    return 0;
}
