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

/* Phase 1d, session 2 gate test: the first complete, real, on-disk .prc
   file produced by the write facility (prc_write_model.c orchestrating
   prc_write_tree.c / prc_write_file_structure.c / prc_write_global.c /
   prc_write_tess_3d.c), round-tripped through the real, unmodified
   prc_api_open_contents. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_api.h"
#include "prc_write_global.h"
#include "prc_write_tree.h"
#include "prc_write_file_structure.h"
#include "prc_write_model.h"
#include "zlib.h"

static const char *TEST_PRC_FILENAME = "test_file_structure_output.prc";

static uint32_t
read_le_uint32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void
build_one_triangle_file(prc_context *ctx, prc_write_global_tables *tables)
{
    double positions[3 * 3] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, tables), 0);

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 3;
    tess_entry.tri_indices = tris;
    tess_entry.num_triangles = 1;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;
    /* normals == NULL -> TriangleOneNormal path, computed face normal */

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1; /* biased index into the one tess entry */
    ri.is_closed = 0;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_min[0] = 0.0; root.bbox_min[1] = 0.0; root.bbox_min[2] = 0.0;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0; root.bbox_max[2] = 0.0;

    PRC_ASSERT_EQ(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, tables, &root, &tess_entry, 1), 0);
}

static void
test_one_triangle_roundtrip(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_data *pd;

    printf("  sub-case: minimal 1-triangle PRC round trip via prc_api_open_contents\n");

    build_one_triangle_file(ctx, &tables);

    pd = (prc_data *)prc_api_open_contents(ctx, TEST_PRC_FILENAME);
    if (pd == NULL)
        prc_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(pd);

    PRC_ASSERT_EQ(pd->file_structure_count, 1);
    PRC_ASSERT_NOT_NULL(pd->file_struct);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].tessellation);
    PRC_ASSERT_EQ(pd->file_struct[0].tessellation->tess_count, 1);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].tree);
    PRC_ASSERT_EQ(pd->file_struct[0].tree->parts_count, 1);
    PRC_ASSERT_EQ(pd->file_struct[0].tree->product_count, 1);

    prc_api_release_data(ctx, (prc_api_data)pd, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_write_global_tables_free(ctx, &tables);
}

static void
test_prc_signature(prc_context *ctx)
{
    prc_write_global_tables tables;
    FILE *fid;
    uint8_t sig[3];

    printf("  sub-case: PRC magic bytes at offset 0\n");

    build_one_triangle_file(ctx, &tables);

    fid = fopen(TEST_PRC_FILENAME, "rb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fread(sig, 1, 3, fid), 3);
    fclose(fid);

    PRC_ASSERT_EQ(sig[0], 'P');
    PRC_ASSERT_EQ(sig[1], 'R');
    PRC_ASSERT_EQ(sig[2], 'C');

    prc_write_global_tables_free(ctx, &tables);
}

/* Manually locates the model section (addressed via the main header's
   start_offset/end_offset) and independently confirms it is a valid zlib
   deflate stream whose decompressed size matches a freshly-encoded copy of
   the same content. The field byte positions come from
   prc_write_main_header_compute_layout -- the same layout function
   prc_write_model.c's own writer uses -- rather than hardcoded byte
   offsets, so this test can't silently desync from the real layout if the
   section count (PRC_WRITE_PRC_FILE_SECTION_COUNT) ever changes. */
static void
test_zlib_section_valid(prc_context *ctx)
{
    prc_write_global_tables tables;
    FILE *fid;
    long file_size;
    uint8_t *file_bytes;
    uint32_t start_offset, end_offset;
    prc_write_main_header_layout layout;
    z_stream strm;
    uint8_t *inflated;
    size_t inflated_cap;
    int zret;
    prc_bit_write_state expect_s;
    uint32_t root_biased_index_expected = 1; /* single-node tree: root is the only (last) product */

    printf("  sub-case: zlib-compressed model section is valid\n");

    build_one_triangle_file(ctx, &tables);

    fid = fopen(TEST_PRC_FILENAME, "rb");
    PRC_ASSERT_NOT_NULL(fid);
    fseek(fid, 0, SEEK_END);
    file_size = ftell(fid);
    fseek(fid, 0, SEEK_SET);
    file_bytes = (uint8_t *)malloc((size_t)file_size);
    PRC_ASSERT_NOT_NULL(file_bytes);
    PRC_ASSERT_EQ(fread(file_bytes, 1, (size_t)file_size, fid), (size_t)file_size);
    fclose(fid);

    prc_write_main_header_compute_layout(PRC_WRITE_PRC_FILE_SECTION_COUNT, &layout);
    PRC_ASSERT((long)layout.total_size < file_size);
    start_offset = read_le_uint32(file_bytes + layout.start_offset_pos);
    end_offset = read_le_uint32(file_bytes + layout.end_offset_pos);
    PRC_ASSERT(end_offset > start_offset);
    PRC_ASSERT((long)end_offset <= file_size);

    inflated_cap = 4096;
    inflated = (uint8_t *)malloc(inflated_cap);
    PRC_ASSERT_NOT_NULL(inflated);

    memset(&strm, 0, sizeof(strm));
    PRC_ASSERT_EQ(inflateInit(&strm), Z_OK);
    strm.next_in = file_bytes + start_offset;
    strm.avail_in = end_offset - start_offset;
    strm.next_out = inflated;
    strm.avail_out = (uInt)inflated_cap;

    zret = inflate(&strm, Z_FINISH);
    PRC_ASSERT_EQ(zret, Z_STREAM_END);
    inflateEnd(&strm);

    /* Cross-check against an independently-produced copy of the same
       section content (same encoder, fresh call -- not just re-reading
       what was already written). */
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &expect_s, 256), 0);
    PRC_ASSERT_EQ(prc_write_model_file_to_stream(ctx, &expect_s, NULL, root_biased_index_expected, 1), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &expect_s), 0);

    PRC_ASSERT_EQ(strm.total_out, expect_s.byte_pos);
    PRC_ASSERT_EQ(memcmp(inflated, expect_s.buf, expect_s.byte_pos), 0);

    prc_bitwrite_release(ctx, &expect_s);
    free(inflated);
    free(file_bytes);
    prc_write_global_tables_free(ctx, &tables);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("file structure / model file writer, first complete .prc file");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_one_triangle_roundtrip(ctx);
    test_prc_signature(ctx);
    test_zlib_section_valid(ctx);

    prc_release_context(ctx);
    remove(TEST_PRC_FILENAME);

    PRC_TEST_END;
}
