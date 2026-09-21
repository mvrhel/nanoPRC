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

/* A textured quad written through the public write struct and read back with
   the real parser. prc_write_tess_3d already has its own texture case in
   test_tess_3d.c; this one exists to prove the coordinates survive the whole
   public path -- prc_api_write_tessellation -> prc_write_file_structure ->
   the encoder -> deflate -> prc_api_open_contents -- because the fields were
   present on the encoder for some time while the public struct had no way to
   reach them, and a capability nothing can call is indistinguishable from one
   that does not work.

   Three UV pairs for four corners, deliberately: it forces the texture index
   stream to diverge from the position index stream. With one UV per corner
   the two run in parallel, and a writer that emitted texture indices into the
   position slot, or scaled them by 3 instead of 2, would still round-trip and
   the case would prove nothing. The assertion below checks the streams really
   do differ before trusting anything else. */

/* build_one_triangle_file, plus two embedded uncompressed files added before
   the write so they land in the file-structure header. */
static void
build_one_triangle_file_with_files(prc_context *ctx, prc_write_global_tables *tables,
    const uint8_t *a, uint32_t a_size, const uint8_t *b, uint32_t b_size,
    uint32_t *idx_a, uint32_t *idx_b)
{
    double positions[3 * 3] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, tables), 0);

    *idx_a = prc_write_embedded_file_add(ctx, tables, a, a_size);
    *idx_b = prc_write_embedded_file_add(ctx, tables, b, b_size);

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 3;
    tess_entry.tri_indices = tris;
    tess_entry.num_triangles = 1;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0;

    PRC_ASSERT_EQ(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, tables, &root, &tess_entry, 1), 0);
}

/* Embedded uncompressed files in the file-structure header. This is where
   raster images live -- prc_parse_main.c says so at the read site -- and a
   prc_graph_picture reaches them through biased_uncompressed_file_index.
   Nothing wrote them before; file_count was a hardcoded 0.

   The header stops being fixed-size once blocks are present, and every
   section offset after it is computed from its length, so the risk is that a
   block shifts the file and the offsets do not follow. That would not fail
   here -- it would fail somewhere later and look unrelated -- so this case
   checks the bytes came back AND that the rest of the file still parses. */
static void
test_embedded_files(prc_context *ctx)
{
    static const uint8_t blob_a[5] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
    static const uint8_t blob_b[3] = { 0x11, 0x22, 0x33 };
    prc_write_global_tables tables;
    prc_data *pd;
    uint32_t idx_a, idx_b;
    uint32_t k;

    printf("  sub-case: embedded uncompressed files round trip\n");

    /* Two blocks of DIFFERENT lengths, because equal lengths would pass
       against a writer that used a fixed stride, and the second's index must
       be 2 -- a writer returning the count before appending would give 1
       twice and both pictures would point at the same image. */
    build_one_triangle_file_with_files(ctx, &tables, blob_a, sizeof(blob_a),
                                       blob_b, sizeof(blob_b), &idx_a, &idx_b);
    PRC_ASSERT_EQ(idx_a, 1);
    PRC_ASSERT_EQ(idx_b, 2);

    pd = (prc_data *)prc_api_open_contents(ctx, TEST_PRC_FILENAME);
    if (pd == NULL)
        prc_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(pd);

    PRC_ASSERT_EQ(pd->file_structure_count, 1);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].header);
    PRC_ASSERT_EQ(pd->file_struct[0].header->file_count, 2);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].header->files);

    PRC_ASSERT_EQ(pd->file_struct[0].header->files[0].block_size, sizeof(blob_a));
    for (k = 0; k < sizeof(blob_a); k++)
        PRC_ASSERT_EQ(pd->file_struct[0].header->files[0].block[k], blob_a[k]);

    PRC_ASSERT_EQ(pd->file_struct[0].header->files[1].block_size, sizeof(blob_b));
    for (k = 0; k < sizeof(blob_b); k++)
        PRC_ASSERT_EQ(pd->file_struct[0].header->files[1].block[k], blob_b[k]);

    /* The blocks shifted every section after the header. If the offsets did
       not follow, the tessellation is where the geometry should be and this
       is where it shows. */
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].tessellation);
    PRC_ASSERT_EQ(pd->file_struct[0].tessellation->tess_count, 1);

    prc_api_release_data(ctx, (prc_api_data)pd, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_write_global_tables_free(ctx, &tables);
    remove(TEST_PRC_FILENAME);
}

static void
build_textured_quad_file(prc_context *ctx, prc_write_global_tables *tables)
{
    static const double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    static const double normals[3] = { 0.0, 0.0, 1.0 };
    static const double tex_coords[3 * 2] = {
        0.0, 0.0,
        1.0, 0.0,
        0.5, 1.0
    };
    static const uint32_t tris[6]     = { 0, 1, 2,  0, 2, 3 };
    static const uint32_t norm_idx[6] = { 0, 0, 0,  0, 0, 0 };
    static const uint32_t tex_idx[6]  = { 0, 1, 2,  0, 2, 1 };
    static const uint32_t face_tri_counts[1] = { 2 };
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;
    int streams_differ = 0;
    int k;

    for (k = 0; k < 6; k++)
        if (tex_idx[k] != tris[k])
            streams_differ = 1;
    PRC_ASSERT(streams_differ);

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, tables), 0);

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 4;
    tess_entry.normals = normals;
    tess_entry.num_normals = 1;
    tess_entry.tri_indices = tris;
    tess_entry.norm_indices = norm_idx;
    tess_entry.num_triangles = 2;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;
    tess_entry.tex_coords = tex_coords;
    tess_entry.num_tex_coords = 3;   /* PAIRS, not doubles */
    tess_entry.tex_indices = tex_idx;

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;
    ri.is_closed = 0;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_min[0] = 0.0; root.bbox_min[1] = 0.0; root.bbox_min[2] = 0.0;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0; root.bbox_max[2] = 0.0;

    PRC_ASSERT_EQ(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, tables, &root, &tess_entry, 1), 0);
}

static void
build_textured_quad_file_with_image(prc_context *ctx, prc_write_global_tables *tables,
    const uint8_t *pixels, size_t pixels_size, uint32_t w, uint32_t h)
{
    static const double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    static const double normals[3] = { 0.0, 0.0, 1.0 };
    static const double tex_coords[3 * 2] = {
        0.0, 0.0,
        1.0, 0.0,
        0.5, 1.0
    };
    static const uint32_t tris[6]     = { 0, 1, 2,  0, 2, 3 };
    static const uint32_t norm_idx[6] = { 0, 0, 0,  0, 0, 0 };
    static const uint32_t tex_idx[6]  = { 0, 1, 2,  0, 2, 1 };
    static const uint32_t face_tri_counts[1] = { 2 };
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;
    int streams_differ = 0;
    int k;

    for (k = 0; k < 6; k++)
        if (tex_idx[k] != tris[k])
            streams_differ = 1;
    PRC_ASSERT(streams_differ);

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, tables), 0);

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 4;
    tess_entry.normals = normals;
    tess_entry.num_normals = 1;
    tess_entry.tri_indices = tris;
    tess_entry.norm_indices = norm_idx;
    tess_entry.num_triangles = 2;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;
    tess_entry.tex_coords = tex_coords;
    tess_entry.num_tex_coords = 3;   /* PAIRS, not doubles */
    tess_entry.tex_indices = tex_idx;

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;
    ri.is_closed = 0;
    ri.has_material = 1;
    ri.material_color[0] = ri.material_color[1] = ri.material_color[2] = 1.0;
    ri.material_alpha = 1.0;
    ri.has_texture = 1;
    ri.texture_image = pixels;
    ri.texture_image_size = pixels_size;
    ri.texture_format = PRC_API_WRITE_TEXTURE_RGB;
    ri.texture_width = w;
    ri.texture_height = h;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_min[0] = 0.0; root.bbox_min[1] = 0.0; root.bbox_min[2] = 0.0;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0; root.bbox_max[2] = 0.0;

    PRC_ASSERT_EQ(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, tables, &root, &tess_entry, 1), 0);
}

/* A texture reaches a renderer through a chain of five biased indices, and a
   break anywhere in it renders as "no texture" or "the wrong texture" rather
   than as an error:

       style -> TextureApplication -> TextureDefinition -> Picture -> file

   So walk the whole chain rather than checking that a picture exists. Each
   assertion below is a different way the feature can be wrong. */
static void
test_texture_chain_roundtrip(prc_context *ctx)
{
    /* 2x2 RGB, four distinguishable pixels. Distinguishable on purpose: a
       uniform image would pass against a writer that stored the wrong
       region, and the corner values pin the byte order. */
    static const uint8_t pixels[2 * 2 * 3] = {
        0xFF, 0x00, 0x00,   0x00, 0xFF, 0x00,
        0x00, 0x00, 0xFF,   0xFF, 0xFF, 0x00
    };
    prc_write_global_tables tables;
    prc_data *pd;
    const prc_file_struct_internal_global_data *gd;
    uint32_t style_idx, app_idx, def_idx, pic_idx, file_idx, k;

    printf("  sub-case: texture chain style->application->definition->picture->file\n");

    build_textured_quad_file_with_image(ctx, &tables, pixels, sizeof(pixels), 2, 2);

    pd = (prc_data *)prc_api_open_contents(ctx, TEST_PRC_FILENAME);
    if (pd == NULL)
        prc_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(pd);
    gd = &pd->file_struct[0].globals->global_data;

    /* The item's style. */
    {
        const prc_asm_file_structure_tree *tree = pd->file_struct[0].tree;
        const prc_ri *ri;

        PRC_ASSERT_NOT_NULL(tree);
        PRC_ASSERT(tree->parts_count >= 1);
        PRC_ASSERT(tree->parts[0].num_rep_items >= 1);
        ri = &tree->parts[0].rep_items[0];
        PRC_ASSERT_NOT_NULL(ri->ri_poly_brep_model);
        style_idx = ri->ri_poly_brep_model->item_content.base.graphics_content.biased_index_of_line_style;
        PRC_ASSERT(style_idx != 0);
    }

    /* -> a TextureApplication, not a plain Material. A writer that attached
       the image to the material itself, or forgot the application layer,
       fails here. */
    PRC_ASSERT(style_idx - 1 < gd->style_count);
    PRC_ASSERT(gd->styles[style_idx - 1].is_material);
    app_idx = gd->styles[style_idx - 1].biased_color_index;
    PRC_ASSERT(app_idx != 0 && app_idx - 1 < gd->material_count);
    PRC_ASSERT_EQ(gd->materials[app_idx - 1].tag, PRC_TYPE_GRAPH_TextureApplication);

    /* -> the underlying material must still be there: the colour a textured
       item modulates comes from it, so a zero here is a white-out. */
    PRC_ASSERT(gd->materials[app_idx - 1].biased_material_generic_index != 0);

    /* -> and the UV index must be BIASED. prc_style_api.c stores
       biased_uv_coordinates_index - 1, and -1 is its "this texture has no UV
       coordinates" sentinel, so writing the obvious 0 here produces a file
       that parses, carries the image, and renders untextured. Nothing else
       in this chain notices: every assertion above and below still holds.
       Checked by sabotage -- writing 0 leaves all 15 tests passing without
       this line. */
    PRC_ASSERT_EQ(gd->materials[app_idx - 1].biased_uv_coordinates_index, 1);

    /* -> the definition. */
    def_idx = gd->materials[app_idx - 1].biased_texture_definition_index;
    PRC_ASSERT(def_idx != 0);
    PRC_ASSERT(def_idx - 1 < gd->texture_count);
    PRC_ASSERT_EQ(gd->textures[def_idx - 1].texture_mapping_type, PRC_texture_mapping_retrieve_UV);

    /* -> the picture. */
    pic_idx = gd->textures[def_idx - 1].biased_picture_index;
    PRC_ASSERT(pic_idx != 0 && pic_idx - 1 < gd->picture_count);
    PRC_ASSERT_EQ(gd->pictures[pic_idx - 1].pixel_width, 2);
    PRC_ASSERT_EQ(gd->pictures[pic_idx - 1].pixel_height, 2);

    /* -> the bytes. This is the link that did not exist at all before: every
       picture used to be written with file index 0. */
    file_idx = gd->pictures[pic_idx - 1].biased_uncompressed_file_index;
    PRC_ASSERT(file_idx != 0);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].header);
    PRC_ASSERT(file_idx - 1 < pd->file_struct[0].header->file_count);
    PRC_ASSERT_EQ(pd->file_struct[0].header->files[file_idx - 1].block_size, sizeof(pixels));
    for (k = 0; k < sizeof(pixels); k++)
        PRC_ASSERT_EQ(pd->file_struct[0].header->files[file_idx - 1].block[k], pixels[k]);

    prc_api_release_data(ctx, (prc_api_data)pd, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_write_global_tables_free(ctx, &tables);
    remove(TEST_PRC_FILENAME);
}

static void
test_textured_quad_roundtrip(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_data *pd;
    prc_tess_3d *t3d;
    int k;

    printf("  sub-case: textured quad round trip through the public write struct\n");

    build_textured_quad_file(ctx, &tables);

    pd = (prc_data *)prc_api_open_contents(ctx, TEST_PRC_FILENAME);
    if (pd == NULL)
        prc_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(pd);

    PRC_ASSERT_EQ(pd->file_structure_count, 1);
    PRC_ASSERT_NOT_NULL(pd->file_struct[0].tessellation);
    PRC_ASSERT_EQ(pd->file_struct[0].tessellation->tess_count, 1);

    t3d = pd->file_struct[0].tessellation->tess[0].tess_3d;
    PRC_ASSERT_NOT_NULL(t3d);

    /* Stored as a count of DOUBLES, not of (u,v) pairs -- the convention
       filed as pdf-association/pdf-issues#810. Three pairs in, six out. */
    PRC_ASSERT_EQ(t3d->number_of_texture_coordinates, 6);
    PRC_ASSERT_NOT_NULL(t3d->texture_coordinates);
    for (k = 0; k < 6; k++)
    {
        static const double expect[6] = { 0.0, 0.0, 1.0, 0.0, 0.5, 1.0 };
        PRC_ASSERT_NEAR(t3d->texture_coordinates[k], expect[k], 1e-12);
    }

    PRC_ASSERT_EQ(t3d->number_of_face_tessellation, 1);
    PRC_ASSERT_NOT_NULL(t3d->face_tessellation_data);
    PRC_ASSERT_EQ(t3d->face_tessellation_data[0].number_of_textured_coordinate_indexes, 1);

    prc_api_release_data(ctx, (prc_api_data)pd, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_write_global_tables_free(ctx, &tables);
}

/* The two ways a caller can ask for something the writer cannot honour. Both
   must be refused rather than quietly dropped: prc_write_tess_3d treats a
   half-supplied pair as "no textures", and the compressed encoder has no
   texture path at all, so without these checks the UVs would simply vanish
   from the output with no error anywhere. */
static void
test_schema_globals_never_108_bytes(prc_context *ctx)
{
    /* Adobe Acrobat shows an empty model tree for a file whose schema+globals
       section is exactly 108 bytes before compression. The writer pads past
       that length; this checks the pad is still there.

       The test reads the written file's own section offsets and decompresses
       the section, rather than trusting the writer's internal count, so it
       measures what a reader would see. The material colour is the knob: it
       is the field whose encoded width moves that section by single bytes,
       and (1.0, 0.75, 0.875) is a triple that lands on 108 without the pad. */
    static const double positions[3 * 3] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    static const double normals[3] = { 0.0, 0.0, 1.0 };
    static const uint32_t tris[3] = { 0, 1, 2 };
    static const uint32_t norm_idx[3] = { 0, 0, 0 };
    static const uint32_t face_tri_counts[1] = { 1 };
    static const uint8_t texpixels[2 * 2 * 3] = {
        0xFF, 0x00, 0x00,  0x00, 0xFF, 0x00,
        0x00, 0x00, 0xFF,  0xFF, 0xFF, 0x00
    };
    static const double uvs[3 * 2] = { 0.0, 0.0,  1.0, 0.0,  0.0, 1.0 };
    static const uint32_t uv_idx[3] = { 0, 1, 2 };
    prc_write_global_tables tables;
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;
    FILE *f;
    long fsize;
    uint8_t *buf;
    uint32_t section_count, off_globals, off_next;
    size_t hdr;

    printf("  sub-case: the schema+globals section is never 108 bytes\n");

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;
    ri.has_material = 1;
    ri.material_color[0] = 1.0;
    ri.material_color[1] = 0.5;
    ri.material_color[2] = 1.0;
    ri.material_alpha = 1.0;
    ri.material_shininess = 0.1;
    /* A texture is required, not decoration: an untextured one-triangle file
       produces a schema+globals section of 77 to 93 bytes and cannot reach
       108 at all, so a test without one can never exercise the guard. With
       this 2x2 picture the section spans 107 to 112, and this colour lands
       on 108 exactly when the guard is removed. */
    ri.has_texture = 1;
    ri.texture_image = texpixels;
    ri.texture_image_size = sizeof(texpixels);
    ri.texture_format = PRC_API_WRITE_TEXTURE_RGB;
    ri.texture_width = 2;
    ri.texture_height = 2;

    memset(&root, 0, sizeof(root));
    root.name = "g";
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0;

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 3;
    tess_entry.normals = normals;
    tess_entry.num_normals = 1;
    tess_entry.tri_indices = tris;
    tess_entry.norm_indices = norm_idx;
    tess_entry.num_triangles = 1;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;
    tess_entry.tex_coords = uvs;
    tess_entry.num_tex_coords = 3;
    tess_entry.tex_indices = uv_idx;

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);
    PRC_ASSERT_EQ(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, &tables,
                                     &root, &tess_entry, 1), 0);

    f = fopen(TEST_PRC_FILENAME, "rb");
    PRC_ASSERT_NOT_NULL(f);
    fseek(f, 0, SEEK_END); fsize = ftell(f); fseek(f, 0, SEEK_SET);
    PRC_ASSERT(fsize > 0);
    buf = (uint8_t *)malloc((size_t)fsize);
    PRC_ASSERT_NOT_NULL(buf);
    PRC_ASSERT_EQ(fread(buf, 1, (size_t)fsize, f), (size_t)fsize);
    fclose(f);

    /* "PRC" + min_vers + auth_vers + 2 unique ids + filestructure_count
       + file_info uid + reserved + section_count, then the offset table. */
    hdr = 3 + 4 + 4 + 16 + 16 + 4 + 16 + 4;
    section_count = read_le_uint32(buf + hdr);
    PRC_ASSERT(section_count >= 3);
    off_globals = read_le_uint32(buf + hdr + 4 + 4);      /* section_offset[1] */
    off_next    = read_le_uint32(buf + hdr + 4 + 8);      /* section_offset[2] */
    PRC_ASSERT(off_next > off_globals);
    PRC_ASSERT((long)off_next <= fsize);

    {
        /* Decompress and check the length a reader would see. Without the
           pad this colour yields exactly 108. */
        uint8_t out[4096];
        uLongf out_len = (uLongf)sizeof(out);
        int zr = uncompress(out, &out_len, buf + off_globals, (uLong)(off_next - off_globals));

        PRC_ASSERT_EQ(zr, Z_OK);
        PRC_ASSERT(out_len != 108);
    }

    free(buf);
    prc_write_global_tables_free(ctx, &tables);
    remove(TEST_PRC_FILENAME);
}

static void
test_texture_misuse_is_refused(prc_context *ctx)
{
    static const double positions[3 * 3] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    static const double normals[3] = { 0.0, 0.0, 1.0 };
    static const double tex_coords[3 * 2] = { 0.0, 0.0, 1.0, 0.0, 0.0, 1.0 };
    static const uint32_t tris[3] = { 0, 1, 2 };
    static const uint32_t norm_idx[3] = { 0, 0, 0 };
    static const uint32_t tex_idx[3] = { 0, 1, 2 };
    static const uint32_t face_tri_counts[1] = { 1 };
    prc_write_global_tables tables;
    prc_write_tess_entry tess_entry;
    prc_write_rep_item ri;
    prc_write_tree_node root;

    printf("  sub-case: texture coordinates are refused where they cannot be honoured\n");

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;
    root.bbox_max[0] = 1.0; root.bbox_max[1] = 1.0;

    memset(&tess_entry, 0, sizeof(tess_entry));
    tess_entry.kind = PRC_WRITE_TESS_KIND_3D;
    tess_entry.positions = positions;
    tess_entry.num_positions = 3;
    tess_entry.normals = normals;
    tess_entry.num_normals = 1;
    tess_entry.tri_indices = tris;
    tess_entry.norm_indices = norm_idx;
    tess_entry.num_triangles = 1;
    tess_entry.face_tri_counts = face_tri_counts;
    tess_entry.num_faces = 1;

    /* Coordinates without indices. */
    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);
    tess_entry.tex_coords = tex_coords;
    tess_entry.num_tex_coords = 3;
    tess_entry.tex_indices = NULL;
    PRC_ASSERT(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, &tables, &root, &tess_entry, 1) != 0);
    prc_write_global_tables_free(ctx, &tables);

    /* Indices without coordinates. */
    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);
    tess_entry.tex_coords = NULL;
    tess_entry.num_tex_coords = 0;
    tess_entry.tex_indices = tex_idx;
    PRC_ASSERT(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, &tables, &root, &tess_entry, 1) != 0);
    prc_write_global_tables_free(ctx, &tables);

    /* Both, but on a kind with no texture path. */
    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);
    tess_entry.kind = PRC_WRITE_TESS_KIND_COMPRESSED;
    tess_entry.tex_coords = tex_coords;
    tess_entry.num_tex_coords = 3;
    tess_entry.tex_indices = tex_idx;
    PRC_ASSERT(prc_write_prc_file(ctx, TEST_PRC_FILENAME, NULL, &tables, &root, &tess_entry, 1) != 0);
    prc_write_global_tables_free(ctx, &tables);

    /* Each refusal above leaves an entry on the context error stack. That is
       expected here -- these cases assert that the writer complains -- and
       there is no public call to clear it, so nothing tries to. */
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
    test_textured_quad_roundtrip(ctx);
    test_texture_misuse_is_refused(ctx);
    test_schema_globals_never_108_bytes(ctx);
    test_prc_signature(ctx);
    test_zlib_section_valid(ctx);
    test_embedded_files(ctx);
    test_texture_chain_roundtrip(ctx);

    prc_release_context(ctx);
    remove(TEST_PRC_FILENAME);

    PRC_TEST_END;
}
