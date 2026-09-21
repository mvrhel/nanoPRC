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

/* Phase 1d, session 1 gate test: prc_write_global (the globals-section
   encoder for colors/materials/pictures/styles) round-tripped through the
   real, unmodified prc_parse_global_data. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_write_global.h"
#include "prc_parse_global.h"
#include "prc_api.h"

/* Mirrors the (static, so not reusable directly) prc_release_global_data in
   prc_release.c, specialized to what this encoder ever actually populates:
   every base's attribute_data/name is written empty (attribute_count=0,
   name.same=1), so those sub-fields are never allocated on the parse side;
   texture/line_pattern/fill/ref_coord counts are always written 0. */
static void
free_parsed_globals(prc_context *ctx, prc_file_struct_internal_global_data *g)
{
    if (g->colors != NULL) prc_free(ctx, g->colors);
    if (g->materials != NULL) prc_free(ctx, g->materials);
    if (g->pictures != NULL) prc_free(ctx, g->pictures);
    if (g->styles != NULL) prc_free(ctx, g->styles);
}

static void
test_color_dedup(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_rgb_color red;
    uint32_t idx1, idx2;

    printf("  sub-case: color add + dedup\n");

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);

    red.red = 1.0; red.green = 0.0; red.blue = 0.0; red.alpha = 1.0;
    idx1 = prc_write_color_add(ctx, &tables, &red);
    idx2 = prc_write_color_add(ctx, &tables, &red);

    PRC_ASSERT(idx1 != 0);
    PRC_ASSERT_EQ(idx1, idx2);
    PRC_ASSERT_EQ(tables.color_count, 1);

    prc_write_global_tables_free(ctx, &tables);
}

static void
test_material_roundtrip(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_rgb_color ambient;
    prc_graph_material mat;
    uint32_t color_idx, mat_idx;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_file_struct_internal_global_data parsed;
    prc_file_structure_header header;
    int code;

    printf("  sub-case: material round trip\n");

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);

    ambient.red = 0.2; ambient.green = 0.2; ambient.blue = 0.2; ambient.alpha = 1.0;
    color_idx = prc_write_color_add(ctx, &tables, &ambient);
    PRC_ASSERT(color_idx != 0);

    memset(&mat, 0, sizeof(mat));
    mat.tag = PRC_TYPE_GRAPH_Material;
    mat.biased_ambient_index = color_idx;
    mat.biased_diffuse_index = 0;
    mat.biased_emissive_index = 0;
    mat.biased_specular_index = 0;
    mat.shininess = 32.5;
    mat.ambient_alpha = 1.0;
    mat.diffuse_alpha = 0.8;
    mat.emissive_alpha = 0.0;
    mat.specular_alpha = 1.0;

    mat_idx = prc_write_material_add(ctx, &tables, &mat);
    PRC_ASSERT(mat_idx != 0);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    PRC_ASSERT_EQ(prc_write_globals_to_stream(ctx, &w, &tables), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    memset(&parsed, 0, sizeof(parsed));
    memset(&header, 0, sizeof(header));
    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_global_data(ctx, &r, &parsed, &header);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);

    PRC_ASSERT_EQ(parsed.color_count, 1);
    PRC_ASSERT_EQ(parsed.material_count, 1);
    PRC_ASSERT_EQ(parsed.materials[0].tag, PRC_TYPE_GRAPH_Material);
    PRC_ASSERT_EQ(parsed.materials[0].biased_ambient_index, color_idx);
    PRC_ASSERT_EQ(parsed.materials[0].biased_diffuse_index, 0);
    PRC_ASSERT(fabs(parsed.materials[0].shininess - 32.5) < 1e-12);
    PRC_ASSERT(fabs(parsed.materials[0].ambient_alpha - 1.0) < 1e-12);
    PRC_ASSERT(fabs(parsed.materials[0].diffuse_alpha - 0.8) < 1e-12);
    PRC_ASSERT(fabs(parsed.materials[0].specular_alpha - 1.0) < 1e-12);

    free_parsed_globals(ctx, &parsed);
    prc_write_global_tables_free(ctx, &tables);
    prc_bitwrite_release(ctx, &w);
}

static void
test_style_transparency(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_rgb_color color;
    prc_graph_style style;
    uint32_t color_idx, style_idx;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_file_struct_internal_global_data parsed;
    prc_file_structure_header header;
    int code;

    printf("  sub-case: style with transparency=180\n");

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);

    color.red = 0.5; color.green = 0.5; color.blue = 0.5; color.alpha = 1.0;
    color_idx = prc_write_color_add(ctx, &tables, &color);
    PRC_ASSERT(color_idx != 0);

    memset(&style, 0, sizeof(style));
    style.line_width = 2.5;
    style.is_vpicture = 0;
    style.biased_pattern_index = 0;
    style.is_material = 0;
    style.biased_color_index = color_idx;
    style.is_transparency = 1;
    style.transparency = 180;
    style.is_rendering_parameters = 0;

    style_idx = prc_write_style_add(ctx, &tables, &style);
    PRC_ASSERT(style_idx != 0);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    PRC_ASSERT_EQ(prc_write_globals_to_stream(ctx, &w, &tables), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    memset(&parsed, 0, sizeof(parsed));
    memset(&header, 0, sizeof(header));
    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_global_data(ctx, &r, &parsed, &header);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);

    PRC_ASSERT_EQ(parsed.style_count, 1);
    PRC_ASSERT_EQ(parsed.styles[0].is_transparency, 1);
    PRC_ASSERT_EQ(parsed.styles[0].transparency, 180);
    PRC_ASSERT_EQ(parsed.styles[0].is_material, 0);
    PRC_ASSERT_EQ(parsed.styles[0].biased_color_index, color_idx);
    PRC_ASSERT(fabs(parsed.styles[0].line_width - 2.5) < 1e-12);
    PRC_ASSERT_EQ(parsed.styles[0].is_rendering_parameters, 0);

    free_parsed_globals(ctx, &parsed);
    prc_write_global_tables_free(ctx, &tables);
    prc_bitwrite_release(ctx, &w);
}

static void
test_picture_roundtrip(prc_context *ctx)
{
    prc_write_global_tables tables;
    prc_write_picture pic;
    uint8_t rgb_bytes[10 * 5 * 3];
    uint8_t rgba_bytes[8 * 4 * 4];
    /* Minimal PNG: signature + IHDR chunk (width=64, height=32). Only the
       first 24 bytes are ever inspected. */
    static const uint8_t png_bytes[33] = {
        0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A,
        0x00, 0x00, 0x00, 0x0D, 'I', 'H', 'D', 'R',
        0x00, 0x00, 0x00, 0x40, /* width = 64 */
        0x00, 0x00, 0x00, 0x20, /* height = 32 */
        0x08, 0x06, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00  /* dummy CRC */
    };
    /* Minimal JPEG: SOI + APP0(JFIF) + SOF0 (height=48, width=96). No scan
       data -- the encoder only scans markers up to and including SOF. */
    static const uint8_t jpeg_bytes[39] = {
        0xFF, 0xD8,                                           /* SOI */
        0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,      /* APP0 */
        0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
        0xFF, 0xC0, 0x00, 0x11,                                /* SOF0 */
        0x08,                                                  /* precision */
        0x00, 0x30,                                            /* height = 48 */
        0x00, 0x60,                                            /* width = 96 */
        0x03,                                                  /* num components */
        0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01
    };
    uint32_t idx_rgb, idx_rgba, idx_png, idx_jpeg;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_file_struct_internal_global_data parsed;
    prc_file_structure_header header;
    int code;

    printf("  sub-case: picture round trip (RGB, RGBA, PNG, JPEG)\n");

    memset(rgb_bytes, 0x7F, sizeof(rgb_bytes));
    memset(rgba_bytes, 0x3C, sizeof(rgba_bytes));

    PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &tables), 0);

    memset(&pic, 0, sizeof(pic));
    pic.format = PRC_WRITE_PIX_RGB;
    pic.data = rgb_bytes;
    pic.data_size = sizeof(rgb_bytes);
    pic.width = 10;
    pic.height = 5;
    idx_rgb = prc_write_picture_add(ctx, &tables, &pic);
    PRC_ASSERT(idx_rgb != 0);

    memset(&pic, 0, sizeof(pic));
    pic.format = PRC_WRITE_PIX_RGBA;
    pic.data = rgba_bytes;
    pic.data_size = sizeof(rgba_bytes);
    pic.width = 8;
    pic.height = 4;
    idx_rgba = prc_write_picture_add(ctx, &tables, &pic);
    PRC_ASSERT(idx_rgba != 0);

    memset(&pic, 0, sizeof(pic));
    pic.format = PRC_WRITE_PIX_PNG;
    pic.data = png_bytes;
    pic.data_size = sizeof(png_bytes);
    idx_png = prc_write_picture_add(ctx, &tables, &pic);
    PRC_ASSERT(idx_png != 0);

    memset(&pic, 0, sizeof(pic));
    pic.format = PRC_WRITE_PIX_JPEG;
    pic.data = jpeg_bytes;
    pic.data_size = sizeof(jpeg_bytes);
    idx_jpeg = prc_write_picture_add(ctx, &tables, &pic);
    PRC_ASSERT(idx_jpeg != 0);

    /* Each picture stores its bytes in the embedded-file table and points at
       them; before that existed every picture was metadata with no image
       behind it, with biased_uncompressed_file_index always 0.

       The value to check is each PICTURE's link to its image, not what
       prc_write_picture_add returns -- that is the picture's own index and is
       1,2,3,4 whatever the images do. A first version of this checked the
       returned indices and passed with the file table handing every picture
       image 1, which renders as "the wrong texture" rather than as an error. */
    PRC_ASSERT_EQ(tables.file_count, 4);
    PRC_ASSERT_EQ(tables.pictures[0].biased_uncompressed_file_index, 1);
    PRC_ASSERT_EQ(tables.pictures[1].biased_uncompressed_file_index, 2);
    PRC_ASSERT_EQ(tables.pictures[2].biased_uncompressed_file_index, 3);
    PRC_ASSERT_EQ(tables.pictures[3].biased_uncompressed_file_index, 4);

    /* Four different lengths, so a writer using a fixed stride or storing
       data_size blindly fails here. The raw formats store the pixels the
       DIMENSIONS describe; PNG/JPEG store the whole encoded file. */
    PRC_ASSERT_EQ(tables.files[0].size, 10u * 5u * 3u);
    PRC_ASSERT_EQ(tables.files[1].size, 8u * 4u * 4u);
    PRC_ASSERT_EQ(tables.files[2].size, (uint32_t)sizeof(png_bytes));
    PRC_ASSERT_EQ(tables.files[3].size, (uint32_t)sizeof(jpeg_bytes));

    /* And the bytes are the caller's: the RGB buffer was filled with 0x7F and
       the RGBA with 0x3C, so a mix-up between them shows here. */
    PRC_ASSERT_EQ(tables.files[0].data[0], 0x7F);
    PRC_ASSERT_EQ(tables.files[1].data[0], 0x3C);
    PRC_ASSERT_EQ(tables.files[3].data[0], 0xFF);   /* JPEG SOI */
    PRC_ASSERT_EQ(tables.files[3].data[1], 0xD8);

    /* A texture definition over the first picture, and a material AFTER it.
       The material is not incidental: the texture table sits between the
       pictures and the materials, and a field-order mistake inside Table 96
       does not fail on its own -- it leaves the stream misaligned and the
       next thing parsed is what notices. Written without a following
       material, a definition missing its last four fields round-tripped
       silently, which is exactly how those fields came to be missing. */
    PRC_ASSERT(prc_write_texture_definition_add(ctx, &tables, 1, NULL) == 1);
    {
        prc_graph_material app;

        memset(&app, 0, sizeof(app));
        app.tag = PRC_TYPE_GRAPH_TextureApplication;
        app.biased_material_generic_index = 1;
        app.biased_texture_definition_index = 1;
        PRC_ASSERT(prc_write_material_add(ctx, &tables, &app) != 0);
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 1024), 0);
    PRC_ASSERT_EQ(prc_write_globals_to_stream(ctx, &w, &tables), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    memset(&parsed, 0, sizeof(parsed));
    memset(&header, 0, sizeof(header)); /* file_count=0: JPG/PNG raw-image
        decode in prc_parse_graph_picture is skipped; only Table-93
        format/dimensions metadata is checked here. */
    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_global_data(ctx, &r, &parsed, &header);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);

    PRC_ASSERT_EQ(parsed.picture_count, 4);

    PRC_ASSERT_EQ(parsed.pictures[0].format, KEPRCPicture_BITMAP_RGB_BYTE);
    PRC_ASSERT_EQ(parsed.pictures[0].pixel_width, 10);
    PRC_ASSERT_EQ(parsed.pictures[0].pixel_height, 5);

    PRC_ASSERT_EQ(parsed.pictures[1].format, KEPRCPicture_BITMAP_RGBA_BYTE);
    PRC_ASSERT_EQ(parsed.pictures[1].pixel_width, 8);
    PRC_ASSERT_EQ(parsed.pictures[1].pixel_height, 4);

    /* The encoded formats declare 0: 7.5.5 says these fields are ignored when
       format is PNG or JPEG, and the image carries its own dimensions. The
       raw formats above must still declare theirs, because nothing else in
       the file records them -- that contrast is the point of checking all
       four here rather than just one. */
    PRC_ASSERT_EQ(parsed.pictures[2].format, KEPRCPicture_PNG);
    PRC_ASSERT_EQ(parsed.pictures[2].pixel_width, 0);
    PRC_ASSERT_EQ(parsed.pictures[2].pixel_height, 0);

    PRC_ASSERT_EQ(parsed.pictures[3].format, KEPRCPicture_JPG);
    PRC_ASSERT_EQ(parsed.pictures[3].pixel_width, 0);
    PRC_ASSERT_EQ(parsed.pictures[3].pixel_height, 0);

    /* The dimensions are still PARSED, even though they are not stored: a
       buffer that is not the format the caller claimed is rejected. Zeroing
       the record must not turn that validation off. */
    {
        prc_write_global_tables t2;
        prc_write_picture pic;
        static const uint8_t not_a_png[16] = { 0 };

        PRC_ASSERT_EQ(prc_write_global_tables_init(ctx, &t2), 0);
        memset(&pic, 0, sizeof(pic));
        pic.format = PRC_WRITE_PIX_PNG;
        pic.data = not_a_png;
        pic.data_size = sizeof(not_a_png);
        PRC_ASSERT_EQ(prc_write_picture_add(ctx, &t2, &pic), 0);
        prc_write_global_tables_free(ctx, &t2);
    }

    free_parsed_globals(ctx, &parsed);
    prc_write_global_tables_free(ctx, &tables);
    prc_bitwrite_release(ctx, &w);
}

static void
test_tolerance_modes(prc_context *ctx)
{
    printf("  sub-case: tolerance mode resolution\n");

    /* Absolute: value passes through unchanged regardless of bbox diagonal. */
    PRC_ASSERT(fabs(prc_write_tol_resolve(ctx, prc_write_tol_absolute(5.0), 1000.0) - 5.0) < 1e-12);
    PRC_ASSERT(fabs(prc_write_tol_resolve(ctx, prc_write_tol_absolute(5.0), 0.0) - 5.0) < 1e-12);

    /* Relative: value * bbox_diagonal. */
    PRC_ASSERT(fabs(prc_write_tol_resolve(ctx, prc_write_tol_relative(0.001), 1000.0) - 1.0) < 1e-12);

    /* Floor clamp: anything below 1e-7 mm is raised to the floor. */
    PRC_ASSERT(fabs(prc_write_tol_resolve(ctx, prc_write_tol_absolute(1e-10), 1000.0) - 1e-7) < 1e-15);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("globals section encoder round trip");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_color_dedup(ctx);
    test_material_roundtrip(ctx);
    test_style_transparency(ctx);
    test_picture_roundtrip(ctx);
    test_tolerance_modes(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
