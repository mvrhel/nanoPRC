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

/* Per-representation-item materials through the public write API.

   Two leaves, two different colours, written and reopened. What this pins down
   is that the globals tables really carry both materials and that the two
   items do not share one style -- the failure this is guarding against is a
   writer that registers a material and then hands every item the shared
   default anyway, which would look perfectly fine in the file and render as
   one flat colour.

   The style resolution happens before the globals section is serialised, so a
   regression there shows up as a missing style rather than a wrong one. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_api.h"
#include "prc_data.h"

#define TEST_MATERIALS_FILENAME "test_write_materials_tmp.prc"

/* One triangle, so each leaf has something to hang a material on. */
static const double tri_positions[3 * 3] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0
};
static const uint32_t tri_indices[3] = { 0, 1, 2 };
static const uint32_t tri_face_counts[1] = { 1 };

static void
fill_tess(prc_api_write_tessellation *t)
{
    memset(t, 0, sizeof(*t));
    t->kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
    t->positions = tri_positions;
    t->num_positions = 3;
    t->tri_indices = tri_indices;
    t->num_triangles = 1;
    t->face_tri_counts = tri_face_counts;
    t->num_faces = 1;
    t->must_calculate_normals = 1;
    t->crease_angle_degrees = 30.0;
}

static void
test_two_materials(prc_context *ctx)
{
    prc_api_write_tessellation tess[2];
    prc_api_write_rep_item items_a[1];
    prc_api_write_rep_item items_b[1];
    prc_api_write_node leaf_a, leaf_b, root;
    prc_api_write_node *kids[2];
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_data *data = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;

    printf("  sub-case: two items, two materials, through the public API\n");

    fill_tess(&tess[0]);
    fill_tess(&tess[1]);

    memset(items_a, 0, sizeof(items_a));
    items_a[0].kind = PRC_API_WRITE_RI_SURFACE;
    items_a[0].biased_tessellation_index = 1;
    items_a[0].has_material = 1;
    items_a[0].material_color[0] = 1.0;   /* red */
    items_a[0].material_color[1] = 0.0;
    items_a[0].material_color[2] = 0.0;
    items_a[0].material_alpha = 1.0;
    items_a[0].material_shininess = 0.5;

    memset(items_b, 0, sizeof(items_b));
    items_b[0].kind = PRC_API_WRITE_RI_SURFACE;
    items_b[0].biased_tessellation_index = 2;
    items_b[0].has_material = 1;
    items_b[0].material_color[0] = 0.0;
    items_b[0].material_color[1] = 0.0;
    items_b[0].material_color[2] = 1.0;   /* blue */
    items_b[0].material_alpha = 0.25;     /* and mostly transparent */
    items_b[0].material_shininess = 0.1;

    memset(&leaf_a, 0, sizeof(leaf_a));
    leaf_a.rep_items = items_a;
    leaf_a.num_rep_items = 1;
    leaf_a.name = "red_leaf";
    leaf_a.bbox_max[0] = leaf_a.bbox_max[1] = 1.0;

    memset(&leaf_b, 0, sizeof(leaf_b));
    leaf_b.rep_items = items_b;
    leaf_b.num_rep_items = 1;
    leaf_b.name = "blue_leaf";
    leaf_b.bbox_max[0] = leaf_b.bbox_max[1] = 1.0;

    kids[0] = &leaf_a;
    kids[1] = &leaf_b;
    memset(&root, 0, sizeof(root));
    root.name = "root";
    root.children = kids;
    root.num_children = 2;
    root.bbox_max[0] = root.bbox_max[1] = 1.0;

    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "materials_model", &root,
        tess, 2, &buf, &buf_size), 0);
    PRC_ASSERT_NOT_NULL(buf);
    PRC_ASSERT(buf_size > 0);

    fid = fopen(TEST_MATERIALS_FILENAME, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = prc_api_open_contents(ctx, TEST_MATERIALS_FILENAME);
    if (data == NULL)
        prc_api_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(data);

    PRC_ASSERT_EQ(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups), 0);

    /* The styles have to reach the items, not merely exist in the tables.
       Registering a material and then handing every item the shared default
       produces a file that looks right in the globals and renders as one flat
       colour -- an earlier version of this test checked only the tables and
       passed against exactly that bug. */
    {
        const prc_asm_file_structure_tree *tree = data->file_struct[0].tree;
        uint32_t styles[2];
        uint32_t found = 0;
        uint32_t i;

        PRC_ASSERT_NOT_NULL(tree);
        for (i = 0; i < tree->parts_count && found < 2; i++)
        {
            const prc_asm_parts_definition *part = &tree->parts[i];
            uint32_t j;

            for (j = 0; j < part->num_rep_items && found < 2; j++)
            {
                const prc_ri *ri = &part->rep_items[j];

                if (ri->ri_poly_brep_model == NULL)
                    continue;
                styles[found++] =
                    ri->ri_poly_brep_model->item_content.base.graphics_content.biased_index_of_line_style;
            }
        }

        PRC_ASSERT_EQ(found, 2);
        /* Each item carries a real style... */
        PRC_ASSERT(styles[0] != 0);
        PRC_ASSERT(styles[1] != 0);
        /* ...and they are not the same one, which is what a writer that
           ignored has_material would produce. */
        PRC_ASSERT(styles[0] != styles[1]);

        /* Distinct is not the same as CORRECT, and the difference is not
           academic: with the colour index written as an entry index rather
           than a double index, these two styles were still distinct and
           still resolved -- to the shared default grey, both of them. Acrobat
           showed grey where red and blue were asked for while this test
           passed. So follow each style the whole way to a colour and compare
           it against what went in.

           style -> material -> ambient colour, unbiasing by three at the last
           step exactly as prc_style_api.c does. */
        {
            const prc_file_struct_internal_global_data *gd =
                &data->file_struct[0].globals->global_data;
            double want[2][3];
            uint32_t k;

            want[0][0] = 1.0; want[0][1] = 0.0; want[0][2] = 0.0;   /* red */
            want[1][0] = 0.0; want[1][1] = 0.0; want[1][2] = 1.0;   /* blue */

            for (k = 0; k < 2; k++)
            {
                uint32_t si = styles[k] - 1;
                uint32_t mi, ci;
                const prc_graph_style *st;
                const prc_graph_material *mt;
                const prc_rgb_color *c;

                PRC_ASSERT(si < gd->style_count);
                st = &gd->styles[si];
                PRC_ASSERT(st->is_material);

                mi = st->biased_color_index - 1;
                PRC_ASSERT(mi < gd->material_count);
                mt = &gd->materials[mi];

                PRC_ASSERT(mt->biased_ambient_index > 0);
                ci = (mt->biased_ambient_index - 1) / 3;
                PRC_ASSERT(ci < gd->color_count);
                c = &gd->colors[ci];

                PRC_ASSERT(c->red   > want[k][0] - 0.02 && c->red   < want[k][0] + 0.02);
                PRC_ASSERT(c->green > want[k][1] - 0.02 && c->green < want[k][1] + 0.02);
                PRC_ASSERT(c->blue  > want[k][2] - 0.02 && c->blue  < want[k][2] + 0.02);

                /* Transparency lives on the style, not the material: a reader
                   takes the effective alpha from is_transparency/transparency
                   and ignores the material's alpha components entirely. The
                   red item asked for alpha 1.0 and must carry no flag; the
                   blue asked for 0.25 and must carry 64.

                   0.25 rather than 0.5, deliberately. The field is an OPACITY
                   byte (7.5.3, "0 (transparent) to 255 (opaque)"), so it is
                   alpha*255 rounded; writing (1-alpha)*255 instead would be
                   inverted in every viewer while round-tripping perfectly
                   through our own reader. At alpha 0.5 both formulas give 128,
                   so that value cannot detect the inversion -- a first version
                   of this assertion used it, and a control confirmed it caught
                   nothing. At 0.25 they give 64 and 191. */
                if (k == 0)
                {
                    PRC_ASSERT_EQ(st->is_transparency, 0);
                }
                else
                {
                    PRC_ASSERT_EQ(st->is_transparency, 1);
                    PRC_ASSERT_EQ(st->transparency, 64);
                }
            }
        }
    }

    /* And the tables really hold both materials. */
    {
        const prc_asm_file_structure_globals *g = data->file_struct[0].globals;
        const prc_file_struct_internal_global_data *gd;
        uint32_t i, found_red = 0, found_blue = 0;

        PRC_ASSERT_NOT_NULL(g);
        gd = &g->global_data;

        PRC_ASSERT(gd->style_count >= 3);      /* default + red + blue */
        PRC_ASSERT(gd->material_count >= 3);
        PRC_ASSERT(gd->color_count >= 3);

        /* Colours are stored as bytes 0..255 after the writer's conversion,
           so look for the two we asked for rather than comparing doubles. */
        for (i = 0; i < gd->color_count; i++)
        {
            const prc_rgb_color *c = &gd->colors[i];

            if (c->red > 0.99 && c->green < 0.01 && c->blue < 0.01)
                found_red = 1;
            if (c->blue > 0.99 && c->red < 0.01 && c->green < 0.01)
                found_blue = 1;
        }
        PRC_ASSERT_EQ(found_red, 1);
        PRC_ASSERT_EQ(found_blue, 1);
    }

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, NULL);
    remove(TEST_MATERIALS_FILENAME);
}

/* An item that asks for no material still gets the shared default, which is
   the behaviour every caller written before materials existed relies on. */
static void
test_default_still_applies(prc_context *ctx)
{
    prc_api_write_tessellation tess[1];
    prc_api_write_rep_item items[1];
    prc_api_write_node leaf, root;
    prc_api_write_node *kids[1];
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_data *data = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;

    printf("  sub-case: an item with no material keeps the shared default\n");

    fill_tess(&tess[0]);

    memset(items, 0, sizeof(items));
    items[0].kind = PRC_API_WRITE_RI_SURFACE;
    items[0].biased_tessellation_index = 1;
    /* has_material deliberately left 0 */

    memset(&leaf, 0, sizeof(leaf));
    leaf.rep_items = items;
    leaf.num_rep_items = 1;
    leaf.bbox_max[0] = leaf.bbox_max[1] = 1.0;

    kids[0] = &leaf;
    memset(&root, 0, sizeof(root));
    root.children = kids;
    root.num_children = 1;
    root.bbox_max[0] = root.bbox_max[1] = 1.0;

    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "default_model", &root,
        tess, 1, &buf, &buf_size), 0);
    PRC_ASSERT_NOT_NULL(buf);

    fid = fopen(TEST_MATERIALS_FILENAME, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = prc_api_open_contents(ctx, TEST_MATERIALS_FILENAME);
    PRC_ASSERT_NOT_NULL(data);
    PRC_ASSERT_EQ(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups), 0);

    {
        const prc_asm_file_structure_globals *g = data->file_struct[0].globals;

        PRC_ASSERT_NOT_NULL(g);
        /* Exactly the one default style, and nothing extra registered. */
        PRC_ASSERT_EQ(g->global_data.style_count, 1);
    }

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, NULL);
    remove(TEST_MATERIALS_FILENAME);
}

/* A face must name its item's style, and only when it can do so correctly.

   Until this was added the writer emitted size_of_line_attributes = 0 on every
   face -- "no graphics here, inherit from the owner of the TESS_3D" -- so any
   reader resolving style per face found nothing. Our own public API reported
   is_texture = 0 on files whose texture Acrobat displays, because Acrobat
   resolves the part-level style instead and that masked it.

   The negative half matters as much as the positive one: a tessellation shared
   by two items cannot carry either one's style, because the single face record
   would then be wrong for the other. That case must still write 0. */
static void
test_face_names_its_style(prc_context *ctx)
{
    prc_api_write_tessellation tess[1];
    prc_api_write_rep_item items[1];
    prc_api_write_node leaf, root;
    prc_api_write_node *kids[1];
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_data *data = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;

    printf("  sub-case: a face names the style of its one styled item\n");

    fill_tess(&tess[0]);

    memset(items, 0, sizeof(items));
    items[0].kind = PRC_API_WRITE_RI_SURFACE;
    items[0].biased_tessellation_index = 1;
    items[0].has_material = 1;
    items[0].material_color[0] = 1.0;
    items[0].material_alpha = 1.0;
    items[0].material_shininess = 0.5;

    memset(&leaf, 0, sizeof(leaf));
    leaf.rep_items = items;
    leaf.num_rep_items = 1;
    leaf.bbox_max[0] = leaf.bbox_max[1] = 1.0;

    kids[0] = &leaf;
    memset(&root, 0, sizeof(root));
    root.children = kids;
    root.num_children = 1;
    root.bbox_max[0] = root.bbox_max[1] = 1.0;

    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "face_style_model", &root,
        tess, 1, &buf, &buf_size), 0);
    PRC_ASSERT_NOT_NULL(buf);

    fid = fopen(TEST_MATERIALS_FILENAME, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = prc_api_open_contents(ctx, TEST_MATERIALS_FILENAME);
    PRC_ASSERT_NOT_NULL(data);
    PRC_ASSERT_EQ(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups), 0);

    {
        const prc_tess_3d *t3;
        const prc_tess_face *fc;

        PRC_ASSERT_NOT_NULL(data->file_struct[0].tessellation);
        PRC_ASSERT(data->file_struct[0].tessellation->tess_count > 0);
        t3 = data->file_struct[0].tessellation->tess[0].tess_3d;
        PRC_ASSERT_NOT_NULL(t3);
        PRC_ASSERT(t3->number_of_face_tessellation > 0);
        fc = &t3->face_tessellation_data[0];

        /* Table 140: 1 is "one graphic associated with the whole face
           tessellation data", and the entry is (index_of_line_style + 1), so a
           real style is >= 1 and 0 would mean none. */
        PRC_ASSERT_EQ(fc->size_of_line_attributes, 1);
        PRC_ASSERT_NOT_NULL(fc->line_attributes);
        PRC_ASSERT(fc->line_attributes[0] > 0);
        /* 7.8.6.1 points at behavior_bit_field (Table 34); PRC_GRAPHICS_Show
           is what prc_write_tree.c writes for the owning entity. */
        PRC_ASSERT_EQ(fc->behavior, (uint32_t)PRC_GRAPHICS_Show);
    }

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, NULL);
    remove(TEST_MATERIALS_FILENAME);
}

/* Two items sharing one tessellation: neither style can be baked into the
   single face record, so it must stay at 0 and both items keep resolving
   through their own item style as before. */
static void
test_shared_tessellation_names_no_style(prc_context *ctx)
{
    prc_api_write_tessellation tess[1];
    prc_api_write_rep_item items_a[1];
    prc_api_write_rep_item items_b[1];
    prc_api_write_node leaf_a, leaf_b, root;
    prc_api_write_node *kids[2];
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_data *data = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;

    printf("  sub-case: a tessellation shared by two items names no style\n");

    fill_tess(&tess[0]);

    memset(items_a, 0, sizeof(items_a));
    items_a[0].kind = PRC_API_WRITE_RI_SURFACE;
    items_a[0].biased_tessellation_index = 1;
    items_a[0].has_material = 1;
    items_a[0].material_color[0] = 1.0;
    items_a[0].material_alpha = 1.0;

    memset(items_b, 0, sizeof(items_b));
    items_b[0].kind = PRC_API_WRITE_RI_SURFACE;
    items_b[0].biased_tessellation_index = 1;   /* the SAME tessellation */
    items_b[0].has_material = 1;
    items_b[0].material_color[2] = 1.0;
    items_b[0].material_alpha = 1.0;

    memset(&leaf_a, 0, sizeof(leaf_a));
    leaf_a.rep_items = items_a;
    leaf_a.num_rep_items = 1;
    leaf_a.bbox_max[0] = leaf_a.bbox_max[1] = 1.0;

    memset(&leaf_b, 0, sizeof(leaf_b));
    leaf_b.rep_items = items_b;
    leaf_b.num_rep_items = 1;
    leaf_b.bbox_max[0] = leaf_b.bbox_max[1] = 1.0;

    kids[0] = &leaf_a;
    kids[1] = &leaf_b;
    memset(&root, 0, sizeof(root));
    root.children = kids;
    root.num_children = 2;
    root.bbox_max[0] = root.bbox_max[1] = 1.0;

    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "shared_tess_model", &root,
        tess, 1, &buf, &buf_size), 0);
    PRC_ASSERT_NOT_NULL(buf);

    fid = fopen(TEST_MATERIALS_FILENAME, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = prc_api_open_contents(ctx, TEST_MATERIALS_FILENAME);
    PRC_ASSERT_NOT_NULL(data);
    PRC_ASSERT_EQ(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups), 0);

    {
        const prc_tess_3d *t3 = data->file_struct[0].tessellation->tess[0].tess_3d;

        PRC_ASSERT_NOT_NULL(t3);
        PRC_ASSERT(t3->number_of_face_tessellation > 0);
        PRC_ASSERT_EQ(t3->face_tessellation_data[0].size_of_line_attributes, 0);
    }

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, NULL);
    remove(TEST_MATERIALS_FILENAME);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("test_write_materials");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_two_materials(ctx);
    test_default_still_applies(ctx);
    test_face_names_its_style(ctx);
    test_shared_tessellation_names_no_style(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
