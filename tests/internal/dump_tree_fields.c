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

   WHAT: Opens a file through the normal public read API, walks the full
   product/part tree, and dumps every product occurrence's and part's
   public-API-level fields (name, type, num_children, biased_tess_index,
   biased_style_index, biased_layer_index, has_inherited_style,
   part_detail_index, has_entity_ref, behavior_bit_field1/2,
   biased_local_coordinate_index, num_rep_items) as plain text.

   WHY IT EXISTS: dump_uncompressed_tess_fields.c compared the tessellation
   record itself between real-world uncompressed-PRC files and this write
   facility's own TRIANGLES output and found it mostly matches real
   convention -- so the remaining Acrobat-blank-model-tree divergence (see
   reauthor_tess_pdf.c's header comment) is more likely one level up, in
   the part/product-occurrence/rep-item structure. This tool is the
   tree-level analog of that comparison, at the public API level (no raw
   bitstream parsing needed, since prc_api_part/prc_api_product already
   expose the fields worth comparing).

   HOW: usage: dump_tree_fields.exe <file.prc|file.pdf> */
#include <stdio.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

static void
dump_part(const prc_api_part *part, int depth)
{
    if (part == NULL) return;
    printf("%*spart name=%s biased_tess_index=%u tess_file_index=%u biased_style_index=%u "
           "biased_layer_index=%u behavior_bit_field1=%u behavior_bit_field2=%u "
           "biased_local_coordinate_index=%u num_rep_items=%zu has_inherited_style=%u "
           "part_detail_index=%u has_entity_ref=%u\n",
        depth * 2, "", part->name ? part->name : "(null)",
        part->biased_tess_index, part->tess_file_index, part->biased_style_index,
        part->biased_layer_index, part->behavior_bit_field1, part->behavior_bit_field2,
        part->biased_local_coordinate_index, part->num_rep_items, part->has_inherited_style,
        part->part_detail_index, part->has_entity_ref);
    {
        size_t i;
        for (i = 0; i < part->num_rep_items; i++)
            dump_part(&part->rep_items[i], depth + 1);
    }
}

static void
dump_product(const prc_api_product *prod, int depth)
{
    size_t i;
    if (prod == NULL) return;
    printf("%*sproduct name=%s is_model=%u type=%d num_children=%zu location_set=%u "
           "num_markups=%u file_index=%d has_part=%d\n",
        depth * 2, "", prod->name ? prod->name : "(null)",
        prod->is_model, (int)prod->type, prod->num_children, prod->location_set,
        prod->num_markups, prod->file_index, prod->part != NULL);
    dump_part(prod->part, depth + 1);
    for (i = 0; i < prod->num_children; i++)
        dump_product(&prod->children[i], depth + 1);
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    int code;

    if (argc < 2) { printf("usage: %s <file.prc|file.pdf>\n", argv[0]); return 2; }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("RESULT\tCONTEXT_FAIL\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("RESULT\tOPEN_FAIL\n"); prc_api_print_error_stack(ctx); return 1; }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code != 0) { printf("RESULT\tPREP_TREE_FAIL\tcode=%d\n", code); return 1; }

    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code != 0) { printf("RESULT\tCREATE_TREE_FAIL\tcode=%d\n", code); return 1; }

    dump_product(model_tree, 0);

    printf("RESULT\tOK\n");
    prc_api_release_context(ctx);
    return 0;
}
