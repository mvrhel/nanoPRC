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

/* INTERNAL DEVELOPMENT TOOL -- not registered with CTest, no exit-code
   contract.

   WHAT: Counts PRC_TYPE_MKP_View entities (Table 127) in a file's model tree,
   and how many of them set is_default_view (Table 128).

   usage: view_census <file.prc|file.pdf> [more files...]

   WHY IT EXISTS: is_default_view is the only statement of "which camera should
   this model use" that does not come from the PDF 3D annotation, so it is the
   only one available when a .prc is rendered on its own. Two questions came up
   that need counting rather than reasoning: how often a model declares a view
   at all, and whether more than one view per file sets the default flag -- the
   clause reads "If TRUE the view is the default view", which sounds like it
   should be unique but does not say so.

   A count from a second reader is worth having because the alternative
   explanation for a surprising number of views is not a surprising file, it is
   a tree walk that has lost alignment and is manufacturing entities out of
   whatever follows.

   WHERE THEY LIVE: views hang off two different structures -- parts
   (prc_asm_parts_definition.views) and product occurrences
   (prc_asm_product_occurrence.views). Counting only one of them undercounts,
   which is worth stating because the two are easy to confuse and only the
   product-occurrence array is reachable from the public API's model tree. */

#include <stdio.h>
#include <string.h>

#include "prc_api.h"
#include "prc_context.h"
#include "prc_data.h"

static void
census_one(const char *path)
{
    prc_context *ctx;
    prc_data *pd;
    uint32_t fs, k, v;
    unsigned long views_parts = 0, views_products = 0;
    unsigned long defaults_parts = 0, defaults_products = 0;

    ctx = prc_new_context(NULL);
    if (ctx == NULL)
    {
        printf("%s: could not create a context\n", path);
        fflush(stdout);
        return;
    }

    pd = (prc_data *)prc_api_open_contents(ctx, path);
    if (pd == NULL)
    {
        printf("%s: open failed\n", path);
        fflush(stdout);
        prc_api_release_context(ctx);
        return;
    }

    for (fs = 0; fs < pd->file_structure_count; fs++)
    {
        prc_asm_file_structure_tree *tree = pd->file_struct[fs].tree;

        if (tree == NULL)
            continue;

        for (k = 0; k < tree->parts_count; k++)
        {
            prc_asm_parts_definition *part = &tree->parts[k];
            views_parts += part->number_views;
            for (v = 0; v < part->number_views; v++)
                if (part->views != NULL && part->views[v].is_default_view)
                    defaults_parts++;
        }

        for (k = 0; k < tree->product_count; k++)
        {
            prc_asm_product_occurrence *prod = &tree->products[k];
            views_products += prod->number_of_views;
            for (v = 0; v < prod->number_of_views; v++)
                if (prod->views != NULL && prod->views[v].is_default_view)
                    defaults_products++;
        }
    }

    printf("%s\n", path);
    printf("  file structures        : %u\n", pd->file_structure_count);
    printf("  views on parts         : %lu  (default %lu)\n", views_parts, defaults_parts);
    printf("  views on products      : %lu  (default %lu)\n", views_products, defaults_products);
    printf("  views total            : %lu  (default %lu)\n",
           views_parts + views_products, defaults_parts + defaults_products);

    prc_api_release_data(ctx, (prc_api_data)pd, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_api_release_context(ctx);

    /* Flushed per file rather than left to exit. Several files in the public
       corpus still crash this library, and a crash on a later argument throws
       away the buffered results of every earlier one -- which is how this line
       came to be here. A diagnostic that loses the measurements it already took
       is worse than one that is merely slow. */
    fflush(stdout);
}

int
main(int argc, char **argv)
{
    int i;

    if (argc < 2)
    {
        printf("usage: view_census <file.prc|file.pdf> [more files...]\n");
        return 1;
    }

    for (i = 1; i < argc; i++)
        census_one(argv[i]);

    return 0;
}
