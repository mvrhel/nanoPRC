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

   WHAT: Opens a PDF/PRC file and reports whether its schema declares a
   PRC_TYPE_ROOTBaseWithGraphics extension -- the condition that makes
   splice_tree_section.c's tree-only splice methodologically invalid (see
   that tool's own header comment: when this extension is declared, every
   base-with-graphics record in the tree carries extra schema-defined data
   this write facility's tree encoder never emits, causing an immediate
   bitstream desync unrelated to any genuine tree-encoding defect).

   WHY IT EXISTS: Built to settle, before trusting any result, whether a
   given reference oracle file's schema declares this extension -- needed
   to know which splice tools produce methodologically valid results for
   that specific file, as part of building a clean evidence matrix for the
   Acrobat-blank-model-tree (TRIANGLES write path) investigation.

   HOW: usage: check_schema_extension.exe <input.pdf_or_prc> */
#include <stdio.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_parse_common.h"
#include "prc_data.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    int code;

    if (argc < 2)
    {
        printf("usage: %s <input.pdf_or_prc>\n", argv[0]);
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL)
    {
        printf("open failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }

    code = prc_check_for_schema(ctx, PRC_TYPE_ROOTBaseWithGraphics);
    if (code > 0)
        printf("SCHEMA DECLARES PRC_TYPE_ROOTBaseWithGraphics extension (index %d)\n", code);
    else
        printf("schema does NOT declare a PRC_TYPE_ROOTBaseWithGraphics extension\n");

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL);
    prc_api_release_context(ctx);
    return 0;
}
