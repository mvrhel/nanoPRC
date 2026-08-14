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

   WHAT: Reads a raw .prc byte stream (real or produced by another tool in
   this directory, e.g. one of the splice_ tools or reencode_real_tess)
   and wraps it, byte for byte and completely unmodified, in a new PDF
   using ONLY nanoPRC's own PDF container writer (prc_api_pdf_embed_prc)
   -- default page/margin/view options, no named views.

   HOW: usage: embed_real_prc.exe <input.prc> <output.pdf>. Just reads the
   whole input file into memory and calls prc_api_pdf_embed_prc once; does
   no parsing or validation of the PRC content itself.

   WHY IT EXISTS: The complementary test to a section-splice test. Instead
   of putting OUR generated PRC content into a REAL, proven-working PDF
   container (what the splice_* tools do), this puts REAL, independently-
   produced PRC content into OUR PDF container. If a reader (e.g. Adobe
   Acrobat) populates the model tree from the result, the container/PDF-
   wrapping code is cleared as a suspect and the bug must be in PRC
   content generation; if it still shows a blank tree, the container code
   itself is implicated. This was the test that first proved nanoPRC's
   PDF container code is NOT the cause of the Acrobat blank-model-tree
   bug (see the "Acrobat blank-tree investigation" project notes).

   LIMITATIONS: Performs no validation -- if the input isn't a well-formed
   .prc stream, the resulting PDF will simply fail to open too, with no
   more specific diagnostic than that. Always uses default PDF options
   (Letter-ish page, half-inch margins, no named views); does not expose
   a way to add a camera view. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long size;
    uint8_t *buf;
    size_t read_size;
    int code;

    if (argc < 3)
    {
        printf("usage: %s <input.prc> <output.pdf>\n", argv[0]);
        return 2;
    }

    fid = fopen(argv[1], "rb");
    if (fid == NULL)
    {
        printf("failed to open %s\n", argv[1]);
        return 1;
    }
    fseek(fid, 0L, SEEK_END);
    size = ftell(fid);
    fseek(fid, 0L, SEEK_SET);

    buf = (uint8_t *)malloc((size_t)size);
    if (buf == NULL)
    {
        printf("allocation failed\n");
        fclose(fid);
        return 1;
    }
    read_size = fread(buf, 1, (size_t)size, fid);
    fclose(fid);
    if (read_size != (size_t)size)
    {
        printf("short read\n");
        free(buf);
        return 1;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        printf("context creation failed\n");
        free(buf);
        return 1;
    }

    code = prc_api_pdf_embed_prc(ctx, argv[2], buf, (size_t)size, NULL);
    free(buf);
    if (code != 0)
    {
        printf("prc_api_pdf_embed_prc failed: code=%d\n", code);
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }

    printf("Wrote %s (embedding %ld bytes of real PRC content)\n", argv[2], size);
    prc_api_release_context(ctx);
    return 0;
}
