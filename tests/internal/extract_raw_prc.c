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

   WHAT: Extracts the raw PRC byte stream embedded in a PDF's 3D
   annotation and writes it out as a standalone .prc file.

   HOW: usage: extract_raw_prc.exe <input.pdf> <output.prc>. Calls nanoPRC's
   own internal PDF-extraction function (pdf_extract_prc, declared in the
   private prc_pdf.h -- not part of the public API) directly, bypassing
   the full parse pipeline, so it works even on PDFs whose PRC content
   nanoPRC cannot yet fully parse.

   WHY IT EXISTS: The other diagnostic tools in this directory (the
   splice_ tools and reencode_real_tess) operate on raw .prc byte streams,
   not PDFs, since they need direct access to the PRC main-header/section-
   offset table. Real-world reference files are usually only available as
   PDFs (e.g. downloaded vendor samples, or files with attachments
   embedded in a spec document), so this tool bridges the gap. Also
   transparently handles PDF-level decryption (owner-password/
   permissions-only encryption, common in vendor sample PDFs) since it
   goes through the same internal extraction path prc_api_open_contents
   itself uses.

   LIMITATIONS: Extracts only the FIRST 3D stream found; does not handle
   PDFs with multiple independent 3D annotations. No validation of the
   extracted bytes -- if extraction silently grabs the wrong stream (e.g.
   a malformed or unusual PDF structure), the output will just fail to
   parse downstream with no more specific diagnostic than that. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_pdf.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    uint8_t *prc_out = NULL;
    uint32_t prc_size = 0;
    prc_pdf_view_array *views = NULL;
    uint32_t number_views = 0;
    int code;
    FILE *out_fid;

    if (argc < 3)
    {
        printf("usage: %s <input.pdf> <output.prc>\n", argv[0]);
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

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    code = pdf_extract_prc(ctx, buf, (uint32_t)fsize, &prc_out, &prc_size, &views, &number_views);
    if (code < 0)
    {
        printf("pdf_extract_prc failed: code=%d\n", code);
        prc_api_print_error_stack(ctx);
        return 1;
    }

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(prc_out, 1, prc_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%u bytes)\n", argv[2], prc_size);

    prc_free(ctx, prc_out);
    prc_api_release_context(ctx);
    free(buf);
    return 0;
}
