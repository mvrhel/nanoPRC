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

   WHAT: Wraps an existing raw .prc byte stream (e.g. one produced by
   extract_raw_prc.c, or one of the per-file-structure slices produced
   while investigating a multi-file-structure container) into a new,
   minimal, single-page PDF 1.7 file as a standard 3D annotation, using
   the public API's own prc_api_pdf_embed_prc (bytes copied in as-is, no
   re-encoding) -- so an isolated raw .prc fragment that only our own
   low-level tools can open can be checked stand-alone in a real PDF
   viewer (Acrobat, PDF-XChange) instead.

   HOW: usage: prc_to_pdf.exe <input.prc> <output.pdf> */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    int code;

    if (argc < 3)
    {
        printf("usage: %s <input.prc> <output.pdf>\n", argv[0]);
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
    if (memcmp(buf, "PRC", 3) != 0) { printf("not a raw PRC file\n"); return 1; }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    code = prc_api_pdf_embed_prc(ctx, argv[2], buf, (size_t)fsize, NULL);
    if (code < 0)
    {
        printf("prc_api_pdf_embed_prc failed: %d\n", code);
        prc_api_print_error_stack(ctx);
        return 1;
    }

    printf("Wrote %s (%ld bytes of PRC embedded)\n", argv[2], fsize);

    free(buf);
    prc_api_release_context(ctx);
    return 0;
}
