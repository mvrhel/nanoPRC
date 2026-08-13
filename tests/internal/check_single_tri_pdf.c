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

   WHAT: Generates a minimal single-triangle 3D-PDF via nanoPRC's own
   complete, ordinary write path (prc_api_write_prc_buffer + prc_api_pdf_
   embed_prc) -- the simplest possible non-trivial output this write
   facility can produce, with no splicing or hand-crafted bytes involved.

   HOW: Builds one compressed-tessellation triangle in memory (3 explicit
   positions, no supplied normals so the encoder recalculates them),
   wraps it in a single unnamed part/product tree, and writes it straight
   out to a PDF via the public API, exactly the way a real SDK consumer
   would.

   WHY IT EXISTS: Part of the investigation into Adobe Acrobat showing a
   blank model tree for this write facility's compressed-tessellation
   output (see the "Acrobat blank-tree investigation" project notes) --
   used to test whether the symptom is content-encoding-related in
   general, or specific to larger/more complex meshes (every real
   reference file used elsewhere in this investigation happens to have
   1-8 faces; nanoPRC's own teapot demo has 2048).

   LIMITATIONS: Hardcoded single triangle, hardcoded tolerance (1e-4
   absolute), no CLI arguments -- always writes to a fixed output path in
   the working directory. Not a regression test: does not itself assert
   anything about Acrobat's behavior, only produces a file for manual
   testing in a real 3D-PDF viewer. */
#include <stdio.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

int main(void)
{
    prc_context *ctx = NULL;
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep;
    prc_api_write_node root;
    prc_pdf_view_spec view;
    prc_pdf_write_options pdf_opts;
    double positions[9] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    uint8_t *prc_buf = NULL;
    size_t prc_size = 0;
    FILE *out;
    int code;

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("new_context failed\n"); return 1; }

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = 3;
    tess.normals = NULL;
    tess.tri_indices = tris;
    tess.num_triangles = 1;
    tess.tolerance = prc_write_tol_absolute(1e-4);
    tess.crease_angle_degrees = 30.0;

    memset(&rep, 0, sizeof(rep));
    rep.kind = PRC_API_WRITE_RI_SURFACE;
    rep.biased_tessellation_index = 1;
    rep.is_closed = 0;

    memset(&root, 0, sizeof(root));
    root.rep_items = &rep;
    root.num_rep_items = 1;
    /* TEMP experiment: a real, working reference file's PartDefinition uses
       a "practically infinite" placeholder bbox (-1e20..1e20 per axis)
       instead of a real, tight computed one -- testing whether Acrobat's
       tree population depends on this. */
    root.bbox_min[0] = -1e20; root.bbox_min[1] = -1e20; root.bbox_min[2] = -1e20;
    root.bbox_max[0] = 1e20; root.bbox_max[1] = 1e20; root.bbox_max[2] = 1e20;
    /* TEMP experiment: the working reference file's ProductOccurrence has
       no name at all ("Unnamed Product"); ours always names it. */
    root.name = NULL;
    root.part_name = "SingleTriPart";

    code = prc_api_write_prc_buffer(ctx, "SingleTriModel", &root, &tess, 1, &prc_buf, &prc_size);
    if (code != 0)
    {
        printf("write_prc_buffer failed: %d\n", code);
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }

    out = fopen("single_tri.prc", "wb");
    if (out != NULL) { fwrite(prc_buf, 1, prc_size, out); fclose(out); }
    printf("Wrote single_tri.prc (%u bytes)\n", (unsigned)prc_size);

    memset(&view, 0, sizeof(view));
    view.name = "Default";
    view.eye[0] = 3.0; view.eye[1] = -4.0; view.eye[2] = 2.5;
    view.target[0] = 0.33; view.target[1] = 0.33; view.target[2] = 0.0;
    view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
    view.is_default = 1;

    memset(&pdf_opts, 0, sizeof(pdf_opts));
    pdf_opts.views = &view;
    pdf_opts.num_views = 1;

    code = prc_api_pdf_embed_prc(ctx, "single_tri.pdf", prc_buf, prc_size, &pdf_opts);
    if (code != 0)
    {
        printf("pdf_embed_prc failed: %d\n", code);
        prc_api_print_error_stack(ctx);
    }
    else
    {
        printf("Wrote single_tri.pdf\n");
    }

    prc_api_write_prc_buffer_free(ctx, prc_buf);
    prc_api_release_context(ctx);
    return 0;
}
