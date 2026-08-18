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

   WHAT: Writes a fully-standalone PRC/PDF file (same prc_api_write_prc_buffer
   path as standalone_tetrahedron_triangles.c/teapot_write.c) containing a
   synthetic, tunable-size NxN grid mesh (2*(N-1)*(N-1) triangles), with real
   per-vertex normals (norm_indices != NULL, the PRC_FACETESSDATA_Triangle
   branch -- see standalone_tetrahedron_triangles.c row 15's own comment for
   why this branch matters), single face, TRIANGLES kind.

   WHY IT EXISTS: standalone_tetrahedron_triangles.c's row 15 (4 triangles,
   real per-vertex normals) works in both Acrobat and PDF-XChange, but
   reauthor_tess_pdf.c's real turbine output (8868-26540 triangles, same
   PRC_FACETESSDATA_Triangle branch) fails in Acrobat only. This tool
   isolates whether pure SCALE (clean, synthetic, non-real-world data) alone
   reproduces the failure, independent of any property specific to the real
   decoded turbine geometry (degenerate triangles, extreme coordinate
   magnitudes, duplicate positions, etc.) -- run at a few sizes spanning the
   working/failing range to bisect where (if anywhere) it breaks.

   UPDATE 2026-07-15: the blank-tree evidence matrix's row 19 found that a flat
   two-level tree (root [no part] -> leaf [real part]) is itself a cause of
   Acrobat's blank canvas/tree for OPEN-topology meshes -- inserting one
   extra empty-part intermediate level (root [no part] -> intermediate
   [empty part] -> leaf [real part]) fixed a minimal 2-triangle quad in both
   Acrobat and PDF-XChange. But applying that identical tree fix to the real
   turbine geometry (reauthor_tess_pdf.c, rows 20/21) did NOT fix it -- still
   fails in Acrobat. This tool now ALWAYS uses the row-19 three-level tree
   shape (previously it used a flat two-level tree, which is no longer a
   live variable worth re-testing on its own) so that scale/complexity can be
   bisected as a SEPARATE variable from tree shape, to find out whether the
   deep-tree fix itself breaks down somewhere between this tool's small
   synthetic scale and the turbine's real 8868-26540 triangle scale, or
   whether scale was never the differentiator and something else in the real
   decoded geometry (multiple face groups, degenerate triangles, disconnected
   components, etc.) is still unaccounted for.

   HOW: usage: standalone_grid_triangles.exe <output.prc> <N> [output.pdf] [mustcalc] [flattree]
   [roundup|rounddown|roundzero] [compressed]
   N is the grid side length; produces N*N positions, 2*(N-1)*(N-1)
   triangles. E.g. N=100 -> 10000 positions, 19602 triangles (close to
   turbine tess 902's scale); N=175 -> ~30000 triangles (close to tess 901).

   UPDATE 2026-07-15 (six-agent meta-analysis plan item #2a): passing
   "mustcalc" as the 4th argument switches this tool from real per-vertex
   normals (norm_indices != NULL) to must_calculate_normals=1 (no stored
   normals at all, crease_angle=45 degrees) -- the untested convention
   ElevationMeshIS_ePRC.pdf/xml-sample-wrl_ePRC.pdf (real, Acrobat-working
   oracles) use and nanoPRC's writer could not previously emit at all. Row
   22a (N=10, real per-vertex normals) is CONFIRMED FAILING in Acrobat
   (10x reopen-determinism-checked); this mode isolates whether the missing
   must_calculate_normals convention, not scale or tree shape, is what
   Acrobat actually requires.

   UPDATE 2026-07-16: passing "flattree" as a 5th argument switches from the
   row-19 tree shape to a SHALLOW 2-level tree (Model -> root [HAS the real
   part directly, rep_items attached there] -> one nested empty part) --
   matching the tree shape found in a real, independently-produced,
   Acrobat-CONFIRMED-WORKING file (libPRC/asymptote's own harness output,
   `25_libprc_N70_mustcalc.pdf`, see blanktree-matrix-evidence-table.md's
   row 25) at the exact N=70/9522-triangle scale where THIS tool's row-19
   tree shape fails even with must_calculate_normals=1 (row 23c). Directly
   tests whether row-19's "needs an empty-part CONTAINER between the
   part-less root and the real part" mechanism is right, or whether
   (as libPRC's shallower working tree suggests) less nesting is what
   actually matters. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fenv.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    uint32_t n, num_positions, num_triangles;
    double *positions = NULL;
    double *normals = NULL;
    uint32_t *tri_indices = NULL;
    uint32_t face_tri_counts[1];
    uint32_t x, y, t;
    const char *out_pdf = NULL;
    int mustcalc = 0;
    int flattree = 0;
    int compressed = 0;
    int round_mode = FE_TONEAREST; /* default */
    int arg_i;
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, intermediate, root, nested;
    prc_api_write_node *part_node_ptr;
    prc_api_write_node *intermediate_ptr;
    prc_api_write_node *nested_ptr;
    double bbox_min[3], bbox_max[3];

    if (argc < 3)
    {
        printf("usage: %s <output.prc> <N> [output.pdf]\n", argv[0]);
        return 2;
    }
    n = (uint32_t)atoi(argv[2]);
    if (n < 2) { printf("N must be >= 2\n"); return 2; }
    if (argc >= 4) out_pdf = argv[3];
    for (arg_i = 4; arg_i < argc; arg_i++)
    {
        if (strcmp(argv[arg_i], "mustcalc") == 0) mustcalc = 1;
        if (strcmp(argv[arg_i], "flattree") == 0) flattree = 1;
        /* 2026-07-17 addition: PRC_API_WRITE_TESS_KIND_COMPRESSED instead of
           TRIANGLES, for scale/robustness stress-testing that write path
           independently -- see prc_api.h's own doc comment for its two
           separate, already-known, unrelated-to-Bug-#2 open issues (a
           per-vertex normal-sign bug, and an independent reader reportedly
           seeing null/empty geometry for COMPRESSED output). Not compatible
           with mustcalc (COMPRESSED has its own separate recalculated-
           normals convention via crease_angle_degrees, orthogonal to this
           tool's mustcalc flag, which only applies to TRIANGLES). */
        if (strcmp(argv[arg_i], "compressed") == 0) compressed = 1;
        /* 2026-07-16 addition: FPU rounding mode for the position/normal
           computation -- tests whether a specific ULP-level bit pattern in
           the computed doubles (not the encoding algorithm, already
           exhaustively ruled out elsewhere) is what trips Acrobat, per
           user suggestion. Defaults to FE_TONEAREST (IEEE754/normal libm
           behavior) when omitted, so existing invocations are unaffected. */
        if (strcmp(argv[arg_i], "roundup") == 0) round_mode = FE_UPWARD;
        if (strcmp(argv[arg_i], "rounddown") == 0) round_mode = FE_DOWNWARD;
        if (strcmp(argv[arg_i], "roundzero") == 0) round_mode = FE_TOWARDZERO;
    }
    fesetround(round_mode);

    /* A gently-curved surface (a shallow sine wave), not a flat plane, so
       real per-vertex normals actually vary -- structurally closer to real
       CAD geometry than a flat grid (whose vertex normals would all be
       identical, (0,0,1), and might accidentally not exercise whatever
       per-vertex-normal-diversity path matters here) than a genuine
       curved surface: a section of a cylinder, radius R=5, spanning +/-60
       degrees around its axis and the full length along it. Every vertex's
       normal is the true radial direction, so adjacent normals differ by a
       small but non-zero, exactly-known angle -- the closest analytic
       surface to "a real curved CAD panel" while still being fully
       synthetic/clean (no real-world data quirks). */
    {
        const double radius = 5.0;
        const double half_angle_deg = 60.0;
        const double half_angle = half_angle_deg * 3.14159265358979323846 / 180.0;
        const double half_length = 5.0;

        num_positions = n * n;
        positions = (double *)malloc(sizeof(double) * 3 * num_positions);
        normals = (double *)malloc(sizeof(double) * 3 * num_positions);
        for (y = 0; y < n; y++)
        {
            for (x = 0; x < n; x++)
            {
                uint32_t i = y * n + x;
                /* x sweeps the angle around the cylinder's axis (Z), y sweeps
                   straight along the axis. */
                double theta = ((double)x / (double)(n - 1) * 2.0 - 1.0) * half_angle;
                double along = ((double)y / (double)(n - 1) * 2.0 - 1.0) * half_length;
                double nx = sin(theta), ny = cos(theta); /* radial direction, in the XY plane */

                positions[i * 3 + 0] = radius * nx;
                positions[i * 3 + 1] = radius * ny;
                positions[i * 3 + 2] = along;
                normals[i * 3 + 0] = nx;
                normals[i * 3 + 1] = ny;
                normals[i * 3 + 2] = 0.0;
            }
        }
    }

    num_triangles = 2 * (n - 1) * (n - 1);
    tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * num_triangles);
    t = 0;
    for (y = 0; y + 1 < n; y++)
    {
        for (x = 0; x + 1 < n; x++)
        {
            uint32_t i00 = y * n + x, i10 = y * n + (x + 1);
            uint32_t i01 = (y + 1) * n + x, i11 = (y + 1) * n + (x + 1);

            tri_indices[t * 3 + 0] = i00; tri_indices[t * 3 + 1] = i10; tri_indices[t * 3 + 2] = i11; t++;
            tri_indices[t * 3 + 0] = i00; tri_indices[t * 3 + 1] = i11; tri_indices[t * 3 + 2] = i01; t++;
        }
    }

    face_tri_counts[0] = num_triangles;

    memset(&tess, 0, sizeof(tess));
    tess.kind = compressed ? PRC_API_WRITE_TESS_KIND_COMPRESSED : PRC_API_WRITE_TESS_KIND_TRIANGLES;
    tess.positions = positions;
    tess.num_positions = num_positions;
    tess.normals = normals;
    tess.num_normals = num_positions;
    tess.tri_indices = tri_indices;
    tess.norm_indices = tri_indices; /* real per-vertex normals, paired 1:1 with positions -- same convention as reauthor_tess_pdf.c */
    tess.num_triangles = num_triangles;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;
    if (mustcalc)
    {
        /* must_calculate_normals=1: no stored normals at all, matching the
           real-oracle convention (see file header comment). crease_angle=45
           degrees matches ElevationMeshIS_ePRC.pdf's own stored value; this
           mesh is genuinely curved (unlike the degenerate crease_angle=0
           tetrahedron oracle) so a non-trivial crease angle is meaningful. */
        tess.normals = NULL;
        tess.num_normals = 0;
        tess.norm_indices = NULL;
        tess.must_calculate_normals = 1;
        tess.crease_angle_degrees = 45.0;
    }

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    /* Cylinder section: radius 5, +/-60 degrees -> x in [-5*sin60, 5*sin60]
       ~ [-4.33, 4.33], y in [5*cos60, 5] = [2.5, 5.0], z (along axis) in
       [-5, 5]. Padded slightly. */
    bbox_min[0] = -4.5; bbox_min[1] = 2.0; bbox_min[2] = -5.5;
    bbox_max[0] = 4.5; bbox_max[1] = 5.5; bbox_max[2] = 5.5;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = bbox_min[0]; part_node.bbox_min[1] = bbox_min[1]; part_node.bbox_min[2] = bbox_min[2];
    part_node.bbox_max[0] = bbox_max[0]; part_node.bbox_max[1] = bbox_max[1]; part_node.bbox_max[2] = bbox_max[2];
    part_node.name = "grid_body";
    part_node.part_name = "grid_faces";

    if (flattree)
    {
        /* Shallow tree matching libPRC's own real, Acrobat-working output:
           Model -> root [HAS the real part directly, rep_items attached
           here] -> nested [one further empty part, no children]. No
           part-less container above the real part at all. */
        memset(&nested, 0, sizeof(nested));
        nested.has_empty_part = 1;
        nested.name = "grid_nested";
        /* A has_empty_part node's own bbox is still serialized and NOT
           safely ignorable by real Acrobat -- see stl_import.c's own fix
           for the same pattern. No child of its own to copy from here, so
           match root's (the real geometry's) bbox instead. */
        nested.bbox_min[0] = bbox_min[0]; nested.bbox_min[1] = bbox_min[1]; nested.bbox_min[2] = bbox_min[2];
        nested.bbox_max[0] = bbox_max[0]; nested.bbox_max[1] = bbox_max[1]; nested.bbox_max[2] = bbox_max[2];

        nested_ptr = &nested;
        memset(&root, 0, sizeof(root));
        root.rep_items = &rep_item;
        root.num_rep_items = 1;
        root.bbox_min[0] = bbox_min[0]; root.bbox_min[1] = bbox_min[1]; root.bbox_min[2] = bbox_min[2];
        root.bbox_max[0] = bbox_max[0]; root.bbox_max[1] = bbox_max[1]; root.bbox_max[2] = bbox_max[2];
        root.name = "root";
        root.part_name = "grid_faces";
        root.children = &nested_ptr;
        root.num_children = 1;
    }
    else
    {
        /* Row-19 tree shape: root [no part] -> intermediate [empty part] ->
           leaf [real part] -- see the file header comment's 2026-07-15 update. */
        part_node_ptr = &part_node;
        memset(&intermediate, 0, sizeof(intermediate));
        intermediate.children = &part_node_ptr;
        intermediate.num_children = 1;
        intermediate.name = "grid_scene";
        intermediate.has_empty_part = 1;
        /* A has_empty_part node's own bbox is still serialized and NOT
           safely ignorable by real Acrobat -- see stl_import.c's own fix
           for the same pattern. Reuse the one real child's bbox. */
        intermediate.bbox_min[0] = part_node.bbox_min[0]; intermediate.bbox_min[1] = part_node.bbox_min[1]; intermediate.bbox_min[2] = part_node.bbox_min[2];
        intermediate.bbox_max[0] = part_node.bbox_max[0]; intermediate.bbox_max[1] = part_node.bbox_max[1]; intermediate.bbox_max[2] = part_node.bbox_max[2];

        intermediate_ptr = &intermediate;
        memset(&root, 0, sizeof(root));
        root.children = &intermediate_ptr;
        root.num_children = 1;
        root.name = "grid";
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;
        FILE *out;

        printf("Encoding %u positions, %u triangles (N=%u, %s)...\n", num_positions, num_triangles, n,
            mustcalc ? "must_calculate_normals=1" : "real per-vertex normals");
        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, &tess, 1, &prc_buf, &prc_buf_size);
        if (code != 0)
        {
            printf("prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            prc_api_release_context(ctx);
            return 1;
        }

        out = fopen(argv[1], "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            printf("failed to write %s\n", argv[1]);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            return 1;
        }
        fclose(out);
        printf("Wrote %s (%zu bytes)\n", argv[1], prc_buf_size);

        if (out_pdf != NULL)
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = 15.0; view.eye[1] = -15.0; view.eye[2] = 12.0;
            view.target[0] = 0.0; view.target[1] = 3.75; view.target[2] = 0.0;
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;

            code = prc_api_pdf_embed_prc(ctx, out_pdf, prc_buf, prc_buf_size, &pdf_opts);
            if (code < 0)
            {
                printf("prc_api_pdf_embed_prc failed: %d\n", code);
                prc_api_print_error_stack(ctx);
            }
            else
            {
                printf("Wrote %s (with explicit Default view)\n", out_pdf);
            }
        }

        prc_api_write_prc_buffer_free(ctx, prc_buf);
    }

    prc_api_release_context(ctx);
    free(positions);
    free(normals);
    free(tri_indices);
    return 0;
}
