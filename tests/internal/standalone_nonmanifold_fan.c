/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Minimal (<=8 triangle) COMPRESSED-tessellation repro for the
   hand.stl Acrobat blank-model-tree root cause found 2026-07-22 (see
   E:\Work\nanoPRC_supplementary_files\ISO-SPEC\
   hand-stl-acrobat-blanktree-investigation-22July.md): a real (non-
   duplicate, non-degenerate) edge shared by 4 distinct triangles -- a
   "fan" -- embedded in an otherwise-ordinary small manifold patch, rather
   than floating in isolation, to keep it structurally similar to how the
   real defect appears in hand.stl (one non-manifold patch inside a large
   otherwise-2-manifold surface). Confirmed on the full 109,833-triangle
   file that (a) the DEFAULT behavior (silently link first two triangles
   on the edge, drop the rest) produces Acrobat blank-tree; (b) fully
   REMOVING the excess triangles fixes it; (c) merely un-linking the edge
   internally (keeping all triangles, but treating the edge as a boundary
   for everyone during traversal) does NOT fix it -- the final decoded
   geometry is unchanged either way, suggesting Acrobat validates
   reconstructed 3D topology, not our encoder's internal adjacency
   bookkeeping. This tool tests untried workarounds at a scale small
   enough to iterate quickly, before committing to any change in the
   production encoder.

   Vertices (8): A, B are the shared-fan edge's endpoints; C1/C2 are the
   two "normal" (2-manifold) triangles' apexes; C3/C4 are the two EXTRA
   fan triangles' apexes (the ones a 2-manifold traversal can't represent
   as adjacent to anything); D, F extend the mesh slightly past C1/C2 so
   the fan sits inside a small patch rather than floating alone.

   HOW: usage: standalone_nonmanifold_fan.exe <mode> <output.prc>
   [output.pdf]
   mode:
     baseline4   - all 6 triangles (4-fan on A-B + 2 extension wings),
                   default encoder behavior (silently links first two on
                   A-B, drops the rest) -- expected to reproduce the bug
                   at this tiny scale if the defect is genuinely about the
                   fan itself, not something scale-dependent.
     exclude     - drop the 2 extra fan triangles (C3,C4) entirely, keep
                   the other 4 -- matches the fix already confirmed
                   working at full scale.
     splitvertex - keep all 6 triangles, but give the 2 extra fan
                   triangles (C3,C4) their OWN PRIVATE COPY of vertices A
                   and B (a tiny numerical epsilon offset, not true
                   duplicates -- see WHY below) so they no longer share
                   literal vertex indices with the "normal" pair, avoiding
                   mesh-topology non-manifoldness while preserving all
                   surface area/visual appearance. Untested workaround --
                   the whole point of this tool.
     threefan    - 5 triangles: only 3 on edge A-B (drop C4) -- tests
                   whether a 3-triangle (not just 4-triangle) fan alone
                   is already enough to reproduce the bug.

   WHY splitvertex uses a tiny epsilon offset, not exact duplicate
   coordinates: the write facility's own weld tolerance (or a caller's,
   if it re-welds) would silently merge exactly-duplicate positions right
   back into the same non-manifold configuration. An offset well below
   any reasonable weld tolerance but large enough to produce genuinely
   different quantized point_array values tests whether this is a viable
   real workaround (imperceptible seam) as opposed to merely appearing to
   work because two encoders' welds disagree on the epsilon. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    const char *mode;
    const char *out_pdf = NULL;
    double eps = 1e-3; /* well above any reasonable weld tolerance's floor, well below the mesh's own ~1-2 unit scale */

    /* Base 8 positions: A=0 B=1 C1=2 C2=3 C3=4 C4=5 D=6 F=7 */
    double positions[10 * 3] = {
        0.0, 0.0, 0.0,      /* 0: A */
        1.0, 0.0, 0.0,      /* 1: B */
        0.5, 1.5, 0.0,      /* 2: C1 */
        0.5, -1.5, 0.0,     /* 3: C2 */
        0.5, 0.4, 1.0,      /* 4: C3 (extra fan #1) */
        0.5, 0.3, -0.9,     /* 5: C4 (extra fan #2) */
        2.0, 0.8, 0.0,      /* 6: D (extends past C1) */
        -1.0, -0.8, 0.0,    /* 7: F (extends past C2) */
        0.0, 0.0, 0.0,      /* 8: A' (splitvertex private copy, filled in below) */
        1.0, 0.0, 0.0       /* 9: B' (splitvertex private copy, filled in below) */
    };
    uint32_t tri_indices[8 * 3];
    uint32_t num_positions = 8;
    uint32_t num_triangles = 0;
    uint32_t face_tri_counts[1];
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;

    if (argc < 3)
    {
        printf("usage: %s <mode:baseline4|exclude|splitvertex|threefan> <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }
    mode = argv[1];
    if (argc >= 4) out_pdf = argv[3];

    if (strcmp(mode, "baseline4") == 0)
    {
        uint32_t t[] = {
            0,1,2,  /* A,B,C1 -- normal */
            1,0,3,  /* B,A,C2 -- normal */
            0,1,4,  /* A,B,C3 -- extra fan triangle */
            0,1,5,  /* A,B,C4 -- extra fan triangle */
            1,6,2,  /* B,D,C1 -- extends past C1 */
            3,0,7   /* C2,A,F -- extends past C2 */
        };
        num_triangles = 6;
        memcpy(tri_indices, t, sizeof(t));
    }
    else if (strcmp(mode, "exclude") == 0)
    {
        uint32_t t[] = {
            0,1,2,  /* A,B,C1 */
            1,0,3,  /* B,A,C2 */
            1,6,2,  /* B,D,C1 */
            3,0,7   /* C2,A,F */
        };
        num_triangles = 4;
        memcpy(tri_indices, t, sizeof(t));
    }
    else if (strcmp(mode, "splitvertex") == 0)
    {
        /* A'=8, B'=9: private, epsilon-offset copies of A/B used only by
           the two extra fan triangles, so edge A-B stays exactly
           2-manifold (only T0/T1 reference it) while the extra
           triangles' geometry is visually indistinguishable. */
        positions[8*3+0] = 0.0 + eps; positions[8*3+1] = 0.0 + eps; positions[8*3+2] = 0.0;
        positions[9*3+0] = 1.0 + eps; positions[9*3+1] = 0.0 + eps; positions[9*3+2] = 0.0;
        num_positions = 10;
        {
            uint32_t t[] = {
                0,1,2,  /* A,B,C1 -- normal */
                1,0,3,  /* B,A,C2 -- normal */
                8,9,4,  /* A',B',C3 -- extra fan triangle, private edge copy */
                8,9,5,  /* A',B',C4 -- extra fan triangle, private edge copy */
                1,6,2,  /* B,D,C1 */
                3,0,7   /* C2,A,F */
            };
            num_triangles = 6;
            memcpy(tri_indices, t, sizeof(t));
        }
    }
    else if (strcmp(mode, "threefan") == 0)
    {
        uint32_t t[] = {
            0,1,2,  /* A,B,C1 -- normal */
            1,0,3,  /* B,A,C2 -- normal */
            0,1,4,  /* A,B,C3 -- extra fan triangle (only ONE extra, 3 total on the edge) */
            1,6,2,  /* B,D,C1 */
            3,0,7   /* C2,A,F */
        };
        num_triangles = 5;
        memcpy(tri_indices, t, sizeof(t));
    }
    else
    {
        printf("unrecognized mode '%s'\n", mode);
        return 2;
    }

    face_tri_counts[0] = num_triangles;

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = num_positions;
    tess.normals = NULL;               /* must_recalculate_normals path, matching hand.stl's default encoding */
    tess.num_normals = 0;
    tess.tri_indices = tri_indices;
    tess.norm_indices = NULL;
    tess.num_triangles = num_triangles;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;
    tess.crease_angle_degrees = 30.0;

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = -2.0; part_node.bbox_min[1] = -2.0; part_node.bbox_min[2] = -1.5;
    part_node.bbox_max[0] = 2.5; part_node.bbox_max[1] = 2.0; part_node.bbox_max[2] = 1.5;
    part_node.name = "fan_body";
    part_node.part_name = "fan_faces";

    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "nonmanifold_fan";

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;
        FILE *out;

        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, &tess, 1, &prc_buf, &prc_buf_size);
        if (code != 0)
        {
            printf("prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            prc_api_release_context(ctx);
            return 1;
        }

        out = fopen(argv[2], "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            printf("failed to write %s\n", argv[2]);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            return 1;
        }
        fclose(out);
        printf("mode=%s num_triangles=%u num_positions=%u\n", mode, num_triangles, num_positions);
        printf("Wrote %s (%zu bytes)\n", argv[2], prc_buf_size);

        if (out_pdf != NULL)
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = 6.0; view.eye[1] = -6.0; view.eye[2] = 4.0;
            view.target[0] = 0.5; view.target[1] = 0.0; view.target[2] = 0.0;
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
    return 0;
}
