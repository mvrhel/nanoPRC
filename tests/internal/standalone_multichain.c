/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Minimal repro for a hypothesis raised investigating UK_original.stl's
   Acrobat blank-model-tree bug (2026-07-23, see
   E:\Work\nanoPRC_supplementary_files\ISO-SPEC\
   hand-stl-acrobat-blanktree-investigation-22July.md): isolating that
   file's connected components one at a time found ONE small (358-triangle)
   piece that still produces Acrobat blank-tree even though (a) nanoPRC's
   own reader round-trips it correctly, (b) an independent, non-nanoPRC PRC
   reader also reads back the correct triangle count/structure -- so this is
   NOT data loss, same category as the three already-fixed defect classes
   (non-manifold edge/vertex, sliver triangles), not the entry-397-style
   multi-entry corruption bug (also already fixed, unrelated). That specific
   piece's own diagnostics (PRC_DIAG_MESH_QUALITY) showed something new: the
   non-manifold-vertex fix's private-vertex-splitting, applied to that
   piece's 4 non-manifold vertices, leaves it as ONE tessellation entry
   (num_faces=1, matching STL-import's own weld-based "this is one
   connected part" judgment) but with 5 GENUINELY DISCONNECTED sub-graphs
   internally (prc_encode_preprocess's own num_components=5) -- i.e. one
   COMPRESSED entry whose EdgeBreaker traversal needs 5 separate restart
   chains, not the usual 1. This tool tests that specific structural
   property in isolation, built from truly disjoint pieces from the start
   (no non-manifold topology at all, so if this alone reproduces the bug,
   non-manifold-ness isn't even a necessary ingredient -- just "many
   disconnected chains in one entry" is enough).

   HOW: usage: standalone_multichain.exe <num_pieces> <output.prc>
   [output.pdf]
   Writes <num_pieces> single-triangle, mutually disjoint (no shared
   vertices, spaced well apart) pieces as ONE COMPRESSED tessellation
   entry, num_faces=1 (mirroring uk_c3's own shape -- a single face
   containing multiple disconnected components), rather than nanoPRC's own
   default STL-import behavior of giving each disjoint piece its own
   separate tessellation entry. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    uint32_t num_pieces;
    const char *out_pdf = NULL;
    double *positions;
    uint32_t *tri_indices;
    uint32_t num_positions, num_triangles;
    uint32_t face_tri_counts[1];
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;
    uint32_t i;
    double bbox_span;

    if (argc < 3)
    {
        printf("usage: %s <num_pieces> <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }
    num_pieces = (uint32_t)strtoul(argv[1], NULL, 10);
    if (argc >= 4) out_pdf = argv[3];
    if (num_pieces == 0)
    {
        printf("num_pieces must be >= 1\n");
        return 2;
    }

    num_positions = num_pieces * 3;
    num_triangles = num_pieces;
    positions = (double *)malloc(sizeof(double) * 3 * num_positions);
    tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * num_triangles);
    if (positions == NULL || tri_indices == NULL)
    {
        printf("allocation failed\n");
        return 1;
    }

    /* Each piece is a small triangle offset by 10 units along x from the
       last -- far enough apart that no weld tolerance nanoPRC uses could
       ever merge them, so these stay genuinely, unambiguously disjoint. */
    for (i = 0; i < num_pieces; i++)
    {
        double ox = (double)i * 10.0;
        positions[i * 9 + 0] = ox + 0.0; positions[i * 9 + 1] = 0.0; positions[i * 9 + 2] = 0.0;
        positions[i * 9 + 3] = ox + 1.0; positions[i * 9 + 4] = 0.0; positions[i * 9 + 5] = 0.0;
        positions[i * 9 + 6] = ox + 0.5; positions[i * 9 + 7] = 1.0; positions[i * 9 + 8] = 0.0;
        tri_indices[i * 3 + 0] = i * 3 + 0;
        tri_indices[i * 3 + 1] = i * 3 + 1;
        tri_indices[i * 3 + 2] = i * 3 + 2;
    }
    bbox_span = (double)(num_pieces - 1) * 10.0 + 1.0;

    face_tri_counts[0] = num_triangles;

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = num_positions;
    tess.normals = NULL;
    tess.num_normals = 0;
    tess.tri_indices = tri_indices;
    tess.norm_indices = NULL;
    tess.num_triangles = num_triangles;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1; /* the key property being tested: ONE face, many disconnected chains */
    tess.crease_angle_degrees = 30.0;

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = -1.0; part_node.bbox_min[1] = -1.0; part_node.bbox_min[2] = -1.0;
    part_node.bbox_max[0] = bbox_span + 1.0; part_node.bbox_max[1] = 2.0; part_node.bbox_max[2] = 1.0;
    part_node.name = "multichain_body";
    part_node.part_name = "multichain_faces";

    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "multichain";

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); free(positions); free(tri_indices); return 1; }

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
            free(positions); free(tri_indices);
            return 1;
        }

        out = fopen(argv[2], "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            printf("failed to write %s\n", argv[2]);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            free(positions); free(tri_indices);
            return 1;
        }
        fclose(out);
        printf("num_pieces=%u num_triangles=%u num_positions=%u\n", num_pieces, num_triangles, num_positions);
        printf("Wrote %s (%zu bytes)\n", argv[2], prc_buf_size);

        if (out_pdf != NULL)
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = bbox_span * 0.5; view.eye[1] = -bbox_span * 0.8; view.eye[2] = bbox_span * 0.6;
            view.target[0] = bbox_span * 0.5; view.target[1] = 0.0; view.target[2] = 0.0;
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
    free(tri_indices);
    return 0;
}
