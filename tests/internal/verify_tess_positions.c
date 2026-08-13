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

   WHAT: Encodes a small, hand-computable, KNOWN-COORDINATE mesh through
   nanoPRC's own compressed-tessellation encoder, then decodes it back
   through the real parser, and prints the decoded per-triangle vertex
   positions IN TRIANGLE ORDER alongside the original input -- unlike the
   permanent test suite's round-trip checks (test_compress_tess.c), which
   use an order-agnostic "does each decoded vertex land near SOME input
   vertex" comparison (see nearest_input_vertex there) that cannot detect
   a winding-order or frame-sign error that still places every decoded
   vertex near a geometrically correct position.

   HOW: usage: verify_tess_positions.exe (no arguments). Mesh is a 4-
   triangle fan around a shared central vertex (5 vertices total: one
   center + 4 rim points at simple axis-aligned coordinates), specifically
   chosen so the SEED triangle has TWO unvisited neighbors and therefore
   produces edge_status_array[0] == 3 (both the right-child AND left-
   child bits set) -- this is the critical case a simpler 1-2-triangle
   test cannot exercise, since a lone seed triangle has no children at
   all and a simple 2-triangle chain only ever grows ONE child (right OR
   left, never both), silently leaving half of the encoder/decoder's gate
   logic completely untested. Prints both C1 (must_recalculate_normals)
   and C2 (supplied-normals) paths.

   WHY IT EXISTS: Built after a same-day attempt to fix a suspected gate-
   direction bug in the compressed-tessellation encoder (right-child gate
   basis computed from vertices in the wrong order, confirmed via careful
   manual tracing against the oracle document's pseudocode) made things
   WORSE, not better, when tested in real readers (PDF-XChange showed the
   fixed puck re-encode still black; the fixed teapot's mesh was
   "destroyed, unrecognizable" -- see the "Acrobat blank-tree
   investigation" project notes for full detail). The fix round-tripped
   fine through nanoPRC's own reader and reported the correct triangle
   COUNT via the independent ground-truth reader, yet visibly corrupted
   real geometry -- proof that count-only and order-agnostic-nearest-
   vertex checks are insufficient to validate this code path, and that
   any future fix attempt needs a tool that prints actual decoded
   positions in a form a human can check against the known input by hand
   BEFORE trusting a round-trip "looks successful".

   LIMITATIONS: Currently READ-ONLY diagnostic -- prints positions for
   manual inspection, does not itself assert correctness (no automated
   pass/fail). A natural next step is to hand-verify the fan mesh's
   expected decoded positions once (they are exactly computable: the
   center and 4 rim points never move, only get quantized to the encode
   tolerance), then hardcode those expected values as assertions so this
   becomes a real regression test -- not done yet, since the point of
   this tool's first use is to establish ground truth by hand, not to
   assume it. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_write_compress_tess.h"
#include "prc_write_common.h"

int main(void)
{
    prc_context *ctx;
    /* 5 vertices: center (0) + 4 rim points at simple axis coordinates. */
    double positions[5 * 3] = {
        0.0, 0.0, 0.0,   /* 0: center */
        1.0, 0.0, 0.0,   /* 1: +X rim */
        0.0, 1.0, 0.0,   /* 2: +Y rim */
       -1.0, 0.0, 0.0,   /* 3: -X rim */
        0.0,-1.0, 0.0    /* 4: -Y rim */
    };
    /* 4 triangles, all winding center->rim->rim (CCW looking down +Z):
       seed is triangle 0 (0,1,2); its neighbors across edges (1,2)... no,
       across (0,1) and (0,2) are triangles 3 and 1 respectively -- both
       unvisited when triangle 0 is emitted, so edge_status_array[0] must
       come out as 3 (both bits set). */
    uint32_t tris[4 * 3] = {
        0, 1, 2,   /* triangle 0: seed */
        0, 2, 3,   /* triangle 1: shares edge (0,2) with triangle 0 */
        0, 3, 4,   /* triangle 2: shares edge (0,3) with triangle 1 */
        0, 4, 1    /* triangle 3: shares edge (0,4) with triangle 2, and (0,1) with triangle 0 */
    };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_compressed *parsed = NULL;
    uint32_t k;
    int code;

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    printf("=== Input mesh ===\n");
    for (k = 0; k < 5; k++)
        printf("  V%u = (%.3f, %.3f, %.3f)\n", k, positions[k*3+0], positions[k*3+1], positions[k*3+2]);
    for (k = 0; k < 4; k++)
        printf("  T%u = (V%u, V%u, V%u)\n", k, tris[k*3+0], tris[k*3+1], tris[k*3+2]);

    code = prc_encode_preprocess(ctx, positions, 5, tris, 4, prc_write_tol_absolute(1e-4), &mesh);
    if (code != 0) { printf("preprocess failed: %d\n", code); return 1; }

    code = prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL);
    if (code != 0) { printf("traversal failed: %d\n", code); return 1; }

    printf("\n=== Traversal result ===\n");
    printf("  edge_status_array_size=%u: [", res.edge_status_array_size);
    for (k = 0; k < res.edge_status_array_size; k++)
        printf("%u ", res.edge_status_array[k]);
    printf("]\n");
    if (res.edge_status_array_size > 0 && res.edge_status_array[0] != 3)
    {
        printf("  *** WARNING: expected edge_status_array[0] == 3 (both children grown from\n");
        printf("      the seed) for this mesh -- got %u instead. The fan topology comment\n", res.edge_status_array[0]);
        printf("      above may not hold for the actual adjacency this preprocessing computed;\n");
        printf("      re-check before trusting this run exercises both gate branches. ***\n");
    }
    printf("  num_decoded_points=%u\n", res.num_decoded_points);

    code = prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev);
    if (code != 0) { printf("normals_c1 failed: %d\n", code); return 1; }

    memset(&w, 0, sizeof(w));
    if (prc_bitwrite_init(ctx, &w, 512) != 0) { printf("bitwrite_init failed\n"); return 1; }
    code = prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL);
    if (code != 0) { printf("write failed: %d\n", code); return 1; }
    if (prc_bitwrite_flush(ctx, &w) != 0) { printf("flush failed\n"); return 1; }

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code != 0)
    {
        printf("decode failed: %d\n", code);
        prc_api_print_error_stack(ctx);
        return 1;
    }

    printf("\n=== Decoded result ===\n");
    printf("  num_vertices=%d num_triangles=%d\n",
        parsed->num_vertices_prc_compressed_3d, parsed->num_triangle_indices_prc_compressed_3d / 3);
    for (k = 0; k < (uint32_t)parsed->num_vertices_prc_compressed_3d; k++)
        printf("  decoded V%u = (%.6f, %.6f, %.6f)\n", k,
            parsed->vertices_prc_compressed_3d[k*3+0],
            parsed->vertices_prc_compressed_3d[k*3+1],
            parsed->vertices_prc_compressed_3d[k*3+2]);
    for (k = 0; k < (uint32_t)parsed->num_triangle_indices_prc_compressed_3d / 3; k++)
    {
        uint32_t i0 = parsed->triangle_indices_prc_compressed_3d[k*3+0];
        uint32_t i1 = parsed->triangle_indices_prc_compressed_3d[k*3+1];
        uint32_t i2 = parsed->triangle_indices_prc_compressed_3d[k*3+2];
        printf("  decoded T%u = (decodedV%u, decodedV%u, decodedV%u) = "
            "(%.3f,%.3f,%.3f) (%.3f,%.3f,%.3f) (%.3f,%.3f,%.3f)\n",
            k, i0, i1, i2,
            parsed->vertices_prc_compressed_3d[i0*3+0], parsed->vertices_prc_compressed_3d[i0*3+1], parsed->vertices_prc_compressed_3d[i0*3+2],
            parsed->vertices_prc_compressed_3d[i1*3+0], parsed->vertices_prc_compressed_3d[i1*3+1], parsed->vertices_prc_compressed_3d[i1*3+2],
            parsed->vertices_prc_compressed_3d[i2*3+0], parsed->vertices_prc_compressed_3d[i2*3+1], parsed->vertices_prc_compressed_3d[i2*3+2]);
    }

    printf("\n=== Manual check ===\n");
    printf("  Expect (up to winding/relabeling): each decoded triangle's 3 vertices\n");
    printf("  should numerically match one of T0..T3's real input coordinates above,\n");
    printf("  IN THE SAME CYCLIC ORDER the input listed them (center, rim, rim) --\n");
    printf("  not just be A PERMUTATION of the right 3 points. A cyclic reversal\n");
    printf("  (e.g. decoded (rim,rim,center) when input was (center,rim,rim), same\n");
    printf("  3 points but reversed winding) indicates a frame/gate-direction bug\n");
    printf("  exactly like the one found and reverted earlier today.\n");

    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    prc_api_release_context(ctx);
    return 0;
}
