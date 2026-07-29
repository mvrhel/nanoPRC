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

/* Phase 1b, Session 2 gate test: prc_encode_traversal round trip. The encoder
   output is fed into the real, unmodified prc_decode_compressed_tess and the
   decoded geometry must reproduce the input unit-square mesh (2 triangles,
   4 unique vertices, shared diagonal not duplicated). */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_write_compress_tess.h"
#include "prc_decode_compressed_tess.h"

static double
dist3(const double *a, const double *b)
{
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* Generic encode -> real-decode round trip: every decoded triangle must map
   (by nearest-position corner matching, within 2 * tolerance) onto a distinct
   input triangle, and the decoded vertex count must equal the deduplicated
   input vertex count (no shared vertex ever duplicated). */
static void
check_roundtrip(prc_context *ctx, const double *positions, uint32_t npos,
    const uint32_t *tris, uint32_t ntris, uint32_t min_references)
{
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed data;
    uint8_t *norm_rev;
    uint8_t *tri_used;
    uint32_t k, c, i;
    int code;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, npos, tris, ntris,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT(res.point_reference_array_size >= min_references);

    memset(&data, 0, sizeof(data));
    data.tolerance = mesh.tolerance_mm;
    data.origin_array.x = res.origin[0];
    data.origin_array.y = res.origin[1];
    data.origin_array.z = res.origin[2];
    data.point_array = res.point_array;
    data.point_array_size = res.point_array_size;
    data.edge_status_array = res.edge_status_array;
    data.edge_status_array_size = res.edge_status_array_size;
    data.triangle_face_array = (uint32_t *)res.triangle_face_array;
    data.triangle_face_array_size = res.triangle_face_array_size;
    data.has_faces = 0;
    data.reference_array_size = res.points_is_reference_array_size;
    data.points_is_reference_array = res.points_is_reference_array;
    data.point_reference_array = res.point_reference_array;
    data.point_reference_array_size = res.point_reference_array_size;
    data.must_recalculate_normals = 1;
    norm_rev = (uint8_t *)prc_calloc(ctx, mesh.num_triangles, sizeof(uint8_t));
    PRC_ASSERT_NOT_NULL(norm_rev);
    data.normal_is_reversed = norm_rev;

    code = prc_decode_compressed_tess(ctx, &data, 0);
    if (code < 0)
        prc_api_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);

    PRC_ASSERT_EQ((uint32_t)data.num_triangle_indices_prc_compressed_3d / 3, mesh.num_triangles);
    PRC_ASSERT_EQ((uint32_t)data.num_vertices_prc_compressed_3d, mesh.num_positions);

    tri_used = (uint8_t *)prc_calloc(ctx, mesh.num_triangles, sizeof(uint8_t));
    PRC_ASSERT_NOT_NULL(tri_used);

    for (k = 0; k < mesh.num_triangles; k++)
    {
        uint32_t mapped[3];
        uint32_t found = 0;

        for (c = 0; c < 3; c++)
        {
            uint32_t idx = data.triangle_indices_prc_compressed_3d[(size_t)k * 3 + c];
            const double *p;
            uint32_t best = 0;
            double best_d = 0.0;

            PRC_ASSERT(idx < mesh.num_positions);
            p = &data.vertices_prc_compressed_3d[(size_t)idx * 3];
            for (i = 0; i < mesh.num_positions; i++)
            {
                double d = dist3(p, &mesh.positions[(size_t)i * 3]);
                if (i == 0 || d < best_d)
                {
                    best = i;
                    best_d = d;
                }
            }
            PRC_ASSERT(best_d <= 2.0 * mesh.tolerance_mm);
            mapped[c] = best;
        }

        for (i = 0; i < mesh.num_triangles && !found; i++)
        {
            const uint32_t *t = &mesh.tri_indices[(size_t)i * 3];

            if (tri_used[i])
                continue;
            if ((mapped[0] == t[0] || mapped[0] == t[1] || mapped[0] == t[2]) &&
                (mapped[1] == t[0] || mapped[1] == t[1] || mapped[1] == t[2]) &&
                (mapped[2] == t[0] || mapped[2] == t[1] || mapped[2] == t[2]) &&
                mapped[0] != mapped[1] && mapped[1] != mapped[2] && mapped[0] != mapped[2])
            {
                tri_used[i] = 1;
                found = 1;
            }
        }
        PRC_ASSERT(found);
    }

    prc_free(ctx, tri_used);
    prc_free(ctx, data.triangle_indices_prc_compressed_3d);
    prc_free(ctx, data.normal_indices_prc_compressed_3d);
    prc_free(ctx, data.vertices_prc_compressed_3d);
    prc_free(ctx, data.normals_prc_compressed_3d);
    if (data.point_colors_prc_compressed_3d != NULL)
        prc_free(ctx, data.point_colors_prc_compressed_3d);
    if (data.triangle_styles != NULL)
        prc_free(ctx, data.triangle_styles);
    prc_free(ctx, norm_rev);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
}

/* Two disjoint non-planar 5x5 grids: interior vertices are shared by up to
   six triangles, forcing reference continuations and multi-chain restarts
   (reference-based chain starts), and the second component exercises the
   outer unvisited-triangle scan. */
static void
test_grid_roundtrip(prc_context *ctx)
{
#define GRID_N 5
    enum { grid_pts = (GRID_N + 1) * (GRID_N + 1) };
    double positions[2 * grid_pts * 3];
    uint32_t tris[2 * GRID_N * GRID_N * 2 * 3];
    uint32_t comp, gx, gy, n = 0;

    for (comp = 0; comp < 2; comp++)
    {
        double x_off = comp == 0 ? 0.0 : 10.0;
        uint32_t base = comp * grid_pts;

        for (gy = 0; gy <= GRID_N; gy++)
        {
            for (gx = 0; gx <= GRID_N; gx++)
            {
                double *p = &positions[(size_t)(base + gy * (GRID_N + 1) + gx) * 3];
                p[0] = x_off + (double)gx;
                p[1] = (double)gy;
                p[2] = 0.13 * (double)gx * (double)gx + 0.27 * (double)gy;
            }
        }
        for (gy = 0; gy < GRID_N; gy++)
        {
            for (gx = 0; gx < GRID_N; gx++)
            {
                uint32_t v00 = base + gy * (GRID_N + 1) + gx;
                uint32_t v10 = v00 + 1;
                uint32_t v01 = v00 + (GRID_N + 1);
                uint32_t v11 = v01 + 1;

                tris[n++] = v00;
                tris[n++] = v10;
                tris[n++] = v11;
                tris[n++] = v00;
                tris[n++] = v11;
                tris[n++] = v01;
            }
        }
    }

    check_roundtrip(ctx, positions, 2 * grid_pts, tris, n / 3, 1);
#undef GRID_N
}

/* Triangles connected only through vertices (or through a chain-start's
   never-grown base edge, as with the first two), so after the first 0-ref
   chain every following triangle is its own chain start hitting a specific
   empty-stack branch: 2-ref (all three non-ref positions), 1-ref (all three
   ref positions), 3-ref, including the decoder's index0 > index1 swap. */
static void
test_chain_start_variants(prc_context *ctx)
{
    double positions[12 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 1.0, 0.5,
        0.0, 2.0, 0.3,
        1.0, 2.0, 0.0,
        2.0, 0.0, 0.7,
        2.0, 1.0, 0.0,
        3.0, 0.0, 0.0,
        3.0, 1.0, 0.4,
        4.0, 0.0, 0.0,
        2.0, 2.0, 0.6
    };
    uint32_t tris[8 * 3] = {
        0, 1, 2,   /* chain start, 0 refs */
        1, 0, 3,   /* shares the base edge of the first: 2-ref, non-ref pos 2, swapped */
        2, 4, 5,   /* 1-ref at pos 0 */
        6, 3, 7,   /* 1-ref at pos 1, swapped */
        8, 9, 5,   /* 1-ref at pos 2 */
        10, 0, 5,  /* 2-ref, non-ref pos 0, swapped */
        3, 11, 2,  /* 2-ref, non-ref pos 1 */
        6, 8, 10   /* 3-ref */
    };

    /* This mesh's vertex-only (never edge-only) sharing was originally
       chosen specifically to hit the chain-start variants described
       above, and used to reference 2+1+1+1+2+2+3 = 12 slots doing so.
       Since the non-manifold-vertex fix in prc_encode_preprocess (private-
       vertex-splitting for vertices whose incident triangles form 2+
       disconnected fans -- exactly what "connected only through vertices"
       means for a vertex touched by non-edge-adjacent triangles), most of
       that sharing is no longer preserved as-is: each such vertex gets
       split into private per-fan copies, which is the intended, correct
       behavior for real non-manifold meshes. The actual current count
       (2) is a regression-guard floor, not a design target -- if it
       changes again, re-derive it, don't just bump the number blindly. */
    check_roundtrip(ctx, positions, 12, tris, 8, 2);
}

int
main(void)
{
    double positions[12] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    /* diagonal 0-2 shared between the two triangles */
    uint32_t tris[6] = { 0, 1, 2, 0, 2, 3 };
    prc_context *ctx;
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed data;
    uint8_t *norm_rev;
    uint32_t k, c, i, j;
    uint32_t masks[2] = { 0, 0 };
    int code;

    PRC_TEST_BEGIN("compressed tess traversal round-trip");

    ctx = prc_api_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 4, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_triangles, 2);
    PRC_ASSERT_EQ(mesh.num_positions, 4);

    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_NOT_NULL(res.point_array);
    PRC_ASSERT_NOT_NULL(res.edge_status_array);
    PRC_ASSERT_NOT_NULL(res.points_is_reference_array);
    PRC_ASSERT_EQ(res.edge_status_array_size, 2);
    PRC_ASSERT_EQ(res.triangle_face_array_size, 2);
    /* the shared diagonal must not force duplicate points: 4 emitted points */
    PRC_ASSERT_EQ(res.point_array_size, 12);

    memset(&data, 0, sizeof(data));
    data.tolerance = mesh.tolerance_mm;
    data.origin_array.x = res.origin[0];
    data.origin_array.y = res.origin[1];
    data.origin_array.z = res.origin[2];
    data.point_array = res.point_array;
    data.point_array_size = res.point_array_size;
    data.edge_status_array = res.edge_status_array;
    data.edge_status_array_size = res.edge_status_array_size;
    data.triangle_face_array = (uint32_t *)res.triangle_face_array;
    data.triangle_face_array_size = res.triangle_face_array_size;
    data.face_number = 0;
    data.has_faces = 0;
    data.reference_array_size = res.points_is_reference_array_size;
    data.points_is_reference_array = res.points_is_reference_array;
    data.point_reference_array = res.point_reference_array;
    data.point_reference_array_size = res.point_reference_array_size;
    data.must_recalculate_normals = 1;
    norm_rev = (uint8_t *)prc_calloc(ctx, mesh.num_triangles, sizeof(uint8_t));
    PRC_ASSERT_NOT_NULL(norm_rev);
    data.normal_is_reversed = norm_rev;

    code = prc_decode_compressed_tess(ctx, &data, 0);
    if (code < 0)
        prc_api_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);

    PRC_ASSERT_EQ(data.num_triangle_indices_prc_compressed_3d / 3, 2);
    PRC_ASSERT_EQ(data.num_vertices_prc_compressed_3d, 4);
    PRC_ASSERT_NOT_NULL(data.triangle_indices_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(data.vertices_prc_compressed_3d);

    /* Decoded triangle/vertex order is an encoder-internal decision: match
       every decoded corner to the nearest original position and require the
       two decoded triangles to cover the original vertex sets {0,1,2} and
       {0,2,3} (in either triangle order). */
    for (k = 0; k < 2; k++)
    {
        for (c = 0; c < 3; c++)
        {
            uint32_t idx = data.triangle_indices_prc_compressed_3d[k * 3 + c];
            const double *p;
            uint32_t best = 0;
            double best_d = 0.0;

            PRC_ASSERT(idx < 4);
            p = &data.vertices_prc_compressed_3d[(size_t)idx * 3];
            for (i = 0; i < 4; i++)
            {
                double d = dist3(p, &positions[(size_t)i * 3]);
                if (i == 0 || d < best_d)
                {
                    best = i;
                    best_d = d;
                }
            }
            /* 2.0*tolerance budgets ordinary quantization noise; the additional
               sqrt(3)*PRC_ENCODE_JITTER_TOLERANCE_FACTOR*tolerance term budgets the
               deterministic position jitter prc_encode_preprocess now applies to
               every vertex (see that constant's own comment, prc_write_compress_
               tess.h) -- worst case is the jitter maxing out in all 3 axes at once.
               This check compares against the ORIGINAL, pre-jitter local `positions`
               array (unlike the check earlier in this file at line ~116, which
               compares against mesh.positions -- already jittered by preprocessing --
               and so does not need this extra budget). */
            PRC_ASSERT(best_d <= (2.0 + sqrt(3.0) * PRC_ENCODE_JITTER_TOLERANCE_FACTOR) * mesh.tolerance_mm);
            masks[k] |= 1u << best;
        }
    }
    PRC_ASSERT((masks[0] == 0x7u && masks[1] == 0xDu) ||
               (masks[0] == 0xDu && masks[1] == 0x7u));

    /* No accidental duplicates: shared diagonal vertices appear exactly once,
       so every decoded vertex pair must be well separated. */
    for (i = 0; i < 4; i++)
    {
        for (j = i + 1; j < 4; j++)
        {
            PRC_ASSERT(dist3(&data.vertices_prc_compressed_3d[(size_t)i * 3],
                &data.vertices_prc_compressed_3d[(size_t)j * 3]) > mesh.tolerance_mm);
        }
    }

    prc_free(ctx, data.triangle_indices_prc_compressed_3d);
    prc_free(ctx, data.normal_indices_prc_compressed_3d);
    prc_free(ctx, data.vertices_prc_compressed_3d);
    prc_free(ctx, data.normals_prc_compressed_3d);
    if (data.point_colors_prc_compressed_3d != NULL)
        prc_free(ctx, data.point_colors_prc_compressed_3d);
    if (data.triangle_styles != NULL)
        prc_free(ctx, data.triangle_styles);
    prc_free(ctx, norm_rev);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);

    test_grid_roundtrip(ctx);
    test_chain_start_variants(ctx);

    /* Under PRC_ENABLE_DEBUG_MEMORY this fails if anything above leaked */
    PRC_ASSERT_EQ(prc_api_release_context(ctx), 0);

    PRC_TEST_END;
}
