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

/* Phase 1b consolidated gate test: Step A preprocessing (bbox, tolerance
   resolution, quantized vertex dedup, degenerate removal, edge adjacency,
   connected components), full-bitstream C1 and C2 round trips through the
   real, unmodified prc_parse_tess_3d_compressed, chain-restart accounting,
   numeric-stability stress (1M-triangle grid, large world offsets), the
   vertex-analysis stub, and absolute/relative tolerance-mode equivalence. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_write_compress_tess.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"

/* Same leak-check pattern as test_bitwrite.c: baseline the number of live
   debug-tracked allocations, assert the test returned to it. No-op unless
   PRC_ENABLE_DEBUG_MEMORY is configured. */
#if PRC_DEBUG_MEMORY
static size_t
count_outstanding(prc_context *ctx)
{
    size_t k, count = 0;
    for (k = 0; k < ctx->current_memory_index; k++)
        if (!ctx->debug_memory[k].is_free)
            count++;
    return count;
}
#define LEAK_CHECK_BEGIN(ctx) size_t leak_baseline = count_outstanding(ctx)
#define LEAK_CHECK_END(ctx) PRC_ASSERT_EQ(count_outstanding(ctx), leak_baseline)
#else
#define LEAK_CHECK_BEGIN(ctx) ((void)0)
#define LEAK_CHECK_END(ctx) ((void)0)
#endif

static void
test_single_triangle(prc_context *ctx)
{
    double positions[] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[] = { 0, 1, 2 };
    prc_encode_mesh mesh;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 3, tris, 1,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 3);
    PRC_ASSERT_EQ(mesh.num_triangles, 1);
    PRC_ASSERT_EQ(mesh.num_edges, 3);
    PRC_ASSERT_EQ(mesh.num_components, 1);
    PRC_ASSERT_NOT_NULL(mesh.edges);
    PRC_ASSERT_EQ(mesh.edges[0].tri1, -1);
    PRC_ASSERT_EQ(mesh.edges[1].tri1, -1);
    PRC_ASSERT_EQ(mesh.edges[2].tri1, -1);
    PRC_ASSERT(mesh.tolerance_mm == 1e-4);
    PRC_ASSERT(mesh.bbox[0] == 0.0 && mesh.bbox[3] == 1.0);
    prc_encode_preprocess_free(ctx, &mesh);
    PRC_ASSERT_NULL(mesh.positions);
    prc_encode_preprocess_free(ctx, &mesh);
}

static void
test_quad_shared_edge(prc_context *ctx)
{
    double positions[] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    /* diagonal 0-2 shared between the two triangles */
    uint32_t tris[] = { 0, 1, 2, 0, 2, 3 };
    prc_encode_mesh mesh;
    uint32_t i, shared_found = 0;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 4, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 4);
    PRC_ASSERT_EQ(mesh.num_triangles, 2);
    PRC_ASSERT_EQ(mesh.num_edges, 5);
    PRC_ASSERT_EQ(mesh.num_components, 1);
    for (i = 0; i < mesh.num_edges; i++)
    {
        if (mesh.edges[i].v0 == 0 && mesh.edges[i].v1 == 2)
        {
            shared_found = 1;
            PRC_ASSERT(mesh.edges[i].tri0 != -1);
            PRC_ASSERT(mesh.edges[i].tri1 != -1);
            PRC_ASSERT((mesh.edges[i].tri0 == 0 && mesh.edges[i].tri1 == 1) ||
                       (mesh.edges[i].tri0 == 1 && mesh.edges[i].tri1 == 0));
        }
        else
        {
            PRC_ASSERT_EQ(mesh.edges[i].tri1, -1);
        }
    }
    PRC_ASSERT_EQ(shared_found, 1);
    prc_encode_preprocess_free(ctx, &mesh);
}

static void
test_disjoint_triangles(prc_context *ctx)
{
    double positions[] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        10.0, 0.0, 0.0,
        11.0, 0.0, 0.0,
        10.0, 1.0, 0.0
    };
    uint32_t tris[] = { 0, 1, 2, 3, 4, 5 };
    prc_encode_mesh mesh;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 6, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 6);
    PRC_ASSERT_EQ(mesh.num_triangles, 2);
    PRC_ASSERT_EQ(mesh.num_edges, 6);
    PRC_ASSERT_EQ(mesh.num_components, 2);
    PRC_ASSERT(mesh.tri_component[0] != mesh.tri_component[1]);
    prc_encode_preprocess_free(ctx, &mesh);
}

static void
test_degenerate_weld(prc_context *ctx)
{
    /* vertices 1 and 2 differ by 1e-8 << 1e-4 tolerance: they weld to one
       deduplicated slot, making the triangle degenerate */
    double positions[] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0 + 1e-8, 0.0, 0.0
    };
    uint32_t tris[] = { 0, 1, 2 };
    prc_encode_mesh mesh;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 3, tris, 1,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 2);
    PRC_ASSERT_EQ(mesh.num_triangles, 0);
    PRC_ASSERT_EQ(mesh.num_edges, 0);
    PRC_ASSERT_EQ(mesh.num_components, 0);
    PRC_ASSERT_NULL(mesh.tri_indices);
    prc_encode_preprocess_free(ctx, &mesh);
}

/* Two vertices exactly tolerance/2 apart must still weld. The dedup key is
   llround(coord / tol), so a pair anchored at a cell edge (0 and tol/2)
   would weld or split purely on llround's half-way rounding direction;
   centering the pair at +-tol/4 around one cell center keeps both keys
   deterministically equal while the separation stays exactly tol/2 (both
   offsets are power-of-two fractions of tol, so the arithmetic is exact). */
static void
test_half_tolerance_weld(prc_context *ctx)
{
    double positions[3 * 3] = {
         0.0,     1.0, 0.0,
        -2.5e-4,  0.0, 0.0,
         2.5e-4,  0.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    prc_encode_mesh mesh;

    printf("  sub-case: half-tolerance boundary weld\n");

    /* This test is deliberately about the dedup pass's own quantization-key boundary
       arithmetic, not the position jitter mitigation (prc_write_compress_tess.h's
       PRC_ENCODE_JITTER_TOLERANCE_FACTOR) -- jitter is now applied BEFORE this same
       quantization step (by design: it needs to influence dedup decisions, not just move
       already-decided positions), which would otherwise perturb this test's carefully
       exact-arithmetic offsets and make the weld outcome depend on jitter's own hash rather
       than the boundary logic under test. Disable it for just this sub-case. */
#ifdef _WIN32
    _putenv_s("PRC_DIAG_DISABLE_POSITION_JITTER", "1");
#else
    setenv("PRC_DIAG_DISABLE_POSITION_JITTER", "1", 1);
#endif
    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 3, tris, 1,
        prc_write_tol_absolute(1e-3), &mesh), 0);
#ifdef _WIN32
    _putenv_s("PRC_DIAG_DISABLE_POSITION_JITTER", "");
#else
    unsetenv("PRC_DIAG_DISABLE_POSITION_JITTER");
#endif
    PRC_ASSERT(positions[6] - positions[3] == 0.5 * mesh.tolerance_mm);
    PRC_ASSERT_EQ(mesh.num_positions, 2);
    PRC_ASSERT_EQ(mesh.num_triangles, 0);
    PRC_ASSERT_EQ(mesh.num_edges, 0);
    PRC_ASSERT_EQ(mesh.num_components, 0);
    PRC_ASSERT_NULL(mesh.tri_indices);
    prc_encode_preprocess_free(ctx, &mesh);
}

/* ABSOLUTE and RELATIVE tolerances resolving to the same effective value must
   be genuinely interchangeable, not merely close: identical dedup, identical
   surviving triangles, identical remapped indices. */
static void
test_tolerance_mode_equivalence(prc_context *ctx)
{
    /* vertex 4 sits 1e-8 from vertex 1 and welds away; the three triangles
       all survive, so both the position and index outputs are nontrivial */
    double positions[6 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0,
        1.0 + 1e-8, 0.0, 0.0,
        0.5, 2.0, 0.3
    };
    uint32_t tris[3 * 3] = { 0, 1, 2, 0, 2, 3, 4, 2, 5 };
    const double v = 1e-4;
    prc_encode_mesh mesh_a, mesh_b;
    double dx, dy, dz, diagonal;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: absolute vs relative tolerance equivalence\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 6, tris, 3,
        prc_write_tol_absolute(v), &mesh_a), 0);
    PRC_ASSERT_EQ(mesh_a.num_positions, 5);
    PRC_ASSERT_EQ(mesh_a.num_triangles, 3);

    /* same diagonal formula prc_encode_preprocess itself resolves relative
       tolerances against, so (v / D) * D re-derives v to within one
       floating-point rounding */
    dx = mesh_a.bbox[3] - mesh_a.bbox[0];
    dy = mesh_a.bbox[4] - mesh_a.bbox[1];
    dz = mesh_a.bbox[5] - mesh_a.bbox[2];
    diagonal = sqrt(dx * dx + dy * dy + dz * dz);

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 6, tris, 3,
        prc_write_tol_relative(v / diagonal), &mesh_b), 0);

    PRC_ASSERT(fabs(mesh_a.tolerance_mm - mesh_b.tolerance_mm) < 1e-12);
    PRC_ASSERT_EQ(mesh_b.num_positions, mesh_a.num_positions);
    PRC_ASSERT_EQ(mesh_b.num_triangles, mesh_a.num_triangles);
    PRC_ASSERT_NOT_NULL(mesh_a.tri_indices);
    PRC_ASSERT_NOT_NULL(mesh_b.tri_indices);
    PRC_ASSERT_EQ(memcmp(mesh_a.tri_indices, mesh_b.tri_indices,
        (size_t)mesh_a.num_triangles * 3 * sizeof(uint32_t)), 0);

    prc_encode_preprocess_free(ctx, &mesh_a);
    prc_encode_preprocess_free(ctx, &mesh_b);
    LEAK_CHECK_END(ctx);
}

static double
dist3(const double *a, const double *b)
{
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];

    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double
dot3(const double *a, const double *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static uint32_t
nearest_input_vertex(const double *p, const double *positions, uint32_t npos,
    double tolerance)
{
    uint32_t i, best = 0;
    double best_d = 0.0;

    for (i = 0; i < npos; i++)
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
       tess.h) -- worst case is the jitter maxing out in all 3 axes at once. */
    PRC_ASSERT(best_d <= (2.0 + sqrt(3.0) * PRC_ENCODE_JITTER_TOLERANCE_FACTOR) * tolerance);
    return best;
}

/* Frees everything prc_parse_tess_3d_compressed allocated: the same field
   list as prc_release.c's (static) prc_release_tess_3d_compressed, minus the
   texture/behavior sub-objects this encoder never emits. */
static void
free_parsed_tess(prc_context *ctx, prc_tess_3d_compressed *d)
{
    if (d == NULL)
        return;
    if (d->point_array != NULL)
        prc_free(ctx, d->point_array);
    if (d->edge_status_array != NULL)
        prc_free(ctx, d->edge_status_array);
    if (d->triangle_face_array != NULL)
        prc_free(ctx, d->triangle_face_array);
    if (d->points_is_reference_array != NULL)
        prc_free(ctx, d->points_is_reference_array);
    if (d->point_reference_array != NULL)
        prc_free(ctx, d->point_reference_array);
    if (d->normal_is_reversed != NULL)
        prc_free(ctx, d->normal_is_reversed);
    if (d->normal_binary_data != NULL)
        prc_free(ctx, d->normal_binary_data);
    if (d->normal_angle_array != NULL)
        prc_free(ctx, d->normal_angle_array);
    if (d->normal_angle_array_scaled != NULL)
        prc_free(ctx, d->normal_angle_array_scaled);
    if (d->is_face_planar != NULL)
        prc_free(ctx, d->is_face_planar);
    if (d->is_point_color_on_face != NULL)
        prc_free(ctx, d->is_point_color_on_face);
    if (d->point_color_array != NULL)
        prc_free(ctx, d->point_color_array);
    if (d->decoded_point_color_array != NULL)
        prc_free(ctx, d->decoded_point_color_array);
    if (d->is_multiple_line_attribute_on_face != NULL)
        prc_free(ctx, d->is_multiple_line_attribute_on_face);
    if (d->line_attribute_array != NULL)
        prc_free(ctx, d->line_attribute_array);
    if (d->face_has_texture != NULL)
        prc_free(ctx, d->face_has_texture);
    if (d->uv_coordinates_3d != NULL)
        prc_free(ctx, d->uv_coordinates_3d);
    if (d->triangle_indices_prc_compressed_3d != NULL)
        prc_free(ctx, d->triangle_indices_prc_compressed_3d);
    if (d->point_colors_prc_compressed_3d != NULL)
        prc_free(ctx, d->point_colors_prc_compressed_3d);
    if (d->normal_indices_prc_compressed_3d != NULL)
        prc_free(ctx, d->normal_indices_prc_compressed_3d);
    if (d->vertices_prc_compressed_3d != NULL)
        prc_free(ctx, d->vertices_prc_compressed_3d);
    if (d->normals_prc_compressed_3d != NULL)
        prc_free(ctx, d->normals_prc_compressed_3d);
    if (d->triangle_styles != NULL)
        prc_free(ctx, d->triangle_styles);
    if (d->edge_vertices != NULL)
        prc_free(ctx, d->edge_vertices);
    if (d->edge_indices != NULL)
        prc_free(ctx, d->edge_indices);
    prc_free(ctx, d);
}

/* Smallest possible full-bitstream round trip: unlike test_single_triangle
   (Step A only), this drives the complete C1 pipeline through the real
   parser and checks the decoded geometry. */
static void
test_single_triangle_roundtrip(prc_context *ctx)
{
    double positions[3 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t c, mask = 0;
    int code;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: single-triangle C1 round trip\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 3, tris, 1,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev), 0);
    PRC_ASSERT_NOT_NULL(rev);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 1);
    PRC_ASSERT_EQ(parsed->num_vertices_prc_compressed_3d, 3);

    for (c = 0; c < 3; c++)
    {
        uint32_t idx = parsed->triangle_indices_prc_compressed_3d[c];

        PRC_ASSERT(idx < 3);
        mask |= 1u << nearest_input_vertex(
            &parsed->vertices_prc_compressed_3d[(size_t)idx * 3],
            positions, 3, mesh.tolerance_mm);
    }
    PRC_ASSERT_EQ(mask, 0x7u);

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Shared-edge quad through the full C1 pipeline. Beyond the decode checks,
   the RAW encoder output must show the traversal grew across the shared
   diagonal (a nonzero edge-status bit) instead of restarting a second chain. */
static void
test_quad_roundtrip(prc_context *ctx)
{
    double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[2 * 3] = { 0, 1, 2, 0, 2, 3 };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t k, c;
    uint32_t masks[2] = { 0, 0 };
    int code;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: shared-edge quad C1 round trip\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 4, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ(res.edge_status_array_size, 2);
    PRC_ASSERT(res.edge_status_array[0] != 0 || res.edge_status_array[1] != 0);
    PRC_ASSERT_EQ(res.num_decoded_points, 4);

    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev), 0);
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 2);
    /* the shared diagonal's vertices are emitted once, not duplicated */
    PRC_ASSERT_EQ(parsed->num_vertices_prc_compressed_3d, 4);

    for (k = 0; k < 2; k++)
    {
        for (c = 0; c < 3; c++)
        {
            uint32_t idx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];

            PRC_ASSERT(idx < 4);
            masks[k] |= 1u << nearest_input_vertex(
                &parsed->vertices_prc_compressed_3d[(size_t)idx * 3],
                positions, 4, mesh.tolerance_mm);
        }
    }
    PRC_ASSERT((masks[0] == 0x7u && masks[1] == 0xDu) ||
               (masks[0] == 0xDu && masks[1] == 0x7u));

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Two components sharing nothing: each triangle must be its own chain start.
   A chain start consumes 3 reference-bit slots and each grow step 1, so
   slots == num_tris + 2 * num_chains and the derived chain count must be 2. */
static void
test_disjoint_roundtrip(prc_context *ctx)
{
    double positions[6 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        10.0, 0.0, 0.0,
        11.0, 0.0, 0.0,
        10.0, 1.0, 0.0
    };
    uint32_t tris[2 * 3] = { 0, 1, 2, 3, 4, 5 };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t k, c;
    uint32_t masks[2] = { 0, 0 };
    int code;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: disjoint-pair C1 round trip (chain restart)\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 6, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ((res.points_is_reference_array_size - mesh.num_triangles) % 2, 0);
    PRC_ASSERT_EQ((res.points_is_reference_array_size - mesh.num_triangles) / 2, 2);
    PRC_ASSERT_EQ(res.num_decoded_points, 6);

    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev), 0);
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 2);
    PRC_ASSERT_EQ(parsed->num_vertices_prc_compressed_3d, 6);

    for (k = 0; k < 2; k++)
    {
        for (c = 0; c < 3; c++)
        {
            uint32_t idx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];

            PRC_ASSERT(idx < 6);
            masks[k] |= 1u << nearest_input_vertex(
                &parsed->vertices_prc_compressed_3d[(size_t)idx * 3],
                positions, 6, mesh.tolerance_mm);
        }
    }
    PRC_ASSERT((masks[0] == 0x07u && masks[1] == 0x38u) ||
               (masks[0] == 0x38u && masks[1] == 0x07u));

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Session 3 gate: cube through the complete C1 pipeline and the REAL parser.
   preprocess -> traversal -> C1 reversal bits -> full bitstream -> flush ->
   prc_parse_tess_3d_compressed (which itself runs the real decoder). */
static void
test_cube_c1_roundtrip(prc_context *ctx)
{
    double positions[8 * 3] = {
        -0.5, -0.5, -0.5,
         0.5, -0.5, -0.5,
         0.5,  0.5, -0.5,
        -0.5,  0.5, -0.5,
        -0.5, -0.5,  0.5,
         0.5, -0.5,  0.5,
         0.5,  0.5,  0.5,
        -0.5,  0.5,  0.5
    };
    uint32_t tris[12 * 3] = {
        0, 2, 1,   0, 3, 2,   /* bottom */
        4, 5, 6,   4, 6, 7,   /* top */
        0, 1, 5,   0, 5, 4,   /* front */
        1, 2, 6,   1, 6, 5,   /* right */
        2, 3, 7,   2, 7, 6,   /* back */
        3, 0, 4,   3, 4, 7    /* left */
    };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t k, c, i;
    int code;

    printf("  sub-case: cube C1 round trip\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 8, tris, 12,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 8);
    PRC_ASSERT_EQ(mesh.num_triangles, 12);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_NOT_NULL(res.triangle_point_indices);
    PRC_ASSERT_NOT_NULL(res.decoded_positions);
    PRC_ASSERT_EQ(res.num_decoded_points, 8);

    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev), 0);
    PRC_ASSERT_NOT_NULL(rev);
    for (k = 0; k < 12; k++)
        PRC_ASSERT_EQ(rev[k], 0);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 12);
    PRC_ASSERT_EQ(parsed->num_vertices_prc_compressed_3d, 8);
    PRC_ASSERT_NOT_NULL(parsed->vertices_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(parsed->triangle_indices_prc_compressed_3d);

    for (k = 0; k < 12; k++)
    {
        for (c = 0; c < 3; c++)
        {
            uint32_t idx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];
            const double *p;
            double best_d = 0.0;

            PRC_ASSERT(idx < 8);
            p = &parsed->vertices_prc_compressed_3d[(size_t)idx * 3];
            for (i = 0; i < 8; i++)
            {
                double d = dist3(p, &positions[(size_t)i * 3]);

                if (i == 0 || d < best_d)
                    best_d = d;
            }
            /* See nearest_input_vertex's own comment above for why this budgets
               the position jitter prc_encode_preprocess now applies, on top of
               ordinary quantization noise -- this check compares against the
               ORIGINAL, pre-jitter local `positions` array. */
            PRC_ASSERT(best_d <= (2.0 + sqrt(3.0) * PRC_ENCODE_JITTER_TOLERANCE_FACTOR) * mesh.tolerance_mm);
        }
    }

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
}

/* Cube through the C2 supplied-normals pipeline: per-corner outward face
   normals, must_recalculate_normals = 0, round-tripped through the real
   parser. Decoded normals are held to the 10-bit angle quantization itself:
   theta and phi each round to the nearest (PI/2)/1023 step, so the combined
   worst case approaches one step per angle; two steps of slack absorbs both
   roundings landing in the same direction while staying ~3000x tighter than
   an ad-hoc 0.99 dot-product bound.
   The triangulation is deliberately NOT the C1 cube's: C2 has no
   per-triangle reversal bits, so prc_encode_normals_c2 refuses (by design)
   supplied normals that oppose the decoder's treated orientation of any
   GROWING triangle -- and treated orientation is set by the decoder's grow
   mechanics (grown corners decode as (min_index, max_index, new)), not by
   the input winding, so it inevitably flips somewhere on a closed surface.
   Orientation-flipped triangles are only legal as traversal leaves; this
   inward-wound arrangement (bottom-face pair ordered so the chain starts on
   {0,2,3}) was verified against the deterministic traversal to keep every
   flipped triangle a leaf while the supplied/asserted normals stay the true
   outward face normals. */
static void
test_cube_c2_roundtrip(prc_context *ctx)
{
    double positions[8 * 3] = {
        -0.5, -0.5, -0.5,
         0.5, -0.5, -0.5,
         0.5,  0.5, -0.5,
        -0.5,  0.5, -0.5,
        -0.5, -0.5,  0.5,
         0.5, -0.5,  0.5,
         0.5,  0.5,  0.5,
        -0.5,  0.5,  0.5
    };
    uint32_t tris[12 * 3] = {
        0, 2, 3,   0, 1, 2,   /* bottom */
        4, 6, 5,   4, 7, 6,   /* top */
        0, 5, 1,   0, 4, 5,   /* front */
        1, 6, 2,   1, 5, 6,   /* right */
        2, 7, 3,   2, 6, 7,   /* back */
        3, 4, 0,   3, 7, 4    /* left */
    };
    /* one outward axis-aligned normal per face, faces in the tris[] order
       above (triangles 2f and 2f + 1 belong to face f) */
    static const double face_normals[6][3] = {
        {  0.0,  0.0, -1.0 },
        {  0.0,  0.0,  1.0 },
        {  0.0, -1.0,  0.0 },
        {  1.0,  0.0,  0.0 },
        {  0.0,  1.0,  0.0 },
        { -1.0,  0.0,  0.0 }
    };
    double corner_normals[12 * 9];
    const double angle_step = PRC_HALF_PI / 1023.0;
    const double dot_min = cos(2.0 * angle_step);
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    int32_t *angles = NULL;
    uint8_t *bin = NULL;
    uint32_t acount = 0, bsize = 0;
    prc_bit_write_state w;
    prc_bit_state r;
    uint8_t tri_used[12];
    uint32_t k, c, i;
    int code;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: cube C2 supplied-normals round trip\n");

    for (k = 0; k < 12; k++)
        for (c = 0; c < 3; c++)
            memcpy(&corner_normals[(size_t)k * 9 + (size_t)c * 3],
                face_normals[k / 2], 3 * sizeof(double));
    memset(tri_used, 0, sizeof(tri_used));

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 8, tris, 12,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 8);
    PRC_ASSERT_EQ(mesh.num_triangles, 12);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);

    /* NULL planarity candidate / 0 faces: this test covers the ordinary
       per-vertex path. The planar-face path is exercised separately by
       tests/internal/planar_roundtrip.c. */
    code = prc_encode_normals_c2(ctx, &mesh, &res, corner_normals,
        NULL, 0, NULL,
        &angles, &acount, &bin, &bsize);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(angles);
    PRC_ASSERT_NOT_NULL(bin);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        NULL, 0.0, angles, acount, bin, bsize, 0, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 12);
    PRC_ASSERT_EQ(parsed->num_vertices_prc_compressed_3d, 8);
    PRC_ASSERT_NOT_NULL(parsed->normals_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(parsed->normal_indices_prc_compressed_3d);

    for (k = 0; k < 12; k++)
    {
        uint32_t mapped[3];
        uint32_t input_tri = 12;

        for (c = 0; c < 3; c++)
        {
            uint32_t idx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];

            PRC_ASSERT(idx < 8);
            mapped[c] = nearest_input_vertex(
                &parsed->vertices_prc_compressed_3d[(size_t)idx * 3],
                positions, 8, mesh.tolerance_mm);
        }

        for (i = 0; i < 12 && input_tri == 12; i++)
        {
            const uint32_t *t = &tris[(size_t)i * 3];

            if (tri_used[i])
                continue;
            if ((mapped[0] == t[0] || mapped[0] == t[1] || mapped[0] == t[2]) &&
                (mapped[1] == t[0] || mapped[1] == t[1] || mapped[1] == t[2]) &&
                (mapped[2] == t[0] || mapped[2] == t[1] || mapped[2] == t[2]) &&
                mapped[0] != mapped[1] && mapped[1] != mapped[2] && mapped[0] != mapped[2])
            {
                tri_used[i] = 1;
                input_tri = i;
            }
        }
        PRC_ASSERT(input_tri < 12);

        for (c = 0; c < 3; c++)
        {
            uint32_t nidx = parsed->normal_indices_prc_compressed_3d[k * 3 + c];
            const double *dn;

            PRC_ASSERT(nidx < (uint32_t)parsed->num_normals_prc_compressed_3d);
            dn = &parsed->normals_prc_compressed_3d[(size_t)nidx * 3];
            PRC_ASSERT(dot3(dn, face_normals[input_tri / 2]) > dot_min);
        }
    }
    for (k = 0; k < 12; k++)
        PRC_ASSERT_EQ(tri_used[k], 1);

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, angles);
    prc_free(ctx, bin);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Empirical pin of the C1 reversal-bit sign convention: a single triangle
   (no growable edges, so any bit value keeps the traversal in sync) with
   supplied normals parallel then anti-parallel to its winding cross product.
   The decoded normal must come back on the supplied side both times. */
static void
test_c1_sign_convention(prc_context *ctx, double nz)
{
    double positions[3 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    double normals[3 * 3] = {
        0.0, 0.0, nz,
        0.0, 0.0, nz,
        0.0, 0.0, nz
    };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_tess_3d_compressed *parsed = NULL;
    uint8_t *rev = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t nidx;
    double dot;
    int code;

    printf("  sub-case: C1 sign convention, input normal z = %+.0f\n", nz);

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 3, tris, 1,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, normals, &rev), 0);
    PRC_ASSERT_NOT_NULL(rev);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->num_triangle_indices_prc_compressed_3d / 3, 1);
    PRC_ASSERT_NOT_NULL(parsed->normal_indices_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(parsed->normals_prc_compressed_3d);

    nidx = parsed->normal_indices_prc_compressed_3d[0];
    PRC_ASSERT(nidx < (uint32_t)parsed->num_normals_prc_compressed_3d);
    dot = parsed->normals_prc_compressed_3d[(size_t)nidx * 3 + 2] * nz;
    PRC_ASSERT(dot > 0.9);

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
}

/* Session 4: analysis stub lifecycle. The quad is a single connected chain,
   so every emitted point must carry chain 0 and an offset equal to its own
   emission order. */
static void
test_vertex_analysis_stub(prc_context *ctx)
{
    double positions[] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[] = { 0, 1, 2, 0, 2, 3 };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    prc_vertex_analysis *analysis = NULL;
    uint32_t analysis_count = 0xFFFFFFFFu;
    uint32_t i;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: vertex analysis stub lifecycle\n");

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 4, tris, 2,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res,
        &analysis, &analysis_count, NULL), 0);
    PRC_ASSERT_NOT_NULL(analysis);
    PRC_ASSERT_EQ(analysis_count, res.num_decoded_points);
    PRC_ASSERT_EQ(analysis_count, 4);
    for (i = 0; i < analysis_count; i++)
    {
        const double *p = &mesh.positions[(size_t)res.point_mesh_vertex[i] * 3];

        PRC_ASSERT(analysis[i].reconstructed_position[0] == 0.0f);
        PRC_ASSERT(analysis[i].reconstructed_position[1] == 0.0f);
        PRC_ASSERT(analysis[i].reconstructed_position[2] == 0.0f);
        PRC_ASSERT(analysis[i].original_position[0] == (float)p[0]);
        PRC_ASSERT(analysis[i].original_position[1] == (float)p[1]);
        PRC_ASSERT(analysis[i].original_position[2] == (float)p[2]);
        PRC_ASSERT_EQ(analysis[i].chain_index, 0);
        PRC_ASSERT_EQ(analysis[i].chain_offset, i);
    }
    prc_free(ctx, analysis);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Session 4: 1,000,000-triangle regular grid, the first at-scale exercise of
   Steps A+B, including ~16 forced chain restarts past the 64K-per-chain cap.
   Deliberately stops before Steps C/E: the existing Huffman writer's
   O(count * distinct_leaves) leaf lookup makes bitstream assembly of a huge,
   highly-varied point_array a separate, known performance concern outside
   this test's numeric-stability scope. */
static void
test_million_triangle_grid(prc_context *ctx)
{
    enum { GRID_W = 1000, GRID_H = 500 };
    const uint32_t num_pos = (GRID_W + 1) * (GRID_H + 1);
    const uint32_t num_tris = 2 * GRID_W * GRID_H;
    double *positions;
    uint32_t *tris;
    uint32_t row, col, t, i;
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    clock_t start;
    double elapsed;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: 1,000,000-triangle grid stability\n");

    /* plain malloc: test scaffolding, not part of the leak-tracked encoder */
    positions = (double *)malloc((size_t)num_pos * 3 * sizeof(double));
    tris = (uint32_t *)malloc((size_t)num_tris * 3 * sizeof(uint32_t));
    PRC_ASSERT_NOT_NULL(positions);
    PRC_ASSERT_NOT_NULL(tris);

    for (row = 0; row <= GRID_H; row++)
    {
        for (col = 0; col <= GRID_W; col++)
        {
            size_t v = ((size_t)row * (GRID_W + 1) + col) * 3;

            positions[v + 0] = col / (double)GRID_W;
            positions[v + 1] = row / (double)GRID_H;
            positions[v + 2] = 0.0;
        }
    }
    t = 0;
    for (row = 0; row < GRID_H; row++)
    {
        for (col = 0; col < GRID_W; col++)
        {
            uint32_t v00 = row * (GRID_W + 1) + col;
            uint32_t v01 = v00 + 1;
            uint32_t v10 = v00 + (GRID_W + 1);
            uint32_t v11 = v10 + 1;

            tris[(size_t)t * 3 + 0] = v00;
            tris[(size_t)t * 3 + 1] = v01;
            tris[(size_t)t * 3 + 2] = v10;
            t++;
            tris[(size_t)t * 3 + 0] = v10;
            tris[(size_t)t * 3 + 1] = v01;
            tris[(size_t)t * 3 + 2] = v11;
            t++;
        }
    }
    PRC_ASSERT_EQ(t, num_tris);

    start = clock();
    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, num_pos, tris, num_tris,
        prc_write_tol_relative(1e-6), &mesh), 0);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res,
        NULL, NULL, NULL), 0);
    elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    printf("    preprocess + traversal wall time: %.3f s\n", elapsed);
    PRC_ASSERT(elapsed < 30.0);

    /* grid spacing (1e-3) >> tolerance (~1.4e-6): nothing may weld away */
    PRC_ASSERT_EQ(mesh.num_positions, num_pos);
    PRC_ASSERT_EQ(mesh.num_triangles, 1000000);
    PRC_ASSERT_EQ(res.edge_status_array_size, mesh.num_triangles);
    PRC_ASSERT_EQ(res.num_decoded_points, num_pos);
    /* a chain start emits 3 reference-bit slots, a grow step 1, so
       slots == num_tris + 2 * num_chains; the 64K-per-chain cap must have
       forced at least ceil(1e6 / 65536) == 16 chains */
    PRC_ASSERT_EQ((res.points_is_reference_array_size - mesh.num_triangles) % 2, 0);
    PRC_ASSERT((res.points_is_reference_array_size - mesh.num_triangles) / 2 >= 16);
    /* guard band tighter than INT32_MAX to catch near-overflow drift */
    for (i = 0; i < res.point_array_size; i++)
        PRC_ASSERT(res.point_array[i] >= -(1 << 30) && res.point_array[i] <= (1 << 30));

    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    free(positions);
    free(tris);
    LEAK_CHECK_END(ctx);
}

/* Session 4: unit cube pushed far from the world origin. The relative
   tolerance resolves against the LOCAL bbox diagonal (sqrt(3)), so quantized
   deltas must stay local-scale: the ceiling is diagonal/tolerance ==
   1/1e-6 == 1e6 steps (all deltas are measured from the bbox min corner or
   an edge basis, never from world zero). A regression to naive world-space
   quantization would produce ~1e11, overflow int32 and fail the traversal
   outright, so both assertions below would catch it. */
static void
test_large_offset_cube(prc_context *ctx)
{
    const double off[3] = { 100000.0, 50000.0, 75000.0 };
    double positions[8 * 3] = {
        -0.5, -0.5, -0.5,
         0.5, -0.5, -0.5,
         0.5,  0.5, -0.5,
        -0.5,  0.5, -0.5,
        -0.5, -0.5,  0.5,
         0.5, -0.5,  0.5,
         0.5,  0.5,  0.5,
        -0.5,  0.5,  0.5
    };
    uint32_t tris[12 * 3] = {
        0, 2, 1,   0, 3, 2,
        4, 5, 6,   4, 6, 7,
        0, 1, 5,   0, 5, 4,
        1, 2, 6,   1, 6, 5,
        2, 3, 7,   2, 7, 6,
        3, 0, 4,   3, 4, 7
    };
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    uint32_t i;
    int32_t max_abs = 0;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: large-world-offset cube\n");

    for (i = 0; i < 8; i++)
    {
        positions[i * 3 + 0] += off[0];
        positions[i * 3 + 1] += off[1];
        positions[i * 3 + 2] += off[2];
    }

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, 8, tris, 12,
        prc_write_tol_relative(1e-6), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, 8);
    PRC_ASSERT_EQ(mesh.num_triangles, 12);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res,
        NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ(res.num_decoded_points, 8);
    PRC_ASSERT_EQ(res.edge_status_array_size, 12);

    for (i = 0; i < res.point_array_size; i++)
    {
        int32_t a = res.point_array[i] < 0 ? -res.point_array[i] : res.point_array[i];

        if (a > max_abs)
            max_abs = a;
    }
    printf("    max |point_array| = %ld (local ceiling 1000001)\n", (long)max_abs);
    PRC_ASSERT(max_abs <= 1000001);

    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    LEAK_CHECK_END(ctx);
}

/* Utah teapot, 32 bicubic Bezier patches (a NURBS patch with unit weights and
   a Bezier-clamped knot vector is exactly a plain Bezier patch, so no
   weight/knot machinery is needed to evaluate this classic dataset). */
#define TEAPOT_NUM_PATCHES 32

static const double teapot_patches[TEAPOT_NUM_PATCHES][16][3] =
{
    { /* Patch 0 */
        {1.4,0,2.4},{1.4,-0.784,2.4},{0.784,-1.4,2.4},{0,-1.4,2.4},
        {1.3375,0,2.53125},{1.3375,-0.749,2.53125},{0.749,-1.3375,2.53125},{0,-1.3375,2.53125},
        {1.4375,0,2.53125},{1.4375,-0.805,2.53125},{0.805,-1.4375,2.53125},{0,-1.4375,2.53125},
        {1.5,0,2.4},{1.5,-0.84,2.4},{0.84,-1.5,2.4},{0,-1.5,2.4}
    },
    { /* Patch 1 */
        {0,-1.4,2.4},{-0.784,-1.4,2.4},{-1.4,-0.784,2.4},{-1.4,0,2.4},
        {0,-1.3375,2.53125},{-0.749,-1.3375,2.53125},{-1.3375,-0.749,2.53125},{-1.3375,0,2.53125},
        {0,-1.4375,2.53125},{-0.805,-1.4375,2.53125},{-1.4375,-0.805,2.53125},{-1.4375,0,2.53125},
        {0,-1.5,2.4},{-0.84,-1.5,2.4},{-1.5,-0.84,2.4},{-1.5,0,2.4}
    },
    { /* Patch 2 */
        {-1.4,0,2.4},{-1.4,0.784,2.4},{-0.784,1.4,2.4},{0,1.4,2.4},
        {-1.3375,0,2.53125},{-1.3375,0.749,2.53125},{-0.749,1.3375,2.53125},{0,1.3375,2.53125},
        {-1.4375,0,2.53125},{-1.4375,0.805,2.53125},{-0.805,1.4375,2.53125},{0,1.4375,2.53125},
        {-1.5,0,2.4},{-1.5,0.84,2.4},{-0.84,1.5,2.4},{0,1.5,2.4}
    },
    { /* Patch 3 */
        {0,1.4,2.4},{0.784,1.4,2.4},{1.4,0.784,2.4},{1.4,0,2.4},
        {0,1.3375,2.53125},{0.749,1.3375,2.53125},{1.3375,0.749,2.53125},{1.3375,0,2.53125},
        {0,1.4375,2.53125},{0.805,1.4375,2.53125},{1.4375,0.805,2.53125},{1.4375,0,2.53125},
        {0,1.5,2.4},{0.84,1.5,2.4},{1.5,0.84,2.4},{1.5,0,2.4}
    },
    { /* Patch 4 */
        {1.5,0,2.4},{1.5,-0.84,2.4},{0.84,-1.5,2.4},{0,-1.5,2.4},
        {1.75,0,1.875},{1.75,-0.98,1.875},{0.98,-1.75,1.875},{0,-1.75,1.875},
        {2,0,1.35},{2,-1.12,1.35},{1.12,-2,1.35},{0,-2,1.35},
        {2,0,0.9},{2,-1.12,0.9},{1.12,-2,0.9},{0,-2,0.9}
    },
    { /* Patch 5 */
        {0,-1.5,2.4},{-0.84,-1.5,2.4},{-1.5,-0.84,2.4},{-1.5,0,2.4},
        {0,-1.75,1.875},{-0.98,-1.75,1.875},{-1.75,-0.98,1.875},{-1.75,0,1.875},
        {0,-2,1.35},{-1.12,-2,1.35},{-2,-1.12,1.35},{-2,0,1.35},
        {0,-2,0.9},{-1.12,-2,0.9},{-2,-1.12,0.9},{-2,0,0.9}
    },
    { /* Patch 6 */
        {-1.5,0,2.4},{-1.5,0.84,2.4},{-0.84,1.5,2.4},{0,1.5,2.4},
        {-1.75,0,1.875},{-1.75,0.98,1.875},{-0.98,1.75,1.875},{0,1.75,1.875},
        {-2,0,1.35},{-2,1.12,1.35},{-1.12,2,1.35},{0,2,1.35},
        {-2,0,0.9},{-2,1.12,0.9},{-1.12,2,0.9},{0,2,0.9}
    },
    { /* Patch 7 */
        {0,1.5,2.4},{0.84,1.5,2.4},{1.5,0.84,2.4},{1.5,0,2.4},
        {0,1.75,1.875},{0.98,1.75,1.875},{1.75,0.98,1.875},{1.75,0,1.875},
        {0,2,1.35},{1.12,2,1.35},{2,1.12,1.35},{2,0,1.35},
        {0,2,0.9},{1.12,2,0.9},{2,1.12,0.9},{2,0,0.9}
    },
    { /* Patch 8 */
        {2,0,0.9},{2,-1.12,0.9},{1.12,-2,0.9},{0,-2,0.9},
        {2,0,0.45},{2,-1.12,0.45},{1.12,-2,0.45},{0,-2,0.45},
        {1.5,0,0.225},{1.5,-0.84,0.225},{0.84,-1.5,0.225},{0,-1.5,0.225},
        {1.5,0,0.15},{1.5,-0.84,0.15},{0.84,-1.5,0.15},{0,-1.5,0.15}
    },
    { /* Patch 9 */
        {0,-2,0.9},{-1.12,-2,0.9},{-2,-1.12,0.9},{-2,0,0.9},
        {0,-2,0.45},{-1.12,-2,0.45},{-2,-1.12,0.45},{-2,0,0.45},
        {0,-1.5,0.225},{-0.84,-1.5,0.225},{-1.5,-0.84,0.225},{-1.5,0,0.225},
        {0,-1.5,0.15},{-0.84,-1.5,0.15},{-1.5,-0.84,0.15},{-1.5,0,0.15}
    },
    { /* Patch 10 */
        {-2,0,0.9},{-2,1.12,0.9},{-1.12,2,0.9},{0,2,0.9},
        {-2,0,0.45},{-2,1.12,0.45},{-1.12,2,0.45},{0,2,0.45},
        {-1.5,0,0.225},{-1.5,0.84,0.225},{-0.84,1.5,0.225},{0,1.5,0.225},
        {-1.5,0,0.15},{-1.5,0.84,0.15},{-0.84,1.5,0.15},{0,1.5,0.15}
    },
    { /* Patch 11 */
        {0,2,0.9},{1.12,2,0.9},{2,1.12,0.9},{2,0,0.9},
        {0,2,0.45},{1.12,2,0.45},{2,1.12,0.45},{2,0,0.45},
        {0,1.5,0.225},{0.84,1.5,0.225},{1.5,0.84,0.225},{1.5,0,0.225},
        {0,1.5,0.15},{0.84,1.5,0.15},{1.5,0.84,0.15},{1.5,0,0.15}
    },
    { /* Patch 12 */
        {-1.6,0,2.025},{-1.6,-0.3,2.025},{-1.5,-0.3,2.25},{-1.5,0,2.25},
        {-2.3,0,2.025},{-2.3,-0.3,2.025},{-2.5,-0.3,2.25},{-2.5,0,2.25},
        {-2.7,0,2.025},{-2.7,-0.3,2.025},{-3,-0.3,2.25},{-3,0,2.25},
        {-2.7,0,1.8},{-2.7,-0.3,1.8},{-3,-0.3,1.8},{-3,0,1.8}
    },
    { /* Patch 13 */
        {-1.5,0,2.25},{-1.5,0.3,2.25},{-1.6,0.3,2.025},{-1.6,0,2.025},
        {-2.5,0,2.25},{-2.5,0.3,2.25},{-2.3,0.3,2.025},{-2.3,0,2.025},
        {-3,0,2.25},{-3,0.3,2.25},{-2.7,0.3,2.025},{-2.7,0,2.025},
        {-3,0,1.8},{-3,0.3,1.8},{-2.7,0.3,1.8},{-2.7,0,1.8}
    },
    { /* Patch 14 */
        {-2.7,0,1.8},{-2.7,-0.3,1.8},{-3,-0.3,1.8},{-3,0,1.8},
        {-2.7,0,1.575},{-2.7,-0.3,1.575},{-3,-0.3,1.35},{-3,0,1.35},
        {-2.5,0,1.125},{-2.5,-0.3,1.125},{-2.65,-0.3,0.9375},{-2.65,0,0.9375},
        {-2,0,0.9},{-2,-0.3,0.9},{-1.9,-0.3,0.6},{-1.9,0,0.6}
    },
    { /* Patch 15 */
        {-3,0,1.8},{-3,0.3,1.8},{-2.7,0.3,1.8},{-2.7,0,1.8},
        {-3,0,1.35},{-3,0.3,1.35},{-2.7,0.3,1.575},{-2.7,0,1.575},
        {-2.65,0,0.9375},{-2.65,0.3,0.9375},{-2.5,0.3,1.125},{-2.5,0,1.125},
        {-1.9,0,0.6},{-1.9,0.3,0.6},{-2,0.3,0.9},{-2,0,0.9}
    },
    { /* Patch 16 */
        {1.7,0,1.425},{1.7,-0.66,1.425},{1.7,-0.66,0.6},{1.7,0,0.6},
        {2.6,0,1.425},{2.6,-0.66,1.425},{3.1,-0.66,0.825},{3.1,0,0.825},
        {2.3,0,2.1},{2.3,-0.25,2.1},{2.4,-0.25,2.025},{2.4,0,2.025},
        {2.7,0,2.4},{2.7,-0.25,2.4},{3.3,-0.25,2.4},{3.3,0,2.4}
    },
    { /* Patch 17 */
        {1.7,0,0.6},{1.7,0.66,0.6},{1.7,0.66,1.425},{1.7,0,1.425},
        {3.1,0,0.825},{3.1,0.66,0.825},{2.6,0.66,1.425},{2.6,0,1.425},
        {2.4,0,2.025},{2.4,0.25,2.025},{2.3,0.25,2.1},{2.3,0,2.1},
        {3.3,0,2.4},{3.3,0.25,2.4},{2.7,0.25,2.4},{2.7,0,2.4}
    },
    { /* Patch 18 */
        {2.7,0,2.4},{2.7,-0.25,2.4},{3.3,-0.25,2.4},{3.3,0,2.4},
        {2.8,0,2.475},{2.8,-0.25,2.475},{3.525,-0.25,2.49375},{3.525,0,2.49375},
        {2.9,0,2.475},{2.9,-0.15,2.475},{3.45,-0.15,2.5125},{3.45,0,2.5125},
        {2.8,0,2.4},{2.8,-0.15,2.4},{3.2,-0.15,2.4},{3.2,0,2.4}
    },
    { /* Patch 19 */
        {3.3,0,2.4},{3.3,0.25,2.4},{2.7,0.25,2.4},{2.7,0,2.4},
        {3.525,0,2.49375},{3.525,0.25,2.49375},{2.8,0.25,2.475},{2.8,0,2.475},
        {3.45,0,2.5125},{3.45,0.15,2.5125},{2.9,0.15,2.475},{2.9,0,2.475},
        {3.2,0,2.4},{3.2,0.15,2.4},{2.8,0.15,2.4},{2.8,0,2.4}
    },
    { /* Patch 20 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0.8,0,3.15},{0.8,-0.45,3.15},{0.45,-0.8,3.15},{0,-0.8,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0.2,0,2.7},{0.2,-0.112,2.7},{0.112,-0.2,2.7},{0,-0.2,2.7}
    },
    { /* Patch 21 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0,-0.8,3.15},{-0.45,-0.8,3.15},{-0.8,-0.45,3.15},{-0.8,0,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0,-0.2,2.7},{-0.112,-0.2,2.7},{-0.2,-0.112,2.7},{-0.2,0,2.7}
    },
    { /* Patch 22 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {-0.8,0,3.15},{-0.8,0.45,3.15},{-0.45,0.8,3.15},{0,0.8,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {-0.2,0,2.7},{-0.2,0.112,2.7},{-0.112,0.2,2.7},{0,0.2,2.7}
    },
    { /* Patch 23 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0,0.8,3.15},{0.45,0.8,3.15},{0.8,0.45,3.15},{0.8,0,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0,0.2,2.7},{0.112,0.2,2.7},{0.2,0.112,2.7},{0.2,0,2.7}
    },
    { /* Patch 24 */
        {0.2,0,2.7},{0.2,-0.112,2.7},{0.112,-0.2,2.7},{0,-0.2,2.7},
        {0.4,0,2.55},{0.4,-0.224,2.55},{0.224,-0.4,2.55},{0,-0.4,2.55},
        {1.3,0,2.55},{1.3,-0.728,2.55},{0.728,-1.3,2.55},{0,-1.3,2.55},
        {1.3,0,2.4},{1.3,-0.728,2.4},{0.728,-1.3,2.4},{0,-1.3,2.4}
    },
    { /* Patch 25 */
        {0,-0.2,2.7},{-0.112,-0.2,2.7},{-0.2,-0.112,2.7},{-0.2,0,2.7},
        {0,-0.4,2.55},{-0.224,-0.4,2.55},{-0.4,-0.224,2.55},{-0.4,0,2.55},
        {0,-1.3,2.55},{-0.728,-1.3,2.55},{-1.3,-0.728,2.55},{-1.3,0,2.55},
        {0,-1.3,2.4},{-0.728,-1.3,2.4},{-1.3,-0.728,2.4},{-1.3,0,2.4}
    },
    { /* Patch 26 */
        {-0.2,0,2.7},{-0.2,0.112,2.7},{-0.112,0.2,2.7},{0,0.2,2.7},
        {-0.4,0,2.55},{-0.4,0.224,2.55},{-0.224,0.4,2.55},{0,0.4,2.55},
        {-1.3,0,2.55},{-1.3,0.728,2.55},{-0.728,1.3,2.55},{0,1.3,2.55},
        {-1.3,0,2.4},{-1.3,0.728,2.4},{-0.728,1.3,2.4},{0,1.3,2.4}
    },
    { /* Patch 27 */
        {0,0.2,2.7},{0.112,0.2,2.7},{0.2,0.112,2.7},{0.2,0,2.7},
        {0,0.4,2.55},{0.224,0.4,2.55},{0.4,0.224,2.55},{0.4,0,2.55},
        {0,1.3,2.55},{0.728,1.3,2.55},{1.3,0.728,2.55},{1.3,0,2.55},
        {0,1.3,2.4},{0.728,1.3,2.4},{1.3,0.728,2.4},{1.3,0,2.4}
    },
    { /* Patch 28 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {1.425,0,0},{1.425,0.798,0},{0.798,1.425,0},{0,1.425,0},
        {1.5,0,0.075},{1.5,0.84,0.075},{0.84,1.5,0.075},{0,1.5,0.075},
        {1.5,0,0.15},{1.5,0.84,0.15},{0.84,1.5,0.15},{0,1.5,0.15}
    },
    { /* Patch 29 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {0,1.425,0},{-0.798,1.425,0},{-1.425,0.798,0},{-1.425,0,0},
        {0,1.5,0.075},{-0.84,1.5,0.075},{-1.5,0.84,0.075},{-1.5,0,0.075},
        {0,1.5,0.15},{-0.84,1.5,0.15},{-1.5,0.84,0.15},{-1.5,0,0.15}
    },
    { /* Patch 30 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {-1.425,0,0},{-1.425,-0.798,0},{-0.798,-1.425,0},{0,-1.425,0},
        {-1.5,0,0.075},{-1.5,-0.84,0.075},{-0.84,-1.5,0.075},{0,-1.5,0.075},
        {-1.5,0,0.15},{-1.5,-0.84,0.15},{-0.84,-1.5,0.15},{0,-1.5,0.15}
    },
    { /* Patch 31 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {0,-1.425,0},{0.798,-1.425,0},{1.425,-0.798,0},{1.425,0,0},
        {0,-1.5,0.075},{0.84,-1.5,0.075},{1.5,-0.84,0.075},{1.5,0,0.075},
        {0,-1.5,0.15},{0.84,-1.5,0.15},{1.5,-0.84,0.15},{1.5,0,0.15}
    }
};

static void
teapot_bernstein(double t, double b[4])
{
    double mt = 1.0 - t;

    b[0] = mt * mt * mt;
    b[1] = 3.0 * t * mt * mt;
    b[2] = 3.0 * t * t * mt;
    b[3] = t * t * t;
}

static void
teapot_eval(const double cp[16][3], double u, double v, double out[3])
{
    double bu[4], bv[4];
    int i, j;

    teapot_bernstein(u, bu);
    teapot_bernstein(v, bv);
    out[0] = out[1] = out[2] = 0.0;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            double weight = bu[i] * bv[j];
            const double *p = cp[i * 4 + j];

            out[0] += weight * p[0];
            out[1] += weight * p[1];
            out[2] += weight * p[2];
        }
    }
}

/* Utah teapot, 32 patches each encoded as its own face group -- the first
   test in this file with more than one face id, exercising per-face
   grouping (triangle_face_array) at scale. Each patch is tessellated on an
   independent (TEAPOT_SAMPLES x TEAPOT_SAMPLES) grid, sampled INSET from
   u,v in {0,1} (never exactly at a patch corner), because 8 of the 32
   patches (the lid tip, patches 20-23, and the base, patches 28-31) collapse
   one whole control-point row to a single point; sampling exactly at that
   row would make every triangle touching it degenerate after quantized
   dedup, which would then have to be excluded from both the expected and
   decoded centroid to compare like with like. Staying inset sidesteps that
   entirely: every one of the (TEAPOT_SAMPLES)^2 samples per patch is
   guaranteed distinct, so nothing welds or gets removed, the expected
   centroid is a plain average, and the test is purely about face-group
   round-tripping, not degenerate-triangle bookkeeping (already covered
   elsewhere in this file). Patches are consequently not stitched into a
   watertight surface at their real seams -- irrelevant here, since only
   per-patch centroids are checked, never cross-patch connectivity. */
#define TEAPOT_GRID_N 4
#define TEAPOT_SAMPLES (TEAPOT_GRID_N + 1)
#define TEAPOT_VERTS_PER_PATCH (TEAPOT_SAMPLES * TEAPOT_SAMPLES)
#define TEAPOT_TRIS_PER_PATCH (2 * TEAPOT_GRID_N * TEAPOT_GRID_N)
#define TEAPOT_TOTAL_VERTS (TEAPOT_NUM_PATCHES * TEAPOT_VERTS_PER_PATCH)
#define TEAPOT_TOTAL_TRIS (TEAPOT_NUM_PATCHES * TEAPOT_TRIS_PER_PATCH)

static void
test_teapot_face_groups(prc_context *ctx)
{
    double *positions;
    uint32_t *tris;
    uint32_t *face_indices;
    double expected_centroid[TEAPOT_NUM_PATCHES][3];
    double decoded_sum[TEAPOT_NUM_PATCHES][3];
    uint32_t decoded_count[TEAPOT_NUM_PATCHES];
    uint8_t *seen;
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    uint8_t *rev = NULL;
    prc_tess_3d_compressed *parsed = NULL;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t p, i, j, k, c;
    int code;
    LEAK_CHECK_BEGIN(ctx);

    printf("  sub-case: Utah teapot, 32 patches as separate face groups\n");

    positions = (double *)malloc(sizeof(double) * TEAPOT_TOTAL_VERTS * 3);
    tris = (uint32_t *)malloc(sizeof(uint32_t) * TEAPOT_TOTAL_TRIS * 3);
    face_indices = (uint32_t *)malloc(sizeof(uint32_t) * TEAPOT_TOTAL_TRIS);
    PRC_ASSERT_NOT_NULL(positions);
    PRC_ASSERT_NOT_NULL(tris);
    PRC_ASSERT_NOT_NULL(face_indices);

    for (p = 0; p < TEAPOT_NUM_PATCHES; p++)
    {
        uint32_t vbase = p * TEAPOT_VERTS_PER_PATCH;
        uint32_t tbase = p * TEAPOT_TRIS_PER_PATCH;
        double sum[3];
        uint32_t t;

        sum[0] = sum[1] = sum[2] = 0.0;
        for (i = 0; i < TEAPOT_SAMPLES; i++)
        {
            double u = (double)(i + 1) / (double)(TEAPOT_SAMPLES + 1);

            for (j = 0; j < TEAPOT_SAMPLES; j++)
            {
                double v = (double)(j + 1) / (double)(TEAPOT_SAMPLES + 1);
                double pt[3];
                uint32_t vidx = vbase + i * TEAPOT_SAMPLES + j;

                teapot_eval(teapot_patches[p], u, v, pt);
                positions[vidx * 3 + 0] = pt[0];
                positions[vidx * 3 + 1] = pt[1];
                positions[vidx * 3 + 2] = pt[2];
                sum[0] += pt[0];
                sum[1] += pt[1];
                sum[2] += pt[2];
            }
        }
        expected_centroid[p][0] = sum[0] / TEAPOT_VERTS_PER_PATCH;
        expected_centroid[p][1] = sum[1] / TEAPOT_VERTS_PER_PATCH;
        expected_centroid[p][2] = sum[2] / TEAPOT_VERTS_PER_PATCH;

        t = 0;
        for (i = 0; i < TEAPOT_GRID_N; i++)
        {
            for (j = 0; j < TEAPOT_GRID_N; j++)
            {
                uint32_t v00 = vbase + i * TEAPOT_SAMPLES + j;
                uint32_t v01 = vbase + i * TEAPOT_SAMPLES + (j + 1);
                uint32_t v10 = vbase + (i + 1) * TEAPOT_SAMPLES + j;
                uint32_t v11 = vbase + (i + 1) * TEAPOT_SAMPLES + (j + 1);
                uint32_t tidx = tbase + t;

                tris[tidx * 3 + 0] = v00;
                tris[tidx * 3 + 1] = v10;
                tris[tidx * 3 + 2] = v11;
                face_indices[tidx] = p;
                t++;
                tidx = tbase + t;
                tris[tidx * 3 + 0] = v00;
                tris[tidx * 3 + 1] = v11;
                tris[tidx * 3 + 2] = v01;
                face_indices[tidx] = p;
                t++;
            }
        }
    }

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, TEAPOT_TOTAL_VERTS,
        tris, TEAPOT_TOTAL_TRIS, prc_write_tol_relative(1e-6), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, TEAPOT_TOTAL_VERTS);
    PRC_ASSERT_EQ(mesh.num_triangles, TEAPOT_TOTAL_TRIS);

    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, face_indices, mesh.tolerance_mm,
        &res, NULL, NULL, NULL), 0);
    PRC_ASSERT_EQ(prc_encode_normals_c1(ctx, &mesh, &res, NULL, &rev), 0);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 4096), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        rev, 30.0, NULL, 0, NULL, 0, 1, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, &parsed, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ((uint32_t)parsed->num_triangle_indices_prc_compressed_3d / 3, TEAPOT_TOTAL_TRIS);
    PRC_ASSERT_EQ((uint32_t)parsed->num_vertices_prc_compressed_3d, TEAPOT_TOTAL_VERTS);
    PRC_ASSERT_NOT_NULL(parsed->triangle_face_array);

    memset(decoded_sum, 0, sizeof(decoded_sum));
    memset(decoded_count, 0, sizeof(decoded_count));
    seen = (uint8_t *)calloc((size_t)parsed->num_vertices_prc_compressed_3d, 1);
    PRC_ASSERT_NOT_NULL(seen);

    for (p = 0; p < TEAPOT_NUM_PATCHES; p++)
    {
        memset(seen, 0, (size_t)parsed->num_vertices_prc_compressed_3d);
        for (k = 0; k < (uint32_t)parsed->num_triangle_indices_prc_compressed_3d / 3; k++)
        {
            if (parsed->triangle_face_array[k] != p)
                continue;
            for (c = 0; c < 3; c++)
            {
                uint32_t vidx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];

                if (seen[vidx])
                    continue;
                seen[vidx] = 1;
                decoded_sum[p][0] += parsed->vertices_prc_compressed_3d[(size_t)vidx * 3 + 0];
                decoded_sum[p][1] += parsed->vertices_prc_compressed_3d[(size_t)vidx * 3 + 1];
                decoded_sum[p][2] += parsed->vertices_prc_compressed_3d[(size_t)vidx * 3 + 2];
                decoded_count[p]++;
            }
        }
    }
    free(seen);

    for (p = 0; p < TEAPOT_NUM_PATCHES; p++)
    {
        double cx, cy, cz, err;

        PRC_ASSERT_EQ(decoded_count[p], TEAPOT_VERTS_PER_PATCH);
        cx = decoded_sum[p][0] / decoded_count[p];
        cy = decoded_sum[p][1] / decoded_count[p];
        cz = decoded_sum[p][2] / decoded_count[p];
        err = fabs(cx - expected_centroid[p][0]) +
              fabs(cy - expected_centroid[p][1]) +
              fabs(cz - expected_centroid[p][2]);
        PRC_ASSERT(err < 1e-3);
    }

    free_parsed_tess(ctx, parsed);
    prc_free(ctx, rev);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
    free(positions);
    free(tris);
    free(face_indices);
    LEAK_CHECK_END(ctx);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("compress_tess Phase 1b consolidated gate");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_single_triangle(ctx);
    test_quad_shared_edge(ctx);
    test_disjoint_triangles(ctx);
    test_degenerate_weld(ctx);
    test_half_tolerance_weld(ctx);
    test_tolerance_mode_equivalence(ctx);
    test_single_triangle_roundtrip(ctx);
    test_quad_roundtrip(ctx);
    test_disjoint_roundtrip(ctx);
    test_cube_c1_roundtrip(ctx);
    test_cube_c2_roundtrip(ctx);
    test_c1_sign_convention(ctx, 1.0);
    test_c1_sign_convention(ctx, -1.0);
    test_vertex_analysis_stub(ctx);
    test_million_triangle_grid(ctx);
    test_large_offset_cube(ctx);
    test_teapot_face_groups(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
