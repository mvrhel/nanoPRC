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

/* Phase 1b, Session 3 gate test: C2 "supplied normals" encoding
   (prc_encode_normals_c2 + prc_write_compress_tess_to_stream) round-tripped
   through the real, unmodified prc_parse_tess_3d_compressed. Covers both the
   has_multiple_normals = 0 path (every incident triangle agrees per vertex)
   and a genuine multi-normal vertex with a variable-width back-reference. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_write_compress_tess.h"
#include "prc_parse_tess.h"

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

/* Full C2 pipeline: preprocess -> traversal -> prc_encode_normals_c2 ->
   prc_write_compress_tess_to_stream -> flush -> the REAL parser. The exact
   normal_binary_data / normal_angle_array sizes are asserted so the per-point
   state machine (4 bits at first encounter, 0 for a single-normal reuse,
   1 + 3 bits for a new multi-normal entry, 1 + width bits for a back
   reference) is pinned, not just "something decodable". */
static void
encode_and_parse_c2(prc_context *ctx, const double *positions, uint32_t npos,
    const uint32_t *tris, uint32_t ntris, const double *corner_normals,
    uint32_t expect_bin_bits, uint32_t expect_angle_count,
    prc_tess_3d_compressed **parsed_out, double *tolerance_out)
{
    prc_encode_mesh mesh;
    prc_encode_traversal_result res;
    int32_t *angles = NULL;
    uint8_t *bin = NULL;
    uint32_t acount = 0, bsize = 0;
    prc_bit_write_state w;
    prc_bit_state r;
    int code;

    PRC_ASSERT_EQ(prc_encode_preprocess(ctx, positions, npos, tris, ntris,
        prc_write_tol_absolute(1e-4), &mesh), 0);
    PRC_ASSERT_EQ(mesh.num_positions, npos);
    PRC_ASSERT_EQ(mesh.num_triangles, ntris);
    PRC_ASSERT_EQ(prc_encode_traversal(ctx, &mesh, NULL, mesh.tolerance_mm, &res, NULL, NULL, NULL, 0), 0);
    PRC_ASSERT_NOT_NULL(res.triangle_point_indices);
    PRC_ASSERT_NOT_NULL(res.decoded_positions);

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
    PRC_ASSERT_EQ(bsize, expect_bin_bits);
    PRC_ASSERT_EQ(acount, expect_angle_count);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_compress_tess_to_stream(ctx, &w, &res, mesh.tolerance_mm,
        NULL, 0.0, angles, acount, bin, bsize, 0, NULL), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_compressed(ctx, &r, parsed_out, 0);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(*parsed_out);
    PRC_ASSERT_EQ((uint32_t)(*parsed_out)->num_triangle_indices_prc_compressed_3d / 3, ntris);
    PRC_ASSERT_EQ((uint32_t)(*parsed_out)->num_vertices_prc_compressed_3d, npos);

    *tolerance_out = mesh.tolerance_mm;
    prc_free(ctx, angles);
    prc_free(ctx, bin);
    prc_bitwrite_release(ctx, &w);
    prc_encode_traversal_free(ctx, &res);
    prc_encode_preprocess_free(ctx, &mesh);
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

/* Case 1: smooth shading -- every vertex's incident triangles agree on one
   normal, so every point takes the has_multiple_normals = 0 path and later
   visits consume zero bits. Deliberately asymmetric geometry so the decoder's
   theta1/theta2/theta3 basis-branch comparisons have wide margins. */
static void
test_smooth_agreeing_normals(prc_context *ctx)
{
    double positions[4 * 3] = {
         0.0, 0.0, 0.0,
         1.2, 0.0, 0.0,
         1.0, 1.1, 0.0,
        -0.2, 0.9, 0.0
    };
    uint32_t tris[2 * 3] = { 0, 1, 2, 0, 2, 3 };
    double corner_normals[2 * 9];
    double expected[3] = { 0.0, 0.0, -1.0 };
    prc_tess_3d_compressed *parsed = NULL;
    double tolerance = 0.0;
    uint32_t k, c;

    printf("  sub-case: smooth quad, all vertices agree (has_multiple = 0 path)\n");

    for (k = 0; k < 6; k++)
    {
        corner_normals[k * 3 + 0] = 0.0;
        corner_normals[k * 3 + 1] = 0.0;
        corner_normals[k * 3 + 2] = -1.0;
    }

    /* 4 first-time visits x 4 bits; the 2 revisit corners of the second
       triangle consume nothing; 2 angle entries per first-time visit */
    encode_and_parse_c2(ctx, positions, 4, tris, 2, corner_normals,
        16, 8, &parsed, &tolerance);

    PRC_ASSERT_NOT_NULL(parsed->normals_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(parsed->normal_indices_prc_compressed_3d);

    for (k = 0; k < 2; k++)
    {
        for (c = 0; c < 3; c++)
        {
            uint32_t vidx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];
            uint32_t nidx = parsed->normal_indices_prc_compressed_3d[k * 3 + c];
            const double *dn;

            PRC_ASSERT(vidx < 4);
            PRC_ASSERT(nidx < (uint32_t)parsed->num_normals_prc_compressed_3d);
            (void)nearest_input_vertex(&parsed->vertices_prc_compressed_3d[(size_t)vidx * 3],
                positions, 4, tolerance);
            dn = &parsed->normals_prc_compressed_3d[(size_t)nidx * 3];
            PRC_ASSERT(dot3(dn, expected) > 0.99);
        }
    }

    free_parsed_tess(ctx, parsed);
}

/* Case 2: a fan of four triangles around one central vertex where triangles
   A and D want the SAME center normal (N1), and B and C each want their own
   (N2, N3). The center therefore becomes a multiple-normals vertex with three
   stored entries by the time D visits it, forcing a genuine back-reference
   decoded through get_number_bits_to_store_unsigned_integer2 (2 bits wide
   here, not the trivial 1). Irregular radii/angles keep the basis-branch
   comparisons far from ties. */
static void
test_fan_multi_normal_back_reference(prc_context *ctx)
{
    double positions[6 * 3] = {
         0.0,       0.0,       0.0,   /* center */
         1.0,       0.0,       0.0,   /* 1.0 @ 0 deg */
         0.745623,  1.064861,  0.0,   /* 1.3 @ 55 deg */
        -0.380357,  0.815677,  0.0,   /* 0.9 @ 115 deg */
        -1.181769,  0.208378,  0.0,   /* 1.2 @ 170 deg */
        -0.630975, -0.901039,  0.0    /* 1.1 @ 235 deg */
    };
    uint32_t tris[4 * 3] = {
        0, 1, 2,   /* A: center normal N1 */
        0, 2, 3,   /* B: center normal N2 (new entry) */
        0, 3, 4,   /* C: center normal N3 (another new entry) */
        0, 4, 5    /* D: center normal N1 again (back-reference to A's) */
    };
    double n1[3] = { 0.0, 0.0, -1.0 };
    double n2[3] = { 0.5, 0.0, -0.8660254037844386 };
    double n3[3] = { -0.5, 0.0, -0.8660254037844386 };
    double corner_normals[4 * 9];
    const double *center_expected[4];
    prc_tess_3d_compressed *parsed = NULL;
    double tolerance = 0.0;
    uint32_t k, c, i;
    uint8_t tri_used[4] = { 0, 0, 0, 0 };

    printf("  sub-case: fan with multi-normal center vertex + back-reference\n");

    for (k = 0; k < 12; k++)
        memcpy(&corner_normals[(size_t)k * 3], n1, 3 * sizeof(double));
    memcpy(&corner_normals[1 * 9 + 0], n2, 3 * sizeof(double)); /* B center */
    memcpy(&corner_normals[2 * 9 + 0], n3, 3 * sizeof(double)); /* C center */
    center_expected[0] = n1;
    center_expected[1] = n2;
    center_expected[2] = n3;
    center_expected[3] = n1;

    /* A: 3 first-time visits (center becomes MULTIPLE)      -> 12 bits, 3 events
       B: center new entry (1+3), ring reuse 0, new ring 4   ->  8 bits, 2 events
       C: same shape as B                                    ->  8 bits, 2 events
       D: center back-ref (1 + 2 width bits), 0, new ring 4  ->  7 bits, 1 event */
    encode_and_parse_c2(ctx, positions, 6, tris, 4, corner_normals,
        35, 16, &parsed, &tolerance);

    PRC_ASSERT_NOT_NULL(parsed->normals_prc_compressed_3d);
    PRC_ASSERT_NOT_NULL(parsed->normal_indices_prc_compressed_3d);

    for (k = 0; k < 4; k++)
    {
        uint32_t mapped[3];
        uint32_t input_tri = 4;
        uint32_t min_ring = 6;

        for (c = 0; c < 3; c++)
        {
            uint32_t vidx = parsed->triangle_indices_prc_compressed_3d[k * 3 + c];

            PRC_ASSERT(vidx < 6);
            mapped[c] = nearest_input_vertex(
                &parsed->vertices_prc_compressed_3d[(size_t)vidx * 3],
                positions, 6, tolerance);
        }

        /* input triangle i covers {0, i+1, i+2}: identify by smallest ring id */
        PRC_ASSERT(mapped[0] == 0 || mapped[1] == 0 || mapped[2] == 0);
        for (c = 0; c < 3; c++)
        {
            if (mapped[c] != 0 && mapped[c] < min_ring)
                min_ring = mapped[c];
        }
        PRC_ASSERT(min_ring >= 1 && min_ring <= 4);
        input_tri = min_ring - 1;
        for (i = 0; i < 3; i++)
        {
            uint32_t want = i == 0 ? 0 : input_tri + i;

            PRC_ASSERT(mapped[0] == want || mapped[1] == want || mapped[2] == want);
        }
        PRC_ASSERT_EQ(tri_used[input_tri], 0);
        tri_used[input_tri] = 1;

        for (c = 0; c < 3; c++)
        {
            uint32_t nidx = parsed->normal_indices_prc_compressed_3d[k * 3 + c];
            const double *dn;
            const double *expect = mapped[c] == 0 ? center_expected[input_tri] : n1;

            PRC_ASSERT(nidx < (uint32_t)parsed->num_normals_prc_compressed_3d);
            dn = &parsed->normals_prc_compressed_3d[(size_t)nidx * 3];
            PRC_ASSERT(dot3(dn, expect) > 0.99);

            if (mapped[c] == 0)
            {
                /* prove the right stored normal was picked, not just "some":
                   B's center must NOT be N1, and D's back-referenced center
                   must NOT be N2 or N3 */
                if (input_tri == 1)
                    PRC_ASSERT(dot3(dn, n1) < 0.98);
                if (input_tri == 3)
                {
                    PRC_ASSERT(dot3(dn, n2) < 0.98);
                    PRC_ASSERT(dot3(dn, n3) < 0.98);
                }
            }
        }
    }
    for (k = 0; k < 4; k++)
        PRC_ASSERT_EQ(tri_used[k], 1);

    free_parsed_tess(ctx, parsed);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("compressed tess C2 supplied-normals round-trip");

    ctx = prc_api_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_smooth_agreeing_normals(ctx);
    test_fan_multi_normal_back_reference(ctx);

    /* Under PRC_ENABLE_DEBUG_MEMORY this fails if anything above leaked */
    PRC_ASSERT_EQ(prc_api_release_context(ctx), 0);

    PRC_TEST_END;
}
