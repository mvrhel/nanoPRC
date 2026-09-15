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

/* Phase 1c, session 1 gate test: prc_write_tess_3d (the uncompressed
   tessellation encoder) round-tripped through the real, unmodified
   prc_parse_tess_3d. No quantization in this format, so positions and
   normals must come back bit-for-bit exact. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_write_tess_3d.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"

/* Mirrors the (static, so not reusable directly) prc_release_tess_3d /
   prc_release_tess_face in prc_release.c -- frees exactly the fields this
   simple Triangle/TriangleOneNormal-only, no-texture, no-wire, no-vertex-
   color encoding ever populates. */
static void
free_parsed_tess_3d(prc_context *ctx, prc_tess_3d *d)
{
    uint32_t k;

    if (d == NULL)
        return;
    if (d->tessellation_coordinates.coordinates != NULL)
        prc_free(ctx, d->tessellation_coordinates.coordinates);
    if (d->normal_coordinates != NULL)
        prc_free(ctx, d->normal_coordinates);
    if (d->wire_indices != NULL)
        prc_free(ctx, d->wire_indices);
    if (d->triangulated_index_array != NULL)
        prc_free(ctx, d->triangulated_index_array);
    if (d->texture_coordinates != NULL)
        prc_free(ctx, d->texture_coordinates);
    if (d->face_tessellation_data != NULL)
    {
        for (k = 0; k < d->number_of_face_tessellation; k++)
            if (d->face_tessellation_data[k].triangulateddata != NULL)
                prc_free(ctx, d->face_tessellation_data[k].triangulateddata);
        prc_free(ctx, d->face_tessellation_data);
    }
    prc_free(ctx, d);
}

/* Flat quad, 2 triangles sharing a diagonal, 1 face, per-vertex normals
   supplied (PRC_FACETESSDATA_Triangle path). No quantization in this
   format, so positions and normals must come back exact. */
static void
test_flat_quad_exact(prc_context *ctx)
{
    double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    double normals[4 * 3] = {
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0
    };
    uint32_t tris[2 * 3] = { 0, 1, 2, 0, 2, 3 };
    uint32_t norm_idx[2 * 3] = { 0, 1, 2, 0, 2, 3 };
    uint32_t face_tri_counts[1] = { 2 };
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    uint32_t i, k;
    int code;

    printf("  sub-case: flat quad, exact positions and normals\n");

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_tess_3d(ctx, &w, positions, 4, normals, 4,
        tris, norm_idx, 2, face_tri_counts, 1, NULL, 0, NULL, 0, 0.0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->tessellation_coordinates.number_of_coordinates, 12);
    PRC_ASSERT_NOT_NULL(parsed->tessellation_coordinates.coordinates);
    for (i = 0; i < 12; i++)
        PRC_ASSERT(parsed->tessellation_coordinates.coordinates[i] == positions[i]);

    PRC_ASSERT_EQ(parsed->number_of_normal_coordinates, 12);
    PRC_ASSERT_NOT_NULL(parsed->normal_coordinates);
    for (i = 0; i < 12; i++)
        PRC_ASSERT(parsed->normal_coordinates[i] == normals[i]);

    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 1);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].used_entities_flag, PRC_FACETESSDATA_Triangle);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].size_of_triangulateddata, 1);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].triangulateddata[0], 2);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].start_triangulated, 0);

    /* Walk the global index array exactly as prc_internal_api_get_normal_position_index
       does for PRC_FACETESSDATA_Triangle (multi-norm, no texture, normals
       present): 2 raw values per vertex, [normal_index*3, position_index*3]. */
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 12);
    for (k = 0; k < 2; k++)
    {
        for (i = 0; i < 3; i++)
        {
            uint32_t base = parsed->face_tessellation_data[0].start_triangulated + (k * 3 + i) * 2;
            uint32_t decoded_norm = parsed->triangulated_index_array[base] / 3;
            uint32_t decoded_pos = parsed->triangulated_index_array[base + 1] / 3;

            PRC_ASSERT_EQ(decoded_norm, norm_idx[k * 3 + i]);
            PRC_ASSERT_EQ(decoded_pos, tris[k * 3 + i]);
        }
    }

    free_parsed_tess_3d(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* Three faces of 2 triangles each: verify face_tri_counts is recovered
   correctly from the decoded face structure (each face's own
   triangulateddata[0], and start_triangulated advancing by exactly this
   face's contribution to the shared global index array). */
static void
test_multi_face_counts(prc_context *ctx)
{
    /* 6 independent triangles (18 distinct vertices), grouped 2+2+2 into 3 faces */
    double positions[18 * 3];
    double normals[18 * 3];
    uint32_t tris[6 * 3];
    uint32_t norm_idx[6 * 3];
    uint32_t face_tri_counts[3] = { 2, 2, 2 };
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    uint32_t t, v, f;
    int code;

    printf("  sub-case: multi-face, face_tri_counts recovered\n");

    for (t = 0; t < 6; t++)
    {
        for (v = 0; v < 3; v++)
        {
            uint32_t idx = t * 3 + v;

            positions[idx * 3 + 0] = (double)t;
            positions[idx * 3 + 1] = (double)v;
            positions[idx * 3 + 2] = 0.0;
            normals[idx * 3 + 0] = 0.0;
            normals[idx * 3 + 1] = 0.0;
            normals[idx * 3 + 2] = 1.0;
            tris[idx] = idx;
            norm_idx[idx] = idx;
        }
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    PRC_ASSERT_EQ(prc_write_tess_3d(ctx, &w, positions, 18, normals, 18,
        tris, norm_idx, 6, face_tri_counts, 3, NULL, 0, NULL, 0, 0.0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);
    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 3);

    for (f = 0; f < 3; f++)
    {
        PRC_ASSERT_EQ(parsed->face_tessellation_data[f].used_entities_flag, PRC_FACETESSDATA_Triangle);
        PRC_ASSERT_EQ(parsed->face_tessellation_data[f].size_of_triangulateddata, 1);
        PRC_ASSERT_EQ(parsed->face_tessellation_data[f].triangulateddata[0], face_tri_counts[f]);
        /* each face's 2 triangles * 3 vertices * 2 raw values = 12 words */
        PRC_ASSERT_EQ(parsed->face_tessellation_data[f].start_triangulated, f * 12);
    }
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 36);

    free_parsed_tess_3d(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* No normals supplied (norm_indices == NULL): verify the
   PRC_FACETESSDATA_TriangleOneNormal path is used, and that the single
   written normal is the correct outward-facing cross product of the
   face's first triangle. */
static void
test_no_normals_one_normal_path(prc_context *ctx)
{
    /* Right triangle in the XY plane, CCW when viewed from +Z: cross(e1,e2)
       for e1=(1,0,0), e2=(0,1,0) is (0,0,1). */
    double positions[3 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    uint32_t base;
    int code;

    printf("  sub-case: no normals supplied, TriangleOneNormal path\n");

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_tess_3d(ctx, &w, positions, 3, NULL, 0,
        tris, NULL, 1, face_tri_counts, 1, NULL, 0, NULL, 0, 0.0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 1);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].used_entities_flag, PRC_FACETESSDATA_TriangleOneNormal);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].triangulateddata[0], 1);

    /* Exactly one face normal was computed and written */
    PRC_ASSERT_EQ(parsed->number_of_normal_coordinates, 3);
    PRC_ASSERT(fabs(parsed->normal_coordinates[0] - 0.0) < 1e-12);
    PRC_ASSERT(fabs(parsed->normal_coordinates[1] - 0.0) < 1e-12);
    PRC_ASSERT(fabs(parsed->normal_coordinates[2] - 1.0) < 1e-12);

    /* TriangleOneNormal layout: [normal_index*3, pos0*3, pos1*3, pos2*3] */
    base = parsed->face_tessellation_data[0].start_triangulated;
    PRC_ASSERT_EQ(parsed->triangulated_index_array[base] / 3, 0);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[base + 1] / 3, tris[0]);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[base + 2] / 3, tris[1]);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[base + 3] / 3, tris[2]);

    free_parsed_tess_3d(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* must_calculate_normals=1: no normal data stored at all, position-only
   triangulated_index_array, PRC_FACETESSDATA_Triangle -- the real-producer
   convention confirmed via dump_uncompressed_tess_fields.c on
   xml-sample-wrl_ePRC.pdf/ElevationMeshIS_ePRC.pdf (normal_recalculation_
   flags=0, crease_angle stored as raw degrees). */
static void
test_must_calculate_normals(prc_context *ctx)
{
    double positions[3 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    int code;

    printf("  sub-case: must_calculate_normals=1, no stored normals\n");

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_tess_3d(ctx, &w, positions, 3, NULL, 0,
        tris, NULL, 1, face_tri_counts, 1, NULL, 0, NULL, 1, 45.0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->must_calculate_normals, 1);
    PRC_ASSERT_EQ(parsed->normal_recalculation_flags, 0);
    PRC_ASSERT(fabs(parsed->crease_angle - 45.0 * PRC_PI / 180.0) < 1e-12);
    PRC_ASSERT_EQ(parsed->number_of_normal_coordinates, 0);

    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 1);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].used_entities_flag, PRC_FACETESSDATA_Triangle);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].triangulateddata[0], 1);

    /* Position-only layout (no interleaved normal index): [pos0*3, pos1*3, pos2*3] */
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 3);
    PRC_ASSERT_EQ(parsed->face_tessellation_data[0].start_triangulated, 0);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[0] / 3, tris[0]);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[1] / 3, tris[1]);
    PRC_ASSERT_EQ(parsed->triangulated_index_array[2] / 3, tris[2]);

    free_parsed_tess_3d(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* Textured cube: 8 shared corner positions, 6 face normals, and only FOUR
   texture coordinates reused across all six faces.

   Deliberately not 24 duplicated corners. Duplicating positions would make
   the texture indices run parallel to the position indices, so an encoder
   that emitted them in the wrong order, or scaled them like positions, would
   still round-trip. Sharing 8 corners while giving each face its own UV
   assignment forces the two index streams genuinely apart: for most vertices
   texture index != position index, which is what makes the interleave and
   the scale observable.

   Two conventions are asserted here that the specification does not state --
   see pdf-association/pdf-issues#810 (CR-29/CR-30). Both were measured
   against real files (welding_robot_kinematics_animation.stream-45 and
   PartList-Helico.stream-435 in the public prc-db corpus) before being
   implemented:
     - number_of_texture_coordinates counts DOUBLES, so 4 (u,v) pairs are
       written as 8;
     - texture indices are stored pre-multiplied by 2, while position and
       normal indices are pre-multiplied by 3.

   Per Table 139 "PRC tessellation types", PRC_FACETESSDATA_TriangleTextured
   lays each vertex out as (normal, texture, point). */
static void
test_textured_cube(prc_context *ctx)
{
    /* Unit cube corners. */
    double positions[8 * 3] = {
        0.0, 0.0, 0.0,   1.0, 0.0, 0.0,   1.0, 1.0, 0.0,   0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,   1.0, 0.0, 1.0,   1.0, 1.0, 1.0,   0.0, 1.0, 1.0
    };
    /* One outward normal per face, in the face order used below. */
    double normals[6 * 3] = {
        0.0, 0.0, -1.0,   /* bottom */
        0.0, 0.0,  1.0,   /* top    */
        0.0, -1.0, 0.0,   /* front  */
        1.0, 0.0,  0.0,   /* right  */
        0.0, 1.0,  0.0,   /* back   */
        -1.0, 0.0, 0.0    /* left   */
    };
    /* Four UV pairs, shared by every face: the corners of the texture. */
    double tex_coords[4 * 2] = {
        0.0, 0.0,
        1.0, 0.0,
        1.0, 1.0,
        0.0, 1.0
    };
    /* Six quads, each split into two triangles. */
    uint32_t tris[12 * 3] = {
        0, 3, 2,  0, 2, 1,      /* bottom */
        4, 5, 6,  4, 6, 7,      /* top    */
        0, 1, 5,  0, 5, 4,      /* front  */
        1, 2, 6,  1, 6, 5,      /* right  */
        2, 3, 7,  2, 7, 6,      /* back   */
        3, 0, 4,  3, 4, 7       /* left   */
    };
    uint32_t norm_idx[12 * 3];
    /* Every quad maps to the same texture corners: (0,1,2) then (0,2,3). */
    uint32_t tex_idx[12 * 3] = {
        0, 1, 2,  0, 2, 3,
        0, 1, 2,  0, 2, 3,
        0, 1, 2,  0, 2, 3,
        0, 1, 2,  0, 2, 3,
        0, 1, 2,  0, 2, 3,
        0, 1, 2,  0, 2, 3
    };
    uint32_t face_tri_counts[6] = { 2, 2, 2, 2, 2, 2 };
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    uint32_t f, k, i, c;
    int code;

    printf("  sub-case: textured cube, 8 shared corners and 4 reused UVs\n");

    /* Both triangles of face f carry face f's normal. */
    for (f = 0; f < 6; f++)
        for (i = 0; i < 6; i++)
            norm_idx[f * 6 + i] = f;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 1024), 0);
    PRC_ASSERT_EQ(prc_write_tess_3d(ctx, &w, positions, 8, normals, 6,
        tris, norm_idx, 12, face_tri_counts, 6,
        tex_coords, 4, tex_idx, 0, 0.0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->tessellation_coordinates.number_of_coordinates, 24);
    PRC_ASSERT_EQ(parsed->number_of_normal_coordinates, 18);

    /* Counted in doubles, not in coordinates: 4 pairs -> 8. */
    PRC_ASSERT_EQ(parsed->number_of_texture_coordinates, 8);
    PRC_ASSERT_NOT_NULL(parsed->texture_coordinates);
    for (i = 0; i < 8; i++)
        PRC_ASSERT(parsed->texture_coordinates[i] == tex_coords[i]);

    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 6);

    /* 12 triangles x 3 vertices x 3 entries (normal, texture, point). */
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 108);

    for (f = 0; f < 6; f++)
    {
        prc_tess_face *face = &parsed->face_tessellation_data[f];

        PRC_ASSERT_EQ(face->used_entities_flag, PRC_FACETESSDATA_TriangleTextured);
        PRC_ASSERT_EQ(face->number_of_textured_coordinate_indexes, 1);
        PRC_ASSERT_EQ(face->size_of_triangulateddata, 1);
        PRC_ASSERT_EQ(face->triangulateddata[0], 2);
        PRC_ASSERT_EQ(face->start_triangulated, f * 18);

        /* Walk exactly as prc_internal_api_get_normal_texture_position_index
           does for the multi-normal textured case: per vertex the array holds
           [normal*3, texture*2, position*3]. The differing divisors are the
           whole point of the case -- a writer that scaled the texture index
           by 3, or emitted it in the wrong slot, fails here. */
        for (k = 0; k < 2; k++)
        {
            for (c = 0; c < 3; c++)
            {
                uint32_t base = face->start_triangulated + (k * 3 + c) * 3;
                uint32_t raw_norm = parsed->triangulated_index_array[base];
                uint32_t raw_tex = parsed->triangulated_index_array[base + 1];
                uint32_t raw_pos = parsed->triangulated_index_array[base + 2];
                uint32_t src = (f * 2 + k) * 3 + c;

                PRC_ASSERT_EQ(raw_norm % 3, 0);
                PRC_ASSERT_EQ(raw_tex % 2, 0);
                PRC_ASSERT_EQ(raw_pos % 3, 0);

                PRC_ASSERT_EQ(raw_norm / 3, norm_idx[src]);
                PRC_ASSERT_EQ(raw_tex / 2, tex_idx[src]);
                PRC_ASSERT_EQ(raw_pos / 3, tris[src]);

                /* The decoded texture index must stay inside the array, which
                   is the failure mode an unscaled or 3-scaled index produces
                   on a real file. */
                PRC_ASSERT(raw_tex / 2 < parsed->number_of_texture_coordinates / 2);
            }
        }
    }

    /* The two streams really are independent in this fixture: if they were
       not, the case would prove nothing about the interleave. */
    {
        uint32_t differing = 0;
        for (i = 0; i < 36; i++)
            if (tex_idx[i] != tris[i])
                differing++;
        PRC_ASSERT(differing > 0);
    }

    free_parsed_tess_3d(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("tess_3d uncompressed encoder round trip");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_flat_quad_exact(ctx);
    test_multi_face_counts(ctx);
    test_no_normals_one_normal_path(ctx);
    test_must_calculate_normals(ctx);
    test_textured_cube(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
