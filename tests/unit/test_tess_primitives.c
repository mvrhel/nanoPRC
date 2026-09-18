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

/* Triangle fans, triangle strips and per-vertex colours on the uncompressed
   write path, each written and then read back with this project's own parser.

   A round trip against our own reader proves the two sides agree, not that
   either matches the format -- the sign-extension defect survived years of
   round-tripping for exactly that reason. What makes these tests worth having
   is that they assert the *stored* fields as well: the entity flags, the
   triangulateddata words, the single-normal marker on a group's length word.
   Those are the things a third-party reader looks at, so pinning them catches
   a writer that has quietly agreed with our parser about the wrong encoding. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_write_tess_3d.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"

/* Mirrors prc_release_tess_3d / prc_release_tess_face, which are static in
   prc_release.c, plus the vertex-colour remainder array these cases populate
   and the plain triangle tests do not. */
static void
release_tess(prc_context *ctx, prc_tess_3d *d)
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
        {
            if (d->face_tessellation_data[k].triangulateddata != NULL)
                prc_free(ctx, d->face_tessellation_data[k].triangulateddata);
            if (d->face_tessellation_data[k].vertex_colors.color_data.remaining_vertices != NULL)
                prc_free(ctx, d->face_tessellation_data[k].vertex_colors.color_data.remaining_vertices);
        }
        prc_free(ctx, d->face_tessellation_data);
    }
    prc_free(ctx, d);
}

/* A square pyramid's four sides as one fan around the apex, plus a two-triangle
   base, so the face carries triangles and a fan at once -- which is the case
   the format allows and the writer has to get the ordering right for. */
static void
test_fan_with_triangles(prc_context *ctx)
{
    double positions[5 * 3] = {
        0.0, 0.0, 1.0,   /* 0 apex */
        -1.0, -1.0, 0.0, /* 1 */
        1.0, -1.0, 0.0,  /* 2 */
        1.0, 1.0, 0.0,   /* 3 */
        -1.0, 1.0, 0.0   /* 4 */
    };
    uint32_t tris[2 * 3] = { 1, 2, 3, 1, 3, 4 };      /* the base */
    uint32_t face_tri_counts[1] = { 2 };
    uint32_t face_fan_counts[1] = { 1 };
    uint32_t fan_vertex_counts[1] = { 6 };            /* apex + 4 rim + wrap */
    uint32_t fan_indices[6] = { 0, 1, 2, 3, 4, 1 };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    const prc_tess_face *face;
    int code;

    printf("  sub-case: one face carrying two triangles and a fan\n");

    memset(&p, 0, sizeof(p));
    p.positions = positions;
    p.num_positions = 5;
    p.tri_indices = tris;
    p.num_triangles = 2;
    p.face_tri_counts = face_tri_counts;
    p.num_faces = 1;
    p.face_fan_counts = face_fan_counts;
    p.fan_vertex_counts = fan_vertex_counts;
    p.fan_indices = fan_indices;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    code = prc_write_tess_3d_ex(ctx, &w, &p);
    if (code != 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->number_of_face_tessellation, 1);
    face = &parsed->face_tessellation_data[0];

    /* No normals supplied and must_calculate_normals not set, so both groups
       take their one-normal forms. */
    PRC_ASSERT_EQ(face->used_entities_flag,
        (uint32_t)(PRC_FACETESSDATA_TriangleOneNormal | PRC_FACETESSDATA_TriangleFanOneNormal));

    /* [tri_count][fan_count][fan0 length | single-normal marker] */
    PRC_ASSERT_EQ(face->size_of_triangulateddata, 3);
    PRC_ASSERT_EQ(face->triangulateddata[0], 2);
    PRC_ASSERT_EQ(face->triangulateddata[1], 1);
    PRC_ASSERT_EQ(face->triangulateddata[2] & ~(uint32_t)PRC_FACETESSDATA_NORMAL_Single, 6u);
    PRC_ASSERT(face->triangulateddata[2] & (uint32_t)PRC_FACETESSDATA_NORMAL_Single);

    /* Index stream: the two triangles first, each [face_normal, p, p, p] under
       the one-normal form, then the fan as [face_normal, p x 6]. */
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 2u * 4u + 7u);
    {
        const uint32_t *ix = parsed->triangulated_index_array;
        uint32_t base = face->start_triangulated;
        uint32_t fan_base = base + 8;
        uint32_t i;

        PRC_ASSERT_EQ(ix[base + 1] / 3, 1u);      /* first triangle, first vertex */
        PRC_ASSERT_EQ(ix[fan_base] / 3, 0u);      /* the fan's normal slot */
        for (i = 0; i < 6; i++)
            PRC_ASSERT_EQ(ix[fan_base + 1 + i] / 3, fan_indices[i]);
    }

    release_tess(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* A strip with per-vertex normals, which takes the multi-normal form and so
   carries a normal index beside every vertex and no marker on its length. */
static void
test_strip_with_normals(prc_context *ctx)
{
    double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 1.0, 0.0
    };
    double normals[2 * 3] = {
        0.0, 0.0, 1.0,
        0.0, 0.0, -1.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t norm_idx[3] = { 0, 0, 0 };
    uint32_t face_tri_counts[1] = { 1 };
    uint32_t face_strip_counts[1] = { 1 };
    uint32_t strip_vertex_counts[1] = { 4 };
    uint32_t strip_indices[4] = { 0, 1, 2, 3 };
    uint32_t strip_norm_indices[4] = { 0, 1, 0, 1 };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    const prc_tess_face *face;
    int code;

    printf("  sub-case: strip with supplied per-vertex normals\n");

    memset(&p, 0, sizeof(p));
    p.positions = positions;
    p.num_positions = 4;
    p.normals = normals;
    p.num_normals = 2;
    p.tri_indices = tris;
    p.norm_indices = norm_idx;
    p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts;
    p.num_faces = 1;
    p.face_strip_counts = face_strip_counts;
    p.strip_vertex_counts = strip_vertex_counts;
    p.strip_indices = strip_indices;
    p.strip_norm_indices = strip_norm_indices;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    code = prc_write_tess_3d_ex(ctx, &w, &p);
    if (code != 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    face = &parsed->face_tessellation_data[0];
    PRC_ASSERT_EQ(face->used_entities_flag,
        (uint32_t)(PRC_FACETESSDATA_Triangle | PRC_FACETESSDATA_TriangleStripe));

    /* [tri_count][strip_count][strip0 length], with no marker this time. */
    PRC_ASSERT_EQ(face->size_of_triangulateddata, 3);
    PRC_ASSERT_EQ(face->triangulateddata[0], 1);
    PRC_ASSERT_EQ(face->triangulateddata[1], 1);
    PRC_ASSERT_EQ(face->triangulateddata[2], 4u);
    PRC_ASSERT((face->triangulateddata[2] & (uint32_t)PRC_FACETESSDATA_NORMAL_Single) == 0);

    /* One triangle at 2 words per vertex, then the strip at 2 words per vertex. */
    PRC_ASSERT_EQ(parsed->number_of_triangulated_indicies, 3u * 2u + 4u * 2u);
    {
        const uint32_t *ix = parsed->triangulated_index_array;
        uint32_t base = face->start_triangulated + 6;
        uint32_t i;

        for (i = 0; i < 4; i++)
        {
            PRC_ASSERT_EQ(ix[base + i * 2] / 3, strip_norm_indices[i]);
            PRC_ASSERT_EQ(ix[base + i * 2 + 1] / 3, strip_indices[i]);
        }
    }

    release_tess(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* Vertex colours on a mesh. Two triangles, six vertex references, and a colour
   for each: the delta encoding has to survive a run of identical colours and a
   change in the middle of the run. */
static void
test_vertex_colors_rgb(prc_context *ctx)
{
    double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[2 * 3] = { 0, 1, 2, 0, 2, 3 };
    uint32_t face_tri_counts[1] = { 2 };
    /* references 0..2 red, 3 red again (tests is_same), 4..5 blue */
    uint8_t colors[6 * 3] = {
        255, 0, 0,
        255, 0, 0,
        255, 0, 0,
        255, 0, 0,
        0, 0, 255,
        0, 0, 255
    };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    const prc_tess_face *face;
    int code;

    printf("  sub-case: per-vertex RGB colours on a mesh face\n");

    memset(&p, 0, sizeof(p));
    p.positions = positions;
    p.num_positions = 4;
    p.tri_indices = tris;
    p.num_triangles = 2;
    p.face_tri_counts = face_tri_counts;
    p.num_faces = 1;
    p.vertex_colors = colors;
    p.num_vertex_colors = 6;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    code = prc_write_tess_3d_ex(ctx, &w, &p);
    if (code != 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    face = &parsed->face_tessellation_data[0];
    PRC_ASSERT_EQ(face->has_vertex_colors, 1);
    PRC_ASSERT_EQ(face->vertex_colors.is_rgba, 0);
    PRC_ASSERT_EQ(face->vertex_colors.is_segment_color, 0);
    PRC_ASSERT_EQ(face->vertex_colors.b_optimized, 0);

    PRC_ASSERT(face->vertex_colors.color_data.first_vertex.red == 255.0);
    PRC_ASSERT(face->vertex_colors.color_data.first_vertex.green == 0.0);
    PRC_ASSERT(face->vertex_colors.color_data.first_vertex.blue == 0.0);

    /* Entries 1..3 repeat the first, so the encoder must have written is_same
       for them; entry 4 changes and entry 5 repeats it. */
    {
        const prc_color_data_remainder *rem = face->vertex_colors.color_data.remaining_vertices;

        PRC_ASSERT_NOT_NULL(rem);
        PRC_ASSERT_EQ(rem[0].is_same, 1);
        PRC_ASSERT_EQ(rem[1].is_same, 1);
        PRC_ASSERT_EQ(rem[2].is_same, 1);
        PRC_ASSERT_EQ(rem[3].is_same, 0);
        PRC_ASSERT(rem[3].color.blue == 255.0);
        PRC_ASSERT(rem[3].color.red == 0.0);
        PRC_ASSERT_EQ(rem[4].is_same, 1);
        PRC_ASSERT(rem[4].color.blue == 255.0);
    }

    release_tess(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* RGBA, and colours covering fan vertices as well as triangle ones, which is
   where the count rule (one per vertex reference, not per position) bites. */
static void
test_vertex_colors_rgba_with_fan(prc_context *ctx)
{
    double positions[4 * 3] = {
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    uint32_t face_fan_counts[1] = { 1 };
    uint32_t fan_vertex_counts[1] = { 4 };
    uint32_t fan_indices[4] = { 0, 1, 2, 3 };
    /* 3 triangle references + 4 fan references = 7 colours */
    uint8_t colors[7 * 4] = {
        255, 0, 0, 255,
        255, 0, 0, 255,
        255, 0, 0, 255,
        0, 255, 0, 128,
        0, 255, 0, 128,
        0, 255, 0, 128,
        0, 255, 0, 128
    };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    const prc_tess_face *face;
    int code;

    printf("  sub-case: RGBA colours spanning triangles and a fan\n");

    memset(&p, 0, sizeof(p));
    p.positions = positions;
    p.num_positions = 4;
    p.tri_indices = tris;
    p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts;
    p.num_faces = 1;
    p.face_fan_counts = face_fan_counts;
    p.fan_vertex_counts = fan_vertex_counts;
    p.fan_indices = fan_indices;
    p.vertex_colors = colors;
    p.num_vertex_colors = 7;
    p.vertex_colors_have_alpha = 1;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 512), 0);
    code = prc_write_tess_3d_ex(ctx, &w, &p);
    if (code != 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    face = &parsed->face_tessellation_data[0];
    PRC_ASSERT_EQ(face->has_vertex_colors, 1);
    PRC_ASSERT_EQ(face->vertex_colors.is_rgba, 1);
    PRC_ASSERT(face->vertex_colors.color_data.first_vertex.alpha == 255.0);
    {
        const prc_color_data_remainder *rem = face->vertex_colors.color_data.remaining_vertices;

        PRC_ASSERT_EQ(rem[2].is_same, 0);           /* the change at reference 3 */
        PRC_ASSERT(rem[2].color.green == 255.0);
        PRC_ASSERT(rem[2].color.alpha == 128.0);
        PRC_ASSERT_EQ(rem[5].is_same, 1);           /* last fan vertex repeats */
    }

    release_tess(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* The refusals. Each of these is a case the writer could silently mangle, so
   it returns an error instead. */
static void
test_refusals(prc_context *ctx)
{
    double positions[3 * 3] = { 0,0,0, 1,0,0, 0,1,0 };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    uint32_t face_fan_counts[1] = { 1 };
    uint32_t short_fan[1] = { 2 };             /* a fan needs 3 */
    uint32_t fan_indices[2] = { 0, 1 };
    uint8_t colors[3 * 3] = { 1,2,3, 4,5,6, 7,8,9 };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;

    printf("  sub-case: malformed input is refused, not mangled\n");

    /* a fan of two vertices */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    p.face_fan_counts = face_fan_counts;
    p.fan_vertex_counts = short_fan;
    p.fan_indices = fan_indices;
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT(prc_write_tess_3d_ex(ctx, &w, &p) != 0);
    prc_bitwrite_release(ctx, &w);

    /* fan counts without the indices to go with them */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    p.face_fan_counts = face_fan_counts;
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT(prc_write_tess_3d_ex(ctx, &w, &p) != 0);
    prc_bitwrite_release(ctx, &w);

    /* a colour array that does not cover every vertex reference: one triangle
       needs three, and two is the kind of off-by-one that would otherwise
       desync the delta run */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    p.vertex_colors = colors;
    p.num_vertex_colors = 2;
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT(prc_write_tess_3d_ex(ctx, &w, &p) != 0);
    prc_bitwrite_release(ctx, &w);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("test_tess_primitives");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_fan_with_triangles(ctx);
    test_strip_with_normals(ctx);
    test_vertex_colors_rgb(ctx);
    test_vertex_colors_rgba_with_fan(ctx);
    test_refusals(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
