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
    uint32_t fan_indices[6] = { 0, 1, 2, 3, 4, 1 };   /* apex + 4 rim + wrap */
    prc_api_write_tri_group fans[1];
    prc_api_write_face_groups groups[1];
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
    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].vertex_indices = fan_indices;
    fans[0].num_vertices = 6;
    groups[0].fans = fans;
    groups[0].num_fans = 1;
    p.face_groups = groups;

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
    uint32_t strip_indices[4] = { 0, 1, 2, 3 };
    uint32_t strip_norm_indices[4] = { 0, 1, 0, 1 };
    prc_api_write_tri_group strips[1];
    prc_api_write_face_groups groups[1];
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
    memset(strips, 0, sizeof(strips));
    memset(groups, 0, sizeof(groups));
    strips[0].vertex_indices = strip_indices;
    strips[0].normal_indices = strip_norm_indices;
    strips[0].num_vertices = 4;
    groups[0].strips = strips;
    groups[0].num_strips = 1;
    p.face_groups = groups;

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
    uint32_t fan_indices[4] = { 0, 1, 2, 3 };
    prc_api_write_tri_group fans[1];
    prc_api_write_face_groups groups[1];
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
    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].vertex_indices = fan_indices;
    fans[0].num_vertices = 4;
    groups[0].fans = fans;
    groups[0].num_fans = 1;
    p.face_groups = groups;
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
    uint32_t fan_indices[2] = { 0, 1 };
    prc_api_write_tri_group fans[1];
    prc_api_write_face_groups groups[1];
    uint8_t colors[3 * 3] = { 1,2,3, 4,5,6, 7,8,9 };
    prc_write_tess_3d_params p;
    prc_bit_write_state w;

    printf("  sub-case: malformed input is refused, not mangled\n");

    /* a fan of two vertices */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].vertex_indices = fan_indices;
    fans[0].num_vertices = 2;                  /* a fan needs 3 */
    groups[0].fans = fans;
    groups[0].num_fans = 1;
    p.face_groups = groups;
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT(prc_write_tess_3d_ex(ctx, &w, &p) != 0);
    prc_bitwrite_release(ctx, &w);

    /* a fan together with must_calculate_normals. The read side refuses this
       combination outright (prc_tri_primitives_api.c), so writing it would
       produce a file that opens cleanly and yields no geometry -- which is
       what it did before this refusal existed, and what a fixture opened in a
       real viewer showed. Measured absent from 307 third-party files. */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].vertex_indices = fan_indices;
    fans[0].num_vertices = 3;
    groups[0].fans = fans;
    groups[0].num_fans = 1;
    p.face_groups = groups;
    p.must_calculate_normals = 1;
    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT(prc_write_tess_3d_ex(ctx, &w, &p) != 0);
    prc_bitwrite_release(ctx, &w);

    /* a group declared with no indices to go with it */
    memset(&p, 0, sizeof(p));
    p.positions = positions; p.num_positions = 3;
    p.tri_indices = tris; p.num_triangles = 1;
    p.face_tri_counts = face_tri_counts; p.num_faces = 1;
    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].num_vertices = 3;                  /* but vertex_indices is NULL */
    groups[0].fans = fans;
    groups[0].num_fans = 1;
    p.face_groups = groups;
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

/* The other cases call prc_write_tess_3d_ex directly, which proves the writer
   works but says nothing about whether a caller can REACH it. That gap is not
   hypothetical: the public fields were once published while
   prc_write_file_structure.c still called the old fixed-argument writer, so a
   caller could set face_groups and silently get a plain triangle mesh, and
   every direct-writer test still passed.

   So this case goes the whole way round: build a mesh through the public
   prc_api_write_tessellation, write it with prc_api_write_prc_buffer, reopen
   the file and assert the fan actually survived into the stored face. */
static void
test_fan_through_public_api(prc_context *ctx)
{
    static const char *fname = "test_tess_primitives_public.prc";
    double positions[5 * 3] = { 0,0,0,  1,0,0,  1,1,0,  0,1,0,  0.5,0.5,1 };
    uint32_t tris[3] = { 0, 1, 2 };
    uint32_t face_tri_counts[1] = { 1 };
    uint32_t fan_indices[4] = { 4, 0, 1, 2 };
    double normals[2 * 3] = { 0,0,1,  0,1,0 };
    uint32_t tri_norm[3] = { 0, 0, 0 };
    uint32_t fan_norm[4] = { 1, 1, 1, 1 };
    prc_api_write_tri_group fans[1];
    prc_api_write_face_groups groups[1];
    prc_api_write_tessellation tess[1];
    prc_api_write_rep_item items[1];
    prc_api_write_node root;
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_data *data = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;

    printf("  sub-case: a fan survives the public write API end to end\n");

    memset(fans, 0, sizeof(fans));
    memset(groups, 0, sizeof(groups));
    fans[0].vertex_indices = fan_indices;
    fans[0].normal_indices = fan_norm;
    fans[0].num_vertices = 4;
    groups[0].fans = fans;
    groups[0].num_fans = 1;

    memset(tess, 0, sizeof(tess));
    tess[0].kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
    tess[0].positions = positions;
    tess[0].num_positions = 5;
    tess[0].tri_indices = tris;
    tess[0].num_triangles = 1;
    tess[0].face_tri_counts = face_tri_counts;
    tess[0].num_faces = 1;
    tess[0].normals = normals;
    tess[0].num_normals = 2;
    tess[0].norm_indices = tri_norm;
    tess[0].face_groups = groups;

    memset(items, 0, sizeof(items));
    items[0].kind = PRC_API_WRITE_RI_SURFACE;
    items[0].biased_tessellation_index = 1;

    memset(&root, 0, sizeof(root));
    root.name = "fan_root";
    root.rep_items = items;
    root.num_rep_items = 1;
    root.bbox_max[0] = root.bbox_max[1] = root.bbox_max[2] = 1.0;

    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "fan_model", &root,
        tess, 1, &buf, &buf_size), 0);
    PRC_ASSERT_NOT_NULL(buf);
    PRC_ASSERT(buf_size > 0);

    fid = fopen(fname, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = (prc_data *)prc_api_open_contents(ctx, fname);
    if (data == NULL)
        prc_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(data);
    PRC_ASSERT_EQ(data->file_structure_count, 1);
    PRC_ASSERT_NOT_NULL(data->file_struct[0].tessellation);
    PRC_ASSERT_EQ(data->file_struct[0].tessellation->tess_count, 1);

    /* The stored face must carry the fan flag and the fan's length word. A
       writer that dropped face_groups on the floor would still produce a
       valid file here -- with a triangle count word only -- so these two
       assertions are the whole point of the case. */
    {
        prc_tess_3d *t3d = data->file_struct[0].tessellation->tess[0].tess_3d;
        const prc_tess_face *face;

        PRC_ASSERT_NOT_NULL(t3d);
        PRC_ASSERT(t3d->number_of_face_tessellation >= 1);
        face = &t3d->face_tessellation_data[0];

        /* The fan supplies normal indices, so it takes the multi-normal form
           and its length word is NOT masked. (It used to reach this state via
           must_calculate_normals, which the writer now refuses with fans.) */
        PRC_ASSERT(face->used_entities_flag & PRC_FACETESSDATA_TriangleFan);
        PRC_ASSERT_EQ(face->size_of_triangulateddata, 3);   /* tri count, fan count, one length */
        PRC_ASSERT_EQ(face->triangulateddata[1], 1u);       /* one fan */
        PRC_ASSERT_EQ(face->triangulateddata[2], 4u);       /* four vertices, unmasked */
    }

    prc_api_release_data(ctx, (prc_api_data)data, NULL, 0, NULL, 0, NULL, 0, NULL);
    remove(fname);
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
    test_fan_through_public_api(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
