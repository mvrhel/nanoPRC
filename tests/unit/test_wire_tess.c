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

/* Phase 1c, session 2 gate test: prc_write_wire_tess (the wire/polyline
   tessellation encoder) round-tripped through the real, unmodified
   prc_parse_tess_3d_wire. */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_write_wire_tess.h"
#include "prc_parse_tess.h"

/* Mirrors the (static, so not reusable directly) prc_release_tess_3d_wire
   in prc_release.c. */
static void
free_parsed_wire(prc_context *ctx, prc_tess_3d_wire *d)
{
    uint32_t k;

    if (d == NULL)
        return;
    if (d->tessellation_coordinates.coordinates != NULL)
        prc_free(ctx, d->tessellation_coordinates.coordinates);
    if (d->wire_elements != NULL)
    {
        for (k = 0; k < d->number_of_wire_elements; k++)
            if (d->wire_elements[k].wire_indexes != NULL)
                prc_free(ctx, d->wire_elements[k].wire_indexes);
        prc_free(ctx, d->wire_elements);
    }
    if (d->has_vertex_colors && d->vertex_color_data.color_data.remaining_vertices != NULL)
        prc_free(ctx, d->vertex_color_data.color_data.remaining_vertices);
    prc_free(ctx, d);
}

/* 3 independent line segments (2 vertices each), no colors. */
static void
test_three_line_segments_no_colors(prc_context *ctx)
{
    float p[6][3] = {
        { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f },
        { 2.0f, 0.0f, 0.0f }, { 2.0f, 1.0f, 0.0f },
        { 3.0f, 0.0f, 0.0f }, { 3.0f, 1.0f, 1.0f }
    };
    prc_write_wire_element elems[3];
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_wire *parsed = NULL;
    uint32_t e, v;
    int code;

    printf("  sub-case: 3 line segments, no colors\n");

    memset(elems, 0, sizeof(elems));
    for (e = 0; e < 3; e++)
    {
        elems[e].positions = p[e * 2];
        elems[e].num_vertices = 2;
        elems[e].is_closed = 0;
        elems[e].is_continuous = 0;
        elems[e].colors = NULL;
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_wire_tess(ctx, &w, elems, 3), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_wire(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->number_of_wire_elements, 3);
    PRC_ASSERT_EQ(parsed->has_vertex_colors, 0);

    for (e = 0; e < 3; e++)
    {
        PRC_ASSERT_EQ(parsed->wire_elements[e].number_of_wire_indexes, 2);
        PRC_ASSERT_EQ(parsed->wire_elements[e].is_connected & PRC_3DWIRETESSDATA_IsClosing, 0);

        for (v = 0; v < 2; v++)
        {
            uint32_t pool_idx = parsed->wire_elements[e].wire_indexes[v] / 3;
            double *pos = &parsed->tessellation_coordinates.coordinates[(size_t)pool_idx * 3];

            PRC_ASSERT(pos[0] == (double)p[e * 2 + v][0]);
            PRC_ASSERT(pos[1] == (double)p[e * 2 + v][1]);
            PRC_ASSERT(pos[2] == (double)p[e * 2 + v][2]);
        }
    }

    free_parsed_wire(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* Closed triangle polyline: is_closed = 1, 3 vertices. */
static void
test_closed_triangle(prc_context *ctx)
{
    float p[3][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f }
    };
    prc_write_wire_element elem;
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_wire *parsed = NULL;
    int code;

    printf("  sub-case: closed triangle polyline\n");

    memset(&elem, 0, sizeof(elem));
    elem.positions = p[0];
    elem.num_vertices = 3;
    elem.is_closed = 1;
    elem.is_continuous = 0;
    elem.colors = NULL;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_wire_tess(ctx, &w, &elem, 1), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_wire(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->number_of_wire_elements, 1);
    PRC_ASSERT_EQ(parsed->wire_elements[0].number_of_wire_indexes, 3);
    PRC_ASSERT(parsed->wire_elements[0].is_connected & PRC_3DWIRETESSDATA_IsClosing);

    free_parsed_wire(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* 2 elements, 3 vertices each, per-vertex RGBA colors. Colors chosen as
   exact multiples of 1/255 (pure 0.0/1.0 channel values) so the uint8
   round trip is unambiguous. */
static void
test_per_vertex_colors(prc_context *ctx)
{
    float p[6][3] = {
        { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 0.0f },
        { 5.0f, 0.0f, 0.0f }, { 6.0f, 0.0f, 0.0f }, { 6.0f, 1.0f, 0.0f }
    };
    float col[6][4] = {
        { 1.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f },
        { 1.0f, 1.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 1.0f, 1.0f }
    };
    prc_write_wire_element elems[2];
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_wire *parsed = NULL;
    uint32_t e, v;
    int code;
    prc_rgb_color decoded[6];

    printf("  sub-case: per-vertex colors\n");

    memset(elems, 0, sizeof(elems));
    for (e = 0; e < 2; e++)
    {
        elems[e].positions = p[e * 3];
        elems[e].num_vertices = 3;
        elems[e].is_closed = 0;
        elems[e].is_continuous = 0;
        elems[e].colors = col[e * 3];
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_wire_tess(ctx, &w, elems, 2), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_wire(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->has_vertex_colors, 1);
    PRC_ASSERT_EQ(parsed->vertex_color_count, 6);
    PRC_ASSERT(parsed->vertex_color_data.is_rgba);
    PRC_ASSERT(!parsed->vertex_color_data.b_optimized);

    /* Reconstruct the sequential 6-entry color list: first_vertex, then
       remaining_vertices[0..4] (never is_same, since the encoder always
       writes fresh values). */
    decoded[0] = parsed->vertex_color_data.color_data.first_vertex;
    for (v = 1; v < 6; v++)
    {
        PRC_ASSERT(!parsed->vertex_color_data.color_data.remaining_vertices[v - 1].is_same);
        decoded[v] = parsed->vertex_color_data.color_data.remaining_vertices[v - 1].color;
    }

    for (v = 0; v < 6; v++)
    {
        PRC_ASSERT_NEAR(decoded[v].red / 255.0, col[v][0], 1.0 / 255.0);
        PRC_ASSERT_NEAR(decoded[v].green / 255.0, col[v][1], 1.0 / 255.0);
        PRC_ASSERT_NEAR(decoded[v].blue / 255.0, col[v][2], 1.0 / 255.0);
        PRC_ASSERT_NEAR(decoded[v].alpha / 255.0, col[v][3], 1.0 / 255.0);
    }

    free_parsed_wire(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* Table 143's is_segment_color: one colour per SEGMENT, not per point.

   The count is the whole risk. The array is a delta run with no terminator
   and the reader derives its length rather than being told it --
   segment_color_count = vertex_color_count - number_of_wire_elements -- so
   writing the per-point count with the segment flag set would not fail here,
   it would leave the reader finishing in the wrong place with everything
   after it noise.

   Two elements are used, one open and one closed, because they differ: an
   open run of 3 points has 2 segments, a closed one has 3. A test with only
   open elements, or only closed, would pass against a writer that got the
   closed case backwards. */
static void
test_per_segment_colors(prc_context *ctx)
{
    /* float, not double: prc_api_write_wire_element.positions is a
       `const float *`. Declaring these as double made the writer read 18
       floats out of a 48-byte array -- 24 bytes past the end -- and the
       positions it encoded were reinterpreted double bit patterns. The case
       still passed, because everything it asserts is a colour or a count,
       which is exactly why the overread went unnoticed. */
    static const float p[6][3] = {
        { 0,0,0 }, { 1,0,0 }, { 2,0,0 },        /* open, 3 points -> 2 segments */
        { 0,1,0 }, { 1,1,0 }, { 1,2,0 }         /* closed, 3 points -> 3 segments */
    };
    static const float col[6][4] = {
        { 1,0,0,1 }, { 0,1,0,1 }, { 0,0,1,1 },
        { 1,1,0,1 }, { 1,0,1,1 }, { 0,1,1,1 }
    };
    prc_write_wire_element elems[2];
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_wire *parsed = NULL;
    int code;

    printf("  sub-case: per-segment colors\n");

    memset(elems, 0, sizeof(elems));
    elems[0].positions = p[0];
    elems[0].num_vertices = 3;
    elems[0].is_closed = 0;
    elems[0].colors = col[0];

    elems[1].positions = p[3];
    elems[1].num_vertices = 3;
    elems[1].is_closed = 1;
    elems[1].colors = col[3];

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 256), 0);
    PRC_ASSERT_EQ(prc_write_wire_tess_ex(ctx, &w, elems, 2, 1), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_wire(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    PRC_ASSERT_EQ(parsed->has_vertex_colors, 1);
    PRC_ASSERT(parsed->vertex_color_data.is_segment_color);

    /* vertex_color_count is the POINT count the reader derives: 3 + (3+1 for
       the closing element) = 7. The segment count it then uses is that minus
       one per element, so 5 -- which is 2 + 3, the two elements' segments. */
    PRC_ASSERT_EQ(parsed->vertex_color_count, 7);

    /* The positions, which nothing here used to check. Every assertion in
       this case is about colours or counts, so the array could be -- and was
       -- the wrong type without any of them noticing. */
    {
        uint32_t e, v;

        for (e = 0; e < 2; e++)
        {
            for (v = 0; v < 3; v++)
            {
                uint32_t pool_idx = parsed->wire_elements[e].wire_indexes[v] / 3;
                const double *pos =
                    &parsed->tessellation_coordinates.coordinates[(size_t)pool_idx * 3];

                PRC_ASSERT(pos[0] == (double)p[e * 3 + v][0]);
                PRC_ASSERT(pos[1] == (double)p[e * 3 + v][1]);
                PRC_ASSERT(pos[2] == (double)p[e * 3 + v][2]);
            }
        }
    }

    /* A clean parse proves nothing about the count. Writing too MANY entries
       leaves trailing bits the reader never reads, and it succeeds -- a first
       version of this case asserted only the parse and passed with the writer
       emitting per-point counts under the segment flag.

       So decode the five and compare them. The correct writer skips the last
       point of the open element and uses all three of the closed one, giving
       colours 0,1 then 3,4,5. A per-point writer would give 0,1,2 then 3,4 --
       the third entry differs, and that is what this catches. */
    {
        static const float want[5][4] = {
            { 1,0,0,1 }, { 0,1,0,1 },               /* open: segments 0 and 1 */
            { 1,1,0,1 }, { 1,0,1,1 }, { 0,1,1,1 }   /* closed: all three */
        };
        prc_rgb_color got[5];
        uint32_t k;

        got[0] = parsed->vertex_color_data.color_data.first_vertex;
        for (k = 1; k < 5; k++)
        {
            PRC_ASSERT(!parsed->vertex_color_data.color_data.remaining_vertices[k - 1].is_same);
            got[k] = parsed->vertex_color_data.color_data.remaining_vertices[k - 1].color;
        }
        for (k = 0; k < 5; k++)
        {
            PRC_ASSERT_NEAR(got[k].red   / 255.0, want[k][0], 1.0 / 255.0);
            PRC_ASSERT_NEAR(got[k].green / 255.0, want[k][1], 1.0 / 255.0);
            PRC_ASSERT_NEAR(got[k].blue  / 255.0, want[k][2], 1.0 / 255.0);
            PRC_ASSERT_NEAR(got[k].alpha / 255.0, want[k][3], 1.0 / 255.0);
        }
    }

    free_parsed_wire(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

/* 2 elements sharing one vertex (identical float position): must
   deduplicate to a single entry in tessellation_coordinates. */
static void
test_shared_vertex_dedup(prc_context *ctx)
{
    float shared[3] = { 4.0f, 4.0f, 4.0f };
    float a0[3] = { 0.0f, 0.0f, 0.0f };
    float c0[3] = { 9.0f, 9.0f, 9.0f };
    prc_write_wire_element elems[2];
    float elem0_pts[2][3];
    float elem1_pts[2][3];
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d_wire *parsed = NULL;
    int code;
    uint32_t idx_shared_from_elem0, idx_shared_from_elem1;

    printf("  sub-case: shared vertex deduplicated\n");

    memcpy(elem0_pts[0], a0, sizeof(a0));
    memcpy(elem0_pts[1], shared, sizeof(shared));
    memcpy(elem1_pts[0], shared, sizeof(shared));
    memcpy(elem1_pts[1], c0, sizeof(c0));

    memset(elems, 0, sizeof(elems));
    elems[0].positions = elem0_pts[0];
    elems[0].num_vertices = 2;
    elems[1].positions = elem1_pts[0];
    elems[1].num_vertices = 2;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 128), 0);
    PRC_ASSERT_EQ(prc_write_wire_tess(ctx, &w, elems, 2), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d_wire(ctx, &r, &parsed);
    if (code < 0)
        prc_print_error_stack(ctx);
    PRC_ASSERT_EQ(code, 0);
    PRC_ASSERT_NOT_NULL(parsed);

    /* 3 unique positions (a0, shared, c0), not 4 */
    PRC_ASSERT_EQ(parsed->tessellation_coordinates.number_of_coordinates, 9);

    idx_shared_from_elem0 = parsed->wire_elements[0].wire_indexes[1] / 3;
    idx_shared_from_elem1 = parsed->wire_elements[1].wire_indexes[0] / 3;
    PRC_ASSERT_EQ(idx_shared_from_elem0, idx_shared_from_elem1);

    free_parsed_wire(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("wire tessellation encoder round trip");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_three_line_segments_no_colors(ctx);
    test_closed_triangle(ctx);
    test_per_vertex_colors(ctx);
    test_shared_vertex_dedup(ctx);
    test_per_segment_colors(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
