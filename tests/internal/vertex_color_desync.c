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

/* INTERNAL DEVELOPMENT TOOL -- not registered with CTest.

   WHAT: Hand-assembles one PRC_TYPE_TESS_3D record carrying a per-face
   vertex-colour array, parses it back with the real prc_parse_tess_3d, and
   reports whether the parser consumed the whole record or stopped short.

   usage: vertex_color_desync            (runs all three fixtures)

   WHY IT EXISTS: the vertex-colour array in Table 143 "VertexColors" stores
   no element count. It is delta-encoded -- the first colour in full, then one
   is_same bit per remaining entry and a full colour only when that bit is
   clear -- and nothing in the bitstream says how many entries there are. Both
   sides have to DERIVE the count from something else, and the specification
   never says from what. nanoPRC derives it at prc_parse_tess.c:

       code = prc_parse_vertexcolors(ctx, bit_state, &data->vertex_colors,
           data->triangulateddata[0] * 3, true);

   with a TODO of our own sitting next to it: "This could be an issue if we
   have strips or fans or multiple object types".

   That derivation is right for a face built from plain triangles alone, where
   triangulateddata[0] is the triangle count and every triangle contributes
   three vertex references. It is wrong the moment a face carries a second
   entity type, because triangulateddata[0] then describes only the FIRST
   group and the vertices belonging to the fans or strips after it are never
   counted. The parser stops early, the remaining colours are left unread, and
   every field after the colour block is read from the wrong bit offset.

   WHY IT IS HAND-ASSEMBLED RATHER THAN WRITTEN: nanoPRC's own encoder cannot
   produce this input. prc_write_tess_3d writes has_vertex_colors as a hard 0
   (prc_write_tess_3d.c, in the per-face record) and emits a single entity type
   per face, so neither half of the fixture is reachable through the writer.
   Assembling the bytes directly is also the more faithful choice: the pattern
   being reproduced is another implementation's, and writing it through our own
   encoder would bake in our reading of the very ambiguity under test.

   WHY IT IS SYNTHETIC RATHER THAN A CORPUS FILE: across the 310-file public
   prc-db corpus, all 674 coloured faces are plain Triangle with a single size
   entry. There is no real file that exercises the mixed-entity case, which is
   why the fragility has stayed theoretical and why a fixture has to be built
   to reach it.

   READING THE OUTPUT: the control fixture is a plain-triangle face, where the
   derivation is correct and the parser should land exactly on the end of the
   record. The two mixed fixtures add a triangle fan and a triangle strip to
   that same face -- both, because the TODO quoted above claims the problem for
   "strips or fans or multiple object types", and one example would leave the
   other two as assertions. They miscount identically, which is the evidence
   that the cause is the presence of a second group rather than anything
   particular to either type.

   Exit status is 0 if every fixture consumes its whole record and 1 if any
   stops short, so once the derivation is fixed this becomes a ready-made
   regression check and can be promoted into tests/unit unchanged. Today it
   exits 1. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "prc_context.h"
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"

#define NUM_POSITIONS 16

/* 14 plain triangles + one 8-index fan = 20 triangles, 50 vertex references.
   The fan's 8 indices describe 6 triangles (an n-index fan is n-2 triangles),
   so 14 + 6 is the 20 the fixture claims. */
#define PLAIN_TRIS      14
#define SECOND_INDICES   8
#define PLAIN_REFS      (PLAIN_TRIS * 3)                /* 42 */
#define MIXED_REFS      (PLAIN_REFS + SECOND_INDICES)   /* 50 */

/* Which entity type follows the plain triangles in the same face. A fan and a
   strip are laid out identically in triangulateddata -- a group count, then one
   index count per group -- and both describe n-2 triangles from n indices, so
   the two mixed fixtures differ only in the used_entities_flag bit. That is the
   point of running both: it shows the miscount follows from there being a
   second group at all, not from anything particular to fans. */
#define GROUP_NONE   0
#define GROUP_FAN    1
#define GROUP_STRIP  2

/* Write the colour block of Table 143 for `count` entries, as a face-level
   array (is_face TRUE, so no is_segment_color bit). Every entry after the
   first is written as a distinct colour rather than a repeat: an is_same run
   would cost one bit each and make an undercount hard to see, whereas a full
   entry costs 1 + 24 bits, so a shortfall shows up as a large, unmistakable
   number of unread bits rather than a handful. */
static int
emit_vertex_colors(prc_context *ctx, prc_bit_write_state *w, uint32_t count)
{
    uint32_t k;

    if (prc_bitwrite_bit(ctx, w, 0) != 0) return -1;   /* is_rgba = 0, RGB only */
    if (prc_bitwrite_bit(ctx, w, 0) != 0) return -1;   /* b_optimized = 0 */

    /* first_vertex, in full */
    if (prc_bitwrite_uint8(ctx, w, 10) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, w, 20) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, w, 30) != 0) return -1;

    for (k = 1; k < count; k++)
    {
        if (prc_bitwrite_bit(ctx, w, 0) != 0) return -1;  /* is_same = 0 */
        if (prc_bitwrite_uint8(ctx, w, (uint8_t)(k & 0xFF)) != 0) return -1;
        if (prc_bitwrite_uint8(ctx, w, (uint8_t)((k * 3) & 0xFF)) != 0) return -1;
        if (prc_bitwrite_uint8(ctx, w, (uint8_t)((k * 7) & 0xFF)) != 0) return -1;
    }
    return 0;
}

/* Assemble one TESS_3D record. `second_group` selects what follows the plain
   triangles in the face: nothing, a triangle fan, or a triangle strip.
   Field order follows prc_write_tess_3d.c exactly -- this is the same wire
   format, just reachable by hand. */
static int
emit_tess_3d(prc_context *ctx, prc_bit_write_state *w, int second_group,
             uint32_t *colors_written_out, uint32_t *derived_count_out)
{
    uint32_t refs = (second_group == GROUP_NONE) ? PLAIN_REFS : MIXED_REFS;
    uint32_t tri_data[3];
    uint32_t tri_data_size;
    uint32_t used_entities;
    uint32_t i;

    if (second_group != GROUP_NONE)
    {
        /* [ plain triangle count, group count, group 0 index count ]. The
           parser reads the groups in flag order -- Triangle, then TriangleFan,
           then TriangleStripe -- so a fan and a strip occupy the same slots. */
        tri_data[0] = PLAIN_TRIS;
        tri_data[1] = 1;
        tri_data[2] = SECOND_INDICES;
        tri_data_size = 3;
        used_entities = PRC_FACETESSDATA_Triangle |
            ((second_group == GROUP_FAN) ? PRC_FACETESSDATA_TriangleFan
                                         : PRC_FACETESSDATA_TriangleStripe);
    }
    else
    {
        tri_data[0] = PLAIN_TRIS;
        tri_data_size = 1;
        used_entities = PRC_FACETESSDATA_Triangle;
    }

    *colors_written_out = refs;
    *derived_count_out = tri_data[0] * 3;   /* what prc_parse_tess.c will compute */

    if (prc_bitwrite_bit(ctx, w, 0) != 0) return -1;                    /* is_calculated */

    if (prc_bitwrite_uint32(ctx, w, NUM_POSITIONS * 3) != 0) return -1; /* number_of_coordinates */
    for (i = 0; i < NUM_POSITIONS * 3; i++)
        if (prc_bitwrite_double(ctx, w, (double)i * 0.25) != 0) return -1;

    if (prc_bitwrite_bit(ctx, w, 1) != 0) return -1;                    /* has_faces */
    if (prc_bitwrite_bit(ctx, w, 0) != 0) return -1;                    /* has_loops */

    /* must_calculate_normals: keeps the index array to bare position indices
       (no interleaved normal index per vertex), which is both simpler to
       assemble and the layout the real libPRC-written file uses. */
    if (prc_bitwrite_bit(ctx, w, 1) != 0) return -1;                    /* must_calculate_normals */
    if (prc_bitwrite_uint8(ctx, w, 0) != 0) return -1;                  /* normal_recalculation_flags */
    if (prc_bitwrite_double(ctx, w, 30.0) != 0) return -1;              /* crease_angle */
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* number_of_normal_coordinates */

    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* number_of_wire_indices */

    if (prc_bitwrite_uint32(ctx, w, refs) != 0) return -1;              /* number_of_triangulated_indicies */
    for (i = 0; i < refs; i++)
        if (prc_bitwrite_uint32(ctx, w, (i % NUM_POSITIONS) * 3) != 0) return -1;

    if (prc_bitwrite_uint32(ctx, w, 1) != 0) return -1;                 /* number_of_face_tessellation */

    if (prc_bitwrite_uint32(ctx, w, PRC_TYPE_TESS_Face) != 0) return -1;
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* size_of_line_attributes */
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* start_of_wire_data */
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* size_of_sizes_wire */
    if (prc_bitwrite_uint32(ctx, w, used_entities) != 0) return -1;     /* used_entities_flag */
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* start_triangulated */
    if (prc_bitwrite_uint32(ctx, w, tri_data_size) != 0) return -1;     /* size_of_triangulateddata */
    for (i = 0; i < tri_data_size; i++)
        if (prc_bitwrite_uint32(ctx, w, tri_data[i]) != 0) return -1;
    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* number_of_textured_coordinate_indexes */

    if (prc_bitwrite_bit(ctx, w, 1) != 0) return -1;                    /* has_vertex_colors */
    if (emit_vertex_colors(ctx, w, refs) != 0) return -1;

    if (prc_bitwrite_uint32(ctx, w, 0) != 0) return -1;                 /* number_of_texture_coordinates */

    return 0;
}

static void
release_parsed(prc_context *ctx, prc_tess_3d *d)
{
    if (d == NULL)
        return;
    /* Deliberately minimal: this tool exists to measure a bit offset, and a
       desynced parse leaves the structure in a shape the normal release path
       reasons about differently. Run it under the leak checker only once the
       derivation is fixed and the parse actually completes. */
    (void)ctx;
    (void)d;
}

static int
run_fixture(prc_context *ctx, int second_group, const char *label)
{
    prc_bit_write_state w;
    prc_bit_state r;
    prc_tess_3d *parsed = NULL;
    uint32_t colors_written = 0, derived = 0;
    int64_t total_bits, consumed_bits;
    int code;
    int short_read;

    printf("--- %s ---\n", label);

    if (prc_bitwrite_init(ctx, &w, 4096) != 0)
    {
        printf("  could not initialise the write state\n");
        return 1;
    }
    if (emit_tess_3d(ctx, &w, second_group, &colors_written, &derived) != 0)
    {
        printf("  could not assemble the fixture\n");
        prc_bitwrite_release(ctx, &w);
        return 1;
    }
    /* Captured BEFORE the flush. prc_bitwrite_flush pads the final partial
       byte, so w.byte_pos * 8 afterwards overstates the record by up to seven
       bits. Comparing against the padded figure reported a phantom 3-bit
       desync on the control fixture -- exactly the kind of harness artefact
       that would have discredited the real finding on the mixed one. */
    total_bits = (int64_t)w.byte_pos * 8 + (int64_t)w.bit_fill;

    if (prc_bitwrite_flush(ctx, &w) != 0)
    {
        printf("  could not flush the fixture\n");
        prc_bitwrite_release(ctx, &w);
        return 1;
    }


    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    code = prc_parse_tess_3d(ctx, &r, &parsed);
    consumed_bits = r.bit_position;

    printf("  triangles                 : %u (%u plain%s)\n",
           (second_group == GROUP_NONE) ? PLAIN_TRIS
                                        : (PLAIN_TRIS + SECOND_INDICES - 2),
           PLAIN_TRIS,
           (second_group == GROUP_NONE) ? "" :
           (second_group == GROUP_FAN)  ? " + one 8-index fan"
                                        : " + one 8-index strip");
    printf("  vertex references in face : %u\n", colors_written);
    printf("  colours written           : %u\n", colors_written);
    printf("  colours nanoPRC derives   : %u   (triangulateddata[0] * 3)\n", derived);
    printf("  parse return code         : %d\n", code);
    printf("  record size               : %lld bits\n", (long long)total_bits);
    printf("  parser consumed           : %lld bits\n", (long long)consumed_bits);

    short_read = (consumed_bits != total_bits);
    if (short_read)
        printf("  RESULT                    : DESYNC, %lld bits unread "
               "(%u colours never consumed)\n",
               (long long)(total_bits - consumed_bits),
               colors_written - derived);
    else
        printf("  RESULT                    : clean, whole record consumed\n");

    release_parsed(ctx, parsed);
    prc_bitwrite_release(ctx, &w);
    printf("\n");
    return short_read ? 1 : 0;
}

int
main(void)
{
    prc_context *ctx;
    int bad = 0;

    ctx = prc_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "could not create a prc_context\n");
        return 2;
    }

    printf("vertex-colour count derivation, Table 143 VertexColors\n\n");

    bad |= run_fixture(ctx, GROUP_NONE,  "control: face of plain triangles only");
    bad |= run_fixture(ctx, GROUP_FAN,   "mixed: same face plus a triangle fan");
    bad |= run_fixture(ctx, GROUP_STRIP, "mixed: same face plus a triangle strip");

    if (bad)
        printf("At least one fixture desynced. The derivation at "
               "prc_parse_tess.c (triangulateddata[0] * 3) does not account "
               "for every entity type in the face.\n");
    else
        printf("Both fixtures consumed their whole record.\n");

    prc_release_context(ctx);
    return bad;
}
