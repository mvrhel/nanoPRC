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

   WHAT: A bit-level bisection splicer for the Acrobat blank-model-tree
   investigation's tessellation-section content, after the section-swap
   splice test (splice_tess_section.py, rows 28a/28b) proved the defect
   lives entirely inside nanoPRC's raw tessellation bitstream, and after
   exhaustive field-level/algorithm-level/value-level testing (rows 26-33,
   see ISO-SPEC/blanktree-matrix-evidence-table.md) found no single named
   difference responsible. This tool builds a fresh tessellation section
   for the SAME geometry where the first K positions' raw, bit-exact
   encoding is copied verbatim from a "donor" file (e.g. libPRC's own,
   Acrobat-confirmed-working output) and the remaining positions are
   re-encoded via nanoPRC's own prc_bitwrite_double (deterministic, so
   bit-identical to what nanoPRC's original writer produced) -- everything
   else (index array, face records, wrapper fields) is reconstructed
   unchanged from the "base" (nanoPRC) file. Varying K lets a human bisect
   which specific position(s), if any, are responsible for Acrobat's
   failure, by testing successive splits in Acrobat.

   HOW IT WORKS: both input files' tessellation sections are inflated: the
   base (nanoPRC) file is read sequentially start-to-finish (advancing
   normally through every field so the reconstructed output matches it
   field-for-field beyond the bisected positions); the donor file is only
   consulted for its per-position bit ranges (recorded via bit_position
   before/after a normal prc_bitread_double call), then re-extracted with a
   from-scratch seek + bit-by-bit copy (prc_bit_state has no seek of its
   own, so this tool implements one locally) so the donor's EXACT chosen
   bits -- not just its decoded value -- land in the output for the
   positions selected.

   LIMITATIONS: assumes a single PRC_TYPE_TESS_3D entry, no wire indices
   worth iterating (any count is still copied through), no vertex colors
   (errors out if encountered -- not used by any file in this
   investigation), and that both input files' tessellation entries have
   the same number_of_coordinates (checked; the two grid geometries this
   investigation compares always do).

   HOW: usage: bisect_tess_positions.exe <base.prc> <donor.prc>
   <out_tess_section.bin> <K> -- K = number of LEADING positions (0-based,
   3 coordinates each) to source from donor; positions [K..N) come from
   base. Output is a raw deflated byte blob -- splice it into base's own
   file in place of its tessellation section (e.g. via
   scratchpad/splice_tess_section.py, treating this tool's output as a
   plain byte file donor instead of extracting from a second .prc). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "prc_write_file_structure.h"
#include "zlib.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* Seeks `state` to an absolute bit position within the buffer it was
   originally initialized from (base_ptr, total_bits). prc_bit_state has no
   seek of its own (it is purely sequential), so this recomputes ptr/
   bitmask directly per prc_init_bit_state/prc_next_bit's MSB-first
   convention (bitmask starts at 0x80, shifts right per bit, wraps to the
   next byte at 0). */
static void
seek_bits(prc_bit_state *state, uint8_t *base_ptr, int64_t total_bits, int64_t bit_position)
{
    state->ptr = base_ptr + (bit_position / 8);
    state->bitmask = (uint8_t)(0x80 >> (bit_position % 8));
    state->bit_count = total_bits - bit_position;
    state->bit_position = bit_position;
    state->overrun = 0;
    state->curr_name = NULL;
}

static uint8_t *
inflate_tess_section(prc_context *ctx, const char *path, size_t *out_len)
{
    FILE *fid;
    long fsize;
    uint8_t *buf, *tess_inflated = NULL;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count, i;
    uint32_t *section_offsets;
    uLongf dest_len;
    uint32_t comp_len;

    fid = fopen(path, "rb");
    if (fid == NULL) { printf("failed to open %s\n", path); exit(1); }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize) { printf("short read on %s\n", path); exit(1); }
    if (memcmp(buf, "PRC", 3) != 0) { printf("%s: not a raw PRC file\n", path); exit(1); }

    p = buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("%s: only single-file-structure inputs supported\n", path); exit(1); }
    p += 4;
    p += 16;
    p += 4;
    section_count = read_le_u32(p); p += 4;
    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
    if (section_count < 5) { printf("%s: unexpected section_count=%u\n", path, section_count); exit(1); }

    comp_len = section_offsets[4] - section_offsets[3];
    dest_len = comp_len * 40u + 4096u;
    tess_inflated = (uint8_t *)malloc(dest_len);
    for (;;)
    {
        uLongf try_len = dest_len;
        int zret = uncompress(tess_inflated, &try_len, buf + section_offsets[3], comp_len);
        if (zret == Z_OK) { dest_len = try_len; break; }
        if (zret == Z_BUF_ERROR) { dest_len *= 2; free(tess_inflated); tess_inflated = (uint8_t *)malloc(dest_len); continue; }
        printf("%s: zlib uncompress failed: %d\n", path, zret);
        exit(1);
    }
    free(section_offsets);
    free(buf);
    (void)ctx;
    *out_len = (size_t)dest_len;
    return tess_inflated;
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    uint8_t *buf_a, *buf_b;
    size_t len_a, len_b;
    uint32_t K;
    prc_bit_state state_a, state_b;
    prc_bit_write_state out;
    uint32_t wrapper_tag, tess_count, entry_tag;
    uint32_t wrapper_tag_b, tess_count_b, entry_tag_b;
    prc_content_prc_base wrapper_base_a, wrapper_base_b;
    uint8_t is_calc;
    uint32_t num_coords, num_coords_b;
    uint32_t num_positions, k, c, f, i;
    uint8_t has_faces, has_loops, must_calc;
    uint8_t *comp;
    size_t comp_len;
    FILE *fo;

    if (argc < 5)
    {
        printf("usage: %s <base.prc> <donor.prc> <out_tess_section.bin> <K>\n", argv[0]);
        return 2;
    }
    K = (uint32_t)strtoul(argv[4], NULL, 10);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    buf_a = inflate_tess_section(ctx, argv[1], &len_a);
    buf_b = inflate_tess_section(ctx, argv[2], &len_b);

    prc_init_bit_state(ctx, &state_a, buf_a, len_a);
    prc_init_bit_state(ctx, &state_b, buf_b, len_b);

    if (prc_bitwrite_init(ctx, &out, 1u << 20) != 0) { printf("bitwrite_init failed\n"); return 1; }

    /* --- wrapper: PRC_TYPE_ASM_FileStructureTessellation + ContentPRCBase + tess_count + entry tag --- */
    wrapper_tag = prc_bitread_uint32(ctx, &state_a);
    memset(&wrapper_base_a, 0, sizeof(wrapper_base_a));
    prc_parse_content_prc_base(ctx, &state_a, &wrapper_base_a);
    tess_count = prc_bitread_uint32(ctx, &state_a);
    entry_tag = prc_bitread_uint32(ctx, &state_a);

    wrapper_tag_b = prc_bitread_uint32(ctx, &state_b);
    memset(&wrapper_base_b, 0, sizeof(wrapper_base_b));
    prc_parse_content_prc_base(ctx, &state_b, &wrapper_base_b);
    tess_count_b = prc_bitread_uint32(ctx, &state_b);
    entry_tag_b = prc_bitread_uint32(ctx, &state_b);
    (void)wrapper_tag_b; (void)tess_count_b; (void)entry_tag_b;

    prc_bitwrite_uint32(ctx, &out, wrapper_tag);
    prc_bitwrite_uint32(ctx, &out, 0);   /* base.attribute_count */
    prc_bitwrite_bit(ctx, &out, 1);      /* base.name.same */
    prc_bitwrite_uint32(ctx, &out, tess_count);
    prc_bitwrite_uint32(ctx, &out, entry_tag);

    /* --- is_calculated + number_of_coordinates --- */
    is_calc = prc_bitread_bit(ctx, &state_a);
    num_coords = prc_bitread_uint32(ctx, &state_a);
    prc_bitread_bit(ctx, &state_b);
    num_coords_b = prc_bitread_uint32(ctx, &state_b);
    if (num_coords != num_coords_b)
    {
        printf("MISMATCH: base has %u coordinates, donor has %u -- refusing to bisect\n", num_coords, num_coords_b);
        return 1;
    }
    prc_bitwrite_bit(ctx, &out, is_calc);
    prc_bitwrite_uint32(ctx, &out, num_coords);

    num_positions = num_coords / 3;
    for (k = 0; k < num_positions; k++)
    {
        for (c = 0; c < 3; c++)
        {
            double va;
            int64_t b_before, b_after, nbits, bi;

            va = prc_bitread_double(ctx, &state_a);
            b_before = state_b.bit_position;
            prc_bitread_double(ctx, &state_b);
            b_after = state_b.bit_position;

            if (k < K)
            {
                prc_bit_state tmp;
                nbits = b_after - b_before;
                seek_bits(&tmp, buf_b, (int64_t)len_b * 8, b_before);
                for (bi = 0; bi < nbits; bi++)
                {
                    uint8_t bit = prc_bitread_bit(ctx, &tmp);
                    prc_bitwrite_bit(ctx, &out, bit);
                }
            }
            else
            {
                /* Re-encode base's own decoded value via nanoPRC's own
                   writer -- deterministic, reproduces base's original bits
                   exactly since it's the same function applied to the
                   same value. */
                prc_bitwrite_double(ctx, &out, va);
            }
        }
    }

    /* --- has_faces / has_loops / must_calculate_normals [+ recalc flags + crease_angle] --- */
    has_faces = prc_bitread_bit(ctx, &state_a);
    has_loops = prc_bitread_bit(ctx, &state_a);
    must_calc = prc_bitread_bit(ctx, &state_a);
    prc_bitwrite_bit(ctx, &out, has_faces);
    prc_bitwrite_bit(ctx, &out, has_loops);
    prc_bitwrite_bit(ctx, &out, must_calc);
    if (must_calc)
    {
        uint8_t recalc_flags = prc_bitread_uint8(ctx, &state_a);
        double crease = prc_bitread_double(ctx, &state_a);
        prc_bitwrite_uint8(ctx, &out, recalc_flags);
        prc_bitwrite_double(ctx, &out, crease);
    }

    /* --- number_of_normal_coordinates [+ normal_coordinates] --- */
    {
        uint32_t n = prc_bitread_uint32(ctx, &state_a);
        prc_bitwrite_uint32(ctx, &out, n);
        for (i = 0; i < n; i++)
        {
            double v = prc_bitread_double(ctx, &state_a);
            prc_bitwrite_double(ctx, &out, v);
        }
    }

    /* --- number_of_wire_indices [+ wire_indices] --- */
    {
        uint32_t n = prc_bitread_uint32(ctx, &state_a);
        prc_bitwrite_uint32(ctx, &out, n);
        for (i = 0; i < n; i++)
        {
            uint32_t v = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, v);
        }
    }

    /* --- number_of_triangulated_indicies [+ triangulated_index_array] --- */
    {
        uint32_t n = prc_bitread_uint32(ctx, &state_a);
        prc_bitwrite_uint32(ctx, &out, n);
        for (i = 0; i < n; i++)
        {
            uint32_t v = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, v);
        }
    }

    /* --- number_of_face_tessellation [+ face records] --- */
    {
        uint32_t nf = prc_bitread_uint32(ctx, &state_a);
        prc_bitwrite_uint32(ctx, &out, nf);
        for (f = 0; f < nf; f++)
        {
            uint32_t tag = prc_bitread_uint32(ctx, &state_a);
            uint32_t size_line_attr, start_wire, size_sizes_wire, used_entities, start_tri, size_tri_data, num_tex_idx;
            uint8_t has_vc;

            prc_bitwrite_uint32(ctx, &out, tag);

            size_line_attr = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, size_line_attr);
            for (i = 0; i < size_line_attr; i++)
            {
                uint32_t v = prc_bitread_uint32(ctx, &state_a);
                prc_bitwrite_uint32(ctx, &out, v);
            }

            start_wire = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, start_wire);

            size_sizes_wire = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, size_sizes_wire);
            for (i = 0; i < size_sizes_wire; i++)
            {
                uint32_t v = prc_bitread_uint32(ctx, &state_a);
                prc_bitwrite_uint32(ctx, &out, v);
            }

            used_entities = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, used_entities);

            start_tri = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, start_tri);

            size_tri_data = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, size_tri_data);
            for (i = 0; i < size_tri_data; i++)
            {
                uint32_t v = prc_bitread_uint32(ctx, &state_a);
                prc_bitwrite_uint32(ctx, &out, v);
            }

            num_tex_idx = prc_bitread_uint32(ctx, &state_a);
            prc_bitwrite_uint32(ctx, &out, num_tex_idx);

            has_vc = prc_bitread_bit(ctx, &state_a);
            prc_bitwrite_bit(ctx, &out, has_vc);
            if (has_vc)
            {
                printf("has_vertex_colors not supported by this tool (not used by any file in this investigation)\n");
                return 1;
            }

            if (size_line_attr > 0)
            {
                uint32_t behavior = prc_bitread_uint32(ctx, &state_a);
                prc_bitwrite_uint32(ctx, &out, behavior);
            }
        }
    }

    /* --- number_of_texture_coordinates [+ texture_coordinates] --- */
    {
        uint32_t n = prc_bitread_uint32(ctx, &state_a);
        prc_bitwrite_uint32(ctx, &out, n);
        for (i = 0; i < n; i++)
        {
            double v = prc_bitread_double(ctx, &state_a);
            prc_bitwrite_double(ctx, &out, v);
        }
    }

    if (prc_bitwrite_flush(ctx, &out) != 0) { printf("bitwrite_flush failed\n"); return 1; }

    if (prc_write_deflate(ctx, out.buf, out.byte_pos, &comp, &comp_len) != 0)
    {
        printf("deflate failed\n");
        return 1;
    }

    fo = fopen(argv[3], "wb");
    if (fo == NULL || fwrite(comp, 1, comp_len, fo) != comp_len)
    {
        printf("failed to write %s\n", argv[3]);
        return 1;
    }
    fclose(fo);
    printf("wrote %s: raw=%zu compressed=%zu (positions [0,%u) from donor, [%u,%u) from base, of %u total)\n",
        argv[3], out.byte_pos, comp_len, K, K, num_positions, num_positions);

    free(buf_a);
    free(buf_b);
    prc_free(ctx, comp);
    prc_bitwrite_release(ctx, &out);
    prc_api_release_context(ctx);
    return 0;
}
