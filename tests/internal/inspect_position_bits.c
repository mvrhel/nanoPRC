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

   WHAT: Prints the exact raw bit sequence (as a 0/1 string, plus hex bytes)
   nanoPRC's prc_bitwrite_double chose for one specific position's x/y/z
   doubles in a raw .prc file's first PRC_TYPE_TESS_3D entry, for direct
   comparison against a second ("donor") file encoding the SAME decoded
   values. Built for the final step of the Acrobat blank-model-tree
   bisection (see bisect_tess_positions.c and ISO-SPEC/blanktree-matrix-
   evidence-table.md): bit-level bisection narrowed the entire defect down
   to exactly one position (index 3359 of 4900, N=70 mustcalc grid) whose
   nanoPRC-encoded bits fail in Acrobat and whose libPRC-encoded bits (for
   the identical decoded value) work -- this tool shows precisely how the
   two encoders' bit choices for that one position differ.

   HOW: usage: inspect_position_bits.exe <file.prc> <position_index> */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "zlib.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

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

static void
print_bits(prc_bit_state *state_src, uint8_t *base_ptr, int64_t total_bits, int64_t start, int64_t nbits)
{
    prc_context *ctx = NULL;
    prc_bit_state tmp;
    int64_t i;
    seek_bits(&tmp, base_ptr, total_bits, start);
    printf("    %lld bits: ", (long long)nbits);
    for (i = 0; i < nbits; i++)
    {
        uint8_t bit = prc_bitread_bit(ctx, &tmp);
        putchar(bit ? '1' : '0');
        if (i % 8 == 7) putchar(' ');
    }
    printf("\n    hex: ");
    seek_bits(&tmp, base_ptr, total_bits, start);
    {
        int64_t remaining = nbits;
        while (remaining > 0)
        {
            uint8_t byte = 0;
            int b;
            int take = remaining < 8 ? (int)remaining : 8;
            for (b = 0; b < take; b++)
            {
                uint8_t bit = prc_bitread_bit(ctx, &tmp);
                byte = (uint8_t)(byte | (bit << (7 - b)));
            }
            printf("%02X ", byte);
            remaining -= take;
        }
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf, *tess_inflated;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count, i;
    uint32_t *section_offsets;
    uLongf dest_len;
    uint32_t comp_len;
    prc_bit_state state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_content_prc_base wrapper_base;
    uint8_t is_calc;
    uint32_t num_coords;
    uint32_t target_pos;
    uint32_t k, c;

    if (argc < 3) { printf("usage: %s <file.prc> <position_index>\n", argv[0]); return 2; }
    target_pos = (uint32_t)strtoul(argv[2], NULL, 10);

    ctx = prc_api_new_context(NULL);

    fid = fopen(argv[1], "rb");
    if (fid == NULL) { printf("failed to open %s\n", argv[1]); return 1; }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize) { printf("short read\n"); return 1; }
    if (memcmp(buf, "PRC", 3) != 0) { printf("not a raw PRC file\n"); return 1; }

    p = buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("only single-file-structure inputs supported\n"); return 1; }
    p += 4; p += 16; p += 4;
    section_count = read_le_u32(p); p += 4;
    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }

    comp_len = section_offsets[4] - section_offsets[3];
    dest_len = comp_len * 40u + 4096u;
    tess_inflated = (uint8_t *)malloc(dest_len);
    for (;;)
    {
        uLongf try_len = dest_len;
        int zret = uncompress(tess_inflated, &try_len, buf + section_offsets[3], comp_len);
        if (zret == Z_OK) { dest_len = try_len; break; }
        if (zret == Z_BUF_ERROR) { dest_len *= 2; free(tess_inflated); tess_inflated = (uint8_t *)malloc(dest_len); continue; }
        printf("zlib uncompress failed: %d\n", zret);
        return 1;
    }

    prc_init_bit_state(ctx, &state, tess_inflated, (size_t)dest_len);

    wrapper_tag = prc_bitread_uint32(ctx, &state);
    memset(&wrapper_base, 0, sizeof(wrapper_base));
    prc_parse_content_prc_base(ctx, &state, &wrapper_base);
    tess_count = prc_bitread_uint32(ctx, &state);
    entry_tag = prc_bitread_uint32(ctx, &state);
    (void)wrapper_tag; (void)tess_count; (void)entry_tag;

    is_calc = prc_bitread_bit(ctx, &state);
    num_coords = prc_bitread_uint32(ctx, &state);
    (void)is_calc;

    printf("file=%s num_positions=%u target=%u\n", argv[1], num_coords / 3, target_pos);

    for (k = 0; k < num_coords / 3; k++)
    {
        for (c = 0; c < 3; c++)
        {
            int64_t before = state.bit_position;
            double v = prc_bitread_double(ctx, &state);
            int64_t after = state.bit_position;
            if (k == target_pos)
            {
                const char *label = (c == 0) ? "x" : (c == 1) ? "y" : "z";
                printf("  %s = %.17g\n", label, v);
                print_bits(&state, tess_inflated, (int64_t)dest_len * 8, before, after - before);
            }
        }
        if (k == target_pos) break;
    }

    prc_api_release_context(ctx);
    return 0;
}
