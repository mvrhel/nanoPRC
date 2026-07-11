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

/* Write-path counterpart to prc_bit.c. Nothing in prc_bit.c (prc_bit_state,
   any prc_bitread_* function) is modified; this file is purely additive.
   Every prc_bitwrite_X function is written to produce exactly the bit
   pattern its prc_bitread_X counterpart in prc_bit.c consumes -- each
   function below says which read function it pairs with and where. */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_double.h"

/* ------------------------------------------------------------------ */
/* Core buffer / bit-level primitives                                  */
/* ------------------------------------------------------------------ */

static int
prc_bitwrite_ensure_capacity(prc_context *ctx, prc_bit_write_state *state, size_t needed)
{
    size_t new_capacity;
    uint8_t *new_buf;

    if (needed <= state->capacity)
        return 0;

    new_capacity = state->capacity ? state->capacity : 64;
    while (new_capacity < needed)
        new_capacity *= 2;

    new_buf = (uint8_t *)prc_realloc(ctx, state->buf, new_capacity);
    if (new_buf == NULL)
    {
        state->error = 1;
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error growing prc_bit_write_state buffer\n");
        return -1;
    }
    state->buf = new_buf;
    state->capacity = new_capacity;
    return 0;
}

int
prc_bitwrite_init(prc_context *ctx, prc_bit_write_state *state, size_t initial_capacity)
{
    if (initial_capacity == 0)
        initial_capacity = 64;

    state->capacity = 0;
    state->byte_pos = 0;
    state->bit_accum = 0;
    state->bit_fill = 0;
    state->error = 0;

    state->buf = (uint8_t *)prc_malloc(ctx, initial_capacity);
    if (state->buf == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitwrite_init\n");
        state->error = 1;
        return -1;
    }
    state->capacity = initial_capacity;
    return 0;
}

void
prc_bitwrite_release(prc_context *ctx, prc_bit_write_state *state)
{
    if (state->buf != NULL)
        prc_free(ctx, state->buf);
    state->buf = NULL;
    state->capacity = 0;
    state->byte_pos = 0;
    state->bit_accum = 0;
    state->bit_fill = 0;
    state->error = 0;
}

/* Core primitive. MSB-first within each byte, mirroring prc_next_bit /
   prc_bitread_bit (prc_bit.c 35-48, 116-142): the first bit written lands
   in the top bit of the byte. */
int
prc_bitwrite_bit(prc_context *ctx, prc_bit_write_state *state, uint8_t bit)
{
    if (state->error)
        return -1;

    if (bit)
        state->bit_accum = (uint8_t)(state->bit_accum | (uint8_t)(1u << (7 - state->bit_fill)));
    state->bit_fill++;

    if (state->bit_fill == 8)
    {
        if (prc_bitwrite_ensure_capacity(ctx, state, state->byte_pos + 1) != 0)
            return -1;
        state->buf[state->byte_pos++] = state->bit_accum;
        state->bit_accum = 0;
        state->bit_fill = 0;
    }
    return 0;
}

int
prc_bitwrite_flush(prc_context *ctx, prc_bit_write_state *state)
{
    if (state->error)
        return -1;
    while (state->bit_fill != 0)
        if (prc_bitwrite_bit(ctx, state, 0) != 0)
            return -1;
    return 0;
}

/* Pairs with prc_bitread_uint8 (prc_bit.c 318-331): 8 bits, MSB first. */
int
prc_bitwrite_uint8(prc_context *ctx, prc_bit_write_state *state, uint8_t val)
{
    int k;
    if (state->error)
        return -1;
    for (k = 7; k >= 0; k--)
        if (prc_bitwrite_bit(ctx, state, (uint8_t)((val >> k) & 1)) != 0)
            return -1;
    return 0;
}

/* Pairs with prc_bitread_uint32 (prc_bit.c 333-344): a variable number of
   little-endian bytes, each preceded by a "one more byte follows"
   continuation bit, terminated by a continuation bit of 0. Uses the
   minimal number of bytes that reproduces val exactly (0 bytes for
   val == 0), which the decoder's zero-initialized accumulator handles
   correctly since it never touches bits beyond what was read. */
int
prc_bitwrite_uint32(prc_context *ctx, prc_bit_write_state *state, uint32_t val)
{
    uint32_t nbytes;
    uint32_t pos;
    uint32_t tmp;

    if (state->error)
        return -1;

    nbytes = 0;
    tmp = val;
    while (tmp != 0)
    {
        nbytes++;
        tmp >>= 8;
    }

    for (pos = 0; pos < nbytes; pos++)
    {
        if (prc_bitwrite_bit(ctx, state, 1) != 0)
            return -1;
        if (prc_bitwrite_uint8(ctx, state, (uint8_t)((val >> (8 * pos)) & 0xFF)) != 0)
            return -1;
    }
    return prc_bitwrite_bit(ctx, state, 0);
}

/* Pairs with prc_bitread_int32 (prc_bit.c 346-360). Despite the different
   return type, prc_bitread_int32 runs the exact same variable-length loop
   as prc_bitread_uint32 and builds its result in a uint32_t; the trailing
   "val <<= (4-pos)*8; val >>= (4-pos)*8;" is a no-op given how val is built
   (the shifted-out/shifted-in bits are already 0 on both sides), so the
   reader ends up returning the raw accumulated bit pattern reinterpreted as
   int32_t. Reusing prc_bitwrite_uint32 on the same bit pattern reproduces
   that exactly. */
int
prc_bitwrite_int32(prc_context *ctx, prc_bit_write_state *state, int32_t val)
{
    uint32_t bits;
    if (state->error)
        return -1;
    memcpy(&bits, &val, sizeof(bits));
    return prc_bitwrite_uint32(ctx, state, bits);
}

/* Pairs with prc_bitread_float (prc_bit.c 414-425): 4 raw bytes, low byte
   first. */
int
prc_bitwrite_float(prc_context *ctx, prc_bit_write_state *state, float val)
{
    union float_uint u;
    if (state->error)
        return -1;
    u.float_val = val;
    if (prc_bitwrite_uint8(ctx, state, (uint8_t)(u.uint_val & 0xFF)) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, state, (uint8_t)((u.uint_val >> 8) & 0xFF)) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, state, (uint8_t)((u.uint_val >> 16) & 0xFF)) != 0) return -1;
    return prc_bitwrite_uint8(ctx, state, (uint8_t)((u.uint_val >> 24) & 0xFF));
}

/* Pairs with prc_bitread_string (prc_bit.c 375-412). The reader's null_flag
   field is really "no string follows" on read-in (1 = absent, and it is
   reset to 0 once a string has been read); this writer instead just looks
   at whether string->string is non-NULL to decide what to emit. */
int
prc_bitwrite_string(prc_context *ctx, prc_bit_write_state *state, const prc_string *string)
{
    uint32_t k;
    uint8_t present;

    if (state->error)
        return -1;

    present = (string != NULL && string->string != NULL) ? 1 : 0;
    if (prc_bitwrite_bit(ctx, state, present) != 0)
        return -1;
    if (!present)
        return 0;

    if (prc_bitwrite_uint32(ctx, state, string->size) != 0)
        return -1;
    for (k = 0; k < string->size; k++)
        if (prc_bitwrite_uint8(ctx, state, string->string[k]) != 0)
            return -1;
    return 0;
}

/* Pairs with prc_bitread_uint_variable_bit (prc_bit.c 1253-1267): bit_length
   bits, MSB first. */
int
prc_bitwrite_uint_variable_bit(prc_context *ctx, prc_bit_write_state *state, uint32_t value, uint32_t bit_length)
{
    int32_t k;
    if (state->error)
        return -1;
    for (k = (int32_t)bit_length - 1; k >= 0; k--)
        if (prc_bitwrite_bit(ctx, state, (uint8_t)((value >> k) & 1)) != 0)
            return -1;
    return 0;
}

/* Pairs with prc_bitread_int_variable_bit (prc_bit.c 1270-1283): a sign bit
   then (bit_length - 1) magnitude bits. The magnitude is computed via
   int64_t so INT32_MIN doesn't overflow negation. */
int
prc_bitwrite_int_variable_bit(prc_context *ctx, prc_bit_write_state *state, int32_t value, uint32_t bit_length)
{
    uint8_t sign;
    uint32_t magnitude;

    if (state->error)
        return -1;

    sign = (value < 0) ? 1 : 0;
    magnitude = sign ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

    if (prc_bitwrite_bit(ctx, state, sign) != 0)
        return -1;
    return prc_bitwrite_uint_variable_bit(ctx, state, magnitude, bit_length - 1);
}

/* ------------------------------------------------------------------ */
/* Compact double encoding                                             */
/* ------------------------------------------------------------------ */

/* Finds a VT_double entry (a fully pre-encoded "frequent" literal value)
   whose stored magnitude bit-pattern matches `magnitude` exactly. The
   dedicated zero-code entry (NumberOfBits==2, Bits==1) is excluded here --
   it decodes without ever reading a sign bit (prc_bit.c 453-455), so it can
   only ever represent +0.0 and is handled as its own special case in
   prc_bitwrite_double instead. */
static const sCodageOfFrequentDoubleOrExponent *
prc_acofdoe_find_double(double magnitude)
{
    int k;
    uint64_t target_bits;

    memcpy(&target_bits, &magnitude, sizeof(target_bits));

    for (k = 0; k < NUMBEROFELEMENTINACOFDOE; k++)
    {
        if (acofdoe[k].Type != VT_double)
            continue;
        if (acofdoe[k].NumberOfBits == 2 && acofdoe[k].Bits == 1)
            continue;
        {
            double entry_val = acofdoe[k].u2uod.Value;
            uint64_t entry_bits;
            memcpy(&entry_bits, &entry_val, sizeof(entry_bits));
            entry_bits &= ~((uint64_t)1 << 63);
            if (entry_bits == target_bits)
                return &acofdoe[k];
        }
    }
    return NULL;
}

/* Finds the shortest-code VT_exponent entry whose stored exponent field
   matches exponent_value. The table is expected to have exactly one entry
   per 11-bit exponent value; if more than one ever matched, the shortest
   code is the compact choice. */
static const sCodageOfFrequentDoubleOrExponent *
prc_acofdoe_find_exponent(unsigned exponent_value)
{
    int k;
    const sCodageOfFrequentDoubleOrExponent *best = NULL;

    for (k = 0; k < NUMBEROFELEMENTINACOFDOE; k++)
    {
        union ieee754_double tmp;
        if (acofdoe[k].Type != VT_exponent)
            continue;
        tmp.d = acofdoe[k].u2uod.Value;
        if (tmp.ieee.exponent == exponent_value)
            if (best == NULL || acofdoe[k].NumberOfBits < best->NumberOfBits)
                best = &acofdoe[k];
    }
    return best;
}

/* Pairs with prc_bitread_double (prc_bit.c 437-524). Picks the shortest
   matching entry in the acofdoe frequent-double/exponent table (a fixed
   pre-built prefix code -- see the acofdoe[] table in prc_double.c), then
   the sign bit, then -- for entries that need one -- the mantissa. Mantissa
   bytes are chosen by directly checking, byte by byte, which of the
   reader's encodings ("literal byte", "repeat of the immediately preceding
   byte", or the two tail-fill shortcuts) reconstructs the real target
   byte(s); this is always correct, though not maximally compact: offsets
   2-5 and 7 (repeat of a byte further back than the immediately preceding
   one) are valid per the reader but are never emitted by this encoder. */
int
prc_bitwrite_double(prc_context *ctx, prc_bit_write_state *state, double val)
{
    union ieee754_double target;
    uint64_t val_bits;

    if (state->error)
        return -1;

    target.d = val;
    memcpy(&val_bits, &val, sizeof(val_bits));

    /* Exact +0.0: the table's dedicated 2-bit code, no sign bit follows. */
    if (val_bits == 0)
        return prc_bitwrite_uint_variable_bit(ctx, state, 1, 2);

    {
        double magnitude = fabs(val);
        const sCodageOfFrequentDoubleOrExponent *entry = prc_acofdoe_find_double(magnitude);

        if (entry != NULL)
        {
            if (prc_bitwrite_uint_variable_bit(ctx, state, entry->Bits, (uint32_t)entry->NumberOfBits) != 0)
                return -1;
            return prc_bitwrite_bit(ctx, state, (uint8_t)target.ieee.negative);
        }
    }

    {
        const sCodageOfFrequentDoubleOrExponent *entry = prc_acofdoe_find_exponent(target.ieee.exponent);
        uint8_t prev_byte;
        uint8_t tb[6];
        int i;

        if (entry == NULL)
        {
            /* The table is built to cover every possible 11-bit exponent
               value; if it somehow doesn't, fail rather than emit a stream
               the reader can't reconstruct. */
            state->error = 1;
            prc_error(ctx, PRC_ERROR_PARSE, "No acofdoe entry for exponent in prc_bitwrite_double\n");
            return -1;
        }

        if (prc_bitwrite_uint_variable_bit(ctx, state, entry->Bits, (uint32_t)entry->NumberOfBits) != 0)
            return -1;
        if (prc_bitwrite_bit(ctx, state, (uint8_t)target.ieee.negative) != 0)
            return -1;

        if (target.ieee.mantissa0 == 0 && target.ieee.mantissa1 == 0)
            return prc_bitwrite_bit(ctx, state, 0); /* no mantissa */

        if (prc_bitwrite_bit(ctx, state, 1) != 0)
            return -1;

        /* Uppermost 4 bits of the 20-bit mantissa0, MSB first. */
        if (prc_bitwrite_uint_variable_bit(ctx, state, (target.ieee.mantissa0 >> 16) & 0xF, 4) != 0)
            return -1;

        /* Remaining 6 mantissa bytes, most significant first: mantissa0's
           low byte, then mantissa1's 4 bytes from most to least significant
           (see the byte-layout derivation in prc_bit.c's LE branch, lines
           478-482). `prev_byte` mirrors BYTEAT(currentByte,1) -- the byte
           the reader would see "one position back" -- starting from the
           byte holding the exponent's low nibble plus the mantissa nibble
           just written. */
        prev_byte = (uint8_t)(((target.ieee.exponent & 0xF) << 4) | ((target.ieee.mantissa0 >> 16) & 0xF));
        tb[0] = (uint8_t)((target.ieee.mantissa0 >> 8) & 0xFF);
        tb[1] = (uint8_t)(target.ieee.mantissa0 & 0xFF);
        tb[2] = (uint8_t)((target.ieee.mantissa1 >> 24) & 0xFF);
        tb[3] = (uint8_t)((target.ieee.mantissa1 >> 16) & 0xFF);
        tb[4] = (uint8_t)((target.ieee.mantissa1 >> 8) & 0xFF);
        tb[5] = (uint8_t)(target.ieee.mantissa1 & 0xFF);

        for (i = 0; i < 6; )
        {
            int j, all_eq;

            all_eq = 1;
            for (j = i; j < 6; j++)
                if (tb[j] != prev_byte) { all_eq = 0; break; }
            if (all_eq)
            {
                if (prc_bitwrite_bit(ctx, state, 0) != 0) return -1;
                if (prc_bitwrite_uint_variable_bit(ctx, state, 0, 3) != 0) return -1;
                break;
            }

            if (6 - i > 1)
            {
                all_eq = 1;
                for (j = i; j < 5; j++)
                    if (tb[j] != prev_byte) { all_eq = 0; break; }
                if (all_eq)
                {
                    if (prc_bitwrite_bit(ctx, state, 0) != 0) return -1;
                    if (prc_bitwrite_uint_variable_bit(ctx, state, 6, 3) != 0) return -1;
                    if (prc_bitwrite_uint8(ctx, state, tb[5]) != 0) return -1;
                    break;
                }
            }

            if (tb[i] == prev_byte)
            {
                if (prc_bitwrite_bit(ctx, state, 0) != 0) return -1;
                if (prc_bitwrite_uint_variable_bit(ctx, state, 1, 3) != 0) return -1;
                i++;
                continue;
            }

            if (prc_bitwrite_bit(ctx, state, 1) != 0) return -1;
            if (prc_bitwrite_uint8(ctx, state, tb[i]) != 0) return -1;
            prev_byte = tb[i];
            i++;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Embedded-Huffman-array encoding, shared by prc_bitwrite_character_array
   and prc_bitwrite_short_array's compressed branches.                 */
/* ------------------------------------------------------------------ */

/* LSB-first bit writer for the embedded Huffman sub-stream: mirrors
   prc_next_huff_bit / prc_bitread_huff_bit (prc_bit.c 50-62, 293-311),
   where the bitmask starts at 0x01 and shifts left, i.e. the first bit
   written lands in the low bit of the byte. This is deliberately a
   different convention from prc_bitwrite_bit above (which is MSB-first) --
   it operates on a plain prc_bit_write_state used purely as a growable
   byte buffer, matching how the reader treats the embedded huffman_array
   as its own independent bit-addressed region. */
static int
prc_huffwrite_bit(prc_context *ctx, prc_bit_write_state *state, uint8_t bit)
{
    if (state->error)
        return -1;
    if (bit)
        state->bit_accum = (uint8_t)(state->bit_accum | (uint8_t)(1u << state->bit_fill));
    state->bit_fill++;
    if (state->bit_fill == 8)
    {
        if (prc_bitwrite_ensure_capacity(ctx, state, state->byte_pos + 1) != 0)
            return -1;
        state->buf[state->byte_pos++] = state->bit_accum;
        state->bit_accum = 0;
        state->bit_fill = 0;
    }
    return 0;
}

/* Pairs with prc_bitread_huff_data (prc_bit.c 526-537): num_bits bits, LSB
   first. */
static int
prc_huffwrite_data(prc_context *ctx, prc_bit_write_state *state, uint32_t value, uint32_t num_bits)
{
    uint32_t k;
    for (k = 0; k < num_bits; k++)
        if (prc_huffwrite_bit(ctx, state, (uint8_t)((value >> k) & 1)) != 0)
            return -1;
    return 0;
}

static int
prc_huffwrite_flush(prc_context *ctx, prc_bit_write_state *state)
{
    while (state->bit_fill != 0)
        if (prc_huffwrite_bit(ctx, state, 0) != 0)
            return -1;
    return 0;
}

typedef struct prc_huff_build_node_s prc_huff_build_node;
struct prc_huff_build_node_s
{
    uint64_t freq;
    uint32_t value;
    uint8_t is_leaf;
    prc_huff_build_node *left;
    prc_huff_build_node *right;
};

static uint32_t
prc_bit_width_u32(uint32_t v)
{
    uint32_t width = 0;
    while (v != 0)
    {
        v >>= 1;
        width++;
    }
    return width;
}

/* ISO/CD 14739-1 SS9.1 "GetNumberOfBitsUsedToStoreUnsignedInteger":
   unsigned GetNumberOfBitsUsedToStoreUnsignedInteger(unsigned uValue) {
       unsigned uNbBit = 1, uTemp = 1;
       while (uValue > uTemp) { uTemp *= 2; uNbBit++; }
       return uNbBit;
   }
   This is NOT the same function as prc_bit_width_u32 above (which counts
   bits in v's binary representation, i.e. smallest k with v < 2^k): the
   spec's version starts its comparison at uTemp==1 with uNbBit==1 already
   counted, so it returns one MORE than prc_bit_width_u32(uValue) whenever
   uValue is not itself an exact power of two (or zero) -- e.g. uValue=252
   (not a power of 2): spec returns 9, prc_bit_width_u32(252) returns 8.
   Algebraically, GetNumberOfBitsUsedToStoreUnsignedInteger(v) == 1 +
   prc_bit_width_u32(v > 0 ? v - 1 : 0) for all v >= 0 (verified by direct
   trace of the spec's while loop above). Used both for signed per-value
   bit-lengths (prc_int32_bit_width_signed below) and for the
   max_code_length FIELD written by prc_bitwrite_huffman_block, which
   turned out to need the SAME spec function applied to the true maximum
   code length among leaves, not "true max minus 1" as an earlier,
   incomplete fix assumed (2026-07-10: that heuristic only coincidentally
   matched real files for max values of 4-5; it broke down for point_
   array's max_code_length of 8, where the correct field value is 4, not
   7 -- found by comparing this write facility's own leaf table, which
   already matched the real file's leaf-for-leaf exactly by this point,
   against the real file's actual max_code_length field value). */
static uint32_t
prc_spec_bits_for_unsigned(uint32_t v)
{
    return 1 + prc_bit_width_u32(v > 0 ? v - 1 : 0);
}

static int
prc_huff_value_compare(const void *a, const void *b)
{
    uint32_t va = *(const uint32_t *)a;
    uint32_t vb = *(const uint32_t *)b;
    if (va < vb) return -1;
    if (va > vb) return 1;
    return 0;
}

/* Builds a Huffman tree over the distinct values present in
   values[0..count-1] (a simple O(D^2) pairwise-minimum merge over the D
   distinct values -- fine for the array sizes this write facility deals
   with; a large, highly diverse alphabet would want a heap instead).
   *out_nodes / *out_node_count receive the flat list of every build node
   allocated (for teardown); *out_root receives the tree root. A distinct
   alphabet of size 1 is padded with one zero-frequency phantom leaf so the
   reader's tree walk (which always consumes at least one bit per decoded
   value, even for a single-symbol alphabet -- prc_bit.c 843-877) has
   somewhere to go. Returns 0 on success, -1 on allocation failure (in which
   case *out_nodes / *out_node_count still describe whatever was allocated so
   the caller can free it). */
static int
prc_huff_build_tree(prc_context *ctx, const uint32_t *values, uint32_t count,
    prc_huff_build_node **out_root, prc_huff_build_node ***out_nodes, uint32_t *out_node_count)
{
    uint32_t *sorted;
    uint32_t distinct_count;
    uint32_t *distinct_values;
    uint64_t *distinct_freqs;
    prc_huff_build_node **nodes;
    uint32_t node_count, node_capacity;
    prc_huff_build_node **active;
    uint32_t active_count;
    uint32_t i;

    *out_nodes = NULL;
    *out_node_count = 0;
    *out_root = NULL;

    sorted = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * count);
    if (sorted == NULL)
        return -1;
    memcpy(sorted, values, sizeof(uint32_t) * count);
    qsort(sorted, count, sizeof(uint32_t), prc_huff_value_compare);

    distinct_values = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * count);
    distinct_freqs = (uint64_t *)prc_malloc(ctx, sizeof(uint64_t) * count);
    if (distinct_values == NULL || distinct_freqs == NULL)
    {
        prc_free(ctx, sorted);
        prc_free(ctx, distinct_values);
        prc_free(ctx, distinct_freqs);
        return -1;
    }

    distinct_count = 0;
    for (i = 0; i < count; i++)
    {
        if (distinct_count == 0 || sorted[i] != distinct_values[distinct_count - 1])
        {
            distinct_values[distinct_count] = sorted[i];
            distinct_freqs[distinct_count] = 1;
            distinct_count++;
        }
        else
        {
            distinct_freqs[distinct_count - 1]++;
        }
    }
    prc_free(ctx, sorted);

    /* +2 over the "normal" 2*distinct_count-1 node budget covers the
       active_count==1 single-value special case's extra phantom leaf below;
       +2 more (total +4) covers the unconditional top-level phantom-wrap
       merge (two more nodes: the dead phantom branch and the new outer
       root) added after the main merge loop, in every case including that
       single-value one. */
    node_capacity = distinct_count * 2 + 4;
    nodes = (prc_huff_build_node **)prc_calloc(ctx, node_capacity, sizeof(prc_huff_build_node *));
    active = (prc_huff_build_node **)prc_calloc(ctx, node_capacity, sizeof(prc_huff_build_node *));
    if (nodes == NULL || active == NULL)
    {
        prc_free(ctx, distinct_values);
        prc_free(ctx, distinct_freqs);
        prc_free(ctx, nodes);
        prc_free(ctx, active);
        return -1;
    }
    node_count = 0;
    active_count = 0;

    for (i = 0; i < distinct_count; i++)
    {
        prc_huff_build_node *leaf = (prc_huff_build_node *)prc_malloc(ctx, sizeof(prc_huff_build_node));
        if (leaf == NULL)
        {
            prc_free(ctx, distinct_values);
            prc_free(ctx, distinct_freqs);
            prc_free(ctx, active);
            *out_nodes = nodes;
            *out_node_count = node_count;
            return -1;
        }
        leaf->freq = distinct_freqs[i];
        leaf->value = distinct_values[i];
        leaf->is_leaf = 1;
        leaf->left = NULL;
        leaf->right = NULL;
        nodes[node_count++] = leaf;
        active[active_count++] = leaf;
    }
    prc_free(ctx, distinct_values);
    prc_free(ctx, distinct_freqs);

    if (active_count == 1)
    {
        prc_huff_build_node *phantom = (prc_huff_build_node *)prc_malloc(ctx, sizeof(prc_huff_build_node));
        if (phantom == NULL)
        {
            prc_free(ctx, active);
            *out_nodes = nodes;
            *out_node_count = node_count;
            return -1;
        }
        phantom->freq = 0;
        phantom->value = (active[0]->value == 0) ? 1u : 0u;
        phantom->is_leaf = 1;
        phantom->left = NULL;
        phantom->right = NULL;
        nodes[node_count++] = phantom;
        active[active_count++] = phantom;
    }

    while (active_count > 1)
    {
        uint32_t min1, min2, k;
        prc_huff_build_node *parent;

        min1 = 0;
        for (k = 1; k < active_count; k++)
            if (active[k]->freq < active[min1]->freq)
                min1 = k;
        min2 = (min1 == 0) ? 1 : 0;
        for (k = 0; k < active_count; k++)
            if (k != min1 && active[k]->freq < active[min2]->freq)
                min2 = k;

        parent = (prc_huff_build_node *)prc_malloc(ctx, sizeof(prc_huff_build_node));
        if (parent == NULL)
        {
            prc_free(ctx, active);
            *out_nodes = nodes;
            *out_node_count = node_count;
            return -1;
        }
        parent->freq = active[min1]->freq + active[min2]->freq;
        parent->is_leaf = 0;
        parent->value = 0;
        parent->left = active[min1];
        parent->right = active[min2];
        nodes[node_count++] = parent;

        active[min1] = parent;
        active[min2] = active[active_count - 1];
        active_count--;
    }

    /* A real, independently-produced compressed PRC file's Huffman tree for
       these array fields is always structurally INCOMPLETE in one specific
       way: every actual leaf code begins with bit 1, and the root's entire
       0-branch is unused (Kraft sum 0.5, not 1.0) -- confirmed by decoding
       the real leaf tables for edge_status_array, triangle_face_array's and
       point_reference_array's bit_lengths sub-arrays (2026-07-10 causal-
       isolation investigation: swapping only this write facility's own,
       complete/optimal tree into an otherwise-real, Acrobat-working file
       reproduces Acrobat's blank-model-tree rejection; the real tree's
       *codes*, once this extra wrap is added, match hand-verified digit for
       digit). Reproduce it by wrapping the tree built above (over the real
       leaves only) with one more merge against an unused phantom leaf,
       placed as the LEFT (0) child so every real leaf's code gains a
       leading 1 bit -- matching the real encoder's output exactly, not
       merely a same-size, differently-shaped valid alternative. */
    {
        prc_huff_build_node *phantom_wrap = (prc_huff_build_node *)prc_malloc(ctx, sizeof(prc_huff_build_node));
        prc_huff_build_node *new_root;

        if (phantom_wrap == NULL)
        {
            prc_free(ctx, active);
            *out_root = active[0];
            *out_nodes = nodes;
            *out_node_count = node_count;
            return -1;
        }
        /* NOT a leaf: a dead internal node with no children. The real
           encoder's leaf table has exactly as many entries as there are
           distinct real values (confirmed against the real file: 4 leaves
           for edge_status_array, not 5) -- prc_huff_assign_codes only
           records a table entry for is_leaf nodes it actually pops, and
           this node (is_leaf=0, left=right=NULL) contributes nothing when
           popped, exactly mirroring how the decoder's own on-demand tree
           walk (prc_huffman_data_decoder / prc_bitread_character_array)
           never touches root->left in the first place, since no real
           leaf's code starts with bit 0. */
        phantom_wrap->freq = 0;
        phantom_wrap->value = 0;
        phantom_wrap->is_leaf = 0;
        phantom_wrap->left = NULL;
        phantom_wrap->right = NULL;
        nodes[node_count++] = phantom_wrap;

        new_root = (prc_huff_build_node *)prc_malloc(ctx, sizeof(prc_huff_build_node));
        if (new_root == NULL)
        {
            prc_free(ctx, active);
            *out_root = active[0];
            *out_nodes = nodes;
            *out_node_count = node_count;
            return -1;
        }
        new_root->freq = active[0]->freq;
        new_root->is_leaf = 0;
        new_root->value = 0;
        new_root->left = phantom_wrap;
        new_root->right = active[0];
        nodes[node_count++] = new_root;

        active[0] = new_root;
    }

    *out_root = active[0];
    *out_nodes = nodes;
    *out_node_count = node_count;
    prc_free(ctx, active);
    return 0;
}

typedef struct
{
    prc_huff_build_node *node;
    uint32_t code;
    uint32_t depth;
} prc_huff_walk_item;

/* Iterative (not recursive) leaf-code assignment: tree depth is driven by
   the frequency distribution of caller-supplied data, so a pathological
   distribution could otherwise drive deep C-stack recursion. Also doubles
   as the max_code_length enforcement point: the reader rejects
   max_code_length > 32 (prc_bit.c 645-650), so this aborts as soon as a
   code would need more than 32 bits rather than building an
   unreadable-by-design table. Returns 0 on success, -1 on a too-long code
   or allocation failure.

   KNOWN LOOSE END (2026-07-10): for leaves with EQUAL frequency, this
   write facility's code VALUE assignment does not always match what real,
   independently-produced PRC files use -- confirmed by simulating this
   exact algorithm (including prc_huff_build_tree's specific "first found,
   ascending-index" tie-break and swap-with-last active-list mechanics)
   against normal_angle_array's real 22-leaf frequency table: 12 of 22
   leaves matched exactly (same code_length AND code_value), but all 10
   mismatches were confined to frequency-tied leaves (e.g. two leaves both
   occurring 23 times get the same code LENGTH in both this tree and the
   real one, but the real file swaps which of the two gets the lower vs.
   higher code VALUE). Tested the standard "canonical Huffman" hypothesis
   (reassign code values purely from the already-correct lengths, sorted
   by value, independent of tree shape) -- did not match either. The real
   encoder's exact tie-breaking rule for equal-frequency leaves has not
   been determined; would need tied-frequency examples from more real
   files to pin down with confidence rather than guess from one. Does NOT
   affect code LENGTH (hence total field size is already correct) and is
   not shown to matter for decodability by any reader tried, including
   Acrobat -- unlike the phantom-wrap/tree-shape fix elsewhere in this
   file, which IS confirmed causal to Acrobat's blank-model-tree
   rejection. Affects normal_angle_array and (very likely, same
   mechanism, not independently re-verified) point_reference_array. */
static int
prc_huff_assign_codes(prc_context *ctx, prc_huff_build_node *root, uint32_t leaf_count,
    uint32_t *leaf_values, uint32_t *code_lengths, uint32_t *code_values)
{
    prc_huff_walk_item *stack;
    uint32_t stack_size = 0;
    uint32_t stack_capacity = leaf_count * 2 + 2;
    uint32_t leaf_index = 0;
    int result = 0;

    stack = (prc_huff_walk_item *)prc_malloc(ctx, sizeof(prc_huff_walk_item) * stack_capacity);
    if (stack == NULL)
        return -1;

    stack[stack_size].node = root;
    stack[stack_size].code = 0;
    stack[stack_size].depth = 0;
    stack_size++;

    while (stack_size > 0)
    {
        prc_huff_walk_item item = stack[--stack_size];

        if (item.node->is_leaf)
        {
            if (item.depth == 0 || item.depth > 32 || leaf_index >= leaf_count)
            {
                result = -1;
                break;
            }
            leaf_values[leaf_index] = item.node->value;
            code_lengths[leaf_index] = item.depth;
            code_values[leaf_index] = item.code;
            leaf_index++;
            continue;
        }

        if (item.depth >= 32)
        {
            result = -1;
            break;
        }
        if (stack_size + 2 > stack_capacity)
        {
            uint32_t new_capacity = stack_capacity * 2;
            prc_huff_walk_item *new_stack = (prc_huff_walk_item *)prc_realloc(ctx, stack,
                sizeof(prc_huff_walk_item) * new_capacity);
            if (new_stack == NULL)
            {
                result = -1;
                break;
            }
            stack = new_stack;
            stack_capacity = new_capacity;
        }
        if (item.node->left != NULL)
        {
            stack[stack_size].node = item.node->left;
            stack[stack_size].code = item.code << 1;
            stack[stack_size].depth = item.depth + 1;
            stack_size++;
        }
        if (item.node->right != NULL)
        {
            stack[stack_size].node = item.node->right;
            stack[stack_size].code = (item.code << 1) | 1;
            stack[stack_size].depth = item.depth + 1;
            stack_size++;
        }
    }

    prc_free(ctx, stack);
    return result;
}

/* Builds and writes one embedded-Huffman-array block to the MAIN stream:
   array_size (repurposed as the embedded sub-stream's word count) + the
   raw prc_bitread_uncompressed_uint32-shaped words + bits_last_integer --
   exactly what prc_huffman_data_decoder and prc_bitread_character_array's
   compressed branch expect to read back (prc_bit.c 585-894, 947-1234).
   `values` holds `count` symbol values already widened to uint32_t;
   `num_bits` is the leaf-value bit width (matching the reader's num_bits /
   number_of_bits parameter). Leaves the leading array_size field to be
   written by this function (not the caller), since its value --
   huffman_array_size -- isn't known until the whole sub-stream is built. */
static int
prc_bitwrite_huffman_block(prc_context *ctx, prc_bit_write_state *state,
    const uint32_t *values, uint32_t count, uint8_t num_bits)
{
    prc_huff_build_node *root = NULL;
    prc_huff_build_node **nodes = NULL;
    uint32_t node_count = 0;
    uint32_t *leaf_values = NULL;
    uint32_t *code_lengths = NULL;
    uint32_t *code_values = NULL;
    uint32_t leaf_count;
    uint32_t max_code_length;
    uint32_t k;
    prc_bit_write_state sub;
    int sub_initialized = 0;
    size_t bits_written;
    size_t huffman_array_size;
    size_t bits_last_integer;
    int result = -1;

    if (state->error)
        return -1;
    if (count == 0 || values == NULL)
    {
        state->error = 1;
        prc_error(ctx, PRC_ERROR_PARSE, "prc_bitwrite_huffman_block called with no data\n");
        return -1;
    }

    if (prc_huff_build_tree(ctx, values, count, &root, &nodes, &node_count) != 0)
        goto cleanup;

    leaf_count = 0;
    for (k = 0; k < node_count; k++)
        if (nodes[k]->is_leaf)
            leaf_count++;

    leaf_values = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * leaf_count);
    code_lengths = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * leaf_count);
    code_values = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * leaf_count);
    if (leaf_values == NULL || code_lengths == NULL || code_values == NULL)
        goto cleanup;

    if (prc_huff_assign_codes(ctx, root, leaf_count, leaf_values, code_lengths, code_values) != 0)
    {
        state->error = 1;
        prc_error(ctx, PRC_ERROR_HUFFMAN, "Huffman code table build failed in prc_bitwrite_huffman_block\n");
        goto cleanup;
    }

    /* prc_huff_assign_codes emits leaves in tree-traversal order (right
       child before left, depth-first), not sorted by value -- the real
       file's own leaf table is always listed in ascending leaf VALUE
       order regardless of tree shape (confirmed against the real
       edge_status_array table: values 0,1,2,3 in that order, while this
       tree's own traversal order was 0,3,1,2). Table order has no bearing
       on decodability (each value/code_length/code_value triple is
       self-contained), only on matching the real encoder's exact bytes;
       sort here so this write facility's output does. */
    {
        uint32_t a, b_idx;
        for (a = 0; a + 1 < leaf_count; a++)
        {
            uint32_t min_idx = a;
            for (b_idx = a + 1; b_idx < leaf_count; b_idx++)
                if (leaf_values[b_idx] < leaf_values[min_idx])
                    min_idx = b_idx;
            if (min_idx != a)
            {
                uint32_t tmp;
                tmp = leaf_values[a]; leaf_values[a] = leaf_values[min_idx]; leaf_values[min_idx] = tmp;
                tmp = code_lengths[a]; code_lengths[a] = code_lengths[min_idx]; code_lengths[min_idx] = tmp;
                tmp = code_values[a]; code_values[a] = code_values[min_idx]; code_values[min_idx] = tmp;
            }
        }
    }

    {
        uint32_t true_max_code_length = 0;
        for (k = 0; k < leaf_count; k++)
            if (code_lengths[k] > true_max_code_length)
                true_max_code_length = code_lengths[k];

        /* The real file's max_code_length FIELD is NOT the true maximum
           code length among leaves (after prc_huff_build_tree's
           unconditional top-level phantom wrap) -- it is SS9.1
           "GetNumberOfBitsUsedToStoreUnsignedInteger" applied to that true
           maximum (prc_spec_bits_for_unsigned above), i.e. the number of
           bits needed to represent the true maximum as an unsigned value
           per the spec's own counting function, used here as a FIXED
           BIT-WIDTH for each leaf's code_length field (which can therefore
           exceed the field's own numeric value -- it is a bit-width, not a
           value cap). An earlier fix used "true_max - 1" as a shortcut,
           which coincidentally matches this spec formula for true_max in
           {3,4,5} but diverges for larger values (true_max=8 needs field=4,
           not 7) -- found via point_array's larger, 9-leaf alphabet, where
           this write facility's own leaf table already matched the real
           file leaf-for-leaf, isolating the max_code_length field itself
           as the sole remaining discrepancy (2026-07-10). */
        max_code_length = prc_spec_bits_for_unsigned(true_max_code_length);
    }

    if (prc_bitwrite_init(ctx, &sub, 64) != 0)
        goto cleanup;
    sub_initialized = 1;

    if (prc_huffwrite_data(ctx, &sub, leaf_count, (uint32_t)num_bits + 1) != 0) goto cleanup;
    if (prc_huffwrite_data(ctx, &sub, max_code_length, 8) != 0) goto cleanup;

    for (k = 0; k < leaf_count; k++)
    {
        if (prc_huffwrite_data(ctx, &sub, leaf_values[k], num_bits) != 0) goto cleanup;
        if (prc_huffwrite_data(ctx, &sub, code_lengths[k], max_code_length) != 0) goto cleanup;
        if (prc_huffwrite_data(ctx, &sub, code_values[k], code_lengths[k]) != 0) goto cleanup;
    }

    if (prc_huffwrite_data(ctx, &sub, count, 32) != 0) goto cleanup;

    /* Per-value path bits, root to leaf, MSB first -- code_values[k]'s bit
       at position (code_lengths[k]-1) is the root branch (see
       prc_huff_assign_codes: code is built left=0/right=1 while descending
       from the root), matching the order prc_huffman_data_decoder's
       per-value tree walk consumes bits in (prc_bit.c 842-877). Leaf
       lookup here is a linear scan (fine for the alphabet sizes in
       practice; a hash map would help for a very large distinct alphabet). */
    for (k = 0; k < count; k++)
    {
        uint32_t j;
        int found = 0;
        for (j = 0; j < leaf_count; j++)
        {
            if (leaf_values[j] == values[k])
            {
                uint32_t m;
                for (m = 0; m < code_lengths[j]; m++)
                {
                    uint8_t path_bit = (uint8_t)((code_values[j] >> (code_lengths[j] - 1 - m)) & 1);
                    if (prc_huffwrite_bit(ctx, &sub, path_bit) != 0)
                        goto cleanup;
                }
                found = 1;
                break;
            }
        }
        if (!found)
        {
            state->error = 1;
            prc_error(ctx, PRC_ERROR_INTERNAL, "Huffman leaf lookup failed in prc_bitwrite_huffman_block\n");
            goto cleanup;
        }
    }

    if (sub.error)
        goto cleanup;

    bits_written = sub.byte_pos * 8 + sub.bit_fill;
    if (prc_huffwrite_flush(ctx, &sub) != 0)
        goto cleanup;

    huffman_array_size = (bits_written + 31) / 32;
    if (huffman_array_size == 0)
        huffman_array_size = 1;
    bits_last_integer = bits_written - (huffman_array_size - 1) * 32;

    /* Pad the sub-buffer up to a whole number of 32-bit words; these
       padding bits are beyond bits_written (i.e. beyond max_bits on the
       read side, see prc_init_huff_bit_state) and are never read back. */
    if (prc_bitwrite_ensure_capacity(ctx, &sub, huffman_array_size * 4) != 0)
        goto cleanup;
    while (sub.byte_pos < huffman_array_size * 4)
        sub.buf[sub.byte_pos++] = 0;

    if (prc_bitwrite_uint32(ctx, state, (uint32_t)huffman_array_size) != 0) goto cleanup;
    for (k = 0; k < (uint32_t)(huffman_array_size * 4); k++)
        if (prc_bitwrite_uint8(ctx, state, sub.buf[k]) != 0) goto cleanup;
    if (prc_bitwrite_uint32(ctx, state, (uint32_t)bits_last_integer) != 0) goto cleanup;

    result = 0;

cleanup:
    if (sub_initialized)
        prc_bitwrite_release(ctx, &sub);
    for (k = 0; k < node_count; k++)
        prc_free(ctx, nodes[k]);
    prc_free(ctx, nodes);
    prc_free(ctx, leaf_values);
    prc_free(ctx, code_lengths);
    prc_free(ctx, code_values);
    if (result != 0)
        state->error = 1;
    return result;
}

/* ------------------------------------------------------------------ */
/* Array writers                                                       */
/* ------------------------------------------------------------------ */

/* SS9.13 "WriteIntegerWithVariableBitNumber": 1 sign bit + (uBitNumber-1)
   magnitude bits via WriteUnsignedIntegerWithVariableBitNumber (SS9.14,
   direct MSB-first binary of the magnitude itself, not biased) -- this
   write facility's prc_bitwrite_int_variable_bit already implements that
   packing correctly; only the LENGTH this function computes (fed in as
   bit_lengths[k], i.e. uBitNumber) was wrong. */
static uint32_t
prc_int32_bit_width_signed(int32_t v)
{
    uint32_t magnitude = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    return 1 + prc_spec_bits_for_unsigned(magnitude);
}

/* Pairs with prc_bitread_character_array (prc_bit.c 896-1251, "10.6 Inverse
   of Writecharacterarray"). Compression is chosen the same way the reader
   determines it (has_comp_bit / num_known, prc_bit.c 910-945): when
   has_comp_bit lets this writer choose, it always compresses (a real
   Huffman encoder now exists -- see prc_bitwrite_huffman_block); otherwise
   it mirrors the format-mandated rule -- Huffman is required once
   num_known > 3, and forbidden otherwise. */
int
prc_bitwrite_character_array(prc_context *ctx, prc_bit_write_state *state,
    const uint8_t *data, uint32_t data_size, uint8_t num_bits, uint8_t has_comp_bit, uint32_t num_known)
{
    uint8_t compressed;
    uint32_t k;

    if (state->error)
        return -1;

    /* matches: if (has_comp_bit) compressed = prc_bitread_bit(ctx, state);
       (prc_bit.c 911-912) -- when has_comp_bit is false, the reader instead
       derives `compressed` further down from num_known (prc_bit.c 922-934):
       false when num_known <= 3, true (mandatory Huffman) otherwise. Since a
       real Huffman encoder exists, this writer takes the compressed path
       whenever it's free to choose (has_comp_bit), not just when forced. */
    compressed = has_comp_bit ? 1 : ((num_known > 3) ? 1 : 0);
    if (has_comp_bit)
        if (prc_bitwrite_bit(ctx, state, compressed) != 0)
            return -1;

    /* matches: array_size = prc_bitread_uint32(ctx, state); (prc_bit.c 915)
       -- read unconditionally, before has_comp_bit/num_known is consulted
       again, so exactly one array_size field is written here regardless of
       branch. Its MEANING differs by branch (data_size vs huffman_array_size,
       prc_bit.c 922-934/936-945): the uncompressed branch below writes it
       directly as data_size; the compressed branch instead leaves it to
       prc_bitwrite_huffman_block, which writes the same field position but
       with the value huffman_array_size (unknown until the sub-stream is
       built). matches: if (array_size == 0) { *data_size = 0; return NULL; }
       (prc_bit.c 916-920) -- zero-size has no payload in EITHER branch, so
       it's handled uniformly here before any branch-specific write. */
    if (data_size == 0)
        return prc_bitwrite_uint32(ctx, state, 0);

    if (!compressed)
    {
        /* matches: data = malloc(*data_size); for (k = 0; k < *data_size;
           k++) data[k] = prc_bitread_uint8(ctx, state); (prc_bit.c 1237-1248) */
        if (prc_bitwrite_uint32(ctx, state, data_size) != 0)
            return -1;
        for (k = 0; k < data_size; k++)
            if (prc_bitwrite_uint8(ctx, state, data[k]) != 0)
                return -1;
        return 0;
    }

    {
        /* matches: the compressed branch (prc_bit.c 947-1227), which reads
           huffman_array_size raw words + bits_last_integer, then the
           leaf table + per-value codes from the embedded huff sub-stream --
           all produced by prc_bitwrite_huffman_block. Values are widened to
           uint32_t here purely because that helper's signature is shared
           with prc_bitwrite_short_array (whose element type is int32_t, not
           uint8_t); the widening has no effect on the bits written. */
        uint32_t *widened;
        int result;

        widened = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * data_size);
        if (widened == NULL)
        {
            state->error = 1;
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitwrite_character_array\n");
            return -1;
        }
        for (k = 0; k < data_size; k++)
            widened[k] = data[k];

        result = prc_bitwrite_huffman_block(ctx, state, widened, data_size, num_bits);
        prc_free(ctx, widened);
        return result;
    }
}

/* Pairs with prc_bitread_short_array (prc_bit.c 1458-1524, "10.7 Inverse of
   Writeshortarray"). Unlike character_array, compression is only ever
   possible when has_comp_bit is set -- the reader hard-codes compressed to
   false whenever has_comp_bit is false (prc_bit.c 1466-1470) -- so that's
   the only case this writer can legally choose Huffman, and (per the
   "always compress when the format lets us" rule) does. The plain branch
   reconstructs only a 16-bit unsigned value per element (temp1 + temp2<<8,
   prc_bit.c 1518-1520), so it writes the low 16 bits of each value; values
   outside that range are a pre-existing limitation of this branch of the
   format, not something introduced here. */
int
prc_bitwrite_short_array(prc_context *ctx, prc_bit_write_state *state,
    const int32_t *data, uint32_t data_size, uint8_t has_comp_bit, uint8_t number_of_bits)
{
    uint8_t compressed;
    uint32_t k;

    if (state->error)
        return -1;

    /* matches: if (has_comp_bit) compressed = prc_bitread_bit(ctx, state);
       else compressed = false; (prc_bit.c 1466-1470). Unlike
       character_array, there's no num_known-driven "Huffman is mandatory"
       case here -- has_comp_bit == false always means plain output on the
       read side, so that's the only thing this branch can produce then. */
    compressed = has_comp_bit ? 1 : 0;
    if (has_comp_bit)
        if (prc_bitwrite_bit(ctx, state, compressed) != 0)
            return -1;

    /* matches: size = prc_bitread_uint32(ctx, state); if (size == 0) return
       NULL; (prc_bit.c 1473-1475) -- one shared size field regardless of
       branch, as with character_array above. */
    if (data_size == 0)
        return prc_bitwrite_uint32(ctx, state, 0);

    if (!compressed)
    {
        /* matches: output = malloc(size); for (j = 0; j < size; j++) {
           temp1 = prc_bitread_uint8(ctx, state); temp2 =
           prc_bitread_uint8(ctx, state); output[j] = temp1 + (temp2 << 8);
           } (prc_bit.c 1504-1521) -- reconstructs only a 16-bit unsigned
           value per element, so only the low 16 bits of data[k] survive;
           low byte first, matching temp1 (bits 0-7) before temp2 (bits 8-15). */
        if (prc_bitwrite_uint32(ctx, state, data_size) != 0)
            return -1;
        for (k = 0; k < data_size; k++)
        {
            if (prc_bitwrite_uint8(ctx, state, (uint8_t)(data[k] & 0xFF)) != 0) return -1;
            if (prc_bitwrite_uint8(ctx, state, (uint8_t)((data[k] >> 8) & 0xFF)) != 0) return -1;
        }
        return 0;
    }

    {
        /* matches: data = prc_huffman_data_decoder(ctx, state, number_of_bits,
           size, data_size); ... memcpy(output, data, sizeof(int32_t) *
           (*data_size)); (prc_bit.c 1480-1500) -- the decoder's leaf values
           are bit-reinterpreted (via memcpy) straight into the int32_t
           output, so widening data[k] to uint32_t via memcpy here (rather
           than a value-converting cast) is the correct mirror. */
        uint32_t *widened;
        int result;

        widened = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * data_size);
        if (widened == NULL)
        {
            state->error = 1;
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitwrite_short_array\n");
            return -1;
        }
        for (k = 0; k < data_size; k++)
            memcpy(&widened[k], &data[k], sizeof(uint32_t));

        /* prc_bitwrite_huffman_block writes the leading size field itself
           (here meaning huffman_array_size, not data_size -- see
           prc_bitread_short_array's `size` reuse, prc_bit.c 1473, 1483). */
        result = prc_bitwrite_huffman_block(ctx, state, widened, data_size, number_of_bits);
        prc_free(ctx, widened);
        return result;
    }
}

/* Pairs with prc_bitread_compressed_integer_array (prc_bit.c 1425-1456,
   "Inverse of 10.8 Writecompressedintegerarray"): a character array of
   per-element bit lengths, then each value written with exactly the bit
   length its own entry specifies (no delta-encoding here, unlike
   compressed_indice_array below). */
int
prc_bitwrite_compressed_integer_array(prc_context *ctx, prc_bit_write_state *state,
    const int32_t *data, uint32_t data_size)
{
    uint8_t *bit_lengths;
    uint32_t k;
    int result;

    if (state->error)
        return -1;
    /* matches: bit_lengths = prc_bitread_character_array(ctx, state, &size,
       6, true, 0); (prc_bit.c 1435) called with has_comp_bit=true, num_known=0
       regardless of data_size, so the empty case still writes a character
       array (of size 0) rather than nothing. */
    if (data_size == 0)
        return prc_bitwrite_character_array(ctx, state, NULL, 0, 6, 1, 0);

    /* matches: bit_lengths[k] is later fed straight into
       prc_bitread_int_variable_bit(ctx, state, bit_lengths[k]) per element
       (prc_bit.c 1447) with no delta/accumulation -- so each entry here is
       simply the absolute bit length (sign bit + magnitude bits) data[k]
       itself needs, computed up front so it can be written as the
       character array below before any data value is written. */
    bit_lengths = (uint8_t *)prc_malloc(ctx, sizeof(uint8_t) * data_size);
    if (bit_lengths == NULL)
    {
        state->error = 1;
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitwrite_compressed_integer_array\n");
        return -1;
    }
    for (k = 0; k < data_size; k++)
        bit_lengths[k] = (uint8_t)prc_int32_bit_width_signed(data[k]);

    /* matches: bit_lengths = prc_bitread_character_array(ctx, state, &size,
       6, true, 0); (prc_bit.c 1435) -- the bit-length table itself, written
       (and possibly Huffman-compressed) before any of the values it
       describes. */
    result = prc_bitwrite_character_array(ctx, state, bit_lengths, data_size, 6, 1, 0);
    if (result == 0)
    {
        /* matches: for (k = 0; k < size; k++) data[k] =
           prc_bitread_int_variable_bit(ctx, state, bit_lengths[k]);
           (prc_bit.c 1445-1450) -- each value written with exactly the bit
           length its own (already-written) table entry specifies. */
        for (k = 0; k < data_size; k++)
        {
            if (prc_bitwrite_int_variable_bit(ctx, state, data[k], bit_lengths[k]) != 0)
            {
                result = -1;
                break;
            }
        }
    }

    prc_free(ctx, bit_lengths);
    return result;
}

/* Pairs with prc_bitread_compressed_indice_array (prc_bit.c 1526-1585,
   "Inverse of 10.9 Writecompressedindicearray. These are difference
   encoded"). bit_lengths[0] is the absolute bit length needed for data[0];
   bit_lengths[k] (k >= 1) is a SIGNED delta against the running bit-length
   total, stored as a 6-bit two's complement value to match the reader's
   ">= 32 => | 0xc0" reinterpretation (prc_bit.c 1553-1565), not an absolute
   length. */
int
prc_bitwrite_compressed_indice_array(prc_context *ctx, prc_bit_write_state *state,
    const int32_t *data, uint32_t data_size, uint8_t has_comp_bit, uint32_t num_known)
{
    uint8_t *bit_lengths;
    uint32_t *needed;
    uint32_t k;
    int result;

    if (state->error)
        return -1;
    /* matches: bit_lengths = prc_bitread_character_array(ctx, state, &size,
       6, has_comp_bit, num_known); *data_size = size; if (bit_lengths ==
       NULL || size == 0) return NULL; (prc_bit.c 1543-1547) -- has_comp_bit
       / num_known are forwarded as given (unlike compressed_integer_array,
       which always hard-codes true/0), since the reader lets the caller
       control compression here too. */
    if (data_size == 0)
        return prc_bitwrite_character_array(ctx, state, NULL, 0, 6, has_comp_bit, num_known);

    bit_lengths = (uint8_t *)prc_malloc(ctx, sizeof(uint8_t) * data_size);
    needed = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * data_size);
    if (bit_lengths == NULL || needed == NULL)
    {
        prc_free(ctx, bit_lengths);
        prc_free(ctx, needed);
        state->error = 1;
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitwrite_compressed_indice_array\n");
        return -1;
    }

    /* matches: bit_count = bit_lengths[0]; data[0] =
       prc_bitread_int_variable_bit(ctx, state, bit_count); (prc_bit.c
       1568-1570) -- bit_lengths[0] is read back as-is (no >=32
       reinterpretation makes sense for an absolute, always-non-negative
       length), so it must stay within 0-31 to round-trip through the
       reader's 6-bit character-array element unambiguously. */
    needed[0] = prc_int32_bit_width_signed(data[0]);
    if (needed[0] >= 32)
    {
        prc_free(ctx, bit_lengths);
        prc_free(ctx, needed);
        state->error = 1;
        prc_error(ctx, PRC_ERROR_PARSE, "First index needs >= 32 bits in prc_bitwrite_compressed_indice_array\n");
        return -1;
    }
    bit_lengths[0] = (uint8_t)needed[0];

    /* matches: for (k = 0; k < size; k++) if (bit_lengths[k] >= 32)
       bit_lengths[k] |= 0xc0; ... for (k = 1; k < size; k++) { bit_count +=
       (int)bit_lengths[k]; temp = prc_bitread_int_variable_bit(ctx, state,
       (uint8_t)bit_count); data[k] = temp + data[k-1]; } (prc_bit.c
       1559-1581) -- bit_lengths[k] (k>=1) is a delta against the running
       bit_count, not an absolute length; values in [-32,-1] are stored as
       the 6-bit two's-complement patterns 32-63 (i.e. `delta & 0x3F` below
       is exactly the inverse of the reader's `| 0xc0` sign-extension), so
       delta must stay within [-32, 31] to be unambiguous both ways. */
    for (k = 1; k < data_size; k++)
    {
        int64_t diff = (int64_t)data[k] - (int64_t)data[k - 1];
        uint32_t abs_diff = (diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff;
        int32_t delta;

        /* Same SS9.1 GetNumberOfBitsUsedToStoreUnsignedInteger formula as
           prc_int32_bit_width_signed (this write facility's other signed
           bit-length site) -- this call site computed the delta's signed
           bit-length independently and had the identical bug: 1 +
           prc_bit_width_u32(abs_diff) undercounts by 1 bit whenever
           abs_diff isn't itself an exact power of two. Found via triangle_
           face_array, whose more varied face-index deltas exposed this
           where point_reference_array's narrower delta range happened not
           to (2026-07-10). */
        needed[k] = 1 + prc_spec_bits_for_unsigned(abs_diff);
        delta = (int32_t)needed[k] - (int32_t)needed[k - 1];
        if (delta < -32 || delta > 31)
        {
            prc_free(ctx, bit_lengths);
            prc_free(ctx, needed);
            state->error = 1;
            prc_error(ctx, PRC_ERROR_PARSE, "Bit-length delta out of range in prc_bitwrite_compressed_indice_array\n");
            return -1;
        }
        bit_lengths[k] = (uint8_t)(delta & 0x3F);
    }

    /* matches: bit_lengths = prc_bitread_character_array(ctx, state, &size,
       6, has_comp_bit, num_known); (prc_bit.c 1543) -- the (possibly
       Huffman-compressed) delta table, written before any index value. */
    result = prc_bitwrite_character_array(ctx, state, bit_lengths, data_size, 6, has_comp_bit, num_known);

    /* matches: data[0] = prc_bitread_int_variable_bit(ctx, state, bit_count);
       (prc_bit.c 1570), bit_count == bit_lengths[0] == needed[0] here. */
    if (result == 0)
        if (prc_bitwrite_int_variable_bit(ctx, state, data[0], needed[0]) != 0)
            result = -1;

    /* matches: temp = prc_bitread_int_variable_bit(ctx, state,
       (uint8_t)bit_count); data[k] = temp + data[k-1]; (prc_bit.c 1578-1579)
       -- writes the DIFFERENCE (not data[k] itself) using needed[k]-1
       magnitude bits, mirroring prc_bitwrite_int_variable_bit's sign-then-
       magnitude layout directly (rather than going through that function)
       so a diff that doesn't fit in int32_t is never truncated. */
    for (k = 1; k < data_size && result == 0; k++)
    {
        int64_t diff = (int64_t)data[k] - (int64_t)data[k - 1];
        uint8_t sign = (diff < 0) ? 1 : 0;
        uint32_t magnitude = (uint32_t)(sign ? -diff : diff);

        if (prc_bitwrite_bit(ctx, state, sign) != 0 ||
            prc_bitwrite_uint_variable_bit(ctx, state, magnitude, needed[k] - 1) != 0)
        {
            result = -1;
        }
    }

    prc_free(ctx, bit_lengths);
    prc_free(ctx, needed);
    return result;
}
