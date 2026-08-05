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

//#define PRC_COUNT_BIT

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "prc_diag_env.h"
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_double.h"
#include "debug.h"
#include "prc_parse_common.h"

/* prc_next_bit and prc_bitread_bit now live in prc_bit.h as static inline
   (see PERF-1 in the review action plan) so every translation unit that reads
   the bitstream can inline the hottest per-bit call instead of paying a real
   cross-TU function call for it. */

inline static void
prc_next_huff_bit(prc_context *ctx, prc_bit_state *state)
{
    if (state->bitmask == 0x80)
    {
        state->ptr += 1;
        state->bitmask = 0x01;
    }
    else
    {
        state->bitmask <<= 1;
    }
}

void
prc_init_huff_bit_state(prc_context *ctx, prc_bit_state *state, uint8_t* ptr, int64_t max_bits)
{
    state->ptr = ptr;
    state->bitmask = 0x01;
    state->bit_count = max_bits;
    state->bit_position = 0;
    state->overrun = 0;
    state->curr_name = NULL;
}

void
prc_init_bit_state(prc_context *ctx, prc_bit_state *state, uint8_t *ptr, size_t byte_count)
{
    state->ptr = ptr;
    state->bitmask = 0x80;
    state->bit_count = (int64_t) byte_count * 8;
    state->bit_position = 0;
    state->overrun = 0;
    state->curr_name = NULL;

#if 1
   // if (ctx->reset_graphics_state)
   // {
        state->graphics_content.behavior_bit_field1 = 0;
        state->graphics_content.behavior_bit_field2 = 0;
        state->graphics_content.biased_index_of_line_style = 0;
        state->graphics_content.biased_layer_index = 0;
        state->graphics_content.has_entity_ref = 0;
       // ctx->reset_graphics_state = 0;
   // }
#endif
}

void prc_bitread_rewind(prc_context *ctx, prc_bit_state *state, uint8_t backup_bits)
{
    while (backup_bits > 0)
    {
        if (state->bitmask == 0x80)
        {
            state->ptr--;
            state->bitmask = 0x01;
            backup_bits = backup_bits - 1;
        }
        else
        {
            state->bitmask = state->bitmask << 1;
            backup_bits = backup_bits - 1;
        }
    }
}

/* The manual, breakpoint-only bitstream search aids that used to live here
   (prc_debug_stream, prc_debug_view_stream_bounding_box_search,
   prc_debug_view_stream_on_byte_boundary) moved to prc_bit_debug.c -- this
   file is the hottest per-bit read path in the parser (see PERF-1) and
   shouldn't carry ~150 lines of unused debug scaffolding inline. */

static uint8_t
prc_bitread_huff_bit(prc_context *ctx, prc_bit_state *state)
{
    uint8_t bit;

    if (PRC_UNLIKELY(state->bit_count <= 0))
    {
        /* Same rationale as prc_bitread_bit: num_values/num_leaves etc. are
           file-controlled and must not be able to walk this reader past the
           end of the huffman_array buffer. */
        state->overrun = 1;
        return 0;
    }

    bit = *(state->ptr) & state->bitmask;
    prc_next_huff_bit(ctx, state);
    state->bit_count--;

    if (bit > 0)
        return 1;
    else
        return 0;
}

uint8_t
prc_bitread_uint8(prc_context *ctx, prc_bit_state *state)
{
    uint8_t val = 0;
    int k;

    val |= prc_bitread_bit(ctx, state);
    for (k = 0; k < 7; k++)
    {
        val <<= 1;
        val |= prc_bitread_bit(ctx, state);
    }
    return val;
}

uint32_t
prc_bitread_uint32(prc_context *ctx, prc_bit_state *state)
{
    uint32_t val = 0;
    uint32_t pos = 0;

    while (prc_bitread_bit(ctx, state))
    {
        val |= (((uint32_t)(prc_bitread_uint8(ctx, state))) << (8 * pos++));
    }
    return val;
}

int32_t
prc_bitread_int32(prc_context *ctx, prc_bit_state *state)
{
    uint32_t val = 0;
    uint32_t pos = 0;

    while (prc_bitread_bit(ctx, state))
    {
        val |= (((uint32_t)(prc_bitread_uint8(ctx, state))) << (8 * pos++));
    }

    val <<= (4 - pos) * 8;
    val >>= (4 - pos) * 8;
    return val;
}

int32_t
prc_bitread_uncompressed_uint32(prc_context *ctx, prc_bit_state *state)
{
    uint32_t val;

    val = prc_bitread_uint8(ctx, state);
    val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 8);
    val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 16);
    val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 24);

    return val;
}

int
prc_bitread_string(prc_context *ctx, prc_bit_state *state, prc_string* string)
{
    unsigned int k;

    string->null_flag = prc_bitread_bit(ctx, state);
    string->string = NULL;
    if (string->null_flag)
    {
        string->size = prc_bitread_uint32(ctx, state);

        /* string->size is attacker-controlled. Guard against size+1 wrapping to 0
           (which would allocate a near-empty buffer while the fill loop below still
           runs `size` iterations), and against a size that couldn't possibly be
           backed by the remaining input (each character consumes at least 1 bit). */
        if (string->size == UINT32_MAX || (int64_t)string->size > state->bit_count)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Invalid string size in prc_bitread_string\n");
            return PRC_ERROR_PARSE;
        }

        string->string = (unsigned char*)prc_malloc(ctx, string->size + 1);
        if (string->string == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitread_string\n");
            return PRC_ERROR_MEMORY;
        }

        /* Lets hope the character array is not Huffman compressed... Spec is horrid */
        for (k = 0; k < string->size; k++)
        {
            string->string[k] = prc_bitread_uint8(ctx, state);
        }
        string->string[string->size] = 0x0;
        string->null_flag = 0;
    }
    return 0;
}

float
prc_bitread_float(prc_context *ctx, prc_bit_state *state)
{
    union float_uint val;

    val.uint_val = prc_bitread_uint8(ctx, state);
    val.uint_val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 8);
    val.uint_val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 16);
    val.uint_val += (((uint32_t)prc_bitread_uint8(ctx, state)) << 24);

    return val.float_val;
}

double
prc_bitread_double_with_variable_bit_number(prc_context *ctx, prc_bit_state *state,
    unsigned int number_of_bits, double tolerance)
{
    uint8_t is_negative = prc_bitread_bit(ctx, state);
    uint32_t mantissa = prc_bitread_uint_variable_bit(ctx, state, number_of_bits - 1);

    return tolerance * (is_negative ? -1.0 : 1.0) * (double)mantissa;
}

double
prc_bitread_double(prc_context *ctx, prc_bit_state *state)
{
    union ieee754_double value;
    value.d = 0;
    const sCodageOfFrequentDoubleOrExponent* pcofdoe;
    unsigned int ucofdoe = 0;
    for (int i = 1; i <= 22; ++i)
    {
        ucofdoe <<= 1;
        ucofdoe |= prc_bitread_bit(ctx, state);
        if ((pcofdoe = get_acofdoe_value(ctx, ucofdoe, i)) != NULL)
            break;
    }
    value.d = pcofdoe->u2uod.Value;

    // check if zero
    if (pcofdoe->NumberOfBits == 2 && pcofdoe->Bits == 1 && pcofdoe->Type == VT_double)
        return value.d;

    value.ieee.negative = prc_bitread_bit(ctx, state); // get sign

    if (pcofdoe->Type == VT_double) // double from list
        return value.d;

    if (prc_bitread_bit(ctx, state) == 0) // no mantissa
        return value.d;

    // read the mantissa
    // read uppermost 4 bits of mantissa
    unsigned char b4 = 0;
    for (int i = 0; i < 4; ++i)
    {
        b4 <<= 1;
        b4 |= prc_bitread_bit(ctx, state);
    }

#ifdef PRC_BIG_ENDIAN
    *((unsigned char*)(&value) + 1) |= b4;
    unsigned char* lastByte = (unsigned char*)(&value) + 7;
    unsigned char* currentByte = (unsigned char*)(&value) + 2;
#else
    *((unsigned char*)(&value) + 6) |= b4;
    unsigned char* lastByte = (unsigned char*)(&value) + 0;
    unsigned char* currentByte = (unsigned char*)(&value) + 5;
#endif

    for (; MOREBYTE(currentByte, lastByte); NEXTBYTE(currentByte))
    {
        if (prc_bitread_bit(ctx, state))
        {
            // new byte
            *currentByte = prc_bitread_uint8(ctx, state);
        }
        else
        {
            // get 3 bit offset
            unsigned int offset = 0;
            offset |= (prc_bitread_bit(ctx, state) << 2);
            offset |= (prc_bitread_bit(ctx, state) << 1);
            offset |= prc_bitread_bit(ctx, state);
            if (offset == 0)
            {
                // fill remaining bytes in mantissa with previous byte
                unsigned char pByte = BYTEAT(currentByte, 1);
                for (; MOREBYTE(currentByte, lastByte); NEXTBYTE(currentByte))
                    *currentByte = pByte;
                break;
            }
            else if (offset == 6)
            {
                // fill remaining bytes except last byte with previous byte
                unsigned char pByte = BYTEAT(currentByte, 1);
                PREVIOUSBYTE(lastByte);
                for (; MOREBYTE(currentByte, lastByte); NEXTBYTE(currentByte))
                    *currentByte = pByte;
                *currentByte = prc_bitread_uint8(ctx, state);
                break;
            }
            else
            {
                // one repeated byte
                //
                // offset is an untrusted 3-bit value (1-5 or 7 here; 0 and 6
                // are handled above) read straight from the input stream.
                // BYTEAT(currentByte, offset) dereferences currentByte+offset
                // (little-endian) or currentByte-offset (big-endian) with no
                // relation to how far the loop has actually progressed, so a
                // large offset early in the loop can point past the 8-byte
                // `value` union. Treat an out-of-range offset the same as the
                // offset==0 case (fill the remaining mantissa bytes with the
                // previous byte) instead of reading out of bounds.
                unsigned char *value_bytes = (unsigned char*)&value;
#ifdef PRC_BIG_ENDIAN
                uint8_t in_range = (currentByte - value_bytes) >= (ptrdiff_t)offset;
#else
                uint8_t in_range = (currentByte + offset) <= (value_bytes + sizeof(value) - 1);
#endif
                if (!in_range)
                {
                    unsigned char pByte = BYTEAT(currentByte, 1);
                    for (; MOREBYTE(currentByte, lastByte); NEXTBYTE(currentByte))
                        *currentByte = pByte;
                    break;
                }
                *currentByte = BYTEAT(currentByte, offset);
            }
        }
    }
    return value.d;
}

uint32_t
prc_bitread_huff_data(prc_context *ctx, prc_bit_state *state, uint32_t num_bits)
{
    uint32_t k;
    uint32_t val = 0;

    for (k = 0; k < num_bits; k++)
    {
        val += (prc_bitread_huff_bit(ctx, state) << k);
    }
    return val;
}

/* Huffman decoder. Can be used to decode data of different bit lengths on the leaves.
*  This particular implemenation can support char (1 byte) or short (2 byte) arrays
*  as input to the encoder, and compresses them using a number of bits that depends
*  upon the maximum value that the array contains.  This number of bits used,
*  is specified by the data type. For example, for the Edge_status_array the number
*  is equal to 2.  In that case, it is writing out a character array each with 2 bits
*  per character.  normal_angle_array is a shortarray with 16bits per character.
*  Data values are written into an elem_size-wide buffer (1 or 4 bytes -- the only
*  two widths any caller needs); data could have been smaller when compressed.
*  The characterArray, shortArray, character_array_compressed
*
*  Here is some types and bit sizes that the spec says:
*
*  CharacterArray
*  point_color_array --> CharacterArray  8 bits.  Is it compressed?
*  behaviors_array --> CharacterArray 8 bits.  Is it compressed?
*  edge_status_array --> CharacterArray with 2 bits per character. Is it huffman compressed?
*  character_array (character_array_compressed) 6 bits.  I guess if a character array is compressed it uses 6 bits.
*  point_reference_array --> CompressedIndiceArray.  An odd one with the flag of it being compressed specified if number_of_reference_points >= 3 is from CompressedIndiceArray type.
*  normal_angle_array (normal_angle_array_compressed) optionally compressed using normal_angle_number_of_bits.  Is should be lower than 16 and set to 10 for good performance
*
* ShortArray
* normal_angle_array, line_attribute_array, line_attribute_array  --> is compressed bit will tell if data is compressed and using huffman. But what is the number of bits used?
*
* point_array -> CompressedIntegerArray
*
* CompressedIndiceArray  --> invokes WriteCharacterArray but with out having written if the data is compressed or not. It is impled from the number of reference_points.
* point_reference_array, triangle_face_array
*
* This is the shared implementation behind prc_huffman_data_decoder (elem_size 4,
* used by prc_bitread_short_array) and the compressed branch of
* prc_bitread_character_array (elem_size 1) -- those two used to be ~250 lines
* of independently-maintained duplicated logic (S1 in the encode/decode review
* action plan), differing only in this output element width. Consolidated here
* so a future fix (like the num_leaves==0/max_code_length-error-code cleanup
* folded in below) only needs to happen once.
*/
static void *
prc_huffman_decode_core(prc_context *ctx, prc_bit_state *state, uint8_t num_bits,
    uint32_t huffman_array_size, size_t elem_size, uint32_t *data_decode_size)
{
    uint32_t k;
    void *data;
    uint32_t *huffman_array = NULL;
    uint32_t bits_last_integer;
    prc_bit_state huff_state;
    uint32_t num_leaves;
    uint32_t max_code_length;
    uint32_t *leaf_values;
    uint32_t *code_length;
    uint32_t *code_values;
    uint32_t bit_mask, bit;
    int j;
    prc_huff_node *root_node;
    prc_huff_node *curr_node;
    uint32_t num_values;
    uint32_t index;
    size_t max_bits;
    prc_huff_node *node_arena = NULL;
    uint64_t node_arena_capacity;
    uint64_t node_arena_used = 0;

    if (huffman_array_size == 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "huffman_array_size is zero\n");
        return NULL;
    }

    /* Each element is read via prc_bitread_uncompressed_uint32 below, a
       fixed 32 bits per word; reject a huffman_array_size that couldn't
       possibly be backed by what's left of the stream so a single
       corrupted length field can't drive an arbitrarily large calloc
       before any of it is validated (same pattern/precedent as the
       tess_count fix, prc_parse_file_structure.c ~line 749). */
    if (state->bit_count <= 0 ||
        (uint64_t)huffman_array_size > (uint64_t)state->bit_count / 32)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "huffman_array_size implausible for remaining section size\n");
        return NULL;
    }

    huffman_array = (uint32_t *)prc_calloc(ctx, huffman_array_size, sizeof(uint32_t));
    if (huffman_array == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman_array\n");
        return NULL;
    }

    for (k = 0; k < huffman_array_size; k++)
    {
        huffman_array[k] = prc_bitread_uncompressed_uint32(ctx, state);
    }
    bits_last_integer = prc_bitread_uint32(ctx, state);

    /* Total bits actually available in huffman_array. Everything read from
       huff_state below is bounded against this so a crafted num_leaves/num_values
       cannot walk the reader past the end of huffman_array. */
    max_bits = (size_t)(huffman_array_size - 1) * 32 + bits_last_integer;

    /* Now take the huffman array apart into the values, codes and code lengths */
    /* Everything is bit compressed.  And of course it is encoded different
       than the rest of the data, hence the special _huff_ calls.  The spec
       obfuscates this fact.  Madness. */
    prc_init_huff_bit_state(ctx, &huff_state, (uint8_t *)huffman_array, (int64_t)max_bits);

    /* First read in huffman array */
    num_leaves = prc_bitread_huff_data(ctx, &huff_state, num_bits + 1);

    max_code_length = prc_bitread_huff_data(ctx, &huff_state, 8);
    if (max_code_length == 0 || max_code_length > 32)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "huffman_array max_code_length is invalid\n");
        prc_free(ctx, huffman_array);
        return NULL;
    }

    /* Each leaf consumes at least num_bits + max_code_length bits; reject a
       leaf count that could not possibly be backed by the remaining data
       instead of allocating/looping on an attacker-chosen huge count, and
       reject zero leaves (used below as a node-array size). */
    if (huff_state.overrun || num_leaves == 0 ||
        (uint64_t)num_leaves * ((uint64_t)num_bits + max_code_length) > (uint64_t)max_bits)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "huffman_array num_leaves is invalid\n");
        prc_free(ctx, huffman_array);
        return NULL;
    }

    leaf_values = (uint32_t *)prc_calloc(ctx, num_leaves, sizeof(uint32_t));
    if (leaf_values == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman leaf_values\n");
        prc_free(ctx, huffman_array);
        return NULL;
    }

    code_length = (uint32_t *)prc_calloc(ctx, num_leaves, sizeof(uint32_t));
    if (code_length == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman code_length\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        return NULL;
    }

    code_values = (uint32_t *)prc_calloc(ctx, num_leaves, sizeof(uint32_t));
    if (code_values == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman code_values\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        prc_free(ctx, code_length);
        return NULL;
    }

    for (k = 0; k < num_leaves; k++)
    {
        leaf_values[k] = prc_bitread_huff_data(ctx, &huff_state, num_bits);
        code_length[k] = prc_bitread_huff_data(ctx, &huff_state, max_code_length);
        code_values[k] = prc_bitread_huff_data(ctx, &huff_state, code_length[k]);
    }

    /* DIAGNOSTIC (2026-07-24, PRC_DIAG_READ_HUFF_TABLE): dumps the actual
       leaf-value/code-length/code-value table as READ from an arbitrary
       (including third-party-encoded) file's huffman_array, for comparing
       table SHAPE against nanoPRC's own writer output -- added while
       cross-checking the mixed_chains/fan8 Acrobat blank-tree investigation
       against an independent encoder's file that hits the same superficial
       "22-bit value + tie among other leaves" condition yet works in real
       Acrobat. Zero behavior change when unset. */
    if (prc_diag_getenv("PRC_DIAG_READ_HUFF_TABLE") != NULL)
    {
        fprintf(stderr, "PRC_DIAG_READ_HUFF_TABLE: num_bits=%u max_code_length=%u num_leaves=%u\n",
            num_bits, max_code_length, num_leaves);
        for (k = 0; k < num_leaves; k++)
            fprintf(stderr, "PRC_DIAG_READ_HUFF_TABLE:   leaf_value=%u code_length=%u code_value=%u\n",
                leaf_values[k], code_length[k], code_values[k]);
    }

    /* Arena-allocate the tree nodes in one block instead of one prc_malloc per
       node (PERF-3, matches the encode side's node_bank in prc_huff.c). Each of
       the num_leaves root-to-leaf walks below creates at most one new node per
       bit of its code_length, plus the root itself, so 1 + sum(code_length[k])
       is an exact upper bound. The arena is sized once, up front, and never
       reallocated -- a mid-build realloc could move the block and invalidate
       already-issued node pointers (root_node, curr_node->left/right). */
    node_arena_capacity = 1;
    for (k = 0; k < num_leaves; k++)
        node_arena_capacity += code_length[k];

    if (node_arena_capacity > SIZE_MAX / sizeof(prc_huff_node))
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Huffman tree node count overflow\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        prc_free(ctx, code_length);
        prc_free(ctx, code_values);
        return NULL;
    }

    node_arena = (prc_huff_node *)prc_calloc(ctx, (size_t)node_arena_capacity, sizeof(prc_huff_node));
    if (node_arena == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman node_arena\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        prc_free(ctx, code_length);
        prc_free(ctx, code_values);
        return NULL;
    }

    /* Build the tree.  The idea here is to only construct the path to each leaf */
    root_node = &node_arena[node_arena_used++];

    for (k = 0; k < num_leaves; k++)
    {
        /* Traverse current tree. Adding nodes as need to get to leaf */
        curr_node = root_node;
        for (j = code_length[k] - 1; j >= 0; j--)
        {
            bit_mask = 1 << j;
            bit = code_values[k] & bit_mask;
            if (bit == 0)
            {
                /* Go left */
                if (curr_node->left == NULL)
                {
                    if (node_arena_used >= node_arena_capacity)
                    {
                        prc_error(ctx, PRC_ERROR_INTERNAL, "Huffman node arena exhausted\n");
                        prc_free(ctx, huffman_array);
                        prc_free(ctx, leaf_values);
                        prc_free(ctx, code_length);
                        prc_free(ctx, code_values);
                        prc_free(ctx, node_arena);
                        return NULL;
                    }
                    curr_node->left = &node_arena[node_arena_used++];
                }
                curr_node = curr_node->left;
            }
            else
            {
                /* Go right */
                if (curr_node->right == NULL)
                {
                    if (node_arena_used >= node_arena_capacity)
                    {
                        prc_error(ctx, PRC_ERROR_INTERNAL, "Huffman node arena exhausted\n");
                        prc_free(ctx, huffman_array);
                        prc_free(ctx, leaf_values);
                        prc_free(ctx, code_length);
                        prc_free(ctx, code_values);
                        prc_free(ctx, node_arena);
                        return NULL;
                    }
                    curr_node->right = &node_arena[node_arena_used++];
                }
                curr_node = curr_node->right;
            }
        }
        /* At a leaf */
        curr_node->is_leaf = 1;
        curr_node->value = leaf_values[k];
    }

    /* Now get the number of data values that were encoded */
    num_values = prc_bitread_huff_data(ctx, &huff_state, 32);

    /* Each decoded value consumes at least one bit; reject a count that could
       not possibly be backed by the remaining data instead of allocating on
       an attacker-chosen huge value. */
    if (huff_state.overrun || (uint64_t)num_values > (uint64_t)max_bits)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "huffman num_values is invalid\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        prc_free(ctx, code_length);
        prc_free(ctx, code_values);
        prc_free(ctx, node_arena);
        return NULL;
    }

    data = prc_calloc(ctx, num_values, elem_size);
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of huffman data\n");
        prc_free(ctx, huffman_array);
        prc_free(ctx, leaf_values);
        prc_free(ctx, code_length);
        prc_free(ctx, code_values);
        prc_free(ctx, node_arena);
        return NULL;
    }

    /* And finally the encoded values */
    index = 0;
    curr_node = root_node;
    while (index < num_values)
    {
        bit = prc_bitread_huff_bit(ctx, &huff_state);
        if (bit == 0)
        {
            curr_node = curr_node->left;
        }
        else
        {
            curr_node = curr_node->right;
        }
        if (curr_node == NULL || huff_state.overrun)
        {
            /* A crafted/incomplete code table can walk into an unbuilt branch,
               or the substream can run dry before num_values is reached.
               Fail cleanly instead of dereferencing NULL or spinning. */
            prc_error(ctx, PRC_ERROR_HUFFMAN, "Invalid huffman code encountered while decoding\n");
            prc_free(ctx, huffman_array);
            prc_free(ctx, leaf_values);
            prc_free(ctx, code_length);
            prc_free(ctx, code_values);
            prc_free(ctx, data);
            prc_free(ctx, node_arena);
            return NULL;
        }
        if (curr_node->is_leaf)
        {
            /* elem_size is always 1 (character array) or 4 (uint32_t array,
               matching prc_huff_node.value's own width) -- a plain narrowing
               cast for the 1-byte case is exactly what each original copy did
               for its own output type, so this preserves both byte-for-byte. */
            if (elem_size == sizeof(uint8_t))
                ((uint8_t *)data)[index] = (uint8_t)curr_node->value;
            else
                ((uint32_t *)data)[index] = curr_node->value;
            index += 1;
            curr_node = root_node;
        }
    }
    *data_decode_size = num_values;

    prc_free(ctx, huffman_array);
    prc_free(ctx, leaf_values);
    prc_free(ctx, code_length);
    prc_free(ctx, code_values);
    prc_free(ctx, node_arena);

    return data;
}

static uint32_t*
prc_huffman_data_decoder(prc_context *ctx, prc_bit_state *state, uint8_t num_bits, uint32_t huffman_array_size, uint32_t *data_decode_size)
{
    return (uint32_t *)prc_huffman_decode_core(ctx, state, num_bits, huffman_array_size, sizeof(uint32_t), data_decode_size);
}

/* 10.6 Inverse of Writecharacterarray. */
uint8_t*
prc_bitread_character_array(prc_context *ctx, prc_bit_state *state, uint32_t *data_size,
    uint8_t num_bits, uint8_t has_comp_bit, uint32_t num_known)
{
    uint8_t compressed;
    uint32_t k;
    uint8_t*data;
    uint32_t huffman_array_size;
    uint32_t array_size;

    /* Get the is_compressed bit. */
    if (has_comp_bit)
        compressed = prc_bitread_bit(ctx, state);

    /* Go ahead and read the size. This is the used to decide if the data was compressed or not for the indice array */
    array_size = prc_bitread_uint32(ctx, state);
    if (array_size == 0)
    {
        *data_size = 0;
        return NULL;
    }

    if (!has_comp_bit)
    {
        if (num_known <= 3) /* HTML doc has < 3 but <= 3 seems to be uncompressed */
        {
            compressed = false;
            *data_size = array_size;
        }
        else
        {
            compressed = true;
            huffman_array_size = array_size;
        }
    }
    else
    {
        if (compressed)
        {
            huffman_array_size = array_size;
        }
        else
        {
            *data_size = array_size;
        }
    }

    if (compressed)
    {
        if (huffman_array_size > 0)
        {
            /* Shared with prc_huffman_data_decoder -- see prc_huffman_decode_core. */
            data = (uint8_t *)prc_huffman_decode_core(ctx, state, num_bits, huffman_array_size, sizeof(uint8_t), data_size);
            return data;
        }
        else
        {
            *data_size = 0;  /* I don't think this is an error */
            //prc_error(ctx, PRC_ERROR_HUFFMAN, "Huffman array size is zero\n");
            return NULL;
        }
    }
    else
    {
        data = (uint8_t*)prc_malloc(ctx, sizeof(uint8_t) * *data_size);
        if (data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of prc_bitread_character_array data\n");
            return NULL;
        }

        for (k = 0; k < *data_size; k++)
        {
            data[k] = prc_bitread_uint8(ctx, state);
        }
        return data;
    }
    return NULL;
}

uint32_t
prc_bitread_uint_variable_bit(prc_context *ctx, prc_bit_state *state, uint32_t bit_length)
{
    uint32_t value = 0;
    uint32_t k;
    uint8_t bit;

    for (k = 0; k < bit_length; k++)
    {
        bit = prc_bitread_bit(ctx, state);
        if (bit)
            value += 1 << (bit_length - 1 - k);
    }
    return value;
}


int32_t
prc_bitread_int_variable_bit(prc_context *ctx, prc_bit_state *state, uint32_t bit_length)
{
    int32_t value;
    uint8_t sign;

    sign = prc_bitread_bit(ctx, state);
    value = (int32_t)prc_bitread_uint_variable_bit(ctx, state, bit_length - 1);

    if (sign)
        return -value;
    else
        return value;
}

/* A special version for dealin with the ana curves */
int
prc_bitread_compressed_entity_type_analoop(prc_context *ctx, prc_bit_state *state,
    uint32_t *entity_type)
{
    uint8_t bit0, bit1, bit2, bit3;
    uint32_t value;


    /* Read the first two bits */
    bit0 = prc_bitread_bit(ctx, state);
    bit1 = prc_bitread_bit(ctx, state);

    if (bit0 == 1 && bit1 == 1)
    {
        /* Read the next two bits */
        bit2 = prc_bitread_bit(ctx, state);
        bit3 = prc_bitread_bit(ctx, state);

        if (bit2 != 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_bit_read_compressed_entity_type\n", value);
            return PRC_ERROR_PARSE;
        }
        if (bit3 == 0)
        {
            *entity_type = PRC_HCG_Ellipse;
            return 0;
        }
        else
        {
            *entity_type = PRC_HCG_CompositeCurve;
            return 0;
        }
    }
    else
    {
        value = (bit0 << 1) | bit1;
        switch (value)
        {
        case 0:
            *entity_type = PRC_HCG_Line;
            return 0;

        case 1:
            *entity_type = PRC_HCG_Circle;
            return 0;

        case 2:
            *entity_type = PRC_HCG_BsplineHermiteCurve;
            return 0;

        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_bit_read_compressed_entity_type\n", value);
            return PRC_ERROR_PARSE;

        }
    }

}

/* This may need to be checked for bit order in the is_curve case */
int
prc_bitread_compressed_entity_type(prc_context *ctx, prc_bit_state *state,
                                    uint8_t *is_curve, uint32_t *entity_type)
{
    uint8_t bit0, bit1, bit2, bit3;
    uint32_t value;

    *is_curve = prc_bitread_bit(ctx, state);

    if (*is_curve)
    {
        /* Read the first two bits */
        bit0 = prc_bitread_bit(ctx, state);
        bit1 = prc_bitread_bit(ctx, state);

        if (bit0 == 1 && bit1 == 1)
        {
            /* Read the next two bits */
            bit2 = prc_bitread_bit(ctx, state);
            bit3 = prc_bitread_bit(ctx, state);

            if (bit2 != 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_bit_read_compressed_entity_type\n", value);
                return PRC_ERROR_PARSE;
            }
            if (bit3 == 0)
            {
                *entity_type = PRC_HCG_Ellipse;
                return 0;
            }
            else
            {
                *entity_type = PRC_HCG_CompositeCurve;
                return 0;
            }
        }
        else
        {
            value = (bit0 << 1) | bit1;
            switch (value)
            {
                case 0:
                    *entity_type = PRC_HCG_Line;
                    return 0;

                case 1:
                    *entity_type = PRC_HCG_Circle;
                    return 0;

                case 2:
                    *entity_type = PRC_HCG_BsplineHermiteCurve;
                    return 0;

                default:
                    prc_error(ctx, PRC_ERROR_PARSE, "Unknown entity type %d in prc_bit_read_compressed_entity_type\n", value);
                    return PRC_ERROR_PARSE;

            }
        }
    }
    else
    {
        *entity_type = prc_bitread_uint_variable_bit(ctx, state, 4);
    }
    return 0;
}

/* An odd one. Similar to the variable bit case but the number of bits is stored
   in the stream. 9.15 in the HTML document  */
uint32_t
prc_bitread_number_of_bits_then_unsigned_int(prc_context *ctx, prc_bit_state *state)
{
    /* The number of bits is written out with 5 bits itself */
    uint32_t bit_length = prc_bitread_uint_variable_bit(ctx, state, 5);
    return prc_bitread_uint_variable_bit(ctx, state, bit_length);
}

/* Inverse of 10.8 Writecompressedintegerarray */
uint32_t*
prc_bitread_compressed_integer_array(prc_context *ctx, prc_bit_state *state, uint32_t *data_size)
{
    uint8_t *bit_lengths;
    uint32_t size;
    uint32_t k;
    uint32_t *data;

    /* First get the bit length storages. These could be Huffman encoded */
    bit_lengths = prc_bitread_character_array(ctx, state, &size, 6, true, 0);
    if (bit_lengths == NULL)
        return NULL;

    data = (uint32_t*)prc_calloc(ctx, size, sizeof(uint32_t));
    if (data == NULL)
        return NULL;

    /* Now get the integers of variable lengths */
   // DEBUG_LOG("prc_bitread_compressed_integer_array\n");
    {
        uint8_t diag_bitpos = (prc_diag_getenv("PRC_DIAG_POINT_ARRAY_BITPOS") != NULL);
        for (k = 0; k < size; k++)
        {
            /* DIAGNOSTIC (2026-07-24, PRC_DIAG_POINT_ARRAY_BITPOS): dumps the
               exact bit_position (and its mod-8 byte-alignment phase) where
               each point_array value's field STARTS, before consuming it --
               added to check whether byte alignment (not the value/bit-length
               itself, already micro-analyzed and found clean) explains the
               fan8/mixed_chains Acrobat blank-tree bug's 22-bit boundary.
               Zero behavior change when unset. */
            if (diag_bitpos)
                fprintf(stderr, "PRC_DIAG_POINT_ARRAY_BITPOS: k=%u bit_position=%lld byte_offset=%lld bit_in_byte=%d bit_length=%u\n",
                    k, (long long)state->bit_position, (long long)(state->bit_position / 8),
                    (int)(state->bit_position % 8), bit_lengths[k]);
            data[k] = prc_bitread_int_variable_bit(ctx, state, bit_lengths[k]);
            //DEBUG_LOG("data[%d] = %d\n", k, data[k]);
       //     DEBUG_LOG("%d, ", data[k]); /* MATLAB style */
        }
    }

    prc_free(ctx, bit_lengths);

    *data_size = size;
    return data;
}

/* Inverse of 10.7 Writeshortarray */
int32_t*
prc_bitread_short_array(prc_context *ctx, prc_bit_state *state, uint32_t *data_size, uint8_t has_comp_bit, uint8_t number_of_bits)
{
    uint32_t size;
    uint8_t compressed;
    int32_t *output;

    /* Get the is_compressed bit. */
    if (has_comp_bit)
        compressed = prc_bitread_bit(ctx, state);
    else
        compressed = false;

    /* Get the storage size. This could be the size of the Huffman data or the array size */
    size = prc_bitread_uint32(ctx, state);
    if (size == 0)
        return NULL;

    /* Split between Huffman and non-Huffman case */
    if (compressed)
    {
        uint32_t *data;
        uint32_t j;

        data = prc_huffman_data_decoder(ctx, state, number_of_bits, size, data_size);
        if (data == NULL || *data_size == 0)
        {
            /* Force an error */
            *data_size = 1;
            prc_error(ctx, PRC_ERROR_HUFFMAN, "Error in huffman decoder\n");
            return NULL;
        }

        output = (int32_t *)prc_calloc(ctx, *data_size, sizeof(int32_t));
        if (output == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitread_short_array\n");
            return NULL;
        }

        memcpy(output, data, sizeof(int32_t) * (*data_size));
        prc_free(ctx, data);
    }
    else
    {
        uint32_t j;
        int32_t temp1, temp2;

        /* Need better error handling here */
        output = (int32_t *)prc_calloc(ctx, size, sizeof(int32_t));
        if (output == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_bitread_short_array\n");
            return NULL;
        }
        *data_size = size;

        for (j = 0; j < size; j++)
        {
            temp1 = prc_bitread_uint8(ctx, state);
            temp2 = prc_bitread_uint8(ctx, state);
            output[j] = temp1 + (temp2 << 8);
        }
    }
    return output;
}

/* Inverse of 10.9 Writecompressedindicearray. These are difference encoded */
int32_t*
prc_bitread_compressed_indice_array(prc_context *ctx, prc_bit_state *state,
                                    uint32_t *data_size, uint8_t has_comp_bit,
                                    uint32_t num_known)
{
    char *bit_lengths;
    uint32_t size;
    uint32_t k;
    int32_t *data;
    int32_t temp;
    char bit_count;

    /* First get the bit length storages. These could be Huffman encoded.
       They are difference encoded */
    /* This is an odd one. It is possible that there were no reference points and the size
       here is zero. I would suspect we would not do the readings here then. */
    bit_lengths = (char*) prc_bitread_character_array(ctx, state, &size, 6, has_comp_bit,
                                                      num_known);
    *data_size = size;
    if (bit_lengths == NULL || size == 0)
        return NULL;

    data = (int32_t*)prc_calloc(ctx, size, sizeof(int32_t));
    if (data == NULL)
        return NULL;

    /* This is not clear in the spec bit lengths must be less than 32 which
       is 1 for the sign bit and 31 for the unsignedinteger.  Make sure we
       are less than 32. If not, then the encoding is negative??? Looking
       carefully at this these are always 6 bits.  If they are negative
       the sixth bit (32) is set.  In that case, or this with 0xC0 to get
       the proper negative number */
    for (k = 0; k < size; k++)
    {
        if (bit_lengths[k] >= 32)
        {
            bit_lengths[k] = bit_lengths[k] | 0xc0;  /* Crazy this is not in the spec */
        }
    }

    /* Read in the first variable encoded value */
    bit_count = bit_lengths[0];
 //   DEBUG_LOG("\nprc_bitread_compressed_indice_array\n");
    data[0] = prc_bitread_int_variable_bit(ctx, state, (uint8_t) bit_count);
 //   DEBUG_LOG("data[0] = %d\n", data[0]);

    /* Now get the rest which are difference encoded integers of variable lengths */
    for (k = 1; k < size; k++)
    {
        bit_count += (int)bit_lengths[k];
       // bit_count = (int)bit_lengths[k];
        temp = prc_bitread_int_variable_bit(ctx, state, (uint8_t) bit_count);
        data[k] = temp + data[k - 1];
 //       DEBUG_LOG("data[%d] = %d\n", k, data[k]);
    }

    prc_free(ctx, bit_lengths);

    return data;
}
