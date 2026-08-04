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

#ifndef PRC_BIT_H
#define PRC_BIT_H

/* Branch-prediction hint for the hot per-bit overrun checks below (the
   non-overrun path is taken on every single bit of a well-formed stream).
   No-op on MSVC (which lacks __builtin_expect); a pure optimizer hint
   elsewhere, so it cannot change behavior, only misprediction cost. */
#if defined(__GNUC__) || defined(__clang__)
#define PRC_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define PRC_UNLIKELY(x) (x)
#endif

typedef struct prc_bit_state_s prc_bit_state;
struct prc_bit_state_s
{
	uint8_t *ptr;
	uint8_t bitmask;
	int64_t bit_count;
    int64_t bit_position;
	uint8_t overrun; /* set once a read is attempted past bit_count; callers should
	                    treat the stream as invalid once this is set */
	prc_name *curr_name;
	prc_graphics_content graphics_content;
};

typedef struct prc_huff_node_s prc_huff_node;
struct prc_huff_node_s
{
	prc_huff_node *left;
	prc_huff_node *right;
	uint32_t value;
	uint32_t code;
	uint32_t code_length;
	uint8_t is_leaf;
	prc_huff_node* next;
	prc_huff_node *prev;
	prc_huff_node *parent;
	int freq;
};

union float_uint
{
	float float_val;
	unsigned int uint_val;
};

/* prc_bitread_bit is the hottest call in the parser (once per stream bit);
   defining it (and its prc_next_bit helper) as static inline here, instead of
   as an external symbol in prc_bit.c, lets every translation unit that reads
   the bitstream inline it directly instead of paying a real call per bit.
   PRC_COUNT_BIT is a manual, single-TU debug toggle (see prc_bit.c) -- when
   defined before this header is included, each including TU gets its own
   independent bit_count, which matches its existing single-file-debug intent. */
#ifdef PRC_COUNT_BIT
#include <stdio.h>
static size_t bit_count = 0;
#endif

static inline void
prc_next_bit(prc_context *ctx, prc_bit_state *state)
{
	state->bitmask >>= 1;
	if (state->bitmask == 0)
	{
		state->ptr += 1;
		state->bitmask = 0x80;
	}
#ifdef PRC_COUNT_BIT
	bit_count++;
	printf("Bit pos %zu\n", bit_count);
#endif
}

static inline uint8_t
prc_bitread_bit(prc_context *ctx, prc_bit_state *state)
{
	uint8_t bit;

	if (PRC_UNLIKELY(state->bit_count <= 0))
	{
		/* File-supplied counts elsewhere in the parser can drive reads past the
		   end of this section's buffer. Once the declared length is exhausted,
		   stop dereferencing the buffer (which would walk off the allocation)
		   and report the overrun so callers can fail the parse instead. */
		state->overrun = 1;
		state->bit_position++;
		return 0;
	}

	bit = *(state->ptr) & state->bitmask;
	prc_next_bit(ctx, state);

	state->bit_count--;
	state->bit_position++;

	if (bit > 0)
		return 1;
	else
		return 0;
}

void prc_init_bit_state(prc_context* ctx, prc_bit_state* state, uint8_t* ptr, size_t byte_count);
uint8_t prc_bitread_uint8(prc_context* ctx, prc_bit_state* state);
uint32_t prc_bitread_uint32(prc_context* ctx, prc_bit_state* state);
int32_t prc_bitread_int32(prc_context* ctx, prc_bit_state* state);
double prc_bitread_double(prc_context* ctx, prc_bit_state* state);
int prc_bitread_string(prc_context* ctx, prc_bit_state* state, prc_string *data);
float prc_bitread_float(prc_context* ctx, prc_bit_state* state);
uint8_t* prc_bitread_character_array(prc_context* ctx, prc_bit_state* state, uint32_t* data_size, uint8_t bit_number, uint8_t has_comp_bit, uint32_t known_count);
uint32_t prc_bitread_uint_variable_bit(prc_context* ctx, prc_bit_state* state, uint32_t bit_length);
int32_t prc_bitread_int_variable_bit(prc_context* ctx, prc_bit_state* state, uint32_t bit_length);
uint32_t* prc_bitread_compressed_integer_array(prc_context* ctx, prc_bit_state* state, uint32_t* data_size);
int32_t* prc_bitread_compressed_indice_array(prc_context* ctx, prc_bit_state* state, uint32_t* data_size, uint8_t has_comp_bit, uint32_t known_count);
int32_t prc_bitread_uncompressed_uint32(prc_context* ctx, prc_bit_state* state);
uint32_t prc_bitread_huff_data(prc_context* ctx, prc_bit_state* state, uint32_t num_bits);
void prc_init_huff_bit_state(prc_context* ctx, prc_bit_state* state, uint8_t* ptr, int64_t max_bits);
void prc_bitread_rewind(prc_context *ctx, prc_bit_state *state, uint8_t backup_bits);
int32_t *prc_bitread_short_array(prc_context *ctx, prc_bit_state *state, uint32_t *data_size, uint8_t has_comp_bit, uint8_t number_of_bits);
uint32_t prc_bitread_number_of_bits_then_unsigned_int(prc_context* ctx, prc_bit_state* state);
int prc_bitread_compressed_entity_type(prc_context *ctx, prc_bit_state *state, uint8_t *is_curve, uint32_t *entity_type);
double prc_bitread_double_with_variable_bit_number(prc_context *ctx, prc_bit_state *state, unsigned int number_of_bits, double tolerance);

/* ---- Write path (prc_write_bit.c). Additive only: none of the prc_bit_state /
   prc_bitread_* declarations above are touched. Each prc_bitwrite_X function
   produces exactly the bit pattern its prc_bitread_X counterpart consumes;
   see prc_write_bit.c for the read/write pairing of each one. */

typedef struct prc_bit_write_state_s prc_bit_write_state;
struct prc_bit_write_state_s
{
    uint8_t  *buf;
    size_t    capacity;
    size_t    byte_pos;
    uint8_t   bit_accum;
    uint8_t   bit_fill;
    int       error;     /* sticky: non-zero means unrecoverable failure */
};

int prc_bitwrite_init(prc_context *ctx, prc_bit_write_state *state, size_t initial_capacity);
void prc_bitwrite_release(prc_context *ctx, prc_bit_write_state *state);
int prc_bitwrite_flush(prc_context *ctx, prc_bit_write_state *state);
int prc_bitwrite_bit(prc_context *ctx, prc_bit_write_state *state, uint8_t bit);
int prc_bitwrite_uint8(prc_context *ctx, prc_bit_write_state *state, uint8_t val);
int prc_bitwrite_uint32(prc_context *ctx, prc_bit_write_state *state, uint32_t val);
int prc_bitwrite_int32(prc_context *ctx, prc_bit_write_state *state, int32_t val);
int prc_bitwrite_double(prc_context *ctx, prc_bit_write_state *state, double val);
int prc_bitwrite_float(prc_context *ctx, prc_bit_write_state *state, float val);
int prc_bitwrite_string(prc_context *ctx, prc_bit_write_state *state, const prc_string *string);
int prc_bitwrite_uint_variable_bit(prc_context *ctx, prc_bit_write_state *state, uint32_t value, uint32_t bit_length);
int prc_bitwrite_int_variable_bit(prc_context *ctx, prc_bit_write_state *state, int32_t value, uint32_t bit_length);
int prc_bitwrite_compressed_integer_array(prc_context *ctx, prc_bit_write_state *state, const int32_t *data, uint32_t data_size);
int prc_bitwrite_character_array(prc_context *ctx, prc_bit_write_state *state, const uint8_t *data, uint32_t data_size, uint8_t num_bits, uint8_t has_comp_bit, uint32_t num_known);
int prc_bitwrite_short_array(prc_context *ctx, prc_bit_write_state *state, const int32_t *data, uint32_t data_size, uint8_t has_comp_bit, uint8_t number_of_bits);
int prc_bitwrite_compressed_indice_array(prc_context *ctx, prc_bit_write_state *state, const int32_t *data, uint32_t data_size, uint8_t has_comp_bit, uint32_t num_known);

#endif
