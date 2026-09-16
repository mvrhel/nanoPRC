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

/* Phase 1a, Session 3 of 3 gate test: exercises every prc_bitwrite_* function
   against its prc_bitread_* counterpart (write(X) then read() == X), the
   sticky error flag, and prc_bitwrite_flush's byte-alignment padding. */

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_bit.h"

/* Not exposed via any header (see prc_context.c), matching that file's own
   "extern prc_hooks prc_hooks_default;" for the default allocator hooks. */
extern prc_hooks prc_hooks_default;

#if PRC_DEBUG_MEMORY
static size_t
count_outstanding(prc_context *ctx)
{
    size_t k, count = 0;
    for (k = 0; k < ctx->current_memory_index; k++)
        if (!ctx->debug_memory[k].is_free)
            count++;
    return count;
}
#define LEAK_CHECK_BEGIN(ctx) size_t leak_baseline = count_outstanding(ctx)
#define LEAK_CHECK_END(ctx) PRC_ASSERT_EQ(count_outstanding(ctx), leak_baseline)
#else
#define LEAK_CHECK_BEGIN(ctx) ((void)0)
#define LEAK_CHECK_END(ctx) ((void)0)
#endif

/* ------------------------------------------------------------------ */
/* 1. Primitive round-trips                                            */
/* ------------------------------------------------------------------ */

static void
test_uint8(prc_context *ctx)
{
    uint8_t values[] = { 0, 1, 127, 255 };
    size_t i;

    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        uint8_t out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_uint8(ctx, &w, values[i]), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_uint8(ctx, &r);
        PRC_ASSERT_EQ(out, values[i]);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

static void
test_uint32(prc_context *ctx)
{
    uint32_t values[] = { 0, 1, 0x7FFFFFFFu, 0xFFFFFFFFu };
    size_t i;

    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        uint32_t out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_uint32(ctx, &w, values[i]), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_uint32(ctx, &r);
        PRC_ASSERT_EQ(out, values[i]);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

/* Clause 9.12 spends the fewest bytes that preserve the sign, so the encoded
   length is part of the contract and not an implementation detail: a reader
   takes the sign from the last byte it is handed. Pinning the byte counts here
   is what distinguishes a conforming writer from one that merely round-trips
   against our own reader. 128 is the case that matters most -- written in one
   byte it reads back as -128. */
static void
test_int32_encoding_length(prc_context *ctx)
{
    struct { int32_t value; size_t bytes; } cases[] = {
        {           0, 0 },   /* the lone terminator bit, no data bytes */
        {           1, 1 },
        {         127, 1 },
        {         128, 2 },   /* not 1: a lone 0x80 is -128 */
        {         255, 2 },   /* not 1: a lone 0xFF is -1 */
        {         256, 2 },
        {       32767, 2 },
        {       32768, 3 },   /* not 2, for the same reason as 128 */
        {       65535, 3 },
        {          -1, 1 },   /* not 4, which is what we used to write */
        {        -128, 1 },
        {        -129, 2 },
        {        -256, 2 },
        {      -32768, 2 },
        {      -32769, 3 },
        {  INT32_MAX, 4 },
        {  INT32_MIN, 4 }
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        prc_bit_write_state w;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_int32(ctx, &w, cases[i].value), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        /* each data byte costs 9 bits (its tag bit plus the byte), and the run
           is closed by one more bit; the flush rounds up to whole bytes */
        PRC_ASSERT_EQ(w.byte_pos, (cases[i].bytes * 9 + 1 + 7) / 8);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

static void
test_int32(prc_context *ctx)
{
    int32_t values[] = { 0, 1, -1, 127, 128, 255, 256, 32768, 65535, -128, -129,
                         -32769, INT32_MIN, INT32_MAX };
    size_t i;

    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        int32_t out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_int32(ctx, &w, values[i]), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_int32(ctx, &r);
        PRC_ASSERT_EQ(out, values[i]);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

static void
test_double(prc_context *ctx)
{
    double values[] = { 0.0, 1.0, -1.0, 3.14159265358979, DBL_MAX, -DBL_MAX };
    size_t i;

    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        double out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
        PRC_ASSERT_EQ(prc_bitwrite_double(ctx, &w, values[i]), 0);
        PRC_ASSERT_EQ(w.error, 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_double(ctx, &r);
        PRC_ASSERT(out == values[i]);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

static void
test_float(prc_context *ctx)
{
    float values[] = { 0.0f, 1.0f, -1.0f, FLT_MAX };
    size_t i;

    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        float out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_float(ctx, &w, values[i]), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_float(ctx, &r);
        PRC_ASSERT(out == values[i]);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

static void
test_string_case(prc_context *ctx, const unsigned char *data, uint32_t size)
{
    prc_bit_write_state w;
    prc_bit_state r;
    prc_string in, out;
    LEAK_CHECK_BEGIN(ctx);

    in.null_flag = 0;
    in.size = size;
    in.string = (unsigned char *)data;

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
    PRC_ASSERT_EQ(prc_bitwrite_string(ctx, &w, &in), 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    memset(&out, 0, sizeof(out));
    PRC_ASSERT_EQ(prc_bitread_string(ctx, &r, &out), 0);
    PRC_ASSERT_EQ(out.size, size);
    PRC_ASSERT_NOT_NULL(out.string);
    /* memcmp, not strcmp: an embedded null byte is data, not a terminator */
    PRC_ASSERT(memcmp(out.string, data, size) == 0);

    prc_free(ctx, out.string);
    prc_bitwrite_release(ctx, &w);
    LEAK_CHECK_END(ctx);
}

static void
test_string(prc_context *ctx)
{
    unsigned char empty_buf[1] = { 0 };
    unsigned char hello_buf[] = "hello";
    unsigned char long_buf[255];
    unsigned char embedded_null_buf[] = { 'a', 'b', 0x00, 'c', 'd' };
    uint32_t i;

    for (i = 0; i < 255; i++)
        long_buf[i] = (unsigned char)(i % 256);

    test_string_case(ctx, empty_buf, 0);
    test_string_case(ctx, hello_buf, 5);
    test_string_case(ctx, long_buf, 255);
    test_string_case(ctx, embedded_null_buf, 5);
}

static void
test_uint_variable_bit(prc_context *ctx)
{
    struct { uint32_t value; uint32_t bits; } cases[] = {
        { 5, 3 }, { 1023, 10 }, { 0, 1 }
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        prc_bit_write_state w;
        prc_bit_state r;
        uint32_t out;
        LEAK_CHECK_BEGIN(ctx);

        PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
        PRC_ASSERT_EQ(prc_bitwrite_uint_variable_bit(ctx, &w, cases[i].value, cases[i].bits), 0);
        PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

        prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
        out = prc_bitread_uint_variable_bit(ctx, &r, cases[i].bits);
        PRC_ASSERT_EQ(out, cases[i].value);

        prc_bitwrite_release(ctx, &w);
        LEAK_CHECK_END(ctx);
    }
}

/* ------------------------------------------------------------------ */
/* 2. Compressed-array round-trips                                     */
/* ------------------------------------------------------------------ */

static const uint32_t ARRAY_SIZES[] = { 0, 1, 10, 1000 };
#define NUM_ARRAY_SIZES (sizeof(ARRAY_SIZES) / sizeof(ARRAY_SIZES[0]))

static void
test_character_array_of_size(prc_context *ctx, uint32_t count)
{
    uint8_t *data = NULL;
    uint8_t *out = NULL;
    uint32_t out_size = 0xFFFFFFFFu;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t i;
    LEAK_CHECK_BEGIN(ctx);

    if (count > 0)
    {
        data = (uint8_t *)malloc(count);
        for (i = 0; i < count; i++)
            data[i] = (uint8_t)((i * 37 + 5) % 64); /* fits num_bits == 6 */
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
    PRC_ASSERT_EQ(prc_bitwrite_character_array(ctx, &w, data, count, 6, 1, 0), 0);
    PRC_ASSERT_EQ(w.error, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    out = prc_bitread_character_array(ctx, &r, &out_size, 6, 1, 0);

    if (count == 0)
    {
        PRC_ASSERT_NULL(out);
        PRC_ASSERT_EQ(out_size, 0);
    }
    else
    {
        PRC_ASSERT_NOT_NULL(out);
        PRC_ASSERT_EQ(out_size, count);
        for (i = 0; i < count; i++)
            PRC_ASSERT_EQ(out[i], data[i]);
        prc_free(ctx, out);
    }

    prc_bitwrite_release(ctx, &w);
    if (data != NULL) free(data);
    LEAK_CHECK_END(ctx);
}

/* use_compressed selects has_comp_bit; when 0, the reader's plain branch
   only reconstructs the low 16 unsigned bits per element (prc_bit.c
   1518-1520), so the data used here stays within [0, 65535] for that case. */
static void
test_short_array_of_size(prc_context *ctx, uint32_t count, int use_compressed)
{
    int32_t *data = NULL;
    int32_t *out = NULL;
    uint32_t out_size = 0xFFFFFFFFu;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t i;
    LEAK_CHECK_BEGIN(ctx);

    /* Negative values are deliberately NOT used for the compressed branch:
       its Huffman leaf value is the raw bit pattern (see
       prc_bitwrite_short_array's memcpy-based widening), and a negative
       int32's two's-complement pattern needs all 32 bits (sign-extended
       through the top). That in turn would need number_of_bits == 32, but
       the reader's prc_bitread_huff_data(ctx, &huff_state, num_bits + 1)
       (prc_bit.c 526-537, used to read num_leaves) shifts a uint32_t by up
       to num_bits -- a shift of 32 there is undefined behavior, corrupting
       num_leaves. That's a pre-existing reader-side ceiling (num_bits <= 31
       for this path), not something this write-only session touches, so
       the compressed case here stays non-negative and within 20 bits
       instead -- still wider than the plain branch's 16-bit limit, so it
       still exercises a genuinely different value range. */
    if (count > 0)
    {
        data = (int32_t *)malloc(sizeof(int32_t) * count);
        for (i = 0; i < count; i++)
        {
            if (use_compressed)
                data[i] = (int32_t)((i * 12345u) % 1000000u);
            else
                data[i] = (int32_t)((i * 977u) % 65536u);
        }
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
    PRC_ASSERT_EQ(prc_bitwrite_short_array(ctx, &w, data, count, (uint8_t)use_compressed, 20), 0);
    PRC_ASSERT_EQ(w.error, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    out = prc_bitread_short_array(ctx, &r, &out_size, (uint8_t)use_compressed, 20);

    if (count == 0)
    {
        PRC_ASSERT_NULL(out);
    }
    else
    {
        PRC_ASSERT_NOT_NULL(out);
        PRC_ASSERT_EQ(out_size, count);
        for (i = 0; i < count; i++)
            PRC_ASSERT_EQ(out[i], data[i]);
        prc_free(ctx, out);
    }

    prc_bitwrite_release(ctx, &w);
    if (data != NULL) free(data);
    LEAK_CHECK_END(ctx);
}

static void
test_compressed_integer_array_of_size(prc_context *ctx, uint32_t count)
{
    int32_t *data = NULL;
    uint32_t *out = NULL;
    uint32_t out_size = 0xFFFFFFFFu;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t i;
    LEAK_CHECK_BEGIN(ctx);

    if (count > 0)
    {
        data = (int32_t *)malloc(sizeof(int32_t) * count);
        for (i = 0; i < count; i++)
            data[i] = (i % 2 == 0) ? (int32_t)(i * 123 - 500) : -(int32_t)(i * 7 + 1);
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
    PRC_ASSERT_EQ(prc_bitwrite_compressed_integer_array(ctx, &w, data, count), 0);
    PRC_ASSERT_EQ(w.error, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    out = prc_bitread_compressed_integer_array(ctx, &r, &out_size);

    if (count == 0)
    {
        PRC_ASSERT_NULL(out);
    }
    else
    {
        PRC_ASSERT_NOT_NULL(out);
        PRC_ASSERT_EQ(out_size, count);
        for (i = 0; i < count; i++)
        {
            int32_t got;
            memcpy(&got, &out[i], sizeof(got));
            PRC_ASSERT_EQ(got, data[i]);
        }
        prc_free(ctx, out);
    }

    prc_bitwrite_release(ctx, &w);
    if (data != NULL) free(data);
    LEAK_CHECK_END(ctx);
}

static void
test_compressed_indice_array_of_size(prc_context *ctx, uint32_t count)
{
    int32_t *data = NULL;
    int32_t *out = NULL;
    uint32_t out_size = 0xFFFFFFFFu;
    prc_bit_write_state w;
    prc_bit_state r;
    uint32_t i;
    LEAK_CHECK_BEGIN(ctx);

    if (count > 0)
    {
        data = (int32_t *)malloc(sizeof(int32_t) * count);
        data[0] = -12345; /* negative starting index: format permits it */
        for (i = 1; i < count; i++)
            data[i] = data[i - 1] + ((i % 3 == 0) ? -7 : (i % 5 == 0 ? 200 : 3));
    }

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 16), 0);
    PRC_ASSERT_EQ(prc_bitwrite_compressed_indice_array(ctx, &w, data, count, 1, 0), 0);
    PRC_ASSERT_EQ(w.error, 0);
    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    prc_init_bit_state(ctx, &r, w.buf, w.byte_pos);
    out = prc_bitread_compressed_indice_array(ctx, &r, &out_size, 1, 0);

    if (count == 0)
    {
        PRC_ASSERT_NULL(out);
    }
    else
    {
        PRC_ASSERT_NOT_NULL(out);
        PRC_ASSERT_EQ(out_size, count);
        for (i = 0; i < count; i++)
            PRC_ASSERT_EQ(out[i], data[i]);
        prc_free(ctx, out);
    }

    prc_bitwrite_release(ctx, &w);
    if (data != NULL) free(data);
    LEAK_CHECK_END(ctx);
}

static void
test_compressed_arrays(prc_context *ctx)
{
    size_t i;
    for (i = 0; i < NUM_ARRAY_SIZES; i++)
    {
        test_character_array_of_size(ctx, ARRAY_SIZES[i]);
        test_short_array_of_size(ctx, ARRAY_SIZES[i], 0);
        test_short_array_of_size(ctx, ARRAY_SIZES[i], 1);
        test_compressed_integer_array_of_size(ctx, ARRAY_SIZES[i]);
        test_compressed_indice_array_of_size(ctx, ARRAY_SIZES[i]);
    }
}

/* ------------------------------------------------------------------ */
/* 3. Sticky error flag                                                 */
/* ------------------------------------------------------------------ */

typedef struct
{
    prc_hooks real;
    int fail_next; /* if nonzero, the very next malloc/realloc call fails
                       (returns NULL) and this resets to 0 immediately --
                       i.e. exactly one allocation fails, not every one from
                       here on. This matters because the library's own
                       error-reporting path (prc_error -> prc_vferror)
                       allocates a small record for the exception itself;
                       if the injected failure stayed "on" indefinitely,
                       that allocation would fail too and cascade into
                       prc_vferror's own last-resort abort() path, which
                       tests a self-inflicted design bug in this hook
                       rather than prc_bitwrite's sticky-error behavior. */
} fail_hooks_state;

static void *
fail_malloc(void *opaque, size_t size)
{
    fail_hooks_state *fh = (fail_hooks_state *)opaque;
    if (fh->fail_next) { fh->fail_next = 0; return NULL; }
    return fh->real.malloc(fh->real.opaque, size);
}

static void *
fail_realloc(void *opaque, void *p, size_t size)
{
    fail_hooks_state *fh = (fail_hooks_state *)opaque;
    if (fh->fail_next) { fh->fail_next = 0; return NULL; }
    return fh->real.realloc(fh->real.opaque, p, size);
}

static void
fail_free(void *opaque, void *p)
{
    fail_hooks_state *fh = (fail_hooks_state *)opaque;
    fh->real.free(fh->real.opaque, p);
}

static void
test_sticky_error(void)
{
    fail_hooks_state fh;
    prc_hooks hooks;
    prc_context *fctx;
    prc_bit_write_state w;

    fh.real = prc_hooks_default;
    fh.fail_next = 0;
    hooks.opaque = &fh;
    hooks.malloc = fail_malloc;
    hooks.realloc = fail_realloc;
    hooks.free = fail_free;

    fctx = prc_new_context(&hooks);
    PRC_ASSERT_NOT_NULL(fctx);

    /* A tiny initial capacity so the very next multi-byte write forces
       prc_bitwrite_ensure_capacity's growth realloc. */
    PRC_ASSERT_EQ(prc_bitwrite_init(fctx, &w, 1), 0);
    PRC_ASSERT_EQ(w.error, 0);

    /* Fail exactly the next allocation call (that growth realloc); every
       call after it -- including any the library makes internally to
       report the failure -- succeeds normally again. */
    fh.fail_next = 1;

    PRC_ASSERT(prc_bitwrite_uint32(fctx, &w, 0xFFFFFFFFu) != 0);
    PRC_ASSERT(w.error != 0);

    /* Once set, error is sticky: subsequent calls are no-ops that also
       report failure, without needing (or causing) any further allocation
       at all. */
    PRC_ASSERT(prc_bitwrite_bit(fctx, &w, 1) != 0);
    PRC_ASSERT(prc_bitwrite_uint8(fctx, &w, 42) != 0);
    PRC_ASSERT(prc_bitwrite_flush(fctx, &w) != 0);
    PRC_ASSERT(w.error != 0);

    prc_bitwrite_release(fctx, &w);
    prc_release_context(fctx);
}

/* ------------------------------------------------------------------ */
/* 4. Flush byte-alignment                                              */
/* ------------------------------------------------------------------ */

static void
test_flush_padding(prc_context *ctx)
{
    prc_bit_write_state w;
    LEAK_CHECK_BEGIN(ctx);

    PRC_ASSERT_EQ(prc_bitwrite_init(ctx, &w, 8), 0);
    PRC_ASSERT_EQ(prc_bitwrite_bit(ctx, &w, 1), 0);
    PRC_ASSERT_EQ(prc_bitwrite_bit(ctx, &w, 0), 0);
    PRC_ASSERT_EQ(prc_bitwrite_bit(ctx, &w, 1), 0);

    /* Before flush: no full byte has been emitted yet. */
    PRC_ASSERT_EQ(w.byte_pos, 0);
    PRC_ASSERT_EQ(w.bit_fill, 3);

    PRC_ASSERT_EQ(prc_bitwrite_flush(ctx, &w), 0);

    PRC_ASSERT_EQ(w.byte_pos, 1);
    PRC_ASSERT_EQ(w.bit_fill, 0);
    /* MSB-first: bits 1,0,1 land in bits 7,6,5; the padded low 5 bits must
       be zero, i.e. buf[0] == 0xA0 exactly. */
    PRC_ASSERT_EQ(w.buf[0], 0xA0);
    PRC_ASSERT_EQ(w.buf[0] & 0x1F, 0);

    prc_bitwrite_release(ctx, &w);
    LEAK_CHECK_END(ctx);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("test_bitwrite");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_uint8(ctx);
    test_uint32(ctx);
    test_int32(ctx);
    test_int32_encoding_length(ctx);
    test_double(ctx);
    test_float(ctx);
    test_string(ctx);
    test_uint_variable_bit(ctx);
    test_compressed_arrays(ctx);
    test_flush_padding(ctx);

    prc_release_context(ctx);

    /* Uses its own context (with fault-injecting hooks), so it's run after
       releasing the main one rather than threading a second context through
       everything above. */
    test_sticky_error();

    PRC_TEST_END;
}
