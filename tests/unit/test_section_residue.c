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

/* prc_check_section_consumed: the rule that separates a section a walk
   finished from one it merely stopped in the middle of.

   The motivating file cannot live here -- it is a third-party conformance
   fixture -- so these cases are hand-built buffers that reproduce its shape.
   The numbers in the "stopped short" case are the ones measured on that file:
   a 70-byte section, a walk that returned success at bit 194, and 366 bits of
   real content left behind. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "prc_test.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_parse_file_structure.h"

/* A bit_state positioned as though a walk had consumed `bits` bits of `buf`. */
static void
state_at(prc_context *ctx, prc_bit_state *state, uint8_t *buf, uint32_t size, int64_t bits)
{
    prc_init_bit_state(ctx, state, buf, size);
    state->bit_position = bits;
}

static void
test_fully_consumed(prc_context *ctx)
{
    uint8_t buf[8];
    prc_bit_state s;

    memset(buf, 0xAA, sizeof(buf));
    state_at(ctx, &s, buf, sizeof(buf), 64);
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"), 0);
}

/* Up to seven bits left is the padding a byte-aligned section ends on, and is
   accepted whatever those bits hold. */
static void
test_padding_is_accepted(prc_context *ctx)
{
    uint8_t buf[8];
    prc_bit_state s;
    int64_t bits;

    memset(buf, 0xFF, sizeof(buf));
    for (bits = 57; bits <= 64; bits++)
    {
        state_at(ctx, &s, buf, sizeof(buf), bits);
        PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"), 0);
    }
}

/* Ending well short is not by itself an error. 86 of the 2,816 geometry
   sections in the public corpus do it, 81 of them by exactly 81 bits, because
   one writer pads generously -- and every one of those tails is zero. */
static void
test_long_zero_tail_is_accepted(prc_context *ctx)
{
    uint8_t buf[13];              /* 104 bits, the size those sections declare */
    prc_bit_state s;

    memset(buf, 0, sizeof(buf));
    buf[0] = 0xAB;                /* content at the front, zeros after it */
    buf[1] = 0xCD;
    state_at(ctx, &s, buf, sizeof(buf), 23);   /* 81 bits short, as measured */
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"), 0);
}

/* The case this exists for: a walk that returned success having read a
   fraction of the section, with unread content behind it. */
static void
test_short_with_content_is_rejected(prc_context *ctx)
{
    uint8_t buf[70];              /* 560 bits */
    prc_bit_state s;

    memset(buf, 0, sizeof(buf));
    buf[40] = 0x01;               /* one set bit, well past where the walk stopped */
    state_at(ctx, &s, buf, sizeof(buf), 194);
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"),
                  PRC_ERROR_PARSE);
}

/* A single set bit anywhere in the tail is enough, including the very last one
   -- otherwise the scan could stop early and call content padding. */
static void
test_last_bit_of_tail_counts(prc_context *ctx)
{
    uint8_t buf[16];
    prc_bit_state s;

    memset(buf, 0, sizeof(buf));
    buf[15] = 0x01;               /* the final bit of the section */
    state_at(ctx, &s, buf, sizeof(buf), 8);
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"),
                  PRC_ERROR_PARSE);
}

/* Exactly eight bits left is where the rule starts applying: seven is padding,
   eight is a byte the walk did not read. */
static void
test_threshold_is_eight_bits(prc_context *ctx)
{
    uint8_t buf[8];
    prc_bit_state s;

    memset(buf, 0, sizeof(buf));
    buf[7] = 0xFF;

    state_at(ctx, &s, buf, sizeof(buf), 57);   /* 7 bits left: padding */
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"), 0);

    state_at(ctx, &s, buf, sizeof(buf), 56);   /* 8 bits left, and set: content */
    PRC_ASSERT_EQ(prc_check_section_consumed(ctx, &s, buf, sizeof(buf), "test"),
                  PRC_ERROR_PARSE);
}

int
main(void)
{
    prc_context *ctx;

    PRC_TEST_BEGIN("test_section_residue");

    ctx = prc_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    test_fully_consumed(ctx);
    test_padding_is_accepted(ctx);
    test_long_zero_tail_is_accepted(ctx);
    test_short_with_content_is_rejected(ctx);
    test_last_bit_of_tail_counts(ctx);
    test_threshold_is_eight_bits(ctx);

    prc_release_context(ctx);

    PRC_TEST_END;
}
