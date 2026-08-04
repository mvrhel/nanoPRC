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

/* Manual, breakpoint-only bitstream search aids, relocated out of prc_bit.c
   (the hottest per-bit read path in the parser) so they don't clutter that
   file. None of these are called anywhere in the normal parse path -- set a
   breakpoint inside one, call it from a debugger with a live prc_bit_state,
   and inspect locals/memory while it walks the stream bit-by-bit looking for
   a recognizable tag/pattern. */

#include <stdio.h>
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_bit_debug.h"
#include "prc_parse_common.h"

/* Place a break point here and look at memory to search for tags when we get
   lost in the bit stream */
void
prc_debug_stream(prc_context *ctx, prc_bit_state *bit_state)
{
    int k;
    uint32_t value1, value2, value3, value4, value5, value6;
    uint8_t *ptr_curr;
    uint8_t bitmask_curr;
    int64_t bit_count;
    int64_t bit_pos;
    int code;
    uint8_t is_curve;

    for (k = 0; k < 7131600; k++)
    {
        /* Grab the state values */
        ptr_curr = bit_state->ptr;
        bitmask_curr = bit_state->bitmask;
        bit_count = bit_state->bit_count;
        bit_pos = bit_state->bit_position;

        /* Get the next uint32 and show in the debugger */
      //  value1 = prc_bitread_bit(ctx, bit_state);
       // value1 = prc_bitread_uint32(ctx, bit_state);
        value2 = prc_bitread_uint32(ctx, bit_state);


        //value2 = prc_bitread_uint_variable_bit(ctx, bit_state, 4);
        //code = prc_bitread_compressed_entity_type(ctx, bit_state, &is_curve, &value2);

        //value3 = prc_bitread_uint32(ctx, bit_state);
        //value4 = prc_bitread_uint32(ctx, bit_state);
       // value4 = prc_bitread_bit(ctx, state);
       // value5 = prc_bitread_bit(ctx, state);
      //  value6 = prc_bitread_uint32(ctx, state);

        //PRC_TYPE_GRAPH_Material
        //PRC_TYPE_GRAPH_TextureTransformation

        if (value2 == PRC_TYPE_TOPO_Connex ||
            value2 == PRC_TYPE_TOPO_Shell ||
            value2 == PRC_TYPE_TOPO_Face ||
            value2 == PRC_TYPE_TOPO_BrepData ||
            value2 == PRC_TYPE_TOPO_BrepDataCompress ||
            value2 == PRC_TYPE_TOPO_Context ||
            value2 == PRC_TYPE_TOPO_Item ||
            value2 == PRC_TYPE_TOPO_MultipleVertex ||
            value2 == PRC_TYPE_TOPO_UniqueVertex ||
            value2 == PRC_TYPE_TOPO_Body
            )
        {
            printf("Found value\n");
        }

        /* Reset the state */
        bit_state->ptr = ptr_curr;
        bit_state->bitmask = bitmask_curr;
        bit_state->bit_count = bit_count;
        bit_state->bit_position = bit_pos;

        /* Go forward 1 bit */
        prc_bitread_bit(ctx, bit_state);
    }
}

/* This does a debug where we check what the bounding box is from individual bit shifts */
void
prc_debug_view_stream_bounding_box_search(prc_context *ctx, prc_bit_state *state)
{
    uint32_t value;
    uint8_t *ptr_curr;
    uint8_t bitmask_curr;
    int64_t bit_count;
    int k, i, j;
    prc_bounding_box bounding_box;
    int code;

    for (k = 0; k < 7131600; k++)
    {
        /* Grab the state values */
        ptr_curr = state->ptr;
        bitmask_curr = state->bitmask;
        bit_count = state->bit_count;

        code = prc_parse_bound_box(ctx, state, &bounding_box);

        if (bounding_box.maximum_corner.x == 0 && bounding_box.maximum_corner.y == 0 && bounding_box.maximum_corner.z == 0 &&
            bounding_box.minimum_corner.x == 0 && bounding_box.minimum_corner.y == 0 && bounding_box.minimum_corner.z == 0)
        {
            printf("Found box\n");
        }

        /* Reset the state */
        state->ptr = ptr_curr;
        state->bitmask = bitmask_curr;
        state->bit_count = bit_count;

        /* Go forward 1 bit */
        prc_bitread_bit(ctx, state);
    }
}


/* This does a debug where it copies the next N bits into
   a memory pointer so that I can actually view the contents.
   Handy for looking for names for examples */
void
prc_debug_view_stream_on_byte_boundary(prc_context *ctx, prc_bit_state *state)
{
    uint32_t value;
    uint8_t *ptr_curr;
    uint8_t bitmask_curr;
    int64_t bit_count;
    unsigned char temp[10];
    int k, i, j;

    for (k = 0; k < 7131600; k++)
    {
        /* Grab the state values */
        ptr_curr = state->ptr;
        bitmask_curr = state->bitmask;
        bit_count = state->bit_count;

        for (i = 0; i < 10; i++)
        {
            temp[i] = 0;
            for (j = 0; j < 8; j++)
            {
                temp[i] <<= 1;
                temp[i] |= prc_bitread_bit(ctx, state);
            }
        }

        if (temp[0] == 'S' && temp[1] == 'o')
        {
            printf("Found string\n");
        }

        /* Reset the state */
        state->ptr = ptr_curr;
        state->bitmask = bitmask_curr;
        state->bit_count = bit_count;

        /* Go forward 1 bit */
        prc_bitread_bit(ctx, state);
    }
}
