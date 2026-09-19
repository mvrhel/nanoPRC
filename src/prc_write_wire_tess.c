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

#include "prc_write_wire_tess.h"
#include "prc_data.h"

/* Exact-float-match vertex deduplication via a simple open-addressing hash
   table (linear probing, load factor <= 0.5). Positions are promoted to
   double on insert -- lossless for finite float values -- so bit-identical
   input floats always compare bit-identical as doubles, making direct
   double equality safe for dedup. */

#define PRC_WIRE_EMPTY_SLOT 0xFFFFFFFFu

static uint32_t
prc_wire_next_pow2(uint32_t v)
{
    v--;
    v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    v++;
    return v < 16u ? 16u : v;
}

static uint32_t
prc_wire_hash3(double x, double y, double z)
{
    double v[3];
    const unsigned char *b = (const unsigned char *)v;
    uint32_t h = 2166136261u;
    size_t i;

    v[0] = x; v[1] = y; v[2] = z;
    for (i = 0; i < sizeof(v); i++)
    {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t
prc_wire_dedup_insert(double *pool, uint32_t *pool_count, uint32_t *table,
    uint32_t table_cap, double x, double y, double z)
{
    uint32_t mask = table_cap - 1;
    uint32_t idx = prc_wire_hash3(x, y, z) & mask;

    for (;;)
    {
        if (table[idx] == PRC_WIRE_EMPTY_SLOT)
        {
            uint32_t new_index = *pool_count;

            pool[(size_t)new_index * 3 + 0] = x;
            pool[(size_t)new_index * 3 + 1] = y;
            pool[(size_t)new_index * 3 + 2] = z;
            *pool_count = new_index + 1;
            table[idx] = new_index;
            return new_index;
        }
        if (pool[(size_t)table[idx] * 3 + 0] == x &&
            pool[(size_t)table[idx] * 3 + 1] == y &&
            pool[(size_t)table[idx] * 3 + 2] == z)
        {
            return table[idx];
        }
        idx = (idx + 1) & mask;
    }
}

/* Convert a [0,1]-ish float color channel to the [0,255] uint8 range used
   by the on-disk RGB8 color encoding (prc_parse_rgb8), rounding and
   clamping. */
static uint8_t
prc_wire_color_channel(float c)
{
    double v = (double)c * 255.0 + 0.5;

    if (v < 0.0) v = 0.0;
    if (v > 255.0) v = 255.0;
    return (uint8_t)v;
}

static int
prc_write_wire_one_color(prc_context *ctx, prc_bit_write_state *s,
    uint8_t r, uint8_t g, uint8_t b, uint8_t a, uint8_t *first_written)
{
    if (!*first_written)
    {
        *first_written = 1;
    }
    else
    {
        if (prc_bitwrite_bit(ctx, s, 0) != 0) return -1;   /* is_same = never */
    }
    if (prc_bitwrite_uint8(ctx, s, r) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, s, g) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, s, b) != 0) return -1;
    if (prc_bitwrite_uint8(ctx, s, a) != 0) return -1;
    return 0;
}

int
prc_write_wire_tess(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_wire_element *elements, uint32_t num_elements)
{
    return prc_write_wire_tess_ex(ctx, s, elements, num_elements, 0);
}

int
prc_write_wire_tess_ex(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_wire_element *elements, uint32_t num_elements,
    int per_segment)
{
    uint32_t total_vertices = 0;
    uint32_t any_colors = 0;
    uint32_t closing_count = 0;
    uint32_t e, v, i;
    uint32_t table_cap;
    uint32_t *table = NULL;
    double *pool = NULL;
    uint32_t pool_count = 0;
    uint32_t *vert_idx = NULL;
    uint32_t vert_cursor;
    uint32_t number_of_wire_indexes;
    int ret = PRC_ERROR_INTERNAL;

    if (ctx == NULL || s == NULL || elements == NULL || num_elements == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_wire_tess: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    for (e = 0; e < num_elements; e++)
    {
        if (elements[e].positions == NULL || elements[e].num_vertices < 2)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_wire_tess: invalid element\n");
            return PRC_ERROR_INTERNAL;
        }
        total_vertices += elements[e].num_vertices;
        if (elements[e].colors != NULL)
            any_colors = 1;
        if (elements[e].is_closed)
            closing_count++;
    }

    table_cap = prc_wire_next_pow2(total_vertices * 2);
    table = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * table_cap);
    pool = (double *)prc_malloc(ctx, sizeof(double) * 3 * total_vertices);
    vert_idx = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * total_vertices);
    if (table == NULL || pool == NULL || vert_idx == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_wire_tess\n");
        goto cleanup;
    }
    for (i = 0; i < table_cap; i++)
        table[i] = PRC_WIRE_EMPTY_SLOT;

    vert_cursor = 0;
    for (e = 0; e < num_elements; e++)
    {
        for (v = 0; v < elements[e].num_vertices; v++)
        {
            const float *p = &elements[e].positions[(size_t)v * 3];

            vert_idx[vert_cursor++] = prc_wire_dedup_insert(pool, &pool_count, table,
                table_cap, (double)p[0], (double)p[1], (double)p[2]);
        }
    }

    /* number_of_wire_indexes is a total WORD count: one header word plus
       each element's vertex indices, summed across all elements. */
    number_of_wire_indexes = num_elements + total_vertices;

    if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                          /* is_calculated */
    if (prc_bitwrite_uint32(ctx, s, pool_count * 3) != 0) goto fail;          /* number_of_coordinates */
    for (i = 0; i < pool_count * 3; i++)
        if (prc_bitwrite_double(ctx, s, pool[i]) != 0) goto fail;

    if (prc_bitwrite_uint32(ctx, s, number_of_wire_indexes) != 0) goto fail;

    vert_cursor = 0;
    for (e = 0; e < num_elements; e++)
    {
        uint32_t n = elements[e].num_vertices;
        uint32_t flags = 0;

        if (elements[e].is_closed) flags |= (uint32_t)PRC_3DWIRETESSDATA_IsClosing;
        if (elements[e].is_continuous) flags |= (uint32_t)PRC_3DWIRETESSDATA_IsContinuous;

        if (prc_bitwrite_uint32(ctx, s, (n & 0x0FFFFFFFu) | flags) != 0) goto fail;
        for (v = 0; v < n; v++)
            if (prc_bitwrite_uint32(ctx, s, vert_idx[vert_cursor++] * 3) != 0) goto fail;
    }

    if (prc_bitwrite_bit(ctx, s, any_colors ? 1 : 0) != 0) goto fail;         /* has_vertex_colors */
    if (any_colors)
    {
        uint8_t first_written = 0;

        if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;                     /* is_rgba */
        if (prc_bitwrite_bit(ctx, s, per_segment ? 1 : 0) != 0) goto fail;   /* is_segment_color */
        if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                     /* b_optimized */

        for (e = 0; e < num_elements; e++)
        {
            uint32_t n = elements[e].num_vertices;
            uint8_t r0 = 255, g0 = 255, b0 = 255, a0 = 255;

            /* Per SEGMENT rather than per point: exactly one fewer entry per
               element, which is how the read side derives it --
               segment_color_count = vertex_color_count - number_of_wire_elements
               (prc_parse_tess.c). An open run of n points has n-1 segments; a
               closing one has n, counting the wrap back to the first point.
               Each segment takes the colour of the point it starts at, so the
               loop below simply stops one short and the closing element's
               extra entry is not written. */
            if (per_segment)
            {
                uint32_t segs = elements[e].is_closed ? n : (n > 0 ? n - 1 : 0);

                for (v = 0; v < segs; v++)
                {
                    uint8_t r = 255, g = 255, b = 255, a = 255;

                    if (elements[e].colors != NULL)
                    {
                        const float *c = &elements[e].colors[(size_t)v * 4];

                        r = prc_wire_color_channel(c[0]);
                        g = prc_wire_color_channel(c[1]);
                        b = prc_wire_color_channel(c[2]);
                        a = prc_wire_color_channel(c[3]);
                    }
                    if (prc_write_wire_one_color(ctx, s, r, g, b, a, &first_written) != 0)
                        goto fail;
                }
                continue;
            }

            for (v = 0; v < n; v++)
            {
                uint8_t r = 255, g = 255, b = 255, a = 255;

                if (elements[e].colors != NULL)
                {
                    const float *c = &elements[e].colors[(size_t)v * 4];

                    r = prc_wire_color_channel(c[0]);
                    g = prc_wire_color_channel(c[1]);
                    b = prc_wire_color_channel(c[2]);
                    a = prc_wire_color_channel(c[3]);
                }
                if (v == 0) { r0 = r; g0 = g; b0 = b; a0 = a; }

                if (prc_write_wire_one_color(ctx, s, r, g, b, a, &first_written) != 0)
                    goto fail;
            }

            if (elements[e].is_closed)
            {
                /* Closed loops need one extra color entry for the
                   wrap-around edge back to the first vertex (matches the
                   parser's "+1 if IsClosing" vertex_color_count accounting):
                   duplicate that vertex's own color. */
                if (prc_write_wire_one_color(ctx, s, r0, g0, b0, a0, &first_written) != 0)
                    goto fail;
            }
        }
    }

    ret = 0;
    goto cleanup;

fail:
    ret = s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;

cleanup:
    if (table != NULL) prc_free(ctx, table);
    if (pool != NULL) prc_free(ctx, pool);
    if (vert_idx != NULL) prc_free(ctx, vert_idx);
    return ret;
}
