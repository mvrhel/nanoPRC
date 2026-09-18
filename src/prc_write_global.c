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

#include <string.h>
#include "prc_write_global.h"
#include "prc_data.h"

int
prc_write_global_tables_init(prc_context *ctx, prc_write_global_tables *tables)
{
    if (ctx == NULL || tables == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_global_tables_init: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }
    memset(tables, 0, sizeof(*tables));
    return 0;
}

void
prc_write_global_tables_free(prc_context *ctx, prc_write_global_tables *tables)
{
    if (tables == NULL)
        return;
    if (tables->colors != NULL) prc_free(ctx, tables->colors);
    if (tables->materials != NULL) prc_free(ctx, tables->materials);
    if (tables->pictures != NULL) prc_free(ctx, tables->pictures);
    if (tables->styles != NULL) prc_free(ctx, tables->styles);
    memset(tables, 0, sizeof(*tables));
}

/* Doubles capacity (starting at 4) for one of the four table arrays. Only
   grows when count has caught up to cap, so callers just check-and-append. */
static int
prc_write_global_array_grow(prc_context *ctx, void **arr, uint32_t *cap, uint32_t count, size_t elem_size)
{
    uint32_t new_cap;
    void *new_arr;

    if (count < *cap)
        return 0;

    new_cap = (*cap == 0) ? 4u : (*cap * 2u);
    new_arr = prc_realloc(ctx, *arr, (size_t)new_cap * elem_size);
    if (new_arr == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_global_tables\n");
        return -1;
    }
    *arr = new_arr;
    *cap = new_cap;
    return 0;
}

/* Adds a colour, deduplicating, and returns the 1-biased index that every
   colour-referencing field expects. 0 means failure.

   The index counts DOUBLES, not colour entries: the Nth colour is index
   N*3+1, because the array is addressed by its individual components. The
   specification describes these fields as "an index into the RGB array",
   which reads as an entry index and is what this function used to return --
   real readers divide by three. Our own does, in prc_style_api.c:

       color_index = (graph_material.biased_ambient_index - 1) / 3;

   The entry-index form was harmless while only one real colour existed, since
   entry 0 is index 1 either way, which is why it survived until per-item
   materials added a second. Reported against the specification's wording as
   pdf-association/pdf-issues#816. */
uint32_t
prc_write_color_add(prc_context *ctx, prc_write_global_tables *tables, const prc_rgb_color *color)
{
    uint32_t i;

    if (ctx == NULL || tables == NULL || color == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_color_add: invalid arguments\n");
        return 0;
    }

    for (i = 0; i < tables->color_count; i++)
    {
        if (tables->colors[i].red == color->red &&
            tables->colors[i].green == color->green &&
            tables->colors[i].blue == color->blue)
            return i * 3 + 1;
    }

    if (prc_write_global_array_grow(ctx, (void **)&tables->colors, &tables->color_cap,
            tables->color_count, sizeof(prc_rgb_color)) != 0)
        return 0;

    tables->colors[tables->color_count] = *color;
    tables->color_count++;
    return (tables->color_count - 1) * 3 + 1;
}

static int
prc_write_material_equal(const prc_graph_material *a, const prc_graph_material *b)
{
    if (a->tag != b->tag)
        return 0;
    if (a->tag == PRC_TYPE_GRAPH_Material)
    {
        return a->biased_ambient_index == b->biased_ambient_index &&
               a->biased_diffuse_index == b->biased_diffuse_index &&
               a->biased_emissive_index == b->biased_emissive_index &&
               a->biased_specular_index == b->biased_specular_index &&
               a->shininess == b->shininess &&
               a->ambient_alpha == b->ambient_alpha &&
               a->diffuse_alpha == b->diffuse_alpha &&
               a->emissive_alpha == b->emissive_alpha &&
               a->specular_alpha == b->specular_alpha;
    }
    return a->biased_material_generic_index == b->biased_material_generic_index &&
           a->biased_texture_definition_index == b->biased_texture_definition_index &&
           a->biased_next_texture_index == b->biased_next_texture_index &&
           a->biased_uv_coordinates_index == b->biased_uv_coordinates_index;
}

uint32_t
prc_write_material_add(prc_context *ctx, prc_write_global_tables *tables, const prc_graph_material *material)
{
    uint32_t i;

    if (ctx == NULL || tables == NULL || material == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_material_add: invalid arguments\n");
        return 0;
    }
    if (material->tag != PRC_TYPE_GRAPH_Material && material->tag != PRC_TYPE_GRAPH_TextureApplication)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_material_add: unknown material tag\n");
        return 0;
    }

    for (i = 0; i < tables->material_count; i++)
    {
        if (prc_write_material_equal(&tables->materials[i], material))
            return i + 1;
    }

    if (prc_write_global_array_grow(ctx, (void **)&tables->materials, &tables->material_cap,
            tables->material_count, sizeof(prc_graph_material)) != 0)
        return 0;

    tables->materials[tables->material_count] = *material;
    tables->material_count++;
    return tables->material_count;
}

static int
prc_write_style_equal(const prc_graph_style *a, const prc_graph_style *b)
{
    if (a->is_material != b->is_material)
        return 0;
    if (a->biased_color_index != b->biased_color_index)
        return 0;
    if (a->is_transparency != b->is_transparency)
        return 0;
    if (a->is_transparency && a->transparency != b->transparency)
        return 0;
    if (a->is_rendering_parameters != b->is_rendering_parameters)
        return 0;
    if (a->is_rendering_parameters && a->rendering_parameters != b->rendering_parameters)
        return 0;
    return 1;
}

uint32_t
prc_write_style_add(prc_context *ctx, prc_write_global_tables *tables, const prc_graph_style *style)
{
    uint32_t i;
    prc_graph_style entry;

    if (ctx == NULL || tables == NULL || style == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_style_add: invalid arguments\n");
        return 0;
    }

    for (i = 0; i < tables->style_count; i++)
    {
        if (prc_write_style_equal(&tables->styles[i], style))
            return i + 1;
    }

    if (prc_write_global_array_grow(ctx, (void **)&tables->styles, &tables->style_cap,
            tables->style_count, sizeof(prc_graph_style)) != 0)
        return 0;

    entry = *style;
    entry.tag = PRC_TYPE_GRAPH_Style;
    tables->styles[tables->style_count] = entry;
    tables->style_count++;
    return tables->style_count;
}

/* Table 93's PNG signature + IHDR chunk: 8-byte magic, 4-byte chunk length,
   4-byte "IHDR" type, then width/height as big-endian uint32 (bytes 16-23). */
static int
prc_write_parse_png_dims(const uint8_t *data, size_t size, uint32_t *width, uint32_t *height)
{
    static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n' };

    if (data == NULL || size < 24)
        return -1;
    if (memcmp(data, sig, 8) != 0)
        return -1;
    if (memcmp(data + 12, "IHDR", 4) != 0)
        return -1;

    *width  = ((uint32_t)data[16] << 24) | ((uint32_t)data[17] << 16) |
              ((uint32_t)data[18] << 8)  |  (uint32_t)data[19];
    *height = ((uint32_t)data[20] << 24) | ((uint32_t)data[21] << 16) |
              ((uint32_t)data[22] << 8)  |  (uint32_t)data[23];
    return 0;
}

/* Minimal JPEG marker scanner: validates the SOI marker, then walks marker
   segments looking for a SOFn (baseline/progressive/etc, excluding DHT/JPG/
   DAC which share the 0xC4/0xC8/0xCC codes in the same range) to read its
   precision/height/width triple. Does not attempt to decode entropy data. */
static int
prc_write_parse_jpeg_dims(const uint8_t *data, size_t size, uint32_t *width, uint32_t *height)
{
    size_t pos;

    if (data == NULL || size < 4 || data[0] != 0xFF || data[1] != 0xD8)
        return -1;

    pos = 2;
    while (pos < size)
    {
        uint32_t seg_len;
        uint8_t marker;

        if (data[pos] != 0xFF)
        {
            pos++;
            continue;
        }
        while (pos < size && data[pos] == 0xFF)
            pos++;
        if (pos >= size)
            break;
        marker = data[pos];
        pos++;

        if (marker == 0xD8 || marker == 0xD9)
            continue;
        if (marker >= 0xD0 && marker <= 0xD7)
            continue;

        if (pos + 2 > size)
            break;
        seg_len = ((uint32_t)data[pos] << 8) | data[pos + 1];
        if (seg_len < 2 || pos + seg_len > size)
            break;

        if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC)
        {
            if (pos + 7 > size)
                return -1;
            *height = ((uint32_t)data[pos + 3] << 8) | data[pos + 4];
            *width  = ((uint32_t)data[pos + 5] << 8) | data[pos + 6];
            return 0;
        }

        pos += seg_len;
    }
    return -1;
}

uint32_t
prc_write_picture_add(prc_context *ctx, prc_write_global_tables *tables, const prc_write_picture *picture)
{
    prc_graph_picture entry;
    uint32_t width = 0, height = 0;

    if (ctx == NULL || tables == NULL || picture == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_picture_add: invalid arguments\n");
        return 0;
    }

    memset(&entry, 0, sizeof(entry));
    entry.tag = PRC_TYPE_GRAPH_Picture;

    switch (picture->format)
    {
    case PRC_WRITE_PIX_RGB:
        if (picture->data == NULL || picture->width == 0 || picture->height == 0 ||
            picture->data_size < (size_t)picture->width * picture->height * 3)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_picture_add: invalid RGB picture\n");
            return 0;
        }
        entry.format = KEPRCPicture_BITMAP_RGB_BYTE;
        width = picture->width;
        height = picture->height;
        entry.num_elements_per_pixel = 3;
        break;

    case PRC_WRITE_PIX_RGBA:
        if (picture->data == NULL || picture->width == 0 || picture->height == 0 ||
            picture->data_size < (size_t)picture->width * picture->height * 4)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_picture_add: invalid RGBA picture\n");
            return 0;
        }
        entry.format = KEPRCPicture_BITMAP_RGBA_BYTE;
        width = picture->width;
        height = picture->height;
        entry.num_elements_per_pixel = 4;
        break;

    case PRC_WRITE_PIX_PNG:
        if (prc_write_parse_png_dims(picture->data, picture->data_size, &width, &height) != 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "prc_write_picture_add: invalid PNG data\n");
            return 0;
        }
        entry.format = KEPRCPicture_PNG;
        break;

    case PRC_WRITE_PIX_JPEG:
        if (prc_write_parse_jpeg_dims(picture->data, picture->data_size, &width, &height) != 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "prc_write_picture_add: invalid JPEG data\n");
            return 0;
        }
        entry.format = KEPRCPicture_JPG;
        break;

    default:
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_picture_add: unknown pixel format\n");
        return 0;
    }

    /* The globals section stores only Table 93's format/dimensions
       metadata -- see the header comment on prc_write_picture_add. */
    entry.biased_uncompressed_file_index = 0;
    entry.pixel_width = width;
    entry.pixel_height = height;

    if (prc_write_global_array_grow(ctx, (void **)&tables->pictures, &tables->picture_cap,
            tables->picture_count, sizeof(prc_graph_picture)) != 0)
        return 0;

    tables->pictures[tables->picture_count] = entry;
    tables->picture_count++;
    return tables->picture_count;
}

/* ContentPRCBase (Table 28): empty attribute list + a "same as inherited"
   name -- this encoder does not support named/attributed global entities. */
static int
prc_write_content_prc_base(prc_context *ctx, prc_bit_write_state *s)
{
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;  /* attribute_count */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) return -1;      /* name.same */
    return 0;
}

/* ContentPRCRefBase (Table 29): as above, plus the three id fields. */
static int
prc_write_content_prc_ref_base(prc_context *ctx, prc_bit_write_state *s, const prc_content_prc_ref_base *base)
{
    if (prc_write_content_prc_base(ctx, s) != 0) return -1;
    if (prc_bitwrite_uint32(ctx, s, base->nonpersistent_id_cad) != 0) return -1;
    if (prc_bitwrite_uint32(ctx, s, base->unique_id_cad) != 0) return -1;
    if (prc_bitwrite_uint32(ctx, s, base->unique_id) != 0) return -1;
    return 0;
}

int
prc_write_globals_to_stream(prc_context *ctx, prc_bit_write_state *s, const prc_write_global_tables *tables)
{
    uint32_t i;

    if (ctx == NULL || s == NULL || tables == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_globals_to_stream: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    /* tess_chord / tess_angle: tessellation-quality globals. Previously
       written as 0.0/0.0 ("harmless defaults" -- untested assumption).
       Two independently-produced real files both show non-zero values
       here (chord 600/2000, angle 40/40 -- the angle agreeing exactly
       across two different generators strongly suggests a shared,
       widely-used default), while every file this write facility has
       produced left both at a literal zero -- which a stricter reader
       could plausibly read as "zero tolerance", i.e. a degenerate/invalid
       quality target, rather than "unset". Match the real convention. */
    if (prc_bitwrite_double(ctx, s, 2000.0) != 0) goto fail;
    if (prc_bitwrite_double(ctx, s, 40.0) != 0) goto fail;

    /* serialize_help: no markup font-key data in this phase */
    if (prc_bitwrite_string(ctx, s, NULL) != 0) goto fail;   /* default_font_family_name: absent */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;      /* font_keys_count */

    /* colors -- prc_parse_rgb(..., has_alpha=false): 3 doubles/entry, no alpha */
    if (prc_bitwrite_uint32(ctx, s, tables->color_count) != 0) goto fail;
    for (i = 0; i < tables->color_count; i++)
    {
        if (prc_bitwrite_double(ctx, s, tables->colors[i].red) != 0) goto fail;
        if (prc_bitwrite_double(ctx, s, tables->colors[i].green) != 0) goto fail;
        if (prc_bitwrite_double(ctx, s, tables->colors[i].blue) != 0) goto fail;
    }

    /* pictures (Table 93 metadata only -- see prc_write_picture_add) */
    if (prc_bitwrite_uint32(ctx, s, tables->picture_count) != 0) goto fail;
    for (i = 0; i < tables->picture_count; i++)
    {
        const prc_graph_picture *p = &tables->pictures[i];

        if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_GRAPH_Picture) != 0) goto fail;
        if (prc_write_content_prc_base(ctx, s) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, (uint32_t)p->format) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, p->biased_uncompressed_file_index) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, p->pixel_width) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, p->pixel_height) != 0) goto fail;
    }

    /* texture_definition table: out of this session's scope -- always empty */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;

    /* materials */
    if (prc_bitwrite_uint32(ctx, s, tables->material_count) != 0) goto fail;
    for (i = 0; i < tables->material_count; i++)
    {
        const prc_graph_material *m = &tables->materials[i];

        if (prc_bitwrite_uint32(ctx, s, m->tag) != 0) goto fail;
        if (prc_write_content_prc_ref_base(ctx, s, &m->base) != 0) goto fail;

        if (m->tag == PRC_TYPE_GRAPH_Material)
        {
            if (prc_bitwrite_uint32(ctx, s, m->biased_ambient_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_diffuse_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_emissive_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_specular_index) != 0) goto fail;
            if (prc_bitwrite_double(ctx, s, m->shininess) != 0) goto fail;
            if (prc_bitwrite_double(ctx, s, m->ambient_alpha) != 0) goto fail;
            if (prc_bitwrite_double(ctx, s, m->diffuse_alpha) != 0) goto fail;
            if (prc_bitwrite_double(ctx, s, m->emissive_alpha) != 0) goto fail;
            if (prc_bitwrite_double(ctx, s, m->specular_alpha) != 0) goto fail;
        }
        else /* PRC_TYPE_GRAPH_TextureApplication */
        {
            if (prc_bitwrite_uint32(ctx, s, m->biased_material_generic_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_texture_definition_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_next_texture_index) != 0) goto fail;
            if (prc_bitwrite_uint32(ctx, s, m->biased_uv_coordinates_index) != 0) goto fail;
        }
    }

    /* line_pattern table: out of this session's scope -- always empty */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;

    /* styles */
    if (prc_bitwrite_uint32(ctx, s, tables->style_count) != 0) goto fail;
    for (i = 0; i < tables->style_count; i++)
    {
        const prc_graph_style *st = &tables->styles[i];

        if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_GRAPH_Style) != 0) goto fail;
        if (prc_write_content_prc_ref_base(ctx, s, &st->base) != 0) goto fail;
        if (prc_bitwrite_double(ctx, s, st->line_width) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_vpicture) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, st->biased_pattern_index) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_material) != 0) goto fail;
        if (prc_bitwrite_uint32(ctx, s, st->biased_color_index) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_transparency) != 0) goto fail;
        if (st->is_transparency)
            if (prc_bitwrite_uint8(ctx, s, st->transparency) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_rendering_parameters) != 0) goto fail;
        if (st->is_rendering_parameters)
            if (prc_bitwrite_uint8(ctx, s, st->rendering_parameters) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_rendering_parameters2) != 0) goto fail;
        if (st->is_rendering_parameters2)
            if (prc_bitwrite_uint8(ctx, s, st->rendering_parameters2) != 0) goto fail;
        if (prc_bitwrite_bit(ctx, s, st->is_rendering_parameters3) != 0) goto fail;
        if (st->is_rendering_parameters3)
            if (prc_bitwrite_uint8(ctx, s, st->rendering_parameters3) != 0) goto fail;
    }

    /* fill_pattern table: out of this session's scope -- always empty */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;

    /* ref_coord (RI coordinate system) table: out of this session's scope -- always empty */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;

    return 0;

fail:
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}
