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

#ifndef PRC_WRITE_GLOBAL_H
#define PRC_WRITE_GLOBAL_H

#include <stddef.h>
#include "prc_write_common.h"
#include "prc_data.h"
#include "prc_bit.h"

/* Encoder for the globals section (Table 40, prc_parse_global_data /
   prc_parse_global.c): the file-wide color/material/picture/style tables
   that representation items reference by biased index. Out of this
   session's scope (always emitted as empty/default so the real parser's
   fixed field order is satisfied): texture definitions, line patterns,
   fill patterns, and reference coordinate systems.

   Usage: init a prc_write_global_tables, add entries via the *_add
   functions (each returns the 1-based "biased index" -- 0 means error --
   that other write-side records should store to reference the entry),
   then call prc_write_globals_to_stream once all entries are added. */

/* Input pixel format for prc_write_picture_add. RGB/RGBA are raw pixel
   bytes the caller has already decoded; PNG/JPEG are the original
   encoded file bytes (dimensions are extracted from them here). */
typedef enum prc_write_pix_format_e
{
    PRC_WRITE_PIX_RGB = 0,
    PRC_WRITE_PIX_RGBA,
    PRC_WRITE_PIX_PNG,
    PRC_WRITE_PIX_JPEG
} prc_write_pix_format;

typedef struct prc_write_picture_s
{
    prc_write_pix_format format;
    const uint8_t *data;      /* RGB/RGBA: width*height*[3|4] raw bytes;
                                  PNG/JPEG: the encoded file bytes */
    size_t          data_size;
    uint32_t        width;    /* required for RGB/RGBA; ignored (parsed
                                  from `data`) for PNG/JPEG */
    uint32_t        height;   /* required for RGB/RGBA; ignored (parsed
                                  from `data`) for PNG/JPEG */
} prc_write_picture;

typedef struct prc_write_global_tables_s
{
    prc_rgb_color      *colors;
    uint32_t             color_count, color_cap;

    prc_graph_material  *materials;
    uint32_t             material_count, material_cap;

    prc_graph_picture   *pictures;
    uint32_t             picture_count, picture_cap;

    prc_graph_style     *styles;
    uint32_t             style_count, style_cap;

    /* Embedded uncompressed files, written into the file-structure header
       rather than the globals section. This is where raster images live: a
       prc_graph_picture above carries only format and dimensions, and points
       here through biased_uncompressed_file_index. The bytes are owned by
       these tables and freed with them. */
    prc_write_embedded_file *files;
    uint32_t             file_count, file_cap;

    prc_graph_texture_definition *texture_definitions;
    uint32_t             texture_definition_count, texture_definition_cap;
} prc_write_global_tables;

int prc_write_global_tables_init(prc_context *ctx, prc_write_global_tables *tables);
void prc_write_global_tables_free(prc_context *ctx, prc_write_global_tables *tables);

/* Each *_add function returns the 1-based biased index of the (possibly
   deduplicated, possibly newly appended) table entry, or 0 on error. */

/* Dedup on (red, green, blue) only -- the on-disk format
   (prc_parse_rgb(..., has_alpha=false)) never stores alpha, so two colors
   differing only in alpha are indistinguishable once written. */
uint32_t prc_write_color_add(prc_context *ctx, prc_write_global_tables *tables,
    const prc_rgb_color *color);

/* material->tag must be PRC_TYPE_GRAPH_Material or
   PRC_TYPE_GRAPH_TextureApplication (Table 89's two material variants).
   Dedup compares the tag-appropriate numeric fields only (not `base`). */
uint32_t prc_write_material_add(prc_context *ctx, prc_write_global_tables *tables,
    const prc_graph_material *material);

/* Dedup key is (is_material, biased_color_index, is_transparency +
   transparency, is_rendering_parameters + rendering_parameters), per the
   design's specified style dedup tuple. style->tag is ignored on input;
   the entry is always written out as PRC_TYPE_GRAPH_Style. */
/* One representation item's own material: colour + material + style, all
   deduplicated against what the tables already hold. Returns the biased style
   index, or 0 on failure. */
/* Optional diffuse texture for prc_write_add_item_style. Mirrors the public
   prc_api_write_rep_item texture fields; NULL means no texture. */
typedef struct
{
    const uint8_t *image;
    size_t         image_size;
    uint32_t       format;      /* prc_api_write_texture_format */
    uint32_t       width, height;
} prc_write_item_texture;

uint32_t prc_write_add_item_style(prc_context *ctx, prc_write_global_tables *tables,
    const double color[3], double alpha, double shininess,
    const prc_write_item_texture *texture);

uint32_t prc_write_style_add(prc_context *ctx, prc_write_global_tables *tables,
    const prc_graph_style *style);

/* No deduplication. For PNG/JPEG, `picture->data` is parsed here (validating
   the PNG signature/IHDR chunk or the JPEG SOI marker + a scanned SOF
   marker) purely to recover pixel_width/pixel_height -- consistent with the
   real prc_graph_picture (Table 93), the globals section stores only this
   format/dimensions metadata, never the bytes themselves.

   The bytes are copied into the embedded-file table in the file-structure
   header, and biased_uncompressed_file_index points at them. For the raw
   formats only width*height*components bytes are stored, not the whole of
   `data_size`, since a caller may legitimately pass a larger buffer.

   Callers may release their own copy as soon as this returns. */
uint32_t prc_write_picture_add(prc_context *ctx, prc_write_global_tables *tables,
    const prc_write_picture *picture);

/* Emit the complete globals section (Table 40) in the exact field order
   prc_parse_global_data expects. */
int prc_write_globals_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_global_tables *tables);


/* Adds an embedded uncompressed file (a raster image's bytes) and returns the
   1-biased index for prc_graph_picture.biased_uncompressed_file_index. The
   bytes are copied, so the caller may release its own copy immediately. */
uint32_t prc_write_embedded_file_add(prc_context *ctx, prc_write_global_tables *tables,
    const uint8_t *data, uint32_t size);


/* Binds a picture to UV sampling and returns the 1-biased index for a
   TextureApplication's biased_texture_definition_index. transformation is
   optional placement and may be NULL. Only the plain diffuse form is written
   -- see the implementation for what each field is set to and why. */
uint32_t prc_write_texture_definition_add(prc_context *ctx, prc_write_global_tables *tables,
    uint32_t biased_picture_index,
    const prc_cart_transformation *transformation);

#endif
