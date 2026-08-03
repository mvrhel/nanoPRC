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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_write_model.h"
#include "prc_data.h"
#include "prc_diag_env.h"

int
prc_write_model_file_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const char *model_name, uint32_t root_biased_index, uint32_t file_struct_count)
{
    uint32_t default_uid[4] = { PRC_WRITE_FILE_STRUCT_UID0, 0, 0, 0 };
    return prc_write_model_file_to_stream_ex(ctx, s, model_name, root_biased_index, file_struct_count, default_uid);
}

int
prc_write_model_file_to_stream_ex(prc_context *ctx, prc_bit_write_state *s,
    const char *model_name, uint32_t root_biased_index, uint32_t file_struct_count,
    const uint32_t file_struct_uid[4])
{
    uint32_t k;

    if (ctx == NULL || s == NULL || file_struct_count == 0 || file_struct_uid == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_model_file_to_stream_ex: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* leading uint32 the parser skips unexplained */
    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_ASM_ModelFile) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;        /* base.attribute_count */
    if (prc_write_name(ctx, s, model_name) != 0) goto fail;    /* base.name */

    /* units_from_CAD_flag: a real, independently-produced, tessellation-
       only compressed PRC file (used as a working reference specifically
       because it opens correctly in Acrobat) always writes this true, even
       though units_from_CAD_file's actual value (1.0, i.e. no scaling) is
       identical whether or not the flag is set. Acrobat's handling of an
       unset flag here is untested/unconfirmed -- but since every field this
       write facility has copied from real producer conventions so far has
       turned out to matter more than it looked like it should, match it. */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;      /* units_from_CAD_flag */
    if (prc_bitwrite_double(ctx, s, 1.0) != 0) goto fail; /* units_from_CAD_file */

    if (prc_bitwrite_uint32(ctx, s, 1) != 0) goto fail; /* number_of_root_product_occurrences */

    /* ProductOccurrenceReference: unique_id + root_index + is_active.
       `unique_id` here is a FAR REFERENCE to the FILE STRUCTURE that owns
       the actual root product occurrence -- NOT the product's own
       tree-assigned ContentPRCRefBase.unique_id (confirmed the hard way:
       an external reader failed with "can not find FileStructure
       {1,0,0,0}" when this was written as the tree's root unique_id).
       This write facility only ever emits one file structure, so the
       reference here must match that file structure's own identity
       (PRC_WRITE_FILE_STRUCT_UID0 -- see its doc comment in
       prc_write_common.h for how this was confirmed against a real PRC
       stream), not the root product's identity.

       root_index also turned out to be a BIASED (1-based, 0 = none) index
       into that file structure's products[] array, like every other
       cross-reference in this format -- NOT a plain 0-based index despite
       its parser-side field name. Confirmed against a real PRC stream: a
       2-product file structure had root_index=2 pointing at its LAST
       product (0-based index 1); writing a plain 0-based root_index=0
       here was silently treated as "no root", which produced an "Empty
       Scene Detected" error even though every other part of the file
       parsed and resolved correctly. root_biased_index is
       prc_write_tree_to_stream's *root_biased_index_out, which -- since
       the tree encoder always places the root last -- equals that file
       structure's total product count. */
    if (prc_bitwrite_uint32(ctx, s, file_struct_uid[0]) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, file_struct_uid[1]) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, file_struct_uid[2]) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, file_struct_uid[3]) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, root_biased_index) != 0) goto fail; /* root_index */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;    /* product_occurence_is_active */

    for (k = 0; k < file_struct_count; k++)
        if (prc_bitwrite_uint32(ctx, s, k) != 0) goto fail; /* file_structure_index_in_model_file[k] */

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* user_data.stream_size */

    return 0;

fail:
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}

/* Table 5/35 main prc_header, single file structure only. This is the ONE
   place that knows the field order/sizes; prc_write_main_header_bytes (the
   writer) and every reader of a field position (prc_write_main_header_size,
   and any external caller such as a test independently re-reading
   start_offset/end_offset out of a written file) go through this function
   instead of separately re-deriving byte offsets, so the two can't
   silently drift apart when a field is added, removed, or the section
   count changes. */
void
prc_write_main_header_compute_layout(uint32_t section_count, prc_write_main_header_layout *out)
{
    size_t pos = 0;

    pos += PRC_WRITE_SIGNATURE_BYTES; /* "PRC" */
    pos += PRC_WRITE_U32_BYTES;       /* min_vers_for_read */
    pos += PRC_WRITE_U32_BYTES;       /* auth_vers */
    pos += PRC_WRITE_UNIQUE_ID_BYTES; /* unique_id_file */
    pos += PRC_WRITE_UNIQUE_ID_BYTES; /* unique_id_application */
    pos += PRC_WRITE_U32_BYTES;       /* filestructure_count */

    pos += PRC_WRITE_UNIQUE_ID_BYTES; /* file_info[0].unique_id */
    pos += PRC_WRITE_U32_BYTES;       /* reserved/pad field the parser skips past */
    pos += PRC_WRITE_U32_BYTES;       /* section_count */

    out->section_offset_table_pos = pos;
    pos += (size_t)PRC_WRITE_U32_BYTES * section_count; /* section_offset[] */

    out->start_offset_pos = pos;
    pos += PRC_WRITE_U32_BYTES;
    out->end_offset_pos = pos;
    pos += PRC_WRITE_U32_BYTES;
    pos += PRC_WRITE_U32_BYTES; /* file_count */

    out->total_size = pos;
}

/* DIAGNOSTIC (2026-07-27): override the declared min_vers_for_read/auth_vers
   pair, gated behind env vars, zero behavior change when unset. Added to
   test a hypothesis raised while comparing nanoPRC's output against an
   independent encoder on the same real-world files: that encoder declares
   a single, self-consistent version (min_vers_for_read == auth_vers), while
   nanoPRC declares a ~16-year gap (7094 / 23306) -- per this header's own
   documented PRC_TYPE_GRAPH_DirectionalLight precedent, some fields are
   gated purely on auth_vers crossing a threshold with "no secondary flag",
   so a large declared gap seemed a plausible source of version-conditional
   parsing mismatches in stricter readers.

   DISPROVEN (2026-07-27, same day): matching the independent encoder's
   exact self-consistent pair (8137/8137 via PRC_DIAG_MIN_VERS_FOR_READ=8137
   PRC_DIAG_AUTH_VERS=8137) on three real files that reproducibly blank the
   model tree in Acrobat (Y200-A-AC-B.stream-7, 2368573.stream-146,
   2003002153310.stream-7) made no difference -- all three still blanked
   the tree, byte-identical otherwise. Declared version is not the (sole)
   cause of this bug family. Kept as a diagnostic in case a different
   specific value or a narrower per-field investigation is worth trying
   later, but this specific hypothesis is closed. */
static uint32_t
prc_write_diag_min_vers_for_read(void)
{
    const char *ov = prc_diag_getenv("PRC_DIAG_MIN_VERS_FOR_READ");
    return ov != NULL ? (uint32_t)strtoul(ov, NULL, 10) : PRC_WRITE_MIN_VERS_FOR_READ;
}

static uint32_t
prc_write_diag_auth_vers(void)
{
    const char *ov = prc_diag_getenv("PRC_DIAG_AUTH_VERS");
    return ov != NULL ? (uint32_t)strtoul(ov, NULL, 10) : PRC_WRITE_AUTH_VERS;
}

static size_t
prc_write_main_header_size(uint32_t section_count)
{
    prc_write_main_header_layout layout;

    prc_write_main_header_compute_layout(section_count, &layout);
    return layout.total_size;
}

static void
prc_write_main_header_bytes(uint8_t *out, uint32_t section_count, const uint32_t *section_offsets,
    uint32_t start_offset, uint32_t end_offset)
{
    uint8_t *p = out;
    uint32_t i;

    p[0] = 'P'; p[1] = 'R'; p[2] = 'C';
    p += PRC_WRITE_SIGNATURE_BYTES;
    p = prc_write_le_uint32(p, prc_write_diag_min_vers_for_read());
    p = prc_write_le_uint32(p, prc_write_diag_auth_vers());
    p = prc_write_le_unique_id(p, PRC_WRITE_FILE_UID0);
    p = prc_write_le_unique_id(p, PRC_WRITE_APP_UID0);
    p = prc_write_le_uint32(p, 1); /* filestructure_count */

    /* Must equal the file-structure header's own unique_id_file
       (prc_write_file_struct_header_bytes) and the model section's far
       reference to this file structure (prc_write_model_file_to_stream) --
       see PRC_WRITE_FILE_STRUCT_UID0's doc comment in prc_write_common.h. */
    p = prc_write_le_unique_id(p, PRC_WRITE_FILE_STRUCT_UID0);
    p = prc_write_le_uint32(p, 0); /* 4-byte reserved field the parser skips past */
    p = prc_write_le_uint32(p, section_count);
    for (i = 0; i < section_count; i++)
        p = prc_write_le_uint32(p, section_offsets[i]);

    p = prc_write_le_uint32(p, start_offset);
    p = prc_write_le_uint32(p, end_offset);
    p = prc_write_le_uint32(p, 0); /* file_count: multi-file not supported */
}

/* Adds one plain gray material + style to `tables` and returns its
   biased (1-based) style index (0 on allocation failure). Every file
   this write facility produces gets exactly this one default style,
   applied to every part/product/representation-item in the tree (see
   prc_write_tree_to_stream's default_biased_style_index parameter) --
   real-world PRC producers apparently always attach at least a default
   material somewhere, and a genuinely empty style table, while it
   round-trips through nanoPRC's own parser and an independent PRC
   converter's reader, was narrowed down (by elimination against a real,
   independently-produced, proven-working PDF/PRC container) to being the
   difference that keeps real 3D-capable PDF viewers from rendering this
   facility's output. */
static uint32_t
prc_write_add_default_style(prc_context *ctx, prc_write_global_tables *tables)
{
    prc_rgb_color gray, black;
    prc_graph_material material;
    prc_graph_style style;
    uint32_t biased_gray_index, biased_black_index, biased_material_index;

    /* A warm terracotta tone, not a light gray: a (0.7,0.7,0.7) neutral
       gray reads as near-white under typical viewer lighting and default
       (often white) backgrounds, making solid geometry effectively
       invisible until the background is manually changed -- confirmed the
       hard way testing this write facility's teapot demo in a real 3D-PDF
       viewer. This is the one shared default every write-facility caller
       gets when it doesn't specify its own style, so it needs to be
       visible against both light and dark backgrounds, not just
       plausible-looking in isolation. */
    memset(&gray, 0, sizeof(gray));
    gray.red = 0.72; gray.green = 0.42; gray.blue = 0.20; gray.alpha = 1.0;
    biased_gray_index = prc_write_color_add(ctx, tables, &gray);
    if (biased_gray_index == 0)
        return 0;

    memset(&black, 0, sizeof(black));
    black.red = 0.0; black.green = 0.0; black.blue = 0.0; black.alpha = 1.0;
    biased_black_index = prc_write_color_add(ctx, tables, &black);
    if (biased_black_index == 0)
        return 0;

    memset(&material, 0, sizeof(material));
    material.tag = PRC_TYPE_GRAPH_Material;
    material.biased_ambient_index = biased_gray_index;
    material.biased_diffuse_index = biased_gray_index;
    material.biased_specular_index = biased_gray_index;
    /* prc_internal_get_surface_material (prc_style_api.c) requires all
       four *_index fields non-zero, not just ambient/diffuse/specular --
       confirmed the hard way: leaving emissive at 0 ("no emissive
       reference", which reads as spec-reasonable) rejects the whole
       material with a parse error the moment a real viewer/our own
       full-fidelity reader actually resolves it, even though every
       lighter-weight check earlier in the pipeline accepts a material
       with only three of the four set. */
    material.biased_emissive_index = biased_black_index;
    material.shininess = 0.3;
    material.ambient_alpha = 1.0;
    material.diffuse_alpha = 1.0;
    material.emissive_alpha = 1.0;
    material.specular_alpha = 1.0;
    biased_material_index = prc_write_material_add(ctx, tables, &material);
    if (biased_material_index == 0)
        return 0;

    memset(&style, 0, sizeof(style));
    style.is_material = 1;
    style.biased_color_index = biased_material_index; /* "biased_color_index" holds the material index when is_material */
    return prc_write_style_add(ctx, tables, &style);
}

int
prc_write_prc_buffer(prc_context *ctx,
    const char *model_name,
    prc_write_global_tables *tables,
    const prc_write_tree_node *root,
    const prc_write_tess_entry *tess_entries, uint32_t num_tess_entries,
    uint8_t **out_buf, size_t *out_size)
{
    prc_bit_write_state schema_s, tree_s, tess_s, geom_s, extra_geom_s, model_s;
    uint8_t *schema_comp = NULL, *tree_comp = NULL, *tess_comp = NULL, *geom_comp = NULL, *extra_geom_comp = NULL, *model_comp = NULL;
    size_t schema_comp_len = 0, tree_comp_len = 0, tess_comp_len = 0, geom_comp_len = 0, extra_geom_comp_len = 0, model_comp_len = 0;
    uint32_t root_biased_index = 0;
    uint32_t default_style_index;
    int ret = PRC_ERROR_INTERNAL;
    /* See PRC_WRITE_PRC_FILE_SECTION_COUNT's doc comment: file-struct-header
       (implicit) + schema_globals + tree + tessellation + geometry +
       extra_geometry. The (always-empty) geometry/extra_geometry sections
       are written even though this write facility never produces exact
       B-Rep content: some third-party PRC readers assume Table 6's fixed
       section set is always present and misread a later section's bytes
       as the missing one otherwise -- see prc_write_geometry_section_to_
       stream and prc_write_extra_geometry_section_to_stream. */
    const uint32_t section_count = PRC_WRITE_PRC_FILE_SECTION_COUNT;
    uint32_t section_offsets[PRC_WRITE_PRC_FILE_SECTION_COUNT];
    uint32_t start_offset, end_offset;
    size_t header_size;
    uint8_t file_struct_header[PRC_WRITE_FILE_STRUCT_HEADER_SIZE];
    uint8_t *buf = NULL;
    size_t total_size;

    memset(&schema_s, 0, sizeof(schema_s));
    memset(&tree_s, 0, sizeof(tree_s));
    memset(&tess_s, 0, sizeof(tess_s));
    memset(&geom_s, 0, sizeof(geom_s));
    memset(&extra_geom_s, 0, sizeof(extra_geom_s));
    memset(&model_s, 0, sizeof(model_s));

    if (ctx == NULL || tables == NULL || root == NULL || out_buf == NULL || out_size == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_prc_buffer: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }
    *out_buf = NULL;
    *out_size = 0;

    default_style_index = prc_write_add_default_style(ctx, tables);
    if (default_style_index == 0)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error adding default style in prc_write_prc_buffer\n");
        return PRC_ERROR_MEMORY;
    }

    if (prc_bitwrite_init(ctx, &schema_s, 1024) != 0) goto cleanup;
    if (prc_write_schema_and_globals_to_stream(ctx, &schema_s, tables) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &schema_s) != 0) goto cleanup;

    if (prc_bitwrite_init(ctx, &tree_s, 1024) != 0) goto cleanup;
    if (prc_write_tree_to_stream(ctx, &tree_s, root, &root_biased_index, default_style_index) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &tree_s) != 0) goto cleanup;

    if (prc_bitwrite_init(ctx, &tess_s, 1024) != 0) goto cleanup;
    if (prc_write_tessellation_section_to_stream(ctx, &tess_s, tess_entries, num_tess_entries) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &tess_s) != 0) goto cleanup;

    if (prc_bitwrite_init(ctx, &geom_s, 64) != 0) goto cleanup;
    if (prc_write_geometry_section_to_stream(ctx, &geom_s) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &geom_s) != 0) goto cleanup;

    if (prc_bitwrite_init(ctx, &extra_geom_s, 64) != 0) goto cleanup;
    if (prc_write_extra_geometry_section_to_stream(ctx, &extra_geom_s) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &extra_geom_s) != 0) goto cleanup;

    if (prc_bitwrite_init(ctx, &model_s, 256) != 0) goto cleanup;
    if (prc_write_model_file_to_stream(ctx, &model_s, model_name, root_biased_index, 1) != 0) goto cleanup;
    if (prc_bitwrite_flush(ctx, &model_s) != 0) goto cleanup;

    if (prc_write_deflate(ctx, schema_s.buf, schema_s.byte_pos, &schema_comp, &schema_comp_len) != 0) goto cleanup;
    if (prc_write_deflate(ctx, tree_s.buf, tree_s.byte_pos, &tree_comp, &tree_comp_len) != 0) goto cleanup;
    if (prc_write_deflate(ctx, tess_s.buf, tess_s.byte_pos, &tess_comp, &tess_comp_len) != 0) goto cleanup;
    if (prc_write_deflate(ctx, geom_s.buf, geom_s.byte_pos, &geom_comp, &geom_comp_len) != 0) goto cleanup;
    if (prc_write_deflate(ctx, extra_geom_s.buf, extra_geom_s.byte_pos, &extra_geom_comp, &extra_geom_comp_len) != 0) goto cleanup;
    if (prc_write_deflate(ctx, model_s.buf, model_s.byte_pos, &model_comp, &model_comp_len) != 0) goto cleanup;

    /* Every section's final size is now known, so the complete layout
       (and therefore the main header's offset table) can be computed
       up front -- no placeholder-then-seek-back patch needed, unlike a
       direct-to-file writer that doesn't know trailing sizes until it
       has already written the header. */
    header_size = prc_write_main_header_size(section_count);
    prc_write_file_struct_header_bytes(file_struct_header, prc_write_diag_min_vers_for_read(), prc_write_diag_auth_vers());

    section_offsets[0] = (uint32_t)header_size;
    section_offsets[1] = section_offsets[0] + (uint32_t)sizeof(file_struct_header);
    section_offsets[2] = section_offsets[1] + (uint32_t)schema_comp_len;
    section_offsets[3] = section_offsets[2] + (uint32_t)tree_comp_len;
    section_offsets[4] = section_offsets[3] + (uint32_t)tess_comp_len;
    section_offsets[5] = section_offsets[4] + (uint32_t)geom_comp_len;
    /* The model section is addressed via start_offset/end_offset directly,
       not through section_offset[] -- see prc_parse_main.c's "the model
       file is defined special" handling -- but it must still be placed
       LAST, immediately after every regular section, not merely somewhere
       else in the file. Confirmed against a real, known-good PRC stream
       (extracted from examples/cube.pdf): its start_offset/end_offset
       field values are the file's last two dwords, with end_offset ==
       the total file size. External testing showed at least one
       third-party reader relies on this contiguity -- it derives the last
       regular section's end as the model's start_offset, so placing the
       model section anywhere else produces a bogus (tiny) "end" for the
       real last regular section. */
    start_offset = section_offsets[5] + (uint32_t)extra_geom_comp_len;
    end_offset = start_offset + (uint32_t)model_comp_len;
    total_size = (size_t)end_offset;

    buf = (uint8_t *)prc_malloc(ctx, total_size);
    if (buf == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_prc_buffer\n");
        goto cleanup;
    }

    prc_write_main_header_bytes(buf, section_count, section_offsets, start_offset, end_offset);
    memcpy(buf + section_offsets[0], file_struct_header, sizeof(file_struct_header));
    memcpy(buf + section_offsets[1], schema_comp, schema_comp_len);
    memcpy(buf + section_offsets[2], tree_comp, tree_comp_len);
    memcpy(buf + section_offsets[3], tess_comp, tess_comp_len);
    memcpy(buf + section_offsets[4], geom_comp, geom_comp_len);
    memcpy(buf + section_offsets[5], extra_geom_comp, extra_geom_comp_len);
    memcpy(buf + start_offset, model_comp, model_comp_len);

    *out_buf = buf;
    *out_size = total_size;
    buf = NULL;
    ret = 0;

cleanup:
    if (buf != NULL) prc_free(ctx, buf);
    if (schema_comp != NULL) prc_free(ctx, schema_comp);
    if (tree_comp != NULL) prc_free(ctx, tree_comp);
    if (tess_comp != NULL) prc_free(ctx, tess_comp);
    if (geom_comp != NULL) prc_free(ctx, geom_comp);
    if (extra_geom_comp != NULL) prc_free(ctx, extra_geom_comp);
    if (model_comp != NULL) prc_free(ctx, model_comp);
    prc_bitwrite_release(ctx, &schema_s);
    prc_bitwrite_release(ctx, &tree_s);
    prc_bitwrite_release(ctx, &tess_s);
    prc_bitwrite_release(ctx, &geom_s);
    prc_bitwrite_release(ctx, &extra_geom_s);
    prc_bitwrite_release(ctx, &model_s);
    return ret;
}

int
prc_write_prc_file(prc_context *ctx, const char *filename,
    const char *model_name,
    prc_write_global_tables *tables,
    const prc_write_tree_node *root,
    const prc_write_tess_entry *tess_entries, uint32_t num_tess_entries)
{
    uint8_t *buf = NULL;
    size_t size = 0;
    FILE *fid;
    int code;

    if (filename == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_prc_file: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    code = prc_write_prc_buffer(ctx, model_name, tables, root, tess_entries, num_tess_entries, &buf, &size);
    if (code != 0)
        return code;

    fid = fopen(filename, "wb");
    if (fid == NULL)
    {
        prc_error(ctx, PRC_ERROR_IO, "prc_write_prc_file: failed to open output file\n");
        prc_free(ctx, buf);
        return PRC_ERROR_IO;
    }

    code = (fwrite(buf, 1, size, fid) == size) ? 0 : PRC_ERROR_IO;
    fclose(fid);
    prc_free(ctx, buf);

    if (code != 0)
        prc_error(ctx, PRC_ERROR_IO, "prc_write_prc_file: I/O error writing output file\n");

    return code;
}
