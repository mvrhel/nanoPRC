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

/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   *** KNOWN BUG, DO NOT TRUST OUTPUT UNTIL FIXED ***: this tool's regenerated
   model section is built via the plain prc_write_model_file_to_stream, which
   hardcodes nanoPRC's own PRC_WRITE_FILE_STRUCT_UID0 constant as its far
   reference to "which file structure this model belongs to" -- but this
   tool always keeps the REAL file's main header untouched, which declares a
   DIFFERENT (the real file's own) file-structure UID. The two silently
   mismatch, which is exactly the failure class prc_write_model_file_to_
   stream's own doc comment already describes ("can not find FileStructure").
   Confirmed via matrix_blank_tree.c (see ISO-SPEC/blanktree-matrix-evidence-
   table.md): fixing this (threading the real file's own fs_uid through
   prc_write_model_file_to_stream_ex instead) turned previously-"failing"
   Acrobat/PDF-XChange results into working ones for equivalent content. Any
   "Acrobat blank tree" conclusion drawn from this tool's prior output should
   be treated as invalid until re-tested with the same fix applied here.

   WHAT: Takes a REAL, independently-produced, complete raw .prc file and
   rebuilds it keeping the file-struct-header, tessellation, geometry,
   extra_geometry, and model-file sections byte-for-byte identical, but
   REGENERATES the schema/globals AND tree sections TOGETHER, as an
   internally-consistent matched pair, using this write facility's own
   encoders.

   HOW: usage: splice_schema_tree_section.exe <real_input.prc> <output.prc>.
   The minimal regenerated tree's one representation item references the
   real file's first tessellation entry (biased_index_tessellation=1);
   its default style comes from this write facility's own one-material
   default style (matching prc_write_prc_buffer's normal behavior), so
   the regenerated schema section is non-empty exactly the way every real
   self-consistent output of this write facility already is. The tree
   always has exactly 2 products (root last, biased index 2) -- this only
   resolves correctly if the real input's own model-file root_index also
   happens to be 2 (true of both reference files tested; check with
   scan_prc's DEBUG2 trace for an arbitrary new input).

   WHY IT EXISTS: The prior single-section tree-only splice (splice_tree_
   section.c) hit a wall: real producers' schema sections can declare a
   PRC_TYPE_ROOTBaseWithGraphics extension (used to carry per-entity
   attributes like Title/Author/Description), which changes the bit
   encoding of EVERY base-with-graphics record in the tree -- this write
   facility's tree writer never emits that extra data, so a tree spliced
   onto a schema that declares it immediately desyncs. That was not a
   tree-encoding bug, just an invalid cross-generator splice. Swapping
   schema+tree TOGETHER keeps them mutually consistent (this write
   facility's own schema never declares the extension, matching what its
   tree writer assumes) while still testing schema/tree content against a
   reader using REAL tessellation/geometry/model-file sections alongside
   them.

   Also regenerates the model-file section to match root_biased_index --
   see splice_tree_section.c's header comment for why keeping the real
   model-file section (with its own, potentially different, root_index)
   invalidates the test. An earlier version of this tool that kept the
   real model-file section reported "schema+tree as a matched pair opened
   correctly in Acrobat"; that conclusion did not survive re-verification
   after the root_index confound was found and fixed elsewhere, and should
   be treated as stale until re-confirmed with this corrected version.

   LIMITATIONS: Only handles single-file-structure inputs. The coupling
   between schema and tree this tool works around means it cannot isolate
   "is the schema wrong" from "is the tree wrong" independently -- only
   "is the schema+tree PAIR wrong". See splice_schema_tree_tess_section.c
   for the next step (adding tessellation to the swapped set). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_write_model.h"
#include "prc_write_tree.h"
#include "prc_write_global.h"
#include "prc_write_common.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void write_le_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v&0xff); p[1]=(uint8_t)((v>>8)&0xff); p[2]=(uint8_t)((v>>16)&0xff); p[3]=(uint8_t)((v>>24)&0xff); }

static uint32_t
add_default_style(prc_context *ctx, prc_write_global_tables *tables)
{
    prc_rgb_color gray, black;
    prc_graph_material material;
    prc_graph_style style;
    uint32_t biased_gray_index, biased_black_index, biased_material_index;

    memset(&gray, 0, sizeof(gray));
    gray.red = 0.72; gray.green = 0.42; gray.blue = 0.20; gray.alpha = 1.0;
    biased_gray_index = prc_write_color_add(ctx, tables, &gray);
    if (biased_gray_index == 0) return 0;

    memset(&black, 0, sizeof(black));
    black.red = 0.0; black.green = 0.0; black.blue = 0.0; black.alpha = 1.0;
    biased_black_index = prc_write_color_add(ctx, tables, &black);
    if (biased_black_index == 0) return 0;

    memset(&material, 0, sizeof(material));
    material.tag = PRC_TYPE_GRAPH_Material;
    material.biased_ambient_index = biased_gray_index;
    material.biased_diffuse_index = biased_gray_index;
    material.biased_specular_index = biased_gray_index;
    material.biased_emissive_index = biased_black_index;
    material.shininess = 0.3;
    material.ambient_alpha = 1.0;
    material.diffuse_alpha = 1.0;
    material.emissive_alpha = 1.0;
    material.specular_alpha = 1.0;
    biased_material_index = prc_write_material_add(ctx, tables, &material);
    if (biased_material_index == 0) return 0;

    memset(&style, 0, sizeof(style));
    style.is_material = 1;
    style.biased_color_index = biased_material_index;
    return prc_write_style_add(ctx, tables, &style);
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count, root_biased_index, default_style_index;
    uint32_t *section_offsets;
    uint32_t start_offset, end_offset;
    uint32_t i;
    size_t header_prefix_len;
    size_t section_off_table_byte_pos;
    prc_bit_write_state schema_s, tree_s, model_s;
    uint8_t *schema_comp = NULL, *tree_comp = NULL, *model_comp = NULL;
    size_t schema_comp_len = 0, tree_comp_len = 0, model_comp_len = 0;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    FILE *out_fid;
    prc_write_global_tables tables;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;

    if (argc < 3)
    {
        printf("usage: %s <real_input.prc> <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }

    fid = fopen(argv[1], "rb");
    if (fid == NULL) { printf("failed to open %s\n", argv[1]); return 1; }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize) { printf("short read\n"); return 1; }

    if (memcmp(buf, "PRC", 3) != 0) { printf("not a raw PRC file\n"); return 1; }

    p = buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("only single-file-structure inputs supported\n"); return 1; }
    p += 4;
    p += 16; /* fs_uid */
    p += 4;  /* reserved */
    section_off_table_byte_pos = (size_t)(p - buf) + 4;
    section_count = read_le_u32(p); p += 4;
    header_prefix_len = (size_t)(p - buf);

    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
    start_offset = read_le_u32(p); p += 4;
    end_offset = read_le_u32(p); p += 4;

    if (section_count < 4) { printf("unexpected section_count=%u\n", section_count); return 1; }
    printf("Real file: section_offsets=[");
    for (i = 0; i < section_count; i++) printf("%u ", section_offsets[i]);
    printf("] start=%u end=%u\n", start_offset, end_offset);
    printf("Real schema section: %u bytes (offset %u..%u)\n", section_offsets[2] - section_offsets[1], section_offsets[1], section_offsets[2]);
    printf("Real tree section: %u bytes (offset %u..%u)\n", section_offsets[3] - section_offsets[2], section_offsets[2], section_offsets[3]);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    if (prc_write_global_tables_init(ctx, &tables) != 0) { printf("global_tables_init failed\n"); return 1; }
    default_style_index = add_default_style(ctx, &tables);
    if (default_style_index == 0) { printf("add_default_style failed\n"); return 1; }

    memset(&schema_s, 0, sizeof(schema_s));
    if (prc_bitwrite_init(ctx, &schema_s, 1024) != 0) { printf("schema bitwrite_init failed\n"); return 1; }
    if (prc_write_schema_and_globals_to_stream(ctx, &schema_s, &tables) != 0)
    {
        printf("prc_write_schema_and_globals_to_stream failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }
    if (prc_bitwrite_flush(ctx, &schema_s) != 0) { printf("schema bitwrite_flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, schema_s.buf, schema_s.byte_pos, &schema_comp, &schema_comp_len) != 0)
    {
        printf("schema prc_write_deflate failed\n");
        return 1;
    }
    printf("Generated replacement schema section: %zu bytes compressed\n", schema_comp_len);

    /* Minimal root+part tree: 1 rep item, referencing real tessellation
       entry 1 (biased/1-based), using OUR default style. */
    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = part_node.bbox_min[1] = part_node.bbox_min[2] = -1.2345678;
    part_node.bbox_max[0] = part_node.bbox_max[1] = part_node.bbox_max[2] = 1.2345678;
    part_node.name = NULL;
    part_node.part_name = NULL;

    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = NULL;

    memset(&tree_s, 0, sizeof(tree_s));
    if (prc_bitwrite_init(ctx, &tree_s, 256) != 0) { printf("tree bitwrite_init failed\n"); return 1; }
    if (prc_write_tree_to_stream(ctx, &tree_s, &root, &root_biased_index, default_style_index) != 0)
    {
        printf("prc_write_tree_to_stream failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }
    if (prc_bitwrite_flush(ctx, &tree_s) != 0) { printf("tree bitwrite_flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, tree_s.buf, tree_s.byte_pos, &tree_comp, &tree_comp_len) != 0)
    {
        printf("tree prc_write_deflate failed\n");
        return 1;
    }
    printf("Generated replacement tree section: %zu bytes compressed, root_biased_index=%u\n", tree_comp_len, root_biased_index);

    /* Regenerate the model-file section too, self-consistently matching
       root_biased_index -- see the header comment for why this matters. */
    memset(&model_s, 0, sizeof(model_s));
    if (prc_bitwrite_init(ctx, &model_s, 256) != 0) { printf("model bitwrite_init failed\n"); return 1; }
    if (prc_write_model_file_to_stream(ctx, &model_s, NULL, root_biased_index, 1) != 0)
    {
        printf("prc_write_model_file_to_stream failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }
    if (prc_bitwrite_flush(ctx, &model_s) != 0) { printf("model bitwrite_flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, model_s.buf, model_s.byte_pos, &model_comp, &model_comp_len) != 0)
    {
        printf("model prc_write_deflate failed\n");
        return 1;
    }
    printf("Generated replacement model section: %zu bytes compressed, root_index=%u\n", model_comp_len, root_biased_index);

    /* Rebuild section_offsets: [0]=same (file-struct-header start), [1]=same
       (schema start), [2] = section_offsets[1] + our schema_comp_len (tree
       start), [3] = [2] + our tree_comp_len (tess start), then every later
       section shifts by the total delta but keeps its own real bytes. */
    new_section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    new_section_offsets[0] = section_offsets[0];
    new_section_offsets[1] = section_offsets[1];
    new_section_offsets[2] = section_offsets[1] + (uint32_t)schema_comp_len;
    new_section_offsets[3] = new_section_offsets[2] + (uint32_t)tree_comp_len;
    {
        int32_t delta = (int32_t)new_section_offsets[3] - (int32_t)section_offsets[3];
        for (i = 4; i < section_count; i++)
            new_section_offsets[i] = (uint32_t)((int32_t)section_offsets[i] + delta);
        /* start_offset (model section start) shifts like every later
           section, but end_offset is now determined by OUR regenerated
           model section's own length, not a shift of the real one's. */
        new_start_offset = (uint32_t)((int32_t)start_offset + delta);
        new_end_offset = new_start_offset + (uint32_t)model_comp_len;
    }
    new_total_size = (size_t)new_end_offset;

    out_buf = (uint8_t *)malloc(new_total_size);
    memcpy(out_buf, buf, header_prefix_len);
    {
        uint8_t *q = out_buf + section_off_table_byte_pos;
        size_t real_trailer_pos = section_off_table_byte_pos + (size_t)4 * section_count + 4 + 4;
        for (i = 0; i < section_count; i++) { write_le_u32(q, new_section_offsets[i]); q += 4; }
        write_le_u32(q, new_start_offset); q += 4;
        write_le_u32(q, new_end_offset); q += 4;
        memcpy(q, buf + real_trailer_pos, 4);
    }
    /* file-struct-header (section 0), unchanged content, unchanged position */
    memcpy(out_buf + new_section_offsets[0], buf + section_offsets[0], section_offsets[1] - section_offsets[0]);
    /* our new schema section */
    memcpy(out_buf + new_section_offsets[1], schema_comp, schema_comp_len);
    /* our new tree section */
    memcpy(out_buf + new_section_offsets[2], tree_comp, tree_comp_len);
    /* real tessellation/geometry/extra_geometry, unchanged content, shifted position */
    memcpy(out_buf + new_section_offsets[3], buf + section_offsets[3], start_offset - section_offsets[3]);
    /* our new model-file section, NOT the real one */
    memcpy(out_buf + new_start_offset, model_comp, model_comp_len);

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes)\n", argv[2], new_total_size);

    /* Optional PDF embed with an explicit, generously-sized Default view --
       prc_to_pdf.exe's NULL-options embed omits /3DV entirely, which a
       separate test earlier in this investigation showed can matter. This
       tool doesn't decode the real geometry it keeps byte-for-byte, so
       there's no real bbox to frame exactly; a large fixed volume is used
       instead, generous enough to contain any reasonably-scaled test mesh
       rather than requiring per-file tuning. */
    if (argc >= 4)
    {
        prc_pdf_view_spec view;
        prc_pdf_write_options pdf_opts;
        int code;

        memset(&view, 0, sizeof(view));
        view.name = "Default";
        view.eye[0] = 100.0; view.eye[1] = -130.0; view.eye[2] = 70.0;
        view.target[0] = 0.0; view.target[1] = 0.0; view.target[2] = 0.0;
        view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
        view.is_default = 1;

        memset(&pdf_opts, 0, sizeof(pdf_opts));
        pdf_opts.views = &view;
        pdf_opts.num_views = 1;

        code = prc_api_pdf_embed_prc(ctx, argv[3], out_buf, new_total_size, &pdf_opts);
        if (code < 0)
        {
            printf("prc_api_pdf_embed_prc failed: %d\n", code);
            prc_api_print_error_stack(ctx);
        }
        else
        {
            printf("Wrote %s (with explicit Default view)\n", argv[3]);
        }
    }

    prc_free(ctx, schema_comp);
    prc_free(ctx, tree_comp);
    prc_free(ctx, model_comp);
    prc_bitwrite_release(ctx, &schema_s);
    prc_bitwrite_release(ctx, &tree_s);
    prc_bitwrite_release(ctx, &model_s);
    prc_write_global_tables_free(ctx, &tables);
    prc_api_release_context(ctx);
    free(section_offsets);
    free(new_section_offsets);
    free(out_buf);
    free(buf);
    return 0;
}
