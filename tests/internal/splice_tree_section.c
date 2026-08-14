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
   rebuilds it keeping the file-struct-header, schema/globals,
   tessellation, geometry, extra_geometry, and model-file sections byte-
   for-byte identical, but REGENERATES the tree section using this write
   facility's own encoder (prc_write_tree_to_stream) -- a minimal
   root+part tree whose one representation item references the real
   file's first tessellation entry (biased_index_tessellation=1) and
   carries no style reference (default_biased_style_index=0).

   HOW: usage: splice_tree_section.exe <real_input.prc> <output.prc>
   [output.pdf]. The regenerated tree always has exactly 2 products (root
   last, biased index 2). The real file's own model-file section (a
   SEPARATE section from the tree, addressed via start_offset/end_offset,
   not section_offset[]) is ALSO regenerated to match -- it carries its
   own independent root_index reference into the tree's products[] array
   (see prc_write_model.c's own doc comment), and earlier versions of this
   tool that kept the real model-file section byte-for-byte produced a
   silent index mismatch whenever the real file's own tree didn't happen
   to have exactly 2 products (confirmed via DEBUG2:
   data->product_occurences[0].root_index in prc_parse_file_structure.c),
   invalidating any Acrobat-blank-tree conclusion drawn from that
   configuration -- not a genuine tree-encoding result, just an
   uncontrolled variable. Regenerating both together, self-consistently,
   removes that confound entirely.

   *** KNOWN INVALID FOR SCHEMA-EXTENDED FILES *** -- this tool's central
   premise (splice only the tree, keep the real schema) turned out to be
   methodologically broken for real files whose schema declares an
   extension for PRC_TYPE_ROOTBaseWithGraphics (entity_type=2 in the
   schema table -- common in real producer files that attach custom
   per-entity attributes, e.g. Title/Author/Description metadata). When
   that extension is declared, EVERY base-with-graphics record in the
   tree (every Part/Product/RepresentationItem) is expected to carry
   extra schema-defined data that this write facility's tree encoder
   never emits, causing an immediate bitstream desync -- NOT a genuine
   "tree encoding is wrong" result, just an invalid cross-generator
   splice. See splice_schema_tree_section.c for the fix (swap schema+tree
   together as a matched pair instead). Do not draw conclusions from a
   failure with this specific tool alone; confirm via the paired version
   first.

   WHY IT EXISTS: Superseded by splice_schema_tree_section.c once the
   schema-extension problem above was discovered. Kept for reference/
   history; prefer the paired tool for any new work. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_write_model.h"
#include "prc_write_tree.h"
#include "prc_write_common.h"

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void write_le_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v&0xff); p[1]=(uint8_t)((v>>8)&0xff); p[2]=(uint8_t)((v>>16)&0xff); p[3]=(uint8_t)((v>>24)&0xff); }

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count, root_biased_index;
    uint32_t *section_offsets;
    uint32_t start_offset, end_offset;
    uint32_t i;
    size_t header_prefix_len; /* everything up to and including section_count */
    prc_bit_write_state tree_s, model_s;
    uint8_t *tree_comp = NULL, *model_comp = NULL;
    size_t tree_comp_len = 0, model_comp_len = 0;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    size_t section_off_table_byte_pos;
    FILE *out_fid;
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
    section_off_table_byte_pos = (size_t)(p - buf) + 4; /* after section_count dword */
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
    /* Section layout (0-indexed, matches prc_write_prc_buffer's own
       section_offsets[]): [0]=file_struct_header start, [1]=schema start
       (=file_struct_header end), [2]=tree start (=schema end), [3]=tess
       start (=tree end), [4]=geometry start, [5]=extra_geometry start,
       start_offset=model-file start (=extra_geometry end). */
    printf("Real tree section: %u bytes (offset %u..%u)\n", section_offsets[3] - section_offsets[2], section_offsets[2], section_offsets[3]);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    /* Minimal root+part tree: 1 rep item, referencing real tessellation
       entry 1 (biased/1-based), no style reference. */
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
    if (prc_bitwrite_init(ctx, &tree_s, 256) != 0) { printf("bitwrite_init failed\n"); return 1; }
    if (prc_write_tree_to_stream(ctx, &tree_s, &root, &root_biased_index, 1) != 0)
    {
        printf("prc_write_tree_to_stream failed\n");
        prc_api_print_error_stack(ctx);
        return 1;
    }
    if (prc_bitwrite_flush(ctx, &tree_s) != 0) { printf("bitwrite_flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, tree_s.buf, tree_s.byte_pos, &tree_comp, &tree_comp_len) != 0)
    {
        printf("prc_write_deflate failed\n");
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

    /* Rebuild section_offsets: [0]=header_prefix (same), [1]=same (schema
       start, unchanged), [2]=same (tree start, unchanged -- schema's END
       doesn't move), [3] = section_offsets[2] + our tree_comp_len (new
       tree section end / tess section start), then every later section
       shifts by the same delta but keeps its own real bytes. */
    new_section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    new_section_offsets[0] = section_offsets[0];
    new_section_offsets[1] = section_offsets[1];
    new_section_offsets[2] = section_offsets[2];
    new_section_offsets[3] = section_offsets[2] + (uint32_t)tree_comp_len;
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
    /* header prefix (signature..section_count) unchanged */
    memcpy(out_buf, buf, header_prefix_len);
    /* section_offsets[] table + start_offset + end_offset: patched */
    {
        uint8_t *q = out_buf + section_off_table_byte_pos;
        size_t real_trailer_pos = section_off_table_byte_pos + (size_t)4 * section_count + 4 + 4;
        for (i = 0; i < section_count; i++) { write_le_u32(q, new_section_offsets[i]); q += 4; }
        write_le_u32(q, new_start_offset); q += 4;
        write_le_u32(q, new_end_offset); q += 4;
        /* trailing file_count dword (multi-file support flag), unchanged -- must be
           copied explicitly, it isn't covered by header_prefix_len or the offset patch above. */
        memcpy(q, buf + real_trailer_pos, 4);
    }
    /* file-struct-header + schema (sections 0 and 1), unchanged content,
       unchanged position. */
    memcpy(out_buf + new_section_offsets[0], buf + section_offsets[0], section_offsets[2] - section_offsets[0]);
    /* our new tree section, at the real tree section's start */
    memcpy(out_buf + new_section_offsets[2], tree_comp, tree_comp_len);
    /* real tessellation/geometry/extra_geometry (sections 3..end of section_offsets[]), unchanged content, shifted position */
    memcpy(out_buf + new_section_offsets[3], buf + section_offsets[3], start_offset - section_offsets[3]);
    /* our new model-file section, at the (shifted) model section's start -- NOT the real one */
    memcpy(out_buf + new_start_offset, model_comp, model_comp_len);

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes)\n", argv[2], new_total_size);

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

    prc_free(ctx, tree_comp);
    prc_free(ctx, model_comp);
    prc_bitwrite_release(ctx, &tree_s);
    prc_bitwrite_release(ctx, &model_s);
    prc_api_release_context(ctx);
    free(section_offsets);
    free(new_section_offsets);
    free(out_buf);
    free(buf);
    return 0;
}
