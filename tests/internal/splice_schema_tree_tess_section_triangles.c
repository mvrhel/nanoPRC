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

   WHAT: Identical to splice_schema_tree_tess_section.c except the spliced
   tessellation section uses PRC_API_WRITE_TESS_KIND_TRIANGLES (uncompressed
   PRC_TYPE_TESS_3D) instead of PRC_API_WRITE_TESS_KIND_COMPRESSED -- same
   single explicit-normal triangle, same schema/tree, same real geometry/
   extra_geometry/model-file kept byte-for-byte from the input.

   WHY IT EXISTS: splice_schema_tree_tess_section.c (COMPRESSED kind)
   already conclusively proved this write facility's compressed-
   tessellation-section encoding is what breaks a real file in Acrobat,
   independent of schema/tree/model-file (all separately cleared). This
   asks the same question for the uncompressed TRIANGLES path -- the write
   kind actually at issue in the separate Acrobat-blank-model-tree
   investigation for uncompressed output (see
   ISO-SPEC/outer-engine-cover-investigation.md and
   reauthor_tess_pdf.c's own header comment) -- since three independent,
   real-file-comparison-motivated fixes to the PDF-wrapper/tree-shape layer
   (an empty part on every intermediate product level, matching the real
   /3DA activation dictionary values exactly, adding /Resources<</Names[]>>
   to the /Type/3D stream dict) all failed to change Acrobat's behavior,
   and the canvas was found to be blank too (not just the tree panel, with
   no error dialog), pointing back at the bitstream content rather than the
   PDF/annotation wrapper. This isolates whether it's specifically this
   write facility's uncompressed tessellation-section encoding, the same
   way the sibling tool already isolated it for the compressed path.

   HOW: usage: splice_schema_tree_tess_section_triangles.exe
   <real_input.prc> <output.prc> <output.pdf>. Same 2-products/
   root_index=2 constraint, same single-file-structure-input limitation,
   as the COMPRESSED sibling. Embeds output.pdf directly with an explicit
   "Default" 3D view framing the synthetic triangle's own real bbox
   ((0,0,0)-(1,1,0)), not prc_to_pdf.exe's NULL-options (no /3DV at all)
   embed -- a first attempt at this test wrapped via prc_to_pdf.exe, and
   showed a blank canvas in both Acrobat AND PDF-XChange despite nanoPRC's
   own viewer rendering the spliced triangle correctly, which pointed at
   camera framing (no default view at all, real readers' fallback camera
   apparently not finding a 1-unit triangle) rather than the tessellation
   encoding itself -- this rules that specific confound out before trusting
   the result either way. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_write_model.h"
#include "prc_write_tree.h"
#include "prc_write_global.h"
#include "prc_write_file_structure.h"
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
    prc_bit_write_state schema_s, tree_s, tess_s;
    uint8_t *schema_comp = NULL, *tree_comp = NULL, *tess_comp = NULL;
    size_t schema_comp_len = 0, tree_comp_len = 0, tess_comp_len = 0;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    FILE *out_fid;
    prc_write_global_tables tables;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;
    prc_api_write_tessellation tess;
    static const double positions[9] = { 0.0, 0.0, 0.0,  1.0, 0.0, 0.0,  0.0, 1.0, 0.0 };
    static const double normals[9] = { 0.0, 0.0, 1.0,  0.0, 0.0, 1.0,  0.0, 0.0, 1.0 };
    static const uint32_t tri_indices[3] = { 0, 1, 2 };
    static const uint32_t face_tri_counts[1] = { 1 };

    if (argc < 4)
    {
        printf("usage: %s <real_input.prc> <output.prc> <output.pdf>\n", argv[0]);
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
    p += 16;
    p += 4;
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

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    if (prc_write_global_tables_init(ctx, &tables) != 0) { printf("global_tables_init failed\n"); return 1; }
    default_style_index = add_default_style(ctx, &tables);
    if (default_style_index == 0) { printf("add_default_style failed\n"); return 1; }

    memset(&schema_s, 0, sizeof(schema_s));
    if (prc_bitwrite_init(ctx, &schema_s, 1024) != 0) { printf("schema init failed\n"); return 1; }
    if (prc_write_schema_and_globals_to_stream(ctx, &schema_s, &tables) != 0) { printf("schema write failed\n"); prc_api_print_error_stack(ctx); return 1; }
    if (prc_bitwrite_flush(ctx, &schema_s) != 0) { printf("schema flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, schema_s.buf, schema_s.byte_pos, &schema_comp, &schema_comp_len) != 0) { printf("schema deflate failed\n"); return 1; }
    printf("Generated schema section: %zu bytes compressed\n", schema_comp_len);

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
    if (prc_bitwrite_init(ctx, &tree_s, 256) != 0) { printf("tree init failed\n"); return 1; }
    if (prc_write_tree_to_stream(ctx, &tree_s, &root, &root_biased_index, default_style_index) != 0) { printf("tree write failed\n"); prc_api_print_error_stack(ctx); return 1; }
    if (prc_bitwrite_flush(ctx, &tree_s) != 0) { printf("tree flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, tree_s.buf, tree_s.byte_pos, &tree_comp, &tree_comp_len) != 0) { printf("tree deflate failed\n"); return 1; }
    printf("Generated tree section: %zu bytes compressed, root_biased_index=%u\n", tree_comp_len, root_biased_index);

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES; /* the only change from splice_schema_tree_tess_section.c */
    tess.positions = positions;
    tess.num_positions = 3;
    tess.normals = normals;
    tess.num_normals = 3;
    tess.tri_indices = tri_indices;
    tess.norm_indices = tri_indices;
    tess.num_triangles = 1;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;
    tess.tolerance = prc_write_tol_absolute(0.01); /* matches real reference files' own tolerance */

    memset(&tess_s, 0, sizeof(tess_s));
    if (prc_bitwrite_init(ctx, &tess_s, 256) != 0) { printf("tess init failed\n"); return 1; }
    if (prc_write_tessellation_section_to_stream(ctx, &tess_s, &tess, 1) != 0) { printf("tess write failed\n"); prc_api_print_error_stack(ctx); return 1; }
    if (prc_bitwrite_flush(ctx, &tess_s) != 0) { printf("tess flush failed\n"); return 1; }
    if (prc_write_deflate(ctx, tess_s.buf, tess_s.byte_pos, &tess_comp, &tess_comp_len) != 0) { printf("tess deflate failed\n"); return 1; }
    printf("Generated tessellation section: %zu bytes compressed\n", tess_comp_len);

    new_section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    new_section_offsets[0] = section_offsets[0];
    new_section_offsets[1] = section_offsets[1];
    new_section_offsets[2] = section_offsets[1] + (uint32_t)schema_comp_len;
    new_section_offsets[3] = new_section_offsets[2] + (uint32_t)tree_comp_len;
    new_section_offsets[4] = new_section_offsets[3] + (uint32_t)tess_comp_len;
    {
        int32_t delta = (int32_t)new_section_offsets[4] - (int32_t)section_offsets[4];
        for (i = 5; i < section_count; i++)
            new_section_offsets[i] = (uint32_t)((int32_t)section_offsets[i] + delta);
        new_start_offset = (uint32_t)((int32_t)start_offset + delta);
        new_end_offset = (uint32_t)((int32_t)end_offset + delta);
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
    /* our schema section */
    memcpy(out_buf + new_section_offsets[1], schema_comp, schema_comp_len);
    /* our tree section */
    memcpy(out_buf + new_section_offsets[2], tree_comp, tree_comp_len);
    /* our tessellation section */
    memcpy(out_buf + new_section_offsets[3], tess_comp, tess_comp_len);
    /* every remaining real section (geometry, extra_geometry, model-file), unchanged content, shifted position */
    memcpy(out_buf + new_section_offsets[4], buf + section_offsets[4], end_offset - section_offsets[4]);

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);

    printf("Wrote %s (%zu bytes)\n", argv[2], new_total_size);

    /* Explicit Default view framing the synthetic triangle's real bbox
       (0,0,0)-(1,1,0) -- see the header comment for why this matters. */
    {
        prc_pdf_view_spec view;
        prc_pdf_write_options pdf_opts;
        double bbox_min[3] = { 0.0, 0.0, 0.0 };
        double bbox_max[3] = { 1.0, 1.0, 0.0 };
        double center[3], extent[3], diag_len;
        int a;
        int code;

        for (a = 0; a < 3; a++) center[a] = 0.5 * (bbox_min[a] + bbox_max[a]);
        for (a = 0; a < 3; a++) extent[a] = bbox_max[a] - bbox_min[a];
        diag_len = sqrt(extent[0] * extent[0] + extent[1] * extent[1] + extent[2] * extent[2]);
        if (diag_len < 1e-6) diag_len = 1.0;

        memset(&view, 0, sizeof(view));
        view.name = "Default";
        view.eye[0] = center[0] + diag_len * 1.0;
        view.eye[1] = center[1] - diag_len * 1.3;
        view.eye[2] = center[2] + diag_len * 0.7;
        view.target[0] = center[0];
        view.target[1] = center[1];
        view.target[2] = center[2];
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
            return 1;
        }
        printf("Wrote %s (with explicit Default view)\n", argv[3]);
    }

    prc_free(ctx, schema_comp);
    prc_free(ctx, tree_comp);
    prc_free(ctx, tess_comp);
    prc_bitwrite_release(ctx, &schema_s);
    prc_bitwrite_release(ctx, &tree_s);
    prc_bitwrite_release(ctx, &tess_s);
    prc_write_global_tables_free(ctx, &tables);
    prc_api_release_context(ctx);
    free(section_offsets);
    free(new_section_offsets);
    free(out_buf);
    free(buf);
    return 0;
}
