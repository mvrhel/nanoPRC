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
   registered with CTest, no exit-code contract to keep stable. TEMPORARY
   DIAGNOSTIC -- not a candidate for the permanent write facility as-is;
   see WHY IT EXISTS.

   WHAT: Writes the SAME minimal 2-triangle open quad as
   standalone_grid_triangles.exe's N=2 case (row 17a in the blank-tree
   evidence matrix, which fails in Acrobat), but with a HAND-WRITTEN
   tessellation body instead of going through prc_write_tess_3d --
   specifically testing must_calculate_normals=TRUE with NO stored normal
   data (zero-length normal array) and a triangulated_index_array containing
   only POSITION indices per corner (one per corner, not the normal+position
   INDEX PAIR prc_write_tess_3d always emits when real per-vertex normals
   are supplied). This exactly matches the real, independently-produced,
   Acrobat-confirmed-working ElevationMeshIS_ePRC.pdf and xml-sample-
   wrl_ePRC.pdf files' own convention (both use must_calculate_normals=1
   with number_of_normal_coordinates=0 -- confirmed via
   dump_uncompressed_tess_fields.exe).

   WHY IT EXISTS: prc_write_tess_3d (src/prc_write_tess_3d.c) hardcodes
   must_calculate_normals=0 UNCONDITIONALLY (line ~142) and always writes
   explicit stored normal data -- there is currently no way to ask the
   production write facility for the must_calculate_normals=1 convention at
   all. Every real oracle file examined this session (all 5 reference
   PDFs) uses must_calculate_normals=1. This tool tests, in isolation,
   whether that's the actual missing piece behind the open-mesh-topology
   Acrobat failure (see ISO-SPEC/blanktree-matrix-evidence-table.md rows
   17a/17b) before committing to changing production code.

   Schema/tree/geometry/extra_geometry/model sections reuse the exact same
   write-facility calls prc_write_prc_buffer itself uses (copied, not
   duplicated logic beyond the necessary reimplementation of the top-level
   assembly loop) -- ONLY the tessellation section body is hand-written.

   HOW: usage: experiment_mustcalc_quad.exe <output.prc> [output.pdf] */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"
#include "prc_context.h"
#include "prc_data.h"
#include "prc_write_global.h"
#include "prc_write_tree.h"
#include "prc_write_model.h"
#include "prc_write_file_structure.h"
#include "prc_write_common.h"

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

/* Hand-written PRC_TYPE_TESS_3D body: must_calculate_normals=1, zero stored
   normals, triangulated_index_array holds ONLY position indices (one per
   corner) -- matches real oracle files' own convention exactly, unlike
   prc_write_tess_3d's hardcoded must_calculate_normals=0 + explicit
   normal-index-pairs convention. */
static int
write_tess3d_mustcalc(prc_context *ctx, prc_bit_write_state *s,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles)
{
    uint32_t i;

    if (prc_bitwrite_bit(ctx, s, 0) != 0) return -1;                     /* is_calculated */
    if (prc_bitwrite_uint32(ctx, s, num_positions * 3) != 0) return -1;
    for (i = 0; i < num_positions * 3; i++)
        if (prc_bitwrite_double(ctx, s, positions[i]) != 0) return -1;

    if (prc_bitwrite_bit(ctx, s, 1) != 0) return -1;                     /* has_faces */
    if (prc_bitwrite_bit(ctx, s, 0) != 0) return -1;                     /* has_loops */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) return -1;                     /* must_calculate_normals = TRUE */

    /* Only present when must_calculate_normals is TRUE (prc_parse_tess_3d
       reads these two fields conditionally right here) -- omitting them
       desyncs everything after. crease_angle is stored in DEGREES (the
       parser multiplies by PI/180 on read); 30 degrees matches a common
       real-world convention. */
    if (prc_bitwrite_uint8(ctx, s, 0) != 0) return -1;                   /* normal_recalculation_flags (not used; should be zero) */
    if (prc_bitwrite_double(ctx, s, 15.0) != 0) return -1;               /* crease_angle, degrees */

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* number_of_normal_coordinates = 0, no stored normals */

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* number_of_wire_indices */

    if (prc_bitwrite_uint32(ctx, s, num_triangles * 3) != 0) return -1;  /* number_of_triangulated_indicies: ONE index per corner */
    for (i = 0; i < num_triangles * 3; i++)
        if (prc_bitwrite_uint32(ctx, s, tri_indices[i] * 3) != 0) return -1;

    if (prc_bitwrite_uint32(ctx, s, 1) != 0) return -1;                  /* number_of_face_tessellation */
    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_TESS_Face) != 0) return -1; /* tag */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* size_of_line_attributes */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* start_of_wire_data */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* size_of_sizes_wire */
    if (prc_bitwrite_uint32(ctx, s, (uint32_t)PRC_FACETESSDATA_Triangle) != 0) return -1; /* used_entities_flag */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* start_triangulated */
    if (prc_bitwrite_uint32(ctx, s, 1) != 0) return -1;                  /* size_of_triangulateddata */
    if (prc_bitwrite_uint32(ctx, s, num_triangles) != 0) return -1;      /* triangulateddata[0] */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* number_of_textured_coordinate_indexes */
    if (prc_bitwrite_bit(ctx, s, 0) != 0) return -1;                     /* has_vertex_colors */

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) return -1;                  /* number_of_texture_coordinates */

    return 0;
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    /* Same quad as standalone_grid_triangles.exe N=2 (a cylinder-section
       patch, real geometric curvature, NOT a degenerate flat plane): 4
       positions, 2 triangles. */
    double positions[4 * 3];
    uint32_t tri_indices[2 * 3];
    const char *out_pdf = NULL;
    prc_write_global_tables tables;
    uint32_t default_style_index;
    prc_bit_write_state schema_s, tree_s, tess_s, geom_s, extra_geom_s, model_s;
    uint8_t *schema_comp = NULL, *tree_comp = NULL, *tess_comp = NULL, *geom_comp = NULL, *extra_geom_comp = NULL, *model_comp = NULL;
    size_t schema_comp_len = 0, tree_comp_len = 0, tess_comp_len = 0, geom_comp_len = 0, extra_geom_comp_len = 0, model_comp_len = 0;
    uint32_t root_biased_index = 0;
    uint32_t section_offsets[6];
    uint32_t start_offset, end_offset;
    size_t header_size, total_size;
    uint8_t file_struct_header[PRC_WRITE_FILE_STRUCT_HEADER_SIZE];
    uint8_t *buf;
    prc_write_main_header_layout layout;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;

    if (argc < 2)
    {
        printf("usage: %s <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }
    if (argc >= 3) out_pdf = argv[2];

    {
        const double radius = 5.0, half_angle = 60.0 * 3.14159265358979323846 / 180.0, half_length = 5.0;
        uint32_t xi, yi;
        for (yi = 0; yi < 2; yi++)
            for (xi = 0; xi < 2; xi++)
            {
                uint32_t i = yi * 2 + xi;
                double theta = ((double)xi * 2.0 - 1.0) * half_angle;
                double along = ((double)yi * 2.0 - 1.0) * half_length;
                positions[i * 3 + 0] = radius * sin(theta);
                positions[i * 3 + 1] = radius * cos(theta);
                positions[i * 3 + 2] = along;
            }
        /* i00=0(y0,x0) i10=1(y0,x1) i01=2(y1,x0) i11=3(y1,x1) -- same winding as standalone_grid_triangles.c */
        tri_indices[0] = 0; tri_indices[1] = 1; tri_indices[2] = 3;
        tri_indices[3] = 0; tri_indices[4] = 3; tri_indices[5] = 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    memset(&schema_s, 0, sizeof(schema_s));
    memset(&tree_s, 0, sizeof(tree_s));
    memset(&tess_s, 0, sizeof(tess_s));
    memset(&geom_s, 0, sizeof(geom_s));
    memset(&extra_geom_s, 0, sizeof(extra_geom_s));
    memset(&model_s, 0, sizeof(model_s));

    if (prc_write_global_tables_init(ctx, &tables) != 0) { printf("tables_init failed\n"); return 1; }
    default_style_index = add_default_style(ctx, &tables);
    if (default_style_index == 0) { printf("add_default_style failed\n"); return 1; }

    if (prc_bitwrite_init(ctx, &schema_s, 1024) != 0) { printf("schema init failed\n"); return 1; }
    if (prc_write_schema_and_globals_to_stream(ctx, &schema_s, &tables) != 0) { printf("schema write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &schema_s) != 0) { printf("schema flush failed\n"); return 1; }

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;
    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = -4.5; part_node.bbox_min[1] = 2.0; part_node.bbox_min[2] = -5.5;
    part_node.bbox_max[0] = 4.5; part_node.bbox_max[1] = 5.5; part_node.bbox_max[2] = 5.5;
    part_node.name = "quad_body";
    part_node.part_name = "quad_faces";
    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "quad";

    if (prc_bitwrite_init(ctx, &tree_s, 1024) != 0) { printf("tree init failed\n"); return 1; }
    if (prc_write_tree_to_stream(ctx, &tree_s, &root, &root_biased_index, default_style_index) != 0) { printf("tree write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &tree_s) != 0) { printf("tree flush failed\n"); return 1; }

    if (prc_bitwrite_init(ctx, &tess_s, 1024) != 0) { printf("tess init failed\n"); return 1; }
    if (prc_bitwrite_uint32(ctx, &tess_s, PRC_TYPE_ASM_FileStructureTessellation) != 0) { printf("tess wrapper failed\n"); return 1; }
    if (prc_bitwrite_uint32(ctx, &tess_s, 0) != 0) { printf("tess wrapper failed\n"); return 1; } /* base.attribute_count */
    if (prc_bitwrite_bit(ctx, &tess_s, 1) != 0) { printf("tess wrapper failed\n"); return 1; }     /* base.name.same */
    if (prc_bitwrite_uint32(ctx, &tess_s, 1) != 0) { printf("tess wrapper failed\n"); return 1; }  /* tess_count */
    if (prc_bitwrite_uint32(ctx, &tess_s, PRC_TYPE_TESS_3D) != 0) { printf("tess wrapper failed\n"); return 1; }
    if (write_tess3d_mustcalc(ctx, &tess_s, positions, 4, tri_indices, 2) != 0) { printf("tess3d write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &tess_s) != 0) { printf("tess flush failed\n"); return 1; }

    if (prc_bitwrite_init(ctx, &geom_s, 64) != 0) { printf("geom init failed\n"); return 1; }
    if (prc_write_geometry_section_to_stream(ctx, &geom_s) != 0) { printf("geom write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &geom_s) != 0) { printf("geom flush failed\n"); return 1; }

    if (prc_bitwrite_init(ctx, &extra_geom_s, 64) != 0) { printf("extra_geom init failed\n"); return 1; }
    if (prc_write_extra_geometry_section_to_stream(ctx, &extra_geom_s) != 0) { printf("extra_geom write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &extra_geom_s) != 0) { printf("extra_geom flush failed\n"); return 1; }

    if (prc_bitwrite_init(ctx, &model_s, 256) != 0) { printf("model init failed\n"); return 1; }
    if (prc_write_model_file_to_stream(ctx, &model_s, "nanoPRC", root_biased_index, 1) != 0) { printf("model write failed\n"); return 1; }
    if (prc_bitwrite_flush(ctx, &model_s) != 0) { printf("model flush failed\n"); return 1; }

    if (prc_write_deflate(ctx, schema_s.buf, schema_s.byte_pos, &schema_comp, &schema_comp_len) != 0) { printf("schema deflate failed\n"); return 1; }
    if (prc_write_deflate(ctx, tree_s.buf, tree_s.byte_pos, &tree_comp, &tree_comp_len) != 0) { printf("tree deflate failed\n"); return 1; }
    if (prc_write_deflate(ctx, tess_s.buf, tess_s.byte_pos, &tess_comp, &tess_comp_len) != 0) { printf("tess deflate failed\n"); return 1; }
    if (prc_write_deflate(ctx, geom_s.buf, geom_s.byte_pos, &geom_comp, &geom_comp_len) != 0) { printf("geom deflate failed\n"); return 1; }
    if (prc_write_deflate(ctx, extra_geom_s.buf, extra_geom_s.byte_pos, &extra_geom_comp, &extra_geom_comp_len) != 0) { printf("extra_geom deflate failed\n"); return 1; }
    if (prc_write_deflate(ctx, model_s.buf, model_s.byte_pos, &model_comp, &model_comp_len) != 0) { printf("model deflate failed\n"); return 1; }

    prc_write_main_header_compute_layout(6, &layout);
    header_size = layout.total_size;
    prc_write_file_struct_header_bytes(file_struct_header, PRC_WRITE_MIN_VERS_FOR_READ, PRC_WRITE_AUTH_VERS);

    section_offsets[0] = (uint32_t)header_size;
    section_offsets[1] = section_offsets[0] + (uint32_t)sizeof(file_struct_header);
    section_offsets[2] = section_offsets[1] + (uint32_t)schema_comp_len;
    section_offsets[3] = section_offsets[2] + (uint32_t)tree_comp_len;
    section_offsets[4] = section_offsets[3] + (uint32_t)tess_comp_len;
    section_offsets[5] = section_offsets[4] + (uint32_t)geom_comp_len;
    start_offset = section_offsets[5] + (uint32_t)extra_geom_comp_len;
    end_offset = start_offset + (uint32_t)model_comp_len;
    total_size = (size_t)end_offset;

    buf = (uint8_t *)malloc(total_size);
    {
        uint8_t *p = buf;
        p[0] = 'P'; p[1] = 'R'; p[2] = 'C'; p += 3;
        p = prc_write_le_uint32(p, PRC_WRITE_MIN_VERS_FOR_READ);
        p = prc_write_le_uint32(p, PRC_WRITE_AUTH_VERS);
        p = prc_write_le_unique_id(p, PRC_WRITE_FILE_UID0);
        p = prc_write_le_unique_id(p, PRC_WRITE_APP_UID0);
        p = prc_write_le_uint32(p, 1); /* filestructure_count */
        p = prc_write_le_unique_id(p, PRC_WRITE_FILE_STRUCT_UID0);
        p = prc_write_le_uint32(p, 0); /* reserved */
        p = prc_write_le_uint32(p, 6); /* section_count */
        {
            int i;
            for (i = 0; i < 6; i++) p = prc_write_le_uint32(p, section_offsets[i]);
        }
        p = prc_write_le_uint32(p, start_offset);
        p = prc_write_le_uint32(p, end_offset);
        p = prc_write_le_uint32(p, 0); /* file_count */
    }
    memcpy(buf + section_offsets[0], file_struct_header, sizeof(file_struct_header));
    memcpy(buf + section_offsets[1], schema_comp, schema_comp_len);
    memcpy(buf + section_offsets[2], tree_comp, tree_comp_len);
    memcpy(buf + section_offsets[3], tess_comp, tess_comp_len);
    memcpy(buf + section_offsets[4], geom_comp, geom_comp_len);
    memcpy(buf + section_offsets[5], extra_geom_comp, extra_geom_comp_len);
    memcpy(buf + start_offset, model_comp, model_comp_len);

    {
        FILE *out = fopen(argv[1], "wb");
        if (out == NULL || fwrite(buf, 1, total_size, out) != total_size)
        {
            printf("failed to write %s\n", argv[1]);
            return 1;
        }
        fclose(out);
        printf("Wrote %s (%zu bytes)\n", argv[1], total_size);
    }

    if (out_pdf != NULL)
    {
        prc_pdf_view_spec view;
        prc_pdf_write_options pdf_opts;

        memset(&view, 0, sizeof(view));
        view.name = "Default";
        view.eye[0] = 15.0; view.eye[1] = -15.0; view.eye[2] = 12.0;
        view.target[0] = 0.0; view.target[1] = 3.75; view.target[2] = 0.0;
        view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
        view.is_default = 1;
        memset(&pdf_opts, 0, sizeof(pdf_opts));
        pdf_opts.views = &view;
        pdf_opts.num_views = 1;
        code = prc_api_pdf_embed_prc(ctx, out_pdf, buf, total_size, &pdf_opts);
        if (code < 0)
        {
            printf("prc_api_pdf_embed_prc failed: %d\n", code);
            prc_api_print_error_stack(ctx);
        }
        else
        {
            printf("Wrote %s (with explicit Default view)\n", out_pdf);
        }
    }

    prc_bitwrite_release(ctx, &schema_s);
    prc_bitwrite_release(ctx, &tree_s);
    prc_bitwrite_release(ctx, &tess_s);
    prc_bitwrite_release(ctx, &geom_s);
    prc_bitwrite_release(ctx, &extra_geom_s);
    prc_bitwrite_release(ctx, &model_s);
    if (schema_comp != NULL) prc_free(ctx, schema_comp);
    if (tree_comp != NULL) prc_free(ctx, tree_comp);
    if (tess_comp != NULL) prc_free(ctx, tess_comp);
    if (geom_comp != NULL) prc_free(ctx, geom_comp);
    if (extra_geom_comp != NULL) prc_free(ctx, extra_geom_comp);
    if (model_comp != NULL) prc_free(ctx, model_comp);
    prc_write_global_tables_free(ctx, &tables);
    prc_api_release_context(ctx);
    free(buf);
    return 0;
}
