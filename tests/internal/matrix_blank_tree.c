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

   WHAT: Generalizes splice_tree_section.c / splice_schema_tree_section.c /
   reencode_real_tess_triangles.c into ONE tool with four independent axes,
   each selectable real|ours: schema (section 1), tree (section 2, ALWAYS
   paired with the model-file section since the model section's own
   root_index must match whichever tree it addresses -- see
   splice_tree_section.c's header comment for why keeping them decoupled
   silently invalidates results), tessellation (section 3, single
   PRC_TYPE_TESS_3D entry, must_calculate_normals==1, pure-Triangle faces
   only -- same scope as reencode_real_tess_triangles.c), and geometry
   (sections 4+5, geometry+extra_geometry, ALWAYS paired with each other --
   both are minimal/empty for every real reference file tested so far, no
   coupling concern analogous to tree/model has been found, but there is no
   reason to vary them independently either).

   Coupling rule enforced internally: when schema=real, a regenerated
   (ours) tree cannot reference a style/material index, because no such
   index exists in the real schema/globals section it's paired with (style
   index 0 = "no style", matching splice_tree_section.c). When schema=ours,
   a regenerated tree uses this tool's own freshly-added default style,
   matching splice_schema_tree_section.c. This mirrors exactly what the two
   predecessor tools already do; this tool just makes the choice a CLI flag
   instead of being baked into which tool you ran.

   WHY IT EXISTS: built to replace a growing pile of ad-hoc, incrementally-
   versioned one-off splice outputs (xml-sample-wrl_*_v2.pdf, _v3.pdf, ...)
   with ONE reviewed, single code path producing a small, systematically-
   named, complete 2x2x2 matrix for the Acrobat-blank-model-tree (TRIANGLES
   write path) investigation -- see
   project_acrobat_triangles_blank_tree_investigation memory / this
   session's discussion for the full matrix design and why schema/tree can
   be varied independently for this specific oracle file (verified via
   check_schema_extension.exe: none of the reference oracle files declare
   a PRC_TYPE_ROOTBaseWithGraphics schema extension, so the schema/tree
   decoupling that invalidates splice_tree_section.c's results for SOME
   real producer files does not apply here).

   LIMITATIONS: single-file-structure inputs only. Tessellation section
   must be exactly one PRC_TYPE_TESS_3D entry, must_calculate_normals==1,
   every face pure PRC_FACETESSDATA_Triangle -- if tess=ours is requested
   and the real tessellation doesn't match this shape, the tool aborts
   rather than silently falling back to real bytes (a silent fallback
   would be exactly the kind of uncontrolled-variable confound this tool
   exists to avoid).

   HOW: usage: matrix_blank_tree.exe <real_input.prc> <output.prc>
   <schema:real|ours> <tree:real|ours> <tess:real|ours> <geom:real|ours>
   <names:null|real> [output.pdf] */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "prc_write_tess_3d.h"
#include "prc_write_file_structure.h"
#include "prc_write_global.h"
#include "prc_write_tree.h"
#include "prc_write_model.h"
#include "prc_write_common.h"
#include "zlib.h"

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

/* Decode the real file's single tessellation entry (section 3) and
   re-encode it through our own TRIANGLES writer. Identical scope/logic to
   reencode_real_tess_triangles.c's try_reencode_fs_tess, simplified for a
   single, known-good file structure (aborts rather than skips on
   mismatch -- see this tool's own header LIMITATIONS). */
static int
reencode_tess(prc_context *ctx, const uint8_t *buf, uint32_t tess_start, uint32_t tess_end,
    uint8_t **out_bytes, size_t *out_len)
{
    uint32_t comp_len = tess_end - tess_start;
    uint8_t *inflated = NULL;
    uLongf dest_len;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, entry_tag;
    prc_content_prc_base wrapper_base;
    prc_tess_3d *parsed = NULL;
    prc_bit_write_state wrapper_s;
    uint8_t *wrapper_comp = NULL;
    size_t wrapper_comp_len = 0;
    double *positions = NULL;
    uint32_t *tri_indices = NULL;
    uint32_t *face_tri_counts = NULL;
    uint32_t num_positions, num_faces, total_triangles, tri_cursor;
    uint32_t f;
    int code;
    int ok = 0;

    dest_len = comp_len * 40u + 4096u;
    inflated = (uint8_t *)malloc(dest_len);
    for (;;)
    {
        uLongf try_len = dest_len;
        int zret = uncompress(inflated, &try_len, buf + tess_start, comp_len);
        if (zret == Z_OK) { dest_len = try_len; break; }
        if (zret == Z_BUF_ERROR) { dest_len *= 2; free(inflated); inflated = (uint8_t *)malloc(dest_len); continue; }
        printf("tess: zlib uncompress failed (%d)\n", zret);
        free(inflated);
        return 0;
    }

    prc_init_bit_state(ctx, &bit_state, inflated, (size_t)dest_len);
    wrapper_tag = prc_bitread_uint32(ctx, &bit_state);
    if (wrapper_tag != PRC_TYPE_ASM_FileStructureTessellation)
    {
        printf("tess: unexpected wrapper tag %u\n", wrapper_tag);
        free(inflated);
        return 0;
    }
    memset(&wrapper_base, 0, sizeof(wrapper_base));
    code = prc_parse_content_prc_base(ctx, &bit_state, &wrapper_base);
    if (code < 0) { printf("tess: prc_parse_content_prc_base failed (%d)\n", code); free(inflated); return 0; }
    tess_count = prc_bitread_uint32(ctx, &bit_state);
    if (tess_count != 1) { printf("tess: tess_count=%u (not 1)\n", tess_count); free(inflated); return 0; }
    entry_tag = prc_bitread_uint32(ctx, &bit_state);
    if (entry_tag != PRC_TYPE_TESS_3D) { printf("tess: entry tag=%u (not PRC_TYPE_TESS_3D)\n", entry_tag); free(inflated); return 0; }

    code = prc_parse_tess_3d(ctx, &bit_state, &parsed);
    if (code < 0) { printf("tess: prc_parse_tess_3d failed (%d)\n", code); prc_api_print_error_stack(ctx); free(inflated); return 0; }

    if (!parsed->must_calculate_normals)
    {
        printf("tess: must_calculate_normals=0 (stored normals), out of scope\n");
        free(inflated);
        return 0;
    }
    for (f = 0; f < parsed->number_of_face_tessellation; f++)
    {
        if (parsed->face_tessellation_data[f].used_entities_flag != PRC_FACETESSDATA_Triangle)
        {
            printf("tess: face %u used_entities_flag=%u (not pure Triangle), out of scope\n",
                f, parsed->face_tessellation_data[f].used_entities_flag);
            free(inflated);
            return 0;
        }
    }

    num_positions = parsed->tessellation_coordinates.number_of_coordinates / 3;
    num_faces = parsed->number_of_face_tessellation;
    positions = (double *)malloc(sizeof(double) * 3 * (num_positions ? num_positions : 1));
    memcpy(positions, parsed->tessellation_coordinates.coordinates, sizeof(double) * 3 * num_positions);

    face_tri_counts = (uint32_t *)malloc(sizeof(uint32_t) * (num_faces ? num_faces : 1));
    total_triangles = 0;
    for (f = 0; f < num_faces; f++)
    {
        uint32_t n_tri = parsed->face_tessellation_data[f].triangulateddata[0];
        face_tri_counts[f] = n_tri;
        total_triangles += n_tri;
    }

    tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * (total_triangles ? total_triangles : 1));
    tri_cursor = 0;
    for (f = 0; f < num_faces; f++)
    {
        prc_tess_face *face = &parsed->face_tessellation_data[f];
        uint32_t n_corners = face_tri_counts[f] * 3;
        uint32_t c;
        for (c = 0; c < n_corners; c++)
        {
            uint32_t raw = parsed->triangulated_index_array[face->start_triangulated + c];
            tri_indices[tri_cursor * 3 + (c % 3)] = raw / 3;
            if ((c % 3) == 2) tri_cursor++;
        }
    }

    {
        prc_api_write_tessellation tess;

        memset(&tess, 0, sizeof(tess));
        tess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
        tess.positions = positions;
        tess.num_positions = num_positions;
        tess.normals = NULL;
        tess.num_normals = 0;
        tess.tri_indices = tri_indices;
        tess.norm_indices = NULL;
        tess.num_triangles = total_triangles;
        tess.face_tri_counts = face_tri_counts;
        tess.num_faces = num_faces;

        memset(&wrapper_s, 0, sizeof(wrapper_s));
        if (prc_bitwrite_init(ctx, &wrapper_s, 4096) != 0) { printf("tess: wrapper init failed\n"); goto cleanup; }
        code = prc_write_tessellation_section_to_stream(ctx, &wrapper_s, &tess, 1);
        if (code != 0)
        {
            printf("tess: prc_write_tessellation_section_to_stream failed (%d)\n", code);
            prc_api_print_error_stack(ctx);
            goto cleanup;
        }
        if (prc_bitwrite_flush(ctx, &wrapper_s) != 0) { printf("tess: wrapper flush failed\n"); goto cleanup; }
        if (prc_write_deflate(ctx, wrapper_s.buf, wrapper_s.byte_pos, &wrapper_comp, &wrapper_comp_len) != 0)
        {
            printf("tess: deflate failed\n");
            goto cleanup;
        }
        printf("tess reencoded: %u positions, %u faces, %u triangles, %u -> %zu bytes (compressed)\n",
            num_positions, num_faces, total_triangles, comp_len, wrapper_comp_len);
        *out_bytes = wrapper_comp;
        *out_len = wrapper_comp_len;
        ok = 1;
        wrapper_comp = NULL;
    }

cleanup:
    free(positions);
    free(tri_indices);
    free(face_tri_counts);
    prc_bitwrite_release(ctx, &wrapper_s);
    if (wrapper_comp != NULL) prc_free(ctx, wrapper_comp);
    free(inflated);
    return ok;
}

static int
parse_bool_arg(const char *s, const char *label)
{
    if (strcmp(s, "real") == 0) return 0;
    if (strcmp(s, "ours") == 0) return 1;
    printf("invalid value for %s: '%s' (expected 'real' or 'ours')\n", label, s);
    exit(2);
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count, root_biased_index = 0, default_style_index = 0;
    uint32_t real_fs_uid[4];
    uint32_t *section_offsets;
    uint32_t start_offset, end_offset;
    uint32_t i;
    size_t header_prefix_len;
    size_t section_off_table_byte_pos;
    int schema_ours, tree_ours, tess_ours, geom_ours, names_real;
    prc_write_global_tables tables;
    uint8_t have_tables = 0;
    prc_bit_write_state schema_s, tree_s, model_s, geom_s, extra_geom_s;
    uint8_t *schema_comp = NULL, *tree_comp = NULL, *model_comp = NULL, *tess_comp = NULL;
    uint8_t *geom_comp = NULL, *extra_geom_comp = NULL;
    size_t schema_comp_len = 0, tree_comp_len = 0, model_comp_len = 0, tess_comp_len = 0;
    size_t geom_comp_len = 0, extra_geom_comp_len = 0;
    uint32_t *new_section_offsets;
    uint32_t new_start_offset, new_end_offset;
    size_t new_total_size;
    uint8_t *out_buf;
    FILE *out_fid;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;
    double bbox_min[3] = { -1.2345678, -1.2345678, -1.2345678 };
    double bbox_max[3] = { 1.2345678, 1.2345678, 1.2345678 };

    if (argc < 8)
    {
        printf("usage: %s <real_input.prc> <output.prc> <schema:real|ours> <tree:real|ours> <tess:real|ours> <geom:real|ours> <names:null|real> [output.pdf]\n", argv[0]);
        return 2;
    }
    schema_ours = parse_bool_arg(argv[3], "schema");
    tree_ours = parse_bool_arg(argv[4], "tree");
    tess_ours = parse_bool_arg(argv[5], "tess");
    geom_ours = parse_bool_arg(argv[6], "geom");
    /* names_real: when tree=ours, use real strings for model_name/root.name/
       part_node.name/part_node.part_name (matching teapot_write.c's own
       convention, which goes through the same prc_write_tree_to_stream/
       prc_write_model_file_to_stream functions but is Acrobat-confirmed
       working) instead of this tool's previous default of NULL everywhere.
       Isolates whether an unnamed tree/model -- a well-formed, spec-legal
       encoding (prc_write_name(NULL) writes a single "no name" bit) -- is
       nonetheless something Acrobat/PDF-XChange silently mishandle.
       Required (not optional) to avoid ambiguity with the trailing
       [output.pdf] argument. */
    if (strcmp(argv[7], "null") == 0) names_real = 0;
    else if (strcmp(argv[7], "real") == 0) names_real = 1;
    else { printf("invalid value for names: '%s' (expected 'null' or 'real')\n", argv[7]); return 2; }

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
    /* fs_uid: the real file's own file_info[0].unique_id, from its main
       header. MUST be threaded into a regenerated model section's own far
       reference (see the fix note further down and prc_write_model_file_
       to_stream's own doc comment on this exact class of mismatch) --
       the main header itself is always kept real/untouched by this tool,
       so a regenerated model section that instead used nanoPRC's own
       hardcoded PRC_WRITE_FILE_STRUCT_UID0 default would silently
       reference a file structure identity that doesn't match what the
       (real, untouched) main header actually declares. */
    real_fs_uid[0] = read_le_u32(p + 0);
    real_fs_uid[1] = read_le_u32(p + 4);
    real_fs_uid[2] = read_le_u32(p + 8);
    real_fs_uid[3] = read_le_u32(p + 12);
    p += 16; /* fs_uid */
    p += 4;  /* reserved */
    section_off_table_byte_pos = (size_t)(p - buf) + 4;
    section_count = read_le_u32(p); p += 4;
    header_prefix_len = (size_t)(p - buf);

    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
    start_offset = read_le_u32(p); p += 4;
    end_offset = read_le_u32(p); p += 4;

    if (section_count != 6)
    {
        /* The geom_ours byte-assembly path below only handles section 5
           (extra_geometry) as the last section before the model-file region
           -- i.e. exactly 6 sections (header/schema/tree/tess/geometry/
           extra_geometry). A file with more sections would need that path
           extended first; abort rather than silently mis-assemble. */
        printf("unexpected section_count=%u (this tool only handles exactly 6)\n", section_count);
        return 1;
    }
    printf("Real file: section_offsets=[");
    for (i = 0; i < section_count; i++) printf("%u ", section_offsets[i]);
    printf("] start=%u end=%u\n", start_offset, end_offset);
    printf("Requested: schema=%s tree=%s tess=%s\n", schema_ours ? "ours" : "real",
        tree_ours ? "ours" : "real", tess_ours ? "ours" : "real");

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    /* Schema (section 1) */
    if (schema_ours)
    {
        if (prc_write_global_tables_init(ctx, &tables) != 0) { printf("global_tables_init failed\n"); return 1; }
        have_tables = 1;
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
    }
    /* default_style_index stays 0 (no style reference) when schema=real --
       a regenerated tree paired with the real schema cannot reference a
       style index that doesn't exist in that real schema. Matches
       splice_tree_section.c's approach exactly. */

    /* Tree + model (section 2 + separate model-file region) -- always a
       matched pair, see this tool's header comment for why. */
    if (tree_ours)
    {
        memset(&rep_item, 0, sizeof(rep_item));
        rep_item.kind = PRC_API_WRITE_RI_SURFACE;
        rep_item.biased_tessellation_index = 1;
        rep_item.is_closed = 0;

        memset(&part_node, 0, sizeof(part_node));
        part_node.rep_items = &rep_item;
        part_node.num_rep_items = 1;
        part_node.bbox_min[0] = bbox_min[0]; part_node.bbox_min[1] = bbox_min[1]; part_node.bbox_min[2] = bbox_min[2];
        part_node.bbox_max[0] = bbox_max[0]; part_node.bbox_max[1] = bbox_max[1]; part_node.bbox_max[2] = bbox_max[2];
        part_node.name = names_real ? "teapot_body" : NULL;
        part_node.part_name = names_real ? "patch_faces" : NULL;

        part_node_ptr = &part_node;
        memset(&root, 0, sizeof(root));
        root.children = &part_node_ptr;
        root.num_children = 1;
        root.name = names_real ? "teapot" : NULL;

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

        memset(&model_s, 0, sizeof(model_s));
        if (prc_bitwrite_init(ctx, &model_s, 256) != 0) { printf("model bitwrite_init failed\n"); return 1; }
        /* _ex + real_fs_uid, NOT the plain wrapper (which hardcodes
           nanoPRC's own PRC_WRITE_FILE_STRUCT_UID0 constant as the far
           reference) -- see real_fs_uid's own comment above: the main
           header this model section will be embedded alongside is always
           kept real/untouched, so the far reference must match ITS
           declared file-structure identity, not nanoPRC's default. */
        if (prc_write_model_file_to_stream_ex(ctx, &model_s, names_real ? "nanoPRC" : NULL,
                root_biased_index, 1, real_fs_uid) != 0)
        {
            printf("prc_write_model_file_to_stream_ex failed\n");
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
    }

    /* Tessellation (section 3) */
    if (tess_ours)
    {
        if (!reencode_tess(ctx, buf, section_offsets[3], section_offsets[4], &tess_comp, &tess_comp_len))
        {
            printf("tess reencode failed or real tessellation out of scope for this tool -- aborting "
                "rather than silently falling back to real bytes\n");
            return 1;
        }
    }

    /* Geometry + extra_geometry (sections 4 + 5), always a matched pair. */
    if (geom_ours)
    {
        memset(&geom_s, 0, sizeof(geom_s));
        if (prc_bitwrite_init(ctx, &geom_s, 64) != 0) { printf("geom bitwrite_init failed\n"); return 1; }
        if (prc_write_geometry_section_to_stream(ctx, &geom_s) != 0)
        {
            printf("prc_write_geometry_section_to_stream failed\n");
            prc_api_print_error_stack(ctx);
            return 1;
        }
        if (prc_bitwrite_flush(ctx, &geom_s) != 0) { printf("geom bitwrite_flush failed\n"); return 1; }
        if (prc_write_deflate(ctx, geom_s.buf, geom_s.byte_pos, &geom_comp, &geom_comp_len) != 0)
        {
            printf("geom prc_write_deflate failed\n");
            return 1;
        }
        printf("Generated replacement geometry section: %zu bytes compressed\n", geom_comp_len);

        memset(&extra_geom_s, 0, sizeof(extra_geom_s));
        if (prc_bitwrite_init(ctx, &extra_geom_s, 64) != 0) { printf("extra_geom bitwrite_init failed\n"); return 1; }
        if (prc_write_extra_geometry_section_to_stream(ctx, &extra_geom_s) != 0)
        {
            printf("prc_write_extra_geometry_section_to_stream failed\n");
            prc_api_print_error_stack(ctx);
            return 1;
        }
        if (prc_bitwrite_flush(ctx, &extra_geom_s) != 0) { printf("extra_geom bitwrite_flush failed\n"); return 1; }
        if (prc_write_deflate(ctx, extra_geom_s.buf, extra_geom_s.byte_pos, &extra_geom_comp, &extra_geom_comp_len) != 0)
        {
            printf("extra_geom prc_write_deflate failed\n");
            return 1;
        }
        printf("Generated replacement extra_geometry section: %zu bytes compressed\n", extra_geom_comp_len);
    }

    /* Rebuild section_offsets cumulatively: section 0 (header) and any
       section beyond 5 (none for the reference files tested, but handled
       generically) are always real/unchanged length; 1 (schema), 2 (tree),
       3 (tess), 4+5 (geometry+extra_geometry) are conditionally replaced. */
    new_section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    new_section_offsets[0] = section_offsets[0];
    {
        uint32_t cur = section_offsets[0] + (section_offsets[1] - section_offsets[0]); /* header, always real */
        new_section_offsets[1] = cur;
        cur += schema_ours ? (uint32_t)schema_comp_len : (section_offsets[2] - section_offsets[1]);
        new_section_offsets[2] = cur;
        cur += tree_ours ? (uint32_t)tree_comp_len : (section_offsets[3] - section_offsets[2]);
        new_section_offsets[3] = cur;
        cur += tess_ours ? (uint32_t)tess_comp_len : (section_offsets[4] - section_offsets[3]);
        new_section_offsets[4] = cur;
        cur += geom_ours ? (uint32_t)geom_comp_len : (section_offsets[5] - section_offsets[4]);
        new_section_offsets[5] = cur;
        for (i = 6; i < section_count; i++)
        {
            cur += section_offsets[i] - section_offsets[i - 1];
            new_section_offsets[i] = cur;
        }
        cur += geom_ours ? (uint32_t)extra_geom_comp_len : (start_offset - section_offsets[section_count - 1]);
        new_start_offset = cur;
        new_end_offset = new_start_offset + (tree_ours ? (uint32_t)model_comp_len : (end_offset - start_offset));
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
    /* section 0: file-struct-header, always real, unchanged position */
    memcpy(out_buf + new_section_offsets[0], buf + section_offsets[0], section_offsets[1] - section_offsets[0]);
    /* section 1: schema */
    if (schema_ours)
        memcpy(out_buf + new_section_offsets[1], schema_comp, schema_comp_len);
    else
        memcpy(out_buf + new_section_offsets[1], buf + section_offsets[1], section_offsets[2] - section_offsets[1]);
    /* section 2: tree */
    if (tree_ours)
        memcpy(out_buf + new_section_offsets[2], tree_comp, tree_comp_len);
    else
        memcpy(out_buf + new_section_offsets[2], buf + section_offsets[2], section_offsets[3] - section_offsets[2]);
    /* section 3: tess */
    if (tess_ours)
        memcpy(out_buf + new_section_offsets[3], tess_comp, tess_comp_len);
    else
        memcpy(out_buf + new_section_offsets[3], buf + section_offsets[3], section_offsets[4] - section_offsets[3]);
    /* section 4: geometry */
    if (geom_ours)
        memcpy(out_buf + new_section_offsets[4], geom_comp, geom_comp_len);
    else
        memcpy(out_buf + new_section_offsets[4], buf + section_offsets[4], section_offsets[5] - section_offsets[4]);
    /* section 5..end (extra_geometry, ...): section 5 conditionally ours, any beyond it always real */
    if (geom_ours)
        memcpy(out_buf + new_section_offsets[5], extra_geom_comp, extra_geom_comp_len);
    else
        memcpy(out_buf + new_section_offsets[5], buf + section_offsets[5], start_offset - section_offsets[5]);
    /* model-file region */
    if (tree_ours)
        memcpy(out_buf + new_start_offset, model_comp, model_comp_len);
    else
        memcpy(out_buf + new_start_offset, buf + start_offset, end_offset - start_offset);

    out_fid = fopen(argv[2], "wb");
    if (out_fid == NULL) { printf("failed to open output %s\n", argv[2]); return 1; }
    fwrite(out_buf, 1, new_total_size, out_fid);
    fclose(out_fid);
    printf("Wrote %s (%zu bytes)\n", argv[2], new_total_size);

    if (argc >= 9)
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

        code = prc_api_pdf_embed_prc(ctx, argv[8], out_buf, new_total_size, &pdf_opts);
        if (code < 0)
        {
            printf("prc_api_pdf_embed_prc failed: %d\n", code);
            prc_api_print_error_stack(ctx);
        }
        else
        {
            printf("Wrote %s (with explicit Default view)\n", argv[8]);
        }
    }

    if (schema_comp != NULL) prc_free(ctx, schema_comp);
    if (tree_comp != NULL) prc_free(ctx, tree_comp);
    if (model_comp != NULL) prc_free(ctx, model_comp);
    if (tess_comp != NULL) prc_free(ctx, tess_comp);
    if (geom_comp != NULL) prc_free(ctx, geom_comp);
    if (extra_geom_comp != NULL) prc_free(ctx, extra_geom_comp);
    if (schema_ours) prc_bitwrite_release(ctx, &schema_s);
    if (tree_ours) { prc_bitwrite_release(ctx, &tree_s); prc_bitwrite_release(ctx, &model_s); }
    if (geom_ours) { prc_bitwrite_release(ctx, &geom_s); prc_bitwrite_release(ctx, &extra_geom_s); }
    if (have_tables) prc_write_global_tables_free(ctx, &tables);
    prc_api_release_context(ctx);
    free(section_offsets);
    free(new_section_offsets);
    free(out_buf);
    free(buf);
    return 0;
}
