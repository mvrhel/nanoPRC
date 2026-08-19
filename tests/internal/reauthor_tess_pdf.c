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

   WHAT: Decodes one PRC_API_TESS_3D_Compressed tessellation from a real
   file (via the public API, same triangle/primitive expansion as
   verify_manifold_pdf.c) and re-authors it as a brand-new, fully self-
   contained, single-part PRC file (fresh tree/schema, no cross-file-
   structure references at all) using the write facility's
   PRC_API_WRITE_TESS_KIND_COMPRESSED path, then wraps it in a minimal PDF
   via prc_api_pdf_embed_prc.

   Kind history: originally used PRC_API_WRITE_TESS_KIND_TRIANGLES (see
   prc_api.h's own note on why: COMPRESSED had a separate, already-known
   encoder bug against an independent, non-Acrobat ground-truth reader).
   TRIANGLES output turned out to have its own, different problem: Acrobat
   specifically (not PDF-XChange, not nanoPRC's own viewer, not that
   independent reader) showed a blank model tree for it, and neither an
   assembly/part tree-shape fix nor an explicit Default view nor correcting
   has_faces resolved it -- with no genuine real-world uncompressed-PRC file
   available locally to diff against, further guessing had no oracle to
   check against. Switched to COMPRESSED, which teapot_write.c's own
   Acrobat-verified output already demonstrates works in Acrobat; its
   separate known bug is against a different reader and may not manifest
   for this tool's simple single-part/single-face-group case.

   WHY IT EXISTS: A raw single-file-structure slice of a real multi-file-
   structure assembly (e.g. one produced by extracting a single fs from a
   larger container for isolated investigation) can decode fine through
   nanoPRC's own reader while still showing a blank model tree in real
   readers (Acrobat, PDF-XChange), because its product tree references a
   sibling file structure by UID that isn't present in the slice. Rather
   than chase that cross-reference (which may not even be resolvable
   without pulling in further, possibly-also-broken dependencies -- see
   the turbine-outer-shell investigation notes), this tool sidesteps the
   whole problem: decode the geometry, throw away the original tree/
   schema entirely, and write a fresh, minimal, unambiguous single-part
   file around it instead.

   HOW: usage: reauthor_tess_pdf.exe <input.pdf_or_prc> <tess_index>
   <output.pdf> [part_name] [kind=compressed|triangles] [mustcalc].
   <tess_index> is the public-API tessellation index (0-based, matching
   e.g. verify_manifold_pdf.c/scan_prc.c's own numbering for the same input
   file). kind must be "triangles" for "mustcalc" to have any effect --
   see standalone_grid_triangles.c's 2026-07-15 header update for why. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include "prc_api.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations, j;
    uint32_t num_extra_geom_tess;
    uint8_t has_lines = 0;
    int code;
    prc_api_tess tess;
    uint32_t target_index;
    const char *part_name = "reauthored part";

    double *positions = NULL;
    double *normals = NULL;
    uint32_t *tri_indices = NULL;
    uint32_t num_positions = 0, num_triangles = 0, tri_cap = 0;
    double bbox_min[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    double bbox_max[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };

    prc_api_write_tess_kind_t write_kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    int mustcalc = 0;
    if (argc < 4)
    {
        printf("usage: %s <input.pdf_or_prc> <tess_index> <output.pdf> [part_name] [kind=compressed|triangles] [mustcalc]\n", argv[0]);
        return 2;
    }
    target_index = (uint32_t)atoi(argv[2]);
    if (argc >= 5) part_name = argv[4];
    if (argc >= 6 && strcmp(argv[5], "triangles") == 0) write_kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
    if (argc >= 7 && strcmp(argv[6], "mustcalc") == 0) mustcalc = 1;

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("prc_api_open_contents failed\n"); prc_api_print_error_stack(ctx); return 1; }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0) { printf("prep_model_tree failed\n"); return 1; }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0) { printf("create_model_tree failed\n"); return 1; }

    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations, &totalLineTesselations, &num_extra_geom_tess);
    if (code < 0) { printf("get_number_tessellations failed\n"); return 1; }
    if (target_index >= totalTesselations) { printf("tess_index %u out of range (total %u)\n", target_index, totalTesselations); return 1; }

    memset(&tess, 0, sizeof(tess));
    {
        uint32_t nFaces = prc_api_get_number_faces(ctx, data, target_index);
        tess.num_faces = nFaces;
        tess.tess_faces = (prc_api_face *)calloc(nFaces, sizeof(prc_api_face));
        code = prc_api_initialize_tessellation(ctx, data, model_tree, target_index, &tess, NULL, &has_lines);
        if (code < 0) { printf("initialize_tessellation failed\n"); return 1; }
        if (tess.type != PRC_API_TESS_3D_Compressed && tess.type != PRC_API_TESS_3D)
        {
            printf("tess %u is not a 3D surface type (type=%d)\n", target_index, tess.type);
            return 1;
        }
        for (j = 0; j < tess.num_faces; j++)
        {
            code = prc_api_get_tessellation_vertices(ctx, data, target_index, j, tess.tess_faces + j, &tess);
            if (code < 0) { printf("get_tessellation_vertices face %u failed\n", j); return 1; }
        }
    }

    /* Pull positions/normals and expand every face's graphic primitives
       into a flat triangle list -- same technique as verify_manifold_pdf.c/
       detect_tess_spikes.c for the COMPRESSED case (one shared vertex
       buffer, primitive indices are already global). TRIANGLES is
       different: each face has its OWN face_vertices buffer (prc_api_face's
       own comment: "Only valid in the PRC_API_TESS_3D case"), and a
       primitive's indices are local to that face -- concatenate every
       face's vertices into one combined array and remap each face's
       primitive indices by its running offset, same technique
       demos/viewer/src/product.cpp uses ("Offset the primitive.indices by
       the vertex_offset of the face"). */
    {
        prc_api_tess_vertex_buffer *vb = (tess.type == PRC_API_TESS_3D_Compressed) ? &tess.tess_vertices : NULL;
        uint32_t face_limit = vb ? 1 : tess.num_faces;
        uint32_t f, v;
        uint32_t *face_vertex_offset = NULL; /* TRIANGLES case only */

        if (vb != NULL)
        {
            num_positions = vb->num_vertices;
            positions = (double *)malloc(sizeof(double) * 3 * num_positions);
            normals = (double *)malloc(sizeof(double) * 3 * num_positions);
            for (v = 0; v < num_positions; v++)
            {
                positions[v*3+0] = vb->vertices[v].position[0];
                positions[v*3+1] = vb->vertices[v].position[1];
                positions[v*3+2] = vb->vertices[v].position[2];
                normals[v*3+0] = vb->vertices[v].normal[0];
                normals[v*3+1] = vb->vertices[v].normal[1];
                normals[v*3+2] = vb->vertices[v].normal[2];
                if (positions[v*3+0] < bbox_min[0]) bbox_min[0] = positions[v*3+0];
                if (positions[v*3+1] < bbox_min[1]) bbox_min[1] = positions[v*3+1];
                if (positions[v*3+2] < bbox_min[2]) bbox_min[2] = positions[v*3+2];
                if (positions[v*3+0] > bbox_max[0]) bbox_max[0] = positions[v*3+0];
                if (positions[v*3+1] > bbox_max[1]) bbox_max[1] = positions[v*3+1];
                if (positions[v*3+2] > bbox_max[2]) bbox_max[2] = positions[v*3+2];
            }
        }
        else
        {
            uint32_t running = 0;
            face_vertex_offset = (uint32_t *)malloc(sizeof(uint32_t) * (face_limit ? face_limit : 1));
            for (f = 0; f < face_limit; f++)
                running += (uint32_t)tess.tess_faces[f].face_vertices.num_vertices;
            num_positions = running;
            positions = (double *)malloc(sizeof(double) * 3 * (num_positions ? num_positions : 1));
            normals = (double *)malloc(sizeof(double) * 3 * (num_positions ? num_positions : 1));

            running = 0;
            for (f = 0; f < face_limit; f++)
            {
                prc_api_tess_vertex_buffer *fvb = &tess.tess_faces[f].face_vertices;
                uint32_t nv = (uint32_t)fvb->num_vertices;
                face_vertex_offset[f] = running;
                for (v = 0; v < nv; v++)
                {
                    uint32_t gi = running + v;
                    positions[gi*3+0] = fvb->vertices[v].position[0];
                    positions[gi*3+1] = fvb->vertices[v].position[1];
                    positions[gi*3+2] = fvb->vertices[v].position[2];
                    normals[gi*3+0] = fvb->vertices[v].normal[0];
                    normals[gi*3+1] = fvb->vertices[v].normal[1];
                    normals[gi*3+2] = fvb->vertices[v].normal[2];
                    if (positions[gi*3+0] < bbox_min[0]) bbox_min[0] = positions[gi*3+0];
                    if (positions[gi*3+1] < bbox_min[1]) bbox_min[1] = positions[gi*3+1];
                    if (positions[gi*3+2] < bbox_min[2]) bbox_min[2] = positions[gi*3+2];
                    if (positions[gi*3+0] > bbox_max[0]) bbox_max[0] = positions[gi*3+0];
                    if (positions[gi*3+1] > bbox_max[1]) bbox_max[1] = positions[gi*3+1];
                    if (positions[gi*3+2] > bbox_max[2]) bbox_max[2] = positions[gi*3+2];
                }
                running += nv;
            }
        }

        for (f = 0; f < face_limit; f++)
        {
            prc_api_face *face = &tess.tess_faces[f];
            uint32_t voff = (vb == NULL) ? face_vertex_offset[f] : 0;
            uint32_t p;
            for (p = 0; p < face->num_graphic_primitives; p++)
            {
                prc_api_graphic_primitive prim;
                size_t idx;
                code = prc_api_get_graphics_primitive(ctx, data, &tess, (uint8_t)f, p, &prim);
                if (code < 0) continue;

#define APPEND_TRI(a,b,c) do { \
                    if (num_triangles*3 + 3 > tri_cap) { tri_cap = tri_cap ? tri_cap*2 : 4096; tri_indices = (uint32_t *)realloc(tri_indices, sizeof(uint32_t)*tri_cap); } \
                    tri_indices[num_triangles*3+0] = (a) + voff; tri_indices[num_triangles*3+1] = (b) + voff; tri_indices[num_triangles*3+2] = (c) + voff; \
                    num_triangles++; \
                } while (0)

                if (prim.type == PRC_API_TRIANGLES)
                {
                    for (idx = 0; idx + 3 <= prim.num_indices; idx += 3)
                        APPEND_TRI(prim.indices[idx+0], prim.indices[idx+1], prim.indices[idx+2]);
                }
                else if (prim.type == PRC_API_FAN)
                {
                    for (idx = 1; idx + 1 < prim.num_indices; idx++)
                        APPEND_TRI(prim.indices[0], prim.indices[idx], prim.indices[idx+1]);
                }
                else if (prim.type == PRC_API_STRIP)
                {
                    for (idx = 0; idx + 2 < prim.num_indices; idx++)
                    {
                        if (idx % 2 == 0)
                            APPEND_TRI(prim.indices[idx+0], prim.indices[idx+1], prim.indices[idx+2]);
                        else
                            APPEND_TRI(prim.indices[idx+1], prim.indices[idx+0], prim.indices[idx+2]);
                    }
                }
#undef APPEND_TRI
            }
        }
        free(face_vertex_offset);
    }

    printf("Decoded %u positions, %u triangles from tess %u\n", num_positions, num_triangles, target_index);
    if (num_triangles == 0) { printf("no triangles decoded, aborting\n"); return 1; }

    {
        prc_api_write_tessellation wtess;
        prc_api_write_rep_item rep_item;
        prc_api_write_node leaf;
        prc_api_write_node *leaf_ptr = &leaf;
        prc_api_write_node intermediate;
        prc_api_write_node *intermediate_ptr = &intermediate;
        prc_api_write_node root;
        uint32_t face_tri_counts = num_triangles;
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;

        memset(&wtess, 0, sizeof(wtess));
        wtess.kind = write_kind;
        /* tolerance/crease_angle_degrees left at their memset(0) defaults
           (relative 1e-6 / 30 degrees) -- crease_angle is moot anyway since
           normals are supplied below, not recalculated. Ignored entirely
           by the TRIANGLES path. */
        wtess.positions = positions;
        wtess.num_positions = num_positions;
        wtess.normals = normals;
        wtess.num_normals = num_positions;
        wtess.tri_indices = tri_indices;
        wtess.norm_indices = tri_indices; /* positions/normals are paired 1:1 in the decoded vertex buffer */
        wtess.num_triangles = num_triangles;
        wtess.face_tri_counts = &face_tri_counts;
        wtess.num_faces = 1;
        if (mustcalc)
        {
            /* must_calculate_normals=1: drop the decoded normals entirely,
               matching the real-oracle convention (see
               standalone_grid_triangles.c's own comment) -- the reader
               recomputes per-vertex normals from geometry + crease_angle.
               TRIANGLES-only; enforced by prc_write_tess_3d itself. */
            wtess.normals = NULL;
            wtess.num_normals = 0;
            wtess.norm_indices = NULL;
            wtess.must_calculate_normals = 1;
            wtess.crease_angle_degrees = 45.0;
        }

        memset(&rep_item, 0, sizeof(rep_item));
        rep_item.kind = PRC_API_WRITE_RI_SURFACE;
        rep_item.biased_tessellation_index = 1;
        rep_item.is_closed = 0;

        memset(&leaf, 0, sizeof(leaf));
        leaf.rep_items = &rep_item;
        leaf.num_rep_items = 1;
        leaf.bbox_min[0] = bbox_min[0]; leaf.bbox_min[1] = bbox_min[1]; leaf.bbox_min[2] = bbox_min[2];
        leaf.bbox_max[0] = bbox_max[0]; leaf.bbox_max[1] = bbox_max[1]; leaf.bbox_max[2] = bbox_max[2];
        leaf.name = part_name;
        leaf.part_name = part_name;

        /* Three non-Model levels -- root [no part] -> intermediate [empty
           part] -> leaf [real part] -- confirmed (2026-07-15, blank-tree
           evidence matrix row 19, ISO-SPEC/blanktree-matrix-evidence-
           table.md) to be the actual fix for Acrobat's blank model
           tree/canvas on OPEN-topology TRIANGLES meshes. A single empty
           part directly on the root (this tool's PREVIOUS fix attempt,
           `root.has_empty_part = 1` with root->leaf as a flat two-level
           tree) was insufficient by itself -- rows 13/14 still failed with
           that shape. Row 19 isolated the working structure by inserting
           exactly one more container level above the empty-part node,
           matching the deeper trees real, independently-produced PRC
           producers use (dump_tree_fields.c on ElevationMeshIS_ePRC.pdf/
           xml-sample-{wrl,iv,3ds}_ePRC.pdf/Teapot_ePRC.pdf: every
           intermediate level down to the leaf owns its own part, even an
           empty one, and there's more than one such level). */
        memset(&intermediate, 0, sizeof(intermediate));
        intermediate.children = &leaf_ptr;
        intermediate.num_children = 1;
        intermediate.name = part_name;
        intermediate.has_empty_part = 1;
        /* A has_empty_part node's own bbox is still serialized (see
           prc_write_tree.c's prc_write_part) and, contrary to an earlier
           assumption, is NOT safely ignorable by real Acrobat -- a
           degenerate (0,0,0)-(0,0,0) box here silently blanks everything
           nested beneath it. Reuse the one real child's own bbox. */
        intermediate.bbox_min[0] = bbox_min[0]; intermediate.bbox_min[1] = bbox_min[1]; intermediate.bbox_min[2] = bbox_min[2];
        intermediate.bbox_max[0] = bbox_max[0]; intermediate.bbox_max[1] = bbox_max[1]; intermediate.bbox_max[2] = bbox_max[2];

        memset(&root, 0, sizeof(root));
        root.children = &intermediate_ptr;
        root.num_children = 1;
        root.name = part_name;

        code = prc_api_write_prc_buffer(ctx, "nanoPRC-reauthor", &root, &wtess, 1, &prc_buf, &prc_buf_size);
        if (code < 0)
        {
            printf("prc_api_write_prc_buffer failed: %d\n", code);
            prc_api_print_error_stack(ctx);
            return 1;
        }

        /* Also give Acrobat an explicit "Default" view (matching
           teapot_write.c) rather than relying on a NULL-options reader
           fallback -- cheap, and removes one more variable from the
           Acrobat-blank-scene comparison. */
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;
            double center[3], extent[3], diag_len;
            int a;

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

            code = prc_api_pdf_embed_prc(ctx, argv[3], prc_buf, prc_buf_size, &pdf_opts);
        }
        if (code < 0)
        {
            printf("prc_api_pdf_embed_prc failed: %d\n", code);
            prc_api_print_error_stack(ctx);
            return 1;
        }

        printf("Wrote %s (%zu bytes of freshly-authored PRC embedded)\n", argv[3], prc_buf_size);
        prc_api_write_prc_buffer_free(ctx, prc_buf);
    }

    free(positions);
    free(normals);
    free(tri_indices);
    /* prc_api_release_data walks tess.tess_faces[*] to free its internal
       sub-allocations (prc_face_internal_face's reserved data, etc.) -- it
       must run BEFORE freeing the tess_faces array itself (this tool's own
       raw calloc at the top of main), or that walk reads freed memory. Only
       manifested as a hard crash on a larger tessellation (tess 901, 26540
       triangles) under ASan; a smaller one (tess 902) silently read freed-
       but-not-yet-reused memory without crashing. */
    prc_api_release_data(ctx, data, &tess, 1, NULL, 0, NULL, 0, model_tree);
    free(tess.tess_faces);
    prc_api_release_context(ctx);
    return 0;
}
