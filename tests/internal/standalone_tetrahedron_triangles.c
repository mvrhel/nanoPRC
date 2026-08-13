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

   WHAT: Writes a fully-standalone PRC/PDF file (schema+tree+tess+model+
   geometry+extra_geometry ALL generated fresh) for the same tetrahedron
   geometry as xml-sample-wrl_ePRC.pdf, using PRC_API_WRITE_TESS_KIND_
   TRIANGLES, via the SAME public API entry point (prc_api_write_prc_buffer)
   that teapot_write.c uses (Acrobat-confirmed working, COMPRESSED kind) --
   not matrix_blank_tree.c's hand-rolled section splicer.

   WHY IT EXISTS: matrix_blank_tree.c's splicer is ~600 lines of manual
   section-offset arithmetic; every sanity check run on its output so far
   (byte-identical all-real round-trip, nanoPRC's own reader parsing
   cleanly) only proves the ASSEMBLY is well-formed, not that it's
   byte-identical to what the standard public API would produce for
   equivalent content. This tool removes the splicer from the picture
   entirely -- if THIS also fails in Acrobat/PDF-XChange, the defect is
   confirmed to be in the TRIANGLES write path itself (or its interaction
   with tree/model), not an artifact of the splice tool. If it works, the
   splice tool has a bug distinct from the underlying writer functions.

   HOW: usage: standalone_tetrahedron_triangles.exe <output.prc>
   [normals:none|pervertex] [output.pdf]

   normals=pervertex supplies explicit per-corner normals with
   norm_indices != NULL (one real normal value per triangle corner, computed
   from that triangle's own geometric face normal -- structurally identical
   in SHAPE to what reauthor_tess_pdf.c supplies for real decoded geometry,
   even though the VALUES here happen to be flat-per-face). This routes
   prc_write_tess_3d through its PRC_FACETESSDATA_Triangle branch, not the
   PRC_FACETESSDATA_TriangleOneNormal branch normals=none (the default) uses
   -- see ISO-SPEC/blanktree-matrix-evidence-table.md's rows 13/14 analysis
   for why this distinction matters: every other TRIANGLES test built this
   session used normals=NULL, so the Triangle (real per-vertex normal) code
   path was never actually exercised until reauthor_tess_pdf.c's real,
   large-scale turbine output -- the ONLY thing rows 13/14 still fail on. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    /* Same 4 positions as xml-sample-wrl_ePRC.pdf's real tetrahedron
       (confirmed via dump_uncompressed_tess_fields.exe --find). */
    double positions[4 * 3] = {
        -1.0, -1.0,  1.0,
        -1.0,  1.0, -1.0,
         1.0, -1.0, -1.0,
         1.0,  1.0,  1.0
    };
    /* Same triangle_indices/triangulated_index_array as the real file
       (triangulated_index_array 0 3 6 3 0 9 6 9 0 9 6 3, each /3 to get
       vertex index): triangles (0,1,2) (1,0,3) (2,3,0) (3,2,1). */
    uint32_t tri_indices[4 * 3] = {
        0, 1, 2,
        1, 0, 3,
        2, 3, 0,
        3, 2, 1
    };
    uint32_t face_tri_counts[1] = { 4 };
    double vertex_normals[4 * 3];
    uint32_t vertex_normal_count[4] = { 0, 0, 0, 0 };
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;
    int pervertex = 0;
    const char *out_pdf = NULL;
    uint32_t t;

    if (argc < 2)
    {
        printf("usage: %s <output.prc> [normals:none|pervertex] [output.pdf]\n", argv[0]);
        return 2;
    }
    if (argc >= 3)
    {
        if (strcmp(argv[2], "pervertex") == 0) { pervertex = 1; if (argc >= 4) out_pdf = argv[3]; }
        else if (strcmp(argv[2], "none") == 0) { pervertex = 0; if (argc >= 4) out_pdf = argv[3]; }
        else { out_pdf = argv[2]; } /* backward-compat: argv[2] is a pdf path */
    }

    memset(vertex_normals, 0, sizeof(vertex_normals));
    if (pervertex)
    {
        /* Real (smooth) vertex normals: average the geometric face normal
           of every triangle touching each vertex -- one real normal value
           per POSITION (num_normals == num_positions), referenced via
           norm_indices == tri_indices, exactly the convention
           reauthor_tess_pdf.c uses for real decoded geometry ("positions/
           normals are paired 1:1 in the decoded vertex buffer"). */
        for (t = 0; t < 4; t++)
        {
            uint32_t i0 = tri_indices[t * 3 + 0], i1 = tri_indices[t * 3 + 1], i2 = tri_indices[t * 3 + 2];
            double e1[3], e2[3], n[3], len;
            uint32_t c;

            for (c = 0; c < 3; c++) e1[c] = positions[i1 * 3 + c] - positions[i0 * 3 + c];
            for (c = 0; c < 3; c++) e2[c] = positions[i2 * 3 + c] - positions[i0 * 3 + c];
            n[0] = e1[1] * e2[2] - e1[2] * e2[1];
            n[1] = e1[2] * e2[0] - e1[0] * e2[2];
            n[2] = e1[0] * e2[1] - e1[1] * e2[0];
            len = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (len > 1e-12) { n[0] /= len; n[1] /= len; n[2] /= len; }

            {
                uint32_t verts[3]; verts[0] = i0; verts[1] = i1; verts[2] = i2;
                for (c = 0; c < 3; c++)
                {
                    uint32_t v = verts[c];
                    vertex_normals[v * 3 + 0] += n[0];
                    vertex_normals[v * 3 + 1] += n[1];
                    vertex_normals[v * 3 + 2] += n[2];
                    vertex_normal_count[v]++;
                }
            }
        }
        for (t = 0; t < 4; t++)
        {
            double len;
            if (vertex_normal_count[t] == 0) continue;
            vertex_normals[t * 3 + 0] /= vertex_normal_count[t];
            vertex_normals[t * 3 + 1] /= vertex_normal_count[t];
            vertex_normals[t * 3 + 2] /= vertex_normal_count[t];
            len = sqrt(vertex_normals[t*3+0]*vertex_normals[t*3+0] + vertex_normals[t*3+1]*vertex_normals[t*3+1] + vertex_normals[t*3+2]*vertex_normals[t*3+2]);
            if (len > 1e-12) { vertex_normals[t*3+0] /= len; vertex_normals[t*3+1] /= len; vertex_normals[t*3+2] /= len; }
        }
    }

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
    tess.positions = positions;
    tess.num_positions = 4;
    tess.normals = pervertex ? vertex_normals : NULL;
    tess.num_normals = pervertex ? 4 : 0;
    tess.tri_indices = tri_indices;
    tess.norm_indices = pervertex ? tri_indices : NULL;
    tess.num_triangles = 4;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = -1.5; part_node.bbox_min[1] = -1.5; part_node.bbox_min[2] = -1.5;
    part_node.bbox_max[0] = 1.5; part_node.bbox_max[1] = 1.5; part_node.bbox_max[2] = 1.5;
    part_node.name = "tetra_body";
    part_node.part_name = "tetra_faces";

    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "tetrahedron";

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;
        FILE *out;

        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, &tess, 1, &prc_buf, &prc_buf_size);
        if (code != 0)
        {
            printf("prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            prc_api_release_context(ctx);
            return 1;
        }

        out = fopen(argv[1], "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            printf("failed to write %s\n", argv[1]);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            return 1;
        }
        fclose(out);
        printf("Wrote %s (%zu bytes)\n", argv[1], prc_buf_size);

        if (out_pdf != NULL)
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = 5.0; view.eye[1] = -6.5; view.eye[2] = 3.5;
            view.target[0] = 0.0; view.target[1] = 0.0; view.target[2] = 0.0;
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;

            code = prc_api_pdf_embed_prc(ctx, out_pdf, prc_buf, prc_buf_size, &pdf_opts);
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

        prc_api_write_prc_buffer_free(ctx, prc_buf);
    }

    prc_api_release_context(ctx);
    return 0;
}
