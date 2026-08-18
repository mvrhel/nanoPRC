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

   WHAT: Same open 2-triangle quad geometry as standalone_grid_triangles.exe
   N=2 (row 17a in the blank-tree evidence matrix, confirmed FAILING in
   Acrobat), same must_calculate_normals=0 + explicit per-vertex-normal
   convention (also already confirmed failing on its own) -- but with a
   THREE-LEVEL tree (Root -> Intermediate [empty part, has_empty_part=1] ->
   Leaf [real part, the quad's rep_item]) instead of the flat two-level
   (Root -> Leaf) tree every other test in this matrix has used.

   WHY IT EXISTS: dump_tree_fields.exe on BOTH real oracle files
   (xml-sample-wrl_ePRC.pdf: closed, works; ElevationMeshIS_ePRC.pdf: open,
   works) shows a 3-4 level tree with empty intermediate parts -- neither
   matches this session's flat 2-level test convention. But row 15 (closed
   tetrahedron, flat 2-level tree) WORKS despite not matching that real
   structure, so depth isn't required for closed geometry. Whether it's
   required specifically for OPEN geometry is a new, previously untested
   combination -- this isolates exactly that, changing ONLY tree depth
   relative to row 17a (keeping must_calculate_normals=0, already known not
   to be the fix on its own, so a positive result here implicates tree
   depth specifically, not a combination with the normals question).

   HOW: usage: standalone_quad_deeptree.exe <output.prc> [output.pdf] */
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
    double positions[4 * 3];
    double normals[4 * 3];
    uint32_t tri_indices[2 * 3];
    uint32_t face_tri_counts[1] = { 2 };
    const char *out_pdf = NULL;
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node leaf, intermediate, root;
    prc_api_write_node *leaf_ptr, *intermediate_ptr;
    uint32_t xi, yi;

    if (argc < 2)
    {
        printf("usage: %s <output.prc> [output.pdf]\n", argv[0]);
        return 2;
    }
    if (argc >= 3) out_pdf = argv[2];

    /* Identical quad geometry to standalone_grid_triangles.exe N=2. */
    {
        const double radius = 5.0, half_angle = 60.0 * 3.14159265358979323846 / 180.0, half_length = 5.0;
        for (yi = 0; yi < 2; yi++)
            for (xi = 0; xi < 2; xi++)
            {
                uint32_t i = yi * 2 + xi;
                double theta = ((double)xi * 2.0 - 1.0) * half_angle;
                double along = ((double)yi * 2.0 - 1.0) * half_length;
                double nx = sin(theta), ny = cos(theta);
                positions[i * 3 + 0] = radius * nx;
                positions[i * 3 + 1] = radius * ny;
                positions[i * 3 + 2] = along;
                normals[i * 3 + 0] = nx;
                normals[i * 3 + 1] = ny;
                normals[i * 3 + 2] = 0.0;
            }
        tri_indices[0] = 0; tri_indices[1] = 1; tri_indices[2] = 3;
        tri_indices[3] = 0; tri_indices[4] = 3; tri_indices[5] = 2;
    }

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
    tess.positions = positions;
    tess.num_positions = 4;
    tess.normals = normals;
    tess.num_normals = 4;
    tess.tri_indices = tri_indices;
    tess.norm_indices = tri_indices;
    tess.num_triangles = 2;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    /* Leaf: the real part, owns the geometry -- matches "Mesh_0" /
       "shape_0" in the real oracle dumps. */
    memset(&leaf, 0, sizeof(leaf));
    leaf.rep_items = &rep_item;
    leaf.num_rep_items = 1;
    leaf.bbox_min[0] = -4.5; leaf.bbox_min[1] = 2.0; leaf.bbox_min[2] = -5.5;
    leaf.bbox_max[0] = 4.5; leaf.bbox_max[1] = 5.5; leaf.bbox_max[2] = 5.5;
    leaf.name = "quad_leaf";
    leaf.part_name = "quad_leaf_part";

    /* Intermediate: a bare scene/assembly node that STILL owns its own
       (empty) part -- matches "PDF3D IntermediateScene..."/"PDF3D Scene"
       in the real oracle dumps (num_rep_items=0 but has_part=1 there). */
    leaf_ptr = &leaf;
    memset(&intermediate, 0, sizeof(intermediate));
    intermediate.children = &leaf_ptr;
    intermediate.num_children = 1;
    intermediate.name = "quad_scene";
    intermediate.has_empty_part = 1;
    /* A has_empty_part node's own bbox is still serialized and NOT safely
       ignorable by real Acrobat -- see stl_import.c's own fix for the same
       pattern. Reuse the one real child's bbox. */
    intermediate.bbox_min[0] = leaf.bbox_min[0]; intermediate.bbox_min[1] = leaf.bbox_min[1]; intermediate.bbox_min[2] = leaf.bbox_min[2];
    intermediate.bbox_max[0] = leaf.bbox_max[0]; intermediate.bbox_max[1] = leaf.bbox_max[1]; intermediate.bbox_max[2] = leaf.bbox_max[2];

    /* Root: matches "Model" in the real oracle dumps (has_part=0, pure
       container). */
    intermediate_ptr = &intermediate;
    memset(&root, 0, sizeof(root));
    root.children = &intermediate_ptr;
    root.num_children = 1;
    root.name = "quad_root";

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
            view.eye[0] = 15.0; view.eye[1] = -15.0; view.eye[2] = 12.0;
            view.target[0] = 0.0; view.target[1] = 3.75; view.target[2] = 0.0;
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
