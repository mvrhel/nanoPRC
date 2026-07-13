/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Decodes one PRC_API_TESS_3D_Compressed tessellation from a real
   file (via the public API, same triangle/primitive expansion as
   verify_manifold_pdf.c) and re-authors it as a brand-new, fully self-
   contained, single-part PRC file (fresh tree/schema, no cross-file-
   structure references at all) using the write facility's
   PRC_API_WRITE_TESS_KIND_TRIANGLES path (see prc_api.h's own note on
   why: PRC_API_WRITE_TESS_KIND_COMPRESSED has a separate, already-known
   unresolved encoder bug independent of anything investigated here), then
   wraps it in a minimal PDF via prc_api_pdf_embed_prc.

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
   <output.pdf> [part_name]. <tess_index> is the public-API tessellation
   index (0-based, matching e.g. verify_manifold_pdf.c/scan_prc.c's own
   numbering for the same input file). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "prc_api.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations, j;
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

    if (argc < 4)
    {
        printf("usage: %s <input.pdf_or_prc> <tess_index> <output.pdf> [part_name]\n", argv[0]);
        return 2;
    }
    target_index = (uint32_t)atoi(argv[2]);
    if (argc >= 5) part_name = argv[4];

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("prc_api_open_contents failed\n"); prc_api_print_error_stack(ctx); return 1; }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0) { printf("prep_model_tree failed\n"); return 1; }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0) { printf("create_model_tree failed\n"); return 1; }

    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations, &totalLineTesselations);
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
            code = prc_api_get_tessellation_vertices(ctx, data, model_tree, target_index, j, tess.tess_faces + j, &tess);
            if (code < 0) { printf("get_tessellation_vertices face %u failed\n", j); return 1; }
        }
    }

    /* Pull positions/normals straight from the decoded vertex buffer, and
       expand every face's graphic primitives into a flat triangle list --
       same technique as verify_manifold_pdf.c/detect_tess_spikes.c. */
    {
        prc_api_tess_vertex_buffer *vb = (tess.type == PRC_API_TESS_3D_Compressed) ? &tess.tess_vertices : NULL;
        uint32_t face_limit = vb ? 1 : tess.num_faces;
        uint32_t f, v;

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

        for (f = 0; f < face_limit; f++)
        {
            prc_api_face *face = &tess.tess_faces[f];
            uint32_t p;
            for (p = 0; p < face->num_graphic_primitives; p++)
            {
                prc_api_graphic_primitive prim;
                size_t idx;
                code = prc_api_get_graphics_primitive(ctx, data, &tess, (uint8_t)f, p, &prim);
                if (code < 0) continue;

#define APPEND_TRI(a,b,c) do { \
                    if (num_triangles*3 + 3 > tri_cap) { tri_cap = tri_cap ? tri_cap*2 : 4096; tri_indices = (uint32_t *)realloc(tri_indices, sizeof(uint32_t)*tri_cap); } \
                    tri_indices[num_triangles*3+0] = (a); tri_indices[num_triangles*3+1] = (b); tri_indices[num_triangles*3+2] = (c); \
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
    }

    printf("Decoded %u positions, %u triangles from tess %u\n", num_positions, num_triangles, target_index);
    if (num_triangles == 0) { printf("no triangles decoded, aborting\n"); return 1; }

    {
        prc_api_write_tessellation wtess;
        prc_api_write_rep_item rep_item;
        prc_api_write_node leaf;
        uint32_t face_tri_counts = num_triangles;
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;

        memset(&wtess, 0, sizeof(wtess));
        wtess.kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
        wtess.positions = positions;
        wtess.num_positions = num_positions;
        wtess.normals = normals;
        wtess.num_normals = num_positions;
        wtess.tri_indices = tri_indices;
        wtess.norm_indices = tri_indices; /* positions/normals are paired 1:1 in the decoded vertex buffer */
        wtess.num_triangles = num_triangles;
        wtess.face_tri_counts = &face_tri_counts;
        wtess.num_faces = 1;

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

        code = prc_api_write_prc_buffer(ctx, "nanoPRC-reauthor", &leaf, &wtess, 1, &prc_buf, &prc_buf_size);
        if (code < 0)
        {
            printf("prc_api_write_prc_buffer failed: %d\n", code);
            prc_api_print_error_stack(ctx);
            return 1;
        }

        code = prc_api_pdf_embed_prc(ctx, argv[3], prc_buf, prc_buf_size, NULL);
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
    prc_api_release_context(ctx);
    return 0;
}
