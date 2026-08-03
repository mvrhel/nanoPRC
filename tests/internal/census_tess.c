/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Walks every tessellation index (0..totalTesselations-1) via the
   normal public API and prints its type, face count, and vertex count.
   Built specifically because none of the existing tools resolve a named
   part (from dump_tree_fields.c's tree walk -- which reports
   tess_file_index/biased_tess_index, a per-file-structure LOCAL index)
   into the flat tess_index the rest of the public API (and every other
   internal tool in this directory) actually uses; that mapping is buried
   inside prc_api_get_number_tessellations's internal part-matching loop
   and isn't exposed. This censuses every tessellation's size instead, so
   a specific named part (e.g. one covering most of a large assembly, and
   therefore a size outlier) can be identified by cross-referencing size
   against what's visible in another viewer.

   HOW: usage: census_tess.exe <input.pdf> [min_vertices=0] -- only prints
   tessellations with at least min_vertices vertices, to cut the output
   down to plausible candidates for a large/distinctive part.

   KNOWN LIMITATION, FIXED: originally only read tess.tess_vertices, which
   is populated exclusively for PRC_API_TESS_3D_Compressed -- for
   PRC_API_TESS_3D (uncompressed/"TRIANGLES") tessellations, per-vertex
   data instead lives per-face in tess.tess_faces[f].face_vertices, so
   every TRIANGLES-type tessellation silently censused as num_vertices=0.
   Fixed the same way detect_tess_spikes.c and reauthor_tess_pdf.c already
   were: sum each face's own face_vertices.num_vertices for the TRIANGLES
   case instead of reading tess_vertices directly. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations, k, j;
    uint32_t num_extra_geom_tess;
    uint8_t has_lines = 0;
    int code;
    uint32_t min_vertices = 0;

    if (argc < 2) { printf("usage: %s <input.pdf> [min_vertices=0]\n", argv[0]); return 2; }
    if (argc >= 3) min_vertices = (uint32_t)atoi(argv[2]);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }
    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("open_contents failed\n"); prc_api_print_error_stack(ctx); return 1; }
    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0) { printf("prep_model_tree failed\n"); return 1; }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0) { printf("create_model_tree failed\n"); return 1; }
    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations, &totalLineTesselations, &num_extra_geom_tess);
    if (code < 0) { printf("get_number_tessellations failed\n"); return 1; }

    printf("total_tessellations=%u\n", totalTesselations);

    for (k = 0; k < totalTesselations; k++)
    {
        prc_api_tess tess;
        uint32_t nFaces;
        uint8_t hl = 0;
        const char *type_name = "UNKNOWN";

        memset(&tess, 0, sizeof(tess));
        nFaces = prc_api_get_number_faces(ctx, data, k);
        tess.num_faces = nFaces;
        tess.tess_faces = (prc_api_face *)calloc(nFaces, sizeof(prc_api_face));
        code = prc_api_initialize_tessellation(ctx, data, model_tree, k, &tess, NULL, &hl);
        if (code < 0) { free(tess.tess_faces); continue; }

        switch (tess.type)
        {
            case PRC_API_TESS_3D: type_name = "TRIANGLES"; break;
            case PRC_API_TESS_3D_Compressed: type_name = "COMPRESSED"; break;
            case PRC_API_TESS_3D_Wire: type_name = "WIRE"; break;
            case PRC_API_TESS_MarkUp: type_name = "MARKUP"; break;
            default: break;
        }

        if (tess.type == PRC_API_TESS_3D_Compressed || tess.type == PRC_API_TESS_3D)
        {
            size_t num_vertices;

            for (j = 0; j < tess.num_faces; j++)
                prc_api_get_tessellation_vertices(ctx, data, k, j, tess.tess_faces + j, &tess);

            if (tess.type == PRC_API_TESS_3D_Compressed)
            {
                num_vertices = tess.tess_vertices.num_vertices;
            }
            else
            {
                /* TRIANGLES: vertices live per-face, not in tess.tess_vertices. */
                num_vertices = 0;
                for (j = 0; j < tess.num_faces; j++)
                    num_vertices += tess.tess_faces[j].face_vertices.num_vertices;
            }

            if (num_vertices >= min_vertices)
            {
                printf("tess=%u type=%s num_faces=%u num_vertices=%zu\n",
                    k, type_name, nFaces, num_vertices);
            }
        }
        free(tess.tess_faces);
    }

    prc_api_release_context(ctx);
    return 0;
}
