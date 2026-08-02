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

#include <prc_api.h>
#include <stdio.h>
#include <stdint.h>

#define MAX_VERTICES_PRINT 10

/* This is a simple demo that shows how to navigate through the model
   and the tessellation data in the PRC file */
int main(int argc, char *argv[])
{
    const char *file = NULL;
    int code;
    prc_api_product *model_tree;
    uint32_t num_parts, num_products, num_markups;
    uint32_t num_vertices_print;
    uint32_t i, j;
    uint32_t totalTesselations, totalLineTesselations, k;
    uint32_t num_extra_geom_tess = 0;
    uint8_t has_lines = 0;
    uint8_t vertics_printed = 0;
    uint32_t num_graphic_primitives;
    prc_api_face *face = NULL;
    uint8_t face_index = 0;

    if (argc != 2)
    {
        printf("Usage: nano_prc_quick_start <file>\n");
        return 1;
    }

    file = argv[1];

    if (!file)
    {
        printf("Usage: nano_prc_viewer <file> \n");
        return 1;
    }

    prc_context *ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        printf("prc_api_new_context failed\n");
        exit(1);
    }

    /* Do all the parsing */
    prc_api_data data = prc_api_open_contents(ctx, file);
    if (data == NULL)
    {
        /* If memory checking enabled this should catch any leaks that might
           occur due to a parsing error. Those should have been handled
           but this will provide a report if PRC_DEBUG_MEMORY is defined. */
        code = prc_api_release_context(ctx);
        printf("prc_api_open_contents failed\n");
        exit(1);
    }
    prc_api_print_error_stack(ctx);

    /* Lets create the model tree. */
    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0)
    {
        printf("prc_api_prep_model_tree failed\n");
        exit(1);
    }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts,
                                     num_products, num_markups);
    if (code < 0)
    {
        printf("prc_api_create_model_tree failed\n");
        exit(1);
    }

    /* Get the total number of tessellations we need, based upon the parts,
       the styles, the markups. Parts that use the same tessellation/style but just
       vary in spatial transformations will be using the same vertex information.
       Of course we still have to worry about the various faces of the
       tessellations.  Also, we may have to add line (wire) type tessellations
       when we in fact have a 3D compressed or 3D uncompressed tessellation.
       The compressed ones, we add line data for the extreme crease angles (not
       in the spec but Adobe does this) and we can have line data in the uncompressed
       tessellation */
    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations,
                                            &totalLineTesselations, &num_extra_geom_tess);
    if (code < 0)
    {
        printf("prc_api_get_number_tessellations failed\n");
        exit(1);
    }
    prc_api_tess *tesses = calloc(totalTesselations, sizeof(prc_api_tess));
    prc_api_tess *tesses_line = NULL;
    uint32_t line_tess_index = 0;
    if (tesses == NULL)
    {
        printf("failed to allocate tessellation array\n");
        exit(1);
    }

    /* Lets go through all the tessellations and get the data assigning it to
       the part and markups. We have to worry about different faces due to
       the styles that can vary across them.  Also, the 3D tessellations
       can generate line (wire) tessellations (not handled in this example) */
    for (k = 0; k < totalTesselations; k++)
    {
        prc_api_tess *tess = &tesses[k];

        uint32_t nFaces = prc_api_get_number_faces(ctx, data, k);
        tess->num_faces = nFaces;
        tess->tess_faces = calloc(nFaces, sizeof(prc_api_face));
        if (tess->tess_faces == NULL)
        {
            printf("failed to allocate tessellation faces array\n");
            exit(1);
        }

        int code = prc_api_initialize_tessellation(ctx, data, model_tree, k, tess,
                                                   NULL, &has_lines);
        if (code < 0)
        {
            printf("prc_api_initialize_tessellation failed\n");
            exit(1);
        }

        if (tess->type == PRC_API_TESS_UNKNOWN)
        {
            continue;
        }

        if (tess->type == PRC_API_TESS_3D_Wire ||
            tess->type == PRC_API_TESS_MarkUp)
        {
            /* 3D wire case and 3D markup cases */
            code = prc_api_get_tessellation_vertices(ctx, data, k, 0, NULL, tess);
            if (code < 0)
            {
                printf("prc_api_get_tessallation_vertices failed\n");
                exit(1);
            }
        }
        else
        {
            for (j = 0; j < tess->num_faces; j++)
            {
                code = prc_api_get_tessellation_vertices(ctx, data, k, j,
                    tess->tess_faces + j, tess);
                if (code < 0)
                {
                    printf("prc_api_get_tessallation_vertices failed\n");
                    exit(1);
                }
            }
        }
    }

    /* Lets print some of the tessellation data. Just do this for the first
       face of the first tessellation. Skip if they are markups or line types */
    if (totalTesselations > 0)
    {
        for (i = 0; i < totalTesselations; i++)
        {
            prc_api_tess *t = &tesses[i];
            printf("Tessellation %u: Type=%u, NumFaces=%zu\n", i, t->type, t->num_faces);
        }

        for (i = 0; i < totalLineTesselations; i++)
        {
            uint8_t printed_vertices = 0;
            prc_api_tess *tess = &tesses[i];

            if (printed_vertices)
            {
                /* Only print vertices for one tessellation to reduce the output noise */
                break;
            }

            if (tess->num_faces == 0)
            {
                /* tess->tess_faces[face_index] below is only valid when the
                   tessellation actually has faces -- calloc(0, ...) at
                   allocation time means indexing it here is a heap-buffer-
                   overflow (found via ASan on a real prc-db corpus file,
                   D110T_3D.stream-58.prc, whose 3D tessellations include at
                   least one with zero faces). face is reset to NULL so the
                   graphic-primitives block below doesn't see a stale
                   pointer left over from a previous loop iteration. */
                face = NULL;
                printf("Tessellation %d has no faces. Skipping vertex printout.\n", i);
            }
            else if (tess->type == PRC_API_TESS_3D)
            {
                /* In this structure we have different faces.  Here we just will
                   do face 0 */
                face = &tess->tess_faces[face_index];
                prc_api_tess_vertex_buffer *tess_vertices = &face->face_vertices;
                num_vertices_print = tess_vertices->num_vertices < MAX_VERTICES_PRINT ?
                                     tess_vertices->num_vertices : MAX_VERTICES_PRINT;

                printed_vertices = 1;
                printf("First %u vertices of first face of tessellation %d:\n", num_vertices_print, i);
                for (j = 0; j < num_vertices_print; j++)
                {
                    prc_api_vertex v = tess_vertices->vertices[j];
                    printf("  Vertex %u: Position=(%f, %f, %f), Normal=(%f, %f, %f)\n",
                        j, v.position[0], v.position[1], v.position[2], v.normal[0], v.normal[1], v.normal[2]);
                    printf("    Color: R=%f, G=%f, B=%f\n", v.color[0], v.color[1], v.color[2]);
                    printf("    Diffuse Color: R=%f, G=%f, B=%f\n", v.diffuse[0], v.diffuse[1], v.diffuse[2]);
                }
                printf("...\n");
            }
            else if (tess->type == PRC_API_TESS_3D_Compressed)
            {
                /* While PRC_API_TESS_3D_Compressed does have different faces
                   all the triangles are encoded in a compressed way so we keep this
                   as one face */
                prc_api_tess_vertex_buffer *tess_vertices = &tess->tess_vertices;

                /* Graphic primitives and indices are stored here though */
                face = &tess->tess_faces[face_index];

                num_vertices_print = tess_vertices->num_vertices < MAX_VERTICES_PRINT ?
                                     tess_vertices->num_vertices : MAX_VERTICES_PRINT;
                printf("First %u vertices compressed tessellation %d:\n", num_vertices_print, i);
                printed_vertices = 1;
                for (j = 0; j < num_vertices_print; j++)
                {
                    prc_api_vertex v = tess_vertices->vertices[j];
                    printf("  Vertex %u: Position=(%f, %f, %f), Normal=(%f, %f, %f)\n",
                        j, v.position[0], v.position[1], v.position[2], v.normal[0], v.normal[1], v.normal[2]);
                    printf("    Color: R=%f, G=%f, B=%f\n", v.color[0], v.color[1], v.color[2]);
                    printf("    Diffuse Color: R=%f, G=%f, B=%f\n", v.diffuse[0], v.diffuse[1], v.diffuse[2]);
                }
                printf("...\n");
            }
            else
            {
                face = NULL;
                printf("Tessellation %d is not a 3D type. Skipping vertex printout.\n", i);
            }

            /* Lets print out some indices and graphic primitives that index
               into the vertices */
            if (face != NULL)
            {
                num_graphic_primitives = face->num_graphic_primitives;
                for (j = 0; j < num_graphic_primitives; j++)
                {
                    /* Get primitive from PRC data */
                    prc_api_graphic_primitive primitive;
                    code = prc_api_get_graphics_primitive(ctx, data,
                        (prc_api_tess *)tess, face_index, j, &primitive);
                    if (code < 0)
                    {
                        printf("prc_api_get_graphics_primitive failed\n");
                        exit(1);
                    }

                    printf("\n");
                    switch (primitive.type)
                    {
                    default:
                        printf("unknown primitive type\n");
                        exit(1);
                    case PRC_API_TRIANGLES:
                        printf("GL_TRIANGLES primitive with %zu indices\n", primitive.num_indices);
                        break;
                    case PRC_API_FAN:
                        printf("GL_TRIANGLE_FAN primitive with %zu indices\n", primitive.num_indices);
                        break;
                    case PRC_API_STRIP:
                        printf("GL_TRIANGLE_STRIP primitive with %zu indices\n", primitive.num_indices);
                        break;
                    case PRC_API_LINE:
                        printf("GL_LINES primitive with %zu indices\n", primitive.num_indices);
                        break;
                    case PRC_API_LINE_STRIP:
                        printf("GL_LINE_STRIP primitive with %zu indices\n", primitive.num_indices);
                        break;
                    case PRC_API_LINE_LOOP:
                        printf("GL_LINE_LOOP primitive with %zu indices\n", primitive.num_indices);
                        break;
                    }
                    for (k = 0; k < primitive.num_indices && k < MAX_VERTICES_PRINT; k++)
                    {
                        printf("    Index %u: %u\n", k, primitive.indices[k]);
                    }
                }
                printf("...\n");
            }
        }
    }

    /* Clean up */
    prc_api_release_data(ctx, data, tesses, totalTesselations, NULL, 0, model_tree);
    code = prc_api_release_context(ctx);

    return 0;
}
