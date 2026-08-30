/* face_planarity_census -- how many faces in a real PRC file are genuinely planar?
 *
 * WHY THIS EXISTS
 * ---------------
 * The compressed-tessellation format can store one normal per face instead of
 * one per vertex, for faces that are planar, via the is_face_planar array
 * (Table 175, "Optional; if must_recalculate_normals is FALSE"). nanoPRC's
 * writer has never emitted it -- it passes NULL, so every face is encoded
 * per-vertex.
 *
 * Enabling it is not a flag flip. The base text says "The is_face_planar field
 * is TRUE if the face is planar. In this case, only one normal per face is
 * stored", and the decoder implements exactly that, so the flag and the normal
 * encoding must change together (see prc_encode_compute_face_planarity's
 * header comment in src/prc_write_compress_tess.c for the full constraint).
 * Before undertaking that work it is worth knowing what it would buy, which is
 * what this tool measures: across a file's tessellations, how many faces are
 * actually coplanar and what share of the mesh they cover.
 *
 * A second, independent motive: a correspondent reports that at least one real
 * reader validates a face's declared planarity against its actual geometry, so
 * the flag cannot be set unconditionally in either direction. That makes the
 * accuracy of the detector the thing to get right, and this tool is how its
 * output is inspected on real data rather than assumed.
 *
 * WHAT IT REPORTS
 * ---------------
 * Per file: faces examined, how many are planar, the triangle-weighted share
 * those faces represent, and how many planar faces are single triangles. The
 * last matters because a one-triangle face is trivially planar and worth
 * nothing, while a 200-triangle planar wall is where the saving would be --
 * a raw face percentage alone would flatter the result badly.
 *
 * The test itself is prc_encode_compute_face_planarity, the real writer-side
 * function, called here per face with a single-face mapping. This tool
 * deliberately does not reimplement the test, so what is measured is exactly
 * what a future writer would emit.
 *
 * usage:
 *   face_planarity_census <file.prc|file.pdf> [more files...]
 *   face_planarity_census --csv <files...>       one row per file
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "prc_api.h"
#include "prc_context.h"
#include "prc_write_compress_tess.h"

typedef struct
{
    uint32_t faces_total;
    uint32_t faces_planar;
    uint32_t tris_total;
    uint32_t tris_in_planar;
    uint32_t planar_singletons;
    uint32_t faces_skipped;
    uint32_t tess_compressed;   /* not measurable via the per-face path */
} census;

/* Test one face's triangle soup for coplanarity using the writer-side
   detector. The face's vertices arrive as 3-per-triangle (a soup, already
   materialised by the API), so the index array is simply 0,1,2,... */
static int
face_is_planar(prc_context *ctx, const prc_api_vertex *verts, size_t nverts,
    uint8_t *out_planar, uint32_t *out_tris)
{
    prc_encode_mesh mesh;
    double *pos = NULL;
    uint32_t *idx = NULL;
    uint32_t *face_indices = NULL;
    uint8_t *planar = NULL;
    uint32_t ntri, k;
    int code, rc = -1;

    *out_planar = 0;
    *out_tris = 0;

    if (nverts < 3)
        return -1;
    ntri = (uint32_t)(nverts / 3);
    if (ntri == 0)
        return -1;
    *out_tris = ntri;

    pos = (double *)malloc(nverts * 3 * sizeof(double));
    idx = (uint32_t *)malloc((size_t)ntri * 3 * sizeof(uint32_t));
    if (pos == NULL || idx == NULL)
        goto done;

    for (k = 0; k < nverts; k++)
    {
        pos[k * 3 + 0] = (double)verts[k].position[0];
        pos[k * 3 + 1] = (double)verts[k].position[1];
        pos[k * 3 + 2] = (double)verts[k].position[2];
    }
    for (k = 0; k < ntri * 3; k++)
        idx[k] = k;

    /* Relative tolerance: the census runs across files of wildly different
       scale, and the plane-distance test is expressed in the same resolved
       millimetres, so a fixed absolute value would be far too coarse on
       small parts and too fine on large assemblies. */
    code = prc_encode_preprocess(ctx, pos, (uint32_t)nverts, idx, ntri,
                                 prc_write_tol_relative(1.0e-4), &mesh);
    if (code < 0)
        goto done;
    if (mesh.num_triangles == 0)
    {
        prc_encode_preprocess_free(ctx, &mesh);
        goto done;
    }

    face_indices = (uint32_t *)calloc(mesh.num_triangles, sizeof(uint32_t));
    if (face_indices == NULL)
    {
        prc_encode_preprocess_free(ctx, &mesh);
        goto done;
    }

    code = prc_encode_compute_face_planarity(ctx, &mesh, face_indices, 1, &planar);
    if (code >= 0 && planar != NULL)
    {
        *out_planar = planar[0];
        *out_tris = mesh.num_triangles;
        rc = 0;
        prc_free(ctx, planar);
    }
    prc_encode_preprocess_free(ctx, &mesh);

done:
    free(pos);
    free(idx);
    free(face_indices);
    return rc;
}

static int
run_file(const char *path, int csv)
{
    prc_context *ctx = NULL;
    prc_api_data data = NULL;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t total_tess = 0, total_line_tess = 0, num_eg_tess = 0;
    uint8_t has_lines = 0;
    census c;
    uint32_t k, j;
    int code, rc = 1;

    memset(&c, 0, sizeof(c));

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "%s: prc_api_new_context failed\n", path);
        return 1;
    }

    data = prc_api_open_contents(ctx, path);
    if (data == NULL)
    {
        if (!csv) printf("%-46s  OPEN FAILED\n", path);
        prc_api_release_context(ctx);
        return 1;
    }

    if (prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups) < 0 ||
        prc_api_create_model_tree(ctx, data, &model_tree, num_parts,
                                  num_products, num_markups) < 0)
    {
        if (!csv) printf("%-46s  MODEL TREE FAILED\n", path);
        goto out;
    }

    if (prc_api_get_number_tessellations(ctx, data, model_tree, &total_tess,
                                         &total_line_tess, &num_eg_tess) < 0)
    {
        if (!csv) printf("%-46s  TESS COUNT FAILED\n", path);
        goto out;
    }
    if (total_tess == 0)
    {
        if (!csv) printf("%-46s  no tessellations\n", path);
        rc = 0;
        goto out;
    }

    tesses = (prc_api_tess *)calloc(total_tess, sizeof(prc_api_tess));
    if (tesses == NULL)
        goto out;

    for (k = 0; k < total_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t nfaces = prc_api_get_number_faces(ctx, data, k);

        tess->num_faces = nfaces;
        if (nfaces > 0)
        {
            tess->tess_faces = (prc_api_face *)calloc(nfaces, sizeof(prc_api_face));
            if (tess->tess_faces == NULL)
                goto out;
        }

        code = prc_api_initialize_tessellation(ctx, data, model_tree, k, tess,
                                               NULL, &has_lines);
        if (code < 0)
            continue;
        /* Only PRC_API_TESS_3D exposes per-face vertex buffers
           (face->face_vertices). PRC_API_TESS_3D_Compressed materialises the
           whole tessellation into a single tess->tess_vertices buffer and
           keeps the faces only as graphic-primitive ranges, so per-face
           triangles are not directly available on that path and those
           tessellations are counted separately rather than silently dropped.

           This does not bias the measurement. The question being asked is a
           geometric one -- what fraction of real CAD faces are planar -- and
           the answer does not depend on which encoding the file happened to
           use for its tessellation. */
        if (tess->type == PRC_API_TESS_3D_Compressed)
        {
            c.tess_compressed++;
            continue;
        }
        if (tess->type != PRC_API_TESS_3D)
            continue;

        for (j = 0; j < tess->num_faces; j++)
        {
            uint8_t planar = 0;
            uint32_t ntri = 0;

            if (prc_api_get_tessellation_vertices(ctx, data, k, j,
                                                  tess->tess_faces + j, tess) < 0)
                continue;

            if (face_is_planar(ctx, tess->tess_faces[j].face_vertices.vertices,
                               tess->tess_faces[j].face_vertices.num_vertices,
                               &planar, &ntri) < 0)
            {
                c.faces_skipped++;
                continue;
            }

            c.faces_total++;
            c.tris_total += ntri;
            if (planar)
            {
                c.faces_planar++;
                c.tris_in_planar += ntri;
                if (ntri == 1)
                    c.planar_singletons++;
            }
        }
    }

    if (csv)
    {
        printf("%s,%u,%u,%u,%u,%u,%u,%u\n", path, c.faces_total, c.faces_planar,
               c.tris_total, c.tris_in_planar, c.planar_singletons, c.faces_skipped,
               c.tess_compressed);
    }
    else
    {
        double fpct = c.faces_total ? 100.0 * c.faces_planar / c.faces_total : 0.0;
        double tpct = c.tris_total ? 100.0 * c.tris_in_planar / c.tris_total : 0.0;

        printf("%-46s faces %6u  planar %6u (%5.1f%%)  tris %8u  in-planar %8u (%5.1f%%)"
               "  1-tri-planar %5u  skipped %u  compressed-tess %u\n",
               path, c.faces_total, c.faces_planar, fpct,
               c.tris_total, c.tris_in_planar, tpct,
               c.planar_singletons, c.faces_skipped, c.tess_compressed);
    }
    rc = 0;

out:
    if (data != NULL)
        prc_api_release_data(ctx, data, tesses, tesses ? total_tess : 0,
                             NULL, 0, NULL, 0, model_tree);
    prc_api_release_context(ctx);
    return rc;
}

int
main(int argc, char **argv)
{
    int csv = 0, i, first = 1;

    if (argc > 1 && strcmp(argv[1], "--csv") == 0)
    {
        csv = 1;
        first = 2;
    }
    if (argc <= first)
    {
        fprintf(stderr,
                "usage: face_planarity_census [--csv] <file.prc|file.pdf> [...]\n");
        return 2;
    }
    if (csv)
        printf("file,faces,faces_planar,tris,tris_in_planar,"
               "planar_singletons,faces_skipped,tess_compressed\n");

    for (i = first; i < argc; i++)
        run_file(argv[i], csv);
    return 0;
}
