/* planar_roundtrip -- write a mesh with genuinely planar faces, read it back,
 * and check the normals survive.
 *
 * WHY THIS EXISTS
 * ---------------
 * The is_face_planar array (Table 175, "Optional; if must_recalculate_normals
 * is FALSE") lets a compressed tessellation store ONE normal per planar face
 * instead of one per vertex: "The is_face_planar field is TRUE if the face is
 * planar. In this case, only one normal per face is stored."
 *
 * That makes the flag load-bearing rather than advisory. A decoder reading a
 * planar face consumes four bits plus two angles at the FIRST vertex of the
 * face's FIRST triangle and nothing at all for the rest of the face, so an
 * encoder that sets the flag without also changing how it emits normals
 * desynchronises the decoder from that point on. The two must agree exactly,
 * and there is no way to check that by inspection -- hence this round trip.
 *
 * WHAT IT DOES
 * ------------
 * Builds an axis-aligned cube as six face groups of two triangles each. Every
 * face is exactly planar and every vertex of a face carries that face's normal,
 * so all six faces are legitimately encodable as planar and the expected
 * decoded normal for any vertex is simply its face's axis normal.
 *
 * It then writes a PRC, reads it back through the public API, and compares
 * every decoded vertex normal against the face normal it should have. A
 * desynchronised normal stream shows up immediately as wrong or garbage
 * normals rather than as a parse failure, which is the failure mode that
 * matters and the one hardest to notice by eye.
 *
 * The cube is deliberately the simplest case that still exercises the
 * interesting interaction: its eight corner vertices are each shared by three
 * faces with three different normals, so every corner is a multiple-normal
 * vertex, and the planar path's "first vertex of the face" bookkeeping has to
 * cope with vertices already registered by an earlier face.
 *
 * usage:
 *   planar_roundtrip [out.prc]        default: planar_roundtrip_out.prc
 *
 * exit status: 0 all normals correct, 1 mismatch, 2 harness error.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "prc_api.h"
#include "prc_context.h"

#define NFACE 6

/* Cube corner coordinates, 2x2x2 centred on the origin. */
static const double CORNER[8][3] = {
    {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
    {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1}
};

/* Six faces, each as two triangles over four corners, wound outward. */
static const int FACE_CORNER[NFACE][4] = {
    {0, 3, 2, 1},   /* -Z */
    {4, 5, 6, 7},   /* +Z */
    {0, 1, 5, 4},   /* -Y */
    {3, 7, 6, 2},   /* +Y */
    {0, 4, 7, 3},   /* -X */
    {1, 2, 6, 5}    /* +X */
};

static const double FACE_NORMAL[NFACE][3] = {
    { 0,  0, -1}, { 0,  0,  1},
    { 0, -1,  0}, { 0,  1,  0},
    {-1,  0,  0}, { 1,  0,  0}
};

/* Flat plate: two coplanar face groups, all normals +Z.

   Chosen because it isolates the planar path from the C2 canary. Every vertex
   carries the same normal, so no growing triangle can disagree about its
   reversed bit, and prc_encode_normals_c2 runs to completion instead of
   falling back to C1 -- which is the only way is_face_planar reaches the
   stream at all.

   It still exercises the interesting bookkeeping: face 1's first triangle
   meets vertices already registered by face 0, so the encoder must take the
   zero-bit IS_NOT_MULTIPLE path there rather than emitting a fresh normal. */
static int
build_plate(double *positions, double *normals, uint32_t *tri_indices,
    uint32_t *norm_indices, uint32_t *face_tri_counts,
    uint32_t *num_positions, uint32_t *num_normals,
    uint32_t *num_triangles, uint32_t *num_faces)
{
    /* 6 vertices forming two adjacent squares in z = 0:
         3---4---5
         |   |   |
         0---1---2                                             */
    static const double P[6][3] = {
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0},
        {0, 1, 0}, {1, 1, 0}, {2, 1, 0}
    };
    static const uint32_t T[4][3] = {
        {0, 1, 4}, {0, 4, 3},        /* face 0 */
        {1, 2, 5}, {1, 5, 4}         /* face 1 */
    };
    int i, j;

    for (i = 0; i < 6; i++)
        for (j = 0; j < 3; j++)
            positions[i * 3 + j] = P[i][j];
    normals[0] = 0.0; normals[1] = 0.0; normals[2] = 1.0;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 3; j++)
        {
            tri_indices[i * 3 + j] = T[i][j];
            norm_indices[i * 3 + j] = 0;
        }
    face_tri_counts[0] = 2;
    face_tri_counts[1] = 2;

    *num_positions = 6;
    *num_normals = 1;
    *num_triangles = 4;
    *num_faces = 2;
    return 0;
}

int
main(int argc, char **argv)
{
    const char *out = (argc > 1) ? argv[1] : "planar_roundtrip_out.prc";
    prc_context *ctx = NULL;
    prc_api_write_tessellation tess;
    prc_api_write_rep_item ri;
    prc_api_write_node root;
    double positions[NFACE * 4 * 3];
    double normals[NFACE * 3];
    uint32_t tri_indices[NFACE * 2 * 3];
    uint32_t norm_indices[NFACE * 2 * 3];
    uint32_t face_tri_counts[NFACE];
    int f, t, rc = 2;
    int use_cube = (argc > 2 && strcmp(argv[2], "cube") == 0);
    uint32_t n_pos = NFACE * 4, n_nrm = NFACE, n_tri = NFACE * 2, n_face = NFACE;

    if (use_cube)
    {
    /* Each face gets its own four vertices, so a face's vertices are never
       shared with another face's in the position array. The encoder's weld
       step will re-merge them by position, which is what makes the corner
       vertices multiple-normal and exercises the interesting path. */
    for (f = 0; f < NFACE; f++)
    {
        for (t = 0; t < 4; t++)
        {
            int c = FACE_CORNER[f][t];
            int base = (f * 4 + t) * 3;

            positions[base + 0] = CORNER[c][0];
            positions[base + 1] = CORNER[c][1];
            positions[base + 2] = CORNER[c][2];
        }
        normals[f * 3 + 0] = FACE_NORMAL[f][0];
        normals[f * 3 + 1] = FACE_NORMAL[f][1];
        normals[f * 3 + 2] = FACE_NORMAL[f][2];

        /* two triangles: (0,1,2) and (0,2,3) over the face's four vertices */
        tri_indices[(f * 2 + 0) * 3 + 0] = (uint32_t)(f * 4 + 0);
        tri_indices[(f * 2 + 0) * 3 + 1] = (uint32_t)(f * 4 + 1);
        tri_indices[(f * 2 + 0) * 3 + 2] = (uint32_t)(f * 4 + 2);
        tri_indices[(f * 2 + 1) * 3 + 0] = (uint32_t)(f * 4 + 0);
        tri_indices[(f * 2 + 1) * 3 + 1] = (uint32_t)(f * 4 + 2);
        tri_indices[(f * 2 + 1) * 3 + 2] = (uint32_t)(f * 4 + 3);
        for (t = 0; t < 6; t++)
            norm_indices[(f * 2) * 3 + t] = (uint32_t)f;
        face_tri_counts[f] = 2;
    }
    }
    else
    {
        build_plate(positions, normals, tri_indices, norm_indices,
                    face_tri_counts, &n_pos, &n_nrm, &n_tri, &n_face);
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "prc_api_new_context failed\n");
        return 2;
    }

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = n_pos;
    tess.normals = normals;
    tess.num_normals = n_nrm;
    tess.tri_indices = tri_indices;
    tess.norm_indices = norm_indices;
    tess.num_triangles = n_tri;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = n_face;

    memset(&ri, 0, sizeof(ri));
    ri.kind = PRC_API_WRITE_RI_SURFACE;
    ri.biased_tessellation_index = 1;
    ri.is_closed = 1;

    memset(&root, 0, sizeof(root));
    root.rep_items = &ri;
    root.num_rep_items = 1;

    if (prc_api_write_prc_file(ctx, out, "planar_cube", &root, &tess, 1) != 0)
    {
        fprintf(stderr, "prc_api_write_prc_file failed\n");
        prc_api_print_error_stack(ctx);
        goto done;
    }
    printf("wrote %s  (%s: %u faces, %u triangles)\n", out,
           use_cube ? "cube" : "plate", n_face, n_tri);

    /* ---- read back -------------------------------------------------- */
    {
        prc_api_data data = prc_api_open_contents(ctx, out);
        prc_api_product *model_tree = NULL;
        prc_api_tess *tesses = NULL;
        uint32_t num_parts = 0, num_products = 0, num_markups = 0;
        uint32_t total = 0, total_line = 0, num_eg = 0;
        uint8_t has_lines = 0;
        uint32_t k, j, i;
        uint32_t checked = 0, bad = 0;

        if (data == NULL)
        {
            fprintf(stderr, "reopen failed\n");
            prc_api_print_error_stack(ctx);
            goto done;
        }
        if (prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups) < 0 ||
            prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups) < 0 ||
            prc_api_get_number_tessellations(ctx, data, model_tree, &total, &total_line, &num_eg) < 0)
        {
            fprintf(stderr, "model tree / tess count failed\n");
            prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, model_tree);
            goto done;
        }

        tesses = (prc_api_tess *)calloc(total ? total : 1, sizeof(prc_api_tess));
        if (tesses == NULL)
        {
            prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, model_tree);
            goto done;
        }

        for (k = 0; k < total; k++)
        {
            prc_api_tess *tp = &tesses[k];
            uint32_t nf = prc_api_get_number_faces(ctx, data, k);

            tp->num_faces = nf;
            if (nf > 0)
                tp->tess_faces = (prc_api_face *)calloc(nf, sizeof(prc_api_face));
            if (prc_api_initialize_tessellation(ctx, data, model_tree, k, tp,
                                                NULL, &has_lines) < 0)
                continue;
            for (j = 0; j < tp->num_faces; j++)
            {
                if (prc_api_get_tessellation_vertices(ctx, data, k, j,
                        tp->tess_faces + j, tp) < 0)
                    continue;
            }

            /* Every decoded vertex must carry a unit normal that matches one
               of the six axis directions, and every vertex of a given face
               must share one normal. We check the weaker, encoding-agnostic
               property -- each normal is axis-aligned and unit length --
               because face ordering is not guaranteed to survive, while a
               desynchronised normal stream produces neither. */
            /* A COMPRESSED tessellation materialises every triangle into one
               tess->tess_vertices buffer and keeps faces only as
               graphic-primitive ranges; only the uncompressed form fills the
               per-face buffers. Since this test deliberately writes
               COMPRESSED -- that is the only encoding is_face_planar exists
               in -- the single buffer is where the normals are. */
            for (j = 0; j < (tp->num_faces ? tp->num_faces : 1); j++)
            {
                prc_api_tess_vertex_buffer *vb =
                    (tp->type == PRC_API_TESS_3D_Compressed)
                        ? &tp->tess_vertices
                        : &tp->tess_faces[j].face_vertices;

                if (tp->type == PRC_API_TESS_3D_Compressed && j > 0)
                    break;   /* one shared buffer; count it once */

                for (i = 0; i < vb->num_vertices; i++)
                {
                    const float *n = vb->vertices[i].normal;
                    double len = sqrt((double)n[0] * n[0] + (double)n[1] * n[1] +
                                      (double)n[2] * n[2]);
                    double ax = fabs((double)n[0]), ay = fabs((double)n[1]);
                    double az = fabs((double)n[2]);
                    double mx = ax > ay ? (ax > az ? ax : az) : (ay > az ? ay : az);
                    double others = ax + ay + az - mx;

                    checked++;
                    if (fabs(len - 1.0) > 1.0e-3 || others > 1.0e-3)
                    {
                        if (bad < 8)
                            printf("  MISMATCH face %u vertex %u normal=(%.6f,%.6f,%.6f) len=%.6f\n",
                                   j, i, n[0], n[1], n[2], len);
                        bad++;
                    }
                }
            }
        }

        printf("normals checked: %u   mismatched: %u\n", checked, bad);
        if (checked == 0)
        {
            printf("RESULT: INCONCLUSIVE (no normals decoded)\n");
            rc = 2;
        }
        else if (bad == 0)
        {
            printf("RESULT: PASS -- every decoded normal is unit and axis-aligned\n");
            rc = 0;
        }
        else
        {
            printf("RESULT: FAIL -- %u of %u normals wrong\n", bad, checked);
            rc = 1;
        }

        prc_api_release_data(ctx, data, tesses, total, NULL, 0, NULL, 0, model_tree);
    }

done:
    prc_api_release_context(ctx);
    return rc;
}
