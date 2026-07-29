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

/* Black-box regression test for demos/stl_import: exports examples/cube.pdf
   to a binary STL via the existing nano_prc_stl_export demo, re-imports
   that STL via nano_prc_stl_import, and confirms the round trip preserved
   the geometry -- exact triangle count (STL is a fixed triangle-soup
   format with no merging on either side of this round trip, so this must
   match exactly) and a tolerant bounding-box comparison (loose enough to
   absorb stl_export's float64->float32 truncation -- binary STL only
   stores float32 vertices -- but tight enough to catch a real regression).

   Deliberately reuses the already-built nano_prc_stl_export/nano_prc_
   stl_import executables (via the STL_EXPORT_EXE/STL_IMPORT_EXE paths
   tests/functional/CMakeLists.txt supplies as compile definitions, using
   $<TARGET_FILE:...> generator expressions) rather than reimplementing
   STL reading/writing here -- this test is checking the two demos agree
   with each other end to end, not re-deriving their logic. */

#include <prc_api.h>
#include "prc_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(_WIN32)
#define CMD_SUCCEEDED(rc) ((rc) == 0)
#else
#include <sys/wait.h>
#define CMD_SUCCEEDED(rc) ((rc) != -1 && WIFEXITED(rc) && WEXITSTATUS(rc) == 0)
#endif

typedef struct
{
    uint32_t triangle_count;
    double bbox_min[3];
    double bbox_max[3];
} roundtrip_stats;

static void
stats_reset(roundtrip_stats *s)
{
    int i;
    s->triangle_count = 0;
    for (i = 0; i < 3; i++) { s->bbox_min[i] = 1e300; s->bbox_max[i] = -1e300; }
}

static void
stats_expand(roundtrip_stats *s, const float p[3])
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if ((double)p[i] < s->bbox_min[i]) s->bbox_min[i] = (double)p[i];
        if ((double)p[i] > s->bbox_max[i]) s->bbox_max[i] = (double)p[i];
    }
}

/* Counts triangles and computes a bounding box for `path` (a .prc or .pdf
   file). Uses the exact same public-API tessellation-walking sequence
   demos/stl_export/src/stl_export.c uses to enumerate triangles
   (prc_api_initialize_tessellation -> per-face prc_api_get_tessellation_
   vertices -> per-face prc_api_get_graphics_primitive, dispatching on
   TRIANGLES/STRIP/FAN) -- copied rather than reimplemented from scratch,
   so this test's own notion of "how many triangles are in this file"
   can't silently drift from what the exporter it's testing against
   already does. Returns 0 on success. */
static int
count_triangles_and_bbox(prc_context *ctx, const char *path, roundtrip_stats *out)
{
    prc_api_data data = NULL;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t total_tess = 0, total_line_tess = 0;
    uint32_t k;
    int ok = -1;

    stats_reset(out);

    data = prc_api_open_contents(ctx, path);
    if (data == NULL)
    {
        fprintf(stderr, "count_triangles_and_bbox: prc_api_open_contents failed for %s\n", path);
        return -1;
    }

    if (prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups) != 0)
        goto cleanup;
    if (prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups) != 0)
        goto cleanup;
    if (prc_api_get_number_tessellations(ctx, data, model_tree, &total_tess, &total_line_tess) != 0)
        goto cleanup;

    if (total_tess > 0)
    {
        tesses = (prc_api_tess *)calloc(total_tess, sizeof(prc_api_tess));
        if (tesses == NULL)
            goto cleanup;
    }

    for (k = 0; k < total_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t num_faces = prc_api_get_number_faces(ctx, data, k);
        uint8_t has_lines = 0;
        uint32_t j;

        tess->num_faces = num_faces;
        tess->tess_faces = (prc_api_face *)calloc(num_faces, sizeof(prc_api_face));
        if (tess->tess_faces == NULL)
            goto cleanup;

        if (prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL, &has_lines) != 0)
            goto cleanup;
        if (tess->type == PRC_API_TESS_UNKNOWN || tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
            continue;

        for (j = 0; j < tess->num_faces; j++)
        {
            if (prc_api_get_tessellation_vertices(ctx, data, model_tree, k, j, tess->tess_faces + j, tess) != 0)
                goto cleanup;
        }
    }

    for (k = 0; k < total_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        size_t f;

        if (tess->type != PRC_API_TESS_3D && tess->type != PRC_API_TESS_3D_Compressed)
            continue;

        for (f = 0; f < tess->num_faces; f++)
        {
            prc_api_face *face = &tess->tess_faces[f];
            prc_api_tess_vertex_buffer *vertex_buf =
                (tess->type == PRC_API_TESS_3D_Compressed) ? &tess->tess_vertices : &face->face_vertices;
            size_t p;

            if (vertex_buf == NULL || vertex_buf->vertices == NULL || vertex_buf->num_vertices == 0)
                continue;

            for (p = 0; p < face->num_graphic_primitives; p++)
            {
                prc_api_graphic_primitive prim;
                if (prc_api_get_graphics_primitive(ctx, data, tess, (uint32_t)f, p, &prim) < 0)
                    continue;

                if (prim.type == PRC_API_TRIANGLES)
                {
                    size_t idx;
                    for (idx = 0; idx + 2 < prim.num_indices; idx += 3)
                    {
                        uint32_t i0 = prim.indices[idx], i1 = prim.indices[idx + 1], i2 = prim.indices[idx + 2];
                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            stats_expand(out, vertex_buf->vertices[i0].position);
                            stats_expand(out, vertex_buf->vertices[i1].position);
                            stats_expand(out, vertex_buf->vertices[i2].position);
                            out->triangle_count++;
                        }
                    }
                }
                else if (prim.type == PRC_API_STRIP)
                {
                    size_t m;
                    for (m = 2; m < prim.num_indices; m++)
                    {
                        uint32_t i0, i1, i2;
                        if (m % 2 == 0) { i0 = prim.indices[m - 2]; i1 = prim.indices[m - 1]; i2 = prim.indices[m]; }
                        else { i0 = prim.indices[m - 1]; i1 = prim.indices[m - 2]; i2 = prim.indices[m]; }
                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            stats_expand(out, vertex_buf->vertices[i0].position);
                            stats_expand(out, vertex_buf->vertices[i1].position);
                            stats_expand(out, vertex_buf->vertices[i2].position);
                            out->triangle_count++;
                        }
                    }
                }
                else if (prim.type == PRC_API_FAN)
                {
                    size_t m;
                    for (m = 2; m < prim.num_indices; m++)
                    {
                        uint32_t i0 = prim.indices[0], i1 = prim.indices[m - 1], i2 = prim.indices[m];
                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            stats_expand(out, vertex_buf->vertices[i0].position);
                            stats_expand(out, vertex_buf->vertices[i1].position);
                            stats_expand(out, vertex_buf->vertices[i2].position);
                            out->triangle_count++;
                        }
                    }
                }
            }
        }
    }

    ok = 0;

cleanup:
    if (data != NULL)
        prc_api_release_data(ctx, data, tesses, total_tess, NULL, 0, model_tree);
    return ok;
}

int
main(void)
{
    char cmd[2048];
    char stl_path[512], prc_path[512], pdf_path[512];
    prc_context *ctx;
    roundtrip_stats orig, roundtrip;
    double eps, diag;
    double ext[3];
    int i, rc;

    PRC_TEST_BEGIN("test_stl_roundtrip");

    snprintf(stl_path, sizeof(stl_path), "%s/cube_roundtrip.stl", ROUNDTRIP_TMP_DIR);
    snprintf(prc_path, sizeof(prc_path), "%s/cube_roundtrip.prc", ROUNDTRIP_TMP_DIR);
    snprintf(pdf_path, sizeof(pdf_path), "%s/cube_roundtrip.pdf", ROUNDTRIP_TMP_DIR);

    /* system() on Windows runs the command through cmd.exe /c, which has a
       well-known quoting quirk: a command line whose first token is a
       quoted path, followed by further quoted arguments, is misparsed
       ("The filename, directory name, or volume label syntax is
       incorrect.") unless the WHOLE command line is wrapped in one more,
       outer pair of quotes -- confirmed directly against this project's
       own cmd.exe. POSIX system() has no such quirk and the extra quotes
       are harmless there (they'd just need to be literal characters no
       argument itself starts/ends with, which none of these paths do). */
#if defined(_WIN32)
#define CMD_QUOTE "\""
#else
#define CMD_QUOTE ""
#endif

    snprintf(cmd, sizeof(cmd), CMD_QUOTE "\"%s\" \"%s\" \"%s\"" CMD_QUOTE, STL_EXPORT_EXE, EXAMPLE_CUBE_PDF, stl_path);
    rc = system(cmd);
    PRC_ASSERT(CMD_SUCCEEDED(rc));

    snprintf(cmd, sizeof(cmd), CMD_QUOTE "\"%s\" \"%s\" \"%s\" \"%s\"" CMD_QUOTE, STL_IMPORT_EXE, stl_path, prc_path, pdf_path);
    rc = system(cmd);
    PRC_ASSERT(CMD_SUCCEEDED(rc));

    ctx = prc_api_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    PRC_ASSERT_EQ(count_triangles_and_bbox(ctx, EXAMPLE_CUBE_PDF, &orig), 0);
    PRC_ASSERT_EQ(count_triangles_and_bbox(ctx, prc_path, &roundtrip), 0);

    /* Exact match expected: nano_prc_stl_export writes one binary-STL
       facet per PRC triangle (degenerate ones filtered, per its own
       header comment), STL's binary header stores the triangle count
       unambiguously, and nano_prc_stl_import writes one PRC face per
       input STL facet -- no step in this chain merges or drops a
       well-formed triangle. */
    PRC_ASSERT_EQ(orig.triangle_count, roundtrip.triangle_count);
    PRC_ASSERT(orig.triangle_count > 0);

    /* Loose bounding-box tolerance: binary STL only stores float32
       vertices, so stl_export's float64->float32 truncation is expected
       to move each coordinate by up to about a float32 ULP relative to
       the model's own scale -- 1e-3 of the bounding-box diagonal is
       generous enough to absorb that while still catching a real
       regression (e.g. a units/axis mixup, or welding collapsing
       geometry incorrectly). */
    for (i = 0; i < 3; i++)
        ext[i] = orig.bbox_max[i] - orig.bbox_min[i];
    diag = sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]);
    eps = diag * 1e-3;
    if (eps < 1e-6) eps = 1e-6;

    for (i = 0; i < 3; i++)
    {
        PRC_ASSERT_NEAR(orig.bbox_min[i], roundtrip.bbox_min[i], eps);
        PRC_ASSERT_NEAR(orig.bbox_max[i], roundtrip.bbox_max[i], eps);
    }

    prc_api_release_context(ctx);

    PRC_TEST_END;
}
