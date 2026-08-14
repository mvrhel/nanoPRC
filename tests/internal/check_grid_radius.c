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

   WHAT: For a synthetic grid produced by standalone_grid_triangles.c (a
   cylinder segment, radius R=5.0 -- see that tool's own header comment for
   the exact parameters), every valid vertex must satisfy sqrt(x^2+y^2) == R
   exactly (by construction: x = R*sin(theta), y = R*cos(theta)). Checking
   this analytic invariant directly on DECODED vertices -- rather than
   inferring corruption from relative edge-length statistics -- pinpoints
   exactly which vertex/point indices hold wrong position data, independent
   of topology/traversal-order questions (see detect_tess_spikes.c for the
   edge-length-ratio approach used earlier in this same investigation).

   HOW: usage: check_grid_radius.exe <input.pdf> [radius=5.0] [tol=0.01]
   Decodes tess 0's shared vertex buffer (COMPRESSED case) and reports every
   vertex whose sqrt(x^2+y^2) deviates from radius by more than tol, in
   vertex-buffer order (the order the public API delivers them, i.e. decode/
   traversal order -- NOT necessarily original point_array order), so the
   FIRST reported index is the first point the decoder produces that's off
   the analytic surface. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations;
    uint32_t num_extra_geom_tess;
    uint8_t has_lines = 0;
    int code;
    prc_api_tess tess;
    double radius = 5.0;
    double tol = 0.01;
    uint32_t v, bad_count = 0, first_bad = UINT32_MAX;

    if (argc < 2)
    {
        printf("usage: %s <input.pdf> [radius=5.0] [tol=0.01]\n", argv[0]);
        return 2;
    }
    if (argc >= 3) radius = atof(argv[2]);
    if (argc >= 4) tol = atof(argv[3]);

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

    memset(&tess, 0, sizeof(tess));
    {
        uint32_t nFaces = prc_api_get_number_faces(ctx, data, 0);
        uint32_t j;
        tess.num_faces = nFaces;
        tess.tess_faces = (prc_api_face *)calloc(nFaces ? nFaces : 1, sizeof(prc_api_face));
        code = prc_api_initialize_tessellation(ctx, data, model_tree, 0, &tess, NULL, &has_lines);
        if (code < 0) { printf("initialize_tessellation failed\n"); return 1; }
        for (j = 0; j < tess.num_faces; j++)
        {
            code = prc_api_get_tessellation_vertices(ctx, data, 0, j, tess.tess_faces + j, &tess);
            if (code < 0) { printf("get_tessellation_vertices face %u failed\n", j); return 1; }
        }
    }

    printf("tess_type=%d num_vertices=%zu radius=%.6f tol=%.6f\n",
        tess.type, tess.tess_vertices.num_vertices, radius, tol);

    for (v = 0; v < tess.tess_vertices.num_vertices; v++)
    {
        prc_api_vertex *vt = &tess.tess_vertices.vertices[v];
        double x = vt->position[0], y = vt->position[1], z = vt->position[2];
        double r = sqrt(x * x + y * y);
        double dev = fabs(r - radius);

        if (dev > tol)
        {
            if (first_bad == UINT32_MAX) first_bad = v;
            bad_count++;
            if (bad_count <= 40)
                printf("BAD v=%u pos=(%.6f,%.6f,%.6f) r=%.6f dev=%.6f\n", v, x, y, z, r, dev);
        }
    }

    printf("total_bad=%u first_bad_index=%d (of %zu vertices)\n",
        bad_count, first_bad == UINT32_MAX ? -1 : (int)first_bad, tess.tess_vertices.num_vertices);

    return 0;
}
