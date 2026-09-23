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

/* A COMPRESSED tessellation must publish its normals through normal_set, not
   just through normal[].

   prc_api_vertex documents normal_set as "0 = not set", so a consumer is
   entitled to ignore normal[] when the flag is clear. The compressed path
   decoded correct normals into normal[] but never set the flag, because the
   internal position_normal_pair tracker carries a field of the same name and
   setting that one looked like the job was done. Measured over 310 prc-db
   files before the fix: 0 of 55,533,473 compressed vertices had the flag, and
   every one of them had a non-zero normal. Every consumer honouring the
   contract -- obj_export, stl_export, json_export -- therefore fell back to a
   computed flat face normal, and shaded every compressed model flat.

   examples/cube.pdf carries a compressed tessellation, so this runs against a
   file already in the tree. Checking the flag alone would pass on a build that
   set it unconditionally, so the normal itself is required to be unit length,
   and the count of vertices actually inspected is asserted to be non-zero --
   a walk that silently visits nothing would otherwise pass. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_test.h"
#include "prc_api.h"
#include "prc_context.h"

int
main(void)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *tree = NULL;
    prc_api_tess *tesses;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t total = 0, num_lines = 0, num_extra = 0;
    uint32_t k;
    uint8_t has_lines = 0;
    unsigned long compressed_tess = 0, checked = 0;

    PRC_TEST_BEGIN("test_compressed_normals");

    ctx = prc_api_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    data = prc_api_open_contents(ctx, EXAMPLE_CUBE_PDF);
    PRC_ASSERT_NOT_NULL(data);

    PRC_ASSERT(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups) >= 0);
    PRC_ASSERT(prc_api_create_model_tree(ctx, data, &tree, num_parts,
        num_products, num_markups) >= 0);
    PRC_ASSERT(prc_api_get_number_tessellations(ctx, data, tree, &total,
        &num_lines, &num_extra) >= 0);
    PRC_ASSERT(total > 0);

    tesses = (prc_api_tess *)calloc(total, sizeof(prc_api_tess));
    PRC_ASSERT_NOT_NULL(tesses);

    for (k = 0; k < total; k++)
    {
        prc_api_tess *tess = &tesses[k];
        prc_api_face *face;
        const prc_api_tess_vertex_buffer *vertex_buffer;
        uint32_t num_faces = prc_api_get_number_faces(ctx, data, k);
        size_t v;

        tess->num_faces = num_faces;
        tess->tess_faces = (prc_api_face *)calloc(num_faces ? num_faces : 1,
            sizeof(prc_api_face));
        PRC_ASSERT_NOT_NULL(tess->tess_faces);

        if (prc_api_initialize_tessellation(ctx, data, tree, k, tess, NULL,
                &has_lines) < 0)
            continue;
        if (tess->type != PRC_API_TESS_3D_Compressed)
            continue;

        /* A compressed tessellation does not partition geometry by face: every
           face index returns the same complete mesh, on the tessellation's own
           vertex buffer rather than the face's. */
        compressed_tess++;
        face = tess->tess_faces;
        if (prc_api_get_tessellation_vertices(ctx, data, k, 0, face, tess) < 0)
            continue;
        vertex_buffer = &tess->tess_vertices;
        if (vertex_buffer->vertices == NULL)
            continue;

        for (v = 0; v < vertex_buffer->num_vertices; v++)
        {
            const prc_api_vertex *vertex = &vertex_buffer->vertices[v];
            double length;

            PRC_ASSERT(vertex->normal_set == 1);

            length = sqrt((double)vertex->normal[0] * vertex->normal[0] +
                          (double)vertex->normal[1] * vertex->normal[1] +
                          (double)vertex->normal[2] * vertex->normal[2]);
            PRC_ASSERT_NEAR(length, 1.0, 1e-3);
            checked++;
        }
    }

    /* Without these the test would pass on a build that produced no compressed
       tessellation, or no vertices, at all. */
    PRC_ASSERT(compressed_tess > 0);
    PRC_ASSERT(checked > 0);

    prc_api_release_data(ctx, data, tesses, total, NULL, 0, NULL, 0, tree);
    PRC_ASSERT(prc_api_release_context(ctx) == 0);

    PRC_TEST_END;
}
