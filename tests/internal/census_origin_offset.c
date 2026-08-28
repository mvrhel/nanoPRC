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

   WHAT: For every COMPRESSED tessellation entity in a file, compares the
   declared origin_array against the centre of the bounding box of that
   entity's own decoded vertices, and reports the offset both absolutely
   and as a fraction of the bounding-box diagonal.

   WHY: ISO/CD 14739-1.4:2026 Table 175 describes origin_array as "the
   bounding box center of the compressed 3D tessellation data". If that is
   literally true and scoped to the entity, it is an in-file self-check
   that needs no reference decoder: reconstruct, take the bounding box,
   compare its centre to the declared origin. A decoder that reconstructs
   an entity correctly should land near zero.

   Measured across a 310-file third-party corpus (2026-08-27): 34,402
   compressed entities from 145 files, median offset 4.2e-06 of the
   diagonal, 82.1% within 0.1%, 94.0% within 10%. The tail is concentrated
   rather than diffuse -- 81 of 143 files have no entity beyond 10% while
   6 files have every entity beyond it -- which reads as specific writers
   not following the convention rather than the property failing generally.

   HOW: usage: census_origin_offset <input.prc|input.pdf>
   Opens and initializes every tessellation through the public API (that is
   what runs the compressed decode and populates
   vertices_prc_compressed_3d), then walks the internal file structures to
   read origin_array and that same entity's decoded vertices, so there is
   no index-pairing question between the two.

   Prints one CSV row per compressed entity:
     <file>,<file_structure>,<entity>,<nverts>,<offset>,<diagonal>,
     <offset/diagonal>
   A ratio of -1 means the entity's bounding box has zero extent (all
   vertices coincident), so the ratio is undefined; those rows should be
   filtered out before computing statistics.

   KNOWN LIMITATION, IMPORTANT: this ratio is NOT monotonic in decode
   correctness, so do not use it as a scalar score for "how right is my
   reconstruction". A correspondent working the same question measured a
   single-apex frame fix that made an entity strictly more correct (68
   more exactly-reconstructed vertices) while this ratio moved the WRONG
   way, 0.138 -> 0.201. Use first-divergence index against a reference
   decode as the reliable signal and treat this ratio as a detector for
   gross error only. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data_in;
    prc_data *data;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t total_tess = 0, total_line_tess = 0, num_extra_geom_tess = 0;
    uint32_t k, j, fs;

    if (argc < 2)
    {
        printf("usage: census_origin_offset <input.prc|input.pdf>\n");
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
        return 1;

    data_in = prc_api_open_contents(ctx, argv[1]);
    if (data_in == NULL)
    {
        fprintf(stderr, "OPENFAIL %s\n", argv[1]);
        return 1;
    }
    if (prc_api_prep_model_tree(ctx, data_in, &num_parts, &num_products,
                                &num_markups) != 0)
    {
        fprintf(stderr, "PREPFAIL %s\n", argv[1]);
        return 1;
    }
    if (prc_api_create_model_tree(ctx, data_in, &model_tree, num_parts, num_products,
                                  num_markups) != 0)
    {
        fprintf(stderr, "TREEFAIL %s\n", argv[1]);
        return 1;
    }
    if (prc_api_get_number_tessellations(ctx, data_in, model_tree, &total_tess,
                                         &total_line_tess, &num_extra_geom_tess) != 0)
    {
        fprintf(stderr, "COUNTFAIL %s\n", argv[1]);
        return 1;
    }

    /* Ownership mirrors demos/quick_start: prc_api_release_data below frees
       this array's entries, including each tess_faces. */
    if (total_tess > 0)
    {
        tesses = (prc_api_tess *)calloc(total_tess, sizeof(prc_api_tess));
        if (tesses == NULL)
        {
            fprintf(stderr, "ALLOCFAIL %s\n", argv[1]);
            return 1;
        }
    }

    /* Force the compressed decode to run for every entity. */
    for (k = 0; k < total_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t num_faces = prc_api_get_number_faces(ctx, data_in, k);
        uint8_t has_lines = 0;

        tess->num_faces = num_faces;
        tess->tess_faces = (prc_api_face *)calloc(num_faces ? num_faces : 1,
                                                  sizeof(prc_api_face));
        if (tess->tess_faces == NULL)
            break;
        if (prc_api_initialize_tessellation(ctx, data_in, model_tree, k, tess, NULL,
                                            &has_lines) == 0)
        {
            for (j = 0; j < tess->num_faces; j++)
                prc_api_get_tessellation_vertices(ctx, data_in, k, j,
                                                  tess->tess_faces + j, tess);
        }
    }

    /* Now read origin_array and the decoded vertices from the same entity. */
    data = (prc_data *)data_in;
    for (fs = 0; fs < data->file_structure_count; fs++)
    {
        prc_asm_file_structure_tessellation *container =
            data->file_struct[fs].tessellation;
        uint32_t t;

        if (container == NULL || container->tess == NULL)
            continue;

        for (t = 0; t < container->tess_count; t++)
        {
            prc_tess *entry = &container->tess[t];
            prc_tess_3d_compressed *c;
            double mn[3], mx[3], centre[3], d[3], off, diag;
            int n, i, a;

            if (entry->tess_type != PRC_TYPE_TESS_3D_Compressed)
                continue;
            c = entry->tess_3d_compressed;
            if (c == NULL || c->vertices_prc_compressed_3d == NULL)
                continue;
            n = c->num_vertices_prc_compressed_3d;
            if (n <= 0)
                continue;

            for (a = 0; a < 3; a++)
            {
                mn[a] = c->vertices_prc_compressed_3d[a];
                mx[a] = mn[a];
            }
            for (i = 1; i < n; i++)
            {
                for (a = 0; a < 3; a++)
                {
                    double v = c->vertices_prc_compressed_3d[i * 3 + a];
                    if (v < mn[a])
                        mn[a] = v;
                    if (v > mx[a])
                        mx[a] = v;
                }
            }

            for (a = 0; a < 3; a++)
            {
                centre[a] = (mn[a] + mx[a]) * 0.5;
                d[a] = mx[a] - mn[a];
            }
            diag = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

            d[0] = c->origin_array.x - centre[0];
            d[1] = c->origin_array.y - centre[1];
            d[2] = c->origin_array.z - centre[2];
            off = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

            printf("%s,%u,%u,%d,%.10g,%.10g,%.10g\n", argv[1], fs, t, n, off, diag,
                   diag > 0.0 ? off / diag : -1.0);
        }
    }
    fflush(stdout);

    prc_api_release_data(ctx, data_in, tesses, total_tess, NULL, 0, NULL, 0, model_tree);
    free(tesses);
    prc_api_release_context(ctx);
    return 0;
}
