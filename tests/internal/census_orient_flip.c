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

   WHAT: Reports how often the conditional orientation flip in
   prc_compute_triangle_basis actually fires while decoding a file, and how
   often the index-canonicalizing swap in prc_set_left_right_edge_indices
   does, by walking every tessellation through the public API and reading
   the two pairs of debug counters the decoder already maintains
   (prc_debug_orient_flip_*, prc_debug_index_swap_*, declared in
   src/prc_decode_compressed_tess.c).

   WHY: ISO/CD 14739-1.4:2026 7.8.9.3 defines the apex frame via Formulas
   (1)-(3) and has no counterpart to that flip; nanoPRC's decoder both
   negates Formula (1)'s X axis and applies the flip. The two are not
   independent. Given the negated X, the predicate dot(Y, w) > 0 is
   algebraically true for every non-degenerate configuration -- write
   Z_temp = a*X + b*P with P a unit vector orthogonal to the unit X, then
   Z = sign(b)*(P ^ X), Y = Z ^ X = -sign(b)*P, and with w = O - V3 =
   -Z_temp, dot(Y, w) = |b| > 0. The composed frame is therefore the
   spec's (X, Y, Z) with X and Y negated: a 180-degree rotation about Z,
   applied unconditionally.

   This tool measures that on real files. A near-100% firing rate is the
   expected result and confirms the flip is not a discriminator. The
   residual is deducible rather than mysterious: the proof above assumes
   Y = Z ^ X with Z proportional to Z_temp ^ X, so any non-firing triangle
   must be one where that construction did not hold -- precisely the
   null-axis case 7.8.9.3 hands to MakeOrthoRep (plus exact-zero dots).
   See tests/internal/check_apex_frame.py for the numerical companion.

   Measured across a 310-file third-party corpus (2026-08-27): 31,314,935
   of 31,329,211 apex-frame computations took the flip (99.9544%), per-file
   range 99.29%-100.00%, 60 of 129 files at exactly 100%, none below 99%.

   HOW: usage: census_orient_flip <input.prc|input.pdf>
   Prints one CSV row:
     <file>,<flip_fired>,<flip_total>,<swap_fired>,<swap_total>,
     <compressed_entities>,<total_tessellations>

   KNOWN LIMITATIONS: the counters are process-global and cumulative, so
   this deliberately measures one file per invocation -- drive a corpus
   from a shell loop, not by passing many files. The counter denominator
   is calls to prc_compute_triangle_basis (roughly two per triangle, one
   per candidate edge), NOT a triangle count; do not report it as one. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_api.h"

/* Maintained in src/prc_decode_compressed_tess.c. Internal diagnostic
   globals, reached by extern exactly as verify_manifold_pdf.c does. */
extern long prc_debug_orient_flip_triggered_count;
extern long prc_debug_orient_flip_total_count;
extern long prc_debug_index_swap_triggered_count;
extern long prc_debug_index_swap_total_count;

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t total_tess = 0, total_line_tess = 0, num_extra_geom_tess = 0;
    uint32_t k, j;
    long compressed_entities = 0;
    prc_api_tess *tesses = NULL;

    if (argc < 2)
    {
        printf("usage: census_orient_flip <input.prc|input.pdf>\n");
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
        return 1;

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL)
    {
        fprintf(stderr, "OPENFAIL %s\n", argv[1]);
        return 1;
    }

    if (prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups) != 0)
    {
        fprintf(stderr, "PREPFAIL %s\n", argv[1]);
        return 1;
    }
    if (prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products,
                                  num_markups) != 0)
    {
        fprintf(stderr, "TREEFAIL %s\n", argv[1]);
        return 1;
    }
    if (prc_api_get_number_tessellations(ctx, data, model_tree, &total_tess,
                                         &total_line_tess, &num_extra_geom_tess) != 0)
    {
        fprintf(stderr, "COUNTFAIL %s\n", argv[1]);
        return 1;
    }

    /* Ownership mirrors demos/quick_start: this array and each entry's
       tess_faces are handed to prc_api_release_data at the end, which frees
       them -- do not free tess_faces here. */
    if (total_tess > 0)
    {
        tesses = (prc_api_tess *)calloc(total_tess, sizeof(prc_api_tess));
        if (tesses == NULL)
        {
            fprintf(stderr, "ALLOCFAIL %s\n", argv[1]);
            return 1;
        }
    }

    for (k = 0; k < total_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t num_faces;
        uint8_t has_lines = 0;

        num_faces = prc_api_get_number_faces(ctx, data, k);
        tess->num_faces = num_faces;
        tess->tess_faces = (prc_api_face *)calloc(num_faces ? num_faces : 1,
                                                  sizeof(prc_api_face));
        if (tess->tess_faces == NULL)
            break;

        /* This is the call that runs the compressed decode, and therefore
           the call that moves the counters. */
        if (prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL,
                                            &has_lines) == 0)
        {
            if (tess->type == PRC_API_TESS_3D_Compressed)
                compressed_entities++;
            for (j = 0; j < tess->num_faces; j++)
                prc_api_get_tessellation_vertices(ctx, data, k, j, tess->tess_faces + j,
                                                  tess);
        }
    }

    printf("%s,%ld,%ld,%ld,%ld,%ld,%u\n", argv[1],
           prc_debug_orient_flip_triggered_count,
           prc_debug_orient_flip_total_count,
           prc_debug_index_swap_triggered_count,
           prc_debug_index_swap_total_count,
           compressed_entities,
           total_tess);
    fflush(stdout);

    prc_api_release_data(ctx, data, tesses, total_tess, NULL, 0, NULL, 0, model_tree);
    free(tesses);
    prc_api_release_context(ctx);
    return 0;
}
