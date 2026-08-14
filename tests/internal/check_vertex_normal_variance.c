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

   WHAT: For one PRC_API_TESS_3D_Compressed tessellation, groups the public
   API's (position, normal)-deduplicated vertices by position alone (exact
   match after all producers so far in this investigation have used a
   position array with no incidental precision loss between grouped
   entries), then reports how many distinct normals land at each shared
   position and how far apart they are (max pairwise angle, degrees).

   WHY IT EXISTS: reauthor_tess_pdf.c writing a decoded mesh back out via
   PRC_API_WRITE_TESS_KIND_COMPRESSED, then reading it back, was found to
   produce a public-API vertex count several times larger than the number
   of physically distinct positions -- e.g. 58192 API vertices for 14818
   input positions on tess 901 of 3dpdf-Aero-Turbine.pdf, and a near-
   identical ~4x ratio for the much smaller tess 902 too (see
   ISO-SPEC/outer-engine-cover-investigation.md's write-up for the earlier,
   now-superseded framing of this as a hard write-path failure -- rebuilding
   the stale verify_manifold_pdf.exe binary that originally reported that
   showed the round-trip actually parses back fine now). Since API vertices
   are deduplicated by (position, normal) pair, and the same normal was fed
   in for every corner visiting a given position (reauthor_tess_pdf.c pairs
   normals 1:1 with positions, not per-corner), this tool directly checks
   whether the *decoded* normals at a shared position actually still agree
   -- if they don't, the compressed format's per-corner ("C2") normal
   encoder is reconstructing different quantized normals for different
   traversal visits to the same physical vertex, which would explain the
   inflated vertex count and the resulting edge-adjacency fragmentation
   directly, with no defect in position/topology encoding at all.

   HOW: usage: check_vertex_normal_variance.exe <input.pdf> <tess_index>
   [top_n=20] -- prints the top_n position-groups by member count, each
   with member count and max pairwise angle between their normals. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"

typedef struct
{
    double pos[3];
    double normal[3];
} vtx_rec;

static int
vtx_pos_cmp(const void *a, const void *b)
{
    const vtx_rec *va = (const vtx_rec *)a;
    const vtx_rec *vb = (const vtx_rec *)b;
    int k;
    for (k = 0; k < 3; k++)
    {
        if (va->pos[k] < vb->pos[k]) return -1;
        if (va->pos[k] > vb->pos[k]) return 1;
    }
    return 0;
}

typedef struct
{
    uint32_t start;
    uint32_t count;
    double max_angle_deg;
} group_rec;

static int
group_cmp(const void *a, const void *b)
{
    const group_rec *ga = (const group_rec *)a;
    const group_rec *gb = (const group_rec *)b;
    if (ga->count > gb->count) return -1;
    if (ga->count < gb->count) return 1;
    return 0;
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations, j;
    uint32_t num_extra_geom_tess;
    uint8_t has_lines = 0;
    int code;
    uint32_t target_index;
    uint32_t top_n = 20;
    prc_api_tess tess;
    uint32_t nFaces;
    vtx_rec *recs = NULL;
    uint32_t n, num_groups = 0;
    group_rec *groups = NULL;

    if (argc < 3) { printf("usage: %s <input.pdf> <tess_index> [top_n=20]\n", argv[0]); return 2; }
    target_index = (uint32_t)atoi(argv[2]);
    if (argc >= 4) top_n = (uint32_t)atoi(argv[3]);

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
    if (target_index >= totalTesselations) { printf("tess_index %u out of range (total %u)\n", target_index, totalTesselations); return 1; }

    memset(&tess, 0, sizeof(tess));
    nFaces = prc_api_get_number_faces(ctx, data, target_index);
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
        prc_api_get_tessellation_vertices(ctx, data, target_index, j, tess.tess_faces + j, &tess);

    if (tess.type == PRC_API_TESS_3D_Compressed)
    {
        printf("tess=%u type=COMPRESSED num_api_vertices=%zu\n", target_index, tess.tess_vertices.num_vertices);

        recs = (vtx_rec *)malloc(sizeof(vtx_rec) * tess.tess_vertices.num_vertices);
        for (n = 0; n < tess.tess_vertices.num_vertices; n++)
        {
            recs[n].pos[0] = tess.tess_vertices.vertices[n].position[0];
            recs[n].pos[1] = tess.tess_vertices.vertices[n].position[1];
            recs[n].pos[2] = tess.tess_vertices.vertices[n].position[2];
            recs[n].normal[0] = tess.tess_vertices.vertices[n].normal[0];
            recs[n].normal[1] = tess.tess_vertices.vertices[n].normal[1];
            recs[n].normal[2] = tess.tess_vertices.vertices[n].normal[2];
        }
    }
    else
    {
        /* TRIANGLES type: vertices live per-face in tess.tess_faces[f].face_vertices,
           not in tess.tess_vertices -- concatenate exactly as reauthor_tess_pdf.c
           and detect_tess_spikes.c do for this type. */
        uint32_t f, running = 0;
        n = 0;
        for (f = 0; f < tess.num_faces; f++)
            n += (uint32_t)tess.tess_faces[f].face_vertices.num_vertices;
        printf("tess=%u type=TRIANGLES num_api_vertices=%u (concatenated across %zu faces)\n", target_index, n, tess.num_faces);

        recs = (vtx_rec *)malloc(sizeof(vtx_rec) * (n ? n : 1));
        for (f = 0; f < tess.num_faces; f++)
        {
            prc_api_tess_vertex_buffer *fvb = &tess.tess_faces[f].face_vertices;
            uint32_t v;
            for (v = 0; v < (uint32_t)fvb->num_vertices; v++)
            {
                recs[running].pos[0] = fvb->vertices[v].position[0];
                recs[running].pos[1] = fvb->vertices[v].position[1];
                recs[running].pos[2] = fvb->vertices[v].position[2];
                recs[running].normal[0] = fvb->vertices[v].normal[0];
                recs[running].normal[1] = fvb->vertices[v].normal[1];
                recs[running].normal[2] = fvb->vertices[v].normal[2];
                running++;
            }
        }
        tess.tess_vertices.num_vertices = n;
    }
    qsort(recs, tess.tess_vertices.num_vertices, sizeof(vtx_rec), vtx_pos_cmp);

    groups = (group_rec *)malloc(sizeof(group_rec) * (tess.tess_vertices.num_vertices ? tess.tess_vertices.num_vertices : 1));
    {
        uint32_t run_start = 0;
        for (n = 1; n <= tess.tess_vertices.num_vertices; n++)
        {
            if (n == tess.tess_vertices.num_vertices || vtx_pos_cmp(&recs[n], &recs[run_start]) != 0)
            {
                uint32_t g = num_groups++;
                uint32_t a, b;
                double max_angle = 0.0;
                groups[g].start = run_start;
                groups[g].count = n - run_start;
                for (a = run_start; a < n; a++)
                {
                    for (b = a + 1; b < n; b++)
                    {
                        double dot = recs[a].normal[0]*recs[b].normal[0] +
                                     recs[a].normal[1]*recs[b].normal[1] +
                                     recs[a].normal[2]*recs[b].normal[2];
                        double ang;
                        if (dot > 1.0) dot = 1.0;
                        if (dot < -1.0) dot = -1.0;
                        ang = acos(dot) * 180.0 / 3.14159265358979323846;
                        if (ang > max_angle) max_angle = ang;
                    }
                }
                groups[g].max_angle_deg = max_angle;
                run_start = n;
            }
        }
    }

    qsort(groups, num_groups, sizeof(group_rec), group_cmp);

    printf("distinct_positions=%u (vs %zu api vertices, ratio=%.2f)\n",
        num_groups, tess.tess_vertices.num_vertices,
        num_groups ? (double)tess.tess_vertices.num_vertices / (double)num_groups : 0.0);
    printf("top %u position-groups by member count:\n", top_n);
    for (n = 0; n < num_groups && n < top_n; n++)
    {
        const vtx_rec *first = &recs[groups[n].start];
        printf("  count=%u max_angle_deg=%.4f pos=(%.6f, %.6f, %.6f)\n",
            groups[n].count, groups[n].max_angle_deg, first->pos[0], first->pos[1], first->pos[2]);
    }

    {
        uint32_t single = 0, multi_agree = 0, multi_disagree = 0;
        for (n = 0; n < num_groups; n++)
        {
            if (groups[n].count == 1) single++;
            else if (groups[n].max_angle_deg < 0.01) multi_agree++;
            else multi_disagree++;
        }
        printf("summary: single-normal positions=%u  multi-normal-but-agreeing(<0.01deg)=%u  multi-normal-disagreeing=%u\n",
            single, multi_agree, multi_disagree);
    }

    free(recs);
    free(groups);
    free(tess.tess_faces);
    prc_api_release_context(ctx);
    return 0;
}
