/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Finds "spike" triangles -- decoded triangles whose apex lands in a
   geometrically wrong position but still satisfies ordinary closed-2-
   manifold topology (each edge shared by exactly two triangles), so
   verify_manifold_real.c/verify_manifold_pdf.c's edge-adjacency check
   cannot see them. This is the failure mode the oracle document
   ("PRC-compressed-tessellation-spec-additions-with-oracles.pdf") calls
   the gate-direction/gate-order ambiguity ("Open Item 1", the residual,
   unresolved defect on non-revolved geometry): a triangle can be built
   from three valid, correctly-shared edges and still be wrong if one of
   its vertices was placed using the wrong apex-reconstruction frame or
   consumed the wrong coordinate/reference entry during traversal.

   HOW: usage: detect_tess_spikes.exe <input.pdf> [local_ratio_threshold=6.0]
   [top_n=50] [global_ratio_threshold=15.0]. For every
   PRC_API_TESS_3D_Compressed/PRC_API_TESS_3D tessellation (same public-API
   triangle expansion as verify_manifold_pdf.c: TRIANGLES/FAN/STRIP
   primitives from every graphic primitive of every face), builds face-
   adjacency (which triangles share each edge) via the same sorted-edge-run
   technique, then for every triangle computes its own longest edge length
   against TWO different reference scales:
     - LOCAL: the median longest-edge-length of its face-adjacent (1-ring)
       neighbors. Flags an isolated triangle whose own scale diverges
       sharply from its immediate surroundings, while leaving legitimately
       coarse regions (large neighbors too) alone.
     - GLOBAL: the median longest-edge-length across the WHOLE
       tessellation. Catches a defect the local check is blind to: a
       contiguous run/cluster of triangles that are all wrong TOGETHER
       (e.g. many triangles that all incorrectly resolved a reference to
       the same wrong vertex, fanning out from one point) are locally
       self-consistent -- their immediate neighbors are equally bad, so the
       local ratio never trips -- but stand out clearly against the
       tessellation's overall scale. This is the check that actually found
       the turbine's outer-shell defect; the local-only version of this
       tool found nothing on that file.
   A triangle is flagged if EITHER ratio exceeds its threshold. Reports the
   top N candidates globally (by whichever ratio is larger, descending)
   with tessellation index, triangle-local index, vertex positions, and
   both ratios/medians, so a flagged tessellation index can be cross-
   checked against what's visibly wrong in the viewer.

   WHY IT EXISTS: Built after ruling out a suspected memory-safety bug
   (prc_api_get_num_graphics_primitives undercounting compressed-
   tessellation faces vs demos/viewer/src/product.cpp's per-face mesh
   array) as the explanation for stray triangle connectivity seen
   rendering a real 973-part turbine assembly -- demos/viewer/src/
   scene.cpp already forces num_faces=1 for compressed tessellations
   before that code runs, so the array-size mismatch, while real in the
   abstract public-API contract, is unreachable from the viewer's actual
   call pattern. This tool tests the next-most-likely explanation: a
   genuine decode-level defect invisible to topology-only checks.

   LIMITATIONS: Heuristic, not a proof -- a legitimately thin/needle
   triangle in real CAD geometry (a filleted edge, a small chamfer) could
   in principle trigger a false positive; inspect the reported vertex
   positions against the mesh's actual known geometry before concluding a
   flagged triangle is a genuine decode defect. Neighbor lists are capped
   at 8 entries per triangle (plenty for the near-manifold data this
   codebase produces; a highly non-manifold triangle would just have its
   local median computed from a subset of its true neighbors). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"

#define MAX_NEIGHBORS 8

typedef struct { uint32_t a, b, tri; } edge_rec;

typedef struct
{
    uint32_t tess_index;
    uint32_t tri_index;
    double ratio;
    double own_max_edge;
    double local_median;
    double global_median;
    double global_ratio;
    int flagged_by_global;
    float v0[3], v1[3], v2[3];
} spike_candidate;

static int
edge_rec_cmp(const void *pa, const void *pb)
{
    const edge_rec *a = (const edge_rec *)pa;
    const edge_rec *b = (const edge_rec *)pb;
    if (a->a != b->a) return (a->a < b->a) ? -1 : 1;
    if (a->b != b->b) return (a->b < b->b) ? -1 : 1;
    return (a->tri < b->tri) ? -1 : (a->tri > b->tri ? 1 : 0);
}

static int
double_cmp(const void *pa, const void *pb)
{
    double a = *(const double *)pa, b = *(const double *)pb;
    return (a < b) ? -1 : (a > b ? 1 : 0);
}

static double
spike_severity(const spike_candidate *c)
{
    return (c->ratio > c->global_ratio) ? c->ratio : c->global_ratio;
}

static int
spike_cmp(const void *pa, const void *pb)
{
    double sa = spike_severity((const spike_candidate *)pa);
    double sb = spike_severity((const spike_candidate *)pb);
    return (sa < sb) ? 1 : (sa > sb ? -1 : 0);
}

static double
edge_len(const float *p0, const float *p1)
{
    double dx = p0[0] - p1[0], dy = p0[1] - p1[1], dz = p0[2] - p1[2];
    return sqrt(dx*dx + dy*dy + dz*dz);
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree;
    uint32_t num_parts, num_products, num_markups;
    uint32_t totalTesselations, totalLineTesselations, k, j;
    uint8_t has_lines = 0;
    int code;
    prc_api_tess *tesses;
    double ratio_threshold = 6.0;
    double global_ratio_threshold = 15.0;
    uint32_t top_n = 50;
    spike_candidate *candidates = NULL;
    uint32_t num_candidates = 0, cand_cap = 0;
    uint64_t total_tris_scanned = 0;

    if (argc < 2)
    {
        printf("usage: %s <input.pdf> [local_ratio_threshold=6.0] [top_n=50] [global_ratio_threshold=15.0]\n", argv[0]);
        return 2;
    }
    if (argc >= 3) ratio_threshold = atof(argv[2]);
    if (argc >= 4) top_n = (uint32_t)atoi(argv[3]);
    if (argc >= 5) global_ratio_threshold = atof(argv[4]);

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("prc_api_open_contents failed\n"); prc_api_print_error_stack(ctx); return 1; }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0) { printf("prep_model_tree failed\n"); return 1; }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0) { printf("create_model_tree failed\n"); return 1; }

    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations, &totalLineTesselations);
    if (code < 0) { printf("get_number_tessellations failed\n"); return 1; }
    printf("parts=%u products=%u markups=%u total tessellations=%u\n",
        num_parts, num_products, num_markups, totalTesselations);
    printf("local_ratio_threshold=%.2f global_ratio_threshold=%.2f top_n=%u\n\n",
        ratio_threshold, global_ratio_threshold, top_n);

    tesses = (prc_api_tess *)calloc(totalTesselations, sizeof(prc_api_tess));

    for (k = 0; k < totalTesselations; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t nFaces = prc_api_get_number_faces(ctx, data, k);
        tess->num_faces = nFaces;
        tess->tess_faces = (prc_api_face *)calloc(nFaces, sizeof(prc_api_face));

        code = prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL, &has_lines);
        if (code < 0) continue;
        if (tess->type != PRC_API_TESS_3D_Compressed && tess->type != PRC_API_TESS_3D)
            continue;

        for (j = 0; j < tess->num_faces; j++)
        {
            code = prc_api_get_tessellation_vertices(ctx, data, model_tree, k, j, tess->tess_faces + j, tess);
            if (code < 0) continue;
        }

        {
            uint32_t *tri = NULL;
            uint32_t tri_count = 0, tri_cap = 0;
            prc_api_tess_vertex_buffer *vb = (tess->type == PRC_API_TESS_3D_Compressed)
                ? &tess->tess_vertices : NULL;
            uint32_t f;
            /* Same face_limit=1-for-compressed workaround as verify_manifold_pdf.c
               (all triangles live in face_index 0 for compressed tessellations). */
            uint32_t face_limit = vb ? 1 : tess->num_faces;

            for (f = 0; f < face_limit; f++)
            {
                prc_api_face *face = &tess->tess_faces[f];
                uint32_t p;
                for (p = 0; p < face->num_graphic_primitives; p++)
                {
                    prc_api_graphic_primitive prim;
                    size_t idx;
                    code = prc_api_get_graphics_primitive(ctx, data, tess, (uint8_t)f, p, &prim);
                    if (code < 0) continue;

                    if (prim.type == PRC_API_TRIANGLES)
                    {
                        for (idx = 0; idx + 3 <= prim.num_indices; idx += 3)
                        {
                            if (tri_count + 3 > tri_cap) { tri_cap = tri_cap ? tri_cap * 2 : 1024; tri = (uint32_t *)realloc(tri, sizeof(uint32_t) * tri_cap); }
                            tri[tri_count++] = prim.indices[idx+0];
                            tri[tri_count++] = prim.indices[idx+1];
                            tri[tri_count++] = prim.indices[idx+2];
                        }
                    }
                    else if (prim.type == PRC_API_FAN)
                    {
                        for (idx = 1; idx + 1 < prim.num_indices; idx++)
                        {
                            if (tri_count + 3 > tri_cap) { tri_cap = tri_cap ? tri_cap * 2 : 1024; tri = (uint32_t *)realloc(tri, sizeof(uint32_t) * tri_cap); }
                            tri[tri_count++] = prim.indices[0];
                            tri[tri_count++] = prim.indices[idx];
                            tri[tri_count++] = prim.indices[idx+1];
                        }
                    }
                    else if (prim.type == PRC_API_STRIP)
                    {
                        for (idx = 0; idx + 2 < prim.num_indices; idx++)
                        {
                            if (tri_count + 3 > tri_cap) { tri_cap = tri_cap ? tri_cap * 2 : 1024; tri = (uint32_t *)realloc(tri, sizeof(uint32_t) * tri_cap); }
                            if (idx % 2 == 0)
                            {
                                tri[tri_count++] = prim.indices[idx+0];
                                tri[tri_count++] = prim.indices[idx+1];
                                tri[tri_count++] = prim.indices[idx+2];
                            }
                            else
                            {
                                tri[tri_count++] = prim.indices[idx+1];
                                tri[tri_count++] = prim.indices[idx+0];
                                tri[tri_count++] = prim.indices[idx+2];
                            }
                        }
                    }
                }
            }

            if (tri_count >= 9 && vb != NULL)
            {
                uint32_t ntri = tri_count / 3;
                edge_rec *edges = (edge_rec *)malloc(sizeof(edge_rec) * tri_count);
                uint32_t e = 0, t;
                double *max_edge = (double *)malloc(sizeof(double) * ntri);
                double *min_edge = (double *)malloc(sizeof(double) * ntri);
                uint32_t (*neighbors)[MAX_NEIGHBORS] = malloc(sizeof(uint32_t) * ntri * MAX_NEIGHBORS);
                uint32_t *neighbor_count = (uint32_t *)calloc(ntri, sizeof(uint32_t));
                prc_api_vertex *verts = vb->vertices;

                total_tris_scanned += ntri;

                for (t = 0; t < ntri; t++)
                {
                    uint32_t i0 = tri[t*3+0], i1 = tri[t*3+1], i2 = tri[t*3+2];
                    double l0 = edge_len(verts[i0].position, verts[i1].position);
                    double l1 = edge_len(verts[i1].position, verts[i2].position);
                    double l2 = edge_len(verts[i2].position, verts[i0].position);
                    double mx = l0 > l1 ? l0 : l1; if (l2 > mx) mx = l2;
                    double mn = l0 < l1 ? l0 : l1; if (l2 < mn) mn = l2;
                    max_edge[t] = mx;
                    min_edge[t] = mn;

                    {
                        uint32_t pairs[3][2] = { {i0,i1}, {i1,i2}, {i2,i0} };
                        int pe;
                        for (pe = 0; pe < 3; pe++)
                        {
                            uint32_t a = pairs[pe][0], b = pairs[pe][1];
                            edges[e].a = (a < b) ? a : b;
                            edges[e].b = (a < b) ? b : a;
                            edges[e].tri = t;
                            e++;
                        }
                    }
                }
                qsort(edges, e, sizeof(edge_rec), edge_rec_cmp);
                {
                    uint32_t run_start = 0, i2;
                    for (i2 = 1; i2 <= e; i2++)
                    {
                        if (i2 == e || edges[i2].a != edges[run_start].a || edges[i2].b != edges[run_start].b)
                        {
                            uint32_t run_len = i2 - run_start;
                            uint32_t p1, p2;
                            for (p1 = run_start; p1 < i2; p1++)
                            {
                                for (p2 = p1 + 1; p2 < i2; p2++)
                                {
                                    uint32_t t1 = edges[p1].tri, t2 = edges[p2].tri;
                                    if (neighbor_count[t1] < MAX_NEIGHBORS)
                                        neighbors[t1][neighbor_count[t1]++] = t2;
                                    if (neighbor_count[t2] < MAX_NEIGHBORS)
                                        neighbors[t2][neighbor_count[t2]++] = t1;
                                }
                            }
                            (void)run_len;
                            run_start = i2;
                        }
                    }
                }

                /* Per-tessellation GLOBAL median, in addition to the per-triangle
                   LOCAL (1-ring neighbor) median above. A contiguous run/cluster of
                   triangles that are all wrong together (e.g. many triangles that
                   all incorrectly resolved a reference to the same wrong vertex,
                   fanning out from one point) are locally self-consistent -- their
                   immediate neighbors are also bad, so the local ratio stays near 1
                   -- but stand out clearly against the tessellation's overall scale. */
                {
                    double *sorted_all = (double *)malloc(sizeof(double) * ntri);
                    double global_median;
                    memcpy(sorted_all, max_edge, sizeof(double) * ntri);
                    qsort(sorted_all, ntri, sizeof(double), double_cmp);
                    global_median = sorted_all[ntri / 2];
                    free(sorted_all);

                for (t = 0; t < ntri; t++)
                {
                    double sample[MAX_NEIGHBORS];
                    uint32_t nc = neighbor_count[t], si;
                    double local_median = 0.0;
                    double ratio = 0.0;
                    double global_ratio = (global_median > 0.0) ? max_edge[t] / global_median : 0.0;
                    int by_local = 0, by_global = 0;

                    if (nc > 0)
                    {
                        for (si = 0; si < nc; si++)
                            sample[si] = max_edge[neighbors[t][si]];
                        qsort(sample, nc, sizeof(double), double_cmp);
                        local_median = sample[nc / 2];
                        if (local_median > 0.0)
                        {
                            ratio = max_edge[t] / local_median;
                            by_local = (ratio > ratio_threshold);
                        }
                    }
                    by_global = (global_ratio > global_ratio_threshold);

                    if (by_local || by_global)
                    {
                        uint32_t i0 = tri[t*3+0], i1 = tri[t*3+1], i2 = tri[t*3+2];
                        if (num_candidates >= cand_cap)
                        {
                            cand_cap = cand_cap ? cand_cap * 2 : 256;
                            candidates = (spike_candidate *)realloc(candidates, sizeof(spike_candidate) * cand_cap);
                        }
                        candidates[num_candidates].tess_index = k;
                        candidates[num_candidates].tri_index = t;
                        candidates[num_candidates].ratio = ratio;
                        candidates[num_candidates].own_max_edge = max_edge[t];
                        candidates[num_candidates].local_median = local_median;
                        candidates[num_candidates].global_median = global_median;
                        candidates[num_candidates].global_ratio = global_ratio;
                        candidates[num_candidates].flagged_by_global = by_global;
                        memcpy(candidates[num_candidates].v0, verts[i0].position, sizeof(float)*3);
                        memcpy(candidates[num_candidates].v1, verts[i1].position, sizeof(float)*3);
                        memcpy(candidates[num_candidates].v2, verts[i2].position, sizeof(float)*3);
                        num_candidates++;
                    }
                }
                }

                free(edges);
                free(max_edge);
                free(min_edge);
                free(neighbors);
                free(neighbor_count);
            }
            free(tri);
        }
    }

    printf("Scanned %llu triangles across %u tessellations.\n",
        (unsigned long long)total_tris_scanned, totalTesselations);
    printf("Spike candidates (ratio > %.2f): %u\n\n", ratio_threshold, num_candidates);

    qsort(candidates, num_candidates, sizeof(spike_candidate), spike_cmp);

    {
        uint32_t n = num_candidates < top_n ? num_candidates : top_n;
        uint32_t i;
        for (i = 0; i < n; i++)
        {
            spike_candidate *c = &candidates[i];
            printf("#%u  tess=%u tri=%u  local_ratio=%.1fx  global_ratio=%.1fx%s  own_max_edge=%.6g  local_median=%.6g  global_median=%.6g\n",
                i, c->tess_index, c->tri_index, c->ratio, c->global_ratio,
                c->flagged_by_global ? " [GLOBAL]" : "",
                c->own_max_edge, c->local_median, c->global_median);
            printf("     V0=(%.6g,%.6g,%.6g)  V1=(%.6g,%.6g,%.6g)  V2=(%.6g,%.6g,%.6g)\n",
                c->v0[0], c->v0[1], c->v0[2],
                c->v1[0], c->v1[1], c->v1[2],
                c->v2[0], c->v2[1], c->v2[2]);
        }
    }

    free(candidates);
    prc_api_release_data(ctx, data, tesses, totalTesselations, NULL, 0, model_tree);
    prc_api_release_context(ctx);
    return (num_candidates > 0) ? 3 : 0;
}
