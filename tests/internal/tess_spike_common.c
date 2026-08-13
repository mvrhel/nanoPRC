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

/* INTERNAL DEVELOPMENT TOOL SUPPORT -- see tess_spike_common.h. */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "tess_spike_common.h"

double
tess_spike_edge_len(const float *p0, const float *p1)
{
    double dx = p0[0] - p1[0], dy = p0[1] - p1[1], dz = p0[2] - p1[2];
    return sqrt(dx*dx + dy*dy + dz*dz);
}

int
tess_spike_double_cmp(const void *pa, const void *pb)
{
    double a = *(const double *)pa, b = *(const double *)pb;
    return (a < b) ? -1 : (a > b ? 1 : 0);
}

static int
tess_spike_edge_rec_cmp(const void *pa, const void *pb)
{
    const tess_spike_edge_rec *a = (const tess_spike_edge_rec *)pa;
    const tess_spike_edge_rec *b = (const tess_spike_edge_rec *)pb;
    if (a->a != b->a) return (a->a < b->a) ? -1 : 1;
    if (a->b != b->b) return (a->b < b->b) ? -1 : 1;
    return (a->tri < b->tri) ? -1 : (a->tri > b->tri ? 1 : 0);
}

void
tess_spike_build_adjacency(const uint32_t *tri, uint32_t ntri,
    uint32_t (*neighbors)[TESS_SPIKE_MAX_NEIGHBORS], uint32_t *neighbor_count)
{
    tess_spike_edge_rec *edges = (tess_spike_edge_rec *)malloc(sizeof(tess_spike_edge_rec) * (size_t)ntri * 3);
    uint32_t e = 0, t;

    for (t = 0; t < ntri; t++)
        neighbor_count[t] = 0;

    for (t = 0; t < ntri; t++)
    {
        uint32_t i0 = tri[t*3+0], i1 = tri[t*3+1], i2 = tri[t*3+2];
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
    qsort(edges, e, sizeof(tess_spike_edge_rec), tess_spike_edge_rec_cmp);
    {
        uint32_t run_start = 0, i;
        for (i = 1; i <= e; i++)
        {
            if (i == e || edges[i].a != edges[run_start].a || edges[i].b != edges[run_start].b)
            {
                uint32_t p1, p2;
                for (p1 = run_start; p1 < i; p1++)
                {
                    for (p2 = p1 + 1; p2 < i; p2++)
                    {
                        uint32_t t1 = edges[p1].tri, t2 = edges[p2].tri;
                        if (neighbor_count[t1] < TESS_SPIKE_MAX_NEIGHBORS)
                            neighbors[t1][neighbor_count[t1]++] = t2;
                        if (neighbor_count[t2] < TESS_SPIKE_MAX_NEIGHBORS)
                            neighbors[t2][neighbor_count[t2]++] = t1;
                    }
                }
                run_start = i;
            }
        }
    }
    free(edges);
}

void
tess_spike_compute_edge_lengths(const uint32_t *tri, uint32_t ntri,
    const prc_api_vertex *verts, double *max_edge, double *min_edge)
{
    uint32_t t;
    for (t = 0; t < ntri; t++)
    {
        uint32_t i0 = tri[t*3+0], i1 = tri[t*3+1], i2 = tri[t*3+2];
        double l0 = tess_spike_edge_len(verts[i0].position, verts[i1].position);
        double l1 = tess_spike_edge_len(verts[i1].position, verts[i2].position);
        double l2 = tess_spike_edge_len(verts[i2].position, verts[i0].position);
        double mx = l0 > l1 ? l0 : l1; if (l2 > mx) mx = l2;
        max_edge[t] = mx;
        if (min_edge != NULL)
        {
            double mn = l0 < l1 ? l0 : l1; if (l2 < mn) mn = l2;
            min_edge[t] = mn;
        }
    }
}

double
tess_spike_global_median(const double *max_edge, uint32_t ntri)
{
    double *sorted_all = (double *)malloc(sizeof(double) * ntri);
    double median;
    memcpy(sorted_all, max_edge, sizeof(double) * ntri);
    qsort(sorted_all, ntri, sizeof(double), tess_spike_double_cmp);
    median = sorted_all[ntri / 2];
    free(sorted_all);
    return median;
}

int
tess_spike_evaluate(uint32_t t, const double *max_edge,
    const uint32_t (*neighbors)[TESS_SPIKE_MAX_NEIGHBORS], const uint32_t *neighbor_count,
    double global_median, double ratio_threshold, double global_ratio_threshold,
    double *out_ratio, double *out_local_median, double *out_global_ratio, int *out_by_global)
{
    double sample[TESS_SPIKE_MAX_NEIGHBORS];
    uint32_t nc = neighbor_count[t], si;
    double local_median = 0.0;
    double ratio = 0.0;
    double global_ratio = (global_median > 0.0) ? max_edge[t] / global_median : 0.0;
    int by_local = 0, by_global;

    if (nc > 0)
    {
        for (si = 0; si < nc; si++)
            sample[si] = max_edge[neighbors[t][si]];
        qsort(sample, nc, sizeof(double), tess_spike_double_cmp);
        local_median = sample[nc / 2];
        if (local_median > 0.0)
        {
            ratio = max_edge[t] / local_median;
            by_local = (ratio > ratio_threshold);
        }
    }
    by_global = (global_ratio > global_ratio_threshold);

    *out_ratio = ratio;
    *out_local_median = local_median;
    *out_global_ratio = global_ratio;
    *out_by_global = by_global;
    return by_local || by_global;
}

double
tess_spike_severity(const tess_spike_candidate *c)
{
    return (c->ratio > c->global_ratio) ? c->ratio : c->global_ratio;
}

int
tess_spike_cmp(const void *pa, const void *pb)
{
    double sa = tess_spike_severity((const tess_spike_candidate *)pa);
    double sb = tess_spike_severity((const tess_spike_candidate *)pb);
    return (sa < sb) ? 1 : (sa > sb ? -1 : 0);
}
