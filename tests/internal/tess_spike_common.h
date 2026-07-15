/* INTERNAL DEVELOPMENT TOOL SUPPORT -- not part of the public API, no
   stability contract.

   Shared spike-detection primitives factored out of detect_tess_spikes.c
   (see that file's header comment for the full rationale/methodology) so
   repair_tess_spikes.c can reuse the identical adjacency-building and
   ratio-scoring logic instead of duplicating it -- including re-scoring a
   hypothetical candidate vertex position, not just the as-decoded one. */
#ifndef TESS_SPIKE_COMMON_H
#define TESS_SPIKE_COMMON_H

#include <stdint.h>
#include "prc_api.h"

#define TESS_SPIKE_MAX_NEIGHBORS 8

typedef struct { uint32_t a, b, tri; } tess_spike_edge_rec;

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
} tess_spike_candidate;

double tess_spike_edge_len(const float *p0, const float *p1);

/* qsort comparator over `double`, ascending. */
int tess_spike_double_cmp(const void *pa, const void *pb);

/* Builds face adjacency (sorted-edge-run technique: every triangle-edge
   canonicalized to (min,max) vertex index, sorted, then every pair of
   triangles sharing a run is a neighbor pair) for `ntri` triangles given
   as flat vertex-index triples in `tri` (3*ntri entries). Caller supplies
   pre-allocated neighbors[ntri][TESS_SPIKE_MAX_NEIGHBORS] and
   neighbor_count[ntri]; both are zeroed/filled by this function. Neighbor
   lists are capped at TESS_SPIKE_MAX_NEIGHBORS entries per triangle. */
void tess_spike_build_adjacency(const uint32_t *tri, uint32_t ntri,
    uint32_t (*neighbors)[TESS_SPIKE_MAX_NEIGHBORS], uint32_t *neighbor_count);

/* Computes each triangle's own longest/shortest edge length from `verts`
   (indexed by `tri`). Caller supplies pre-allocated max_edge[ntri]/
   min_edge[ntri] (min_edge may be NULL if not needed). */
void tess_spike_compute_edge_lengths(const uint32_t *tri, uint32_t ntri,
    const prc_api_vertex *verts, double *max_edge, double *min_edge);

/* Median of max_edge[0..ntri) -- the whole-tessellation reference scale
   used by the global-ratio check. */
double tess_spike_global_median(const double *max_edge, uint32_t ntri);

/* Evaluates triangle t's spike ratios against precomputed adjacency/
   max_edge/global_median data. Returns 1 if flagged (local ratio exceeds
   ratio_threshold OR global ratio exceeds global_ratio_threshold), 0
   otherwise. *out_ratio / *out_local_median / *out_global_ratio / *out_by_global
   are always filled regardless of the flag outcome, so a caller re-scoring
   a hypothetical candidate position (with max_edge[t] recomputed for that
   candidate, everything else held fixed) can compare severities directly. */
int tess_spike_evaluate(uint32_t t, const double *max_edge,
    const uint32_t (*neighbors)[TESS_SPIKE_MAX_NEIGHBORS], const uint32_t *neighbor_count,
    double global_median, double ratio_threshold, double global_ratio_threshold,
    double *out_ratio, double *out_local_median, double *out_global_ratio, int *out_by_global);

/* max(ratio, global_ratio) -- the single severity score candidates are
   ranked by. */
double tess_spike_severity(const tess_spike_candidate *c);

/* qsort comparator over tess_spike_candidate, descending by severity. */
int tess_spike_cmp(const void *pa, const void *pb);

#endif
