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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "prc_write_compress_tess.h"
#include "prc_vector_util.h"
#include "prc_parse_common.h"
#include "prc_decode_compressed_tess.h"
#include "prc_huff.h"
#include "prc_diag_env.h"

typedef struct
{
    int64_t key[3];
    uint32_t index;
    uint8_t used;
} prc_vtx_slot;

typedef struct
{
    uint32_t v0, v1;
    uint32_t edge_index;
    uint8_t used;
} prc_edge_slot;

/* EXPERIMENT (2026-08-06), mixed_chains/QCD_Leinweber connectivity-gap investigation:
   an alternative to this file's default hash-grid vertex dedup (prc_vtx_hash below),
   modeled on a described-but-unseen independent encoder's own weld algorithm (project
   notes: "Missing-Weld-Algorithm-Conjecture"). The hash-grid approach quantizes each
   coordinate via llround(v/tol) and requires an EXACT post-quantization key match --
   two genuinely-within-tolerance points straddling a quantization cell boundary (one
   rounds down, one rounds up) are silently never merged, with no neighbor-cell search
   to catch it (unlike demos/stl_import's own external weld_grid, which does search a
   3x3x3 neighborhood -- this internal pass does not). The sort-based alternative:
   sort every vertex lexicographically by (Z, Y, X), then walk the sorted array
   comparing each point ONLY to its immediate predecessor in sort order (per-axis
   fabs(a-b) <= tolerance, not Euclidean distance) -- points within tolerance of their
   neighbor join the same group (transitively: a gradually-drifting run of points can
   all collapse into one group even if the first and last members are more than
   `tolerance` apart from each other directly). Gated behind PRC_DIAG_WELD_SORT_METHOD;
   default (unset) behavior is completely unchanged. */
typedef struct
{
    double x, y, z;
    uint32_t orig_index;
} prc_weldsort_vtx;

static int
prc_weldsort_cmp(const void *pa, const void *pb)
{
    const prc_weldsort_vtx *a = (const prc_weldsort_vtx *)pa;
    const prc_weldsort_vtx *b = (const prc_weldsort_vtx *)pb;
    if (a->z < b->z) return -1;
    if (a->z > b->z) return 1;
    if (a->y < b->y) return -1;
    if (a->y > b->y) return 1;
    if (a->x < b->x) return -1;
    if (a->x > b->x) return 1;
    if (a->orig_index < b->orig_index) return -1;
    if (a->orig_index > b->orig_index) return 1;
    return 0;
}

static size_t
prc_next_pow2(size_t v)
{
    size_t c = 16;
    while (c < v)
        c *= 2;
    return c;
}

static uint64_t
prc_mix64(uint64_t h)
{
    h ^= h >> 33;
    h *= 0xFF51AFD7ED558CCDULL;
    h ^= h >> 33;
    h *= 0xC4CEB9FE1A85EC53ULL;
    h ^= h >> 33;
    return h;
}

static uint64_t
prc_vtx_hash(int64_t kx, int64_t ky, int64_t kz)
{
    uint64_t h = prc_mix64((uint64_t)kx);
    h = prc_mix64(h ^ (uint64_t)ky);
    h = prc_mix64(h ^ (uint64_t)kz);
    return h;
}

/* MITIGATION (2026-07-26, mixed_chains/UK_original.stl/beetle_1000000.stl Acrobat blank-tree
   investigation): applies a tiny, deterministic per-vertex offset to every deduplicated position
   before any encoding happens, to break a real, reproducible, but still-unidentified Acrobat
   defect where two independent chains sharing one COMPRESSED entry can, for specific EXACT
   real-world vertex values, blank the model tree -- confirmed causal via a minimal repro (nudging
   the offending chain's own vertex by as little as 0.01 fixed it; nanoPRC's own decoder, and at
   least one independent PRC reader, both already decode the un-nudged geometry correctly, so this
   is Acrobat-side, not a correctness bug in this codebase) and confirmed as a working mitigation
   on both real files that originally motivated this investigation. Root cause NOT identified --
   see project notes -- this is an empirical mitigation, not a fix for a known mechanism.

   Deterministic (same input position always produces the same offset) so re-encoding the same
   mesh is reproducible, and offsets only depend on each vertex's OWN (already-deduplicated)
   position -- never on the number of times it's referenced or on unrelated other vertices -- so
   this cannot un-weld or split anything that was already correctly merged upstream. Magnitude is
   tied to the encoder's own resolved tolerance rather than an absolute constant so it scales with
   whatever precision the caller actually asked for. The exact magnitude needed to escape a given
   real collision varies by mesh (confirmed empirically: 1e-4 sufficed for one real file, another
   needed roughly 100x more) -- PRC_ENCODE_JITTER_TOLERANCE_FACTOR (prc_write_compress_tess.h,
   shared with test code that needs to budget for it) is chosen generously given that uncertainty,
   erring toward "definitely breaks the coincidence" over "minimal possible change". This is a real,
   documented widening of this write facility's position-fidelity contract, not just quantization
   noise -- see that constant's own comment. */

/* Minimal, single-block-only MD5 (RFC 1321): the jitter's own input is always exactly 12 bytes
   (3 packed float32 coordinates), which always fits in one 64-byte MD5 block after standard
   padding, so this deliberately does not implement the general streaming/multi-block case.
   Written to REPLICATE, byte-for-byte, an earlier externally-scripted (Python hashlib.md5)
   version of this mitigation that was empirically confirmed working on a real file
   (beetle_1000000.stl), after FIVE separate simpler-hash variants (mix64-based, in various
   combinations of seed/magnitude/float-precision/pre-vs-post-weld application order) all failed
   to reproduce that result -- see this function's own caller for the fuller story. Whatever
   about MD5's specific output distribution happens to work for this mesh is not understood
   mechanistically; this exists to get the PROVEN-working bit pattern, not because MD5 is
   believed to be special. */
static void
prc_md5_12bytes(const uint8_t in[12], uint8_t out[16])
{
    static const uint32_t K[64] = {
        0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
        0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,0x6b901122,0xfd987193,0xa679438e,0x49b40821,
        0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
        0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
        0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
        0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
        0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
        0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391 };
    static const uint32_t S[64] = {
        7,12,17,22, 7,12,17,22, 7,12,17,22, 7,12,17,22,
        5, 9,14,20, 5, 9,14,20, 5, 9,14,20, 5, 9,14,20,
        4,11,16,23, 4,11,16,23, 4,11,16,23, 4,11,16,23,
        6,10,15,21, 6,10,15,21, 6,10,15,21, 6,10,15,21 };
    uint8_t block[64];
    uint32_t M[16];
    uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
    uint32_t a = a0, b = b0, c = c0, d = d0;
    int i;

    memset(block, 0, sizeof(block));
    memcpy(block, in, 12);
    block[12] = 0x80;
    /* bit length (12*8=96) as 64-bit little-endian in the last 8 bytes */
    block[56] = 96; /* fits in one byte; rest already zero */

    for (i = 0; i < 16; i++)
        M[i] = (uint32_t)block[i * 4] | ((uint32_t)block[i * 4 + 1] << 8) |
               ((uint32_t)block[i * 4 + 2] << 16) | ((uint32_t)block[i * 4 + 3] << 24);

    for (i = 0; i < 64; i++)
    {
        uint32_t f, g, tmp;
        if (i < 16) { f = (b & c) | (~b & d); g = (uint32_t)i; }
        else if (i < 32) { f = (d & b) | (~d & c); g = (uint32_t)((5 * i + 1) % 16); }
        else if (i < 48) { f = b ^ c ^ d; g = (uint32_t)((3 * i + 5) % 16); }
        else { f = c ^ (b | ~d); g = (uint32_t)((7 * i) % 16); }
        tmp = d;
        d = c;
        c = b;
        f = a + f + K[i] + M[g];
        b = b + ((f << S[i]) | (f >> (32 - S[i])));
        a = tmp;
    }
    a0 += a; b0 += b; c0 += c; d0 += d;
    out[0] = (uint8_t)a0; out[1] = (uint8_t)(a0 >> 8); out[2] = (uint8_t)(a0 >> 16); out[3] = (uint8_t)(a0 >> 24);
    out[4] = (uint8_t)b0; out[5] = (uint8_t)(b0 >> 8); out[6] = (uint8_t)(b0 >> 16); out[7] = (uint8_t)(b0 >> 24);
    out[8] = (uint8_t)c0; out[9] = (uint8_t)(c0 >> 8); out[10] = (uint8_t)(c0 >> 16); out[11] = (uint8_t)(c0 >> 24);
    out[12] = (uint8_t)d0; out[13] = (uint8_t)(d0 >> 8); out[14] = (uint8_t)(d0 >> 16); out[15] = (uint8_t)(d0 >> 24);
}

static void
prc_encode_position_jitter(double x, double y, double z, double magnitude, uint64_t seed,
    double *jx, double *jy, double *jz)
{
    float fx, fy, fz;
    uint8_t in[12];
    uint8_t digest[16];
    uint32_t hx, hy, hz;
    (void)seed; /* the MD5 replication has no seed input; kept for call-site compatibility */

    fx = (float)x;
    fy = (float)y;
    fz = (float)z;
    memcpy(in + 0, &fx, 4);
    memcpy(in + 4, &fy, 4);
    memcpy(in + 8, &fz, 4);
    prc_md5_12bytes(in, digest);

    if (prc_diag_getenv("PRC_DIAG_MD5_SELFTEST") != NULL)
    {
        int di;
        fprintf(stderr, "PRC_DIAG_MD5_SELFTEST digest=");
        for (di = 0; di < 16; di++) fprintf(stderr, "%02x", digest[di]);
        fprintf(stderr, "\n");
    }

    hx = (uint32_t)digest[0] | ((uint32_t)digest[1] << 8) | ((uint32_t)digest[2] << 16) | ((uint32_t)digest[3] << 24);
    hy = (uint32_t)digest[4] | ((uint32_t)digest[5] << 8) | ((uint32_t)digest[6] << 16) | ((uint32_t)digest[7] << 24);
    hz = (uint32_t)digest[8] | ((uint32_t)digest[9] << 8) | ((uint32_t)digest[10] << 16) | ((uint32_t)digest[11] << 24);

    *jx = (((double)hx / (double)0xFFFFFFFFu) * 2.0 - 1.0) * magnitude;
    *jy = (((double)hy / (double)0xFFFFFFFFu) * 2.0 - 1.0) * magnitude;
    *jz = (((double)hz / (double)0xFFFFFFFFu) * 2.0 - 1.0) * magnitude;
}

/* Path halving keeps find() strictly iterative, per the project rule that
   input-size-driven depth must never use C call-stack recursion. */
static uint32_t
prc_uf_find(uint32_t *parent, uint32_t x)
{
    while (parent[x] != x)
    {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

/* Builds (or rebuilds) the edge-adjacency hash table and out->edges array
   from a (possibly just-remapped) triangle index array. `etable` must be
   zeroed by the caller before each call (reused across builds rather than
   reallocated); `edges_out` must have capacity for at least clean_tris*3
   entries, an upper bound independent of how many times this is called.
   Any edge that still ends up with a 3rd+ triangle referencing it (should
   not happen after the non-manifold-edge/-vertex fixes upstream have run,
   but is not re-verified here to avoid an unbounded fixed-point loop) is
   conservatively left with only its first two triangles linked, same as
   this write facility's original behavior for genuinely-3+-way edges. */
static uint32_t
prc_encode_rebuild_edges(prc_edge_slot *etable, size_t ecap,
    const uint32_t *tri_indices, uint32_t clean_tris, prc_encode_edge *edges_out)
{
    uint32_t nedges = 0;
    uint32_t i;

    for (i = 0; i < clean_tris; i++)
    {
        uint32_t e;
        for (e = 0; e < 3; e++)
        {
            uint32_t a = tri_indices[(size_t)i * 3 + e];
            uint32_t b = tri_indices[(size_t)i * 3 + ((e + 1) % 3)];
            uint32_t v0 = a < b ? a : b;
            uint32_t v1 = a < b ? b : a;
            size_t slot = (size_t)(prc_mix64(((uint64_t)v0 << 32) | (uint64_t)v1) & (uint64_t)(ecap - 1));

            for (;;)
            {
                prc_edge_slot *s = &etable[slot];
                if (!s->used)
                {
                    s->used = 1;
                    s->v0 = v0;
                    s->v1 = v1;
                    s->edge_index = nedges;
                    edges_out[nedges].v0 = v0;
                    edges_out[nedges].v1 = v1;
                    edges_out[nedges].tri0 = (int32_t)i;
                    edges_out[nedges].tri1 = -1;
                    nedges++;
                    break;
                }
                if (s->v0 == v0 && s->v1 == v1)
                {
                    if (edges_out[s->edge_index].tri1 == -1)
                        edges_out[s->edge_index].tri1 = (int32_t)i;
                    break;
                }
                slot = (slot + 1) & (ecap - 1);
            }
        }
    }
    return nedges;
}

int
prc_encode_preprocess_ex(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance,
    uint8_t skip_nonmanifold_edge_remap,
    prc_encode_mesh *out)
{
    prc_vtx_slot *vtable = NULL;
    prc_edge_slot *etable = NULL;
    uint32_t *remap = NULL;
    uint32_t *parent = NULL;
    uint32_t *label = NULL;
    uint32_t *presplit_tri_component = NULL;
    double diagonal, tol;
    uint32_t i;
    uint32_t clean_tris = 0;
    uint32_t prc_nonmanifold_edge_count_diag = 0;
    int ret = PRC_ERROR_INTERNAL;

    if (out == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_preprocess: NULL output mesh\n");
        return PRC_ERROR_INTERNAL;
    }
    memset(out, 0, sizeof(*out));

    if ((num_positions > 0 && positions == NULL) ||
        (num_triangles > 0 && tri_indices == NULL))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_preprocess: NULL input array with non-zero count\n");
        return PRC_ERROR_INTERNAL;
    }

    {
        size_t n;
        for (n = 0; n < (size_t)num_triangles * 3; n++)
        {
            if (tri_indices[n] >= num_positions)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_preprocess: triangle index out of range\n");
                return PRC_ERROR_INTERNAL;
            }
        }
    }

    if (num_positions > 0)
    {
        out->bbox[0] = out->bbox[3] = positions[0];
        out->bbox[1] = out->bbox[4] = positions[1];
        out->bbox[2] = out->bbox[5] = positions[2];
        for (i = 1; i < num_positions; i++)
        {
            uint32_t k;
            for (k = 0; k < 3; k++)
            {
                double v = positions[(size_t)i * 3 + k];
                if (v < out->bbox[k])
                    out->bbox[k] = v;
                if (v > out->bbox[k + 3])
                    out->bbox[k + 3] = v;
            }
        }
    }
    diagonal = sqrt(
        (out->bbox[3] - out->bbox[0]) * (out->bbox[3] - out->bbox[0]) +
        (out->bbox[4] - out->bbox[1]) * (out->bbox[4] - out->bbox[1]) +
        (out->bbox[5] - out->bbox[2]) * (out->bbox[5] - out->bbox[2]));
    tol = prc_write_tol_resolve(ctx, tolerance, diagonal);
    {
        const char *ov = prc_diag_getenv("PRC_DIAG_FORCE_TOLERANCE_MM");
        if (ov != NULL)
            tol = atof(ov);
    }
    out->tolerance_mm = tol;

    if (num_positions > 0)
    {
        /* Fixed-capacity table: capacity is a power of two >= 2 * the maximum
           number of insertions, so the load factor stays <= 0.5 and linear
           probing always terminates without needing rehash/growth. */
        size_t vcap = prc_next_pow2((size_t)num_positions * 2);
        uint32_t ndedup = 0;

        vtable = (prc_vtx_slot *)prc_calloc(ctx, vcap, sizeof(prc_vtx_slot));
        remap = (uint32_t *)prc_malloc(ctx, (size_t)num_positions * sizeof(uint32_t));
        out->positions = (double *)prc_malloc(ctx, (size_t)num_positions * 3 * sizeof(double));
        if (vtable == NULL || remap == NULL || out->positions == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess vertex dedup\n");
            ret = PRC_ERROR_MEMORY;
            goto fail;
        }

        if (prc_diag_getenv("PRC_DIAG_WELD_SORT_METHOD") != NULL)
        {
            /* See prc_weldsort_vtx's own comment above for the full rationale.
               EXTENDED (2026-08-06): RG's own real Walnut file keeps 65 separate
               chains where this method (merge tolerance == the wire/quantization
               tolerance, same as the old hash-grid dedup did) collapses the same
               file to 15-24 -- RG's own weld is LESS aggressive than any method
               tried so far, not more. The wire tolerance is a decoder-visible
               quantization value; nothing requires an encoder's own internal
               merge decision to use that same number -- RG is free to (and
               plausibly does) weld at a much TIGHTER radius while still quantizing
               at its own coarser wire tolerance. PRC_DIAG_WELD_SORT_TOLERANCE_MM
               lets the merge radius be set independently of `tol` (which continues
               to govern quantization only); defaults to `tol` (old behavior)
               when unset. */
            prc_weldsort_vtx *sorted = (prc_weldsort_vtx *)prc_malloc(ctx,
                (size_t)num_positions * sizeof(prc_weldsort_vtx));
            uint32_t group;
            double weld_tol = tol;
            const char *weld_tol_env = prc_diag_getenv("PRC_DIAG_WELD_SORT_TOLERANCE_MM");
            if (weld_tol_env != NULL)
                weld_tol = atof(weld_tol_env);

            if (sorted == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess weld-sort\n");
                ret = PRC_ERROR_MEMORY;
                goto fail;
            }
            for (i = 0; i < num_positions; i++)
            {
                sorted[i].x = positions[(size_t)i * 3 + 0];
                sorted[i].y = positions[(size_t)i * 3 + 1];
                sorted[i].z = positions[(size_t)i * 3 + 2];
                sorted[i].orig_index = i;
            }
            qsort(sorted, num_positions, sizeof(prc_weldsort_vtx), prc_weldsort_cmp);

            group = 0;
            out->positions[0] = sorted[0].x;
            out->positions[1] = sorted[0].y;
            out->positions[2] = sorted[0].z;
            remap[sorted[0].orig_index] = 0;
            for (i = 1; i < num_positions; i++)
            {
                double dz = fabs(sorted[i].z - sorted[i - 1].z);
                double dy = fabs(sorted[i].y - sorted[i - 1].y);
                double dx = fabs(sorted[i].x - sorted[i - 1].x);

                if (dz <= weld_tol && dy <= weld_tol && dx <= weld_tol)
                {
                    remap[sorted[i].orig_index] = group;
                }
                else
                {
                    group++;
                    out->positions[(size_t)group * 3 + 0] = sorted[i].x;
                    out->positions[(size_t)group * 3 + 1] = sorted[i].y;
                    out->positions[(size_t)group * 3 + 2] = sorted[i].z;
                    remap[sorted[i].orig_index] = group;
                }
            }
            ndedup = group + 1;
            prc_free(ctx, sorted);
        }
        else
        {
            /* MITIGATION (2026-07-26/27), OPT-IN ONLY: jitter must be applied BEFORE this
               dedup loop's own quantization/hashing, not after -- confirmed empirically via a
               clean, isolated A/B test on beetle_1000000.stl. Applying it HERE (before this
               function's own internal dedup) is a no-op for callers that already externally
               welded their input before calling in (e.g. demos/stl_import, which does its own
               separate weld pass first) -- confirmed empirically: this function's own dedup
               found nothing further to merge in that case, so jittering before vs after it
               produced byte-identical output either way, and did NOT reproduce the real fix
               (which required jittering demos/stl_import's own RAW, pre-its-own-weld positions
               instead -- see that file's PRC_DIAG_PREWELD_JITTER_MAG mitigation, the real
               default for that pipeline). Left here, OPT-IN via PRC_DIAG_ENABLE_POSITION_JITTER,
               as a possible mitigation for OTHER callers who do NOT externally pre-weld before
               calling prc_write_compress_tess_entry (for whom this dedup pass is the ONLY
               deduplication step, so jittering before it is the only point that COULD matter) --
               this has not itself been verified to fix any such case, unlike demos/stl_import's
               own mitigation. NOT a default: stacking with demos/stl_import's own mitigation
               double-jitters and has been confirmed to produce a DIFFERENT (still-failing)
               result than either mitigation alone. */
            uint8_t do_jitter = (prc_diag_getenv("PRC_DIAG_ENABLE_POSITION_JITTER") != NULL);
            double jitter_magnitude = tol * PRC_ENCODE_JITTER_TOLERANCE_FACTOR;
            uint64_t jitter_seed = 0;
            const char *seed_env = prc_diag_getenv("PRC_DIAG_JITTER_SEED");
            const char *factor_env = prc_diag_getenv("PRC_DIAG_JITTER_TOLERANCE_FACTOR");

            if (seed_env != NULL)
                jitter_seed = (uint64_t)strtoull(seed_env, NULL, 10);
            if (factor_env != NULL)
                jitter_magnitude = tol * atof(factor_env);

            for (i = 0; i < num_positions; i++)
            {
                const double *raw = positions + (size_t)i * 3;
                double p[3];
                int64_t kx, ky, kz;
                size_t slot;

                p[0] = raw[0]; p[1] = raw[1]; p[2] = raw[2];
                if (do_jitter)
                {
                    double jx, jy, jz;
                    prc_encode_position_jitter(p[0], p[1], p[2], jitter_magnitude, jitter_seed, &jx, &jy, &jz);
                    p[0] += jx; p[1] += jy; p[2] += jz;
                    /* Truncate to float32: confirmed empirically necessary, not cosmetic -- an
                       earlier, externally-scripted version of this mitigation wrote the
                       jittered result into a binary STL file (float32-only storage) before
                       nanoPRC ever read it back, so the value actually used was float32-
                       rounded. Without this truncation, the exact same hash/magnitude/
                       application-point combination (verified byte-identical to the working
                       script's own pre-truncation computation) still failed on
                       beetle_1000000.stl; adding just this truncation step, with nothing else
                       changed, fixed it. */
                    p[0] = (double)(float)p[0];
                    p[1] = (double)(float)p[1];
                    p[2] = (double)(float)p[2];
                }

                kx = (int64_t)llround(p[0] / tol);
                ky = (int64_t)llround(p[1] / tol);
                kz = (int64_t)llround(p[2] / tol);
                slot = (size_t)(prc_vtx_hash(kx, ky, kz) & (uint64_t)(vcap - 1));

                for (;;)
                {
                    prc_vtx_slot *s = &vtable[slot];
                    if (!s->used)
                    {
                        s->used = 1;
                        s->key[0] = kx;
                        s->key[1] = ky;
                        s->key[2] = kz;
                        s->index = ndedup;
                        memcpy(out->positions + (size_t)ndedup * 3, p, 3 * sizeof(double));
                        remap[i] = ndedup;
                        ndedup++;
                        break;
                    }
                    if (s->key[0] == kx && s->key[1] == ky && s->key[2] == kz)
                    {
                        remap[i] = s->index;
                        break;
                    }
                    slot = (slot + 1) & (vcap - 1);
                }
            }
        }
        out->num_positions = ndedup;
        if (ndedup < num_positions)
        {
            double *shrunk = (double *)prc_realloc(ctx, out->positions, (size_t)ndedup * 3 * sizeof(double));
            if (shrunk != NULL)
                out->positions = shrunk;
        }

        prc_free(ctx, vtable);
        vtable = NULL;
    }

    if (num_triangles > 0)
    {
        uint32_t removed = 0;
        /* Sliver-triangle filtering (default ON, 2026-07-23): the a==b/
           b==c/a==c check above only catches EXACT welded-duplicate
           corners. Real-world meshes (3D scans, CAD exports) commonly
           contain triangles with three DISTINCT welded corners that are
           still nearly collinear (near-zero area) -- confirmed via a real
           customer file (757-3516-1.STL, otherwise perfectly topologically
           clean: watertight, zero non-manifold edges/vertices, single
           component) to independently cause an Adobe-Acrobat-specific
           blank-model-tree rejection, distinct from the non-manifold-edge
           defect class fixed elsewhere in this function -- Acrobat's own
           (opaque) geometry validation apparently rejects some slivers
           outright even though this write facility's own encoder/decoder
           round-trip every position bit-for-bit self-consistently either
           way (i.e. this is NOT a bug in OUR bit-packing; every other
           reader tried tolerates the same bits fine). A sin(angle)-between-
           edges threshold of 0.01 (~0.57 degrees) is used: small enough
           that legitimate thin CAD features survive (a real, independently-
           produced, Acrobat-working reference file for a DIFFERENT
           investigation this session had its own thinnest sliver at
           sin-angle ~1.1e-3, an order of magnitude below this threshold,
           and opened fine), large enough to catch the confirmed-bad
           757-3516-1.STL case (worst sliver there: sin-angle ~4.9e-3).
           PRC_DIAG_SLIVER_SIN_THRESHOLD overrides this default (e.g. "0" to
           disable entirely for diagnostic comparison, or another positive
           value to tune) but is not required for normal operation. */
        const char *sliver_env = prc_diag_getenv("PRC_DIAG_SLIVER_SIN_THRESHOLD");
        double sliver_sin_threshold = sliver_env != NULL ? atof(sliver_env) : 0.01;
        uint32_t sliver_removed = 0;

        out->tri_indices = (uint32_t *)prc_malloc(ctx, (size_t)num_triangles * 3 * sizeof(uint32_t));
        out->tri_orig_index = (uint32_t *)prc_malloc(ctx, (size_t)num_triangles * sizeof(uint32_t));
        if (out->tri_indices == NULL || out->tri_orig_index == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess triangle remap\n");
            ret = PRC_ERROR_MEMORY;
            goto fail;
        }

        for (i = 0; i < num_triangles; i++)
        {
            uint32_t a = remap[tri_indices[(size_t)i * 3 + 0]];
            uint32_t b = remap[tri_indices[(size_t)i * 3 + 1]];
            uint32_t c = remap[tri_indices[(size_t)i * 3 + 2]];
            if (a == b || b == c || a == c)
            {
                removed++;
                continue;
            }
            if (sliver_sin_threshold > 0.0)
            {
                double *pa = &out->positions[(size_t)a * 3];
                double *pb = &out->positions[(size_t)b * 3];
                double *pc = &out->positions[(size_t)c * 3];
                double e0[3], e1[3], cr[3];
                double len_e0, len_e1, len_cr, denom;
                uint32_t d;

                for (d = 0; d < 3; d++) { e0[d] = pb[d] - pa[d]; e1[d] = pc[d] - pa[d]; }
                cr[0] = e0[1]*e1[2] - e0[2]*e1[1];
                cr[1] = e0[2]*e1[0] - e0[0]*e1[2];
                cr[2] = e0[0]*e1[1] - e0[1]*e1[0];
                len_e0 = sqrt(e0[0]*e0[0]+e0[1]*e0[1]+e0[2]*e0[2]);
                len_e1 = sqrt(e1[0]*e1[0]+e1[1]*e1[1]+e1[2]*e1[2]);
                len_cr = sqrt(cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]);
                denom = len_e0 * len_e1;
                if (denom > 0.0 && (len_cr / denom) < sliver_sin_threshold)
                {
                    removed++;
                    sliver_removed++;
                    continue;
                }
            }
            out->tri_indices[(size_t)clean_tris * 3 + 0] = a;
            out->tri_indices[(size_t)clean_tris * 3 + 1] = b;
            out->tri_indices[(size_t)clean_tris * 3 + 2] = c;
            out->tri_orig_index[clean_tris] = i;
            clean_tris++;
        }
        out->num_triangles = clean_tris;
        (void)removed;
        if (sliver_removed > 0 && prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
            printf("PRC_DIAG_SLIVER_SIN_THRESHOLD=%.6g: removed %u sliver triangles (of %u total removed)\n",
                sliver_sin_threshold, sliver_removed, removed);

        if (clean_tris == 0)
        {
            prc_free(ctx, out->tri_indices);
            out->tri_indices = NULL;
            prc_free(ctx, out->tri_orig_index);
            out->tri_orig_index = NULL;
        }
        else if (clean_tris < num_triangles)
        {
            uint32_t *shrunk = (uint32_t *)prc_realloc(ctx, out->tri_indices, (size_t)clean_tris * 3 * sizeof(uint32_t));
            uint32_t *shrunk_orig = (uint32_t *)prc_realloc(ctx, out->tri_orig_index, (size_t)clean_tris * sizeof(uint32_t));
            if (shrunk != NULL)
                out->tri_indices = shrunk;
            if (shrunk_orig != NULL)
                out->tri_orig_index = shrunk_orig;
        }
    }
    if (remap != NULL)
    {
        prc_free(ctx, remap);
        remap = NULL;
    }

    /* DIAGNOSTIC (2026-07-22, PRC_DIAG_MESH_QUALITY): measures how close
       to degenerate (in absolute terms, not just the a==b/b==c/a==c exact-
       duplicate check above) the welded mesh's triangles are -- real 3D-
       scan data can contain near-collinear/near-zero-area slivers that
       exact-duplicate-vertex filtering doesn't catch. Read-only, does not
       affect encoder output. */
    if (clean_tris > 0 && prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
    {
        double min_edge_len = 1e300, min_cross_len = 1e300;
        double min_edge_len_rel = 1e300, min_cross_len_rel = 1e300;
        uint32_t near_degenerate_flt_eps = 0;
        uint32_t i2;

        for (i2 = 0; i2 < clean_tris; i2++)
        {
            uint32_t a = out->tri_indices[(size_t)i2 * 3 + 0];
            uint32_t b = out->tri_indices[(size_t)i2 * 3 + 1];
            uint32_t c = out->tri_indices[(size_t)i2 * 3 + 2];
            double *pa = &out->positions[(size_t)a * 3];
            double *pb = &out->positions[(size_t)b * 3];
            double *pc = &out->positions[(size_t)c * 3];
            double e0[3], e1[3], e2[3], cr[3];
            double len_e0, len_e1, len_e2, len_cr;
            uint32_t d;

            for (d = 0; d < 3; d++)
            {
                e0[d] = pb[d] - pa[d];
                e1[d] = pc[d] - pa[d];
                e2[d] = pc[d] - pb[d];
            }
            cr[0] = e0[1] * e1[2] - e0[2] * e1[1];
            cr[1] = e0[2] * e1[0] - e0[0] * e1[2];
            cr[2] = e0[0] * e1[1] - e0[1] * e1[0];
            len_e0 = sqrt(e0[0]*e0[0]+e0[1]*e0[1]+e0[2]*e0[2]);
            len_e1 = sqrt(e1[0]*e1[0]+e1[1]*e1[1]+e1[2]*e1[2]);
            len_e2 = sqrt(e2[0]*e2[0]+e2[1]*e2[1]+e2[2]*e2[2]);
            len_cr = sqrt(cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]);

            if (len_e0 < min_edge_len) min_edge_len = len_e0;
            if (len_e1 < min_edge_len) min_edge_len = len_e1;
            if (len_e2 < min_edge_len) min_edge_len = len_e2;
            if (len_cr < min_cross_len) min_cross_len = len_cr;
            /* relative to this triangle's own longest edge, to catch
               slivers that are small in absolute terms too but whose
               DEGENERACY (near-collinearity) matters more than absolute
               scale -- cross product length / (edge0_len * edge1_len)
               approximates sin(angle) between the two edges. */
            {
                double denom = len_e0 * len_e1;
                if (denom > 0.0)
                {
                    double sin_angle = len_cr / denom;
                    if (sin_angle < min_cross_len_rel) min_cross_len_rel = sin_angle;
                }
            }
            if (len_e0 < FLT_EPSILON || len_e1 < FLT_EPSILON || len_e2 < FLT_EPSILON || len_cr < FLT_EPSILON)
                near_degenerate_flt_eps++;
        }
        (void)min_edge_len_rel;
        printf("PRC_DIAG_MESH_QUALITY: clean_tris=%u min_edge_len=%.9e min_cross_len=%.9e min_sin_angle=%.9e near_FLT_EPSILON_count=%u bbox_diagonal=%.6f FLT_EPSILON=%.9e\n",
            clean_tris, min_edge_len, min_cross_len, min_cross_len_rel, near_degenerate_flt_eps, diagonal, (double)FLT_EPSILON);
    }

    /* Non-manifold input (3+ triangles sharing one edge): a single-pass
       2-manifold traversal can only ever treat an edge as connecting
       exactly two triangles. Confirmed via real-file bisection (2026-07-22
       investigation, hand.stl -- a real 3D scan with a genuine 4-triangle
       edge fan, most likely a fold/overlap artifact from the scanning
       process) that BOTH of the alternatives tried before this one fail:
       (a) silently linking just the first two triangles and dropping the
       rest (the original behavior) produces a blank model tree in Adobe
       Acrobat while every other reader tolerates it; (b) leaving every
       triangle in place but retroactively treating the edge as a boundary
       for everyone ALSO fails -- the final decoded geometry is identical
       either way, so Acrobat's rejection tracks the reconstructed 3D
       topology itself, not this write facility's own internal
       chain-growth bookkeeping. Confirmed via a minimal 6-triangle
       standalone repro that fully REMOVING the excess triangles fixes it,
       but that loses real user geometry. The fix that both fixes Acrobat
       AND keeps every triangle (confirmed on the same repro): give every
       triangle beyond the first two on a shared edge its own PRIVATE,
       imperceptibly-offset copy of that edge's two vertices, so no edge
       in the final mesh is ever shared by more than two triangles -- a
       genuinely valid 2-manifold structure, with the "extra" triangles now
       just very slightly detached (an offset far below visual perception,
       scaled off this mesh's own resolved encoding tolerance) rather than
       welded onto the same edge as everyone else.

       Mechanics: walk every triangle's 3 edges, hashing each by its
       (min-vertex, max-vertex) pair into an open-addressed table (etable)
       so the two triangles sharing an edge always land on the same slot
       regardless of which one is visited first. The first two triangles
       seen for a given edge get linked normally (out->edges[...].tri0/
       tri1); a THIRD (or later) triangle on that same edge is not linked
       at all -- it's queued (excess_tri/excess_slot) for the private-copy
       treatment below instead, once every triangle has been scanned. */
    if (clean_tris > 0)
    {
        size_t max_edges = (size_t)clean_tris * 3;
        size_t ecap = prc_next_pow2(max_edges * 2);
        uint32_t nedges = 0;
        uint32_t *excess_tri = NULL;
        uint32_t *excess_slot = NULL;
        uint32_t num_excess = 0;

        etable = (prc_edge_slot *)prc_calloc(ctx, ecap, sizeof(prc_edge_slot));
        out->edges = (prc_encode_edge *)prc_malloc(ctx, max_edges * sizeof(prc_encode_edge));
        excess_tri = (uint32_t *)prc_malloc(ctx, max_edges * sizeof(uint32_t));
        excess_slot = (uint32_t *)prc_malloc(ctx, max_edges * sizeof(uint32_t));
        if (etable == NULL || out->edges == NULL || excess_tri == NULL || excess_slot == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess edge adjacency\n");
            if (excess_tri != NULL) prc_free(ctx, excess_tri);
            if (excess_slot != NULL) prc_free(ctx, excess_slot);
            ret = PRC_ERROR_MEMORY;
            goto fail;
        }

        for (i = 0; i < clean_tris; i++)
        {
            uint32_t e;
            for (e = 0; e < 3; e++)
            {
                uint32_t a = out->tri_indices[(size_t)i * 3 + e];
                uint32_t b = out->tri_indices[(size_t)i * 3 + ((e + 1) % 3)];
                uint32_t v0 = a < b ? a : b;
                uint32_t v1 = a < b ? b : a;
                size_t slot = (size_t)(prc_mix64(((uint64_t)v0 << 32) | (uint64_t)v1) & (uint64_t)(ecap - 1));

                for (;;)
                {
                    prc_edge_slot *s = &etable[slot];
                    if (!s->used)
                    {
                        s->used = 1;
                        s->v0 = v0;
                        s->v1 = v1;
                        s->edge_index = nedges;
                        out->edges[nedges].v0 = v0;
                        out->edges[nedges].v1 = v1;
                        out->edges[nedges].tri0 = (int32_t)i;
                        out->edges[nedges].tri1 = -1;
                        nedges++;
                        break;
                    }
                    if (s->v0 == v0 && s->v1 == v1)
                    {
                        if (out->edges[s->edge_index].tri1 == -1)
                        {
                            out->edges[s->edge_index].tri1 = (int32_t)i;
                        }
                        else
                        {
                            /* Third (or later) triangle on this edge: queue
                               it for private-vertex remapping below, leave
                               the first two exactly as linked. */
                            excess_tri[num_excess] = i;
                            excess_slot[num_excess] = e;
                            num_excess++;
                            if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
                            {
                                prc_nonmanifold_edge_count_diag++;
                                printf("PRC_DIAG_MESH_QUALITY: nonmanifold edge v0=%u v1=%u tri0=%d tri1=%d extra_tri=%u "
                                    "(queued for private-vertex remap) orig_idx tri0=%d tri1=%d extra_tri=%u\n",
                                    v0, v1, out->edges[s->edge_index].tri0, out->edges[s->edge_index].tri1, i,
                                    out->edges[s->edge_index].tri0 >= 0 ? (int)out->tri_orig_index[out->edges[s->edge_index].tri0] : -1,
                                    out->edges[s->edge_index].tri1 >= 0 ? (int)out->tri_orig_index[out->edges[s->edge_index].tri1] : -1,
                                    out->tri_orig_index[i]);
                                if (prc_diag_getenv("PRC_DIAG_DUMP_NONMANIFOLD_REGION") != NULL)
                                {
                                    printf("PRC_DIAG_DUMP_REGION: edge v0=%u pos=(%.17g,%.17g,%.17g) v1=%u pos=(%.17g,%.17g,%.17g)\n",
                                        v0, out->positions[(size_t)v0 * 3 + 0], out->positions[(size_t)v0 * 3 + 1], out->positions[(size_t)v0 * 3 + 2],
                                        v1, out->positions[(size_t)v1 * 3 + 0], out->positions[(size_t)v1 * 3 + 1], out->positions[(size_t)v1 * 3 + 2]);
                                }
                            }
                        }
                        break;
                    }
                    slot = (slot + 1) & (ecap - 1);
                }
            }
        }

        if (num_excess > 0 && !skip_nonmanifold_edge_remap)
        {
            /* For each queued (triangle, edge-slot) pair, give that
               triangle two brand-new vertices standing in for the shared
               edge's own two endpoints (i0/i1 below), instead of reusing
               the original shared ones -- everything else about the
               triangle (its third, "apex" vertex) is untouched. */
            /* Offset magnitude: well above the resolved encoding tolerance
               (so the quantized point_array value is genuinely distinct,
               not silently re-merged by quantization), far below anything
               visually perceptible on real geometry. */
            double offset_mag = tol * 50.0;
            double *grown;
            uint32_t new_count = out->num_positions;

            grown = (double *)prc_realloc(ctx, out->positions,
                ((size_t)out->num_positions + (size_t)num_excess * 2) * 3 * sizeof(double));
            if (grown == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess non-manifold remap\n");
                prc_free(ctx, excess_tri);
                prc_free(ctx, excess_slot);
                ret = PRC_ERROR_MEMORY;
                goto fail;
            }
            out->positions = grown;

            for (i = 0; i < num_excess; i++)
            {
                uint32_t ti = excess_tri[i];
                uint32_t e = excess_slot[i];
                uint32_t i0 = out->tri_indices[(size_t)ti * 3 + e];
                uint32_t i1 = out->tri_indices[(size_t)ti * 3 + ((e + 1) % 3)];
                uint32_t apex = out->tri_indices[(size_t)ti * 3 + ((e + 2) % 3)];
                double *p0 = &out->positions[(size_t)i0 * 3];
                double *p1 = &out->positions[(size_t)i1 * 3];
                double *papex = &out->positions[(size_t)apex * 3];
                double mid[3], dir[3], len;
                uint32_t d;
                uint32_t new_i0, new_i1;

                for (d = 0; d < 3; d++) mid[d] = (p0[d] + p1[d]) * 0.5;
                for (d = 0; d < 3; d++) dir[d] = papex[d] - mid[d];
                len = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
                if (len > 1e-300)
                    for (d = 0; d < 3; d++) dir[d] /= len;
                else
                    { dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0; } /* degenerate apex-at-midpoint fallback */

                new_i0 = new_count++;
                new_i1 = new_count++;
                for (d = 0; d < 3; d++) out->positions[(size_t)new_i0 * 3 + d] = p0[d] + dir[d] * offset_mag;
                for (d = 0; d < 3; d++) out->positions[(size_t)new_i1 * 3 + d] = p1[d] + dir[d] * offset_mag;

                out->tri_indices[(size_t)ti * 3 + e] = new_i0;
                out->tri_indices[(size_t)ti * 3 + ((e + 1) % 3)] = new_i1;
            }
            out->num_positions = new_count;

            /* Rebuild edge adjacency from scratch: the remapped triangles
               no longer reference the over-shared edge at all, so a clean
               second pass is simplest and cheapest (no in-place edge-table
               surgery needed) and self-verifying (any lingering 3+-way
               edge after remapping would show up as a repeat of this same
               diagnostic on the next iteration were this looped, though it
               never should since every excess occurrence got its own new
               vertices). */
            memset(etable, 0, ecap * sizeof(prc_edge_slot));
            nedges = prc_encode_rebuild_edges(etable, ecap, out->tri_indices, clean_tris, out->edges);
        }

        out->num_edges = nedges;
        if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
        {
            uint32_t boundary_edges = 0, ei;
            for (ei = 0; ei < nedges; ei++)
                if (out->edges[ei].tri1 == -1)
                    boundary_edges++;
            printf("PRC_DIAG_MESH_QUALITY: num_edges=%u boundary_edges=%u nonmanifold_occurrences_remapped=%u final_num_positions=%u\n",
                nedges, boundary_edges, prc_nonmanifold_edge_count_diag, out->num_positions);
        }
        prc_free(ctx, excess_tri);
        prc_free(ctx, excess_slot);

        /* Pre-split edge-connected components (2026-07-31, davidgbarnes.stl
           Acrobat blank-tree investigation): computed on the mesh as it
           stands BEFORE the non-manifold-vertex fix below runs, purely to
           let that fix distinguish a GENUINE bowtie (multiple fans meeting
           at a vertex that are ALSO connected via some other edge-path
           elsewhere -- a real defect within one intended surface) from two
           (or more) otherwise entirely independent mesh pieces that happen
           to touch at exactly one point (perfectly valid topology, which
           PRC's point_reference_array mechanism -- already correctly
           implemented in the encode traversal's own vtx_map reference logic
           -- represents correctly without any split at all). Same
           union-find-over-shared-edges algorithm as the later, real
           out->tri_component computation below, just run earlier and
           discarded once the vertex-split loop is done with it -- edges
           only connect two triangles when they share a genuine 2-vertex
           edge (never a lone shared vertex), so two fans meeting ONLY at
           one non-manifold vertex are, by construction, in different
           components here unless they reconnect via some other path. */
        {
            uint32_t *presplit_parent = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            uint32_t *presplit_label = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            presplit_tri_component = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            if (presplit_parent == NULL || presplit_label == NULL || presplit_tri_component == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess pre-split components\n");
                prc_free(ctx, presplit_parent);
                prc_free(ctx, presplit_label);
                prc_free(ctx, presplit_tri_component);
                presplit_tri_component = NULL;
                ret = PRC_ERROR_MEMORY;
                goto fail;
            }
            for (i = 0; i < clean_tris; i++)
            {
                presplit_parent[i] = i;
                presplit_label[i] = UINT32_MAX;
            }
            for (i = 0; i < nedges; i++)
            {
                if (out->edges[i].tri1 != -1)
                {
                    uint32_t ra = prc_uf_find(presplit_parent, (uint32_t)out->edges[i].tri0);
                    uint32_t rb = prc_uf_find(presplit_parent, (uint32_t)out->edges[i].tri1);
                    if (ra != rb)
                        presplit_parent[ra] = rb;
                }
            }
            {
                uint32_t presplit_ncomp = 0;
                for (i = 0; i < clean_tris; i++)
                {
                    uint32_t root = prc_uf_find(presplit_parent, i);
                    if (presplit_label[root] == UINT32_MAX)
                    {
                        presplit_label[root] = presplit_ncomp;
                        presplit_ncomp++;
                    }
                    presplit_tri_component[i] = presplit_label[root];
                }
            }
            prc_free(ctx, presplit_parent);
            prc_free(ctx, presplit_label);
        }

        /* Non-manifold VERTEX fix (2026-07-23): a DIFFERENT defect class
           than the non-manifold EDGE case above -- a vertex is non-
           manifold if the triangles touching it don't form a single
           connected "fan" (each consecutive pair sharing an edge through
           that vertex) but rather two or more separate fans meeting only
           at that one point (a "bowtie"). Confirmed via a real customer
           file (UK_original.stl) that has non-manifold vertices in every
           one of its 7 connected parts and still failed in Acrobat after
           the edge fix and the sliver-triangle filter above were both
           already active -- this was previously only diagnosed (read-only
           census), never fixed. Same general remedy as the edge case: keep
           the first fan on the original vertex, give every OTHER fan its
           own private, imperceptibly-offset copy of that vertex's
           position, remapping only that fan's triangles' corner(s) at this
           vertex. Uses a per-vertex incident-edge index (not a scan over
           ALL edges per vertex) so this stays O(V+E) and scales to
           multi-million-triangle meshes. */
        if (out->num_positions > 0 && clean_tris > 0)
        {
            uint32_t *vtri_count = (uint32_t *)prc_calloc(ctx, out->num_positions, sizeof(uint32_t));
            uint32_t *vtri_start = (uint32_t *)prc_calloc(ctx, (size_t)out->num_positions + 1, sizeof(uint32_t));
            uint32_t *vtri_list = NULL;
            uint32_t *vedge_count = (uint32_t *)prc_calloc(ctx, out->num_positions, sizeof(uint32_t));
            uint32_t *vedge_start = (uint32_t *)prc_calloc(ctx, (size_t)out->num_positions + 1, sizeof(uint32_t));
            uint32_t *vedge_list = NULL;
            uint32_t *vparent = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            uint32_t *vfan_new_vertex = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            /* Global-triangle-id -> local-fan-index lookup, reused across
               every vertex via a generation stamp instead of being cleared
               each time (clearing clean_tris entries per vertex would
               itself be O(V*T)). Confirmed necessary (2026-07-27): without
               this, the edge-to-local-index resolution below was a linear
               scan over the vertex's own `deg` triangles per incident edge,
               i.e. O(deg^2) per vertex -- fine for ordinary meshes but a
               genuine multi-minute hang on a real-world file
               (2368549.stream-147.stl) with heavily-duplicated geometry
               giving some vertices a triangle degree over 5000. */
            uint32_t *tri_local = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
            uint32_t *tri_local_stamp = (uint32_t *)prc_calloc(ctx, clean_tris, sizeof(uint32_t));
            uint32_t stamp = 0;
            uint32_t total = 0, etotal = 0, vi, ti2, ei2, nonmanifold_vertices = 0, splits_needed = 0;
            uint32_t corner_touches_preserved = 0;
            /* Captured BEFORE the per-vertex loop below, which grows
               out->num_positions in place as vertices get split -- the
               per-vertex triangle/edge index arrays (vtri_.. and vedge_..)
               are sized/built once, up front, for the ORIGINAL vertex
               count only; newly-split vertices never need their own fan
               analysis (they're each already a single, freshly-created,
               trivially-manifold point), so the loop bound must stay
               fixed at this snapshot, not track the live (growing)
               out->num_positions. */
            uint32_t orig_num_positions = out->num_positions;

            for (ti2 = 0; ti2 < clean_tris; ti2++)
            {
                uint32_t c;
                for (c = 0; c < 3; c++)
                    vtri_count[out->tri_indices[(size_t)ti2 * 3 + c]]++;
            }
            for (vi = 0; vi < orig_num_positions; vi++) { vtri_start[vi] = total; total += vtri_count[vi]; }
            vtri_start[orig_num_positions] = total;
            vtri_list = (uint32_t *)prc_malloc(ctx, (size_t)total * sizeof(uint32_t));
            memset(vtri_count, 0, (size_t)orig_num_positions * sizeof(uint32_t));
            for (ti2 = 0; ti2 < clean_tris; ti2++)
            {
                uint32_t c;
                for (c = 0; c < 3; c++)
                {
                    uint32_t v = out->tri_indices[(size_t)ti2 * 3 + c];
                    vtri_list[vtri_start[v] + vtri_count[v]] = ti2;
                    vtri_count[v]++;
                }
            }

            for (ei2 = 0; ei2 < out->num_edges; ei2++)
            {
                if (out->edges[ei2].tri1 == -1) continue;
                vedge_count[out->edges[ei2].v0]++;
                vedge_count[out->edges[ei2].v1]++;
            }
            for (vi = 0; vi < orig_num_positions; vi++) { vedge_start[vi] = etotal; etotal += vedge_count[vi]; }
            vedge_start[orig_num_positions] = etotal;
            vedge_list = (uint32_t *)prc_malloc(ctx, (size_t)etotal * sizeof(uint32_t));
            memset(vedge_count, 0, (size_t)orig_num_positions * sizeof(uint32_t));
            for (ei2 = 0; ei2 < out->num_edges; ei2++)
            {
                uint32_t v0e, v1e;
                if (out->edges[ei2].tri1 == -1) continue;
                v0e = out->edges[ei2].v0; v1e = out->edges[ei2].v1;
                vedge_list[vedge_start[v0e] + vedge_count[v0e]] = ei2; vedge_count[v0e]++;
                vedge_list[vedge_start[v1e] + vedge_count[v1e]] = ei2; vedge_count[v1e]++;
            }

            /* ---- Non-manifold "fan" detection and splitting, per vertex ----
               Algorithm, in plain terms: for every deduplicated vertex touched
               by 2+ triangles, treat those triangles as nodes in a graph and
               union-find them together whenever two of them share an EDGE
               (not just the vertex itself) that also touches this same
               vertex. The resulting connected components are that vertex's
               "fans" -- maximal groups of triangles that form one continuous
               surface patch around it. A normal, manifold vertex always ends
               up as exactly one fan (every incident triangle edge-connects
               to its neighbors in a ring). A vertex where two otherwise-
               unrelated parts of the mesh happen to touch at a single point
               (no shared edges between the two triangle groups, only this
               one shared vertex) ends up as 2+ fans -- that's the defect this
               loop finds and fixes: every fan after the first gets its own
               freshly-allocated vertex, so what was one shared, ambiguous
               point becomes N distinct points, one per genuinely-separate
               surface patch, and no reader's normal-averaging or connectivity
               logic ever has to reconcile triangles that were never really
               part of the same local surface to begin with. */
            if (vtri_count && vtri_start && vtri_list && vedge_count && vedge_start && vedge_list && vparent && vfan_new_vertex
                && tri_local && tri_local_stamp
                && prc_diag_getenv("PRC_DIAG_DISABLE_NONMANIFOLD_SPLIT") == NULL)
            {
                for (vi = 0; vi < orig_num_positions; vi++)
                {
                    uint32_t deg = vtri_start[vi + 1] - vtri_start[vi];
                    uint32_t k2, ncomp2, root0, m2;
                    if (deg < 2) continue; /* a single incident triangle can't be non-manifold on its own */
                    stamp++;
                    /* Renumber this vertex's `deg` incident triangles to local
                       indices 0..deg-1 (tri_local/tri_local_stamp double as a
                       sparse "is this global triangle index one of THIS
                       vertex's incident triangles, and if so which local
                       slot" lookup, reused/overwritten via the stamp counter
                       across every vertex in this outer loop rather than
                       cleared each time) and start each in its own
                       union-find set. */
                    for (k2 = 0; k2 < deg; k2++)
                    {
                        uint32_t t = vtri_list[vtri_start[vi] + k2];
                        tri_local[t] = k2;
                        tri_local_stamp[t] = stamp;
                        vparent[k2] = k2; /* local indices 0..deg-1 into vtri_list[vtri_start[vi]+k2] */
                    }
                    /* Union two incident triangles whenever an edge touching
                       this vertex is shared between them (ed->tri0/tri1 are
                       that edge's two adjacent triangles, or -1 if boundary).
                       Only edges actually touching vi are walked (vedge_list
                       is vi's own incident-edge list), and only pairs where
                       BOTH sides are also incident to vi this same call
                       (tri_local_stamp[...] == stamp) count -- an edge can
                       touch this vertex on one side and a completely
                       different vertex's own triangle fan on the other. */
                    for (m2 = vedge_start[vi]; m2 < vedge_start[vi + 1]; m2++)
                    {
                        const prc_encode_edge *ed = &out->edges[vedge_list[m2]];
                        int32_t ta = ed->tri0, tb = ed->tri1;
                        uint32_t la = (ta >= 0 && tri_local_stamp[ta] == stamp) ? tri_local[ta] : UINT32_MAX;
                        uint32_t lb = (tb >= 0 && tri_local_stamp[tb] == stamp) ? tri_local[tb] : UINT32_MAX;
                        if (la != UINT32_MAX && lb != UINT32_MAX)
                        {
                            uint32_t ra = prc_uf_find(vparent, la);
                            uint32_t rb = prc_uf_find(vparent, lb);
                            if (ra != rb) vparent[ra] = rb;
                        }
                    }
                    /* root0 = the fan containing local triangle 0, arbitrarily
                       chosen as "the one that keeps the original vertex" (see
                       the split loop below -- every OTHER fan gets a new
                       vertex instead). If every incident triangle unions back
                       to root0, there's only one fan: an ordinary manifold
                       vertex, nothing to split. */
                    root0 = prc_uf_find(vparent, 0);
                    ncomp2 = 1;
                    for (k2 = 1; k2 < deg; k2++)
                        if (prc_uf_find(vparent, k2) != root0) { ncomp2 = 2; break; }
                    if (ncomp2 < 2) continue;

                    nonmanifold_vertices++;
                    if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
                        printf("PRC_DIAG_MESH_QUALITY: nonmanifold VERTEX v=%u incident_triangles=%u (multiple disconnected fans, splitting)\n",
                            vi, deg);
                    /* Diagnostic-only, separately gated (verbose): dump this
                       vertex's own position and every incident triangle's
                       full geometry, enough to reconstruct a minimal
                       synthetic repro of just this fix site by hand. */
                    if (prc_diag_getenv("PRC_DIAG_DUMP_NONMANIFOLD_REGION") != NULL)
                    {
                        uint32_t k4;
                        printf("PRC_DIAG_DUMP_REGION: vertex v=%u pos=(%.17g,%.17g,%.17g)\n", vi,
                            out->positions[(size_t)vi * 3 + 0], out->positions[(size_t)vi * 3 + 1],
                            out->positions[(size_t)vi * 3 + 2]);
                        for (k4 = 0; k4 < deg; k4++)
                        {
                            uint32_t t = vtri_list[vtri_start[vi] + k4];
                            uint32_t a = out->tri_indices[(size_t)t * 3 + 0];
                            uint32_t b = out->tri_indices[(size_t)t * 3 + 1];
                            uint32_t c = out->tri_indices[(size_t)t * 3 + 2];
                            printf("PRC_DIAG_DUMP_REGION:   tri=%u orig_idx=%u fan_root=%u verts=(%u,%u,%u) "
                                "p0=(%.17g,%.17g,%.17g) p1=(%.17g,%.17g,%.17g) p2=(%.17g,%.17g,%.17g)\n",
                                t, out->tri_orig_index[t], prc_uf_find(vparent, k4), a, b, c,
                                out->positions[(size_t)a * 3 + 0], out->positions[(size_t)a * 3 + 1], out->positions[(size_t)a * 3 + 2],
                                out->positions[(size_t)b * 3 + 0], out->positions[(size_t)b * 3 + 1], out->positions[(size_t)b * 3 + 2],
                                out->positions[(size_t)c * 3 + 0], out->positions[(size_t)c * 3 + 1], out->positions[(size_t)c * 3 + 2]);
                        }
                    }

                    /* Assign each LOCAL triangle a "new vertex" slot per
                       fan-root, except the fan containing local index 0
                       (kept on the original vertex vi). Fan roots are
                       discovered on first sight; UINT32_MAX marks
                       "not yet assigned"/"keep original". */
                    for (k2 = 0; k2 < deg; k2++) vfan_new_vertex[k2] = UINT32_MAX;
                    for (k2 = 0; k2 < deg; k2++)
                    {
                        uint32_t root = prc_uf_find(vparent, k2);
                        if (root == root0) continue; /* stays on original vertex */
                        /* Corner-touch check (2026-07-31, davidgbarnes.stl Acrobat blank-tree
                           investigation): this fan and fan 0 meet ONLY at this one vertex --
                           genuinely non-manifold by definition. But if they're not otherwise
                           edge-connected ANYWHERE ELSE in the mesh either (different pre-split
                           components), they're two independent pieces that happen to touch at
                           exactly one point, not a bowtie defect within one intended surface.
                           That's valid topology PRC's point_reference_array already represents
                           correctly (see prc_encode_chain_start's vtx_map reference logic) --
                           leave this fan on the original vertex instead of splitting, exactly
                           like fan 0, so the traversal below references it instead of emitting
                           (and Acrobat then choking on) a spurious duplicate point. Only fans
                           that reconnect to fan 0 elsewhere (same pre-split component -- a
                           genuine bowtie) still get split. */
                        if (presplit_tri_component != NULL)
                        {
                            uint32_t this_tri = vtri_list[vtri_start[vi] + k2];
                            uint32_t fan0_tri = vtri_list[vtri_start[vi] + 0];
                            if (presplit_tri_component[this_tri] != presplit_tri_component[fan0_tri])
                            {
                                corner_touches_preserved++;
                                continue;
                            }
                        }
                        if (vfan_new_vertex[root] == UINT32_MAX)
                        {
                            /* Allocate a new position lazily, in place, by
                               growing out->positions one vertex at a time
                               -- num_positions can grow arbitrarily here so
                               a single up-front bound isn't practical;
                               realloc growth is amortized by the caller's
                               allocator. */
                            double *pv = &out->positions[(size_t)vi * 3];
                            double *grown2 = (double *)prc_realloc(ctx, out->positions,
                                ((size_t)out->num_positions + 1) * 3 * sizeof(double));
                            uint32_t newv;
                            double offset_mag2 = tol * 50.0;
                            uint32_t d2;
                            /* DIAGNOSTIC (2026-07-26, mixed_chains investigation):
                               override the non-manifold vertex split offset
                               magnitude (as a multiple of tol) to test whether
                               the split's SIZE matters to the Acrobat blank
                               -tree bug, independent of the split MECHANISM
                               itself. PRC_DIAG_SPLIT_OFFSET_MULT=N. */
                            {
                                const char *ov = prc_diag_getenv("PRC_DIAG_SPLIT_OFFSET_MULT");
                                if (ov != NULL)
                                    offset_mag2 = tol * strtod(ov, NULL);
                            }
                            if (grown2 == NULL)
                            {
                                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess non-manifold vertex split\n");
                                prc_free(ctx, vtri_count); prc_free(ctx, vtri_start); prc_free(ctx, vtri_list);
                                prc_free(ctx, vedge_count); prc_free(ctx, vedge_start); prc_free(ctx, vedge_list);
                                prc_free(ctx, vparent); prc_free(ctx, vfan_new_vertex);
                                if (tri_local != NULL) prc_free(ctx, tri_local);
                                if (tri_local_stamp != NULL) prc_free(ctx, tri_local_stamp);
                                ret = PRC_ERROR_MEMORY;
                                goto fail;
                            }
                            out->positions = grown2;
                            pv = &out->positions[(size_t)vi * 3]; /* re-fetch: realloc may have moved the buffer */
                            newv = out->num_positions;
                            out->num_positions++;
                            /* Offset toward this fan's own first triangle's
                               centroid -- keeps the split imperceptible and
                               avoids an arbitrary/shared direction across
                               fans. */
                            {
                                uint32_t rep_tri = vtri_list[vtri_start[vi] + k2];
                                double cen[3], dir2[3], len2;
                                uint32_t cc;
                                cen[0] = cen[1] = cen[2] = 0.0;
                                for (cc = 0; cc < 3; cc++)
                                {
                                    uint32_t vv = out->tri_indices[(size_t)rep_tri * 3 + cc];
                                    uint32_t dd;
                                    for (dd = 0; dd < 3; dd++) cen[dd] += out->positions[(size_t)vv * 3 + dd] / 3.0;
                                }
                                for (d2 = 0; d2 < 3; d2++) dir2[d2] = cen[d2] - pv[d2];
                                len2 = sqrt(dir2[0]*dir2[0] + dir2[1]*dir2[1] + dir2[2]*dir2[2]);
                                if (len2 > 1e-300)
                                    for (d2 = 0; d2 < 3; d2++) dir2[d2] /= len2;
                                else
                                    { dir2[0] = 1.0; dir2[1] = 0.0; dir2[2] = 0.0; }
                                for (d2 = 0; d2 < 3; d2++)
                                    out->positions[(size_t)newv * 3 + d2] = pv[d2] + dir2[d2] * offset_mag2;
                            }
                            vfan_new_vertex[root] = newv;
                            splits_needed++;
                        }
                        {
                            uint32_t t = vtri_list[vtri_start[vi] + k2];
                            uint32_t cc;
                            for (cc = 0; cc < 3; cc++)
                                if (out->tri_indices[(size_t)t * 3 + cc] == vi)
                                    out->tri_indices[(size_t)t * 3 + cc] = vfan_new_vertex[root];
                        }
                    }
                }
                if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
                    printf("PRC_DIAG_MESH_QUALITY: nonmanifold_vertices=%u vertex_splits=%u corner_touches_preserved=%u (out of %u positions before splitting)\n",
                        nonmanifold_vertices, splits_needed, corner_touches_preserved, orig_num_positions);
            }
            out->nonmanifold_vertices = nonmanifold_vertices;
            if (vtri_count != NULL) prc_free(ctx, vtri_count);
            if (vtri_start != NULL) prc_free(ctx, vtri_start);
            if (vtri_list != NULL) prc_free(ctx, vtri_list);
            if (vedge_count != NULL) prc_free(ctx, vedge_count);
            if (vedge_start != NULL) prc_free(ctx, vedge_start);
            if (vedge_list != NULL) prc_free(ctx, vedge_list);
            if (vparent != NULL) prc_free(ctx, vparent);
            if (vfan_new_vertex != NULL) prc_free(ctx, vfan_new_vertex);
            if (tri_local != NULL) prc_free(ctx, tri_local);
            if (tri_local_stamp != NULL) prc_free(ctx, tri_local_stamp);

            if (nonmanifold_vertices > 0)
            {
                /* Vertex splitting changed tri_indices (and possibly grew
                   out->positions), so out->edges must be rebuilt from
                   scratch once more before the connected-components pass
                   below reads it. out->edges' allocation (max_edges =
                   clean_tris*3, an UPPER bound independent of vertex
                   count) is still valid and sufficient. */
                memset(etable, 0, ecap * sizeof(prc_edge_slot));
                nedges = prc_encode_rebuild_edges(etable, ecap, out->tri_indices, clean_tris, out->edges);
                out->num_edges = nedges;
            }
        }

        if ((size_t)nedges < max_edges)
        {
            prc_encode_edge *shrunk = (prc_encode_edge *)prc_realloc(ctx, out->edges, (size_t)nedges * sizeof(prc_encode_edge));
            if (shrunk != NULL)
                out->edges = shrunk;
        }
        prc_free(ctx, etable);
        etable = NULL;
    }

    if (clean_tris > 0)
    {
        uint32_t ncomp = 0;

        parent = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
        label = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
        out->tri_component = (uint32_t *)prc_malloc(ctx, (size_t)clean_tris * sizeof(uint32_t));
        if (parent == NULL || label == NULL || out->tri_component == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_preprocess components\n");
            ret = PRC_ERROR_MEMORY;
            goto fail;
        }

        for (i = 0; i < clean_tris; i++)
        {
            parent[i] = i;
            label[i] = UINT32_MAX;
        }
        for (i = 0; i < out->num_edges; i++)
        {
            if (out->edges[i].tri1 != -1)
            {
                uint32_t ra = prc_uf_find(parent, (uint32_t)out->edges[i].tri0);
                uint32_t rb = prc_uf_find(parent, (uint32_t)out->edges[i].tri1);
                if (ra != rb)
                    parent[ra] = rb;
            }
        }
        for (i = 0; i < clean_tris; i++)
        {
            uint32_t root = prc_uf_find(parent, i);
            if (label[root] == UINT32_MAX)
            {
                label[root] = ncomp;
                ncomp++;
            }
            out->tri_component[i] = label[root];
        }
        out->num_components = ncomp;

        if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
            printf("PRC_DIAG_MESH_QUALITY: num_components=%u clean_tris=%u num_positions=%u\n",
                ncomp, clean_tris, out->num_positions);

        prc_free(ctx, parent);
        parent = NULL;
        prc_free(ctx, label);
        label = NULL;
    }

    prc_free(ctx, presplit_tri_component);
    presplit_tri_component = NULL;

    return 0;

fail:
    if (vtable != NULL)
        prc_free(ctx, vtable);
    if (etable != NULL)
        prc_free(ctx, etable);
    if (remap != NULL)
        prc_free(ctx, remap);
    if (parent != NULL)
        prc_free(ctx, parent);
    if (label != NULL)
        prc_free(ctx, label);
    if (presplit_tri_component != NULL)
        prc_free(ctx, presplit_tri_component);
    prc_encode_preprocess_free(ctx, out);
    return ret;
}

/* Every existing caller (COMPRESSED encoding, where an edge shared by 3+
   triangles genuinely must be resolved -- the EdgeBreaker-style traversal
   below assumes at most 2 triangles per edge) wants the non-manifold-edge
   remap; only prc_api_mesh_weld_and_split (an uncompressed-TRIANGLES-
   oriented caller, which has no such traversal and so no need for that
   specific cleanup -- see its own doc comment) opts out via
   prc_encode_preprocess_ex directly. */
int
prc_encode_preprocess(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance,
    prc_encode_mesh *out)
{
    return prc_encode_preprocess_ex(ctx, positions, num_positions, tri_indices, num_triangles,
        tolerance, 0, out);
}

/* Depth-first traversal (Step B): emits the compressed-tessellation arrays by
   simulating, in lockstep, exactly what prc_decode_compressed_tess will do
   when it reads them back. All predictions (chain-start deltas, edge bases,
   averages) are computed from the decoder-visible RECONSTRUCTED positions --
   not the original input positions -- using the same prc_vec_* helpers in the
   same order, so encoder and decoder stay bit-for-bit synchronized. */

#define PRC_ENCODE_MAX_CHAIN 65536

/* TEMPORARY DIAGNOSTIC (2026-07-22, remove after the current Acrobat blank-
   tree investigation concludes -- see project memory): runtime override for
   PRC_ENCODE_MAX_CHAIN via PRC_DIAG_MAX_CHAIN env var, to test the SAME
   mesh topology at different chain-fragmentation patterns without varying
   mesh size (the technique that isolated the origin-truncation bug in an
   earlier session of this same investigation). Falls back to the compile-
   time constant when unset or invalid. */
static uint32_t
prc_encode_max_chain(void)
{
    static int cached = 0;
    static uint32_t value = PRC_ENCODE_MAX_CHAIN;
    if (!cached)
    {
        const char *env = prc_diag_getenv("PRC_DIAG_MAX_CHAIN");
        if (env != NULL)
        {
            long v = strtol(env, NULL, 10);
            if (v > 0)
                value = (uint32_t)v;
        }
        cached = 1;
    }
    return value;
}

typedef struct
{
    prc_vec3 x_basis, y_basis, z_basis;
    prc_vec3 origin;
    int32_t index0, index1;   /* decoder point indices of the edge, index0 < index1 */
    uint32_t mesh_v0, mesh_v1; /* mesh vertex ids aligned with index0/index1 */
    uint32_t target_tri;      /* mesh triangle to grow across this edge */
    /* This child's inherited tri_reversed value -- the parent's own value,
       unchanged, for both right- and left-grown children alike. See the
       long comment at this field's write site (prc_encode_edge_status) for
       why: normal_was_reversed is constant along an unbroken growth chain,
       confirmed empirically 2026-08-10. Used directly as the child's own
       tri_reversed when it's later popped (prc_encode_traversal's main
       loop) -- no independent per-triangle geometric decision for growing
       triangles anymore. */
    uint8_t inherited_reversed;
} prc_encode_grow_op;

typedef struct
{
    prc_context *ctx;
    const prc_encode_mesh *mesh;
    double tol;
    prc_vec3 origin;
    int32_t *vtx_map;         /* mesh vertex -> decoder point index, -1 unseen */
    prc_vec3 *decoded_pos;    /* decoder-exact reconstructed positions */
    /* PRC_DIAG_MESH_QUALITY reporting only: number of un-broken relative-encoding
       steps separating each decoder point from the nearest origin-anchored point
       (0 = a chain-start's own V0, encoded directly against the global origin;
       every other point's hop_depth = 1 + the max hop_depth of whatever point(s)
       its own encoding was measured relative to). A point_reference_array
       reference reuses an existing point's hop_depth unchanged -- it doesn't
       create a new hop, but also doesn't reset anyone else's. Tests whether
       reconstruction error compounds with hop depth specifically (not just raw
       chain position, which conflates depth with mere triangle count). Indexed
       exactly like decoded_pos, same allocation lifetime. */
    uint32_t *hop_depth;
    uint32_t n_points;
    uint8_t *visited;
    uint8_t *pending;         /* triangle has a live grow op on the stack */
    int32_t *neighbor;        /* 3 per triangle (local edge slots), -1 = boundary */
    /* Explicit heap stack for the depth-first growth (mesh-size-driven depth
       must never use C call-stack recursion). */
    prc_encode_grow_op *stack;
    uint32_t stack_size;
    uint32_t stack_capacity;
    uint32_t chain_len;
    uint32_t current_chain;   /* 0-based id of the chain being grown */
    uint32_t chain_offset;    /* points created so far within the current chain */
    prc_vertex_analysis *analysis; /* NULL unless the caller requested capture */
    prc_encode_traversal_result *out;
    /* mesh.num_triangles entries, mesh order, owned by prc_encode_traversal;
       NULL (real_normals == NULL) == no triangle ever reversed. Filled in
       PROGRESSIVELY, one entry per triangle, at the moment each triangle's
       idx[]/mv[] are finalized in the main loop -- NOT precomputed up front
       -- so prc_encode_edge_status always reads a value decided from that
       triangle's own true final vertex order, never a stale baseline's (see
       prc_encode_traversal's header comment for why that distinction is the
       whole point of this design). tri_reversed[t] mirrors the decoder's
       prc_set_left_right_edge_indices: when true, triangle t's right/left
       edge roles (and therefore which edge's basis/grow-op gets pushed as
       "right" vs "left") are swapped relative to the un-reversed convention
       below, so encoder and decoder agree on which physical edge each
       edge_status_array bit refers to. */
    uint8_t *tri_reversed;
    const double *real_normals; /* mesh->num_triangles*9 entries (3 per corner, MESH order,
                                    same layout as corner_normals); NULL == disabled */
    /* PRC_DIAG_MESH_QUALITY reporting only, see prc_encode_refuse_alternate_basis_grow's
       own comment: how often a grow-op's basis needed the ambiguous fallback, and how
       many of those were refused (become chain starts instead) vs. grown anyway. */
    uint32_t alt_basis_count;
    uint32_t alt_basis_refused_count;
    /* PRC_DIAG_MESH_QUALITY reporting only: per chain-start triangle (the
       num_refs==0 case, RESULT 12's V0+V2 mechanism), how far the
       RECONSTRUCTED (decoded) triangle normal tilts away from the TRUE
       (original mesh, full double precision) normal -- 1-dot(true,
       reconstructed), 0 = perfect match. A direct, orientation-independent
       generalization of the normal-tilt signal RESULT 12's own manual
       cross-product analysis found correlated with the V0+V2 failure mode
       on a minimal synthetic repro; this measures it automatically across
       every chain-start triangle in a real file. */
    uint32_t chain_start_count;
    double chain_start_tilt_max;
    uint32_t chain_start_tilt_gt_1e_6;
    uint32_t chain_start_tilt_gt_1e_4;
    uint32_t chain_start_tilt_gt_1e_2;
    /* PRC_DIAG_MESH_QUALITY reporting only: longest single chain (grow-run) ever
       reached, to check whether an unusually long chain (as opposed to just an
       unusual chain COUNT) correlates with anything. */
    uint32_t max_chain_len;
    /* PRC_DIAG_MESH_QUALITY reporting only: per-triangle (ALL triangles, not just
       chain-starts) true-vs-reconstructed normal tilt, bucketed by how deep into
       its own chain the triangle sits -- tests whether reconstruction error
       accumulates/drifts progressively along a very long chain (each grow-step's
       basis is built from the PREVIOUS step's already-quantized position, so a
       small per-step bias could compound over hundreds of thousands of steps in
       a way RESULT 12's one-shot chain-start-only V0+V2 mechanism never captured). */
    double alltri_tilt_max;
    uint32_t alltri_tilt_max_tri;
    uint32_t alltri_tilt_max_offset;
    double bucket_tilt_max[6]; /* offset ranges: [0,10) [10,100) [100,1000) [1000,10000) [10000,100000) [100000,inf) */
    /* PRC_DIAG_MESH_QUALITY reporting only: same tilt data, bucketed by hop_depth
       instead of raw chain offset -- isolates "how many un-broken relative-
       encoding steps from an origin anchor" from "how many triangles into the
       chain", which conflate whenever references are sparse (see hop_depth's
       own comment). Buckets: 0-1, 2-3, 4-7, 8-15, 16-31, 32+. */
    double hopbucket_tilt_max[6];
    uint32_t hopbucket_count[6];
    /* PRC_DIAG_MESH_QUALITY reporting only: max deviation from perfect orthonormality
       (|dot(x,y)|, |dot(y,z)|, |dot(x,z)|, ||x|-1|, ||y|-1|, ||z|-1|) of every grow-op
       basis actually used, bucketed by the growing edge's own hop_depth. Tests whether
       the basis itself measurably loses orthonormality with depth -- a truly orthonormal
       transform can't amplify error, so if this stays flat, error growth must come from
       somewhere else (e.g. the quantization/re-snap step itself), not basis drift. */
    double orthobucket_dev_max[6];
    uint32_t orthobucket_count[6];
    /* PRC_DIAG_MESH_QUALITY reporting only: raw per-point positional reconstruction
       error (|true - decoded|, not normal tilt), bucketed by chain_offset, computed
       directly where both are already in hand (emit_axis_point/emit_basis_point) --
       cheaper and more direct than the per-triangle tilt scan, isolates whether
       POSITION error itself grows with depth (independent of any triangle-shape
       amplification). */
    double posbucket_err_max[6];
    uint32_t posbucket_count[6];
    /* PRC_DIAG_DUMP_ALLTRI=<path>: opened/closed around the whole traversal
       when set, one line per triangle written from the same per-triangle
       tilt-scan block that already computes true/decoded positions and
       hop_depth -- lets an offline script do a DISTRIBUTIONAL (not
       single-anecdote) out-of-plane-vs-in-plane error decomposition across
       every triangle in a real file, mirroring the decode side's own
       PRC_DIAG_DUMP_HOP_DEPTH. */
    FILE *alltri_dump;
    /* PROBE (2026-08-06): ground-truth reference count per mesh triangle (NOT
       an hop_depth-arithmetic proxy) -- chain_start sets this to num_refs
       (0-3, how many of its 3 corners were st->vtx_map hits); grow_triangle
       sets it to 0 or 1 (whether its one new corner resolved via
       st->vtx_map, the exact same condition that drives
       out->points_is_reference_array for that step). Lets an offline script
       classify "did this triangle involve a real point_reference_array
       reference" directly instead of inferring it from a hop_depth pattern
       (which conflates ordinary active-boundary continuation with genuine
       reference reuse). Included in the PRC_DIAG_DUMP_ALLTRI dump. */
    uint8_t *tri_is_ref;
    /* PROBE (2026-08-06): the grow-op edge-basis z_basis actually used to
       decode this triangle's apex, captured at pop/grow time (chain-start
       triangles, which don't use an edge basis, leave this zeroed -- a real
       basis is always unit length, so (0,0,0) is an unambiguous "not set"
       sentinel). Lets an offline script measure step-to-step ORIENTATION
       stability of the local quantization frame (distinct from the
       orthonormality/VALIDITY check orthobucket_dev_max already does) by
       comparing consecutive grow-triangles' z_basis directions -- included
       in the PRC_DIAG_DUMP_ALLTRI dump. */
    prc_vec3 *tri_zbasis;
} prc_encode_state;

/* Chain bookkeeping only this phase: reconstructed_position stays zeroed
   until the reconstruction pass lands. Must run before n_points advances
   (the entry index is the point being created). */
static void
prc_encode_record_analysis(prc_encode_state *st, uint32_t mesh_vtx)
{
    if (st->analysis != NULL)
    {
        prc_vertex_analysis *a = &st->analysis[st->n_points];
        const double *p = st->mesh->positions + (size_t)mesh_vtx * 3;

        a->original_position[0] = (float)p[0];
        a->original_position[1] = (float)p[1];
        a->original_position[2] = (float)p[2];
        a->reconstructed_position[0] = 0.0f;
        a->reconstructed_position[1] = 0.0f;
        a->reconstructed_position[2] = 0.0f;
        a->chain_index = st->current_chain;
        a->chain_offset = st->chain_offset;
    }
    st->chain_offset++;
}

static int
prc_encode_quantize(prc_context *ctx, double v, double tol, int32_t *dv)
{
    long long q = llround(v / tol);

    if (q > (long long)INT32_MAX || q < (long long)INT32_MIN)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: delta exceeds int32 range\n");
        return PRC_ERROR_INTERNAL;
    }
    *dv = (int32_t)q;
    return 0;
}

/* PROBE (2026-08-06, PRC_DIAG_NUDGE_ZERO_DELTAS): targeted causal test for
   whether point_array deltas with 2+ exact-zero components (RESULT 12's
   original V0+V2 chain-start trigger pattern, generalized here to EVERY
   point role, not just chain-start -- beetle_1000000.stl's own jitter-off
   encode shows a real point_array-wide 2+-zero-component rate roughly 2x
   higher than jitter-on) are the actual causal mechanism, independent of
   full-mesh jitter's many OTHER side effects (Huffman table shape included
   -- already tested and falsified separately). Nudges just enough of the
   zero components (by the minimal possible perturbation, 1 quantization
   unit) to bring any point's zero-count down to at most 1, leaving every
   other point_array value, and the resulting Huffman table, as close to the
   unperturbed encode as possible. Zero cost/behavior change when unset. */
static void
prc_diag_nudge_zero_delta(int32_t dv[3])
{
    if (prc_diag_getenv("PRC_DIAG_NUDGE_ZERO_DELTAS") == NULL)
        return;
    {
        int zeros = (dv[0] == 0) + (dv[1] == 0) + (dv[2] == 0);
        int k;
        if (zeros < 2)
            return;
        for (k = 0; k < 3 && zeros > 1; k++)
        {
            if (dv[k] == 0)
            {
                dv[k] = 1;
                zeros--;
            }
        }
    }
}

/* PROBE (2026-08-06): see posbucket_err_max's own comment on prc_encode_state. */
static void
prc_diag_track_pos_error(prc_encode_state *st, const double *p, prc_vec3 rec)
{
    if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
    {
        double dx = p[0] - rec.x, dy = p[1] - rec.y, dz = p[2] - rec.z;
        double err = sqrt(dx * dx + dy * dy + dz * dz);
        uint32_t offset = st->chain_offset;
        int bucket = offset < 10 ? 0 : offset < 100 ? 1 : offset < 1000 ? 2 :
            offset < 10000 ? 3 : offset < 100000 ? 4 : 5;
        if (err > st->posbucket_err_max[bucket])
            st->posbucket_err_max[bucket] = err;
        st->posbucket_count[bucket]++;
    }
}

/* Emit a new point predicted along the global axes: DV = round((P - base)/tol),
   reconstructed exactly like the decoder's prc_vec_add(point_array_scaled, base). */
static int
prc_encode_emit_axis_point(prc_encode_state *st, uint32_t mesh_vtx, prc_vec3 base,
    int32_t *out_index)
{
    const double *p = st->mesh->positions + (size_t)mesh_vtx * 3;
    prc_vec3 scaled, rec;
    int32_t dv[3];
    int code;

    code = prc_encode_quantize(st->ctx, p[0] - base.x, st->tol, &dv[0]);
    if (code < 0)
        return code;
    code = prc_encode_quantize(st->ctx, p[1] - base.y, st->tol, &dv[1]);
    if (code < 0)
        return code;
    code = prc_encode_quantize(st->ctx, p[2] - base.z, st->tol, &dv[2]);
    if (code < 0)
        return code;

    prc_diag_nudge_zero_delta(dv);

    if (st->ctx->trace_reversed)
        fprintf(stderr, "ENC_AXISPOINT mesh_vtx=%u base=(%.17g,%.17g,%.17g) p=(%.17g,%.17g,%.17g) "
            "raw_delta=(%.17g,%.17g,%.17g) tol=%.17g dv=(%d,%d,%d) n_points=%u\n",
            mesh_vtx, base.x, base.y, base.z, p[0], p[1], p[2],
            p[0] - base.x, p[1] - base.y, p[2] - base.z, st->tol,
            dv[0], dv[1], dv[2], st->n_points);

    /* DIAGNOSTIC (2026-07-26): precisely identify CHAIN-START axis points
       (base is exactly the global origin, not a neighboring decoded point)
       whose delta has 2+ exact-zero components -- the proven trigger
       condition, but only for THIS specific kind of point (mixed_chains
       investigation). Avoids the false-positive risk of scanning the whole
       point_array indiscriminately (regular grow-point deltas routinely and
       harmlessly share 2 coordinates for unrelated geometric reasons).
       Opt-in via PRC_DIAG_CHAINSTART_ZERO, zero cost/behavior change when
       unset. */
    if (prc_diag_getenv("PRC_DIAG_CHAINSTART_ZERO") != NULL &&
        base.x == st->origin.x && base.y == st->origin.y && base.z == st->origin.z)
    {
        int zeros = (dv[0] == 0) + (dv[1] == 0) + (dv[2] == 0);
        if (zeros >= 2)
            fprintf(stderr, "PRC_DIAG_CHAINSTART_ZERO: mesh_vtx=%u dv=(%d,%d,%d) n_points=%u zeros=%d\n",
                mesh_vtx, dv[0], dv[1], dv[2], st->n_points, zeros);
    }

    st->out->point_array[st->out->point_array_size + 0] = dv[0];
    st->out->point_array[st->out->point_array_size + 1] = dv[1];
    st->out->point_array[st->out->point_array_size + 2] = dv[2];
    st->out->point_array_size += 3;

    scaled.x = ((double)dv[0]) * st->tol;
    scaled.y = ((double)dv[1]) * st->tol;
    scaled.z = ((double)dv[2]) * st->tol;
    prc_vec_add(scaled, base, &rec);
    prc_diag_track_pos_error(st, p, rec);

    st->decoded_pos[st->n_points] = rec;
    st->vtx_map[mesh_vtx] = (int32_t)st->n_points;
    *out_index = (int32_t)st->n_points;
    prc_encode_record_analysis(st, mesh_vtx);
    st->n_points++;
    return 0;
}

/* EXPERIMENT (2026-07-28), DISPROVEN same day: reference pseudocode (sample_
   compressed_mesh_write_pseudocode.md) decomposes the target point into the
   edge basis by solving the 3x3 linear system [X Y Z] * v = diff via
   Cramer's rule (determinant ratios), not by assuming X/Y/Z form a
   perfectly orthonormal frame and projecting via dot products. For a
   genuinely orthonormal basis the two are mathematically identical (the
   inverse of an orthonormal matrix is its transpose); they diverge only to
   the extent floating-point error has left the basis very slightly
   non-orthonormal. Tested against three real files that reproducibly blank
   the Acrobat model tree via the >100-part lumped-COMPRESSED path
   (davidgbarnes-submitted-version.stream-90, Walnut_Viewport3_PDF3D.
   stream-64, QCD_Leinweber_ActionXs24t36black_Anim_r9796.stream-219) --
   all three still blanked the tree identically, alone and combined with
   the halved-tolerance experiment below. Basis decomposition method is not
   the cause of this bug family. Gated behind PRC_DIAG_USE_CRAMER_BASIS so
   default behavior (dot-product projection, unchanged for years of
   validated real-file output) is completely unaffected; kept as a
   diagnostic in case a narrower investigation wants it later. */
static int
prc_encode_use_cramer_basis(void)
{
    static int cached = -1;
    if (cached < 0)
        cached = (prc_diag_getenv("PRC_DIAG_USE_CRAMER_BASIS") != NULL) ? 1 : 0;
    return cached;
}

/* EXPERIMENT (2026-08-05), mixed_chains investigation: prc_encode_edge_basis's
   use_alternate_basis fallback (near-collinear edge/opposite-vertex triple,
   primary cross-product z-axis underflows normalize) picks an axis via
   prc_vec_make_orth_basis's own nearest-world-axis heuristic, then a
   handedness sign flip on vectors that are themselves near-degenerate for a
   sliver triangle -- unlike the primary path, nothing about this fallback's
   OUTCOME is written to the stream; a real decoder must re-derive the exact
   same choice from reconstructed geometry alone. nanoPRC's own decoder shares
   this code so it always agrees with itself, but an independent decoder
   (Acrobat) implementing the same ambiguous fallback slightly differently
   would silently reconstruct a DIFFERENT basis, corrupting every point in
   that grow op and cascading into whatever chains off it. Gated behind
   PRC_DIAG_REFUSE_ALTERNATE_BASIS_GROW so default behavior is unaffected;
   when set, an edge whose basis needed the fallback is treated exactly like
   a basis-computation failure (grow refused, neighbor becomes its own chain
   start instead, which uses the fallback-free origin-relative axis
   encoding). */
static int
prc_encode_refuse_alternate_basis_grow(void)
{
    static int cached = -1;
    if (cached < 0)
        cached = (prc_diag_getenv("PRC_DIAG_REFUSE_ALTERNATE_BASIS_GROW") != NULL) ? 1 : 0;
    return cached;
}

static void
prc_encode_cramer_decompose(prc_vec3 diff, prc_vec3 x, prc_vec3 y, prc_vec3 z,
    double *out_x, double *out_y, double *out_z)
{
    prc_vec3 yz, xdiffz_cross, ydiff_cross;
    double det;

    prc_vec_cross(y, z, &yz);
    det = prc_vec_dot_product(x, yz);

    if (det > -1.0e-300 && det < 1.0e-300)
    {
        /* Degenerate (should not happen for a basis that already survived
           prc_encode_edge_basis's own normalization/fallback) -- fall back
           to the ordinary dot-product projection rather than divide by
           (near-)zero. */
        *out_x = prc_vec_dot_product(diff, x);
        *out_y = prc_vec_dot_product(diff, y);
        *out_z = prc_vec_dot_product(diff, z);
        return;
    }

    *out_x = prc_vec_dot_product(diff, yz) / det;

    prc_vec_cross(diff, z, &xdiffz_cross);
    *out_y = prc_vec_dot_product(x, xdiffz_cross) / det;

    prc_vec_cross(y, diff, &ydiff_cross);
    *out_z = prc_vec_dot_product(x, ydiff_cross) / det;
}

/* Emit a new point predicted in a grow op's orthonormal edge basis; the
   reconstruction mirrors prc_decode_next_point_post_scale exactly. */
static int
prc_encode_emit_basis_point(prc_encode_state *st, uint32_t mesh_vtx,
    const prc_encode_grow_op *op, int32_t *out_index)
{
    const double *p = st->mesh->positions + (size_t)mesh_vtx * 3;
    prc_vec3 diff, x, y, z, temp, temp2, rec;
    int32_t dv[3];
    int code;
    double proj_x, proj_y, proj_z;

    diff.x = p[0] - op->origin.x;
    diff.y = p[1] - op->origin.y;
    diff.z = p[2] - op->origin.z;

    if (prc_encode_use_cramer_basis())
        prc_encode_cramer_decompose(diff, op->x_basis, op->y_basis, op->z_basis,
            &proj_x, &proj_y, &proj_z);
    else
    {
        proj_x = prc_vec_dot_product(diff, op->x_basis);
        proj_y = prc_vec_dot_product(diff, op->y_basis);
        proj_z = prc_vec_dot_product(diff, op->z_basis);
    }

    code = prc_encode_quantize(st->ctx, proj_x, st->tol, &dv[0]);
    if (code < 0)
        return code;
    code = prc_encode_quantize(st->ctx, proj_y, st->tol, &dv[1]);
    if (code < 0)
        return code;
    code = prc_encode_quantize(st->ctx, proj_z, st->tol, &dv[2]);
    if (code < 0)
        return code;

    prc_diag_nudge_zero_delta(dv);

    if (st->ctx->trace_reversed)
        fprintf(stderr, "ENC_BASISPOINT mesh_vtx=%u op_origin=(%.17g,%.17g,%.17g) p=(%.17g,%.17g,%.17g) "
            "diff=(%.17g,%.17g,%.17g) x_basis=(%.17g,%.17g,%.17g) y_basis=(%.17g,%.17g,%.17g) "
            "dv=(%d,%d,%d) n_points=%u\n",
            mesh_vtx, op->origin.x, op->origin.y, op->origin.z, p[0], p[1], p[2],
            diff.x, diff.y, diff.z, op->x_basis.x, op->x_basis.y, op->x_basis.z,
            op->y_basis.x, op->y_basis.y, op->y_basis.z, dv[0], dv[1], dv[2], st->n_points);

    st->out->point_array[st->out->point_array_size + 0] = dv[0];
    st->out->point_array[st->out->point_array_size + 1] = dv[1];
    st->out->point_array[st->out->point_array_size + 2] = dv[2];
    st->out->point_array_size += 3;

    x = op->x_basis;
    x.x = ((double)dv[0]) * st->tol * x.x;
    x.y = ((double)dv[0]) * st->tol * x.y;
    x.z = ((double)dv[0]) * st->tol * x.z;
    y = op->y_basis;
    y.x = ((double)dv[1]) * st->tol * y.x;
    y.y = ((double)dv[1]) * st->tol * y.y;
    y.z = ((double)dv[1]) * st->tol * y.z;
    z = op->z_basis;
    z.x = ((double)dv[2]) * st->tol * z.x;
    z.y = ((double)dv[2]) * st->tol * z.y;
    z.z = ((double)dv[2]) * st->tol * z.z;
    prc_vec_add(op->origin, x, &temp);
    prc_vec_add(temp, y, &temp2);
    prc_vec_add(temp2, z, &rec);
    prc_diag_track_pos_error(st, p, rec);

    st->decoded_pos[st->n_points] = rec;
    st->vtx_map[mesh_vtx] = (int32_t)st->n_points;
    *out_index = (int32_t)st->n_points;
    prc_encode_record_analysis(st, mesh_vtx);
    st->n_points++;
    return 0;
}

/* Re-derivation of the decoder's prc_compute_triangle_basis: the decoder
   recomputes this basis from its reconstructed vertices when it pops a grow
   op, so the encoder must reproduce it exactly, including the
   prc_vec_make_orth_basis fallback and the unchecked final y-normalize.
   prc_vec_compute_basis_origin looks similar but uses the opposite X sign
   convention (v1-v0) and lacks the y-dot-w flip, so it cannot be used here. */
static int
prc_encode_edge_basis(prc_vec3 E0, prc_vec3 E1, prc_vec3 E3,
    prc_vec3 *x_out, prc_vec3 *y_out, prc_vec3 *z_out, prc_vec3 *origin_out,
    uint8_t *out_used_alternate)
{
    prc_vec3 x, z, z_temp, w, origin;
    prc_vec3 y = { 0.0, 0.0, 0.0 }; /* always overwritten before use; silences a
                                       false-positive uninitialized-use warning
                                       the compiler can't resolve across the
                                       use_alternate_basis branches */
    prc_basis basis;
    int code;
    uint8_t use_alternate_basis = 0;

    *out_used_alternate = 0;
    prc_vec_avg(E0, E1, &origin);
    prc_vec_sub(E0, E1, &x);
    code = prc_vec_normalize(&x);
    if (code < 0)
        return code;

    prc_vec_sub(E3, origin, &z_temp);
    prc_vec_cross(z_temp, x, &z);
    code = prc_vec_normalize(&z);
    if (code < 0)
        use_alternate_basis = 1;

    if (!use_alternate_basis)
    {
        prc_vec_cross(z, x, &y);
        code = prc_vec_normalize(&y);
        if (code < 0)
            use_alternate_basis = 1;
    }

    if (use_alternate_basis)
    {
        *out_used_alternate = 1;
        basis.X = x;
        code = prc_vec_make_orth_basis(&basis);
        if (code < 0)
            return code;
        y = basis.Y;
        z = basis.Z;
        prc_vec_cross(z, x, &y);
        /* the decoder ignores this normalize result; mirror that */
        (void)prc_vec_normalize(&y);
    }

    prc_vec_sub(origin, E3, &w);
    if (prc_vec_dot_product(y, w) > 0.0)
    {
        prc_vec_negate(&z);
        prc_vec_negate(&y);
    }

    *x_out = x;
    *y_out = y;
    *z_out = z;
    *origin_out = origin;
    return 0;
}

static int32_t
prc_encode_local_edge_slot(const prc_encode_mesh *mesh, uint32_t tri,
    uint32_t va, uint32_t vb)
{
    uint32_t e;

    for (e = 0; e < 3; e++)
    {
        uint32_t a = mesh->tri_indices[(size_t)tri * 3 + e];
        uint32_t b = mesh->tri_indices[(size_t)tri * 3 + (e + 1) % 3];
        if ((a == va && b == vb) || (a == vb && b == va))
            return (int32_t)e;
    }
    return -1;
}

static int
prc_encode_stack_push(prc_encode_state *st, const prc_encode_grow_op *op)
{
    if (st->stack_size == st->stack_capacity)
    {
        uint32_t new_cap = st->stack_capacity ? st->stack_capacity * 2 : 64;
        prc_encode_grow_op *grown = (prc_encode_grow_op *)prc_realloc(st->ctx,
            st->stack, (size_t)new_cap * sizeof(prc_encode_grow_op));
        if (grown == NULL)
        {
            prc_error(st->ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_traversal stack\n");
            return PRC_ERROR_MEMORY;
        }
        st->stack = grown;
        st->stack_capacity = new_cap;
    }
    st->stack[st->stack_size] = *op;
    st->stack_size++;
    return 0;
}

/* Start a fresh chain at triangle tri (the decoder's empty-stack path).
   Vertex order is kept exactly as given in mesh->tri_indices; already-emitted
   vertices become references so shared vertices are never duplicated. */
static int
prc_encode_chain_start(prc_encode_state *st, uint32_t tri,
    int32_t idx[3], uint32_t mv[3])
{
    prc_encode_traversal_result *out = st->out;
    uint8_t r[3];
    uint32_t num_refs, k;
    prc_vec3 temp;
    int code;

    for (k = 0; k < 3; k++)
        mv[k] = st->mesh->tri_indices[(size_t)tri * 3 + k];
    num_refs = 0;
    for (k = 0; k < 3; k++)
    {
        r[k] = (uint8_t)(st->vtx_map[mv[k]] >= 0 ? 1 : 0);
        num_refs += r[k];
        out->points_is_reference_array[out->points_is_reference_array_size] = r[k];
        out->points_is_reference_array_size++;
    }
    if (st->tri_is_ref != NULL)
        st->tri_is_ref[tri] = (uint8_t)num_refs;
    if (st->ctx->trace_reversed)
    {
        fprintf(stderr, "ENC_CHAINSTART tri=%u mv=(%u,%u,%u) r=(%u,%u,%u) num_refs=%u "
            "pref_size_before=%u pisref_size_before=%u\n",
            tri, mv[0], mv[1], mv[2], r[0], r[1], r[2], num_refs,
            out->point_reference_array_size, out->points_is_reference_array_size - 3);
    }

    if (num_refs == 0)
    {
        /* Mirrors prc_compute_first_triangle */
        code = prc_encode_emit_axis_point(st, mv[0], st->origin, &idx[0]);
        if (code < 0)
            return code;
        st->hop_depth[idx[0]] = 0;
        code = prc_encode_emit_axis_point(st, mv[1], st->decoded_pos[idx[0]], &idx[1]);
        if (code < 0)
            return code;
        st->hop_depth[idx[1]] = st->hop_depth[idx[0]] + 1;
        prc_vec_avg(st->decoded_pos[idx[0]], st->decoded_pos[idx[1]], &temp);
        code = prc_encode_emit_axis_point(st, mv[2], temp, &idx[2]);
        if (code < 0)
            return code;
        st->hop_depth[idx[2]] = (st->hop_depth[idx[0]] > st->hop_depth[idx[1]] ?
            st->hop_depth[idx[0]] : st->hop_depth[idx[1]]) + 1;

        /* PROBE (2026-08-05), mixed_chains investigation continued: see
           chain_start_tilt_max's own comment on prc_encode_state. Computed
           unconditionally whenever PRC_DIAG_MESH_QUALITY is set, cheap
           relative to everything else this function already does per
           triangle. */
        if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
        {
            const double *p0 = st->mesh->positions + (size_t)mv[0] * 3;
            const double *p1 = st->mesh->positions + (size_t)mv[1] * 3;
            const double *p2 = st->mesh->positions + (size_t)mv[2] * 3;
            prc_vec3 tp0, tp1, tp2, e1, e2, true_n, r_e1, r_e2, rec_n;

            tp0.x = p0[0]; tp0.y = p0[1]; tp0.z = p0[2];
            tp1.x = p1[0]; tp1.y = p1[1]; tp1.z = p1[2];
            tp2.x = p2[0]; tp2.y = p2[1]; tp2.z = p2[2];
            prc_vec_sub(tp1, tp0, &e1);
            prc_vec_sub(tp2, tp0, &e2);
            prc_vec_cross(e1, e2, &true_n);
            prc_vec_sub(st->decoded_pos[idx[1]], st->decoded_pos[idx[0]], &r_e1);
            prc_vec_sub(st->decoded_pos[idx[2]], st->decoded_pos[idx[0]], &r_e2);
            prc_vec_cross(r_e1, r_e2, &rec_n);

            if (prc_vec_normalize(&true_n) == 0 && prc_vec_normalize(&rec_n) == 0)
            {
                double tilt = 1.0 - prc_vec_dot_product(true_n, rec_n);
                st->chain_start_count++;
                if (tilt > st->chain_start_tilt_max)
                    st->chain_start_tilt_max = tilt;
                if (tilt > 1e-6) st->chain_start_tilt_gt_1e_6++;
                if (tilt > 1e-4) st->chain_start_tilt_gt_1e_4++;
                if (tilt > 1e-2) st->chain_start_tilt_gt_1e_2++;
            }
        }
    }
    else if (num_refs == 1)
    {
        /* Mirrors prc_set_one_ref_treated_triangle */
        if (r[0])
        {
            idx[0] = st->vtx_map[mv[0]];
            out->point_reference_array[out->point_reference_array_size++] = idx[0];
            code = prc_encode_emit_axis_point(st, mv[1], st->decoded_pos[idx[0]], &idx[1]);
            if (code < 0)
                return code;
            st->hop_depth[idx[1]] = st->hop_depth[idx[0]] + 1;
            prc_vec_avg(st->decoded_pos[idx[0]], st->decoded_pos[idx[1]], &temp);
            code = prc_encode_emit_axis_point(st, mv[2], temp, &idx[2]);
            if (code < 0)
                return code;
            st->hop_depth[idx[2]] = (st->hop_depth[idx[0]] > st->hop_depth[idx[1]] ?
                st->hop_depth[idx[0]] : st->hop_depth[idx[1]]) + 1;
        }
        else if (r[1])
        {
            idx[1] = st->vtx_map[mv[1]];
            out->point_reference_array[out->point_reference_array_size++] = idx[1];
            code = prc_encode_emit_axis_point(st, mv[0], st->origin, &idx[0]);
            if (code < 0)
                return code;
            st->hop_depth[idx[0]] = 0;
            prc_vec_avg(st->decoded_pos[idx[1]], st->decoded_pos[idx[0]], &temp);
            code = prc_encode_emit_axis_point(st, mv[2], temp, &idx[2]);
            if (code < 0)
                return code;
            st->hop_depth[idx[2]] = (st->hop_depth[idx[0]] > st->hop_depth[idx[1]] ?
                st->hop_depth[idx[0]] : st->hop_depth[idx[1]]) + 1;
        }
        else
        {
            idx[2] = st->vtx_map[mv[2]];
            out->point_reference_array[out->point_reference_array_size++] = idx[2];
            code = prc_encode_emit_axis_point(st, mv[0], st->origin, &idx[0]);
            if (code < 0)
                return code;
            st->hop_depth[idx[0]] = 0;
            code = prc_encode_emit_axis_point(st, mv[1], st->decoded_pos[idx[0]], &idx[1]);
            if (code < 0)
                return code;
            st->hop_depth[idx[1]] = st->hop_depth[idx[0]] + 1;
        }
    }
    else if (num_refs == 2)
    {
        /* Mirrors prc_set_two_ref_treated_triangle */
        if (!r[0])
        {
            idx[1] = st->vtx_map[mv[1]];
            idx[2] = st->vtx_map[mv[2]];
            out->point_reference_array[out->point_reference_array_size++] = idx[1];
            out->point_reference_array[out->point_reference_array_size++] = idx[2];
            code = prc_encode_emit_axis_point(st, mv[0], st->origin, &idx[0]);
            if (code < 0)
                return code;
            st->hop_depth[idx[0]] = 0;
        }
        else if (!r[1])
        {
            idx[0] = st->vtx_map[mv[0]];
            idx[2] = st->vtx_map[mv[2]];
            out->point_reference_array[out->point_reference_array_size++] = idx[0];
            out->point_reference_array[out->point_reference_array_size++] = idx[2];
            code = prc_encode_emit_axis_point(st, mv[1], st->decoded_pos[idx[0]], &idx[1]);
            if (code < 0)
                return code;
            st->hop_depth[idx[1]] = st->hop_depth[idx[0]] + 1;
        }
        else
        {
            idx[0] = st->vtx_map[mv[0]];
            idx[1] = st->vtx_map[mv[1]];
            out->point_reference_array[out->point_reference_array_size++] = idx[0];
            out->point_reference_array[out->point_reference_array_size++] = idx[1];
            prc_vec_avg(st->decoded_pos[idx[0]], st->decoded_pos[idx[1]], &temp);
            code = prc_encode_emit_axis_point(st, mv[2], temp, &idx[2]);
            if (code < 0)
                return code;
            st->hop_depth[idx[2]] = (st->hop_depth[idx[0]] > st->hop_depth[idx[1]] ?
                st->hop_depth[idx[0]] : st->hop_depth[idx[1]]) + 1;
        }
    }
    else
    {
        /* Mirrors prc_set_three_ref_treated_triangle */
        for (k = 0; k < 3; k++)
        {
            idx[k] = st->vtx_map[mv[k]];
            out->point_reference_array[out->point_reference_array_size++] = idx[k];
        }
    }

    /* Mirror prc_handle_empty_stack_decode's final index0 > index1 swap so the
       left/right edge bookkeeping below matches the decoder's view. */
    if (idx[0] > idx[1])
    {
        int32_t ti = idx[0];
        uint32_t tm = mv[0];
        idx[0] = idx[1];
        idx[1] = ti;
        mv[0] = mv[1];
        mv[1] = tm;
    }
    return 0;
}

/* Grow the popped op's target triangle: new triangle is (V0, V1, third). */
static int
prc_encode_grow_triangle(prc_encode_state *st, const prc_encode_grow_op *op,
    int32_t idx[3], uint32_t mv[3])
{
    prc_encode_traversal_result *out = st->out;
    uint32_t t = op->target_tri;
    uint32_t third = UINT32_MAX;
    uint32_t k;
    int code;

    for (k = 0; k < 3; k++)
    {
        uint32_t v = st->mesh->tri_indices[(size_t)t * 3 + k];
        if (v != op->mesh_v0 && v != op->mesh_v1)
            third = v;
    }
    if (third == UINT32_MAX)
    {
        prc_error(st->ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: grow target has no apex vertex\n");
        return PRC_ERROR_INTERNAL;
    }

    if (st->vtx_map[third] >= 0)
    {
        out->points_is_reference_array[out->points_is_reference_array_size++] = 1;
        idx[2] = st->vtx_map[third];
        out->point_reference_array[out->point_reference_array_size++] = idx[2];
        if (st->tri_is_ref != NULL)
            st->tri_is_ref[t] = 1;
    }
    else
    {
        out->points_is_reference_array[out->points_is_reference_array_size++] = 0;
        if (st->tri_is_ref != NULL)
            st->tri_is_ref[t] = 0;
        code = prc_encode_emit_basis_point(st, third, op, &idx[2]);
        if (code < 0)
            return code;
        /* PROBE (2026-08-06): see hop_depth's own comment on prc_encode_state.
           This apex point's own basis was built from op->index0/index1 (the
           edge just grown across), so its hop depth is one more than whichever
           of those two carries the deeper (more error-compounding-prone)
           chain. */
        st->hop_depth[idx[2]] = (st->hop_depth[op->index0] > st->hop_depth[op->index1] ?
            st->hop_depth[op->index0] : st->hop_depth[op->index1]) + 1;
    }

    idx[0] = op->index0;
    idx[1] = op->index1;
    mv[0] = op->mesh_v0;
    mv[1] = op->mesh_v1;
    mv[2] = third;
    return 0;
}

/* Decide whether the just-finalized triangle (idx[]/mv[] already established
   by chain_start or grow_triangle for THIS traversal, not a stale baseline)
   should be marked normal_was_reversed, from the real supplied normal vs.
   the geometric normal derived from its own decoded positions:
   dot(corner0_real_normal, cross(P1-P0,P2-P0)) > 0. Deciding this inline,
   right here, means it always sees this triangle's true final vertex order
   rather than one computed from a separate, potentially stale traversal pass.

   Uses TRAVERSAL corner 0's (idx[0]'s) real supplied normal specifically --
   not an average over all 3 corners -- to match prc_encode_normals_c2's own
   rejection check and the decoder's prc_is_normal_reversed_single_normal
   ("always use the one at V[0]") exactly, rather than a coarser per-position
   average that generally disagrees with the corner-0-specific test whenever
   a vertex's incident corners carry different supplied normals (i.e. most
   real, smooth-shaded meshes) -- that disagreement is what made C2 reject on
   essentially every real mesh tested before this. real_normals is mesh->
   num_triangles*9 doubles, 3 per corner, MESH order (mesh->tri_indices
   alignment) -- the same layout as corner_normals, deliberately not
   per-deduplicated-position, so the corner-0-specific value can be
   recovered exactly. mv[0]/cur locate which mesh-order corner of triangle
   cur is traversal slot 0. */
static uint8_t
prc_encode_decide_reversed(const prc_encode_state *st, uint32_t cur, const int32_t idx[3], const uint32_t mv[3])
{
    prc_vec3 P0 = st->decoded_pos[idx[0]];
    prc_vec3 P1 = st->decoded_pos[idx[1]];
    prc_vec3 P2 = st->decoded_pos[idx[2]];
    prc_vec3 e1, e2, raw_cross, n;
    const double *nd;
    uint32_t j, corner = 3;

    prc_vec_sub(P1, P0, &e1);
    prc_vec_sub(P2, P0, &e2);
    prc_vec_cross(e1, e2, &raw_cross);

    for (j = 0; j < 3; j++)
    {
        if (st->mesh->tri_indices[(size_t)cur * 3 + j] == mv[0])
            corner = j;
    }
    /* Cannot happen: mv[0] is one of triangle cur's own three mesh vertices
       by construction (chain_start/grow_triangle only ever assign mv[] from
       mesh->tri_indices[cur*3..+3]). */
    if (corner == 3)
        return 0;

    nd = &st->real_normals[(size_t)cur * 9 + (size_t)corner * 3];
    n.x = nd[0]; n.y = nd[1]; n.z = nd[2];
    return (uint8_t)(prc_vec_dot_product(n, raw_cross) > 0.0);
}

/* Decide the just-emitted triangle's edge status bits and push its growable
   edges (right first, then left -- the decoder's LIFO push order).

   When st->tri_reversed[tri] is set, this mirrors the decoder's
   prc_set_left_right_edge_indices: the decoder swaps its own right/left
   edge role assignment for a reversed triangle, so we swap which physical
   edge (idx1-idx2 vs idx0-idx2) we treat as "right"/slot-0 (pushed first,
   bit 0) vs "left"/slot-1 (pushed second, bit 1) to match. Swapping the
   slot CONTENTS up front, before the rest of this function (which always
   treats slot 0 as "right, processed/pushed first"), makes the remainder
   of the function correct unchanged for both cases.

   An edge is growable only if its neighbor exists, is unvisited AND has no
   grow op already pending on the stack. Never re-pushing a pending triangle
   means a popped op's target is always fresh, so the decoder never needs its
   treated-edge discard search and both sides pop in exact lockstep. */
static int
prc_encode_edge_status(prc_encode_state *st, uint32_t tri,
    const int32_t idx[3], const uint32_t mv[3], uint8_t *status_out)
{
    uint8_t bits[2];
    uint32_t e;
    /* [0] = right edge (idx1, idx2, apex idx0), [1] = left edge (idx0, idx2, apex idx1)
       -- for an un-reversed triangle; see the reversed-swap below. */
    int32_t ex[2], ey[2], ez[2];
    uint32_t ma[2], mb[2];
    int code;

    ex[0] = idx[1];
    ey[0] = idx[2];
    ez[0] = idx[0];
    ma[0] = mv[1];
    mb[0] = mv[2];

    ex[1] = idx[0];
    ey[1] = idx[2];
    ez[1] = idx[1];
    ma[1] = mv[0];
    mb[1] = mv[2];

    if (st->tri_reversed != NULL && st->tri_reversed[tri])
    {
        int32_t ti;
        uint32_t tm;

        ti = ex[0]; ex[0] = ex[1]; ex[1] = ti;
        ti = ey[0]; ey[0] = ey[1]; ey[1] = ti;
        ti = ez[0]; ez[0] = ez[1]; ez[1] = ti;
        tm = ma[0]; ma[0] = ma[1]; ma[1] = tm;
        tm = mb[0]; mb[0] = mb[1]; mb[1] = tm;
    }

    for (e = 0; e < 2; e++)
    {
        int32_t slot, nb;
        uint8_t growable;

        if (ex[e] > ey[e])
        {
            int32_t ti = ex[e];
            uint32_t tm = ma[e];
            ex[e] = ey[e];
            ey[e] = ti;
            ma[e] = mb[e];
            mb[e] = tm;
        }

        slot = prc_encode_local_edge_slot(st->mesh, tri, ma[e], mb[e]);
        nb = slot >= 0 ? st->neighbor[(size_t)tri * 3 + (uint32_t)slot] : -1;
        /* The 64K chain cap counts already-emitted chain triangles plus the
           pending grow ops (each adds exactly one more triangle to this
           chain). Once the budget is spent we simply stop declaring growable
           edges and let the decoder's stack drain naturally -- abandoning
           live stack entries is not an option because the decoder would still
           pop them; unreached neighbors are picked up later by the outer
           unvisited-triangle scan as fresh chain starts. */
        growable = (uint8_t)(nb >= 0 &&
            !st->visited[nb] && !st->pending[nb] &&
            st->chain_len + st->stack_size < prc_encode_max_chain());

        if (growable)
        {
            prc_encode_grow_op op;
            uint8_t used_alternate = 0;

            code = prc_encode_edge_basis(st->decoded_pos[ex[e]],
                st->decoded_pos[ey[e]], st->decoded_pos[ez[e]],
                &op.x_basis, &op.y_basis, &op.z_basis, &op.origin, &used_alternate);
            if (used_alternate)
                st->alt_basis_count++;
            if (code == 0 && prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
            {
                double dxy = prc_vec_dot_product(op.x_basis, op.y_basis);
                double dyz = prc_vec_dot_product(op.y_basis, op.z_basis);
                double dxz = prc_vec_dot_product(op.x_basis, op.z_basis);
                double lx = sqrt(prc_vec_dot_product(op.x_basis, op.x_basis)) - 1.0;
                double ly = sqrt(prc_vec_dot_product(op.y_basis, op.y_basis)) - 1.0;
                double lz = sqrt(prc_vec_dot_product(op.z_basis, op.z_basis)) - 1.0;
                double dev = fabs(dxy);
                uint32_t edge_hop = st->hop_depth[ex[e]] > st->hop_depth[ey[e]] ?
                    st->hop_depth[ex[e]] : st->hop_depth[ey[e]];
                int obucket = edge_hop < 10 ? 0 : edge_hop < 100 ? 1 : edge_hop < 1000 ? 2 :
                    edge_hop < 10000 ? 3 : edge_hop < 100000 ? 4 : 5;
                if (fabs(dyz) > dev) dev = fabs(dyz);
                if (fabs(dxz) > dev) dev = fabs(dxz);
                if (fabs(lx) > dev) dev = fabs(lx);
                if (fabs(ly) > dev) dev = fabs(ly);
                if (fabs(lz) > dev) dev = fabs(lz);
                if (dev > st->orthobucket_dev_max[obucket])
                    st->orthobucket_dev_max[obucket] = dev;
                st->orthobucket_count[obucket]++;
            }
            if (code < 0 ||
                (used_alternate && prc_encode_refuse_alternate_basis_grow()))
            {
                /* Degenerate edge: the decoder would fail computing this
                   basis, so leave the edge un-grown; the neighbor is reached
                   later as its own chain start. */
                if (code == 0 && used_alternate)
                    st->alt_basis_refused_count++;
                growable = 0;
            }
            else
            {
                op.index0 = ex[e];
                op.index1 = ey[e];
                op.mesh_v0 = ma[e];
                op.mesh_v1 = mb[e];
                op.target_tri = (uint32_t)nb;
                /* A growing (non-chain-start) triangle inherits its parent's
                   own tri_reversed value UNCHANGED, for both the right- and
                   left-grown child alike -- normal_was_reversed is constant
                   along an unbroken growth chain, not independently re-
                   decided per triangle. Confirmed empirically (2026-08-10):
                   swept 4 candidate propagation rules (right-flips/left-
                   preserves, the reverse, both-flip, both-preserve) against
                   PRC_DIAG_MESH_QUALITY's alltri_tilt_max ground-truth check
                   (tilt=1.0-dot(true_n,rec_n), true_n from the ORIGINAL mesh
                   geometry) on real prc-db files -- only "both preserve"
                   (this one) collapses tilt from a perfect 2.0 (fully
                   inverted reconstructed normal) down to ordinary
                   quantization-level noise (~1e-5) on every file tested,
                   across every chain-depth bucket. This is what
                   prc_encode_decide_reversed's independent per-triangle
                   geometric decision (comparing against a SMOOTHED PROXY
                   normal, necessarily only a local approximation) was
                   getting wrong for a meaningful fraction of growing
                   triangles -- proxy-normal noise occasionally disagrees
                   with the single correct chain-wide value, especially
                   deep in long chains where more triangles have a chance to
                   hit a locally-ambiguous proxy comparison. Chain STARTS
                   still call prc_encode_decide_reversed (no parent to
                   inherit from) -- a real geometric decision is unavoidable
                   there, and is the only remaining source of incorrect
                   reversed-bit values after this fix (see the file with
                   many small parts in the same investigation, where a
                   residual issue was traced to a chain-start decision
                   specifically, not fixed by this change). */
                op.inherited_reversed = (uint8_t)(st->tri_reversed != NULL && st->tri_reversed[tri]);
                {
                    const char *trace_push_env = prc_diag_getenv("PRC_DIAG_TRACE_PUSH");
                    if (trace_push_env != NULL && nb == (int32_t)strtoul(trace_push_env, NULL, 10))
                        fprintf(stderr, "PUSH: parent_tri=%u e=%u(%s) parent_tri_reversed=%u -> child=%u inherited=%u\n",
                            tri, e, e == 0 ? "right" : "left",
                            st->tri_reversed != NULL ? st->tri_reversed[tri] : 0,
                            nb, op.inherited_reversed);
                }
                code = prc_encode_stack_push(st, &op);
                if (code < 0)
                    return code;
                st->pending[nb] = 1;
            }
        }
        bits[e] = growable;
    }

    *status_out = (uint8_t)((bits[0] ? 1 : 0) | (bits[1] ? 2 : 0));
    return 0;
}

int
prc_encode_traversal(prc_context *ctx, const prc_encode_mesh *mesh,
    const uint32_t *face_indices, double tolerance_mm,
    prc_encode_traversal_result *out,
    prc_vertex_analysis **analysis_out, uint32_t *analysis_count_out,
    const double *real_normals)
{
    prc_encode_state st;
    uint32_t i, num_tris, num_pos;
    uint32_t emitted = 0;
    uint32_t scan_pos = 0;
    int ret = PRC_ERROR_INTERNAL;
    int code;

    if (out == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: NULL output\n");
        return PRC_ERROR_INTERNAL;
    }
    memset(out, 0, sizeof(*out));
    memset(&st, 0, sizeof(st));

    {
        const char *alltri_dump_path = prc_diag_getenv("PRC_DIAG_DUMP_ALLTRI");
        if (alltri_dump_path != NULL)
            st.alltri_dump = fopen(alltri_dump_path, "w");
    }

    if (analysis_out != NULL)
        *analysis_out = NULL;
    if (analysis_count_out != NULL)
        *analysis_count_out = 0;

    if (mesh == NULL || !(tolerance_mm > 0.0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: bad mesh/tolerance\n");
        return PRC_ERROR_INTERNAL;
    }

    /* One global origin (the decoder's origin_array), used by every chain
       start and one/two-ref branch. Round-tripped through float BEFORE any
       encoder math uses it, because prc_write_compress_tess_to_stream
       writes this value as a 32-bit float (prc_bitwrite_float) -- a real
       decoder can only ever recover that float-precision value, never the
       full double.

       Previously the bbox min corner -- replaced (2026-07-26, mixed_chains
       investigation's third trigger) after confirming real Adobe Acrobat
       rejects a COMPRESSED entry whose axis-point delta has two or more
       EXACT-ZERO components (real-Acrobat-causal on a minimal, single
       -variable real-world repro: one isolated triangle from
       UK_original.stl whose chain-start vertex happened to sit exactly at
       the bbox min in X and Y). The bbox min is BY CONSTRUCTION always
       exactly equal to some mesh vertex in at least one axis -- any
       chain-start vertex that's also extremal in a second axis reproduces
       the trigger, and a bbox corner can also be arbitrarily far from
       whichever part of a large/fragmented mesh a given chain actually
       lives in, which is nanoPRC's own already-documented "shard
       corruption on large COMPRESSED meshes" bug (large deltas losing
       precision through the float round-trip -- see project memory).

       Then mesh triangle 0's centroid (2026-07-26 through 2026-08-10): a
       triangle's centroid structurally cannot exactly equal any of its own
       vertices unless the triangle is degenerate (already excluded
       upstream), closing the zero-delta trigger. But an EXPERIMENT
       (2026-08-05, mixed_chains investigation continued) found RG (an
       independent, real-world PRC encoder used throughout this project as a
       comparison oracle) resolves its own origin to something close to the
       mesh's overall vertex average, not any single triangle's centroid --
       confirmed on QCD_Leinweber, whose then-default (triangle-0-centroid)
       origin landed near a mesh EXTREME (Z~0) while RG's sits centrally
       (Z~18.24 on a mesh spanning roughly that range). A poorly-centered
       origin inflates every chain-start's V0 delta magnitude, directly
       feeding RESULT 12's V0+V2 reconstruction-error mechanism, and every
       chain restart elsewhere in the mesh pays the same inflated-delta cost
       -- worse the larger/more spread-out the mesh.

       Now (2026-08-10): the MIDPOINT OF THE TWO DEDUPLICATED MESH VERTICES
       NEAREST the arithmetic mean of every deduplicated vertex -- combines
       both properties instead of trading one for the other. An earlier
       version of this change tried "nearest mesh TRIANGLE's centroid"
       instead, but that drifted surprisingly far from the mesh-average
       point on real solid meshes (measured 15% of bbox diagonal on one
       real test file) -- every triangle centroid necessarily lies ON the
       mesh SURFACE, while the vertex average of a solid, roughly-convex
       mesh typically sits well inside the volume, so "nearest surface
       point" and "the average" can disagree substantially. The midpoint of
       the two nearest actual VERTICES doesn't have that constraint (not
       tied to any single face) and stays bound to the average by local
       vertex density, which for any real mesh with more than a handful of
       vertices is far tighter than a whole triangle's extent. Retains the
       zero-delta-collision safety property: the midpoint of two DISTINCT
       (deduplicated, hence non-coincident) points cannot equal either of
       them, or any third vertex, without an exact three-point-symmetry
       coincidence. Deliberately NOT the plain vertex-average point itself
       (which was live as an opt-in probe, `PRC_DIAG_ORIGIN_VERTEX_AVERAGE`,
       2026-08-05 through 2026-08-10, and briefly the permanent default
       earlier the same day) and deliberately not tuned to match RG's own
       (proprietary, unexamined) origin-selection algorithm bit-for-bit --
       being well-centered and collision-safe is the goal, not reproducing
       any other encoder's specific value. Not expected to fix any specific
       known bug on its own -- purely a numerical-conditioning improvement
       (smaller V0 delta magnitude at every chain start/restart). */
    if (mesh->num_positions >= 2)
    {
        double sx = 0.0, sy = 0.0, sz = 0.0;
        double avg[3];
        double best_dist2 = -1.0, second_dist2 = -1.0;
        uint32_t best_vi = 0, second_vi = 0;
        uint32_t vi;

        for (vi = 0; vi < mesh->num_positions; vi++)
        {
            sx += mesh->positions[(size_t)vi * 3 + 0];
            sy += mesh->positions[(size_t)vi * 3 + 1];
            sz += mesh->positions[(size_t)vi * 3 + 2];
        }
        avg[0] = sx / (double)mesh->num_positions;
        avg[1] = sy / (double)mesh->num_positions;
        avg[2] = sz / (double)mesh->num_positions;

        for (vi = 0; vi < mesh->num_positions; vi++)
        {
            const double *v = &mesh->positions[(size_t)vi * 3];
            double d0 = v[0] - avg[0], d1 = v[1] - avg[1], d2 = v[2] - avg[2];
            double dist2 = d0 * d0 + d1 * d1 + d2 * d2;

            if (best_dist2 < 0.0 || dist2 < best_dist2)
            {
                second_dist2 = best_dist2;
                second_vi = best_vi;
                best_dist2 = dist2;
                best_vi = vi;
            }
            else if (second_dist2 < 0.0 || dist2 < second_dist2)
            {
                second_dist2 = dist2;
                second_vi = vi;
            }
        }
        {
            const double *a = &mesh->positions[(size_t)best_vi * 3];
            const double *b = &mesh->positions[(size_t)second_vi * 3];
            out->origin[0] = (double)(float)((a[0] + b[0]) / 2.0);
            out->origin[1] = (double)(float)((a[1] + b[1]) / 2.0);
            out->origin[2] = (double)(float)((a[2] + b[2]) / 2.0);
        }
    }
    else
    {
        out->origin[0] = (double)(float)mesh->bbox[0];
        out->origin[1] = (double)(float)mesh->bbox[1];
        out->origin[2] = (double)(float)mesh->bbox[2];
    }
    {
        const char *ov = prc_diag_getenv("PRC_DIAG_FORCE_ORIGIN");
        if (ov != NULL)
        {
            double ox, oy, oz;
            if (sscanf(ov, "%lf,%lf,%lf", &ox, &oy, &oz) == 3)
            {
                out->origin[0] = (double)(float)ox;
                out->origin[1] = (double)(float)oy;
                out->origin[2] = (double)(float)oz;
            }
        }
    }
    if (prc_diag_getenv("PRC_DIAG_PRINT_BBOX_ORIGIN") != NULL)
    {
        fprintf(stderr, "PRC_DIAG_PRINT_BBOX_ORIGIN: bbox=[%.6f,%.6f,%.6f .. %.6f,%.6f,%.6f] origin=[%.6f,%.6f,%.6f]\n",
            mesh->bbox[0], mesh->bbox[1], mesh->bbox[2],
            mesh->bbox[3], mesh->bbox[4], mesh->bbox[5],
            out->origin[0], out->origin[1], out->origin[2]);
    }

    num_tris = mesh->num_triangles;
    num_pos = mesh->num_positions;
    if (num_tris == 0)
        return 0;

    st.ctx = ctx;
    st.mesh = mesh;
    st.tol = tolerance_mm;
    st.origin.x = out->origin[0];
    st.origin.y = out->origin[1];
    st.origin.z = out->origin[2];
    st.out = out;
    st.real_normals = real_normals;

    /* Every emitted point is a distinct deduplicated mesh vertex, so
       num_pos triples bounds point_array; 3 reference-bit slots and 3
       reference entries per triangle bound the other variable arrays. */
    out->point_array = (int32_t *)prc_malloc(ctx, (size_t)num_pos * 3 * sizeof(int32_t));
    out->edge_status_array = (uint8_t *)prc_malloc(ctx, (size_t)num_tris * sizeof(uint8_t));
    out->triangle_face_array = (int32_t *)prc_malloc(ctx, (size_t)num_tris * sizeof(int32_t));
    out->points_is_reference_array = (uint8_t *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(uint8_t));
    out->point_reference_array = (int32_t *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(int32_t));
    out->triangle_point_indices = (int32_t *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(int32_t));
    out->triangle_mesh_order = (uint32_t *)prc_malloc(ctx, (size_t)num_tris * sizeof(uint32_t));
    out->point_mesh_vertex = (int32_t *)prc_malloc(ctx, (size_t)num_pos * sizeof(int32_t));
    out->decoded_positions = (double *)prc_malloc(ctx, (size_t)num_pos * 3 * sizeof(double));
    out->triangle_reversed = (uint8_t *)prc_calloc(ctx, num_tris, sizeof(uint8_t));
    st.visited = (uint8_t *)prc_calloc(ctx, num_tris, sizeof(uint8_t));
    st.pending = (uint8_t *)prc_calloc(ctx, num_tris, sizeof(uint8_t));
    st.neighbor = (int32_t *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(int32_t));
    st.vtx_map = (int32_t *)prc_malloc(ctx, (size_t)num_pos * sizeof(int32_t));
    st.decoded_pos = (prc_vec3 *)prc_malloc(ctx, (size_t)num_pos * sizeof(prc_vec3));
    st.hop_depth = (uint32_t *)prc_calloc(ctx, num_pos, sizeof(uint32_t));
    st.tri_reversed = (uint8_t *)prc_calloc(ctx, num_tris, sizeof(uint8_t));
    st.tri_is_ref = (uint8_t *)prc_malloc(ctx, (size_t)num_tris * sizeof(uint8_t));
    if (st.tri_is_ref != NULL)
        memset(st.tri_is_ref, 0xFF, (size_t)num_tris * sizeof(uint8_t));
    st.tri_zbasis = (prc_vec3 *)prc_calloc(ctx, num_tris, sizeof(prc_vec3));
    if (out->point_array == NULL || out->edge_status_array == NULL ||
        out->triangle_face_array == NULL || out->points_is_reference_array == NULL ||
        out->point_reference_array == NULL || out->triangle_point_indices == NULL ||
        out->triangle_mesh_order == NULL || out->point_mesh_vertex == NULL ||
        out->decoded_positions == NULL || out->triangle_reversed == NULL ||
        st.visited == NULL || st.pending == NULL || st.neighbor == NULL ||
        st.vtx_map == NULL || st.decoded_pos == NULL || st.hop_depth == NULL || st.tri_reversed == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_traversal\n");
        ret = PRC_ERROR_MEMORY;
        goto fail;
    }
    if (analysis_out != NULL)
    {
        /* Same num_pos upper bound as decoded_pos: one entry per point the
           decoder will actually emit, shrunk to n_points on success. */
        st.analysis = (prc_vertex_analysis *)prc_malloc(ctx,
            (size_t)num_pos * sizeof(prc_vertex_analysis));
        if (st.analysis == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_traversal analysis\n");
            ret = PRC_ERROR_MEMORY;
            goto fail;
        }
    }

    for (i = 0; i < num_tris * 3; i++)
        st.neighbor[i] = -1;
    for (i = 0; i < num_pos; i++)
        st.vtx_map[i] = -1;
    for (i = 0; i < mesh->num_edges; i++)
    {
        const prc_encode_edge *edge = &mesh->edges[i];
        int32_t s0, s1;

        if (edge->tri1 == -1)
            continue;
        s0 = prc_encode_local_edge_slot(mesh, (uint32_t)edge->tri0, edge->v0, edge->v1);
        s1 = prc_encode_local_edge_slot(mesh, (uint32_t)edge->tri1, edge->v0, edge->v1);
        if (s0 < 0 || s1 < 0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: adjacency/index mismatch\n");
            goto fail;
        }
        st.neighbor[(size_t)edge->tri0 * 3 + (uint32_t)s0] = edge->tri1;
        st.neighbor[(size_t)edge->tri1 * 3 + (uint32_t)s1] = edge->tri0;
    }

    while (emitted < num_tris)
    {
        int32_t idx[3];
        uint32_t mv[3];
        uint32_t cur;
        uint8_t is_growing = 0;
        uint8_t entering_reversed = 0;

        if (st.stack_size > 0)
        {
            prc_encode_grow_op op;

            st.stack_size--;
            op = st.stack[st.stack_size];
            is_growing = 1;
            entering_reversed = op.inherited_reversed;
            {
                const char *trace_pop_env = prc_diag_getenv("PRC_DIAG_TRACE_PUSH");
                if (trace_pop_env != NULL && op.target_tri == (uint32_t)strtoul(trace_pop_env, NULL, 10))
                    fprintf(stderr, "POP: cur=%u entering_reversed=%u index0=%d index1=%d mesh_v0=%u mesh_v1=%u\n",
                        op.target_tri, entering_reversed, op.index0, op.index1, op.mesh_v0, op.mesh_v1);
            }
            if (st.visited[op.target_tri])
            {
                /* Cannot happen: growable edges are never declared toward
                   visited or pending triangles, so popped targets are fresh. */
                prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: popped a visited target\n");
                goto fail;
            }
            code = prc_encode_grow_triangle(&st, &op, idx, mv);
            if (code < 0)
            {
                ret = code;
                goto fail;
            }
            cur = op.target_tri;
            if (st.tri_zbasis != NULL)
                st.tri_zbasis[cur] = op.z_basis;
            st.visited[cur] = 1;
            st.pending[cur] = 0;
            st.chain_len++;
            if (st.chain_len > st.max_chain_len)
                st.max_chain_len = st.chain_len;
        }
        else
        {
            while (scan_pos < num_tris && st.visited[scan_pos])
                scan_pos++;
            if (scan_pos >= num_tris)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_traversal: no unvisited triangle left\n");
                goto fail;
            }
            cur = scan_pos;
            /* Both the first chain and every restart pass through here;
               emitted == 0 distinguishes the first so chain ids start at 0. */
            if (emitted > 0)
                st.current_chain++;
            /* DIAGNOSTIC (2026-07-22, PRC_DIAG_RESTART_REASON): for each
               genuine chain restart (not the very first chain), report
               whether this triangle's 3 mesh-topology neighbor slots (from
               st.neighbor[], populated purely from mesh adjacency,
               independent of visited/pending state) exist at all, and if so
               whether they were already visited by an earlier chain vs.
               genuinely absent (a true mesh boundary/non-manifold-dropped
               edge). Distinguishes "genuine dead end" from "neighbor exists
               but timing stranded this triangle". Read-only, no behavior
               change. */
            if (emitted > 0 && prc_diag_getenv("PRC_DIAG_RESTART_REASON") != NULL)
            {
                int32_t n0 = st.neighbor[(size_t)cur * 3 + 0];
                int32_t n1 = st.neighbor[(size_t)cur * 3 + 1];
                int32_t n2 = st.neighbor[(size_t)cur * 3 + 2];
                fprintf(stderr, "RESTART emitted=%u cur=%u n=(%d,%d,%d) visited=(%d,%d,%d)\n",
                    emitted, cur, n0, n1, n2,
                    n0 >= 0 ? (int)st.visited[n0] : -1,
                    n1 >= 0 ? (int)st.visited[n1] : -1,
                    n2 >= 0 ? (int)st.visited[n2] : -1);
            }
            st.chain_offset = 0;
            code = prc_encode_chain_start(&st, cur, idx, mv);
            if (code < 0)
            {
                ret = code;
                goto fail;
            }
            st.visited[cur] = 1;
            st.chain_len = 1;
        }

        /* Face ids are a pure side channel reordered into traversal order;
           without caller-provided ids every triangle maps to face 0. */
        out->triangle_face_array[emitted] = face_indices != NULL ? (int32_t)face_indices[cur] : 0;

        out->triangle_point_indices[(size_t)emitted * 3 + 0] = idx[0];
        out->triangle_point_indices[(size_t)emitted * 3 + 1] = idx[1];
        out->triangle_point_indices[(size_t)emitted * 3 + 2] = idx[2];
        out->triangle_mesh_order[emitted] = cur;

        /* PROBE (2026-08-06): see alltri_tilt_max's own comment on prc_encode_state.
           Computed for EVERY triangle (chain-start or grow), not just chain-starts. */
        if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
        {
            const double *p0 = mesh->positions + (size_t)mv[0] * 3;
            const double *p1 = mesh->positions + (size_t)mv[1] * 3;
            const double *p2 = mesh->positions + (size_t)mv[2] * 3;
            prc_vec3 tp0, tp1, tp2, e1, e2, true_n, r_e1, r_e2, rec_n;
            /* PROBE (2026-08-08): a real reader renders this triangle's
               vertex order as idx[0],idx[1],idx[2] UNCHANGED when
               prc_encode_decide_reversed is true, but SWAPPED (idx[0],
               idx[2],idx[1]) when it's false -- prc_decode_compressed_
               tess.c's own decoder does this at store-time (~line 480-492).
               Found via a 213-file corpus sweep: without this swap, every
               triangle whose reversed decision is false gets an artificially
               EXACT-INVERTED (tilt=2.0) rec_n here -- comparing against the
               wrong winding, not a real reconstruction defect. Uses the same
               pure decision function called again below for the real
               encoding purpose (idx[]/mv[] are already finalized at this
               point in the loop either way, so no ordering dependency).
               MUST mirror the real call site's own st->real_normals != NULL
               guard: when real_normals is unavailable, st->tri_reversed[cur]
               is never explicitly set and stays at its calloc'd default of
               0, so the WRITTEN normal_is_reversed bit is 0/false, which is
               exactly the "swap" case at decode time -- default to that,
               not to calling prc_encode_decide_reversed unguarded (it reads
               st->real_normals[...] with no null check of its own). */
            uint8_t tilt_scan_reversed;
            if (st.real_normals == NULL)
                tilt_scan_reversed = 0;
            else if (is_growing)
                tilt_scan_reversed = entering_reversed;
            else
                tilt_scan_reversed = prc_encode_decide_reversed(&st, cur, idx, mv);
            int32_t ridx1 = tilt_scan_reversed ? idx[1] : idx[2];
            int32_t ridx2 = tilt_scan_reversed ? idx[2] : idx[1];

            tp0.x = p0[0]; tp0.y = p0[1]; tp0.z = p0[2];
            tp1.x = p1[0]; tp1.y = p1[1]; tp1.z = p1[2];
            tp2.x = p2[0]; tp2.y = p2[1]; tp2.z = p2[2];
            prc_vec_sub(tp1, tp0, &e1);
            prc_vec_sub(tp2, tp0, &e2);
            prc_vec_cross(e1, e2, &true_n);
            prc_vec_sub(st.decoded_pos[ridx1], st.decoded_pos[idx[0]], &r_e1);
            prc_vec_sub(st.decoded_pos[ridx2], st.decoded_pos[idx[0]], &r_e2);
            prc_vec_cross(r_e1, r_e2, &rec_n);

            if (prc_vec_normalize(&true_n) == 0 && prc_vec_normalize(&rec_n) == 0)
            {
                double tilt = 1.0 - prc_vec_dot_product(true_n, rec_n);
                uint32_t offset = st.chain_offset > 0 ? st.chain_offset - 1 : 0;
                int bucket = offset < 10 ? 0 : offset < 100 ? 1 : offset < 1000 ? 2 :
                    offset < 10000 ? 3 : offset < 100000 ? 4 : 5;
                uint32_t max_hop = st.hop_depth[idx[0]];
                int hopbucket;
                if (st.hop_depth[idx[1]] > max_hop) max_hop = st.hop_depth[idx[1]];
                if (st.hop_depth[idx[2]] > max_hop) max_hop = st.hop_depth[idx[2]];
                hopbucket = max_hop < 10 ? 0 : max_hop < 100 ? 1 : max_hop < 1000 ? 2 :
                    max_hop < 10000 ? 3 : max_hop < 100000 ? 4 : 5;

                /* DIAGNOSTIC (2026-08-06, PRC_DIAG_TRACE_TRI=<mesh_tri_index>): full
                   detail dump for one specific triangle, to inspect an outlier found
                   via alltri_tilt_max without flooding output for the whole mesh. */
                {
                    const char *trace_tri_env = prc_diag_getenv("PRC_DIAG_TRACE_TRI");
                    if (trace_tri_env != NULL && cur == (uint32_t)strtoul(trace_tri_env, NULL, 10))
                    {
                        fprintf(stderr, "PRC_DIAG_TRACE_TRI: mesh_tri=%u chain_offset=%u tilt=%.9e "
                            "mv=(%u,%u,%u) idx=(%d,%d,%d) hop_depth=(%u,%u,%u)\n"
                            "  true p0=(%.17g,%.17g,%.17g) p1=(%.17g,%.17g,%.17g) p2=(%.17g,%.17g,%.17g)\n"
                            "  decoded p0=(%.17g,%.17g,%.17g) p1=(%.17g,%.17g,%.17g) p2=(%.17g,%.17g,%.17g)\n"
                            "  true_n=(%.9g,%.9g,%.9g) rec_n=(%.9g,%.9g,%.9g)\n",
                            cur, offset, tilt, mv[0], mv[1], mv[2], idx[0], idx[1], idx[2],
                            st.hop_depth[idx[0]], st.hop_depth[idx[1]], st.hop_depth[idx[2]],
                            p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], p2[0], p2[1], p2[2],
                            st.decoded_pos[idx[0]].x, st.decoded_pos[idx[0]].y, st.decoded_pos[idx[0]].z,
                            st.decoded_pos[idx[1]].x, st.decoded_pos[idx[1]].y, st.decoded_pos[idx[1]].z,
                            st.decoded_pos[idx[2]].x, st.decoded_pos[idx[2]].y, st.decoded_pos[idx[2]].z,
                            true_n.x, true_n.y, true_n.z, rec_n.x, rec_n.y, rec_n.z);
                    }
                }

                /* DIAGNOSTIC (2026-08-06, PRC_DIAG_DUMP_ALLTRI=<path>): same
                   fields as PRC_DIAG_TRACE_TRI above, but for every triangle,
                   written to a file instead of stderr for one -- see
                   alltri_dump's own comment on prc_encode_state. */
                if (st.alltri_dump != NULL)
                {
                    prc_vec3 zb = st.tri_zbasis != NULL ? st.tri_zbasis[cur] : (prc_vec3){0,0,0};
                    fprintf(st.alltri_dump, "%u %u %.9e %u %u %u %d "
                        "%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g "
                        "%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g "
                        "%.17g %.17g %.17g %u\n",
                        cur, offset, tilt,
                        st.hop_depth[idx[0]], st.hop_depth[idx[1]], st.hop_depth[idx[2]],
                        st.tri_is_ref != NULL ? (int)st.tri_is_ref[cur] : -1,
                        p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], p2[0], p2[1], p2[2],
                        st.decoded_pos[idx[0]].x, st.decoded_pos[idx[0]].y, st.decoded_pos[idx[0]].z,
                        st.decoded_pos[idx[1]].x, st.decoded_pos[idx[1]].y, st.decoded_pos[idx[1]].z,
                        st.decoded_pos[idx[2]].x, st.decoded_pos[idx[2]].y, st.decoded_pos[idx[2]].z,
                        zb.x, zb.y, zb.z, st.current_chain);
                }

                if (tilt > st.bucket_tilt_max[bucket])
                    st.bucket_tilt_max[bucket] = tilt;
                if (tilt > st.hopbucket_tilt_max[hopbucket])
                    st.hopbucket_tilt_max[hopbucket] = tilt;
                st.hopbucket_count[hopbucket]++;
                if (tilt > st.alltri_tilt_max)
                {
                    st.alltri_tilt_max = tilt;
                    st.alltri_tilt_max_tri = cur;
                    st.alltri_tilt_max_offset = offset;
                }
            }
        }

        /* Decide normal_was_reversed for THIS triangle now, using idx[]/mv[]
           as just finalized above -- i.e. the triangle's true final vertex
           order for this traversal, not a precomputed baseline's -- and feed
           it into the SAME-iteration edge_status call below, which is what
           actually needs it to pick push order for descendants. See
           prc_encode_traversal's header comment for why this ordering (decide,
           then push, in one pass) is the whole point. */
        if (st.real_normals != NULL)
        {
            /* A growing (non-chain-start) triangle's normal_was_reversed is
               simply inherited from its parent, unchanged -- NOT an
               independent per-triangle geometric decision. See
               prc_encode_grow_op's own comment (where the inherited value is
               computed, in prc_encode_edge_status) for the full rationale
               and the empirical evidence behind this (2026-08-10). Chain
               starts have no parent to inherit from, so they still make a
               real geometric decision via prc_encode_decide_reversed.
               PRC_DIAG_FORCE_UNREVERSED (pre-existing diagnostic) still
               overrides both cases to 0, for comparison/regression testing. */
            if (prc_diag_getenv("PRC_DIAG_FORCE_UNREVERSED") != NULL)
                st.tri_reversed[cur] = 0;
            else if (is_growing)
                st.tri_reversed[cur] = entering_reversed;
            else
                st.tri_reversed[cur] = prc_encode_decide_reversed(&st, cur, idx, mv);
            out->triangle_reversed[emitted] = st.tri_reversed[cur];
        }

        code = prc_encode_edge_status(&st, cur, idx, mv, &out->edge_status_array[emitted]);
        if (code < 0)
        {
            ret = code;
            goto fail;
        }
        /* PRC_TRACE_REVERSED / PRC_TRACE_NORMALS: env-var gated stderr tracing
           (same zero-cost-when-unset convention as prc_decode_compressed_tess.c's
           PRC_DEBUG_DISABLE_* hooks), added to compare compressed-tessellation
           encode vs. decode triangle-by-triangle -- see
           ISO-SPEC/compressed-write-normal-sign-bug.md for what this diagnoses
           and how to read its output. The matching read-side prints live in
           prc_decode_compressed_tess.c. */
        if (ctx->trace_reversed)
        {
            fprintf(stderr, "ENC k=%u tri=%u mv=(%u,%u,%u) idx=(%d,%d,%d) edge_status=%u P0=(%.6f,%.6f,%.6f) P1=(%.6f,%.6f,%.6f) P2=(%.6f,%.6f,%.6f)\n",
                emitted, cur, mv[0], mv[1], mv[2], idx[0], idx[1], idx[2], out->edge_status_array[emitted],
                st.decoded_pos[idx[0]].x, st.decoded_pos[idx[0]].y, st.decoded_pos[idx[0]].z,
                st.decoded_pos[idx[1]].x, st.decoded_pos[idx[1]].y, st.decoded_pos[idx[1]].z,
                st.decoded_pos[idx[2]].x, st.decoded_pos[idx[2]].y, st.decoded_pos[idx[2]].z);
        }
        emitted++;
    }

    out->edge_status_array_size = num_tris;
    out->triangle_face_array_size = num_tris;

    out->num_decoded_points = st.n_points;
    for (i = 0; i < num_pos; i++)
        out->point_mesh_vertex[i] = -1;
    for (i = 0; i < num_pos; i++)
    {
        if (st.vtx_map[i] >= 0)
            out->point_mesh_vertex[st.vtx_map[i]] = (int32_t)i;
    }
    for (i = 0; i < st.n_points; i++)
    {
        out->decoded_positions[(size_t)i * 3 + 0] = st.decoded_pos[i].x;
        out->decoded_positions[(size_t)i * 3 + 1] = st.decoded_pos[i].y;
        out->decoded_positions[(size_t)i * 3 + 2] = st.decoded_pos[i].z;
    }

    if (out->point_array_size < num_pos * 3)
    {
        if (out->point_array_size == 0)
        {
            prc_free(ctx, out->point_array);
            out->point_array = NULL;
        }
        else
        {
            int32_t *shrunk = (int32_t *)prc_realloc(ctx, out->point_array,
                (size_t)out->point_array_size * sizeof(int32_t));
            if (shrunk != NULL)
                out->point_array = shrunk;
        }
    }
    if (out->points_is_reference_array_size < num_tris * 3)
    {
        uint8_t *shrunk = (uint8_t *)prc_realloc(ctx, out->points_is_reference_array,
            (size_t)out->points_is_reference_array_size * sizeof(uint8_t));
        if (shrunk != NULL)
            out->points_is_reference_array = shrunk;
    }
    if (out->point_reference_array_size < num_tris * 3)
    {
        if (out->point_reference_array_size == 0)
        {
            prc_free(ctx, out->point_reference_array);
            out->point_reference_array = NULL;
        }
        else
        {
            int32_t *shrunk = (int32_t *)prc_realloc(ctx, out->point_reference_array,
                (size_t)out->point_reference_array_size * sizeof(int32_t));
            if (shrunk != NULL)
                out->point_reference_array = shrunk;
        }
    }

    if (analysis_out != NULL)
    {
        if (st.n_points > 0 && st.n_points < num_pos)
        {
            prc_vertex_analysis *shrunk = (prc_vertex_analysis *)prc_realloc(ctx,
                st.analysis, (size_t)st.n_points * sizeof(prc_vertex_analysis));
            if (shrunk != NULL)
                st.analysis = shrunk;
        }
        *analysis_out = st.analysis;
        st.analysis = NULL;
        if (analysis_count_out != NULL)
            *analysis_count_out = out->num_decoded_points;
    }

    if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL)
    {
        printf("PRC_DIAG_MESH_QUALITY: alt_basis_count=%u alt_basis_refused_count=%u "
            "(refuse_alternate_basis_grow=%d)\n",
            st.alt_basis_count, st.alt_basis_refused_count,
            prc_encode_refuse_alternate_basis_grow());
        printf("PRC_DIAG_MESH_QUALITY: chain_start_count=%u chain_start_tilt_max=%.9e "
            "tilt_gt_1e-6=%u tilt_gt_1e-4=%u tilt_gt_1e-2=%u max_chain_len=%u\n",
            st.chain_start_count, st.chain_start_tilt_max,
            st.chain_start_tilt_gt_1e_6, st.chain_start_tilt_gt_1e_4, st.chain_start_tilt_gt_1e_2,
            st.max_chain_len);
        printf("PRC_DIAG_MESH_QUALITY: alltri_tilt_max=%.9e at mesh_tri=%u chain_offset=%u "
            "bucket_tilt_max[0-10)=%.9e [10-100)=%.9e [100-1e3)=%.9e [1e3-1e4)=%.9e "
            "[1e4-1e5)=%.9e [1e5+)=%.9e\n",
            st.alltri_tilt_max, st.alltri_tilt_max_tri, st.alltri_tilt_max_offset,
            st.bucket_tilt_max[0], st.bucket_tilt_max[1], st.bucket_tilt_max[2],
            st.bucket_tilt_max[3], st.bucket_tilt_max[4], st.bucket_tilt_max[5]);
        printf("PRC_DIAG_MESH_QUALITY: hopbucket_tilt_max[0-10)=%.9e(n=%u) [10-100)=%.9e(n=%u) "
            "[100-1e3)=%.9e(n=%u) [1e3-1e4)=%.9e(n=%u) [1e4-1e5)=%.9e(n=%u) [1e5+)=%.9e(n=%u)\n",
            st.hopbucket_tilt_max[0], st.hopbucket_count[0],
            st.hopbucket_tilt_max[1], st.hopbucket_count[1],
            st.hopbucket_tilt_max[2], st.hopbucket_count[2],
            st.hopbucket_tilt_max[3], st.hopbucket_count[3],
            st.hopbucket_tilt_max[4], st.hopbucket_count[4],
            st.hopbucket_tilt_max[5], st.hopbucket_count[5]);
        printf("PRC_DIAG_MESH_QUALITY: orthobucket_dev_max[0-10)=%.9e(n=%u) [10-100)=%.9e(n=%u) "
            "[100-1e3)=%.9e(n=%u) [1e3-1e4)=%.9e(n=%u) [1e4-1e5)=%.9e(n=%u) [1e5+)=%.9e(n=%u)\n",
            st.orthobucket_dev_max[0], st.orthobucket_count[0],
            st.orthobucket_dev_max[1], st.orthobucket_count[1],
            st.orthobucket_dev_max[2], st.orthobucket_count[2],
            st.orthobucket_dev_max[3], st.orthobucket_count[3],
            st.orthobucket_dev_max[4], st.orthobucket_count[4],
            st.orthobucket_dev_max[5], st.orthobucket_count[5]);
        printf("PRC_DIAG_MESH_QUALITY: posbucket_err_max[0-10)=%.9e(n=%u) [10-100)=%.9e(n=%u) "
            "[100-1e3)=%.9e(n=%u) [1e3-1e4)=%.9e(n=%u) [1e4-1e5)=%.9e(n=%u) [1e5+)=%.9e(n=%u)\n",
            st.posbucket_err_max[0], st.posbucket_count[0],
            st.posbucket_err_max[1], st.posbucket_count[1],
            st.posbucket_err_max[2], st.posbucket_count[2],
            st.posbucket_err_max[3], st.posbucket_count[3],
            st.posbucket_err_max[4], st.posbucket_count[4],
            st.posbucket_err_max[5], st.posbucket_count[5]);
    }

    /* PROBE (2026-08-05), mixed_chains investigation continued: scans this
       entry's own DECODED (reconstructed) positions -- computed exactly as a
       compliant decoder would reconstruct them -- for any two DIFFERENT point
       indices (never merged/deduplicated upstream, so structurally meant to
       represent distinct vertices) that land in the same quantization cell
       anyway. That's a genuine, decoder-visible anomaly regardless of who's
       reading the file: two "different" vertices reconstructing to the
       identical position can produce degenerate (zero-length) edges/
       triangles and ambiguous vertex-normal-averaging groups downstream --
       a plausible trigger for a strict decoder's own robustness check.
       Reuses the same quantize-and-hash technique as the vertex dedup pass
       near the top of this file (prc_vtx_hash), just applied to the
       RECONSTRUCTED positions of an already-built traversal instead of raw
       input positions. Read-only report, PRC_DIAG_MESH_QUALITY-gated, no
       behavior change. */
    if (prc_diag_getenv("PRC_DIAG_MESH_QUALITY") != NULL && out->num_decoded_points > 0)
    {
        uint32_t dup_pairs = 0;
        size_t cap = prc_next_pow2((size_t)out->num_decoded_points * 2);
        prc_vtx_slot *dtable = (prc_vtx_slot *)prc_calloc(ctx, cap, sizeof(prc_vtx_slot));
        if (dtable != NULL)
        {
            uint32_t i;
            for (i = 0; i < out->num_decoded_points; i++)
            {
                double px = out->decoded_positions[(size_t)i * 3 + 0];
                double py = out->decoded_positions[(size_t)i * 3 + 1];
                double pz = out->decoded_positions[(size_t)i * 3 + 2];
                int64_t kx = (int64_t)llround(px / tolerance_mm);
                int64_t ky = (int64_t)llround(py / tolerance_mm);
                int64_t kz = (int64_t)llround(pz / tolerance_mm);
                size_t slot = (size_t)(prc_vtx_hash(kx, ky, kz) & (uint64_t)(cap - 1));
                for (;;)
                {
                    prc_vtx_slot *s = &dtable[slot];
                    if (!s->used)
                    {
                        s->used = 1;
                        s->key[0] = kx; s->key[1] = ky; s->key[2] = kz;
                        s->index = i;
                        break;
                    }
                    if (s->key[0] == kx && s->key[1] == ky && s->key[2] == kz)
                    {
                        dup_pairs++;
                        break;
                    }
                    slot = (slot + 1) & (cap - 1);
                }
            }
            prc_free(ctx, dtable);
            printf("PRC_DIAG_MESH_QUALITY: decoded_position_collisions=%u (out of %u decoded points)\n",
                dup_pairs, out->num_decoded_points);
        }
    }

    ret = 0;
    goto cleanup;

fail:
    prc_encode_traversal_free(ctx, out);

cleanup:
    if (st.alltri_dump != NULL)
        fclose(st.alltri_dump);
    if (st.analysis != NULL)
        prc_free(ctx, st.analysis);
    if (st.visited != NULL)
        prc_free(ctx, st.visited);
    if (st.pending != NULL)
        prc_free(ctx, st.pending);
    if (st.neighbor != NULL)
        prc_free(ctx, st.neighbor);
    if (st.vtx_map != NULL)
        prc_free(ctx, st.vtx_map);
    if (st.decoded_pos != NULL)
        prc_free(ctx, st.decoded_pos);
    if (st.hop_depth != NULL)
        prc_free(ctx, st.hop_depth);
    if (st.tri_is_ref != NULL)
        prc_free(ctx, st.tri_is_ref);
    if (st.tri_zbasis != NULL)
        prc_free(ctx, st.tri_zbasis);
    if (st.tri_reversed != NULL)
        prc_free(ctx, st.tri_reversed);
    if (st.stack != NULL)
        prc_free(ctx, st.stack);
    return ret;
}

void
prc_encode_traversal_free(prc_context *ctx, prc_encode_traversal_result *out)
{
    if (out == NULL)
        return;
    if (out->point_array != NULL)
        prc_free(ctx, out->point_array);
    if (out->edge_status_array != NULL)
        prc_free(ctx, out->edge_status_array);
    if (out->triangle_face_array != NULL)
        prc_free(ctx, out->triangle_face_array);
    if (out->points_is_reference_array != NULL)
        prc_free(ctx, out->points_is_reference_array);
    if (out->point_reference_array != NULL)
        prc_free(ctx, out->point_reference_array);
    if (out->triangle_point_indices != NULL)
        prc_free(ctx, out->triangle_point_indices);
    if (out->triangle_mesh_order != NULL)
        prc_free(ctx, out->triangle_mesh_order);
    if (out->point_mesh_vertex != NULL)
        prc_free(ctx, out->point_mesh_vertex);
    if (out->decoded_positions != NULL)
        prc_free(ctx, out->decoded_positions);
    if (out->triangle_reversed != NULL)
        prc_free(ctx, out->triangle_reversed);
    memset(out, 0, sizeof(*out));
}

/* ---- Step C: normal encoding ------------------------------------------ */

#define PRC_ENCODE_NORMAL_ANGLE_BITS 10
#define PRC_ENCODE_NORMAL_ANGLE_MAX ((1 << PRC_ENCODE_NORMAL_ANGLE_BITS) - 1)

typedef struct
{
    int32_t theta_q;
    int32_t phi_q;
    uint8_t tri_reversed;
    uint8_t x_reversed;
    uint8_t y_reversed;
} prc_encode_normal_tuple;

typedef struct
{
    uint8_t state;            /* prc_vertex_norm_case_t values */
    uint8_t all_same;
    uint8_t has_first;
    prc_vec3 first_normal;
    prc_vec3 single_decoded;
    uint32_t num_stored;
    uint32_t cap;
    prc_vec3 *slot_input;     /* input normal that created each stored slot */
    prc_vec3 *slot_decoded;   /* decoder-visible vector each slot resolves to */
} prc_encode_point_norm;

static prc_vec3
prc_encode_decoded_vec(const prc_encode_traversal_result *trav, int32_t point_index)
{
    prc_vec3 v;

    v.x = trav->decoded_positions[(size_t)point_index * 3 + 0];
    v.y = trav->decoded_positions[(size_t)point_index * 3 + 1];
    v.z = trav->decoded_positions[(size_t)point_index * 3 + 2];
    return v;
}

static int
prc_encode_check_trav_arrays(prc_context *ctx, const prc_encode_mesh *mesh,
    const prc_encode_traversal_result *trav)
{
    uint32_t k, num_tris;

    if (mesh == NULL || trav == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "normal encoding: NULL mesh/traversal\n");
        return PRC_ERROR_INTERNAL;
    }
    num_tris = trav->edge_status_array_size;
    if (num_tris == 0)
        return 0;
    if (trav->triangle_point_indices == NULL || trav->triangle_mesh_order == NULL ||
        trav->point_mesh_vertex == NULL || trav->decoded_positions == NULL ||
        trav->edge_status_array == NULL || trav->num_decoded_points == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "normal encoding: incomplete traversal result\n");
        return PRC_ERROR_INTERNAL;
    }
    /* edge_status_array_size / num_decoded_points / mesh counts are
       independently supplied; validate every cross-index before any of the
       encoding passes below dereferences through them. */
    for (k = 0; k < num_tris * 3; k++)
    {
        int32_t pt = trav->triangle_point_indices[k];
        int32_t mv;

        if (pt < 0 || (uint32_t)pt >= trav->num_decoded_points)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "normal encoding: point index out of range\n");
            return PRC_ERROR_INTERNAL;
        }
        mv = trav->point_mesh_vertex[pt];
        if (mv < 0 || (uint32_t)mv >= mesh->num_positions)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "normal encoding: mesh vertex out of range\n");
            return PRC_ERROR_INTERNAL;
        }
    }
    for (k = 0; k < num_tris; k++)
    {
        if (trav->triangle_mesh_order[k] >= mesh->num_triangles)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "normal encoding: mesh triangle out of range\n");
            return PRC_ERROR_INTERNAL;
        }
    }
    return 0;
}

int
prc_encode_normals_c1(prc_context *ctx, const prc_encode_mesh *mesh,
    const prc_encode_traversal_result *trav, const double *input_normals,
    uint8_t **normal_is_reversed_out)
{
    uint8_t *rev;
    uint32_t k, num_tris;
    int code;

    if (normal_is_reversed_out == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c1: NULL output\n");
        return PRC_ERROR_INTERNAL;
    }
    *normal_is_reversed_out = NULL;
    code = prc_encode_check_trav_arrays(ctx, mesh, trav);
    if (code < 0)
        return code;
    num_tris = trav->edge_status_array_size;
    if (num_tris == 0)
        return 0;

    rev = (uint8_t *)prc_calloc(ctx, num_tris, sizeof(uint8_t));
    if (rev == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_normals_c1\n");
        return PRC_ERROR_MEMORY;
    }

    /* A fully isolated triangle -- the ONLY member of its connected
       component, with no neighbor anywhere in the mesh -- needs its
       reversal bit set even when normals are recalculated rather than
       supplied: the decoder's prc_store_triangle_indices swaps the output
       vertex order to (idx0, idx2, idx1) whenever this bit is 0
       (prc_decode_compressed_tess.c's prc_store_triangle_indices), which
       silently flips the triangle's winding relative to the caller's
       original tri_indices order unless the bit says otherwise. Confirmed
       the hard way: a lone triangle round-tripped through nanoPRC's own
       decoder with tri_indices==(0,1,2) as supplied came back as (0,2,1)
       -- an inverted winding -- while a real, independently-produced
       compressed PRC file for the identical triangle set this bit TRUE.

       This is deliberately identified via connected-component size
       (mesh->tri_component/num_components), NOT trav->edge_status_array[k]
       == 0: an ordinary "leaf" triangle deep inside a larger connected
       mesh -- one whose remaining edges just happen to already have
       treated neighbors by the time the traversal reaches it -- also gets
       edge_status_array[k] == 0, despite having real neighbors. An
       earlier version of this fix used that condition directly and broke
       test_cube_c1_roundtrip's explicit "every rev[k] == 0" expectation
       for the (fully connected, no isolated triangles) 12-triangle cube.

       This does NOT generalize to triangles with a neighbor (chain start
       later grown into, or a grow step itself): a real, independently-
       produced two-triangle quad (chain start WITH a neighbor, plus its
       grow step) has this bit TRUE on both triangles too, and
       prc_set_left_right_edge_indices's "if (normal_was_reversed) swap
       left and right bases" shows the decoder does have real support for
       that combination -- but blindly setting rev[k]=1 for every triangle
       breaks test_quad_roundtrip's decoded VERTEX POSITIONS (not just
       winding), confirming the encoder's own grow-step basis/point math
       (prc_encode_edge_basis and friends) does not yet produce correct
       point data to pair with a reversed bit on a growing triangle -- a
       real, currently-unimplemented gap, not merely an overcautious
       restriction. Left at the calloc'd default (0) for any triangle with
       a real neighbor until that's fixed; the "reversed && growing"
       restriction a few lines down (input_normals != NULL branch only)
       remains in place for the supplied-normals path. */
    if (mesh->num_components > 0)
    {
        uint32_t *component_size = (uint32_t *)prc_calloc(ctx, mesh->num_components, sizeof(uint32_t));

        if (component_size == NULL)
        {
            prc_free(ctx, rev);
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_normals_c1\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < mesh->num_triangles; k++)
            component_size[mesh->tri_component[k]]++;
        for (k = 0; k < num_tris; k++)
        {
            uint32_t orig_tri = trav->triangle_mesh_order[k];

            if (component_size[mesh->tri_component[orig_tri]] == 1)
                rev[k] = 1;
            if (ctx->trace_reversed)
                fprintf(stderr, "RC_DIAG_COMPSIZE k=%u tri=%u comp=%u compsize=%u rev=%u\n",
                    k, orig_tri, mesh->tri_component[orig_tri],
                    component_size[mesh->tri_component[orig_tri]], rev[k]);
        }
        prc_free(ctx, component_size);
    }

    /* DIAGNOSTIC (2026-07-22, PRC_DIAG_COUNT_REVERSED): input_normals==NULL
       (must_recalculate_normals path) never computes a per-triangle
       reversed-bit decision for growing (non-isolated) triangles -- it's
       unconditionally left at the calloc'd 0 (see the long comment above on
       mesh->num_components). This block does NOT change rev[] -- it only
       measures how often the SAME dot-product test used below for the
       supplied-normals case would have wanted rev[k]=1 for a growing
       triangle, using a smooth per-mesh-vertex normal (equal-weighted
       average of adjacent MESH-order face normals, i.e. a proxy for what
       the decoder's own crease-angle smoothing will approximately
       reconstruct) as the stand-in "input normal". Purely additive/read-
       only; does not affect encoder output. */
    if (input_normals == NULL && prc_diag_getenv("PRC_DIAG_COUNT_REVERSED") != NULL)
    {
        prc_vec3 *vertex_normal = (prc_vec3 *)prc_calloc(ctx, mesh->num_positions, sizeof(prc_vec3));
        uint32_t growing = 0, would_want_rev = 0;

        if (vertex_normal != NULL)
        {
            for (k = 0; k < mesh->num_triangles; k++)
            {
                uint32_t i0 = mesh->tri_indices[(size_t)k * 3 + 0];
                uint32_t i1 = mesh->tri_indices[(size_t)k * 3 + 1];
                uint32_t i2 = mesh->tri_indices[(size_t)k * 3 + 2];
                prc_vec3 Q0, Q1, Q2, f1, f2, fn;

                Q0.x = mesh->positions[(size_t)i0 * 3 + 0]; Q0.y = mesh->positions[(size_t)i0 * 3 + 1]; Q0.z = mesh->positions[(size_t)i0 * 3 + 2];
                Q1.x = mesh->positions[(size_t)i1 * 3 + 0]; Q1.y = mesh->positions[(size_t)i1 * 3 + 1]; Q1.z = mesh->positions[(size_t)i1 * 3 + 2];
                Q2.x = mesh->positions[(size_t)i2 * 3 + 0]; Q2.y = mesh->positions[(size_t)i2 * 3 + 1]; Q2.z = mesh->positions[(size_t)i2 * 3 + 2];
                prc_vec_sub(Q1, Q0, &f1);
                prc_vec_sub(Q2, Q0, &f2);
                prc_vec_cross(f1, f2, &fn);
                vertex_normal[i0].x += fn.x; vertex_normal[i0].y += fn.y; vertex_normal[i0].z += fn.z;
                vertex_normal[i1].x += fn.x; vertex_normal[i1].y += fn.y; vertex_normal[i1].z += fn.z;
                vertex_normal[i2].x += fn.x; vertex_normal[i2].y += fn.y; vertex_normal[i2].z += fn.z;
            }

            for (k = 0; k < num_tris; k++)
            {
                const int32_t *idx = &trav->triangle_point_indices[(size_t)k * 3];
                prc_vec3 P0 = prc_encode_decoded_vec(trav, idx[0]);
                prc_vec3 P1 = prc_encode_decoded_vec(trav, idx[1]);
                prc_vec3 P2 = prc_encode_decoded_vec(trav, idx[2]);
                prc_vec3 e1, e2, raw_cross, avg;
                uint32_t c;
                double dot_val;

                if (trav->edge_status_array[k] == 0)
                    continue; /* leaf/chain-start, not a "growing" triangle */
                growing++;

                prc_vec_sub(P1, P0, &e1);
                prc_vec_sub(P2, P0, &e2);
                prc_vec_cross(e1, e2, &raw_cross);

                avg.x = avg.y = avg.z = 0.0;
                for (c = 0; c < 3; c++)
                {
                    uint32_t mv = (uint32_t)trav->point_mesh_vertex[idx[c]];
                    avg.x += vertex_normal[mv].x / 3.0;
                    avg.y += vertex_normal[mv].y / 3.0;
                    avg.z += vertex_normal[mv].z / 3.0;
                }
                dot_val = prc_vec_dot_product(avg, raw_cross);
                if (dot_val > 0.0)
                    would_want_rev++;
            }
            printf("PRC_DIAG_COUNT_REVERSED: growing_triangles=%u would_want_rev=%u (currently forced to 0)\n",
                growing, would_want_rev);
            prc_free(ctx, vertex_normal);
        }
    }

    if (input_normals != NULL)
    {
        for (k = 0; k < num_tris; k++)
        {
            const int32_t *idx = &trav->triangle_point_indices[(size_t)k * 3];
            prc_vec3 P0 = prc_encode_decoded_vec(trav, idx[0]);
            prc_vec3 P1 = prc_encode_decoded_vec(trav, idx[1]);
            prc_vec3 P2 = prc_encode_decoded_vec(trav, idx[2]);
            prc_vec3 e1, e2, raw_cross, avg;
            uint32_t c;
            double dot_val;

            prc_vec_sub(P1, P0, &e1);
            prc_vec_sub(P2, P0, &e2);
            prc_vec_cross(e1, e2, &raw_cross);

            avg.x = avg.y = avg.z = 0.0;
            for (c = 0; c < 3; c++)
            {
                const double *n = &input_normals[(size_t)trav->point_mesh_vertex[idx[c]] * 3];

                avg.x += n[0] / 3.0;
                avg.y += n[1] / 3.0;
                avg.z += n[2] / 3.0;
            }
            dot_val = prc_vec_dot_product(avg, raw_cross);
            /* prc_derive_normal negates its cross product when the stored bit
               is 0, so bit = 1 selects the un-negated (+cross) direction.
               Empirically verified against the real decoder by the
               single-triangle round-trip cases in test_compress_tess.c. */
            rev[k] = (uint8_t)(dot_val > 0.0);
            /* A set bit makes the decoder swap its left/right edge handling
               for this triangle's grow pushes, which the already-emitted
               traversal arrays assumed never happens -- the encoder's own
               grow-step point/basis math doesn't yet produce correct point
               data to pair with a reversed bit on a growing triangle (see
               the longer comment above on mesh->num_components). Rather
               than discard every OTHER triangle's data-driven bit over
               this one triangle, leave this triangle at the calloc
               default (0) -- same as the no-input-normals case -- and
               keep going; its decoded normal sign may end up wrong, but
               its decoded position/topology stays correct either way. */
            if (rev[k] && trav->edge_status_array[k] != 0)
                rev[k] = 0;
            if (ctx->trace_reversed)
                fprintf(stderr, "RC_DIAG_REV k=%u tri=%u rev=%u edge_status=%u dot_val=%.9f comp=%u\n",
                    k, trav->triangle_mesh_order[k], rev[k], trav->edge_status_array[k], dot_val,
                    mesh->tri_component ? mesh->tri_component[trav->triangle_mesh_order[k]] : 0xFFFFFFFFu);
        }
    }
    *normal_is_reversed_out = rev;
    return 0;
}

/* Mirror of the basis phase of the decoder's prc_compute_vertex_normal,
   BEFORE any reversal bit is applied: the decoder negates Z first and only
   then derives Y = normalize(cross(Z, X)), and normalize(-v) == -normalize(v)
   exactly, so the reversed variants differ from this raw basis only by signs
   the tuple's tri/x/y bits reproduce. Same helpers, same operation order,
   including the prc_vec_make_orth_basis_normals fallback. */
static int
prc_encode_vertex_normal_basis(prc_vec3 V1, prc_vec3 V2, prc_vec3 V3,
    prc_vec3 *x_out, prc_vec3 *y_out, prc_vec3 *z_out)
{
    prc_vec3 V1_norm, V2_norm, V3_norm, temp1, temp2;
    prc_vec3 X_norm, Y_norm, Z_norm;
    double theta1 = 0, theta2 = 0, theta3 = 0;
    int code;

    prc_vec_sub(V2, V1, &V1_norm);
    code = prc_vec_normalize(&V1_norm);
    if (code < 0)
        return code;
    prc_vec_sub(V3, V1, &V2_norm);
    code = prc_vec_normalize(&V2_norm);
    if (code < 0)
        return code;
    prc_vec_sub(V3, V2, &V3_norm);
    code = prc_vec_normalize(&V3_norm);
    if (code < 0)
        return code;

    code = prc_vec_angle_between_vectors_normal(V1_norm, V2_norm, &theta1);
    if (code < 0)
        return code;
    prc_vec_copy(V1_norm, &temp1, 1);
    code = prc_vec_angle_between_vectors_normal(V3_norm, temp1, &theta2);
    if (code < 0)
        return code;
    prc_vec_copy(V2_norm, &temp1, 1);
    prc_vec_copy(V3_norm, &temp2, 1);
    code = prc_vec_angle_between_vectors_normal(temp1, temp2, &theta3);
    if (code < 0)
        return code;

    if ((theta1 < theta2) && (theta1 < theta3))
    {
        prc_vec_copy(V1_norm, &X_norm, 0);
        prc_vec_cross(V1_norm, V2_norm, &Z_norm);
    }
    else if (theta2 < theta3)
    {
        prc_vec_copy(V3_norm, &X_norm, 0);
        prc_vec_copy(V3_norm, &temp1, 1);
        prc_vec_cross(temp1, V1_norm, &Z_norm);
    }
    else
    {
        prc_vec_copy(V2_norm, &X_norm, 1);
        prc_vec_cross(V2_norm, V3_norm, &Z_norm);
    }

    code = prc_vec_normalize(&Z_norm);
    if (code < 0)
    {
        prc_basis basis;

        basis.X = X_norm;
        code = prc_vec_make_orth_basis_normals(&basis);
        if (code < 0)
            return code;
        Z_norm = basis.Z;
    }

    prc_vec_cross(Z_norm, X_norm, &Y_norm);
    code = prc_vec_normalize(&Y_norm);
    if (code < 0)
        return code;

    *x_out = X_norm;
    *y_out = Y_norm;
    *z_out = Z_norm;
    return 0;
}

static int32_t
prc_encode_quantize_angle(double angle)
{
    long long q = llround(angle * (double)PRC_ENCODE_NORMAL_ANGLE_MAX / PRC_HALF_PI);

    if (q < 0)
        q = 0;
    if (q > PRC_ENCODE_NORMAL_ANGLE_MAX)
        q = PRC_ENCODE_NORMAL_ANGLE_MAX;
    return (int32_t)q;
}

/* Invert the decoder's reconstruction
     n = cos(theta)cos(phi)*Xf + sin(theta)cos(phi)*Yf + sin(phi)*Zf
   where Zf = +-Z (tri bit), Yf = +-cross(Zf, X) (y bit), Xf = +-X (x bit) and
   theta, phi are stored unsigned in [0, PI/2]: project N onto the raw basis
   and fold every sign into the three bits. */
static void
prc_encode_project_normal(prc_vec3 N, prc_vec3 X, prc_vec3 Y, prc_vec3 Z,
    prc_encode_normal_tuple *t)
{
    double nx = prc_vec_dot_product(N, X);
    double ny = prc_vec_dot_product(N, Y);
    double nz = prc_vec_dot_product(N, Z);
    double az = fabs(nz);

    t->x_reversed = (uint8_t)(nx < 0.0);
    t->tri_reversed = (uint8_t)(nz < 0.0);
    t->y_reversed = (uint8_t)((ny < 0.0) != (nz < 0.0));
    if (az > 1.0)
        az = 1.0;
    t->theta_q = prc_encode_quantize_angle(atan2(fabs(ny), fabs(nx)));
    t->phi_q = prc_encode_quantize_angle(asin(az));
}

/* The vector the decoder will actually store for this tuple, needed both to
   predict its normal_was_reversed decision and to know what a back-reference
   would resolve to. */
static void
prc_encode_simulate_decoded_normal(prc_vec3 X, prc_vec3 Y, prc_vec3 Z,
    const prc_encode_normal_tuple *t, prc_vec3 *out)
{
    double scale = PRC_HALF_PI / (double)PRC_ENCODE_NORMAL_ANGLE_MAX;
    double theta = (double)t->theta_q * scale;
    double phi = (double)t->phi_q * scale;
    double f1 = cos(theta) * cos(phi);
    double f2 = sin(theta) * cos(phi);
    double f3 = sin(phi);

    if (t->tri_reversed)
    {
        prc_vec_negate(&Z);
        prc_vec_negate(&Y);
    }
    if (t->x_reversed)
        prc_vec_negate(&X);
    if (t->y_reversed)
        prc_vec_negate(&Y);

    out->x = f1 * X.x + f2 * Y.x + f3 * Z.x;
    out->y = f1 * X.y + f2 * Y.y + f3 * Z.y;
    out->z = f1 * X.z + f2 * Y.z + f3 * Z.z;
    (void)prc_vec_normalize(out);
}

static int
prc_encode_point_norm_append(prc_context *ctx, prc_encode_point_norm *p,
    prc_vec3 input_n, prc_vec3 decoded_n)
{
    if (p->num_stored == p->cap)
    {
        uint32_t new_cap = p->cap ? p->cap * 2 : 4;
        prc_vec3 *grown;

        grown = (prc_vec3 *)prc_realloc(ctx, p->slot_input, (size_t)new_cap * sizeof(prc_vec3));
        if (grown == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_point_norm_append\n");
            return PRC_ERROR_MEMORY;
        }
        p->slot_input = grown;
        grown = (prc_vec3 *)prc_realloc(ctx, p->slot_decoded, (size_t)new_cap * sizeof(prc_vec3));
        if (grown == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_point_norm_append\n");
            return PRC_ERROR_MEMORY;
        }
        p->slot_decoded = grown;
        p->cap = new_cap;
    }
    p->slot_input[p->num_stored] = input_n;
    p->slot_decoded[p->num_stored] = decoded_n;
    p->num_stored++;
    return 0;
}

int
prc_encode_normals_c2(prc_context *ctx, const prc_encode_mesh *mesh,
    const prc_encode_traversal_result *trav, const double *corner_normals,
    int32_t **normal_angle_array_out, uint32_t *normal_angle_count_out,
    uint8_t **normal_binary_data_out, uint32_t *normal_binary_data_size_out)
{
    uint32_t num_tris, npts, k, c, v;
    prc_vec3 *visit_normals = NULL;
    prc_vec3 *bx = NULL, *by = NULL, *bz = NULL;
    prc_encode_normal_tuple *tuples = NULL;
    prc_encode_point_norm *pn = NULL;
    int32_t *angles = NULL;
    uint8_t *bin = NULL;
    uint32_t angle_count = 0, bin_count = 0;
    int ret = PRC_ERROR_INTERNAL;
    int code;

    if (normal_angle_array_out == NULL || normal_angle_count_out == NULL ||
        normal_binary_data_out == NULL || normal_binary_data_size_out == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c2: NULL output\n");
        return PRC_ERROR_INTERNAL;
    }
    *normal_angle_array_out = NULL;
    *normal_angle_count_out = 0;
    *normal_binary_data_out = NULL;
    *normal_binary_data_size_out = 0;

    code = prc_encode_check_trav_arrays(ctx, mesh, trav);
    if (code < 0)
        return code;
    num_tris = trav->edge_status_array_size;
    if (num_tris == 0)
        return 0;
    if (corner_normals == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c2: NULL corner normals\n");
        return PRC_ERROR_INTERNAL;
    }
    npts = trav->num_decoded_points;

    visit_normals = (prc_vec3 *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(prc_vec3));
    bx = (prc_vec3 *)prc_malloc(ctx, (size_t)num_tris * sizeof(prc_vec3));
    by = (prc_vec3 *)prc_malloc(ctx, (size_t)num_tris * sizeof(prc_vec3));
    bz = (prc_vec3 *)prc_malloc(ctx, (size_t)num_tris * sizeof(prc_vec3));
    tuples = (prc_encode_normal_tuple *)prc_malloc(ctx, (size_t)num_tris * 3 * sizeof(prc_encode_normal_tuple));
    pn = (prc_encode_point_norm *)prc_calloc(ctx, npts, sizeof(prc_encode_point_norm));
    angles = (int32_t *)prc_malloc(ctx, (size_t)num_tris * 6 * sizeof(int32_t));
    /* Per-visit worst case: 4 fresh bits, or 1 reference bit plus at most 32
       back-reference index bits. */
    bin = (uint8_t *)prc_malloc(ctx, (size_t)num_tris * 3 * 40 * sizeof(uint8_t));
    if (visit_normals == NULL || bx == NULL || by == NULL || bz == NULL ||
        tuples == NULL || pn == NULL || angles == NULL || bin == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_encode_normals_c2\n");
        ret = PRC_ERROR_MEMORY;
        goto fail;
    }

    for (k = 0; k < num_tris; k++)
    {
        const int32_t *idx = &trav->triangle_point_indices[(size_t)k * 3];
        uint32_t mesh_tri = trav->triangle_mesh_order[k];

        code = prc_encode_vertex_normal_basis(prc_encode_decoded_vec(trav, idx[0]),
            prc_encode_decoded_vec(trav, idx[1]), prc_encode_decoded_vec(trav, idx[2]),
            &bx[k], &by[k], &bz[k]);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c2: degenerate triangle basis\n");
            goto fail;
        }

        for (c = 0; c < 3; c++)
        {
            int32_t mv = trav->point_mesh_vertex[idx[c]];
            uint32_t j, corner = 3;
            prc_vec3 n;

            for (j = 0; j < 3; j++)
            {
                if (mesh->tri_indices[(size_t)mesh_tri * 3 + j] == (uint32_t)mv)
                    corner = j;
            }
            if (corner == 3)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c2: traversal/mesh corner mismatch\n");
                goto fail;
            }
            n.x = corner_normals[(size_t)mesh_tri * 9 + (size_t)corner * 3 + 0];
            n.y = corner_normals[(size_t)mesh_tri * 9 + (size_t)corner * 3 + 1];
            n.z = corner_normals[(size_t)mesh_tri * 9 + (size_t)corner * 3 + 2];
            if (prc_vec_normalize(&n) < 0)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "prc_encode_normals_c2: zero-length input normal\n");
                goto fail;
            }
            visit_normals[(size_t)k * 3 + c] = n;
            prc_encode_project_normal(n, bx[k], by[k], bz[k], &tuples[(size_t)k * 3 + c]);
        }
    }

    /* has_multiple_normals must be fixed for a point's entire lifetime at its
       first visit, so decide it up front: only if every incident visit wants
       the same input normal can later visits legally consume zero bits. */
    for (v = 0; v < npts; v++)
        pn[v].all_same = 1;
    for (k = 0; k < num_tris * 3; k++)
    {
        int32_t pt = trav->triangle_point_indices[k];

        if (!pn[pt].has_first)
        {
            pn[pt].first_normal = visit_normals[k];
            pn[pt].has_first = 1;
        }
        else if (prc_vec_dot_product(pn[pt].first_normal, visit_normals[k]) < 1.0 - 1.0e-9)
        {
            pn[pt].all_same = 0;
        }
    }

    for (k = 0; k < num_tris; k++)
    {
        const int32_t *idx = &trav->triangle_point_indices[(size_t)k * 3];
        prc_vec3 corner0_decoded;

        corner0_decoded.x = corner0_decoded.y = corner0_decoded.z = 0.0;
        for (c = 0; c < 3; c++)
        {
            uint32_t visit = k * 3 + c;
            const prc_encode_normal_tuple *t = &tuples[visit];
            prc_encode_point_norm *p = &pn[idx[c]];
            prc_vec3 assigned;

            if (p->state == PRC_VERTEX_NORM_NOT_ENCOUNTERED)
            {
                uint8_t hm = (uint8_t)!p->all_same;

                bin[bin_count++] = hm;
                bin[bin_count++] = t->tri_reversed;
                bin[bin_count++] = t->x_reversed;
                bin[bin_count++] = t->y_reversed;
                angles[angle_count++] = t->theta_q;
                angles[angle_count++] = t->phi_q;
                prc_encode_simulate_decoded_normal(bx[k], by[k], bz[k], t, &assigned);
                if (hm)
                {
                    p->state = PRC_VERTEX_NORM_IS_MULTIPLE;
                    code = prc_encode_point_norm_append(ctx, p, visit_normals[visit], assigned);
                    if (code < 0)
                    {
                        ret = code;
                        goto fail;
                    }
                }
                else
                {
                    p->state = PRC_VERTEX_NORM_IS_NOT_MULTIPLE;
                    p->single_decoded = assigned;
                }
            }
            else if (p->state == PRC_VERTEX_NORM_IS_NOT_MULTIPLE)
            {
                /* the decoder reuses the single stored normal without reading
                   any bits; valid because all_same guaranteed every incident
                   visit wants this normal */
                assigned = p->single_decoded;
            }
            else
            {
                uint32_t s, found = UINT32_MAX;

                for (s = 0; s < p->num_stored; s++)
                {
                    if (prc_vec_dot_product(p->slot_input[s], visit_normals[visit]) > 1.0 - 1.0e-9)
                    {
                        found = s;
                        break;
                    }
                }
                if (found != UINT32_MAX)
                {
                    /* inverse of the decoder's
                       ref_index = (num_stored - 1) - read_index, emitted
                       LSB-first over exactly the bit width the decoder will
                       compute from its own num_stored */
                    uint32_t number_bits = get_number_bits_to_store_unsigned_integer2(p->num_stored - 1);
                    uint32_t read_index = (p->num_stored - 1) - found;
                    uint32_t b;

                    bin[bin_count++] = 1;
                    for (b = 0; b < number_bits; b++)
                        bin[bin_count++] = (uint8_t)((read_index >> b) & 1u);
                    assigned = p->slot_decoded[found];
                }
                else
                {
                    bin[bin_count++] = 0;
                    bin[bin_count++] = t->tri_reversed;
                    bin[bin_count++] = t->x_reversed;
                    bin[bin_count++] = t->y_reversed;
                    angles[angle_count++] = t->theta_q;
                    angles[angle_count++] = t->phi_q;
                    prc_encode_simulate_decoded_normal(bx[k], by[k], bz[k], t, &assigned);
                    code = prc_encode_point_norm_append(ctx, p, visit_normals[visit], assigned);
                    if (code < 0)
                    {
                        ret = code;
                        goto fail;
                    }
                }
            }
            if (ctx->trace_normals)
            {
                prc_vec3 pos = prc_encode_decoded_vec(trav, idx[c]);
                fprintf(stderr, "ENCNORM k=%u c=%u pt=%d rev=%u xrev=%u yrev=%u theta=%d phi=%d input_normal=(%.6f,%.6f,%.6f) assigned=(%.6f,%.6f,%.6f) pos=(%.6f,%.6f,%.6f)\n",
                    k, c, idx[c], t->tri_reversed, t->x_reversed, t->y_reversed, t->theta_q, t->phi_q,
                    visit_normals[visit].x, visit_normals[visit].y, visit_normals[visit].z,
                    assigned.x, assigned.y, assigned.z, pos.x, pos.y, pos.z);
            }
            if (c == 0)
                corner0_decoded = assigned;
        }

        /* The decoder derives normal_was_reversed from the corner-0 normal
           (prc_is_normal_reversed_single_normal) and, when set, swaps its
           left/right edge handling for this triangle's grow pushes.
           prc_encode_traversal now decides each triangle's normal_was_reversed
           bit inline (prc_encode_decide_reversed, using TRAVERSAL corner 0's
           specific real supplied normal from real_normals/corner_normals --
           the same corner-0-specific convention this check uses) in the SAME
           pass that builds edge_status_array/push order, and prc_encode_edge_status
           already applied the matching left/right swap for growing triangles
           -- reversed=1 on a growing triangle is now a fully supported,
           already-correctly-handled case, NOT something to reject outright
           (an earlier version of this check predated that support and
           rejected unconditionally, which is why it rejected on essentially
           every real mesh's first growing triangle).

           What actually needs checking is DISAGREEMENT: did the traversal's
           decision (from the RAW real corner-0 normal) come out different
           from what this test derives (from the round-tripped DECODED
           corner-0 normal, after the Taylor-angle quantize/reconstruct
           simulation above)? Those two normally agree exactly, but
           quantization noise could in principle flip the sign for a
           triangle whose real normal is very close to perpendicular to its
           own face -- kept as a canary for that case, not deleted. A hit
           here means the decoder will walk this triangle's left/right
           edges opposite to what the wire format's topology arrays assume,
           a real desync -- surface it, don't just log it and continue,
           until this has been confirmed never to fire on real meshes. */
        {
            prc_vec3 P0 = prc_encode_decoded_vec(trav, idx[0]);
            prc_vec3 P1 = prc_encode_decoded_vec(trav, idx[1]);
            prc_vec3 P2 = prc_encode_decoded_vec(trav, idx[2]);
            prc_vec3 mid, e1, e2, cross1;
            uint8_t derived_reversed;

            prc_vec_avg(P0, P1, &mid);
            prc_vec_sub(P1, P0, &e1);
            prc_vec_sub(P2, mid, &e2);
            prc_vec_cross(e2, e1, &cross1);
            derived_reversed = (uint8_t)(prc_vec_dot_product(cross1, corner0_decoded) < 0.0);
            if (derived_reversed != trav->triangle_reversed[k] &&
                trav->edge_status_array[k] != 0)
            {
                fprintf(stderr, "prc_encode_normals_c2: WARNING corner-0 DECODED normal's reversed-bit "
                    "(%u) disagrees with the traversal's own inline decision (%u, from the RAW real "
                    "normal) on growing triangle %u -- canary check, see comment above\n",
                    derived_reversed, trav->triangle_reversed[k], k);
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_encode_normals_c2: normals reverse a growing triangle (unsupported)\n");
                goto fail;
            }
        }
    }

    if (angle_count > 0 && (size_t)angle_count < (size_t)num_tris * 6)
    {
        int32_t *shrunk = (int32_t *)prc_realloc(ctx, angles, (size_t)angle_count * sizeof(int32_t));

        if (shrunk != NULL)
            angles = shrunk;
    }
    if (bin_count > 0 && (size_t)bin_count < (size_t)num_tris * 3 * 40)
    {
        uint8_t *shrunk = (uint8_t *)prc_realloc(ctx, bin, (size_t)bin_count * sizeof(uint8_t));

        if (shrunk != NULL)
            bin = shrunk;
    }
    *normal_angle_array_out = angles;
    *normal_angle_count_out = angle_count;
    *normal_binary_data_out = bin;
    *normal_binary_data_size_out = bin_count;
    angles = NULL;
    bin = NULL;
    ret = 0;

fail:
    if (angles != NULL)
        prc_free(ctx, angles);
    if (bin != NULL)
        prc_free(ctx, bin);
    if (visit_normals != NULL)
        prc_free(ctx, visit_normals);
    if (bx != NULL)
        prc_free(ctx, bx);
    if (by != NULL)
        prc_free(ctx, by);
    if (bz != NULL)
        prc_free(ctx, bz);
    if (tuples != NULL)
        prc_free(ctx, tuples);
    if (pn != NULL)
    {
        for (v = 0; v < npts; v++)
        {
            if (pn[v].slot_input != NULL)
                prc_free(ctx, pn[v].slot_input);
            if (pn[v].slot_decoded != NULL)
                prc_free(ctx, pn[v].slot_decoded);
        }
        prc_free(ctx, pn);
    }
    return ret;
}

/* ---- Step E: bitstream assembly ---------------------------------------- */

int
prc_write_compress_tess_to_stream(prc_context *ctx, prc_bit_write_state *state,
    const prc_encode_traversal_result *trav, double tolerance_mm,
    const uint8_t *normal_is_reversed_c1, double crease_angle_degrees,
    const int32_t *normal_angle_array, uint32_t normal_angle_array_count,
    const uint8_t *normal_binary_data, uint32_t normal_binary_data_size,
    uint8_t must_recalculate_normals, const uint8_t *is_face_planar)
{
    uint32_t k, num_refs = 0;

    if (state == NULL || trav == NULL || !(tolerance_mm > 0.0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: bad arguments\n");
        return PRC_ERROR_INTERNAL;
    }
    if ((trav->point_array == NULL && trav->point_array_size != 0) ||
        (trav->edge_status_array == NULL && trav->edge_status_array_size != 0) ||
        (trav->triangle_face_array == NULL && trav->triangle_face_array_size != 0) ||
        (trav->points_is_reference_array == NULL && trav->points_is_reference_array_size != 0) ||
        (trav->point_reference_array == NULL && trav->point_reference_array_size != 0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: NULL array with non-zero count\n");
        return PRC_ERROR_INTERNAL;
    }
    if (must_recalculate_normals)
    {
        if (normal_is_reversed_c1 == NULL && trav->triangle_face_array_size != 0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: missing C1 reversal bits\n");
            return PRC_ERROR_INTERNAL;
        }
    }
    else if ((normal_angle_array == NULL && normal_angle_array_count != 0) ||
        (normal_binary_data == NULL && normal_binary_data_size != 0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: missing C2 normal data\n");
        return PRC_ERROR_INTERNAL;
    }

    for (k = 0; k < trav->points_is_reference_array_size; k++)
    {
        if (trav->points_is_reference_array[k])
            num_refs++;
    }
    /* The reader sizes point_reference_array by counting these 1-bits, so a
       mismatch would silently desynchronize everything after it. */
    if (num_refs != trav->point_reference_array_size)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: reference bookkeeping mismatch\n");
        return PRC_ERROR_INTERNAL;
    }

#define PRC_DIAG_TESS_FIELD_SIZES_MARK(label) \
    do { if (prc_diag_getenv("PRC_DIAG_TESS_FIELD_SIZES") != NULL) \
        fprintf(stderr, "PRC_DIAG_TESS_FIELD_SIZES: after %-24s byte_pos=%zu bit_fill=%u\n", \
            (label), state->byte_pos, (unsigned)state->bit_fill); } while (0)

    PRC_DIAG_TESS_FIELD_SIZES_MARK("start");

    if (prc_bitwrite_bit(ctx, state, 0) != 0)   /* is_calculated */
        goto werr;
    /* has_faces: per the spec (Table 175), "TRUE if the entity is built
       using geometrical faces". A prior revision of this comment argued
       for always FALSE, based on a locally-converted reference file that
       had real multi-value triangle_face_array data alongside has_faces ==
       FALSE. A genuinely third-party, in-the-wild, real-world Acrobat-
       targeted 3D PDF (unlike that locally-converted reference) shows the
       opposite: has_faces == TRUE alongside its own real, multi-face
       triangle_face_array data -- and, while not proven in isolation as
       the sole fix, this is the only remaining value-level discrepancy
       found against that reference after every other section of this
       write facility's output was independently confirmed byte/structure-
       correct via extensive splice testing against the same file. Setting
       it FALSE while genuinely emitting face-indexed triangle data is
       also simply self-contradictory on its face ("no faces" plus 2048
       faces of triangle data in the very next arrays). */
    if (prc_bitwrite_bit(ctx, state, 1) != 0)   /* has_faces */
        goto werr;
    if (prc_bitwrite_double(ctx, state, tolerance_mm) != 0)
        goto werr;
    /* float per the format; the reader applies the same truncation */
    if (prc_bitwrite_float(ctx, state, (float)trav->origin[0]) != 0)
        goto werr;
    if (prc_bitwrite_float(ctx, state, (float)trav->origin[1]) != 0)
        goto werr;
    if (prc_bitwrite_float(ctx, state, (float)trav->origin[2]) != 0)
        goto werr;
    PRC_DIAG_TESS_FIELD_SIZES_MARK("tolerance+origin");
    if (prc_bitwrite_compressed_integer_array(ctx, state, trav->point_array,
            trav->point_array_size) != 0)
        goto werr;
    PRC_DIAG_TESS_FIELD_SIZES_MARK("point_array");
    /* edge_status_array is documented (ISO/CD 14739-1 §7.8.9, Table 175/
       CR-14) to hold 3*T entries, not T -- one 2-bit field per triangle is
       the only one a decoder actually consumes (indexed edge_status[t],
       not edge_status[3t]), but entries [T .. 3T-1] must still be present
       on disk as zero padding. Writing only T entries (this write
       facility's own prior behavior) round-trips fine through a reader
       that just trusts the stored count, but is not what the format
       specifies.

       Despite the wording of an earlier version of this comment, this is
       NOT a confirmed-causal Acrobat fix -- a later investigation
       (PRC_DIAG_NO_EDGE_STATUS_PADDING, added to test this exact question
       against real Acrobat blank-tree repros) found disabling the padding
       "tested and not causal" for those bugs, and a direct same-geometry
       A/B test (2026-08-13) found Acrobat accepts unpadded (case-A, 1*T)
       output too. CR-14 itself, the spec citation this fix leans on, was
       submitted by a maintainer of this project based on their own earlier
       observation, not independent external validation, so it isn't
       corroborating evidence on its own either.

       Padding is kept as the default anyway, for two independent reasons
       that held up under scrutiny: (1) a broad real-world corpus census
       (~34,500 COMPRESSED tessellation instances, prc-db) found case-B
       (3*T) outnumbers case-A (1*T) roughly 3:1 in practice: 76.6% vs
       23.4%; (2) Adobe's own PRC support is believed to derive from the
       Tech Soft 3D/HOOPS codebase lineage (the dominant case-B writer
       family), meaning Acrobat's own parser may be exercised far more
       thoroughly against case-B's shape than case-A's, even where both
       are spec-valid. Neither is proof case-A is unsafe, but both point
       the same direction, so case-B stays the lower-risk default. */
    {
        uint32_t t_count = trav->edge_status_array_size;
        uint32_t padded_count = (prc_diag_getenv("PRC_DIAG_NO_EDGE_STATUS_PADDING") != NULL) ? t_count : t_count * 3;
        uint8_t *padded = (uint8_t *)prc_calloc(ctx, padded_count > 0 ? padded_count : 1, sizeof(uint8_t));

        if (padded == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "prc_write_compress_tess_to_stream: allocation error\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(padded, trav->edge_status_array, (size_t)t_count * sizeof(uint8_t));
        if (prc_bitwrite_character_array(ctx, state, padded, padded_count, 2, 1, 0) != 0)
        {
            prc_free(ctx, padded);
            goto werr;
        }
        prc_free(ctx, padded);
    }
    PRC_DIAG_TESS_FIELD_SIZES_MARK("edge_status_array");
    if (prc_bitwrite_compressed_indice_array(ctx, state, trav->triangle_face_array,
            trav->triangle_face_array_size, 1, 0) != 0)
        goto werr;
    PRC_DIAG_TESS_FIELD_SIZES_MARK("triangle_face_array");
    if (prc_bitwrite_uint32(ctx, state, trav->points_is_reference_array_size) != 0)
        goto werr;
    for (k = 0; k < trav->points_is_reference_array_size; k++)
    {
        if (prc_bitwrite_bit(ctx, state, trav->points_is_reference_array[k]) != 0)
            goto werr;
    }
    PRC_DIAG_TESS_FIELD_SIZES_MARK("points_is_reference_array");
    if (prc_bitwrite_compressed_indice_array(ctx, state, trav->point_reference_array,
            trav->point_reference_array_size, 0, num_refs) != 0)
        goto werr;
    PRC_DIAG_TESS_FIELD_SIZES_MARK("point_reference_array");
    if (prc_bitwrite_bit(ctx, state, must_recalculate_normals ? 1 : 0) != 0)
        goto werr;

    /* face_number, i.e. max(triangle_face_array) + 1 -- needed both by the
       C2 is_face_planar array below and by line_attribute_array further
       down, for both C1 and C2. */
    {
        uint32_t face_count = 1;

        for (k = 0; k < trav->triangle_face_array_size; k++)
        {
            if (trav->triangle_face_array[k] >= 0 &&
                (uint32_t)trav->triangle_face_array[k] + 1 > face_count)
                face_count = (uint32_t)trav->triangle_face_array[k] + 1;
        }

        if (must_recalculate_normals)
        {
            for (k = 0; k < trav->triangle_face_array_size; k++)
            {
                if (prc_bitwrite_bit(ctx, state, normal_is_reversed_c1[k]) != 0)
                    goto werr;
            }
            /* stored in degrees; the reader converts to radians itself */
            if (prc_bitwrite_double(ctx, state, crease_angle_degrees) != 0)
                goto werr;
            if (prc_bitwrite_uint8(ctx, state, 0) != 0)   /* normal_recalculation_flags */
                goto werr;
        }
        else
        {
            if (prc_bitwrite_uint8(ctx, state, PRC_ENCODE_NORMAL_ANGLE_BITS) != 0)
                goto werr;
            if (prc_bitwrite_uint32(ctx, state, normal_binary_data_size) != 0)
                goto werr;
            for (k = 0; k < normal_binary_data_size; k++)
            {
                if (prc_bitwrite_bit(ctx, state, normal_binary_data[k]) != 0)
                    goto werr;
            }
            if (prc_bitwrite_short_array(ctx, state, normal_angle_array,
                    normal_angle_array_count, 1, PRC_ENCODE_NORMAL_ANGLE_BITS) != 0)
                goto werr;
            /* is_face_planar: NULL (this write facility's only real
               caller) means every face non-planar, routing all decoding
               through the per-vertex path and never the separate
               per-face-planar normal-sharing path. A caller-supplied
               array (diagnostics re-encoding a real file's own decoded
               planarity) is written as-is instead -- the normal_angle_
               array/normal_binary_data above were generated assuming
               specific faces use the planar shortcut, so silently
               forcing them all non-planar here would desync the
               decoder's normal reconstruction against that data. */
            for (k = 0; k < face_count; k++)
            {
                if (prc_bitwrite_bit(ctx, state, is_face_planar != NULL ? is_face_planar[k] : 0) != 0)
                    goto werr;
            }
        }
        PRC_DIAG_TESS_FIELD_SIZES_MARK("normals_block");

        if (prc_bitwrite_bit(ctx, state, 0) != 0)   /* is_point_color */
            goto werr;
        if (prc_bitwrite_bit(ctx, state, 0) != 0)   /* is_multiple_line_attribute */
            goto werr;
        /* line_attribute_array: read unconditionally by the parser, one
           entry per face (face_count), regardless of is_multiple_line_
           attribute or C1/C2 -- confirmed against a real, independently-
           produced multi-face compressed PRC file (2 distinct faces): its
           line_attribute_array_size was 2 (== face_number), not a single
           global entry, even with is_multiple_line_attribute == FALSE. An
           earlier version of this code wrote either a genuinely empty
           array (C1) or exactly one entry regardless of face count (both
           since corrected): the former is confirmed to be what an
           independent, strict PRC reader rejects outright (returns null
           geometry); the latter was only ever verified against face_
           count == 1 reference files and throws an "invalid vector
           subscript" exception in that same reader once face_count > 1,
           consistent with it expecting one entry per face. Every entry is
           the same no-style (biased 0) placeholder value, since this
           write facility never emits real per-face style data. */
        {
            int32_t *no_style_per_face = (int32_t *)prc_malloc(ctx, (size_t)face_count * sizeof(int32_t));

            if (no_style_per_face == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "prc_write_compress_tess_to_stream: allocation error\n");
                return PRC_ERROR_MEMORY;
            }
            memset(no_style_per_face, 0, (size_t)face_count * sizeof(int32_t));
            if (prc_bitwrite_short_array(ctx, state, no_style_per_face, face_count, 1, 16) != 0)
            {
                prc_free(ctx, no_style_per_face);
                goto werr;
            }
            prc_free(ctx, no_style_per_face);
            PRC_DIAG_TESS_FIELD_SIZES_MARK("line_attribute_array");
        }
    }
    if (prc_bitwrite_bit(ctx, state, 1) != 0)   /* no_texture */
        goto werr;
    if (prc_bitwrite_bit(ctx, state, 0) != 0)   /* has_behaviors */
        goto werr;
    return 0;

werr:
    prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_to_stream: bit write failed\n");
    return PRC_ERROR_INTERNAL;
}

void
prc_encode_preprocess_free(prc_context *ctx, prc_encode_mesh *m)
{
    if (m == NULL)
        return;
    if (m->positions != NULL)
        prc_free(ctx, m->positions);
    if (m->tri_indices != NULL)
        prc_free(ctx, m->tri_indices);
    if (m->tri_orig_index != NULL)
        prc_free(ctx, m->tri_orig_index);
    if (m->edges != NULL)
        prc_free(ctx, m->edges);
    if (m->tri_component != NULL)
        prc_free(ctx, m->tri_component);
    m->positions = NULL;
    m->tri_indices = NULL;
    m->tri_orig_index = NULL;
    m->edges = NULL;
    m->tri_component = NULL;
    m->num_positions = 0;
    m->num_triangles = 0;
    m->num_edges = 0;
    m->num_components = 0;
}

int
prc_api_mesh_has_nonmanifold_fans(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance)
{
    prc_encode_mesh mesh;
    int code;
    int has_fans;

    code = prc_encode_preprocess(ctx, positions, num_positions, tri_indices, num_triangles, tolerance, &mesh);
    if (code < 0)
        return code;
    has_fans = mesh.nonmanifold_vertices > 0;
    prc_encode_preprocess_free(ctx, &mesh);
    return has_fans;
}

int
prc_api_mesh_weld_and_split(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance,
    double **out_positions, uint32_t *out_num_positions,
    uint32_t **out_tri_indices, uint32_t *out_num_triangles)
{
    prc_encode_mesh mesh;
    int code;

    if (out_positions == NULL || out_num_positions == NULL ||
        out_tri_indices == NULL || out_num_triangles == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_api_mesh_weld_and_split: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    /* skip_nonmanifold_edge_remap=1: this function's whole point is to fix
       up a mesh for uncompressed TRIANGLES output, which has no traversal
       algorithm caring how many triangles share an edge -- only the
       fan-vertex split (for smooth-normal-reconstruction correctness) is
       relevant here, not COMPRESSED's own edge-count constraint. Skipping
       it turned out to matter a lot in practice: on one real mechanical
       assembly this was tested against, the edge remap alone added
       ~390,000 extra vertices (a real, if COMPRESSED-specific, mesh-quality
       fixup applied to a file with many 3+-way-shared edges) versus a few
       hundred from the fan split actually wanted here. */
    code = prc_encode_preprocess_ex(ctx, positions, num_positions, tri_indices, num_triangles,
        tolerance, 1, &mesh);
    if (code < 0)
        return code;

    *out_positions = mesh.positions;
    *out_num_positions = mesh.num_positions;
    *out_tri_indices = mesh.tri_indices;
    *out_num_triangles = mesh.num_triangles;

    /* Steal positions/tri_indices ownership for the caller; free everything
       else prc_encode_preprocess allocated (edges, tri_component, etc). */
    mesh.positions = NULL;
    mesh.tri_indices = NULL;
    prc_encode_preprocess_free(ctx, &mesh);
    return 0;
}

void
prc_api_mesh_weld_and_split_free(prc_context *ctx, double *positions, uint32_t *tri_indices)
{
    if (positions != NULL) prc_free(ctx, positions);
    if (tri_indices != NULL) prc_free(ctx, tri_indices);
}

int
prc_write_compress_tess_entry(prc_context *ctx, prc_bit_write_state *s,
    const double *positions, uint32_t num_positions,
    const double *normals, uint32_t num_normals,
    const uint32_t *tri_indices, const uint32_t *norm_indices, uint32_t num_triangles,
    const uint32_t *face_tri_counts, uint32_t num_faces,
    prc_write_tolerance tolerance, double crease_angle_degrees)
{
    prc_encode_mesh mesh;
    prc_encode_traversal_result trav;
    uint32_t *orig_face_id = NULL;      /* num_triangles entries, ORIGINAL (pre-preprocess) order */
    uint32_t *face_indices_post = NULL; /* mesh.num_triangles entries, POST-preprocess order */
    double *corner_normals = NULL;      /* mesh.num_triangles * 9, POST-preprocess order; also fed
                                            into prc_encode_traversal as real_normals so it can decide
                                            each triangle's normal_was_reversed bit inline */
    uint8_t *rev = NULL;
    int32_t *angles = NULL;
    uint8_t *bin = NULL;
    uint32_t acount = 0, bsize = 0;
    uint8_t must_recalculate_normals;
    int mesh_ready = 0, trav_ready = 0;
    int ret = PRC_ERROR_INTERNAL;
    int code;
    uint32_t f, t, k;

    (void)num_normals;

    if (ctx == NULL || s == NULL || num_triangles == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_entry: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    code = prc_encode_preprocess(ctx, positions, num_positions, tri_indices, num_triangles, tolerance, &mesh);
    if (code != 0) return code;
    mesh_ready = 1;

    if (mesh.num_triangles == 0)
    {
        /* Every input triangle was degenerate after welding; nothing to
           encode. Treated as a caller error (same as num_triangles == 0
           above) rather than silently emitting an empty compressed
           record, which this pipeline has never been exercised against. */
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_compress_tess_entry: no surviving triangles after weld\n");
        goto cleanup;
    }

    if (face_tri_counts != NULL && num_faces > 0)
    {
        orig_face_id = (uint32_t *)prc_malloc(ctx, (size_t)num_triangles * sizeof(uint32_t));
        face_indices_post = (uint32_t *)prc_malloc(ctx, (size_t)mesh.num_triangles * sizeof(uint32_t));
        if (orig_face_id == NULL || face_indices_post == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_compress_tess_entry\n");
            ret = PRC_ERROR_MEMORY;
            goto cleanup;
        }
        t = 0;
        for (f = 0; f < num_faces; f++)
        {
            uint32_t n = face_tri_counts[f];
            for (k = 0; k < n; k++)
                orig_face_id[t++] = f;
        }
        for (k = 0; k < mesh.num_triangles; k++)
            face_indices_post[k] = orig_face_id[mesh.tri_orig_index[k]];
    }

    must_recalculate_normals = (normals == NULL) ? 1u : 0u;
    if (!must_recalculate_normals)
    {
        corner_normals = (double *)prc_malloc(ctx, (size_t)mesh.num_triangles * 9 * sizeof(double));
        if (corner_normals == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_compress_tess_entry\n");
            ret = PRC_ERROR_MEMORY;
            goto cleanup;
        }
        for (k = 0; k < mesh.num_triangles; k++)
        {
            uint32_t orig_tri = mesh.tri_orig_index[k];
            uint32_t c;
            for (c = 0; c < 3; c++)
            {
                uint32_t nidx = norm_indices[(size_t)orig_tri * 3 + c];
                memcpy(&corner_normals[((size_t)k * 3 + c) * 3], &normals[(size_t)nidx * 3], 3 * sizeof(double));
            }
        }

    }
    /* When normals must be recalculated (no per-vertex normals supplied),
       compute a smooth per-mesh-vertex proxy normal (equal-weighted average
       of adjacent MESH-order face normals) and feed it into
       prc_encode_traversal's real_normals parameter so
       prc_encode_decide_reversed makes a real, traversal-consistent
       reversed-bit decision for EVERY triangle (including growing ones),
       instead of leaving trav.triangle_reversed all-zero (the previous
       behavior, via prc_encode_normals_c1's must-recalculate path, which
       unconditionally forced growing triangles' reversed bit to 0 even when
       geometrically wrong -- see prc_encode_normals_c1's own long comment
       on that gap). This was originally an opt-in diagnostic
       (PRC_DIAG_USE_PROXY_NORMALS) added to test whether matching a real
       encoder's non-zero reversed-bit pattern (see the RG cross-check in
       the mixed_chains investigation writeup) fixes Acrobat rendering --
       promoted to permanent default behavior after verifying it's lossless
       (nano_prc_stl_import --verify passes on UK_original.stl/
       beetle_1000000.stl) and regression-free (full ctest suite), and
       matches an independently-produced encoder's reversed-bit/
       edge_status_array output exactly on the fan8 synthetic case. It did
       NOT resolve the specific Acrobat blank-tree bug those two real files
       exhibit (still open), but is a genuine, standalone correctness fix
       (previously 100% of growing triangles got the wrong reversed-normal
       bit whenever normals had to be recalculated) worth keeping on its
       own merits -- also produces smaller output (more internally
       consistent data compresses better). */
    if (must_recalculate_normals)
    {
        double *vertex_normal = (double *)prc_calloc(ctx, (size_t)mesh.num_positions * 3, sizeof(double));

        corner_normals = (double *)prc_malloc(ctx, (size_t)mesh.num_triangles * 9 * sizeof(double));
        if (vertex_normal == NULL || corner_normals == NULL)
        {
            if (vertex_normal != NULL) prc_free(ctx, vertex_normal);
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_compress_tess_entry (proxy normals)\n");
            ret = PRC_ERROR_MEMORY;
            goto cleanup;
        }
        for (k = 0; k < mesh.num_triangles; k++)
        {
            uint32_t i0 = mesh.tri_indices[(size_t)k * 3 + 0];
            uint32_t i1 = mesh.tri_indices[(size_t)k * 3 + 1];
            uint32_t i2 = mesh.tri_indices[(size_t)k * 3 + 2];
            double *pa = &mesh.positions[(size_t)i0 * 3];
            double *pb = &mesh.positions[(size_t)i1 * 3];
            double *pc = &mesh.positions[(size_t)i2 * 3];
            double e1[3], e2[3], fn[3];
            uint32_t d;
            for (d = 0; d < 3; d++) { e1[d] = pb[d] - pa[d]; e2[d] = pc[d] - pa[d]; }
            fn[0] = e1[1]*e2[2] - e1[2]*e2[1];
            fn[1] = e1[2]*e2[0] - e1[0]*e2[2];
            fn[2] = e1[0]*e2[1] - e1[1]*e2[0];
            for (d = 0; d < 3; d++)
            {
                vertex_normal[(size_t)i0 * 3 + d] += fn[d];
                vertex_normal[(size_t)i1 * 3 + d] += fn[d];
                vertex_normal[(size_t)i2 * 3 + d] += fn[d];
            }
        }
        for (k = 0; k < mesh.num_triangles; k++)
        {
            uint32_t c;
            for (c = 0; c < 3; c++)
            {
                uint32_t mv = mesh.tri_indices[(size_t)k * 3 + c];
                memcpy(&corner_normals[((size_t)k * 3 + c) * 3], &vertex_normal[(size_t)mv * 3], 3 * sizeof(double));
            }
        }
        prc_free(ctx, vertex_normal);
    }

    /* corner_normals (mesh order, 9 doubles/triangle -- NULL when
       must_recalculate_normals) is passed straight through as
       prc_encode_traversal's real_normals: the inline reversed-bit decision
       needs TRAVERSAL corner 0's specific real normal, not a blurred
       per-position average, to agree with prc_encode_normals_c2's own
       corner-0-specific rejection check below (see prc_encode_decide_reversed's
       comment) -- see prc_encode_traversal's header comment for why a
       precomputed baseline (the old two-pass design) can't decide this
       correctly at all: reversing a triangle changes push order for
       everything downstream of it in the traversal, invalidating a
       baseline's assumptions for those triangles. Measured on turbine tess
       902: 49% of triangles diverged between a baseline-computed value and
       reality, confirming this isn't a rare edge case. */
    code = prc_encode_traversal(ctx, &mesh, face_indices_post, mesh.tolerance_mm, &trav, NULL, NULL, corner_normals);
    if (code != 0) goto cleanup;
    trav_ready = 1;

    if (!must_recalculate_normals)
    {
        code = prc_encode_normals_c2(ctx, &mesh, &trav, corner_normals, &angles, &acount, &bin, &bsize);
        if (prc_diag_getenv("PRC_DIAG_C2_FALLBACK") != NULL)
            printf("PRC_DIAG_C2_FALLBACK: prc_encode_normals_c2 code=%d (0=succeeded, nonzero=fell back to C1)\n", code);
        if (code != 0)
        {
            /* Supplied per-corner normals can legitimately conflict with the
               decoder's canonical traversal winding for a "growing" (non-
               leaf) triangle -- a real geometric constraint of the C2 path,
               not a bug (see prc_encode_normals_c2's own error message).
               Rather than fail the whole entry over it, fall back to C1
               (decoder-reconstructed normals from geometry): every real
               mesh has SOME valid encoding, and reconstructed-but-rendered
               beats exact-but-rejected.

               The traversal above already decided each triangle's
               normal_was_reversed bit consistently (using corner_normals, in
               the same single pass that built the topology), so recovering
               rev[] is just reading it back -- no second traversal needed. */
            prc_free(ctx, corner_normals);
            corner_normals = NULL;
            must_recalculate_normals = 1u;

            rev = (uint8_t *)prc_malloc(ctx, (size_t)trav.edge_status_array_size * sizeof(uint8_t));
            if (rev == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_compress_tess_entry\n");
                ret = PRC_ERROR_MEMORY;
                goto cleanup;
            }
            memcpy(rev, trav.triangle_reversed, (size_t)trav.edge_status_array_size * sizeof(uint8_t));
            code = 0;
        }
    }
    else
    {
        /* Proxy normals were fed to prc_encode_traversal above, so
           trav.triangle_reversed already holds a real, traversal-consistent
           decision for every triangle (see prc_encode_decide_reversed) --
           just copy it, same as the C2-fallback path above does. */
        rev = (uint8_t *)prc_malloc(ctx, (size_t)trav.edge_status_array_size * sizeof(uint8_t));
        if (rev == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_compress_tess_entry\n");
            ret = PRC_ERROR_MEMORY;
            goto cleanup;
        }
        memcpy(rev, trav.triangle_reversed, (size_t)trav.edge_status_array_size * sizeof(uint8_t));
        code = 0;
    }
    if (code != 0) goto cleanup;

    code = prc_write_compress_tess_to_stream(ctx, s, &trav, mesh.tolerance_mm,
        rev, crease_angle_degrees, angles, acount, bin, bsize, must_recalculate_normals, NULL);
    ret = code;

cleanup:
    if (orig_face_id != NULL) prc_free(ctx, orig_face_id);
    if (face_indices_post != NULL) prc_free(ctx, face_indices_post);
    if (corner_normals != NULL) prc_free(ctx, corner_normals);
    if (rev != NULL) prc_free(ctx, rev);
    if (angles != NULL) prc_free(ctx, angles);
    if (bin != NULL) prc_free(ctx, bin);
    if (trav_ready) prc_encode_traversal_free(ctx, &trav);
    if (mesh_ready) prc_encode_preprocess_free(ctx, &mesh);
    return ret;
}
