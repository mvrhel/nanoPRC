/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Decodes a real, independently-produced raw .prc file's compressed
   tessellation (tess_count==1, single file structure -- same restriction as
   reencode_real_tess.c, whose section-locating prefix this duplicates) and
   checks the decoded triangle mesh for closed-2-manifold validity: every
   edge must be shared by exactly two triangles. Also reports Euler
   characteristic, flags edge-length statistical outliers (a proxy for the
   "corner spike" defect described in the oracle document), reproduces that
   document's edge_status_array bit-sum table (T, E=3T/2, sum of set bits,
   sum-(E-1)) for direct cross-checking, reports the point_reference_array
   sequentiality fingerprint, and prints the trigger counts of the decoder's
   two debug-hooked mechanisms (prc_debug_orient_flip_*,
   prc_debug_index_swap_* -- internal globals in
   prc_decode_compressed_tess.c, not part of the public API, declared
   extern here for diagnostic reach-in like other tools in this directory
   reach into private headers).

   HOW: usage: verify_manifold_real.exe <real_input.prc>. Set
   PRC_DEBUG_DISABLE_ORIENT_FLIP=1 and/or PRC_DEBUG_DISABLE_INDEX_SWAP=1
   in the environment before running to ablate the corresponding decoder
   mechanism and see whether manifold/spike defects appear -- this is the
   empirical test for whether either mechanism is load-bearing on a given
   real file, as opposed to a redundant no-op on files that never exercise
   the code path it guards.

   WHY IT EXISTS: Built to test nanoPRC's compressed-tessellation decoder
   against the four surfaces-of-revolution test meshes (puck, wheel, coin
   cell, washer) attached to the ISO committee's
   "PRC-compressed-tessellation-spec-additions-with-oracles.pdf", which
   proposes a fixed (non-conditional) apex-frame formula and reports a
   residual "corner spike" defect on some meshes attributable to a gate-
   direction/gate-order ambiguity it could not resolve locally. nanoPRC's
   decoder differs from that document's formula by an additional local,
   per-triangle conditional sign flip; this tool measures whether real
   files decode to a defect-free manifold with that flip present, and
   whether removing it (or the separate index-canonicalizing swap in
   prc_set_left_right_edge_indices) introduces exactly the kind of defect
   the document describes.

   LIMITATIONS: Single-file-structure, tess_count==1 inputs only. The
   edge-length outlier heuristic is a coarse proxy for "corner spike" --
   a real spike will produce a grossly wrong vertex position and therefore
   grossly long incident edges, but the threshold is heuristic, not derived
   from the document's own (unspecified) detection method. Assumes a single
   connected 2-manifold per shell; does not separately validate genus. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "zlib.h"

/* Internal debug-hook counters, defined in prc_decode_compressed_tess.c --
   not part of the public API. See that file's prc_debug_hooks_init(). */
extern long prc_debug_orient_flip_triggered_count;
extern long prc_debug_orient_flip_total_count;
extern long prc_debug_index_swap_triggered_count;
extern long prc_debug_index_swap_total_count;

static uint32_t read_le_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

typedef struct { uint32_t a, b; } edge_key;

static int
edge_key_cmp(const void *pa, const void *pb)
{
    const edge_key *a = (const edge_key *)pa;
    const edge_key *b = (const edge_key *)pb;
    if (a->a != b->a) return (a->a < b->a) ? -1 : 1;
    if (a->b != b->b) return (a->b < b->b) ? -1 : 1;
    return 0;
}

static int
double_cmp(const void *pa, const void *pb)
{
    double a = *(const double *)pa;
    double b = *(const double *)pb;
    if (a < b) return -1;
    if (a > b) return 1;
    return 0;
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    FILE *fid;
    long fsize;
    uint8_t *buf;
    size_t read_size;
    const uint8_t *p;
    uint32_t section_count;
    uint32_t *section_offsets;
    uint32_t i;
    uint8_t *tess_inflated = NULL;
    prc_bit_state bit_state;
    uint32_t wrapper_tag, attribute_count, tess_count, entry_tag;
    prc_name wrapper_name;
    prc_tess_3d_compressed *parsed = NULL;
    int code;
    uint32_t T, V, k;
    edge_key *edges;
    uint32_t nedges;
    uint32_t bad_edges = 0, boundary_edges = 0;
    double *lengths;
    long set_bits = 0;

    if (argc < 2)
    {
        printf("usage: %s <real_input.prc>\n", argv[0]);
        return 2;
    }

    fid = fopen(argv[1], "rb");
    if (fid == NULL) { printf("failed to open %s\n", argv[1]); return 1; }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize) { printf("short read\n"); return 1; }
    if (memcmp(buf, "PRC", 3) != 0) { printf("not a raw PRC file\n"); return 1; }

    p = buf + 3 + 4 + 4 + 16 + 16;
    if (read_le_u32(p) != 1) { printf("only single-file-structure inputs supported\n"); return 1; }
    p += 4;
    p += 16; /* fs_uid */
    p += 4;  /* reserved */
    section_count = read_le_u32(p); p += 4;
    section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
    for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }
    if (section_count < 4) { printf("unexpected section_count=%u\n", section_count); return 1; }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uLongf dest_len;
        uint32_t comp_len = section_offsets[4] - section_offsets[3];
        dest_len = comp_len * 40u + 4096u;
        tess_inflated = (uint8_t *)malloc(dest_len);
        for (;;)
        {
            uLongf try_len = dest_len;
            int zret = uncompress(tess_inflated, &try_len, buf + section_offsets[3], comp_len);
            if (zret == Z_OK) { dest_len = try_len; break; }
            if (zret == Z_BUF_ERROR) { dest_len *= 2; free(tess_inflated); tess_inflated = (uint8_t *)malloc(dest_len); continue; }
            printf("zlib uncompress failed: %d\n", zret);
            return 1;
        }
        prc_init_bit_state(ctx, &bit_state, tess_inflated, (size_t)dest_len);
    }

    wrapper_tag = prc_bitread_uint32(ctx, &bit_state);
    if (wrapper_tag != PRC_TYPE_ASM_FileStructureTessellation)
    {
        printf("unexpected wrapper tag %u\n", wrapper_tag);
        return 1;
    }
    {
        prc_content_prc_base wrapper_base;
        memset(&wrapper_base, 0, sizeof(wrapper_base));
        code = prc_parse_content_prc_base(ctx, &bit_state, &wrapper_base);
        if (code < 0) { printf("prc_parse_content_prc_base failed: %d\n", code); return 1; }
        attribute_count = wrapper_base.attribute_data.attribute_count;
        wrapper_name = wrapper_base.name;
    }
    (void)attribute_count; (void)wrapper_name;
    tess_count = prc_bitread_uint32(ctx, &bit_state);
    if (tess_count != 1) { printf("this tool only handles tess_count==1 files (got %u)\n", tess_count); return 1; }

    entry_tag = prc_bitread_uint32(ctx, &bit_state);
    if (entry_tag != PRC_TYPE_TESS_3D_Compressed)
    {
        printf("first tessellation entry is not COMPRESSED (tag=%u)\n", entry_tag);
        return 1;
    }

    code = prc_parse_tess_3d_compressed(ctx, &bit_state, &parsed, 0);
    if (code < 0)
    {
        printf("prc_parse_tess_3d_compressed failed: %d\n", code);
        prc_api_print_error_stack(ctx);
        return 1;
    }

    T = parsed->triangle_face_array_size;
    V = (uint32_t)parsed->num_vertices_prc_compressed_3d;
    printf("=== %s ===\n", argv[1]);
    printf("T (triangles) = %u   V (decoded vertices) = %u   reference_array_size=%u point_reference_array_size=%u\n",
        T, V, parsed->reference_array_size, parsed->point_reference_array_size);

    /* --- Closed-2-manifold check: build undirected edge list, one entry per
       triangle side, canonicalized (min,max), then sort and scan run-lengths. --- */
    edges = (edge_key *)malloc(sizeof(edge_key) * (size_t)T * 3);
    for (k = 0; k < T; k++)
    {
        uint32_t i0 = parsed->triangle_indices_prc_compressed_3d[k*3+0];
        uint32_t i1 = parsed->triangle_indices_prc_compressed_3d[k*3+1];
        uint32_t i2 = parsed->triangle_indices_prc_compressed_3d[k*3+2];
        uint32_t pairs[3][2] = { {i0,i1}, {i1,i2}, {i2,i0} };
        int e;
        for (e = 0; e < 3; e++)
        {
            uint32_t a = pairs[e][0], b = pairs[e][1];
            edge_key ek;
            ek.a = (a < b) ? a : b;
            ek.b = (a < b) ? b : a;
            edges[k*3+e] = ek;
        }
    }
    nedges = T * 3;
    qsort(edges, nedges, sizeof(edge_key), edge_key_cmp);

    lengths = (double *)malloc(sizeof(double) * nedges);
    {
        uint32_t run_start = 0, unique_edges = 0, li = 0;
        for (k = 1; k <= nedges; k++)
        {
            if (k == nedges || edge_key_cmp(&edges[k], &edges[run_start]) != 0)
            {
                uint32_t run_len = k - run_start;
                unique_edges++;
                if (run_len == 1) boundary_edges++;
                else if (run_len != 2) bad_edges++;
                {
                    double dx = parsed->vertices_prc_compressed_3d[edges[run_start].a*3+0] - parsed->vertices_prc_compressed_3d[edges[run_start].b*3+0];
                    double dy = parsed->vertices_prc_compressed_3d[edges[run_start].a*3+1] - parsed->vertices_prc_compressed_3d[edges[run_start].b*3+1];
                    double dz = parsed->vertices_prc_compressed_3d[edges[run_start].a*3+2] - parsed->vertices_prc_compressed_3d[edges[run_start].b*3+2];
                    lengths[li++] = sqrt(dx*dx + dy*dy + dz*dz);
                }
                run_start = k;
            }
        }
        printf("Unique edges E = %u   (3T/2 = %.1f)\n", unique_edges, 1.5 * (double)T);
        printf("Manifold check: boundary edges (shared by 1 tri) = %u, non-manifold edges (shared by !=1,2 tris) = %u\n",
            boundary_edges, bad_edges);
        printf("Euler characteristic V - E + T = %d - %u + %u = %ld\n",
            V, unique_edges, T, (long)V - (long)unique_edges + (long)T);

        /* --- Edge-length outlier ("corner spike") scan --- */
        {
            double *sorted = (double *)malloc(sizeof(double) * li);
            double median, mean = 0.0, sd = 0.0;
            uint32_t n_outliers = 0;
            memcpy(sorted, lengths, sizeof(double) * li);
            qsort(sorted, li, sizeof(double), double_cmp);
            median = sorted[li/2];
            for (k = 0; k < li; k++) mean += lengths[k];
            mean /= (double)li;
            for (k = 0; k < li; k++) sd += (lengths[k]-mean)*(lengths[k]-mean);
            sd = sqrt(sd / (double)li);
            printf("Edge length: median=%.6g mean=%.6g stddev=%.6g\n", median, mean, sd);
            for (k = 0; k < li; k++)
            {
                if (lengths[k] > median * 8.0 && lengths[k] > mean + 6.0 * sd)
                    n_outliers++;
            }
            printf("Edge-length outliers (len > 8x median AND > mean+6sd): %u of %u edges\n", n_outliers, li);
            free(sorted);
        }
    }

    /* --- edge_status_array bit-sum table (oracle doc Section 6 cross-check) --- */
    for (k = 0; k < T; k++)
    {
        uint8_t v = parsed->edge_status_array[k];
        set_bits += (v & 1) + ((v & 2) >> 1);
    }
    printf("edge_status bit-sum: T=%u E=%.0f sum_set_bits=%ld  sum-(E-1)=%ld\n",
        T, 1.5*(double)T, set_bits, set_bits - (long)(1.5*(double)T - 1.0));

    /* --- point_reference_array sequentiality fingerprint --- */
    {
        uint32_t seq_matches = 0;
        uint32_t n = parsed->point_reference_array_size < 20 ? parsed->point_reference_array_size : 20;
        printf("point_reference_array head (%u of %u): [", n, parsed->point_reference_array_size);
        for (k = 0; k < n; k++) printf("%u ", parsed->point_reference_array[k]);
        printf("]\n");
        for (k = 0; k < parsed->point_reference_array_size; k++)
        {
            /* "close to sequential in vertex-creation order" per the oracle doc --
               check each entry is within a small window of its own array index. */
            long diff = (long)parsed->point_reference_array[k] - (long)k;
            if (diff > -4 && diff < 4) seq_matches++;
        }
        printf("point_reference_array entries within +/-4 of own index: %u / %u\n",
            seq_matches, parsed->point_reference_array_size);
    }

    /* --- decoder debug-hook trigger stats --- */
    printf("orientation-flip triggered: %ld / %ld (%.1f%%)\n",
        prc_debug_orient_flip_triggered_count, prc_debug_orient_flip_total_count,
        prc_debug_orient_flip_total_count ? 100.0 * (double)prc_debug_orient_flip_triggered_count / (double)prc_debug_orient_flip_total_count : 0.0);
    printf("index-swap triggered: %ld / %ld (%.1f%%)\n",
        prc_debug_index_swap_triggered_count, prc_debug_index_swap_total_count,
        prc_debug_index_swap_total_count ? 100.0 * (double)prc_debug_index_swap_triggered_count / (double)prc_debug_index_swap_total_count : 0.0);

    free(edges);
    free(lengths);
    free(section_offsets);
    free(tess_inflated);
    free(buf);
    prc_api_release_context(ctx);
    return (bad_edges > 0) ? 3 : 0;
}
