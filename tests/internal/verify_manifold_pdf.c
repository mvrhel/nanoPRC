/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Same closed-2-manifold check as verify_manifold_real.c, but driven
   through the PUBLIC API (prc_api_open_contents / model tree / per-
   tessellation graphic primitives, the same call sequence as
   demos/quick_start) instead of the low-level single-file-structure raw
   .prc parser. verify_manifold_real.c bails on any file with more than one
   PRC file structure ("only single-file-structure inputs supported"); real
   multi-part assemblies (a turbine, a furnished room, anything with more
   than one body) routinely have dozens of file structures and hundreds of
   tessellations, each its own separate closed shell. This tool walks all
   of them via the same public API entry points a real consumer (a
   renderer, an exporter) would use, so it works on arbitrarily complex
   real-world files without reimplementing nanoPRC's own multi-file-
   structure/multi-tessellation container-walking logic.

   HOW: usage: verify_manifold_pdf.exe <input.pdf>. For each
   PRC_API_TESS_3D_Compressed tessellation, expands every face's graphic
   primitives (TRIANGLES/FAN/STRIP) into individual triangles referencing
   that tessellation's own vertex buffer, then runs the same edge-adjacency
   manifold check as verify_manifold_real.c, per tessellation (bodies are
   not topologically connected to each other, so checking the whole file as
   one mesh would be meaningless). Aggregates pass/fail counts across all
   tessellations, plus the same decoder debug-hook trigger counters
   (prc_debug_orient_flip_*, prc_debug_index_swap_* -- see
   prc_decode_compressed_tess.c) so the same PRC_DEBUG_DISABLE_* env-var
   ablation used on the four oracle-document test meshes can be applied to
   arbitrarily large real-world files too.

   LIMITATIONS: No reencoding/round-trip check -- reencode_real_tess.c's
   splice mechanics assume single-file-structure input and have not been
   generalized to multi-file-structure containers; round-tripping a file
   like this would need that generalization first. Read-only diagnostic. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"

extern long prc_debug_orient_flip_triggered_count;
extern long prc_debug_orient_flip_total_count;
extern long prc_debug_index_swap_triggered_count;
extern long prc_debug_index_swap_total_count;

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
    uint32_t checked = 0, clean = 0;
    uint64_t total_boundary = 0, total_nonmanifold = 0, total_tris = 0, total_verts = 0;

    if (argc < 2)
    {
        printf("usage: %s <input.pdf>\n", argv[0]);
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL) { printf("prc_api_open_contents failed\n"); prc_api_print_error_stack(ctx); return 1; }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0) { printf("prep_model_tree failed\n"); return 1; }
    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0) { printf("create_model_tree failed\n"); return 1; }

    printf("parts=%u products=%u markups=%u\n", num_parts, num_products, num_markups);

    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations, &totalLineTesselations);
    if (code < 0) { printf("get_number_tessellations failed\n"); return 1; }
    printf("total tessellations=%u (line=%u)\n", totalTesselations, totalLineTesselations);

    tesses = (prc_api_tess *)calloc(totalTesselations, sizeof(prc_api_tess));

    for (k = 0; k < totalTesselations; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t nFaces = prc_api_get_number_faces(ctx, data, k);
        tess->num_faces = nFaces;
        tess->tess_faces = (prc_api_face *)calloc(nFaces, sizeof(prc_api_face));

        code = prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL, &has_lines);
        if (code < 0) { printf("  tess %u: initialize failed, skipping\n", k); continue; }

        if (tess->type != PRC_API_TESS_3D_Compressed && tess->type != PRC_API_TESS_3D)
            continue;

        for (j = 0; j < tess->num_faces; j++)
        {
            code = prc_api_get_tessellation_vertices(ctx, data, model_tree, k, j, tess->tess_faces + j, tess);
            if (code < 0) { printf("  tess %u face %u: get_vertices failed\n", k, j); continue; }
        }

        /* Build this tessellation's own triangle list from every face's graphic primitives. */
        {
            uint32_t *tri = NULL;
            uint32_t tri_count = 0, tri_cap = 0;
            prc_api_tess_vertex_buffer *vb = (tess->type == PRC_API_TESS_3D_Compressed)
                ? &tess->tess_vertices : NULL;
            uint32_t f;

            /* For PRC_API_TESS_3D_Compressed, all triangles live in face_index
               0 regardless of num_faces (see demos/quick_start's comment: "we
               keep this as one face") -- looping every face here would re-read
               and duplicate the same shared primitive set num_faces times. */
            uint32_t face_limit = vb ? 1 : tess->num_faces;
            for (f = 0; f < face_limit; f++)
            {
                prc_api_face *face = &tess->tess_faces[f];
                prc_api_tess_vertex_buffer *face_vb = vb ? vb : &face->face_vertices;
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
                    (void)face_vb;
                }
            }

            if (tri_count >= 9)
            {
                uint32_t ntri = tri_count / 3;
                edge_key *edges = (edge_key *)malloc(sizeof(edge_key) * tri_count);
                uint32_t e = 0, t;
                uint32_t boundary = 0, bad = 0;

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
                        e++;
                    }
                }
                qsort(edges, e, sizeof(edge_key), edge_key_cmp);
                {
                    uint32_t run_start = 0, i2;
                    for (i2 = 1; i2 <= e; i2++)
                    {
                        if (i2 == e || edge_key_cmp(&edges[i2], &edges[run_start]) != 0)
                        {
                            uint32_t run_len = i2 - run_start;
                            if (run_len == 1) boundary++;
                            else if (run_len != 2) bad++;
                            run_start = i2;
                        }
                    }
                }
                checked++;
                total_tris += ntri;
                total_boundary += boundary;
                total_nonmanifold += bad;
                if (boundary == 0 && bad == 0)
                    clean++;
                else
                    printf("  tess %u: NOT CLEAN -- %u tris, boundary=%u non-manifold=%u\n",
                        k, ntri, boundary, bad);
                free(edges);
            }
            free(tri);
        }
        total_verts += (tess->type == PRC_API_TESS_3D_Compressed) ? tess->tess_vertices.num_vertices : 0;
    }

    printf("\n=== Summary ===\n");
    printf("Compressed/3D tessellations with >=3 triangles checked: %u\n", checked);
    printf("Clean closed-2-manifolds: %u / %u\n", clean, checked);
    printf("Total triangles checked: %llu   total vertices (compressed tess only): %llu\n",
        (unsigned long long)total_tris, (unsigned long long)total_verts);
    printf("Total boundary edges: %llu   total non-manifold edges: %llu\n",
        (unsigned long long)total_boundary, (unsigned long long)total_nonmanifold);
    printf("orientation-flip triggered: %ld / %ld (%.1f%%)\n",
        prc_debug_orient_flip_triggered_count, prc_debug_orient_flip_total_count,
        prc_debug_orient_flip_total_count ? 100.0 * (double)prc_debug_orient_flip_triggered_count / (double)prc_debug_orient_flip_total_count : 0.0);
    printf("index-swap triggered: %ld / %ld (%.1f%%)\n",
        prc_debug_index_swap_triggered_count, prc_debug_index_swap_total_count,
        prc_debug_index_swap_total_count ? 100.0 * (double)prc_debug_index_swap_triggered_count / (double)prc_debug_index_swap_total_count : 0.0);

    prc_api_release_data(ctx, data, tesses, totalTesselations, NULL, 0, model_tree);
    prc_api_release_context(ctx);
    return (total_boundary > 0 || total_nonmanifold > 0) ? 3 : 0;
}
