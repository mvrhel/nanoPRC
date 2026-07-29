/* Ad-hoc diagnostic: reads hand-ic3d-extracted.prc's geometry via the
   PUBLIC read API (same pattern as demos/stl_export), then re-writes it
   via the PUBLIC write API using OUR OWN tree/schema/model construction
   (matching demos/stl_import's 3-level shape) instead of keeping
   Interchange3D's own tree/schema/model. Isolates whether "our tree/
   schema/model" or "vertex/triangle ordering" is the real variable,
   since reencode_real_tess.c's control test confounded both. */
#include <prc_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: %s <input.prc> <output.prc> [trivial]\n", argv[0]);
        return 1;
    }

    /* TEST MODE: skip reading real geometry entirely, use a hardcoded
       trivial single-triangle mesh instead -- exercises the EXACT SAME
       tree/schema-building code below with data known to be safe, so its
       output's schema/tree bytes can be diffed against the real-geometry
       (failing) run to see if tree/schema generation is itself influenced
       by tessellation content. */
    if (argc >= 4 && strcmp(argv[3], "trivial") == 0)
    {
        prc_context *ctx = prc_api_new_context(NULL);
        double welded[9] = {0,0,0, 1,0,0, 0,1,0};
        uint32_t out_tri_indices[3] = {0,1,2};
        uint32_t num_welded = 3, num_tris = 1;

        prc_api_write_tessellation wtess;
        memset(&wtess, 0, sizeof(wtess));
        wtess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
        wtess.positions = welded;
        wtess.num_positions = num_welded;
        wtess.tri_indices = out_tri_indices;
        wtess.num_triangles = num_tris;
        wtess.crease_angle_degrees = 30.0;

        prc_api_write_rep_item rep_item;
        memset(&rep_item, 0, sizeof(rep_item));
        rep_item.kind = PRC_API_WRITE_RI_SURFACE;
        rep_item.biased_tessellation_index = 1;

        prc_api_write_node leaf;
        memset(&leaf, 0, sizeof(leaf));
        leaf.rep_items = &rep_item;
        leaf.num_rep_items = 1;
        leaf.name = "part_1";
        leaf.part_name = "part_1";
        leaf.bbox_min[0]=0; leaf.bbox_min[1]=0; leaf.bbox_min[2]=0;
        leaf.bbox_max[0]=1; leaf.bbox_max[1]=1; leaf.bbox_max[2]=0;

        prc_api_write_node *leaf_ptr = &leaf;
        prc_api_write_node intermediate;
        memset(&intermediate, 0, sizeof(intermediate));
        intermediate.has_empty_part = 1;
        intermediate.children = &leaf_ptr;
        intermediate.num_children = 1;

        prc_api_write_node *inter_ptr = &intermediate;
        prc_api_write_node root;
        memset(&root, 0, sizeof(root));
        root.name = "hand";
        root.children = &inter_ptr;
        root.num_children = 1;

        int code = prc_api_write_prc_file(ctx, argv[2], "nanoPRC", &root, &wtess, 1);
        printf("write code=%d\n", code);
        if (code != 0) prc_api_print_error_stack(ctx);
        return code;
    }

    prc_context *ctx = prc_api_new_context(NULL);
    prc_api_data data = prc_api_open_contents(ctx, argv[1]);
    if (!data) { fprintf(stderr, "open failed\n"); return 1; }

    uint32_t num_parts, num_products, num_markups;
    prc_api_product *model_tree = NULL;
    if (prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups) != 0) { fprintf(stderr, "prep failed\n"); return 1; }
    if (prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups) != 0) { fprintf(stderr, "create failed\n"); return 1; }

    uint32_t total_tess = 0, total_line_tess = 0;
    if (prc_api_get_number_tessellations(ctx, data, model_tree, &total_tess, &total_line_tess) != 0) { fprintf(stderr, "count failed\n"); return 1; }
    printf("total_tess=%u\n", total_tess);

    /* Collect all triangles across all faces of tessellation 0 into flat
       position/tri_indices arrays, deduplicating positions by exact value
       match via a simple linear-probe hash (mirrors the scale of demos/
       stl_import's own weld, just exact instead of tolerance-based since
       these are already-decoded, presumably-clean values). */
    prc_api_tess tess;
    memset(&tess, 0, sizeof(tess));
    uint32_t num_faces = prc_api_get_number_faces(ctx, data, 0);
    tess.num_faces = num_faces;
    tess.tess_faces = (prc_api_face *)calloc(num_faces, sizeof(prc_api_face));
    uint8_t has_lines = 0;
    if (prc_api_initialize_tessellation(ctx, data, model_tree, 0, &tess, NULL, &has_lines) != 0) { fprintf(stderr, "init tess failed\n"); return 1; }
    for (uint32_t j = 0; j < tess.num_faces; j++)
        if (prc_api_get_tessellation_vertices(ctx, data, model_tree, 0, j, tess.tess_faces + j, &tess) != 0) { fprintf(stderr, "get verts failed\n"); return 1; }

    /* Count triangles across all graphic primitives (TRIANGLES/STRIP/FAN),
       same pattern as demos/stl_export.c. */
    size_t max_tris = 0;
    for (size_t f = 0; f < tess.num_faces; f++)
    {
        prc_api_face *face = &tess.tess_faces[f];
        for (size_t p = 0; p < face->num_graphic_primitives; p++)
        {
            prc_api_graphic_primitive prim;
            if (prc_api_get_graphics_primitive(ctx, data, &tess, (uint32_t)f, p, &prim) < 0) continue;
            if (prim.type == PRC_API_TRIANGLES) max_tris += prim.num_indices / 3;
            else if (prim.num_indices >= 2) max_tris += prim.num_indices - 2;
        }
    }
    printf("max_tris estimate=%zu\n", max_tris);

    double *raw_positions = (double *)malloc(sizeof(double) * max_tris * 9);
    uint32_t *tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * max_tris * 3);
    uint32_t num_tris = 0;

    for (size_t f = 0; f < tess.num_faces; f++)
    {
        prc_api_face *face = &tess.tess_faces[f];
        prc_api_tess_vertex_buffer *vbuf = (tess.type == PRC_API_TESS_3D_Compressed) ? &tess.tess_vertices : &face->face_vertices;
        if (!vbuf || !vbuf->vertices || vbuf->num_vertices == 0) continue;
        for (size_t p = 0; p < face->num_graphic_primitives; p++)
        {
            prc_api_graphic_primitive prim;
            if (prc_api_get_graphics_primitive(ctx, data, &tess, (uint32_t)f, p, &prim) < 0) continue;
            if (prim.type == PRC_API_TRIANGLES)
            {
                for (size_t idx = 0; idx + 2 < prim.num_indices; idx += 3)
                {
                    uint32_t i0 = prim.indices[idx], i1 = prim.indices[idx+1], i2 = prim.indices[idx+2];
                    if (i0 >= vbuf->num_vertices || i1 >= vbuf->num_vertices || i2 >= vbuf->num_vertices) continue;
                    memcpy(&raw_positions[(size_t)num_tris*9+0], vbuf->vertices[i0].position, sizeof(float)*3);
                    memcpy(&raw_positions[(size_t)num_tris*9+3], vbuf->vertices[i1].position, sizeof(float)*3);
                    memcpy(&raw_positions[(size_t)num_tris*9+6], vbuf->vertices[i2].position, sizeof(float)*3);
                    /* widen float->double manually since memcpy above copied floats into a double array -- fix below */
                    num_tris++;
                }
            }
        }
    }
    /* Redo properly: the memcpy above is wrong (float vs double size) --
       do an explicit per-component widen instead. */
    num_tris = 0;
    for (size_t f = 0; f < tess.num_faces; f++)
    {
        prc_api_face *face = &tess.tess_faces[f];
        prc_api_tess_vertex_buffer *vbuf = (tess.type == PRC_API_TESS_3D_Compressed) ? &tess.tess_vertices : &face->face_vertices;
        if (!vbuf || !vbuf->vertices || vbuf->num_vertices == 0) continue;
        for (size_t p = 0; p < face->num_graphic_primitives; p++)
        {
            prc_api_graphic_primitive prim;
            if (prc_api_get_graphics_primitive(ctx, data, &tess, (uint32_t)f, p, &prim) < 0) continue;
            if (prim.type != PRC_API_TRIANGLES) continue;
            for (size_t idx = 0; idx + 2 < prim.num_indices; idx += 3)
            {
                uint32_t i0 = prim.indices[idx], i1 = prim.indices[idx+1], i2 = prim.indices[idx+2];
                if (i0 >= vbuf->num_vertices || i1 >= vbuf->num_vertices || i2 >= vbuf->num_vertices) continue;
                for (int c = 0; c < 3; c++)
                {
                    raw_positions[(size_t)num_tris*9+0+c] = (double)vbuf->vertices[i0].position[c];
                    raw_positions[(size_t)num_tris*9+3+c] = (double)vbuf->vertices[i1].position[c];
                    raw_positions[(size_t)num_tris*9+6+c] = (double)vbuf->vertices[i2].position[c];
                }
                num_tris++;
            }
        }
    }
    printf("collected num_tris=%u\n", num_tris);

    if (getenv("PRC_DIAG_DUMP_TRIS") != NULL)
    {
        uint32_t dump_idx[] = {83816, 83817, 83833, 83837, 83836, 84300, 84301, 84302};
        for (uint32_t d = 0; d < sizeof(dump_idx)/sizeof(dump_idx[0]); d++)
        {
            uint32_t t = dump_idx[d];
            if (t >= num_tris) continue;
            double *p = &raw_positions[(size_t)t*9];
            printf("TRI %u: (%.9f,%.9f,%.9f) (%.9f,%.9f,%.9f) (%.9f,%.9f,%.9f)\n",
                t, p[0],p[1],p[2], p[3],p[4],p[5], p[6],p[7],p[8]);
        }
    }

    /* TEST: PRC_DIAG_EXCLUDE_TRIS="idx1,idx2,..." -- drop specific
       (post-collection, pre-weld) triangle indices before proceeding, to
       test whether specific non-manifold-edge-contributing triangles are
       the root cause of the Acrobat blank-tree bug (see 2026-07-22
       investigation writeup). Indices refer to this collection loop's
       num_tris counter, which matches prc_encode_preprocess's clean_tris
       indexing exactly for this file (zero exact-duplicate-vertex
       triangles get filtered by that stage, confirmed separately). */
    if (getenv("PRC_DIAG_EXCLUDE_TRIS") != NULL)
    {
        uint32_t exclude[64]; uint32_t nexclude = 0;
        const char *env_val = getenv("PRC_DIAG_EXCLUDE_TRIS");
        char spec[1024];
        strncpy(spec, env_val, sizeof(spec) - 1);
        spec[sizeof(spec) - 1] = '\0';
        char *tok = strtok(spec, ",");
        while (tok != NULL && nexclude < 64)
        {
            exclude[nexclude++] = (uint32_t)atoi(tok);
            tok = strtok(NULL, ",");
        }
        uint32_t new_num_tris = 0;
        for (uint32_t t = 0; t < num_tris; t++)
        {
            uint8_t skip = 0;
            for (uint32_t e = 0; e < nexclude; e++)
                if (exclude[e] == t) { skip = 1; break; }
            if (skip) continue;
            if (new_num_tris != t)
                memcpy(&raw_positions[(size_t)new_num_tris*9], &raw_positions[(size_t)t*9], 9*sizeof(double));
            new_num_tris++;
        }
        printf("PRC_DIAG_EXCLUDE_TRIS: excluded %u of %u requested, num_tris %u -> %u\n",
            nexclude, nexclude, num_tris, new_num_tris);
        num_tris = new_num_tris;
    }

    /* Simple exact-match weld via linear-probe hash on quantized coords. */
    uint32_t num_buckets = 1; while (num_buckets < num_tris * 3 * 2) num_buckets *= 2;
    int32_t *bucket_vertex = (int32_t *)malloc(sizeof(int32_t) * num_buckets);
    for (uint32_t i = 0; i < num_buckets; i++) bucket_vertex[i] = -1;
    double *welded = (double *)malloc(sizeof(double) * num_tris * 3 * 3);
    uint32_t num_welded = 0;
    for (uint32_t t = 0; t < num_tris; t++)
        tri_indices[t] = 0; /* placeholder, filled below per-corner */
    uint32_t *out_tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * num_tris * 3);
    /* TEST: skip welding entirely if PRC_TEST_NO_WELD is set -- pure
       triangle soup, every corner gets its own unique position index, no
       vertex sharing at all. Tests whether welding degree/vertex-sharing
       pattern (not raw triangle count) is the real variable. */
    if (getenv("PRC_TEST_NO_WELD") != NULL)
    {
        for (uint32_t t = 0; t < num_tris; t++)
        {
            for (int c = 0; c < 3; c++)
            {
                uint32_t idx = t * 3 + (uint32_t)c;
                welded[(size_t)idx*3+0] = raw_positions[(size_t)t*9 + c*3 + 0];
                welded[(size_t)idx*3+1] = raw_positions[(size_t)t*9 + c*3 + 1];
                welded[(size_t)idx*3+2] = raw_positions[(size_t)t*9 + c*3 + 2];
                out_tri_indices[idx] = idx;
            }
        }
        num_welded = num_tris * 3;
        goto skip_weld;
    }
    for (uint32_t t = 0; t < num_tris; t++)
    {
        for (int c = 0; c < 3; c++)
        {
            double *p = &raw_positions[(size_t)t*9 + c*3];
            uint64_t h = 1469598103934665603ULL;
            for (int b = 0; b < 3; b++)
            {
                int64_t q = (int64_t)(p[b] * 1e6);
                h ^= (uint64_t)q; h *= 1099511628211ULL;
            }
            uint32_t bi = (uint32_t)(h ^ (h>>32)) & (num_buckets - 1);
            int32_t found = -1;
            for (uint32_t probe = 0; probe < num_buckets; probe++)
            {
                uint32_t slot = (bi + probe) & (num_buckets - 1);
                if (bucket_vertex[slot] == -1) { bucket_vertex[slot] = -2; /*mark reserve*/
                    welded[(size_t)num_welded*3+0]=p[0]; welded[(size_t)num_welded*3+1]=p[1]; welded[(size_t)num_welded*3+2]=p[2];
                    bucket_vertex[slot] = (int32_t)num_welded;
                    found = (int32_t)num_welded;
                    num_welded++;
                    break;
                }
                else
                {
                    double *wp = &welded[(size_t)bucket_vertex[slot]*3];
                    if (wp[0]==p[0] && wp[1]==p[1] && wp[2]==p[2]) { found = bucket_vertex[slot]; break; }
                }
            }
            out_tri_indices[(size_t)t*3+c] = (uint32_t)found;
        }
    }
skip_weld:
    printf("num_welded=%u\n", num_welded);

    /* Now write using OUR OWN 3-level tree/schema/model (matching demos/
       stl_import's shape), COMPRESSED kind, must_calculate_normals-style
       (normals=NULL) to match the default STL-import path. */
    prc_api_write_tessellation wtess;
    memset(&wtess, 0, sizeof(wtess));
    wtess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    wtess.positions = welded;
    wtess.num_positions = num_welded;
    wtess.tri_indices = out_tri_indices;
    wtess.num_triangles = num_tris;
    wtess.crease_angle_degrees = 30.0;
    if (getenv("PRC_DIAG_TOLERANCE_ABS") != NULL)
        wtess.tolerance = prc_write_tol_absolute(atof(getenv("PRC_DIAG_TOLERANCE_ABS")));
    /* TEST: standalone_grid_triangles.c explicitly sets num_faces=1 with a
       real face_tri_counts array covering all triangles, even for
       COMPRESSED; we'd been leaving this NULL/0 (legal per docs). */
    uint32_t *face_tri_counts = (uint32_t *)malloc(sizeof(uint32_t));
    face_tri_counts[0] = num_tris;
    wtess.face_tri_counts = face_tri_counts;
    wtess.num_faces = 1;

    prc_api_write_rep_item rep_item;
    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;

    double bbox_min[3] = {1e300,1e300,1e300}, bbox_max[3] = {-1e300,-1e300,-1e300};
    for (uint32_t i = 0; i < num_welded; i++)
        for (int c = 0; c < 3; c++)
        {
            if (welded[(size_t)i*3+c] < bbox_min[c]) bbox_min[c] = welded[(size_t)i*3+c];
            if (welded[(size_t)i*3+c] > bbox_max[c]) bbox_max[c] = welded[(size_t)i*3+c];
        }

    prc_api_write_node leaf;
    memset(&leaf, 0, sizeof(leaf));
    leaf.rep_items = &rep_item;
    leaf.num_rep_items = 1;
    leaf.name = "part_1";
    leaf.part_name = "part_1";
    /* TEST: trivial round-number bbox instead of hand.stl's real computed
       values, to test a value-coincidence bug in bbox double encoding. */
    for (int c = 0; c < 3; c++) { leaf.bbox_min[c] = -1000.0; leaf.bbox_max[c] = 1000.0; }

    prc_api_write_node *leaf_ptr = &leaf;
    prc_api_write_node intermediate;
    memset(&intermediate, 0, sizeof(intermediate));
    intermediate.has_empty_part = 1;
    intermediate.children = &leaf_ptr;
    intermediate.num_children = 1;
    intermediate.name = "hand_scene"; /* TEST: standalone_grid_triangles.c names its intermediate node; ours left it NULL */

    prc_api_write_node *inter_ptr = &intermediate;
    prc_api_write_node root;
    memset(&root, 0, sizeof(root));
    root.name = "hand";
    root.children = &inter_ptr;
    root.num_children = 1;

    int code = prc_api_write_prc_file(ctx, argv[2], "nanoPRC", &root, &wtess, 1);
    printf("write code=%d\n", code);
    if (code != 0) prc_api_print_error_stack(ctx);

    return code;
}
