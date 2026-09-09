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

/* Assembly-tree round trip: write a model of a given SHAPE through the
 * public write API, read it back through the public read API, and check the
 * shape survived.
 *
 * WHY THIS EXISTS
 * ---------------
 * The rest of the suite covers the encoder well and the assembly path not at
 * all. test_compress_tess exercises seventeen mesh cases up to a million
 * triangles, but every one of them is a SINGLE mesh; test_file_structure is
 * the only test that reaches the file-structure/model-file writer and it
 * writes one triangle. A grep for prc_api_write_node across tests/ before
 * this file was added returned nothing -- no test built a write-side node
 * tree of any shape.
 *
 * That left the two shapes real files actually take completely untested:
 *
 *   - FLAT AND WIDE: one root, many sibling parts, one tessellation entry
 *     each. This is what a multi-body CAD export or an STL with many
 *     connected components produces, and demos/stl_import has a
 *     hard-coded 100-part threshold above which it abandons this shape and
 *     lumps everything into one entry -- a threshold no test touches on
 *     either side.
 *
 *   - DEEP AND NARROW: a long chain of nested assemblies with the geometry
 *     at the bottom. This is where per-node placement transforms compose,
 *     and where an intermediate node's bounding box matters: a degenerate
 *     box on an empty-part ancestor is already on record as having blanked
 *     an entire model tree in Acrobat while the geometry below it was
 *     intact (see prc_api_write_node's bbox_min comment).
 *
 * WHAT IT CHECKS
 * --------------
 * Per shape: the write succeeds; the file reads back; the product count
 * prc_api_prep_model_tree reports is exactly what was written plus the two
 * root levels, and its part number is at least the real part count (it is a
 * preallocation capacity, not a count -- measured between 1.1x and 2x
 * depending on shape); the tree prc_api_create_model_tree hands back has the
 * expected depth and part-bearing node count; the tessellation count matches
 * the number of geometry-bearing leaves; and the triangles actually decode,
 * summing to the number written.
 *
 * The last point is the one that makes this more than a structural test --
 * counting nodes would pass on a tree whose geometry never survived, so
 * every leaf's triangles are decoded and counted.
 *
 * Encoded size and elapsed time are PRINTED per case, not asserted. They
 * are here to give the numbers a baseline before anyone tunes compression
 * or chases a performance regression; turning either into a threshold
 * requires knowing what the normal spread is, which this does not yet.
 *
 * HOW THE SHAPES ARE BUILT
 * ------------------------
 * One recursive generator over a (branching, depth) pair, so flat and deep
 * are the same code at different parameters rather than two hand-built
 * fixtures that can drift apart: branching=N depth=1 is flat-and-wide,
 * branching=1 depth=D is a chain, and anything else is a bushy tree.
 * Internal nodes carry an empty part and the union of their children's
 * bounding boxes, which is the shape real producers emit.
 *
 * Leaf geometry is a triangle strip, not a triangle soup: adjacent
 * triangles share edges, so the COMPRESSED path's traversal and welding do
 * real work rather than degenerating to isolated triangles. Each leaf is
 * translated along X by its index so leaves stay spatially distinct and a
 * collapsed tree cannot pass by coincidence.
 *
 * LIMITATIONS
 * -----------
 * Node NAMES are written but not asserted on read-back, and per-node
 * transforms are not exercised here -- both deserve their own case and
 * neither is what these shapes were added to cover. Wire tessellations are
 * out of scope (test_wire_tess covers those). The largest case is kept
 * small enough to stay inside a unit test's time budget; the wide end of
 * the part-count range belongs in a benchmark, not here.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "prc_api.h"
#include "prc_context.h"
#include "prc_test.h"

#define TEST_TREE_FILENAME "test_write_tree_tmp.prc"

/* Leaves are spaced further apart than a single strip is long, so their
   bounding boxes cannot overlap and a dropped or duplicated leaf shows up
   as a wrong triangle total rather than passing unnoticed. */
#define LEAF_SPACING 10.0

typedef struct tree_spec_s
{
    const char *label;
    uint32_t branching;      /* children per internal node */
    uint32_t depth;          /* levels of internal nodes; leaves sit at depth+1 */
    uint32_t tris_per_leaf;  /* must be even -- geometry is a quad strip */
    prc_api_write_tess_kind_t kind;
} tree_spec;

/* Everything one generated model owns, freed together by fixture_free. The
   write API takes borrowed pointers throughout, so all of this has to stay
   alive across the prc_api_write_prc_buffer call. */
typedef struct fixture_s
{
    uint32_t num_leaves;
    uint32_t num_nodes;             /* internal nodes + leaves, excluding root */
    uint32_t expected_triangles;

    prc_api_write_node *nodes;      /* [num_nodes], leaves first then internals */
    prc_api_write_node **child_ptrs;/* [num_nodes], the children arrays point in here */
    prc_api_write_node root;

    prc_api_write_rep_item *rep_items;   /* [num_leaves], one per leaf */
    prc_api_write_tessellation *tess;    /* [num_leaves] */
    double **positions;                  /* [num_leaves] */
    uint32_t **tri_indices;              /* [num_leaves] */
    char (*names)[32];                   /* [num_nodes] backing store for node names */

    uint32_t next_node;             /* bump allocator cursors, used during build */
    uint32_t next_leaf;
    uint32_t next_child_slot;
} fixture;

static uint32_t
ipow(uint32_t base, uint32_t exp)
{
    uint32_t r = 1;
    uint32_t i;
    for (i = 0; i < exp; i++)
        r *= base;
    return r;
}

static void
fixture_free(fixture *f)
{
    uint32_t i;

    if (f->positions != NULL)
        for (i = 0; i < f->num_leaves; i++)
            free(f->positions[i]);
    if (f->tri_indices != NULL)
        for (i = 0; i < f->num_leaves; i++)
            free(f->tri_indices[i]);

    free(f->positions);
    free(f->tri_indices);
    free(f->nodes);
    free(f->child_ptrs);
    free(f->rep_items);
    free(f->tess);
    free(f->names);
    memset(f, 0, sizeof(*f));
}

/* A quad strip of `tris` triangles lying in z=0, translated along X so this
   leaf does not overlap any other. Adjacent quads share an edge, which is
   what gives the COMPRESSED encoder's traversal something to chain. */
static int
build_leaf_geometry(fixture *f, uint32_t leaf, uint32_t tris, double x_offset,
    double *bbox_min, double *bbox_max)
{
    uint32_t quads = tris / 2;
    uint32_t num_positions = 2 * (quads + 1);
    double *pos;
    uint32_t *idx;
    uint32_t q;

    pos = (double *)malloc((size_t)num_positions * 3 * sizeof(double));
    idx = (uint32_t *)malloc((size_t)tris * 3 * sizeof(uint32_t));
    if (pos == NULL || idx == NULL)
    {
        free(pos);
        free(idx);
        return -1;
    }

    for (q = 0; q <= quads; q++)
    {
        pos[(size_t)(2 * q) * 3 + 0] = x_offset + (double)q;
        pos[(size_t)(2 * q) * 3 + 1] = 0.0;
        pos[(size_t)(2 * q) * 3 + 2] = 0.0;
        pos[(size_t)(2 * q + 1) * 3 + 0] = x_offset + (double)q;
        pos[(size_t)(2 * q + 1) * 3 + 1] = 1.0;
        pos[(size_t)(2 * q + 1) * 3 + 2] = 0.0;
    }

    for (q = 0; q < quads; q++)
    {
        idx[(size_t)(2 * q) * 3 + 0] = 2 * q;
        idx[(size_t)(2 * q) * 3 + 1] = 2 * q + 1;
        idx[(size_t)(2 * q) * 3 + 2] = 2 * q + 2;
        idx[(size_t)(2 * q + 1) * 3 + 0] = 2 * q + 1;
        idx[(size_t)(2 * q + 1) * 3 + 1] = 2 * q + 3;
        idx[(size_t)(2 * q + 1) * 3 + 2] = 2 * q + 2;
    }

    f->positions[leaf] = pos;
    f->tri_indices[leaf] = idx;

    bbox_min[0] = x_offset;
    bbox_min[1] = 0.0;
    bbox_min[2] = 0.0;
    bbox_max[0] = x_offset + (double)quads;
    bbox_max[1] = 1.0;
    bbox_max[2] = 0.0;
    return 0;
}

static void
bbox_union(double *min_a, double *max_a, const double *min_b, const double *max_b, int first)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if (first)
        {
            min_a[i] = min_b[i];
            max_a[i] = max_b[i];
        }
        else
        {
            if (min_b[i] < min_a[i]) min_a[i] = min_b[i];
            if (max_b[i] > max_a[i]) max_a[i] = max_b[i];
        }
    }
}

/* Builds the subtree rooted at `node` and returns its bounding box. `level`
   counts down; at 0 the node is a leaf and owns geometry. */
static int
build_subtree(fixture *f, const tree_spec *spec, prc_api_write_node *node,
    uint32_t level, double *out_min, double *out_max)
{
    uint32_t i;

    memset(node, 0, sizeof(*node));

    if (level == 0)
    {
        uint32_t leaf = f->next_leaf++;
        prc_api_write_tessellation *t = &f->tess[leaf];
        prc_api_write_rep_item *ri = &f->rep_items[leaf];

        if (build_leaf_geometry(f, leaf, spec->tris_per_leaf,
                (double)leaf * LEAF_SPACING, out_min, out_max) != 0)
            return -1;

        memset(t, 0, sizeof(*t));
        t->kind = spec->kind;
        t->positions = f->positions[leaf];
        t->num_positions = 2 * (spec->tris_per_leaf / 2 + 1);
        t->tri_indices = f->tri_indices[leaf];
        t->num_triangles = spec->tris_per_leaf;
        /* One face group covering the whole strip. TRIANGLES requires at
           least one face; COMPRESSED would accept NULL but is given the
           same shape so the two kinds differ only in encoding. */
        t->face_tri_counts = &f->tess[leaf].num_triangles;
        t->num_faces = 1;
        if (spec->kind == PRC_API_WRITE_TESS_KIND_TRIANGLES)
            t->must_calculate_normals = 1;

        ri->kind = PRC_API_WRITE_RI_SURFACE;
        ri->biased_tessellation_index = leaf + 1;  /* 1-based */
        ri->is_closed = 0;

        node->rep_items = ri;
        node->num_rep_items = 1;
        node->part_name = "leafpart";
    }
    else
    {
        uint32_t base = f->next_child_slot;
        int first = 1;

        f->next_child_slot += spec->branching;

        for (i = 0; i < spec->branching; i++)
        {
            prc_api_write_node *child = &f->nodes[f->next_node++];
            double cmin[3], cmax[3];

            if (build_subtree(f, spec, child, level - 1, cmin, cmax) != 0)
                return -1;
            f->child_ptrs[base + i] = child;
            bbox_union(out_min, out_max, cmin, cmax, first);
            first = 0;
        }

        node->children = &f->child_ptrs[base];
        node->num_children = spec->branching;
        /* Real producers attach an empty part at every intermediate level,
           and the bounding box on such a node is NOT ignored -- see
           prc_api_write_node's bbox_min comment. Setting it to the union of
           the children is the whole point of computing these. */
        node->has_empty_part = 1;
    }

    memcpy(node->bbox_min, out_min, sizeof(node->bbox_min));
    memcpy(node->bbox_max, out_max, sizeof(node->bbox_max));
    return 0;
}

static int
fixture_build(fixture *f, const tree_spec *spec)
{
    uint32_t levels;
    uint32_t total_internal = 0;
    uint32_t i;
    double rmin[3], rmax[3];

    memset(f, 0, sizeof(*f));

    f->num_leaves = ipow(spec->branching, spec->depth);
    f->expected_triangles = f->num_leaves * spec->tris_per_leaf;

    /* Nodes below the root: branching^1 + ... + branching^depth. */
    for (levels = 1; levels <= spec->depth; levels++)
        total_internal += ipow(spec->branching, levels);
    f->num_nodes = total_internal;

    f->nodes = (prc_api_write_node *)calloc(f->num_nodes, sizeof(prc_api_write_node));
    f->child_ptrs = (prc_api_write_node **)calloc(f->num_nodes, sizeof(prc_api_write_node *));
    f->rep_items = (prc_api_write_rep_item *)calloc(f->num_leaves, sizeof(prc_api_write_rep_item));
    f->tess = (prc_api_write_tessellation *)calloc(f->num_leaves, sizeof(prc_api_write_tessellation));
    f->positions = (double **)calloc(f->num_leaves, sizeof(double *));
    f->tri_indices = (uint32_t **)calloc(f->num_leaves, sizeof(uint32_t *));
    f->names = (char (*)[32])calloc(f->num_nodes, sizeof(*f->names));

    if (f->nodes == NULL || f->child_ptrs == NULL || f->rep_items == NULL ||
        f->tess == NULL || f->positions == NULL || f->tri_indices == NULL ||
        f->names == NULL)
    {
        fixture_free(f);
        return -1;
    }

    if (build_subtree(f, spec, &f->root, spec->depth, rmin, rmax) != 0)
    {
        fixture_free(f);
        return -1;
    }

    /* Names are assigned after the walk so an index appears in the name; the
       generator itself does not care about them. */
    for (i = 0; i < f->num_nodes; i++)
    {
        snprintf(f->names[i], sizeof(f->names[i]), "node%u", (unsigned)i);
        f->nodes[i].name = f->names[i];
    }
    f->root.name = "root";
    return 0;
}

/* Depth of the read-back tree, counting the root as 1. */
static uint32_t
tree_depth(const prc_api_product *node)
{
    uint32_t best = 0;
    size_t i;

    if (node == NULL)
        return 0;
    for (i = 0; i < node->num_children; i++)
    {
        uint32_t d = tree_depth(&node->children[i]);
        if (d > best)
            best = d;
    }
    return best + 1;
}

static uint32_t
count_nodes_with_parts(const prc_api_product *node)
{
    uint32_t n = 0;
    size_t i;

    if (node == NULL)
        return 0;
    if (node->part != NULL)
        n++;
    for (i = 0; i < node->num_children; i++)
        n += count_nodes_with_parts(&node->children[i]);
    return n;
}

/* Decodes every tessellation and returns the total triangle count, so a
   structurally correct tree whose geometry did not survive still fails.

   The call order matters and is not obvious: the faces array has to be sized
   with prc_api_get_number_faces and allocated by the CALLER before
   prc_api_initialize_tessellation, and vertices are then fetched one face at
   a time. The single-call form with face 0 and a NULL face pointer is only
   for wire and markup tessellations. This mirrors demos/quick_start, which
   is the canonical sequence. */
static uint32_t
count_decoded_triangles(prc_context *ctx, prc_api_data data,
    prc_api_product *model_tree, prc_api_tess *tesses, uint32_t num_tess)
{
    uint32_t total = 0;
    uint32_t k;

    for (k = 0; k < num_tess; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint8_t has_line = 0;
        uint32_t num_faces;
        size_t face;

        num_faces = prc_api_get_number_faces(ctx, data, k);
        tess->num_faces = num_faces;
        tess->tess_faces = (prc_api_face *)calloc(num_faces, sizeof(prc_api_face));
        if (tess->tess_faces == NULL)
            return 0;

        if (prc_api_initialize_tessellation(ctx, data, model_tree, k, tess,
                NULL, &has_line) < 0)
            return 0;
        if (tess->type == PRC_API_TESS_UNKNOWN)
            continue;

        for (face = 0; face < tess->num_faces; face++)
        {
            size_t p;

            if (prc_api_get_tessellation_vertices(ctx, data, k, (uint32_t)face,
                    tess->tess_faces + face, tess) < 0)
                return 0;

            for (p = 0; p < tess->tess_faces[face].num_graphic_primitives; p++)
            {
                prc_api_graphic_primitive prim;
                memset(&prim, 0, sizeof(prim));
                if (prc_api_get_graphics_primitive(ctx, data, tess,
                        (uint32_t)face, p, &prim) != 0)
                    continue;
                if (prim.type == PRC_API_TRIANGLES)
                    total += (uint32_t)(prim.num_indices / 3);
            }
        }
    }
    return total;
}

static void
run_shape(prc_context *ctx, const tree_spec *spec)
{
    fixture f;
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    FILE *fid;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t num_tess = 0, num_line_tess = 0, num_exact = 0;
    uint32_t decoded;
    uint32_t i;
    clock_t start;
    double elapsed;

    printf("  sub-case: %s (branching %u, depth %u, %u leaves, %u tris/leaf, %s)\n",
        spec->label, (unsigned)spec->branching, (unsigned)spec->depth,
        (unsigned)ipow(spec->branching, spec->depth), (unsigned)spec->tris_per_leaf,
        spec->kind == PRC_API_WRITE_TESS_KIND_COMPRESSED ? "COMPRESSED" : "TRIANGLES");

    PRC_ASSERT_EQ(fixture_build(&f, spec), 0);

    start = clock();
    PRC_ASSERT_EQ(prc_api_write_prc_buffer(ctx, "tree_model", &f.root,
        f.tess, f.num_leaves, &buf, &buf_size), 0);
    elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    PRC_ASSERT_NOT_NULL(buf);
    PRC_ASSERT(buf_size > 0);

    /* Baseline metrics, deliberately printed rather than asserted -- see the
       header. bytes/triangle is the comparable number across shapes. */
    printf("    encoded %lu bytes (%.1f bytes/triangle), write %.3f s\n",
        (unsigned long)buf_size,
        (double)buf_size / (double)f.expected_triangles, elapsed);

    fid = fopen(TEST_TREE_FILENAME, "wb");
    PRC_ASSERT_NOT_NULL(fid);
    PRC_ASSERT_EQ(fwrite(buf, 1, buf_size, fid), buf_size);
    fclose(fid);
    prc_api_write_prc_buffer_free(ctx, buf);

    data = prc_api_open_contents(ctx, TEST_TREE_FILENAME);
    if (data == NULL)
        prc_api_print_error_stack(ctx);
    PRC_ASSERT_NOT_NULL(data);

    PRC_ASSERT_EQ(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products,
        &num_markups), 0);
    /* One part per geometry-bearing leaf, plus one per empty-part internal
       node; every node in the tree is a product occurrence. */
    /* num_parts is a preallocation CAPACITY, not a count -- measured at
       roughly 1.1x to 2x the real part count depending on tree shape -- so
       only the lower bound is meaningful here. num_products is exact:
       every written node, plus the written root, plus the model-file root
       that the writer wraps around it. */
    PRC_ASSERT(num_parts >= f.num_nodes + 1);
    PRC_ASSERT_EQ(num_products, f.num_nodes + 2);
    PRC_ASSERT_EQ(num_markups, 0);

    PRC_ASSERT_EQ(prc_api_create_model_tree(ctx, data, &model_tree, num_parts,
        num_products, num_markups), 0);
    PRC_ASSERT_NOT_NULL(model_tree);

    /* +2, not +1: one level for the root we wrote, and one more for the
       model-file root the writer wraps around it. A reader's model tree
       therefore always shows one more level than the caller built. */
    PRC_ASSERT_EQ(tree_depth(model_tree), spec->depth + 2);
    /* Every written node carries a part -- a real one at the leaves, an
       empty one at the internal nodes -- and so does the written root. The
       model-file root above it does not. */
    PRC_ASSERT_EQ(count_nodes_with_parts(model_tree), f.num_nodes + 1);

    PRC_ASSERT_EQ(prc_api_get_number_tessellations(ctx, data, model_tree,
        &num_tess, &num_line_tess, &num_exact), 0);
    PRC_ASSERT_EQ(num_tess, f.num_leaves);
    PRC_ASSERT_EQ(num_exact, 0);
    /* The two encodings differ here, and legitimately so. For TRIANGLES the
       reader counts a line tessellation only where the file actually stores
       wire indices, and these models store none. For COMPRESSED it counts
       one wherever the decoder derived a non-empty boundary/crease edge set
       for wireframe display (prc_tess_3d_compressed.number_of_edges, filled
       in by prc_decode_compressed_tess) -- which every one of these strips
       has, since a strip has a perimeter. So one per entry is correct, not
       a phantom; asserting it pins that derived-edge path against
       regression. */
    if (spec->kind == PRC_API_WRITE_TESS_KIND_COMPRESSED)
        PRC_ASSERT_EQ(num_line_tess, num_tess);
    else
        PRC_ASSERT_EQ(num_line_tess, 0);

    tesses = (prc_api_tess *)calloc(num_tess, sizeof(prc_api_tess));
    PRC_ASSERT_NOT_NULL(tesses);

    start = clock();
    decoded = count_decoded_triangles(ctx, data, model_tree, tesses, num_tess);
    elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    printf("    decoded %u triangles in %.3f s\n", (unsigned)decoded, elapsed);
    PRC_ASSERT_EQ(decoded, f.expected_triangles);

    prc_api_release_data(ctx, data, tesses, num_tess, NULL, 0, NULL, 0, model_tree);
    /* release_data frees each face's vertex buffer but deliberately not the
       faces array itself (the free is commented out in prc_api.c) -- the
       caller allocated it, so the caller frees it. */
    for (i = 0; i < num_tess; i++)
        free(tesses[i].tess_faces);
    free(tesses);
    fixture_free(&f);
    remove(TEST_TREE_FILENAME);
}

int
main(void)
{
    prc_context *ctx;
    uint32_t i;

    /* Flat-and-wide and deep-and-narrow are the two shapes this file exists
       for; the bushy case sits between them so a bug that only appears when
       a node has both siblings and children is not missed by testing only
       the extremes. Each shape is run in both encodings. */
    static const tree_spec shapes[] = {
        { "flat, 32 sibling parts",   32, 1, 8,  PRC_API_WRITE_TESS_KIND_COMPRESSED },
        { "flat, 32 sibling parts",   32, 1, 8,  PRC_API_WRITE_TESS_KIND_TRIANGLES  },
        { "deep chain, 12 levels",     1, 12, 8, PRC_API_WRITE_TESS_KIND_COMPRESSED },
        { "deep chain, 12 levels",     1, 12, 8, PRC_API_WRITE_TESS_KIND_TRIANGLES  },
        { "bushy, 3 wide x 4 deep",    3, 4, 8,  PRC_API_WRITE_TESS_KIND_COMPRESSED },
        { "bushy, 3 wide x 4 deep",    3, 4, 8,  PRC_API_WRITE_TESS_KIND_TRIANGLES  },
        { "flat, 8 parts x 500 tris",  8, 1, 500, PRC_API_WRITE_TESS_KIND_COMPRESSED }
    };

    PRC_TEST_BEGIN("assembly tree write/read round trip");

    ctx = prc_api_new_context(NULL);
    PRC_ASSERT_NOT_NULL(ctx);

    for (i = 0; i < sizeof(shapes) / sizeof(shapes[0]); i++)
        run_shape(ctx, &shapes[i]);

    /* Under PRC_ENABLE_DEBUG_MEMORY this fails if anything above leaked. */
    PRC_ASSERT_EQ(prc_api_release_context(ctx), 0);

    PRC_TEST_END;
}
