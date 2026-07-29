/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Follow-up to standalone_multichain.c's hypothesis test (which used
   5 UNIFORM 1-triangle disjoint pieces in one COMPRESSED entry and worked
   fine in real Acrobat, ruling out "multiple chains in one entry" as a
   defect on its own). Investigating UK_original.stl's remaining Acrobat
   blank-tree bug (2026-07-23) narrowed it to an 11-real-triangle minimal
   repro whose encoder-internal connectivity is NOT uniform: one 8-triangle
   fan, one 2-triangle pair, and one COMPLETELY ISOLATED single triangle
   (created by the non-manifold edge fix and non-manifold vertex fix both
   touching the same triangle in sequence, privatizing all 3 of its
   corners) -- num_components=3, chain sizes {8,2,1}. This tool tests that
   specific MIXED-size-chain shape in isolation, with clean synthetic
   geometry (three genuinely disjoint, non-manifold-free patches: an
   8-triangle fan, a 2-triangle strip, and 1 lone triangle), to see whether
   heterogeneous chain sizes -- not mere multiplicity -- is the actual
   defect, independent of anything about UK_original.stl's own real
   geometry or the non-manifold fixes that produced this shape there.

   HOW: usage: standalone_mixed_chains.exe <mode> <output.prc> [output.pdf]
   Always writes exactly one COMPRESSED entry, num_faces=1. mode selects
   which of the three disjoint pieces (8-triangle fan, 2-triangle strip,
   1 lone triangle) to include, to isolate exactly which size combination
   triggers the defect:
     all         - fan + strip + lone (8+2+1=11 tris) -- the original test
     fan_lone    - fan + lone only (8+1=9 tris), drop the strip
     strip_lone  - strip + lone only (2+1=3 tris), drop the fan
     fan_strip   - fan + strip only (8+2=10 tris), drop the lone triangle
                   (tests whether it's ANY size disparity, or specifically
                   a lone/1-triangle chain, that matters) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    int code;
    const char *mode;
    const char *out_pdf = NULL;
    double positions[64 * 3];
    uint32_t tri_indices[64 * 3];
    uint32_t num_positions = 0, num_triangles = 0;
    uint8_t include_fan = 0, include_strip = 0, include_lone = 0, include_minifan = 0;
    uint32_t fan_size = 8;
    uint32_t face_tri_counts[1];
    prc_api_write_tessellation tess;
    prc_api_write_rep_item rep_item;
    prc_api_write_node part_node, root;
    prc_api_write_node *part_node_ptr;
    int k;

    if (argc < 3)
    {
        printf("usage: %s <mode:all|fan_lone|strip_lone|fan_strip|small_triple|custom|custom_fan_strip|custom_fan_only> <output.prc> [output.pdf] [fan_size (custom/custom_fan_strip/custom_fan_only modes only, default 8)]\n", argv[0]);
        return 2;
    }
    mode = argv[1];
    if (argc >= 4) out_pdf = argv[3];

    if (strcmp(mode, "all") == 0) { include_fan = include_strip = include_lone = 1; }
    else if (strcmp(mode, "fan_lone") == 0) { include_fan = include_lone = 1; }
    else if (strcmp(mode, "strip_lone") == 0) { include_strip = include_lone = 1; }
    else if (strcmp(mode, "fan_strip") == 0) { include_fan = include_strip = 1; }
    else if (strcmp(mode, "small_triple") == 0) { include_minifan = include_strip = include_lone = 1; }
    else if (strcmp(mode, "custom") == 0)
    {
        include_fan = include_strip = include_lone = 1;
        if (argc >= 5) fan_size = (uint32_t)strtoul(argv[4], NULL, 10);
    }
    else if (strcmp(mode, "custom_fan_strip") == 0)
    {
        include_fan = include_strip = 1;
        if (argc >= 5) fan_size = (uint32_t)strtoul(argv[4], NULL, 10);
    }
    else if (strcmp(mode, "custom_fan_only") == 0)
    {
        /* Fan alone: no strip, no chain restart, single connected component
           -- breaks the perfect correlation between fan_size and total
           position/triangle count that every other mode has (each fan_size
           tested so far maps to a unique total, so fan_size, num_positions,
           and num_triangles could not be distinguished as the operative
           variable). If fan_size=8 alone still fails, the defect is
           intrinsic to the fan shape/count; if it works, the strip/
           chain-restart interaction is essential. */
        include_fan = 1;
        if (argc >= 5) fan_size = (uint32_t)strtoul(argv[4], NULL, 10);
    }
    else { printf("unrecognized mode '%s'\n", mode); return 2; }

    /* Fan: hub + fan_size-ring (default 8), centered at origin, radius 1.
       PRC_DIAG_FAN_ROTATION_DEGREES (default 0) rotates every ring point
       by a fixed angle -- for testing whether an exact x==y/x==-y
       coordinate symmetry (unavoidable at fan_size==8's 45-degree step,
       landing points exactly on the diagonals) is what actually matters,
       independent of fan_size==8 as a bare integer: a rotated fan keeps
       the same triangle/chain-restart structure and fan_size, but no
       longer has any point sitting exactly on a diagonal. */
    if (include_fan)
    {
        uint32_t base = num_positions;
        double rotation_rad = 0.0;
        double fan_y_offset = 0.0;
        if (getenv("PRC_DIAG_FAN_ROTATION_DEGREES") != NULL)
            rotation_rad = atof(getenv("PRC_DIAG_FAN_ROTATION_DEGREES")) * 3.14159265358979323846 / 180.0;
        /* PRC_DIAG_FAN_Y_OFFSET (default 0): shifts the fan (hub+ring) in Y
           only, WITHOUT rotating it -- keeps every ring point's own x==y/
           x==-y diagonal symmetry intact, but (since the fan is what sets
           the mesh's bbox_min, hence the encoder's global origin) changes
           origin's Y component without changing its X, breaking origin's
           OWN x==y symmetry. Tests whether the trigger is really "delta
           FROM ORIGIN has dx==dy" rather than "the point's own raw x==y" --
           those are different claims that PRC_DIAG_FAN_ROTATION_DEGREES
           alone can't distinguish, since rotating changes both at once. */
        if (getenv("PRC_DIAG_FAN_Y_OFFSET") != NULL)
            fan_y_offset = atof(getenv("PRC_DIAG_FAN_Y_OFFSET"));
        /* PRC_DIAG_FAN_RADIUS (default 1.0): scales the ring uniformly.
           Uniform scaling preserves every exact angular relationship
           (cos/sin ratios, hence the fan's own exact-tie structure) while
           shifting its quantized magnitudes -- and therefore the total bit
           count consumed before any later chain-restart field -- without
           touching fan_size or rotation. Added to test whether a co-
           occurring 22-bit value's BYTE-ALIGNMENT PHASE (not just its bit-
           length) is part of the mixed_chains/fan8 Acrobat blank-tree bug's
           trigger condition. */
        double fan_radius = 1.0;
        if (getenv("PRC_DIAG_FAN_RADIUS") != NULL)
            fan_radius = atof(getenv("PRC_DIAG_FAN_RADIUS"));
        positions[base * 3 + 0] = 0.0; positions[base * 3 + 1] = 0.0 + fan_y_offset; positions[base * 3 + 2] = 0.0;
        for (k = 0; k < (int)fan_size; k++)
        {
            double a = rotation_rad + 2.0 * 3.14159265358979323846 * k / (double)fan_size;
            positions[(base + 1 + k) * 3 + 0] = fan_radius * cos(a);
            positions[(base + 1 + k) * 3 + 1] = fan_radius * sin(a) + fan_y_offset;
            positions[(base + 1 + k) * 3 + 2] = 0.0;
        }
        for (k = 0; k < (int)fan_size; k++)
        {
            tri_indices[num_triangles * 3 + 0] = base;
            tri_indices[num_triangles * 3 + 1] = base + 1 + k;
            tri_indices[num_triangles * 3 + 2] = base + 1 + ((uint32_t)(k + 1) % fan_size);
            num_triangles++;
        }
        num_positions += fan_size + 1;
    }

    /* Mini fan: hub + 3-ring -- 3 triangles. Same shape as the 8-triangle
       fan above, just smaller, to test whether the {8,2,1} defect depends
       on chain-size MAGNITUDE/ratio or merely on having >=3 distinct sizes
       ({3,2,1} here) mixed in one entry. */
    if (include_minifan)
    {
        uint32_t base = num_positions;
        positions[base * 3 + 0] = 0.0; positions[base * 3 + 1] = 0.0; positions[base * 3 + 2] = 0.0;
        for (k = 0; k < 3; k++)
        {
            double a = 2.0 * 3.14159265358979323846 * k / 3.0;
            positions[(base + 1 + k) * 3 + 0] = cos(a);
            positions[(base + 1 + k) * 3 + 1] = sin(a);
            positions[(base + 1 + k) * 3 + 2] = 0.0;
        }
        for (k = 0; k < 3; k++)
        {
            tri_indices[num_triangles * 3 + 0] = base;
            tri_indices[num_triangles * 3 + 1] = base + 1 + k;
            tri_indices[num_triangles * 3 + 2] = base + 1 + ((k + 1) % 3);
            num_triangles++;
        }
        num_positions += 4;
    }

    /* 2-triangle strip, fixed offset (NOT num_positions-scaled -- this
       must match the ORIGINAL geometry that produced the confirmed-
       failing mixed_chains_acrobat_blanktree_repro.prc exactly, since
       that turned out to matter: even same-topology, different-absolute-
       coordinate regenerations changed the outcome. See PRC_DIAG_FORCE_
       TOLERANCE's own comment and the investigation memory/writeup for
       why this fixed-offset version replaced an earlier num_positions-
       scaled one that could no longer reproduce the original failure). */
    if (include_strip)
    {
        uint32_t base = num_positions;
        double ox = 20.0;
        /* PRC_DIAG_STRIP_BASE_X_OFFSET (default 20.0): moves the WHOLE
           strip's base position, keeping its shape (the two triangles'
           relative coordinates) identical -- added to test whether fan8's
           Acrobat blank-tree failure (confirmed 2026-07-24 to require fan8
           + THIS SPECIFIC STRIP, since fan8 alone and fan8+lone-triangle
           both work) depends on the strip's ABSOLUTE POSITION (a
           coincidental numeric collision with fan8's own quantized values,
           specific to ox=20) or merely on having a 2-triangle second chain
           at all, regardless of where. */
        if (getenv("PRC_DIAG_STRIP_BASE_X_OFFSET") != NULL)
            ox = atof(getenv("PRC_DIAG_STRIP_BASE_X_OFFSET"));
        /* PRC_DIAG_STRIP_SECOND_X_OFFSET (default 1.0): the strip's own
           2nd vertex's X-offset from its 1st. Exists to directly engineer
           (independent of fan_size/shape) whether this chain-restart's
           2nd emitted point's quantized X-delta-from-1st-point collides
           exactly with the 1st point's own quantized Y-delta-from-origin
           -- see the UK_original.stl/mixed_chains Acrobat blank-tree
           investigation memory/writeup for why that specific collision is
           the leading hypothesis for what actually triggers this bug. */
        double strip_second_x_offset = 1.0;
        if (getenv("PRC_DIAG_STRIP_SECOND_X_OFFSET") != NULL)
            strip_second_x_offset = atof(getenv("PRC_DIAG_STRIP_SECOND_X_OFFSET"));
        positions[base * 3 + 0] = ox + 0.0; positions[base * 3 + 1] = 0.0; positions[base * 3 + 2] = 0.0;
        positions[(base + 1) * 3 + 0] = ox + strip_second_x_offset; positions[(base + 1) * 3 + 1] = 0.0; positions[(base + 1) * 3 + 2] = 0.0;
        positions[(base + 2) * 3 + 0] = ox + 0.5; positions[(base + 2) * 3 + 1] = 1.0; positions[(base + 2) * 3 + 2] = 0.0;
        positions[(base + 3) * 3 + 0] = ox + 1.5; positions[(base + 3) * 3 + 1] = 1.0; positions[(base + 3) * 3 + 2] = 0.0;
        tri_indices[num_triangles * 3 + 0] = base;     tri_indices[num_triangles * 3 + 1] = base + 1; tri_indices[num_triangles * 3 + 2] = base + 2;
        num_triangles++;
        tri_indices[num_triangles * 3 + 0] = base + 1; tri_indices[num_triangles * 3 + 1] = base + 3; tri_indices[num_triangles * 3 + 2] = base + 2;
        num_triangles++;
        num_positions += 4;
    }

    /* Lone isolated triangle, fixed offset -- see the strip piece's own
       comment above for why this must NOT be num_positions-scaled. */
    if (include_lone)
    {
        uint32_t base = num_positions;
        double ox = 40.0;
        positions[base * 3 + 0] = ox + 0.0; positions[base * 3 + 1] = 0.0; positions[base * 3 + 2] = 0.0;
        positions[(base + 1) * 3 + 0] = ox + 1.0; positions[(base + 1) * 3 + 1] = 0.0; positions[(base + 1) * 3 + 2] = 0.0;
        positions[(base + 2) * 3 + 0] = ox + 0.5; positions[(base + 2) * 3 + 1] = 1.0; positions[(base + 2) * 3 + 2] = 0.0;
        tri_indices[num_triangles * 3 + 0] = base; tri_indices[num_triangles * 3 + 1] = base + 1; tri_indices[num_triangles * 3 + 2] = base + 2;
        num_triangles++;
        num_positions += 3;
    }

    face_tri_counts[0] = num_triangles;

    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = num_positions;
    tess.normals = NULL;
    tess.num_normals = 0;
    tess.tri_indices = tri_indices;
    tess.norm_indices = NULL;
    tess.num_triangles = num_triangles;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = 1;
    tess.crease_angle_degrees = 30.0;
    /* Left at the memset(0) default (resolves to prc_write_tol_relative(1e-6)
       -- i.e. 1e-6 * this mesh's own bbox diagonal) unless overridden --
       this is what makes tolerance differ silently between differently-
       offset/scaled geometry, which turned out to be the actual variable
       distinguishing this investigation's known-failing vs known-working
       repro (see the UK_original.stl Acrobat blank-tree investigation
       memory/writeup): same topology, different absolute coordinates ->
       different auto-computed tolerance -> different Acrobat outcome.
       PRC_DIAG_FORCE_TOLERANCE lets a caller pin an exact absolute
       tolerance (mm) regardless of this run's own geometry, to test
       tolerance as an isolated variable against a FIXED shape. */
    if (getenv("PRC_DIAG_FORCE_TOLERANCE") != NULL)
        tess.tolerance = prc_write_tol_absolute(atof(getenv("PRC_DIAG_FORCE_TOLERANCE")));

    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0;

    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    part_node.bbox_min[0] = -1.5; part_node.bbox_min[1] = -1.5; part_node.bbox_min[2] = -1.0;
    part_node.bbox_max[0] = 42.0; part_node.bbox_max[1] = 2.0; part_node.bbox_max[2] = 1.0;
    part_node.name = "mixed_chains_body";
    part_node.part_name = "mixed_chains_faces";

    part_node_ptr = &part_node;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "mixed_chains";

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    {
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;
        FILE *out;

        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, &tess, 1, &prc_buf, &prc_buf_size);
        if (code != 0)
        {
            printf("prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            prc_api_release_context(ctx);
            return 1;
        }

        out = fopen(argv[2], "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            printf("failed to write %s\n", argv[2]);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            return 1;
        }
        fclose(out);
        printf("mode=%s num_triangles=%u num_positions=%u\n", mode, num_triangles, num_positions);
        printf("Wrote %s (%zu bytes)\n", argv[2], prc_buf_size);

        if (out_pdf != NULL)
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = 20.0; view.eye[1] = -20.0; view.eye[2] = 15.0;
            view.target[0] = 20.0; view.target[1] = 0.0; view.target[2] = 0.0;
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;

            code = prc_api_pdf_embed_prc(ctx, out_pdf, prc_buf, prc_buf_size, &pdf_opts);
            if (code < 0)
            {
                printf("prc_api_pdf_embed_prc failed: %d\n", code);
                prc_api_print_error_stack(ctx);
            }
            else
            {
                printf("Wrote %s (with explicit Default view)\n", out_pdf);
            }
        }

        prc_api_write_prc_buffer_free(ctx, prc_buf);
    }

    prc_api_release_context(ctx);
    return 0;
}
