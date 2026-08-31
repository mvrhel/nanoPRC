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

#ifndef PRC_WRITE_COMPRESS_TESS_H
#define PRC_WRITE_COMPRESS_TESS_H

#include <stdint.h>
#include "prc_write_common.h"
#include "prc_data.h"
#include "prc_bit.h"

typedef struct prc_encode_edge_s
{
    uint32_t v0, v1;     /* deduplicated vertex indices, v0 < v1 */
    int32_t  tri0, tri1; /* -1 if boundary (only one adjacent triangle) */
} prc_encode_edge;

typedef struct prc_encode_mesh_s
{
    double   *positions;       /* num_positions * 3 doubles, deduplicated */
    uint32_t  num_positions;
    uint32_t *tri_indices;     /* num_triangles * 3, into positions[], after dedup + degenerate removal */
    uint32_t  num_triangles;
    /* num_triangles entries: tri_orig_index[k] is the index into the
       ORIGINAL, pre-preprocessing tri_indices/face array that surviving
       triangle k came from. Surviving triangles keep their relative input
       order (preprocessing only ever drops degenerate ones), so this is
       exactly the subsequence of 0..original_num_triangles-1 that
       survived -- needed to correctly align caller-supplied per-triangle
       data (face groups, per-corner normals) with the post-preprocessing
       triangle order every later encoding step operates on. */
    uint32_t *tri_orig_index;
    prc_encode_edge *edges;    /* one entry per unique undirected edge in the clean index array */
    uint32_t  num_edges;
    uint32_t *tri_component;   /* num_triangles entries: connected-component label per triangle */
    uint32_t  num_components;
    double    bbox[6];         /* xmin,ymin,zmin,xmax,ymax,zmax */
    double    tolerance_mm;    /* resolved tolerance actually used for dedup */
    /* Number of deduplicated vertices touched by 2+ triangle "fans" that
       don't share an edge with each other (see prc_encode_preprocess's own
       split-handling comment) -- see prc_api_mesh_has_nonmanifold_fans's
       doc comment (include/prc_api.h) for why callers may want to check
       this before choosing COMPRESSED. */
    uint32_t  nonmanifold_vertices;
} prc_encode_mesh;

/* MITIGATION (2026-07-26, mixed_chains/UK_original.stl/beetle_1000000.stl Acrobat blank-tree
   investigation): prc_encode_preprocess (src/prc_write_compress_tess.c) applies a deterministic,
   per-vertex position jitter of up to this many multiples of the resolved encoding tolerance, in
   each of x/y/z independently, to every deduplicated vertex -- see that function's own comment for
   the full rationale. This is now part of this write facility's documented position-fidelity
   contract: a decoded vertex may differ from its original input position by up to this factor
   (times sqrt(3) for the worst-case combined 3D displacement) times tolerance, not just ordinary
   quantization noise -- callers/tests comparing decoded output against original input must budget
   for this. Shared here (not left as a private constant in the .c file) so test code that verifies
   position fidelity uses the same value rather than a duplicated magic number. */
#define PRC_ENCODE_JITTER_TOLERANCE_FACTOR 100.0

int prc_encode_preprocess(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance,
    prc_encode_mesh *out);

/* Same as prc_encode_preprocess, plus skip_nonmanifold_edge_remap: when
   non-zero, skips the step that gives a private vertex pair to any triangle
   that's the 3rd-or-later one sharing a given edge. That step exists only
   because COMPRESSED's own EdgeBreaker-style traversal assumes at most 2
   triangles per edge; a caller with no such traversal (e.g. building plain
   uncompressed TRIANGLES output) can skip it and keep the (often much
   larger, on real-world meshes with many 3+-way edges) vertex count it
   would otherwise add down to just what prc_api_mesh_has_nonmanifold_fans'
   own vertex-fan splitting needs. prc_encode_preprocess is a thin wrapper
   for this with the flag hardcoded to 0 (existing behavior, unchanged). */
int prc_encode_preprocess_ex(prc_context *ctx,
    const double *positions, uint32_t num_positions,
    const uint32_t *tri_indices, uint32_t num_triangles,
    prc_write_tolerance tolerance,
    uint8_t skip_nonmanifold_edge_remap,
    prc_encode_mesh *out);

void prc_encode_preprocess_free(prc_context *ctx, prc_encode_mesh *m);

typedef struct prc_encode_traversal_result_s
{
    int32_t  *point_array;               /* 3 int32 per emitted point, DV triples */
    uint32_t  point_array_size;          /* == 3 * number of points emitted */
    uint8_t  *edge_status_array;         /* 1 byte per triangle: bit0 = right edge grows, bit1 = left edge grows */
    uint32_t  edge_status_array_size;    /* == num_triangles */
    int32_t  *triangle_face_array;       /* 1 per triangle, in traversal emission order */
    uint32_t  triangle_face_array_size;  /* == num_triangles */
    uint8_t  *points_is_reference_array; /* 1 byte (0/1) per reference-bit slot */
    uint32_t  points_is_reference_array_size;
    int32_t  *point_reference_array;     /* existing-vertex index per reference slot consumed */
    uint32_t  point_reference_array_size;
    double    origin[3];                 /* the one global chain origin (decoder's origin_array) */
    int32_t  *triangle_point_indices;    /* 3 per triangle, TRAVERSAL order, in the decoder's
                                            treated order: the vertices_out[] slot each corner
                                            of the triangle decodes into */
    uint32_t *triangle_mesh_order;       /* 1 per triangle, TRAVERSAL order: index of the input
                                            mesh triangle emitted at that traversal position */
    int32_t  *point_mesh_vertex;         /* decoder point index -> deduplicated mesh vertex
                                            (index into mesh->positions) */
    double   *decoded_positions;         /* 3 per decoder point: the decoder-exact reconstructed
                                            position, so normal bases derived from these match
                                            the decoder's own basis construction */
    uint32_t  num_decoded_points;
    uint8_t  *triangle_reversed;         /* 1 per triangle, TRAVERSAL order: the normal_was_reversed
                                            bit decided inline during traversal from real_normals
                                            (see prc_encode_traversal); all zero if real_normals
                                            was NULL. Reindex by triangle_mesh_order for mesh order. */
} prc_encode_traversal_result;

/* Optional per-decoder-point diagnostics captured during traversal, one entry
   per emitted point (num_decoded_points). reconstructed_position is a stub
   for the upcoming reconstruction/analysis phase and is always zeroed. */
typedef struct prc_vertex_analysis_s
{
    float original_position[3];
    float reconstructed_position[3];  /* stub: always {0,0,0} this phase */
    uint32_t chain_index;
    uint32_t chain_offset;
} prc_vertex_analysis;

/* analysis_out may be NULL to skip analysis capture entirely; when non-NULL
   it receives a caller-owned (prc_free) array of *analysis_count_out
   (== out->num_decoded_points) entries.

   real_normals may be NULL (no triangle ever marked reversed, prior
   behavior unchanged) or mesh->num_triangles*9 entries (3 doubles per
   corner, MESH order -- same shape/layout as prc_encode_normals_c2's
   corner_normals, deliberately NOT one-per-deduplicated-position: the
   decision needs TRAVERSAL corner 0's specific real normal to match
   prc_encode_normals_c2's own corner-0-specific rejection check exactly,
   see prc_encode_decide_reversed's comment for why a coarser per-position
   average made C2 reject almost every real mesh). When non-NULL, each
   triangle's normal_was_reversed bit is decided INLINE, at the moment its
   final decoder point-index order (idx[0..2]) is established (chain start
   or grow step), by comparing that triangle's corner-0 real supplied normal
   against the geometric normal derived from its own just-assigned decoded
   positions. This is deliberately
   NOT a two-pass (baseline-then-rebuild) design: reversing a triangle
   changes which of its two forward edges is pushed/popped first (see
   prc_encode_edge_status), which changes traversal order for everything
   downstream of it, which would invalidate a precomputed array's
   assumptions for those triangles -- confirmed empirically (measured 49%
   of triangles diverging between a precomputed baseline and reality on a
   real mesh). Deciding the bit at the moment idx[] is finalized, in the
   same single forward pass that establishes idx[] in the first place,
   has no such staleness: by construction, the bit used to swap left/right
   for triangle T is always based on T's own real, final vertex order.

   The resulting bit is used exactly like the old tri_reversed[] parameter
   to swap which physical edge of a triangle is treated as "right" vs
   "left" (mirroring the decoder's prc_set_left_right_edge_indices swap),
   and is ALSO returned via out->triangle_reversed so the caller doesn't
   need a separate post-pass to recover it. */
/* normals_are_proxy: 1 when real_normals holds the smoothed proxy normals
   synthesised for the must_recalculate_normals path, 0 when it holds the
   caller's genuine per-corner normals. It selects how a GROWING triangle's
   normal_was_reversed bit is decided -- inherited from the parent for
   proxies, decided geometrically for real normals. See the branch itself
   for why the right answer differs between the two. */
int prc_encode_traversal(prc_context *ctx, const prc_encode_mesh *mesh,
    const uint32_t *face_indices, double tolerance_mm,
    prc_encode_traversal_result *out,
    prc_vertex_analysis **analysis_out, uint32_t *analysis_count_out,
    const double *real_normals, uint8_t normals_are_proxy);

void prc_encode_traversal_free(prc_context *ctx, prc_encode_traversal_result *out);

/* Step C1: geometric-derivation encoding -- no real per-corner normals are
   stored; the decoder recomputes flat/averaged normals purely from triangle
   geometry plus a crease-angle threshold (must_recalculate_normals=1 on the
   wire). This function computes per-triangle normal reversal bits for that
   path. input_normals is 3 doubles per DEDUPLICATED mesh position
   (mesh->num_positions entries) -- the caller must have already reduced any
   per-original-vertex normals down to one per deduplicated position -- or NULL
   to request all-zero reversal bits. The returned array (traversal order,
   trav->edge_status_array_size entries) is owned by the caller (prc_free). */
int prc_encode_normals_c1(prc_context *ctx, const prc_encode_mesh *mesh,
    const prc_encode_traversal_result *trav, const double *input_normals,
    uint8_t **normal_is_reversed_out);

/* Step C2: per-corner supplied-normal encoding -- the caller's real normals
   are preserved, quantized into compact theta/phi angle codes per corner, so
   the decoder reconstructs values close to the originals rather than
   deriving them from geometry. corner_normals is 9 doubles per input
   mesh triangle (3 per corner, aligned with mesh->tri_indices order), so the
   same position can carry different normals on different triangles. Outputs
   the quantized normal_angle_array (2 entries per decode event) and the
   per-vertex-state normal_binary_data bit array (one 0/1 byte per bit) the
   decoder's non-planar path consumes; both are owned by the caller. */
int prc_encode_normals_c2(prc_context *ctx, const prc_encode_mesh *mesh,
    const prc_encode_traversal_result *trav, const double *corner_normals,
    const uint8_t *face_planar_candidate, uint32_t face_count,
    uint8_t **face_planar_effective_out,
    int32_t **normal_angle_array_out, uint32_t *normal_angle_count_out,
    uint8_t **normal_binary_data_out, uint32_t *normal_binary_data_size_out);

/* Step E: emit the complete PRC_TYPE_TESS_3D_Compressed bitstream (without
   the type tag, which the caller-side dispatcher owns) into an initialized
   prc_bit_write_state. Exactly one of the two normal-data sets applies:
   must_recalculate_normals != 0 selects the C1 fields, 0 selects the C2
   fields. is_face_planar (C2 only, ignored otherwise) is a caller-owned
   array of face_count entries (face_count == max(triangle_face_array)+1),
   or NULL to mark every face non-planar (this write facility's own
   encoder never detects/uses the planar-face shortcut for freshly
   generated content, so its one caller always passes NULL here --
   this parameter exists for diagnostics re-encoding a real file's
   already-decoded per-face planarity, which must be reproduced exactly
   to keep that file's own normal_angle_array/normal_binary_data valid). */
/* Per-face coplanarity for the is_face_planar flag (Table 175, "Optional; if
   must_recalculate_normals is FALSE"). Sets is_face_planar_out[f] to 1 where
   face f's triangles are coplanar AND consistently wound, 0 otherwise; the
   caller owns and frees the array. face_indices maps post-preprocessing
   triangle index -> face id, as built by prc_write_compress_tess_entry.
   Returns 0 and leaves *is_face_planar_out NULL when there is nothing to do.
   See the definition's header comment for the tolerance rationale and for
   why the flag cannot yet be written. */
int prc_encode_compute_face_planarity(prc_context *ctx, const prc_encode_mesh *mesh,
    const uint32_t *face_indices, uint32_t num_faces, uint8_t **is_face_planar_out);

int prc_write_compress_tess_to_stream(prc_context *ctx, prc_bit_write_state *state,
    const prc_encode_traversal_result *trav, double tolerance_mm,
    const uint8_t *normal_is_reversed_c1, double crease_angle_degrees,
    const int32_t *normal_angle_array, uint32_t normal_angle_array_count,
    const uint8_t *normal_binary_data, uint32_t normal_binary_data_size,
    uint8_t must_recalculate_normals, const uint8_t *is_face_planar);

/* Full orchestration (Steps A through E) for one PRC_TYPE_TESS_3D_Compressed
   tessellation entry, from raw caller-supplied geometry straight to bits:
   preprocess (dedup + degenerate removal) -> traversal -> normal encoding
   (C2 supplied-normals if `normals` is non-NULL, else C1 recalculated) ->
   bitstream emission. Mirrors prc_write_tess_3d's input shape exactly
   (same positions/normals/indices/face-group fields) so the two encoders
   are interchangeable from the caller's side -- only the wire format and
   these two extra parameters (tolerance, crease_angle_degrees; both only
   meaningful for the encoder's own quantization/weld and C1's flat-normal
   reconstruction) differ. face_tri_counts/num_faces may be NULL/0 to treat
   the whole entry as one face. */
int prc_write_compress_tess_entry(prc_context *ctx, prc_bit_write_state *s,
    const double *positions, uint32_t num_positions,
    const double *normals, uint32_t num_normals,
    const uint32_t *tri_indices, const uint32_t *norm_indices, uint32_t num_triangles,
    const uint32_t *face_tri_counts, uint32_t num_faces,
    prc_write_tolerance tolerance, double crease_angle_degrees);

#endif
