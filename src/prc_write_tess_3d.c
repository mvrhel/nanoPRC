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

#include <string.h>
#include "prc_write_tess_3d.h"
#include "prc_data.h"
#include "prc_vector_util.h"

/* The per-vertex normal/position index pairs for every face are NOT stored
   in that face's own triangulateddata (which holds only structural counts,
   e.g. a bare triangle count for our PRC_FACETESSDATA_Triangle /
   TriangleOneNormal-only case); they live concatenated in the tess_3d-level
   triangulated_index_array, and each face's start_triangulated is its
   starting offset into that shared array (see
   prc_tri_primitives_api.c's src_index_data / prc_internal_api_set_triangles,
   and prc_internal_api_get_normal_position_index for the exact per-vertex
   layout this mirrors). So the whole array is built in memory first, since
   every face's start_triangulated must be known before any face record is
   written, and the array's own size must be written before its contents. */
/* Table 143 VertexColors, for one entity group run.

   `count` is a count of vertex REFERENCES, which is what the read side derives
   from the face's entity groups -- the array carries no length of its own. The
   encoding is delta: the first colour in full, then one is_same bit per
   remaining entry and a full colour only where it changes, so a run of one
   colour costs three bytes and then a bit each.

   is_segment_color exists only on the wire path; a face never writes it (see
   prc_parse_vertexcolors, which reads it only when !is_face). b_optimized is
   always 0: the read side rejects 1 outright. */
static int
prc_write_vertex_colors(prc_context *ctx, prc_bit_write_state *s,
    const uint8_t *colors, uint32_t first, uint32_t count,
    int have_alpha, int is_wire, int per_segment)
{
    uint32_t comps = have_alpha ? 4u : 3u;
    uint32_t k, c;

    if (prc_bitwrite_bit(ctx, s, have_alpha ? 1 : 0) != 0) return -1;   /* is_rgba */
    if (is_wire)
        if (prc_bitwrite_bit(ctx, s, per_segment ? 1 : 0) != 0) return -1; /* is_segment_color */
    if (prc_bitwrite_bit(ctx, s, 0) != 0) return -1;                    /* b_optimized */

    for (c = 0; c < comps; c++)
        if (prc_bitwrite_uint8(ctx, s, colors[(size_t)first * comps + c]) != 0) return -1;

    for (k = 1; k < count; k++)
    {
        const uint8_t *prev = &colors[(size_t)(first + k - 1) * comps];
        const uint8_t *cur = &colors[(size_t)(first + k) * comps];
        int same = 1;

        for (c = 0; c < comps; c++)
            if (prev[c] != cur[c]) { same = 0; break; }

        if (prc_bitwrite_bit(ctx, s, same ? 1 : 0) != 0) return -1;
        if (!same)
            for (c = 0; c < comps; c++)
                if (prc_bitwrite_uint8(ctx, s, cur[c]) != 0) return -1;
    }
    return 0;
}

int
prc_write_tess_3d_ex(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_tess_3d_params *p)
{
    const double *positions;
    uint32_t num_positions;
    const double *normals;
    uint32_t num_normals;
    const uint32_t *tri_indices;
    const uint32_t *norm_indices;
    uint32_t num_triangles;
    const uint32_t *face_tri_counts;
    uint32_t num_faces;
    const double *tex_coords;
    uint32_t num_tex_coords;
    const uint32_t *tex_indices;
    int must_calculate_normals;
    double crease_angle_degrees;

    if (ctx == NULL || s == NULL || p == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_tess_3d_ex: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    positions = p->positions;
    num_positions = p->num_positions;
    normals = p->normals;
    num_normals = p->num_normals;
    tri_indices = p->tri_indices;
    norm_indices = p->norm_indices;
    num_triangles = p->num_triangles;
    face_tri_counts = p->face_tri_counts;
    num_faces = p->num_faces;
    tex_coords = p->tex_coords;
    num_tex_coords = p->num_tex_coords;
    tex_indices = p->tex_indices;
    must_calculate_normals = p->must_calculate_normals;
    crease_angle_degrees = p->crease_angle_degrees;

    {
    uint32_t *global_idx = NULL;
    uint32_t *face_start = NULL;
    uint32_t *face_tri_words = NULL;
    double *face_normals = NULL;
    uint32_t global_count = 0;
    uint32_t f, k, c, i, tri_cursor, check_sum;
    uint32_t total_fans = 0, total_fan_verts = 0;
    uint32_t total_strips = 0, total_strip_verts = 0;
    uint32_t color_cursor = 0;
    uint32_t idx_capacity;
    int has_texture;
    int has_fans, has_strips;
    int ret = PRC_ERROR_INTERNAL;

    (void)num_normals;

    if (ctx == NULL || s == NULL || positions == NULL || tri_indices == NULL ||
        face_tri_counts == NULL || num_faces == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_tess_3d: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }
    if (must_calculate_normals && norm_indices != NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL,
            "prc_write_tess_3d: must_calculate_normals is incompatible with supplied norm_indices\n");
        return PRC_ERROR_INTERNAL;
    }
    /* The tessellation carries the UVs; each primitive kind carries its own
       indices into them. A face drawn entirely as fans or strips therefore has
       no triangle-level tex_indices, so those cannot be what makes a
       tessellation textured. */
    has_texture = (tex_coords != NULL && num_tex_coords > 0);
    if (has_texture && num_triangles > 0 && tex_indices == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL,
            "prc_write_tess_3d: textured tessellation has triangles but no tex_indices\n");
        return PRC_ERROR_INTERNAL;
    }
    if (has_texture && num_triangles > 0 && norm_indices == NULL && !must_calculate_normals)
    {
        /* Would need PRC_FACETESSDATA_TriangleOneNormalTextured, which this
           project's own reader rejects with PRC_ERROR_NOT_IMPLEMENTED, so
           writing it would produce a file we cannot read back. See the
           header. */
        prc_error(ctx, PRC_ERROR_INTERNAL,
            "prc_write_tess_3d: texture coordinates require supplied normals or "
            "must_calculate_normals (TriangleOneNormalTextured is not supported)\n");
        return PRC_ERROR_INTERNAL;
    }
    /* One pass over the groups: establish which kinds are present, that every
       group is well formed, and the vertex totals the colour check needs. */
    for (f = 0; f < num_faces; f++)
    {
        const prc_api_write_face_groups *fg;
        uint32_t g;

        if (p->face_groups == NULL)
            break;
        fg = &p->face_groups[f];

        if ((fg->num_fans > 0 && fg->fans == NULL) ||
            (fg->num_strips > 0 && fg->strips == NULL))
        {
            prc_error(ctx, PRC_ERROR_INTERNAL,
                "prc_write_tess_3d_ex: face %u declares groups but supplies no array\n", f);
            return PRC_ERROR_INTERNAL;
        }
        for (g = 0; g < fg->num_fans; g++)
        {
            const prc_api_write_tri_group *grp = &fg->fans[g];
            if (grp->num_vertices < 3)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a fan needs at least 3 vertices\n");
                return PRC_ERROR_INTERNAL;
            }
            if (grp->vertex_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a fan needs vertex_indices\n");
                return PRC_ERROR_INTERNAL;
            }
            if (grp->normal_indices != NULL && norm_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a fan's normal_indices needs triangle norm_indices too\n");
                return PRC_ERROR_INTERNAL;
            }
            /* used_entities_flag carries one bit per group KIND per face, so
               every fan in a face must agree on whether it supplies normals.
               A mixed face has no representation in the format. */
            if ((grp->normal_indices != NULL) != (fg->fans[0].normal_indices != NULL))
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: face %u mixes fans with and without normal_indices, "
                    "which used_entities_flag cannot express\n", f);
                return PRC_ERROR_INTERNAL;
            }
            if (has_texture && grp->tex_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: face %u has texture coordinates, so its fans need "
                    "tex_indices too -- an untextured fan beside textured triangles would drop the UVs\n", f);
                return PRC_ERROR_INTERNAL;
            }
            if (has_texture && grp->normal_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a textured fan needs normal_indices "
                    "(PRC_FACETESSDATA_TriangleFanOneNormalTextured is not written)\n");
                return PRC_ERROR_INTERNAL;
            }
            total_fan_verts += grp->num_vertices;
        }
        for (g = 0; g < fg->num_strips; g++)
        {
            const prc_api_write_tri_group *grp = &fg->strips[g];
            if (grp->num_vertices < 3)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a strip needs at least 3 vertices\n");
                return PRC_ERROR_INTERNAL;
            }
            if (grp->vertex_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a strip needs vertex_indices\n");
                return PRC_ERROR_INTERNAL;
            }
            if (grp->normal_indices != NULL && norm_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a strip's normal_indices needs triangle norm_indices too\n");
                return PRC_ERROR_INTERNAL;
            }
            if ((grp->normal_indices != NULL) != (fg->strips[0].normal_indices != NULL))
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: face %u mixes strips with and without normal_indices, "
                    "which used_entities_flag cannot express\n", f);
                return PRC_ERROR_INTERNAL;
            }
            if (has_texture && grp->tex_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: face %u has texture coordinates, so its strips need "
                    "tex_indices too -- an untextured strip beside textured triangles would drop the UVs\n", f);
                return PRC_ERROR_INTERNAL;
            }
            if (has_texture && grp->normal_indices == NULL)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "prc_write_tess_3d_ex: a textured strip needs normal_indices "
                    "(PRC_FACETESSDATA_TriangleStripeOneNormalTextured is not written)\n");
                return PRC_ERROR_INTERNAL;
            }
            total_strip_verts += grp->num_vertices;
        }
        total_fans += fg->num_fans;
        total_strips += fg->num_strips;
    }
    has_fans = (total_fans > 0);
    has_strips = (total_strips > 0);

    if ((has_fans || has_strips) && must_calculate_normals)
    {
        /* Same reasoning as the textured case above: this project's own reader
           refuses the combination outright, in prc_tri_primitives_api.c --
           "if we have to compute the normals, we only support that case where
           we have pure triangles. No strips or fans." Writing it produces a
           file that is structurally valid, opens without error, and yields no
           geometry at all.

           The comment there also asks whether such files occur. Measured
           across 307 third-party files: 17,522 uncompressed TESS_3D entities,
           of which 6,878 set must_calculate_normals and 7,587 carry fans or
           strips -- and ZERO do both. The format permits the combination and
           no real writer uses it, so refusing costs nothing a caller can
           reach for, and supplying normals is the way to write fan geometry
           here. */
        prc_error(ctx, PRC_ERROR_INTERNAL,
            "prc_write_tess_3d_ex: fans and strips need supplied normals; "
            "must_calculate_normals is triangles-only, and the read side "
            "rejects the combination\n");
        return PRC_ERROR_INTERNAL;
    }
    if (p->vertex_colors != NULL)
    {
        uint32_t refs = num_triangles * 3u + total_fan_verts + total_strip_verts;

        if (p->num_vertex_colors != refs)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL,
                "prc_write_tess_3d_ex: vertex colours must cover every vertex reference "
                "(3 per triangle plus each fan and strip vertex)\n");
            return PRC_ERROR_INTERNAL;
        }
    }

    check_sum = 0;
    for (f = 0; f < num_faces; f++)
        check_sum += face_tri_counts[f];
    if (check_sum != num_triangles)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_tess_3d: face_tri_counts does not sum to num_triangles\n");
        return PRC_ERROR_INTERNAL;
    }

    /* Worst case is 3 entries per vertex (normal, texture, point): 9 per
       triangle, 3 per fan or strip vertex, plus one normal entry per group in
       the one-normal forms. */
    idx_capacity = (uint32_t)num_triangles * 9u
                 + (total_fan_verts + total_strip_verts) * 3u
                 + total_fans + total_strips;
    global_idx = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * (size_t)idx_capacity);
    face_start = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * num_faces);
    face_tri_words = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * num_faces);
    if (global_idx == NULL || face_start == NULL || face_tri_words == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_tess_3d\n");
        goto cleanup;
    }

    if (norm_indices == NULL && !must_calculate_normals)
    {
        face_normals = (double *)prc_malloc(ctx, sizeof(double) * 3 * num_faces);
        if (face_normals == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_tess_3d\n");
            goto cleanup;
        }
        tri_cursor = 0;
        for (f = 0; f < num_faces; f++)
        {
            const uint32_t *t = &tri_indices[(size_t)tri_cursor * 3];
            prc_vec3 v0, v1, v2, e1, e2, n;

            v0.x = positions[(size_t)t[0] * 3 + 0]; v0.y = positions[(size_t)t[0] * 3 + 1]; v0.z = positions[(size_t)t[0] * 3 + 2];
            v1.x = positions[(size_t)t[1] * 3 + 0]; v1.y = positions[(size_t)t[1] * 3 + 1]; v1.z = positions[(size_t)t[1] * 3 + 2];
            v2.x = positions[(size_t)t[2] * 3 + 0]; v2.y = positions[(size_t)t[2] * 3 + 1]; v2.z = positions[(size_t)t[2] * 3 + 2];
            prc_vec_sub(v1, v0, &e1);
            prc_vec_sub(v2, v0, &e2);
            prc_vec_cross(e1, e2, &n);
            if (prc_vec_normalize(&n) < 0)
                n.x = n.y = n.z = 0.0; /* degenerate first triangle: harmless zero normal */
            face_normals[(size_t)f * 3 + 0] = n.x;
            face_normals[(size_t)f * 3 + 1] = n.y;
            face_normals[(size_t)f * 3 + 2] = n.z;
            tri_cursor += face_tri_counts[f];
        }
    }

    tri_cursor = 0;
    for (f = 0; f < num_faces; f++)
    {
        face_start[f] = global_count;
        for (k = 0; k < face_tri_counts[f]; k++)
        {
            uint32_t t = tri_cursor + k;

            if (norm_indices != NULL)
            {
                for (c = 0; c < 3; c++)
                {
                    global_idx[global_count++] = norm_indices[(size_t)t * 3 + c] * 3;
                    if (has_texture)
                        global_idx[global_count++] = tex_indices[(size_t)t * 3 + c] * 2;
                    global_idx[global_count++] = tri_indices[(size_t)t * 3 + c] * 3;
                }
            }
            else if (must_calculate_normals)
            {
                /* Per spec, when must_calculate_normals is set the normal
                   index half of each pair is omitted entirely -- the array
                   holds bare position indices (see prc_adjust_offsets in
                   prc_parse_tess.c, the read-side counterpart of this). */
                for (c = 0; c < 3; c++)
                {
                    if (has_texture)
                        global_idx[global_count++] = tex_indices[(size_t)t * 3 + c] * 2;
                    global_idx[global_count++] = tri_indices[(size_t)t * 3 + c] * 3;
                }
            }
            else
            {
                global_idx[global_count++] = f * 3;
                for (c = 0; c < 3; c++)
                    global_idx[global_count++] = tri_indices[(size_t)t * 3 + c] * 3;
            }
        }
        tri_cursor += face_tri_counts[f];

        /* Fans, then strips: the order the read side visits them in. */
        if (p->face_groups != NULL)
        {
            const prc_api_write_face_groups *fg = &p->face_groups[f];
            uint32_t kind;

            for (kind = 0; kind < 2; kind++)
            {
                const prc_api_write_tri_group *groups = kind ? fg->strips : fg->fans;
                uint32_t ngroups = kind ? fg->num_strips : fg->num_fans;

                for (k = 0; k < ngroups; k++)
                {
                    const prc_api_write_tri_group *grp = &groups[k];

                    for (c = 0; c < grp->num_vertices; c++)
                    {
                        if (grp->normal_indices != NULL)
                            global_idx[global_count++] = grp->normal_indices[c] * 3;
                        else if (!must_calculate_normals && c == 0)
                            global_idx[global_count++] = f * 3;   /* one normal, on the first vertex only */
                        /* Between the normal and the position, exactly where
                           the triangle path puts it: the read side's
                           prc_internal_api_get_normal_texture_position_index
                           takes normal, then num_texture_coords indices, then
                           the position. Validation above guarantees a textured
                           group supplies normal_indices, so the one-normal
                           branch above is never the textured one. */
                        if (has_texture)
                            global_idx[global_count++] = grp->tex_indices[c] * 2;
                        global_idx[global_count++] = grp->vertex_indices[c] * 3;
                    }
                }
            }
        }
    }

    if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                          /* is_calculated */
    if (prc_bitwrite_uint32(ctx, s, num_positions * 3) != 0) goto fail;
    for (i = 0; i < num_positions * 3; i++)
        if (prc_bitwrite_double(ctx, s, positions[i]) != 0) goto fail;

    /* has_faces: the analogy to the compressed writer's e48cdfb bug #3 (set
       FALSE there, since that facility never emits exact B-Rep geometry)
       turned out not to transfer here -- three independently-produced
       real-world uncompressed PRC files (ElevationMeshIS_ePRC.pdf,
       xml-sample-wrl_ePRC.pdf, xml-sample-iv_ePRC.pdf; see
       dump_uncompressed_tess_fields.c) all write has_faces=TRUE for
       PRC_TYPE_TESS_3D despite presumably having the same "no real B-Rep"
       property this facility does. Matching real-producer convention. */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;                          /* has_faces */
    if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                          /* has_loops */
    if (prc_bitwrite_bit(ctx, s, must_calculate_normals ? 1 : 0) != 0) goto fail; /* must_calculate_normals */

    if (must_calculate_normals)
    {
        /* normal_recalculation_flags=0, matching xml-sample-wrl_ePRC.pdf's
           real-producer convention (see dump_uncompressed_tess_fields
           output). crease_angle is stored as raw degrees -- the read side
           (prc_parse_tess.c) multiplies by PI/180 itself. */
        if (prc_bitwrite_uint8(ctx, s, 0) != 0) goto fail;                    /* normal_recalculation_flags */
        if (prc_bitwrite_double(ctx, s, crease_angle_degrees) != 0) goto fail; /* crease_angle */
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* number_of_normal_coordinates */
    }
    else if (norm_indices != NULL)
    {
        if (prc_bitwrite_uint32(ctx, s, num_normals * 3) != 0) goto fail;
        for (i = 0; i < num_normals * 3; i++)
            if (prc_bitwrite_double(ctx, s, normals[i]) != 0) goto fail;
    }
    else
    {
        if (prc_bitwrite_uint32(ctx, s, num_faces * 3) != 0) goto fail;
        for (i = 0; i < num_faces * 3; i++)
            if (prc_bitwrite_double(ctx, s, face_normals[i]) != 0) goto fail;
    }

    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                       /* number_of_wire_indices */

    if (prc_bitwrite_uint32(ctx, s, global_count) != 0) goto fail;            /* number_of_triangulated_indicies */
    for (i = 0; i < global_count; i++)
        if (prc_bitwrite_uint32(ctx, s, global_idx[i]) != 0) goto fail;

    if (prc_bitwrite_uint32(ctx, s, num_faces) != 0) goto fail;               /* number_of_face_tessellation */
    for (f = 0; f < num_faces; f++)
    {
        if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_TESS_Face) != 0) goto fail;  /* tag */
        /* size_of_line_attributes: 0 means "no graphics here, inherit from the
           owner of the TESS_3D data", which is what this writer emitted for
           every face until now. A reader that resolves style per face then
           finds nothing, which is why our own textured files read back as
           untextured even though Acrobat draws them -- it resolves from the
           part-level style instead. Table 140 defines 1 as "one graphic
           associated with the whole face tessellation data", and the entry as
           (index_of_line_style + 1). */
        if (p->face_style_biased > 0)
        {
            if (prc_bitwrite_uint32(ctx, s, 1) != 0) goto fail;               /* size_of_line_attributes */
            if (prc_bitwrite_uint32(ctx, s, p->face_style_biased) != 0) goto fail;
        }
        else
        {
            if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;               /* size_of_line_attributes */
        }
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* start_of_wire_data */
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* size_of_sizes_wire */
        {
            const prc_api_write_face_groups *fg =
                (p->face_groups != NULL) ? &p->face_groups[f] : NULL;
            uint32_t flags = 0;
            uint32_t nfans = (fg != NULL) ? fg->num_fans : 0;
            uint32_t nstrips = (fg != NULL) ? fg->num_strips : 0;
            int multi_norm = (norm_indices != NULL || must_calculate_normals);
            /* Checked uniform across the face during validation, so the first
               group speaks for all of them. */
            int fans_multi = nfans > 0 &&
                (fg->fans[0].normal_indices != NULL || must_calculate_normals);
            int strips_multi = nstrips > 0 &&
                (fg->strips[0].normal_indices != NULL || must_calculate_normals);
            uint32_t words;

            if (face_tri_counts[f] > 0)
                flags |= has_texture ? PRC_FACETESSDATA_TriangleTextured
                       : (multi_norm ? PRC_FACETESSDATA_Triangle
                                     : PRC_FACETESSDATA_TriangleOneNormal);
            /* has_texture implies fans_multi/strips_multi -- a textured
               group without normal_indices is refused during validation --
               so the textured one-normal bits are never reachable here. */
            if (nfans > 0)
                flags |= has_texture ? PRC_FACETESSDATA_TriangleFanTextured
                       : (fans_multi ? PRC_FACETESSDATA_TriangleFan
                                     : PRC_FACETESSDATA_TriangleFanOneNormal);
            if (nstrips > 0)
                flags |= has_texture ? PRC_FACETESSDATA_TriangleStripeTextured
                       : (strips_multi ? PRC_FACETESSDATA_TriangleStripe
                                       : PRC_FACETESSDATA_TriangleStripeOneNormal);

            /* One count word per group kind present, plus one length word per
               fan and per strip. A face with no triangles writes no triangle
               count word at all -- the read side only looks for one when the
               corresponding flag bit is set. */
            words = (face_tri_counts[f] > 0 ? 1u : 0u)
                  + (nfans > 0 ? 1u + nfans : 0u)
                  + (nstrips > 0 ? 1u + nstrips : 0u);
            face_tri_words[f] = words;

            if (prc_bitwrite_uint32(ctx, s, flags) != 0) goto fail;           /* used_entities_flag */
            if (prc_bitwrite_uint32(ctx, s, face_start[f]) != 0) goto fail;   /* start_triangulated */
            if (prc_bitwrite_uint32(ctx, s, words) != 0) goto fail;           /* size_of_triangulateddata */

            if (face_tri_counts[f] > 0)
                if (prc_bitwrite_uint32(ctx, s, face_tri_counts[f]) != 0) goto fail;

            if (nfans > 0)
            {
                uint32_t g;

                if (prc_bitwrite_uint32(ctx, s, nfans) != 0) goto fail;
                for (g = 0; g < nfans; g++)
                {
                    uint32_t len = fg->fans[g].num_vertices;

                    /* The one-normal forms tag the length word; the read side
                       masks it off again (prc_internal_api_set_fans). */
                    if (prc_bitwrite_uint32(ctx, s,
                            fans_multi ? len : (len | PRC_FACETESSDATA_NORMAL_Single)) != 0)
                        goto fail;
                }
            }
            if (nstrips > 0)
            {
                uint32_t g;

                if (prc_bitwrite_uint32(ctx, s, nstrips) != 0) goto fail;
                for (g = 0; g < nstrips; g++)
                {
                    uint32_t len = fg->strips[g].num_vertices;

                    if (prc_bitwrite_uint32(ctx, s,
                            strips_multi ? len : (len | PRC_FACETESSDATA_NORMAL_Single)) != 0)
                        goto fail;
                }
            }
        }
        if (prc_bitwrite_uint32(ctx, s, has_texture ? 1u : 0u) != 0) goto fail; /* number_of_textured_coordinate_indexes */

        if (p->vertex_colors == NULL)
        {
            if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                 /* has_vertex_colors */
        }
        else
        {
            /* One colour per vertex reference in this face's groups, in the
               same order the index stream visits them. */
            uint32_t refs = face_tri_counts[f] * 3u;
            uint32_t g;

            if (p->face_groups != NULL)
            {
                const prc_api_write_face_groups *cfg = &p->face_groups[f];

                for (g = 0; g < cfg->num_fans; g++)
                    refs += cfg->fans[g].num_vertices;
                for (g = 0; g < cfg->num_strips; g++)
                    refs += cfg->strips[g].num_vertices;
            }

            if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;                 /* has_vertex_colors */
            if (prc_write_vertex_colors(ctx, s, p->vertex_colors, color_cursor, refs,
                                        p->vertex_colors_have_alpha, 0, 0) != 0)
                goto fail;

            color_cursor += refs;
        }

        /* behavior closes the record, and the read side consults it if and
           only if size_of_line_attributes was non-zero (Table 140, and
           prc_parse_tess.c). Writing one without the other desyncs this face
           and every face after it, so the two are emitted together or not at
           all. 7.8.6.1 defines it as the graphics behaviour of the entity
           owning the face, per behavior_bit_field in Table 34, where
           PRC_GRAPHICS_Show is "the entity is shown" -- the same value
           prc_write_tree.c already writes for the owning entity, having found
           the hard way that leaving it clear renders nothing. */
        if (p->face_style_biased > 0)
            if (prc_bitwrite_uint32(ctx, s, (uint32_t)PRC_GRAPHICS_Show) != 0) goto fail;
    }

    /* Counted in DOUBLES, not coordinates -- see the header and #810. */
    if (prc_bitwrite_uint32(ctx, s, has_texture ? num_tex_coords * 2u : 0u) != 0) goto fail; /* number_of_texture_coordinates */
    if (has_texture)
        for (i = 0; i < num_tex_coords * 2u; i++)
            if (prc_bitwrite_double(ctx, s, tex_coords[i]) != 0) goto fail;

    ret = 0;
    goto cleanup;

fail:
    ret = s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;

cleanup:
    if (global_idx != NULL) prc_free(ctx, global_idx);
    if (face_start != NULL) prc_free(ctx, face_start);
    if (face_tri_words != NULL) prc_free(ctx, face_tri_words);
    if (face_normals != NULL) prc_free(ctx, face_normals);
    return ret;
    }
}

/* The original entry point, now a thin call through. Kept because it is what
   every caller written before fans, strips and vertex colours existed uses,
   and because a plain triangle mesh needs none of the new fields. */
int
prc_write_tess_3d(prc_context *ctx, prc_bit_write_state *s,
    const double *positions, uint32_t num_positions,
    const double *normals, uint32_t num_normals,
    const uint32_t *tri_indices, const uint32_t *norm_indices,
    uint32_t num_triangles,
    const uint32_t *face_tri_counts, uint32_t num_faces,
    const double *tex_coords, uint32_t num_tex_coords,
    const uint32_t *tex_indices,
    int must_calculate_normals, double crease_angle_degrees)
{
    prc_write_tess_3d_params p;

    memset(&p, 0, sizeof(p));
    p.positions = positions;
    p.num_positions = num_positions;
    p.normals = normals;
    p.num_normals = num_normals;
    p.tri_indices = tri_indices;
    p.norm_indices = norm_indices;
    p.num_triangles = num_triangles;
    p.face_tri_counts = face_tri_counts;
    p.num_faces = num_faces;
    p.tex_coords = tex_coords;
    p.num_tex_coords = num_tex_coords;
    p.tex_indices = tex_indices;
    p.must_calculate_normals = must_calculate_normals;
    p.crease_angle_degrees = crease_angle_degrees;
    return prc_write_tess_3d_ex(ctx, s, &p);
}
