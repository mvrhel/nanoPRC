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
    uint32_t *global_idx = NULL;
    uint32_t *face_start = NULL;
    double *face_normals = NULL;
    uint32_t global_count = 0;
    uint32_t f, k, c, i, tri_cursor, check_sum;
    int has_texture;
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
    has_texture = (tex_indices != NULL && tex_coords != NULL && num_tex_coords > 0);
    if (has_texture && norm_indices == NULL && !must_calculate_normals)
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
    check_sum = 0;
    for (f = 0; f < num_faces; f++)
        check_sum += face_tri_counts[f];
    if (check_sum != num_triangles)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_tess_3d: face_tri_counts does not sum to num_triangles\n");
        return PRC_ERROR_INTERNAL;
    }

    /* Worst case is 3 entries per vertex (normal, texture, point) = 9 per triangle. */
    global_idx = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * (size_t)num_triangles * 9);
    face_start = (uint32_t *)prc_malloc(ctx, sizeof(uint32_t) * num_faces);
    if (global_idx == NULL || face_start == NULL)
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
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* size_of_line_attributes */
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* start_of_wire_data */
        if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail;                   /* size_of_sizes_wire */
        if (prc_bitwrite_uint32(ctx, s, has_texture ?
                PRC_FACETESSDATA_TriangleTextured :
                ((norm_indices != NULL || must_calculate_normals) ?
                    PRC_FACETESSDATA_Triangle : PRC_FACETESSDATA_TriangleOneNormal)) != 0)
            goto fail;                                                       /* used_entities_flag */
        if (prc_bitwrite_uint32(ctx, s, face_start[f]) != 0) goto fail;       /* start_triangulated */
        if (prc_bitwrite_uint32(ctx, s, 1) != 0) goto fail;                  /* size_of_triangulateddata */
        if (prc_bitwrite_uint32(ctx, s, face_tri_counts[f]) != 0) goto fail;  /* triangulateddata[0] */
        if (prc_bitwrite_uint32(ctx, s, has_texture ? 1u : 0u) != 0) goto fail; /* number_of_textured_coordinate_indexes */
        if (prc_bitwrite_bit(ctx, s, 0) != 0) goto fail;                     /* has_vertex_colors */
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
    if (face_normals != NULL) prc_free(ctx, face_normals);
    return ret;
}
