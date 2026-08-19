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

/**
 * @file stl_export.c
 * @brief Production utility to extract 3D tessellation data from PRC files and export to binary STL.
 *
 * This utility traverses the Product Representation Compact (PRC)
 * model tree and streams geometric tessellations into a high-performance, single-pass
 * binary STL file layout suitable for 3D printing and CAD interchange.
 *
 * - Lumps all faces into a single mesh without hierarchy or part structure.
 * - Only exports face normals, as the average of vertex normals or derived from the face geometry.
 * - Mesh connectivity is ignored, each triangle is independent using duplicate vertices.
 * - Binary only: STLA and other variants are not implemented.
 * - No dependencies, the exporter is completely self-contained here.
 * - Overwrites any existing output file without warning.
 * - Filters out any degenerate triangles before export.
 * - Only outputs Triangles, Tri-Strips, and Fans, ignoring all other PRC types.
 */

#include <prc_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define EPSILON 1e-7f

/**
 * @brief Macro to check the return code of an API call, print a descriptive failure message,
 * and transfer control to the centralized cleanup block if a negative error code is encountered.
 */
#define PRC_CHECK_RC(expr, msg) \
    do { \
        int rc_ = (expr); \
        if (rc_ < 0) { \
            fprintf(stderr, "Error: %s (code: %d)\n", (msg), rc_); \
            ret = 1; \
            goto cleanup; \
        } \
    } while (0)

/**
 * @brief Helper to compute the geometric normal of a triangle and check for degeneracy.
 *
 * @param v1 First vertex position.
 * @param v2 Second vertex position.
 * @param v3 Third vertex position.
 * @param out_normal Output normal array.
 * @return int 1 if the triangle is valid, 0 if it is degenerate (zero area).
 */
static int compute_geometric_normal(const float v1[3], const float v2[3], const float v3[3], float out_normal[3])
{
    float u[3] = { v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2] };
    float v[3] = { v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2] };

    /* Cross product: U x V */
    out_normal[0] = u[1] * v[2] - u[2] * v[1];
    out_normal[1] = u[2] * v[0] - u[0] * v[2];
    out_normal[2] = u[0] * v[1] - u[1] * v[0];

    float length = sqrtf(out_normal[0] * out_normal[0] +
                         out_normal[1] * out_normal[1] +
                         out_normal[2] * out_normal[2]);

    if (length < EPSILON)
    {
        /* Degenerate triangle / zero area */
        return 0;
    }

    /* Normalize */
    out_normal[0] /= length;
    out_normal[1] /= length;
    out_normal[2] /= length;

    return 1;
}

/**
 * @brief Formats and writes a single triangle record into the binary STL stream.
 */
static void write_stl_triangle(FILE *out, const float v1[3], const float v2[3], const float v3[3], const float normal[3])
{
    uint16_t attribute_byte_count = 0;

    fwrite(normal, sizeof(float), 3, out);
    fwrite(v1, sizeof(float), 3, out);
    fwrite(v2, sizeof(float), 3, out);
    fwrite(v3, sizeof(float), 3, out);
    fwrite(&attribute_byte_count, sizeof(uint16_t), 1, out);
}

/**
 * @brief Applies a transform's LINEAR (3x3, no translation) part to a normal
 * vector and re-normalizes. Exact for rotation + uniform scale (including
 * reflections); an approximation under non-uniform scale/shear, which would
 * strictly need the inverse-transpose of the linear part -- not attempted
 * here since PRC_TRANSFORMATION_NonUniformScale/NonOrtho are optional,
 * uncommon flags and getting this exactly right adds real complexity for a
 * rare case. Positions (see prc_api_transform_point, used on actual vertex
 * data below) are always exact regardless.
 */
static void transform_normal(const prc_api_transform *xform, const float in[3], float out[3])
{
    const double *m;
    float len;

    if (xform == NULL || xform->is_identity)
    {
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
        return;
    }

    m = xform->matrix;
    out[0] = (float)(m[0] * in[0] + m[4] * in[1] + m[8]  * in[2]);
    out[1] = (float)(m[1] * in[0] + m[5] * in[1] + m[9]  * in[2]);
    out[2] = (float)(m[2] * in[0] + m[6] * in[1] + m[10] * in[2]);

    len = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    if (len > EPSILON)
    {
        out[0] /= len; out[1] /= len; out[2] /= len;
    }
    else
    {
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
    }
}

/**
 * @brief Processes a single triangle vertex triplet, computes/averages the normal,
 * validates against degeneracy, and writes it directly to the binary STL output stream.
 *
 * @param out Output FILE stream.
 * @param v1 First vertex data structure.
 * @param v2 Second vertex data structure.
 * @param v3 Third vertex data structure.
 * @param xform Accumulated world-space transform for this instance (NULL or
 *              identity for no-op) -- applied to positions exactly, normals
 *              via their linear part only (see transform_normal).
 * @param count Pointer to the accumulated counter of exported triangles.
 */
static void process_and_write_triangle(FILE *out, prc_api_vertex v1, prc_api_vertex v2, prc_api_vertex v3, const prc_api_transform *xform, uint32_t *count)
{
    float computed_n[3];

    if (xform != NULL && !xform->is_identity)
    {
        double p[3];
        int i;
        prc_api_vertex *vs[3] = { &v1, &v2, &v3 };
        for (i = 0; i < 3; i++)
        {
            double in[3] = { vs[i]->position[0], vs[i]->position[1], vs[i]->position[2] };
            prc_api_transform_point(xform, in, p);
            vs[i]->position[0] = (float)p[0];
            vs[i]->position[1] = (float)p[1];
            vs[i]->position[2] = (float)p[2];
            if (vs[i]->normal_set)
            {
                float n_out[3];
                transform_normal(xform, vs[i]->normal, n_out);
                vs[i]->normal[0] = n_out[0];
                vs[i]->normal[1] = n_out[1];
                vs[i]->normal[2] = n_out[2];
            }
        }
    }

    if (!compute_geometric_normal(v1.position, v2.position, v3.position, computed_n))
    {
        return; /* Skip degenerate triangle */
    }

    float final_n[3];
    if (v1.normal_set && v2.normal_set && v3.normal_set)
    {
        /* Average the vertex normals provided by the PRC source */
        final_n[0] = (v1.normal[0] + v2.normal[0] + v3.normal[0]) / 3.0f;
        final_n[1] = (v1.normal[1] + v2.normal[1] + v3.normal[1]) / 3.0f;
        final_n[2] = (v1.normal[2] + v2.normal[2] + v3.normal[2]) / 3.0f;

        float n_len = sqrtf(final_n[0] * final_n[0] + final_n[1] * final_n[1] + final_n[2] * final_n[2]);
        if (n_len > EPSILON)
        {
            final_n[0] /= n_len;
            final_n[1] /= n_len;
            final_n[2] /= n_len;
        }
        else
        {
            memcpy(final_n, computed_n, sizeof(float) * 3);
        }
    }
    else
    {
        /* Fallback to cross-product generated face normal */
        memcpy(final_n, computed_n, sizeof(float) * 3);
    }

    write_stl_triangle(out, v1.position, v2.position, v3.position, final_n);
    (*count)++;
}

/**
 * @brief Writes one tessellation's triangle geometry to the STL stream,
 * with a given world-space transform applied to every vertex (NULL/identity
 * for no-op). Body unchanged from this file's original single flat
 * "for i in 0..total_tessellations" write loop, just parameterized by a
 * specific tess pointer + transform instead of an implicit flat index, so
 * it can be called once per model-tree rep-item instance (see
 * export_node_recursive) instead of once per tessellation index.
 */
static void write_tessellation_geometry(prc_context *ctx, prc_api_data data, FILE *stl_file,
    prc_api_tess *tess, const prc_api_transform *xform, uint32_t *triangles_exported)
{
    if (tess->type != PRC_API_TESS_3D && tess->type != PRC_API_TESS_3D_Compressed)
    {
        return;
    }

    prc_api_tess_vertex_buffer *vertex_buf = (tess->type == PRC_API_TESS_3D_Compressed) ? &tess->tess_vertices : NULL;

    /* COMPRESSED tessellations don't partition geometry by face: every
       face_index's primitive returns the SAME complete mesh (see
       prc_api_get_tessellation_vertices's own comment on this, in
       src/prc_tri_primitives_api.c) -- face_index only carries meaning
       for per-face STYLE lookup there, never for geometry. Walking every
       face and accumulating its "primitive" here would therefore re-emit
       the whole mesh once per face instead of once total. */
    uint32_t num_faces_to_walk = (tess->type == PRC_API_TESS_3D_Compressed) ? 1 : tess->num_faces;

    for (size_t f = 0; f < num_faces_to_walk; f++)
    {
        prc_api_face *face = &tess->tess_faces[f];
        if (tess->type == PRC_API_TESS_3D)
        {
            vertex_buf = &face->face_vertices;
        }

        if (!vertex_buf || !vertex_buf->vertices || vertex_buf->num_vertices == 0)
        {
            continue;
        }

        for (size_t p = 0; p < face->num_graphic_primitives; p++)
        {
            prc_api_graphic_primitive prim;
            if (prc_api_get_graphics_primitive(ctx, data, tess, (uint32_t)f, p, &prim) < 0)
            {
                continue;
            }

            /* Simplified iteration patterns matching core graphic type definitions */
            if (prim.type == PRC_API_TRIANGLES)
            {
                for (size_t idx = 0; idx + 2 < prim.num_indices; idx += 3)
                {
                    uint32_t i0 = prim.indices[idx];
                    uint32_t i1 = prim.indices[idx + 1];
                    uint32_t i2 = prim.indices[idx + 2];

                    if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                    {
                        process_and_write_triangle(stl_file, vertex_buf->vertices[i0], vertex_buf->vertices[i1], vertex_buf->vertices[i2], xform, triangles_exported);
                    }
                }
            }
            else if (prim.type == PRC_API_STRIP)
            {
                for (size_t m = 2; m < prim.num_indices; m++)
                {
                    uint32_t i0, i1, i2;
                    if (m % 2 == 0)
                    {
                        i0 = prim.indices[m - 2];
                        i1 = prim.indices[m - 1];
                        i2 = prim.indices[m];
                    }
                    else
                    {
                        i0 = prim.indices[m - 1];
                        i1 = prim.indices[m - 2];
                        i2 = prim.indices[m];
                    }

                    if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                    {
                        process_and_write_triangle(stl_file, vertex_buf->vertices[i0], vertex_buf->vertices[i1], vertex_buf->vertices[i2], xform, triangles_exported);
                    }
                }
            }
            else if (prim.type == PRC_API_FAN)
            {
                for (size_t m = 2; m < prim.num_indices; m++)
                {
                    uint32_t i0 = prim.indices[0];
                    uint32_t i1 = prim.indices[m - 1];
                    uint32_t i2 = prim.indices[m];

                    if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                    {
                        process_and_write_triangle(stl_file, vertex_buf->vertices[i0], vertex_buf->vertices[i1], vertex_buf->vertices[i2], xform, triangles_exported);
                    }
                }
            }
        }
    }
}

/**
 * @brief Recursively walks the model tree, accumulating each node's own
 * local (parent-relative) placement transform (prc_api_product::location)
 * into a running world transform via prc_api_update_transform, and writes
 * every rep-item's geometry it finds using that instance's correctly
 * composed world-space transform. This is what actually fixes real-world
 * multi-part assemblies whose parts are encoded in local/shared coordinate
 * space with a real per-instance placement transform -- previously ignored
 * entirely, leaving every such part positioned at whatever local origin its
 * geometry happened to be encoded around instead of its true assembly
 * position (e.g. a caliper's geometry sitting at the assembly's shared
 * local origin instead of mounted on its bracket).
 *
 * Marks each visited tessellation index in tess_visited so the caller can
 * catch any tessellation never referenced by the tree -- should not
 * normally happen in a well-formed file, but degrades gracefully (a
 * fallback identity-transformed pass, see main()) rather than silently
 * dropping geometry if it does.
 */
static void export_node_recursive(prc_context *ctx, prc_api_data data, FILE *stl_file,
    prc_api_tess *tesses, uint32_t total_tessellations, prc_api_product *node,
    prc_api_transform parent_world, uint32_t *triangles_exported, uint8_t *tess_visited)
{
    prc_api_transform node_world = parent_world;
    size_t k;

    prc_api_update_transform(ctx, &node_world, &node->location);

    if (node->part != NULL)
    {
        size_t ri;
        for (ri = 0; ri < node->part->num_rep_items; ri++)
        {
            prc_api_part *rep = &node->part->rep_items[ri];
            if (rep->biased_tess_index > 0 && rep->biased_tess_index - 1 < total_tessellations)
            {
                uint32_t tess_idx = rep->biased_tess_index - 1;
                write_tessellation_geometry(ctx, data, stl_file, &tesses[tess_idx], &node_world, triangles_exported);
                tess_visited[tess_idx] = 1;
            }
        }
    }

    for (k = 0; k < node->num_children; k++)
    {
        export_node_recursive(ctx, data, stl_file, tesses, total_tessellations,
            &node->children[k], node_world, triangles_exported, tess_visited);
    }
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage: stl_export <input_file.prc> <output_file.stl>\n");
        return 1;
    }

    const char *input_path = argv[1];
    const char *output_path = argv[2];

    prc_context *ctx = NULL;
    prc_api_data data = NULL;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    uint8_t *tess_visited = NULL;
    FILE *stl_file = NULL;
    int ret = 0;

    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t total_tessellations = 0, total_line_tessellations = 0;
    uint32_t triangles_exported = 0;
    uint32_t num_extra_geom_tess = 0;

    /* Initialize PRC context */
    ctx = prc_api_new_context(NULL);
    if (!ctx)
    {
        fprintf(stderr, "Error: Failed to create prc_api_new_context\n");
        return 1;
    }

    /* Parse content data */
    data = prc_api_open_contents(ctx, input_path);
    if (!data)
    {
        fprintf(stderr, "Error: Failed to open or parse input contents from %s\n", input_path);
        prc_api_release_context(ctx);
        return 1;
    }
    prc_api_print_error_stack(ctx);

    /* Construct assembly structural model tree using the simplified checking macro */
    PRC_CHECK_RC(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups),
                 "prc_api_prep_model_tree failed");

    PRC_CHECK_RC(prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups),
                 "prc_api_create_model_tree failed");

    PRC_CHECK_RC(prc_api_get_number_tessellations(ctx, data, model_tree, &total_tessellations, &total_line_tessellations, &num_extra_geom_tess),
                 "prc_api_get_number_tessellations failed");

    if (total_tessellations == 0)
    {
        fprintf(stderr, "Warning: No surface tessellations found in file.\n");
    }
    else
    {
        tesses = calloc(total_tessellations, sizeof(prc_api_tess));
        if (!tesses)
        {
            fprintf(stderr, "Error: Memory allocation failed for tessellation registry array\n");
            ret = 1;
            goto cleanup;
        }
    }

    /* Extract and deserialize topological vertices and faces */
    for (uint32_t k = 0; k < total_tessellations; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t num_faces = prc_api_get_number_faces(ctx, data, k);

        tess->num_faces = num_faces;
        tess->tess_faces = calloc(num_faces, sizeof(prc_api_face));
        if (!tess->tess_faces)
        {
            fprintf(stderr, "Error: Memory allocation failed for tessellation faces\n");
            ret = 1;
            goto cleanup;
        }

        uint8_t has_lines = 0;
        PRC_CHECK_RC(prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL, &has_lines),
                     "prc_api_initialize_tessellation failed");

        if (tess->type == PRC_API_TESS_UNKNOWN || tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
        {
            /* Skip elements without 3D polygon surface data */
            continue;
        }

        for (uint32_t j = 0; j < tess->num_faces; j++)
        {
            PRC_CHECK_RC(prc_api_get_tessellation_vertices(ctx, data, k, j, tess->tess_faces + j, tess),
                         "prc_api_get_tessellation_vertices failed");
        }
    }

    /* Initialize Output Binary STL File stream */
    stl_file = fopen(output_path, "wb");
    if (!stl_file)
    {
        fprintf(stderr, "Error: Could not open output destination file path: %s\n", output_path);
        ret = 1;
        goto cleanup;
    }

    /* Step 1: Write an 80-byte header placeholder block */
    char header[80];
    snprintf(header, sizeof(header), "Generated by nanoPRC - AGPLv3 PRC parsing library (c) CascadiaVoxel LLC.");
    fwrite(header, 1, 80, stl_file);

    /* Step 2: Write a 4-byte temporary zero placeholder tracking total triangle counts */
    uint32_t count_placeholder = 0;
    fwrite(&count_placeholder, sizeof(uint32_t), 1, stl_file);

    /* Step 3: walk the model tree, applying each rep-item instance's
       correctly composed world-space placement transform (see
       export_node_recursive/prc_api_update_transform/
       prc_api_transform_point). Fixes this exporter's previous flat,
       tree-blind write loop, which left every part positioned at whatever
       LOCAL coordinate space it happened to be encoded in -- correct for
       parts encoded directly in world space, but wrong (typically
       clustered near a shared local origin) for real multi-part assemblies
       that encode shared/local part geometry plus a real per-instance
       placement transform. */
    if (total_tessellations > 0)
    {
        tess_visited = (uint8_t *)calloc(total_tessellations, sizeof(uint8_t));
        if (tess_visited == NULL)
        {
            fprintf(stderr, "Error: Memory allocation failed for tessellation visited-tracking array\n");
            ret = 1;
            goto cleanup;
        }
    }

    if (model_tree != NULL)
    {
        prc_api_transform root_transform;
        prc_api_set_transform_identity(ctx, &root_transform);
        export_node_recursive(ctx, data, stl_file, tesses, total_tessellations,
            model_tree, root_transform, &triangles_exported, tess_visited);
    }

    /* Fallback: any real 3D surface tessellation never reached via the tree
       walk above (should not happen in a well-formed file -- every real
       geometry tessellation should be referenced by at least one rep-item)
       is still exported, identity-transformed, matching this exporter's
       previous unconditional behavior. Degrades gracefully instead of
       silently dropping geometry if it ever does happen. */
    for (uint32_t i = 0; i < total_tessellations; i++)
    {
        if (!tess_visited[i])
        {
            prc_api_tess *tess = &tesses[i];
            if (tess->type == PRC_API_TESS_3D || tess->type == PRC_API_TESS_3D_Compressed)
            {
                fprintf(stderr, "Warning: tessellation %u not referenced by any model-tree node -- exporting with identity placement.\n", i);
                write_tessellation_geometry(ctx, data, stl_file, tess, NULL, &triangles_exported);
            }
        }
    }

    /* Step 4: Seek back to the header placeholder offset to patch the final count */
    if (fseek(stl_file, 80, SEEK_SET) == 0)
    {
        fwrite(&triangles_exported, sizeof(uint32_t), 1, stl_file);
    }
    else
    {
        fprintf(stderr, "Error: Failed to patch triangle count header metadata layout.\n");
    }

    printf("Export successful. Total valid triangles written: %u\n", triangles_exported);

cleanup:
    free(tess_visited);
    if (stl_file)
    {
        fclose(stl_file);
    }
    if (ctx)
    {
        if (data)
        {
            prc_api_release_data(ctx, data, tesses, total_tessellations, NULL, 0, NULL, 0, model_tree);
        }
        prc_api_release_context(ctx);
    }

    return ret;
}
