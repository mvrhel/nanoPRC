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
@file obj_export.c
@brief Optimized utility program to extract 3D polygon tessellations and textures from PRC files,
exporting to Wavefront OBJ + MTL + PNG maps with automatic texture deduplication.
 
The obj_export program is a high-performance, cross-platform command-line utility designed to extract 3D polygon tessellations,
material definitions, and embedded surface textures from Product Representation Compact (PRC) files or PDF containers,
converting them into industry-standard Wavefront OBJ asset bundles.

Program Features
- Automated Asset Unbundling: Processes a single PRC file and automatically outputs a matched set of resources:
a geometry file (.obj), a material library file (.mtl), and a collection of extracted texture maps (.png).

- Embedded Texture Extraction: Automatically detects embedded raster imagery maps mapped to individual faces,
extracts the uncompressed raw pixel streams, and saves them locally as portable PNG images.

- Smart Asset Linking: Dynamically generates localized, distinct material identifiers within the .mtl library,
binding surface color coefficients (ambient, diffuse, specular, emissive, and transparency) 
and linking extracted texture maps via the standard map_Kd syntax.

- Full Vertex Attributes Export: Formats and writes absolute geometric coordinates, including spatial positions (v),
explicit surface normal vectors (vn), and normalized texture mapping coordinates (vt).

- Multi-Topology Primitive Triangulation: Decodes and unwinds complex GPU primitives - including independent triangle listings,
high-density triangle strips, and triangle fans - translating them into uniform 3-element face records (f).

- Platform-Agnostic Path Resolving: Automatically parses input and output paths to isolate file stems and base names,
ensuring correct material library declarations (mtllib) and relative texture paths regardless of whether it is running on Linux or Windows.

Command-Line Usage
  The tool operates via a simple, non-interactive command-line interface requiring an explicit input file pathway and a target destination.

Syntax
    obj_export <input_file.prc> <output_file.obj>
Example Execution
    ./obj_export engine_assembly.prc output/engine.obj

Generated Output Artifacts
- Executing the program generates a clean suite of asset files in the designated target directory:

- Main Geometry File (<filename>.obj): Contains the indexed arrangement of all geometric points,
texture coordinates, and surface normals grouped sequentially by internal part names.
It references the companion material file at the header via mtllib.

- Material Library File (<filename>.mtl): Declares distinct material definitions for each
individual mesh component or face configuration. This file tracks transparency metrics (d),
shininess indices (Ns), and color spaces.

- Extracted Texture Maps (<filename>_tex_<tessellation_id>_<face_id>.png):
Generated only if embedded textures are present. These files use a structured numerical suffix
corresponding to the precise internal structural indices of the source PRC model to avoid asset collisions.

Limitations & Behavioral Constraints
- 3D Surface Tessellation Focus: The tool is explicitly optimized to capture solid surface envelopes
(PRC_API_TESS_3D and PRC_API_TESS_3D_Compressed). It ignores standalone wireframe curves (PRC_API_TESS_3D_Wire),
structural bounding outlines, and floating 2D markup sketches.

- Fixed Texture Export Format: Textures are extracted and uncompressed directly into PNG format
to ensure cross-platform compatibility and lossless fidelity. Alternative image encodings (such as JPEG or BMP) are not supported.

- Local Workspace Permissions: Because the utility isolates filename stems to dump companion
texture maps alongside the .obj file, it requires active write permissions within the targeted destination
folder or execution will fail during asset serialization. Existing files are overwritten without warning.
 
 */

#include <prc_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Embed STB Image Writer for portable texture image file generation */
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * @brief Structure to track unique texture assets to optimize disk operations 
 * and avoid writing duplicate image files.
 */
typedef struct {
    const unsigned char *data; /* Shallow pointer to the context's internal texture pixel buffer */
    uint32_t width;
    uint32_t height;
    uint32_t num_channels;
    char filename[256];        /* Local filename string to reuse across matching MTL references */
} TrackedTexture;

/**
 * @brief Centralized macro to check API return status codes, log clean diagnostics,
 * and redirect control flow to the cleanup label on failure.
 */
#define PRC_CHECK_RC(expr, msg) \
    do { \
        int rc_ = (expr); \
        if (rc_ < 0) { \
            fprintf(stderr, "Error: %s (code: %d)\n", (msg), rc_); \
            goto cleanup; \
        } \
    } while (0)

/**
 * @brief Helper to write a material entry into the companion .mtl file descriptor, 
 * supporting optional diffuse texture maps via map_Kd.
 */
static void write_mtl_material(FILE *mtl_out, const char *mat_name, const prc_api_material *mat, const char *texture_filename)
{
    fprintf(mtl_out, "newmtl %s\n", mat_name);
    fprintf(mtl_out, "  Kd %f %f %f\n", mat->diffuse[0], mat->diffuse[1], mat->diffuse[2]);
    fprintf(mtl_out, "  Ks %f %f %f\n", mat->specular[0], mat->specular[1], mat->specular[2]);
    fprintf(mtl_out, "  Ka %f %f %f\n", mat->ambient[0], mat->ambient[1], mat->ambient[2]);
    fprintf(mtl_out, "  Ke %f %f %f\n", mat->emissive[0], mat->emissive[1], mat->emissive[2]);
    fprintf(mtl_out, "  Ns %f\n", mat->shininess);
    fprintf(mtl_out, "  d %f\n", mat->diffuse_alpha);
    
    if (texture_filename && strlen(texture_filename) > 0)
    {
        fprintf(mtl_out, "  map_Kd %s\n", texture_filename);
    }
    fprintf(mtl_out, "\n");
}

/**
 * @brief Formats and writes a single triangle face record into the Wavefront OBJ standard stream,
 * utilizing independent global counters to prevent cross-attribute index drift.
 */
static void process_and_write_obj_triangle(FILE* obj_out, prc_api_vertex v1, prc_api_vertex v2, prc_api_vertex v3,
    uint32_t* v_count, uint32_t* vt_count, uint32_t* vn_count)
{
    uint8_t has_uv = (v1.uv_set && v2.uv_set && v3.uv_set);

    /* Capture current 1-based start offsets for this specific triangle */
    uint32_t v_idx = *v_count;
    uint32_t vt_idx = *vt_count;
    uint32_t vn_idx = *vn_count;

    /* 1. Write vertex spatial coordinates */
    fprintf(obj_out, "v %f %f %f\n", v1.position[0], v1.position[1], v1.position[2]);
    fprintf(obj_out, "v %f %f %f\n", v2.position[0], v2.position[1], v2.position[2]);
    fprintf(obj_out, "v %f %f %f\n", v3.position[0], v3.position[1], v3.position[2]);
    *v_count += 3;

    /* 2. Write texture coordinates only if explicitly available */
    if (has_uv)
    {
        fprintf(obj_out, "vt %f %f\n", v1.uv[0], v1.uv[1]);
        fprintf(obj_out, "vt %f %f\n", v2.uv[0], v2.uv[1]);
        fprintf(obj_out, "vt %f %f\n", v3.uv[0], v3.uv[1]);
        *vt_count += 3;
    }

    /* 3. Determine and write normals */
    float n1[3], n2[3], n3[3];
    if (v1.normal_set && v2.normal_set && v3.normal_set)
    {
        memcpy(n1, v1.normal, sizeof(float) * 3);
        memcpy(n2, v2.normal, sizeof(float) * 3);
        memcpy(n3, v3.normal, sizeof(float) * 3);
    }
    else
    {
        float u[3] = { v2.position[0] - v1.position[0], v2.position[1] - v1.position[1], v2.position[2] - v1.position[2] };
        float v[3] = { v3.position[0] - v1.position[0], v3.position[1] - v1.position[1], v3.position[2] - v1.position[2] };
        float nx = u[1] * v[2] - u[2] * v[1];
        float ny = u[2] * v[0] - u[0] * v[2];
        float nz = u[0] * v[1] - u[1] * v[0];
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len > 1e-7f)
        {
            nx /= len; ny /= len; nz /= len;
        }
        n1[0] = nx; n1[1] = ny; n1[2] = nz;
        memcpy(n2, n1, sizeof(float) * 3);
        memcpy(n3, n1, sizeof(float) * 3);
    }

    fprintf(obj_out, "vn %f %f %f\n", n1[0], n1[1], n1[2]);
    fprintf(obj_out, "vn %f %f %f\n", n2[0], n2[1], n2[2]);
    fprintf(obj_out, "vn %f %f %f\n", n3[0], n3[1], n3[2]);
    *vn_count += 3;

    /* 4. Emit Wavefront OBJ element face record using the decoupled index handles */
    if (has_uv)
    {
        /* Format: vertex/texture/normal */
        fprintf(obj_out, "f %u/%u/%u %u/%u/%u %u/%u/%u\n",
            v_idx, vt_idx, vn_idx,
            v_idx + 1, vt_idx + 1, vn_idx + 1,
            v_idx + 2, vt_idx + 2, vn_idx + 2);
    }
    else
    {
        /* Format: vertex//normal (safely skipping texture coordinates) */
        fprintf(obj_out, "f %u//%u %u//%u %u//%u\n",
            v_idx, vn_idx,
            v_idx + 1, vn_idx + 1,
            v_idx + 2, vn_idx + 2);
    }
}

/**
 * @brief Resolves base paths and filenames to safely manage platform-agnostic file writing.
 */
static void extract_stem_properties(const char *obj_path, char *out_stem_path, char *out_stem_filename, size_t max_len)
{
    strncpy(out_stem_path, obj_path, max_len - 1);
    out_stem_path[max_len - 1] = '\0';
    
    char *dot = strrchr(out_stem_path, '.');
    if (dot && strcmp(dot, ".obj") == 0)
    {
        *dot = '\0';
    }

    const char *slash1 = strrchr(out_stem_path, '/');
    const char *slash2 = strrchr(out_stem_path, '\\');
    const char *slash = (slash1 > slash2) ? slash1 : slash2;

    if (slash)
    {
        strncpy(out_stem_filename, slash + 1, max_len - 1);
    }
    else
    {
        strncpy(out_stem_filename, out_stem_path, max_len - 1);
    }
    out_stem_filename[max_len - 1] = '\0';
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage: obj_export <input_file.prc> <output_file.obj>\n");
        return 1;
    }

    const char *input_path = argv[1];
    const char *output_path = argv[2];

    prc_context *ctx = NULL;
    prc_api_data data = NULL;
    prc_api_product *model_tree = NULL;
    prc_api_tess *tesses = NULL;
    FILE *obj_file = NULL;
    FILE *mtl_file = NULL;

    /* Deduplication Tracking Registry */
    TrackedTexture *tracked_textures = NULL;
    size_t tracked_count = 0;
    size_t tracked_capacity = 0;

    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t total_tessellations = 0, total_line_tessellations = 0;
  
    uint32_t global_v_counter = 1;
    uint32_t global_vt_counter = 1;
    uint32_t global_vn_counter = 1;

    char stem_path[512];
    char stem_filename[512];
    extract_stem_properties(output_path, stem_path, stem_filename, sizeof(stem_path));

    char mtl_full_path[512];
    char mtl_local_filename[512];
    sprintf(mtl_full_path, "%s.mtl", stem_path);
    sprintf(mtl_local_filename, "%s.mtl", stem_filename);

    ctx = prc_api_new_context(NULL);
    if (!ctx)
    {
        fprintf(stderr, "Error: Failed to instantiate prc_api_new_context frame.\n");
        return 1;
    }

    data = prc_api_open_contents(ctx, input_path);
    if (!data)
    {
        fprintf(stderr, "Error: Failed to process or open input content pathway: %s\n", input_path);
        prc_api_release_context(ctx);
        return 1;
    }

    PRC_CHECK_RC(prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups),
                 "prc_api_prep_model_tree compilation failed");

    PRC_CHECK_RC(prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups),
                 "prc_api_create_model_tree node creation failed");

    PRC_CHECK_RC(prc_api_get_number_tessellations(ctx, data, model_tree, &total_tessellations, &total_line_tessellations),
                 "prc_api_get_number_tessellations count retrieval failed");

    if (total_tessellations > 0)
    {
        tesses = calloc(total_tessellations, sizeof(prc_api_tess));
        if (!tesses)
        {
            fprintf(stderr, "Error: Calloc memory allocation space failure for tessellation elements registry.\n");
            goto cleanup;
        }
    }

    for (uint32_t k = 0; k < total_tessellations; k++)
    {
        prc_api_tess *tess = &tesses[k];
        uint32_t num_faces = prc_api_get_number_faces(ctx, data, k);
        
        tess->num_faces = num_faces;
        tess->tess_faces = calloc(num_faces, sizeof(prc_api_face));
        if (!tess->tess_faces)
        {
            fprintf(stderr, "Error: Face array cache segment allocation failure.\n");
            goto cleanup;
        }

        uint8_t has_lines = 0;
        PRC_CHECK_RC(prc_api_initialize_tessellation(ctx, data, model_tree, k, tess, NULL, &has_lines),
                     "prc_api_initialize_tessellation block setup failure");

        if (tess->type == PRC_API_TESS_UNKNOWN || tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
        {
            continue;
        }

        for (uint32_t j = 0; j < tess->num_faces; j++)
        {
            PRC_CHECK_RC(prc_api_get_tessellation_vertices(ctx, data, model_tree, k, j, tess->tess_faces + j, tess),
                         "prc_api_get_tessellation_vertices mapping lookup failure");
        }
    }

    obj_file = fopen(output_path, "w");
    if (!obj_file)
    {
        fprintf(stderr, "Error: Could not open file destination OBJ stream layout path: %s\n", output_path);
        goto cleanup;
    }

    mtl_file = fopen(mtl_full_path, "w");
    if (!mtl_file)
    {
        fprintf(stderr, "Error: Could not open material destination MTL stream layout path: %s\n", mtl_full_path);
        goto cleanup;
    }

    fprintf(obj_file, "# Generated by nanoPRC - AGPLv3 PRC parsing library (c) CascadiaVoxel LLC.\n");
    fprintf(obj_file, "mtllib %s\n\n", mtl_local_filename);
    fprintf(mtl_file, "# Generated by nanoPRC - AGPLv3 PRC parsing library (c) CascadiaVoxel LLC.\n\n");

    for (uint32_t i = 0; i < total_tessellations; i++)
    {
        prc_api_tess *tess = &tesses[i];
        if (tess->type != PRC_API_TESS_3D && tess->type != PRC_API_TESS_3D_Compressed)
        {
            continue;
        }

        const char *part_boundary_name = (tess->name && strlen(tess->name) > 0) ? tess->name : "Unnamed_PRC_Component_Part";
        fprintf(obj_file, "\n# Part: %s\n", part_boundary_name);

        prc_api_tess_vertex_buffer *vertex_buf = (tess->type == PRC_API_TESS_3D_Compressed) ? &tess->tess_vertices : NULL;

        for (size_t f = 0; f < tess->num_faces; f++)
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

            char dynamic_mat_id[128];
            char texture_local_name[256] = {0};
            sprintf(dynamic_mat_id, "mat_tess_%u_face_%zu", i, f);

            /* Texture Optimization: Deduplicate embedded raster streams */
            if (face->is_texture && face->texture.data && face->texture.width > 0 && face->texture.height > 0)
            {
                int found_duplicate = 0;
                size_t current_texture_bytes = (size_t)face->texture.width * face->texture.height * face->texture.num_channels;

                /* Check registry for an identical byte layout */
                for (size_t t = 0; t < tracked_count; t++)
                {
                    if (tracked_textures[t].width == face->texture.width &&
                        tracked_textures[t].height == face->texture.height &&
                        tracked_textures[t].num_channels == face->texture.num_channels)
                    {
                        if (memcmp(tracked_textures[t].data, face->texture.data, current_texture_bytes) == 0)
                        {
                            /* Match found! Share the file link and skip the redundant write sequence */
                            strcpy(texture_local_name, tracked_textures[t].filename);
                            found_duplicate = 1;
                            break;
                        }
                    }
                }

                if (!found_duplicate)
                {
                    char texture_full_path[512];
                    sprintf(texture_full_path, "%s_tex_%u_%zu.png", stem_path, i, f);
                    sprintf(texture_local_name, "%s_tex_%u_%zu.png", stem_filename, i, f);

                    int write_ok = stbi_write_png(texture_full_path, 
                                                 (int)face->texture.width, 
                                                 (int)face->texture.height, 
                                                 (int)face->texture.num_channels, 
                                                 face->texture.data, 
                                                 (int)(face->texture.width * face->texture.num_channels));
                    if (write_ok)
                    {
                        /* Dynamically grow optimization array capacity if needed */
                        if (tracked_count >= tracked_capacity)
                        {
                            tracked_capacity = tracked_capacity == 0 ? 16 : tracked_capacity * 2;
                            TrackedTexture *new_arr = realloc(tracked_textures, tracked_capacity * sizeof(TrackedTexture));
                            if (!new_arr)
                            {
                                fprintf(stderr, "Error: Out of memory growing texture tracking array.\n");
                                goto cleanup;
                            }
                            tracked_textures = new_arr;
                        }
                        
                        /* Store metadata for subsequent comparison checks */
                        tracked_textures[tracked_count].data = face->texture.data;
                        tracked_textures[tracked_count].width = face->texture.width;
                        tracked_textures[tracked_count].height = face->texture.height;
                        tracked_textures[tracked_count].num_channels = face->texture.num_channels;
                        strncpy(tracked_textures[tracked_count].filename, texture_local_name, sizeof(tracked_textures[tracked_count].filename) - 1);
                        tracked_textures[tracked_count].filename[sizeof(tracked_textures[tracked_count].filename) - 1] = '\0';
                        tracked_count++;
                    }
                    else
                    {
                        fprintf(stderr, "Warning: Failed to write texture file: %s\n", texture_full_path);
                        texture_local_name[0] = '\0';
                    }
                }
            }

            /* Resolve material records */
            if (face->is_material)
            {
                write_mtl_material(mtl_file, dynamic_mat_id, &face->material, texture_local_name);
            }
            else if (tess->is_material)
            {
                write_mtl_material(mtl_file, dynamic_mat_id, &tess->tess_material, texture_local_name);
            }
            else
            {
                prc_api_material fallback_mat = {
                    .diffuse = { 0.80f, 0.80f, 0.80f },
                    .specular = { 0.25f, 0.25f, 0.25f },
                    .ambient = { 0.20f, 0.20f, 0.20f },
                    .emissive = { 0.00f, 0.00f, 0.00f },
                    .shininess = 32.0f,
                    .diffuse_alpha = 1.0f
                };
                write_mtl_material(mtl_file, dynamic_mat_id, &fallback_mat, texture_local_name);
            }

            fprintf(obj_file, "usemtl %s\n", dynamic_mat_id);

            /* Traverse graphic primitives and build faces */
            for (size_t p = 0; p < face->num_graphic_primitives; p++)
            {
                prc_api_graphic_primitive primitive;
                if (prc_api_get_graphics_primitive(ctx, data, tess, (uint32_t)f, p, &primitive) < 0)
                {
                    continue;
                }

                if (primitive.type == PRC_API_TRIANGLES)
                {
                    for (size_t idx = 0; idx + 2 < primitive.num_indices; idx += 3)
                    {
                        uint32_t i0 = primitive.indices[idx];
                        uint32_t i1 = primitive.indices[idx + 1];
                        uint32_t i2 = primitive.indices[idx + 2];

                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            // Update all triangle invocation references inside the primitive types parser loops:
                            process_and_write_obj_triangle(obj_file,
                                vertex_buf->vertices[i0],
                                vertex_buf->vertices[i1],
                                vertex_buf->vertices[i2],
                                &global_v_counter,
                                &global_vt_counter,
                                &global_vn_counter);
                        }
                    }
                }
                else if (primitive.type == PRC_API_STRIP)
                {
                    for (size_t m = 2; m < primitive.num_indices; m++)
                    {
                        uint32_t i0, i1, i2;
                        if (m % 2 == 0)
                        {
                            i0 = primitive.indices[m - 2];
                            i1 = primitive.indices[m - 1];
                            i2 = primitive.indices[m];
                        }
                        else
                        {
                            i0 = primitive.indices[m - 1];
                            i1 = primitive.indices[m - 2];
                            i2 = primitive.indices[m];
                        }

                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            // Update all triangle invocation references inside the primitive types parser loops:
                            process_and_write_obj_triangle(obj_file,
                                vertex_buf->vertices[i0],
                                vertex_buf->vertices[i1],
                                vertex_buf->vertices[i2],
                                &global_v_counter,
                                &global_vt_counter,
                                &global_vn_counter);
                        }
                    }
                }
                else if (primitive.type == PRC_API_FAN)
                {
                    for (size_t m = 2; m < primitive.num_indices; m++)
                    {
                        uint32_t i0 = primitive.indices[0];
                        uint32_t i1 = primitive.indices[m - 1];
                        uint32_t i2 = primitive.indices[m];

                        if (i0 < vertex_buf->num_vertices && i1 < vertex_buf->num_vertices && i2 < vertex_buf->num_vertices)
                        {
                            // Update all triangle invocation references inside the primitive types parser loops:
                            process_and_write_obj_triangle(obj_file,
                                vertex_buf->vertices[i0],
                                vertex_buf->vertices[i1],
                                vertex_buf->vertices[i2],
                                &global_v_counter,
                                &global_vt_counter,
                                &global_vn_counter);
                        }
                    }
                }
            }
        }
    }

    printf("OBJ export completed successfully. Unique textures written: %zu. Total vertices: %u\n", 
           tracked_count, global_v_counter - 1);

cleanup:
    if (tracked_textures)
    {
        free(tracked_textures);
    }
    if (obj_file)
    {
        fclose(obj_file);
    }
    if (mtl_file)
    {
        fclose(mtl_file);
    }
    if (ctx)
    {
        if (data)
        {
            prc_api_release_data(ctx, data, tesses, total_tessellations, NULL, 0, model_tree);
        }
        prc_api_release_context(ctx);
    }

    return 0;
}
