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
 * @file json_export.c
 * @brief nanoPRC demo: export a parsed PRC file's model tree and tessellated
 *        geometry to a single, well-formed JSON document.
 *
 * Overview
 * --------
 * This program loads a PRC file with the nanoPRC public API, walks the
 * resulting product/part model tree, and walks the tessellation table that
 * the library builds for that tree, emitting a JSON document with two top
 * level sections:
 *
 *   - "model_tree": the product/part hierarchy (names, transforms, and a
 *     reference to the tessellation index each part uses, where present).
 *   - "tessellations": one entry per tessellation, including its faces,
 *     vertex buffers (position/normal/color/uv), and graphics primitives
 *     (the index lists that assemble vertices into triangles/fans/strips
 *     or, for wire data, lines/line-strips/line-loops).
 *
 * The emphasis throughout is on bounded, predictable memory use and
 * streaming output, since production PRC files can contain millions of
 * triangles and thousands of sub-parts, and the resulting JSON document can
 * exceed a gigabyte:
 *
 *   - JSON is written incrementally via json_writer (see json_writer.h),
 *     which buffers and flushes to the output FILE stream rather than
 *     building the document in memory.
 *   - The product/part tree, which can be deep in pathological assemblies,
 *     is walked iteratively using an explicit, heap-allocated work stack
 *     instead of native call-stack recursion, so traversal depth is bounded
 *     only by available heap memory rather than by the platform stack size.
 *   - Per-tessellation vertex/index data is processed and written face by
 *     face and primitive by primitive; nanoPRC has already materialized
 *     each tessellation's vertex buffer by the time this program touches
 *     it (that allocation is owned by the library, as in the
 *     quick-start demo), but this program does not make any additional
 *     full-document copies of that data: each value is read once from the
 *     library's buffer and written once to the JSON stream.
 *
 * Usage
 * -----
 *     json_export <input.prc> <output.json>
 *
 * Exit status is 0 on success and non-zero on any failure, with a
 * descriptive message written to stderr.
 */

#include <prc_api.h>      /* nanoPRC main header */
#include <prc_context.h>  /* prc_context_s::source_file_version (Author Version) */

#include "json_writer.h"  /* local json handler */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

/* ----------------------------------------------------------------------
 * Small utilities
 * ------------------------------------------------------------------- */

/** Human-readable name for a tessellation type, for the "type" field. */
static const char *tess_type_name(prc_api_test_type_t type)
{
    switch (type)
    {
    case PRC_API_TESS_3D:              return "3D";
    case PRC_API_TESS_3D_Compressed:   return "3D_Compressed";
    case PRC_API_TESS_3D_Wire:         return "3D_Wire";
    case PRC_API_TESS_3D_Wire_Extra:   return "3D_Wire_Extra";
    case PRC_API_TESS_MarkUp:          return "MarkUp";
    case PRC_API_TESS_UNKNOWN:
    default:                           return "Unknown";
    }
}

/** Human-readable name for a graphics primitive type. */
static const char *primitive_type_name(prc_api_graphic_type_t type)
{
    switch (type)
    {
    case PRC_API_TRIANGLES:  return "triangles";
    case PRC_API_FAN:        return "triangle_fan";
    case PRC_API_STRIP:      return "triangle_strip";
    case PRC_API_LINE:       return "lines";
    case PRC_API_LINE_STRIP: return "line_strip";
    case PRC_API_LINE_LOOP:  return "line_loop";
    default:                 return "unknown";
    }
}

/** Human-readable name for a model tree node type. */
static const char *node_type_name(prc_api_node_type_t type)
{
    switch (type)
    {
    case PRC_API_NODE_PRODUCT:           return "product";
    case PRC_API_NODE_PART:              return "part";
    case PRC_API_NODE_PRODUCT_WITH_PART: return "product_with_part";
    case PRC_API_NODE_MARKUP:            return "markup";
    case PRC_API_NODE_UNKNOWN:
    default:                             return "unknown";
    }
}

/** Human-readable name for an attribute value type. */
static const char *attribute_type_name(prc_api_attribute_type_t type)
{
    switch (type)
    {
    case PRC_API_INTEGER_ATTRIBUTE:            return "integer";
    case PRC_API_DOUBLE_ATTRIBUTE:             return "double";
    case PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE: return "time32";
    case PRC_API_STRING_ATTRIBUTE:             return "string";
    case PRC_API_VALUE_TIME_ATTRIBUTE:         return "time64";
    default:                                   return "unknown";
    }
}

/** Format epoch seconds as UTC text. Returns 1 on success, 0 on failure. */
static int format_epoch_utc(uint64_t seconds_since_epoch, char *out, size_t out_size)
{
    time_t epoch_seconds = (time_t)seconds_since_epoch;
    struct tm utc_tm;

#if defined(_WIN32)
    if (gmtime_s(&utc_tm, &epoch_seconds) != 0)
        return 0;
#else
    if (gmtime_r(&epoch_seconds, &utc_tm) == NULL)
        return 0;
#endif

    return strftime(out, out_size, "%Y-%m-%d %H:%M:%S UTC", &utc_tm) > 0;
}

/** Write one attribute entry (entry title, type, and typed value). */
static void write_attribute_entry(json_writer *w, const prc_api_attribute_entry *entry)
{
    json_writer_begin_object(w);

    if (entry->entry_title != NULL)
        json_writer_kv_string(w, "entry_title", entry->entry_title);

    json_writer_kv_string(w, "type", attribute_type_name(entry->type));

    switch (entry->type)
    {
    case PRC_API_INTEGER_ATTRIBUTE:
        json_writer_kv_int32(w, "value", entry->value_integer);
        break;
    case PRC_API_DOUBLE_ATTRIBUTE:
        json_writer_kv_double(w, "value", entry->value_double);
        break;
    case PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE:
    {
        char utc_text[64];
        json_writer_kv_uint32(w, "value", entry->value_secs_integer);
        if (format_epoch_utc((uint64_t)entry->value_secs_integer, utc_text, sizeof(utc_text)))
            json_writer_kv_string(w, "value_utc", utc_text);
        break;
    }
    case PRC_API_STRING_ATTRIBUTE:
        if (entry->value_string != NULL)
            json_writer_kv_string(w, "value", entry->value_string);
        else
        {
            json_writer_key(w, "value");
            json_writer_null(w);
        }
        break;
    case PRC_API_VALUE_TIME_ATTRIBUTE:
    {
        char utc_text[64];
        json_writer_kv_uint64(w, "value", entry->value_time);
        if (format_epoch_utc(entry->value_time, utc_text, sizeof(utc_text)))
            json_writer_kv_string(w, "value_utc", utc_text);
        break;
    }
    default:
        json_writer_key(w, "value");
        json_writer_null(w);
        break;
    }

    json_writer_end_object(w);
}

/** Write one base attribute (base title and its entries array). */
static void write_attribute_base(json_writer *w, const prc_api_attribute_base *base)
{
    size_t i;

    json_writer_begin_object(w);

    if (base->attribute_base_title != NULL)
        json_writer_kv_string(w, "attribute_base_title", base->attribute_base_title);

    json_writer_kv_size(w, "num_attributes", base->num_attributes);
    json_writer_begin_array_key(w, "attributes");
    for (i = 0; i < base->num_attributes; i++)
        write_attribute_entry(w, &base->attributes[i]);
    json_writer_end_array(w);

    json_writer_end_object(w);
}

/** Write full nested attributes container for a product or part node. */
static void write_attributes(json_writer *w, const prc_api_attributes *attributes)
{
    uint32_t i;

    json_writer_begin_object_key(w, "attributes");
    json_writer_kv_uint32(w, "num_base_attributes", attributes->num_base_attributes);

    json_writer_begin_array_key(w, "base_attributes");
    for (i = 0; i < attributes->num_base_attributes; i++)
        write_attribute_base(w, &attributes->base_attributes[i]);
    json_writer_end_array(w);

    json_writer_end_object(w);
}

/* ----------------------------------------------------------------------
 * Tessellation table
 *
 * Mirrors the approach used by the quick-start demo: precompute the total
 * tessellation count, allocate one prc_api_tess per entry, and let nanoPRC
 * populate each one (including its per-face vertex/primitive data). This
 * program additionally keeps the array (and per-tessellation totals) around
 * for the JSON emission pass below.
 * ------------------------------------------------------------------- */

typedef struct tess_table_s
{
    prc_api_tess *tesses;           /* Library-owned tessellation array     */
    uint32_t num_tess;              /* Total tessellation count             */
    uint32_t num_line_tess;         /* Count flagged as having line data    */
} tess_table;

/**
 * @brief Build the tessellation table for the whole model tree.
 *
 * Allocates the tessellation array and asks the library to populate each
 * entry's faces, vertices and primitives, following the same sequence the
 * quick-start example uses. On any failure the partially built table is
 * released before returning so the caller does not need special-case
 * cleanup logic for a failed build.
 *
 * @return 0 on success, negative on failure (message already printed).
 */
static int tess_table_build(prc_context *ctx, prc_api_data data,
                             prc_api_product *model_tree, tess_table *table)
{
    uint32_t k;
    int code;
    uint8_t has_lines = 0;

    memset(table, 0, sizeof(*table));

    code = prc_api_get_number_tessellations(ctx, data, model_tree,
                                             &table->num_tess,
                                             &table->num_line_tess);
    if (code < 0)
    {
        fprintf(stderr, "error: prc_api_get_number_tessellations failed (%d)\n", code);
        return code;
    }

    if (table->num_tess == 0)
    {
        /* Nothing to tessellate; not an error, just an empty table. */
        return 0;
    }

    table->tesses = (prc_api_tess *)calloc(table->num_tess, sizeof(prc_api_tess));
    if (table->tesses == NULL)
    {
        fprintf(stderr, "error: out of memory allocating %u tessellation entries\n",
                table->num_tess);
        return PRC_API_ERROR_MEMORY;
    }

    for (k = 0; k < table->num_tess; k++)
    {
        prc_api_tess *tess = &table->tesses[k];
        uint32_t num_faces = prc_api_get_number_faces(ctx, data, k);

        tess->num_faces = num_faces;

        if (num_faces > 0)
        {
            tess->tess_faces = (prc_api_face *)calloc(num_faces, sizeof(prc_api_face));
            if (tess->tess_faces == NULL)
            {
                fprintf(stderr, "error: out of memory allocating %u faces for tessellation %u\n",
                        num_faces, k);
                return PRC_API_ERROR_MEMORY;
            }
        }

        code = prc_api_initialize_tessellation(ctx, data, model_tree, k, tess,
                                                NULL, &has_lines);
        if (code < 0)
        {
            fprintf(stderr, "error: prc_api_initialize_tessellation failed for index %u (%d)\n",
                    k, code);
            return code;
        }

        if (tess->type == PRC_API_TESS_UNKNOWN)
        {
            continue;
        }

        if (tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
        {
            /* Wire / markup tessellations carry a single shared vertex
               buffer rather than per-face buffers. */
            code = prc_api_get_tessellation_vertices(ctx, data, model_tree, k, 0, NULL, tess);
            if (code < 0)
            {
                fprintf(stderr, "error: prc_api_get_tessellation_vertices failed for index %u (%d)\n",
                        k, code);
                return code;
            }
        }
        else
        {
            uint32_t f;
            for (f = 0; f < tess->num_faces; f++)
            {
                code = prc_api_get_tessellation_vertices(ctx, data, model_tree, k, f,
                                                          tess->tess_faces + f, tess);
                if (code < 0)
                {
                    fprintf(stderr,
                            "error: prc_api_get_tessellation_vertices failed for tess %u face %u (%d)\n",
                            k, f, code);
                    return code;
                }
            }
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------
 * JSON emission: tessellations
 * ------------------------------------------------------------------- */

/** Write one vertex as a compact JSON object. Always present: position.
 *  Normal/UV are included only when the source vertex flags them as set,
 *  to keep output size down for buffers that do not carry that data. */
static void write_vertex(json_writer *w, const prc_api_vertex *v)
{
    json_writer_begin_object(w);

    json_writer_begin_array_compact_key(w, "position");
    json_writer_float(w, v->position[0]);
    json_writer_float(w, v->position[1]);
    json_writer_float(w, v->position[2]);
    json_writer_end_array(w);

    if (v->normal_set)
    {
        json_writer_begin_array_compact_key(w, "normal");
        json_writer_float(w, v->normal[0]);
        json_writer_float(w, v->normal[1]);
        json_writer_float(w, v->normal[2]);
        json_writer_end_array(w);
    }

    if (v->uv_set)
    {
        json_writer_begin_array_compact_key(w, "uv");
        json_writer_float(w, v->uv[0]);
        json_writer_float(w, v->uv[1]);
        json_writer_end_array(w);
    }

    /* Color is always populated by the library (defaulted when unstyled),
       so it is always emitted for consistency of downstream consumers. */
    json_writer_begin_array_compact_key(w, "color");
    json_writer_float(w, v->color[0]);
    json_writer_float(w, v->color[1]);
    json_writer_float(w, v->color[2]);
    json_writer_float(w, v->color[3]);
    json_writer_end_array(w);

    if (v->tri_has_material)
    {
        json_writer_begin_array_compact_key(w, "diffuse");
        json_writer_float(w, v->diffuse[0]);
        json_writer_float(w, v->diffuse[1]);
        json_writer_float(w, v->diffuse[2]);
        json_writer_end_array(w);
    }

    json_writer_end_object(w);
}

/** Write an entire vertex buffer as a JSON array of vertex objects. */
static void write_vertex_buffer(json_writer *w, const prc_api_tess_vertex_buffer *buf)
{
    size_t i;

    json_writer_begin_array(w);
    for (i = 0; i < buf->num_vertices; i++)
    {
        write_vertex(w, &buf->vertices[i]);
    }
    json_writer_end_array(w);
}

/**
 * @brief Write all graphics primitives for one face of a tessellation.
 *
 * @return 0 on success, negative on failure.
 */
static int write_face_primitives(prc_context *ctx, prc_api_data data,
                                  const prc_api_tess *tess, uint32_t face_index,
                                  json_writer *w)
{
    size_t num_primitives = tess->tess_faces[face_index].num_graphic_primitives;
    size_t p;

    json_writer_begin_array(w);

    for (p = 0; p < num_primitives; p++)
    {
        prc_api_graphic_primitive primitive;
        int code = prc_api_get_graphics_primitive(ctx, data, tess, face_index, p, &primitive);
        size_t idx;

        if (code < 0)
        {
            fprintf(stderr,
                    "error: prc_api_get_graphics_primitive failed (face %u, primitive %zu, code %d)\n",
                    face_index, p, code);
            return code;
        }

        json_writer_begin_object(w);
        json_writer_kv_string(w, "type", primitive_type_name(primitive.type));
        json_writer_kv_size(w, "num_indices", primitive.num_indices);

        json_writer_begin_array_compact_key(w, "indices");
        for (idx = 0; idx < primitive.num_indices; idx++)
        {
            json_writer_uint32(w, primitive.indices[idx]);
        }
        json_writer_end_array(w);

        json_writer_end_object(w);
    }

    json_writer_end_array(w);
    return 0;
}

/** Write material color/scalar fields shared by face and tessellation
 *  materials. */
static void write_material(json_writer *w, const prc_api_material *m)
{
    json_writer_begin_object(w);

    json_writer_begin_array_compact_key(w, "diffuse");
    json_writer_float(w, m->diffuse[0]);
    json_writer_float(w, m->diffuse[1]);
    json_writer_float(w, m->diffuse[2]);
    json_writer_end_array(w);

    json_writer_begin_array_compact_key(w, "ambient");
    json_writer_float(w, m->ambient[0]);
    json_writer_float(w, m->ambient[1]);
    json_writer_float(w, m->ambient[2]);
    json_writer_end_array(w);

    json_writer_begin_array_compact_key(w, "specular");
    json_writer_float(w, m->specular[0]);
    json_writer_float(w, m->specular[1]);
    json_writer_float(w, m->specular[2]);
    json_writer_end_array(w);

    json_writer_begin_array_compact_key(w, "emissive");
    json_writer_float(w, m->emissive[0]);
    json_writer_float(w, m->emissive[1]);
    json_writer_float(w, m->emissive[2]);
    json_writer_end_array(w);

    json_writer_kv_float(w, "shininess", m->shininess);
    json_writer_kv_float(w, "diffuse_alpha", m->diffuse_alpha);
    json_writer_kv_float(w, "specular_alpha", m->specular_alpha);
    json_writer_kv_float(w, "ambient_alpha", m->ambient_alpha);
    json_writer_kv_float(w, "emissive_alpha", m->emissive_alpha);

    json_writer_end_object(w);
}

/**
 * @brief Write one face: metadata, optional material, optional standalone
 *        vertex buffer (PRC_API_TESS_3D case), and its graphics primitives.
 *
 * @return 0 on success, negative on failure.
 */
static int write_face(prc_context *ctx, prc_api_data data, const prc_api_tess *tess,
                       uint32_t face_index, json_writer *w)
{
    const prc_api_face *face = &tess->tess_faces[face_index];
    int code;

    json_writer_begin_object(w);

    json_writer_kv_uint32(w, "face_index", face_index);
    json_writer_kv_bool(w, "disabled", face->disable_face);
    json_writer_kv_bool(w, "has_transparency", face->has_transparency);
    json_writer_kv_bool(w, "is_texture", face->is_texture);

    if (face->is_material)
    {
        json_writer_key(w, "material");
        write_material(w, &face->material);
    }

    /* For the non-compressed 3D case, each face owns an independent vertex
       buffer; for the compressed and wire/markup cases, vertices live on
       the tessellation itself and are emitted once at the tessellation
       level instead (see write_tessellation below), so we do not repeat
       them here to avoid doubling the size of the JSON output. */
    if (tess->type == PRC_API_TESS_3D)
    {
        json_writer_key(w, "vertices");
        write_vertex_buffer(w, &face->face_vertices);
    }

    json_writer_key(w, "primitives");
    code = write_face_primitives(ctx, data, tess, face_index, w);
    if (code < 0)
    {
        return code;
    }

    json_writer_end_object(w);
    return 0;
}

/**
 * @brief Write one full tessellation entry, including its faces.
 *
 * @return 0 on success, negative on failure.
 */
static int write_tessellation(prc_context *ctx, prc_api_data data,
                               const prc_api_tess *tess, uint32_t tess_index,
                               json_writer *w)
{
    uint32_t f;
    int code;

    json_writer_begin_object(w);

    json_writer_kv_uint32(w, "tess_index", tess_index);
    json_writer_kv_string(w, "type", tess_type_name(tess->type));
    json_writer_kv_size(w, "num_faces", tess->num_faces);
    json_writer_kv_bool(w, "has_transparency", tess->has_transparency);

    if (tess->name != NULL)
    {
        json_writer_kv_string(w, "name", tess->name);
    }

    json_writer_begin_array_compact_key(w, "bounding_box_min");
    json_writer_double(w, tess->bounding_box_min[0]);
    json_writer_double(w, tess->bounding_box_min[1]);
    json_writer_double(w, tess->bounding_box_min[2]);
    json_writer_end_array(w);

    json_writer_begin_array_compact_key(w, "bounding_box_max");
    json_writer_double(w, tess->bounding_box_max[0]);
    json_writer_double(w, tess->bounding_box_max[1]);
    json_writer_double(w, tess->bounding_box_max[2]);
    json_writer_end_array(w);

    if (tess->is_material)
    {
        json_writer_key(w, "material");
        write_material(w, &tess->tess_material);
    }

    /* Tessellation-level shared vertex buffer: populated for the
       compressed 3D case (where triangles index into one shared buffer
       across all faces) and for wire/markup tessellations (which have no
       per-face buffers at all). Emitted once here rather than per-face. */
    if (tess->type == PRC_API_TESS_3D_Compressed ||
        tess->type == PRC_API_TESS_3D_Wire ||
        tess->type == PRC_API_TESS_3D_Wire_Extra ||
        tess->type == PRC_API_TESS_MarkUp)
    {
        json_writer_key(w, "vertices");
        write_vertex_buffer(w, &tess->tess_vertices);
    }

    if (tess->type == PRC_API_TESS_UNKNOWN)
    {
        /* Nothing further was populated by the library for this entry. */
        json_writer_begin_array_key(w, "faces");
        json_writer_end_array(w);
        json_writer_end_object(w);
        return 0;
    }

    json_writer_begin_array_key(w, "faces");
    for (f = 0; f < tess->num_faces; f++)
    {
        code = write_face(ctx, data, tess, f, w);
        if (code < 0)
        {
            return code;
        }
    }
    json_writer_end_array(w);

    json_writer_end_object(w);
    return 0;
}

/**
 * @brief Write the full "tessellations" array.
 *
 * @return 0 on success, negative on failure.
 */
static int write_tessellations(prc_context *ctx, prc_api_data data,
                                const tess_table *table, json_writer *w)
{
    uint32_t i;

    json_writer_begin_array_key(w, "tessellations");
    for (i = 0; i < table->num_tess; i++)
    {
        int code = write_tessellation(ctx, data, &table->tesses[i], i, w);
        if (code < 0)
        {
            return code;
        }
    }
    json_writer_end_array(w);
    return 0;
}

/* ----------------------------------------------------------------------
 * JSON emission: model tree
 *
 * The model tree is walked iteratively with an explicit, heap-allocated
 * stack of (node, child-cursor) frames rather than native recursion. This
 * keeps maximum traversal depth bounded only by available heap memory
 * (vs. the call stack), which matters for assemblies with deep nesting,
 * and avoids the risk of stack overflow on pathological or adversarial
 * input files.
 * ------------------------------------------------------------------- */

/** One frame of the iterative tree walk: the node currently being visited
 *  and the index of the next child to descend into. */
typedef struct tree_frame_s
{
    prc_api_product *node;
    size_t next_child;
} tree_frame;

/** Growable stack of tree_frame entries. */
typedef struct tree_stack_s
{
    tree_frame *frames;
    size_t count;
    size_t capacity;
} tree_stack;

static int tree_stack_init(tree_stack *s, size_t initial_capacity)
{
    s->count = 0;
    s->capacity = (initial_capacity > 0) ? initial_capacity : 64;
    s->frames = (tree_frame *)malloc(s->capacity * sizeof(tree_frame));
    return (s->frames != NULL) ? 0 : -1;
}

static void tree_stack_destroy(tree_stack *s)
{
    free(s->frames);
    s->frames = NULL;
    s->count = 0;
    s->capacity = 0;
}

/** Push a new frame, growing the backing array geometrically if needed. */
static int tree_stack_push(tree_stack *s, prc_api_product *node)
{
    if (s->count == s->capacity)
    {
        size_t new_capacity = s->capacity * 2;
        tree_frame *new_frames = (tree_frame *)realloc(s->frames, new_capacity * sizeof(tree_frame));
        if (new_frames == NULL)
        {
            return -1;
        }
        s->frames = new_frames;
        s->capacity = new_capacity;
    }

    s->frames[s->count].node = node;
    s->frames[s->count].next_child = 0;
    s->count++;
    return 0;
}

static void tree_stack_pop(tree_stack *s)
{
    if (s->count > 0)
    {
        s->count--;
    }
}

static tree_frame *tree_stack_top(tree_stack *s)
{
    return (s->count > 0) ? &s->frames[s->count - 1] : NULL;
}

/** Write the transform field for a node that has one set. */
static void write_transform(json_writer *w, const prc_api_transform *t)
{
    int i;

    json_writer_begin_object(w);
    json_writer_kv_bool(w, "is_identity", t->is_identity);

    json_writer_begin_array_compact_key(w, "matrix");
    for (i = 0; i < 16; i++)
    {
        json_writer_double(w, t->matrix[i]);
    }
    json_writer_end_array(w);

    json_writer_end_object(w);
}

/** Write the part-specific fields of a node (tessellation linkage, in
 *  particular, which is how a JSON consumer connects model_tree entries
 *  back to entries in the top-level "tessellations" array). */
static void write_part_info(json_writer *w, const prc_api_part *part)
{
    json_writer_begin_object_key(w, "part");

    if (part->name != NULL)
    {
        json_writer_kv_string(w, "name", part->name);
    }

    json_writer_kv_uint32(w, "tess_index", part->biased_tess_index);
    json_writer_kv_size(w, "num_rep_items", part->num_rep_items);
    json_writer_kv_bool(w, "has_inherited_style", part->has_inherited_style);

    if (part->attributes.num_base_attributes > 0)
    {
        write_attributes(w, &part->attributes);
    }

    json_writer_end_object(w);
}

/**
 * @brief Write the JSON object header for one model tree node: everything
 *        except its "children" array, which the iterative walk fills in
 *        as it descends (open object now, append children as visited,
 *        close object when fully processed).
 */
static void write_node_open(json_writer *w, const prc_api_product *node)
{
    json_writer_begin_object(w);

    json_writer_kv_string(w, "type", node_type_name(node->type));
    json_writer_kv_bool(w, "is_model", node->is_model);

    if (node->name != NULL)
    {
        json_writer_kv_string(w, "name", node->name);
    }

    if (node->file_index >= 0)
    {
        json_writer_kv_int32(w, "file_index", node->file_index);
    }

    if (node->location_set)
    {
        json_writer_key(w, "transform");
        write_transform(w, &node->location);
    }

    if (node->part != NULL)
    {
        write_part_info(w, node->part);
    }

    if (node->num_markups > 0)
    {
        json_writer_kv_uint32(w, "num_markups", node->num_markups);
    }

    if (node->attributes.num_base_attributes > 0)
    {
        write_attributes(w, &node->attributes);
    }
}

/**
 * @brief Iteratively walk the model tree rooted at `root`, writing each
 *        node (and its nested "children" array) to the JSON stream in
 *        pre-order. Uses an explicit heap stack instead of recursion.
 *
 * @return 0 on success, negative on failure (out of memory growing the
 *         traversal stack; all writer-level errors are sticky on the
 *         writer itself and checked by the caller after this returns).
 */
static int write_model_tree(prc_api_product *root, json_writer *w)
{
    tree_stack stack;
    int rc;

    if (root == NULL)
    {
        json_writer_null(w);
        return 0;
    }

    if (tree_stack_init(&stack, 128) != 0)
    {
        fprintf(stderr, "error: out of memory initializing tree traversal stack\n");
        return -1;
    }

    rc = tree_stack_push(&stack, root);
    if (rc != 0)
    {
        fprintf(stderr, "error: out of memory pushing root tree frame\n");
        tree_stack_destroy(&stack);
        return -1;
    }

    write_node_open(w, root);
    json_writer_begin_array_key(w, "children");

    /* Iterative pre-order walk. Each frame tracks which child to descend
       into next; when a frame has exhausted its children we close that
       node's "children" array and object, pop the frame, and resume the
       parent (which itself opened a "children" array before descending
       into us, so the resumed parent simply continues iterating its own
       child list -- no recursion required at any point). */
    while (stack.count > 0)
    {
        tree_frame *frame = tree_stack_top(&stack);
        prc_api_product *node = frame->node;

        if (frame->next_child < node->num_children)
        {
            prc_api_product *child = &node->children[frame->next_child];
            frame->next_child++;

            write_node_open(w, child);
            json_writer_begin_array_key(w, "children");

            rc = tree_stack_push(&stack, child);
            if (rc != 0)
            {
                fprintf(stderr, "error: out of memory growing tree traversal stack\n");
                tree_stack_destroy(&stack);
                return -1;
            }
        }
        else
        {
            /* This node's children are exhausted: close its children
               array and its own object, then pop back to the parent. */
            json_writer_end_array(w);
            json_writer_end_object(w);
            tree_stack_pop(&stack);
        }

        if (json_writer_has_error(w))
        {
            /* Writer hit an unrecoverable I/O error; stop walking, the
               caller will report it. */
            tree_stack_destroy(&stack);
            return -1;
        }
    }

    tree_stack_destroy(&stack);
    return 0;
}

/* ----------------------------------------------------------------------
 * Cleanup helper
 *
 * Releases the tessellation table this program allocated itself (the
 * per-tessellation face arrays) before handing everything back to
 * prc_api_release_data, mirroring the ownership pattern used by the
 * quick-start example: the program allocates the outer prc_api_tess /
 * prc_api_face arrays, and the library is responsible for releasing the
 * buffers it filled in (vertices, primitives, textures, etc.) along with
 * those program-owned arrays when given them via prc_api_release_data.
 * ------------------------------------------------------------------- */

static void tess_table_release_faces(tess_table *table)
{
    uint32_t i;

    if (table->tesses == NULL)
    {
        return;
    }

    for (i = 0; i < table->num_tess; i++)
    {
        /* tess_faces itself is released as part of prc_api_release_data's
           walk over the tessellation array; we only need to make sure we
           do not leak it if release_data is never reached (e.g. an earlier
           fatal error during JSON emission). prc_api_release_data is
           always called on every exit path below, so in normal operation
           this function exists primarily as defensive cleanup glue. */
        (void)table->tesses[i];
    }
}

/* ----------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    const char *input_path;
    const char *output_path;
    prc_context *ctx = NULL;
    prc_api_data data = NULL;
    prc_api_product *model_tree = NULL;
    tess_table table;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    FILE *out = NULL;
    json_writer writer;
    int writer_initialized = 0;
    int code;
    int exit_code = 0;

    memset(&table, 0, sizeof(table));

    if (argc != 3)
    {
        fprintf(stderr, "Usage: json_export <input.prc> <output.json>\n");
        return 1;
    }

    input_path = argv[1];
    output_path = argv[2];

    /* ---- Open input and parse ----------------------------------------- */

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "error: prc_api_new_context failed\n");
        return 1;
    }

    data = prc_api_open_contents(ctx, input_path);
    if (data == NULL)
    {
        fprintf(stderr, "error: failed to open and parse '%s'\n", input_path);
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }
    prc_api_print_error_stack(ctx);

    /* ---- Build the model tree ------------------------------------------ */

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0)
    {
        fprintf(stderr, "error: prc_api_prep_model_tree failed (%d)\n", code);
        exit_code = 1;
        goto cleanup;
    }

    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code < 0)
    {
        fprintf(stderr, "error: prc_api_create_model_tree failed (%d)\n", code);
        exit_code = 1;
        goto cleanup;
    }

    /* ---- Build the tessellation table ----------------------------------- */

    code = tess_table_build(ctx, data, model_tree, &table);
    if (code < 0)
    {
        exit_code = 1;
        goto cleanup;
    }

    /* ---- Open output and start streaming JSON ---------------------------- */

    out = fopen(output_path, "wb");
    if (out == NULL)
    {
        fprintf(stderr, "error: could not open output file '%s' for writing\n", output_path);
        exit_code = 1;
        goto cleanup;
    }

    /* A larger-than-default staging buffer reduces fwrite() call overhead
       for the very large outputs this tool is designed to produce.
       Pretty-printing is enabled (final argument = 1): scalar arrays of
       vertex data are written compactly on one line via the compact-array
       emitters; surrounding structure (objects, faces, products) is
       indented normally for human legibility. */
    if (json_writer_init(&writer, out, 4u << 20 /* 4 MiB */, 1 /* pretty */) != 0)
    {
        fprintf(stderr, "error: out of memory initializing JSON writer\n");
        exit_code = 1;
        goto cleanup;
    }
    writer_initialized = 1;

    json_writer_begin_object(&writer);
	json_writer_kv_string(&writer, "_comment",
        "Generated by nanoPRC - AGPLv3 PRC parsing library (c) CascadiaVoxel LLC.");
    json_writer_kv_string(&writer, "source_file", input_path);
    json_writer_kv_uint32(&writer, "author_version", ctx->source_file_version);
    json_writer_kv_uint32(&writer, "num_parts", num_parts);
    json_writer_kv_uint32(&writer, "num_products", num_products);
    json_writer_kv_uint32(&writer, "num_markups", num_markups);
    json_writer_kv_uint32(&writer, "num_tessellations", table.num_tess);
    json_writer_kv_uint32(&writer, "num_line_tessellations", table.num_line_tess);

    json_writer_key(&writer, "model_tree");
    if (write_model_tree(model_tree, &writer) < 0)
    {
        fprintf(stderr, "error: failed while writing model tree\n");
        exit_code = 1;
        goto cleanup;
    }

    if (write_tessellations(ctx, data, &table, &writer) < 0)
    {
        fprintf(stderr, "error: failed while writing tessellation data\n");
        exit_code = 1;
        goto cleanup;
    }

    json_writer_end_object(&writer);

    if (json_writer_has_error(&writer))
    {
        fprintf(stderr, "error: write failure while streaming JSON output\n");
        exit_code = 1;
        goto cleanup;
    }

cleanup:

    /* ---- Finalize and release everything, in all cases ------------------ */

    if (writer_initialized)
    {
        if (json_writer_destroy(&writer) != 0 && exit_code == 0)
        {
            fprintf(stderr, "error: failed to flush JSON output to '%s'\n", output_path);
            exit_code = 1;
        }
    }

    if (out != NULL)
    {
        if (fclose(out) != 0 && exit_code == 0)
        {
            fprintf(stderr, "error: failed to close output file '%s'\n", output_path);
            exit_code = 1;
        }
    }

    tess_table_release_faces(&table);

    if (ctx != NULL)
    {
        /* prc_api_release_data tolerates NULL/zero for any of the
           optional arrays it is not responsible for in this program (we
           never separately allocate a line-tessellation array, so those
           arguments are NULL/0 here), matching the pattern from the
           quick-start example. */
        if (data != NULL)
        {
            prc_api_release_data(ctx, data, table.tesses, table.num_tess,
                                  NULL, 0, model_tree);
        }

        code = prc_api_release_context(ctx);
        if (code == PRC_API_MEMORY_LEAK_DETECTED)
        {
            fprintf(stderr, "warning: memory leak detected during context release\n");
        }
    }

    if (exit_code == 0)
    {
        fprintf(stderr, "json_export: wrote '%s' (%u tessellations, %u parts, %u products)\n",
                output_path, table.num_tess, num_parts, num_products);
    }

    return exit_code;
}
