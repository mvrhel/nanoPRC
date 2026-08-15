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

#ifndef PRC_CONTEXT_H
#define PRC_CONTEXT_H

#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

/**
 * @file prc_context.h
 * @brief Context, allocation hooks, and internal error stack primitives.
 */

/** @defgroup context_api Context API
 *  @brief Context lifecycle, allocation wrappers, and error reporting.
 *  @{
 */

#ifndef PRC_DEBUG_MEMORY
#define PRC_DEBUG_MEMORY 0
#endif

#ifndef PRC_DEBUG_MEMORY_SIZE
#define PRC_DEBUG_MEMORY_SIZE 10000000
#endif

typedef struct prc_context_s prc_context;
typedef struct prc_exception prc_exception;
typedef struct prc_nano_brep_compressed_data_s prc_nano_brep_compressed_data;

/**
 * @brief Caller-provided allocation hooks and opaque user payload.
 *
 * If hooks are omitted, default allocator behavior is used.
 */
typedef struct
{
    void *opaque;
    void *(*malloc)(void *, size_t);
    void *(*realloc)(void *, void *, size_t);
    void (*free)(void *, void *);
} prc_hooks;

/**
 * @brief One entry in the context error stack.
 */
struct prc_exception {
    char message[256];
    int code;
    uint32_t repeat_count; /* consecutive additional pushes with an identical
                               code+message that were squelched (not given
                               their own node) rather than pushed again */
    struct prc_exception *next, *prev;
};

typedef struct {
    uint32_t biased_layer_index;
    uint32_t biased_index_of_line_style;
    uint8_t behavior_bit_field1;
    uint8_t behavior_bit_field2;
    void **ref_base_ptr;
    uint32_t ref_base_ptr_capacity;
    uint32_t ref_base_count;
} prc_graphics_content_ctx;

typedef struct {
    uint32_t reader_version;
    void *schema;
    prc_nano_brep_compressed_data *nano_brep_data;
} prc_internal;

#if PRC_DEBUG_MEMORY
typedef struct
{
    size_t index;
    void *data;
    uint8_t is_free;
    void *alloc_callsite;
    void *free_callsite;
} prc_debug_memory;
#endif

struct prc_context_s
{
    void* user;
    prc_hooks hooks;
    prc_exception *exception;
    prc_internal internal;
    prc_graphics_content_ctx graphics_content;
    uint32_t source_file_version;
    uint32_t current_file_index;
    /* PRC_TRACE_REVERSED / PRC_TRACE_NORMALS / PRC_TRACE_GATE: read once at
       context creation instead of via getenv on every triangle/corner (see
       prc_new_context). */
    uint8_t trace_reversed;
    uint8_t trace_normals;
    uint8_t trace_gate;
 #if PRC_DEBUG_MEMORY
    prc_debug_memory *debug_memory;
    size_t debug_memory_size;
    size_t current_memory_index;
    uint8_t debug_memory_table_full_warned;
    size_t debug_memory_untracked_alloc_count;
 #endif
};

/**
 * @brief Push a formatted error entry onto the context error stack.
 *
 * @param ctx Active context.
 * @param code Error code.
 * @param file Source file name.
 * @param line Source line number.
 * @param format printf-style format string.
 * @param args Vararg list.
 */
void prc_vferror(prc_context *ctx, int code, const char *file, int line, const char *format, va_list args);

/**
 * @brief Push a formatted error entry onto the context error stack.
 *
 * @param ctx Active context.
 * @param code Error code.
 * @param file Source file name.
 * @param line Source line number.
 * @param format printf-style format string.
 */
void prc_ferror(prc_context *ctx, int code, const char *file, int line, const char *format, ...);

#define prc_error(ctx, code, format, ...) prc_ferror((ctx), (code), __FILE__, __LINE__, "%s", (format))

/**
 * @brief Create a new low-level PRC context.
 *
 * @param hooks Optional caller-provided allocation hooks.
 * @return New context on success, NULL on failure.
 */
prc_context* prc_new_context(const prc_hooks *hooks);

/**
 * @brief Release a low-level PRC context.
 *
 * @param ctx Context to release.
 */
void prc_release_context(prc_context *ctx);

/**
 * @brief Context-aware malloc wrapper.
 *
 * @param ctx Active context.
 * @param size Number of bytes.
 * @return Allocated block, or NULL on failure.
 */
void *prc_malloc(prc_context *ctx, size_t size);

/**
 * @brief Context-aware calloc wrapper.
 *
 * @param ctx Active context.
 * @param count Element count.
 * @param size Size per element.
 * @return Allocated zeroed block, or NULL on failure.
 */
void *prc_calloc(prc_context *ctx, size_t count, size_t size);

/**
 * @brief Context-aware realloc wrapper.
 *
 * @param ctx Active context.
 * @param p Existing allocation, or NULL.
 * @param size New allocation size.
 * @return Reallocated block, or NULL on failure.
 */
void *prc_realloc(prc_context *ctx, void *p, size_t size);

/**
 * @brief Context-aware free wrapper.
 *
 * @param ctx Active context.
 * @param p Allocation to free.
 */
void prc_free(prc_context *ctx, void *p);

/**
 * @brief Print current context error stack.
 *
 * @param ctx Active context.
 */
void prc_print_error_stack(prc_context *ctx);

#define PRC_ERROR_MEMORY -1
#define PRC_ERROR_PARSE  -2
#define PRC_ERROR_KEY_NOT_FOUND -3
#define PRC_ERROR_HUFFMAN -4
#define PRC_ERROR_IO -5
#define PRC_FILE_VERS -6
#define PRC_TAG_ERROR -7
#define PRC_SCHEMA_ERROR -8
#define PRC_ERROR_NOT_IMPLEMENTED -9
#define PRC_ERROR_EDGE_NOT_FOUND -10
#define PRC_ERROR_FILE -11
#define PRC_ERROR_INTERNAL -12
#define PRC_API_INDEX -13
#define PRC_ERROR_PDF -14
#define PRC_ERROR_PASSWORD -15
#define PRC_API_ERROR -16

/** @} */

#endif
