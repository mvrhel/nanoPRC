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
#include <prc_context.h>
#include "prc_diag_env.h"

extern prc_hooks prc_hooks_default;

#define INIT_REFBASE_CAPACITY 256

prc_context*
prc_new_context(const prc_hooks *hooks)
{
    prc_context *ctx;

    if (hooks == NULL)
        hooks = &prc_hooks_default;

    ctx = hooks->malloc(hooks->opaque, sizeof(prc_context));
    if (!ctx)
        return NULL;

    memset(ctx, 0, sizeof(prc_context));
    ctx->hooks = *hooks;

    ctx->internal.reader_version = 10001; /* PRC ISO Standard Version Number */

    ctx->trace_reversed = (prc_diag_getenv("PRC_TRACE_REVERSED") != NULL);
    ctx->trace_normals = (prc_diag_getenv("PRC_TRACE_NORMALS") != NULL);
    ctx->trace_gate = (prc_diag_getenv("PRC_TRACE_GATE") != NULL);

#if PRC_DEBUG_MEMORY
    ctx->debug_memory = calloc(PRC_DEBUG_MEMORY_SIZE, sizeof(prc_debug_memory));
    if (!ctx->debug_memory)
    {
        hooks->free(hooks->opaque, ctx);
        return NULL;
    }
    ctx->debug_memory_size = PRC_DEBUG_MEMORY_SIZE;
    ctx->current_memory_index = 0;
    ctx->debug_memory_table_full_warned = 0;
    ctx->debug_memory_untracked_alloc_count = 0;
#endif

    ctx->graphics_content.behavior_bit_field1 = 0;
    ctx->graphics_content.behavior_bit_field2 = 0;
    ctx->graphics_content.biased_index_of_line_style = 0;
    ctx->graphics_content.biased_layer_index = 0;

    ctx->graphics_content.ref_base_ptr = prc_calloc(ctx, INIT_REFBASE_CAPACITY, sizeof(void*));
    if (!ctx->graphics_content.ref_base_ptr)
    {
        prc_release_context(ctx);
        return NULL;
    }
    ctx->graphics_content.ref_base_ptr_capacity = INIT_REFBASE_CAPACITY;
    ctx->graphics_content.ref_base_count = 0;

    return ctx;
}

void
prc_release_context(prc_context *ctx)
{
    if (ctx != NULL)
    {
        if (ctx->exception != NULL)
        {
            prc_exception *curr = ctx->exception;
            prc_exception *next;
            while (curr != NULL)
            {
                next = curr->prev;
                prc_free(ctx, curr);
                curr = next;
            }
        }

        if (ctx->graphics_content.ref_base_ptr != NULL)
        {
            prc_free(ctx, ctx->graphics_content.ref_base_ptr);
        }

#if !PRC_DEBUG_MEMORY
        ctx->hooks.free(ctx->hooks.opaque, ctx);
#endif
    }
}
