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

/* Pins the ownership contract of prc_release_compressed_curve.
 *
 * WHAT WENT WRONG, AND WHY A TEST IS NEEDED HERE
 * ----------------------------------------------
 * A prc_ref_or_compressed_curve's compressed_curve pointer has three different
 * owners depending on where the record came from:
 *
 *   embedded      prc_topo_single_wire_compress points at its own member
 *   shared table  prc_parse_ref_or_compressed_curve hands out
 *                 &prc_nano_brep_compressed_data.curves[current_curve_index++]
 *   independent   prc_parse_ana_face_trim_loop allocates each one outright
 *
 * prc_release_compressed_curve is contents-only for all three: it frees what a
 * curve itself owns and never the struct. Callers decide about the struct.
 *
 * It used to violate that for composite curves, descending into each
 * sub-curve's compressed_curve and freeing its contents too. Those sub-curves
 * come from the shared table, whose own release pass frees them again -- a
 * double free that has never fired only because no file in any corpus we hold
 * contains a composite curve.
 *
 * HOW THIS TEST CATCHES IT
 * ------------------------
 * Not by leak counting: the leak gate needs PRC_ENABLE_DEBUG_MEMORY, which is
 * off by default and not run in CI, and a regression test that silently passes
 * in the default configuration is not a regression test.
 *
 * Instead the context is given allocation hooks that COUNT frees, and the test
 * asserts how many happen at each step. Counting rather than matching pointers
 * is deliberate: prc_malloc always prepends a guard block (PRC_MEMORY_GUARDS in
 * prc_memory.c is unconditional), so the pointer the free hook receives is the
 * block base and not the pointer the caller holds. A count is exact, needs no
 * knowledge of that layout, and does not break if the layout changes.
 *
 * On the old code, releasing the composite frees three blocks instead of one --
 * its record array plus the sub-curve's points and tangents -- and the first
 * assertion below fails. If the sub-curve is then released, as its real owner
 * would, the guard check in prc_free traps on the second free instead. Either
 * way the test does not pass.
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "prc_test.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_data.h"

/* prc_release_compressed_curve is internal but not static; prc_release.c and
   prc_parse_extra_geometry.c both declare it locally the same way. */
void prc_release_compressed_curve(prc_context *ctx, prc_compressed_curve *data);

typedef struct
{
    int free_count;
} alloc_log;

static void *
log_malloc(void *opaque, size_t size)
{
    (void)opaque;
    return malloc(size);
}

static void *
log_realloc(void *opaque, void *ptr, size_t size)
{
    (void)opaque;
    return realloc(ptr, size);
}

static void
log_free(void *opaque, void *ptr)
{
    alloc_log *log = (alloc_log *)opaque;

    if (ptr != NULL)
        log->free_count++;
    free(ptr);
}

int
main(void)
{
    alloc_log log;
    prc_hooks hooks;
    prc_context *ctx;
    int baseline;

    /* The shared curve table, as prc_parse_ref_or_compressed_curve would build
       it: entry 0 is the composite, entries 1 and 2 are its sub-curves and are
       elements of this array rather than separate allocations. */
    prc_compressed_curve table[3];
    prc_ref_or_compressed_curve *records;
    prc_vec3 *points;
    prc_vec3 *tangents;

    PRC_TEST_BEGIN("compressed curve release ownership");

    memset(&log, 0, sizeof(log));
    hooks.opaque = &log;
    hooks.malloc = log_malloc;
    hooks.realloc = log_realloc;
    hooks.free = log_free;

    ctx = prc_api_new_context(&hooks);
    PRC_ASSERT(ctx != NULL);

    memset(table, 0, sizeof(table));

    /* Everything the release path will free must come from prc_malloc, not
       from the hook directly: prc_free expects its guard header to be there. */

    /* Entry 1: a Hermite sub-curve owning two allocations. These are what the
       old code freed a second time. */
    points = (prc_vec3 *)prc_malloc(ctx, 3 * sizeof(prc_vec3));
    tangents = (prc_vec3 *)prc_malloc(ctx, 3 * sizeof(prc_vec3));
    PRC_ASSERT(points != NULL && tangents != NULL);
    table[1].curve_type = PRC_HCG_BsplineHermiteCurve;
    table[1].hcg_bspline_hermite_curve.number_points = 3;
    table[1].hcg_bspline_hermite_curve.points = points;
    table[1].hcg_bspline_hermite_curve.tangents = tangents;

    /* Entry 2: a Line sub-curve, owning nothing. */
    table[2].curve_type = PRC_HCG_Line;

    /* Entry 0: the composite. It owns the record array and nothing else; each
       record points INTO the table above. */
    records = (prc_ref_or_compressed_curve *)prc_malloc(ctx,
        2 * sizeof(prc_ref_or_compressed_curve));
    PRC_ASSERT(records != NULL);
    memset(records, 0, 2 * sizeof(prc_ref_or_compressed_curve));
    records[0].curve_is_not_already_stored = 1;
    records[0].compressed_curve = &table[1];
    records[1].curve_is_not_already_stored = 1;
    records[1].compressed_curve = &table[2];

    table[0].curve_type = PRC_HCG_CompositeCurve;
    table[0].hcg_composite_curve.type = PRC_HCG_CompositeCurve;
    table[0].hcg_composite_curve.number_of_curves = 2;
    table[0].hcg_composite_curve.curves = records;

    /* Creating the context may itself have allocated and freed; count from
       here rather than from zero. */
    baseline = log.free_count;

    /* The composite owns its record array and nothing else: exactly one free. */
    prc_release_compressed_curve(ctx, &table[0]);
    PRC_ASSERT_EQ(log.free_count - baseline, 1);

    /* Now the shared table's own release pass, which is the sole owner of the
       sub-curves' contents. After the fix this is the first and only time they
       are freed; before it, it was the second. Two more frees, for points and
       tangents; the Line owns nothing and adds none. */
    prc_release_compressed_curve(ctx, &table[1]);
    PRC_ASSERT_EQ(log.free_count - baseline, 3);

    prc_release_compressed_curve(ctx, &table[2]);
    PRC_ASSERT_EQ(log.free_count - baseline, 3);

    /* A composite with no sub-curves and a NULL record array: reachable from a
       conforming file, since number_of_curves is read before the array is
       allocated. Must be inert rather than crashing. */
    {
        prc_compressed_curve empty;

        memset(&empty, 0, sizeof(empty));
        empty.curve_type = PRC_HCG_CompositeCurve;
        empty.hcg_composite_curve.number_of_curves = 0;
        empty.hcg_composite_curve.curves = NULL;
        prc_release_compressed_curve(ctx, &empty);
        PRC_ASSERT_EQ(log.free_count - baseline, 3);
    }

    /* And a NULL curve, which the release walkers pass freely. */
    prc_release_compressed_curve(ctx, NULL);
    PRC_ASSERT_EQ(log.free_count - baseline, 3);

    prc_api_release_context(ctx);

    PRC_TEST_END;
}
