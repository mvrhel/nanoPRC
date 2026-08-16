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

#include <stdio.h>
#include <string.h>
#include "prc_write_file_structure.h"
#include "prc_write_tess_3d.h"
#include "prc_write_compress_tess.h"
#include "prc_data.h"
#include "zlib.h"

int
prc_write_schema_and_globals_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_global_tables *tables)
{
    if (ctx == NULL || s == NULL || tables == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_schema_and_globals_to_stream: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    /* Table 8 PRC_Schema: no custom entity schema */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* schema_count */

    /* Table 39 PRC_TYPE_ASM_FileStructureGlobals */
    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_ASM_FileStructureGlobals) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* base.attribute_count */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;     /* base.name.same */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* external file_count (cross-file references) */

    if (prc_write_globals_to_stream(ctx, s, tables) != 0) goto fail;

    return 0;

fail:
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}

int
prc_write_tessellation_section_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_tess_entry *entries, uint32_t num_entries)
{
    uint32_t i;
    uint8_t *demote = NULL;
    uint8_t *skip = NULL;
    uint32_t surviving_count = num_entries;

    if (ctx == NULL || s == NULL || (entries == NULL && num_entries > 0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_tessellation_section_to_stream: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    /* FIX (2026-08-08, prc-db 310-file corpus sweep): a COMPRESSED entry
       whose own weld/degenerate-triangle-removal (prc_encode_preprocess)
       leaves zero surviving triangles used to hard-abort the ENTIRE
       tessellation section -- confirmed on 6 real-world multi-part
       assemblies (each split into dozens of connected-part entries by the
       caller, at least one degenerating to nothing after weld). By the time
       prc_write_compress_tess_entry's own error surfaces, this function has
       already committed tess_count (line below, historically) to the
       stream, so a later entry can't be handled without corrupting what's
       already written -- the decision has to happen in a pre-pass, BEFORE
       tess_count is written, not in the main loop. Re-runs
       prc_encode_preprocess a second time for each COMPRESSED entry (the
       real per-entry write below does its own, unavoidable given
       prc_write_compress_tess_entry doesn't accept a pre-built mesh) --
       an acceptable cost for a rare-in-practice case, not a hot path.

       FIX (2026-08-15, HeavyDutyCasterWheel-Partlist-PDF3D): that original
       fix DROPPED the entry and wrote a reduced tess_count, which silently
       DESYNCS every caller-held tessellation index past the dropped one.
       Callers assign prc_api_write_rep_item::biased_tessellation_index
       themselves, up front, positionally (demos/stl_import assigns
       component c -> biased index c+1), and the model tree is serialized
       BEFORE this function runs (prc_write_prc_buffer), so nothing can
       renumber it afterwards. On HeavyDutyCaster (40 components, component
       8 a 3-triangle sliver that quantizes away entirely) that left parts
       9..39 pointing at their neighbour's geometry and part 40 pointing at
       biased index 40 with only 39 entries present -- a dangling reference
       real Acrobat responds to by blanking the whole model tree.

       So: DEMOTE rather than drop. The entry's raw geometry is untouched
       and perfectly writable -- it is only the COMPRESSED encoder's own
       quantization that collapses it -- so write it as an uncompressed
       PRC_TYPE_TESS_3D entry instead, exactly the fallback already used
       for content the COMPRESSED path can't represent (non-manifold fans,
       demos/stl_import). tess_count stays equal to num_entries, so every
       caller-side index remains valid by construction and this whole class
       of desync disappears. Dropping is retained only for an entry that
       carries no writable geometry at ALL (num_triangles/num_faces zero --
       unreachable from any in-tree caller, since a connected component
       always has at least one triangle); such an entry has no valid
       representation in either encoding, and a caller referencing it was
       already inconsistent. */
    if (num_entries > 0)
    {
        demote = (uint8_t *)prc_calloc(ctx, num_entries, sizeof(uint8_t));
        skip = (uint8_t *)prc_calloc(ctx, num_entries, sizeof(uint8_t));
        if (demote == NULL || skip == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_tessellation_section_to_stream\n");
            if (demote != NULL) prc_free(ctx, demote);
            if (skip != NULL) prc_free(ctx, skip);
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < num_entries; i++)
        {
            const prc_write_tess_entry *e = &entries[i];
            prc_write_tolerance tol;
            prc_encode_mesh mesh;
            int code;

            if (e->kind != PRC_WRITE_TESS_KIND_COMPRESSED)
                continue;

            tol = (e->tolerance.value > 0.0) ? e->tolerance : prc_write_tol_relative(1e-6);
            code = prc_encode_preprocess(ctx, e->positions, e->num_positions,
                e->tri_indices, e->num_triangles, tol, &mesh);
            if (code == 0)
            {
                if (mesh.num_triangles == 0)
                {
                    if (e->num_triangles > 0)
                    {
                        printf("Note: a COMPRESSED tessellation entry has zero surviving triangles "
                            "after welding (entry %u of %u) -- writing it uncompressed instead, so "
                            "caller-side tessellation indices stay valid.\n", i, num_entries);
                        demote[i] = 1;
                    }
                    else
                    {
                        printf("Note: tessellation entry %u of %u carries no writable geometry "
                            "-- dropping it. Caller-side tessellation indices past this entry "
                            "shift by one.\n", i, num_entries);
                        skip[i] = 1;
                        surviving_count--;
                    }
                }
                prc_encode_preprocess_free(ctx, &mesh);
            }
            /* A real error here (not just "welded to nothing") is left for
               the real per-entry call below to report/fail on, same as
               before this fix -- this pre-pass never masks a genuine error. */
        }
    }

    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_ASM_FileStructureTessellation) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* base.attribute_count */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;     /* base.name.same */

    if (prc_bitwrite_uint32(ctx, s, surviving_count) != 0) goto fail; /* tess_count */
    for (i = 0; i < num_entries; i++)
    {
        const prc_write_tess_entry *e = &entries[i];

        if (skip != NULL && skip[i])
            continue;

        if (e->kind == PRC_WRITE_TESS_KIND_3D || (demote != NULL && demote[i]))
        {
            int must_calc = e->must_calculate_normals;
            double crease = e->crease_angle_degrees;
            const uint32_t *face_counts = e->face_tri_counts;
            uint32_t face_count = e->num_faces;
            uint32_t one_face = e->num_triangles;
            int code;

            if (demote != NULL && demote[i])
            {
                /* A COMPRESSED entry legitimately carries no face grouping
                   (num_faces == 0 means "whole entry is one face", see
                   prc_api_write_tess_entry's doc comment) and no normal
                   data of its own (COMPRESSED reconstructs normals from
                   geometry). The uncompressed writer requires both, so
                   supply the equivalent single face and ask it to
                   recompute normals -- matching what demos/stl_import
                   already does for its own TRIANGLES fallback entries. */
                if (face_counts == NULL || face_count == 0)
                {
                    face_counts = &one_face;
                    face_count = 1;
                }
                if (e->normals == NULL && e->norm_indices == NULL)
                {
                    must_calc = 1;
                    if (crease <= 0.0) crease = 30.0;
                }
            }

            if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_TESS_3D) != 0) goto fail;
            code = prc_write_tess_3d(ctx, s, e->positions, e->num_positions, e->normals, e->num_normals,
                e->tri_indices, e->norm_indices, e->num_triangles, face_counts, face_count,
                must_calc, crease);
            if (code != 0) goto fail;
        }
        else if (e->kind == PRC_WRITE_TESS_KIND_COMPRESSED)
        {
            /* A zero-valued tolerance/crease-angle (the memset(0) default
               most callers start from) means "use a sensible default"
               rather than a literal zero-mm weld distance or a zero-degree
               crease angle -- see the field doc comments in prc_api.h. */
            prc_write_tolerance tol = (e->tolerance.value > 0.0) ? e->tolerance : prc_write_tol_relative(1e-6);
            double crease = (e->crease_angle_degrees > 0.0) ? e->crease_angle_degrees : 30.0;

            if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_TESS_3D_Compressed) != 0) goto fail;
            if (prc_write_compress_tess_entry(ctx, s, e->positions, e->num_positions,
                    e->normals, e->num_normals, e->tri_indices, e->norm_indices, e->num_triangles,
                    e->face_tri_counts, e->num_faces, tol, crease) != 0)
                goto fail;
        }
        else
        {
            if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_TESS_3D_Wire) != 0) goto fail;
            if (prc_write_wire_tess(ctx, s, e->wire_elements, e->num_wire_elements) != 0) goto fail;
        }
    }

    if (skip != NULL)
        prc_free(ctx, skip);
    if (demote != NULL)
        prc_free(ctx, demote);
    return 0;

fail:
    if (skip != NULL)
        prc_free(ctx, skip);
    if (demote != NULL)
        prc_free(ctx, demote);
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}

int
prc_write_geometry_section_to_stream(prc_context *ctx, prc_bit_write_state *s)
{
    if (ctx == NULL || s == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_geometry_section_to_stream: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_ASM_FileStructureGeometry) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* base.attribute_count */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;     /* base.name.same */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* exact_geometry.topo_context_count */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* user_data.stream_size */

    return 0;

fail:
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}

/* PRC_TYPE_ASM_FileStructureExtraGeometry (Table 51): a real, independently-
   produced, tessellation-only compressed PRC file (used as a working
   reference specifically because it opens correctly in Acrobat) has SIX
   sections in its file structure's offset table (file-struct-header +
   five content sections) -- this write facility matches that: the six-entry
   offset table (see PRC_WRITE_PRC_FILE_SECTION_COUNT) covers file-struct-
   header + schema/globals + tree + tessellation + geometry + ExtraGeometry,
   this function providing the last of those, called from
   prc_write_prc_buffer alongside prc_write_geometry_section_to_stream.
   Field order/content mirrors prc_parse_file_extra_geometry exactly
   (prc_parse_file_structure.c): tag, ContentPRCBase (attribute_count,
   name), then extra_geom_count -- that parser does NOT consume a trailing
   user_data stream despite the struct having a user_data field, so this
   writer doesn't emit one either. */
int
prc_write_extra_geometry_section_to_stream(prc_context *ctx, prc_bit_write_state *s)
{
    if (ctx == NULL || s == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_extra_geometry_section_to_stream: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    if (prc_bitwrite_uint32(ctx, s, PRC_TYPE_ASM_FileStructureExtraGeometry) != 0) goto fail;
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* base.attribute_count */
    if (prc_bitwrite_bit(ctx, s, 1) != 0) goto fail;     /* base.name.same */
    if (prc_bitwrite_uint32(ctx, s, 0) != 0) goto fail; /* extra_geom_count */

    return 0;

fail:
    return s->error ? PRC_ERROR_MEMORY : PRC_ERROR_INTERNAL;
}

int
prc_write_deflate(prc_context *ctx, const uint8_t *src, size_t src_len, uint8_t **out, size_t *out_len)
{
    z_stream strm;
    uLong bound;
    uint8_t *dst;
    int ret;

    if (ctx == NULL || out == NULL || out_len == NULL || (src == NULL && src_len > 0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_deflate: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }

    memset(&strm, 0, sizeof(strm));
    if (deflateInit(&strm, Z_DEFAULT_COMPRESSION) != Z_OK)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_deflate: deflateInit failed\n");
        return PRC_ERROR_INTERNAL;
    }

    bound = deflateBound(&strm, (uLong)src_len);
    dst = (uint8_t *)prc_malloc(ctx, bound == 0 ? 1 : bound);
    if (dst == NULL)
    {
        deflateEnd(&strm);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_deflate\n");
        return PRC_ERROR_MEMORY;
    }

    strm.next_in = (Bytef *)src;
    strm.avail_in = (uInt)src_len;
    strm.next_out = (Bytef *)dst;
    strm.avail_out = (uInt)bound;

    ret = deflate(&strm, Z_FINISH);
    if (ret != Z_STREAM_END)
    {
        deflateEnd(&strm);
        prc_free(ctx, dst);
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_deflate: deflate failed\n");
        return PRC_ERROR_INTERNAL;
    }

    *out_len = strm.total_out;
    deflateEnd(&strm);
    *out = dst;
    return 0;
}

void
prc_write_file_struct_header_bytes(uint8_t *out, uint32_t min_vers_for_read, uint32_t auth_vers)
{
    uint8_t *p = out;

    p[0] = 'P'; p[1] = 'R'; p[2] = 'C';
    p += PRC_WRITE_SIGNATURE_BYTES;
    p = prc_write_le_uint32(p, min_vers_for_read);
    p = prc_write_le_uint32(p, auth_vers);
    /* Must equal the main header's file_info[0].unique_id and the model
       section's far reference to this file structure -- see
       PRC_WRITE_FILE_STRUCT_UID0's doc comment in prc_write_common.h. */
    p = prc_write_le_unique_id(p, PRC_WRITE_FILE_STRUCT_UID0);
    p = prc_write_le_unique_id(p, PRC_WRITE_APP_UID0);
    p = prc_write_le_uint32(p, 0); /* file_count (embedded uncompressed files) */
}
