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

#ifndef PRC_WRITE_FILE_STRUCTURE_H
#define PRC_WRITE_FILE_STRUCTURE_H

#include <stddef.h>
#include "prc_write_common.h"
#include "prc_data.h"
#include "prc_bit.h"
#include "prc_write_global.h"
#include "prc_write_wire_tess.h"

/* prc_write_tess_kind and prc_write_tess_entry are aliases of the public
   prc_api_write_tess_kind_t / prc_api_write_tessellation (include/prc_api.h)
   -- see those types' doc comments for field semantics. Aliased rather
   than redefined so the internal encoder and the public API can never
   drift apart. Bridges to the existing per-format encoders
   (prc_write_tess_3d.c / prc_write_wire_tess.c) -- exactly one of the two
   field groups is read, selected by `kind`. */
typedef prc_api_write_tess_kind_t prc_write_tess_kind;
#define PRC_WRITE_TESS_KIND_3D PRC_API_WRITE_TESS_KIND_TRIANGLES
#define PRC_WRITE_TESS_KIND_WIRE PRC_API_WRITE_TESS_KIND_WIRE
#define PRC_WRITE_TESS_KIND_COMPRESSED PRC_API_WRITE_TESS_KIND_COMPRESSED

typedef prc_api_write_tessellation prc_write_tess_entry;

/* Schema (Table 8, always empty -- no custom entity schema in this write
   facility) + FileStructureGlobals (Table 39) combined section content.
   prc_parse_file_schema_and_global treats these as one deflated section
   with no leading type tag on the schema half ("the schema is the first
   one and does NOT have a type"). */
int prc_write_schema_and_globals_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_global_tables *tables);

/* PRC_TYPE_ASM_FileStructureTessellation (Table 48) section content.

   entry_styles is num_entries biased style indices, one per entry, naming the
   style each entry's faces should carry, or NULL to leave every face styled by
   its owner as before. See prc_write_resolve_face_styles. */
int prc_write_tessellation_section_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_tess_entry *entries, uint32_t num_entries,
    const uint32_t *entry_styles);

/* PRC_TYPE_ASM_FileStructureGeometry (Table 49) section content, always
   empty (topo_context_count = 0 -- exact B-Rep geometry is out of scope
   for this write facility). Some third-party PRC readers assume this
   section is always present in the file-structure's section table (Table
   6's fixed section order), even when it has no content, rather than
   treating it as fully optional the way nanoPRC's own type-tag-dispatch
   reader does -- write it unconditionally so those readers don't
   misinterpret a later (present) section's bytes as the missing one. */
int prc_write_geometry_section_to_stream(prc_context *ctx, prc_bit_write_state *s);

/* PRC_TYPE_ASM_FileStructureExtraGeometry (Table 51): written unconditionally
   empty for the same reason as prc_write_geometry_section_to_stream above --
   Table 6's fixed section set includes this as a distinct section from
   FileStructureGeometry, and a real, independently-produced, Acrobat-
   working reference file always has six sections (file-struct-header +
   five content sections) where this write facility previously only ever
   produced five, omitting this one entirely. */
int prc_write_extra_geometry_section_to_stream(prc_context *ctx, prc_bit_write_state *s);

/* Deflate-compresses `src` (zlib, default level, single deflateInit/deflate
   .../Z_FINISH/deflateEnd call sized via deflateBound) into a caller-owned
   (prc_free) buffer. Returns 0 on success. */
int prc_write_deflate(prc_context *ctx, const uint8_t *src, size_t src_len,
    uint8_t **out, size_t *out_len);

/* Uncompressed prc_file_structure_header (Table 37, the content at a file
   structure's section_offset[0]): "PRC" + min_vers_for_read + auth_vers +
   2 unique ids + file_count, then one block per embedded file.

   PRC_WRITE_FILE_STRUCT_HEADER_SIZE is the size with NO embedded files, and
   is what prc_write_file_struct_header_bytes writes. The macro is built from
   the same PRC_WRITE_*_BYTES unit constants that function writes (in
   prc_write_common.h) rather than a separately hand-counted number, so the
   two cannot silently drift apart.

   file_count is the embedded-uncompressed-file table, which is where raster
   images live -- prc_parse_main.c says so at the read site: "This is where
   the texture images are stored! Spec is wrong on the type of these". Each
   block is a uint32 byte count followed by that many raw bytes, with no type
   tag; what the bytes are is only known once the referring
   PRC_TYPE_GRAPH_Picture is read. */
#define PRC_WRITE_FILE_STRUCT_HEADER_SIZE \
    (PRC_WRITE_SIGNATURE_BYTES + 2u * PRC_WRITE_U32_BYTES + 2u * PRC_WRITE_UNIQUE_ID_BYTES + PRC_WRITE_U32_BYTES)
void prc_write_file_struct_header_bytes(uint8_t *out, uint32_t min_vers_for_read, uint32_t auth_vers);

/* Total header size once `num_files` blocks are appended. Equals
   PRC_WRITE_FILE_STRUCT_HEADER_SIZE exactly when num_files is 0, so a file
   with no embedded images is byte-identical to one written before this
   existed. Returns 0 if the total would overflow. */
size_t prc_write_file_struct_header_size_ex(const prc_write_embedded_file *files,
    uint32_t num_files);

/* As prc_write_file_struct_header_bytes, then the blocks. `out` must have
   room for prc_write_file_struct_header_size_ex bytes. */
void prc_write_file_struct_header_bytes_ex(uint8_t *out, uint32_t min_vers_for_read,
    uint32_t auth_vers, const prc_write_embedded_file *files, uint32_t num_files);

#endif
