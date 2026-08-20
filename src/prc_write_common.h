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

#ifndef PRC_WRITE_COMMON_H
#define PRC_WRITE_COMMON_H

#include <stdint.h>

/* Internal shared declarations for the write facility (encoder). The public
   write-side types (prc_write_tolerance, prc_write_tol_resolve) live in
   prc_api.h; this header exists for internals shared between the encoder's
   translation units. */

#include "../include/prc_api.h"
#include "prc_data.h"
#include "prc_bit.h"

/* Byte sizes of the raw (non-bit-packed) little-endian fields used by the
   two file-level headers this write facility produces manually --
   prc_write_file_structure.c's file-structure header and
   prc_write_model.c's main prc_header. Shared here so both files' byte-
   layout math (and each file's own PRC_WRITE_*_SIZE constant/layout
   function) is built from the same named units instead of separately
   re-deriving "16" or "4" in each place. */
#define PRC_WRITE_U32_BYTES 4u
#define PRC_WRITE_UNIQUE_ID_BYTES 16u /* 4x uint32 */
#define PRC_WRITE_SIGNATURE_BYTES 3u  /* "PRC" */

/* Writes `v` as 4 little-endian bytes at `p`, returning p + 4. The one
   raw-byte (non-bit-packed) primitive shared by prc_write_file_structure.c
   and prc_write_model.c, exactly mirroring prc_read_32bits_unsigned's
   little-endian convention in prc_parse_main.c. */
uint8_t *prc_write_le_uint32(uint8_t *p, uint32_t v);

/* Fixed, non-zero placeholder unique IDs (Table 5/35's 4x uint32 "unique
   id" fields, used both raw in the two file-level headers and via
   prc_bitwrite_uint32 in the model section's far reference). All-zero IDs
   were confirmed the hard way to be rejected as a null/invalid sentinel by
   at least one external reader ("Empty Scene Detected" persisted after
   every graphics-visibility and reference-target fix, and cleared once
   these were made non-zero) -- these do not need to be cryptographically
   random for a single-file writer to work correctly, only non-zero and
   mutually consistent everywhere the format requires equality.

   PRC_WRITE_FILE_UID0 is independent of PRC_WRITE_FILE_STRUCT_UID0 --
   revised back from a brief attempt at making them equal, which was based
   on a single reference file (not examples/cube.pdf) whose main header's
   unique_id_file happened to equal its file structure's own identity. A
   THIRD, independently-produced real file (a different generator again)
   directly contradicts that: its main header's unique_id_file and its
   file structure's own embedded unique_id_file are different values.
   Between three real producers agreeing on nothing about this field's
   relationship to the file structure's identity beyond "non-zero", it
   looks like generator-specific noise rather than a real requirement, so
   this write facility goes back to treating it as independent. PRC_WRITE_
   APP_UID0 (authoring application identity) is likewise independent of
   both in every real file checked so far. A future revision could
   generate these per call instead of reusing fixed values. */
#define PRC_WRITE_FILE_UID0 1u
#define PRC_WRITE_APP_UID0 2u
#define PRC_WRITE_FILE_STRUCT_UID0 3u

/* Writes a 4x uint32 unique id {word0, 0, 0, 0} as raw little-endian bytes
   at `p`, returning p + 16 (PRC_WRITE_UNIQUE_ID_BYTES). */
uint8_t *prc_write_le_unique_id(uint8_t *p, uint32_t word0);

/* min_vers_for_read / auth_vers, written in both the main prc_header and
   every PRC_TYPE_ASM_FileStructure header (Tables 5/37). Confirmed the
   hard way: 0 does NOT mean "any reader accepts this" the way an earlier
   assumption in this codebase held -- the spec documents at least some
   entity fields as present only conditionally on authoring_version
   crossing a threshold, "the sole mechanism for detecting its presence;
   there is no secondary flag" (e.g. PRC_TYPE_GRAPH_DirectionalLight's
   intensity, gated on >= 8030).

   The ISO 14739 draft is explicit that 10001 is the ONLY compliant PRC
   version ("This international standard specifies the PRC version 10001.
   Documents with other PRC versions are non-compliant" -- version numbers
   are year-modulo-2000 + day-of-year, so 10001 = 2010 day 1, this
   document's own publication version) -- but a real, deployed reader
   (Adobe Acrobat) REJECTS files declaring it outright: "This 3D model
   requires a more recent version of Acrobat", blank 3D scene, empty model
   tree, even on a fully updated Acrobat install. This matches the spec's
   own described reader behavior to the letter ("if [the reader's]
   current_version is less than the file's minimal_version_for_read, the
   reader shall not continue to process the file and report an error") --
   Acrobat's PRC engine has apparently never been updated to recognize
   version 10001 as valid, so declaring the literal spec-compliant value
   makes the single most important real-world reader refuse the file
   entirely.

   7094/23306 (examples/cube.pdf's values, confirmed to open successfully
   in real Acrobat) were used for a long time, but that pairing was flagged
   (2026-07-27) as its own correctness problem independent of the above:
   auth_vers=23306 asserts full compliance with the PRC spec as amended
   through late 2023, while grepping every source_file_version-gated code
   path in this codebase (prc_parse_tree.c, the only file with any) shows
   the highest version this parser actually knows how to handle differently
   is 8016 -- everything the spec added between 2008 and 2023 is something
   this codebase has never read a word of, let alone implemented, yet every
   nanoPRC-written file was unconditionally claiming full knowledge of it.
   (One such gated field, PRC_TYPE_GRAPH_DirectionalLight's intensity at
   >=8030, WAS read unconditionally by this codebase regardless of declared
   version. Fixed 2026-08-20 -- prc_parse_tree.c now gates it on
   source_file_version and defaults to 1.0 below 8030, per the spec's own
   "shall not attempt to read intensity from the bitstream". That was a read
   -side desync on third-party files, unrelated to what this write facility
   declares; nanoPRC's own writer emits no light nodes.)

   8016/8016 was used from that point (2026-07-27) on the reasoning above:
   the highest version this codebase could then justify. **Superseded
   2026-08-20 by 8137/8137**, for four independent reasons, the first of
   which makes 8016 not merely conservative but wrong:

   1. THIS WRITE FACILITY ALREADY EMITS 8137-ERA CONTENT. prc_write_tree.c
      writes PRC_TYPE_MISC_GeneralTransformation (type tag + 16 doubles) for
      any node with a non-identity placement transform. ISO/CD 14739-1
      SS8.4.2 states that entity "was defined in version 8137 of the PRC
      file Format Specification", and its Table 342 schema is exactly
      Block_Version 8137 + SimpleFor 16 + Data_Double -- matching our
      emission field for field. Declaring 8016 with min == auth means no
      schema is written (the spec's "writer does not have to write any extra
      data" case), so a reader conforming to exactly 8016 would meet an
      entity type that did not exist at its version with nothing in the file
      telling it how to skip. That is the precise failure the versioning
      mechanism exists to prevent.
   2. 8137 IS THE HIGHEST VERSION THE SPECIFICATION ITSELF DEFINES. The only
      version-shaped numbers anywhere in the draft are 8030 (the
      DirectionalLight intensity gate), 8137 (SS8.4.2), an illustrative 9149
      inside a schema example, and 10001 -- which SS4.2 identifies as this
      document's own publication version ("the first day of the year 2010"),
      not a feature-set marker. Nothing in the document gates anything above
      8137, so declaring 8137 claims conformance to the document as written.
   3. ACROBAT'S PRC ENGINE current_version IS EXACTLY 8137. Measured
      2026-08-18 by bisection on a known-good file: 8137 accepted, 8138
      refused with "This 3D format is supported by a more recent release of
      Adobe Acrobat", on a fully-updated 2026 install. 8137 decodes as
      16 May 2008 -- the engine is ~18 years frozen.
   4. IT IS THE ECOSYSTEM CONVENTION. PDF3DReportGen and an independent
      OpenSCAD-based serializer both emit 8137/8137, as do 213 of the 310
      files in the prc-db real-world corpus (69%).

   min == auth, so no schema is required. Note what this does NOT claim to
   fix: declared version was directly tested against the Acrobat blank-model
   -tree family (2026-07-27, three real failing files at 8137/8137) and made
   no difference, and files that reproduce that defect are now known to do so
   while declaring 8137 themselves. This is a correctness fix for (1) and an
   alignment for (2)-(4), not a blank-tree fix.

   Re-validate against the full previously-Acrobat-confirmed file set before
   treating this as safely shipped -- it changes the header of every written
   file, same as any bitstream change. */
#define PRC_WRITE_MIN_VERS_FOR_READ 8137u
#define PRC_WRITE_AUTH_VERS 8137u

/* Writes ContentPRCBase/ContentPRCRefBase's `name` field (Table 31): bit
   same=1 with no following string if `name` is NULL (matching this write
   facility's long-standing "no name" default), else same=0 followed by
   `name`'s bytes. Shared by every translation unit that emits a name --
   parts, product occurrences, and the model file -- since it's the one
   piece of write-side name support to add rather than duplicate; see
   prc_api_write_node's name/part_name fields and prc_api_write_prc_file's
   model_name parameter in include/prc_api.h for the public entry points
   that supply it. */
int prc_write_name(prc_context *ctx, prc_bit_write_state *s, const char *name);

#endif
