# Copyright (C) 2023-2026 CascadiaVoxel LLC
#
#    nanoPRC is free software: you can redistribute it and/or modify it under
#    the terms of the GNU Affero General Public License as published by the
#    Free Software Foundation, either version 3 of the License, or (at your
#    option) any later version.
#
#    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
#    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
#    License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.
#
#    A commercial license is also available -- see README.md.
"""Bit-level splicer for a PRC file's TESSELLATION section.

Replaces an exact BIT RANGE of one PRC's tessellation section with the same
range from another PRC, recompresses the section, and fixes up the
section-offset table. Built (2026-08-17) to answer a question that two earlier
Acrobat blank-tree investigations reached and stopped at: once our encoder's
output has been forced to agree with an independent encoder's on every
inspectable field, only a handful of quantized values still differ -- do those
values actually matter, or is the defect somewhere else in the stream?

Splicing answers that directly: inject just those bits into a known-good file
and test the hybrid. (Result on the file it was written for: the differing
point_array values were NOT causal.)

USAGE AND LIMITS -- read before trusting a result:

  * Both inputs must have BIT-ALIGNED, ideally equal-length tessellation
    sections. Verify with PRC_DIAG_POINT_ARRAY_BITPOS on both files first: the
    per-entry bit positions must line up. Splicing misaligned streams produces
    garbage that merely looks like a test.

  * ALWAYS check the hybrid still decodes (e.g. nano_prc_quick_start) before
    drawing any conclusion from a reader's verdict on it. A file that does not
    decode cannot distinguish "this field is the defect" from "the splice broke
    the stream". In practice mid-stream single-field swaps frequently desync,
    while nested SUFFIX splices (end_bit = -1) usually survive -- prefer those.

  * Only the single-filestructure, single-top-level-file case is handled (what
    demos/stl_import and these repro files produce). Mirrors prc_parse_main.c's
    header layout: prc_parse_filestruct_header / prc_parse_main_header and
    prc_open_contents' section-walk loop.

Sections are stored back-to-back with no explicit length, each one a
self-terminating zlib stream, so a section's real compressed length is however
many bytes its zlib stream consumes.

usage: bitsplice_tess.py <base.prc> <donor.prc> <start_bit> <end_bit> <out.prc>
       end_bit is exclusive; pass -1 for "to the end of the section"
"""
import struct
import sys
import zlib

# section_offsets[] indices 2.. carry these section types in this fixed order
# (304 TREE, 305 TESSELLATION, 306 GEOMETRY, 307 EXTRA_GEOMETRY). Using the
# known order rather than decompressing to read each leading type tag sidesteps
# GEOMETRY/EXTRA_GEOMETRY being too short (3 uncompressed bytes) to read a
# 4-byte tag from.
FIXED_SECTION_ORDER = ['TREE', 'TESSELLATION', 'GEOMETRY', 'EXTRA_GEOMETRY']


def _u32(buf, off):
    return struct.unpack_from('<I', buf, off)[0], off + 4


def parse_header(buf):
    if buf[0:3] != b'PRC':
        raise ValueError('not a PRC stream (missing "PRC" magic)')
    off = 3
    _min_vers, off = _u32(buf, off)
    _auth_vers, off = _u32(buf, off)
    off += 16 + 16                      # uid_file + uid_app
    fs_count, off = _u32(buf, off)
    if fs_count != 1:
        raise ValueError('only single-filestructure files are supported (got %d)' % fs_count)
    off += 16 + 4                       # fs_uid + the 4-byte pad prc_parse_main.c also skips
    section_count, off = _u32(buf, off)
    section_offsets = []
    for _ in range(section_count):
        v, off = _u32(buf, off)
        section_offsets.append(v)
    start_offset, off = _u32(buf, off)
    end_offset, off = _u32(buf, off)
    top_file_count, off = _u32(buf, off)
    if top_file_count != 0:
        raise ValueError('top-level embedded-file case is not supported')
    return {'section_count': section_count, 'section_offsets': section_offsets,
            'start_offset': start_offset, 'end_offset': end_offset}


def section_offsets_table_byte_offset():
    """Byte offset of the section_offsets[] array, so it can be patched in
    place when a spliced section changes length."""
    return 3 + 4 + 4 + 16 + 16 + 4 + 16 + 4 + 4


def compressed_section_length(buf, byte_offset):
    d = zlib.decompressobj()
    chunk = buf[byte_offset:]
    d.decompress(chunk)
    return len(chunk) - len(d.unused_data)


def locate_tessellation(buf, hdr):
    idx = 2 + FIXED_SECTION_ORDER.index('TESSELLATION')
    if idx >= hdr['section_count']:
        raise ValueError('file has no TESSELLATION section')
    start = hdr['section_offsets'][idx]
    return start, compressed_section_length(buf, start)


def _tess_bytes(path):
    buf = open(path, 'rb').read()
    hdr = parse_header(buf)
    start, length = locate_tessellation(buf, hdr)
    return buf, hdr, start, length, zlib.decompressobj().decompress(buf[start:start + length])


def splice_bits(a, b, lo, hi):
    """Copy of bytes `a` with bits [lo,hi) replaced by those of `b`."""
    out = bytearray(a)
    for bit in range(lo, hi):
        byte, shift = bit >> 3, 7 - (bit & 7)
        if (b[byte] >> shift) & 1:
            out[byte] |= (1 << shift)
        else:
            out[byte] &= ~(1 << shift) & 0xFF
    return bytes(out)


def main(argv):
    if len(argv) != 6:
        print(__doc__)
        return 2
    base_path, donor_path, lo, hi, out_path = argv[1:]
    lo, hi = int(lo), int(hi)

    base, hdr, start, length, base_tess = _tess_bytes(base_path)
    _, _, _, _, donor_tess = _tess_bytes(donor_path)

    if len(base_tess) != len(donor_tess):
        print('WARNING: tessellation sections differ in length (%d vs %d bytes) -- '
              'the splice is very likely incoherent; verify the output decodes'
              % (len(base_tess), len(donor_tess)))
    if hi < 0:
        hi = min(len(base_tess), len(donor_tess)) * 8

    comp = zlib.compress(splice_bits(base_tess, donor_tess, lo, hi), 9)
    new = bytearray(base[:start] + comp + base[start + length:])

    delta = len(comp) - length
    if delta:
        tbl = section_offsets_table_byte_offset()
        offs = hdr['section_offsets']
        for i, v in enumerate(offs):
            if v > start:
                new[tbl + 4 * i: tbl + 4 * i + 4] = (v + delta).to_bytes(4, 'little')
        so = tbl + 4 * len(offs)        # start_offset/end_offset follow the table
        for k in range(2):
            v = int.from_bytes(new[so + 4 * k: so + 4 * k + 4], 'little')
            if v > start:
                new[so + 4 * k: so + 4 * k + 4] = (v + delta).to_bytes(4, 'little')

    open(out_path, 'wb').write(bytes(new))
    print('wrote %s (%d bytes; tessellation section %d -> %d bytes, bits [%d,%d) from donor)'
          % (out_path, len(new), length, len(comp), lo, hi))
    print('NOW VERIFY the output decodes (e.g. nano_prc_quick_start) before trusting any verdict.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
