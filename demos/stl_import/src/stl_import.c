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

/* ====================================================================
 * nanoPRC STL importer: read an STL mesh (binary or ASCII) and write it
 * out as a .prc file and a minimal 3D-annotated .pdf, using only the
 * public write API declared in include/prc_api.h -- the same one
 * demos/teapot_write uses, and worth reading first if you haven't seen it;
 * this demo assumes familiarity with its "three-level shape of the write
 * API" explanation (tessellation -> representation item -> tree node)
 * rather than repeating it here.
 *
 * Usage:
 *     nano_prc_stl_import <input.stl> [output.prc] [output.pdf]
 *                          [--original-normals] [--weld-tolerance F]
 * Output paths default to the input file's own basename with .stl swapped
 * for .prc/.pdf if omitted. --original-normals and --weld-tolerance are
 * documented where they're used below (stl_options_parse) and in the
 * per-part tessellation build step (stl_import_build_tess_entries).
 *
 * THE STL FORMAT
 * ---------------------------------------------------------------------
 * STL ("STereoLithography", sometimes backronymed "Standard Triangle
 * Language") originated as the native format of 3D Systems' first
 * stereolithography CAD software in 1987. Unlike PRC/PDF (ISO 14739 /
 * ISO 32000) it was never formally standardized by ISO or any other body
 * -- "the STL format" means the union of what 3D Systems' own (public
 * domain, informally circulated) "StereoLithography Interface
 * Specification" describes and what decades of divergent tool
 * implementations actually produce. See
 * https://en.wikipedia.org/wiki/STL_(file_format) for the format's
 * history and a good description of both variants parsed here:
 *
 *   - BINARY: an 80-byte free-form header (frequently, but not always,
 *     conventional ASCII text -- never parsed for meaning by a compliant
 *     reader), a little-endian uint32 triangle count, then that many
 *     50-byte facet records: a 3-float facet normal, three 3-float
 *     vertices, and a 2-byte "attribute byte count" field. That last
 *     field is a vendor extension slot (some tools stash a 16-bit color
 *     in it) this importer does not interpret.
 *   - ASCII: whitespace- and case-insensitive keyword syntax,
 *     `solid [name]` ... repeated `facet normal ni nj nk` / `outer loop`
 *     / three `vertex x y z` lines / `endloop` / `endfacet` ... blocks
 *     ... `endsolid [name]`.
 *
 * Both variants describe pure "triangle soup": every facet carries its
 * own independent 3 vertices, even where two facets share an edge in the
 * intended geometry, and there is no shared-vertex index, no face
 * grouping beyond one-facet-at-a-time, and no part/assembly structure at
 * all -- everything PRC's tree/tessellation model expects but STL simply
 * doesn't carry has to be reconstructed here: vertex welding (Step 2,
 * stl_weld_mesh) to get a genuinely connected mesh instead of a pile of
 * disconnected triangles, and connected-component analysis (also Step 2)
 * to notice when the welded mesh is actually several disjoint parts and
 * give each its own PRC tree node.
 * ==================================================================== */

#include <prc_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <prc_diag_env.h>

/* Minimal, single-block-only MD5 (RFC 1321), duplicated from src/prc_write_compress_tess.c's
   own (private/static) copy -- see that copy's comment for why this exists (replicating an
   earlier externally-scripted, empirically-confirmed-working mitigation byte-for-byte, after
   several simpler-hash variants failed to reproduce the same result). Input is always exactly
   12 bytes (3 packed float32 coordinates), always fits one 64-byte MD5 block. */
static void
prc_md5_12bytes(const uint8_t in[12], uint8_t out[16])
{
    static const uint32_t K[64] = {
        0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
        0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,0x6b901122,0xfd987193,0xa679438e,0x49b40821,
        0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
        0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
        0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
        0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
        0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
        0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391 };
    static const uint32_t S[64] = {
        7,12,17,22, 7,12,17,22, 7,12,17,22, 7,12,17,22,
        5, 9,14,20, 5, 9,14,20, 5, 9,14,20, 5, 9,14,20,
        4,11,16,23, 4,11,16,23, 4,11,16,23, 4,11,16,23,
        6,10,15,21, 6,10,15,21, 6,10,15,21, 6,10,15,21 };
    uint8_t block[64];
    uint32_t M[16];
    uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
    uint32_t a = a0, b = b0, c = c0, d = d0;
    int i;

    memset(block, 0, sizeof(block));
    memcpy(block, in, 12);
    block[12] = 0x80;
    block[56] = 96;

    for (i = 0; i < 16; i++)
        M[i] = (uint32_t)block[i * 4] | ((uint32_t)block[i * 4 + 1] << 8) |
               ((uint32_t)block[i * 4 + 2] << 16) | ((uint32_t)block[i * 4 + 3] << 24);

    for (i = 0; i < 64; i++)
    {
        uint32_t f, g, tmp;
        if (i < 16) { f = (b & c) | (~b & d); g = (uint32_t)i; }
        else if (i < 32) { f = (d & b) | (~d & c); g = (uint32_t)((5 * i + 1) % 16); }
        else if (i < 48) { f = b ^ c ^ d; g = (uint32_t)((3 * i + 5) % 16); }
        else { f = c ^ (b | ~d); g = (uint32_t)((7 * i) % 16); }
        tmp = d;
        d = c;
        c = b;
        f = a + f + K[i] + M[g];
        b = b + ((f << S[i]) | (f >> (32 - S[i])));
        a = tmp;
    }
    a0 += a; b0 += b; c0 += c; d0 += d;
    out[0] = (uint8_t)a0; out[1] = (uint8_t)(a0 >> 8); out[2] = (uint8_t)(a0 >> 16); out[3] = (uint8_t)(a0 >> 24);
    out[4] = (uint8_t)b0; out[5] = (uint8_t)(b0 >> 8); out[6] = (uint8_t)(b0 >> 16); out[7] = (uint8_t)(b0 >> 24);
    out[8] = (uint8_t)c0; out[9] = (uint8_t)(c0 >> 8); out[10] = (uint8_t)(c0 >> 16); out[11] = (uint8_t)(c0 >> 24);
    out[12] = (uint8_t)d0; out[13] = (uint8_t)(d0 >> 8); out[14] = (uint8_t)(d0 >> 16); out[15] = (uint8_t)(d0 >> 24);
}

/* ====================================================================
 * STL_IMPORT_SINGLE_MODEL_PART_THRESHOLD -- adjust this to change when
 * stl_import switches from one-model-per-part to a single lumped model.
 * ====================================================================
 *
 * When a welded STL splits into multiple disjoint connected components,
 * the normal behavior (stl_import_build_parts, below) gives each
 * component its own PRC_TYPE_ASM_FileStructureTessellation entry, rep
 * item, and 3-level tree branch (root -> intermediate[empty part] ->
 * leaf[real part]) -- so each part shows up separately in the model
 * tree. At real-world STL-scan scale that can mean tens or hundreds of
 * thousands of tree nodes and tessellation entries sharing one file
 * structure's bitstream section -- Adobe Acrobat/Reader are
 * independently known to have performance problems with very large/deep
 * model trees.
 *
 * When the welded STL has MORE than this many disjoint components,
 * stl_import instead merges every component's triangles into a single
 * lumped model: one tessellation entry, one rep item, one tree leaf,
 * with no per-part identity preserved in the tree (this is a plain
 * merge, not the abandoned "components as face groups within one entry"
 * approach -- see git history around 2026-07-23/24 if that's ever worth
 * revisiting; it was reverted because verifying multi-face-group entries
 * correctly needs its own dedicated internal test, not production code,
 * after it exposed a still-unfixed O(faces) per-call vertex_indices
 * allocation bug in prc_api_get_tessellation_vertices). A value of 0
 * would mean "always lump, never show separate parts" -- not a useful
 * setting in practice, so this define is not meant to be set to 0. */
#define STL_IMPORT_SINGLE_MODEL_PART_THRESHOLD 100

/* Multiplies `count * elem_size`, checking for size_t overflow first.
   Every allocation in this file that scales with a value read from (or
   derived from counting through) an untrusted input file goes through
   this rather than a bare `malloc(count * elem_size)` -- STL's binary
   triangle count is a full uint32 with no inherent upper bound beyond
   what the file's own declared size implies (checked separately, in
   stl_is_binary), and this importer is meant to handle real multi-
   million-triangle meshes, so `count` legitimately gets large enough that
   the multiplication itself, not just the resulting allocation, needs
   guarding -- particularly on a 32-bit build, where size_t is only 32
   bits and a wraparound here would silently return a too-small buffer
   that a subsequent fixed-stride write loop then overruns. Returns 0 and
   sets *out on success; returns -1 (leaving *out untouched) on overflow,
   which every call site treats as an ordinary allocation failure. */
static int
safe_mul_size(size_t count, size_t elem_size, size_t *out)
{
    if (elem_size != 0 && count > (SIZE_MAX / elem_size))
        return -1;
    *out = count * elem_size;
    return 0;
}

/* ---------------------------------------------------------------------
 * Section 1: STL parsing (binary + ASCII). Nothing in this section
 * touches nanoPRC's API -- it just turns a file on disk into a flat
 * "triangle soup": 9 doubles (3 vertices) and 3 doubles (the file's own
 * stored facet normal) per triangle, exactly as read, no welding yet.
 * Skip to Section 2 (stl_weld_mesh) if you just want to see how the
 * result gets turned into PRC geometry.
 * ------------------------------------------------------------------- */

#define STL_SOLID_NAME_CAP 256

typedef struct
{
    double *raw_positions;   /* 9 doubles per triangle (v0,v1,v2 xyz), num_triangles*9 entries */
    double *facet_normals;   /* 3 doubles per triangle, num_triangles*3 entries */
    uint32_t num_triangles;
    char solid_name[STL_SOLID_NAME_CAP]; /* first ASCII "solid NAME" seen, or "" (binary/unnamed) */
} stl_mesh;

static void
stl_mesh_free(stl_mesh *mesh)
{
    free(mesh->raw_positions);
    free(mesh->facet_normals);
    mesh->raw_positions = NULL;
    mesh->facet_normals = NULL;
    mesh->num_triangles = 0;
}

/* Reads an entire file into a heap buffer. Returns 0 on success. */
static int
stl_read_file(const char *path, uint8_t **out_buf, size_t *out_size)
{
    FILE *f;
    /* long is only 32 bits on Windows even in 64-bit builds (MSVC's ABI
       choice, unlike Linux/macOS where it's 64-bit) -- ftell()/fseek()
       silently misbehave above 2GB there. A real file over 2GB
       (48.8M-triangle binary STL, ~2.44GB) hit exactly this: reproducibly
       failed with "could not determine size", confirmed 2026-07-28. */
#if defined(_WIN32)
    __int64 size;
#else
    off_t size;
#endif
    uint8_t *buf;
    size_t read_count;

    *out_buf = NULL;
    *out_size = 0;

    f = fopen(path, "rb");
    if (f == NULL)
    {
        fprintf(stderr, "Error: could not open %s\n", path);
        return -1;
    }
#if defined(_WIN32)
    if (_fseeki64(f, 0, SEEK_END) != 0 || (size = _ftelli64(f)) < 0 || _fseeki64(f, 0, SEEK_SET) != 0)
#else
    if (fseeko(f, 0, SEEK_END) != 0 || (size = ftello(f)) < 0 || fseeko(f, 0, SEEK_SET) != 0)
#endif
    {
        fprintf(stderr, "Error: could not determine size of %s\n", path);
        fclose(f);
        return -1;
    }

    buf = (uint8_t *)malloc((size_t)size > 0 ? (size_t)size : 1);
    if (buf == NULL)
    {
        fprintf(stderr, "Error: allocation failed reading %s\n", path);
        fclose(f);
        return -1;
    }
    read_count = fread(buf, 1, (size_t)size, f);
    if (read_count != (size_t)size)
    {
        fprintf(stderr, "Error: short read on %s\n", path);
        free(buf);
        fclose(f);
        return -1;
    }
    fclose(f);

    *out_buf = buf;
    *out_size = (size_t)size;
    return 0;
}

/* Little-endian field readers: the STL binary format is explicitly
   little-endian regardless of host byte order (per the original
   StereoLithography spec and every real-world implementation), so these
   assemble values byte-by-byte instead of trusting the host's own
   endianness -- portable to a big-endian host, unlike a raw memcpy would
   be. The memcpy in stl_read_le_f32 is the standard portable way to
   reinterpret a 4-byte pattern as `float` in C without violating strict
   aliasing (a pointer-cast-and-dereference would not be). */
static uint32_t
stl_read_le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float
stl_read_le_f32(const uint8_t *p)
{
    uint32_t bits = stl_read_le_u32(p);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

/* Binary STL's triangle count is stored explicitly (unlike ASCII, where
   it must be discovered by parsing), so binary detection doesn't need to
   guess from the header bytes -- an 80-byte binary header can itself
   start with the literal bytes "solid" (a well-known STL footgun,
   several real-world binary exporters do exactly this), so keyword
   sniffing alone is not reliable. Instead: read the declared triangle
   count and check whether the file's actual size matches exactly what a
   binary file with that many triangles must be (84-byte fixed part +
   50 bytes/triangle). If the arithmetic matches, it's binary regardless
   of what the header bytes look like; ASCII is only attempted as a
   fallback when it doesn't. */
static int
stl_is_binary(const uint8_t *buf, size_t size, uint32_t *out_num_triangles)
{
    uint32_t count;
    uint64_t expected_size;

    if (size < 84)
        return 0;

    count = stl_read_le_u32(buf + 80);
    expected_size = (uint64_t)84 + (uint64_t)count * 50;
    if (expected_size != (uint64_t)size)
        return 0;

    *out_num_triangles = count;
    return 1;
}

static int
stl_parse_binary(const uint8_t *buf, size_t size, uint32_t num_triangles, stl_mesh *mesh)
{
    uint32_t t;
    const uint8_t *p = buf + 84;
    size_t positions_bytes, normals_bytes;

    (void)size; /* already validated by stl_is_binary */

    /* stl_is_binary already confirmed the file is physically large enough
       to hold num_triangles STL records (50 bytes each), which loosely
       bounds num_triangles by the file's own size on disk -- but this
       importer stores everything as double precision internally (72
       bytes/triangle for positions alone, vs STL's 50), so that loose
       bound doesn't by itself rule out the multiplications below
       overflowing size_t, particularly on a 32-bit build where a
       wraparound would silently hand back a too-small buffer for the
       fixed-stride write loop further down to overrun. safe_mul_size
       checks each multiplication explicitly instead of assuming it fits. */
    if (safe_mul_size((size_t)num_triangles, sizeof(double) * 9, &positions_bytes) != 0 ||
        safe_mul_size((size_t)num_triangles, sizeof(double) * 3, &normals_bytes) != 0)
    {
        fprintf(stderr, "Error: %u triangles is too large to process\n", (unsigned)num_triangles);
        return -1;
    }

    mesh->num_triangles = num_triangles;
    mesh->raw_positions = (double *)malloc(positions_bytes);
    mesh->facet_normals = (double *)malloc(normals_bytes);
    mesh->solid_name[0] = '\0';
    if ((num_triangles > 0) && (mesh->raw_positions == NULL || mesh->facet_normals == NULL))
    {
        stl_mesh_free(mesh);
        fprintf(stderr, "Error: allocation failed for %u binary STL triangles\n", (unsigned)num_triangles);
        return -1;
    }

    for (t = 0; t < num_triangles; t++)
    {
        int c;
        for (c = 0; c < 3; c++)
            mesh->facet_normals[t * 3 + c] = (double)stl_read_le_f32(p + c * 4);
        p += 12;
        for (c = 0; c < 9; c++)
            mesh->raw_positions[t * 9 + c] = (double)stl_read_le_f32(p + c * 4);
        p += 36;
        p += 2; /* attribute byte count, ignored */
    }
    return 0;
}

/* Minimal growable-array helper shared by the ASCII parser's two
   destination arrays (positions/normals) -- ASCII STL doesn't declare its
   triangle count up front the way binary does, so capacity is discovered
   by doubling as triangles are parsed. */
static int
stl_ascii_reserve(void **arr, uint32_t *cap, uint32_t need, size_t elem_size)
{
    uint32_t new_cap;
    size_t new_bytes;
    void *new_arr;

    if (need <= *cap)
        return 0;
    new_cap = (*cap == 0) ? 64 : (*cap * 2);
    /* Guard the doubling loop itself: if `need` is pathologically close to
       UINT32_MAX (only reachable by an ASCII STL file large enough to
       contain that many "facet ... endfacet" blocks, but defend anyway --
       see safe_mul_size's own comment for why this file treats untrusted-
       input-driven sizes as adversarial by default), new_cap *= 2 would
       eventually overflow uint32_t and wrap to a small value, making
       `new_cap < need` true forever instead of terminating. */
    while (new_cap < need)
    {
        if (new_cap > UINT32_MAX / 2)
        {
            new_cap = need;
            break;
        }
        new_cap *= 2;
    }
    if (safe_mul_size((size_t)new_cap, elem_size, &new_bytes) != 0)
        return -1;
    new_arr = realloc(*arr, new_bytes);
    if (new_arr == NULL)
        return -1;
    *arr = new_arr;
    *cap = new_cap;
    return 0;
}

/* Case-insensitive, whitespace-delimited token reader over an in-memory
   buffer that is not guaranteed to be NUL-terminated (stl_read_file's
   buffer isn't). Advances *cursor past the token. Returns 0 at EOF. */
static int
stl_ascii_next_token(const uint8_t *buf, size_t size, size_t *cursor, char *token, size_t token_cap)
{
    size_t i = *cursor;
    size_t len = 0;

    while (i < size && isspace(buf[i]))
        i++;
    if (i >= size)
    {
        *cursor = i;
        return 0;
    }
    while (i < size && !isspace(buf[i]) && len + 1 < token_cap)
        token[len++] = (char)buf[i++];
    /* If the real token is longer than token_cap, keep consuming it (an
       overlong token can only be a malformed/foreign line, not a real STL
       keyword or number) so the cursor still lands past it. */
    while (i < size && !isspace(buf[i]))
        i++;
    token[len] = '\0';
    *cursor = i;
    return 1;
}

static int
stl_token_ieq(const char *token, const char *keyword)
{
    size_t i;
    for (i = 0; token[i] != '\0' || keyword[i] != '\0'; i++)
    {
        if (tolower((unsigned char)token[i]) != tolower((unsigned char)keyword[i]))
            return 0;
    }
    return 1;
}

/* Consumes the rest of the current line (used for the "solid [name]"
   line, whose name may itself contain spaces -- some real-world
   exporters do this even though it's not, strictly, one whitespace-
   delimited token) and copies it, trimmed, into `out`. */
static void
stl_ascii_rest_of_line(const uint8_t *buf, size_t size, size_t *cursor, char *out, size_t out_cap)
{
    size_t i = *cursor;
    size_t start, end, len;

    while (i < size && (buf[i] == ' ' || buf[i] == '\t'))
        i++;
    start = i;
    while (i < size && buf[i] != '\n' && buf[i] != '\r')
        i++;
    end = i;
    while (end > start && isspace(buf[end - 1]))
        end--;
    len = end - start;
    if (len >= out_cap)
        len = out_cap - 1;
    memcpy(out, buf + start, len);
    out[len] = '\0';
    while (i < size && (buf[i] == '\n' || buf[i] == '\r'))
        i++;
    *cursor = i;
}

/* Parses one or more concatenated "solid ... endsolid" blocks (the ASCII
   spec strictly allows exactly one, but real-world files -- e.g. some
   CAD tools' combined multi-body exports -- concatenate several; looping
   here instead of erroring on trailing content after the first
   "endsolid" tolerates that without any special-case part-naming logic,
   since Section 2's connected-component analysis will separate the
   resulting geometry into distinct PRC parts on its own regardless of
   which solid block each triangle came from). Returns 0 on success. */
static int
stl_parse_ascii(const uint8_t *buf, size_t size, stl_mesh *mesh)
{
    size_t cursor = 0;
    char token[64];
    uint32_t count = 0;
    /* Separate capacity trackers for positions/normals -- they grow in
       lockstep (one triangle appends exactly one of each), but have
       different element sizes (9 doubles vs. 3), so sharing a single cap
       variable between the two stl_ascii_reserve calls below would let
       positions' growth (a larger element size, so it grows its
       underlying byte capacity faster) satisfy normals' `need <= cap`
       check without normals ever actually being allocated -- normals
       stays NULL, and the write below segfaults on the first triangle. */
    uint32_t positions_cap = 0, normals_cap = 0;
    double *positions = NULL;
    double *normals = NULL;
    int have_name = 0;

    mesh->solid_name[0] = '\0';

    while (stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)))
    {
        if (!stl_token_ieq(token, "solid"))
        {
            fprintf(stderr, "Error: expected 'solid', got '%s'\n", token);
            goto fail;
        }
        {
            char name[STL_SOLID_NAME_CAP];
            stl_ascii_rest_of_line(buf, size, &cursor, name, sizeof(name));
            if (!have_name && name[0] != '\0')
            {
                memcpy(mesh->solid_name, name, sizeof(mesh->solid_name));
                have_name = 1;
            }
        }

        for (;;)
        {
            double normal[3], verts[9];
            int i;

            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)))
            {
                fprintf(stderr, "Error: unexpected end of file inside 'solid' block\n");
                goto fail;
            }
            if (stl_token_ieq(token, "endsolid"))
            {
                /* Optional trailing name token(s) on this line are irrelevant. */
                stl_ascii_rest_of_line(buf, size, &cursor, token, sizeof(token));
                break;
            }
            if (!stl_token_ieq(token, "facet"))
            {
                fprintf(stderr, "Error: expected 'facet' or 'endsolid', got '%s'\n", token);
                goto fail;
            }
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "normal"))
            {
                fprintf(stderr, "Error: expected 'normal' after 'facet'\n");
                goto fail;
            }
            for (i = 0; i < 3; i++)
            {
                if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)))
                {
                    fprintf(stderr, "Error: truncated facet normal\n");
                    goto fail;
                }
                normal[i] = strtod(token, NULL);
            }
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "outer"))
            {
                fprintf(stderr, "Error: expected 'outer loop'\n");
                goto fail;
            }
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "loop"))
            {
                fprintf(stderr, "Error: expected 'loop' after 'outer'\n");
                goto fail;
            }
            {
                int v;
                for (v = 0; v < 3; v++)
                {
                    if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "vertex"))
                    {
                        fprintf(stderr, "Error: expected 'vertex'\n");
                        goto fail;
                    }
                    for (i = 0; i < 3; i++)
                    {
                        if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)))
                        {
                            fprintf(stderr, "Error: truncated vertex\n");
                            goto fail;
                        }
                        verts[v * 3 + i] = strtod(token, NULL);
                    }
                }
            }
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "endloop"))
            {
                fprintf(stderr, "Error: expected 'endloop'\n");
                goto fail;
            }
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "endfacet"))
            {
                fprintf(stderr, "Error: expected 'endfacet'\n");
                goto fail;
            }

            if (stl_ascii_reserve((void **)&positions, &positions_cap, count + 1, sizeof(double) * 9) != 0 ||
                stl_ascii_reserve((void **)&normals, &normals_cap, count + 1, sizeof(double) * 3) != 0)
            {
                fprintf(stderr, "Error: allocation failed parsing ASCII STL\n");
                goto fail;
            }
            memcpy(positions + (size_t)count * 9, verts, sizeof(verts));
            memcpy(normals + (size_t)count * 3, normal, sizeof(normal));
            count++;
        }

        /* Peek for another "solid" block; stop at EOF. */
        {
            size_t save = cursor;
            if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)))
                break;
            if (!stl_token_ieq(token, "solid"))
            {
                fprintf(stderr, "Error: expected 'solid' or end of file, got '%s'\n", token);
                goto fail;
            }
            cursor = save;
        }
    }

    mesh->raw_positions = positions;
    mesh->facet_normals = normals;
    mesh->num_triangles = count;
    return 0;

fail:
    free(positions);
    free(normals);
    return -1;
}

/* Parses `path` into `mesh`. On success, establishes an invariant every
   later allocation in this file relies on without re-checking:
   mesh->num_triangles * 72 (sizeof(double)*9, the single largest per-
   triangle byte cost anywhere in this importer -- raw_positions and, in
   stl_import_build_parts, each part's local_positions/local_normals
   upper bound) fits in size_t. stl_parse_binary checks this explicitly
   via safe_mul_size before allocating; stl_parse_ascii's growth (via
   stl_ascii_reserve, called with that same 72-byte element size for its
   positions array) can only have grown mesh->num_triangles as far as an
   actual successful allocation at that size allowed, so the invariant
   holds there too, just established incrementally rather than up front.
   Every other per-triangle array in this file (weld_index, component_of_
   triangle, local_tri_indices, ...) costs strictly fewer bytes/triangle
   than 72, so this one invariant is sufficient to bound them all. */
static int
stl_mesh_load(const char *path, stl_mesh *mesh)
{
    uint8_t *buf;
    size_t size;
    uint32_t bin_count;
    int rc;

    memset(mesh, 0, sizeof(*mesh));

    if (stl_read_file(path, &buf, &size) != 0)
        return -1;

    if (stl_is_binary(buf, size, &bin_count))
    {
        rc = stl_parse_binary(buf, size, bin_count, mesh);
    }
    else
    {
        size_t cursor = 0;
        char token[64];
        if (!stl_ascii_next_token(buf, size, &cursor, token, sizeof(token)) || !stl_token_ieq(token, "solid"))
        {
            fprintf(stderr, "Error: %s is neither a well-formed binary STL nor an ASCII STL "
                             "starting with 'solid'\n", path);
            free(buf);
            return -1;
        }
        rc = stl_parse_ascii(buf, size, mesh);
    }

    free(buf);
    return rc;
}

/* Compacts degenerate triangles out of `mesh` in place: both exactly (or
   near-) zero-area triangles (three collinear/coincident points) and
   extreme "sliver" triangles (real, nonzero area, but one edge orders of
   magnitude shorter than the other two -- three nearly-collinear points).
   Neither contributes meaningful surface detail. Filtering here, before
   these triangles reach vertex welding or the write API, isn't just
   hygiene: real customer STL data (from a mesh-cutting operation) was
   found, during this importer's own real-world dataset testing, to
   contain both kinds and to trigger a write/decode-path bug
   (`points_is_reference_index out of range` from an otherwise-unrelated
   part of the library, `prc_decode_compressed_tess.c`, when reading the
   written file back) when they reached the COMPRESSED tessellation
   encoder -- an exactly-zero-area triangle alone was not sufficient to
   reproduce it; a cluster of extreme slivers (aspect ratios from ~2e4 up
   to several million, confirmed by direct inspection of the failing
   triangles) in the same region was. Degenerate input the encoder isn't
   expected to handle gracefully either way, and none of it carries real
   geometric information, so this drops both classes rather than
   depending on downstream encoder robustness to either.
   `area_threshold` is an absolute area (not squared); `aspect_threshold`
   bounds `longest_edge^2 / area` (dimensionally a length, so it scales
   naturally with `area_threshold` -- see the caller for how both are
   chosen). Returns the number of triangles removed. */
static uint32_t
stl_filter_degenerate_triangles(stl_mesh *mesh, double area_threshold, double aspect_threshold)
{
    uint32_t src, dst = 0;

    for (src = 0; src < mesh->num_triangles; src++)
    {
        const double *v0 = &mesh->raw_positions[(size_t)src * 9 + 0];
        const double *v1 = &mesh->raw_positions[(size_t)src * 9 + 3];
        const double *v2 = &mesh->raw_positions[(size_t)src * 9 + 6];
        double ux = v1[0] - v0[0], uy = v1[1] - v0[1], uz = v1[2] - v0[2]; /* v0->v1 */
        double wx = v2[0] - v0[0], wy = v2[1] - v0[1], wz = v2[2] - v0[2]; /* v0->v2 */
        double vx = v2[0] - v1[0], vy = v2[1] - v1[1], vz = v2[2] - v1[2]; /* v1->v2 */
        double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
        double area = 0.5 * sqrt(cx * cx + cy * cy + cz * cz);
        double e0_sq = ux * ux + uy * uy + uz * uz;
        double e1_sq = vx * vx + vy * vy + vz * vz;
        double e2_sq = wx * wx + wy * wy + wz * wz;
        double max_edge_sq = e0_sq;
        if (e1_sq > max_edge_sq) max_edge_sq = e1_sq;
        if (e2_sq > max_edge_sq) max_edge_sq = e2_sq;

        if (area <= area_threshold || max_edge_sq / area > aspect_threshold)
            continue; /* degenerate or extreme sliver: drop it */

        if (dst != src)
        {
            memcpy(&mesh->raw_positions[(size_t)dst * 9], &mesh->raw_positions[(size_t)src * 9], sizeof(double) * 9);
            memcpy(&mesh->facet_normals[(size_t)dst * 3], &mesh->facet_normals[(size_t)src * 3], sizeof(double) * 3);
        }
        dst++;
    }

    {
        uint32_t removed = mesh->num_triangles - dst;
        mesh->num_triangles = dst;
        return removed;
    }
}

/* ---------------------------------------------------------------------
 * Section 2: vertex welding + connected-component analysis.
 *
 * STL's triangle soup gives every facet its own private 3 vertices, even
 * along edges shared with a neighboring facet in the intended geometry.
 * This section merges vertices that coincide within a tolerance (a
 * classic spatial-hash weld: quantize each point to a grid cell sized to
 * the tolerance, then check only that cell and its 26 neighbors for an
 * existing point within range -- checking neighbors, not just the same
 * cell, is what makes this correct across a cell boundary; checking real
 * distance afterward, not just cell membership, is what keeps two
 * unrelated points sharing a cell from being wrongly merged), then does a
 * union-find flood-fill over the now-welded triangles to find connected
 * components -- each becomes its own PRC tree node in Section 3.
 *
 * This is done here in the demo rather than left entirely to
 * PRC_API_WRITE_TESS_KIND_COMPRESSED's own internal tolerance-based weld
 * (include/prc_api.h) for one reason that has nothing to do with which
 * tessellation kind ends up on the wire: Section 4 below writes
 * COMPRESSED, which does its own tolerance-based vertex weld internally
 * (redundant but harmless on data already welded to the same tolerance --
 * see Section 4's own comment), but that internal weld is opaque to the
 * caller. Section 4's connected-component split into separate PRC tree
 * nodes needs an explicit adjacency graph over welded vertices, which only
 * exists if this demo builds it itself.
 *
 * (History note, since an earlier version of this comment cited it as the
 * reason to prefer TRIANGLES over COMPRESSED: include/prc_api.h's
 * prc_api_write_tess_kind_t used to warn that an independent, non-nanoPRC
 * PRC reader returned null/empty geometry for every COMPRESSED-written
 * file. That was a real bug (large-mesh shard corruption in the encoder),
 * but it was fixed and independently confirmed rendering correctly in
 * both Adobe Acrobat and PDF-XChange -- see that header's own doc comment
 * for the current, authoritative guidance. COMPRESSED is used below both
 * because it's now confirmed solid and because its EdgeBreaker-style
 * traversal encoding gives meaningfully smaller output files than
 * TRIANGLES at the multi-million-triangle scale a real-world STL import
 * needs to handle.)
 * ------------------------------------------------------------------- */

/* --- 2a: tolerance-based spatial-hash vertex weld --- */

typedef struct
{
    int64_t cell[3];
    uint32_t vertex_index;
} weld_bucket_entry;

typedef struct
{
    weld_bucket_entry *entries;
    uint32_t count;
    uint32_t capacity;
} weld_bucket;

typedef struct
{
    weld_bucket *buckets;
    uint32_t num_buckets; /* power of two */
    double cell_size;
    double *positions;    /* welded positions, growable, 3 doubles each */
    uint32_t count;
    uint32_t capacity;
} weld_grid;

/* Spatial-hash cell mixing function (the 73856093/19349663/83492791 primes
   are the well-known constants from Teschner et al., "Optimized Spatial
   Hashing for Collision Detection of Deformable Objects", used here only
   as a generic public technique, not project-specific tuning), followed
   by a 64-bit finalizer mix (Murmur3-style) so nearby cells don't land in
   nearby buckets. */
static uint32_t
weld_hash_cell(int64_t ix, int64_t iy, int64_t iz)
{
    uint64_t h = (uint64_t)ix * 73856093ULL ^ (uint64_t)iy * 19349663ULL ^ (uint64_t)iz * 83492791ULL;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    return (uint32_t)h;
}

static int
weld_grid_init(weld_grid *g, double cell_size, uint32_t expected_vertices)
{
    uint32_t want = expected_vertices < 8 ? 8 : expected_vertices;
    uint32_t n = 16;
    while (n < want * 2)
        n *= 2;

    memset(g, 0, sizeof(*g));
    g->cell_size = cell_size > 0.0 ? cell_size : 1e-9;
    g->num_buckets = n;
    g->buckets = (weld_bucket *)calloc(n, sizeof(weld_bucket));
    return g->buckets != NULL ? 0 : -1;
}

static void
weld_grid_free(weld_grid *g)
{
    uint32_t i;
    for (i = 0; i < g->num_buckets; i++)
        free(g->buckets[i].entries);
    free(g->buckets);
    free(g->positions);
    memset(g, 0, sizeof(*g));
}

static int
weld_bucket_append(weld_bucket *b, int64_t ix, int64_t iy, int64_t iz, uint32_t vertex_index)
{
    if (b->count == b->capacity)
    {
        uint32_t new_cap = b->capacity == 0 ? 4 : b->capacity * 2;
        weld_bucket_entry *new_entries = (weld_bucket_entry *)realloc(b->entries, sizeof(weld_bucket_entry) * new_cap);
        if (new_entries == NULL)
            return -1;
        b->entries = new_entries;
        b->capacity = new_cap;
    }
    b->entries[b->count].cell[0] = ix;
    b->entries[b->count].cell[1] = iy;
    b->entries[b->count].cell[2] = iz;
    b->entries[b->count].vertex_index = vertex_index;
    b->count++;
    return 0;
}

/* Finds an existing welded vertex within `tolerance` of `p`, or creates a
   new one. Returns the welded vertex index, or UINT32_MAX on allocation
   failure. */
static uint32_t
weld_grid_lookup_or_insert(weld_grid *g, const double p[3], double tolerance)
{
    int64_t base[3];
    int dx, dy, dz, c;
    double tol2 = tolerance * tolerance;

    for (c = 0; c < 3; c++)
        base[c] = (int64_t)floor(p[c] / g->cell_size);

    for (dx = -1; dx <= 1; dx++)
    {
        for (dy = -1; dy <= 1; dy++)
        {
            for (dz = -1; dz <= 1; dz++)
            {
                int64_t ix = base[0] + dx, iy = base[1] + dy, iz = base[2] + dz;
                uint32_t bucket_index = weld_hash_cell(ix, iy, iz) & (g->num_buckets - 1);
                weld_bucket *b = &g->buckets[bucket_index];
                uint32_t i;

                for (i = 0; i < b->count; i++)
                {
                    const weld_bucket_entry *e = &b->entries[i];
                    double ddx, ddy, ddz, dist2;
                    const double *existing;

                    if (e->cell[0] != ix || e->cell[1] != iy || e->cell[2] != iz)
                        continue;
                    existing = &g->positions[e->vertex_index * 3];
                    ddx = existing[0] - p[0];
                    ddy = existing[1] - p[1];
                    ddz = existing[2] - p[2];
                    dist2 = ddx * ddx + ddy * ddy + ddz * ddz;
                    if (dist2 <= tol2)
                        return e->vertex_index;
                }
            }
        }
    }

    /* No match within tolerance: create a new welded vertex, indexed under
       its own (not a neighbor's) cell. */
    if (g->count == g->capacity)
    {
        uint32_t new_cap;
        size_t new_bytes;
        double *new_positions;

        if (g->capacity == 0)
            new_cap = 64;
        else if (g->capacity > UINT32_MAX / 2)
            new_cap = UINT32_MAX; /* can't double further without wrapping; grow by the largest safe step instead */
        else
            new_cap = g->capacity * 2;

        if (safe_mul_size((size_t)new_cap, sizeof(double) * 3, &new_bytes) != 0)
            return UINT32_MAX;
        new_positions = (double *)realloc(g->positions, new_bytes);
        if (new_positions == NULL)
            return UINT32_MAX;
        g->positions = new_positions;
        g->capacity = new_cap;
    }
    g->positions[g->count * 3 + 0] = p[0];
    g->positions[g->count * 3 + 1] = p[1];
    g->positions[g->count * 3 + 2] = p[2];
    {
        uint32_t bucket_index = weld_hash_cell(base[0], base[1], base[2]) & (g->num_buckets - 1);
        if (weld_bucket_append(&g->buckets[bucket_index], base[0], base[1], base[2], g->count) != 0)
            return UINT32_MAX;
    }
    return g->count++;
}

/* --- 2b: union-find over welded vertices, to find connected components --- */

static uint32_t
uf_find(uint32_t *parent, uint32_t x)
{
    while (parent[x] != x)
    {
        parent[x] = parent[parent[x]]; /* path halving */
        x = parent[x];
    }
    return x;
}

static void
uf_union(uint32_t *parent, uint8_t *rank, uint32_t a, uint32_t b)
{
    a = uf_find(parent, a);
    b = uf_find(parent, b);
    if (a == b)
        return;
    if (rank[a] < rank[b])
    {
        uint32_t t = a; a = b; b = t;
    }
    parent[b] = a;
    if (rank[a] == rank[b])
        rank[a]++;
}

/* ---------------------------------------------------------------------
 * Section 3: normal-index deduplication for --original-normals.
 *
 * Real STL files frequently repeat the exact same facet normal across
 * many triangles (e.g. every triangle on one face of a box). This is
 * only ever used on values read verbatim from the file (never geometry-
 * derived), so an exact bit-pattern match is the right notion of
 * "duplicate" here -- unlike Section 2's position weld, no tolerance is
 * involved. A much smaller, simpler hash than weld_grid suffices since
 * there is no neighbor-cell search: two normals either hash to the same
 * bucket and compare equal, or they don't.
 * ------------------------------------------------------------------- */

typedef struct
{
    double value[3];
    uint32_t normal_index;
} normal_bucket_entry;

typedef struct
{
    normal_bucket_entry *entries;
    uint32_t count;
    uint32_t capacity;
} normal_bucket;

typedef struct
{
    normal_bucket *buckets;
    uint32_t num_buckets;
    double *normals; /* deduplicated normals, growable, 3 doubles each */
    uint32_t count;
    uint32_t capacity;
} normal_dedup;

static int
normal_dedup_init(normal_dedup *d, uint32_t expected)
{
    uint32_t want = expected < 8 ? 8 : expected;
    uint32_t n = 16;
    while (n < want * 2)
        n *= 2;
    memset(d, 0, sizeof(*d));
    d->num_buckets = n;
    d->buckets = (normal_bucket *)calloc(n, sizeof(normal_bucket));
    return d->buckets != NULL ? 0 : -1;
}

static void
normal_dedup_free(normal_dedup *d)
{
    uint32_t i;
    for (i = 0; i < d->num_buckets; i++)
        free(d->buckets[i].entries);
    free(d->buckets);
    free(d->normals);
    memset(d, 0, sizeof(*d));
}

static uint32_t
normal_dedup_hash(const double v[3])
{
    uint64_t bits[3], h = 1469598103934665603ULL; /* FNV-1a offset basis */
    int c;
    memcpy(bits, v, sizeof(bits));
    for (c = 0; c < 3; c++)
    {
        h ^= bits[c];
        h *= 1099511628211ULL; /* FNV-1a prime */
    }
    h ^= h >> 33;
    return (uint32_t)h;
}

static uint32_t
normal_dedup_get(normal_dedup *d, const double v[3])
{
    uint32_t bucket_index = normal_dedup_hash(v) & (d->num_buckets - 1);
    normal_bucket *b = &d->buckets[bucket_index];
    uint32_t i;

    for (i = 0; i < b->count; i++)
    {
        if (memcmp(b->entries[i].value, v, sizeof(double) * 3) == 0)
            return b->entries[i].normal_index;
    }

    if (d->count == d->capacity)
    {
        uint32_t new_cap;
        size_t new_bytes;
        double *new_normals;

        if (d->capacity == 0)
            new_cap = 64;
        else if (d->capacity > UINT32_MAX / 2)
            new_cap = UINT32_MAX;
        else
            new_cap = d->capacity * 2;

        if (safe_mul_size((size_t)new_cap, sizeof(double) * 3, &new_bytes) != 0)
            return UINT32_MAX;
        new_normals = (double *)realloc(d->normals, new_bytes);
        if (new_normals == NULL)
            return UINT32_MAX;
        d->normals = new_normals;
        d->capacity = new_cap;
    }
    d->normals[d->count * 3 + 0] = v[0];
    d->normals[d->count * 3 + 1] = v[1];
    d->normals[d->count * 3 + 2] = v[2];

    if (b->count == b->capacity)
    {
        uint32_t new_bcap = b->capacity == 0 ? 4 : b->capacity * 2;
        normal_bucket_entry *new_entries = (normal_bucket_entry *)realloc(b->entries, sizeof(normal_bucket_entry) * new_bcap);
        if (new_entries == NULL)
            return UINT32_MAX;
        b->entries = new_entries;
        b->capacity = new_bcap;
    }
    memcpy(b->entries[b->count].value, v, sizeof(double) * 3);
    b->entries[b->count].normal_index = d->count;
    b->count++;

    return d->count++;
}

/* ---------------------------------------------------------------------
 * Section 4: turning the welded, component-labeled mesh into the public
 * write API's three-level shape -- one prc_api_write_tessellation +
 * prc_api_write_rep_item + prc_api_write_node per connected component,
 * all under one bare assembly root. This is the section that actually
 * calls into nanoPRC's own data model; Sections 1-3 above are pure STL/
 * geometry-processing code with no nanoPRC dependency at all.
 * ------------------------------------------------------------------- */

static void
bbox_reset(double bmin[3], double bmax[3])
{
    int i;
    for (i = 0; i < 3; i++) { bmin[i] = 1e300; bmax[i] = -1e300; }
}

static void
bbox_expand(double bmin[3], double bmax[3], const double p[3])
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if (p[i] < bmin[i]) bmin[i] = p[i];
        if (p[i] > bmax[i]) bmax[i] = p[i];
    }
}

/* Restricts `in` to a conservative safe character set (ASCII letters,
   digits, underscore, hyphen, space, period) before it becomes a PRC tree
   node name. Both inputs this is used on are attacker-controlled -- the
   STL file's own "solid" name (arbitrary bytes from an untrusted file)
   and the input filename (whatever a caller's script assembled) -- and a
   node name ultimately ends up as text inside a binary format that other
   tools (Acrobat, this library's own reader, third-party PRC viewers)
   will later parse; letting control characters, stray quotes/backslashes,
   or non-ASCII bytes that might not even be valid UTF-8 straight through
   risks confusing one of those consumers. Everything outside the safe set
   becomes '_' rather than being rejected outright, so a malformed name
   degrades to something ugly but safe instead of aborting the whole
   import; `fallback` covers the case where nothing safe was left at all
   (e.g. an empty or entirely-symbolic name). */
static void
stl_sanitize_name(const char *in, char *out, size_t out_cap, const char *fallback)
{
    size_t i, len = 0, start;

    for (i = 0; in[i] != '\0' && len + 1 < out_cap; i++)
    {
        unsigned char ch = (unsigned char)in[i];
        int ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
                 ch == '_' || ch == '-' || ch == ' ' || ch == '.';
        out[len++] = ok ? (char)ch : '_';
    }
    out[len] = '\0';

    /* Trim leading/trailing spaces left over from the filter -- cosmetic,
       not a safety requirement, but keeps e.g. a name that was all
       control characters from rendering as an invisible blank. */
    while (len > 0 && out[len - 1] == ' ')
        out[--len] = '\0';
    start = 0;
    while (out[start] == ' ')
        start++;
    if (start > 0)
        memmove(out, out + start, len - start + 1);

    if (out[0] == '\0')
        snprintf(out, out_cap, "%s", fallback);
}

typedef struct
{
    prc_api_write_tessellation *tess_entries; /* num_components entries */
    prc_api_write_rep_item *rep_items;        /* num_components entries */
    prc_api_write_node *children;             /* num_components entries -- leaf nodes, each owning one real part */
    prc_api_write_node **child_ptrs;          /* num_components entries, child_ptrs[i] == &children[i] */
    prc_api_write_node *intermediates;        /* num_components entries -- see stl_import_build_parts' comment */
    prc_api_write_node **intermediate_ptrs;   /* num_components entries, intermediate_ptrs[i] == &intermediates[i] */
    char (*names)[64];                        /* num_components entries, backing storage for children[i].name */
    uint32_t num_components;
} stl_parts;

static void
stl_parts_free(stl_parts *parts)
{
    uint32_t i;
    for (i = 0; i < parts->num_components; i++)
    {
        /* prc_api_write_tessellation's array fields are `const` (the write
           API never mutates caller data), so freeing them back requires
           casting the const away -- these pointers are exclusively owned
           by this demo (allocated in stl_import_build_parts below, never
           aliased elsewhere), so that's safe here. */
        free((void *)parts->tess_entries[i].positions);
        free((void *)parts->tess_entries[i].tri_indices);
        free((void *)parts->tess_entries[i].normals);
        free((void *)parts->tess_entries[i].norm_indices);
        /* Set only when the non-manifold-fan check downgrades an entry
           (per-component or lumped) from COMPRESSED to TRIANGLES; NULL
           otherwise, and free(NULL) is a no-op. */
        free((void *)parts->tess_entries[i].face_tri_counts);
    }
    free(parts->tess_entries);
    free(parts->rep_items);
    free(parts->children);
    free(parts->child_ptrs);
    free(parts->intermediates);
    free(parts->intermediate_ptrs);
    free(parts->names);
    memset(parts, 0, sizeof(*parts));
}

/* STL_IMPORT_SINGLE_MODEL_PART_THRESHOLD path (see that #define's own
   comment near the top of the file): builds exactly ONE tessellation
   entry / rep item / tree leaf for the WHOLE mesh, merging every
   connected component's triangles into one plain, ungrouped COMPRESSED
   entry (no face_tri_counts/num_faces grouping -- see include/prc_api.h's
   prc_api_write_tessellation doc comment: NULL/0 there is legal for
   COMPRESSED and means "whole entry treated as one face"). No per-part
   identity survives in the model tree; that's the whole point of this
   path existing, for STLs too fragmented to give every part its own
   tree node/entry without risking Acrobat's known large/deep-tree
   performance problems. Comes back with parts->num_components == 1, so
   every caller downstream of stl_import_build_parts (root node assembly,
   prc_api_write_prc_buffer, stl_parts_free in Section 5) works completely
   unchanged -- from the outside this looks exactly like the
   num_components==1 case of the default per-component path.

   No per-component grouping/reordering is needed here (unlike the
   default path, and unlike the abandoned face-groups approach this
   replaced): a plain merge doesn't care what order triangles come in, so
   weld_index's global welded-vertex ids into welded_positions, and
   global_normal_index's ids into dedup_normals, are used directly,
   triangle by triangle, in their original order. */
static int
stl_import_build_single_lumped_model(const stl_mesh *mesh, const double *welded_positions, uint32_t num_welded,
    const uint32_t *weld_index, int original_normals, const uint32_t *global_normal_index,
    const double *dedup_normals, double weld_tolerance_fraction, int check_nonmanifold, stl_parts *parts)
{
    uint32_t *combined_tri_indices = NULL;
    double *combined_positions = NULL;
    double *combined_normals = NULL;
    uint32_t *combined_norm_indices = NULL;
    double bbox_min[3], bbox_max[3];
    uint32_t t;
    uint32_t max_normal_id = 0;
    int ok = -1;
    prc_context *nonmanifold_check_ctx = prc_api_new_context(NULL);

    memset(parts, 0, sizeof(*parts));

    /* tess_entries[].positions/tri_indices are demo-owned and freed by
       stl_parts_free, so this can't just alias the caller's own
       weld_index/welded_positions buffers (main() owns and frees those
       separately) -- copy instead. */
    combined_tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * mesh->num_triangles);
    combined_positions = (double *)malloc(sizeof(double) * 3 * (num_welded > 0 ? num_welded : 1));
    if (combined_tri_indices == NULL || combined_positions == NULL)
    {
        fprintf(stderr, "Error: allocation failed building lumped single model\n");
        goto cleanup;
    }
    memcpy(combined_positions, welded_positions, sizeof(double) * 3 * num_welded);
    memcpy(combined_tri_indices, weld_index, sizeof(uint32_t) * 3 * mesh->num_triangles);

    bbox_reset(bbox_min, bbox_max);
    for (t = 0; t < num_welded; t++)
        bbox_expand(bbox_min, bbox_max, &welded_positions[t * 3]);

    if (original_normals)
    {
        for (t = 0; t < mesh->num_triangles; t++)
            if (global_normal_index[t] + 1 > max_normal_id) max_normal_id = global_normal_index[t] + 1;
        combined_normals = (double *)malloc(sizeof(double) * 3 * (max_normal_id > 0 ? max_normal_id : 1));
        combined_norm_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * mesh->num_triangles);
        if (combined_normals == NULL || combined_norm_indices == NULL)
        {
            fprintf(stderr, "Error: allocation failed building lumped single model normals\n");
            goto cleanup;
        }
        /* dedup_normals is already a global, deduplicated array (Section 3)
           -- global_normal_index's values index it directly, no remap
           needed for a plain merge. */
        memcpy(combined_normals, dedup_normals, sizeof(double) * 3 * max_normal_id);
        for (t = 0; t < mesh->num_triangles; t++)
        {
            uint32_t gn = global_normal_index[t];
            combined_norm_indices[t * 3 + 0] = gn;
            combined_norm_indices[t * 3 + 1] = gn;
            combined_norm_indices[t * 3 + 2] = gn;
        }
    }

    parts->tess_entries = (prc_api_write_tessellation *)calloc(1, sizeof(prc_api_write_tessellation));
    parts->rep_items = (prc_api_write_rep_item *)calloc(1, sizeof(prc_api_write_rep_item));
    parts->children = (prc_api_write_node *)calloc(1, sizeof(prc_api_write_node));
    parts->child_ptrs = (prc_api_write_node **)calloc(1, sizeof(prc_api_write_node *));
    parts->intermediates = (prc_api_write_node *)calloc(1, sizeof(prc_api_write_node));
    parts->intermediate_ptrs = (prc_api_write_node **)calloc(1, sizeof(prc_api_write_node *));
    parts->names = (char (*)[64])calloc(1, sizeof(char[64]));
    if (parts->tess_entries == NULL || parts->rep_items == NULL || parts->children == NULL ||
        parts->child_ptrs == NULL || parts->intermediates == NULL || parts->intermediate_ptrs == NULL ||
        parts->names == NULL)
    {
        fprintf(stderr, "Error: allocation failed building lumped single model part list\n");
        goto cleanup;
    }
    parts->num_components = 1;

    {
        prc_api_write_tessellation *tess = &parts->tess_entries[0];
        tess->kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
        tess->positions = combined_positions;
        tess->num_positions = num_welded;
        tess->tri_indices = combined_tri_indices;
        tess->num_triangles = mesh->num_triangles;
        /* face_tri_counts/num_faces left at NULL/0 for the COMPRESSED case:
           this is a plain merge, not the abandoned face-groups approach --
           see this function's own doc comment. May be set below instead if
           the non-manifold-fan check downgrades this entry to TRIANGLES. */
        {
            /* EXPERIMENT (2026-07-28), DISPROVEN same day: reference
               pseudocode (sample_compressed_mesh_write_pseudocode.md) halves
               the encoding tolerance outright "if the mesh lacks
               connectivity" -- this function is only ever reached when
               merging multiple originally-disjoint connected components
               into one entry (forced either by the >100-part threshold or
               the {3,7,14} known-bad-count list), i.e. exactly that case,
               unconditionally, every time. Tested against three real files
               that reproducibly blank the Acrobat model tree via this exact
               path (davidgbarnes-submitted-version.stream-90,
               Walnut_Viewport3_PDF3D.stream-64, QCD_Leinweber_
               ActionXs24t36black_Anim_r9796.stream-219) -- all three still
               blanked the tree identically, alone and combined with the
               Cramer's-rule basis experiment above. Tolerance margin is not
               the cause of this bug family. Gated behind
               PRC_DIAG_HALVE_LUMPED_TOLERANCE so default behavior (years of
               validated real-file output at the unhalved tolerance) is
               unaffected; kept as a diagnostic in case a narrower
               investigation wants it later. */
            double effective_fraction = weld_tolerance_fraction;
            if (prc_diag_getenv("PRC_DIAG_HALVE_LUMPED_TOLERANCE") != NULL)
                effective_fraction *= 0.5;
            tess->tolerance = prc_write_tol_relative(effective_fraction);
        }

        /* MITIGATION (2026-07-27, narrowed same day): this lump path
           (forced either by the >100-part threshold or by the {3,7,14}
           known-bad-tessellation-count list above) used to skip the
           non-manifold-fan check entirely -- it's only performed in
           stl_import_build_parts' per-component loop below, which this
           path bypasses via the early `goto cleanup` above. Confirmed via
           a real prc-db file (2368549.stream-147.stl, whose 3 parts hit
           the known-bad-count lump specifically): 2 of its 3 parts DO
           contain non-manifold vertices and would have gotten the safe
           TRIANGLES fallback on the normal per-part path, but the lumped
           lookup order let a real Acrobat blank-tree defect back in
           through this gap.

           Only run this check when `check_nonmanifold` is set (i.e. only
           for the {3,7,14} known-bad-count lump, never for the ordinary
           >100-part threshold lump) -- confirmed the SAME day this was
           first tried unconditionally: 3D-PDF-Sample-Carburetor.stream-
           48.stl (759 real parts, lumped only by the >100 threshold, was
           already real-Acrobat-confirmed working as a plain COMPRESSED
           lump before this whole investigation) has its own non-manifold
           vertices too -- but they're ordinary touching-part assembly
           junctions (759 separate solids welded into one merge will
           routinely touch at shared points), not the single-part
           "disconnected fan" mesh-quality defect this check was designed
           to catch. Forcing THAT case to TRIANGLES-with-recalculated-
           normals produced a NEW, real Acrobat blank-tree/blank-canvas
           failure of its own on this file -- i.e. the "fix" was a net
           regression for the >100-threshold case, trading a theoretical
           risk for a confirmed one. Restricting to check_nonmanifold
           keeps the >100-threshold lump's previously-validated behavior
           completely untouched. */
        if (check_nonmanifold && prc_diag_getenv("PRC_DIAG_DISABLE_NONMANIFOLD_FALLBACK") == NULL &&
            nonmanifold_check_ctx != NULL && prc_api_mesh_has_nonmanifold_fans(nonmanifold_check_ctx,
                combined_positions, num_welded, combined_tri_indices, mesh->num_triangles, tess->tolerance) > 0)
        {
            uint32_t *one_face = (uint32_t *)malloc(sizeof(uint32_t));
            if (one_face == NULL)
            {
                fprintf(stderr, "Error: allocation failed checking lumped model for non-manifold fans\n");
                goto cleanup;
            }
            one_face[0] = mesh->num_triangles;
            fprintf(stderr, "Note: lumped model contains a non-manifold vertex (disconnected triangle fans) -- "
                "writing it uncompressed to avoid a known Acrobat compatibility issue.\n");
            tess->kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
            tess->face_tri_counts = one_face;
            tess->num_faces = 1;
        }

        if (original_normals)
        {
            tess->normals = combined_normals;
            tess->num_normals = max_normal_id;
            tess->norm_indices = combined_norm_indices;
            if (tess->kind == PRC_API_WRITE_TESS_KIND_TRIANGLES)
                tess->must_calculate_normals = 0;
        }
        else
        {
            tess->normals = NULL;
            tess->num_normals = 0;
            tess->norm_indices = NULL;
            /* TRIANGLES (the non-manifold-fan-fallback branch above) only
               recalculates normals from geometry/crease_angle_degrees when
               must_calculate_normals is explicitly set (include/prc_api.h) --
               unlike COMPRESSED, which always reconstructs from NULL
               normals regardless of this flag. Missing this line left the
               fallback entry with no usable normal data at all: confirmed
               (2026-07-27) via real Acrobat testing on 2368549.stream-147.stl
               and 3D-PDF-Sample-Carburetor.stream-48.stl -- both loaded and
               displayed geometry, but with all shading/shape definition
               gone flat. */
            if (tess->kind == PRC_API_WRITE_TESS_KIND_TRIANGLES)
                tess->must_calculate_normals = 1;
            tess->crease_angle_degrees = 30.0;
            if (prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES") != NULL)
                tess->crease_angle_degrees = atof(prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES"));
        }
    }

    parts->rep_items[0].kind = PRC_API_WRITE_RI_SURFACE;
    parts->rep_items[0].biased_tessellation_index = 1;
    parts->rep_items[0].is_closed = 0; /* STL doesn't reliably signal watertightness; stay conservative */

    if (mesh->solid_name[0] != '\0')
        stl_sanitize_name(mesh->solid_name, parts->names[0], sizeof(parts->names[0]), "parts");
    else
        snprintf(parts->names[0], sizeof(parts->names[0]), "parts");

    {
        prc_api_write_node *node = &parts->children[0];
        int a;
        memset(node, 0, sizeof(*node));
        node->rep_items = &parts->rep_items[0];
        node->num_rep_items = 1;
        node->name = parts->names[0];
        node->part_name = parts->names[0];
        /* Padded (not exactly-tight) bbox over the WHOLE mesh, same
           50%-per-axis padding as stl_import_build_parts uses per part. */
        for (a = 0; a < 3; a++)
        {
            double mid = 0.5 * (bbox_min[a] + bbox_max[a]);
            double half = 0.5 * (bbox_max[a] - bbox_min[a]);
            if (half < 1e-6) half = 1e-6;
            node->bbox_min[a] = mid - half * 1.5;
            node->bbox_max[a] = mid + half * 1.5;
        }
    }
    parts->child_ptrs[0] = &parts->children[0];

    {
        prc_api_write_node *intermediate = &parts->intermediates[0];
        memset(intermediate, 0, sizeof(*intermediate));
        intermediate->has_empty_part = 1;
        intermediate->children = &parts->child_ptrs[0];
        intermediate->num_children = 1;
    }
    parts->intermediate_ptrs[0] = &parts->intermediates[0];

    ok = 0;
    /* Ownership of these transferred into tess_entries[0] on success --
       stl_parts_free (Section 4's own, unmodified) frees them from there;
       NULL them out here so this function's own cleanup: below doesn't
       double-free on the success path. */
    combined_positions = NULL;
    combined_tri_indices = NULL;
    combined_normals = NULL;
    combined_norm_indices = NULL;

cleanup:
    free(combined_tri_indices);
    free(combined_positions);
    free(combined_normals);
    free(combined_norm_indices);
    if (nonmanifold_check_ctx != NULL)
        prc_api_release_context(nonmanifold_check_ctx);
    if (ok != 0)
        stl_parts_free(parts);
    return ok;
}

/* Builds one part (tessellation + rep item + tree node) per connected
   component. `weld_index` is 3*num_triangles welded vertex indices (one
   per triangle corner, from Section 2a); `component_of_welded` maps each
   welded vertex (0..num_welded-1) to its component id (0..num_components-1,
   from Section 2b's union-find); `global_normal_index`/`dedup_normals`
   (only used when original_normals is set) map each triangle to a
   deduplicated normal in `dedup_normals` (Section 3). Returns 0 on
   success.

   Each component gets a 3-level tree shape under the caller's root:
   root [no part] -> intermediate [empty part, has_empty_part=1] -> leaf
   [the real part, in `children`] -- not the flatter root-owns-the-part-
   directly shape demos/teapot_write uses. tests/internal/standalone_
   grid_triangles.c's own header comment documents why: a part-less root
   with the real part attached directly one level down (no empty-part
   container in between) was found to cause Acrobat blank-canvas/blank-
   tree specifically for open-topology meshes (rep_item.is_closed == 0,
   which this importer always sets, below, since STL doesn't reliably
   signal watertightness) -- confirmed against this importer's own
   real-world dataset testing at STL-scan scale (tens of thousands of
   triangles), where teapot_write's shallower shape (fine at teapot's own
   ~4000-triangle scale) produced exactly that symptom in real Acrobat. */
static int
stl_import_build_parts(const stl_mesh *mesh, const double *welded_positions, uint32_t num_welded,
    const uint32_t *weld_index, const uint32_t *component_of_welded, uint32_t num_components,
    int original_normals, const uint32_t *global_normal_index, const double *dedup_normals,
    double weld_tolerance_fraction, stl_parts *parts)
{
    uint32_t *component_of_triangle = NULL;
    uint32_t *tri_count_per_component = NULL;
    uint32_t *component_offset = NULL;
    uint32_t *triangle_order = NULL;
    uint32_t *fill_cursor = NULL;
    int32_t *local_vertex_id = NULL;
    uint32_t *touched_vertices = NULL;
    int32_t *local_normal_id = NULL;
    uint32_t *touched_normals = NULL;
    uint32_t t, c;
    int ok = -1;
    prc_context *nonmanifold_check_ctx = prc_api_new_context(NULL);

    memset(parts, 0, sizeof(*parts));

    {
        uint32_t lump_threshold = STL_IMPORT_SINGLE_MODEL_PART_THRESHOLD;
        const char *ov = prc_diag_getenv("PRC_DIAG_LUMP_THRESHOLD");
        uint8_t force_lump = 0;
        /* MITIGATION (2026-07-26, mixed_chains/UK_original.stl Acrobat blank-tree
           investigation): a real, reproducible Acrobat defect blanks the model tree for specific
           TOTAL tessellation-entry counts, independent of content -- confirmed on trivial synthetic
           geometry at exactly these counts, with a dense sweep of ~25 other counts all working and
           no formula found relating them. Root cause NOT identified -- see project notes. Since
           there's no known way to predict other bad counts, this is a narrow, incomplete
           empirical list of only the counts directly confirmed bad, not a general solution -- a
           multi-part file whose count isn't in this list could still turn out to be bad; add to
           this list if/when another one is found. Forcing every multi-part file through the
           lumped-model path unconditionally (matching the existing 100+-part threshold's own
           tree shape) would sidestep this class entirely, but a working file with real multi-part
           semantics (separately selectable parts in a PDF3D viewer) would lose that structure --
           deemed not worth the tradeoff for a bug this narrow (~3 of ~28 counts tested) until more
           is known. */
        static const uint32_t known_bad_tess_counts[] = { 3, 7, 14 };
        if (ov != NULL)
            lump_threshold = (uint32_t)atoi(ov);
        else
        {
            size_t bi;
            for (bi = 0; bi < sizeof(known_bad_tess_counts) / sizeof(known_bad_tess_counts[0]); bi++)
            {
                if (num_components == known_bad_tess_counts[bi])
                {
                    force_lump = 1;
                    break;
                }
            }
        }
        if (num_components > lump_threshold || force_lump)
        {
            ok = stl_import_build_single_lumped_model(mesh, welded_positions, num_welded, weld_index,
                original_normals, global_normal_index, dedup_normals, weld_tolerance_fraction, force_lump, parts);
            goto cleanup;
        }
    }

    component_of_triangle = (uint32_t *)malloc(sizeof(uint32_t) * mesh->num_triangles);
    tri_count_per_component = (uint32_t *)calloc(num_components, sizeof(uint32_t));
    component_offset = (uint32_t *)malloc(sizeof(uint32_t) * (num_components + 1));
    triangle_order = (uint32_t *)malloc(sizeof(uint32_t) * mesh->num_triangles);
    fill_cursor = (uint32_t *)malloc(sizeof(uint32_t) * num_components);
    local_vertex_id = (int32_t *)malloc(sizeof(int32_t) * num_welded);
    touched_vertices = (uint32_t *)malloc(sizeof(uint32_t) * num_welded);
    if (component_of_triangle == NULL || tri_count_per_component == NULL || component_offset == NULL ||
        triangle_order == NULL || fill_cursor == NULL || local_vertex_id == NULL || touched_vertices == NULL)
    {
        fprintf(stderr, "Error: allocation failed grouping triangles by part\n");
        goto cleanup;
    }
    for (t = 0; t < num_welded; t++)
        local_vertex_id[t] = -1;

    if (original_normals)
    {
        /* dedup_normals' count isn't passed in directly, but every entry
           global_normal_index[t] references is < that count by
           construction (Section 3); the caller sizes these off the same
           dedup structure, so this is safe. */
        uint32_t max_normal_id = 0;
        for (t = 0; t < mesh->num_triangles; t++)
            if (global_normal_index[t] + 1 > max_normal_id) max_normal_id = global_normal_index[t] + 1;
        local_normal_id = (int32_t *)malloc(sizeof(int32_t) * (max_normal_id > 0 ? max_normal_id : 1));
        touched_normals = (uint32_t *)malloc(sizeof(uint32_t) * (max_normal_id > 0 ? max_normal_id : 1));
        if (local_normal_id == NULL || touched_normals == NULL)
        {
            fprintf(stderr, "Error: allocation failed for normal deduplication\n");
            goto cleanup;
        }
        for (t = 0; t < max_normal_id; t++)
            local_normal_id[t] = -1;
    }

    /* Counting sort: group triangle indices contiguously by component in
       one O(num_triangles + num_components) pass, so each component's
       triangles are triangle_order[component_offset[c] .. component_offset[c+1]-1]. */
    for (t = 0; t < mesh->num_triangles; t++)
    {
        component_of_triangle[t] = component_of_welded[weld_index[t * 3 + 0]];
        tri_count_per_component[component_of_triangle[t]]++;
    }
    component_offset[0] = 0;
    for (c = 0; c < num_components; c++)
        component_offset[c + 1] = component_offset[c] + tri_count_per_component[c];
    memcpy(fill_cursor, component_offset, sizeof(uint32_t) * num_components);
    for (t = 0; t < mesh->num_triangles; t++)
        triangle_order[fill_cursor[component_of_triangle[t]]++] = t;

    parts->tess_entries = (prc_api_write_tessellation *)calloc(num_components, sizeof(prc_api_write_tessellation));
    parts->rep_items = (prc_api_write_rep_item *)calloc(num_components, sizeof(prc_api_write_rep_item));
    parts->children = (prc_api_write_node *)calloc(num_components, sizeof(prc_api_write_node));
    parts->child_ptrs = (prc_api_write_node **)calloc(num_components, sizeof(prc_api_write_node *));
    parts->intermediates = (prc_api_write_node *)calloc(num_components, sizeof(prc_api_write_node));
    parts->intermediate_ptrs = (prc_api_write_node **)calloc(num_components, sizeof(prc_api_write_node *));
    parts->names = (char (*)[64])calloc(num_components, sizeof(char[64]));
    if (parts->tess_entries == NULL || parts->rep_items == NULL || parts->children == NULL ||
        parts->child_ptrs == NULL || parts->intermediates == NULL || parts->intermediate_ptrs == NULL ||
        parts->names == NULL)
    {
        fprintf(stderr, "Error: allocation failed building part list\n");
        goto cleanup;
    }
    parts->num_components = num_components;

    for (c = 0; c < num_components; c++)
    {
        uint32_t begin = component_offset[c], end = component_offset[c + 1];
        uint32_t tri_count = end - begin;
        uint32_t touched_v_count = 0, touched_n_count = 0;
        uint32_t local_vertex_count = 0, local_normal_count = 0;
        double comp_bbox_min[3], comp_bbox_max[3];
        /* Upper bound: a component can have at most 3 distinct local
           vertices per triangle (welding only ever reduces this), so
           over-allocating to that bound and using only the first
           local_vertex_count entries avoids a realloc-as-we-go dance --
           tess_entries[c].num_positions below tells the write API exactly
           how many of these are real. */
        double *local_positions = (double *)malloc(sizeof(double) * 3 * tri_count * 3);
        uint32_t *local_tri_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * tri_count);
        double *local_normals = NULL;
        uint32_t *local_norm_indices = NULL;
        uint32_t k;

        if (local_positions == NULL || local_tri_indices == NULL)
        {
            fprintf(stderr, "Error: allocation failed building part %u\n", (unsigned)(c + 1));
            free(local_positions); free(local_tri_indices);
            goto cleanup;
        }
        if (original_normals)
        {
            local_normals = (double *)malloc(sizeof(double) * 3 * tri_count * 3);
            local_norm_indices = (uint32_t *)malloc(sizeof(uint32_t) * 3 * tri_count);
            if (local_normals == NULL || local_norm_indices == NULL)
            {
                fprintf(stderr, "Error: allocation failed building part %u normals\n", (unsigned)(c + 1));
                free(local_positions); free(local_tri_indices);
                free(local_normals); free(local_norm_indices);
                goto cleanup;
            }
        }

        bbox_reset(comp_bbox_min, comp_bbox_max);

        for (k = begin; k < end; k++)
        {
            uint32_t tri = triangle_order[k];
            uint32_t out_tri = k - begin;
            int corner;

            for (corner = 0; corner < 3; corner++)
            {
                uint32_t wv = weld_index[tri * 3 + corner];
                int32_t lid = local_vertex_id[wv];
                if (lid < 0)
                {
                    lid = (int32_t)local_vertex_count;
                    local_vertex_id[wv] = lid;
                    touched_vertices[touched_v_count++] = wv;
                    local_positions[lid * 3 + 0] = welded_positions[wv * 3 + 0];
                    local_positions[lid * 3 + 1] = welded_positions[wv * 3 + 1];
                    local_positions[lid * 3 + 2] = welded_positions[wv * 3 + 2];
                    bbox_expand(comp_bbox_min, comp_bbox_max, &welded_positions[wv * 3]);
                    local_vertex_count++;
                }
                local_tri_indices[out_tri * 3 + corner] = (uint32_t)lid;
            }

            if (original_normals)
            {
                uint32_t gn = global_normal_index[tri];
                int32_t nid = local_normal_id[gn];
                if (nid < 0)
                {
                    nid = (int32_t)local_normal_count;
                    local_normal_id[gn] = nid;
                    touched_normals[touched_n_count++] = gn;
                    memcpy(&local_normals[(uint32_t)nid * 3], &dedup_normals[gn * 3], sizeof(double) * 3);
                    local_normal_count++;
                }
                local_norm_indices[out_tri * 3 + 0] = (uint32_t)nid;
                local_norm_indices[out_tri * 3 + 1] = (uint32_t)nid;
                local_norm_indices[out_tri * 3 + 2] = (uint32_t)nid;
            }
        }

        /* Reset only the entries this component touched, not the whole
           num_welded-sized scratch array -- keeps the total cost across
           all components O(num_triangles), not O(num_welded * num_components). */
        for (k = 0; k < touched_v_count; k++)
            local_vertex_id[touched_vertices[k]] = -1;
        for (k = 0; k < touched_n_count; k++)
            local_normal_id[touched_normals[k]] = -1;

        /* Shrink the position/normal buffers from their 3-per-triangle
           upper bound down to what welding/deduplication actually used --
           on a well-connected mesh (the common case) local_vertex_count is
           typically a small fraction of 3*tri_count, and at the >1M-
           triangle scale this demo is meant to handle, that difference is
           real memory, not noise. A failed shrink (new pointer NULL) just
           means keeping the original, larger-than-needed block; realloc
           leaves the original allocation untouched in that case, so it's
           always safe to fall back to it rather than treating this as a
           fatal error. */
        {
            double *shrunk = (double *)realloc(local_positions, sizeof(double) * 3 * (local_vertex_count > 0 ? local_vertex_count : 1));
            if (shrunk != NULL) local_positions = shrunk;
        }
        if (original_normals)
        {
            double *shrunk = (double *)realloc(local_normals, sizeof(double) * 3 * (local_normal_count > 0 ? local_normal_count : 1));
            if (shrunk != NULL) local_normals = shrunk;
        }

        {
            prc_api_write_tessellation *tess = &parts->tess_entries[c];
            prc_write_tolerance comp_tolerance = prc_write_tol_relative(weld_tolerance_fraction);
            /* COMPRESSED (not TRIANGLES): welding, degenerate-triangle
               removal, and EdgeBreaker-style traversal compression are all
               built in, giving meaningfully smaller output than TRIANGLES
               at the multi-million-triangle scale this importer targets.
               See Section 2's own comment for why this demo still does its
               own separate weld pass despite COMPRESSED welding internally
               too (short version: connected-component splitting needs an
               explicit adjacency graph the internal weld doesn't expose),
               and include/prc_api.h's prc_api_write_tess_kind_t doc comment
               for why COMPRESSED, once flagged there as having an
               independent-reader conformance bug, is now confirmed solid. */
            tess->kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
            tess->positions = local_positions;
            tess->num_positions = local_vertex_count;
            tess->tri_indices = local_tri_indices;
            tess->num_triangles = tri_count;
            /* face_tri_counts/num_faces left at 0/NULL: legal for COMPRESSED
               ("whole entry treated as one face", include/prc_api.h) and
               there is no meaningful face grouping to preserve from STL in
               the first place -- unlike TRIANGLES, COMPRESSED's own normal
               reconstruction (below) operates over the whole entry's
               connectivity, not per face group, so there is nothing this
               importer would gain from allocating and filling that array. */
            /* Re-weld at the SAME tolerance already used to build
               local_positions above -- redundant (points are already at
               least that far apart or exactly coincident) but harmless,
               and keeps this entry's own encoder-side tolerance consistent
               with what actually produced its vertex data, in case a
               future caller inspects tess->tolerance. */
            tess->tolerance = comp_tolerance;

            /* WORKAROUND (2026-07-26, mixed_chains/UK_original.stl/beetle_
               1000000.stl Acrobat blank-tree bug -- see the project memory
               writeup): COMPRESSED's own non-manifold vertex-splitting has
               an unresolved, real-Acrobat-confirmed compatibility issue
               specifically for vertices touched by disconnected triangle
               "fans" (a real, if uncommon, real-world artifact -- typically
               a duplicated-vertex/T-junction quirk from the mesh's source
               tool). Root cause not identified despite extensive
               investigation (nanoPRC's own reader, and at least one
               independent PRC reader, both decode the affected COMPRESSED
               output correctly -- some real Acrobat builds still blank the
               model tree). Falling back to TRIANGLES for just the affected
               component keeps COMPRESSED's size win for the (overwhelming
               majority of) components that don't hit this, at the cost of
               larger output for the rare component that does. */
            if (prc_diag_getenv("PRC_DIAG_DISABLE_NONMANIFOLD_FALLBACK") == NULL &&
                nonmanifold_check_ctx != NULL && prc_api_mesh_has_nonmanifold_fans(nonmanifold_check_ctx,
                    local_positions, local_vertex_count, local_tri_indices, tri_count, comp_tolerance) > 0)
            {
                uint32_t *one_face = (uint32_t *)malloc(sizeof(uint32_t));
                if (one_face == NULL)
                {
                    fprintf(stderr, "Error: allocation failed building part %u\n", (unsigned)(c + 1));
                    free(local_positions); free(local_tri_indices);
                    if (local_normals) free(local_normals);
                    if (local_norm_indices) free(local_norm_indices);
                    goto cleanup;
                }
                one_face[0] = tri_count;
                fprintf(stderr, "Note: part %u contains a non-manifold vertex (disconnected triangle fans) -- "
                    "writing this component uncompressed to avoid a known Acrobat compatibility issue.\n",
                    (unsigned)(c + 1));
                tess->kind = PRC_API_WRITE_TESS_KIND_TRIANGLES;
                tess->face_tri_counts = one_face;
                tess->num_faces = 1;

                /* DIAGNOSTIC (2026-07-26, mixed_chains investigation): also
                   fully expand vertices (no index sharing AT ALL between
                   any triangles, matching what a third-party PDF3D-SDK-
                   based converter's own uncompressed output does for the
                   same geometry -- see project memory) instead of using
                   this component's normally-welded/shared buffer, gated by
                   PRC_DIAG_EXPAND_TRIANGLES so the default behavior above
                   (still deduplicated) is unaffected until this is proven
                   causal. Zero behavior change when unset. */
                if (prc_diag_getenv("PRC_DIAG_EXPAND_TRIANGLES") != NULL)
                {
                    uint32_t exp_count = tri_count * 3;
                    double *exp_positions = (double *)malloc(sizeof(double) * 3 * exp_count);
                    uint32_t *exp_indices = (uint32_t *)malloc(sizeof(uint32_t) * exp_count);
                    uint32_t tt, cc;
                    if (exp_positions == NULL || exp_indices == NULL)
                    {
                        fprintf(stderr, "Error: allocation failed expanding part %u\n", (unsigned)(c + 1));
                        free(exp_positions); free(exp_indices);
                        free(local_positions); free(local_tri_indices);
                        if (local_normals) free(local_normals);
                        if (local_norm_indices) free(local_norm_indices);
                        goto cleanup;
                    }
                    for (tt = 0; tt < tri_count; tt++)
                    {
                        for (cc = 0; cc < 3; cc++)
                        {
                            uint32_t src = local_tri_indices[(size_t)tt * 3 + cc];
                            uint32_t dst = tt * 3 + cc;
                            memcpy(&exp_positions[(size_t)dst * 3], &local_positions[(size_t)src * 3], sizeof(double) * 3);
                            exp_indices[dst] = dst;
                        }
                    }
                    tess->positions = exp_positions;
                    tess->num_positions = exp_count;
                    tess->tri_indices = exp_indices;
                }
                if (original_normals)
                {
                    tess->normals = local_normals;
                    tess->num_normals = local_normal_count;
                    tess->norm_indices = local_norm_indices;
                    tess->must_calculate_normals = 0;
                }
                else
                {
                    tess->normals = NULL;
                    tess->num_normals = 0;
                    tess->norm_indices = NULL;
                    tess->must_calculate_normals = 1;
                    tess->crease_angle_degrees = 30.0;
                    if (prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES") != NULL)
                        tess->crease_angle_degrees = atof(prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES"));
                }
                goto tess_entry_done;
            }

            if (original_normals)
            {
                tess->normals = local_normals;
                tess->num_normals = local_normal_count;
                tess->norm_indices = local_norm_indices;
            }
            else
            {
                /* --original-normals is off (the default): no normal data
                   at all -- COMPRESSED reconstructs smooth per-vertex
                   normals from this welded geometry itself when `normals`
                   is NULL. 30 degrees is this field's own documented
                   default (include/prc_api.h); set explicitly here rather
                   than relying on the 0-means-default behavior, so this
                   demo's chosen crease angle is visible at the call site. */
                tess->normals = NULL;
                tess->num_normals = 0;
                tess->norm_indices = NULL;
                tess->crease_angle_degrees = 30.0;
                if (prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES") != NULL)
                    tess->crease_angle_degrees = atof(prc_diag_getenv("PRC_DIAG_CREASE_ANGLE_DEGREES"));
            }
tess_entry_done:;
        }

        parts->rep_items[c].kind = PRC_API_WRITE_RI_SURFACE;
        parts->rep_items[c].biased_tessellation_index = c + 1;
        parts->rep_items[c].is_closed = 0; /* STL doesn't reliably signal watertightness; stay conservative */

        if (num_components == 1 && mesh->solid_name[0] != '\0')
            stl_sanitize_name(mesh->solid_name, parts->names[c], sizeof(parts->names[c]), "part_1");
        else
            snprintf(parts->names[c], sizeof(parts->names[c]), "part_%u", (unsigned)(c + 1));

        {
            prc_api_write_node *node = &parts->children[c];
            int a;
            memset(node, 0, sizeof(*node));
            node->rep_items = &parts->rep_items[c];
            node->num_rep_items = 1;
            node->name = parts->names[c];
            node->part_name = parts->names[c];
            /* Padded (not exactly-tight) bbox, same 50%-per-axis padding as
               demos/teapot_write -- real-world PRC producers' PartDefinition
               boxes are rarely mesh-exact either. */
            for (a = 0; a < 3; a++)
            {
                double mid = 0.5 * (comp_bbox_min[a] + comp_bbox_max[a]);
                double half = 0.5 * (comp_bbox_max[a] - comp_bbox_min[a]);
                if (half < 1e-6) half = 1e-6;
                node->bbox_min[a] = mid - half * 1.5;
                node->bbox_max[a] = mid + half * 1.5;
            }
        }
        parts->child_ptrs[c] = &parts->children[c];

        /* Empty-part intermediate level -- see this function's own doc
           comment for why. Unnamed (falls back to a generic label): it's
           a pure structural pass-through, not something meant to read as
           its own meaningful tree entry. */
        {
            prc_api_write_node *intermediate = &parts->intermediates[c];
            memset(intermediate, 0, sizeof(*intermediate));
            intermediate->has_empty_part = 1;
            intermediate->children = &parts->child_ptrs[c];
            intermediate->num_children = 1;
        }
        parts->intermediate_ptrs[c] = &parts->intermediates[c];
    }

    ok = 0;

cleanup:
    free(component_of_triangle);
    free(tri_count_per_component);
    free(component_offset);
    free(triangle_order);
    free(fill_cursor);
    free(local_vertex_id);
    free(touched_vertices);
    free(local_normal_id);
    free(touched_normals);
    if (nonmanifold_check_ctx != NULL)
        prc_api_release_context(nonmanifold_check_ctx);
    if (ok != 0)
        stl_parts_free(parts);
    return ok;
}

/* ---------------------------------------------------------------------
 * Section 5: CLI, orchestration, and the write + verify calls themselves
 * (mirroring demos/teapot_write's Step 5-7: new_context -> write_prc_buffer
 * -> write .prc -> pdf_embed_prc -> read both back as a self-check).
 * ------------------------------------------------------------------- */

/* Above this many triangles, the Step 7 self-check (reading both output
   files back and confirming they agree) is skipped by default -- see its
   own comment at the call site in main() for why: reading back a large
   COMPRESSED tessellation through prc_api_open_contents was measured, in
   this session, to cost far more than writing it in the first place
   (roughly quadratic in triangle count against this library's current
   decoder, confirmed by timing a 100K/300K/1M-triangle progression), to
   the point of turning a sub-second import into a multi-minute one for
   real multi-million-triangle STL files. That's a read-path performance
   characteristic of the existing library, not something this importer's
   own write-side code can fix -- so instead of always paying it, only
   pay it by default when it's cheap, and let --verify override the
   default in either direction. */
#define STL_VERIFY_DEFAULT_TRIANGLE_LIMIT 200000

static void
print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s <input.stl> [output.prc] [output.pdf] [--original-normals]\n"
        "                          [--weld-tolerance F] [--verify|--no-verify]\n\n"
        "  input.stl            Binary or ASCII STL file to import.\n"
        "  output.prc/.pdf      Default to input's own basename with .stl swapped for .prc/.pdf.\n"
        "  --original-normals   Use the STL file's own stored per-facet normals (flat\n"
        "                       per-facet shading) instead of the default: no stored\n"
        "                       normals at all, letting the reader compute smooth\n"
        "                       per-vertex normals from the welded geometry.\n"
        "  --weld-tolerance F   Vertex-welding tolerance, as a fraction of the mesh's\n"
        "                       bounding-box diagonal. Default 1e-6.\n"
        "  --verify              Force the read-back self-check (see below) even on a\n"
        "                       large mesh where it's skipped by default.\n"
        "  --no-verify           Skip the read-back self-check even on a small mesh\n"
        "                       where it would otherwise run by default.\n"
        "                       By default, the self-check runs for meshes up to\n"
        "                       %u triangles and is skipped above that (reading a\n"
        "                       large COMPRESSED tessellation back can take far\n"
        "                       longer than writing it -- see --verify's own note\n"
        "                       printed at import time when this applies).\n",
        prog, (unsigned)STL_VERIFY_DEFAULT_TRIANGLE_LIMIT);
}

/* Splits `input_path` into just its filename, without directory or
   extension (e.g. "models/part.stl" -> "part"), for deriving default
   output paths and the root tree node's name. */
static void
stl_basename_noext(const char *input_path, char *out, size_t out_cap)
{
    const char *slash = strrchr(input_path, '/');
    const char *bslash = strrchr(input_path, '\\');
    const char *base = input_path;
    const char *dot;
    size_t len;

    if (bslash != NULL && (slash == NULL || bslash > slash))
        base = bslash + 1;
    else if (slash != NULL)
        base = slash + 1;

    dot = strrchr(base, '.');
    len = dot != NULL ? (size_t)(dot - base) : strlen(base);
    if (len >= out_cap)
        len = out_cap - 1;
    memcpy(out, base, len);
    out[len] = '\0';
}

/* Opens `path` (either a plain .prc file or a 3D-annotated .pdf --
   prc_api_open_contents handles both transparently) and reports its
   total tessellation count, for the round-trip self-check in main(). */
static int
stl_import_count_tessellations(prc_context *ctx, const char *path, uint32_t *out_num_tess)
{
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts, num_products, num_markups, num_line_tess;
    uint32_t num_extra_geom_tess = 0;
    int code;

    data = prc_api_open_contents(ctx, path);
    if (data == NULL)
    {
        fprintf(stderr, "verification: prc_api_open_contents failed for %s\n", path);
        return -1;
    }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code == 0)
        code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code == 0)
        code = prc_api_get_number_tessellations(ctx, data, model_tree, out_num_tess, &num_line_tess, &num_extra_geom_tess);

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, model_tree);
    if (code != 0)
    {
        fprintf(stderr, "verification: failed reading back the model tree from %s\n", path);
        return -1;
    }
    return 0;
}

int
main(int argc, char *argv[])
{
    const char *input_path = NULL;
    char prc_path_buf[512], pdf_path_buf[512];
    const char *prc_path = NULL;
    const char *pdf_path = NULL;
    int original_normals = 0;
    double weld_tolerance_fraction = 1e-6;
    int verify_mode = 0; /* 0 = auto (size-based default), 1 = force on, -1 = force off */
    int positional = 0;
    int i;

    stl_mesh mesh;
    double bbox_min[3], bbox_max[3], diagonal;
    weld_grid grid;
    uint32_t *weld_index = NULL;
    uint32_t *parent = NULL;
    uint8_t *rank = NULL;
    uint32_t *component_of_root = NULL;
    uint32_t *component_of_welded = NULL;
    uint32_t num_components = 0;
    normal_dedup nd;
    uint32_t *global_normal_index = NULL;
    stl_parts parts;
    prc_api_write_node root;
    char root_name[256];
    prc_context *ctx = NULL;
    uint8_t *prc_buf = NULL;
    size_t prc_buf_size = 0;
    int exit_code = 1;
    int have_grid = 0, have_nd = 0, have_parts = 0;

    memset(&mesh, 0, sizeof(mesh));
    memset(&grid, 0, sizeof(grid));
    memset(&nd, 0, sizeof(nd));
    memset(&parts, 0, sizeof(parts));

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--original-normals") == 0)
        {
            original_normals = 1;
            continue;
        }
        if (strcmp(argv[i], "--weld-tolerance") == 0)
        {
            if (i + 1 >= argc)
            {
                print_usage(argv[0]);
                return 1;
            }
            weld_tolerance_fraction = atof(argv[++i]);
            continue;
        }
        if (strcmp(argv[i], "--verify") == 0)
        {
            verify_mode = 1;
            continue;
        }
        if (strcmp(argv[i], "--no-verify") == 0)
        {
            verify_mode = -1;
            continue;
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        switch (positional++)
        {
            case 0: input_path = argv[i]; break;
            case 1: prc_path = argv[i]; break;
            case 2: pdf_path = argv[i]; break;
            default:
                fprintf(stderr, "Error: unexpected extra argument '%s'\n", argv[i]);
                print_usage(argv[0]);
                return 1;
        }
    }
    if (input_path == NULL)
    {
        print_usage(argv[0]);
        return 1;
    }
    if (weld_tolerance_fraction <= 0.0)
    {
        fprintf(stderr, "Error: --weld-tolerance must be positive\n");
        return 1;
    }

    if (prc_path == NULL)
    {
        char base[256];
        stl_basename_noext(input_path, base, sizeof(base));
        snprintf(prc_path_buf, sizeof(prc_path_buf), "%s.prc", base);
        prc_path = prc_path_buf;
    }
    if (pdf_path == NULL)
    {
        char base[256];
        stl_basename_noext(input_path, base, sizeof(base));
        snprintf(pdf_path_buf, sizeof(pdf_path_buf), "%s.pdf", base);
        pdf_path = pdf_path_buf;
    }

    /* Step 1: parse the STL file (Section 1). */
    if (stl_mesh_load(input_path, &mesh) != 0)
        return 1;
    if (mesh.num_triangles == 0)
    {
        fprintf(stderr, "Error: %s contains no triangles\n", input_path);
        stl_mesh_free(&mesh);
        return 1;
    }
    printf("Parsed %s: %u triangles\n", input_path, (unsigned)mesh.num_triangles);

    /* TEST: PRC_DIAG_TRI_RANGE="START,END" -- restrict to a contiguous
       slice [START,END) of the RAW, file-order triangle stream, before any
       degenerate-filtering or welding. For bisecting which region of a
       large STL file (triangles are just a flat list, no spatial ordering
       guarantee) triggers an Acrobat rejection that survives all
       currently-known fixes -- halve/quarter the range repeatedly to
       narrow down the responsible triangles. Diagnostic only; not for
       production use (arbitrarily truncating a real mesh this way
       produces an open/non-representative sub-mesh). */
    if (prc_diag_getenv("PRC_DIAG_TRI_RANGE") != NULL)
    {
        char spec[128];
        uint32_t start, end;
        strncpy(spec, prc_diag_getenv("PRC_DIAG_TRI_RANGE"), sizeof(spec) - 1);
        spec[sizeof(spec) - 1] = '\0';
        if (sscanf(spec, "%u,%u", &start, &end) == 2 && start < end && end <= mesh.num_triangles)
        {
            uint32_t new_n = end - start;
            memmove(mesh.raw_positions, &mesh.raw_positions[(size_t)start * 9], (size_t)new_n * 9 * sizeof(double));
            memmove(mesh.facet_normals, &mesh.facet_normals[(size_t)start * 3], (size_t)new_n * 3 * sizeof(double));
            mesh.num_triangles = new_n;
            printf("PRC_DIAG_TRI_RANGE=%u,%u: restricted to %u triangles\n", start, end, new_n);
        }
        else
        {
            fprintf(stderr, "PRC_DIAG_TRI_RANGE: invalid range '%s' (num_triangles=%u)\n", spec, (unsigned)mesh.num_triangles);
            stl_mesh_free(&mesh);
            return 1;
        }
    }

    /* Step 2: weld vertices + find connected components (Section 2). */
    bbox_reset(bbox_min, bbox_max);
    for (i = 0; i < (int)mesh.num_triangles * 3; i++)
        bbox_expand(bbox_min, bbox_max, &mesh.raw_positions[(size_t)i * 3]);
    {
        double ext[3];
        int a;
        for (a = 0; a < 3; a++) ext[a] = bbox_max[a] - bbox_min[a];
        diagonal = sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]);
        if (diagonal < 1e-9) diagonal = 1.0; /* degenerate (single-point) mesh guard */
    }

    /* Drop degenerate/sliver triangles before they reach welding or the
       write API -- see stl_filter_degenerate_triangles's own comment for
       why this matters beyond hygiene. area_threshold: a triangle is
       degenerate if its area is below (1e-9 * bbox diagonal)^2 -- three
       orders of magnitude tighter than the default vertex-weld tolerance
       (1e-6), so this alone only catches triangles degenerate at the
       mesh's own scale (collinear/coincident points, floating-point
       noise), not legitimately small detail triangles in a fine mesh.
       aspect_threshold (1e5) additionally catches extreme slivers with
       real but tiny area relative to their longest edge -- chosen well
       above any aspect ratio a normally-shaped mesh triangle would have,
       but well below the ~2e4-8e6 range observed on the real slivers that
       motivated this filter in the first place. */
    {
        double area_eps = 1e-9 * diagonal;
        uint32_t removed = stl_filter_degenerate_triangles(&mesh, area_eps * area_eps, 1e5);
        if (removed > 0)
            printf("Filtered %u degenerate (near-zero-area) triangle%s\n", (unsigned)removed, removed == 1 ? "" : "s");
        if (mesh.num_triangles == 0)
        {
            fprintf(stderr, "Error: %s contains no non-degenerate triangles\n", input_path);
            goto cleanup;
        }
    }

    /* MITIGATION (2026-07-26/27, mixed_chains/beetle_1000000.stl Acrobat blank-tree
       investigation): jitters RAW, pre-weld positions by a small, deterministic,
       bbox/tolerance-relative amount before this importer's own weld pass runs. Real,
       reproducible, but still-unidentified Acrobat defect: two independent traversal chains
       sharing one COMPRESSED entry can, for specific EXACT real-world vertex values, blank the
       model tree (nanoPRC's own reader, and at least one independent PRC reader, both already
       decode the un-jittered geometry correctly, so this is Acrobat-side, not a correctness bug
       here). Root cause not identified -- this is an empirical, BEST-EFFORT mitigation, not a
       guaranteed fix.

       IMPORTANT, hard-won limitation: the magnitude that actually escapes a given file's own
       coincidence appears to sit in a NARROW window, not a broad "big enough" range --
       confirmed via a controlled A/B on beetle_1000000.stl where exactly 1e-2 (absolute) works
       but 1.1548e-2 (only ~15% more, from an earlier, more "principled"-looking 10x-weld-
       tolerance formula) does NOT. PRC_ENCODE_JITTER_TOLERANCE_FACTOR below is therefore NOT a
       theoretically-derived safety margin -- it's reverse-engineered from the one value
       confirmed (via real Acrobat testing, with the redundant downstream jitter in
       prc_encode_preprocess correctly disabled to isolate this specific mechanism) to work,
       expressed as a multiple of weld tolerance so it at least scales with the mesh's own size
       rather than being a fixed mm constant. There is no guarantee this SAME multiple lands in
       whatever narrow window a DIFFERENT file's own coincidence has; a file that still fails at
       this default should be retried with a different PRC_DIAG_PREWELD_JITTER_MAG value (or
       PRC_DIAG_JITTER_SEED on prc_encode_preprocess's own independent, downstream jitter
       mechanism) rather than assumed unfixable.

       Exact magnitude and float32-truncation-before-use (below) were both confirmed load-
       bearing via the same controlled A/B: jittering AFTER this importer's own weld (inside
       prc_encode_preprocess, downstream) does NOT reproduce this result, only jittering the RAW
       pre-weld positions here does, because only here can jitter influence WHICH near-duplicate
       (but not exactly coincident) raw corners end up welding together -- a real topological
       effect no later-stage jitter can produce. PRC_DIAG_PREWELD_JITTER_MAG overrides the
       automatic magnitude (absolute units) for testing; PRC_DIAG_DISABLE_PREWELD_JITTER
       disables this mitigation entirely. */
    {
        uint8_t preweld_disabled = (prc_diag_getenv("PRC_DIAG_DISABLE_PREWELD_JITTER") != NULL);
        const char *preweld_env = prc_diag_getenv("PRC_DIAG_PREWELD_JITTER_MAG");
        if (!preweld_disabled)
        {
            double mag = (preweld_env != NULL) ? atof(preweld_env) : (weld_tolerance_fraction * diagonal * 8.65969267656085);
            uint32_t vi;
            for (vi = 0; vi < mesh.num_triangles * 3; vi++)
            {
                double *pv = &mesh.raw_positions[(size_t)vi * 3];
                float fx = (float)pv[0], fy = (float)pv[1], fz = (float)pv[2];
                uint8_t in[12], digest[16];
                uint32_t hx, hy, hz;

                memcpy(in + 0, &fx, 4);
                memcpy(in + 4, &fy, 4);
                memcpy(in + 8, &fz, 4);
                prc_md5_12bytes(in, digest);
                hx = (uint32_t)digest[0] | ((uint32_t)digest[1] << 8) | ((uint32_t)digest[2] << 16) | ((uint32_t)digest[3] << 24);
                hy = (uint32_t)digest[4] | ((uint32_t)digest[5] << 8) | ((uint32_t)digest[6] << 16) | ((uint32_t)digest[7] << 24);
                hz = (uint32_t)digest[8] | ((uint32_t)digest[9] << 8) | ((uint32_t)digest[10] << 16) | ((uint32_t)digest[11] << 24);
                pv[0] += (((double)hx / (double)0xFFFFFFFFu) * 2.0 - 1.0) * mag;
                pv[1] += (((double)hy / (double)0xFFFFFFFFu) * 2.0 - 1.0) * mag;
                pv[2] += (((double)hz / (double)0xFFFFFFFFu) * 2.0 - 1.0) * mag;
                /* Truncate to float32: an earlier, externally-scripted version of this
                   mitigation wrote the jittered result back into a binary STL file (float32-
                   only storage) before nanoPRC ever read it, so the value actually used was
                   float32-rounded, not full double precision. Confirmed empirically to matter:
                   without this truncation, the exact same hash/magnitude/application-point
                   combination (verified byte-identical to the working script's own computation
                   before this rounding step) still failed on beetle_1000000.stl. */
                pv[0] = (double)(float)pv[0];
                pv[1] = (double)(float)pv[1];
                pv[2] = (double)(float)pv[2];
                if (vi == 0 && prc_diag_getenv("PRC_DIAG_JITTER_DEBUG") != NULL)
                    fprintf(stderr, "PRC_DIAG_JITTER_DEBUG orig=(%.17g,%.17g,%.17g) jittered=(%.17g,%.17g,%.17g)\n",
                        (double)fx, (double)fy, (double)fz, pv[0], pv[1], pv[2]);
            }
        }
    }

    if (weld_grid_init(&grid, weld_tolerance_fraction * diagonal, mesh.num_triangles * 3) != 0)
    {
        fprintf(stderr, "Error: allocation failed initializing the weld grid\n");
        goto cleanup;
    }
    have_grid = 1;

    weld_index = (uint32_t *)malloc(sizeof(uint32_t) * (size_t)mesh.num_triangles * 3);
    if (weld_index == NULL)
    {
        fprintf(stderr, "Error: allocation failed for weld index\n");
        goto cleanup;
    }
    for (i = 0; i < (int)mesh.num_triangles * 3; i++)
    {
        uint32_t wv = weld_grid_lookup_or_insert(&grid, &mesh.raw_positions[(size_t)i * 3], weld_tolerance_fraction * diagonal);
        if (wv == UINT32_MAX)
        {
            fprintf(stderr, "Error: allocation failed welding vertices\n");
            goto cleanup;
        }
        weld_index[i] = wv;
    }
    printf("Welded to %u vertices (tolerance %.3g x bbox diagonal)\n", (unsigned)grid.count, weld_tolerance_fraction);

    parent = (uint32_t *)malloc(sizeof(uint32_t) * grid.count);
    rank = (uint8_t *)calloc(grid.count, sizeof(uint8_t));
    if (parent == NULL || rank == NULL)
    {
        fprintf(stderr, "Error: allocation failed for connectivity analysis\n");
        goto cleanup;
    }
    for (i = 0; i < (int)grid.count; i++)
        parent[i] = (uint32_t)i;
    for (i = 0; i < (int)mesh.num_triangles; i++)
    {
        uf_union(parent, rank, weld_index[i * 3 + 0], weld_index[i * 3 + 1]);
        uf_union(parent, rank, weld_index[i * 3 + 1], weld_index[i * 3 + 2]);
    }

    component_of_root = (uint32_t *)malloc(sizeof(uint32_t) * grid.count);
    component_of_welded = (uint32_t *)malloc(sizeof(uint32_t) * grid.count);
    if (component_of_root == NULL || component_of_welded == NULL)
    {
        fprintf(stderr, "Error: allocation failed labeling connected components\n");
        goto cleanup;
    }
    for (i = 0; i < (int)grid.count; i++)
        component_of_root[i] = UINT32_MAX;
    for (i = 0; i < (int)grid.count; i++)
    {
        uint32_t r = uf_find(parent, (uint32_t)i);
        if (component_of_root[r] == UINT32_MAX)
            component_of_root[r] = num_components++;
        component_of_welded[i] = component_of_root[r];
    }
    printf("Found %u connected part%s\n", (unsigned)num_components, num_components == 1 ? "" : "s");

    /* Diagnostic-only (not for production use): restrict output to a
       caller-chosen subset of the connected components just labeled
       above, by component index (0-based, in union-find-root-discovery
       order -- stable for a given input file/tolerance, not necessarily
       meaningful across different files or tolerances). For isolating
       which of several parts in a multi-part STL is responsible for an
       Acrobat-rejection defect: e.g. PRC_DIAG_COMPONENT_FILTER=0 keeps
       only the first-discovered component, dropping everything else,
       without needing to know which raw STL facet range it occupies (the
       way PRC_DIAG_TRI_RANGE would require) -- components need not be
       contiguous in facet order. Incompatible with --original-normals
       (global_normal_index is built below, indexed by the ORIGINAL
       per-triangle order, which this filter would desync from the
       now-compacted triangle list) -- rejected outright rather than
       silently producing wrong normals. */
    {
        const char *diag_component_filter = prc_diag_getenv("PRC_DIAG_COMPONENT_FILTER");
        if (diag_component_filter != NULL)
        {
            uint8_t *keep;
            uint32_t *new_id;
            uint32_t *weld_index_filtered;
            uint32_t kept_triangles = 0, kept_components = 0;
            char *filter_copy, *tok;
            uint32_t t, c;

            if (original_normals)
            {
                fprintf(stderr, "Error: PRC_DIAG_COMPONENT_FILTER is incompatible with --original-normals\n");
                goto cleanup;
            }

            keep = (uint8_t *)calloc(num_components, sizeof(uint8_t));
            new_id = (uint32_t *)malloc(sizeof(uint32_t) * num_components);
            weld_index_filtered = (uint32_t *)malloc(sizeof(uint32_t) * 3 * mesh.num_triangles);
            filter_copy = (char *)malloc(strlen(diag_component_filter) + 1);
            if (keep == NULL || new_id == NULL || weld_index_filtered == NULL || filter_copy == NULL)
            {
                fprintf(stderr, "Error: allocation failed for PRC_DIAG_COMPONENT_FILTER\n");
                free(keep); free(new_id); free(weld_index_filtered); free(filter_copy);
                goto cleanup;
            }
            strcpy(filter_copy, diag_component_filter);
            for (tok = strtok(filter_copy, ","); tok != NULL; tok = strtok(NULL, ","))
            {
                long idx = strtol(tok, NULL, 10);
                if (idx >= 0 && (uint32_t)idx < num_components) keep[idx] = 1;
            }
            free(filter_copy);
            for (c = 0; c < num_components; c++)
                new_id[c] = keep[c] ? kept_components++ : UINT32_MAX;

            for (t = 0; t < mesh.num_triangles; t++)
            {
                uint32_t comp = component_of_welded[weld_index[t * 3 + 0]];
                if (!keep[comp]) continue;
                weld_index_filtered[kept_triangles * 3 + 0] = weld_index[t * 3 + 0];
                weld_index_filtered[kept_triangles * 3 + 1] = weld_index[t * 3 + 1];
                weld_index_filtered[kept_triangles * 3 + 2] = weld_index[t * 3 + 2];
                /* Filtered-triangle order is exactly raw-facet-index order
                   restricted to kept components (this loop walks t in
                   ascending order and only ever appends), so with a SINGLE
                   kept component -- the common case for this diagnostic --
                   this filtered index is also exactly the local component-
                   relative triangle index prc_encode_preprocess will later
                   see as its own input order (its own tri_orig_index[]
                   diagnostic field indexes into THAT space). Printing this
                   mapping lets tri_orig_index values be traced all the way
                   back to a real raw STL facet index, e.g. for building a
                   precise (not reconstructed-from-rounded-text) minimal
                   repro via PRC_DIAG_TRI_RANGE on just that facet. */
                if (prc_diag_getenv("PRC_DIAG_DUMP_TRIANGLE_MAP") != NULL)
                    printf("PRC_DIAG_TRIANGLE_MAP: filtered_index=%u raw_facet_index=%u\n",
                        (unsigned)kept_triangles, (unsigned)t);
                kept_triangles++;
            }
            for (c = 0; c < grid.count; c++)
                component_of_welded[c] = keep[component_of_welded[c]] ? new_id[component_of_welded[c]] : UINT32_MAX;

            free(weld_index);
            weld_index = weld_index_filtered;
            mesh.num_triangles = kept_triangles;
            num_components = kept_components;
            free(keep);
            free(new_id);
            printf("PRC_DIAG_COMPONENT_FILTER=%s: restricted to %u triangles, %u component%s\n",
                diag_component_filter, (unsigned)kept_triangles, (unsigned)kept_components,
                kept_components == 1 ? "" : "s");
        }
    }

    /* Step 3: original-facet-normal deduplication, only if requested (Section 3). */
    if (original_normals)
    {
        if (normal_dedup_init(&nd, mesh.num_triangles) != 0)
        {
            fprintf(stderr, "Error: allocation failed initializing normal deduplication\n");
            goto cleanup;
        }
        have_nd = 1;
        global_normal_index = (uint32_t *)malloc(sizeof(uint32_t) * mesh.num_triangles);
        if (global_normal_index == NULL)
        {
            fprintf(stderr, "Error: allocation failed for normal indices\n");
            goto cleanup;
        }
        for (i = 0; i < (int)mesh.num_triangles; i++)
        {
            uint32_t ni = normal_dedup_get(&nd, &mesh.facet_normals[(size_t)i * 3]);
            if (ni == UINT32_MAX)
            {
                fprintf(stderr, "Error: allocation failed deduplicating normals\n");
                goto cleanup;
            }
            global_normal_index[i] = ni;
        }
    }

    /* Step 4: build the per-part tessellations + tree nodes (Section 4). */
    if (stl_import_build_parts(&mesh, grid.positions, grid.count, weld_index, component_of_welded, num_components,
            original_normals, global_normal_index, nd.normals, weld_tolerance_fraction, &parts) != 0)
        goto cleanup;
    have_parts = 1;

    /* Step 5: assemble the root node and write the file. Bare assembly
       root (no rep items of its own) with one empty-part intermediate per
       part, each owning one real leaf part -- see stl_import_build_parts'
       own comment for why this is a 3-level shape, not demos/teapot_
       write's shallower 2-level one (real-Acrobat-confirmed bug at STL-
       scan scale, not present at teapot's own small scale). */
    {
        char raw_name[256];
        stl_basename_noext(input_path, raw_name, sizeof(raw_name));
        stl_sanitize_name(raw_name, root_name, sizeof(root_name), "model");
    }
    memset(&root, 0, sizeof(root));
    root.name = root_name;
    root.children = parts.intermediate_ptrs;
    root.num_children = parts.num_components;

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "Error: prc_api_new_context failed\n");
        goto cleanup;
    }

    {
        int code;
        FILE *out;
        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, parts.tess_entries, parts.num_components,
            &prc_buf, &prc_buf_size);

        if (code != 0)
        {
            fprintf(stderr, "Error: prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            goto cleanup;
        }

        out = fopen(prc_path, "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            fprintf(stderr, "Error: failed to write %s\n", prc_path);
            if (out != NULL) fclose(out);
            goto cleanup;
        }
        fclose(out);
        printf("Wrote %s (%u bytes)\n", prc_path, (unsigned)prc_buf_size);

        /* Step 6: embed the same encoded bytes in a minimal 3D-annotated
           PDF, with one default view framed from the whole mesh's bounding
           box. Eye/target/up formula copied verbatim from demos/teapot_
           write's own Step 6 (see its comment for the distance-multiplier
           tuning history that arrived at these constants) -- no reason to
           re-derive a new, untested framing here. */
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;
            double center[3];
            int a;

            for (a = 0; a < 3; a++) center[a] = 0.5 * (bbox_min[a] + bbox_max[a]);

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            view.eye[0] = center[0] + diagonal * 1.0;
            view.eye[1] = center[1] - diagonal * 1.3;
            view.eye[2] = center[2] + diagonal * 0.7;
            view.target[0] = center[0];
            view.target[1] = center[1];
            view.target[2] = center[2];
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;

            code = prc_api_pdf_embed_prc(ctx, pdf_path, prc_buf, prc_buf_size, &pdf_opts);
            if (code != 0)
            {
                fprintf(stderr, "Error: prc_api_pdf_embed_prc failed (code %d)\n", code);
                prc_api_print_error_stack(ctx);
                goto cleanup;
            }
            printf("Wrote %s\n", pdf_path);
        }
    }

    /* Step 7: read both files back as a self-check, same pattern as
       demos/teapot_write's own Step 7 -- except, unlike that demo (which
       only ever writes a fixed ~2600-triangle teapot), gated by mesh size
       by default: see STL_VERIFY_DEFAULT_TRIANGLE_LIMIT's own comment for
       why reading a large COMPRESSED tessellation back can cost far more
       than writing it did, measured directly in this session to turn a
       sub-second import into a multi-minute one at real STL-scale
       triangle counts otherwise. --verify/--no-verify (verify_mode)
       override this default in either direction. */
    if (verify_mode > 0 || (verify_mode == 0 && mesh.num_triangles <= STL_VERIFY_DEFAULT_TRIANGLE_LIMIT))
    {
        uint32_t prc_tess = 0, pdf_tess = 0;
        if (stl_import_count_tessellations(ctx, prc_path, &prc_tess) != 0)
            goto cleanup;
        if (stl_import_count_tessellations(ctx, pdf_path, &pdf_tess) != 0)
            goto cleanup;
        if (prc_tess != pdf_tess)
        {
            fprintf(stderr, "verification: .prc and .pdf disagree on tessellation count (%u vs %u)\n",
                (unsigned)prc_tess, (unsigned)pdf_tess);
            goto cleanup;
        }
        printf("Verification: %u tessellation%s in both outputs\n", (unsigned)prc_tess, prc_tess == 1 ? "" : "s");
    }
    else if (verify_mode < 0)
    {
        printf("Verification skipped (--no-verify)\n");
    }
    else
    {
        printf("Verification skipped (%u triangles > %u default limit; pass --verify to force it)\n",
            (unsigned)mesh.num_triangles, (unsigned)STL_VERIFY_DEFAULT_TRIANGLE_LIMIT);
    }

    exit_code = 0;

cleanup:
    if (ctx != NULL && prc_buf != NULL)
        prc_api_write_prc_buffer_free(ctx, prc_buf);
    if (ctx != NULL)
        prc_api_release_context(ctx);
    if (have_parts)
        stl_parts_free(&parts);
    if (have_nd)
        normal_dedup_free(&nd);
    free(global_normal_index);
    free(component_of_root);
    free(component_of_welded);
    free(parent);
    free(rank);
    free(weld_index);
    if (have_grid)
        weld_grid_free(&grid);
    stl_mesh_free(&mesh);

    return exit_code;
}
