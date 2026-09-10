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

/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Walks the compressed tessellation entities of one or more raw .prc
   files and measures the two array-sizing conventions ISO 14739 leaves
   unstated, alongside the file header fields that have been proposed as
   predictors of them:

     - texture_parameters (Table 176 CompressedTextureParameter): element size
       and size-field unit -- floats or elements, pairs or triples.
     - edge_status_array (Table 175): whether it is sized T or 3T, where T is
       triangle_face_array_size.
     - per file: minimal_version_for_read, authoring_version, the last word of
       unique_id_file, and unique_id_application -- the candidate discriminators
       for the edge_status sizing.

   Three questions in one walker rather than three tools because they need the
   same expensive thing: an entity-level walk of the tessellation section that
   a corpus sweep runs 310 times.

   -- texture_parameters --------------------------------------------------

   WHY: ISO 14739 states neither. The uncompressed clause has the same gap and
   it is the subject of pdf-issues #810; the compressed clause needs its own
   measurement because a reader cannot infer one from the other. The
   uncompressed path carries a pre-multiplied index into a flat array, while
   the compressed path references UV parameters per vertex through
   binary_texture_data and reference_array and has no such index -- so the
   index-scaling argument that applies to the uncompressed clause has no
   counterpart here. What both clauses share is the unstated question this
   tool answers: is texture_parameters_size a count of floats or a count of
   coordinate elements, and are those elements pairs (u,v) or triples (u,v,w)?

   The element-size question is answered by divisibility, which needs no
   denominator and so cannot be argued with: if texture_parameters_size is even
   in every entity ever seen, the elements are pairs and the field counts
   floats. That is the load-bearing measurement here.

   The per-vertex question needs a denominator and does not have a single
   obvious one, which is why this tool reports three numbers rather than a
   ratio. An entity carries:

     fresh points   point_array_size / 3 -- positions stored outright
     total points   reference_array_size -- fresh plus those reused through
                    point_reference_array, i.e. every point the traversal
                    visits
     uv_refs        the texture block own reference_array_size, an index
                    stream into the UV pool

   texture_parameters is a pool, not a parallel array: uv_refs indexes into it.
   So tps/2 is the number of distinct UV pairs, which equals the fresh point
   count only when no position carries two UVs. A texture seam splits one
   position into two UVs and lifts tps/2 above it. Measuring against a single
   file where nothing is reused makes the two indistinguishable, so the summary
   counts how often tps/2 equals the fresh count and how often it merely lies
   between fresh and total -- and, separately, the cases that fall outside that
   bracket in either direction, which are the ones worth looking at.

   -- edge_status_array ---------------------------------------------------

   Both T and 3T occur in conforming files, with the tail of a 3T array
   uniformly zero; that much is settled (pdf-issues #727, CR-14a-d). What is
   not settled is whether anything in the file predicts which sizing a reader
   will meet. A per-writer rule was proposed and withdrawn -- a single
   application id emits files of both sizings -- so what this tool measures is
   the surviving candidate: minimal_version_for_read, taken as a compatibility
   floor declared per file rather than a producer habit. The summary reports
   the sizing broken down by each candidate discriminator, and counts files
   that MIX the two sizings internally, which is the observation that would
   sink the whole idea.

   -- header fields -------------------------------------------------------

   unique_id_application sits at byte offset 27 of the uncompressed header,
   after "PRC" (3), minimal_version_for_read (4), authoring_version (4) and
   unique_id_file (16). It is worth stating because a 32-bit word at offset 23
   -- the last word of unique_id_file -- also varies by producer lineage and is
   easy to mistake for it. This tool reports both so they cannot be confused.

   SCOPE, and one real limitation: this reads the tessellation section
   directly rather than through the public API, because the API materializes
   UV coordinates and does not expose the raw size field. It walks every file
   structure, and within each, tessellation entries in order. It cannot skip
   past a PRC_TYPE_TESS_3D (uncompressed) entry -- entries are not
   length-prefixed, so once an unparsed one is met the bit cursor is lost and
   the walk for that file structure stops there. Files are reported with what
   was reached rather than silently truncated: the summary counts how many
   files stopped early, so a low textured-entity count can be told apart from
   a corpus that genuinely has few.

   HOW: usage: compressed_tess_census.exe <file.prc> [more files...]
        compressed_tess_census.exe --csv <files...>       one row per entity
        compressed_tess_census.exe --files <files...>     one row per file
   Raw .prc only, not .pdf -- extract with extract_raw_prc first. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"
#include "prc_bit.h"
#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "zlib.h"

/* Sizing verdict for one entity or one file. */
#define SIZING_NONE   0
#define SIZING_T      1
#define SIZING_3T     2
#define SIZING_OTHER  3
#define SIZING_MIXED  4

/* How many distinct minimal_version_for_read values the summary will break the
   sizing down by before it gives up and lumps the rest together. Real corpora
   have shown three; ten leaves room without making the table unreadable. */
#define MAX_VERSION_ROWS 10

typedef struct
{
    uint32_t min_vers;
    uint32_t files_t;
    uint32_t files_3t;
    uint32_t files_mixed;
    uint32_t files_other;
    uint32_t entities_t;
    uint32_t entities_3t;
} version_row;

typedef struct
{
    uint32_t files_seen;
    uint32_t files_stopped_early;
    uint32_t entities_compressed;

    /* texture_parameters */
    uint32_t files_with_texture;
    uint32_t entities_textured;
    uint32_t tps_even;          /* texture_parameters_size divisible by 2   */
    uint32_t uv_eq_fresh;       /* tps/2 == fresh point count               */
    uint32_t uv_in_range;       /* fresh <= tps/2 <= total points           */
    uint32_t uv_below_fresh;    /* tps/2 <  fresh -- fewer UVs than points  */
    uint32_t uv_above_total;    /* tps/2 >  total points                    */

    /* edge_status_array */
    uint32_t entities_edge_t;
    uint32_t entities_edge_3t;
    uint32_t entities_edge_other;
    uint32_t files_edge_t;
    uint32_t files_edge_3t;
    uint32_t files_edge_mixed;

    version_row versions[MAX_VERSION_ROWS];
    uint32_t version_count;
    uint32_t versions_overflowed;

    /* unique_id_application partition */
    uint32_t files_app_null;
    uint32_t files_app_set;
} census;

/* Header fields of one file, read before any section is touched. */
typedef struct
{
    uint32_t min_vers;
    uint32_t auth_vers;
    uint32_t uid_file_word3;    /* byte offset 23 -- easily mistaken for the
                                   application id, so reported beside it */
    uint32_t app_id[4];         /* byte offset 27 */
    int app_id_null;
} file_header;

static uint32_t
read_le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static const char *
basename_of(const char *path)
{
    const char *slash = strrchr(path, '/');
    const char *back = strrchr(path, '\\');
    if (back != NULL && (slash == NULL || back > slash))
        slash = back;
    return (slash != NULL) ? slash + 1 : path;
}

/* "PRC" (3) + minimal_version_for_read (4) + authoring_version (4) +
   unique_id_file (16) + unique_id_application (16) + file_structure_count (4).
   Caller has already checked the buffer is at least 47 bytes. */
static void
read_file_header(const uint8_t *buf, file_header *h)
{
    int i;

    h->min_vers = read_le_u32(buf + 3);
    h->auth_vers = read_le_u32(buf + 7);
    h->uid_file_word3 = read_le_u32(buf + 23);
    h->app_id_null = 1;
    for (i = 0; i < 4; i++)
    {
        h->app_id[i] = read_le_u32(buf + 27 + 4 * i);
        if (h->app_id[i] != 0)
            h->app_id_null = 0;
    }
}

static void
format_app_id(const file_header *h, char *out, size_t cap)
{
    if (h->app_id_null)
    {
        snprintf(out, cap, "null");
        return;
    }
    snprintf(out, cap, "%08x-%08x-%08x-%08x",
        h->app_id[0], h->app_id[1], h->app_id[2], h->app_id[3]);
}

/* Fold a new entity verdict into a running per-file one. */
static int
merge_sizing(int running, int one)
{
    if (one == SIZING_NONE)
        return running;
    if (running == SIZING_NONE)
        return one;
    if (running == one)
        return running;
    return SIZING_MIXED;
}

static const char *
sizing_name(int s)
{
    switch (s)
    {
        case SIZING_T:     return "T";
        case SIZING_3T:    return "3T";
        case SIZING_OTHER: return "other";
        case SIZING_MIXED: return "MIXED";
        default:           return "none";
    }
}

static version_row *
version_row_for(census *c, uint32_t min_vers)
{
    uint32_t i;

    for (i = 0; i < c->version_count; i++)
        if (c->versions[i].min_vers == min_vers)
            return &c->versions[i];
    if (c->version_count >= MAX_VERSION_ROWS)
    {
        c->versions_overflowed++;
        return NULL;
    }
    c->versions[c->version_count].min_vers = min_vers;
    return &c->versions[c->version_count++];
}

/* Inflate one file structure tessellation section and walk its entries,
   accumulating into c. Returns 0 if the whole section was walked, 1 if the
   walk stopped early (unparsable entry, or a section that is not one).
   *file_sizing accumulates this file's edge_status verdict across sections. */
static int
walk_tess_section(prc_context *ctx, const uint8_t *buf, const uint32_t *section_offsets,
    const char *name, uint32_t fs, int mode, census *c, int *file_sizing)
{
    uint8_t *inflated = NULL;
    uLongf dest_len;
    uint32_t comp_len = section_offsets[4] - section_offsets[3];
    prc_bit_state bit_state;
    uint32_t wrapper_tag, tess_count, k;
    int stopped = 0;

    dest_len = (uLongf)comp_len * 40u + 4096u;
    inflated = (uint8_t *)malloc(dest_len);
    if (inflated == NULL)
        return 1;
    for (;;)
    {
        uLongf try_len = dest_len;
        int zret = uncompress(inflated, &try_len, buf + section_offsets[3], comp_len);
        if (zret == Z_OK) { dest_len = try_len; break; }
        if (zret == Z_BUF_ERROR)
        {
            dest_len *= 2;
            free(inflated);
            inflated = (uint8_t *)malloc(dest_len);
            if (inflated == NULL) return 1;
            continue;
        }
        free(inflated);
        return 1;
    }

    prc_init_bit_state(ctx, &bit_state, inflated, (size_t)dest_len);
    wrapper_tag = prc_bitread_uint32(ctx, &bit_state);
    if (wrapper_tag != PRC_TYPE_ASM_FileStructureTessellation)
    {
        free(inflated);
        return 1;
    }
    {
        prc_content_prc_base wrapper_base;
        memset(&wrapper_base, 0, sizeof(wrapper_base));
        if (prc_parse_content_prc_base(ctx, &bit_state, &wrapper_base) < 0)
        {
            free(inflated);
            return 1;
        }
    }
    tess_count = prc_bitread_uint32(ctx, &bit_state);

    for (k = 0; k < tess_count; k++)
    {
        prc_tess_3d_compressed *parsed = NULL;
        uint32_t entry_tag = prc_bitread_uint32(ctx, &bit_state);
        uint32_t total, fresh, tps, uv_refs, T, esz;
        int sizing;

        if (entry_tag != PRC_TYPE_TESS_3D_Compressed)
        {
            stopped = 1;
            break;
        }
        if (prc_parse_tess_3d_compressed(ctx, &bit_state, &parsed, 0) < 0)
        {
            stopped = 1;
            break;
        }
        c->entities_compressed++;

        /* Fresh points are those stored outright; total points adds those
           reused through point_reference_array. reference_array_size, the
           size of points_is_reference_array, counts both. On a file with no
           reuse the two coincide, which is exactly why a single-file
           measurement cannot tell them apart. */
        fresh = parsed->point_array_size / 3u;
        total = parsed->reference_array_size;
        tps = (parsed->texture_data != NULL) ? parsed->texture_data->texture_parameters_size : 0u;
        uv_refs = (parsed->texture_data != NULL) ? parsed->texture_data->reference_array_size : 0u;
        T = parsed->triangle_face_array_size;
        esz = parsed->edge_status_array_size;

        if (esz == 0 || T == 0)
            sizing = SIZING_NONE;
        else if (esz == T)
            sizing = SIZING_T;
        else if (esz == 3u * T)
            sizing = SIZING_3T;
        else
            sizing = SIZING_OTHER;

        if (sizing == SIZING_T) c->entities_edge_t++;
        else if (sizing == SIZING_3T) c->entities_edge_3t++;
        else if (sizing == SIZING_OTHER) c->entities_edge_other++;

        *file_sizing = merge_sizing(*file_sizing, sizing);

        if (!parsed->no_texture && tps > 0)
        {
            uint32_t uv = tps / 2u;

            c->entities_textured++;
            if (tps % 2u == 0u) c->tps_even++;
            if (uv == fresh) c->uv_eq_fresh++;
            if (uv < fresh) c->uv_below_fresh++;
            if (uv > total) c->uv_above_total++;
            if (uv >= fresh && uv <= total) c->uv_in_range++;
        }

        if (mode == 1)      /* --csv: one row per entity */
        {
            printf("%s,%u,%u,%u,%u,%u,%u,%u,%u,%s\n", basename_of(name), fs, k,
                T, fresh, total, uv_refs, tps, esz, sizing_name(sizing));
        }
        else if (mode == 0)
        {
            printf("  fs %u entity %u: T=%u  fresh_points=%u  total_points=%u  "
                "uv_refs=%u  texture_parameters_size=%u  tps/2=%u%s  "
                "edge_status=%u (%s)\n",
                fs, k, T, fresh, total, uv_refs, tps, tps / 2u,
                (tps % 2u == 0u) ? "" : " <-- ODD",
                esz, sizing_name(sizing));
        }
    }

    free(inflated);
    return stopped;
}

/* mode: 0 = human-readable per entity, 1 = --csv per entity, 2 = --files per file */
static void
census_file(prc_context *ctx, const char *path, int mode, census *c)
{
    FILE *fid;
    long fsize;
    size_t read_size;
    uint8_t *buf, *p;
    uint32_t fs_count, fs, i;
    uint32_t textured_before;
    int stopped_any = 0;
    int file_sizing = SIZING_NONE;
    file_header h;
    char app[64];
    version_row *row;

    fid = fopen(path, "rb");
    if (fid == NULL)
    {
        if (mode == 0) printf("%s: cannot open\n", path);
        return;
    }
    fseek(fid, 0L, SEEK_END);
    fsize = ftell(fid);
    fseek(fid, 0L, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)fsize);
    if (buf == NULL) { fclose(fid); return; }
    read_size = fread(buf, 1, (size_t)fsize, fid);
    fclose(fid);
    if (read_size != (size_t)fsize || fsize < 47 || memcmp(buf, "PRC", 3) != 0)
    {
        if (mode == 0) printf("%s: not a raw PRC file\n", basename_of(path));
        free(buf);
        return;
    }
    c->files_seen++;
    textured_before = c->entities_textured;

    read_file_header(buf, &h);
    format_app_id(&h, app, sizeof(app));
    if (h.app_id_null) c->files_app_null++; else c->files_app_set++;

    p = buf + 3 + 4 + 4 + 16 + 16;
    fs_count = read_le_u32(p); p += 4;

    if (mode == 0)
        printf("%s  (%u file structure%s)  min_vers=%u auth_vers=%u "
            "uid_file[3]=%u app_id=%s\n",
            basename_of(path), fs_count, (fs_count == 1) ? "" : "s",
            h.min_vers, h.auth_vers, h.uid_file_word3, app);

    for (fs = 0; fs < fs_count; fs++)
    {
        uint32_t section_count;
        uint32_t *section_offsets;

        p += 16;    /* file structure unique id */
        p += 4;     /* reserved */
        section_count = read_le_u32(p); p += 4;
        section_offsets = (uint32_t *)malloc(sizeof(uint32_t) * section_count);
        if (section_offsets == NULL) break;
        for (i = 0; i < section_count; i++) { section_offsets[i] = read_le_u32(p); p += 4; }

        if (section_count >= 5)
        {
            if (walk_tess_section(ctx, buf, section_offsets, path, fs, mode, c, &file_sizing))
                stopped_any = 1;
        }
        free(section_offsets);
    }

    if (stopped_any)
        c->files_stopped_early++;
    if (c->entities_textured > textured_before)
        c->files_with_texture++;

    if (file_sizing == SIZING_T) c->files_edge_t++;
    else if (file_sizing == SIZING_3T) c->files_edge_3t++;
    else if (file_sizing == SIZING_MIXED) c->files_edge_mixed++;

    row = version_row_for(c, h.min_vers);
    if (row != NULL)
    {
        if (file_sizing == SIZING_T) row->files_t++;
        else if (file_sizing == SIZING_3T) row->files_3t++;
        else if (file_sizing == SIZING_MIXED) row->files_mixed++;
        else if (file_sizing == SIZING_OTHER) row->files_other++;
    }

    if (mode == 2)
    {
        printf("%s,%u,%u,%u,%s,%u,%s,%u\n", basename_of(path),
            h.min_vers, h.auth_vers, h.uid_file_word3, app,
            fs_count, sizing_name(file_sizing), stopped_any);
    }

    free(buf);
}

static void
print_summary(const census *c)
{
    uint32_t i;

    printf("\n== summary ==\n");
    printf("files read                    %u\n", c->files_seen);
    printf("files where the walk stopped  %u\n", c->files_stopped_early);
    printf("compressed entities walked    %u\n", c->entities_compressed);

    printf("\n-- texture_parameters --\n");
    printf("files with textured entities  %u\n", c->files_with_texture);
    printf("textured entities             %u\n", c->entities_textured);
    printf("  texture_parameters_size even (UV pairs)   %u\n", c->tps_even);
    printf("  tps/2 == fresh point count                %u\n", c->uv_eq_fresh);
    printf("  tps/2 within [fresh .. total points]      %u\n", c->uv_in_range);
    printf("  tps/2 below fresh point count             %u\n", c->uv_below_fresh);
    printf("  tps/2 above total point count             %u\n", c->uv_above_total);

    printf("\n-- edge_status_array --\n");
    printf("entities sized T              %u\n", c->entities_edge_t);
    printf("entities sized 3T             %u\n", c->entities_edge_3t);
    printf("entities sized neither        %u\n", c->entities_edge_other);
    printf("files all-T                   %u\n", c->files_edge_t);
    printf("files all-3T                  %u\n", c->files_edge_3t);
    printf("files MIXED                   %u\n", c->files_edge_mixed);

    printf("\n-- sizing by minimal_version_for_read --\n");
    printf("%-12s %8s %8s %8s %8s\n", "min_vers", "files T", "files 3T", "MIXED", "other");
    for (i = 0; i < c->version_count; i++)
    {
        printf("%-12u %8u %8u %8u %8u\n", c->versions[i].min_vers,
            c->versions[i].files_t, c->versions[i].files_3t,
            c->versions[i].files_mixed, c->versions[i].files_other);
    }
    if (c->versions_overflowed)
        printf("(%u files fell outside the first %d distinct versions and are not "
            "in the table)\n", c->versions_overflowed, MAX_VERSION_ROWS);

    printf("\n-- unique_id_application --\n");
    printf("files with a null application id  %u\n", c->files_app_null);
    printf("files with one set                %u\n", c->files_app_set);
}

int
main(int argc, char **argv)
{
    prc_context *ctx;
    census c;
    int mode = 0;
    int first = 1;
    int i;

    if (argc < 2)
    {
        printf("usage: %s [--csv|--files] <file.prc> [more files...]\n", argv[0]);
        return 2;
    }
    if (strcmp(argv[1], "--csv") == 0) { mode = 1; first = 2; }
    else if (strcmp(argv[1], "--files") == 0) { mode = 2; first = 2; }
    if (first >= argc)
    {
        printf("usage: %s [--csv|--files] <file.prc> [more files...]\n", argv[0]);
        return 2;
    }

    memset(&c, 0, sizeof(c));
    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    if (mode == 1)
        printf("file,fs,entity,T,fresh_points,total_points,uv_refs,"
            "texture_parameters_size,edge_status_array_size,edge_sizing\n");
    else if (mode == 2)
        printf("file,min_vers,auth_vers,uid_file_word3,app_id,file_structures,"
            "edge_sizing,walk_stopped\n");

    for (i = first; i < argc; i++)
        census_file(ctx, argv[i], mode, &c);

    if (mode == 0)
    {
        print_summary(&c);
    }
    else
    {
        /* Machine-readable modes keep stdout to rows only; the summary that a
           sweep still wants goes to stderr so it survives a redirect. */
        fprintf(stderr, "files=%u entities=%u textured=%u even=%u eq_fresh=%u "
            "in_range=%u below=%u above=%u edge_T=%u edge_3T=%u edge_other=%u "
            "files_T=%u files_3T=%u files_MIXED=%u app_null=%u app_set=%u "
            "stopped=%u\n",
            c.files_seen, c.entities_compressed, c.entities_textured, c.tps_even,
            c.uv_eq_fresh, c.uv_in_range, c.uv_below_fresh, c.uv_above_total,
            c.entities_edge_t, c.entities_edge_3t, c.entities_edge_other,
            c.files_edge_t, c.files_edge_3t, c.files_edge_mixed,
            c.files_app_null, c.files_app_set, c.files_stopped_early);
        for (i = 0; i < (int)c.version_count; i++)
            fprintf(stderr, "min_vers=%u files_T=%u files_3T=%u MIXED=%u other=%u\n",
                c.versions[i].min_vers, c.versions[i].files_t,
                c.versions[i].files_3t, c.versions[i].files_mixed,
                c.versions[i].files_other);
    }

    prc_api_release_context(ctx);
    return 0;
}
