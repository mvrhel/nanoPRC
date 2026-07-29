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

/* Object graph written here, confirmed by extracting and reading the raw
   bytes of a real, working 3D PDF (examples/cube.pdf) rather than from
   memory of the spec:

     Catalog     <</Type/Catalog /Pages <Pages ref>>>
     Pages       <</Type/Pages /Kids[<Page ref>] /Count 1>>
     Page        <</Type/Page /Parent <Pages ref> /MediaBox[0 0 W H]
                   /Annots[<Annot ref>]>>            -- no /Contents needed;
                                                         confirmed optional
     3D stream   <</Type/3D /Subtype/PRC /Resources<</Names[]>>
                   /VA[<View ref> ...] /Length N>>
                 stream ... raw PRC bytes ... endstream
                                                      -- /VA: indirect refs
                                                         to 3DView objects,
                                                         omitted entirely if
                                                         no views given.
                                                         /Resources<</Names[]>>
                                                         present, identically,
                                                         on all five real
                                                         %PDF-1.7 references
                                                         (see the /3DA comment
                                                         below for which)
     3DView      <</Type/3DView /XN(name) /C2W[12 numbers] /CO d>>
                                                      -- no /IN: cube.pdf's
                                                         real views never
                                                         carry it alongside
                                                         /XN, only /XN alone
     BorderStyle <</S/S /Type/Border /W 0>>            -- cube.pdf carries
                                                         both this AND the
                                                         old-style /Border
                                                         array on its Annot,
                                                         not one or the other
     Annot       <</Type/Annot /Subtype/3D /Rect[x0 y0 x1 y1] /P <Page ref>
                   /3DD <3D stream ref> /3DV <default View ref> /3DI true
                   /3DA<</A/XA/AIS/L/D/XD/DIS/L/NP false/TB true>>
                   /AP<</N <Appearance ref>>> /BS <BorderStyle ref>
                   /Border[0 0 0] /Contents(...) /NM(...) /UID(1)>>
                                                      -- /3DA's AIS/DIS/NP/TB
                                                         copied verbatim from
                                                         five real,
                                                         independently-
                                                         produced, Acrobat-
                                                         confirmed-working
                                                         %PDF-1.7 files
                                                         (ElevationMeshIS_ePRC.pdf,
                                                         xml-sample-{wrl,iv,3ds}_ePRC.pdf,
                                                         Teapot_ePRC.pdf), not
                                                         cube.pdf -- cube.pdf
                                                         declares %PDF-1.6 and
                                                         includes /Transparent
                                                         true, a PDF-2.0-only
                                                         key (ISO 32000-2 Table
                                                         310) invalid in a
                                                         %PDF-1.7 file (which is
                                                         what this writer
                                                         declares, and what all
                                                         five real references
                                                         declare too); the five
                                                         real files' /3DA has no
                                                         /Transparent key at
                                                         all. /A and /D shown
                                                         here are this writer's
                                                         own default -- no
                                                         auto-open, see
                                                         PRC_PDF_AUTO_ACTIVATE_3D
                                                         below -- all five real
                                                         references actually
                                                         use /A/PV /D/PI
                                                         (auto-open), still
                                                         available by flipping
                                                         that define. /UID is a
                                                         fixed placeholder,
                                                         cube.pdf has one too
     Appearance  <</Type/XObject /Subtype/Form /BBox[0 0 w h]
                   /Matrix[1 0 0 1 0 0]
                   /Resources<</Font<</F1<</Type/Font/Subtype/Type1
                                            /BaseFont/Helvetica-Bold>>>>>>
                   /Length L>>
                 stream ... solid light-blue filled rectangle, with a
                            centered "nanoPRC" title in dark blue ... endstream
                                                      -- placeholder shown
                                                         outside interactive
                                                         3D viewers; real
                                                         viewers replace it
                                                         with live 3D content
                                                         on activation
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "prc_write_pdf_3d.h"
#include "prc_write_pdf.h"

/* Document Information dictionary values. PRC_PDF_PRODUCER identifies this
   write facility to anything inspecting the PDF's own metadata (distinct
   from the embedded PRC stream's own version/producer fields, which are
   unrelated). */
#define PRC_PDF_PRODUCER "nanoPRC (AGPL)"

/* Default page: landscape, 4:3 aspect (width:height = 4:3), long edge
   (width) = 11in at 72pt/in -- height = 11in * 3/4 = 8.25in = 594pt.
   Portrait is still available to a caller: prc_pdf_write_options lets
   page_width_pt/page_height_pt be set directly -- pass a height greater
   than the width (e.g. swap these two constants) for a portrait page.
   There is no separate orientation flag, just whichever of the two
   dimensions the caller makes larger. */
#define PRC_PDF_DEFAULT_PAGE_WIDTH  792.0
#define PRC_PDF_DEFAULT_PAGE_HEIGHT 594.0
#define PRC_PDF_DEFAULT_MARGIN      36.0   /* half an inch */

/* Light-blue placeholder fill (device RGB, 0..1) shown in non-interactive
   contexts (printing, thumbnailing) in place of live 3D content. */
#define PRC_PDF_APPEARANCE_R 0.85
#define PRC_PDF_APPEARANCE_G 0.92
#define PRC_PDF_APPEARANCE_B 1.00

/* Placeholder title text drawn centered over the fill, dark blue, in a
   large built-in (no embedding needed) font. */
#define PRC_PDF_TITLE_TEXT      "nanoPRC"
#define PRC_PDF_TITLE_FONT_SIZE 48.0
#define PRC_PDF_TITLE_R 0.00
#define PRC_PDF_TITLE_G 0.00
#define PRC_PDF_TITLE_B 0.50

/* Rough average glyph-width-as-a-fraction-of-font-size for Helvetica-Bold,
   used only to horizontally center the fixed title string above -- close
   enough for a placeholder without pulling in real AFM metrics. */
#define PRC_PDF_TITLE_AVG_CHAR_WIDTH_EM 0.6

/* 1 = auto-activate the 3D view as soon as its page becomes visible (the
   original behavior: /A/PV /D/PI, confirmed against 5 real, independently-
   produced %PDF-1.7 files -- see the /3DA comment above prc_write_pdf_3d_
   annotation).
   0 (default) = require explicit user activation ("click to play"):
   /A/XA /D/XD. XA is the PDF-spec DEFAULT value for /A when the key is
   omitted entirely (verified against Adobe's Acrobat SDK "Working with 3D
   Annotations" documentation); XD is inferred by naming symmetry with the
   PV/PI pairing above (explicit-activate pairs with explicit-deactivate,
   the same way page-visible pairs with page-invisible) and has not been
   independently spec-verified to the same standard as XA. Either way,
   AIS/DIS/NP/TB are unaffected -- only the activation/deactivation
   *trigger* changes, not what state is reached once triggered. */
#define PRC_PDF_AUTO_ACTIVATE_3D 0

typedef struct { double x, y, z; } prc_pdf_vec3;

static prc_pdf_vec3
vec3_sub(prc_pdf_vec3 a, prc_pdf_vec3 b)
{
    prc_pdf_vec3 r;
    r.x = a.x - b.x; r.y = a.y - b.y; r.z = a.z - b.z;
    return r;
}

static double
vec3_dot(prc_pdf_vec3 a, prc_pdf_vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static prc_pdf_vec3
vec3_cross(prc_pdf_vec3 a, prc_pdf_vec3 b)
{
    prc_pdf_vec3 r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

static double
vec3_length(prc_pdf_vec3 a)
{
    return sqrt(vec3_dot(a, a));
}

static prc_pdf_vec3
vec3_scale(prc_pdf_vec3 a, double s)
{
    prc_pdf_vec3 r;
    r.x = a.x * s; r.y = a.y * s; r.z = a.z * s;
    return r;
}

/* Converts an eye/target/up "look-at" camera into the PDF format's
   camera-to-world matrix and center-of-orbit distance.

   2026-07-10: the previous version of this function was wrong in two
   compounding ways, found by extracting examples/cube.pdf's raw, real
   3DView object (`C2W[0.176366 -0.821474 0.542287 -0.573928 0.361762
   0.734666 -0.799688 -0.440804 -0.407664 199.596 116.632 67.996]/CO
   219.534`) and comparing it against the cube's own true geometric
   center, computed precisely from its full exported vertex data:
   (24.037, 19.861, -21.498).

   (1) The 12-number layout is 3 CONSECUTIVE ROWS, each one full basis
       vector (R.x R.y R.z, U.x U.y U.z, F.x F.y F.z), NOT 3 rows of
       INTERLEAVED components (R.x U.x F.x, R.y U.y F.y, R.z U.z F.z) as
       this function previously wrote. Confirmed by reading c2w[0..2] as
       a single vector directly from cube.pdf's own bytes and checking it
       is unit length (it is), then verifying right = cross(up, forward)
       reproduces it (see below).
   (2) The third basis vector is FORWARD (eye -> target), not BACK
       (target -> eye) as this function previously computed. Confirmed
       by solving for the cube's real center two ways from cube.pdf's
       real numbers: target = eye - CO*back lands at (80.5,-44.6,157.5)
       (nowhere near the cube); target = eye + CO*forward, with forward
       read directly as cube.pdf's own third row, lands at
       (24.042,19.857,-21.498) -- matching the true center to 3+ decimal
       places.

   Degenerate inputs (eye == target, or up parallel to the view
   direction) fall back to a well-defined axis-aligned basis instead of
   producing NaNs. */
static void
prc_pdf_compute_c2w_co(const prc_pdf_view_spec *v, double c2w[12], double *co_out)
{
    prc_pdf_vec3 eye, target, up_in, forward, right, true_up;
    double dist;

    eye.x = v->eye[0]; eye.y = v->eye[1]; eye.z = v->eye[2];
    target.x = v->target[0]; target.y = v->target[1]; target.z = v->target[2];
    up_in.x = v->up[0]; up_in.y = v->up[1]; up_in.z = v->up[2];

    forward = vec3_sub(target, eye);
    dist = vec3_length(forward);
    if (dist < 1e-9)
    {
        forward.x = 0.0; forward.y = 0.0; forward.z = -1.0;
        dist = 0.0;
    }
    else
    {
        forward = vec3_scale(forward, 1.0 / dist);
    }

    right = vec3_cross(up_in, forward);
    if (vec3_length(right) < 1e-9)
    {
        /* Requested up is parallel to the view direction -- pick whichever
           world axis is least aligned with `forward` as a fallback up. */
        prc_pdf_vec3 fallback_up;
        fallback_up.x = 0.0; fallback_up.y = 1.0; fallback_up.z = 0.0;
        if (fabs(forward.y) > 0.9)
        {
            fallback_up.x = 1.0; fallback_up.y = 0.0; fallback_up.z = 0.0;
        }
        right = vec3_cross(fallback_up, forward);
    }
    right = vec3_scale(right, 1.0 / vec3_length(right));
    true_up = vec3_cross(forward, right);

    /* Rows = camera-space right/up/forward expressed in world space, each
       one a contiguous triple -- matches the 12-number layout read
       directly out of examples/cube.pdf's own 3DView object. */
    c2w[0] = right.x;    c2w[1] = right.y;    c2w[2] = right.z;
    c2w[3] = true_up.x;  c2w[4] = true_up.y;  c2w[5] = true_up.z;
    c2w[6] = forward.x;  c2w[7] = forward.y;  c2w[8] = forward.z;
    c2w[9] = eye.x; c2w[10] = eye.y; c2w[11] = eye.z;

    *co_out = dist;
}

static int
prc_pdf_write_view_obj(prc_context *ctx, prc_pdf_writer *w, uint32_t obj_num,
    const prc_pdf_view_spec *v)
{
    double c2w[12];
    double co;
    const char *name = (v->name != NULL) ? v->name : "View";
    int i;

    prc_pdf_compute_c2w_co(v, c2w, &co);

    if (prc_pdf_begin_obj(ctx, w, obj_num) != 0) return PRC_ERROR_IO;
    if (fprintf(w->fid, "<</Type/3DView/XN") < 0) goto io_fail;
    prc_pdf_write_escaped_string(w, name);
    /* No /IN (internal name): examples/cube.pdf's real, working 3DView
       objects only ever carry /XN, never /IN alongside it. */
    if (fprintf(w->fid, "/C2W[") < 0) goto io_fail;
    for (i = 0; i < 12; i++)
        if (fprintf(w->fid, i == 0 ? "%.6f" : " %.6f", c2w[i]) < 0) goto io_fail;
    /* /MS selects how the reader should interpret the camera parameters
       below; /M ("C2W matrix + center-of-orbit distance") is what /C2W and
       /CO actually are, and is the value examples/cube.pdf's own working
       view objects use. Without it, a reader has no defined way to know
       what /C2W/CO mean. */
    if (fprintf(w->fid, "]/CO %.6f/MS/M>>", co) < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, w) != 0) return PRC_ERROR_IO;

    return 0;

io_fail:
    w->error = 1;
    prc_error(ctx, PRC_ERROR_IO, "prc_pdf_write_view_obj: write failed\n");
    return PRC_ERROR_IO;
}

int
prc_write_pdf_3d_annotation(prc_context *ctx, const char *pdf_path,
    const uint8_t *prc_data, size_t prc_size,
    const prc_pdf_write_options *options)
{
    static const prc_pdf_write_options default_options = { 0.0, 0.0, 0.0, NULL, 0u };
    FILE *fid = NULL;
    prc_pdf_writer w;
    int writer_initialized = 0;
    double page_w, page_h, margin;
    double rect_x0, rect_y0, rect_x1, rect_y1, rect_w, rect_h;
    char appearance_content[384];
    double title_width, title_x, title_y;
    int appearance_content_len;
    uint32_t catalog_num, pages_num, page_num, annots_num, annot_num, appearance_num, stream_num, border_style_num, info_num;
    uint32_t *view_nums = NULL;
    uint32_t default_view_index = 0;
    uint32_t i;
    int code;
    int ret = PRC_ERROR_INTERNAL;

    if (ctx == NULL || pdf_path == NULL || (prc_data == NULL && prc_size > 0))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_pdf_3d_annotation: invalid arguments\n");
        return PRC_ERROR_INTERNAL;
    }
    if (options == NULL)
        options = &default_options;

    page_w = (options->page_width_pt > 0.0) ? options->page_width_pt : PRC_PDF_DEFAULT_PAGE_WIDTH;
    page_h = (options->page_height_pt > 0.0) ? options->page_height_pt : PRC_PDF_DEFAULT_PAGE_HEIGHT;
    margin = (options->margin_pt > 0.0) ? options->margin_pt : PRC_PDF_DEFAULT_MARGIN;

    rect_x0 = margin; rect_y0 = margin;
    rect_x1 = page_w - margin; rect_y1 = page_h - margin;
    rect_w = rect_x1 - rect_x0;
    rect_h = rect_y1 - rect_y0;
    if (rect_w <= 0.0 || rect_h <= 0.0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_pdf_3d_annotation: margin too large for page size\n");
        return PRC_ERROR_INTERNAL;
    }

    if (options->num_views > 0)
    {
        view_nums = (uint32_t *)prc_malloc(ctx, options->num_views * sizeof(uint32_t));
        if (view_nums == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_write_pdf_3d_annotation\n");
            return PRC_ERROR_MEMORY;
        }
        for (i = 0; i < options->num_views; i++)
            if (options->views[i].is_default) { default_view_index = i; break; }
    }

    fid = fopen(pdf_path, "wb");
    if (fid == NULL)
    {
        prc_error(ctx, PRC_ERROR_IO, "prc_write_pdf_3d_annotation: failed to open output file\n");
        goto cleanup;
    }

    code = prc_pdf_writer_init(ctx, &w, fid);
    /* prc_pdf_writer_init memsets w (and allocates obj_offsets) before it
       can fail, so releasing it is safe from this point on regardless of
       whether init itself succeeded. */
    writer_initialized = 1;
    if (code != 0) { ret = code; goto cleanup; }

    /* Allocate every object number up front so forward references (Pages
       -> Page, Page -> Annot, Annot -> 3D stream/view/appearance) can be
       written by number regardless of the order bodies are emitted in. */
    catalog_num = prc_pdf_writer_alloc_obj(ctx, &w);
    pages_num = prc_pdf_writer_alloc_obj(ctx, &w);
    page_num = prc_pdf_writer_alloc_obj(ctx, &w);
    annots_num = prc_pdf_writer_alloc_obj(ctx, &w);
    annot_num = prc_pdf_writer_alloc_obj(ctx, &w);
    appearance_num = prc_pdf_writer_alloc_obj(ctx, &w);
    stream_num = prc_pdf_writer_alloc_obj(ctx, &w);
    border_style_num = prc_pdf_writer_alloc_obj(ctx, &w);
    info_num = prc_pdf_writer_alloc_obj(ctx, &w);
    for (i = 0; i < options->num_views; i++)
        view_nums[i] = prc_pdf_writer_alloc_obj(ctx, &w);
    if (w.error) { ret = PRC_ERROR_MEMORY; goto cleanup; }

    /* 3DView objects. */
    for (i = 0; i < options->num_views; i++)
    {
        code = prc_pdf_write_view_obj(ctx, &w, view_nums[i], &options->views[i]);
        if (code != 0) { ret = code; goto cleanup; }
    }

    /* 3D stream: /Type/3D /Subtype/PRC, raw PRC bytes, no PDF-level
       compression (each PRC section is already zlib-deflated). /Resources
       <</Names[]>> (an empty named-resources dict, not the geometry itself)
       is present, identically, on all five real, independently-produced,
       Acrobat-confirmed-working reference files this writer's structure is
       otherwise based on (ElevationMeshIS_ePRC.pdf, xml-sample-
       {wrl,iv,3ds}_ePRC.pdf, Teapot_ePRC.pdf) -- included here to match,
       even though this writer never populates any named resources. */
    if (prc_pdf_begin_obj(ctx, &w, stream_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/3D/Subtype/PRC/Resources<</Names[]>>") < 0) goto io_fail;
    if (options->num_views > 0)
    {
        if (fprintf(fid, "/VA[") < 0) goto io_fail;
        for (i = 0; i < options->num_views; i++)
            if (fprintf(fid, "%u 0 R ", view_nums[i]) < 0) goto io_fail;
        if (fprintf(fid, "]") < 0) goto io_fail;
    }
    if (fprintf(fid, "/Length %lu>>", (unsigned long)prc_size) < 0) goto io_fail;
    if (prc_pdf_write_stream_body(ctx, &w, prc_data, prc_size) != 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Appearance XObject: a solid light-blue rectangle filling the
       annotation's rect, with a centered "nanoPRC" title over it, shown in
       place of live 3D content outside an interactive 3D viewer. Title
       centering uses PRC_PDF_TITLE_AVG_CHAR_WIDTH_EM (an approximate,
       not-real-AFM-metrics glyph width) since this is a fixed, short
       placeholder string rather than arbitrary caller text. */
    title_width = (double)strlen(PRC_PDF_TITLE_TEXT) * PRC_PDF_TITLE_FONT_SIZE * PRC_PDF_TITLE_AVG_CHAR_WIDTH_EM;
    title_x = (rect_w - title_width) / 2.0;
    title_y = (rect_h - PRC_PDF_TITLE_FONT_SIZE) / 2.0 + PRC_PDF_TITLE_FONT_SIZE * 0.2;
    appearance_content_len = snprintf(appearance_content, sizeof(appearance_content),
        "q\n%.3f %.3f %.3f rg\n0 0 %.2f %.2f re\nf\nQ\n"
        "q\n%.3f %.3f %.3f rg\nBT\n/F1 %.2f Tf\n%.2f %.2f Td\n(%s) Tj\nET\nQ",
        PRC_PDF_APPEARANCE_R, PRC_PDF_APPEARANCE_G, PRC_PDF_APPEARANCE_B, rect_w, rect_h,
        PRC_PDF_TITLE_R, PRC_PDF_TITLE_G, PRC_PDF_TITLE_B, PRC_PDF_TITLE_FONT_SIZE,
        title_x, title_y, PRC_PDF_TITLE_TEXT);
    if (appearance_content_len < 0 || (size_t)appearance_content_len >= sizeof(appearance_content))
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "prc_write_pdf_3d_annotation: appearance content stream too long\n");
        goto cleanup;
    }
    if (prc_pdf_begin_obj(ctx, &w, appearance_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/XObject/Subtype/Form/BBox[0 0 %.2f %.2f]/Matrix[1 0 0 1 0 0]"
                "/Resources<</Font<</F1<</Type/Font/Subtype/Type1/BaseFont/Helvetica-Bold>>>>>>/Length %d>>",
                rect_w, rect_h, appearance_content_len) < 0) goto io_fail;
    if (prc_pdf_write_stream_body(ctx, &w, (const uint8_t *)appearance_content, (size_t)appearance_content_len) != 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Border style dict, as its own indirect object: examples/cube.pdf's
       real, working annotation carries BOTH the old-style /Border[0 0 0]
       array AND a /BS reference to this dict (`<</S/S/Type/Border/W 0>>`)
       -- not one or the other. */
    if (prc_pdf_begin_obj(ctx, &w, border_style_num) != 0) goto io_fail;
    if (fprintf(fid, "<</S/S/Type/Border/W 0>>") < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Annotation. /3DA's AIS/DIS/NP/TB values based on five real,
       independently-produced, Acrobat-confirmed-working %PDF-1.7 files
       (ElevationMeshIS_ePRC.pdf, xml-sample-{wrl,iv,3ds}_ePRC.pdf,
       Teapot_ePRC.pdf): both instantiation states "live", hide the
       navigation panel, show the toolbar. /A and /D (the activation/
       deactivation triggers) are this writer's own choice, gated by
       PRC_PDF_AUTO_ACTIVATE_3D above -- all five real references use
       /A/PV /D/PI (auto-activate/deactivate on page visibility), but that
       is not the default here; see the define's own comment. No
       /Transparent key -- an earlier version of this code included it,
       reasoning that examples/cube.pdf (a %PDF-1.6 file) has it too so it
       must be safe; that reasoning doesn't hold once actually compared
       against real %PDF-1.7 output (this writer's own declared version):
       /Transparent is a PDF-2.0-only key (ISO 32000-2 Table 310), invalid
       in a 1.7 file, and none of the five real 1.7 references use it at
       all -- cube.pdf being 1.6 is exactly why its inclusion there doesn't
       generalize. /UID is a stable per-annotation identifier some readers
       use for revision tracking; cube.pdf has one (an arbitrary-looking
       numeric string), so a fixed placeholder is written here too rather
       than omitting the key entirely. */
    if (prc_pdf_begin_obj(ctx, &w, annot_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/Annot/Subtype/3D/Rect[%.2f %.2f %.2f %.2f]/P %u 0 R",
                rect_x0, rect_y0, rect_x1, rect_y1, page_num) < 0) goto io_fail;
    if (fprintf(fid, "/3DD %u 0 R", stream_num) < 0) goto io_fail;
    if (options->num_views > 0)
        if (fprintf(fid, "/3DV %u 0 R", view_nums[default_view_index]) < 0) goto io_fail;
#if PRC_PDF_AUTO_ACTIVATE_3D
    if (fprintf(fid, "/3DI true/3DA<</A/PV/AIS/L/D/PI/DIS/L/NP false/TB true>>") < 0) goto io_fail;
#else
    if (fprintf(fid, "/3DI true/3DA<</A/XA/AIS/L/D/XD/DIS/L/NP false/TB true>>") < 0) goto io_fail;
#endif
    if (fprintf(fid, "/AP<</N %u 0 R>>/BS %u 0 R/Border[0 0 0]/Contents", appearance_num, border_style_num) < 0) goto io_fail;
    prc_pdf_write_escaped_string(&w, "3D Model");
    if (fprintf(fid, "/NM") < 0) goto io_fail;
    prc_pdf_write_escaped_string(&w, "nanoPRC_3DAnnot");
    if (fprintf(fid, "/UID(1)>>") < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Annots array as its own indirect object rather than inline in the
       Page dict -- both are spec-legal, but this matches examples/cube.pdf's
       real structure exactly (its own /Annots is `14 0 R`, a separate
       array object). */
    if (prc_pdf_begin_obj(ctx, &w, annots_num) != 0) goto io_fail;
    if (fprintf(fid, "[%u 0 R]", annot_num) < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Page: no static /Contents needed -- the annotation's own appearance
       (and, in an interactive viewer, the live 3D content) supplies
       everything visible; confirmed against examples/cube.pdf, whose own
       Page object omits /Contents entirely. /Resources<<>> is likewise
       present (even though empty) to match cube.pdf's Page exactly. */
    if (prc_pdf_begin_obj(ctx, &w, page_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/Page/Parent %u 0 R/MediaBox[0 0 %.2f %.2f]/Resources<<>>/Annots %u 0 R>>",
                pages_num, page_w, page_h, annots_num) < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    if (prc_pdf_begin_obj(ctx, &w, pages_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/Pages/Kids[%u 0 R]/Count 1>>", page_num) < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    if (prc_pdf_begin_obj(ctx, &w, catalog_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Type/Catalog/Pages %u 0 R>>", pages_num) < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    /* Document Information dictionary: /Producer identifies this write
       facility; /CreationDate is this file's actual write time, in the
       PDF date-string format (ISO 32000-1 7.9.4), UTC ('Z' relationship,
       no offset digits needed). gmtime()'s static buffer is used and
       copied out immediately -- no other call between here and its use. */
    if (prc_pdf_begin_obj(ctx, &w, info_num) != 0) goto io_fail;
    if (fprintf(fid, "<</Producer") < 0) goto io_fail;
    prc_pdf_write_escaped_string(&w, PRC_PDF_PRODUCER);
    {
        time_t now = time(NULL);
        struct tm tm_utc;
        struct tm *tm_ptr = gmtime(&now);
        char date_str[32];

        if (tm_ptr != NULL)
        {
            tm_utc = *tm_ptr;
            strftime(date_str, sizeof(date_str), "D:%Y%m%d%H%M%SZ", &tm_utc);
            if (fprintf(fid, "/CreationDate(%s)", date_str) < 0) goto io_fail;
        }
    }
    if (fprintf(fid, ">>") < 0) goto io_fail;
    if (prc_pdf_end_obj(ctx, &w) != 0) goto io_fail;

    if (prc_pdf_write_xref_and_trailer(ctx, &w, catalog_num, info_num) != 0) goto cleanup;

    ret = 0;
    goto cleanup;

io_fail:
    w.error = 1;
    prc_error(ctx, PRC_ERROR_IO, "prc_write_pdf_3d_annotation: I/O error writing output file\n");
    ret = PRC_ERROR_IO;

cleanup:
    if (view_nums != NULL) prc_free(ctx, view_nums);
    if (writer_initialized) prc_pdf_writer_release(ctx, &w);
    if (fid != NULL) fclose(fid);
    return ret;
}
