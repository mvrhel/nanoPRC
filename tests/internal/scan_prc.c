/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Opens a single file (raw .prc OR .pdf -- prc_api_open_contents
   auto-detects which) through nanoPRC's normal public read API and
   reports whether it opens cleanly: part/product/markup/tessellation
   counts, and for the first tessellation found, its type and face count.
   On failure, prints nanoPRC's full internal error stack (file/line/
   message chain) so the actual parse failure point is visible without
   attaching a debugger.

   HOW: usage: scan_prc.exe <file.prc|file.pdf>. Exit code 0 = opened and
   walked the tree successfully, 1 = failed at some stage (context
   creation, open, tree prep, tree creation, tessellation counting, or
   first-tessellation initialization -- stdout indicates which). Designed
   to be run ONCE PER PROCESS INVOCATION (one file per run, via an
   external loop/script for batch scanning), specifically so that a
   crash or infinite loop on one bad file doesn't take down a whole
   corpus scan -- this was used to batch-scan a 305-file real-world PRC
   corpus (see the corpus-scan project notes) with per-file timeouts.

   WHY IT EXISTS: The most basic and most frequently reused diagnostic in
   this session -- the first thing to run against any new file (real
   reference or our own generated output) to confirm nanoPRC's own reader
   considers it well-formed before doing anything more elaborate (Acrobat
   testing, byte-level comparison, etc.).

   LIMITATIONS: Only reports the FIRST tessellation entry's details, even
   if a file has many (see the tess=N count for the total). Does not
   report geometry/style/attribute details -- see nano_prc_json_export
   (a real demo, not an internal tool) for that. num_faces requires a
   separate prc_api_get_number_faces call after initialize_tessellation
   (prc_api_initialize_tessellation alone does not populate it -- this
   tripped up an earlier version of this exact tool; see the inline
   comment at the call site). */
#include <stdio.h>
#include <string.h>
#include "prc_api.h"
#include "prc_context.h"

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts = 0, num_products = 0, num_markups = 0;
    uint32_t num_tess = 0, num_line_tess = 0, num_extra_geom_tess = 0;
    int code;

    if (argc < 2)
    {
        printf("usage: %s <file.prc>\n", argv[0]);
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        printf("RESULT\tCONTEXT_FAIL\n");
        return 1;
    }

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL)
    {
        printf("RESULT\tOPEN_FAIL\n");
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }

    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code != 0)
    {
        printf("RESULT\tPREP_TREE_FAIL\tcode=%d\n", code);
        prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL);
        prc_api_release_context(ctx);
        return 1;
    }

    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
    if (code != 0)
    {
        printf("RESULT\tCREATE_TREE_FAIL\tcode=%d\n", code);
        prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL);
        prc_api_release_context(ctx);
        return 1;
    }

    code = prc_api_get_number_tessellations(ctx, data, model_tree, &num_tess, &num_line_tess, &num_extra_geom_tess);
    if (code != 0)
    {
        printf("RESULT\tGET_NUM_TESS_FAIL\tcode=%d\n", code);
        prc_api_release_data(ctx, data, NULL, 0, NULL, 0, model_tree);
        prc_api_release_context(ctx);
        return 1;
    }

    printf("RESULT\tOK\tparts=%u\tproducts=%u\tmarkups=%u\ttess=%u\tline_tess=%u\n",
        num_parts, num_products, num_markups, num_tess, num_line_tess);

    if (num_tess > 0)
    {
        prc_api_tess api_tess, api_tess_line;
        uint8_t has_line = 0;

        memset(&api_tess, 0, sizeof(api_tess));
        memset(&api_tess_line, 0, sizeof(api_tess_line));
        code = prc_api_initialize_tessellation(ctx, data, model_tree, 0, &api_tess, &api_tess_line, &has_line);
        if (code != 0)
        {
            printf("RESULT\tINIT_TESS0_FAIL\tcode=%d\n", code);
        }
        else
        {
            const char *type_name = "UNKNOWN";
            uint32_t real_num_faces;

            switch (api_tess.type)
            {
                case PRC_API_TESS_3D: type_name = "TRIANGLES"; break;
                case PRC_API_TESS_3D_Compressed: type_name = "COMPRESSED"; break;
                case PRC_API_TESS_3D_Wire: type_name = "WIRE"; break;
                case PRC_API_TESS_MarkUp: type_name = "MARKUP"; break;
                default: break;
            }
            /* prc_api_initialize_tessellation does not itself populate
               num_faces -- that requires this separate call (confirmed by
               cross-checking against demos/teapot_write's own verification
               code, after this scanner's naive api_tess.num_faces read
               showed 0 for files independently confirmed via json_export
               to have real, non-empty geometry). */
            real_num_faces = prc_api_get_number_faces(ctx, data, 0);
            printf("TESS0\ttype=%s\tnum_faces=%u\tbbox_min=(%f,%f,%f)\tbbox_max=(%f,%f,%f)\n",
                type_name, real_num_faces,
                api_tess.bounding_box_min[0], api_tess.bounding_box_min[1], api_tess.bounding_box_min[2],
                api_tess.bounding_box_max[0], api_tess.bounding_box_max[1], api_tess.bounding_box_max[2]);
        }
    }

    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, model_tree);
    prc_api_release_context(ctx);
    return 0;
}
