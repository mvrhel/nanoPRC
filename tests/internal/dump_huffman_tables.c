/* dump_huffman_tables -- dump every Huffman code table in a PRC, with the
 * symbol frequencies that produced it, and report a census of their shape.
 *
 * WHY THIS EXISTS
 * ---------------
 * Section 10.2 declares HuffmanTreeCalculation and never defines it, so the
 * tree an encoder must build is not specified. Real files answer the question
 * directly: the producer's own code table is written into every Huffman-coded
 * array, and the decoded array gives the symbol frequencies that produced it.
 * Each array is therefore a labelled test case -- "these frequencies produced
 * this table" -- needing no cooperation from the producer.
 *
 * The table is live only inside prc_huffman_decode_core (prc_bit.c), which
 * reads it, builds the tree and frees it, so the library writes the dump from
 * there under PRC_DIAG_HUFF_DUMP. This tool drives that and then summarises
 * what came out.
 *
 * THE CENSUS
 * ----------
 * The headline property is the Kraft sum of the stored code lengths. A
 * complete prefix code sums to exactly 1. Every real PRC table measured so far
 * sums to exactly 1/2, which is the signature of one wasted leading bit: the
 * true root's other branch is an unused phantom leaf, so every real code
 * begins with the same bit. That is a writer-side requirement invisible to a
 * decoder, which reads the stored codes directly, and it is why an encoder
 * building a textbook-optimal tree produces output real readers reject.
 *
 * This tool reports that distribution, so the claim can be re-measured on any
 * corpus rather than taken on trust.
 *
 * usage:
 *   PRC_DIAG_HUFF_DUMP=<out.tsv> [PRC_DIAG_HUFF_TAG=<label>] \
 *       dump_huffman_tables <input.pdf|input.prc>
 *
 * The dump is APPENDED, so a whole corpus can be swept into one file by
 * running the tool per input with a per-input PRC_DIAG_HUFF_TAG.
 *
 * Requires a build configured with -DPRC_ENABLE_DIAG_ENV=ON; without it the
 * PRC_DIAG_* hooks are compiled out entirely and nothing is written.
 *
 * exit status: 0 dumped and summarised, 1 error, 2 usage.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "prc_api.h"
#include "prc_context.h"

/* One A record and the L records that follow it. */
static int
summarise(const char *path)
{
    FILE *fp;
    char line[512];
    unsigned long arrays = 0, leaves = 0, single_leaf = 0;
    unsigned long kraft_half = 0, kraft_one = 0, kraft_other = 0;
    uint32_t pending = 0;          /* L records still expected for this array */
    uint64_t scaled = 0;           /* sum of 2^(32-L), so 2^31 == 1/2 */
    int have_array = 0;

    fp = fopen(path, "rb");
    if (fp == NULL)
    {
        /* Not an error: an input carrying no compressed tessellation has no
           Huffman-coded arrays, so nothing was ever appended. */
        printf("no Huffman-coded arrays in this input\n");
        return 0;
    }

    while (fgets(line, (int)sizeof(line), fp) != NULL)
    {
        if (line[0] == 'A')
        {
            unsigned nb, es, nl, mcl, nv;
            char tag[128];

            if (have_array && pending == 0)
            {
                /* close out the previous array */
                if (scaled == ((uint64_t)1 << 31)) kraft_half++;
                else if (scaled == ((uint64_t)1 << 32)) kraft_one++;
                else kraft_other++;
            }
            if (sscanf(line, "A\t%127s\t%*lu\t%u\t%u\t%u\t%u\t%u",
                       tag, &nb, &es, &nl, &mcl, &nv) == 6)
            {
                arrays++;
                leaves += nl;
                if (nl == 1) single_leaf++;
                pending = nl;
                scaled = 0;
                have_array = 1;
            }
        }
        else if (line[0] == 'L' && have_array)
        {
            unsigned v, len, code, freq;

            if (sscanf(line, "L\t%u\t%u\t%u\t%u", &v, &len, &code, &freq) == 4)
            {
                if (len > 0 && len <= 32)
                    scaled += ((uint64_t)1 << (32 - len));
                if (pending > 0)
                    pending--;
            }
        }
    }
    if (have_array)
    {
        if (scaled == ((uint64_t)1 << 31)) kraft_half++;
        else if (scaled == ((uint64_t)1 << 32)) kraft_one++;
        else kraft_other++;
    }
    fclose(fp);

    printf("huffman arrays      : %lu\n", arrays);
    printf("total leaves        : %lu\n", leaves);
    printf("single-leaf arrays  : %lu\n", single_leaf);
    printf("\nKraft sum of stored code lengths\n");
    printf("  exactly 1/2       : %lu%s\n", kraft_half,
           (arrays > 0 && kraft_half == arrays) ? "   (all)" : "");
    printf("  exactly 1         : %lu\n", kraft_one);
    printf("  anything else     : %lu\n", kraft_other);
    if (arrays > 0)
        printf("\n1/2 accounts for %.3f%% of arrays\n",
               100.0 * (double)kraft_half / (double)arrays);
    return 0;
}

int
main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_api_product *model_tree = NULL;
    uint32_t num_parts, num_products, num_markups;
    uint32_t total_tess, total_line_tess, num_extra_geom_tess;
    const char *path;
    int code;

    if (argc < 2)
    {
        printf("usage: PRC_DIAG_HUFF_DUMP=<out.tsv> %s <input.pdf|input.prc>\n",
               argv[0]);
        return 2;
    }

    path = getenv("PRC_DIAG_HUFF_DUMP");
    if (path == NULL || path[0] == '\0')
    {
        printf("PRC_DIAG_HUFF_DUMP is not set -- nothing would be written.\n");
        printf("usage: PRC_DIAG_HUFF_DUMP=<out.tsv> %s <input.pdf|input.prc>\n",
               argv[0]);
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL) { printf("context creation failed\n"); return 1; }

    /* Parsing is all that is needed: the tables are dumped as each Huffman
       array is decoded, which happens while the tessellation records are
       read. Walking the model tree afterwards is what forces that to happen
       for every tessellation in the file. */
    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL)
    {
        printf("open_contents failed\n");
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }
    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code >= 0)
        code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts,
                                         num_products, num_markups);
    if (code >= 0)
        code = prc_api_get_number_tessellations(ctx, data, model_tree,
                                                &total_tess, &total_line_tess,
                                                &num_extra_geom_tess);
    if (code < 0)
    {
        printf("parse failed before the model tree was walked\n");
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }

    /* release_context tears down everything reachable from the context; the
       per-tessellation release_data form needs arrays this tool never builds. */
    prc_api_release_context(ctx);

    printf("dumped to %s\n\n", path);
    return summarise(path);
}
