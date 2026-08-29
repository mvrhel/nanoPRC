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

   WHAT: Prints every Section-8 (EXTENSION) schema program declared by a PRC
   file, as a symbolic opcode sequence, one line per declared entity type:

     PROG type=<entity_type> tokens=<count> : <op> <op> ...

   Opcode names come from prc_schema_tokens_t (src/prc_data.h). A token the
   VM treats as an inline operand rather than an opcode -- a version number,
   a type constant, a variable index -- has no name and is printed as
   `#<value>`, so `BlockVersion #15083` reads as "block gated at version
   15083".

   WHY: ISO/CD 14739-1.4:2026 Section 8 lets a writer ship a little program
   describing entity fields the reader's compiled code does not know about.
   The instruction set is expressive -- variables, comparisons, branches,
   counted loops, arithmetic -- which raises an architectural question worth
   answering with data rather than assumption: are these programs used to
   describe LAYOUT (so a reader knows how many bits to consume, and can skip
   what it does not understand), or do they express SEMANTIC behaviour that
   a reader is expected to act on?

   That distinction matters because it determines where new format behaviour
   can live. A program that only describes layout cannot say what a field
   MEANS; any interpretation must be in the reader's compiled code, keyed on
   entity type and version. See also prc_schema.c, whose VM reads values into
   a function-local prc_schema_read and discards them -- the only lasting
   effect of executing a program in this implementation is where the
   bitstream cursor ends up.

   HOW: usage: dump_schema_programs <input.prc|input.pdf>

   For each program it prints the raw opcode line first, then an indented
   pseudo-code rendering so the program can be read without knowing the
   opcode set:

     PROG type=806 tokens=10 : BlockStart IGN1 Double BlockVersion #15216 ...
       {
         ignore 1 operand
         read double
         if (reader_version < 15216) {          // fields added in v15216
           read double
           read double
           read double
           read double
         }
       }

   The rendering is a READABILITY AID, not a faithful decompiler: it walks
   the token list linearly and indents on block/branch/loop openers. It does
   not reproduce the VM's operand-consumption rules exactly (see
   prc_execute_schema_instruction in src/prc_schema.c for those), so an
   unusual program may render awkwardly even though the raw line above it is
   always exact. Trust the raw line; use the pseudo-code to orient.

   Prints nothing for a file that declares no schema (the common case).
   Drive a corpus from a shell loop and classify the output; the opcode
   families that distinguish layout description from value computation are
   FRAME/READ (layout), DISPATCH/DELEGATE (conditional field selection),
   REPEAT (counted arrays) and the arithmetic operators MULT/DIV/ADD/SUB.

   Measured across a 310-file third-party corpus (2026-08-29): 271 declared
   programs, 13 distinct (type, body) pairs, spanning entity types 2, 207,
   303, 501, 741, 801, 802 and 806. 64.9% read a fixed field list, 23.6%
   additionally branch on a tag or delegate to a parent type, and 11.4%
   additionally use counted repetition. Arithmetic opcodes appear in ZERO
   programs -- and of the comparison operators only EQ occurs, used solely to
   select a branch.

   KNOWN LIMITATION: this reports what a file DECLARES, which is not the same
   as what a reader EXECUTES. nanoPRC executes programs for three types at
   top level (ROOTBaseWithGraphics, ASM_FileStructureGlobals, MKP_View) and
   reaches others only through ParentType delegation, and Block_Version gates
   skip whole bodies whose version is at or below the reader version. Use the
   PRC_TRACE/diagnostic paths in prc_schema.c if the question is what ran,
   rather than what was offered. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prc_data.h"
#include "prc_api.h"
#include "prc_context.h"

static const char *
schema_opcode_name(unsigned int token)
{
    switch (token)
    {
    case EPRCSchema_Data_Boolean:          return "Bool";
    case EPRCSchema_Data_Double:           return "Double";
    case EPRCSchema_Data_Character:        return "Char";
    case EPRCSchema_Data_Unsigned_Integer: return "UInt";
    case EPRCSchema_Data_Integer:          return "Int";
    case EPRCSchema_Data_String:           return "String";
    case EPRCSchema_Parent_Type:           return "ParentType";
    case EPRCSchema_Vector_2D:             return "Vec2";
    case EPRCSchema_Vector_3D:             return "Vec3";
    case EPRCSchema_Extent_1D:             return "Ext1";
    case EPRCSchema_Extent_2D:             return "Ext2";
    case EPRCSchema_Extent_3D:             return "Ext3";
    case EPRCSchema_Ptr_Type:              return "PtrType";
    case EPRCSchema_Ptr_Surface:           return "PtrSurface";
    case EPRCSchema_Ptr_Curve:             return "PtrCurve";
    case EPRCSchema_For:                   return "For";
    case EPRCSchema_SimpleFor:             return "SimpleFor";
    case EPRCSchema_If:                    return "If";
    case EPRCSchema_Else:                  return "Else";
    case EPRCSchema_Block_Start:           return "BlockStart";
    case EPRCSchema_Block_Version:         return "BlockVersion";
    case EPRCSchema_Block_End:             return "BlockEnd";
    case EPRCSchema_Value_Declare:         return "ValDeclare";
    case EPRCSchema_Value_Set:             return "ValSet";
    case EPRCSchema_Value_DeclareAndSet:   return "ValDeclSet";
    case EPRCSchema_Value:                 return "Val";
    case EPRCSchema_Value_Constant:        return "ValConst";
    case EPRCSchema_Value_For:             return "ValFor";
    case EPRCSchema_Value_CurveIs3D:       return "ValCurveIs3D";
    case EPRCSchema_Operator_MULT:         return "MULT";
    case EPRCSchema_Operator_DIV:          return "DIV";
    case EPRCSchema_Operator_ADD:          return "ADD";
    case EPRCSchema_Operator_SUB:          return "SUB";
    case EPRCSchema_Operator_LT:           return "LT";
    case EPRCSchema_Operator_LE:           return "LE";
    case EPRCSchema_Operator_GT:           return "GT";
    case EPRCSchema_Operator_GE:           return "GE";
    case EPRCSchema_Operator_EQ:           return "EQ";
    case EPRCSchema_Operator_NEQ:          return "NEQ";
    case EPRCSchema_Operator_IGNORE1:      return "IGN1";
    case EPRCSchema_Operator_IGNORE2:      return "IGN2";
    default:                               return NULL;
    }
}

/* Plain-English statement for a data-reading opcode, or NULL if the token is
   not a read. */
static const char *
schema_read_english(unsigned int token)
{
    switch (token)
    {
    case EPRCSchema_Data_Boolean:          return "read boolean";
    case EPRCSchema_Data_Double:           return "read double";
    case EPRCSchema_Data_Character:        return "read character";
    case EPRCSchema_Data_Unsigned_Integer: return "read uint32";
    case EPRCSchema_Data_Integer:          return "read int32";
    case EPRCSchema_Data_String:           return "read string";
    case EPRCSchema_Vector_2D:             return "read vector2d";
    case EPRCSchema_Vector_3D:             return "read vector3d";
    case EPRCSchema_Extent_1D:             return "read extent1d";
    case EPRCSchema_Extent_2D:             return "read extent2d";
    case EPRCSchema_Extent_3D:             return "read extent3d";
    case EPRCSchema_Ptr_Type:              return "read pointer (type)";
    case EPRCSchema_Ptr_Surface:           return "read pointer (surface)";
    case EPRCSchema_Ptr_Curve:             return "read pointer (curve)";
    default:                               return NULL;
    }
}

/* PRC version numbers are DATES, not arbitrary integers: 4.2 "Versioning"
   defines them as (year mod 2000) * 1000 + day-of-year. Rendering the date
   alongside the number is deliberate -- a bare 15083 reads as a magic
   constant and invites treating version comparisons as ordinary integer
   arithmetic, which is a trap this project and others have fallen into.
   Writes "YYYY-MM-DD" into out, or "not a valid date" if the day-of-year is
   out of range for that year (which does occur: values like 8500 appear in
   the wild and decode to day 500). */
static void
prc_version_to_date(unsigned int version, char *out, size_t out_size)
{
    static const int days_in_month[12] =
        { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    unsigned int year = 2000 + (version / 1000u);
    int day = (int)(version % 1000u);
    int leap, month, dim;

    leap = ((year % 4u) == 0u && ((year % 100u) != 0u || (year % 400u) == 0u)) ? 1 : 0;

    if (day < 1 || day > (365 + leap))
    {
        snprintf(out, out_size, "not a valid date");
        return;
    }

    for (month = 0; month < 12; month++)
    {
        dim = days_in_month[month] + ((month == 1) ? leap : 0);
        if (day <= dim)
            break;
        day -= dim;
    }
    snprintf(out, out_size, "%u-%02d-%02d", year, month + 1, day);
}

static void
indent_by(int depth)
{
    int i;
    printf("  ");
    for (i = 0; i < depth; i++)
        printf("  ");
}

/* Linear pseudo-code rendering. See the header's caveat: this indents on
   block/branch/loop openers and prints reads as statements, which is enough
   to read a program's shape, but it is not the VM's own operand grammar. */
static void
render_pseudocode(const prc_entity_schema *entry)
{
    uint32_t j;
    int depth = 0;

    for (j = 0; j < entry->token_count; j++)
    {
        unsigned int token = (unsigned int)entry->schema_tokens[j];
        const char *english = schema_read_english(token);

        if (english != NULL)
        {
            indent_by(depth);
            printf("%s\n", english);
            continue;
        }

        switch (token)
        {
        case EPRCSchema_Block_Start:
            indent_by(depth);
            printf("{\n");
            depth++;
            break;

        case EPRCSchema_Block_Version:
            /* Next token is the version this block was introduced at. The VM
               executes the body only when reader_version < that value; at or
               above it the block is skipped as already-understood. */
            indent_by(depth);
            if (j + 1 < entry->token_count)
            {
                unsigned int vers = (unsigned int)entry->schema_tokens[j + 1];
                char date[32];

                prc_version_to_date(vers, date, sizeof(date));
                printf("if (reader_version < %u) {          // fields added in v%u (%s)\n",
                       vers, vers, date);
                j++;
            }
            else
            {
                printf("if (reader_version < ?) {\n");
            }
            depth++;
            break;

        case EPRCSchema_Block_End:
            if (depth > 0)
                depth--;
            indent_by(depth);
            printf("}\n");
            break;

        case EPRCSchema_Parent_Type:
            indent_by(depth);
            if (j + 1 < entry->token_count)
            {
                printf("execute schema of parent type %u\n",
                       (unsigned int)entry->schema_tokens[j + 1]);
                j++;
            }
            else
            {
                printf("execute schema of parent type ?\n");
            }
            break;

        case EPRCSchema_If:
            /* Fold the whole condition onto one line. The tokens following
               If are a comparison operator and its operands; they run until
               the branch body opens (BlockStart) or the construct ends. One
               token per line is unreadable, and the condition is the part a
               human most wants to see whole. */
            {
                unsigned int k;
                int first = 1;

                indent_by(depth);
                printf("if (");
                for (k = j + 1; k < entry->token_count; k++)
                {
                    unsigned int t = (unsigned int)entry->schema_tokens[k];
                    const char *nm;

                    if (t == EPRCSchema_Block_Start || t == EPRCSchema_Block_End ||
                        t == EPRCSchema_Else || t == EPRCSchema_Block_Version)
                        break;

                    if (!first)
                        printf(" ");
                    first = 0;

                    switch (t)
                    {
                    case EPRCSchema_Operator_EQ:  printf("=="); break;
                    case EPRCSchema_Operator_NEQ: printf("!="); break;
                    case EPRCSchema_Operator_LT:  printf("<");  break;
                    case EPRCSchema_Operator_LE:  printf("<="); break;
                    case EPRCSchema_Operator_GT:  printf(">");  break;
                    case EPRCSchema_Operator_GE:  printf(">="); break;
                    case EPRCSchema_Value:        printf("var"); break;
                    case EPRCSchema_Value_Constant: printf("const"); break;
                    default:
                        nm = schema_opcode_name(t);
                        if (nm != NULL)
                            printf("%s", nm);
                        else
                            printf("%u", t);
                        break;
                    }
                }
                printf(")\n");
                j = k - 1;
            }
            break;

        case EPRCSchema_Else:
            indent_by(depth);
            printf("else\n");
            break;

        case EPRCSchema_For:
        case EPRCSchema_SimpleFor:
            indent_by(depth);
            printf("repeat <count> times\n");
            break;

        case EPRCSchema_Operator_IGNORE1:
            indent_by(depth);
            printf("ignore 1 operand\n");
            break;

        case EPRCSchema_Operator_IGNORE2:
            indent_by(depth);
            printf("ignore 2 operands\n");
            break;

        default:
            /* Operators, variable ops and inline operands: shown verbatim so
               nothing is silently dropped from the rendering. */
            {
                const char *name = schema_opcode_name(token);
                indent_by(depth);
                if (name != NULL)
                    printf("<%s>\n", name);
                else
                    printf("<operand %u>\n", token);
            }
            break;
        }
    }
}

int main(int argc, char **argv)
{
    prc_context *ctx;
    prc_api_data data;
    prc_schema *schema;
    uint32_t i, j;

    if (argc < 2)
    {
        printf("usage: dump_schema_programs <input.prc|input.pdf>\n");
        return 2;
    }

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
        return 1;

    data = prc_api_open_contents(ctx, argv[1]);
    if (data == NULL)
    {
        fprintf(stderr, "OPENFAIL %s\n", argv[1]);
        return 1;
    }

    /* The schema is parsed into the context while opening the container, so
       nothing further needs to be walked to see what a file declares. */
    schema = ctx->internal.schema;
    if (schema == NULL)
        return 0;

    for (i = 0; i < schema->schema_count; i++)
    {
        prc_entity_schema *entry = &schema->entity_schema[i];

        printf("PROG type=%u tokens=%u :", (unsigned)entry->entity_type,
               (unsigned)entry->token_count);
        for (j = 0; j < entry->token_count; j++)
        {
            unsigned int token = (unsigned int)entry->schema_tokens[j];
            const char *name = schema_opcode_name(token);

            if (name != NULL)
                printf(" %s", name);
            else
                printf(" #%u", token);
        }
        printf("\n");
        render_pseudocode(entry);
    }
    fflush(stdout);

    return 0;
}
