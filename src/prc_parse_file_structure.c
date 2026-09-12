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
#include <stdlib.h>
#include <string.h>
#include "prc_diag_env.h"
#include "prc_parse_file_structure.h"
#include "prc_parse_common.h"
#include "prc_parse_extra_geometry.h"
#include "prc_parse_global.h"
#include "prc_parse_tree.h"
#include "prc_parse_tess.h"
#include "prc_schema.h"
#include "debug.h"

#define DEBUG_MODEL_PARSING 1

static int
prc_parse_entity_schema(prc_context *ctx, prc_bit_state *bit_state, prc_entity_schema *data)
{
    size_t k;

    data->entity_type = prc_bitread_uint32(ctx, bit_state);
    data->token_count = prc_bitread_uint32(ctx, bit_state);

    if (data->token_count > 0)
    {
        data->schema_tokens = (unsigned int *)prc_calloc(ctx, data->token_count, sizeof(unsigned int));
        if (data->schema_tokens == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of schema_tokens in prc_parse_entity_schema\n");
            return PRC_ERROR_MEMORY;
        }
        for (k = 0; k < data->token_count; k++)
        {
            data->schema_tokens[k] = prc_bitread_uint32(ctx, bit_state);
        }
    }
    return 0;
}

/* Table 39 PRC_TYPE_ASM_FileStructureGlobals */
static int
prc_parse_file_globals(prc_context *ctx, prc_filestructure *file_struct, prc_bit_state *bit_state)
{
    uint32_t type;
    int code;
    int code_schema;
    uint32_t k;

    code_schema = prc_read_check_tag(ctx, bit_state, PRC_TYPE_ASM_FileStructureGlobals, &type);
    if (code_schema < 0)
    {
        prc_error(ctx, code_schema, "Error in prc_read_check_tag\n");
        return code_schema;
    }

    file_struct->globals = (prc_asm_file_structure_globals *)prc_calloc(ctx, 1, sizeof(prc_asm_file_structure_globals));
    if (file_struct->globals == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_globals\n");
        return PRC_ERROR_MEMORY;
    }

    file_struct->globals->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, bit_state, &file_struct->globals->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    /* count */
    file_struct->globals->file_count = prc_bitread_uint32(ctx, bit_state);

    if (file_struct->globals->file_count > 0)
    {
        file_struct->globals->unique_ids = (prc_unique_id *)prc_calloc(ctx, file_struct->globals->file_count, sizeof(prc_unique_id));
        if (file_struct->globals->unique_ids == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_globals\n");
            return PRC_ERROR_MEMORY;
        }

       for (k = 0; k < file_struct->globals->file_count; k++)
        {
            file_struct->globals->unique_ids[k] = prc_get_compressed_unique_id(ctx, bit_state);
        }
    }

    /* Now the actual global data */
    code = prc_parse_global_data(ctx, bit_state, &file_struct->globals->global_data, file_struct->header);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_global_data\n");
        return code;
    }

    /* Is at end */
    if (code_schema > 0)
    {
        int recursion = 0;
        code = prc_execute_schema(ctx, bit_state, prc_get_schema(ctx, code_schema - 1), &recursion);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_execute_schema\n");
            return code;
        }
    }
    return code;
}

/* Table 8 PRC_Schema */
int
prc_parse_file_schema(prc_context *ctx, prc_filestructure *file_struct, prc_bit_state *bit_state)
{
    int code;
    size_t k;
    prc_schema *data;

    /* Place the schema into the context, as we will need to make use of it during the parsing */
    /* Free the prior schema if there was one from another file section, as we are
       done with that one */
    data = ctx->internal.schema;
    if (data != NULL)
    {
        if (data->entity_schema != NULL)
        {
            for (k = 0; k < data->schema_count; k++)
            {
                if (data->entity_schema[k].schema_tokens != NULL)
                {
                    prc_free(ctx, data->entity_schema[k].schema_tokens);
                    data->entity_schema[k].schema_tokens = NULL;
                }
            }
            prc_free(ctx, data->entity_schema);
        }
        prc_free(ctx, data);
    }

    data = ctx->internal.schema = (prc_schema *)prc_calloc(ctx, 1, sizeof(prc_schema));
    if (data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_schema\n");
        return PRC_ERROR_MEMORY;
    }

    data->schema_count = prc_bitread_uint32(ctx, bit_state);
    if (data->schema_count > 0)
    {
        data->entity_schema = (prc_entity_schema *)prc_calloc(ctx, data->schema_count, sizeof(prc_entity_schema));
        if (data->entity_schema == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error of schema in prc_parse_file_schema\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->schema_count; k++)
        {
            code = prc_parse_entity_schema(ctx, bit_state, &data->entity_schema[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_schema\n");
                return code;
            }
        }
    }
#if DEBUG_MODEL_PARSING
    DEBUG_LOG2("schema_count = %u\n", data->schema_count);
    for (k = 0; k < data->schema_count; k++)
        DEBUG_LOG2("  entity_schema[%zu].entity_type = %u\n", k, data->entity_schema[k].entity_type);
#endif
    return 0;
}

static void
prc_parse_product_occurrence_reference(prc_context *ctx, prc_bit_state *bit_state,
                                        prc_product_occurrence_reference *data)
{
    data->unique_id = prc_get_compressed_unique_id(ctx, bit_state);
    data->root_index = prc_bitread_uint32(ctx, bit_state);
    data->product_occurence_is_active = prc_bitread_bit(ctx, bit_state);
}

/* Parse the prc_type_asm_modelfile Table 36 */
int
prc_parse_model_file(prc_context *ctx, prc_filestructure *file_struct,
                     uint32_t file_struct_count, uint32_t index)
{
    prc_bit_state bit_state;
    uint32_t type;
    int code;
    prc_type_asm_modelfile *data;

    prc_init_bit_state(ctx, &bit_state, file_struct->model_unzipped, file_struct->model_size);

    /* For some reason there is an extra uint32 at the start of this */
    {
        uint32_t leading_u32 = prc_bitread_uint32(ctx, &bit_state);
#if DEBUG_MODEL_PARSING
        DEBUG_LOG2("model_file leading_uint32 = %u\n", leading_u32);
#endif
    }

    code = prc_read_check_tag(ctx, &bit_state, PRC_TYPE_ASM_ModelFile, &type);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data = file_struct->model = (prc_type_asm_modelfile *)prc_calloc(ctx, 1, sizeof(prc_type_asm_modelfile));
    if (file_struct->model == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_model_file\n");
        return PRC_ERROR_MEMORY;
    }

    data->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, &bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    /* If the model does not have a name, then assign the name "model" to it */
    if (data->base.name.name.null_flag == 1)
    {
        data->base.name.name.null_flag = 0;
        data->base.name.name.string = (unsigned char *)prc_malloc(ctx, 6);
        if (data->base.name.name.string == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_model_file\n");
            return PRC_ERROR_MEMORY;
        }
        strcpy((char*) data->base.name.name.string, "model");
        data->base.name.same = 0;
        data->base.name.name.size = 6;
    }

    data->units_from_CAD_flag = prc_bitread_bit(ctx, &bit_state);
    data->units_from_CAD_file = prc_bitread_double(ctx, &bit_state);
#if DEBUG_MODEL_PARSING
    DEBUG_LOG2("model_file attribute_count=%u name.same=%u name.null=%u units_from_CAD_flag=%u units_from_CAD_file=%f\n",
        data->base.attribute_data.attribute_count, data->base.name.same, data->base.name.name.null_flag,
        data->units_from_CAD_flag, data->units_from_CAD_file);
#endif

    data->number_of_root_product_occurrences = prc_bitread_uint32(ctx, &bit_state);

#if DEBUG_MODEL_PARSING
    DEBUG_LOG2("Model file index = %d\n", index);
    DEBUG_LOG2("File UID = [%u %u %u %u]\n", file_struct->header->unique_id_file.unique_id0,
        file_struct->header->unique_id_file.unique_id1, file_struct->header->unique_id_file.unique_id2,
        file_struct->header->unique_id_file.unique_id3);
    DEBUG_LOG2("number_of_root_product_occurrences = %d\n", data->number_of_root_product_occurrences);
#endif

    if (data->number_of_root_product_occurrences > 0)
    {
        data->product_occurences = (prc_product_occurrence_reference *)prc_calloc(ctx,
            data->number_of_root_product_occurrences, sizeof(prc_product_occurrence_reference));
        if (data->product_occurences == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_model_file\n");
            return PRC_ERROR_MEMORY;
        }
        for (uint32_t k = 0; k < data->number_of_root_product_occurrences; k++)
        {
            prc_parse_product_occurrence_reference(ctx, &bit_state, &data->product_occurences[k]);
#if DEBUG_MODEL_PARSING
            DEBUG_LOG2("    data->product_occurences[%d].unique_id = [%d %d %d %d]\n", k,
                data->product_occurences[k].unique_id.unique_id0,
                data->product_occurences[k].unique_id.unique_id1,
                data->product_occurences[k].unique_id.unique_id2,
                data->product_occurences[k].unique_id.unique_id3);
            DEBUG_LOG2("    data->product_occurences[%d].root_index = %d\n", k, data->product_occurences[k].root_index);
            DEBUG_LOG2("    data->product_occurences[%d].product_occurence_is_active = %d\n", k, data->product_occurences[k].product_occurence_is_active);
#endif

            prc_unique_id unique_id;
            prc_unsigned_int root_index;
            uint8_t product_occurence_is_active;
        }

        data->file_structure_index_in_model_file = (uint32_t *)prc_calloc(ctx,
            file_struct_count, sizeof(uint32_t));
        if (data->file_structure_index_in_model_file == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_model_file\n");
            return PRC_ERROR_MEMORY;
        }
        for (uint32_t k = 0; k < file_struct_count; k++)
        {
            data->file_structure_index_in_model_file[k] = prc_bitread_uint32(ctx, &bit_state);
#if DEBUG_MODEL_PARSING
            DEBUG_LOG2("File structure index in model file [%d] = %d\n", k, data->file_structure_index_in_model_file[k]);
#endif
        }
#if DEBUG_MODEL_PARSING
        DEBUG_LOG2("\n");
#endif
    }

    /* And user data */
    code = prc_parse_user_data(ctx, &bit_state, &data->user_data);
    return code;
}

/* This is not documented. The schema and the globals are zipped into the same
   section. This is why I could never find the global tag. */
int
prc_parse_file_schema_and_global(prc_context *ctx, prc_filestructure *file_struct)
{
    prc_bit_state bit_state;
    int code;

    prc_init_bit_state(ctx, &bit_state, file_struct->schema_globals_unzipped,
                       file_struct->schema_globals_size);

    code = prc_parse_file_schema(ctx, file_struct, &bit_state);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_file_schema\n");
        return code;
    }

    code = prc_parse_file_globals(ctx, file_struct, &bit_state);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_file_globals\n");
        return code;
    }
    return 0;
}

/* Table 51 PRC_TYPE_ASM_FileStructureExtraGeometry */
int
prc_parse_file_extra_geometry(prc_context *ctx, prc_filestructure *file_struct)
{
    prc_bit_state bit_state;
    uint32_t type, k;
    int code;
    prc_asm_file_structure_extra_geometry *data;

    prc_init_bit_state(ctx, &bit_state, file_struct->extra_geometry_unzipped, file_struct->extra_geometry_size);

    code = prc_read_check_tag(ctx, &bit_state, PRC_TYPE_ASM_FileStructureExtraGeometry, &type);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data = file_struct->extra_geometry = (prc_asm_file_structure_extra_geometry *)prc_calloc(ctx, 1, sizeof(prc_asm_file_structure_extra_geometry));
    if (file_struct->extra_geometry == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_extra_geometry\n");
        return PRC_ERROR_MEMORY;
    }

    data->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, &bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    data->extra_geom_count = prc_bitread_uint32(ctx, &bit_state);
    if (data->extra_geom_count > 0)
    {
        data->extra_geom = (prc_extra_geometry *)prc_calloc(ctx, data->extra_geom_count, sizeof(prc_extra_geometry));
        if (data->extra_geom == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_extra_geometry\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->extra_geom_count; k++)
        {
            code = prc_parse_extra_geometry(ctx, &bit_state, &data->extra_geom[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_extra_geometry\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 38 PRC_TYPE_ASM_FileStructure */
static int
prc_parse_file_struct_internal_data(prc_context *ctx, prc_bit_state *bit_state, prc_type_asm_file_struct_internal_data *data)
{
    int code;

    code = prc_read_check_tag(ctx, bit_state, PRC_TYPE_ASM_FileStructure, &data->tag);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    code = prc_parse_content_prc_base(ctx, bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_content_prc_base\n");
        return code;
    }

    data->next_available_index = prc_bitread_uint32(ctx, bit_state);
    data->index_product_occurrence = prc_bitread_uint32(ctx, bit_state);

    return 0;
}


/* Table 47 PRC_TYPE_ASM_FileStructureTree */
int
prc_parse_file_tree(prc_context *ctx, prc_filestructure *file_struct)
{
    prc_bit_state bit_state;
    uint32_t type;
    int code;
    prc_asm_file_structure_tree *data;
    uint32_t k;

    prc_init_bit_state(ctx, &bit_state, file_struct->tree_unzipped, file_struct->tree_size);

    code = prc_read_check_tag(ctx, &bit_state, PRC_TYPE_ASM_FileStructureTree, &type);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data = file_struct->tree = (prc_asm_file_structure_tree *)prc_calloc(ctx, 1, sizeof(prc_asm_file_structure_tree));
    if (file_struct->tree == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_tree\n");
        return PRC_ERROR_MEMORY;
    }

    data->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, &bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    data->parts_count = prc_bitread_uint32(ctx, &bit_state);
    DEBUG_LOG2("Number of parts = %d\n", data->parts_count);
    if (data->parts_count > 0)
    {
        data->parts = (prc_asm_parts_definition *)prc_calloc(ctx, data->parts_count, sizeof(prc_asm_parts_definition));
        if (data->parts == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_tree\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->parts_count; k++)
        {
            code = prc_parse_parts(ctx, &bit_state, &data->parts[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_parts\n");
                return code;
            }
#if DEBUG_MODEL_PARSING
            DEBUG_LOG2("data->parts[%d] id = %d\n", k, data->parts[k].base.base.unique_id);
            if (data->parts[k].base.base.name.name.null_flag == 0)
            {
                DEBUG_LOG2("    Part %d name = %s\n", k, data->parts[k].base.base.name.name.string);
            }
            else
            {
                DEBUG_LOG2("    Part %d name = <null>\n", k);
            }

            if (data->parts[k].base.same_graphics)
            {
                DEBUG_LOG2("            Part has same graphics\n");
                DEBUG_LOG2("              Biased line style = %d \n", data->parts[k].base.graphics_content.biased_index_of_line_style);
                DEBUG_LOG2("              Biased layer index = %d \n", data->parts[k].base.graphics_content.biased_layer_index);
                DEBUG_LOG2("              Behavior bit 1 = %d \n", data->parts[k].base.graphics_content.behavior_bit_field1);
                DEBUG_LOG2("              Behavior bit 2 = %d \n", data->parts[k].base.graphics_content.behavior_bit_field2);

            }
            else
            {
                DEBUG_LOG2("            Part does not have same graphics\n");
                DEBUG_LOG2("              Biased line style = %d \n", data->parts[k].base.graphics_content.biased_index_of_line_style);
                DEBUG_LOG2("              Biased layer index = %d \n", data->parts[k].base.graphics_content.biased_layer_index);
                DEBUG_LOG2("              Behavior bit 1 = %d \n", data->parts[k].base.graphics_content.behavior_bit_field1);
                DEBUG_LOG2("              Behavior bit 2 = %d \n", data->parts[k].base.graphics_content.behavior_bit_field2);
            }

            for (int jj = 0; jj < data->parts[k].num_rep_items; jj++)
            {
                DEBUG_LOG2("        Representation item %d: type = %d, id = %d\n", jj,
                    data->parts[k].rep_items[jj].representation_type,
                    data->parts[k].rep_items[jj].item_content.base.base.unique_id);
                if (data->parts[k].rep_items[jj].item_content.base.base.name.name.null_flag == 0)
                {
                    DEBUG_LOG2("        name = %s\n", data->parts[k].rep_items[jj].item_content.base.base.name.name.string);
                }
                else
                {
                    DEBUG_LOG2("        name = <null>\n");
                }

                if (data->parts[k].rep_items[jj].item_content.base.same_graphics)
                {
                    DEBUG_LOG2("            RI has same graphics\n");
                    DEBUG_LOG2("              Biased line style = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.biased_index_of_line_style);
                    DEBUG_LOG2("              Biased layer index = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.biased_layer_index);
                    DEBUG_LOG2("              Behavior bit 1 = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.behavior_bit_field1);
                    DEBUG_LOG2("              Behavior bit 2 = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.behavior_bit_field2);

                }
                else
                {
                    DEBUG_LOG2("            RI does not have same graphics\n");
                    DEBUG_LOG2("              Biased line style = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.biased_index_of_line_style);
                    DEBUG_LOG2("              Biased layer index = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.biased_layer_index);
                    DEBUG_LOG2("              Behavior bit 1 = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.behavior_bit_field1);
                    DEBUG_LOG2("              Behavior bit 2 = %d \n", data->parts[k].rep_items[jj].item_content.base.graphics_content.behavior_bit_field2);
                }


                DEBUG_LOG2("        Coordinate system index = %d\n", data->parts[k].rep_items[jj].item_content.biased_index_local_coordinate_system);
                DEBUG_LOG2("        Tessellation index = %d\n", data->parts[k].rep_items[jj].item_content.biased_index_tessellation);
            }
#endif
        }
    }

    data->product_count = prc_bitread_uint32(ctx, &bit_state);
    DEBUG_LOG2("Number of products = %d\n", data->product_count);
    if (data->product_count > 0)
    {
        data->products = (prc_asm_product_occurrence *)prc_calloc(ctx, data->product_count, sizeof(prc_asm_product_occurrence));
        if (data->products == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_tree\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->product_count; k++)
        {
            code = prc_parse_product_occurrence(ctx, &bit_state, &data->products[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_product_occurrence\n");
                return code;
            }
#if DEBUG_MODEL_PARSING
            DEBUG_LOG2("         data->products[%d]  unique_id = %d\n", k,
                data->products[k].base.base.unique_id);
            if (data->products[k].base.base.name.name.null_flag == 0)
            {
                DEBUG_LOG2("         Product %d name = %s\n", k, data->products[k].base.base.name.name.string);
            }
            else
            {
                DEBUG_LOG2("         Product %d name = <null>\n", k);
            }
            if (data->products[k].base.same_graphics)
            {
                DEBUG_LOG2("            Product has same graphics\n");
                DEBUG_LOG2("              Biased line style = %d \n", data->products[k].base.graphics_content.biased_index_of_line_style);
                DEBUG_LOG2("              Biased layer index = %d \n", data->products[k].base.graphics_content.biased_layer_index);
                DEBUG_LOG2("              Behavior bit 1 = %d \n", data->products[k].base.graphics_content.behavior_bit_field1);
                DEBUG_LOG2("              Behavior bit 2 = %d \n", data->products[k].base.graphics_content.behavior_bit_field2);
            }
            else
            {
                DEBUG_LOG2("            Product does not have same graphics\n");
                DEBUG_LOG2("              Biased line style = %d \n", data->products[k].base.graphics_content.biased_index_of_line_style);
                DEBUG_LOG2("              Biased layer index = %d \n", data->products[k].base.graphics_content.biased_layer_index);
                DEBUG_LOG2("              Behavior bit 1 = %d \n", data->products[k].base.graphics_content.behavior_bit_field1);
                DEBUG_LOG2("              Behavior bit 2 = %d \n", data->products[k].base.graphics_content.behavior_bit_field2);
            }
            DEBUG_LOG2("         Product has transform = %d\n", data->products[k].has_transform);
            if (data->products[k].has_transform)
            {
                if (data->products[k].location.transformation_type == PRC_TYPE_MISC_CartesianTransformation)
                {
                    DEBUG_LOG2("         Transformation type = PRC_TYPE_MISC_CartesianTransformation\n");
                    DEBUG_LOG2("         Transform Behaviour = %d\n", data->products[k].location.cartesian.behavior);
                    char temp_behavior = data->products[k].location.cartesian.behavior;

                    if (temp_behavior & PRC_TRANSFORMATION_Translate)
                    {
                        DEBUG_LOG2("         Has translation\n");
                        DEBUG_LOG2("         translation = [%f %f %f]\n", data->products[k].location.cartesian.translation.x,
                            data->products[k].location.cartesian.translation.y, data->products[k].location.cartesian.translation.z);
                    }

                    if (temp_behavior & PRC_TRANSFORMATION_NonOrtho)
                    {
                        DEBUG_LOG2("         Has non-orthogonal matrix\n");
                        DEBUG_LOG2("         non-ortho matrix row 0 = [%f %f %f]\n", data->products[k].location.cartesian.non_ortho_matrix[0].x,
                            data->products[k].location.cartesian.non_ortho_matrix[0].y, data->products[k].location.cartesian.non_ortho_matrix[0].z);
                        DEBUG_LOG2("         non-ortho matrix row 1 = [%f %f %f]\n", data->products[k].location.cartesian.non_ortho_matrix[1].x,
                            data->products[k].location.cartesian.non_ortho_matrix[1].y, data->products[k].location.cartesian.non_ortho_matrix[1].z);
                        DEBUG_LOG2("         non-ortho matrix row 2 = [%f %f %f]\n", data->products[k].location.cartesian.non_ortho_matrix[2].x,
                            data->products[k].location.cartesian.non_ortho_matrix[2].y, data->products[k].location.cartesian.non_ortho_matrix[2].z);;
                    }

                    if (temp_behavior & PRC_TRANSFORMATION_Rotate)
                    {
                        DEBUG_LOG2("         Has rotation\n");
                        DEBUG_LOG2("         rotation[0] = [%f %f %f]\n", data->products[k].location.cartesian.rotation[0].x,
                            data->products[k].location.cartesian.rotation[0].y, data->products[k].location.cartesian.rotation[0].z);
                        DEBUG_LOG2("         rotation[1] = [%f %f %f]\n", data->products[k].location.cartesian.rotation[1].x,
                            data->products[k].location.cartesian.rotation[1].y, data->products[k].location.cartesian.rotation[1].z);
                    }

                    if (temp_behavior & PRC_TRANSFORMATION_NonUniformScale)
                    {
                        DEBUG_LOG2("         Has non-uniform scale\n");
                        DEBUG_LOG2("         non-uniform scale = [%f %f %f]\n", data->products[k].location.cartesian.non_uniform_scale.x,
                            data->products[k].location.cartesian.non_uniform_scale.y, data->products[k].location.cartesian.non_uniform_scale.z);
                    }

                    if (temp_behavior & PRC_TRANSFORMATION_Scale)
                    {
                        DEBUG_LOG2("         Has scale\n");
                        DEBUG_LOG2("         scale = %f\n", data->products[k].location.cartesian.scale);
                    }

                    if (temp_behavior & PRC_TRANSFORMATION_Homogeneous)
                    {
                        DEBUG_LOG2("         Has homogeneous matrix\n");
                        DEBUG_LOG2("         homogeneous matrix row 0 = [%f %f %f %f]\n", data->products[k].location.cartesian.homogeneous[0],
                            data->products[k].location.cartesian.homogeneous[1], data->products[k].location.cartesian.homogeneous[2],
                            data->products[k].location.cartesian.homogeneous[3]);
                    }
                }
                else if (data->products[k].location.transformation_type == PRC_TYPE_MISC_GeneralTransformation)
                {
                    DEBUG_LOG2("         Transformation type = PRC_TYPE_MISC_GeneralTransformation\n");
                    for (int jj = 0; jj < 4; jj++)
                    {
                        for (int ii = 0; ii < 4; ii++)
                        {
                            DEBUG_LOG2("         matrix[%d][%d] = %f\n", jj, ii, data->products[k].location.general_transformation.general_transform[jj * 4 + ii]);
                        }
                    }
                }
            }
#endif
        }
    }

#if DEBUG_MODEL_PARSING
    DEBUG_LOG2("\n");
#endif
    code = prc_parse_file_struct_internal_data(ctx, &bit_state, &data->internal_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_product_occurrence\n");
        return code;
    }

    code = prc_parse_user_data(ctx, &bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    return 0;
}

/* Table 48 PRC_TYPE_ASM_FileStructureTessellation */
int
prc_parse_file_tessellation(prc_context *ctx, prc_filestructure *file_struct, uint8_t debug_tess_in)
{
    prc_bit_state bit_state;
    uint32_t type;
    prc_asm_file_structure_tessellation *data;
    int code;
    uint32_t k;

    prc_init_bit_state(ctx, &bit_state, file_struct->tessellation_unzipped, file_struct->tessellation_size);

    code = prc_read_check_tag(ctx, &bit_state, PRC_TYPE_ASM_FileStructureTessellation, &type);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data = file_struct->tessellation = (prc_asm_file_structure_tessellation *)prc_calloc(ctx, 1, sizeof(prc_asm_file_structure_tessellation));
    if (file_struct->tessellation == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_tessellation\n");
        return PRC_ERROR_MEMORY;
    }

    file_struct->tessellation->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, &bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    data->tess_count = prc_bitread_uint32(ctx, &bit_state);
    if (data->tess_count > 0)
    {
        /* Each tessellation entry reads at least a 32-bit type tag up front
           (prc_parse_tess); reject a tess_count that couldn't possibly fit in
           what's left of this section so a single corrupted count field can't
           drive a huge calloc that then gets walked entry-by-entry on release.
           Found via PRC_ENABLE_UNZIPPED_FUZZ: a mutated tess_count of ~18M
           produced an 8GB+ mostly-zeroed array whose release loop printed an
           "Unknown Tessellation type" error tens of millions of times. */
        if (bit_state.bit_count <= 0 ||
            (uint64_t)data->tess_count > (uint64_t)bit_state.bit_count / 32)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "tess_count implausible for remaining tessellation section size\n");
            return PRC_ERROR_PARSE;
        }

        data->tess = (prc_tess *)prc_calloc(ctx, data->tess_count, sizeof(prc_tess));
        if (data->tess == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_tessellation\n");
            return PRC_ERROR_MEMORY;
        }

        uint8_t debug_tess = 0;
        for (k = 0; k < data->tess_count; k++)
        {
            code = prc_parse_tess(ctx, &bit_state, &data->tess[k], debug_tess_in);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_parse_tess\n");
                return code;
            }
        }
    }
    return 0;
}

/* Table 49 PRC_TYPE_ASM_FileStructureGeometry */
int
prc_parse_file_geometry(prc_context *ctx, prc_filestructure *file_struct)
{
    prc_bit_state bit_state;
    uint32_t type;
    int code;
    prc_asm_file_structure_geometry *data;

    prc_init_bit_state(ctx, &bit_state, file_struct->geometry_unzipped, file_struct->geometry_size);

    /* Names the origin that PRC_DIAG_CET_OFFSETS reports against, so a trace
       can be compared with another implementation without having to guess
       where a run of offsets is measured from. Bit positions in the [cet]
       lines that follow are counted from this point. The file structure index
       is printed by the caller in prc_parse_main.c, which is the only place
       that holds it. */
    if (prc_diag_getenv("PRC_DIAG_CET_OFFSETS") != NULL)
        fprintf(stderr, "[cet] --- geometry section begins, %u bytes ---\n",
            (unsigned)file_struct->geometry_size);

    code = prc_read_check_tag(ctx, &bit_state, PRC_TYPE_ASM_FileStructureGeometry, &type);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_read_check_tag\n");
        return code;
    }

    data = file_struct->geometry = (prc_asm_file_structure_geometry *)prc_calloc(ctx, 1, sizeof(prc_asm_file_structure_geometry));
    if (file_struct->geometry == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_file_geometry\n");
        return PRC_ERROR_MEMORY;
    }

    file_struct->geometry->tag = type;

    /* PRC Base */
    code = prc_parse_content_prc_base(ctx, &bit_state, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_base\n");
        return code;
    }

    code = prc_parse_exact_geometry(ctx, &bit_state, &data->exact_geometry);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_exact_geometry\n");
        return 0; /* Eat this */
        //return code;
    }

    code = prc_parse_user_data(ctx, &bit_state, &data->user_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Parsing error in prc_parse_user_data\n");
        return code;
    }

    return 0;
}
