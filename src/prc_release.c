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

#include <stdlib.h>
#include "prc_schema.h"

static void prc_release_prc_type_surface(prc_context *ctx, prc_type_surf *data);

void
prc_release_string(prc_context *ctx, prc_string *data)
{
    if (data->string != NULL && data->null_flag != 1)
    {
        prc_free(ctx, data->string);
    }
}

static void
prc_release_name(prc_context *ctx, prc_name *data)
{
    if (data == NULL)
        return;

    if (!data->same)
    {
        prc_release_string(ctx, &data->name);
    }
}

static void
prc_release_attribute_title(prc_context *ctx, prc_attribute_entry *data)
{
    if (!data->flag)
    {
        prc_release_string(ctx, &data->string_title);
    }
}

static void
prc_release_attribute_key_value(prc_context *ctx, prc_attribute_key_value *data)
{
    if (data == NULL)
        return;

    prc_release_attribute_title(ctx, &data->title);
    if (data->type == PRC_ATTRIBUTE_TYPE_CHAR_UTF8)
    {
        prc_release_string(ctx, &data->val_string);
    }
}

static void
prc_release_prc_ref_base(prc_context *ctx, prc_content_prc_ref_base *base)
{
    size_t k, j;

    if (base == NULL)
        return;

    prc_release_string(ctx, &base->name.name);
    prc_attribute_data attribute_data = base->attribute_data;

    for (k = 0; k < attribute_data.attribute_count; k++)
    {
        prc_release_attribute_title(ctx, &attribute_data.attributes[k].attribute_title);
        if (attribute_data.attributes[k].attributes != NULL)
        {
            for (j = 0; j < attribute_data.attributes[k].number_attributes; j++)
            {
                prc_release_attribute_key_value(ctx, &attribute_data.attributes[k].attributes[j]);
            }
        }
        prc_free(ctx, (attribute_data.attributes[k].attributes));
    }
    prc_free(ctx, (attribute_data.attributes));
}

static void
prc_release_base_with_graphics(prc_context *ctx, prc_base_with_graphics *data)
{
    if (data == NULL)
        return;

    prc_release_prc_ref_base(ctx, &data->base);
}

static void
prc_release_content_base(prc_context *ctx, prc_content_prc_base *base)
{
    size_t k, j;

    if (base == NULL)
        return;

    prc_release_string(ctx, &base->name.name);
    prc_attribute_data attribute_data = base->attribute_data;

    for (k = 0; k < attribute_data.attribute_count; k++)
    {
        prc_release_attribute_title(ctx, &attribute_data.attributes[k].attribute_title);
        if (attribute_data.attributes[k].attributes != NULL)
        {
            for (j = 0; j < attribute_data.attributes[k].number_attributes; j++)
            {
                prc_release_attribute_key_value(ctx, &attribute_data.attributes[k].attributes[j]);
            }
        }
        prc_free(ctx, (attribute_data.attributes[k].attributes));
    }
    prc_free(ctx, (attribute_data.attributes));
}

static void
prc_release_treat_types(prc_context *ctx, prc_graphics_information *treat_type)
{
    if (treat_type == NULL)
        return;

    if (treat_type->element_information != NULL)
    {
        prc_free(ctx, treat_type->element_information);
        treat_type->element_information = NULL;
    }
}

static void
prc_release_context_graphics(prc_context *ctx, prc_context_graphics *context_graphics)
{
    size_t k;

    if (context_graphics == NULL)
        return;

    for (k = 0; k < context_graphics->number_of_treat_type; k++)
    {
        prc_release_treat_types(ctx, &context_graphics->treat_types[k]);
    }
    prc_free(ctx, context_graphics->treat_types);
    context_graphics->treat_types = NULL;
}

static void
prc_release_file_struct_desc(prc_context *ctx, prc_file_struct_desc *file_struct_desc)
{
    if (file_struct_desc == NULL)
        return;

    if (file_struct_desc->section_offset != NULL)
    {
        prc_free(ctx, file_struct_desc->section_offset);
        file_struct_desc->section_offset = NULL;
    }
}

static void
prc_release_uncomp_block(prc_context *ctx, prc_uncomp_block *uncomp_block)
{
    if (uncomp_block == NULL)
        return;

    if (uncomp_block->block != NULL)
    {
        prc_free(ctx, uncomp_block->block);
    }
    if (uncomp_block->raw_image != NULL)
    {
        prc_free(ctx, uncomp_block->raw_image);
    }
}

static void
prc_release_uncomp_file(prc_context *ctx, prc_uncomp_file *uncomp_file)
{
    size_t k;

    if (uncomp_file == NULL)
        return;

    for (k = 0; k < uncomp_file->file_count; k++)
    {
        prc_release_uncomp_block(ctx, &uncomp_file->files[k]);
        if (uncomp_file->files[k].raw_image != NULL)
        {
            prc_free(ctx, uncomp_file->files[k].raw_image);
        }
        prc_free(ctx, uncomp_file);
    }
}

void
prc_release_header(prc_context *ctx, prc_header *header)
{
    uint32_t k;

    if (header == NULL)
        return;

    if (header->file_info != NULL)
    {
        for (k = 0; k < header->filestructure_count; k++)
        {
            prc_release_file_struct_desc(ctx, &header->file_info[k]);
        }
        prc_free(ctx, header->file_info);
    }

    prc_release_uncomp_file(ctx, header->files);
    if (header->files != NULL)
    {
        prc_free(ctx, header->files);
    }

    /* At this time release the schema too if there is one */
    prc_schema *data = ctx->internal.schema;
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
        ctx->internal.schema = NULL;
    }

    prc_free(ctx, header);
}

static void
prc_release_file_struct_header(prc_context *ctx, prc_file_structure_header *header)
{
    size_t k;

    if (header == NULL)
        return;

    for (k = 0; k < header->file_count; k++)
    {
        prc_release_uncomp_block(ctx, &header->files[k]);
    }
    prc_free(ctx, header->files);
}

static void
prc_release_entity_schema(prc_context *ctx, prc_entity_schema *entity_schema)
{
    if (entity_schema == NULL)
        return;

    if (entity_schema->schema_tokens != NULL)
    {
        prc_free(ctx, entity_schema->schema_tokens);
        entity_schema->schema_tokens = NULL;
    }
}

static void
prc_release_prc_ref_base_name(prc_context *ctx, prc_content_prc_ref_base *base)
{
    if (base->name.name.string != NULL)
    {
        prc_free(ctx, base->name.name.string);
        base->name.name.string = NULL;
        base->name.name.null_flag = true;
    }
}

static void
prc_release_font_keys_same_font(prc_context *ctx, prc_font_keys_same_font *data)
{
    if (data == NULL)
        return;

    prc_release_string(ctx, &data->font_name);
    if (data->font_key_list != NULL)
    {
        prc_free(ctx, data->font_key_list);
        data->font_key_list = NULL;
    }
}

static void
prc_release_serialize_help(prc_context *ctx, prc_markup_serialization_helper *data)
{
    uint32_t k;

    prc_release_string(ctx, &data->default_font_family_name);
    if (data->font_keys_of_font != NULL)
    {
        for (k = 0; k < data->font_keys_count; k++)
        {
            prc_release_font_keys_same_font(ctx, &data->font_keys_of_font[k]);
        }
        prc_free(ctx, data->font_keys_of_font);
    }
}

static void
prc_release_prc_pattern(prc_context *ctx, prc_graph_fill_pattern *data)
{
    uint32_t k;

    if (data == NULL)
        return;

    switch (data->fill_pattern_type)
    {

    case PRC_TYPE_GRAPH_DottingPattern:
        if (data->dotting_pattern != NULL)
        {
            prc_release_prc_ref_base(ctx, &data->dotting_pattern->base);
            prc_free(ctx, data->dotting_pattern);
            data->dotting_pattern = NULL;
        }
        break;

    case PRC_TYPE_GRAPH_VpicturePattern:
        if (data->picture_pattern != NULL)
        {
            prc_release_prc_ref_base(ctx, &data->picture_pattern->base);

            if (data->picture_pattern->markup.tessellation_coordinates.coordinates != NULL)
            {
                prc_free(ctx, data->picture_pattern->markup.tessellation_coordinates.coordinates);
                data->picture_pattern->markup.tessellation_coordinates.coordinates = NULL;
            }
            if (data->picture_pattern->markup.code_numbers != NULL)
            {
                prc_free(ctx, data->picture_pattern->markup.code_numbers);
                data->picture_pattern->markup.code_numbers = NULL;
            }
            if (data->picture_pattern->markup.text_strings != NULL)
            {
                for (k = 0; k < data->picture_pattern->markup.number_of_text_strings; k++)
                {
                    prc_release_string(ctx, &data->picture_pattern->markup.text_strings[k]);
                }
                prc_free(ctx, data->picture_pattern->markup.text_strings);
                data->picture_pattern->markup.text_strings = NULL;
            }
            prc_release_string(ctx, &data->picture_pattern->markup.tessellation_label);
            prc_free(ctx, data->picture_pattern);
            data->picture_pattern = NULL;
        }
        break;

    case PRC_TYPE_GRAPH_HatchingPattern:
        if (data->hatching_pattern != NULL)
        {
            prc_release_prc_ref_base(ctx, &data->hatching_pattern->base);
            if (data->hatching_pattern->hatch != NULL)
            {
                prc_free(ctx, data->hatching_pattern->hatch);
            }
            data->hatching_pattern = NULL;
        }
        break;

    default:
        /* No additional data to free */
        break;
    }
}

static void
prc_release_global_data(prc_context *ctx, prc_file_struct_internal_global_data *global_data)
{
    uint32_t k;

    if (global_data == NULL)
        return;

    prc_release_serialize_help(ctx, &global_data->serialize_help);

    if (global_data->colors != NULL)
    {
        prc_free(ctx, global_data->colors);
        global_data->colors = NULL;
    }
    if (global_data->materials != NULL)
    {
        for (k = 0; k < global_data->material_count; k++)
        {
            prc_release_prc_ref_base(ctx, &global_data->materials[k].base);
        }
        prc_free(ctx, global_data->materials);
        global_data->materials = NULL;
    }
    if (global_data->textures != NULL)
    {
        for (k = 0; k < global_data->texture_count; k++)
        {
            prc_release_prc_ref_base(ctx, &global_data->textures[k].base);
            if (global_data->textures[k].texture_mapping_attributes_intensities != NULL)
            {
                prc_free(ctx, global_data->textures[k].texture_mapping_attributes_intensities);
                global_data->textures[k].texture_mapping_attributes_intensities = NULL;
            }
            if (global_data->textures[k].texture_mapping_attributes_components != NULL)
            {
                prc_free(ctx, global_data->textures[k].texture_mapping_attributes_components);
                global_data->textures[k].texture_mapping_attributes_components = NULL;
            }
        }
        prc_free(ctx, global_data->textures);
        global_data->textures = NULL;
    }
    if (global_data->pictures != NULL)
    {
        for (k = 0; k < global_data->picture_count; k++)
        {
            prc_release_string(ctx, &global_data->pictures[k].base.name.name);
        }
        prc_free(ctx, global_data->pictures);
        global_data->pictures = NULL;
    }
    if (global_data->line_patterns != NULL)
    {
        for (k = 0; k < global_data->line_pattern_count; k++)
        {
            prc_release_prc_ref_base(ctx, &global_data->line_patterns[k].base);
            if (global_data->line_patterns[k].lengths != NULL)
            {
                prc_free(ctx, global_data->line_patterns[k].lengths);
                global_data->line_patterns[k].lengths = NULL;
            }
        }
        prc_free(ctx, global_data->line_patterns);
        global_data->line_patterns = NULL;
    }
    if (global_data->styles != NULL)
    {
        for (k = 0; k < global_data->style_count; k++)
        {
            prc_release_prc_ref_base(ctx, &global_data->styles[k].base);
        }
        prc_free(ctx, global_data->styles);
        global_data->styles = NULL;
    }
    if (global_data->fills != NULL)
    {
        for (k = 0; k < global_data->fill_count; k++)
        {
            prc_release_prc_pattern(ctx, &global_data->fills[k]);
        }
        prc_free(ctx, global_data->fills);
        global_data->fills = NULL;
    }
    if (global_data->ref_coords != NULL)
    {
        prc_free(ctx, global_data->ref_coords);
        global_data->ref_coords = NULL;
    }
}

static void
prc_release_file_struct_model(prc_context *ctx, prc_type_asm_modelfile *model)
{
    size_t k;

    if (model == NULL)
        return;

    if (model->product_occurences != NULL)
    {
        prc_free(ctx, model->product_occurences);
        model->product_occurences = NULL;
    }

    if (model->file_structure_index_in_model_file != NULL)
    {
        prc_free(ctx, model->file_structure_index_in_model_file);
        model->file_structure_index_in_model_file = NULL;
    }

    prc_release_content_base(ctx, &model->base);

    prc_free(ctx, model);
}

static void
prc_release_file_struct_globals(prc_context *ctx, prc_asm_file_structure_globals *globals)
{
    if (globals == NULL)
        return;

    if (globals->user_data.user_data != NULL)
    {
        prc_free(ctx, globals->user_data.user_data);
        globals->user_data.user_data = NULL;
    }
    prc_free(ctx, globals->unique_ids);
    globals->unique_ids = NULL;
    prc_release_global_data(ctx, &globals->global_data);
    prc_release_content_base(ctx, &globals->base);
    prc_free(ctx, globals);
}

static void
prc_release_internal_global_data(prc_context *ctx, prc_type_asm_file_struct_internal_data *global_data)
{

}


/* Work-list entry for the iterative release walk below: either a prc_ri node
   to release, or a deferred "free this rep_items array" marker. The marker is
   needed because rep_items is one contiguous allocation that individual
   elements point into -- it must not be freed until every element pushed
   from it has actually been popped and released, so it's pushed *before*
   its elements (LIFO: whatever is pushed last is popped first, so the
   elements all get processed before the marker is). */
typedef struct
{
    uint8_t is_cleanup;
    prc_ri *ri;
    prc_ri *array_to_free;
} prc_release_ri_work;

static int
prc_release_ri_push(prc_context *ctx, prc_release_ri_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint8_t is_cleanup,
    prc_ri *ri, prc_ri *array_to_free)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_release_ri_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_release_ri_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_release_ri_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].is_cleanup = is_cleanup;
    (*worklist)[*worklist_size].ri = ri;
    (*worklist)[*worklist_size].array_to_free = array_to_free;
    (*worklist_size)++;
    return 0;
}

/* Iterative (not recursive): RI_Set nesting (and RepresentationalItem
   wrapping) follows the file's structure, which is attacker-controlled and
   could otherwise drive unbounded C-stack recursion. Uses an explicit
   heap-allocated work list instead. On an out-of-memory failure growing the
   work list, this stops early (leaking the remainder) rather than crashing --
   matching this release path's existing best-effort-cleanup convention. */
static void
prc_release_representation_item(prc_context *ctx, prc_ri *data)
{
    prc_release_ri_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;

    worklist = (prc_release_ri_work *)prc_malloc(ctx, worklist_capacity * sizeof(prc_release_ri_work));
    if (worklist == NULL)
        return;
    if (prc_release_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity, 0, data, NULL) < 0)
    {
        prc_free(ctx, worklist);
        return;
    }

    while (worklist_size > 0)
    {
        prc_release_ri_work item = worklist[--worklist_size];
        prc_ri *curr;
        uint32_t k;

        if (item.is_cleanup)
        {
            prc_free(ctx, item.array_to_free);
            continue;
        }

        curr = item.ri;
        prc_release_base_with_graphics(ctx, &curr->item_content.base);

        switch (curr->representation_type)
        {
        case PRC_TYPE_RI_RepresentationalItem:
            if (prc_release_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                    0, curr->ri_rep_item, NULL) < 0)
            {
                prc_free(ctx, worklist);
                return;
            }
            break;
        case PRC_TYPE_RI_BrepModel:
            prc_free(ctx, curr->ri_brep_model);
            break;
        case PRC_TYPE_RI_Curve:
            prc_free(ctx, curr->ri_curve);
            break;
        case PRC_TYPE_RI_Direction:
            prc_free(ctx, curr->ri_direction);
            break;
        case PRC_TYPE_RI_Plane:
            prc_free(ctx, curr->ri_plane);
            break;
        case PRC_TYPE_RI_PointSet:
            if (curr->ri_point_set->points != NULL)
            {
                prc_free(ctx, curr->ri_point_set->points);
            }
            prc_free(ctx, curr->ri_point_set);
            break;
        case PRC_TYPE_RI_PolyBrepModel:
            prc_free(ctx, curr->ri_poly_brep_model);
            break;
        case PRC_TYPE_RI_PolyWire:
            prc_free(ctx, curr->ri_poly_wire);
            break;
        case PRC_TYPE_RI_Set:
            if (curr->ri_set->rep_items != NULL)
            {
                /* Push the cleanup marker first so it ends up at the bottom
                   of this batch and is popped only after all the elements
                   pushed after it. */
                if (prc_release_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        1, NULL, curr->ri_set->rep_items) < 0)
                {
                    prc_free(ctx, worklist);
                    return;
                }
                for (k = 0; k < curr->ri_set->number_of_items; k++)
                {
                    if (prc_release_ri_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                            0, &curr->ri_set->rep_items[k], NULL) < 0)
                    {
                        prc_free(ctx, worklist);
                        return;
                    }
                }
            }
            prc_free(ctx, curr->ri_set);
            break;
        case PRC_TYPE_RI_CoordinateSystem:
            prc_free(ctx, curr->ri_coordinate_system);
            break;
        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_release_representation_item\n");
        }
    }

    prc_free(ctx, worklist);
}

static void
prc_release_misc_reference_on_topology(prc_context *ctx, prc_misc_reference_on_topology *data)
{
    if (data->data.indices != NULL)
    {
        prc_free(ctx, data->data.indices);
    }
}

static void
prc_release_reference_data(prc_context *ctx, prc_reference_data *data)
{
    if (data->ref_type == PRC_TYPE_MISC_ReferenceOnTopology)
    {
        prc_release_misc_reference_on_topology(ctx, &data->topo_reference);
    }
}

static void
prc_release_content_entity_reference(prc_context *ctx, prc_content_entity_reference *data)
{
    prc_release_base_with_graphics(ctx, &data->base);

    if (data->flag)
    {
        prc_release_reference_data(ctx, &data->reference_data);
    }
}

static void
prc_release_content_extended_entity_ref(prc_context *ctx, prc_content_extended_entity_ref *data)
{
    prc_release_content_entity_reference(ctx, &data->content_entity_ref);

    if (data->has_reference_data)
    {
        prc_release_reference_data(ctx, &data->reference_data);
    }
}

static void
prc_release_link_item(prc_context *ctx, prc_misc_markup_linked_item *data)
{
    prc_release_content_extended_entity_ref(ctx, &data->content_entity_ref);
}

static void
prc_release_markup(prc_context *ctx, prc_mkp_markup *data)
{
    int k;

    prc_release_base_with_graphics(ctx, &data->base);
    if (data->linked_items != NULL)
    {
        prc_free(ctx, data->linked_items);
    }
    if (data->leaders != NULL)
    {
        prc_free(ctx, data->leaders);
    }
}

static void
prc_release_leader(prc_context *ctx, prc_mkp_leader *data)
{
    prc_release_base_with_graphics(ctx, &data->base);
}

/* Work-list entry for the iterative release walk below: either a
   prc_annotation_entity node to release, or a deferred "free this
   annotations array" marker -- needed for the same reason as
   prc_release_ri_work above (annotations is one contiguous allocation that
   individual elements point into, so it must outlive all of them). */
typedef struct
{
    uint8_t is_cleanup;
    prc_annotation_entity *entity;
    prc_annotation_entity *array_to_free;
} prc_release_annotation_work;

static int
prc_release_annotation_push(prc_context *ctx, prc_release_annotation_work **worklist,
    uint32_t *worklist_size, uint32_t *worklist_capacity, uint8_t is_cleanup,
    prc_annotation_entity *entity, prc_annotation_entity *array_to_free)
{
    if (*worklist_size >= *worklist_capacity)
    {
        prc_release_annotation_work *new_worklist;
        uint32_t new_capacity = (*worklist_capacity) * 2;
        new_worklist = (prc_release_annotation_work *)prc_realloc(ctx, *worklist,
            new_capacity * sizeof(prc_release_annotation_work));
        if (new_worklist == NULL)
            return -1;
        *worklist = new_worklist;
        *worklist_capacity = new_capacity;
    }
    (*worklist)[*worklist_size].is_cleanup = is_cleanup;
    (*worklist)[*worklist_size].entity = entity;
    (*worklist)[*worklist_size].array_to_free = array_to_free;
    (*worklist_size)++;
    return 0;
}

/* Iterative (not recursive): nested annotation sets follow the file's
   structure, which is attacker-controlled and could otherwise drive
   unbounded C-stack recursion. Uses an explicit heap-allocated work list
   instead. */
static void
prc_release_annotation_entities(prc_context *ctx, prc_annotation_entity *data)
{
    prc_release_annotation_work *worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;

    worklist = (prc_release_annotation_work *)prc_malloc(ctx,
        worklist_capacity * sizeof(prc_release_annotation_work));
    if (worklist == NULL)
        return;
    if (prc_release_annotation_push(ctx, &worklist, &worklist_size, &worklist_capacity, 0, data, NULL) < 0)
    {
        prc_free(ctx, worklist);
        return;
    }

    while (worklist_size > 0)
    {
        prc_release_annotation_work item = worklist[--worklist_size];
        prc_annotation_entity *curr;
        size_t k;

        if (item.is_cleanup)
        {
            prc_free(ctx, item.array_to_free);
            continue;
        }

        curr = item.entity;
        switch (curr->tag)
        {
        case PRC_TYPE_MKP_AnnotationItem:
            prc_release_base_with_graphics(ctx, &curr->item.base);
            break;
        case PRC_TYPE_MKP_AnnotationSet:
            prc_release_base_with_graphics(ctx, &curr->set.base);
            if (curr->set.annotations != NULL)
            {
                /* Push the cleanup marker first so it is popped only after
                   all the elements pushed after it. */
                if (prc_release_annotation_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                        1, NULL, curr->set.annotations) < 0)
                {
                    prc_free(ctx, worklist);
                    return;
                }
                for (k = 0; k < curr->set.number_of_annotations; k++)
                {
                    if (prc_release_annotation_push(ctx, &worklist, &worklist_size, &worklist_capacity,
                            0, &curr->set.annotations[k], NULL) < 0)
                    {
                        prc_free(ctx, worklist);
                        return;
                    }
                }
            }
            break;
        case PRC_TYPE_MKP_AnnotationReference:
            prc_release_base_with_graphics(ctx, &curr->ref.base);
            if (curr->ref.linked_items != NULL)
            {
                prc_free(ctx, curr->ref.linked_items);
            }
            break;
        }
    }

    prc_free(ctx, worklist);
}

static void
prc_release_attribute_data(prc_context *ctx, prc_attribute_data *data)
{
    size_t k, j;

    if (data->attributes != NULL)
    {
        for (k = 0; k < data->attribute_count; k++)
        {
            prc_release_string(ctx, &data->attributes[k].attribute_title.string_title);
            if (data->attributes[k].attributes != NULL)
            {
                for (j = 0; j < data->attributes[k].number_attributes; j++)
                {
                    prc_release_string(ctx, &data->attributes[k].attributes[j].title.string_title);
                    if (data->attributes[k].attributes[j].type == PRC_ATTRIBUTE_TYPE_CHAR_UTF8 &&
                        data->attributes[k].attributes[j].val_string.string != NULL)
                    {
                        prc_free(ctx, data->attributes[k].attributes[j].val_string.string);
                    }
                }
                prc_free(ctx, data->attributes[k].attributes);
            }
        }
        prc_free(ctx, data->attributes);
        data->attributes = NULL;
    }
}

static void
prc_release_content_surface(prc_context *ctx, prc_content_surface *data)
{
    if (data->has_base_geometry)
    {
        prc_release_attribute_data(ctx, &data->attribute_data);
        prc_release_string(ctx, &data->name.name);
    }
}

static void
prc_release_markup_data(prc_context *ctx, prc_markup_data *data)
{
    int k;

    if (data->linked_items != NULL)
    {
        for (k = 0; k < data->number_of_linked_items; k++)
        {
            prc_release_link_item(ctx, &data->linked_items[k]);
        }
        prc_free(ctx, data->linked_items);
    }

    if (data->leaders != NULL)
    {
        for (size_t k = 0; k < data->number_of_leaders; k++)
        {
            prc_release_leader(ctx, &data->leaders[k]);
        }
        prc_free(ctx, data->leaders);
    }

    if (data->markups != NULL)
    {
        size_t k;
        for (k = 0; k < data->number_of_markups; k++)
        {
            prc_release_markup(ctx, &data->markups[k]);
        }
        prc_free(ctx, data->markups);
    }

    if (data->annotation_entities != NULL)
    {
        for (k = 0; k < data->number_of_annotation_entities; k++)
        {
            prc_release_annotation_entities(ctx, &data->annotation_entities[k]);
        }
        prc_free(ctx, data->annotation_entities);
    }
}

static void
prc_release_graph_light_object(prc_context *ctx, prc_graph_light_object *data)
{
    prc_release_prc_ref_base(ctx, &data->base);
}

static void
prc_release_scene_display_parameters(prc_context *ctx, prc_scene_display_parameters *data)
{
    size_t k;

    prc_release_prc_ref_base(ctx, &data->base);

    if (data->lights != NULL)
    {
        for (k = 0; k < data->number_of_lights; k++)
        {
            prc_release_graph_light_object(ctx, &data->lights[k]);
        }
        prc_free(ctx, data->lights);
    }

    if (data->camera_defined)
    {
        prc_release_prc_ref_base(ctx, &data->camera.base);
    }

    if (data->clipping_planes != NULL)
    {
        for (k = 0; k < data->number_of_clipping_planes; k++)
        {
            prc_release_content_surface(ctx, &data->clipping_planes[k].curve_data);
        }
        prc_free(ctx, data->clipping_planes);
    }
    if (data->styles != NULL)
    {
        prc_free(ctx, data->styles);
    }
}

static void
prc_release_misc_entity_reference(prc_context *ctx, prc_misc_entity_reference *data)
{
    prc_release_content_entity_reference(ctx, &data->content_entity_ref);
}

static void
prc_release_content_entity_filter_item(prc_context *ctx, prc_content_entity_filter_item *data)
{
    uint32_t k;

    if (data->entities != NULL)
    {
       for (k = 0; k < data->number_of_entities; k++)
       {
           prc_release_misc_entity_reference(ctx, &data->entities[k]);
       }
       prc_free(ctx, data->entities);
    }
}

static void
prc_release_content_layer_filter_item(prc_context *ctx, prc_content_layer_filter_item *data)
{
    if (data->layers != NULL)
    {
        prc_free(ctx, data->layers);
    }
}

static void
prc_release_asm_filter(prc_context *ctx, prc_asm_filter *data)
{
    prc_release_prc_ref_base(ctx, &data->base);
    prc_release_content_entity_filter_item(ctx, &data->entity_filter);
    prc_release_content_layer_filter_item(ctx, &data->layer_filter);
}

static void
prc_release_mkp_view(prc_context *ctx, prc_mkp_view *data)
{
    size_t k;

    prc_release_base_with_graphics(ctx, &data->base);

    if (data->annotations != NULL)
    {
        prc_free(ctx, data->annotations);
    }

    if (data->has_parameters)
    {
        prc_release_scene_display_parameters(ctx, &data->scene_display_parameters);
    }

    if (data->linked_items != NULL)
    {
        prc_free(ctx, data->linked_items);
    }

    if (data->filters != NULL)
    {
        for (k = 0; k < data->number_of_filters; k++)
        {
            prc_release_asm_filter(ctx, &data->filters[k]);
        }
        prc_free(ctx, data->filters);
    }
}

static void
prc_release_part(prc_context *ctx, prc_asm_parts_definition *data)
{
    size_t k;

    prc_release_base_with_graphics(ctx, &data->base);

    for (k = 0; k < data->num_rep_items; k++)
    {
        prc_release_representation_item(ctx, &data->rep_items[k]);
    }

    if (data->rep_items != NULL)
    {
        prc_free(ctx, data->rep_items);
    }

    if (data->views != NULL)
    {
        for (k = 0; k < data->number_views; k++)
        {
            prc_release_mkp_view(ctx, &data->views[k]);
        }
        prc_free(ctx, data->views);
    }

    prc_release_markup_data(ctx, &data->markups);
}

static void
prc_release_reference_product_occurrence(prc_context *ctx, prc_references_of_product_occurrence *data)
{
    if (data->index_child_occurrence != NULL)
    {
        prc_free(ctx, data->index_child_occurrence);
    }
}

static void
prc_release_product_occurrence(prc_context *ctx, prc_asm_product_occurrence *data)
{
    size_t k;

    prc_release_base_with_graphics(ctx, &data->base);

    prc_release_reference_product_occurrence(ctx, &data->references_product_occurrence);

    if (data->entity_reference != NULL)
    {
        for (k = 0; k < data->entity_ref_count; k++)
        {
            prc_release_content_entity_reference(ctx, &data->entity_reference[k].content_entity_ref);
        }
        prc_free(ctx, data->entity_reference);
    }

    prc_release_markup_data(ctx, &data->markups);

    if (data->views != NULL)
    {
        for (k = 0; k < data->number_of_views; k++)
        {
            prc_release_mkp_view(ctx, &data->views[k]);
        }
        prc_free(ctx, data->views);
    }

    if (data->has_filter)
    {
        prc_release_asm_filter(ctx, &data->entity_filter);
    }

    if (data->display_filters != NULL)
    {
        for (k = 0; k < data->number_of_display_filters; k++)
        {
            prc_release_asm_filter(ctx, &data->display_filters[k]);
        }
        prc_free(ctx, data->display_filters);
    }

    if (data->scene_display_parameters != NULL)
    {
        for (k = 0; k < data->number_of_scene_parameters; k++)
        {
            prc_release_scene_display_parameters(ctx, &data->scene_display_parameters[k]);
        }
        prc_free(ctx, data->scene_display_parameters);
    }
}

static void
prc_release_file_struct_tree(prc_context *ctx, prc_asm_file_structure_tree *data)
{
    size_t k;

    if (data == NULL)
        return;

    if (data->user_data.user_data != NULL)
    {
        prc_free(ctx, data->user_data.user_data);
        data->user_data.user_data = NULL;
    }

    if (data->parts != NULL)
    {
        for (k = 0; k < data->parts_count; k++)
        {
            prc_release_part(ctx, &data->parts[k]);
        }
        prc_free(ctx, data->parts);
        data->parts = NULL;
    }

    if (data->products != NULL)
    {
        for (k = 0; k < data->product_count; k++)
        {
            prc_release_product_occurrence(ctx, &data->products[k]);
        }
        prc_free(ctx, data->products);
        data->products = NULL;
    }
    prc_release_internal_global_data(ctx, &data->internal_data);
    prc_release_content_base(ctx, &data->base);
    prc_free(ctx, data);
}

static void
prc_release_tess_face(prc_context *ctx, prc_tess_face data)
{
    if (data.line_attributes != NULL)
        prc_free(ctx, data.line_attributes);

    if (data.sizes_wire != NULL)
        prc_free(ctx, data.sizes_wire);

    if (data.triangulateddata != NULL)
        prc_free(ctx, data.triangulateddata);

    if (data.has_vertex_colors)
    {
        if (data.vertex_colors.color_data.remaining_vertices != NULL)
            prc_free(ctx, data.vertex_colors.color_data.remaining_vertices);
    }
}

static void
prc_release_tess_3d(prc_context *ctx, prc_tess_3d *data)
{
    if (data == NULL)
        return;

    if (data->tessellation_coordinates.coordinates != NULL)
        prc_free(ctx, data->tessellation_coordinates.coordinates);

    if (data->normal_coordinates != NULL)
        prc_free(ctx, data->normal_coordinates);

    if (data->wire_indices != NULL)
        prc_free(ctx, data->wire_indices);

    if (data->triangulated_index_array != NULL)
        prc_free(ctx, data->triangulated_index_array);

    if (data->texture_coordinates != NULL)
        prc_free(ctx, data->texture_coordinates);

    if (data->face_tessellation_data != NULL)
    {
        for (uint32_t k = 0; k < data->number_of_face_tessellation; k++)
        {
            prc_release_tess_face(ctx, data->face_tessellation_data[k]);
        }
        prc_free(ctx, data->face_tessellation_data);
    }
    prc_free(ctx, data);
}

static void
prc_release_tess_3d_wire(prc_context *ctx, prc_tess_3d_wire *data)
{
    if (data == NULL)
        return;

    if (data->tessellation_coordinates.coordinates != NULL)
        prc_free(ctx, data->tessellation_coordinates.coordinates);

    if (data->wire_elements != NULL)
    {
        for (uint32_t k = 0; k < data->number_of_wire_indexes; k++)
        {
            if (data->wire_elements[k].wire_indexes != NULL)
                prc_free(ctx, data->wire_elements[k].wire_indexes);
        }
        prc_free(ctx, data->wire_elements);
    }

    if (data->has_vertex_colors && data->vertex_color_data.color_data.remaining_vertices != NULL)
    {
        prc_free(ctx, data->vertex_color_data.color_data.remaining_vertices);
    }
    prc_free(ctx, data);
}

static void
prc_release_texture_data(prc_context *ctx, prc_compressed_texture_parameter *data)
{
    if (data == NULL)
        return;

    if (data->binary_texture_data != NULL)
    {
        if (data->binary_texture_data->texture_binary_data != NULL)
            prc_free(ctx, data->binary_texture_data->texture_binary_data);
        prc_free(ctx, data->binary_texture_data);
    }

    if (data->reference_array != NULL)
        prc_free(ctx, data->reference_array);

    if (data->texture_parameters != NULL)
        prc_free(ctx, data->texture_parameters);

    prc_free(ctx, data);
}

static void
prc_release_tess_3d_compressed(prc_context *ctx, prc_tess_3d_compressed *data)
{
    if (data == NULL)
        return;

    if (data->point_array != NULL)
        prc_free(ctx, data->point_array);

    if (data->edge_status_array != NULL)
        prc_free(ctx, data->edge_status_array);

    if (data->triangle_face_array != NULL)
        prc_free(ctx, data->triangle_face_array);

    if (data->points_is_reference_array != NULL)
        prc_free(ctx, data->points_is_reference_array);

    if (data->point_reference_array != NULL)
        prc_free(ctx, data->point_reference_array);

    if (data->normal_is_reversed != NULL)
        prc_free(ctx, data->normal_is_reversed);

    if (data->normal_binary_data != NULL)
        prc_free(ctx, data->normal_binary_data);

    if (data->normal_angle_array != NULL)
        prc_free(ctx, data->normal_angle_array);

    if (data->is_face_planar != NULL)
        prc_free(ctx, data->is_face_planar);

    if (data->is_point_color_on_face != NULL)
        prc_free(ctx, data->is_point_color_on_face);

    if (data->point_color_array != NULL)
        prc_free(ctx, data->point_color_array);

    if (data->decoded_point_color_array != NULL)
        prc_free(ctx, data->decoded_point_color_array);

    if (data->is_multiple_line_attribute_on_face != NULL)
        prc_free(ctx, data->is_multiple_line_attribute_on_face);

    if (data->line_attribute_array != NULL)
        prc_free(ctx, data->line_attribute_array);

    if (data->texture_data != NULL)
        prc_release_texture_data(ctx, data->texture_data);

    if (data->face_has_texture != NULL)
        prc_free(ctx, data->face_has_texture);

    if (data->has_behaviors)
        prc_free(ctx, data->behaviors_array);

    if (data->normal_angle_array_scaled != NULL)
        prc_free(ctx, data->normal_angle_array_scaled);

    if (data->uv_coordinates_3d != NULL)
        prc_free(ctx, data->uv_coordinates_3d);

    if (data->triangle_indices_prc_compressed_3d != NULL)
        prc_free(ctx, data->triangle_indices_prc_compressed_3d);

    if (data->point_colors_prc_compressed_3d != NULL)
        prc_free(ctx, data->point_colors_prc_compressed_3d);

    if (data->normal_indices_prc_compressed_3d != NULL)
        prc_free(ctx, data->normal_indices_prc_compressed_3d);

    if (data->vertices_prc_compressed_3d != NULL)
        prc_free(ctx, data->vertices_prc_compressed_3d);

    if (data->normals_prc_compressed_3d != NULL)
        prc_free(ctx, data->normals_prc_compressed_3d);

    if (data->triangle_styles != NULL)
        prc_free(ctx, data->triangle_styles);

    if (data->edge_vertices != NULL)
        prc_free(ctx, data->edge_vertices);

    if (data->edge_indices != NULL)
        prc_free(ctx, data->edge_indices);

    prc_free(ctx, data);
}

static void
prc_release_tess_markup(prc_context *ctx, prc_tess_markup *data)
{
    if (data == NULL)
        return;

    if (data->tessellation_coordinates.coordinates != NULL)
        prc_free(ctx, data->tessellation_coordinates.coordinates);

    if (data->code_numbers != NULL)
        prc_free(ctx, data->code_numbers);

    if (data->decode_primitives != NULL)
    {
        for (uint32_t k = 0; k < data->decode_number_primitives; k++)
        {
            if (data->decode_primitives[k].indices != NULL)
                prc_free(ctx, data->decode_primitives[k].indices);
            if (data->decode_primitives[k].text != NULL)
                prc_free(ctx, data->decode_primitives[k].text);
        }
        prc_free(ctx, data->decode_primitives);
    }

    if (data->decode_vertices)
        prc_free(ctx, data->decode_vertices);

    if (data->text_strings != NULL)
    {
        for (uint32_t k = 0; k < data->number_of_text_strings; k++)
        {
            prc_release_string(ctx, &data->text_strings[k]);
        }
        prc_free(ctx, data->text_strings);
    }

    if (data->tessellation_label.string != NULL)
        prc_release_string(ctx, &data->tessellation_label);

    prc_free(ctx, data);
}

static void
prc_release_tess(prc_context *ctx, prc_tess *data)
{
    if (data == NULL)
        return;

    switch (data->tess_type)
    {
    case PRC_TYPE_TESS_3D:
        prc_release_tess_3d(ctx, data->tess_3d);
        break;
    case PRC_TYPE_TESS_3D_Compressed:
        prc_release_tess_3d_compressed(ctx, data->tess_3d_compressed);
        break;
    case PRC_TYPE_TESS_3D_Wire:
        prc_release_tess_3d_wire(ctx, data->tess_3d_wire);
        break;
    case PRC_TYPE_TESS_MarkUp:
        prc_release_tess_markup(ctx, data->tess_markup);
        break;
    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Unknown Tessellation type\n");
        return;
    }
}

void prc_release_compressed_curve(prc_context *ctx, prc_compressed_curve *data);

/* Iterative (not recursive): composite curves nesting sub-curves follows the
   file's structure, which is attacker-controlled and could otherwise drive
   unbounded C-stack recursion. Uses an explicit heap-allocated work list
   instead. Unlike prc_release_representation_item/prc_release_annotation_entities,
   no cleanup-marker ordering is needed here: each curves[k].compressed_curve
   is its own independent allocation (not an element of a shared array that
   pointers are taken into), so freeing hcg_composite_curve.curves right after
   queuing its elements is safe. */
void
prc_release_compressed_curve(prc_context *ctx, prc_compressed_curve *data)
{
    prc_compressed_curve **worklist;
    uint32_t worklist_size = 0;
    uint32_t worklist_capacity = 64;

    if (data == NULL)
        return;

    worklist = (prc_compressed_curve **)prc_malloc(ctx, worklist_capacity * sizeof(prc_compressed_curve *));
    if (worklist == NULL)
        return;
    worklist[worklist_size++] = data;

    while (worklist_size > 0)
    {
        prc_compressed_curve *curr = worklist[--worklist_size];
        uint32_t k;

        if (curr == NULL)
            continue;

        switch (curr->curve_type)
        {
        case PRC_HCG_Line:
        case PRC_HCG_Circle:
            break;

        case  PRC_HCG_BsplineHermiteCurve:
            if (curr->hcg_bspline_hermite_curve.points != NULL)
                prc_free(ctx, curr->hcg_bspline_hermite_curve.points);
            if (curr->hcg_bspline_hermite_curve.tangents != NULL)
                prc_free(ctx, curr->hcg_bspline_hermite_curve.tangents);
            if (curr->hcg_bspline_hermite_curve.compressed_points != NULL)
                prc_free(ctx, curr->hcg_bspline_hermite_curve.compressed_points);
            if (curr->hcg_bspline_hermite_curve.compressed_tangents != NULL)
                prc_free(ctx, curr->hcg_bspline_hermite_curve.compressed_tangents);
            break;

        case PRC_HCG_CompositeCurve:
            if (curr->hcg_composite_curve.curves != NULL)
            {
                for (k = 0; k < curr->hcg_composite_curve.number_of_curves; k++)
                {
                    if (curr->hcg_composite_curve.curves[k].compressed_curve != NULL)
                    {
                        if (worklist_size >= worklist_capacity)
                        {
                            prc_compressed_curve **new_worklist;
                            worklist_capacity *= 2;
                            new_worklist = (prc_compressed_curve **)prc_realloc(ctx, worklist,
                                worklist_capacity * sizeof(prc_compressed_curve *));
                            if (new_worklist == NULL)
                            {
                                prc_free(ctx, worklist);
                                return;
                            }
                            worklist = new_worklist;
                        }
                        worklist[worklist_size++] = curr->hcg_composite_curve.curves[k].compressed_curve;
                    }
                }
                prc_free(ctx, curr->hcg_composite_curve.curves);
            }
            break;

        default:
            break;
        }
    }

    prc_free(ctx, worklist);
}

static void
prc_release_file_struct_tessallation(prc_context *ctx, prc_asm_file_structure_tessellation *data)
{
    size_t k;

    if (data == NULL)
        return;

    if (data->user_data.user_data != NULL)
    {
        prc_free(ctx, data->user_data.user_data);
        data->user_data.user_data = NULL;
    }

    if (data->tess != NULL)
    {
        for (k = 0; k < data->tess_count; k++)
        {
            prc_release_tess(ctx, &data->tess[k]);
        }
        prc_free(ctx, data->tess);
        data->tess = NULL;
    }
    prc_free(ctx, data);
}

static void
prc_release_base_topology(prc_context *ctx, prc_base_topology *data)
{
    if (data->has_base)
    {
        prc_release_attribute_data(ctx, &data->attribute_data);
        prc_release_name(ctx, &data->name);
    }
}

static void
prc_release_content_body(prc_context *ctx, prc_content_body *data)
{
    prc_release_base_topology(ctx, &data->base_topology);
}

static void
prc_release_hcg_bspline_hermite_curve(prc_context *ctx, prc_hcg_bspline_hermite_curve *data)
{
    if (data->points != NULL)
    {
        prc_free(ctx, data->points);
    }

    if (data->tangents != NULL)
    {
        prc_free(ctx, data->tangents);
    }
}

static void
prc_release_trim_loop_curves(prc_context *ctx, prc_ana_face_trim_loop *data)
{
    uint32_t k;

    if (data->trim_curves != NULL)
    {
        for (k = 0; k < data->curve_count; k++)
        {
            if (data->trim_curves[k].compressed_curve != NULL)
            {
                if (data->trim_curves[k].compressed_curve->curve_type == PRC_HCG_BsplineHermiteCurve)
                {
                    prc_release_hcg_bspline_hermite_curve(ctx, &data->trim_curves[k].compressed_curve->hcg_bspline_hermite_curve);
                }
                prc_free(ctx, data->trim_curves[k].compressed_curve);
            }
        }
        prc_free(ctx, data->trim_curves);
    }
}

static void
prc_release_content_compressed_ana_face(prc_context *ctx, prc_content_compressed_ana_face *data)
{
    uint32_t k;

    if (data->is_trimmed)
    {
        for (k = 0; k < data->trim_loop_count; k++)
        {
            prc_release_trim_loop_curves(ctx, &data->trim_loop[k]);
        }
        prc_free(ctx, data->trim_loop);
    }
}

static void
prc_release_compressed_knots(prc_context *ctx, prc_compressed_knots *data)
{
    if (data->compressed_knots != NULL)
    {
        prc_free(ctx, data->compressed_knots);
    }
}

static void
prc_release_compressed_control_points(prc_context *ctx, prc_compressed_control_points *data)
{
    if (data->ccpt_interior != NULL)
    {
        prc_free(ctx, data->ccpt_interior);
    }
    if (data->ccpt_in_v != NULL)
    {
        prc_free(ctx, data->ccpt_in_v);
    }
    if (data->ccpt_in_u != NULL)
    {
        prc_free(ctx, data->ccpt_in_u);
    }
}

static void
prc_release_compressed_weights(prc_context *ctx, prc_compressed_weights *data)
{
    if (data->compressed_weights != NULL)
    {
        prc_free(ctx, data->compressed_weights);
    }
}

static void
prc_release_compressed_nurbs(prc_context *ctx, prc_compressed_nurbs *data)
{
    if (data->mult_u != NULL)
    {
        prc_free(ctx, data->mult_u);
    }
    if (data->mult_v != NULL)
    {
        prc_free(ctx, data->mult_v);
    }
    prc_release_compressed_control_points(ctx, &data->compressed_control_points);
    prc_release_compressed_knots(ctx, &data->knot_vector_u.knots);
    prc_release_compressed_knots(ctx, &data->knot_vector_v.knots);
    prc_release_compressed_weights(ctx, &data->compressed_weights);
}

static void
prc_release_compressed_face(prc_context *ctx, prc_compressed_face *data)
{
    /* The ANA ones are the only ones that do allocations */
    switch (data->tag)
    {
    case PRC_HCG_AnaPlane:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_plane.face.ana_face);
        break;
    case PRC_HCG_AnaCylinder:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_cylinder.face.ana_face);
        break;
    case PRC_HCG_AnaCone:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_cone.face.ana_face);
        break;
    case PRC_HCG_AnaSphere:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_sphere.face.ana_face);
        break;
    case PRC_HCG_AnaTorus:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_torus.face.ana_face);
        break;
    case PRC_HCG_AnaNURBS:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_nurbs.face.ana_face);
        prc_release_compressed_nurbs(ctx, &data->hcg_ana_nurbs.compressed_surface);
        break;
    case PRC_HCG_AnaGenericFace:
        prc_release_content_compressed_ana_face(ctx, &data->hcg_ana_generic_face.face.ana_face);
        prc_release_prc_type_surface(ctx, &data->hcg_ana_generic_face.surface);
        break;
    case PRC_HCG_IsoNURBS:
        prc_release_compressed_nurbs(ctx, &data->hcg_iso_nurbs.surface);
    }
}

static void
prc_release_compressed_shell(prc_context *ctx, prc_compressed_shell *data)
{
    uint32_t k;

    if (data->faces != NULL)
    {
        for (k = 0; k < data->number_of_faces; k++)
        {
            prc_release_compressed_face(ctx, &data->faces[k]);
        }
        prc_free(ctx, data->faces);
        prc_free(ctx, data->is_iso_face);
    }
}

static void
prc_release_compressed_connex(prc_context *ctx, prc_compressed_connex *data)
{
    uint32_t k;
    uint32_t num_shells = data->number_of_shells;

    for (k = 0; k < num_shells; k++)
    {
        prc_release_compressed_shell(ctx, &data->shells[k]);
    }
    if (data->shells != NULL)
    {
        prc_free(ctx, data->shells);
    }
}

static void
prc_release_multiple_connex(prc_context *ctx, prc_multi_compressed_connex *data)
{
    uint32_t num_connex = data->number_of_connex;
    uint32_t k;

    for (k = 0; k < num_connex; k++)
    {
        prc_release_compressed_connex(ctx, &data->connex[k]);
    }
    if (data->connex != NULL)
    {
        prc_free(ctx, data->connex);
    }
}

static void
prc_release_brep_data_compress(prc_context *ctx, prc_topo_brep_data_compress *data)
{
    uint32_t k;

    prc_release_content_body(ctx, &data->base);

    if (data->single_connex_test)
    {
        prc_release_compressed_shell(ctx, &data->single_connex);
    }
    else
    {
        prc_release_multiple_connex(ctx, &data->multi_connex);
    }

    if (data->base_topology != NULL)
    {
        for (k = 0; k < data->number_of_faces; k++)
        {
            prc_release_base_topology(ctx, &data->base_topology[k]);
        }
        prc_free(ctx, data->base_topology);
    }
}

static void
prc_release_nano_brep_data(prc_context *ctx, prc_nano_brep_compressed_data *data)
{
    int k;

    if (data->vertices != NULL)
    {
        prc_free(ctx, data->vertices);
        data->vertices = NULL;
    }
    if (data->curves != NULL)
    {
        prc_free(ctx, data->curves);
        data->curves = NULL;
    }
}

/* PRC_TYPE_TOPO_Body self-nests to a depth that mirrors prc_parse_topo's
   parsing (which caps at PRC_TOPO_BODY_RECURSION_MAX = 256), so this is
   already transitively bounded for structures this codebase parsed itself --
   the cap here is defense-in-depth so this release function isn't the only
   thing relying on that cross-file invariant staying true. */
#define PRC_RELEASE_TOPO_BODY_RECURSION_MAX 256
static void prc_release_topo(prc_context *ctx, prc_topo *body, int depth);
static void prc_release_ptr_curve(prc_context *ctx, prc_ptr_curve *data);
static void prc_release_ptr_surface(prc_context *ctx, prc_ptr_surface *data);

static void
prc_release_content_curve(prc_context *ctx, prc_content_curve *data)
{
    if (data == NULL)
        return;

    if (data->has_base_geometry)
    {
        prc_release_attribute_data(ctx, &data->attribute_data);
        prc_release_name(ctx, &data->name);
    }
}

static void
prc_release_surf_blend01(prc_context *ctx, prc_surf_blend01 *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->center_curve);
    prc_release_ptr_curve(ctx, &data->origin_curve);
    prc_release_ptr_curve(ctx, &data->tangent_curve);
}

static void
prc_release_surf_blend02(prc_context *ctx, prc_surf_blend02 *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->bound_surface0);
    prc_release_ptr_curve(ctx, &data->bound_curve0);
    prc_release_ptr_surface(ctx, &data->bound_surface1);
    prc_release_ptr_curve(ctx, &data->bound_curve1);
    prc_release_ptr_curve(ctx, &data->center_curve);
    prc_release_ptr_surface(ctx, &data->cliff_surface0);
    prc_release_ptr_surface(ctx, &data->cliff_surface1);
}

static void
prc_release_surf_blend03(prc_context *ctx, prc_surf_blend03 *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    if (data->parameters != NULL)
        prc_free(ctx, data->parameters);
    if (data->multiplicities != NULL)
        prc_free(ctx, data->multiplicities);
    if (data->points != NULL)
        prc_free(ctx, data->points);
    if (data->rail_2_angles_v != NULL)
        prc_free(ctx, data->rail_2_angles_v);
    if (data->tangents != NULL)
        prc_free(ctx, data->tangents);
    if (data->rail_2_derivatives_v != NULL)
        prc_free(ctx, data->rail_2_derivatives_v);
    if (data->second_derivatives != NULL)
        prc_free(ctx, data->second_derivatives);
    if (data->rail_2_second_derivatives != NULL)
        prc_free(ctx, data->rail_2_second_derivatives);
    if (data->supplemental_doubles != NULL)
        prc_free(ctx, data->supplemental_doubles);
}

static void
prc_release_surf_nurbs(prc_context *ctx, prc_surf_nurbs *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    if (data->p != NULL)
        prc_free(ctx, data->p);
    if (data->knot_vector_u != NULL)
        prc_free(ctx, data->knot_vector_u);
    if (data->knot_vector_v != NULL)
        prc_free(ctx, data->knot_vector_v);
}

static void
prc_release_surf_cone(prc_context *ctx, prc_surf_cone *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
}

static void
prc_release_surf_cylinder(prc_context *ctx, prc_surf_cylinder *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
}

static void
prc_release_surf_cylindrical(prc_context *ctx, prc_surf_cylindrical *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->base_surface);
}

static void
prc_release_surf_offset(prc_context *ctx, prc_surf_offset *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->base_surface);
}

static void
prc_release_surf_pipe(prc_context *ctx, prc_surf_pipe *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->center_curve);
    prc_release_ptr_curve(ctx, &data->origin_curve);
}

static void
prc_release_surf_plane(prc_context *ctx, prc_surf_plane *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
}

static void
prc_release_surf_ruled(prc_context *ctx, prc_surf_ruled *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->first_curve);
    prc_release_ptr_curve(ctx, &data->second_curve);
}

static void
prc_release_surf_sphere(prc_context *ctx, prc_surf_sphere *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
}

static void
prc_release_surf_revolution(prc_context *ctx, prc_surf_revolution *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->base_curve);
}

static void
prc_release_surf_extrusion(prc_context *ctx, prc_surf_extrusion *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->base_curve);
}

static void
prc_release_surf_fromcurves(prc_context *ctx, prc_surf_fromcurves *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->first_curve);
    prc_release_ptr_curve(ctx, &data->second_curve);
}

static void
prc_release_surf_torus(prc_context *ctx, prc_surf_torus *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
}

static void
prc_release_surf_transform(prc_context *ctx, prc_surf_transform *data)
{
    if (data == NULL)
        return;
    prc_release_content_surface(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->base_surface);
}

static void
prc_release_prc_type_surface(prc_context *ctx, prc_type_surf *data)
{
    if (data == NULL)
        return;

    switch (data->surface_type)
    {
    case PRC_TYPE_ROOT:
        break;
    case PRC_TYPE_SURF_Blend01:
        prc_release_surf_blend01(ctx, data->surf_blend01);
        prc_free(ctx, data->surf_blend01);
        data->surf_blend01 = NULL;
        break;
    case PRC_TYPE_SURF_Blend02:
        prc_release_surf_blend02(ctx, data->surf_blend02);
        prc_free(ctx, data->surf_blend02);
        data->surf_blend02 = NULL;
        break;
    case PRC_TYPE_SURF_Blend03:
        prc_release_surf_blend03(ctx, data->surf_blend03);
        prc_free(ctx, data->surf_blend03);
        data->surf_blend03 = NULL;
        break;
    case PRC_TYPE_SURF_NURBS:
        prc_release_surf_nurbs(ctx, data->surf_nurbs);
        prc_free(ctx, data->surf_nurbs);
        data->surf_nurbs = NULL;
        break;
    case PRC_TYPE_SURF_Cone:
        prc_release_surf_cone(ctx, data->surf_cone);
        prc_free(ctx, data->surf_cone);
        data->surf_cone = NULL;
        break;
    case PRC_TYPE_SURF_Cylinder:
        prc_release_surf_cylinder(ctx, data->surf_cylinder);
        prc_free(ctx, data->surf_cylinder);
        data->surf_cylinder = NULL;
        break;
    case PRC_TYPE_SURF_Cylindrical:
        prc_release_surf_cylindrical(ctx, data->surf_cylindrical);
        prc_free(ctx, data->surf_cylindrical);
        data->surf_cylindrical = NULL;
        break;
    case PRC_TYPE_SURF_Offset:
        prc_release_surf_offset(ctx, data->surf_offset);
        prc_free(ctx, data->surf_offset);
        data->surf_offset = NULL;
        break;
    case PRC_TYPE_SURF_Pipe:
        prc_release_surf_pipe(ctx, data->surf_pipe);
        prc_free(ctx, data->surf_pipe);
        data->surf_pipe = NULL;
        break;
    case PRC_TYPE_SURF_Plane:
        prc_release_surf_plane(ctx, data->surf_plane);
        prc_free(ctx, data->surf_plane);
        data->surf_plane = NULL;
        break;
    case PRC_TYPE_SURF_Ruled:
        prc_release_surf_ruled(ctx, data->surf_ruled);
        prc_free(ctx, data->surf_ruled);
        data->surf_ruled = NULL;
        break;
    case PRC_TYPE_SURF_Sphere:
        prc_release_surf_sphere(ctx, data->surf_sphere);
        prc_free(ctx, data->surf_sphere);
        data->surf_sphere = NULL;
        break;
    case PRC_TYPE_SURF_Revolution:
        prc_release_surf_revolution(ctx, data->surf_revolution);
        prc_free(ctx, data->surf_revolution);
        data->surf_revolution = NULL;
        break;
    case PRC_TYPE_SURF_Extrusion:
        prc_release_surf_extrusion(ctx, data->surf_extrusion);
        prc_free(ctx, data->surf_extrusion);
        data->surf_extrusion = NULL;
        break;
    case PRC_TYPE_SURF_FromCurves:
        prc_release_surf_fromcurves(ctx, data->surf_fromcurves);
        prc_free(ctx, data->surf_fromcurves);
        data->surf_fromcurves = NULL;
        break;
    case PRC_TYPE_SURF_Torus:
        prc_release_surf_torus(ctx, data->surf_torus);
        prc_free(ctx, data->surf_torus);
        data->surf_torus = NULL;
        break;
    case PRC_TYPE_SURF_Transform:
        prc_release_surf_transform(ctx, data->surf_transform);
        prc_free(ctx, data->surf_transform);
        data->surf_transform = NULL;
        break;
    default:
        break;
    }
}

static void
prc_release_ptr_surface(prc_context *ctx, prc_ptr_surface *data)
{
    if (data == NULL || data->is_referenced)
        return;

    prc_release_prc_type_surface(ctx, &data->surface);
}

static void
prc_release_crv_blend02_boundary(prc_context *ctx, prc_crv_blend02_boundary *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->surface);
    prc_release_ptr_surface(ctx, &data->bound_surface);
    if (data->crossing_points != NULL)
        prc_free(ctx, data->crossing_points);
}

static void
prc_release_crv_nurbs(prc_context *ctx, prc_crv_nurbs *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    if (data->p != NULL)
        prc_free(ctx, data->p);
    if (data->u != NULL)
        prc_free(ctx, data->u);
}

static void
prc_release_crv_circle(prc_context *ctx, prc_crv_circle *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_composite(prc_context *ctx, prc_crv_composite *data)
{
    uint32_t k;
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    if (data->subcurves != NULL)
    {
        for (k = 0; k < data->number_of_subcurves; k++)
        {
            prc_release_ptr_curve(ctx, &data->subcurves[k].ptr_curve);
        }
        prc_free(ctx, data->subcurves);
    }
}

static void
prc_release_crv_onsurf(prc_context *ctx, prc_crv_onsurf *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->uv_curve);
    prc_release_ptr_surface(ctx, &data->surface);
}

static void
prc_release_crv_ellipse(prc_context *ctx, prc_crv_ellipse *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_equation(prc_context *ctx, prc_crv_equation *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_helix01(prc_context *ctx, prc_crv_helix01 *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_hyperbola(prc_context *ctx, prc_crv_hyperbola *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_intersection(prc_context *ctx, prc_crv_intersection *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    prc_release_ptr_surface(ctx, &data->surface1);
    prc_release_ptr_surface(ctx, &data->surface2);
    if (data->crossing_points != NULL)
        prc_free(ctx, data->crossing_points);
}

static void
prc_release_crv_line(prc_context *ctx, prc_crv_line *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_offset(prc_context *ctx, prc_crv_offset *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->base_curve);
}

static void
prc_release_crv_parabola(prc_context *ctx, prc_crv_parabola *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
}

static void
prc_release_crv_polyline(prc_context *ctx, prc_crv_polyline *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    if (data->points != NULL)
        prc_free(ctx, data->points);
}

static void
prc_release_crv_transform(prc_context *ctx, prc_crv_transform *data)
{
    if (data == NULL)
        return;
    prc_release_content_curve(ctx, &data->curve_data);
    prc_release_ptr_curve(ctx, &data->base_curve);
}

static void
prc_release_ptr_curve(prc_context *ctx, prc_ptr_curve *data)
{
    if (data == NULL || data->is_referenced)
        return;

    switch (data->curve_type)
    {
    case PRC_TYPE_ROOT:
        break;
    case PRC_TYPE_CRV_Blend02Boundary:
        prc_release_crv_blend02_boundary(ctx, data->crv_blend02_boundary);
        prc_free(ctx, data->crv_blend02_boundary);
        data->crv_blend02_boundary = NULL;
        break;
    case PRC_TYPE_CRV_NURBS:
        prc_release_crv_nurbs(ctx, data->crv_nurbs);
        prc_free(ctx, data->crv_nurbs);
        data->crv_nurbs = NULL;
        break;
    case PRC_TYPE_CRV_Circle:
        prc_release_crv_circle(ctx, data->crv_circle);
        prc_free(ctx, data->crv_circle);
        data->crv_circle = NULL;
        break;
    case PRC_TYPE_CRV_Composite:
        prc_release_crv_composite(ctx, data->crv_composite);
        prc_free(ctx, data->crv_composite);
        data->crv_composite = NULL;
        break;
    case PRC_TYPE_CRV_OnSurf:
        prc_release_crv_onsurf(ctx, data->crv_onsurf);
        prc_free(ctx, data->crv_onsurf);
        data->crv_onsurf = NULL;
        break;
    case PRC_TYPE_CRV_Ellipse:
        prc_release_crv_ellipse(ctx, data->crv_ellipse);
        prc_free(ctx, data->crv_ellipse);
        data->crv_ellipse = NULL;
        break;
    case PRC_TYPE_CRV_Equation:
        prc_release_crv_equation(ctx, data->crv_equation);
        prc_free(ctx, data->crv_equation);
        data->crv_equation = NULL;
        break;
    case PRC_TYPE_CRV_Helix01:
        prc_release_crv_helix01(ctx, data->crv_helix01);
        prc_free(ctx, data->crv_helix01);
        data->crv_helix01 = NULL;
        break;
    case PRC_TYPE_CRV_Hyperbola:
        prc_release_crv_hyperbola(ctx, data->crv_hyperbola);
        prc_free(ctx, data->crv_hyperbola);
        data->crv_hyperbola = NULL;
        break;
    case PRC_TYPE_CRV_Intersection:
        prc_release_crv_intersection(ctx, data->crv_intersection);
        prc_free(ctx, data->crv_intersection);
        data->crv_intersection = NULL;
        break;
    case PRC_TYPE_CRV_Line:
        prc_release_crv_line(ctx, data->crv_line);
        prc_free(ctx, data->crv_line);
        data->crv_line = NULL;
        break;
    case PRC_TYPE_CRV_Offset:
        prc_release_crv_offset(ctx, data->crv_offset);
        prc_free(ctx, data->crv_offset);
        data->crv_offset = NULL;
        break;
    case PRC_TYPE_CRV_Parabola:
        prc_release_crv_parabola(ctx, data->crv_parabola);
        prc_free(ctx, data->crv_parabola);
        data->crv_parabola = NULL;
        break;
    case PRC_TYPE_CRV_PolyLine:
        prc_release_crv_polyline(ctx, data->crv_polyline);
        prc_free(ctx, data->crv_polyline);
        data->crv_polyline = NULL;
        break;
    case PRC_TYPE_CRV_Transform:
        prc_release_crv_transform(ctx, data->crv_transform);
        prc_free(ctx, data->crv_transform);
        data->crv_transform = NULL;
        break;
    default:
        break;
    }
}

static void
prc_release_ptr_topology(prc_context *ctx, prc_ptr_topology *data)
{
    if (data == NULL)
        return;

    if (!data->is_stored && data->topo != NULL)
    {
        prc_release_topo(ctx, data->topo, 0);
        prc_free(ctx, data->topo);
        data->topo = NULL;
    }
}

static void
prc_release_content_wire_edge(prc_context *ctx, prc_content_wire_edge *data)
{
    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    prc_release_ptr_curve(ctx, &data->ptr_curve);
}

static void
prc_release_topo_multiple_vertex(prc_context *ctx, prc_topo_multiple_vertex *data)
{
    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    if (data->points != NULL)
    {
        prc_free(ctx, data->points);
        data->points = NULL;
    }
}

static void
prc_release_topo_unique_vertex(prc_context *ctx, prc_topo_unique_vertex *data)
{
    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
}

static void
prc_release_topo_wire_edge(prc_context *ctx, prc_topo_wire_edge *data)
{
    if (data == NULL)
        return;

    prc_release_content_wire_edge(ctx, &data->curve);
}

static void
prc_release_topo_edge(prc_context *ctx, prc_topo_edge *data)
{
    if (data == NULL)
        return;

    prc_release_content_wire_edge(ctx, &data->wire_edge);
    prc_release_ptr_topology(ctx, &data->start_vertex);
    prc_release_ptr_topology(ctx, &data->end_vertex);
}

static void
prc_release_topo_coedge(prc_context *ctx, prc_topo_coedge *data)
{
    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    prc_release_ptr_topology(ctx, &data->ptr_topology);
    prc_release_ptr_curve(ctx, &data->ptr_curves);
}

static void
prc_release_topo_loop(prc_context *ctx, prc_topo_loop *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    if (data->coedge != NULL)
    {
        for (k = 0; k < data->number_of_coedges; k++)
        {
            prc_release_ptr_topology(ctx, &data->coedge[k].next_coedge);
        }
        prc_free(ctx, data->coedge);
        data->coedge = NULL;
    }
}

static void
prc_release_topo_face(prc_context *ctx, prc_topo_face *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    prc_release_ptr_surface(ctx, &data->surface_geometry);
    if (data->loops != NULL)
    {
        for (k = 0; k < data->number_of_loops; k++)
        {
            prc_release_ptr_topology(ctx, &data->loops[k]);
        }
        prc_free(ctx, data->loops);
        data->loops = NULL;
    }
}

static void
prc_release_topo_shell(prc_context *ctx, prc_topo_shell *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    if (data->faces != NULL)
    {
        for (k = 0; k < data->number_of_faces; k++)
        {
            prc_release_ptr_topology(ctx, &data->faces[k].face);
        }
        prc_free(ctx, data->faces);
        data->faces = NULL;
    }
}

static void
prc_release_topo_connex(prc_context *ctx, prc_topo_connex *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_base_topology(ctx, &data->base);
    if (data->shells != NULL)
    {
        for (k = 0; k < data->number_of_shells; k++)
        {
            prc_release_ptr_topology(ctx, &data->shells[k]);
        }
        prc_free(ctx, data->shells);
        data->shells = NULL;
    }
}

static void
prc_release_topo_single_wire_body(prc_context *ctx, prc_topo_single_wire_body *data)
{
    if (data == NULL)
        return;

    prc_release_content_body(ctx, &data->base);
    prc_release_ptr_topology(ctx, &data->wire_body);
}

static void
prc_release_topo_brep_data(prc_context *ctx, prc_topo_brep_data *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_content_body(ctx, &data->base);
    if (data->connex != NULL)
    {
        for (k = 0; k < data->number_of_connex; k++)
        {
            prc_release_ptr_topology(ctx, &data->connex[k]);
        }
        prc_free(ctx, data->connex);
        data->connex = NULL;
    }
}

static void
prc_release_topo(prc_context *ctx, prc_topo *body, int depth)
{
    if (body == NULL)
        return;

    if (depth > PRC_RELEASE_TOPO_BODY_RECURSION_MAX)
        return;

    switch (body->tag)
    {
    case PRC_TYPE_TOPO_MultipleVertex:
        if (body->topo_multiple_vertex != NULL)
        {
            prc_release_topo_multiple_vertex(ctx, body->topo_multiple_vertex);
            prc_free(ctx, body->topo_multiple_vertex);
            body->topo_multiple_vertex = NULL;
        }
        break;
    case PRC_TYPE_TOPO_UniqueVertex:
        if (body->topo_unique_vertex != NULL)
        {
            prc_release_topo_unique_vertex(ctx, body->topo_unique_vertex);
            prc_free(ctx, body->topo_unique_vertex);
            body->topo_unique_vertex = NULL;
        }
        break;
    case PRC_TYPE_TOPO_WireEdge:
        if (body->topo_wire_edge != NULL)
        {
            prc_release_topo_wire_edge(ctx, body->topo_wire_edge);
            prc_free(ctx, body->topo_wire_edge);
            body->topo_wire_edge = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Edge:
        if (body->topo_edge != NULL)
        {
            prc_release_topo_edge(ctx, body->topo_edge);
            prc_free(ctx, body->topo_edge);
            body->topo_edge = NULL;
        }
        break;
    case PRC_TYPE_TOPO_CoEdge:
        if (body->topo_coedge != NULL)
        {
            prc_release_topo_coedge(ctx, body->topo_coedge);
            prc_free(ctx, body->topo_coedge);
            body->topo_coedge = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Loop:
        if (body->topo_loop != NULL)
        {
            prc_release_topo_loop(ctx, body->topo_loop);
            prc_free(ctx, body->topo_loop);
            body->topo_loop = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Face:
        if (body->topo_face != NULL)
        {
            prc_release_topo_face(ctx, body->topo_face);
            prc_free(ctx, body->topo_face);
            body->topo_face = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Shell:
        if (body->topo_shell != NULL)
        {
            prc_release_topo_shell(ctx, body->topo_shell);
            prc_free(ctx, body->topo_shell);
            body->topo_shell = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Connex:
        if (body->topo_connex != NULL)
        {
            prc_release_topo_connex(ctx, body->topo_connex);
            prc_free(ctx, body->topo_connex);
            body->topo_connex = NULL;
        }
        break;
    case PRC_TYPE_TOPO_Body:
        if (body->topo_body != NULL)
        {
            prc_release_topo(ctx, body->topo_body, depth + 1);
            prc_free(ctx, body->topo_body);
            body->topo_body = NULL;
        }
        break;
    case PRC_TYPE_TOPO_SingleWireBody:
        if (body->topo_single_wire_body != NULL)
        {
            prc_release_topo_single_wire_body(ctx, body->topo_single_wire_body);
            prc_free(ctx, body->topo_single_wire_body);
            body->topo_single_wire_body = NULL;
        }
        break;
    case PRC_TYPE_TOPO_BrepData:
        if (body->topo_brep_data != NULL)
        {
            prc_release_topo_brep_data(ctx, body->topo_brep_data);
            prc_free(ctx, body->topo_brep_data);
            body->topo_brep_data = NULL;
        }
        break;
    case PRC_TYPE_TOPO_SingleWireBodyCompress:
        if (body->topo_single_wire_compress != NULL)
        {
            if (body->topo_single_wire_compress->connex != NULL)
            {
                prc_free(ctx, body->topo_single_wire_compress->connex);
                body->topo_single_wire_compress->connex = NULL;
            }
            prc_free(ctx, body->topo_single_wire_compress);
            body->topo_single_wire_compress = NULL;
        }
        break;
    case PRC_TYPE_TOPO_BrepDataCompress:
        if (ctx->internal.nano_brep_data != NULL)
        {
            prc_release_nano_brep_data(ctx, ctx->internal.nano_brep_data);
            prc_free(ctx, ctx->internal.nano_brep_data);
            ctx->internal.nano_brep_data = NULL;
        }
        if (body->topo_brep_data_compress != NULL)
        {
            prc_release_brep_data_compress(ctx, body->topo_brep_data_compress);
            prc_free(ctx, body->topo_brep_data_compress);
            body->topo_brep_data_compress = NULL;
        }
        break;
    default:
        break;
    }
}

static void
prc_release_topo_contexts(prc_context *ctx, prc_topo_context *topo_context)
{
    size_t k;

    if (topo_context == NULL)
        return;

    if (topo_context->bodies != NULL)
    {
        for (k = 0; k < topo_context->number_of_bodies; k++)
        {
            prc_release_topo(ctx, &topo_context->bodies[k], 0);
        }
        prc_free(ctx, topo_context->bodies);
        topo_context->bodies = NULL;
    }
}

static void
prc_release_exact_geometry(prc_context *ctx, prc_file_structure_exact_geometry *exact_geometry)
{
    size_t k;

    if (exact_geometry == NULL)
        return;

    if (exact_geometry->topo_contexts != NULL)
    {
        for (k = 0; k < exact_geometry->topo_context_count; k++)
        {
            prc_release_topo_contexts(ctx, &exact_geometry->topo_contexts[k]);
        }
        prc_free(ctx, exact_geometry->topo_contexts);
        exact_geometry->topo_contexts = NULL;
    }
}

static void
prc_release_file_struct_geometry(prc_context *ctx, prc_asm_file_structure_geometry *geometry)
{
    if (geometry == NULL)
        return;

    if (geometry->user_data.user_data != NULL)
    {
        prc_free(ctx, geometry->user_data.user_data);
        geometry->user_data.user_data = NULL;
    }

    prc_release_exact_geometry(ctx, &geometry->exact_geometry);
    prc_free(ctx, geometry);
}

static void
prc_release_extra_geometry(prc_context *ctx, prc_extra_geometry *extra_geometry)
{
    if (extra_geometry == NULL)
        return;

    if (extra_geometry->summary.bodies != NULL)
    {
        prc_free(ctx, extra_geometry->summary.bodies);
        extra_geometry->summary.bodies = NULL;
    }

    prc_release_context_graphics(ctx, &extra_geometry->context_graphics);
}

static void
prc_release_file_struct_extra_geometry(prc_context *ctx, prc_asm_file_structure_extra_geometry *extra_geometry)
{
    size_t k;

    if (extra_geometry == NULL)
        return;

    if (extra_geometry->user_data.user_data != NULL)
    {
        prc_free(ctx, extra_geometry->user_data.user_data);
        extra_geometry->user_data.user_data = NULL;
    }

    for (k = 0; k < extra_geometry->extra_geom_count; k++)
    {
        prc_release_extra_geometry(ctx, &extra_geometry->extra_geom[k]);
    }
    prc_free(ctx, extra_geometry->extra_geom);
    prc_free(ctx, extra_geometry);
}

static void
prc_release_file_struct_schema(prc_context *ctx)
{
    size_t k;
    prc_schema *schema = ctx->internal.schema;

    if (schema == NULL)
        return;

    for (k = 0; k < schema->schema_count; k++)
    {
        prc_release_entity_schema(ctx, &schema->entity_schema[k]);
    }
    if (schema->entity_schema != NULL)
    {
        prc_free(ctx, schema->entity_schema);
        schema->entity_schema = NULL;
    }
    prc_free(ctx, schema);
}

static void
prc_release_file_struct(prc_context *ctx, prc_filestructure *file_struct)
{
    if (file_struct == NULL)
        return;

    if (file_struct->schema_globals_unzipped != NULL)
    {
        prc_free(ctx, file_struct->schema_globals_unzipped);
        file_struct->schema_globals_unzipped = NULL;
    }
    if (file_struct->tree_unzipped != NULL)
    {
        prc_free(ctx, file_struct->tree_unzipped);
        file_struct->tree_unzipped = NULL;
    }
    if (file_struct->tessellation_unzipped != NULL)
    {
        prc_free(ctx, file_struct->tessellation_unzipped);
        file_struct->tessellation_unzipped = NULL;
    }
    if (file_struct->geometry_unzipped != NULL)
    {
        prc_free(ctx, file_struct->geometry_unzipped);
        file_struct->geometry_unzipped = NULL;
    }
    if (file_struct->extra_geometry_unzipped != NULL)
    {
        prc_free(ctx, file_struct->extra_geometry_unzipped);
        file_struct->extra_geometry_unzipped = NULL;
    }
    if (file_struct->model_unzipped != NULL)
    {
        prc_free(ctx, file_struct->model_unzipped);
        file_struct->model_unzipped = NULL;
    }
    if (file_struct->header != NULL)
    {
        prc_release_file_struct_header(ctx, file_struct->header);
        prc_free(ctx, file_struct->header);
        file_struct->header = NULL;
    }

    prc_release_file_struct_model(ctx, file_struct->model);
    file_struct->model = NULL;

    prc_release_file_struct_globals(ctx, file_struct->globals);
    file_struct->globals = NULL;

    prc_release_file_struct_tree(ctx, file_struct->tree);
    file_struct->tree = NULL;

    prc_release_file_struct_tessallation(ctx, file_struct->tessellation);
    file_struct->tessellation = NULL;

    prc_release_file_struct_geometry(ctx, file_struct->geometry);
    file_struct->geometry = NULL;

    prc_release_file_struct_extra_geometry(ctx, file_struct->extra_geometry);
    file_struct->extra_geometry = NULL;
}

static void
prc_release_exact_geometry_tess(prc_context *ctx, prc_exact_geom_tess *tess)
{
    if (tess->shells != NULL)
    {
        for (size_t k = 0; k < tess->number_of_shells; k++)
        {
            if (tess->shells[k].faces != NULL)
            {
                for (size_t j = 0; j < tess->shells[k].number_of_faces; j++)
                {
                    if (tess->shells[k].faces[j].tess_data != NULL)
                    {
                        for (size_t i = 0; i < tess->shells[k].faces[j].tess_data->number_of_triangles; i++)
                        {
                            if (tess->shells[k].faces[j].tess_data->triangles != NULL)
                            {
                                prc_free(ctx, tess->shells[k].faces[j].tess_data->triangles);
                                tess->shells[k].faces[j].tess_data->triangles = NULL;
                            }
                            if (tess->shells[k].faces[j].tess_data->vertices != NULL)
                            {
                                prc_free(ctx, tess->shells[k].faces[j].tess_data->vertices);
                                tess->shells[k].faces[j].tess_data->vertices = NULL;
                            }
                        }
                        prc_free(ctx, tess->shells[k].faces[j].tess_data);
                        tess->shells[k].faces[j].tess_data = NULL;
                    }
                    if (tess->shells[k].faces[j].wire_data != NULL)
                    {
                        for (size_t i = 0; i < tess->shells[k].faces[j].wire_data->number_of_points; i++)
                        {
                            if (tess->shells[k].faces[j].wire_data->points != NULL)
                            {
                                prc_free(ctx, tess->shells[k].faces[j].wire_data->points);
                                tess->shells[k].faces[j].wire_data->points = NULL;
                            }
                        }
                        prc_free(ctx, tess->shells[k].faces[j].wire_data);
                        tess->shells[k].faces[j].wire_data = NULL;
                    }
                }
                prc_free(ctx, tess->shells[k].faces);
            }
        }
        prc_free(ctx, tess->shells);
        tess->shells = NULL;
    }
}

/* The root release method */
void
prc_release_data(prc_context *ctx, prc_data *data)
{
    size_t k;

    if (data == NULL)
        return;

    prc_release_header(ctx, data->header);

    for (k = 0; k < data->file_structure_count; k++)
    {
        prc_release_file_struct(ctx, &data->file_struct[k]);
    }

    if (data->views != NULL)
    {
        for (k = 0; k < data->view_count; k++)
        {
            if (data->views[k].external_name != NULL)
            {
                prc_free(ctx, data->views[k].external_name);
            }
            if (data->views[k].internal_name != NULL)
            {
                prc_free(ctx, data->views[k].internal_name);
            }
        }
        prc_free(ctx, data->views);
    }

    if (data->exact_geom_capacity > 0 && data->exact_geom_tess != NULL)
    {
        for (k = 0; k < data->exact_geom_tess_count; k++)
        {
            prc_release_exact_geometry_tess(ctx, &data->exact_geom_tess[k]);
        }
        prc_free(ctx, data->exact_geom_tess);
    }

    prc_release_file_struct_schema(ctx);
    prc_free(ctx, data->file_struct);
    prc_free(ctx, data);
}
