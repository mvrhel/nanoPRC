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
#include <stdio.h>
#include "prc_parse_common.h"
#include "prc_schema.h"

/* Table 61 */
void
prc_parse_file_indentifier(prc_context *ctx, prc_bit_state *bit_state, prc_file_identifier *data)
{
    data->flag = prc_bitread_bit(ctx, bit_state);
    if (data->flag == 0)
    {
        data->unique_id = prc_get_compressed_unique_id(ctx, bit_state);
    }
}

/* 7.3.3.2 */
prc_unique_id
prc_get_compressed_unique_id(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_unique_id val;

    val.unique_id0 = prc_bitread_uint32(ctx, bit_state);
    val.unique_id1 = prc_bitread_uint32(ctx, bit_state);
    val.unique_id2 = prc_bitread_uint32(ctx, bit_state);
    val.unique_id3 = prc_bitread_uint32(ctx, bit_state);

    return val;
}

/* Table 34 GraphicsContent */
static int
prc_parse_graphics_content(prc_context *ctx, prc_bit_state *bit_state, prc_graphics_content *data)
{
    data->biased_layer_index = prc_bitread_uint32(ctx, bit_state);
    data->biased_index_of_line_style = prc_bitread_uint32(ctx, bit_state);
    data->behavior_bit_field1 = prc_bitread_uint8(ctx, bit_state);
    data->behavior_bit_field2 = prc_bitread_uint8(ctx, bit_state);
    data->has_entity_ref = 0;
    return 0;
}

/* Table 32 PRC_TYPE_ROOT_PRCBaseWithGraphics */
int
prc_parse_base_with_graphics(prc_context *ctx, prc_bit_state *bit_state,
    uint8_t parent_is_RI, void *RIparent, prc_base_with_graphics *data)
{
    int code;

    code = prc_parse_content_prc_ref_base(ctx, bit_state, parent_is_RI, RIparent, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_content_prc_ref_base\n");
        return code;
    }

    data->same_graphics = prc_bitread_bit(ctx, bit_state);

    if (data->same_graphics)
    {
        data->graphics_content = bit_state->graphics_content;
       // data->graphics_content.behavior_bit_field1 = ctx->graphics_content.behavior_bit_field1;
       // data->graphics_content.behavior_bit_field2 = ctx->graphics_content.behavior_bit_field2;
       // data->graphics_content.biased_index_of_line_style = ctx->graphics_content.biased_index_of_line_style;
       // data->graphics_content.biased_layer_index = ctx->graphics_content.biased_layer_index;
    }
    else
    {
        code = prc_parse_graphics_content(ctx, bit_state, &data->graphics_content);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_parse_graphics_content\n");
            return code;
        }

        /* If the data read is zero don't reset */
       // if (data->graphics_content.biased_index_of_line_style != 0 ||
       //     data->graphics_content.biased_layer_index != 0)
       // {
            bit_state->graphics_content = data->graphics_content;
          //  ctx->graphics_content.behavior_bit_field1 = data->graphics_content.behavior_bit_field1;
          //  ctx->graphics_content.behavior_bit_field2 = data->graphics_content.behavior_bit_field2;
          //  ctx->graphics_content.biased_index_of_line_style = data->graphics_content.biased_index_of_line_style;
          //  ctx->graphics_content.biased_layer_index = data->graphics_content.biased_layer_index;
       // }
    }

    /* Need to check if PRC_TYPE_ROOT_PRCBaseWithGraphics is in the schema */
    code = prc_check_for_schema(ctx, PRC_TYPE_ROOTBaseWithGraphics);
    if (code > 0)
    {
        int recursion = 0;
        code = prc_execute_schema(ctx, bit_state, prc_get_schema(ctx, code - 1), &recursion);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_execute_schema\n");
            return code;
        }
    }
    return 0;
}

/* PRC_TYPE_MISC_CartesianTransformation
   PRC_TYPE_MISC_GeneralTransformation Miscellaneous data entity types */
int
prc_parse_misc_transform(prc_context *ctx, prc_bit_state *bit_state, prc_misc_transformation *data)
{

    data->transformation_type = prc_bitread_uint32(ctx, bit_state);

    switch (data->transformation_type)
    {
        case PRC_TYPE_MISC_CartesianTransformation:
            prc_parse_3d_transform(ctx, bit_state, &data->cartesian);
            break;
        case PRC_TYPE_MISC_GeneralTransformation:
            prc_parse_general_transform(ctx, bit_state, &data->general_transformation);
            break;
        default:
            prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_misc_transform\n");
            return PRC_ERROR_PARSE;
    }
    return 0;
}

/* Table 83 */
void
prc_parse_general_transform(prc_context *ctx, prc_bit_state *bit_state,  prc_misc_general_trans *data)
{
    /* As of now, this type is part of an abstract type and the tag has already been read */
    for (int k = 0; k < 16; k++)
    {
        data->general_transform[k] = prc_bitread_double(ctx, bit_state);
    }
}

/* Table 87 */
void
prc_parse_3d_transform(prc_context *ctx, prc_bit_state *bit_state, prc_trans_3d *data)
{
    data->behavior = prc_bitread_uint8(ctx, bit_state);

    if (data->behavior & PRC_TRANSFORMATION_Translate)
    {
        data->translation = prc_parse_3d_vector(ctx, bit_state);
    }

    if (data->behavior & PRC_TRANSFORMATION_NonOrtho)
    {
        data->non_ortho_matrix[0] = prc_parse_3d_vector(ctx, bit_state);
        data->non_ortho_matrix[1] = prc_parse_3d_vector(ctx, bit_state);
        data->non_ortho_matrix[2] = prc_parse_3d_vector(ctx, bit_state);
    }

    if (data->behavior & PRC_TRANSFORMATION_Rotate)
    {
        data->rotation[0] = prc_parse_3d_vector(ctx, bit_state);
        data->rotation[1] = prc_parse_3d_vector(ctx, bit_state);
    }

    if (data->behavior & PRC_TRANSFORMATION_NonUniformScale)
    {
        data->non_uniform_scale = prc_parse_3d_vector(ctx, bit_state);
    }

    if (data->behavior & PRC_TRANSFORMATION_Scale)
    {
        data->scale = prc_bitread_double(ctx, bit_state);
    }

    if (data->behavior & PRC_TRANSFORMATION_Homogeneous)
    {
        /* This is an odd one */
        for (int k = 0; k < 4; k++)
        {
            data->homogeneous[k] = prc_bitread_double(ctx, bit_state);
        }
    }
}

/* Table 76 */
int
prc_parse_cart_trans(prc_context *ctx, prc_bit_state *bit_state, prc_cart_transformation *data)
{
    /* Is this really an unsigned int or a unsigned char? */
    data->name = prc_bitread_uint32(ctx, bit_state);

    /* Is there another byte read?  This may be the behavior bits */
    //unsigned char x = prc_bitread_uint8(ctx, bit_state);

    switch (data->name)
    {
    case PRC_TRANSFORMATION_Identity:
        /* No data */
        break;

    case PRC_TRANSFORMATION_Translate:
        data->transform.translation = prc_parse_3d_vector(ctx, bit_state);
        break;

    case PRC_TRANSFORMATION_Rotate:
        data->transform.rotation[0] = prc_parse_3d_vector(ctx, bit_state);
        data->transform.rotation[1] = prc_parse_3d_vector(ctx, bit_state);
        break;

    case PRC_TRANSFORMATION_Mirror:
        /* Check this */
        data->transform.rotation[0] = prc_parse_3d_vector(ctx, bit_state);
        data->transform.rotation[1] = prc_parse_3d_vector(ctx, bit_state);
        break;

    case PRC_TRANSFORMATION_Scale:
        data->transform.scale = prc_bitread_double(ctx, bit_state);
        break;

    case PRC_TRANSFORMATION_NonUniformScale:
        data->transform.non_uniform_scale = prc_parse_3d_vector(ctx, bit_state);
        break;

    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_cart_trans\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

int
prc_parse_user_data(prc_context *ctx, prc_bit_state *bit_state, prc_user_data *data)
{
    uint32_t k;

    data->stream_size = prc_bitread_uint32(ctx, bit_state);
    data->user_data = NULL;

    for (k = 0; k < data->stream_size; k++)
    {
        prc_bitread_bit(ctx, bit_state);
    }

    return 0;
}

/* Table 137 ContentBaseTessData */
int
prc_parse_content_base_tess_data(prc_context *ctx, prc_bit_state *bit_state, prc_content_base_tess_data *data)
{
    uint32_t k;

    data->is_calculated = prc_bitread_bit(ctx, bit_state);
    data->number_of_coordinates = prc_bitread_uint32(ctx, bit_state);

    if (data->number_of_coordinates > 0)
    {
        data->coordinates = (double *)prc_calloc(ctx, data->number_of_coordinates, sizeof(double));
        if (data->coordinates == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_base_tess_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->number_of_coordinates; k++)
        {
            data->coordinates[k] = prc_bitread_double(ctx, bit_state);
        }
    }
    return 0;
}

/* Table 22 */
prc_domain
prc_parse_domain(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_domain data;

    data.min_uv = prc_parse_2d_vector(ctx, bit_state);
    data.max_uv = prc_parse_2d_vector(ctx, bit_state);

    return data;
}

int
prc_parse_bound_box(prc_context *ctx, prc_bit_state *bit_state, prc_bounding_box *data)
{
    data->minimum_corner = prc_parse_3d_vector(ctx, bit_state);
    data->maximum_corner = prc_parse_3d_vector(ctx, bit_state);

    return 0;
}

/* Table 21 */
prc_parameterization
prc_parse_parameterization(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_parameterization data;

    data.interval = prc_parse_interval(ctx, bit_state);
    data.coeff_a = prc_bitread_double(ctx, bit_state);
    data.coeff_b = prc_bitread_double(ctx, bit_state);

    return data;
}

/* Table 20 */
prc_interval
prc_parse_interval(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_interval data;

    data.min_value = prc_bitread_double(ctx, bit_state);
    data.max_value = prc_bitread_double(ctx, bit_state);

    return data;
}

/* Table 25 � Vector2d */
prc_vec2
prc_parse_2d_vector(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_vec2 data;

    data.x = prc_bitread_double(ctx, bit_state);
    data.y = prc_bitread_double(ctx, bit_state);

    return data;
}

/* Table 25 */
prc_vec3
prc_parse_3d_vector(prc_context *ctx, prc_bit_state *bit_state)
{
    prc_vec3 data;

    data.x = prc_bitread_double(ctx, bit_state);
    data.y = prc_bitread_double(ctx, bit_state);
    data.z = prc_bitread_double(ctx, bit_state);

    return data;
}

/* 8.2.3.3 ContentPRCRefBase Table 29 */
int
prc_parse_content_prc_ref_base(prc_context *ctx, prc_bit_state *bit_state,
    uint8_t parent_is_RI, void *parent, prc_content_prc_ref_base *data)
{
    int code = 0;

    code = prc_parse_attribute_data(ctx, bit_state, &data->attribute_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_attribute_data\n");
        return code;
    }

    code = prc_parse_name(ctx, bit_state, &data->name);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_name\n");
        return code;
    }

    data->nonpersistent_id_cad = prc_bitread_uint32(ctx, bit_state); /* Missing in spec */
    data->unique_id_cad = prc_bitread_uint32(ctx, bit_state);
    data->unique_id = prc_bitread_uint32(ctx, bit_state);
    data->file_index = ctx->current_file_index;
    data->parent_is_RI = parent_is_RI;
    data->parent = parent;

    if (ctx->graphics_content.ref_base_count == ctx->graphics_content.ref_base_ptr_capacity - 1)
    {
        size_t new_capacity = ctx->graphics_content.ref_base_ptr_capacity * 2;
        void **new_ptr = (void **)prc_realloc(ctx, ctx->graphics_content.ref_base_ptr, sizeof(void *) * new_capacity);

        if (new_ptr == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_content_prc_ref_base\n");
            return PRC_ERROR_MEMORY;
        }
        ctx->graphics_content.ref_base_ptr = new_ptr;
        ctx->graphics_content.ref_base_ptr_capacity = new_capacity;
    }
    ctx->graphics_content.ref_base_ptr[ctx->graphics_content.ref_base_count] = (void *)data;
    ctx->graphics_content.ref_base_count++;

    return code;
}

/* Table 46 RGB color.  Spec also talks of color with alpha  */
prc_rgb_color
prc_parse_rgb(prc_context *ctx, prc_bit_state *bit_state, uint8_t has_alpha)
{
    prc_rgb_color data;

    data.red = prc_bitread_double(ctx, bit_state);
    data.green = prc_bitread_double(ctx, bit_state);
    data.blue = prc_bitread_double(ctx, bit_state);

    if (has_alpha)
        data.alpha = prc_bitread_double(ctx, bit_state);
    else
        data.alpha = 1.0;

    return data;
}

/* 8 bit version */
/* Table 46 RGB color.  Spec also talks of color with alpha  */
prc_rgb_color
prc_parse_rgb8(prc_context *ctx, prc_bit_state *bit_state, uint8_t has_alpha)
{
    prc_rgb_color data;

    data.red = prc_bitread_uint8(ctx, bit_state);
    data.green = prc_bitread_uint8(ctx, bit_state);
    data.blue = prc_bitread_uint8(ctx, bit_state);

    if (has_alpha)
        data.alpha = prc_bitread_uint8(ctx, bit_state);
    else
        data.alpha = 255.0;

    return data;
}

void
debug_prc_unsignedint_bitstream(prc_context *ctx, prc_bit_state *bit_state, uint32_t find, size_t byte_length)
{
    prc_bit_state temp;
    prc_bit_state restore;
    uint32_t val_read;
    size_t start_pos = (size_t)bit_state->ptr;
    size_t size = 0;
    size_t bits_read = 0;
    uint8_t bit;

    temp.ptr = bit_state->ptr;
    temp.bitmask = bit_state->bitmask;
    restore = temp;

    while (size < byte_length)
    {
        val_read = prc_bitread_uint32(ctx, bit_state);
        size = (size_t)bit_state->ptr - start_pos;
        if (val_read == find)
        {
            printf("Found key\n");
            printf("bytes read = %zd\n", size);
            printf("bits read = %zd\n", bits_read);
            break;
        }

        bit = prc_bitread_bit(ctx, &temp);
        bit_state->bitmask = temp.bitmask;
        bit_state->ptr = temp.ptr;
        bits_read++;
    }

    // bit_state->bitmask = restore.bitmask;
    // bit_state->ptr = restore.ptr;
}

int
prc_get_attribute_title(prc_context *ctx, prc_bit_state *bit_state, prc_attribute_entry *title)
{
    int code;

    title->flag = prc_bitread_bit(ctx, bit_state);
    if (title->flag)
    {
        /* Title is an enumerated type */
        title->integer_title = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        /* Title is a string */
        code = prc_bitread_string(ctx, bit_state, &title->string_title);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_bitread_string\n");
            return code;
        }
    }
    return 0;
}

static int
prc_parse_attribute_key_values(prc_context *ctx, prc_bit_state *bit_state, prc_attribute_key_value *data)
{
    int code;

    code = prc_get_attribute_title(ctx, bit_state, &data->title);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_get_attribute_title\n");
        return code;
    }
    data->type = prc_bitread_uint32(ctx, bit_state);

    switch (data->type)
    {
    case PRC_ATTRIBUTE_TYPE_INT:
        data->value_integer = prc_bitread_uint32(ctx, bit_state);
        break;
    case PRC_ATTRIBUTE_TYPE_DOUBLE:
        data->value_double = prc_bitread_double(ctx, bit_state);
        break;
    case PRC_ATTRIBUTE_TYPE_TIME32:
        data->value_secs_integer = prc_bitread_uint32(ctx, bit_state);
        break;
    case PRC_ATTRIBUTE_TYPE_CHAR_UTF8:
        code = prc_bitread_string(ctx, bit_state, &data->val_string);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_bitread_string\n");
            return code;
        }
        break;
    case PRC_ATTRIBUTE_TYPE_TIME64:
        data->value_time_msp = prc_bitread_uint32(ctx, bit_state);
        data->value_time_lsp = prc_bitread_uint32(ctx, bit_state);
        break;
    default:
        prc_error(ctx, PRC_ERROR_KEY_NOT_FOUND, "Error parsing prc_parse_attribute_key_values\n");
        return PRC_ERROR_KEY_NOT_FOUND;
    }
    return 0;
}

int
prc_parse_attribute_data(prc_context *ctx, prc_bit_state *bit_state, prc_attribute_data *data)
{
    size_t k, j;
    int code = 0;

    data->attribute_count = prc_bitread_uint32(ctx, bit_state);

    if (data->attribute_count > 0)
    {
        data->attributes = (prc_misc_attribute *)prc_calloc(ctx, data->attribute_count, sizeof(prc_misc_attribute));
        if (data->attributes == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_attribute_data\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < data->attribute_count; k++)
        {
            /* prc_misc_attribute */
            data->attributes[k].tag = prc_bitread_uint32(ctx, bit_state);
            if (data->attributes[k].tag != PRC_TYPE_MISC_Attribute)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_attribute_data\n");
                return PRC_ERROR_PARSE;
            }

            code = prc_get_attribute_title(ctx, bit_state, &data->attributes[k].attribute_title);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_get_attribute_title\n");
                return code;
            }

            data->attributes[k].number_attributes = prc_bitread_uint32(ctx, bit_state);
            if (data->attributes[k].number_attributes > 0)
            {
                data->attributes[k].attributes =
                    (prc_attribute_key_value *)prc_calloc(ctx, data->attributes[k].number_attributes,
                        sizeof(prc_attribute_key_value));
                if (data->attributes == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_attribute_data\n");
                    return PRC_ERROR_MEMORY;
                }

                for (j = 0; j < data->attributes[k].number_attributes; j++)
                {
                    code = prc_parse_attribute_key_values(ctx, bit_state, &data->attributes[k].attributes[j]);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed in prc_parse_attribute_key_values\n");
                        return code;
                    }
                }
            }
        }
    }
    return code;
}

/* 8.2.3.3.2 Name */
int
prc_parse_name(prc_context *ctx, prc_bit_state *bit_state, prc_name *val)
{
    int code = 0;
    uint8_t same;

    same = prc_bitread_bit(ctx, bit_state);

    if (!same)
    {
        code = prc_bitread_string(ctx, bit_state, &val->name);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_bitread_string\n");
            return code;
        }
    }

    val->same = same;

    return code;
}

/* 8.2.3.2 ContentPRCBase */
int
prc_parse_content_prc_base(prc_context *ctx, prc_bit_state *bit_state, prc_content_prc_base *base)
{
    int code;

    code = prc_parse_attribute_data(ctx, bit_state, &base->attribute_data);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_attribute_data\n");
        return code;
    }

    code = prc_parse_name(ctx, bit_state, &base->name);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_name\n");
    }

    return code;
}

/* Table 116 RepresentationItemContent */
int
prc_parse_representation_item_content(prc_context *ctx, prc_bit_state *bit_state, prc_representation_item_content *data)
{
    int code;

    code = prc_parse_base_with_graphics(ctx, bit_state, true, data, &data->base);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_parse_representation_item_content\n");
        return PRC_ERROR_PARSE;
    }

    /* Biased indices. The table says unsigned integer but the value can be 0 which
      is negative 1 meaning  that the value is not an index. */
    data->biased_index_local_coordinate_system = prc_bitread_uint32(ctx, bit_state);
    data->biased_index_tessellation = prc_bitread_uint32(ctx, bit_state);

    data->style_inheritance.has_any_inheritance = 0;

    return 0;
}

/* Returns < 0 if a mismatch occurs. Returns 0 if tag is OK and no schema present, Returns > 0 if schema present and
   returns the schema index + 1 (biased value). */
int
prc_read_check_tag(prc_context *ctx, prc_bit_state *bit_state, prc_unsigned_int expected_tag, prc_unsigned_int *read_tag)
{
    prc_schema *schema = ctx->internal.schema;

    *read_tag = prc_bitread_uint32(ctx, bit_state);
    if (*read_tag != expected_tag)
    {
        prc_error(ctx, PRC_TAG_ERROR, "Tag error in prc_read_check_tag\n");
        return PRC_TAG_ERROR;
    }

    if (schema != NULL)
    {
        uint32_t k;
        uint32_t schema_count = schema->schema_count;

        /* Check if schema is present for this type  */
        for (k = 0; k < schema_count; k++)
        {
            if (expected_tag == schema->entity_schema[k].entity_type)
            {
                return k + 1;
            }
        }
    }

    return 0;
}

/* Consume the Section-8 extension data for an entity whose type declares a
   schema. schema_code is what prc_read_check_tag returned for this entity: 0
   when the type declares no extension, or the 1-based index of its entity
   schema.

   Call this at the END of the entity's own fields, and before any user data.
   The extension is stored there rather than after the tag -- the one site that
   already did this by hand, prc_parse_file_globals, executes it after the
   global data and says so, and prc_check_for_schema's own comment says the
   same.

   Returns 0 when there is nothing to do, so a caller can invoke it
   unconditionally.

   UNVERIFIED -- the placement of this call within an entity is not confirmed by
   any file available to us, and two things in this codebase disagree about it.
   The rule followed here is the one prc_check_for_schema documents, extension
   before user data; prc_parse_mkp_view executes after its user data instead and
   has been left alone rather than changed on a guess. Measured across 694 files:
   every extension program that executes for these types consumes ZERO bits, so
   no corpus file can distinguish a correct placement from a wrong one. The
   control that makes that a real measurement rather than a dead instrument is
   prc_parse_file_globals, whose extension does consume data -- 1, 80 and 186
   bits on three different files -- so the evaluator itself works.

   The consequence: this is right by the clause and inert in practice. The first
   file that carries a non-empty extension for one of these types is the test,
   and if it desyncs, placement relative to user data is the first thing to
   suspect. */
int
prc_consume_schema_extension(prc_context *ctx, prc_bit_state *bit_state,
    int schema_code)
{
    int recursion = 0;
    int code;

    if (schema_code <= 0)
        return 0;

    code = prc_execute_schema(ctx, bit_state,
        prc_get_schema(ctx, (uint32_t)(schema_code - 1)), &recursion);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_execute_schema\n");
        return code;
    }
    return 0;
}

/* Some types dont have tags but can have schemas. The data for these types is stored at the end and must be read (prior to the user data) */
int
prc_check_for_schema(prc_context *ctx, prc_unsigned_int expected_tag)
{
    prc_schema *schema = ctx->internal.schema;

    if (schema != NULL)
    {
        uint32_t k;
        uint32_t schema_count = schema->schema_count;

        /* Check if schema is present for this type  */
        for (k = 0; k < schema_count; k++)
        {
            if (expected_tag == schema->entity_schema[k].entity_type)
            {
                return k + 1;
            }
        }
    }
    return 0;
}

/* Table 322 � PRC_TYPE_MATH_FCT_1D_Polynom */
static int
prc_parse_math_fct_1d_polynom(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_1d_polynom *data, uint8_t read_tag)
{
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_1D_Polynom;
    }

    data->number_of_coefficients = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_coefficients > 0)
    {
        data->coefficients = (double *)prc_calloc(ctx, data->number_of_coefficients, sizeof(double));
        if (data->coefficients == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d_polynom\n");
            return PRC_ERROR_MEMORY;
        }
        for (uint32_t k = 0; k < data->number_of_coefficients; k++)
        {
            data->coefficients[k] = prc_bitread_double(ctx, bit_state);
        }
    }

    return 0;
}

/* Table 323 � PRC_TYPE_MATH_FCT_1D_Trigonometric */
static int
prc_parse_math_fct_1d_trigonometric(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_1d_trigonometric *data, uint8_t read_tag)
{
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_1D_Trigonometric;
    }

    data->amplitude = prc_bitread_double(ctx, bit_state);
    data->phase = prc_bitread_double(ctx, bit_state);
    data->freq = prc_bitread_double(ctx, bit_state);
    data->dc_offset = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table 324 � PRC_TYPE_MATH_FCT_1D_Fraction */
static int
prc_parse_math_fct_1d_fraction(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_1d_fraction *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_1D_Fraction;
    }

    code = prc_parse_math_fct_1d(ctx, bit_state, &data->numerator);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_math_fct_1d for numerator\n");
        return code;
    }

    code = prc_parse_math_fct_1d(ctx, bit_state, &data->denominator);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_math_fct_1d for denominator\n");
        return code;
    }

    return 0;
}

/* Table 325 � PRC_TYPE_MATH_FCT_1D_ArctanCos */
static int
prc_parse_math_fct_1d_arctancos(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_1d_arctancos *data, uint8_t read_tag)
{
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_1D_ArctanCos;
    }

    data->a = prc_bitread_double(ctx, bit_state);
    data->amplitude = prc_bitread_double(ctx, bit_state);
    data->frequency = prc_bitread_double(ctx, bit_state);
    data->phase = prc_bitread_double(ctx, bit_state);
    data->e = prc_bitread_double(ctx, bit_state);

    return 0;
}

/* Table  327 � CombinationFunctions */
static int
prc_parse_combination_functions(prc_context *ctx, prc_bit_state *bit_state,
    prc_combination_functions *data)
{
    int code;

    data->coefficient = prc_bitread_double(ctx, bit_state);
    code = prc_parse_math_fct_1d(ctx, bit_state, &data->function);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_math_fct_1d for combination function\n");
        return code;
    }

    return 0;
}

/* Table 326 � PRC_TYPE_MATH_FCT_1D_Combination */
static int
prc_parse_math_fct_1d_combination(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_1d_combination *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_1D_Combination;
    }
    data->number_of_coefficients = prc_bitread_uint32(ctx, bit_state);
    if (data->number_of_coefficients > 0)
    {
        data->coefficients = (prc_combination_functions *)prc_calloc(ctx, data->number_of_coefficients, sizeof(prc_combination_functions));
        if (data->coefficients == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d_combination\n");
            return PRC_ERROR_MEMORY;
        }
        for (uint32_t k = 0; k < data->number_of_coefficients; k++)
        {
            code = prc_parse_combination_functions(ctx, bit_state, &data->coefficients[k]);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_parse_math_fct_1d for combination function\n");
                return code;
            }
        }
    }
    return 0;
}

/* Math methods */
int
prc_parse_math_fct_1d(prc_context *ctx, prc_bit_state *bit_state, prc_math_fct_1d *data)
{
    int code = 0;

    data->tag = prc_bitread_uint32(ctx, bit_state);

    switch (data->tag)
    {
    case PRC_TYPE_MATH_FCT_1D_Polynom:
        data->polynom = (prc_math_fct_1d_polynom *)prc_calloc(ctx, 1, sizeof(prc_math_fct_1d_polynom));
        if (data->polynom == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_1d_polynom(ctx, bit_state, data->polynom, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_1d_polynom\n");
            return code;
        }
        break;

    case PRC_TYPE_MATH_FCT_1D_Trigonometric:
        data->trigonometric = (prc_math_fct_1d_trigonometric *)prc_calloc(ctx, 1, sizeof(prc_math_fct_1d_trigonometric));
        if (data->trigonometric == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_1d_trigonometric(ctx, bit_state, data->trigonometric, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_1d_trigonometric\n");
            return code;
        }
        break;

    case PRC_TYPE_MATH_FCT_1D_Fraction:
        data->fraction = (prc_math_fct_1d_fraction *)prc_calloc(ctx, 1, sizeof(prc_math_fct_1d_fraction));
        if (data->fraction == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_1d_fraction(ctx, bit_state, data->fraction, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_1d_fraction\n");
            return code;
        }
        break;

    case PRC_TYPE_MATH_FCT_1D_ArctanCos:
        data->arctancos = (prc_math_fct_1d_arctancos *)prc_calloc(ctx, 1, sizeof(prc_math_fct_1d_arctancos));
        if (data->arctancos == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_1d_arctancos(ctx, bit_state, data->arctancos, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_1d_arctancos\n");
            return code;
        }
        break;

    case PRC_TYPE_MATH_FCT_1D_Combination:
        data->combination = (prc_math_fct_1d_combination *)prc_calloc(ctx, 1, sizeof(prc_math_fct_1d_combination));
        if (data->combination == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_1d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_1d_combination(ctx, bit_state, data->combination, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_1d_combination\n");
            return code;
        }
        break;
    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_math_fct_1d\n");
        return PRC_ERROR_PARSE;
    };

    return 0;
}

/* Table 328 � PRC_TYPE_MATH_FCT_3D_Linear */
static int
prc_parse_math_fct_3d_linear(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_3d_linear *data, uint8_t read_tag)
{
    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_3D_Linear;
    }

    for (uint32_t k = 0; k < 9; k++)
    {
        data->mat[k] = prc_bitread_double(ctx, bit_state);
    }

    for (uint32_t k = 0; k < 3; k++)
    {
        data->vect[k] = prc_bitread_double(ctx, bit_state);
    }

    return 0;
}

/* Table 329 � PRC_TYPE_MATH_FCT_3D_nonLinear */
static int
prc_parse_math_fct_3d_nonlinear(prc_context *ctx, prc_bit_state *bit_state,
    prc_math_fct_3d_nonlinear *data, uint8_t read_tag)
{
    int code;

    if (read_tag)
    {
        data->tag = prc_bitread_uint32(ctx, bit_state);
    }
    else
    {
        data->tag = PRC_TYPE_MATH_FCT_3D_nonLinear;
    }

    code = prc_parse_math_fct_3d(ctx, bit_state, &data->left_transformation);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_math_fct_3d for left_transformation\n");
        return code;
    }
    code = prc_parse_math_fct_3d(ctx, bit_state, &data->right_transformation);
    if (code < 0)
    {
        prc_error(ctx, code, "Error in prc_parse_math_fct_3d for right_transformation\n");
        return code;
    }

    data->d2 = prc_bitread_double(ctx, bit_state);
    data->reserved_double = prc_bitread_double(ctx, bit_state);
    data->reserved_int1 = prc_bitread_int32(ctx, bit_state);
    data->reserved_int2 = prc_bitread_int32(ctx, bit_state);
    data->reserved_int3 = prc_bitread_int32(ctx, bit_state);

    return 0;
}

int
prc_parse_math_fct_3d(prc_context *ctx, prc_bit_state *bit_state, prc_math_fct_3d *data)
{
    int code = 0;

    data->tag = prc_bitread_uint32(ctx, bit_state);

    switch (data->tag)
    {
    case PRC_TYPE_MATH_FCT_3D_Linear:
        data->linear = (prc_math_fct_3d_linear *)prc_calloc(ctx, 1, sizeof(prc_math_fct_3d_linear));
        if (data->linear == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_3d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_3d_linear(ctx, bit_state, data->linear, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_3d_linear\n");
            return code;
        }
        break;

    case PRC_TYPE_MATH_FCT_3D_nonLinear:
        data->nonlinear = (prc_math_fct_3d_nonlinear *)prc_calloc(ctx, 1, sizeof(prc_math_fct_3d_nonlinear));
        if (data->nonlinear == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_parse_math_fct_3d\n");
            return PRC_ERROR_MEMORY;
        }
        code = prc_parse_math_fct_3d_nonlinear(ctx, bit_state, data->nonlinear, DONT_READ_TAG);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_parse_math_fct_3d_nonlinear\n");
            return code;
        }
    default:
        prc_error(ctx, PRC_ERROR_PARSE, "Parsing error in prc_parse_math_fct_3d\n");
        return PRC_ERROR_PARSE;
    };

    return 0;
}
