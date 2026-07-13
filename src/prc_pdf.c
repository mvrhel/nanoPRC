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
#include <memory.h>
#include <string.h>
#include <stdio.h>
#include "prc_context.h"
#include "prc_pdf.h"
#include "prc_parse_file_structure.h"

int prc_uncompress(prc_context *ctx, const unsigned char *src, size_t src_len,
    unsigned char **dst);

static int
pdf_extract_prc_internal(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
    uint8_t **buff_out, uint32_t *size_out, prc_pdf_view_array **views,
    uint32_t *number_views, uint32_t recursive_depth);

static int
pdf_cleanup_seen_ptr(void **seen, uint32_t seen_count, void *p)
{
    uint32_t i;
    for (i = 0; i < seen_count; i++)
    {
        if (seen[i] == p)
        {
            return 1;
        }
    }
    return 0;
}

static void
pdf_cleanup_free_once(prc_context *ctx, void *p, const char *label,
    void **seen, uint32_t *seen_count, uint32_t seen_capacity)
{
#if PRC_DEBUG_MEMORY
    char msg[192];
#endif

    if (p == NULL)
    {
        return;
    }

    if (pdf_cleanup_seen_ptr(seen, *seen_count, p))
    {
#if PRC_DEBUG_MEMORY
        snprintf(msg, sizeof(msg),
            "Cleanup duplicate pointer skipped %s (%p)\n", label, p);
        prc_error(ctx, PRC_ERROR_PARSE, msg);
#endif
        return;
    }

    if (*seen_count < seen_capacity)
    {
        seen[*seen_count] = p;
        (*seen_count)++;
    }

#if PRC_DEBUG_MEMORY
    snprintf(msg, sizeof(msg), "Cleanup free %s (%p)\n", label, p);
    prc_error(ctx, PRC_ERROR_PARSE, msg);
#endif
    prc_free(ctx, p);
}

static void
pdf_cleanup_extract_state(prc_context *ctx, prc_pdf_head_xref *head_xref,
    prc_pdf_uncompressed_object_stream_list *stream_list, uint32_t **xref_offsets)
{
    uint32_t k;
    uint32_t seen_capacity = 8;
    uint32_t seen_count = 0;
    void **seen_ptrs = NULL;

    if (stream_list != NULL && stream_list->number_streams > 0)
    {
        seen_capacity = stream_list->number_streams * 2 + 8;
    }

    seen_ptrs = (void **)malloc(sizeof(void *) * seen_capacity);
    if (seen_ptrs == NULL)
    {
        seen_capacity = 0;
    }

    if (stream_list != NULL && stream_list->number_streams > 0 && stream_list->ustream != NULL)
    {
        for (k = 0; k < stream_list->number_streams; k++)
        {
            char label[80];
            snprintf(label, sizeof(label), "stream_list->ustream[%u].stream", k);
            pdf_cleanup_free_once(ctx, stream_list->ustream[k].stream, label,
                seen_ptrs, &seen_count, seen_capacity);
            stream_list->ustream[k].stream = NULL;

            snprintf(label, sizeof(label), "stream_list->ustream[%u].object_offsets", k);
            pdf_cleanup_free_once(ctx, stream_list->ustream[k].object_offsets, label,
                seen_ptrs, &seen_count, seen_capacity);
            stream_list->ustream[k].object_offsets = NULL;
        }

        pdf_cleanup_free_once(ctx, stream_list->ustream, "stream_list->ustream",
            seen_ptrs, &seen_count, seen_capacity);
        stream_list->ustream = NULL;
        stream_list->number_streams = 0;
    }

    if (xref_offsets != NULL && *xref_offsets != NULL)
    {
        pdf_cleanup_free_once(ctx, *xref_offsets, "xref_offsets",
            seen_ptrs, &seen_count, seen_capacity);
        *xref_offsets = NULL;
    }

    if (head_xref != NULL && head_xref->xref_objects != NULL)
    {
        pdf_cleanup_free_once(ctx, head_xref->xref_objects, "head_xref->xref_objects",
            seen_ptrs, &seen_count, seen_capacity);
        head_xref->xref_objects = NULL;
        head_xref->num_objects = 0;
    }

    if (seen_ptrs != NULL)
    {
        free(seen_ptrs);
    }
}

int
pdf_eat_white_space(prc_context *ctx, uint8_t **ptr, uint8_t *boundary)
{
    while (**ptr == ' ' || **ptr == '\n' || **ptr == '\r' || **ptr == '\t')
    {
        (*ptr)++;
        if (*ptr == boundary)
        {
            return PRC_ERROR_INTERNAL;
        }
    }
    return 0;
}

static int
pdf_get_next_line(prc_context *ctx, uint8_t **ptr, uint8_t *boundary)
{
    while (**ptr != '\n')
    {
        (*ptr)++;
        if (*ptr == boundary)
        {
            return PRC_ERROR_INTERNAL;
        }
    }
    (*ptr)++;
    return 0;
}

int32_t
pdf_search_for_tag(prc_context *ctx, uint8_t *ptr, uint8_t *boundary,
    uint8_t *tag_name, uint32_t tag_name_len, uint8_t *bound_tag_name,
    uint32_t bound_tag_name_len, uint8_t *found)
{
    uint32_t count = 0;

    *found = 0;
    while (ptr < boundary)
    {
        /* The immediate character after the string we are searching for can not be
           another letter character */
        if (strncmp((const char *)ptr, (const char *)tag_name, tag_name_len) == 0 &&
            !((ptr + tag_name_len < boundary) &&
            (((* (ptr + tag_name_len) >= 'a') && (* (ptr + tag_name_len) <= 'z')) ||
             ((* (ptr + tag_name_len) >= 'A') && (* (ptr + tag_name_len) <= 'Z')))))
        {
            *found = 1;
            return count;
        }
        if (bound_tag_name_len != 0)
        {
            if (strncmp((const char *)ptr, (const char *)bound_tag_name, bound_tag_name_len) == 0)
            {
                return 0;
            }
        }
        ptr++;
        count++;
    }
    return 0;
}

int
pdf_get_stream_info(prc_context *ctx, uint8_t *ptr_in, uint8_t *boundary,
    uint8_t **ptr_stream_start, uint32_t *stream_length, uint32_t *obj_num,
    uint32_t *gen_num)
{
    uint8_t *ptr_stream = NULL;
    uint8_t *ptr = ptr_in;
    int code;

    /* Read the obj_num and gen_num */
    code = sscanf((const char *)ptr, "%u %u obj", obj_num, gen_num);
    if (code != 2)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read object number and generation number in PDF object\n");
        return PRC_ERROR_PARSE;
    }

    while (ptr < boundary)
    {
        if (strncmp((const char *)ptr, PDF_STREAM_NAME, PDF_STREAM_NAME_LEN) == 0)
        {
            /* Get to the next line */
            ptr += PDF_STREAM_NAME_LEN;
            code = pdf_get_next_line(ctx, &ptr, boundary);
            if (code < 0)
            {
                return code;
            }
            ptr_stream = ptr;
            break;
        }

        /* If we run into endobj that is an error */
        if (strncmp((const char *)ptr, PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN) == 0)
        {
            //prc_error(ctx, PRC_ERROR_PARSE, "Did not find stream in PDF object\n");
            return PRC_ERROR_PARSE;
        }
        ptr++;
    }

    if (ptr_stream == NULL)
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not find stream in PDF object\n");
        return PRC_ERROR_PARSE;
    }
    *ptr_stream_start = ptr_stream;

    /* Also get the Length */
    ptr = ptr_in;
    while (ptr < boundary)
    {
        if (strncmp((const char *)ptr, PDF_LENGTH_NAME, PDF_LENGTH_NAME_LEN) == 0)
        {
            ptr += PDF_LENGTH_NAME_LEN;
            code = pdf_eat_white_space(ctx, &ptr, boundary);
            if (code < 0)
            {
                return code;
            }
            /* Read the stream length */
            code = sscanf((const char*) ptr, "%u", stream_length);
            if (code != 1)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read stream length in PDF object\n");
                return PRC_ERROR_PARSE;
            }

            /* The /Length value is attacker-controlled. Validate it against the
               bytes actually remaining after the stream start in this buffer
               before any caller uses it to allocate/copy, otherwise a crafted
               /Length larger than the real stream causes an out-of-bounds read. */
            if (ptr_stream > boundary || (size_t)*stream_length > (size_t)(boundary - ptr_stream))
            {
                prc_error(ctx, PRC_ERROR_PARSE, "PDF stream /Length exceeds available data\n");
                return PRC_ERROR_PARSE;
            }
            break;
        }

        /* If we hit stream then that is an error */
        if (strncmp((const char *)ptr, PDF_STREAM_NAME, PDF_STREAM_NAME_LEN) == 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find Length in PDF object\n");
            return PRC_ERROR_PARSE;
        }
        ptr++;
    }
    return 0;
}

/* Brute force search for a particular object number */
static int
pdf_get_ptr_to_obj(prc_context *ctx, uint32_t object_num_in, uint8_t *ptr_in,
    uint8_t *boundary, uint8_t **ptr_out)
{
    uint8_t *ptr = ptr_in;
    uint8_t *ptr_temp;
    int code;
    uint32_t object_num = 0;

    while (ptr < boundary)
    {
        if (strncmp((const char *)ptr, PDF_OBJECT_NAME, PDF_OBJECT_NAME_LEN) == 0)
        {
            /* Make sure we are not at an endobj. Only look 3 bytes back if
               there actually are 3 bytes before ptr in this buffer. */
            if (ptr - ptr_in >= 3 && strncmp((const char *)(ptr - 3), PDF_ENDOBJ_NAME,
                PDF_ENDOBJ_NAME_LEN) == 0)
            {
                ptr += 3;
                continue;
            }
            /* Go to the start of the line */
            ptr_temp = ptr + PDF_OBJECT_NAME_LEN;
            while (ptr > ptr_in && !(*(ptr - 1) == '\n' || *(ptr - 1) == '\r'))
            {
                ptr--;
            }
            code = pdf_eat_white_space(ctx, &ptr, boundary);
            if (code < 0)
            {
                return code;
            }
            /* Read the object number */
            code = sscanf((const char*) ptr, "%u", &object_num);
            if (code != 1)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read object number in PDF file\n");
                return PRC_ERROR_PARSE;
            }
            if (object_num == object_num_in)
            {
                /* We have found the object */
                code = pdf_eat_white_space(ctx, &ptr_temp, boundary);
                if (code < 0)
                {
                    return code;
                }
                *ptr_out = ptr_temp;
                return 0;
            }
            else
            {
                /* Not the object we are looking for. Go to the end of the line */
                ptr = ptr_temp;
            }
        }
        ptr++;
    }
    *ptr_out = NULL;
    return PRC_ERROR_INTERNAL;
}

/* Will return deflated buffer or buffer if not encoded */
int
pdf_get_stream_data(prc_context *ctx, uint8_t *ptr_in_obj, uint8_t *boundary,
    uint8_t *ptr_in_stream, uint32_t stream_length, uint8_t **buff_out,
    uint32_t *size_out)
{
    uint8_t *ptr;
    uint8_t must_deflate = 0;
    int code;
    uint8_t keyout[PDF_MAX_DICT_VALUE];
    uint32_t actual_keyout_len;

    ptr = ptr_in_obj;

    /* See if we need to deflate */
    code = prc_pdf_dict_get_type(ctx, ptr, boundary,
        PDF_FILTER_NAME, keyout, PDF_FLATEDECODE_NAME_LEN, &actual_keyout_len);
    if (code > 0 && strncmp((const char*) keyout, PDF_FLATEDECODE_NAME, actual_keyout_len) == 0)
    {
        must_deflate = 1;
    }

    if (!must_deflate)
    {
        /* The stream is not encoded */
        *buff_out = (uint8_t *)prc_malloc(ctx, stream_length);
        if (*buff_out == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate buff_out\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(*buff_out, ptr_in_stream, stream_length);
        *size_out = stream_length;
    }
    else
    {
        /* The stream is deflated. We will inflate it here */
        uint8_t *buff_deflated;
        uint32_t size;

        buff_deflated = (uint8_t *)prc_malloc(ctx, stream_length);
        if (buff_deflated == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate buff\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(buff_deflated, ptr_in_stream, stream_length);
        code = prc_uncompress(ctx, buff_deflated, stream_length, buff_out);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_uncompress\n");
            prc_free(ctx, buff_deflated);
            if (*buff_out != NULL)
            {
                prc_free(ctx, *buff_out);
                *buff_out = NULL;
            }
            return code;
        }
        prc_free(ctx, buff_deflated);
        *size_out = code;
    }
    return 0;
}

/* Deals with encodings of the internal and external names for the views. Lenght
   is zero if we have a (text) encoding and dont know the length */
static int
pdf_parse_text_prc(prc_context *ctx, uint32_t length, uint8_t *ptr_in,
    uint8_t *boundary, char **name_out)
{
    uint8_t *ptr_temp = ptr_in;
    uint8_t *str_start;
    int code;
    char *name;
    uint32_t len_name = 0;
    uint8_t has_special_char = 0;
    uint8_t null_at_start = 0;
    uint8_t null_at_end = 0;
    uint32_t count = 0;

    /* Look at the first 2 bytes to see if we have FEFF or FFFE which indicate
       unicode encoding. This is English centric here. This really should be
       done differently, but that is not the focus of this project */
    if (ptr_temp[0] == 0xFE && ptr_temp[1] == 0xFF)
    {
        /* Null then character */
        ptr_temp += 2;
        null_at_start = 1;
    }
    else if (ptr_temp[0] == 0xFF && ptr_temp[1] == 0xFE)
    {
        /* Character then Null. Need to see this in a file still */
        ptr_temp += 2;
        null_at_end = 1;
    }
    count = count + 2;

    /* Read the characters into a string until we get to )
       But this is only the case if we dont know the length */
    while (!(*ptr_temp == ')' || (length != 0 && count == length)))
    {
        if (null_at_start)
        {
            ptr_temp++;
            count++;
        }
        if (ptr_temp >= boundary)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read ) in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        if (*ptr_temp == 0x5c && (*(ptr_temp+1) == ')' || *(ptr_temp+1) == '(')) /* This is a special case of \. We need to read the next character */
        {
            has_special_char = 1;
            ptr_temp++;
            count++;
            if (ptr_temp >= boundary)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read ) in PDF file\n");
                return PRC_ERROR_PARSE;
            }
        }
        ptr_temp++;
        len_name++;
        count++;
        if (null_at_end)
        {
            ptr_temp++;
            count++;
        }
    }
    /* Now we have the string */
    name = (char *)prc_malloc(ctx, len_name + 1);
    if (name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate name\n");
        return PRC_ERROR_MEMORY;
    }

    if (null_at_end || null_at_start)
    {
        str_start = ptr_in + 2;
    }
    else
    {
        str_start = ptr_in;
    }

    if (has_special_char || null_at_end || null_at_start)
    {
        /* We need to copy the string and replace the special ( ) character
            and or get us to simple character string */
        char *ptr_temp2 = name;
        while (str_start < ptr_temp)
        {
            if (null_at_start && *str_start == 0x00)
            {
                str_start++;
            }
            if (*str_start == 0x5c && (*(str_start + 1) == ')' || *(str_start + 1) == '(') )
            {
                *ptr_temp2 = *(str_start + 1);
                str_start++;
            }
            else
            {
                *ptr_temp2 = *str_start;
            }
            str_start++;
            ptr_temp2++;
            if (null_at_end && *str_start == 0x00)
            {
                str_start++;
            }
        }
        *ptr_temp2 = '\0';
    }
    else
    {
        memcpy(name, str_start, ptr_temp - str_start);
        name[ptr_temp - str_start] = '\0';
    }
    *name_out = name;

    return 0;
}

static int
prc_pdf_get_hexstring(prc_context *ctx, prc_pdf_decrypt_params *decrypt_params,
                      uint32_t obj_num, uint32_t gen_num, uint8_t *input_buffer,
                      uint8_t *buffer_end, char **name_out)
{
    uint8_t *ptr = input_buffer;
    size_t k = 0;
    uint8_t evennibble = 1;
    uint32_t num_digits = 0;
    uint32_t num_to_allocate;
    uint8_t found_end = 0;
    char *name;
    uint8_t *name_decrypted = NULL;
    uint32_t decypted_size = 0;
    uint8_t is_encrypted = (decrypt_params->version != 0);
    int code;

    /* First find the length of the string by counting the digits until we
       get to > */
    while (ptr < buffer_end)
    {
        if (ptr[0] == '>')
        {
            found_end = 1;
            break;
        }
        ptr++;
        num_digits++;
    }

    if (found_end == 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "No characters in encoded string\n");
        return PRC_ERROR_PARSE;
    }

    if ((num_digits % 2) == 1)
    {
        num_to_allocate = (num_digits + 1) / 2;
    }
    else
    {
        num_to_allocate = num_digits / 2;
    }

    name = (char*) prc_calloc(ctx, num_to_allocate, 1);
    if (name == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed in allocation of view name\n");
        return PRC_ERROR_MEMORY;
    }

    ptr = input_buffer;
    while (ptr < buffer_end)
    {
        if (ptr[0] == '>')
            break;
        if ((ptr[0] >= '0' && ptr[0] <= '9') ||
            (ptr[0] >= 'A' && ptr[0] <= 'F') ||
            (ptr[0] >= 'a' && ptr[0] <= 'f'))
        {
            if (evennibble)
            {
                name[k] = 0;
                if (ptr[0] >= '0' && ptr[0] <= '9')
                    name[k] = (ptr[0] - '0') << 4;
                else if (ptr[0] >= 'A' && ptr[0] <= 'F')
                    name[k] = (ptr[0] - 'A' + 10) << 4;
                else
                    name[k] = (ptr[0] - 'a' + 10) << 4;
                evennibble = 0;
            }
            else
            {
                if (ptr[0] >= '0' && ptr[0] <= '9')
                    name[k] |= (ptr[0] - '0');
                else if (ptr[0] >= 'A' && ptr[0] <= 'F')
                    name[k] |= (ptr[0] - 'A' + 10);
                else
                    name[k] |= (ptr[0] - 'a' + 10);
                evennibble = 1;
                k++;
            }
        }
        ptr++;
    }

    /* Now we may need to decrypt this */
    if (is_encrypted)
    {
        code = pdf_decrypt_string(ctx, (uint8_t*) name, num_to_allocate, decrypt_params,
            obj_num, gen_num, &name_decrypted, &decypted_size);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in pdf_decrypt_string\n");
            prc_free(ctx, name);
            prc_free(ctx, name_decrypted);
            return code;
        }

        /* Now we need to decode it */
        code = pdf_parse_text_prc(ctx, decypted_size, name_decrypted,
            name_decrypted + decypted_size, name_out);

        prc_free(ctx, name_decrypted);
        prc_free(ctx, name);
        return code;
    }
    *name_out = name;
    return 0;
}

static void
pdf_free_view_array_partial(prc_context *ctx, prc_pdf_view_array *views,
    uint32_t count)
{
    uint32_t i;

    if (views == NULL)
    {
        return;
    }

    for (i = 0; i < count; i++)
    {
        prc_free(ctx, views[i].external_name);
        prc_free(ctx, views[i].internal_name);
    }
    prc_free(ctx, views);
}

/* Parse the view object. Ideally things are not references. If they are, this
   will fail. TODO */
static int
pdf_parse_view_prc(prc_context *ctx, prc_pdf_decrypt_params *decrypt_params,
                   uint32_t obj_num, uint32_t gen_num, uint8_t *ptr_in,
                   uint8_t *boundary, prc_pdf_view_array *view)
{
    /* Look for PDF_C2W_NAME, PDF_CO_NAME, PDF_IN_NAME, PDF_XN_NAME before endobj */
    int code;
    uint8_t *ptr_temp = ptr_in;
    uint8_t *str_start;
    uint8_t found;

    code = pdf_search_for_tag(ctx, ptr_temp, boundary, (uint8_t*) PDF_C2W_NAME, PDF_C2W_NAME_LEN,
                             (uint8_t *) PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN, &found);
    if (found)
    {
        ptr_temp += code;
        ptr_temp += PDF_C2W_NAME_LEN;
        /* Get rid of white spaces and read the [ */
        code = pdf_eat_white_space(ctx, &ptr_temp, boundary);
        if (code < 0)
        {
            return code;
        }
        if (strncmp((const char *)ptr_temp, "[", 1) != 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read [ in PDF file\n");
            return PRC_ERROR_PARSE;
        }
        ptr_temp++;
        /* Now read the 12 matrix entries */
        code = sscanf((const char *)ptr_temp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &view->matrix[0], &view->matrix[1], &view->matrix[2], &view->matrix[3],
            &view->matrix[4], &view->matrix[5], &view->matrix[6], &view->matrix[7],
            &view->matrix[8], &view->matrix[9], &view->matrix[10], &view->matrix[11]);
        if (code != 12)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read matrix in PDF file\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find C2W in PDF file\n");
        return code;
    }
    ptr_temp = ptr_in;
    code = pdf_search_for_tag(ctx, ptr_temp, boundary, (uint8_t *) PDF_CO_NAME, PDF_CO_NAME_LEN,
                              (uint8_t *) PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN, &found);
    if (found)
    {
        /* Read in a single double value */
        ptr_temp += code;
        ptr_temp += PDF_CO_NAME_LEN;
        code = pdf_eat_white_space(ctx, &ptr_temp, boundary);
        if (code < 0)
        {
            return code;
        }
        code = sscanf((const char *)ptr_temp, "%lf", &view->center_orbit_z);
        if (code != 1)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read center orbit in PDF file\n");
            return PRC_ERROR_PARSE;
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find CO in PDF file\n");
        return code;
    }

#if 0
    ptr_temp = ptr_in;
    code = pdf_search_for_tag(ctx, ptr_temp, boundary, (uint8_t *) PDF_IN_NAME, PDF_IN_NAME_LEN,
                              (uint8_t *) PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN, &found);
    if (found)
    {
        /* Read in a string that is bracketed by parenthesis ( ) or < > */
        ptr_temp += code;
        ptr_temp += PDF_IN_NAME_LEN;
        code = pdf_eat_white_space(ctx, &ptr_temp, boundary);
        if (code < 0)
        {
            return code;
        }

        /* We can have a literal string or a hexadecimal encoded string */
        if (!(strncmp((const char *)ptr_temp, "(", 1) == 0 || strncmp((const char *)ptr_temp, "<", 1) == 0))
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read ( or < in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        if (strncmp((const char *)ptr_temp, "(", 1) == 0)
        {
            ptr_temp++;
            str_start = ptr_temp;

            code = pdf_parse_text_prc(ctx, 0, str_start, boundary, &view->internal_name);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read internal name in PDF file\n");
                return code;
            }
        }
        else
        {
            /* String is hex encoded and may be encrypted */
            ptr_temp++;
            str_start = ptr_temp;

            code = prc_pdf_get_hexstring(ctx, decrypt_params, obj_num, gen_num,
                                    str_start, boundary, &view->internal_name);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read internal name in PDF file\n");
                return code;
            }
        }
    } /* That was optional content. No need to throw an error if it was not found */
#endif
    ptr_temp = ptr_in;
    code = pdf_search_for_tag(ctx, ptr_temp, boundary, (uint8_t *) PDF_XN_NAME, PDF_XN_NAME_LEN,
                              (uint8_t *) PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN, &found);
    if (found)
    {
        ptr_temp += code;
        ptr_temp += PDF_XN_NAME_LEN;
        /* Read in a string that is bracketed by parenthesis ( ) */
        code = pdf_eat_white_space(ctx, &ptr_temp, boundary);
        if (code < 0)
        {
            return code;
        }

        /* We can have a literal string or a hexadecimal encoded string */
        if (!(strncmp((const char *)ptr_temp, "(", 1) == 0 ||
              strncmp((const char *)ptr_temp, "<", 1) == 0))
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read ( or < in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        if (strncmp((const char *)ptr_temp, "(", 1) == 0)
        {
            ptr_temp++;
            str_start = ptr_temp;

            code = pdf_parse_text_prc(ctx, 0, str_start, boundary, &view->external_name);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read externaL name in PDF file\n");
                return code;
            }
        }
        else
        {
            /* String is hex encoded */
            ptr_temp++;
            str_start = ptr_temp;

            code = prc_pdf_get_hexstring(ctx, decrypt_params, obj_num, gen_num,
                                         str_start, boundary, &view->external_name);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read externaL name in PDF file\n");
                return code;
            }
        }
    }
    else
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find XN in PDF file\n");
        return code;
    }
    return 0;
}

/* Parse the values %d %d R that are contained with the brackets [ to  ]
   When we call this we should already be sitting on a [ as the next character */
static int
pdf_parse_array_prc(prc_context *ctx, uint8_t *boundary,
                    uint8_t *ptr_in, uint32_t *num_entries, uint32_t **list_obj,
                    uint32_t **list_gen)
{
    int code;
    uint8_t *ptr = ptr_in;
    uint32_t val1, val2;
    uint32_t index = 0;
    uint32_t *obj_entries = NULL;
    uint32_t *gen_entries = NULL;

    if (*ptr != '[')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read [ in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    ptr++;

    /* First find out how many R values there are before we hit ] */
    *num_entries = 0;
    while (ptr < boundary && *ptr != ']')
    {
        if (strncmp((const char *)ptr, "R", 1) == 0)
        {
            (*num_entries)++;
        }
        ptr++;
    }
    if (ptr >= boundary || *ptr != ']')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read ] in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    if (*num_entries == 0)
    {
        /* No views in the array. I have seen this before*/
        return 0;
    }
    /* Now we have to allocate the entries */
    gen_entries = (uint32_t *)prc_calloc(ctx, *num_entries, sizeof(uint32_t));
    if (gen_entries == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate entries\n");
        return PRC_ERROR_MEMORY;
    }
    obj_entries = (uint32_t *)prc_calloc(ctx, *num_entries, sizeof(uint32_t));
    if (obj_entries == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate entries\n");
        prc_free(ctx, gen_entries);
        return PRC_ERROR_MEMORY;
    }

    /* Now we have to read the entries */
    ptr = ptr_in + 1;
    while (ptr < boundary && *ptr != ']')
    {
        code = sscanf((const char *)ptr, "%u %u", &val1, &val2);
        if (code != 2)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read entry in PDF file\n");
            goto fail;
        }
        /* Eat white space and the R character */
        code = pdf_eat_white_space(ctx, &ptr, boundary);
        if (code < 0)
        {
            goto fail;
        }
        obj_entries[index] = val1;
        gen_entries[index] = val2;
        index++;

        /* Move ptr past 'R' */
        while (*ptr != 'R')
        {
            ptr++;
            if (ptr >= boundary)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read R in PDF file\n");
                goto fail;
            }
        }
        ptr++;
        /* Eat any white space */
        code = pdf_eat_white_space(ctx, &ptr, boundary);
        if (code < 0)
        {
            goto fail;
        }
    }
    if (ptr >= boundary || *ptr != ']')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read ] in PDF file\n");
        goto fail;
    }
    *list_obj = obj_entries;
    *list_gen = gen_entries;
    return 0;

fail:
    prc_free(ctx, obj_entries);
    prc_free(ctx, gen_entries);
    return (code < 0) ? code : PRC_ERROR_PARSE;
}

/* Working from the object where the 3D stream is defined get the dictionary
   that has the views. Those entries could be all there or we could be
   referencing another object that has the entries. Use the xref object to
   search and return them */
static int
pdf_get_view_array_prc(prc_context *ctx, prc_pdf_decrypt_params *decrypt_params,
    prc_pdf_head_xref *head_xref, prc_pdf_uncompressed_object_stream_list *stream_list,
    uint8_t *pdf_buff_in, uint32_t size_in, uint8_t *ptr, uint32_t *num_views,
    prc_pdf_view_array **views)
{
    int code;
    uint32_t object_ref_num = 0;
    uint32_t object_generation_num;
    uint8_t found_array = 0;
    uint8_t found_ref = 0;
    uint32_t *dict_obj_num = NULL;
    uint32_t *dict_gen_num = NULL;
    uint8_t *ptr_temp;
    prc_pdf_view_array *cam_views = NULL;
    uint32_t k;
    uint8_t found_object = 0;
    uint32_t size_out;
    uint32_t offset;
    uint8_t is_object_stream_item = 0;
    uint8_t *array_boundary = pdf_buff_in + size_in;

    *num_views = 0;

    /* Read until we get /VA */
    while (ptr < pdf_buff_in + size_in)
    {
        if (strncmp((const char *)ptr, PDF_VA_NAME, PDF_VA_NAME_LEN) == 0)
        {
            ptr += PDF_VA_NAME_LEN;
            code = pdf_eat_white_space(ctx, &ptr, pdf_buff_in + size_in);
            if (code < 0)
            {
                return code;
            }

            /* This will either be an array or an object reference */
            if (*ptr == '[')
            {
                /* We have an array */
                found_array = 1;
            }
            else
            {
                /* We have an object reference */
                code = sscanf((const char *)ptr, "%u %u", &object_ref_num, &object_generation_num);
                if (code != 2)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not read object in PDF file\n");
                    return PRC_ERROR_PARSE;
                }
                found_ref = 1;
                /* Go to that location */
#if 0
                uint8_t *ptr_temp;
                code = pdf_get_ptr_to_obj(ctx, object_ref_num, pdf_buff_in,
                    pdf_buff_in + size_in, &ptr_temp);
                if (code < 0)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not read object in PDF file\n");
                    return code;
                }
                ptr = ptr_temp;
                if (*ptr == '[')
                {
                    /* We have a dictionary */
                    found_dict = 1;
                }
#endif
            }
            break;
        }
        ptr++;
    }

    if (!found_array && found_ref)
    {
        /* We have not found the array yet. The VA entry referenced a single
           item. I suppose this could be a single view, but that would be weird.
           For now lets just assume this reference will reference an array */
        if (head_xref == NULL)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "No xref table in PDF file\n");
            return PRC_ERROR_PARSE;
        }
        /* Search through the xref table for the object reference */
        ptr_temp = prc_pdf_get_ptr_to_obj(ctx, pdf_buff_in, size_in,
            head_xref, stream_list, object_ref_num, &size_out,
            &is_object_stream_item);

        if (ptr_temp == NULL)
        {
            /* Fallback for files where this object is not indexed by current xref data.
               This only finds direct objects (not compressed object stream members). */
            code = pdf_get_ptr_to_obj(ctx, object_ref_num, pdf_buff_in,
                pdf_buff_in + size_in, &ptr_temp);
            if (code < 0 || ptr_temp == NULL)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not find object in PDF file\n");
                return PRC_ERROR_PARSE;
            }
            is_object_stream_item = 0;
            size_out = size_in - (uint32_t)(ptr_temp - pdf_buff_in);
        }

        if (!is_object_stream_item)
        {
            /* Get past PDF_OBJECT_NAME */

            offset = pdf_search_for_tag(ctx, ptr_temp, ptr_temp + size_out,
                (uint8_t *) PDF_OBJECT_NAME, PDF_OBJECT_NAME_LEN,
                (uint8_t *) PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN, &found_object);
            if (!found_object)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not find object in PDF file\n");
                return PRC_ERROR_PARSE;
            }
            ptr_temp += offset + PDF_OBJECT_NAME_LEN;

        }
        /* Eat white space and then check for [ */
        code = pdf_eat_white_space(ctx, &ptr_temp, pdf_buff_in + size_in);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
            return code;
        }
        if (*ptr_temp == '[')
        {
            found_array = 1;
            ptr = ptr_temp;
            if (is_object_stream_item)
            {
                array_boundary = ptr_temp + size_out;
            }
            else
            {
                array_boundary = pdf_buff_in + size_in;
            }
        }
    }

    if (found_array)
    {
        code = pdf_parse_array_prc(ctx, array_boundary,
            ptr, num_views, &dict_obj_num, &dict_gen_num);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not parse dictionary in PDF file\n");
            return code;
        }
        if (*num_views > 0)
        {
            cam_views = (prc_pdf_view_array *)prc_calloc(ctx, *num_views,
                                                    sizeof(prc_pdf_view_array));
            if (cam_views == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate views\n");
                prc_free(ctx, dict_obj_num);
                prc_free(ctx, dict_gen_num);
                return PRC_ERROR_MEMORY;
            }
            for (k = 0; k < *num_views; k++)
            {
                /* Search for that object using the xref table */
                ptr_temp = prc_pdf_get_ptr_to_obj(ctx, pdf_buff_in, size_in,
                    head_xref, stream_list, dict_obj_num[k], &size_out,
                    &is_object_stream_item);
                if (ptr_temp == NULL)
                {
                    /* Fallback for direct objects missing from xref index state. */
                    code = pdf_get_ptr_to_obj(ctx, dict_obj_num[k], pdf_buff_in,
                        pdf_buff_in + size_in, &ptr_temp);
                    if (code < 0 || ptr_temp == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_PARSE, "Did not find object in PDF file\n");
                        pdf_free_view_array_partial(ctx, cam_views, k);
                        prc_free(ctx, dict_obj_num);
                        prc_free(ctx, dict_gen_num);
                        return PRC_ERROR_PARSE;
                    }
                    is_object_stream_item = 0;
                    size_out = size_in - (uint32_t)(ptr_temp - pdf_buff_in);
                }

                /* And now parse the camera view object */
                code = pdf_parse_view_prc(ctx, decrypt_params, dict_obj_num[k],
                                          dict_gen_num[k], ptr_temp,
                                          ptr_temp + size_out, &cam_views[k]);
                if (code < 0)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not parse view in PDF file\n");
                    pdf_free_view_array_partial(ctx, cam_views, k + 1);
                    prc_free(ctx, dict_obj_num);
                    prc_free(ctx, dict_gen_num);
                    return code;
                }
            }
            prc_free(ctx, dict_obj_num);
            prc_free(ctx, dict_gen_num);
        }
    }
    *views = cam_views;
    return 0;
}

static int
pdf_char_to_lower(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'Z')
    {
        return (uint8_t)(ch - 'A' + 'a');
    }
    return ch;
}

static int
pdf_buffer_contains_ascii_case_insensitive(const uint8_t *buf, uint32_t buf_len,
    const char *needle)
{
    uint32_t i;
    uint32_t j;
    uint32_t needle_len = (uint32_t)strlen(needle);

    if (needle_len == 0 || buf_len < needle_len)
    {
        return 0;
    }

    for (i = 0; i + needle_len <= buf_len; i++)
    {
        for (j = 0; j < needle_len; j++)
        {
            if (pdf_char_to_lower(buf[i + j]) !=
                pdf_char_to_lower((uint8_t)needle[j]))
            {
                break;
            }
        }
        if (j == needle_len)
        {
            return 1;
        }
    }
    return 0;
}

static int
pdf_buffer_looks_like_pdf(const uint8_t *buf, uint32_t buf_len)
{
    uint32_t i;
    uint32_t head_scan_len;
    uint32_t tail_scan_len;
    const uint8_t *tail_ptr;

    if (buf == NULL || buf_len < 16)
    {
        return 0;
    }

    head_scan_len = (buf_len < 1024) ? buf_len : 1024;
    for (i = 0; i + 5 <= head_scan_len; i++)
    {
        if (memcmp(buf + i, "%PDF-", 5) == 0)
        {
            break;
        }
    }
    if (i + 5 > head_scan_len)
    {
        return 0;
    }

    tail_scan_len = (buf_len < 8192) ? buf_len : 8192;
    tail_ptr = buf + (buf_len - tail_scan_len);
    if (!pdf_buffer_contains_ascii_case_insensitive(tail_ptr, tail_scan_len,
        "startxref"))
    {
        return 0;
    }

    return 1;
}

static int
pdf_try_extract_from_embedded_pdfs(prc_context *ctx, uint8_t *pdf_buff_in,
    uint32_t size_in, prc_pdf_head_xref *head_xref,
    prc_pdf_uncompressed_object_stream_list *stream_list,
    uint8_t streams_encrypted, prc_pdf_decrypt_params *decrypt_params,
    uint8_t **buff_out,
    uint32_t *size_out, prc_pdf_view_array **views, uint32_t *number_views,
    uint32_t recursive_depth)
{
    uint32_t j;
    uint32_t tried_candidates = 0;
    const uint32_t max_candidates = 64;
    uint8_t *file_end = pdf_buff_in + size_in;

    if (head_xref == NULL || head_xref->xref_objects == NULL)
    {
        return 0;
    }

    for (j = 0; j < head_xref->num_objects; j++)
    {
        uint8_t *filespec_ptr;
        uint8_t *ef_dict_start;
        uint8_t *ef_dict_end;
        uint8_t keyout[PDF_MAX_DICT_VALUE];
        uint32_t actual_keyout_len;
        uint32_t filespec_size;
        uint32_t embedded_obj_num = 0;
        uint32_t embedded_gen_num = 0;
        uint8_t is_object_stream_item = 0;
        int code;

        if (tried_candidates >= max_candidates)
        {
            break;
        }

        if (head_xref->xref_objects[j].type != PRC_PDF_XREF_USED_TYPE)
        {
            continue;
        }

        filespec_ptr = pdf_buff_in + head_xref->xref_objects[j].byte_offset;

        code = prc_pdf_dict_get_type(ctx, filespec_ptr, file_end,
            PDF_TYPE_NAME, keyout, PDF_MAX_DICT_VALUE, &actual_keyout_len);
        if (code <= 0 || strncmp((const char *)keyout, "/Filespec", actual_keyout_len) != 0)
        {
            continue;
        }

        filespec_ptr = prc_pdf_get_ptr_to_obj(ctx, pdf_buff_in, size_in, head_xref,
            stream_list, head_xref->xref_objects[j].object_number,
            &filespec_size, &is_object_stream_item);
        if (filespec_ptr == NULL)
        {
            continue;
        }

        code = prc_pdf_dict_get_dict(ctx, filespec_ptr, filespec_ptr + filespec_size,
            PDF_EF_NAME, &ef_dict_start, &ef_dict_end);
        if (code <= 0)
        {
            continue;
        }

        code = prc_pdf_dict_get_ref(ctx, ef_dict_start, ef_dict_end, PDF_F_NAME,
            &embedded_obj_num, &embedded_gen_num);
        if (code <= 0 || embedded_obj_num == 0)
        {
            continue;
        }

        prc_error(ctx, PRC_ERROR_PARSE,
            "Embedded Filespec candidate stream ref: %u %u\n",
            embedded_obj_num, embedded_gen_num);

        {
            uint8_t *embedded_ptr;
            uint32_t embedded_size;
            uint8_t embedded_is_stream_item;
            uint8_t *ptr_stream;
            uint32_t stream_length;
            uint32_t stream_obj_num;
            uint32_t stream_gen_num;
            uint8_t *decoded_stream = NULL;
            uint32_t decoded_size = 0;

            embedded_ptr = prc_pdf_get_ptr_to_obj(ctx, pdf_buff_in, size_in,
                head_xref, stream_list, embedded_obj_num, &embedded_size,
                &embedded_is_stream_item);
            if (embedded_ptr == NULL)
            {
                continue;
            }

            code = pdf_get_stream_info(ctx, embedded_ptr, file_end, &ptr_stream,
                &stream_length, &stream_obj_num, &stream_gen_num);
            if (code < 0)
            {
                continue;
            }

            if (stream_length == 0 || stream_length > (64U * 1024U * 1024U))
            {
                continue;
            }

            if (streams_encrypted)
            {
                size_t decrypted_size = pdf_decrypt_get_size(ctx, decrypt_params,
                    stream_length);
                uint8_t *decrypted_data;
                uint32_t actual_decrypted_size;

                if (decrypted_size == 0)
                {
                    continue;
                }

                decrypted_data = (uint8_t *)prc_calloc(ctx, decrypted_size,
                    sizeof(uint8_t));
                if (decrypted_data == NULL)
                {
                    continue;
                }

                code = pdf_get_decrypted_stream_data(ctx, ptr_stream, stream_length,
                    decrypt_params, decrypted_data, decrypted_size, stream_obj_num,
                    stream_gen_num, &actual_decrypted_size);
                if (code < 0)
                {
                    prc_free(ctx, decrypted_data);
                    continue;
                }

                code = pdf_get_stream_data(ctx, embedded_ptr, file_end, decrypted_data,
                    actual_decrypted_size, &decoded_stream, &decoded_size);
                prc_free(ctx, decrypted_data);
                if (code < 0)
                {
                    continue;
                }
            }
            else
            {
                code = pdf_get_stream_data(ctx, embedded_ptr, file_end, ptr_stream,
                    stream_length, &decoded_stream, &decoded_size);
                if (code < 0)
                {
                    continue;
                }
            }

            if (decoded_stream == NULL)
            {
                continue;
            }

            if (pdf_buffer_looks_like_pdf(decoded_stream, decoded_size))
            {
                tried_candidates++;
                prc_error(ctx, PRC_ERROR_PARSE,
                    "Trying embedded child PDF (Filespec path), size=%u, attempt=%u\n",
                    decoded_size, tried_candidates);
                code = pdf_extract_prc_internal(ctx, decoded_stream, decoded_size,
                    buff_out, size_out, views, number_views, recursive_depth);
                prc_free(ctx, decoded_stream);
                if (code == 0)
                {
                    prc_error(ctx, PRC_ERROR_PARSE,
                        "Embedded child PDF extraction succeeded (Filespec path)\n");
                    return 1;
                }
                continue;
            }

            prc_free(ctx, decoded_stream);
        }
    }

    for (j = 0; j < head_xref->num_objects; j++)
    {
        uint8_t *obj_ptr;
        uint8_t *ptr_stream;
        uint32_t stream_length;
        uint32_t stream_obj_num;
        uint32_t stream_gen_num;
        uint8_t *decoded_stream = NULL;
        uint32_t decoded_size = 0;
        int code;

        if (tried_candidates >= max_candidates)
        {
            break;
        }

        if (head_xref->xref_objects[j].type != PRC_PDF_XREF_USED_TYPE)
        {
            continue;
        }

        obj_ptr = pdf_buff_in + head_xref->xref_objects[j].byte_offset;
        code = pdf_get_stream_info(ctx, obj_ptr, file_end, &ptr_stream,
            &stream_length, &stream_obj_num, &stream_gen_num);
        if (code < 0)
        {
            continue;
        }

        if (stream_length == 0 || stream_length > (64U * 1024U * 1024U))
        {
            continue;
        }

        if (streams_encrypted)
        {
            size_t decrypted_size = pdf_decrypt_get_size(ctx, decrypt_params,
                stream_length);
            uint8_t *decrypted_data;
            uint32_t actual_decrypted_size;

            if (decrypted_size == 0)
            {
                continue;
            }

            decrypted_data = (uint8_t *)prc_calloc(ctx, decrypted_size,
                sizeof(uint8_t));
            if (decrypted_data == NULL)
            {
                continue;
            }

            code = pdf_get_decrypted_stream_data(ctx, ptr_stream, stream_length,
                decrypt_params, decrypted_data, decrypted_size, stream_obj_num,
                stream_gen_num, &actual_decrypted_size);
            if (code < 0)
            {
                prc_free(ctx, decrypted_data);
                continue;
            }

            code = pdf_get_stream_data(ctx, obj_ptr, file_end, decrypted_data,
                actual_decrypted_size, &decoded_stream, &decoded_size);
            prc_free(ctx, decrypted_data);
            if (code < 0)
            {
                continue;
            }
        }
        else
        {
            code = pdf_get_stream_data(ctx, obj_ptr, file_end, ptr_stream,
                stream_length, &decoded_stream, &decoded_size);
            if (code < 0)
            {
                continue;
            }
        }

        if (decoded_stream == NULL)
        {
            continue;
        }

        if (pdf_buffer_looks_like_pdf(decoded_stream, decoded_size))
        {
            tried_candidates++;
            prc_error(ctx, PRC_ERROR_PARSE,
                "Trying embedded child PDF (stream scan path), size=%u, attempt=%u\n",
                decoded_size, tried_candidates);
            code = pdf_extract_prc_internal(ctx, decoded_stream, decoded_size,
                buff_out, size_out, views, number_views, recursive_depth);
            prc_free(ctx, decoded_stream);
            if (code == 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE,
                    "Embedded child PDF extraction succeeded (stream scan path)\n");
                return 1;
            }
            continue;
        }

        prc_free(ctx, decoded_stream);
    }

    return 0;
}

static int
pdf_name_has_prc_extension(const uint8_t *name_start, const uint8_t *name_end)
{
    uint32_t len;

    if (name_end < name_start)
    {
        return 0;
    }

    len = (uint32_t)(name_end - name_start);
    if (len >= 4)
    {
        if ((pdf_char_to_lower(name_end[-4]) == '.') &&
            (pdf_char_to_lower(name_end[-3]) == 'p') &&
            (pdf_char_to_lower(name_end[-2]) == 'r') &&
            (pdf_char_to_lower(name_end[-1]) == 'c'))
        {
            return 1;
        }
        if ((pdf_char_to_lower(name_end[-4]) == '.') &&
            (pdf_char_to_lower(name_end[-3]) == 'u') &&
            (pdf_char_to_lower(name_end[-2]) == '3') &&
            (pdf_char_to_lower(name_end[-1]) == 'd'))
        {
            return 1;
        }
    }
    return 0;
}

static int
prc_pdf_name_tree_find_prc(prc_context *ctx, uint8_t *ptr_in, uint8_t *ptr_end,
    uint8_t *file_start, uint32_t file_size, prc_pdf_head_xref *head_xref,
    prc_pdf_uncompressed_object_stream_list *stream_list,
    uint32_t *obj_num, uint32_t *gen_num)
{
    int code;
    int found_start = 0;
    uint32_t first_obj_num = 0;
    uint32_t first_gen_num = 0;
    uint32_t ext_obj_num = 0;
    uint32_t ext_gen_num = 0;

    *obj_num = 0;
    *gen_num = 0;

    /* First lets look for [ */
    while (ptr_in < ptr_end)
    {
        if (*ptr_in == '[')
        {
            ptr_in++;
            found_start = 1;
            break;
        }
        ptr_in++;
    }
    if (!found_start)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find name tree array in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    /* Now read the entries */
    while (ptr_in < ptr_end)
    {
        /* Look for the string entry */
        if (*ptr_in == '(')
        {
            uint8_t *str_start = ptr_in + 1;
            uint8_t *str_end = str_start;
            uint8_t *ptr_filespec;
            uint8_t subtype_value[PDF_MAX_DICT_VALUE];
            uint32_t subtype_value_len = 0;
            uint32_t filespec_obj_num = 0;
            uint32_t filespec_gen_num = 0;
            uint32_t filespec_size = 0;
            uint8_t is_object_stream_item = 0;
            int has_prc_extension;
            /* Find the end of the string */
            while (str_end < ptr_end && *str_end != ')')
            {
                str_end++;
            }
            if (str_end >= ptr_end)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not find end of string in PDF file\n");
                return PRC_ERROR_PARSE;
            }

            has_prc_extension = pdf_name_has_prc_extension(str_start, str_end);

            ptr_in = str_end + 1;
            code = sscanf((const char *)ptr_in, "%u %u R", &filespec_obj_num, &filespec_gen_num);
            if (code == 2 && filespec_obj_num != 0)
            {
                if (first_obj_num == 0)
                {
                    first_obj_num = filespec_obj_num;
                    first_gen_num = filespec_gen_num;
                }

                ptr_filespec = prc_pdf_get_ptr_to_obj(ctx, file_start, file_size,
                    head_xref, stream_list, filespec_obj_num, &filespec_size,
                    &is_object_stream_item);
                if (ptr_filespec != NULL)
                {
                    code = prc_pdf_dict_get_type(ctx, ptr_filespec,
                        ptr_filespec + filespec_size, PDF_SUBTYPE_NAME,
                        subtype_value, PDF_MAX_DICT_VALUE, &subtype_value_len);
                    if (code > 0 &&
                        (pdf_buffer_contains_ascii_case_insensitive(subtype_value,
                            subtype_value_len, "prc") ||
                         pdf_buffer_contains_ascii_case_insensitive(subtype_value,
                            subtype_value_len, "u3d")))
                    {
                        *obj_num = filespec_obj_num;
                        *gen_num = filespec_gen_num;
                        return 1;
                    }
                }

                if (has_prc_extension && ext_obj_num == 0)
                {
                    ext_obj_num = filespec_obj_num;
                    ext_gen_num = filespec_gen_num;
                }
            }
        }
        else
        {
            ptr_in++;
        }
    }

    if (ext_obj_num != 0)
    {
        *obj_num = ext_obj_num;
        *gen_num = ext_gen_num;
        return 1;
    }

    if (first_obj_num != 0)
    {
        *obj_num = first_obj_num;
        *gen_num = first_gen_num;
        return 1;
    }

    return 0;
}

/* Look for Annotations that is subtype RichMedia. This is for PDF 2.0 style  */
static int
pdf_object_is_rich_media(prc_context *ctx, prc_pdf_decrypt_params *decrypt_params,
    prc_pdf_uncompressed_object_stream_list *stream_list, prc_pdf_head_xref *head_xref,
    uint8_t *pdf_buff_in, uint32_t size_in, uint32_t byte_offset, uint8_t *file_start,
    uint32_t file_size, uint32_t *prc_offset, uint32_t *rich_media_view_obj_num,
    uint32_t *rich_media_view_gen_num)
{
    uint8_t *ptr = pdf_buff_in + byte_offset;
    int code;
    uint8_t found_obj = 0;
    uint8_t found_dict_start = 0;
    uint8_t keyout[PDF_MAX_DICT_VALUE];
    uint32_t actual_keyout_len;
    uint32_t object_num_media;
    uint32_t gen_num_media;
    uint32_t object_num_media_settings;
    uint32_t gen_num_media_settings;
    uint8_t has_settings = 0;
    uint8_t *ptr_rich_media_assets;
    uint8_t *ptr_rich_media_names;
    uint8_t *ptr_rich_media_settings;
    uint8_t *ptr_activation_settings;
    uint8_t *dict_end;
    uint32_t size_out;
    uint8_t is_object_stream_item;
    uint32_t prc_file_spec_obj_num;
    uint32_t prc_file_spec_gen_num;
    uint32_t prc_file_obj_num;
    uint32_t prc_file_gen_num;

    *prc_offset = 0;
    *rich_media_view_obj_num = 0;
    *rich_media_view_gen_num = 0;

    /* Check if this is a dictionary */
    while (ptr < pdf_buff_in + size_in)
    {
        /* Read until we get past obj */
        if (strncmp((const char *)ptr, PDF_OBJECT_NAME, PDF_OBJECT_NAME_LEN) == 0)
        {
            found_obj = 1;
            break;
        }
        ptr++;
    }
    if (!found_obj)
    {
        ptr = pdf_buff_in + byte_offset;
        while (ptr + 1 < pdf_buff_in + size_in)
        {
            if (*ptr == '<' && *(ptr + 1) == '<')
            {
                found_dict_start = 1;
                break;
            }
            ptr++;
        }
        if (!found_dict_start)
        {
            return 0;
        }
    }
    else
    {
        ptr += PDF_OBJECT_NAME_LEN;

        /* Eat any white space */
        code = pdf_eat_white_space(ctx, &ptr, pdf_buff_in + size_in);
        if (code < 0)
        {
            return code;
        }
        /* Is it a dictionary */
        if (strncmp((const char *)ptr, "<<", 2) != 0)
        {
            return 0;
        }
    }

    /* Yes. Lets look for a Annot Type */
    code = prc_pdf_dict_get_type(ctx, ptr, pdf_buff_in + size_in,
        PDF_TYPE_NAME, keyout, PDF_MAX_DICT_VALUE, &actual_keyout_len);
    if (code <= 0)
    {
        return code;
    }
    if (strncmp((const char *)keyout, PDF_TYPE_ANNOT, actual_keyout_len) != 0)
    {
        return 0;
    }

    /* Now look for the Subtype */
    code = prc_pdf_dict_get_type(ctx, ptr, pdf_buff_in + size_in,
        PDF_SUBTYPE_NAME, keyout, PDF_MAX_DICT_VALUE, &actual_keyout_len);
    if (code <= 0)
    {
        return code;
    }
    if (strncmp((const char *)keyout, PDF_RICHMEDIA_NAME, actual_keyout_len) != 0)
    {
        return 0;
    }

    /* OK we have a RichMedia Annotation. It has to have content */
    code = prc_pdf_dict_get_ref(ctx, ptr, pdf_buff_in + size_in,
        PDF_RICHMEDIACONTENT_NAME, &object_num_media,
        &gen_num_media);
    if (code <= 0)
    {
        return code;
    }

    /* If might not have a settings entry. That is optional */
    code = prc_pdf_dict_get_ref(ctx, ptr, pdf_buff_in + size_in,
        PDF_RICHMEDIASETTINGS_NAME, &object_num_media_settings,
        &gen_num_media_settings);
    if (code < 0)
    {
        return code;
    }
    has_settings = code;

    /* Get the dictionary at the rich media content. That should have a list
       of embedded file names, one of which could be a prc file */
    ptr_rich_media_assets = prc_pdf_get_ptr_to_obj(ctx, file_start, file_size,
        head_xref, stream_list, object_num_media, &size_out, &is_object_stream_item);
    if (ptr_rich_media_assets == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find rich media assets object in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    /* I believe this should be a dictionary of Assets which has a dictionary
       of names. Lets get the dictionary of names */
    code = prc_pdf_dict_get_dict(ctx, ptr_rich_media_assets,
        ptr_rich_media_assets + size_out, PDF_ASSETS_NAME,
        &ptr_rich_media_names, &dict_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find rich media assets dictionary in PDF file\n");
        return code;
    }

    /* Now we need to get to the array that contains the name tree. This has
       a string name e.g. (myfile.prc) and value (obj num, gen num) pairs.
       We want to look for a name that has a .prc file name suffix */
    code = prc_pdf_name_tree_find_prc(ctx, ptr_rich_media_names, dict_end,
                                      file_start, file_size, head_xref,
                                      stream_list, &prc_file_spec_obj_num,
                                      &prc_file_spec_gen_num);
    if (code < 0 || prc_file_spec_obj_num == 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find prc file spec in rich media assets name tree\n");
        return code;
    }

    /* Get us the pointer to prc_file_spec_obj_num */
    ptr = prc_pdf_get_ptr_to_obj(ctx, file_start, file_size,
        head_xref, stream_list, prc_file_spec_obj_num, &size_out,
        &is_object_stream_item);
    if (ptr == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find prc file spec object in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    /* Get a pointer to the /EF dictionary */
    code = prc_pdf_dict_get_dict(ctx, ptr, ptr + size_out, PDF_EF_NAME, &ptr, &dict_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find EF dictionary in PDF file\n");
        return code;
    }

    /* Now get the obj number and gen number associated with the /F entry */
    code = prc_pdf_dict_get_ref(ctx, ptr, dict_end, PDF_F_NAME, &prc_file_obj_num,
                                &prc_file_gen_num);
    if (code < 0 || prc_file_obj_num == 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find F entry in EF dictionary in PDF file\n");
        return code;
    }

    /* Finally we are at the actually PRC stream data. */
    ptr = prc_pdf_get_ptr_to_obj(ctx, file_start, file_size,
        head_xref, stream_list, prc_file_obj_num, &size_out,
        &is_object_stream_item);
    if (ptr == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find prc file spec object in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    *prc_offset = ptr - file_start;

    if (has_settings)
    {
        ptr_rich_media_settings = prc_pdf_get_ptr_to_obj(ctx, file_start, file_size,
            head_xref, stream_list, object_num_media_settings, &size_out, &is_object_stream_item);
        if (ptr_rich_media_settings == NULL)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find rich media settings object in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        /* Get the /Activation dictionary within this dictionary */
        code = prc_pdf_dict_get_dict(ctx, ptr_rich_media_settings,
            ptr_rich_media_settings + size_out, PDF_ACTIVATION_NAME,
            &ptr_activation_settings, &dict_end);

        /* Now look for the \View object num gen num in this dictionary */
        code = prc_pdf_dict_get_ref(ctx, ptr_activation_settings,
            ptr_activation_settings + size_out, PDF_VIEW_NAME,
            rich_media_view_obj_num, rich_media_view_gen_num);
        if (code < 0)
        {
            rich_media_view_obj_num = 0;
            rich_media_view_gen_num = 0;
        }
    }

    return 0;
 }

static int
pdf_object_is_prc(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
    uint32_t byte_offset)
{
    uint8_t *ptr = pdf_buff_in + byte_offset;
    int code;
    uint8_t keyout[PDF_MAX_DICT_VALUE];
    uint32_t actual_keyout_len;

    code = prc_pdf_dict_get_type(ctx, ptr, pdf_buff_in + size_in,
        PDF_SUBTYPE_NAME, keyout, PDF_PRC_NAME_LEN, &actual_keyout_len);

    if (code > 0 && strncmp((const char*) keyout, PDF_PRC_NAME, actual_keyout_len) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static int
pdf_find_prc_stream_by_used_stream_scan(prc_context *ctx, uint8_t *pdf_buff_in,
    uint32_t size_in, prc_pdf_head_xref *head_xref, uint8_t streams_encrypted,
    prc_pdf_decrypt_params *decrypt_params, uint32_t *prc_offset)
{
    uint32_t j;
    uint8_t *file_end = pdf_buff_in + size_in;

    *prc_offset = 0;

    if (head_xref == NULL || head_xref->xref_objects == NULL)
    {
        return 0;
    }

    for (j = 0; j < head_xref->num_objects; j++)
    {
        uint8_t *obj_ptr;
        uint8_t *ptr_stream;
        uint32_t stream_length;
        uint32_t stream_obj_num;
        uint32_t stream_gen_num;
        uint8_t *decoded_stream = NULL;
        uint32_t decoded_size = 0;
        int code;

        if (head_xref->xref_objects[j].type != PRC_PDF_XREF_USED_TYPE)
        {
            continue;
        }

        obj_ptr = pdf_buff_in + head_xref->xref_objects[j].byte_offset;
        code = pdf_get_stream_info(ctx, obj_ptr, file_end, &ptr_stream,
            &stream_length, &stream_obj_num, &stream_gen_num);
        if (code < 0)
        {
            continue;
        }

        if (stream_length >= PDF_PRC_STREAM_HEADER_LEN &&
            strncmp((const char *)ptr_stream, PDF_PRC_STREAM_HEADER,
                PDF_PRC_STREAM_HEADER_LEN) == 0)
        {
            *prc_offset = head_xref->xref_objects[j].byte_offset;
            return 1;
        }

        if (streams_encrypted)
        {
            size_t decrypted_size = pdf_decrypt_get_size(ctx, decrypt_params, stream_length);
            uint8_t *decrypted_data;
            uint32_t actual_decrypted_size;

            if (decrypted_size == 0)
            {
                continue;
            }

            decrypted_data = (uint8_t *)prc_calloc(ctx, decrypted_size, sizeof(uint8_t));
            if (decrypted_data == NULL)
            {
                continue;
            }

            code = pdf_get_decrypted_stream_data(ctx, ptr_stream, stream_length,
                decrypt_params, decrypted_data, decrypted_size,
                stream_obj_num, stream_gen_num, &actual_decrypted_size);
            if (code < 0)
            {
                prc_free(ctx, decrypted_data);
                continue;
            }

            code = pdf_get_stream_data(ctx, obj_ptr, file_end, decrypted_data,
                actual_decrypted_size, &decoded_stream, &decoded_size);
            prc_free(ctx, decrypted_data);
            if (code < 0)
            {
                continue;
            }
        }
        else
        {
            code = pdf_get_stream_data(ctx, obj_ptr, file_end, ptr_stream,
                stream_length, &decoded_stream, &decoded_size);
            if (code < 0)
            {
                continue;
            }
        }

        if (decoded_stream != NULL)
        {
            if (decoded_size >= PDF_PRC_STREAM_HEADER_LEN &&
                strncmp((const char *)decoded_stream, PDF_PRC_STREAM_HEADER,
                    PDF_PRC_STREAM_HEADER_LEN) == 0)
            {
                prc_free(ctx, decoded_stream);
                *prc_offset = head_xref->xref_objects[j].byte_offset;
                return 1;
            }
            prc_free(ctx, decoded_stream);
        }
    }

    return 0;
}

static int32_t
pdf_search_xref_subsection(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
    uint32_t num_objects, uint8_t **ptr)
{
    uint32_t k;
    uint32_t object_byte_offset;
    uint32_t object_generation_num;
    int code;

    /* Now we have to step through the objects in the xref table */
    for (k = 0; k < num_objects; k++)
    {
        if (strncmp((const char *)ptr, PDF_TRAILER_NAME, PDF_TRAILER_NAME_LEN) == 0)
        {
            break;
        }
        code = sscanf((const char*) *ptr, "%u %u", &object_byte_offset, &object_generation_num);
        if (code != 2)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read object byte offset in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        /* Now we have to read the object */
        code = pdf_object_is_prc(ctx, pdf_buff_in, size_in, object_byte_offset);
        if (code == 1)
        {
            /* Found the PRC object */
            return object_byte_offset;
        }
        *ptr += PDF_XREF_ENTRY_LENGTH;
    }
    return 0;
}

int
pdf_get_tag_value_int(prc_context *ctx, uint8_t *ptr, uint8_t *boundary,
    uint8_t *tag_name, uint32_t tag_name_len, uint32_t *tag_value)
{
    int code;

    while (ptr < boundary)
    {
        if (strncmp((const char *)ptr, (const char *)tag_name, tag_name_len) == 0)
        {
            ptr += tag_name_len;
            pdf_eat_white_space(ctx, &ptr, boundary);
            code = sscanf((const char*) ptr, "%u", tag_value);
            if (code != 1)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read tag value in PDF file\n");
                return PRC_ERROR_PARSE;
            }
            return 1;
        }
        ptr++;
    }
    return 0;
}

static int
pdf_get_view(prc_context *ctx, prc_pdf_3dview *view, uint32_t view_ref_num,
    uint32_t view_gen_num, uint8_t *pdf_buff_in, prc_pdf_xref *xref_table,
    uint32_t num_xref_objects, uint8_t *boundary)
{
    uint32_t k;
    uint32_t byte_offset;
    uint8_t found;
    uint32_t code;

    if (xref_table == NULL)
    {
        /* Brute force search */

    }
    else
    {
        /* Search through the xref table for the ref and gen number */
        byte_offset = 0;
        for (k = 0; k < num_xref_objects; k++)
        {
            if (xref_table[k].object_number == view_ref_num &&
                xref_table[k].generation_number == view_gen_num)
            {
                byte_offset = xref_table[k].byte_offset;
                break;
            }
        }

        if (byte_offset == 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find view in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        /* Make sure the Type is a 3DView */
        code = pdf_search_for_tag(ctx, pdf_buff_in + byte_offset, boundary,
            (uint8_t *) PDF_3DVIEW_NAME, PDF_3DVIEW_NAME_LEN, (uint8_t *) PDF_ENDOBJ_NAME,
            PDF_ENDOBJ_NAME_LEN, &found);
        if (!found)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find 3DView in PDF file\n");
            return PRC_ERROR_PARSE;
        }

        /* Now get the view data TODO CONTINUE HERE WITH PDF PARSING */

    }

    return 0;
}

int
pdf_check_for_dict_int_entry(prc_context *ctx, uint8_t *ptr_in, uint8_t *boundary,
    uint8_t *tag_name, uint32_t tag_name_len, int *value_out)
{
    uint8_t *ptr = ptr_in;
    int code;
    int ret_count;

    if (strncmp((const char *)ptr, (const char *)tag_name, tag_name_len) == 0)
    {
        ptr += tag_name_len;
        code = pdf_eat_white_space(ctx, &ptr, boundary);
        if (code < 0)
        {
            return code;
        }
        ret_count = sscanf((const char *)ptr, "%d", value_out);
        if (ret_count != 1)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not read dict entry in PDF file\n");
            return PRC_ERROR_PARSE;
        }
        /* Move the pointer to a / or a > */
        while (*ptr != '/' && *ptr != '>')
        {
            ptr++;
            if (ptr >= boundary)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not find / or > in PDF file\n");
                return PRC_ERROR_PARSE;
            }
        }
        /* Return the ptr offset */
        return ptr - ptr_in;
    }
    return 0;
}

int
pdf_get_integer_prc(prc_context *ctx, uint8_t *ptr_in, uint8_t *file_end,
    uint8_t *tag_name, uint32_t tag_length, uint8_t *tag_end_name,
    uint32_t tag_end_length, uint32_t *value)
{
    int32_t count;
    uint8_t *ptr = ptr_in;
    int code;
    uint32_t k;
    uint8_t found;

    count = pdf_search_for_tag(ctx, ptr_in, file_end, tag_name, tag_length,
        tag_end_name, tag_end_length, &found);
    if (!found)
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not find tag in PDF\n");
        return PRC_ERROR_PARSE;
    }
    ptr += count + tag_length;

    /* Read the integer */
    code = sscanf((const char *)ptr, "%u", value);
    if (code != 1)
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not read integer in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    return 0;
}

/* If num_elements_in is zero then we read until we get to ] or max_elements */
int
pdf_get_integer_array_prc(prc_context *ctx, uint8_t *ptr_in, uint8_t *file_end,
    uint8_t *tag_name, uint32_t tag_length, uint8_t *tag_end_name,
    uint32_t tag_end_length, int32_t *elements, uint32_t *num_elements_in,
    uint32_t max_elements)
{
    int32_t count;
    uint8_t *ptr = ptr_in;
    int code;
    uint32_t k;
    uint8_t found;
    uint32_t num_elements = *num_elements_in;

    count = pdf_search_for_tag(ctx, ptr_in, file_end, tag_name, tag_length,
        tag_end_name, tag_end_length, &found);
    if (!found)
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not find tag in PDF\n");
        return PRC_ERROR_PARSE;
    }
    ptr += count + tag_length;
    code = pdf_eat_white_space(ctx, &ptr, file_end);
    if (code < 0)
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
        return code;
    }
    /* Next character should be [ */
    if (*ptr != '[')
    {
        //prc_error(ctx, PRC_ERROR_PARSE, "Did not find [ in PDF\n");
        return PRC_ERROR_PARSE;
    }
    ptr++; /* Move past [ */

    if (num_elements == 0)
    {
        /* Read until ] or max_elements */
        num_elements = 0;
        while (*ptr != ']')
        {
            if (num_elements >= max_elements)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Too many elements in PDF array\n");
                return PRC_ERROR_PARSE;
            }
            code = sscanf((const char *)ptr, "%d", &elements[num_elements]);
            if (code != 1)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read integer in PDF file\n");
                return PRC_ERROR_PARSE;
            }
            num_elements++;
            code = pdf_eat_white_space(ctx, &ptr, file_end);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
                return code;
            }
            /* Move ptr to next integer or ] */
            while (*ptr != ' ' && *ptr != ']')
            {
                ptr++;
                if (ptr >= file_end)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not read ] in PDF file\n");
                    return PRC_ERROR_PARSE;
                }
            }
        }
        *num_elements_in = num_elements;
    }
    else
    {
        /* Read num_elements into elements */
        for (k = 0; k < num_elements; k++)
        {
            code = sscanf((const char *)ptr, "%d", &elements[k]);
            if (code != 1)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not read integer in PDF file\n");
                return PRC_ERROR_PARSE;
            }

            code = pdf_eat_white_space(ctx, &ptr, file_end);
            if (code < 0)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
                return code;
            }

            /* Move ptr to next integer or ] */
            while (*ptr != ' ' && *ptr != ']')
            {
                ptr++;
                if (ptr >= file_end)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not read ] in PDF file\n");
                    return PRC_ERROR_PARSE;
                }
            }
        }
    }
    return 0;
}

int
pdf_is_encrypted(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
    uint32_t *encrypted_object_num)
{
    uint8_t *ptr = pdf_buff_in + size_in - 1;
    uint8_t *file_end = ptr;
    uint32_t count;
    uint8_t found;
    int code;

    *encrypted_object_num = 0;
    while (ptr > (pdf_buff_in + PDF_TRAILER_NAME_LEN - 1))
    {
        /* Look for startxref */
        if (strncmp((const char *)(ptr - PDF_TRAILER_NAME_LEN + 1), PDF_TRAILER_NAME,
            PDF_TRAILER_NAME_LEN) == 0)
        {
            ptr++;
            break;
        }
        ptr--;
        if (ptr == pdf_buff_in)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find trailer in PDF file\n");
            return PRC_ERROR_PARSE;
        }
    }

    /* Look for /Encrypt before we hit startxref */
    count = pdf_search_for_tag(ctx, ptr, file_end, (uint8_t *) PDF_ENCRYPT_NAME, PDF_ENCRYPT_NAME_LEN,
                               (uint8_t *) PDF_STARTXREF_NAME, PDF_STARTXREF_NAME_LEN, &found);
    if (!found)
    {
        return 0; /* Not encrypted */
    }
    ptr += count + PDF_ENCRYPT_NAME_LEN;

    code = sscanf((const char*) ptr, "%u", encrypted_object_num);
    if (code != 1)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read encrypted object number in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    /* Is encrypted */
    return 1;
}

static int
pdf_read_hex_string(prc_context *ctx, uint8_t *ptr_in, uint8_t *file_end, uint8_t *hexstring,
    uint32_t max_value_len, uint32_t *value_len_out)
{
    uint8_t *ptr = ptr_in;
    int code;
    uint32_t value_len = 0;
    uint32_t k = 0;
    uint8_t evennibble = 1;

    if (*ptr != '<')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find < at start of hex string in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    ptr++; /* Move past < */

    while (ptr < file_end && k < max_value_len)
    {
        if (ptr[0] == '>')
            break;
        if ((ptr[0] >= '0' && ptr[0] <= '9') ||
            (ptr[0] >= 'A' && ptr[0] <= 'F') ||
            (ptr[0] >= 'a' && ptr[0] <= 'f'))
        {
            if (evennibble)
            {
                hexstring[k] = 0;
                if (ptr[0] >= '0' && ptr[0] <= '9')
                    hexstring[k] = (ptr[0] - '0') << 4;
                else if (ptr[0] >= 'A' && ptr[0] <= 'F')
                    hexstring[k] = (ptr[0] - 'A' + 10) << 4;
                else
                    hexstring[k] = (ptr[0] - 'a' + 10) << 4;
                evennibble = 0;
            }
            else
            {
                if (ptr[0] >= '0' && ptr[0] <= '9')
                    hexstring[k] |= (ptr[0] - '0');
                else if (ptr[0] >= 'A' && ptr[0] <= 'F')
                    hexstring[k] |= (ptr[0] - 'A' + 10);
                else
                    hexstring[k] |= (ptr[0] - 'a' + 10);
                evennibble = 1;
                k++;
            }
        }
        ptr++;
    }

    *value_len_out = k;

    return 0;
}

/* In this case we are at the trailer object and we are going to see if /ID
   is present in the dictionary.  If it is then we can deal with the [ < > < > ]
   delimiters */
int
pdf_get_file_id_from_object(prc_context *ctx, uint8_t *ptr_in, uint8_t *file_end,
    uint8_t *file_id, uint32_t *file_id_length)
{
    int code;

    while (ptr_in < file_end)
    {
        if (strncmp((const char *)ptr_in, PDF_ID_NAME, PDF_ID_NAME_LEN) == 0)
        {
            ptr_in += PDF_ID_NAME_LEN;
            break;
        }

        /* If we see an endobj before we see the ID then there is no ID */
        if (strncmp((const char *)ptr_in, PDF_ENDOBJ_NAME, PDF_ENDOBJ_NAME_LEN) == 0)
                {
                    return 0; /* No ID found */
        }
        ptr_in++;
    }
    if (ptr_in >= file_end)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find ID in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    /* The ID is an array of two strings. We want the first one. It should be a hex
       string. We will read up to 16 bytes of this string as the ID. */
       /* The two strings are delimited by  [ < hexstring1> <hexstring2> ]  Just get
       * the first hexstring as they should be the same */
    code = pdf_eat_white_space(ctx, &ptr_in, file_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
        return code;
    }
    if (*ptr_in != '[')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find [ in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    ptr_in++; /* Move past [ */
    code = pdf_eat_white_space(ctx, &ptr_in, file_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
        return code;
    }

    /* Use the unencyprted version of read hexstring */
    code = pdf_read_hex_string(ctx, ptr_in, file_end, file_id, PDF_MAX_DICT_VALUE, file_id_length);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read ID hex string in PDF file\n");
        return code;
    }

    /* Has ID */
    return 1;
}

int
pdf_get_file_id(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
    uint8_t *file_id, uint32_t *file_id_length)
{
    uint8_t *ptr = pdf_buff_in + size_in - 1;
    uint8_t *file_end = ptr;
    uint32_t count;
    uint8_t found;
    int code;

    *file_id_length = 0;

    while (ptr > (pdf_buff_in + PDF_TRAILER_NAME_LEN - 1))
    {
        /* Look for trailer */
        if (strncmp((const char *)(ptr - PDF_TRAILER_NAME_LEN + 1), PDF_TRAILER_NAME,
            PDF_TRAILER_NAME_LEN) == 0)
        {
            ptr++;
            break;
        }
        ptr--;

        /* If we see an endobj before the trailer then there is no trailer */
        if (strncmp((const char *)(ptr - PDF_ENDOBJ_NAME_LEN + 1), PDF_ENDOBJ_NAME,
            PDF_ENDOBJ_NAME_LEN) == 0)
        {
            return 0; /* No ID found */
        }

        if (ptr == pdf_buff_in)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find trailer in PDF file\n");
            return PRC_ERROR_PARSE;
        }
    }

    /* Look for /ID before we hit startxref */
    count = pdf_search_for_tag(ctx, ptr, file_end, (uint8_t *) PDF_ID_NAME, PDF_ID_NAME_LEN,
                               (uint8_t *) PDF_STARTXREF_NAME, PDF_STARTXREF_NAME_LEN, &found);
    if (!found)
    {
        return 0; /* No ID found */
    }
    ptr += count + PDF_ID_NAME_LEN;

    /* The ID is an array of two strings. We want the first one. It should be a hex
       string. We will read up to 16 bytes of this string as the ID. */
    /* The two strings are delimited by  [ < hexstring1> <hexstring2> ]  Just get
       hexstring1 as they should be the same */
    code = pdf_eat_white_space(ctx, &ptr, file_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
        return code;
    }
    if (*ptr != '[')
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find [ in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    ptr++; /* Move past [ */
    code = pdf_eat_white_space(ctx, &ptr, file_end);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not eat white space in PDF file\n");
        return code;
    }

    /* Use the unencyprted version of read hexstring */
    code = pdf_read_hex_string(ctx, ptr, file_end, file_id, PDF_MAX_DICT_VALUE, file_id_length);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not read ID hex string in PDF file\n");
        return code;
    }

    /* Has ID */
    return 1;
}

/* Find the PRC object in the PDF stream. We have to step through the PDF objects
   looking for the PRC tag information. This is NOT robust at this time. TODO
   make this more robust.  I don't handle subsections in the xref table or deal
   with free objects properly */
int
pdf_extract_prc_internal(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
                uint8_t **buff_out, uint32_t *size_out, prc_pdf_view_array **views,
                uint32_t *number_views, uint32_t recursive_depth)
{
    /* Get us to the trailer in the PDF file */
    uint8_t *ptr = pdf_buff_in;
    uint32_t xref_offset = 0;
    int code;
    int8_t found_prc = 0;
    uint32_t prc_offset = 0;
    uint8_t *ptr_stream;
    uint32_t stream_length;
    uint8_t xref_encoded = 0;
    uint8_t xref_predictor_present = 0;
    uint8_t *file_end = pdf_buff_in + size_in;
    uint8_t *xref_start = NULL;
    prc_pdf_view_array *cam_views = NULL;
    prc_pdf_head_xref head_xref = { 0 };
    uint32_t num_content_streams = 0;
    prc_pdf_uncompressed_object_stream_list stream_list = { 0 };
    uint32_t k, j;
    uint32_t total_xref_objects = 0;
    uint32_t length;
    uint8_t streams_encrypted = 0;
    prc_pdf_decrypt_params decrypt_params = { 0 };
    size_t decypted_size;
    uint8_t *decrypted_data = NULL;
    uint32_t stream_obj_num;
    uint32_t stream_gen_num;
    uint8_t is_richmedia = 0;
    uint32_t rich_media_view_gen_num = 0;
    uint32_t rich_media_view_obj_num = 0;
    uint8_t is_object_stream_item;
    uint32_t num_xrefs;
    uint32_t *xref_offsets = NULL;
    uint32_t xref_stm_offset = 0;
    uint8_t xref_stm_already_listed = 0;

    /* Get to the startxref line */
    ptr = pdf_buff_in + size_in - 1;
    while (ptr > (pdf_buff_in + PDF_STARTXREF_NAME_LEN - 1))
    {
        /* Look for startxref */
        if (strncmp((const char *)(ptr - PDF_STARTXREF_NAME_LEN + 1), PDF_STARTXREF_NAME,
                             PDF_STARTXREF_NAME_LEN) == 0)
        {
            ptr++;
            break;
        }
        ptr--;
        if (ptr == pdf_buff_in)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find startxref in PDF file\n");
            return PRC_ERROR_PARSE;
        }
    }

    code = pdf_eat_white_space(ctx, &ptr, file_end);
    if (code < 0)
    {
        return code;
    }

    /* pdf_buff_in is a raw byte buffer with no guaranteed NUL terminator, so
       scanning it directly with sscanf risks reading past the allocation if
       the numeric run isn't followed by a non-digit before EOF (the
       startxref value is frequently the last thing in the file). Copy a
       bounded, NUL-terminated window into a stack buffer first instead --
       more than enough for any real xref byte offset. */
    {
        char numbuf[32];
        size_t avail = (size_t)(file_end - ptr);
        size_t n = avail < sizeof(numbuf) - 1 ? avail : sizeof(numbuf) - 1;
        memcpy(numbuf, ptr, n);
        numbuf[n] = '\0';
        code = sscanf(numbuf, "%u", &xref_offset);
    }
    if (code != 1)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find xref offset in PDF file\n");
        return PRC_ERROR_PARSE;
    }
    if (xref_offset <=0 || xref_offset >= size_in)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not find xref offset in PDF file\n");
        return PRC_ERROR_PARSE;
    }

    /* First lets see how many xrefs there are by starting at the first one and
       going through the prev entries */
    num_xrefs = 1;
    code = pdf_count_xref_sections(ctx, pdf_buff_in, xref_offset,
        size_in, &num_xrefs, &xref_offsets);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not count xref sections in PDF file\n");
        goto fail;
    }

    code = pdf_get_file_id(ctx, pdf_buff_in, size_in, decrypt_params.file_id,
                           &decrypt_params.file_id_length);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not get file ID in PDF file\n");
        goto fail;
    }

    for (k = 0; k < num_xrefs; k++)
    {
        code = pdf_parse_xref(ctx, pdf_buff_in, size_in, &head_xref,
            &stream_list, &xref_offsets[k], &streams_encrypted, &decrypt_params);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not parse xref in PDF file\n");
            goto fail;
        }

        /* Hybrid-reference PDFs can store supplemental xref info in /XRefStm.
           Parse that stream too when present and not already in our xref list. */
        ptr = pdf_buff_in + xref_offsets[k];
        code = pdf_get_integer_prc(ctx, ptr, file_end, (uint8_t*)PDF_XREFSTM_NAME,
            PDF_XREFSTM_NAME_LEN, (uint8_t *)PDF_STREAM_NAME, PDF_STREAM_NAME_LEN,
            &xref_stm_offset);
        if (code >= 0 && xref_stm_offset > 0 && xref_stm_offset < size_in)
        {
            xref_stm_already_listed = 0;
            for (j = 0; j < num_xrefs; j++)
            {
                if (xref_offsets[j] == xref_stm_offset)
                {
                    xref_stm_already_listed = 1;
                    break;
                }
            }

            if (!xref_stm_already_listed)
            {
                uint32_t xref_stm_offset_local = xref_stm_offset;
                code = pdf_parse_xref(ctx, pdf_buff_in, size_in, &head_xref,
                    &stream_list, &xref_stm_offset_local, &streams_encrypted,
                    &decrypt_params);
                if (code < 0)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "Did not parse XRefStm in PDF file\n");
                    goto fail;
                }
            }
        }
        xref_start = pdf_buff_in + xref_offset;

        /* Keep accumulating xref sections first. We can still search for PRC while
           doing so, but do not stop parsing older sections if already found. */
        if (!found_prc)
        {
            /* We have to search through the xref table for the PRC object. It is
               going to be in an inuse section. It can by deflated and/or encrypted */
            for (j = 0; j < head_xref.num_objects; j++)
            {
                if (head_xref.xref_objects[j].type == PRC_PDF_XREF_USED_TYPE)
                {
                    ptr = pdf_buff_in + head_xref.xref_objects[j].byte_offset;
                    length = file_end - ptr;
                    code = pdf_object_is_prc(ctx, ptr, length, 0);
                    if (code == 1)
                    {
                        /* Found the PRC object */
                        prc_offset = head_xref.xref_objects[j].byte_offset;
                        found_prc = 1;
                        break;
                    }
                }
            }

            if (prc_offset == 0)
            {
                /* Did not find it. However we could be in a PDF 2.0 document which
                   can encode this as an RichMedia annotation type. Why would they do
                   this?  On top of that, I see files that claim to be 1.4 version with
                   2.0 data types... */
                /* We have to search through the xref table for anno object. It is
                   going to be in an inuse section. It can by deflated and/or encrypted.
                   This needs a little more work. */
                for (j = 0; j < head_xref.num_objects; j++)
                {
                    if (head_xref.xref_objects[j].type == PRC_PDF_XREF_USED_TYPE)
                    {
                        uint8_t *ptr_stream_local;
                        uint32_t stream_length_local;
                        uint32_t stream_obj_num_local;
                        uint32_t stream_gen_num_local;
                        uint8_t *decoded_richmedia = NULL;
                        uint32_t decoded_richmedia_size = 0;

                        ptr = pdf_buff_in + head_xref.xref_objects[j].byte_offset;
                        length = file_end - ptr;
                        code = pdf_object_is_rich_media(ctx, &decrypt_params, &stream_list,
                            &head_xref, ptr, length, 0, pdf_buff_in,
                            size_in, &prc_offset,
                            &rich_media_view_obj_num,
                            &rich_media_view_gen_num);
                        if (prc_offset != 0)
                        {
                            /* Found the PRC object as a rich media object */
                            is_richmedia = 1;
                            found_prc = 1;
                            break;
                        }

                        code = pdf_get_stream_info(ctx, ptr, file_end, &ptr_stream_local,
                            &stream_length_local, &stream_obj_num_local, &stream_gen_num_local);
                        if (code < 0)
                        {
                            continue;
                        }

                        if (streams_encrypted)
                        {
                            size_t decrypted_size_local = pdf_decrypt_get_size(ctx,
                                &decrypt_params, stream_length_local);
                            uint8_t *decrypted_local;
                            uint32_t actual_decrypted_size_local;

                            if (decrypted_size_local == 0)
                            {
                                continue;
                            }

                            decrypted_local = (uint8_t *)prc_calloc(ctx,
                                decrypted_size_local, sizeof(uint8_t));
                            if (decrypted_local == NULL)
                            {
                                continue;
                            }

                            code = pdf_get_decrypted_stream_data(ctx, ptr_stream_local,
                                stream_length_local, &decrypt_params, decrypted_local,
                                decrypted_size_local, stream_obj_num_local,
                                stream_gen_num_local, &actual_decrypted_size_local);
                            if (code < 0)
                            {
                                prc_free(ctx, decrypted_local);
                                continue;
                            }

                            code = pdf_get_stream_data(ctx, ptr, file_end, decrypted_local,
                                actual_decrypted_size_local, &decoded_richmedia,
                                &decoded_richmedia_size);
                            prc_free(ctx, decrypted_local);
                            if (code < 0)
                            {
                                continue;
                            }
                        }
                        else
                        {
                            code = pdf_get_stream_data(ctx, ptr, file_end,
                                ptr_stream_local, stream_length_local,
                                &decoded_richmedia, &decoded_richmedia_size);
                            if (code < 0)
                            {
                                continue;
                            }
                        }

                        if (decoded_richmedia != NULL)
                        {
                            code = pdf_object_is_rich_media(ctx, &decrypt_params,
                                &stream_list, &head_xref, decoded_richmedia,
                                decoded_richmedia_size, 0, pdf_buff_in, size_in,
                                &prc_offset, &rich_media_view_obj_num,
                                &rich_media_view_gen_num);
                            prc_free(ctx, decoded_richmedia);

                            if (prc_offset != 0)
                            {
                                is_richmedia = 1;
                                found_prc = 1;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if (!found_prc)
    {
        /* Fallback for odd files: scan all used stream objects for a raw PRC header. */
        code = pdf_find_prc_stream_by_used_stream_scan(ctx, pdf_buff_in, size_in,
            &head_xref, streams_encrypted, &decrypt_params, &prc_offset);
        if (code == 1)
        {
            found_prc = 1;
        }
        else
        {
            if (recursive_depth < 1)
            {
                code = pdf_try_extract_from_embedded_pdfs(ctx, pdf_buff_in, size_in,
                    &head_xref, &stream_list, streams_encrypted,
                    &decrypt_params, buff_out, size_out, views, number_views,
                    recursive_depth + 1);
                if (code == 1)
                {
                    goto success;
                }
            }
            prc_error(ctx, PRC_ERROR_PARSE, "Did not find PRC object in PDF file\n");
            code = PRC_ERROR_PARSE;
            goto fail;
        }
    }

    /* Now we have the offset to the PRC object, we need to get the buffer from
       stream to endstream */
    ptr = pdf_buff_in + prc_offset;
    code = pdf_get_stream_info(ctx, ptr, file_end, &ptr_stream, &stream_length,
                    &stream_obj_num, &stream_gen_num);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Did not get stream info in PDF file\n");
        goto fail;
    }

    /* This is an interesting one. I have seen cases there the PRC stream
       is encrypted and cases where it is not. Why bother encrypting the
       thing as already crazy. In any event look for the PRC header in the
       stream code. If present, assume it is not encrypted even if the
       streams are supposedly encrypted. */
    if (stream_length > PDF_PRC_STREAM_HEADER_LEN &&
        strncmp((const char *)ptr_stream, PDF_PRC_STREAM_HEADER, PDF_PRC_STREAM_HEADER_LEN) == 0)
    {
        streams_encrypted = 0;
    }

    /* First check if the stream is encrypted */
    if (streams_encrypted)
    {
        uint32_t actual_decrypted_size;
        decypted_size = pdf_decrypt_get_size(ctx, &decrypt_params, stream_length);
        decrypted_data = (uint8_t *)prc_calloc(ctx, decypted_size, sizeof(uint8_t));
        if (decrypted_data == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate decrypted data\n");
            code = PRC_ERROR_MEMORY;
            goto fail;
        }

        code = pdf_get_decrypted_stream_data(ctx, ptr_stream,
                                        stream_length, &decrypt_params,
                                        decrypted_data, decypted_size,
                                        stream_obj_num, stream_gen_num,
                                        &actual_decrypted_size);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not get decrypted stream data in PDF file\n");
            prc_free(ctx, decrypted_data);
            goto fail;
        }

        /* Now we need to deflate the decrypted stream */
        code = pdf_get_stream_data(ctx, ptr, file_end, decrypted_data, actual_decrypted_size,
                                   buff_out, size_out);
        prc_free(ctx, decrypted_data);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not get PRC stream data (encrypted) in PDF file\n");
            goto fail;
        }
    }
    else
    {
        code = pdf_get_stream_data(ctx, ptr, file_end, ptr_stream, stream_length,
                                   buff_out, size_out);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Did not get PRC stream data in PDF file\n");
            goto fail;
        }
    }

    if (is_richmedia && rich_media_view_obj_num != 0)
    {
        /* First set ptr to rich_media_view_obj_num */
        ptr = prc_pdf_get_ptr_to_obj(ctx, pdf_buff_in, size_in,
            &head_xref, &stream_list, rich_media_view_obj_num,
            &stream_length, &is_object_stream_item);

        if (ptr != NULL)
        {
            /* One view for now, until I get a PDF 2.0 file that has multiple
               views */
            cam_views = (prc_pdf_view_array *)prc_calloc(ctx, 1, sizeof(prc_pdf_view_array));
            if (cam_views == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Did not allocate views\n");
                code = PRC_ERROR_MEMORY;
                goto fail;
            }

            /* And now parse the camera view object */
            code = pdf_parse_view_prc(ctx, &decrypt_params, rich_media_view_obj_num,
                rich_media_view_gen_num, ptr, pdf_buff_in + size_in, &cam_views[0]);
            if (code < 0)
            {
                prc_error(ctx, code, "Did not parse rich media view\n");
                goto fail;
            }
            *number_views = 1;
        }
    }
    else
    {
        /* This ptr is at the main 3D data or the View data of the Richmedia.
           Lets see if we can get any of the view information */
        code = pdf_get_view_array_prc(ctx, &decrypt_params, &head_xref,
                                      &stream_list, pdf_buff_in, size_in, ptr,
                                      number_views, &cam_views);
        if (code < 0)
        {
            prc_error(ctx, code, "Did not get view array\n");
            goto fail;
        }
    }

    if (*number_views > 0)
    {
        *views = cam_views;
    }
    else
    {
        *views = NULL;
    }

success:
    pdf_cleanup_extract_state(ctx, &head_xref, &stream_list, &xref_offsets);

    return 0;

fail:
    if (cam_views != NULL)
    {
        if (is_richmedia)
        {
            pdf_free_view_array_partial(ctx, cam_views, 1);
        }
        else if (number_views != NULL && *number_views > 0)
        {
            pdf_free_view_array_partial(ctx, cam_views, *number_views);
        }
        else
        {
            prc_free(ctx, cam_views);
        }
    }

    if (*buff_out != NULL)
    {
        prc_free(ctx, *buff_out);
        *buff_out = NULL;
    }
    pdf_cleanup_extract_state(ctx, &head_xref, &stream_list, &xref_offsets);

    return code;
}

int
pdf_extract_prc(prc_context *ctx, uint8_t *pdf_buff_in, uint32_t size_in,
                uint8_t **buff_out, uint32_t *size_out, prc_pdf_view_array **views,
                uint32_t *number_views)
{
    return pdf_extract_prc_internal(ctx, pdf_buff_in, size_in, buff_out,
        size_out, views, number_views, 0);
}
