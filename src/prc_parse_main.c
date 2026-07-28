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
#include <memory.h>
#include <string.h>
#include "prc_parse_file_structure.h"
#include "prc_parse_common.h"
#include "prc_parse_extra_geometry.h"
#include "prc_parse_tess.h"

#include "prc_bit.h"
#include "zlib.h"
#include "debug.h"
#include "prc_vector_util.h"
#include "prc_huff.h"

#ifndef PRC_ENABLE_UNZIPPED_FUZZ
#define PRC_ENABLE_UNZIPPED_FUZZ 0
#endif

#if PRC_ENABLE_UNZIPPED_FUZZ
#ifndef PRC_FUZZ_DEFAULT_SEED
#define PRC_FUZZ_DEFAULT_SEED 0xC0FFEE01u
#endif

#ifndef PRC_FUZZ_DEFAULT_RATE_DIVISOR
#define PRC_FUZZ_DEFAULT_RATE_DIVISOR 512u
#endif

#ifndef PRC_FUZZ_DEFAULT_MAX_MUTATIONS
#define PRC_FUZZ_DEFAULT_MAX_MUTATIONS 64u
#endif

#define PRC_FUZZ_SECTION_SCHEMA_GLOBALS (1u << 0)
#define PRC_FUZZ_SECTION_TREE (1u << 1)
#define PRC_FUZZ_SECTION_TESSELLATION (1u << 2)
#define PRC_FUZZ_SECTION_GEOMETRY (1u << 3)
#define PRC_FUZZ_SECTION_EXTRA_GEOMETRY (1u << 4)
#define PRC_FUZZ_SECTION_MODEL (1u << 5)
#define PRC_FUZZ_SECTION_ALL (PRC_FUZZ_SECTION_SCHEMA_GLOBALS | PRC_FUZZ_SECTION_TREE | \
                              PRC_FUZZ_SECTION_TESSELLATION | PRC_FUZZ_SECTION_GEOMETRY | \
                              PRC_FUZZ_SECTION_EXTRA_GEOMETRY | PRC_FUZZ_SECTION_MODEL)

static uint32_t
prc_fuzz_xorshift32(uint32_t *state)
{
    if (*state == 0u)
    {
        *state = 0xA341316Cu;
    }
    *state ^= (*state << 13);
    *state ^= (*state >> 17);
    *state ^= (*state << 5);
    return *state;
}

static uint32_t
prc_fuzz_hash_string(const char *str)
{
    uint32_t hash = 2166136261u;
    if (str == NULL)
    {
        return hash;
    }

    while (*str != '\0')
    {
        hash ^= (uint8_t)(*str);
        hash *= 16777619u;
        str++;
    }
    return hash;
}

static uint32_t
prc_fuzz_mix_seed(uint32_t seed, uint32_t hash)
{
    seed ^= hash + 0x9e3779b9u + (seed << 6) + (seed >> 2);
    if (seed == 0u)
    {
        seed = PRC_FUZZ_DEFAULT_SEED;
    }
    return seed;
}

static uint32_t
prc_fuzz_read_env_u32(const char *env_name, uint32_t default_value, uint32_t min_value,
                      uint32_t max_value)
{
    const char *text = getenv(env_name);
    unsigned long parsed;
    char *endptr;

    if (text == NULL || *text == '\0')
    {
        return default_value;
    }

    parsed = strtoul(text, &endptr, 0);
    if (endptr == text || *endptr != '\0')
    {
        return default_value;
    }
    if (parsed < (unsigned long)min_value)
    {
        return min_value;
    }
    if (parsed > (unsigned long)max_value)
    {
        return max_value;
    }
    return (uint32_t)parsed;
}

static uint32_t
prc_fuzz_read_seed(void)
{
    return prc_fuzz_read_env_u32("PRC_FUZZ_SEED", PRC_FUZZ_DEFAULT_SEED, 1u, 0xffffffffu);
}

static uint32_t
prc_fuzz_section_bit_from_name(const char *name)
{
    if (name == NULL)
    {
        return 0u;
    }

    if (strcmp(name, "schema") == 0 || strcmp(name, "schema_globals") == 0 ||
        strcmp(name, "schema_globals_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_SCHEMA_GLOBALS;
    }
    if (strcmp(name, "tree") == 0 || strcmp(name, "tree_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_TREE;
    }
    if (strcmp(name, "tessellation") == 0 || strcmp(name, "tessellation_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_TESSELLATION;
    }
    if (strcmp(name, "geometry") == 0 || strcmp(name, "geometry_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_GEOMETRY;
    }
    if (strcmp(name, "extra") == 0 || strcmp(name, "extra_geometry") == 0 ||
        strcmp(name, "extra_geometry_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_EXTRA_GEOMETRY;
    }
    if (strcmp(name, "model") == 0 || strcmp(name, "model_unzipped") == 0)
    {
        return PRC_FUZZ_SECTION_MODEL;
    }
    if (strcmp(name, "all") == 0)
    {
        return PRC_FUZZ_SECTION_ALL;
    }

    return 0u;
}

static uint32_t
prc_fuzz_read_section_mask(void)
{
    uint32_t mask_override;
    const char *text = getenv("PRC_FUZZ_SECTION");
    char buffer[256];
    char *token;
    uint32_t mask = 0u;

    mask_override = prc_fuzz_read_env_u32("PRC_FUZZ_SECTION_MASK", 0u, 0u, PRC_FUZZ_SECTION_ALL);
    if (mask_override != 0u)
    {
        return mask_override;
    }

    if (text == NULL || *text == '\0')
    {
        return PRC_FUZZ_SECTION_ALL;
    }

    strncpy(buffer, text, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    token = strtok(buffer, ", ");
    while (token != NULL)
    {
        uint32_t bit = prc_fuzz_section_bit_from_name(token);
        if (bit != 0u)
        {
            if (bit == PRC_FUZZ_SECTION_ALL)
            {
                return PRC_FUZZ_SECTION_ALL;
            }
            mask |= bit;
        }
        token = strtok(NULL, ", ");
    }

    if (mask == 0u)
    {
        return PRC_FUZZ_SECTION_ALL;
    }
    return mask;
}

static int
prc_fuzz_should_mutate(uint32_t section_mask, uint32_t section_bit)
{
    return (section_mask & section_bit) != 0u;
}

static const char*
prc_fuzz_log_path(void)
{
    const char *path = getenv("PRC_FUZZ_LOG");
    if (path == NULL || *path == '\0')
    {
        return "prc_unzipped_fuzz.log";
    }
    return path;
}

static void
prc_fuzz_log_start(const char *infile, uint32_t seed, uint32_t section_mask)
{
    FILE *fid = fopen(prc_fuzz_log_path(), "ab");
    if (fid == NULL)
    {
        return;
    }
    fprintf(fid, "START seed=%u sections=0x%02X file=%s\n", seed, section_mask,
            (infile == NULL ? "(null)" : infile));
    fclose(fid);
}

static void
prc_fuzz_log_event(uint32_t file_index, const char *section_name, size_t section_size,
                   uint32_t before_state, size_t mutation_count)
{
    FILE *fid = fopen(prc_fuzz_log_path(), "ab");
    if (fid == NULL)
    {
        return;
    }

    fprintf(fid,
            "MUTATE file=%u section=%s size=%zu mutations=%zu state_before=%u\n",
            file_index,
            (section_name == NULL ? "(null)" : section_name),
            section_size,
            mutation_count,
            before_state);
    fclose(fid);
}

static size_t
prc_fuzz_mutate_buffer(uint8_t *data, size_t size, uint32_t *state, uint32_t rate_divisor,
                       uint32_t max_mutations)
{
    size_t mutation_budget;
    size_t mutation_count;
    size_t i;

    if (data == NULL || state == NULL || size == 0)
    {
        return 0;
    }
    if (rate_divisor == 0)
    {
        rate_divisor = 1;
    }
    if (max_mutations == 0)
    {
        max_mutations = 1;
    }

    mutation_budget = (size / rate_divisor) + 1;
    if (mutation_budget > (size_t)max_mutations)
    {
        mutation_budget = (size_t)max_mutations;
    }

    mutation_count = 1 + (prc_fuzz_xorshift32(state) % mutation_budget);
    for (i = 0; i < mutation_count; i++)
    {
        size_t idx = (size_t)(prc_fuzz_xorshift32(state) % (uint32_t)size);
        uint32_t op = prc_fuzz_xorshift32(state) % 3u;

        if (op == 0u)
        {
            data[idx] ^= (uint8_t)(1u << (prc_fuzz_xorshift32(state) & 7u));
        }
        else if (op == 1u)
        {
            int delta = (int)(prc_fuzz_xorshift32(state) % 17u) - 8;
            data[idx] = (uint8_t)(data[idx] + delta);
        }
        else
        {
            data[idx] = (uint8_t)(prc_fuzz_xorshift32(state) & 0xffu);
        }
    }

    return mutation_count;
}
#endif

static void
debug_prc_character_bitstream(prc_context *ctx, prc_bit_state *bit_state, uint8_t find, size_t byte_length)
{
    prc_bit_state temp;
    prc_bit_state restore;
    uint32_t val_read;
    size_t start_pos = (size_t) bit_state->ptr;
    size_t size = 0;
    size_t bits_read = 0;
    uint8_t bit;

    temp.ptr = bit_state->ptr;
    temp.bitmask = bit_state->bitmask;
    restore = temp;

    while (size < byte_length)
    {
        val_read = prc_bitread_uint8(ctx, bit_state);
        size = (size_t) (bit_state->ptr) - start_pos;
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

static prc_header*
prc_new_header(prc_context *ctx)
{
    return prc_calloc(ctx, 1, sizeof(prc_header));
}

static prc_file_structure_header*
prc_new_file_struct_header(prc_context *ctx)
{
    return prc_calloc(ctx, 1, sizeof(prc_file_structure_header));
}

static prc_file_struct_desc* prc_create_file_struct_array(prc_context *ctx, size_t num)
{
    return (prc_file_struct_desc*)prc_calloc(ctx, num, sizeof(prc_file_struct_desc));
}

static uint8_t*
prc_read_32bits_unsigned(prc_context *ctx, uint8_t* data, prc_unsigned_int *value)
{
    *value = data[0] + (((uint32_t) data[1]) << 8) +
                     (((uint32_t) data[2]) << 16) +
                     (((uint32_t) data[3]) << 24);
    return data + 4;
}

static uint8_t*
prc_read_uniqueid(prc_context *ctx, uint8_t* data, prc_unique_id* value)
{
    uint8_t* ptr = data;

    ptr = prc_read_32bits_unsigned(ctx, ptr, &value->unique_id0);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &value->unique_id1);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &value->unique_id2);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &value->unique_id3);
    return ptr;
}

int
prc_uncompress(prc_context *ctx, const unsigned char *src, size_t src_len,
               unsigned char **dst)
{
    z_stream zInfo = { 0 };
    size_t des_len = 2 * src_len;
    unsigned char *dest;

    dest = (unsigned char*)prc_malloc(ctx, des_len);
    *dst = dest;
    if (dest == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_uncompress\n");
        return PRC_ERROR_MEMORY;
    }

    zInfo.total_in = (uLong) src_len;
    zInfo.avail_in = (uInt) src_len;
    zInfo.total_out = (uLong) des_len;
    zInfo.avail_out = (uInt) des_len;
    zInfo.next_out = (Bytef*) dest;
    zInfo.next_in = (Bytef*) src;
    int err, ret = -1;
    err = inflateInit(&zInfo);
    if (err == Z_OK)
    {
        do
        {
            err = inflate(&zInfo, Z_SYNC_FLUSH);
            if (err == Z_OK) {
                unsigned char *new_dest;
                /* I need a second set of eyes to check this */
                des_len *= 2;
                new_dest = (unsigned char *)prc_realloc(ctx, dest, des_len);
                if (new_dest == NULL)
                {
                    inflateEnd(&zInfo);
                    prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_uncompress realloc\n");
                    prc_free(ctx, dest);
                    dest = NULL;
                    *dst = NULL;
                    return PRC_ERROR_MEMORY;
                }
                dest = new_dest;
                *dst = dest;
                zInfo.next_out = dest + zInfo.total_out;
                zInfo.avail_out = des_len - zInfo.total_out;
            }
            else if (err < 0)
            {
                inflateEnd(&zInfo);
                prc_free(ctx, dest);
                dest = NULL;
                *dst = dest;
                return err;
            }
        } while (err != Z_STREAM_END);
    }
    ret = inflateEnd(&zInfo);   // zlib function
    if (ret == Z_OK)
    {
        /* NUL-terminate: callers of prc_uncompress (e.g. pdf_get_stream_data,
           used for PDF object-stream decompression) hand this buffer to
           sscanf-based text parsing (pdf_parse_array_prc and friends) that
           assumes a NUL-terminated C string. total_out can legitimately
           equal the current allocation's size exactly (the buffer is only
           ever grown in response to inflate() returning Z_OK with the
           output space exhausted), leaving no guaranteed trailing byte --
           confirmed via AddressSanitizer as a real heap-buffer-overread on
           a real-world multi-object-stream PDF (DiscBrakeAssemblySolidWorks
           PRC.pdf), same bug class as the main file-read buffer's own
           missing NUL terminator (prc_parse_main.c's prc_open_contents). */
        if ((size_t)zInfo.total_out >= des_len)
        {
            unsigned char *padded = (unsigned char *)prc_realloc(ctx, dest, (size_t)zInfo.total_out + 1);
            if (padded == NULL)
            {
                prc_free(ctx, dest);
                *dst = NULL;
                prc_error(ctx, PRC_ERROR_MEMORY, "Allocation error in prc_uncompress NUL-pad\n");
                return PRC_ERROR_MEMORY;
            }
            dest = padded;
            *dst = dest;
        }
        dest[zInfo.total_out] = '\0';
        return zInfo.total_out;
    }
    return ret; // -1 or len of output
}

char desc[3];
prc_unsigned_int min_vers_for_read;
prc_unsigned_int auth_vers;
prc_unique_id unique_id_file;
prc_unique_id unique_id_application;
prc_unsigned_int file_count;
prc_uncomp_file* files;

static uint8_t*
prc_parse_uncomp_file(prc_context *ctx, uint8_t *ptr, prc_uncomp_block *file)
{
    uint32_t k;
    int code;

    ptr = prc_read_32bits_unsigned(ctx, ptr, &file->block_size);

    if (file->block_size > 0)
    {
        file->block = prc_calloc(ctx, file->block_size, sizeof(uint8_t));
        if (file->block == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate file->block\n");
            return NULL;
        }
        memcpy(file->block, ptr, file->block_size);
        ptr += file->block_size;
        /* We don't know the data type until later when we read the PRC_TYPE_GRAPH_Picture data */
    }
    return ptr;
}

prc_file_structure_header*
prc_parse_filestruct_header(prc_context *ctx, uint8_t* buff)
{
    prc_file_structure_header* header;
    uint8_t* ptr;
    uint32_t k;
    int code;

    if (buff[0] != 'P' && buff[1] != 'R' && buff[2] != 'C')
    {
        return NULL;
    }

    header = prc_new_file_struct_header(ctx);
    if (header == NULL)
    {
        return NULL;
    }

    memcpy(&header->desc, buff, 3);
    ptr = buff + 3;
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->min_vers_for_read);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->auth_vers);
    ptr = prc_read_uniqueid(ctx, ptr, &header->unique_id_file);
    ptr = prc_read_uniqueid(ctx, ptr, &header->unique_id_application);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->file_count);

    /* This is where the texture images are stored! Spec is wrong on the type of these */
    if (header->file_count > 0)
    {
        header->files = prc_calloc(ctx, header->file_count, sizeof(prc_uncomp_block));
        if (header->files == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate header->files\n");
            return NULL;
        }
        for (k = 0; k < header->file_count; k++)
        {
            ptr = prc_parse_uncomp_file(ctx, ptr, &header->files[k]);
            if (ptr == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_parse_uncomp_file\n");
                return NULL;
            }
        }
    }
    return header;
}

prc_header*
prc_parse_main_header(prc_context *ctx, uint8_t *buff)
{
    prc_header* header;
    size_t k, j;
    uint8_t* ptr;

    if (buff[0] != 'P' && buff[1] != 'R' && buff[2] != 'C')
    {
        return NULL;
    }

    header = prc_new_header(ctx);
    if (header == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate main header\n");
        return NULL;
    }

    memcpy(&header->desc, buff, 3);
    ptr = buff + 3;
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->min_vers_for_read);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->auth_vers);
    ptr = prc_read_uniqueid(ctx, ptr, &header->unique_id_file);
    ptr = prc_read_uniqueid(ctx, ptr, &header->unique_id_application);
    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->filestructure_count);

    if (header->filestructure_count > 0)
    {
        header->file_info = prc_create_file_struct_array(ctx, header->filestructure_count);
        if (header->file_info == NULL)
        {
            prc_free(ctx, header);
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate header->file_info\n");
            return NULL;
        }

        for (k = 0; k < header->filestructure_count; k++)
        {
            ptr = prc_read_uniqueid(ctx, ptr, &header->file_info[k].unique_id);
            ptr += 4;
            ptr = prc_read_32bits_unsigned(ctx, ptr, &header->file_info[k].section_count);

            if (header->file_info[k].section_count > 0)
            {
                header->file_info[k].section_offset =
                    (prc_unsigned_int*)prc_calloc(ctx, header->file_info[k].section_count,
                    sizeof(prc_unsigned_int));
                if (header->file_info[k].section_offset == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate section_offset\n");
                    prc_free(ctx, header->file_info);
                    prc_free(ctx, header);
                    return NULL;
                }
                for (j = 0; j < header->file_info[k].section_count; j++)
                {
                    ptr = prc_read_32bits_unsigned(ctx, ptr, &header->file_info[k].section_offset[j]);
                }
            }
        }
        ptr = prc_read_32bits_unsigned(ctx, ptr, &header->start_offset);
        ptr = prc_read_32bits_unsigned(ctx, ptr, &header->end_offset);
        ptr = prc_read_32bits_unsigned(ctx, ptr, &header->file_count);
        if (header->file_count > 0)
        {
            /* We are not going to worry about the multiple file case just yet */
            prc_free(ctx, header->file_info);
            prc_free(ctx, header);
            prc_error(ctx, PRC_ERROR_PARSE, "Multi-file case not supported\n");
            return NULL;
        }
    }
    return header;
}

static void
prc_dump_file(const char* filename, uint8_t* buff, size_t size)
{
    FILE* fid = fopen(filename, "wb");
    if (fid == NULL)
        return;
    fwrite(buff, 1, size, fid);
    fclose(fid);
}

prc_data*
prc_open_contents(prc_context *ctx, const char* infile)
{
    FILE* fid;
    uint8_t *buff = NULL;
    uint8_t *buff2 = NULL;
    size_t size;
    size_t size_read;
    prc_header* header;
    uint32_t k, j;
    uint8_t* ptr;
    prc_filestructure* file_struct = NULL;
    int code;
    uint8_t *ptr_raw, *ptr_raw2;
    prc_bit_state bit_state;
    uint32_t type;
    prc_data *output;
    uint32_t prc_size;
    prc_pdf_view_array *views = NULL;
    uint32_t number_views = 0;
#if PRC_ENABLE_UNZIPPED_FUZZ
    uint32_t fuzz_seed = 0;
    uint32_t fuzz_state = 0;
    uint32_t fuzz_rate_divisor = PRC_FUZZ_DEFAULT_RATE_DIVISOR;
    uint32_t fuzz_max_mutations = PRC_FUZZ_DEFAULT_MAX_MUTATIONS;
    uint32_t fuzz_section_mask = PRC_FUZZ_SECTION_ALL;
#endif

    output = (prc_data *)prc_calloc(ctx, 1, sizeof(prc_data));
    if (output == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate prc_data\n");
        return NULL;
    }

    /* Not ideal but just read in all the data for now */
    fid = fopen(infile, "rb");
    if (fid == NULL)
    {
        prc_error(ctx, PRC_ERROR_IO, "Failed to open file\n");
        return NULL;
    }

    /* ftell()/fseek() use `long`, only 32 bits on Windows even in 64-bit
       builds (unlike Linux/macOS where it's 64-bit) -- silently misbehaves
       above 2GB there. A real 2.44GB PRC/PDF input reproducibly hit this. */
#if defined(_WIN32)
    _fseeki64(fid, 0, SEEK_END);
    size = (size_t)_ftelli64(fid);
    _fseeki64(fid, 0, SEEK_SET);
#else
    fseeko(fid, 0, SEEK_END);
    size = (size_t)ftello(fid);
    fseeko(fid, 0, SEEK_SET);
#endif

    /* +1 and NUL-terminate: the PDF-parsing path (prc_pdf*.c) runs sscanf()
       directly on pointers into this buffer, which assumes a NUL-terminated
       C string -- without the extra byte, a PDF whose xref data runs to the
       end of the file with no embedded NUL causes sscanf/strnlen to read
       past the allocation. */
    buff = prc_malloc(ctx, size + 1);
    if (buff == NULL)
    {
        prc_free(ctx, output);
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate main buffer\n");
        return NULL;
    }

    size_read = fread(buff, 1, size, fid);
    fclose(fid);
    buff[size] = '\0';

    /* Determine if this is a PDF or a PRC file */
    if (buff[0] != 'P' && buff[1] != 'R' && buff[2] != 'C')
    {
        /* Not a PRC file. Check if it is a PDF file */
        if (buff[0] != 0x25 && buff[1] != 0x50 && buff[2] != 0x44 && buff[3] != 0x46)
        {
            prc_free(ctx, buff);
            prc_free(ctx, output);
            prc_error(ctx, PRC_ERROR_PARSE, "Not a PRC or PDF file\n");
            return NULL;
        }

        /* OK we need to extract the PRC stream from the PDF file */
        code = pdf_extract_prc(ctx, buff, size, &buff2, &prc_size, &views,
                               &number_views);
        if (code < 0)
        {
            prc_free(ctx, buff);
            prc_free(ctx, output);
            prc_error(ctx, code, "Failed in prc_extract_prc\n");
            return NULL;
        }
        prc_free(ctx, buff);
        buff = buff2;
        size = prc_size;

        if (views != NULL)
        {
            output->views = views;
            output->view_count = number_views;
        }
    }

    /* For debug */
    // prc_dump_file("debug.prc", buff, size);

#if PRC_ENABLE_UNZIPPED_FUZZ
    fuzz_seed = prc_fuzz_read_seed();
    fuzz_state = prc_fuzz_mix_seed(fuzz_seed, prc_fuzz_hash_string(infile));
    fuzz_rate_divisor = prc_fuzz_read_env_u32("PRC_FUZZ_RATE", PRC_FUZZ_DEFAULT_RATE_DIVISOR,
                                              1u, 1000000u);
    fuzz_max_mutations = prc_fuzz_read_env_u32("PRC_FUZZ_MAX_MUTATIONS",
                                               PRC_FUZZ_DEFAULT_MAX_MUTATIONS,
                                               1u,
                                               1000000u);
    fuzz_section_mask = prc_fuzz_read_section_mask();
    prc_fuzz_log_start(infile, fuzz_seed, fuzz_section_mask);
#endif

    /* The header */
    header = prc_parse_main_header(ctx, buff);
    if (header == NULL)
    {
        prc_free(ctx, buff);
        prc_release_data(ctx, output);
        prc_error(ctx, PRC_ERROR_PARSE, "Failed in prc_parse_main_header\n");
        return NULL;
    }

    ctx->source_file_version = header->auth_vers;

    /* Table 6 Filestructure */
    if (header->filestructure_count > 0)
    {
        file_struct = (prc_filestructure*)prc_calloc(ctx, header->filestructure_count,
                                                     sizeof(prc_filestructure));
        if (file_struct == NULL)
        {
            prc_free(ctx, buff);
            output->header = header;
            prc_release_data(ctx, output);
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate prc_filestructure\n");
            return NULL;
        }

        /* Parse the file structure details */
        for (k = 0; k < header->filestructure_count; k++)
        {
            ctx->current_file_index = k;
            if (header->file_info[k].section_count > 0)
            {
                /* First the header which is not deflated (required) */
                ptr = &buff[header->file_info[k].section_offset[0]];
                file_struct[k].header = prc_parse_filestruct_header(ctx, ptr);

                if (file_struct[k].header->min_vers_for_read > ctx->internal.reader_version)
                {
                    prc_error(ctx, PRC_FILE_VERS, "File version not supported\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }

                /* The model file is defined special */
                ptr = &buff[header->start_offset];
                code = prc_uncompress(ctx, ptr, header->end_offset - header->start_offset, &ptr_raw);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_uncompress\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }
                ptr_raw2 = (uint8_t*)prc_malloc(ctx, code);
                if (ptr_raw2 == NULL)
                {
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate ptr_raw2\n");
                    prc_free(ctx, ptr_raw);
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }

                memcpy(ptr_raw2, ptr_raw, code);
                prc_free(ctx, ptr_raw);
                file_struct[k].model_unzipped = ptr_raw2;
                file_struct[k].model_size = code;

#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_MODEL))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].model_unzipped,
                                                              file_struct[k].model_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "model_unzipped", file_struct[k].model_size,
                                       before_state, mutations);
                }
#endif

                code = prc_parse_model_file(ctx, &file_struct[k], header->filestructure_count, k);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_parse_model_file\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }

                /* All other sections are deflated but can be independently read from the offset */
                for (j = 1; j < header->file_info[k].section_count; j++)
                {
                    ptr = &buff[header->file_info[k].section_offset[j]];
                    code = prc_uncompress(ctx, ptr, size - header->file_info[k].section_offset[j], &ptr_raw);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed in prc_uncompress\n");
                        prc_free(ctx, buff);
                        output->header = header;
                        output->file_struct = file_struct;
                        output->file_structure_count = k + 1;
                        prc_release_data(ctx, output);
                        return NULL;
                    }
                    ptr_raw2 = (uint8_t*)prc_malloc(ctx, code);
                    if (ptr_raw2 == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate ptr_raw2\n");
                        prc_free(ctx, ptr_raw);
                        prc_free(ctx, buff);
                        output->header = header;
                        output->file_struct = file_struct;
                        output->file_structure_count = k + 1;
                        prc_release_data(ctx, output);
                        return NULL;
                    }

                    memcpy(ptr_raw2, ptr_raw, code);
                    prc_free(ctx, ptr_raw);

                    /* Section has been inflated. Now initialize the bit state */
                    prc_init_bit_state(ctx, &bit_state, ptr_raw2, code);

                    /* Note that schema is the first one and does NOT have a type.
                       This I feel is just one of many issues in this spec */
                    if (j == 1)
                    {
                        /* Schema case (required) */
                        file_struct[k].schema_globals_unzipped = ptr_raw2;
                        file_struct[k].schema_globals_size = code;
                    }
                    else
                    {
                        /* The items are supposed to be in the order that they
                           are given in Table 6. I have a table that has a tag
                           type encoded as 30651 Very odd
                           as 306 is PRC_TYPE_ASM_FileStructureGeometry and that
                           is the position that this section is.  Does one
                           throw an error or move on. */
                        type = prc_bitread_uint32(ctx, &bit_state);

                        switch (type)
                        {
                        case PRC_TYPE_ASM:
                        case PRC_TYPE_ASM_ProductOccurrence:
                        case PRC_TYPE_ASM_PartDefinition:
                        case PRC_TYPE_ASM_Filter:
                        case PRC_TYPE_ASM_ModelFile:
                        case PRC_TYPE_ASM_FileStructure:
                            fprintf(stderr, "Unimplemented assembly type\n");
                            break;

                        case PRC_TYPE_ASM_FileStructureTree:
                            file_struct[k].tree_unzipped = ptr_raw2;
                            file_struct[k].tree_size = code;
                            break;
                        case PRC_TYPE_ASM_FileStructureTessellation:
                            file_struct[k].tessellation_unzipped = ptr_raw2;
                            file_struct[k].tessellation_size = code;
                            break;
                        case PRC_TYPE_ASM_FileStructureGeometry:
                            file_struct[k].geometry_unzipped = ptr_raw2;
                            file_struct[k].geometry_size = code;
                            break;
                        case PRC_TYPE_ASM_FileStructureExtraGeometry:
                            file_struct[k].extra_geometry_unzipped = ptr_raw2;
                            file_struct[k].extra_geometry_size = code;
                            break;
                        case PRC_TYPE_ROOTBaseNoReference:
                            fprintf(stderr, "PRC_TYPE_ROOT_PRCBaseNoReference type\n");
                            break;
                        default:
                            fprintf(stderr, "Unknown assembly type\n");
                            break;
                        }
                    }
                }
            }

            /* Now parse the unzipped sections */
            if (file_struct[k].schema_globals_unzipped != NULL)
            {
#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_SCHEMA_GLOBALS))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].schema_globals_unzipped,
                                                              file_struct[k].schema_globals_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "schema_globals_unzipped",
                                       file_struct[k].schema_globals_size,
                                       before_state, mutations);
                }
#endif
                code = prc_parse_file_schema_and_global(ctx, &file_struct[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Error in prc_parse_file_schema_and_global\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }
            }
            if (file_struct[k].tessellation_unzipped != NULL)
            {
#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_TESSELLATION))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].tessellation_unzipped,
                                                              file_struct[k].tessellation_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "tessellation_unzipped",
                                       file_struct[k].tessellation_size,
                                       before_state, mutations);
                }
#endif
                uint8_t debug_tess = 0;
                code = prc_parse_file_tessellation(ctx, &file_struct[k], debug_tess);
                if (code < 0)
                {
                    prc_error(ctx, code, "Error in prc_parse_file_tessellation\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }
            }
            if (file_struct[k].tree_unzipped != NULL)
            {
#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_TREE))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].tree_unzipped,
                                                              file_struct[k].tree_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "tree_unzipped", file_struct[k].tree_size,
                                       before_state, mutations);
                }
#endif
                code = prc_parse_file_tree(ctx, &file_struct[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Error in prc_parse_file_tree\n");
                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }
            }
            if (file_struct[k].geometry_unzipped != NULL)
            {
#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_GEOMETRY))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].geometry_unzipped,
                                                              file_struct[k].geometry_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "geometry_unzipped", file_struct[k].geometry_size,
                                       before_state, mutations);
                }
#endif
                /* This is the exact geometry data */
                code = prc_parse_file_geometry(ctx, &file_struct[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Error in prc_parse_file_geometry\n");

                    prc_free(ctx, buff);
                    output->header = header;
                    output->file_struct = file_struct;
                    output->file_structure_count = k + 1;
                    prc_release_data(ctx, output);
                    return NULL;
                }
            }
            if (file_struct[k].extra_geometry_unzipped != NULL)
            {
#if PRC_ENABLE_UNZIPPED_FUZZ
                if (prc_fuzz_should_mutate(fuzz_section_mask, PRC_FUZZ_SECTION_EXTRA_GEOMETRY))
                {
                    uint32_t before_state = fuzz_state;
                    size_t mutations = prc_fuzz_mutate_buffer(file_struct[k].extra_geometry_unzipped,
                                                              file_struct[k].extra_geometry_size,
                                                              &fuzz_state,
                                                              fuzz_rate_divisor,
                                                              fuzz_max_mutations);
                    prc_fuzz_log_event(k, "extra_geometry_unzipped",
                                       file_struct[k].extra_geometry_size,
                                       before_state, mutations);
                }
#endif
                code = prc_parse_file_extra_geometry(ctx, &file_struct[k]);
                if (code < 0)
                {
                    prc_error(ctx, code, "Error in prc_parse_file_extra_geometry\n");
                    prc_free(ctx, buff);

                    prc_release_data(ctx, output);
                    return NULL;
                }
            }
        } /* Done with all the files */
    }

    prc_free(ctx, buff);
    output->header = header;
    output->file_struct = file_struct;
    output->file_structure_count = header->filestructure_count;

    return output;
}
