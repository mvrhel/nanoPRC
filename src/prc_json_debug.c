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

/* A debug helper file where we can read in JSON data that is what we would expect
   to be gettting in the parser when we decode the compressed stream. Requires
   build with cJSON which can be added as a third party library. */

#include "prc_context.h"
#include "prc_vector_util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if 0
#include "cJSON.h"

int
debug_prc_read_vertices_from_JSON_file(prc_context *ctx, const char *file_name,
    prc_vec3 **vertices, uint32_t num_points)
{
    char name[256];
    const cJSON *element = NULL;

    /* Open the file */
    FILE *fid = fopen(file_name, "r");
    if (fid == NULL)
    {
        prc_error(ctx, PRC_ERROR_FILE, "Failed to open file %s\n", file_name);
        return PRC_ERROR_FILE;
    }

    /* Read the file */
    fseek(fid, 0, SEEK_END);
    size_t size = ftell(fid);
    fseek(fid, 0, SEEK_SET);

    char *buff = (char *)prc_malloc(ctx, size);
    if (buff == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate buffer for file %s\n", file_name);
        fclose(fid);
        return PRC_ERROR_MEMORY;
    }

    size_t size_read = fread(buff, 1, size, fid);
    fclose(fid);

    /* Parse the JSON data */
    cJSON *json = cJSON_Parse(buff);
    if (json == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Failed to parse JSON data from file %s\n", file_name);
        prc_free(ctx, buff);
        return PRC_ERROR_PARSE;
    }

    /* Allocate the vertices */
    *vertices = (prc_vec3 *)prc_malloc(ctx, sizeof(prc_vec3) * num_points);
    if (*vertices == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate vertices\n");
        cJSON_Delete(json);
        prc_free(ctx, buff);
        return PRC_ERROR_MEMORY;
    }

    cJSON_ArrayForEach(element, json)
    {
        cJSON *x = cJSON_GetObjectItem(element, "X");
        cJSON *y = cJSON_GetObjectItem(element, "Y");
        cJSON *z = cJSON_GetObjectItem(element, "Z");
        if (x == NULL || y == NULL || z == NULL)
        {
            continue;
        }

        /* The element string includes the index for my array */
        int index = atoi(element->string + 8);
        (*vertices)[index].x = x->valuedouble;
        (*vertices)[index].y = y->valuedouble;
        (*vertices)[index].z = z->valuedouble;
    }

    /* Clean up */
    cJSON_Delete(json);
    prc_free(ctx, buff);

    return 0;
}

int
debug_prc_read_indices_from_JSON_file(prc_context *ctx, const char *file_name,
    int **indices, uint32_t num_points)
{
    char name[256];
    const cJSON *element = NULL;

    /* Open the file */
    FILE *fid = fopen(file_name, "r");
    if (fid == NULL)
    {
        prc_error(ctx, PRC_ERROR_FILE, "Failed to open file %s\n", file_name);
        return PRC_ERROR_FILE;
    }

    /* Read the file */
    fseek(fid, 0, SEEK_END);
    size_t size = ftell(fid);
    fseek(fid, 0, SEEK_SET);

    char *buff = (char *)prc_malloc(ctx, size);
    if (buff == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate buffer for file %s\n", file_name);
        fclose(fid);
        return PRC_ERROR_MEMORY;
    }

    size_t size_read = fread(buff, 1, size, fid);
    fclose(fid);

    /* Parse the JSON data */
    cJSON *json = cJSON_Parse(buff);
    if (json == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Failed to parse JSON data from file %s\n", file_name);
        prc_free(ctx, buff);
        return PRC_ERROR_PARSE;
    }

    /* Allocate the vertices */
    *indices = (int *)prc_malloc(ctx, sizeof(int) * num_points);
    if (*indices == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate indices\n");
        cJSON_Delete(json);
        prc_free(ctx, buff);
        return PRC_ERROR_MEMORY;
    }

    cJSON_ArrayForEach(element, json)
    {
        /* Compare element string to Element.  It if is different then continue */
        if (strncmp(element->string, "Element", 7) != 0)
        {
            continue;
        }

        /* The element string includes the index for my array */
        int index = atoi(element->string + 8);
        (*indices)[index] = element->valueint;
    }

    /* Clean up */
    cJSON_Delete(json);
    prc_free(ctx, buff);

    return 0;
}
#endif