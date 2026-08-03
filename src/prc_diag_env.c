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

#include "prc_diag_env.h"

#if PRC_ENABLE_DIAG_ENV

#include <stdlib.h>
#include <string.h>

/* Linear-scan cache, not a hash table: there are on the order of 80 distinct
   diagnostic names in this codebase, so a strcmp scan per (still cached
   after first hit) lookup is negligible next to the getenv() syscall it
   replaces. Fixed-size and non-growing on purpose -- this is debug-only
   tooling; silently falling back to an uncached getenv() past the cap
   (rather than erroring) keeps a bug here from ever becoming a hard
   failure in a diagnostic path. */
#define PRC_DIAG_ENV_CACHE_MAX 128

typedef struct
{
    const char *name;
    const char *value;
} prc_diag_env_entry;

static prc_diag_env_entry prc_diag_env_cache[PRC_DIAG_ENV_CACHE_MAX];
static int prc_diag_env_cache_count = 0;

const char *
prc_diag_getenv(const char *name)
{
    int i;

    for (i = 0; i < prc_diag_env_cache_count; i++)
    {
        if (strcmp(prc_diag_env_cache[i].name, name) == 0)
            return prc_diag_env_cache[i].value;
    }

    {
        const char *value = getenv(name);
        if (prc_diag_env_cache_count < PRC_DIAG_ENV_CACHE_MAX)
        {
            prc_diag_env_cache[prc_diag_env_cache_count].name = name;
            prc_diag_env_cache[prc_diag_env_cache_count].value = value;
            prc_diag_env_cache_count++;
        }
        return value;
    }
}

#endif /* PRC_ENABLE_DIAG_ENV */
