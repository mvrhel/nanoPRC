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

#ifndef PRC_DIAG_ENV_H
#define PRC_DIAG_ENV_H

/* Every PRC_DIAG_*, PRC_TRACE_*, PRC_FUZZ_* runtime diagnostic hook in this
   codebase reads its controlling environment variable through
   prc_diag_getenv() instead of calling getenv() directly. Two reasons:

   1. Performance: a handful of these checks sit in hot loops (once per
      triangle/chain during encode/decode) and were calling getenv() --
      itself a linear scan of the process environment block -- on every
      iteration, redundantly, since the answer can't change mid-run. The
      enabled implementation (prc_diag_env.c) caches each distinct name's
      result after its first lookup.
   2. Attack surface: these names are discoverable by scanning the binary
      for ASCII strings, and several accept caller-controlled VALUES (via
      atof/atoi at the call site) that feed into geometry math with no
      validation. PRC_ENABLE_DIAG_ENV (CMake option, default OFF) controls
      whether any of this exists in the build at all -- when off,
      prc_diag_getenv expands to a macro that never evaluates its `name`
      argument, so neither the getenv() call NOR the string literal itself
      is emitted into the binary. This is the production default; enable it
      only for local debugging builds.

   PRC_ENABLE_DIAG_ENV is a CMake-defined compile flag applied to both
   library targets (nano_prc, prc_static); the fallback default below
   covers any translation unit built outside that CMake configuration
   (e.g. a standalone scratchpad tool compiled directly with cl.exe/gcc),
   matching prc_context.h's PRC_DEBUG_MEMORY fallback -- safe-by-default
   (diagnostics off) unless explicitly requested. */
#ifndef PRC_ENABLE_DIAG_ENV
#define PRC_ENABLE_DIAG_ENV 0
#endif

#if PRC_ENABLE_DIAG_ENV

/* Drop-in replacement for getenv(name) at every PRC_DIAG_*, PRC_TRACE_*,
   PRC_FUZZ_* call site: same signature, same NULL-if-unset semantics,
   cached after the first lookup per distinct name. Not thread-safe (no
   locking around the cache) -- matches the risk profile of the
   single-value cache this generalizes (prc_write_bit.c's
   prc_diag_bitwrite_double_enabled_cache); diagnostics are opt-in,
   single-context debug tooling, not used concurrently in practice. */
const char *prc_diag_getenv(const char *name);

#else

#define prc_diag_getenv(name) ((const char *)0)

#endif

#endif /* PRC_DIAG_ENV_H */
