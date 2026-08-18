# Copyright (C) 2023-2026 CascadiaVoxel LLC
#
#    nanoPRC is free software: you can redistribute it and/or modify it under
#    the terms of the GNU Affero General Public License as published by the
#    Free Software Foundation, either version 3 of the License, or (at your
#    option) any later version.
#
#    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
#    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
#    License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.
#
#    A commercial license is also available -- see README.md.
"""Pre-PR gate: check every first-party C source parses as strict C99.

WHY THIS EXISTS. This project pins C_STANDARD 99, but a common local build is
MSVC, whose C mode silently accepts constructs C99 removed. The motivating case:
a `static` function called before its definition is an implicit function
declaration (legal in C89, removed in C99). MSVC emitted no warning at all;
GCC and Clang rejected it, because the implied *extern* declaration conflicts
with the later `static` definition:

    error: static declaration of 'X' follows non-static declaration

That reached CI and broke the Linux build, the macOS build and python-wheels
from one missing prototype. A green MSVC build is NOT evidence of C99
conformance.

WHAT IT DOES. Parses each first-party .c file with a real Clang or GCC frontend
at -std=c99 and fails on hard errors and implicit declarations. It only parses
-- it does not link, and it is not a substitute for CI. It catches the
compile-time conformance class, which is the class MSVC hides.

PORTABILITY. Tries, in order: clang-tidy / clang / gcc / cc on PATH (the normal
case on Linux and macOS); then, on Windows, any Visual Studio installation's
bundled clang-tidy -- located via vswhere if present, else by globbing the
standard install roots for 2019 and 2022 across all editions. Nothing is
hardcoded to one VS version or edition.

usage: python tests/internal/check_c99.py [--build-dir DIR] [-v]
       exit 0 = clean, 1 = violations found, 2 = no usable compiler
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# demos/viewer is C++14 (SDL3 + Dear ImGui) and the Python binding is C++17;
# thirdparty/ and the bundled stb_image headers are not held to C99 either.
SOURCE_GLOBS = ['src/*.c', 'tests/internal/*.c', 'demos/*/src/*.c']
EXCLUDE_DIRS = ['demos/viewer', 'thirdparty', 'python']

# Mirrors the root CMakeLists' add_definitions(). prc_double.h #errors out
# without an endianness macro, so every file fails to parse without these --
# and _CRT_SECURE_NO_WARNINGS silences the MSVC-headers fopen deprecation
# noise that is irrelevant to conformance.
DEFINES = ['-DPRC_LITTLE_ENDIAN', '-DPRC_BUILD_SHARED', '-DPRC_ENABLE_DIAG_ENV=1',
           '-D_CRT_SECURE_NO_WARNINGS', '-DPRC_VERSION_STRING="0.0.0-c99check"']

# A clang/gcc diagnostic line: <path>:<line>:<col>: error: ...
# Matching on the position prefix matters -- a bare `'error:' in line` test also
# matches echoed SOURCE lines containing strings like "internal error: ...",
# which produced three phantom failures the first time this was written.
DIAG_RE = re.compile(r'^.+?:\d+:\d+:\s+(fatal error|error):\s')
IMPLICIT_RE = re.compile(r'implicit declaration of function|implicit-function-declaration')
# clang-analyzer findings (leaks etc.) are real but are not conformance; this
# gate deliberately does not police them.
IGNORE_RE = re.compile(r'\[clang-analyzer-|deprecated-declarations')

VS_GLOBS = [
    'C:/Program Files/Microsoft Visual Studio/*/*/VC/Tools/Llvm/bin/clang-tidy.exe',
    'C:/Program Files (x86)/Microsoft Visual Studio/*/*/VC/Tools/Llvm/bin/clang-tidy.exe',
]
VSWHERE = ('C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe')


def find_tool():
    """Returns (kind, path); kind is 'clang-tidy' or 'compiler'."""
    for name in ('clang-tidy',):
        p = shutil.which(name)
        if p:
            return 'clang-tidy', p
    for name in ('clang', 'gcc', 'cc'):
        p = shutil.which(name)
        if p:
            return 'compiler', p
    # Windows: find any VS install's bundled clang-tidy (2019, 2022, any edition)
    if os.path.exists(VSWHERE):
        try:
            out = subprocess.run([VSWHERE, '-all', '-products', '*', '-property',
                                  'installationPath'],
                                 capture_output=True, text=True).stdout
            for root in out.split('\n'):
                root = root.strip()
                if not root:
                    continue
                cand = os.path.join(root, 'VC', 'Tools', 'Llvm', 'bin', 'clang-tidy.exe')
                if os.path.exists(cand):
                    return 'clang-tidy', cand
        except OSError:
            pass
    for g in VS_GLOBS:
        hits = sorted(glob.glob(g))
        if hits:
            return 'clang-tidy', hits[-1]      # newest VS wins
    return None, None


def include_dirs(build_dir):
    dirs = [os.path.join(REPO, d) for d in ('include', 'src', 'thirdparty/zlib')]
    gen = os.path.join(REPO, build_dir, 'generated')
    if os.path.isdir(gen):
        dirs.append(gen)
    else:
        print('warning: %s not found -- configure CMake once so prc_version.h exists'
              % gen, file=sys.stderr)
    for root, _d, files in os.walk(os.path.join(REPO, build_dir)):
        if 'zconf.h' in files:           # generated into the build tree by zlib
            dirs.append(root)
            break
    return dirs


def sources():
    out = []
    for g in SOURCE_GLOBS:
        for f in glob.glob(os.path.join(REPO, g)):
            rel = os.path.relpath(f, REPO).replace('\\', '/')
            if not any(rel.startswith(x + '/') for x in EXCLUDE_DIRS):
                out.append(f)
    return sorted(out)


def check(kind, tool, src, incs):
    flags = (['-std=c99', '-Wimplicit-function-declaration'] + DEFINES
             + ['-I' + d for d in incs])
    if kind == 'clang-tidy':
        # clang-diagnostic-* MUST be enabled. With --checks=-* (even with
        # --allow-no-checks) clang-tidy short-circuits on "no checks enabled",
        # never parses the file, and exits 0 -- indistinguishable from success.
        cmd = [tool, '--checks=clang-diagnostic-*', '--quiet', src, '--'] + flags
    else:
        cmd = [tool, '-fsyntax-only'] + flags + [src]
    r = subprocess.run(cmd, capture_output=True, text=True)
    text = (r.stdout or '') + (r.stderr or '')
    return [ln.strip() for ln in text.splitlines()
            if not IGNORE_RE.search(ln)
            and (DIAG_RE.match(ln) or IMPLICIT_RE.search(ln))]


def main():
    ap = argparse.ArgumentParser(description='strict-C99 conformance gate')
    ap.add_argument('--build-dir', default='build',
                    help='CMake build dir holding generated headers (default: build)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    kind, tool = find_tool()
    if not tool:
        print('error: no clang-tidy, clang or gcc found -- cannot verify C99 '
              'conformance locally. On Windows, Visual Studio ships clang-tidy '
              'under VC/Tools/Llvm/bin (install the "C++ Clang tools" component).',
              file=sys.stderr)
        return 2

    incs = include_dirs(args.build_dir)
    srcs = sources()
    print('C99 conformance check: %d files via %s' % (len(srcs), tool))

    failures = 0
    for s in srcs:
        bad = check(kind, tool, s, incs)
        rel = os.path.relpath(s, REPO).replace('\\', '/')
        if bad:
            failures += 1
            print('\nFAIL %s' % rel)
            for ln in bad[:8]:
                print('   ' + ln)
        elif args.verbose:
            print('  ok   %s' % rel)

    print()
    if failures:
        print('%d file(s) FAILED strict C99. Fix before opening a PR -- a green '
              'MSVC build does not catch this.' % failures)
        return 1
    print('all %d first-party C sources parse clean at -std=c99' % len(srcs))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
