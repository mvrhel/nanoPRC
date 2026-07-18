#!/usr/bin/env python3
"""Sanity check for nanoprc_py debug environment on Windows.

This script verifies that Python is importing the local Debug extension build
and reports common causes when it is not.
"""

from __future__ import annotations

import os
import platform
import sys
from pathlib import Path


def _norm(path: str) -> str:
    return str(Path(path).resolve()).replace("\\", "/").lower()


def main() -> int:
    print("[nanoprc-py debug env check]")
    print(f"python_executable: {sys.executable}")
    print(f"python_version: {sys.version.split()[0]}")
    print(f"platform: {platform.platform()}")

    if os.name != "nt":
        print("WARN: This script currently focuses on Windows debug-path checks.")

    repo_root = Path(__file__).resolve().parents[2]
    expected_prefix = _norm(str(repo_root / "python" / "build-debug" / "Debug"))

    print(f"repo_root: {repo_root}")
    print(f"expected_debug_prefix: {expected_prefix}")

    try:
        import nanoprc_py  # type: ignore
        import nanoprc_py._core as core  # type: ignore
    except Exception as exc:
        print(f"FAIL: import nanoprc_py failed: {exc}")
        return 2

    pkg_file = Path(nanoprc_py.__file__).resolve()
    core_file = Path(core.__file__).resolve()

    print(f"nanoprc_py_file: {pkg_file}")
    print(f"core_file: {core_file}")

    core_norm = _norm(str(core_file))
    is_debug_core = core_norm.startswith(expected_prefix + "/")

    if is_debug_core:
        print("PASS: Local Debug _core extension is loaded.")
        return 0

    print("FAIL: _core is not loaded from python/build-debug/Debug.")
    print("Suggested checks:")
    print("  1. Build Debug extension: cmake --build python/build-debug --config Debug")
    print("  2. Verify debug solution/project settings in Visual Studio (_core startup project)")
    print("  3. Re-run: python -m pip install -e python")
    print("  4. Re-check with this script")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
