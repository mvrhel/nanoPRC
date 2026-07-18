from pathlib import Path
import os
import sys
import importlib.machinery
import importlib.util

_DLL_DIRECTORY_HANDLES = []


def _try_import_local_debug_extension() -> None:
    fullname = __name__ + "._core"
    pkg_dir = Path(__file__).resolve().parent
    repo_root = pkg_dir.parents[2]
    candidates = [
        repo_root / "python" / "build-debug" / "Debug",
        repo_root / "python" / "build" / "Debug",
    ]

    for base in candidates:
        if not base.is_dir():
            continue

        for suffix in importlib.machinery.EXTENSION_SUFFIXES:
            pyd_path = base / ("_core" + suffix)
            if not pyd_path.is_file():
                continue

            loader = importlib.machinery.ExtensionFileLoader(fullname, str(pyd_path))
            spec = importlib.util.spec_from_file_location(fullname, str(pyd_path),
                loader=loader)
            if spec is None or spec.loader is None:
                continue

            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            sys.modules[fullname] = module
            return


def _prefer_local_debug_extension() -> None:
    pkg_dir = Path(__file__).resolve().parent
    repo_root = pkg_dir.parents[2]
    candidates = [
        repo_root / "python" / "build-debug" / "Debug",
        repo_root / "python" / "build" / "Debug",
        repo_root / "build-debug" / "Debug",
        repo_root / "build" / "Debug",
    ]

    for path in candidates:
        if path.is_dir() and any(path.glob("_core*.pyd")):
            str_path = str(path)
            if str_path not in __path__:
                __path__.insert(0, str_path)
            break


def _add_windows_dll_search_paths() -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return

    pkg_dir = Path(__file__).resolve().parent
    repo_root = pkg_dir.parents[2]
    candidates = [
        pkg_dir,
        repo_root / "build-debug" / "bin" / "Debug",
        repo_root / "build-debug" / "Debug",
        repo_root / "build" / "bin" / "Release",
        repo_root / "build" / "bin" / "Debug",
        repo_root / "build" / "Release",
        repo_root / "build" / "Debug",
    ]

    for path in candidates:
        if path.is_dir():
            handle = os.add_dll_directory(str(path))
            if handle is not None:
                _DLL_DIRECTORY_HANDLES.append(handle)


_prefer_local_debug_extension()
_add_windows_dll_search_paths()
_try_import_local_debug_extension()


from . import _core as _core_mod

Context = _core_mod.Context
Document = _core_mod.Document
ModelNode = _core_mod.ModelNode
View = _core_mod.View

PRC_API_ERROR_MEMORY = _core_mod.PRC_API_ERROR_MEMORY
PRC_API_ERROR_PARAMETER = _core_mod.PRC_API_ERROR_PARAMETER
PRC_API_ERROR_PARSER = _core_mod.PRC_API_ERROR_PARSER
PRC_API_ERROR_UNSUPPORTED = _core_mod.PRC_API_ERROR_UNSUPPORTED

# Keep import resilient if an older debug _core build is still on disk.
PRC_API_WRITE_RI_SURFACE = getattr(_core_mod, "PRC_API_WRITE_RI_SURFACE", 0)
PRC_API_WRITE_RI_WIRE = getattr(_core_mod, "PRC_API_WRITE_RI_WIRE", 1)
PRC_API_WRITE_TESS_KIND_TRIANGLES = getattr(_core_mod, "PRC_API_WRITE_TESS_KIND_TRIANGLES", 0)
PRC_API_WRITE_TESS_KIND_WIRE = getattr(_core_mod, "PRC_API_WRITE_TESS_KIND_WIRE", 1)
PRC_API_WRITE_TESS_KIND_COMPRESSED = getattr(_core_mod, "PRC_API_WRITE_TESS_KIND_COMPRESSED", 2)

__all__ = [
    "Context",
    "Document",
    "ModelNode",
    "View",
    "PRC_API_ERROR_MEMORY",
    "PRC_API_ERROR_PARAMETER",
    "PRC_API_ERROR_PARSER",
    "PRC_API_ERROR_UNSUPPORTED",
    "PRC_API_WRITE_RI_SURFACE",
    "PRC_API_WRITE_RI_WIRE",
    "PRC_API_WRITE_TESS_KIND_TRIANGLES",
    "PRC_API_WRITE_TESS_KIND_WIRE",
    "PRC_API_WRITE_TESS_KIND_COMPRESSED",
]
