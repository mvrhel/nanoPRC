# nanoprc-py Development Guide

This document is for maintainers and contributors working on local builds,
debugging, and package release automation.

## Local Editable Install

From repo root:

```bash
python -m pip install -e python
```

From inside `python/`:

```bash
python -m pip install -e .
```

## Build Prerequisites

If you need to build manually:

```bash
python -m pip install --upgrade pip
python -m pip install scikit-build-core pybind11
```

If CMake cannot find pybind11, print its package path:

```bash
python -c "import pybind11, os; print(os.path.join(os.path.dirname(pybind11.__file__), 'share', 'cmake', 'pybind11'))"
```

Then pass `-Dpybind11_DIR=<that path>` to CMake.

## Debug Build (Windows)

Build the core library:

```powershell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --config Debug
```

Build Python extension:

```powershell
cd python
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --config Debug
```

If needed, provide import library path explicitly:

```powershell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -DNANOPRC_LIBRARY="../build-debug/Debug/nano_prc.lib"
cmake --build build-debug --config Debug
```

If stale cache values remain, clear them:

```powershell
cmake -S . -B build-debug -U NANOPRC_LIBRARY
```

## Debug Sanity Check

```powershell
python python/tools/debug_env_check.py
```

Expected result: `_core` loaded from `python/build-debug/Debug`.

## Example Commands

```bash
python python/examples/01_import_and_context.py
python python/examples/02_open_file.py path/to/model.prc
python python/examples/04_list_views.py path/to/model.prc
python python/examples/05_model_tree_summary.py path/to/model.prc
python python/examples/06_tessellation_summary.py path/to/model.prc
python python/examples/07_opengl_viewer.py path/to/model.prc
python python/examples/08_display_attributes.py path/to/model.prc
python python/examples/09_prc_teapot_write.py [output.prc] [output.pdf] [samples]
```

## Python Release Workflows

- `.github/workflows/python-wheels.yaml`
  - Builds wheels for Linux, macOS, and Windows.
- `.github/workflows/python-publish.yaml`
  - Publishes to TestPyPI or PyPI via Trusted Publishing.
- `.github/workflows/python-verify-testpypi.yaml`
  - Verifies install from TestPyPI across OS/Python matrix.
- `.github/workflows/python-verify-pypi.yaml`
  - Verifies install from PyPI across OS/Python matrix.

## Trusted Publishing Setup

For TestPyPI:

- Owner: `mvrhel`
- Repository: `nanoPRC`
- Workflow: `.github/workflows/python-publish.yaml`
- Environment: `testpypi`
- Project name: `nanoprc-py`

For PyPI:

- Owner: `mvrhel`
- Repository: `nanoPRC`
- Workflow: `.github/workflows/python-publish.yaml`
- Environment: `pypi`
- Project name: `nanoprc-py`

## Release Sequence

1. Build/check wheels in CI.
2. Publish to TestPyPI.
3. Run TestPyPI verification workflow.
4. Manual smoke install from TestPyPI.
5. Publish to PyPI.
6. Run/confirm PyPI verification workflow.
