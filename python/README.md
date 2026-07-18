# nanoprc-py (scaffold)

This directory contains an initial Python binding scaffold for nanoPRC using
`pybind11` + `scikit-build-core`.

## Current binding surface

- `nanoprc_py.Context`
  - Owns a `prc_context*` via RAII.
  - `open(path)` returns a `nanoprc_py.Document`.
  - `print_error_stack()` forwards to `prc_api_print_error_stack`.
- `nanoprc_py.Document`
  - Owns a `prc_api_data` handle and releases it automatically.
  - `is_open` property reports whether handle is valid.
  - `number_of_views`, `get_view(view_index)` for camera/view metadata.
  - `prepare_model_tree()`, `create_model_tree()`, `tessellation_counts()` for model-tree and tessellation counts.
  - `number_of_faces()`, `face_vertex_count()`, `face_vertices()` for face-level vertex access.
- `nanoprc_py.ModelNode`
  - Exposes model tree nodes and children.
- `nanoprc_py.View`
  - Exposes view name, transform matrix, and camera distance.

This is intentionally minimal but now includes initial model-tree and tessellation inspection support.

## Build prerequisites

1. Build the main nanoPRC library first:

```bash
cmake -S . -B build
cmake --build build --config Release
```

2. Install Python build dependencies:

```bash
python -m pip install --upgrade pip
python -m pip install scikit-build-core pybind11
```

If CMake cannot find `pybind11`, print its CMake package location and pass it explicitly:

```bash
python -c "import pybind11, os; print(os.path.join(os.path.dirname(pybind11.__file__), 'share', 'cmake', 'pybind11'))"
```

Then add `-Dpybind11_DIR=<that path>` to the CMake command.

> Windows note: prefer `python -m pip ...` instead of `pip ...` to avoid PATH/script location warnings. If you want an isolated environment, create one with:
>
> ```powershell
> py -3.13 -m venv .venv
> .\.venv\Scripts\Activate.ps1
> python -m pip install --upgrade pip
> python -m pip install scikit-build-core pybind11
> ```

## Build/install the Python package

From repo root:

```bash
python -m pip install -e python
```

If you are already inside the `python` directory, install from the current folder instead:

```bash
python -m pip install -e .
```

## Debug build

To debug native issues, build both the main nanoPRC library and the Python extension in `Debug` mode.

1. Build the main library in Debug.

```powershell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --config Debug
```

2. Build the Python extension in Debug.

```powershell
cd python
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --config Debug
```

If CMake cannot find `pybind11`, add the package path from:

```powershell
python -c "import pybind11, os; print(os.path.join(os.path.dirname(pybind11.__file__), 'share', 'cmake', 'pybind11'))"
```

then run:

```powershell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -Dpybind11_DIR="<path>"
cmake --build build-debug --config Debug
```

If the Python build cannot find the debug `nano_prc` library, pass the import library explicitly:

```powershell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -Dpybind11_DIR="<path>" -DNANOPRC_LIBRARY="../build-debug/Debug/nano_prc.lib"
cmake --build build-debug --config Debug
```

If this still fails, clear the stale cached value first by deleting `python/build-debug/CMakeCache.txt` or rerunning CMake with `-U NANOPRC_LIBRARY`:

```powershell
cmake -S . -B build-debug -U NANOPRC_LIBRARY -DCMAKE_BUILD_TYPE=Debug -Dpybind11_DIR="<path>" -DNANOPRC_LIBRARY="../build-debug/Debug/nano_prc.lib"
cmake --build build-debug --config Debug
```

3. Ensure the debug `nano_prc.dll` is on the DLL search path when importing `nanoprc_py`.

On Windows, `python/src/nanoprc_py/__init__.py` already adds debug DLL directories including `build-debug/bin/Debug`, `build-debug/Debug`, `build/bin/Debug`, and `build/Debug` to the DLL search path.

4. Verify Python is importing the local Debug extension (not a stale site-packages wheel):

```powershell
python -c "import nanoprc_py; import nanoprc_py._core as c; print('nanoprc_py:', nanoprc_py.__file__); print('_core:', c.__file__)"
```

Expected `_core` path for Debug sessions:

```text
.../python/build-debug/Debug/_core.cp<pyver>-win_amd64.pyd
```

5. Run Python with the same interpreter you used to build the extension.

## Troubleshooting

- If `pybind11` is not found, pass `-Dpybind11_DIR="<path>"` with the path printed by:

  ```powershell
  python -c "import pybind11, os; print(os.path.join(os.path.dirname(pybind11.__file__), 'share', 'cmake', 'pybind11'))"
  ```

- Always link against the import library `nano_prc.lib`, not the DLL.

- If CMake still caches the old path, delete `python/build-debug/CMakeCache.txt` before reconfiguring.

- Use the same `python` interpreter for both installing `pybind11` and building the extension.

- Quick one-command debug sanity check:

  ```powershell
  python python/tools/debug_env_check.py
  ```

  This reports `PASS` only when `_core` is loaded from `python/build-debug/Debug`.

```powershell
python python/examples/06_tessellation_summary.py examples/cylinder.pdf
```

6. Debug in Visual Studio by launching `python.exe` and using the example script as the command arguments.
Open `python/build-debug/nanoprc_py.sln`, set `_core` as the Startup Project, and set:

- Command: the same `python.exe` used to build/install
- Command Arguments: `python/examples/06_tessellation_summary.py examples/cylinder.pdf`
- Working Directory: repo root

You can keep `build/nano_prc.sln` open in a separate Visual Studio instance.

This lets you set breakpoints in `python/src/bindings.cpp` and the native nanoPRC sources such as `src/prc_api.c`.

## Quick smoke test

```python
import nanoprc_py

ctx = nanoprc_py.Context()
doc = ctx.open("examples/triangle.pdf")
print(doc.is_open)
```

## Examples

The OpenGL viewer example requires `glfw` and `PyOpenGL`:
This work is still under development

```bash
python -m pip install glfw PyOpenGL
```

From repo root, after `python -m pip install -e python`:

```bash
python python/examples/01_import_and_context.py
python python/examples/02_open_file.py path/to/model.prc
NANOPRC_SAMPLE=path/to/model.prc python python/examples/03_pytest_style_smoke.py
python python/examples/04_list_views.py path/to/model.prc
python python/examples/05_model_tree_summary.py path/to/model.prc
python python/examples/06_tessellation_summary.py path/to/model.prc
python python/examples/07_opengl_viewer.py path/to/model.prc
python python/examples/08_display_attributes.py path/to/model.prc
python python/examples/09_prc_teapot_write.py [output.prc] [output.pdf] [samples]
```

In `07_opengl_viewer.py`, press `P` to show/hide the attributes window for the currently selected tessellation (single-tess mode).

PowerShell equivalent for the third example:

```powershell
$env:NANOPRC_SAMPLE = "path/to/model.prc"
python python/examples/03_pytest_style_smoke.py
```

## Next steps

- NumPy zero/low-copy paths for vertex/index buffers are now available.
- Face, material, and text primitive accessors are now exposed for richer tessellation export.
- Add pytest fixtures with golden PRC files for regression coverage.
- Add CI validation for the Python package build and example smoke tests.
