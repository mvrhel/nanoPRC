# nanoprc-py

Python bindings for nanoPRC, a library for reading PRC and PDF-embedded PRC 3D data.

This package provides:

- PRC/PDF open and parse APIs
- Model tree and view inspection
- Tessellation and face-level geometry access
- PRC writing and optional PDF embedding

## Install

```bash
python -m pip install nanoprc-py
```

## Quick Start

```python
import nanoprc_py

ctx = nanoprc_py.Context()
doc = ctx.open("path/to/model.prc")
print("Opened:", doc.is_open)
print("Views:", doc.number_of_views)
```

## Included Examples

The source repository includes runnable examples under `python/examples/`:

- Open files and validate parser state
- Inspect model trees and tessellation summaries
- OpenGL viewer demo
- Attribute inspection
- Teapot write example (PRC and optional PDF)

## Optional Viewer Dependencies

For the OpenGL viewer example:

```bash
python -m pip install glfw PyOpenGL
```

## Supported Python Versions

- Python 3.9+

## License

AGPL-3.0-or-later.

## Links

- Source: https://github.com/mvrhel/nanoPRC
- Issues: https://github.com/mvrhel/nanoPRC/issues

## Maintainer and Development Docs

Local build/debug instructions, release automation, and packaging details are in `python/DEVELOPMENT.md` in the repository.
