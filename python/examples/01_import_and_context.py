"""Minimal example: import package and create a Context.

Run from repo root after installing editable package:
    python python/examples/01_import_and_context.py
"""

import nanoprc_py


def main() -> int:
    ctx = nanoprc_py.Context()
    print("Context created:", type(ctx).__name__)
    print("A known error code PRC_API_ERROR_PARSER =", nanoprc_py.PRC_API_ERROR_PARSER)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
