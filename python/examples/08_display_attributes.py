#!/usr/bin/env python3
"""Print model-tree attributes using nanoprc_py bindings.

Usage:
    python python/examples/08_display_attributes.py path/to/file.prc
"""

from __future__ import annotations

import sys

import nanoprc_py


def _format_entry(entry: dict) -> str:
    title = entry.get("entry_title") or "(untitled)"
    value = entry.get("value")
    type_id = entry.get("type")
    return f"{title} = {value!r} (type={type_id})"


def _print_attribute_bases(label: str, bases: list[dict], indent: str) -> None:
    if not bases:
        return

    print(f"{indent}{label}:")
    for base in bases:
        base_title = base.get("base_title") or "(no base title)"
        print(f"{indent}  - base: {base_title}")
        for entry in base.get("entries", []):
            print(f"{indent}      {_format_entry(entry)}")


def _walk(node: nanoprc_py.ModelNode, depth: int = 0) -> None:
    indent = "  " * depth
    name = node.name if node.name else "(unnamed)"
    print(f"{indent}- node: {name} [type={node.node_type}]")

    _print_attribute_bases("node attributes", node.attributes(), indent + "  ")

    if node.has_part:
        part_name = node.part_name if node.part_name else "(unnamed part)"
        same_name = "yes" if node.part_name_same_as_product else "no"
        print(f"{indent}  part: {part_name} (name_same_as_product={same_name})")
        _print_attribute_bases("part attributes", node.part_attributes(), indent + "  ")

    for child in node.children():
        _walk(child, depth + 1)


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    if len(argv) != 1:
        print("Usage: python python/examples/08_display_attributes.py path/to/file.prc")
        return 1

    infile = argv[0]
    ctx = nanoprc_py.Context()
    doc = ctx.open(infile)

    doc.prepare_model_tree()
    root = doc.create_model_tree()

    print(f"Attributes for: {infile}")
    _walk(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
