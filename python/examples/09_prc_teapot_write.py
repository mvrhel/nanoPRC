#!/usr/bin/env python3
"""Write a Utah teapot PRC file (and optional 3D PDF) using nanoprc_py.

This example reuses the classic teapot control-point table from
`demos/teapot_write/src/teapot_write.c`, evaluates the bicubic Bezier patches,
then writes PRC bytes through `Context.write_prc_buffer`.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

import nanoprc_py


def _extract_teapot_patches(c_file: Path) -> np.ndarray:
    text = c_file.read_text(encoding="utf-8")
    match = re.search(
        r"teapot_patches\s*\[[^\]]+\]\[[^\]]+\]\[3\]\s*=\s*\{(.*?)\};",
        text,
        flags=re.S,
    )
    if not match:
        raise RuntimeError("Failed to locate teapot_patches initializer in teapot_write.c")

    body = re.sub(r"/\*.*?\*/", "", match.group(1), flags=re.S)
    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", body)
    values = np.asarray([float(x) for x in nums], dtype=np.float64)

    expected = 32 * 16 * 3
    if values.size != expected:
        raise RuntimeError(f"Unexpected teapot patch value count: {values.size} (expected {expected})")

    return values.reshape((32, 16, 3))


def _bernstein(t: float) -> np.ndarray:
    mt = 1.0 - t
    return np.asarray([
        mt * mt * mt,
        3.0 * t * mt * mt,
        3.0 * t * t * mt,
        t * t * t,
    ], dtype=np.float64)


def _bernstein_deriv(t: float) -> np.ndarray:
    mt = 1.0 - t
    return np.asarray([
        -3.0 * mt * mt,
        3.0 * mt * mt - 6.0 * t * mt,
        6.0 * t * mt - 3.0 * t * t,
        3.0 * t * t,
    ], dtype=np.float64)


def _eval_patch(cp: np.ndarray, u: float, v: float) -> np.ndarray:
    bu = _bernstein(u)
    bv = _bernstein(v)
    out = np.zeros(3, dtype=np.float64)

    for i in range(4):
        for j in range(4):
            w = bu[i] * bv[j]
            out += w * cp[i * 4 + j]
    return out


def _eval_partials(cp: np.ndarray, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
    bu = _bernstein(u)
    bv = _bernstein(v)
    dbu = _bernstein_deriv(u)
    dbv = _bernstein_deriv(v)
    dsu = np.zeros(3, dtype=np.float64)
    dsv = np.zeros(3, dtype=np.float64)

    for i in range(4):
        for j in range(4):
            p = cp[i * 4 + j]
            dsu += (dbu[i] * bv[j]) * p
            dsv += (bu[i] * dbv[j]) * p

    return dsu, dsv


def _eval_normal(cp: np.ndarray, u: float, v: float) -> np.ndarray:
    uu = float(u)
    vv = float(v)

    for attempt in range(6):
        du, dv = _eval_partials(cp, uu, vv)
        n = np.cross(du, dv)
        length = float(np.linalg.norm(n))
        if length > 1e-9:
            return n / length

        shift = 0.01 * (attempt + 1)
        uu = u + (shift if u < 0.5 else -shift)
        vv = v + (shift if v < 0.5 else -shift)

    return np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def build_teapot_mesh(samples: int = 9) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if samples < 2:
        raise ValueError("samples must be >= 2")

    c_file = Path(__file__).resolve().parents[2] / "demos" / "teapot_write" / "src" / "teapot_write.c"
    patches = _extract_teapot_patches(c_file)

    positions = []
    normals = []
    tri_indices = []
    face_tri_counts = []

    for patch_idx in range(patches.shape[0]):
        cp = patches[patch_idx]
        base = len(positions)

        for i in range(samples):
            u = i / (samples - 1)
            for j in range(samples):
                v = j / (samples - 1)
                positions.append(_eval_patch(cp, u, v))
                normals.append(_eval_normal(cp, u, v))

        for i in range(samples - 1):
            for j in range(samples - 1):
                v00 = base + i * samples + j
                v01 = base + i * samples + (j + 1)
                v10 = base + (i + 1) * samples + j
                v11 = base + (i + 1) * samples + (j + 1)

                tri_indices.append((v00, v10, v11))
                tri_indices.append((v00, v11, v01))
                face_tri_counts.append(2)

    pos = np.asarray(positions, dtype=np.float64)
    nrm = np.asarray(normals, dtype=np.float64)
    tri = np.asarray(tri_indices, dtype=np.uint32)
    face_counts = np.asarray(face_tri_counts, dtype=np.uint32)

    bbox_min = pos.min(axis=0)
    bbox_max = pos.max(axis=0)

    mid = 0.5 * (bbox_min + bbox_max)
    half = 0.5 * (bbox_max - bbox_min)
    half = np.maximum(half, 1e-6)
    padded_min = mid - 1.5 * half
    padded_max = mid + 1.5 * half

    return pos, nrm, tri, face_counts, padded_min, padded_max


def _default_pdf_view(bbox_min: np.ndarray, bbox_max: np.ndarray) -> dict:
    center = 0.5 * (bbox_min + bbox_max)
    extent = bbox_max - bbox_min
    diag = float(np.linalg.norm(extent))
    if diag < 1e-6:
        diag = 1.0

    return {
        "name": "Default",
        "eye": [center[0] + diag * 1.0, center[1] - diag * 1.3, center[2] + diag * 0.7],
        "target": center.tolist(),
        "up": [0.0, 0.0, 1.0],
        "is_default": True,
    }


def write_teapot_prc(output_path: Path, samples: int = 9, output_pdf_path: Path | None = None) -> None:
    positions, normals, tri_indices, face_tri_counts, bbox_min, bbox_max = build_teapot_mesh(samples)

    tess = {
        "kind": nanoprc_py.PRC_API_WRITE_TESS_KIND_COMPRESSED,
        "positions": positions,
        "normals": normals,
        "tri_indices": tri_indices,
        "norm_indices": tri_indices,
        "face_tri_counts": face_tri_counts,
    }

    rep_item = {
        "kind": nanoprc_py.PRC_API_WRITE_RI_SURFACE,
        "biased_tessellation_index": 1,
        "is_closed": False,
    }

    part_node = {
        "name": "teapot_body",
        "part_name": "patch_faces",
        "rep_items": [rep_item],
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
    }

    root = {
        "name": "teapot",
        "children": [part_node],
    }

    ctx = nanoprc_py.Context()
    prc_bytes = ctx.write_prc_buffer("nanoPRC", root, [tess])
    output_path.write_bytes(prc_bytes)

    if output_pdf_path is not None:
        pdf_options = {
            "views": [_default_pdf_view(bbox_min, bbox_max)],
        }
        try:
            ctx.pdf_embed_prc(str(output_pdf_path), prc_bytes, pdf_options)
        except RuntimeError as exc:
            raise RuntimeError(
                f"Failed to write PDF '{output_pdf_path}'. Close Adobe/Acrobat if it has the file open, then retry."
            ) from exc


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    output = Path(argv[0]) if len(argv) >= 1 else Path("utah_teapot_python.prc")
    output_pdf = None
    samples = 9

    if len(argv) >= 2:
        try:
            samples = int(argv[1])
        except ValueError:
            output_pdf = Path(argv[1])

    if len(argv) >= 3:
        samples = int(argv[2])

    write_teapot_prc(output, samples=samples, output_pdf_path=output_pdf)
    print(f"Wrote {output}")
    if output_pdf is not None:
        print(f"Wrote {output_pdf}")
    print(f"Patch sampling: {samples}x{samples}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
