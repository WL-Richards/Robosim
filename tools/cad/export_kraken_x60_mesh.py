#!/usr/bin/env python3
"""Export the Kraken X60 STEP source to visualizer runtime OBJ meshes.

Run with FreeCAD's command-line binary, for example:

    FreeCAD_1.1.1-Linux-x86_64-py311.AppImage freecadcmd tools/cad/export_kraken_x60_mesh.py

The AP214 STEP file contains color metadata, but FreeCADCmd's headless Import
path exposes geometry only. Keep the source STEP as the color authority until
the asset pipeline has a color-aware exporter.
"""

from __future__ import annotations

import os
from pathlib import Path

import FreeCAD
import Import
import Mesh
import MeshPart


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_STEP = REPO_ROOT / "assets/cad/motors/kraken_x60/WCP-0940.STEP"
OUTPUT_DIR = REPO_ROOT / "assets/meshes/motors/kraken_x60"
FULL_OBJ = OUTPUT_DIR / "kraken_x60.obj"
RUNTIME_OBJ = OUTPUT_DIR / "kraken_x60_decimated.obj"

LINEAR_DEFLECTION = 0.04
ANGULAR_DEFLECTION = 0.45

DECIMATE_TOLERANCE = 0.005
DECIMATE_REDUCTION = 0.62


def build_full_mesh() -> Mesh.Mesh:
    doc = FreeCAD.newDocument("robosim_kraken_x60_convert")
    Import.insert(str(SOURCE_STEP), doc.Name)
    doc.recompute()

    combined = Mesh.Mesh()
    shape_count = 0
    for obj in doc.Objects:
        shape = getattr(obj, "Shape", None)
        if shape is None or shape.isNull():
            continue
        mesh = MeshPart.meshFromShape(
            Shape=shape,
            LinearDeflection=LINEAR_DEFLECTION,
            AngularDeflection=ANGULAR_DEFLECTION,
            Relative=False,
        )
        if mesh.CountFacets > 0:
            combined.addMesh(mesh)
            shape_count += 1

    if combined.CountFacets == 0:
        raise RuntimeError("no meshable shapes found")

    print(
        f"source shapes={shape_count} points={combined.CountPoints} "
        f"facets={combined.CountFacets}"
    )
    return combined


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    full_mesh = build_full_mesh()
    full_mesh.write(str(FULL_OBJ))
    print(f"wrote {FULL_OBJ}")

    runtime_mesh = Mesh.Mesh(str(FULL_OBJ))
    print(
        f"before decimation points={runtime_mesh.CountPoints} "
        f"facets={runtime_mesh.CountFacets}"
    )
    runtime_mesh.decimate(DECIMATE_TOLERANCE, DECIMATE_REDUCTION)
    runtime_mesh.harmonizeNormals()
    runtime_mesh.write(str(RUNTIME_OBJ))
    print(
        f"after decimation points={runtime_mesh.CountPoints} "
        f"facets={runtime_mesh.CountFacets}"
    )
    print(f"wrote {RUNTIME_OBJ}")


main()
