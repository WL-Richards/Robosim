# Kraken X60 Runtime Mesh

`kraken_x60_decimated.obj` is the runtime mesh derived from the canonical
Kraken X60 STEP file at `assets/cad/motors/kraken_x60/WCP-0940.STEP`.

Current mesh details:

- Source export: `kraken_x60.obj`
- Runtime mesh: `kraken_x60_decimated.obj`
- Decimated facets: 81,592
- Decimated points: 40,678
- Runtime mesh size: about 6.9 MB

Regenerate with the FreeCAD AppImage's headless command entrypoint, not the GUI
entrypoint:

```sh
/home/will/Downloads/FreeCAD_1.1.1-Linux-x86_64-py311.AppImage freecadcmd tools/cad/export_kraken_x60_mesh.py
```

The important detail is the `freecadcmd` argument after the AppImage path. If it
is omitted, the AppImage launches the FreeCAD GUI.

The source STEP is AP214 and contains original color/style records. The current
headless FreeCADCmd export keeps the geometry but does not expose those colors
through the imported objects, so this OBJ is intentionally geometry-only for now.
Keep the STEP file as the color authority and prefer a color-aware export path
when renderer mesh material support is added.
