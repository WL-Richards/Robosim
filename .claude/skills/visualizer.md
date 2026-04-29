---
name: visualizer
description: Use when working on the 3D visualizer at `src/viz/` — the standalone GUI for inspecting and editing robot descriptions, and (post-v0) animating live sim runs and scrubbing recorded WPILOGs. Covers scope, public surface (CLI flags, panels, hotkeys, modes), the tech stack, the determinism exception, the scene-snapshot seam, and known limits.
---

# Visualizer

The visualizer is the 3D GUI for robosim. It is a single binary at
`src/viz/`, built only when `ROBOSIM_BUILD_VIZ=ON`, with three modes
that share one renderer / scene graph / camera / selection model:

- **Edit** — load a `descriptions/<robot>.json`, render the kinematic
  tree, click-pick entities, manipulate origins via Unreal-style gizmo
  handles, save back to JSON. **v0 ships Edit mode only.**
- **Live** — subscribe to a running sim and animate the rendered robot
  from the state stream (WPILOG / NetworkTables). Force vectors,
  contact visualization, game-piece flow live here. **Post-v0.**
- **Replay** — load a recorded WPILOG and scrub through it in 3D,
  complementing AdvantageScope's 2D view of the same log. **Post-v0.**

Binding decisions live in `docs/ARCHITECTURE.md` "Visualizer". The
phased plan is `docs/VISUALIZER_V0_PLAN.md`. This skill is the working
knowledge for implementers; the architecture doc is the contract.

## Scope

**In (v0 — Edit mode):**
- Load via `description::load_from_file`.
- Render the kinematic tree using primitive shapes (link = cylinder
  along its `length_m`, joint axis drawn as an arrow).
- Orbit camera (rotate / pan / zoom around a pivot, frame-fit hotkey).
- Inspect every entity from the scene tree (left panel) and an
  inspector panel (right) showing schema fields read-only.
- Pick entities by clicking in the 3D viewport.
- Manipulate the selected link's visual origin and the selected
  joint's origin with Unreal-style ImGuizmo handles (translate /
  rotate, world / local toggle, W / E hotkeys).
- Save back to JSON via the `description::save_to_file` serializer
  (Phase VC4).

**Out of v0 (deferred, tracked but not blocking):**
- **Live and Replay modes.** The architectural seam (the renderer reads
  from `scene_snapshot`, not from `robot_description` directly) is
  preserved; no state-stream consumer or WPILOG reader in v0.
- **CAD / mesh import** (STEP, IGES, glTF, OBJ, STL). Edit-mode
  roadmap item; primitive shapes only for v0.
- **Authoring from scratch** — creating new links / joints / motors via
  the GUI. v0 edits an existing file.
- **Vendor-binding UI** — assigning CAN IDs, picking firmware versions
  through the GUI.
- **Multi-robot / shared-module support** — single robot per file (per
  schema decision in OQ-4).
- **Validation panel beyond loader errors** — surface
  `description::load_error` in the UI; richer rule-checks come later.

## Public surface

| Surface                         | Form                                                          |
|---------------------------------|---------------------------------------------------------------|
| Binary name                     | `robosim-viz`                                                 |
| CLI                             | `robosim-viz <path-to-description.json> [--mode=edit]`        |
| Mode argument                   | `--mode=edit` (default in v0). `--mode=live` / `replay` reserved. |
| Window title                    | `robosim-viz — <robot.name> [<mode>]`                         |
| Panels                          | left: scene tree; right: inspector; bottom: status bar (loader errors). |
| Viewport hotkeys                | `F` frame-fit selection (or whole scene if no selection); `W` translate gizmo; `E` rotate gizmo; `X` toggle world / local space; `Esc` clear selection. |
| Mouse — orbit camera            | MMB-drag rotates around pivot.                                |
| Mouse — pan camera              | RMB-drag pans.                                                |
| Mouse — zoom camera             | Wheel zooms.                                                  |
| Mouse — pick                    | LMB-click on a primitive selects the corresponding entity.    |
| File menu (Phase VD)            | Save (writes back to loaded path); Save As; Reload from disk (with confirmation if there are unsaved edits). |

The CLI / hotkeys / panel layout are the **stable public surface** for
the visualizer. Internal headers (`scene_snapshot.h`,
`edit_mode_builder.h`, `camera.h`, `picking.h`, `renderer.h`) are
implementation, not contract — refactor freely under the public
surface.

## Tech stack

Per `docs/ARCHITECTURE.md` "Visualizer":

| Concern               | Library                       | License           |
|-----------------------|-------------------------------|-------------------|
| Window + input        | GLFW                          | zlib/libpng       |
| Renderer              | OpenGL 3.3 core via glad      | (loader is MIT)   |
| UI                    | Dear ImGui (docking branch)   | MIT               |
| Gizmos                | ImGuizmo                      | MIT               |
| Math                  | GLM                           | MIT               |
| Mesh import (post-v0) | Assimp (BSD); OpenCASCADE much later | BSD / LGPL |

All deps via `FetchContent` with pinned commit SHAs (same pattern as
the top-level `CMakeLists.txt` for nlohmann/json, googletest, spdlog).
The viz subtree gates the FetchContent calls behind
`ROBOSIM_BUILD_VIZ` so headless CI does not pull GLFW / OpenGL.

**System dev packages** required when `ROBOSIM_BUILD_VIZ=ON` (Ubuntu
24.04 names; equivalents on other distros):

```
libgl-dev libwayland-dev wayland-protocols libxkbcommon-dev xorg-dev
python3 python3-jinja2
```

These are installed by the viz CI job, not the headless sanitizer
matrix. glad's loader generation runs at configure time and needs
`python3` plus the `jinja2` template library.

## Architectural shape

- **First-class subsystem at `src/viz/`.** Not a tool. Tests live at
  `tests/viz/`.
- **Separate binary**, never linked into sim core. Sim core does not
  link against, depend on, or know about the visualizer.
- **Single binary, mode switching.** Edit / Live / Replay are modes
  inside one app.
- **Depends on** `src/description/` (loader + schema + serializer).
  **Does not depend on** MuJoCo, Layer 5, sim core, HAL, or backends.
- **Process model** — standalone GUI app. No IPC with sim core in v0.
  Live mode (post-v0) connects via the same WPILOG / NetworkTables
  stream that AdvantageScope reads — no custom IPC.

## Determinism exception

The visualizer is interactive tooling, not sim core. The bans on
`std::chrono::system_clock`, `std::chrono::steady_clock`,
`std::random_device`, and default-seeded RNG (see
`.claude/skills/code-style.md` "Determinism") **do not apply** inside
`src/viz/`. Wall-clock for animation, frame pacing, and GLFW input
timestamps is acceptable.

The sim core's determinism guarantees are unaffected because the
visualizer reads from the sim core (and the description JSON), never
writes to it. The `lint.sh` banned-clock check explicitly skips
`src/viz/`.

Tests under `tests/viz/` likewise do not need seeded RNG; state this
explicitly in test plans so the exception is visible to the
test-reviewer.

## Scene-snapshot seam

**The single architectural commitment that lets Edit-only v0 grow into
Live / Replay without a rewrite.**

The renderer must **not** read directly from `robot_description`. It
reads from a `scene_snapshot` — a flat, immutable bundle of nodes with
world transforms, primitive shapes, axes, selection state, and
per-frame data slots reserved for Live / Replay (joint angles, contact
points, force vectors).

```
description::robot_description ──► edit_mode_builder ──► scene_snapshot ──► renderer
                                                          ▲
                                                          │ (post-v0)
                                  state_stream ───────────┘
                                  (Live / Replay)
```

- Edit mode populates the snapshot from a `robot_description`.
- Live mode (post-v0) populates it from a state-stream message.
- Replay mode (post-v0) populates it from a WPILOG sample.

Do not bypass this seam. The Phase VB test plan pins it explicitly:
the renderer touches only the snapshot; any data source can produce a
snapshot.

## Primitive visual defaults

Phase VB renders schema v1 descriptions with simple primitives:

- **Link visuals** are cylinders along local `+Z`. The local origin
  sits at the **proximal cap** (the end attached to the parent joint);
  the cylinder extends to `z = link.length_m`. Fixed radius `0.05 m`.
- **Joint axes** are arrows along local `+Z` with the same anchoring
  convention: base at the local origin, tip at `z = length_m`.
  Default `length_m = 0.20`, `radius_m = 0.01`.
- **Box primitives** use explicit half-extents on all three axes,
  centered at the local origin. `length_m` is not overloaded for
  boxes.
- **Sphere primitives** are centered at the local origin with
  `radius_m`.

The proximal-end origin for cylinders / arrows is the rev-4 update
to TEST_PLAN conventions #8 / #9: an arm link's natural pivot is the
parent-joint attachment, not its midspan. Picking, AABB, and the
unit-cylinder mesh all share this convention.

These constants are v0 readability defaults, not schema. If schema v2
adds `link.visual_radius_m`, the builder behavior becomes "use the
field when present, otherwise keep the `0.05 m` default."

## Schema implications

Edit-mode gizmo manipulation requires joint and link origin transforms
in the description schema. Schema v1 (which the v0 loader ships) has
no such fields. The v1→v2 schema bump and the matching
`description::save_to_file` round-trip serializer are Phase VC of the
visualizer plan. v0 viz Phase VB renders v1 files (everything at
world origin); Phase VD writes v2 fields when gizmos move things.

The schema bump is **a sim-core change** under the same TDD discipline
as the rest of the description loader — see
`.claude/skills/robot-description-loader.md`. It is not "viz work"
hidden from sim-core review.

## Known limits

- **Save flow loses JSON comments and whitespace.** JSON has no
  comment support; a hand-authored file with whitespace conventions
  loses them on round-trip through the serializer. v0 documents this
  and accepts it. Comment-preserving migration (sidecar or YAML) is a
  v1+ concern.
- **Headless smoke test.** The OpenGL pipeline is exercised only by a
  smoke test (`--smoke-test` flag). On headless CI we either Xvfb-host
  the build job or run the smoke test only on developer-machine
  matrices. Pure-logic pieces (snapshot, camera, picking, scene tree,
  gizmo math) are unit-testable without a display.
- **No CAD / mesh import.** Primitive shapes only. STEP / glTF / STL
  import lives behind Assimp (and OpenCASCADE for STEP) and is
  post-v0.
- **Single-robot per file.** Multi-robot / shared-module is OQ-4-
  decided to be a v1+ feature.
- **Gizmo doesn't move motors / sensors.** Motors and sensors bind to
  joints, not space — moving the gizmo on a joint moves the bound
  devices implicitly. There is no "drag the motor in 3D" affordance
  and won't be one until we have a reason for it.
- **Joint-axis arrow doesn't reflect `joint.axis`.** The arrow is
  drawn along local `+Z` regardless of the actual joint axis vector.
  Aligning the arrow to `joint_axis_local` is a follow-up; for the
  v0-arm the axis happens to be `[0, 1, 0]` so the arrow is
  visually misleading until that lands.

## Open follow-ups

- **Smoke-test transport** — finalize whether `--smoke-test` uses
  EGL_SURFACELESS_MESA, headless `xvfb-run`, or runtime-detect skip.
  Decide in Phase VB1 test plan.
- **Live mode state-stream subscriber** — a separate design pass when
  Live mode is scheduled. The stream format is whatever the sim core
  emits for AdvantageScope (WPILOG + NT); the viz subscribes
  read-only.
- **Replay scrubber UX** — keyboard-driven scrub with adjustable
  speed, log-time vs. wall-time toggle. Out of v0 scope but worth
  sketching before Live lands so the playback head model is shared.
- **Gizmo snap config** — translate snap (m), rotate snap (deg).
  Surfaced in Phase VD; default values TBD.
- **Joint axis arrow orientation** — render the joint-axis arrow
  along `scene_node::joint_axis_local`, not always local `+Z`.
  Cosmetic but high-impact for non-trivial axes.

## Cross-references

- `docs/VISUALIZER_V0_PLAN.md` — phased plan (VA scaffold, VB
  read-side, VC schema bump + serializer, VD gizmo manipulation).
- `docs/ARCHITECTURE.md` "Visualizer" — binding architecture.
- `docs/OPEN_QUESTIONS.md` OQ-7 (decided 2026-04-29) — original
  framing.
- `.claude/skills/robot-description-loader.md` — upstream loader
  consumed by Edit mode.
- `.claude/skills/code-style.md` — applies to viz code, with the
  determinism exception noted above.
- `.claude/skills/tdd-workflow.md` and
  `.claude/skills/adding-a-feature.md` — apply unchanged.
