# Visualizer v0 implementation plan

This is the **entry point for the first visualizer session**. It assumes
a fresh Claude Code session with no prior context. It runs in parallel
with the main v0 work in `docs/V0_PLAN.md`; nothing in the sim core
depends on the visualizer being attached.

## Read first, in this order

1. **`CLAUDE.md`** — repo orientation, non-negotiables, the rule about
   reading skills before implementing.
2. **`docs/ARCHITECTURE.md`** — the **"Visualizer"** section is the
   binding architecture for this work. Also: process model, robot
   description format, logging / observability.
3. **`docs/OPEN_QUESTIONS.md`** — OQ-7 (decided 2026-04-29) and OQ-11
   (folded into OQ-7). The original framings are preserved for
   context; the resolutions are in `ARCHITECTURE.md`.
4. **`docs/VISUALIZER_V0_PLAN.md`** — this file.
5. **`.claude/skills/code-style.md`** — C++ style; visualizer code
   follows the same rules as the sim core, with the determinism
   exception noted in `ARCHITECTURE.md` "Visualizer" and elaborated
   in the visualizer skill.
6. **`.claude/skills/tdd-workflow.md`** + **`adding-a-feature.md`** —
   TDD discipline and the test-reviewer gate apply here too.
7. **`.claude/skills/visualizer.md`** — created in Phase VA1. Once it
   exists, prefer it as the entry point.

## Status

Foundational design decided (OQ-7 resolution + ARCHITECTURE.md
"Visualizer"). **Phase VA scaffold and Phase VB read-side are
landed:** `src/viz/` has the scene-snapshot seam, edit-mode builder,
orbit camera, ray-vs-primitive picking, OpenGL primitive renderer,
ImGui scene tree / inspector / status bar, and a working
`robosim-viz` CLI. The cylinder/arrow primitive convention was
amended to anchor at the proximal cap (TEST_PLAN rev 4) so a link's
local origin coincides with its parent-joint attachment. 49 viz-side
tests are green.

ImGuizmo translate / rotate manipulators are wired in `main.cpp`
against `scene_node::world_from_local`, but they only mutate the
in-memory snapshot — there is no `link.visual_origin` /
`joint.origin` schema field to write back to and no
`description::save_to_file` yet. **Phase VC (schema bump + serializer)
is the next gating step.** Phase VD's UI scaffolding (gizmo + snap
controls) is partly in place; the formal VD test plan + save flow
land after VC.

## Scope

The 3D viewer is a single binary with three modes (per `ARCHITECTURE.md`
"Visualizer"): **Edit**, **Live**, **Replay**. **v0 ships Edit mode
only.** Live and Replay are post-v0.

### In scope (v0 — Edit mode)

- **Load** an existing robot description JSON via the existing loader.
- **Render** the kinematic tree in 3D using primitive shapes (link =
  cylinder along its `length_m`; joint axis drawn as an arrow).
- **Orbit camera** (rotate / pan / zoom around a pivot, frame-fit
  hotkey).
- **Inspect** every entity from an ImGui scene tree (left panel) and an
  inspector panel (right) showing all schema fields read-only.
- **Pick** entities by clicking in the 3D viewport.
- **Manipulate** the selected link's visual origin and the selected
  joint's origin with **Unreal-style ImGuizmo handles** (translate /
  rotate, world / local toggle, W / E hotkeys).
- **Save back to JSON** — round-trips through a new
  `description::save_to_file` so edits persist.

### Explicitly out of scope (deferred)

- **Live and Replay modes.** Architectural seam preserved in Phase VB
  (renderer reads from a scene snapshot, not from the loader directly)
  but no state-stream consumer or WPILOG reader in v0.
- **CAD / mesh import** (STEP, IGES, glTF, OBJ, STL). Edit-mode
  roadmap item; primitive shapes only for v0.
- **Authoring from scratch** — creating new links / joints / motors via
  the GUI. v0 edits an existing file; create-from-scratch is an
  Edit-mode roadmap item.
- **Vendor-binding UI** — assigning CAN IDs, picking firmware versions
  through the GUI. Edit-mode roadmap item.
- **Multi-robot / shared-module support** — single robot per file (per
  schema decision in OQ-4).
- **Validation panel beyond loader errors** — surface the loader's
  `load_error` in the UI; richer rule-checks come later.

## Architectural shape

The binding decisions are in `ARCHITECTURE.md` "Visualizer". Restated
here for the work-plan context:

- **Lives at `src/viz/` as a first-class subsystem.** Not in `tools/`.
- **Separate binary**, not linked into sim core. Built by an opt-in
  CMake flag `ROBOSIM_BUILD_VIZ=OFF` (default OFF so the headless
  CI matrix doesn't need GLFW / OpenGL).
- **Single binary, mode switching.** Edit / Live / Replay are modes
  inside one app, not separate binaries — they share the 3D engine,
  scene graph, camera, and selection model. v0 implements only Edit
  mode but the architecture must not preclude the others.
- **Depends on** `src/description/` (loader + schema) and the new
  `src/description/serializer.{h,cpp}` (Phase VC). **Does not depend
  on** MuJoCo, Layer 5, sim core, HAL, or backends.
- **Process model:** standalone GUI app. No IPC with sim core in v0.
  Live mode (post-v0) connects via the same WPILOG / NetworkTables
  stream that AdvantageScope reads — no custom IPC.

### Tech stack

Per `ARCHITECTURE.md` "Visualizer":

- **Window + input:** GLFW (zlib/libpng license).
- **Renderer:** OpenGL 3.3 core (portable, mature on Linux). glad as
  the loader.
- **UI:** Dear ImGui (docking branch).
- **Gizmos:** ImGuizmo (Unreal-style translate / rotate / scale, world
  / local toggle, snap support).
- **Math:** GLM (header-only, MIT). ImGuizmo already uses it.
- **Mesh import (post-v0):** Assimp (BSD) — **not added in v0**.
  STEP via OpenCASCADE much later.

All deps via `FetchContent` with pinned commit SHAs, same pattern as
the existing top-level `CMakeLists.txt`. SHAs resolved via
`scripts/resolve-dep-sha.sh`.

## Design constraint: data-source seam (carries through every phase)

The renderer must **not** read directly from `robot_description`. It
reads from a `scene_snapshot` (working name) — a flat, immutable bundle
of nodes with world transforms, primitive shapes, axes, selection
state, and per-frame data slots reserved for Live / Replay (joint
angles, contact points, force vectors). Edit mode populates the
snapshot from a `robot_description`; Live mode (post-v0) populates it
from a state-stream message; Replay mode (post-v0) populates it from a
WPILOG sample.

This is the single architectural commitment that lets Edit-only v0 grow
into Live / Replay without a rewrite. Phase VB enforces it.

## Phases

Each phase is a candidate session boundary. Phases VB / VC / VD each go
through the test-reviewer gate before any test code is written.

### Phase VA — Scaffold

No TDD step; this is engineering infrastructure.

1. **VA1. Skill drafted.** `.claude/skills/visualizer.md` with scope,
   public surface (CLI flags, panels, hotkeys, modes), tech stack,
   determinism exception, data-source seam, and known limits.
2. **VA2. Directory + CMake.** Create `src/viz/` (with
   `CMakeLists.txt`) and `tests/viz/`. Top-level CMake gains
   `ROBOSIM_BUILD_VIZ` option (default OFF) and conditional
   `FetchContent` for GLFW, glad, Dear ImGui, ImGuizmo, GLM. Pinned
   SHAs.
3. **VA3. Empty entry point.** `src/viz/main.cpp` opens a GLFW window,
   initializes ImGui + OpenGL3 backends, runs an empty frame loop with
   a "Hello, robosim" demo window, exits cleanly. The binary is named
   `robosim-viz`.
4. **VA4. CI.** Add a separate workflow / matrix entry that builds the
   visualizer (compile-only; no display required). Existing sanitizer
   matrix stays headless and unaffected.

Commit boundary: VA1 (skill) one commit; VA2 + VA3 one commit; VA4 one
commit.

### Phase VB — Read-side: scene snapshot, render, inspect, pick

**This is the first visualizer feature under TDD.** Pure-logic pieces
are TDD-able with GoogleTest; rendering / ImGui glue is exercised by a
smoke test that builds and exits cleanly with `--smoke-test` (no
display required if we use `EGL_SURFACELESS_MESA` or skip with a
runtime-detect fallback — finalize in test plan).

1. **VB1. Test plan.** Cover, at minimum:
   - **Scene snapshot data model**: the seam itself. A pure POD bundle
     (nodes with world transforms, primitive shapes, axes, selection
     state, reserved per-frame slots for joint angles / contact /
     forces) with **no dependency on `robot_description`**. Tests
     pin the contract: any data source can produce a snapshot; the
     renderer reads only the snapshot.
   - **Edit-mode snapshot builder**: `robot_description` →
     `scene_snapshot`. Correct kinematic-tree structure (parent /
     child), world-origin-rooted transforms (v0 schema), joint axes
     preserved, primitive shape per link (cylinder from `length_m`).
   - **Camera math**: orbit (pitch / yaw / zoom around pivot), pan,
     frame-fit-to-AABB. Assertions on output matrices vs. closed-form
     values.
   - **Picking**: ray vs. cylinder / box / sphere primitives, returns
     hit index + entity reference. Boundary cases (parallel ray,
     grazing, behind camera).
   - **Determinism note**: the visualizer is exempt from sim-core
     determinism rules; tests do not need seeded RNG. State this
     explicitly in the test plan to make the exception visible.
2. **VB2. Submit to test-reviewer.** Iterate until
   `ready-to-implement`.
3. **VB3. Implement failing tests.**
4. **VB4. Implement** `src/viz/scene_snapshot.{h,cpp}` (the seam),
   `src/viz/edit_mode_builder.{h,cpp}` (description → snapshot),
   `src/viz/camera.{h,cpp}` (orbit math), `src/viz/picking.{h,cpp}`
   (ray-primitive intersection).
5. **VB5. Implement** `src/viz/renderer.{h,cpp}` — primitive cylinders /
   boxes / arrows, grid, world axes, basic Phong-ish lighting.
   Selection highlight pass. **Reads from `scene_snapshot` only.**
6. **VB6. Implement panels** — scene tree panel (ImGui, hierarchical),
   inspector panel (read-only fields driven by the schema), status bar
   showing loader errors.
7. **VB7. CLI** — `robosim-viz <path-to-description.json>`; load,
   render, inspect, exit on window close. Mode argument (`--mode=edit`)
   reserved but defaults to `edit` for v0.
8. **VB8. Skill update** — fill in public surface and known limits in
   `.claude/skills/visualizer.md`. Document the snapshot seam contract.

Commit boundary: snapshot + edit-mode builder + camera + picking (and
their tests) one commit; renderer + panels one commit; CLI + skill
update one commit.

### Phase VC — Schema extension: origin transforms ✓ LANDED

**VC1–VC4 are complete.** See `tests/description/TEST_PLAN_VC.md` for
the full test contracts (rev-4 reviewer-approved, 63+89 = 152 tests
green across clang/gcc Debug + ASan + UBSan).

1. **VC1. Schema design write-up.** ✓ Done. `origin_pose` POD added to
   `src/description/schema.h`; `link.visual_origin` and `joint.origin`
   added as optional fields (default identity per D-VC-2).
   `descriptions/v0-arm.json` bumped to schema_version 2 (no origin
   keys — identity is the v0-arm's intent). `docs/ARCHITECTURE.md`
   updated to show the v2 form.
2. **VC2. Test plan + test-reviewer.** ✓ Done. `tests/description/
   TEST_PLAN_VC.md` (rev-4, `ready-to-implement`). Covers V0–V10
   (loader) and S1a/S1/S2/S3/S4/S5/S6/S8 (serializer).
3. **VC3. Loader implementation.** ✓ Done. `src/description/loader.cpp`
   extended for schema_version 2. `src/description/origin_pose.{h,cpp}`
   expose `compose_origin`, `validate_finite_xyz`, `validate_finite_rpy`.
   Configure-time lint `cmake/lint_loader_finite_seam.cmake` enforces
   the D-VC-3 seam call-site pin (no bare `isfinite` etc. in non-helper
   TUs). 152 tests green.
4. **VC4. Serializer.** ✓ Done. `src/description/serializer.{h,cpp}`
   implement `save_to_file` using `nlohmann::ordered_json` (key order
   per D-VC-8, identity origins omitted per D-VC-9, `dump(2)` per
   D-VC-10). S8 SHA guard pins `73fd52187ceb...` against pinned
   nlohmann v3.12 SHA. Non-atomicity documented in
   `.claude/skills/robot-description-loader.md` (D-VC-11).

**VD is now unblocked** — gizmo persistence can write `joint.origin`
and `link.visual_origin` to disk via `save_to_file`.

### Phase VD — Gizmo manipulation ✓ LANDED

VD1–VD6 are complete. See `tests/viz/TEST_PLAN_VD.md` (rev-2
reviewer-approved, `ready-to-implement`) for the full test contract.
The patch:

- Extended `viz::build_edit_mode_snapshot` to honor schema-v2 origins
  with the kinematic-vs-visual frame distinction (B section).
- Added `description::decompose_origin` as the algebraic inverse of
  `compose_origin` (O section).
- Added `viz::apply_gizmo_target` (`src/viz/edit_mode_apply.{h,cpp}`)
  — pure logic for writing gizmo targets back to the description's
  `joint.origin` / `link.visual_origin` (A section).
- Added `viz::edit_session` + `load_session` / `save_session` /
  `reload_session` (`src/viz/edit_session.{h,cpp}`) — load/save/reload
  state machine with a dirty bit (D section).
- Wired the above into `src/viz/main.cpp`: gizmo apply route, File
  menu (Save, Save As text-input modal, Reload-from-disk with
  Discard-changes confirmation), `Ctrl+S` hotkey.
- 70+ new tests added, 457/457 viz + description suite green.

Original VD plan retained below for reference:

1. **VD1. Test plan.** Pure logic only here; the rest is ImGuizmo
   integration tested via the smoke test.
   - Selection model: clicking a link / joint sets the selection;
     escape clears it.
   - Gizmo delta application: given a selected joint and a delta
     transform, produce an updated `joint.origin` and an updated
     in-memory `robot_description`. Snapshot rebuilds from the
     updated description.
   - Save flow: edit-then-save round-trips identically to a hand-
     edited file (uses the VC4 serializer).
2. **VD2. Test-reviewer.**
3. **VD3. Failing tests, then implementation.**
4. **VD4. ImGuizmo wiring** — translate + rotate gizmos for selected
   link visual origins and joint origins. World / local space toggle,
   W (translate) / E (rotate) hotkeys, snap (configurable). No scale
   gizmo for v0 (link sizing is driven by `length_m` / inertia, which
   is a follow-up concern).
5. **VD5. File menu** — Save (writes back to the loaded path), Save
   As, Reload from disk (drops unsaved edits with confirmation).
6. **VD6. Skill update** — public surface for gizmo flow, hotkeys,
   save semantics, known limits (e.g. gizmo doesn't move motors /
   sensors because they bind to joints, not space).

Commit boundary: gizmo + selection + save flow as one commit; UI
polish as a follow-up if needed.

### Post-VD follow-up — Surface attachment ✓ LANDED

Edit mode now has a Fusion-style two-click Attach command on top of
the VD write path:

- `viz::attachment` (`src/viz/attachment.{h,cpp}`) lives in
  `robosim_viz_core` and is description-free.
- `pick_attachment_plane(scene_snapshot, ray)` turns a viewport hover
  or click into an attachment plane.
- `compute_attachment_target(...)` computes the target transform from
  the moving plane, target plane, and modifiers.
- The UI flow is `Start attach` → click moving surface → click target
  surface; the second click applies immediately through
  `apply_gizmo_target`.
- The Attach window only holds modifiers/status: `Offset m`,
  `Quarter turns`, and `Flip normal`.
- Hover preview draws a translucent temporary plane over the surface
  that will be captured.
- Supported v0 surfaces are cylinder/arrow caps and sides, box faces,
  rotation-arrow box proxies, and mesh oriented-AABB faces. Kraken X60
  picking/attachment accounts for the renderer's 180-degree Y rotation.

Known follow-ups: keep the first selected plane visible after click
one, add a live ghost preview before click two, add undo/redo, and
eventually decide whether attachments become persistent mate records,
fixed joints, or remain baked origin edits.

## What NOT to do in early sessions

- **Don't break the snapshot seam.** Phase VB is the one chance to get
  this right cheaply; bypassing it (renderer reading
  `robot_description` directly) makes Live / Replay much more expensive
  later.
- **Don't import CAD.** Tracked, deferred. v0 is primitive shapes only.
- **Don't subscribe to a running sim's state stream.** Live mode is
  post-v0 and needs its own design pass.
- **Don't add a "create new link / joint" UI.** v0 edits an existing
  description; create-from-scratch is an Edit-mode roadmap item.
- **Don't link the visualizer into sim core, or make sim core depend
  on it.** Always a separate binary, always optional.
- **Don't use `system_clock` / `steady_clock` / `random_device` outside
  `src/viz/`.** The exception applies only inside the visualizer
  subtree.

## Suggested session boundaries

- **Session 1.** Phases VA + VB1–VB2 (scaffold + read-side test plan
  approved by test-reviewer). Hard stop after `ready-to-implement`.
- **Session 2.** VB3–VB8 (read-side implementation, viewer ships
  view + inspect for the v0-arm).
- **Session 3.** Phase VC end-to-end (schema bump + serializer).
- **Session 4.** Phase VD end-to-end (gizmos + save).

If a session runs short, stop at the last passing test boundary.

## Risks and unknowns to track

- **`ImGuizmo` for non-trivial parent-frame edits.** ImGuizmo expects a
  4×4 model matrix in some space; mapping it to "joint origin in the
  parent link's frame" is straightforward but needs verification. The
  test plan in VD1 will pin this contract.
- **Pure-logic test reach for the rendering layer.** Snapshot, camera,
  picking, scene tree, and gizmo math are all testable. The OpenGL
  pipeline itself is exercised only by the smoke test. If smoke
  testing proves flaky on headless CI we either Xvfb-host the build
  job or skip the smoke test on CI and run it in a developer-machine
  matrix.
- **Schema bump (Phase VC) breaks the loader's existing test suite if
  not done carefully.** The 83 existing tests assume v1 schema. The
  schema-design writeup in VC1 must enumerate which existing tests
  change (ideally none, by making the new fields optional) and which
  new tests are added.
- **Save flow with comments / formatting in JSON.** JSON has no
  comment support; a hand-authored file with whitespace conventions
  will lose them on round-trip. v0 documents this as a known limit
  and accepts it. Schema-version-aware migration (e.g. comment-
  preserving via a sidecar or YAML migration) is a v1+ concern.
- **Snapshot data model under Live / Replay.** The snapshot must
  carry per-frame data slots that v0 doesn't populate (joint angles,
  contact lists, force vectors) without making them awkward when
  they're empty. VB1 should sketch the post-v0 fields even though v0
  doesn't write them.

## If you get stuck

- The binding architecture is in `docs/ARCHITECTURE.md` "Visualizer".
  Don't re-litigate decided questions silently.
- The sim-core layers' design intent is in their layer skills.
  Visualizer-specific decisions live in `.claude/skills/visualizer.md`
  once Phase VA1 lands.
- For schema questions, the user makes the call. Surface the
  trade-off; don't guess.
