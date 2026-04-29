# Visualizer v0 implementation plan

This is the **entry point for the first visualizer session**. It assumes
a fresh Claude Code session with no prior context. It runs in parallel
with the main v0 work in `docs/V0_PLAN.md`; nothing in the sim core
depends on the visualizer being attached.

## Read first, in this order

1. **`CLAUDE.md`** — repo orientation, non-negotiables, the rule about
   reading skills before implementing.
2. **`docs/ARCHITECTURE.md`** — process model (visualizer is a separate
   process), robot description format, logging / observability section.
3. **`docs/OPEN_QUESTIONS.md`** — **OQ-7 (Visualizer)** and **OQ-11
   (Authoring GUI / CAD)**. This work resolves OQ-7 in part and reopens
   OQ-11 in a tightly-scoped form. Do not start before those resolutions
   are written up (Phase VA1).
4. **`docs/VISUALIZER_V0_PLAN.md`** — this file.
5. **`.claude/skills/code-style.md`** — C++ style; visualizer code
   follows the same rules as the sim core (with the exceptions in
   §"Determinism" below).
6. **`.claude/skills/tdd-workflow.md`** + **`adding-a-feature.md`** —
   TDD discipline and the test-reviewer gate apply here too.
7. **`.claude/skills/visualizer.md`** — created in Phase VA. Once it
   exists, prefer it as the entry point.

## Status

Foundational design fixed; implementation gated on (a) OQ-7 / OQ-11
write-up and (b) `descriptions/v0-arm.json` actually loading via
`description::load_from_file` (V0_PLAN Phase B). Phase VA scaffold can
land before V0_PLAN B7/B8 finish — only Phase VB onward consumes the
loader.

## Scope

### In scope (v0 visualizer)

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

- **CAD / mesh import** (STEP, IGES, glTF, OBJ, STL). Tracked under
  OQ-11; primitive shapes only for v0.
- **State-stream / replay viewer** — animating joints from a running
  sim's WPILOG output. AdvantageScope owns this; revisit post-v0 if
  there's a sim-specific view AdvantageScope can't show.
- **Authoring from scratch** — creating new links/joints/motors via the
  GUI. v0 edits an existing file; create-from-scratch is a follow-up.
- **Multi-robot / shared-module support** — single robot per file (per
  schema decision in OQ-4).
- **Validation panel beyond loader errors** — surface the loader's
  `load_error` in the UI; richer rule-checks come later.
- **Determinism / replay discipline** — visualizer is interactive
  tooling, not sim core.

## OQ resolutions this plan implies (must be written up in VA1)

### OQ-7 (Visualizer) — partial resolution

Pick **option C** with a v0 scoping clarification:

- **AdvantageScope** owns log replay and time-series visualization.
- **A custom viewer** (this project) owns description-time
  visualization and authoring assist — things AdvantageScope cannot do
  because they involve editing the description, not viewing match
  output.
- v0 ships the description-time viewer. State-stream consumption stays
  open under OQ-7 for post-v0.

### OQ-11 (Authoring GUI / CAD) — partial reopen

Re-open under tight scope:

- v0 GUI **edits an existing JSON** in place (origins of already-
  declared links and joints). It does **not** create new entities, does
  **not** import CAD, does **not** rewire CAN bindings via GUI.
- Full authoring (create from scratch, CAD import, vendor binding UI)
  remains deferred. Re-trigger when a real second robot description is
  authored and hand-editing JSON becomes the bottleneck.

Both writeups land in `docs/ARCHITECTURE.md` (new "Visualizer" section)
and `docs/OPEN_QUESTIONS.md` (status flips). User sign-off required
before VA scaffold is committed.

## Architectural shape

- **Separate binary**, not linked into sim core. Built by an opt-in
  CMake flag `ROBOSIM_BUILD_VISUALIZER=OFF` (default OFF so the headless
  CI matrix doesn't need GLFW/OpenGL).
- **Lives in `tools/visualizer/`** alongside `tools/rio-bench/`. It is
  tooling, not a layer of the sim.
- **Depends on** `src/description/` (loader + schema) and the new
  `src/description/serializer.{h,cpp}` (Phase VC). **Does not depend
  on** MuJoCo, Layer 5, sim core, HAL, or backends.
- **Process model:** standalone GUI app. No IPC with sim core in v0.

### Tech stack (proposed; finalized in VA1)

- **Window + input:** GLFW (zlib/libpng license).
- **Renderer:** OpenGL 3.3 core (portable, mature on Linux). glad as
  the loader. Vulkan rejected as overkill for v0.
- **UI:** Dear ImGui (docking branch).
- **Gizmos:** ImGuizmo (Unreal-style translate / rotate / scale, world
  / local toggle, snap support).
- **Math:** GLM (header-only, MIT). ImGuizmo already uses it.
- **Mesh import (later):** Assimp (BSD) — **not added in v0**, listed
  here for the post-v0 path.

All deps via `FetchContent` with pinned commit SHAs, same pattern as
the existing top-level `CMakeLists.txt`.

## Determinism

Visualizer is interactive tooling. The bans on `system_clock` /
`steady_clock` / `random_device` from `code-style.md` **apply only to
the sim core**, not to the visualizer. The visualizer is allowed to use
wall-clock time for animation, frame pacing, and GLFW input timestamps.
This is a documented exception that goes into `.claude/skills/visualizer.md`.

## Phases

Each phase is a candidate session boundary. Phases VB / VC / VD each go
through the test-reviewer gate before any test code is written.

### Phase VA — Scaffold

No TDD step; this is engineering infrastructure.

1. **VA1. OQ resolutions written up.** Update
   `docs/ARCHITECTURE.md` + `docs/OPEN_QUESTIONS.md` per the resolutions
   above. **Get user sign-off before committing.**
2. **VA2. Skill drafted.** `.claude/skills/visualizer.md` with scope,
   public surface (CLI flags, panels, hotkeys), tech stack, determinism
   exception, and known limits. Add to the index in `CLAUDE.md`.
3. **VA3. Directory + CMake.** Create `tools/visualizer/`
   (with `CMakeLists.txt`) and `tests/visualizer/`. Top-level CMake
   gains `ROBOSIM_BUILD_VISUALIZER` option (default OFF) and
   conditional `FetchContent` for GLFW, glad, Dear ImGui, ImGuizmo,
   GLM. Pinned SHAs, resolved via the existing
   `scripts/resolve-dep-sha.sh` pattern.
4. **VA4. Empty entry point.** `tools/visualizer/main.cpp` opens a GLFW
   window, initializes ImGui + OpenGL3 backends, runs an empty frame
   loop with a "Hello, robosim" demo window, exits cleanly.
5. **VA5. CI.** Add a separate workflow / matrix entry that builds the
   visualizer (still no display required for build-only). Existing
   sanitizer matrix stays headless and unaffected.

Commit boundary: VA1 (docs) and VA2 (skill) can be one commit; VA3 +
VA4 one commit; VA5 one commit.

### Phase VB — Read-side: scene model, render, inspect, pick

**This is the first visualizer feature under TDD.** Pure-logic pieces
are TDD-able with GoogleTest; rendering / ImGui glue is exercised by a
smoke test that builds and exits cleanly with `--smoke-test` (no
display required if we use `EGL_SURFACELESS_MESA` or skip with a
runtime-detect fallback — finalize in test plan).

1. **VB1. Test plan.** Cover, at minimum:
   - **Camera math**: orbit (pitch/yaw/zoom around pivot), pan, frame-
     fit-to-AABB. Assertions on output matrices vs. closed-form values.
   - **Scene-tree builder**: given a `robot_description`, produce a
     tree of nodes with correct parent/child relationships and (for
     v0) world-origin-rooted transforms. Joint axis preserved.
   - **Picking**: ray vs. cylinder / box / sphere primitives, returns
     hit index + entity reference. Boundary cases (parallel ray,
     grazing, behind camera).
   - **Determinism note**: the visualizer is exempt from sim-core
     determinism rules; tests do not need seeded RNG. State this
     explicitly in the test plan to make the exception visible.
2. **VB2. Submit to test-reviewer.** Iterate until
   `ready-to-implement`.
3. **VB3. Implement failing tests.**
4. **VB4. Implement** `tools/visualizer/scene.{h,cpp}` (kinematic tree
   from description), `camera.{h,cpp}` (orbit math), `picking.{h,cpp}`
   (ray-primitive intersection).
5. **VB5. Implement** `renderer.{h,cpp}` — primitive cylinders / boxes
   / arrows, grid, world axes, basic Phong-ish lighting. Selection
   highlight pass.
6. **VB6. Implement panels** — scene tree panel (ImGui, hierarchical),
   inspector panel (read-only fields driven by the schema), status bar
   showing loader errors.
7. **VB7. CLI** — `robosim-viz <path-to-description.json>`; load,
   render, inspect, exit on window close.
8. **VB8. Skill update** — fill in public surface and known limits in
   `.claude/skills/visualizer.md`.

Commit boundary: scene + camera + picking (and their tests) one
commit; renderer + panels one commit; CLI + skill update one commit.

### Phase VC — Schema extension: origin transforms

**Blocker for VD.** The current schema (v1) has no link visual origin
or joint origin — everything is implicitly at world origin. That's fine
for the single-DOF arm, but gizmo-manipulation needs fields to write
into.

1. **VC1. Schema design write-up.** Propose:
   - `link.visual_origin`: `{ xyz_m: [x, y, z], rpy_rad: [r, p, y] }`,
     **optional**, defaults to identity.
   - `joint.origin`: `{ xyz_m: [x, y, z], rpy_rad: [r, p, y] }`,
     **optional**, defaults to identity. Expressed in the parent
     link's frame.
   - Bump `schema_version` from 1 to 2. Loader accepts both; for v1
     files the optional fields default to identity (no behavioral
     change for the existing v0-arm).
   - Update `descriptions/v0-arm.json` to v2 explicitly (origins still
     identity).
   - Update `docs/ARCHITECTURE.md` "Robot description format" sketch.
2. **VC2. Test plan + test-reviewer.** Loader tests for the new
   optional fields: present-and-valid, present-and-malformed (each
   sub-field), absent (defaults), v1→v2 forward compatibility, both
   versions produce equivalent struct when origins are identity.
3. **VC3. Loader implementation** — extend
   `src/description/{schema.h,loader.cpp}`. Re-run existing loader test
   suite; existing tests must not need changes (defaults preserve
   behavior).
4. **VC4. Serializer** — `src/description/serializer.{h,cpp}` exposing
   `save_to_file(robot_description, path) -> std::expected<void, save_error>`.
   Serializer test plan + test-reviewer gate. Round-trip property test:
   `load_from_file(p)` → `save_to_file(d, p')` → `load_from_file(p')`
   yields a struct equal to the first load. Also: serialized JSON is
   byte-stable for the same input (canonical key ordering, fixed float
   formatting), so diff-review of GUI edits is sane.

Commit boundary: schema + loader extension one commit; serializer one
commit.

**This phase modifies sim-core surfaces** (the loader and schema). It
goes through the same TDD discipline as V0_PLAN Phase B and the same
review gates. It is **not** "visualizer work" hidden from sim-core
review.

### Phase VD — Gizmo manipulation

1. **VD1. Test plan.** Pure logic only here; the rest is ImGuizmo
   integration tested via the smoke test.
   - Selection model: clicking a link / joint sets the selection;
     escape clears it.
   - Gizmo delta application: given a selected joint and a delta
     transform, produce an updated `joint.origin` and an updated
     in-memory `robot_description`.
   - Save flow: edit-then-save round-trips identically to a hand-
     edited file (uses the VC3 serializer).
2. **VD2. Test-reviewer.**
3. **VD3. Failing tests, then implementation.**
4. **VD4. ImGuizmo wiring** — translate + rotate gizmos for selected
   link visual origins and joint origins. World / local space toggle,
   W (translate) / E (rotate) hotkeys, snap (configurable). No scale
   gizmo for v0 (link sizing is driven by `length_m` / inertia, which
   is a Phase-V-stretch concern).
5. **VD5. File menu** — Save (writes back to the loaded path), Save
   As, Reload from disk (drops unsaved edits with confirmation).
6. **VD6. Skill update** — public surface for gizmo flow, hotkeys,
   save semantics, known limits (e.g. gizmo doesn't move motors /
   sensors because they bind to joints, not space).

Commit boundary: gizmo + selection + save flow as one commit; UI
polish as a follow-up if needed.

## What NOT to do in early sessions

- **Don't skip the OQ resolution writeups.** The user must sign off
  before VA scaffold lands. Implementing first and writing the OQ
  resolution as an afterthought is exactly the "silently take a
  position" failure mode `adding-a-feature.md` warns against.
- **Don't import CAD.** Tracked, deferred. v0 is primitive shapes only.
- **Don't subscribe to a running sim's state stream.** That's a v1
  follow-up and needs a separate design pass.
- **Don't add a "create new link/joint" UI.** v0 edits an existing
  description; create-from-scratch is OQ-11-deferred.
- **Don't link the visualizer into sim core, or make sim core depend
  on it.** Always a separate binary, always optional.

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
- **Pure-logic test reach for the rendering layer.** Camera, picking,
  scene tree, and gizmo math are all testable. The OpenGL pipeline
  itself is exercised only by the smoke test. If smoke testing proves
  flaky on headless CI we either Xvfb-host the build job or skip the
  smoke test on CI and run it in a developer-machine matrix.
- **Schema bump (Phase VC) breaks the loader's existing test
  suite if not done carefully.** Loader's existing tests assume the
  v1 schema. The schema-design writeup in VC1 must enumerate which
  existing tests change (ideally none, by making the new fields
  optional) and which new tests are added.
- **Save flow with comments / formatting in JSON.** JSON has no
  comment support; a hand-authored file with whitespace conventions
  will lose them on round-trip. v0 documents this as a known limit
  and accepts it. Schema-version-aware migration (e.g. comment-
  preserving via a sidecar or YAML migration) is a v1+ concern.

## If you get stuck

- Re-read this plan and the linked OQ entries. Don't silently take a
  position on OQ-7 / OQ-11 by implementing.
- The sim-core layers' design intent is in their layer skills.
  Visualizer-specific decisions live in `.claude/skills/visualizer.md`
  once Phase VA2 lands.
- For schema questions, the user makes the call. Surface the
  trade-off; don't guess.
