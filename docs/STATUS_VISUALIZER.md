# Visualizer status

Parallel work stream under `src/viz/`, opt-in via `ROBOSIM_BUILD_VIZ=ON`.
See `docs/VISUALIZER_V0_PLAN.md` for the plan and
`.claude/skills/visualizer.md` for actionable knowledge.

## Headline

**v0 Edit mode is functionally complete.** Phases VA + VB + VC + VD all
landed; layout-persistence follow-up (VL) and surface attachment
follow-up added on top. Viz tests are green.

## Phase VA — scaffold

Scene-snapshot seam, `robosim-viz` CLI, opt-in build flag.

## Phase VB — read-side

- Edit-mode builder honoring v2 origins via the kinematic-vs-visual
  frame distinction (descendants compose against parent kinematic,
  never parent visual).
- Orbit camera.
- Ray-vs-primitive picking.
- OpenGL primitive renderer.
- ImGui panels.
- Cylinder / arrow primitives anchor at the **proximal cap**
  (TEST_PLAN rev 4) so a link's local origin lands at its
  parent-joint attachment.

## Phase VC — schema v2 + serializer

Description schema v2 + serializer, with
`description::decompose_origin` providing the algebraic inverse of
`compose_origin` (URDF intrinsic-Z-Y′-X″, gimbal-pole branch pins
yaw=0 and absorbs residual into roll while preserving the composed
transform).

## Phase VD — gizmo persistence

- ImGuizmo translate / rotate routed through
  `viz::apply_gizmo_target` (pure logic) into the description's
  `joint.origin` / `link.visual_origin` / `motor.visual_origin`.
- `viz::edit_session` driving load / save / reload with a dirty bit.
- File menu: Save / Save As / Reload-from-disk with Discard-changes
  confirmation; Ctrl+S hotkey.

## Surface attachment follow-up

- Added `viz::attachment` pure math in `robosim_viz_core`.
- Edit mode has a two-click Attach command:
  1. `Start attach`
  2. click the moving surface
  3. click the target surface, which applies immediately
- Hover preview shows the surface plane that will be captured.
  Preview planes are translucent cyan overlays.
- Attach modifiers live in the Attach window:
  `Offset m`, `Quarter turns`, and `Flip normal`.
- Attachment writes through the existing `apply_gizmo_target` path,
  so attached links, joints, and motors persist via Save.
- Current primitive support: cylinder caps/sides, box faces, rotation
  arrow boxes, and mesh AABB faces. Kraken X60 mesh picking/attachment
  accounts for the renderer's 180-degree Y rotation so the clickable
  attach bounds match the rendered motor.

Known limits:

- The first selected attachment plane is not yet kept visible after
  click one.
- There is no live ghost preview of the moving object before click
  two.
- No undo stack yet.
- Mesh attachment uses oriented AABB faces, not actual triangle-level
  CAD faces.

## Phase VL — layout persistence

- `viz::resolve_imgui_ini_path` resolves a per-user persistent path
  for ImGui's saved layout (`$XDG_CONFIG_HOME/robosim-viz/imgui.ini`
  → `$HOME/.config/robosim-viz/imgui.ini` → fallback to ImGui's
  default in CWD).
- `viz::apply_default_dock_layout` builds the first-run dock layout
  via `ImGui::DockBuilder*`: Scene top-right, Inspector below it on
  the right column, Gizmo top-left, central viewport bottom-left.
  Subsequent runs restore the user's saved layout.
- Gizmo controls window dropped `NoDocking | NoSavedSettings`, so
  it docks like the other panels. Configure-time grep
  (`cmake/lint_gizmo_window_flags.cmake`) prevents a regression.

## What's next

Live mode and Replay mode (the remaining v0 visualizer modes beyond
Edit). See `docs/VISUALIZER_V0_PLAN.md`.

## References

- `tests/viz/TEST_PLAN.md`
- `tests/description/TEST_PLAN_VC.md`
- `tests/viz/TEST_PLAN_VD.md`
- `tests/viz/TEST_PLAN_VL.md`
