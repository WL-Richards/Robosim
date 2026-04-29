# Visualizer gizmo persistence + save flow (Phase VD) — test plan

**Status:** **`ready-to-implement`** per the `test-reviewer` agent
(rev-2 review). The B4 fixture-path documentation gap the reviewer
flagged as non-blocking has been folded into B4's procedure in this
patch. Implements `docs/VISUALIZER_V0_PLAN.md` Phase VD
(steps VD1–VD5):

- The snapshot builder reads schema-v2 origins (`joint.origin`,
  `link.visual_origin`) and composes them into per-node world
  transforms. (This was deferred from VB — VB shipped with all nodes
  rooted at identity because v1 had no origins. VC added the schema
  fields; VD wires them through the renderer-facing snapshot.)
- A pure-logic apply step takes a target `world_from_local`
  (the matrix ImGuizmo's `Manipulate` writes) and a selected node,
  computes the matching `origin_pose` in the parent frame, and writes
  it back to the in-memory `robot_description`.
- An `edit_session` value-type tracks the loaded description, its
  source path, and a dirty bit; `save_session` / `reload_session`
  drive the file menu's data-layer state machine. The ImGui modal
  itself is exercised by the smoke test, not unit tests.

**Implementer's first action:** read `.claude/skills/visualizer.md`,
`.claude/skills/robot-description-loader.md`, and the VC test plan
(`tests/description/TEST_PLAN_VC.md`); then submit this plan to
`test-reviewer`. Iterate to `ready-to-implement` before writing any
test code. Once the gate passes, write the failing tests as approved
and run them to confirm they fail for the expected reason, then
implement the minimum code to turn them green.

**Review history:**
- rev 1 → `not-ready`. Multiple correctable findings:
  - **B5** setup too symmetric — `elbow.origin = identity` made
    the discriminator weak. rev-2 sets `elbow.origin = (0,1,0)`
    so the right answer (`elbow.world = (0,1,0)`) and the
    visual-frame-leak bug (`elbow.world = (10,1,0)`) split.
  - **B2** missed the child link's rotation propagation; rev-2
    adds an arm-link assertion against `compose_origin(joint.origin)`.
  - **A3 / A4** asserted only on the written origin field; rev-2
    adds a snapshot-rebuild assertion in each so a sign-flipped
    inversion is caught from the world side.
  - **A6** contract was over-stated ("decompose-then-compose
    round-trips exactly for representable origins" — only true
    for the identity-rotation root-joint case). rev-2 narrows
    the contract and drops the misleading "A7 covers the
    non-identity no-op" sentence (no such test exists, and
    shouldn't — non-identity no-op isn't bit-equal because of
    trig).
  - **A2** parameter cases were all clearly away-from-pole;
    rev-2 replaces the small-angle row with a near-pole-but-not-at-
    pole case (pitch = `+π/2 - 0.05`) to pin ill-conditioned-but-
    not-pole behavior.
  - **D5** pinned only "dirty=true is preserved through reload-
    failure," not "dirty=false is preserved." rev-2 adds D5b
    (clean session, file corrupted, reload fails, dirty stays
    false).
  - **S3** covered joint apply only; rev-2 adds S3b (apply on
    link, assert `visual_origin` key emitted and `origin` key
    absent on the joint).
  - **S5** title overpromised ("adversarial floats round-trip
    bit-equal"); only the translation path was tested. rev-2
    renames to clarify the translation-only scope and adds a
    rotation-side complement test (S5b) with the correct
    composed-transform tolerance instead of bit-equal.
  - **A4b** added — rotation on a link node was uncovered.
  - **D0 / D0b** added — `load_session` constructor contract
    and load-failure path were uncovered.
  - **D3** strengthened with an explicit reload-equality
    assertion at step 3.
  - **O2 / O5 / A8** prose tightened: O2 swaps to exact-π-fraction
    inputs; O5 / A8 spell out the `validate_finite_*` calling
    pattern (`.has_value() == false` ⇒ all finite).
  - **find_node / test_id** mechanics now spelled out so the
    implementer doesn't guess.
  - **Out-of-range `selected_index`** explicitly documented as
    UB on the public surface; ASan catches misuse, no runtime
    test added.
  - Reviewer endorsed all four design choices the author flagged
    (decompose location in `description::`, apply as free
    function, gimbal-pole "preserve composed transform" policy,
    reload-failure leaves dirty=true). Carried forward unchanged.

---

## Why this phase exists

Phase VB shipped the read-side with all `scene_node::world_from_local`
values fixed at identity, because schema v1 had no origin transforms
to compose. ImGuizmo translate / rotate handles in `src/viz/main.cpp`
mutate the in-memory `world_from_local` directly today, but those
mutations are throwaway: they don't reach the description, and they
get clobbered the next time `build_edit_mode_snapshot` rebuilds the
snapshot.

Phase VC added `link.visual_origin` and `joint.origin` to the schema,
plus a `description::save_to_file` round-trip serializer. Phase VD
closes the loop: the snapshot builder honors v2 origins, gizmo deltas
flow back into the description, and the user can save the result to
disk and reload it.

The two-stage write path is deliberate: `apply_gizmo_target` is pure
logic over `(description, snapshot, selected_index, target)` and is
independent of ImGuizmo. The ImGui-side wiring (gizmo, modal,
hotkeys) is a thin caller that delegates the data work to the pure
logic. That seam is the only way the math is unit-testable; without
it, the contract sits behind ImGuizmo's per-frame `Manipulate` call
and only the smoke test can reach it.

---

## Determinism note (visible to test-reviewer)

The visualizer is exempt from sim-core determinism rules
(`.claude/skills/visualizer.md` "Determinism exception"). Tests under
`tests/viz/` may use `std::chrono::system_clock` /
`std::chrono::steady_clock` / `std::random_device` if a real reason
arises (none does for this plan). Stating it here so the reviewer does
not flag it as a smell.

The tests themselves remain deterministic — same input, same output,
every run. No PRNG is introduced; adversarial parameter spreads are
hand-authored (mirrors VC's S5 policy).

---

## Conventions (carried over from VB / VC; new pins called out)

The VB plan pins the rendering-side conventions
(`tests/viz/TEST_PLAN.md` "Conventions" #1–#12). All carry forward.
The VC plan pins the schema-side conventions (D-VC-1 … D-VC-11). All
carry forward. New convention pins specific to VD:

13. **Kinematic frame vs. visual frame.** Each link / joint has two
    distinct frames in the snapshot builder's bookkeeping:
    - **Kinematic frame** — the chained frame used as the composition
      base for descendants. For a joint, this is
      `parent_link_kinematic * compose_origin(joint.origin)`. For a
      link, this is `parent_joint_kinematic` (a link does not move
      relative to its parent joint in Edit mode — joint motion is
      Live-mode-only).
    - **Visual frame** — the frame the renderer draws the primitive
      in. For a joint, this equals the joint's kinematic frame. For
      a link, this is `link_kinematic * compose_origin(link.visual_origin)`.
    - The snapshot's `scene_node::world_from_local` is the **visual
      frame** (what the renderer needs). The kinematic frame is
      builder-internal; descendants compose against the parent's
      kinematic frame, not the parent's visual frame. This
      distinction is load-bearing — pinned by B5.

14. **`world` parent maps to identity kinematic frame.** A joint with
    `parent: "world"` has `parent_link_kinematic == identity`. (Carries
    VB convention "world maps to nullopt parent_index" through to the
    composition rule.)

15. **Apply input is a target world transform, not a delta.**
    `apply_gizmo_target` takes the *new* `world_from_local` value
    (the matrix ImGuizmo writes after `Manipulate`), not a delta from
    the prior value. ImGuizmo's `Manipulate` always returns the new
    full model matrix; consuming it as a delta would force every
    caller to remember the prior matrix. The apply step inverts the
    parent kinematic frame internally to recover the parent-relative
    `origin_pose`.

16. **Apply pins the composed transform, not the rpy_rad values.**
    The Euler decomposition is non-unique at the gimbal-lock pole
    (`pitch == ±π/2`). The contract `apply_gizmo_target` honors is:
    after apply, `compose_origin(updated_origin)` reproduces the
    target's parent-relative transform within `1e-9`. Tests assert
    on *recomposed transforms*, not on raw rpy_rad components,
    except when the case is unambiguously away from the pole and a
    direct rpy_rad assertion adds value (A1, A3 do this — both at
    `pitch == 0`). The gimbal-lock corner is pinned by A8 below.

---

## Decisions pinned (VD-specific)

These are committed-to during the test-reviewer cycle and are the
contract VD honors. The full rationale lives in this document; what's
listed here is what implementers and consumers internalize.

1. **D-VD-1.** The snapshot builder is the single composition point.
   It honors `joint.origin` and `link.visual_origin`; no other v0
   producer of `scene_snapshot` exists. Live / Replay producers are
   post-v0; they will compose the same way but feed in joint angles
   from a state stream. Pinned by B1–B7.

2. **D-VD-2.** Apply is pure: `apply_gizmo_target` is a free function
   over `(robot_description, scene_snapshot, std::size_t,
   transform) -> robot_description`. It does not mutate its inputs;
   the caller swaps the returned description into the session.
   (Rationale: pure simplifies testing, and the session struct's
   dirty bit is then a derived fact — it's set only when the caller
   chooses to install the new description, which it always does in
   v0 but a Future "preview without commit" flow could opt out.)
   Pinned by A5/A6's "unrelated fields preserved" assertions and the
   A6 "no-op apply round-trips equal" assertion.

3. **D-VD-3.** Decompose lives next to compose. A new
   `description::decompose_origin(transform_4x4) -> origin_pose`
   helper is added to `src/description/origin_pose.{h,cpp}` as the
   inverse of `compose_origin`. They share the URDF
   intrinsic-Z-Y'-X'' Euler convention (D-VC-5a) and the
   declaration-order rpy element layout. Pinned by O1–O5.

4. **D-VD-4.** Decompose-then-compose is identity within `1e-12` away
   from the gimbal pole; non-unique at the pole. The apply path
   tolerates non-uniqueness by asserting on recomposed transforms
   per convention #16. Pinned by O3 (round-trip away from the pole)
   and O5 (round-trip at the pole — composed transform matches even
   though rpy_rad does not).

5. **D-VD-5.** `edit_session` is the data-layer view of "what's
   loaded, where it came from, and whether it's been edited since
   the last save." It carries `description`, `source_path`, and
   `dirty`. `apply_gizmo_target` does not touch the session; the
   caller in `main.cpp` swaps the new description in and sets
   `dirty = true`. `save_session` clears `dirty` on success.
   `reload_session` clears `dirty` on success. The "discard
   unsaved changes?" modal is **not** part of the session's
   contract — the caller checks `dirty` before invoking
   `reload_session` and shows the modal in ImGui. Pinned by D1–D4.

6. **D-VD-6.** Save reuses `description::save_to_file` unchanged.
   No new save error kinds, no new format. The serializer's D-VC-9
   (identity origins omitted) is preserved through apply: an apply
   that decomposes back to identity origins emits an origin-key-free
   file. Pinned by S2.

7. **D-VD-7.** Save As is "change `source_path`, then save." There
   is no separate `save_as_session` — the caller mutates
   `session.source_path` and calls `save_session`. This avoids a
   redundant API surface and pins that the dirty-bit semantics are
   identical between Save and Save As. Pinned implicitly by D1
   (the Save path) — Save As needs no new test.

8. **D-VD-8.** Reload from disk replaces the session's description
   wholesale. There is no merging of unsaved changes. The "Discard
   changes?" modal is the *only* affordance protecting unsaved work;
   if the user confirms discard, `reload_session` runs unconditionally
   and overwrites the in-memory state. Pinned by D2.

9. **D-VD-9.** Save propagates `description::save_to_file`'s
   `save_error` arm to the caller without retrying or transforming.
   `dirty` is **not** cleared on failure — the user's edits are still
   in memory and unsaved. Pinned by S4.

10. **D-VD-10.** Reload propagates `description::load_from_file`'s
    `load_error` arm to the caller without retrying or transforming.
    On failure, the session's `description`, `source_path`, and
    `dirty` are unchanged (the on-disk file is broken; the user's
    in-memory state is the only authoritative copy). Pinned by D5.

---

## Public surface under test

```cpp
// --- src/description/origin_pose.h (extended) -----------------------

namespace robosim::description {

// Inverse of compose_origin. Extracts xyz_m from the translation
// column and rpy_rad via intrinsic-Z-Y'-X'' Euler decomposition.
//
// At the gimbal pole (|m[0][2]| within 1 - 1e-12 of unity) the
// decomposition is non-unique; this function returns one valid
// solution (yaw == 0 by convention; roll absorbs the residual
// rotation). Callers that round-trip via compose_origin recover the
// same transform within 1e-12; callers that compare rpy_rad
// component-wise must be aware of the gimbal-pole branch.
//
// Returns the pose such that compose_origin(returned) reproduces the
// rotation and translation of `m` within 1e-12 (away from the pole;
// see A8 / O5 for the at-pole contract).
[[nodiscard]] origin_pose decompose_origin(const transform_4x4& m);

}  // namespace robosim::description

// --- src/viz/edit_mode_apply.h (new) ----------------------------------

namespace robosim::viz {

// Apply a gizmo-produced target world transform to the selected node's
// origin. Returns the description with the appropriate origin field
// (joint.origin for joint nodes, link.visual_origin for link nodes)
// updated so that build_edit_mode_snapshot(returned) places the
// selected node at target_world_from_local within 1e-9.
//
// Preconditions (caller's responsibility):
//   - selected_index < snapshot.nodes.size();
//   - snapshot was built from `source` via build_edit_mode_snapshot.
//
// Behavior is undefined when preconditions are violated. ASan/UBSan
// catch out-of-range access; debug builds may add an assert(). v0
// does not pin a runtime test for the out-of-range case — the
// caller (main.cpp) only invokes apply when ImGuizmo's IsUsing()
// returns true, which implies a current selection, which implies
// a valid index.
//
// Does not mutate `source` or `snapshot`. Other description fields
// (motors, sensors, mass_kg, every other link/joint) are copied
// through unchanged.
[[nodiscard]] description::robot_description apply_gizmo_target(
    const description::robot_description& source,
    const scene_snapshot& snapshot,
    std::size_t selected_index,
    const transform& target_world_from_local);

}  // namespace robosim::viz

// --- src/viz/edit_session.h (new) -------------------------------------

namespace robosim::viz {

struct edit_session {
  description::robot_description description;
  std::filesystem::path source_path;
  bool dirty = false;
};

// Load `path` and wrap into a fresh edit_session with dirty == false.
[[nodiscard]] std::expected<edit_session, description::load_error>
load_session(const std::filesystem::path& path);

// Write session.description to session.source_path via
// description::save_to_file. On success, clears session.dirty. On
// failure, propagates the save_error and leaves session unchanged.
[[nodiscard]] std::expected<void, description::save_error>
save_session(edit_session& session);

// Reload session.description from session.source_path. On success,
// replaces session.description with the on-disk content and clears
// session.dirty. On failure, propagates the load_error and leaves
// session.description / session.source_path / session.dirty
// unchanged.
[[nodiscard]] std::expected<void, description::load_error>
reload_session(edit_session& session);

}  // namespace robosim::viz
```

The `scene_snapshot` / `scene_node` / `transform` types are unchanged
from VB. The builder's signature is unchanged; its **behavior** is
extended (composition now reads v2 origins).

---

## Tolerance policy

Inherits VB's tolerance tiers (`tests/viz/TEST_PLAN.md` "Tolerance
policy"):

- **Exact equality** for outputs of pure copy / single multiply with
  representable inputs (B7 identity-origin pass-through; A5 unrelated
  fields).
- **Absolute `1e-12`** for one matmul of representable inputs (B1
  translation-only composition; O3 decompose-then-compose round-trip
  away from the pole).
- **Absolute `1e-9`** for ≤ 3 matmuls or compose / decompose / compose
  chains with non-trivial rotation (B2 rotation composition; A2 / A7
  apply round-trips; F1 end-to-end).
- **Tighter** when the test has a special reason; **looser** never
  without an inline justification.

---

## Test fixture mechanics

Reuses VB's `tests/viz/fixtures.{h,cpp}` helpers
(`make_v0_arm_description`, `make_two_joint_chain_description`,
`make_long_named_description`). All three already produce
`schema_version = 2` with identity origins (per the VC patch
deliverables).

Two new fixture helpers in `tests/viz/fixtures.{h,cpp}` for v2
origin scenarios:

- `make_v0_arm_with_joint_origin(const description::origin_pose&)` —
  v0-arm with `joints[0].origin` set to the argument. Used by B1 /
  B2 / A1 / A2 / A6.
- `make_two_joint_chain_with_origins(const description::origin_pose&
  shoulder_origin, const description::origin_pose& elbow_origin,
  const description::origin_pose& arm_visual,
  const description::origin_pose& forearm_visual)` —
  two-joint chain with all four origins explicitly set. Used by
  B3 / B5 / B6 / A3 / A4.

The on-disk save / reload tests (S1–S5b, F1) use a per-test temp
file. The path is derived from the running GoogleTest test name:
```cpp
const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
const auto temp_path = std::filesystem::temp_directory_path() /
    std::format("vd_test_{}_{}.json", info->test_suite_name(), info->name());
```
Each test removes its temp file in a `TearDown` or RAII guard so
parallel test runs don't collide. No new shared fixture machinery.

Test-side helpers reused from VB:
- `find_node(snapshot, kind, name) -> const scene_node&` lives in
  `tests/viz/edit_mode_builder_test.cpp`'s anonymous namespace today
  (with `EXPECT_NE` against `end()`). For VD it migrates to
  `tests/viz/fixtures.{h,cpp}` so all VD test files can share it
  without duplication. Signature pinned:
  ```cpp
  [[nodiscard]] const scene_node& find_node(
      const scene_snapshot& s, node_kind kind, std::string_view name);
  [[nodiscard]] std::size_t find_node_index(
      const scene_snapshot& s, node_kind kind, std::string_view name);
  ```
  Both fail the test (`ADD_FAILURE` then return the first node /
  index 0 sentinel) when no matching node exists. Section A tests
  use `find_node_index` to look up the `selected_index` argument.

---

## Test layout

- New test files:
  - `tests/description/origin_pose_decompose_test.cpp` — section O.
  - `tests/viz/edit_mode_builder_v2_test.cpp` — section B.
  - `tests/viz/edit_mode_apply_test.cpp` — section A.
  - `tests/viz/edit_session_test.cpp` — sections D, S.
  - `tests/viz/vd_e2e_test.cpp` — section F.
- Suite names: `OriginPose`, `EditModeBuilder` (extends VB),
  `EditModeApply`, `EditSession`, `VdEndToEnd`.
- GoogleTest. `INSTANTIATE_TEST_SUITE_P` for parameterized cases
  (O1, A2, S5).
- Section A's apply tests assert via the round-trip rule per
  convention #16: they call `apply_gizmo_target`, then either rebuild
  the snapshot and assert on the new `world_from_local` (preferred —
  this is the user-observable contract) or `compose_origin` the
  resulting `origin_pose` and assert on the composed transform. Tests
  do **not** assert on `decompose_origin` outputs directly except in
  section O.

---

## O. Decompose helper (`description::decompose_origin`)

### O1. `OriginPose.DecomposeReproducesXyzFromTranslationColumn` (parameterized)

- Layer / contract: D-VD-3 — translation extraction is the pure copy
  of the matrix's translation column.
- Bug class: an implementation that extracts translation from the
  inverse-rotation-applied position (mistaking `m[3][.]` for body-
  frame position) silently breaks for any non-identity rotation.
- Parameters (5 cases, all with rotation = identity to isolate the
  translation path):
  - `xyz = (0, 0, 0)` → identity transform.
  - `xyz = (1, 0, 0)` → pure +X translation.
  - `xyz = (0, -2.5, 0)` → pure -Y translation.
  - `xyz = (0.1, 0.2, 0.3)` → small triple.
  - `xyz = (-1e3, 1e3, 1e3)` → large symmetric triple.
- Procedure: build `m = compose_origin({xyz, rpy=(0,0,0)})`; call
  `decompose_origin(m)`; assert `result.xyz_m == xyz` exactly.
- Tolerance: exact (representable inputs, single copy).

### O2. `OriginPose.DecomposeReproducesRpyForSingleAxisRotations` (parameterized)

- Layer / contract: D-VD-3 — rotation extraction matches URDF
  intrinsic-Z-Y'-X''.
- Bug class: an implementation that swaps the rpy element order
  (e.g. maps `rpy_rad[0]` to yaw instead of roll, mirroring VC V5
  case D's discriminator) silently rotates the inverse direction.
- Parameters (6 cases, each isolating one of the three axes at
  `±M_PI / 6.0` — a non-trivial angle that's far from both 0 and
  the pole, written as exact π fractions so the input value is the
  unrounded `M_PI / 6.0`, not a 4-digit decimal that's already lost
  precision before the test starts):
  - `rpy = (+M_PI / 6.0, 0, 0)` → pure roll.
  - `rpy = (-M_PI / 6.0, 0, 0)` → pure roll, negative.
  - `rpy = (0, +M_PI / 6.0, 0)` → pure pitch.
  - `rpy = (0, -M_PI / 6.0, 0)` → pure pitch, negative.
  - `rpy = (0, 0, +M_PI / 6.0)` → pure yaw.
  - `rpy = (0, 0, -M_PI / 6.0)` → pure yaw, negative.
- Procedure: `m = compose_origin({xyz=(0,0,0), rpy})`; call
  `decompose_origin(m)`; assert `result.rpy_rad == rpy` per element.
- Tolerance: `1e-12` per element. The cited path is one forward
  trig (compose) + one inverse trig (decompose), each rounding
  once to nearest-even on a value that fits inside double's mantissa
  with margin — empirical worst case for `compose(decompose(x)) - x`
  on this kind of input is ~2 ULP, leaving `1e-12` (~4500 ULP at
  `0.5`) with plenty of headroom for future implementation tweaks.

### O3. `OriginPose.DecomposeThenComposeRoundTripsAwayFromPole` (parameterized)

- Layer / contract: D-VD-4 — the round-trip identity holds away from
  the gimbal pole.
- Bug class: a sign error or transposed-matrix decomposition that
  produces a different rotation under recompose; an off-by-one in
  rpy element order that survives single-axis cases (O2) but breaks
  on combined rotations.
- Parameters (4 cases, each combining all three axes at non-trivial
  values — these are the cases VC V5 uses to discriminate
  intrinsic-Z-Y'-X'' from intrinsic-X-Y'-Z''):
  - `rpy = (+π/4, +π/6, +π/3)` (each axis distinct, all positive).
  - `rpy = (-π/4, +π/6, -π/3)` (mixed signs).
  - `rpy = (+π/4, -π/6, +π/3)` (negative pitch, well below the
    pole).
  - `rpy = (+0.1, +0.2, +0.3)` (small angles — distinct case from
    the multiples-of-π values, exercises the small-angle path).
- Procedure: `m = compose_origin({xyz=(0.1, -0.2, 0.3), rpy})`; call
  `decompose_origin(m)`; recompose
  `m_round = compose_origin(decompose_origin(m))`; assert each of the
  16 elements of `m` and `m_round` agree within `1e-12`.
- Tolerance: `1e-12` per element (decompose -> compose with
  representable inputs, no FP catastrophic cancellation).
- Notes: assertions are on the **recomposed transform**, not on
  individual rpy_rad components, per convention #16. (For these four
  cases the rpy_rad do round-trip exactly, but pinning the contract
  on the recomposed transform is what generalizes to A and F.)

### O4. `OriginPose.DecomposeIdentityYieldsZeroPose`

- Layer / contract: positive-degenerate case. The identity transform
  must decompose to `xyz = (0,0,0), rpy = (0,0,0)`.
- Bug class: an implementation that returns `rpy = (NaN, NaN, NaN)`
  for the identity input because `atan2(0, 0)` was used unguarded
  on a path that doesn't reach. (Standard `atan2(0, 0) == 0`, but
  not every implementation language guarantees that — pin it.)
- Procedure: `m = transform_4x4{}` initialized to identity (1 on
  diagonal, 0 elsewhere — explicit construction to avoid relying on
  `compose_origin({})`); call `decompose_origin(m)`; assert
  `result.xyz_m == (0,0,0)` and `result.rpy_rad == (0,0,0)` exactly.
- Tolerance: exact.

### O5. `OriginPose.DecomposeAtGimbalPolePreservesComposedTransform`

- Layer / contract: D-VD-4 — at the gimbal pole, the rpy_rad output
  is non-unique but `compose_origin(decomposed)` matches the input
  rotation.
- Bug class: a decomposition that returns `NaN` at the pole (because
  `asin(±1)` evaluates fine but `atan2(0, 0)` is hit on the
  follow-up); a decomposition that silently returns a different
  rotation at the pole.
- Procedure: build `m = compose_origin({xyz=(0,0,0),
  rpy=(+0.4, +M_PI / 2.0, -0.3)})` (pitch exactly at the positive
  pole; roll and yaw arbitrary). Call `decompose_origin(m)`;
  recompose `m_round`; assert each of the 16 elements of `m` and
  `m_round` agree within `1e-9` (looser than O3's `1e-12` because
  the at-pole branch evaluates `atan2` on residuals where the
  small `atan2` arguments accumulate FP noise from the
  `R_z · R_y · R_x` matrix elements — the conditioning isn't bad,
  but it's not "single rounding" either).
- Additional assertion — finite output:
  ```cpp
  const origin_pose decomposed = decompose_origin(m);
  EXPECT_FALSE(validate_finite_xyz(decomposed.xyz_m, "/test/xyz_m"));
  EXPECT_FALSE(validate_finite_rpy(decomposed.rpy_rad, "/test/rpy_rad"));
  ```
  The `validate_finite_*` helpers (D-VC-3, `src/description/origin_pose.h`)
  return `std::optional<load_error>`. `std::nullopt` (i.e.
  `.has_value() == false`) means **all elements are finite**;
  any populated return is the first non-finite element. The
  `base_pointer` strings here are throwaway — only the populated/
  empty discriminant is used. (A small `is_finite_pose(origin_pose)`
  predicate would be cleaner but is not worth a new function for
  two test sites; reuse the existing helper.)
- Tolerance: `1e-9` per element of recomposed transform.
- Notes: the test does **not** assert on the rpy_rad values
  themselves — they are intentionally underdetermined at the pole.
  The contract is that the recomposed transform agrees, and that
  the output is finite (not NaN-poisoned).

---

## B. Snapshot builder reads v2 origins

### B1. `EditModeBuilder.JointOriginTranslationPropagatesToWorldFromLocal`

- Layer / contract: D-VD-1 — the builder honors `joint.origin` for a
  joint whose parent is `world`.
- Bug class: the builder ignores `joint.origin` and produces identity
  for every joint (the VB pre-VD behavior). Without this test, the
  pre-VD identity behavior would still pass every VB assertion.
- Procedure: build a description via `make_v0_arm_with_joint_origin(
  origin_pose{xyz=(0.5, 0, 0), rpy=(0,0,0)})`. Call
  `build_edit_mode_snapshot`. Locate the `shoulder` joint node.
- Expected: `shoulder.world_from_local.m[3][0] == 0.5`,
  `m[3][1] == 0.0`, `m[3][2] == 0.0`, all other off-diagonals
  exactly `0.0`, diagonals exactly `1.0`.
- Tolerance: exact.

### B2. `EditModeBuilder.JointOriginRotationMatchesComposeOriginOutput`

- Layer / contract: D-VD-1 — the builder uses `compose_origin` for
  rotation composition, not a wrong-convention impl.
- Bug class: the builder rolls its own Euler-to-matrix code that
  disagrees with `compose_origin` (e.g., uses
  `R = R_x · R_y · R_z` instead of `R = R_z · R_y · R_x`); the
  snapshot then disagrees with the apply path's inverse, and the
  whole VD round-trip drifts.
- Procedure: build via `make_v0_arm_with_joint_origin(origin_pose{
  xyz=(0,0,0), rpy=(+M_PI/4.0, +M_PI/6.0, +M_PI/3.0)})`. Build
  snapshot. Locate `shoulder` joint and `arm` link. Compute the
  expected matrix `m_expected = compose_origin({xyz=(0,0,0),
  rpy=(+M_PI/4.0, +M_PI/6.0, +M_PI/3.0)})` directly.
- Expected:
  - Each of the 16 elements of `shoulder.world_from_local.m` and
    `m_expected` agree within `1e-12` (joint kinematic = composed
    origin, since parent is world).
  - Each of the 16 elements of `arm.world_from_local.m` and
    `m_expected` agree within `1e-12` (the arm link's
    `visual_origin` is identity in this fixture, so the child
    link's full visual world transform inherits the parent
    joint's kinematic — i.e. the parent's full rotation, not just
    its translation). This second assertion catches a builder
    that propagates joint *translation* to the child link but
    drops joint *rotation* — a bug that B3's translation-only
    fixture would not detect.
- Tolerance: `1e-12` per element (one trig + one matmul each).

### B3. `EditModeBuilder.ChildLinkInheritsJointKinematicFrame`

- Layer / contract: convention #13 — a child link's snapshot
  `world_from_local` starts from the parent joint's *kinematic*
  frame.
- Bug class: the builder treats the child link's snapshot
  `world_from_local` as identity rather than inheriting the joint's
  composition; or treats it as the joint's *visual* frame (which
  for a joint equals the kinematic frame, so this confusion only
  surfaces under B5).
- Procedure: build via `make_v0_arm_with_joint_origin(origin_pose{
  xyz=(0.5, 0, 0), rpy=(0,0,0)})` (joint translated in +X; arm
  link's `visual_origin` left at identity by the helper). Build
  snapshot.
- Expected: the `arm` link node's
  `world_from_local.m[3][0] == 0.5` (inherits joint translation),
  `m[3][1] == 0.0`, `m[3][2] == 0.0`, rotation block is identity.
- Tolerance: exact.

### B4. `EditModeBuilder.LinkVisualOriginOffsetsLinkWorldFromLocal`

- Layer / contract: D-VD-1 — the builder honors
  `link.visual_origin` for the link's snapshot transform.
- Bug class: the builder ignores `link.visual_origin` (the field is
  loaded into `description::link::visual_origin` but never read at
  the snapshot-builder use-site, mirroring the pre-VD behavior).
- Procedure: start from `make_v0_arm_description()` (joint and
  link visual origins both default to identity), then mutate
  in-place: `desc.links[0].visual_origin = origin_pose{xyz=(0.1,
  0, 0), rpy=(0,0,0)}`. (No new fixture helper for v0; the
  rev-2 helpers `make_v0_arm_with_joint_origin` and
  `make_two_joint_chain_with_origins` cover the joint-origin and
  multi-node cases respectively, but the link-visual-only
  single-arm case is rare enough that an in-test mutation is
  cleaner than a third helper.) Build snapshot.
- Expected: the `arm` link node's
  `world_from_local.m[3][0] == 0.1` (purely from visual_origin,
  with joint at identity), other translation components and
  rotation block as in B3.
- Tolerance: exact.

### B5. `EditModeBuilder.LinkVisualOriginDoesNotPropagateToDescendantJointKinematicFrame`

- Layer / contract: convention #13 — descendants compose against the
  parent's *kinematic* frame, not its *visual* frame.
- Bug class: the builder uses a single `world_from_local` field both
  to position the visual and to anchor descendants. A non-identity
  `visual_origin` then drags every descendant joint along with the
  visual offset, which is wrong: the visual offset is purely
  cosmetic.
- Procedure: build via `make_two_joint_chain_with_origins(
  shoulder_origin = identity,
  elbow_origin = origin_pose{xyz=(0, 1, 0), rpy=(0,0,0)},
  arm_visual = origin_pose{xyz=(10, 0, 0), rpy=(0,0,0)},  // huge visual offset on arm
  forearm_visual = identity)`.
  Build snapshot. The non-identity `elbow_origin` is the
  rev-2 strengthening: a builder that mistakenly uses arm's
  *visual* frame as elbow's parent base would compose
  elbow.origin against `(10, 0, 0)` and yield `elbow.world =
  (10, 1, 0)`, while the correct answer (parent kinematic =
  identity) yields `elbow.world = (0, 1, 0)`. Without a
  non-identity elbow.origin, both right and wrong answers
  collapse to identity in the elbow row.
- Expected:
  - `arm.world_from_local.m[3]` columns: `(10, 0, 0, 1)` (arm's
    visual is shifted by the visual_origin).
  - `elbow.world_from_local.m[3]` columns: `(0, 1, 0, 1)`
    (elbow's parent kinematic frame is arm's kinematic frame =
    identity, NOT arm's visual frame which is at +10X; the
    elbow's own origin xyz `(0, 1, 0)` is composed against
    identity to yield `(0, 1, 0)`).
  - `forearm.world_from_local.m[3]` columns: `(0, 1, 0, 1)`
    (forearm's link kinematic = elbow's kinematic = `(0, 1, 0)`;
    forearm's visual_origin is identity, so it inherits the
    elbow's kinematic). This row also pins that arm.visual does
    not propagate two levels down to forearm even when the
    intermediate elbow has a non-identity origin.
  - All rotation blocks are identity.
- Tolerance: exact (representable inputs throughout).

### B6. `EditModeBuilder.JointChainComposesOriginsLeftToRight`

- Layer / contract: D-VD-1 + convention #13 — composition order is
  `parent_kinematic * compose_origin(child.origin)`, applied in
  parent-to-child order.
- Bug class: a builder that composes in the wrong order (e.g.,
  `compose_origin(child.origin) * parent_kinematic`) produces wrong
  positions for non-commutative chains; for translation-only chains
  the answer happens to coincide, so this test uses translation
  along distinct axes to make the chain order observable from
  positions alone (avoiding rotation noise).
- Procedure: build via `make_two_joint_chain_with_origins(
  shoulder_origin = origin_pose{xyz=(1, 0, 0), rpy=(0,0,0)},
  elbow_origin = origin_pose{xyz=(0, 1, 0), rpy=(0,0,0)},
  arm_visual = identity, forearm_visual = identity)`. Build
  snapshot.
- Expected:
  - `shoulder.world_from_local.m[3]` columns: `(1, 0, 0, 1)` (joint
    1's kinematic frame).
  - `arm.world_from_local.m[3]` columns: `(1, 0, 0, 1)` (arm
    inherits joint 1's kinematic).
  - `elbow.world_from_local.m[3]` columns: `(1, 1, 0, 1)`
    (parent_kinematic = `(1,0,0)`; elbow's origin xyz = `(0,1,0)`;
    composed translation = `(1,1,0)`).
  - `forearm.world_from_local.m[3]` columns: `(1, 1, 0, 1)`
    (forearm inherits elbow's kinematic).
  - All rotation blocks are identity.
- Tolerance: exact (representable inputs, translation-only chain).

### B7. `EditModeBuilder.IdentityOriginsYieldIdentityWorldFromLocal`

- Layer / contract: forward-compat — the v0-arm production fixture
  has no origin keys; the builder must continue to produce identity
  `world_from_local` for every node. Without this test, a future
  change to the composition path could regress the existing VB
  E2 contract silently.
- Bug class: a refactor that always composes origin into
  `world_from_local` even when the origin is identity, but
  introduces a tiny `1e-15` drift via FP error. Existing VB tests
  use `EXPECT_EQ` on identity components and would catch the drift,
  but only on cylinder dimensions — not on every node's full
  transform.
- Procedure: load `descriptions/v0-arm.json` via
  `description::load_from_file`. Build snapshot.
- Expected: every node's `world_from_local == transform::identity()`
  (`operator==`, exact).
- Tolerance: exact (the v0-arm has identity origins; `compose_origin
  ({xyz=(0,0,0), rpy=(0,0,0)})` is exactly the identity matrix
  because `cos(0) == 1.0` and `sin(0) == 0.0` in IEEE-754).

---

## A. `apply_gizmo_target` (the gizmo write path)

### A1. `EditModeApply.AppliesPureTranslationToRootJointOrigin`

- Layer / contract: D-VD-2 — apply at a root joint with identity
  current origin and a pure-translation target writes the joint's
  origin xyz_m and leaves rpy_rad at zero.
- Bug class: apply writes to `links[].visual_origin` instead of
  `joints[].origin` (or vice versa); apply mis-handles the
  parent-frame inversion at root (where parent_kinematic is
  identity, so inversion is a no-op — easiest case to get right).
- Procedure: source = v0-arm with identity origins. Snapshot built
  from source. selected_index = the `shoulder` joint node's index
  (looked up via `find_node`). target = `transform` with translation
  column `(0.7, 0, 0, 1)`, identity rotation.
- Expected: returned description's `joints[0].origin.xyz_m ==
  (0.7, 0, 0)`, `rpy_rad == (0, 0, 0)`. Exact for both
  (representable input, identity rotation, identity parent
  kinematic).

### A2. `EditModeApply.AppliesRotationViaCompositionRoundTrip` (parameterized)

- Layer / contract: convention #16 — apply pins the *composed
  transform*, not the rpy_rad components.
- Bug class: apply uses a wrong-convention decompose (e.g.
  intrinsic-X-Y'-Z'' instead of URDF Z-Y'-X''). The rpy_rad output
  would round-trip via the wrong-convention compose but not match
  the input target via `compose_origin`.
- Parameters (4 cases, mirroring VC V5 + a near-pole case):
  - `target = compose_origin({xyz=(0,0,0),
    rpy=(+M_PI/4.0, +M_PI/6.0, +M_PI/3.0)})`.
  - Same with `rpy=(-M_PI/4.0, +M_PI/6.0, -M_PI/3.0)`.
  - Same with `rpy=(+M_PI/4.0, -M_PI/6.0, +M_PI/3.0)`.
  - **Near-pole case:** `rpy=(0, +M_PI/2.0 - 0.05, 0)` —
    pitch sits 0.05 rad (~2.9°) below the positive pole. The
    `decompose_origin` away-from-pole branch must still handle
    this correctly (the pole branch is gated by `|m[0][2]| > 1
    - 1e-12`, which is far tighter than `|sin(M_PI/2 - 0.05)|
    ≈ 0.99875`). Replaces rev-1's `(+0.1, +0.2, +0.3)` row,
    which only exercised the easy-conditioning case.
- Procedure: source = v0-arm with identity origins. selected_index =
  `shoulder` joint. Apply target. Take the resulting description's
  `joints[0].origin` and recompose: `m_round =
  compose_origin(updated_origin)`. The target was constructed by
  `compose_origin(input_pose)`; assert each of the 16 elements of
  `target.m` and `m_round` agree within `1e-9`.
- Tolerance: `1e-9` per element (compose -> apply (decompose) ->
  compose; tier "≥ 3 matmuls / trig" per VB tolerance policy).

### A3. `EditModeApply.AppliesAtChildJointInvertsParentKinematic`

- Layer / contract: D-VD-2 — apply at a non-root joint inverts the
  parent kinematic frame to recover the parent-relative origin.
- Bug class: apply uses world-frame target as the new origin
  directly, ignoring that joint origins are in the parent link's
  frame. For a non-trivial parent kinematic, this places the joint
  in the wrong absolute position.
- Procedure: source = `make_two_joint_chain_with_origins(
  shoulder_origin = origin_pose{xyz=(1, 0, 0), rpy=(0,0,0)},
  elbow_origin = identity,
  arm_visual = identity, forearm_visual = identity)`. Snapshot
  built from source. selected_index = `elbow` joint. target =
  translation `(1.5, 0, 0)`, identity rotation.
- Expected:
  - Returned description's `joints[1].origin.xyz_m` is
    `(0.5, 0, 0)` (the parent kinematic for elbow is shoulder's
    kinematic = `(1, 0, 0)`; inverse-translate the target to get
    `(0.5, 0, 0)` in parent frame). `joints[1].origin.rpy_rad ==
    (0, 0, 0)`. Exact for both.
  - **Rebuild assertion (rev 2):** rebuild the snapshot from the
    returned description; the elbow's
    `world_from_local.m[3][0] == 1.5` exactly (and
    `m[3][1] == m[3][2] == 0` exactly, rotation block identity).
    Without this assertion, a sign-flipped inversion that wrote
    `joint.origin.xyz_m = (-0.5, 0, 0)` would also pass the field-
    level assertion if the test happened to assert on a wrong
    expected value. The rebuild check pins what the user sees in
    the viewport, which is the contract that matters.
- Notes: this is the test that proves the parent-inversion math is
  there. Without parent inversion, the resulting xyz_m would be
  `(1.5, 0, 0)` and the elbow would render at world `(2.5, 0, 0)`
  on the next snapshot rebuild (compounding shoulder + elbow) —
  caught by either the field assertion or the rebuild assertion.

### A4. `EditModeApply.AppliesAtLinkInvertsLinkKinematicToWriteVisualOrigin`

- Layer / contract: D-VD-2 — apply at a link node writes
  `link.visual_origin` (not `joint.origin`), and inverts the link's
  kinematic frame to recover the link-relative visual offset.
- Bug class: apply at a link writes to the parent joint's origin
  (confusing the kinematic chain with the visual chain); apply uses
  world-frame target as visual_origin directly, ignoring that
  visual_origin is in the link's own frame.
- Procedure: source = `make_v0_arm_with_joint_origin(origin_pose{
  xyz=(0.5, 0, 0), rpy=(0,0,0)})`. Snapshot built from source.
  selected_index = `arm` link. target = translation `(0.75, 0, 0)`,
  identity rotation. (Powers-of-2 throughout — 0.5, 0.75, 0.25 are
  bit-exact in IEEE-754, so the inverse / matmul / decompose chain
  has no rounding noise. The natural-feeling `0.7 → 0.2` would
  force a tolerance band even on translation-only inputs because
  `0.7` rounds at construction and the `0.7 - 0.5` subtraction
  preserves the rounding.)
- Expected:
  - Returned description's `links[0].visual_origin.xyz_m ==
    (0.25, 0, 0)` (link kinematic for arm = shoulder's kinematic =
    `(0.5, 0, 0)`; inverse-translate target gives `(0.25, 0, 0)`).
  - `joints[0].origin.xyz_m` is **unchanged** at `(0.5, 0, 0)`
    (apply at a link must not touch the joint).
  - **Rebuild assertion (rev 2):** rebuild the snapshot from the
    returned description; the arm link node's
    `world_from_local.m[3][0] == 0.75` exactly (and `m[3][1] ==
    m[3][2] == 0` exactly, rotation block identity). Same
    rationale as A3: pin what the user sees, not just the field.
- Tolerance: exact (translation-only inputs, representable).

### A4b. `EditModeApply.AppliesRotationAtLinkRoundTripsUnderRebuild`

- Layer / contract: D-VD-2 — apply at a link with a non-trivial
  rotation target writes `link.visual_origin.rpy_rad` such that
  the rebuilt snapshot reproduces the target.
- Bug class: A4 covers translation-on-link; A2 covers
  rotation-on-joint. A bug specific to link+rotation (e.g., the
  link branch uses a different decompose call than the joint
  branch and only one is wired correctly) would slip through both.
- Procedure: source = `make_v0_arm_with_joint_origin(origin_pose{
  xyz=(0,0,0), rpy=(0, 0, +M_PI/4.0)})` (joint has a yaw, so the
  link's kinematic frame is rotated relative to world). Snapshot
  built from source. selected_index = `arm` link. target =
  `compose_origin({xyz=(0.1, 0.2, 0.3),
  rpy=(+M_PI/6.0, -M_PI/8.0, +M_PI/12.0)})` (a non-trivial pose
  in *world* — apply must invert the link's kinematic frame
  to recover the link-relative `visual_origin`).
- Expected:
  - `joints[0].origin` unchanged from source (apply on link does
    not touch joint).
  - Rebuild snapshot from result. Each of the 16 elements of
    `rebuilt.arm.world_from_local.m` and `target.m` agree within
    `1e-9` (compose -> apply (decompose) -> compose; tier "≥ 3
    matmuls / trig").
- Tolerance: `1e-9` per element.

### A5. `EditModeApply.UnrelatedFieldsAreCopiedThrough`

- Layer / contract: D-VD-2 — apply is a copy-with-edit; only the
  selected node's origin field changes.
- Bug class: apply rebuilds a fresh `robot_description` and forgets
  to carry over motors / sensors / mass_kg / inertia_kgm2 / etc.
- Procedure: source = v0-arm (one motor, one sensor, link with
  mass_kg=2.0, inertia_kgm2=0.166, etc.). Apply on shoulder with
  target = translation `(0.5, 0, 0)`.
- Expected:
  - `result.name == source.name` (exact string).
  - `result.schema_version == source.schema_version`.
  - `result.motors == source.motors` (vector `operator==`).
  - `result.sensors == source.sensors`.
  - `result.links[0].mass_kg == source.links[0].mass_kg`,
    `result.links[0].length_m == source.links[0].length_m`,
    `result.links[0].inertia_kgm2 == source.links[0].inertia_kgm2`,
    `result.links[0].visual_origin == source.links[0].visual_origin`
    (untouched — apply is on the joint).
  - `result.joints[0]` differs from `source.joints[0]` only in
    `origin`: `name`, `type`, `parent`, `child`, `axis`,
    `limit_lower_rad`, `limit_upper_rad`,
    `viscous_friction_nm_per_rad_per_s` are all exactly equal.
- Tolerance: exact.

### A6. `EditModeApply.IdentityRotationNoOpApplyAtRootJointReturnsEqualDescription`

- Layer / contract: D-VD-2 — apply with target equal to the
  selected node's current `world_from_local` is a no-op at the
  description level, **for the specific case** of a root joint
  (parent kinematic = identity) whose current origin has identity
  rotation. Outside this case, decompose-then-compose introduces
  FP drift through trig and the no-op is not bit-equal.
- Bug class: apply always rewrites the origin even on a no-op,
  producing FP drift on repeated apply at the only case where
  bit-equality is feasible. (The dirty-bit transitions caused by
  this drift are the *caller's* concern — apply itself is pure
  per D-VD-2 — but a non-bit-equal no-op forces the caller to
  do its own equality check before setting dirty, which is more
  surface than necessary for v0.)
- Procedure: source = `make_v0_arm_with_joint_origin(origin_pose{
  xyz=(0.5, 0, 0), rpy=(0,0,0)})`. Snapshot built from source.
  selected_index = `shoulder` joint (root: parent is "world", so
  parent kinematic is identity). target =
  `snapshot.nodes[shoulder_idx].world_from_local` (the current
  value).
- Expected: returned description `operator==` source.
- Tolerance: exact. The path here is: snapshot's
  `shoulder.world_from_local` was produced by
  `compose_origin({(0.5,0,0), (0,0,0)})`, which evaluates to a
  matrix with `m[3][0] = 0.5` and identity rotation block bit-for-
  bit (because `cos(0) == 1.0` and `sin(0) == 0.0` exactly under
  IEEE-754). Apply inverts identity (no-op), decomposes that exact
  matrix back to `{(0.5,0,0), (0,0,0)}`, and writes it. No trig
  is invoked on a non-zero argument anywhere in the chain, so
  the round-trip is bit-equal.
- Notes: A7 covers the *non-identity-rotation* case at the
  user-observable layer (rebuild snapshot, assert within `1e-9`).
  There is intentionally no "non-identity-rotation no-op returns
  bit-equal description" test, because that round-trip is *not*
  bit-equal — trig identities lose ULP. A7's rebuild-equality is
  the meaningful contract for non-identity inputs.

### A7. `EditModeApply.AppliedTargetReproducesUnderSnapshotRebuild`

- Layer / contract: end-to-end pin — apply followed by snapshot
  rebuild produces a snapshot whose selected node's
  `world_from_local` equals the original target within `1e-9`.
- Bug class: apply writes a description that builds a different
  snapshot than the user requested — the gizmo "drifts" on every
  update.
- Procedure: source = v0-arm with identity origins. selected_index =
  shoulder. target = `compose_origin({xyz=(0.3, 0.4, 0.5),
  rpy=(+π/4, +π/6, +π/3)})`. Apply, then call
  `build_edit_mode_snapshot` on the result.
- Expected: each of the 16 elements of `rebuilt.nodes[shoulder_idx]
  .world_from_local.m` and `target.m` agree within `1e-9`.
- Tolerance: `1e-9` per element (compose -> apply -> compose; tier
  "≥ 3 matmuls / trig").

### A8. `EditModeApply.AtGimbalPolePreservesComposedTransformOnRebuild`

- Layer / contract: convention #16 + D-VD-4 — at the gimbal pole,
  the rpy_rad written to the description is non-unique, but the
  rebuilt snapshot's `world_from_local` matches the target.
- Bug class: apply at the pole writes `NaN` to the description's
  `rpy_rad` (poisoning the file on save), or writes a value that
  recomposes to a different rotation.
- Procedure: source = v0-arm with identity origins. selected_index =
  shoulder. target = `compose_origin({xyz=(0,0,0),
  rpy=(+0.4, +M_PI/2.0, -0.3)})` (pitch exactly at +π/2).
- Expected:
  - Every component of `result.joints[0].origin.xyz_m` and
    `rpy_rad` is finite (no NaN). Exercised the same way as O5:
    ```cpp
    EXPECT_FALSE(validate_finite_xyz(
        result.joints[0].origin.xyz_m, "/test/xyz_m"));
    EXPECT_FALSE(validate_finite_rpy(
        result.joints[0].origin.rpy_rad, "/test/rpy_rad"));
    ```
    (`std::nullopt` return ⇒ all finite; the throwaway
    `base_pointer` strings are unused on the success path.)
  - Each element of `build_edit_mode_snapshot(result)
    .nodes[shoulder_idx].world_from_local.m` and `target.m` agree
    within `1e-9`.
- Notes: pinned alongside the rest of A so the gimbal-pole concern
  is visible in CI logs as a named test, not buried in a parameter
  case.

---

## D. `edit_session` dirty bit + reload

### D0. `EditSession.LoadInitializesDirtyFalseAndPathSet`

- Layer / contract: D-VD-5 — `load_session` constructor contract.
- Bug class: `load_session` returns a session with `dirty = true`
  (so the user is prompted to save a file they haven't edited),
  or with `source_path` empty (Save then writes to `""` and
  fails or worse, writes to the working directory).
- Procedure: write the v0-arm fixture to a temp path. Call
  `load_session(path).value()` and capture the session.
- Expected:
  - `session.dirty == false`.
  - `session.source_path == path`.
  - `session.description.name == "v0-arm"` (sanity check that the
    description was actually populated, not default-constructed).
- Tolerance: exact (string equality).

### D0b. `EditSession.LoadFailurePropagatesLoadError`

- Layer / contract: `load_session` propagates the loader's error
  arm without transformation.
- Bug class: `load_session` swallows the error and returns a
  default-constructed session (which would then save garbage to
  disk).
- Procedure: call `load_session("/nonexistent/path.json")`.
- Expected:
  - Return arm is unexpected (`load_error`).
  - `error.kind == load_error_kind::file_not_found`.
  - `error.file_path == "/nonexistent/path.json"`.
- Tolerance: exact.

### D1. `EditSession.SaveClearsDirtyBitOnSuccess`

- Layer / contract: D-VD-5 — successful save clears the dirty bit.
- Bug class: dirty stays true after save; the user is repeatedly
  prompted to save the same file.
- Procedure: load v0-arm into a session via `load_session(temp_v0_arm
  _path)`. Mutate `session.description.joints[0].origin.xyz_m =
  (0.5, 0, 0)` and set `session.dirty = true` (simulating the
  caller's post-apply update). Call `save_session(session)`.
- Expected:
  - Return value is `expected<void, save_error>` in the success
    arm.
  - `session.dirty == false`.
  - `session.source_path` unchanged.
  - The file at `session.source_path` parses back via
    `load_from_file` and the resulting description equals
    `session.description`.
- Tolerance: exact (description `operator==`; bit-equal file
  contents per VC's S2 lossless round-trip guarantee).

### D2. `EditSession.ReloadReplacesDescriptionAndClearsDirtyBit`

- Layer / contract: D-VD-5 + D-VD-8 — reload overwrites in-memory
  state with the on-disk content and clears dirty.
- Bug class: reload merges instead of replacing; reload leaves
  dirty=true so the next save would write the (now merged) state
  back even though the user asked to discard.
- Procedure: load v0-arm into a session. Mutate
  `session.description.name = "DIRTY"` and set `session.dirty =
  true`. Call `reload_session(session)`.
- Expected:
  - Return value is `expected<void, load_error>` in the success
    arm.
  - `session.dirty == false`.
  - `session.description.name == "v0-arm"` (the on-disk value, not
    "DIRTY").
  - `session.description` equals
    `load_from_file(session.source_path).value()`.
- Tolerance: exact.

### D3. `EditSession.RepeatedSaveAndReloadAlternatesDirtyBitDeterministically`

- Layer / contract: D-VD-5 — the dirty bit's transitions are
  deterministic across many cycles. Catches a save / reload that
  silently retains state across cycles (e.g. caches the prior
  description and feeds it back instead of re-reading disk).
- Procedure:
  1. load v0-arm into session. Capture `desc_initial =
     session.description`.
  2. Mutate description (set `joints[0].origin.xyz_m[0] = 0.1`);
     `session.dirty = true`. Capture `desc_after_first_apply =
     session.description`. Save → `dirty == false`. Assert
     `session.description == desc_after_first_apply` (save did
     not perturb the in-memory state).
  3. Mutate again (`xyz_m[0] = 0.2`); `dirty = true`. Reload →
     `dirty == false` AND `session.description ==
     desc_after_first_apply` (the value from step 2's save, not
     the dirty 0.2 — and bit-equal at the description level, not
     just at `xyz_m[0]`, so a reload that restored the right
     scalar but corrupted other fields would also be caught).
  4. Save → `dirty == false`. Mutate (`xyz_m[0] = 0.3`); `dirty
     = true`. Capture `desc_after_third_apply`. Save →
     `dirty == false`. Reload → `dirty == false` AND
     `session.description == desc_after_third_apply`.
- Expected: every step's assertion holds.
- Tolerance: exact (representable inputs, lossless round-trip;
  description `operator==` is exact per loader decision #13).

### D4. `EditSession.SaveAsByMutatingSourcePathRedirectsOutputAndClearsDirty`

- Layer / contract: D-VD-7 — Save As is "mutate `source_path`, then
  save."
- Bug class: a hidden cache of the original path that survives a
  `source_path` mutation and writes to the wrong file; or a
  successful Save As that fails to clear `dirty` because the path
  changed.
- Procedure: load v0-arm into session at `path_a`. Mutate
  description; `dirty = true`. Set `session.source_path = path_b`
  (a different temp path). Call `save_session`.
- Expected:
  - Return arm is success.
  - `session.dirty == false`.
  - `path_b` exists and contains the saved description.
  - `path_a` is unchanged (still the original v0-arm bytes).
- Tolerance: exact.

### D5b. `EditSession.ReloadFailureFromCleanStatePreservesNotDirty`

- Layer / contract: D-VD-10 — reload-failure does not fabricate a
  dirty bit. Pairs with D5: D5 pins "dirty stays true on reload-
  failure when it was true going in"; D5b pins "dirty stays false
  on reload-failure when it was false going in." Together they pin
  that reload-failure preserves dirty rather than always setting
  it to true.
- Bug class: a reload-failure path that always sets `dirty = true`
  on the theory "the in-memory state may differ from disk now."
  D5 alone cannot catch this, because its starting state is
  `dirty = true` and the bug's output is also `dirty = true`.
- Procedure: load v0-arm into session (clean state — D0 pins
  `dirty == false`). Capture `desc_clean = session.description`.
  Overwrite the file at `session.source_path` with literal
  `"{ malformed"`. Call `reload_session`.
- Expected:
  - Return arm is unexpected (`load_error`) with
    `kind == load_error_kind::parse_error`.
  - `session.dirty == false` (unchanged — reload-failure did not
    promote the clean session to dirty).
  - `session.description == desc_clean`.
  - `session.source_path` unchanged.

### D5. `EditSession.ReloadFailureLeavesSessionUnchanged`

- Layer / contract: D-VD-10 — load failure during reload leaves the
  in-memory state intact.
- Bug class: a partial-reload that empties the description before
  attempting the load, leaving the user with no in-memory state if
  the file is broken.
- Procedure: load v0-arm into session. Mutate description; `dirty =
  true`. Overwrite the file at `session.source_path` with malformed
  content (literal text `"{ malformed"`). Call `reload_session`.
- Expected:
  - Return arm is the unexpected (`load_error`) arm with
    `kind == load_error_kind::parse_error`.
  - `session.description` equals the mutated state from before the
    reload attempt (unchanged).
  - `session.source_path` unchanged.
  - `session.dirty == true` (still dirty — the in-memory edits
    are still unsaved).

---

## S. Save flow (apply -> save -> reload via VC4 serializer)

### S1. `EditSession.SaveAfterApplyRoundTripsToByteIdenticalFile`

- Layer / contract: D-VD-6 + VC D-VC-6 — apply produces a
  description that the VC4 serializer writes byte-stably.
- Bug class: apply writes a description with FP drift in untouched
  fields (e.g., copies `mass_kg = 2.0` through a roundtripping
  helper that introduces `2.0000000000000004`); the VC S1 byte-
  identical guarantee then breaks across save / load / save.
- Procedure: load v0-arm. Apply target = translation `(0.123, 0.456,
  0.789)` on shoulder via `apply_gizmo_target`. Install the result
  into a session. `save_session` → `path_a`. `load_session(path_a)`
  → `session2`. `session2.source_path = path_b` (different temp
  path). `save_session(session2)`. Byte-compare `path_a` and
  `path_b`.
- Expected: byte-equal.

### S2. `EditSession.IdentityNoOpApplyOmitsOriginKeysOnSave`

- Layer / contract: D-VD-6 + VC D-VC-9 — identity origins are
  omitted from output; an apply that decomposes to identity
  preserves that.
- Bug class: apply always emits a `joint.origin` field even when
  the resulting xyz_m and rpy_rad are zero, breaking the v0-arm's
  intended identity-only on-disk form.
- Procedure: load v0-arm. snapshot built; selected_index =
  shoulder. target = `snapshot.nodes[shoulder_idx].world_from_local`
  (the current identity world transform, since v0-arm has identity
  origins). Apply. Install in session. Save.
- Expected: parse the saved file as `nlohmann::ordered_json`; assert
  `parsed["joints"][0]` has no `origin` key and `parsed["links"][0]`
  has no `visual_origin` key.

### S3. `EditSession.NonIdentityApplyEmitsOriginKeysOnSave`

- Layer / contract: D-VD-6 + VC D-VC-9 (the positive-side
  complement of S2).
- Bug class: apply mutates the description in a way the serializer
  doesn't notice (e.g. writes via a different field than
  `joint.origin`); the file omits the origin even though the user
  edited it.
- Procedure: load v0-arm. selected_index = shoulder. target =
  translation `(0.5, 0, 0)`. Apply. Install in session. Save. Parse
  saved file as `ordered_json`.
- Expected: `parsed["joints"][0]["origin"]["xyz_m"]` exists and
  equals `[0.5, 0.0, 0.0]`. `parsed["joints"][0]["origin"]
  ["rpy_rad"]` exists and equals `[0.0, 0.0, 0.0]`.

### S3b. `EditSession.NonIdentityApplyOnLinkEmitsVisualOriginAndOmitsJointOrigin`

- Layer / contract: D-VD-6 + VC D-VC-9 — the link complement of
  S3. A bug that emits `joint.origin` correctly but mis-routes
  `link.visual_origin` through the same field would pass S3 and
  slip through; this test pins both halves of "the right field
  is written."
- Bug class: apply-on-link writes the origin to the joint's
  field; or apply-on-link writes nothing because the link branch
  was never wired.
- Procedure: load v0-arm. selected_index = `arm` link. target =
  `compose_origin({xyz=(0.5, 0, 0), rpy=(0, 0, 0)})`. Apply.
  Install in session. Save. Parse saved file as
  `nlohmann::ordered_json`.
- Expected:
  - `parsed["links"][0]["visual_origin"]["xyz_m"]` exists and
    equals `[0.5, 0.0, 0.0]`.
  - `parsed["links"][0]["visual_origin"]["rpy_rad"]` exists and
    equals `[0.0, 0.0, 0.0]`.
  - `parsed["joints"][0]` has no `origin` key (apply on the link
    must not emit a joint origin).

### S4. `EditSession.SaveFailureRetainsDirtyAndPropagatesError`

- Layer / contract: D-VD-9 — save failure leaves dirty intact and
  propagates the underlying `save_error`.
- Bug class: a save that fails partway through but clears `dirty`
  anyway; the user thinks the file is saved, walks away, loses
  edits.
- Procedure: load v0-arm. Mutate description; `dirty = true`. Set
  `session.source_path` to a directory path that's chmod 0500
  (read+execute, no write — same recipe as VC S6). Call
  `save_session`.
- Expected:
  - Return arm is unexpected (`save_error`) with
    `kind == save_error_kind::io_error`.
  - `session.dirty == true` (unchanged).
  - `session.description` unchanged.
  - `session.source_path` unchanged (the bad path, not silently
    reset).
- Cleanup: chmod back to 0700 in test teardown so the temp dir is
  removable.
- Notes: skip on platforms where chmod is a no-op (none in v0;
  Linux only — the viz subtree's CI matrix is Linux-only by virtue
  of GLFW deps).

### S5. `EditSession.ApplyAtAdversarialTranslationTargetsRoundTripsBitEqualThroughSave` (parameterized)

- Layer / contract: VC S5 (lossless `%.17g` round-trip) survives
  the apply path **on the translation copy side**. Renamed in
  rev 2 from "ApplyAtAdversarialFloatTargets..." to make the
  scope explicit — only the translation-copy path can be bit-
  equal; the rotation-decompose path can't (trig eats precision).
  S5b covers the rotation side with the appropriate non-bit-equal
  tolerance.
- Bug class: apply rounds an adversarial translation float (e.g.,
  a `static_cast<float>` slip on the translation extraction path
  in `decompose_origin`) and the saved file has lower precision
  than the loaded file.
- Parameter table (each value placed into `target.m[3][0]` — the
  translation x-component, leaving rotation identity and other
  translation components zero):
  - `std::numeric_limits<double>::denorm_min()`.
  - `std::numeric_limits<double>::min()`.
  - `std::nextafter(1.0, 2.0)`.
  - `std::nextafter(1.0, 0.0)`.
  - `0.1`.
  - `1e-300`.
- Procedure (each case): load v0-arm; selected_index = shoulder;
  build target with `m[3][0] = value`, rest identity; apply; install
  in session; save to temp; reload; assert
  `reloaded.joints[0].origin.xyz_m[0]` is bit-equal to `value`
  (`std::bit_cast<uint64_t>(reloaded) ==
  std::bit_cast<uint64_t>(value)`, mirroring VC S5's bit-equal
  rule).
- Notes: the rotation block remains identity for every case; the
  rpy_rad output is `(0, 0, 0)` exactly; only the translation
  scalar is adversarial. This pins that apply doesn't lose
  precision on the translation copy path.

### S5b. `EditSession.ApplyAtAdversarialRotationTargetsRoundTripsRecomposedTransformThroughSave` (parameterized)

- Layer / contract: the rotation-side complement of S5. The
  contract here is **not** bit-equality (decompose's `atan2` /
  `asin` path eats precision in well-defined ways); it's
  "after apply -> save -> reload, the recomposed transform
  matches the original target within `1e-9`."
- Bug class: apply's rotation-decompose path silently truncates
  precision (e.g., uses `std::atan2f` instead of `std::atan2`),
  so a small but non-zero adversarial rotation gets quantized to
  zero on the way out. Bit-equality on the recomposed transform
  isn't possible, but a `1e-9` agreement is the meaningful
  contract — anything coarser is a precision regression.
- Parameter table (each value placed into the roll component of
  the target's rpy_rad):
  - `+1e-12` (smaller than apply's gimbal-pole epsilon, so the
    away-from-pole branch handles it).
  - `+M_PI / 1000.0` (small but well-conditioned).
  - `+M_PI / 4.0` (moderate).
  - `+M_PI / 2.0 - 1e-3` (near pole but not at — pin that the
    near-pole branch survives precision-wise).
- Procedure (each case): load v0-arm; selected_index = shoulder;
  build `target = compose_origin({xyz=(0,0,0), rpy=(value, 0, 0)})`;
  apply; install in session; save to temp; reload; recompose
  `m_round = compose_origin(reloaded.joints[0].origin)`; assert
  each of the 16 elements of `target.m` and `m_round.m` agree
  within `1e-9`.
- Tolerance: `1e-9` per element (compose -> apply -> save (`%.17g`
  lossless on the bit pattern) -> load -> compose; the only
  precision-eating step is decompose's trig, which is well-
  conditioned away from the pole).
- Notes: contrast with VC's S5 which uses bit-equal on raw scalars;
  here the round-trip goes through trig in both directions, so
  bit-equal isn't reachable. The contract is on the *rotation
  the user ends up with*, not on the rpy_rad scalars.

---

## F. Fixture-driven end-to-end

### F1. `VdEndToEnd.ApplyThenRebuildThenSaveThenReloadYieldsRoundTripEquality`

- Layer / contract: full integration — load → apply → snapshot rebuild
  → save → reload reproduces the user's edit at every observable
  level.
- Bug class: any drift across the chain that the focused tests
  individually miss. Example: B7 + A7 + S1 each pass, but the
  composition of all three breaks because the snapshot rebuilder
  uses one rpy_rad convention and the apply path uses another, and
  the test data happens to land on a case where each component
  succeeds in isolation.
- Procedure:
  1. `session = load_session(descriptions/v0-arm.json)`.
  2. `snapshot = build_edit_mode_snapshot(session.description)`.
  3. `target = compose_origin({xyz=(0.123, 0.456, 0.789),
     rpy=(+0.1, +0.2, +0.3)})`.
  4. `session.description = apply_gizmo_target(session.description,
     snapshot, shoulder_idx, target)`. `session.dirty = true`.
  5. `snapshot_after_apply = build_edit_mode_snapshot
     (session.description)`. Assert each of the 16 elements of
     `snapshot_after_apply.nodes[shoulder_idx].world_from_local.m`
     and `target.m` agree within `1e-9`.
  6. `session.source_path = temp path`. `save_session(session)`
     succeeds; `dirty == false`.
  7. `session2 = load_session(temp path)`. Assert
     `session2.description == session.description` (description
     `operator==`, exact — VC S2 lossless round-trip on bit
     pattern).
  8. `snapshot2 = build_edit_mode_snapshot(session2.description)`.
     Assert `snapshot2 == snapshot_after_apply` (snapshot
     `operator==`, exact — same description through same builder).

---

## VD patch deliverables outside the test plan

Per CLAUDE.md non-negotiable on "Updating the skill is part of every
feature change," none of these are optional. The patch without the
skill update is incomplete.

### Production code deliverables

- **`src/description/origin_pose.{h,cpp}`**: append `decompose_origin`
  helper. Implementation uses URDF intrinsic-Z-Y'-X'' decomposition
  via `atan2` on the rotation block's elements; the gimbal-pole
  branch is taken when `|m[0][2]| > 1 - 1e-12`, returns `yaw = 0`
  and absorbs the residual rotation into roll. The `1e-12` threshold
  is a debugging-friendly value far enough from the pole that O3's
  away-from-pole cases never hit the branch.
- **`src/viz/edit_mode_builder.cpp`**: extend the BFS traversal to
  carry a `parent_kinematic` 4×4 alongside the current bookkeeping,
  composing `joint.origin` and `link.visual_origin` per convention
  #13. The visual-frame transform for each node is what lands on
  `scene_node::world_from_local`.
- **`src/viz/edit_mode_apply.{h,cpp}`** (new): pure
  `apply_gizmo_target` per the public surface above. Implementation
  walks the description to recover `parent_kinematic` for the
  selected node (joint: parent link's kinematic; link: parent
  joint's kinematic), inverts it (4×4 inverse via the standard
  rigid-transform formula — translation column negated and
  rotation block transposed; no general-matrix-inverse needed
  because every transform in the chain is rigid), multiplies into
  the target to get the parent-relative transform, decomposes via
  `description::decompose_origin`, writes back to the appropriate
  origin field.
- **`src/viz/edit_session.{h,cpp}`** (new): `edit_session` struct +
  `load_session` / `save_session` / `reload_session` per the public
  surface above.
- **`src/viz/main.cpp`** updates:
  - Replace the direct snapshot / description bookkeeping with an
    `edit_session`; the existing global `panels.description` becomes
    `panels.session.description` (or panels takes the session
    by reference).
  - Wire the gizmo handler to call `apply_gizmo_target` and rebuild
    the snapshot on every successful `Manipulate`. Remove the
    direct `world_from_local` mutation in
    `draw_selected_gizmo`.
  - Add ImGui menu bar with **File** menu: `Save` (Ctrl+S),
    `Save As…`, `Reload from disk`. Save As opens a path-text
    modal (single-line ImGui input — no native file dialog in v0,
    which is documented in the visualizer skill as a known limit).
    Reload checks `session.dirty` and shows a "Discard changes?"
    confirmation modal if dirty before invoking `reload_session`.
  - Display save / reload errors in the status-bar panel
    (`panels_state` gains a transient `last_save_error_message`
    field, displayed alongside the existing `load_error_message`).

### Test fixture deliverables

- **`tests/viz/fixtures.{h,cpp}`** updates: add
  `make_v0_arm_with_joint_origin` and
  `make_two_joint_chain_with_origins` per "Test fixture mechanics"
  above. Existing helpers (`make_v0_arm_description` etc.) keep
  their identity-origin defaults; the new helpers wrap them and
  install the requested origins.
- **`tests/description/CMakeLists.txt`**: add
  `origin_pose_decompose_test.cpp` to the test executable.
- **`tests/viz/CMakeLists.txt`**: add the four new test files
  (`edit_mode_builder_v2_test.cpp`, `edit_mode_apply_test.cpp`,
  `edit_session_test.cpp`, `vd_e2e_test.cpp`) to the test
  executable.

### Documentation deliverables

- **`.claude/skills/visualizer.md`** updates (per CLAUDE.md
  non-negotiable):
  - Public surface: append `apply_gizmo_target`, `edit_session`,
    `load_session` / `save_session` / `reload_session`. File menu
    actions and hotkeys.
  - Architectural shape: add the kinematic-vs-visual frame
    distinction (convention #13).
  - Known limits: append "Gimbal-pole rpy_rad output is
    underdetermined — apply preserves the composed transform but
    the saved rpy_rad values may not match the input rpy_rad
    component-wise. Cross-reference convention #16 + D-VD-4."
    Append "Save As path entry uses a single-line ImGui text
    input rather than a native OS file dialog; native dialogs
    are a v1+ concern (would require a portable cross-platform
    dialog dep like `nfd`)."
  - Open follow-ups: append "Multi-select gizmo. v0 manipulates
    one node at a time."
- **`.claude/skills/robot-description-loader.md`** updates:
  - Public surface: append `decompose_origin` to the
    `origin_pose.h` row.
- **`docs/VISUALIZER_V0_PLAN.md`** Phase VD section: mark VD1–VD6
  as landed; reference this plan.
- **`CLAUDE.md`** status: append a sentence noting that VD landed
  (snapshot composition + gizmo persistence + save/reload), the
  visualizer + loader skills are updated, and v0 visualizer Edit
  mode is functionally complete.

---

## What this plan does NOT cover

- **The ImGuizmo `Manipulate` integration itself.** Smoke test only
  (`robosim-viz --smoke-test`). The pure `apply_gizmo_target`
  contract is what the unit tests pin; ImGuizmo's matrix output
  shape is the smoke test's concern.
- **The "Discard changes?" ImGui modal pixels.** Smoke test only.
  The `dirty` bit is the testable contract; the modal is the
  caller's affordance built on top of it.
- **Native OS file dialogs.** Out of v0 scope; documented as a
  known limit in the skill.
- **Multi-select gizmo manipulation.** Out of v0 scope; documented
  as an open follow-up.
- **Joint-axis-aware gizmo constraints** (snap-to-axis when the
  user is rotating about the joint's permissible DOF). Out of v0
  scope.
- **Undo / redo.** Out of v0 scope; would warrant its own design
  pass on what "undo" means at the description level.
- **Live / Replay snapshot composition.** Same composition rule
  applies, but the kinematic chain folds in joint angles from the
  state stream — out of v0 scope, will land with Live mode.

---

## Open follow-ups for the test-reviewer

The following are choices made in this draft that the reviewer may
push back on:

1. **Decompose lives in `description::`, not `viz::`.** Rationale:
   it's the algebraic inverse of `compose_origin`, and the URDF
   convention pin (D-VC-5a) is owned by the loader. Putting them in
   sibling files keeps the convention in one place. Alternative: put
   `decompose_origin` in `viz::` since only the viz subtree consumes
   it. Push back if the reviewer prefers the alternative.
2. **Apply is a free function, not a method on `edit_session`.**
   Rationale: D-VD-2's purity is easier to enforce when the function
   doesn't see the session at all. Alternative: make it
   `edit_session::apply(snapshot, idx, target)` and have it set
   `dirty` itself. Push back if the reviewer wants the merged form.
3. **No undo / redo in v0.** Tracked as out-of-scope; user can
   reload from disk to discard unsaved changes. Push back if the
   reviewer thinks even a single-step undo is table stakes.
4. **Save As is "mutate `source_path` then save," not its own
   function.** Rationale: D-VD-7 above (avoid redundant API
   surface). Push back if the reviewer wants the explicit
   `save_session_as(session, new_path)` API.
5. **Gimbal-pole behavior is "preserve the composed transform,
   accept rpy_rad non-uniqueness."** Rationale: anything else
   requires either a different rotation representation
   (quaternions) on `origin_pose`, which is a v1+ schema change,
   or a clever pole-handling decomposition (axis-angle fallback).
   Push back if the reviewer wants the second; it's plausibly
   in scope but would expand the patch.
6. **Reload-failure leaves `dirty=true`** (D-VD-10 / D5). The
   alternative is `dirty=false` on the basis that "the in-memory
   state is now disconnected from disk in either direction and the
   user shouldn't be misled into thinking they can save it." Push
   back if the reviewer prefers the alternative; the current choice
   feels closer to user intent ("I made edits; the disk is broken;
   my edits are still unsaved").
