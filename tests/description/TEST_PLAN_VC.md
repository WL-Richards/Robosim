# Robot description schema v1 → v2 (Phase VC) — draft test plan

**Status:** **`rev 3 / draft`** — addresses test-reviewer rev-2
findings (three new blockers: V5 case B math was wrong, V7/V7b's
parser mechanic was empirically false on the pinned nlohmann v3.12,
rpy element order was unpinned; plus three minor changes). Awaiting
next test-reviewer cycle. Do not write test code from this document
yet.

**Review history:**
- rev 1 → `not-ready`. Three blockers (D-VC-5 storage drift, V5 not
  pinning the convention it claimed, S5 not adversarial against
  `%.17g`) plus mechanical gaps (V7 fixture mechanics unspecified,
  V0 forward-compat described as CI-pass instead of a test,
  save-error coverage absent, no FetchContent-SHA guard for
  nlohmann, JSON-pointer-into-array convention not pinned, several
  decision pins ambiguous).
- rev 2 → `not-ready`. Rev-1 blockers closed; three new blockers
  surfaced on the rewritten material:
  - V5 case B claimed
    `R = R_z(0) · R_y(π/2) · R_x(π/2)` applied to `[0,1,0]^T` lands
    on `[0,0,1]^T`; the actual answer is `[1,0,0]^T` (or `[-1,0,0]`
    once rpy element order is pinned). Hand-derivation in a test
    plan must be correct, not aspirational.
  - V7/V7b's "hand-written JSON text with literal `NaN` /
    `Infinity` tokens" mechanic was empirically falsified against
    the FetchContent-pinned `nlohmann v3.12` — strict-mode parse
    rejects all three tokens with `parse_error.101`. The test as
    written would never reach the `domain_error` path.
  - rpy element order (`[roll, pitch, yaw]` vs.
    `[yaw, pitch, roll]`) was never explicitly pinned.
    D-VC-5a said "Same as URDF" (URDF is `[roll, pitch, yaw]`) but
    V5's prose treated the array as `[yaw, pitch, roll]`. Without
    a pin, V5 case B's correct expected vector flips sign (and
    case A's prose reads wrong).
- rev 3 → this revision. All three rev-2 blockers addressed:
  - **rpy order pinned to URDF: `[roll, pitch, yaw]`.**
    `rpy_rad[0] = roll`, `rpy_rad[1] = pitch`, `rpy_rad[2] = yaw`.
    Stated as a discrete bullet in D-VC-5a so it can't be
    re-confused.
  - **V5 cases re-derived inline.** All three cases now show the
    full three-step matrix-vector multiplication, ending with the
    correct expected vector. Case C is no longer "implementer
    derives later"; the value is pinned in the plan.
  - **V7/V7b switched to a unit-level seam** (test calls a
    factored-out per-scalar validator helper directly, not via the
    JSON parser). The "load-from-file integration test" framing
    is dropped; the contract on `domain_error` for non-finite
    values is pinned at the validator's seam, which is the only
    in-process producer of `domain_error` on numeric scalars.
  - Plus minor: S5 gains `DBL_MAX` and `DBL_MIN` rows (rev-2
    reviewer-suggested coverage); S7 dropped entirely (rev-2
    reviewer flagged it as paper-exercise; the non-atomicity
    contract moves to the loader skill); V8 gains an outer-non-
    object case (`"origin": 5`).

  Three open questions from rev 1 remain closed (intrinsic-Z-Y'-X''
  Euler convention, raw-only in-memory storage, nlohmann
  `ordered_json::dump()` pinned with a SHA guard test).

**Implements:** `docs/VISUALIZER_V0_PLAN.md` Phase VC (steps VC1–VC4):
schema extension to carry visual / joint origins, plus the
`description::save_to_file` round-trip serializer. This is **sim-core
work**, gated by the same test-reviewer discipline as the v1 loader's
test plan (`tests/description/TEST_PLAN.md`).

This document is an **addendum** to that v1 plan: it adds new
contracts on top of the v1 surface, does not replace it. The v1
loader's 83 tests must continue to pass without modification, since
the new fields are optional and default to identity. Forward-compat
is enforced by **test V0** below, not by an aspirational CI note.

---

## Why this phase exists

Phase VB (visualizer read-side) ships with all link / joint nodes
rooted at the world origin because schema v1 has no origin transforms.
ImGuizmo translate / rotate handles in `src/viz/main.cpp` mutate the
in-memory `scene_node::world_from_local` directly, but there is
nowhere to **persist** that mutation — the schema can't express it,
and there is no save serializer.

VC closes both gaps:

- **VC1 / VC3.** Schema v2 adds `link.visual_origin` and
  `joint.origin`, both optional, both defaulting to identity, both
  expressed in the parent frame.
- **VC4.** A new `description::save_to_file` writes a `robot_description`
  back to JSON in a byte-stable canonical form.

---

## VC1 — schema extension design

### New optional fields

```jsonc
{
  "schema_version": 2,
  "links": [
    {
      "name": "arm",
      "mass_kg": 2.0,
      "length_m": 0.5,
      "inertia_kgm2": 0.166,

      // optional; defaults to identity if absent.
      "visual_origin": {
        "xyz_m":   [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0]
      }
    }
  ],
  "joints": [
    {
      "name": "shoulder",
      "type": "revolute",
      "parent": "world",
      "child":  "arm",
      "axis":   [0.0, 1.0, 0.0],
      "limit_lower_rad": -1.5708,
      "limit_upper_rad":  1.5708,
      "viscous_friction_nm_per_rad_per_s": 0.1,

      // optional; defaults to identity if absent.
      // expressed in the parent link's frame ("world" frame
      // when parent == "world").
      "origin": {
        "xyz_m":   [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0]
      }
    }
  ]
}
```

### `origin` shape

- `xyz_m`: 3-element JSON array of finite doubles, meters.
- `rpy_rad`: 3-element JSON array of finite doubles, radians,
  intrinsic-Z-Y'-X'' (yaw → pitch → roll). Pinned by D-VC-5a.
- Both sub-fields are required when `origin` (or `visual_origin`) is
  present. Partial-origin objects (`{ "xyz_m": [...] }` without
  `rpy_rad`) are a `schema_error` (D-VC-4).
- The `origin` key itself is optional. A link/joint with no `origin`
  key behaves byte-identically to a v1-era input: `xyz_m = [0,0,0]`,
  `rpy_rad = [0,0,0]`.

### `schema_version` handling

- The loader accepts `1` and `2`. Anything else continues to be a
  `schema_error` (existing v1 B1).
- Under `schema_version: 1`, the `origin` / `visual_origin` keys are
  **rejected**. This is **not a new validation rule**: it falls out
  of v1 decision #8 ("Unknown fields rejected"). Pinned by D-VC-1
  with explicit cross-reference.
- Under `schema_version: 2`, both keys are optional; absence yields
  in-memory identity (D-VC-2).

### In-memory representation

- The new fields land on `link` and `joint` as **raw arrays**:

  ```cpp
  struct origin_pose {
      std::array<double, 3> xyz_m;
      std::array<double, 3> rpy_rad;  // intrinsic-Z-Y'-X''
      bool operator==(const origin_pose&) const = default;
  };
  ```

  Stored on `link::visual_origin` and `joint::origin` as `origin_pose`.
- **No composed transform on the loader's struct.** Composition into
  a 4×4 happens in the visualizer's snapshot builder, not in the
  loader. Pinned by D-VC-5b. The reasoning is `code-style.md`'s
  "no shortcuts / no defaults": one source of truth (raw user-
  authored values); derived views are computed at use-site. The
  rejected alternative (storing both raw and composed) would couple
  `operator==` to FP comparison, breaking H1's exact-equality
  guarantee on round-trip.

### `descriptions/v0-arm.json` migration

- Bump `schema_version` to `2` in the production fixture.
- Do **not** add `origin` / `visual_origin` keys (identity is the
  v0-arm's intent).
- Keep a v1 copy of the fixture at
  `tests/description/fixtures/v0_arm_v1.json` for test V0.
- The three viz fixtures
  (`make_v0_arm_description`, `make_two_joint_chain_description`,
  `make_long_named_description`) are updated to set
  `schema_version = 2` and to leave the new optional fields at their
  identity defaults.

### ARCHITECTURE.md update

- The "Robot description format" sketch in `docs/ARCHITECTURE.md`
  lines 378–410 is updated to show the v2 form (the example arm with
  `schema_version: 2`, no `origin` keys present).
- A short paragraph follows the sketch noting the v2 additions and
  pointing at this plan.

---

## VC2 — loader test plan

### Implements

The v1 loader public API is unchanged: same `load_from_file`,
same `robot_description` struct (with `origin_pose` fields appended
to `link` and `joint`), same `load_error` taxonomy.

### Decisions pinned (carried over + new)

All v1 decisions in `tests/description/TEST_PLAN.md` "Decisions
pinned" carry forward unchanged. Plus:

- **D-VC-1.** Under any `schema_version`, an unknown JSON key produces
  `schema_error` at that key's RFC-6901 pointer. This is the same
  rule as v1 decision #8; it covers the case "v1 file with v2-era
  `origin` key" *and* the case "v2 file with hypothetical v3-era
  key." The tests V1 / V1b are special-cases of v1 M1, kept in this
  plan for explicit version-gating coverage.
- **D-VC-1b** (new convention). For per-element domain errors inside
  a JSON array, `json_pointer` includes the **element index**
  (e.g. `/joints/0/origin/xyz_m/2` for a NaN at slot 2). For
  *shape* errors against an array (wrong arity, wrong element type
  uniformly, missing array entirely), the pointer is the array's
  path (e.g. `/joints/0/origin/xyz_m`), matching v1 D1's existing
  posture. Per-element vs. per-array distinction is by error kind:
  `domain_error` → element pointer; `schema_error` → array pointer.
- **D-VC-2.** When `origin` is absent in a `schema_version: 2` file,
  the in-memory `origin_pose` has `xyz_m = {0,0,0}` and
  `rpy_rad = {0,0,0}` (i.e. equals `origin_pose{}`). No
  `std::optional` tristate.
- **D-VC-3.** Non-finite floats (`NaN`, `±Inf`) inside `xyz_m` or
  `rpy_rad` are a `domain_error` with a json_pointer to the
  offending scalar (per D-VC-1b). The contract is pinned at the
  loader's per-scalar validator seam, not via JSON-text injection
  — see "Test fixture mechanics" and V7/V7b below.

  The loader uses `nlohmann::json::parse` with the strict default
  (no `parser_callback_t`, no `ignore_comments`). The pinned
  nlohmann v3.12 rejects `NaN` / `Infinity` / `-Infinity` literal
  tokens at parse time as `parse_error.101`, so production JSON
  files cannot reach the `domain_error` path on non-finite values.
  The `domain_error` arm of D-VC-3 exists as a defensive guard for
  the only other entry point that *could* place a non-finite value
  on the in-memory `origin_pose`: a future build helper or
  visualizer-side mutation flow. Per `code-style.md` "no
  half-finished implementations," that guard must still be
  pinned by a test, hence V7/V7b — but the test exercises the
  validator helper directly, not via the parser.
- **D-VC-4.** Missing `xyz_m` or missing `rpy_rad` inside a present
  `origin` object is a `schema_error` at the missing key's pointer
  (consistent with v1 D1). An entirely empty `origin: {}` is also
  a `schema_error`; the *first-offender* sub-field by declaration
  order (`xyz_m`) is reported as missing, mirroring v1's
  deterministic-second-offender rule (decision #2) but flipped to
  first-offender because both are missing simultaneously and the
  rule must be deterministic.
- **D-VC-5a.** Euler convention for `rpy_rad`:
  **intrinsic-Z-Y'-X''**, same as URDF.
  - **Array element order**: `rpy_rad[0] = roll`,
    `rpy_rad[1] = pitch`, `rpy_rad[2] = yaw`. **URDF order, pinned
    explicitly so V5 cases are unambiguous.**
  - **Composed rotation** (column-vector convention):
    `R = R_z(yaw) · R_y(pitch) · R_x(roll)`. Equivalent to
    intrinsic-Z-Y'-X'' (yaw about world Z first, then pitch about
    the rotated Y', then roll about the doubly-rotated X''), and
    equivalent to extrinsic-X-Y-Z under `R = R_z · R_y · R_x`.
    Disagrees with intrinsic-X-Y'-Z'' (i.e.,
    `R = R_x · R_y · R_z`) on any case where two or more rpy
    components are nonzero — that's what V5 pins.
  - The composition is implemented in
    `description::compose_origin(const origin_pose&) ->
    std::array<std::array<double, 4>, 4>`, exposed as part of the
    loader's public API so V5 can assert on it directly without
    going through the snapshot builder.
- **D-VC-5b.** In-memory storage shape: **raw `xyz_m` + `rpy_rad`
  only**. Composition into a 4×4 transform happens at the snapshot-
  builder use-site, not on the `link` / `joint` struct. The struct's
  defaulted `operator==` keeps doing exact-equality on doubles, which
  is what v1 H1 (8-fresh-loads invariance) requires.
- **D-VC-6.** Serializer output is **byte-stable**. (a) For the same
  `robot_description` input, two `save_to_file` calls write
  byte-identical bytes (S1a). (b) For a save→load→save cycle, both
  files are byte-identical (S1).
- **D-VC-7.** **(retired in rev 2; folded into D-VC-6.)**
- **D-VC-8.** Key ordering in saved JSON is the C++ schema's
  declaration order in `src/description/schema.h`, mirrored in the
  v0-arm fixture and the ARCHITECTURE.md sketch — a single source
  of truth. **Implementation must use `nlohmann::ordered_json`**
  (insertion-order map), not `nlohmann::json` (lexicographic-key
  std::map). Pinned in this decision rather than buried in "risks."
- **D-VC-9.** Identity origins are **omitted** from output (so v0-arm
  v2 round-trips to a file without `origin` keys, identical to the
  human-authored intent).
- **D-VC-10.** Float formatting uses
  `nlohmann::ordered_json::dump(2)` (2-space indent), with the
  default float dump policy. Byte-stability is "byte-stable for the
  pinned nlohmann FetchContent SHA." A guard test (S8) hashes the
  dump of a fixed reference value and asserts the SHA-256 matches a
  pinned constant; a silent nlohmann update that changes float
  formatting fails this test loudly.

### Cross-cutting assertion shape (extends v1 plan)

VC tests live in the same `RobotDescriptionLoader` GoogleTest suite
as v1 tests. Naming follows v1 convention
(`RobotDescriptionLoader.<test_name>`). Assertions follow v1's
"Cross-cutting assertion shape" (TEST_PLAN.md lines 119–137):
1. `result.has_value() == false` for error tests.
2. `kind == <expected load_error_kind>`.
3. `json_pointer == <expected RFC-6901 string>`.
4. `file_path == <input path>`.
5. For tests where the user's actionable info is a leaf field name,
   `message` contains the leaf field name as a substring (e.g.,
   `"xyz_m"`, `"rpy_rad"`, `"origin"`, `"visual_origin"`).

Tests do NOT assert on phrasing words ("malformed", "expected",
"must be").

### Test fixture mechanics

Carries over the v1 plan's pattern (TEST_PLAN.md lines 140–153):

- A new helper `make_v0_arm_v2_description()` returns the canonical
  v2 fixture as `nlohmann::ordered_json`, derived from the v1 helper
  `make_v0_arm_description()` plus a `schema_version` bump and
  optional `origin` / `visual_origin` injection.
- For wrong-arity / missing-key tests, `nlohmann::ordered_json::erase`
  / `at(...) = ...` build the malformed input deterministically,
  the file is dumped via `dump(2)`, and the loader is invoked.
- For non-finite-value tests (V7/V7b), the JSON-text injection
  approach **does not work** on the FetchContent-pinned nlohmann
  v3.12: hand-written `NaN` / `Infinity` / `-Infinity` literals
  fail strict-mode parse as `parse_error.101`. The plan does not
  attempt that path; V7/V7b instead exercise the loader's per-
  scalar validator helper directly. See V7's "Mechanic" subsection
  below.

### v1 forward-compatibility

#### V0. `RobotDescriptionLoader.loads_v1_arm_unchanged_under_v2_loader`

- Layer / contract: the v2 loader does not regress v1 inputs.
- Bug class: a refactor of `parse_link` / `parse_joint` that branches
  on `schema_version` accidentally drops a v1 field, or computes a
  v1 default differently.
- Fixture: `tests/description/fixtures/v0_arm_v1.json` — an exact
  copy of the pre-VC `descriptions/v0-arm.json` (schema_version 1).
- Procedure: load the v1 fixture; assert the resulting
  `robot_description` is `operator==` equal to a hand-built reference
  (a `make_v0_arm_v1_reference()` helper that mirrors the v1
  fixture's leaf values).
- Tolerance: zero.
- Notes: the existing v1 suite's 83 tests already exercise the v1
  fixture path indirectly. V0 explicitly pins "v2 loader, v1 input,
  unchanged output" as a single failing-fast test rather than relying
  on whole-suite green.

### New v2 schema tests

#### V1. `RobotDescriptionLoader.schema_version_1_rejects_origin_key`

- Layer / contract: pins D-VC-1 (a v1 file with a v2-era key fails
  via the existing unknown-field path, not via a special v2-aware
  branch).
- Bug class: a refactor that introduces a dedicated v2-key check
  and forgets to invoke v1's unknown-field check, so v1 silently
  accepts v2 keys.
- Input: v0-arm v1 fixture with an injected
  `joints[0].origin = { xyz_m: [0,0,0], rpy_rad: [0,0,0] }`.
- Expected: `schema_error`, `json_pointer == "/joints/0/origin"`.
- Notes: this is a **special case of v1 M1** (parameterized
  unknown-field rejection), not a new contract. Kept as a named
  test so a regression on this specific key is explicit in CI logs.

#### V1b. `RobotDescriptionLoader.schema_version_1_rejects_visual_origin_key`

- Symmetric to V1 for `links[0].visual_origin`.
- Expected: `schema_error`, `json_pointer == "/links/0/visual_origin"`.

#### V2. `RobotDescriptionLoader.loads_identity_when_origin_keys_absent`

- Layer / contract: pins D-VC-2.
- Bug class: a refactor introduces `std::optional<origin_pose>` and
  silently distinguishes "absent" from "identity," propagating the
  distinction downstream where the snapshot builder gets confused.
- Input: v0-arm v2 fixture (production form: no `origin` keys).
- Expected (per D-VC-5b — assertions on the **raw arrays**, not on a
  composed transform):
  - For every link: `link.visual_origin.xyz_m == {0,0,0}` and
    `link.visual_origin.rpy_rad == {0,0,0}` (exact).
  - For every joint: `joint.origin.xyz_m == {0,0,0}` and
    `joint.origin.rpy_rad == {0,0,0}` (exact).

#### V3. `RobotDescriptionLoader.absent_and_explicit_identity_origins_compare_equal`

- Layer / contract: pins that `operator==` doesn't distinguish absent
  from explicit identity (orthogonal to V2's per-field assertion).
- Bug class: a struct member that defaults differently from what the
  loader writes when explicit identity is given.
- Input A: v0-arm v2 with no `origin` keys.
- Input B: v0-arm v2 with `joints[0].origin = { xyz_m: [0,0,0],
  rpy_rad: [0,0,0] }` and `links[0].visual_origin = { ... }`
  identity.
- Expected: `desc_a == desc_b` (only this assertion; field-level
  checks are V2's job).

#### V4. `RobotDescriptionLoader.joint_origin_xyz_translation_is_propagated_raw`

- Layer / contract: pins D-VC-5b — the loader stores raw values, no
  composition.
- Bug class: the loader composes the transform, leaving the raw
  fields zero (or vice versa).
- Input: v0-arm v2 with `joints[0].origin = { xyz_m: [0.1, 0.2, 0.3],
  rpy_rad: [0, 0, 0] }`.
- Expected: `desc.joints[0].origin.xyz_m == {0.1, 0.2, 0.3}`,
  `desc.joints[0].origin.rpy_rad == {0, 0, 0}`. Tolerance: exact —
  pure double assignment, no trig, no matmul.

#### V4b. `RobotDescriptionLoader.link_visual_origin_propagated_raw`

- Symmetric to V4 for `links[0].visual_origin`.

#### V5. `RobotDescriptionLoader.rpy_intrinsic_zyx_disagrees_with_intrinsic_xyz` (parameterized)

- Layer / contract: pins D-VC-5a — the convention itself is
  **intrinsic-Z-Y'-X''** (URDF), which is observable only when at
  least two angles are nonzero (single-axis rotations are
  degenerate across every common convention).
- Bug class: a wrong-convention implementation (intrinsic-X-Y'-Z'',
  i.e., `R = R_x · R_y · R_z` instead of
  `R = R_z · R_y · R_x`; or array-index swap that maps
  `rpy_rad[0]` to yaw instead of roll) silently rotates everything
  wrong.
- This test does **not** load the JSON — it asserts on the
  loader's exposed composition helper
  `description::compose_origin(const origin_pose&) -> std::array<std::array<double, 4>, 4>`.
  That helper is the single point where the convention is encoded
  and is part of the loader's public surface so this contract is
  observable from a unit test. Per D-VC-5a, the helper computes
  `R = R_z(rpy_rad[2]) · R_y(rpy_rad[1]) · R_x(rpy_rad[0])`.
- Right-handed column-vector rotation matrices used throughout (so
  the test plan and impl agree on signs):
  ```
  R_x(θ) = [1   0       0   ]    R_y(θ) = [ cos(θ)  0  sin(θ)]    R_z(θ) = [cos(θ) -sin(θ) 0]
           [0  cos(θ) -sin(θ)]            [ 0       1  0     ]            [sin(θ)  cos(θ) 0]
           [0  sin(θ)  cos(θ)]            [-sin(θ)  0  cos(θ)]            [0       0      1]
  ```
- Procedure (parameterized over three cases, plus the
  array-index-swap discriminator):
  - **Case A** (rpy = `[π/2, π/2, 0]`: roll = +π/2, pitch = +π/2,
    yaw = 0). Apply `R = R_z(0) · R_y(π/2) · R_x(π/2)` to
    `v_in = [1, 0, 0]^T`:
    - Step 1: `R_x(π/2) · [1, 0, 0]^T = [1, 0, 0]^T`.
    - Step 2: `R_y(π/2) · [1, 0, 0]^T = [0, 0, -1]^T`.
    - Step 3: `R_z(0) · [0, 0, -1]^T = [0, 0, -1]^T`.
    - **Expected: `[0, 0, -1]^T`** (within `1e-9`).
    - Naive `R = R_x · R_y · R_z` impl would give `[0, 1, 0]^T`
      (computed by reversing the order; flagged inline in the
      test as a debugging aid).
  - **Case B** (rpy = `[0, π/2, π/2]`: roll = 0, pitch = +π/2,
    yaw = +π/2). Apply `R = R_z(π/2) · R_y(π/2) · R_x(0)` to
    `v_in = [0, 1, 0]^T`:
    - Step 1: `R_x(0) · [0, 1, 0]^T = [0, 1, 0]^T`.
    - Step 2: `R_y(π/2) · [0, 1, 0]^T = [0, 1, 0]^T` (rotation
      about Y leaves the Y-component unchanged).
    - Step 3: `R_z(π/2) · [0, 1, 0]^T = [-1, 0, 0]^T`.
    - **Expected: `[-1, 0, 0]^T`** (within `1e-9`).
    - Naive `R = R_x · R_y · R_z` impl would give `[0, 0, 1]^T`.
  - **Case C** (rpy = `[-π/2, -π/2, 0]`: roll = -π/2,
    pitch = -π/2, yaw = 0). Apply
    `R = R_z(0) · R_y(-π/2) · R_x(-π/2)` to `v_in = [1, 0, 0]^T`:
    - Step 1: `R_x(-π/2) · [1, 0, 0]^T = [1, 0, 0]^T`.
    - Step 2: `R_y(-π/2) · [1, 0, 0]^T = [0, 0, 1]^T` (cos(-π/2)=0,
      -sin(-π/2)=+1 in the lower-left entry of `R_y`).
    - Step 3: `R_z(0) · [0, 0, 1]^T = [0, 0, 1]^T`.
    - **Expected: `[0, 0, 1]^T`** (within `1e-9`).
    - Pins sign symmetry: Case A with positive angles lands on
      `[0, 0, -1]`; Case C with negated angles lands on
      `[0, 0, +1]`.
  - **Case D** (array-index-swap discriminator). Input
    `rpy = [π/2, 0, 0]` (roll = +π/2, pitch = 0, yaw = 0).
    Apply `R = R_z(0) · R_y(0) · R_x(π/2)` to `v_in = [0, 1, 0]^T`:
    - Steps collapse to `R_x(π/2) · [0, 1, 0]^T = [0, 0, 1]^T`.
    - **Expected: `[0, 0, 1]^T`** (exact: `cos(π/2)` evaluates to
      a representable `0.0` only in symbolic algebra; tolerance
      `1e-9`).
    - An impl that maps `rpy_rad[0]` to yaw would compute
      `R_z(π/2) · [0,1,0]^T = [-1, 0, 0]^T`, which the assertion
      catches.
- Tolerance: `1e-9` per component (one trig + one matmul, with
  margin against ULP accumulation).
- Notes:
  - rev-1 V5 used pure single-axis rotations, every common Euler
    convention agreed, the test could not detect a wrong-convention
    impl. rev-2 fixed the cases but had a math error in case B
    (claimed `[0, 0, 1]`; correct is `[-1, 0, 0]`) and left rpy
    element order unpinned. rev-3 pins both the order and the
    derived expected vectors.
  - Future composition tests for `xyz_m + rpy_rad` together (i.e.,
    pinning the order in which translation composes with rotation
    in `compose_origin`) live in Phase VD's plan, not here.

#### V6. `RobotDescriptionLoader.partial_origin_missing_xyz_is_schema_error`

- Pins D-VC-4 + cross-references v1 decision #1 (RFC-6901 pointer at
  the would-be-present path).
- Input: v0-arm v2 with `joints[0].origin = { rpy_rad: [0,0,0] }`.
- Expected: `schema_error`, `json_pointer == "/joints/0/origin/xyz_m"`,
  message contains `"xyz_m"`.

#### V6b. `RobotDescriptionLoader.partial_origin_missing_rpy_is_schema_error`

- Symmetric: missing `rpy_rad`.

#### V6c. `RobotDescriptionLoader.empty_origin_object_is_schema_error_at_xyz`

- Pins D-VC-4's first-offender rule for a doubly-empty `origin: {}`.
- Bug class: the validator reports an arbitrary one of the two
  missing fields, making the error pointer non-deterministic.
- Input: v0-arm v2 with `joints[0].origin = {}`.
- Expected: `schema_error`,
  `json_pointer == "/joints/0/origin/xyz_m"` (declaration order:
  `xyz_m` first, so it's the first-offender). Message contains
  `"xyz_m"`.

#### V7. `RobotDescriptionLoader.non_finite_xyz_component_is_domain_error` (parameterized)

- Pins D-VC-3 + D-VC-1b (per-element pointer convention).
- **Mechanic** (rev-3 change): tests call the loader's exposed
  per-scalar validator helper directly, not via JSON text.
  Specifically, the loader's parsing path factors out
  `description::validate_finite_xyz(const std::array<double, 3>& xyz,
  std::string_view base_pointer) -> std::optional<load_error>` (and
  symmetrically `validate_finite_rpy(...)`). The seam exists for
  this test and to keep the production parsing-loop free of
  inlined `isfinite` checks. The tests pass an `xyz` array with a
  non-finite value at the offending slot and assert the helper
  returns `load_error{kind: domain_error, json_pointer: <expected>,
  message: <name-substring>}`. **The full `load_from_file`
  integration path is not exercised** for this contract — see the
  D-VC-3 explanation above for why JSON-text injection is not
  viable on pinned nlohmann v3.12.
- 9 parameter combinations: `{NaN, +Inf, -Inf}` × `{slot 0, slot 1,
  slot 2}`.
- Expected: returns `load_error`,
  `json_pointer == "<base>/<slot>"` (e.g.,
  `"/joints/0/origin/xyz_m/2"`), `message` contains `"xyz_m"`.
- Tolerance: zero (categorical pass/fail).
- Notes:
  - The contract on `file_path` (cross-cutting assertion #4) does
    not apply at this seam — the helper has no `file_path` to
    fill in. The `load_error` it returns has an empty `file_path`,
    and the loader's caller fills it in before propagating to the
    user. The test asserts `file_path.empty()` to pin that
    convention so a refactor that pre-fills the path inside the
    helper doesn't silently change the error propagation order.

#### V7b. `RobotDescriptionLoader.non_finite_rpy_component_is_domain_error` (parameterized)

- Symmetric to V7 for `rpy_rad` (calls `validate_finite_rpy`).

#### V8. `RobotDescriptionLoader.wrong_shape_origin_field_is_schema_error` (parameterized)

- Pins D-VC-1b's *array-pointer* half (shape errors point at the
  array or container, not into it). Covers both wrong-arity arrays
  for the inner sub-fields and wrong-type values for the outer
  `origin` / `visual_origin` container.
- Parameter table:

  | Field            | Bad value         | Pointer                          |
  |------------------|-------------------|----------------------------------|
  | `xyz_m`          | `[0, 0]`          | `/joints/0/origin/xyz_m`         |
  | `xyz_m`          | `[0, 0, 0, 0]`    | `/joints/0/origin/xyz_m`         |
  | `xyz_m`          | `[]`              | `/joints/0/origin/xyz_m`         |
  | `rpy_rad`        | `[0, 0]`          | `/joints/0/origin/rpy_rad`       |
  | `rpy_rad`        | `[0, 0, 0, 0]`    | `/joints/0/origin/rpy_rad`       |
  | `rpy_rad`        | `[]`              | `/joints/0/origin/rpy_rad`       |
  | `joints[0].origin` | `5` (number)    | `/joints/0/origin`               |
  | `joints[0].origin` | `"identity"`    | `/joints/0/origin`               |
  | `joints[0].origin` | `[1, 2, 3]`     | `/joints/0/origin`               |
  | `links[0].visual_origin` | `5`       | `/links/0/visual_origin`         |
- Expected: `schema_error` at the listed pointer, message contains
  the offending field name (`"xyz_m"` / `"rpy_rad"` / `"origin"` /
  `"visual_origin"` as appropriate).
- Notes: rev-2 covered only the two inner-array shape cases. rev-3
  adds outer-non-object cases per reviewer feedback — a JSON value
  that is a number/string/array where the schema expects an object.

#### V9. `RobotDescriptionLoader.accepts_subnormal_finite_xyz_value`

- Positive corner: subnormals are finite (IEEE-754); they must load
  successfully. Bug class: an over-strict guard that calls
  `std::isnormal` (rejecting subnormals) instead of `std::isfinite`.
- Input: v0-arm v2 with
  `joints[0].origin.xyz_m[0] = std::numeric_limits<double>::denorm_min()`.
  Inject via `nlohmann::ordered_json` direct assignment, then dump
  to file — the literal text representation is the round-tripped
  decimal of `denorm_min()`.
- Expected: `desc.joints[0].origin.xyz_m[0]` exactly equals
  `denorm_min()` (lossless `%.17g` round-trip on a representable
  double).

#### V10. `RobotDescriptionLoader.v2_file_with_dangling_motor_joint_reference_returns_reference_error`

- Coverage gap from rev-1 review: cross-reference behavior under v2
  is not exercised. Catches a v2 path that short-circuits past v1's
  reference checks.
- Input: v0-arm v2 with `motors[0].joint = "nonexistent"`.
- Expected: `reference_error`, `json_pointer == "/motors/0/joint"`,
  message contains `"nonexistent"`. (Mirrors v1 R1.)

---

## VC4 — serializer test plan

### Public API

```cpp
namespace robosim::description {

enum class save_error_kind {
  io_error,           // open / write / fsync / rename failed
};

struct save_error {
  save_error_kind kind;
  std::filesystem::path file_path;
  std::string message;
};

[[nodiscard]] std::expected<void, save_error>
save_to_file(const robot_description& d, const std::filesystem::path& path);

}  // namespace robosim::description
```

**Removed from rev-1 API: `serialization_error`.** Per D-VC-3 the
loader rejects non-finite values on the way in, and there is no
in-process path that writes a non-finite into the in-memory
`origin_pose` (the visualizer's gizmo flow only writes finite
results from ImGuizmo's matrix decomposition). Carrying an
unreachable error kind violates `code-style.md`'s "no
half-finished implementations." If a future need arises (e.g., a
deserialize-into-mutable-struct API that bypasses the loader's
`isfinite` check), the kind comes back with a triggering test.

### Decisions pinned

- **D-VC-6 / D-VC-8 / D-VC-9 / D-VC-10** above govern the
  serializer's output shape.
- **D-VC-11.** v0 serializer is **non-atomic**: writes in-place via
  `std::ofstream` + close. A partial write on disk-full leaves a
  corrupted file. Documented as a known limit in the visualizer
  skill; revisited in v1 if the gizmo-save flow exposes the corrupted-
  file failure mode in the field. The plan does not test atomicity
  in v0; it tests that disk-full produces `io_error` (S6).

### New tests

#### S1a. `Serializer.same_input_yields_byte_identical_output_twice`

- Pins D-VC-6 (a) — pure serializer determinism, isolated from the
  loader.
- Procedure:
  1. Load `descriptions/v0-arm.json` (v2).
  2. `save_to_file(d, tmp_a)`.
  3. `save_to_file(d, tmp_b)` (same `d`, different path).
  4. Read both file bytes, byte-compare.
- Expected: byte-equal.
- Notes: deliberately separated from S1 so a failure here vs.
  there localizes to "serializer non-determinism" vs. "loader →
  serializer round-trip drift."

#### S1. `Serializer.round_trip_v0_arm_v2_yields_byte_identical_file`

- Pins D-VC-6 (b) — full round-trip determinism.
- Procedure:
  1. Load v0-arm v2.
  2. Save to `tmp_a`.
  3. Load `tmp_a`.
  4. Save to `tmp_b`.
  5. Byte-compare `tmp_a` and `tmp_b`.
- Expected: byte-equal.

#### S2. `Serializer.round_trip_with_modified_joint_origin_persists_value`

- Procedure:
  1. Load v0-arm v2.
  2. Mutate `joints[0].origin.xyz_m = {0.1, 0.2, 0.3}`.
  3. Save → load.
- Expected: re-loaded `joints[0].origin.xyz_m == {0.1, 0.2, 0.3}`.
- **Why this is exact equality**, not a tolerance: `%.17g` is a
  lossless round-trip for IEEE-754 doubles regardless of whether the
  underlying value is exactly representable. The values `0.1`,
  `0.2`, `0.3` are not representable, but their unique IEEE-754
  approximations are written and re-read losslessly. Equality holds
  on the bit pattern, not on the decimal.

#### S3. `Serializer.identity_origin_is_omitted_from_output`

- Pins D-VC-9.
- Procedure: load v0-arm v2 (no origin keys), save, **parse** the
  output file as `nlohmann::ordered_json`, walk its `links` and
  `joints` arrays, assert each object has no `origin` /
  `visual_origin` key.
- Notes: parsed-walk is more robust than substring search, which
  was rev-1's fragile choice (a future user-authored entity name
  containing the substring `"origin"` would break a substring assert).

#### S4. `Serializer.non_identity_origin_is_emitted_in_canonical_key_order`

- Pins D-VC-8 — declaration-order keys.
- Procedure: load v0-arm v2, set `joints[0].origin.xyz_m = {1, 2, 3}`,
  save, parse the output as `nlohmann::ordered_json` (which preserves
  insertion order), iterate `output["joints"][0]`'s keys.
- Expected: iteration order is exactly
  `[name, type, parent, child, axis, limit_lower_rad,
   limit_upper_rad, viscous_friction_nm_per_rad_per_s, origin]`.

#### S5. `Serializer.round_trips_adversarial_float_corners` (parameterized)

- Pins D-VC-10 — `%.17g` lossless round-trip across hand-picked
  adversarial float values. **PRNG forbidden**: sim-core determinism
  rules apply to loader/serializer tests (same-seed → same-bytes is
  a non-negotiable; introducing a `std::mt19937` for "more coverage"
  defeats reproducibility on a different libstdc++).
- Parameter table (each value placed into
  `joints[0].origin.xyz_m[0]` for the test):

  | Value                                                      | Why adversarial |
  |------------------------------------------------------------|-----------------|
  | `std::numeric_limits<double>::denorm_min()`                | smallest positive subnormal — formatters that drop to fixed-precision short-form lose it |
  | `std::nextafter(1.0, 2.0)`                                 | smallest distinguishable-from-1 — short-form formatters round it back to 1.0 |
  | `0.1`                                                      | classic non-representable; tests bit-pattern round-trip |
  | `1e-300`                                                   | very small but normal; tests exponent precision |
  | `1e+300`                                                   | very large; tests exponent + mantissa precision |
  | `-0.0`                                                     | signed zero — does the formatter preserve the sign bit? |
  | `M_PI`                                                     | transcendental; classic round-trip test value |
  | `1.0 - 1e-16`                                              | boundary of distinguishability from 1.0 |

- Procedure (each case): build a v0-arm v2 description with the
  bad-value injected; `save_to_file` to a temp path; `load_from_file`;
  assert the re-loaded scalar **bit-equal** to the original
  (`std::bit_cast<uint64_t>(reloaded) == std::bit_cast<uint64_t>(original)`,
  not `==`, because `==` on `-0.0` and `+0.0` returns true and would
  hide a sign-bit drop).

#### S6. `Serializer.unwritable_path_returns_io_error`

- Pins the `io_error` arm of `save_error`.
- Procedure: create a temp directory, `chmod 0500` it (read+execute,
  no write), call `save_to_file(d, tmpdir / "out.json")`.
- Expected: returns `unexpected(save_error{kind: io_error,
  file_path: <the path>})`. Message non-empty, contains the path.
- Tolerance: zero.
- Cleanup: chmod back to 0700 in test teardown so the temp dir is
  removable.
- **Skip on platforms where chmod is a no-op** (none in v0; Linux
  and macOS only). Document inline.

#### S7. `Serializer.partial_write_on_disk_full_corrupts_file_documenting_v0_limit`

- Pins D-VC-11 (no atomicity in v0). This is a **negative test of an
  intentionally-accepted limit**: it exists so a future change that
  adds atomicity (write-to-temp + rename) breaks the test, forcing a
  deliberate decision to re-verify the failure-mode contract.
- Procedure: redirect `save_to_file` to a small (`mkfifo`-backed?
  truncated tmpfs?) target where the second write fails partway
  through. Verify the on-disk file is corrupt-or-empty (not a
  complete valid JSON), and that the function returns `io_error`.
- **Implementation note**: this test is platform-specific and
  fiddly. If it cannot be made deterministic on Linux (no portable
  way to fail-mid-write reliably), demote to a skipped-with-comment
  test that documents the limit. Demoted form is acceptable in v0
  because the **contract** is "no atomicity"; the test exists as
  documentation more than verification.

#### S8. `Serializer.nlohmann_dump_byte_format_matches_pinned_sha`

- Pins D-VC-10's "byte-stable for a pinned nlohmann SHA" guard.
- Procedure:
  1. Build a fixed reference `nlohmann::ordered_json` value with
     ints, floats (including `0.1`, `1e-300`, `M_PI`),
     nested arrays, and nested objects. The exact value is
     hand-authored in the test source.
  2. `dump(2)`.
  3. SHA-256 the resulting string.
  4. Assert the digest equals a hex constant pinned in the test.
- Notes: when nlohmann's FetchContent SHA is bumped in
  `CMakeLists.txt`, this test will fail. The fix is a deliberate
  two-step:
  - re-run the test once to print the new digest;
  - paste it as the new pinned constant **only after a human**
    verifies the dump output is what we want.
  This is the foot-gun guard the rev-1 reviewer asked for.

---

## What this plan does NOT cover

- **Phase VD test plan** (gizmo + save-flow integration). Lives
  separately once VC lands.
- **Visualizer-side composition tests** for v2 origins. The viz
  test plan's rev-2 E2 note ("after VC, this test's expected value
  depends on joint.origin") will be re-opened in the VD plan; this
  document is loader-only.
- **MuJoCo / Layer 5 consumption** of the new origin fields. Out
  of v0 scope.
- **Lenient JSON parse modes** for non-finite literals. The loader
  uses nlohmann's strict default; if a real production file
  someday legitimately contains `Infinity`, that's a separate
  decision.

---

## Open questions (rev 1 → rev 2 status)

The three rev-1 open questions are all closed:

1. **Euler convention** → intrinsic-Z-Y'-X'' (D-VC-5a, pinned by V5).
2. **Storage shape** → raw `xyz_m` + `rpy_rad` only (D-VC-5b),
   composition at the snapshot-builder use-site. Rejected
   alternative documented inline.
3. **nlohmann pin** → yes, with the S8 SHA-guard test (D-VC-10).

No open questions remain in rev 2. If reviewer disagrees on any of
the three closures, push back and the discussion reopens.
