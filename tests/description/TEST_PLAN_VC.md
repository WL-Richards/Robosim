# Robot description schema v1 → v2 (Phase VC) — draft test plan

**Status:** **`ready-to-implement`** per the test-reviewer agent
(rev-4 review, with five non-blocking tightenings applied in this
revision per the reviewer's "no further test-reviewer cycle is
required if applied in the same patch" guidance). Implement the
tests and supporting deliverables in this document as specified;
if substantive deviation is needed during implementation, re-run
the test-reviewer cycle on the change rather than drifting silently.
See `.claude/skills/tdd-workflow.md` step 6.

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
- rev 4 → `ready-to-implement` with five non-blocking tightenings
  the reviewer asked to be folded into the implementation patch
  rather than another review cycle:
  - **Lint tokenset extended** to
    `isfinite | isnan | isinf | isnormal | fpclassify` (the latter
    two close the most likely accidental drift; `isnormal` would
    silently break V9 if invoked, so V9 + the lint form a
    closure-tight pair against accidental drift).
  - **Lint grep scope broadened** to every TU under
    `src/description/` other than the helper's defining TU, so an
    implementer who splits the parser across `loader.cpp` +
    `parse_origin.cpp` can't sidestep the lint by accident.
  - **Lint posture explicit:** the script is a tripwire against
    accidental drift, with V9 as the runtime backstop. Determined
    evasion (bit-cast finite-checks, third-TU wrapper functions)
    is out of scope — that's what `code-style.md`'s "no shortcuts"
    non-negotiable handles culturally. The plan now states this
    explicitly so a future maintainer doesn't read the lint as a
    closure-tight static analysis.
  - **V0b parameterized** over four representative v1-required
    leaves (`links[0].mass_kg`,
    `joints[0].viscous_friction_nm_per_rad_per_s`,
    `motors[0].controller_can_id`, the root `motors` key) — the
    asymmetric-relaxation bug class spans more than one leaf, and
    a single-leaf V0b was a thin sample.
  - **V8 wrong-element-type row added** (`xyz_m: [0, "x", 0]`) and
    D-VC-1b refined to distinguish "uniform shape error" (array
    pointer) from "per-element type mismatch" (element pointer).
    Mirrors the per-element vs. per-array distinction the decision
    already pins for `domain_error`.
  - **D-VC-2 amended** with a short paragraph stating that JSON
    `null` is treated as wrong-shape (not as absent), with the
    rationale (forecloses a future drift toward
    `null`-as-identity). Lifted from V8's notes so the decision
    is discoverable at the decision pin.
  - **Lint runs on every configure** confirmed: configure-time
    invocation from the top-level `CMakeLists.txt`, unconditional
    on `ROBOSIM_BUILD_VIZ`, so headless / loader-only CI matrices
    hit it the same as the viz matrix. Stated explicitly in the
    deliverables checklist.

- rev 3 → `not-ready, but barely`. V5 math verified case-by-case;
  V8 outer-non-object pointer convention approved; nlohmann v3.12
  NaN rejection empirically reconfirmed; D-VC-5a/5b/1b decisions
  and `serialization_error` removal all approved. Two required
  follow-ups: (a) the V7/V7b helper-seam needs to pin that the
  loader's parse path *actually calls* the helper (otherwise
  `loader.cpp` could carry a parallel inline `isfinite` check
  that nothing tests — "no half-finished implementations"); (b)
  the D-VC-11 redirection to the loader skill must track the
  skill update as part of the VC patch (CLAUDE.md non-negotiable
  on "Updating the skill is part of every feature change"). Three
  suggested polish items (D-VC-4 wording, S5 row clarity, two
  small coverage corners).
- rev 4 → this revision. The two required rev-3 follow-ups are
  addressed:
  - **V7/V7b seam call-site pin.** D-VC-3 and the V7 "Mechanic"
    subsection now require the loader to call
    `validate_finite_xyz` / `validate_finite_rpy` as its **only**
    non-finite-rejection mechanism. Enforced by a CMake-level
    lint check (`cmake/lint_loader_finite_seam.cmake`, modeled on
    the existing `lint_renderer_isolation.cmake`) that fails
    configure if `loader.cpp` contains a bare `std::isfinite`
    outside the helper. Plan tracks the lint script as a VC3
    deliverable.
  - **D-VC-11 skill-update checklist.** A new "VC patch deliverables
    outside the test plan" subsection at the end of the VC4 section
    enumerates the loader-skill update (add `save_to_file` to
    Public surface; add non-atomicity to Known limits with
    cross-reference to D-VC-11), the lint-script deliverable, and
    the v0-arm fixture migration. The plan's promise of skill
    documentation is no longer passive prose — it is a tracked
    deliverable.

  Plus rev-4 polish (all three rev-3 suggestions applied):
  - **D-VC-4 wording.** "First-offender" terminology dropped in
    favor of "schema-declaration-order tiebreaker" (V6c). Avoids
    confusion with v1 decision #2's "second-offender" rule, which
    is about a different problem (temporal collisions in a name
    table).
  - **S5 row clarity.** `1.0 - 1e-16` replaced with
    `std::nextafter(1.0, 0.0)` so the adversarial intent
    (smallest distinguishable-from-1 from below) is explicit and
    rounding-mode-independent. Mirrors the existing
    `nextafter(1.0, 2.0)` row.
  - **Two small coverage corners** added: V8 gains an
    `origin: null` row (pinning that null is treated as wrong-
    shape, not as "absent"), and a new V0b case
    (`v2_missing_required_v1_leaf_is_schema_error`) pins that v2
    did not relax v1's required-field rules.

  All three rev-1 open questions remain closed (intrinsic-Z-Y'-X'',
  raw-only in-memory storage, nlohmann SHA pin).

- rev 3 → previous revision. All three rev-2 blockers addressed:
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
- **D-VC-1b** (new convention). Pointer convention for errors
  inside an array depends on whether the error is *positional* or
  *whole-container*:
  - **Per-element error** (the element's value is wrong but the
    container's shape is fine): pointer includes the **element
    index** (e.g. `/joints/0/origin/xyz_m/2`). Applies to
    `domain_error` on a finite-but-out-of-range scalar (none in
    VC, but the convention extends to v1 D2/D5-class follow-ups)
    *and* to `schema_error` on per-element type mismatch
    (`xyz_m: [0, "x", 0]` → pointer at slot 1, not at the array).
  - **Whole-container error** (the container's shape, arity, or
    type is wrong): pointer is the **array's path**
    (e.g. `/joints/0/origin/xyz_m`). Applies to wrong arity
    (`xyz_m: [0, 0]`), uniform wrong type (`xyz_m: ["a", "b", "c"]`
    — every element is wrong, the *array as a whole* is the wrong
    shape), missing array entirely, and a non-array value where
    an array is expected (`xyz_m: 5`).
  - The split is by *locator precision*: when the user can act on
    a specific slot, point at the slot; when the problem is the
    container itself, point at the container. v1 D1's
    "string-where-array-expected at `/joints/0/axis`" is the
    whole-container side of this rule.
- **D-VC-2.** When `origin` is absent in a `schema_version: 2` file,
  the in-memory `origin_pose` has `xyz_m = {0,0,0}` and
  `rpy_rad = {0,0,0}` (i.e. equals `origin_pose{}`). No
  `std::optional` tristate.

  **JSON `null` is wrong-shape, not absent.** A literal
  `"origin": null` (or `"visual_origin": null`) is rejected as
  `schema_error` (V8). nlohmann's `ordered_json` distinguishes
  *missing key* from *present-key-with-null-value*; the loader
  does too. Treating `null` as a synonym for "absent" would
  forfeit the ability to ever pin `null` as a meaningful value
  in a future schema (e.g., a tristate "explicitly cleared" vs.
  "never set"), so v0 keeps `null` an error and the door open.
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

  **Seam call-site pin (rev 4).** To prevent the helper from
  becoming a paper test of code the loader doesn't actually use,
  the loader's parse path must call `validate_finite_xyz` /
  `validate_finite_rpy` as its **only** non-finite-rejection
  mechanism. The constraint is enforced mechanically by a
  configure-time CMake lint script
  `cmake/lint_loader_finite_seam.cmake` (modeled on the existing
  `cmake/lint_renderer_isolation.cmake` for the visualizer T1
  isolation rule).
  - **Banned tokenset**: `isfinite | isnan | isinf | isnormal |
    fpclassify`. The first three are the direct intents (a
    `std::isfinite` / `std::isnan` / `std::isinf` call line);
    `isnormal` and `fpclassify` close the most likely
    *accidental* drift — `isnormal` would silently break V9
    (subnormals) if invoked, and `fpclassify` is the next most
    plausible "I'll write my own finite-check" reach.
  - **Allow-list scope**: every `*.{h,cpp}` under
    `src/description/` *except* the helper's defining TU
    (`src/description/origin_pose.cpp` per the deliverables
    checklist; or the file the implementer chooses, listed
    explicitly in the lint script). An implementer who splits
    the parser across `loader.cpp` + `parse_origin.cpp` can't
    sidestep the lint by accident — every loader-side TU is
    grepped.
  - **Failure mode**: CMake configure fails with a message
    citing this decision, the offending file path, and the line
    of the banned token. Same posture as `scripts/lint.sh`'s
    banned-clock check in the determinism rules.
  - **Posture**: this is a tripwire against accidental drift,
    backed by V9 at runtime (which catches `isnormal` even if
    the lint missed it). It is **not** closure-tight against
    deliberate evasion (bit-cast finite-checks, third-TU wrapper
    functions, templated indirections). Deliberate evasion is
    handled culturally by `code-style.md`'s "no shortcuts"
    non-negotiable, not by the lint. A future maintainer reading
    the lint script should not extend the tokenset to chase
    every plausible workaround — V9 + the cultural rule is
    sufficient.

  Without this pin, V7/V7b can pass against the helper while the
  loader carries an inline check that no test exercises — exactly
  the bug class `code-style.md`'s "no shortcuts" non-negotiable
  forbids.
- **D-VC-4.** Missing `xyz_m` or missing `rpy_rad` inside a present
  `origin` object is a `schema_error` at the missing key's pointer
  (consistent with v1 D1). An entirely empty `origin: {}` is also
  a `schema_error`; the **schema-declaration-order tiebreaker**
  selects which missing sub-field is reported (declaration order
  in `schema.h`, so `xyz_m` is reported first when both are
  missing). This is a different rule from v1 decision #2's
  "deterministic second-offender" rule (which is about temporal
  collisions in a name table — wholly unrelated). The shared goal
  is the same: error reporting must be deterministic; the
  mechanism here is just "ordering by declaration."
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

#### V0b. `RobotDescriptionLoader.v2_missing_required_v1_leaf_is_schema_error` (parameterized)

- Layer / contract: v1's required-field rules survive untouched
  under `schema_version: 2`. Catches a v2 path that silently
  relaxes a v1 constraint.
- Bug class: a v2 branch that wraps v1 leaf-validation in
  "if v1 only" and drops the requirement on v2 inputs.
- Parameter table — four representative v1-required leaves picked
  to span the leaf-types most likely to drift on a v2 refactor:

  | Field removed                                          | Pointer                                          | Why representative |
  |--------------------------------------------------------|--------------------------------------------------|--------------------|
  | `links[0].mass_kg`                                     | `/links/0/mass_kg`                               | classic numeric leaf in a sub-array element |
  | `joints[0].viscous_friction_nm_per_rad_per_s`          | `/joints/0/viscous_friction_nm_per_rad_per_s`    | v1 decision #3 explicitly forbids defaulting; specifically tempting to default on a v2 refactor |
  | `motors[0].controller_can_id`                          | `/motors/0/controller_can_id`                    | integer leaf with its own v1 decisions (#5 / #10); pins that v2 doesn't pre-validate via a different route that bypasses v1 |
  | `motors` (root key)                                    | `/motors`                                        | root-level required key (v1 decision #4); pins that v2 doesn't relax the no-defaults rule one level up |
- Procedure (each row): start from v0-arm v2 in-memory
  `ordered_json`; `erase` the listed pointer's leaf; dump to a
  temp file; load.
- Expected: `schema_error` at the listed pointer, message contains
  the leaf field name (or root-key name).
- Notes: V0 catches the *equality* regression (v1 input + v2
  loader → unchanged struct), but a loader that asymmetrically
  *relaxed* a v1 constraint while still accepting v1 inputs
  unchanged would pass V0. V0b catches that. Four cases is a
  representative sweep across (numeric leaf, pinned-no-default
  leaf, int leaf with its own v1 decisions, root-level required
  key) — not exhaustive, but enough to fail loudly on the
  asymmetric-relaxation bug class. Full per-leaf coverage is
  what the v1 C/D-suites already provide for v1 inputs.

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

#### V6c. `RobotDescriptionLoader.empty_origin_object_is_schema_error_at_declaration_first_field`

- Pins D-VC-4's schema-declaration-order tiebreaker for a
  doubly-empty `origin: {}`.
- Bug class: the validator reports an arbitrary one of the two
  missing fields, making the error pointer non-deterministic.
- Input: v0-arm v2 with `joints[0].origin = {}`.
- Expected: `schema_error`,
  `json_pointer == "/joints/0/origin/xyz_m"` (`xyz_m` is declared
  before `rpy_rad` in `schema.h`'s `origin_pose` struct, so it's
  the deterministically-reported missing field). Message contains
  `"xyz_m"`.

#### V7. `RobotDescriptionLoader.non_finite_xyz_component_is_domain_error` (parameterized)

- Pins D-VC-3 + D-VC-1b (per-element pointer convention).
- **Mechanic** (rev-3 change, seam call-site pinned in rev 4):
  tests call the loader's exposed per-scalar validator helper
  directly, not via JSON text. Specifically, the loader's parsing
  path factors out
  `description::validate_finite_xyz(const std::array<double, 3>& xyz,
  std::string_view base_pointer) -> std::optional<load_error>` (and
  symmetrically `validate_finite_rpy(...)`). The seam exists for
  this test and to keep the production parsing path free of inlined
  `isfinite` checks. The tests pass an `xyz` array with a non-finite
  value at the offending slot and assert the helper returns
  `load_error{kind: domain_error, json_pointer: <expected>,
  message: <name-substring>}`. **The full `load_from_file`
  integration path is not exercised** for this contract — see the
  D-VC-3 explanation above for why JSON-text injection is not
  viable on pinned nlohmann v3.12.

  **Seam call-site enforcement** (D-VC-3, rev 4): the helper is the
  loader's only non-finite-rejection mechanism. Enforced by
  `cmake/lint_loader_finite_seam.cmake` at configure time, not by
  a runtime test. Without that lint, V7/V7b would be a paper test
  of a helper whose call-site nobody verifies — the
  `code-style.md` "no shortcuts" non-negotiable specifically
  forbids that shape.
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
  | `xyz_m`          | `[0, "x", 0]`     | `/joints/0/origin/xyz_m/1`       |
  | `rpy_rad`        | `[0, 0]`          | `/joints/0/origin/rpy_rad`       |
  | `rpy_rad`        | `[0, 0, 0, 0]`    | `/joints/0/origin/rpy_rad`       |
  | `rpy_rad`        | `[]`              | `/joints/0/origin/rpy_rad`       |
  | `joints[0].origin` | `5` (number)    | `/joints/0/origin`               |
  | `joints[0].origin` | `"identity"`    | `/joints/0/origin`               |
  | `joints[0].origin` | `[1, 2, 3]`     | `/joints/0/origin`               |
  | `joints[0].origin` | `null`          | `/joints/0/origin`               |
  | `links[0].visual_origin` | `5`       | `/links/0/visual_origin`         |
- Expected: `schema_error` at the listed pointer, message contains
  the offending field name (`"xyz_m"` / `"rpy_rad"` / `"origin"` /
  `"visual_origin"` as appropriate).
- Notes: rev-2 covered only the two inner-array shape cases.
  rev-3 added outer-non-object cases. rev-4 adds `origin: null`
  (`null` is treated as wrong-shape per D-VC-2's rev-4 paragraph,
  not as "absent") and the per-element type-mismatch row
  `xyz_m: [0, "x", 0]` → `/joints/0/origin/xyz_m/1`. The
  per-element row's pointer follows D-VC-1b's "per-element error
  → element pointer" half: the array's *shape* is correct (3
  elements, named field present), but slot 1's *type* is wrong,
  so the locator points at slot 1 — that's the actionable
  position for the user.

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
  corrupted file. **No test pins this property** — it is the
  *absence* of a contract, and pinning the absence with a skipped
  test adds no value (rev-2 reviewer flagged the rev-2 S7 as a
  paper exercise; dropped in rev 3). The non-atomicity limit is
  instead documented in `.claude/skills/robot-description-loader.md`
  alongside the existing loader limits, with this decision pin as
  the cross-reference. If a future change introduces atomicity
  (write-to-temp + rename), it lands with its own positive test
  of the new contract — the right time to write a test of "saves
  atomically" is when atomic save is implemented, not now.

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
  | `std::numeric_limits<double>::min()`                       | smallest *normal* positive (≈ `2.2250738585072014e-308`) — sits at the normal/subnormal boundary, where some formatters change precision behavior |
  | `std::numeric_limits<double>::max()`                       | largest finite (≈ `1.7976931348623157e+308`) — `%g` with insufficient precision can round to `Infinity` on re-parse |
  | `std::nextafter(1.0, 2.0)`                                 | smallest distinguishable-from-1 — short-form formatters round it back to 1.0 |
  | `0.1`                                                      | classic non-representable; tests bit-pattern round-trip |
  | `1e-300`                                                   | very small but normal; tests exponent precision |
  | `1e+300`                                                   | very large; tests exponent + mantissa precision |
  | `-0.0`                                                     | signed zero — does the formatter preserve the sign bit? |
  | `M_PI`                                                     | transcendental; classic round-trip test value |
  | `std::nextafter(1.0, 0.0)`                                 | smallest distinguishable-from-1 from below — short-form formatters round it back to 1.0; mirrors the `nextafter(1.0, 2.0)` row above |

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

#### S7. *(dropped in rev 3)*

The rev-2 S7 attempted to pin "no atomicity in v0" as a negative
test against a hypothetical future atomic-save change. The rev-2
reviewer flagged it as a paper exercise: a `GTEST_SKIP`'d test
nobody ever runs, "verifying" the absence of a property. rev 3
drops it entirely. The non-atomicity limit is documented in
`.claude/skills/robot-description-loader.md` and pinned by D-VC-11.
If a future change introduces atomic save, it lands with a positive
test of the new contract.

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

## VC patch deliverables outside the test plan

The VC patch is more than the failing-tests-then-implementation
sequence below. The following are part of the same patch and are
tracked here so an implementer can't ship VC code without them.

### Production code deliverables

- **`src/description/schema.h`**: append `origin_pose` POD struct
  (`std::array<double, 3> xyz_m`, `std::array<double, 3> rpy_rad`,
  defaulted `operator==`). Append `visual_origin` to `link` and
  `origin` to `joint`, both as plain `origin_pose` members
  (no `std::optional`; absent JSON → identity instance per D-VC-2).
- **`src/description/loader.cpp`**: schema_version=2 path; calls to
  `validate_finite_xyz` / `validate_finite_rpy` for non-finite
  rejection (D-VC-3 + lint).
- **`src/description/loader.h`** (or a new
  `src/description/origin_pose.h`): expose
  `validate_finite_xyz(...)`, `validate_finite_rpy(...)`, and
  `compose_origin(const origin_pose&) -> std::array<std::array<double, 4>, 4>`
  on the loader's public API. The compose helper is required for
  V5; the validators are required for V7/V7b.
- **`src/description/serializer.{h,cpp}`** (new files):
  `save_to_file(const robot_description&, const std::filesystem::path&)
  -> std::expected<void, save_error>`. Uses `nlohmann::ordered_json`
  per D-VC-8. Omits identity origins per D-VC-9.

### Schema fixture deliverables

- **`descriptions/v0-arm.json`**: bump `schema_version` to `2`. No
  origin keys added (production fixture remains identity-only).
- **`tests/description/fixtures/v0_arm_v1.json`** (new file):
  byte-identical copy of the pre-VC `descriptions/v0-arm.json`
  (schema_version 1). Used by V0.
- **`tests/viz/fixtures.cpp`** updates: bump the three viz fixture
  helpers (`make_v0_arm_description`, `make_two_joint_chain_description`,
  `make_long_named_description`) to set `schema_version = 2` with
  identity origins. Existing viz tests must remain green
  unchanged.

### Lint / build deliverables

- **`cmake/lint_loader_finite_seam.cmake`** (new file): enforces
  D-VC-3's seam call-site pin. Greps **every `*.{h,cpp}` under
  `src/description/` except the helper's defining TU** for the
  banned tokenset
  `isfinite | isnan | isinf | isnormal | fpclassify`; fails
  configure if any are found. Modeled on
  `cmake/lint_renderer_isolation.cmake`.
- **Top-level `CMakeLists.txt`**: `include()` and invoke the new
  lint script **unconditionally** (not gated on
  `ROBOSIM_BUILD_VIZ` or any other option), so headless /
  loader-only CI matrices hit the lint on every configure exactly
  the same as the viz matrix. The loader is a non-optional target;
  its lint is a non-optional check.
- **`CMakeLists.txt` nlohmann pin**: stays at v3.12 SHA
  `55f93686c01528224f448c19128836e7df245f72` (the version against
  which V7/V7b's parser-reject claim and S8's SHA constant are
  baselined). Any change to this SHA must update both S8's pinned
  digest and re-verify the V7/V7b posture.

### Documentation deliverables

- **`docs/ARCHITECTURE.md`** "Robot description format" sketch
  (lines ~378–410): update the example arm to show
  `schema_version: 2` and add a paragraph noting the v2 additions
  with a pointer to this plan.
- **`.claude/skills/robot-description-loader.md`** updates (D-VC-11
  cross-reference home):
  - Public surface: append `save_to_file`, `compose_origin`,
    `validate_finite_xyz`, `validate_finite_rpy`.
  - Known limits: append "Save flow is non-atomic in v0 — partial
    writes on disk-full leave a corrupted file. See D-VC-11 in
    `tests/description/TEST_PLAN_VC.md`. Atomic save is a v1+
    concern."
  - Schema decisions: append "schema_version 2 adds optional
    `link.visual_origin` and `joint.origin` (each `xyz_m` +
    `rpy_rad`, intrinsic-Z-Y'-X'' Euler, URDF order). Stored raw
    on the in-memory struct; composition at the snapshot-builder
    use-site (D-VC-5a/5b)."
  - Cross-references: append a pointer to `TEST_PLAN_VC.md`.
- **`docs/VISUALIZER_V0_PLAN.md`** Phase VC section: mark VC1–VC4
  as landed; reference this plan and the lint script.
- **`CLAUDE.md`** status: append a sentence noting that schema v2
  + serializer landed, the loader skill is updated, and Phase VD
  (gizmo persistence) is now unblocked.

Per CLAUDE.md non-negotiable on "Updating the skill is part of
every feature change," none of these are optional. The patch
without the skill update is incomplete.

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

## Open questions (rev 1 → rev 4 status)

The three rev-1 open questions are all closed:

1. **Euler convention** → intrinsic-Z-Y'-X'' (D-VC-5a, pinned by V5).
2. **Storage shape** → raw `xyz_m` + `rpy_rad` only (D-VC-5b),
   composition at the snapshot-builder use-site. Rejected
   alternative documented inline.
3. **nlohmann pin** → yes, with the S8 SHA-guard test (D-VC-10).

No open questions remain in rev 4. If reviewer disagrees on any of
the three closures or the rev-2/3/4 derived choices (rpy element
order = `[roll, pitch, yaw]`; V7/V7b unit-seam + lint enforcement;
S7 dropped + non-atomicity in skill), push back and the discussion
reopens.
