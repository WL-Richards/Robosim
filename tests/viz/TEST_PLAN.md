# Visualizer read-side — approved test plan (rev 4)

**Status:** **`ready-to-implement`** per the `test-reviewer` agent
(rev-3 review), amended in rev 4 (cylinder/arrow primitive origin
moved from the geometric center to the proximal end — see "Changelog
vs. rev 3" below). The amendment is a single convention swap; if
substantive deviation beyond it is needed during implementation,
re-run the test-reviewer cycle on the change rather than drifting
silently. See `.claude/skills/tdd-workflow.md` step 6.

**Review history:**
- rev 1 → `not-ready` (substantive findings: convention pinning
  missing, several test expectations wrong, decision call-outs
  unresolved).
- rev 2 → `not-ready` (narrowly): C8 angular-vs-NDC bound conflated,
  M1 sign at yaw=+π/2 wrong, plus several one-line fixes.
- rev 3 → **`ready-to-implement`** with one non-blocking note (P2/P6
  duplicate). The note has been applied in this revision: P6 now
  pins exactly-at-radius hit behavior (closed-shape semantics)
  rather than duplicating P2.
- rev 4 → cylinder/arrow primitive origin moved to the proximal end
  (`z = 0`), so a link's local frame coincides with its parent-joint
  attachment point — the natural pivot for an arm-style robot. The
  convention change is mechanical: every "centered at origin /
  ±length_m/2" expectation becomes "base at origin / spans `[0,
  length_m]`", and dependent picking and AABB test expectations are
  re-derived. No behavior outside this swap is changed.

**Implements:** `docs/VISUALIZER_V0_PLAN.md` Phase VB (steps VB1–VB4).
The subsequent renderer / panels / CLI work (VB5–VB7) is exercised by
the smoke test, not by GoogleTest unit tests.

**Implementer's first action:** read the visualizer skill
(`.claude/skills/visualizer.md`), then submit this plan to
`test-reviewer`. Iterate to `ready-to-implement` before writing any
test code.

**Changelog vs. rev 1:**

- Pinned conventions explicitly (handedness, matrix layout, cylinder
  extent, box primitive struct) so individual tests don't have to.
- Pinned the tolerance policy.
- T1 reframed as a CMake structural check; T3 replaced with a
  static_assert on field types.
- Resolved decision call-outs E5 / C3 / C4 / C6 / P7 with the
  reviewer-recommended choice; the call-outs in the body now point
  at the resolution.
- Dropped E3 (pinned implementation-internal call structure, not
  behavior). Deferred E5 (becomes a loader domain check; tracked as
  follow-up).
- Fixed C1 / C2 (expectations re-derived against the pinned
  convention), C6 (re-derived from frustum geometry, not the
  implementation's formula), C7/C8 (tolerances justified).
- Fixed P5 (ray now actually grazes the bounding sphere), P8 (box
  data model fixed), P9 (renamed; differentiated from P3).
- Added: section F (end-to-end fixture + `transform::identity()`),
  section A (AABB), section M (camera pan / zoom), section X (empty-
  snapshot + degenerate boundary cases), and S8 (selection
  semantics). Plus a non-cylinder-only picking subsection.

**Changelog vs. rev 3 (this revision):**

- **Cylinder convention #8 changed.** Cylinder local frame now has
  its origin at the proximal end (`z = 0`), with the cylinder
  extending to `z = length_m` along local +Z. Old convention had
  the cylinder centered (`±length_m/2`). Rationale: an arm link's
  natural pivot is at its parent-joint attachment, not at its
  midspan; the gizmo / world transform now lands where users expect.
- **Arrow convention #9 changed** to match cylinder #8 (base at
  origin, tip at `z = length_m`). Joint-axis arrows now visually
  emerge from the joint pivot rather than straddling it.
- **Re-derived test expectations:**
  - **A1, A2, A3, A4, A6** (cylinder/arrow AABB cases): cylinder/
    arrow z-extent flips from `[-length/2, +length/2]` to
    `[0, length_m]`. Box and sphere cases are unchanged. The
    rotated-cylinder corner sweep in A4 lands the cylinder span on
    world `[-length, 0]` along the rotated axis (was symmetric
    `±length/2`).
  - **P1, P3b, P4, P6, P10, P11** (cylinder picking): `t_along_ray`
    expectations shift by `length_m / 2` — closer cap is now at the
    primitive's local origin rather than `-length_m/2`. Specific
    new values are derived inline at each test.
  - **P2, P3, P5, P7, P9, P12, P8/P8b/P8c/P8d** unchanged (boundary
    miss / sphere-only / box-only / empty cases don't depend on the
    cylinder's z-anchor).
- All other rev-3 contracts (decisions list, sections T/E/M/C/F/X)
  carry forward unchanged.

**Changelog vs. rev 2 (rev 3):**

- **C8 reframed.** Rev-2 conflated angular FOV fraction with
  projected NDC fraction. Replaced with the simpler "every NDC
  component in `[-1, +1]`" pin, with a closed-form derivation of
  the worst corner cited in the test as a debugging aid. C6 carries
  the 80% promise alone; C8 verifies composition consistency.
- **M1 sign fixed at yaw=+π/2.** Camera-local +X (screen-right) at
  yaw=+π/2 is world `(0, 0, -1)` per glm::lookAtRH, not `(0, 0, +1)`.
  Test expectation corrected; "camera-local +X" pinned explicitly as
  the screen-right direction (the inverse-view first-row).
- **T2 procedure aligned with fixture.** Procedure text now
  references the two-joint-chain fixture (matching the fixture text
  one paragraph below); rev-2 had a v0-arm-vs-two-joint contradiction.
- **T3 trivial assert dropped.** `!is_pointer_v` on `world_from_local`
  couldn't fail in any plausible refactor; replaced with
  `is_same_v<..., transform>`.
- **A2 transform construction pinned explicitly** so a re-implementer
  doesn't accidentally use a different translation order.
- **C3 / C3b read-back sentence struck.** The contract is on
  `eye_world()` / `view_matrix()`, not on the field value.
- **C7 input pinned** to reuse the C5 AABB so the test is reproducible.
- **P11 parenthetical fixed.** R_x(+π/2) sends local +Z to world -Y,
  not +Y; the assertion is unaffected (cylinder is symmetric about
  its axis), but the parenthetical was misleading.
- **T1 pre-VB5 behavior pinned.** Target-link rule no-ops when the
  renderer target doesn't yet exist; source-grep rule still fires.
- **A6 added** (AABB over box / sphere primitives) per reviewer
  coverage-gap suggestion.
- **P2/P6 consolidated note**: P6 stays but with a slightly
  different boundary (parallel ray exactly at the radius — the
  borderline case).
- **Composite A5+X1 test** added inline as note in X1.

---

## Determinism note (visible to the test-reviewer)

The visualizer is exempt from sim-core determinism rules. Tests under
`tests/viz/` may use `std::chrono::system_clock` /
`std::chrono::steady_clock` / `std::random_device` if a real reason
arises (none does for the read-side plan). State this here so the
reviewer does not flag it as a smell. See
`.claude/skills/visualizer.md` "Determinism exception" and
`docs/ARCHITECTURE.md` "Visualizer".

The exception covers wall-clock and RNG affordances. The tests
themselves remain deterministic — same input, same output, every run.

---

## Public surface under test

### Conventions (pinned)

These are referenced from every test that needs them. Pinning them
once here means individual tests don't have to re-derive them and
can't silently drift.

1. **Coordinate system: right-handed.** +X right, +Y up, +Z out of
   the screen toward the viewer. Matches GLM's default and OpenGL's
   default.
2. **Matrix storage: column-major.** Memory layout matches GLM
   (`glm::mat4`).
3. **Matrix indexing: `m[col][row]`.** First subscript selects the
   column; second selects the row within that column. `m[3][0]` is
   `tx`, `m[3][1]` is `ty`, `m[3][2]` is `tz`. Same convention as
   GLM's `glm::mat4::operator[]`. Matches `std::array<std::array<
   double, 4>, 4>` where the outer index is column.
4. **View matrix: GLM `lookAtRH` convention.** A camera at world
   `(0, 0, 5)` looking at the origin with `up = (0, 1, 0)` produces
   the matrix
   ```
        col 0  col 1  col 2  col 3
   row 0  1     0      0      0
   row 1  0     1      0      0
   row 2  0     0      1     -5
   row 3  0     0      0      1
   ```
   i.e. `m[0][0] == m[1][1] == m[2][2] == m[3][3] == 1`,
   `m[3][2] == -5`, all other entries `0`.
5. **Projection matrix: standard right-handed OpenGL perspective.**
   Depth maps to NDC `[-1, +1]` (no reverse-Z; that's a v1+
   optimization per `docs/VISUALIZER_V0_PLAN.md` and the resolution
   of decision C4 below). Closed-form entries:
   ```
   m[0][0] =  1 / (aspect * tan(fov_y / 2))
   m[1][1] =  1 / tan(fov_y / 2)
   m[2][2] = -(far + near) / (far - near)
   m[3][2] = -(2 * far * near) / (far - near)
   m[2][3] = -1
   ```
   All other entries `0`. (`m[2][3]` is the `w`-output element under
   `m[col][row]` indexing.)
6. **Yaw rotation: right-hand rule about +Y.** Positive `yaw_rad`
   rotates the eye about +Y in the direction that sends `+Z → +X →
   -Z → -X`. Numerically: yaw = +π/2 sends an eye at
   `(0, 0, 5)` to `(5, 0, 0)`.
7. **Pitch rotation: right-hand rule about the camera's local +X
   after yaw.** Positive pitch tilts the eye toward +Y. Numerically:
   pitch = +π/2 sends an eye at `(0, 0, 5)` to `(0, 5, 0)` (the
   pole).
8. **Cylinder primitive local frame.** Axis along local +Z. Local
   origin at the **proximal end** (the cap at `z = 0`); cylinder
   extends to `z = length_m`. `radius_m` is the radius. (Rev 4
   change: previously centered at the local origin.)
9. **Arrow primitive local frame.** Same as cylinder: axis along
   local +Z, base at the local origin (`z = 0`), tip at
   `z = length_m`. `radius_m` is the shaft radius.
10. **Sphere primitive local frame.** Centered at local origin,
    radius `radius_m`. `length_m` and the box half-extents are
    unused.
11. **Box primitive local frame.** Axis-aligned in local frame,
    centered at the local origin. Half-extents along the three axes
    are `half_extent_x_m`, `half_extent_y_m`, `half_extent_z_m`.
    `length_m` and `radius_m` are unused for boxes (the rev-1 plan's
    overloading of `length_m` for boxes is fixed in this rev).
12. **Joint axis in `scene_node`: unit vector** in the joint's local
    frame. The loader is the enforcement point — see decision E5
    resolution below; the snapshot assumes axes are already
    normalized.

### Header surface

```cpp
namespace robosim::viz {

// --- scene_snapshot.h -------------------------------------------------

enum class primitive_kind {
  cylinder,
  box,
  sphere,
  arrow,
};

struct primitive {
  primitive_kind kind;
  double length_m;          // cylinder, arrow
  double radius_m;          // cylinder, arrow, sphere
  double half_extent_x_m;   // box
  double half_extent_y_m;   // box
  double half_extent_z_m;   // box

  bool operator==(const primitive&) const = default;
};

struct transform {
  // 4x4 column-major homogeneous, m[col][row] indexing — see
  // "Conventions" #2 / #3 above.
  std::array<std::array<double, 4>, 4> m;

  static transform identity();
  bool operator==(const transform&) const = default;
};

enum class node_kind {
  link,
  joint,
};

struct scene_node {
  std::string entity_name;
  node_kind kind;
  std::optional<std::size_t> parent_index;
  transform world_from_local;
  std::optional<primitive> shape;
  std::array<double, 3> joint_axis_local;  // unit vector; zero for links

  // Per-frame slots reserved for Live / Replay; v0 leaves these empty.
  std::optional<double> joint_angle_rad;
  std::vector<std::array<double, 3>> contact_points_world;
  std::vector<std::array<double, 3>> force_vectors_world;

  bool operator==(const scene_node&) const = default;
};

struct scene_snapshot {
  std::vector<scene_node> nodes;
  std::optional<std::size_t> selected_index;

  bool operator==(const scene_snapshot&) const = default;
};

// --- edit_mode_builder.h ----------------------------------------------

[[nodiscard]] scene_snapshot build_edit_mode_snapshot(
    const robosim::description::robot_description& desc);

// --- camera.h ---------------------------------------------------------

struct orbit_camera {
  std::array<double, 3> pivot_world{};
  double distance_m = 5.0;
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double fov_y_rad = 0.785398;  // 45 deg
  double aspect = 16.0 / 9.0;
  double near_plane_m = 0.05;
  double far_plane_m = 100.0;

  [[nodiscard]] std::array<double, 3> eye_world() const;
  [[nodiscard]] std::array<std::array<double, 4>, 4> view_matrix() const;
  [[nodiscard]] std::array<std::array<double, 4>, 4> projection_matrix() const;
};

// In-place mutators. The .cpp may dispatch through any internal helper
// shape; the public contract is just "after the call, the camera state
// reflects the operation."
void pan(orbit_camera& cam,
         double right_offset_m, double up_offset_m);
void zoom(orbit_camera& cam, double scroll_units);

struct aabb {
  std::array<double, 3> min_world;
  std::array<double, 3> max_world;

  bool operator==(const aabb&) const = default;
};

[[nodiscard]] aabb compute_world_aabb(const scene_snapshot& s);

void frame_fit(orbit_camera& cam, const aabb& world_box);

// --- picking.h --------------------------------------------------------

struct ray {
  std::array<double, 3> origin_world;
  std::array<double, 3> direction_world;  // not required to be unit
};

struct pick_hit {
  std::size_t node_index;
  double t_along_ray;  // origin + t * direction = hit point; t >= 0
};

[[nodiscard]] std::optional<pick_hit> pick(
    const scene_snapshot& s, const ray& r);

}  // namespace robosim::viz
```

The renderer / ImGui panels / picking-from-screen-space are not in
this test plan. The smoke test exercises them at the
`robosim-viz --smoke-test` boundary (Phase VB5–VB7).

### Tolerance policy

To keep individual tests from re-justifying tolerances ad hoc:

- **Exact equality (`==`)** for outputs derived from inputs that are
  exactly representable in `double` and pass through ≤ 1 multiplication
  without summation. (E.g. `length_m = 0.5` is propagated to a
  primitive's `length_m`.)
- **Absolute `1e-12`** for outputs of one matmul or a normalization
  with exactly representable inputs. (E.g. axis normalization
  dividing by a representable length.)
- **Absolute `1e-9`** for outputs of ≤ 3 matmuls of double-precision
  math. Default for view / projection matrix individual elements.
- **Absolute `1e-6`** for outputs of ≥ 3 matmuls or trig with
  arbitrary angles. Used in C8 (proj × view × point).
- **Tighter** when the test has a special reason; **looser** never
  without an inline justification.

Tests cite which tier they're in.

---

## Decisions pinned

These were committed to during the test-reviewer cycle (rev 1 →
rev 2). The plan implements each one; tests below pin each one.

1. **Renderer reads only from `scene_snapshot`.** Pinned by T1 (a
   CMake structural check, not a runtime test).
2. **Snapshot is POD with `operator== = default`.** Pinned by T2 +
   F1.
3. **Edit-mode builder is the only producer of snapshots in v0.**
   Live / Replay producers are post-v0; their absence is
   intentional.
4. **Joint axes are stored in the joint's local frame** in
   `scene_node`, normalized to unit length. **Loader enforces unit
   length** (decision E5 resolution below).
5. **`world` parent in a description joint maps to `parent_index ==
   nullopt`.** Pinned by S5.
6. **Frame-fit is idempotent** (within `1e-9` per scalar).
7. **Picking returns the closest hit** (smallest `t_along_ray`).
8. **Picking is exact ray-vs-primitive**, not screen-space AABB or
   bounding-sphere.
9. **Picking is a half-line** (`t >= 0`); `t == 0` for rays starting
   inside a primitive (decision P7 resolution).
10. **Box primitive has explicit half-extents** along all three
    axes. `length_m` is cylinder/arrow only.
11. **Cylinder primitive has its local origin at the proximal end**
    (cap at `z = 0`), extending to `z = length_m` along local +Z.
12. **Standard OpenGL perspective for projection** in v0 (decision
    C4 resolution); reverse-Z deferred.
13. **Pitch is clamped to `[-π/2 + 1e-3, +π/2 - 1e-3]`** to avoid
    the gimbal pole (decision C3 resolution); not configurable in
    v0.
14. **Frame-fit fills 80% of the vertical FOV's *angular*
    extent** (decision C6 resolution); not configurable in v0.

### Decision call-out resolutions (rev 2)

| Section | Decision | Rev-2 resolution |
|---|---|---|
| **E5** | Normalize axis in builder vs. reject in loader | **Reject in loader** as a domain check. Snapshot assumes unit axes. Loader work tracked in [Loader follow-up: axis-magnitude domain check](#loader-followups) below. E5 deferred. |
| **C3** | Pitch clamp `1e-3 rad`, configurable? | **Hard-code `1e-3 rad`.** Revisit on user complaint. |
| **C4** | Standard OpenGL vs. reverse-Z | **Standard OpenGL** in v0; reverse-Z post-v0. |
| **C6** | 80% framing, configurable? | **Hard-code 80%.** Revisit on user complaint. |
| **P7** | Ray inside primitive — `t == 0` or skip? | **Return `t == 0`** (with `t < 1e-9` tolerance for robust interior-test implementations). |

### Loader follow-ups (rev 2)

The deferred E5 decision becomes a loader test-plan addition. To track
explicitly so it doesn't get lost:

- Add a domain-check case to `tests/description/loader_test.cpp`
  parameterized J1 (or a new J3): joint `axis` magnitude must be
  within `[1.0 - 1e-9, 1.0 + 1e-9]`, otherwise `domain_error`.
- The current loader does not enforce this; it would be added
  alongside this VB plan landing.
- This is sim-core work, not viz work, and goes through the loader's
  test-reviewer cycle. Cross-reference both plans in the next loader
  plan revision.

---

## Cross-cutting assertion shape

For each test:

1. Build inputs deterministically: derive `robot_description`s from
   the v0-arm fixture via a `make_v0_arm_description()` helper
   (mirrors the loader test plan), or load `descriptions/v0-arm.json`
   directly via `description::load_from_file`.
2. Call exactly one public function from the surface above.
3. Assert via the tolerance policy.

Tests do **not** pin internal data-structure layout (e.g.
"`scene_snapshot::nodes` has capacity X"). They pin the public
contract.

### Test fixtures (shared)

A `tests/viz/fixtures.{h,cpp}` translation unit exposes:

- `make_v0_arm_description()` — in-memory `robot_description`
  matching the production `descriptions/v0-arm.json`. Used by
  builder tests so they don't hit the filesystem.
- `make_two_joint_chain_description()` — extends the v0-arm with a
  second link `forearm` and a second revolute joint `elbow` whose
  `parent: "arm"`, `child: "forearm"`. Used by T2, E1, end-to-end
  tests.
- `make_long_named_description()` — same shape as v0-arm but every
  `name` field is ≥ 32 chars (defeats SSO). Used by T3.

The shipped fixture file at `descriptions/v0-arm.json` is the source
of truth A1 / F1 load directly.

---

## Test layout

- All tests in `tests/viz/`. Suite names: `SceneSnapshot`,
  `EditModeBuilder`, `Aabb`, `OrbitCamera`, `Picking`,
  `RendererSeam`, `Transform`, `EndToEnd`.
- GoogleTest. `INSTANTIATE_TEST_SUITE_P` for parameter spreads
  (camera angle sweep, picking ray sweep).
- Shared helpers in `tests/viz/fixtures.{h,cpp}`.

---

## T. Renderer / data-source seam

This section pins the most important architectural commitment in the
visualizer.

### T1. CMake structural check — renderer source/include isolation

- **Not a GoogleTest test; a CMake configure-time structural check.**
  Lives at `cmake/lint_renderer_isolation.cmake`, invoked from the
  top-level `CMakeLists.txt` when `ROBOSIM_BUILD_VIZ=ON`.
- **What it does:**
  1. Greps `src/viz/` for any inclusion of `description/schema.h`,
     `description/loader.h`, `description/error.h`, or any
     `robosim::description::` qualified name.
  2. **Allow-list:** `src/viz/edit_mode_builder.{h,cpp}` and
     `tests/viz/fixtures.{h,cpp}` are the only files permitted to
     reference the description namespace.
  3. **Target-level rule:** the renderer's CMake target
     (`robosim_viz_renderer`, landing in Phase VB5) is asserted to
     not transitively link `robosim_description`. Implemented via
     `get_target_property(... INTERFACE_LINK_LIBRARIES ...)` walked
     to a fixed point.
  4. Configure fails (not test-time fails) if either rule trips.
- **Pre-VB5 behavior:** the renderer target (`robosim_viz_renderer`)
  lands in Phase VB5. Until then, the target-link rule no-ops
  (the target does not exist) and only the source-grep rule fires.
  Implementation must `if(TARGET robosim_viz_renderer)` around the
  target walk so the check is not a "passes-by-skip" gate during
  Phases VB1–VB4.
- **Bug class:** a future change adds `#include "description/
  schema.h"` to a renderer-internal header for convenience, coupling
  the renderer to the loader's struct and making Live / Replay
  expensive to retrofit.
- **Why configure-time and not GoogleTest:** a runtime test cannot
  pin a `#include` graph, and a "delete the test" bypass is too
  cheap. Configure-time means the build itself fails — same posture
  as `scripts/lint.sh`'s banned-clock check.
- **Known limitation:** the source-grep is text-only — `using`
  aliases (`using rd = robosim::description;`) or `auto`-deduced
  uses can sneak past it. v0 codebase doesn't use such aliases;
  worth a comment in the implementation. Tighter static analysis is
  a follow-up.

### T2. `snapshot_is_value_equal_when_built_from_eight_fresh_loads`

- Layer / contract: edit-mode builder reproducibility.
- Bug class: hidden non-determinism (unordered iteration,
  hash-randomization seed).
- **Fixture:** `make_two_joint_chain_description()` serialized to a
  temp JSON file once. Three nodes (link `arm` → joint `shoulder` →
  link `forearm` → joint `elbow` ... actually four nodes: two links
  + two joints) are enough to make ordering observable. (The v0-arm
  fixture has only two nodes — too few to spot an ordering swap.)
- Procedure: load the temp JSON file 8 times via
  `description::load_from_file` into 8 separately-constructed
  `robot_description` instances. For each, call
  `build_edit_mode_snapshot`. Assert all 8 snapshots are
  `operator==` equal to the first.
- Tolerance: zero.
- Notes: 8 (rather than 2) shakes out hash-iteration variance some
  implementations introduce after warm-up; matches loader H1.

### T3. `scene_node_owns_its_string_data` (compile-time)

- Layer / contract: snapshot is decoupled from source description
  lifetime.
- Bug class: snapshot stores `std::string_view` / raw pointers into
  the source `robot_description`; UB after destruction, and Live /
  Replay producers must each remember the rule.
- Procedure: a `static_assert` block in
  `tests/viz/scene_node_traits_test.cpp`:
  ```cpp
  static_assert(std::is_same_v<decltype(scene_node{}.entity_name),
                               std::string>);
  static_assert(std::is_same_v<
      decltype(scene_node{}.contact_points_world),
      std::vector<std::array<double, 3>>>);
  static_assert(std::is_same_v<
      decltype(scene_node{}.shape),
      std::optional<primitive>>);
  static_assert(std::is_same_v<
      decltype(scene_node{}.world_from_local),
      transform>);
  ```
  (Rev-2 had `!is_pointer_v` for the last two; replaced with
  `is_same_v<..., expected_type>` which actually catches a
  refactor that changes the field type.)
  Plus one runtime test (`scene_node_survives_source_description_destruction`):
  build a snapshot from a temporary description constructed via
  `make_long_named_description()` (entity names ≥ 32 chars to defeat
  SSO so a real string lifetime bug actually corrupts), let the
  description go out of scope, then assert the snapshot equals one
  built from a fresh long-named description. **This runtime test
  is meaningful only under ASan/UBSan** — call out explicitly in the
  test that the CI sanitizer matrix is the gate; without ASan it
  may pass-on-luck.
- Tolerance: zero (`operator==`).
- Notes: rev 1 acknowledged the SSO-vs-real-bug gap; rev 2 fixes by
  switching the primary contract to a static_assert (no SSO loophole)
  and using long names in the runtime sibling test.

---

## S. Snapshot data model

### S1. `snapshot_node_count_matches_link_plus_joint_count`

- Bug class: per-motor or per-sensor nodes leaking; duplicated nodes.
- Input: v0-arm fixture.
- Expected: `snapshot.nodes.size() == 2`.

### S1b. `snapshot_distinguishes_link_and_joint_nodes_by_kind`

- Bug class: only `link` nodes (joint metadata lost) or only `joint`
  nodes (link visuals lost).
- Input: v0-arm fixture.
- Expected: exactly one node with `kind == link`, name `"arm"`;
  exactly one with `kind == joint`, name `"shoulder"`.

### S2. `link_node_has_cylinder_primitive_with_correct_dimensions`

- Bug class: placeholder dimensions ("0.1 everywhere") rendering
  wrong-sized geometry; `radius_m` defaulting to zero produces
  invisible links.
- Input: v0-arm fixture (`links[0].length_m = 0.5`).
- Expected:
  - `arm.shape->kind == cylinder`.
  - `arm.shape->length_m == 0.5` (exact).
  - `arm.shape->radius_m == 0.05` (the documented default — see
    decision below).
- **Decision pin:** v0 uses a **fixed default cylinder radius of
  0.05 m** for link visuals. The schema does not carry a link
  visual radius; v0 picks a constant rather than computing from
  inertia. Schema v2 may add a `link.visual_radius_m` field; the
  builder's behavior then changes to "if present, use it; else
  default 0.05". Documented in `.claude/skills/visualizer.md` "Open
  follow-ups" at implementation time.

### S3. `joint_node_has_arrow_primitive_with_correct_dimensions`

- Bug class: arrow defaulting to zero length / radius — the joint
  axis isn't visible.
- Input: v0-arm fixture.
- Expected:
  - `shoulder.shape->kind == arrow`.
  - `shoulder.shape->length_m == 0.20` (default arrow length, m).
  - `shoulder.shape->radius_m == 0.01` (default arrow shaft radius,
    m).
- **Decision pin:** arrow defaults are `length_m = 0.20`,
  `radius_m = 0.01`. Eyeball-tuned for visibility on a 0.5-m link;
  documented in the skill as a follow-up if a use case demands
  configurability.

### S4. `joint_node_records_axis_in_local_frame`

- Pins decision #4.
- Input: v0-arm fixture (joint axis `[0, 1, 0]`).
- Expected: `joint_node.joint_axis_local == {0.0, 1.0, 0.0}` exact.

### S5. `world_parent_maps_to_nullopt_parent_index`

- Pins decision #5.
- Input: v0-arm fixture.
- Expected: `shoulder.parent_index == std::nullopt`.

### S6. `child_link_node_has_parent_index_pointing_at_its_joint`

- Bug class: kinematic tree flattened, parent relationships lost.
- Input: v0-arm fixture.
- Expected: `arm.parent_index` is set; the node it points at has
  `kind == joint` and `entity_name == "shoulder"`.

### S7. `per_frame_slots_default_empty`

- Pins that v0 builder leaves Live / Replay slots untouched.
- Input: v0-arm fixture.
- Expected: every node has `joint_angle_rad == std::nullopt`,
  `contact_points_world.empty()`, `force_vectors_world.empty()`.

### S8. `selected_index_defaults_to_nullopt`

- Pins selection's default state. Builder doesn't set selection;
  selection is an Edit-mode UI concern.
- Input: v0-arm fixture.
- Expected: `snapshot.selected_index == std::nullopt`.

---

## E. Edit-mode builder (description → snapshot)

### E1. `multi_link_chain_preserves_topological_order` (parameterized)

- Bug class: ordering breaks downstream scans assuming
  parent-before-child.
- **Two parameterized cases** (this is the rev-2 strengthening):
  - Case A: source description has joints in topological order
    (`shoulder` then `elbow`).
  - Case B: source description has joints in **reverse** order
    (`elbow` then `shoulder`). The JSON is constructed
    deliberately out of order.
- Procedure (each case): build the snapshot.
- Expected: every node's `parent_index`, when set, refers to a node
  whose vector index is strictly less than the current node's index.
  This holds in **both** cases.
- Notes: case B is the non-degenerate version. A builder that copies
  input-order would pass case A but fail case B.

### E2. `world_parent_joint_has_identity_world_transform`

- Renamed from rev-1 `unknown_world_origin_links_root_to_identity_transform`
  for clarity.
- Bug class: builder injects a non-identity transform from somewhere
  even when the schema doesn't specify one.
- Input: v0-arm fixture.
- Expected: `shoulder.world_from_local == transform::identity()`
  exact.
- **Phase-VC migration note:** schema v2 adds `joint.origin`. After
  VC, this test's expected value depends on the joint's origin
  field. The VC test plan must include a proper composition test
  (the rev-1 E3 was the would-be home for it; rev 2 drops E3 and
  defers composition entirely to VC, where origins exist).

### ~~E3.~~ (dropped in rev 2)

The rev-1 E3 pinned an internal-call-site invariant ("builder calls
into a transform-composition path even when the result is identity")
that is not behaviorally observable. Dropped per reviewer feedback.
The composition behavior will be tested in Phase VC alongside the
schema bump; tracked as a follow-up there, not as a placeholder
here.

### E4. `builder_handles_passive_robot_with_no_motors_or_sensors`

- Pins that snapshot is purely geometric — motors / sensors don't
  appear.
- Procedure: build snapshot S1 from the v0-arm fixture; mutate the
  fixture in-memory to `motors: []`, `sensors: []` (still loadable
  per loader I1), build snapshot S2.
- Expected: `S1 == S2`.
- Notes: `selected_index` is `nullopt` in both (S8 pins).

### ~~E5.~~ (deferred in rev 2)

The rev-1 E5 normalized non-unit axes in the builder. Per reviewer
feedback, the better home for this rule is the loader as a domain
check (decision pinned above; loader follow-up tracked above). v0
builder assumes unit-length axes (snapshot conventions #12). When
the loader's axis-magnitude check lands, this test plan does not
need a builder-side counterpart — S4 is sufficient.

---

## A. AABB computation

### A1. `compute_world_aabb_bounds_single_cylinder_at_origin`

- Bug class: AABB defaults to `{0,0,0}-{0,0,0}` ignoring primitive
  extent; downstream frame-fit then frames against a zero-volume
  box and the camera distance is wrong.
- Input: snapshot with one cylinder node, `world_from_local =
  identity`, `length_m = 1.0`, `radius_m = 0.1`.
- Expected:
  - `min_world == {-0.1, -0.1, 0.0}` (-X / -Y radial extremes,
    proximal cap at `z = 0` per convention #8).
  - `max_world == { 0.1,  0.1, 1.0}`.
- Tolerance: exact (representable inputs).

### A2. `compute_world_aabb_combines_multiple_primitives`

- Input: snapshot with two cylinders. Cylinder 0 at world origin
  (`world_from_local = transform::identity()`, length 1.0, radius
  0.1). Cylinder 1 translated by `(2, 0, 0)`: `world_from_local` is
  the identity matrix with `m[3][0] = 2.0` (translation column,
  x-row, per convention #3); all other off-diagonals zero,
  diagonals 1. Same dimensions as cylinder 0.
- Expected:
  - `min_world == {-0.1, -0.1, 0.0}`.
  - `max_world == { 2.1,  0.1, 1.0}`.

### A3. `compute_world_aabb_respects_world_from_local_translation`

- Input: snapshot with one cylinder translated by `(5, 0, 0)`.
- Expected: `min_world == {4.9, -0.1, 0.0}`,
  `max_world == {5.1, 0.1, 1.0}`.

### A4. `compute_world_aabb_handles_rotated_cylinder_via_obb_envelope`

- Bug class: AABB ignores rotation, returning the local-frame AABB
  in world-frame coordinates and missing the actual extent.
- Input: cylinder with `world_from_local` = rotation about +X by
  90°, length 1.0, radius 0.1. Local +Z lands on world -Y per
  right-hand rule, so the cylinder's local span `z ∈ [0, 1]` becomes
  world `y ∈ [-1, 0]`.
- Expected: a tight AABB enclosing the rotated cylinder.
  - `min_world ≈ {-0.1, -1.0, -0.1}`.
  - `max_world ≈ { 0.1,  0.0,  0.1}`.
  - Tolerance `1e-9` (one matmul).
- **Decision pin:** the AABB is computed from the primitive's 8
  bounding-box corners transformed by `world_from_local`, not by
  re-derived per-primitive math. Tighter envelopes (e.g. for
  rotated cylinders) are post-v0; the OBB-corner method
  over-approximates by up to `~10%` for highly oblique angles,
  which is acceptable for camera framing. State explicitly so a
  future "tighten the AABB" PR knows to update this test.

### A5. `compute_world_aabb_returns_degenerate_for_empty_snapshot`

- Boundary case.
- Input: `scene_snapshot{}` with no nodes.
- Expected: a documented sentinel — `min_world == {0,0,0}` and
  `max_world == {0,0,0}` (a zero-volume box at the origin). Code
  comment in the implementation cites this test.

### A6. `compute_world_aabb_handles_non_cylinder_primitives` (parameterized)

- Coverage gap from rev-2 review: A1–A4 only test cylinders. If
  the AABB implementation special-cases `primitive_kind` and the
  box / sphere / arrow branches are missing, A1–A4 wouldn't catch
  it.
- Parameterized over the three non-cylinder kinds:
  - **Box** with `half_extent_x_m = 0.4`, `half_extent_y_m = 0.3`,
    `half_extent_z_m = 0.2`, `world_from_local = identity`:
    expected `min = {-0.4, -0.3, -0.2}`, `max = {0.4, 0.3, 0.2}`.
  - **Sphere** with `radius_m = 0.5`, `world_from_local = identity`:
    expected `min = {-0.5, -0.5, -0.5}`, `max = {0.5, 0.5, 0.5}`.
  - **Arrow** with `length_m = 0.20`, `radius_m = 0.01`,
    `world_from_local = identity` (per convention #9 — same shape
    as a cylinder along +Z, base at origin): expected
    `min = {-0.01, -0.01, 0.0}`, `max = {0.01, 0.01, 0.20}`.
- Expected: tolerance exact (representable inputs, no rotation).

---

## C. Orbit camera

### C1. `view_matrix_at_default_pose_is_translation_minus_five_z`

- Bug class: handedness flip; row/column-major mismatch; sign error
  in look-at.
- Input: default `orbit_camera` (`pivot_world = (0,0,0)`, distance
  5.0, yaw 0, pitch 0).
- Expected (per convention #4 above):
  - `cam.eye_world() == {0.0, 0.0, 5.0}` (exact).
  - `m[0][0] == 1.0`, `m[1][1] == 1.0`, `m[2][2] == 1.0`,
    `m[3][2] == -5.0`, `m[3][3] == 1.0` (each `1e-12`).
  - All other elements `== 0.0` (exact).

### C2. `yaw_plus_half_pi_rotates_eye_to_positive_x`

- Pins yaw direction (convention #6).
- Input: default camera, set `yaw_rad = +π/2`.
- Expected: `eye_world() ≈ {5.0, 0.0, 0.0}` (within `1e-9` per
  component — one trig evaluation).

### C2b. `pitch_plus_half_pi_minus_clamp_rotates_eye_toward_positive_y`

- Pins pitch direction (convention #7) without hitting the clamp.
- Input: default camera, set `pitch_rad = π/4` (well within clamp).
- Expected: `eye_world() ≈ {0.0, 5*sin(π/4), 5*cos(π/4)} =
  (0.0, 3.5355..., 3.5355...)` (within `1e-9` per component).

### C3. `pitch_clamped_to_avoid_gimbal_singularity_positive_pole`

- Pins decision #13 (`±π/2 ∓ 1e-3` clamp; v0 not configurable).
- Contract: the clamp is observed on the *derived outputs*
  (`eye_world()`, `view_matrix()`), not on the field. The
  implementation may evaluate the clamp lazily inside those getters
  or eagerly in a setter — either is fine. The field's read-back
  value is unspecified; tests must not assert on it.
- Procedure: set `pitch_rad = +1e9`. Read `eye_world()`. Compare to
  the closed-form eye position at `pitch == +π/2 - 1e-3` and the
  current `yaw_rad` / `distance_m`.
- Tolerance: `1e-6` per component. The clamp epsilon is `1e-3` rad;
  the eye position is derived through one trig and one multiply at
  that angle.

### C3b. `pitch_clamped_to_avoid_gimbal_singularity_negative_pole`

- Symmetric to C3 — pin both clamp ends.
- Input: `pitch_rad = -1e9`.
- Expected: `eye_world()` matches the closed-form position at
  `pitch == -π/2 + 1e-3` (within `1e-6` per component).
- Contract / read-back rule: same as C3.

### C4. `projection_matrix_is_standard_opengl_perspective`

- Pins decision #12 (standard right-handed perspective; no
  reverse-Z in v0).
- Input: default camera (fov_y = π/4, aspect = 16/9, near = 0.05,
  far = 100.0).
- Expected (convention #5; tolerance `1e-12` per element since these
  are closed-form expressions of representable inputs and one trig):
  - `m[0][0] = 1 / (16/9 * tan(π/8))`.
  - `m[1][1] = 1 / tan(π/8)`.
  - `m[2][2] = -(100 + 0.05) / (100 - 0.05)`.
  - `m[3][2] = -(2 * 100 * 0.05) / (100 - 0.05)`.
  - `m[2][3] = -1.0`.
  - All other elements `== 0.0` (exact).

### C5. `frame_fit_centers_pivot_on_aabb_centroid`

- Bug class: pivot left at world origin; orbits drift away from the
  model.
- Input: AABB `min = {0.5, 1.5, 2.5}`, `max = {1.5, 2.5, 3.5}`
  (centroid `{1, 2, 3}`).
- Expected: `cam.pivot_world == {1.0, 2.0, 3.0}` exact.

### C6. `frame_fit_distance_satisfies_eighty_percent_of_vertical_fov`

- Pins decision #14 (80% of vertical FOV's *angular* extent —
  resolves rev-1's ambiguous wording).
- **Derivation (cited inline):** for a sphere of radius `r` centered
  on the camera's forward direction at distance `d`, the angle it
  subtends at the camera's apex is `2 * arcsin(r / d)`. Setting the
  subtended angle equal to `0.80 * fov_y` gives
  `d = r / sin(0.40 * fov_y)`.
  We define `r` as the half-diagonal of the AABB
  (`r = 0.5 * length(max_world - min_world)`) so the bounding sphere
  encloses every corner. No additional safety margin in v0 (rev-1's
  trailing `+ aabb_radius` was unjustified and is removed).
- Input: AABB centered at origin with extent `{2, 2, 2}` (half-
  diagonal `r = sqrt(3)`); default camera fov_y = π/4.
- Expected: `cam.distance_m == sqrt(3) / sin(0.40 * π/4)` (within
  `1e-9`).

### C7. `frame_fit_is_idempotent`

- Pins decision #6.
- Input: the C5 AABB (`min = {0.5, 1.5, 2.5}`, `max = {1.5, 2.5,
  3.5}`); default camera; call `frame_fit` twice.
- Expected: `cam.pivot_world[i]`, `cam.distance_m`, `cam.yaw_rad`,
  `cam.pitch_rad` are within `1e-9` of their post-first-call values.
- Notes: reusing C5's AABB makes the post-first-call state hand-
  derivable for debugging (`pivot_world == {1, 2, 3}`,
  `distance_m == sqrt(3*0.5²) / sin(0.40 * π/4)`).

### C8. `view_then_projection_keeps_aabb_corners_inside_clip_space_after_frame_fit`

- End-to-end pin: the composition `proj * view` is consistent with
  `frame_fit` — every corner of a frame-fit AABB lands inside the
  `[-1, +1]^3` clip volume.
- Procedure: take the AABB centered at origin with extent `{2, 2,
  2}` (half-diagonal `r = sqrt(3)`), call `frame_fit(cam, box)`,
  build the 8 corner points `(±1, ±1, ±1)`, for each compute
  `clip = proj * view * (corner, 1)`, divide by `clip.w` to get
  `ndc`.
- **Bound: every NDC component lies in `[-1, +1]`** (within the
  tolerance below). This is the loose-but-correct claim.
- **Worst-corner derivation (cited inline as a debugging aid, not
  asserted):**
  - At `fov_y = π/4`, `aspect = 16/9`, frame-fit puts the camera
    at `d = sqrt(3) / sin(0.40 * π/4) ≈ 5.605` along the
    initial-pose forward axis.
  - The corner closest to the camera (e.g. `(1, 1, 1)`) projects
    to roughly `ndc ≈ (0.295, 0.524, 0.979)` for the default
    `near = 0.05`, `far = 100.0`. Computed by hand once and left
    in the test as a sanity comment so a future regression has a
    reference point.
  - The 80% angular framing pinned by C6 does **not** translate
    linearly to the NDC `y` bound: `ndc.y` for a sphere-surface
    corner at the centroid depth is `tan(0.40 * fov_y) / tan(0.50
    * fov_y) ≈ 0.7844`, not 0.80. (Rev-2 conflated these. C8
    doesn't assert on the 80% — that's C6's job — so the looser
    `[-1, +1]^3` claim is the one that lives here.)
- Tolerance: `1e-6` per component (≥3 matmuls; tolerance policy
  tier "≥ 3 matmuls").

---

## M. Camera mutators (pan / zoom)

### M1. `pan_translates_pivot_in_camera_local_axes`

- Bug class: pan operates in world axes instead of camera-local
  axes; user drags the model laterally and it drifts vertically
  because the camera's yaw is non-zero.
- **"Camera-local +X" pinned:** the world-space direction of the
  camera's *screen-right* vector — equivalently, the inverse-view
  matrix's first column (or, since the rotation block of a view
  matrix is orthonormal, the transpose-view's first row). Per
  glm::lookAtRH: at default pose (eye `(0,0,5)`, target origin, up
  +Y), camera-local +X is world `(1, 0, 0)`. At `yaw_rad = +π/2`
  (eye `(5,0,0)` per C2), camera-local +X is world `(0, 0, -1)`
  — derivation:
  - `f = normalize(target - eye) = (-1, 0, 0)`
  - `s = normalize(cross(f, up)) = normalize(cross((-1,0,0),
    (0,1,0))) = (0, 0, -1)` ← this is the right (screen-right)
    vector.
- Procedure:
  - **Sub-case A (yaw = 0):** default camera, call `pan(cam, 1.0,
    0.0)`. Camera-local +X aligned with world +X. Expected
    `cam.pivot_world == {1.0, 0.0, 0.0}` exact.
  - **Sub-case B (yaw = +π/2):** start from default, set
    `yaw_rad = +π/2`, then call `pan(cam, 1.0, 0.0)`. Per the
    derivation above, camera-local +X is world `(0, 0, -1)`.
    Expected `cam.pivot_world == {0.0, 0.0, -1.0}` (within
    `1e-9` per component).
  - **Sub-case C (vertical pan at yaw = 0):** default camera, call
    `pan(cam, 0.0, 1.0)`. Camera-local +Y is world +Y. Expected
    `cam.pivot_world == {0.0, 1.0, 0.0}` exact.

### M2. `pan_distance_does_not_change`

- Pin: pan moves the pivot, not the eye. Distance from pivot to eye
  is unchanged.
- Input: default camera, `pan(cam, 0.5, 0.5)`.
- Expected: `cam.distance_m == 5.0` exact.

### M3. `zoom_changes_distance_multiplicatively`

- **Decision pin:** `zoom(cam, +1.0)` multiplies `distance_m` by
  `0.9` (zoom in). `zoom(cam, -1.0)` multiplies by `1.0 / 0.9`
  (zoom out). Multiplicative (not additive) zoom keeps the
  perceived speed consistent across scales.
- Procedure:
  - Default camera (`distance_m = 5.0`).
  - Call `zoom(cam, 1.0)`. Expected `distance_m == 5.0 * 0.9 ==
    4.5` (exact — representable).
  - Call `zoom(cam, -1.0)` from the original state. Expected
    `distance_m == 5.0 / 0.9` (within `1e-12`).

### M4. `zoom_clamps_to_positive_distance`

- Boundary case: prevent `distance_m <= 0` (the camera flips inside
  out).
- Procedure: default camera, call `zoom(cam, +1000.0)` (would
  multiply `distance_m` by `0.9^1000 ≈ 1.7e-46`).
- **Decision pin:** distance is clamped at minimum
  `1e-3 m` (`1 mm`) to avoid floating-point underflow and weird
  rendering at sub-mm scales. Test expects
  `cam.distance_m == 1e-3` after the call.

---

## P. Picking

### P1. `picks_cylinder_when_ray_passes_through_axis`

- Per cylinder convention #8: cylinder local origin at the proximal
  cap (`z = 0`), extending to `z = length_m` along +Z.
- Input: snapshot with one cylinder node, `world_from_local =
  identity`, `length_m = 1.0`, `radius_m = 0.1`. Ray from
  `{0, 0, -10}` along `{0, 0, 1}`.
- Expected: hit at node 0; `t_along_ray == 10.0` (entering proximal
  cap at world `z = 0`). Within `1e-9`.

### P2. `misses_cylinder_when_ray_passes_outside_radius`

- Input: same cylinder; ray from `{0.5, 0, -10}` along `{0, 0, 1}`
  (offset 0.5 m from axis; cylinder radius 0.1 m).
- Expected: `pick(...) == std::nullopt`.

### P3. `misses_cylinder_when_ray_origin_past_far_cap_and_direction_is_away`

- Boundary: ray with no positive-t intersection (origin past
  cylinder, direction continuing further away).
- Input: cylinder as in P1 (spans `z ∈ [0, 1]` per #8); ray from
  `{0, 0, 10}` along `{0, 0, 1}`.
- Expected: no hit.

### P3b. `hits_cylinder_when_ray_origin_past_far_cap_but_direction_returns`

- **New in rev 2** — pins half-line semantics from the productive
  side: origin past the cylinder but direction back toward it
  *should* hit (with `t > 0`).
- Input: cylinder as in P1 (spans `z ∈ [0, 1]`); ray from
  `{0, 0, 10}` along `{0, 0, -1}`.
- Expected: hit; `t_along_ray == 9.0` (entering far cap at world
  `z = +1.0`, which is `t = 10 - 1.0 = 9.0` along `-Z`).

### P4. `picks_closer_of_two_cylinders_along_ray`

- Pins decision #7 (closest hit).
- Input: snapshot with two cylinders. Cylinder 0 at world origin
  (spans `z ∈ [0, 1]`); cylinder 1 at `world_from_local =
  translate(0, 0, 5)` (spans `z ∈ [5, 6]`). Ray from
  `{0, 0, -10}` along `{0, 0, 1}`.
- Expected: hit at node 0 (proximal cap at world `z = 0`,
  `t = 10.0`), not node 1 (`z = 5`, `t = 15`).

### P5. `grazing_ray_inside_bounding_sphere_but_outside_cylinder_misses`

- Pins decision #8 (exact ray-vs-primitive, not bounding sphere).
- **Rev-2 fix:** the rev-1 ray actually missed the bounding sphere
  too, so it didn't pin the contract. Recomputed:
  - Cylinder length 1.0, radius 0.1.
  - Bounding sphere radius `r_b = sqrt(0.1² + 0.5²) ≈ 0.5099`.
  - Ray from `{0.3, 0, -10}` along `{0, 0, 1}`. Perpendicular
    distance from cylinder axis is `0.3 m`. This is **inside** the
    bounding sphere (`0.3 < 0.5099`) but **outside** the cylinder
    radius (`0.3 > 0.1`).
- Expected: `pick(...) == std::nullopt`.

### P6. `parallel_ray_at_exactly_the_cylinder_radius_hits` (boundary)

- **Decision pin:** the cylinder is a *closed* shape — the side
  surface (radial distance `== radius_m` exactly) and the cap
  surfaces (`z == 0` and `z == length_m` exactly) belong to the
  shape. A parallel ray whose perpendicular distance from the axis
  equals the radius exactly hits.
- Input: cylinder length 1.0, radius 0.1; ray from
  `{0.1, 0, -10}` along `{0, 0, 1}` — perpendicular distance from
  the axis is exactly `0.1 = radius_m`.
- Expected: hit; `t_along_ray == 10.0` (entering the proximal cap at
  world `z = 0`, `(x, y) = (0.1, 0)` — on the rim of the cap,
  which belongs to the closed shape).
- Bug class: a strict-inequality interior test (`distance_sq <
  radius_sq` instead of `<=`) misses rays exactly at the rim,
  producing flicker when the camera is precisely aligned. Closed-
  shape semantics avoid that.
- Notes: rev 3 changed this from "outside-radius miss" (a verbatim
  duplicate of P2) to "exactly-at-radius hit," per reviewer
  feedback.

### P7. `ray_starting_inside_cylinder_returns_t_near_zero`

- Pins decision #9 (P7 resolution: t == 0 when origin is inside).
- Procedure: cylinder length 1.0 radius 0.5 at origin; ray from
  `{0, 0, 0}` along `{1, 0, 0}`. Origin is interior.
- Expected: hit at node 0; `0.0 <= t_along_ray < 1e-9`. Loosened
  from rev-1 strict equality per reviewer feedback (some interior
  tests evaluate to a tiny positive epsilon under FP).

### P8. `picks_box_primitive_via_slab_test`

- **Rev-2 fix:** box uses `half_extent_x_m`,
  `half_extent_y_m`, `half_extent_z_m` (convention #11).
- Input: box at world origin, half-extents `{0.5, 0.5, 0.5}`. Ray
  from `{-10, 0, 0}` along `{1, 0, 0}`.
- Expected: hit; `t_along_ray == 9.5` (entering `-X` face at
  world `x = -0.5`).

### P8b. `misses_box_when_ray_passes_outside_slab`

- Box-side complement to P2.
- Input: box `{0.5, 0.5, 0.5}` at origin. Ray from
  `{-10, 1.0, 0}` along `{1, 0, 0}` (above the +Y face).
- Expected: no hit.

### P8c. `picks_sphere_when_ray_passes_through_center`

- Coverage-gap fix from the reviewer: sphere had no test in rev 1.
- Input: sphere at origin, `radius_m = 0.5`. Ray from
  `{0, 0, -10}` along `{0, 0, 1}`.
- Expected: hit; `t_along_ray == 9.5` (entering near hemisphere at
  `z = -0.5`).

### P8d. `misses_sphere_when_ray_passes_outside_radius`

- Input: sphere `radius_m = 0.5` at origin. Ray from
  `{1.0, 0, -10}` along `{0, 0, 1}` (offset 1.0 m).
- Expected: no hit.

### P9. `ray_pointing_away_from_primitive_does_not_pick_it`

- Renamed from rev-1's misleading "behind_camera" name.
- Differentiated from P3: P3 has origin past + direction away (no
  intersection at all); P9 has origin in front of the cylinder but
  direction *away* (a forward picker would hit if it allowed
  negative t).
- Input: cylinder; ray from `{0, 0, -10}` along `{0, 0, -1}` (origin
  before cylinder; direction continues away in -Z).
- Expected: `pick(...) == std::nullopt`. Pins half-line semantics
  from the negative side.

### P10. `picking_respects_world_from_local_translation`

- Input: cylinder length 1.0 radius 0.1 at
  `world_from_local = translate(5, 0, 0)` (spans `z ∈ [0, 1]` in
  local; world bounds unchanged in z, x shifted by 5). Ray from
  `{5, 0, -10}` along `{0, 0, 1}`.
- Expected: hit; `t_along_ray == 10.0` (proximal cap at world
  `z = 0`).

### P11. `picking_respects_world_from_local_rotation`

- Pins that picking transforms rays (or primitives) under rotation,
  not just translation.
- Input: cylinder length 1.0 radius 0.1 at
  `world_from_local = rotate(+π/2, axis=+X)`. Local +Z lands on
  world `-Y` per RHR; the cylinder's local span `z ∈ [0, 1]`
  becomes world `y ∈ [-1, 0]` (proximal cap at world `y = 0`,
  far cap at `y = -1`). Ray from `{0, -10, 0}` along `{0, 1, 0}`.
- Expected: hit; `t_along_ray == 9.0` (entering far cap at world
  `y = -1`, `t = -1 - (-10) = 9`).

### P12. `picks_returns_nullopt_for_empty_snapshot`

- Coverage gap from the reviewer.
- Input: `scene_snapshot{}`; arbitrary ray.
- Expected: `std::nullopt`.

---

## F. Fixture-driven end-to-end

### F1. `loads_v0_arm_from_disk_and_builds_a_consistent_snapshot`

- Coverage gap from the reviewer: every other test goes through
  in-memory fixtures, missing JSON-field-name regressions.
- Procedure: call `description::load_from_file(
  "descriptions/v0-arm.json")`; expect success; build the snapshot;
  assert it equals the snapshot built from
  `make_v0_arm_description()`.
- Bug class: a JSON field rename in the production fixture that
  every in-memory test misses because they hand-construct the
  struct directly.

### F2. `transform_identity_returns_the_identity_matrix`

- Coverage gap from the reviewer. Cheap.
- Expected: `transform::identity().m` element-by-element equal to
  the literal identity (1 on diagonal, 0 elsewhere). Exact.

---

## X. Boundary cases

### X1. `frame_fit_handles_degenerate_aabb_with_default_distance`

- Boundary: AABB is a single point (zero extent).
- Procedure: AABB `min == max == {0, 0, 0}`. Call `frame_fit`.
- **Decision pin:** when AABB extent is `< 1e-9` along every axis,
  `frame_fit` falls back to `distance_m = 1.0` and `pivot_world =
  min_world`. Documented in implementation comment + this test.
- Expected: `cam.pivot_world == {0,0,0}`,
  `cam.distance_m == 1.0`, both exact.

### X2. `frame_fit_on_empty_snapshot_aabb_uses_default_distance`

- Composite of A5 + X1 (rev-2 reviewer suggested this). Pins the
  composite contract: `compute_world_aabb(empty_snapshot)` returns
  the A5 sentinel `{0,0,0}-{0,0,0}`, which `frame_fit` then
  recognizes as degenerate (X1) and falls through to the default
  distance.
- Procedure:
  ```cpp
  scene_snapshot empty;
  aabb box = compute_world_aabb(empty);   // A5: {0,0,0}-{0,0,0}
  orbit_camera cam{};
  frame_fit(cam, box);                    // X1: degenerate path
  ```
- Expected: `cam.pivot_world == {0, 0, 0}`,
  `cam.distance_m == 1.0`. Exact.
- Bug class: a future change tightens A5's sentinel (e.g. returns
  a 1-meter unit box instead of zero-volume) without thinking about
  the composite path; X1's degenerate fallback then never fires for
  empty snapshots and the composite breaks. This test pins the
  contract end-to-end.

---

## Coverage I am explicitly NOT testing in v0

- **Renderer pixel output.** Smoke test only.
- **Panel layout / docking.** Smoke test only; data-layer assertions
  cover content.
- **Picking from screen-space coordinates.** Phase VB6 manual rubric
  + smoke test.
- **Live / Replay snapshot producers.** Post-v0; no surface yet.
- **Save flow.** Phase VD test plan.
- **Animation / time.** Out of v0 scope.
- **`description::axis_magnitude` domain check** — covered in the
  loader's test plan; tracked as a follow-up there (see "Loader
  follow-ups" above).

## Open follow-ups for the test-reviewer (rev 2)

The rev-1 decision call-outs are **resolved** in the body above. The
following are new in rev 2 and may benefit from reviewer eyes:

1. **AABB envelope strategy (A4 decision pin)** — over-approximate
   via OBB-corner transform. Acceptable to ship as v0?
2. **Default cylinder radius (S2)** = `0.05 m`, default arrow
   length / radius (S3) = `0.20 m / 0.01 m`. Eyeball-tuned. Should
   the reviewer want bench data, surface that.
3. **Zoom factor (M3 decision pin)** = `0.9` per scroll click. UX
   tuning, not physics; reviewer can wave through if they like.
4. **Distance clamp (M4 decision pin)** = `1e-3 m`. Safe FP floor
   for v0; can revisit when reverse-Z lands.
5. **Pan velocity scale** — not tested in this plan because it's a
   driver-level concern (mouse delta in pixels → camera local-space
   meters is computed from frustum geometry per-frame). Out of unit
   scope; smoke-test rubric.
