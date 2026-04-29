---
name: robot-description-loader
description: Use when working on the robot description loader, the JSON description schema, or any code that consumes the loaded `robot_description` struct (Layer 5 MJCF transpilation, Layer 2 backend CAN ID enumeration, Layer 3 vendor firmware version pinning). Covers the public API, the 13 pinned design decisions, error semantics, schema-version evolution, and the test approach.
---

# Robot description loader

The loader reads `descriptions/<robot-name>.json`, validates it, and
returns an in-memory `robot_description` struct. v0 ships schema
version 1 covering links, joints, motors, sensors. The loader is the
single source of truth for what's in a robot description; downstream
layers consume the struct, never the JSON directly.

## Scope

**In:**
- File I/O for `descriptions/*.json` files.
- JSON parsing (via `nlohmann/json`).
- Type-strict, range-strict, unknown-field-strict validation.
- Cross-reference resolution (joint→link, motor→joint, sensor→joint,
  `"world"` magic-string rules).
- Identifier-conflict detection (per-kind name uniqueness, CAN-ID
  uniqueness across motors ∪ sensors).
- Error reporting with typed kind discriminant + RFC-6901 JSON pointer
  + entity-name-bearing message.

**Out (deferred to other features):**
- MJCF transpilation — Layer 5 reads `robot_description.{links,
  joints}` and emits MJCF for MuJoCo.
- CAN bus enumeration — Layer 2 reads `robot_description.{motors,
  sensors}` to register devices on the modeled bus.
- Vendor firmware-version pinning — Layer 3 reads the per-device
  `controller_firmware_version` to select the right firmware model.
- Schema evolution / forward migration — only one schema version
  exists in v0.
- Sub-system reuse / shared modules (e.g. a "swerve module" definition
  reused across robots) — v1+ feature.
- An authoring GUI / CAD-import tool — deferred (`docs/OPEN_QUESTIONS.md`
  OQ-11).

## Public surface

| Header                         | Contains                                                    |
|--------------------------------|-------------------------------------------------------------|
| `src/description/loader.h`     | `load_from_file(path) -> std::expected<robot_description, load_error>` |
| `src/description/schema.h`     | `link`, `joint`, `motor`, `sensor`, `robot_description`, `joint_type`, `origin_pose`. All POD with `operator== = default`. |
| `src/description/error.h`      | `load_error`, `load_error_kind` (six-kind enum: `file_not_found`, `parse_error`, `schema_error`, `domain_error`, `reference_error`, `conflict_error`). |
| `src/description/origin_pose.h` | `compose_origin(const origin_pose&) -> transform_4x4` — compose raw `xyz_m + rpy_rad` into a 4×4 column-major transform (intrinsic-Z-Y′-X″, URDF order). `decompose_origin(transform_4x4) -> origin_pose` — algebraic inverse (Phase VD); returns one valid solution at the gimbal pole (yaw = 0, residual absorbed into roll). `validate_finite_xyz(xyz, base_ptr)` / `validate_finite_rpy(rpy, base_ptr)` — return `domain_error` on the first non-finite element, or `std::nullopt`. These are the loader's **sole** non-finite-rejection mechanism (D-VC-3 seam). |
| `src/description/serializer.h` | `save_to_file(const robot_description&, path) -> std::expected<void, save_error>`. `save_error_kind::io_error` only (no unreachable `serialization_error`). |

Internal helpers in `src/description/loader.cpp` are not public API
and may be refactored without semver impact.

## Pinned design decisions

These were committed to during the test-reviewer cycle and are the
contract the loader honors. The full rationale for each lives in
`tests/description/TEST_PLAN.md`'s "Decisions pinned" section; what's
listed here is what implementers and consumers need to internalize.

1. **JSON pointer convention.** Errors point to the path that *should
   have held* the offending value (so missing fields yield
   `/links/0/mass_kg`, not `""`). `parse_error` and `file_not_found`
   use `""`.
2. **Conflict reporting picks the second offender deterministically.**
   When two locations equally describe a conflict (duplicate CAN IDs,
   duplicate names), the validator always reports the second (the one
   whose introduction created the conflict). Same input → same error
   every time. Pinned by `H2_error_reporting_is_deterministic`.
3. **`viscous_friction_nm_per_rad_per_s` is required.** No silent
   zero-defaulting. "No defaults — explicit or rejected."
4. **Root keys `motors` and `sensors` are required.** Empty arrays
   accepted; missing keys rejected. Same no-defaults rationale.
5. **Integer fields reject integer-valued floats.**
   `controller_can_id: 1.0` is a `schema_error`. JSON-literal *form*
   matters, not just numeric value.
6. **Float fields accept JSON integer literals.** Asymmetric to #5:
   `mass_kg: 2` parses to `2.0`. RFC 8259 doesn't distinguish, and
   requiring the trailing `.0` would be gratuitous.
7. **Equal joint limits accepted, inverted rejected.**
   `lower == upper` declares a fixed joint expressed as revolute with
   zero range. `lower > upper` is a typo.
8. **Unknown fields rejected** at every nesting level. The strongest
   "no shortcuts" guard in the loader — typo'd field names like
   `gear_ration` are rejected, not silently dropped.
9. **`"world"` is parent-only.** A joint can have `parent: "world"`;
   `child: "world"` is rejected.
10. **CAN ID range `[0, 63]` inclusive.** FRC CAN is 6-bit. Will
    tighten with citations and validator changes when the bus model
    lands.
11. **Per-kind name uniqueness; cross-kind reuse allowed.** Two motors
    named `"shoulder"` is an error. Motor `"shoulder"` + sensor
    `"shoulder"` is fine — they live in different lookup tables.
12. **Case-sensitive enum strings.** `joints[0].type = "Revolute"`
    rejected; `"revolute"` is the only valid form.
13. **`robot_description` carries `operator== = default`** on every
    sub-type. Used by determinism tests and by Layer 5 fixture
    comparisons.

## Error semantics

`load_error` carries:
- `kind` — one of the six `load_error_kind` values. **Tests pin on
  this**, never on message phrasing.
- `file_path` — the input path (always set, including for
  `parse_error` / `file_not_found`).
- `json_pointer` — RFC-6901 pointer per decision #1.
- `message` — human-readable. Contains the names of entities involved
  (field name for schema/domain errors; both entity names for
  reference/conflict errors). Does *not* contain validator-vocabulary
  words like "expected", "must", "type" — assertions on those would
  pin phrasing rather than contract.

## Schema evolution

Schema versions 1 and 2 are accepted. The gate fires before any
field-level validation. Version 3+ produces a `schema_error` at
`/schema_version` with the offending number in the message.

**schema_version 2 adds optional `link.visual_origin` and
`joint.origin`** (each `{xyz_m: [...], rpy_rad: [...]}`, intrinsic-Z-Y′-X″
Euler, URDF order). Stored raw on the in-memory struct (`origin_pose`);
composition into a 4×4 transform happens at the snapshot-builder
use-site (D-VC-5a/5b). Both default to `origin_pose{}` (identity) when
absent in the JSON. Under schema_version 1, these keys are rejected as
unknown fields (v1 decision #8 applies unconditionally).

**Version 1 (current default):**

Schema version 1 gate behavior:

- Missing `schema_version` → `schema_error` at `/schema_version`.
- Unsupported version (anything other than `1`) → `schema_error` at
  `/schema_version`, message contains the offending number.
- Wrong type (`"1"`, `1.0`, etc.) → `schema_error` at
  `/schema_version`.

When v2 lands, bump the version and add the v2 fields *additively*.
Do not remove or change the meaning of existing fields. v0/v1 has no
forward-migration story; old files become invalid against new
versions and the user updates them. Keep this story honest until a
user actually has a multi-robot codebase that needs migration.

## Validation status

The loader is a structural feature, not a physical model — there's no
real-world quantity to compare against. "Validation" here means: the
83-test suite in `tests/description/loader_test.cpp` passes under
clang Debug, gcc Debug, and clang ASAN+UBSAN. Coverage organized in
`tests/description/TEST_PLAN.md` sections A through M.

When changing the loader, the test plan is the source of truth. If a
test needs substantive change (new case, different assertion), re-run
the test-reviewer agent on the change before writing the new test
code. Drift between approved-plan and shipped-test defeats the gate.

## Known limits

- **Save flow is non-atomic in v0.** `save_to_file` writes in-place via
  `std::ofstream` + close. A partial write on disk-full leaves a corrupted
  file. See D-VC-11 in `tests/description/TEST_PLAN_VC.md`. Atomic save
  (write-to-temp + rename) is a v1+ concern; it will land with a positive
  test of the new contract.
- **NaN / Infinity in numeric fields.** Strict JSON has no `NaN` /
  `Infinity` literals; the loader uses `nlohmann/json`'s default
  strict parser, so these aren't reachable from a JSON file. The
  `validate_finite_xyz` / `validate_finite_rpy` helpers are the loader's
  sole non-finite guard (D-VC-3); they exist as a defensive seam for any
  future in-process path (e.g. visualizer gizmo flow) that might write
  a non-finite value into an `origin_pose`.
- **Very large numbers.** `mass_kg: 1e308` parses fine; downstream
  MuJoCo would explode. The loader does not cap upper bounds. Add
  range upper-bounds when a real failure mode demands them.
- **Unicode in `name` fields.** Loader treats names as opaque UTF-8
  strings — no normalization, no length cap, no character-class
  restriction. Two names that differ only by Unicode normalization
  form will be treated as distinct.
- **Multiple errors per document.** Only the first error is reported;
  the user iterates. Pinned by H2 (deterministic which error fires).
  When users start hitting the "fix one, find the next" friction at
  scale, consider a "report all" mode — but the determinism test
  needs a parallel update.
- **`motor_model` / `controller` / `sensor_model` enum strictness.**
  These are treated as opaque non-empty strings in v0. Layer 3/4 will
  ship registries of valid values when the vendor model implementations
  land; the loader will tighten in lockstep at that point.

## Open follow-ups

- **Domain checks for upper bounds.** Currently we reject obviously
  invalid lower bounds (negative mass, etc.) but accept arbitrarily
  large values. When a real bound is identified (e.g. MuJoCo's safe
  inertia range), wire it in here with a citation.
- **Multi-bus CAN.** Real FRC robots can run multiple CAN buses
  (RIO's CAN0 + CANivore's CAN1+). The current schema treats
  `controller_can_id` as a single global address space. Multi-bus
  support is tracked in `docs/OPEN_QUESTIONS.md` OQ-8 and will require
  a schema-version bump.
- **`load_from_string` / `load_from_json`.** Tests use a temp-file
  helper because the only public entry point is `load_from_file`.
  Adding an in-memory overload would let tests skip the filesystem
  hop, but `load_from_file` is the production entry point so the
  current shape is correct for v0.

## Cross-references

- `tests/description/TEST_PLAN.md` — v1 approved test plan (83 tests).
- `tests/description/TEST_PLAN_VC.md` — v2 + serializer approved test plan (Phase VC).
- `docs/V0_PLAN.md` Phase B — the implementation plan.
- `docs/ARCHITECTURE.md` "Robot description format" — the schema
  sketch (now showing v2 form).
- `.claude/skills/layer-5-world-physics.md` — downstream consumer
  (geometry → MJCF).
- `.claude/skills/layer-2-control-system-backend.md` — downstream
  consumer (CAN ID enumeration).
- `.claude/skills/layer-3-device-bus-vendor-firmware.md` — downstream
  consumer (firmware version pinning).
