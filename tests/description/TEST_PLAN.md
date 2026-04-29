# Robot description loader — approved test plan

**Status:** `ready-to-implement` per the `test-reviewer` agent (two
review rounds, 2026-04-29). Implement the tests in this file exactly
as specified; if substantive deviation is needed during implementation,
re-run the test-reviewer cycle on the change rather than drifting
silently. See `.claude/skills/tdd-workflow.md` step 6.

**Implements:** `docs/V0_PLAN.md` Phase B (steps B5–B9).

**Implementer's first action (next session):** read this document, then
write the failing tests in this directory, then confirm they fail for
the *right* reason before writing the loader.

---

## Public API

```cpp
namespace robosim::description {

enum class load_error_kind {
  file_not_found,    // path does not resolve to a regular file
  parse_error,       // JSON syntax invalid
  schema_error,      // structural: missing field, wrong type, unsupported schema_version
  domain_error,      // value out of allowed range (negative mass, CAN ID > 63, etc.)
  reference_error,   // cross-reference to a name that doesn't exist
  conflict_error,    // duplicate identifier (CAN ID or name) across the address space
};

struct load_error {
  load_error_kind kind;
  std::filesystem::path file_path;
  std::string json_pointer;   // RFC 6901; see "JSON pointer convention" below
  std::string message;        // human-readable; tests assert only on entity names, never on phrasing words
};

struct robot_description {
  // POD struct, defaulted operator==, see "Determinism" below.
  // Sub-types (link, joint, motor, sensor) likewise = default on operator==.
};

[[nodiscard]] std::expected<robot_description, load_error>
load_from_file(const std::filesystem::path& path);

}  // namespace robosim::description
```

---

## Decisions pinned (do not re-litigate during implementation)

These were committed to during the test-reviewer cycle. The validator
implements each one; the tests in this document pin each one against
that implementation.

1. **JSON pointer convention.** For schema/domain/reference/conflict
   errors, `json_pointer` is the RFC-6901 path that *should have held*
   the offending value (so missing-field errors produce
   `/links/0/mass_kg`, not `""`). For `parse_error` and
   `file_not_found`, `json_pointer` is `""`.
2. **Conflict location, deterministic second-offender.** When two
   locations equally describe a conflict (duplicate CAN IDs, duplicate
   names), the validator always reports the *second* offender — the
   one whose introduction created the conflict. Tests pin a specific
   pointer (the second one's), not `AnyOf({first, second})`. This keeps
   error reporting deterministic (test H2 pins this property).
3. **`viscous_friction_nm_per_rad_per_s` is required**, not optional
   with a default. Consistent with `code-style.md`'s "no defaults —
   explicit or rejected."
4. **Root keys `motors` and `sensors` are required.** Empty arrays
   (`"motors": []`) are accepted; missing keys are rejected. Same
   no-defaults rationale as #3.
5. **Integer-valued floats in integer fields are rejected.**
   `controller_can_id: 1.0` is a `schema_error`, even though it equals
   1. nlohmann/json's number-type discrimination is the contract:
   integer literal vs. float literal matters.
6. **JSON integer literals in float fields are accepted.**
   `mass_kg: 2` parses to `2.0`. RFC 8259 doesn't distinguish, and
   requiring users to write `2.0` instead of `2` is gratuitous.
7. **Equal joint limits accepted.** `limit_lower_rad == limit_upper_rad`
   declares a fixed joint expressed as revolute with zero range — a
   deliberate construction, not a typo. `lower > upper` is rejected
   (J2).
8. **Unknown fields rejected.** Any extraneous key in any object
   produces `schema_error`. Typo'd field names like `gear_ration`
   instead of `gear_ratio` are exactly the silent-bug class
   `CLAUDE.md`'s "no shortcuts" non-negotiable forbids.
9. **`"world"` is valid only as joint `parent`**, never `child`. The
   asymmetry is deliberate (the world frame can't be a leaf).
10. **CAN ID range `[0, 63]` inclusive.** FRC CAN uses a 6-bit device
    ID. Will tighten with citations and validator changes when the bus
    model lands.
11. **Per-kind name uniqueness; cross-kind reuse allowed.** A motor and
    a sensor with the same name are unambiguous (different lookup
    tables). Two motors with the same name are not.
12. **Case-sensitive enum strings.** `joints[0].type = "Revolute"` is
    rejected; `"revolute"` is the only valid form. (Same will apply to
    `motor_model` / `sensor_model` / `controller` once those enum
    registries land in Layer 3/4 work.)
13. **`robot_description` requires a defaulted `operator==`** on every
    sub-type. Used by H1 / H2; also useful for Layer 5 fixture
    comparisons.

### Coverage gaps deferred (document in the loader skill)

- NaN / Infinity in numeric fields. Strict JSON has no such literal;
  we use the strict parser default. If a future need arises (lenient
  parsing, in-memory variant), add a test then.
- Very large numbers (`mass_kg: 1e308`). Parses fine; would break
  MuJoCo. Out of v0 scope; documented as a known loader limit.
- Unicode in `name` fields. Loader treats names as opaque UTF-8
  strings; no special handling. No test required.
- Multiple validation errors per document — only the first is reported;
  the user iterates. Pinned by H2 (deterministic which error fires).

---

## Cross-cutting assertion shape

Every error test asserts these in this order:

1. `result.has_value() == false`
2. `result.error().kind == <expected load_error_kind>`
3. `result.error().json_pointer == <expected RFC-6901 string>`
4. `result.error().file_path == <input path>`

Plus, where the user's actionable information is the name of an entity
in their file (field name, device name, dangling reference target),
the test additionally asserts:

5. `result.error().message` contains those names as substrings
   (case-sensitive — names are user-provided).

Tests do **not** assert on phrasing words ("expected", "schema",
"syntax", "not found", "must be", etc.). `kind` carries the contract;
messages carry the locator data.

---

## Test layout

- All tests in `tests/description/`. Suite name `RobotDescriptionLoader`.
- GoogleTest. Parameterized cases via `INSTANTIATE_TEST_SUITE_P`.
- Test fixtures use `std::filesystem::temp_directory_path()`; each test
  cleans up its temp files (RAII helper).
- A small `make_v0_arm_description()` helper returns the canonical v0
  fixture as a `nlohmann::json` value. Cases derive from it via
  `erase` / `at(...) = ...` to keep the deltas obvious.
- The shipped fixture file at `descriptions/v0-arm.json` is the
  source-of-truth that A1 loads directly. Other tests serialize a
  modified `nlohmann::json` to a temp file and load that.

---

## A. Happy path

### A1. `loads_minimal_valid_v0_arm`

- Layer/contract: full `load_from_file` round-trip on the production
  fixture; pins every leaf field of `robot_description` schema v1.
- Bug class: misnamed JSON key not detected by validator; accidental
  defaulting of an "optional" field that should be required;
  off-by-one indexing into arrays; FP literal-to-double truncation;
  reference resolution mix-ups (motor's `joint` resolves to wrong
  joint).
- Inputs: `descriptions/v0-arm.json` — the production fixture, as
  written per `docs/ARCHITECTURE.md` "Robot description format" sketch
  (single revolute shoulder joint, one link `arm`, one Kraken motor,
  one CANcoder).
- Expected: every leaf field asserted explicitly. Vectors size 1;
  per-field assertions on `[0]`. Cross-references resolve:
  `motors[0].joint == "shoulder"`, `sensors[0].joint == "shoulder"`.
- Tolerance: zero. FP fields use exact equality. Inline comment in the
  test justifies it: the v0 sketch numbers (`2.0`, `0.5`, `0.166`,
  `100.0`, `0.1`, `1.5708`) are all exactly representable as `double`,
  and `nlohmann::json` round-trips representable JSON literals
  losslessly. Any deviation = bug.
- Implicit pins: `"world"` accepted as joint parent (E4);
  `viscous_friction_nm_per_rad_per_s` accepted when present (its
  optionality is pinned by C1).

### A1b. `accepts_json_integer_literal_in_float_field`

- Pins decision #6.
- Input: in-memory variant of A1 with `links[0].mass_kg: 2` (JSON
  integer literal, no decimal).
- Expected: success; `desc.links[0].mass_kg == 2.0`.

---

## B. Schema version

### B1. `rejects_when_schema_version_unsupported`

- Bug class: silent acceptance of unknown future version.
- Input: A1 fixture with `schema_version: 999`.
- Expected: `kind == schema_error`, `json_pointer == "/schema_version"`,
  message contains `"999"`.

### B1b. `schema_version_gate_runs_before_field_validation`

- Pins error precedence: a fixture with both `schema_version: 999` and
  a missing required field returns the schema_version error, not the
  missing-field error.
- Bug class: a refactor that runs validation in a different order
  silently re-routes errors and breaks user-actionable diagnostics.
- Input: A1 fixture with `schema_version: 999` AND `links[0].mass_kg`
  removed.
- Expected: `kind == schema_error`, `json_pointer == "/schema_version"`
  (the version error wins), message contains `"999"`.

(The `schema_version` *missing* case is in C1 below.)

---

## C. Required-field validation

### C1. `rejects_when_required_field_missing` (parameterized)

- Bug class: validator skips a field via copy-paste oversight;
  partially-initialized struct propagates default zero/empty
  downstream.
- Parameterization: `(field_path_to_remove, expected_json_pointer)`.
  Each case takes the A1 in-memory fixture, removes the named field
  via `nlohmann::json::erase`, expects `kind == schema_error`,
  `json_pointer == expected`, message contains the leaf field name.
- Cases (exhaustive against the v0 schema, including root keys per
  decision #4):

  | Field path                                                 | Expected pointer                                  |
  |------------------------------------------------------------|---------------------------------------------------|
  | `schema_version`                                           | `/schema_version`                                 |
  | `name`                                                     | `/name`                                           |
  | `links`                                                    | `/links`                                          |
  | `joints`                                                   | `/joints`                                         |
  | `motors`                                                   | `/motors`                                         |
  | `sensors`                                                  | `/sensors`                                        |
  | `links/0/name`                                             | `/links/0/name`                                   |
  | `links/0/mass_kg`                                          | `/links/0/mass_kg`                                |
  | `links/0/length_m`                                         | `/links/0/length_m`                               |
  | `links/0/inertia_kgm2`                                     | `/links/0/inertia_kgm2`                           |
  | `joints/0/name`                                            | `/joints/0/name`                                  |
  | `joints/0/type`                                            | `/joints/0/type`                                  |
  | `joints/0/parent`                                          | `/joints/0/parent`                                |
  | `joints/0/child`                                           | `/joints/0/child`                                 |
  | `joints/0/axis`                                            | `/joints/0/axis`                                  |
  | `joints/0/limit_lower_rad`                                 | `/joints/0/limit_lower_rad`                       |
  | `joints/0/limit_upper_rad`                                 | `/joints/0/limit_upper_rad`                       |
  | `joints/0/viscous_friction_nm_per_rad_per_s`               | `/joints/0/viscous_friction_nm_per_rad_per_s`     |
  | `motors/0/name`                                            | `/motors/0/name`                                  |
  | `motors/0/motor_model`                                     | `/motors/0/motor_model`                           |
  | `motors/0/controller`                                      | `/motors/0/controller`                            |
  | `motors/0/controller_can_id`                               | `/motors/0/controller_can_id`                     |
  | `motors/0/controller_firmware_version`                     | `/motors/0/controller_firmware_version`           |
  | `motors/0/joint`                                           | `/motors/0/joint`                                 |
  | `motors/0/gear_ratio`                                      | `/motors/0/gear_ratio`                            |
  | `sensors/0/name`                                           | `/sensors/0/name`                                 |
  | `sensors/0/sensor_model`                                   | `/sensors/0/sensor_model`                         |
  | `sensors/0/controller_can_id`                              | `/sensors/0/controller_can_id`                    |
  | `sensors/0/controller_firmware_version`                    | `/sensors/0/controller_firmware_version`          |
  | `sensors/0/joint`                                          | `/sensors/0/joint`                                |

  Note: cases erase object members only, never array elements (so the
  removal doesn't shift sibling indices). Document this in the helper
  so a future case-adder doesn't trip.

---

## D. Type-mismatch validation

### D1. `rejects_when_field_type_mismatched` (parameterized)

- Bug class: silent corruption — `"1"` becomes 0, `"true"` becomes 1,
  scalar where array expected default-constructs. nlohmann/json's
  permissive coercion is *not* relied on.
- Parameterization: `(field_path, replacement_value, expected_pointer)`.
  Each case starts from A1 in-memory fixture, replaces the value,
  expects `kind == schema_error`, `json_pointer == expected`, message
  contains the leaf field name.

  | Field path                       | Replacement              | Why this case                                  |
  |----------------------------------|--------------------------|------------------------------------------------|
  | `schema_version`                 | `1.0` (float)            | float in integer field                         |
  | `links/0/mass_kg`                | `"two"` (string)         | string in number field                         |
  | `joints/0/axis`                  | `1.0` (scalar)           | scalar in array field                          |
  | `joints/0/axis`                  | `"y"` (string)           | string in array field (different shape)        |
  | `joints/0/axis`                  | `[1.0, 0.0]` (length 2)  | wrong-length array                             |
  | `motors/0/controller_can_id`     | `"1"` (string)           | string in integer field                        |
  | `motors/0/controller_can_id`     | `1.5` (non-integer float)| non-integer in integer field                   |
  | `motors/0/controller_can_id`     | `1.0` (integer-valued float)| integer-valued float still rejected (decision #5) |
  | `motors/0/gear_ratio`            | `true` (bool)            | bool in number field                           |

- Tests do not assert that the message names the expected type. Kind +
  pointer + field-name-substring is the contract.

---

## E. Reference integrity

### E1. `rejects_motor_referencing_unknown_joint`

- Input: A1 with `motors[0].joint = "no_such_joint"`.
- Expected: `kind == reference_error`,
  `json_pointer == "/motors/0/joint"`, message contains
  `"shoulder_motor"` and `"no_such_joint"`.

### E2. `rejects_sensor_referencing_unknown_joint`

- Symmetric: `sensors[0].joint = "no_such_joint"`.

### E3. `rejects_joint_referencing_unknown_link_via_child`

- Input: A1 with `joints[0].child = "no_such_link"`.
- Expected: `kind == reference_error`,
  `json_pointer == "/joints/0/child"`, message contains `"shoulder"`
  and `"no_such_link"`.

### E3b. `rejects_joint_referencing_unknown_link_via_parent`

- Pins that the validator checks `parent`, not just `child`.
- Fixture construction (call out explicitly so no shadow validation
  error fires first):
  - Add a second link `forearm` to `links[]` (mass / length / inertia
    same as `arm` for simplicity).
  - Add a second joint `joints[1]`:
    `name = "elbow"` (distinct from `"shoulder"`),
    `parent = "no_such_link"`,
    `child = "forearm"`,
    valid `axis`, `limit_*`, `viscous_friction_*`, `type = "revolute"`.
  - Keep `motors[]` and `sensors[]` referencing the original
    `"shoulder"` joint (no extras needed).
- Expected: `kind == reference_error`,
  `json_pointer == "/joints/1/parent"`, message contains `"elbow"` and
  `"no_such_link"`.

### E4. `accepts_world_as_special_parent_in_joint`

- Documentation test of the `"world"` magic-string contract. Already
  implicit in A1, but kept as an explicit pin against a future
  refactor that "tightens everything" and drops the special case.
- Input: A1 fixture as-is.
- Expected: success.

### E4b. `rejects_world_as_joint_child`

- Pins decision #9 (asymmetry).
- Input: A1 with `joints[0].child = "world"`.
- Expected: `kind == reference_error`,
  `json_pointer == "/joints/0/child"`, message contains `"world"` and
  `"shoulder"`.

---

## F. Conflicts

Per decision #2: validator reports the *second offender*.

### F1. `rejects_two_motors_with_same_can_id`

- Input: A1 extended to two motors. Motor 0: original
  `shoulder_motor` (`controller_can_id: 1`). Motor 1: a copy with a
  distinct `name` (e.g. `"shoulder_motor_2"`) but
  `controller_can_id: 1`. Both reference `joints[0]`.
- Expected: `kind == conflict_error`,
  `json_pointer == "/motors/1/controller_can_id"` (the second
  offender), message contains both motor names and `"1"`.

### F2. `rejects_motor_and_sensor_with_same_can_id`

- Pins cross-kind CAN-ID address-space unification.
- Input: A1 with `sensors[0].controller_can_id = 1` (matches the
  motor's ID).
- Expected: `kind == conflict_error`,
  `json_pointer == "/sensors/0/controller_can_id"` (sensor is
  declared after motor → sensor is the second offender), message
  contains motor name, sensor name, and `"1"`.

### F3. `rejects_two_sensors_with_same_can_id`

- Symmetric to F1 across `sensors[]`. Pointer:
  `/sensors/1/controller_can_id`.

---

## G. File and JSON-level errors

### G1. `rejects_when_file_not_found`

- Input: a path under the test temp dir that has been verified not to
  exist.
- Expected: `kind == file_not_found`, `json_pointer == ""`,
  `file_path == <input>`. Message non-empty (no substring assertion —
  wording differs across OSes).

### G2. `rejects_when_json_syntactically_invalid` (parameterized)

- Bug class: parser silently swallows malformed input or only handles
  EOF-style truncation.
- Two cases:
  - File contents: `{` (single-byte truncation).
  - File contents: `{"name": }` (missing value, multi-character
    malformed).
- Expected: `kind == parse_error`, `json_pointer == ""`,
  `file_path == <input>`. Message non-empty.

---

## H. Determinism

### H1. `loads_deterministically`

- Project non-negotiable #5 — same input → same output — at the loader
  level.
- Bug class: any non-determinism in parsing — `unordered_map`
  iteration leaking into a vector, locale-dependent FP parsing,
  hash-randomization seed.
- Procedure: load `descriptions/v0-arm.json` 8 times into a
  `std::vector<robot_description>`; assert every entry equals the
  first via the defaulted `operator==`.
- Tolerance: zero. Structs equal or fail.
- Notes: 8 loads (rather than 2) is cheap and shakes out
  hash-iteration variance some implementations introduce after a
  warm-up period.

### H2. `error_reporting_is_deterministic`

- Pins decision #2 from the validator side: same invalid document, same
  reported error every time.
- Bug class: hash-randomization affects which dangling reference fires
  first; user sees flaky error locations across runs.
- Procedure: load an invalid in-memory document (use the F2 fixture —
  motor + sensor sharing CAN ID 1) 8 times; assert every result has
  the same `(kind, json_pointer)` tuple.
- Tolerance: zero.

---

## I. Empty / missing arrays

### I1. `accepts_description_with_empty_motors_and_sensors`

- Pins that empty `motors: []` and `sensors: []` are accepted (a
  passive linkage is a valid v0 robot — though not the demo).
- Input: A1 with `motors: []` and `sensors: []`.
- Expected: success; `desc.motors.empty() && desc.sensors.empty()`.

### I2. `rejects_when_links_empty`

- Input: A1 with `links: []`.
- Expected: `kind == schema_error`, `json_pointer == "/links"`,
  message contains `"links"`.

### I3. `rejects_when_joints_empty`

- Input: A1 with `joints: []`, `motors: []`, `sensors: []` (the
  device emptying prevents reference errors firing first).
- Expected: `kind == schema_error`, `json_pointer == "/joints"`,
  message contains `"joints"`.

(Missing root keys — as opposed to empty arrays — are covered by C1.)

---

## J. Domain-range validation

### J1. `rejects_when_value_outside_allowed_range` (parameterized)

- Bug class: physically nonsensical value reaches MuJoCo / the bus
  model and produces undefined behavior or arbitrary collisions.
- Parameterization: `(field_path, replacement_value, expected_pointer)`.

  | Field path                       | Replacement | Why                                                    |
  |----------------------------------|-------------|--------------------------------------------------------|
  | `links/0/mass_kg`                | `-2.0`      | mass must be > 0                                       |
  | `links/0/mass_kg`                | `0.0`       | zero mass also rejected (symmetry with inertia)        |
  | `links/0/inertia_kgm2`           | `0.0`       | inertia must be > 0 (MuJoCo divides by it)             |
  | `links/0/length_m`               | `-0.5`      | length must be >= 0                                    |
  | `motors/0/controller_can_id`    | `-1`        | CAN ID must be in [0, 63]                              |
  | `motors/0/controller_can_id`    | `64`        | CAN ID must be in [0, 63]                              |
  | `motors/0/gear_ratio`            | `0.0`       | gear ratio must be > 0                                 |
  | `motors/0/gear_ratio`            | `-1.0`      | negative also rejected                                 |

- Expected: `kind == domain_error`, named pointer, message contains
  the leaf field name.

### J1b. `accepts_can_id_boundaries` (parameterized, positive)

- Pins decision #10's inclusive-range commitment.
- Two cases: `motors[0].controller_can_id = 0`; and (separately)
  `controller_can_id = 63`. Sensor's CAN ID adjusted to a non-conflicting
  value in each case.
- Expected: success.

### J2. `accepts_equal_joint_limits_but_rejects_inverted` (two cases)

- Pins decision #7.
- Case A — accepted: `joints[0].limit_lower_rad = 0.5`,
  `limit_upper_rad = 0.5`. Expected: success.
- Case B — rejected: `joints[0].limit_lower_rad = 1.0`,
  `limit_upper_rad = -1.0`. Expected: `kind == domain_error`,
  `json_pointer == "/joints/0/limit_upper_rad"` (the second-offender
  rule applies — `upper` is parsed after `lower`), message contains
  joint name `"shoulder"`.

---

## K. Duplicate names

### K1. `rejects_when_duplicate_name_within_kind` (parameterized)

- Pins decision #11.
- Bug class: name lookup collides; reference resolution silently picks
  one.
- Critical fixture rule: motors/sensors duplicate-name fixtures use
  *distinct* CAN IDs (and *distinct* references where applicable) so
  the duplicate-name conflict is the *only* conflict in the document.
  Otherwise the test could pass for the wrong reason (the validator
  reports a CAN-ID conflict instead of the name conflict, the kind
  assertion still holds, but the pointer assertion would catch it
  noisily).
- Cases (each adds a distinctly-typed second entry with a duplicated
  name; per decision #2, validator reports the second offender):

  | Kind        | Fixture delta                                                                                          | Expected pointer                  |
  |-------------|--------------------------------------------------------------------------------------------------------|-----------------------------------|
  | `links`     | Add `links[1]` named `"arm"` (duplicate of `links[0]`); add a second joint `"elbow"` with parent=`"arm"`, child=`"arm"` (or any consistent extension that doesn't introduce other errors). | `/links/1/name`                   |
  | `joints`    | Add `joints[1]` named `"shoulder"` (duplicate); reference different valid endpoints if needed.        | `/joints/1/name`                  |
  | `motors`    | Add `motors[1]` named `"shoulder_motor"` (duplicate); CAN ID `2` (not `1`); references `"shoulder"`. | `/motors/1/name`                  |
  | `sensors`   | Add `sensors[1]` named `"shoulder_encoder"` (duplicate); CAN ID `3` (not `2`); references `"shoulder"`. | `/sensors/1/name`                 |

- Expected for all cases: `kind == conflict_error`, message contains
  the duplicated name.
- Cross-kind name reuse is *not* tested as a conflict (decision #11):
  a motor and a sensor with the same name are unambiguous.

---

## L. Enum-like field strictness

### L1. `rejects_when_enum_field_has_wrong_case`

- Pins decision #12.
- Input: A1 with `joints[0].type = "Revolute"` (uppercase R).
- Expected: `kind == schema_error`,
  `json_pointer == "/joints/0/type"`, message contains `"Revolute"`.

### L2. `rejects_when_enum_field_has_unknown_value`

- Input: A1 with `joints[0].type = "wibble"`.
- Expected: `kind == schema_error`,
  `json_pointer == "/joints/0/type"`, message contains `"wibble"`.

(`motor_model`, `controller`, and `sensor_model` are also enum-like
but their valid-value registries land with Layer 3/4 vendor models in
later sessions. For v0 the loader treats them as opaque non-empty
strings. L1/L2 cover only `joints[0].type`.)

---

## M. Unknown-field rejection

### M1. `rejects_when_unknown_field_present` (parameterized)

- Pins decision #8 — the strongest "no shortcuts" guard in the suite.
- Bug class: typo'd field names (`gear_ration` instead of
  `gear_ratio`, `mass_g` instead of `mass_kg`) silently dropped,
  producing a struct that uses defaults the user didn't ask for.
- Parameterization: `(parent_path, unknown_key, expected_pointer)`.
  Each case takes A1, adds the unknown key to the named parent
  object, expects `kind == schema_error`, `json_pointer == expected`,
  message contains the unknown key name.

  | Parent       | Unknown key       | Expected pointer                  |
  |--------------|-------------------|-----------------------------------|
  | root         | `extra_root_key`  | `/extra_root_key`                 |
  | `links/0`    | `mass_g`          | `/links/0/mass_g`                 |
  | `joints/0`   | `axisz`           | `/joints/0/axisz`                 |
  | `motors/0`   | `gear_ration`     | `/motors/0/gear_ration`           |
  | `sensors/0`  | `cancoder_thing`  | `/sensors/0/cancoder_thing`       |

---

## Coverage I am explicitly NOT testing in v0

(Repeated from the "deferred" list above for the implementer's
benefit.)

- Sub-system reuse / shared modules — v1+ feature.
- Schema-version forward migration — only one version exists.
- Performance — loader runs once at startup.
- Concurrency — single-threaded at startup.
- I/O failures other than file-not-found (permissions, disk full).
- The MJCF transpilation step — separate Layer 5 feature.
- Cross-kind name uniqueness — allowed (decision #11).
- `motor_model` / `controller` / `sensor_model` enum strictness —
  Layer 3/4 concern.
- NaN / Infinity numeric literals — strict JSON has none; deferred.
- Very-large-number overflow — out of v0 scope.
- Unicode in `name` fields — opaque-string default; no test required.
- Multi-error reporting — only first error returned; H2 pins which
  one fires deterministically.
