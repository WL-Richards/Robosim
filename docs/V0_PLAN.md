# v0 implementation plan

This document is the **entry point for the first implementation
session**. It assumes a fresh Claude Code session with no prior
context. All foundational design decisions are recorded — your job is
to read them, then execute Phase A (scaffold), then begin Phase B
(first feature) under TDD discipline.

## Read first, in this order

1. **`CLAUDE.md`** — repo orientation, non-negotiables, the rule about
   reading skills before implementing.
2. **`docs/ARCHITECTURE.md`** — the binding architecture. Process
   model, tier hierarchy, HAL boundary protocol, time discipline,
   v0 scope, language and toolchain, physics engine, robot
   description format.
3. **`docs/OPEN_QUESTIONS.md`** — status board at the top. v0-gating
   questions are all decided; their resolutions are in
   `ARCHITECTURE.md`. The original framings remain for context.
4. **`.claude/skills/code-style.md`** — C++ style rules, naming,
   determinism rules, error-handling discipline.
5. **`.claude/skills/tdd-workflow.md`** — the TDD cycle and the
   `test-reviewer` agent gate.
6. **`.claude/skills/adding-a-feature.md`** — the workflow every
   feature follows.
7. **The relevant layer skill(s)** for what you're touching.
8. **`docs/REFERENCES.md`** — skim. Look for relevant entries when
   you cite constants or behaviors.

The `test-reviewer` agent at `.claude/agents/test-reviewer.md` is
mandatory before any test code is written. **Do not skip the review
step.**

## Repo

- Remote: `git@github.com:WL-Richards/Robosim.git`
- Not yet initialized locally. First task in Phase A.

## Phase A — Project scaffold

No TDD step here; this is engineering infrastructure, not a feature.
Each numbered task is a candidate commit. Confirm with the user
before pushing the first commit if uncertain.

### A1. Initialize the repo

- `git init` in `/home/will/Documents/frc/robosim/`.
- Add the remote: `git remote add origin git@github.com:WL-Richards/Robosim.git`.
- Default branch: `main`.
- Initial `.gitignore` covering: `build/`, CMake artifacts, IDE
  cruft (`.vscode/`, `.idea/`, `*.swp`), compiled outputs (`*.o`,
  `*.a`, `*.so`, `*.dylib`, `*.dll`, `*.exe`), Python `__pycache__`,
  `.DS_Store`, etc.
- Initial commit: existing design docs, skills, agent — exactly as
  they are today. Commit message: scope is "design groundwork."
- Push to `origin/main`.

### A2. Top-level CMake

- `CMakeLists.txt` at repo root:
  - C++20 minimum (`set(CMAKE_CXX_STANDARD 20)` + required + no
    extensions).
  - `-Wall -Wextra -Wpedantic -Werror` baseline.
  - Sanitizer presets: a `RELEASE`, `DEBUG`, `ASAN`, `TSAN`,
    `UBSAN` build type. Multiple sanitizers selectable per build.
  - Compiler detection: support Clang as primary, GCC as
    secondary; warn on neither.
  - `FetchContent` declarations for: `nlohmann/json`, `GoogleTest`
    (with `GoogleMock`), `spdlog`, `mujoco`. **Pin specific
    versions** (commit SHA preferred over tag for reproducibility).
  - `enable_testing()` and `add_subdirectory(tests)`.
- Each `src/` and `tests/` subdirectory gets its own
  `CMakeLists.txt` (added as part of A4 below).
- Verify a fresh clone builds with:
  ```
  cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
  cmake --build build
  ctest --test-dir build
  ```
  (Empty test suite passes; that's the goal.)

### A3. Style and lint configuration

- `.clang-format` — project rules:
  - 2-space indent, 100-column line limit, K&R-ish brace style.
  - `IncludeBlocks: Regroup`, project headers first, third-party,
    then standard library; alphabetized.
- `.clang-tidy` — enable:
  - `cppcoreguidelines-*` (with explicit excludes for noise).
  - `bugprone-*`.
  - `misc-*`.
  - `readability-*` (selective).
  - **Custom rule** banning `std::chrono::system_clock` and
    `std::chrono::steady_clock` in sim core paths. (If a custom
    clang-tidy check is too heavy for v0, use a `clang-tidy`
    `regex` check or a CI grep step as a temporary substitute, with
    a TODO to upgrade to a real check.)
- A short `clang-format`/`clang-tidy` runner script at
  `scripts/lint.sh`.

### A4. Directory tree

Create the directories below as empty (each with a placeholder
`CMakeLists.txt` if needed for the build to succeed). Don't
implement anything yet.

```
src/
  backend/                # Layer 2 — HAL shim + state cache + sync
    common/               # protocol structs, tier-agnostic logic
    tier1/                # native: shared-memory transport
    tier2/                # qemu-user: socket transport
  sim_core/               # main sim core process entry point
  bus/
    can/                  # Layer 3 — CAN bus arbitration model
  vendors/
    phoenix6/
      talon_fx/           # Layer 3 — Talon FX MVP
      cancoder/           # Layer 3 — CANcoder MVP
      pigeon2/            # Layer 3 — Pigeon 2 MVP (idle in demo)
  models/
    electrical/           # Layer 4 — battery, brownout, PDH
    motors/
      kraken_x60/         # Layer 4 — Kraken X60 model
    sensors/              # Layer 4 — sensor models
  world/
    dynamics/             # Layer 5 — engine-agnostic interface
    engine/
      mujoco/             # Layer 5 — MuJoCo binding
  description/            # JSON robot description loader
  time/                   # sim clock interface
  rng/                    # seed distribution
  log/                    # WPILOG state-stream emitter
  util/                   # units, det_math, error types

tests/                    # mirrors src/ exactly

tools/
  rio-bench/              # separate Java/Gradle WPILib robot project
                          # (NOT built by the top-level CMake)

descriptions/
  v0-arm.json             # placeholder; loader implementation in B

references/
  rio2-hal-costs/         # versioned fixture files (estimated for now)

cmake/
  toolchains/
    arm-rio.cmake         # ARM cross-compile for tier 2

.github/
  workflows/
    ci.yml                # added in A5
```

Each `src/<dir>/` and `tests/<dir>/` gets a minimal `CMakeLists.txt`
that declares its translation units (none yet) so the build remains
clean.

### A5. CI

- `.github/workflows/ci.yml`:
  - Matrix: `{compiler: [clang, gcc]} × {sanitizer: [none, asan, tsan, ubsan]}`.
    Drop combinations that don't make sense (e.g. TSan with
    GoogleTest's death tests off).
  - Steps: install toolchain, configure CMake, build, run ctest,
    run `clang-format` check, run `clang-tidy` on changed files.
  - All combinations must pass before merge.
- A separate workflow for the ARM cross-compile build (no test
  execution; just verify the cross-compile succeeds — full tier-2
  testing requires qemu-user, added later).

### A6. ARM cross-compile toolchain

- `cmake/toolchains/arm-rio.cmake` — pointing to the WPILib ARM
  cross-compiler. Use the same toolchain RIOEmulator uses; it's
  publicly distributed by WPILib for the targeted year.
- Verify the empty tier-2 backend stub cross-compiles cleanly.

### A7. Verification and commit boundary

Before declaring Phase A complete:

- `cmake -B build && cmake --build build && ctest` passes locally
  on host (clang and gcc).
- `cmake -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-rio.cmake && cmake --build build-arm` succeeds (just a build check).
- `clang-format --dry-run --Werror` clean across the tree.
- `clang-tidy` runs without errors on the (empty) source tree.
- CI workflow passes on a push.

Commit boundary: small, focused commits per A1–A6. Don't pile the
whole scaffold into one commit. The scaffold should be reviewable.

## Phase B — First feature: robot description loader

This is the first feature implemented under full TDD discipline.
**You must follow `tdd-workflow.md` step-by-step.** No skipping.

### B1. Understand what the loader does

Reads `descriptions/<robot-name>.json`, parses it, validates it,
returns an in-memory `RobotDescription` struct (shape and field
names defined as part of this work, in `src/description/schema.h`).
The struct will be consumed by:

- `src/world/engine/mujoco/` — to emit MJCF.
- `src/backend/` — for CAN ID assignments to the device bus.
- `src/vendors/phoenix6/*/` — for firmware version pinning.

For v0 the schema covers links, joints, motors, sensors. See
`docs/ARCHITECTURE.md` "Robot description format" for the v0 arm
JSON sketch.

### B2. Read first

- `.claude/skills/tdd-workflow.md`
- `.claude/skills/adding-a-feature.md`
- `.claude/skills/code-style.md`
- `.claude/skills/layer-1-robot-code.md` (loader is Layer-adjacent
  to robot code via the description it produces)
- `.claude/skills/layer-5-world-physics.md` (consumer of the
  geometry portion)

### B3. Draft the test plan

Per `tdd-workflow.md` step 3. For each test, write down:

- **Name**, **layer/contract**, **bug class caught**, **inputs**,
  **expected outputs**, **tolerance** (if numerical), **determinism
  notes**, **assumptions/stubs**.

Cover at minimum:

- Happy-path: minimal valid v0-arm description loads to the
  expected struct.
- Schema-version validation: missing or unsupported `schema_version`
  rejected with a clear error.
- Required-field validation: each required field missing → clear
  error naming the field.
- Type-mismatch validation: wrong types (e.g. string where number
  expected) rejected with location info.
- Reference integrity: motor referencing a nonexistent joint
  rejected; sensor referencing a nonexistent joint rejected.
- CAN ID conflict: two devices with the same `controller_can_id`
  rejected.
- Determinism: same input file → bit-identical output struct
  across runs (test runs the loader twice and compares).
- Error messages include file path and JSON pointer to the
  offending location.

### B4. Submit to test-reviewer

Invoke the `test-reviewer` agent (`.claude/agents/test-reviewer.md`)
with the test plan, layer skill pointers, and a link to the
`v0-arm.json` example in `docs/ARCHITECTURE.md`. Iterate until the
reviewer's verdict is `ready-to-implement`.

**If the reviewer rejects:** address findings in writing, resubmit.
Don't argue in code. If you genuinely disagree with a finding, the
override goes in the test file as a comment with reasoning.

### B5. Implement the (failing) tests

In `tests/description/`. Mirror the source layout. GoogleTest +
GoogleMock idioms.

### B6. Confirm tests fail for the expected reasons

Each test must fail before its production code exists, and fail for
the right reason. Tautological-pass tests are bugs.

### B7. Implement the loader

In `src/description/`:

- `schema.h` — `RobotDescription`, `Link`, `Joint`, `Motor`,
  `Sensor` POD structs. Use `std::optional` / `std::variant` where
  the JSON has optional or alternative fields.
- `loader.h` / `loader.cpp` — `load_from_file(path) -> std::expected<RobotDescription, LoadError>`.
- `validation.h` / `validation.cpp` — schema, reference, conflict
  checks separated from parsing for clarity.
- `error.h` — `LoadError` with file path, JSON pointer, message.

Minimum code to pass tests. No speculative features.

### B8. `descriptions/v0-arm.json`

Replace the Phase-A placeholder with the real v0 arm description
that the loader's happy-path test uses. Pin `controller_firmware_version`
fields with placeholder strings to be replaced when the real Phoenix 6
firmware version is selected at v0 ship.

### B9. Skill updates

Per `adding-a-feature.md`:

- Update `layer-5-world-physics.md` if the loader interface to
  Layer 5's MJCF generator needs documenting.
- The loader is substantial enough that it warrants its own skill.
  Create `.claude/skills/robot-description-loader.md` with: scope,
  public surface, design decisions, citations (none for v0 since
  this is structural, not physical), known limits.
- Update `CLAUDE.md`'s skill index to list the new skill.

### B10. Commit boundary

Phase B can be one or two commits — separate the schema definition
from the loader implementation if it makes the diff readable.
Tests, source, and the v0-arm.json file all in the same set of
commits.

## What NOT to do in the first session

- Don't start Layer 2 (HAL shim) work. The protocol-schema POD
  structs come next, but only after Phase B and a fresh design
  discussion in a follow-up session.
- Don't start Layer 3 (vendor models). MVP per device is real work
  and needs its own TDD cycle and bench-validation plan.
- Don't pre-implement MuJoCo bindings. Wait for Layer 5 work to
  start in a later session.
- Don't add features past Phase B "just because." First-feature
  done is a reasonable session boundary.

## Suggested session boundary

Stop the first session when **Phase A is committed and pushed AND
Phase B's test plan has received `ready-to-implement` from the
test-reviewer**. That gives a clean handoff with the next session
implementing the approved tests + loader.

If time runs out earlier, stop at the end of Phase A — that's also a
clean handoff.

## What's still open (post-v0)

These remain in `OPEN_QUESTIONS.md`:

- **OQ-6** — vendor model fidelity strategy. Resolve when the first
  vendor model lands (likely session 3 or 4).
- **OQ-8** — CAN bus topology fidelity. Resolve before the bus
  arbitration model lands (probably session 4 or 5).
- **OQ-9** — vision pipeline. Deferred until v1+.
- **OQ-10** — test field for validation. Resolve as bench access
  becomes concrete.

OQ-7 (visualizer) was decided 2026-04-29 — option C, with the 3D
viewer at `src/viz/` (Edit / Live / Replay modes) running parallel to
the sim core. OQ-11 (authoring GUI / CAD) was folded into OQ-7 as the
visualizer's Edit mode. See `docs/VISUALIZER_V0_PLAN.md`.

## If you get stuck

- Re-read the relevant layer skill — design intent is captured there.
- Check `docs/ARCHITECTURE.md` for the binding decision; don't
  re-litigate decided questions silently.
- Check the user before making any decision that contradicts a
  recorded design choice.
