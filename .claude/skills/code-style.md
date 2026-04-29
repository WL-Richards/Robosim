---
name: code-style
description: Use whenever writing or refactoring code in robosim. Defines what "clean, robust, modular" means here — function size, single responsibility, naming, units, comments, error-handling policy, and the coupling rules between layers.
---

# Code style

Code in robosim is held to a higher standard than typical because it
lives across many layers, languages, and processes, and because physical
models that are hard to read are hard to validate. These rules aren't
preferences; they protect the project from the kind of slow rot that
makes simulators stop being trusted.

## Modularity

- **One file, one concept.** A file holds a single primary type or a
  small cluster of tightly-related helpers. If you're scrolling past
  several unrelated classes/structs, split.
- **Single responsibility per function.** A function does one thing the
  reader can describe in one sentence. If the description has "and,"
  split.
- **Narrow interfaces.** Public API of a module should be the smallest
  surface that lets callers do their job. Anything not needed externally
  is private/internal. Adding a public function should be deliberate.
- **Layer boundaries are sacred.** The layer interfaces in
  `docs/ARCHITECTURE.md` are stable contracts. Don't reach across them.
  If a Layer 4 motor model is asking for the physics engine's contact
  list, the design is wrong, not the layer boundary.
- **Composition over inheritance.** Class hierarchies for "reusable
  behavior" age badly. Functions and small structs compose; deep type
  trees don't.

## Function size

- Soft cap **~30 lines** of body, including the function signature and
  closing brace. Beyond that, the reader is paging.
- A loop body that's longer than a few lines is a function. Pull it
  out.
- An exception: pure data tables (e.g. motor constants) are allowed to
  be long. They're not "logic" in the size-cap sense.
- Number of parameters: **prefer ≤ 4**. More than that and a struct/
  options-object is almost always clearer.

## Naming

- Names describe **what** the value or function *is* or *does*, not how
  it's implemented. `applied_voltage` not `last_set_v`. `update_pose`
  not `do_step_2`.
- SI base names. `torque_nm`, `current_a`, `voltage_v`, `position_rad`,
  `velocity_rad_s`. The unit suffix is mandatory at API boundaries —
  it's the cheapest defense against a unit-mismatch bug.
- Avoid abbreviations the reader has to decode (`pwr`, `mtr`, `ctl`).
  Spell it out: `power`, `motor`, `controller`. The exception is
  domain abbreviations universally understood in FRC (`PDP`, `PWM`,
  `CAN`, `IMU`).
- Test names read as sentences:
  `motor_stalls_when_load_exceeds_torque_limit`, not `test_motor_3`.

## Units

- **SI internally, everywhere.** Volts, amps, ohms, newtons, newton-
  meters, kilograms, meters, radians, seconds.
- Convert at the API boundary if WPILib or a vendor wants different
  units. Conversions live in one place per boundary, named (e.g.
  `talonfx_native_to_rad_per_s`).
- Constants get unit suffixes (see Naming).

## Comments

Default to writing **no** comments. Add one only when the *why* is
non-obvious — a hidden constraint, a workaround, behavior that would
surprise a reader, a citation for a physical constant.

```rust
// Kraken X60 stator stall current at 24 V — measured by CTRE,
// see references/ctre/kraken-x60-curves.csv (commit a4f2e1).
const KRAKEN_STALL_CURRENT_A: f64 = 366.0;
```

Don't:

- Explain *what* the code does — the names already do that.
- Reference the current task or a PR ("added for swerve story").
- Leave dead code commented out — delete it.
- Write multi-paragraph docstrings for trivial functions.

## Error handling

- **Validate at system boundaries** (config load, user input, network,
  IPC). Internally, trust internal callers.
- Don't add fallbacks or error paths for cases that *can't* happen
  given the type system or earlier invariants. If you write a defensive
  branch, you've made an assertion the caller is broken — make it an
  assertion, not silent recovery.
- For physical models: numerical edge cases (NaN, infinity, divide-by-
  zero) are bugs, not cases to handle. Find the upstream cause.
- For determinism: errors that are non-deterministic (clock-dependent
  retries, random backoff) are forbidden in the simulation core. Use
  deterministic protocols.

## Backwards compatibility / migration

The repo is in early phase. **Don't preserve compatibility you don't
need.** When a design changes, replace the old code; don't add a shim.
There are no users yet — the tax of compatibility shims is pure cost.

## Cross-language

When the same concept (e.g. a motor model) is reachable from multiple
languages, the canonical implementation lives in one language and
others bind to it. Don't reimplement. Bindings are generated where
possible (cbindgen, JNI generators, etc.).

## File and directory layout

Language is **C++** (decided 2026-04-28; full toolchain in
`docs/ARCHITECTURE.md` "Language and toolchain"). Conventions:

- One concept per file, named after the concept. Header + source
  split: `motor_model.h` + `motor_model.cpp`.
- `snake_case` for files, types, functions, variables — matches
  WPILib HAL C-ABI naming so there's no flip at the boundary.
- Tests live in a parallel `tests/` tree mirroring the source tree
  (`src/motor/kraken_x60.cpp` → `tests/motor/kraken_x60_test.cpp`).
  GoogleTest convention.
- Per-vendor subdirectories under `vendors/<name>/`.
- Per-backend subdirectories under `backends/<name>/`.
- Per-tier transport implementations under `backends/<name>/tier1/`,
  `backends/<name>/tier2/`.

## C++ style specifics

These layer on top of the language-agnostic rules above.

- **Standard:** C++20 minimum; C++23 where compilers support it.
- **Compiler flags:** `-Wall -Wextra -Wpedantic -Werror` baseline.
  Sanitizers (UBSan + ASan + TSan) green in CI before merge.
- **Header guards:** `#pragma once`. No include guards.
- **Includes:** project headers first, then third-party, then
  standard library. Each block alphabetized. Forward-declare where
  possible to keep compile times sane.
- **Smart pointers:** `std::unique_ptr` for ownership, raw pointers
  or references for non-owning. `std::shared_ptr` only when shared
  ownership is genuinely required (rare). No raw `new`/`delete`
  outside explicitly-marked arena allocators.
- **`auto`:** use when the type is obvious from the right-hand side
  or when the type name would add noise. Don't use to obscure
  important types.
- **`const`:** correct everywhere. `const` member functions, `const`
  references, `const` locals. Mutability is opt-in.
- **Error handling:** `std::expected<T, Error>` (or equivalent) in
  sim-core hot paths. Exceptions only at process boundaries
  (config load, IPC failure, fatal init). `noexcept` on functions
  that genuinely don't throw — the optimizer rewards it.
- **Headers:** prefer `*.h` (project convention) for consistency.
  Pimpl idiom only at the HAL shim's external ABI surface; not
  internally.
- **Templates:** use them when they pay (typed tests for motor
  models, generic containers). Don't template-ize for "future
  flexibility" with one current use site.
- **Concepts (C++20):** use them on template parameters where they
  document intent and improve error messages. Don't write
  one-off concepts for the sake of it.
- **`std::format` / `std::print`:** prefer over `iostream` for
  user-facing output. `iostream` is fine for debug.
- **Determinism:**
  - **Banned in sim core:** `std::chrono::system_clock`,
    `std::chrono::steady_clock`, `std::random_device`, default-
    seeded `std::mt19937` etc. clang-tidy rule enforces.
  - All time access goes through the sim-time interface.
  - All RNG seeded from the project root seed via the central
    seed-distribution facility.
  - FP-determinism-sensitive code lives in `det_math::` namespace
    with explicit pragmas (FMA disabled, etc.).

## Testing-specific style

- **Framework:** GoogleTest + GoogleMock.
- **Test name format:** `MotorModel_StallsWhenLoadExceedsTorqueLimit`
  reads as a sentence about the behavior. (Note: GoogleTest's
  `TEST(SuiteName, TestName)` macros — suite is the SUT, test name
  is the behavior.)
- **Parameterized tests:** use `INSTANTIATE_TEST_SUITE_P` for
  "test this motor model at N operating points." Each parameter
  case carries its citation as a comment / member field.
- **Fixtures:** prefer minimal fixtures; tests should be readable
  top-to-bottom without jumping to setup.
- **No `EXPECT_*` in sim-core production code.** Test-only.

## Things we don't use

- `using namespace std;` — never, even in implementation files.
- Raw arrays as member fields — use `std::array`.
- C-style casts — `static_cast` / `reinterpret_cast` (justified)
  / `const_cast` (very rare, with comment explaining why).
- Macros for control flow — `inline` functions or `constexpr` instead.
  Macros are only for things genuinely impossible without them
  (e.g., `__FILE__` / `__LINE__` capture).
- `std::endl` — `'\n'` is faster and has the same line-ending
  behavior in 99% of cases.

## Robustness

- **No silent failure.** Anything that should never happen but might
  (e.g. CAN frame from an unknown ID) is logged at warn or higher and
  surfaced through the sim's diagnostic stream.
- **Reproducibility over convenience.** Don't depend on host state
  (locale, timezone, env vars) inside the sim core. Configuration is
  explicit and recorded with each run.
- **Resource discipline.** Allocate up front where the access pattern
  permits it. The 1 kHz physics loop must not be allocating per tick
  in the steady state.

## Robustness anti-patterns

- `try { … } catch { /* ignore */ }`. If you don't know what to do,
  let it propagate. Silent catches break debugging.
- "Future-proof" generics with one current user. Add the abstraction
  when the second use case lands.
- Configuration flags that change physical behavior at runtime. The
  sim's behavior is determined by config-at-start, not toggles.
