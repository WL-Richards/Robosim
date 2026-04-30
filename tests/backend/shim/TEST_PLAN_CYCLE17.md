# HAL shim core — cycle 17 test plan (clock_state HAL_Bool helper lift + 4 new readers)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, two review rounds). Round-1 verdict was `not-ready` with one required
change: the helper's placement relative to `hal_c.cpp`'s
`extern "C"` boundary was unspecified, leaving the implementer to
make a structural decision the plan should have made. Round-2 picks
Option B (helper inside `robosim::backend::shim::{anonymous}`,
parallel to the existing `g_installed_shim_` storage) plus
**file-scope `using` declarations after the namespace block** for
`clock_state` and `clock_state_hal_bool_read`, so the five
`extern "C"` wrappers each remain genuine 1-line `return helper(...)`
bodies. All four open-question rulings (no combinator test; no
float helper this cycle; 4 tests are sufficient; bundling is
justified) are upheld.

**Implements:** the seventeenth TDD cycle of the HAL shim core. This
is the **first explicitly bundled cycle** in the C HAL ABI series:
it lifts a shared `clock_state_hal_bool_read` helper in
`hal_c.cpp`'s anonymous namespace, refactors the existing
`HAL_GetBrownedOut` (cycle 15) to call it, and adds **four new
HAL_Bool readers** that all share the same shape:

- `HAL_GetSystemActive` → reads `latest_clock_state_->system_active`
- `HAL_GetSystemTimeValid` → reads `latest_clock_state_->system_time_valid`
- `HAL_GetFPGAButton` → reads `latest_clock_state_->fpga_button_latched`
- `HAL_GetRSLState` → reads `latest_clock_state_->rsl_state`

After cycle 17, the **clock_state HAL_Bool reader subset is closed**
(5 of 5 fields wired: `browned_out`, `system_active`,
`system_time_valid`, `fpga_button_latched`, `rsl_state`). The
remaining clock_state field — `comms_disable_count` (uint32_t) —
is its own cycle since it is a different return type.

Cycles 1–16 are landed and 137-shim-test green; full project ctest
452/452 green under both clang Debug and GCC Debug + ASan + UBSan.

---

## Why this cycle exists (and why it is bundled)

After cycle 15's `HAL_GetBrownedOut`, four more HAL_Bool readers on
the same cache slot remained. Each one structurally is a copy of
`HAL_GetBrownedOut` with a different field. Per the cycle-9/10/11
precedent, "one HAL_* per cycle" was the established cadence — but
that precedent was set when each cycle introduced a meaningfully
different design decision (typed-outbound, active-prefix,
shutdown-terminal, …). The four remaining clock_state HAL_Bool
readers introduce **zero new design decisions individually**.

Cycle 16's reviewer flagged the helper-lift opportunity directly:

> "[A]bout `HAL_GetVinVoltage`, `HAL_GetVinCurrent`,
> `HAL_GetBrownoutVoltage`: three functions sharing return type,
> cache slot, cast, and differing only in pointer-to-member
> field. A helper narrowed to just this subgroup needs **one
> parameter**, not four template parameters." (cycle-16 review,
> OQ-C16-LIFTING-A-HELPER ruling)

The same reasoning applies to the five clock_state HAL_Bool readers:
`HAL_Bool` return, `latest_clock_state()` cache slot, no cast,
differ only in `hal_bool clock_state::*` field. A
pointer-to-member helper is the natural shape, with one parameter
per call site.

Bundling four new surfaces into one cycle is justified here
because:

1. **Zero new design decisions per surface** — each new wrapper is
   a 1-line `return helper(status, &clock_state::<field>);`.
   Per-surface tests would only re-verify the helper's contracts
   that cycle 15 already pinned, plus one wrong-field check.
2. **The helper-lift trigger is real now.** The cycle-16 reviewer's
   pointer-to-member analysis applies. A 6th-or-7th-similar-function
   trigger has arrived, and the 5-function clock_state HAL_Bool
   subset is the natural target.
3. **Per-wrapper coverage is preserved.** Each new HAL_* still gets
   its own wrong-field positive test (D-C17-PER-WRAPPER-WRONG-
   FIELD-COVERAGE), so the "implementer wired the wrong field"
   bug class is per-function-checked.

This is **not** a precedent for bundling outbound-buffering surfaces
or struct-out-param surfaces or any cycle that introduces a new
mechanic. Bundling is reserved for cycles where the new surfaces
introduce literally zero new design decisions and a helper lift is
already structurally motivated.

---

## Contract under test

### New public surface

`src/backend/shim/hal_c.h` — additions:

```cpp
extern "C" {

// ... existing ...

// Mirrors WPILib HAL_GetSystemActive from hal/include/hal/HALBase.h.
// Returns 1 if the FPGA's safety system has been enabled (not
// browned out, not in fault). Reads `latest_clock_state_->
// system_active` directly. Status semantics same as
// HAL_GetBrownedOut.
HAL_Bool HAL_GetSystemActive(std::int32_t* status);

// Mirrors WPILib HAL_GetSystemTimeValid from hal/include/hal/HALBase.h.
// Returns 1 if the FPGA timestamp is valid (RTC initialized and
// running). Reads `latest_clock_state_->system_time_valid`.
HAL_Bool HAL_GetSystemTimeValid(std::int32_t* status);

// Mirrors WPILib HAL_GetFPGAButton from hal/include/hal/HALBase.h.
// Returns 1 if the FPGA's user button is currently latched
// (pressed). Reads `latest_clock_state_->fpga_button_latched`.
HAL_Bool HAL_GetFPGAButton(std::int32_t* status);

// Mirrors WPILib HAL_GetRSLState from hal/include/hal/HALBase.h.
// Returns 1 if the Robot Signal Light is currently on (driven by
// the FPGA's enable/safety logic). Reads
// `latest_clock_state_->rsl_state`.
HAL_Bool HAL_GetRSLState(std::int32_t* status);

}  // extern "C"
```

### Internal additions

The helper lives **inside `robosim::backend::shim`'s anonymous
namespace**, parallel to the cycle-12 `g_installed_shim_` storage.
This placement means name lookup inside the helper resolves
`shim_core`, `kHalHandleError`, `kHalSuccess` (all in
`robosim::backend::shim`), `clock_state` and `hal_bool` (both in
`robosim::backend`, the parent namespace), and `HAL_Bool` (file-
scope from the `extern "C"` typedef in `hal_c.h`) all by unqualified
name — no `using` clutter inside the helper body.

```cpp
namespace robosim::backend::shim {
namespace {

// (existing) shim_core* g_installed_shim_ = nullptr;

// Cycle-17 helper: shared dispatch path for HAL_Bool readers on
// latest_clock_state_. Pointer-to-member parameter selects which
// hal_bool field is returned. Encapsulates the cycle-12 null-shim,
// empty-cache, status-write, and latest-wins contracts (all five
// callers — HAL_GetBrownedOut + the four cycle-17 surfaces —
// inherit these for free without per-function code repetition).
HAL_Bool clock_state_hal_bool_read(std::int32_t* status,
                                   hal_bool clock_state::* field) {
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_clock_state();
  if (!cached.has_value()) {
    return 0;
  }
  return cached.value().*field;
}

}  // namespace
}  // namespace robosim::backend::shim
```

To keep the five `extern "C"` wrappers as genuine 1-line bodies
without per-wrapper `using` decl noise, two file-scope `using`
declarations bridge the helper and the `clock_state` type into the
extern "C" block's name lookup scope:

```cpp
// Between the closing `}  // namespace robosim::backend::shim` and
// the opening `extern "C" {` block:
using robosim::backend::clock_state;
using robosim::backend::shim::clock_state_hal_bool_read;
```

Both `using`s are legal C++ even though `clock_state_hal_bool_read`
is in an anonymous namespace — anonymous-namespace symbols are
accessible by qualified name within the same TU, and `using` simply
brings the (already-uniquely-named-by-the-compiler) symbol into the
current scope while preserving its internal linkage.

### Refactored existing code

`HAL_GetBrownedOut` (cycle 15): body changes from inline 5-line
read to a 1-line helper call. The `using` declarations that the
old body carried (`using robosim::backend::shim::shim_core; ...`)
are deleted — they are no longer needed because the helper does
the work:

```cpp
HAL_Bool HAL_GetBrownedOut(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::browned_out);
}
```

`clock_state` and `clock_state_hal_bool_read` resolve via the
file-scope `using` declarations introduced just before the
`extern "C"` block.

The refactor preserves observable behavior; cycle-15's existing 4
tests (C15-1 / C15-2 / C15-3 / C15-4) plus the renamed
`Types.HalBoolIsInt32` re-verify the contract under the helper.

### New wrappers

```cpp
HAL_Bool HAL_GetSystemActive(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::system_active);
}
HAL_Bool HAL_GetSystemTimeValid(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::system_time_valid);
}
HAL_Bool HAL_GetFPGAButton(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::fpga_button_latched);
}
HAL_Bool HAL_GetRSLState(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::rsl_state);
}
```

Each wrapper is **structurally a single line**. The bug class each
wrapper can independently introduce is exactly: "passed the wrong
pointer-to-member to the helper". That bug class is what the
cycle-17 tests target.

### Out of scope (deferred)

- `HAL_GetCommsDisableCount` — different return type (uint32_t),
  needs its own helper or its own inline body. Future cycle.
- ds_state readers (HAL_GetControlWord, HAL_GetAlliance, …) — first
  struct-out-param surface; new mechanic.
- power_state float helper lift. Same OQ-C16-LIFTING-A-HELPER
  question for `HAL_GetVinVoltage` / `HAL_GetVinCurrent` /
  `HAL_GetBrownoutVoltage`. The cycle-16 reviewer flagged this as
  a future opportunity; cycle 17 only lifts the HAL_Bool clock_state
  helper because the user explicitly bundled the clock_state
  HAL_Bool surface set. The float helper waits until that subset
  is touched again or a 4th float reader lands.
- Outbound-buffering surfaces (HAL_SendError, HAL_CAN_SendMessage,
  HAL_Notifier*) — fundamentally new mechanics.
- HAL_Initialize / HAL_Shutdown — lifecycle.
- Threading; LD_PRELOAD; NULL-status guard.

---

## Decisions pinned

### New: D-C17-CLOCK-STATE-HAL-BOOL-HELPER

A `clock_state_hal_bool_read(int32_t*, hal_bool clock_state::*)`
helper inside `hal_c.cpp`'s `robosim::backend::shim::{anonymous}`
block (parallel to the cycle-12 `g_installed_shim_` storage)
encapsulates the null-shim / empty-cache / status-write / latest-
wins behavior for all HAL_Bool readers on `latest_clock_state_`.
All five callers (refactored `HAL_GetBrownedOut` + four cycle-17
surfaces) consume the helper via two file-scope `using` declarations
(`using robosim::backend::clock_state;` and `using
robosim::backend::shim::clock_state_hal_bool_read;`) that sit
between the closing `}  // namespace robosim::backend::shim` and
the opening `extern "C" {` block. Cycle-15's existing test suite
exercises the helper through `HAL_GetBrownedOut`'s code path; the
helper is not directly tested as a separate symbol because it has
internal linkage and no other valid call site than the five
wrappers.

**Placement rationale (round-1 reviewer required clarification):**
The helper could equivalently live in a file-scope anonymous
namespace before `robosim::backend::shim` opens (Option A in the
round-1 review), but Option B (parallel to `g_installed_shim_`
inside `robosim::backend::shim::{anonymous}`) is preferred because
(a) the helper's body benefits from name lookup inside
`robosim::backend::shim` — `shim_core`, `kHalHandleError`,
`kHalSuccess` are all visible by unqualified name, and `clock_state`
/ `hal_bool` resolve via `robosim::backend` (parent namespace
lookup); (b) it sits next to its own state-equivalent
(`g_installed_shim_`) which the existing code structure has
already normalized; (c) the file-scope `using` declarations that
expose it to `extern "C"` are two lines for the entire cycle, not
per-wrapper. Each `extern "C"` wrapper is a genuine 1-line
`return clock_state_hal_bool_read(status, &clock_state::field);`
without per-wrapper `using` clutter.

**Lift rationale:** The cycle-16 reviewer's pointer-to-member
observation ("one parameter, not four") applies directly. Five
callers of an identically-shaped function is the trigger threshold
the cycle-14 reviewer set ("wait for the 4th or 5th"). Lifting now
prevents the same five-line block from being copy-pasted four more
times for the remaining clock_state HAL_Bool fields. The helper is
**scoped narrowly** — it is not a general HAL_Get template, just
the clock_state-HAL_Bool subset; no template parameters needed.

### New: D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE

Each of the four new HAL_* wrappers gets a single positive test
that pins it passes the correct `hal_bool clock_state::*` member
pointer to the helper. Tests use the cycle-15 fixture pattern:
populate `clock_state` with only the target field set to 1 and all
four sibling HAL_Bool fields set to 0, then assert the wrapper
returns 1.

A wrapper that passes the wrong member pointer reads a sibling
field that is 0 by fixture, fails the `b == 1` assertion. A wrapper
that bypasses the helper entirely (e.g. returns hardcoded 0) also
fails the `b == 1` assertion. A wrapper that calls the helper but
swaps the status pointer is a non-existent bug class given the
1-line wrapper structure (`return helper(status, ...)`).

The helper's null-shim, empty-cache, status-write, and latest-wins
contracts are **NOT** re-tested per-wrapper. Cycle 15's existing
tests cover those branches via `HAL_GetBrownedOut`, which after the
refactor goes through the same helper. A regression in the helper
fails C15-1 / C15-2 / C15-4 (and the new wrappers' positive tests
indirectly, since they all consume the helper). This is the same
session-vs-per-function trim convention cycles 13–16 applied.

### New: D-C17-BUNDLED-SCOPE

Bundling multiple HAL_* surfaces into a single cycle is justified
when **all** of the following hold:

1. The surfaces introduce zero new design decisions individually.
2. They share an exact code shape that motivates a helper lift.
3. Per-wrapper coverage of the surfaces is preserved through
   focused per-function tests.
4. The bundling is the natural granularity for the helper's scope
   (e.g. "all HAL_Bool readers on clock_state" — narrower than
   "all HAL_Get*" but broader than one).

This is **not** a precedent for bundling outbound-buffering, struct-
out-param, or any other "new mechanic" cycles. Cycle 17 is
explicitly the first time bundling fits all four criteria; future
cycles must justify bundling against this rubric or decline it.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR (now centralized in the helper;
  per-function test coverage delegated to cycle 15's existing
  HAL_GetBrownedOut tests via the refactor).
- D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO per-function variant for
  `clock_state` (centralized in helper; same delegation).
- D-C12-STATUS-WRITE-UNCONDITIONAL (centralized in helper).
- D-C12-LATEST-WINS-READ (centralized in helper).
- D-C15-HAL-BOOL-SIGNED-PARITY (`hal_bool == int32_t == HAL_Bool`).
- D-C15-HAL-BOOL-TYPEDEF-IN-HAL-C-H (`HAL_Bool` exported from
  `hal_c.h`).

---

## Test fixtures

No new helpers. Cycle 17 reuses `valid_clock_state`,
`shim_global_install_guard`, and the existing inline field-
assignment pattern.

---

## Determinism notes

`HAL_Bool` reads are scalar reads with the same shape as
`HAL_GetBrownedOut`. Cycle 8's `latest_clock_state_` byte-identity
replay covers the underlying cache. The helper introduces no new
state. No new determinism test.

---

## Proposed tests (revision 1)

### C17-1. `HalGetSystemActive.WithCachedClockStateSystemActiveTrueReturnsOneAndSetsSuccessStatus`

- **Layer / contract:** D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE for
  `system_active`. Pins that the wrapper passes
  `&clock_state::system_active` to the helper.
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - `auto state = valid_clock_state(/*sim_time_us=*/50'000);` then
    explicitly **only `system_active` = 1**, all four siblings 0:
    ```cpp
    state.system_active       = 1;  // distinguished
    state.browned_out         = 0;
    state.system_time_valid   = 0;
    state.fpga_button_latched = 0;
    state.rsl_state           = 0;
    ```
  - Drive `clock_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `HAL_Bool b = HAL_GetSystemActive(&status);`
- **Expected:** `b == 1`; `status == kHalSuccess`.
- **Bug class:** wrapper passes wrong member pointer (e.g.
  `&clock_state::browned_out` → reads 0, fails); wrapper returns
  hardcoded 0 (fails); wrapper bypasses the helper entirely.

### C17-2. `HalGetSystemTimeValid.WithCachedClockStateSystemTimeValidTrueReturnsOneAndSetsSuccessStatus`

- **Layer / contract:** D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE for
  `system_time_valid`.
- **Setup:** identical to C17-1 except **only
  `system_time_valid = 1`** with all four siblings 0:
  ```cpp
  state.system_active       = 0;
  state.browned_out         = 0;
  state.system_time_valid   = 1;  // distinguished
  state.fpga_button_latched = 0;
  state.rsl_state           = 0;
  ```
- **Action:** `HAL_Bool b = HAL_GetSystemTimeValid(&status);`
- **Expected:** `b == 1`; `status == kHalSuccess`.
- **Bug class:** wrapper passes wrong member pointer; wrapper
  bypasses helper.

### C17-3. `HalGetFPGAButton.WithCachedClockStateFpgaButtonLatchedTrueReturnsOneAndSetsSuccessStatus`

- **Layer / contract:** D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE for
  `fpga_button_latched`.
- **Setup:** identical to C17-1 except **only
  `fpga_button_latched = 1`** with all four siblings 0:
  ```cpp
  state.system_active       = 0;
  state.browned_out         = 0;
  state.system_time_valid   = 0;
  state.fpga_button_latched = 1;  // distinguished
  state.rsl_state           = 0;
  ```
- **Action:** `HAL_Bool b = HAL_GetFPGAButton(&status);`
- **Expected:** `b == 1`; `status == kHalSuccess`.
- **Bug class:** wrapper passes wrong member pointer; wrapper
  bypasses helper. **Note:** WPILib's HAL function name is
  `HAL_GetFPGAButton`; our schema field is `fpga_button_latched`.
  A wrapper that passes `&clock_state::fpga_button_latched`
  (correct) reads 1; a wrapper that mistakenly tries to pass
  `&clock_state::fpga_button` would not compile (no such field).
  This catches the realistic bug — copy-paste from another wrapper
  with the wrong field.

### C17-4. `HalGetRSLState.WithCachedClockStateRslStateTrueReturnsOneAndSetsSuccessStatus`

- **Layer / contract:** D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE for
  `rsl_state`.
- **Setup:** identical to C17-1 except **only `rsl_state = 1`**:
  ```cpp
  state.system_active       = 0;
  state.browned_out         = 0;
  state.system_time_valid   = 0;
  state.fpga_button_latched = 0;
  state.rsl_state           = 1;  // distinguished
  ```
- **Action:** `HAL_Bool b = HAL_GetRSLState(&status);`
- **Expected:** `b == 1`; `status == kHalSuccess`.
- **Bug class:** wrapper passes wrong member pointer.

---

## Tests deliberately not added (and why)

- **Per-wrapper null-shim tests for the 4 new HAL_*.** The helper is
  the single point of dispatch for all 5 callers. Cycle-15's C15-1
  (`HalGetBrownedOut.WithNoShimInstalled...`) covers the helper's
  null-shim path through one of the consumers; after the refactor,
  that test exercises the same code path the new wrappers also
  invoke. A regression in the helper's null path fails C15-1 even
  though the 4 new wrappers don't have their own. Per
  D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE, only the per-function
  bug class (wrong field) gets per-function coverage.
- **Per-wrapper empty-cache tests.** Same reasoning. C15-2 covers
  the helper's empty-cache path through HAL_GetBrownedOut; the
  4 new wrappers inherit.
- **Per-wrapper latest-wins tests.** C15-4 covers latest-wins
  through HAL_GetBrownedOut. Adding 4 more wouldn't catch a new
  bug class.
- **A combinator test that calls all 5 HAL_Bool readers against a
  single fixture with all 5 fields distinct.** Considered.
  Rationale to skip: each wrong-field bug is already caught by
  C17-1 / C17-2 / C17-3 / C17-4 individually with their isolated
  fixtures. A combinator test would catch only contrived bugs like
  "the helper caches the last `field` parameter across calls",
  which is not a realistic implementation. The cost is one more
  test that mostly duplicates coverage. If the reviewer wants it,
  it's a 30-line add.
- **A test that the refactored HAL_GetBrownedOut still passes its
  cycle-15 tests.** Implicit: ctest re-runs the cycle-15 tests
  against the refactored implementation; the refactor is verified
  by the green cycle-15 suite plus the new cycle-17 tests
  collectively passing.
- **NULL status guard.** UB matching WPILib.
- **Determinism replay.** Cycle 8 covers `latest_clock_state_`.

---

## Cross-test invariants

Same as cycles 12–16. Sentinel pattern (999), guard usage rules
(C17-1 through C17-4 all use the RAII guard), `make_connected_shim`,
all-other-siblings-zero fixture discipline (one field distinguished
to 1, four to 0).

---

## Implementation companion recommendations

- **Order of operations:**
  1. Add `clock_state_hal_bool_read` helper to `hal_c.cpp` **inside
     `robosim::backend::shim`'s existing anonymous namespace**
     (parallel to `g_installed_shim_`).
  2. Add the two file-scope `using` declarations
     (`using robosim::backend::clock_state;` and
     `using robosim::backend::shim::clock_state_hal_bool_read;`)
     between the closing `}  // namespace robosim::backend::shim`
     and the opening `extern "C" {`.
  3. Refactor `HAL_GetBrownedOut` to call the helper. Delete the
     three `using robosim::backend::shim::*` declarations from its
     body — they are no longer needed.
  4. Run full ctest under both build matrices; confirm the
     cycle-15 suite (4 tests in the `HalGetBrownedOut` suite +
     `Types.HalBoolIsInt32`) still passes via the refactored
     helper-based body.
  5. Add the 4 new declarations to `hal_c.h`.
  6. Add 4 stub definitions to `hal_c.cpp` (returning 0 without
     writing status, broken-by-design TDD pattern).
  7. Add the 4 new tests to `shim_core_test.cpp`; confirm they
     fail.
  8. Replace stubs with real 1-line `return
     clock_state_hal_bool_read(status, &clock_state::field);`
     bodies.
  9. Confirm 456/456 green under both matrices.
- The four new wrappers are genuine 1-line bodies. Place them
  adjacent in `hal_c.cpp` (after `HAL_GetBrownedOut`) so the visual
  repetition signals the shared shape.
- Doc comments on the four new declarations in `hal_c.h` should
  briefly mirror WPILib's documentation for each function.

---

## Open questions

**OQ-C17-COMBINATOR-TEST.** Should cycle 17 add a 5-reader
combinator test (call all 5 HAL_Bool readers against one fixture
with all 5 fields distinct, assert each returns its expected value)?
Plan picks NO — wrong-field bugs are already caught by the 4
isolated tests. Reviewer welcome to argue for inclusion if they
think the "helper caches last field" or "wrapper aliasing" bug
class deserves explicit coverage.

**OQ-C17-BUNDLE-FLOAT-HELPER-TOO.** Should cycle 17 also lift the
power_state float helper (the natural OQ-C16-LIFTING-A-HELPER
target)? Plan picks NO — the user's bundling directive specifically
covered the clock_state HAL_Bool subset. Lifting the float helper
in the same cycle would expand scope to a different cache slot and
different cast pattern. The float helper waits for either a 4th
power_state float reader (none planned in v0) or a maintenance
event that surfaces the duplication cost. Reviewer welcome to
push back.

**OQ-C17-TEST-COUNT.** Plan proposes 4 tests for 4 new surfaces.
That is the minimum-viable count under D-C17-PER-WRAPPER-WRONG-
FIELD-COVERAGE. Is that enough, or should each new wrapper also
get a null-shim and empty-cache test for symmetry with cycle 15?
Plan argues "no — those are session-level via the helper". Reviewer
welcome to argue for symmetry.
