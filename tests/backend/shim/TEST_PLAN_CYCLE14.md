# HAL shim core — cycle 14 test plan (HAL_GetVinCurrent)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, single review round). Round-1 verdict
`ready-to-implement` with no blockers and one required plan-text
correction (a muddled parenthetical in C14-3's bug-class narrative
about wrong-field copy-paste — addressed in this revision).
Reviewer ruled both open questions: OQ-C14-VALUE-CHOICE keep the
different-from-C13 fixture (catches the "copy-pasted whole function
body including field name" bug); OQ-C14-LIFTING-A-HELPER do NOT
lift a shared helper at three functions (defer until 4th/5th lands
and the pattern is stable; the three current readers differ in
return type, cache slot, AND field, so a shared helper would need
three template parameters and obscure the one line that varies).

**Implements:** the fourteenth TDD cycle of the HAL shim core, the
**third C HAL ABI surface**: `extern "C" double
HAL_GetVinCurrent(int32_t* status)` reading from
`latest_power_state_->vin_a`. Cycles 1–13 are landed and 125-shim-
test green; full project ctest 440/440 green under both clang Debug
and GCC Debug + ASan + UBSan. Cycle 14 introduces **no new design
decisions** — every contract inherits from cycles 12 and 13.

---

## Why this cycle exists

`HAL_GetVinCurrent` is the sibling of cycle 13's `HAL_GetVinVoltage`:
same `power_state` cache, different `float` field (`vin_a` instead of
`vin_v`), same widening cast at the C ABI seam. The cycle is a pure
mirror that validates **D-C13-FLOAT-TO-DOUBLE-CAST generalizes** to a
second consumer without re-litigation — the same bug class that
cycle 13 caught for `vin_v` is caught here for `vin_a`. The wrong-
field bug (`vin_v` instead of `vin_a`) is per-function, so per-function
coverage is required.

This also exercises one important property: the cycle-13 reviewer
explicitly endorsed "one HAL_* per cycle" over bundling
(OQ-C13-PEER-BUNDLING resolution). Cycle 14 honors that — it is its
own focused cycle with its own targeted fixture rather than being
folded into cycle 13.

After cycle 14, every per-`power_state`-float reader will have its
own focused cycle (cycle 13 = `vin_v`; cycle 14 = `vin_a`); the third
field `brownout_voltage_v` is a separate later cycle, since it has
the same float→double shape but a different semantic (the brownout
*threshold*, not the live current).

---

## Contract under test

### New public surface

`src/backend/shim/hal_c.h` — additions only:

```cpp
extern "C" {

// ... existing HAL_GetFPGATime, HAL_GetVinVoltage ...

// Mirrors WPILib HAL_GetVinCurrent from hal/include/hal/HALBase.h.
// Returns the FPGA's reported battery current draw in amps — for the
// shim, the most-recently-cached `power_state::vin_a` widened from
// float to double via static_cast (D-C13-FLOAT-TO-DOUBLE-CAST
// inherited).
//
// Status semantics mirror HAL_GetVinVoltage exactly.
double HAL_GetVinCurrent(std::int32_t* status);

}  // extern "C"
```

### Internal additions

- `hal_c.cpp` gains `HAL_GetVinCurrent` definition. Exact same shape
  as `HAL_GetVinVoltage` — null-shim → handle error, empty cache →
  success-zero, populated cache → cast `vin_a` to double. The two
  function bodies after this cycle are 5 lines each and look
  near-identical (per code-style.md: three similar lines is better
  than a premature abstraction).
- No new storage. No changes to `shim_core.h` / `shim_core.cpp`.

### CMake

No change.

### Out of scope (deferred to later cycles)

- `HAL_GetBrownedOut` (returns HAL_Bool, not double — different
  return-type convention; reads `clock_state::browned_out`).
- `HAL_GetBrownoutVoltage` (reads `power_state::brownout_voltage_v`;
  same float→double shape but different semantic — the threshold,
  not the live measurement).
- Every other HAL_* surface.
- Threading; LD_PRELOAD wiring; NULL-status guard. (All same as
  cycle 12/13.)

---

## Decisions pinned

### Inherited unchanged (no new decisions in cycle 14)

- **D-C12-GLOBAL-ACCESSOR** (session-level; pinned by C12-1/2/3).
- **D-C12-NULL-SHIM-IS-HANDLE-ERROR** (per-function; pinned by C14-1).
- **D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO** per-function variant for
  `power_state` (pinned by C14-2).
- **D-C12-STATUS-WRITE-UNCONDITIONAL** (per-function; pinned by
  C14-1's sentinel-overwrite path).
- **D-C12-LATEST-WINS-READ** (per-function; pinned by C14-4).
- **D-C13-FLOAT-TO-DOUBLE-CAST** (per-function; pinned by C14-3).

This cycle introduces no new D-C14-* decisions. If the reviewer
finds one, that's a sign cycle 14's scope is wrong.

---

## Test fixtures

No new helpers. Cycle 14 reuses `valid_power_state(vin_v, vin_a,
brownout_voltage_v)` and `shim_global_install_guard` from earlier
cycles.

---

## Determinism notes

Same as cycle 13: cycle-8's `latest_power_state_` byte-identity replay
already pins the cache contents, and the C ABI wrapper adds no new
nondeterminism class. No new determinism test in cycle 14.

---

## Proposed tests (revision 1)

### C14-1. `HalGetVinCurrent.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function);
  D-C12-STATUS-WRITE-UNCONDITIONAL (per-function).
- **Setup:**
  - `shim_core::install_global(nullptr);` precondition.
  - `ASSERT_EQ(shim_core::current(), nullptr);`
  - `int32_t status = 999;`
- **Action:** `double a = HAL_GetVinCurrent(&status);`
- **Expected:** `a == 0.0`; `status == kHalHandleError`.
- **Bug class:** null-pointer dereference; status not written on null
  path; function dispatches to a non-`current()` lookup that's
  inconsistent with the cycle-12 plumbing.

### C14-2. `HalGetVinCurrent.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO
  per-function variant for `power_state`.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - `ASSERT_FALSE(shim.latest_power_state().has_value());`
  - `shim_global_install_guard guard{shim};`
  - `int32_t status = 999;`
- **Action:** `double a = HAL_GetVinCurrent(&status);`
- **Expected:**
  - `a == 0.0`; `status == kHalSuccess`.
  - `shim.latest_power_state().has_value() == false` after the call
    (pure read).
- **Bug class:** UB on disengaged optional; wrong error code;
  function reads from the wrong cache slot when empty (closed by C14-3).

### C14-3. `HalGetVinCurrent.WithCachedPowerStateReturnsItsVinCurrentAndSetsSuccessStatus`

- **Layer / contract:** D-C13-FLOAT-TO-DOUBLE-CAST per-function
  variant for `vin_a`; end-to-end protocol-session →
  `latest_power_state_` → C ABI integration, **for the second
  power_state field** to validate D-C13 generalizes.
- **Setup:**
  - **Bit-distinct value choice — DIFFERENT FROM C13-3 SO WRONG-FIELD
    BUGS BETWEEN THE TWO HAL_Get* FUNCTIONS ARE CAUGHT:**
    `vin_v = 7.5f`, `vin_a = 42.25f`, `brownout_voltage_v = 6.5f`.
    The three values are pairwise distinct, all exactly representable
    in IEEE-754 single AND double precision (15/2, 169/4, 13/2).
    **Notably**, cycle 13's C13-3 used `vin_v = 12.5f, vin_a =
    99.25f, brownout_voltage_v = 6.75f`. A "shim copy-pasted
    `HAL_GetVinVoltage`'s body into `HAL_GetVinCurrent` and forgot
    to change `vin_v` to `vin_a`" bug, run against C14-3's own
    fixture, returns `7.5` (C14-3's `vin_v`) — not equal to the
    expected `42.25`, so the test catches it. If the implementer
    *also* copy-pasted the C13-3 fixture verbatim, the same
    wrong-field read would return `12.5` (C13-3's `vin_v`), also
    not equal to `42.25`. Either way, the wrong-field bug fails
    the test loudly.
  - Drive a `power_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `double a = HAL_GetVinCurrent(&status);`
- **Expected:** `a == 42.25` (exact equality; loss-free widening);
  `status == kHalSuccess`.
- **Bug class:** function reads `vin_v` (returns 7.5; caught via
  `42.25 != 7.5`); function reads `brownout_voltage_v` (returns
  6.5; caught via `42.25 != 6.5`); function reads from a different
  cache slot like `latest_clock_state_->sim_time_us` reinterpreted
  as a float (would be a wildly wrong value); function omits the
  float→double cast (compile error — narrowing back wouldn't
  compile, so this bug class is moot); **copy-paste-from-cycle-13
  with no field update** (the implementer copied
  `HAL_GetVinVoltage`'s body and forgot to change `vin_v` to
  `vin_a` — same observable as "wrong field" above; either C14-3's
  own `vin_v=7.5f` or C13-3's `vin_v=12.5f` produces a value
  unequal to `42.25` so the test catches it regardless of which
  fixture the implementer accidentally used).

### C14-4. `HalGetVinCurrent.LatestWinsAcrossTwoUpdatesReturnsTheMostRecentVinCurrent`

- **Layer / contract:** D-C12-LATEST-WINS-READ per-function variant
  for `latest_power_state_->vin_a`. Interleaved poll/read structure
  catches "caches-on-first-call" bugs per the cycle-12 / cycle-13
  precedent.
- **Setup:**
  - Fresh region; shim made; install guard installed.
- **Action sequence:**
  1. Drive `power_state{vin_v=10.0f, vin_a=3.5f, brownout=6.0f}` via
     core→shim, poll, OK.
  2. `int32_t status1 = 999; double a1 = HAL_GetVinCurrent(&status1);`
  3. Drive `power_state{vin_v=12.0f, vin_a=8.25f, brownout=7.0f}` via
     core→shim, poll, OK.
  4. `int32_t status2 = 888; double a2 = HAL_GetVinCurrent(&status2);`
- **Expected:**
  - After step 2: `a1 == 3.5` (= 7/2; exact); `status1 == kHalSuccess`.
  - After step 4: `a2 == 8.25` (= 33/4; exact); `status2 == kHalSuccess`.
  - The two updates change `vin_v` and `brownout` as well, so a wrong-
    field bug also changes between calls; the test does not
    accidentally pass via a fixed-field coincidence.
- **Bug class:** caches-on-first-call (would show `a2 == 3.5`); global
  pointer captured at install-time (same observable failure); function
  reads from a TU-static shadow updated only at install time.

---

## Tests deliberately not added

- **Idempotency mirror.** Same trim as cycle 13 (OQ-C13-IDEMPOTENCY-
  TEST resolution applies unchanged); the per-function read path is
  structurally identical to HAL_GetVinVoltage and the bug class is
  pinned.
- **NULL status guard.** UB matching WPILib.
- **Cross-cutting cycle-12 mirrors.** Session-level; pinned once.
- **Determinism replay.** Cycle 8 already pins `latest_power_state_`
  byte-identity.

---

## Cross-test invariants

Same as cycle 13. Sentinel pattern (999/888), guard usage rules
(C14-1 no guard; C14-2/3/4 with guard), make_connected_shim from
cycle 12, fixtures with bit-distinct values. The C14-3 fixture is
*intentionally different* from C13-3 to catch the "implementer
copy-pasted cycle-13's body forgetting to update the field" bug.

---

## Implementation companion recommendations (non-test)

- Doc comment on `HAL_GetVinCurrent` in `hal_c.h` mirrors
  `HAL_GetVinVoltage`'s, with "battery current" rather than
  "battery voltage" and `vin_a` rather than `vin_v`.
- The implementation function body is a 5-line pure mirror of
  `HAL_GetVinVoltage`. After this cycle, the `hal_c.cpp` file has
  three near-identical read functions (HAL_GetFPGATime,
  HAL_GetVinVoltage, HAL_GetVinCurrent). **Per code-style.md, do
  NOT lift a shared helper yet** — three similar functions is
  the project's "below the abstraction threshold" baseline. The
  natural lift point would be when the *fourth or fifth* read
  function lands AND the patterns are demonstrably stable
  (mirrors the D-C9-NO-HELPER → D-C10-EXTRACT-ACTIVE-PREFIX
  progression for outbound active-prefix helpers).

---

## Open questions

**OQ-C14-VALUE-CHOICE.** The plan picks `vin_v=7.5f, vin_a=42.25f,
brownout=6.5f` for C14-3 — explicitly different from C13-3's values.
The intent is to catch a "copy-pasted cycle-13's test body without
updating the field name" bug. Reviewer push-back welcome on whether
this is over-paranoia or genuinely valuable coverage.

**OQ-C14-LIFTING-A-HELPER.** With three near-identical read
functions in `hal_c.cpp` after this cycle, is now the right time to
lift a shared helper? Plan picks NO — three is the baseline below
which premature abstraction is the bigger risk. Reviewer welcome to
argue that three is enough.
