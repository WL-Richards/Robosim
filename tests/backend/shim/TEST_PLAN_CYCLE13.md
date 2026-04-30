# HAL shim core — cycle 13 test plan (HAL_GetVinVoltage)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, single review round, no blockers, no required changes).
Reviewer explicitly resolved all three open questions:
OQ-C13-IDEMPOTENCY-TEST (trim is defensible — no new bug class
beyond what C13-3/C13-4 catch); OQ-C13-PEER-BUNDLING (do not bundle;
one HAL_* per cycle); OQ-C13-FIXTURE-VIN-CHOICE (12.5/99.25/6.75
exact-equality assertion is correct because float→double widening is
lossless for finite values, so a tolerance would be wrong).

**Implements:** the thirteenth TDD cycle of the HAL shim core, the
**second C HAL ABI surface**: `extern "C" double
HAL_GetVinVoltage(int32_t* status)` reading from
`latest_power_state_->vin_v`. Cycle 12 (the first C HAL surface
plus all the plumbing) is landed and 121-shim-test green; full
project ctest 436/436 green under both clang Debug and GCC Debug +
ASan + UBSan. Cycle 13 inherits every D-C12-* decision unchanged
and adds one new design decision (D-C13-FLOAT-TO-DOUBLE-CAST) for
the schema-vs-WPILib width mismatch on this surface.

---

## Why this cycle exists

`HAL_GetVinVoltage` is the most direct mirror of cycle 12's
`HAL_GetFPGATime`: a pure cache read into a different observer
(`latest_power_state_` from cycle 2). It validates that the
"every HAL_* read function looks the same" pattern actually
generalizes — if cycle 12's plumbing is right, cycle 13 should be
mechanical.

Two meaningful differences from cycle 12:

1. **Different cache slot.** The dispatch reads
   `latest_power_state_->vin_v`, not `latest_clock_state_->sim_time_us`.
   A "shim wired the wrong observer" bug class is per-function and
   needs per-function coverage.
2. **Different return type and width.** `HAL_GetVinVoltage` returns
   `double`; the schema's `power_state::vin_v` is `float`. A widening
   cast happens at the C ABI seam. This is a new design decision
   (D-C13-FLOAT-TO-DOUBLE-CAST) worth pinning at the cycle that
   first encounters it; subsequent float-returning HAL_* surfaces
   inherit it.

`HAL_GetVinCurrent` (returns `latest_power_state_->vin_a`) and
`HAL_GetBrownedOut` (returns `kHalSuccess` + bool from `clock_state`
or `power_state`) are the natural cycle-14/15 follow-ons but are
**not** in cycle 13's scope. One surface per cycle, mirroring the
cycles-9/10/11 outbound pattern.

---

## Contract under test

### New public surface

`src/backend/shim/hal_c.h` — additions only (preserving the cycle-12
declarations):

```cpp
extern "C" {

// ... existing HAL_GetFPGATime ...

// Mirrors WPILib HAL_GetVinVoltage from hal/include/hal/HALBase.h.
// Returns the FPGA's reported battery voltage in volts — for the
// shim, the most-recently-cached power_state::vin_v widened from
// float to double (D-C13-FLOAT-TO-DOUBLE-CAST).
//
// Status semantics mirror HAL_GetFPGATime exactly:
//   - shim_core::current() == nullptr: *status = kHalHandleError;
//     return 0.0.
//   - shim installed but latest_power_state() == nullopt: *status =
//     kHalSuccess; return 0.0. Matches the model that vin_v is 0
//     before the first power_state arrives, plus the WPILib
//     contract that HAL_GetVinVoltage always succeeds.
//   - shim installed and power_state cached: *status = kHalSuccess;
//     return static_cast<double>(latest_power_state()->vin_v).
//
// `status` must not be NULL — UB if it is.
double HAL_GetVinVoltage(std::int32_t* status);

}  // extern "C"
```

### Internal additions

- `hal_c.cpp` gains the `HAL_GetVinVoltage` definition. Same
  three-branch shape as `HAL_GetFPGATime` but reading
  `latest_power_state()` and returning a `double` widened from the
  schema's `float vin_v`.
- No new storage. The `g_installed_shim_` TU-static (cycle 12) is
  reused.
- No changes to `shim_core.h` / `shim_core.cpp`. The new function is
  purely in the C HAL TU.

### CMake

No change. `hal_c.cpp` is already in the `robosim_backend_shim`
target sources from cycle 12.

### Out of scope (deferred to later cycles)

- Every other HAL_* function. Each gets its own cycle.
- HAL_GetVinCurrent / HAL_GetBrownedOut peers — natural cycle-14/15
  follow-ons but not bundled here. The cycle-12 precedent
  established that one HAL_* per cycle is the right granularity.
- Threading (single-threaded v0; same as cycle 12).
- LD_PRELOAD wiring (downstream).
- NULL-status-pointer protection (UB matching WPILib; same as cycle
  12).
- Validation of `vin_v` value range. The schema validator does
  framing only; semantic ranges are caller-side.

---

## Decisions pinned

### Inherited unchanged from cycle 12

- **D-C12-GLOBAL-ACCESSOR** — same `shim_core::current()` lookup.
  No re-test (session-level, pinned by C12-1 / C12-2 / C12-3).
- **D-C12-NULL-SHIM-IS-HANDLE-ERROR** — per-function code path.
  Pinned per-function by **C13-1**.
- **D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO** — per-function variant
  ("empty cache slot returns 0 with success"). Pinned per-function
  by **C13-2**. Same rationale as cycle 12: the WPILib contract
  for HAL_GetVinVoltage is "always succeeds"; the model is
  "battery voltage is 0 before the first power_state arrives."
- **D-C12-STATUS-WRITE-UNCONDITIONAL** — per-function code path.
  Pinned per-function via the `int32_t status = 999` sentinel
  pattern in **C13-1** (status overwrite from 999 → -1 on the
  null-shim path proves unconditional write).
- **D-C12-LATEST-WINS-READ** — per-function code path. Pinned
  per-function by **C13-4** (interleaved poll/read structure
  catching "caches-on-first-call" bugs).

### New: D-C13-FLOAT-TO-DOUBLE-CAST

`HAL_GetVinVoltage` returns `double` while `power_state::vin_v` is
`float`. The C ABI seam does an explicit
`static_cast<double>(latest_power_state()->vin_v)`. This widening
cast is **loss-free** for every IEEE-754 single-precision value;
no precision concern. Pinned by **C13-3**.

**Rationale for the float-typed schema field:** the WPILib HAL is
the only consumer of these values, and the wire format was sized
for compactness (4 bytes per value, not 8). A double-typed schema
would double the `power_state` payload size with no precision benefit
to robot code, since real RIO firmware also reports battery voltage
through a float register and WPILib widens at its own boundary.
Matching that pattern keeps the wire small and the cast at the
correct seam.

**Why pin this now and not in a later cycle?** First time the
mismatch shows up; later HAL_* surfaces with float→double casts
(`HAL_GetVinCurrent`, `HAL_GetBrownoutVoltage`,
`HAL_GetUserVoltage*`, etc.) inherit this decision rather than
re-deciding it. Same logic as cycle 9 pinning D-C9-TYPED-OUTBOUND
once for all three outbound schemas.

---

## Test fixtures

No new helpers. Existing `valid_power_state(vin_v, vin_a,
brownout_v)` from `test_helpers.h` covers the fixture surface.
`shim_global_install_guard` from cycle 12 is reused unchanged.

---

## Determinism notes

`HAL_GetVinVoltage` is a pure read with the same shape as cycle
12's `HAL_GetFPGATime`. The cycle-8 determinism replay already pins
byte-identical content for `latest_power_state_` across two runs.
Cycle 13 adds **no new determinism test** — same reasoning as cycle
12: the C ABI wrapper is a thin scalar read with no
uninitialized-stack-bytes class and no cross-thread ordering
question in single-threaded v0.

The float→double cast is deterministic by IEEE-754 — every
`static_cast<double>(f)` produces the same bits for the same `f`,
on every conforming compiler. No new bug class.

---

## Proposed tests (revision 1)

### C13-1. `HalGetVinVoltage.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function);
  D-C12-STATUS-WRITE-UNCONDITIONAL (per-function).
- **Setup:**
  - `shim_core::install_global(nullptr);` explicit pre-clear.
  - `ASSERT_EQ(shim_core::current(), nullptr);` precondition.
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetVinVoltage(&status);`
- **Expected:**
  - `v == 0.0`
  - `status == kHalHandleError`
- **Bug class:** function dereferences a null shim pointer (UB,
  ASan-detectable); function returns garbage instead of 0.0;
  function only writes status on the success path (would leave 999
  in place); function calls a different observer when no shim is
  installed (e.g. `latest_clock_state` instead of `latest_power_state`,
  but neither is reachable when current() is null — the bug class
  is "bypassed null-check").

### C13-2. `HalGetVinVoltage.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO
  per-function variant for `power_state`.
- **Setup:**
  - Fresh region; `auto shim = make_connected_shim(region, core);`.
  - `ASSERT_FALSE(shim.latest_power_state().has_value());` — confirms
    `make_connected_shim` does not implicitly cache power_state.
  - `shim_global_install_guard guard{shim};`
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetVinVoltage(&status);`
- **Expected:**
  - `v == 0.0`
  - `status == kHalSuccess`
  - `shim.latest_power_state().has_value() == false` (post-call;
    pure read, no cache mutation).
- **Bug class:** function reads from a disengaged optional (UB);
  function returns `kHalHandleError` on empty cache (would surface
  sim-internal boot-race state to robot code); function reads from
  the wrong cache slot when empty (e.g. clock_state) and finds it
  also empty so the test would still pass — the bug class is
  caught by C13-3 below.

### C13-3. `HalGetVinVoltage.WithCachedPowerStateReturnsItsVinVoltageAndSetsSuccessStatus`

- **Layer / contract:** D-C13-FLOAT-TO-DOUBLE-CAST (the schema is
  float, the C ABI is double); end-to-end protocol-session →
  `latest_power_state_` → C ABI integration.
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - **Bit-distinct value choice:** `vin_v = 12.5f`, `vin_a = 99.25f`,
    `brownout_voltage_v = 6.75f`. The three values are pairwise
    distinct; reading `vin_a` instead of `vin_v` would return 99.25,
    not 12.5 — catching a "wrong field" bug. All three are
    representable exactly in IEEE-754 single precision (12.5 = 25/2;
    99.25 = 397/4; 6.75 = 27/4) and exactly in double precision —
    the float→double cast is bit-equivalent to the literal `12.5`,
    so the test asserts `v == 12.5` (a `double` literal) rather than
    via a tolerance.
  - Drive an inbound `power_state` via the existing transport API:
    `core.send(envelope_kind::tick_boundary, schema_id::power_state,
    bytes_of(state), sim_time)` then `shim.poll()`.
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetVinVoltage(&status);`
- **Expected:**
  - `v == 12.5` (exact equality; the value is exactly representable
    in both float and double, so `static_cast<double>(12.5f) == 12.5`
    holds bit-for-bit on every IEEE-754 platform).
  - `status == kHalSuccess`.
- **Bug class:** function reads `vin_a` or `brownout_voltage_v`
  instead of `vin_v` (catches via the distinct fixture values);
  function reads `latest_clock_state_->sim_time_us` and reinterprets
  bytes as float (would return a wildly wrong value); function
  forgets the float→double cast and returns `vin_v` directly (would
  fail to compile, but a `static_cast<double>(int_value)` typo
  would produce a different result); function returns the cast of
  an uninitialized stack copy of the optional's contents.

### C13-4. `HalGetVinVoltage.LatestWinsAcrossTwoUpdatesReturnsTheMostRecentVinVoltage`

- **Layer / contract:** D-C12-LATEST-WINS-READ per-function variant
  for `latest_power_state_`. Pins the same "reads cache observer at
  call time, not at install time" contract that C12-7 pinned for
  HAL_GetFPGATime — interleaved poll/read structure catches
  "caches-on-first-call" bugs.
- **Setup:**
  - Fresh region; shim made; install guard installed.
- **Action sequence:**
  1. Drive `power_state{vin_v=11.0f, vin_a=1.0f, brownout=6.0f}` via
     core→shim, poll, OK.
  2. `int32_t status1 = 999; double v1 = HAL_GetVinVoltage(&status1);`
  3. Drive `power_state{vin_v=13.5f, vin_a=2.0f, brownout=7.0f}` via
     core→shim, poll, OK.
  4. `int32_t status2 = 888; double v2 = HAL_GetVinVoltage(&status2);`
- **Expected:**
  - After step 2: `v1 == 11.0`; `status1 == kHalSuccess`.
  - After step 4: `v2 == 13.5`; `status2 == kHalSuccess`.
  - (Both 11.0 and 13.5 are exactly representable in IEEE-754
    single and double precision: 11.0 = 11/1, 13.5 = 27/2.)
- **Bug class:** HAL_GetVinVoltage caches its own copy of `vin_v`
  on first read (would show `v2 == 11.0`); the global pointer is
  captured as a snapshot at install time rather than dereferenced
  at call time (same observable failure); the implementation reads
  from a TU-static shadow that is updated only at install time.

---

## Tests deliberately not added (and why)

- **C13-5 idempotency mirror of C12-8** ("multiple calls without
  intervening poll return the same value"). The cycle-10 trim
  convention applies: per-function tests that re-prove "function
  has no internal state mutation" are coverage padding when the
  per-function read path is structurally identical to the cycle-12
  reference. Cycle 12's C12-8 was load-bearing because it was the
  first encounter with the bug class; cycle 13's mirror would not
  catch a new bug class. Reviewer welcome to argue for inclusion.
- **C13-1b NULL-status guard.** Same reasoning as cycle 12: NULL
  status is UB matching WPILib; testing for NULL would tie us to
  a contract WPILib doesn't honor.
- **C13-x determinism replay.** Same reasoning as cycle 12: cycle
  8's `latest_power_state_` byte-identity replay already pins the
  cache contents; the C ABI wrapper adds no new nondeterminism
  class.
- **C13-x cross-cutting cycle-12 mirrors** (`install_global` /
  `current` re-tests; `kHalSuccess` / `kHalHandleError` constant
  values). Session-level and value-level; pinned by cycle 12.

---

## Cross-test invariants

Same as cycle 12, with these notes:

- All four cycle-13 tests use the cycle-12 anonymous-namespace
  helpers (`make_connected_shim`, `shim_global_install_guard`).
- C13-1 does NOT use the RAII guard (no shim installed).
- C13-2/C13-3/C13-4 all use the RAII guard.
- All tests use `int32_t status = <sentinel>` (999, 888) for the
  status-write-unconditional contract.
- `vin_v`, `vin_a`, `brownout_voltage_v` fixture values are
  bit-distinct (12.5 / 99.25 / 6.75 in C13-3; the C13-4 pairs use
  different values for the same reason) so a "wrong field" bug
  fails loudly on byte equality.

---

## Implementation companion recommendations (non-test)

- **Add the doc comment to `hal_c.h`** mirroring HAL_GetFPGATime's
  comment block — calls out the float→double cast, the same
  status-write-unconditional contract, and the cache-empty success-
  zero behavior.
- **Implementation is a 4-branch read** following HAL_GetFPGATime's
  exact shape: lookup → null check → success check → cast and
  return. The float→double cast is the only schema-specific line;
  everything else is a structural copy. Code-style.md "three
  similar lines is better than a premature abstraction" — the two
  HAL_Get* functions in `hal_c.cpp` after this cycle will be
  near-identical, and that's fine. Future cycles add more peers;
  the natural place to lift a shared helper would be after the
  third or fourth read function lands (mirroring D-C9-NO-HELPER
  → D-C10-EXTRACT-ACTIVE-PREFIX progression).

---

## Open questions

**OQ-C13-IDEMPOTENCY-TEST.** Should cycle 13 add a C13-5
idempotency test mirroring C12-8 (three calls without intervening
poll)? Plan picks NO — the per-function read path is structurally
a copy of HAL_GetFPGATime, and an idempotency bug there would
catch the same code-shape that C12-8 already catches. Reviewer
welcome to argue for the cross-function structural-parity case.

**OQ-C13-PEER-BUNDLING.** Should cycle 13 bundle HAL_GetVinCurrent
(reads `vin_a`) with HAL_GetVinVoltage? Plan picks NO — one HAL_*
per cycle, mirroring the cycle-9/10/11 outbound precedent. The
HAL_GetVinCurrent cycle would be ~3 tests at most (no new design
decision; D-C13-FLOAT-TO-DOUBLE-CAST inherits) but bundling
violates the established "one focused thing per cycle" pattern.
Reviewer welcome to argue for bundling on the "near-identical
sibling" angle.

**OQ-C13-FIXTURE-VIN-CHOICE.** The plan picks
`valid_power_state(12.5, 99.25, 6.75)` for C13-3. The values are
exactly representable in IEEE-754 single and double precision, so
the test asserts exact equality rather than a tolerance. Reviewer
push-back on the value choice or the exact-equality assertion
welcome.
