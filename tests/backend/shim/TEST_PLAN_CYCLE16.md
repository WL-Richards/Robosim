# HAL shim core — cycle 16 test plan (HAL_GetBrownoutVoltage)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, single review round). Round-1 verdict
`ready-to-implement` with one required change (C16-3's plan text
must spell out the explicit `valid_power_state(14.5f, 2.5f, 6.875f)`
call to prevent an implementer from accidentally relying on helper
defaults — addressed in this revision). Reviewer ruled both open
questions: OQ-C16-LIFTING-A-HELPER (still don't lift; defensible
but reviewer noted that a scoped pointer-to-member helper for the
three power_state float readers is one parameter, not four — the
plan's "four template parameters" framing applied to a maximally
general helper, not the natural scoped one); OQ-C16-CLOSE-POWER-
STATE-MILESTONE (the opening-paragraph mention is the right level
of ceremony — factual, not load-bearing). Reviewer's pointer-to-
member observation is captured as a non-blocking note for future
cycles.

**Implements:** the sixteenth TDD cycle of the HAL shim core, the
**fifth C HAL ABI surface**: `extern "C" double
HAL_GetBrownoutVoltage(int32_t* status)` reading from
`latest_power_state_->brownout_voltage_v`. Cycles 1–15 are landed
and 133-shim-test green; full project ctest 448/448 green under
both clang Debug and GCC Debug + ASan + UBSan. Cycle 16 introduces
**no new design decisions** — every contract inherits from cycles
12 and 13. With cycle 16 the **power_state read surface is closed**
(all three float fields wired: `vin_v`, `vin_a`,
`brownout_voltage_v`).

---

## Why this cycle exists

`HAL_GetBrownoutVoltage` is the third float reader on
`latest_power_state_`, mirroring cycles 13 (`vin_v`) and 14
(`vin_a`) against the third and final field. It validates that
**D-C13-FLOAT-TO-DOUBLE-CAST generalizes to a third consumer** with
a different semantic (the brownout *threshold*, not a live
measurement) but the same C ABI shape.

Per the cycle-14 reviewer's OQ-C14-LIFTING-A-HELPER ruling,
**a shared helper is still NOT lifted**. After cycle 16 there are
five HAL_Get* functions in `hal_c.cpp`:

1. `HAL_GetFPGATime` (uint64_t, reads `latest_clock_state_->sim_time_us`)
2. `HAL_GetVinVoltage` (double from float, `latest_power_state_->vin_v`)
3. `HAL_GetVinCurrent` (double from float, `latest_power_state_->vin_a`)
4. `HAL_GetBrownedOut` (HAL_Bool from int32, `latest_clock_state_->browned_out`)
5. `HAL_GetBrownoutVoltage` (double from float, `latest_power_state_->brownout_voltage_v`)

Three of them (2, 3, 5) are float→double readers with the same
shape but a different cache field. A shared helper at this point
would need template parameters for the cache observer (member
function pointer or callable), the field name, and possibly the
return type. The reviewer's previous reasoning — "three template
parameters obscure the one line that actually varies" — still
holds. Lifting waits for either (a) a sixth structurally identical
read function, or (b) a maintenance burden that becomes visible
(e.g. someone has to update all five functions for a new shared
contract). Cycle 16 is the natural OQ-C16-LIFTING-A-HELPER
revisit; the plan picks "still don't lift" with rationale below.

---

## Contract under test

### New public surface

`src/backend/shim/hal_c.h` — additions only:

```cpp
extern "C" {

// ... existing HAL_GetFPGATime, HAL_GetVinVoltage, HAL_GetVinCurrent,
//     HAL_GetBrownedOut ...

// Mirrors WPILib HAL_GetBrownoutVoltage from hal/include/hal/HALBase.h.
// Returns the brownout threshold voltage (NOT the live battery voltage
// — that is HAL_GetVinVoltage). For the shim, the most-recently-
// cached `power_state::brownout_voltage_v` widened from float to
// double via static_cast (D-C13-FLOAT-TO-DOUBLE-CAST inherited).
//
// Status semantics mirror HAL_GetVinVoltage / HAL_GetVinCurrent
// exactly.
double HAL_GetBrownoutVoltage(std::int32_t* status);

}  // extern "C"
```

### Internal additions

- `hal_c.cpp` gains `HAL_GetBrownoutVoltage`. Same 5-line shape as
  `HAL_GetVinVoltage` / `HAL_GetVinCurrent` — null-shim → handle
  error, empty cache → success-zero, populated cache → cast
  `brownout_voltage_v` to double. After this cycle the file has
  five near-identical read functions; per OQ-C16-LIFTING-A-HELPER
  resolution below, no shared helper is lifted yet.

### CMake

No change.

### Out of scope (deferred to later cycles)

- Every clock_state hal_bool reader other than `browned_out`:
  `HAL_GetSystemActive`, `HAL_GetSystemTimeValid`,
  `HAL_GetFPGAButton`, `HAL_GetRSLState` — pure mirrors of cycle 15
  (HAL_Bool, no float cast).
- `HAL_GetControlWord` — first struct-output-parameter surface;
  introduces a new calling convention.
- `HAL_SendError`, `HAL_CAN_SendMessage`, `HAL_Notifier*` —
  outbound-buffering surfaces, fundamentally new mechanic.
- `HAL_Initialize` / `HAL_Shutdown` — lifecycle semantics.
- Threading; LD_PRELOAD wiring; NULL-status guard.
- `HAL_GetCommsDisableCount` (reads `clock_state::comms_disable_count`,
  `uint32_t`) — different return type, different cycle.

---

## Decisions pinned

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR (session-level).
- D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function; pinned by C16-1).
- D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO per-function variant for
  `power_state` (pinned by C16-2). Same exact reasoning as cycle
  13/14 — the WPILib contract for HAL_GetBrownoutVoltage is "always
  succeeds"; the model is "brownout threshold is 0 before the first
  power_state arrives."
- D-C12-STATUS-WRITE-UNCONDITIONAL (per-function via sentinel).
- D-C12-LATEST-WINS-READ (per-function; pinned by C16-4).
- D-C13-FLOAT-TO-DOUBLE-CAST (per-function; pinned by C16-3).

### No new D-C16-* decisions.

If the reviewer finds one, the cycle's scope is wrong.

---

## Test fixtures

No new helpers. Existing `valid_power_state(vin_v, vin_a,
brownout_voltage_v)` covers the surface. The cycle-12 RAII
`shim_global_install_guard` is reused unchanged.

---

## Determinism notes

Same as cycles 13/14: cycle 8's `latest_power_state_` byte-identity
replay covers the cache. No new determinism class. No new
determinism test.

---

## Proposed tests (revision 1)

### C16-1. `HalGetBrownoutVoltage.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function);
  D-C12-STATUS-WRITE-UNCONDITIONAL (per-function).
- **Setup:**
  - `shim_core::install_global(nullptr);`
  - `ASSERT_EQ(shim_core::current(), nullptr);`
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetBrownoutVoltage(&status);`
- **Expected:** `v == 0.0`; `status == kHalHandleError`.
- **Bug class:** null deref; status not written on null path.

### C16-2. `HalGetBrownoutVoltage.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO
  per-function variant for `power_state`.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - `ASSERT_FALSE(shim.latest_power_state().has_value());`
  - `shim_global_install_guard guard{shim};`
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetBrownoutVoltage(&status);`
- **Expected:**
  - `v == 0.0`; `status == kHalSuccess`.
  - `shim.latest_power_state().has_value() == false` (pure read).
- **Bug class:** UB on disengaged optional; wrong status code.

### C16-3. `HalGetBrownoutVoltage.WithCachedPowerStateReturnsItsBrownoutVoltageAndSetsSuccessStatus`

- **Layer / contract:** D-C13-FLOAT-TO-DOUBLE-CAST per-function
  variant for `brownout_voltage_v`; end-to-end protocol-session →
  `latest_power_state_` → C ABI for the **third and final**
  power_state field, validating the cast pattern across all three
  consumers.
- **Setup:**
  - **Bit-distinct value choice — DIFFERENT FROM C13-3 / C14-3 to
    catch "copy-pasted cycle-13/14 body without changing the
    field" bug:** call the helper as
    **`valid_power_state(14.5f, 2.5f, 6.875f)`** with all three
    positional arguments supplied explicitly (NOT relying on the
    helper's defaults of `12.5f / 2.0f / 6.8f`). The values are
    pairwise distinct, all exactly representable in IEEE-754
    single AND double precision (29/2, 5/2, 55/8). The
    distinguished value 6.875 is meaningfully different from
    cycle 13's `brownout_voltage_v=6.75f` and cycle 14's `6.5f`,
    so a "copy-paste with stale fixture" bug fails by `6.875 !=
    6.75` or `6.875 != 6.5`. **Round-1 reviewer required change:**
    the explicit positional spelling protects against an
    implementer accidentally writing `valid_power_state()` with
    only the brownout argument and leaving `vin_v`/`vin_a` at
    helper defaults — which would weaken the wrong-field bug
    detection.
  - **Wrong-field hygiene:** a copy-pasted HAL_GetVinVoltage that
    reads `vin_v` returns 14.5 — **not** equal to expected 6.875,
    caught. A copy-pasted HAL_GetVinCurrent reading `vin_a`
    returns 2.5 — also caught.
  - Drive a `power_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `double v = HAL_GetBrownoutVoltage(&status);`
- **Expected:** `v == 6.875` (exact equality; loss-free
  widening); `status == kHalSuccess`.
- **Bug class:** wrong-field read (`vin_v` returns 14.5 != 6.875;
  `vin_a` returns 2.5 != 6.875); function reads from a different
  cache slot entirely; cast omission (compile error since float can
  silently widen to double — moot).

### C16-4. `HalGetBrownoutVoltage.LatestWinsAcrossTwoUpdatesReturnsTheMostRecentBrownoutVoltage`

- **Layer / contract:** D-C12-LATEST-WINS-READ per-function variant
  for `latest_power_state_->brownout_voltage_v`. Mirrors C13-4 /
  C14-4 with the interleaved poll/read structure that catches
  caches-on-first-call bugs.
- **Setup:**
  - Fresh region; shim made; install guard installed.
- **Action sequence:**
  1. Drive `power_state{vin_v=10.0f, vin_a=4.0f,
     brownout_voltage_v=6.25f}`. Poll, OK.
  2. `int32_t status1 = 999; double v1 = HAL_GetBrownoutVoltage(&status1);`
  3. Drive `power_state{vin_v=12.5f, vin_a=5.5f,
     brownout_voltage_v=6.5f}`. Poll, OK.
  4. `int32_t status2 = 888; double v2 = HAL_GetBrownoutVoltage(&status2);`
- **Expected:**
  - After step 2: `v1 == 6.25` (= 25/4; exact); `status1 == kHalSuccess`.
  - After step 4: `v2 == 6.5` (= 13/2; exact); `status2 == kHalSuccess`.
  - The two updates change `vin_v` and `vin_a` as well, so a
    wrong-field bug also changes between calls and the test does
    not coincidentally pass. (E.g. wrong-field-`vin_v` would show
    v1=10.0, v2=12.5 — both fail.)
- **Bug class:** caches-on-first-call (would show `v2 == 6.25`);
  global-pointer snapshot at install time; wrong-field read.

---

## Tests deliberately not added

- **Idempotency mirror.** Same trim as cycles 13/14/15.
- **NULL status guard.** UB matching WPILib.
- **Cross-cutting cycle-12 mirrors.** Session-level; pinned once.
- **Determinism replay for HAL_GetBrownoutVoltage.** Cycle 8
  covers `latest_power_state_` byte-identity.
- **A test that the brownout-voltage reading is consistent with
  `HAL_GetBrownedOut`'s cached field.** The two surfaces read
  different fields (one is `power_state::brownout_voltage_v` —
  the *threshold*; the other is `clock_state::browned_out` — the
  *trip flag*). They are not semantically linked at the C ABI; the
  WPILib contract treats them as independent. Adding a
  cross-surface consistency test would couple the two cycles
  artificially.

---

## Cross-test invariants

Same as cycles 12–15. Sentinel pattern (999/888), guard usage rules
(C16-1 no guard; C16-2/3/4 with guard), `make_connected_shim`,
distinguishing-fixture-value discipline.

---

## Implementation companion recommendations

- Doc comment on `HAL_GetBrownoutVoltage` in `hal_c.h` mirrors the
  `HAL_GetVinVoltage` comment block — calls out the float→double
  cast and the threshold-vs-live-measurement semantic distinction.
- The 5-line implementation body mirrors `HAL_GetVinVoltage` /
  `HAL_GetVinCurrent` with `cached->brownout_voltage_v` instead.
- Per OQ-C16-LIFTING-A-HELPER (below), do NOT lift a shared helper
  yet. After this cycle `hal_c.cpp` has five near-identical read
  functions; that is still below the threshold the cycle-14
  reviewer set ("wait for fourth or fifth, AND the pattern is
  demonstrably stable; the three current readers differ in return
  type, cache slot, AND field — three template params would
  obscure the one line that varies"). Cycle 16 is the fifth, but
  the variation pattern across them is unchanged: still three
  axes of variation. The function bodies are not yet a maintenance
  burden because (a) no shared contract has changed under them,
  (b) ASan / UBSan / clang-tidy treat them as independent
  functions — there is no false maintenance signal. Defer.

---

## Open questions

**OQ-C16-LIFTING-A-HELPER.** With five HAL_Get* functions in
`hal_c.cpp` after this cycle, does the helper-lifting threshold
flip? Plan picks NO. The variation axes (return type, cache slot,
field, optional cast) have not narrowed since the cycle-14 review;
a hypothetical helper would be:

```cpp
template <typename Ret, auto CacheGetter, auto FieldGetter, auto Cast>
Ret hal_read(int32_t* status) { ... }
```

— four template parameters, hard to read at the call site, and
each of the five typed wrappers would still need to spell out
the parameters. Net code reduction: marginal. Net clarity loss:
real. Defer until a sixth or seventh function lands AND the
variation axes start to narrow (e.g. five readers all returning
`double` from `power_state` would justify a `power_state_double_read`
helper). Reviewer welcome to argue for lifting now if the
duplication is more painful than I see.

**OQ-C16-CLOSE-POWER-STATE-MILESTONE.** Should this cycle's plan
explicitly note "power_state read surface is closed"? Plan says
yes (in the opening paragraph). Reviewer welcome to argue this is
ceremonial and not worth pinning, vs. it being load-bearing for
future cycles to know.
