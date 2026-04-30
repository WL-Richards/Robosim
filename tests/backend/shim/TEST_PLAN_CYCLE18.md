# HAL shim core — cycle 18 test plan (HAL_GetCommsDisableCount)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, two review rounds). Round-1 verdict was `not-ready` with one required
change: C18-3 conflated field-read correctness and cast-wraparound
into a single test. Round-2 splits into two: a new C18-3 with an
in-range value (`comms_disable_count = 42`) pinning field-read
correctness, and a new C18-4 with `0xCAFEBABE` pinning the
wraparound cast seam. The existing latest-wins test renames from
C18-4 → C18-5. Total test count: 5. Both reviewer rulings on the
open questions are upheld: OQ-C18-WRAPAROUND-VALUE-CHOICE resolved
in favor of the split (this revision); OQ-C18-CHANGE-SCHEMA-INSTEAD
resolved as NO (keep `uint32_t` in schema, cast at seam — D-C18-
UINT32-TO-INT32-CAST endorsed as the standing decision for future
`int32_t`-returning readers on `uint32_t` schema fields).

**Implements:** the eighteenth TDD cycle of the HAL shim core, the
**tenth C HAL ABI surface and final clock_state reader**:
`extern "C" int32_t HAL_GetCommsDisableCount(int32_t* status)`
reading from `latest_clock_state_->comms_disable_count`. With cycle
18 the **clock_state read surface is fully closed** (all six fields
wired: `sim_time_us` + 5 hal_bool fields + `comms_disable_count`).

Cycles 1–17 are landed and 141-shim-test green; full project ctest
456/456 green under both clang Debug and GCC Debug + ASan + UBSan.

---

## Why this cycle exists

`HAL_GetCommsDisableCount` is the last clock_state reader. WPILib's
signature (verified against the pinned `kWpilibParitySha`
1fd159938f65ee9d0b22c8a369030b68e0efa82c, tag v2026.2.2):

```c
int32_t HAL_GetCommsDisableCount(int32_t* status);
```

Our schema's `clock_state::comms_disable_count` is `uint32_t` — a
deliberate choice tested at the `UINT32_MAX` boundary (see
`tests/backend/common/protocol_test.cpp:401-406` and TEST_PLAN.md
F3 "comms_disable_count round-trips at 0 and UINT32_MAX"). The
schema's unsigned width gives 4 GiB of headroom on the count's
range; WPILib's signed width caps it at ~2.1 billion.

**Cross-type seam:** the C ABI returns `int32_t` while the schema
holds `uint32_t`. The shim casts at the seam:
`static_cast<int32_t>(cached->comms_disable_count)`. Pinned as
**D-C18-UINT32-TO-INT32-CAST**. For counts ≤ `INT32_MAX` the cast
is identity (the realistic operating range — a 50 Hz robot loop
disabling continuously would need 1.4 years to reach INT32_MAX).
For counts in `[INT32_MAX+1, UINT32_MAX]` the cast is well-defined
in C++20 (wraparound to negative) but practically unreachable.

---

## Contract under test

### New public surface

`src/backend/shim/hal_c.h` — additions only:

```cpp
extern "C" {

// ... existing ...

// Mirrors WPILib HAL_GetCommsDisableCount from hal/include/hal/HALBase.h.
// Returns the number of times communications have been disabled by
// the FPGA's safety system. WPILib returns int32_t; the schema's
// underlying field is uint32_t. The shim casts at the seam
// (D-C18-UINT32-TO-INT32-CAST). For counts <= INT32_MAX the cast is
// identity; for counts in (INT32_MAX, UINT32_MAX] the cast wraps to
// negative per C++20 — practically unreachable for real-world FRC
// operation.
//
// Status semantics mirror the other cycle-12+ HAL_Get* surfaces.
std::int32_t HAL_GetCommsDisableCount(std::int32_t* status);

}  // extern "C"
```

### Internal additions

- `hal_c.cpp` gains `HAL_GetCommsDisableCount` definition. **Inline
  body** following the cycle-12 / cycle-13 / cycle-14 / cycle-16
  shape (no helper lift; this is the only `uint32_t` reader on
  `clock_state` and there is no second consumer to motivate a
  helper). Per OQ-C16-LIFTING-A-HELPER's "wait for second consumer"
  rule, defer.
- No changes to the cycle-17 `clock_state_hal_bool_read` helper —
  it is type-specific to `hal_bool clock_state::*` and cannot be
  extended to handle `uint32_t` fields without templates.

### Out of scope

- A second `clock_state_uint32_read` helper. Cycle 18 has only one
  consumer; no helper lift. If a future cycle adds a second
  `uint32_t` reader on `clock_state`, that cycle revisits the
  question.
- `HAL_GetSystemClockTicksPerMicrosecond` (returns `int32_t`, not
  cached in our model — it's a static FPGA property, not a per-tick
  field). Future cycle if needed; not in v0 scope.
- ds_state readers (HAL_GetControlWord, HAL_GetAlliance, etc.) —
  next major work.
- Outbound-buffering surfaces, lifecycle, threading, etc. — same
  deferrals as cycles 12–17.

---

## Decisions pinned

### New: D-C18-UINT32-TO-INT32-CAST

WPILib's `HAL_GetCommsDisableCount` returns `int32_t`; our schema's
`clock_state::comms_disable_count` is `uint32_t`. The shim casts at
the C ABI seam via `static_cast<int32_t>(cached->comms_disable_count)`.
For values ≤ `INT32_MAX` (~2.1 billion) the cast is identity; for
values in `(INT32_MAX, UINT32_MAX]` the cast is well-defined in
C++20 (modular wraparound to negative). The realistic operating
range for FRC is well below `INT32_MAX`, so the practical contract
is "identity cast". Future `int32_t`-returning readers on `uint32_t`
schema fields inherit this decision.

**Rationale:** Same shape as D-C13-FLOAT-TO-DOUBLE-CAST and
D-C15-HAL-BOOL-SIGNED-PARITY — the schema and WPILib disagree on
type signedness, and the shim is the right seam to bridge them.
Unlike D-C15 (where we changed `hal_bool` to match WPILib because
the values 0/1 are byte-identical between signed/unsigned int32),
here the schema's `uint32_t` is **load-bearing**: existing parity
tests pin `comms_disable_count` round-tripping at `UINT32_MAX`.
Changing the schema field to `int32_t` would invalidate the
boundary test and require re-justifying every wire-format guarantee
the schema makes about this field.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function).
- D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO per-function variant for
  `clock_state`.
- D-C12-STATUS-WRITE-UNCONDITIONAL.
- D-C12-LATEST-WINS-READ.
- D-C13-FLOAT-TO-DOUBLE-CAST (does not apply — no float).
- D-C15-HAL-BOOL-SIGNED-PARITY (does not apply — not HAL_Bool).
- D-C17-CLOCK-STATE-HAL-BOOL-HELPER (does not apply — different
  field type, helper is HAL_Bool-specific).

---

## Test fixtures

No new helpers. Cycle 18 reuses `valid_clock_state` and
`shim_global_install_guard`.

---

## Determinism notes

Same as cycles 13/14/16: cycle 8's `latest_clock_state_` byte-
identity replay covers the cache. The `static_cast<int32_t>(uint32_t)`
is deterministic by the C++20 standard. No new determinism class.

---

## Proposed tests (revision 1)

### C18-1. `HalGetCommsDisableCount.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function);
  D-C12-STATUS-WRITE-UNCONDITIONAL (per-function).
- **Setup:**
  - `shim_core::install_global(nullptr);`
  - `ASSERT_EQ(shim_core::current(), nullptr);`
  - `int32_t status = 999;`
- **Action:** `int32_t n = HAL_GetCommsDisableCount(&status);`
- **Expected:** `n == 0`; `status == kHalHandleError`.
- **Bug class:** null deref; status not written.

### C18-2. `HalGetCommsDisableCount.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO
  per-function variant for `clock_state`.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - `ASSERT_FALSE(shim.latest_clock_state().has_value());`
  - `shim_global_install_guard guard{shim};`
  - `int32_t status = 999;`
- **Action:** `int32_t n = HAL_GetCommsDisableCount(&status);`
- **Expected:**
  - `n == 0`; `status == kHalSuccess`.
  - `shim.latest_clock_state().has_value() == false` (pure read).
- **Bug class:** UB on disengaged optional; wrong status on
  empty-cache path.

### C18-3. `HalGetCommsDisableCount.WithCachedClockStateReturnsItsCommsDisableCountAndSetsSuccessStatus`

- **Layer / contract:** end-to-end protocol-session →
  `latest_clock_state_->comms_disable_count` → C ABI integration.
  Pins that the wrapper reads `comms_disable_count` (NOT
  `sim_time_us` reinterpreted, NOT any `hal_bool` field).
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - `auto state = valid_clock_state(/*sim_time_us=*/50'000);`
    (helper defaults: `sim_time_us=50'000`, `system_active=1`,
    `system_time_valid=1`, `rsl_state=1`, all other fields zero).
    Then explicitly set the distinguished value:
    ```cpp
    state.comms_disable_count = 42;
    ```
    Value `42` is in range for both `uint32_t` and `int32_t`, so
    the cast at the seam is identity; the test reads as a clean
    "field-read correctness" check without ceremony around the
    cast. Wrong-field hygiene: a wrapper reading `sim_time_us`
    returns a 64-bit value truncated to int32 (`50'000` happens
    to fit, but the schema field is `uint64_t` not `uint32_t` —
    a wrong-cache-slot read can be detected via the helper-internal
    type mismatch at compile time, and at runtime via the
    distinguished `42` value); a wrapper reading any `hal_bool`
    field returns `0` or `1`, distinct from `42`.
  - Drive `clock_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `int32_t n = HAL_GetCommsDisableCount(&status);`
- **Expected:** `n == 42`; `status == kHalSuccess`.
- **Bug class:** function reads the wrong cache field; function
  returns 0 for non-zero count (would mean it ignores the cache
  field entirely or always returns the empty-cache path).

### C18-4. `HalGetCommsDisableCount.WithCachedClockStateAndCountAboveInt32MaxReturnsWraparoundCastValue`

- **Layer / contract:** D-C18-UINT32-TO-INT32-CAST. Pins that the
  shim performs `static_cast<int32_t>` on the schema field, with
  the C++20 modular wraparound semantics for values > `INT32_MAX`.
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - `auto state = valid_clock_state(/*sim_time_us=*/50'000);` then
    ```cpp
    state.comms_disable_count = 0xCAFEBABEu;  // > INT32_MAX
    ```
    `0xCAFEBABE` (3'405'691'582) is greater than `INT32_MAX`
    (2'147'483'647). A faithful `uint32 → int32` cast under C++20
    modular conversion produces `-889'275'714` (= 0xCAFEBABE
    interpreted as a two's-complement signed int32).
  - Drive `clock_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `int32_t n = HAL_GetCommsDisableCount(&status);`
- **Expected:**
  - `n == static_cast<int32_t>(std::uint32_t{0xCAFEBABEu})` —
    spelling the expected value via the cast itself rather than
    a hardcoded `-889'275'714` literal. The assertion reads as
    "the C++ standard says this is the result of the cast",
    making the contract self-documenting.
  - `status == kHalSuccess`.
- **Bug class:** function clamps the return to `INT32_MAX` for
  out-of-range values (would fail by returning `2'147'483'647`
  instead of `-889'275'714`); function returns `0` for the
  wraparound case (would mean it rejects out-of-range values);
  function uses `reinterpret_cast` or pointer-casting tricks
  rather than `static_cast` (same observable for valid bit
  patterns, but a future implementation that uses something
  other than the standard's modular conversion fails C18-4 too).

### C18-5. `HalGetCommsDisableCount.LatestWinsAcrossTwoUpdatesReturnsTheMostRecentCount`

- **Layer / contract:** D-C12-LATEST-WINS-READ per-function for
  `latest_clock_state_->comms_disable_count`. Mirrors C13-4 / C14-4
  / C16-4 with the interleaved poll/read structure.
- **Setup:**
  - Fresh region; shim made; install guard installed.
- **Action sequence:**
  1. Drive a clock_state with `comms_disable_count = 7`. Poll, OK.
  2. `int32_t status1 = 999; int32_t n1 = HAL_GetCommsDisableCount(&status1);`
  3. Drive a clock_state with `comms_disable_count = 42`. Poll, OK.
  4. `int32_t status2 = 888; int32_t n2 = HAL_GetCommsDisableCount(&status2);`
- **Expected:**
  - After step 2: `n1 == 7`; `status1 == kHalSuccess`.
  - After step 4: `n2 == 42`; `status2 == kHalSuccess`.
  - The two updates use small, distinct, in-int32-range values to
    keep the test readable; the wraparound case is C18-4's job.
- **Bug class:** caches-on-first-call (would show `n2 == 7`);
  global-pointer snapshot at install time.

---

## Tests deliberately not added

- **Idempotency mirror.** Same trim as cycles 13/14/15/16/17.
- **NULL status guard.** UB matching WPILib.
- **Cross-cutting cycle-12 mirrors.** Session-level; pinned once.
- **Determinism replay.** Cycle 8 covers `latest_clock_state_`.
- **A test that the cast is `static_cast<int32_t>` specifically and
  not, say, `reinterpret_cast` or some other mechanism.** C++ does
  not let us observe the difference at runtime for valid bit patterns.
  C18-3's wraparound assertion (`n == -889'275'714`) catches the
  load-bearing observable.

---

## Cross-test invariants

Same as cycles 12–17. Sentinel pattern (999/888), guard usage rules
(C18-1 no guard, C18-2/3/4 with guard), `make_connected_shim`,
distinguishing-value discipline.

---

## Implementation companion recommendations

- The 5-line implementation body mirrors `HAL_GetVinVoltage` /
  `HAL_GetVinCurrent` / `HAL_GetBrownoutVoltage` exactly, with
  `cached->comms_disable_count` cast via `static_cast<int32_t>`
  instead of `static_cast<double>`. No helper lift.
- Doc comment in `hal_c.h` on `HAL_GetCommsDisableCount` should
  briefly mirror WPILib's documentation and explicitly call out
  D-C18-UINT32-TO-INT32-CAST.
- After this cycle, the **clock_state read surface is fully closed**:
  - `sim_time_us` → `HAL_GetFPGATime` (cycle 12)
  - `system_active` → `HAL_GetSystemActive` (cycle 17)
  - `browned_out` → `HAL_GetBrownedOut` (cycle 15)
  - `system_time_valid` → `HAL_GetSystemTimeValid` (cycle 17)
  - `fpga_button_latched` → `HAL_GetFPGAButton` (cycle 17)
  - `rsl_state` → `HAL_GetRSLState` (cycle 17)
  - `comms_disable_count` → `HAL_GetCommsDisableCount` (cycle 18)

---

## Open questions

**OQ-C18-WRAPAROUND-VALUE-CHOICE.** **Resolved at round 1 in favor
of split.** The single-test form conflated field-read correctness
and the cast-wraparound seam into a test that fails ambiguously.
Round-2 splits into two tests: C18-3 uses an in-range value (`42`)
to pin field-read correctness with no cast ceremony at the
assertion; C18-4 uses `0xCAFEBABE` to pin the wraparound cast with
the expected value spelled as `static_cast<int32_t>(0xCAFEBABEu)`.
The split costs one extra test and buys clearer failure messages.

**OQ-C18-CHANGE-SCHEMA-INSTEAD.** **Resolved at round 1 as NO.**
Reviewer endorsed: the schema's `UINT32_MAX` boundary test
(`tests/backend/common/protocol_test.cpp:401-406`) is load-bearing
for the wire-format contract that `comms_disable_count` is unsigned.
Unlike the cycle-15 `hal_bool` fix (where 0/1 are bit-identical
between signed/unsigned int32 and the fix closed a documented
parity claim), there is no analogous claim on `comms_disable_count`.
The cast at the C ABI seam per D-C18-UINT32-TO-INT32-CAST is the
right architecture, and D-C18-UINT32-TO-INT32-CAST is endorsed as
the standing decision for future `int32_t`-returning readers on
`uint32_t` schema fields.
