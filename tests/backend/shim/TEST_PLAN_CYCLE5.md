# HAL shim core — cycle 5 test plan (can_status)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, two review rounds). Cycle-5 production code
and tests landed and 50-shim-test green under both clang Debug
and GCC Debug + ASan + UBSan.

**Review history:**

- rev 1 → `not-ready`. One blocker, one minor:
  - **C5-R10 left "is `notifier_state` fixed-size or variable-size?"
    as an implementation-time check.** It is not — `validator.cpp`
    pins it as variable-size (`fixed_payload_size` returns 0;
    `expected_variable_payload_size` has an explicit case with
    `header_size = offsetof(notifier_state, slots)`,
    `element_size = sizeof(notifier_slot)`,
    `capacity = kMaxNotifiers`). Sending `bytes_of(notifier_state{})`
    (the full struct) would be rejected by the validator on the
    count↔length mismatch — exactly the trap that bit cycle 3's
    first attempt at C3-R10 and required a post-implementation
    correction. Plan must prescribe the active-prefix approach
    explicitly and pin the size.
  - **`offsetof(notifier_state, slots) == 8`, not 4.**
    `notifier_slot` carries `alignof == 8` (it has a `uint64_t`
    `trigger_time_us` first), so the array's alignment forces
    a 4-byte interior pad after the 4-byte `count`. The active
    prefix for `count == 0` is therefore an **8-byte zero buffer**,
    not the 4-byte buffer that `can_frame_batch` uses. The plan
    must pin this explicitly — a literal find/replace from the
    `can_frame_batch` 4-byte pattern would silently produce a
    framing-validation failure.
  - C5-3f minor: the `can_frame_batch` payload must use
    `active_prefix_bytes(...)`, not `bytes_of(...)`. The plan
    pointed at "mirror C4-3e's structure" without explicitly
    calling out that C4-3e used the `active_prefix_bytes` helper.
    Inline checklist note added.

- rev 2 → this revision. Both rev-1 issues addressed:
  - **C5-R10 bullet 1 rewritten** to definitively prescribe
    `notifier_state{}` zero-init plus an 8-byte zero active
    prefix (= `offsetof(notifier_state, slots)`). The
    "implementation-time check" language is removed. A forward-
    reference note is added pointing cycle-6's plan author at the
    same `offsetof` pin.
  - **C5-3f gains an inline implementer-checklist note** that the
    `can_frame_batch` payload must be built via
    `active_prefix_bytes(valid_can_frame_batch(...))`, not
    `bytes_of(...)`.

**Implements:** the fifth TDD cycle of the HAL shim core. Cycles
1–4 are landed and 40-shim-test green. Cycle 5 promotes `can_status`
from "fall-through unsupported_payload_schema" to "accepted with its
own latest-wins cache slot."

This document is an **addendum** to the cycle-1/2/3/4 plans. The
existing 40 tests must continue to pass without behavior change;
**one cycle-1 test (test 10) is rewritten again** to migrate its
probe schema from `can_status` (now supported) to `notifier_state`
(next still-unsupported by `schema_id` numeric order).

---

## Why this cycle exists

Cycle 4 left the shim with four supported inbound state schemas
and `can_status` unimplemented. `can_status` is structurally the
simplest schema this addendum has handled since `power_state`:
fixed-size (20 bytes), 5 × 4-byte fields (1 `float` + 4 `uint32_t`),
**no padding**, no nested aggregates, no variable-size active
prefix. The cycle-5 test plan reuses cycle-2's shape almost
verbatim with the slot count extended by one and the cross-
population matrix extended accordingly.

The smallest cohesive feature is:

- **Accept** `tick_boundary` envelopes carrying
  `schema_id::can_status` (`payload_bytes == sizeof(can_status) ==
  20`) and byte-copy into a new `latest_can_status_` cache slot.
- **Expose** the new cache via
  `const std::optional<can_status>& latest_can_status() const`.
- **Preserve** every existing cycle-1/2/3/4 contract.

---

## Contract under test (additions only)

System under test is unchanged: `robosim::backend::shim::shim_core`.

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<can_status>& latest_can_status() const` —
  `nullopt` until at least one `tick_boundary`/`can_status`
  envelope has been accepted; latest-wins thereafter.

### New error kinds

**None.**

### Internal additions

- New private member `std::optional<can_status> latest_can_status_;`
  on `shim_core`, sibling to the four existing slots.
- New `case schema_id::can_status:` arm in the `tick_boundary`
  dispatch in `shim_core::poll()`, structurally identical to the
  `clock_state` / `power_state` / `ds_state` arms (fixed-size
  byte-copy via `sizeof(can_status)`, no zero-init dance because
  there is no padding and no variable prefix).

### Out of scope

- Inbound schemas other than `can_status` and the cycle-1/2/3/4
  set. `notifier_state` is cycle 6.
- C HAL ABI surfaces (`HAL_CAN_GetCANStatus`).
- Outbound `can_status`. The shim is purely consumer in cycle 5.

---

## Decisions pinned

- **D-C5-1.** All five cache slots are **independent storage**.
  None mutates any other. Inherits and extends D-C4-1. Pinned by
  C5-5 (cross-slot independence over the eight new direction-pair
  bug classes that arise once a fifth slot exists):
  - cs-arm clobbers clock/power/ds/cf-slot (4 directions)
  - clock/power/ds/cf-arm clobbers cs-slot (4 directions)

  The twelve prior cycle-2/3/4 directions stay covered by C2-5,
  C3-5, and C4-5, which remain in the file unchanged.

- **D-C5-2.** Latest-wins. Inherits.

- **D-C5-3.** Schema-specific dispatch is **strictly post-session**.
  Pre-boot-ack `tick_boundary`/`can_status` surfaces
  `expected_boot_ack_first` from the session, not
  `unsupported_payload_schema`. Pinned by C5-4.

- **D-C5-PROBE.** Cycle-1 test 10's probe schema migrates from
  `can_status` to `notifier_state`. `notifier_state == 6` is the
  next per-tick-allowed schema by `schema_id` numeric order.

- **D-C5-4.** No new `shim_error_kind`.

- **D-C5-5.** `latest_can_status_` value-initialized as
  `std::nullopt` at construction.

- **D-C5-NO-PADDING.** `can_status` carries no padding bytes
  (20 bytes = 5 × 4-byte fields, all naturally aligned). Per the
  cycle-3 D-C3-7 / cycle-4 D-C4-PADDING reasoning, no `std::memcmp`
  companion is needed in C5-6 for `can_status`; the existing
  `operator==` check is byte-equal-to-`memcmp` for this struct. The
  existing `memcmp` assertions on `ds_state` (D-C3-7) and
  `can_frame_batch` (D-C4-PADDING) carry forward into C5-6.

---

## Cross-cutting assertion shape

Same as cycles 1–4: full-struct `operator==` for cache comparisons,
`ASSERT_TRUE` on every `has_value()` prerequisite before any
dereference, explicit-constant fixture values, single-concern
tests.

---

## Test fixtures

Cycle 5 adds one inline helper to `tests/backend/tier1/test_helpers.h`:

```cpp
inline can_status valid_can_status(
    float percent_bus_utilization     = 0.25f,
    std::uint32_t bus_off_count       = 1,
    std::uint32_t tx_full_count       = 2,
    std::uint32_t receive_error_count = 3,
    std::uint32_t transmit_error_count = 4) {
  return can_status{percent_bus_utilization, bus_off_count,
                    tx_full_count, receive_error_count,
                    transmit_error_count};
}
```

**Rationale for the defaults:** all five fields non-zero, distinct,
and bit-distinct from `valid_clock_state()` and `valid_power_state()`
defaults so a "shim writes zeroes" or "shim writes the wrong slot"
bug fails byte-equality. The values are realistic operating-point
reference values (25% bus utilization, single-digit error counters).

**Implementer checklist:** add `#include "can_status.h"` to
`test_helpers.h` (alphabetical between `can_frame.h` and
`clock_state.h`).

---

## Determinism notes

C5-6 is a strict superset of C4-6: same scenario plus a `can_status`
arrival, with the same `ASSERT_TRUE` gates and the same `memcmp`
assertions on `ds_state` and `can_frame_batch`. No new `memcmp`
companion needed for `can_status` (D-C5-NO-PADDING).

---

## Proposed tests (revision 1)

### C5-1. `ShimCorePoll.AcceptsCanStatusAndCachesByteEqualValue`

- **Layer / contract:** Layer 2 HAL shim inbound dispatch;
  `latest_can_status()` accessor; D-C5-1 (no cross-slot
  contamination at first arrival).
- **Bug class caught:** payload byte corruption; wrong cache slot
  written; partial-field copy.
- **Inputs:** shim connected; core sends `tick_boundary`/`can_status`
  at sequence 1 with `valid_can_status()` defaults; `sim_time_us
  = 1'000`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_can_status().has_value())`.
  - `*shim.latest_can_status() == valid_can_status()` (full-struct
    `operator==`).
  - All four other slots `nullopt`.
  - `region.core_to_backend.state == empty`.

### C5-2. `ShimCorePoll.LatestWinsForRepeatedCanStatusUpdates`

- **Layer / contract:** D-C5-2.
- **Bug class:** cache holds first value forever; field-wise
  partial update.
- **Inputs:** two `tick_boundary`/`can_status` envelopes with
  every distinguished field differing between them
  (`valid_can_status(0.10f, 1, 2, 3, 4)` then
  `valid_can_status(0.85f, 9, 8, 7, 6)`).
- **Expected outputs:** after each poll, full-struct equality
  against the corresponding fixture.

### C5-3a. `ShimCoreObservers.CanStatusNulloptBeforeAnyPoll`

- **Layer / contract:** D-C5-5.
- **Suite placement:** `ShimCoreObservers` (cycle-2/3/4 precedent).
- **Inputs:** shim post-`make`, no `poll()`.
- **Expected outputs:** `latest_can_status() == nullopt`.

### C5-3b. `ShimCorePoll.CanStatusNulloptAfterBootAckIsAccepted`

- **Layer / contract:** D-C5-5 + D-C5-1 — `boot_ack` arm does not
  contaminate cs-slot.
- **Inputs / expected outputs:** mirror C4-3b's structure.
  `ASSERT_TRUE(poll().has_value())`, `ASSERT_TRUE(shim.is_connected())`
  (prerequisite), then `EXPECT_FALSE(shim.latest_can_status().has_value())`.

### C5-3c through C5-3f. *Sibling-schema contamination tests.*

Four tests, one per existing inbound schema
(`clock_state`/`power_state`/`ds_state`/`can_frame_batch`), each
mirroring the C4-3c/d/e structure. Each sends one envelope of the
respective sibling schema, asserts the prerequisite via
`ASSERT_TRUE` on that sibling's accessor, then asserts
`EXPECT_FALSE(shim.latest_can_status().has_value())`.

The four tests are:
- C5-3c: `CanStatusNulloptAfterClockStateIsAccepted` (payload =
  `bytes_of(valid_clock_state(...))`).
- C5-3d: `CanStatusNulloptAfterPowerStateIsAccepted` (payload =
  `bytes_of(valid_power_state(...))`).
- C5-3e: `CanStatusNulloptAfterDsStateIsAccepted` (payload =
  `bytes_of(valid_ds_state(...))`).
- C5-3f: `CanStatusNulloptAfterCanFrameBatchIsAccepted`. **Payload
  must be `active_prefix_bytes(valid_can_frame_batch(...))`, not
  `bytes_of(...)`.** `can_frame_batch` is variable-size and the
  framing validator rejects `bytes_of(batch)` on the count↔length
  mismatch. Mirrors the landed C4-5 / C4-6 pattern. Sending an
  empty-batch (`valid_can_frame_batch()` with default-empty span)
  is acceptable here — the prerequisite `ASSERT_TRUE` checks that
  the cf-slot was populated; the contents do not matter for the
  cs-slot non-contamination contract.

The bug class for each is identical in shape: a copy-paste error
from the new `can_status` arm into a sibling arm that accidentally
writes both slots, or a refactor that aliases the dispatch table.
The four arms are distinct functions, so each can fail
independently — hence one test per arm. (The cycle-4 plan's stated
parameterization cutover is at 8 tests in this family; cycle 5
brings it to 6 tests, still under the threshold.)

### C5-4. `ShimCorePoll.RejectsCanStatusBeforeBootAckPreservingLane`

- **Layer / contract:** D-C5-3.
- **Inputs / expected outputs:** mirror C4-4 with schema swapped
  to `can_status`. The injected envelope uses
  `payload_bytes = sizeof(can_status)`, payload =
  `bytes_of(valid_can_status())`. Expected error is
  `expected_boot_ack_first`. All five cache slots nullopt; lane
  preserved unchanged.

### C5-5. `ShimCorePoll.AllFiveCachesAreIndependentlyMaintained`

- **Layer / contract:** D-C5-1 — covers the eight new
  cross-population bug classes.
- **Setup:** shim connected. A nine-step interleaved scenario.
  All `has_value()` prerequisites are `ASSERT_TRUE`.

  - **Step 1: clock_state** `valid_clock_state(100'000)` →
    populated; others nullopt.
  - **Step 2: power_state** `valid_power_state(12.5f, 2.0f, 6.8f)`
    → populated; clock unchanged.
  - **Step 3: ds_state** `valid_ds_state()` → populated; clock,
    power unchanged.
  - **Step 4: can_frame_batch** `valid_can_frame_batch(std::array{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 2, 0xB0)})` (call this
    `cf_first`) → populated; clock, power, ds unchanged.
  - **Step 5: can_status** `valid_can_status()` (defaults) — call
    this `cs_first` → populated; *clock, power, ds, cf all
    unchanged* (***catches "cs-arm clobbers
    clock/power/ds/cf-slot"*** — 4 directions).
  - **Step 6: second clock_state** `valid_clock_state(200'000)` →
    clock updated; power, ds, cf unchanged; **cs unchanged** (===
    `cs_first`) (***catches "clock-arm clobbers cs-slot"***).
  - **Step 7: second power_state** `valid_power_state(13.5f, 5.0f,
    6.5f)` → power updated; clock, ds, cf unchanged; **cs
    unchanged** (***catches "power-arm clobbers cs-slot"***).
  - **Step 8: second ds_state** `valid_ds_state(/*joystick0_axis_count=*/4,
    /*joystick0_axis_0_value=*/-0.25f,
    /*control_bits=*/kControlEnabled | kControlAutonomous,
    /*station=*/alliance_station::blue_2,
    /*type=*/match_type::elimination,
    /*match_number=*/99,
    /*match_time_seconds=*/30.0)` → ds updated; clock, power, cf
    unchanged; **cs unchanged** (***catches "ds-arm clobbers
    cs-slot"***).
  - **Step 9: second can_frame_batch** `valid_can_frame_batch(std::array{
      valid_can_frame(0xCC, 5000, 4, 0xDD)})` (single-frame, distinct
    from `cf_first`) → cf updated; clock, power, ds unchanged;
    **cs unchanged** (***catches "cf-arm clobbers cs-slot"***).
- **Expected outputs:** at every step, every slot's expected state
  is asserted (populated or nullopt; if populated, the expected
  value or "unchanged from prior step"). Every dereference is
  preceded by `ASSERT_TRUE(...has_value())`.

### C5-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllFiveSlots`

- **Layer / contract:** CLAUDE.md non-negotiable #5. Strict
  superset of cycle-4 C4-6. Replaces (drops) C4-6.
- **Inputs:** two independent setups running the identical sequence:
  - boot_ack at seq 0; poll.
  - clock_state at seq 1, `valid_clock_state(50'000)`; poll.
  - power_state at seq 2, `valid_power_state(12.5f, 2.0f, 6.8f)`; poll.
  - ds_state at seq 3, `valid_ds_state()`; poll.
  - can_frame_batch at seq 4, 2-frame batch; poll.
  - can_status at seq 5, `valid_can_status()`; poll.
  - clock_state at seq 6, `valid_clock_state(100'000)`; poll.
- **Expected outputs:**
  - All ten `has_value()` gates `ASSERT_TRUE`.
  - `operator==` equality for all five slots between the two setups.
  - **`std::memcmp` on `ds_state`** (D-C3-7).
  - **`std::memcmp` on `can_frame_batch`** (D-C4-PADDING).
  - No `memcmp` on `can_status` (D-C5-NO-PADDING).
  - `EXPECT_TRUE(setup_a.is_connected())`, ditto setup_b.

---

## Test rewrites (cycle-1/4 plan touched)

### C5-R10. `ShimCorePoll.RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext` (rewrite #4)

- **Source of change:** D-C5-PROBE. Cycle 5 makes `can_status`
  supported.
- **Mechanical changes to test 10:**
  1. **Send line.** Change `schema_id::can_status` →
     `schema_id::notifier_state`. **`notifier_state` is
     definitively variable-size** per `validator.cpp` —
     `fixed_payload_size(schema_id::notifier_state)` returns 0
     and `is_variable_size` returns true; the validator's
     `expected_variable_payload_size` computes
     `expected_payload_bytes == offsetof(notifier_state, slots)
     + count * sizeof(notifier_slot)`. **`bytes_of(notifier_state{})`
     would send the full struct and be rejected on the
     count↔length mismatch** — repeating the trap that bit
     cycle-3's first attempt at C3-R10 and required a post-
     implementation correction. The cycle-5 plan prescribes the
     active-prefix approach now, at plan-writing time.

     Specifically: the local variable `cstatus` is replaced with
     ```cpp
     // notifier_state is variable-size; offsetof(notifier_state, slots)
     // is 8 (not 4) because notifier_slot has alignof == 8 due to
     // its uint64_t trigger_time_us field. count=0 means the active
     // prefix is exactly the 8-byte header (count + 4-byte interior
     // pad to slots-array alignment).
     constexpr std::size_t kNotifierStatePrefixBytes =
         offsetof(notifier_state, slots);
     static_assert(kNotifierStatePrefixBytes == 8,
                   "notifier_state header pad must be 8; alignof "
                   "notifier_slot is 8 (uint64_t trigger_time_us). "
                   "If this static_assert fires, the struct layout "
                   "changed and this comment plus the matching "
                   "header_size in validator.cpp need updating.");
     const std::array<std::uint8_t, kNotifierStatePrefixBytes>
         empty_notifier_prefix{};
     ```
     and the `core.send` payload becomes `empty_notifier_prefix`.
     **Pass `empty_notifier_prefix` directly as the payload span
     — no `bytes_of` wrapper.** The `std::array<std::uint8_t, 8>`
     is already a span of `uint8_t` and converts implicitly to
     `std::span<const std::uint8_t>` at the `core.send` callsite;
     wrapping it via `bytes_of` is redundant. This mirrors the
     cycle-4 `empty_batch_header` pattern but with the correct
     8-byte size for `notifier_state`'s header padding. Using the wrong size (e.g. 4 bytes) would surface
     as a framing-validation rejection in Step A's `core.send`
     call before the test ever exercises the dispatch reject
     contract — a loud failure at implementation time that
     points back here.

     **Forward-reference note for cycle 6's plan author:** when
     cycle 6 lands `notifier_state` as a *supported* schema, the
     dispatch arm must read `received->payload.size()` bytes
     (not `sizeof(notifier_state)`) and zero-init the destination
     before memcpy, exactly as cycle 4 did for `can_frame_batch`.
     `offsetof(notifier_state, slots) == 8` is also load-bearing
     for any test fixture that builds a multi-element notifier
     state — the active-prefix size formula is `8 + count * 88`
     (since `sizeof(notifier_slot) == 88`).
  2. **Inline comment at the call site.** Update the schema name
     from `can_status` to `notifier_state` and update the
     "cycle 5" → "cycle 6" forward references; the trailing
     migration note now points at `notifier_alarm_batch` as the
     cycle-7 next-probe. Note that `notifier_state` is variable-
     size, mirroring cycle-3's `can_frame_batch` framing.
  3. **Block-comment header above the `TEST(...)` line.** Update
     the cumulative supported-schema set to include `can_status`,
     and update "cycle-4 limit" → "cycle-5 limit".
- **Bug class (unchanged).**

### C5-R6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllFourSlots` (drop)

Dropped from the file in favor of C5-6 (strict superset). Same
"no half-finished implementations" rationale that previous cycles
used.

---

## Cross-test invariants (additions)

- One shim per test, no shared state.
- Explicit-constant fixture values; no wall-clock, no PRNG.
- Full-struct `operator==` for cache comparisons; `memcmp`
  companion only where padding is present (`ds_state`,
  `can_frame_batch`).
- All `has_value()` prerequisites before dereferences are
  `ASSERT_TRUE`.
- C5-5 and C5-6 explicitly assert on **all five** cache slots at
  the relevant steps.

## Stub disclosure

No new mocks. C5-4 uses `manually_fill_lane` (cycle-1/2/3/4
precedent for pre-boot-ack injection). The SUT is never mocked.

---

## What this plan does NOT cover

- Inbound schemas other than `can_status` and the cycle-1/2/3/4
  set.
- C HAL ABI (`HAL_CAN_GetCANStatus` etc.) — cycle 9+.
- Validity-domain checks (e.g. `percent_bus_utilization` in
  [0, 1]) — consumer's responsibility.
- Padding-byte memcmp on `clock_state`, `power_state`,
  `can_status` — neither has padding (D-C5-NO-PADDING).
- Post-shutdown / post-`lane_in_progress` `latest_can_status()`
  invariance — by-construction inheritance from cycle-1 tests 12
  and 13.

---

## Open questions

None for cycle 5. The shape directly mirrors cycle 2 with the
slot-count extended by one (and the cross-population matrix
extended by eight directions). No new design decisions beyond the
mechanical D-C5-PROBE migration.
