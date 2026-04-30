# HAL shim core — cycle 6 test plan (notifier_state)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, single review round). Cycle-6 production code
and tests landed and 63-shim-test green under both clang Debug
and GCC Debug + ASan + UBSan.

**Implements:** the sixth TDD cycle of the HAL shim core. Cycles
1–5 are landed and 50-shim-test green. Cycle 6 promotes
`notifier_state` from "fall-through unsupported_payload_schema"
to "accepted with its own latest-wins cache slot."

---

## Why this cycle exists

`notifier_state` is the second variable-size schema (after
`can_frame_batch`). It also has *more* implicit padding bytes
than any prior schema:

- 4-byte interior pad between `count` (offset 0–3) and `slots`
  (offset 8), because `notifier_slot` has `alignof == 8` due to
  its `uint64_t trigger_time_us` field. **Already pinned by the
  cycle-5 C5-R10 `static_assert(offsetof(notifier_state, slots)
  == 8)`.**
- 4 trailing pad bytes per `notifier_slot` (sum of named fields =
  84; sizeof = 88; trailing pad to satisfy alignof 8). With
  `kMaxNotifiers == 32`, the per-slot total is 128 padding bytes
  across the 32 slots.
- Total padding in `notifier_state`: 4 + 128 = **132 bytes** out
  of 2824 total. `operator==` is field-by-field and pins none of
  these. C6-6 must include a `std::memcmp` companion (D-C6-PADDING
  inheritance from D-C3-7 / D-C4-PADDING).

Variable-size dispatch is identical in shape to cycle 4: zero-init
the destination, memcpy `received->payload.size()` bytes (the
active prefix), latest-wins replace.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<notifier_state>& latest_notifier_state() const`.

### Internal additions

- New private member `std::optional<notifier_state>
  latest_notifier_state_;`.
- New `case schema_id::notifier_state:` arm in `tick_boundary`
  dispatch, structurally identical to the cycle-4 `can_frame_batch`
  arm: zero-init `notifier_state{}` then memcpy
  `received->payload.size()` bytes.

### Out of scope

- Inbound schemas other than `notifier_state` and the
  cycle-1/2/3/4/5 set. `notifier_alarm_batch` is cycle 7.
- C HAL ABI surfaces (`HAL_InitializeNotifier`,
  `HAL_UpdateNotifierAlarm`, etc.).
- Outbound `notifier_state`. Shim is purely consumer in cycle 6.

---

## Decisions pinned

- **D-C6-1.** All six cache slots are independent storage.
  Inherits and extends D-C5-1. Pinned by C6-5 over the **ten new
  cross-population direction-pairs** that arise from the sixth slot
  (5 × ns-arm-clobbers-other + 5 × other-arm-clobbers-ns). Prior
  20 directions stay covered by C2-5/C3-5/C4-5/C5-5.
- **D-C6-LATEST-WINS.** Latest-wins, no queue. Inherits.
- **D-C6-VARIABLE-SIZE.** Active-prefix memcpy with
  zero-init-before-write. Inherits D-C4-2 / D-C4-VARIABLE-SIZE.
  `notifier_state{}` zero-init covers (a) the 4-byte interior
  count→slots pad, (b) the 4-byte trailing pad in every unused
  `slots[count..31]`, (c) the 4-byte trailing pad in every active
  slot. Pinned by C6-1 (multi-slot) and C6-1b (zero-count) and
  C6-2b (shrinking notifier list zero-fills trailing slots).
- **D-C6-PADDING.** `notifier_state` has 132 implicit padding
  bytes; `notifier_state::operator==` pins none of them. C6-6's
  determinism test asserts `std::memcmp == 0` directly on
  `latest_notifier_state()`. Inherits D-C3-7 / D-C4-PADDING.
- **D-C6-3.** Schema-specific dispatch is strictly post-session.
  Pinned by C6-4.
- **D-C6-PROBE.** Cycle-1 test 10's probe migrates from
  `notifier_state` to `notifier_alarm_batch` (schema_id 7). Like
  `notifier_state`, `notifier_alarm_batch` is variable-size with
  `offsetof(notifier_alarm_batch, events) == 8` (the array's
  element `notifier_alarm_event` has alignof 8). The
  cycle-5-style `empty_notifier_alarm_batch_prefix` pattern
  applies — an 8-byte zero-init `std::array`, with a
  `static_assert` guard mirroring the cycle-5 pattern.
- **D-C6-4.** No new `shim_error_kind`.
- **D-C6-5.** `latest_notifier_state_` value-initialized as
  `nullopt`.

---

## Test fixtures

Cycle 6 adds three inline helpers to
`tests/backend/tier1/test_helpers.h`:

```cpp
inline notifier_slot valid_notifier_slot(
    std::uint64_t trigger_time_us = 1'000'000,
    hal_handle handle             = 42,
    hal_bool alarm_active         = 1,
    hal_bool canceled             = 0,
    const char* name              = "notifier") {
  notifier_slot s{};  // zero-init covers the 4 trailing padding bytes.
  s.trigger_time_us = trigger_time_us;
  s.handle = handle;
  s.alarm_active = alarm_active;
  s.canceled = canceled;
  if (name != nullptr) {
    const std::size_t n = std::min(std::strlen(name), s.name.size() - 1);
    std::memcpy(s.name.data(), name, n);
  }
  return s;
}

inline notifier_state valid_notifier_state(
    std::span<const notifier_slot> slots = {}) {
  notifier_state state{};  // zero-init covers all padding and unused slots.
  state.count = static_cast<std::uint32_t>(slots.size());
  for (std::size_t i = 0; i < slots.size() && i < state.slots.size(); ++i) {
    state.slots[i] = slots[i];
  }
  return state;
}

inline std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_state& state) {
  const std::size_t active_size =
      offsetof(notifier_state, slots) +
      static_cast<std::size_t>(state.count) * sizeof(notifier_slot);
  return {reinterpret_cast<const std::uint8_t*>(&state), active_size};
}
```

The `active_prefix_bytes` overload for `notifier_state` is added
alongside the cycle-4 `can_frame_batch` overload — same name,
different parameter type, resolves via overload set.

**Implementer checklist:** add `#include "notifier_state.h"` to
`test_helpers.h` (already added to `shim_core_test.cpp` in cycle 5
for the C5-R10 forward-reference). Also `#include <cstring>` if
not transitively included (it is — already in test_helpers.h).

---

## Determinism notes

C6-6 is a strict superset of C5-6 with one extra arrival. The
existing `memcmp` companions on `ds_state` and `can_frame_batch`
carry forward; **a new `memcmp` companion on `notifier_state`** is
added for the 132 padding bytes.

---

## Proposed tests (revision 1)

### C6-1. `ShimCorePoll.AcceptsNotifierStateAndCachesByteEqualValue`

- D-C6-1, D-C6-VARIABLE-SIZE.
- Inputs: shim connected; 3-slot notifier_state via
  `valid_notifier_state(std::array{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
      valid_notifier_slot(300, 3, 1, 1, "gamma")})`. Send via
  `active_prefix_bytes(state)` (8 + 3*88 = 272 bytes).
- Expected: `*shim.latest_notifier_state() == state` (full-struct
  equality); all five sibling slots `nullopt`; lane drained.

### C6-1b. `ShimCorePoll.AcceptsEmptyNotifierState`

- Boundary: count=0, 8-byte minimal active prefix.
- Inputs: `valid_notifier_state()` (default-empty span); send
  `active_prefix_bytes(state)` (8 bytes).
- Expected: `latest_notifier_state()->count == 0`;
  `*latest_notifier_state() == valid_notifier_state()`.

### C6-2. `ShimCorePoll.LatestWinsForRepeatedNotifierStateUpdates`

- D-C6-LATEST-WINS. Two batches with different slot counts and
  different per-slot fields. Full-struct equality after each.
- **Fixture values must be bit-distinct from `notifier_slot{}`**
  default-init in every named field — otherwise a "shim writes
  zeros" bug would pass against an all-zero fixture. Concrete
  example: first batch `valid_notifier_state(std::array{
    valid_notifier_slot(100, 1, 1, 0, "alpha"),
    valid_notifier_slot(200, 2, 0, 1, "beta")})`; second batch
  `valid_notifier_state(std::array{
    valid_notifier_slot(500, 5, 1, 1, "delta"),
    valid_notifier_slot(600, 6, 0, 0, "epsilon"),
    valid_notifier_slot(700, 7, 1, 0, "zeta")})`. Both batches
  non-empty; second has a different slot count and every
  distinguished field changes.

### C6-2b. `ShimCorePoll.ShrinkingNotifierStateClearsTrailingSlotsInCache`

- D-C6-VARIABLE-SIZE — zero-init-before-write. 5-slot batch
  replaced by 2-slot batch; assert `slots[2..4] == notifier_slot{}`
  individually plus a `std::memcmp` byte-level guard on slots[2].
- **Memcmp size: `sizeof(notifier_slot)` (88 bytes)**, not just
  the 4-byte trailing pad region. Using the full slot size catches
  any partial-field-clear bug that zeros named fields while
  preserving the 4-byte trailing pad — same rationale as cycle-4
  C4-2b's `sizeof(can_frame)` choice. Concretely:
  `EXPECT_EQ(std::memcmp(&shim.latest_notifier_state()->slots[2],
  &empty_slot, sizeof(notifier_slot)), 0)` where
  `const notifier_slot empty_slot{};`.
- **The 4-byte interior `count → slots` pad** is also covered by
  zero-init at every write, but C6-2b does not add a dedicated
  assertion for it — the zero-init happens unconditionally before
  every memcpy (not only on shrinking writes), so the cycle-1b
  default test already verifies this property at the moment of
  every write. C6-6's full-struct memcmp pins it across the
  determinism scenario.

### C6-3 family.

7 tests (default + 6 contamination directions):
- **C6-3a** `ShimCoreObservers.NotifierStateNulloptBeforeAnyPoll`.
- **C6-3b** `ShimCorePoll.NotifierStateNulloptAfterBootAckIsAccepted`.
- **C6-3c** `...AfterClockStateIsAccepted` (`bytes_of`).
- **C6-3d** `...AfterPowerStateIsAccepted` (`bytes_of`).
- **C6-3e** `...AfterDsStateIsAccepted` (`bytes_of`).
- **C6-3f** `...AfterCanFrameBatchIsAccepted` (`active_prefix_bytes`,
  empty batch is fine).
- **C6-3g** `...AfterCanStatusIsAccepted` (`bytes_of`).

Family count is now 7. Cycle-4-stated parameterization cutover at
8 fires in cycle 7 — cycle 6 stays linear.

### C6-4. `ShimCorePoll.RejectsNotifierStateBeforeBootAckPreservingLane`

- D-C6-3. Mirrors C4-4 for variable-size schema. Inject 8-byte
  count=0 active prefix via `manually_fill_lane` directly (no
  `notifier_state` struct construction needed; just an
  `std::array<std::uint8_t, 8>{}` payload). All six cache slots
  remain `nullopt`; lane preserved unchanged; expected error is
  `expected_boot_ack_first`.

### C6-5. `ShimCorePoll.AllSixCachesAreIndependentlyMaintained`

- D-C6-1 — covers the ten new cross-population directions.
- 11-step interleaved scenario (`2N-1` for N=6 slots): populate
  each existing slot (steps 1–5), introduce notifier_state (step
  6, catches ns→clock/power/ds/cf/cs — 5 directions), re-write
  each prior slot (steps 7–11, each catches the corresponding
  prior-arm-clobbers-ns direction). All `has_value()`
  prerequisites are `ASSERT_TRUE`. `can_frame_batch` payloads use
  `active_prefix_bytes`.

### C6-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllSixSlots`

- Strict superset of C5-6. Replaces (drops) C5-6 per established
  precedent.
- Scenario: boot_ack, clock@1, power@2, ds@3, cf@4, cs@5,
  notifier_state@6 (3-slot batch), clock@7. Assert all 12
  `has_value()` gates with `ASSERT_TRUE`, full-struct `operator==`
  on all six slots between two setups, **`std::memcmp` on
  `ds_state`, `can_frame_batch`, AND `notifier_state`** (3
  padding-bearing schemas). No memcmp on the 3 padding-free
  schemas (`clock_state`, `power_state`, `can_status`).
- **Memcmp size for `notifier_state` is `sizeof(notifier_state)`
  (2824 bytes), NOT the active-prefix size of the 3-slot scenario
  (272 bytes).** Using the full struct size pins every padding
  byte across all 32 slot entries (active and unused) plus the
  4-byte interior `count → slots` pad. `notifier_state::operator==`
  is field-by-field and pins none of these 132 implicit padding
  bytes; only `std::memcmp` over `sizeof(notifier_state)` does.
  Concretely:
  `EXPECT_EQ(std::memcmp(&*shim_a.latest_notifier_state(),
  &*shim_b.latest_notifier_state(), sizeof(notifier_state)), 0)`.
  Same rationale as cycle-3 C3-6's `sizeof(ds_state)` and cycle-4
  C4-6's `sizeof(can_frame_batch)` choices.

---

## Test rewrites

### C6-R10. test 10 probe migration `notifier_state` → `notifier_alarm_batch`

`notifier_alarm_batch` is variable-size with `offsetof(notifier_alarm_batch,
events) == 8` (because `notifier_alarm_event` has alignof 8 from its
`uint64_t fired_at_us` field). Same 8-byte zero-init prefix pattern
as cycle 5's `notifier_state` migration:

```cpp
constexpr std::size_t kNotifierAlarmBatchPrefixBytes =
    offsetof(notifier_alarm_batch, events);
static_assert(kNotifierAlarmBatchPrefixBytes == 8,
              "notifier_alarm_batch header pad must be 8; alignof "
              "notifier_alarm_event is 8 (uint64_t fired_at_us). "
              "If this static_assert fires, the struct layout "
              "changed and this comment plus the matching "
              "header_size in validator.cpp need updating.");
const std::array<std::uint8_t, kNotifierAlarmBatchPrefixBytes>
    empty_notifier_alarm_batch_prefix{};
// pass directly as the payload — no bytes_of wrapper.
```

Block-comment header: cumulative supported set adds
`notifier_state`; "cycle-5 limit" → "cycle-6 limit"; trailing
migration note points at `error_message_batch` as the cycle-8
next-probe. Inline call-site comment: notifier_alarm_batch is
variable-size, mirrors cycle-5/notifier_state framing, etc.

### C6-R6. drop C5-6 in favor of C6-6

Strict-superset replacement per established precedent.

---

## Cross-test invariants

- All `has_value()` prerequisites before dereference are
  `ASSERT_TRUE`.
- `can_frame_batch` payloads use `active_prefix_bytes`;
  `notifier_state` payloads use `active_prefix_bytes`; fixed-size
  schemas use `bytes_of`.
- Padding-bearing schemas in C6-6 get a `std::memcmp` companion;
  padding-free schemas do not.

## What this plan does NOT cover

- Inbound schemas other than `notifier_state` and prior cycles'
  set.
- C HAL ABI for notifiers.
- Validity-domain checks on notifier fields.
- Post-shutdown / `lane_in_progress` `latest_notifier_state()`
  invariance — by-construction inheritance.

---

## Open questions

None. Variable-size pattern fully inherited from cycle 4; padding-
byte pattern fully inherited from cycle 4; `offsetof(notifier_state,
slots) == 8` already pinned in cycle 5's C5-R10 work.
