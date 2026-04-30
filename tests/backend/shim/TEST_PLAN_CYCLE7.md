# HAL shim core — cycle 7 test plan (notifier_alarm_batch)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, single review round). Round-1 verdict
approved with two plan-text clarifications (C7-2's full-struct
equality form explicit; C7-3 family's `ShimCorePoll` suite
placement explicit for tests b-h) plus one non-blocking
recommendation: use `sizeof(notifier_alarm_batch)` (not the
literal 520) in the implementation memcmp call so the
documentation claim cannot silently rot. Cycle-7 production
code and tests landed and 77-shim-test green under both clang
Debug and GCC Debug + ASan + UBSan.

**Implements:** the seventh TDD cycle of the HAL shim core. Cycles
1–6 are landed and 63-shim-test green. Cycle 7 promotes
`notifier_alarm_batch` from "fall-through unsupported_payload_schema"
to a seventh latest-wins cache slot.

---

## Why this cycle exists

`notifier_alarm_batch` is the third variable-size schema (after
`can_frame_batch` and `notifier_state`). It shares
`notifier_state`'s 8-byte header layout — `offsetof(notifier_alarm_batch,
events) == 8` is already pinned by the cycle-6 C6-R10
`static_assert`. The differences from cycle 6 are mechanical:

- Per-element struct (`notifier_alarm_event`) is 16 bytes, alignof
  8, with a **named** `reserved_pad` field (not implicit padding) —
  so per-element `operator==` covers all bytes of each event, no
  implicit padding inside an event.
- Total implicit padding in `notifier_alarm_batch`: just the
  4-byte interior `count → events` pad. C7-6 still needs a
  `std::memcmp` companion to pin those 4 bytes that
  `operator==` does not see.
- `sizeof(notifier_alarm_batch) == 520` (8-byte header + 32 × 16
  events), much smaller than `notifier_state`'s 2824 bytes.

Variable-size dispatch is identical in shape to cycles 4 and 6.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<notifier_alarm_batch>& latest_notifier_alarm_batch() const`.

### Internal additions

- New private member `std::optional<notifier_alarm_batch>
  latest_notifier_alarm_batch_;`.
- New `case schema_id::notifier_alarm_batch:` arm in
  `tick_boundary` dispatch, structurally identical to the
  cycle-4/6 variable-size pattern.

### Out of scope

- Inbound schemas other than `notifier_alarm_batch` and the
  cycle-1/2/3/4/5/6 set. `error_message_batch` is cycle 8.
- C HAL ABI. Outbound traffic. Reset/reconnect. Threading.

---

## Decisions pinned

- **D-C7-1.** Seven cache slots, all independent. Inherits and
  extends D-C6-1. Pinned by C7-5 over the 12 new cross-population
  direction-pairs (6 × ab-arm-clobbers-other + 6 × other-arm-
  clobbers-ab).
- **D-C7-LATEST-WINS.** Latest-wins, no queue. Inherits.
- **D-C7-VARIABLE-SIZE.** Active-prefix memcpy with
  zero-init-before-write. Inherits D-C4-2 / D-C6-VARIABLE-SIZE.
  Zero-init covers the 4-byte interior count→events pad and any
  unused `events[count..31]`. Pinned by C7-1 (multi-event) and
  C7-1b (zero-count) and C7-2b (shrinking batch zero-fills
  trailing events).
- **D-C7-PADDING.** `notifier_alarm_batch` has 4 implicit
  padding bytes (the count→events interior pad).
  `notifier_alarm_event` has no implicit padding — its
  `reserved_pad` is a *named* field that defaulted `operator==`
  covers. C7-6's `std::memcmp` companion uses
  `sizeof(notifier_alarm_batch)` (520 bytes), pinning the 4
  interior bytes plus all unused events.
- **D-C7-3.** Schema-specific dispatch is strictly post-session.
  Pinned by C7-4.
- **D-C7-PROBE.** Cycle-1 test 10's probe migrates from
  `notifier_alarm_batch` to `error_message_batch` (schema_id 8).
  `error_message_batch` is variable-size; the implementer must
  determine the correct count=0 active-prefix length at
  implementation time by examining `validator.cpp`'s
  `expected_variable_payload_size` for `error_message_batch` and
  the `error_message_batch` struct layout in
  `error_message.h`. **Forward-reference note:**
  `error_message_batch` has a *named* 4-byte
  `reserved_pad` field between `count` and `messages`, so
  `offsetof(error_message_batch, messages)` is likely 8 (the
  named pad is part of the schema layout). The implementer for
  C7-R10 must add a `static_assert(kErrorMessageBatchPrefixBytes ==
  offsetof(error_message_batch, messages))` guard mirroring the
  cycle-5/6 patterns.
- **D-C7-4.** No new `shim_error_kind`.
- **D-C7-5.** `latest_notifier_alarm_batch_` value-initialized as
  `nullopt`.
- **D-C7-FAMILY-LINEAR.** The C-N-3 family reaches **8 tests** at
  cycle 7 (default + 7 contamination directions). The cycle-4
  plan committed to converting to `INSTANTIATE_TEST_SUITE_P` at
  this point. **Cycle 7 defers the parameterization to cycle 8 or
  later.** Rationale: cycle 8 (`error_message_batch`) is the last
  schema cycle and brings the family to 9 tests; converting at
  cycle 8 means the parameterization lands once with the final
  set of probe schemas, rather than twice (once at cycle 7, then
  re-extended at cycle 8). The two-cycle deferral is a tradeoff
  — 8 linear tests at cycle 7 is borderline acceptable; 9 at
  cycle 8 is genuinely starting to feel mechanical and the
  benefit of conversion is clearest at that point. If reviewer
  disagrees, cycle 7 can land the parameterization here; the
  bug-class mechanics are unchanged either way.

---

## Test fixtures

Cycle 7 adds three inline helpers to
`tests/backend/tier1/test_helpers.h`:

```cpp
inline notifier_alarm_event valid_notifier_alarm_event(
    std::uint64_t fired_at_us = 1'000'000,
    hal_handle handle         = 42) {
  notifier_alarm_event event{};  // zero-init covers reserved_pad too.
  event.fired_at_us = fired_at_us;
  event.handle = handle;
  // reserved_pad stays zero from zero-init (D #24).
  return event;
}

inline notifier_alarm_batch valid_notifier_alarm_batch(
    std::span<const notifier_alarm_event> events = {}) {
  notifier_alarm_batch batch{};
  batch.count = static_cast<std::uint32_t>(events.size());
  for (std::size_t i = 0; i < events.size() && i < batch.events.size(); ++i) {
    batch.events[i] = events[i];
  }
  return batch;
}

inline std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_alarm_batch& batch) {
  const std::size_t active_size =
      offsetof(notifier_alarm_batch, events) +
      static_cast<std::size_t>(batch.count) * sizeof(notifier_alarm_event);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}
```

Third overload of `active_prefix_bytes` (after `can_frame_batch`
and `notifier_state`); same name resolves via parameter type.

---

## Determinism notes

C7-6 is a strict superset of C6-6 with one extra arrival. The
existing memcmps on `ds_state`, `can_frame_batch`, and
`notifier_state` carry forward; **a new memcmp companion on
`notifier_alarm_batch`** is added for the 4 interior padding bytes.

---

## Proposed tests (revision 1)

### C7-1. `ShimCorePoll.AcceptsNotifierAlarmBatchAndCachesByteEqualValue`

- D-C7-1, D-C7-VARIABLE-SIZE.
- Inputs: 3-event batch via
  `valid_notifier_alarm_batch(std::array{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
      valid_notifier_alarm_event(300, 3)})`. Send via
  `active_prefix_bytes(batch)` (8 + 3*16 = 56 bytes).
- Expected: `*shim.latest_notifier_alarm_batch() == batch`; all
  six sibling slots `nullopt`; lane drained.

### C7-1b. `ShimCorePoll.AcceptsEmptyNotifierAlarmBatch`

- count=0; 8-byte minimal active prefix.
- Expected: `count == 0`; full-struct equality against
  `valid_notifier_alarm_batch()`.

### C7-2. `ShimCorePoll.LatestWinsForRepeatedNotifierAlarmBatchUpdates`

- D-C7-LATEST-WINS. Two non-empty batches with bit-distinct
  fields. Concrete fixtures specified to avoid the cycle-6 review
  flag: first batch `valid_notifier_alarm_batch(std::array{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2)})`; second batch
  `valid_notifier_alarm_batch(std::array{
      valid_notifier_alarm_event(500, 5),
      valid_notifier_alarm_event(600, 6),
      valid_notifier_alarm_event(700, 7)})`.
- **Assertion form:** full-struct `operator==` after each
  arrival. After first poll:
  `EXPECT_EQ(*shim.latest_notifier_alarm_batch(), first)`. After
  second poll: `EXPECT_EQ(*shim.latest_notifier_alarm_batch(),
  second)`. Mirrors the cycle-4/6 latest-wins test pattern.

### C7-2b. `ShimCorePoll.ShrinkingNotifierAlarmBatchClearsTrailingEventsInCache`

- D-C7-VARIABLE-SIZE. 5-event → 2-event shrink. Assert
  `events[2..4] == notifier_alarm_event{}` plus byte-level
  `std::memcmp(&events[2], &empty_event, sizeof(notifier_alarm_event))`
  guard. Per cycle-4/6 reasoning, the memcmp uses the full event
  size (16 bytes), not just any specific pad region. Note that
  `notifier_alarm_event` has no *implicit* padding (its
  `reserved_pad` is a named field), so the byte-level guard is
  defensive — it pins the same property that field-by-field
  `operator==` already pins. Keeping it for consistency with the
  cycle-4/6 family pattern.

### C7-3 family.

8 tests (default + 7 contamination directions per
D-C7-FAMILY-LINEAR). **Suite placement:** C7-3a in
`ShimCoreObservers` (matches cycle-1 test-4 / C2-3a / C3-3a /
C4-3a / C5-3a / C6-3a precedent for post-make pre-poll observer
state). C7-3b through C7-3h in `ShimCorePoll` (matches the
landed cycle-2 through cycle-6 contamination tests; each runs a
`poll()` cycle to drive the sibling-schema arrival).
- C7-3a `ShimCoreObservers.NotifierAlarmBatchNulloptBeforeAnyPoll`.
- C7-3b `ShimCorePoll.NotifierAlarmBatchNulloptAfterBootAckIsAccepted`.
- C7-3c `ShimCorePoll.NotifierAlarmBatchNulloptAfterClockStateIsAccepted`
  (`bytes_of`).
- C7-3d `ShimCorePoll.NotifierAlarmBatchNulloptAfterPowerStateIsAccepted`
  (`bytes_of`).
- C7-3e `ShimCorePoll.NotifierAlarmBatchNulloptAfterDsStateIsAccepted`
  (`bytes_of`).
- C7-3f `ShimCorePoll.NotifierAlarmBatchNulloptAfterCanFrameBatchIsAccepted`
  (`active_prefix_bytes`).
- C7-3g `ShimCorePoll.NotifierAlarmBatchNulloptAfterCanStatusIsAccepted`
  (`bytes_of`).
- C7-3h `ShimCorePoll.NotifierAlarmBatchNulloptAfterNotifierStateIsAccepted`
  (`active_prefix_bytes`).

### C7-4. `ShimCorePoll.RejectsNotifierAlarmBatchBeforeBootAckPreservingLane`

- D-C7-3. Inject 8-byte count=0 active prefix via
  `manually_fill_lane`. All seven cache slots remain `nullopt`;
  expected `expected_boot_ack_first`.

### C7-5. `ShimCorePoll.AllSevenCachesAreIndependentlyMaintained`

- D-C7-1 — covers the 12 new cross-population directions.
- 13-step interleaved scenario (`2N-1` for N=7 slots). Steps
  1–6 populate existing slots, step 7 introduces
  notifier_alarm_batch (catches ab→6 prior slots, 6 directions),
  steps 8–13 re-write each prior slot (each catches the
  corresponding prior-arm-clobbers-ab direction, 6 directions).
  All `has_value()` prerequisites are `ASSERT_TRUE`.
  Variable-size payloads use `active_prefix_bytes`.

### C7-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllSevenSlots`

- Strict superset of C6-6. Replaces (drops) C6-6.
- Scenario: boot_ack, clock@1, power@2, ds@3, cf@4, cs@5,
  notifier_state@6 (3-slot batch), notifier_alarm_batch@7
  (3-event batch), clock@8.
- 14 `has_value()` gates with `ASSERT_TRUE`; full-struct
  `operator==` on all seven slots between two setups; `memcmp`
  on `ds_state`, `can_frame_batch`, `notifier_state`, AND
  `notifier_alarm_batch` (4 padding-bearing schemas). The
  notifier_alarm_batch memcmp uses
  `sizeof(notifier_alarm_batch)` (520 bytes), pinning the 4
  interior pad bytes plus all 32 event slots' contents (active
  and unused).

---

## Test rewrites

### C7-R10. test 10 probe migration `notifier_alarm_batch` → `error_message_batch`

`error_message_batch` is variable-size. Per `error_message.h`:
the struct is `count` (4 bytes) + named `reserved_pad[4]`
(4 bytes) + `messages[8]`. So
`offsetof(error_message_batch, messages) == 8`. The named
`reserved_pad` is part of the *schema* layout — it's not
implicit C++ padding, but it's part of the wire bytes the
validator expects to see. The active prefix for count=0 is
therefore the same 8-byte zero buffer as the cycle-5/6 patterns.

Mechanical changes mirror C6-R10:
1. Send line: `schema_id::notifier_alarm_batch` →
   `schema_id::error_message_batch`. Local variable
   `empty_notifier_alarm_batch_prefix` →
   `empty_error_message_batch_prefix`. Add
   `static_assert(kErrorMessageBatchPrefixBytes == 8, "...")`.
   Update guarding comment to reference
   `error_message_batch`'s named-pad layout.
2. Inline call-site comment: schema name and forward reference
   updated to "cycle 8 lands error_message_batch."
3. Block-comment header: cumulative supported set adds
   `notifier_alarm_batch`; "cycle-6 limit" → "cycle-7 limit";
   migration note: "cycle 8 will land error_message_batch and
   migrate this probe..." — but cycle 8 is the LAST per-tick
   schema, so the migration note must also state that
   **cycle 8's rewrite of test 10 will need a different
   approach** (deletion of the test and replacement with a
   positive test, since no still-unsupported per-tick schema
   remains after cycle 8). This is a forward-reference for cycle
   8's plan author.

### C7-R6. drop C6-6 in favor of C7-6

Strict-superset replacement.

---

## Cross-test invariants

Same as cycles 4–6. `active_prefix_bytes` for variable-size,
`bytes_of` for fixed-size, `ASSERT_TRUE` on prerequisites,
memcmp companion on padding-bearing schemas.

## What this plan does NOT cover

- C HAL ABI for notifier alarms (consumer-side).
- Validity-domain checks on alarm fields.
- Post-shutdown / lane_in_progress invariance — by-construction
  inheritance.

---

## Open questions

**D-C7-FAMILY-LINEAR** — defers parameterization to cycle 8.
Reviewer push-back welcome if they want the conversion to land
in cycle 7 with the family at 8 tests.
