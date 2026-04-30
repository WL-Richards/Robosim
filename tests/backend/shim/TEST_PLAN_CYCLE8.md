# HAL shim core — cycle 8 test plan (error_message_batch)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, two review rounds). Round-1 verdict
`not-ready` with one blocker (C8-6 framing) and several
non-blocking clarifications; round-2 verdict
`ready-to-implement`. Cycle-8 production code and tests landed
and 91-shim-test green under both clang Debug and GCC Debug +
ASan + UBSan; full project ctest 406/406 green under both
toolchains. Rev 2 addressed:

- **Blocker:** C8-6 reframed as **correction-and-extension** of
  C7-6, not strict superset. Documents that the landed C7-6
  implementation (`shim_core_test.cpp:2694–2805`) inadvertently
  omitted `notifier_alarm_batch` from the scenario, the
  `has_value()` gates, the `operator==` checks, and the `memcmp`
  companion — despite the test name "AllSevenSlots" and the
  cycle-7 plan calling for the coverage. C8-6 is therefore
  closing a pre-existing 1-slot determinism gap (the 4-byte
  interior `count → events` implicit pad in
  `notifier_alarm_batch` has not been pinned by any landed test)
  alongside extending coverage to the new 8th slot.
- **Non-blocking (addressed):** C8-2b's memcmp purpose disambiguated
  from C8-6's omission rationale. C8-5's step-8 comprehensive
  snapshot expectation made explicit. `#include "error_message.h"`
  added to the test-helpers checklist. Recommend
  `static_assert(sizeof(error_message_batch) == 18600)` companion
  in `error_message.h`.

**Implements:** the eighth (and final per-tick payload) TDD cycle of
the HAL shim core. Cycles 1–7 are landed and 77-shim-test green.
Cycle 8 promotes `error_message_batch` from "fall-through
unsupported_payload_schema" to an eighth latest-wins cache slot.

---

## Why this cycle exists

`error_message_batch` is the fourth (and last) variable-size schema
and the last per-tick payload schema overall. It shares the
8-byte-header / variable-active-prefix shape introduced in cycle 4
and refined in cycles 6 and 7. Two things distinguish it from prior
cycles:

1. **Zero implicit C++ padding.** Per `error_message.h`:
   - `error_message_batch` has a *named* `reserved_pad[4]` field
     between `count` (offset 0–3) and `messages` (offset 8). Because
     `error_message` has `alignof == 4`, the C++ compiler would not
     have inserted any implicit pad here — the named pad is a
     deliberate schema-layout reservation for future element
     alignment growth (see `error_message.h:62-63` block comment).
   - `error_message` has a *named* `reserved_pad[3]` field between
     `truncation_flags` (offset 16, 1 byte) and `details` (offset
     20, alignof 1). Again, deliberate, not implicit.
   - All other fields are naturally aligned with no gaps.
   - `sizeof(error_message_batch) == 18600` (8-byte header +
     8 × 2324 element bytes), `alignof == 4`. **No implicit padding
     bytes anywhere.** Defaulted `operator==` covers every byte.
   - Consequence: C8-6's determinism replay does **not** add a
     memcmp companion for `error_message_batch` (operator== covers
     every byte). The four prior padding-bearing schemas
     (`ds_state`, `can_frame_batch`, `notifier_state`,
     `notifier_alarm_batch`) still need their memcmp companions
     carried forward. Cycle 8 stays at 4 memcmps total, same as
     cycle 7.

2. **Last per-tick schema → no probe to migrate test 10 to.** The
   `unsupported_payload_schema` reject branch in `shim_core::poll`
   becomes unreachable from any valid envelope after cycle 8. The
   forward-reference note in C7-R10 explicitly anticipated this:

   > cycle 8's rewrite of test 10 will need a different approach
   > (deletion of the test and replacement with a positive test,
   > since no still-unsupported per-tick schema remains after cycle 8)

   Cycle 8's plan resolves this as **C8-R10: deletion** (not a
   replacement). The "positive arrival" property the deletion
   would otherwise leave uncovered is already pinned by the
   cycle-N-1 family of tests (`AcceptsXAndCachesByteEqualValue`,
   one per accepted schema), and after cycle 8 lands C8-1 covers
   `error_message_batch` directly. The session-counter-advance-
   through-reject contract is structurally pinned across cycles 2–7
   for the seven other schemas; no new probe is needed at cycle 8.

   The production-code reject branch in `shim_core.cpp` is **kept**
   intentionally as a defensive forward-compat structural guard:
   if a post-v0 schema is added to `protocol_version.h` and to the
   validator's allowed set but not yet wired in the shim's
   dispatch, the branch ensures a loud failure rather than silent
   discard. Per CLAUDE.md non-negotiable #1 ("no shortcuts"),
   removing the branch to make all production code "live" would
   trade defensiveness for line-count and isn't worth it.

Variable-size dispatch is identical in shape to cycles 4, 6, and 7.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<error_message_batch>& latest_error_message_batch() const`.

### Internal additions

- New private member `std::optional<error_message_batch>
  latest_error_message_batch_;`.
- New `case schema_id::error_message_batch:` arm in
  `tick_boundary` dispatch, structurally identical to the
  cycle-4/6/7 variable-size pattern.

### Out of scope

- All inbound schemas other than the cycle-1 through cycle-8 set —
  but at cycle 8 the per-tick schema set is closed; no further
  inbound-schema cycles remain.
- C HAL ABI for error reporting (consumer-side `HAL_SendError`,
  `HAL_SendConsoleLine`, etc.). Outbound traffic past `boot`.
  Reset/reconnect. Threading.

---

## Decisions pinned

- **D-C8-1.** Eight cache slots, all independent. Inherits and
  extends D-C7-1. Pinned by C8-5 over the 14 new cross-population
  direction-pairs (7 × ab-arm-clobbers-other + 7 × other-arm-
  clobbers-ab) at the 8-slot interleaved scenario.
- **D-C8-LATEST-WINS.** Latest-wins, no queue. Inherits.
- **D-C8-VARIABLE-SIZE.** Active-prefix memcpy with
  zero-init-before-write. Inherits D-C4-2 / D-C6-VARIABLE-SIZE /
  D-C7-VARIABLE-SIZE. Zero-init covers any unused
  `messages[count..7]`. Pinned by C8-1 (multi-message), C8-1b
  (zero-count), and C8-2b (shrinking batch zero-fills trailing
  messages).
- **D-C8-PADDING-FREE.** `error_message_batch` and
  `error_message` have **zero implicit C++ padding bytes**. The
  4-byte interior reserved (count → messages) and the 3-byte
  reserved after `truncation_flags` are both *named* fields; the
  defaulted `operator==` covers every byte. C8-6 carries forward
  the cycle-7 four memcmps on (`ds_state`, `can_frame_batch`,
  `notifier_state`, `notifier_alarm_batch`) but adds **no**
  memcmp on `error_message_batch`. C8-2b nonetheless includes a
  byte-level `std::memcmp` companion on the cleared trailing
  message slots, for consistency with the cycle-4/6/7 family
  pattern (defensive, not informational beyond what `operator==`
  already pins).
- **D-C8-3.** Schema-specific dispatch is strictly post-session.
  Pinned by C8-4.
- **D-C8-PROBE-RETIRED.** Test 10's "unsupported probe" contract
  is retired at cycle 8. No still-unsupported per-tick schema
  remains, so there is no probe to migrate to. The production-code
  reject branch is kept defensively (see "Why this cycle exists"
  point 2 above); the test is deleted outright. No replacement
  test is added — the session-counter-advance-through-reject
  contract is already pinned across cycles 2–7.
- **D-C8-4.** No new `shim_error_kind`.
- **D-C8-5.** `latest_error_message_batch_` value-initialized as
  `nullopt`.
- **D-C8-FAMILY-LINEAR.** The C-N-3 family reaches **9 tests** at
  cycle 8 (default + 8 contamination directions). The cycle-7
  plan committed to converting to `INSTANTIATE_TEST_SUITE_P` "at
  cycle 8 or later." **Cycle 8 stays linear.** Rationale: cycle 8
  is the last per-tick schema cycle; the future-proofing benefit
  of parameterization (one more parameter row when adding a new
  schema) does not materialize because no new per-tick schemas
  are planned. The readability cost of `INSTANTIATE_TEST_SUITE_P`
  (parameter rows lose the descriptive `NulloptAfterXIsAccepted`
  test name and gain `ContaminationCornerCases/.../N`) is real;
  9 hand-written tests is borderline acceptable but readable.
  Reviewer push-back welcome; the bug-class mechanics are
  unchanged either way. Note that C8-3a stays separate from the
  contamination subset regardless because its structure (pre-poll
  observer) differs from C8-3b–i (post-poll arrival).
- **D-C8-DEAD-BRANCH.** The `unsupported_payload_schema` arm in
  `shim_core::poll`'s `tick_boundary` switch is defensible dead
  code at cycle 8 and is **kept**. See "Why this cycle exists"
  point 2 for the rationale. No test pins this branch directly;
  if a post-v0 schema is added without wiring, the structural
  guard is its own diagnostic.

---

## Test fixtures

Cycle 8 adds three inline helpers to
`tests/backend/tier1/test_helpers.h`. The header does not yet
include `error_message.h`; the implementer must add
`#include "error_message.h"` near the existing schema includes
before the helpers will compile.

```cpp
inline error_message valid_error_message(
    std::int32_t error_code             = 1,
    hal_bool severity                   = 1,
    hal_bool is_lv_code                 = 0,
    hal_bool print_msg                  = 1,
    std::uint8_t truncation_flags       = 0,
    std::string_view details_text       = "",
    std::string_view location_text      = "",
    std::string_view call_stack_text    = "") {
  error_message msg{};  // zero-init covers reserved_pad and the
                        // tail of every fixed-size buffer.
  msg.error_code       = error_code;
  msg.severity         = severity;
  msg.is_lv_code       = is_lv_code;
  msg.print_msg        = print_msg;
  msg.truncation_flags = truncation_flags;
  // reserved_pad stays zero from zero-init.
  // Copy at most buffer_size - 1 bytes; final byte stays NUL.
  const auto copy_into = [](auto& buf, std::string_view s) {
    const std::size_t n = std::min(s.size(), buf.size() - 1);
    std::memcpy(buf.data(), s.data(), n);
  };
  copy_into(msg.details,    details_text);
  copy_into(msg.location,   location_text);
  copy_into(msg.call_stack, call_stack_text);
  return msg;
}

inline error_message_batch valid_error_message_batch(
    std::span<const error_message> messages = {}) {
  error_message_batch batch{};
  batch.count = static_cast<std::uint32_t>(messages.size());
  for (std::size_t i = 0;
       i < messages.size() && i < batch.messages.size();
       ++i) {
    batch.messages[i] = messages[i];
  }
  // batch.reserved_pad stays zero from zero-init.
  return batch;
}

inline std::span<const std::uint8_t> active_prefix_bytes(
    const error_message_batch& batch) {
  const std::size_t active_size =
      offsetof(error_message_batch, messages) +
      static_cast<std::size_t>(batch.count) * sizeof(error_message);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}
```

Fourth overload of `active_prefix_bytes` (after `can_frame_batch`,
`notifier_state`, and `notifier_alarm_batch`); same name resolves
via parameter type.

---

## Determinism notes

C8-6 is a **correction-and-extension** of the landed C7-6 test,
not a strict superset. The cycle-7 plan called for C7-6 to cover
all 7 slots with `notifier_alarm_batch` included in the scenario,
the `has_value()` gates, the `operator==` checks, and a
4th `memcmp` companion (since `notifier_alarm_batch` has 4 bytes
of implicit interior padding between `count` and `events`). The
landed implementation at `tests/backend/shim/shim_core_test.cpp:
2694–2805` *does not include `notifier_alarm_batch`* — the
`run_scenario` lambda has only 6 distinct schemas (the 7th step
is a *second* `clock_state` at line 2750), the `has_value()`
gate count is 12 (6 slots × 2 setups, line 2765 comment) instead
of 14, the `operator==` block has 6 checks instead of 7, and the
memcmp block has 3 companions (`ds_state`, `can_frame_batch`,
`notifier_state`) instead of 4. The test name and block-comment
header both claim "all seven slots," but the implementation
covers six.

Consequence: **the 4-byte interior `count → events` implicit
padding in `notifier_alarm_batch` is not pinned by any landed
determinism test today.** This is a real coverage gap — the
sole determinism replay in the suite. C8-6 corrects this gap
while extending coverage to the new 8th slot.

What C8-6 adds vs what is currently in shim_core_test.cpp's
landed AllSevenSlots test:

- A `notifier_alarm_batch` scenario step (step 7, between
  `notifier_state` and `error_message_batch`). 3-event batch
  with bit-distinct fields, sent via `active_prefix_bytes`.
- An `error_message_batch` scenario step (step 8). 3-message
  batch with bit-distinct fields per C8-1's fixtures.
- A trailing `clock_state` arrival (step 9) to verify that an
  unrelated post-emb arrival does not disturb slots 1–7.
- 4 new `has_value()` gates: `latest_notifier_alarm_batch()`
  and `latest_error_message_batch()` × 2 setups. Total goes
  12 → 16 (8 slots × 2 setups).
- 2 new `operator==` checks: one for `notifier_alarm_batch`,
  one for `error_message_batch`. Total goes 6 → 8.
- 1 new `memcmp` companion: `notifier_alarm_batch` (its 4 bytes
  of implicit interior pad). Total goes 3 → 4. **No memcmp is
  added for `error_message_batch`** because it has zero implicit
  C++ padding bytes (D-C8-PADDING-FREE) — defaulted `operator==`
  is field-by-field across all named fields and pins every byte.

The cycle-8 determinism replay covers all 8 cache slots with
full-struct `operator==` and 4 memcmp companions on the four
padding-bearing schemas (`ds_state`, `can_frame_batch`,
`notifier_state`, `notifier_alarm_batch`).

---

## Proposed tests (revision 1)

### C8-1. `ShimCorePoll.AcceptsErrorMessageBatchAndCachesByteEqualValue`

- D-C8-1, D-C8-VARIABLE-SIZE.
- Inputs: 3-message batch via
  `valid_error_message_batch(std::array{
      valid_error_message(/*error_code=*/100, /*severity=*/1,
                          /*is_lv_code=*/0, /*print_msg=*/1,
                          /*truncation_flags=*/0,
                          "first detail", "first location",
                          "first call stack"),
      valid_error_message(200, 0, 1, 0, kErrorTruncDetails,
                          "second detail", "second location",
                          "second call stack"),
      valid_error_message(300, 1, 0, 1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail", "third location",
                          "third call stack")})`. Send via
  `active_prefix_bytes(batch)` (8 + 3 × 2324 = 6980 bytes).
- Expected: `*shim.latest_error_message_batch() == batch`; all
  seven sibling slots `nullopt`; lane drained.
- **Why these inputs:** every named field gets a bit-distinct
  value across the three messages, including the 8-bit
  `truncation_flags` field whose top bits are unused. A "shim
  copies only the first N bytes of each message" bug would fail
  on either the location/call_stack content or on the
  truncation_flags differing across messages. A "shim treats
  reserved_pad as implicit" (e.g. memcpy-element-by-element with
  the wrong offset) would fail on any field after offset 16.

### C8-1b. `ShimCorePoll.AcceptsEmptyErrorMessageBatch`

- count=0; 8-byte minimal active prefix
  (`offsetof(error_message_batch, messages)`, the 4-byte count
  plus the named 4-byte reserved_pad).
- Expected: `count == 0`; full-struct equality against
  `valid_error_message_batch()`.
- Note: this is the same minimal-prefix case test 10 used as its
  reject probe under cycle 7 (C7-R10). At cycle 8 the same
  payload is now a positive arrival.

### C8-2. `ShimCorePoll.LatestWinsForRepeatedErrorMessageBatchUpdates`

- D-C8-LATEST-WINS. Two non-empty batches with bit-distinct
  fields. First batch:
  `valid_error_message_batch(std::array{
      valid_error_message(10, 1, 0, 1, 0,
                          "alpha details", "alpha location",
                          "alpha stack"),
      valid_error_message(20, 0, 1, 0, kErrorTruncDetails,
                          "beta details", "beta location",
                          "beta stack")})`. Second batch:
  `valid_error_message_batch(std::array{
      valid_error_message(50, 1, 1, 1, kErrorTruncLocation,
                          "delta details", "delta location",
                          "delta stack"),
      valid_error_message(60, 0, 0, 0,
                          kErrorTruncDetails | kErrorTruncCallStack,
                          "epsilon details", "epsilon location",
                          "epsilon stack"),
      valid_error_message(70, 1, 0, 1, kErrorTruncCallStack,
                          "zeta details", "zeta location",
                          "zeta stack")})`. Both batches non-empty;
  second has a different message count and every distinguished
  field changes.
- **Assertion form:** full-struct `operator==` after each arrival.
  After first poll:
  `EXPECT_EQ(*shim.latest_error_message_batch(), first)`. After
  second poll:
  `EXPECT_EQ(*shim.latest_error_message_batch(), second)`. Mirrors
  the cycle-4/6/7 latest-wins test pattern.

### C8-2b. `ShimCorePoll.ShrinkingErrorMessageBatchClearsTrailingMessagesInCache`

- D-C8-VARIABLE-SIZE. 5-message batch followed by 2-message batch.
  Assert `messages[2..4] == error_message{}` individually plus a
  byte-level `std::memcmp(&messages[2], &empty_message,
  sizeof(error_message))` defensive guard. Per cycle-4/6/7
  reasoning, the memcmp uses the full element size (2324 bytes),
  not just any specific reserved-pad region.
- **Memcmp purpose (distinguishing from C8-6's omission):** the
  memcmp here targets a **trailing-cleared slice**
  (`messages[2..4]`) and serves the same purpose as C4-2b's
  per-frame trailing-pad check and C6-2b/C7-2b's per-slot/event
  checks: it pins that "the destination buffer was zeroed before
  the active prefix was memcpy'd into it." That property is
  about the destination state at the trailing-elements offset,
  not about implicit padding inside an element. C8-6 omits a
  *full-struct* memcmp on `error_message_batch` because the
  struct has zero implicit padding bytes, so a full-struct
  memcmp would be redundant with `operator==` (D-C8-PADDING-FREE).
  The two cases are answering different questions: C8-2b asks
  "did the shim zero-fill before write?", C8-6 would-but-doesn't
  ask "are there padding bytes that `operator==` cannot see?".
  For `error_message_batch` the second answer is "no" so its
  C8-6 check reduces to the `operator==` already present; for
  C8-2b the first question still applies.
- Concrete fixtures: first batch is 5 messages with `error_code`
  values 1..5 and bit-distinct other fields; second batch is the
  first 2 messages of the first batch (so the cache mismatch
  between the in-memory shrunken batch and the live cache slot is
  exclusively in `messages[2..4]`).

### C8-3 family.

9 tests (default + 8 contamination directions per
D-C8-FAMILY-LINEAR — staying linear, see the open question
below). **Suite placement:** C8-3a in `ShimCoreObservers` (matches
cycle-1 test-4 / C2-3a / C3-3a / C4-3a / C5-3a / C6-3a / C7-3a
precedent for post-make pre-poll observer state). C8-3b through
C8-3i in `ShimCorePoll` (matches the landed cycle-2 through
cycle-7 contamination tests; each runs a `poll()` cycle to drive
the sibling-schema arrival).
- C8-3a `ShimCoreObservers.ErrorMessageBatchNulloptBeforeAnyPoll`.
- C8-3b `ShimCorePoll.ErrorMessageBatchNulloptAfterBootAckIsAccepted`.
- C8-3c `ShimCorePoll.ErrorMessageBatchNulloptAfterClockStateIsAccepted`
  (`bytes_of`).
- C8-3d `ShimCorePoll.ErrorMessageBatchNulloptAfterPowerStateIsAccepted`
  (`bytes_of`).
- C8-3e `ShimCorePoll.ErrorMessageBatchNulloptAfterDsStateIsAccepted`
  (`bytes_of`).
- C8-3f `ShimCorePoll.ErrorMessageBatchNulloptAfterCanFrameBatchIsAccepted`
  (`active_prefix_bytes`).
- C8-3g `ShimCorePoll.ErrorMessageBatchNulloptAfterCanStatusIsAccepted`
  (`bytes_of`).
- C8-3h `ShimCorePoll.ErrorMessageBatchNulloptAfterNotifierStateIsAccepted`
  (`active_prefix_bytes`).
- C8-3i `ShimCorePoll.ErrorMessageBatchNulloptAfterNotifierAlarmBatchIsAccepted`
  (`active_prefix_bytes`).

### C8-4. `ShimCorePoll.RejectsErrorMessageBatchBeforeBootAckPreservingLane`

- D-C8-3. Inject 8-byte count=0 active prefix via
  `manually_fill_lane`. All eight cache slots remain `nullopt`;
  expected `expected_boot_ack_first`.

### C8-5. `ShimCorePoll.AllEightCachesAreIndependentlyMaintained`

- D-C8-1 — covers the 14 new cross-population directions.
- 15-step interleaved scenario (`2N-1` for N=8 slots). Steps 1–7
  populate existing slots in cycle order (clock, power, ds, cf,
  cs, ns, nab); step 8 introduces error_message_batch (catches
  ab→7 prior slots, 7 directions); steps 9–15 re-write each prior
  slot in cycle order (each catches the corresponding
  prior-arm-clobbers-ab direction, 7 directions). All
  `has_value()` prerequisites are `ASSERT_TRUE`. Variable-size
  payloads use `active_prefix_bytes`.
- **Step-8 comprehensive snapshot:** after step 8 (the
  error_message_batch arrival), the test must explicitly assert
  that **all seven prior slots remain at the values written in
  steps 1–7** (not just `has_value()`). This is what catches
  "ab dispatch arm clobbered slot N" bugs across the 7
  ab→prior directions. Mirrors C7-5's step-7 snapshot pattern
  (see `shim_core_test.cpp:2595–2608` for the cycle-7 precedent).
- **Step 9–15 re-write snapshots:** after each re-write, assert
  the rewritten slot has the new value AND that the
  error_message_batch slot is unchanged from step 8 — the latter
  is what catches "prior dispatch arm clobbered ab slot."
- **Concrete payload values per step:** to ensure that "shim
  applies the latest payload, not a stale one" is pinned, each
  re-write step (9–15) uses bit-distinct field values from the
  initial population step. E.g. step 1 sends
  `valid_clock_state(100)`, step 9 sends `valid_clock_state(900)`;
  the ASSERT after step 9 is `EXPECT_EQ(*shim.latest_clock_state(),
  valid_clock_state(900))`, not just `has_value()`.

### C8-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllEightSlots`

- **Correction-and-extension** of the landed C7-6, not a strict
  superset. See "Determinism notes" above for the full diff
  against C7-6's implementation. Replaces (drops) C7-6.
- Scenario: boot_ack@0, clock@1, power@2, ds@3, cf@4 (3-frame
  batch), cs@5, ns@6 (3-slot batch), nab@7 (3-event batch),
  emb@8 (3-message batch), clock@9.
- 16 `has_value()` gates with `ASSERT_TRUE` (8 slots × 2 setups,
  4 more than C7-6's landed 12).
- Full-struct `operator==` on all eight slots between two setups
  (8 checks total, 2 more than C7-6's landed 6).
- `memcmp` on `ds_state`, `can_frame_batch`, `notifier_state`,
  AND `notifier_alarm_batch` (4 padding-bearing schemas; 1 more
  than C7-6's landed 3). The notifier_alarm_batch memcmp pins
  the 4-byte interior `count → events` implicit padding that
  has not been pinned by any prior landed test. **No memcmp on
  `error_message_batch`** per D-C8-PADDING-FREE — `operator==` is
  byte-equivalent for this schema.
- 3-message error_message_batch fixture mirrors C8-1: every named
  field is bit-distinct across the three messages.

---

## Test rewrites

### C8-R10. delete test 10 (`RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext`)

The cycle-7 forward-reference resolves to outright deletion. Per
D-C8-PROBE-RETIRED:

1. Remove the `TEST(ShimCorePoll, RejectsUnsupportedPayloadSchema...)`
   function entirely from `tests/backend/shim/shim_core_test.cpp`.
2. Remove the now-unused `kErrorMessageBatchPrefixBytes` local
   constant and the `static_assert` on it (those were only used
   to size the reject probe).
3. Update `tests/backend/shim/TEST_PLAN.md` if it references test
   10's rationale (search-and-replace the cycle-7 limit comment).
4. The production-code dispatch arm's
   `unsupported_payload_schema` return statement stays — see
   D-C8-DEAD-BRANCH.

The session-counter-advance-through-reject contract is *not* lost:
it was pinned by tests 10 across cycles 2 (clock_state probe),
3 (power_state probe), 4 (ds_state probe), 5 (can_frame_batch
probe), 6 (can_status probe), 7 (notifier_state probe), and the
landed cycle-7 (notifier_alarm_batch probe). The structural
property "the session does not get desynchronized when the shim
rejects a schema" has been observed seven times across distinct
schemas. After cycle 8, no schema remains to construct a fresh
observation, so no test exists for it. This is the principled
trade-off (testing-against-a-property-with-no-witness) of closing
out the schema set, not a regression.

### C8-R6. drop C7-6 in favor of C8-6

**Correction-and-superset replacement, not strict-superset.** The
landed C7-6 (`shim_core_test.cpp:2694–2805`) is named
`RepeatedRunsProduceByteIdenticalAllSevenSlots` but covers only 6
slots: `notifier_alarm_batch` is absent from the scenario, the
`has_value()` gates, the `operator==` checks, and the `memcmp`
companion list. The cycle-7 plan's text called for 7-slot
coverage including `notifier_alarm_batch` with a 4th memcmp;
the landed implementation diverged from the plan for this one
test. Cycle 8's deletion of C7-6 in favor of C8-6 closes that
divergence — C8-6's name `AllEightSlots` is accurate for what
the test actually covers, and the 4 memcmp companions match
the 4 padding-bearing schemas. After C8-R6 lands, the
"AllSevenSlots" name no longer exists in the suite, so the
discrepancy is retired.

This R6 entry exists for traceability — implementers reading
the cycle-8 plan should not be surprised when they delete the
landed test that *appears* to cover 7 slots but in fact covers
6.

---

## Cross-test invariants

Same as cycles 4–7. `active_prefix_bytes` for variable-size,
`bytes_of` for fixed-size, `ASSERT_TRUE` on prerequisites,
memcmp companion on padding-bearing schemas (still 4 of those
after cycle 8 lands).

## What this plan does NOT cover

- C HAL ABI for error reporting (consumer-side `HAL_SendError`,
  `HAL_SendConsoleLine`).
- Validity-domain checks on error message fields (severity must
  be 0/1, NUL-termination of the fixed-size string buffers,
  consistency of `truncation_flags` with actual content
  truncation).
- Outbound `error_message_batch` (the shim emitting errors). Cycle
  N+ for outbound traffic past `boot`.
- Post-shutdown / lane_in_progress invariance — by-construction
  inheritance from cycles 1–7's structural coverage; no schema-
  specific repeat needed.
- The C HAL surfaces in general (`HAL_GetFPGATime`,
  `HAL_GetVinVoltage`, ...). Out of scope; future cycles.

---

## Implementation companion recommendations (non-test)

These are not test plan items but are worth landing in the same
cycle to prevent the documented size from rotting:

- Add `static_assert(sizeof(error_message_batch) == 18600)` to
  `src/backend/common/error_message.h` next to the existing
  `static_assert(sizeof(error_message) == 2324)` and the trivially-
  copyable / standard-layout asserts. This pins the documented
  total size (8-byte header + 8 × 2324 element bytes) so a future
  layout change can't silently shift it. Mirrors the per-element
  precedent already in the file.

---

## Open questions

**D-C8-FAMILY-LINEAR vs PARAM** — cycle 7 deferred parameterization
of the C-N-3 family to cycle 8 "or later." Cycle 8 stays linear at
9 tests with the rationale above. Reviewer push-back welcome if
they want the conversion to land at cycle 8 instead — the
parameterization would be over a `std::function<void(tier1_endpoint&,
std::uint64_t)> sender` (to abstract over `bytes_of` vs
`active_prefix_bytes`) and a probe-name string for diagnostic
output. The bug-class mechanics are unchanged either way.

**D-C8-DEAD-BRANCH retention.** The `unsupported_payload_schema`
arm in production is unreachable from valid traffic at cycle 8.
The plan keeps it as a defensive forward-compat structural guard.
Reviewer push-back welcome if they think the dead branch should
be deleted (with a comment explaining that any future schema must
also re-add the reject branch); the current plan errs on the side
of "no shortcuts" / non-negotiable #1.
