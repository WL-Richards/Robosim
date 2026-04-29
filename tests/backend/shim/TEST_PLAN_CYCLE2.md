# HAL shim core — cycle 2 test plan (power_state)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (four review rounds, 2026-04-29); cycle-2 production code
and tests landed and 20-shim-test green under both clang Debug and
GCC Debug + ASan + UBSan.

**Review history:**

- rev 1 → `not-ready`. Three blockers:
  - C2-3 conflated four distinct initial-state bug classes into one
    test body, violating the plan's own "one assert per behavior"
    rule, and including a `latest_clock_state().has_value() == true`
    control assertion that would cause the test to fail for an
    unrelated reason if `clock_state` caching ever broke.
  - C2-3's name (`PowerStateNulloptUntilFirstAcceptedPowerStateEnvelope`)
    implied a transition out of `nullopt` that the test body never
    exercised — that transition is C2-1's concern.
  - C2-R10's "Mechanical change" bullet covered the docstring
    update but did not enumerate the inline comment at the
    `core.send()` call site (currently lines 307–309 of the landed
    `shim_core_test.cpp`), which still references `power_state` in
    a way that would survive a literal find/replace on the schema
    alone.
  - Plus three reviewer notes that the rev was already correct on
    (drop of cycle-1 test 14 confirmed right; `valid_power_state`
    defaults adequate; C2-4 is not redundant with cycle-1 test 9).

- rev 2 → `not-ready`. All three rev-1 blockers closed; two new
  blockers surfaced on the rewritten material:
  - C2-3b and C2-3c specified prerequisite checks
    (`shim.is_connected()`, `shim.latest_clock_state().has_value()`)
    without pinning whether they should be `ASSERT_TRUE` or
    `EXPECT_TRUE`. With `EXPECT_TRUE`, a silent failure of the
    prerequisite (e.g. boot_ack not processed) would let the
    primary assertion `latest_power_state() == nullopt` pass
    trivially, producing a false green — the slot was never
    exercised, so of course it stays nullopt. The plan must
    pin `ASSERT_TRUE` and explain why.
  - The "Proposed tests" heading still said "(revision 1)" —
    stale text not updated when rev 2 was written.
  - Plus one rev-2 non-blocker recommendation: C2-3a was placed
    in `ShimCorePoll` but tests post-`make` pre-`poll()`
    observer state, the exact pattern of cycle-1's test 4
    (`IsConnectedFalseAndClockStateNulloptUntilBootAck`) which
    lives in `ShimCoreObservers`. Suite placement should match
    the cycle-1 precedent.

- rev 3 → `not-ready`. Both rev-2 blockers closed and the
  recommendation adopted, but three mechanical follow-ups
  surfaced:
  - The rev-3 summary bullet still said the heading was
    "corrected to 'Proposed tests (revision 2)'", which is
    stale text — the heading was actually corrected to
    revision 3, matching the document's overall version. Same
    class of stale-text issue that triggered a rev-2 blocker.
  - `poll().has_value() == true` in C2-3b and C2-3c was left
    unpinned between `ASSERT_TRUE` and `EXPECT_TRUE`. Not a
    false-green risk (the next-line `ASSERT_TRUE` on the
    accessor catches the failure path), but an implementer
    ambiguity worth removing.
  - The `has_value()` guards before dereferences in C2-6 were
    similarly unpinned. Here `ASSERT_TRUE` is the only correct
    choice — dereferencing a null `std::optional` is UB rather
    than a clean test failure.
  - Plus one implementer-checklist note: `valid_power_state` is
    added to `tests/backend/tier1/test_helpers.h`, which means
    the helper file gains a new `#include "power_state.h"`
    dependency it does not currently carry.

- rev 4 → this revision. All rev-3 follow-ups addressed:
  - **Stale "revision 2" text in the rev-3 summary bullet
    rewritten** to read "revision 3" unambiguously.
  - **`poll().has_value() == true` pinned to `ASSERT_TRUE`** in
    both C2-3b and C2-3c. Aligns the assertion-strength of the
    poll-success check with the immediately-following
    prerequisite check, so a failed `poll()` aborts the test
    cleanly at the same point as a failed prerequisite.
  - **`has_value()` guards in C2-6 pinned to `ASSERT_TRUE`**
    on both setups, immediately before each dereference. Avoids
    the UB-on-null-optional path entirely.
  - **Implementer-checklist note added to the "Test fixtures"
    section** that `tests/backend/tier1/test_helpers.h` gains
    `#include "power_state.h"` as part of this cycle.

- rev 3 (history retained) addressed all rev-2 issues:
  - **C2-3b and C2-3c require `ASSERT_TRUE`** for the
    prerequisite check. Inline language explains the no-shortcuts
    rationale (CLAUDE.md non-negotiable #1) and the false-green
    failure mode that `EXPECT_TRUE` would permit.
  - **C2-3a moved to `ShimCoreObservers`**, matching cycle-1
    test 4's suite. The plan now states "Suite placement"
    explicitly so the rationale is durable.
  - **Heading corrected** to "Proposed tests (revision 3)".
  - **C2-5's ellipsis** in the step-2 expected `valid_power_state(...)`
    replaced with the explicit-args form `valid_power_state(12.5f,
    2.0f, 6.8f)` so the expected value is unambiguous.

**Implements:** the second TDD cycle of the HAL shim core, named in
`.claude/skills/hal-shim.md` "Out / Cycle 2+" and in
`.claude/skills/layer-2-control-system-backend.md` "Still ahead, each
its own TDD cycle: HAL shim cycles 2+: additional inbound schemas
(power_state, …)".

This document is an **addendum** to the cycle-1 plan
(`tests/backend/shim/TEST_PLAN.md`, rev 2, status
`ready-to-implement` and now landed). It adds new contracts on top of
the cycle-1 surface, does not replace it. The cycle-1 14 tests must
continue to pass without behavior change; **one cycle-1 test (test 10)
is rewritten** to keep its meaning, see C2-R10 below.

---

## Why this cycle exists

Cycle 1 left the shim with exactly one supported inbound state schema
(`clock_state` under `tick_boundary`) and one terminal-flag schema
(`shutdown` / `boot_ack` under `none`). Every other inbound schema
under `tick_boundary` is dispatched through a single fall-through path
that returns `unsupported_payload_schema`. Cycle 2 promotes
`power_state` from "rejected" to "accepted with its own latest-wins
cache slot", mirroring `clock_state`'s shape.

The smallest cohesive feature is:

- **Accept** a `tick_boundary` envelope carrying `schema_id::power_state`
  with `payload_bytes == sizeof(power_state) == 12` and byte-copy the
  payload into a new `latest_power_state_` cache slot.
- **Expose** the new cache slot via
  `std::optional<power_state> latest_power_state() const` on
  `shim_core`.
- **Preserve** every existing cycle-1 contract verbatim — boot
  publish, boot_ack handshake, shutdown terminal path, lane-busy
  failure-atomicity, pre-boot-ack rejection, post-rejection counter
  recovery, determinism replay.

The post-rejection counter recovery contract (cycle-1 test 10) needs
one mechanical update: `power_state` is now the *valid* probe in
Step A, so the test's "still-unsupported probe" must shift to
`ds_state` (the next cycle-3 target). Otherwise the test would
silently no-op on the fall-through dispatch.

---

## Contract under test (additions only)

System under test is unchanged: `robosim::backend::shim::shim_core`.

### New public surface

`src/backend/shim/shim_core.h`:

- `std::optional<power_state> latest_power_state() const` —
  `nullopt` until at least one inbound `tick_boundary` envelope
  carrying `schema_id::power_state` has been accepted; the most
  recent `power_state` byte-copy thereafter. Latest-wins, no queue.
  Returned by value (mirrors cycle-1's `latest_clock_state()` shape:
  reference-to-`std::optional`).

  **Note:** cycle-1 returned
  `const std::optional<clock_state>& latest_clock_state() const`
  (reference). Cycle 2 mirrors that exact shape for symmetry, so
  the new accessor is
  `const std::optional<power_state>& latest_power_state() const`.

### New error kinds

**None.** The fall-through dispatch (`unsupported_payload_schema`,
`unsupported_envelope_kind`) is unchanged. No new
`shim_error_kind` is introduced.

### Internal additions

- New private member `std::optional<power_state> latest_power_state_;`
  on `shim_core`, sibling to `latest_clock_state_`.
- New `case schema_id::power_state:` arm in the
  `tick_boundary` dispatch in `shim_core::poll()`:
  byte-copy the 12 bytes into `latest_power_state_` and return
  success. Symmetric to the existing `clock_state` arm.

### Out of scope for this cycle (each gets its own future cycle)

- Inbound schemas other than `power_state` and the cycle-1 set
  (`clock_state`, `none` for boot_ack/shutdown). `ds_state` is
  cycle 3; `can_frame_batch` / `can_status` / `notifier_alarm_batch` /
  `error_message_batch` follow.
- Any outbound traffic past the construction-time `boot`.
- Threading.
- The C HAL ABI (`HAL_GetVinVoltage`, `HAL_GetBrownoutVoltage`, etc.)
  that will eventually read from `latest_power_state_`. Cycle 4+
  exports those as separate surface-files.
- Reset / reconnect.

---

## Decisions pinned

- **D-C2-1.** `power_state` and `clock_state` caches are
  **independent storage**. A `power_state` envelope must not mutate
  `latest_clock_state_`; a `clock_state` envelope must not mutate
  `latest_power_state_`. Pinned by C2-5 (cross-slot independence).

  **Why this is a decision, not an obvious code shape:** a refactor
  that "deduplicates" the two slots into a single
  `std::variant<clock_state, power_state>` (or a
  schema-tagged single slot, "we only need the latest *anything*")
  passes most other tests. C2-5 is the regression check.

- **D-C2-2.** Latest-wins, no queue. Same as cycle-1 D #7. A
  power_state at higher `sim_time_us` arriving after a power_state at
  lower `sim_time_us` overwrites; the converse case (out-of-order
  delivery) is impossible in cycle 2 because `protocol_session`
  already enforces sequence-order on the receive side, but the
  shim's cache itself is unconditionally latest-wins on its own
  receive ordering.

- **D-C2-3.** The dispatch table is **strictly post-session**. The
  protocol_session validates framing (magic, version, sender,
  sequence) and ordering (boot_ack-before-tick_boundary, shutdown
  terminal) before any byte of `power_state` payload is touched.
  The shim does not (and must not) short-circuit dispatch based on
  schema before the session's checks. Pinned by C2-4
  (pre-boot-ack power_state still surfaces
  `expected_boot_ack_first`, not `unsupported_payload_schema`).

- **D-C2-4.** **Cycle-1 test 10's probe schema migrates from
  `power_state` to `ds_state`.** This is a one-line mechanical
  rewrite, not a new contract. It is called out as a decision so
  reviewers can audit the rewrite is faithful to the original test's
  bug class. C2-R10 below specifies the exact change.

- **D-C2-5.** **No new shim_error_kind.** The fall-through dispatch
  remains the path for unsupported schemas under `tick_boundary`;
  cycle 2 just removes `power_state` from that fall-through. If a
  future cycle introduces an error class that doesn't fit the cycle-1
  taxonomy (e.g. malformed payload bytes that survived
  `protocol_session`), it gets a new kind and a triggering test
  there.

- **D-C2-6.** `latest_power_state_` is **value-initialized as
  `std::nullopt`** at the same point in `shim_core`'s default-member-
  init that `latest_clock_state_` is. Mirrors cycle-1 D #1 (no
  partial state). Pinned by C2-3a (default-init drift), with
  C2-3b and C2-3c pinning the related "no contamination by
  unrelated traffic" half of the contract.

---

## Cross-cutting assertion shape (extends cycle 1)

Cycle-2 tests live in the same `ShimCoreMake` / `ShimCorePoll` /
`ShimCoreDeterminism` GoogleTest suites as cycle-1 tests, in
`tests/backend/shim/shim_core_test.cpp`. Naming and the
"one assert per behavior" convention carry over.

Per cycle-1's pattern, all payload comparisons use either the
defaulted `power_state::operator==` (full-struct equality) or
`std::memcmp(&a, &b, sizeof(power_state)) == 0` over the 12 bytes.
Tests do **not** field-by-field compare `vin_v`, `vin_a`,
`brownout_voltage_v` — that would mask a "shim only copies the first
N bytes" bug.

`power_state::operator==` on `float` fields is bit-equal-as-`==` for
every value the tests produce (all finite, all explicit constants,
none arithmetic-derived). A NaN comparison would be UB-as-`==` but
no test path generates one.

---

## Test fixtures (additions only)

Cycle 2 adds one inline helper to
`tests/backend/tier1/test_helpers.h`:

```cpp
inline power_state valid_power_state(float vin_v = 12.5f,
                                     float vin_a = 2.0f,
                                     float brownout_voltage_v = 6.8f) {
  return power_state{vin_v, vin_a, brownout_voltage_v};
}
```

**Rationale for the default values:** all three are non-zero,
distinct, finite, and bit-distinct from `valid_clock_state()`'s
default fields (which are `std::uint64_t` / `std::uint8_t`). A
"shim writes 0s" bug fails the byte-equal assertion loudly. The
defaults are also realistic FRC operating-point values (12.5 V VIN,
2 A current, 6.8 V brownout threshold matches WPILib's default), so
the test reads as a plausible runtime scenario.

**Inline comment to add at the helper definition** (per rev-2
clarification): a one-line note that consumers compare via
`power_state::operator==` (defaulted, bit-equal-as-`==` on the
`float` fields for every value the cycle-2 tests produce), not
`std::memcmp`, because every test-produced value is an exact
constructor-supplied constant rather than an arithmetic result —
so there is no NaN, no rounding-residue concern, and no signed-
zero ambiguity to navigate. This pins the "why operator== is
appropriate here" reasoning at the helper rather than spreading it
across each call-site.

**Implementer checklist** (per rev-3 reviewer note): adding
`valid_power_state` to `test_helpers.h` requires adding
`#include "power_state.h"` to that header — the file does not
currently transitively include it. The include should land
alphabetically with the existing `#include "boot_descriptor.h"`
and `#include "clock_state.h"` block (placing it between
`clock_state.h` and `protocol_version.h`).

No new envelope helper is needed — `make_envelope` and
`manually_fill_lane` are schema-agnostic.

No new `bytes_of` overload is needed — the existing
`template <typename T> bytes_of(const T&)` works on any
trivially-copyable struct.

---

## Determinism notes

Single-threaded same-process tests. No new sources of nondeterminism
(no sleep, no real time, no thread spawn, no RNG). C2-6 explicitly
re-asserts byte-identical output across two runs of an
expanded scenario that includes both `clock_state` and `power_state`
updates.

---

## Proposed tests (revision 4)

### C2-1. `ShimCorePoll.AcceptsPowerStateAndCachesByteEqualValue`

- **Layer / contract:** D-C2-1, D-C2-2 — post-connect, a
  `tick_boundary`/`power_state` envelope updates a new cache slot
  with full byte fidelity.
- **Inputs:**
  - Shim connected via `make_connected_shim` (boot + boot_ack
    consumed; expected receive sequence is now 1).
  - Core endpoint sends `tick_boundary`/`power_state` at sequence 1
    with payload `valid_power_state(12.5f, 2.0f, 6.8f)` and
    `sim_time_us = 1'000`.
- **Expected outputs after `poll()`:**
  - `poll().has_value() == true`.
  - `shim.latest_power_state().has_value() == true`.
  - `*shim.latest_power_state() == valid_power_state(12.5f, 2.0f, 6.8f)`
    using the defaulted `operator==` — full-struct equality, not
    field-by-field.
  - `region.core_to_backend.state == empty`.
  - `shim.latest_clock_state() == std::nullopt` (D-C2-1: the
    clock-state slot is unaffected).
- **Bug class:** payload corruption between transport and cache;
  power_state cache populated for the wrong schema; shim writes
  only some fields; shim writes power_state into the clock_state
  slot.
- **Tolerance:** zero. No arithmetic on the float fields; copy is
  byte-exact.

### C2-2. `ShimCorePoll.LatestWinsForRepeatedPowerStateUpdates`

- **Layer / contract:** D-C2-2 — cache replacement keeps only the
  most recent `power_state`.
- **Inputs:** shim connected. Two `tick_boundary`/`power_state`
  envelopes applied via the core endpoint in order: first with
  `valid_power_state(12.0f, 1.0f, 6.8f)` at sequence 1, then with
  `valid_power_state(13.5f, 5.0f, 6.5f)` at sequence 2. A `poll()`
  is called between them and after the second.
- **Expected outputs:**
  - After first poll: `*shim.latest_power_state() ==
    valid_power_state(12.0f, 1.0f, 6.8f)`.
  - After second poll: `*shim.latest_power_state() ==
    valid_power_state(13.5f, 5.0f, 6.5f)`. All three fields
    differ between the two values, so a "cache only updates one
    field" bug fails.
- **Bug class:** cache holds the first value forever; cache
  arithmetic-merges two values; cache does field-wise update
  instead of struct-wise replace.

### C2-3a. `ShimCoreObservers.PowerStateNulloptBeforeAnyPoll`

- **Layer / contract:** D-C2-6 — `latest_power_state_` is value-
  initialized to `std::nullopt` at the same point in `shim_core`'s
  default-member-init that `latest_clock_state_` is.
- **Suite placement:** `ShimCoreObservers`, the same suite as
  cycle-1 test 4 `IsConnectedFalseAndClockStateNulloptUntilBootAck`.
  Like that test, C2-3a tests an observer's initial state without
  driving `poll()`. Placing it in `ShimCorePoll` (where the rest
  of cycle 2 lives) would be inconsistent with the cycle-1
  precedent for "constructor-time observer state."
- **Inputs:** shim post-`make` (boot already published); no
  `poll()` yet.
- **Expected outputs:**
  - `shim.latest_power_state() == std::nullopt`.
- **Bug class:** `latest_power_state_` default-init drifts to
  `power_state{}` instead of `nullopt`; member ordering puts
  `latest_power_state_` ahead of construction-time init.

### C2-3b. `ShimCorePoll.PowerStateNulloptAfterBootAckIsAccepted`

- **Layer / contract:** D-C2-6 — `boot_ack` (kind `boot_ack`,
  schema `none`) does not contaminate the power-state cache slot.
  This is a distinct bug class from C2-3a: a refactor that
  initializes `latest_power_state_` correctly but writes through
  it on every accepted envelope (regardless of schema) would
  pass C2-3a and fail this test.
- **Inputs:**
  - Shim post-`make`.
  - Core endpoint sends `boot_ack` (kind `boot_ack`, schema
    `none`, payload bytes 0, sequence 0).
  - Single `poll()`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`. **Required `ASSERT_TRUE`
    (not `EXPECT_TRUE`)** — pinned per rev-3 reviewer feedback.
    Aligns with the immediately-following prerequisite check:
    if `poll()` returned an error, there is no point continuing
    to the prerequisite or primary assertions; aborting here
    surfaces the transport/session diagnostic cleanly.
  - `ASSERT_TRUE(shim.is_connected())`. **Required `ASSERT_TRUE`
    (not `EXPECT_TRUE`)** — this is a prerequisite check, not
    the test's primary concern. If `boot_ack` processing
    silently failed, the primary assertion below would pass
    trivially (the slot was never exercised), producing a
    false green. `ASSERT_TRUE` aborts the test and surfaces the
    real failure; `EXPECT_TRUE` would let the false-green
    pattern through. This is a no-shortcuts (CLAUDE.md
    non-negotiable #1) constraint at the test layer.
  - `shim.latest_power_state() == std::nullopt`. (Primary
    assertion.)
- **Bug class:** `boot_ack` dispatch arm accidentally writes
  zero-init power_state into the cache; dispatch table has a
  fall-through that populates power on any accepted envelope.

### C2-3c. `ShimCorePoll.PowerStateNulloptAfterClockStateIsAccepted`

- **Layer / contract:** D-C2-1 — independent storage, viewed
  from the "the slot starts empty and stays empty under
  unrelated traffic" direction. (C2-5 viewed the same decision
  from the "the slot holds its value while the other slot
  updates" direction; both directions are needed.)
- **Inputs:**
  - Shim connected via `make_connected_shim` (boot + boot_ack
    consumed).
  - Core endpoint sends `tick_boundary`/`clock_state` at
    sequence 1 with `valid_clock_state(50'000)`.
  - Single `poll()`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`. **Required `ASSERT_TRUE`
    (not `EXPECT_TRUE`)** — pinned per rev-3 reviewer feedback,
    same rationale as C2-3b's poll-success check.
  - `ASSERT_TRUE(shim.latest_clock_state().has_value())`.
    **Required `ASSERT_TRUE` (not `EXPECT_TRUE`)** — same
    no-shortcuts rationale as C2-3b. If `clock_state`
    processing silently failed, no `clock_state` traffic ever
    exercised the cross-population risk and the primary
    assertion below would pass trivially. `ASSERT_TRUE` aborts
    and surfaces the real failure.
  - `shim.latest_power_state() == std::nullopt`. (Primary
    assertion.)
- **Bug class:** `clock_state` dispatch arm accidentally
  cross-populates the power slot (e.g. via a copy-paste error
  that writes the clock payload bytes into both slots); a
  shared union-typed cache that gets schema-tagged as
  `power_state` after a `clock_state` arrival.

### C2-4. `ShimCorePoll.RejectsPowerStateBeforeBootAckPreservingLane`

- **Layer / contract:** D-C2-3 — schema-specific dispatch is
  post-session. A pre-boot-ack `tick_boundary`/`power_state`
  surfaces `expected_boot_ack_first` from the session, **not**
  `unsupported_payload_schema` from the dispatch.
- **Setup (mirrors cycle-1 test 9, schema swapped):**
  - Fresh region; backend endpoint; shim via `make` (boot
    published).
  - Core peer is **not** used to send `boot_ack`; instead, the
    test uses `manually_fill_lane` to inject a well-framed
    envelope directly into `region.core_to_backend`:
    - `kind = tick_boundary`,
    - `payload_schema = power_state`,
    - `payload_bytes = sizeof(power_state) == 12`,
    - `sequence = 0`,
    - `sender = core_to_backend`,
    - `sim_time_us = 100'000`,
    - payload bytes = `bytes_of(valid_power_state())`.
  - **Why direct injection:** no peer can legitimately produce
    this state. A core-side `protocol_session` would refuse to
    build a `tick_boundary` before sending `boot_ack`. Same
    rationale as cycle-1 test 9.
- **Expected outputs:**
  - `poll()` returns
    `shim_error{shim_error_kind::receive_failed, transport_error =
    tier1_transport_error{kind = session_rejected_envelope,
    session_error = {kind = expected_boot_ack_first, ...}}, ...}`.
  - `region.core_to_backend.state == full` (lane preserved per
    tier1 failure-atomicity).
  - `region.core_to_backend.envelope` and `.payload_bytes` and
    `.payload` byte-equal the injected values.
  - `shim.is_connected() == false`.
  - `shim.latest_clock_state() == std::nullopt`.
  - `shim.latest_power_state() == std::nullopt`.
- **Bug class:** schema-specific dispatch leaks before the session
  check (would surface `unsupported_payload_schema` or, worse,
  silently populate the cache); shim accepts power_state ahead of
  the protocol agreement.

### C2-5. `ShimCorePoll.ClockAndPowerCachesAreIndependentlyMaintained`

- **Layer / contract:** D-C2-1 — independent cache storage.
- **Setup:**
  - Shim connected.
  - Sequence of three inbound envelopes, each followed by `poll()`:
    1. `tick_boundary`/`clock_state` at seq 1, payload
       `valid_clock_state(100'000)`.
    2. `tick_boundary`/`power_state` at seq 2, payload
       `valid_power_state(12.5f, 2.0f, 6.8f)`.
    3. `tick_boundary`/`clock_state` at seq 3, payload
       `valid_clock_state(200'000)`.
- **Expected outputs:**
  - After step 1: `latest_clock_state()->sim_time_us == 100'000`,
    `latest_power_state() == nullopt`.
  - After step 2: `latest_clock_state()->sim_time_us == 100'000`
    (unchanged — the power_state envelope must not have touched
    this slot), `*latest_power_state() == valid_power_state(12.5f,
    2.0f, 6.8f)` (full struct equality, explicit-args form to
    match step 2's input exactly).
  - After step 3: `latest_clock_state()->sim_time_us == 200'000`
    (updated), `*latest_power_state() == valid_power_state(12.5f,
    2.0f, 6.8f)` (unchanged — the clock_state envelope must not
    have touched this slot).
- **Bug class:** shared single slot tagged by schema ("we only
  need the latest *anything*"); union-typed cache where the second
  arrival clobbers the first; shim copies into the wrong slot
  based on a misread schema_id.

### C2-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalLatestPowerAndClockState`

- **Layer / contract:** non-negotiable #5. Replaces (rather than
  augments) cycle-1 test 14 — the existing
  `RepeatedRunsProduceByteIdenticalLatestClockState` covered
  clock-only; this version covers both slots in one scenario, and
  cycle-1 test 14 is **rewritten as part of this cycle** to extend
  the scenario. See "Test rewrites" below.
- **Inputs:** two independent setups (two regions, two shims, two
  core peers) running the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core sends `boot_ack` at seq 0; shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 1 with
    `valid_clock_state(50'000)`; shim `poll()`.
  - Core sends `tick_boundary`/`power_state` at seq 2 with
    `valid_power_state(12.5f, 2.0f, 6.8f)`; shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 3 with
    `valid_clock_state(100'000)`; shim `poll()`.
- **Expected outputs:**
  - For both setups (each gated independently):
    `ASSERT_TRUE(setup_a.latest_clock_state().has_value())`,
    `ASSERT_TRUE(setup_a.latest_power_state().has_value())`,
    `ASSERT_TRUE(setup_b.latest_clock_state().has_value())`,
    `ASSERT_TRUE(setup_b.latest_power_state().has_value())`.
    **Required `ASSERT_TRUE` (not `EXPECT_TRUE`)** — pinned per
    rev-3 reviewer feedback. The next assertions dereference
    these optionals; dereferencing a null `std::optional` is
    UB rather than a clean test failure, so the gates must
    abort on null, not soft-fail and continue.
  - `*setup_a.latest_clock_state() == *setup_b.latest_clock_state()`
    (full struct equality, post-`ASSERT_TRUE` dereference).
  - `*setup_a.latest_power_state() == *setup_b.latest_power_state()`
    (full struct equality, post-`ASSERT_TRUE` dereference).
  - `EXPECT_TRUE(setup_a.is_connected())`,
    `EXPECT_TRUE(setup_b.is_connected())`. Structural soundness
    check on the two-run setup; safe with `EXPECT_TRUE` because
    no dereference follows.
- **Bug class:** any seam-level nondeterminism in the new
  power_state path (uninitialized stack bytes in the float
  fields' padding — `power_state` is 12 bytes with no padding,
  so this *should* be impossible, but the test pins it loudly);
  ABI mismatch on the power_state struct between the two compile
  configurations the CI matrix runs.

---

## Test rewrites (cycle-1 plan touched)

### C2-R10. `ShimCorePoll.RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext` (rewrite)

- **Source of change:** D-C2-4. Cycle 2 makes `power_state`
  supported, so cycle-1 test 10's Step A would now succeed and
  silently no-op the rejection contract.
- **Mechanical changes to `tests/backend/shim/shim_core_test.cpp`
  test 10** (all three are required; the inline comment update
  is called out explicitly because a literal find/replace on the
  schema_id alone would leave it stale):
  1. **Send line.** Change `schema_id::power_state` →
     `schema_id::ds_state`. The payload changes from
     `bytes_of(power_state{})` to `bytes_of(ds_state{})` (the
     helper template works on any trivially-copyable struct;
     the dispatch path rejects on schema, not on payload
     contents, so any framing-valid payload of the right size
     is fine). The local variable `pstate` becomes `dstate`.
  2. **Inline comment at the call site** (currently lines
     307–309 in the landed file):
     ```
     // Step A — send a valid tick_boundary/power_state. Framing is valid
     // (kPerKindAllowedSchemas allows power_state under tick_boundary),
     // so the protocol_session accepts and advances next-expected to 2.
     // The shim's post-session dispatch refuses the schema.
     ```
     becomes:
     ```
     // Step A — send a valid tick_boundary/ds_state. Framing is valid
     // (kPerKindAllowedSchemas allows ds_state under tick_boundary),
     // so the protocol_session accepts and advances next-expected to 2.
     // The shim's post-session dispatch refuses the schema. Cycle 3
     // will land ds_state and migrate this probe to the next
     // still-unsupported schema.
     ```
  3. **Block-comment header above the `TEST(...)` line** (the
     "Test 10:" header at lines 296–301 of the landed file):
     update the comment-prose to match the new probe schema.
     Specifically the phrase "Schemas other than `clock_state`,
     `none`" must update to "Schemas other than `clock_state`,
     `power_state`, `none`" so the per-cycle cumulative
     supported-schema set is accurate.
- **Bug class (unchanged):** shim silently dropping schemas it
  can't yet handle (would mask cycle-3+ divergence); shim
  corrupting the session receive counter on dispatch reject (the
  Step B `clock_state` send at the next sequence would fail).
- **Source-of-truth note for future cycles:** each cycle that
  adds a schema must rewrite this test to migrate the probe one
  schema along the cycle-3..cycle-N order
  (`ds_state` → `can_frame_batch` → `can_status` → … →
  `error_message_batch`). Cycle N=8 (after every schema is
  supported) deletes the test entirely; it gets replaced by a
  positive test of the last schema added. That decision lives
  in cycle 8's plan, not here.

### C2-R14. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalLatestClockState`

This cycle-1 determinism test is **dropped from the file** and
replaced by C2-6 (which is a strict superset: it asserts the
same byte-equality on `latest_clock_state` *and* on
`latest_power_state`, in a scenario that interleaves both schemas).

Dropping rather than keeping both is per `code-style.md`'s "no
half-finished implementations" — keeping the cycle-1 single-slot
determinism test alongside the cycle-2 two-slot version is dead
code: any failure mode the old test catches, the new one catches
strictly harder.

---

## Cross-test invariants (additions)

- All cycle-2 tests construct one shim per test; no shared state.
- All `sim_time_us` and `vin_v` / `vin_a` / `brownout_voltage_v`
  values are explicit constants. No wall-clock, no derived-from-
  current-state.
- All payload comparisons use full-struct `operator==` or
  `std::memcmp` over `sizeof(power_state)` bytes.
- C2-3a/b/c each pin a single bug class around the power-slot
  observer (default-init drift, boot_ack contamination,
  clock_state contamination respectively). Where C2-3b and
  C2-3c reference observers on the *other* slot, the
  references are prerequisite checks, not the test's primary
  concern; the primary concern is always
  `latest_power_state() == std::nullopt`.
- C2-5 and C2-6 explicitly assert on **both** cache slots at
  the relevant steps. A test that asserts on only one slot
  misses the cross-population bug class for that step.

## Stub disclosure (additions)

- The "core" peer is real `tier1_endpoint` + real
  `protocol_session` (no fakes), same as cycle 1.
- C2-4 reaches behind the endpoint's send path via
  `manually_fill_lane`, same as cycle-1 test 9. The injection
  rationale is identical: no real peer can produce a pre-boot-ack
  `tick_boundary`/`power_state`.
- No new mocks, no new stubs. The SUT (`shim_core`) is never
  mocked.

---

## What this plan does NOT cover

- **Outbound power_state.** The shim never sends `power_state` in
  cycle 2 (the shim is purely consumer). Outbound past boot is
  cycle 3+.
- **C HAL ABI surfaces** (`HAL_GetVinVoltage`,
  `HAL_GetBrownoutVoltage`, etc.) that read from
  `latest_power_state_`. Cycle 4+.
- **Validity-domain checks on `power_state` field values** (e.g.
  rejecting a NaN `vin_v`). The shim is a byte-copy cache; field-
  domain validation is the producer's responsibility (sim core)
  and the consumer's responsibility (the future C HAL surfaces),
  not the shim's. If a future cycle introduces consumer-side
  domain checks, those land with their own positive tests.
- **Brownout-driven shutdown semantics.** The shim caches; the
  consumer decides what to do with `brownout_voltage_v`.
- **Post-shutdown `latest_power_state()` invariance.** The
  cycle-1 `shutdown_already_observed` short-circuit short-
  circuits before any dispatch, so the post-shutdown path is
  schema-agnostic — cycle-1 test 12 already pins that
  `latest_clock_state()` is unchanged after a post-shutdown
  poll, and the same code path covers `latest_power_state()`
  by construction (no per-schema branch exists post-shutdown).
  Adding a new test for the power-slot variant would verify
  the same line of code from a second angle. Out of scope for
  cycle 2 per `code-style.md`'s "no half-finished
  implementations" — coverage that exists already by
  construction does not need duplicate exercise. If a future
  cycle changes the shutdown short-circuit to be schema-aware
  (e.g. allowing a final `power_state` snapshot through after
  shutdown for graceful-brownout reasons), that change lands
  with its own positive test.
- **Post-`lane_in_progress` `latest_power_state()` invariance.**
  Same reasoning as above against cycle-1 test 13: the
  `lane_in_progress` path returns before any dispatch, so the
  invariance is by construction. Out of scope for cycle 2.

---

## Open questions

None for cycle 2. The shape mirrors cycle 1 closely enough that the
design space has already been committed. If reviewer disagrees on a
specific decision (D-C2-1 .. D-C2-6) the discussion reopens.
