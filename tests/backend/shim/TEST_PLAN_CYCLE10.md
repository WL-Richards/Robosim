# HAL shim core — cycle 10 test plan (outbound notifier_state)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, single review round). Round-1 verdict
`ready-to-implement` with one required documentation
clarification (C10-1's memcmp range coverage explicit) plus one
non-blocking improvement (C10-2 mirror that language) — both
addressed in-place. Cycle-10 production code and 6 new tests
landed and 107-shim-test green (full project ctest 422/422
green) under both clang Debug and GCC Debug + ASan + UBSan.
The D-C10-EXTRACT-ACTIVE-PREFIX refactor was verified by the
existing 101 cycle-1-through-9 tests staying green alongside
the 6 new cycle-10 tests. All three open questions resolved by
the reviewer:
- **OQ-C10-PER-METHOD-MIRRORS:** trim ENDORSED. 6 tests is
  correct; the 4 skipped mirrors test cross-cutting contracts
  already verified by cycle 9. The reviewer's note: "A future
  reviewer should not revert to 10-test coverage without a
  concrete new bug class that the trim fails to catch."
- **OQ-C10-HELPER-LOCATION:** per-schema-header inline placement
  ENDORSED. The aggregator alternative would be premature
  abstraction.
- **OQ-C10-NOTIFIER-ALARM-OVERLOAD:** keep test-only ENDORSED.
  No production caller for `notifier_alarm_batch` outbound.

**Implements:** the tenth TDD cycle of the HAL shim core, and the
**second outbound-past-boot** cycle. Cycles 1–9 are landed and 101-
shim-test green; full project ctest 416/416 green under both clang
Debug and GCC Debug + ASan + UBSan. Cycle 10 adds the second
outbound-meaningful schema's typed publish method
(`send_notifier_state`) and **cashes in D-C9-NO-HELPER's "lift the
helper when the second call site lands" trigger**: extracts
`active_prefix_bytes` overloads into production code (in the
respective schema headers) so both `send_can_frame_batch` (cycle 9,
currently inlined) and `send_notifier_state` (this cycle) call the
same primitive.

---

## Why this cycle exists

Cycle 9 wired the first outbound surface but explicitly left two
loose ends:

1. **The other outbound-meaningful schemas.** Notifier registrations
   (`notifier_state`) and `HAL_SendError`-bound logs
   (`error_message_batch`) are the other two semantically-meaningful
   outbound schemas per `kPerKindAllowedSchemas` (the five sim-
   authoritative schemas — `clock_state`, `power_state`, `ds_state`,
   `can_status`, `notifier_alarm_batch` — are intentionally absent
   from the outbound API per D-C9-TYPED-OUTBOUND). Cycle 10 lands
   `send_notifier_state`; cycle 11 will land
   `send_error_message_batch`.

2. **The D-C9-NO-HELPER deferral.** Cycle 9's `send_can_frame_batch`
   inlined the active-prefix size computation
   (`offsetof(B, elements) + count * sizeof(E)`), with the explicit
   contract: *"when a second outbound variable-size schema lands, the
   duplication between the inlined production path and the test-side
   `tier1::helpers::active_prefix_bytes` is the trigger for lifting a
   shared helper into production code."* That trigger fires now.

`notifier_state` is the natural second pick because it is the next
variable-size schema in `schema_id` enum order (4 → 6 → 8 — `5` is
`can_status` which is fixed-size and not outbound-meaningful), it
exercises a meaningfully different layout (8-byte header vs cycle 9's
4-byte header; 88-byte element vs cycle 9's 20-byte element; 32-slot
capacity vs cycle 9's 64-frame capacity; **132 implicit padding
bytes** total — 4 interior `count → slots` pad + 4 trailing per
`notifier_slot` × 32 — vs cycle 9's 192 trailing-pad bytes inside
unused frame slots), and it forces the helper extraction so cycle 11
can follow the same pattern without re-litigating the design.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

```cpp
// Publishes `state` as a tick_boundary envelope into the shim's
// outbound backend_to_core lane. Sends exactly the active prefix
// (offsetof(notifier_state, slots) + state.count * sizeof(notifier_slot))
// — NOT sizeof(notifier_state) — so the validator's count-vs-length
// contract holds. Returns send_failed wrapping the underlying tier1
// transport error if the lane is busy / in-progress, or
// shutdown_already_observed once is_shutting_down() is true.
[[nodiscard]] std::expected<void, shim_error> send_notifier_state(
    const notifier_state& state, std::uint64_t sim_time_us);
```

Header note: the per-schema offset is **8** (not 4) because
`notifier_slot` has `alignof == 8` from its `std::uint64_t
trigger_time_us` field. This is already pinned by
`offsetof(notifier_state, slots) == 8` static_assertions in cycle-6's
test plan and the C8-6 determinism replay; cycle 10's outbound path
must respect the same offset.

`src/backend/common/can_frame.h` (extracted from `shim_core.cpp`'s
inlined path per D-C10-EXTRACT-ACTIVE-PREFIX below):

```cpp
inline std::span<const std::uint8_t> active_prefix_bytes(
    const can_frame_batch& batch) {
  const std::size_t active_size =
      offsetof(can_frame_batch, frames) +
      static_cast<std::size_t>(batch.count) * sizeof(can_frame);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}
```

`src/backend/common/notifier_state.h` (new, mirroring shape):

```cpp
inline std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_state& state) {
  const std::size_t active_size =
      offsetof(notifier_state, slots) +
      static_cast<std::size_t>(state.count) * sizeof(notifier_slot);
  return {reinterpret_cast<const std::uint8_t*>(&state), active_size};
}
```

(No production helper for `notifier_alarm_batch` since it's sim-
authoritative and never published outbound by the shim. The test-only
overload in `tests/backend/tier1/test_helpers.h` stays. Same for
`error_message_batch` until cycle 11 lands its outbound API.)

### Internal additions

- New `send_notifier_state` definition in `shim_core.cpp` calls
  `endpoint_.send(envelope_kind::tick_boundary,
  schema_id::notifier_state, backend::active_prefix_bytes(state),
  sim_time_us)` after the same `shutdown_observed_` short-circuit
  pattern as `send_can_frame_batch`.
- `send_can_frame_batch` is **refactored** to call
  `backend::active_prefix_bytes(batch)` instead of the inlined
  computation — D-C10-EXTRACT-ACTIVE-PREFIX. The function body
  shrinks from 12 lines to 9.
- `tests/backend/tier1/test_helpers.h`: the existing
  `active_prefix_bytes(can_frame_batch)` (line 155) and
  `active_prefix_bytes(notifier_state)` (line 219) overloads are
  **deleted**. Test code that calls them resolves to the production
  `backend::active_prefix_bytes` via a `using backend::active_prefix_bytes;`
  added near the top of `shim_core_test.cpp` and the existing tier1
  tests. The `notifier_alarm_batch` (line 252) and
  `error_message_batch` (line 305) overloads stay in test_helpers
  for now (no production caller; they're consumed only by inbound
  test injection).

### Out of scope

- `send_error_message_batch` — cycle 11.
- The five inbound-only schemas as outbound API (per D-C9-TYPED-
  OUTBOUND, intentionally inaccessible).
- `on_demand_request` / `on_demand_reply` envelopes carrying
  `notifier_state`. Future on-demand cycle.
- Shim-initiated `shutdown` envelope. Future cycle.
- The C HAL ABI `HAL_Notifier*` surface. The C HAL Notifier cycle
  will own the per-registration buffering and per-tick-flush
  discipline that drives `send_notifier_state`; cycle 10 stops at
  the C++ shim API.
- Outbound `count > kMaxNotifiers` validation. Same upstream-
  validator coverage and same C-HAL-cycle-owns-the-clamp deferral as
  D-C9 documented for `can_frame_batch` `count > kMaxCanFramesPerBatch`.
- Threading. Single-threaded; caller drives both `poll()` and
  `send_*` methods.

---

## Decisions pinned

- **D-C10-EXTRACT-ACTIVE-PREFIX.** Cycle 9's D-C9-NO-HELPER deferral
  cashes in. The active-prefix size computation moves from inlined-
  in-`shim_core.cpp` to free overloaded inline functions in the
  respective schema headers (`can_frame.h` gets the existing
  computation; `notifier_state.h` gets a new one). Both production
  send methods (`send_can_frame_batch` after refactor;
  `send_notifier_state` from day 1) call them. The test-side
  `tier1::helpers::active_prefix_bytes` overloads for these two
  schemas are **deleted** to avoid duplication; tests resolve to the
  production version via a `using backend::active_prefix_bytes;`.
  Pinned by:
  - The cycle-9 test suite (101 tests) staying green under the
    refactor — proves `send_can_frame_batch`'s wire format is
    unchanged after the helper extraction.
  - C10-1 (the production helper produces correct active-prefix
    bytes for `notifier_state`).
- **D-C10-TYPED-OUTBOUND-INHERITS.** D-C9-TYPED-OUTBOUND applies
  unchanged to `send_notifier_state`: it is a typed-per-schema method
  taking a `const notifier_state&`; no generic
  `send_tick_boundary(schema_id, ...)` is introduced; the four
  remaining inbound-only schemas are still absent from the outbound
  API surface.
- **D-C10-ACTIVE-PREFIX-OUT-INHERITS.** D-C9-ACTIVE-PREFIX-OUT
  applies per-schema. `send_notifier_state` writes exactly
  `offsetof(notifier_state, slots) + state.count *
  sizeof(notifier_slot)` bytes onto the wire. The `offsetof` is **8**
  (not 4) because `notifier_slot` has `alignof == 8`. A "send
  computes the offset assuming alignof 4 like can_frame_batch does"
  bug would deliver `4 + count*88` bytes instead of `8 + count*88`,
  and the validator's `expected_variable_payload_size` would reject
  on `payload_size_mismatch`. Pinned by C10-1 and C10-1b.
- **D-C10-EMPTY-BATCH-OUT-INHERITS.** D-C9-EMPTY-BATCH-OUT applies:
  count=0 is a legal publish; the active prefix is the 8-byte header
  (count word + 4-byte interior pad). Pinned by C10-1b.
- **D-C10-SEQUENCE-ADVANCE-INHERITS.** D-C9-SEQUENCE-ADVANCE
  applies. Pinned by C10-2 (positive) and C10-3 (failure preserves).
- **D-C10-SHUTDOWN-TERMINAL-INHERITS.** D-C9-SHUTDOWN-TERMINAL
  applies — but **per-method**: each typed `send_*` method must
  carry its own `if (shutdown_observed_) return shutdown_already_observed`
  short-circuit. A "implementer added the new method but forgot the
  short-circuit" bug is a real risk on each new method, not
  inheritable from the cycle-9 `send_can_frame_batch`. Pinned
  per-method by C10-5.
- **D-C10-WRAPPED-SEND-ERROR-INHERITS.** D-C9-WRAPPED-SEND-ERROR
  applies: `send_notifier_state` consumes the same
  `wrap_send_error` lambda (which already carries the generic
  message after the cycle-9 refactor).
- **D-C10-INBOUND-INDEPENDENCE-INHERITS.** D-C9-INBOUND-INDEPENDENCE
  applies. Cycle 10 does NOT add a per-method test for this because
  the outbound code path operates on the outbound lane and the
  session's send-side counter; the implementation reuses the
  cycle-9 pattern verbatim. The cycle-9 C9-6 test exercised the
  general principle. Reviewer push-back welcome if they want a
  per-method mirror — see open question OQ-C10-PER-METHOD-MIRRORS.
- **D-C10-NO-CONNECT-GATE-INHERITS.** D-C9-NO-CONNECT-GATE applies
  via the same protocol-session contract (`validate_send_order`
  only requires `has_sent_boot_` for backend-side non-boot sends);
  no per-method test added for the same reason as D-C10-INBOUND-
  INDEPENDENCE-INHERITS. See OQ-C10-PER-METHOD-MIRRORS.
- **D-C10-RECEIVE-COUNTER-INDEPENDENCE-INHERITS.** D-C9-receive-
  counter-non-perturbation (C9-8) applies — outbound mutates only
  `next_sequence_to_send_`, never `next_expected_receive_sequence_`.
  No per-method test; same reason as above.
- **D-C10-NEW-SUITE-PLACEMENT.** Cycle-10 outbound tests live in
  the existing `ShimCoreSend` suite alongside cycle 9's tests
  (not a new `ShimCoreSendNotifier` sub-suite). The determinism
  test lives in `ShimCoreDeterminism` alongside C9-7 and C8-6.

---

## Test fixtures

Cycle 10 adds NO new helpers to `tests/backend/tier1/test_helpers.h`.
The existing `valid_notifier_slot` (line 181) and
`valid_notifier_state` (line 205) cover the fixture surface.

The `active_prefix_bytes(notifier_state)` overload in
`test_helpers.h` (line 219) is **deleted** per D-C10-EXTRACT-ACTIVE-
PREFIX; tests call `backend::active_prefix_bytes(state)` instead.

---

## Determinism notes

`notifier_state`'s padding profile is the most demanding of any
outbound schema landing in v0:

- 4-byte interior pad between `count` (offset 0–3) and `slots`
  (offset 8) — pinned by `offsetof(notifier_state, slots) == 8`.
- 4-byte trailing pad per `notifier_slot` (sum of named fields = 84;
  `sizeof(notifier_slot) == 88`) × 32 slots = 128 bytes.
- Total = 132 implicit padding bytes inside `sizeof(notifier_state)
  == 2824`.

Of these, only the **4-byte interior pad** is within the active
prefix when `count >= 1`; the per-slot trailing pads inside
`slots[0..count-1]` are also within the active prefix (each
`notifier_slot` carries its own 4 bytes of trailing pad inside its
88-byte size). For `count = N`, the bytes within the active prefix
that are implicit padding are `4 (interior) + N * 4 (per-slot
trailing) = 4 + 4N` bytes.

C10-7's `std::memcmp` companion over the active prefix pins these
bytes byte-equal across two runs. The source-side
`notifier_state{}` and `notifier_slot{}` zero-init that
`valid_notifier_state` uses ensures these padding bytes are zero on
both runs. A "shim picks up uninit stack bytes when copying the
batch" bug would diverge between the two setups.

(Bytes past the active prefix — `slots[count..31]` and their per-
slot pads, plus the 4-byte interior pad if `count == 0` is treated
specially — are not transmitted, so they cannot affect wire
determinism.)

---

## Proposed tests (revision 1)

### C10-1. `ShimCoreSend.PublishesNotifierStateAsTickBoundaryWithActivePrefixWireBytes`

- **Layer / contract:** `send_notifier_state` → tier1 wire,
  D-C10-ACTIVE-PREFIX-OUT-INHERITS, D-C10-SEQUENCE-ADVANCE-INHERITS
  positive (first send), D-C10-EXTRACT-ACTIVE-PREFIX (the production
  helper produces correct bytes).
- **Setup:**
  - Fresh region; backend endpoint; shim via
    `make_connected_shim(region, core)` (boot+boot_ack handshake
    completes).
  - Construct a 3-slot batch:
    `valid_notifier_state(std::array{
        valid_notifier_slot(/*trigger_time_us=*/1'000'000,
                            /*handle=*/10, /*alarm_active=*/1,
                            /*canceled=*/0, /*name=*/"alpha"),
        valid_notifier_slot(2'000'000, 20, 0, 1, "beta"),
        valid_notifier_slot(3'000'000, 30, 1, 1, "gamma")})`.
- **Action:** `auto sent = shim.send_notifier_state(batch, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - The core peer's `try_receive()` returns a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::notifier_state`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1` (boot took 0).
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 8 + 3 * 88 == 272` (the active-prefix
      size; 8-byte header per `offsetof(notifier_state, slots) == 8`).
    - `payload.size() == 272`.
    - `std::memcmp(payload.data(), &batch, 272) == 0`. **Why memcmp
      over the active prefix specifically:** sending `sizeof(batch)`
      = 2824 bytes would also satisfy "payload contains the active
      prefix at the front" but would fail the validator's count-vs-
      length contract. The exact-272 byte size check is what catches
      the "shim sends sizeof regardless of count" bug AND the
      "shim computes offset-4 instead of offset-8" bug (D-C10-ACTIVE-
      PREFIX-OUT-INHERITS).
    - **What the 272-byte memcmp range covers (load-bearing for the
      cycle's headline bug class):** the range `[0, 272)` includes
      the 4-byte interior implicit pad at offsets 4–7 (the `count →
      slots` gap pinned by `offsetof(notifier_state, slots) == 8`)
      and the 4-byte trailing implicit pad inside each of the three
      active `notifier_slot` entries (at offsets 4..7 within each
      88-byte slot's tail). That's `4 + 3*4 = 16` implicit padding
      bytes within the active prefix that the memcmp covers. This
      is the specific contract gap relative to cycle 9's C9-1:
      `can_frame_batch` has only per-frame trailing pad (3 bytes per
      frame, all within the active prefix) and no interior pad;
      `notifier_state` adds the interior pad. A "shim reads from a
      `notifier_state` source that was not zero-initialized" bug
      would leak uninit stack bytes into offsets 4–7 (and into the
      per-slot trailing 4 bytes). The 272-byte memcmp catches this;
      `valid_notifier_state`'s `notifier_state{}` zero-init is what
      makes the comparison deterministic on the source side.
- **Bug class:** missing-publish; wrong envelope kind; wrong schema
  id; **header-offset-4-vs-8 confusion** (the headline new-schema
  bug class — would deliver 268 bytes instead of 272 and the
  validator would reject as `payload_size_mismatch`); active-prefix
  vs sizeof confusion; sequence not starting at 1; sim_time
  hardcoded; the production helper extraction broken.

### C10-1b. `ShimCoreSend.PublishesEmptyNotifierStateAsHeaderOnlyTickBoundary`

- **Layer / contract:** D-C10-EMPTY-BATCH-OUT-INHERITS,
  D-C10-ACTIVE-PREFIX-OUT-INHERITS boundary (count=0); the 8-byte
  header (count word + 4-byte interior pad) lands on the wire
  byte-for-byte.
- **Setup:** identical to C10-1 through the boot+boot_ack handshake.
  Construct an empty batch via `valid_notifier_state()` (default-arg
  empty span).
- **Action:** `shim.send_notifier_state(empty_state, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - Core peer receives a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::notifier_state`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1`.
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 8` (the header-only minimum).
    - `payload.size() == 8`.
  - `std::memcmp(payload.data(), &empty_state, 8) == 0` — the count
    word PLUS the 4-byte interior pad (which is byte-zero from the
    `notifier_state{}` zero-init in `valid_notifier_state`) lands on
    the wire byte-for-byte.
- **Bug class:** "shim refuses count=0" (semantically valid heartbeat;
  the validator allows it); shim sends only the 4-byte count word
  and forgets the interior pad (would deliver 4 bytes; validator
  rejects); shim sends `sizeof(notifier_state) = 2824` bytes for a
  count=0 batch (validator rejects).

### C10-2. `ShimCoreSend.SecondNotifierStateSendAdvancesOutboundSequenceToTwo`

- **Layer / contract:** D-C10-SEQUENCE-ADVANCE-INHERITS positive arm,
  per-method verification.
- **Setup:** identical to C10-1. After the first
  `send_notifier_state` succeeds, the core peer drains the published
  envelope to free the outbound lane, and the shim sends a second
  batch with bit-distinct field values.
- **Concrete payload values:**
  - First batch: 2 slots, handles `{11, 22}`, names `{"first0",
    "first1"}`, trigger_time_us `{100, 200}`.
  - Second batch: 4 slots (different count from first so a "shim
    treats sequence advance as count-based" bug fails), handles
    `{33, 44, 55, 66}`, names `{"second0", "second1", "second2",
    "second3"}`, trigger_time_us `{300, 400, 500, 600}`.
- **Action:**
  - `shim.send_notifier_state(first, 100'000);`
  - `core.try_receive();` (drain)
  - `shim.send_notifier_state(second, 200'000);`
- **Expected outputs:**
  - Both sends `has_value() == true`.
  - First received envelope: `sequence == 1`, `payload_bytes == 8 + 2*88 == 184`.
  - Second received envelope: `sequence == 2`, `payload_bytes == 8 + 4*88 == 360`, `sim_time_us == 200'000`.
  - Full active-prefix `std::memcmp` byte-equality on both envelopes' payloads against the source batches. **Same active-prefix coverage shape as C10-1's memcmp** (the 4-byte interior pad at offsets 4–7 plus the per-slot trailing 4 bytes inside each active slot are within both ranges: 12 bytes of implicit padding for the 184-byte first send, 20 bytes for the 360-byte second).
- **Bug class:** sequence counter doesn't advance; advances by `count`
  instead of 1; resets between sends; the helper-extracted
  `active_prefix_bytes` returns the wrong size for one of the two
  counts (would fail size assertion).

### C10-3. `ShimCoreSend.LaneBusyNotifierStateSendIsRejectedAndPreservesSequenceCounter`

- **Layer / contract:** D-C10-SEQUENCE-ADVANCE-INHERITS negative arm
  (`lane_busy` for the new method); D-C10-WRAPPED-SEND-ERROR-INHERITS.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make` (boot envelope
    sent into `region.backend_to_core`, lane state `full`).
  - Core peer does NOT drain the boot envelope.
  - Capture the boot envelope and payload bytes for non-clobber
    assertion.
- **Action:** `auto first_attempt = shim.send_notifier_state(valid_notifier_state(std::array{valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")}), /*sim_time_us=*/100'000);`
- **Expected outputs:**
  - `first_attempt` returns
    `shim_error{shim_error_kind::send_failed,
                transport_error = tier1_transport_error{kind = lane_busy, ...},
                offending_field_name = "lane",
                ...}`.
  - The lane state is still `full` (boot envelope unchanged); the
    lane payload bytes still byte-equal the captured boot bytes.
  - Now drain the boot via the core peer. Send the same batch again:
    succeeds at `sequence == 1` (NOT 2). **Recovery proof for
    D-C10-SEQUENCE-ADVANCE-INHERITS negative arm:** if the failed
    first attempt had advanced the counter, the second would publish
    at sequence 2 and the core peer's session would reject as
    `sequence_mismatch`.
- **Bug class:** failed sends burn sequence numbers (fatal — wedges
  the channel); failed sends silently clobber the lane (would
  corrupt the boot envelope); failed sends return success; the new
  method's lane-busy path differs from `send_can_frame_batch`'s.

### C10-5. `ShimCoreSend.PostShutdownNotifierStateSendIsRejectedWithoutTouchingLane`

- **Layer / contract:** D-C10-SHUTDOWN-TERMINAL-INHERITS — **per-
  method verification** (the only D-C9-derived contract that needs
  a per-method test, because each typed send method carries its
  own `shutdown_observed_` short-circuit and an "implementer
  forgot the short-circuit on the new method" bug is the central
  risk here).
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`.
  - Core peer drains boot via `core.try_receive()`.
  - Core peer sends `boot_ack` at sequence 0.
  - Shim polls (`is_connected() == true`).
  - Core peer sends a `shutdown` envelope at sequence 1.
  - Shim polls (`is_shutting_down() == true`).
- **Pre-action assertions:**
  - `ASSERT_EQ(region.backend_to_core.state.load(), empty)` —
    outbound lane is empty (boot was drained; the shim has not sent
    anything since). Mirror of C9-5's precondition.
  - `ASSERT_TRUE(shim.is_shutting_down())`.
- **Action:** `auto sent = shim.send_notifier_state(valid_notifier_state(std::array{valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")}), 250'000);`
- **Expected outputs:**
  - `sent` returns
    `shim_error{shim_error_kind::shutdown_already_observed,
                transport_error = std::nullopt,
                offending_field_name = "kind",
                ...}`.
  - `region.backend_to_core.state == empty` (unchanged).
- **Bug class:** the implementer added `send_notifier_state` but
  forgot to copy the cycle-9 shutdown short-circuit, so the method
  publishes wire bytes after the peer has signaled shutdown. The
  `transport_error == nullopt` assertion specifically catches "the
  short-circuit was added but in the wrong place (e.g. inside
  endpoint_.send's failure handling instead of before it)" — a
  short-circuit-after-`endpoint_.send` bug would produce
  `transport_error.has_value() == true`.

### C10-7. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundNotifierState`

- **Layer / contract:** non-negotiable #5 (deterministic and
  replayable). Outbound mirror of C8-6 / C9-7 for a schema with the
  most demanding padding profile (132 implicit pad bytes; 4 + 4N
  within the active prefix for count=N).
- **Setup:** two independent setups (two regions, two shim+core
  pairs) running the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core peer drains boot, sends `boot_ack` at sequence 0, shim polls.
  - Shim sends a 3-slot batch (bit-distinct field values per slot)
    at `sim_time_us = 250'000`. Concrete:
    `valid_notifier_state(std::array{
        valid_notifier_slot(1'000'000, 10, 1, 0, "alpha"),
        valid_notifier_slot(2'000'000, 20, 0, 1, "beta"),
        valid_notifier_slot(3'000'000, 30, 1, 1, "gamma")})`.
  - Core peer drains the published envelope into a `tier1_message`.
  - Shim sends a second 1-slot batch (different count, different
    fields) at `sim_time_us = 500'000`.
  - Core peer drains the second published envelope.
- **Expected outputs:**
  - For both setups: both sends succeed; both received envelopes
    have `has_value() == true`.
  - Setup A's first received envelope is byte-identical to setup B's
    first received envelope:
    - `setup_a.first.envelope == setup_b.first.envelope` via
      defaulted `sync_envelope::operator==`.
    - `setup_a.first.payload == setup_b.first.payload` via
      `std::vector<std::uint8_t>::operator==`.
    - `std::memcmp(setup_a.first.payload.data(),
                   setup_b.first.payload.data(),
                   setup_a.first.payload.size()) == 0` — byte-level
      companion that pins the **4-byte interior pad + per-slot
      trailing pads inside the active prefix** (`4 + 3*4 = 16` bytes
      for the 3-slot batch). For `notifier_state` this companion is
      load-bearing in a way that C9-7's was not: cycle 9's
      `can_frame_batch` had only per-frame trailing pad (3 bytes
      per frame), no interior pad. `notifier_state` adds the 4-byte
      interior `count → slots` pad to the mix, and a "shim's source
      `notifier_state{}` does not zero-init the interior pad on
      copy" bug would leak uninit between two runs.
  - Same byte-identity check for the second received envelope (4 +
    1*4 = 8 padding bytes inside its active prefix).
- **Bug class:** any seam-level nondeterminism on the outbound
  notifier_state path: shim including uninitialized stack bytes in
  the envelope build; per-slot or interior padding bytes diverging
  because of ABI surprise; sequence counter starting from a non-
  zero memory-residue value. The 132-pad-byte profile makes this
  schema the most sensitive of the three outbound-meaningful schemas
  to padding-byte determinism bugs.

---

## Tests deliberately not added (and why)

Per D-C10-INBOUND-INDEPENDENCE-INHERITS / D-C10-NO-CONNECT-GATE-
INHERITS / D-C10-RECEIVE-COUNTER-INDEPENDENCE-INHERITS and the
analogous reasoning for `lane_in_progress`, the following cycle-9
mirrors are **not added** for cycle 10:

- **C10-3b (lane_in_progress mirror of C9-3b).** The
  `lane_in_progress` failure path inside `endpoint_.send` is the
  same regardless of which schema is being sent — it's checked
  before `build_envelope` is called. The cycle-9 C9-3b test
  exercises this path; per-method mirrors don't add coverage.
- **C10-4 (no-connect-gate mirror of C9-4).** `validate_send_order`
  in `protocol_session.cpp` only checks `has_sent_boot_` for
  backend-side non-boot sends — this is a session-level gate, not a
  per-method gate. Cycle 9's C9-4 verified the principle.
- **C10-6 (inbound-independence mirror of C9-6).** The 8-cache-slot
  independence is structural: outbound code paths operate on the
  outbound lane and the session's send-side counter; the inbound
  cache slots are written only by `poll()`. The cycle-9 test
  exercised the principle; per-method mirrors don't add coverage
  unless a future implementation introduces shared mutable state
  between inbound and outbound (which would be a much bigger
  design change requiring its own cycle).
- **C10-8 (receive-counter independence mirror of C9-8).** Same
  reasoning as C10-6 — the send path's interaction with the session
  is through `build_envelope`, which mutates only
  `next_sequence_to_send_`. Per-method verification doesn't add
  coverage of the send/receive seam.

D-C10-SHUTDOWN-TERMINAL-INHERITS is the **one** D-C9-derived
contract that DOES get a per-method test (C10-5) because each typed
send method carries its own short-circuit code, so an "implementer
forgot the short-circuit on the new method" bug is a real risk
specific to per-method scope.

If reviewer disagrees with this trim — wants per-method mirrors of
all four — that's open question OQ-C10-PER-METHOD-MIRRORS below.

---

## Cross-test invariants

- Every test constructs one shim per test (no shared state).
- All `sim_time_us` values are explicit constants.
- All payload comparisons against published wire bytes use
  full-active-prefix `std::memcmp` (C10-1, C10-1b, C10-2) or
  defaulted `sync_envelope::operator==` plus vector-byte equality
  plus an explicit `std::memcmp` companion (C10-7).
- Pre-condition handshake steps use `ASSERT_TRUE` (failed setup
  invalidates the test, not just adds a spurious failure).
- Tests reuse cycle-9's anonymous-namespace helpers
  (`make_connected_shim`, `drain_boot_only`, `receive_from_shim`)
  unchanged.

---

## What this plan does NOT cover

- `send_error_message_batch` — cycle 11.
- The five inbound-only schemas as outbound API — intentionally
  inaccessible per D-C9-TYPED-OUTBOUND.
- `on_demand_request` / `on_demand_reply` envelopes carrying
  `notifier_state`. Future on-demand cycle.
- Shim-initiated `shutdown` envelope. Future cycle.
- The C HAL ABI `HAL_Notifier*` surface. Future cycle.
- Outbound `count > kMaxNotifiers` (count > 32) validation. Same
  upstream-validator coverage and same C-HAL-cycle-owns-the-clamp
  deferral as D-C9 documented for `can_frame_batch`. The shim
  trusts the caller to pass a well-constructed batch; the inlined
  active-prefix computation reads `state.count` directly. Reading
  past `slots[31]` for `count = 33+` is UB that the validator
  surfaces only after the read.
- Validity-domain checks on `notifier_slot` fields (e.g. `handle >=
  0`, `name` NUL-termination). Schema validator does framing only.
- The interaction between repeated `send_notifier_state` calls and
  `latest_notifier_state_` (the inbound cache slot, populated only
  by `poll()`). Cycle 10 does NOT touch the inbound side.
- Threading. Single-threaded; caller drives both `poll()` and
  `send_*` methods.
- Reset / reconnect.

---

## Implementation companion recommendations (non-test)

These are not test plan items but should land in the same cycle:

- **`#include <span>` and `#include <cstddef>` in
  `src/backend/common/can_frame.h` and
  `src/backend/common/notifier_state.h`** for the new
  `active_prefix_bytes` overloads. Currently `<span>` is not in
  either header.
- **`[[nodiscard]]` on the new `send_notifier_state`** — same
  reasoning as `make()` / `poll()` / `send_can_frame_batch`: a
  caller that ignores the returned `expected` would silently lose
  lane-busy / shutdown errors.
- **Doc comment on `send_notifier_state`** mirroring
  `send_can_frame_batch`'s — calls out the active-prefix discipline,
  the 8-byte header (vs cycle 9's 4-byte header), and the
  `tick_boundary`-only envelope kind.
- **Update the `using` directives in `tests/backend/shim/shim_core_test.cpp`**
  to add `using backend::active_prefix_bytes;` and remove the now-
  redundant `using tier1::helpers::active_prefix_bytes;` if none of
  the test-only overloads are needed (notifier_alarm_batch and
  error_message_batch overloads in test_helpers stay since their
  schemas have no production helper yet — keep both `using`s if
  any test still calls helpers::active_prefix_bytes for those two
  schemas).
- **Update `tests/backend/tier1/test_helpers.h`** to delete the
  `active_prefix_bytes(can_frame_batch)` and
  `active_prefix_bytes(notifier_state)` overloads. Keep the
  overloads for `notifier_alarm_batch` and `error_message_batch`.
  The tier1 test files that use these should follow the same
  `using` pattern as shim_core_test.cpp.

---

## Open questions

**OQ-C10-PER-METHOD-MIRRORS** — cycle 10 trims the cycle-9 test set
from 10 to 6 by skipping the C10-3b / C10-4 / C10-6 / C10-8
mirrors, on the rationale that these test cross-cutting contracts
already pinned by cycle 9. The trim aligns with code-style.md's
"don't add features past Phase B 'just because'" but breaks the
inbound-cycle precedent (cycles 2–8 maintained the full per-schema
family). Reviewer push-back welcome:
- If reviewer wants the full 10-test set: add C10-3b, C10-4, C10-6,
  C10-8 as mechanical mirrors of C9-3b / C9-4 / C9-6 / C9-8, just
  with `send_can_frame_batch` → `send_notifier_state`.
- If reviewer agrees with the trim: cycle 10 ships 6 tests.
- If reviewer wants to convert to a parameterized fixture
  (`INSTANTIATE_TEST_SUITE_P` over `(typed-send-method, schema-id,
  payload-builder)`): that conversion would land at cycle 10 OR
  cycle 11 (when the third outbound method exists). Cycle 10 stays
  linear in this plan; reviewer may push to convert here.

**OQ-C10-HELPER-LOCATION** — D-C10-EXTRACT-ACTIVE-PREFIX puts the
production `active_prefix_bytes` overloads in the schema headers
themselves (`can_frame.h`, `notifier_state.h`). Alternative: a new
`src/backend/common/active_prefix.h` header that includes the four
schema headers and declares all overloads in one place. The plan
picks per-schema-header for three reasons: (a) one fewer file, (b)
the helper lives next to the struct it operates on, (c) callers
that already include `can_frame.h` get the helper without an extra
include. Reviewer push-back welcome.

**OQ-C10-NOTIFIER-ALARM-OVERLOAD** — the test-only
`active_prefix_bytes(notifier_alarm_batch)` in test_helpers.h stays
because no production code sends notifier_alarm_batch (it's sim-
authoritative). Alternative: add a production overload anyway "for
symmetry" so all four variable-size schemas have the same shape.
The plan picks "no production overload until there's a production
caller" per the no-speculative-features rule. Reviewer push-back
welcome.
