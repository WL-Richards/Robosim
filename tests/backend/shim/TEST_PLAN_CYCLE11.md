# HAL shim core — cycle 11 test plan (outbound error_message_batch)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-30, single review round, no blockers). Reviewer
explicitly resolved both open questions: OQ-C11-FULL-CAPACITY-TEST
NOT needed (would be coverage-exercise; the arithmetic is already
exercised at counts 0, 2, 3, 4 across the suite); OQ-C11-MEMCMP-IN-
DETERMINISM keep for parity (documentary value preserves cross-
cycle structural consistency). Plus one non-blocking recommendation
to add `static_assert(offsetof(error_message_batch, messages) == 8)`
to `error_message.h` — addressed in implementation. Cycle-11
production code and 6 new tests landed and 113-shim-test green
(full project ctest 428/428 green) under both clang Debug and
GCC Debug + ASan + UBSan.

**Implements:** the eleventh TDD cycle of the HAL shim core, the
**third (and final) outbound-meaningful schema** for v0, and **closes
the outbound-past-boot surface set** for the three semantically-
meaningful outbound schemas. Cycles 1–10 are landed and 107-shim-
test green; full project ctest 422/422 green under both clang Debug
and GCC Debug + ASan + UBSan. Cycle 11 adds `send_error_message_batch`
(the third typed `send_*` method) and adds the third production
`active_prefix_bytes` overload to `error_message.h`.

---

## Why this cycle exists

Cycles 9 and 10 wired the first two outbound surfaces and their
production `active_prefix_bytes` helpers. The third
outbound-meaningful schema, `error_message_batch` (`HAL_SendError`-
bound logs from the robot to the sim core), is the only piece left
before the outbound API surface is complete. Without it the future
C HAL ABI cycle that wires `HAL_SendError` has nowhere to push.

`error_message_batch` is the natural follow-on because:

1. **It closes the outbound set.** `kPerKindAllowedSchemas` allows
   eight payload schemas under `tick_boundary`; five are sim-
   authoritative and intentionally absent from the outbound API per
   D-C9-TYPED-OUTBOUND. The remaining three — `can_frame_batch`
   (cycle 9), `notifier_state` (cycle 10), `error_message_batch`
   (this cycle) — exhaust the outbound-meaningful set.
2. **It is the only padding-free variable-size schema.** Per
   D-C8-PADDING-FREE, `error_message_batch` and `error_message`
   have **zero implicit C++ padding** because both `reserved_pad`
   fields are *named* (the 4-byte interior between `count` and
   `messages`, and the 3-byte interior after `truncation_flags`).
   This is a meaningful contrast with cycle 10's `notifier_state`,
   which had 132 implicit padding bytes that the C10-7 memcmp
   companion had to pin. C11-7's memcmp companion is kept for
   parity with C9-7 / C10-7 but is byte-equivalent to the
   `vector::operator==` it sits next to — the cycle's "Determinism
   notes" section explains why.
3. **It is the largest outbound payload by far.** `sizeof(error_message)
   == 2324` bytes; `sizeof(error_message_batch) == 18600` bytes.
   A 3-message active prefix is 8 + 3*2324 = 6980 bytes (vs cycle
   9's 64 bytes for 3 frames, cycle 10's 272 bytes for 3 slots).
   The lane capacity (`kTier1MaxPayloadBytes`) is sized exactly to
   `sizeof(error_message_batch) == 18600` to accommodate a full
   8-message batch — verifying full-capacity send is a meaningful
   boundary test for the lane sizing assumption.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

```cpp
// Publishes `batch` as a tick_boundary envelope into the shim's
// outbound backend_to_core lane. Sends exactly the active prefix
// (offsetof(error_message_batch, messages) + batch.count *
// sizeof(error_message)) — NOT sizeof(error_message_batch) — so the
// validator's count-vs-length contract holds. Returns send_failed
// wrapping the underlying tier1 transport error if the lane is busy /
// in-progress, or shutdown_already_observed once is_shutting_down()
// is true.
[[nodiscard]] std::expected<void, shim_error> send_error_message_batch(
    const error_message_batch& batch, std::uint64_t sim_time_us);
```

The header offset is **8** (the 4-byte count word + the 4-byte
*named* `reserved_pad[4]`). Same numeric offset as cycle 10's
`notifier_state` but for a different reason: cycle 10's offset of 8
came from implicit pad bytes (alignof 8 on `notifier_slot`); cycle
11's offset of 8 comes from the named `reserved_pad[4]` field that
is part of the wire schema. Both `offsetof` calls return 8;
`payload_bytes` arithmetic is identical in form even though the
underlying byte layout differs.

`src/backend/common/error_message.h` (third production
`active_prefix_bytes` overload, mirroring the cycle-10 extraction
pattern from `can_frame.h` and `notifier_state.h`):

```cpp
inline std::span<const std::uint8_t> active_prefix_bytes(
    const error_message_batch& batch) {
  const std::size_t active_size =
      offsetof(error_message_batch, messages) +
      static_cast<std::size_t>(batch.count) * sizeof(error_message);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}
```

### Internal additions

- `send_error_message_batch` definition in `shim_core.cpp` calls
  `endpoint_.send(envelope_kind::tick_boundary,
  schema_id::error_message_batch, backend::active_prefix_bytes(batch),
  sim_time_us)` after the `shutdown_observed_` short-circuit pattern
  (D-C9-SHUTDOWN-TERMINAL inherited).
- `tests/backend/tier1/test_helpers.h`: the existing
  `active_prefix_bytes(error_message_batch)` overload (currently the
  fourth and last one in the file) is **deleted** per the cycle-10
  extraction pattern. The remaining test-only overload is
  `notifier_alarm_batch` (sim-authoritative; no production caller).

### Out of scope

- The five inbound-only schemas as outbound API (intentionally
  inaccessible per D-C9-TYPED-OUTBOUND).
- `on_demand_request` / `on_demand_reply` envelopes carrying
  `error_message_batch`. The validator's `kPerKindAllowedSchemas`
  table allows `error_message_batch` under `on_demand_reply` (a
  reply may carry a synchronous error result) but NOT under
  `on_demand_request` (errors are pushed, not requested). The
  on-demand cycle handles this; cycle 11 only publishes via
  `tick_boundary`.
- Shim-initiated `shutdown` envelope. Future cycle.
- The C HAL ABI `HAL_SendError` / `HAL_SendConsoleLine` surfaces.
  Future cycle owns the buffering-per-call + flush-at-tick-boundary
  discipline that drives `send_error_message_batch`.
- Outbound `count > kMaxErrorsPerBatch` (count > 8) clamp. Same
  upstream-validator coverage and same C-HAL-cycle-owns-the-clamp
  deferral as cycles 9 and 10 documented.
- Validity-domain checks on `error_message` fields (e.g. NUL-
  termination of the fixed-size string buffers, consistency between
  `truncation_flags` and actual content truncation). Schema
  validator does framing only; truncation is the caller's job via
  `copy_truncated`.
- Threading. Single-threaded; caller drives both `poll()` and
  `send_*` methods.

---

## Decisions pinned

Cycle 11 introduces **no new design decisions**. All applicable
contracts inherit from cycles 9 and 10:

- **D-C9-TYPED-OUTBOUND** — `send_error_message_batch` is a typed-
  per-schema method; no generic `send_tick_boundary(...)` is
  introduced.
- **D-C9-ACTIVE-PREFIX-OUT** — `send_error_message_batch` writes
  exactly `offsetof(error_message_batch, messages) + batch.count *
  sizeof(error_message)` bytes onto the wire. Pinned by C11-1 and
  C11-1b.
- **D-C9-EMPTY-BATCH-OUT** — count=0 is a legal publish; the active
  prefix is the 8-byte header (4-byte count + 4-byte named
  reserved_pad). Pinned by C11-1b.
- **D-C9-SEQUENCE-ADVANCE** — pinned by C11-2 (positive) and C11-3
  (failure preserves).
- **D-C9-SHUTDOWN-TERMINAL** — pinned **per-method** by C11-5; the
  one D-C9-derived contract that needs per-method verification per
  the D-C10-SHUTDOWN-TERMINAL-INHERITS reasoning (each typed `send_*`
  method carries its own short-circuit code; an "implementer added
  the new method but forgot the short-circuit" bug is per-method
  scope).
- **D-C9-WRAPPED-SEND-ERROR** — `send_error_message_batch` consumes
  the same shared `wrap_send_error` lambda.
- **D-C9-NO-CONNECT-GATE / D-C9-INBOUND-INDEPENDENCE / receive-
  counter-non-perturbation (C9-8)** — all session/transport-level,
  no per-method test. Same trim convention as cycle 10
  (D-C10-{INBOUND-INDEPENDENCE / NO-CONNECT-GATE / RECEIVE-COUNTER-
  INDEPENDENCE}-INHERITS), endorsed by the cycle-10 reviewer with
  the directive "A future reviewer should not revert to 10-test
  coverage without a concrete new bug class that the trim fails to
  catch."
- **D-C10-EXTRACT-ACTIVE-PREFIX** — extended to cover
  `error_message_batch` by adding the third overload to
  `error_message.h` and deleting the test-helpers duplicate.
- **D-C8-PADDING-FREE** — `error_message_batch` has zero implicit
  C++ padding; this is a known property from cycle 8 that affects
  the C11-7 determinism test framing (see "Determinism notes"
  below).

---

## Test fixtures

Cycle 11 adds NO new helpers to `tests/backend/tier1/test_helpers.h`.
The existing `valid_error_message` and `valid_error_message_batch`
helpers cover the fixture surface.

The `active_prefix_bytes(error_message_batch)` overload in
`test_helpers.h` is **deleted** per D-C10-EXTRACT-ACTIVE-PREFIX
extension; tests call `backend::active_prefix_bytes(batch)` instead
(via ADL since `error_message_batch` lives in `robosim::backend`).

---

## Determinism notes

`error_message_batch` is **the only outbound v0 schema with zero
implicit C++ padding** (D-C8-PADDING-FREE inherited from cycle 8):

- The 4-byte interior gap between `count` (offsets 0–3) and
  `messages` (offset 8) is a *named* `reserved_pad[4]` field.
- The 3-byte interior gap after `error_message::truncation_flags`
  (offsets 17–19) is a *named* `reserved_pad[3]` field.
- All other fields are naturally aligned with no implicit gaps.
- Defaulted `error_message_batch::operator==` and
  `error_message::operator==` both cover every byte of the struct
  through the named `reserved_pad` fields.

Consequence for C11-7: the `std::memcmp` companion is kept for
**parity with C9-7 and C10-7's pattern**, but it is **byte-equivalent
to the `vector::operator==` and `tier1_message::operator==`** that
sit next to it. There is no "padding bytes that operator== cannot
see" angle here; the memcmp is documentation symmetry, not load-
bearing the way it was for `notifier_state`.

The bug class C11-7 catches is the same as C9-7's: "shim includes
uninitialized stack bytes in the envelope build" — but for
`error_message_batch` this would be caught by `vector::operator==`
alone, since every byte of the 6980-byte active prefix (for a
3-message batch) is part of a named field that participates in
`operator==`. The memcmp is kept because removing it would create
an asymmetry across the three outbound determinism tests; the cost
is one extra assertion line per test.

---

## Proposed tests (revision 1)

### C11-1. `ShimCoreSend.PublishesErrorMessageBatchAsTickBoundaryWithActivePrefixWireBytes`

- **Layer / contract:** `send_error_message_batch` → tier1 wire;
  D-C9-ACTIVE-PREFIX-OUT; D-C9-SEQUENCE-ADVANCE positive (first
  send); D-C10-EXTRACT-ACTIVE-PREFIX (the new production helper
  produces correct bytes).
- **Setup:**
  - Fresh region; backend endpoint; shim via
    `make_connected_shim(region, core)`.
  - Construct a 3-message batch:
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
                            "third call stack")})`. Bit-distinct field
    values per message catch field-misalignment bugs.
- **Action:** `auto sent = shim.send_error_message_batch(batch, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - The core peer's `try_receive()` returns a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::error_message_batch`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1` (boot took 0).
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 6980` (= 8 + 3*2324).
    - `payload.size() == 6980`.
    - `std::memcmp(payload.data(), &batch, 6980) == 0`.
  - **What the 6980-byte memcmp range covers:** the 4-byte count
    word at offsets 0–3, the 4-byte named `reserved_pad` at offsets
    4–7, and three full 2324-byte `error_message` entries at offsets
    8–6979. Per D-C8-PADDING-FREE there is **zero implicit padding**
    inside this range; every byte is part of a named field that
    participates in `operator==`. The memcmp here is byte-equivalent
    to `defaulted error_message_batch::operator==`, kept to mirror
    C9-1 / C10-1's structural shape.
- **Bug class:** wrong envelope kind; wrong schema id; sequence not
  starting at 1; sim_time hardcoded; active-prefix vs sizeof
  confusion (would deliver 18600 bytes instead of 6980; the
  validator rejects on `payload_size_mismatch`); the production
  helper's offsetof returning 4 instead of 8 (would deliver 6976
  bytes; validator rejects); per-message field-offset confusion
  (the wide 2324-byte element with 8 named fields makes "shim
  copies only the first N bytes per element" bugs detectable on
  any field past the first).

### C11-1b. `ShimCoreSend.PublishesEmptyErrorMessageBatchAsHeaderOnlyTickBoundary`

- **Layer / contract:** D-C9-EMPTY-BATCH-OUT; D-C9-ACTIVE-PREFIX-OUT
  boundary (count=0); the 8-byte header (count word + named
  `reserved_pad[4]`) lands on the wire byte-for-byte.
- **Setup:** identical to C11-1 through the boot+boot_ack handshake.
  Construct an empty batch via `valid_error_message_batch()` (default-
  arg empty span).
- **Action:** `shim.send_error_message_batch(empty_batch, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - Core peer receives a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::error_message_batch`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1`.
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 8`.
    - `payload.size() == 8`.
  - `std::memcmp(payload.data(), &empty_batch, 8) == 0` — covers
    the count word AND the named `reserved_pad[4]`, both byte-zero
    from the `error_message_batch{}` zero-init in
    `valid_error_message_batch`.
- **Bug class:** "shim refuses count=0"; shim treats the named
  `reserved_pad` as if it were implicit padding and skips it (would
  deliver 4 bytes instead of 8; validator rejects); shim sends
  `sizeof = 18600` bytes for a count=0 batch.

### C11-2. `ShimCoreSend.SecondErrorMessageBatchSendAdvancesOutboundSequenceToTwo`

- **Layer / contract:** D-C9-SEQUENCE-ADVANCE positive arm,
  per-method verification.
- **Setup:** identical to C11-1. After the first
  `send_error_message_batch` succeeds, the core peer drains the
  published envelope and the shim sends a second batch with bit-
  distinct fields and a different count.
- **Concrete payload values:**
  - First batch: 2 messages, error_codes `{10, 20}`, distinct
    severity / truncation_flags / detail strings per message.
  - Second batch: 4 messages, error_codes `{50, 60, 70, 80}`.
    Different count from first catches a "shim treats sequence
    advance as count-based" bug.
- **Action:**
  - `shim.send_error_message_batch(first, 100'000);`
  - `core.try_receive();`
  - `shim.send_error_message_batch(second, 200'000);`
- **Expected outputs:**
  - Both sends `has_value() == true`.
  - First received envelope: `sequence == 1`,
    `payload_bytes == 4656` (= 8 + 2*2324).
  - Second received envelope: `sequence == 2`,
    `payload_bytes == 9304` (= 8 + 4*2324),
    `sim_time_us == 200'000`.
  - Full active-prefix `std::memcmp` byte-equality on both envelopes.
- **Bug class:** sequence counter doesn't advance / advances by count
  / resets between sends; the helper produces wrong size for one of
  the two counts.

### C11-3. `ShimCoreSend.LaneBusyErrorMessageBatchSendIsRejectedAndPreservesSequenceCounter`

- **Layer / contract:** D-C9-SEQUENCE-ADVANCE negative arm
  (`lane_busy` for the new method); D-C9-WRAPPED-SEND-ERROR.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make` (boot envelope
    sent into `region.backend_to_core`, lane state `full`).
  - Core peer does NOT drain the boot envelope.
  - Capture boot bytes for non-clobber assertion.
- **Action:** `auto first_attempt = shim.send_error_message_batch(...3-message batch..., /*sim_time_us=*/100'000);`
- **Expected outputs:**
  - `first_attempt` returns
    `shim_error{shim_error_kind::send_failed,
                transport_error = tier1_transport_error{kind = lane_busy, ...},
                offending_field_name = "lane",
                ...}`.
  - Lane state still `full`; boot envelope and payload bytes
    unchanged.
  - Recovery proof: drain boot, resend, assert sequence == 1.
- **Bug class:** failed sends burn sequence numbers; failed sends
  clobber the lane; failed sends return success.

### C11-5. `ShimCoreSend.PostShutdownErrorMessageBatchSendIsRejectedWithoutTouchingLane`

- **Layer / contract:** D-C9-SHUTDOWN-TERMINAL — per-method
  verification (the only D-C9-derived contract that needs per-method
  testing per the D-C10-SHUTDOWN-TERMINAL-INHERITS reasoning).
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`.
  - Core peer drains boot, sends `boot_ack` at sequence 0, shim polls.
  - Core peer sends `shutdown` at sequence 1, shim polls
    (`is_shutting_down() == true`).
- **Pre-action assertions:**
  - `ASSERT_EQ(region.backend_to_core.state.load(), empty)` (the
    outbound lane is empty because the shim has not sent anything
    since the drained boot — same precondition as C9-5 / C10-5).
  - `ASSERT_TRUE(shim.is_shutting_down())`.
- **Action:** `auto sent = shim.send_error_message_batch(...1-message batch..., 250'000);`
- **Expected outputs:**
  - `sent` returns
    `shim_error{shim_error_kind::shutdown_already_observed,
                transport_error = std::nullopt,
                offending_field_name = "kind",
                ...}`.
  - `region.backend_to_core.state == empty` (unchanged).
- **Bug class:** implementer forgot the short-circuit on the new
  method; short-circuit placed after `endpoint_.send` (would
  produce `transport_error.has_value() == true` instead of
  `nullopt`).

### C11-7. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundErrorMessageBatch`

- **Layer / contract:** non-negotiable #5 (deterministic and
  replayable). Outbound determinism for the third (and last) outbound
  v0 schema, and the only padding-free one.
- **Setup:** two independent setups (two regions, two shim+core
  pairs) running the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core drains boot, sends `boot_ack`, shim polls.
  - Shim sends a 3-message batch (bit-distinct field values per
    message — see C11-1's fixture) at `sim_time_us = 250'000`.
  - Core drains.
  - Shim sends a second 1-message batch (different count, different
    fields) at `sim_time_us = 500'000`.
  - Core drains.
- **Expected outputs:**
  - Both sends succeed in both setups.
  - First envelope byte-identical between setups: `envelope ==`,
    `payload ==`, plus `std::memcmp` companion. **The memcmp is
    byte-equivalent to vector::operator==** for this schema (per
    D-C8-PADDING-FREE there are no padding bytes that `operator==`
    cannot see); kept for parity with C9-7 / C10-7's structural
    shape.
  - Same byte-identity check for the second envelope.
- **Bug class:** any seam-level nondeterminism on the outbound
  error_message_batch path: shim including uninitialized stack
  bytes in the envelope build; sequence counter starting from a
  non-zero memory-residue value; sim_time_us getting clobbered by
  uninit. For this schema, since there is no implicit padding, the
  `vector::operator==` already catches the bug class — the memcmp
  is documentation, not coverage.

---

## Tests deliberately not added (and why)

Per the cycle-10-established trim convention, the cycle-9 mirrors
that test cross-cutting session/transport-level contracts are NOT
re-added per outbound schema:

- **C11-3b (lane_in_progress).** Same path as C9-3b / cycle-10's
  trimmed mirror. The `lane_in_progress` check is schema-agnostic
  (lives at `shared_memory_transport.cpp:250-253`, before
  `build_envelope`).
- **C11-4 (no-connect-gate).** Session-level, not per-method.
- **C11-6 (inbound-independence).** Structural — outbound code paths
  operate only on the outbound lane and the session's send-side
  counter.
- **C11-8 (receive-counter-independence).** Structural —
  `build_envelope` mutates only `next_sequence_to_send_`.

Test-reviewer's cycle-10 directive: "A future reviewer should not
revert to 10-test coverage without a concrete new bug class that
the trim fails to catch." Cycle 11 sees no new bug class; trim
applies.

---

## Cross-test invariants

Same as cycles 9 and 10:
- One shim per test.
- All `sim_time_us` values are explicit constants.
- Full-active-prefix `std::memcmp` for payload comparisons (C11-1,
  C11-1b, C11-2) or defaulted `sync_envelope::operator==` plus
  vector-byte equality plus a memcmp companion (C11-7).
- Pre-condition setup uses `ASSERT_TRUE`.
- Tests reuse cycle-9's anonymous-namespace helpers
  (`make_connected_shim`, `drain_boot_only`, `receive_from_shim`).

---

## What this plan does NOT cover

- The five inbound-only schemas as outbound API (intentionally
  inaccessible per D-C9-TYPED-OUTBOUND).
- `on_demand_request` / `on_demand_reply` envelopes carrying
  `error_message_batch`. Future on-demand cycle.
- Shim-initiated `shutdown` envelope. Future cycle.
- The C HAL ABI `HAL_SendError` / `HAL_SendConsoleLine` surface.
  Future cycle.
- Outbound `count > kMaxErrorsPerBatch` (count > 8). Same
  upstream-validator coverage and same C-HAL-cycle-owns-the-clamp
  deferral as cycles 9 and 10.
- Validity-domain checks on `error_message` fields (NUL-termination
  of fixed-size buffers; `truncation_flags` consistency with content).
  Schema validator does framing only.
- Full-capacity send (count=8, payload=18600 bytes = exactly
  `kTier1MaxPayloadBytes`). The plan considered adding C11-1c for
  this boundary but skipped it: the tier1 transport tests already
  pin the `kTier1MaxPayloadBytes` lane capacity, and the shim's
  `send_error_message_batch` is a pure pass-through of the active
  prefix to `endpoint_.send` — the boundary is downstream. If
  reviewer wants the explicit shim-side test, see open question
  OQ-C11-FULL-CAPACITY-TEST.
- Threading. Single-threaded.
- Reset / reconnect.

---

## Implementation companion recommendations (non-test)

- **`#include <span>` and `#include <cstddef>` in `error_message.h`**
  for the new `active_prefix_bytes` overload. Currently `<span>`
  is not in the header (`<cstddef>` already is per the existing
  `static_assert(sizeof(error_message_batch) ==
  offsetof(error_message_batch, messages) + ...)` check).
- **Doc comment on `send_error_message_batch`** mirroring
  `send_can_frame_batch` / `send_notifier_state` — calls out the
  active-prefix discipline, the 8-byte header (named reserved_pad
  rather than implicit padding, but same numeric offset as cycle
  10), and the `tick_boundary`-only envelope kind.
- **Update `tests/backend/tier1/test_helpers.h`** to delete the
  `active_prefix_bytes(error_message_batch)` overload. After this
  deletion, the only remaining test-only overload is
  `notifier_alarm_batch` (sim-authoritative, no production caller).

---

## Open questions

**OQ-C11-FULL-CAPACITY-TEST** — should cycle 11 add an explicit
boundary test for `count == kMaxErrorsPerBatch == 8`, exercising
the `payload_bytes == 18600 == kTier1MaxPayloadBytes` boundary?
The plan picks "no" because the tier1 transport tests already pin
the lane capacity and `send_error_message_batch` is a pure pass-
through. The bug class "shim's active-prefix computation overflows
or wraps for count=8" is caught by the helper's straightforward
`offsetof + count*sizeof` arithmetic which the cycle-10 reviewer
already endorsed. Reviewer push-back welcome — adding C11-1c with
a count=8 batch is one extra test.

**OQ-C11-MEMCMP-IN-DETERMINISM** — C11-7's memcmp companion is
byte-equivalent to `vector::operator==` per D-C8-PADDING-FREE.
Should cycle 11 drop the memcmp and rely on `vector::operator==`
alone? Plan picks "keep for parity with C9-7 / C10-7's structural
shape" but a "drop the redundant assertion" alternative is
defensible. Reviewer push-back welcome.
