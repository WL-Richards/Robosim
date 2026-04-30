# HAL shim core — cycle 9 test plan (outbound can_frame_batch)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, two review rounds). Round-1 verdict was
`not-ready` with 2 blockers + 3 non-blocking items + a systemic
off-by-one error in the decision cross-references that the
reviewer flagged for one decision but that affected four;
round-2 verdict `ready-to-implement` with one residual non-
blocking stale comment (the `drain_boot_only` helper's "useful
for C9-5" reference, which after the rev-2 renumber should have
read "C9-4") that was corrected in-place after round 2 closed.
Cycle-9 production code and 10 new tests landed and 101-shim-
test green (full project ctest 416/416 green) under both clang
Debug and GCC Debug + ASan + UBSan. Rev 2 addresses all six
round-1 items; see "Revision rationale (round 1 → round 2)" at
the bottom of this file.

**Implements:** the ninth TDD cycle of the HAL shim core, and the
**first outbound-past-boot** cycle. Cycles 1–8 are landed and 91-shim-
test green; full project ctest 406/406 green under both clang Debug
and GCC Debug + ASan + UBSan. Cycle 9 promotes the shim from
"send-only-once-at-construction" to a working duplex by adding the
first post-boot outbound surface: the shim publishes a
`can_frame_batch` payload as a `tick_boundary` envelope into its
`backend_to_core` lane.

---

## Why this cycle exists

After cycle 8 the shim's inbound path is closed (8/8 per-tick
schemas wired). Outbound still has only one envelope ever — the
boot envelope sent from `make()`. Every other outbound concern
(tick_boundary, on_demand_request, shim-initiated shutdown) is
"future cycle" handwave.

Closing that asymmetry next is the right call:

1. **It unblocks the C HAL ABI cycles.** The exported C HAL
   surfaces (`HAL_CAN_SendMessage`, `HAL_SendError`, `HAL_Notifier*`)
   that future cycles add must end up writing into outbound
   `tick_boundary` envelopes. With no outbound surface today, those
   cycles would have to invent it as a side-effect of the C HAL
   work — bigger commits, harder reviews, and it would silently
   take on outbound design decisions that deserve their own TDD
   pass.
2. **It exercises the protocol_session's send-side state machine
   end-to-end.** Cycles 1–8 only ever drove the receive side past
   the boot handshake. The send-side `validate_send_order` for
   non-boot kinds (the `expected_local_boot_first` gate, the
   sequence-counter advance, the no-`expected_boot_ack_first`-gate
   semantics on outbound) has been pinned in
   `protocol_session_test.cpp` directly but never through the
   shim. Cycle 9 closes that.
3. **It picks the most representative outbound schema first.**
   `can_frame_batch` is variable-size (so cycle 9 establishes the
   active-prefix discipline on the outbound side, mirroring D-C4-
   VARIABLE-SIZE on the inbound side), and CAN TX is the highest-
   frequency outbound path on a real robot — the one that will
   get exercised the most across future C HAL work.

The shim's outbound surface for non-boot envelopes is the **typed-
per-schema-method** shape (D-C9-TYPED-OUTBOUND below). Cycle 9
lands `send_can_frame_batch`. Future cycles will add
`send_notifier_state`, `send_error_message_batch`, on-demand
request/reply pairing, and shim-initiated shutdown as their own
TDD passes.

`can_frame_batch` is allowed under `tick_boundary`,
`on_demand_request`, and `on_demand_reply` per
`kPerKindAllowedSchemas` (`validator.h:35-44`). Cycle 9 only
publishes via `tick_boundary`; on-demand pairing is a separate
future cycle. The method name `send_can_frame_batch` therefore
implicitly carries the `envelope_kind::tick_boundary` choice;
when on-demand lands, the method shape there will be different
(it returns a reply) so naming collision does not arise.

---

## Contract under test (additions only)

### New public surface

`src/backend/shim/shim_core.h`:

```cpp
// Publishes `batch` as a tick_boundary envelope into the shim's
// outbound backend_to_core lane. Sends exactly the active prefix
// (offsetof(can_frame_batch, frames) + batch.count * sizeof(can_frame))
// — NOT sizeof(can_frame_batch) — so the validator's count-vs-length
// contract holds. Returns send_failed wrapping the underlying tier1
// transport error if the lane is busy / in-progress / oversize, or
// shutdown_already_observed once is_shutting_down() is true.
[[nodiscard]] std::expected<void, shim_error> send_can_frame_batch(
    const can_frame_batch& batch, std::uint64_t sim_time_us);
```

### Internal additions

- The implementation computes `active_prefix_size = offsetof(can_frame_batch, frames) + batch.count * sizeof(can_frame)` and calls `endpoint_.send(envelope_kind::tick_boundary, schema_id::can_frame_batch, {bytes, active_prefix_size}, sim_time_us)`. **Inlined** in `send_can_frame_batch`; no helper extraction this cycle (D-C9-NO-HELPER below).
- The post-shutdown short-circuit (`if (shutdown_observed_) return shutdown_already_observed`) is added to `send_can_frame_batch` to mirror `poll()`'s terminal contract (D-C9-SHUTDOWN-TERMINAL).
- No new fields. No new private methods.

### Out of scope

- The other four outbound-meaningful schemas (`send_notifier_state`, `send_error_message_batch` — and arguably the five inbound-only schemas if a future "shim relays sim-side state to a logger" use case ever materialises, which it won't in v0). Each is its own cycle.
- All five inbound-only schemas (`clock_state`, `power_state`, `ds_state`, `can_status`, `notifier_alarm_batch`) — these are sim-authoritative; the shim has no semantic reason to publish them, and the typed-method API per D-C9-TYPED-OUTBOUND makes wrong-direction publishing impossible at the API boundary even though the validator's `kPerKindAllowedSchemas` table allows it under `tick_boundary` in either direction.
- `on_demand_request` / `on_demand_reply` envelopes carrying `can_frame_batch`. Cycle 9 only publishes via `tick_boundary`; on-demand cycles add their own send paths.
- Shim-initiated `shutdown` envelope. Future cycle.
- Promotion of the inbound `latest_can_frame_batch_` from latest-wins to a queue. Per D-C4-LATEST-WINS, that decision waits for the cycle that wires `HAL_CAN_ReadStreamSession`. Outbound cycle 9 does NOT touch the inbound side.
- The C HAL ABI (`HAL_CAN_SendMessage`, `HAL_CAN_WriteMessage`). Cycle 9 stops at the C++ shim API. The C HAL surface that batches per-frame `HAL_CAN_SendMessage` calls and flushes via `send_can_frame_batch` at tick boundaries is a future cycle.
- Threading. Single-threaded; caller drives both `poll()` and `send_can_frame_batch`. The C HAL cycle that introduces multi-thread call sites also introduces the threading model.

---

## Decisions pinned

- **D-C9-TYPED-OUTBOUND.** Each outbound-meaningful schema gets its own typed `send_<schema>` method; no generic `send_tick_boundary(schema_id, ...)` surface. Two reasons: (a) the typed method makes the schema-id passing impossible to get wrong at the call site, mirroring how `make()` takes a typed `boot_descriptor&` rather than `(schema_id::boot_descriptor, span<bytes>)`, and (b) the shim's outbound surface is intentionally narrow — only the three semantically-meaningful outbound schemas (`can_frame_batch`, `notifier_state`, `error_message_batch`) get methods, and there is no API path to publish a `clock_state` or other sim-authoritative schema by accident. Pinned by the public-surface shape (no parameterized test, but the absence of a generic method is the contract).
- **D-C9-ACTIVE-PREFIX-OUT.** `send_can_frame_batch` writes exactly `offsetof(can_frame_batch, frames) + batch.count * sizeof(can_frame)` bytes onto the wire — NOT `sizeof(can_frame_batch)`. This is the outbound mirror of D-C4-VARIABLE-SIZE on the inbound side, and it is what makes `protocol_session::build_envelope`'s payload-bytes validation hold (see `validator.cpp:62-112`'s `expected_variable_payload_size`, which the build-side validator runs against the same payload span). A "shim sends sizeof regardless of count" bug would deliver 1284 bytes for a count=3 batch (expected 64); the validator inside `build_envelope` rejects it as `payload_size_mismatch`, the build returns an error, no envelope is published, and the lane stays empty. Pinned by C9-1 (count=3 lands exactly 64 bytes on the wire) and C9-1b (count=0 lands exactly 4 bytes).
- **D-C9-EMPTY-BATCH-OUT.** count=0 batches are a legal outbound publish; the active prefix is `offsetof(can_frame_batch, frames)` = 4 bytes (just the count word). Useful for "the shim has nothing to publish this tick but wants to advance the wire heartbeat." Pinned by C9-1b.
- **D-C9-SEQUENCE-ADVANCE.** A successful `send_can_frame_batch` advances the outbound sequence counter by exactly 1; a failed one (lane_busy, etc.) does NOT advance it. This is the protocol_session's contract (decision #4 of `protocol-session.md`) and the tier1 transport's contract (decision #4 of `tier1-shared-memory-transport.md`); cycle 9 verifies it surfaces correctly through the shim. The boot envelope took sequence 0; the first post-boot send takes sequence 1; lane-busy preserves "next is still 1." Pinned by C9-2 (positive advance) and C9-3 (failure preserves) and C9-3b (lane_in_progress also preserves).
- **D-C9-NO-CONNECT-GATE.** `send_can_frame_batch` succeeds even when `is_connected() == false`, as long as the outbound lane is drainable. The protocol_session's `validate_send_order` only requires `has_sent_boot_` for backend-side non-boot sends (`protocol_session.cpp:162-166`); `has_received_boot_ack_` is not on the send-side path. A "shim defensively gates outbound on `is_connected()`" bug would refuse the publish; the protocol does not require it. Pinned by C9-4 (publishes successfully before the boot_ack inbound poll).
- **D-C9-SHUTDOWN-TERMINAL.** Once `is_shutting_down() == true`, `send_can_frame_batch` returns `shutdown_already_observed` without touching the lane or the session counter. Mirror of `poll()`'s short-circuit (cycle-1 design decision #6). Rationale: the terminal-state non-negotiable applies symmetrically to send and receive; publishing wire bytes after the peer has signaled shutdown is wasted work and clouds the "shim is dead" semantic. Pinned by C9-5.
- **D-C9-INBOUND-INDEPENDENCE.** A successful `send_can_frame_batch` does NOT mutate any of the eight inbound cache slots. The outbound path operates on the outbound lane and the session's send-side counter; touching `latest_can_frame_batch_` (or any sibling) would be a copy-paste-from-poll() bug. Pinned by C9-6 (all 8 cache observers stay `nullopt` across the send).
- **D-C9-NO-HELPER.** The active-prefix size computation (`offsetof(can_frame_batch, frames) + count * sizeof(can_frame)`) is inlined in `send_can_frame_batch`. The `tests/backend/tier1/test_helpers.h::active_prefix_bytes(const can_frame_batch&)` helper is not promoted into production code this cycle — only one production caller would consume it today, and code-style.md ("Don't add features past Phase B 'just because.'") favors inlining. When the second outbound variable-size schema lands (`send_notifier_state` or `send_error_message_batch`), the duplication is the trigger for extraction. The implementation comment must state this explicitly so the third-cycle implementer knows where to lift the helper.
- **D-C9-WRAPPED-SEND-ERROR.** Transport errors from the underlying `endpoint_.send` are wrapped in `shim_error{kind=send_failed, transport_error=...}`, reusing the existing `wrap_send_error` lambda from `shim_core.cpp:10-16` that boot uses today. The wrap function's message field is updated from "transport rejected outbound boot envelope" to a generic "transport rejected outbound envelope" so it serves both call sites; alternatively, two separate wrap functions, one per call site. **Plan picks: refactor to one shared wrap function with a generic message** to avoid hard-coding the call-site name in the message — the typed `kind` and `offending_field_name` carry the diagnostic, the message is human-readable supplement. Reviewer push-back welcome on the single-vs-two-wrap-functions decision.
- **D-C9-NEW-SUITE.** Outbound tests live in a new `ShimCoreSend` suite, mirroring how cycles 1–8's poll-driven tests live in `ShimCorePoll` and observer-state tests live in `ShimCoreObservers`. Determinism stays in `ShimCoreDeterminism`. Suite name is the SUT-aspect, test name is the behavior-sentence — same pattern as the existing four suites.

---

## Test fixtures

Cycle 9 adds NO new helpers. The existing
`tests/backend/tier1/test_helpers.h` already provides
`valid_can_frame`, `valid_can_frame_batch`, and
`active_prefix_bytes(const can_frame_batch&)`, all introduced in
cycle 4. The shim test consumes them from the same header.

Two ergonomic helpers in `shim_core_test.cpp`'s anonymous
namespace will likely emerge during implementation; they are
**not part of the contract** but the plan flags them so the
implementer doesn't reinvent them per test:

```cpp
// Drains the shim's outbound boot envelope from the core peer's
// inbound side, leaving backend_to_core empty so the shim's first
// post-boot send can take the lane. Mirrors the *first half* of
// complete_handshake() but does NOT send boot_ack — useful for
// C9-4 (D-C9-NO-CONNECT-GATE).
inline void drain_boot_only(tier1_endpoint& core) {
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
}

// Pulls one envelope from the shim's backend_to_core lane via the
// core peer and returns the receiver-owned tier1_message. The
// caller asserts on it. Used by every C9 test that inspects the
// published wire bytes.
inline tier1_message receive_from_shim(tier1_endpoint& core) {
  auto msg = core.try_receive();
  EXPECT_TRUE(msg.has_value());
  return std::move(*msg);
}
```

These are local to the test TU, not exported.

---

## Determinism notes

Single-threaded same-process tests. No sleep, no real time, no
thread spawn, no RNG. Same inputs → byte-identical outbound lane
state across two runs (C9-8 pins this explicitly).

The variable-size active-prefix discipline interacts with
determinism in a subtle way: the **bytes past the active prefix**
in the source `can_frame_batch{}` (i.e. `frames[count..63]` and
the per-frame trailing padding bytes within the unused slots) are
**not** transmitted, so they cannot affect determinism on the
wire. But the bytes WITHIN the active prefix (the per-frame
3-byte trailing padding inside `frames[0..count-1]`) ARE
transmitted, and source-side `can_frame{}` zero-init covers them
deterministically. C9-8's `std::memcmp` over the wire payload's
active prefix pins this — a "shim picks up garbage bytes from
uninit stack" bug would diverge between the two setups.

---

## Proposed tests (revision 1)

### C9-1. `ShimCoreSend.PublishesCanFrameBatchAsTickBoundaryWithActivePrefixWireBytes`

- **Layer / contract:** `send_can_frame_batch` publishes a
  well-framed `tick_boundary` envelope carrying the input batch
  via D-C9-ACTIVE-PREFIX-OUT.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make(endpoint, desc, /*sim_time_us=*/0)`.
  - Core peer drains the boot envelope and sends `boot_ack` at sequence 0.
  - Shim `poll()` to consume the boot_ack, flipping `is_connected() == true` and advancing inbound expected sequence to 1.
  - Construct a 3-frame batch:
    `valid_can_frame_batch(std::array{
        valid_can_frame(/*message_id=*/0x101, /*timestamp_us=*/1'000, /*data_size=*/4, /*fill_byte=*/0xA1),
        valid_can_frame(0x202, 2'000, 8, 0xB2),
        valid_can_frame(0x303, 3'000, 0, 0xC3)})`.
- **Action:** `auto sent = shim.send_can_frame_batch(batch, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - The core peer's `try_receive()` returns a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::can_frame_batch`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1` (boot took 0).
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 4 + 3 * 20 == 64` (the active-prefix size).
    - `payload.size() == 64`.
    - `std::memcmp(payload.data(), &batch, 64) == 0`. **Why memcmp over the active prefix specifically and not just over `bytes_of(batch)`:** sending the full 1284 bytes of the batch would also satisfy "payload contains the active prefix at the front," but it would fail the validator's count-vs-length contract. The exact-64 byte size check is what catches the "shim sends sizeof regardless of count" bug.
- **Bug class:** missing-publish; wrong envelope kind (e.g. shim emits `boot` again, or `on_demand_request`, or `shutdown`); wrong schema id; sequence-counter not initialized to "post-boot=1"; sim_time hardcoded to 0; **active-prefix vs sizeof confusion** (this is the cycle's headline bug class — the validator inside `build_envelope` would reject sizeof-bytes for a count=3 batch, but a "validator is too permissive" regression could let it through silently and a separate sender→receiver mismatch would result).

### C9-1b. `ShimCoreSend.PublishesEmptyCanFrameBatchAsHeaderOnlyTickBoundary`

- **Layer / contract:** D-C9-EMPTY-BATCH-OUT. count=0 is a legal outbound publish.
- **Setup:** identical to C9-1 through the boot+boot_ack handshake. Construct an empty batch via `valid_can_frame_batch()` (default-arg empty span).
- **Action:** `shim.send_can_frame_batch(empty_batch, /*sim_time_us=*/250'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - Core peer receives a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::can_frame_batch`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1` (boot took 0).
    - `envelope.sim_time_us == 250'000`.
    - `envelope.payload_bytes == 4` (just the `count` word) and `payload.size() == 4`.
  - `std::memcmp(payload.data(), &empty_batch, 4) == 0` — the `count = 0` word lands on the wire byte-for-byte.
- **Bug class:** "shim refuses count=0 because of an off-by-one assumption that 'empty batches are nonsense'" — semantically they're a valid heartbeat. The validator allows `count=0` (capacity check uses `>` not `>=`); the shim must too.

### C9-2. `ShimCoreSend.SecondSendAdvancesOutboundSequenceToTwo`

- **Layer / contract:** D-C9-SEQUENCE-ADVANCE positive arm.
- **Setup:** identical to C9-1. After the first `send_can_frame_batch` succeeds, the core peer drains the published envelope to free the outbound lane, and the shim sends a second batch with bit-distinct field values.
- **Concrete payload values:**
  - First batch: 2 frames, message_ids `{0x111, 0x222}`, fill_byte `{0xAA, 0xBB}`.
  - Second batch: 4 frames, message_ids `{0x333, 0x444, 0x555, 0x666}`, fill_byte `{0xCC, 0xDD, 0xEE, 0xFF}`. Different count from first so a "shim treats sequence advance as count-based" bug fails.
- **Action:**
  - `shim.send_can_frame_batch(first, 100'000); core.try_receive(); shim.send_can_frame_batch(second, 200'000);`
- **Expected outputs:**
  - Both sends `has_value() == true`.
  - First received envelope: `sequence == 1`, `payload_bytes == 4 + 2*20 == 44`.
  - Second received envelope: `sequence == 2`, `payload_bytes == 4 + 4*20 == 84`, `sim_time_us == 200'000`.
  - Full-active-prefix byte-equality on both envelopes' payloads against the source batches.
- **Bug class:** sequence counter doesn't advance (would wedge any peer); sequence counter advances by more than 1 (e.g. by `payload_bytes`); sequence counter resets between sends.

### C9-3. `ShimCoreSend.LaneBusySendIsRejectedAndPreservesSequenceCounter`

- **Layer / contract:** D-C9-SEQUENCE-ADVANCE negative arm + D-C9-WRAPPED-SEND-ERROR.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make` (boot envelope sent into `region.backend_to_core`, lane state `full`).
  - Core peer does NOT drain the boot envelope.
  - Shim has not polled yet; `is_connected() == false`.
- **Action:** `auto first_attempt = shim.send_can_frame_batch(valid_can_frame_batch(std::array{valid_can_frame(0x101, 1'000, 4, 0xA1)}), /*sim_time_us=*/100'000);`
- **Expected outputs:**
  - `first_attempt` returns
    `shim_error{shim_error_kind::send_failed,
                transport_error = tier1_transport_error{kind = lane_busy, ...},
                offending_field_name = "lane",
                ...}`.
  - The lane state is still `full` (boot envelope unchanged); the lane payload bytes still byte-equal the boot envelope (no shim has clobbered them).
  - Now drain the boot via `core.try_receive()`. Send the same batch again: `shim.send_can_frame_batch(same_batch, 100'000)` returns `has_value() == true`. The second attempt's published envelope has `sequence == 1` (NOT 2). **This is what proves D-C9-SEQUENCE-ADVANCE's negative arm:** if the failed first attempt had advanced the counter, the second would publish at sequence 2 and the core peer's session would reject it as `sequence_mismatch`.
- **Bug class:** failed sends burn sequence numbers (fatal — every subsequent send wedges the channel); failed sends silently clobber the lane (would corrupt the boot envelope mid-handshake); failed sends return success.

### C9-3b. `ShimCoreSend.LaneInProgressSendIsRejectedAndPreservesSequenceCounter`

- **Layer / contract:** D-C9-SEQUENCE-ADVANCE negative arm + D-C9-WRAPPED-SEND-ERROR for the `lane_in_progress` transport-error variant. Outbound mirror of cycle-1 test 13 (which covered the inbound `poll()` analogue).
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`.
  - Core peer drains boot, sends `boot_ack`, shim polls (now connected; `is_connected() == true`; outbound lane empty).
  - Test directly sets `region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::writing), std::memory_order_release)` to simulate a partially-completed write that crashed mid-way (a state no live peer can produce in single-process tests, only reachable across a crashed-mid-publish multi-process scenario; this is the same synthetic-state technique used by cycle-1 test 13 for the inbound analogue).
- **Action:** `auto attempt = shim.send_can_frame_batch(valid_can_frame_batch(std::array{valid_can_frame(0x101, 1'000, 4, 0xA1)}), 100'000);`
- **Expected outputs:**
  - `attempt` returns
    `shim_error{shim_error_kind::send_failed,
                transport_error = tier1_transport_error{kind = lane_in_progress, ...},
                offending_field_name = "state",
                ...}`. The `offending_field_name` value comes from `shared_memory_transport.cpp:251-253`'s `make_error` for the `lane_in_progress` arm.
  - `region.backend_to_core.state` is still `writing` (the synthetic state we planted; the shim must not have clobbered it back to `empty` or advanced it to `full`).
  - Recovery proof: now manually reset the lane state back to `empty` and call `shim.send_can_frame_batch(same_batch, 100'000)`. The recovery send succeeds at `sequence == 1` (NOT 2). This is what proves D-C9-SEQUENCE-ADVANCE's negative arm holds for the `lane_in_progress` failure mode the same way C9-3 proves it for the `lane_busy` failure mode.
- **Bug class:** failed sends burn sequence numbers on the `lane_in_progress` path even when they don't on the `lane_busy` path (a real risk if the shim's send path branches on the transport error kind and only one branch is correct); shim swallows the `lane_in_progress` error or rewrites it as a generic `send_failed`-with-no-transport-error (would lose the underlying transport diagnostic, mirroring cycle-1 test 13's bug class for the inbound side).

### C9-4. `ShimCoreSend.PublishesBeforeBootAckIsReceivedSinceOutboundDoesNotGateOnConnect`

- **Layer / contract:** D-C9-NO-CONNECT-GATE.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make` (boot sent).
  - `drain_boot_only(core)` — boot envelope removed, but the core peer does NOT send `boot_ack`. Shim has not polled. Therefore `is_connected() == false`.
- **Action:** `auto sent = shim.send_can_frame_batch(valid_can_frame_batch(std::array{valid_can_frame(0x101, 1'000, 4, 0xA1)}), 100'000);`
- **Expected outputs:**
  - `sent.has_value() == true`.
  - Core peer's `try_receive()` returns the published `tick_boundary` envelope at `sequence == 1` with the active-prefix payload.
  - `shim.is_connected() == false` (still — outbound publishing did not flip it).
- **Bug class:** "shim author defensively requires `is_connected()` before allowing outbound" — would break the protocol's bidirectional-handshake-overlap allowance and block any outbound traffic until the inbound `boot_ack` arrives. The protocol explicitly permits both peers to begin streaming outbound `tick_boundary` after sending their handshake half (`protocol_session.cpp:162-166` only checks `has_sent_boot_`).

### C9-5. `ShimCoreSend.PostShutdownSendIsRejectedWithoutTouchingLane`

- **Layer / contract:** D-C9-SHUTDOWN-TERMINAL.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`. (Boot envelope now in `region.backend_to_core`, lane state `full`.)
  - Core peer drains boot via `core.try_receive()`. (`region.backend_to_core` now `empty`.)
  - Core peer sends `boot_ack` at sequence 0. (`region.core_to_backend` now `full`.)
  - Shim polls. (`is_connected() == true`; `region.core_to_backend` now `empty`.)
  - Core peer sends a `shutdown` envelope at sequence 1. (`region.core_to_backend` now `full` again.)
  - Shim polls. (`is_shutting_down() == true`; `region.core_to_backend` now `empty`.)
- **Pre-action assertions (precondition the test depends on):**
  - `ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire), static_cast<std::uint32_t>(tier1_lane_state::empty))` — outbound lane is empty (the shim has not sent anything since boot, and boot was drained in step 2 above). This makes the post-action `state == empty` assertion meaningful: if the shim's short-circuit accidentally called `endpoint_.send`, the lane would flip from `empty` to `full` (or to `writing` mid-publish), and the post-action assertion would catch it. **Why explicit precondition:** without this assertion, a future refactor that leaves the boot envelope in the lane (e.g. by removing the boot-publish-on-make contract) would silently invalidate the test's claim — the post-action `state == empty` assertion would then be checking a tautology against a buggy precondition.
  - `ASSERT_TRUE(shim.is_shutting_down())` — terminal state confirmed before the action.
- **Action:** `auto sent = shim.send_can_frame_batch(valid_can_frame_batch(std::array{valid_can_frame(0x101, 1'000, 4, 0xA1)}), 250'000);`
- **Expected outputs:**
  - `sent` returns
    `shim_error{shim_error_kind::shutdown_already_observed,
                transport_error = std::nullopt,
                offending_field_name = "kind",
                ...}`. **`transport_error` is nullopt** because the rejection is shim-internal — `endpoint_.send` was never called.
  - `region.backend_to_core.state == empty` (unchanged from precondition).
  - The shim's outbound sequence counter (only observable through the next post-shutdown-undo… which doesn't exist) is preserved at 1. **Verification path:** since there's no observer, this is pinned indirectly. The precondition `state == empty` plus the post-action `state == empty` plus the `transport_error == nullopt` together prove `endpoint_.send` was never called — and `endpoint_.send` is the only path that mutates the session's send counter. So the counter is preserved by construction. No additional assertion needed.
- **Bug class:** shim continues publishing past peer-signaled shutdown (wasted bytes; clouds shim-is-dead semantic); shim's short-circuit accidentally still calls `endpoint_.send` (would either succeed and publish a now-meaningless envelope, or fail and corrupt the error type to `send_failed` instead of `shutdown_already_observed`); shim's short-circuit returns the wrong typed kind (`send_failed` instead of `shutdown_already_observed`) which would mask the terminal-state semantic.

### C9-6. `ShimCoreSend.SuccessfulSendDoesNotMutateAnyInboundCacheSlot`

- **Layer / contract:** D-C9-INBOUND-INDEPENDENCE.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`.
  - Core peer drains boot, sends `boot_ack`, shim polls.
  - Pre-assertion: all 8 cache observers (`latest_clock_state()` through `latest_error_message_batch()`) return `nullopt`. **`ASSERT_TRUE`** on each, not just `EXPECT_TRUE` — if any is non-null at this point the rest of the test is meaningless.
- **Action:** `shim.send_can_frame_batch(valid_can_frame_batch(std::array{valid_can_frame(0x101, 1'000, 4, 0xA1)}), 250'000);`
- **Expected outputs after the send:**
  - All 8 cache observers still return `nullopt`. (One `EXPECT_FALSE(latest_X().has_value())` per slot — eight assertions total. Mirrors the cycles 2–8 cross-population family in shape.)
  - The send returned `has_value() == true`.
- **Bug class:** copy-paste-from-`poll()` bug where `send_can_frame_batch`'s implementation accidentally writes into `latest_can_frame_batch_` (the inbound cache) — a real risk because the field name mirrors the schema name, and a future C HAL ABI cycle that wires `HAL_CAN_SendMessage` into `send_can_frame_batch` would silently treat the just-sent batch as if it had been received from sim. The "send poisons receive cache" bug class is exactly the kind of seam violation cycle 9 must rule out before the C HAL layer arrives.

### C9-7. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundCanFrameBatch`

- **Layer / contract:** non-negotiable #5 (deterministic and replayable). Outbound mirror of `RepeatedRunsProduceByteIdenticalAllEightSlots` (C8-6).
- **Setup:** two independent setups (two regions, two shim+core pairs) run the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core peer drains boot, sends `boot_ack` at sequence 0, shim polls.
  - Shim sends a 3-frame batch (bit-distinct field values per frame) at `sim_time_us = 250'000`. Concrete:
    `valid_can_frame_batch(std::array{
        valid_can_frame(0x101, 1'000, 4, 0xA1),
        valid_can_frame(0x202, 2'000, 8, 0xB2),
        valid_can_frame(0x303, 3'000, 0, 0xC3)})`.
  - Core peer drains the published envelope into a `tier1_message`.
  - Shim sends a second 1-frame batch (different count, different fields) at `sim_time_us = 500'000`.
  - Core peer drains the second published envelope.
- **Expected outputs:**
  - For both setups: both sends `has_value() == true`; both received envelopes have `has_value() == true`.
  - Setup A's first received envelope is byte-identical to setup B's first received envelope:
    - `setup_a.first.envelope == setup_b.first.envelope` via defaulted `sync_envelope::operator==`.
    - `setup_a.first.payload == setup_b.first.payload` via `std::vector<std::uint8_t>::operator==`.
    - `std::memcmp(setup_a.first.payload.data(), setup_b.first.payload.data(), setup_a.first.payload.size()) == 0` — byte-level companion that pins the per-frame trailing-padding bytes (3 bytes per `can_frame`, 9 bytes total for a 3-frame batch) inside the active prefix. **Why memcmp in addition to vector ==:** `std::vector::operator==` compares element-by-element using `std::uint8_t::operator==`, which IS byte-equal — the memcmp is technically redundant. Kept for explicit "padding bytes are pinned" documentation symmetry with C8-6's memcmp companions on the inbound side.
  - Same byte-identity check for the second received envelope.
- **Bug class:** any seam-level nondeterminism on the outbound path: shim including uninitialized stack bytes in its envelope build; shim reading wall-clock; per-frame padding bytes diverging because of ABI surprise on a porting target; sequence counter starting from a non-zero memory-residue value; sim_time_us getting clobbered by uninit. Mirror of C8-6's inbound determinism guarantee, now extended to the outbound path.

### C9-8. `ShimCoreSend.SendDoesNotPerturbProtocolSessionsReceiveExpectedSequence`

- **Layer / contract:** session-side independence — outbound mutates only the send-side counter, never the receive-side counter.
- **Setup:**
  - Fresh region; backend endpoint; shim via `make`.
  - Core peer drains boot, sends `boot_ack` at sequence 0, shim polls.
  - At this point the shim's session has `next_expected_receive_sequence == 1` (boot_ack consumed seq 0).
- **Action:**
  - Shim sends three batches in succession, draining each via the core peer between sends to free the lane.
  - Then the core peer sends a valid `clock_state` `tick_boundary` envelope at sequence 1. Shim polls.
- **Expected outputs:**
  - All three outbound sends succeed.
  - The post-send `poll()` accepts the `clock_state` envelope at sequence 1 — `latest_clock_state().has_value() == true` and equals `valid_clock_state(/*sim_time_us=*/100'000)`. **Verification path:** the receive-side session counter is not directly observable through the shim's public surface, so this test verifies it indirectly by sending a valid envelope at sequence 1 and asserting the shim accepts it. A "shim's send path mutates the receive counter" bug would either cause the inbound poll to expect a different sequence (rejecting with `sequence_mismatch`) or mis-cache.
- **Bug class:** the shim's send implementation accidentally calls `accept_envelope` instead of `build_envelope`, or otherwise mutates session receive state. This is a real risk because the protocol_session has both `next_sequence_to_send_` and `next_expected_receive_sequence_` fields and a copy-paste implementation could grab the wrong one. Cycle 9 must rule out the receive-counter perturbation before any future cycle stacks more send paths on top.

---

## Test rewrites

None. Cycles 1–8 are inbound-only; cycle 9 introduces a new
public surface and a new test suite (`ShimCoreSend`). No existing
test needs revision or replacement.

---

## Cross-test invariants

- Every test constructs one shim per test (no shared state).
- All `sim_time_us` values are explicit constants. No wall-clock use.
- All payload comparisons against the published wire bytes use either:
  - full-active-prefix `std::memcmp` (C9-1, C9-1b, C9-2), OR
  - defaulted `sync_envelope::operator==` plus vector-byte equality plus an explicit `std::memcmp` companion (C9-7).
- The two ergonomic helpers (`drain_boot_only`, `receive_from_shim`) live in the test TU's anonymous namespace, not in `tests/backend/tier1/test_helpers.h`. Promoting them to the shared header is a future cycle's call once 2+ outbound test TUs exist.
- All `has_value()` checks on lane drain operations and pre-condition handshake steps are `ASSERT_TRUE`, not `EXPECT_TRUE` — a failed setup invalidates the test's claim, not just adds a spurious failure.

---

## What this plan does NOT cover

- The other two outbound-meaningful schemas (`send_notifier_state`, `send_error_message_batch`). Each is a future cycle following C9's pattern.
- The five inbound-only schemas as outbound API. The typed-method pattern per D-C9-TYPED-OUTBOUND makes them inaccessible by design; no test enumerates them.
- `on_demand_request` / `on_demand_reply` envelopes. Future cycle.
- Shim-initiated `shutdown`. Future cycle. Note that cycle 9's D-C9-SHUTDOWN-TERMINAL handles **inbound** shutdown's effect on the outbound path; emitting `shutdown` from the shim is a separate concern.
- The C HAL ABI surface. `send_can_frame_batch` is a C++ method; `HAL_CAN_SendMessage` is a separate cycle that batches per-frame calls and flushes via this method at tick boundaries.
- Validity-domain checks on `can_frame` fields (e.g. `data_size <= 8`, `message_id` ID-range). The schema validator does not check these; the protocol contract is "framing only, element bodies are receiver scope." If the sim core needs domain validation it lives there, not in the shim's outbound path.
- **Outbound `count > kMaxCanFramesPerBatch` (count > 64).** The `can_frame_batch::count` field is a `std::uint32_t` and is settable by callers, so a hostile or buggy caller could pass a `can_frame_batch{}` with `batch.count = 65`. The shim's inlined active-prefix size computation (`offsetof(...) + count * sizeof(can_frame)`) would then read past `batch.frames[63]` into out-of-bounds memory, AND `protocol_session::build_envelope`'s inlined call to `validate_envelope` would feed a payload span longer than `sizeof(can_frame_batch)` to the validator's `expected_variable_payload_size`, which would reject it as `payload_size_mismatch` (count > capacity per `validator.cpp:105-110`). This is an upstream-validator concern that cycle 9's shim does not need its own coverage for — `protocol_test.cpp`'s capacity-rejection cases already pin the validator behavior. **However**, the shim's inlined active-prefix computation reading past `batch.frames[63]` is undefined behavior that the validator rejection would surface only after the read had already happened. Cycle 9's contract is "caller passes a well-constructed batch"; the C HAL ABI cycle that wires `HAL_CAN_SendMessage` will own the count-clamping discipline at the seam where untrusted caller data enters. Documenting the gap here so the C HAL cycle's test plan picks it up.
- Threading. Single-threaded; caller drives both poll and send in alternation.
- Reset / reconnect. Out per cycle 1.
- Inbound CAN RX queueing semantics. Per D-C4-LATEST-WINS, deferred until `HAL_CAN_ReadStreamSession` lands. Cycle 9 does not touch the inbound `latest_can_frame_batch_` slot.

---

## Implementation companion recommendations (non-test)

These are not test plan items but should land in the same cycle:

- **The `wrap_send_error` lambda's message string** in `shim_core.cpp:10-16` is currently `"transport rejected outbound boot envelope"` — call-site-specific. Cycle 9's two send call sites (boot in `make()`, tick_boundary in `send_can_frame_batch`) both want to reuse it. Per D-C9-WRAPPED-SEND-ERROR: change the message to the generic `"transport rejected outbound envelope"`. Test 2 (`MakeFailsWhenBackendLaneAlreadyFullAndDoesNotClobberLane`) currently does NOT assert on the exact message string — it only asserts on `kind` and `transport_error.kind` — so the rename is test-safe. Verify by reading `shim_core_test.cpp:101-127`.
- **The `[[nodiscard]]` attribute** on `send_can_frame_batch` is mandatory; a caller that ignores the returned `expected` would silently lose lane-busy / shutdown errors. Same as `make()` and `poll()`.
- **Doc comment on `send_can_frame_batch`** must call out the active-prefix discipline and the `tick_boundary`-only envelope kind, since those are non-obvious from the signature. The example comment in "New public surface" above is the proposed text.
- **No new fields on `shim_core`.** The send path is stateless within the shim; all state lives in the wrapped `tier1_endpoint` (and transitively its `protocol_session`).

---

## Open questions

**OQ-C9-WRAPPER-SHARED-VS-SPLIT** — D-C9-WRAPPED-SEND-ERROR picks
"single shared `wrap_send_error` with a generic message." The
alternative is two wrap functions, one per call site, each with
its own message. The single-function path is what the plan is
written against; reviewer push-back welcome if they think the
two-function path is better (e.g. for diagnostic clarity in logs
where the typed `kind` alone might not say "this came from
send_can_frame_batch vs boot").

**OQ-C9-METHOD-NAME** — `send_can_frame_batch` vs
`publish_can_frame_batch` vs `send_tick_can_frame_batch`. The
plan picks `send_can_frame_batch` for parity with the underlying
tier1 endpoint's `send` method, accepting that the implicit
envelope-kind choice (`tick_boundary`) lives in the doc comment
rather than the name. When on-demand cycles land, their
request/reply method shape returns a different type, so naming
collision is a non-issue. Reviewer push-back welcome on the
naming.

**OQ-C9-FIRST-OUTBOUND-SCHEMA** — cycle 9 picks
`can_frame_batch` over `notifier_state` and
`error_message_batch` because (a) it's the most-frequent
outbound path on a real robot, (b) it exercises the variable-
size active-prefix discipline (the headline contract for this
class of cycles), and (c) `kMaxCanFramesPerBatch` and the
existing test fixtures are mature. The other two outbound
schemas follow this cycle's pattern; reviewer push-back welcome
if they prefer one of them as the seam-establishing first cycle
(would only swap which schema gets the cycle-9 slot vs cycle-10
or -11 slot).

**OQ-C9-EMPTY-BATCH-SEMANTIC** — C9-1b accepts count=0 as a
legal publish. The alternative is "shim refuses count=0 because
no caller has a use for it; YAGNI." The plan picks accept-it
because (a) the validator already accepts it, (b) it's a
legitimate "I have nothing this tick but my heartbeat counter
should advance" case the C HAL ABI cycle may want, and (c)
adding a guard now and removing it later is more churn than
just accepting it. Reviewer push-back welcome — could go either
way.

---

## Revision rationale (round 1 → round 2)

Round-1 verdict was `not-ready` (2026-04-29) with two blockers
plus three non-blocking items, plus a systemic off-by-one error
in the decision cross-references that the reviewer flagged for
one decision (D-C9-INBOUND-INDEPENDENCE) but that I subsequently
found also affected three other decisions. Rev 2 addresses all
six items.

### Blockers (both addressed)

- **B1 — C9-5 setup logical error.** Round-1 plan said "Core
  peer drains the next backend_to_core lane state (it should
  still be `empty` — shim has not sent anything since boot, so
  the lane is empty already)." The "drain" step was contradictory
  with the parenthetical: the lane IS already empty at that
  point (boot was drained earlier in setup), so a faithful
  `core.try_receive()` call would return `no_message` and an
  `ASSERT_TRUE(msg.has_value())` would fail in setup. **Rev 2
  fix:** removed the "drain" step. Replaced with an explicit
  precondition assertion block (`ASSERT_EQ(region.backend_to_core
  .state.load(), empty)` and `ASSERT_TRUE(shim.is_shutting_down())`)
  before the action, plus an explicit walkthrough of the lane-
  state transitions through each setup step. Documented the
  precondition's purpose (catches a future "boot envelope leaks
  past make()" refactor that would invalidate the post-action
  `state == empty` assertion).

- **B2 — D-C9-INBOUND-INDEPENDENCE cross-ref off-by-one.**
  Decision text said "Pinned by C9-7" but the test is the sixth
  test (C9-6); C9-7 is the determinism test. **Rev 2 fix:** all
  four off-by-one decision cross-refs corrected (D-C9-SEQUENCE-
  ADVANCE was C9-3/C9-4 → now C9-2/C9-3/C9-3b; D-C9-NO-CONNECT-
  GATE was C9-5 → now C9-4; D-C9-SHUTDOWN-TERMINAL was C9-6 →
  now C9-5; D-C9-INBOUND-INDEPENDENCE was C9-7 → now C9-6). The
  reviewer caught one of the four; auditing the rest revealed
  the systemic miscount (likely caused by adding C9-1b mid-draft
  and not propagating the renumber to the decision references).

### Non-blocking items (all addressed)

- **NB1 — C9-1b missing explicit `sequence == 1` assertion.**
  Round-1 C9-1b only asserted on payload bytes. **Rev 2 fix:**
  expanded C9-1b's expected-output block to mirror C9-1's full
  envelope-metadata pin set (kind, payload_schema, sender,
  sequence, sim_time_us, payload_bytes, plus the memcmp on the
  4-byte payload).

- **NB2 — `lane_in_progress` gap on outbound.** Round-1 covered
  `lane_busy` (C9-3) but not the `writing`-state failure mode.
  **Rev 2 fix:** added C9-3b
  (`LaneInProgressSendIsRejectedAndPreservesSequenceCounter`),
  the outbound mirror of cycle-1 test 13 for the inbound side.
  Uses the same direct-state-mutation technique (`region.<lane>
  .state.store(writing)`) and the same recovery proof (manually
  reset to `empty`, send same batch, assert sequence == 1).
  D-C9-SEQUENCE-ADVANCE's pin set updated to include C9-3b.

- **NB3 — Outbound `count > kMaxCanFramesPerBatch` documentation
  gap.** Round-1 documented this for the inbound path but not
  the outbound. **Rev 2 fix:** added a paragraph to "What this
  plan does NOT cover" calling out the upstream-validator
  coverage, the shim's inlined-active-prefix UB risk if a
  hostile caller passes count=65, and the C HAL ABI cycle's
  responsibility to own the count-clamping discipline at the
  untrusted-caller seam.

### What rev 2 does not change

- The 9 tests' overall structure, suite naming (`ShimCoreSend`),
  decision pin set, and contract scope. The reviewer approved
  the substantive design.
- The four open questions (OQ-C9-WRAPPER-SHARED-VS-SPLIT, OQ-C9-
  METHOD-NAME, OQ-C9-FIRST-OUTBOUND-SCHEMA, OQ-C9-EMPTY-BATCH-
  SEMANTIC). Reviewer did not push back on any.
- D-C9-NO-HELPER (the "inline active-prefix size in
  `send_can_frame_batch`, defer extraction to the second outbound
  schema") — reviewer explicitly endorsed.
- D-C9-WRAPPED-SEND-ERROR (the shared `wrap_send_error` with
  generic message) — reviewer verified the existing test 2's
  assertions don't pin the message string.
