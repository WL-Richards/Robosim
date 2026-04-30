---
name: hal-shim
description: Use when working on the in-process HAL shim (`src/backend/shim/`) — the orchestrator the user's robot binary loads to talk to the Sim Core. Through cycle 11: boot handshake, inbound `clock_state` / `power_state` / `ds_state` / `can_frame_batch` / `can_status` / `notifier_state` / `notifier_alarm_batch` / `error_message_batch` cache slots (the four variable-size — `can_frame_batch`, `notifier_state`, `notifier_alarm_batch`, `error_message_batch` — use active-prefix memcpy with zero-init-before-write semantics; the rest are fixed-size byte-copy), shutdown terminal receive path, **plus the outbound `send_can_frame_batch` (cycle 9), `send_notifier_state` (cycle 10), and `send_error_message_batch` (cycle 11) surfaces — all three outbound-meaningful schemas wired — publishing as `tick_boundary` envelopes into `backend_to_core` via the production `active_prefix_bytes` overloads in the schema headers (extracted at cycle 10 per D-C10-EXTRACT-ACTIVE-PREFIX), with the same shutdown-terminal short-circuit as `poll()` per typed method**. All 8 per-tick payload schemas are wired inbound; **all three outbound-meaningful schemas wired** (`can_frame_batch`, `notifier_state`, `error_message_batch` — the v0 outbound surface is closed). Does not yet cover the C HAL ABI (`HAL_GetFPGATime`, `HAL_GetVinVoltage`, `HAL_GetControlWord`, `HAL_CAN_ReadStreamSession`, `HAL_CAN_GetCANStatus`, `HAL_Notifier*`, `HAL_SendError`, etc.), on-demand request/reply, shim-initiated shutdown, CAN RX queueing semantics (deferred to whichever cycle wires the C HAL CAN consumer), threading, or reset/reconnect.
---

# HAL shim core

The in-process Layer-2 component the user's robot binary will load
(eventually as `libhal.so`). Sits *underneath* the WPILib HAL surface;
the robot code is unmodified. v0 cycle 1 implemented the shim's core
orchestrator (boot handshake, post-boot inbound drain loop, the
inbound `clock_state` cache slot, and the shutdown terminal receive
path); cycle 2 added the inbound `power_state` cache slot as a peer
of `clock_state`; cycle 3 added the inbound `ds_state` cache slot as
a third peer, with explicit padding-byte determinism pinned via
`std::memcmp`; cycle 4 added the inbound `can_frame_batch` cache slot
as a fourth peer — **the first variable-size schema** (active-prefix
memcpy with zero-init before every write so a shrinking batch leaves
unused frame slots byte-zero rather than stale; per-frame padding-
byte determinism extends D-C3-7); cycle 5 added the inbound
`can_status` cache slot as a fifth peer — back to the fixed-size,
no-padding shape of cycles 1/2/3; cycle 6 added the inbound
`notifier_state` cache slot as a sixth peer — **the second
variable-size schema** with the most implicit padding of any cached
schema so far (the 4-byte interior `count → slots` pad pinned by
`offsetof(notifier_state, slots) == 8`, plus 4 trailing pad bytes
per `notifier_slot` × 32 slots = 132 padding bytes total, all
covered by zero-init-before-write); cycle 7 added the inbound
`notifier_alarm_batch` cache slot as a seventh peer — **the third
variable-size schema**, sharing the 8-byte header layout of
`notifier_state` but with a *named* `reserved_pad` field per
`notifier_alarm_event` (so each event has no implicit padding; only
the 4-byte interior `count → events` pad in the batch is implicit);
cycle 8 added the inbound `error_message_batch` cache slot as the
**eighth and final** per-tick peer — the fourth variable-size
schema, distinguished by having **zero implicit C++ padding** (its
4-byte interior between `count` and `messages` is a *named*
`reserved_pad[4]` field, and `error_message`'s 3-byte gap after
`truncation_flags` is a *named* `reserved_pad[3]` field; defaulted
`operator==` covers every byte). Cycle 8 also retired test 10
(D-C8-PROBE-RETIRED) since no per-tick schema remains unsupported
and closed a pre-existing C7-6 determinism gap where
`notifier_alarm_batch` had been inadvertently omitted from the
"AllSevenSlots" replay despite the test name. **Cycle 9 added
the first outbound-past-boot surface: `send_can_frame_batch`
publishes a `can_frame_batch` payload as a `tick_boundary`
envelope into `backend_to_core`. Inlined active-prefix size
computation (D-C9-NO-HELPER) mirrors the inbound D-C4-VARIABLE-
SIZE on the outbound side. The same shutdown-terminal short-
circuit as `poll()` (cycle-1 design decision #6) applies: once
`is_shutting_down()` is true, the send returns
`shutdown_already_observed` without calling `endpoint_.send`,
so neither the lane nor the session counter is touched. The
shared `wrap_send_error` lambda's message string was generalized
from "transport rejected outbound boot envelope" to "transport
rejected outbound envelope" so it serves both call sites
(D-C9-WRAPPED-SEND-ERROR). Cycle 10 added the second outbound-
meaningful schema (`send_notifier_state`) and cashed in
D-C9-NO-HELPER's "second-call-site" trigger by extracting the
active-prefix size computation into production `active_prefix_bytes`
overloads in the schema headers (`can_frame.h` and
`notifier_state.h`) per D-C10-EXTRACT-ACTIVE-PREFIX. Both
`send_can_frame_batch` (refactored) and `send_notifier_state`
(new) call the production helpers; the test-side overloads in
`tests/backend/tier1/test_helpers.h` for these two schemas were
deleted, and tests resolve to the production version via ADL.
Cycle 10 also explicitly trimmed per-method test mirrors of the
cycle-9 cross-cutting contracts (D-C10-INBOUND-INDEPENDENCE-
INHERITS, D-C10-NO-CONNECT-GATE-INHERITS, D-C10-RECEIVE-COUNTER-
INDEPENDENCE-INHERITS, plus the lane_in_progress mirror) on the
test-reviewer's explicit endorsement that those contracts are
session/transport-level, not per-method, and don't need re-
verification per outbound schema. Only D-C10-SHUTDOWN-TERMINAL-
INHERITS gets a per-method test (C10-5) because each typed
`send_*` method carries its own short-circuit code. Cycle 11
added the third (and final v0) outbound-meaningful schema
(`send_error_message_batch`) and added the third production
`active_prefix_bytes` overload to `error_message.h` (plus a
`static_assert(offsetof(error_message_batch, messages) == 8)`
recommended by the cycle-11 reviewer). The cycle-10 trim
convention applied unchanged. With cycle 11, the v0 outbound
surface is **closed** for the three semantically-meaningful
schemas; the five sim-authoritative schemas remain
intentionally absent from the outbound API per D-C9-TYPED-
OUTBOUND.** Future cycles cover on-demand request/reply, shim-
initiated `shutdown`, the C HAL ABI, threading, and reset/
reconnect.

## Scope

**In (cycles 1–11):**
- `shim_core::make(endpoint, desc, sim_time_us)` — moves the caller-
  owned `tier1_endpoint` into the shim and immediately publishes one
  `boot` envelope carrying `desc` via `endpoint.send`.
- `shim_core::send_can_frame_batch(batch, sim_time_us)` (cycle 9;
  refactored at cycle 10 to call the production
  `backend::active_prefix_bytes(batch)` helper instead of the inlined
  computation) — publishes `batch` as a `tick_boundary` envelope into
  the shim's outbound `backend_to_core` lane. Sends exactly the
  active prefix (`offsetof(can_frame_batch, frames) + batch.count *
  sizeof(can_frame)`). Wraps transport errors via the shared
  `wrap_send_error`. Short-circuits with `shutdown_already_observed`
  once `is_shutting_down()` is true, without touching the lane or
  the session counter.
- `shim_core::send_notifier_state(state, sim_time_us)` (cycle 10) —
  publishes `state` as a `tick_boundary` envelope into the shim's
  outbound `backend_to_core` lane. Sends exactly the active prefix
  (`offsetof(notifier_state, slots) + state.count *
  sizeof(notifier_slot)` — note the **8-byte header** offset, not 4,
  because `notifier_slot` has `alignof == 8`). Same shutdown short-
  circuit and `wrap_send_error` semantics as `send_can_frame_batch`.
  Calls the production `backend::active_prefix_bytes(state)` helper
  in `notifier_state.h`.
- `shim_core::send_error_message_batch(batch, sim_time_us)` (cycle 11)
  — publishes `batch` as a `tick_boundary` envelope into the shim's
  outbound `backend_to_core` lane. Sends exactly the active prefix
  (`offsetof(error_message_batch, messages) + batch.count *
  sizeof(error_message)`). The 8-byte header offset comes from a
  **NAMED `reserved_pad[4]` field** (not implicit padding); per
  D-C8-PADDING-FREE the schema has zero implicit C++ padding. Same
  shutdown short-circuit and `wrap_send_error` semantics. Calls the
  production `backend::active_prefix_bytes(batch)` helper in
  `error_message.h`.
- `backend::active_prefix_bytes(const can_frame_batch&)`,
  `backend::active_prefix_bytes(const notifier_state&)`, and
  `backend::active_prefix_bytes(const error_message_batch&)` —
  production helpers in `src/backend/common/can_frame.h`,
  `notifier_state.h`, and `error_message.h` respectively (extracted
  at cycle 10 per D-C10-EXTRACT-ACTIVE-PREFIX, with the third
  overload added at cycle 11). Each returns a span over the schema's
  active-prefix bytes. Tests resolve to these via ADL; the test-side
  overloads in `tests/backend/tier1/test_helpers.h` for these three
  schemas were deleted as each cycle landed. The only remaining
  test-side overload is `notifier_alarm_batch` (sim-authoritative;
  no production caller because it is never published outbound by
  the shim).
- `shim_core::poll()` — drains zero-or-one inbound message per call;
  dispatches by `(envelope_kind, schema_id)`:
  - `boot_ack` → flips `is_connected()` to true.
  - `tick_boundary` + `clock_state` → byte-copies the payload into
    the `latest_clock_state_` cache slot (cycle 1).
  - `tick_boundary` + `power_state` → byte-copies the payload into
    the `latest_power_state_` cache slot (cycle 2). Storage is
    independent of `latest_clock_state_`.
  - `tick_boundary` + `ds_state` → byte-copies the 2384-byte payload
    (including the five interior padding bytes) into the
    `latest_ds_state_` cache slot (cycle 3). Storage is independent
    of all other slots.
  - `tick_boundary` + `can_frame_batch` → zero-inits a destination
    `can_frame_batch{}`, then byte-copies exactly
    `received->payload.size()` bytes (the active-prefix length the
    `protocol_session` validator has already verified against the
    payload's leading `count` field) into it (cycle 4). The
    zero-init covers per-frame padding bytes and any frame slots
    `frames[count..63]` not in the active prefix — so a shrinking
    batch (M-frame replacing an N-frame batch with M < N) does not
    leak stale frames. Storage is independent of all other slots.
  - `tick_boundary` + `can_status` → byte-copies the 20-byte
    payload into the `latest_can_status_` cache slot (cycle 5).
    Fixed-size, no padding (5 × 4-byte fields all naturally
    aligned). Storage is independent of all other slots.
  - `tick_boundary` + `notifier_state` → zero-inits a destination
    `notifier_state{}`, then byte-copies exactly
    `received->payload.size()` bytes (the active prefix the
    `protocol_session` validator has already verified) into it
    (cycle 6). The zero-init covers the 4-byte interior `count →
    slots` pad, the 4 trailing pad bytes per `notifier_slot`, and
    any slots `slots[count..31]` not in the active prefix — so a
    shrinking batch leaves no stale slots and all 132 implicit
    padding bytes deterministically zero. Storage is independent
    of all other slots.
  - `tick_boundary` + `notifier_alarm_batch` → zero-inits a
    destination `notifier_alarm_batch{}`, then byte-copies exactly
    `received->payload.size()` bytes into it (cycle 7). The
    zero-init covers the 4-byte interior `count → events` pad and
    any events `events[count..31]` not in the active prefix.
    `notifier_alarm_event` has no implicit padding (its
    `reserved_pad` is a *named* field), so per-event padding
    determinism falls out of `operator==` directly — but C8-6's
    determinism replay (which replaced the landed-but-incomplete
    C7-6) adds a `std::memcmp` companion over
    `sizeof(notifier_alarm_batch)` (520 bytes) to pin the 4
    interior pad bytes plus all 32 event slots' contents.
    Storage is independent of all other slots.
  - `tick_boundary` + `error_message_batch` → zero-inits a
    destination `error_message_batch{}`, then byte-copies exactly
    `received->payload.size()` bytes into it (cycle 8). Unique
    among the cached schemas in having **zero implicit C++
    padding** — the 4-byte gap between `count` and `messages` is
    a *named* `reserved_pad[4]` field, and `error_message`'s
    3-byte gap after `truncation_flags` is a *named*
    `reserved_pad[3]` field. Defaulted `operator==` covers every
    byte, so C8-6's determinism replay does not add a memcmp
    companion for this schema. Zero-init still applies for the
    shrinking-batch contract (D-C8-VARIABLE-SIZE) — unused
    `messages[count..7]` stay byte-zero rather than stale.
    Storage is independent of all other slots.
  - `tick_boundary` + any other schema → returns
    `unsupported_payload_schema`. At cycle 8 the per-tick payload
    set is closed (8 schemas wired), so this branch is unreachable
    from valid traffic. Retained as a defensive forward-compat
    structural guard (D-C8-DEAD-BRANCH): a future schema added to
    `protocol_version.h` and the validator's allowed set but not
    yet wired here will fail loudly rather than silently
    discarded. The `protocol_session` has already accepted the
    framing and advanced the receive counter, so the next valid
    envelope is unaffected.
  - `shutdown` → flips `is_shutting_down()` to true.
  - any other `envelope_kind` → returns `unsupported_envelope_kind`.
- Terminal post-shutdown short-circuit: once `is_shutting_down()` is
  true, every subsequent `poll()` returns
  `shutdown_already_observed` without touching the lane.
- Observers: `is_connected()`, `is_shutting_down()`,
  `latest_clock_state()`, `latest_power_state()`,
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_can_status()`, `latest_notifier_state()`,
  `latest_notifier_alarm_batch()`,
  `latest_error_message_batch()`.

**Out:**
- The per-tick payload schema set is closed at cycle 8 (all 8
  schemas wired) on the inbound side. The
  `unsupported_payload_schema` arm is retained as a defensive
  forward-compat guard; if a post-v0 schema is added to
  `protocol_version.h` and the validator's allowed set but not
  yet wired in dispatch, the loud reject path still fires.
- **CAN RX queueing semantics.** `latest_can_frame_batch_` uses
  latest-wins (D-C4-LATEST-WINS) — a deferred decision pinned by
  the cycle-4 tests. Cycle-1 D #7 anticipated that CAN RX may
  eventually need queueing, but no live consumer drives that
  requirement yet. The cycle that wires `HAL_CAN_ReadStreamSession`
  brings the queue contract with it; until then, latest-wins is
  the explicit and tested behavior.
- The five inbound-only schemas (`clock_state`, `power_state`,
  `ds_state`, `can_status`, `notifier_alarm_batch`) are sim-
  authoritative and have no outbound API by design
  (D-C9-TYPED-OUTBOUND makes wrong-direction publishing impossible
  at the API boundary). With cycle 11, the three semantically-
  meaningful outbound schemas (`can_frame_batch`, `notifier_state`,
  `error_message_batch`) are all wired; the v0 outbound surface is
  closed.
- `on_demand_request` / `on_demand_reply` envelopes (in either
  direction). Future cycle drives the `protocol_session`'s
  single-flight pairing surface.
- Shim-initiated `shutdown` envelope. Future cycle.
- The exported C HAL ABI (`HAL_GetFPGATime`, `HAL_Initialize`,
  `HAL_GetAlliance`, `HAL_CAN_SendMessage`, `HAL_SendError`,
  `HAL_Notifier*`, etc.). Future cycle adds these as separate
  surface-files that read from the shim's cache and write into
  the shim's outbound API. The C HAL CAN TX cycle that wires
  `HAL_CAN_SendMessage` will own the per-frame-batching and
  per-tick-flush discipline that drives `send_can_frame_batch`;
  cycle 9 stops at the C++ shim API.
- Outbound oversize-count validation for both `send_can_frame_batch`
  (`count > kMaxCanFramesPerBatch == 64`) and `send_notifier_state`
  (`count > kMaxNotifiers == 32`). Both methods trust the caller to
  pass a well-constructed batch; the production
  `active_prefix_bytes` overloads read `count` directly. Upstream,
  `protocol_session::build_envelope` and the validator catch
  oversize batches with `payload_size_mismatch`, but the shim's
  read past `frames[63]` / `slots[31]` is UB that the validator
  surfaces only after the read has already happened. The C HAL ABI
  cycles (`HAL_CAN_SendMessage`, `HAL_Notifier*`) own the count-
  clamping discipline at the untrusted-caller seam.
- Threading. The shim is single-threaded; the caller drives both
  `poll()` and `send_can_frame_batch`. The C HAL ABI cycle that
  introduces multi-thread call sites also introduces the
  threading model.
- Reset / reconnect. After shutdown, the shim object is dead —
  on both the inbound (`poll()`) and outbound (`send_*`) sides.
- LD_PRELOAD libc time interception. Separate cycle.
- Tier 2 socket transport. The shim today consumes a concrete
  `tier1::tier1_endpoint`; the seam will generalize when tier 2 lands.

## Public Surface

`src/backend/shim/shim_core.h`:

- `shim_error_kind` — `send_failed`, `receive_failed`,
  `unsupported_envelope_kind`, `unsupported_payload_schema`,
  `shutdown_already_observed`.
- `shim_error` — `kind` + optional embedded
  `tier1::tier1_transport_error` (which itself optionally carries an
  embedded `session_error`) + offending field name + message.
- `shim_core::make(tier1::tier1_endpoint, const boot_descriptor&,`
  ` std::uint64_t sim_time_us)` — factory.
- `shim_core::poll()` — drain one inbound; apply.
- `shim_core::send_can_frame_batch(const can_frame_batch&,`
  ` std::uint64_t sim_time_us)` (cycle 9) — publish a
  `tick_boundary` envelope carrying the batch's active prefix.
  Refactored at cycle 10 to call `backend::active_prefix_bytes`.
- `shim_core::send_notifier_state(const notifier_state&,`
  ` std::uint64_t sim_time_us)` (cycle 10) — publish a
  `tick_boundary` envelope carrying the state's active prefix
  (8-byte header + count*88 bytes).
- `shim_core::send_error_message_batch(const error_message_batch&,`
  ` std::uint64_t sim_time_us)` (cycle 11) — publish a
  `tick_boundary` envelope carrying the batch's active prefix
  (8-byte header from named `reserved_pad[4]` + count*2324 bytes).
- `shim_core::is_connected()`, `is_shutting_down()`,
  `latest_clock_state()`, `latest_power_state()`,
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_can_status()`, `latest_notifier_state()`,
  `latest_notifier_alarm_batch()`,
  `latest_error_message_batch()` — observers.

`shim_core` is move-only; the wrapped `tier1_endpoint` is move-only.

## Design Decisions

1. **Boot publishes on construction.** `make()` is the only path that
   sends boot; if the lane is busy or the endpoint is wrong-direction,
   `make()` fails and no shim object exists. This eliminates a
   "sent boot but failed to construct" intermediate.
2. **Endpoint owned by value.** The shim takes the endpoint by value
   and moves it into a member. The caller no longer holds the
   endpoint, so no aliasing concern.
3. **Poll is nonblocking and returns success on empty lane.** Caller
   chooses the cadence; soft real-time and deterministic-replay modes
   both work. No spin, no condvar.
4. **Cycle-1 explicit reject for unsupported schemas.** Dispatching
   on `(envelope_kind, schema_id)` and returning a typed error for
   anything not yet supported is the no-shortcuts path: a future
   cycle that adds `power_state` cannot accidentally inherit a
   silent-discard. Per CLAUDE.md non-negotiable #1.
5. **Session counter advances through dispatch reject.** When the
   shim refuses a schema it doesn't yet handle, the underlying
   `protocol_session` has already accepted the framing and consumed
   the receive counter. That's intentional: the next valid envelope
   at the next sequence still works. Tested by
   `ShimCorePoll.RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext`.
6. **Shutdown short-circuits before transport.** Once shutdown is
   observed, `poll()` returns `shutdown_already_observed` immediately
   without calling `try_receive`. The lane may be left non-empty;
   that's fine — the shim is terminal. Cycle 2+ may revisit this
   when reset semantics arrive.
7. **Cache holds only the latest sim-authoritative value.** No queue
   semantics. Latest-wins is what `HAL_GetFPGATime` and
   `HAL_GetVinVoltage` and friends will need; if a future schema
   needs queueing (e.g. CAN RX), its slot gets queue semantics, not
   a layered queue under `clock_state` / `power_state`.
9. **Independent storage per cached schema.** Each accepted state
   schema lives in its own `std::optional<schema>` member; no
   variant, no schema-tagged single slot. Cycle-3's `ds_state`
   arrival never mutates `latest_clock_state_` or
   `latest_power_state_`, and vice versa. Pinned by the cycle-2
   and cycle-3 cross-slot independence and interleaved-traffic
   tests (`ClockAndPowerCachesAreIndependentlyMaintained`,
   `ClockPowerAndDsCachesAreIndependentlyMaintained`).
10. **Padding-byte determinism for variable-padding structs.**
    `ds_state` (cycle 3), `can_frame_batch` (cycle 4),
    `notifier_state` (cycle 6), and `notifier_alarm_batch`
    (cycle 7) are the cached schemas with implicit padding.
    `ds_state` has 5 interior padding bytes; `can_frame_batch`
    has 3 trailing padding bytes per `can_frame` × 64 frame
    slots; `notifier_state` has 4 interior pad bytes (the
    `count → slots` word, pinned by `offsetof(notifier_state,
    slots) == 8`) plus 4 trailing pad bytes per `notifier_slot`
    × 32 slots = 132 implicit pad bytes total;
    `notifier_alarm_batch` has just the 4-byte interior
    `count → events` pad (its `notifier_alarm_event` carries a
    *named* `reserved_pad`, not implicit padding).
    `error_message_batch` (cycle 8) has **zero implicit
    padding** — both its interior `count → messages` 4-byte
    gap and `error_message`'s 3-byte gap after
    `truncation_flags` are *named* `reserved_pad` fields, so
    defaulted `operator==` covers every byte and no `memcmp`
    companion is added. The shim's `std::memcpy` byte-copy
    preserves padding verbatim; the test fixtures' value-init
    patterns ensure source padding is zero on both runs. The
    current determinism test
    (`RepeatedRunsProduceByteIdenticalAllEightSlots`,
    introduced at cycle 8 to replace the landed-but-incomplete
    cycle-7 test) asserts `std::memcmp == 0` directly on
    `latest_ds_state()`, `latest_can_frame_batch()`,
    `latest_notifier_state()`, and
    `latest_notifier_alarm_batch()`, in addition to
    `operator==` on all 8 cached slots. `clock_state`,
    `power_state`, `can_status`, and `error_message_batch` are
    padding-free by their field layouts and don't need a
    `memcmp` companion. CLAUDE.md non-negotiable #5
    ("byte-identical logs") demands this.

11. **Variable-size schemas use active-prefix memcpy with
    zero-init before every write** (cycle 4 `can_frame_batch`,
    cycle 6 `notifier_state`, cycle 7 `notifier_alarm_batch`,
    cycle 8 `error_message_batch`).
    The dispatch arm reads `received->payload.size()` bytes
    (not `sizeof(...)`); the protocol_session validator has
    already verified the count↔length contract. The
    destination is freshly zero-initialized before every
    memcpy so a shrinking batch (M-element batch replacing an
    N-element batch with M < N) leaves
    `frames[M..63]` / `slots[M..31]` / `events[M..31]` /
    `messages[M..7]` byte-zero, not stale. The same zero-init
    also covers the 4-byte interior `count → slots`
    (`notifier_state`) / `count → events`
    (`notifier_alarm_batch`) implicit pad word that
    `operator==` cannot see. The naive "memcpy directly into
    the existing optional's storage" implementation leaks
    stale elements; pinned by C4-2b, C6-2b, C7-2b, and C8-2b.

12. **`unsupported_payload_schema` is dead code at cycle 8 —
    intentionally retained.** All 8 per-tick payload schemas
    are wired, so the fall-through return from the
    `tick_boundary` dispatch switch is unreachable from valid
    traffic. The branch stays as a defensive forward-compat
    structural guard (D-C8-DEAD-BRANCH): if a post-v0 schema
    is added to `protocol_version.h` and to the validator's
    allowed set but not yet wired in dispatch, the branch
    surfaces `unsupported_payload_schema` rather than silently
    discarding. Removing the branch would force a future
    cycle to re-add it AND its dispatch arm together — the
    "no shortcuts" non-negotiable is what keeps it in place.
    No test exercises this branch directly at cycle 8; the
    cycles-2-through-7 test-10 set covered it across the
    seven probe-schemas as they were progressively wired, and
    cycle 8 retired test 10 (D-C8-PROBE-RETIRED) since no
    probe schema remains.

12. **Latest-wins is the cache contract for every schema,
    including CAN.** Cycle-1 D #7 anticipated that CAN RX may
    eventually need queueing, but cycle 4 commits to
    latest-wins for `can_frame_batch` until a live consumer
    (`HAL_CAN_ReadStreamSession`) requires queue semantics.
    That consumer's cycle will replace the optional with a
    queue type and bring its own positive contract test;
    until then, latest-wins is explicit and tested.
8. **Move-only RAII.** `shim_core` is non-copyable, move-only. The
   destructor is implicit (the wrapped endpoint cleans up on its own).

13. **Typed outbound API per schema, not generic
    `send_tick_boundary(schema_id, ...)`** (D-C9-TYPED-OUTBOUND,
    cycle 9). `send_can_frame_batch` is a typed entry point;
    future cycles will add `send_notifier_state` and
    `send_error_message_batch` as siblings. Two reasons: (a) the
    typed method makes wrong-schema-id passing impossible at the
    call site, mirroring how `make()` takes a typed
    `boot_descriptor&` rather than `(schema_id::boot_descriptor,
    span<bytes>)`; (b) the shim's outbound surface is intentionally
    narrow — only the three semantically-meaningful outbound
    schemas get methods, and there is no API path to publish a
    sim-authoritative `clock_state` / `power_state` / `ds_state` /
    `can_status` / `notifier_alarm_batch` by accident, even though
    the validator's `kPerKindAllowedSchemas` table allows them
    under `tick_boundary` in either direction.

14. **Outbound active-prefix discipline mirrors inbound**
    (D-C9-ACTIVE-PREFIX-OUT, cycle 9). `send_can_frame_batch`
    publishes exactly `offsetof(can_frame_batch, frames) +
    batch.count * sizeof(can_frame)` bytes onto the wire, NOT
    `sizeof(can_frame_batch)`. This is the outbound mirror of
    D-C4-VARIABLE-SIZE on the inbound side, and it is what makes
    `protocol_session::build_envelope`'s payload-bytes validation
    hold (the validator's `expected_variable_payload_size` decodes
    the count header from the payload prefix and enforces
    `payload_bytes == header_size + count * element_size`). The
    size computation is **inlined** in `send_can_frame_batch` per
    D-C9-NO-HELPER — when a second outbound variable-size schema
    lands, the duplication between the inlined production path and
    the test-side `tier1::helpers::active_prefix_bytes` is the
    trigger for lifting a shared helper into production code; one
    call site is not enough to justify the abstraction (code-style.md
    "Don't add features past Phase B 'just because.'").

15. **Outbound shutdown short-circuits before transport**
    (D-C9-SHUTDOWN-TERMINAL, cycle 9). Once `is_shutting_down()`
    is true, `send_can_frame_batch` returns
    `shutdown_already_observed` without calling `endpoint_.send`.
    Mirror of `poll()`'s short-circuit (cycle-1 design decision
    #6). Rationale: the terminal-state non-negotiable applies
    symmetrically to send and receive; publishing wire bytes after
    the peer has signaled shutdown is wasted work and clouds the
    "shim is dead" semantic. The short-circuit returns before
    `endpoint_.send` is called, so neither the lane nor the session
    counter is touched — the test's lane-state == empty + the
    `transport_error == nullopt` assertion together prove this.

16. **Shared `wrap_send_error` with generic message**
    (D-C9-WRAPPED-SEND-ERROR, cycle 9). Cycle 9's two send call
    sites (boot in `make()`, tick_boundary in
    `send_can_frame_batch`) both consume the same
    `wrap_send_error` lambda; the message string was generalized
    from "transport rejected outbound boot envelope" to "transport
    rejected outbound envelope" so it serves both. Diagnostic
    information lives in the typed `kind` and the
    `offending_field_name` carried from the underlying transport
    error, not the human-readable message — and the existing tests
    assert on `kind` and `transport_error.kind`, never on the
    message string, so the rename was test-safe.

17. **No outbound connect-gate** (D-C9-NO-CONNECT-GATE, cycle 9).
    `send_can_frame_batch` succeeds even when `is_connected() ==
    false`. The protocol_session's `validate_send_order` only
    requires `has_sent_boot_` for backend-side non-boot sends
    (`protocol_session.cpp:162-166`); `has_received_boot_ack_` is
    not on the send-side path. The protocol explicitly permits
    both peers to begin streaming `tick_boundary` after sending
    their handshake half, without requiring the other side to ack
    first. A "shim author defensively requires `is_connected()`
    before allowing outbound" bug would block the publish; the
    protocol does not require it.

18. **Active-prefix size lives in the schema headers**
    (D-C10-EXTRACT-ACTIVE-PREFIX, cycle 10). The cycle-9
    `send_can_frame_batch` deferred extracting the
    `offsetof(B, elements) + count*sizeof(E)` computation from the
    inlined production path until a second outbound variable-size
    schema landed. Cycle 10's `send_notifier_state` is that second
    site, so the helper extracts to `inline std::span<const
    std::uint8_t> active_prefix_bytes(const X&)` overloads in the
    schema headers themselves (`can_frame.h`, `notifier_state.h`).
    Both shim send paths call `backend::active_prefix_bytes`. The
    test-side overloads in `tests/backend/tier1/test_helpers.h` for
    these two schemas were deleted; tests resolve to the production
    version via ADL since `can_frame_batch` and `notifier_state`
    live in `robosim::backend`. The `notifier_alarm_batch` and
    `error_message_batch` overloads stay in `test_helpers.h` until
    their respective outbound cycles land — "no production helper
    without a production caller" is the rule (test-reviewer
    explicitly endorsed). Extraction location: per-schema-header
    inline rather than a new aggregator header, because (a) callers
    already including the schema header get the helper for free,
    (b) the helper is a pure function of the struct it operates on,
    and (c) an aggregator would force including all four schema
    headers including the two with no production caller.

19. **Per-method test mirrors are trimmed for outbound cycles past
    cycle 9** (D-C10-INBOUND-INDEPENDENCE-INHERITS / D-C10-NO-
    CONNECT-GATE-INHERITS / D-C10-RECEIVE-COUNTER-INDEPENDENCE-
    INHERITS, cycle 10). The cycle-9 cross-cutting contracts
    (inbound-independence, no-connect-gate, receive-counter-
    independence, lane_in_progress wrapping) are session/transport-
    level rather than per-method, so each new outbound schema cycle
    does NOT re-test them. The one cycle-9 contract that DOES
    require a per-method test is the shutdown short-circuit
    (D-C10-SHUTDOWN-TERMINAL-INHERITS pinned by C10-5) because
    each typed `send_*` method carries its own short-circuit code
    block. Test-reviewer ruling: "A future reviewer should not
    revert to 10-test coverage without a concrete new bug class
    that the trim fails to catch." This is a deliberate departure
    from the inbound-cycle precedent (cycles 2–8 maintained full
    per-schema family coverage) because the outbound path does NOT
    have per-schema branching analogous to `poll()`'s dispatch arms
    — each new typed method is a thin wrapper around the shared
    `endpoint_.send` infrastructure.

## Cycle-1 control flow

```
caller                     shim_core             tier1_endpoint    region
------                     ---------             --------------    ------
make(endpoint, desc, t)
   --> store endpoint
       send(boot, …)  --> publish backend lane
           returns OK <--
   <-- shim_core
poll()
   --> try_receive   --> drain core lane
                         (or empty/error)
       dispatch by (kind, schema)
       update flags / cache
   <-- expected<void, shim_error>
```

## Validation Status

Structural feature; no physical source data applies.

- Cycle-1 plan approved by the `test-reviewer` agent on 2026-04-29
  after two review rounds.
- Cycle-2 plan approved by the `test-reviewer` agent on 2026-04-29
  after four review rounds (`tests/backend/shim/TEST_PLAN_CYCLE2.md`).
- Cycle-3 plan approved by the `test-reviewer` agent on 2026-04-29
  after two review rounds (`tests/backend/shim/TEST_PLAN_CYCLE3.md`),
  the second round closing on documentation correctness only —
  test logic was approved at rev 2.
- Cycle-4 plan approved by the `test-reviewer` agent on 2026-04-29
  after a single review round
  (`tests/backend/shim/TEST_PLAN_CYCLE4.md`); test logic was
  approved unchanged with two plan-text refinements.
- Cycle-5 plan approved by the `test-reviewer` agent on 2026-04-29
  after two review rounds
  (`tests/backend/shim/TEST_PLAN_CYCLE5.md`); the round-1 blocker
  was C5-R10's deferred variable-size determination for
  `notifier_state` (`offsetof(notifier_state, slots) == 8`, not 4,
  due to `alignof(notifier_slot) == 8`).
- Cycle-6 plan approved by the `test-reviewer` agent on 2026-04-29
  after a single review round
  (`tests/backend/shim/TEST_PLAN_CYCLE6.md`).
- Cycle-7 plan approved by the `test-reviewer` agent on 2026-04-29
  after a single review round
  (`tests/backend/shim/TEST_PLAN_CYCLE7.md`); test logic
  approved with two plan-text clarifications (C7-2 full-struct
  equality form; C7-3 family suite placement) and a
  non-blocking `sizeof(notifier_alarm_batch)`-not-literal-520
  recommendation that landed. **Note:** the landed C7-6
  determinism replay implementation inadvertently omitted the
  `notifier_alarm_batch` schema from the scenario, gates,
  `operator==` checks, and memcmp companion (despite the test
  name "AllSevenSlots"). Cycle 8's C8-6 corrects this gap.
- Cycle-8 plan approved by the `test-reviewer` agent on
  2026-04-29 after two review rounds
  (`tests/backend/shim/TEST_PLAN_CYCLE8.md`); round-1 blocker
  was C8-6's framing (must explicitly correct the C7-6
  notifier_alarm_batch gap rather than presenting as a
  strict superset) plus several non-blocking clarifications
  (C8-2b memcmp purpose, C8-5 step-8 snapshot, test_helpers.h
  include note, error_message_batch sizeof static_assert
  recommendation), all addressed in rev 2. Cycle 8 retires
  test 10 outright (D-C8-PROBE-RETIRED) since no per-tick
  schema remains unsupported.
- Cycle-9 plan approved by the `test-reviewer` agent on
  2026-04-29 after two review rounds
  (`tests/backend/shim/TEST_PLAN_CYCLE9.md`). Round-1 verdict
  `not-ready` with two blockers (C9-5's contradictory "drain"
  setup step against an already-empty lane; D-C9-INBOUND-
  INDEPENDENCE cross-referenced "C9-7" but the test was C9-6)
  plus three non-blocking items (C9-1b missing explicit
  `sequence == 1` assertion; `lane_in_progress` outbound gap;
  outbound `count > kMaxCanFramesPerBatch` documentation gap).
  Auditing the cross-references during rev 2 surfaced a systemic
  off-by-one error affecting four decision blocks (only one of
  which the reviewer caught), all corrected; rev 2 also added
  C9-3b for the `lane_in_progress` outbound failure mode (mirror
  of cycle-1 test 13 for the inbound side). Round-2 verdict
  `ready-to-implement` with one residual stale comment in the
  `drain_boot_only` helper docstring (corrected after round 2
  closed).
- Cycle-10 plan approved by the `test-reviewer` agent on
  2026-04-29 after a single review round
  (`tests/backend/shim/TEST_PLAN_CYCLE10.md`). Round-1 verdict
  `ready-to-implement` with one required clarification (C10-1's
  memcmp range-coverage explicit) and one non-blocking
  improvement (C10-2 mirror that language) — both addressed
  in-place. The reviewer **explicitly endorsed** the cycle's
  three open questions: OQ-C10-PER-METHOD-MIRRORS (trim from 10
  to 6 tests by skipping cross-cutting cycle-9 mirrors —
  endorsed with the directive "A future reviewer should not
  revert to 10-test coverage without a concrete new bug class
  that the trim fails to catch"); OQ-C10-HELPER-LOCATION (per-
  schema-header inline placement endorsed over an aggregator
  header); OQ-C10-NOTIFIER-ALARM-OVERLOAD (test-only overload
  stays since notifier_alarm_batch has no production caller).
- Cycle-11 plan approved by the `test-reviewer` agent on
  2026-04-30 after a single review round
  (`tests/backend/shim/TEST_PLAN_CYCLE11.md`). Round-1 verdict
  `ready-to-implement` with no blockers and one non-blocking
  recommendation (add `static_assert(offsetof(error_message_batch,
  messages) == 8)` to `error_message.h`, addressed in-place
  during implementation). Reviewer explicitly resolved both
  open questions: OQ-C11-FULL-CAPACITY-TEST (do NOT add a
  count=8 boundary test — the arithmetic is already exercised
  at counts 0/2/3/4 across the suite and the lane-capacity
  boundary is owned by tier1 tests); OQ-C11-MEMCMP-IN-DETERMINISM
  (keep the C11-7 memcmp companion despite being byte-equivalent
  to `vector::operator==` for this padding-free schema, for
  cross-cycle structural parity with C9-7 / C10-7). Cycle 11
  introduces no new design decisions; every contract inherits
  from cycles 9 and 10.
- `tests/backend/shim/shim_core_test.cpp` covers, across cycles
  1–11, **113 tests**: boot publish layout, make-failure-atomicity
  (lane busy), wrong-direction endpoint surfacing
  `boot_wrong_direction`, post-make observer initial state for all
  eight cache slots, empty-poll no-op, boot_ack handshake,
  post-connect byte-equal caching for `clock_state` / `power_state`
  / `ds_state` / `can_frame_batch` / `can_status` /
  `notifier_state` / `notifier_alarm_batch` /
  `error_message_batch` (the four variable-size schemas via
  active-prefix memcpy, including the empty-batch count=0
  boundary for each), latest-wins for each slot, the
  shrinking-batch zero-fill contract for each variable-size
  schema (C4-2b, C6-2b, C7-2b, C8-2b), out-of-order rejection
  of all eight schemas, cross-slot non-contamination across
  the full 8-slot interleaved scenario (15-step `2N-1` walk
  with all eight slots asserted at every step, covering 14
  cross-population directions added at cycle 8), shutdown
  handling, post-shutdown terminal, `lane_in_progress` error
  wrapping (both inbound poll path and outbound send path),
  a 10-step interleaved-scenario determinism replay
  (`RepeatedRunsProduceByteIdenticalAllEightSlots`,
  introduced at cycle 8) asserting byte-identical contents
  for all eight slots — including direct `std::memcmp` on
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_notifier_state()`, and
  `latest_notifier_alarm_batch()` to pin padding-byte
  determinism (no memcmp on `error_message_batch` per
  D-C8-PADDING-FREE), **and the 10 cycle-9 outbound tests in
  the new `ShimCoreSend` suite** plus the
  `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundCanFrameBatch`
  outbound determinism replay: positive publish (3-frame and
  empty batches), sequence-counter advance on success and
  preservation on both `lane_busy` and `lane_in_progress`
  failure modes, no-connect-gate (publishes before boot_ack
  is consumed), post-shutdown send short-circuit (proves
  `endpoint_.send` not called via lane-state == empty +
  `transport_error == nullopt`), 8-cache-slot inbound
  independence after a successful send, and a receive-counter
  non-perturbation indirect verification via 3 sends followed
  by a valid inbound at sequence 1. **Cycle 10 added 6 more
  outbound tests in the same `ShimCoreSend` suite plus
  `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundNotifierState`**:
  positive `send_notifier_state` publish (3-slot batch with
  payload_bytes == 272 = 8 + 3*88, with the 16 implicit padding
  bytes within the active prefix explicitly memcmp'd; and the
  count=0 boundary case at payload_bytes == 8), sequence-counter
  advance to 2 across two sends with different counts (184 and
  360 bytes), `lane_busy` preservation with recovery proof,
  per-method post-shutdown short-circuit (the one D-C9-derived
  contract that needs per-method verification per
  D-C10-SHUTDOWN-TERMINAL-INHERITS), and outbound determinism
  pinning the 4-byte interior `count → slots` pad plus the
  per-slot trailing 4 bytes inside each active slot. Cycle 10
  deliberately trimmed the cycle-9 cross-cutting mirrors
  (`lane_in_progress` outbound, no-connect-gate, inbound-cache
  independence, receive-counter independence) per the test-
  reviewer's OQ-C10-PER-METHOD-MIRRORS endorsement — those
  contracts are session/transport-level and were verified once
  through `send_can_frame_batch` at cycle 9. **Cycle 11 added 6
  more outbound tests in the same `ShimCoreSend` suite plus
  `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundError`
  `MessageBatch`**: positive `send_error_message_batch` publish
  (3-message batch with payload_bytes == 6980 = 8 + 3*2324; the
  count=0 boundary case at payload_bytes == 8 covering count
  word + named `reserved_pad[4]`), sequence-counter advance to
  2 across counts 2 and 4 (4656 and 9304 bytes), `lane_busy`
  preservation with recovery proof, per-method post-shutdown
  short-circuit, and outbound determinism. Per D-C8-PADDING-FREE
  the schema has zero implicit C++ padding (both `reserved_pad`
  fields are NAMED), so the C11-7 memcmp companions are byte-
  equivalent to `vector::operator==` but kept for cross-cycle
  structural parity per the test-reviewer's OQ-C11-MEMCMP-IN-
  DETERMINISM resolution. Same trim convention as cycle 10.
  Test 10 (the cycles-2-through-7 unsupported-schema reject +
  recovery probe) was retired at cycle 8 (D-C8-PROBE-RETIRED);
  the production-code dispatch arm for
  `unsupported_payload_schema` is retained as a defensive
  forward-compat structural guard (D-C8-DEAD-BRANCH).
- Verification run: `ctest --test-dir build` and `ctest --test-dir
  build-asan` both green for the 113-test shim suite under clang
  Debug and GCC Debug + ASan + UBSan respectively (and
  428/428 across the full project ctest under both toolchains).

## Known Limits

- All eight inbound per-tick state schemas wired (`clock_state`,
  `power_state`, `ds_state`, `can_frame_batch`, `can_status`,
  `notifier_state`, `notifier_alarm_batch`,
  `error_message_batch`); the per-tick set is closed on the
  inbound side. On-demand replies (via `on_demand_reply`)
  follow as their own TDD cycle.
- CAN RX queueing is **not implemented**; `latest_can_frame_batch_`
  is latest-wins. Per D-C4-LATEST-WINS this is deferred until the
  cycle that wires `HAL_CAN_ReadStreamSession`.
- All three outbound-meaningful schemas wired
  (`send_can_frame_batch` at cycle 9 + `send_notifier_state` at
  cycle 10 + `send_error_message_batch` at cycle 11); the v0
  outbound surface is closed for tick_boundary publishing. The
  five inbound-only schemas are intentionally absent from the
  outbound API (D-C9-TYPED-OUTBOUND).
- No outbound oversize-count clamp on any send method
  (`count > kMaxCanFramesPerBatch == 64` for `send_can_frame_batch`,
  `count > kMaxNotifiers == 32` for `send_notifier_state`,
  `count > kMaxErrorsPerBatch == 8` for `send_error_message_batch`).
  Reading past the last valid element is UB that the upstream
  validator surfaces only after the read. The C HAL ABI cycles
  that wire `HAL_CAN_SendMessage`, `HAL_Notifier*`, and
  `HAL_SendError` own the seam-level count clamp.
- No `on_demand_request` / `on_demand_reply` traffic in either
  direction. Single-flight pairing in `protocol_session` exists
  but is not yet driven by the shim.
- No shim-initiated `shutdown` envelope. Cycle 9 covers post-
  inbound-shutdown's effect on outbound (the
  `shutdown_already_observed` short-circuit); emitting
  `shutdown` from the shim is a separate concern.
- No threading model. Caller drives `poll()` and
  `send_can_frame_batch` in alternation. No notifications, no
  waits.
- No reset / reconnect. After `is_shutting_down()` becomes true,
  the shim object is unusable on both inbound and outbound paths.
- No C HAL surface (`HAL_*` symbols). Future cycles add per-surface
  exports that read from the cache and write into the outbound
  send API.
- Bound to `tier1::tier1_endpoint` directly. Generalization to a
  tier-agnostic transport seam waits until tier 2 lands.

## Cross-references

- `.claude/skills/protocol-schema.md` — wire-format POD structs,
  validator, kind/schema table; consumed transitively.
- `.claude/skills/protocol-session.md` — sequence counters,
  boot/boot_ack ordering, shutdown terminal receive state; the shim
  inherits these via `tier1_endpoint`.
- `.claude/skills/tier1-shared-memory-transport.md` — endpoint API,
  shared region ABI, lane state semantics.
- `.claude/skills/layer-2-control-system-backend.md` — broader
  Layer 2 backend status and remaining TDD cycles.
- `tests/backend/shim/TEST_PLAN.md` — approved cycle-1 test plan
  with per-test bug-class rationale.
- `tests/backend/shim/TEST_PLAN_CYCLE2.md` — approved cycle-2
  addendum (power_state inbound cache slot).
- `tests/backend/shim/TEST_PLAN_CYCLE3.md` — approved cycle-3
  addendum (ds_state inbound cache slot, padding-byte
  determinism, three-slot interleaved cross-population coverage).
- `tests/backend/shim/TEST_PLAN_CYCLE4.md` — approved cycle-4
  addendum (can_frame_batch variable-size cache slot,
  zero-init-before-write shrinking contract, per-frame
  padding determinism, four-slot interleaved cross-population
  coverage, deferred CAN RX queueing decision).
- `tests/backend/shim/TEST_PLAN_CYCLE5.md` — approved cycle-5
  addendum (can_status fixed-size cache slot, five-slot
  interleaved cross-population coverage, probe migration to
  `notifier_state` with the `offsetof(notifier_state, slots) ==
  8` pin for cycle 6).
- `tests/backend/shim/TEST_PLAN_CYCLE6.md` — approved cycle-6
  addendum (notifier_state second variable-size cache slot,
  132-byte padding determinism via direct `std::memcmp`,
  six-slot interleaved cross-population coverage,
  zero-init-before-write shrinking contract for the
  variable-prefix `slots[]` array, probe migration to
  `notifier_alarm_batch` for cycle 7).
- `tests/backend/shim/TEST_PLAN_CYCLE7.md` — approved cycle-7
  addendum (notifier_alarm_batch third variable-size cache slot,
  4-byte interior padding determinism, seven-slot interleaved
  cross-population coverage with 12 new direction-pairs,
  zero-init-before-write shrinking contract for `events[]`,
  probe migration to `error_message_batch` for cycle 8 with the
  forward-reference note that cycle 8 must replace test 10
  outright since no still-unsupported per-tick schema remains).
- `tests/backend/shim/TEST_PLAN_CYCLE8.md` — approved cycle-8
  addendum (error_message_batch fourth variable-size cache slot
  and last per-tick payload schema; D-C8-PADDING-FREE because
  both `reserved_pad` fields are named, so `operator==` covers
  every byte and no memcmp companion is added; eight-slot
  interleaved cross-population coverage with 14 new
  direction-pairs; zero-init-before-write shrinking contract for
  `messages[]`; D-C8-PROBE-RETIRED deleting test 10 outright;
  D-C8-DEAD-BRANCH retaining the production-code reject arm as
  forward-compat structural guard; C8-6 corrected the landed
  C7-6's silent omission of `notifier_alarm_batch` from the
  determinism replay).
- `tests/backend/shim/TEST_PLAN_CYCLE9.md` — approved cycle-9
  addendum (first outbound-past-boot surface;
  `send_can_frame_batch` publishes `can_frame_batch` as a
  `tick_boundary` envelope; D-C9-TYPED-OUTBOUND for the typed-
  per-schema API shape; D-C9-ACTIVE-PREFIX-OUT for the outbound
  mirror of D-C4-VARIABLE-SIZE; D-C9-NO-HELPER for the inlined
  size computation deferred until cycle 10's second site;
  D-C9-SHUTDOWN-TERMINAL for the symmetric short-circuit on
  `is_shutting_down()`; D-C9-NO-CONNECT-GATE pinning that
  outbound does not require `boot_ack`; D-C9-WRAPPED-SEND-ERROR
  refactoring the shared `wrap_send_error` to a generic message;
  new C9-3b for the `lane_in_progress` outbound failure mode
  added in rev 2 to close the round-1 reviewer's coverage gap).
- `tests/backend/shim/TEST_PLAN_CYCLE10.md` — approved cycle-10
  addendum (second outbound-meaningful schema;
  `send_notifier_state` publishes `notifier_state` as a
  `tick_boundary` envelope with the 8-byte header offset;
  D-C10-EXTRACT-ACTIVE-PREFIX cashes in cycle 9's deferred
  helper extraction by moving `active_prefix_bytes` overloads
  to the schema headers; D-C10-{INBOUND-INDEPENDENCE / NO-
  CONNECT-GATE / RECEIVE-COUNTER-INDEPENDENCE}-INHERITS
  document which cycle-9 contracts apply unchanged and need no
  per-method test; D-C10-SHUTDOWN-TERMINAL-INHERITS gets a
  per-method test (C10-5) because the short-circuit lives in
  per-method code; trimmed test set of 6 endorsed by
  test-reviewer with the directive that future reviewers
  should not revert without a concrete new bug class).
- `tests/backend/shim/TEST_PLAN_CYCLE11.md` — approved cycle-11
  addendum (third and final outbound-meaningful schema;
  `send_error_message_batch` publishes `error_message_batch`
  as a `tick_boundary` envelope; D-C10-EXTRACT-ACTIVE-PREFIX
  extended to add the third production helper to
  `error_message.h` plus the recommended
  `static_assert(offsetof(error_message_batch, messages) == 8)`;
  no new design decisions — every contract inherits from cycles
  9 and 10; per-method trim convention applied unchanged from
  cycle 10; OQ-C11-FULL-CAPACITY-TEST resolved as "do not add"
  and OQ-C11-MEMCMP-IN-DETERMINISM resolved as "keep for
  parity"; this cycle closes the v0 outbound surface for the
  three semantically-meaningful outbound schemas).
- `tests/backend/shim/shim_core_test.cpp` — implementation of the
  approved cycle-1 through cycle-11 tests.
- `tests/backend/tier1/test_helpers.h` — shared test helpers
  (`manually_fill_lane`, `make_envelope`, payload constructors,
  endpoint factories, `complete_handshake`) consumed by both the
  tier1 transport tests and the shim tests.
