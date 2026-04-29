---
name: hal-shim
description: Use when working on the in-process HAL shim (`src/backend/shim/`) — the orchestrator the user's robot binary loads to talk to the Sim Core. Through cycle 5: boot handshake, inbound `clock_state` / `power_state` / `ds_state` / `can_frame_batch` / `can_status` cache slots (`can_frame_batch` is variable-size with active-prefix memcpy and zero-init-before-write semantics; the rest are fixed-size byte-copy), shutdown terminal receive path. Does not yet cover the C HAL ABI (`HAL_GetFPGATime`, `HAL_GetVinVoltage`, `HAL_GetControlWord`, `HAL_CAN_ReadStreamSession`, `HAL_CAN_GetCANStatus`, etc.), outbound traffic past boot, schemas other than `clock_state`/`power_state`/`ds_state`/`can_frame_batch`/`can_status`/`none`, CAN RX queueing semantics (deferred to whichever cycle wires the C HAL CAN consumer), threading, or reset/reconnect.
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
byte determinism extends D-C3-7); cycle 5 adds the inbound
`can_status` cache slot as a fifth peer — back to the fixed-size,
no-padding shape of cycles 1/2/3. Future cycles add the remaining
inbound schemas, outbound traffic, the C HAL ABI, threading, and
reset/reconnect.

## Scope

**In (cycles 1–5):**
- `shim_core::make(endpoint, desc, sim_time_us)` — moves the caller-
  owned `tier1_endpoint` into the shim and immediately publishes one
  `boot` envelope carrying `desc` via `endpoint.send`.
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
  - `tick_boundary` + any other schema → returns
    `unsupported_payload_schema`. The `protocol_session` has already
    accepted the framing and advanced the receive counter, so the
    next valid envelope is unaffected.
  - `shutdown` → flips `is_shutting_down()` to true.
  - any other `envelope_kind` → returns `unsupported_envelope_kind`.
- Terminal post-shutdown short-circuit: once `is_shutting_down()` is
  true, every subsequent `poll()` returns
  `shutdown_already_observed` without touching the lane.
- Observers: `is_connected()`, `is_shutting_down()`,
  `latest_clock_state()`, `latest_power_state()`,
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_can_status()`.

**Out:**
- All inbound schemas other than `clock_state`, `power_state`,
  `ds_state`, `can_frame_batch`, `can_status`, and `none`. Cycle
  6+ adds `notifier_state`, then `notifier_alarm_batch`,
  `error_message_batch`, on-demand replies.
- **CAN RX queueing semantics.** `latest_can_frame_batch_` uses
  latest-wins (D-C4-LATEST-WINS) — a deferred decision pinned by
  the cycle-4 tests. Cycle-1 D #7 anticipated that CAN RX may
  eventually need queueing, but no live consumer drives that
  requirement yet. The cycle that wires `HAL_CAN_ReadStreamSession`
  brings the queue contract with it; until then, latest-wins is
  the explicit and tested behavior.
- All outbound traffic past the construction-time `boot`. Cycle 3+
  adds shim-side `tick_boundary` (notifier registrations, error
  messages, CAN TX), `on_demand_request`, and shim-initiated
  `shutdown`.
- The exported C HAL ABI (`HAL_GetFPGATime`, `HAL_Initialize`,
  `HAL_GetAlliance`, etc.). Cycle 4+ adds these as separate
  surface-files that read from / write to the shim's cache.
- Threading. The shim is single-threaded; the caller drives `poll()`.
- Reset / reconnect. After shutdown, the shim object is dead.
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
- `shim_core::is_connected()`, `is_shutting_down()`,
  `latest_clock_state()`, `latest_power_state()`,
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_can_status()` — observers.

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
    `ds_state` (cycle 3) and `can_frame_batch` (cycle 4) are
    the cached schemas with padding. `ds_state` has 5 interior
    padding bytes; `can_frame_batch` has 3 trailing padding
    bytes per `can_frame` × 64 frame slots. The shim's
    `std::memcpy` byte-copy preserves padding verbatim; the
    test fixtures' `ds_state{}` / `can_frame_batch{}` /
    `can_frame{}` zero-init patterns ensure source padding is
    zero on both runs. The current determinism test
    (`RepeatedRunsProduceByteIdenticalAllFourSlots`) asserts
    `std::memcmp == 0` directly on both `latest_ds_state()`
    and `latest_can_frame_batch()`, in addition to `operator==`,
    because defaulted `operator==` is field-by-field and does
    not pin padding bytes. CLAUDE.md non-negotiable #5
    ("byte-identical logs") demands this. `clock_state` and
    `power_state` are padding-free by their field layouts and
    don't need a `memcmp` companion.

11. **Variable-size schemas use active-prefix memcpy with
    zero-init before every write** (cycle 4, `can_frame_batch`).
    The dispatch arm reads `received->payload.size()` bytes
    (not `sizeof(can_frame_batch)`); the protocol_session
    validator has already verified the count↔length contract.
    The destination is freshly zero-initialized before every
    memcpy so a shrinking batch (M-frame batch replacing an
    N-frame batch with M < N) leaves `frames[M..63]`
    byte-zero, not stale. The naive
    "memcpy directly into the existing optional's storage"
    implementation leaks stale frames; pinned by C4-2b.

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
- `tests/backend/shim/shim_core_test.cpp` covers, across cycles
  1–5, 50 tests: boot publish layout, make-failure-atomicity (lane
  busy), wrong-direction endpoint surfacing
  `boot_wrong_direction`, post-make observer initial state for all
  four cache slots, empty-poll no-op, boot_ack handshake,
  post-connect byte-equal caching for `clock_state` / `power_state`
  / `ds_state` / `can_frame_batch` (the last via active-prefix
  memcpy of variable-size payloads, including the empty-batch
  count=0 boundary), latest-wins for each slot, the cycle-4-only
  shrinking-batch zero-fill contract, out-of-order rejection of
  all four schemas, cross-slot non-contamination across the full
  pairwise matrix (boot_ack/clock/power/ds → cf-slot stays
  nullopt; interleaved 7-step scenario with all four slots
  asserted at every step covers the 6 new cycle-4 cross-population
  directions), unsupported-schema reject + recovery (probe schema
  is `can_status`; cycle 5 will migrate it forward to
  `notifier_state`), shutdown handling, post-shutdown terminal,
  `lane_in_progress` error wrapping, and an interleaved-scenario
  determinism replay asserting byte-identical contents for all
  four slots — including direct `std::memcmp` on both
  `latest_ds_state()` and `latest_can_frame_batch()` to pin
  padding-byte determinism.
- Verification run: `ctest --test-dir build` and `ctest --test-dir
  build-asan` both green for the 50-test shim suite under clang
  Debug and GCC Debug + ASan + UBSan respectively.

## Known Limits

- Five inbound state schemas wired (`clock_state`, `power_state`,
  `ds_state`, `can_frame_batch`, `can_status`); other schemas are
  loud failures. Cycle 6+ will add `notifier_state`, then
  `notifier_alarm_batch`, `error_message_batch`, on-demand
  replies, one at a time, each as its own TDD cycle.
- CAN RX queueing is **not implemented**; `latest_can_frame_batch_`
  is latest-wins. Per D-C4-LATEST-WINS this is deferred until the
  cycle that wires `HAL_CAN_ReadStreamSession`.
- No outbound traffic past `boot`. The shim does not yet emit
  `tick_boundary`, `on_demand_request`, or `shutdown`.
- No threading model. Caller drives `poll()`. No notifications, no
  waits.
- No reset / reconnect. After `is_shutting_down()` becomes true,
  the shim object is unusable.
- No C HAL surface (`HAL_*` symbols). Future cycles add per-surface
  exports that read from / write to the cache.
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
- `tests/backend/shim/shim_core_test.cpp` — implementation of the
  approved cycle-1 + cycle-2 + cycle-3 + cycle-4 + cycle-5 tests.
- `tests/backend/tier1/test_helpers.h` — shared test helpers
  (`manually_fill_lane`, `make_envelope`, payload constructors,
  endpoint factories, `complete_handshake`) consumed by both the
  tier1 transport tests and the shim tests.
