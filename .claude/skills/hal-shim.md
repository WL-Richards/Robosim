---
name: hal-shim
description: Use when working on the in-process HAL shim (`src/backend/shim/`) — the orchestrator the user's robot binary loads to talk to the Sim Core. Through cycle 8: boot handshake, inbound `clock_state` / `power_state` / `ds_state` / `can_frame_batch` / `can_status` / `notifier_state` / `notifier_alarm_batch` / `error_message_batch` cache slots (the four variable-size — `can_frame_batch`, `notifier_state`, `notifier_alarm_batch`, `error_message_batch` — use active-prefix memcpy with zero-init-before-write semantics; the rest are fixed-size byte-copy), shutdown terminal receive path. **All 8 per-tick payload schemas are now wired**; the per-tick set is closed. Does not yet cover the C HAL ABI (`HAL_GetFPGATime`, `HAL_GetVinVoltage`, `HAL_GetControlWord`, `HAL_CAN_ReadStreamSession`, `HAL_CAN_GetCANStatus`, `HAL_Notifier*`, `HAL_SendError`, etc.), outbound traffic past boot, on-demand request/reply, CAN RX queueing semantics (deferred to whichever cycle wires the C HAL CAN consumer), threading, or reset/reconnect.
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
"AllSevenSlots" replay despite the test name. Future cycles cover
outbound traffic past `boot`, on-demand request/reply, the C HAL
ABI, threading, and reset/reconnect — the per-tick payload schema
set is closed.

## Scope

**In (cycles 1–8):**
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
  schemas wired). On-demand replies (`on_demand_request` /
  `on_demand_reply`) and outbound traffic follow as separate
  cycles. The `unsupported_payload_schema` arm is retained as
  a defensive forward-compat guard; if a post-v0 schema is
  added to `protocol_version.h` and the validator's allowed set
  but not yet wired in dispatch, the loud reject path still
  fires.
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
- `tests/backend/shim/shim_core_test.cpp` covers, across cycles
  1–8, 91 tests: boot publish layout, make-failure-atomicity (lane
  busy), wrong-direction endpoint surfacing
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
  wrapping, and a 10-step interleaved-scenario determinism
  replay (`RepeatedRunsProduceByteIdenticalAllEightSlots`,
  introduced at cycle 8 to replace and correct the landed
  C7-6) asserting byte-identical contents for all eight
  slots — including direct `std::memcmp` on
  `latest_ds_state()`, `latest_can_frame_batch()`,
  `latest_notifier_state()`, and
  `latest_notifier_alarm_batch()` to pin padding-byte
  determinism (no memcmp on `error_message_batch` per
  D-C8-PADDING-FREE). Test 10 (the cycles-2-through-7
  unsupported-schema reject + recovery probe) was retired at
  cycle 8 (D-C8-PROBE-RETIRED) since no still-unsupported
  per-tick schema remains; the production-code dispatch arm
  for `unsupported_payload_schema` is retained as a defensive
  forward-compat structural guard (D-C8-DEAD-BRANCH).
- Verification run: `ctest --test-dir build` and `ctest --test-dir
  build-asan` both green for the 91-test shim suite under clang
  Debug and GCC Debug + ASan + UBSan respectively (and
  406/406 across the full project ctest under both toolchains).

## Known Limits

- All eight inbound per-tick state schemas wired (`clock_state`,
  `power_state`, `ds_state`, `can_frame_batch`, `can_status`,
  `notifier_state`, `notifier_alarm_batch`,
  `error_message_batch`); the per-tick set is closed.
  On-demand replies (via `on_demand_reply`) and outbound
  traffic past `boot` follow as their own TDD cycle(s).
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
- `tests/backend/shim/shim_core_test.cpp` — implementation of the
  approved cycle-1 through cycle-8 tests.
- `tests/backend/tier1/test_helpers.h` — shared test helpers
  (`manually_fill_lane`, `make_envelope`, payload constructors,
  endpoint factories, `complete_handshake`) consumed by both the
  tier1 transport tests and the shim tests.
