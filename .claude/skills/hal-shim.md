---
name: hal-shim
description: Use when working on the in-process HAL shim (`src/backend/shim/`) — the orchestrator the user's robot binary loads to talk to the Sim Core. Cycle-1 surface only: boot handshake, inbound clock_state cache slot, shutdown terminal receive path. Does not yet cover the C HAL ABI (`HAL_GetFPGATime` etc.), outbound traffic past boot, schemas other than `clock_state`/`none`, threading, or reset/reconnect.
---

# HAL shim core

The in-process Layer-2 component the user's robot binary will load
(eventually as `libhal.so`). Sits *underneath* the WPILib HAL surface;
the robot code is unmodified. v0 cycle 1 implements the shim's core
orchestrator only — the boot handshake, the post-boot inbound drain
loop, and the inbound `clock_state` cache slot. Future cycles add the
remaining inbound schemas, outbound traffic, the C HAL ABI, threading,
and reset/reconnect.

## Scope

**In (cycle 1):**
- `shim_core::make(endpoint, desc, sim_time_us)` — moves the caller-
  owned `tier1_endpoint` into the shim and immediately publishes one
  `boot` envelope carrying `desc` via `endpoint.send`.
- `shim_core::poll()` — drains zero-or-one inbound message per call;
  dispatches by `(envelope_kind, schema_id)`:
  - `boot_ack` → flips `is_connected()` to true.
  - `tick_boundary` + `clock_state` → byte-copies the payload into
    the cached slot.
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
  `latest_clock_state()`.

**Out:**
- All inbound schemas other than `clock_state` and `none`. Cycle 2+
  adds `power_state`, `ds_state`, `can_status`, `can_frame_batch`,
  `notifier_alarm_batch`, `error_message_batch`, on-demand replies.
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
  `latest_clock_state()` — observers.

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
   semantics. Latest-wins is what `HAL_GetFPGATime` and friends will
   need; if a future schema needs queueing (e.g. CAN RX), its slot
   gets queue semantics, not a layered queue under `clock_state`.
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

- Test plan approved by the `test-reviewer` agent on 2026-04-29 after
  two review rounds.
- `tests/backend/shim/shim_core_test.cpp` adds 14 tests covering: boot
  publish layout, make-failure-atomicity (lane busy), wrong-direction
  endpoint surfacing `boot_wrong_direction`, post-make observer
  initial state, empty-poll no-op, boot_ack handshake, post-connect
  `clock_state` byte-equal caching, latest-wins, out-of-order
  rejection (via direct `manually_fill_lane` injection),
  unsupported-schema reject + recovery, shutdown handling,
  post-shutdown terminal, `lane_in_progress` error wrapping, and
  determinism replay.
- Verification run: `ctest --test-dir build --output-on-failure`
  passes 243 tests under both clang Debug and GCC Debug + ASan +
  UBSan.

## Known Limits

- Single inbound schema (`clock_state`); other schemas are loud
  failures. Cycle 2+ will add `power_state`, `ds_state`, etc. one at
  a time, each as its own TDD cycle.
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
- `tests/backend/shim/TEST_PLAN.md` — approved test plan with
  per-test bug-class rationale.
- `tests/backend/shim/shim_core_test.cpp` — implementation of the
  approved tests.
- `tests/backend/tier1/test_helpers.h` — shared test helpers
  (`manually_fill_lane`, `make_envelope`, payload constructors,
  endpoint factories, `complete_handshake`) consumed by both the
  tier1 transport tests and the shim tests.
