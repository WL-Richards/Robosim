# HAL shim core - cycle 38 test plan (user-program observer publication)

Status: implemented

Cycle 38 publishes the user-program observer state recorded in cycle 35 through
a dedicated protocol schema and explicit shim flush boundary. It does not merge
observer state with cycle 37 joystick output publication.

Primary references:

- WPILib 2026.2.2 release C++ docs for `hal/DriverStation.h` list
  `HAL_ObserveUserProgramStarting`, `HAL_ObserveUserProgramDisabled`,
  `HAL_ObserveUserProgramAutonomous`, `HAL_ObserveUserProgramTeleop`, and
  `HAL_ObserveUserProgramTest`.
- `tests/backend/shim/TEST_PLAN_CYCLE35.md` pins existing per-shim storage:
  no-shim calls are no-ops, calls are latest-wins, state is per shim, calls do
  not publish directly, and post-`HAL_Shutdown` calls are no-ops.
- `tests/backend/shim/TEST_PLAN_CYCLE37.md` pins the adjacent DS output
  publication pattern: dedicated schema, explicit flush, storage-only HAL call,
  protocol-valid inbound traffic rejected by shim dispatch.
- `.claude/skills/protocol-schema.md` requires a `kProtocolVersion` bump when a
  new `schema_id` is added.

## Slice decision

The smallest coherent Cycle 38 slice is an observer-only fixed-size outbound
schema plus host-facing shim snapshot/flush methods:

```cpp
enum class user_program_observer_mode : std::int32_t {
  none = 0,
  starting = 1,
  disabled = 2,
  autonomous = 3,
  teleop = 4,
  test = 5,
};

struct user_program_observer_snapshot {
  user_program_observer_mode mode;
  std::array<std::uint8_t, 4> reserved_pad;
};
```

## Decisions

- **D-C38-OBSERVER-ONLY:** Cycle 38 includes user-program observer state only.
  It does not combine observer state with joystick output/rumble publication.
- **D-C38-PROTOCOL-V3:** Adding `schema_id::user_program_observer_snapshot`
  changes the closed protocol schema set, so `kProtocolVersion` bumps from 2
  to 3.
- **D-C38-TICK-PUSH:** `user_program_observer_snapshot` is allowed by the
  validator under the `tick_boundary` kind only. The intended publisher is
  backend-to-core; a protocol-valid core-to-backend observer snapshot is still
  rejected by shim dispatch.
- **D-C38-FIXED-SIZE:** The observer snapshot is fixed-size, not a batch. There
  is exactly one latest observer mode per shim object.
- **D-C38-NAMED-PADDING:** The snapshot has a named `reserved_pad` field.
  Senders zero-initialize it; the validator follows existing reserved-byte
  policy and ignores its contents on receive.
- **D-C38-SNAPSHOT:** `shim_core::current_user_program_observer_snapshot()`
  returns the latest stored observer mode as a protocol snapshot.
- **D-C38-EMPTY-DEFAULT:** A fresh shim snapshots to `mode == none`, and
  `flush_user_program_observer(sim_time_us)` publishes that default snapshot.
- **D-C38-REPEATED-FLUSH:** Flushing does not clear observer state. Repeated
  flushes publish the latest snapshot again with normal outbound sequence
  advancement.
- **D-C38-EXPLICIT-FLUSH:** Observer HAL calls remain storage-only. Publication
  happens only when host code calls `flush_user_program_observer`.
- **D-C38-SHUTDOWN:** After the shim observes protocol shutdown,
  `send_user_program_observer_snapshot` and `flush_user_program_observer`
  return `shutdown_already_observed` without touching the outbound lane.
- **D-C38-INBOUND-DEFENSIVE:** Even though the validator recognizes the new
  schema under `tick_boundary`, the HAL shim does not consume core-to-backend
  observer snapshots. If one arrives inbound, `poll()` fails with
  `unsupported_payload_schema` and does not mutate existing caches.

## Proposed tests

### C38-1 - user-program observer schema has pinned POD layout
- Layer / contract: Layer 2 protocol schema for observer publication.
- Bug class caught: implicit padding, wrong enum backing type, or wrong field
  order that would make host/shim disagree on wire bytes.
- Inputs: compile/runtime assertions on `user_program_observer_mode` and
  `user_program_observer_snapshot`.
- Expected: enum backing is `int32_t`; values are none=0, starting=1,
  disabled=2, autonomous=3, teleop=4, test=5; snapshot is trivially copyable
  standard-layout aggregate; offsets are `mode=0`, `reserved_pad=4`;
  `sizeof == 8`, `alignof == 4`.
- Tolerance / determinism: exact byte sizes; deterministic.

### C38-2 - validator accepts observer snapshots only at fixed size
- Layer / contract: stateless protocol validator fixed-size schema gate.
- Bug class caught: forgetting the new fixed-size payload in the validator,
  accepting truncated snapshots, or accepting oversized snapshots with trailing
  stale bytes.
- Inputs: `tick_boundary + user_program_observer_snapshot` envelopes with
  payload sizes `sizeof(snapshot)`, `sizeof(snapshot) - 1`, and
  `sizeof(snapshot) + 1`.
- Expected: exact size validates; undersized and oversized payloads fail with
  `payload_size_mismatch`.
- Tolerance / determinism: exact sizes; deterministic.

### C38-3 - kind/schema mapping exposes observer snapshot only as tick push
- Layer / contract: protocol v3 closed kind/schema allowance table.
- Bug class caught: accidentally allowing observer snapshots as on-demand
  traffic or forgetting to recognize schema id 11 after bumping the enum.
- Inputs: validate `schema_id::user_program_observer_snapshot` across every
  `envelope_kind`, and include it in the existing exhaustive cross-product.
- Expected: only `tick_boundary` accepts it; `kSchemaIdMaxValid == 11`;
  `kProtocolVersion == 3`.
- Tolerance / determinism: exact enum/table values; deterministic.

### C38-4 - send_user_program_observer_snapshot publishes tick_boundary wire bytes
- Layer / contract: shim outbound typed publisher.
- Bug class caught: publishing on the wrong schema id, wrong fixed payload
  length, or wrong sim time / sequence.
- Inputs: connected shim, drained boot lane, observer snapshot with
  `mode == teleop`.
- Expected: core receives a backend-to-core `tick_boundary` envelope with
  `schema_id::user_program_observer_snapshot`, the requested sim time,
  sequence 1, and payload bytes equal to the snapshot bytes.
- Tolerance / determinism: exact bytes; deterministic.

### C38-5 - fresh shim flush publishes observer none snapshot
- Layer / contract: D-C38-EMPTY-DEFAULT host flush behavior.
- Bug class caught: treating default observer state as no-op and leaving the
  host unable to observe "robot has not reported a user-program mode yet".
- Inputs: connected shim, drained boot lane, no observer calls, call
  `flush_user_program_observer(250000)`.
- Expected: flush succeeds; core receives a snapshot with `mode == none` and
  zeroed reserved bytes.
- Tolerance / determinism: exact bytes; deterministic.

### C38-6 - flush publishes latest observer mode mapping
- Layer / contract: D-C38-SNAPSHOT and cycle 35 latest-wins observer storage.
- Bug class caught: stale first-call-wins behavior, swapped disabled/autonomous/
  teleop/test mapping, or failure to publish starting distinctly from modes.
- Inputs: install shim; call starting and flush; call disabled, autonomous,
  teleop, test and flush after each.
- Expected: each received snapshot mode matches the most recent observer call.
- Tolerance / determinism: exact enum values; deterministic.

### C38-7 - observer HAL calls do not publish until explicit flush
- Layer / contract: D-C38-EXPLICIT-FLUSH and cycle 35 storage behavior.
- Bug class caught: reintroducing direct publication from observer HAL calls and
  making timing depend on robot-code call sites.
- Inputs: connected shim, drained boot lane, install shim, call
  `HAL_ObserveUserProgramTeleop`, then inspect the core inbound lane before
  flushing.
- Expected: no outbound message is pending after the HAL call; a later flush
  publishes `mode == teleop`.
- Tolerance / determinism: exact lane state; deterministic.

### C38-8 - repeated flush republishes latest observer snapshot without clearing it
- Layer / contract: D-C38-REPEATED-FLUSH.
- Bug class caught: clearing observer state after the first flush or using an
  append/event queue instead of latest snapshot state.
- Inputs: call autonomous; flush at `100000`; drain; flush again at `200000`.
- Expected: both flushes publish `mode == autonomous`; the second envelope
  sequence advances by one and uses the second sim time.
- Tolerance / determinism: exact bytes and sequence; deterministic.

### C38-9 - post-shutdown observer publication is rejected without touching lane
- Layer / contract: D-C38-SHUTDOWN.
- Bug class caught: outbound sends after terminal shutdown.
- Inputs: connected shim observes a shutdown envelope; ensure outbound lane is
  empty; call both `send_user_program_observer_snapshot` and
  `flush_user_program_observer`.
- Expected: each returns `shutdown_already_observed`; no outbound message is
  pending.
- Tolerance / determinism: exact error kind and lane state; deterministic.

### C38-10 - protocol-valid inbound observer snapshot is rejected by the shim
- Layer / contract: D-C38-INBOUND-DEFENSIVE shim dispatch boundary.
- Bug class caught: accidentally treating host observer reports as
  sim-authoritative inbound state or silently discarding a newly valid schema.
- Inputs: connected shim with a known cached `clock_state`; core sends a valid
  `tick_boundary + user_program_observer_snapshot` inbound payload.
- Expected: `poll()` fails with `unsupported_payload_schema`; existing cached
  clock state remains unchanged.
- Tolerance / determinism: exact error kind and cached value; deterministic.

### C38-11 - joystick output publication is not included in observer snapshots
- Layer / contract: D-C38-OBSERVER-ONLY / joystick publication remains separate.
- Bug class caught: smuggling joystick output state through the observer schema
  or making observer payload depend on unrelated DS output hooks.
- Inputs: write joystick outputs and no observer calls, flush observer; then
  call an observer hook and write different joystick outputs, flush observer
  again.
- Expected: first observer payload is `none`; second payload contains only the
  observer mode. Joystick output state remains publishable through the cycle 37
  flush but is not encoded in observer snapshots.
- Tolerance / determinism: exact payload bytes; deterministic.

### C38-12 - repeated runs publish byte-identical observer snapshots
- Layer / contract: deterministic outbound replay for the new protocol schema.
- Bug class caught: uninitialized reserved bytes or process-global leakage
  affecting snapshot bytes.
- Inputs: run the same scenario twice: call starting, teleop, disabled, flush
  and capture the outbound payload bytes.
- Expected: the two payload byte vectors are identical and contain
  `mode == disabled` with zeroed reserved bytes.
- Tolerance / determinism: byte-for-byte equality; deterministic.

## Deferred

- A combined Driver Station output/observer host-state schema.
- Automatic 50 ms publication cadence.
- Driver Station console/user-status data beyond the current observer enum.
