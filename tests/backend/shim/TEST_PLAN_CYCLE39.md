# HAL shim core - cycle 39 test plan (Driver Station output pump)

Status: implemented

Cycle 39 adds a host-facing flush boundary for Driver Station output-side
state without adding a new protocol schema. Cycles 37 and 38 already publish
the two DS-output streams independently:

- `joystick_output_batch` for `HAL_SetJoystickOutputs`.
- `user_program_observer_snapshot` for `HAL_ObserveUserProgram*`.

The Tier 1 outbound lane carries one message at a time, so a helper that
tries to publish both schemas in one call would either require a concurrent
peer drain or would partially publish the first message and fail the second.
The smallest coherent slice is therefore a deterministic one-message pump:

```cpp
std::expected<schema_id, shim_error>
shim_core::flush_next_driver_station_output(std::uint64_t sim_time_us);
```

The return value is the schema that was successfully published. The pump
starts with `user_program_observer_snapshot`, then `joystick_output_batch`,
then repeats. It advances to the next phase only after a successful send.

## Decisions

- **D-C39-NO-NEW-SCHEMA:** Cycle 39 does not add a combined DS-output schema
  and does not bump `kProtocolVersion`. It coordinates the cycle 37/38
  schemas.
- **D-C39-ONE-MESSAGE-PUMP:** One call publishes at most one outbound message.
  This matches the single-message Tier 1 lane and avoids partial multi-send
  semantics.
- **D-C39-ORDER:** The pump order is observer snapshot first, joystick outputs
  second. This publishes the user-code mode that caused the tick before the
  joystick output/rumble commands for the same host flush cadence.
- **D-C39-DEFAULTS:** Fresh shims still publish meaningful defaults through
  the pump: observer `none` first, then an empty joystick-output header.
- **D-C39-NO-CLEAR:** The pump delegates to the existing flush methods, so
  neither observer state nor joystick output storage is cleared.
- **D-C39-FAILURE-NO-ADVANCE:** If the selected phase fails to send, the pump
  returns that `shim_error` and does not advance to the next phase. A retry
  attempts the same schema again.
- **D-C39-SHUTDOWN:** After protocol shutdown, the selected phase returns
  `shutdown_already_observed` through the existing flush method and does not
  touch the outbound lane.
- **D-C39-NO-SHORTCUT-BEHAVIOR:** The pump is host-facing orchestration only.
  HAL calls remain storage-only and are not published directly.

## Proposed tests

### C39-1 - fresh pump publishes observer then joystick defaults
- Layer / contract: host-facing DS-output pump default behavior.
- Bug class caught: reversed phase order, treating default observer/empty
  joystick state as no-op, or publishing both messages in one call.
- Inputs: connected shim with no observer calls and no joystick outputs; call
  `flush_next_driver_station_output(100000)`, drain, then call
  `flush_next_driver_station_output(200000)`.
- Expected: first call returns `schema_id::user_program_observer_snapshot` and
  publishes `mode == none`; second call returns `schema_id::joystick_output_batch`
  and publishes a header-only joystick batch with `count == 0`. Outbound
  sequences are 1 and 2, and sim times match the respective calls.
- Tolerance / determinism: exact schema ids and payload bytes; deterministic.

### C39-2 - pump publishes latest observer and joystick output state
- Layer / contract: pump delegates to cycle 37/38 latest-snapshot flushes.
- Bug class caught: coordinator reading stale state, skipping one schema, or
  bypassing the existing snapshot mapping.
- Inputs: install shim; call `HAL_ObserveUserProgramTeleop`; write joystick
  slot 2 outputs/rumble; pump twice with drains between calls.
- Expected: first message is observer `teleop`; second message is a
  `joystick_output_batch` with one slot, joystick number 2, and the exact raw
  output/rumble values.
- Tolerance / determinism: exact enum and integer values; deterministic.

### C39-3 - pump repeats observer/joystick cycle without clearing state
- Layer / contract: D-C39-NO-CLEAR and phase cycling.
- Bug class caught: clearing state after the first two publishes, failing to
  wrap back to observer, or using event-queue semantics instead of snapshots.
- Inputs: set observer disabled and one joystick output; pump four times,
  draining after each success.
- Expected: schemas arrive observer, joystick, observer, joystick. The two
  observer payloads are byte-identical and the two joystick payloads are
  byte-identical; sequences advance 1..4.
- Tolerance / determinism: exact schema order and byte equality; deterministic.

### C39-4 - lane-busy failure does not advance the selected phase
- Layer / contract: D-C39-FAILURE-NO-ADVANCE on Tier 1 backpressure.
- Bug class caught: advancing the pump phase even though the selected schema
  was not published, which would silently drop a DS-output snapshot.
- Inputs: connected shim; call the pump once and leave the observer message in
  the outbound lane; call the pump again without draining; then drain and call
  the pump a third time.
- Expected: first call succeeds with observer. Second call fails with
  `send_failed` because the selected joystick-output phase could not enter the
  full lane. Third call retries and publishes joystick outputs, proving the
  failed second call did not advance past joystick.
- Tolerance / determinism: exact error kind, schema ids, and lane state;
  deterministic.

### C39-5 - post-shutdown pump is rejected without touching lane
- Layer / contract: D-C39-SHUTDOWN.
- Bug class caught: outbound DS-output publication after terminal shutdown.
- Inputs: connected shim observes a shutdown envelope; confirm backend-to-core
  lane is empty; call `flush_next_driver_station_output`.
- Expected: returns `shutdown_already_observed`; backend-to-core lane remains
  empty.
- Tolerance / determinism: exact error kind and lane state; deterministic.

### C39-6 - pump does not publish directly from HAL calls
- Layer / contract: D-C39-NO-SHORTCUT-BEHAVIOR.
- Bug class caught: wiring observer or joystick HAL calls to publish
  immediately instead of waiting for the host pump.
- Inputs: create a shim, drain boot only, install it, call
  `HAL_ObserveUserProgramAutonomous` and `HAL_SetJoystickOutputs`.
- Expected: no outbound message is pending until
  `flush_next_driver_station_output` is called; that call publishes the
  observer snapshot first.
- Tolerance / determinism: exact lane emptiness and schema id; deterministic.

### C39-7 - repeated runs produce byte-identical pump output sequence
- Layer / contract: deterministic replay for the DS-output pump.
- Bug class caught: uninitialized phase/result state or cross-shim leakage
  affecting schema order or payload bytes.
- Inputs: run the same scenario twice: observer starting, joystick slot 4
  output, pump two messages and capture `(schema_id, payload)` pairs.
- Expected: both runs produce the same two schema ids in the same order and
  byte-identical payloads.
- Tolerance / determinism: byte-for-byte equality; deterministic.

## Deferred

- A combined DS-output host-state wire schema.
- Atomic multi-message publish semantics.
- Automatic 50 ms publication cadence.
- A host event loop that repeatedly calls the pump until a full DS-output cycle
  has been drained by the peer.
