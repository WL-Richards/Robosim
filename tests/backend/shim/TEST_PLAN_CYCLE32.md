# HAL shim core - cycle 32 test plan (HAL_RefreshDSData)

Status: implemented

Cycle 32 implements the first Driver Station runtime-loop surface after the
cache-read DS APIs:

```cpp
HAL_Bool HAL_RefreshDSData(void);
```

Primary WPILib reference:

- `hal/DriverStation.h` release docs, generated 2026-02-27:
  `HAL_Bool HAL_RefreshDSData(void)` - "Refresh the DS control word" and
  returns true if updated.

## Slice

This cycle is intentionally limited to `HAL_RefreshDSData`. It does not
implement DS new-data event handles, user-program observer flags,
`HAL_GetOutputsEnabled`, or joystick outputs/rumble. Those surfaces need their
own decisions about WPI signal-object integration and outbound DS/user status
schema rather than being folded into refresh as placeholders.

## Decisions

- **D-C32-NO-SHIM:** No installed shim returns false and does not crash.
- **D-C32-ONE-POLL:** Each call attempts one inbound receive/dispatch step,
  using the same validation and dispatch semantics as `shim_core::poll()`.
  The current Tier 1 lane holds at most one message, so this is a design
  constraint rather than a multi-message queue-drain test target in this cycle.
- **D-C32-TRUE-ONLY-DS:** The return value is true only when that call accepts
  a valid `tick_boundary` `ds_state` payload and updates the cached DS state.
  Valid non-DS inbound messages still take effect but return false.
- **D-C32-NO-MESSAGE:** Installed shim with no inbound envelope returns false
  without mutating existing caches.
- **D-C32-LATEST-WINS:** Repeated DS refreshes update the same
  `latest_ds_state_` cache used by existing DS getters.
- **D-C32-ERRORS-FALSE:** Because the C ABI has no status out-parameter, v0
  maps poll/dispatch errors to false. This cycle does not add tests for
  malformed transport lanes; existing `poll()` tests own those error paths.
- **D-C32-SHUTDOWN:** If the next inbound envelope is `shutdown`, refresh
  observes the terminal shutdown state and returns false. Later calls also
  return false without clearing the shutdown state.

## Proposed tests

### C32-1 - no shim returns false

- **Layer / contract:** Layer 2 C HAL no-shim behavior for a status-less DS
  runtime-loop call.
- **Bug class caught:** null global shim dereference or incorrectly claiming
  new data without a transport.
- **Inputs:** no installed shim; call `HAL_RefreshDSData()`.
- **Expected:** returns `0`.

### C32-2 - no inbound message returns false and preserves existing DS getters

- **Layer / contract:** Layer 2 no-message refresh behavior.
- **Bug class caught:** fabricating an update from no inbound message or
  clearing the last DS snapshot when nothing arrived.
- **Inputs:** installed connected shim; first refresh one valid `ds_state` with
  distinct control bits, alliance station, and match time; then call
  `HAL_RefreshDSData()` again with no pending `core_to_backend` envelope.
- **Expected:** first refresh returns `1`; second refresh returns `0`;
  `HAL_GetControlWord`, `HAL_GetAllianceStation`, and `HAL_GetMatchTime` still
  expose the first packet's values with `kHalSuccess`.

### C32-3 - DS packet returns true and updates existing DS getters

- **Layer / contract:** HAL refresh drives the public DS read cache.
- **Bug class caught:** returning true without dispatching the payload, or
  updating a separate cache not observed by `HAL_GetControlWord`,
  `HAL_GetAllianceStation`, and `HAL_GetMatchTime`.
- **Inputs:** installed connected shim; one valid `ds_state` with distinct
  control bits, alliance station, and match time.
- **Expected:** `HAL_RefreshDSData()` returns `1`; subsequent DS getters return
  the packet's values with `kHalSuccess`.

### C32-4 - non-DS packet dispatches but returns false and preserves DS getters

- **Layer / contract:** return value means "DS data updated", not "any inbound
  message processed".
- **Bug class caught:** treating any successful poll as a DS refresh, or
  ignoring valid non-DS state while refresh is responsible for dispatch.
- **Inputs:** installed connected shim; first refresh one valid `ds_state`,
  then refresh one valid `clock_state` packet.
- **Expected:** clock refresh returns `0`; `HAL_GetFPGATime()` returns the clock
  packet timestamp with `kHalSuccess`; DS getters still expose the earlier DS
  packet's values with `kHalSuccess`.

### C32-5 - repeated DS refreshes use latest-wins cache semantics

- **Layer / contract:** refresh writes the same latest cache observed by prior
  DS C HAL reads.
- **Bug class caught:** first DS packet sticks forever, stale getter values, or
  appending DS packets instead of replacing the latest snapshot.
- **Inputs:** installed connected shim; refresh first `ds_state`, read
  `HAL_GetMatchTime`; refresh second `ds_state` with a different match time
  and control word.
- **Expected:** first read sees the first state; second read sees the second
  state.

### C32-6 - shutdown returns false and terminal state prevents later DS updates

- **Layer / contract:** HAL refresh shares `poll()` terminal shutdown
  semantics.
- **Bug class caught:** reporting shutdown as new DS data, ignoring shutdown,
  or accepting post-shutdown DS packets.
- **Inputs:** installed connected shim; first refresh one valid `ds_state` and
  observe it through DS getters; then enqueue a `shutdown` envelope and call
  refresh; then place a valid `ds_state` packet with different getter-visible
  values on the inbound lane and call refresh again.
- **Expected:** shutdown refresh returns `0`; post-shutdown refresh returns
  `0`; `HAL_GetControlWord`, `HAL_GetAllianceStation`, and
  `HAL_GetMatchTime` still expose the pre-shutdown DS values.

## Deferred

- `HAL_ProvideNewDataEventHandle` / `HAL_RemoveNewDataEventHandle`.
- WPI signal-object creation, setting, reset, wait, and destruction.
- User-program observer calls:
  `HAL_ObserveUserProgramStarting`, `HAL_ObserveUserProgramDisabled`,
  `HAL_ObserveUserProgramAutonomous`, `HAL_ObserveUserProgramTeleop`, and
  `HAL_ObserveUserProgramTest`.
- `HAL_GetOutputsEnabled`.
- `HAL_SetJoystickOutputs`.
