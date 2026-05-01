# HAL shim core - cycle 40 test plan (HAL runtime type)

Status: implemented

Cycle 40 adds the HALBase startup/runtime query:

```cpp
HAL_RuntimeType HAL_GetRuntimeType(void);
```

This is a small HAL C ABI surface needed by unmodified WPILib startup code.
It is deliberately not a protocol change. The existing protocol
`runtime_type` enum is a wire/runtime descriptor for boot envelopes and is
already closed independently; the C HAL enum mirrors WPILib's `HAL_RuntimeType`
values.

## Decisions

- **D-C40-C-ABI-ONLY:** Cycle 40 adds only the C HAL enum/declaration and
  implementation for `HAL_GetRuntimeType`. No protocol schema, protocol
  version, boot descriptor, or shim_core state changes are part of this slice.
- **D-C40-WPILIB-VALUES:** The C enum mirrors the WPILib HALBase values:
  `HAL_Runtime_RoboRIO == 0`, `HAL_Runtime_RoboRIO2 == 1`, and
  `HAL_Runtime_Simulation == 2`. This is intentionally distinct from the
  robosim protocol `runtime_type` wire enum.
- **D-C40-ROBORIO2:** v0 reports `HAL_Runtime_RoboRIO2`, because this shim
  targets the roboRIO 2 runtime profile already used by boot descriptors.
- **D-C40-NO-SHIM-REQUIRED:** `HAL_GetRuntimeType` is process/runtime metadata,
  so it returns RoboRIO 2 even when no shim is installed.
- **D-C40-SHUTDOWN-STABLE:** `HAL_Shutdown` detaches the installed shim but
  does not change runtime identity; calls after shutdown still return RoboRIO 2.
- **D-C40-NO-PUBLISH:** The call is read-only runtime metadata. It must not
  publish outbound messages, poll inbound messages, mutate cached sim state, or
  require boot/boot_ack connection.
- **D-C40-NO-STATUS:** The WPILib signature has no status parameter, so there
  is no error-status path to expose in this cycle.

## Proposed tests

### C40-1 - HAL runtime enum and signature mirror WPILib
- Layer / contract: C HAL ABI compile-time surface.
- Bug class caught: wrong return type, status parameter drift, or enum value
  drift that would make robot code observe a non-WPILib runtime value.
- Inputs: compile-time checks against `decltype(&HAL_GetRuntimeType)` and
  enum backing values.
- Expected: function pointer type is `HAL_RuntimeType (*)()`;
  `HAL_Runtime_RoboRIO == 0`, `HAL_Runtime_RoboRIO2 == 1`,
  `HAL_Runtime_Simulation == 2`, and `sizeof(HAL_RuntimeType) == 4`.
- Tolerance / determinism: compile-time exact.

### C40-2 - no-shim runtime type reports RoboRIO2
- Layer / contract: no-shim behavior for runtime metadata.
- Bug class caught: requiring an installed shim for static HAL runtime
  information or returning simulation because the host process is local.
- Inputs: ensure no shim is globally installed, then call
  `HAL_GetRuntimeType`.
- Expected: returns `HAL_Runtime_RoboRIO2`.
- Tolerance / determinism: exact enum value; deterministic.

### C40-3 - installed shim runtime query is read-only and does not publish or poll
- Layer / contract: D-C40-NO-PUBLISH for an installed shim before boot_ack.
- Bug class caught: shortcut behavior where a read-only HAL metadata query
  sends protocol messages or consumes inbound messages.
- Inputs: create a shim, drain only the boot envelope, queue a valid
  `clock_state` in the inbound core-to-backend lane, install the shim, then
  call `HAL_GetRuntimeType`.
- Expected: returns `HAL_Runtime_RoboRIO2`; backend-to-core lane remains empty;
  core-to-backend lane remains full with the clock message still pending; latest
  clock/power/DS caches remain unset.
- Tolerance / determinism: exact lane states and cached-state emptiness.

### C40-4 - runtime type stays stable after boot, state updates, and shutdown
- Layer / contract: D-C40-SHUTDOWN-STABLE and D-C40-NO-PUBLISH in a normal
  connected session.
- Bug class caught: tying runtime type to connection state, sim state, or
  shutdown state.
- Inputs: connect a shim, install it, deliver clock/power/DS updates, call
  `HAL_GetRuntimeType`, then call `HAL_Shutdown` and call it again.
- Expected: both calls return `HAL_Runtime_RoboRIO2`; no extra outbound
  messages are published by either call.
- Tolerance / determinism: exact enum value and lane state.

## Deferred

- `HAL_GetTeamNumber`, `HAL_GetFPGAVersion`, `HAL_GetFPGARevision`,
  `HAL_GetSerialNumber`, and `HAL_GetComments`.
- `HAL_ExpandFPGATime` and system clock tick expansion behavior.
- Protocol changes to expose host-selected runtime variants.
