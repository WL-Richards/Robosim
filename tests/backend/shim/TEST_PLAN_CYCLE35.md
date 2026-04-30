# HAL shim core - cycle 35 test plan (user-program observer calls)

Status: implemented

Cycle 35 implements:

```cpp
void HAL_ObserveUserProgramStarting(void);
void HAL_ObserveUserProgramDisabled(void);
void HAL_ObserveUserProgramAutonomous(void);
void HAL_ObserveUserProgramTeleop(void);
void HAL_ObserveUserProgramTest(void);
```

Primary WPILib references:

- WPILib 2026.2.2 release C++ docs for `hal/DriverStation.h` list the
  five signatures above.
- The same docs describe `HAL_ObserveUserProgramStarting` as setting the
  program-starting flag in the DS, and the mode observers as setting the
  disabled/autonomous/teleop/test response flags about every 50 ms.
- WPILib sim `DriverStation.cpp` currently routes
  `HAL_ObserveUserProgramStarting` to `HALSIM_SetProgramStarted()` and
  leaves the four mode observers as TODOs.

## Decisions

- **D-C35-NO-SHIM:** No installed shim observer calls are no-ops.
- **D-C35-VOID-SURFACE:** These functions return no status and never publish
  directly to the existing v0 outbound protocol schemas.
- **D-C35-STATE-MODEL:** The shim records a per-shim
  `user_program_observer_state` with values `none`, `starting`, `disabled`,
  `autonomous`, `teleop`, and `test`. This makes the calls observable to the
  host and avoids a no-op implementation.
- **D-C35-HOST-ACCESSOR:** `shim_core` exposes a documented host-facing v0
  accessor, `user_program_observer_state()`, so tests and future Sim Core
  integration can observe the latest state without depending on private
  implementation details.
- **D-C35-PER-SHIM:** Observer state belongs to the installed `shim_core`
  object, not process-global storage.
- **D-C35-LATEST-WINS:** Repeated observer calls replace the latest observer
  state.
- **D-C35-POST-SHUTDOWN:** `HAL_Shutdown` clears the global shim pointer, so
  later observer calls are no-shim no-ops and do not mutate the previously
  installed object.
- **D-C35-OUTBOUND-DEFERRED:** Publishing observer state to Sim Core is
  deferred until a dedicated protocol schema or request/reply surface exists.
  Encoding it into unrelated outbound schemas would hide the contract and make
  unmodified robot behavior harder to reason about later.

## Proposed tests

### C35-1 - fresh shim reports no observer state
- Layer / contract: host-facing shim_core v0 observer-state API.
- Bug class caught: defaulting a new shim to a mode state before robot code has
  reported readiness.
- Inputs: installed connected shim; no observer calls.
- Expected: `shim.user_program_observer_state()` returns
  `user_program_observer_state::none`.

### C35-2 - no shim observer calls are no-ops
- Layer / contract: Layer 2 C HAL no-shim behavior for void observer calls.
- Bug class caught: null global shim dereference or accidental global state.
- Inputs: clear global shim; call all five observer functions.
- Expected: no crash; a later installed shim still reports `none`.

### C35-3 - starting records program-starting state
- Layer / contract: WPILib starting observer surface.
- Bug class caught: leaving the concrete WPILib sim observer as an empty no-op.
- Inputs: installed shim; call `HAL_ObserveUserProgramStarting()`.
- Expected: `shim.user_program_observer_state()` reports
  `user_program_observer_state::starting`.

### C35-4 - mode observers record distinct states
- Layer / contract: WPILib disabled/autonomous/teleop/test observer surface.
- Bug class caught: routing all mode observers to one generic enabled/disabled
  flag or accidentally swapping autonomous/teleop/test.
- Inputs: installed shim; call disabled, autonomous, teleop, test.
- Expected: after each call, `shim.user_program_observer_state()` reports the
  corresponding distinct state.

### C35-5 - latest observer call wins
- Layer / contract: last-observed robot program state for periodic calls.
- Bug class caught: append-only or first-call-wins behavior.
- Inputs: installed shim; call starting, disabled, autonomous, teleop, test,
  disabled.
- Expected: final reported state is `disabled`.

### C35-6 - observer calls do not publish to existing outbound schemas
- Layer / contract: D-C35-VOID-SURFACE / D-C35-OUTBOUND-DEFERRED.
- Bug class caught: smuggling observer state through an unrelated v0 outbound
  schema.
- Inputs: create shim, drain its boot envelope, install it, verify no outbound
  message is pending, call all five observer functions.
- Expected: observer state is updated to the last call and the outbound lane
  remains empty.

### C35-7 - observer state is per shim object
- Layer / contract: no observer leakage across installed shim objects.
- Bug class caught: process-global observer state instead of per-object state.
- Inputs: install shim A and call teleop; install shim B and call autonomous.
- Expected: shim A remains `teleop`; shim B reports `autonomous`.

### C35-8 - shutdown makes later observer calls no-ops for the old shim
- Layer / contract: Cycle 28 shutdown/global-detach semantics inherited by
  observer calls.
- Bug class caught: observer wrappers retaining stale shim pointers after
  shutdown.
- Inputs: install shim; call starting; call `HAL_Shutdown`; call test.
- Expected: global current is null and the old shim still reports `starting`.

## Deferred

- Protocol publication of user-program observer state.
- Timing/50 ms watchdog semantics.
- Driver Station console/user-status data packing beyond the state enum above.
