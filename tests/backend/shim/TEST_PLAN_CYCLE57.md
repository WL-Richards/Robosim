# Cycle 57 test plan - DriverStation observer JNI adapters

Status: implemented.

Reviewer: approved as ready.

## Context

Cycle 56 added the NotifierJNI lifecycle/wait adapters and the smoke script now
reaches:

```text
********** Robot program startup complete **********
```

`TimedRobot.startCompetition()` calls
`DriverStationJNI.observeUserProgramStarting()` immediately after that print.
The C HAL observer surface already exists from Cycle 35 and publication of the
latest observer state already exists from Cycles 38-39:

- `HAL_ObserveUserProgramStarting`
- `HAL_ObserveUserProgramDisabled`
- `HAL_ObserveUserProgramAutonomous`
- `HAL_ObserveUserProgramTeleop`
- `HAL_ObserveUserProgramTest`

WPILib 2026.2.1 Java signatures pinned with `javap -s`:

```text
observeUserProgramStarting()V
observeUserProgramDisabled()V
observeUserProgramAutonomous()V
observeUserProgramTeleop()V
observeUserProgramTest()V
```

## Scope

In:

- Add JNI exports:
  - `Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramStarting`
  - `Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramDisabled`
  - `Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramAutonomous`
  - `Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTeleop`
  - `Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTest`
- Adapt each directly to the matching C HAL observer function.
- Update the JNI artifact export check to require the new symbols.

Out:

- No new observer protocol schema; `user_program_observer_snapshot` already
  exists.
- No automatic flush from the JNI observer calls; publication remains explicit
  through `flush_user_program_observer` / `flush_next_driver_station_output`.
- No Driver Station mode synthesis.
- No Java object construction.

## v0 decisions

- **D-C57-ADAPTERS-ONLY:** Observer JNI methods are thin adapters over the
  existing C HAL observer calls and do not inspect `shim_core` directly.
- **D-C57-VOID-NO-STATUS:** Java and C observer methods are void. No-shim paths
  are no-ops inherited from the C HAL functions.
- **D-C57-LATEST-WINS-INHERITS:** Repeated JNI observer calls inherit the C HAL
  latest-wins per-shim observer state from Cycle 35.
- **D-C57-PUBLICATION-DEFERRED-INHERITS:** JNI observer calls do not publish
  outbound messages directly. Existing explicit flush/pump boundaries remain
  the only publication path.

## Tests

### C57-1 - artifact exports DriverStation observer JNI symbols

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  `DriverStationJNI.observeUserProgramStarting()` or related mode observers.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include all five observer JNI
  methods listed above.
- **Determinism notes:** Static artifact inspection only.

### C57-2 - `observeUserProgramStarting` records starting state

- **Layer / contract:** JNI adapter over C `HAL_ObserveUserProgramStarting`.
- **Bug class caught:** Missing adapter, no-op implementation, or bypassing the
  C HAL observer state.
- **Inputs:** Installed connected shim; call
  `DriverStationJNI.observeUserProgramStarting()`.
- **Expected outputs:** `shim.user_program_observer_state()` is
  `user_program_observer_state::starting`.
- **Determinism notes:** In-process.

### C57-3 - mode observer JNI methods record distinct states

- **Layer / contract:** JNI adapters over disabled/autonomous/teleop/test C HAL
  observer calls.
- **Bug class caught:** Swapped mode mapping, all modes routed to one generic
  state, or accidental no-op.
- **Inputs:** Installed shim; call disabled, autonomous, teleop, and test JNI
  observers.
- **Expected outputs:** After each call, `shim.user_program_observer_state()`
  reports the matching distinct state.
- **Determinism notes:** In-process.

### C57-4 - latest observer JNI call wins

- **Layer / contract:** JNI adapters inherit Cycle 35 latest-wins behavior.
- **Bug class caught:** append-only or first-call-wins behavior in the JNI
  layer.
- **Inputs:** Installed shim; call starting, disabled, autonomous, teleop, test,
  disabled through JNI.
- **Expected outputs:** Final observer state is `disabled`.
- **Determinism notes:** In-process.

### C57-5 - no-shim observer JNI calls are no-ops and do not create fallback state

- **Layer / contract:** no-shim void observer behavior and Cycle 51 fallback
  boundary.
- **Bug class caught:** null shim dereference, accidental global state, or
  observer JNI methods creating the fallback shim.
- **Inputs:** Reset JNI launch state and clear `shim_core::current()`. Call all
  five observer JNI methods. Then install a fresh shim.
- **Expected outputs:** No crash; no fallback state exists; fresh shim observer
  state remains `none`.
- **Determinism notes:** In-process.

### C57-6 - observer JNI calls do not publish until explicit flush

- **Layer / contract:** explicit outbound publication boundary.
- **Bug class caught:** JNI observer methods smuggling state through outbound
  protocol traffic or auto-flushing.
- **Inputs:** Create a shim, drain boot, verify outbound lane empty, install it,
  call `observeUserProgramTeleop()` through JNI.
- **Expected outputs:** Outbound lane remains empty until
  `flush_user_program_observer`; after explicit flush, payload mode is teleop.
- **Determinism notes:** In-process.

### C57-7 - manual Java launch advances past starting observer

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** `scripts/run_timed_robot_smoke.sh`.
- **Expected outputs:** Output still reaches
  `********** Robot program startup complete **********`; no missing JNI error
  for `DriverStationJNI.observeUserProgramStarting`.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts.
