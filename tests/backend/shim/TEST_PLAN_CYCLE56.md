# Cycle 56 test plan - NotifierJNI lifecycle adapters

Status: implemented.

Reviewer: approved as ready.

## Context

Cycle 55 added `DriverStationJNI.sendError(...)`. The empty TimedRobot smoke
now prints:

```text
********** Robot program starting **********
```

and exits without a missing `sendError` JNI symbol. It still does not print
WPILib's later:

```text
********** Robot program startup complete **********
```

`TimedRobot` constructs and drives a HAL Notifier through
`edu.wpi.first.hal.NotifierJNI`. The C HAL Notifier lifecycle and wait surface
already exists from cycles 25-27:

- `HAL_InitializeNotifier`
- `HAL_SetNotifierName`
- `HAL_StopNotifier`
- `HAL_CleanNotifier`
- `HAL_UpdateNotifierAlarm`
- `HAL_CancelNotifierAlarm`
- `HAL_WaitForNotifierAlarm`

Cycle 50 only exported `NotifierJNI.setHALThreadPriority(...)`. The next
coherent launch slice is the remaining NotifierJNI lifecycle adapters over the
existing C HAL surface.

WPILib 2026.2.1 Java signatures pinned with `javap -s`:

```text
initializeNotifier()I
setNotifierName(ILjava/lang/String;)V
stopNotifier(I)V
cleanNotifier(I)V
updateNotifierAlarm(IJ)V
cancelNotifierAlarm(I)V
waitForNotifierAlarm(I)J
```

## Scope

In:

- Add JNI exports:
  - `Java_edu_wpi_first_hal_NotifierJNI_initializeNotifier`
  - `Java_edu_wpi_first_hal_NotifierJNI_setNotifierName`
  - `Java_edu_wpi_first_hal_NotifierJNI_stopNotifier`
  - `Java_edu_wpi_first_hal_NotifierJNI_cleanNotifier`
  - `Java_edu_wpi_first_hal_NotifierJNI_updateNotifierAlarm`
  - `Java_edu_wpi_first_hal_NotifierJNI_cancelNotifierAlarm`
  - `Java_edu_wpi_first_hal_NotifierJNI_waitForNotifierAlarm`
- Adapt each directly to the matching C HAL Notifier function.
- Convert the Java notifier name with the same scoped UTF conversion used by
  Cycle 55.
- Update the JNI artifact export check to require the new symbols.

Out:

- No new Notifier semantics in `shim_core`; blocking/wake behavior remains
  owned by the existing C HAL tests from cycles 25-27.
- No automatic host-side generation of `notifier_alarm_batch`.
- No scheduler/time shortcut to keep Java loops moving without a host.
- No Driver Station mode synthesis.

## v0 decisions

- **D-C56-ADAPTERS-ONLY:** `NotifierJNI` lifecycle methods are thin adapters
  over existing C HAL Notifier functions and do not inspect `shim_core`
  directly.
- **D-C56-STATUS-DROPPED:** Java NotifierJNI methods have no status out-param.
  Return-valued methods keep their C HAL return value (`initializeNotifier`
  handle or `waitForNotifierAlarm` timestamp). Void methods call the C HAL
  function and drop status.
- **D-C56-NO-FALLBACK-CREATION:** These JNI methods do not create the
  cycle-51 fallback shim. Only `HAL.initialize` owns fallback creation.
- **D-C56-NAME-UTF:** `setNotifierName` converts Java strings with
  `GetStringUTFChars` and releases acquired pointers after `HAL_SetNotifierName`
  copies the name. Null Java strings, null envs, or missing
  `GetStringUTFChars` pass null C pointers and inherit the C HAL empty-name
  behavior.
- **D-C56-WAIT-BLOCKING-INHERITS:** `waitForNotifierAlarm` inherits the C HAL
  blocking/wake contract. Cycle 56 tests only already-queued and stop-wake
  paths in JNI so the test suite remains deterministic.

## Tests

### C56-1 - artifact exports NotifierJNI lifecycle symbols

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  the NotifierJNI lifecycle used by `TimedRobot`.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include all seven new
  `Java_edu_wpi_first_hal_NotifierJNI_*` lifecycle/wait methods listed above.
- **Determinism notes:** Static artifact inspection only.

### C56-2 - `initializeNotifier` returns a C HAL notifier handle

- **Layer / contract:** JNI adapter over `HAL_InitializeNotifier`.
- **Bug class caught:** Missing adapter, returning a placeholder handle, or
  bypassing the C HAL notifier table.
- **Inputs:** Installed connected shim; call
  `NotifierJNI.initializeNotifier()`.
- **Expected outputs:** Returns a nonzero handle; `current_notifier_state()`
  has one slot for that handle with inactive alarm and zero name bytes.
- **Determinism notes:** In-process.

### C56-3 - `initializeNotifier` no-shim path returns zero without fallback state

- **Layer / contract:** no-shim JNI behavior inherited from C HAL.
- **Bug class caught:** JNI silently creates fallback state or reports a fake
  valid handle without a shim.
- **Inputs:** Reset JNI launch state, clear `shim_core::current()`, call
  `NotifierJNI.initializeNotifier()`.
- **Expected outputs:** Returns `0`; no JNI fallback state exists; no shim is
  installed.
- **Determinism notes:** In-process.

### C56-4 - `setNotifierName` converts and releases Java name strings

- **Layer / contract:** JNI string ownership and C HAL name update.
- **Bug class caught:** Missing string conversion, wrong handle forwarding,
  leaking UTF chars, or retaining JVM-owned pointers.
- **Inputs:** Installed shim; create a notifier; fake JNI env with name
  `"TimedRobot"`; call `setNotifierName(handle, name)`.
- **Expected outputs:** Notifier slot name is `"TimedRobot"` with C HAL
  zero-fill/truncation rules; one UTF string is acquired and released with the
  matching `jstring` and pointer.
- **Determinism notes:** In-process fake JNI table.

### C56-5 - `setNotifierName` null/minimal paths clear to empty name

- **Layer / contract:** JNI adapter inherits C HAL null-name behavior.
- **Bug class caught:** Dereferencing null env/name, skipping the C HAL call, or
  inventing placeholder text.
- **Inputs:** Installed shim; create notifier; set name to `"previous"`; call
  `setNotifierName(handle, nullptr)` with null env; then set `"again"` and call
  through a fake env missing `GetStringUTFChars`.
- **Expected outputs:** Both calls leave all name bytes zero for the notifier
  slot.
- **Determinism notes:** In-process.

### C56-6 - alarm update, cancel, stop, and clean delegate to C HAL state

- **Layer / contract:** JNI adapters over the C HAL Notifier control plane.
- **Bug class caught:** Wrong handle forwarding, wrong timestamp width, missing
  status-dropping calls, or no-op void adapters.
- **Inputs:** Installed shim; create notifier; call
  `updateNotifierAlarm(handle, 1234567890123)`, then
  `cancelNotifierAlarm(handle)`, then `stopNotifier(handle)`, then
  `cleanNotifier(handle)`.
- **Expected outputs:** After update the slot has `trigger_time_us` equal to the
  64-bit timestamp and `alarm_active=1`; after cancel it has inactive/canceled
  state; stop preserves inactive/canceled state; clean removes the slot.
- **Determinism notes:** In-process.

### C56-7 - `waitForNotifierAlarm` returns already queued matching alarm

- **Layer / contract:** JNI adapter over C HAL wait/drain behavior.
- **Bug class caught:** Not forwarding the handle, truncating the 64-bit return
  value, or returning a placeholder timestamp.
- **Inputs:** Installed shim; create notifier; inject and poll a
  `notifier_alarm_batch` containing an event for that handle at
  `9876543210`.
- **Expected outputs:** `waitForNotifierAlarm(handle)` returns `9876543210`.
- **Determinism notes:** In-process, no blocking because event is queued first.

### C56-8 - `waitForNotifierAlarm` stop wake returns zero

- **Layer / contract:** JNI adapter inherits C HAL stop-wake shape.
- **Bug class caught:** Treating zero timestamp as error in JNI, or not calling
  the C HAL wait path.
- **Inputs:** Installed shim; create notifier; call `stopNotifier(handle)`;
  then call `waitForNotifierAlarm(handle)`.
- **Expected outputs:** Returns `0`.
- **Determinism notes:** In-process, no blocking because stopped state is set
  before wait.

### C56-9 - manual Java launch advances past NotifierJNI startup blocker

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** `scripts/run_timed_robot_smoke.sh`.
- **Expected outputs:** Output no longer contains missing JNI errors for
  `NotifierJNI.initializeNotifier` or the lifecycle methods. The process
  reaches `********** Robot program startup complete **********` and exits
  without a native crash. The likely next missing runtime method is reported
  through the buffered `sendError` path rather than as a console
  `UnsatisfiedLinkError`.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts and timeout behavior.

### C56-10 - shutdown wake remains safe if fallback shim is destroyed before waiter returns

- **Layer / contract:** JNI fallback lifecycle plus C HAL notifier wait
  shutdown wake.
- **Bug class caught:** use-after-free when `HAL.shutdown` wakes a Java daemon
  thread blocked in `NotifierJNI.waitForNotifierAlarm`, then destroys the
  cycle-51 fallback shim before that thread has returned from native code.
- **Inputs:** Create a JNI fallback shim with `HAL.initialize`, initialize one
  notifier, start one async `waitForNotifierAlarm(handle)`, wait until the shim
  reports one pending waiter, call `HAL.shutdown` through JNI, and let the
  waiter return after fallback state is reset.
- **Expected outputs:** The wait returns `0` rather than crashing; fallback
  state is destroyed and no shim remains installed.
- **Determinism notes:** In-process. Uses the shim's pending-wait counter as
  the readiness signal before shutdown; no wall-clock sleeps.
