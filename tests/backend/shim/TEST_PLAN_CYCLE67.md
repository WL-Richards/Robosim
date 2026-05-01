# Cycle 67 test plan - attached NotifierJNI state publication

Status: implemented.

Reviewer round 1 was `not-ready`: stop/clean publication and successful-
mutation-only behavior were in scope but not covered. The revised suite adds
those checks, and C67-1 now compares the received state to the shim's current
snapshot rather than accepting a partial hand-built message.

## Context

Cycle 65 put the timed-robot smoke script on the attached host runtime, and
Cycle 66 added diagnostics. Manual smoke with the attached launcher now shows
boot/startup diagnostics but stalls before repeated `robotPeriodic()`: the Java
TimedRobot path calls `NotifierJNI.updateNotifierAlarm`, but the attached shim
does not automatically publish the updated `notifier_state` to the host. Cycle
64 tests used explicit `shim.flush_notifier_state(...)`, which is not available
to unmodified Java robot code.

Cycle 67 adds the smallest attached-JNI publication boundary needed for
unmodified TimedRobot notifier scheduling: after successful `NotifierJNI`
alarm updates/cancels/stops/cleans, an env-attached shim publishes its current
`notifier_state` to the host through the existing robot-to-host protocol. This
is not fallback behavior and does not change C HAL semantics.

## Scope

In:

- Attached-JNI `NotifierJNI.updateNotifierAlarm` flushes current notifier state
  after a successful C HAL update.
- Attached-JNI cancel/stop/clean operations also flush after successful state
  mutation so the host can observe deactivation/removal.
- No-env fallback launches do not publish this attached host output.
- Host-installed/non-JNI C HAL calls still require explicit
  `flush_notifier_state`; this cycle is only the Java attached runtime boundary.

Out:

- No new wire schema.
- No host runtime policy changes.
- No Driver Station input/control changes.
- No automatic flushing from every C HAL notifier call.

## v0 decisions

- **D-C67-JNI-ATTACHED-ONLY:** Automatic notifier-state publication lives at the
  attached JNI adapter boundary, not the C HAL boundary. This keeps unit tests
  and non-JNI host-installed shims deterministic with explicit flush calls.
- **D-C67-SUCCESSFUL-MUTATION-ONLY:** The adapter flushes only after the
  delegated C HAL operation succeeds. Invalid handles/no-shim paths do not
  publish compensating snapshots.
- **D-C67-BEST-EFFORT-FLUSH:** Java NotifierJNI methods have no status return
  for these operations, so a lane-busy flush failure is not surfaced to Java.
  The shim state remains updated and a later NotifierJNI mutation can publish a
  fresh snapshot.

## Tests

### C67-1 - attached NotifierJNI update publishes notifier state to host

- **Layer / contract:** attached Java notifier scheduling boundary.
- **Bug class caught:** TimedRobot stalls because the host never receives the
  notifier alarm table after Java schedules an alarm.
- **Inputs:** Create an attached JNI fixture and host runtime over the same
  mapping. Accept boot. Initialize a notifier via `NotifierJNI`, then call
  `NotifierJNI.updateNotifierAlarm(handle, 20'000)`. Do not call
  `shim.flush_notifier_state` from the test.
- **Expected outputs:** `runtime.poll_robot_outputs()` receives
  `notifier_state_received`; the cached state contains one active slot with the
  handle and trigger `20'000`, and byte-equals the attached shim's
  `current_notifier_state()`. No fallback launch state exists.
- **Determinism notes:** In-process attached mapping only.

### C67-2 - attached NotifierJNI cancel and stop publish deactivated notifier state

- **Layer / contract:** attached Java notifier deactivation boundary.
- **Bug class caught:** host keeps firing a stale alarm after Java cancels or
  stops it.
- **Inputs:** Attached JNI fixture. Initialize/update a notifier and drain the
  update snapshot on the host. Then call `NotifierJNI.cancelNotifierAlarm`,
  drain that snapshot, update again, drain the update snapshot, and call
  `NotifierJNI.stopNotifier`.
- **Expected outputs:** The next host poll receives a notifier state where the
  slot for the handle is present but canceled/inactive according to existing
  notifier table semantics after cancel. After stop, the next host poll receives
  a state that byte-equals the shim's current stopped snapshot and cannot be
  interpreted by the host runtime as due. No fallback state exists.
- **Determinism notes:** In-process attached mapping only.

### C67-3 - attached NotifierJNI clean publishes notifier removal

- **Layer / contract:** attached Java notifier lifecycle/removal boundary.
- **Bug class caught:** host retains a removed notifier slot after Java cleans
  it.
- **Inputs:** Attached JNI fixture. Initialize/update a notifier, drain the
  update snapshot, then call `NotifierJNI.cleanNotifier`.
- **Expected outputs:** The next host poll receives a notifier state that
  byte-equals the shim's current snapshot and has `count == 0` for the cleaned
  single-notifier table. No fallback state exists.
- **Determinism notes:** In-process attached mapping only.

### C67-4 - invalid attached NotifierJNI mutation does not publish a snapshot

- **Layer / contract:** successful-mutation-only publication boundary.
- **Bug class caught:** invalid Java notifier handles produce compensating or
  misleading host snapshots.
- **Inputs:** Attached JNI fixture and host runtime. Accept boot, then call
  `NotifierJNI.updateNotifierAlarm(9999, 20'000)` with an invalid handle.
- **Expected outputs:** `runtime.poll_robot_outputs()` reports no message; the
  host runtime has no cached notifier state; no fallback state exists.
- **Determinism notes:** In-process attached mapping only.

### C67-5 - attached NotifierJNI update can wake a host-runtime smoke step without explicit flush

- **Layer / contract:** real TimedRobot periodic path through attached host
  runtime.
- **Bug class caught:** tests pass only because they call private
  `flush_notifier_state`, while unmodified Java robot code still stalls.
- **Inputs:** Attached JNI fixture and `timed_robot_smoke_host_runtime`.
  Accept boot, initialize/update notifier via JNI, start async JNI wait, then
  run runtime steps at `20'000` with the robot poll loop active.
- **Expected outputs:** The runtime first receives robot output, then publishes
  clock and notifier alarm; the JNI wait returns `20'000`. Fallback sleep/pacing
  logs remain empty.
- **Determinism notes:** Bounded yield loops on observable state; no sleeps.

### C67-6 - repeated attached NotifierJNI updates can wake successive smoke ticks

- **Layer / contract:** repeated TimedRobot periodic scheduling through attached
  host runtime.
- **Bug class caught:** first periodic wake works, but the next Java update is
  dropped or deduplicated incorrectly so the smoke stalls after one
  `robotPeriodic()`.
- **Inputs:** Attached JNI fixture and `timed_robot_smoke_host_runtime`.
  Accept boot, initialize/update notifier to `20'000`, wait and pump it through
  the runtime, then update the same handle to `40'000` without explicit flush
  and start a second wait. Run runtime steps at `20'000` until no-due, then at
  `40'000`.
- **Expected outputs:** First wait returns `20'000`; second wait returns
  `40'000`. Runtime counters show two robot outputs, two clocks, and two
  notifier alarms. Fallback sleep/pacing logs remain empty.
- **Determinism notes:** In-process attached mapping only; bounded wait loops.

## Manual verification

- Run:

  ```bash
  ROBOSIM_TIMED_ROBOT_TIMEOUT=2s scripts/build_and_run_timed_robot_smoke.sh
  ```

- Expected: diagnostics show attached boot, and the empty TimedRobot reaches
  repeated `robotPeriodic()` through the attached host runtime.
