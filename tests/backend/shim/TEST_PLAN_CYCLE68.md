# Cycle 68 test plan — attached smoke host cadence waits for active notifier trigger

## Slice

Cycle 68 tightens the attached timed-robot smoke host cadence policy without
adding new HAL surfaces or protocol schemas. Cycle 67 proved unmodified
TimedRobot can publish attached NotifierJNI `notifier_state` updates and wake
through host-published alarms, but manual smoke diagnostics still show many
more clock/no-due phases than notifier alarms. The smallest coherent fix is to
make `timed_robot_smoke_host_runtime` report that it is waiting for a known
future active Notifier trigger instead of treating an old `pump_to(...)`
timestamp as a no-due alarm.

This is host-runtime behavior only:

- No changes to `shim_host_runtime_driver::pump_to(...)`; it remains a generic
  nonblocking clock/notifier pump.
- No changes to C HAL or JNI Notifier semantics.
- No wall-clock sleeps in unit tests.
- Fallback-owned timing remains unchanged.

## v0 behavior decisions

- A new smoke-host step result, `waiting_for_notifier_trigger`, is returned
  when the runtime has a cached active, non-canceled notifier trigger whose
  time is greater than the caller's `sim_time_us`.
- Returning `waiting_for_notifier_trigger` does not publish a clock, does not
  publish a notifier alarm, and does not increment `no_due_count`.
- The runtime still polls exactly one robot-output message before making the
  waiting decision. Fresh NotifierJNI updates therefore take effect before a
  wait/no-due decision.
- Empty, missing, inactive, or canceled notifier state still falls through to
  the existing `no_due_alarm` behavior.
- Repeated calls at the same old time keep returning
  `waiting_for_notifier_trigger` without side effects until the caller advances
  to the trigger time.
- At the trigger time, the existing two-phase host behavior is preserved:
  first publish clock, then publish notifier alarm.
- Shutdown, lane-busy, boot, and no-shim behavior are unchanged.

## Proposed tests

### C68-1 `TimedRobotSmokeHostCadence.FutureActiveNotifierDoesNotPublishNoDueClockChurn`

- **Layer / contract:** Layer 2 HAL shim host smoke runtime cadence boundary.
- **Bug class:** The runtime treats a known future active notifier as a no-due
  alarm at the old timestamp, incrementing `no_due_count` and allowing the
  launcher to spin/publish unnecessary clocks before the robot's next TimedRobot
  period.
- **Setup:** Attached JNI fixture with host-created Tier 1 mapping; accept boot
  through `timed_robot_smoke_host_runtime`; initialize one notifier through the
  Java JNI attached path; publish an alarm update at `40'000us`.
- **Inputs:** Call `runtime.step(20'000)` to receive the robot output, then call
  `runtime.step(20'000)` again.
- **Expected outputs:** Second call returns
  `timed_robot_smoke_host_step::waiting_for_notifier_trigger`; the robot shim
  has no new clock state and no notifier alarm batch; counters show one robot
  output and zero clocks/alarms/no-due.
- **Determinism:** No sleeps; all communication is explicit shared-memory
  polling.

### C68-2 `TimedRobotSmokeHostCadence.RepeatedOldTimeWaitsAreSideEffectFreeUntilTrigger`

- **Layer / contract:** Layer 2 HAL shim host smoke runtime cadence boundary.
- **Bug class:** Repeated old-time calls publish duplicate clock/no-due phases
  or advance counters while waiting for a future Notifier trigger.
- **Setup:** Same attached JNI fixture and future trigger at `40'000us`.
- **Inputs:** After consuming the robot output, call `runtime.step(20'000)`
  twice, then call `runtime.step(40'000)` twice.
- **Expected outputs:** Both old-time calls return
  `waiting_for_notifier_trigger`; counters remain unchanged across those calls.
  At `40'000us`, the first call returns `clock_published`, the attached shim
  observes `HALUtil.getFPGATime() == 40'000`, and the second call returns
  `notifier_alarm_published`.
- **Determinism:** No wall clock or sleeps; uses explicit poll/wait helpers
  already present in the shim suite for attached JNI state propagation.

### C68-3 `TimedRobotSmokeHostCadence.MissingEmptyInactiveAndCanceledNotifierStateStillReportNoDue`

- **Layer / contract:** Layer 2 HAL shim host smoke runtime cadence boundary.
- **Bug class:** The new waiting branch masks the existing no-due behavior for
  missing, empty, inactive, or canceled notifier state, which would hide a
  legitimately idle robot or incorrectly stall the host runtime.
- **Setup:** Host-created Tier 1 mapping with an attached shim and
  `timed_robot_smoke_host_runtime`; accept boot and poll the robot-side ack.
- **Inputs and expected outputs:**
  - With no cached notifier state, `runtime.step(20'000)` returns
    `clock_published`, then a second `runtime.step(20'000)` returns
    `no_due_alarm`; `no_due_count == 1`.
  - Publish and consume an empty `notifier_state{count == 0}` snapshot. At
    `40'000us`, the runtime returns `clock_published` then `no_due_alarm`, not
    `waiting_for_notifier_trigger`; `no_due_count == 2`.
  - Publish and consume a snapshot with one inactive, not-canceled future slot
    (`alarm_active == 0`, `canceled == 0`, trigger `80'000us`). At `60'000us`,
    the runtime returns `clock_published` then `no_due_alarm`, not
    `waiting_for_notifier_trigger`; `no_due_count == 3`.
  - Publish and consume a snapshot with one active, canceled future slot
    (`alarm_active == 1`, `canceled == 1`, trigger `100'000us`). At `80'000us`,
    the runtime returns `clock_published` then `no_due_alarm`, not
    `waiting_for_notifier_trigger`; `no_due_count == 4`.
- **Determinism:** Pure in-process protocol steps; no sleeps.

### C68-4 `TimedRobotSmokeHostCadence.ResumeAfterRobotOutputRequiresFutureTrigger`

- **Layer / contract:** Layer 2 timed-robot smoke launcher cadence policy.
- **Bug class:** After publishing an alarm, the launcher waits for the robot to
  publish a new notifier state, but it resumes when it sees a stale
  already-fired trigger equal to or older than the current sim time. That sends
  the main loop back into clock/no-due churn before the robot publishes the next
  TimedRobot alarm.
- **Setup:** Pure helper coverage for the wait-release predicate used by
  `robosim_timed_robot_smoke_host` after `poll_robot_outputs_only()`.
- **Inputs:** Evaluate the predicate with no trigger, a stale trigger less than
  `sim_time_us`, an equal trigger, and a future trigger greater than
  `sim_time_us`.
- **Expected outputs:** Only the future trigger releases the launcher from
  robot-output wait and yields the resume time. Missing, stale, and equal
  triggers keep waiting and produce no resume time.
- **Determinism:** Pure value test; no process launch, no wall clock.

### C68-5 `TimedRobotSmokeHostCadence.InitialRobotOutputCanReleaseWithoutFutureTrigger`

- **Layer / contract:** Layer 2 timed-robot smoke launcher startup cadence
  policy.
- **Bug class:** Reusing the stricter post-alarm wait-release rule during
  startup can deadlock the smoke host after it consumes an initial notifier
  snapshot that has no future active trigger yet; the host never publishes the
  first clock needed for TimedRobot scheduling to progress.
- **Setup:** Pure helper coverage for the startup wait-release predicate used
  after boot acceptance.
- **Inputs:** Evaluate the startup predicate with no trigger, stale trigger,
  equal trigger, and future trigger against `sim_time_us == 20'000`.
- **Expected outputs:** Missing, stale, and equal triggers release at the
  current sim time; a future trigger releases at that future time. This keeps
  startup moving while still allowing a prepublished future trigger to align the
  first clock.
- **Determinism:** Pure value test; no process launch, no wall clock.

## Reviewer notes

The tests intentionally target `timed_robot_smoke_host_runtime` and a pure
launcher wait-release helper, not process stderr/timeouts, because the cadence
bug is deterministic at those boundaries and process-level tests would be
weaker.
