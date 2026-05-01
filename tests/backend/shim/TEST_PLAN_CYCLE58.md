# Cycle 58 test plan - JNI fallback notifier alarm pump

Status: implemented.

Reviewer: approved after revision.

## Context

Cycle 57 gets the unmodified Java TimedRobot template through startup:

```text
********** Robot program startup complete **********
```

The process then stays alive until the smoke-script timeout because
`TimedRobot` schedules a notifier alarm and calls
`NotifierJNI.waitForNotifierAlarm(...)`, but the cycle-51 JNI-owned fallback
shim has no host endpoint driving inbound `notifier_alarm_batch` messages.

The C HAL notifier wait path from Cycle 27 must remain authoritative: waits
wake from queued protocol `notifier_alarm_batch` events, stop, clean, or
shutdown. Cycle 58 adds only the smallest fallback host-side pump needed for
the repo-owned Java launch path to feed that existing protocol path.

## Scope

In:

- Add a product helper in `hal_jni.cpp` that, when the installed shim is the
  JNI-owned fallback shim and the requested notifier handle has an active alarm,
  sends one inbound `notifier_alarm_batch` through the fallback shared-memory
  region's `core_to_backend` lane and polls the fallback shim.
- Call that helper from
  `Java_edu_wpi_first_hal_NotifierJNI_waitForNotifierAlarm` before delegating
  to `HAL_WaitForNotifierAlarm`.
- Keep the existing C HAL notifier wait semantics unchanged.
- Add focused C++ tests for fallback-only pumping, active-alarm selection,
  repeated alarm updates, JNI wait delegation, and non-fallback/no-active
  behavior.
- Re-run the timed-robot smoke script and verify `robotPeriodic()` output can
  appear without modifying the robot program or bypassing notifier wait.

Out:

- No general Sim Core scheduler or real-time clock model.
- No background host thread.
- No boot-ack synthesis during `HAL.initialize`; the fallback shim still starts
  disconnected until the fallback host pump first needs to deliver normal
  inbound traffic.
- No installed-host shim pumping; a host-provided shim must still receive
  alarms from its host endpoint.
- No Driver Station mode synthesis.
- No changes to `HAL_WaitForNotifierAlarm` itself.

## v0 decisions

- **D-C58-FALLBACK-ONLY:** The pump runs only when
  `shim_core::current()` is exactly the cycle-51 JNI-owned fallback shim.
  Host-installed shims keep normal blocking wait behavior and must be driven by
  their host.
- **D-C58-PROTOCOL-NOT-SHORTCUT:** The pump does not return a timestamp to Java
  directly. It sends a `tick_boundary` +
  `schema_id::notifier_alarm_batch` payload through the fallback
  `core_to_backend` lane, calls `shim.poll()`, and then lets the existing C HAL
  wait drain the queued event.
- **D-C58-FIRST-PUMP-HANDSHAKE:** Protocol-session rules require the backend to
  receive `boot_ack` before normal inbound traffic. The first fallback pump
  call drains the fallback boot envelope on its owned core endpoint, sends a
  real `boot_ack`, polls the fallback shim to connected, and only then sends the
  alarm batch. `HAL.initialize` itself still does not synthesize boot_ack.
- **D-C58-ACTIVE-ALARM-ONLY:** If the handle is missing, cleaned, canceled,
  stopped, or has `alarm_active == 0`, the pump sends nothing and returns
  false.
- **D-C58-EXACT-TRIGGER-TIME:** The synthesized event uses the current notifier
  slot's `trigger_time_us` as both `notifier_alarm_event::fired_at_us` and the
  envelope `sim_time_us`. This is deterministic and advances TimedRobot's loop
  without introducing a wall-clock scheduler.
- **D-C58-ONE-EVENT-PER-WAIT:** Each pump call emits at most one matching event.
  Repeated TimedRobot loops are driven by repeated
  `updateNotifierAlarm(...)` + `waitForNotifierAlarm(...)` calls.
- **D-C58-LANE-BUSY-RETRY:** If the fallback inbound lane is busy, the pump
  polls once and retries the send once. If it still cannot enqueue the event,
  it returns false and the normal wait behavior applies.
- **D-C58-SHUTDOWN-NO-PUMP:** After JNI shutdown clears fallback launch state,
  the pump returns false and creates no new fallback state.

## Tests

### C58-1 - fallback pump enqueues an active alarm through protocol

- **Layer / contract:** JNI fallback host boundary over Layer 2
  `notifier_alarm_batch` protocol.
- **Bug class caught:** shortcutting `waitForNotifierAlarm`, publishing on the
  wrong lane, failing to poll, or not matching the active notifier handle.
- **Inputs:** Reset global state, call JNI `HAL.initialize` to create the
  disconnected fallback shim, initialize one notifier, update its alarm to
  `20'000`, then call the pump helper for that handle.
- **Expected outputs:** Pump returns true; `HAL_WaitForNotifierAlarm` returns
  `20'000` with success status; the fallback shim's
  `latest_notifier_alarm_batch()` is present with `count == 1`,
  matching handle, and `fired_at_us == 20'000`; the fallback shim is connected
  only after the pump's real `boot_ack` is polled.
- **Determinism notes:** No wall clock; timestamp comes from explicit
  `updateNotifierAlarm`.

### C58-2 - fallback pump is inactive without fallback ownership

- **Layer / contract:** Cycle-51 fallback ownership boundary.
- **Bug class caught:** pumping host-installed shims, creating fallback state
  from a wait helper, or altering no-shim behavior.
- **Inputs:** First run with no shim and no fallback. Then install a normal
  connected shim, allocate/update a notifier, and call the pump helper.
- **Expected outputs:** Both calls return false; no fallback launch state is
  created; the host-installed shim has no queued notifier alarm batch.
- **Determinism notes:** In-process only.

### C58-3 - inactive, canceled, stopped, and cleaned alarms do not pump

- **Layer / contract:** notifier handle/alarm state semantics.
- **Bug class caught:** emitting stale alarms after cancel/stop/clean or
  treating allocated-but-inactive notifiers as ready.
- **Inputs:** Create fallback state. For one notifier, call the pump helper
  before any alarm, after canceling an alarm, after stopping an alarm, and after
  cleaning the handle.
- **Expected outputs:** Pump returns false in each inactive state and
  `latest_notifier_alarm_batch()` is still absent unless a previous active pump
  explicitly ran in that subcase.
- **Determinism notes:** In-process only.

### C58-4 - repeated updates pump the latest trigger once per call

- **Layer / contract:** fallback pump repeated-write behavior and existing
  notifier FIFO drain.
- **Bug class caught:** first-trigger-wins state, duplicate event emission per
  pump, stale timestamps, or failure to repoll after the first event.
- **Inputs:** Fallback notifier handle. Update alarm to `20'000`, pump, wait
  and read `20'000`; then update the same handle to `40'000`, pump, wait and
  read `40'000`.
- **Expected outputs:** Both pump calls return true and each wait returns the
  current trigger exactly once.
- **Determinism notes:** No sleeps; explicit timestamps.

### C58-5 - JNI wait pumps fallback alarm before delegating to C wait

- **Layer / contract:** `NotifierJNI.waitForNotifierAlarm` adapter behavior.
- **Bug class caught:** helper exists but the JNI adapter still blocks because
  it does not call the pump before `HAL_WaitForNotifierAlarm`.
- **Inputs:** JNI fallback shim, notifier handle, alarm updated to `60'000`;
  call `Java_edu_wpi_first_hal_NotifierJNI_waitForNotifierAlarm` on a helper
  future. Use the fallback shim's existing `pending_notifier_wait_count(handle)`
  as the in-process signal for the failure path. If the wait enters the pending
  state, stop the notifier, join the future, and fail because the adapter
  delegated without pumping.
- **Expected outputs:** The future is ready before the wait is registered as a
  pending notifier wait, and returns `60'000` through the existing C wait path.
- **Determinism notes:** No sleeps or wall-clock timeouts; the bounded loop
  observes either future readiness or the shim's own pending-wait counter, then
  releases the waiter explicitly on failure.

### C58-6 - busy fallback inbound lane is polled once and retried

- **Layer / contract:** fallback pump backpressure behavior over Tier 1
  `core_to_backend` lane.
- **Bug class caught:** failing to recover from one stale inbound message,
  dropping the alarm when the lane is initially full, or retrying by bypassing
  protocol validation.
- **Inputs:** Create JNI fallback state. Using the fallback-owned core endpoint,
  drain boot, send a real `boot_ack`, and poll the fallback shim so the session
  can legally receive normal traffic. Pre-fill the `core_to_backend` lane with a
  valid `tick_boundary` `ds_state` message, then update a notifier alarm to
  `70'000` and call the pump helper.
- **Expected outputs:** Pump returns true; the fallback shim first accepts the
  prefilled `ds_state`, then accepts the retried `notifier_alarm_batch`; waiting
  for the notifier returns `70'000`.
- **Determinism notes:** In-process transport lane state only.

### C58-7 - shutdown clears fallback pump eligibility

- **Layer / contract:** JNI fallback lifecycle after `HAL.shutdown`.
- **Bug class caught:** use-after-free of fallback region/shim from the pump, or
  hidden fallback recreation after shutdown.
- **Inputs:** Create fallback state, allocate/update notifier, call JNI
  `HAL.shutdown`, then call the pump helper for the old handle.
- **Expected outputs:** Pump returns false; `shim_core::current()` is null; no
  fallback launch state exists.
- **Determinism notes:** In-process only.

### C58-8 - manual timed robot smoke reaches `robotPeriodic`

- **Layer / contract:** unmodified Java robot launch smoke through repo-owned
  HAL/JNI artifacts.
- **Inputs:** `ROBOSIM_TIMED_ROBOT_TIMEOUT=2s scripts/run_timed_robot_smoke.sh`
  against the generated `tools/timed-robot-template` jar.
- **Expected outputs:** Startup still completes and the robot program's
  `robotPeriodic()` print appears at least once.
- **Determinism notes:** Manual smoke because it depends on local JDK/template
  artifacts and process timeout behavior.
