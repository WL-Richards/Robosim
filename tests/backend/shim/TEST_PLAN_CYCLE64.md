# Cycle 64 test plan - host runtime clock/notifier pump

Status: implemented.

Reviewer: requested explicit empty/default notifier behavior coverage and a
clear fallback-log setup in the attached JNI test. Both are incorporated below.

## Context

Cycle 61 added `shim_host_driver`, a real host-side helper that accepts boot,
sends `boot_ack`, and can publish clock/notifier messages. Cycle 63 added the
robot-side background poll loop for `ROBOSIM_TIER1_FD` attached JNI shims. The
remaining fallback for real timed robot execution is the Cycle 58-60 JNI-owned
fallback clock/notifier pump: it reads notifier state directly inside the robot
process and synthesizes clock/alarm traffic there.

Cycle 64 adds the smallest deterministic host runtime pump on top of
`shim_host_driver`. It keeps the runtime driver host-owned, receives robot
notifier-state snapshots through the existing outbound protocol, and publishes
host-authoritative clock ticks plus due notifier alarms back through the
existing inbound protocol. It does not launch a Java process yet.

## Scope

In:

- Add a host runtime driver/pump in the shim host layer.
- The runtime owns or wraps a `shim_host_driver` created over a caller-provided
  Tier 1 region.
- The runtime performs the normal boot accept + `boot_ack` handshake.
- The runtime can receive one robot-to-host `notifier_state` message and cache
  it as the latest host-known notifier table.
- The runtime `pump_to(sim_time_us)` publishes at most one host-to-robot
  message per call:
  - first a default v0 `clock_state` for a new timestamp;
  - then one `notifier_alarm_batch` containing due alarms from the cached
    notifier table after the clock for that timestamp has been accepted.
- The runtime surfaces lane/session failures without hidden retry.
- Tests exercise both direct in-process shim polling and the Cycle-63 attached
  JNI poll loop where that proves fallback removal.

Out:

- No Java child-process launcher changes.
- No wall-clock thread, sleep, or pacing policy.
- No Driver Station packet generation.
- No automatic shim-side flushing of `notifier_state`; tests use the existing
  explicit `flush_notifier_state` boundary to put robot output on the wire.
- No removal of the Cycle 58-60 fallback helpers yet.
- No combined DS-output host-state schema.

## v0 decisions

- **D-C64-HOST-OWNED:** The runtime operates only on a caller-provided Tier 1
  region and never creates or touches JNI fallback launch state.
- **D-C64-ONE-MESSAGE-PUMP:** `pump_to` attempts at most one host-to-robot
  protocol send per call. This matches the single-message Tier 1 lane and makes
  backpressure explicit.
- **D-C64-CLOCK-FIRST:** For a new timestamp, `pump_to` publishes a default
  v0 `clock_state` before any notifier alarm for that timestamp. The default
  clock mirrors the fallback smoke clock fields: `system_active = 1`,
  `system_time_valid = 1`, `rsl_state = 1`, other fields zero.
- **D-C64-DUE-AFTER-CLOCK:** Due notifier alarms for a timestamp are eligible
  only after that timestamp's clock publish has succeeded. If the lane is busy
  on the clock publish, the runtime does not advance to alarm phase.
- **D-C64-NOTIFIER-SNAPSHOT-SOURCE:** The runtime learns notifier alarm state
  only by receiving a real robot-to-host `notifier_state` protocol message.
  It does not inspect a `shim_core` object directly.
- **D-C64-ACTIVE-DUE-ONLY:** A notifier slot is due when
  `alarm_active != 0`, `canceled == 0`, and `trigger_time_us <= sim_time_us`.
  Inactive, canceled, or future-trigger slots are not published.
- **D-C64-TRIGGER-DEDUP:** A `(handle, trigger_time_us)` pair is published at
  most once. A later notifier-state snapshot with the same handle but a new
  trigger time is eligible again.
- **D-C64-NO-FALLBACK:** Runtime clock/alarm publication must not use
  `pump_jni_fallback_notifier_alarm`, fallback clock seeding, or fallback
  pacing state.

## Tests

### C64-1 - host runtime publishes a default clock tick first

- **Layer / contract:** real host runtime clock publication boundary.
- **Bug class caught:** failing to publish a clock, publishing an alarm before
  clock, using fallback clock seeding, or directly mutating the shim cache.
- **Inputs:** Create a shared region, backend shim, and host runtime. Accept
  boot, poll the shim connected, install it globally, then call
  `pump_to(20'000)`.
- **Expected outputs:** The pump reports `clock_published`; before shim poll,
  `HAL_GetFPGATime` remains `0`; after shim poll, the cached clock equals the
  v0 default clock at `20'000` and `HAL_GetFPGATime` returns `20'000`.
  JNI fallback state remains absent.
- **Determinism notes:** In-process only; no sleeps.

### C64-2 - host runtime receives notifier state only through protocol

- **Layer / contract:** robot-to-host notifier-state intake.
- **Bug class caught:** runtime reading `shim_core::current_notifier_state()`
  directly, using fallback launch state, wrong schema dispatch, or stale cache.
- **Inputs:** Complete host runtime boot. Allocate two notifiers through the
  C HAL surface, update one active alarm, call `shim.flush_notifier_state` to
  publish the robot output, then call `runtime.poll_robot_outputs()`.
- **Expected outputs:** The runtime reports `notifier_state_received`; its
  latest cached notifier state byte-equals the flushed snapshot. No host-to-
  robot clock/alarm is published by `poll_robot_outputs`.
- **Determinism notes:** Explicit protocol flush; no wall clock.

### C64-3 - due cached notifier alarm wakes an attached JNI wait through host pump

- **Layer / contract:** real host replacement for fallback notifier pump.
- **Bug class caught:** fallback pump reuse, direct wait wake, alarm before
  clock, failure to work with the Cycle-63 attached poll loop, or missed due
  alarm publication.
- **Inputs:** Create a host Tier 1 mapping, set `ROBOSIM_TIER1_FD`, initialize
  JNI so the robot-side attached poll loop starts, create the host runtime over
  the same mapping, accept boot, initialize a notifier through JNI, update its
  alarm to `40'000`, explicitly flush notifier state from the attached shim,
  and let the runtime receive that robot output. Install the existing fallback
  sleep recorder before the attached launch so the test can assert it remains
  unused. Start async `NotifierJNI.waitForNotifierAlarm`.
- **Expected outputs:** First `pump_to(40'000)` publishes the clock; after the
  attached poll loop accepts the clock, the second `pump_to(40'000)` publishes
  one notifier alarm. The Java wait future returns `40'000`; fallback launch
  state remains non-fallback and fallback sleep/pacing logs remain empty.
- **Determinism notes:** Bounded yield loops on observable shim/JNI state; no
  sleeps or wall-clock timing assertions.

### C64-4 - inactive canceled and future notifier slots are not due

- **Layer / contract:** notifier due filtering.
- **Bug class caught:** firing canceled/stopped/inactive alarms or firing future
  trigger times early.
- **Inputs:** Runtime caches a notifier_state with four slots: one inactive,
  one canceled, one active at `60'000`, and one active at `100'000`. Pump to
  `60'000`, draining the clock before the alarm phase.
- **Expected outputs:** Only the active non-canceled `60'000` slot is published
  in the alarm batch. The future `100'000` slot is not included.
- **Determinism notes:** In-process protocol lane state only.

### C64-5 - no cached or empty notifier state produces no alarm

- **Layer / contract:** empty/default host runtime notifier behavior.
- **Bug class caught:** publishing garbage alarms from missing/default state,
  treating an empty notifier snapshot as a due alarm, or creating fallback
  state to compensate for no due alarms.
- **Inputs:** Complete runtime boot. Call `pump_to(20'000)` and drain the
  clock. With no cached notifier state, call `pump_to(20'000)` again. Then
  publish a zero-count `notifier_state` from the shim via
  `flush_notifier_state`, call `runtime.poll_robot_outputs()`, and call
  `pump_to(40'000)` through clock and alarm phases.
- **Expected outputs:** The no-cache alarm phase reports `no_due_alarm` and
  sends nothing. The zero-count snapshot is cached, the `40'000` clock publishes
  normally, and the following alarm phase again reports `no_due_alarm` with no
  notifier alarm batch delivered to the shim. Fallback state remains absent.
- **Determinism notes:** In-process protocol lane state only.

### C64-6 - repeated pump does not republish the same trigger

- **Layer / contract:** host runtime notifier dedup.
- **Bug class caught:** repeatedly waking the same notifier from one stale
  notifier-state snapshot, or suppressing a new trigger for the same handle.
- **Inputs:** Cache a notifier snapshot for handle H at `20'000`, pump clock
  and alarm at `20'000`, drain them, then call the alarm phase again with the
  same cached snapshot. Next cache a new snapshot for H at `40'000`, pump to
  `40'000`.
- **Expected outputs:** The second pump at `20'000` reports no due alarm and
  sends nothing. The `40'000` snapshot is eligible and publishes one new alarm.
- **Determinism notes:** No sleeps; explicit cache updates.

### C64-7 - lane busy does not advance runtime phase

- **Layer / contract:** host runtime backpressure.
- **Bug class caught:** hidden retry, skipped clock phase, or dropped alarm
  phase after a busy lane.
- **Inputs:** Complete boot, pre-fill the host-to-robot lane with a valid clock
  using a prior runtime pump, then call `pump_to(80'000)` before the shim drains
  the lane.
- **Expected outputs:** The pump returns the `lane_busy` transport error and
  does not mark the `80'000` clock as published. After the shim drains the
  earlier message, another `pump_to(80'000)` publishes the `80'000` clock rather
  than advancing directly to notifier alarms.
- **Determinism notes:** Single-lane protocol state only.
