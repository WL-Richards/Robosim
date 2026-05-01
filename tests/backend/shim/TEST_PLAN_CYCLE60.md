# Cycle 60 test plan - JNI fallback real-time pacing driver

Status: implemented.

Reviewer: approved after revision.

## Context

Cycle 59 gets the generated TimedRobot template into `robotPeriodic()`, but the
fallback runtime loop is unpaced. The fallback notifier pump immediately
publishes each active alarm's `clock_state` and `notifier_alarm_batch`, so
`TimedRobot` spins as fast as the process can run.

Cycle 60 adds the smallest real-time driver for this fallback-only Java smoke
path: before a fallback notifier alarm is delivered, the JNI fallback host
driver waits for the real-time delta corresponding to the alarm's simulated
time delta. The C HAL wait path and the protocol traffic remain unchanged.

## Scope

In:

- Add fallback-owned real-time pacing state to `hal_jni.cpp`.
- Let fallback `HALUtil.getFPGATime()` seeding establish the real-time origin at
  deterministic sim time `20'000us` without sleeping.
- Before the fallback notifier pump publishes a clock/alarm at
  `fired_at_us`, compute the positive simulated-time delta since the previous
  paced fallback time and sleep for that delta.
- Use `std::this_thread::sleep_for(std::chrono::microseconds(delta))` in
  production.
- Add a test-only injectable sleep hook so unit tests assert requested delays
  without wall-clock sleeps.

Out:

- No host-installed shim pacing.
- No background thread or scheduler.
- No changes to C `HAL_WaitForNotifierAlarm`.
- No changes to protocol schemas.
- No attempt to compensate for OS scheduling jitter in v0.
- No wall-clock assertions in CTest.

## v0 decisions

- **D-C60-FALLBACK-ONLY:** Pacing runs only when the installed shim is the
  JNI-owned fallback shim. Host-installed shims still rely on their host.
- **D-C60-PACE-BEFORE-PUBLISH:** The sleep happens before publishing the
  fallback clock/alarm pair, so Java wakes only after the real-time interval has
  elapsed.
- **D-C60-SEED-NO-SLEEP:** The Cycle-59 initial fallback
  `HALUtil.getFPGATime()` seed at `20'000us` initializes the pacing origin and
  does not sleep.
- **D-C60-DELTA-SLEEP:** For each later fallback notifier alarm, sleep for
  `fired_at_us - last_paced_sim_time_us` when the delta is positive, then set
  `last_paced_sim_time_us = fired_at_us`.
- **D-C60-NONMONOTONIC-NO-SLEEP:** If the next alarm time is less than or equal
  to the last paced time, publish immediately and update the last paced time to
  the new value. This preserves existing deterministic protocol behavior for
  odd inputs without introducing negative sleeps.
- **D-C60-TEST-HOOK:** Tests can replace the sleep function with an in-process
  recorder. Resetting JNI fallback state also restores the production sleeper
  and clears pacing state.

## Tests

### C60-1 - initial fallback clock seed establishes pacing origin without sleeping

- **Layer / contract:** fallback real-time pacing origin.
- **Bug class caught:** sleeping during TimedRobot construction, failing to
  initialize pacing origin, or seeding from wall-clock time instead of explicit
  sim time.
- **Inputs:** Install a test sleep recorder, create JNI fallback state, call
  `Java_edu_wpi_first_hal_HALUtil_getFPGATime`.
- **Expected outputs:** JNI returns `20'000`; recorder saw no sleeps; a
  test-visible `last_paced_sim_time_us` reports `20'000`.
- **Determinism notes:** No wall clock; injected recorder only.

### C60-2 - fallback notifier pump sleeps for the positive simulated delta before alarm publish

- **Layer / contract:** fallback notifier pacing.
- **Bug class caught:** no pacing, sleeping after wake publication, wrong delta
  source, or using absolute alarm time as delay.
- **Inputs:** Seed fallback clock at `20'000`; initialize notifier; update alarm
  to `40'000`; call the notifier pump with a recorder installed. The recorder
  callback also inspects fallback state at sleep time.
- **Expected outputs:** Recorder sees exactly one sleep request of `20'000us`;
  while inside that recorder callback, fallback `latest_clock_state()` is still
  `20'000` and `latest_notifier_alarm_batch()` is still absent, proving the
  sleep happened before publication. After the pump returns,
  `HAL_WaitForNotifierAlarm` returns `40'000`; fallback clock is `40'000`.
- **Determinism notes:** No wall clock.

### C60-3 - repeated fallback notifier pumps sleep for per-cycle deltas

- **Layer / contract:** repeated pacing state.
- **Bug class caught:** always sleeping from origin, never advancing last paced
  time, or accumulating duplicate sleeps.
- **Inputs:** Seed at `20'000`; pump/wait alarm `40'000`; then update and
  pump/wait alarm `60'000`.
- **Expected outputs:** Recorder saw sleeps `[20'000, 20'000]`; waits return
  `40'000` then `60'000`.
- **Determinism notes:** No wall clock.

### C60-4 - nonmonotonic fallback alarm does not request a negative sleep

- **Layer / contract:** fallback pacing boundary case.
- **Bug class caught:** unsigned underflow, huge sleep after a backwards alarm,
  or rejecting otherwise valid protocol publication.
- **Inputs:** Seed at `20'000`; pump alarm `40'000`; then update and pump the
  same handle at `35'000`.
- **Expected outputs:** Recorder saw only `[20'000]`; second wait returns
  `35'000`; last paced sim time becomes `35'000`.
- **Determinism notes:** No wall clock.

### C60-5 - no-shim and host-installed notifier pump remain unpaced and inactive

- **Layer / contract:** fallback ownership boundary.
- **Bug class caught:** sleeping, creating fallback state, or publishing for
  no-shim/host-installed paths through the fallback helper.
- **Inputs:** With no shim installed, install recorder and call
  `pump_jni_fallback_notifier_alarm(1)`. Then install a normal connected shim,
  update an active notifier alarm, keep the recorder installed, and call
  `pump_jni_fallback_notifier_alarm` for that handle.
- **Expected outputs:** Both calls return false; recorder saw no sleeps; the
  no-shim phase creates no fallback state; host clock and notifier-alarm caches
  remain empty; no fallback state exists.
- **Determinism notes:** In-process only.

### C60-6 - resetting fallback state clears pacing test hook and last time

- **Layer / contract:** fallback lifecycle cleanup.
- **Bug class caught:** test hook leaking between tests or stale pacing state
  after `HAL.shutdown`.
- **Inputs:** Install recorder, seed fallback clock, reset fallback state, then
  create a new fallback and seed again without reinstalling the recorder.
- **Expected outputs:** First seed sets last paced time to `20'000` and the
  test-visible sleep-hook observer reports non-default while the recorder is
  installed. After reset, last paced time is absent and
  `hal_jni_fallback_sleep_hook_is_default_for_test()` reports true. A second
  fallback seed reinitializes last paced time to `20'000`.
- **Determinism notes:** No wall clock sleeps are triggered in this test because
  it only seeds.

### C60-7 - manual timed robot smoke is visibly paced

- **Layer / contract:** Java smoke behavior.
- **Inputs:** `ROBOSIM_TIMED_ROBOT_TIMEOUT=1s scripts/run_timed_robot_smoke.sh`.
- **Expected outputs:** Startup completes and `Robot Periodic Run!` appears, but
  output count is roughly bounded by the 20ms period rather than hundreds of
  thousands of lines.
- **Tolerance:** Manual smoke only; OS scheduling jitter makes exact counts
  inappropriate for CTest.
