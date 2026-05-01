# Cycle 59 test plan - JNI fallback clock pump for TimedRobot

Status: implemented.

Reviewer: approved after revision.

## Context

Cycle 58 added a fallback-owned notifier alarm pump that feeds
`notifier_alarm_batch` through the real protocol path before
`NotifierJNI.waitForNotifierAlarm(...)` delegates to the C HAL wait.

Manual smoke then advanced through robot construction and `robotInit()`, but
still did not reach `robotPeriodic()`. Decompiling the WPILib 2026.2.1
`TimedRobot` bytecode shows why: its constructor reads
`RobotController.getFPGATime()` to seed callback expiration times. In the
cycle-51 fallback launch there is no inbound `clock_state`, so
`HAL_GetFPGATime` returns the Cycle-12 empty-cache value `0`. The first
periodic callback is therefore scheduled at alarm time `0`, and
`TimedRobot.startCompetition()` treats a `waitForNotifierAlarm()` return value
of `0` as the stop sentinel.

Cycle 59 adds only the fallback host-side clock publication needed for Java
startup. The C HAL `HAL_GetFPGATime` empty-cache semantics for ordinary
installed shims remain unchanged.

## Scope

In:

- Add a fallback-only clock pump in `hal_jni.cpp` that publishes a real inbound
  `tick_boundary` + `clock_state` through the JNI fallback core endpoint and
  polls the fallback shim.
- Call the clock pump from
  `Java_edu_wpi_first_hal_HALUtil_getFPGATime` before delegating to
  `HAL_GetFPGATime`.
- Have the Cycle-58 notifier alarm pump also publish a fallback clock state at
  the alarm's `fired_at_us` before it publishes the alarm batch, so
  `RobotController.getFPGATime()` inside `loopFunc()` observes the same
  deterministic simulated time as the alarm that woke the loop.
- Keep host-installed shim and no-shim behavior unchanged.

Out:

- No wall-clock scheduler.
- No background clock thread.
- No changes to C `HAL_GetFPGATime`.
- No global Sim Core time model.
- No fallback clock pumping from unrelated C HAL readers.

## v0 decisions

- **D-C59-FALLBACK-ONLY:** Clock pumping runs only when
  `shim_core::current()` is exactly the JNI-owned fallback shim. Host-installed
  shims still require host-provided `clock_state`; no-shim returns the existing
  handle-error/zero behavior.
- **D-C59-PROTOCOL-NOT-SHORTCUT:** The pump does not return a timestamp
  directly from JNI. It sends a real `clock_state` payload over
  `core_to_backend`, polls the fallback shim, and then lets
  `HAL_GetFPGATime` read `latest_clock_state()`.
- **D-C59-INITIAL-TIME:** The first fallback Java `HALUtil.getFPGATime()` with
  an empty clock cache publishes deterministic `sim_time_us == 20'000`. This
  avoids TimedRobot's `0` stop sentinel without using wall clock.
- **D-C59-NO-OVERRIDE-CACHED-CLOCK:** If the fallback already has a
  `clock_state`, `HALUtil.getFPGATime()` does not publish a replacement just to
  satisfy a read; it delegates to the C HAL cached value.
- **D-C59-NOTIFIER-ADVANCES-CLOCK:** When the fallback notifier pump emits an
  alarm for an active handle, it first publishes `clock_state.sim_time_us ==
  fired_at_us`, then publishes the alarm. Repeated notifier waits therefore
  advance the fallback clock to the loop's deterministic alarm time.
- **D-C59-HANDSHAKE-INHERITS:** Clock pumping uses the Cycle-58 fallback
  first-pump handshake: drain boot, send real `boot_ack`, and poll connected
  before normal inbound traffic. `HAL.initialize` itself remains disconnected.

## Tests

### C59-1 - fallback `HALUtil.getFPGATime` seeds clock through protocol

- **Layer / contract:** JNI fallback host clock boundary over Layer 2
  `clock_state`.
- **Bug class caught:** direct JNI timestamp shortcut, missing handshake,
  leaving fallback clock empty, or returning the C HAL empty-cache zero.
- **Inputs:** Reset global state, call JNI `HAL.initialize`, assert fallback is
  disconnected and has no clock cache, then call
  `Java_edu_wpi_first_hal_HALUtil_getFPGATime`.
- **Expected outputs:** JNI returns `20'000`; fallback is connected; fallback
  `latest_clock_state()` is present and byte-equal to `valid_clock_state(20'000)`:
  `system_active == 1`, `system_time_valid == 1`, `rsl_state == 1`,
  `browned_out == 0`, `fpga_button_latched == 0`, and
  `comms_disable_count == 0`.
- **Determinism notes:** No wall clock.

### C59-2 - no-shim and host-installed `HALUtil.getFPGATime` do not pump fallback clock

- **Layer / contract:** fallback ownership boundary and existing C HAL
  `HAL_GetFPGATime` behavior.
- **Bug class caught:** creating fallback state from a utility read, changing
  host-installed empty-cache semantics, or hiding missing host clock traffic.
- **Inputs:** First call JNI `HALUtil.getFPGATime` with no shim. Then install a
  normal connected shim with empty clock cache and call the JNI method again.
- **Expected outputs:** No-shim returns `0` and creates no fallback. Host shim
  returns `0`, status behavior remains inherited from C HAL, no fallback state
  exists, and the host shim clock cache remains empty.
- **Determinism notes:** In-process only.

### C59-3 - fallback `HALUtil.getFPGATime` does not override cached clock

- **Layer / contract:** latest host-provided clock state remains authoritative.
- **Bug class caught:** every Java clock read resetting time to the initial v0
  seed.
- **Inputs:** Create fallback state, use the fallback-owned core endpoint to
  complete the real handshake, publish a `clock_state` with
  `sim_time_us == 123'456`, poll it, then call JNI `HALUtil.getFPGATime`.
- **Expected outputs:** JNI returns `123'456`; cached clock state remains byte
  equal to the provided state.
- **Determinism notes:** In-process protocol message.

### C59-4 - notifier fallback pump advances clock before queuing alarm

- **Layer / contract:** fallback host time coherence between notifier alarms
  and `HAL_GetFPGATime`.
- **Bug class caught:** alarm wake succeeds but loop sees stale FPGA time;
  alarm pump queues only `notifier_alarm_batch` and not the corresponding
  `clock_state`.
- **Inputs:** Create fallback state, initialize notifier, update alarm to
  `40'000`, call the notifier pump, then call `HAL_GetFPGATime` and
  `HAL_WaitForNotifierAlarm`.
- **Expected outputs:** `HAL_GetFPGATime` returns `40'000`; wait returns
  `40'000`; fallback latest clock state and latest notifier alarm batch are
  both present.
- **Determinism notes:** Explicit timestamp only.

### C59-5 - host-installed notifier pump remains inactive and does not publish clock

- **Layer / contract:** Cycle-58 notifier pump fallback ownership boundary plus
  Cycle-12 ordinary-shim empty-clock behavior.
- **Bug class caught:** extending the notifier clock side effect to
  host-installed shims, hiding missing host clock traffic, or creating fallback
  state from a host notifier.
- **Inputs:** Install a normal connected shim with empty clock and notifier
  alarm caches. Allocate a notifier, update its alarm to `40'000`, then call
  the notifier pump helper for that handle.
- **Expected outputs:** Pump returns false; no fallback launch state exists;
  host `latest_clock_state()` remains empty; host
  `latest_notifier_alarm_batch()` remains empty.
- **Determinism notes:** In-process only.

### C59-6 - repeated notifier pumps advance clock to each trigger

- **Layer / contract:** repeated fallback time advancement.
- **Bug class caught:** first-trigger-wins clock, stale cache after subsequent
  waits, or duplicate event behavior.
- **Inputs:** Fallback notifier handle. Pump/wait at `20'000`, then update and
  pump/wait at `40'000`.
- **Expected outputs:** After each pump, `HAL_GetFPGATime` returns that pump's
  trigger and the wait returns that same trigger exactly once.
- **Determinism notes:** No sleeps.

### C59-7 - manual timed robot smoke reaches `robotPeriodic`

- **Layer / contract:** unmodified Java robot launch smoke through repo-owned
  HAL/JNI artifacts.
- **Inputs:** `ROBOSIM_TIMED_ROBOT_TIMEOUT=2s scripts/run_timed_robot_smoke.sh`
  against the generated `tools/timed-robot-template` jar.
- **Expected outputs:** Startup completes and `Robot Periodic Run!` appears at
  least once before timeout.
- **Determinism notes:** Manual smoke because it depends on local JDK/template
  artifacts and process timeout behavior.
