# Cycle 65 test plan - timed robot smoke host runtime launcher

Status: implemented.

Reviewer round 1 was `not-ready`: the integration test was only clock-level
and would not catch a launcher that still failed to wake TimedRobot's notifier
wait, and the documented child-exit behavior was untested. The revised suite
adds notifier-state/due-alarm coverage and a pure wait-status translation test.

## Context

Cycle 64 added a deterministic `shim_host_runtime_driver` that can run the
host side of the attached Tier 1 protocol: accept robot boot, receive
robot-published `notifier_state`, publish default clock ticks, and publish due
notifier alarms. The manual timed-robot smoke script still launches Java
directly with no `ROBOSIM_TIER1_FD`, so Java uses the Cycle 58-60 JNI fallback
clock/notifier pumps.

Cycle 65 moves the repo-owned timed-robot smoke script onto a real host-owned
Tier 1 mapping. The script should delegate to a small host runtime launcher
that creates the mapping, passes an inheritable fd to the Java child as
`ROBOSIM_TIER1_FD`, and runs the Cycle 64 runtime pump while the child is
alive. This cycle does not remove the fallback code; it removes the smoke
script's need for it.

## Scope

In:

- Add a small host smoke launcher executable for Linux/POSIX.
- The launcher creates a Tier 1 shared mapping in the parent process.
- The launcher duplicates the mapping fd for the child and explicitly clears
  `FD_CLOEXEC` on only that child handoff fd.
- The launcher sets `ROBOSIM_TIER1_FD=<child-fd>` in the child environment and
  then execs the requested robot command.
- The parent closes its copy of the child handoff fd after fork and uses the
  retained host mapping to create `shim_host_runtime_driver`.
- The parent waits for boot, sends `boot_ack`, then repeatedly:
  - consumes robot output messages when present;
  - advances simulated time in deterministic 20 ms steps;
  - calls `pump_to` enough times per timestamp to publish both clock and any
    due notifier alarm.
- `scripts/run_timed_robot_smoke.sh` launches Java through this host launcher.

Out:

- No fallback helper removal.
- No Driver Station input generation.
- No general process supervisor API beyond the smoke executable.
- No Windows support.
- No attempt to make wall-clock pacing deterministic; this remains a manual
  smoke launcher, not a deterministic replay test.

## v0 decisions

- **D-C65-SMOKE-ONLY:** The launcher is an executable used by the smoke script,
  not the final simulator process model.
- **D-C65-CHILD-FD-ONLY-INHERITS:** Only the duplicate fd named in
  `ROBOSIM_TIER1_FD` is made inheritable. Parent-owned mapping fds keep their
  normal close-on-exec behavior.
- **D-C65-BOOT-BEFORE-PUMP:** The parent does not publish clock or notifier
  messages before it receives the child's boot descriptor and sends `boot_ack`.
- **D-C65-ONE-TICK-REALTIME-PACE:** The manual launcher advances simulation in
  fixed 20 ms steps paced by `steady_clock`. This mirrors the timed-robot smoke
  period and avoids the old busy runaway behavior; it is not part of the
  deterministic backend contract.
- **D-C65-NO-FALLBACK-ENV:** The script must not intentionally omit
  `ROBOSIM_TIER1_FD`; fallback remains available only for no-env development
  launches outside this host runtime script.
- **D-C65-CHILD-EXIT-PASSTHROUGH:** The launcher exits with the child process
  status where possible. If the child is killed by a signal, the launcher
  returns `128 + signal`.
- **D-C65-PARENT-FD-CLEANUP:** Closing the parent copy of the child handoff fd
  after fork is implementation hygiene for the full process launcher and is
  covered by ownership review/manual smoke rather than a unit test in this
  cycle; the unit contract pins the observable exec-inheritance flags.

## Tests

### C65-1 - child handoff fd is the only fd made exec-inheritable

- **Layer / contract:** Tier 1 launch handoff boundary.
- **Bug class caught:** passing a `CLOEXEC` fd that Java cannot map, or clearing
  close-on-exec on the parent-owned mapping fd and leaking more descriptors
  than intended.
- **Inputs:** Create a `tier1_shared_mapping`, duplicate its fd for child
  handoff, call the launcher helper that prepares the child fd for exec.
- **Expected outputs:** The original mapping fd still has `FD_CLOEXEC`; the
  child handoff fd does not. The helper reports the same raw fd number that
  should be placed in `ROBOSIM_TIER1_FD`.
- **Determinism notes:** Same-process fd flag checks only; no fork or sleep.

### C65-2 - child environment contains the selected Tier 1 fd

- **Layer / contract:** smoke launcher child environment construction.
- **Bug class caught:** forgetting to set `ROBOSIM_TIER1_FD`, setting a stale fd
  value, or mutating unrelated environment entries.
- **Inputs:** Build a small environment vector containing `PATH=/bin` and an
  old `ROBOSIM_TIER1_FD=stale`, then ask the launcher helper to produce the
  child environment for fd `42`.
- **Expected outputs:** The result contains exactly one `ROBOSIM_TIER1_FD=42`,
  preserves `PATH=/bin`, and removes/replaces the stale entry.
- **Determinism notes:** Pure string-vector test.

### C65-3 - host smoke pump waits for boot before publishing clocks

- **Layer / contract:** smoke runtime loop over the Cycle 64 host runtime.
- **Bug class caught:** parent publishing clock/notifier traffic before the Java
  child's shim boot handshake, which would either fail session validation or
  accidentally reintroduce a fallback path.
- **Inputs:** Create a host mapping and runtime loop object with no robot boot
  message in the shared region. Run one nonblocking loop iteration at simulated
  time `20'000`.
- **Expected outputs:** The loop reports that it is waiting for boot and the
  host-to-robot lane remains empty.
- **Determinism notes:** In-process shared memory only; no child process.

### C65-4 - host smoke pump advances an attached shim clock through protocol

- **Layer / contract:** smoke runtime integration with attached Tier 1 shim.
- **Bug class caught:** accepting boot but failing to make the attached robot
  observe host-published clock traffic, or relying on fallback timing instead
  of protocol messages.
- **Inputs:** Create a shared mapping, map a duplicate fd as the robot side,
  create a `shim_core` over the robot endpoint, and create the smoke runtime
  loop over the host mapping. Run loop iterations until boot is accepted, the
  shim polls `boot_ack`, then run tick `20'000` and poll the shim.
- **Expected outputs:** The runtime reports boot accepted, publishes the
  `20'000` clock through protocol, and `HAL_GetFPGATime` returns `20'000` only
  after the shim polls the host message. JNI fallback launch state remains
  absent.
- **Determinism notes:** In-process shared memory only; no Java process or
  sleeps.

### C65-5 - host smoke pump wakes an attached notifier through protocol

- **Layer / contract:** smoke runtime integration for the TimedRobot periodic
  notifier path.
- **Bug class caught:** a launcher that publishes clocks but never consumes
  robot `notifier_state`, never publishes due `notifier_alarm_batch`, directly
  wakes waits, or falls back to JNI-owned timing.
- **Inputs:** Create an attached in-process shim and host smoke runtime as in
  C65-4. Complete boot. Install the shim globally, initialize a notifier
  through the C HAL surface, update its alarm to `20'000`, flush notifier state
  from the robot shim, and let the smoke runtime consume robot outputs. Start
  an async `HAL_WaitForNotifierAlarm` on that handle. Run the smoke runtime at
  `20'000` enough iterations to publish the clock and due alarm, polling the
  robot shim between host messages.
- **Expected outputs:** The runtime receives notifier state through protocol,
  publishes the `20'000` clock first, then publishes one due notifier alarm.
  The wait returns `20'000`. JNI fallback launch state remains absent and the
  fallback sleep/pacing log remains empty.
- **Determinism notes:** Bounded yield loops on observable state; no sleeps or
  wall-clock assertions.

### C65-6 - child wait status maps to launcher exit code

- **Layer / contract:** smoke launcher process-status boundary.
- **Bug class caught:** always returning success, collapsing child failures, or
  mishandling signal exits.
- **Inputs:** Feed the launcher helper synthetic wait statuses for normal exit
  code `17` and signal `SIGTERM`.
- **Expected outputs:** Normal exit maps to `17`; signal termination maps to
  `128 + SIGTERM`.
- **Determinism notes:** Pure status-code helper test; no subprocess.

### C65-7 - smoke script delegates Java launch to the host runtime executable

- **Layer / contract:** repo-owned timed-robot smoke command.
- **Bug class caught:** leaving the script on the old direct Java launch that
  omits `ROBOSIM_TIER1_FD` and therefore still depends on JNI fallback timing.
- **Inputs:** Inspect `scripts/run_timed_robot_smoke.sh`.
- **Expected outputs:** The script `exec`s the built host runtime launcher and
  passes the Java command after `--`; it does not directly `exec timeout ...
  java`.
- **Determinism notes:** Static script check via CTest/CMake; no Java launch.

## Manual verification

- Build `robosim_timed_robot_smoke_host`, `robosim_wpi_hal`, and
  `robosim_wpi_hal_jni`.
- Run:

  ```bash
  ROBOSIM_TIMED_ROBOT_TIMEOUT=2s scripts/run_timed_robot_smoke.sh
  ```

- Expected: the empty TimedRobot reaches `robotPeriodic()` through the attached
  host runtime path. The process still exits by the outer timeout.
