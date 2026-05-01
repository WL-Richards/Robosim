# Cycle 66 test plan - attached smoke runtime diagnostics

Status: implemented.

Reviewer round 1 was `not-ready`: the plan claimed runtime no-due phases are
counted, but no test drove an actual `no_due_alarm` step. C66-2 now includes a
real no-due phase after the alarm has fired.

## Context

Cycle 65 moved `scripts/run_timed_robot_smoke.sh` onto
`robosim_timed_robot_smoke_host`, which creates a host-owned Tier 1 mapping,
passes `ROBOSIM_TIER1_FD` to the Java child, and drives the Cycle 64 runtime.
Manual smoke reaches repeated `robotPeriodic()` through that path. The remaining
rough edge is observability: when the smoke runs, it should be obvious that the
attached host runtime path booted and pumped real protocol traffic, rather than
silently falling back to the no-env JNI timing path.

Cycle 66 adds a small diagnostic surface to the smoke host and a convenience
build/run script. It does not change HAL behavior or remove fallback helpers.

## Scope

In:

- Add a public smoke-host counters struct owned by
  `timed_robot_smoke_host_runtime`.
- Increment counters when the runtime accepts boot, receives robot output,
  publishes clocks, publishes notifier alarms, and observes no-due alarm phases.
- Add a pure formatter for one-line smoke-host status/summary text.
- Make the smoke host executable print concise attached-runtime diagnostics
  when the fd handoff is prepared, boot is accepted, and before normal child
  exit. The runtime counter/formatter is the tested contract; the process-level
  manual smoke remains outside CTest.
- Add a repo script that builds the shim artifacts, the smoke host, the timed
  robot template, then runs the existing smoke script.

Out:

- No fallback helper removal.
- No deterministic replay logging.
- No Java process CTest.
- No Driver Station input generation or mode-control changes.
- No signal-handler summary for timeout-killed manual smoke. The host still
  exits by the outer `timeout` during normal smoke.

## v0 decisions

- **D-C66-COUNTERS-NOT-LOGS:** Unit tests assert runtime counters and formatted
  strings, not process stderr output. Log text is a human diagnostic, while
  counters are the stable contract.
- **D-C66-BOOT-COUNTS-ONCE:** `boot_accept_count` increments exactly once when
  boot is accepted. Waiting-for-boot iterations do not increment it.
- **D-C66-PROTOCOL-ACTIVITY-COUNTS:** Runtime steps increment the counter for
  the actual protocol result returned by the Cycle 64 runtime:
  robot output, clock publication, notifier alarm publication, or no-due alarm.
- **D-C66-NO-FALLBACK-SIGNAL:** The diagnostic path does not inspect or depend
  on JNI fallback state. Existing attached tests continue to assert fallback
  absence when proving real protocol activity.
- **D-C66-BUILD-RUN-SCRIPT:** The convenience script may run external build
  tools and the manual smoke; it is verified statically in CTest rather than
  executed there.

## Tests

### C66-1 - waiting for boot does not increment smoke runtime counters

- **Layer / contract:** smoke runtime diagnostic counters.
- **Bug class caught:** false diagnostics claiming an attached boot or protocol
  activity before a robot process has published boot.
- **Inputs:** Create a host Tier 1 mapping and runtime with no robot endpoint.
  Call `step(20'000)`.
- **Expected outputs:** The step reports `waiting_for_boot`; all counters remain
  zero.
- **Determinism notes:** In-process shared memory only.

### C66-2 - smoke runtime counters track attached clock and notifier protocol

- **Layer / contract:** attached smoke runtime observability over real protocol.
- **Bug class caught:** diagnostics counting attempted work instead of
  successful protocol messages, or claiming host-runtime activity while relying
  on fallback timing/direct wake behavior.
- **Inputs:** Create an attached in-process shim and host smoke runtime. Accept
  boot, initialize a notifier through C HAL, update it to `20'000`, flush
  notifier state, run steps for robot output, clock, and notifier alarm while
  polling the shim between host messages.
- **Expected outputs:** Counters show `boot_accept_count == 1`,
  `robot_output_count == 1`, `clock_publish_count == 1`,
  `notifier_alarm_publish_count == 1`, and `no_due_count == 0`. The notifier
  wait returns `20'000`. A following step at the same timestamp reports
  `no_due_alarm`; counters then show `no_due_count == 1` without publishing a
  second alarm. JNI fallback launch state remains absent and fallback sleep/
  pacing logs remain empty.
- **Determinism notes:** Bounded yield loops on observable wait state; no sleeps
  or wall-clock assertions.

### C66-3 - smoke runtime status formatter includes attached protocol counts

- **Layer / contract:** human diagnostic text boundary.
- **Bug class caught:** diagnostic output omits the evidence needed to
  distinguish attached runtime from fallback/no-env launches.
- **Inputs:** Build a counters struct with boot `1`, robot outputs `2`, clocks
  `3`, alarms `4`, and no-due `5`; format it with child fd `42`.
- **Expected outputs:** The string includes `ROBOSIM_TIER1_FD=42`,
  `boot=1`, `robot_outputs=2`, `clocks=3`, `alarms=4`, and `no_due=5`.
- **Determinism notes:** Pure string formatting.

### C66-4 - build-and-run smoke script builds all required artifacts then delegates

- **Layer / contract:** repo-owned manual smoke workflow.
- **Bug class caught:** user rebuilds only Java or only shim artifacts and then
  runs stale code; helper script bypasses the host runtime smoke script.
- **Inputs:** Inspect the new script.
- **Expected outputs:** The script builds `robosim_wpi_hal`,
  `robosim_wpi_hal_jni`, `robosim_timed_robot_smoke_host`, builds
  `tools/timed-robot-template`, then invokes `scripts/run_timed_robot_smoke.sh`.
- **Determinism notes:** Static CMake/script check; no Gradle/Java launch in
  CTest.

## Manual verification

- Run:

  ```bash
  ROBOSIM_TIMED_ROBOT_TIMEOUT=2s scripts/build_and_run_timed_robot_smoke.sh
  ```

- Expected: the smoke host prints attached-runtime diagnostic lines and the
  empty TimedRobot reaches repeated `robotPeriodic()` before timeout.
