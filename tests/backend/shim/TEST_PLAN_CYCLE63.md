# Cycle 63 test plan - JNI env-attached shim poll loop

Status: implemented.

Reviewer: requested explicit synchronization scope, failed-attach/no-thread
coverage, and direct idle-hook coverage; all are incorporated below.

## Context

Cycle 62 lets `libwpiHaljni.so` install a robot-side shim over a
host-provided Tier 1 mapping via `ROBOSIM_TIER1_FD`. However, host-published
messages still require an explicit in-process `shim_core::poll()` call. A real
Java robot process blocked in `NotifierJNI.waitForNotifierAlarm()` cannot make
that call from the host.

Cycle 63 adds the smallest robot-side receive loop for env-attached JNI shims.
It polls the installed shim in a background thread so host-published
`boot_ack`, `clock_state`, and `notifier_alarm_batch` messages are accepted by
the robot process without fallback pumps or direct cache mutation.

## Scope

In:

- Start a background poll loop after successful `ROBOSIM_TIER1_FD` attach.
- Poll only env-fd attached shims; fallback shims remain driven by the existing
  fallback pumps.
- Stop and join the poll loop during JNI `HAL.shutdown` and test reset.
- Add test-only observers for poll-loop active state, poll count, and an
  optional idle hook so tests can observe progress without wall-clock sleeps.
- Synchronize poll-loop mutation against HAL ABI cache reads touched by the
  loop, starting with clock and notifier-alarm delivery used by TimedRobot.
- Keep host publication through `shim_host_driver`.

Out:

- No host scheduler.
- No real-time pacing in the real host path.
- No smoke-script process launcher changes yet.
- No DS packet generation.
- No changes to fallback clock/notifier pumps.
- No retry policy beyond repeatedly polling the robot-side receive lane.

## v0 decisions

- **D-C63-ATTACHED-ONLY:** The poll loop starts only for env-fd attached JNI
  shims. Absent-env fallback behavior remains disconnected after initialize
  until fallback pumps run.
- **D-C63-POLL-THREAD-OWNS-POLL:** For attached shims, the background thread is
  the robot-side receiver. Tests and host code should not need explicit
  `shim.poll()` for inbound host traffic.
- **D-C63-IDLE-SLEEP:** Production waits briefly after empty/no-progress polls
  to avoid a hot spin. Tests can replace that idle hook with an in-process
  observer/no-op; no test asserts wall-clock duration.
- **D-C63-ERROR-TOLERANCE:** The loop keeps running after ordinary no-message
  polls and stops when the launch state is shutting down. Unexpected poll
  errors are counted as poll attempts but do not crash the process in v0.
- **D-C63-SHUTDOWN-JOIN:** JNI shutdown and test reset request stop and join
  the thread before destroying the mapped shim/mapping.
- **D-C63-NO-FALLBACK-PUMPS:** Env-attached shims remain ineligible for
  fallback clock/notifier pumps and fallback pacing.
- **D-C63-SYNCHRONIZED-RECEIVE:** This cycle makes the new concurrent path
  synchronized rather than relying on the old v0 single-thread assumption.
  `shim_core::poll()` and HAL-reader access to the cache state exercised by the
  poll loop are protected so C63 tests are intended to pass under the existing
  sanitizer gate, including TSan. At minimum this includes global launch-state
  access, `clock_state` reads used by `HAL_GetFPGATime`/`HALUtil.getFPGATime`,
  and notifier-alarm queue/wait delivery.

## Tests

All tests that set `ROBOSIM_TIER1_FD` use the Cycle-62 scoped env guard.
Tests wait for background effects with bounded yield loops against observable
state, not sleeps or timing thresholds.

### C63-1 - env-attached initialize starts poll loop and accepts boot_ack

- **Layer / contract:** JNI attached receive loop startup.
- **Bug class caught:** not starting the poll thread, starting it on the wrong
  shim, failing to accept `boot_ack`, or requiring explicit test-side
  `shim.poll()`.
- **Inputs:** Create host mapping, set `ROBOSIM_TIER1_FD`, call JNI initialize,
  create `shim_host_driver`, accept boot and send `boot_ack`.
- **Expected outputs:** Poll-loop active observer becomes true; without calling
  `shim.poll()`, the attached shim eventually reports `is_connected() == true`;
  poll count advances.
- **Determinism notes:** Bounded yield loop on `is_connected()` and poll count.

### C63-2 - host-published clock reaches HALUtil through poll loop

- **Layer / contract:** background receive of fixed-size clock payload.
- **Bug class caught:** direct cache mutation, fallback clock pump reuse, or
  poll loop not processing tick-boundary messages.
- **Inputs:** Complete C63 attached boot. Host publishes
  `clock_state{sim_time_us=96'000}` through `shim_host_driver`.
- **Expected outputs:** Without explicit `shim.poll()`, `HALUtil.getFPGATime()`
  eventually returns `96'000`; fallback state remains absent.
- **Determinism notes:** Bounded yield loop on HAL read value.

### C63-3 - host-published notifier alarm wakes Java wait through poll loop

- **Layer / contract:** background receive plus notifier wait wake.
- **Bug class caught:** polling but not dispatching alarm batches, direct wait
  wake shortcut, or fallback notifier pump reuse.
- **Inputs:** Attached initialize, host boot_ack, initialize notifier, update an
  active alarm, start async `NotifierJNI.waitForNotifierAlarm`, wait until the
  shim records one waiter for that handle, then host publishes the matching
  notifier alarm through `shim_host_driver`.
- **Expected outputs:** The future returns the host-published timestamp and
  fallback state/pacing remain absent. The test never calls `shim.poll()`.
- **Determinism notes:** Existing future timeout helper only guards test hangs;
  readiness uses pending-waiter observer.

### C63-4 - fallback initialize does not start poll loop

- **Layer / contract:** attached-only boundary.
- **Bug class caught:** starting the new background loop for fallback shims,
  changing Cycle-51 disconnected fallback semantics, or double-driving fallback
  traffic.
- **Inputs:** Unset `ROBOSIM_TIER1_FD`, call JNI initialize.
- **Expected outputs:** Fallback state exists; attached poll-loop active
  observer is false; fallback shim remains disconnected until fallback pumps run.
- **Determinism notes:** No host traffic.

### C63-5 - failed env attach does not start poll loop

- **Layer / contract:** attach failure lifecycle.
- **Bug class caught:** starting the background thread before attach has fully
  succeeded or leaking a partially initialized launch state.
- **Inputs:** Set `ROBOSIM_TIER1_FD` to a valid decimal fd backed by a
  wrong-size memfd and call JNI initialize.
- **Expected outputs:** JNI initialize returns false; poll-loop active observer
  is false; poll count remains zero; no shim, fallback state, or attached
  mapping remains installed.
- **Determinism notes:** In-process only.

### C63-6 - idle hook is used while attached poll loop has no traffic

- **Layer / contract:** no hot-spin idle behavior.
- **Bug class caught:** tight-spinning poll loop, unused test hook, or idle hook
  mutating cache state.
- **Inputs:** Install a test idle hook counter, attach through env fd, complete
  boot_ack, then send no further host messages.
- **Expected outputs:** Idle hook counter advances while latest clock and alarm
  caches stay empty; poll loop remains active.
- **Determinism notes:** Bounded yield loop on idle counter.

### C63-7 - JNI shutdown stops poll loop before destroying attached state

- **Layer / contract:** poll-thread lifecycle cleanup.
- **Bug class caught:** use-after-free on mapped shim/mapping, thread leak, or
  stale global state after shutdown.
- **Inputs:** Attached initialize, complete boot_ack through host, confirm poll
  loop active, call JNI shutdown.
- **Expected outputs:** Poll-loop active observer becomes false; launch state is
  cleared; global shim is null; a later host publish to the old mapping does not
  mutate any installed shim because none is installed.
- **Determinism notes:** Shutdown joins synchronously.

### C63-8 - test reset stops poll loop and restores idle hook

- **Layer / contract:** test lifecycle cleanup.
- **Bug class caught:** poll-loop or test hook leaking across tests.
- **Inputs:** Install a test idle hook, attached initialize, confirm the hook is
  non-default and poll loop active, then call `reset_hal_jni_launch_state_for_test`.
- **Expected outputs:** Poll loop inactive, launch state cleared, idle hook is
  default for the next launch, and no shim remains installed.
- **Determinism notes:** In-process only.
