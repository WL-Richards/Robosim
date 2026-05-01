# Cycle 61 test plan - host-side shim driver

Status: implemented.

Reviewer: requested direct boot-first coverage and stronger zero-count
active-prefix coverage; both are incorporated below.

## Context

Cycles 58-60 made the generated TimedRobot smoke path work by adding a
temporary JNI fallback host inside `libwpiHaljni.so`. That fallback is useful
for bootstrapping, but the real architecture needs the simulator host to own
the `core_to_backend` Tier 1 endpoint and publish protocol messages to an
installed shim.

Cycle 61 adds the smallest real host-facing surface: a production helper that
owns the host endpoint, accepts the shim boot descriptor, sends `boot_ack`, and
publishes clock/notifier messages over the existing protocol. It does not start
a separate process, background poll loop, or scheduler yet.

## Scope

In:

- Add a `shim_host_driver` helper in the shim layer for the host side of an
  installed shim.
- The helper creates a `core_to_backend` Tier 1 endpoint from a shared region.
- The helper accepts the shim's boot envelope and returns the observed boot
  descriptor.
- The helper sends `boot_ack` at an explicit simulation timestamp.
- The helper publishes `clock_state` as a fixed-size tick-boundary payload.
- The helper publishes `notifier_alarm_batch` with active-prefix bytes,
  including a convenience one-alarm publish method.
- Unit tests use this helper instead of raw `tier1_endpoint` sends for the new
  real-host path.

Out:

- No JNI fallback changes.
- No HALSIM usage.
- No shared-memory fd launch/env plumbing for a separate robot process.
- No background poll thread in the robot process.
- No real-time sleep/scheduler in the host helper.
- No Driver Station output-side message pulling or observer publication from
  robot-to-host; those schemas already exist from earlier cycles and remain
  shim-driven.
- No wall-clock assertions.

## v0 decisions

- **D-C61-REAL-HOST-ONLY:** The helper operates only on a caller-provided
  Tier 1 shared region and never creates or touches the JNI fallback state.
- **D-C61-EXPLICIT-POLL:** The helper publishes host messages only. The caller
  remains responsible for calling `shim_core::poll()` or otherwise driving the
  robot-side receive loop.
- **D-C61-EXPLICIT-TIME:** Every send takes an explicit `sim_time_us`. The
  helper does not infer time and does not sleep.
- **D-C61-BOOT-FIRST:** `accept_boot_and_send_ack()` must be called before
  tick-boundary publishes. Protocol-session ordering enforces this.
- **D-C61-ACTIVE-PREFIX-ALARMS:** Notifier alarms use
  `offsetof(notifier_alarm_batch, events) + count * sizeof(notifier_alarm_event)`
  bytes. A zero-count batch is valid and sends only the header prefix.
- **D-C61-ONE-LANE-BACKPRESSURE:** The helper does not retry or poll on
  `lane_busy`. It returns the transport failure so a later runtime scheduler can
  decide whether to retry, drop, or fail.
- **D-C61-NO-SHORTCUT-DELIVERY:** The helper does not mutate shim caches or wake
  waits directly; all behavior must pass through the protocol lane and
  `shim_core::poll()`.
- **D-C61-FALLBACK-UNCHANGED:** No-shim and JNI fallback behavior are unchanged.

## Tests

### C61-1 - host driver accepts boot and sends boot_ack

- **Layer / contract:** real host handshake over `core_to_backend`.
- **Bug class caught:** helper using the wrong direction, skipping boot drain,
  not returning the boot descriptor, or mutating shim connection state without a
  real protocol message.
- **Inputs:** Create a region, create a backend shim with a distinct boot
  descriptor, create a host driver on the same region, call
  `accept_boot_and_send_ack(123'000)`.
- **Expected outputs:** The returned descriptor equals the shim descriptor; the
  shim is still disconnected before `shim.poll()`; after `shim.poll()` it is
  connected.
- **Determinism notes:** In-process only; no sleeps.

### C61-2 - host driver publishes clock only after explicit shim poll

- **Layer / contract:** real host clock publication boundary.
- **Bug class caught:** direct cache mutation, fallback clock pump reuse, wrong
  schema, or missing fixed-size payload.
- **Inputs:** Complete C61 host handshake; install the shim globally; publish
  `clock_state{sim_time_us=42'000}` at `42'000`.
- **Expected outputs:** Before `shim.poll()`, `HAL_GetFPGATime` remains `0`
  because no clock has been accepted. After `shim.poll()`, `HAL_GetFPGATime`
  returns `42'000` with success.
- **Determinism notes:** No fallback and no wall clock.

### C61-3 - host driver publishes one notifier alarm through protocol

- **Layer / contract:** real host notifier alarm delivery.
- **Bug class caught:** direct wait wake, wrong notifier handle, missing active
  prefix, wrong schema, or publication before protocol acceptance.
- **Inputs:** Complete C61 host handshake; install shim; initialize a notifier;
  publish one alarm for that handle at `75'000`.
- **Expected outputs:** A nonblocking check before `shim.poll()` shows no cached
  alarm batch. After `shim.poll()`, the latest alarm batch contains one event
  and `HAL_WaitForNotifierAlarm` returns `75'000`.
- **Determinism notes:** Wait is called only after the event is queued.

### C61-4 - zero-count alarm batch is a real active-prefix message

- **Layer / contract:** empty/default host publication.
- **Bug class caught:** rejecting empty batches, sending `sizeof` with stale
  events, or treating empty as no-op.
- **Inputs:** Complete C61 host handshake and publish a deliberately poisoned
  zero-count notifier alarm batch: `count = 0`, but `events[0]` contains a
  nonzero handle/time that must be outside the active prefix.
- **Expected outputs:** After `shim.poll()`, `latest_notifier_alarm_batch()` is
  present and byte-equal to a zero-initialized empty batch.
- **Determinism notes:** In-process only.

### C61-5 - publishes before boot_ack are rejected by session ordering

- **Layer / contract:** boot-first host ordering.
- **Bug class caught:** lazy hidden handshake in first publish, sending
  tick-boundary before `boot_ack`, direct cache mutation, or fallback reuse.
- **Inputs:** Create region, create backend shim, create host driver, but do
  not call `accept_boot_and_send_ack()`. Attempt to publish clock state.
- **Expected outputs:** Publish fails with a transport/session rejection; shim
  remains disconnected; clock and alarm caches remain empty after a no-message
  poll attempt; no JNI fallback state exists.
- **Determinism notes:** In-process only.

### C61-6 - lane busy is surfaced without shortcut retry

- **Layer / contract:** host send backpressure.
- **Bug class caught:** hidden poll/retry behavior in the helper, swallowed
  transport errors, or overwriting a full lane.
- **Inputs:** Complete C61 host handshake; publish one clock message and do not
  poll the shim; immediately publish a second clock message.
- **Expected outputs:** The first publish succeeds; the second returns
  `tier1_transport_error_kind::lane_busy`; after one `shim.poll()`, the shim
  observes the first clock, not the second.
- **Determinism notes:** Single-message lane state only.

### C61-7 - host driver does not create or use JNI fallback state

- **Layer / contract:** fallback ownership boundary.
- **Bug class caught:** real helper accidentally reusing Cycle 58-60 fallback
  launch state.
- **Inputs:** Reset JNI fallback state, run host-driver handshake and publish a
  clock through an installed shim.
- **Expected outputs:** `hal_jni_launch_state_has_fallback_for_test()` remains
  false throughout; the installed shim accepts the clock after polling.
- **Determinism notes:** In-process only.
