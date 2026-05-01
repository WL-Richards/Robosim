# Cycle 62 test plan - JNI attach to host-provided Tier 1 mapping

Status: implemented.

Reviewer: requested parseable-but-unmappable fd coverage and scoped env cleanup
notes; both are incorporated below.

## Context

Cycle 61 added `shim_host_driver`, the real host-side helper for an installed
shim. The remaining gap before a real Java process can use it is handoff:
`libwpiHaljni.so` must be able to attach its robot-side shim to a Tier 1 shared
memory mapping created by the host instead of always creating the Cycle-51
fallback mapping.

Cycle 62 adds the smallest handoff boundary: an environment-variable file
descriptor attach path in JNI `HAL.initialize`. It installs a normal
`backend_to_core` shim over the mapped region, publishes the boot descriptor to
the host, and leaves all host driving and robot-side polling explicit for later
cycles.

## Scope

In:

- Add a documented env var, `ROBOSIM_TIER1_FD`, read by
  `Java_edu_wpi_first_hal_HAL_initialize`.
- When the env var is present and parses as a non-negative integer fd, map that
  fd with `tier1_shared_mapping::map_existing`, create the backend endpoint on
  the mapped region, create/install `shim_core`, and call C `HAL_Initialize`.
- Preserve Cycle-51 fallback behavior when the env var is absent.
- Treat a present-but-invalid env var as an attach failure: return false from
  JNI initialize and do not silently create fallback state.
- Keep `shim_host_driver` as the host-side peer used by tests.

Out:

- No background poll loop.
- No host scheduler or real-time driver loop.
- No script-level process launcher changes yet.
- No fd-passing subprocess integration test yet.
- No changes to fallback notifier/clock pumps.
- No changes to C `HAL_Initialize` semantics for already-installed shims.

## v0 decisions

- **D-C62-ENV-FD:** `ROBOSIM_TIER1_FD` is the v0 robot-process handoff
  mechanism. It contains a decimal file descriptor number inherited by the
  robot process.
- **D-C62-PRESENT-MEANS-REAL:** If `ROBOSIM_TIER1_FD` is present, JNI startup
  attempts the real attach path. Any parse, fd, size, endpoint, or shim-creation
  failure returns false and does not fall back.
- **D-C62-ABSENT-FALLBACK:** If `ROBOSIM_TIER1_FD` is absent, the existing
  Cycle-51 fallback path remains unchanged.
- **D-C62-NO-HOST-ENDPOINT-IN-JNI:** The attached robot process owns only the
  backend/shim endpoint. It does not create a `core_to_backend` endpoint; the
  host owns that through `shim_host_driver`.
- **D-C62-EXPLICIT-POLL:** The attach path does not add a robot-side polling
  thread. Host-published state is observed only after explicit `shim.poll()` in
  tests; a later runtime cycle will add polling.
- **D-C62-NO-FALLBACK-PUMPS:** JNI fallback clock/notifier pumps remain
  fallback-owned only. An env-fd attached shim must not be paced or driven by
  those helpers.
- **D-C62-FD-LIFETIME:** The attached launch state owns the mapped peer
  `tier1_shared_mapping`. `map_existing` duplicates the supplied fd, so the
  shim remains valid even if the caller closes its original duplicate after
  initialization.
- **D-C62-SCOPED-ENV-TESTS:** Tests that set `ROBOSIM_TIER1_FD` must restore or
  unset it with an RAII guard so failures cannot poison later fallback tests.

## Tests

### C62-1 - JNI initialize attaches to host mapping and publishes boot

- **Layer / contract:** process handoff, robot-side endpoint creation.
- **Bug class caught:** ignoring the env fd, creating fallback instead, using
  the wrong endpoint direction, failing to publish boot, or not installing the
  shim globally.
- **Inputs:** Host creates `tier1_shared_mapping`, duplicates its fd, sets
  `ROBOSIM_TIER1_FD` to that fd number, calls JNI `HAL.initialize`, then creates
  `shim_host_driver` on the host mapping and accepts boot/sends ack.
- **Expected outputs:** JNI initialize returns true; `shim_core::current()` is
  non-null; fallback-state observer reports false; host observes the boot
  descriptor through `shim_host_driver`; the shim becomes connected only after
  explicit `shim.poll()`.
- **Determinism notes:** In-process fd handoff only; no sleeps.

### C62-2 - host-published clock reaches attached JNI shim through explicit poll

- **Layer / contract:** attached mapping is the real shared region.
- **Bug class caught:** mapping a private region, direct cache mutation, or
  fallback clock pump reuse.
- **Inputs:** Complete C62 attach and host boot_ack. Host publishes
  `clock_state{sim_time_us=64'000}` through `shim_host_driver`.
- **Expected outputs:** Before `shim.poll()`, `HALUtil.getFPGATime()` returns 0
  and no fallback state exists. After `shim.poll()`, `HALUtil.getFPGATime()`
  returns `64'000`.
- **Determinism notes:** No wall clock and no fallback pump.

### C62-3 - attached shim remains valid after caller closes original fd

- **Layer / contract:** fd lifetime boundary.
- **Bug class caught:** storing a borrowed fd/mapping that dies when the caller
  closes its duplicate.
- **Inputs:** Host creates mapping and duplicate, sets env, calls JNI
  initialize, then lets the duplicate fd object destruct/close before host
  accepts boot and publishes a clock.
- **Expected outputs:** Host still accepts boot, sends ack, publishes clock,
  and the attached shim observes it after poll.
- **Determinism notes:** In-process fd close only.

### C62-4 - invalid present env fd fails closed without fallback

- **Layer / contract:** attach failure behavior.
- **Bug class caught:** silently falling back when a real host handoff was
  requested, accepting malformed fd strings, skipping mapping-size validation,
  or leaving partial launch state installed.
- **Inputs:** Set `ROBOSIM_TIER1_FD` to a malformed string and call JNI
  initialize. Repeat with `-1`. Repeat with a parseable valid fd number backed
  by a memfd of the wrong size.
- **Expected outputs:** Each call returns false; no shim is installed; fallback
  state is absent; no launch mapping remains visible through test helpers.
- **Determinism notes:** No filesystem or wall clock.

### C62-5 - absent env preserves existing fallback initialize behavior

- **Layer / contract:** fallback compatibility.
- **Bug class caught:** breaking Cycle-51 startup while adding the real attach
  path.
- **Inputs:** Unset `ROBOSIM_TIER1_FD`, call JNI initialize.
- **Expected outputs:** JNI initialize returns true; fallback state exists; the
  fallback shim is installed and initially disconnected just as before.
- **Determinism notes:** Mirrors existing fallback tests but pins the env split.

### C62-6 - fallback pumps do not drive env-fd attached shims

- **Layer / contract:** fallback ownership boundary.
- **Bug class caught:** treating any JNI-owned shim as fallback-owned, sleeping,
  or publishing fallback clock/alarm into a real host-attached shim.
- **Inputs:** Attach through env fd, initialize a notifier, update an active
  alarm, call `pump_jni_fallback_notifier_alarm(handle)` and
  `HALUtil.getFPGATime()`.
- **Expected outputs:** The fallback pump returns false; no fallback pacing time
  is recorded; `HALUtil.getFPGATime()` returns the ordinary installed-shim empty
  clock value `0`; no fallback state exists.
- **Determinism notes:** No sleeps; uses existing test sleep hook observer.
