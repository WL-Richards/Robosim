# HAL shim core - cycle 42 test plan (HAL FPGA version/revision)

Status: implemented

Cycle 42 adds the paired HALBase FPGA metadata queries:

```cpp
std::int32_t HAL_GetFPGAVersion(std::int32_t* status);
std::int64_t HAL_GetFPGARevision(std::int32_t* status);
```

These are startup metadata reads. They do not consume sim-time state and do
not add protocol schemas. The slice intentionally lands both functions in one
cycle because WPILib documents them together as HALBase FPGA metadata and they
share identical status/default/no-publish behavior.

## Decisions

- **D-C42-C-ABI-ONLY:** Cycle 42 adds only the two HALBase C ABI functions.
  No protocol schema, protocol version, boot descriptor layout, or host
  publication change is part of this slice.
- **D-C42-V0-CONSTANTS:** v0 reports deterministic RoboRIO-2 shim constants:
  `HAL_GetFPGAVersion` returns `2026`, matching WPILib's documented
  "competition year" convention for FPGA version; `HAL_GetFPGARevision`
  returns `0` as an explicit unknown/unmodeled FPGA image revision until a
  hardware-profile metadata source exists.
- **D-C42-STATUS:** Both functions write `kHalSuccess` with an installed shim
  and write `kHalHandleError` with no installed shim.
- **D-C42-EMPTY-CACHE:** These calls are not backed by inbound `clock_state` or
  any other per-tick cache. An installed shim with no accepted inbound state
  still returns the v0 constants with success.
- **D-C42-SHUTDOWN-DETACH:** `HAL_Shutdown` detaches the process-global shim;
  after shutdown, both functions follow the no-shim path and return `0` with
  `kHalHandleError`.
- **D-C42-NO-PUBLISH-NO-POLL:** These calls are read-only startup metadata.
  They must not publish outbound messages, poll inbound messages, or mutate
  cached sim state.

## Proposed tests

### C42-1 - FPGA version/revision signatures
- Layer / contract: C HAL ABI.
- Bug class caught: wrong return width, missing status parameter, or swapped
  version/revision signatures.
- Inputs: compile-time function pointer checks.
- Expected: `HAL_GetFPGAVersion` has type `std::int32_t (*)(std::int32_t*)`;
  `HAL_GetFPGARevision` has type `std::int64_t (*)(std::int32_t*)`.
- Tolerance / determinism: compile-time exact.

### C42-2 - no-shim paths write handle error and return zero
- Layer / contract: D-C42-STATUS no-shim behavior.
- Bug class caught: treating static metadata as globally available despite the
  status channel, or failing to overwrite caller status.
- Inputs: clear global shim; call both functions with sentinel statuses.
- Expected: both return 0 and overwrite status with `kHalHandleError`.
- Tolerance / determinism: exact values.

### C42-3 - installed empty-cache shim returns v0 constants with success
- Layer / contract: D-C42-V0-CONSTANTS and D-C42-EMPTY-CACHE.
- Bug class caught: requiring boot_ack or inbound clock/power state before
  returning startup metadata.
- Inputs: create a shim, drain only boot, install it, call both functions.
- Expected: version returns `2026`, revision returns `0`, and both statuses are
  `kHalSuccess`; no outbound message is published.
- Tolerance / determinism: exact values and lane state.

### C42-4 - calls overwrite status on repeated mixed paths
- Layer / contract: status overwrite discipline.
- Bug class caught: writing status only on success or preserving stale status
  across no-shim/installed transitions.
- Inputs: call both functions with no shim and sentinel statuses, then install a
  shim and call again with different sentinels.
- Expected: no-shim calls write `kHalHandleError`; installed calls write
  `kHalSuccess`.
- Tolerance / determinism: exact statuses.

### C42-5 - installed queries do not publish or poll
- Layer / contract: D-C42-NO-PUBLISH-NO-POLL.
- Bug class caught: shortcut behavior where metadata queries send protocol
  messages or consume pending inbound traffic.
- Inputs: create a shim, drain boot, queue a valid `clock_state` in the inbound
  core-to-backend lane, install the shim, call both functions.
- Expected: version/revision return v0 constants with success;
  backend-to-core lane remains empty; core-to-backend lane remains full; latest
  clock/power/DS caches remain unset.
- Tolerance / determinism: exact values, lane states, and cache emptiness.

### C42-6 - post-shutdown follows detached no-shim path
- Layer / contract: D-C42-SHUTDOWN-DETACH.
- Bug class caught: stale global pointer after shutdown or treating these
  status-writing calls like process-global runtime metadata.
- Inputs: connected shim; install it; call both functions; call
  `HAL_Shutdown`; call both again.
- Expected: installed calls return constants with `kHalSuccess`; after shutdown
  the global shim is null and both calls return 0 with `kHalHandleError`.
- Tolerance / determinism: exact values and statuses.

## Deferred

- Configurable hardware-profile FPGA version/revision metadata.
- `HAL_GetSerialNumber` and `HAL_GetComments`.
- `HAL_GetLastError` / `HAL_GetErrorMessage`.
