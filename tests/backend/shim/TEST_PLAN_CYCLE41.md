# HAL shim core - cycle 41 test plan (HAL team number)

Status: implemented

Cycle 41 adds the HALBase team-number startup query:

```cpp
std::int32_t HAL_GetTeamNumber(void);
```

The value already exists in the protocol boot descriptor. Cycle 41's smallest
coherent slice is to retain the caller-supplied `boot_descriptor` in
`shim_core` and expose only `team_number` through the C HAL ABI. This is not a
protocol change.

## Decisions

- **D-C41-C-ABI-ONLY:** Cycle 41 adds only retained boot metadata and
  `HAL_GetTeamNumber`. No protocol schema, protocol version, boot envelope
  layout, or host publication change is part of this slice.
- **D-C41-BOOT-DESCRIPTOR-OWNER:** `shim_core::make` stores an exact copy of
  the `boot_descriptor` it successfully publishes. This avoids reparsing the
  outbound lane and gives HALBase metadata calls one source of truth.
- **D-C41-READONLY-ACCESSOR:** `shim_core::boot_descriptor_snapshot()` exposes
  the retained descriptor by const reference for host-facing and test reads.
  It does not mutate, poll, publish, or require connection.
- **D-C41-NO-SHIM-DEFAULT:** With no installed shim, `HAL_GetTeamNumber`
  returns `0`, matching WPILib's documented "not found" default for team
  number.
- **D-C41-INSTALLED-BOOT-VALUE:** With an installed shim, `HAL_GetTeamNumber`
  returns the retained `boot_descriptor.team_number`, including zero and
  negative/sentinel test values without clamping.
- **D-C41-SHUTDOWN-DETACH:** `HAL_Shutdown` detaches the process-global shim;
  after shutdown, `HAL_GetTeamNumber` follows the no-shim path and returns
  `0`. The caller-owned shim still retains its boot descriptor.
- **D-C41-NO-PUBLISH-NO-POLL:** `HAL_GetTeamNumber` is read-only metadata. It
  must not publish outbound messages, poll inbound messages, or mutate cached
  sim state.

## Proposed tests

### C41-1 - shim retains the exact boot descriptor after make
- Layer / contract: D-C41-BOOT-DESCRIPTOR-OWNER.
- Bug class caught: storing only `team_number`, default-initializing boot
  metadata, or retaining a mutated descriptor rather than the caller-provided
  one.
- Inputs: create a valid descriptor with non-default team number, vendor
  capabilities, reserved bytes, and WPILib version bytes; call
  `shim_core::make`.
- Expected: boot envelope payload remains byte-identical to the supplied
  descriptor, and `boot_descriptor_snapshot()` is byte-identical to the same
  descriptor.
- Tolerance / determinism: exact bytes.

### C41-2 - HAL_GetTeamNumber signature and no-shim default
- Layer / contract: C HAL ABI and D-C41-NO-SHIM-DEFAULT.
- Bug class caught: wrong return type, accidental status parameter, or requiring
  an installed shim for the documented default.
- Inputs: compile-time function pointer check; clear global shim; call
  `HAL_GetTeamNumber`.
- Expected: signature is `std::int32_t (*)()` and return value is `0`.
- Tolerance / determinism: exact value.

### C41-3 - installed shim returns boot descriptor team number before boot_ack
- Layer / contract: D-C41-INSTALLED-BOOT-VALUE before connection.
- Bug class caught: requiring boot_ack or inbound state before returning
  boot-time metadata.
- Inputs: create shim with team number 971, drain only boot envelope, install
  shim, call `HAL_GetTeamNumber`.
- Expected: returns 971 and publishes no outbound message.
- Tolerance / determinism: exact integer and lane state.

### C41-4 - team number preserves boundary values without clamping
- Layer / contract: D-C41-INSTALLED-BOOT-VALUE integer preservation.
- Bug class caught: unsigned narrowing, nonzero-only validation, or clamping
  negative/sentinel values.
- Inputs: run independent shims with team numbers `0`, `-1`,
  `std::numeric_limits<std::int32_t>::min()`, and
  `std::numeric_limits<std::int32_t>::max()`.
- Expected: each installed shim returns exactly the descriptor value supplied
  to `make`.
- Tolerance / determinism: exact integer values.

### C41-5 - installed team-number query does not publish or poll
- Layer / contract: D-C41-NO-PUBLISH-NO-POLL.
- Bug class caught: shortcut behavior where a read-only metadata query sends
  protocol messages or consumes pending inbound traffic.
- Inputs: create a shim, drain boot, queue a valid `clock_state` in the inbound
  core-to-backend lane, install the shim, call `HAL_GetTeamNumber`.
- Expected: returns the descriptor team number; backend-to-core lane remains
  empty; core-to-backend lane remains full with the clock message still
  pending; latest clock/power/DS caches remain unset.
- Tolerance / determinism: exact lane states and cache emptiness.

### C41-6 - team number follows global detach while shim retains metadata
- Layer / contract: D-C41-SHUTDOWN-DETACH.
- Bug class caught: global stale pointer after shutdown or clearing retained
  boot metadata on shutdown.
- Inputs: connected shim with non-default team number; install it; call
  `HAL_GetTeamNumber`; call `HAL_Shutdown`; call `HAL_GetTeamNumber` again.
- Expected: first call returns the descriptor team number; after shutdown the
  global call returns 0; the caller-owned shim's `boot_descriptor_snapshot()`
  still has the original team number.
- Tolerance / determinism: exact integer values.

## Deferred

- `HAL_GetFPGAVersion`, `HAL_GetFPGARevision`, `HAL_GetSerialNumber`, and
  `HAL_GetComments`.
- Any host protocol for changing boot metadata after shim construction.
- `HAL_ExpandFPGATime` and system clock tick expansion behavior.
