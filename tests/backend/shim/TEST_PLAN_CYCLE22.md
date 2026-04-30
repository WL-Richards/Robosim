# HAL shim core — cycle 22 test plan (HAL_CAN_GetCANStatus)

**Status:** implemented. Round-1 verdict was `not-ready`: the reviewer
approved the overall surface but required the empty-cache test to pin
read purity and the latest-wins test to interleave read calls between
cache updates. Revision 2 resolved those findings and received
`ready-to-implement`.

**Implements:** the twenty-second TDD cycle of the HAL shim core. Cycle
22 wires the 2026 C HAL CAN status read surface:

```c
void HAL_CAN_GetCANStatus(float* percentBusUtilization,
                          uint32_t* busOffCount,
                          uint32_t* txFullCount,
                          uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount,
                          int32_t* status);
```

Official WPILib 2026.2.2 generated docs for `hal/CAN.h` define this
signature and parameter order. The protocol schema's `can_status`
struct already mirrors the same order:

```cpp
struct can_status {
  float percent_bus_utilization;
  std::uint32_t bus_off_count;
  std::uint32_t tx_full_count;
  std::uint32_t receive_error_count;
  std::uint32_t transmit_error_count;
};
```

---

## Why this cycle exists

Cycle 5 added the inbound `can_status` cache slot and explicitly left
the C HAL ABI surface out of scope. Cycles 20 and 21 completed CAN
send and CAN stream receive surfaces. Cycle 22 closes the simple CAN
status read seam by copying the latest cached `can_status` fields into
WPILib's out-parameters.

This is a struct-out-parameter read, unlike prior scalar readers. The
main bug class is field-order drift: `txFullCount`,
`receiveErrorCount`, and `transmitErrorCount` are all `uint32_t` and
easy to accidentally transpose.

---

## Contract under test

### New extern "C" surface

`src/backend/shim/hal_c.h`:

```cpp
void HAL_CAN_GetCANStatus(float* percentBusUtilization,
                          std::uint32_t* busOffCount,
                          std::uint32_t* txFullCount,
                          std::uint32_t* receiveErrorCount,
                          std::uint32_t* transmitErrorCount,
                          std::int32_t* status);
```

### Behavior

- No installed shim: write `*status = kHalHandleError`; write zero to
  every data out-parameter.
- Installed shim, empty `latest_can_status_`: write
  `*status = kHalSuccess`; write zero to every data out-parameter.
- Installed shim, populated cache: write `*status = kHalSuccess` and
  copy all five fields in WPILib parameter order:
  - `percentBusUtilization = latest.percent_bus_utilization`
  - `busOffCount = latest.bus_off_count`
  - `txFullCount = latest.tx_full_count`
  - `receiveErrorCount = latest.receive_error_count`
  - `transmitErrorCount = latest.transmit_error_count`
- Latest-wins is inherited from cycle 5: the newest accepted
  `can_status` cache value is what the C HAL reader returns.

### Out of scope

- CAN status computation. Sim Core owns the authoritative values and
  sends them via `can_status` tick-boundary payloads.
- CAN stream sessions, send, and receive-message surfaces. Cycles 20
  and 21 already cover send/stream; `HAL_CAN_ReceiveMessage` remains a
  future one-shot read surface.
- NULL out-pointers. Existing HAL_* read surfaces treat required
  out-pointers as UB matching WPILib's unconditional write-through C
  shape; this cycle inherits that rule for all six pointers.

---

## Decisions pinned

### New: D-C22-STRUCT-OUT-PARAM-ZERO-DEFAULT

No-shim and empty-cache paths write zero to all five data
out-parameters. This keeps stale caller storage from surviving a failed
or empty read and matches the existing scalar-reader convention that
empty cache returns the simulator's zero-start state with success.

### New: D-C22-CAN-STATUS-FIELD-ORDER

`HAL_CAN_GetCANStatus` copies from the schema to WPILib out-parameters
in exact official header order:
`percentBusUtilization`, `busOffCount`, `txFullCount`,
`receiveErrorCount`, `transmitErrorCount`. The schema was deliberately
laid out in that order in cycle 5.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: `status` is written on every call;
  NULL `status` is UB.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim writes
  `kHalHandleError`.
- D-C5 latest-wins cache semantics for `latest_can_status_`.

---

## Proposed tests

### C22-1. `HalCanGetCanStatus.WithNoShimInstalledSetsHandleErrorAndZerosOutputs`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR,
  D-C22-STRUCT-OUT-PARAM-ZERO-DEFAULT.
- **Setup:** `shim_core::install_global(nullptr)`; all five data
  out-parameters and status initialized to nonzero sentinels.
- **Action:** call `HAL_CAN_GetCANStatus(...)`.
- **Expected:** `status == kHalHandleError`; all five data outputs are
  zero.
- **Bug class:** no-shim path leaves stale caller values or forgets to
  write one output.

### C22-2. `HalCanGetCanStatus.WithShimInstalledButCacheEmptySetsSuccessAndZerosOutputs`

- **Layer / contract:** empty-cache variant of
  D-C22-STRUCT-OUT-PARAM-ZERO-DEFAULT.
- **Setup:** connected shim installed globally; no inbound
  `can_status` has been polled; data outputs and status initialized to
  nonzero sentinels. Assert `latest_can_status()` is `nullopt` before
  the call.
- **Action:** call `HAL_CAN_GetCANStatus(...)`.
- **Expected:** `status == kHalSuccess`; all five data outputs are
  zero; `latest_can_status()` remains `nullopt` after the call.
- **Bug class:** empty cache reports handle error, returns stale data,
  confuses `latest_can_status_` with another cache, or materializes a
  zero cache value as a read side effect.

### C22-3. `HalCanGetCanStatus.WithCachedCanStatusCopiesEveryFieldInParameterOrder`

- **Layer / contract:** D-C22-CAN-STATUS-FIELD-ORDER.
- **Setup:** connected shim installed globally; inject/poll
  `valid_can_status(0.625f, 11, 22, 33, 44)`.
- **Action:** call `HAL_CAN_GetCANStatus(...)`.
- **Expected:** `status == kHalSuccess`,
  `percentBusUtilization == 0.625f`, `busOffCount == 11`,
  `txFullCount == 22`, `receiveErrorCount == 33`,
  `transmitErrorCount == 44`.
- **Bug class:** transposed integer fields, wrong cache slot, or
  partial copy.

### C22-4. `HalCanGetCanStatus.LatestWinsAcrossTwoUpdatesReturnsMostRecentStatus`

- **Layer / contract:** inherited D-C5 latest-wins through C HAL seam.
- **Setup:** connected shim installed globally.
- **Action:** inject/poll first `can_status(0.125f, 1, 2, 3, 4)`;
  call `HAL_CAN_GetCANStatus(...)` with status sentinel `999`; then
  inject/poll second `can_status(0.875f, 9, 8, 7, 6)` and call
  `HAL_CAN_GetCANStatus(...)` again with status sentinel `888`.
- **Expected:** first call succeeds and returns the first status; second
  call succeeds and returns the second status.
- **Bug class:** reader returns first/old cached value or combines
  fields from multiple updates; implementation caches the first HAL
  read result instead of reading the current shim cache each call.

---

## Tests deliberately not added

- NULL out-pointer tests. This cycle inherits the existing C HAL
  required-pointer UB rule.
- Re-testing inbound `can_status` framing, pre-boot rejection,
  no-padding, and cross-cache independence. Cycle 5 already owns those
  contracts; this cycle tests the C ABI read seam.
- Float tolerance tests. The schema stores `percent_bus_utilization`
  as `float` and the HAL API also writes `float`; expected values are
  compared exactly after using representable fixture literals.
