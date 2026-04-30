# HAL shim core — cycle 23 test plan (Driver Station scalar reads)

**Status:** implemented. Round-1 verdict was `not-ready`: the reviewer
approved the surface and most tests but required the cached
`HAL_GetControlWord` test to use a non-symmetric bit pattern so
named-bit transpositions cannot pass. Round 2 remained `not-ready`
because a single non-symmetric pattern still left some fields
indistinguishable. Revision 3 replaced that with one-hot coverage for
all six named bits and received `ready-to-implement`.

**Implements:** the twenty-third TDD cycle of the HAL shim core. Cycle
23 wires the three simple Driver Station scalar C HAL read surfaces:

```c
int32_t HAL_GetControlWord(HAL_ControlWord* controlWord);
HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status);
double HAL_GetMatchTime(int32_t* status);
```

Official WPILib 2026.2.2 generated docs for `hal/DriverStation.h`
define these signatures. `docs/HAL_SHIM_NEXT_SURFACES_REPORT.md`
recommends this exact surface as the next cycle after `HAL_CAN_GetCANStatus`.

---

## Why this cycle exists

Cycle 3 added the inbound `ds_state` cache slot and deliberately left
C HAL Driver Station read APIs out of scope. Cycle 23 exposes the
smallest useful read-only slice of that cache:

- `ds_state::control.bits` through WPILib's `HAL_ControlWord` out-param.
- `ds_state::station` through `HAL_GetAllianceStation`.
- `ds_state::match_time_seconds` through `HAL_GetMatchTime`.

The main bug classes are wrong-cache reads, stale caller out-params on
empty/no-shim paths, bit-order drift for `HAL_ControlWord`, enum value
drift for `HAL_AllianceStationID`, and returning a previously-read DS
value instead of the current cache.

---

## Contract under test

### New extern "C" surface

`src/backend/shim/hal_c.h`:

```cpp
struct HAL_ControlWord {
  std::uint32_t enabled : 1;
  std::uint32_t autonomous : 1;
  std::uint32_t test : 1;
  std::uint32_t eStop : 1;
  std::uint32_t fmsAttached : 1;
  std::uint32_t dsAttached : 1;
  std::uint32_t control_reserved : 26;
};

enum HAL_AllianceStationID : std::int32_t {
  HAL_AllianceStationID_kUnknown = 0,
  HAL_AllianceStationID_kRed1    = 1,
  HAL_AllianceStationID_kRed2    = 2,
  HAL_AllianceStationID_kRed3    = 3,
  HAL_AllianceStationID_kBlue1   = 4,
  HAL_AllianceStationID_kBlue2   = 5,
  HAL_AllianceStationID_kBlue3   = 6,
};

std::int32_t HAL_GetControlWord(HAL_ControlWord* controlWord);
HAL_AllianceStationID HAL_GetAllianceStation(std::int32_t* status);
double HAL_GetMatchTime(std::int32_t* status);
```

### Behavior

- No installed shim:
  - `HAL_GetControlWord`: zero the out-param and return
    `kHalHandleError`.
  - `HAL_GetAllianceStation`: write `*status = kHalHandleError` and
    return `HAL_AllianceStationID_kUnknown`.
  - `HAL_GetMatchTime`: write `*status = kHalHandleError` and return
    `0.0`.
- Installed shim, empty `latest_ds_state_`:
  - `HAL_GetControlWord`: zero the out-param and return `kHalSuccess`.
  - `HAL_GetAllianceStation`: write `*status = kHalSuccess` and return
    `HAL_AllianceStationID_kUnknown`.
  - `HAL_GetMatchTime`: write `*status = kHalSuccess` and return `0.0`.
- Installed shim, populated cache:
  - `HAL_GetControlWord`: return `kHalSuccess` and copy the six control
    bits from `latest_ds_state_->control.bits` into WPILib bitfield
    order: enabled, autonomous, test, eStop, fmsAttached, dsAttached.
  - `HAL_GetAllianceStation`: write `*status = kHalSuccess` and return
    the cached station enum value.
  - `HAL_GetMatchTime`: write `*status = kHalSuccess` and return
    `latest_ds_state_->match_time_seconds`.
- Latest-wins is inherited from cycle 3: the newest accepted
  `ds_state` cache value is what the C HAL readers return.

### Out of scope

- Driver Station packet parsing and FMS match-time policy. Sim Core owns
  the authoritative `ds_state` values.
- Joystick, descriptor, match-info, console, and error surfaces.
- NULL out-pointers. Existing HAL_* read surfaces treat required
  out-pointers as UB matching WPILib's unconditional C write-through
  shape; this cycle inherits that rule.
- Validating impossible enum values. The schema parity tests already pin
  the enum ABI; this C seam copies cached values.

---

## Decisions pinned

### New: D-C23-DS-SCALAR-ZERO-DEFAULT

No-shim and empty-cache paths write or return zero/default DS scalar
values. `HAL_GetControlWord` clears the full 32-bit word, not just the
six named fields, so reserved bits cannot leak stale caller storage.
`HAL_GetAllianceStation` defaults to unknown and `HAL_GetMatchTime`
defaults to `0.0`.

### New: D-C23-CONTROL-WORD-BITFIELD-ORDER

`HAL_GetControlWord` translates the protocol `control_word::bits`
mask into the official WPILib bitfield order:
enabled, autonomous, test, eStop, fmsAttached, dsAttached. Existing
common parity tests pin the same bit assignments against the WPILib
mirror; this cycle pins the C HAL read seam.

### New: D-C23-DS-READERS-SHARE-DS-CACHE

All three functions read from `latest_ds_state_` only. Empty-cache reads
must not materialize a zero `ds_state`, and calling one DS scalar reader
must not affect the others.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: `status` is written on every
  `HAL_GetAllianceStation` and `HAL_GetMatchTime` call; NULL `status`
  is UB.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim reports
  `kHalHandleError`.
- D-C3 latest-wins cache semantics for `latest_ds_state_`.

---

## Proposed tests

### C23-1. `HalGetControlWord.WithNoShimInstalledReturnsHandleErrorAndZerosWord`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR,
  D-C23-DS-SCALAR-ZERO-DEFAULT.
- **Setup:** `shim_core::install_global(nullptr)`; initialize a
  `HAL_ControlWord` out-param to all 1 bits.
- **Action:** call `HAL_GetControlWord(&word)`.
- **Expected:** return value is `kHalHandleError`; the complete 32-bit
  word is zero.
- **Bug class:** no-shim path leaves stale control bits or mistakenly
  reports success.

### C23-2. `HalGetControlWord.WithShimInstalledButCacheEmptyReturnsSuccessAndZerosWord`

- **Layer / contract:** empty-cache variant of
  D-C23-DS-SCALAR-ZERO-DEFAULT and D-C23-DS-READERS-SHARE-DS-CACHE.
- **Setup:** connected shim installed globally; no inbound `ds_state`
  has been polled; initialize `HAL_ControlWord` to all 1 bits. Assert
  `latest_ds_state()` is `nullopt` before the call.
- **Action:** call `HAL_GetControlWord(&word)`.
- **Expected:** return value is `kHalSuccess`; full 32-bit word is zero;
  `latest_ds_state()` remains `nullopt`.
- **Bug class:** empty cache reports handle error, leaks stale caller
  storage, or materializes a zero cache value as a read side effect.

### C23-3. `HalGetControlWord.WithCachedDsStateCopiesEachNamedBitAndClearsReservedBits`

- **Layer / contract:** D-C23-CONTROL-WORD-BITFIELD-ORDER.
- **Setup:** connected shim installed globally. For each one-hot case,
  inject/poll a `valid_ds_state(..., control_bits = <one named
  kControl*> ...)`; initialize the `HAL_ControlWord` out-param to all
  1 bits before each call. The six one-hot cases are:
  - `kControlEnabled` → only `enabled` is 1.
  - `kControlAutonomous` → only `autonomous` is 1.
  - `kControlTest` → only `test` is 1.
  - `kControlEStop` → only `eStop` is 1.
  - `kControlFmsAttached` → only `fmsAttached` is 1.
  - `kControlDsAttached` → only `dsAttached` is 1.
- **Action:** after each poll, call `HAL_GetControlWord(&word)`.
- **Expected:** every call returns `kHalSuccess`; the one expected
  named field is 1, the other five named fields are 0, and reserved
  bits are zero.
- **Bug class:** transposed bit order, partial assignment that leaves
  reserved stale bits, or wrong cache slot.

### C23-4. `HalGetAllianceStation.WithNoShimInstalledSetsHandleErrorAndReturnsUnknown`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR,
  D-C23-DS-SCALAR-ZERO-DEFAULT.
- **Setup:** `shim_core::install_global(nullptr)`; status initialized
  to a nonzero sentinel.
- **Action:** call `HAL_GetAllianceStation(&status)`.
- **Expected:** `status == kHalHandleError`; return value is
  `HAL_AllianceStationID_kUnknown`.
- **Bug class:** no-shim path returns stale enum or fails to write status.

### C23-5. `HalGetAllianceStation.WithShimInstalledButCacheEmptySetsSuccessAndReturnsUnknown`

- **Layer / contract:** empty-cache variant of
  D-C23-DS-SCALAR-ZERO-DEFAULT and D-C23-DS-READERS-SHARE-DS-CACHE.
- **Setup:** connected shim installed globally; no inbound `ds_state`;
  status sentinel; assert `latest_ds_state()` is `nullopt`.
- **Action:** call `HAL_GetAllianceStation(&status)`.
- **Expected:** `status == kHalSuccess`; return value is
  `HAL_AllianceStationID_kUnknown`; `latest_ds_state()` remains
  `nullopt`.
- **Bug class:** empty cache reports handle error, returns a non-default
  enum, or creates cache state as a read side effect.

### C23-6. `HalGetAllianceStation.WithCachedDsStateReturnsCachedStation`

- **Layer / contract:** cached read of `ds_state::station`.
- **Setup:** connected shim installed globally; inject/poll
  `valid_ds_state(..., station = alliance_station::blue_3, ...)`;
  status sentinel.
- **Action:** call `HAL_GetAllianceStation(&status)`.
- **Expected:** `status == kHalSuccess`; return value is
  `HAL_AllianceStationID_kBlue3`.
- **Bug class:** wrong enum mapping, wrong cache slot, or always-default
  implementation.

### C23-7. `HalGetMatchTime.WithNoShimInstalledSetsHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR,
  D-C23-DS-SCALAR-ZERO-DEFAULT.
- **Setup:** `shim_core::install_global(nullptr)`; status initialized
  to a nonzero sentinel.
- **Action:** call `HAL_GetMatchTime(&status)`.
- **Expected:** `status == kHalHandleError`; return value is `0.0`.
- **Bug class:** no-shim path returns stale/nonzero time or fails to
  write status.

### C23-8. `HalGetMatchTime.WithShimInstalledButCacheEmptySetsSuccessAndReturnsZero`

- **Layer / contract:** empty-cache variant of
  D-C23-DS-SCALAR-ZERO-DEFAULT and D-C23-DS-READERS-SHARE-DS-CACHE.
- **Setup:** connected shim installed globally; no inbound `ds_state`;
  status sentinel; assert `latest_ds_state()` is `nullopt`.
- **Action:** call `HAL_GetMatchTime(&status)`.
- **Expected:** `status == kHalSuccess`; return value is `0.0`;
  `latest_ds_state()` remains `nullopt`.
- **Bug class:** empty cache reports handle error, returns stale time,
  or creates cache state as a read side effect.

### C23-9. `HalGetMatchTime.WithCachedDsStateReturnsCachedMatchTime`

- **Layer / contract:** cached read of `ds_state::match_time_seconds`.
- **Setup:** connected shim installed globally; inject/poll
  `valid_ds_state(..., match_time_seconds = 135.25)`; status sentinel.
- **Action:** call `HAL_GetMatchTime(&status)`.
- **Expected:** `status == kHalSuccess`; return value is `135.25`.
- **Bug class:** wrong cache field, integer truncation, or always-zero
  implementation.

### C23-10. `HalDriverStationScalarReads.LatestWinsAcrossTwoDsUpdates`

- **Layer / contract:** inherited D-C3 latest-wins through all three C
  HAL seams.
- **Setup:** connected shim installed globally.
- **Action:** inject/poll first `ds_state` with control
  `kControlEnabled`, station `red_1`, match time `15.0`; read all
  three functions. Then inject/poll second `ds_state` with control
  `kControlAutonomous | kControlTest | kControlDsAttached`, station
  `blue_2`, match time `42.75`; read all three functions again.
- **Expected:** first read group returns first values; second read group
  returns second values. Every status/return code is success.
- **Bug class:** reader returns first/old cached DS state, mixes fields
  from different updates, or one reader uses a separate stale cache.

---

## Tests deliberately not added

- NULL out-pointer tests. This cycle inherits the existing C HAL
  required-pointer UB rule.
- Re-testing inbound `ds_state` framing, pre-boot rejection,
  padding-byte determinism, and cross-cache independence. Cycle 3
  already owns those contracts; this cycle tests the C ABI read seam.
- Joystick and match-info reads. Those are larger struct-copy surfaces
  and are intentionally reserved for later cycles.
- Exhaustive alliance enum coverage. Common parity tests already pin
  all enum values; C23-6 plus C23-10 are enough to catch wrong mapping
  and latest-wins behavior at this seam.
