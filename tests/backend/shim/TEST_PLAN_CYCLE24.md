# HAL shim core — cycle 24 test plan (Driver Station joystick reads)

**Status:** implemented. Round-1 verdict was `not-ready`: the reviewer
approved the overall behavior but required explicit `HAL_Joystick*`
layout assertions against the backend structs, valid populated-cache
boundary coverage for joystick indices 0 and 5, and full byte-equality
assertions in the latest-wins test. Revision 2 resolved those findings
and received `ready-to-implement`.

**Implements:** the twenty-fourth TDD cycle of the HAL shim core. Cycle
24 wires the three Driver Station joystick C HAL read surfaces:

```c
int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes);
int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs);
int32_t HAL_GetJoystickButtons(int32_t joystickNum,
                               HAL_JoystickButtons* buttons);
```

Official WPILib 2026.2.2 generated docs for `hal/DriverStation.h`
define these signatures. `docs/HAL_SHIM_NEXT_SURFACES_REPORT.md`
recommends joystick reads immediately after the Driver Station scalar
cycle and captures the WPILib struct constants already mirrored locally:
`HAL_kMaxJoysticks = 6`, `HAL_kMaxJoystickAxes = 12`, and
`HAL_kMaxJoystickPOVs = 12`.

---

## Why this cycle exists

Cycle 3 added the inbound `ds_state` cache slot and cycle 23 exposed
the first scalar Driver Station reads. Cycle 24 exposes the joystick
array subset of the same cache:

- `ds_state::joystick_axes_[joystickNum]` through
  `HAL_GetJoystickAxes`.
- `ds_state::joystick_povs_[joystickNum]` through
  `HAL_GetJoystickPOVs`.
- `ds_state::joystick_buttons_[joystickNum]` through
  `HAL_GetJoystickButtons`.

The main bug classes are wrong joystick index, wrong array, stale caller
struct bytes on default paths, out-of-range index policy drift, partial
struct copies that leave padding or array suffix stale, and returning a
previously-read DS value instead of the current cache.

---

## Contract under test

### New extern "C" surface

`src/backend/shim/hal_c.h`:

```cpp
inline constexpr std::int32_t HAL_kMaxJoystickAxes = 12;
inline constexpr std::int32_t HAL_kMaxJoystickPOVs = 12;
inline constexpr std::int32_t HAL_kMaxJoysticks = 6;

struct HAL_JoystickAxes {
  std::int16_t count;
  float axes[HAL_kMaxJoystickAxes];
  std::uint8_t raw[HAL_kMaxJoystickAxes];
};

struct HAL_JoystickPOVs {
  std::int16_t count;
  std::int16_t povs[HAL_kMaxJoystickPOVs];
};

struct HAL_JoystickButtons {
  std::uint32_t buttons;
  std::uint8_t count;
};

std::int32_t HAL_GetJoystickAxes(std::int32_t joystickNum,
                                 HAL_JoystickAxes* axes);
std::int32_t HAL_GetJoystickPOVs(std::int32_t joystickNum,
                                 HAL_JoystickPOVs* povs);
std::int32_t HAL_GetJoystickButtons(std::int32_t joystickNum,
                                    HAL_JoystickButtons* buttons);
```

### Behavior

- No installed shim: return `kHalHandleError` and zero the full output
  struct bytes.
- Installed shim, empty `latest_ds_state_`: return `kHalSuccess`, zero
  the full output struct bytes, and do not materialize a cache value.
- Installed shim, populated cache, valid `joystickNum` in `[0, 5]`:
  return `kHalSuccess` and byte-copy the corresponding cached struct.
- Installed shim, populated cache, invalid `joystickNum` (`< 0` or
  `>= HAL_kMaxJoysticks`): return `kHalSuccess` and zero the full output
  struct bytes.
- Latest-wins is inherited from cycle 3: the newest accepted `ds_state`
  cache value is what the C HAL readers return.

### Out of scope

- Joystick descriptor, match-info, and all-joystick aggregate reads.
- Driver Station packet parsing and device normalization. Sim Core owns
  the authoritative `ds_state` values.
- Validation of counts inside cached structs. The shim copies the
  accepted schema value; protocol/schema tests own layout parity.
- NULL out-pointers. Existing HAL_* read surfaces treat required
  out-pointers as UB matching WPILib's unconditional C write-through
  shape; this cycle inherits that rule.

---

## Decisions pinned

### New: D-C24-JOYSTICK-STRUCT-ZERO-DEFAULT

No-shim, empty-cache, and invalid-index paths write zero to every byte
of the requested output struct. This includes implicit padding in
`HAL_JoystickAxes` between `count` and `axes`, and trailing padding in
`HAL_JoystickButtons`.

### New: D-C24-JOYSTICK-INDEX-RANGE

Valid joystick indices are exactly `0 <= joystickNum < 6`, matching
`HAL_kMaxJoysticks`. Negative values and values >= 6 are invalid. v0
returns `kHalSuccess` with zero/default output for invalid indices,
matching the recommendation in `docs/HAL_SHIM_NEXT_SURFACES_REPORT.md`
and avoiding new status constants before Rio parity data.

### New: D-C24-JOYSTICK-STRUCT-BYTE-COPY

For valid populated-cache reads, each function copies the complete
byte-compatible cached struct into the C HAL output struct. Tests assert
byte equality, not just public fields, because the HAL ABI structs have
padding that must not retain caller sentinels.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim reports
  `kHalHandleError`.
- D-C23-DS-READERS-SHARE-DS-CACHE: DS readers read from
  `latest_ds_state_` only.
- D-C3 latest-wins cache semantics for `latest_ds_state_`.

---

## Proposed tests

### C24-0. `HalJoystickStructLayout.MatchesBackendJoystickAbiMirrors`

- **Layer / contract:** exported `hal_c.h` ABI mirrors for joystick
  structs.
- **Setup:** compile-time/static layout checks plus offset assertions.
- **Action:** no runtime action beyond the test body's assertions.
- **Expected:** `HAL_JoystickAxes`, `HAL_JoystickPOVs`, and
  `HAL_JoystickButtons` match `backend::joystick_axes`,
  `backend::joystick_povs`, and `backend::joystick_buttons` for
  `sizeof`, `alignof`, and every named field offset.
- **Bug class:** new exported C ABI struct drifts from the already
  WPILib-pinned backend schema struct.

### C24-1. `HalGetJoystickAxes.WithNoShimInstalledReturnsHandleErrorAndZerosOutput`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR,
  D-C24-JOYSTICK-STRUCT-ZERO-DEFAULT.
- **Setup:** `shim_core::install_global(nullptr)`; initialize a
  `HAL_JoystickAxes` out-param to nonzero sentinel bytes.
- **Action:** call `HAL_GetJoystickAxes(0, &axes)`.
- **Expected:** return value is `kHalHandleError`; every byte of
  `axes` is zero.
- **Bug class:** no-shim path leaves stale fields/padding or falsely
  reports success.

### C24-2. `HalGetJoystickAxes.WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput`

- **Layer / contract:** empty-cache variant of
  D-C24-JOYSTICK-STRUCT-ZERO-DEFAULT and D-C23-DS-READERS-SHARE-DS-CACHE.
- **Setup:** connected shim installed globally; no inbound `ds_state`;
  output initialized to sentinel bytes. Assert `latest_ds_state()` is
  `nullopt` before the call.
- **Action:** call `HAL_GetJoystickAxes(0, &axes)`.
- **Expected:** return value is `kHalSuccess`; every byte of `axes` is
  zero; `latest_ds_state()` remains `nullopt`.
- **Bug class:** empty cache reports handle error, leaks stale bytes, or
  materializes a zero cache value as a read side effect.

### C24-3. `HalGetJoystickAxes.WithCachedDsStateCopiesBoundaryAxisSlotsByteForByte`

- **Layer / contract:** D-C24-JOYSTICK-STRUCT-BYTE-COPY.
- **Setup:** connected shim installed globally; build a `ds_state` with
  joystick slot 0 and slot 5 containing distinct `joystick_axes`
  values, including `count`, multiple `axes[]`, and `raw[]` bytes.
  Inject/poll it. Initialize output to sentinel bytes.
- **Action:** call `HAL_GetJoystickAxes(0, &axes)` and
  `HAL_GetJoystickAxes(5, &axes)` with a freshly-sentinel-filled output
  for each subcase.
- **Expected:** both return `kHalSuccess`; output bytes equal
  `state.joystick_axes_[0]` and `state.joystick_axes_[5]`
  respectively.
- **Bug class:** wrong index, wrong array, field-only partial copy, or
  stale padding/array suffix.

### C24-4. `HalGetJoystickAxes.WithInvalidIndicesReturnsSuccessAndZerosOutput`

- **Layer / contract:** D-C24-JOYSTICK-INDEX-RANGE.
- **Setup:** connected shim installed globally with a populated
  `ds_state` whose joystick 0 axes are nonzero; output initialized to
  sentinel bytes for each subcase.
- **Action:** call `HAL_GetJoystickAxes(-1, &axes)` and
  `HAL_GetJoystickAxes(6, &axes)`.
- **Expected:** both return `kHalSuccess`; every byte of each output is
  zero.
- **Bug class:** negative index wraps to a large unsigned index, index 6
  reads past the array, or invalid index leaves stale output.

### C24-5. `HalGetJoystickPOVs.WithNoShimInstalledReturnsHandleErrorAndZerosOutput`

- **Layer / contract:** no-shim path for POV reader.
- **Setup:** no installed shim; `HAL_JoystickPOVs` initialized to
  nonzero sentinel bytes.
- **Action:** call `HAL_GetJoystickPOVs(0, &povs)`.
- **Expected:** `kHalHandleError`; every byte of `povs` is zero.
- **Bug class:** no-shim path leaves stale POV bytes or reports success.

### C24-6. `HalGetJoystickPOVs.WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput`

- **Layer / contract:** empty-cache POV reader.
- **Setup:** connected shim installed globally; no inbound `ds_state`;
  sentinel output; assert `latest_ds_state()` is `nullopt`.
- **Action:** call `HAL_GetJoystickPOVs(0, &povs)`.
- **Expected:** `kHalSuccess`; every byte of `povs` is zero;
  `latest_ds_state()` remains `nullopt`.
- **Bug class:** empty cache reports handle error, leaks stale POV
  bytes, or materializes cache state.

### C24-7. `HalGetJoystickPOVs.WithCachedDsStateCopiesBoundaryPovSlotsByteForByte`

- **Layer / contract:** D-C24-JOYSTICK-STRUCT-BYTE-COPY.
- **Setup:** connected shim installed globally; build a `ds_state` with
  joystick slot 0 and slot 5 containing distinct POV data, including
  `count` and multiple `povs[]` values. Inject/poll it. Sentinel output.
- **Action:** call `HAL_GetJoystickPOVs(0, &povs)` and
  `HAL_GetJoystickPOVs(5, &povs)` with a freshly-sentinel-filled output
  for each subcase.
- **Expected:** both return `kHalSuccess`; output bytes equal
  `state.joystick_povs_[0]` and `state.joystick_povs_[5]`
  respectively.
- **Bug class:** wrong index, wrong array, field-only partial copy.

### C24-8. `HalGetJoystickPOVs.WithInvalidIndicesReturnsSuccessAndZerosOutput`

- **Layer / contract:** D-C24-JOYSTICK-INDEX-RANGE.
- **Setup:** connected shim installed globally with populated nonzero
  POV data; sentinel output for each subcase.
- **Action:** call `HAL_GetJoystickPOVs(-1, &povs)` and
  `HAL_GetJoystickPOVs(6, &povs)`.
- **Expected:** both return `kHalSuccess`; every byte of each output is
  zero.
- **Bug class:** invalid index reads outside the array or leaks stale
  caller output.

### C24-9. `HalGetJoystickButtons.WithNoShimInstalledReturnsHandleErrorAndZerosOutput`

- **Layer / contract:** no-shim path for buttons reader.
- **Setup:** no installed shim; `HAL_JoystickButtons` initialized to
  nonzero sentinel bytes.
- **Action:** call `HAL_GetJoystickButtons(0, &buttons)`.
- **Expected:** `kHalHandleError`; every byte of `buttons` is zero.
- **Bug class:** no-shim path leaves stale buttons/count/padding or
  reports success.

### C24-10. `HalGetJoystickButtons.WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput`

- **Layer / contract:** empty-cache buttons reader.
- **Setup:** connected shim installed globally; no inbound `ds_state`;
  sentinel output; assert `latest_ds_state()` is `nullopt`.
- **Action:** call `HAL_GetJoystickButtons(0, &buttons)`.
- **Expected:** `kHalSuccess`; every byte of `buttons` is zero;
  `latest_ds_state()` remains `nullopt`.
- **Bug class:** empty cache reports handle error, leaks stale
  buttons/count/padding, or materializes cache state.

### C24-11. `HalGetJoystickButtons.WithCachedDsStateCopiesRequestedButtonsSlotByteForByte`

- **Layer / contract:** D-C24-JOYSTICK-STRUCT-BYTE-COPY.
- **Setup:** connected shim installed globally; build a `ds_state` with
  joystick slot 0 and slot 5 containing distinct button masks and
  counts. Inject/poll it. Sentinel output.
- **Action:** call `HAL_GetJoystickButtons(5, &buttons)`.
- **Expected:** `kHalSuccess`; output bytes equal
  `state.joystick_buttons_[5]`, not slot 0; trailing padding bytes are
  zero from the cached schema value, not caller sentinels.
- **Bug class:** wrong index, wrong array, field-only partial copy that
  leaves trailing padding stale.

### C24-12. `HalGetJoystickButtons.WithInvalidIndicesReturnsSuccessAndZerosOutput`

- **Layer / contract:** D-C24-JOYSTICK-INDEX-RANGE.
- **Setup:** connected shim installed globally with populated nonzero
  button data; sentinel output for each subcase.
- **Action:** call `HAL_GetJoystickButtons(-1, &buttons)` and
  `HAL_GetJoystickButtons(6, &buttons)`.
- **Expected:** both return `kHalSuccess`; every byte of each output is
  zero.
- **Bug class:** invalid index reads outside the array or leaks stale
  caller output.

### C24-13. `HalJoystickReads.LatestWinsAcrossTwoDsUpdates`

- **Layer / contract:** inherited D-C3 latest-wins through all three
  joystick C HAL seams.
- **Setup:** connected shim installed globally.
- **Action:** inject/poll first `ds_state` with nonzero axes in slot 0,
  POVs in slot 1, and buttons in slot 2; read all three. Then
  inject/poll second `ds_state` with distinct values in the same slots;
  read all three again.
- **Expected:** first read group returns bytes equal to the first cached
  structs; second read group returns bytes equal to the second cached
  structs. Every return code is `kHalSuccess`.
- **Bug class:** reader returns first/old cached DS state, mixes fields
  from different updates, or one reader uses a separate stale cache.

---

## Tests deliberately not added

- NULL out-pointer tests. This cycle inherits the existing C HAL
  required-pointer UB rule.
- Joystick descriptor and all-joystick aggregate reads. They are related
  but separate C HAL surfaces.
- Re-testing inbound `ds_state` framing, pre-boot rejection,
  padding-byte determinism, and cross-cache independence. Cycle 3
  already owns those contracts; this cycle tests the C ABI read seam.
- Exhaustive count-boundary validation inside cached joystick structs.
  This cycle copies accepted schema state; future DS ingestion/parity
  work owns clamping or rejecting impossible DS packet contents.
