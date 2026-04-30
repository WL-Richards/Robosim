# HAL shim core — cycle 29 test plan (DS match info + joystick descriptors)

Status: implemented

Cycle 29 closes the remaining per-slot Driver Station metadata reads backed
by `ds_state`: match info and joystick descriptors. Cycle 30 owns the
aggregate `HAL_GetAllJoystickData` surface so this cycle stays focused on
metadata and string/value descriptor getters.

Primary signatures, from WPILib release DriverStation HAL docs:

```cpp
int32_t HAL_GetMatchInfo(HAL_MatchInfo* info);
int32_t HAL_GetJoystickDescriptor(int32_t joystickNum,
                                  HAL_JoystickDescriptor* desc);
HAL_Bool HAL_GetJoystickIsXbox(int32_t joystickNum);
int32_t HAL_GetJoystickType(int32_t joystickNum);
void HAL_GetJoystickName(struct WPI_String* name, int32_t joystickNum);
int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis);
```

Relevant local schemas:

- `ds_state::match` is byte-compatible with `HAL_MatchInfo`.
- `ds_state::joystick_descriptors[joystickNum]` is byte-compatible with
  `HAL_JoystickDescriptor`.
- `WPI_String` is `{ const char* str; size_t len; }`.

## Decisions

- **D-C29-NO-SHIM:** No installed shim returns/produces zero/default
  values. Status-returning functions return `kHalHandleError`; value/string
  descriptor helpers have no status channel and return false/zero/empty.
- **D-C29-EMPTY-CACHE:** Installed shim with no cached `ds_state` returns
  success for status-returning functions and zero/default outputs. Value/string
  helpers return false/zero/empty.
- **D-C29-VALID-INDEX:** Valid joystick indices are `0..5`.
- **D-C29-INVALID-INDEX:** Invalid joystick indices return success with
  zero/default output for `HAL_GetJoystickDescriptor`, matching the WPILib
  descriptor documentation and cycle-24 invalid-index behavior. Value/string
  helpers return false/zero/empty.
- **D-C29-AXIS-RANGE:** `HAL_GetJoystickAxisType` returns the cached axis type
  only when both joystick index and axis index are valid (`0..11`); otherwise
  it returns 0.
- **D-C29-OUTPUT-POINTERS:** Output pointers are treated like existing HAL
  read surfaces: non-null is required. Tests do not pass null pointers.
- **D-C29-NAME:** `HAL_GetJoystickName` copies bytes from the fixed 256-byte
  descriptor name field up to the first NUL byte, or all 256 bytes if no NUL
  exists. The returned `WPI_String` is heap-owned and must be releasable with
  `std::free` in tests, matching WPILib's free-after-call ownership contract.
- **D-C29-LATEST-WINS:** All readers observe the latest cached `ds_state`.

## Proposed tests

### C29-0 — exported descriptor/match structs match backend ABI mirrors

- **Layer / contract:** Layer 2 C HAL ABI layout.
- **Bug class caught:** field-order, alignment, or spelling drift at the C
  seam that would make byte-copy unsafe.
- **Inputs:** `sizeof`, `alignof`, and `offsetof` checks for
  `HAL_JoystickDescriptor`, `HAL_MatchInfo`, and `WPI_String`.
- **Expected:** C structs match backend mirror sizes/offsets; `WPI_String` is
  pointer then `size_t`.
- **Determinism:** pure compile/runtime layout checks.

### C29-1 — no shim `HAL_GetMatchInfo` returns handle error and zeroes output

- **Layer / contract:** Layer 2 C HAL no-shim behavior.
- **Bug class caught:** stale caller bytes leak when the shim is absent.
- **Inputs:** no installed shim; sentinel-filled `HAL_MatchInfo`.
- **Expected:** return `kHalHandleError`; every output byte is zero.

### C29-2 — empty cache `HAL_GetMatchInfo` succeeds and zeroes output

- **Layer / contract:** Layer 2 DS empty-cache behavior.
- **Bug class caught:** treating missing first DS packet as an error.
- **Inputs:** installed connected shim with no `ds_state`; sentinel output.
- **Expected:** return `kHalSuccess`; zero output; cache remains empty.

### C29-3 — cached `HAL_GetMatchInfo` copies match info byte-for-byte

- **Layer / contract:** Layer 2 DS cached match metadata read.
- **Bug class caught:** lossy per-field copy, missed interior padding, or
  wrong enum backing copy.
- **Inputs:** cached `ds_state` with event name, match type, match number,
  replay number, game-specific bytes/size, and nonzero padding from the
  injected schema.
- **Expected:** return `kHalSuccess`; output bytes equal `ds_state::match`.

### C29-4 — no shim `HAL_GetJoystickDescriptor` returns handle error and defaults

- **Layer / contract:** Layer 2 C HAL no-shim behavior.
- **Bug class caught:** descriptor output not filled when unavailable.
- **Inputs:** no installed shim; valid joystick 0; sentinel descriptor.
- **Expected:** return `kHalHandleError`; output bytes are zero.

### C29-5 — empty cache `HAL_GetJoystickDescriptor` succeeds and defaults

- **Layer / contract:** Layer 2 DS empty-cache behavior.
- **Bug class caught:** accidental error on startup before the first DS packet.
- **Inputs:** installed shim with no `ds_state`; valid joystick 0; sentinel
  descriptor.
- **Expected:** return `kHalSuccess`; output bytes are zero.

### C29-6 — cached descriptors copy boundary slots byte-for-byte

- **Layer / contract:** Layer 2 per-slot descriptor read.
- **Bug class caught:** off-by-one slot selection or field-by-field copy drift.
- **Inputs:** cached `ds_state` with distinct descriptors in slots 0 and 5.
- **Expected:** both reads return `kHalSuccess` and byte-match the selected
  backend descriptors.

### C29-7 — invalid descriptor indices succeed with zero/default output

- **Layer / contract:** WPILib descriptor default-fill behavior plus cycle-24
  invalid-index convention.
- **Bug class caught:** out-of-bounds read or handle-error divergence for
  too-large joystick indices.
- **Inputs:** cached descriptor in slot 0; reads for -1 and 6.
- **Expected:** return `kHalSuccess`; output bytes are zero.

### C29-8 — no-shim scalar descriptor helpers return zero/default

- **Layer / contract:** Layer 2 C HAL no-shim behavior for descriptor scalar
  helpers with no status channel.
- **Bug class caught:** null-shim dereference or accidental nonzero default.
- **Inputs:** no installed shim; calls to `HAL_GetJoystickIsXbox(0)`,
  `HAL_GetJoystickType(0)`, and `HAL_GetJoystickAxisType(0, 0)`.
- **Expected:** all return `0`.

### C29-9 — empty-cache scalar descriptor helpers return zero/default

- **Layer / contract:** Layer 2 DS empty-cache behavior for descriptor scalar
  helpers.
- **Bug class caught:** treating missing first DS packet as a populated
  descriptor or reading stale memory.
- **Inputs:** installed connected shim with no `ds_state`; calls to
  `HAL_GetJoystickIsXbox(0)`, `HAL_GetJoystickType(0)`, and
  `HAL_GetJoystickAxisType(0, 0)`.
- **Expected:** all return `0`; cache remains empty.

### C29-10 — cached scalar descriptor helpers return cached fields

- **Layer / contract:** Layer 2 descriptor scalar helper reads.
- **Bug class caught:** wrong descriptor field, bool widening error, or
  off-by-one axis indexing.
- **Inputs:** cached slot 2 with `is_xbox=1`, `type=7`, and axis types
  `10..21`.
- **Expected:** `HAL_GetJoystickIsXbox(2) == 1`,
  `HAL_GetJoystickType(2) == 7`, and
  `HAL_GetJoystickAxisType(2, 0/11)` returns `10/21`.

### C29-11 — scalar descriptor helpers default invalid indices

- **Layer / contract:** Layer 2 descriptor scalar helper invalid-index
  behavior.
- **Bug class caught:** out-of-bounds descriptor or axis array read.
- **Inputs:** cached slot 2 with nonzero descriptor fields; invalid joystick
  `-1`/`6` for `IsXbox` and `Type`; invalid combinations for
  `AxisType`: valid joystick + axis `-1` and `12`, invalid joystick `-1` and
  `6` + valid axis `0`, and invalid joystick `-1` + invalid axis `12`.
- **Expected:** every invalid call returns `0`.

### C29-12 — `HAL_GetJoystickName` returns an owned `WPI_String`

- **Layer / contract:** Layer 2 descriptor string helper and ownership seam.
- **Bug class caught:** returning a pointer into cached state, including
  trailing zero-filled bytes in the length, or non-freeable memory.
- **Inputs:** cached slot 3 name `"Flight Stick"` with trailing zeros; call
  `HAL_GetJoystickName(&out, 3)`.
- **Expected:** `out.len == 12`, `out.str` contains the exact bytes without a
  trailing NUL requirement in the length, `out.str` is non-null for nonempty
  names, and `std::free(const_cast<char*>(out.str))` is valid.

### C29-13 — `HAL_GetJoystickName` handles default and full-length names

- **Layer / contract:** Layer 2 fixed-buffer-to-`WPI_String` conversion.
- **Bug class caught:** reading past the 256-byte field or truncating names
  that legitimately fill the whole fixed field.
- **Inputs:** no-shim/empty-cache/invalid-index calls; one cached descriptor
  whose 256 name bytes contain no NUL.
- **Expected:** unavailable paths return `{nullptr, 0}`; full field returns
  length 256 and byte-identical allocated data.

### C29-14 — descriptor readers observe latest-wins across DS updates

- **Layer / contract:** Layer 2 latest-cache semantics inherited from
  `latest_ds_state_`.
- **Bug class caught:** stale descriptor/match snapshots after a later poll.
- **Inputs:** inject first `ds_state`, read match info, descriptor,
  `HAL_GetJoystickIsXbox`, `HAL_GetJoystickType`, name, and axis type; inject
  second `ds_state` with different values; read again.
- **Expected:** first reads match first state; second reads match second state.

## Deferred

- `HAL_GetAllJoystickData` is Cycle 30.
- Joystick output/rumble and DS event handles remain future surfaces.
