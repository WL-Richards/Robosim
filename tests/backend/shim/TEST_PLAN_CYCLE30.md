# HAL shim core — cycle 30 test plan (HAL_GetAllJoystickData)

Status: implemented

Cycle 30 implements the aggregate Driver Station joystick read surface after
Cycle 29 closes match/descriptor metadata:

```cpp
void HAL_GetAllJoystickData(HAL_JoystickAxes* axes,
                            HAL_JoystickPOVs* povs,
                            HAL_JoystickButtons* buttons);
```

This is a convenience C HAL surface that copies all six joystick axes, POV,
and button slots in one call. It has no status return.

## Decisions

- **D-C30-NO-SHIM:** No installed shim zeroes all provided outputs.
- **D-C30-EMPTY-CACHE:** Installed shim with no cached `ds_state` zeroes all
  provided outputs.
- **D-C30-CACHED:** Populated cache copies all six slots from
  `ds_state::joystick_axes_`, `ds_state::joystick_povs_`, and
  `ds_state::joystick_buttons_`.
- **D-C30-OUTPUT-POINTERS:** Output pointers are required to be non-null,
  matching the existing shim read-surface convention. Tests do not pass null
  pointers.
- **D-C30-LATEST-WINS:** The aggregate surface observes the same latest cached
  `ds_state` as the per-slot Cycle 24 readers.

## Proposed tests

### C30-1 — no shim zeroes all aggregate joystick outputs

- **Layer / contract:** Layer 2 C HAL no-shim behavior for void aggregate
  read.
- **Bug class caught:** stale caller bytes leak when the shim is absent.
- **Inputs:** no installed shim; sentinel-filled arrays:
  `HAL_JoystickAxes[6]`, `HAL_JoystickPOVs[6]`, `HAL_JoystickButtons[6]`.
- **Expected:** every byte of all three arrays is zero.

### C30-2 — empty cache zeroes all aggregate joystick outputs

- **Layer / contract:** Layer 2 DS empty-cache behavior.
- **Bug class caught:** treating missing first DS packet as leaving caller
  memory untouched.
- **Inputs:** installed connected shim with no `ds_state`; sentinel arrays.
- **Expected:** every byte of all arrays is zero; cache remains empty.

### C30-3 — cached DS state copies all six joystick arrays byte-for-byte

- **Layer / contract:** Layer 2 aggregate joystick data read.
- **Bug class caught:** copying only slot 0, mixing axes/POV/button arrays,
  or copying only logical counts instead of fixed ABI structs.
- **Inputs:** cached `ds_state` with distinct axes, POVs, and buttons in all
  six joystick slots.
- **Expected:** each output slot byte-matches the corresponding backend slot.

### C30-4 — aggregate read matches per-slot readers for the same cache

- **Layer / contract:** Cross-surface consistency with Cycle 24.
- **Bug class caught:** aggregate and per-slot surfaces diverge in struct
  layout or slot ordering.
- **Inputs:** cached `ds_state` with distinct values in slots 0, 3, and 5.
- **Expected:** aggregate outputs for those slots byte-match outputs returned
  by `HAL_GetJoystickAxes/POVs/Buttons` for the same slots.

### C30-5 — aggregate joystick data observes latest-wins across DS updates

- **Layer / contract:** Layer 2 latest-cache semantics.
- **Bug class caught:** stale aggregate snapshot retained after a later poll.
- **Inputs:** inject first DS state and read aggregate data; inject second DS
  state with different values in multiple slots and read again.
- **Expected:** first aggregate output matches first state; second output
  matches second state.

## Deferred

- Null output pointer tolerance is intentionally not implemented in v0.
- DS event handles, joystick outputs/rumble, and refresh-observer calls remain
  future surfaces.
