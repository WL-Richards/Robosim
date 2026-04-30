# HAL shim core - cycle 36 test plan (HAL_SetJoystickOutputs)

Status: implemented

Cycle 36 implements:

```cpp
int32_t HAL_SetJoystickOutputs(int32_t joystickNum,
                               int64_t outputs,
                               int32_t leftRumble,
                               int32_t rightRumble);
```

Primary WPILib references:

- WPILib 2026.2.2 release C++ docs for `hal/DriverStation.h` list the
  signature above.
- The docs define `outputs` as a bitmask, and `leftRumble` /
  `rightRumble` as rumble values in the `0..0xFFFF` range.
- WPILib sim `DriverStation.cpp` forwards the call to
  `SimDriverStationData->SetJoystickOutputs(...)` and returns `0`.

## Decisions

- **D-C36-NO-SHIM:** No installed shim returns `kHalHandleError`.
- **D-C36-VALID-SLOTS:** Valid joystick indices are exactly `0..5`, matching
  `HAL_kMaxJoysticks`.
- **D-C36-INVALID-INDEX:** Invalid joystick indices return `kHalHandleError`
  and do not mutate any stored output slot. This differs from read surfaces
  that can return zero/default data; writes need an explicit failure result.
- **D-C36-STATE-MODEL:** The shim records per-shim joystick output state as
  `joystick_output_state { outputs, left_rumble, right_rumble }` for each
  valid slot. `shim_core::joystick_outputs(joystickNum)` is the documented
  host-facing v0 accessor and returns `std::optional<joystick_output_state>`.
- **D-C36-ACCESSOR-INVALID:** The host-facing accessor returns
  `std::nullopt` for invalid joystick indices, without touching storage.
- **D-C36-EMPTY-DEFAULT:** A fresh shim has no recorded joystick output state
  for any slot.
- **D-C36-LATEST-WINS:** Repeated calls for the same joystick replace that
  slot's latest state without affecting other slots.
- **D-C36-RAW-VALUES:** v0 stores the signed C ABI arguments exactly. It does
  not clamp rumble values or mask output bits; higher-level WPILib callers are
  expected to send documented rumble ranges, and preserving raw values catches
  accidental narrowing at the HAL seam.
- **D-C36-OUTBOUND-DEFERRED:** Observer-state publication to Sim Core is
  deferred until a dedicated protocol schema or request/reply surface exists.
  Existing outbound schemas (`can_frame_batch`, `notifier_state`,
  `error_message_batch`) must not be reused for joystick outputs.
- **D-C36-POST-SHUTDOWN:** `HAL_Shutdown` clears the global shim pointer, so
  later `HAL_SetJoystickOutputs` calls return `kHalHandleError` and do not
  mutate the previously installed shim object.

## Proposed tests

### C36-1 - C HAL signature matches WPILib
- Layer / contract: C HAL ABI declaration.
- Bug class caught: returning `void`, narrowing `outputs` to `int32_t`,
  omitting the declaration from `hal_c.h`, or changing parameter order.
- Inputs: compile-time type check using
  `decltype(&HAL_SetJoystickOutputs)`.
- Expected: type is exactly
  `std::int32_t (*)(std::int32_t, std::int64_t, std::int32_t,
  std::int32_t)`.

### C36-2 - no shim returns handle error
- Layer / contract: Layer 2 C HAL no-shim behavior for a status-returning DS
  write surface.
- Bug class caught: null global shim dereference or false success when the
  write has nowhere to go.
- Inputs: clear global shim; call `HAL_SetJoystickOutputs(0, 1, 2, 3)`.
- Expected: returns `kHalHandleError`.

### C36-3 - fresh shim has no joystick outputs recorded
- Layer / contract: host-facing `shim_core::joystick_outputs` v0 API.
- Bug class caught: defaulting fresh output slots to active zero commands.
- Inputs: installed connected shim; query all six slots.
- Expected: every slot returns `std::nullopt`.

### C36-4 - accessor invalid indices return nullopt
- Layer / contract: D-C36-ACCESSOR-INVALID host-facing API boundary.
- Bug class caught: out-of-bounds reads or modulo indexing in the host
  accessor.
- Inputs: installed connected shim; query `joystick_outputs(-1)` and
  `joystick_outputs(6)`.
- Expected: both return `std::nullopt`.

### C36-5 - valid slots record exact output and rumble values
- Layer / contract: WPILib `HAL_SetJoystickOutputs` parameter mapping.
- Bug class caught: swapping left/right rumble, truncating the 64-bit output
  bitmask, or writing the wrong joystick slot.
- Inputs: call slot 0 with `outputs = 0x1'0000'0001`, `leftRumble = 0`,
  `rightRumble = 0xFFFF`; call slot 5 with `outputs = -1`,
  `leftRumble = 0x1234`, `rightRumble = 0x5678`.
- Expected: both slots report the exact values passed; untouched slots remain
  `std::nullopt`; return value is `kHalSuccess`.

### C36-6 - invalid joystick indices report handle error and preserve state
- Layer / contract: D-C36-INVALID-INDEX write failure behavior.
- Bug class caught: out-of-bounds array writes, modulo indexing, or mutating a
  valid slot on invalid input.
- Inputs: set slot 0 to a sentinel state; call indices `-1` and `6` with
  different values.
- Expected: invalid calls return `kHalHandleError`; slot 0 still has the
  sentinel state; all other slots remain `std::nullopt`.

### C36-7 - repeated writes are latest-wins per slot
- Layer / contract: periodic DS output command behavior.
- Bug class caught: first-write-wins, append-only behavior, or cross-slot
  clearing on update.
- Inputs: set slot 2; set slot 3; set slot 2 again with different values.
- Expected: slot 2 has the second slot-2 state; slot 3 is unchanged.

### C36-8 - raw rumble and output values are preserved
- Layer / contract: D-C36-RAW-VALUES at the C ABI seam.
- Bug class caught: clamping rumble to `0..0xFFFF`, narrowing outputs to
  32 bits, or treating negative values as invalid before a protocol contract
  exists.
- Inputs: call slot 1 with a high-bit `int64_t` output mask, negative
  `leftRumble`, and `rightRumble > 0xFFFF`.
- Expected: call returns `kHalSuccess`; accessor reports the exact signed
  values passed.

### C36-9 - state is per shim object
- Layer / contract: no leakage across installed shim objects.
- Bug class caught: process-global joystick output storage.
- Inputs: install shim A and set slot 4; install shim B and set slot 4 to
  different values.
- Expected: shim A keeps its state; shim B has its own state.

### C36-10 - calls do not publish to existing outbound schemas
- Layer / contract: D-C36-OUTBOUND-DEFERRED.
- Bug class caught: encoding joystick outputs through unrelated outbound
  schema traffic.
- Inputs: create shim, drain boot, verify outbound lane empty, install shim,
  call `HAL_SetJoystickOutputs` for a valid slot.
- Expected: call returns `kHalSuccess`; state is recorded; outbound lane
  remains empty.

### C36-11 - shutdown makes later calls fail without mutating old shim
- Layer / contract: Cycle 28 shutdown/global-detach semantics inherited by
  this status-returning write surface.
- Bug class caught: wrappers retaining stale shim pointers after shutdown.
- Inputs: install shim; set slot 0; call `HAL_Shutdown`; call
  `HAL_SetJoystickOutputs(0, ...)`.
- Expected: post-shutdown call returns `kHalHandleError`; old slot 0 state is
  unchanged.

## Deferred

- Protocol publication of joystick outputs/rumble state.
- Driver Station output packet packing and timing.
- Clamping or validating rumble ranges beyond preserving the C ABI values.
