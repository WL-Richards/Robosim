# HAL Shim Next Surfaces Report

Date: 2026-04-30

This report captures read-only prep for the HAL shim cycles after Cycle 22.
It avoids the active Cycle 22 files and is intended as planning input for the
next TDD slices.

## Scope

Reviewed likely next HAL surfaces:

- `HAL_GetControlWord`
- `HAL_GetAllianceStation`
- `HAL_GetMatchTime`
- `HAL_GetJoystickAxes`
- `HAL_GetJoystickPOVs`
- `HAL_GetJoystickButtons`
- `HAL_Notifier*`
- `HAL_CAN_ReceiveMessage`
- `HAL_Initialize`
- `HAL_Shutdown`

Primary references:

- WPILib 2026.2.2 generated C++ docs for `hal/DriverStation.h`.
- WPILib generated source/details for `hal/DriverStationTypes.h`.
- WPILib 2026.2.2 generated C++ docs for notifier functions.
- WPILib 2026.2.2 generated C++ docs for `HALBase` lifecycle functions.
- Existing local parity mirrors and schema pins in `src/backend/common/` and
  `tests/backend/common/wpilib_mirror.h`.

Note: some Doxygen source pages report older generated labels even when the
release reference page says 2026.2.2. Treat the release reference pages as the
authority for exported signatures, and use source pages only for struct layout
details that are also pinned locally.

## WPILib Signatures And Constants

Driver Station scalar and joystick signatures:

```cpp
int32_t HAL_GetControlWord(HAL_ControlWord* controlWord);
HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status);
double HAL_GetMatchTime(int32_t* status);
int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes);
int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs);
int32_t HAL_GetJoystickButtons(int32_t joystickNum,
                               HAL_JoystickButtons* buttons);
```

Driver Station struct and enum facts already mirrored locally:

- `HAL_ControlWord`: bit 0 enabled, bit 1 autonomous, bit 2 test, bit 3
  eStop, bit 4 fmsAttached, bit 5 dsAttached, bits 6..31 reserved.
- `HAL_AllianceStationID`: unknown `0`, red 1..3 as `1..3`, blue 1..3 as
  `4..6`.
- `HAL_MatchType`: none `0`, practice `1`, qualification `2`, elimination
  `3`.
- `HAL_kMaxJoystickAxes = 12`.
- `HAL_kMaxJoystickPOVs = 12`.
- `HAL_kMaxJoysticks = 6`.
- `HAL_JoystickAxes`: `int16_t count`, `float axes[12]`, `uint8_t raw[12]`.
- `HAL_JoystickPOVs`: `int16_t count`, `int16_t povs[12]`.
- `HAL_JoystickButtons`: `uint32_t buttons`, `uint8_t count`.

Notifier signatures:

```cpp
HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status);
HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime,
                                       int32_t priority,
                                       int32_t* status);
void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                         const char* name,
                         int32_t* status);
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status);
void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle);
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime,
                             int32_t* status);
void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status);
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status);
```

Lifecycle signatures:

```cpp
HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode);
void HAL_Shutdown(void);
```

CAN caution:

The local shim currently mirrors the older `HAL_CANStreamMessage` layout:

```cpp
struct HAL_CANStreamMessage {
  uint32_t messageID;
  uint32_t timeStamp;
  uint8_t data[8];
  uint8_t dataSize;
};
```

Generated CAN docs for newer surfaces show bus-ID based signatures and
`HAL_CANMessage` / `HAL_CANReceiveMessage`. Before implementing
`HAL_CAN_ReceiveMessage`, re-confirm the exact pinned header at the local
WPILib parity SHA, not only generated Doxygen pages.

## Recommended Cycle Ordering

Recommended order after Cycle 22:

1. Driver Station scalar reads:
   `HAL_GetControlWord`, `HAL_GetAllianceStation`, `HAL_GetMatchTime`.
2. Joystick reads:
   `HAL_GetJoystickAxes`, `HAL_GetJoystickPOVs`, `HAL_GetJoystickButtons`.
3. Notifier APIs.
4. One-shot CAN receive, after resolving the exact pinned signature.
5. Initialize/shutdown lifecycle.

Rationale: the scalar Driver Station reads are the smallest extension of the
existing cache-read pattern. Joystick reads then reuse the same `ds_state`
cache with index/default-output decisions. Notifier APIs introduce handle
allocation and outbound state updates. CAN receive has the highest signature
drift risk. Lifecycle should be isolated because it changes global process
semantics.

## Status Constants Audit

Existing local constants in `src/backend/shim/hal_c.h`:

```cpp
kHalSuccess                  = 0
kHalHandleError              = -1
kHalCanInvalidBuffer         = -44086
kHalCanMessageNotFound       = -44087
kHalCanNoToken               = 44087
kHalCanNotAllowed            = -44088
kHalCanNotInitialized        = -44089
kHalCanSessionOverrun        = 44050
kHalCanSendPeriodStopRepeating = -1
```

Recommended v0 status behavior:

- DS scalar reads:
  - No shim installed: `kHalHandleError`, zero/default return or output.
  - Installed shim, empty `ds_state` cache: `kHalSuccess`, zero/default
    return or output.
  - Populated cache: `kHalSuccess`, return cached field.
- Joystick reads:
  - No shim installed: `kHalHandleError`, zero/default output.
  - Installed shim, empty `ds_state` cache: `kHalSuccess`, zero/default
    output.
  - Populated cache and valid index: `kHalSuccess`, byte-copy cached struct.
  - Out-of-range index: unresolved. Recommended v0 choice is `kHalSuccess`
    with zero/default output, but this should be called out explicitly in the
    cycle plan because WPILib docs for axes/buttons/POVs do not state it as
    clearly as `HAL_GetJoystickDescriptor`.
- Notifier APIs:
  - Success: `kHalSuccess`.
  - Invalid or closed notifier handle: likely `kHalHandleError`.
  - `HAL_SetNotifierThreadPriority` invalid priority range should either map
    to WPILib thread-priority status constants or be explicitly deferred as a
    v0 no-op/success decision.
  - `HAL_WaitForNotifierAlarm` stopped notifier: return `0` with success-like
    status, matching the documented "stop wakes wait with time 0" behavior.
- CAN one-shot receive:
  - No shim installed: `kHalHandleError`.
  - Invalid output buffer: `kHalCanInvalidBuffer`.
  - No matching frame: likely `kHalCanMessageNotFound` or `kHalCanNoToken`;
    resolve against the exact pinned header/API before plan review.
  - Invalid handle/session style state, if applicable: `kHalCanNotAllowed`.
  - Uninitialized CAN backend, if modeled: `kHalCanNotInitialized`.
- Lifecycle:
  - `HAL_Initialize` returns `HAL_Bool`, not status.
  - `HAL_Shutdown` returns `void`.
  - Define idempotency and no-installed-shim behavior in its own plan.

## Commit Slicing Prep

Suggested commit slices:

- Cycle 19, `HAL_SendError`:
  `src/backend/common/error_message.h`,
  `src/backend/shim/shim_core.h`,
  `src/backend/shim/shim_core.cpp`,
  `src/backend/shim/hal_c.h`,
  `src/backend/shim/hal_c.cpp`,
  `tests/backend/shim/shim_core_test.cpp`,
  `tests/backend/shim/TEST_PLAN_CYCLE19.md`,
  related status/docs.
- Cycle 20, `HAL_CAN_SendMessage`:
  `src/backend/common/can_frame.h`,
  `src/backend/shim/shim_core.h`,
  `src/backend/shim/shim_core.cpp`,
  `src/backend/shim/hal_c.h`,
  `src/backend/shim/hal_c.cpp`,
  `tests/backend/shim/shim_core_test.cpp`,
  `tests/backend/shim/TEST_PLAN_CYCLE20.md`,
  related status/docs.
- Cycle 21, CAN stream sessions:
  `src/backend/common/can_frame.h`,
  `src/backend/shim/shim_core.h`,
  `src/backend/shim/shim_core.cpp`,
  `src/backend/shim/hal_c.h`,
  `src/backend/shim/hal_c.cpp`,
  `tests/backend/shim/shim_core_test.cpp`,
  `tests/backend/shim/TEST_PLAN_CYCLE21.md`,
  related status/docs.
- Cycle 22, CAN status:
  `src/backend/common/can_status.h`,
  `src/backend/shim/shim_core.h`,
  `src/backend/shim/shim_core.cpp`,
  `src/backend/shim/hal_c.h`,
  `src/backend/shim/hal_c.cpp`,
  `tests/backend/shim/shim_core_test.cpp`,
  `tests/backend/shim/TEST_PLAN_CYCLE22.md`,
  related status/docs.

The repo is currently dirty across several broad areas, so use path-limited
staging when committing these slices.

## DS Schema Mapping Review

`ds_state` covers the requested Driver Station getters:

- `HAL_GetControlWord` maps to `ds_state::control`.
- `HAL_GetAllianceStation` maps to `ds_state::station`.
- `HAL_GetMatchTime` maps to `ds_state::match_time_seconds`.
- `HAL_GetJoystickAxes` maps to `ds_state::joystick_axes_[joystickNum]`.
- `HAL_GetJoystickPOVs` maps to `ds_state::joystick_povs_[joystickNum]`.
- `HAL_GetJoystickButtons` maps to
  `ds_state::joystick_buttons_[joystickNum]`.

Fields already available for future DS surfaces:

- `HAL_GetJoystickDescriptor`, `HAL_GetJoystickIsXbox`,
  `HAL_GetJoystickType`, `HAL_GetJoystickName`,
  `HAL_GetJoystickAxisType`: `ds_state::joystick_descriptors`.
- `HAL_GetMatchInfo`: `ds_state::match`.
- `HAL_GetAllJoystickData`: all joystick axes/POVs/buttons arrays.

Missing or ambiguous before starting DS cycles:

- Out-of-range joystick index behavior for axes/buttons/POVs.
- Null output pointer behavior. Existing shim patterns assume WPILib-style
  unconditional write-through for status pointers; output pointer handling
  should be explicitly decided per function.
- Whether `HAL_GetAllJoystickData` belongs in the joystick-read cycle or a
  separate follow-up cycle.
- Whether descriptor/name/type/isXbox/axis-type getters are in scope now or
  intentionally deferred.
