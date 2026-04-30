#pragma once

// Cycle 12 — first C HAL ABI surface. Declares the extern "C" symbols
// that the user's robot binary will eventually call (currently just
// HAL_GetFPGATime), plus the project-side status code constants that
// mirror WPILib HAL/Errors.h. The `extern "C"` block is intentionally
// minimal: every additional HAL_* function lands in its own TDD cycle.
//
// The translation unit hal_c.cpp owns the process-global
// `shim_core*` storage that shim_core::install_global / current touch;
// shim_core.cpp does NOT touch this storage. See
// `.claude/skills/hal-shim.md` and tests/backend/shim/TEST_PLAN_CYCLE12.md.

#include <cstddef>
#include <cstdint>

namespace robosim::backend::shim {

/**
 * Status codes mirroring WPILib HAL/Errors.h.
 *
 * The `kHal` prefix avoids macro-name collisions with the eventual WPILib
 * HAL_* macros.
 */
inline constexpr std::int32_t kHalSuccess = 0;
inline constexpr std::int32_t kHalHandleError = -1;
inline constexpr std::int32_t kHalCanInvalidBuffer = -44086;
inline constexpr std::int32_t kHalCanMessageNotFound = -44087;
inline constexpr std::int32_t kHalCanNoToken = 44087;
inline constexpr std::int32_t kHalCanNotAllowed = -44088;
inline constexpr std::int32_t kHalCanNotInitialized = -44089;
inline constexpr std::int32_t kHalCanSessionOverrun = 44050;
inline constexpr std::int32_t kHalCanSendPeriodStopRepeating = -1;

}  // namespace robosim::backend::shim

extern "C" {

/**
 * Mirrors WPILib HAL_Bool from hal/include/hal/Types.h.
 *
 * Placed inside the extern "C" block so C translation units including this
 * header see the same int32_t ABI alias.
 */
typedef std::int32_t HAL_Bool;

/** Mirrors WPILib WPI_Handle from wpi/Synchronization.h. */
typedef unsigned int WPI_Handle;

/** Mirrors WPILib WPI_EventHandle from wpi/Synchronization.h. */
typedef WPI_Handle WPI_EventHandle;

/** Mirrors WPILib HAL_NotifierHandle from hal/include/hal/Types.h. */
typedef std::int32_t HAL_NotifierHandle;

/**
 * Mirrors WPILib HAL_ControlWord from hal/include/hal/DriverStationTypes.h.
 *
 * The bitfield layout is pinned by common WPILib parity tests. The shim writes
 * it by copying the 32-bit packed protocol control_word mask into this object.
 */
struct HAL_ControlWord {
  std::uint32_t enabled : 1;
  std::uint32_t autonomous : 1;
  std::uint32_t test : 1;
  std::uint32_t eStop : 1;
  std::uint32_t fmsAttached : 1;
  std::uint32_t dsAttached : 1;
  std::uint32_t control_reserved : 26;
};

/** Mirrors WPILib HAL_AllianceStationID from DriverStationTypes.h. */
enum HAL_AllianceStationID : std::int32_t {
  HAL_AllianceStationID_kUnknown = 0,
  HAL_AllianceStationID_kRed1 = 1,
  HAL_AllianceStationID_kRed2 = 2,
  HAL_AllianceStationID_kRed3 = 3,
  HAL_AllianceStationID_kBlue1 = 4,
  HAL_AllianceStationID_kBlue2 = 5,
  HAL_AllianceStationID_kBlue3 = 6,
};

/** Mirrors WPILib HAL_MatchType from DriverStationTypes.h. */
enum HAL_MatchType : std::int32_t {
  HAL_kMatchType_none = 0,
  HAL_kMatchType_practice = 1,
  HAL_kMatchType_qualification = 2,
  HAL_kMatchType_elimination = 3,
};

/** Driver Station joystick limits mirrored from WPILib. */
inline constexpr std::int32_t HAL_kMaxJoystickAxes = 12;
inline constexpr std::int32_t HAL_kMaxJoystickPOVs = 12;
inline constexpr std::int32_t HAL_kMaxJoysticks = 6;

/** Mirrors WPILib HAL_JoystickAxes from DriverStationTypes.h. */
struct HAL_JoystickAxes {
  std::int16_t count;
  float axes[HAL_kMaxJoystickAxes];
  std::uint8_t raw[HAL_kMaxJoystickAxes];
};

/** Mirrors WPILib HAL_JoystickPOVs from DriverStationTypes.h. */
struct HAL_JoystickPOVs {
  std::int16_t count;
  std::int16_t povs[HAL_kMaxJoystickPOVs];
};

/** Mirrors WPILib HAL_JoystickButtons from DriverStationTypes.h. */
struct HAL_JoystickButtons {
  std::uint32_t buttons;
  std::uint8_t count;
};

/** Mirrors WPILib HAL_JoystickDescriptor from DriverStationTypes.h. */
struct HAL_JoystickDescriptor {
  std::uint8_t isXbox;
  std::uint8_t type;
  char name[256];
  std::uint8_t axisCount;
  std::uint8_t axisTypes[12];
  std::uint8_t buttonCount;
  std::uint8_t povCount;
};

/** Mirrors WPILib HAL_MatchInfo from DriverStationTypes.h. */
struct HAL_MatchInfo {
  char eventName[64];
  HAL_MatchType matchType;
  std::uint16_t matchNumber;
  std::uint8_t replayNumber;
  std::uint8_t gameSpecificMessage[64];
  std::uint16_t gameSpecificMessageSize;
};

/** Mirrors WPI_String from wpi/string.h. */
struct WPI_String {
  const char* str;
  std::size_t len;
};

/**
 * Mirrors WPILib HAL_CANStreamMessage from hal/include/hal/CAN.h.
 *
 * This is byte-compatible with robosim::backend::can_frame; the C ABI uses
 * WPILib's field spellings while the protocol schema uses project style.
 */
struct HAL_CANStreamMessage {
  std::uint32_t messageID;
  std::uint32_t timeStamp;
  std::uint8_t data[8];
  std::uint8_t dataSize;
};

/**
 * Mirrors WPILib HAL_Initialize from hal/include/hal/HALBase.h.
 *
 * v0 does not create or own the shim; the host must install one first via
 * shim_core::install_global. timeout and mode are accepted but ignored.
 */
HAL_Bool HAL_Initialize(std::int32_t timeout, std::int32_t mode);

/**
 * Mirrors WPILib HAL_Shutdown from hal/include/hal/HALBase.h.
 *
 * Wakes pending HAL waits and detaches the process-global shim pointer. The
 * caller-owned shim object is not destroyed.
 */
void HAL_Shutdown(void);

/**
 * Mirrors WPILib HAL_GetFPGATime from hal/include/hal/HALBase.h.
 *
 * Returns the latest cached clock_state::sim_time_us. If no shim is installed,
 * writes kHalHandleError and returns 0. If the shim is installed but no
 * clock_state has arrived, writes kHalSuccess and returns 0. `status` must not
 * be null, matching WPILib's unconditional write-through contract.
 */
std::uint64_t HAL_GetFPGATime(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetVinVoltage from hal/include/hal/HALBase.h.
 *
 * Returns latest power_state::vin_v widened to double. Status and empty-cache
 * semantics match HAL_GetFPGATime.
 */
double HAL_GetVinVoltage(std::int32_t* status);

/** Mirrors WPILib HAL_GetVinCurrent; returns latest power_state::vin_a. */
double HAL_GetVinCurrent(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetBrownoutVoltage; returns the brownout threshold, not
 * the live battery voltage.
 */
double HAL_GetBrownoutVoltage(std::int32_t* status);

/** Mirrors WPILib HAL_GetBrownedOut; returns latest clock_state::browned_out. */
HAL_Bool HAL_GetBrownedOut(std::int32_t* status);

/** Mirrors WPILib HAL_GetSystemActive; returns latest clock_state::system_active. */
HAL_Bool HAL_GetSystemActive(std::int32_t* status);

/** Mirrors WPILib HAL_GetSystemTimeValid; returns latest clock_state::system_time_valid. */
HAL_Bool HAL_GetSystemTimeValid(std::int32_t* status);

/** Mirrors WPILib HAL_GetFPGAButton; returns latest clock_state::fpga_button_latched. */
HAL_Bool HAL_GetFPGAButton(std::int32_t* status);

/** Mirrors WPILib HAL_GetRSLState; returns latest clock_state::rsl_state. */
HAL_Bool HAL_GetRSLState(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetCommsDisableCount.
 *
 * The schema stores uint32_t and WPILib returns int32_t; the shim performs the
 * pinned static_cast at the HAL seam.
 */
std::int32_t HAL_GetCommsDisableCount(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetControlWord from hal/include/hal/DriverStation.h.
 *
 * Copies the latest ds_state::control into the out-param and returns a HAL
 * status code directly. No installed shim returns kHalHandleError and zeroes
 * the word; an installed shim with an empty DS cache returns kHalSuccess and
 * zeroes the word. `controlWord` must not be null.
 */
std::int32_t HAL_GetControlWord(HAL_ControlWord* controlWord);

/**
 * Mirrors WPILib HAL_GetAllianceStation from DriverStation.h.
 *
 * Returns the latest ds_state::station or unknown for no-shim/empty-cache
 * paths. `status` must not be null.
 */
HAL_AllianceStationID HAL_GetAllianceStation(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetMatchTime from DriverStation.h.
 *
 * Returns the latest ds_state::match_time_seconds or 0.0 for no-shim/
 * empty-cache paths. `status` must not be null.
 */
double HAL_GetMatchTime(std::int32_t* status);

/**
 * Mirrors WPILib HAL_GetJoystickAxes from DriverStation.h.
 *
 * Copies one joystick axis slot from latest_ds_state_. No shim returns
 * kHalHandleError and zeroes the output; empty cache or invalid joystick index
 * returns kHalSuccess with zero/default output. `axes` must not be null.
 */
std::int32_t HAL_GetJoystickAxes(std::int32_t joystickNum, HAL_JoystickAxes* axes);

/** Mirrors WPILib HAL_GetJoystickPOVs from DriverStation.h. */
std::int32_t HAL_GetJoystickPOVs(std::int32_t joystickNum, HAL_JoystickPOVs* povs);

/** Mirrors WPILib HAL_GetJoystickButtons from DriverStation.h. */
std::int32_t HAL_GetJoystickButtons(std::int32_t joystickNum, HAL_JoystickButtons* buttons);

/**
 * Mirrors WPILib HAL_GetAllJoystickData from DriverStation.h.
 *
 * Copies all six axes/POV/button slots from latest_ds_state_. No shim or empty
 * cache zeroes all outputs. All output pointers must be non-null.
 */
void HAL_GetAllJoystickData(HAL_JoystickAxes* axes,
                            HAL_JoystickPOVs* povs,
                            HAL_JoystickButtons* buttons);

/**
 * Mirrors WPILib HAL_GetJoystickDescriptor from DriverStation.h.
 *
 * Copies one joystick descriptor from latest_ds_state_. No shim returns
 * kHalHandleError and zeroes the descriptor; empty cache or invalid joystick
 * index returns kHalSuccess with zero/default output.
 */
std::int32_t HAL_GetJoystickDescriptor(std::int32_t joystickNum, HAL_JoystickDescriptor* desc);

/** Mirrors WPILib HAL_GetJoystickIsXbox from DriverStation.h. */
HAL_Bool HAL_GetJoystickIsXbox(std::int32_t joystickNum);

/** Mirrors WPILib HAL_GetJoystickType from DriverStation.h. */
std::int32_t HAL_GetJoystickType(std::int32_t joystickNum);

/**
 * Mirrors WPILib HAL_GetJoystickName from DriverStation.h.
 *
 * Fills a WPI_String with heap-owned bytes copied from the descriptor name.
 * Empty/default names return {nullptr, 0}. Nonempty strings must be freed by
 * the caller with the same allocator family as WPI_FreeString/std::free.
 */
void HAL_GetJoystickName(WPI_String* name, std::int32_t joystickNum);

/** Mirrors WPILib HAL_GetJoystickAxisType from DriverStation.h. */
std::int32_t HAL_GetJoystickAxisType(std::int32_t joystickNum, std::int32_t axis);

/**
 * Mirrors WPILib HAL_GetMatchInfo from DriverStation.h.
 *
 * Copies latest_ds_state_::match. No shim returns kHalHandleError and zeroes
 * the output; empty cache returns kHalSuccess with zero/default output.
 */
std::int32_t HAL_GetMatchInfo(HAL_MatchInfo* info);

/**
 * Mirrors WPILib HAL_RefreshDSData from DriverStation.h.
 *
 * Drains one inbound protocol message and returns true only when that message
 * was a Driver Station state update. Other valid messages still dispatch but
 * return false.
 */
HAL_Bool HAL_RefreshDSData(void);

/** Mirrors WPILib HAL_ProvideNewDataEventHandle from DriverStation.h. */
void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle);

/** Mirrors WPILib HAL_RemoveNewDataEventHandle from DriverStation.h. */
void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle);

/**
 * Mirrors WPILib HAL_GetOutputsEnabled from DriverStation.h.
 *
 * Returns true only when the latest cached Driver Station control word has
 * both enabled and DS-attached bits set. No shim or empty cache returns false.
 */
HAL_Bool HAL_GetOutputsEnabled(void);

/** Mirrors WPILib HAL_ObserveUserProgramStarting from DriverStation.h. */
void HAL_ObserveUserProgramStarting(void);

/** Mirrors WPILib HAL_ObserveUserProgramDisabled from DriverStation.h. */
void HAL_ObserveUserProgramDisabled(void);

/** Mirrors WPILib HAL_ObserveUserProgramAutonomous from DriverStation.h. */
void HAL_ObserveUserProgramAutonomous(void);

/** Mirrors WPILib HAL_ObserveUserProgramTeleop from DriverStation.h. */
void HAL_ObserveUserProgramTeleop(void);

/** Mirrors WPILib HAL_ObserveUserProgramTest from DriverStation.h. */
void HAL_ObserveUserProgramTest(void);

/**
 * Mirrors WPILib HAL_SetJoystickOutputs from DriverStation.h.
 *
 * Stores the latest output bitmask and rumble values for one joystick slot.
 * No installed shim or invalid joystick index returns kHalHandleError.
 */
std::int32_t HAL_SetJoystickOutputs(std::int32_t joystickNum,
                                    std::int64_t outputs,
                                    std::int32_t leftRumble,
                                    std::int32_t rightRumble);

/**
 * Mirrors WPILib HAL_SendError from hal/include/hal/DriverStation.h.
 *
 * Translates the C ABI arguments into error_message and buffers them on the
 * installed shim. Returns 0 on success, or kHalHandleError when no shim is
 * installed. Null string pointers are treated as empty strings.
 */
std::int32_t HAL_SendError(HAL_Bool isError,
                           std::int32_t errorCode,
                           HAL_Bool isLVCode,
                           const char* details,
                           const char* location,
                           const char* callStack,
                           HAL_Bool printMsg);

/**
 * Mirrors WPILib HAL_CAN_SendMessage from hal/include/hal/CAN.h.
 *
 * Translates one C HAL CAN TX call into a pending can_frame on the installed
 * shim. `dataSize` must be 0..8. `data == nullptr` is valid only when
 * dataSize == 0. Positive periodMs enqueues one immediate frame in v0;
 * repeat scheduling is deferred. HAL_CAN_SEND_PERIOD_STOP_REPEATING succeeds
 * without enqueueing a data frame.
 */
void HAL_CAN_SendMessage(std::uint32_t messageID,
                         const std::uint8_t* data,
                         std::uint8_t dataSize,
                         std::int32_t periodMs,
                         std::int32_t* status);

/**
 * Mirrors WPILib HAL_CAN_ReceiveMessage from hal/include/hal/CAN.h.
 *
 * Reads the first matching active frame from latest_can_frame_batch_. The
 * messageID pointer is in/out: callers provide the requested ID and receive
 * the actual matched frame ID on success. No shim reports kHalHandleError;
 * empty/no-match reports kHalCanMessageNotFound; null data reports
 * kHalCanInvalidBuffer when a shim is installed.
 */
void HAL_CAN_ReceiveMessage(std::uint32_t* messageID,
                            std::uint32_t messageIDMask,
                            std::uint8_t* data,
                            std::uint8_t* dataSize,
                            std::uint32_t* timeStamp,
                            std::int32_t* status);

/** Mirrors WPILib HAL_CAN_OpenStreamSession from hal/include/hal/CAN.h. */
void HAL_CAN_OpenStreamSession(std::uint32_t* sessionHandle,
                               std::uint32_t messageID,
                               std::uint32_t messageIDMask,
                               std::uint32_t maxMessages,
                               std::int32_t* status);

/** Mirrors WPILib HAL_CAN_CloseStreamSession from hal/include/hal/CAN.h. */
void HAL_CAN_CloseStreamSession(std::uint32_t sessionHandle);

/** Mirrors WPILib HAL_CAN_ReadStreamSession from hal/include/hal/CAN.h. */
void HAL_CAN_ReadStreamSession(std::uint32_t sessionHandle,
                               HAL_CANStreamMessage* messages,
                               std::uint32_t messagesToRead,
                               std::uint32_t* messagesRead,
                               std::int32_t* status);

/** Mirrors WPILib HAL_CAN_GetCANStatus from hal/include/hal/CAN.h. */
void HAL_CAN_GetCANStatus(float* percentBusUtilization,
                          std::uint32_t* busOffCount,
                          std::uint32_t* txFullCount,
                          std::uint32_t* receiveErrorCount,
                          std::uint32_t* transmitErrorCount,
                          std::int32_t* status);

/**
 * Mirrors WPILib HAL_InitializeNotifier from hal/include/hal/Notifier.h.
 *
 * Returns a nonzero handle on success. No installed shim or a full notifier
 * table writes kHalHandleError and returns 0.
 */
HAL_NotifierHandle HAL_InitializeNotifier(std::int32_t* status);

/** Mirrors WPILib HAL_SetNotifierName; null names are treated as empty. */
void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle, const char* name, std::int32_t* status);

/**
 * Mirrors WPILib HAL_SetNotifierThreadPriority.
 *
 * v0 has no scheduler model, so an installed shim accepts all inputs as a
 * deterministic no-op success. No installed shim writes kHalHandleError and
 * returns false.
 */
HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime,
                                       std::int32_t priority,
                                       std::int32_t* status);

/** Mirrors WPILib HAL_StopNotifier for the cycle-25 control-plane state. */
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, std::int32_t* status);

/** Mirrors WPILib HAL_CleanNotifier; invalid/no-shim handles are no-ops. */
void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle);

/** Mirrors WPILib HAL_UpdateNotifierAlarm. */
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             std::uint64_t triggerTime,
                             std::int32_t* status);

/** Mirrors WPILib HAL_CancelNotifierAlarm. */
void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle, std::int32_t* status);

/**
 * Mirrors WPILib HAL_WaitForNotifierAlarm.
 *
 * Waits until a matching inbound notifier_alarm event is available, the
 * notifier is stopped, or the handle is cleaned. Stop wakes return 0 with
 * success; invalid/cleaned handles return 0 with kHalHandleError.
 */
std::uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle, std::int32_t* status);

}  // extern "C"
