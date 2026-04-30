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

#include <cstdint>

namespace robosim::backend::shim {

/**
 * Status codes mirroring WPILib HAL/Errors.h.
 *
 * The `kHal` prefix avoids macro-name collisions with the eventual WPILib
 * HAL_* macros.
 */
inline constexpr std::int32_t kHalSuccess     =  0;
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
  HAL_AllianceStationID_kRed1    = 1,
  HAL_AllianceStationID_kRed2    = 2,
  HAL_AllianceStationID_kRed3    = 3,
  HAL_AllianceStationID_kBlue1   = 4,
  HAL_AllianceStationID_kBlue2   = 5,
  HAL_AllianceStationID_kBlue3   = 6,
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
std::int32_t HAL_GetJoystickAxes(std::int32_t joystickNum,
                                 HAL_JoystickAxes* axes);

/** Mirrors WPILib HAL_GetJoystickPOVs from DriverStation.h. */
std::int32_t HAL_GetJoystickPOVs(std::int32_t joystickNum,
                                 HAL_JoystickPOVs* povs);

/** Mirrors WPILib HAL_GetJoystickButtons from DriverStation.h. */
std::int32_t HAL_GetJoystickButtons(std::int32_t joystickNum,
                                    HAL_JoystickButtons* buttons);

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

}  // extern "C"
