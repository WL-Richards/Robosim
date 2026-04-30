#include "hal_c.h"

#include "can_frame.h"
#include "ds_state.h"
#include "error_message.h"
#include "shim_core.h"
#include "truncate.h"

#include <cstring>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

static_assert(sizeof(HAL_CANStreamMessage) == sizeof(robosim::backend::can_frame));
static_assert(alignof(HAL_CANStreamMessage) == alignof(robosim::backend::can_frame));
static_assert(offsetof(HAL_CANStreamMessage, messageID) ==
              offsetof(robosim::backend::can_frame, message_id));
static_assert(offsetof(HAL_CANStreamMessage, timeStamp) ==
              offsetof(robosim::backend::can_frame, timestamp_us));
static_assert(offsetof(HAL_CANStreamMessage, data) ==
              offsetof(robosim::backend::can_frame, data));
static_assert(offsetof(HAL_CANStreamMessage, dataSize) ==
              offsetof(robosim::backend::can_frame, data_size));
static_assert(sizeof(HAL_ControlWord) == sizeof(robosim::backend::control_word));
static_assert(alignof(HAL_ControlWord) == alignof(robosim::backend::control_word));
static_assert(sizeof(HAL_AllianceStationID) == sizeof(robosim::backend::alliance_station));
static_assert(alignof(HAL_AllianceStationID) == alignof(robosim::backend::alliance_station));
static_assert(sizeof(HAL_JoystickAxes) == sizeof(robosim::backend::joystick_axes));
static_assert(alignof(HAL_JoystickAxes) == alignof(robosim::backend::joystick_axes));
static_assert(offsetof(HAL_JoystickAxes, count) ==
              offsetof(robosim::backend::joystick_axes, count));
static_assert(offsetof(HAL_JoystickAxes, axes) ==
              offsetof(robosim::backend::joystick_axes, axes));
static_assert(offsetof(HAL_JoystickAxes, raw) ==
              offsetof(robosim::backend::joystick_axes, raw));
static_assert(sizeof(HAL_JoystickPOVs) == sizeof(robosim::backend::joystick_povs));
static_assert(alignof(HAL_JoystickPOVs) == alignof(robosim::backend::joystick_povs));
static_assert(offsetof(HAL_JoystickPOVs, count) ==
              offsetof(robosim::backend::joystick_povs, count));
static_assert(offsetof(HAL_JoystickPOVs, povs) ==
              offsetof(robosim::backend::joystick_povs, povs));
static_assert(sizeof(HAL_JoystickButtons) == sizeof(robosim::backend::joystick_buttons));
static_assert(alignof(HAL_JoystickButtons) == alignof(robosim::backend::joystick_buttons));
static_assert(offsetof(HAL_JoystickButtons, buttons) ==
              offsetof(robosim::backend::joystick_buttons, buttons));
static_assert(offsetof(HAL_JoystickButtons, count) ==
              offsetof(robosim::backend::joystick_buttons, count));

namespace robosim::backend::shim {
namespace {

// Process-global shim accessor storage (D-C12-GLOBAL-ACCESSOR; cycle 12).
// Owned by the C HAL surface, not by any one shim instance — future
// HAL_Initialize / HAL_Shutdown / other HAL_* surfaces read the same
// static. v0 is single-threaded; the future threading cycle promotes
// to std::atomic<shim_core*>.
shim_core* g_installed_shim_ = nullptr;

// Cycle-17 helper: shared dispatch path for HAL_Bool readers on
// latest_clock_state_ (D-C17-CLOCK-STATE-HAL-BOOL-HELPER). The
// pointer-to-member parameter selects which hal_bool field is
// returned. Encapsulates the cycle-12 null-shim, empty-cache,
// status-write, and latest-wins contracts so the five HAL_Bool
// reader wrappers (HAL_GetBrownedOut + HAL_GetSystemActive +
// HAL_GetSystemTimeValid + HAL_GetFPGAButton + HAL_GetRSLState)
// inherit them for free without per-function code repetition.
HAL_Bool clock_state_hal_bool_read(std::int32_t* status,
                                   hal_bool clock_state::* field) {
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_clock_state();
  if (!cached.has_value()) {
    return 0;
  }
  return cached.value().*field;
}

std::string_view safe_view(const char* text) noexcept {
  return text != nullptr ? std::string_view{text} : std::string_view{};
}

bool valid_joystick_index(std::int32_t joystick_num) noexcept {
  return joystick_num >= 0 && joystick_num < static_cast<std::int32_t>(kMaxJoysticks);
}

template <typename HalT>
void zero_hal_struct(HalT* out) {
  std::memset(out, 0, sizeof(HalT));
}

template <typename HalT, typename BackendT>
void copy_hal_struct(HalT* out, const BackendT& in) {
  static_assert(sizeof(HalT) == sizeof(BackendT));
  std::memcpy(out, &in, sizeof(HalT));
}

}  // namespace

void shim_core::install_global(shim_core* shim) noexcept {
  g_installed_shim_ = shim;
}

shim_core* shim_core::current() noexcept {
  return g_installed_shim_;
}

}  // namespace robosim::backend::shim

// Cycle-17 file-scope using declarations: bridge the cycle-17 helper
// (lives in robosim::backend::shim::{anonymous}) and the clock_state
// type (lives in robosim::backend) into the extern "C" block's name-
// lookup scope so the five HAL_Bool wrapper bodies are genuine 1-line
// `return clock_state_hal_bool_read(status, &clock_state::field);`
// calls without per-wrapper using-decl noise. The using-declaration
// for an anonymous-namespace symbol is legal C++; the symbol retains
// internal linkage.
using robosim::backend::clock_state;
using robosim::backend::shim::clock_state_hal_bool_read;

extern "C" {

std::uint64_t HAL_GetFPGATime(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_clock_state();
  if (!cached.has_value()) {
    return 0;
  }
  return cached->sim_time_us;
}

double HAL_GetVinVoltage(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0.0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_power_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  // D-C13-FLOAT-TO-DOUBLE-CAST: widening is lossless for finite floats.
  return static_cast<double>(cached->vin_v);
}

double HAL_GetVinCurrent(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0.0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_power_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  // D-C13-FLOAT-TO-DOUBLE-CAST: widening is lossless for finite floats.
  return static_cast<double>(cached->vin_a);
}

double HAL_GetBrownoutVoltage(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0.0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_power_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  // D-C13-FLOAT-TO-DOUBLE-CAST: widening is lossless for finite floats.
  return static_cast<double>(cached->brownout_voltage_v);
}

HAL_Bool HAL_GetBrownedOut(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::browned_out);
}

HAL_Bool HAL_GetSystemActive(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::system_active);
}

HAL_Bool HAL_GetSystemTimeValid(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::system_time_valid);
}

HAL_Bool HAL_GetFPGAButton(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::fpga_button_latched);
}

HAL_Bool HAL_GetRSLState(std::int32_t* status) {
  return clock_state_hal_bool_read(status, &clock_state::rsl_state);
}

std::int32_t HAL_GetCommsDisableCount(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0;
  }
  *status = kHalSuccess;
  const auto& cached = shim->latest_clock_state();
  if (!cached.has_value()) {
    return 0;
  }
  // D-C18-UINT32-TO-INT32-CAST: schema field is uint32_t, WPILib's
  // HAL_GetCommsDisableCount returns int32_t. Identity for values
  // <= INT32_MAX; C++20 modular wraparound for values above.
  return static_cast<std::int32_t>(cached->comms_disable_count);
}

std::int32_t HAL_GetControlWord(HAL_ControlWord* controlWord) {
  using robosim::backend::kControlAutonomous;
  using robosim::backend::kControlDsAttached;
  using robosim::backend::kControlEStop;
  using robosim::backend::kControlEnabled;
  using robosim::backend::kControlFmsAttached;
  using robosim::backend::kControlTest;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  constexpr std::uint32_t kNamedControlBits =
      kControlEnabled | kControlAutonomous | kControlTest | kControlEStop |
      kControlFmsAttached | kControlDsAttached;
  std::uint32_t out_bits = 0;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    std::memcpy(controlWord, &out_bits, sizeof(out_bits));
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (cached.has_value()) {
    out_bits = cached->control.bits & kNamedControlBits;
  }
  std::memcpy(controlWord, &out_bits, sizeof(out_bits));
  return kHalSuccess;
}

HAL_AllianceStationID HAL_GetAllianceStation(std::int32_t* status) {
  using robosim::backend::alliance_station;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return HAL_AllianceStationID_kUnknown;
  }
  *status = kHalSuccess;

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return HAL_AllianceStationID_kUnknown;
  }
  return static_cast<HAL_AllianceStationID>(
      static_cast<std::int32_t>(cached->station));
}

double HAL_GetMatchTime(std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return 0.0;
  }
  *status = kHalSuccess;

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  return cached->match_time_seconds;
}

std::int32_t HAL_GetJoystickAxes(std::int32_t joystickNum,
                                 HAL_JoystickAxes* axes) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  zero_hal_struct(axes);
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value() || !valid_joystick_index(joystickNum)) {
    return kHalSuccess;
  }

  copy_hal_struct(axes, cached->joystick_axes_[static_cast<std::size_t>(joystickNum)]);
  return kHalSuccess;
}

std::int32_t HAL_GetJoystickPOVs(std::int32_t joystickNum,
                                 HAL_JoystickPOVs* povs) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  zero_hal_struct(povs);
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value() || !valid_joystick_index(joystickNum)) {
    return kHalSuccess;
  }

  copy_hal_struct(povs, cached->joystick_povs_[static_cast<std::size_t>(joystickNum)]);
  return kHalSuccess;
}

std::int32_t HAL_GetJoystickButtons(std::int32_t joystickNum,
                                    HAL_JoystickButtons* buttons) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  zero_hal_struct(buttons);
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value() || !valid_joystick_index(joystickNum)) {
    return kHalSuccess;
  }

  copy_hal_struct(buttons, cached->joystick_buttons_[static_cast<std::size_t>(joystickNum)]);
  return kHalSuccess;
}

std::int32_t HAL_SendError(HAL_Bool isError,
                           std::int32_t errorCode,
                           HAL_Bool isLVCode,
                           const char* details,
                           const char* location,
                           const char* callStack,
                           HAL_Bool printMsg) {
  using robosim::backend::error_message;
  using robosim::backend::kErrorTruncCallStack;
  using robosim::backend::kErrorTruncDetails;
  using robosim::backend::kErrorTruncLocation;
  using robosim::backend::copy_truncated;
  using robosim::backend::shim::safe_view;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  error_message msg{};
  msg.error_code = errorCode;
  msg.severity = isError;
  msg.is_lv_code = isLVCode;
  msg.print_msg = printMsg;

  if (copy_truncated(msg.details, safe_view(details))) {
    msg.truncation_flags |= kErrorTruncDetails;
  }
  if (copy_truncated(msg.location, safe_view(location))) {
    msg.truncation_flags |= kErrorTruncLocation;
  }
  if (copy_truncated(msg.call_stack, safe_view(callStack))) {
    msg.truncation_flags |= kErrorTruncCallStack;
  }

  shim->enqueue_error(msg);
  return 0;
}

void HAL_CAN_SendMessage(std::uint32_t messageID,
                         const std::uint8_t* data,
                         std::uint8_t dataSize,
                         std::int32_t periodMs,
                         std::int32_t* status) {
  using robosim::backend::can_frame;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalCanSendPeriodStopRepeating;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return;
  }
  if (dataSize > 8 || (data == nullptr && dataSize > 0) || periodMs < -1) {
    *status = kHalCanInvalidBuffer;
    return;
  }

  *status = kHalSuccess;
  if (periodMs == kHalCanSendPeriodStopRepeating) {
    return;
  }

  can_frame frame{};
  frame.message_id = messageID;
  frame.data_size = dataSize;
  if (data != nullptr && dataSize > 0) {
    std::memcpy(frame.data.data(), data, dataSize);
  }
  shim->enqueue_can_frame(frame);
}

void HAL_CAN_OpenStreamSession(std::uint32_t* sessionHandle,
                               std::uint32_t messageID,
                               std::uint32_t messageIDMask,
                               std::uint32_t maxMessages,
                               std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalCanNotAllowed;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *sessionHandle = 0;
    *status = kHalHandleError;
    return;
  }
  if (maxMessages == 0) {
    *sessionHandle = 0;
    *status = kHalCanInvalidBuffer;
    return;
  }

  const std::uint32_t handle =
      shim->open_can_stream_session(messageID, messageIDMask, maxMessages);
  *sessionHandle = handle;
  *status = handle == 0 ? kHalCanNotAllowed : kHalSuccess;
}

void HAL_CAN_CloseStreamSession(std::uint32_t sessionHandle) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->close_can_stream_session(sessionHandle);
}

void HAL_CAN_ReadStreamSession(std::uint32_t sessionHandle,
                               HAL_CANStreamMessage* messages,
                               std::uint32_t messagesToRead,
                               std::uint32_t* messagesRead,
                               std::int32_t* status) {
  using robosim::backend::can_frame;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalHandleError;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *messagesRead = 0;
    *status = kHalHandleError;
    return;
  }
  if (messages == nullptr && messagesToRead > 0) {
    *messagesRead = 0;
    *status = kHalCanInvalidBuffer;
    return;
  }

  std::vector<can_frame> frames(messagesToRead);
  std::uint32_t read_count = 0;
  const std::int32_t read_status =
      shim->read_can_stream_session(sessionHandle, std::span<can_frame>{frames}, read_count);

  for (std::uint32_t i = 0; i < read_count; ++i) {
    messages[i].messageID = frames[i].message_id;
    messages[i].timeStamp = frames[i].timestamp_us;
    std::memcpy(messages[i].data, frames[i].data.data(), frames[i].data.size());
    messages[i].dataSize = frames[i].data_size;
  }
  *messagesRead = read_count;
  *status = read_status;
}

void HAL_CAN_GetCANStatus(float* percentBusUtilization,
                          std::uint32_t* busOffCount,
                          std::uint32_t* txFullCount,
                          std::uint32_t* receiveErrorCount,
                          std::uint32_t* transmitErrorCount,
                          std::int32_t* status) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;

  *percentBusUtilization = 0.0f;
  *busOffCount = 0;
  *txFullCount = 0;
  *receiveErrorCount = 0;
  *transmitErrorCount = 0;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *status = kHalHandleError;
    return;
  }
  *status = kHalSuccess;

  const auto& cached = shim->latest_can_status();
  if (!cached.has_value()) {
    return;
  }

  *percentBusUtilization = cached->percent_bus_utilization;
  *busOffCount = cached->bus_off_count;
  *txFullCount = cached->tx_full_count;
  *receiveErrorCount = cached->receive_error_count;
  *transmitErrorCount = cached->transmit_error_count;
}

}  // extern "C"
