#include "hal_c.h"

#include "can_frame.h"
#include "ds_state.h"
#include "error_message.h"
#include "shim_core.h"
#include "truncate.h"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

static_assert(sizeof(HAL_CANStreamMessage) == sizeof(robosim::backend::can_frame));
static_assert(alignof(HAL_CANStreamMessage) == alignof(robosim::backend::can_frame));
static_assert(offsetof(HAL_CANStreamMessage, messageID) ==
              offsetof(robosim::backend::can_frame, message_id));
static_assert(offsetof(HAL_CANStreamMessage, timeStamp) ==
              offsetof(robosim::backend::can_frame, timestamp_us));
static_assert(offsetof(HAL_CANStreamMessage, data) == offsetof(robosim::backend::can_frame, data));
static_assert(offsetof(HAL_CANStreamMessage, dataSize) ==
              offsetof(robosim::backend::can_frame, data_size));
static_assert(sizeof(HAL_ControlWord) == sizeof(robosim::backend::control_word));
static_assert(alignof(HAL_ControlWord) == alignof(robosim::backend::control_word));
static_assert(sizeof(HAL_AllianceStationID) == sizeof(robosim::backend::alliance_station));
static_assert(alignof(HAL_AllianceStationID) == alignof(robosim::backend::alliance_station));
static_assert(sizeof(HAL_MatchType) == sizeof(robosim::backend::match_type));
static_assert(alignof(HAL_MatchType) == alignof(robosim::backend::match_type));
static_assert(sizeof(HAL_JoystickAxes) == sizeof(robosim::backend::joystick_axes));
static_assert(alignof(HAL_JoystickAxes) == alignof(robosim::backend::joystick_axes));
static_assert(offsetof(HAL_JoystickAxes, count) ==
              offsetof(robosim::backend::joystick_axes, count));
static_assert(offsetof(HAL_JoystickAxes, axes) == offsetof(robosim::backend::joystick_axes, axes));
static_assert(offsetof(HAL_JoystickAxes, raw) == offsetof(robosim::backend::joystick_axes, raw));
static_assert(sizeof(HAL_JoystickPOVs) == sizeof(robosim::backend::joystick_povs));
static_assert(alignof(HAL_JoystickPOVs) == alignof(robosim::backend::joystick_povs));
static_assert(offsetof(HAL_JoystickPOVs, count) ==
              offsetof(robosim::backend::joystick_povs, count));
static_assert(offsetof(HAL_JoystickPOVs, povs) == offsetof(robosim::backend::joystick_povs, povs));
static_assert(sizeof(HAL_JoystickButtons) == sizeof(robosim::backend::joystick_buttons));
static_assert(alignof(HAL_JoystickButtons) == alignof(robosim::backend::joystick_buttons));
static_assert(offsetof(HAL_JoystickButtons, buttons) ==
              offsetof(robosim::backend::joystick_buttons, buttons));
static_assert(offsetof(HAL_JoystickButtons, count) ==
              offsetof(robosim::backend::joystick_buttons, count));
static_assert(sizeof(HAL_JoystickDescriptor) == sizeof(robosim::backend::joystick_descriptor));
static_assert(alignof(HAL_JoystickDescriptor) == alignof(robosim::backend::joystick_descriptor));
static_assert(offsetof(HAL_JoystickDescriptor, isXbox) ==
              offsetof(robosim::backend::joystick_descriptor, is_xbox));
static_assert(offsetof(HAL_JoystickDescriptor, type) ==
              offsetof(robosim::backend::joystick_descriptor, type));
static_assert(offsetof(HAL_JoystickDescriptor, name) ==
              offsetof(robosim::backend::joystick_descriptor, name));
static_assert(offsetof(HAL_JoystickDescriptor, axisCount) ==
              offsetof(robosim::backend::joystick_descriptor, axis_count));
static_assert(offsetof(HAL_JoystickDescriptor, axisTypes) ==
              offsetof(robosim::backend::joystick_descriptor, axis_types));
static_assert(offsetof(HAL_JoystickDescriptor, buttonCount) ==
              offsetof(robosim::backend::joystick_descriptor, button_count));
static_assert(offsetof(HAL_JoystickDescriptor, povCount) ==
              offsetof(robosim::backend::joystick_descriptor, pov_count));
static_assert(sizeof(HAL_MatchInfo) == sizeof(robosim::backend::match_info));
static_assert(alignof(HAL_MatchInfo) == alignof(robosim::backend::match_info));
static_assert(offsetof(HAL_MatchInfo, eventName) ==
              offsetof(robosim::backend::match_info, event_name));
static_assert(offsetof(HAL_MatchInfo, matchType) == offsetof(robosim::backend::match_info, type));
static_assert(offsetof(HAL_MatchInfo, matchNumber) ==
              offsetof(robosim::backend::match_info, match_number));
static_assert(offsetof(HAL_MatchInfo, replayNumber) ==
              offsetof(robosim::backend::match_info, replay_number));
static_assert(offsetof(HAL_MatchInfo, gameSpecificMessage) ==
              offsetof(robosim::backend::match_info, game_specific_message));
static_assert(offsetof(HAL_MatchInfo, gameSpecificMessageSize) ==
              offsetof(robosim::backend::match_info, game_specific_message_size));
static_assert(offsetof(WPI_String, str) == 0);
static_assert(offsetof(WPI_String, len) == sizeof(const char*));
static_assert(sizeof(HAL_RuntimeType) == sizeof(std::int32_t));
static_assert(static_cast<std::int32_t>(HAL_Runtime_RoboRIO) == 0);
static_assert(static_cast<std::int32_t>(HAL_Runtime_RoboRIO2) == 1);
static_assert(static_cast<std::int32_t>(HAL_Runtime_Simulation) == 2);

namespace robosim::backend::shim {
namespace {

// Process-global shim accessor storage (D-C12-GLOBAL-ACCESSOR; cycle 12).
// Owned by the C HAL surface, not by any one shim instance — future
// HAL_Initialize / HAL_Shutdown / other HAL_* surfaces read the same
// static. v0 is single-threaded; the future threading cycle promotes
// to std::atomic<shim_core*>.
shim_core* g_installed_shim_ = nullptr;

inline constexpr std::int32_t kFpgaVersion = 2026;
inline constexpr std::int64_t kFpgaRevision = 0;

thread_local std::int32_t g_last_error_status_ = kHalSuccess;

std::string g_comments_machine_info_path_override_;
bool g_comments_cache_initialized_ = false;
std::string g_comments_cache_;

void write_status(std::int32_t* status, std::int32_t value) noexcept {
  *status = value;
  if (value != kHalSuccess) {
    g_last_error_status_ = value;
  }
}

const char* error_message_for_status(std::int32_t code) noexcept {
  switch (code) {
    case kHalSuccess:
      return "HAL: Success";
    case kHalHandleError:
      return "HAL: A handle parameter was passed incorrectly";
    case kHalUseLastError:
      return "HAL: Use HAL_GetLastError(status) to get last error";
    case kHalCanInvalidBuffer:
      return "CAN: Invalid Buffer";
    case kHalCanMessageNotFound:
      return "CAN: Message not found";
    case kHalCanNoToken:
      return "CAN: No token";
    case kHalCanNotAllowed:
      return "CAN: Not allowed";
    case kHalCanNotInitialized:
      return "CAN: Not initialized";
    case kHalCanSessionOverrun:
      return "CAN: Session overrun";
    default:
      return "Unknown error status";
  }
}

std::string_view comments_machine_info_path() noexcept {
  if (!g_comments_machine_info_path_override_.empty()) {
    return g_comments_machine_info_path_override_;
  }
  return "/etc/machine-info";
}

std::string read_all_text(std::string_view path) {
  std::ifstream in{std::string{path}, std::ios::binary};
  if (!in.good()) {
    return {};
  }

  std::string contents;
  char ch = '\0';
  while (in.get(ch)) {
    contents.push_back(ch);
  }
  return contents;
}

std::string parse_pretty_hostname(std::string_view contents) {
  constexpr std::string_view kPrefix = "PRETTY_HOSTNAME=\"";
  constexpr std::size_t kMaxCommentsBytes = 64;

  const std::size_t start = contents.find(kPrefix);
  if (start == std::string_view::npos) {
    return {};
  }

  std::string out;
  std::size_t i = start + kPrefix.size();
  while (i < contents.size()) {
    const char ch = contents[i++];
    if (ch == '"') {
      return out;
    }
    if (ch != '\\') {
      if (out.size() < kMaxCommentsBytes) {
        out.push_back(ch);
      }
      continue;
    }

    if (i >= contents.size()) {
      return {};
    }
    const char escaped = contents[i++];
    char decoded = escaped;
    switch (escaped) {
      case '"':
      case '\\':
        decoded = escaped;
        break;
      case 'n':
        decoded = '\n';
        break;
      case 't':
        decoded = '\t';
        break;
      case 'r':
        decoded = '\r';
        break;
      default:
        decoded = escaped;
        break;
    }
    if (out.size() < kMaxCommentsBytes) {
      out.push_back(decoded);
    }
  }

  return {};
}

void initialize_comments_cache() {
  if (g_comments_cache_initialized_) {
    return;
  }
  g_comments_cache_ = parse_pretty_hostname(read_all_text(comments_machine_info_path()));
  g_comments_cache_initialized_ = true;
}

// Cycle-17 helper: shared dispatch path for HAL_Bool readers on
// latest_clock_state_ (D-C17-CLOCK-STATE-HAL-BOOL-HELPER). The
// pointer-to-member parameter selects which hal_bool field is
// returned. Encapsulates the cycle-12 null-shim, empty-cache,
// status-write, and latest-wins contracts so the five HAL_Bool
// reader wrappers (HAL_GetBrownedOut + HAL_GetSystemActive +
// HAL_GetSystemTimeValid + HAL_GetFPGAButton + HAL_GetRSLState)
// inherit them for free without per-function code repetition.
HAL_Bool clock_state_hal_bool_read(std::int32_t* status, hal_bool clock_state::*field) {
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  const auto cached = shim->latest_clock_state_snapshot();
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

bool valid_joystick_axis_index(std::int32_t axis) noexcept {
  return axis >= 0 && axis < static_cast<std::int32_t>(kJoystickAxisTypesLen);
}

bool valid_port_arg(std::int32_t value) noexcept {
  return value >= 0 && value < 255;
}

HAL_PortHandle make_port_handle(std::uint8_t channel, std::uint8_t module) noexcept {
  constexpr std::int32_t kPortHandleType = 2;
  return static_cast<HAL_PortHandle>((kPortHandleType << 24) |
                                     (static_cast<std::int32_t>(module) << 8) |
                                     static_cast<std::int32_t>(channel));
}

template <typename HalT>
void zero_hal_struct(HalT* out) {
  std::memset(out, 0, sizeof(HalT));
}

template <typename HalT>
void zero_hal_array(HalT* out, std::size_t count) {
  std::memset(out, 0, sizeof(HalT) * count);
}

template <typename HalT, typename BackendT>
void copy_hal_struct(HalT* out, const BackendT& in) {
  static_assert(sizeof(HalT) == sizeof(BackendT));
  std::memcpy(out, &in, sizeof(HalT));
}

const ds_state* latest_ds_state_or_null() {
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return nullptr;
  }
  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return nullptr;
  }
  return &*cached;
}

const joystick_descriptor* descriptor_or_null(std::int32_t joystick_num) {
  const ds_state* state = latest_ds_state_or_null();
  if (state == nullptr || !valid_joystick_index(joystick_num)) {
    return nullptr;
  }
  return &state->joystick_descriptors[static_cast<std::size_t>(joystick_num)];
}

std::size_t fixed_name_length(const std::array<char, kJoystickNameLen>& name) noexcept {
  for (std::size_t i = 0; i < name.size(); ++i) {
    if (name[i] == '\0') {
      return i;
    }
  }
  return name.size();
}

void assign_wpi_string(WPI_String* out, std::span<const char> bytes) {
  out->str = nullptr;
  out->len = 0;
  if (bytes.empty()) {
    return;
  }

  void* allocation = std::malloc(bytes.size());
  if (allocation == nullptr) {
    return;
  }
  std::memcpy(allocation, bytes.data(), bytes.size());
  out->str = static_cast<const char*>(allocation);
  out->len = bytes.size();
}

}  // namespace

void shim_core::install_global(shim_core* shim) noexcept {
  g_installed_shim_ = shim;
}

shim_core* shim_core::current() noexcept {
  return g_installed_shim_;
}

void set_hal_comments_machine_info_path_for_test(std::string_view path) {
  g_comments_machine_info_path_override_ = path;
}

void clear_hal_comments_machine_info_path_for_test() {
  g_comments_machine_info_path_override_.clear();
}

void reset_hal_comments_cache_for_test() {
  g_comments_cache_initialized_ = false;
  g_comments_cache_.clear();
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
using robosim::backend::shim::write_status;

extern "C" {

HAL_Bool HAL_Initialize(std::int32_t timeout, std::int32_t mode) {
  using robosim::backend::shim::shim_core;

  (void)timeout;
  (void)mode;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return 0;
  }
  shim->prepare_for_hal_initialize();
  return 1;
}

void HAL_Shutdown(void) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->shutdown_hal_waits();
  shim_core::install_global(nullptr);
}

const char* HAL_GetLastError(std::int32_t* status) {
  using robosim::backend::shim::error_message_for_status;
  using robosim::backend::shim::g_last_error_status_;
  using robosim::backend::shim::kHalUseLastError;

  std::int32_t code = *status;
  if (code == kHalUseLastError) {
    code = g_last_error_status_;
    *status = code;
  }
  return error_message_for_status(code);
}

const char* HAL_GetErrorMessage(std::int32_t code) {
  return robosim::backend::shim::error_message_for_status(code);
}

HAL_RuntimeType HAL_GetRuntimeType(void) {
  return HAL_Runtime_RoboRIO2;
}

std::int32_t HAL_GetTeamNumber(void) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return 0;
  }
  return shim->boot_descriptor_snapshot().team_number;
}

std::int32_t HAL_GetFPGAVersion(std::int32_t* status) {
  using robosim::backend::shim::kFpgaVersion;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  return kFpgaVersion;
}

std::int64_t HAL_GetFPGARevision(std::int32_t* status) {
  using robosim::backend::shim::kFpgaRevision;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  return kFpgaRevision;
}

void HAL_GetSerialNumber(WPI_String* serialNumber) {
  using robosim::backend::shim::assign_wpi_string;
  using robosim::backend::shim::safe_view;

  assign_wpi_string(serialNumber, safe_view(std::getenv("serialnum")));
}

void HAL_GetComments(WPI_String* comments) {
  using robosim::backend::shim::assign_wpi_string;
  using robosim::backend::shim::g_comments_cache_;
  using robosim::backend::shim::initialize_comments_cache;

  initialize_comments_cache();
  assign_wpi_string(comments, std::span<const char>{g_comments_cache_.data(),
                                                    g_comments_cache_.size()});
}

HAL_PortHandle HAL_GetPort(std::int32_t channel) {
  return HAL_GetPortWithModule(1, channel);
}

HAL_PortHandle HAL_GetPortWithModule(std::int32_t module, std::int32_t channel) {
  using robosim::backend::shim::make_port_handle;
  using robosim::backend::shim::valid_port_arg;

  if (!valid_port_arg(module) || !valid_port_arg(channel)) {
    return 0;
  }
  return make_port_handle(static_cast<std::uint8_t>(channel),
                          static_cast<std::uint8_t>(module));
}

std::int64_t HAL_Report(std::int32_t resource,
                        std::int32_t instanceNumber,
                        std::int32_t context,
                        const char* feature) {
  using robosim::backend::shim::safe_view;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return 0;
  }
  return shim->record_usage_report(resource, instanceNumber, context, safe_view(feature));
}

std::uint64_t HAL_GetFPGATime(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  const auto& cached = shim->latest_clock_state();
  if (!cached.has_value()) {
    return 0;
  }
  return cached->sim_time_us;
}

double HAL_GetVinVoltage(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0.0;
  }
  write_status(status, kHalSuccess);
  const auto& cached = shim->latest_power_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  // D-C13-FLOAT-TO-DOUBLE-CAST: widening is lossless for finite floats.
  return static_cast<double>(cached->vin_v);
}

double HAL_GetVinCurrent(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0.0;
  }
  write_status(status, kHalSuccess);
  const auto& cached = shim->latest_power_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  // D-C13-FLOAT-TO-DOUBLE-CAST: widening is lossless for finite floats.
  return static_cast<double>(cached->vin_a);
}

double HAL_GetBrownoutVoltage(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0.0;
  }
  write_status(status, kHalSuccess);
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
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
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
  using robosim::backend::kControlEnabled;
  using robosim::backend::kControlEStop;
  using robosim::backend::kControlFmsAttached;
  using robosim::backend::kControlTest;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  constexpr std::uint32_t kNamedControlBits = kControlEnabled | kControlAutonomous | kControlTest |
                                              kControlEStop | kControlFmsAttached |
                                              kControlDsAttached;
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
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return HAL_AllianceStationID_kUnknown;
  }
  write_status(status, kHalSuccess);

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return HAL_AllianceStationID_kUnknown;
  }
  return static_cast<HAL_AllianceStationID>(static_cast<std::int32_t>(cached->station));
}

double HAL_GetMatchTime(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0.0;
  }
  write_status(status, kHalSuccess);

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return 0.0;
  }
  return cached->match_time_seconds;
}

std::int32_t HAL_GetJoystickAxes(std::int32_t joystickNum, HAL_JoystickAxes* axes) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;

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

std::int32_t HAL_GetJoystickPOVs(std::int32_t joystickNum, HAL_JoystickPOVs* povs) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;

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

std::int32_t HAL_GetJoystickButtons(std::int32_t joystickNum, HAL_JoystickButtons* buttons) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;

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

void HAL_GetAllJoystickData(HAL_JoystickAxes* axes,
                            HAL_JoystickPOVs* povs,
                            HAL_JoystickButtons* buttons) {
  using robosim::backend::kMaxJoysticks;
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::zero_hal_array;

  zero_hal_array(axes, kMaxJoysticks);
  zero_hal_array(povs, kMaxJoysticks);
  zero_hal_array(buttons, kMaxJoysticks);

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return;
  }

  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    copy_hal_struct(&axes[i], cached->joystick_axes_[i]);
    copy_hal_struct(&povs[i], cached->joystick_povs_[i]);
    copy_hal_struct(&buttons[i], cached->joystick_buttons_[i]);
  }
}

std::int32_t HAL_GetJoystickDescriptor(std::int32_t joystickNum, HAL_JoystickDescriptor* desc) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::valid_joystick_index;
  using robosim::backend::shim::zero_hal_struct;

  zero_hal_struct(desc);
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value() || !valid_joystick_index(joystickNum)) {
    return kHalSuccess;
  }

  copy_hal_struct(desc, cached->joystick_descriptors[static_cast<std::size_t>(joystickNum)]);
  return kHalSuccess;
}

HAL_Bool HAL_GetJoystickIsXbox(std::int32_t joystickNum) {
  using robosim::backend::shim::descriptor_or_null;

  const auto* descriptor = descriptor_or_null(joystickNum);
  return descriptor == nullptr ? 0 : static_cast<HAL_Bool>(descriptor->is_xbox);
}

std::int32_t HAL_GetJoystickType(std::int32_t joystickNum) {
  using robosim::backend::shim::descriptor_or_null;

  const auto* descriptor = descriptor_or_null(joystickNum);
  return descriptor == nullptr ? 0 : static_cast<std::int32_t>(descriptor->type);
}

void HAL_GetJoystickName(WPI_String* name, std::int32_t joystickNum) {
  using robosim::backend::shim::assign_wpi_string;
  using robosim::backend::shim::descriptor_or_null;
  using robosim::backend::shim::fixed_name_length;

  const auto* descriptor = descriptor_or_null(joystickNum);
  if (descriptor == nullptr) {
    assign_wpi_string(name, {});
    return;
  }

  const std::size_t length = fixed_name_length(descriptor->name);
  assign_wpi_string(name, std::span<const char>{descriptor->name.data(), length});
}

std::int32_t HAL_GetJoystickAxisType(std::int32_t joystickNum, std::int32_t axis) {
  using robosim::backend::shim::descriptor_or_null;
  using robosim::backend::shim::valid_joystick_axis_index;

  const auto* descriptor = descriptor_or_null(joystickNum);
  if (descriptor == nullptr || !valid_joystick_axis_index(axis)) {
    return 0;
  }
  return static_cast<std::int32_t>(descriptor->axis_types[static_cast<std::size_t>(axis)]);
}

std::int32_t HAL_GetMatchInfo(HAL_MatchInfo* info) {
  using robosim::backend::shim::copy_hal_struct;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::zero_hal_struct;

  zero_hal_struct(info);
  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return kHalSuccess;
  }

  copy_hal_struct(info, cached->match);
  return kHalSuccess;
}

HAL_Bool HAL_RefreshDSData(void) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return 0;
  }
  return shim->refresh_ds_data() ? 1 : 0;
}

void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->provide_new_data_event_handle(handle);
}

void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->remove_new_data_event_handle(handle);
}

HAL_Bool HAL_GetOutputsEnabled(void) {
  using robosim::backend::kControlDsAttached;
  using robosim::backend::kControlEnabled;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return 0;
  }

  const auto& cached = shim->latest_ds_state();
  if (!cached.has_value()) {
    return 0;
  }

  const std::uint32_t bits = cached->control.bits;
  return ((bits & kControlEnabled) != 0 && (bits & kControlDsAttached) != 0) ? 1 : 0;
}

void HAL_ObserveUserProgramStarting(void) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::user_program_observer_state;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->observe_user_program(user_program_observer_state::starting);
}

void HAL_ObserveUserProgramDisabled(void) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::user_program_observer_state;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->observe_user_program(user_program_observer_state::disabled);
}

void HAL_ObserveUserProgramAutonomous(void) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::user_program_observer_state;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->observe_user_program(user_program_observer_state::autonomous);
}

void HAL_ObserveUserProgramTeleop(void) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::user_program_observer_state;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->observe_user_program(user_program_observer_state::teleop);
}

void HAL_ObserveUserProgramTest(void) {
  using robosim::backend::shim::shim_core;
  using robosim::backend::shim::user_program_observer_state;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->observe_user_program(user_program_observer_state::test);
}

std::int32_t HAL_SetJoystickOutputs(std::int32_t joystickNum,
                                    std::int64_t outputs,
                                    std::int32_t leftRumble,
                                    std::int32_t rightRumble) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return kHalHandleError;
  }
  return shim->set_joystick_outputs(joystickNum, outputs, leftRumble, rightRumble);
}

std::int32_t HAL_SendError(HAL_Bool isError,
                           std::int32_t errorCode,
                           HAL_Bool isLVCode,
                           const char* details,
                           const char* location,
                           const char* callStack,
                           HAL_Bool printMsg) {
  using robosim::backend::copy_truncated;
  using robosim::backend::error_message;
  using robosim::backend::kErrorTruncCallStack;
  using robosim::backend::kErrorTruncDetails;
  using robosim::backend::kErrorTruncLocation;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::safe_view;
  using robosim::backend::shim::shim_core;

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
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalCanSendPeriodStopRepeating;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  if (dataSize > 8 || (data == nullptr && dataSize > 0) || periodMs < -1) {
    write_status(status, kHalCanInvalidBuffer);
    return;
  }

  write_status(status, kHalSuccess);
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

void HAL_CAN_ReceiveMessage(std::uint32_t* messageID,
                            std::uint32_t messageIDMask,
                            std::uint8_t* data,
                            std::uint8_t* dataSize,
                            std::uint32_t* timeStamp,
                            std::int32_t* status) {
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalCanMessageNotFound;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  const auto zero_scalars = [&] {
    *messageID = 0;
    *dataSize = 0;
    *timeStamp = 0;
  };
  const auto zero_outputs = [&] {
    zero_scalars();
    if (data != nullptr) {
      std::memset(data, 0, 8);
    }
  };

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    zero_outputs();
    write_status(status, kHalHandleError);
    return;
  }
  if (data == nullptr) {
    zero_scalars();
    write_status(status, kHalCanInvalidBuffer);
    return;
  }

  const std::uint32_t requested_id = *messageID;
  const auto& cached = shim->latest_can_frame_batch();
  if (!cached.has_value()) {
    zero_outputs();
    write_status(status, kHalCanMessageNotFound);
    return;
  }

  for (std::uint32_t i = 0; i < cached->count; ++i) {
    const auto& frame = cached->frames[i];
    if ((frame.message_id & messageIDMask) != (requested_id & messageIDMask)) {
      continue;
    }
    *messageID = frame.message_id;
    std::memcpy(data, frame.data.data(), frame.data.size());
    *dataSize = frame.data_size;
    *timeStamp = frame.timestamp_us;
    write_status(status, kHalSuccess);
    return;
  }

  zero_outputs();
  write_status(status, kHalCanMessageNotFound);
}

void HAL_CAN_OpenStreamSession(std::uint32_t* sessionHandle,
                               std::uint32_t messageID,
                               std::uint32_t messageIDMask,
                               std::uint32_t maxMessages,
                               std::int32_t* status) {
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalCanNotAllowed;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *sessionHandle = 0;
    write_status(status, kHalHandleError);
    return;
  }
  if (maxMessages == 0) {
    *sessionHandle = 0;
    write_status(status, kHalCanInvalidBuffer);
    return;
  }

  const std::uint32_t handle = shim->open_can_stream_session(messageID, messageIDMask, maxMessages);
  *sessionHandle = handle;
  write_status(status, handle == 0 ? kHalCanNotAllowed : kHalSuccess);
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
  using robosim::backend::shim::kHalCanInvalidBuffer;
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    *messagesRead = 0;
    write_status(status, kHalHandleError);
    return;
  }
  if (messages == nullptr && messagesToRead > 0) {
    *messagesRead = 0;
    write_status(status, kHalCanInvalidBuffer);
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
  write_status(status, read_status);
}

void HAL_CAN_GetCANStatus(float* percentBusUtilization,
                          std::uint32_t* busOffCount,
                          std::uint32_t* txFullCount,
                          std::uint32_t* receiveErrorCount,
                          std::uint32_t* transmitErrorCount,
                          std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  *percentBusUtilization = 0.0f;
  *busOffCount = 0;
  *txFullCount = 0;
  *receiveErrorCount = 0;
  *transmitErrorCount = 0;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  write_status(status, kHalSuccess);

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

HAL_NotifierHandle HAL_InitializeNotifier(std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }

  const std::int32_t handle = shim->initialize_notifier();
  if (handle == 0) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  return handle;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                         const char* name,
                         std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::safe_view;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  write_status(status, shim->set_notifier_name(notifierHandle, safe_view(name)));
}

HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime,
                                       std::int32_t priority,
                                       std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  (void)realTime;
  (void)priority;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  write_status(status, kHalSuccess);
  return 1;
}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  write_status(status, shim->stop_notifier(notifierHandle));
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle) {
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    return;
  }
  shim->clean_notifier(notifierHandle);
}

void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             std::uint64_t triggerTime,
                             std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  write_status(status, shim->update_notifier_alarm(notifierHandle, triggerTime));
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle, std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return;
  }
  write_status(status, shim->cancel_notifier_alarm(notifierHandle));
}

std::uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle, std::int32_t* status) {
  using robosim::backend::shim::kHalHandleError;
  using robosim::backend::shim::kHalSuccess;
  using robosim::backend::shim::shim_core;

  shim_core* shim = shim_core::current();
  if (shim == nullptr) {
    write_status(status, kHalHandleError);
    return 0;
  }
  std::int32_t result_status = kHalSuccess;
  const std::uint64_t result = shim->wait_for_notifier_alarm(notifierHandle, result_status);
  write_status(status, result_status);
  return result;
}

}  // extern "C"
