#pragma once

#include "types.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

/**
 * WPILib HAL DriverStation constant mirrors.
 *
 * If WPILib changes these values, parity tests should fail and force an
 * explicit update of both these constants and the WPILib pin. Silent drift
 * would break the shim's fixed-layout memcpy contract.
 */
inline constexpr std::size_t kMaxJoysticks         = 6;
inline constexpr std::size_t kMaxJoystickAxes      = 12;
inline constexpr std::size_t kMaxJoystickPOVs      = 12;
inline constexpr std::size_t kJoystickNameLen      = 256;
inline constexpr std::size_t kEventNameLen         = 64;
inline constexpr std::size_t kGameSpecificMsgLen   = 64;
inline constexpr std::size_t kJoystickAxisTypesLen = 12;

/**
 * HAL_ControlWord packed into a uint32_t.
 *
 * Bit assignments:
 *   bit 0 = enabled
 *   bit 1 = autonomous
 *   bit 2 = test
 *   bit 3 = e_stop
 *   bit 4 = fms_attached
 *   bit 5 = ds_attached
 */
struct control_word {
  std::uint32_t bits;
  bool operator==(const control_word&) const = default;
};

inline constexpr std::uint32_t kControlEnabled     = 0b000001u;
inline constexpr std::uint32_t kControlAutonomous  = 0b000010u;
inline constexpr std::uint32_t kControlTest        = 0b000100u;
inline constexpr std::uint32_t kControlEStop       = 0b001000u;
inline constexpr std::uint32_t kControlFmsAttached = 0b010000u;
inline constexpr std::uint32_t kControlDsAttached  = 0b100000u;

static_assert(sizeof(control_word) == 4);
static_assert(alignof(control_word) == 4);
static_assert(std::is_trivially_copyable_v<control_word>);
static_assert(std::is_standard_layout_v<control_word>);

/** Mirrors HAL_AllianceStationID with the WPILib 4-byte enum ABI. */
enum class alliance_station : std::int32_t {
  unknown = 0,
  red_1   = 1,
  red_2   = 2,
  red_3   = 3,
  blue_1  = 4,
  blue_2  = 5,
  blue_3  = 6,
};

static_assert(sizeof(alliance_station) == 4);
static_assert(alignof(alliance_station) == 4);

/** Mirrors HAL_MatchType with int32_t backing. */
enum class match_type : std::int32_t {
  none         = 0,
  practice     = 1,
  qualification = 2,
  elimination  = 3,
};

static_assert(sizeof(match_type) == 4);
static_assert(alignof(match_type) == 4);

/**
 * Mirrors HAL_JoystickAxes byte-for-byte.
 *
 * Layout: count at 0, axes at 4, raw at 52; sizeof = 64, alignof = 4.
 */
struct joystick_axes {
  std::int16_t count;
  std::array<float, kMaxJoystickAxes> axes;
  std::array<std::uint8_t, kMaxJoystickAxes> raw;

  bool operator==(const joystick_axes&) const = default;
};

static_assert(sizeof(joystick_axes) == 64);
static_assert(alignof(joystick_axes) == 4);
static_assert(std::is_trivially_copyable_v<joystick_axes>);
static_assert(std::is_standard_layout_v<joystick_axes>);

/**
 * Mirrors HAL_JoystickButtons byte-for-byte.
 *
 * Layout: buttons at 0, count at 4, trailing pad at 5..7; sizeof = 8.
 */
struct joystick_buttons {
  std::uint32_t buttons;
  std::uint8_t count;

  bool operator==(const joystick_buttons&) const = default;
};

static_assert(sizeof(joystick_buttons) == 8);
static_assert(alignof(joystick_buttons) == 4);
static_assert(std::is_trivially_copyable_v<joystick_buttons>);
static_assert(std::is_standard_layout_v<joystick_buttons>);

/**
 * Mirrors HAL_JoystickPOVs byte-for-byte.
 *
 * Layout: count at 0, povs at 2; sizeof = 26, alignof = 2.
 */
struct joystick_povs {
  std::int16_t count;
  std::array<std::int16_t, kMaxJoystickPOVs> povs;

  bool operator==(const joystick_povs&) const = default;
};

static_assert(sizeof(joystick_povs) == 26);
static_assert(alignof(joystick_povs) == 2);
static_assert(std::is_trivially_copyable_v<joystick_povs>);
static_assert(std::is_standard_layout_v<joystick_povs>);

/**
 * Mirrors HAL_JoystickDescriptor byte-for-byte.
 *
 * The struct is alignof 1 and intentionally has no padding; fixed-size string
 * fields must be truncated and zero-filled by callers before serialization.
 */
struct joystick_descriptor {
  std::uint8_t is_xbox;
  std::uint8_t type;
  std::array<char, kJoystickNameLen> name;
  std::uint8_t axis_count;
  std::array<std::uint8_t, kJoystickAxisTypesLen> axis_types;
  std::uint8_t button_count;
  std::uint8_t pov_count;

  bool operator==(const joystick_descriptor&) const = default;
};

static_assert(sizeof(joystick_descriptor) == 273);
static_assert(alignof(joystick_descriptor) == 1);
static_assert(std::is_trivially_copyable_v<joystick_descriptor>);
static_assert(std::is_standard_layout_v<joystick_descriptor>);

/**
 * Mirrors HAL_MatchInfo byte-for-byte.
 *
 * Notable layout details: game_specific_message starts at byte 71, one
 * interior pad byte appears before game_specific_message_size at byte 136,
 * and sizeof = 140 due to match_type's 4-byte alignment.
 */
struct match_info {
  std::array<char, kEventNameLen> event_name;
  match_type type;
  std::uint16_t match_number;
  std::uint8_t replay_number;
  std::array<std::uint8_t, kGameSpecificMsgLen> game_specific_message;
  std::uint16_t game_specific_message_size;

  bool operator==(const match_info&) const = default;
};

static_assert(sizeof(match_info) == 140);
static_assert(alignof(match_info) == 4);
static_assert(std::is_trivially_copyable_v<match_info>);
static_assert(std::is_standard_layout_v<match_info>);

/**
 * Complete per-tick Driver Station state.
 *
 * Bundles all six joystick slots, control word, alliance/match metadata, and
 * match time. Field order keeps match_time_seconds naturally 8-aligned and
 * matches the tested v1 wire layout.
 */
struct ds_state {
  std::array<joystick_axes, kMaxJoysticks> joystick_axes_;
  std::array<joystick_buttons, kMaxJoysticks> joystick_buttons_;
  std::array<joystick_povs, kMaxJoysticks> joystick_povs_;
  std::array<joystick_descriptor, kMaxJoysticks> joystick_descriptors;
  control_word control;
  alliance_station station;
  match_info match;
  double match_time_seconds;

  bool operator==(const ds_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<ds_state>);
static_assert(std::is_standard_layout_v<ds_state>);
static_assert(!std::is_polymorphic_v<ds_state>);
static_assert(std::is_aggregate_v<ds_state>);
static_assert(sizeof(ds_state) == 2384);
static_assert(alignof(ds_state) == 8);

}  // namespace robosim::backend
