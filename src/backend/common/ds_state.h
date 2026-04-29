#pragma once

#include "types.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

// Mirrors of WPILib HAL constants from
// hal/include/hal/DriverStationTypes.h@kWpilibParitySha
// (decision #8). If WPILib changes any of these, the parity tests fail
// and the developer updates both the constants here and the WPILib pin
// together; otherwise the future shim's memcpy contract breaks.
inline constexpr std::size_t kMaxJoysticks         = 6;
inline constexpr std::size_t kMaxJoystickAxes      = 12;
inline constexpr std::size_t kMaxJoystickPOVs      = 12;
inline constexpr std::size_t kJoystickNameLen      = 256;
inline constexpr std::size_t kEventNameLen         = 64;
inline constexpr std::size_t kGameSpecificMsgLen   = 64;
inline constexpr std::size_t kJoystickAxisTypesLen = 12;

// HAL_ControlWord: 6 single-bit flags + 26 bits reserved, packed into a
// uint32_t. Bit assignments (LSB-first on GCC/Clang on x86-64+ARM
// Linux; see TEST_PLAN H5 ABI note):
//   bit 0 = enabled
//   bit 1 = autonomous
//   bit 2 = test
//   bit 3 = e_stop
//   bit 4 = fms_attached
//   bit 5 = ds_attached
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

// Mirrors HAL_AllianceStationID. WPILib backs this with a default-int
// enum, so sizeof == 4 on all FRC platforms.
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

// Mirrors HAL_MatchType. Backing pinned to int32_t (decision #21).
enum class match_type : std::int32_t {
  none         = 0,
  practice     = 1,
  qualification = 2,
  elimination  = 3,
};

static_assert(sizeof(match_type) == 4);
static_assert(alignof(match_type) == 4);

// Mirrors HAL_JoystickAxes byte-for-byte:
//   count at 0 (int16_t, 2 bytes)
//   2 bytes leading pad to align float
//   axes  at 4  (float[12] = 48 bytes)
//   raw   at 52 (uint8_t[12] = 12 bytes)
// sizeof = 64, alignof = 4, no trailing pad.
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

// Mirrors HAL_JoystickButtons byte-for-byte:
//   buttons at 0 (uint32_t, 4 bytes)
//   count   at 4 (uint8_t, 1 byte)
//   3 bytes trailing pad to alignof 4
// sizeof = 8.
struct joystick_buttons {
  std::uint32_t buttons;
  std::uint8_t count;

  bool operator==(const joystick_buttons&) const = default;
};

static_assert(sizeof(joystick_buttons) == 8);
static_assert(alignof(joystick_buttons) == 4);
static_assert(std::is_trivially_copyable_v<joystick_buttons>);
static_assert(std::is_standard_layout_v<joystick_buttons>);

// Mirrors HAL_JoystickPOVs byte-for-byte:
//   count at 0 (int16_t, 2 bytes)
//   povs  at 2 (int16_t[12] = 24 bytes)
// sizeof = 26, alignof = 2, no trailing pad. (Round-1/2 reviewer
// caught a draft that had this at 28; correct is 26.)
struct joystick_povs {
  std::int16_t count;
  std::array<std::int16_t, kMaxJoystickPOVs> povs;

  bool operator==(const joystick_povs&) const = default;
};

static_assert(sizeof(joystick_povs) == 26);
static_assert(alignof(joystick_povs) == 2);
static_assert(std::is_trivially_copyable_v<joystick_povs>);
static_assert(std::is_standard_layout_v<joystick_povs>);

// Mirrors HAL_JoystickDescriptor byte-for-byte:
//   is_xbox      at   0 (1 byte)
//   type         at   1 (1 byte)
//   name         at   2 (256 bytes)
//   axis_count   at 258 (1 byte)
//   axis_types   at 259 (12 bytes)
//   button_count at 271 (1 byte)
//   pov_count    at 272 (1 byte)
// sizeof = 273, alignof = 1, no padding (all uint8/char).
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

// Mirrors HAL_MatchInfo byte-for-byte (round-2 reviewer compiled
// against WPILib at kWpilibParitySha to verify):
//   event_name                     at   0 (64 bytes)
//   match_type                     at  64 (4 bytes; int32 backing)
//   match_number                   at  68 (2 bytes)
//   replay_number                  at  70 (1 byte)
//   game_specific_message          at  71 (64 bytes; uint8[] alignof 1
//                                          so no pad before it — round-2
//                                          caught a draft with msg at 72)
//   1 byte interior pad            at 135 (between msg end at 134 and
//                                          align-2 size field at 136)
//   game_specific_message_size     at 136 (2 bytes)
//   2 bytes trailing pad           at 138..139 (struct alignof 4 from
//                                              match_type)
// sizeof = 140, alignof = 4.
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

// ds_state bundles all six joysticks plus the per-tick DS frame state.
// Field order chosen so the 8-byte match_time_seconds lands at offset
// 2376 (already 8-aligned) — no trailing pad needed; one inter-field
// alignment pad of 2 bytes appears between joystick_descriptors (ends
// at 2226) and control_word (4-aligned, at 2228). See TEST_PLAN H8.
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
