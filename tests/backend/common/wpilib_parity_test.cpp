// TEST_PLAN section H — WPILib byte-for-byte parity. The test
// reproduces WPILib's HAL types as local mirror structs (in
// wpilib_mirror.h, with the SHA pinned) and asserts that our protocol
// structs have identical sizeof, alignof, and per-field offsets, and
// that bit-cast round-trips preserve every bit. This is what makes the
// future shim's "memcpy(&hal_struct, &our_struct, sizeof(...))"
// contract sound (decision #9).
//
// Mirror SHA: see wpilib_mirror.h kWpilibParitySha constant.

#include "can_frame.h"
#include "ds_state.h"
#include "wpilib_mirror.h"

#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace robosim::backend {
namespace {

namespace wm = wpilib_mirror;

// ---------------------------------------------------------------------------
// H1 — joystick_axes
// ---------------------------------------------------------------------------

TEST(WpilibParity, JoystickAxesByteForByte) {
  // 2 (count) + 2 (leading pad) + 48 (axes) + 12 (raw) = 64
  EXPECT_EQ(sizeof(joystick_axes), sizeof(wm::HAL_JoystickAxes));
  EXPECT_EQ(sizeof(joystick_axes), 64u);
  EXPECT_EQ(alignof(joystick_axes), alignof(wm::HAL_JoystickAxes));
  EXPECT_EQ(offsetof(joystick_axes, count), 0u);
  EXPECT_EQ(offsetof(joystick_axes, count),
            offsetof(wm::HAL_JoystickAxes, count));
  EXPECT_EQ(offsetof(joystick_axes, axes), 4u);
  EXPECT_EQ(offsetof(joystick_axes, axes),
            offsetof(wm::HAL_JoystickAxes, axes));
  EXPECT_EQ(offsetof(joystick_axes, raw), 52u);
  EXPECT_EQ(offsetof(joystick_axes, raw),
            offsetof(wm::HAL_JoystickAxes, raw));
}

TEST(WpilibParity, JoystickAxesBitCastRoundTrip) {
  joystick_axes ours{};
  ours.count = 5;
  ours.axes[0] = 1.5f;
  ours.axes[11] = -1.0f;
  ours.raw[0] = 0x42;
  ours.raw[11] = 0xCD;

  auto hal = std::bit_cast<wm::HAL_JoystickAxes>(ours);
  EXPECT_EQ(hal.count, 5);
  EXPECT_EQ(hal.axes[0], 1.5f);
  EXPECT_EQ(hal.axes[11], -1.0f);
  EXPECT_EQ(hal.raw[0], 0x42);
  EXPECT_EQ(hal.raw[11], 0xCD);

  auto round_trip = std::bit_cast<joystick_axes>(hal);
  EXPECT_EQ(round_trip, ours);
}

// ---------------------------------------------------------------------------
// H2 — joystick_buttons
// ---------------------------------------------------------------------------

TEST(WpilibParity, JoystickButtonsByteForByte) {
  EXPECT_EQ(sizeof(joystick_buttons), sizeof(wm::HAL_JoystickButtons));
  EXPECT_EQ(sizeof(joystick_buttons), 8u);
  EXPECT_EQ(offsetof(joystick_buttons, buttons), 0u);
  EXPECT_EQ(offsetof(joystick_buttons, buttons),
            offsetof(wm::HAL_JoystickButtons, buttons));
  EXPECT_EQ(offsetof(joystick_buttons, count), 4u);
  EXPECT_EQ(offsetof(joystick_buttons, count),
            offsetof(wm::HAL_JoystickButtons, count));
}

// ---------------------------------------------------------------------------
// H3 — joystick_povs (round-1 reviewer caught a draft with sizeof 28;
// correct value is 26)
// ---------------------------------------------------------------------------

TEST(WpilibParity, JoystickPovsByteForByte) {
  EXPECT_EQ(sizeof(joystick_povs), sizeof(wm::HAL_JoystickPOVs));
  EXPECT_EQ(sizeof(joystick_povs), 26u);
  EXPECT_EQ(alignof(joystick_povs), alignof(wm::HAL_JoystickPOVs));
  EXPECT_EQ(offsetof(joystick_povs, count), 0u);
  EXPECT_EQ(offsetof(joystick_povs, povs), 2u);
  EXPECT_EQ(offsetof(joystick_povs, povs),
            offsetof(wm::HAL_JoystickPOVs, povs));
}

// ---------------------------------------------------------------------------
// H4 — joystick_descriptor
// ---------------------------------------------------------------------------

TEST(WpilibParity, JoystickDescriptorByteForByte) {
  EXPECT_EQ(sizeof(joystick_descriptor), sizeof(wm::HAL_JoystickDescriptor));
  EXPECT_EQ(sizeof(joystick_descriptor), 273u);
  EXPECT_EQ(alignof(joystick_descriptor), 1u);
  EXPECT_EQ(offsetof(joystick_descriptor, is_xbox), 0u);
  EXPECT_EQ(offsetof(joystick_descriptor, is_xbox),
            offsetof(wm::HAL_JoystickDescriptor, isXbox));
  EXPECT_EQ(offsetof(joystick_descriptor, type), 1u);
  EXPECT_EQ(offsetof(joystick_descriptor, type),
            offsetof(wm::HAL_JoystickDescriptor, type));
  EXPECT_EQ(offsetof(joystick_descriptor, name), 2u);
  EXPECT_EQ(offsetof(joystick_descriptor, name),
            offsetof(wm::HAL_JoystickDescriptor, name));
  EXPECT_EQ(offsetof(joystick_descriptor, axis_count), 258u);
  EXPECT_EQ(offsetof(joystick_descriptor, axis_count),
            offsetof(wm::HAL_JoystickDescriptor, axisCount));
  EXPECT_EQ(offsetof(joystick_descriptor, axis_types), 259u);
  EXPECT_EQ(offsetof(joystick_descriptor, axis_types),
            offsetof(wm::HAL_JoystickDescriptor, axisTypes));
  EXPECT_EQ(offsetof(joystick_descriptor, button_count), 271u);
  EXPECT_EQ(offsetof(joystick_descriptor, button_count),
            offsetof(wm::HAL_JoystickDescriptor, buttonCount));
  EXPECT_EQ(offsetof(joystick_descriptor, pov_count), 272u);
  EXPECT_EQ(offsetof(joystick_descriptor, pov_count),
            offsetof(wm::HAL_JoystickDescriptor, povCount));
}

// ---------------------------------------------------------------------------
// H5 — control_word bitfield mirror-cast (ABI: GCC/Clang on x86-64+ARM
// Linux are LSB-first; if ported to MSVC, this test will fail and the
// porter must re-validate)
// ---------------------------------------------------------------------------

TEST(WpilibParity, ControlWordEnabledBit) {
  wm::HAL_ControlWord hcw{};
  hcw.enabled = 1;
  std::uint32_t hcw_bits;
  std::memcpy(&hcw_bits, &hcw, sizeof(hcw));
  EXPECT_EQ(hcw_bits, 0b1u) << "WPILib HAL_ControlWord LSB-first ABI "
                               "violated; if you're on MSVC this is the "
                               "expected first failure";
  EXPECT_EQ(hcw_bits, kControlEnabled);
}

TEST(WpilibParity, ControlWordEachBitMatchesOurConstants) {
  struct case_ {
    std::uint32_t our_constant;
    std::uint32_t wpilib_bits;
  };

  auto bits_of = [](auto setter) {
    wm::HAL_ControlWord cw{};
    setter(cw);
    std::uint32_t out;
    std::memcpy(&out, &cw, sizeof(cw));
    return out;
  };

  EXPECT_EQ(kControlEnabled,
            bits_of([](auto& cw) { cw.enabled = 1; }));
  EXPECT_EQ(kControlAutonomous,
            bits_of([](auto& cw) { cw.autonomous = 1; }));
  EXPECT_EQ(kControlTest,
            bits_of([](auto& cw) { cw.test = 1; }));
  EXPECT_EQ(kControlEStop,
            bits_of([](auto& cw) { cw.eStop = 1; }));
  EXPECT_EQ(kControlFmsAttached,
            bits_of([](auto& cw) { cw.fmsAttached = 1; }));
  EXPECT_EQ(kControlDsAttached,
            bits_of([](auto& cw) { cw.dsAttached = 1; }));
}

// ---------------------------------------------------------------------------
// H6 — alliance_station enum values
// ---------------------------------------------------------------------------

TEST(WpilibParity, AllianceStationValuesMatchWpilib) {
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::unknown),
            wm::HAL_AllianceStationID_kUnknown);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::red_1),
            wm::HAL_AllianceStationID_kRed1);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::red_2),
            wm::HAL_AllianceStationID_kRed2);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::red_3),
            wm::HAL_AllianceStationID_kRed3);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::blue_1),
            wm::HAL_AllianceStationID_kBlue1);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::blue_2),
            wm::HAL_AllianceStationID_kBlue2);
  EXPECT_EQ(static_cast<std::int32_t>(alliance_station::blue_3),
            wm::HAL_AllianceStationID_kBlue3);
  EXPECT_EQ(sizeof(alliance_station), sizeof(wm::HAL_AllianceStationID));
}

// ---------------------------------------------------------------------------
// H7 — match_info (round-2 reviewer caught the offsets)
// ---------------------------------------------------------------------------

TEST(WpilibParity, MatchInfoByteForByte) {
  EXPECT_EQ(sizeof(match_info), sizeof(wm::HAL_MatchInfo));
  EXPECT_EQ(sizeof(match_info), 140u);
  EXPECT_EQ(alignof(match_info), 4u);

  EXPECT_EQ(offsetof(match_info, event_name), 0u);
  EXPECT_EQ(offsetof(match_info, event_name),
            offsetof(wm::HAL_MatchInfo, eventName));

  EXPECT_EQ(offsetof(match_info, type), 64u);
  EXPECT_EQ(offsetof(match_info, type),
            offsetof(wm::HAL_MatchInfo, matchType));

  EXPECT_EQ(offsetof(match_info, match_number), 68u);
  EXPECT_EQ(offsetof(match_info, match_number),
            offsetof(wm::HAL_MatchInfo, matchNumber));

  EXPECT_EQ(offsetof(match_info, replay_number), 70u);
  EXPECT_EQ(offsetof(match_info, replay_number),
            offsetof(wm::HAL_MatchInfo, replayNumber));

  // The critical offset round-2 caught: gameSpecificMessage at 71, not
  // 72. uint8_t[] has alignof 1 so no pad before it.
  EXPECT_EQ(offsetof(match_info, game_specific_message), 71u);
  EXPECT_EQ(offsetof(match_info, game_specific_message),
            offsetof(wm::HAL_MatchInfo, gameSpecificMessage));

  // gameSpecificMessageSize at 136; the 1-byte pad is at offset 135
  // between gameSpecificMessage end at 134 and the align-2 size field
  // at 136.
  EXPECT_EQ(offsetof(match_info, game_specific_message_size), 136u);
  EXPECT_EQ(offsetof(match_info, game_specific_message_size),
            offsetof(wm::HAL_MatchInfo, gameSpecificMessageSize));
}

TEST(WpilibParity, MatchTypeBackingIsInt32) {
  // Decision #21 — HAL_MatchType backing is int32_t.
  EXPECT_EQ(sizeof(match_type), 4u);
  EXPECT_EQ(sizeof(wm::HAL_MatchType), 4u);
  static_assert(std::is_same_v<std::underlying_type_t<match_type>,
                               std::int32_t>);
}

// ---------------------------------------------------------------------------
// H8 — ds_state total size + per-field offsets (round-3 reviewer
// recommendation: pin the offsets so a reorder that preserves total
// size still fails)
// ---------------------------------------------------------------------------

TEST(DsState, FieldOffsetsAndSize) {
  EXPECT_EQ(offsetof(ds_state, joystick_axes_), 0u);
  EXPECT_EQ(offsetof(ds_state, joystick_buttons_), 384u);
  EXPECT_EQ(offsetof(ds_state, joystick_povs_), 432u);
  EXPECT_EQ(offsetof(ds_state, joystick_descriptors), 588u);
  // 2-byte inter-field pad at 2226..2227 before control_word (alignof 4)
  EXPECT_EQ(offsetof(ds_state, control), 2228u);
  EXPECT_EQ(offsetof(ds_state, station), 2232u);
  EXPECT_EQ(offsetof(ds_state, match), 2236u);
  EXPECT_EQ(offsetof(ds_state, match_time_seconds), 2376u);
  EXPECT_EQ(sizeof(ds_state), 2384u);
  EXPECT_EQ(alignof(ds_state), 8u);
}

// ---------------------------------------------------------------------------
// H9 — default-construct ds_state has all-zero string buffers
// ---------------------------------------------------------------------------

TEST(DsState, DefaultConstructIsZeroFilled) {
  ds_state d{};
  std::array<std::uint8_t, sizeof(ds_state)> bytes{};
  std::memcpy(bytes.data(), &d, sizeof(ds_state));
  for (std::size_t i = 0; i < sizeof(ds_state); ++i) {
    EXPECT_EQ(bytes[i], 0u) << "byte " << i << " not zero";
  }
}

// ---------------------------------------------------------------------------
// I1 — can_frame mirrors HAL_CANStreamMessage byte-for-byte (fork F3)
// ---------------------------------------------------------------------------

TEST(WpilibParity, CanFrameByteForByte) {
  EXPECT_EQ(sizeof(can_frame), sizeof(wm::HAL_CANStreamMessage));
  EXPECT_EQ(sizeof(can_frame), 20u);
  EXPECT_EQ(alignof(can_frame), 4u);

  EXPECT_EQ(offsetof(can_frame, message_id), 0u);
  EXPECT_EQ(offsetof(can_frame, message_id),
            offsetof(wm::HAL_CANStreamMessage, messageID));

  EXPECT_EQ(offsetof(can_frame, timestamp_us), 4u);
  EXPECT_EQ(offsetof(can_frame, timestamp_us),
            offsetof(wm::HAL_CANStreamMessage, timeStamp));

  EXPECT_EQ(offsetof(can_frame, data), 8u);
  EXPECT_EQ(offsetof(can_frame, data),
            offsetof(wm::HAL_CANStreamMessage, data));

  EXPECT_EQ(offsetof(can_frame, data_size), 16u);
  EXPECT_EQ(offsetof(can_frame, data_size),
            offsetof(wm::HAL_CANStreamMessage, dataSize));
}

TEST(WpilibParity, CanFrameFlagBitsMatchWpilib) {
  EXPECT_EQ(kCanFlagFrameRemote, wm::HAL_CAN_IS_FRAME_REMOTE);
  EXPECT_EQ(kCanFlagFrame11Bit, wm::HAL_CAN_IS_FRAME_11BIT);
}

}  // namespace
}  // namespace robosim::backend
