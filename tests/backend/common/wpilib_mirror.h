#pragma once

// WPILib HAL byte-layout mirror header used by the parity tests
// (TEST_PLAN section H, decisions #8, #9, #11, #21).
//
// **kWpilibParitySha** below is the WPILib commit these mirrors are
// faithful against. If WPILib bumps and changes a layout, the parity
// tests fail and the developer either:
//   (a) updates kWpilibParitySha + re-transcribes any changed structs
//       here AND bumps kProtocolVersion in src/backend/common/, or
//   (b) keeps the older SHA pin (we lag a WPILib version on purpose).
//
// Drift is a build break, never silent.
//
// Verify the SHA exists at any time with this command (joined to one
// line; the trailing backslash that bash uses must NOT appear in this
// comment because GCC's -Wcomment treats backslash-newline in a //
// comment as a line continuation):
//
//   git ls-remote --exit-code https://github.com/wpilibsuite/allwpilib.git 1fd159938f65ee9d0b22c8a369030b68e0efa82c
//
// (CI is expected to wire that ls-remote check in as a follow-up.)

#include <array>
#include <cstdint>
#include <string_view>

namespace robosim::backend::wpilib_mirror {

inline constexpr std::string_view kWpilibParitySha =
    "1fd159938f65ee9d0b22c8a369030b68e0efa82c";  // tag v2026.2.2

inline constexpr std::size_t HAL_kMaxJoystickAxes  = 12;
inline constexpr std::size_t HAL_kMaxJoystickPOVs  = 12;
inline constexpr std::size_t HAL_kMaxJoysticks     = 6;

// hal/include/hal/Types.h
using HAL_Bool = std::int32_t;
using HAL_Handle = std::int32_t;

// hal/include/hal/DriverStationTypes.h
struct HAL_ControlWord {
  std::uint32_t enabled : 1;
  std::uint32_t autonomous : 1;
  std::uint32_t test : 1;
  std::uint32_t eStop : 1;
  std::uint32_t fmsAttached : 1;
  std::uint32_t dsAttached : 1;
  std::uint32_t control_reserved : 26;
};

enum HAL_AllianceStationID : std::int32_t {
  HAL_AllianceStationID_kUnknown = 0,
  HAL_AllianceStationID_kRed1    = 1,
  HAL_AllianceStationID_kRed2    = 2,
  HAL_AllianceStationID_kRed3    = 3,
  HAL_AllianceStationID_kBlue1   = 4,
  HAL_AllianceStationID_kBlue2   = 5,
  HAL_AllianceStationID_kBlue3   = 6,
};

enum HAL_MatchType : std::int32_t {
  HAL_MatchType_kNone          = 0,
  HAL_MatchType_kPractice      = 1,
  HAL_MatchType_kQualification = 2,
  HAL_MatchType_kElimination   = 3,
};

struct HAL_JoystickAxes {
  std::int16_t count;
  float axes[HAL_kMaxJoystickAxes];
  std::uint8_t raw[HAL_kMaxJoystickAxes];
};

struct HAL_JoystickPOVs {
  std::int16_t count;
  std::int16_t povs[HAL_kMaxJoystickPOVs];
};

struct HAL_JoystickButtons {
  std::uint32_t buttons;
  std::uint8_t count;
};

struct HAL_JoystickDescriptor {
  std::uint8_t isXbox;
  std::uint8_t type;
  char name[256];
  std::uint8_t axisCount;
  std::uint8_t axisTypes[12];
  std::uint8_t buttonCount;
  std::uint8_t povCount;
};

struct HAL_MatchInfo {
  char eventName[64];
  HAL_MatchType matchType;
  std::uint16_t matchNumber;
  std::uint8_t replayNumber;
  std::uint8_t gameSpecificMessage[64];
  std::uint16_t gameSpecificMessageSize;
};

// hal/include/hal/CAN.h
struct HAL_CANStreamMessage {
  std::uint32_t messageID;
  std::uint32_t timeStamp;
  std::uint8_t data[8];
  std::uint8_t dataSize;
};

inline constexpr std::uint32_t HAL_CAN_IS_FRAME_REMOTE = 0x80000000u;
inline constexpr std::uint32_t HAL_CAN_IS_FRAME_11BIT  = 0x40000000u;

}  // namespace robosim::backend::wpilib_mirror
