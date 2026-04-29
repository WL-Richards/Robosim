#pragma once

#include <array>
#include <bit>
#include <cstdint>

namespace robosim::backend {

inline constexpr std::uint16_t kProtocolVersion = 1;
inline constexpr std::array<char, 4> kProtocolMagic{'R', 'S', 'S', '1'};

static_assert(std::endian::native == std::endian::little,
              "v0 protocol pins little-endian (decision #3). Big-endian "
              "host requires a byte-swap layer at the transport boundary, "
              "not in the schema; see TEST_PLAN.md decision #3.");

enum class envelope_kind : std::uint16_t {
  reserved          = 0,
  boot              = 1,
  boot_ack          = 2,
  tick_boundary     = 3,
  on_demand_request = 4,
  on_demand_reply   = 5,
  shutdown          = 6,
};

enum class direction : std::uint8_t {
  reserved        = 0,
  backend_to_core = 1,
  core_to_backend = 2,
};

enum class schema_id : std::uint8_t {
  none                  = 0,
  clock_state           = 1,
  power_state           = 2,
  ds_state              = 3,
  can_frame_batch       = 4,
  can_status            = 5,
  notifier_state        = 6,
  notifier_alarm_batch  = 7,
  error_message_batch   = 8,
  boot_descriptor       = 9,
};

inline constexpr std::uint8_t kSchemaIdMaxValid = 9;

enum class runtime_type : std::uint8_t {
  reserved  = 0,
  roborio_2 = 2,
};

}  // namespace robosim::backend
