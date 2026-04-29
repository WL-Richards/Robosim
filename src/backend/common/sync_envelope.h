#pragma once

#include "protocol_version.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

// 32-byte fixed envelope that prefixes every HAL ↔ Sim Core exchange.
// Wire-contract layout (TEST_PLAN section C1):
//   magic            at  0  (4 bytes)
//   protocol_version at  4  (2 bytes)
//   kind             at  6  (2 bytes)
//   sequence         at  8  (8 bytes)
//   sim_time_us      at 16  (8 bytes)
//   payload_bytes    at 24  (4 bytes)
//   payload_schema   at 28  (1 byte)
//   sender           at 29  (1 byte)
//   reserved         at 30  (2 bytes; sender writes zero, validator
//                            ignores per decision #24)
struct sync_envelope {
  std::array<char, 4> magic;
  std::uint16_t protocol_version;
  envelope_kind kind;
  std::uint64_t sequence;
  std::uint64_t sim_time_us;
  std::uint32_t payload_bytes;
  schema_id payload_schema;
  direction sender;
  std::array<std::uint8_t, 2> reserved;

  bool operator==(const sync_envelope&) const = default;
};

static_assert(std::is_trivially_copyable_v<sync_envelope>);
static_assert(std::is_standard_layout_v<sync_envelope>);
static_assert(!std::is_polymorphic_v<sync_envelope>);
static_assert(std::is_aggregate_v<sync_envelope>);
static_assert(sizeof(sync_envelope) == 32);
static_assert(alignof(sync_envelope) == 8);

}  // namespace robosim::backend
