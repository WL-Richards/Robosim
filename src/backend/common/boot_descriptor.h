#pragma once

#include "protocol_version.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::size_t kWpilibVersionLen = 32;

/**
 * Boot envelope payload describing the robot runtime for a session.
 *
 * Field order is pinned per N1:
 *   runtime_type        at  0 (1 byte)
 *   reserved3           at  1 (3 bytes; zero-pad to align uint32)
 *   team_number         at  4 (4 bytes; session-invariant)
 *   vendor_capabilities at  8 (4 bytes)
 *   wpilib_version      at 12 (32 bytes)
 *
 * sizeof = 44, alignof = 4, no implicit padding.
 */
struct boot_descriptor {
  runtime_type runtime;
  std::array<std::uint8_t, 3> reserved3;
  std::int32_t team_number;
  std::uint32_t vendor_capabilities;
  std::array<char, kWpilibVersionLen> wpilib_version;

  bool operator==(const boot_descriptor&) const = default;
};

static_assert(std::is_trivially_copyable_v<boot_descriptor>);
static_assert(std::is_standard_layout_v<boot_descriptor>);
static_assert(!std::is_polymorphic_v<boot_descriptor>);
static_assert(std::is_aggregate_v<boot_descriptor>);
static_assert(sizeof(boot_descriptor) == 44);
static_assert(alignof(boot_descriptor) == 4);

}  // namespace robosim::backend
