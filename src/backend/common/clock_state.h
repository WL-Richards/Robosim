#pragma once

#include "types.h"

#include <cstdint>
#include <type_traits>

namespace robosim::backend {

/**
 * Per-tick FPGA clock and system-state snapshot.
 *
 * Session invariants such as team number and runtime type live in
 * boot_descriptor. Field order keeps the payload padding-free: the 8-byte
 * timestamp comes first, followed by 4-byte HAL/WPILib fields.
 */
struct clock_state {
  std::uint64_t sim_time_us;
  hal_bool system_active;
  hal_bool browned_out;
  hal_bool system_time_valid;
  hal_bool fpga_button_latched;
  hal_bool rsl_state;
  std::uint32_t comms_disable_count;

  bool operator==(const clock_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<clock_state>);
static_assert(std::is_standard_layout_v<clock_state>);
static_assert(!std::is_polymorphic_v<clock_state>);
static_assert(std::is_aggregate_v<clock_state>);
static_assert(sizeof(clock_state) == 32);

}  // namespace robosim::backend
