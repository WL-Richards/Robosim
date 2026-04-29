#pragma once

#include "types.h"

#include <cstdint>
#include <type_traits>

namespace robosim::backend {

// Per-tick clock + system state. Session-invariants (team_number,
// runtime_type) live in boot_descriptor — fork F1.
//
// Field order chosen for zero interior pad: 8-byte field first, then
// 4-byte fields. Sum = 8 + 6*4 = 32; sizeof = 32; no pad.
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
