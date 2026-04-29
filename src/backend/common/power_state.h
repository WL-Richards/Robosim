#pragma once

#include <type_traits>

namespace robosim::backend {

struct power_state {
  float vin_v;
  float vin_a;
  float brownout_voltage_v;

  bool operator==(const power_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<power_state>);
static_assert(std::is_standard_layout_v<power_state>);
static_assert(!std::is_polymorphic_v<power_state>);
static_assert(std::is_aggregate_v<power_state>);
static_assert(sizeof(power_state) == 12);
static_assert(alignof(power_state) == 4);

}  // namespace robosim::backend
