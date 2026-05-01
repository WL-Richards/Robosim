#pragma once

#include <cstdint>
#include <type_traits>

namespace robosim::backend {

/** Snapshot returned by HAL_CAN_GetCANStatus, in HAL parameter order. */
struct can_status {
  float percent_bus_utilization;
  std::uint32_t bus_off_count;
  std::uint32_t tx_full_count;
  std::uint32_t receive_error_count;
  std::uint32_t transmit_error_count;

  bool operator==(const can_status&) const = default;
};

static_assert(std::is_trivially_copyable_v<can_status>);
static_assert(std::is_standard_layout_v<can_status>);
static_assert(!std::is_polymorphic_v<can_status>);
static_assert(std::is_aggregate_v<can_status>);
static_assert(sizeof(can_status) == 20);
static_assert(alignof(can_status) == 4);

}  // namespace robosim::backend
