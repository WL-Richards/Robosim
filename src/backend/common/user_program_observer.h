#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

/** Host-visible state last reported by the robot program observer HAL calls. */
enum class user_program_observer_mode : std::int32_t {
  none       = 0,
  starting   = 1,
  disabled   = 2,
  autonomous = 3,
  teleop     = 4,
  test       = 5,
};

static_assert(sizeof(user_program_observer_mode) == 4);
static_assert(alignof(user_program_observer_mode) == 4);

/** Fixed-size user-program observer snapshot published by the HAL shim. */
struct user_program_observer_snapshot {
  user_program_observer_mode mode;
  std::array<std::uint8_t, 4> reserved_pad;

  bool operator==(const user_program_observer_snapshot&) const = default;
};

static_assert(std::is_trivially_copyable_v<user_program_observer_snapshot>);
static_assert(std::is_standard_layout_v<user_program_observer_snapshot>);
static_assert(!std::is_polymorphic_v<user_program_observer_snapshot>);
static_assert(std::is_aggregate_v<user_program_observer_snapshot>);
static_assert(offsetof(user_program_observer_snapshot, mode) == 0);
static_assert(offsetof(user_program_observer_snapshot, reserved_pad) == 4);
static_assert(sizeof(user_program_observer_snapshot) == 8);
static_assert(alignof(user_program_observer_snapshot) == 4);

}  // namespace robosim::backend
