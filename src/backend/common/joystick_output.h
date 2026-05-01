#pragma once

#include "ds_state.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace robosim::backend {

/**
 * One Driver Station joystick output command produced by robot code.
 *
 * The raw C HAL values are preserved at the protocol seam: outputs remains the
 * signed 64-bit bitmask accepted by HAL_SetJoystickOutputs, and rumble fields
 * are not clamped by the shim.
 */
struct joystick_output_state {
  std::int32_t joystick_num;
  std::array<std::uint8_t, 4> reserved_pad;
  std::int64_t outputs;
  std::int32_t left_rumble;
  std::int32_t right_rumble;

  bool operator==(const joystick_output_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<joystick_output_state>);
static_assert(std::is_standard_layout_v<joystick_output_state>);
static_assert(!std::is_polymorphic_v<joystick_output_state>);
static_assert(std::is_aggregate_v<joystick_output_state>);
static_assert(offsetof(joystick_output_state, joystick_num) == 0);
static_assert(offsetof(joystick_output_state, reserved_pad) == 4);
static_assert(offsetof(joystick_output_state, outputs) == 8);
static_assert(offsetof(joystick_output_state, left_rumble) == 16);
static_assert(offsetof(joystick_output_state, right_rumble) == 20);
static_assert(sizeof(joystick_output_state) == 24);
static_assert(alignof(joystick_output_state) == 8);

/**
 * Compact active-prefix batch of joystick output commands.
 *
 * Entries are sorted by joystick number when produced by the shim. The batch is
 * serialized with active-prefix bytes only:
 * offsetof(joystick_output_batch, outputs) + count * sizeof(output).
 */
struct joystick_output_batch {
  std::uint32_t count;
  std::array<std::uint8_t, 4> reserved_pad;
  std::array<joystick_output_state, kMaxJoysticks> outputs;

  bool operator==(const joystick_output_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<joystick_output_batch>);
static_assert(std::is_standard_layout_v<joystick_output_batch>);
static_assert(!std::is_polymorphic_v<joystick_output_batch>);
static_assert(std::is_aggregate_v<joystick_output_batch>);
static_assert(offsetof(joystick_output_batch, count) == 0);
static_assert(offsetof(joystick_output_batch, reserved_pad) == 4);
static_assert(offsetof(joystick_output_batch, outputs) == 8);
static_assert(alignof(joystick_output_batch) == 8);

/** Returns the byte span that should be published for this active-prefix batch. */
inline std::span<const std::uint8_t> active_prefix_bytes(const joystick_output_batch& batch) {
  const std::size_t active_size = offsetof(joystick_output_batch, outputs) +
                                  static_cast<std::size_t>(batch.count) *
                                      sizeof(joystick_output_state);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

}  // namespace robosim::backend
