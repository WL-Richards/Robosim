#pragma once

#include "types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::size_t kMaxNotifiers = 32;
inline constexpr std::size_t kMaxAlarmsPerBatch = 32;
inline constexpr std::size_t kNotifierNameLen = 64;

/**
 * One HAL notifier slot as observed by the shim.
 *
 * Field order keeps the active fields free of interior padding. The struct has
 * trailing padding to satisfy alignof 8; active-prefix serialization omits
 * unused slots but includes full used slots.
 */
struct notifier_slot {
  std::uint64_t trigger_time_us;
  hal_handle handle;
  hal_bool alarm_active;
  hal_bool canceled;
  std::array<char, kNotifierNameLen> name;

  bool operator==(const notifier_slot&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_slot>);
static_assert(std::is_standard_layout_v<notifier_slot>);
static_assert(!std::is_polymorphic_v<notifier_slot>);
static_assert(std::is_aggregate_v<notifier_slot>);
static_assert(sizeof(notifier_slot) == 88);
static_assert(alignof(notifier_slot) == 8);

/** Bounded snapshot of active notifiers. */
struct notifier_state {
  std::uint32_t count;
  std::array<notifier_slot, kMaxNotifiers> slots;

  bool operator==(const notifier_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_state>);
static_assert(std::is_standard_layout_v<notifier_state>);

/**
 * Returns the serialized active prefix for notifier state.
 *
 * The prefix includes the 8-byte header needed to align slots[] plus
 * `count * sizeof(notifier_slot)`.
 */
inline std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_state& state) {
  const std::size_t active_size =
      offsetof(notifier_state, slots) +
      static_cast<std::size_t>(state.count) * sizeof(notifier_slot);
  return {reinterpret_cast<const std::uint8_t*>(&state), active_size};
}

/**
 * Fired notifier alarm event.
 *
 * reserved_pad is a named zero-filled field so the schema has no implicit
 * padding in its serialized representation.
 */
struct notifier_alarm_event {
  std::uint64_t fired_at_us;
  hal_handle handle;
  std::uint32_t reserved_pad;

  bool operator==(const notifier_alarm_event&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_alarm_event>);
static_assert(std::is_standard_layout_v<notifier_alarm_event>);
static_assert(!std::is_polymorphic_v<notifier_alarm_event>);
static_assert(std::is_aggregate_v<notifier_alarm_event>);
static_assert(sizeof(notifier_alarm_event) == 16);
static_assert(alignof(notifier_alarm_event) == 8);

/** Bounded batch of fired notifier alarms. */
struct notifier_alarm_batch {
  std::uint32_t count;
  std::array<notifier_alarm_event, kMaxAlarmsPerBatch> events;

  bool operator==(const notifier_alarm_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_alarm_batch>);
static_assert(std::is_standard_layout_v<notifier_alarm_batch>);

}  // namespace robosim::backend
