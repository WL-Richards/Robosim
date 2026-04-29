#pragma once

#include "types.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::size_t kMaxNotifiers = 32;
inline constexpr std::size_t kMaxAlarmsPerBatch = 32;
inline constexpr std::size_t kNotifierNameLen = 64;

// Field order chosen for zero interior pad (uint64 first, then 4-byte
// fields, then byte array).
//   trigger_time_us at  0  (8 bytes)
//   handle          at  8  (4 bytes)
//   alarm_active    at 12  (4 bytes; hal_bool == uint32)
//   canceled        at 16  (4 bytes; hal_bool == uint32)
//   name            at 20  (64 bytes)
// Sum of named fields = 84; sizeof = 88 (trailing pad to align 8);
// alignof = 8.
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

struct notifier_state {
  std::uint32_t count;
  std::array<notifier_slot, kMaxNotifiers> slots;

  bool operator==(const notifier_state&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_state>);
static_assert(std::is_standard_layout_v<notifier_state>);

// Fired alarm event. Layout pinned per K3:
//   fired_at_us  at 0  (8 bytes)
//   handle       at 8  (4 bytes)
//   reserved_pad at 12 (4 bytes; zero-filled, decision #24)
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

struct notifier_alarm_batch {
  std::uint32_t count;
  std::array<notifier_alarm_event, kMaxAlarmsPerBatch> events;

  bool operator==(const notifier_alarm_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<notifier_alarm_batch>);
static_assert(std::is_standard_layout_v<notifier_alarm_batch>);

}  // namespace robosim::backend
