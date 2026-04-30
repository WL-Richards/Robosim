#pragma once

// Shared test helpers for tier1 transport and any code that drives it
// directly (e.g. the HAL shim cycle-1 tests). Inline functions are
// defined here so each consuming TU compiles its own copy without
// requiring a separate translation unit.
//
// What lives here is the *test-fixture* surface: payload constructors,
// envelope builders, endpoint factories, the handshake helper, and the
// `manually_fill_lane` ABI poke. Anything that mutates the shared
// region directly is named explicitly so reviewers can audit each
// call-site for "is this reaching past the transport contract on
// purpose?"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_version.h"
#include "shared_memory_transport.h"
#include "sync_envelope.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace robosim::backend::tier1::helpers {

template <typename T>
inline std::vector<std::uint8_t> bytes_of(const T& value) {
  auto bytes = std::as_bytes(std::span{&value, std::size_t{1}});
  std::vector<std::uint8_t> out(bytes.size());
  std::ranges::transform(
      bytes, out.begin(), [](std::byte b) { return static_cast<std::uint8_t>(b); });
  return out;
}

inline boot_descriptor valid_boot_descriptor() {
  boot_descriptor boot{};
  boot.runtime = runtime_type::roborio_2;
  boot.team_number = 971;
  boot.vendor_capabilities = 0xA5A5'0001u;
  const char version[] = "WPILib-2026.2.2";
  std::memcpy(boot.wpilib_version.data(), version, sizeof(version));
  return boot;
}

inline clock_state valid_clock_state(std::uint64_t sim_time_us = 20'000) {
  clock_state state{};
  state.sim_time_us = sim_time_us;
  state.system_active = 1;
  state.system_time_valid = 1;
  state.rsl_state = 1;
  return state;
}

// Cycle-2 helper. Defaults are realistic FRC operating-point values
// (12.5 V VIN, 2 A current, 6.8 V brownout threshold). All three are
// non-zero, distinct, finite, and bit-distinct from clock_state's
// integer fields, so a "shim writes 0s" or "shim writes the wrong
// slot" bug fails byte-equality loudly. Consumers compare via the
// defaulted power_state::operator== — bit-equal-as-== on the float
// fields for every value the tests produce, since each value is an
// exact constructor-supplied constant rather than an arithmetic
// result. No NaN, no rounding-residue, no signed-zero ambiguity.
inline power_state valid_power_state(float vin_v = 12.5f,
                                     float vin_a = 2.0f,
                                     float brownout_voltage_v = 6.8f) {
  return power_state{vin_v, vin_a, brownout_voltage_v};
}

// Cycle-3 helper. ds_state{}-zero-init is intentional and load-bearing
// (D-C3-7): it covers all five interior padding bytes (2 between
// joystick_descriptors[5] and control, 1 inside match_info between
// game_specific_message[63] and game_specific_message_size, 2 trailing
// in match_info before alignof-4 resync) plus the five non-touched
// joystick slots, the event_name and game_specific_message arrays, and
// every other field this fixture does not assign explicitly. The seven
// distinguished fields span the full 0..2376-byte layout so any
// realistic prefix-or-suffix partial-copy bug fails byte-equality on
// at least one of them. Consumers compare via the defaulted
// ds_state::operator== for field-equality and via std::memcmp where
// padding-byte determinism matters (C3-6); both pass against any
// correct std::memcpy-based implementation.
inline ds_state valid_ds_state(
    std::int16_t joystick0_axis_count   = 3,
    float joystick0_axis_0_value        = 0.5f,
    std::uint32_t control_bits          = kControlEnabled | kControlDsAttached,
    alliance_station station            = alliance_station::red_2,
    match_type type                     = match_type::qualification,
    std::uint16_t match_number          = 42,
    double match_time_seconds           = 12.5) {
  ds_state state{};  // zero-init (D-C3-7).
  state.joystick_axes_[0].count = joystick0_axis_count;
  state.joystick_axes_[0].axes[0] = joystick0_axis_0_value;
  state.control.bits = control_bits;
  state.station = station;
  state.match.type = type;
  state.match.match_number = match_number;
  state.match_time_seconds = match_time_seconds;
  return state;
}

// Cycle-4 helpers.
//
// can_frame{} zero-init covers the 3 trailing padding bytes per frame
// (offsets 17..19) so byte-equality on the wire holds regardless of
// which frame slots are used (D-C4-PADDING).
inline can_frame valid_can_frame(std::uint32_t message_id   = 0x123,
                                 std::uint32_t timestamp_us = 1000,
                                 std::uint8_t data_size     = 4,
                                 std::uint8_t fill_byte     = 0xAB) {
  can_frame f{};
  f.message_id = message_id;
  f.timestamp_us = timestamp_us;
  for (std::size_t i = 0; i < data_size && i < f.data.size(); ++i) {
    f.data[i] = static_cast<std::uint8_t>(fill_byte + i);
  }
  f.data_size = data_size;
  return f;
}

// Builds a can_frame_batch holding `frames.size()` elements with the
// `count` field set accordingly. Unused frames[count..63] are left at
// the can_frame_batch{} zero-init default — important for the
// shrinking-batch contract (D-C4-2) and padding-byte determinism
// (D-C4-PADDING).
inline can_frame_batch valid_can_frame_batch(
    std::span<const can_frame> frames = {}) {
  can_frame_batch batch{};
  batch.count = static_cast<std::uint32_t>(frames.size());
  for (std::size_t i = 0; i < frames.size() && i < batch.frames.size(); ++i) {
    batch.frames[i] = frames[i];
  }
  return batch;
}

// Cycle-5 helper. can_status is fixed-size (20 bytes), no padding —
// 5 × 4-byte fields all naturally aligned. Defaults: 25% bus
// utilization, single-digit error counters; bit-distinct from
// other fixture defaults so a "shim writes wrong slot" bug fails.
inline can_status valid_can_status(
    float percent_bus_utilization     = 0.25f,
    std::uint32_t bus_off_count       = 1,
    std::uint32_t tx_full_count       = 2,
    std::uint32_t receive_error_count = 3,
    std::uint32_t transmit_error_count = 4) {
  return can_status{percent_bus_utilization, bus_off_count,
                    tx_full_count, receive_error_count,
                    transmit_error_count};
}

// Cycle-6 helpers. notifier_slot has 4 trailing implicit pad bytes
// (sizeof = 88 vs. named-field sum = 84). The notifier_slot{} zero-
// init covers them so byte-equality on the wire holds (D-C6-PADDING).
inline notifier_slot valid_notifier_slot(
    std::uint64_t trigger_time_us = 1'000'000,
    hal_handle handle             = 42,
    hal_bool alarm_active         = 1,
    hal_bool canceled             = 0,
    const char* name              = "notifier") {
  notifier_slot s{};
  s.trigger_time_us = trigger_time_us;
  s.handle = handle;
  s.alarm_active = alarm_active;
  s.canceled = canceled;
  if (name != nullptr) {
    const std::size_t n = std::min(std::strlen(name), s.name.size() - 1);
    std::memcpy(s.name.data(), name, n);
  }
  return s;
}

// Builds a notifier_state holding `slots.size()` elements with the
// `count` field set accordingly. Unused slots[count..31] left at the
// notifier_state{} zero-init default — important for the shrinking-
// batch contract (D-C6-VARIABLE-SIZE) and padding-byte determinism
// (D-C6-PADDING; covers the 4-byte interior count→slots pad and all
// per-slot trailing pad).
inline notifier_state valid_notifier_state(
    std::span<const notifier_slot> slots = {}) {
  notifier_state state{};
  state.count = static_cast<std::uint32_t>(slots.size());
  for (std::size_t i = 0; i < slots.size() && i < state.slots.size(); ++i) {
    state.slots[i] = slots[i];
  }
  return state;
}

// Cycle-7 helpers. notifier_alarm_event has a NAMED reserved_pad field
// (not implicit padding); operator== covers all bytes of an event.
inline notifier_alarm_event valid_notifier_alarm_event(
    std::uint64_t fired_at_us = 1'000'000,
    hal_handle handle         = 42) {
  notifier_alarm_event event{};
  event.fired_at_us = fired_at_us;
  event.handle = handle;
  // reserved_pad stays zero from zero-init.
  return event;
}

// notifier_alarm_batch: count + array<notifier_alarm_event, 32>.
// Implicit padding: 4 bytes between count and events (events has
// alignof 8). zero-init covers it (D-C7-PADDING).
inline notifier_alarm_batch valid_notifier_alarm_batch(
    std::span<const notifier_alarm_event> events = {}) {
  notifier_alarm_batch batch{};
  batch.count = static_cast<std::uint32_t>(events.size());
  for (std::size_t i = 0; i < events.size() && i < batch.events.size(); ++i) {
    batch.events[i] = events[i];
  }
  return batch;
}

inline std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_alarm_batch& batch) {
  const std::size_t active_size =
      offsetof(notifier_alarm_batch, events) +
      static_cast<std::size_t>(batch.count) * sizeof(notifier_alarm_event);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

// Cycle-8 helpers. error_message has a NAMED reserved_pad[3] field
// after truncation_flags, and error_message_batch has a NAMED
// reserved_pad[4] field between count and messages. Both are part of
// the schema layout, not implicit C++ padding; defaulted operator==
// covers every byte.
inline error_message valid_error_message(
    std::int32_t error_code             = 1,
    hal_bool severity                   = 1,
    hal_bool is_lv_code                 = 0,
    hal_bool print_msg                  = 1,
    std::uint8_t truncation_flags       = 0,
    std::string_view details_text       = "",
    std::string_view location_text      = "",
    std::string_view call_stack_text    = "") {
  error_message msg{};  // zero-init covers reserved_pad and the tail
                        // of every fixed-size buffer (NUL-terminated).
  msg.error_code       = error_code;
  msg.severity         = severity;
  msg.is_lv_code       = is_lv_code;
  msg.print_msg        = print_msg;
  msg.truncation_flags = truncation_flags;
  // reserved_pad stays zero from zero-init.
  const auto copy_into = [](auto& buf, std::string_view s) {
    const std::size_t n = std::min(s.size(), buf.size() - 1);
    std::memcpy(buf.data(), s.data(), n);
  };
  copy_into(msg.details,    details_text);
  copy_into(msg.location,   location_text);
  copy_into(msg.call_stack, call_stack_text);
  return msg;
}

inline error_message_batch valid_error_message_batch(
    std::span<const error_message> messages = {}) {
  error_message_batch batch{};
  batch.count = static_cast<std::uint32_t>(messages.size());
  for (std::size_t i = 0;
       i < messages.size() && i < batch.messages.size();
       ++i) {
    batch.messages[i] = messages[i];
  }
  // batch.reserved_pad stays zero from zero-init.
  return batch;
}

inline std::vector<std::uint8_t> boot_payload() {
  return bytes_of(valid_boot_descriptor());
}

inline std::vector<std::uint8_t> clock_payload() {
  return bytes_of(valid_clock_state());
}

inline sync_envelope make_envelope(envelope_kind kind,
                                   schema_id payload_schema,
                                   std::uint32_t payload_bytes,
                                   std::uint64_t sequence,
                                   direction sender) {
  sync_envelope env{};
  env.magic = kProtocolMagic;
  env.protocol_version = kProtocolVersion;
  env.kind = kind;
  env.sequence = sequence;
  env.payload_bytes = payload_bytes;
  env.payload_schema = payload_schema;
  env.sender = sender;
  return env;
}

// Direct lane mutation. Used by tests that need to plant a state no
// real peer can legitimately produce (out-of-order envelope; partial
// `writing` state). The shared-region struct is the wire ABI, so this
// is reaching at a public surface, not at private state — but each
// call-site should explain *why* a real peer cannot produce it.
inline void manually_fill_lane(tier1_lane& lane,
                               const sync_envelope& envelope,
                               std::span<const std::uint8_t> payload) {
  lane.envelope = envelope;
  lane.payload_bytes = static_cast<std::uint32_t>(payload.size());
  std::copy(payload.begin(), payload.end(), lane.payload.begin());
  lane.state.store(static_cast<std::uint32_t>(tier1_lane_state::full),
                   std::memory_order_release);
}

inline tier1_endpoint make_backend(tier1_shared_region& region) {
  auto endpoint = tier1_endpoint::make(region, direction::backend_to_core);
  EXPECT_TRUE(endpoint.has_value());
  return std::move(*endpoint);
}

inline tier1_endpoint make_core(tier1_shared_region& region) {
  auto endpoint = tier1_endpoint::make(region, direction::core_to_backend);
  EXPECT_TRUE(endpoint.has_value());
  return std::move(*endpoint);
}

inline void complete_handshake(tier1_endpoint& backend, tier1_endpoint& core) {
  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
  auto ack = backend.try_receive();
  ASSERT_TRUE(ack.has_value());
}

}  // namespace robosim::backend::tier1::helpers
