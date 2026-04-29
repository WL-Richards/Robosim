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
#include "clock_state.h"
#include "protocol_version.h"
#include "shared_memory_transport.h"
#include "sync_envelope.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <span>
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
