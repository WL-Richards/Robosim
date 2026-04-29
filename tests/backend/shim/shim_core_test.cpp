#include "shim_core.h"

#include "boot_descriptor.h"
#include "clock_state.h"
#include "power_state.h"
#include "protocol_session.h"
#include "protocol_version.h"
#include "shared_memory_transport.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>

namespace robosim::backend::shim {
namespace {

using tier1::tier1_endpoint;
using tier1::tier1_lane_state;
using tier1::tier1_shared_region;
using tier1::tier1_transport_error_kind;

using tier1::helpers::bytes_of;
using tier1::helpers::make_backend;
using tier1::helpers::make_core;
using tier1::helpers::make_envelope;
using tier1::helpers::manually_fill_lane;
using tier1::helpers::valid_boot_descriptor;
using tier1::helpers::valid_clock_state;

constexpr std::uint64_t kBootSimTime = 123'456;

// Drives the shim through the boot+boot_ack handshake, so subsequent
// poll()s can exercise post-connect behavior. Returns the live
// shim_core wrapping `region`'s backend endpoint, with `core` already
// initialized to the matching peer.
shim_core make_connected_shim(tier1_shared_region& region, tier1_endpoint& core) {
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  EXPECT_TRUE(shim_or.has_value());
  core = make_core(region);
  // Drain the shim's boot envelope on the core side.
  auto boot = core.try_receive();
  EXPECT_TRUE(boot.has_value());
  // Send boot_ack back so the shim can transition to connected.
  EXPECT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));
  auto poll_result = shim_or->poll();
  EXPECT_TRUE(poll_result.has_value());
  EXPECT_TRUE(shim_or->is_connected());
  return std::move(*shim_or);
}

}  // namespace

// ============================================================================
// Test 1: make() publishes one boot envelope into backend_to_core.
// ============================================================================
TEST(ShimCoreMake, PublishesBootEnvelopeIntoBackendToCoreLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  const auto desc = valid_boot_descriptor();

  auto shim = shim_core::make(std::move(endpoint), desc, kBootSimTime);
  ASSERT_TRUE(shim.has_value());

  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  const auto& env = region.backend_to_core.envelope;
  EXPECT_EQ(env.kind, envelope_kind::boot);
  EXPECT_EQ(env.payload_schema, schema_id::boot_descriptor);
  EXPECT_EQ(env.payload_bytes, sizeof(boot_descriptor));
  EXPECT_EQ(env.sim_time_us, kBootSimTime);
  EXPECT_EQ(env.sender, direction::backend_to_core);
  EXPECT_EQ(env.sequence, 0u);
  EXPECT_EQ(std::memcmp(region.backend_to_core.payload.data(), &desc, sizeof(boot_descriptor)),
            0);
}

// ============================================================================
// Test 2: make() failure-atomicity when the outbound lane is already full.
// ============================================================================
TEST(ShimCoreMake, FailsWhenBackendLaneAlreadyFullAndDoesNotClobberLane) {
  tier1_shared_region region{};
  // Pre-fill backend_to_core with a sentinel payload of 0xCC. Use a
  // valid framing so the lane is structurally well-formed; the lane's
  // `state == full` is what makes a fresh `make` reject.
  const auto sentinel_env = make_envelope(envelope_kind::boot,
                                          schema_id::boot_descriptor,
                                          sizeof(boot_descriptor),
                                          0,
                                          direction::backend_to_core);
  std::vector<std::uint8_t> sentinel_payload(sizeof(boot_descriptor), std::uint8_t{0xCC});
  manually_fill_lane(region.backend_to_core, sentinel_env, sentinel_payload);

  auto endpoint = make_backend(region);
  auto shim = shim_core::make(std::move(endpoint), valid_boot_descriptor(), 0);
  ASSERT_FALSE(shim.has_value());
  EXPECT_EQ(shim.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(shim.error().transport_error.has_value());
  EXPECT_EQ(shim.error().transport_error->kind, tier1_transport_error_kind::lane_busy);

  // Lane unchanged — still full, still 0xCC bytes.
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  for (std::size_t i = 0; i < sizeof(boot_descriptor); ++i) {
    EXPECT_EQ(region.backend_to_core.payload[i], std::uint8_t{0xCC}) << "byte " << i;
  }
}

// ============================================================================
// Test 3: make() rejects a core_to_backend endpoint (wiring bug class).
// ============================================================================
TEST(ShimCoreMake, FailsWhenGivenCoreToBackendEndpoint) {
  tier1_shared_region region{};
  auto core_endpoint = make_core(region);
  auto shim = shim_core::make(std::move(core_endpoint), valid_boot_descriptor(), 0);
  ASSERT_FALSE(shim.has_value());
  EXPECT_EQ(shim.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(shim.error().transport_error.has_value());
  EXPECT_EQ(shim.error().transport_error->kind,
            tier1_transport_error_kind::session_rejected_envelope);
  ASSERT_TRUE(shim.error().transport_error->session_failure.has_value());
  EXPECT_EQ(shim.error().transport_error->session_failure->kind,
            session_error_kind::boot_wrong_direction);

  // Neither lane is touched.
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test 4: post-make state is not-yet-connected, no cached clock.
// ============================================================================
TEST(ShimCoreObservers, IsConnectedFalseAndClockStateNulloptUntilBootAck) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.is_connected());
  EXPECT_FALSE(shim.is_shutting_down());
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// ============================================================================
// Test 5: poll() on an empty inbound lane succeeds with no state change.
// ============================================================================
TEST(ShimCorePoll, OnEmptyInboundLaneSucceedsWithNoStateChange) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  auto result = shim_or->poll();
  EXPECT_TRUE(result.has_value());
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test 6: poll() accepts boot_ack and flips to connected exactly once.
// ============================================================================
TEST(ShimCorePoll, AcceptsBootAckAndFlipsToConnectedExactlyOnce) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  auto core = make_core(region);
  // Drain the shim's boot so the core's session advances.
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
  // Core sends boot_ack.
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  auto first_poll = shim_or->poll();
  EXPECT_TRUE(first_poll.has_value());
  EXPECT_TRUE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->is_shutting_down());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  // Second poll on the now-empty lane is a no-op (succeeds, no flip).
  auto second_poll = shim_or->poll();
  EXPECT_TRUE(second_poll.has_value());
  EXPECT_TRUE(shim_or->is_connected());
}

// ============================================================================
// Test 7: post-connect, clock_state envelope updates the cache byte-equal.
// ============================================================================
TEST(ShimCorePoll, AfterConnectAcceptsClockStateAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_clock_state(250'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        250'000));
  auto result = shim.poll();
  EXPECT_TRUE(result.has_value());

  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), state);
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test 8: latest-wins semantics for repeated clock_state updates.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedClockStateUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // First update.
  const auto first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(first),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);

  // Second update.
  const auto second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(second),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
}

// ============================================================================
// Test 9: out-of-order tick_boundary before boot_ack is rejected; lane
// preserved unchanged.
//
// Why direct lane injection: no real peer can produce this state. A
// core-side protocol_session would refuse to build a tick_boundary
// before sending boot_ack. The bug class under test is "the shim's
// own session correctly rejects an unexpected inbound that bypassed
// pairing" — a real failure mode in a multi-process system where the
// peer could be misbehaving or lane data could be stale after a crash.
// ============================================================================
TEST(ShimCorePoll, RejectsTickBoundaryBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  const auto state = valid_clock_state(100'000);
  const auto payload_bytes = bytes_of(state);
  auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                    schema_id::clock_state,
                                    static_cast<std::uint32_t>(payload_bytes.size()),
                                    0,  // sequence 0 (next-expected for shim)
                                    direction::core_to_backend);
  injected_env.sim_time_us = 100'000;
  manually_fill_lane(region.core_to_backend, injected_env, payload_bytes);

  auto result = shim_or->poll();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, shim_error_kind::receive_failed);
  ASSERT_TRUE(result.error().transport_error.has_value());
  EXPECT_EQ(result.error().transport_error->kind,
            tier1_transport_error_kind::session_rejected_envelope);
  ASSERT_TRUE(result.error().transport_error->session_failure.has_value());
  EXPECT_EQ(result.error().transport_error->session_failure->kind,
            session_error_kind::expected_boot_ack_first);

  // Lane preserved unchanged (failure atomicity).
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, payload_bytes.size());

  // Shim state untouched.
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
}

// ============================================================================
// Test 10: cycle-1 limit. Schemas other than clock_state, none are
// rejected loudly. Crucially, the session counter must advance through
// the rejection so a fresh valid envelope at the next-expected sequence
// still works.
// ============================================================================
TEST(ShimCorePoll, RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // Step A — send a valid tick_boundary/power_state. Framing is valid
  // (kPerKindAllowedSchemas allows power_state under tick_boundary),
  // so the protocol_session accepts and advances next-expected to 2.
  // The shim's post-session dispatch refuses the schema.
  const power_state pstate{};
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(pstate),
                        1'000));
  auto step_a = shim.poll();
  ASSERT_FALSE(step_a.has_value());
  EXPECT_EQ(step_a.error().kind, shim_error_kind::unsupported_payload_schema);
  EXPECT_NE(step_a.error().offending_field_name.find("payload_schema"), std::string::npos);
  EXPECT_TRUE(shim.is_connected());
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  // Lane drained (session accepted the framing).
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  // Step B — send a valid tick_boundary/clock_state at next sequence.
  // If Step A corrupted the session counter, this will fail.
  const auto cstate = valid_clock_state(250'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(cstate),
                        2'000));
  auto step_b = shim.poll();
  ASSERT_TRUE(step_b.has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), cstate);
}

// ============================================================================
// Test 11: shutdown is terminal-ish (transitions shim into shutting-down).
// ============================================================================
TEST(ShimCorePoll, AcceptsShutdownAndTransitionsToShuttingDown) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  auto result = shim.poll();
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(shim.is_shutting_down());
  EXPECT_TRUE(shim.is_connected());
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// ============================================================================
// Test 12: post-shutdown poll returns terminal error and does not
// touch cache.
// ============================================================================
TEST(ShimCorePoll, AfterShutdownReturnsTerminalErrorAndIgnoresLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());

  // Core (whose send-side is unaffected by sending shutdown) sends a
  // tick_boundary. The shim must refuse to drain it.
  const auto state = valid_clock_state(250'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        6'000));
  auto result = shim.poll();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// ============================================================================
// Test 13: lane_in_progress transport error is wrapped without losing
// the underlying diagnostic. (Lane preservation is owned by tier1.)
// ============================================================================
TEST(ShimCorePoll, WrapsTransportInProgressErrorWithoutMutatingState) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  // Set the inbound lane to writing — a state real peers transit
  // through but never leave; the shim must surface the resulting
  // lane_in_progress through its own error type.
  region.core_to_backend.state.store(
      static_cast<std::uint32_t>(tier1_lane_state::writing), std::memory_order_release);

  auto result = shim_or->poll();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, shim_error_kind::receive_failed);
  ASSERT_TRUE(result.error().transport_error.has_value());
  EXPECT_EQ(result.error().transport_error->kind,
            tier1_transport_error_kind::lane_in_progress);
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
}

// ============================================================================
// Test 14: determinism — same input sequence on two independent setups
// produces a byte-identical cached clock_state.
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalLatestClockState) {
  auto run_scenario = [](tier1_shared_region& region) -> shim_core {
    tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
    auto shim = make_connected_shim(region, core);

    const auto first = valid_clock_state(50'000);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::clock_state,
                          bytes_of(first),
                          50'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto second = valid_clock_state(100'000);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::clock_state,
                          bytes_of(second),
                          100'000));
    EXPECT_TRUE(shim.poll().has_value());

    return shim;
  };

  tier1_shared_region region_a{};
  tier1_shared_region region_b{};
  auto shim_a = run_scenario(region_a);
  auto shim_b = run_scenario(region_b);

  ASSERT_TRUE(shim_a.latest_clock_state().has_value());
  ASSERT_TRUE(shim_b.latest_clock_state().has_value());
  EXPECT_EQ(*shim_a.latest_clock_state(), *shim_b.latest_clock_state());
  EXPECT_TRUE(shim_a.is_connected());
  EXPECT_TRUE(shim_b.is_connected());
}

}  // namespace robosim::backend::shim
