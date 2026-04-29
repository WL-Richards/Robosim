#include "shim_core.h"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "notifier_state.h"
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
using tier1::helpers::active_prefix_bytes;
using tier1::helpers::valid_boot_descriptor;
using tier1::helpers::valid_can_frame;
using tier1::helpers::valid_can_frame_batch;
using tier1::helpers::valid_can_status;
using tier1::helpers::valid_clock_state;
using tier1::helpers::valid_ds_state;
using tier1::helpers::valid_power_state;

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
// Test C2-3a: latest_power_state() is nullopt at construction time, before
// any poll(). Pins D-C2-6 default-init drift; cycle-1 test 4 is the
// clock_state analogue and lives in this same suite.
// ============================================================================
TEST(ShimCoreObservers, PowerStateNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_power_state().has_value());
}

// ============================================================================
// Test C3-3a: latest_ds_state() is nullopt at construction time, before
// any poll(). Pins D-C3-6 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, DsStateNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// ============================================================================
// Test C4-3a: latest_can_frame_batch() is nullopt at construction time,
// before any poll(). Pins D-C4-5 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, CanFrameBatchNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C5-3a: latest_can_status() is nullopt at construction time, before
// any poll(). Pins D-C5-5 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, CanStatusNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_can_status().has_value());
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
// Test 10: cycle-5 limit. Schemas other than clock_state, power_state,
// ds_state, can_frame_batch, can_status, none are rejected loudly.
// Crucially, the session counter must advance through the rejection so
// a fresh valid envelope at the next-expected sequence still works. The
// probe schema migrates one step forward each cycle; cycle 6 will land
// notifier_state and migrate this probe to the next still-unsupported
// schema (notifier_alarm_batch).
// ============================================================================
TEST(ShimCorePoll, RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // Step A — send a valid tick_boundary/notifier_state. notifier_state
  // is variable-size (per validator.cpp's expected_variable_payload_size
  // with header_size = offsetof(notifier_state, slots)). For count=0,
  // the active prefix is exactly the 8-byte header (4-byte count plus
  // 4-byte interior pad to slots-array alignment, since notifier_slot
  // has alignof == 8 due to its uint64_t trigger_time_us field).
  // bytes_of(notifier_state{}) would send the full 1+kMaxNotifiers*88
  // bytes and be rejected on the count↔length contract; using the
  // 8-byte header directly is the right pattern, mirroring cycle-4's
  // empty_batch_header for can_frame_batch. Cycle 6 will land
  // notifier_state and migrate this probe forward.
  constexpr std::size_t kNotifierStatePrefixBytes =
      offsetof(notifier_state, slots);
  static_assert(kNotifierStatePrefixBytes == 8,
                "notifier_state header pad must be 8; alignof "
                "notifier_slot is 8 (uint64_t trigger_time_us). If "
                "this static_assert fires, the struct layout changed "
                "and this comment plus the matching header_size in "
                "validator.cpp need updating. A wrong size here would "
                "surface as a framing-validation failure in the "
                "core.send below.");
  const std::array<std::uint8_t, kNotifierStatePrefixBytes> empty_notifier_prefix{};
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        empty_notifier_prefix,
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
// Test C2-1: post-connect tick_boundary/power_state envelope is byte-copied
// into the new latest_power_state_ cache slot. The clock_state slot must
// remain untouched (D-C2-1 cross-slot independence, viewed from a single
// power-state arrival).
// ============================================================================
TEST(ShimCorePoll, AcceptsPowerStateAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(state),
                        1'000));
  auto result = shim.poll();
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), state);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C2-2: cache replacement keeps only the most recent power_state.
// All three fields differ between the two values, so a "shim only updates
// some fields" bug fails on the second assertion.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedPowerStateUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto first = valid_power_state(12.0f, 1.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), first);

  const auto second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), second);
}

// ============================================================================
// Test C2-3b: latest_power_state() stays nullopt after a boot_ack envelope
// is accepted. The prerequisite ASSERT_TRUE on is_connected() must abort on
// failure; otherwise a silent boot_ack-handling regression would let the
// primary nullopt assertion pass trivially.
// ============================================================================
TEST(ShimCorePoll, PowerStateNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  // Drain shim's boot envelope and reply with boot_ack.
  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());  // prerequisite — see plan C2-3b
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
}

// ============================================================================
// Test C2-3c: latest_power_state() stays nullopt after a clock_state
// envelope is accepted (D-C2-1 viewed from "slot stays empty under
// unrelated traffic"). Prerequisite ASSERT_TRUE on latest_clock_state() is
// the abort gate that keeps the test honest.
// ============================================================================
TEST(ShimCorePoll, PowerStateNulloptAfterClockStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_clock_state(50'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());  // prerequisite
  EXPECT_FALSE(shim.latest_power_state().has_value());
}

// ============================================================================
// Test C2-4: a pre-boot-ack tick_boundary/power_state surfaces the session's
// expected_boot_ack_first error, NOT a dispatch-level unsupported_payload
// or a silent cache write. Mirrors cycle-1 test 9 with the schema swapped
// to power_state, pinning that the schema-specific dispatch does not
// short-circuit the session's ordering check (D-C2-3).
// ============================================================================
TEST(ShimCorePoll, RejectsPowerStateBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  // Inject a pre-boot-ack tick_boundary/power_state directly. No legitimate
  // peer can produce this state (a core-side protocol_session would refuse
  // to build a tick_boundary before sending boot_ack).
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::power_state,
                                          sizeof(power_state),
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const auto payload_bytes = bytes_of(valid_power_state());
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

  // Shim state untouched — neither cache populated, not connected.
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
}

// ============================================================================
// Test C2-5: clock_state and power_state caches are independently
// maintained. Interleaved sequence (clock → power → clock) catches both
// directions of cross-slot contamination.
// ============================================================================
TEST(ShimCorePoll, ClockAndPowerCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // Step 1: clock_state at sim_time_us = 100'000.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_first),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());

  // Step 2: power_state — clock slot must be unchanged.
  const auto power = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power),
                        150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), valid_power_state(12.5f, 2.0f, 6.8f));

  // Step 3: clock_state at 200'000 — power slot must be unchanged.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_second),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), valid_power_state(12.5f, 2.0f, 6.8f));  // unchanged
}

// ============================================================================
// Test C3-1: post-connect tick_boundary/ds_state envelope is byte-copied
// into the new latest_ds_state_ cache slot. The clock_state and
// power_state slots remain untouched (D-C3-1 cross-slot independence at
// first arrival).
// ============================================================================
TEST(ShimCorePoll, AcceptsDsStateAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(state),
                        1'000));
  auto result = shim.poll();
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), state);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C3-2: cache replacement keeps only the most recent ds_state. All
// seven distinguished fields differ between the two values, spanning the
// full 0..2376-byte struct layout, so a partial-copy bug at any practical
// prefix or suffix length fails byte-equality.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedDsStateUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto first = valid_ds_state(/*joystick0_axis_count=*/2,
                                    /*joystick0_axis_0_value=*/0.25f,
                                    /*control_bits=*/kControlAutonomous,
                                    /*station=*/alliance_station::blue_1,
                                    /*type=*/match_type::practice,
                                    /*match_number=*/7,
                                    /*match_time_seconds=*/3.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), first);

  const auto second = valid_ds_state(/*joystick0_axis_count=*/4,
                                     /*joystick0_axis_0_value=*/-0.75f,
                                     /*control_bits=*/kControlEnabled | kControlTest,
                                     /*station=*/alliance_station::red_3,
                                     /*type=*/match_type::elimination,
                                     /*match_number=*/15,
                                     /*match_time_seconds=*/45.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), second);
}

// ============================================================================
// Test C3-3b: latest_ds_state() stays nullopt after a boot_ack envelope
// is accepted. Mirrors C2-3b's structure for the new ds slot.
// ============================================================================
TEST(ShimCorePoll, DsStateNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());  // prerequisite — see plan C3-3b
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
}

// ============================================================================
// Test C3-3c: latest_ds_state() stays nullopt after a clock_state envelope
// is accepted (D-C3-1 viewed from "ds slot stays empty under clock_state
// traffic"). Distinct bug class from C3-3d.
// ============================================================================
TEST(ShimCorePoll, DsStateNulloptAfterClockStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_clock_state(50'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());  // prerequisite
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// ============================================================================
// Test C3-3d: latest_ds_state() stays nullopt after a power_state
// envelope is accepted. Distinct bug class from C3-3c (the power_state
// dispatch arm is separate code from the clock_state arm).
// ============================================================================
TEST(ShimCorePoll, DsStateNulloptAfterPowerStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_power_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());  // prerequisite
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// ============================================================================
// Test C3-4: a pre-boot-ack tick_boundary/ds_state surfaces the session's
// expected_boot_ack_first error, NOT a dispatch-level unsupported_payload
// or a silent cache write. Mirrors C2-4 with schema swapped to ds_state.
// ============================================================================
TEST(ShimCorePoll, RejectsDsStateBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  // Inject a pre-boot-ack tick_boundary/ds_state directly. No legitimate
  // peer can produce this state.
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::ds_state,
                                          sizeof(ds_state),
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const auto payload_bytes = bytes_of(valid_ds_state());
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

  // Lane preserved unchanged.
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, payload_bytes.size());

  // Shim state untouched — neither cache populated, not connected.
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
}

// ============================================================================
// Test C3-5: clock_state, power_state, and ds_state caches are
// independently maintained. Five-step interleaved scenario covers all
// four cross-population bug classes:
//   step 3 catches "ds-arm clobbers clock" + "ds-arm clobbers power"
//   step 4 catches "clock-arm clobbers ds"
//   step 5 catches "power-arm clobbers ds"
// All has_value() prerequisites before dereferences are ASSERT_TRUE per
// the no-shortcuts / no-false-green convention.
// ============================================================================
TEST(ShimCorePoll, ClockPowerAndDsCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // --- Step 1: clock_state arrival.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_first),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());

  // --- Step 2: power_state arrival. Clock unchanged.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_first),
                        150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  EXPECT_FALSE(shim.latest_ds_state().has_value());

  // --- Step 3: ds_state arrival. Clock and power unchanged
  // (catches "ds-arm clobbers clock" and "ds-arm clobbers power").
  const auto ds_value = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_value),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);  // unchanged
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_value);

  // --- Step 4: second clock_state. ds unchanged (catches
  // "clock-arm clobbers ds-slot").
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_second),
                        250'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);  // unchanged
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_value);  // unchanged

  // --- Step 5: second power_state. ds unchanged (catches
  // "power-arm clobbers ds-slot").
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_second),
                        300'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);  // updated
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_value);  // unchanged
}

// ============================================================================
// Test C4-1: post-connect tick_boundary/can_frame_batch envelope is
// byte-copied (active prefix only) into the new latest_can_frame_batch_
// cache slot. Sibling slots remain untouched (D-C4-1, D-C4-VARIABLE-SIZE).
// ============================================================================
TEST(ShimCorePoll, AcceptsCanFrameBatchAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 3> frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 8, 0xB0),
      valid_can_frame(0x300, 3000, 0, 0xC0),
  };
  const auto batch = valid_can_frame_batch(frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        1'000));
  auto result = shim.poll();
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), batch);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C4-1b: zero-count can_frame_batch (4-byte active prefix) is the
// boundary case for variable-size dispatch. The cache must accept it and
// hold a count=0 batch with all frame slots zero.
// ============================================================================
TEST(ShimCorePoll, AcceptsEmptyCanFrameBatch) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_can_frame_batch();  // count = 0
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(shim.latest_can_frame_batch()->count, 0u);
  EXPECT_EQ(*shim.latest_can_frame_batch(), valid_can_frame_batch());
}

// ============================================================================
// Test C4-2: latest-wins replacement on can_frame_batch. Both batches
// have the same count but distinct message_ids so an "append" or
// field-merge bug fails the second-batch assertion.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedCanFrameBatchUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 2> first_frames{
      valid_can_frame(0xAA, 100, 4, 0x10),
      valid_can_frame(0xBB, 200, 4, 0x20),
  };
  const auto first = valid_can_frame_batch(first_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), first);

  const std::array<can_frame, 2> second_frames{
      valid_can_frame(0xCC, 300, 4, 0x30),
      valid_can_frame(0xDD, 400, 4, 0x40),
  };
  const auto second = valid_can_frame_batch(second_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), second);
}

// ============================================================================
// Test C4-2b: shrinking-batch contract. After a 5-frame batch is replaced
// by a 2-frame batch, frames[2..4] in the cache must be byte-zero, not
// stale data from the prior batch (D-C4-2 zero-init-before-memcpy).
// Includes a byte-level memcmp on frames[2] to catch a partial-clear bug
// that leaves the prior frame's 3 padding bytes intact.
// ============================================================================
TEST(ShimCorePoll, ShrinkingBatchClearsTrailingFramesInCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 5> first_frames{
      valid_can_frame(0x10, 100, 4, 0x10),
      valid_can_frame(0x20, 200, 4, 0x20),
      valid_can_frame(0x30, 300, 4, 0x30),
      valid_can_frame(0x40, 400, 4, 0x40),
      valid_can_frame(0x50, 500, 4, 0x50),
  };
  const auto first = valid_can_frame_batch(first_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<can_frame, 2> second_frames{
      valid_can_frame(0xA0, 600, 4, 0xA0),
      valid_can_frame(0xB0, 700, 4, 0xB0),
  };
  const auto second = valid_can_frame_batch(second_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());

  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(shim.latest_can_frame_batch()->count, 2u);
  EXPECT_EQ(shim.latest_can_frame_batch()->frames[0], second_frames[0]);
  EXPECT_EQ(shim.latest_can_frame_batch()->frames[1], second_frames[1]);

  // Trailing frames must be zeroed, not stale from the first batch.
  const can_frame empty_frame{};
  EXPECT_EQ(shim.latest_can_frame_batch()->frames[2], empty_frame);
  EXPECT_EQ(shim.latest_can_frame_batch()->frames[3], empty_frame);
  EXPECT_EQ(shim.latest_can_frame_batch()->frames[4], empty_frame);

  // Byte-level guard on frames[2]: catches a partial field-clear that
  // leaves the prior frame's 3 trailing padding bytes intact while
  // zeroing the named fields. operator== alone would not catch that.
  EXPECT_EQ(std::memcmp(&shim.latest_can_frame_batch()->frames[2],
                        &empty_frame,
                        sizeof(can_frame)),
            0);
}

// ============================================================================
// Test C4-3b: latest_can_frame_batch() stays nullopt after a boot_ack
// envelope is accepted.
// ============================================================================
TEST(ShimCorePoll, CanFrameBatchNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C4-3c: latest_can_frame_batch() stays nullopt after a clock_state
// envelope (D-C4-1 from "cf slot stays empty under clock_state traffic").
// ============================================================================
TEST(ShimCorePoll, CanFrameBatchNulloptAfterClockStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_clock_state(50'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C4-3d: latest_can_frame_batch() stays nullopt after a power_state
// envelope.
// ============================================================================
TEST(ShimCorePoll, CanFrameBatchNulloptAfterPowerStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_power_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C4-3e: latest_can_frame_batch() stays nullopt after a ds_state
// envelope. The ds_state arm is the most likely source of a copy-paste
// regression into the new cf arm in source order.
// ============================================================================
TEST(ShimCorePoll, CanFrameBatchNulloptAfterDsStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C4-4: a pre-boot-ack tick_boundary/can_frame_batch surfaces the
// session's expected_boot_ack_first error. The injected envelope uses a
// minimal 4-byte count=0 active prefix so framing is valid (D-C4-3).
// ============================================================================
TEST(ShimCorePoll, RejectsCanFrameBatchBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  // Inject a pre-boot-ack tick_boundary/can_frame_batch directly. No
  // legitimate peer can produce this state. count=0 minimal prefix.
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::can_frame_batch,
                                          /*payload_bytes=*/sizeof(std::uint32_t),
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const std::array<std::uint8_t, sizeof(std::uint32_t)> empty_prefix{};
  manually_fill_lane(region.core_to_backend, injected_env, empty_prefix);

  auto result = shim_or->poll();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, shim_error_kind::receive_failed);
  ASSERT_TRUE(result.error().transport_error.has_value());
  EXPECT_EQ(result.error().transport_error->kind,
            tier1_transport_error_kind::session_rejected_envelope);
  ASSERT_TRUE(result.error().transport_error->session_failure.has_value());
  EXPECT_EQ(result.error().transport_error->session_failure->kind,
            session_error_kind::expected_boot_ack_first);

  // Lane preserved unchanged.
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, empty_prefix.size());

  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
}

// ============================================================================
// Test C4-5: all four caches are independently maintained. Seven-step
// interleaved scenario covers the six new cross-population bug classes:
//   step 4 catches "cf-arm clobbers clock/power/ds" (3 directions)
//   step 5 catches "clock-arm clobbers cf"
//   step 6 catches "power-arm clobbers cf"
//   step 7 catches "ds-arm clobbers cf"
// All has_value() prerequisites before dereferences are ASSERT_TRUE.
// ============================================================================
TEST(ShimCorePoll, AllFourCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // --- Step 1: clock_state arrival.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_first),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());

  // --- Step 2: power_state arrival.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_first),
                        150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());

  // --- Step 3: ds_state arrival.
  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_first),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());

  // --- Step 4: can_frame_batch arrival. Catches cf→clock, cf→power,
  // cf→ds clobber bugs.
  const std::array<can_frame, 2> cf_frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 2, 0xB0),
  };
  const auto cf_first = valid_can_frame_batch(cf_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_first),
                        250'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);  // unchanged
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);  // unchanged
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);

  // --- Step 5: second clock_state. Catches clock→cf clobber.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_second),
                        300'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);  // unchanged

  // --- Step 6: second power_state. Catches power→cf clobber.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_second),
                        350'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);  // updated
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);  // unchanged

  // --- Step 7: second ds_state. Catches ds→cf clobber.
  const auto ds_second = valid_ds_state(/*joystick0_axis_count=*/4,
                                        /*joystick0_axis_0_value=*/-0.25f,
                                        /*control_bits=*/kControlEnabled | kControlAutonomous,
                                        /*station=*/alliance_station::blue_2,
                                        /*type=*/match_type::elimination,
                                        /*match_number=*/99,
                                        /*match_time_seconds=*/30.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_second),
                        400'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);  // updated
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);  // unchanged
}

// ============================================================================
// Test C5-1: post-connect tick_boundary/can_status envelope is byte-copied
// into the new latest_can_status_ cache slot. Sibling slots remain
// untouched (D-C5-1).
// ============================================================================
TEST(ShimCorePoll, AcceptsCanStatusAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(state),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), state);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C5-2: cache replacement keeps only the most recent can_status. All
// five fields differ between the two values so a partial-update bug fails.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedCanStatusUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto first = valid_can_status(0.10f, 1, 2, 3, 4);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), first);

  const auto second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), second);
}

// ============================================================================
// Test C5-3b: latest_can_status() stays nullopt after a boot_ack envelope.
// ============================================================================
TEST(ShimCorePoll, CanStatusNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_can_status().has_value());
}

// ============================================================================
// Test C5-3c: latest_can_status() stays nullopt after a clock_state arrival.
// ============================================================================
TEST(ShimCorePoll, CanStatusNulloptAfterClockStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_clock_state(50'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
}

// ============================================================================
// Test C5-3d: latest_can_status() stays nullopt after a power_state arrival.
// ============================================================================
TEST(ShimCorePoll, CanStatusNulloptAfterPowerStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_power_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
}

// ============================================================================
// Test C5-3e: latest_can_status() stays nullopt after a ds_state arrival.
// ============================================================================
TEST(ShimCorePoll, CanStatusNulloptAfterDsStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
}

// ============================================================================
// Test C5-3f: latest_can_status() stays nullopt after a can_frame_batch
// arrival. Payload uses active_prefix_bytes (NOT bytes_of) — can_frame_batch
// is variable-size; bytes_of(batch) would fail framing on count↔length.
// ============================================================================
TEST(ShimCorePoll, CanStatusNulloptAfterCanFrameBatchIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_can_frame_batch();  // empty batch is fine
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
}

// ============================================================================
// Test C5-4: a pre-boot-ack tick_boundary/can_status surfaces the session's
// expected_boot_ack_first error. Mirrors C4-4 with schema swapped.
// ============================================================================
TEST(ShimCorePoll, RejectsCanStatusBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  // Inject a pre-boot-ack tick_boundary/can_status directly. No legitimate
  // peer can produce this state.
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::can_status,
                                          sizeof(can_status),
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const auto payload_bytes = bytes_of(valid_can_status());
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

  // Lane preserved unchanged.
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, payload_bytes.size());

  // All five cache slots remain nullopt.
  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim_or->latest_can_status().has_value());
}

// ============================================================================
// Test C5-5: all five caches are independently maintained. Nine-step
// interleaved scenario covers the eight new cross-population bug classes
// arising from the fifth slot:
//   step 5 catches "cs-arm clobbers clock/power/ds/cf" (4 directions)
//   step 6 catches "clock-arm clobbers cs"
//   step 7 catches "power-arm clobbers cs"
//   step 8 catches "ds-arm clobbers cs"
//   step 9 catches "cf-arm clobbers cs"
// can_frame_batch payloads use active_prefix_bytes (NOT bytes_of).
// ============================================================================
TEST(ShimCorePoll, AllFiveCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // --- Step 1: clock_state arrival.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_first),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());

  // --- Step 2: power_state arrival.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_first),
                        150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());

  // --- Step 3: ds_state arrival.
  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_first),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());

  // --- Step 4: can_frame_batch arrival. NOTE: active_prefix_bytes, not bytes_of.
  const std::array<can_frame, 2> cf_frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 2, 0xB0),
  };
  const auto cf_first = valid_can_frame_batch(cf_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_first),
                        250'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  EXPECT_FALSE(shim.latest_can_status().has_value());

  // --- Step 5: can_status arrival. Catches cs→clock/power/ds/cf (4 dirs).
  const auto cs_first = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(cs_first),
                        300'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);  // unchanged
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);  // unchanged
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);  // unchanged
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);

  // --- Step 6: second clock_state. Catches clock→cs.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_second),
                        350'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);  // unchanged

  // --- Step 7: second power_state. Catches power→cs.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_second),
                        400'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);  // updated
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);  // unchanged

  // --- Step 8: second ds_state. Catches ds→cs.
  const auto ds_second = valid_ds_state(/*joystick0_axis_count=*/4,
                                        /*joystick0_axis_0_value=*/-0.25f,
                                        /*control_bits=*/kControlEnabled | kControlAutonomous,
                                        /*station=*/alliance_station::blue_2,
                                        /*type=*/match_type::elimination,
                                        /*match_number=*/99,
                                        /*match_time_seconds=*/30.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_second),
                        450'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);  // updated
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);  // unchanged

  // --- Step 9: second can_frame_batch (single distinct frame). Catches cf→cs.
  // NOTE: active_prefix_bytes, not bytes_of.
  const std::array<can_frame, 1> cf_frames_second{
      valid_can_frame(0xCC, 5000, 4, 0xDD),
  };
  const auto cf_second = valid_can_frame_batch(cf_frames_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second),
                        500'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);  // updated
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);  // unchanged
}

// ============================================================================
// Test C5-6: determinism — two independent runs of an interleaved
// boot_ack + clock + power + ds + can_frame_batch + can_status + clock
// scenario produce byte-identical cached values in ALL FIVE slots. Replaces
// cycle-4's RepeatedRunsProduceByteIdenticalAllFourSlots. std::memcmp on
// ds_state and can_frame_batch pin padding-byte determinism (D-C3-7,
// D-C4-PADDING). can_status, clock_state, and power_state are padding-free
// so no memcmp companion is needed for them (D-C5-NO-PADDING).
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalAllFiveSlots) {
  auto run_scenario = [](tier1_shared_region& region) -> shim_core {
    tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
    auto shim = make_connected_shim(region, core);

    const auto first_clock = valid_clock_state(50'000);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::clock_state,
                          bytes_of(first_clock),
                          50'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto power = valid_power_state(12.5f, 2.0f, 6.8f);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::power_state,
                          bytes_of(power),
                          75'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto ds = valid_ds_state();
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::ds_state,
                          bytes_of(ds),
                          90'000));
    EXPECT_TRUE(shim.poll().has_value());

    const std::array<can_frame, 2> cf_frames{
        valid_can_frame(0x100, 1000, 4, 0xA0),
        valid_can_frame(0x200, 2000, 2, 0xB0),
    };
    const auto cf = valid_can_frame_batch(cf_frames);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::can_frame_batch,
                          active_prefix_bytes(cf),
                          95'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto cs = valid_can_status();
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::can_status,
                          bytes_of(cs),
                          97'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto second_clock = valid_clock_state(100'000);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::clock_state,
                          bytes_of(second_clock),
                          100'000));
    EXPECT_TRUE(shim.poll().has_value());

    return shim;
  };

  tier1_shared_region region_a{};
  tier1_shared_region region_b{};
  auto shim_a = run_scenario(region_a);
  auto shim_b = run_scenario(region_b);

  // 10 has_value() gates: 5 slots × 2 setups, all ASSERT_TRUE.
  ASSERT_TRUE(shim_a.latest_clock_state().has_value());
  ASSERT_TRUE(shim_a.latest_power_state().has_value());
  ASSERT_TRUE(shim_a.latest_ds_state().has_value());
  ASSERT_TRUE(shim_a.latest_can_frame_batch().has_value());
  ASSERT_TRUE(shim_a.latest_can_status().has_value());
  ASSERT_TRUE(shim_b.latest_clock_state().has_value());
  ASSERT_TRUE(shim_b.latest_power_state().has_value());
  ASSERT_TRUE(shim_b.latest_ds_state().has_value());
  ASSERT_TRUE(shim_b.latest_can_frame_batch().has_value());
  ASSERT_TRUE(shim_b.latest_can_status().has_value());

  EXPECT_EQ(*shim_a.latest_clock_state(), *shim_b.latest_clock_state());
  EXPECT_EQ(*shim_a.latest_power_state(), *shim_b.latest_power_state());
  EXPECT_EQ(*shim_a.latest_ds_state(), *shim_b.latest_ds_state());
  EXPECT_EQ(*shim_a.latest_can_frame_batch(), *shim_b.latest_can_frame_batch());
  EXPECT_EQ(*shim_a.latest_can_status(), *shim_b.latest_can_status());

  // Padding-byte determinism only on padding-bearing structs.
  EXPECT_EQ(std::memcmp(&*shim_a.latest_ds_state(),
                        &*shim_b.latest_ds_state(),
                        sizeof(ds_state)),
            0);
  EXPECT_EQ(std::memcmp(&*shim_a.latest_can_frame_batch(),
                        &*shim_b.latest_can_frame_batch(),
                        sizeof(can_frame_batch)),
            0);

  EXPECT_TRUE(shim_a.is_connected());
  EXPECT_TRUE(shim_b.is_connected());
}

}  // namespace robosim::backend::shim
