#include "shim_core.h"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
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
// active_prefix_bytes resolves via the production overloads in
// can_frame.h / notifier_state.h for those two schemas (extracted in
// cycle 10 per D-C10-EXTRACT-ACTIVE-PREFIX) and via test_helpers.h for
// notifier_alarm_batch / error_message_batch (which have no production
// outbound caller — sim-authoritative and not-yet-wired respectively).
using tier1::helpers::active_prefix_bytes;
using tier1::helpers::valid_boot_descriptor;
using tier1::helpers::valid_can_frame;
using tier1::helpers::valid_can_frame_batch;
using tier1::helpers::valid_can_status;
using tier1::helpers::valid_clock_state;
using tier1::helpers::valid_ds_state;
using tier1::helpers::valid_error_message;
using tier1::helpers::valid_error_message_batch;
using tier1::helpers::valid_notifier_alarm_batch;
using tier1::helpers::valid_notifier_alarm_event;
using tier1::helpers::valid_notifier_slot;
using tier1::helpers::valid_notifier_state;
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

// Cycle-9 helper. Drains the shim's outbound boot envelope from the
// core peer's inbound side, leaving backend_to_core empty so the
// shim's first post-boot send can take the lane. Mirrors the *first
// half* of make_connected_shim's drain step but does NOT send
// boot_ack — useful for C9-4 (D-C9-NO-CONNECT-GATE).
void drain_boot_only(tier1_endpoint& core) {
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
}

// Cycle-9 helper. Pulls one envelope from the shim's backend_to_core
// lane via the core peer and returns the receiver-owned tier1_message.
// The caller asserts on it. Used by every C9 test that inspects the
// published wire bytes.
tier1::tier1_message receive_from_shim(tier1_endpoint& core) {
  auto msg = core.try_receive();
  EXPECT_TRUE(msg.has_value());
  return std::move(*msg);
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
// Test C6-3a: latest_notifier_state() is nullopt at construction time,
// before any poll(). Pins D-C6-5 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, NotifierStateNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C7-3a: latest_notifier_alarm_batch() is nullopt at construction
// time, before any poll(). Pins D-C7-5 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, NotifierAlarmBatchNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C8-3a: latest_error_message_batch() is nullopt at construction
// time, before any poll(). Pins D-C8-5 default-init drift.
// ============================================================================
TEST(ShimCoreObservers, ErrorMessageBatchNulloptBeforeAnyPoll) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  const auto& shim = *shim_or;

  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
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
// Test 10 (the "unsupported payload schema" probe) was retired at
// cycle 8 (D-C8-PROBE-RETIRED in TEST_PLAN_CYCLE8.md). After cycle 8
// every per-tick payload schema is wired, so no probe schema remains
// to construct an unsupported-but-valid envelope from. The
// session-counter-advance-through-reject contract is structurally
// pinned by tests 10 across cycles 2–7 (each a different probe
// schema). The production-code unsupported_payload_schema arm is
// retained as a defensive forward-compat structural guard
// (D-C8-DEAD-BRANCH).
// ============================================================================

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
// Test C6-1: post-connect tick_boundary/notifier_state envelope is
// byte-copied (active prefix only) into the new latest_notifier_state_
// cache slot. Sibling slots remain untouched (D-C6-1, D-C6-VARIABLE-SIZE).
// ============================================================================
TEST(ShimCorePoll, AcceptsNotifierStateAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 3> slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
      valid_notifier_slot(300, 3, 1, 1, "gamma"),
  };
  const auto state = valid_notifier_state(slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(state),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), state);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C6-1b: zero-count notifier_state (8-byte active prefix) is the
// boundary case for variable-size dispatch.
// ============================================================================
TEST(ShimCorePoll, AcceptsEmptyNotifierState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_notifier_state();  // count = 0
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(state),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(shim.latest_notifier_state()->count, 0u);
  EXPECT_EQ(*shim.latest_notifier_state(), valid_notifier_state());
}

// ============================================================================
// Test C6-2: latest-wins replacement on notifier_state. Both batches have
// non-empty content with bit-distinct fields so a "shim writes zeros" or
// "shim partial-merges" bug fails.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedNotifierStateUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 2> first_slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
  };
  const auto first = valid_notifier_state(first_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), first);

  const std::array<notifier_slot, 3> second_slots{
      valid_notifier_slot(500, 5, 1, 1, "delta"),
      valid_notifier_slot(600, 6, 0, 0, "epsilon"),
      valid_notifier_slot(700, 7, 1, 0, "zeta"),
  };
  const auto second = valid_notifier_state(second_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), second);
}

// ============================================================================
// Test C6-2b: shrinking-batch contract for notifier_state. After a 5-slot
// batch is replaced by a 2-slot batch, slots[2..4] must be byte-zero
// (D-C6-VARIABLE-SIZE zero-init-before-memcpy). The byte-level memcmp on
// slots[2] uses sizeof(notifier_slot) (88 bytes) to catch any partial
// field-clear bug that leaves the 4 trailing pad bytes intact.
// ============================================================================
TEST(ShimCorePoll, ShrinkingNotifierStateClearsTrailingSlotsInCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 5> first_slots{
      valid_notifier_slot(10, 10, 1, 0, "s0"),
      valid_notifier_slot(20, 20, 0, 1, "s1"),
      valid_notifier_slot(30, 30, 1, 1, "s2"),
      valid_notifier_slot(40, 40, 0, 0, "s3"),
      valid_notifier_slot(50, 50, 1, 0, "s4"),
  };
  const auto first = valid_notifier_state(first_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_slot, 2> second_slots{
      valid_notifier_slot(100, 100, 1, 0, "new0"),
      valid_notifier_slot(200, 200, 0, 1, "new1"),
  };
  const auto second = valid_notifier_state(second_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());

  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(shim.latest_notifier_state()->count, 2u);
  EXPECT_EQ(shim.latest_notifier_state()->slots[0], second_slots[0]);
  EXPECT_EQ(shim.latest_notifier_state()->slots[1], second_slots[1]);

  const notifier_slot empty_slot{};
  EXPECT_EQ(shim.latest_notifier_state()->slots[2], empty_slot);
  EXPECT_EQ(shim.latest_notifier_state()->slots[3], empty_slot);
  EXPECT_EQ(shim.latest_notifier_state()->slots[4], empty_slot);

  // Byte-level guard on slots[2] over sizeof(notifier_slot) — catches
  // a partial field-clear that leaves the 4 trailing padding bytes
  // intact while zeroing the named fields.
  EXPECT_EQ(std::memcmp(&shim.latest_notifier_state()->slots[2],
                        &empty_slot,
                        sizeof(notifier_slot)),
            0);
}

// ============================================================================
// Test C6-3b: latest_notifier_state() stays nullopt after a boot_ack.
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-3c: latest_notifier_state() stays nullopt after clock_state.
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterClockStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-3d: latest_notifier_state() stays nullopt after power_state.
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterPowerStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-3e: latest_notifier_state() stays nullopt after ds_state.
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterDsStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-3f: latest_notifier_state() stays nullopt after can_frame_batch.
// Payload uses active_prefix_bytes (variable-size schema).
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterCanFrameBatchIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_can_frame_batch();  // empty
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-3g: latest_notifier_state() stays nullopt after can_status.
// ============================================================================
TEST(ShimCorePoll, NotifierStateNulloptAfterCanStatusIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-4: pre-boot-ack tick_boundary/notifier_state surfaces the
// session's expected_boot_ack_first error. Inject 8-byte count=0 active
// prefix directly via manually_fill_lane.
// ============================================================================
TEST(ShimCorePoll, RejectsNotifierStateBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  constexpr std::size_t kNotifierStatePrefixBytes =
      offsetof(notifier_state, slots);
  static_assert(kNotifierStatePrefixBytes == 8,
                "notifier_state header pad must be 8");
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::notifier_state,
                                          /*payload_bytes=*/kNotifierStatePrefixBytes,
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const std::array<std::uint8_t, kNotifierStatePrefixBytes> empty_prefix{};
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

  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, empty_prefix.size());

  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim_or->latest_can_status().has_value());
  EXPECT_FALSE(shim_or->latest_notifier_state().has_value());
}

// ============================================================================
// Test C6-5: all six caches are independently maintained. Eleven-step
// interleaved scenario covers the 10 new cross-population directions.
// can_frame_batch and notifier_state payloads use active_prefix_bytes.
// ============================================================================
TEST(ShimCorePoll, AllSixCachesAreIndependentlyMaintained) {
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
  EXPECT_FALSE(shim.latest_notifier_state().has_value());

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

  // --- Step 3: ds_state arrival.
  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::ds_state,
                        bytes_of(ds_first),
                        200'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);

  // --- Step 4: can_frame_batch arrival.
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
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);

  // --- Step 5: can_status arrival.
  const auto cs_first = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(cs_first),
                        300'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);
  EXPECT_FALSE(shim.latest_notifier_state().has_value());

  // --- Step 6: notifier_state arrival. Catches ns→clock/power/ds/cf/cs.
  const std::array<notifier_slot, 2> ns_slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
  };
  const auto ns_first = valid_notifier_state(ns_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(ns_first),
                        350'000));
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
  EXPECT_EQ(*shim.latest_can_status(), cs_first);  // unchanged
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);

  // --- Step 7: second clock_state. Catches clock→ns.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(clock_second),
                        400'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged

  // --- Step 8: second power_state. Catches power→ns.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(power_second),
                        450'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged

  // --- Step 9: second ds_state. Catches ds→ns.
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
                        500'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged

  // --- Step 10: second can_frame_batch. Catches cf→ns.
  const std::array<can_frame, 1> cf_frames_second{
      valid_can_frame(0xCC, 5000, 4, 0xDD),
  };
  const auto cf_second = valid_can_frame_batch(cf_frames_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second),
                        550'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged

  // --- Step 11: second can_status. Catches cs→ns.
  const auto cs_second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(cs_second),
                        600'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_second);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged
}

// ============================================================================
// Test C7-1: post-connect tick_boundary/notifier_alarm_batch envelope
// is byte-copied (active prefix only) into the new
// latest_notifier_alarm_batch_ cache slot. Sibling slots remain
// untouched (D-C7-1, D-C7-VARIABLE-SIZE).
// ============================================================================
TEST(ShimCorePoll, AcceptsNotifierAlarmBatchAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_alarm_event, 3> events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
      valid_notifier_alarm_event(300, 3),
  };
  const auto batch = valid_notifier_alarm_batch(events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(batch),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), batch);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C7-1b: zero-count notifier_alarm_batch (8-byte active prefix).
// ============================================================================
TEST(ShimCorePoll, AcceptsEmptyNotifierAlarmBatch) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_notifier_alarm_batch();  // count = 0
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(batch),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->count, 0u);
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), valid_notifier_alarm_batch());
}

// ============================================================================
// Test C7-2: latest-wins replacement on notifier_alarm_batch. Both
// batches non-empty, bit-distinct fields, different counts.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedNotifierAlarmBatchUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_alarm_event, 2> first_events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
  };
  const auto first = valid_notifier_alarm_batch(first_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), first);

  const std::array<notifier_alarm_event, 3> second_events{
      valid_notifier_alarm_event(500, 5),
      valid_notifier_alarm_event(600, 6),
      valid_notifier_alarm_event(700, 7),
  };
  const auto second = valid_notifier_alarm_batch(second_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), second);
}

// ============================================================================
// Test C7-2b: shrinking-batch contract for notifier_alarm_batch. After
// 5→2 shrink, events[2..4] must be byte-zero. The byte-level memcmp on
// events[2] over sizeof(notifier_alarm_event) is defensive (the schema
// has a NAMED reserved_pad field, no implicit padding); kept for
// consistency with cycle-4/6 family pattern.
// ============================================================================
TEST(ShimCorePoll, ShrinkingNotifierAlarmBatchClearsTrailingEventsInCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_alarm_event, 5> first_events{
      valid_notifier_alarm_event(10, 10),
      valid_notifier_alarm_event(20, 20),
      valid_notifier_alarm_event(30, 30),
      valid_notifier_alarm_event(40, 40),
      valid_notifier_alarm_event(50, 50),
  };
  const auto first = valid_notifier_alarm_batch(first_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_alarm_event, 2> second_events{
      valid_notifier_alarm_event(100, 100),
      valid_notifier_alarm_event(200, 200),
  };
  const auto second = valid_notifier_alarm_batch(second_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());

  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->count, 2u);
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->events[0], second_events[0]);
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->events[1], second_events[1]);

  const notifier_alarm_event empty_event{};
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->events[2], empty_event);
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->events[3], empty_event);
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->events[4], empty_event);

  EXPECT_EQ(std::memcmp(&shim.latest_notifier_alarm_batch()->events[2],
                        &empty_event,
                        sizeof(notifier_alarm_event)),
            0);
}

// ============================================================================
// Test C7-3b: latest_notifier_alarm_batch() stays nullopt after boot_ack.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  auto core = make_core(region);

  ASSERT_TRUE(core.try_receive().has_value());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, kBootSimTime));

  ASSERT_TRUE(shim_or->poll().has_value());
  ASSERT_TRUE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3c: latest_notifier_alarm_batch() stays nullopt after clock_state.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterClockStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3d: latest_notifier_alarm_batch() stays nullopt after power_state.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterPowerStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3e: latest_notifier_alarm_batch() stays nullopt after ds_state.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterDsStateIsAccepted) {
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
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3f: latest_notifier_alarm_batch() stays nullopt after
// can_frame_batch. Variable-size payload uses active_prefix_bytes.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterCanFrameBatchIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_can_frame_batch();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3g: latest_notifier_alarm_batch() stays nullopt after can_status.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterCanStatusIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_status,
                        bytes_of(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-3h: latest_notifier_alarm_batch() stays nullopt after
// notifier_state. Variable-size payload uses active_prefix_bytes.
// ============================================================================
TEST(ShimCorePoll, NotifierAlarmBatchNulloptAfterNotifierStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto state = valid_notifier_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(state),
                        50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-4: pre-boot-ack tick_boundary/notifier_alarm_batch surfaces
// expected_boot_ack_first. Inject 8-byte count=0 active prefix.
// ============================================================================
TEST(ShimCorePoll, RejectsNotifierAlarmBatchBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  constexpr std::size_t kPrefixBytes = offsetof(notifier_alarm_batch, events);
  static_assert(kPrefixBytes == 8, "notifier_alarm_batch header pad is 8");
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::notifier_alarm_batch,
                                          /*payload_bytes=*/kPrefixBytes,
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const std::array<std::uint8_t, kPrefixBytes> empty_prefix{};
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

  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, empty_prefix.size());

  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim_or->latest_can_status().has_value());
  EXPECT_FALSE(shim_or->latest_notifier_state().has_value());
  EXPECT_FALSE(shim_or->latest_notifier_alarm_batch().has_value());
}

// ============================================================================
// Test C7-5: all seven caches are independently maintained. 13-step
// interleaved scenario covering 12 new cross-population directions.
// ============================================================================
TEST(ShimCorePoll, AllSevenCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // --- Steps 1-6: populate all six existing slots.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::clock_state,
                        bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::power_state,
                        bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state,
                        bytes_of(ds_first), 200'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<can_frame, 2> cf_frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 2, 0xB0),
  };
  const auto cf_first = valid_can_frame_batch(cf_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_frame_batch,
                        active_prefix_bytes(cf_first), 250'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto cs_first = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_status,
                        bytes_of(cs_first), 300'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_slot, 2> ns_slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
  };
  const auto ns_first = valid_notifier_state(ns_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::notifier_state,
                        active_prefix_bytes(ns_first), 350'000));
  ASSERT_TRUE(shim.poll().has_value());

  // --- Step 7: notifier_alarm_batch. Catches ab→{clock,power,ds,cf,cs,ns}.
  const std::array<notifier_alarm_event, 2> ab_events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
  };
  const auto ab_first = valid_notifier_alarm_batch(ab_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(ab_first), 400'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 8: clock→ab.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::clock_state,
                        bytes_of(clock_second), 450'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 9: power→ab.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::power_state,
                        bytes_of(power_second), 500'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 10: ds→ab.
  const auto ds_second = valid_ds_state(/*joystick0_axis_count=*/4,
                                        /*joystick0_axis_0_value=*/-0.25f,
                                        /*control_bits=*/kControlEnabled | kControlAutonomous,
                                        /*station=*/alliance_station::blue_2,
                                        /*type=*/match_type::elimination,
                                        /*match_number=*/99,
                                        /*match_time_seconds=*/30.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state,
                        bytes_of(ds_second), 550'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 11: cf→ab.
  const std::array<can_frame, 1> cf_frames_second{
      valid_can_frame(0xCC, 5000, 4, 0xDD),
  };
  const auto cf_second = valid_can_frame_batch(cf_frames_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second), 600'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 12: cs→ab.
  const auto cs_second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_status,
                        bytes_of(cs_second), 650'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 13: ns→ab.
  const std::array<notifier_slot, 3> ns_slots_second{
      valid_notifier_slot(500, 5, 1, 1, "delta"),
      valid_notifier_slot(600, 6, 0, 0, "epsilon"),
      valid_notifier_slot(700, 7, 1, 0, "zeta"),
  };
  const auto ns_second = valid_notifier_state(ns_slots_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::notifier_state,
                        active_prefix_bytes(ns_second), 700'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);
}

// ============================================================================
// Cycle 8 — error_message_batch: the 8th and last per-tick payload schema.
// Both error_message and error_message_batch use NAMED reserved_pad
// fields rather than implicit C++ padding (see error_message.h:42, :62),
// so the schema has zero implicit padding bytes and operator== covers
// every byte. Per D-C8-PADDING-FREE, C8-6's determinism replay does NOT
// add a memcmp companion for error_message_batch.
// ============================================================================

// ============================================================================
// Test C8-1: post-connect tick_boundary/error_message_batch envelope is
// accepted, byte-copied via active-prefix length into the
// latest_error_message_batch_ cache slot. Sibling slots remain
// untouched (D-C8-1, D-C8-VARIABLE-SIZE).
// ============================================================================
TEST(ShimCorePoll, AcceptsErrorMessageBatchAndCachesByteEqualValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 3> messages{
      valid_error_message(/*error_code=*/100, /*severity=*/1,
                          /*is_lv_code=*/0, /*print_msg=*/1,
                          /*truncation_flags=*/0,
                          "first detail", "first location",
                          "first call stack"),
      valid_error_message(200, 0, 1, 0, kErrorTruncDetails,
                          "second detail", "second location",
                          "second call stack"),
      valid_error_message(300, 1, 0, 1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail", "third location",
                          "third call stack"),
  };
  const auto batch = valid_error_message_batch(messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(batch),
                        1'000));

  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), batch);

  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C8-1b: zero-count error_message_batch (8-byte active prefix:
// 4-byte count + 4-byte named reserved_pad). The same payload that
// served as the cycle-7 reject probe is now a positive arrival.
// ============================================================================
TEST(ShimCorePoll, AcceptsEmptyErrorMessageBatch) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto batch = valid_error_message_batch();  // count = 0
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(batch),
                        1'000));

  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(shim.latest_error_message_batch()->count, 0u);
  EXPECT_EQ(*shim.latest_error_message_batch(),
            valid_error_message_batch());
}

// ============================================================================
// Test C8-2: latest-wins replacement on error_message_batch. Both
// arrivals are non-empty and bit-distinct in every named field.
// ============================================================================
TEST(ShimCorePoll, LatestWinsForRepeatedErrorMessageBatchUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 2> first_messages{
      valid_error_message(10, 1, 0, 1, 0,
                          "alpha details", "alpha location",
                          "alpha stack"),
      valid_error_message(20, 0, 1, 0, kErrorTruncDetails,
                          "beta details", "beta location",
                          "beta stack"),
  };
  const auto first = valid_error_message_batch(first_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), first);

  const std::array<error_message, 3> second_messages{
      valid_error_message(50, 1, 1, 1, kErrorTruncLocation,
                          "delta details", "delta location",
                          "delta stack"),
      valid_error_message(60, 0, 0, 0,
                          kErrorTruncDetails | kErrorTruncCallStack,
                          "epsilon details", "epsilon location",
                          "epsilon stack"),
      valid_error_message(70, 1, 0, 1, kErrorTruncCallStack,
                          "zeta details", "zeta location",
                          "zeta stack"),
  };
  const auto second = valid_error_message_batch(second_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), second);
}

// ============================================================================
// Test C8-2b: shrinking-batch contract for error_message_batch. After
// a 5-message batch is replaced by a 2-message batch, messages[2..4]
// must be error_message{} (zero-init-before-write per
// D-C8-VARIABLE-SIZE). The byte-level memcmp targets the trailing-
// cleared slice; it pins the same property that operator== already
// pins (error_message has no implicit padding) — kept for consistency
// with the cycle-4/6/7 family pattern. Distinct purpose from C8-6's
// full-struct memcmp omission.
// ============================================================================
TEST(ShimCorePoll, ShrinkingErrorMessageBatchClearsTrailingMessagesInCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 5> first_messages{
      valid_error_message(1, 1, 0, 1, 0, "m1d", "m1l", "m1c"),
      valid_error_message(2, 0, 1, 0, kErrorTruncDetails,
                          "m2d", "m2l", "m2c"),
      valid_error_message(3, 1, 1, 1, kErrorTruncLocation,
                          "m3d", "m3l", "m3c"),
      valid_error_message(4, 0, 0, 1, kErrorTruncCallStack,
                          "m4d", "m4l", "m4c"),
      valid_error_message(5, 1, 0, 0,
                          kErrorTruncDetails | kErrorTruncLocation,
                          "m5d", "m5l", "m5c"),
  };
  const auto first = valid_error_message_batch(first_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(first),
                        1'000));
  ASSERT_TRUE(shim.poll().has_value());

  // Second batch is the first 2 messages of the first batch; any
  // mismatch in the cache slot resides exclusively in messages[2..4].
  const std::array<error_message, 2> second_messages{
      first_messages[0],
      first_messages[1],
  };
  const auto second = valid_error_message_batch(second_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(second),
                        2'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());

  const auto& cached = *shim.latest_error_message_batch();
  EXPECT_EQ(cached.count, 2u);
  EXPECT_EQ(cached.messages[0], first_messages[0]);
  EXPECT_EQ(cached.messages[1], first_messages[1]);

  const error_message empty_message{};
  EXPECT_EQ(cached.messages[2], empty_message);
  EXPECT_EQ(cached.messages[3], empty_message);
  EXPECT_EQ(cached.messages[4], empty_message);

  EXPECT_EQ(std::memcmp(&cached.messages[2],
                        &empty_message,
                        sizeof(error_message)),
            0);
}

// ============================================================================
// Test C8-3b: latest_error_message_batch() stays nullopt after boot_ack.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterBootAckIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  EXPECT_TRUE(shim.is_connected());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3c: latest_error_message_batch() stays nullopt after clock_state.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterClockStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::clock_state,
                        bytes_of(valid_clock_state(123'456)), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3d: latest_error_message_batch() stays nullopt after power_state.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterPowerStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::power_state,
                        bytes_of(valid_power_state(12.5f, 2.0f, 6.8f)), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3e: latest_error_message_batch() stays nullopt after ds_state.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterDsStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state,
                        bytes_of(valid_ds_state()), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3f: latest_error_message_batch() stays nullopt after
// can_frame_batch.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterCanFrameBatchIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 1> frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
  };
  const auto cf = valid_can_frame_batch(frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_frame_batch,
                        active_prefix_bytes(cf), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3g: latest_error_message_batch() stays nullopt after can_status.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterCanStatusIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_status,
                        bytes_of(valid_can_status()), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3h: latest_error_message_batch() stays nullopt after notifier_state.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterNotifierStateIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 1> slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
  };
  const auto ns = valid_notifier_state(slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::notifier_state,
                        active_prefix_bytes(ns), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-3i: latest_error_message_batch() stays nullopt after
// notifier_alarm_batch.
// ============================================================================
TEST(ShimCorePoll, ErrorMessageBatchNulloptAfterNotifierAlarmBatchIsAccepted) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_alarm_event, 1> events{
      valid_notifier_alarm_event(100, 1),
  };
  const auto nab = valid_notifier_alarm_batch(events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(nab), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  EXPECT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-4: pre-boot-ack tick_boundary/error_message_batch surfaces
// expected_boot_ack_first. Inject 8-byte count=0 active prefix
// (offsetof(error_message_batch, messages) == 8: 4-byte count +
// 4-byte named reserved_pad).
// ============================================================================
TEST(ShimCorePoll, RejectsErrorMessageBatchBeforeBootAckPreservingLane) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());

  constexpr std::size_t kPrefixBytes = offsetof(error_message_batch, messages);
  static_assert(kPrefixBytes == 8,
                "error_message_batch header is 4-byte count + 4-byte "
                "named reserved_pad = 8 bytes total. If this fires, "
                "the schema layout in error_message.h changed.");
  const auto injected_env = make_envelope(envelope_kind::tick_boundary,
                                          schema_id::error_message_batch,
                                          /*payload_bytes=*/kPrefixBytes,
                                          /*sequence=*/0,
                                          direction::core_to_backend);
  const std::array<std::uint8_t, kPrefixBytes> empty_prefix{};
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

  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.envelope, injected_env);
  EXPECT_EQ(region.core_to_backend.payload_bytes, empty_prefix.size());

  EXPECT_FALSE(shim_or->is_connected());
  EXPECT_FALSE(shim_or->latest_clock_state().has_value());
  EXPECT_FALSE(shim_or->latest_power_state().has_value());
  EXPECT_FALSE(shim_or->latest_ds_state().has_value());
  EXPECT_FALSE(shim_or->latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim_or->latest_can_status().has_value());
  EXPECT_FALSE(shim_or->latest_notifier_state().has_value());
  EXPECT_FALSE(shim_or->latest_notifier_alarm_batch().has_value());
  EXPECT_FALSE(shim_or->latest_error_message_batch().has_value());
}

// ============================================================================
// Test C8-5: all eight caches are independently maintained. 15-step
// interleaved scenario covering 14 new cross-population directions
// involving error_message_batch (7 ab→prior + 7 prior→ab). Each
// re-write step (9-15) uses bit-distinct values from the initial
// population so a stale-read bug is detectable, and asserts the
// error_message_batch slot is unchanged from step 8.
// ============================================================================
TEST(ShimCorePoll, AllEightCachesAreIndependentlyMaintained) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  // --- Steps 1-7: populate all seven existing slots.
  const auto clock_first = valid_clock_state(100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::clock_state,
                        bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::power_state,
                        bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state,
                        bytes_of(ds_first), 200'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<can_frame, 2> cf_frames{
      valid_can_frame(0x100, 1000, 4, 0xA0),
      valid_can_frame(0x200, 2000, 2, 0xB0),
  };
  const auto cf_first = valid_can_frame_batch(cf_frames);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_frame_batch,
                        active_prefix_bytes(cf_first), 250'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto cs_first = valid_can_status();
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_status,
                        bytes_of(cs_first), 300'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_slot, 2> ns_slots{
      valid_notifier_slot(100, 1, 1, 0, "alpha"),
      valid_notifier_slot(200, 2, 0, 1, "beta"),
  };
  const auto ns_first = valid_notifier_state(ns_slots);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::notifier_state,
                        active_prefix_bytes(ns_first), 350'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_alarm_event, 2> ab_events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
  };
  const auto ab_first = valid_notifier_alarm_batch(ab_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(ab_first), 400'000));
  ASSERT_TRUE(shim.poll().has_value());

  // --- Step 8: error_message_batch. Catches emb→{clock,power,ds,cf,cs,ns,nab}.
  const std::array<error_message, 2> emb_first_messages{
      valid_error_message(11, 1, 0, 1, 0,
                          "first batch alpha details",
                          "first batch alpha location",
                          "first batch alpha stack"),
      valid_error_message(22, 0, 1, 0, kErrorTruncDetails,
                          "first batch beta details",
                          "first batch beta location",
                          "first batch beta stack"),
  };
  const auto emb_first = valid_error_message_batch(emb_first_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(emb_first), 450'000));
  ASSERT_TRUE(shim.poll().has_value());

  // Comprehensive snapshot: all 7 prior slots unchanged, emb populated.
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), clock_first);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_first);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_first);
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_first);
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 9: clock→emb. Bit-distinct re-write (200'000 vs 100'000).
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::clock_state,
                        bytes_of(clock_second), 500'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), clock_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 10: power→emb.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::power_state,
                        bytes_of(power_second), 550'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 11: ds→emb.
  const auto ds_second = valid_ds_state(/*joystick0_axis_count=*/4,
                                        /*joystick0_axis_0_value=*/-0.25f,
                                        /*control_bits=*/kControlEnabled | kControlAutonomous,
                                        /*station=*/alliance_station::blue_2,
                                        /*type=*/match_type::elimination,
                                        /*match_number=*/99,
                                        /*match_time_seconds=*/30.0);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state,
                        bytes_of(ds_second), 600'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_ds_state().has_value());
  EXPECT_EQ(*shim.latest_ds_state(), ds_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 12: cf→emb.
  const std::array<can_frame, 1> cf_frames_second{
      valid_can_frame(0xCC, 5000, 4, 0xDD),
  };
  const auto cf_second = valid_can_frame_batch(cf_frames_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second), 650'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 13: cs→emb.
  const auto cs_second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::can_status,
                        bytes_of(cs_second), 700'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), cs_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 14: ns→emb.
  const std::array<notifier_slot, 3> ns_slots_second{
      valid_notifier_slot(500, 5, 1, 1, "delta"),
      valid_notifier_slot(600, 6, 0, 0, "epsilon"),
      valid_notifier_slot(700, 7, 1, 0, "zeta"),
  };
  const auto ns_second = valid_notifier_state(ns_slots_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::notifier_state,
                        active_prefix_bytes(ns_second), 750'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 15: nab→emb.
  const std::array<notifier_alarm_event, 3> ab_events_second{
      valid_notifier_alarm_event(500, 5),
      valid_notifier_alarm_event(600, 6),
      valid_notifier_alarm_event(700, 7),
  };
  const auto ab_second = valid_notifier_alarm_batch(ab_events_second);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(ab_second), 800'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);
}

// ============================================================================
// Test C8-6: determinism — two independent runs of an interleaved
// scenario produce byte-identical cached values in ALL EIGHT slots.
// Replaces the landed C7-6 (which was named AllSevenSlots but covered
// only six — notifier_alarm_batch was inadvertently absent from the
// scenario, gates, operator==, and memcmp companions). C8-6 closes
// that pre-existing determinism gap and extends to the new 8th slot.
// 16 has_value() gates (8 slots × 2 setups). 8 operator== checks. 4
// memcmp companions on the four padding-bearing schemas (ds_state,
// can_frame_batch, notifier_state, notifier_alarm_batch). NO memcmp
// on error_message_batch per D-C8-PADDING-FREE — its named
// reserved_pad fields mean operator== covers every byte already.
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalAllEightSlots) {
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

    const std::array<notifier_slot, 3> ns_slots{
        valid_notifier_slot(100, 1, 1, 0, "alpha"),
        valid_notifier_slot(200, 2, 0, 1, "beta"),
        valid_notifier_slot(300, 3, 1, 1, "gamma"),
    };
    const auto ns = valid_notifier_state(ns_slots);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::notifier_state,
                          active_prefix_bytes(ns),
                          98'000));
    EXPECT_TRUE(shim.poll().has_value());

    const std::array<notifier_alarm_event, 3> nab_events{
        valid_notifier_alarm_event(150, 1),
        valid_notifier_alarm_event(250, 2),
        valid_notifier_alarm_event(350, 3),
    };
    const auto nab = valid_notifier_alarm_batch(nab_events);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::notifier_alarm_batch,
                          active_prefix_bytes(nab),
                          99'000));
    EXPECT_TRUE(shim.poll().has_value());

    const std::array<error_message, 3> emb_messages{
        valid_error_message(101, 1, 0, 1, 0,
                            "det1", "loc1", "stk1"),
        valid_error_message(202, 0, 1, 0, kErrorTruncDetails,
                            "det2", "loc2", "stk2"),
        valid_error_message(303, 1, 0, 1,
                            kErrorTruncLocation | kErrorTruncCallStack,
                            "det3", "loc3", "stk3"),
    };
    const auto emb = valid_error_message_batch(emb_messages);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::error_message_batch,
                          active_prefix_bytes(emb),
                          99'500));
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

  // 16 has_value() gates: 8 slots × 2 setups, all ASSERT_TRUE.
  ASSERT_TRUE(shim_a.latest_clock_state().has_value());
  ASSERT_TRUE(shim_a.latest_power_state().has_value());
  ASSERT_TRUE(shim_a.latest_ds_state().has_value());
  ASSERT_TRUE(shim_a.latest_can_frame_batch().has_value());
  ASSERT_TRUE(shim_a.latest_can_status().has_value());
  ASSERT_TRUE(shim_a.latest_notifier_state().has_value());
  ASSERT_TRUE(shim_a.latest_notifier_alarm_batch().has_value());
  ASSERT_TRUE(shim_a.latest_error_message_batch().has_value());
  ASSERT_TRUE(shim_b.latest_clock_state().has_value());
  ASSERT_TRUE(shim_b.latest_power_state().has_value());
  ASSERT_TRUE(shim_b.latest_ds_state().has_value());
  ASSERT_TRUE(shim_b.latest_can_frame_batch().has_value());
  ASSERT_TRUE(shim_b.latest_can_status().has_value());
  ASSERT_TRUE(shim_b.latest_notifier_state().has_value());
  ASSERT_TRUE(shim_b.latest_notifier_alarm_batch().has_value());
  ASSERT_TRUE(shim_b.latest_error_message_batch().has_value());

  EXPECT_EQ(*shim_a.latest_clock_state(), *shim_b.latest_clock_state());
  EXPECT_EQ(*shim_a.latest_power_state(), *shim_b.latest_power_state());
  EXPECT_EQ(*shim_a.latest_ds_state(), *shim_b.latest_ds_state());
  EXPECT_EQ(*shim_a.latest_can_frame_batch(), *shim_b.latest_can_frame_batch());
  EXPECT_EQ(*shim_a.latest_can_status(), *shim_b.latest_can_status());
  EXPECT_EQ(*shim_a.latest_notifier_state(), *shim_b.latest_notifier_state());
  EXPECT_EQ(*shim_a.latest_notifier_alarm_batch(),
            *shim_b.latest_notifier_alarm_batch());
  EXPECT_EQ(*shim_a.latest_error_message_batch(),
            *shim_b.latest_error_message_batch());

  // Padding-byte determinism on the four padding-bearing schemas.
  EXPECT_EQ(std::memcmp(&*shim_a.latest_ds_state(),
                        &*shim_b.latest_ds_state(),
                        sizeof(ds_state)),
            0);
  EXPECT_EQ(std::memcmp(&*shim_a.latest_can_frame_batch(),
                        &*shim_b.latest_can_frame_batch(),
                        sizeof(can_frame_batch)),
            0);
  // notifier_state has 132 implicit padding bytes (4 interior count→slots
  // pad + 32 × 4 per-slot trailing pad).
  EXPECT_EQ(std::memcmp(&*shim_a.latest_notifier_state(),
                        &*shim_b.latest_notifier_state(),
                        sizeof(notifier_state)),
            0);
  // notifier_alarm_batch has 4 implicit padding bytes (the interior
  // count→events pad). C7-6 was supposed to pin this but the landed
  // implementation omitted both the schema and this memcmp; C8-6
  // corrects that gap (see test plan C8-R6).
  EXPECT_EQ(std::memcmp(&*shim_a.latest_notifier_alarm_batch(),
                        &*shim_b.latest_notifier_alarm_batch(),
                        sizeof(notifier_alarm_batch)),
            0);
  // No memcmp on error_message_batch — D-C8-PADDING-FREE: the schema
  // has zero implicit C++ padding (both reserved_pad fields are
  // named), so the operator== check above is byte-equivalent.

  EXPECT_TRUE(shim_a.is_connected());
  EXPECT_TRUE(shim_b.is_connected());
}

// ============================================================================
// Cycle 9 — outbound can_frame_batch (`send_can_frame_batch`).
// First post-boot outbound surface; see TEST_PLAN_CYCLE9.md.
// ============================================================================

// ============================================================================
// Test C9-1: send_can_frame_batch publishes a tick_boundary envelope
// carrying exactly the active prefix of the input batch (NOT
// sizeof(can_frame_batch)). Pins D-C9-ACTIVE-PREFIX-OUT and the
// first-post-boot sequence == 1 contract.
// ============================================================================
TEST(ShimCoreSend, PublishesCanFrameBatchAsTickBoundaryWithActivePrefixWireBytes) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 3> frames{
      valid_can_frame(/*message_id=*/0x101, /*timestamp_us=*/1'000,
                      /*data_size=*/4, /*fill_byte=*/0xA1),
      valid_can_frame(0x202, 2'000, 8, 0xB2),
      valid_can_frame(0x303, 3'000, 0, 0xC3)};
  const auto batch = valid_can_frame_batch(frames);

  auto sent = shim.send_can_frame_batch(batch, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::can_frame_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);  // boot took 0.
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 64u);  // 4 + 3*20.
  ASSERT_EQ(msg.payload.size(), 64u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &batch, 64), 0);
}

// ============================================================================
// Test C9-1b: an empty (count=0) batch is a legal publish; the active
// prefix is the 4-byte count word. Pins D-C9-EMPTY-BATCH-OUT.
// ============================================================================
TEST(ShimCoreSend, PublishesEmptyCanFrameBatchAsHeaderOnlyTickBoundary) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto empty_batch = valid_can_frame_batch();

  auto sent = shim.send_can_frame_batch(empty_batch, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::can_frame_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 4u);  // count word only.
  ASSERT_EQ(msg.payload.size(), 4u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &empty_batch, 4), 0);
}

// ============================================================================
// Test C9-2: a second send advances the outbound sequence counter to 2.
// Different counts (2 then 4) catch a "sequence advances by frame count"
// bug. Pins D-C9-SEQUENCE-ADVANCE positive arm.
// ============================================================================
TEST(ShimCoreSend, SecondSendAdvancesOutboundSequenceToTwo) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 2> first_frames{
      valid_can_frame(0x111, 1'000, 4, 0xAA),
      valid_can_frame(0x222, 2'000, 8, 0xBB)};
  const auto first = valid_can_frame_batch(first_frames);

  ASSERT_TRUE(shim.send_can_frame_batch(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 44u);  // 4 + 2*20.
  ASSERT_EQ(first_msg.payload.size(), 44u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 44), 0);

  const std::array<can_frame, 4> second_frames{
      valid_can_frame(0x333, 3'000, 4, 0xCC),
      valid_can_frame(0x444, 4'000, 8, 0xDD),
      valid_can_frame(0x555, 5'000, 0, 0xEE),
      valid_can_frame(0x666, 6'000, 6, 0xFF)};
  const auto second = valid_can_frame_batch(second_frames);

  ASSERT_TRUE(shim.send_can_frame_batch(second, /*sim_time_us=*/200'000).has_value());
  const auto second_msg = receive_from_shim(core);
  EXPECT_EQ(second_msg.envelope.sequence, 2u);
  EXPECT_EQ(second_msg.envelope.sim_time_us, 200'000u);
  EXPECT_EQ(second_msg.envelope.payload_bytes, 84u);  // 4 + 4*20.
  ASSERT_EQ(second_msg.payload.size(), 84u);
  EXPECT_EQ(std::memcmp(second_msg.payload.data(), &second, 84), 0);
}

// ============================================================================
// Test C9-3: lane_busy on send is rejected with the underlying
// transport diagnostic preserved, AND the outbound sequence counter
// is NOT consumed. Pins D-C9-SEQUENCE-ADVANCE negative arm + D-C9-
// WRAPPED-SEND-ERROR for lane_busy.
// ============================================================================
TEST(ShimCoreSend, LaneBusySendIsRejectedAndPreservesSequenceCounter) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  // Boot is now in region.backend_to_core, lane state full. Core has
  // not drained it. Capture the boot bytes so we can assert non-clobber.
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  std::array<std::uint8_t, sizeof(boot_descriptor)> boot_bytes_before{};
  std::memcpy(boot_bytes_before.data(), region.backend_to_core.payload.data(),
              sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto first_attempt = shim_or->send_can_frame_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind,
            tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  // Boot envelope and payload bytes unchanged: the shim must not have
  // clobbered the lane mid-rejection.
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(std::memcmp(region.backend_to_core.payload.data(),
                        boot_bytes_before.data(), sizeof(boot_descriptor)),
            0);

  // Recovery proof: drain boot via the core peer, then resend the same
  // batch. The recovery send must succeed at sequence == 1, NOT 2 — if
  // the failed attempt had advanced the counter, the recovery would
  // publish at sequence 2 and the core peer's session would reject it
  // as sequence_mismatch.
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto boot_drained = core.try_receive();
  ASSERT_TRUE(boot_drained.has_value());

  ASSERT_TRUE(shim_or->send_can_frame_batch(batch, 100'000).has_value());
  const auto recovery_msg = receive_from_shim(core);
  EXPECT_EQ(recovery_msg.envelope.sequence, 1u);
}

// ============================================================================
// Test C9-3b: lane_in_progress on send is rejected with the underlying
// transport diagnostic preserved, AND the outbound sequence counter
// is NOT consumed. Outbound mirror of test 13 (the inbound poll
// analogue). Pins D-C9-SEQUENCE-ADVANCE negative arm + D-C9-WRAPPED-
// SEND-ERROR for the lane_in_progress branch — a path the lane_busy
// test cannot exercise because it goes through a different make_error
// call site in shared_memory_transport.cpp.
// ============================================================================
TEST(ShimCoreSend, LaneInProgressSendIsRejectedAndPreservesSequenceCounter) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  // Plant a synthetic `writing` state on the outbound lane. No live
  // peer can produce this in a single-process test; it models a peer
  // that crashed mid-publish.
  region.backend_to_core.state.store(
      static_cast<std::uint32_t>(tier1_lane_state::writing),
      std::memory_order_release);

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto attempt = shim.send_can_frame_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(attempt.has_value());
  EXPECT_EQ(attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(attempt.error().transport_error.has_value());
  EXPECT_EQ(attempt.error().transport_error->kind,
            tier1_transport_error_kind::lane_in_progress);
  EXPECT_EQ(attempt.error().offending_field_name, "state");

  // Lane state unchanged: the shim must not have reset or advanced it.
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::writing));

  // Recovery proof: clear the synthetic state, retry. Sequence must be
  // 1 — if the failed attempt had advanced the counter, recovery would
  // be rejected by the core peer's session.
  region.backend_to_core.state.store(
      static_cast<std::uint32_t>(tier1_lane_state::empty),
      std::memory_order_release);
  ASSERT_TRUE(shim.send_can_frame_batch(batch, 100'000).has_value());
  const auto recovery_msg = receive_from_shim(core);
  EXPECT_EQ(recovery_msg.envelope.sequence, 1u);
}

// ============================================================================
// Test C9-4: send_can_frame_batch publishes successfully even when
// is_connected() == false, as long as the outbound lane is drainable.
// The protocol explicitly permits backend-side outbound traffic before
// boot_ack arrives. Pins D-C9-NO-CONNECT-GATE.
// ============================================================================
TEST(ShimCoreSend, PublishesBeforeBootAckIsReceivedSinceOutboundDoesNotGateOnConnect) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};

  // Drain the boot envelope on the core side, but do NOT send boot_ack
  // and do NOT poll the shim. is_connected() stays false.
  drain_boot_only(core);
  ASSERT_FALSE(shim_or->is_connected());

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto sent = shim_or->send_can_frame_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::can_frame_batch);

  // Outbound publishing must NOT have flipped is_connected() as a side
  // effect (would indicate a copy-paste-from-poll's-boot_ack-arm bug).
  EXPECT_FALSE(shim_or->is_connected());
}

// ============================================================================
// Test C9-5: post-shutdown send is rejected with shutdown_already_observed
// and does NOT call endpoint_.send (transport_error nullopt; lane
// untouched). Pins D-C9-SHUTDOWN-TERMINAL.
// ============================================================================
TEST(ShimCoreSend, PostShutdownSendIsRejectedWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());
  // Precondition: outbound lane is empty (boot was drained inside
  // make_connected_shim; the shim has not sent anything since). The
  // post-action `state == empty` assertion below would be tautological
  // against a non-empty lane — this assertion catches a future "boot
  // envelope leaks past make()" refactor that would invalidate the
  // test's claim.
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto sent = shim.send_can_frame_batch(batch, /*sim_time_us=*/250'000);
  ASSERT_FALSE(sent.has_value());
  EXPECT_EQ(sent.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(sent.error().transport_error.has_value());
  EXPECT_EQ(sent.error().offending_field_name, "kind");

  // Lane untouched: proves endpoint_.send was never called, which in
  // turn proves the session counter was not mutated.
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C9-6: a successful send_can_frame_batch does NOT mutate any of
// the eight inbound cache slots. Pins D-C9-INBOUND-INDEPENDENCE — the
// outbound path operates only on the outbound lane and the session's
// send-side counter; touching latest_can_frame_batch_ (the inbound
// cache slot whose name shadows the outbound payload's schema name)
// would be a copy-paste-from-poll() bug.
// ============================================================================
TEST(ShimCoreSend, SuccessfulSendDoesNotMutateAnyInboundCacheSlot) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_FALSE(shim.latest_clock_state().has_value());
  ASSERT_FALSE(shim.latest_power_state().has_value());
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  ASSERT_FALSE(shim.latest_can_frame_batch().has_value());
  ASSERT_FALSE(shim.latest_can_status().has_value());
  ASSERT_FALSE(shim.latest_notifier_state().has_value());
  ASSERT_FALSE(shim.latest_notifier_alarm_batch().has_value());
  ASSERT_FALSE(shim.latest_error_message_batch().has_value());

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  ASSERT_TRUE(shim.send_can_frame_batch(batch, /*sim_time_us=*/250'000).has_value());

  EXPECT_FALSE(shim.latest_clock_state().has_value());
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());
  EXPECT_FALSE(shim.latest_notifier_state().has_value());
  EXPECT_FALSE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_FALSE(shim.latest_error_message_batch().has_value());
}

// ============================================================================
// Test C9-7: two independent setups running the same outbound scenario
// produce byte-identical published wire bytes. Outbound determinism;
// non-negotiable #5. Mirror of C8-6 on the outbound side.
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalOutboundCanFrameBatch) {
  const std::array<can_frame, 3> first_frames{
      valid_can_frame(0x101, 1'000, 4, 0xA1),
      valid_can_frame(0x202, 2'000, 8, 0xB2),
      valid_can_frame(0x303, 3'000, 0, 0xC3)};
  const auto first_batch = valid_can_frame_batch(first_frames);
  const std::array<can_frame, 1> second_frames{
      valid_can_frame(0x404, 4'000, 6, 0xD4)};
  const auto second_batch = valid_can_frame_batch(second_frames);

  const auto run_setup = [&](tier1_shared_region& region,
                             tier1_endpoint& core,
                             tier1::tier1_message& out_first,
                             tier1::tier1_message& out_second) {
    core = make_core(region);
    auto endpoint = make_backend(region);
    auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), 0);
    ASSERT_TRUE(shim_or.has_value());
    auto boot = core.try_receive();
    ASSERT_TRUE(boot.has_value());
    ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
    ASSERT_TRUE(shim_or->poll().has_value());
    ASSERT_TRUE(shim_or->is_connected());

    ASSERT_TRUE(shim_or->send_can_frame_batch(first_batch, 250'000).has_value());
    auto first = core.try_receive();
    ASSERT_TRUE(first.has_value());
    out_first = std::move(*first);

    ASSERT_TRUE(shim_or->send_can_frame_batch(second_batch, 500'000).has_value());
    auto second = core.try_receive();
    ASSERT_TRUE(second.has_value());
    out_second = std::move(*second);
  };

  tier1_shared_region region_a{};
  tier1_shared_region region_b{};
  tier1_endpoint core_a{tier1_endpoint::make(region_a, direction::core_to_backend).value()};
  tier1_endpoint core_b{tier1_endpoint::make(region_b, direction::core_to_backend).value()};
  tier1::tier1_message first_a, second_a;
  tier1::tier1_message first_b, second_b;
  run_setup(region_a, core_a, first_a, second_a);
  run_setup(region_b, core_b, first_b, second_b);

  EXPECT_EQ(first_a.envelope, first_b.envelope);
  EXPECT_EQ(first_a.payload, first_b.payload);
  ASSERT_EQ(first_a.payload.size(), first_b.payload.size());
  EXPECT_EQ(std::memcmp(first_a.payload.data(),
                        first_b.payload.data(),
                        first_a.payload.size()),
            0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(),
                        second_b.payload.data(),
                        second_a.payload.size()),
            0);
}

// ============================================================================
// Test C9-8: send mutates only the send-side session counter, never
// the receive-side counter. Indirect verification: after three outbound
// sends the shim must still accept an inbound clock_state at the
// next-expected receive sequence (1) and cache it correctly. A "send
// mutates receive counter" bug would either reject the inbound on
// sequence_mismatch or mis-cache.
// ============================================================================
TEST(ShimCoreSend, SendDoesNotPerturbProtocolSessionsReceiveExpectedSequence) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(shim.send_can_frame_batch(batch, /*sim_time_us=*/100'000).has_value());
    auto drained = core.try_receive();
    ASSERT_TRUE(drained.has_value());
  }

  // Core's next_sequence_to_send_ is still 1 (boot_ack was 0; core has
  // not sent anything else). Send a clock_state at sequence 1 and verify
  // the shim accepts it — proving its receive counter is still 1, not
  // perturbed by the three outbound sends.
  const auto state = valid_clock_state(/*sim_time_us=*/100'000);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(state),
                        100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), state);
}

// ============================================================================
// Cycle 10 — outbound notifier_state (`send_notifier_state`).
// Second outbound-meaningful schema; cashes in D-C9-NO-HELPER's
// active-prefix-helper extraction. See TEST_PLAN_CYCLE10.md.
// ============================================================================

// ============================================================================
// Test C10-1: send_notifier_state publishes a tick_boundary envelope
// carrying exactly the active prefix (offsetof(notifier_state, slots)
// + count*sizeof(notifier_slot) = 8 + 3*88 = 272 bytes for a 3-slot
// batch). Pins D-C10-ACTIVE-PREFIX-OUT-INHERITS (in particular the
// 8-byte header offset, NOT cycle-9's 4-byte one) and verifies the
// extracted production active_prefix_bytes helper.
//
// The 272-byte memcmp covers 16 implicit padding bytes within the
// active prefix: the 4-byte interior count→slots pad at offsets 4–7,
// plus the 4-byte trailing pad inside each of the 3 active
// notifier_slot entries. valid_notifier_state's notifier_state{}
// zero-init makes the comparison deterministic on the source side.
// ============================================================================
TEST(ShimCoreSend, PublishesNotifierStateAsTickBoundaryWithActivePrefixWireBytes) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 3> slots{
      valid_notifier_slot(/*trigger_time_us=*/1'000'000, /*handle=*/10,
                          /*alarm_active=*/1, /*canceled=*/0,
                          /*name=*/"alpha"),
      valid_notifier_slot(2'000'000, 20, 0, 1, "beta"),
      valid_notifier_slot(3'000'000, 30, 1, 1, "gamma")};
  const auto state = valid_notifier_state(slots);

  auto sent = shim.send_notifier_state(state, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::notifier_state);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 272u);  // 8 + 3*88.
  ASSERT_EQ(msg.payload.size(), 272u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &state, 272), 0);
}

// ============================================================================
// Test C10-1b: an empty (count=0) notifier_state batch is a legal
// publish; the active prefix is the 8-byte header (count word + the
// 4-byte interior pad to satisfy alignof(notifier_slot) == 8). Pins
// D-C10-EMPTY-BATCH-OUT-INHERITS and the count=0 boundary of the
// active-prefix discipline. The 8-byte memcmp covers the count word
// AND the interior pad, both byte-zero from the notifier_state{}
// zero-init in valid_notifier_state.
// ============================================================================
TEST(ShimCoreSend, PublishesEmptyNotifierStateAsHeaderOnlyTickBoundary) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto empty_state = valid_notifier_state();

  auto sent = shim.send_notifier_state(empty_state, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::notifier_state);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 8u);  // header-only.
  ASSERT_EQ(msg.payload.size(), 8u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &empty_state, 8), 0);
}

// ============================================================================
// Test C10-2: a second send_notifier_state call advances the outbound
// sequence counter to 2. Different counts (2 then 4) catch a "sequence
// advances by slot count" bug, AND verify the extracted helper produces
// the right size for both counts (184 = 8+2*88; 360 = 8+4*88).
// ============================================================================
TEST(ShimCoreSend, SecondNotifierStateSendAdvancesOutboundSequenceToTwo) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<notifier_slot, 2> first_slots{
      valid_notifier_slot(100, 11, 1, 0, "first0"),
      valid_notifier_slot(200, 22, 0, 1, "first1")};
  const auto first = valid_notifier_state(first_slots);

  ASSERT_TRUE(shim.send_notifier_state(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 184u);  // 8 + 2*88.
  ASSERT_EQ(first_msg.payload.size(), 184u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 184), 0);

  const std::array<notifier_slot, 4> second_slots{
      valid_notifier_slot(300, 33, 1, 1, "second0"),
      valid_notifier_slot(400, 44, 0, 0, "second1"),
      valid_notifier_slot(500, 55, 1, 0, "second2"),
      valid_notifier_slot(600, 66, 0, 1, "second3")};
  const auto second = valid_notifier_state(second_slots);

  ASSERT_TRUE(shim.send_notifier_state(second, /*sim_time_us=*/200'000).has_value());
  const auto second_msg = receive_from_shim(core);
  EXPECT_EQ(second_msg.envelope.sequence, 2u);
  EXPECT_EQ(second_msg.envelope.sim_time_us, 200'000u);
  EXPECT_EQ(second_msg.envelope.payload_bytes, 360u);  // 8 + 4*88.
  ASSERT_EQ(second_msg.payload.size(), 360u);
  EXPECT_EQ(std::memcmp(second_msg.payload.data(), &second, 360), 0);
}

// ============================================================================
// Test C10-3: lane_busy on send_notifier_state is rejected with the
// underlying transport diagnostic preserved, AND the outbound sequence
// counter is NOT consumed. Pins D-C10-SEQUENCE-ADVANCE-INHERITS
// negative arm + D-C10-WRAPPED-SEND-ERROR-INHERITS for the new method.
// ============================================================================
TEST(ShimCoreSend, LaneBusyNotifierStateSendIsRejectedAndPreservesSequenceCounter) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  std::array<std::uint8_t, sizeof(boot_descriptor)> boot_bytes_before{};
  std::memcpy(boot_bytes_before.data(), region.backend_to_core.payload.data(),
              sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<notifier_slot, 1> slots{
      valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")};
  const auto state = valid_notifier_state(slots);

  auto first_attempt = shim_or->send_notifier_state(state, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind,
            tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(std::memcmp(region.backend_to_core.payload.data(),
                        boot_bytes_before.data(), sizeof(boot_descriptor)),
            0);

  // Recovery proof: drain boot, resend, assert sequence == 1.
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto boot_drained = core.try_receive();
  ASSERT_TRUE(boot_drained.has_value());

  ASSERT_TRUE(shim_or->send_notifier_state(state, 100'000).has_value());
  const auto recovery_msg = receive_from_shim(core);
  EXPECT_EQ(recovery_msg.envelope.sequence, 1u);
}

// ============================================================================
// Test C10-5: post-shutdown send_notifier_state is rejected with
// shutdown_already_observed and does NOT call endpoint_.send. Pins
// D-C10-SHUTDOWN-TERMINAL-INHERITS — the one D-C9-derived contract
// that needs per-method verification because each typed send_*
// method carries its own short-circuit code. An "implementer added
// the new method but forgot the short-circuit" bug is a real risk
// per-method.
// ============================================================================
TEST(ShimCoreSend, PostShutdownNotifierStateSendIsRejectedWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  const std::array<notifier_slot, 1> slots{
      valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")};
  const auto state = valid_notifier_state(slots);

  auto sent = shim.send_notifier_state(state, /*sim_time_us=*/250'000);
  ASSERT_FALSE(sent.has_value());
  EXPECT_EQ(sent.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(sent.error().transport_error.has_value());
  EXPECT_EQ(sent.error().offending_field_name, "kind");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C10-7: two independent setups running the same outbound
// notifier_state scenario produce byte-identical published wire bytes.
// Outbound determinism for the schema with the most demanding padding
// profile (132 implicit pad bytes total; 4 + 4*count within the
// active prefix). The std::memcmp companion is load-bearing in a way
// C9-7's was not: notifier_state has the 4-byte interior count→slots
// pad that can_frame_batch lacks, so a "shim reads from a source
// that wasn't zero-initialized" bug would leak uninit stack bytes
// into offsets 4–7 and diverge between two runs.
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalOutboundNotifierState) {
  const std::array<notifier_slot, 3> first_slots{
      valid_notifier_slot(1'000'000, 10, 1, 0, "alpha"),
      valid_notifier_slot(2'000'000, 20, 0, 1, "beta"),
      valid_notifier_slot(3'000'000, 30, 1, 1, "gamma")};
  const auto first_state = valid_notifier_state(first_slots);
  const std::array<notifier_slot, 1> second_slots{
      valid_notifier_slot(4'000'000, 40, 1, 1, "delta")};
  const auto second_state = valid_notifier_state(second_slots);

  const auto run_setup = [&](tier1_shared_region& region,
                             tier1_endpoint& core,
                             tier1::tier1_message& out_first,
                             tier1::tier1_message& out_second) {
    core = make_core(region);
    auto endpoint = make_backend(region);
    auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), 0);
    ASSERT_TRUE(shim_or.has_value());
    auto boot = core.try_receive();
    ASSERT_TRUE(boot.has_value());
    ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
    ASSERT_TRUE(shim_or->poll().has_value());
    ASSERT_TRUE(shim_or->is_connected());

    ASSERT_TRUE(shim_or->send_notifier_state(first_state, 250'000).has_value());
    auto first = core.try_receive();
    ASSERT_TRUE(first.has_value());
    out_first = std::move(*first);

    ASSERT_TRUE(shim_or->send_notifier_state(second_state, 500'000).has_value());
    auto second = core.try_receive();
    ASSERT_TRUE(second.has_value());
    out_second = std::move(*second);
  };

  tier1_shared_region region_a{};
  tier1_shared_region region_b{};
  tier1_endpoint core_a{tier1_endpoint::make(region_a, direction::core_to_backend).value()};
  tier1_endpoint core_b{tier1_endpoint::make(region_b, direction::core_to_backend).value()};
  tier1::tier1_message first_a, second_a;
  tier1::tier1_message first_b, second_b;
  run_setup(region_a, core_a, first_a, second_a);
  run_setup(region_b, core_b, first_b, second_b);

  EXPECT_EQ(first_a.envelope, first_b.envelope);
  EXPECT_EQ(first_a.payload, first_b.payload);
  ASSERT_EQ(first_a.payload.size(), first_b.payload.size());
  EXPECT_EQ(std::memcmp(first_a.payload.data(),
                        first_b.payload.data(),
                        first_a.payload.size()),
            0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(),
                        second_b.payload.data(),
                        second_a.payload.size()),
            0);
}

// ============================================================================
// Cycle 11 — outbound error_message_batch (`send_error_message_batch`).
// Third and final outbound-meaningful schema; closes the v0 outbound
// surface set. See TEST_PLAN_CYCLE11.md.
// ============================================================================

// ============================================================================
// Test C11-1: send_error_message_batch publishes a tick_boundary
// envelope carrying exactly the active prefix (8 + 3*2324 = 6980 bytes
// for a 3-message batch). Pins D-C9-ACTIVE-PREFIX-OUT for the third
// outbound schema and verifies the new production active_prefix_bytes
// helper in error_message.h.
//
// Per D-C8-PADDING-FREE the schema has zero implicit C++ padding —
// both reserved_pad fields are NAMED — so the 6980-byte memcmp is
// byte-equivalent to defaulted operator==. Kept memcmp-shaped for
// parity with C9-1 / C10-1.
// ============================================================================
TEST(ShimCoreSend, PublishesErrorMessageBatchAsTickBoundaryWithActivePrefixWireBytes) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 3> messages{
      valid_error_message(/*error_code=*/100, /*severity=*/1,
                          /*is_lv_code=*/0, /*print_msg=*/1,
                          /*truncation_flags=*/0,
                          "first detail", "first location",
                          "first call stack"),
      valid_error_message(200, 0, 1, 0, kErrorTruncDetails,
                          "second detail", "second location",
                          "second call stack"),
      valid_error_message(300, 1, 0, 1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail", "third location",
                          "third call stack")};
  const auto batch = valid_error_message_batch(messages);

  auto sent = shim.send_error_message_batch(batch, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::error_message_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 6980u);  // 8 + 3*2324.
  ASSERT_EQ(msg.payload.size(), 6980u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &batch, 6980), 0);
}

// ============================================================================
// Test C11-1b: an empty (count=0) error_message_batch is a legal
// publish; the active prefix is the 8-byte header (count word + the
// NAMED reserved_pad[4]). The 8-byte memcmp covers both, byte-zero
// from valid_error_message_batch's zero-init.
// ============================================================================
TEST(ShimCoreSend, PublishesEmptyErrorMessageBatchAsHeaderOnlyTickBoundary) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto empty_batch = valid_error_message_batch();

  auto sent = shim.send_error_message_batch(empty_batch, /*sim_time_us=*/250'000);
  ASSERT_TRUE(sent.has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::error_message_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 250'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 8u);  // header-only.
  ASSERT_EQ(msg.payload.size(), 8u);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &empty_batch, 8), 0);
}

// ============================================================================
// Test C11-2: a second send advances the outbound sequence counter to
// 2. Different counts (2 then 4) catch a "sequence advances by message
// count" bug AND verify the helper produces the right size for both.
// ============================================================================
TEST(ShimCoreSend, SecondErrorMessageBatchSendAdvancesOutboundSequenceToTwo) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 2> first_messages{
      valid_error_message(10, 1, 0, 1, 0,
                          "alpha details", "alpha location", "alpha stack"),
      valid_error_message(20, 0, 1, 0, kErrorTruncDetails,
                          "beta details", "beta location", "beta stack")};
  const auto first = valid_error_message_batch(first_messages);

  ASSERT_TRUE(shim.send_error_message_batch(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 4656u);  // 8 + 2*2324.
  ASSERT_EQ(first_msg.payload.size(), 4656u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 4656), 0);

  const std::array<error_message, 4> second_messages{
      valid_error_message(50, 1, 1, 1, kErrorTruncLocation,
                          "delta details", "delta location", "delta stack"),
      valid_error_message(60, 0, 0, 0,
                          kErrorTruncDetails | kErrorTruncCallStack,
                          "epsilon details", "epsilon location", "epsilon stack"),
      valid_error_message(70, 1, 0, 1, kErrorTruncCallStack,
                          "zeta details", "zeta location", "zeta stack"),
      valid_error_message(80, 0, 1, 1, 0,
                          "eta details", "eta location", "eta stack")};
  const auto second = valid_error_message_batch(second_messages);

  ASSERT_TRUE(shim.send_error_message_batch(second, /*sim_time_us=*/200'000).has_value());
  const auto second_msg = receive_from_shim(core);
  EXPECT_EQ(second_msg.envelope.sequence, 2u);
  EXPECT_EQ(second_msg.envelope.sim_time_us, 200'000u);
  EXPECT_EQ(second_msg.envelope.payload_bytes, 9304u);  // 8 + 4*2324.
  ASSERT_EQ(second_msg.payload.size(), 9304u);
  EXPECT_EQ(std::memcmp(second_msg.payload.data(), &second, 9304), 0);
}

// ============================================================================
// Test C11-3: lane_busy on send is rejected; sequence counter not
// consumed. Recovery proof: drain boot, resend, sequence == 1.
// ============================================================================
TEST(ShimCoreSend, LaneBusyErrorMessageBatchSendIsRejectedAndPreservesSequenceCounter) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  std::array<std::uint8_t, sizeof(boot_descriptor)> boot_bytes_before{};
  std::memcpy(boot_bytes_before.data(), region.backend_to_core.payload.data(),
              sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<error_message, 1> messages{
      valid_error_message(100, 1, 0, 1, 0, "detail", "location", "stack")};
  const auto batch = valid_error_message_batch(messages);

  auto first_attempt = shim_or->send_error_message_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind,
            tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(std::memcmp(region.backend_to_core.payload.data(),
                        boot_bytes_before.data(), sizeof(boot_descriptor)),
            0);

  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto boot_drained = core.try_receive();
  ASSERT_TRUE(boot_drained.has_value());

  ASSERT_TRUE(shim_or->send_error_message_batch(batch, 100'000).has_value());
  const auto recovery_msg = receive_from_shim(core);
  EXPECT_EQ(recovery_msg.envelope.sequence, 1u);
}

// ============================================================================
// Test C11-5: post-shutdown send_error_message_batch is rejected with
// shutdown_already_observed; transport_error nullopt proves
// endpoint_.send was NOT called. Per-method verification per
// D-C10-SHUTDOWN-TERMINAL-INHERITS — each typed send_* method carries
// its own short-circuit code.
// ============================================================================
TEST(ShimCoreSend, PostShutdownErrorMessageBatchSendIsRejectedWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  const std::array<error_message, 1> messages{
      valid_error_message(100, 1, 0, 1, 0, "detail", "location", "stack")};
  const auto batch = valid_error_message_batch(messages);

  auto sent = shim.send_error_message_batch(batch, /*sim_time_us=*/250'000);
  ASSERT_FALSE(sent.has_value());
  EXPECT_EQ(sent.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(sent.error().transport_error.has_value());
  EXPECT_EQ(sent.error().offending_field_name, "kind");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Test C11-7: two independent setups produce byte-identical wire
// output. Determinism guarantee for the third (and final) outbound
// schema. Per D-C8-PADDING-FREE the memcmp companion is byte-
// equivalent to vector::operator==; kept for cross-cycle parity with
// C9-7 / C10-7.
// ============================================================================
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalOutboundErrorMessageBatch) {
  const std::array<error_message, 3> first_messages{
      valid_error_message(100, 1, 0, 1, 0,
                          "first detail", "first location", "first call stack"),
      valid_error_message(200, 0, 1, 0, kErrorTruncDetails,
                          "second detail", "second location", "second call stack"),
      valid_error_message(300, 1, 0, 1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail", "third location", "third call stack")};
  const auto first_batch = valid_error_message_batch(first_messages);
  const std::array<error_message, 1> second_messages{
      valid_error_message(400, 0, 1, 1,
                          kErrorTruncDetails | kErrorTruncLocation |
                              kErrorTruncCallStack,
                          "delta detail", "delta location", "delta call stack")};
  const auto second_batch = valid_error_message_batch(second_messages);

  const auto run_setup = [&](tier1_shared_region& region,
                             tier1_endpoint& core,
                             tier1::tier1_message& out_first,
                             tier1::tier1_message& out_second) {
    core = make_core(region);
    auto endpoint = make_backend(region);
    auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), 0);
    ASSERT_TRUE(shim_or.has_value());
    auto boot = core.try_receive();
    ASSERT_TRUE(boot.has_value());
    ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
    ASSERT_TRUE(shim_or->poll().has_value());
    ASSERT_TRUE(shim_or->is_connected());

    ASSERT_TRUE(shim_or->send_error_message_batch(first_batch, 250'000).has_value());
    auto first = core.try_receive();
    ASSERT_TRUE(first.has_value());
    out_first = std::move(*first);

    ASSERT_TRUE(shim_or->send_error_message_batch(second_batch, 500'000).has_value());
    auto second = core.try_receive();
    ASSERT_TRUE(second.has_value());
    out_second = std::move(*second);
  };

  tier1_shared_region region_a{};
  tier1_shared_region region_b{};
  tier1_endpoint core_a{tier1_endpoint::make(region_a, direction::core_to_backend).value()};
  tier1_endpoint core_b{tier1_endpoint::make(region_b, direction::core_to_backend).value()};
  tier1::tier1_message first_a, second_a;
  tier1::tier1_message first_b, second_b;
  run_setup(region_a, core_a, first_a, second_a);
  run_setup(region_b, core_b, first_b, second_b);

  EXPECT_EQ(first_a.envelope, first_b.envelope);
  EXPECT_EQ(first_a.payload, first_b.payload);
  ASSERT_EQ(first_a.payload.size(), first_b.payload.size());
  EXPECT_EQ(std::memcmp(first_a.payload.data(),
                        first_b.payload.data(),
                        first_a.payload.size()),
            0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(),
                        second_b.payload.data(),
                        second_a.payload.size()),
            0);
}

}  // namespace robosim::backend::shim
