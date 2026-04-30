#include "shim_core.h"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "hal_c.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_session.h"
#include "protocol_version.h"
#include "shared_memory_transport.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <future>
#include <limits>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace {

std::vector<unsigned int>& wpi_set_event_call_log() {
  static std::vector<unsigned int> calls;
  return calls;
}

}  // namespace

extern "C" void WPI_SetEvent(unsigned int handle) {
  wpi_set_event_call_log().push_back(handle);
}

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

// Cycle-12 helper. Installs `shim` as the C-HAL-ABI process-global for
// the duration of the guard's scope, restoring nullptr on destruction.
// Tests that exercise HAL_GetFPGATime (and future HAL_* surfaces)
// construct one of these after make_connected_shim returns. The guard
// is intentionally NOT used by the tests that exercise install_global
// / current directly (C12-1, C12-2, C12-3) — those tests are about
// the install surface itself; using the guard would test the fixture
// rather than the surface.
class shim_global_install_guard {
 public:
  explicit shim_global_install_guard(shim_core& shim) { shim_core::install_global(&shim); }
  ~shim_global_install_guard() { shim_core::install_global(nullptr); }
  shim_global_install_guard(const shim_global_install_guard&) = delete;
  shim_global_install_guard& operator=(const shim_global_install_guard&) = delete;
};

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
  EXPECT_EQ(std::memcmp(region.backend_to_core.payload.data(), &desc, sizeof(boot_descriptor)), 0);
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 250'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);

  // Second update.
  const auto second = valid_clock_state(200'000);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(second), 200'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 6'000));
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
  region.core_to_backend.state.store(static_cast<std::uint32_t>(tier1_lane_state::writing),
                                     std::memory_order_release);

  auto result = shim_or->poll();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, shim_error_kind::receive_failed);
  ASSERT_TRUE(result.error().transport_error.has_value());
  EXPECT_EQ(result.error().transport_error->kind, tier1_transport_error_kind::lane_in_progress);
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 1'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(first), 1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), first);

  const auto second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(second), 2'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());

  // Step 2: power_state — clock slot must be unchanged.
  const auto power = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power), 150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), valid_power_state(12.5f, 2.0f, 6.8f));

  // Step 3: clock_state at 200'000 — power slot must be unchanged.
  const auto clock_second = valid_clock_state(200'000);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 200'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), 1'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(first), 1'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(second), 2'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());

  // --- Step 2: power_state arrival. Clock unchanged.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);  // unchanged
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  EXPECT_FALSE(shim.latest_ds_state().has_value());

  // --- Step 3: ds_state arrival. Clock and power unchanged
  // (catches "ds-arm clobbers clock" and "ds-arm clobbers power").
  const auto ds_value = valid_ds_state();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_value), 200'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 250'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 300'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(batch), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(batch), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(first), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(first), 1'000));
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
  EXPECT_EQ(std::memcmp(&shim.latest_can_frame_batch()->frames[2], &empty_frame, sizeof(can_frame)),
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());

  // --- Step 2: power_state arrival.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());

  // --- Step 3: ds_state arrival.
  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_first), 200'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 300'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 350'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_second), 400'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(state), 1'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(first), 1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_status().has_value());
  EXPECT_EQ(*shim.latest_can_status(), first);

  const auto second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(second), 2'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  EXPECT_FALSE(shim.latest_power_state().has_value());
  EXPECT_FALSE(shim.latest_ds_state().has_value());
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
  EXPECT_FALSE(shim.latest_can_status().has_value());

  // --- Step 2: power_state arrival.
  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_first), 200'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_first), 300'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 350'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 400'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_second), 450'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(state), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(state), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(first), 1'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), first);

  const std::array<notifier_slot, 3> second_slots{
      valid_notifier_slot(500, 5, 1, 1, "delta"),
      valid_notifier_slot(600, 6, 0, 0, "epsilon"),
      valid_notifier_slot(700, 7, 1, 0, "zeta"),
  };
  const auto second = valid_notifier_state(second_slots);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(second), 2'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(first), 1'000));
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_slot, 2> second_slots{
      valid_notifier_slot(100, 100, 1, 0, "new0"),
      valid_notifier_slot(200, 200, 0, 1, "new1"),
  };
  const auto second = valid_notifier_state(second_slots);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(second), 2'000));
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
  EXPECT_EQ(
      std::memcmp(&shim.latest_notifier_state()->slots[2], &empty_slot, sizeof(notifier_slot)), 0);
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(state), 50'000));
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

  constexpr std::size_t kNotifierStatePrefixBytes = offsetof(notifier_state, slots);
  static_assert(kNotifierStatePrefixBytes == 8, "notifier_state header pad must be 8");
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u);
  ASSERT_TRUE(shim.latest_power_state().has_value());
  EXPECT_EQ(*shim.latest_power_state(), power_first);

  // --- Step 3: ds_state arrival.
  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_first), 200'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_first), 300'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 400'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);  // updated
  ASSERT_TRUE(shim.latest_notifier_state().has_value());
  EXPECT_EQ(*shim.latest_notifier_state(), ns_first);  // unchanged

  // --- Step 8: second power_state. Catches power→ns.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 450'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_second), 500'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_second), 600'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), 50'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(state), 50'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_first), 200'000));
  ASSERT_TRUE(shim.poll().has_value());

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

  const auto cs_first = valid_can_status();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_first), 300'000));
  ASSERT_TRUE(shim.poll().has_value());

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

  // --- Step 7: notifier_alarm_batch. Catches ab→{clock,power,ds,cf,cs,ns}.
  const std::array<notifier_alarm_event, 2> ab_events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
  };
  const auto ab_first = valid_notifier_alarm_batch(ab_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(ab_first),
                        400'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 450'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 9: power→ab.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 500'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_second), 550'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second),
                        600'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(*shim.latest_notifier_alarm_batch(), ab_first);

  // --- Step 12: cs→ab.
  const auto cs_second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_second), 650'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(ns_second),
                        700'000));
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
      valid_error_message(/*error_code=*/100,
                          /*severity=*/1,
                          /*is_lv_code=*/0,
                          /*print_msg=*/1,
                          /*truncation_flags=*/0,
                          "first detail",
                          "first location",
                          "first call stack"),
      valid_error_message(200,
                          0,
                          1,
                          0,
                          kErrorTruncDetails,
                          "second detail",
                          "second location",
                          "second call stack"),
      valid_error_message(300,
                          1,
                          0,
                          1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail",
                          "third location",
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
  EXPECT_EQ(*shim.latest_error_message_batch(), valid_error_message_batch());
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
      valid_error_message(10, 1, 0, 1, 0, "alpha details", "alpha location", "alpha stack"),
      valid_error_message(
          20, 0, 1, 0, kErrorTruncDetails, "beta details", "beta location", "beta stack"),
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
      valid_error_message(
          50, 1, 1, 1, kErrorTruncLocation, "delta details", "delta location", "delta stack"),
      valid_error_message(60,
                          0,
                          0,
                          0,
                          kErrorTruncDetails | kErrorTruncCallStack,
                          "epsilon details",
                          "epsilon location",
                          "epsilon stack"),
      valid_error_message(
          70, 1, 0, 1, kErrorTruncCallStack, "zeta details", "zeta location", "zeta stack"),
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
      valid_error_message(2, 0, 1, 0, kErrorTruncDetails, "m2d", "m2l", "m2c"),
      valid_error_message(3, 1, 1, 1, kErrorTruncLocation, "m3d", "m3l", "m3c"),
      valid_error_message(4, 0, 0, 1, kErrorTruncCallStack, "m4d", "m4l", "m4c"),
      valid_error_message(
          5, 1, 0, 0, kErrorTruncDetails | kErrorTruncLocation, "m5d", "m5l", "m5c"),
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

  EXPECT_EQ(std::memcmp(&cached.messages[2], &empty_message, sizeof(error_message)), 0);
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

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of(valid_clock_state(123'456)),
                        1'000));
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

  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::power_state,
                        bytes_of(valid_power_state(12.5f, 2.0f, 6.8f)),
                        1'000));
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

  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(valid_ds_state()), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(cf), 1'000));
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

  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::can_status, bytes_of(valid_can_status()), 1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(ns), 1'000));
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
                        active_prefix_bytes(nab),
                        1'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_first), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto power_first = valid_power_state(12.5f, 2.0f, 6.8f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_first), 150'000));
  ASSERT_TRUE(shim.poll().has_value());

  const auto ds_first = valid_ds_state();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_first), 200'000));
  ASSERT_TRUE(shim.poll().has_value());

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

  const auto cs_first = valid_can_status();
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_first), 300'000));
  ASSERT_TRUE(shim.poll().has_value());

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

  const std::array<notifier_alarm_event, 2> ab_events{
      valid_notifier_alarm_event(100, 1),
      valid_notifier_alarm_event(200, 2),
  };
  const auto ab_first = valid_notifier_alarm_batch(ab_events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(ab_first),
                        400'000));
  ASSERT_TRUE(shim.poll().has_value());

  // --- Step 8: error_message_batch. Catches emb→{clock,power,ds,cf,cs,ns,nab}.
  const std::array<error_message, 2> emb_first_messages{
      valid_error_message(11,
                          1,
                          0,
                          1,
                          0,
                          "first batch alpha details",
                          "first batch alpha location",
                          "first batch alpha stack"),
      valid_error_message(22,
                          0,
                          1,
                          0,
                          kErrorTruncDetails,
                          "first batch beta details",
                          "first batch beta location",
                          "first batch beta stack"),
  };
  const auto emb_first = valid_error_message_batch(emb_first_messages);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::error_message_batch,
                        active_prefix_bytes(emb_first),
                        450'000));
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
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock_second), 500'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), clock_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 10: power→emb.
  const auto power_second = valid_power_state(13.5f, 5.0f, 6.5f);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power_second), 550'000));
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds_second), 600'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(cf_second),
                        650'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), cf_second);
  ASSERT_TRUE(shim.latest_error_message_batch().has_value());
  EXPECT_EQ(*shim.latest_error_message_batch(), emb_first);

  // --- Step 13: cs→emb.
  const auto cs_second = valid_can_status(0.85f, 9, 8, 7, 6);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs_second), 700'000));
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
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_state,
                        active_prefix_bytes(ns_second),
                        750'000));
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
                        active_prefix_bytes(ab_second),
                        800'000));
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
    EXPECT_TRUE(core.send(
        envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(first_clock), 50'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto power = valid_power_state(12.5f, 2.0f, 6.8f);
    EXPECT_TRUE(
        core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(power), 75'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto ds = valid_ds_state();
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(ds), 90'000));
    EXPECT_TRUE(shim.poll().has_value());

    const std::array<can_frame, 2> cf_frames{
        valid_can_frame(0x100, 1000, 4, 0xA0),
        valid_can_frame(0x200, 2000, 2, 0xB0),
    };
    const auto cf = valid_can_frame_batch(cf_frames);
    EXPECT_TRUE(core.send(
        envelope_kind::tick_boundary, schema_id::can_frame_batch, active_prefix_bytes(cf), 95'000));
    EXPECT_TRUE(shim.poll().has_value());

    const auto cs = valid_can_status();
    EXPECT_TRUE(
        core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(cs), 97'000));
    EXPECT_TRUE(shim.poll().has_value());

    const std::array<notifier_slot, 3> ns_slots{
        valid_notifier_slot(100, 1, 1, 0, "alpha"),
        valid_notifier_slot(200, 2, 0, 1, "beta"),
        valid_notifier_slot(300, 3, 1, 1, "gamma"),
    };
    const auto ns = valid_notifier_state(ns_slots);
    EXPECT_TRUE(core.send(
        envelope_kind::tick_boundary, schema_id::notifier_state, active_prefix_bytes(ns), 98'000));
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
        valid_error_message(101, 1, 0, 1, 0, "det1", "loc1", "stk1"),
        valid_error_message(202, 0, 1, 0, kErrorTruncDetails, "det2", "loc2", "stk2"),
        valid_error_message(
            303, 1, 0, 1, kErrorTruncLocation | kErrorTruncCallStack, "det3", "loc3", "stk3"),
    };
    const auto emb = valid_error_message_batch(emb_messages);
    EXPECT_TRUE(core.send(envelope_kind::tick_boundary,
                          schema_id::error_message_batch,
                          active_prefix_bytes(emb),
                          99'500));
    EXPECT_TRUE(shim.poll().has_value());

    const auto second_clock = valid_clock_state(100'000);
    EXPECT_TRUE(core.send(
        envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(second_clock), 100'000));
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
  EXPECT_EQ(*shim_a.latest_notifier_alarm_batch(), *shim_b.latest_notifier_alarm_batch());
  EXPECT_EQ(*shim_a.latest_error_message_batch(), *shim_b.latest_error_message_batch());

  // Padding-byte determinism on the four padding-bearing schemas.
  EXPECT_EQ(std::memcmp(&*shim_a.latest_ds_state(), &*shim_b.latest_ds_state(), sizeof(ds_state)),
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

  const std::array<can_frame, 3> frames{valid_can_frame(/*message_id=*/0x101,
                                                        /*timestamp_us=*/1'000,
                                                        /*data_size=*/4,
                                                        /*fill_byte=*/0xA1),
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

  const std::array<can_frame, 2> first_frames{valid_can_frame(0x111, 1'000, 4, 0xAA),
                                              valid_can_frame(0x222, 2'000, 8, 0xBB)};
  const auto first = valid_can_frame_batch(first_frames);

  ASSERT_TRUE(shim.send_can_frame_batch(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 44u);  // 4 + 2*20.
  ASSERT_EQ(first_msg.payload.size(), 44u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 44), 0);

  const std::array<can_frame, 4> second_frames{valid_can_frame(0x333, 3'000, 4, 0xCC),
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
  std::memcpy(
      boot_bytes_before.data(), region.backend_to_core.payload.data(), sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto first_attempt = shim_or->send_can_frame_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  // Boot envelope and payload bytes unchanged: the shim must not have
  // clobbered the lane mid-rejection.
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(
      std::memcmp(
          region.backend_to_core.payload.data(), boot_bytes_before.data(), sizeof(boot_descriptor)),
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
  region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::writing),
                                     std::memory_order_release);

  const std::array<can_frame, 1> frames{valid_can_frame(0x101, 1'000, 4, 0xA1)};
  const auto batch = valid_can_frame_batch(frames);

  auto attempt = shim.send_can_frame_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(attempt.has_value());
  EXPECT_EQ(attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(attempt.error().transport_error.has_value());
  EXPECT_EQ(attempt.error().transport_error->kind, tier1_transport_error_kind::lane_in_progress);
  EXPECT_EQ(attempt.error().offending_field_name, "state");

  // Lane state unchanged: the shim must not have reset or advanced it.
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::writing));

  // Recovery proof: clear the synthetic state, retry. Sequence must be
  // 1 — if the failed attempt had advanced the counter, recovery would
  // be rejected by the core peer's session.
  region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::empty),
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
  const std::array<can_frame, 3> first_frames{valid_can_frame(0x101, 1'000, 4, 0xA1),
                                              valid_can_frame(0x202, 2'000, 8, 0xB2),
                                              valid_can_frame(0x303, 3'000, 0, 0xC3)};
  const auto first_batch = valid_can_frame_batch(first_frames);
  const std::array<can_frame, 1> second_frames{valid_can_frame(0x404, 4'000, 6, 0xD4)};
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
  EXPECT_EQ(std::memcmp(first_a.payload.data(), first_b.payload.data(), first_a.payload.size()), 0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(), second_b.payload.data(), second_a.payload.size()),
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
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 100'000));
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

  const std::array<notifier_slot, 3> slots{valid_notifier_slot(/*trigger_time_us=*/1'000'000,
                                                               /*handle=*/10,
                                                               /*alarm_active=*/1,
                                                               /*canceled=*/0,
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

  const std::array<notifier_slot, 2> first_slots{valid_notifier_slot(100, 11, 1, 0, "first0"),
                                                 valid_notifier_slot(200, 22, 0, 1, "first1")};
  const auto first = valid_notifier_state(first_slots);

  ASSERT_TRUE(shim.send_notifier_state(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 184u);  // 8 + 2*88.
  ASSERT_EQ(first_msg.payload.size(), 184u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 184), 0);

  const std::array<notifier_slot, 4> second_slots{valid_notifier_slot(300, 33, 1, 1, "second0"),
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
  std::memcpy(
      boot_bytes_before.data(), region.backend_to_core.payload.data(), sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<notifier_slot, 1> slots{valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")};
  const auto state = valid_notifier_state(slots);

  auto first_attempt = shim_or->send_notifier_state(state, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(
      std::memcmp(
          region.backend_to_core.payload.data(), boot_bytes_before.data(), sizeof(boot_descriptor)),
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

  const std::array<notifier_slot, 1> slots{valid_notifier_slot(1'000'000, 10, 1, 0, "alpha")};
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
  const std::array<notifier_slot, 3> first_slots{valid_notifier_slot(1'000'000, 10, 1, 0, "alpha"),
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
  EXPECT_EQ(std::memcmp(first_a.payload.data(), first_b.payload.data(), first_a.payload.size()), 0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(), second_b.payload.data(), second_a.payload.size()),
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
      valid_error_message(/*error_code=*/100,
                          /*severity=*/1,
                          /*is_lv_code=*/0,
                          /*print_msg=*/1,
                          /*truncation_flags=*/0,
                          "first detail",
                          "first location",
                          "first call stack"),
      valid_error_message(200,
                          0,
                          1,
                          0,
                          kErrorTruncDetails,
                          "second detail",
                          "second location",
                          "second call stack"),
      valid_error_message(300,
                          1,
                          0,
                          1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail",
                          "third location",
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
      valid_error_message(10, 1, 0, 1, 0, "alpha details", "alpha location", "alpha stack"),
      valid_error_message(
          20, 0, 1, 0, kErrorTruncDetails, "beta details", "beta location", "beta stack")};
  const auto first = valid_error_message_batch(first_messages);

  ASSERT_TRUE(shim.send_error_message_batch(first, /*sim_time_us=*/100'000).has_value());
  const auto first_msg = receive_from_shim(core);
  EXPECT_EQ(first_msg.envelope.sequence, 1u);
  EXPECT_EQ(first_msg.envelope.payload_bytes, 4656u);  // 8 + 2*2324.
  ASSERT_EQ(first_msg.payload.size(), 4656u);
  EXPECT_EQ(std::memcmp(first_msg.payload.data(), &first, 4656), 0);

  const std::array<error_message, 4> second_messages{
      valid_error_message(
          50, 1, 1, 1, kErrorTruncLocation, "delta details", "delta location", "delta stack"),
      valid_error_message(60,
                          0,
                          0,
                          0,
                          kErrorTruncDetails | kErrorTruncCallStack,
                          "epsilon details",
                          "epsilon location",
                          "epsilon stack"),
      valid_error_message(
          70, 1, 0, 1, kErrorTruncCallStack, "zeta details", "zeta location", "zeta stack"),
      valid_error_message(80, 0, 1, 1, 0, "eta details", "eta location", "eta stack")};
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
  std::memcpy(
      boot_bytes_before.data(), region.backend_to_core.payload.data(), sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;

  const std::array<error_message, 1> messages{
      valid_error_message(100, 1, 0, 1, 0, "detail", "location", "stack")};
  const auto batch = valid_error_message_batch(messages);

  auto first_attempt = shim_or->send_error_message_batch(batch, /*sim_time_us=*/100'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  EXPECT_EQ(first_attempt.error().offending_field_name, "lane");

  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(
      std::memcmp(
          region.backend_to_core.payload.data(), boot_bytes_before.data(), sizeof(boot_descriptor)),
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
      valid_error_message(100, 1, 0, 1, 0, "first detail", "first location", "first call stack"),
      valid_error_message(200,
                          0,
                          1,
                          0,
                          kErrorTruncDetails,
                          "second detail",
                          "second location",
                          "second call stack"),
      valid_error_message(300,
                          1,
                          0,
                          1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail",
                          "third location",
                          "third call stack")};
  const auto first_batch = valid_error_message_batch(first_messages);
  const std::array<error_message, 1> second_messages{
      valid_error_message(400,
                          0,
                          1,
                          1,
                          kErrorTruncDetails | kErrorTruncLocation | kErrorTruncCallStack,
                          "delta detail",
                          "delta location",
                          "delta call stack")};
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
  EXPECT_EQ(std::memcmp(first_a.payload.data(), first_b.payload.data(), first_a.payload.size()), 0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(), second_b.payload.data(), second_a.payload.size()),
            0);
}

// ============================================================================
// Cycle 12 — C HAL ABI plumbing + HAL_GetFPGATime (8 tests).
// ============================================================================

// C12-1. The process-global shim accessor defaults to nullptr before
// any install_global call. The unconditional install_global(nullptr)
// precondition makes the test self-contained even if the suite is
// reshuffled or a prior test failed to clean up.
TEST(ShimCoreCurrent, DefaultsToNullptrBeforeAnyInstallCall) {
  shim_core::install_global(nullptr);

  auto* p = shim_core::current();

  EXPECT_EQ(p, nullptr);
}

// C12-2. install_global(p) makes current() == p. The pre-clear ensures
// the test starts from null state regardless of suite ordering; the
// post-clear leaves null state for downstream tests.
TEST(ShimCoreCurrent, AfterInstallGlobalReturnsTheInstalledPointer) {
  shim_core::install_global(nullptr);

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  shim_core::install_global(&shim);

  auto* p = shim_core::current();
  EXPECT_EQ(p, &shim);

  shim_core::install_global(nullptr);
}

// C12-3. install_global(nullptr) clears current(). Catches the
// "if (p) static_ = p" bug where the clear path is silently dropped,
// leaving a dangling pointer to a destroyed shim — a critical
// correctness bug that the threading cycle inherits.
TEST(ShimCoreCurrent, InstallGlobalNullptrClearsCurrent) {
  shim_core::install_global(nullptr);

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_core::install_global(&shim);

  shim_core::install_global(nullptr);

  auto* p = shim_core::current();
  EXPECT_EQ(p, nullptr);
}

// C12-4. With no shim installed, HAL_GetFPGATime sets *status to
// kHalHandleError and returns 0. The status sentinel 999 catches a
// "function only writes status on the success path" bug.
TEST(HalGetFPGATime, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  std::uint64_t t = HAL_GetFPGATime(&status);

  EXPECT_EQ(t, std::uint64_t{0});
  EXPECT_EQ(status, kHalHandleError);
}

// C12-5. With shim installed but cache empty (post-boot, pre-first-
// clock_state), HAL_GetFPGATime returns 0 with kHalSuccess — matches
// the WPILib contract that HAL_GetFPGATime always succeeds, plus the
// sim-authoritative "sim_time starts at 0" model. Confirms the call
// is a pure read with no cache mutation.
TEST(HalGetFPGATime, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_clock_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  std::uint64_t t = HAL_GetFPGATime(&status);

  EXPECT_EQ(t, std::uint64_t{0});
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// C12-6. End-to-end: a clock_state envelope arrives over the
// transport, the shim caches it, and HAL_GetFPGATime returns the
// cached sim_time_us with kHalSuccess. The magic value
// 0xDEAD'BEEF'CAFE'BABE has no zero bytes and exercises the full
// 64-bit width.
TEST(HalGetFPGATime, WithCachedClockStateReturnsItsSimTimeUsAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  constexpr std::uint64_t kMagicSimTime = 0xDEAD'BEEF'CAFE'BABEull;
  const auto state = valid_clock_state(kMagicSimTime);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), kMagicSimTime));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());

  std::int32_t status = 999;
  std::uint64_t t = HAL_GetFPGATime(&status);

  EXPECT_EQ(t, kMagicSimTime);
  EXPECT_EQ(status, kHalSuccess);
}

// C12-7. Latest-wins across two sequenced updates, observed through
// the C ABI with HAL_GetFPGATime calls INTERLEAVED between the polls.
// The interleaved structure is what catches "HAL_GetFPGATime caches
// its own copy on first read" — a structure that calls only after
// both polls would not.
TEST(HalGetFPGATime, LatestWinsAcrossTwoUpdatesReturnsTheMostRecentSimTimeUs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // First poll → first read.
  const auto state_1 = valid_clock_state(100'000);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_1), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  std::uint64_t t1 = HAL_GetFPGATime(&status1);
  EXPECT_EQ(t1, std::uint64_t{100'000});
  EXPECT_EQ(status1, kHalSuccess);

  // Second poll → second read.
  const auto state_2 = valid_clock_state(200'000);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_2), 200'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  std::uint64_t t2 = HAL_GetFPGATime(&status2);
  EXPECT_EQ(t2, std::uint64_t{200'000});
  EXPECT_EQ(status2, kHalSuccess);
}

// C12-8. HAL_GetFPGATime is idempotent. Three sequential calls without
// an intervening poll return the same value. Three distinct status
// sentinels (999/888/777) ensure each call independently writes
// status, catching a "first-call-only writes status" bug.
TEST(HalGetFPGATime, MultipleCallsWithoutInterveningPollReturnTheSameValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  constexpr std::uint64_t kMagicSimTime = 0xDEAD'BEEF'CAFE'BABEull;
  const auto state = valid_clock_state(kMagicSimTime);
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), kMagicSimTime));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  std::int32_t status2 = 888;
  std::int32_t status3 = 777;
  std::uint64_t t1 = HAL_GetFPGATime(&status1);
  std::uint64_t t2 = HAL_GetFPGATime(&status2);
  std::uint64_t t3 = HAL_GetFPGATime(&status3);

  EXPECT_EQ(t1, kMagicSimTime);
  EXPECT_EQ(t2, kMagicSimTime);
  EXPECT_EQ(t3, kMagicSimTime);
  EXPECT_EQ(status1, kHalSuccess);
  EXPECT_EQ(status2, kHalSuccess);
  EXPECT_EQ(status3, kHalSuccess);
}

// ============================================================================
// Cycle 13 — HAL_GetVinVoltage (4 tests). Mirrors C12 patterns; new
// D-C13-FLOAT-TO-DOUBLE-CAST is the only new design decision.
// ============================================================================

// C13-1. With no shim installed, HAL_GetVinVoltage sets *status to
// kHalHandleError and returns 0.0. Per-function mirror of C12-4.
TEST(HalGetVinVoltage, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  double v = HAL_GetVinVoltage(&status);

  EXPECT_EQ(v, 0.0);
  EXPECT_EQ(status, kHalHandleError);
}

// C13-2. With shim installed but cache empty, HAL_GetVinVoltage
// returns 0.0 with kHalSuccess. Per-function mirror of C12-5 for
// latest_power_state_.
TEST(HalGetVinVoltage, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_power_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  double v = HAL_GetVinVoltage(&status);

  EXPECT_EQ(v, 0.0);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_power_state().has_value());
}

// C13-3. With cached power_state, HAL_GetVinVoltage returns the
// vin_v field widened to double. The fixture values 12.5 / 99.25 /
// 6.75 are pairwise distinct AND exactly representable in IEEE-754
// single AND double precision (25/2, 397/4, 27/4 — all dyadic
// rationals), so static_cast<double>(12.5f) == 12.5 holds bit-for-bit
// and the test asserts exact equality rather than tolerance
// (D-C13-FLOAT-TO-DOUBLE-CAST: float→double widening is lossless
// for finite values). Wrong-field bug (reading vin_a or
// brownout_voltage_v) fails because the values are pairwise distinct.
TEST(HalGetVinVoltage, WithCachedPowerStateReturnsItsVinVoltageAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_power_state(/*vin_v=*/12.5f,
                                       /*vin_a=*/99.25f,
                                       /*brownout_voltage_v=*/6.75f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());

  std::int32_t status = 999;
  double v = HAL_GetVinVoltage(&status);

  EXPECT_EQ(v, 12.5);
  EXPECT_EQ(status, kHalSuccess);
}

// C13-4. Latest-wins across two sequenced power_state updates,
// with HAL_GetVinVoltage calls INTERLEAVED between the polls.
// Mirror of C12-7: the interleaved structure is what catches
// "caches-on-first-call" bugs. The two pairs use distinct vin_a /
// brownout values across the two updates so a wrong-field bug also
// changes between calls and would not coincidentally pass.
// 11.0 and 13.5 are both exactly representable in IEEE-754 single
// and double precision (11/1 and 27/2).
TEST(HalGetVinVoltage, LatestWinsAcrossTwoUpdatesReturnsTheMostRecentVinVoltage) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // First poll → first read.
  const auto state_1 = valid_power_state(/*vin_v=*/11.0f,
                                         /*vin_a=*/1.0f,
                                         /*brownout_voltage_v=*/6.0f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_1), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  double v1 = HAL_GetVinVoltage(&status1);
  EXPECT_EQ(v1, 11.0);
  EXPECT_EQ(status1, kHalSuccess);

  // Second poll → second read.
  const auto state_2 = valid_power_state(/*vin_v=*/13.5f,
                                         /*vin_a=*/2.0f,
                                         /*brownout_voltage_v=*/7.0f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_2), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  double v2 = HAL_GetVinVoltage(&status2);
  EXPECT_EQ(v2, 13.5);
  EXPECT_EQ(status2, kHalSuccess);
}

// ============================================================================
// Cycle 14 — HAL_GetVinCurrent (4 tests). Pure mirror of cycle 13;
// no new design decisions; D-C13-FLOAT-TO-DOUBLE-CAST inherits.
// ============================================================================

// C14-1. With no shim installed, HAL_GetVinCurrent sets *status to
// kHalHandleError and returns 0.0.
TEST(HalGetVinCurrent, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  double a = HAL_GetVinCurrent(&status);

  EXPECT_EQ(a, 0.0);
  EXPECT_EQ(status, kHalHandleError);
}

// C14-2. With shim installed but cache empty, HAL_GetVinCurrent
// returns 0.0 with kHalSuccess.
TEST(HalGetVinCurrent, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_power_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  double a = HAL_GetVinCurrent(&status);

  EXPECT_EQ(a, 0.0);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_power_state().has_value());
}

// C14-3. With cached power_state, HAL_GetVinCurrent returns vin_a
// widened to double. Fixture values 7.5 / 42.25 / 6.5 are
// deliberately different from C13-3's 12.5 / 99.25 / 6.75 so a
// "copy-pasted HAL_GetVinVoltage's body and forgot to change vin_v
// to vin_a" bug fails loudly: against C14-3's vin_v=7.5f the
// wrong-field read returns 7.5 != 42.25; against C13-3's vin_v=12.5f
// it returns 12.5 != 42.25. Either way caught.
TEST(HalGetVinCurrent, WithCachedPowerStateReturnsItsVinCurrentAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_power_state(/*vin_v=*/7.5f,
                                       /*vin_a=*/42.25f,
                                       /*brownout_voltage_v=*/6.5f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());

  std::int32_t status = 999;
  double a = HAL_GetVinCurrent(&status);

  EXPECT_EQ(a, 42.25);
  EXPECT_EQ(status, kHalSuccess);
}

// C14-4. Latest-wins across two power_state updates with
// HAL_GetVinCurrent calls interleaved between the polls. The two
// updates also change vin_v / brownout, so a wrong-field read also
// changes between calls and the test does not coincidentally pass.
// 3.5 (= 7/2) and 8.25 (= 33/4) are exactly representable in IEEE-
// 754 single and double precision.
TEST(HalGetVinCurrent, LatestWinsAcrossTwoUpdatesReturnsTheMostRecentVinCurrent) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // First poll → first read.
  const auto state_1 = valid_power_state(/*vin_v=*/10.0f,
                                         /*vin_a=*/3.5f,
                                         /*brownout_voltage_v=*/6.0f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_1), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  double a1 = HAL_GetVinCurrent(&status1);
  EXPECT_EQ(a1, 3.5);
  EXPECT_EQ(status1, kHalSuccess);

  // Second poll → second read.
  const auto state_2 = valid_power_state(/*vin_v=*/12.0f,
                                         /*vin_a=*/8.25f,
                                         /*brownout_voltage_v=*/7.0f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_2), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  double a2 = HAL_GetVinCurrent(&status2);
  EXPECT_EQ(a2, 8.25);
  EXPECT_EQ(status2, kHalSuccess);
}

// ============================================================================
// Cycle 15 — HAL_Bool parity + HAL_GetBrownedOut (4 tests).
// hal_bool changed from uint32_t → int32_t (D-C15-HAL-BOOL-SIGNED-
// PARITY) to match WPILib's HAL_Bool typedef. HAL_Bool is now exported
// from hal_c.h as a transparent int32_t alias (D-C15-HAL-BOOL-
// TYPEDEF-IN-HAL-C-H). HAL_GetBrownedOut is the first HAL_Bool-
// returning C ABI surface.
// ============================================================================

// C15-1. With no shim installed, HAL_GetBrownedOut returns 0 with
// kHalHandleError. Per-function mirror of C12-4 / C13-1 / C14-1.
TEST(HalGetBrownedOut, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetBrownedOut(&status);

  EXPECT_EQ(b, 0);
  EXPECT_EQ(status, kHalHandleError);
}

// C15-2. With shim installed but cache empty, HAL_GetBrownedOut
// returns 0 with kHalSuccess. The WPILib contract is "always
// succeeds"; surfacing kHalHandleError on empty cache would mislead
// robot bootstrap as a brownout trip.
TEST(HalGetBrownedOut, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_clock_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetBrownedOut(&status);

  EXPECT_EQ(b, 0);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// C15-3. With cached clock_state where browned_out=1 and ALL FOUR
// sibling hal_bool fields are 0, HAL_GetBrownedOut returns 1 with
// kHalSuccess. The all-siblings-zero fixture closes wrong-field bugs
// directly: a shim that erroneously reads system_active /
// system_time_valid / fpga_button_latched / rsl_state instead of
// browned_out returns 0, failing the b == 1 assertion.
TEST(HalGetBrownedOut, WithCachedClockStateBrownedOutTrueReturnsOneAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.system_active = 0;
  state.browned_out = 1;  // distinguished value
  state.system_time_valid = 0;
  state.fpga_button_latched = 0;
  state.rsl_state = 0;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetBrownedOut(&status);

  EXPECT_EQ(b, 1);
  EXPECT_EQ(status, kHalSuccess);
}

// C15-4. Latest-wins across two clock_state updates with
// HAL_GetBrownedOut calls interleaved between polls. The two updates
// invert browned_out vs. all four sibling hal_bool fields, so a
// "shim reads wrong hal_bool field" bug produces the wrong observable
// at every step in opposite directions, AND a "caches-on-first-call"
// bug shows b2 == 0 instead of expected 1.
TEST(HalGetBrownedOut, LatestWinsAcrossTwoUpdatesIsolatesBrownedOutFromOtherHalBoolFields) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // Step 1: browned_out=0 while ALL siblings=1.
  auto state_1 = valid_clock_state(/*sim_time_us=*/50'000);
  state_1.system_active = 1;
  state_1.browned_out = 0;
  state_1.system_time_valid = 1;
  state_1.fpga_button_latched = 1;
  state_1.rsl_state = 1;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_1), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  HAL_Bool b1 = HAL_GetBrownedOut(&status1);
  EXPECT_EQ(b1, 0);
  EXPECT_EQ(status1, kHalSuccess);

  // Step 2: browned_out=1 while ALL siblings=0 (inverted).
  auto state_2 = valid_clock_state(/*sim_time_us=*/100'000);
  state_2.system_active = 0;
  state_2.browned_out = 1;
  state_2.system_time_valid = 0;
  state_2.fpga_button_latched = 0;
  state_2.rsl_state = 0;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_2), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  HAL_Bool b2 = HAL_GetBrownedOut(&status2);
  EXPECT_EQ(b2, 1);
  EXPECT_EQ(status2, kHalSuccess);
}

// ============================================================================
// Cycle 16 — HAL_GetBrownoutVoltage (4 tests). Pure mirror of cycles
// 13/14 against the third (and final) power_state float field; closes
// the power_state read surface. No new design decisions.
// ============================================================================

// C16-1. With no shim installed, HAL_GetBrownoutVoltage returns 0.0
// with kHalHandleError.
TEST(HalGetBrownoutVoltage, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  double v = HAL_GetBrownoutVoltage(&status);

  EXPECT_EQ(v, 0.0);
  EXPECT_EQ(status, kHalHandleError);
}

// C16-2. With shim installed but cache empty, HAL_GetBrownoutVoltage
// returns 0.0 with kHalSuccess.
TEST(HalGetBrownoutVoltage, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_power_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  double v = HAL_GetBrownoutVoltage(&status);

  EXPECT_EQ(v, 0.0);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_power_state().has_value());
}

// C16-3. With cached power_state, HAL_GetBrownoutVoltage returns
// brownout_voltage_v widened to double. Fixture values 14.5 / 2.5 /
// 6.875 are pairwise distinct AND deliberately different from
// cycle-13's 12.5/99.25/6.75 and cycle-14's 7.5/42.25/6.5 so a
// "copy-pasted HAL_GetVinVoltage's body and forgot to change the
// field" bug fails: wrong-field reads of vin_v return 14.5 != 6.875,
// and wrong-field reads of vin_a return 2.5 != 6.875. All three
// values are exactly representable in IEEE-754 single AND double
// precision (29/2, 5/2, 55/8). All three positional args supplied
// explicitly so the helper's defaults (12.5/2.0/6.8) are NOT in
// effect — round-1 reviewer required change.
TEST(HalGetBrownoutVoltage, WithCachedPowerStateReturnsItsBrownoutVoltageAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_power_state(/*vin_v=*/14.5f,
                                       /*vin_a=*/2.5f,
                                       /*brownout_voltage_v=*/6.875f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_power_state().has_value());

  std::int32_t status = 999;
  double v = HAL_GetBrownoutVoltage(&status);

  EXPECT_EQ(v, 6.875);
  EXPECT_EQ(status, kHalSuccess);
}

// C16-4. Latest-wins across two power_state updates with
// HAL_GetBrownoutVoltage calls interleaved between polls. The two
// updates also change vin_v / vin_a, so a wrong-field bug shifts
// between calls and the test does not coincidentally pass. 6.25
// (= 25/4) and 6.5 (= 13/2) are exactly representable in IEEE-754
// single and double.
TEST(HalGetBrownoutVoltage, LatestWinsAcrossTwoUpdatesReturnsTheMostRecentBrownoutVoltage) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // First poll → first read.
  const auto state_1 = valid_power_state(/*vin_v=*/10.0f,
                                         /*vin_a=*/4.0f,
                                         /*brownout_voltage_v=*/6.25f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_1), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  double v1 = HAL_GetBrownoutVoltage(&status1);
  EXPECT_EQ(v1, 6.25);
  EXPECT_EQ(status1, kHalSuccess);

  // Second poll → second read.
  const auto state_2 = valid_power_state(/*vin_v=*/12.5f,
                                         /*vin_a=*/5.5f,
                                         /*brownout_voltage_v=*/6.5f);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::power_state, bytes_of(state_2), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  double v2 = HAL_GetBrownoutVoltage(&status2);
  EXPECT_EQ(v2, 6.5);
  EXPECT_EQ(status2, kHalSuccess);
}

// ============================================================================
// Cycle 17 — clock_state HAL_Bool readers (helper-lifted) + 4 new
// surfaces. HAL_GetBrownedOut (cycle 15) was refactored to call the
// new shared `clock_state_hal_bool_read` helper; these 4 tests pin
// the per-wrapper "passes the correct &clock_state::field to the
// helper" contract for the four new readers. Cycle-15's existing
// HalGetBrownedOut tests cover the helper's null-shim / empty-cache
// / latest-wins paths centrally.
// ============================================================================

// C17-1. HAL_GetSystemActive returns the system_active field (only
// system_active = 1 in the fixture; all four siblings = 0 so a
// wrong-field read returns 0 instead of expected 1).
TEST(HalGetSystemActive, WithCachedClockStateSystemActiveTrueReturnsOneAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.system_active = 1;  // distinguished
  state.browned_out = 0;
  state.system_time_valid = 0;
  state.fpga_button_latched = 0;
  state.rsl_state = 0;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetSystemActive(&status);

  EXPECT_EQ(b, 1);
  EXPECT_EQ(status, kHalSuccess);
}

// C17-2. HAL_GetSystemTimeValid returns the system_time_valid field.
TEST(HalGetSystemTimeValid, WithCachedClockStateSystemTimeValidTrueReturnsOneAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.system_active = 0;
  state.browned_out = 0;
  state.system_time_valid = 1;  // distinguished
  state.fpga_button_latched = 0;
  state.rsl_state = 0;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetSystemTimeValid(&status);

  EXPECT_EQ(b, 1);
  EXPECT_EQ(status, kHalSuccess);
}

// C17-3. HAL_GetFPGAButton returns the fpga_button_latched field.
// Note: the WPILib function is named HAL_GetFPGAButton but the
// schema field is `fpga_button_latched` — a wrapper that erroneously
// references `&clock_state::fpga_button` would not compile (no such
// field), so the realistic copy-paste-from-sibling bug is caught by
// the all-siblings-zero fixture below.
TEST(HalGetFPGAButton, WithCachedClockStateFpgaButtonLatchedTrueReturnsOneAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.system_active = 0;
  state.browned_out = 0;
  state.system_time_valid = 0;
  state.fpga_button_latched = 1;  // distinguished
  state.rsl_state = 0;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetFPGAButton(&status);

  EXPECT_EQ(b, 1);
  EXPECT_EQ(status, kHalSuccess);
}

// C17-4. HAL_GetRSLState returns the rsl_state field.
TEST(HalGetRSLState, WithCachedClockStateRslStateTrueReturnsOneAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.system_active = 0;
  state.browned_out = 0;
  state.system_time_valid = 0;
  state.fpga_button_latched = 0;
  state.rsl_state = 1;  // distinguished
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  HAL_Bool b = HAL_GetRSLState(&status);

  EXPECT_EQ(b, 1);
  EXPECT_EQ(status, kHalSuccess);
}

// ============================================================================
// Cycle 18 — HAL_GetCommsDisableCount (5 tests). Last clock_state
// reader; closes the entire clock_state read surface. Schema field is
// uint32_t; WPILib returns int32_t; shim casts at the C ABI seam per
// D-C18-UINT32-TO-INT32-CAST. Test count is 5 (instead of cycle-13/14/
// 16's 4) because the cast-wraparound branch warrants its own test
// separate from the field-read correctness test, per round-1 reviewer.
// ============================================================================

// C18-1. With no shim installed, HAL_GetCommsDisableCount returns 0
// with kHalHandleError.
TEST(HalGetCommsDisableCount, WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  std::int32_t n = HAL_GetCommsDisableCount(&status);

  EXPECT_EQ(n, 0);
  EXPECT_EQ(status, kHalHandleError);
}

// C18-2. With shim installed but cache empty, HAL_GetCommsDisableCount
// returns 0 with kHalSuccess.
TEST(HalGetCommsDisableCount, WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_clock_state().has_value());

  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  std::int32_t n = HAL_GetCommsDisableCount(&status);

  EXPECT_EQ(n, 0);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_FALSE(shim.latest_clock_state().has_value());
}

// C18-3. With cached clock_state, HAL_GetCommsDisableCount returns
// comms_disable_count. Value 42 is in range for both uint32_t and
// int32_t — the cast at the seam is identity. Wrong-field hygiene:
// sim_time_us=50'000 (helper default) != 42; hal_bool fields are
// 0 or 1 != 42. So a wrong-field read fails the n == 42 assertion.
TEST(HalGetCommsDisableCount, WithCachedClockStateReturnsItsCommsDisableCountAndSetsSuccessStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.comms_disable_count = 42;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_clock_state().has_value());

  std::int32_t status = 999;
  std::int32_t n = HAL_GetCommsDisableCount(&status);

  EXPECT_EQ(n, 42);
  EXPECT_EQ(status, kHalSuccess);
}

// C18-4. With cached clock_state and comms_disable_count > INT32_MAX,
// HAL_GetCommsDisableCount returns the wraparound cast value per
// D-C18-UINT32-TO-INT32-CAST. The expected value is spelled as
// `static_cast<int32_t>(std::uint32_t{0xCAFEBABEu})` rather than the
// hardcoded -889'275'714 so the assertion reads as a statement of
// the C++20 modular-conversion rule.
TEST(HalGetCommsDisableCount, WithCachedClockStateAndCountAboveInt32MaxReturnsWraparoundCastValue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_clock_state(/*sim_time_us=*/50'000);
  state.comms_disable_count = 0xCAFEBABEu;  // > INT32_MAX
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  std::int32_t n = HAL_GetCommsDisableCount(&status);

  EXPECT_EQ(n, static_cast<std::int32_t>(std::uint32_t{0xCAFEBABEu}));
  EXPECT_EQ(status, kHalSuccess);
}

// C18-5. Latest-wins across two clock_state updates with
// HAL_GetCommsDisableCount calls interleaved between the polls. The
// values 7 and 42 are small, in-range, and distinct. Latest-wins is
// a cache-management property, not a cast-behavior property — the
// wraparound case is C18-4's job.
TEST(HalGetCommsDisableCount, LatestWinsAcrossTwoUpdatesReturnsTheMostRecentCount) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  // First poll → first read.
  auto state_1 = valid_clock_state(/*sim_time_us=*/50'000);
  state_1.comms_disable_count = 7;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_1), 50'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status1 = 999;
  std::int32_t n1 = HAL_GetCommsDisableCount(&status1);
  EXPECT_EQ(n1, 7);
  EXPECT_EQ(status1, kHalSuccess);

  // Second poll → second read.
  auto state_2 = valid_clock_state(/*sim_time_us=*/100'000);
  state_2.comms_disable_count = 42;
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state_2), 100'000));
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status2 = 888;
  std::int32_t n2 = HAL_GetCommsDisableCount(&status2);
  EXPECT_EQ(n2, 42);
  EXPECT_EQ(status2, kHalSuccess);
}

// ============================================================================
// Cycle 19 — HAL_SendError + pending outbound error buffering.
// First C HAL ABI write surface. HAL_SendError constructs an
// error_message and appends it to shim_core's fixed-capacity pending
// buffer; the integrator explicitly flushes that buffer at tick
// boundary.
// ============================================================================

// C19-1. With no shim installed, HAL_SendError returns kHalHandleError.
TEST(HalSendError, WithNoShimInstalledReturnsHandleError) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  const std::int32_t r = HAL_SendError(/*isError=*/1,
                                       /*errorCode=*/100,
                                       /*isLVCode=*/0,
                                       "details",
                                       "loc",
                                       "stack",
                                       /*printMsg=*/1);

  EXPECT_EQ(r, kHalHandleError);
}

// C19-2. enqueue_error appends a fully-formed message to the pending buffer.
TEST(ShimCoreEnqueueError, AppendsMessageToPendingBuffer) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_EQ(shim.pending_error_messages().size(), 0u);

  const auto msg = valid_error_message(/*error_code=*/42,
                                       /*severity=*/1,
                                       /*is_lv_code=*/0,
                                       /*print_msg=*/1,
                                       /*truncation_flags=*/0,
                                       "details",
                                       "loc",
                                       "stack");
  shim.enqueue_error(msg);

  ASSERT_EQ(shim.pending_error_messages().size(), 1u);
  EXPECT_EQ(shim.pending_error_messages()[0], msg);
}

// C19-3. HAL_SendError constructs the expected error_message and enqueues it.
TEST(HalSendError, WithShimInstalledEnqueuesConstructedErrorMessage) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  ASSERT_EQ(shim.pending_error_messages().size(), 0u);

  const std::int32_t r = HAL_SendError(/*isError=*/1,
                                       /*errorCode=*/0xCAFE,
                                       /*isLVCode=*/0,
                                       "the details",
                                       "the location",
                                       "the call stack",
                                       /*printMsg=*/1);

  EXPECT_EQ(r, 0);
  ASSERT_EQ(shim.pending_error_messages().size(), 1u);
  const auto expected = valid_error_message(/*error_code=*/0xCAFE,
                                            /*severity=*/1,
                                            /*is_lv_code=*/0,
                                            /*print_msg=*/1,
                                            /*truncation_flags=*/0,
                                            "the details",
                                            "the location",
                                            "the call stack");
  EXPECT_EQ(shim.pending_error_messages()[0], expected);
}

// C19-4. Long strings are truncated to schema capacity and flagged per field.
TEST(HalSendError, TruncatesLongStringsAndSetsTruncationFlags) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::string details_long(2000, 'A');
  const std::string location_long(500, 'B');
  const std::string call_stack_long(2000, 'C');

  const std::int32_t r = HAL_SendError(/*isError=*/1,
                                       /*errorCode=*/0xBABE,
                                       /*isLVCode=*/0,
                                       details_long.c_str(),
                                       location_long.c_str(),
                                       call_stack_long.c_str(),
                                       /*printMsg=*/1);

  EXPECT_EQ(r, 0);
  ASSERT_EQ(shim.pending_error_messages().size(), 1u);
  const auto& msg = shim.pending_error_messages()[0];
  EXPECT_EQ(msg.truncation_flags, kErrorTruncDetails | kErrorTruncLocation | kErrorTruncCallStack);
  EXPECT_EQ(std::string_view(msg.details.data(), kErrorDetailsLen - 1),
            std::string(kErrorDetailsLen - 1, 'A'));
  EXPECT_EQ(msg.details[kErrorDetailsLen - 1], '\0');
  EXPECT_EQ(std::string_view(msg.location.data(), kErrorLocationLen - 1),
            std::string(kErrorLocationLen - 1, 'B'));
  EXPECT_EQ(msg.location[kErrorLocationLen - 1], '\0');
  EXPECT_EQ(std::string_view(msg.call_stack.data(), kErrorCallStackLen - 1),
            std::string(kErrorCallStackLen - 1, 'C'));
  EXPECT_EQ(msg.call_stack[kErrorCallStackLen - 1], '\0');
}

// C19-5. NULL string pointers are empty strings, not truncation.
TEST(HalSendError, NullStringPointersAreTreatedAsEmptyAndDoNotSetTruncationFlags) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::int32_t r = HAL_SendError(/*isError=*/1,
                                       /*errorCode=*/0x1234,
                                       /*isLVCode=*/0,
                                       /*details=*/nullptr,
                                       /*location=*/nullptr,
                                       /*callStack=*/nullptr,
                                       /*printMsg=*/1);

  EXPECT_EQ(r, 0);
  ASSERT_EQ(shim.pending_error_messages().size(), 1u);
  EXPECT_EQ(shim.pending_error_messages()[0],
            valid_error_message(/*error_code=*/0x1234,
                                /*severity=*/1,
                                /*is_lv_code=*/0,
                                /*print_msg=*/1,
                                /*truncation_flags=*/0,
                                "",
                                "",
                                ""));
}

// C19-6. Overflow drops new messages and keeps the first 8.
TEST(ShimCoreEnqueueError, OverflowDropsNewMessagesAtCapacity) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  for (std::int32_t code = 1; code <= static_cast<std::int32_t>(kMaxErrorsPerBatch); ++code) {
    shim.enqueue_error(valid_error_message(code));
  }
  ASSERT_EQ(shim.pending_error_messages().size(), kMaxErrorsPerBatch);

  shim.enqueue_error(valid_error_message(/*error_code=*/999));

  ASSERT_EQ(shim.pending_error_messages().size(), kMaxErrorsPerBatch);
  EXPECT_EQ(shim.pending_error_messages()[kMaxErrorsPerBatch - 1].error_code,
            static_cast<std::int32_t>(kMaxErrorsPerBatch));
  EXPECT_TRUE(std::ranges::none_of(shim.pending_error_messages(),
                                   [](const error_message& msg) { return msg.error_code == 999; }));
}

// C19-7. Empty flush succeeds without publishing an envelope.
TEST(ShimCoreFlushPendingErrors, EmptyBufferIsSuccessNoOpWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  ASSERT_EQ(shim.pending_error_messages().size(), 0u);

  auto r = shim.flush_pending_errors(/*sim_time_us=*/250'000);

  EXPECT_TRUE(r.has_value());
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(shim.pending_error_messages().size(), 0u);
}

// C19-8. Flush publishes accumulated messages as one error_message_batch.
TEST(ShimCoreFlushPendingErrors, PublishesAccumulatedMessagesAsTickBoundaryEnvelope) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const std::array<error_message, 3> messages{
      valid_error_message(100, 1, 0, 1, 0, "first detail", "first location", "first call stack"),
      valid_error_message(200,
                          0,
                          1,
                          0,
                          kErrorTruncDetails,
                          "second detail",
                          "second location",
                          "second call stack"),
      valid_error_message(300,
                          1,
                          0,
                          1,
                          kErrorTruncLocation | kErrorTruncCallStack,
                          "third detail",
                          "third location",
                          "third call stack")};
  for (const auto& msg : messages) {
    shim.enqueue_error(msg);
  }

  auto r = shim.flush_pending_errors(/*sim_time_us=*/500'000);

  ASSERT_TRUE(r.has_value());
  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::error_message_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 500'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 6980u);
  ASSERT_EQ(msg.payload.size(), 6980u);

  error_message_batch destination{};
  std::memcpy(&destination, msg.payload.data(), msg.payload.size());
  EXPECT_EQ(destination, valid_error_message_batch(messages));
  EXPECT_EQ(shim.pending_error_messages().size(), 0u);
}

// C19-8b. Full-capacity flush handles the 8-message boundary.
TEST(ShimCoreFlushPendingErrors, PublishesFullCapacityBatchAtKMaxErrorsPerBatchBoundary) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  std::array<error_message, kMaxErrorsPerBatch> messages{};
  for (std::size_t i = 0; i < messages.size(); ++i) {
    messages[i] = valid_error_message(static_cast<std::int32_t>(i + 1),
                                      static_cast<hal_bool>(i % 2),
                                      static_cast<hal_bool>((i + 1) % 2),
                                      static_cast<hal_bool>(i % 2),
                                      static_cast<std::uint8_t>(i & 0x7),
                                      "detail",
                                      "location",
                                      "stack");
    shim.enqueue_error(messages[i]);
  }
  ASSERT_EQ(shim.pending_error_messages().size(), kMaxErrorsPerBatch);

  auto r = shim.flush_pending_errors(/*sim_time_us=*/750'000);

  ASSERT_TRUE(r.has_value());
  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::error_message_batch);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 750'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 18600u);
  ASSERT_EQ(msg.payload.size(), 18600u);

  error_message_batch destination{};
  std::memcpy(&destination, msg.payload.data(), msg.payload.size());
  EXPECT_EQ(destination, valid_error_message_batch(messages));
  EXPECT_EQ(shim.pending_error_messages().size(), 0u);
}

// C19-9. Failed flush retains the buffer and leaves the occupied lane intact.
TEST(ShimCoreFlushPendingErrors, RetainsBufferOnTransportFailureAndPropagatesError) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  const auto desc = valid_boot_descriptor();
  auto shim_or = shim_core::make(std::move(endpoint), desc, kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));

  std::vector<std::uint8_t> boot_bytes_before(sizeof(boot_descriptor));
  std::memcpy(
      boot_bytes_before.data(), region.backend_to_core.payload.data(), sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;
  const auto msg0 = valid_error_message(10, 1, 0, 1, 0, "a", "b", "c");
  const auto msg1 = valid_error_message(20, 0, 1, 0, kErrorTruncDetails, "d", "e", "f");
  shim_or->enqueue_error(msg0);
  shim_or->enqueue_error(msg1);
  const std::vector<error_message> pending_before{shim_or->pending_error_messages().begin(),
                                                  shim_or->pending_error_messages().end()};

  auto r = shim_or->flush_pending_errors(/*sim_time_us=*/500'000);

  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(r.error().transport_error.has_value());
  EXPECT_EQ(r.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  ASSERT_EQ(shim_or->pending_error_messages().size(), pending_before.size());
  EXPECT_TRUE(std::ranges::equal(shim_or->pending_error_messages(), pending_before));
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(
      std::memcmp(
          region.backend_to_core.payload.data(), boot_bytes_before.data(), sizeof(boot_descriptor)),
      0);
}

// C19-10. Post-shutdown flush returns the terminal error without touching state.
TEST(ShimCoreFlushPendingErrors, PostShutdownReturnsTerminalErrorWithoutTouchingBufferOrLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());

  const auto msg0 = valid_error_message(10);
  const auto msg1 = valid_error_message(20);
  shim.enqueue_error(msg0);
  shim.enqueue_error(msg1);
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  auto r = shim.flush_pending_errors(/*sim_time_us=*/500'000);

  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(r.error().transport_error.has_value());
  ASSERT_EQ(shim.pending_error_messages().size(), 2u);
  EXPECT_EQ(shim.pending_error_messages()[0], msg0);
  EXPECT_EQ(shim.pending_error_messages()[1], msg1);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Cycle 20 — HAL_CAN_SendMessage + pending outbound CAN buffering.
// HAL_CAN_SendMessage constructs one can_frame and appends it to
// shim_core's fixed-capacity CAN TX buffer; the integrator explicitly
// flushes that buffer at tick boundary.
// ============================================================================

// C20-1. With no shim installed, HAL_CAN_SendMessage sets kHalHandleError.
TEST(HalCanSendMessage, WithNoShimInstalledSetsStatusToHandleErrorAndDoesNotCrash) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  const std::array<std::uint8_t, 8> data{1, 2, 3, 4, 5, 6, 7, 8};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x101, data.data(), 8, 0, &status);

  EXPECT_EQ(status, kHalHandleError);
}

// C20-2. enqueue_can_frame appends a fully-formed frame.
TEST(ShimCoreEnqueueCanFrame, AppendsFrameToPendingBuffer) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_EQ(shim.pending_can_frames().size(), 0u);

  const auto frame = valid_can_frame(0x101, 0, 4, 0xA0);
  shim.enqueue_can_frame(frame);

  ASSERT_EQ(shim.pending_can_frames().size(), 1u);
  EXPECT_EQ(shim.pending_can_frames()[0], frame);
}

// C20-3. HAL_CAN_SendMessage preserves CAN ID flags and copies data bytes.
TEST(HalCanSendMessage, WithShimInstalledEnqueuesConstructedFrame) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::array<std::uint8_t, 8> data{0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(kCanFlagFrameRemote | kCanFlagFrame11Bit | 0x123, data.data(), 8, 0, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(shim.pending_can_frames().size(), 1u);
  can_frame expected{};
  expected.message_id = kCanFlagFrameRemote | kCanFlagFrame11Bit | 0x123;
  expected.data = data;
  expected.data_size = 8;
  EXPECT_EQ(shim.pending_can_frames()[0], expected);
}

// C20-4. dataSize > 8 is invalid and does not enqueue.
TEST(HalCanSendMessage, DataSizeAboveEightSetsInvalidBufferAndDoesNotEnqueue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::array<std::uint8_t, 16> data{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x222, data.data(), 16, 0, &status);

  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-5. NULL data is valid for a zero-length frame.
TEST(HalCanSendMessage, NullDataPointerWithZeroSizeEnqueuesZeroLengthFrame) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x555, nullptr, 0, 0, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(shim.pending_can_frames().size(), 1u);
  can_frame expected{};
  expected.message_id = 0x555;
  expected.data_size = 0;
  EXPECT_EQ(shim.pending_can_frames()[0], expected);
}

// C20-5b. NULL data with a nonzero size is invalid and does not enqueue.
TEST(HalCanSendMessage, NullDataPointerWithNonzeroSizeSetsInvalidBufferAndDoesNotEnqueue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x555, nullptr, 4, 0, &status);

  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-6. Overflow drops new CAN frames and keeps the first 64.
TEST(ShimCoreEnqueueCanFrame, OverflowDropsNewFramesAtCapacity) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  for (std::uint32_t id = 1; id <= kMaxCanFramesPerBatch; ++id) {
    shim.enqueue_can_frame(valid_can_frame(id, 0, 1, 0x10));
  }
  ASSERT_EQ(shim.pending_can_frames().size(), kMaxCanFramesPerBatch);

  shim.enqueue_can_frame(valid_can_frame(999, 0, 1, 0x20));

  ASSERT_EQ(shim.pending_can_frames().size(), kMaxCanFramesPerBatch);
  EXPECT_EQ(shim.pending_can_frames()[kMaxCanFramesPerBatch - 1].message_id, kMaxCanFramesPerBatch);
  EXPECT_TRUE(std::ranges::none_of(shim.pending_can_frames(),
                                   [](const can_frame& frame) { return frame.message_id == 999; }));
}

// C20-7. Empty CAN flush succeeds without publishing an envelope.
TEST(ShimCoreFlushPendingCanFrames, EmptyBufferIsSuccessNoOpWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  ASSERT_EQ(shim.pending_can_frames().size(), 0u);

  auto r = shim.flush_pending_can_frames(/*sim_time_us=*/250'000);

  EXPECT_TRUE(r.has_value());
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-8. Flush publishes accumulated frames as one timestamped CAN batch.
TEST(ShimCoreFlushPendingCanFrames, PublishesAccumulatedFramesAsTickBoundaryEnvelope) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  std::array<can_frame, 3> frames{valid_can_frame(0x101, 0, 4, 0xA0),
                                  valid_can_frame(0x202, 0, 8, 0xB0),
                                  valid_can_frame(0x303, 0, 0, 0xC0)};
  for (const auto& frame : frames) {
    shim.enqueue_can_frame(frame);
  }

  auto r = shim.flush_pending_can_frames(/*sim_time_us=*/500'000);

  ASSERT_TRUE(r.has_value());
  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::can_frame_batch);
  EXPECT_EQ(msg.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 500'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, 64u);
  ASSERT_EQ(msg.payload.size(), 64u);

  for (auto& frame : frames) {
    frame.timestamp_us = 500'000;
  }
  can_frame_batch destination{};
  std::memcpy(&destination, msg.payload.data(), msg.payload.size());
  EXPECT_EQ(destination, valid_can_frame_batch(frames));
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-9. Full-capacity CAN flush handles the 64-frame boundary.
TEST(ShimCoreFlushPendingCanFrames, PublishesFullCapacityBatchAtKMaxCanFramesPerBatchBoundary) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  std::array<can_frame, kMaxCanFramesPerBatch> frames{};
  for (std::size_t i = 0; i < frames.size(); ++i) {
    frames[i] = valid_can_frame(static_cast<std::uint32_t>(i + 1),
                                0,
                                static_cast<std::uint8_t>(i % 9),
                                static_cast<std::uint8_t>(0x20 + i));
    shim.enqueue_can_frame(frames[i]);
  }
  ASSERT_EQ(shim.pending_can_frames().size(), kMaxCanFramesPerBatch);

  auto r = shim.flush_pending_can_frames(/*sim_time_us=*/750'000);

  ASSERT_TRUE(r.has_value());
  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::can_frame_batch);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  EXPECT_EQ(msg.envelope.sim_time_us, 750'000u);
  EXPECT_EQ(msg.envelope.payload_bytes, sizeof(can_frame_batch));
  ASSERT_EQ(msg.payload.size(), sizeof(can_frame_batch));

  for (auto& frame : frames) {
    frame.timestamp_us = 750'000;
  }
  can_frame_batch destination{};
  std::memcpy(&destination, msg.payload.data(), msg.payload.size());
  EXPECT_EQ(destination, valid_can_frame_batch(frames));
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-10. Failed CAN flush retains the buffer and leaves the occupied lane intact.
TEST(ShimCoreFlushPendingCanFrames, RetainsBufferOnTransportFailureAndPropagatesError) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  const auto desc = valid_boot_descriptor();
  auto shim_or = shim_core::make(std::move(endpoint), desc, kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));

  std::vector<std::uint8_t> boot_bytes_before(sizeof(boot_descriptor));
  std::memcpy(
      boot_bytes_before.data(), region.backend_to_core.payload.data(), sizeof(boot_descriptor));
  const auto boot_envelope_before = region.backend_to_core.envelope;
  const auto frame0 = valid_can_frame(0x101, 0, 4, 0xA0);
  const auto frame1 = valid_can_frame(0x202, 0, 8, 0xB0);
  shim_or->enqueue_can_frame(frame0);
  shim_or->enqueue_can_frame(frame1);
  const std::vector<can_frame> pending_before{shim_or->pending_can_frames().begin(),
                                              shim_or->pending_can_frames().end()};

  auto r = shim_or->flush_pending_can_frames(/*sim_time_us=*/500'000);

  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(r.error().transport_error.has_value());
  EXPECT_EQ(r.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  ASSERT_EQ(shim_or->pending_can_frames().size(), pending_before.size());
  EXPECT_TRUE(std::ranges::equal(shim_or->pending_can_frames(), pending_before));
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, boot_envelope_before);
  EXPECT_EQ(
      std::memcmp(
          region.backend_to_core.payload.data(), boot_bytes_before.data(), sizeof(boot_descriptor)),
      0);
}

// C20-11. Post-shutdown CAN flush returns terminal error and retains state.
TEST(ShimCoreFlushPendingCanFrames, PostShutdownReturnsTerminalErrorWithoutTouchingBufferOrLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());

  const auto frame0 = valid_can_frame(0x101);
  const auto frame1 = valid_can_frame(0x202);
  shim.enqueue_can_frame(frame0);
  shim.enqueue_can_frame(frame1);
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  auto r = shim.flush_pending_can_frames(/*sim_time_us=*/500'000);

  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(r.error().transport_error.has_value());
  ASSERT_EQ(shim.pending_can_frames().size(), 2u);
  EXPECT_EQ(shim.pending_can_frames()[0], frame0);
  EXPECT_EQ(shim.pending_can_frames()[1], frame1);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// C20-12. Positive periodMs enqueues one immediate frame in v0.
TEST(HalCanSendMessage, PositivePeriodMsEnqueuesOneImmediateFrameInV0) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::array<std::uint8_t, 8> data{8, 7, 6, 5, 4, 3, 2, 1};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x321, data.data(), 8, 20, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(shim.pending_can_frames().size(), 1u);
  can_frame expected{};
  expected.message_id = 0x321;
  expected.data = data;
  expected.data_size = 8;
  EXPECT_EQ(shim.pending_can_frames()[0], expected);
}

// C20-13. Stop-repeat is not an immediate data frame send.
TEST(HalCanSendMessage, StopRepeatingPeriodDoesNotEnqueueADataFrame) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::array<std::uint8_t, 8> data{1, 1, 2, 3, 5, 8, 13, 21};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x321, data.data(), 8, kHalCanSendPeriodStopRepeating, &status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// C20-14. Other negative period values are invalid and do not enqueue.
TEST(HalCanSendMessage, OtherNegativePeriodSetsInvalidBufferAndDoesNotEnqueue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const std::array<std::uint8_t, 8> data{1, 2, 3, 4, 5, 6, 7, 8};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x321, data.data(), 8, -2, &status);

  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(shim.pending_can_frames().size(), 0u);
}

// ============================================================================
// Cycle 21 — HAL_CAN_OpenStreamSession / HAL_CAN_ReadStreamSession /
// HAL_CAN_CloseStreamSession.
// ============================================================================

namespace {

std::uint32_t open_can_stream(std::uint32_t message_id,
                              std::uint32_t message_id_mask,
                              std::uint32_t max_messages,
                              std::int32_t* status) {
  std::uint32_t handle = 0xFFFF'FFFFu;
  HAL_CAN_OpenStreamSession(&handle, message_id, message_id_mask, max_messages, status);
  return handle;
}

void inject_can_batch(tier1_endpoint& core,
                      shim_core& shim,
                      const can_frame_batch& batch,
                      std::uint64_t sim_time_us = 250'000) {
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::can_frame_batch,
                        active_prefix_bytes(batch),
                        sim_time_us));
  ASSERT_TRUE(shim.poll().has_value());
}

void expect_stream_message_eq(const HAL_CANStreamMessage& actual, const can_frame& expected) {
  EXPECT_EQ(actual.messageID, expected.message_id);
  EXPECT_EQ(actual.timeStamp, expected.timestamp_us);
  EXPECT_EQ(actual.dataSize, expected.data_size);
  EXPECT_EQ(std::memcmp(actual.data, expected.data.data(), expected.data.size()), 0);
}

template <std::size_t N>
void fill_stream_messages_with_sentinel(std::array<HAL_CANStreamMessage, N>& messages,
                                        std::uint8_t sentinel = 0xCC) {
  std::memset(messages.data(), sentinel, sizeof(messages));
}

template <std::size_t N>
void expect_stream_messages_unchanged(const std::array<HAL_CANStreamMessage, N>& actual,
                                      const std::array<HAL_CANStreamMessage, N>& before) {
  EXPECT_EQ(std::memcmp(actual.data(), before.data(), sizeof(actual)), 0);
}

}  // namespace

// C21-1. No installed shim makes open fail with handle error.
TEST(HalCanOpenStreamSession, WithNoShimInstalledSetsHandleErrorAndInvalidHandle) {
  shim_core::install_global(nullptr);

  std::uint32_t handle = 0xABCD'1234u;
  std::int32_t status = 999;
  HAL_CAN_OpenStreamSession(&handle, 0x100, 0x7FF, 4, &status);

  EXPECT_EQ(status, kHalHandleError);
  EXPECT_EQ(handle, 0u);
}

// C21-2. Successful open returns a nonzero stream handle.
TEST(HalCanOpenStreamSession, WithInstalledShimReturnsNonzeroHandle) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_NE(handle, 0u);
}

// C21-3. Zero maxMessages is rejected at the C seam.
TEST(HalCanOpenStreamSession, WithZeroMaxMessagesSetsInvalidBufferAndNoHandle) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::uint32_t handle = 0xABCD'1234u;
  std::int32_t status = 999;
  HAL_CAN_OpenStreamSession(&handle, 0x120, 0x7FF, 0, &status);

  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(handle, 0u);
}

// C21-4. No installed shim makes read fail without touching output.
TEST(HalCanReadStreamSession, WithNoShimInstalledSetsHandleErrorAndDoesNotTouchMessages) {
  shim_core::install_global(nullptr);
  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;

  HAL_CAN_ReadStreamSession(1, messages.data(), 1, &messages_read, &status);

  EXPECT_EQ(status, kHalHandleError);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-5. Invalid handle reports NotAllowed and does not touch output.
TEST(HalCanReadStreamSession, WithInvalidHandleSetsNotAllowedAndDoesNotTouchMessages) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;

  HAL_CAN_ReadStreamSession(0, messages.data(), 1, &messages_read, &status);

  EXPECT_EQ(status, kHalCanNotAllowed);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-6. Valid empty stream returns NoToken and preserves output.
TEST(HalCanReadStreamSession, ValidEmptyStreamReturnsNoTokenAndLeavesMessagesUntouched) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);

  EXPECT_EQ(status, kHalCanNoToken);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-7. Stream reads matching frames in FIFO order and drains them.
TEST(HalCanReadStreamSession, ReturnsMatchingFramesInFifoOrderAndDrainsThem) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x120, 1'000, 4, 0xA0);
  const auto frame1 = valid_can_frame(0x121, 2'000, 8, 0xB0);
  const auto frame2 = valid_can_frame(0x120, 3'000, 2, 0xC0);
  const std::array<can_frame, 3> frames{frame0, frame1, frame2};
  inject_can_batch(core, shim, valid_can_frame_batch(frames));

  std::array<HAL_CANStreamMessage, 2> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 2, &messages_read, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 2u);
  expect_stream_message_eq(messages[0], frame0);
  expect_stream_message_eq(messages[1], frame2);

  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 2, &messages_read, &status);

  EXPECT_EQ(status, kHalCanNoToken);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-8. Partial reads leave unread frames queued.
TEST(HalCanReadStreamSession, PartialReadLeavesUnreadSuffixQueued) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x555, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x555, 1'000, 4, 0x10);
  const auto frame1 = valid_can_frame(0x555, 2'000, 4, 0x20);
  const auto frame2 = valid_can_frame(0x555, 3'000, 4, 0x30);
  const std::array<can_frame, 3> frames{frame0, frame1, frame2};
  inject_can_batch(core, shim, valid_can_frame_batch(frames));

  std::array<HAL_CANStreamMessage, 2> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 1u);
  expect_stream_message_eq(messages[0], frame0);

  fill_stream_messages_with_sentinel(messages);
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 2u);
  expect_stream_message_eq(messages[0], frame1);
  expect_stream_message_eq(messages[1], frame2);
}

// C21-9. Stream queues accumulate across multiple inbound batches.
TEST(HalCanReadStreamSession, AccumulatesFramesAcrossMultipleInboundBatches) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x444, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x444, 1'000, 3, 0xA0);
  const auto frame1 = valid_can_frame(0x444, 2'000, 4, 0xB0);
  const auto frame2 = valid_can_frame(0x444, 3'000, 5, 0xC0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame0}));
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 2>{frame1, frame2}));

  std::array<HAL_CANStreamMessage, 3> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 3, &messages_read, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 3u);
  expect_stream_message_eq(messages[0], frame0);
  expect_stream_message_eq(messages[1], frame1);
  expect_stream_message_eq(messages[2], frame2);
}

// C21-10. Mask zero matches every frame and preserves message IDs.
TEST(HalCanReadStreamSession, MaskZeroMatchesAllFramesWithoutRewritingIds) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0, 0, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x101, 1'000, 1, 0xA0);
  const auto frame1 = valid_can_frame(kCanFlagFrameRemote | 0x202, 2'000, 2, 0xB0);
  const auto frame2 = valid_can_frame(kCanFlagFrame11Bit | 0x303, 3'000, 3, 0xC0);
  const std::array<can_frame, 3> frames{frame0, frame1, frame2};
  inject_can_batch(core, shim, valid_can_frame_batch(frames));

  std::array<HAL_CANStreamMessage, 3> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 3, &messages_read, &status);

  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 3u);
  expect_stream_message_eq(messages[0], frame0);
  expect_stream_message_eq(messages[1], frame1);
  expect_stream_message_eq(messages[2], frame2);
}

// C21-11. Opening a stream does not replay the latest CAN cache.
TEST(HalCanReadStreamSession, DoesNotBackfillFramesReceivedBeforeOpen) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  const auto frame = valid_can_frame(0x120, 1'000, 4, 0xA0);
  const auto batch = valid_can_frame_batch(std::array<can_frame, 1>{frame});
  inject_can_batch(core, shim, batch);
  ASSERT_TRUE(shim.latest_can_frame_batch().has_value());
  EXPECT_EQ(*shim.latest_can_frame_batch(), batch);

  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);

  EXPECT_EQ(status, kHalCanNoToken);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-12. Multiple streams receive independent copies.
TEST(HalCanReadStreamSession, MultipleStreamsReceiveIndependentCopies) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle_a = open_can_stream(0x100, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);
  const std::uint32_t handle_b = open_can_stream(0, 0, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x100, 1'000, 4, 0xA0);
  const auto frame1 = valid_can_frame(0x200, 2'000, 4, 0xB0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 2>{frame0, frame1}));

  std::array<HAL_CANStreamMessage, 2> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle_a, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 1u);
  expect_stream_message_eq(messages[0], frame0);

  fill_stream_messages_with_sentinel(messages);
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle_b, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 2u);
  expect_stream_message_eq(messages[0], frame0);
  expect_stream_message_eq(messages[1], frame1);
}

// C21-13. Overflow drops oldest, reports overrun once, and keeps newest.
TEST(HalCanReadStreamSession, OverflowDropsOldestReportsSessionOverrunAndKeepsNewest) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x321, 0x7FF, 2, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame0 = valid_can_frame(0x321, 1'000, 1, 0xA0);
  const auto frame1 = valid_can_frame(0x321, 2'000, 2, 0xB0);
  const auto frame2 = valid_can_frame(0x321, 3'000, 3, 0xC0);
  inject_can_batch(
      core, shim, valid_can_frame_batch(std::array<can_frame, 3>{frame0, frame1, frame2}));

  std::array<HAL_CANStreamMessage, 2> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalCanSessionOverrun);
  ASSERT_EQ(messages_read, 2u);
  expect_stream_message_eq(messages[0], frame1);
  expect_stream_message_eq(messages[1], frame2);

  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalCanNoToken);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-14. Null output with nonzero read is invalid and does not drain.
TEST(HalCanReadStreamSession, NullMessagesWithNonzeroReadSetsInvalidBufferAndDoesNotDrain) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame = valid_can_frame(0x120, 1'000, 4, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame}));

  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, nullptr, 1, &messages_read, &status);
  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(messages_read, 0u);

  std::array<HAL_CANStreamMessage, 1> messages{};
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 1u);
  expect_stream_message_eq(messages[0], frame);
}

// C21-15. Zero-count reads succeed for null or non-null messages.
TEST(HalCanReadStreamSession, ZeroCountReadSucceedsForNullOrNonNullMessagesWithoutTouchingQueue) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x120, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame = valid_can_frame(0x120, 1'000, 4, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame}));

  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, nullptr, 0, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(messages_read, 0u);

  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 0, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);

  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 1u);
  expect_stream_message_eq(messages[0], frame);
}

// C21-16. Closed handles become unreadable.
TEST(HalCanCloseStreamSession, ClosedHandleIsUnreadable) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x222, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  HAL_CAN_CloseStreamSession(handle);

  std::array<HAL_CANStreamMessage, 1> messages{};
  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);

  EXPECT_EQ(status, kHalCanNotAllowed);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C21-17. Closing one stream does not affect other streams.
TEST(HalCanCloseStreamSession, ClosingOneStreamDoesNotAffectOtherStreams) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  std::int32_t open_status = 999;
  const std::uint32_t handle_a = open_can_stream(0x100, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);
  const std::uint32_t handle_b = open_can_stream(0, 0, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  HAL_CAN_CloseStreamSession(handle_a);

  const auto frame0 = valid_can_frame(0x100, 1'000, 4, 0xA0);
  const auto frame1 = valid_can_frame(0x200, 2'000, 4, 0xB0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 2>{frame0, frame1}));

  std::array<HAL_CANStreamMessage, 2> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle_b, messages.data(), 2, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 2u);
  expect_stream_message_eq(messages[0], frame0);
  expect_stream_message_eq(messages[1], frame1);

  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle_a, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalCanNotAllowed);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// ============================================================================
// Cycle 31 — HAL_CAN_ReceiveMessage one-shot CAN receive.
// ============================================================================

namespace {

struct can_receive_outputs {
  std::uint32_t message_id = 0xAAAA'AAAAu;
  std::array<std::uint8_t, 8> data{};
  std::uint8_t data_size = 0xEE;
  std::uint32_t timestamp = 0xBBBB'BBBBu;
  std::int32_t status = 999;
};

void fill_can_receive_with_sentinel(can_receive_outputs& out) {
  out.message_id = 0xAAAA'AAAAu;
  out.data.fill(0xCC);
  out.data_size = 0xEE;
  out.timestamp = 0xBBBB'BBBBu;
  out.status = 999;
}

void receive_can_message(can_receive_outputs& out, std::uint32_t requested_id, std::uint32_t mask) {
  out.message_id = requested_id;
  HAL_CAN_ReceiveMessage(
      &out.message_id, mask, out.data.data(), &out.data_size, &out.timestamp, &out.status);
}

void expect_can_receive_zeroed(const can_receive_outputs& out, std::int32_t expected_status) {
  EXPECT_EQ(out.status, expected_status);
  EXPECT_EQ(out.message_id, 0u);
  EXPECT_EQ(out.timestamp, 0u);
  EXPECT_EQ(out.data_size, 0u);
  for (std::size_t i = 0; i < out.data.size(); ++i) {
    EXPECT_EQ(out.data[i], 0u) << "byte " << i;
  }
}

void expect_can_receive_eq(const can_receive_outputs& out, const can_frame& expected) {
  EXPECT_EQ(out.status, kHalSuccess);
  EXPECT_EQ(out.message_id, expected.message_id);
  EXPECT_EQ(out.timestamp, expected.timestamp_us);
  EXPECT_EQ(out.data_size, expected.data_size);
  EXPECT_EQ(std::memcmp(out.data.data(), expected.data.data(), expected.data.size()), 0);
}

}  // namespace

// C31-1. No installed shim reports handle error and zeroes outputs.
TEST(HalCanReceiveMessage, WithNoShimInstalledSetsHandleErrorAndZerosOutputs) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0x123, 0x7FF);

  expect_can_receive_zeroed(out, kHalHandleError);
}

// C31-2. Null data with an installed shim is invalid and does not drain receive state.
TEST(HalCanReceiveMessage, NullDataWithInstalledShimSetsInvalidBufferWithoutDraining) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto frame = valid_can_frame(0x123, 4'567, 5, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame}));

  std::uint32_t message_id = 0x123;
  std::uint8_t data_size = 0xEE;
  std::uint32_t timestamp = 0xBBBB'BBBBu;
  std::int32_t status = 999;
  HAL_CAN_ReceiveMessage(&message_id, 0x7FF, nullptr, &data_size, &timestamp, &status);

  EXPECT_EQ(status, kHalCanInvalidBuffer);
  EXPECT_EQ(message_id, 0u);
  EXPECT_EQ(data_size, 0u);
  EXPECT_EQ(timestamp, 0u);

  can_receive_outputs valid{};
  fill_can_receive_with_sentinel(valid);
  receive_can_message(valid, 0x123, 0x7FF);
  expect_can_receive_eq(valid, frame);
}

// C31-3. Empty cache reports message-not-found and zeroes outputs.
TEST(HalCanReceiveMessage, WithShimInstalledButCacheEmptySetsMessageNotFoundAndZerosOutputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_can_frame_batch().has_value());
  shim_global_install_guard guard{shim};

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0x123, 0x7FF);

  expect_can_receive_zeroed(out, kHalCanMessageNotFound);
  EXPECT_FALSE(shim.latest_can_frame_batch().has_value());
}

// C31-4. Empty latest batch reports message-not-found even with a zero mask.
TEST(HalCanReceiveMessage, WithEmptyLatestBatchAndZeroMaskSetsMessageNotFoundAndZerosOutputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  can_frame_batch empty{};
  inject_can_batch(core, shim, empty);

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0, 0);

  expect_can_receive_zeroed(out, kHalCanMessageNotFound);
}

// C31-5. Exact mask success copies frame fields and overwrites message ID.
TEST(HalCanReceiveMessage, WithExactMaskCopiesFrameFieldsAndActualMessageId) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto frame = valid_can_frame(0x123, 4'567, 5, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame}));

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0x123, 0x7FF);

  expect_can_receive_eq(out, frame);
}

// C31-6. Masked receive returns the first matching active-prefix frame.
TEST(HalCanReceiveMessage, WithMaskReturnsFirstMatchingActiveFrame) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto nonmatch = valid_can_frame(0x100, 1'000, 1, 0xA0);
  const auto first_match = valid_can_frame(0x121, 2'000, 2, 0xB0);
  const auto later_match = valid_can_frame(0x12A, 3'000, 3, 0xC0);
  inject_can_batch(
      core,
      shim,
      valid_can_frame_batch(std::array<can_frame, 3>{nonmatch, first_match, later_match}));

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0x120, 0x7F0);

  expect_can_receive_eq(out, first_match);
}

// C31-7. Zero mask returns the first active frame's actual ID.
TEST(HalCanReceiveMessage, WithZeroMaskReturnsFirstActiveFrameActualId) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto frame0 = valid_can_frame(0x555, 1'000, 1, 0xA0);
  const auto frame1 = valid_can_frame(0x666, 2'000, 2, 0xB0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 2>{frame0, frame1}));

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0, 0);

  expect_can_receive_eq(out, frame0);
}

// C31-8. No matching frame reports message-not-found and zeroes outputs.
TEST(HalCanReceiveMessage, WithNoMatchingFrameSetsMessageNotFoundAndZerosOutputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto frame0 = valid_can_frame(0x100, 1'000, 1, 0xA0);
  const auto frame1 = valid_can_frame(0x200, 2'000, 2, 0xB0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 2>{frame0, frame1}));

  can_receive_outputs out{};
  fill_can_receive_with_sentinel(out);
  receive_can_message(out, 0x123, 0x7FF);

  expect_can_receive_zeroed(out, kHalCanMessageNotFound);
}

// C31-9. One-shot receive does not drain latest batch or stream sessions.
TEST(HalCanReceiveMessage, DoesNotDrainLatestBatchOrStreamSessions) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t open_status = 999;
  const std::uint32_t handle = open_can_stream(0x123, 0x7FF, 4, &open_status);
  ASSERT_EQ(open_status, kHalSuccess);

  const auto frame = valid_can_frame(0x123, 4'567, 5, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{frame}));

  can_receive_outputs first{};
  fill_can_receive_with_sentinel(first);
  receive_can_message(first, 0x123, 0x7FF);
  expect_can_receive_eq(first, frame);

  can_receive_outputs second{};
  fill_can_receive_with_sentinel(second);
  receive_can_message(second, 0x123, 0x7FF);
  expect_can_receive_eq(second, frame);

  std::array<HAL_CANStreamMessage, 1> messages{};
  std::uint32_t messages_read = 777;
  std::int32_t status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalSuccess);
  ASSERT_EQ(messages_read, 1u);
  expect_stream_message_eq(messages[0], frame);

  fill_stream_messages_with_sentinel(messages);
  const auto before = messages;
  messages_read = 777;
  status = 999;
  HAL_CAN_ReadStreamSession(handle, messages.data(), 1, &messages_read, &status);
  EXPECT_EQ(status, kHalCanNoToken);
  EXPECT_EQ(messages_read, 0u);
  expect_stream_messages_unchanged(messages, before);
}

// C31-10. Latest-wins returns the newer matching batch.
TEST(HalCanReceiveMessage, LatestWinsReturnsNewerMatchingBatch) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = valid_can_frame(0x123, 1'000, 3, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{first}), 50'000);

  can_receive_outputs first_out{};
  fill_can_receive_with_sentinel(first_out);
  receive_can_message(first_out, 0x123, 0x7FF);
  expect_can_receive_eq(first_out, first);

  const auto second = valid_can_frame(0x123, 2'000, 6, 0xB0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{second}), 100'000);

  can_receive_outputs second_out{};
  fill_can_receive_with_sentinel(second_out);
  receive_can_message(second_out, 0x123, 0x7FF);
  expect_can_receive_eq(second_out, second);
}

// C31-11. Latest-wins does not fall back to stale older matches.
TEST(HalCanReceiveMessage, LatestWinsDoesNotFallBackToStaleOlderMatches) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = valid_can_frame(0x123, 1'000, 3, 0xA0);
  inject_can_batch(core, shim, valid_can_frame_batch(std::array<can_frame, 1>{first}), 50'000);

  can_receive_outputs first_out{};
  fill_can_receive_with_sentinel(first_out);
  receive_can_message(first_out, 0x123, 0x7FF);
  expect_can_receive_eq(first_out, first);

  can_frame_batch empty{};
  inject_can_batch(core, shim, empty, 100'000);

  can_receive_outputs second_out{};
  fill_can_receive_with_sentinel(second_out);
  receive_can_message(second_out, 0x123, 0x7FF);
  expect_can_receive_zeroed(second_out, kHalCanMessageNotFound);
}

// ============================================================================
// Cycle 22 — HAL_CAN_GetCANStatus.
// ============================================================================

namespace {

struct can_status_outputs {
  float percent_bus_utilization = -1.0f;
  std::uint32_t bus_off_count = 0xAAAA'AAAAu;
  std::uint32_t tx_full_count = 0xBBBB'BBBBu;
  std::uint32_t receive_error_count = 0xCCCC'CCCCu;
  std::uint32_t transmit_error_count = 0xDDDD'DDDDu;
};

void read_can_status_outputs(can_status_outputs& out, std::int32_t& status) {
  HAL_CAN_GetCANStatus(&out.percent_bus_utilization,
                       &out.bus_off_count,
                       &out.tx_full_count,
                       &out.receive_error_count,
                       &out.transmit_error_count,
                       &status);
}

void expect_can_status_outputs_eq(const can_status_outputs& out, const can_status& expected) {
  EXPECT_EQ(out.percent_bus_utilization, expected.percent_bus_utilization);
  EXPECT_EQ(out.bus_off_count, expected.bus_off_count);
  EXPECT_EQ(out.tx_full_count, expected.tx_full_count);
  EXPECT_EQ(out.receive_error_count, expected.receive_error_count);
  EXPECT_EQ(out.transmit_error_count, expected.transmit_error_count);
}

void expect_can_status_outputs_zero(const can_status_outputs& out) {
  EXPECT_EQ(out.percent_bus_utilization, 0.0f);
  EXPECT_EQ(out.bus_off_count, 0u);
  EXPECT_EQ(out.tx_full_count, 0u);
  EXPECT_EQ(out.receive_error_count, 0u);
  EXPECT_EQ(out.transmit_error_count, 0u);
}

void inject_can_status(tier1_endpoint& core,
                       shim_core& shim,
                       const can_status& state,
                       std::uint64_t sim_time_us = 50'000) {
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::can_status, bytes_of(state), sim_time_us));
  ASSERT_TRUE(shim.poll().has_value());
}

}  // namespace

// C22-1. No installed shim sets handle error and zeroes every output.
TEST(HalCanGetCanStatus, WithNoShimInstalledSetsHandleErrorAndZerosOutputs) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  can_status_outputs out{};
  std::int32_t status = 999;
  read_can_status_outputs(out, status);

  EXPECT_EQ(status, kHalHandleError);
  expect_can_status_outputs_zero(out);
}

// C22-2. Empty cache succeeds, zeroes outputs, and does not populate cache.
TEST(HalCanGetCanStatus, WithShimInstalledButCacheEmptySetsSuccessAndZerosOutputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_can_status().has_value());
  shim_global_install_guard guard{shim};

  can_status_outputs out{};
  std::int32_t status = 999;
  read_can_status_outputs(out, status);

  EXPECT_EQ(status, kHalSuccess);
  expect_can_status_outputs_zero(out);
  EXPECT_FALSE(shim.latest_can_status().has_value());
}

// C22-3. Cached can_status copies every field in WPILib parameter order.
TEST(HalCanGetCanStatus, WithCachedCanStatusCopiesEveryFieldInParameterOrder) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_can_status(0.625f, 11, 22, 33, 44);
  inject_can_status(core, shim, state);
  ASSERT_TRUE(shim.latest_can_status().has_value());

  can_status_outputs out{};
  std::int32_t status = 999;
  read_can_status_outputs(out, status);

  EXPECT_EQ(status, kHalSuccess);
  expect_can_status_outputs_eq(out, state);
}

// C22-4. Latest-wins is observed across interleaved polls and reads.
TEST(HalCanGetCanStatus, LatestWinsAcrossTwoUpdatesReturnsMostRecentStatus) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = valid_can_status(0.125f, 1, 2, 3, 4);
  inject_can_status(core, shim, first, 50'000);

  can_status_outputs out1{};
  std::int32_t status1 = 999;
  read_can_status_outputs(out1, status1);
  EXPECT_EQ(status1, kHalSuccess);
  expect_can_status_outputs_eq(out1, first);

  const auto second = valid_can_status(0.875f, 9, 8, 7, 6);
  inject_can_status(core, shim, second, 100'000);

  can_status_outputs out2{};
  std::int32_t status2 = 888;
  read_can_status_outputs(out2, status2);
  EXPECT_EQ(status2, kHalSuccess);
  expect_can_status_outputs_eq(out2, second);
}

// ============================================================================
// Cycle 23 — Driver Station scalar reads.
// ============================================================================

namespace {

std::uint32_t control_word_bits(const HAL_ControlWord& word) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &word, sizeof(bits));
  return bits;
}

HAL_ControlWord control_word_with_raw_bits(std::uint32_t bits) {
  HAL_ControlWord word{};
  std::memcpy(&word, &bits, sizeof(bits));
  return word;
}

void expect_control_word_bits(const HAL_ControlWord& word, std::uint32_t expected_bits) {
  EXPECT_EQ(control_word_bits(word), expected_bits);
}

void inject_ds_state(tier1_endpoint& core,
                     shim_core& shim,
                     const ds_state& state,
                     std::uint64_t sim_time_us = 50'000) {
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), sim_time_us));
  ASSERT_TRUE(shim.poll().has_value());
}

}  // namespace

// C23-1. No installed shim returns handle error and zeroes the full word.
TEST(HalGetControlWord, WithNoShimInstalledReturnsHandleErrorAndZerosWord) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_ControlWord word = control_word_with_raw_bits(0xFFFF'FFFFu);
  const std::int32_t status = HAL_GetControlWord(&word);

  EXPECT_EQ(status, kHalHandleError);
  expect_control_word_bits(word, 0u);
}

// C23-2. Empty ds_state cache succeeds, zeroes output, and stays empty.
TEST(HalGetControlWord, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosWord) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_ControlWord word = control_word_with_raw_bits(0xFFFF'FFFFu);
  const std::int32_t status = HAL_GetControlWord(&word);

  EXPECT_EQ(status, kHalSuccess);
  expect_control_word_bits(word, 0u);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C23-3. Each named control bit maps independently and clears reserved bits.
TEST(HalGetControlWord, WithCachedDsStateCopiesEachNamedBitAndClearsReservedBits) {
  struct case_ {
    std::uint32_t control_bits;
    std::uint32_t expected_word_bits;
  };
  constexpr std::array<case_, 6> cases{{
      {kControlEnabled, kControlEnabled},
      {kControlAutonomous, kControlAutonomous},
      {kControlTest, kControlTest},
      {kControlEStop, kControlEStop},
      {kControlFmsAttached, kControlFmsAttached},
      {kControlDsAttached, kControlDsAttached},
  }};

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                      /*joystick0_axis_0_value=*/0.5f,
                                      /*control_bits=*/cases[i].control_bits);
    inject_ds_state(core, shim, state, 50'000 + i);

    HAL_ControlWord word = control_word_with_raw_bits(0xFFFF'FFFFu);
    const std::int32_t status = HAL_GetControlWord(&word);

    EXPECT_EQ(status, kHalSuccess);
    expect_control_word_bits(word, cases[i].expected_word_bits);
  }
}

// C23-4. No installed shim reports handle error and unknown alliance.
TEST(HalGetAllianceStation, WithNoShimInstalledSetsHandleErrorAndReturnsUnknown) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  const HAL_AllianceStationID station = HAL_GetAllianceStation(&status);

  EXPECT_EQ(status, kHalHandleError);
  EXPECT_EQ(station, HAL_AllianceStationID_kUnknown);
}

// C23-5. Empty ds_state cache succeeds, returns unknown, and stays empty.
TEST(HalGetAllianceStation, WithShimInstalledButCacheEmptySetsSuccessAndReturnsUnknown) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_AllianceStationID station = HAL_GetAllianceStation(&status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(station, HAL_AllianceStationID_kUnknown);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C23-6. Cached ds_state station is returned through the HAL enum.
TEST(HalGetAllianceStation, WithCachedDsStateReturnsCachedStation) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlEnabled,
                                    /*station=*/alliance_station::blue_3);
  inject_ds_state(core, shim, state);

  std::int32_t status = 999;
  const HAL_AllianceStationID station = HAL_GetAllianceStation(&status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(station, HAL_AllianceStationID_kBlue3);
}

// C23-7. No installed shim reports handle error and zero match time.
TEST(HalGetMatchTime, WithNoShimInstalledSetsHandleErrorAndReturnsZero) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  const double match_time = HAL_GetMatchTime(&status);

  EXPECT_EQ(status, kHalHandleError);
  EXPECT_EQ(match_time, 0.0);
}

// C23-8. Empty ds_state cache succeeds, returns zero, and stays empty.
TEST(HalGetMatchTime, WithShimInstalledButCacheEmptySetsSuccessAndReturnsZero) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const double match_time = HAL_GetMatchTime(&status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(match_time, 0.0);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C23-9. Cached ds_state match time is returned without integer truncation.
TEST(HalGetMatchTime, WithCachedDsStateReturnsCachedMatchTime) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlEnabled,
                                    /*station=*/alliance_station::red_2,
                                    /*type=*/match_type::qualification,
                                    /*match_number=*/42,
                                    /*match_time_seconds=*/135.25);
  inject_ds_state(core, shim, state);

  std::int32_t status = 999;
  const double match_time = HAL_GetMatchTime(&status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(match_time, 135.25);
}

// C23-10. Latest-wins is observed across all three DS scalar readers.
TEST(HalDriverStationScalarReads, LatestWinsAcrossTwoDsUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlEnabled,
                                    /*station=*/alliance_station::red_1,
                                    /*type=*/match_type::qualification,
                                    /*match_number=*/1,
                                    /*match_time_seconds=*/15.0);
  inject_ds_state(core, shim, first, 50'000);

  HAL_ControlWord first_word = control_word_with_raw_bits(0xFFFF'FFFFu);
  EXPECT_EQ(HAL_GetControlWord(&first_word), kHalSuccess);
  expect_control_word_bits(first_word, kControlEnabled);
  std::int32_t first_station_status = 999;
  EXPECT_EQ(HAL_GetAllianceStation(&first_station_status), HAL_AllianceStationID_kRed1);
  EXPECT_EQ(first_station_status, kHalSuccess);
  std::int32_t first_match_time_status = 888;
  EXPECT_EQ(HAL_GetMatchTime(&first_match_time_status), 15.0);
  EXPECT_EQ(first_match_time_status, kHalSuccess);

  const auto second =
      valid_ds_state(/*joystick0_axis_count=*/4,
                     /*joystick0_axis_0_value=*/0.25f,
                     /*control_bits=*/kControlAutonomous | kControlTest | kControlDsAttached,
                     /*station=*/alliance_station::blue_2,
                     /*type=*/match_type::elimination,
                     /*match_number=*/2,
                     /*match_time_seconds=*/42.75);
  inject_ds_state(core, shim, second, 100'000);

  HAL_ControlWord second_word = control_word_with_raw_bits(0xFFFF'FFFFu);
  EXPECT_EQ(HAL_GetControlWord(&second_word), kHalSuccess);
  expect_control_word_bits(second_word, kControlAutonomous | kControlTest | kControlDsAttached);
  std::int32_t second_station_status = 777;
  EXPECT_EQ(HAL_GetAllianceStation(&second_station_status), HAL_AllianceStationID_kBlue2);
  EXPECT_EQ(second_station_status, kHalSuccess);
  std::int32_t second_match_time_status = 666;
  EXPECT_EQ(HAL_GetMatchTime(&second_match_time_status), 42.75);
  EXPECT_EQ(second_match_time_status, kHalSuccess);
}

// ============================================================================
// Cycle 24 — Driver Station joystick reads.
// ============================================================================

namespace {

template <typename T>
void fill_with_sentinel(T& value) {
  std::memset(&value, 0xA5, sizeof(value));
}

template <typename T>
void expect_zero_bytes(const T& value) {
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    EXPECT_EQ(bytes[i], 0u) << "byte offset " << i;
  }
}

template <typename HalT, typename BackendT>
void expect_hal_bytes_eq_backend(const HalT& actual, const BackendT& expected) {
  static_assert(sizeof(HalT) == sizeof(BackendT));
  EXPECT_EQ(std::memcmp(&actual, &expected, sizeof(HalT)), 0);
}

joystick_axes make_axes(std::int16_t count, float base, std::uint8_t raw_base) {
  joystick_axes axes{};
  axes.count = count;
  for (std::size_t i = 0; i < axes.axes.size(); ++i) {
    axes.axes[i] = base + static_cast<float>(i) * 0.25f;
    axes.raw[i] = static_cast<std::uint8_t>(raw_base + i);
  }
  return axes;
}

joystick_povs make_povs(std::int16_t count, std::int16_t base) {
  joystick_povs povs{};
  povs.count = count;
  for (std::size_t i = 0; i < povs.povs.size(); ++i) {
    povs.povs[i] = static_cast<std::int16_t>(base + static_cast<std::int16_t>(i) * 45);
  }
  return povs;
}

joystick_buttons make_buttons(std::uint32_t buttons, std::uint8_t count) {
  joystick_buttons value{};
  value.buttons = buttons;
  value.count = count;
  return value;
}

}  // namespace

// C24-0. Exported C joystick structs match the backend WPILib ABI mirrors.
TEST(HalJoystickStructLayout, MatchesBackendJoystickAbiMirrors) {
  EXPECT_EQ(sizeof(HAL_JoystickAxes), sizeof(joystick_axes));
  EXPECT_EQ(alignof(HAL_JoystickAxes), alignof(joystick_axes));
  EXPECT_EQ(offsetof(HAL_JoystickAxes, count), offsetof(joystick_axes, count));
  EXPECT_EQ(offsetof(HAL_JoystickAxes, axes), offsetof(joystick_axes, axes));
  EXPECT_EQ(offsetof(HAL_JoystickAxes, raw), offsetof(joystick_axes, raw));

  EXPECT_EQ(sizeof(HAL_JoystickPOVs), sizeof(joystick_povs));
  EXPECT_EQ(alignof(HAL_JoystickPOVs), alignof(joystick_povs));
  EXPECT_EQ(offsetof(HAL_JoystickPOVs, count), offsetof(joystick_povs, count));
  EXPECT_EQ(offsetof(HAL_JoystickPOVs, povs), offsetof(joystick_povs, povs));

  EXPECT_EQ(sizeof(HAL_JoystickButtons), sizeof(joystick_buttons));
  EXPECT_EQ(alignof(HAL_JoystickButtons), alignof(joystick_buttons));
  EXPECT_EQ(offsetof(HAL_JoystickButtons, buttons), offsetof(joystick_buttons, buttons));
  EXPECT_EQ(offsetof(HAL_JoystickButtons, count), offsetof(joystick_buttons, count));
}

// C24-1. No installed shim returns handle error and zeroes axes output bytes.
TEST(HalGetJoystickAxes, WithNoShimInstalledReturnsHandleErrorAndZerosOutput) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_JoystickAxes axes{};
  fill_with_sentinel(axes);
  const std::int32_t status = HAL_GetJoystickAxes(0, &axes);

  EXPECT_EQ(status, kHalHandleError);
  expect_zero_bytes(axes);
}

// C24-2. Empty ds_state cache succeeds, zeroes axes output, and stays empty.
TEST(HalGetJoystickAxes, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_JoystickAxes axes{};
  fill_with_sentinel(axes);
  const std::int32_t status = HAL_GetJoystickAxes(0, &axes);

  EXPECT_EQ(status, kHalSuccess);
  expect_zero_bytes(axes);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C24-3. Populated axes reads copy valid boundary slots byte-for-byte.
TEST(HalGetJoystickAxes, WithCachedDsStateCopiesBoundaryAxisSlotsByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_axes_[0] = make_axes(3, -0.5f, 10);
  state.joystick_axes_[5] = make_axes(6, 0.25f, 90);
  inject_ds_state(core, shim, state);

  HAL_JoystickAxes axes0{};
  fill_with_sentinel(axes0);
  EXPECT_EQ(HAL_GetJoystickAxes(0, &axes0), kHalSuccess);
  expect_hal_bytes_eq_backend(axes0, state.joystick_axes_[0]);

  HAL_JoystickAxes axes5{};
  fill_with_sentinel(axes5);
  EXPECT_EQ(HAL_GetJoystickAxes(5, &axes5), kHalSuccess);
  expect_hal_bytes_eq_backend(axes5, state.joystick_axes_[5]);
}

// C24-4. Invalid axes indices succeed with zero/default output.
TEST(HalGetJoystickAxes, WithInvalidIndicesReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_axes_[0] = make_axes(4, 0.75f, 40);
  inject_ds_state(core, shim, state);

  HAL_JoystickAxes negative{};
  fill_with_sentinel(negative);
  EXPECT_EQ(HAL_GetJoystickAxes(-1, &negative), kHalSuccess);
  expect_zero_bytes(negative);

  HAL_JoystickAxes too_high{};
  fill_with_sentinel(too_high);
  EXPECT_EQ(HAL_GetJoystickAxes(6, &too_high), kHalSuccess);
  expect_zero_bytes(too_high);
}

// C24-5. No installed shim returns handle error and zeroes POV output bytes.
TEST(HalGetJoystickPOVs, WithNoShimInstalledReturnsHandleErrorAndZerosOutput) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_JoystickPOVs povs{};
  fill_with_sentinel(povs);
  const std::int32_t status = HAL_GetJoystickPOVs(0, &povs);

  EXPECT_EQ(status, kHalHandleError);
  expect_zero_bytes(povs);
}

// C24-6. Empty ds_state cache succeeds, zeroes POV output, and stays empty.
TEST(HalGetJoystickPOVs, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_JoystickPOVs povs{};
  fill_with_sentinel(povs);
  const std::int32_t status = HAL_GetJoystickPOVs(0, &povs);

  EXPECT_EQ(status, kHalSuccess);
  expect_zero_bytes(povs);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C24-7. Populated POV reads copy valid boundary slots byte-for-byte.
TEST(HalGetJoystickPOVs, WithCachedDsStateCopiesBoundaryPovSlotsByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_povs_[0] = make_povs(2, 0);
  state.joystick_povs_[5] = make_povs(4, 90);
  inject_ds_state(core, shim, state);

  HAL_JoystickPOVs povs0{};
  fill_with_sentinel(povs0);
  EXPECT_EQ(HAL_GetJoystickPOVs(0, &povs0), kHalSuccess);
  expect_hal_bytes_eq_backend(povs0, state.joystick_povs_[0]);

  HAL_JoystickPOVs povs5{};
  fill_with_sentinel(povs5);
  EXPECT_EQ(HAL_GetJoystickPOVs(5, &povs5), kHalSuccess);
  expect_hal_bytes_eq_backend(povs5, state.joystick_povs_[5]);
}

// C24-8. Invalid POV indices succeed with zero/default output.
TEST(HalGetJoystickPOVs, WithInvalidIndicesReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_povs_[0] = make_povs(3, 180);
  inject_ds_state(core, shim, state);

  HAL_JoystickPOVs negative{};
  fill_with_sentinel(negative);
  EXPECT_EQ(HAL_GetJoystickPOVs(-1, &negative), kHalSuccess);
  expect_zero_bytes(negative);

  HAL_JoystickPOVs too_high{};
  fill_with_sentinel(too_high);
  EXPECT_EQ(HAL_GetJoystickPOVs(6, &too_high), kHalSuccess);
  expect_zero_bytes(too_high);
}

// C24-9. No installed shim returns handle error and zeroes buttons output bytes.
TEST(HalGetJoystickButtons, WithNoShimInstalledReturnsHandleErrorAndZerosOutput) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_JoystickButtons buttons{};
  fill_with_sentinel(buttons);
  const std::int32_t status = HAL_GetJoystickButtons(0, &buttons);

  EXPECT_EQ(status, kHalHandleError);
  expect_zero_bytes(buttons);
}

// C24-10. Empty ds_state cache succeeds, zeroes buttons output, and stays empty.
TEST(HalGetJoystickButtons, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_JoystickButtons buttons{};
  fill_with_sentinel(buttons);
  const std::int32_t status = HAL_GetJoystickButtons(0, &buttons);

  EXPECT_EQ(status, kHalSuccess);
  expect_zero_bytes(buttons);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C24-11. Populated buttons read copies the requested max slot byte-for-byte.
TEST(HalGetJoystickButtons, WithCachedDsStateCopiesRequestedButtonsSlotByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_buttons_[0] = make_buttons(0x0000'0003u, 2);
  state.joystick_buttons_[5] = make_buttons(0x8000'0005u, 16);
  inject_ds_state(core, shim, state);

  HAL_JoystickButtons buttons{};
  fill_with_sentinel(buttons);
  EXPECT_EQ(HAL_GetJoystickButtons(5, &buttons), kHalSuccess);
  expect_hal_bytes_eq_backend(buttons, state.joystick_buttons_[5]);
}

// C24-12. Invalid button indices succeed with zero/default output.
TEST(HalGetJoystickButtons, WithInvalidIndicesReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_buttons_[0] = make_buttons(0x0000'00FFu, 8);
  inject_ds_state(core, shim, state);

  HAL_JoystickButtons negative{};
  fill_with_sentinel(negative);
  EXPECT_EQ(HAL_GetJoystickButtons(-1, &negative), kHalSuccess);
  expect_zero_bytes(negative);

  HAL_JoystickButtons too_high{};
  fill_with_sentinel(too_high);
  EXPECT_EQ(HAL_GetJoystickButtons(6, &too_high), kHalSuccess);
  expect_zero_bytes(too_high);
}

// C24-13. Latest-wins is observed across all three joystick readers.
TEST(HalJoystickReads, LatestWinsAcrossTwoDsUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto first = valid_ds_state();
  first.joystick_axes_[0] = make_axes(2, -1.0f, 1);
  first.joystick_povs_[1] = make_povs(1, 45);
  first.joystick_buttons_[2] = make_buttons(0x0000'0007u, 3);
  inject_ds_state(core, shim, first, 50'000);

  HAL_JoystickAxes first_axes{};
  fill_with_sentinel(first_axes);
  EXPECT_EQ(HAL_GetJoystickAxes(0, &first_axes), kHalSuccess);
  expect_hal_bytes_eq_backend(first_axes, first.joystick_axes_[0]);
  HAL_JoystickPOVs first_povs{};
  fill_with_sentinel(first_povs);
  EXPECT_EQ(HAL_GetJoystickPOVs(1, &first_povs), kHalSuccess);
  expect_hal_bytes_eq_backend(first_povs, first.joystick_povs_[1]);
  HAL_JoystickButtons first_buttons{};
  fill_with_sentinel(first_buttons);
  EXPECT_EQ(HAL_GetJoystickButtons(2, &first_buttons), kHalSuccess);
  expect_hal_bytes_eq_backend(first_buttons, first.joystick_buttons_[2]);

  auto second = valid_ds_state();
  second.joystick_axes_[0] = make_axes(5, 0.5f, 51);
  second.joystick_povs_[1] = make_povs(3, 135);
  second.joystick_buttons_[2] = make_buttons(0x0000'0015u, 5);
  inject_ds_state(core, shim, second, 100'000);

  HAL_JoystickAxes second_axes{};
  fill_with_sentinel(second_axes);
  EXPECT_EQ(HAL_GetJoystickAxes(0, &second_axes), kHalSuccess);
  expect_hal_bytes_eq_backend(second_axes, second.joystick_axes_[0]);
  HAL_JoystickPOVs second_povs{};
  fill_with_sentinel(second_povs);
  EXPECT_EQ(HAL_GetJoystickPOVs(1, &second_povs), kHalSuccess);
  expect_hal_bytes_eq_backend(second_povs, second.joystick_povs_[1]);
  HAL_JoystickButtons second_buttons{};
  fill_with_sentinel(second_buttons);
  EXPECT_EQ(HAL_GetJoystickButtons(2, &second_buttons), kHalSuccess);
  expect_hal_bytes_eq_backend(second_buttons, second.joystick_buttons_[2]);
}

// ============================================================================
// Cycle 25 — HAL_Notifier* control plane + notifier_state flush.
// ============================================================================

void expect_name_bytes_eq(const notifier_slot& slot, std::string_view expected) {
  ASSERT_LT(expected.size(), slot.name.size());
  EXPECT_EQ(std::memcmp(slot.name.data(), expected.data(), expected.size()), 0);
  for (std::size_t i = expected.size(); i < slot.name.size(); ++i) {
    EXPECT_EQ(slot.name[i], '\0') << "byte " << i;
  }
}

void expect_all_name_bytes_zero(const notifier_slot& slot) {
  for (std::size_t i = 0; i < slot.name.size(); ++i) {
    EXPECT_EQ(slot.name[i], '\0') << "byte " << i;
  }
}

void expect_notifier_state_bytes_eq(const notifier_state& a, const notifier_state& b) {
  ASSERT_EQ(a.count, b.count);
  const std::size_t active_size =
      offsetof(notifier_state, slots) + static_cast<std::size_t>(a.count) * sizeof(notifier_slot);
  EXPECT_EQ(std::memcmp(&a, &b, active_size), 0);
}

// C25-1. With no shim installed, HAL_InitializeNotifier returns the
// invalid handle 0 and writes kHalHandleError.
TEST(HalInitializeNotifier, WithNoShimInstalledReturnsZeroHandleAndHandleError) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);

  EXPECT_EQ(handle, 0);
  EXPECT_EQ(status, kHalHandleError);
}

// C25-2. Installed shim allocates a nonzero notifier handle and exposes
// the default slot through current_notifier_state().
TEST(HalInitializeNotifier, WithInstalledShimReturnsNonzeroHandleAndSuccess) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);

  EXPECT_NE(handle, 0);
  EXPECT_EQ(status, kHalSuccess);
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, handle);
  EXPECT_EQ(snapshot.slots[0].trigger_time_us, 0u);
  EXPECT_EQ(snapshot.slots[0].alarm_active, 0);
  EXPECT_EQ(snapshot.slots[0].canceled, 0);
  expect_all_name_bytes_zero(snapshot.slots[0]);
}

// C25-3. The 32-slot notifier table accepts exactly kMaxNotifiers
// distinct nonzero handles; the next allocation fails without mutation.
TEST(HalInitializeNotifier, AllocatesDistinctHandlesUntilCapacityThenFails) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::array<HAL_NotifierHandle, kMaxNotifiers> handles{};
  for (std::size_t i = 0; i < kMaxNotifiers; ++i) {
    std::int32_t status = 999;
    handles[i] = HAL_InitializeNotifier(&status);
    EXPECT_EQ(status, kHalSuccess) << "i=" << i;
    EXPECT_NE(handles[i], 0) << "i=" << i;
    for (std::size_t j = 0; j < i; ++j) {
      EXPECT_NE(handles[i], handles[j]) << "i=" << i << " j=" << j;
    }
  }
  const auto full_snapshot = shim.current_notifier_state();
  ASSERT_EQ(full_snapshot.count, static_cast<std::uint32_t>(kMaxNotifiers));

  std::int32_t overflow_status = 999;
  HAL_NotifierHandle overflow = HAL_InitializeNotifier(&overflow_status);

  EXPECT_EQ(overflow, 0);
  EXPECT_EQ(overflow_status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), full_snapshot);
}

// C25-4. Names are zero-filled before copy and overlong names are
// truncated to 63 bytes with a trailing NUL in the fixed schema field.
TEST(HalSetNotifierName, UpdatesNameAndTruncatesOverlongInputWithTrailingNul) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_SetNotifierName(handle, "drive-loop", &status);
  EXPECT_EQ(status, kHalSuccess);
  auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  expect_name_bytes_eq(snapshot.slots[0], "drive-loop");

  const std::string overlong(80, 'N');
  HAL_SetNotifierName(handle, overlong.c_str(), &status);

  EXPECT_EQ(status, kHalSuccess);
  snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(std::memcmp(snapshot.slots[0].name.data(), overlong.data(), 63), 0);
  EXPECT_EQ(snapshot.slots[0].name[63], '\0');
}

// C25-5. A null notifier name is treated as empty and clears previous
// name bytes.
TEST(HalSetNotifierName, NullNameClearsPreviousNameToEmpty) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(handle, "previous", &status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_SetNotifierName(handle, nullptr, &status);

  EXPECT_EQ(status, kHalSuccess);
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  expect_all_name_bytes_zero(snapshot.slots[0]);
}

// C25-6. Invalid handles on set-name report handle error and leave the
// current snapshot byte-identical.
TEST(HalSetNotifierName, InvalidHandleReportsHandleErrorAndDoesNotMutateState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(handle, "kept", &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto before = shim.current_notifier_state();

  HAL_SetNotifierName(0, "bad", &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);

  HAL_SetNotifierName(handle + 999, "bad", &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);
}

// C25-7. Updating a notifier alarm mutates the selected slot, does not
// auto-publish, and explicit flush publishes the active-prefix snapshot.
TEST(HalUpdateNotifierAlarm, ActivatesAlarmAndFlushPublishesActivePrefix) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(first, "first", &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(second, "second", &status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_UpdateNotifierAlarm(second, 1'234'567, &status);

  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 2u);
  EXPECT_EQ(snapshot.slots[0].handle, first);
  EXPECT_EQ(snapshot.slots[0].alarm_active, 0);
  EXPECT_EQ(snapshot.slots[1].handle, second);
  EXPECT_EQ(snapshot.slots[1].trigger_time_us, 1'234'567u);
  EXPECT_EQ(snapshot.slots[1].alarm_active, 1);
  EXPECT_EQ(snapshot.slots[1].canceled, 0);

  ASSERT_TRUE(shim.flush_notifier_state(50'000).has_value());
  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::notifier_state);
  EXPECT_EQ(msg.envelope.sequence, 1u);
  const std::size_t expected_size = offsetof(notifier_state, slots) + 2 * sizeof(notifier_slot);
  EXPECT_EQ(msg.envelope.payload_bytes, expected_size);
  ASSERT_EQ(msg.payload.size(), expected_size);
  EXPECT_EQ(std::memcmp(msg.payload.data(), &snapshot, expected_size), 0);
}

// C25-8. Invalid handles on update report handle error and preserve the
// current snapshot.
TEST(HalUpdateNotifierAlarm, InvalidHandleReportsHandleErrorAndDoesNotMutateState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 111, &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto before = shim.current_notifier_state();

  HAL_UpdateNotifierAlarm(0, 999, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);

  HAL_UpdateNotifierAlarm(handle + 999, 999, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);
}

// C25-9. Cancel leaves the handle live but clears the trigger and marks
// the slot canceled for Sim Core.
TEST(HalCancelNotifierAlarm, ClearsTriggerAndMarksCanceled) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 222, &status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_CancelNotifierAlarm(handle, &status);

  EXPECT_EQ(status, kHalSuccess);
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, handle);
  EXPECT_EQ(snapshot.slots[0].trigger_time_us, 0u);
  EXPECT_EQ(snapshot.slots[0].alarm_active, 0);
  EXPECT_EQ(snapshot.slots[0].canceled, 1);
}

// C25-10. Invalid handles on cancel report handle error and preserve
// the current snapshot.
TEST(HalCancelNotifierAlarm, InvalidHandleReportsHandleErrorAndDoesNotMutateState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 333, &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto before = shim.current_notifier_state();

  HAL_CancelNotifierAlarm(0, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);

  HAL_CancelNotifierAlarm(handle + 999, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);
}

// C25-11. Stop has the control-plane shape of cancel: the handle
// remains live, trigger is cleared, and canceled is set.
TEST(HalStopNotifier, ClearsTriggerAndMarksCanceled) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 444, &status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_StopNotifier(handle, &status);

  EXPECT_EQ(status, kHalSuccess);
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, handle);
  EXPECT_EQ(snapshot.slots[0].trigger_time_us, 0u);
  EXPECT_EQ(snapshot.slots[0].alarm_active, 0);
  EXPECT_EQ(snapshot.slots[0].canceled, 1);
}

// C25-12. Invalid handles on stop report handle error and preserve the
// current snapshot.
TEST(HalStopNotifier, InvalidHandleReportsHandleErrorAndDoesNotMutateState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 555, &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto before = shim.current_notifier_state();

  HAL_StopNotifier(0, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);

  HAL_StopNotifier(handle + 999, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);
}

// C25-13. The void clean call is a no-op with no shim installed.
TEST(HalCleanNotifier, WithNoShimInstalledIsNoOp) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_CleanNotifier(1234);

  EXPECT_EQ(shim_core::current(), nullptr);
}

// C25-14. Clean removes a slot, stale handles cannot mutate state, and
// later allocations use a distinct non-reused handle.
TEST(HalCleanNotifier, RemovesSlotAndStaleHandleCannotMutateNewState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(first, "first", &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(second, "second", &status);
  ASSERT_EQ(status, kHalSuccess);

  HAL_CleanNotifier(first);

  auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, second);
  expect_name_bytes_eq(snapshot.slots[0], "second");
  const auto after_clean = snapshot;

  HAL_UpdateNotifierAlarm(first, 999, &status);
  EXPECT_EQ(status, kHalHandleError);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), after_clean);

  const HAL_NotifierHandle third = HAL_InitializeNotifier(&status);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_NE(third, 0);
  EXPECT_NE(third, first);
  snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 2u);
  EXPECT_EQ(snapshot.slots[0].handle, second);
  EXPECT_EQ(snapshot.slots[1].handle, third);
}

// C25-15. An empty notifier table flush publishes a header-only
// notifier_state snapshot rather than no-oping.
TEST(ShimCoreFlushNotifierState, EmptyTablePublishesHeaderOnlySnapshot) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);

  ASSERT_TRUE(shim.flush_notifier_state(75'000).has_value());

  const auto msg = receive_from_shim(core);
  EXPECT_EQ(msg.envelope.kind, envelope_kind::tick_boundary);
  EXPECT_EQ(msg.envelope.payload_schema, schema_id::notifier_state);
  EXPECT_EQ(msg.envelope.sim_time_us, 75'000u);
  constexpr std::size_t kHeaderOnlyBytes = offsetof(notifier_state, slots);
  static_assert(kHeaderOnlyBytes == 8);
  EXPECT_EQ(msg.envelope.payload_bytes, kHeaderOnlyBytes);
  ASSERT_EQ(msg.payload.size(), kHeaderOnlyBytes);
  const notifier_state empty{};
  EXPECT_EQ(std::memcmp(msg.payload.data(), &empty, kHeaderOnlyBytes), 0);
}

// C25-16. A failed notifier-state flush retains the authoritative
// snapshot so retry publishes identical active-prefix bytes.
TEST(ShimCoreFlushNotifierState, TransportFailureRetainsNotifierStateForRetry) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  shim_global_install_guard guard{*shim_or};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(handle, "retry", &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(handle, 777, &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto snapshot = shim_or->current_notifier_state();

  auto first_attempt = shim_or->flush_notifier_state(90'000);
  ASSERT_FALSE(first_attempt.has_value());
  EXPECT_EQ(first_attempt.error().kind, shim_error_kind::send_failed);
  ASSERT_TRUE(first_attempt.error().transport_error.has_value());
  EXPECT_EQ(first_attempt.error().transport_error->kind, tier1_transport_error_kind::lane_busy);
  expect_notifier_state_bytes_eq(shim_or->current_notifier_state(), snapshot);

  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  drain_boot_only(core);
  ASSERT_TRUE(shim_or->flush_notifier_state(90'000).has_value());
  const auto retry_msg = receive_from_shim(core);
  EXPECT_EQ(retry_msg.envelope.sequence, 1u);
  const auto expected_size = active_prefix_bytes(snapshot).size();
  ASSERT_EQ(retry_msg.payload.size(), expected_size);
  EXPECT_EQ(std::memcmp(retry_msg.payload.data(), &snapshot, expected_size), 0);
}

// C25-17. After shutdown, notifier-state flush is rejected before
// touching the outbound lane and the snapshot remains inspectable.
TEST(ShimCoreFlushNotifierState, PostShutdownIsRejectedWithoutTouchingLane) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  ASSERT_NE(handle, 0);
  const auto before_shutdown = shim.current_notifier_state();

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 5'000));
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.is_shutting_down());
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  auto sent = shim.flush_notifier_state(80'000);

  ASSERT_FALSE(sent.has_value());
  EXPECT_EQ(sent.error().kind, shim_error_kind::shutdown_already_observed);
  EXPECT_FALSE(sent.error().transport_error.has_value());
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before_shutdown);
}

// C25-18. Two independent notifier control-plane runs produce
// byte-identical outbound notifier_state snapshots.
TEST(ShimCoreDeterminism, RepeatedRunsProduceByteIdenticalNotifierStateSnapshots) {
  const auto run_setup = [](tier1_shared_region& region,
                            tier1_endpoint& core,
                            tier1::tier1_message& out_first,
                            tier1::tier1_message& out_second) {
    core = make_core(region);
    auto endpoint = make_backend(region);
    auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), 0);
    ASSERT_TRUE(shim_or.has_value());
    drain_boot_only(core);
    ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
    ASSERT_TRUE(shim_or->poll().has_value());
    shim_global_install_guard guard{*shim_or};

    std::int32_t status = 999;
    const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
    ASSERT_EQ(status, kHalSuccess);
    const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
    ASSERT_EQ(status, kHalSuccess);
    HAL_SetNotifierName(first, "alpha", &status);
    ASSERT_EQ(status, kHalSuccess);
    HAL_SetNotifierName(second, "beta", &status);
    ASSERT_EQ(status, kHalSuccess);
    HAL_UpdateNotifierAlarm(first, 1'000, &status);
    ASSERT_EQ(status, kHalSuccess);
    HAL_CancelNotifierAlarm(second, &status);
    ASSERT_EQ(status, kHalSuccess);

    ASSERT_TRUE(shim_or->flush_notifier_state(250'000).has_value());
    auto first_msg = core.try_receive();
    ASSERT_TRUE(first_msg.has_value());
    out_first = std::move(*first_msg);

    HAL_CleanNotifier(first);
    ASSERT_TRUE(shim_or->flush_notifier_state(500'000).has_value());
    auto second_msg = core.try_receive();
    ASSERT_TRUE(second_msg.has_value());
    out_second = std::move(*second_msg);
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
  EXPECT_EQ(std::memcmp(first_a.payload.data(), first_b.payload.data(), first_a.payload.size()), 0);

  EXPECT_EQ(second_a.envelope, second_b.envelope);
  EXPECT_EQ(second_a.payload, second_b.payload);
  ASSERT_EQ(second_a.payload.size(), second_b.payload.size());
  EXPECT_EQ(std::memcmp(second_a.payload.data(), second_b.payload.data(), second_a.payload.size()),
            0);
  ASSERT_EQ(second_a.payload.size(), offsetof(notifier_state, slots) + sizeof(notifier_slot));
}

// ============================================================================
// Cycle 26 — HAL_SetNotifierThreadPriority v0 no-op.
// ============================================================================

// C26-1. With no shim installed, the Notifier priority surface reports
// handle error and returns false.
TEST(HalSetNotifierThreadPriority, WithNoShimInstalledReturnsFalseAndHandleError) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  HAL_Bool ok = HAL_SetNotifierThreadPriority(1, 40, &status);

  EXPECT_EQ(ok, 0);
  EXPECT_EQ(status, kHalHandleError);
}

// C26-2. With a shim installed, the v0 priority call succeeds as a
// deterministic no-op and does not publish notifier_state.
TEST(HalSetNotifierThreadPriority, WithInstalledShimReturnsTrueAndSuccess) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  ASSERT_EQ(shim.current_notifier_state().count, 0u);
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  std::int32_t status = 999;
  HAL_Bool ok = HAL_SetNotifierThreadPriority(0, 0, &status);

  EXPECT_EQ(ok, 1);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_EQ(shim.current_notifier_state().count, 0u);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// C26-3. The priority call is not notifier_state: it leaves an
// existing table byte-identical and does not flush.
TEST(HalSetNotifierThreadPriority, DoesNotMutateExistingNotifierState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(first, "priority-a", &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(second, 42'000, &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_CancelNotifierAlarm(first, &status);
  ASSERT_EQ(status, kHalSuccess);
  const auto before = shim.current_notifier_state();
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  HAL_Bool ok = HAL_SetNotifierThreadPriority(1, 40, &status);

  EXPECT_EQ(ok, 1);
  EXPECT_EQ(status, kHalSuccess);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// C26-4. v0 accepts boundary priority and HAL_Bool inputs because the
// function is a no-op until the threading model lands.
TEST(HalSetNotifierThreadPriority, AcceptsBoundaryPriorityInputsInV0) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status_min = 999;
  EXPECT_EQ(HAL_SetNotifierThreadPriority(1, std::numeric_limits<std::int32_t>::min(), &status_min),
            1);
  EXPECT_EQ(status_min, kHalSuccess);

  std::int32_t status_max = 888;
  EXPECT_EQ(HAL_SetNotifierThreadPriority(1, std::numeric_limits<std::int32_t>::max(), &status_max),
            1);
  EXPECT_EQ(status_max, kHalSuccess);

  std::int32_t status_non_bool = 777;
  EXPECT_EQ(HAL_SetNotifierThreadPriority(-1, -123, &status_non_bool), 1);
  EXPECT_EQ(status_non_bool, kHalSuccess);
}

// C26-5. Repeated calls overwrite status each time.
TEST(HalSetNotifierThreadPriority, OverwritesStatusOnRepeatedCalls) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t first_status = 999;
  std::int32_t second_status = 888;
  HAL_Bool first = HAL_SetNotifierThreadPriority(0, 5, &first_status);
  HAL_Bool second = HAL_SetNotifierThreadPriority(1, 40, &second_status);

  EXPECT_EQ(first, 1);
  EXPECT_EQ(second, 1);
  EXPECT_EQ(first_status, kHalSuccess);
  EXPECT_EQ(second_status, kHalSuccess);
}

// ============================================================================
// Cycle 27 — HAL_WaitForNotifierAlarm wake/drain semantics.
// ============================================================================

struct notifier_wait_result {
  std::uint64_t timestamp = 0;
  std::int32_t status = 0;
};

notifier_wait_result wait_for_notifier_alarm_async(HAL_NotifierHandle handle) {
  notifier_wait_result result{};
  result.status = 999;
  result.timestamp = HAL_WaitForNotifierAlarm(handle, &result.status);
  return result;
}

void send_notifier_alarm_batch(tier1_endpoint& core,
                               std::span<const notifier_alarm_event> events,
                               std::uint64_t sim_time_us) {
  const auto batch = valid_notifier_alarm_batch(events);
  ASSERT_TRUE(core.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(batch),
                        sim_time_us));
}

notifier_wait_result get_wait_result(std::future<notifier_wait_result>& future) {
  using namespace std::chrono_literals;
  EXPECT_EQ(future.wait_for(1s), std::future_status::ready);
  return future.get();
}

// C27-1. No installed shim reports handle error and returns zero.
TEST(HalWaitForNotifierAlarm, WithNoShimInstalledReturnsZeroAndHandleError) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::int32_t status = 999;
  const std::uint64_t timestamp = HAL_WaitForNotifierAlarm(1, &status);

  EXPECT_EQ(timestamp, 0u);
  EXPECT_EQ(status, kHalHandleError);
}

// C27-2. Already queued matching alarm events are drained by handle in FIFO
// order, while unrelated handles remain queued.
TEST(HalWaitForNotifierAlarm, ReturnsAlreadyQueuedMatchingAlarmWithoutBlocking) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  const std::array<notifier_alarm_event, 3> events{
      valid_notifier_alarm_event(10'000, second),
      valid_notifier_alarm_event(20'000, first),
      valid_notifier_alarm_event(30'000, first),
  };
  send_notifier_alarm_batch(core, events, 100'000);
  ASSERT_TRUE(shim.poll().has_value());

  status = 111;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(first, &status), 20'000u);
  EXPECT_EQ(status, kHalSuccess);

  status = 222;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(first, &status), 30'000u);
  EXPECT_EQ(status, kHalSuccess);

  status = 333;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(second, &status), 10'000u);
  EXPECT_EQ(status, kHalSuccess);
}

// C27-3. A pending wait completes when a later poll receives a matching alarm,
// not when an unrelated handle's event arrives.
TEST(HalWaitForNotifierAlarm, BlocksUntilMatchingAlarmIsPolled) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  std::promise<void> start_wait;
  auto start_future = start_wait.get_future();
  auto waiter = std::async(std::launch::async, [first, start = std::move(start_future)]() mutable {
    start.wait();
    return wait_for_notifier_alarm_async(first);
  });
  start_wait.set_value();

  const std::array<notifier_alarm_event, 1> unrelated{
      valid_notifier_alarm_event(40'000, second),
  };
  send_notifier_alarm_batch(core, unrelated, 110'000);
  ASSERT_TRUE(shim.poll().has_value());

  const std::array<notifier_alarm_event, 1> matching{
      valid_notifier_alarm_event(50'000, first),
  };
  send_notifier_alarm_batch(core, matching, 120'000);
  ASSERT_TRUE(shim.poll().has_value());

  const auto result = get_wait_result(waiter);
  EXPECT_EQ(result.timestamp, 50'000u);
  EXPECT_EQ(result.status, kHalSuccess);

  status = 777;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(second, &status), 40'000u);
  EXPECT_EQ(status, kHalSuccess);
}

// C27-4. Pending wait events accumulate across polled batches, and an empty
// inbound batch does not clear unread queued alarms.
TEST(HalWaitForNotifierAlarm, AccumulatesAcrossMultiplePolledBatches) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  const std::array<notifier_alarm_event, 2> first_batch{
      valid_notifier_alarm_event(100, handle),
      valid_notifier_alarm_event(200, handle),
  };
  send_notifier_alarm_batch(core, first_batch, 100'000);
  ASSERT_TRUE(shim.poll().has_value());

  status = 111;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 100u);
  EXPECT_EQ(status, kHalSuccess);

  send_notifier_alarm_batch(core, {}, 110'000);
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(shim.latest_notifier_alarm_batch()->count, 0u);

  const std::array<notifier_alarm_event, 2> second_batch{
      valid_notifier_alarm_event(300, handle),
      valid_notifier_alarm_event(400, handle),
  };
  send_notifier_alarm_batch(core, second_batch, 120'000);
  ASSERT_TRUE(shim.poll().has_value());

  status = 222;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 200u);
  EXPECT_EQ(status, kHalSuccess);

  status = 333;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 300u);
  EXPECT_EQ(status, kHalSuccess);

  status = 444;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 400u);
  EXPECT_EQ(status, kHalSuccess);
}

// C27-5. Draining the wait queue does not destructively edit the latest-wins
// notifier_alarm_batch observer cache.
TEST(HalWaitForNotifierAlarm, DrainDoesNotMutateLatestAlarmBatchCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  const std::array<notifier_alarm_event, 2> events{
      valid_notifier_alarm_event(1'000, handle),
      valid_notifier_alarm_event(2'000, handle),
  };
  const auto expected_cache = valid_notifier_alarm_batch(events);
  send_notifier_alarm_batch(core, events, 130'000);
  ASSERT_TRUE(shim.poll().has_value());
  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());

  status = 111;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 1'000u);
  EXPECT_EQ(status, kHalSuccess);

  status = 222;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 2'000u);
  EXPECT_EQ(status, kHalSuccess);

  ASSERT_TRUE(shim.latest_notifier_alarm_batch().has_value());
  EXPECT_EQ(
      std::memcmp(
          &*shim.latest_notifier_alarm_batch(), &expected_cache, sizeof(notifier_alarm_batch)),
      0);
}

// C27-6. Invalid and cleaned handles return handle error before draining; a
// stale queued event for a cleaned handle is not returned.
TEST(HalWaitForNotifierAlarm, InvalidOrCleanedHandleReturnsZeroHandleErrorWithoutDrainingOthers) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle live = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle cleaned = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  const std::array<notifier_alarm_event, 2> events{
      valid_notifier_alarm_event(11'000, cleaned),
      valid_notifier_alarm_event(22'000, live),
  };
  send_notifier_alarm_batch(core, events, 140'000);
  ASSERT_TRUE(shim.poll().has_value());

  HAL_CleanNotifier(cleaned);

  status = 111;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(0, &status), 0u);
  EXPECT_EQ(status, kHalHandleError);

  status = 222;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(cleaned, &status), 0u);
  EXPECT_EQ(status, kHalHandleError);

  status = 333;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(cleaned + 10'000, &status), 0u);
  EXPECT_EQ(status, kHalHandleError);

  status = 444;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(live, &status), 22'000u);
  EXPECT_EQ(status, kHalSuccess);
}

// C27-7. Canceling a notifier alarm does not wake a pending wait; a later
// matching alarm still completes with its timestamp.
TEST(HalWaitForNotifierAlarm, CancelDoesNotWakePendingWait) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  std::promise<void> start_wait;
  auto start_future = start_wait.get_future();
  auto waiter = std::async(std::launch::async, [handle, start = std::move(start_future)]() mutable {
    start.wait();
    return wait_for_notifier_alarm_async(handle);
  });
  start_wait.set_value();

  std::int32_t cancel_status = 888;
  HAL_CancelNotifierAlarm(handle, &cancel_status);
  EXPECT_EQ(cancel_status, kHalSuccess);

  const std::array<notifier_alarm_event, 1> events{
      valid_notifier_alarm_event(70'000, handle),
  };
  send_notifier_alarm_batch(core, events, 150'000);
  ASSERT_TRUE(shim.poll().has_value());

  const auto result = get_wait_result(waiter);
  EXPECT_EQ(result.timestamp, 70'000u);
  EXPECT_EQ(result.status, kHalSuccess);
}

// C27-8. Stop wakes a pending wait with the documented zero/success result.
TEST(HalWaitForNotifierAlarm, StopNotifierWakesPendingWaitWithZeroSuccess) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  std::promise<void> start_wait;
  auto start_future = start_wait.get_future();
  auto waiter = std::async(std::launch::async, [handle, start = std::move(start_future)]() mutable {
    start.wait();
    return wait_for_notifier_alarm_async(handle);
  });
  start_wait.set_value();

  std::int32_t stop_status = 888;
  HAL_StopNotifier(handle, &stop_status);
  EXPECT_EQ(stop_status, kHalSuccess);

  const auto result = get_wait_result(waiter);
  EXPECT_EQ(result.timestamp, 0u);
  EXPECT_EQ(result.status, kHalSuccess);

  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, handle);
  EXPECT_EQ(snapshot.slots[0].alarm_active, 0);
  EXPECT_EQ(snapshot.slots[0].canceled, 1);
}

// C27-9. Clean wakes a pending wait with handle error because the handle is no
// longer active.
TEST(HalWaitForNotifierAlarm, CleanNotifierWakesPendingWaitWithHandleError) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  std::promise<void> start_wait;
  auto start_future = start_wait.get_future();
  auto waiter = std::async(std::launch::async, [handle, start = std::move(start_future)]() mutable {
    start.wait();
    return wait_for_notifier_alarm_async(handle);
  });
  start_wait.set_value();

  HAL_CleanNotifier(handle);

  const auto result = get_wait_result(waiter);
  EXPECT_EQ(result.timestamp, 0u);
  EXPECT_EQ(result.status, kHalHandleError);
  EXPECT_EQ(shim.current_notifier_state().count, 0u);
}

// C27-10. Status is overwritten on mixed success/error returns.
TEST(HalWaitForNotifierAlarm, OverwritesStatusOnRepeatedMixedReturns) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  std::int32_t status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  const std::array<notifier_alarm_event, 1> events{
      valid_notifier_alarm_event(90'000, handle),
  };
  send_notifier_alarm_batch(core, events, 160'000);
  ASSERT_TRUE(shim.poll().has_value());

  status = 111;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle, &status), 90'000u);
  EXPECT_EQ(status, kHalSuccess);

  status = 222;
  EXPECT_EQ(HAL_WaitForNotifierAlarm(handle + 10'000, &status), 0u);
  EXPECT_EQ(status, kHalHandleError);
}

// ============================================================================
// Cycle 28 — HAL_Initialize / HAL_Shutdown lifecycle.
// ============================================================================

void send_clock_state(tier1_endpoint& core, const clock_state& state, std::uint64_t sim_time_us) {
  ASSERT_TRUE(core.send(
      envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(state), sim_time_us));
}

void wait_until_notifier_waiters(shim_core& shim,
                                 std::uint32_t total,
                                 std::span<const HAL_NotifierHandle> handles) {
  constexpr int kMaxAttempts = 100'000;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    bool handles_match = true;
    for (const HAL_NotifierHandle handle : handles) {
      handles_match = handles_match && shim.pending_notifier_wait_count(handle) == 1u;
    }
    if (shim.pending_notifier_wait_count() == total && handles_match) {
      return;
    }
    std::this_thread::yield();
  }
  ADD_FAILURE() << "notifier waiters did not register";
}

// C28-1. HAL_Initialize fails when the host has not installed a shim.
TEST(HalInitialize, WithNoShimInstalledReturnsFalse) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  EXPECT_EQ(HAL_Initialize(500, 0), 0);
  EXPECT_EQ(shim_core::current(), nullptr);
}

// C28-2. HAL_Initialize succeeds against the host-installed shim without
// publishing any extra outbound traffic.
TEST(HalInitialize, WithInstalledShimReturnsTrueAndDoesNotPublish) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  ASSERT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  EXPECT_EQ(HAL_Initialize(500, 0), 1);

  EXPECT_EQ(shim_core::current(), &shim);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// C28-3. Repeated initialize calls are idempotent and preserve shim-owned
// state, including notifier handle allocation.
TEST(HalInitialize, RepeatedCallsAreIdempotentAndPreserveState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto clock = valid_clock_state(42'000);
  send_clock_state(core, clock, 42'000);
  ASSERT_TRUE(shim.poll().has_value());

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_SetNotifierName(first, "lifecycle", &status);
  ASSERT_EQ(status, kHalSuccess);
  HAL_UpdateNotifierAlarm(first, 123'000, &status);
  ASSERT_EQ(status, kHalSuccess);

  ASSERT_TRUE(shim.latest_clock_state().has_value());
  const auto expected_clock = *shim.latest_clock_state();
  const auto before_notifiers = shim.current_notifier_state();

  EXPECT_EQ(HAL_Initialize(0, 0), 1);
  EXPECT_EQ(HAL_Initialize(0, 0), 1);

  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), expected_clock);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before_notifiers);

  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  EXPECT_EQ(status, kHalSuccess);
  EXPECT_GT(second, first);
  const auto after_notifiers = shim.current_notifier_state();
  ASSERT_EQ(after_notifiers.count, 2u);
  EXPECT_EQ(after_notifiers.slots[0].handle, first);
  EXPECT_EQ(after_notifiers.slots[1].handle, second);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// C28-4. timeout and mode inputs are ignored in v0 except for the installed
// shim gate.
TEST(HalInitialize, AcceptsBoundaryTimeoutAndModeInputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  EXPECT_EQ(HAL_Initialize(std::numeric_limits<std::int32_t>::min(),
                           std::numeric_limits<std::int32_t>::min()),
            1);
  EXPECT_EQ(HAL_Initialize(std::numeric_limits<std::int32_t>::max(),
                           std::numeric_limits<std::int32_t>::max()),
            1);

  shim_core::install_global(nullptr);
  EXPECT_EQ(HAL_Initialize(std::numeric_limits<std::int32_t>::min(),
                           std::numeric_limits<std::int32_t>::min()),
            0);
  EXPECT_EQ(HAL_Initialize(std::numeric_limits<std::int32_t>::max(),
                           std::numeric_limits<std::int32_t>::max()),
            0);
}

// C28-5. HAL_Shutdown is a no-op when no shim is installed.
TEST(HalShutdown, WithNoShimInstalledIsNoOp) {
  shim_core::install_global(nullptr);

  HAL_Shutdown();
  HAL_Shutdown();

  EXPECT_EQ(shim_core::current(), nullptr);
}

// C28-6. Shutdown detaches the global C HAL pointer but leaves the caller-owned
// shim object and its state alive.
TEST(HalShutdown, DetachesInstalledShimAndSubsequentHalReadsSeeNoShim) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  ASSERT_EQ(HAL_Initialize(500, 0), 1);

  const auto clock = valid_clock_state(84'000);
  send_clock_state(core, clock, 84'000);
  ASSERT_TRUE(shim.poll().has_value());
  std::int32_t notifier_status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&notifier_status);
  ASSERT_EQ(notifier_status, kHalSuccess);

  HAL_Shutdown();

  EXPECT_EQ(shim_core::current(), nullptr);
  std::int32_t status = 999;
  EXPECT_EQ(HAL_GetFPGATime(&status), 0u);
  EXPECT_EQ(status, kHalHandleError);
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), clock);
  const auto snapshot = shim.current_notifier_state();
  ASSERT_EQ(snapshot.count, 1u);
  EXPECT_EQ(snapshot.slots[0].handle, handle);
}

// C28-7. Shutdown wakes all already-registered notifier waiters with
// handle-error semantics.
TEST(HalShutdown, WakesAllPendingNotifierWaitsWithHandleError) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  ASSERT_EQ(HAL_Initialize(500, 0), 1);

  std::int32_t status = 999;
  const HAL_NotifierHandle first = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);
  const HAL_NotifierHandle second = HAL_InitializeNotifier(&status);
  ASSERT_EQ(status, kHalSuccess);

  auto first_waiter =
      std::async(std::launch::async, [first]() { return wait_for_notifier_alarm_async(first); });
  auto second_waiter =
      std::async(std::launch::async, [second]() { return wait_for_notifier_alarm_async(second); });

  const std::array<HAL_NotifierHandle, 2> handles{first, second};
  wait_until_notifier_waiters(shim, 2, handles);

  HAL_Shutdown();

  const auto first_result = get_wait_result(first_waiter);
  const auto second_result = get_wait_result(second_waiter);
  EXPECT_EQ(first_result.timestamp, 0u);
  EXPECT_EQ(first_result.status, kHalHandleError);
  EXPECT_EQ(second_result.timestamp, 0u);
  EXPECT_EQ(second_result.status, kHalHandleError);
  EXPECT_EQ(shim_core::current(), nullptr);
  EXPECT_EQ(shim.pending_notifier_wait_count(), 0u);
}

// C28-8. A host can reinstall its caller-owned shim after shutdown and
// initialize again.
TEST(HalInitialize, CanSucceedAgainAfterHostReinstallsShim) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  EXPECT_EQ(HAL_Initialize(500, 0), 1);
  HAL_Shutdown();
  ASSERT_EQ(shim_core::current(), nullptr);

  shim_core::install_global(&shim);
  EXPECT_EQ(HAL_Initialize(500, 0), 1);
  EXPECT_EQ(shim_core::current(), &shim);
}

// C28-9. Concurrent initialize calls are reentrant read-only operations when
// the host has already installed the shim.
TEST(HalInitialize, ConcurrentCallsAllReturnTrueWithoutMutatingState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto clock = valid_clock_state(126'000);
  send_clock_state(core, clock, 126'000);
  ASSERT_TRUE(shim.poll().has_value());
  std::int32_t notifier_status = 999;
  const HAL_NotifierHandle handle = HAL_InitializeNotifier(&notifier_status);
  ASSERT_EQ(notifier_status, kHalSuccess);
  const auto before_notifiers = shim.current_notifier_state();

  std::promise<void> start;
  auto shared_start = start.get_future().share();
  std::array<std::future<HAL_Bool>, 8> results;
  for (auto& result : results) {
    result = std::async(std::launch::async, [shared_start]() mutable {
      shared_start.wait();
      return HAL_Initialize(500, 0);
    });
  }

  start.set_value();
  for (auto& result : results) {
    EXPECT_EQ(result.get(), 1);
  }

  EXPECT_EQ(shim_core::current(), &shim);
  ASSERT_TRUE(shim.latest_clock_state().has_value());
  EXPECT_EQ(*shim.latest_clock_state(), clock);
  expect_notifier_state_bytes_eq(shim.current_notifier_state(), before_notifiers);
  EXPECT_EQ(shim.current_notifier_state().slots[0].handle, handle);
  EXPECT_EQ(region.backend_to_core.state.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

// ============================================================================
// Cycle 29 — Driver Station match info + joystick descriptor reads.
// ============================================================================

namespace {

struct owned_wpi_string {
  WPI_String value{};

  ~owned_wpi_string() { reset(); }

  owned_wpi_string() = default;
  owned_wpi_string(const owned_wpi_string&) = delete;
  owned_wpi_string& operator=(const owned_wpi_string&) = delete;

  void reset() {
    std::free(const_cast<char*>(value.str));
    value.str = nullptr;
    value.len = 0;
  }
};

joystick_descriptor make_descriptor(std::uint8_t is_xbox,
                                    std::uint8_t type,
                                    std::string_view name,
                                    std::uint8_t axis_base,
                                    std::uint8_t axis_count = 12,
                                    std::uint8_t button_count = 16,
                                    std::uint8_t pov_count = 4) {
  joystick_descriptor descriptor{};
  descriptor.is_xbox = is_xbox;
  descriptor.type = type;
  const std::size_t name_size = std::min(name.size(), descriptor.name.size());
  std::memcpy(descriptor.name.data(), name.data(), name_size);
  descriptor.axis_count = axis_count;
  for (std::size_t i = 0; i < descriptor.axis_types.size(); ++i) {
    descriptor.axis_types[i] = static_cast<std::uint8_t>(axis_base + i);
  }
  descriptor.button_count = button_count;
  descriptor.pov_count = pov_count;
  return descriptor;
}

match_info make_match_info(std::string_view event_name,
                           match_type type,
                           std::uint16_t match_number,
                           std::uint8_t replay_number,
                           std::string_view game_specific_message) {
  match_info info{};
  const std::size_t event_size = std::min(event_name.size(), info.event_name.size());
  std::memcpy(info.event_name.data(), event_name.data(), event_size);
  info.type = type;
  info.match_number = match_number;
  info.replay_number = replay_number;
  const std::size_t game_size =
      std::min(game_specific_message.size(), info.game_specific_message.size());
  std::memcpy(info.game_specific_message.data(), game_specific_message.data(), game_size);
  info.game_specific_message_size = static_cast<std::uint16_t>(game_size);
  return info;
}

void expect_wpi_string_eq(const WPI_String& actual, std::string_view expected) {
  EXPECT_EQ(actual.len, expected.size());
  if (expected.empty()) {
    EXPECT_EQ(actual.str, nullptr);
    return;
  }
  ASSERT_NE(actual.str, nullptr);
  EXPECT_EQ(std::memcmp(actual.str, expected.data(), expected.size()), 0);
}

template <typename HalArrayT, typename BackendArrayT>
void expect_hal_array_bytes_eq_backend(const HalArrayT& actual, const BackendArrayT& expected) {
  static_assert(sizeof(actual) == sizeof(expected));
  EXPECT_EQ(std::memcmp(actual.data(), expected.data(), sizeof(actual)), 0);
}

}  // namespace

// C29-0. Exported C descriptor and match structs match backend ABI mirrors.
TEST(HalDriverStationMetadataStructLayout, MatchesBackendAbiMirrors) {
  EXPECT_EQ(sizeof(HAL_JoystickDescriptor), sizeof(joystick_descriptor));
  EXPECT_EQ(alignof(HAL_JoystickDescriptor), alignof(joystick_descriptor));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, isXbox), offsetof(joystick_descriptor, is_xbox));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, type), offsetof(joystick_descriptor, type));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, name), offsetof(joystick_descriptor, name));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, axisCount), offsetof(joystick_descriptor, axis_count));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, axisTypes), offsetof(joystick_descriptor, axis_types));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, buttonCount),
            offsetof(joystick_descriptor, button_count));
  EXPECT_EQ(offsetof(HAL_JoystickDescriptor, povCount), offsetof(joystick_descriptor, pov_count));

  EXPECT_EQ(sizeof(HAL_MatchInfo), sizeof(match_info));
  EXPECT_EQ(alignof(HAL_MatchInfo), alignof(match_info));
  EXPECT_EQ(offsetof(HAL_MatchInfo, eventName), offsetof(match_info, event_name));
  EXPECT_EQ(offsetof(HAL_MatchInfo, matchType), offsetof(match_info, type));
  EXPECT_EQ(offsetof(HAL_MatchInfo, matchNumber), offsetof(match_info, match_number));
  EXPECT_EQ(offsetof(HAL_MatchInfo, replayNumber), offsetof(match_info, replay_number));
  EXPECT_EQ(offsetof(HAL_MatchInfo, gameSpecificMessage),
            offsetof(match_info, game_specific_message));
  EXPECT_EQ(offsetof(HAL_MatchInfo, gameSpecificMessageSize),
            offsetof(match_info, game_specific_message_size));

  EXPECT_EQ(sizeof(WPI_String), sizeof(const char*) + sizeof(std::size_t));
  EXPECT_EQ(offsetof(WPI_String, str), 0u);
  EXPECT_EQ(offsetof(WPI_String, len), sizeof(const char*));
}

// C29-1. No installed shim returns handle error and zeroes match info.
TEST(HalGetMatchInfo, WithNoShimInstalledReturnsHandleErrorAndZerosOutput) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_MatchInfo info{};
  fill_with_sentinel(info);
  const std::int32_t status = HAL_GetMatchInfo(&info);

  EXPECT_EQ(status, kHalHandleError);
  expect_zero_bytes(info);
}

// C29-2. Empty ds_state cache succeeds and zeroes match info.
TEST(HalGetMatchInfo, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_MatchInfo info{};
  fill_with_sentinel(info);
  const std::int32_t status = HAL_GetMatchInfo(&info);

  EXPECT_EQ(status, kHalSuccess);
  expect_zero_bytes(info);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C29-3. Cached match info is copied byte-for-byte through the C ABI.
TEST(HalGetMatchInfo, WithCachedDsStateCopiesMatchInfoByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.match = make_match_info("PNW District", match_type::elimination, 73, 2, "ABC");
  inject_ds_state(core, shim, state);

  HAL_MatchInfo info{};
  fill_with_sentinel(info);
  const std::int32_t status = HAL_GetMatchInfo(&info);

  EXPECT_EQ(status, kHalSuccess);
  expect_hal_bytes_eq_backend(info, state.match);
}

// C29-4. No installed shim returns handle error and zeroes descriptor output.
TEST(HalGetJoystickDescriptor, WithNoShimInstalledReturnsHandleErrorAndZerosOutput) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_JoystickDescriptor descriptor{};
  fill_with_sentinel(descriptor);
  const std::int32_t status = HAL_GetJoystickDescriptor(0, &descriptor);

  EXPECT_EQ(status, kHalHandleError);
  expect_zero_bytes(descriptor);
}

// C29-5. Empty ds_state cache succeeds and zeroes descriptor output.
TEST(HalGetJoystickDescriptor, WithShimInstalledButCacheEmptyReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  HAL_JoystickDescriptor descriptor{};
  fill_with_sentinel(descriptor);
  const std::int32_t status = HAL_GetJoystickDescriptor(0, &descriptor);

  EXPECT_EQ(status, kHalSuccess);
  expect_zero_bytes(descriptor);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C29-6. Cached descriptors copy valid boundary slots byte-for-byte.
TEST(HalGetJoystickDescriptor, WithCachedDsStateCopiesBoundarySlotsByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_descriptors[0] = make_descriptor(1, 3, "Driver Pad", 10);
  state.joystick_descriptors[5] = make_descriptor(0, 8, "Operator Panel", 40);
  inject_ds_state(core, shim, state);

  HAL_JoystickDescriptor descriptor0{};
  fill_with_sentinel(descriptor0);
  EXPECT_EQ(HAL_GetJoystickDescriptor(0, &descriptor0), kHalSuccess);
  expect_hal_bytes_eq_backend(descriptor0, state.joystick_descriptors[0]);

  HAL_JoystickDescriptor descriptor5{};
  fill_with_sentinel(descriptor5);
  EXPECT_EQ(HAL_GetJoystickDescriptor(5, &descriptor5), kHalSuccess);
  expect_hal_bytes_eq_backend(descriptor5, state.joystick_descriptors[5]);
}

// C29-7. Invalid descriptor indices succeed with zero/default output.
TEST(HalGetJoystickDescriptor, WithInvalidIndicesReturnsSuccessAndZerosOutput) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_descriptors[0] = make_descriptor(1, 4, "Cached", 20);
  inject_ds_state(core, shim, state);

  HAL_JoystickDescriptor negative{};
  fill_with_sentinel(negative);
  EXPECT_EQ(HAL_GetJoystickDescriptor(-1, &negative), kHalSuccess);
  expect_zero_bytes(negative);

  HAL_JoystickDescriptor too_high{};
  fill_with_sentinel(too_high);
  EXPECT_EQ(HAL_GetJoystickDescriptor(6, &too_high), kHalSuccess);
  expect_zero_bytes(too_high);
}

// C29-8. No installed shim returns zero/default for scalar descriptor helpers.
TEST(HalJoystickDescriptorScalars, WithNoShimInstalledReturnZeroDefaults) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  EXPECT_EQ(HAL_GetJoystickIsXbox(0), 0);
  EXPECT_EQ(HAL_GetJoystickType(0), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(0, 0), 0);
}

// C29-9. Empty ds_state cache returns zero/default for scalar helpers.
TEST(HalJoystickDescriptorScalars, WithShimInstalledButCacheEmptyReturnZeroDefaults) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  EXPECT_EQ(HAL_GetJoystickIsXbox(0), 0);
  EXPECT_EQ(HAL_GetJoystickType(0), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(0, 0), 0);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C29-10. Cached scalar descriptor helpers return cached fields.
TEST(HalJoystickDescriptorScalars, WithCachedDsStateReturnCachedFields) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_descriptors[2] = make_descriptor(1, 7, "Flight", 10);
  inject_ds_state(core, shim, state);

  EXPECT_EQ(HAL_GetJoystickIsXbox(2), 1);
  EXPECT_EQ(HAL_GetJoystickType(2), 7);
  EXPECT_EQ(HAL_GetJoystickAxisType(2, 0), 10);
  EXPECT_EQ(HAL_GetJoystickAxisType(2, 11), 21);
}

// C29-11. Invalid joystick or axis indices return zero/default for scalars.
TEST(HalJoystickDescriptorScalars, WithInvalidIndicesReturnZeroDefaults) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_descriptors[2] = make_descriptor(1, 7, "Flight", 10);
  inject_ds_state(core, shim, state);

  EXPECT_EQ(HAL_GetJoystickIsXbox(-1), 0);
  EXPECT_EQ(HAL_GetJoystickIsXbox(6), 0);
  EXPECT_EQ(HAL_GetJoystickType(-1), 0);
  EXPECT_EQ(HAL_GetJoystickType(6), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(2, -1), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(2, 12), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(-1, 0), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(6, 0), 0);
  EXPECT_EQ(HAL_GetJoystickAxisType(-1, 12), 0);
}

// C29-12. HAL_GetJoystickName returns owned WPI_String data for cached names.
TEST(HalGetJoystickName, WithCachedDsStateReturnsOwnedString) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();
  state.joystick_descriptors[3] = make_descriptor(0, 5, "Flight Stick", 30);
  inject_ds_state(core, shim, state);

  owned_wpi_string name;
  HAL_GetJoystickName(&name.value, 3);

  expect_wpi_string_eq(name.value, "Flight Stick");
}

// C29-13. HAL_GetJoystickName handles unavailable and full-length names.
TEST(HalGetJoystickName, HandlesDefaultAndFullLengthNames) {
  shim_core::install_global(nullptr);
  owned_wpi_string no_shim;
  HAL_GetJoystickName(&no_shim.value, 0);
  expect_wpi_string_eq(no_shim.value, "");

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  owned_wpi_string empty_cache;
  HAL_GetJoystickName(&empty_cache.value, 0);
  expect_wpi_string_eq(empty_cache.value, "");

  auto state = valid_ds_state();
  std::string full_name(kJoystickNameLen, 'Z');
  state.joystick_descriptors[1] = make_descriptor(1, 9, full_name, 60);
  inject_ds_state(core, shim, state);

  owned_wpi_string invalid;
  HAL_GetJoystickName(&invalid.value, 6);
  expect_wpi_string_eq(invalid.value, "");

  owned_wpi_string full;
  HAL_GetJoystickName(&full.value, 1);
  expect_wpi_string_eq(full.value, full_name);
}

// C29-14. Match and descriptor readers observe latest-wins DS cache updates.
TEST(HalDriverStationMetadataReads, LatestWinsAcrossTwoDsUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto first = valid_ds_state();
  first.match = make_match_info("Week 1", match_type::qualification, 11, 0, "RED");
  first.joystick_descriptors[1] = make_descriptor(1, 4, "First Stick", 70);
  inject_ds_state(core, shim, first, 50'000);

  HAL_MatchInfo first_match{};
  EXPECT_EQ(HAL_GetMatchInfo(&first_match), kHalSuccess);
  expect_hal_bytes_eq_backend(first_match, first.match);
  HAL_JoystickDescriptor first_descriptor{};
  EXPECT_EQ(HAL_GetJoystickDescriptor(1, &first_descriptor), kHalSuccess);
  expect_hal_bytes_eq_backend(first_descriptor, first.joystick_descriptors[1]);
  EXPECT_EQ(HAL_GetJoystickIsXbox(1), 1);
  EXPECT_EQ(HAL_GetJoystickType(1), 4);
  EXPECT_EQ(HAL_GetJoystickAxisType(1, 2), 72);
  owned_wpi_string first_name;
  HAL_GetJoystickName(&first_name.value, 1);
  expect_wpi_string_eq(first_name.value, "First Stick");

  auto second = valid_ds_state();
  second.match = make_match_info("Week 2", match_type::elimination, 22, 1, "BLUE");
  second.joystick_descriptors[1] = make_descriptor(0, 9, "Second Stick", 90);
  inject_ds_state(core, shim, second, 100'000);

  HAL_MatchInfo second_match{};
  EXPECT_EQ(HAL_GetMatchInfo(&second_match), kHalSuccess);
  expect_hal_bytes_eq_backend(second_match, second.match);
  HAL_JoystickDescriptor second_descriptor{};
  EXPECT_EQ(HAL_GetJoystickDescriptor(1, &second_descriptor), kHalSuccess);
  expect_hal_bytes_eq_backend(second_descriptor, second.joystick_descriptors[1]);
  EXPECT_EQ(HAL_GetJoystickIsXbox(1), 0);
  EXPECT_EQ(HAL_GetJoystickType(1), 9);
  EXPECT_EQ(HAL_GetJoystickAxisType(1, 2), 92);
  owned_wpi_string second_name;
  HAL_GetJoystickName(&second_name.value, 1);
  expect_wpi_string_eq(second_name.value, "Second Stick");
}

// ============================================================================
// Cycle 30 — HAL_GetAllJoystickData aggregate DS joystick read.
// ============================================================================

namespace {

void fill_all_joystick_outputs_with_sentinel(
    std::array<HAL_JoystickAxes, kMaxJoysticks>& axes,
    std::array<HAL_JoystickPOVs, kMaxJoysticks>& povs,
    std::array<HAL_JoystickButtons, kMaxJoysticks>& buttons) {
  fill_with_sentinel(axes);
  fill_with_sentinel(povs);
  fill_with_sentinel(buttons);
}

void expect_all_joystick_outputs_zero(
    const std::array<HAL_JoystickAxes, kMaxJoysticks>& axes,
    const std::array<HAL_JoystickPOVs, kMaxJoysticks>& povs,
    const std::array<HAL_JoystickButtons, kMaxJoysticks>& buttons) {
  expect_zero_bytes(axes);
  expect_zero_bytes(povs);
  expect_zero_bytes(buttons);
}

ds_state ds_state_with_distinct_joysticks(float axis_base,
                                          std::int16_t pov_base,
                                          std::uint32_t button_base) {
  auto state = valid_ds_state();
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    state.joystick_axes_[i] = make_axes(static_cast<std::int16_t>(i + 1),
                                        axis_base + static_cast<float>(i),
                                        static_cast<std::uint8_t>(10 + i));
    state.joystick_povs_[i] =
        make_povs(static_cast<std::int16_t>(i + 1),
                  static_cast<std::int16_t>(pov_base + static_cast<std::int16_t>(i)));
    state.joystick_buttons_[i] =
        make_buttons(button_base + static_cast<std::uint32_t>(i), static_cast<std::uint8_t>(i + 1));
  }
  return state;
}

}  // namespace

// C30-1. No installed shim zeroes all aggregate joystick outputs.
TEST(HalGetAllJoystickData, WithNoShimInstalledZerosAllOutputs) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  std::array<HAL_JoystickAxes, kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> buttons{};
  fill_all_joystick_outputs_with_sentinel(axes, povs, buttons);

  HAL_GetAllJoystickData(axes.data(), povs.data(), buttons.data());

  expect_all_joystick_outputs_zero(axes, povs, buttons);
}

// C30-2. Empty ds_state cache zeroes all aggregate joystick outputs.
TEST(HalGetAllJoystickData, WithShimInstalledButCacheEmptyZerosAllOutputs) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  ASSERT_FALSE(shim.latest_ds_state().has_value());
  shim_global_install_guard guard{shim};

  std::array<HAL_JoystickAxes, kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> buttons{};
  fill_all_joystick_outputs_with_sentinel(axes, povs, buttons);

  HAL_GetAllJoystickData(axes.data(), povs.data(), buttons.data());

  expect_all_joystick_outputs_zero(axes, povs, buttons);
  EXPECT_FALSE(shim.latest_ds_state().has_value());
}

// C30-3. Cached DS state copies all six joystick arrays byte-for-byte.
TEST(HalGetAllJoystickData, WithCachedDsStateCopiesAllJoystickArraysByteForByte) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = ds_state_with_distinct_joysticks(-0.5f, 30, 0x100u);
  inject_ds_state(core, shim, state);

  std::array<HAL_JoystickAxes, kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> buttons{};
  fill_all_joystick_outputs_with_sentinel(axes, povs, buttons);

  HAL_GetAllJoystickData(axes.data(), povs.data(), buttons.data());

  expect_hal_array_bytes_eq_backend(axes, state.joystick_axes_);
  expect_hal_array_bytes_eq_backend(povs, state.joystick_povs_);
  expect_hal_array_bytes_eq_backend(buttons, state.joystick_buttons_);
}

// C30-4. Aggregate read matches the per-slot joystick readers.
TEST(HalGetAllJoystickData, MatchesPerSlotReadersForSameCache) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = ds_state_with_distinct_joysticks(0.25f, 60, 0x200u);
  inject_ds_state(core, shim, state);

  std::array<HAL_JoystickAxes, kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> buttons{};
  HAL_GetAllJoystickData(axes.data(), povs.data(), buttons.data());

  for (std::int32_t slot : {0, 3, 5}) {
    HAL_JoystickAxes slot_axes{};
    HAL_JoystickPOVs slot_povs{};
    HAL_JoystickButtons slot_buttons{};
    ASSERT_EQ(HAL_GetJoystickAxes(slot, &slot_axes), kHalSuccess);
    ASSERT_EQ(HAL_GetJoystickPOVs(slot, &slot_povs), kHalSuccess);
    ASSERT_EQ(HAL_GetJoystickButtons(slot, &slot_buttons), kHalSuccess);
    expect_hal_bytes_eq_backend(axes[static_cast<std::size_t>(slot)], slot_axes);
    expect_hal_bytes_eq_backend(povs[static_cast<std::size_t>(slot)], slot_povs);
    expect_hal_bytes_eq_backend(buttons[static_cast<std::size_t>(slot)], slot_buttons);
  }
}

// C30-5. Aggregate joystick data observes latest-wins DS cache updates.
TEST(HalGetAllJoystickData, LatestWinsAcrossTwoDsUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = ds_state_with_distinct_joysticks(-1.0f, 90, 0x300u);
  inject_ds_state(core, shim, first, 50'000);

  std::array<HAL_JoystickAxes, kMaxJoysticks> first_axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> first_povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> first_buttons{};
  HAL_GetAllJoystickData(first_axes.data(), first_povs.data(), first_buttons.data());
  expect_hal_array_bytes_eq_backend(first_axes, first.joystick_axes_);
  expect_hal_array_bytes_eq_backend(first_povs, first.joystick_povs_);
  expect_hal_array_bytes_eq_backend(first_buttons, first.joystick_buttons_);

  const auto second = ds_state_with_distinct_joysticks(1.0f, 180, 0x400u);
  inject_ds_state(core, shim, second, 100'000);

  std::array<HAL_JoystickAxes, kMaxJoysticks> second_axes{};
  std::array<HAL_JoystickPOVs, kMaxJoysticks> second_povs{};
  std::array<HAL_JoystickButtons, kMaxJoysticks> second_buttons{};
  HAL_GetAllJoystickData(second_axes.data(), second_povs.data(), second_buttons.data());
  expect_hal_array_bytes_eq_backend(second_axes, second.joystick_axes_);
  expect_hal_array_bytes_eq_backend(second_povs, second.joystick_povs_);
  expect_hal_array_bytes_eq_backend(second_buttons, second.joystick_buttons_);
}

// ============================================================================
// Cycle 32 — HAL_RefreshDSData runtime-loop refresh.
// ============================================================================

namespace {

void send_ds_state_for_refresh(tier1_endpoint& core,
                               const ds_state& state,
                               std::uint64_t sim_time_us = 50'000) {
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::ds_state, bytes_of(state), sim_time_us));
}

void expect_ds_scalar_getters(std::uint32_t expected_control_bits,
                              HAL_AllianceStationID expected_station,
                              double expected_match_time) {
  HAL_ControlWord word = control_word_with_raw_bits(0xFFFF'FFFFu);
  EXPECT_EQ(HAL_GetControlWord(&word), kHalSuccess);
  expect_control_word_bits(word, expected_control_bits);

  std::int32_t station_status = 999;
  EXPECT_EQ(HAL_GetAllianceStation(&station_status), expected_station);
  EXPECT_EQ(station_status, kHalSuccess);

  std::int32_t match_time_status = 888;
  EXPECT_EQ(HAL_GetMatchTime(&match_time_status), expected_match_time);
  EXPECT_EQ(match_time_status, kHalSuccess);
}

void force_post_shutdown_ds_lane(tier1_shared_region& region,
                                 const ds_state& state,
                                 std::uint64_t sim_time_us) {
  // A real core peer is terminal after sending shutdown; plant this packet to
  // prove the shim will not surface DS data after observing that terminal state.
  const auto payload = bytes_of(state);
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::ds_state,
                           static_cast<std::uint32_t>(payload.size()),
                           3,
                           direction::core_to_backend);
  env.sim_time_us = sim_time_us;
  manually_fill_lane(region.core_to_backend, env, payload);
}

}  // namespace

// C32-1. No installed shim returns false.
TEST(HalRefreshDSData, WithNoShimInstalledReturnsFalse) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  EXPECT_EQ(HAL_RefreshDSData(), 0);
}

// C32-2. No inbound message returns false and preserves getter-visible DS data.
TEST(HalRefreshDSData, WithNoInboundMessageReturnsFalseAndPreservesExistingDsGetters) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlEnabled | kControlDsAttached,
                                    /*station=*/alliance_station::red_2,
                                    /*type=*/match_type::qualification,
                                    /*match_number=*/12,
                                    /*match_time_seconds=*/98.25);
  send_ds_state_for_refresh(core, state, 50'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);
  expect_ds_scalar_getters(kControlEnabled | kControlDsAttached,
                           HAL_AllianceStationID_kRed2,
                           98.25);

  EXPECT_EQ(HAL_RefreshDSData(), 0);
  expect_ds_scalar_getters(kControlEnabled | kControlDsAttached,
                           HAL_AllianceStationID_kRed2,
                           98.25);
}

// C32-3. A DS packet returns true and updates the existing DS getter surface.
TEST(HalRefreshDSData, WithDsPacketReturnsTrueAndUpdatesExistingDsGetters) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlAutonomous | kControlFmsAttached,
                                    /*station=*/alliance_station::blue_1,
                                    /*type=*/match_type::practice,
                                    /*match_number=*/34,
                                    /*match_time_seconds=*/12.5);
  send_ds_state_for_refresh(core, state, 60'000);

  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_ds_scalar_getters(kControlAutonomous | kControlFmsAttached,
                           HAL_AllianceStationID_kBlue1,
                           12.5);
}

// C32-4. A non-DS packet dispatches, returns false, and preserves DS getters.
TEST(HalRefreshDSData, WithNonDsPacketDispatchesReturnsFalseAndPreservesDsGetters) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto state = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlTest | kControlDsAttached,
                                    /*station=*/alliance_station::blue_3,
                                    /*type=*/match_type::practice,
                                    /*match_number=*/56,
                                    /*match_time_seconds=*/44.75);
  send_ds_state_for_refresh(core, state, 70'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);

  const auto clock = valid_clock_state(123'000);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock), 123'000));

  EXPECT_EQ(HAL_RefreshDSData(), 0);
  std::int32_t clock_status = 999;
  EXPECT_EQ(HAL_GetFPGATime(&clock_status), 123'000u);
  EXPECT_EQ(clock_status, kHalSuccess);
  expect_ds_scalar_getters(kControlTest | kControlDsAttached,
                           HAL_AllianceStationID_kBlue3,
                           44.75);
}

// C32-5. Repeated DS refreshes replace the latest getter-visible snapshot.
TEST(HalRefreshDSData, RepeatedDsPacketsUseLatestWinsCacheSemantics) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto first = valid_ds_state(/*joystick0_axis_count=*/3,
                                    /*joystick0_axis_0_value=*/0.5f,
                                    /*control_bits=*/kControlEnabled,
                                    /*station=*/alliance_station::red_1,
                                    /*type=*/match_type::qualification,
                                    /*match_number=*/1,
                                    /*match_time_seconds=*/15.0);
  send_ds_state_for_refresh(core, first, 80'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);
  expect_ds_scalar_getters(kControlEnabled, HAL_AllianceStationID_kRed1, 15.0);

  const auto second =
      valid_ds_state(/*joystick0_axis_count=*/4,
                     /*joystick0_axis_0_value=*/0.25f,
                     /*control_bits=*/kControlAutonomous | kControlTest | kControlDsAttached,
                     /*station=*/alliance_station::blue_2,
                     /*type=*/match_type::elimination,
                     /*match_number=*/2,
                     /*match_time_seconds=*/42.75);
  send_ds_state_for_refresh(core, second, 90'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_ds_scalar_getters(kControlAutonomous | kControlTest | kControlDsAttached,
                           HAL_AllianceStationID_kBlue2,
                           42.75);
}

// C32-6. Shutdown returns false and prevents later DS updates from surfacing.
TEST(HalRefreshDSData, ShutdownReturnsFalseAndPreventsLaterDsGetterUpdates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  const auto before_shutdown = valid_ds_state(/*joystick0_axis_count=*/3,
                                              /*joystick0_axis_0_value=*/0.5f,
                                              /*control_bits=*/kControlEnabled,
                                              /*station=*/alliance_station::red_3,
                                              /*type=*/match_type::qualification,
                                              /*match_number=*/3,
                                              /*match_time_seconds=*/111.0);
  send_ds_state_for_refresh(core, before_shutdown, 100'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);
  expect_ds_scalar_getters(kControlEnabled, HAL_AllianceStationID_kRed3, 111.0);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 110'000));
  EXPECT_EQ(HAL_RefreshDSData(), 0);
  EXPECT_TRUE(shim.is_shutting_down());

  const auto after_shutdown = valid_ds_state(/*joystick0_axis_count=*/3,
                                             /*joystick0_axis_0_value=*/0.5f,
                                             /*control_bits=*/kControlTest,
                                             /*station=*/alliance_station::blue_1,
                                             /*type=*/match_type::practice,
                                             /*match_number=*/4,
                                             /*match_time_seconds=*/222.0);
  force_post_shutdown_ds_lane(region, after_shutdown, 120'000);

  EXPECT_EQ(HAL_RefreshDSData(), 0);
  expect_ds_scalar_getters(kControlEnabled, HAL_AllianceStationID_kRed3, 111.0);
}

// ============================================================================
// Cycle 33 — Driver Station new-data event handles.
// ============================================================================

namespace {

class wpi_set_event_log_guard {
 public:
  wpi_set_event_log_guard() { wpi_set_event_call_log().clear(); }
  ~wpi_set_event_log_guard() { wpi_set_event_call_log().clear(); }
  wpi_set_event_log_guard(const wpi_set_event_log_guard&) = delete;
  wpi_set_event_log_guard& operator=(const wpi_set_event_log_guard&) = delete;
};

std::vector<unsigned int> sorted_wpi_set_event_calls() {
  auto calls = wpi_set_event_call_log();
  std::ranges::sort(calls);
  return calls;
}

void expect_wpi_set_event_calls(std::initializer_list<WPI_EventHandle> expected) {
  std::vector<unsigned int> sorted_expected(expected.begin(), expected.end());
  std::ranges::sort(sorted_expected);
  EXPECT_EQ(sorted_wpi_set_event_calls(), sorted_expected);
}

}  // namespace

// C33-1. WPI event handle typedefs match WPILib's unsigned handle ABI.
TEST(HalDsNewDataEvents, WpiEventHandleTypeMatchesUnsignedHandleAbi) {
  static_assert(sizeof(WPI_Handle) == sizeof(unsigned int));
  static_assert(alignof(WPI_Handle) == alignof(unsigned int));
  static_assert(std::is_unsigned_v<WPI_Handle>);
  static_assert(std::is_same_v<WPI_EventHandle, WPI_Handle>);
  static_assert(WPI_EventHandle{0} == WPI_Handle{0});
  SUCCEED();
}

// C33-2. No-shim provide/remove calls do not create process-global leakage.
TEST(HalDsNewDataEvents, NoShimProvideRemoveDoesNotLeakIntoLaterShim) {
  wpi_set_event_log_guard event_log;
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_ProvideNewDataEventHandle(11);
  HAL_RemoveNewDataEventHandle(11);

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  send_ds_state_for_refresh(core, valid_ds_state(), 50'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({});
}

// C33-3. Invalid zero handle is ignored even when a shim is installed.
TEST(HalDsNewDataEvents, InvalidZeroHandleIsIgnoredOnInstalledShim) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(0);
  HAL_RemoveNewDataEventHandle(0);

  send_ds_state_for_refresh(core, valid_ds_state(), 60'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({});
}

// C33-4. Refresh accepting DS data signals every registered event handle.
TEST(HalDsNewDataEvents, RegisteredHandlesAreSignaledWhenRefreshAcceptsDsData) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(101);
  HAL_ProvideNewDataEventHandle(202);

  send_ds_state_for_refresh(core, valid_ds_state(), 70'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({101, 202});
}

// C33-5. Direct poll on the installed shim also signals DS new-data handles.
TEST(HalDsNewDataEvents, DirectPollAcceptingDsDataSignalsRegisteredHandles) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(303);

  send_ds_state_for_refresh(core, valid_ds_state(), 80'000);
  ASSERT_TRUE(shim.poll().has_value());
  expect_wpi_set_event_calls({303});
}

// C33-6. Non-DS refreshes do not signal but keep registrations for later DS.
TEST(HalDsNewDataEvents, NonDsRefreshDoesNotSignalAndPreservesRegistration) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(404);

  const auto clock = valid_clock_state(140'000);
  ASSERT_TRUE(
      core.send(envelope_kind::tick_boundary, schema_id::clock_state, bytes_of(clock), 140'000));
  EXPECT_EQ(HAL_RefreshDSData(), 0);
  expect_wpi_set_event_calls({});

  send_ds_state_for_refresh(core, valid_ds_state(), 90'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({404});
}

// C33-7. Removing one handle prevents its wakeup without affecting others.
TEST(HalDsNewDataEvents, RemovingHandlePreventsLaterWakeupsWithoutAffectingOthers) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(501);
  HAL_ProvideNewDataEventHandle(502);
  HAL_RemoveNewDataEventHandle(501);
  HAL_RemoveNewDataEventHandle(999);

  send_ds_state_for_refresh(core, valid_ds_state(), 100'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({502});
}

// C33-8. Providing the same handle twice coalesces to one wakeup per DS packet.
TEST(HalDsNewDataEvents, DuplicateProvideCoalescesToOneWakeupPerDsPacket) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(606);
  HAL_ProvideNewDataEventHandle(606);

  send_ds_state_for_refresh(core, valid_ds_state(), 110'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({606});
}

// C33-9. Event registrations belong to the shim object, not global storage.
TEST(HalDsNewDataEvents, RegistrationsArePerShimObject) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region_a{};
  tier1_endpoint core_a{tier1_endpoint::make(region_a, direction::core_to_backend).value()};
  auto shim_a = make_connected_shim(region_a, core_a);
  shim_global_install_guard guard{shim_a};
  HAL_ProvideNewDataEventHandle(808);

  tier1_shared_region region_b{};
  tier1_endpoint core_b{tier1_endpoint::make(region_b, direction::core_to_backend).value()};
  auto shim_b = make_connected_shim(region_b, core_b);
  shim_core::install_global(&shim_b);

  send_ds_state_for_refresh(core_b, valid_ds_state(), 120'000);
  EXPECT_EQ(HAL_RefreshDSData(), 1);
  expect_wpi_set_event_calls({});
}

// C33-10. Shutdown prevents post-shutdown packets from signaling handles.
TEST(HalDsNewDataEvents, ShutdownPreventsPostShutdownEventWakeups) {
  wpi_set_event_log_guard event_log;
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ProvideNewDataEventHandle(707);

  ASSERT_TRUE(core.send(envelope_kind::shutdown, schema_id::none, {}, 130'000));
  EXPECT_EQ(HAL_RefreshDSData(), 0);
  EXPECT_TRUE(shim.is_shutting_down());
  expect_wpi_set_event_calls({});

  force_post_shutdown_ds_lane(region, valid_ds_state(), 140'000);
  EXPECT_EQ(HAL_RefreshDSData(), 0);
  expect_wpi_set_event_calls({});
}

// ============================================================================
// Cycle 34 — HAL_GetOutputsEnabled.
// ============================================================================

// C34-1. No installed shim returns false.
TEST(HalGetOutputsEnabled, WithNoShimInstalledReturnsFalse) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);
}

// C34-2. Installed shim with no DS packet returns false.
TEST(HalGetOutputsEnabled, WithShimInstalledButDsCacheEmptyReturnsFalse) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);
}

// C34-3. Outputs are enabled only when enabled and DS attached bits are set.
TEST(HalGetOutputsEnabled, ReturnsTrueOnlyWhenEnabledAndDsAttachedBitsAreSet) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();

  state.control.bits = 0;
  inject_ds_state(core, shim, state, 50'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);

  state.control.bits = kControlEnabled;
  inject_ds_state(core, shim, state, 60'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);

  state.control.bits = kControlDsAttached;
  inject_ds_state(core, shim, state, 70'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);

  state.control.bits = kControlEnabled | kControlDsAttached;
  inject_ds_state(core, shim, state, 80'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 1);
}

// C34-4. E-stop does not override the pinned WPILib boolean expression.
TEST(HalGetOutputsEnabled, EStopDoesNotOverridePinnedBooleanExpression) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto state = valid_ds_state();

  state.control.bits = kControlEStop;
  inject_ds_state(core, shim, state, 50'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);

  state.control.bits = kControlEnabled | kControlDsAttached | kControlEStop;
  inject_ds_state(core, shim, state, 60'000);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 1);
}

// C34-5. Refresh updates drive latest-wins outputs-enabled state.
TEST(HalGetOutputsEnabled, LatestRefreshUpdateControlsOutputsEnabledResult) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  auto first = valid_ds_state();
  first.control.bits = kControlEnabled | kControlDsAttached;
  send_ds_state_for_refresh(core, first, 50'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 1);

  auto second = valid_ds_state();
  second.control.bits = kControlDsAttached;
  send_ds_state_for_refresh(core, second, 60'000);
  ASSERT_EQ(HAL_RefreshDSData(), 1);
  EXPECT_EQ(HAL_GetOutputsEnabled(), 0);
}

// ============================================================================
// Cycle 35 — User-program observer calls.
// ============================================================================

// C35-1. Fresh shims report no user-program observer state.
TEST(HalObserveUserProgram, FreshShimReportsNoObserverState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::none);
}

// C35-2. No-shim observer calls are no-ops and do not leak into later shims.
TEST(HalObserveUserProgram, WithNoShimInstalledCallsAreNoOps) {
  shim_core::install_global(nullptr);
  ASSERT_EQ(shim_core::current(), nullptr);

  HAL_ObserveUserProgramStarting();
  HAL_ObserveUserProgramDisabled();
  HAL_ObserveUserProgramAutonomous();
  HAL_ObserveUserProgramTeleop();
  HAL_ObserveUserProgramTest();

  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::none);
}

// C35-3. Starting records the WPILib program-starting observer state.
TEST(HalObserveUserProgram, StartingRecordsProgramStartingState) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ObserveUserProgramStarting();

  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::starting);
}

// C35-4. Disabled/autonomous/teleop/test observers record distinct states.
TEST(HalObserveUserProgram, ModeObserversRecordDistinctStates) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ObserveUserProgramDisabled();
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::disabled);

  HAL_ObserveUserProgramAutonomous();
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::autonomous);

  HAL_ObserveUserProgramTeleop();
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::teleop);

  HAL_ObserveUserProgramTest();
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::test);
}

// C35-5. The latest observer call replaces the prior observed state.
TEST(HalObserveUserProgram, LatestObserverCallWins) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ObserveUserProgramStarting();
  HAL_ObserveUserProgramDisabled();
  HAL_ObserveUserProgramAutonomous();
  HAL_ObserveUserProgramTeleop();
  HAL_ObserveUserProgramTest();
  HAL_ObserveUserProgramDisabled();

  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::disabled);
}

// C35-6. Observer calls do not publish into existing outbound schemas.
TEST(HalObserveUserProgram, ObserverCallsDoNotPublishOutboundMessages) {
  tier1_shared_region region{};
  auto endpoint = make_backend(region);
  auto shim_or = shim_core::make(std::move(endpoint), valid_boot_descriptor(), kBootSimTime);
  ASSERT_TRUE(shim_or.has_value());
  tier1_endpoint core = make_core(region);
  drain_boot_only(core);
  ASSERT_FALSE(core.try_receive().has_value());

  auto& shim = *shim_or;
  shim_global_install_guard guard{shim};

  HAL_ObserveUserProgramStarting();
  HAL_ObserveUserProgramDisabled();
  HAL_ObserveUserProgramAutonomous();
  HAL_ObserveUserProgramTeleop();
  HAL_ObserveUserProgramTest();

  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::test);
  EXPECT_FALSE(core.try_receive().has_value());
}

// C35-7. Observer state belongs to each shim object.
TEST(HalObserveUserProgram, ObserverStateIsPerShimObject) {
  tier1_shared_region region_a{};
  tier1_endpoint core_a{tier1_endpoint::make(region_a, direction::core_to_backend).value()};
  auto shim_a = make_connected_shim(region_a, core_a);
  shim_global_install_guard guard{shim_a};
  HAL_ObserveUserProgramTeleop();

  tier1_shared_region region_b{};
  tier1_endpoint core_b{tier1_endpoint::make(region_b, direction::core_to_backend).value()};
  auto shim_b = make_connected_shim(region_b, core_b);
  shim_core::install_global(&shim_b);
  HAL_ObserveUserProgramAutonomous();

  EXPECT_EQ(shim_a.user_program_observer_state(), user_program_observer_state::teleop);
  EXPECT_EQ(shim_b.user_program_observer_state(), user_program_observer_state::autonomous);
}

// C35-8. Shutdown detaches the shim, so later observer calls are no-ops.
TEST(HalObserveUserProgram, ShutdownMakesLaterObserverCallsNoOpsForOldShim) {
  tier1_shared_region region{};
  tier1_endpoint core{tier1_endpoint::make(region, direction::core_to_backend).value()};
  auto shim = make_connected_shim(region, core);
  shim_global_install_guard guard{shim};

  HAL_ObserveUserProgramStarting();
  ASSERT_EQ(shim.user_program_observer_state(), user_program_observer_state::starting);

  HAL_Shutdown();
  EXPECT_EQ(shim_core::current(), nullptr);

  HAL_ObserveUserProgramTest();
  EXPECT_EQ(shim.user_program_observer_state(), user_program_observer_state::starting);
}

}  // namespace robosim::backend::shim
