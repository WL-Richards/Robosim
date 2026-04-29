#include "shared_memory_transport.h"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "clock_state.h"
#include "error_message.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_session.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

namespace robosim::backend::tier1 {
namespace {

using helpers::boot_payload;
using helpers::bytes_of;
using helpers::clock_payload;
using helpers::complete_handshake;
using helpers::make_backend;
using helpers::make_core;
using helpers::make_envelope;
using helpers::manually_fill_lane;
using helpers::valid_boot_descriptor;
using helpers::valid_clock_state;

std::vector<std::uint8_t> valid_max_payload() {
  error_message_batch batch{};
  batch.count = kMaxErrorsPerBatch;
  for (std::size_t i = 0; i < batch.messages.size(); ++i) {
    batch.messages[i].severity = 1;
    batch.messages[i].error_code = static_cast<std::int32_t>(100 + i);
    batch.messages[i].print_msg = 1;
    batch.messages[i].details[0] = static_cast<char>('A' + i);
    batch.messages[i].location[0] = static_cast<char>('a' + i);
    batch.messages[i].call_stack[0] = static_cast<char>('0' + i);
  }
  auto payload = bytes_of(batch);
  EXPECT_EQ(payload.size(), kTier1MaxPayloadBytes);
  return payload;
}

std::vector<std::uint8_t> can_batch_payload() {
  can_frame_batch batch{};
  batch.count = 2;
  batch.frames[0].message_id = 0x0204'0001u;
  batch.frames[0].timestamp_us = 100;
  batch.frames[0].data = {1, 2, 3, 4, 5, 6, 7, 8};
  batch.frames[0].data_size = 8;
  batch.frames[1].message_id = 0x0204'0002u;
  batch.frames[1].timestamp_us = 200;
  batch.frames[1].data = {8, 7, 6, 5, 4, 3, 2, 1};
  batch.frames[1].data_size = 8;

  const auto payload_bytes = offsetof(can_frame_batch, frames) + 2 * sizeof(can_frame);
  std::vector<std::uint8_t> payload(payload_bytes);
  std::memcpy(payload.data(), &batch, payload.size());
  return payload;
}

}  // namespace

TEST(Tier1SharedRegion, StartsWithBothLanesEmptyAndFixedInlinePayloadCapacity) {
  tier1_shared_region region{};

  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.backend_to_core.payload_bytes, 0u);
  EXPECT_EQ(region.core_to_backend.payload_bytes, 0u);

  constexpr std::size_t expected_capacity = std::max({
      sizeof(clock_state),
      sizeof(power_state),
      sizeof(ds_state),
      sizeof(can_frame_batch),
      sizeof(can_status),
      sizeof(notifier_state),
      sizeof(notifier_alarm_batch),
      sizeof(error_message_batch),
      sizeof(boot_descriptor),
  });
  EXPECT_EQ(kTier1MaxPayloadBytes, expected_capacity);

  EXPECT_TRUE(std::is_standard_layout_v<tier1_lane>);
  EXPECT_TRUE(std::is_standard_layout_v<tier1_shared_region>);
  EXPECT_FALSE(std::is_polymorphic_v<tier1_lane>);
  EXPECT_FALSE(std::is_polymorphic_v<tier1_shared_region>);
  EXPECT_TRUE((std::is_same_v<decltype(tier1_lane::payload),
                              std::array<std::uint8_t, kTier1MaxPayloadBytes>>));
}

TEST(Tier1Endpoint, RejectsReservedLocalDirection) {
  tier1_shared_region region{};
  auto endpoint = tier1_endpoint::make(region, direction::reserved);
  ASSERT_FALSE(endpoint.has_value());
  EXPECT_EQ(endpoint.error().kind, tier1_transport_error_kind::invalid_endpoint_direction);
  EXPECT_NE(endpoint.error().offending_field_name.find("local_direction"), std::string::npos);
}

TEST(Tier1Endpoint, PublishesBackendBootIntoBackendToCoreLane) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  const auto payload = boot_payload();

  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, payload, 1000));

  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.backend_to_core.envelope.sender, direction::backend_to_core);
  EXPECT_EQ(region.backend_to_core.envelope.sequence, 0u);
  EXPECT_EQ(region.backend_to_core.envelope.kind, envelope_kind::boot);
  EXPECT_EQ(region.backend_to_core.envelope.payload_schema, schema_id::boot_descriptor);
  EXPECT_EQ(region.backend_to_core.envelope.payload_bytes, payload.size());
  EXPECT_EQ(region.backend_to_core.envelope.sim_time_us, 1000u);
  EXPECT_EQ(region.backend_to_core.payload_bytes, payload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), region.backend_to_core.payload.begin()));
}

TEST(Tier1Endpoint, RejectsSendWhenPayloadExceedsFixedCapacityWithoutMutatingSessionOrLane) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  std::vector<std::uint8_t> oversize(kTier1MaxPayloadBytes + 1);

  auto send = backend.send(envelope_kind::boot, schema_id::boot_descriptor, oversize, 0);
  ASSERT_FALSE(send.has_value());
  EXPECT_EQ(send.error().kind, tier1_transport_error_kind::payload_too_large);
  EXPECT_NE(send.error().offending_field_name.find("payload"), std::string::npos);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  EXPECT_EQ(region.backend_to_core.envelope.sequence, 0u);
}

TEST(Tier1Endpoint, SendsPayloadExactlyAtFixedCapacityBoundary) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);
  complete_handshake(backend, core);

  const auto payload = valid_max_payload();
  ASSERT_EQ(payload.size(), kTier1MaxPayloadBytes);
  ASSERT_TRUE(
      backend.send(envelope_kind::tick_boundary, schema_id::error_message_batch, payload, 2000));
  EXPECT_EQ(region.backend_to_core.envelope.payload_bytes, kTier1MaxPayloadBytes);

  auto message = core.try_receive();
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->payload.size(), kTier1MaxPayloadBytes);
  EXPECT_EQ(message->payload, payload);
}

TEST(Tier1Endpoint, RejectsSendWhenOutboundLaneIsFullWithoutConsumingSessionSequence) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);
  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));

  auto busy =
      backend.send(envelope_kind::tick_boundary, schema_id::clock_state, clock_payload(), 0);
  ASSERT_FALSE(busy.has_value());
  EXPECT_EQ(busy.error().kind, tier1_transport_error_kind::lane_busy);
  EXPECT_NE(busy.error().offending_field_name.find("lane"), std::string::npos);
  EXPECT_EQ(region.backend_to_core.envelope.kind, envelope_kind::boot);
  EXPECT_EQ(region.backend_to_core.envelope.sequence, 0u);

  ASSERT_TRUE(core.try_receive());
  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
  ASSERT_TRUE(backend.try_receive());
  ASSERT_TRUE(
      backend.send(envelope_kind::tick_boundary, schema_id::clock_state, clock_payload(), 0));
  EXPECT_EQ(region.backend_to_core.envelope.sequence, 1u);
}

TEST(Tier1Endpoint, RejectsSendWhenOutboundLaneIsWritingOrReadingWithoutMutatingSession) {
  for (tier1_lane_state state : {tier1_lane_state::writing, tier1_lane_state::reading}) {
    tier1_shared_region region{};
    region.backend_to_core.payload_bytes = 17;
    region.backend_to_core.state.store(static_cast<std::uint32_t>(state),
                                       std::memory_order_release);
    auto backend = make_backend(region);

    auto send = backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0);
    ASSERT_FALSE(send.has_value());
    EXPECT_EQ(send.error().kind, tier1_transport_error_kind::lane_in_progress);
    EXPECT_NE(send.error().offending_field_name.find("state"), std::string::npos);
    EXPECT_EQ(region.backend_to_core.state.load(), static_cast<std::uint32_t>(state));
    EXPECT_EQ(region.backend_to_core.payload_bytes, 17u);

    region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::empty),
                                       std::memory_order_release);
    ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
    EXPECT_EQ(region.backend_to_core.envelope.sequence, 0u);
  }
}

TEST(Tier1Endpoint, MapsInvalidOutboundEnvelopeToSessionRejectedEnvelopeAndLeavesLaneEmpty) {
  tier1_shared_region region{};
  auto backend = make_backend(region);

  auto send = backend.send(envelope_kind::tick_boundary, schema_id::none, {}, 0);
  ASSERT_FALSE(send.has_value());
  EXPECT_EQ(send.error().kind, tier1_transport_error_kind::session_rejected_envelope);
  ASSERT_TRUE(send.error().session_failure.has_value());
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  EXPECT_EQ(region.backend_to_core.envelope.sequence, 0u);
}

TEST(Tier1Endpoint, CoreReceivesBackendBootAndClearsLaneAfterSuccessfulSessionAccept) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);
  const auto payload = boot_payload();
  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, payload, 0));

  auto message = core.try_receive();
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->envelope.kind, envelope_kind::boot);
  EXPECT_EQ(message->envelope.sender, direction::backend_to_core);
  EXPECT_EQ(message->envelope.sequence, 0u);
  EXPECT_EQ(message->payload, payload);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(region.backend_to_core.payload_bytes, 0u);

  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
  EXPECT_EQ(region.core_to_backend.envelope.sequence, 0u);
}

TEST(Tier1Endpoint, ReturnsNoMessageWhenInboundLaneIsEmptyWithoutChangingState) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);

  auto empty = core.try_receive();
  ASSERT_FALSE(empty.has_value());
  EXPECT_EQ(empty.error().kind, tier1_transport_error_kind::no_message);
  EXPECT_NE(empty.error().offending_field_name.find("lane"), std::string::npos);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
  EXPECT_EQ(boot->envelope.sequence, 0u);
}

TEST(Tier1Endpoint, RejectsInProgressInboundLaneStatesWithoutClearingTheSlot) {
  for (tier1_lane_state state : {tier1_lane_state::writing, tier1_lane_state::reading}) {
    tier1_shared_region region{};
    region.backend_to_core.payload_bytes = 23;
    region.backend_to_core.state.store(static_cast<std::uint32_t>(state),
                                       std::memory_order_release);
    auto core = make_core(region);

    auto receive = core.try_receive();
    ASSERT_FALSE(receive.has_value());
    EXPECT_EQ(receive.error().kind, tier1_transport_error_kind::lane_in_progress);
    EXPECT_NE(receive.error().offending_field_name.find("state"), std::string::npos);
    EXPECT_EQ(region.backend_to_core.state.load(), static_cast<std::uint32_t>(state));
    EXPECT_EQ(region.backend_to_core.payload_bytes, 23u);
  }
}

TEST(Tier1Endpoint, PreservesLaneWhenProtocolSessionRejectsInboundEnvelope) {
  tier1_shared_region region{};
  auto core = make_core(region);
  const auto payload = clock_payload();
  const auto env = make_envelope(envelope_kind::tick_boundary,
                                 schema_id::clock_state,
                                 static_cast<std::uint32_t>(payload.size()),
                                 0,
                                 direction::backend_to_core);
  manually_fill_lane(region.backend_to_core, env, payload);

  auto receive = core.try_receive();
  ASSERT_FALSE(receive.has_value());
  EXPECT_EQ(receive.error().kind, tier1_transport_error_kind::session_rejected_envelope);
  ASSERT_TRUE(receive.error().session_failure.has_value());
  EXPECT_EQ(receive.error().session_failure->kind, session_error_kind::expected_boot_first);
  EXPECT_NE(receive.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));
  EXPECT_EQ(region.backend_to_core.envelope, env);
  EXPECT_EQ(region.backend_to_core.payload_bytes, payload.size());
}

TEST(Tier1Endpoint, CompletesBootBootAckTickRoundTripAcrossBothLanes) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);

  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
  EXPECT_EQ(boot->envelope.sequence, 0u);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(core.send(envelope_kind::boot_ack, schema_id::none, {}, 0));
  auto ack = backend.try_receive();
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->envelope.sequence, 0u);
  EXPECT_EQ(region.core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(
      backend.send(envelope_kind::tick_boundary, schema_id::clock_state, clock_payload(), 20'000));
  auto tick = core.try_receive();
  ASSERT_TRUE(tick.has_value());
  EXPECT_EQ(tick->envelope.sequence, 1u);
  EXPECT_EQ(tick->payload, clock_payload());
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

TEST(Tier1Endpoint, CopiesVariableSizeBatchPayloadExactlyUsingHeaderDerivedPayloadSize) {
  tier1_shared_region region{};
  auto backend = make_backend(region);
  auto core = make_core(region);
  complete_handshake(backend, core);

  const auto payload = can_batch_payload();
  ASSERT_TRUE(
      backend.send(envelope_kind::tick_boundary, schema_id::can_frame_batch, payload, 20'000));
  auto message = core.try_receive();
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->envelope.payload_bytes, payload.size());
  EXPECT_EQ(message->payload, payload);
}

TEST(Tier1Endpoint, DoesNotClearFullLaneWhenReceivePayloadSizeExceedsCapacityMetadata) {
  tier1_shared_region region{};
  auto core = make_core(region);
  region.backend_to_core.payload_bytes = static_cast<std::uint32_t>(kTier1MaxPayloadBytes + 1);
  region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::full),
                                     std::memory_order_release);

  auto receive = core.try_receive();
  ASSERT_FALSE(receive.has_value());
  EXPECT_EQ(receive.error().kind, tier1_transport_error_kind::payload_too_large);
  EXPECT_NE(receive.error().offending_field_name.find("payload_bytes"), std::string::npos);
  EXPECT_EQ(region.backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::full));

  region.backend_to_core.state.store(static_cast<std::uint32_t>(tier1_lane_state::empty),
                                     std::memory_order_release);
  auto backend = make_backend(region);
  ASSERT_TRUE(backend.send(envelope_kind::boot, schema_id::boot_descriptor, boot_payload(), 0));
  auto boot = core.try_receive();
  ASSERT_TRUE(boot.has_value());
  EXPECT_EQ(boot->envelope.sequence, 0u);
}

}  // namespace robosim::backend::tier1
