#include "protocol_session.h"

#include "boot_descriptor.h"
#include "clock_state.h"
#include "sync_envelope.h"
#include "validator_error.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace robosim::backend {
namespace {

sync_envelope make_envelope(envelope_kind kind,
                            schema_id payload_schema,
                            std::uint32_t payload_bytes,
                            std::uint64_t sequence,
                            direction sender) {
  sync_envelope env{};
  env.magic = kProtocolMagic;
  env.protocol_version = kProtocolVersion;
  env.kind = kind;
  env.sequence = sequence;
  env.sim_time_us = 0;
  env.payload_bytes = payload_bytes;
  env.payload_schema = payload_schema;
  env.sender = sender;
  return env;
}

sync_envelope backend_boot(std::uint64_t sequence) {
  return make_envelope(envelope_kind::boot,
                       schema_id::boot_descriptor,
                       sizeof(boot_descriptor),
                       sequence,
                       direction::backend_to_core);
}

sync_envelope backend_tick(std::uint64_t sequence) {
  return make_envelope(envelope_kind::tick_boundary,
                       schema_id::clock_state,
                       sizeof(clock_state),
                       sequence,
                       direction::backend_to_core);
}

sync_envelope backend_reply(std::uint64_t sequence) {
  return make_envelope(envelope_kind::on_demand_reply,
                       schema_id::clock_state,
                       sizeof(clock_state),
                       sequence,
                       direction::backend_to_core);
}

sync_envelope core_boot_ack(std::uint64_t sequence) {
  return make_envelope(
      envelope_kind::boot_ack, schema_id::none, 0, sequence, direction::core_to_backend);
}

sync_envelope core_tick(std::uint64_t sequence) {
  return make_envelope(envelope_kind::tick_boundary,
                       schema_id::clock_state,
                       sizeof(clock_state),
                       sequence,
                       direction::core_to_backend);
}

std::span<const std::uint8_t> empty_payload() {
  return {};
}

}  // namespace

TEST(ProtocolSession, RejectsReservedLocalDirection) {
  auto session = protocol_session::make(direction::reserved);
  ASSERT_FALSE(session.has_value());
  EXPECT_EQ(session.error().kind, session_error_kind::invalid_local_direction);
  EXPECT_NE(session.error().offending_field_name.find("local_direction"), std::string::npos);
}

TEST(ProtocolSession, ExposesOppositeRemoteDirectionForEachValidLocalDirection) {
  auto backend = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(backend.has_value());
  EXPECT_EQ(backend->local_direction(), direction::backend_to_core);
  EXPECT_EQ(backend->remote_direction(), direction::core_to_backend);
  EXPECT_EQ(backend->next_sequence_to_send(), 0u);
  EXPECT_EQ(backend->next_expected_receive_sequence(), 0u);
  EXPECT_FALSE(backend->has_received_boot());
  EXPECT_FALSE(backend->has_sent_boot());
  EXPECT_FALSE(backend->has_received_boot_ack());
  EXPECT_FALSE(backend->has_sent_boot_ack());
  EXPECT_FALSE(backend->has_pending_on_demand_reply());
  EXPECT_FALSE(backend->has_received_shutdown());

  auto core = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(core.has_value());
  EXPECT_EQ(core->local_direction(), direction::core_to_backend);
  EXPECT_EQ(core->remote_direction(), direction::backend_to_core);
  EXPECT_EQ(core->next_sequence_to_send(), 0u);
  EXPECT_EQ(core->next_expected_receive_sequence(), 0u);
  EXPECT_FALSE(core->has_received_boot());
  EXPECT_FALSE(core->has_sent_boot());
  EXPECT_FALSE(core->has_received_boot_ack());
  EXPECT_FALSE(core->has_sent_boot_ack());
  EXPECT_FALSE(core->has_pending_on_demand_reply());
  EXPECT_FALSE(core->has_received_shutdown());
}

TEST(ProtocolSession, BuildsOutboundEnvelopeWithLocalSenderGaplessSequenceAndZeroReservedBytes) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));

  EXPECT_EQ(session->next_sequence_to_send(), 0u);
  auto boot_ack = session->build_envelope(envelope_kind::boot_ack, schema_id::none, 0, 1000);
  ASSERT_TRUE(boot_ack.has_value());
  EXPECT_EQ(boot_ack->sequence, 0u);
  EXPECT_EQ(session->next_sequence_to_send(), 1u);
  EXPECT_EQ(boot_ack->sender, direction::core_to_backend);
  EXPECT_EQ(boot_ack->magic, kProtocolMagic);
  EXPECT_EQ(boot_ack->protocol_version, kProtocolVersion);
  EXPECT_EQ(boot_ack->reserved[0], 0u);
  EXPECT_EQ(boot_ack->reserved[1], 0u);
  EXPECT_EQ(boot_ack->kind, envelope_kind::boot_ack);
  EXPECT_EQ(boot_ack->payload_schema, schema_id::none);
  EXPECT_EQ(boot_ack->payload_bytes, 0u);
  EXPECT_EQ(boot_ack->sim_time_us, 1000u);

  auto tick = session->build_envelope(
      envelope_kind::tick_boundary, schema_id::clock_state, sizeof(clock_state), 2000);
  ASSERT_TRUE(tick.has_value());
  EXPECT_EQ(tick->sequence, 1u);
  EXPECT_EQ(session->next_sequence_to_send(), 2u);
  EXPECT_EQ(tick->sender, direction::core_to_backend);
  EXPECT_EQ(tick->magic, kProtocolMagic);
  EXPECT_EQ(tick->protocol_version, kProtocolVersion);
  EXPECT_EQ(tick->reserved[0], 0u);
  EXPECT_EQ(tick->reserved[1], 0u);
  EXPECT_EQ(tick->kind, envelope_kind::tick_boundary);
  EXPECT_EQ(tick->payload_schema, schema_id::clock_state);
  EXPECT_EQ(tick->payload_bytes, sizeof(clock_state));
  EXPECT_EQ(tick->sim_time_us, 2000u);
}

TEST(ProtocolSession, DoesNotConsumeSendSequenceWhenOutboundEnvelopeFailsStatelessValidation) {
  auto session = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(session.has_value());

  auto invalid = session->build_envelope(envelope_kind::tick_boundary, schema_id::none, 0, 0);
  ASSERT_FALSE(invalid.has_value());
  EXPECT_EQ(invalid.error().kind, session_error_kind::validator_rejected_envelope);
  ASSERT_TRUE(invalid.error().validator_error.has_value());
  EXPECT_EQ(invalid.error().validator_error->kind,
            validate_error_kind::schema_payload_kind_mismatch);
  EXPECT_NE(invalid.error().offending_field_name.find("payload"), std::string::npos);
  EXPECT_EQ(session->next_sequence_to_send(), 0u);

  auto boot = session->build_envelope(
      envelope_kind::boot, schema_id::boot_descriptor, sizeof(boot_descriptor), 0);
  ASSERT_TRUE(boot.has_value());
  EXPECT_EQ(boot->sequence, 0u);
  EXPECT_EQ(session->next_sequence_to_send(), 1u);
}

TEST(ProtocolSession, LetsBackendSendBootBeforeNormalOutboundTraffic) {
  auto session = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(session.has_value());

  auto tick = session->build_envelope(
      envelope_kind::tick_boundary, schema_id::clock_state, sizeof(clock_state), 0);
  ASSERT_FALSE(tick.has_value());
  EXPECT_EQ(tick.error().kind, session_error_kind::expected_local_boot_first);
  EXPECT_NE(tick.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_sequence_to_send(), 0u);

  auto boot = session->build_envelope(
      envelope_kind::boot, schema_id::boot_descriptor, sizeof(boot_descriptor), 0);
  ASSERT_TRUE(boot.has_value());
  EXPECT_EQ(boot->sequence, 0u);
  EXPECT_TRUE(session->has_sent_boot());
  EXPECT_EQ(session->next_sequence_to_send(), 1u);
}

TEST(ProtocolSession, LetsCoreAcceptBootThenSendBootAckBeforeBackendTickData) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());

  std::array<std::uint8_t, sizeof(boot_descriptor)> boot_payload{};
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), std::span(boot_payload)));
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);
  EXPECT_TRUE(session->has_received_boot());

  auto ack = session->build_envelope(envelope_kind::boot_ack, schema_id::none, 0, 0);
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->sequence, 0u);
  EXPECT_TRUE(session->has_sent_boot_ack());
  EXPECT_EQ(session->next_sequence_to_send(), 1u);

  ASSERT_TRUE(session->accept_envelope(backend_tick(1), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
}

TEST(ProtocolSession, RejectsNonBootInboundEnvelopeBeforeBootWithoutAdvancingReceiveSequence) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());

  auto tick = session->accept_envelope(backend_tick(0), empty_payload());
  ASSERT_FALSE(tick.has_value());
  EXPECT_EQ(tick.error().kind, session_error_kind::expected_boot_first);
  EXPECT_NE(tick.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 0u);

  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);
}

TEST(ProtocolSession, RejectsDuplicateBootAfterBootWithoutAdvancingReceiveSequence) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));

  auto duplicate = session->accept_envelope(backend_boot(1), empty_payload());
  ASSERT_FALSE(duplicate.has_value());
  EXPECT_EQ(duplicate.error().kind, session_error_kind::duplicate_boot);
  EXPECT_NE(duplicate.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);

  ASSERT_TRUE(session->accept_envelope(backend_tick(1), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
}

TEST(ProtocolSession, MapsStatelessValidatorFailuresWithoutAdvancingReceiveSequence) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());

  auto bad_sequence = session->accept_envelope(backend_boot(1), empty_payload());
  ASSERT_FALSE(bad_sequence.has_value());
  EXPECT_EQ(bad_sequence.error().kind, session_error_kind::validator_rejected_envelope);
  ASSERT_TRUE(bad_sequence.error().validator_error.has_value());
  EXPECT_EQ(bad_sequence.error().validator_error->kind, validate_error_kind::sequence_mismatch);
  EXPECT_NE(bad_sequence.error().offending_field_name.find("sequence"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 0u);
}

TEST(ProtocolSession, RequiresBackendToReceiveBootAckBeforeAcceptingNormalCoreTraffic) {
  auto session = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->build_envelope(
      envelope_kind::boot, schema_id::boot_descriptor, sizeof(boot_descriptor), 0));

  auto early_tick = session->accept_envelope(core_tick(0), empty_payload());
  ASSERT_FALSE(early_tick.has_value());
  EXPECT_EQ(early_tick.error().kind, session_error_kind::expected_boot_ack_first);
  EXPECT_NE(early_tick.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 0u);

  ASSERT_TRUE(session->accept_envelope(core_boot_ack(0), empty_payload()));
  EXPECT_TRUE(session->has_received_boot_ack());
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);

  ASSERT_TRUE(session->accept_envelope(core_tick(1), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
}

TEST(ProtocolSession, RejectsBootAckBeforeLocalBootAndDuplicateBootAck) {
  auto session = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(session.has_value());

  auto early_ack = session->accept_envelope(core_boot_ack(0), empty_payload());
  ASSERT_FALSE(early_ack.has_value());
  EXPECT_EQ(early_ack.error().kind, session_error_kind::unexpected_boot_ack);
  EXPECT_NE(early_ack.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 0u);

  ASSERT_TRUE(session->build_envelope(
      envelope_kind::boot, schema_id::boot_descriptor, sizeof(boot_descriptor), 0));
  ASSERT_TRUE(session->accept_envelope(core_boot_ack(0), empty_payload()));
  auto duplicate = session->accept_envelope(core_boot_ack(1), empty_payload());
  ASSERT_FALSE(duplicate.has_value());
  EXPECT_EQ(duplicate.error().kind, session_error_kind::duplicate_boot_ack);
  EXPECT_NE(duplicate.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);

  ASSERT_TRUE(session->accept_envelope(core_tick(1), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
}

TEST(ProtocolSession, AcceptsOnDemandReplyOnlyAfterAnOutboundOnDemandRequest) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));
  ASSERT_TRUE(session->build_envelope(envelope_kind::boot_ack, schema_id::none, 0, 0));
  EXPECT_FALSE(session->has_pending_on_demand_reply());

  auto unsolicited = session->accept_envelope(backend_reply(1), empty_payload());
  ASSERT_FALSE(unsolicited.has_value());
  EXPECT_EQ(unsolicited.error().kind, session_error_kind::on_demand_reply_without_request);
  EXPECT_NE(unsolicited.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);
  EXPECT_FALSE(session->has_pending_on_demand_reply());

  auto request = session->build_envelope(
      envelope_kind::on_demand_request, schema_id::clock_state, sizeof(clock_state), 0);
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->sequence, 1u);
  EXPECT_EQ(session->next_sequence_to_send(), 2u);
  EXPECT_TRUE(session->has_pending_on_demand_reply());

  ASSERT_TRUE(session->accept_envelope(backend_reply(1), empty_payload()));
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
  EXPECT_FALSE(session->has_pending_on_demand_reply());
}

TEST(ProtocolSession,
     RejectsSecondOnDemandRequestWhileOneReplyIsPendingWithoutConsumingSendSequence) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));
  ASSERT_TRUE(session->build_envelope(envelope_kind::boot_ack, schema_id::none, 0, 0));

  auto first = session->build_envelope(
      envelope_kind::on_demand_request, schema_id::clock_state, sizeof(clock_state), 0);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->sequence, 1u);
  EXPECT_EQ(session->next_sequence_to_send(), 2u);
  EXPECT_TRUE(session->has_pending_on_demand_reply());

  auto second = session->build_envelope(
      envelope_kind::on_demand_request, schema_id::clock_state, sizeof(clock_state), 0);
  ASSERT_FALSE(second.has_value());
  EXPECT_EQ(second.error().kind, session_error_kind::pending_on_demand_reply);
  EXPECT_NE(second.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_sequence_to_send(), 2u);
  EXPECT_TRUE(session->has_pending_on_demand_reply());

  ASSERT_TRUE(session->accept_envelope(backend_reply(1), empty_payload()));
  EXPECT_FALSE(session->has_pending_on_demand_reply());
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);

  auto tick = session->build_envelope(
      envelope_kind::tick_boundary, schema_id::clock_state, sizeof(clock_state), 0);
  ASSERT_TRUE(tick.has_value());
  EXPECT_EQ(tick->sequence, 2u);
  EXPECT_EQ(session->next_sequence_to_send(), 3u);
}

TEST(ProtocolSession, RejectsInboundEnvelopesAfterShutdownWithoutAdvancingReceiveSequence) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));

  auto shutdown =
      make_envelope(envelope_kind::shutdown, schema_id::none, 0, 1, direction::backend_to_core);
  ASSERT_TRUE(session->accept_envelope(shutdown, empty_payload()));
  EXPECT_TRUE(session->has_received_shutdown());
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);

  auto tick = session->accept_envelope(backend_tick(2), empty_payload());
  ASSERT_FALSE(tick.has_value());
  EXPECT_EQ(tick.error().kind, session_error_kind::shutdown_already_received);
  EXPECT_NE(tick.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);

  auto duplicate_shutdown_env =
      make_envelope(envelope_kind::shutdown, schema_id::none, 0, 2, direction::backend_to_core);
  auto duplicate_shutdown = session->accept_envelope(duplicate_shutdown_env, empty_payload());
  ASSERT_FALSE(duplicate_shutdown.has_value());
  EXPECT_EQ(duplicate_shutdown.error().kind, session_error_kind::shutdown_already_received);
  EXPECT_NE(duplicate_shutdown.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 2u);
}

TEST(ProtocolSession, RejectsBootInTheCoreToBackendDirection) {
  auto session = protocol_session::make(direction::backend_to_core);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->build_envelope(
      envelope_kind::boot, schema_id::boot_descriptor, sizeof(boot_descriptor), 0));

  auto core_boot = make_envelope(envelope_kind::boot,
                                 schema_id::boot_descriptor,
                                 sizeof(boot_descriptor),
                                 0,
                                 direction::core_to_backend);
  auto wrong_direction = session->accept_envelope(core_boot, empty_payload());
  ASSERT_FALSE(wrong_direction.has_value());
  EXPECT_EQ(wrong_direction.error().kind, session_error_kind::boot_wrong_direction);
  EXPECT_NE(wrong_direction.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 0u);

  ASSERT_TRUE(session->accept_envelope(core_boot_ack(0), empty_payload()));
}

TEST(ProtocolSession, RejectsBootAckInTheBackendToCoreDirection) {
  auto session = protocol_session::make(direction::core_to_backend);
  ASSERT_TRUE(session.has_value());
  ASSERT_TRUE(session->accept_envelope(backend_boot(0), empty_payload()));

  auto backend_ack =
      make_envelope(envelope_kind::boot_ack, schema_id::none, 0, 1, direction::backend_to_core);
  auto wrong_direction = session->accept_envelope(backend_ack, empty_payload());
  ASSERT_FALSE(wrong_direction.has_value());
  EXPECT_EQ(wrong_direction.error().kind, session_error_kind::boot_ack_wrong_direction);
  EXPECT_NE(wrong_direction.error().offending_field_name.find("kind"), std::string::npos);
  EXPECT_EQ(session->next_expected_receive_sequence(), 1u);

  ASSERT_TRUE(session->accept_envelope(backend_tick(1), empty_payload()));
}

}  // namespace robosim::backend
