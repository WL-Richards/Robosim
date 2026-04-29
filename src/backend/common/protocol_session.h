#pragma once

#include "sync_envelope.h"
#include "validator_error.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>

namespace robosim::backend {

enum class session_error_kind {
  invalid_local_direction,
  validator_rejected_envelope,
  boot_wrong_direction,
  boot_ack_wrong_direction,
  expected_boot_first,
  expected_boot_ack_first,
  expected_local_boot_first,
  unexpected_boot_ack,
  duplicate_boot,
  duplicate_boot_ack,
  on_demand_reply_without_request,
  pending_on_demand_reply,
  shutdown_already_received,
};

struct session_error {
  session_error_kind kind;
  std::optional<validate_error> validator_error;
  std::string offending_field_name;
  std::string message;

  bool operator==(const session_error&) const = default;
};

class protocol_session {
 public:
  [[nodiscard]] static std::expected<protocol_session, session_error> make(
      direction local_direction);

  [[nodiscard]] direction local_direction() const;
  [[nodiscard]] direction remote_direction() const;
  [[nodiscard]] std::uint64_t next_sequence_to_send() const;
  [[nodiscard]] std::uint64_t next_expected_receive_sequence() const;
  [[nodiscard]] bool has_received_boot() const;
  [[nodiscard]] bool has_sent_boot() const;
  [[nodiscard]] bool has_received_boot_ack() const;
  [[nodiscard]] bool has_sent_boot_ack() const;
  [[nodiscard]] bool has_pending_on_demand_reply() const;
  [[nodiscard]] bool has_received_shutdown() const;

  [[nodiscard]] std::expected<sync_envelope, session_error> build_envelope(
      envelope_kind kind,
      schema_id payload_schema,
      std::uint32_t payload_bytes,
      std::uint64_t sim_time_us);

  [[nodiscard]] std::expected<void, session_error> accept_envelope(
      const sync_envelope& env, std::span<const std::uint8_t> payload = {});

 private:
  explicit protocol_session(direction local_direction);

  [[nodiscard]] std::expected<void, session_error> validate_send_order(envelope_kind kind);

  [[nodiscard]] std::expected<void, session_error> validate_receive_order(envelope_kind kind);

  void apply_send(envelope_kind kind);
  void apply_receive(envelope_kind kind);

  direction local_direction_;
  direction remote_direction_;
  std::uint64_t next_sequence_to_send_ = 0;
  std::uint64_t next_expected_receive_sequence_ = 0;
  bool has_received_boot_ = false;
  bool has_sent_boot_ = false;
  bool has_received_boot_ack_ = false;
  bool has_sent_boot_ack_ = false;
  bool has_pending_on_demand_reply_ = false;
  bool has_received_shutdown_ = false;
};

}  // namespace robosim::backend
