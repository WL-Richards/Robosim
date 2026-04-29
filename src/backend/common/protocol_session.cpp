#include "protocol_session.h"

#include "validator.h"

#include <utility>

namespace robosim::backend {
namespace {

[[nodiscard]] session_error make_error(session_error_kind kind,
                                       std::string field,
                                       std::string message) {
  return session_error{kind, std::nullopt, std::move(field), std::move(message)};
}

[[nodiscard]] session_error wrap_validator_error(validate_error error) {
  std::string field = error.offending_field_name;
  return session_error{session_error_kind::validator_rejected_envelope,
                       std::move(error),
                       std::move(field),
                       "stateless envelope validator rejected envelope"};
}

[[nodiscard]] direction opposite(direction value) {
  return value == direction::backend_to_core ? direction::core_to_backend
                                             : direction::backend_to_core;
}

}  // namespace

std::expected<protocol_session, session_error> protocol_session::make(direction local_direction) {
  if (local_direction == direction::reserved) {
    return std::unexpected(
        make_error(session_error_kind::invalid_local_direction,
                   "local_direction",
                   "local_direction must be backend_to_core or core_to_backend"));
  }
  return protocol_session(local_direction);
}

protocol_session::protocol_session(direction local_direction)
    : local_direction_(local_direction), remote_direction_(opposite(local_direction)) {}

direction protocol_session::local_direction() const {
  return local_direction_;
}

direction protocol_session::remote_direction() const {
  return remote_direction_;
}

std::uint64_t protocol_session::next_sequence_to_send() const {
  return next_sequence_to_send_;
}

std::uint64_t protocol_session::next_expected_receive_sequence() const {
  return next_expected_receive_sequence_;
}

bool protocol_session::has_received_boot() const {
  return has_received_boot_;
}

bool protocol_session::has_sent_boot() const {
  return has_sent_boot_;
}

bool protocol_session::has_received_boot_ack() const {
  return has_received_boot_ack_;
}

bool protocol_session::has_sent_boot_ack() const {
  return has_sent_boot_ack_;
}

bool protocol_session::has_pending_on_demand_reply() const {
  return has_pending_on_demand_reply_;
}

bool protocol_session::has_received_shutdown() const {
  return has_received_shutdown_;
}

std::expected<sync_envelope, session_error> protocol_session::build_envelope(
    envelope_kind kind,
    schema_id payload_schema,
    std::uint32_t payload_bytes,
    std::uint64_t sim_time_us) {
  sync_envelope env{};
  env.magic = kProtocolMagic;
  env.protocol_version = kProtocolVersion;
  env.kind = kind;
  env.sequence = next_sequence_to_send_;
  env.sim_time_us = sim_time_us;
  env.payload_bytes = payload_bytes;
  env.payload_schema = payload_schema;
  env.sender = local_direction_;

  auto valid = validate_envelope(env, local_direction_, next_sequence_to_send_);
  if (!valid.has_value()) {
    return std::unexpected(wrap_validator_error(valid.error()));
  }

  auto order = validate_send_order(kind);
  if (!order.has_value())
    return std::unexpected(order.error());

  apply_send(kind);
  ++next_sequence_to_send_;
  return env;
}

std::expected<void, session_error> protocol_session::accept_envelope(
    const sync_envelope& env, std::span<const std::uint8_t> payload) {
  auto valid = validate_envelope(env, remote_direction_, next_expected_receive_sequence_, payload);
  if (!valid.has_value()) {
    return std::unexpected(wrap_validator_error(valid.error()));
  }

  auto order = validate_receive_order(env.kind);
  if (!order.has_value())
    return std::unexpected(order.error());

  apply_receive(env.kind);
  ++next_expected_receive_sequence_;
  return {};
}

std::expected<void, session_error> protocol_session::validate_send_order(envelope_kind kind) {
  if (kind == envelope_kind::boot) {
    if (local_direction_ != direction::backend_to_core) {
      return std::unexpected(make_error(session_error_kind::boot_wrong_direction,
                                        "kind",
                                        "boot is only valid from backend_to_core"));
    }
    if (has_sent_boot_) {
      return std::unexpected(
          make_error(session_error_kind::duplicate_boot, "kind", "boot already sent"));
    }
    return {};
  }

  if (kind == envelope_kind::boot_ack) {
    if (local_direction_ != direction::core_to_backend) {
      return std::unexpected(make_error(session_error_kind::boot_ack_wrong_direction,
                                        "kind",
                                        "boot_ack is only valid from core_to_backend"));
    }
    if (!has_received_boot_) {
      return std::unexpected(make_error(session_error_kind::unexpected_boot_ack,
                                        "kind",
                                        "boot_ack cannot be sent before receiving boot"));
    }
    if (has_sent_boot_ack_) {
      return std::unexpected(
          make_error(session_error_kind::duplicate_boot_ack, "kind", "boot_ack already sent"));
    }
    return {};
  }

  if (local_direction_ == direction::backend_to_core && !has_sent_boot_) {
    return std::unexpected(make_error(session_error_kind::expected_local_boot_first,
                                      "kind",
                                      "backend must send boot before normal outbound traffic"));
  }

  if (local_direction_ == direction::core_to_backend && !has_received_boot_) {
    return std::unexpected(make_error(session_error_kind::expected_boot_first,
                                      "kind",
                                      "core must receive boot before normal outbound traffic"));
  }

  if (kind == envelope_kind::on_demand_request && has_pending_on_demand_reply_) {
    return std::unexpected(make_error(session_error_kind::pending_on_demand_reply,
                                      "kind",
                                      "an on-demand request is already awaiting a reply"));
  }

  return {};
}

std::expected<void, session_error> protocol_session::validate_receive_order(envelope_kind kind) {
  if (has_received_shutdown_) {
    return std::unexpected(make_error(session_error_kind::shutdown_already_received,
                                      "kind",
                                      "session already received shutdown"));
  }

  if (kind == envelope_kind::boot) {
    if (remote_direction_ != direction::backend_to_core) {
      return std::unexpected(make_error(session_error_kind::boot_wrong_direction,
                                        "kind",
                                        "boot is only valid from backend_to_core"));
    }
    if (has_received_boot_) {
      return std::unexpected(
          make_error(session_error_kind::duplicate_boot, "kind", "boot already received"));
    }
    return {};
  }

  if (kind == envelope_kind::boot_ack) {
    if (remote_direction_ != direction::core_to_backend) {
      return std::unexpected(make_error(session_error_kind::boot_ack_wrong_direction,
                                        "kind",
                                        "boot_ack is only valid from core_to_backend"));
    }
    if (!has_sent_boot_) {
      return std::unexpected(make_error(session_error_kind::unexpected_boot_ack,
                                        "kind",
                                        "boot_ack received before local boot was sent"));
    }
    if (has_received_boot_ack_) {
      return std::unexpected(
          make_error(session_error_kind::duplicate_boot_ack, "kind", "boot_ack already received"));
    }
    return {};
  }

  if (remote_direction_ == direction::backend_to_core && !has_received_boot_) {
    return std::unexpected(make_error(session_error_kind::expected_boot_first,
                                      "kind",
                                      "core must receive boot before normal inbound traffic"));
  }

  if (remote_direction_ == direction::core_to_backend) {
    if (!has_sent_boot_) {
      return std::unexpected(make_error(session_error_kind::expected_local_boot_first,
                                        "kind",
                                        "backend must send boot before accepting inbound traffic"));
    }
    if (!has_received_boot_ack_) {
      return std::unexpected(
          make_error(session_error_kind::expected_boot_ack_first,
                     "kind",
                     "backend must receive boot_ack before normal inbound traffic"));
    }
  }

  if (kind == envelope_kind::on_demand_reply && !has_pending_on_demand_reply_) {
    return std::unexpected(make_error(session_error_kind::on_demand_reply_without_request,
                                      "kind",
                                      "on-demand reply received with no pending request"));
  }

  return {};
}

void protocol_session::apply_send(envelope_kind kind) {
  if (kind == envelope_kind::boot) {
    has_sent_boot_ = true;
  } else if (kind == envelope_kind::boot_ack) {
    has_sent_boot_ack_ = true;
  } else if (kind == envelope_kind::on_demand_request) {
    has_pending_on_demand_reply_ = true;
  }
}

void protocol_session::apply_receive(envelope_kind kind) {
  if (kind == envelope_kind::boot) {
    has_received_boot_ = true;
  } else if (kind == envelope_kind::boot_ack) {
    has_received_boot_ack_ = true;
  } else if (kind == envelope_kind::on_demand_reply) {
    has_pending_on_demand_reply_ = false;
  } else if (kind == envelope_kind::shutdown) {
    has_received_shutdown_ = true;
  }
}

}  // namespace robosim::backend
