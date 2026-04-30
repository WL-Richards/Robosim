#include "shim_core.h"

#include <cstring>
#include <span>
#include <utility>

namespace robosim::backend::shim {
namespace {

[[nodiscard]] shim_error wrap_send_error(tier1::tier1_transport_error err) {
  std::string field = err.offending_field_name;
  return shim_error{shim_error_kind::send_failed,
                    std::move(err),
                    std::move(field),
                    "transport rejected outbound boot envelope"};
}

[[nodiscard]] shim_error wrap_receive_error(tier1::tier1_transport_error err) {
  std::string field = err.offending_field_name;
  return shim_error{shim_error_kind::receive_failed,
                    std::move(err),
                    std::move(field),
                    "transport rejected inbound envelope"};
}

}  // namespace

shim_core::shim_core(tier1::tier1_endpoint endpoint) : endpoint_(std::move(endpoint)) {}

std::expected<shim_core, shim_error> shim_core::make(
    tier1::tier1_endpoint endpoint,
    const boot_descriptor& desc,
    std::uint64_t sim_time_us) {
  const auto* desc_bytes = reinterpret_cast<const std::uint8_t*>(&desc);
  std::span<const std::uint8_t> payload{desc_bytes, sizeof(boot_descriptor)};

  auto sent = endpoint.send(envelope_kind::boot,
                            schema_id::boot_descriptor,
                            payload,
                            sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return shim_core{std::move(endpoint)};
}

std::expected<void, shim_error> shim_core::poll() {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further inbound"});
  }

  auto received = endpoint_.try_receive();
  if (!received.has_value()) {
    if (received.error().kind == tier1::tier1_transport_error_kind::no_message) {
      return {};
    }
    return std::unexpected(wrap_receive_error(std::move(received.error())));
  }

  const auto& env = received->envelope;

  switch (env.kind) {
    case envelope_kind::boot_ack:
      connected_ = true;
      return {};

    case envelope_kind::tick_boundary:
      if (env.payload_schema == schema_id::clock_state) {
        clock_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(clock_state));
        latest_clock_state_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::power_state) {
        power_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(power_state));
        latest_power_state_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::ds_state) {
        ds_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(ds_state));
        latest_ds_state_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::can_frame_batch) {
        // Variable-size schema: copy received->payload.size() bytes (the
        // active prefix), not sizeof(can_frame_batch). The state{} zero-
        // init covers per-frame padding bytes and unused frames[count..63]
        // (D-C4-2 / D-C4-PADDING).
        can_frame_batch state{};
        std::memcpy(&state, received->payload.data(), received->payload.size());
        latest_can_frame_batch_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::can_status) {
        can_status state{};
        std::memcpy(&state, received->payload.data(), sizeof(can_status));
        latest_can_status_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::notifier_state) {
        // Variable-size: copy received->payload.size() bytes (active prefix),
        // not sizeof(notifier_state). The state{} zero-init covers the
        // 4-byte interior count→slots pad and per-slot trailing pad
        // (D-C6-VARIABLE-SIZE / D-C6-PADDING).
        notifier_state state{};
        std::memcpy(&state, received->payload.data(), received->payload.size());
        latest_notifier_state_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::notifier_alarm_batch) {
        // Variable-size: copy received->payload.size() bytes (active prefix),
        // not sizeof(notifier_alarm_batch). The state{} zero-init covers the
        // 4-byte interior count→events pad and unused events[count..31]
        // (D-C7-VARIABLE-SIZE / D-C7-PADDING). notifier_alarm_event has no
        // implicit padding (its reserved_pad is a named field).
        notifier_alarm_batch state{};
        std::memcpy(&state, received->payload.data(), received->payload.size());
        latest_notifier_alarm_batch_ = state;
        return {};
      }
      if (env.payload_schema == schema_id::error_message_batch) {
        // Variable-size: copy received->payload.size() bytes (active prefix).
        // error_message_batch has zero implicit C++ padding — both its 4-byte
        // interior count→messages reserved_pad and error_message's 3-byte
        // post-truncation_flags reserved_pad are NAMED fields (D-C8-PADDING-
        // FREE). The state{} zero-init still applies, covering unused
        // messages[count..7] for the shrinking-batch contract
        // (D-C8-VARIABLE-SIZE).
        error_message_batch state{};
        std::memcpy(&state, received->payload.data(), received->payload.size());
        latest_error_message_batch_ = state;
        return {};
      }
      // D-C8-DEAD-BRANCH: at cycle 8 every per-tick payload schema is wired,
      // so this fall-through is unreachable from valid traffic. Kept as a
      // defensive forward-compat structural guard — a future schema added
      // to protocol_version.h and the validator's allowed set but not yet
      // wired here will fail loudly rather than silently discarded.
      return std::unexpected(shim_error{shim_error_kind::unsupported_payload_schema,
                                        std::nullopt,
                                        "payload_schema",
                                        "shim does not yet handle this schema under tick_boundary"});

    case envelope_kind::shutdown:
      shutdown_observed_ = true;
      return {};

    default:
      return std::unexpected(shim_error{shim_error_kind::unsupported_envelope_kind,
                                        std::nullopt,
                                        "kind",
                                        "shim cycle 1 does not handle this envelope kind"});
  }
}

bool shim_core::is_connected() const {
  return connected_;
}

bool shim_core::is_shutting_down() const {
  return shutdown_observed_;
}

const std::optional<clock_state>& shim_core::latest_clock_state() const {
  return latest_clock_state_;
}

const std::optional<power_state>& shim_core::latest_power_state() const {
  return latest_power_state_;
}

const std::optional<ds_state>& shim_core::latest_ds_state() const {
  return latest_ds_state_;
}

const std::optional<can_frame_batch>& shim_core::latest_can_frame_batch() const {
  return latest_can_frame_batch_;
}

const std::optional<can_status>& shim_core::latest_can_status() const {
  return latest_can_status_;
}

const std::optional<notifier_state>& shim_core::latest_notifier_state() const {
  return latest_notifier_state_;
}

const std::optional<notifier_alarm_batch>& shim_core::latest_notifier_alarm_batch() const {
  return latest_notifier_alarm_batch_;
}

const std::optional<error_message_batch>& shim_core::latest_error_message_batch() const {
  return latest_error_message_batch_;
}

}  // namespace robosim::backend::shim
