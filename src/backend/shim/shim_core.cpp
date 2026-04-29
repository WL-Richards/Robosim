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
      return std::unexpected(shim_error{shim_error_kind::unsupported_payload_schema,
                                        std::nullopt,
                                        "payload_schema",
                                        "shim cycle 1 only handles clock_state under tick_boundary"});

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

}  // namespace robosim::backend::shim
