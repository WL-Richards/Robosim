#include "shim_core.h"

#include "hal_c.h"

#include <algorithm>
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
                    "transport rejected outbound envelope"};
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
        for (std::uint32_t i = 0; i < state.count; ++i) {
          enqueue_can_rx_frame_for_streams(state.frames[i]);
        }
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

std::expected<void, shim_error> shim_core::send_can_frame_batch(
    const can_frame_batch& batch, std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  auto sent = endpoint_.send(envelope_kind::tick_boundary,
                             schema_id::can_frame_batch,
                             active_prefix_bytes(batch),
                             sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return {};
}

std::expected<void, shim_error> shim_core::send_notifier_state(
    const notifier_state& state, std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  auto sent = endpoint_.send(envelope_kind::tick_boundary,
                             schema_id::notifier_state,
                             active_prefix_bytes(state),
                             sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return {};
}

std::expected<void, shim_error> shim_core::send_error_message_batch(
    const error_message_batch& batch, std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  auto sent = endpoint_.send(envelope_kind::tick_boundary,
                             schema_id::error_message_batch,
                             active_prefix_bytes(batch),
                             sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return {};
}

void shim_core::enqueue_error(const error_message& msg) noexcept {
  if (pending_error_count_ >= kMaxErrorsPerBatch) {
    return;
  }
  pending_error_messages_[pending_error_count_] = msg;
  ++pending_error_count_;
}

void shim_core::enqueue_can_frame(const can_frame& frame) noexcept {
  if (pending_can_frame_count_ >= kMaxCanFramesPerBatch) {
    return;
  }
  pending_can_frames_[pending_can_frame_count_] = frame;
  ++pending_can_frame_count_;
}

std::expected<void, shim_error> shim_core::flush_pending_errors(
    std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  if (pending_error_count_ == 0) {
    return {};
  }

  error_message_batch batch{};
  batch.count = pending_error_count_;
  std::copy_n(pending_error_messages_.begin(),
              pending_error_count_,
              batch.messages.begin());

  auto sent = send_error_message_batch(batch, sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(std::move(sent.error()));
  }
  pending_error_count_ = 0;
  return {};
}

std::span<const error_message> shim_core::pending_error_messages() const noexcept {
  return {pending_error_messages_.data(), pending_error_count_};
}

std::expected<void, shim_error> shim_core::flush_pending_can_frames(
    std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  if (pending_can_frame_count_ == 0) {
    return {};
  }

  can_frame_batch batch{};
  batch.count = pending_can_frame_count_;
  std::copy_n(pending_can_frames_.begin(),
              pending_can_frame_count_,
              batch.frames.begin());
  const auto timestamp_us = static_cast<std::uint32_t>(sim_time_us);
  for (std::uint32_t i = 0; i < pending_can_frame_count_; ++i) {
    batch.frames[i].timestamp_us = timestamp_us;
  }

  auto sent = send_can_frame_batch(batch, sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(std::move(sent.error()));
  }
  pending_can_frame_count_ = 0;
  return {};
}

std::span<const can_frame> shim_core::pending_can_frames() const noexcept {
  return {pending_can_frames_.data(), pending_can_frame_count_};
}

std::uint32_t shim_core::open_can_stream_session(
    std::uint32_t message_id,
    std::uint32_t message_id_mask,
    std::uint32_t max_messages) {
  auto slot = std::ranges::find_if(can_stream_sessions_, [](const auto& session) {
    return !session.active;
  });
  if (slot == can_stream_sessions_.end()) {
    return 0;
  }

  slot->active = true;
  slot->handle = next_can_stream_handle_++;
  if (slot->handle == 0) {
    slot->handle = next_can_stream_handle_++;
  }
  slot->message_id = message_id;
  slot->message_id_mask = message_id_mask;
  slot->max_messages = max_messages;
  slot->overrun = false;
  slot->queued_frames.clear();
  slot->queued_frames.reserve(max_messages);
  return slot->handle;
}

void shim_core::close_can_stream_session(std::uint32_t handle) noexcept {
  auto slot = std::ranges::find_if(can_stream_sessions_, [handle](const auto& session) {
    return session.active && session.handle == handle;
  });
  if (slot == can_stream_sessions_.end()) {
    return;
  }
  slot->active = false;
  slot->handle = 0;
  slot->message_id = 0;
  slot->message_id_mask = 0;
  slot->max_messages = 0;
  slot->overrun = false;
  slot->queued_frames.clear();
}

std::int32_t shim_core::read_can_stream_session(
    std::uint32_t handle,
    std::span<can_frame> messages,
    std::uint32_t& messages_read) {
  messages_read = 0;
  auto slot = std::ranges::find_if(can_stream_sessions_, [handle](const auto& session) {
    return session.active && session.handle == handle;
  });
  if (slot == can_stream_sessions_.end()) {
    return kHalCanNotAllowed;
  }

  if (messages.empty()) {
    return kHalSuccess;
  }
  if (slot->queued_frames.empty()) {
    return kHalCanNoToken;
  }

  const auto count = std::min<std::size_t>(messages.size(), slot->queued_frames.size());
  std::copy_n(slot->queued_frames.begin(), count, messages.begin());
  slot->queued_frames.erase(slot->queued_frames.begin(),
                            slot->queued_frames.begin() +
                                static_cast<std::ptrdiff_t>(count));
  messages_read = static_cast<std::uint32_t>(count);

  if (slot->overrun) {
    slot->overrun = false;
    return kHalCanSessionOverrun;
  }
  return kHalSuccess;
}

void shim_core::enqueue_can_rx_frame_for_streams(const can_frame& frame) {
  for (auto& session : can_stream_sessions_) {
    if (!session.active) {
      continue;
    }
    if ((frame.message_id & session.message_id_mask) !=
        (session.message_id & session.message_id_mask)) {
      continue;
    }
    if (session.queued_frames.size() >= session.max_messages) {
      session.queued_frames.erase(session.queued_frames.begin());
      session.overrun = true;
    }
    session.queued_frames.push_back(frame);
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
