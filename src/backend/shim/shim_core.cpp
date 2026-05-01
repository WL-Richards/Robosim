#include "shim_core.h"

#include "hal_c.h"

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <span>
#include <utility>

extern "C" void WPI_SetEvent(WPI_EventHandle handle) __attribute__((weak));

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

struct shim_core::notifier_wait_state {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<notifier_alarm_event> pending_alarm_events;
  std::uint32_t total_waiters = 0;
  std::map<std::int32_t, std::uint32_t> waiters_by_handle;
  bool shutdown_wake = false;
};

shim_core::shim_core(tier1::tier1_endpoint endpoint, const boot_descriptor& desc)
    : endpoint_(std::move(endpoint)),
      boot_descriptor_(desc),
      notifier_wait_state_(std::make_shared<notifier_wait_state>()) {}

std::expected<shim_core, shim_error> shim_core::make(tier1::tier1_endpoint endpoint,
                                                     const boot_descriptor& desc,
                                                     std::uint64_t sim_time_us) {
  const auto* desc_bytes = reinterpret_cast<const std::uint8_t*>(&desc);
  std::span<const std::uint8_t> payload{desc_bytes, sizeof(boot_descriptor)};

  auto sent = endpoint.send(envelope_kind::boot, schema_id::boot_descriptor, payload, sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return shim_core{std::move(endpoint), desc};
}

std::expected<void, shim_error> shim_core::poll() {
  auto result = poll_one();
  if (!result.has_value()) {
    return std::unexpected(std::move(result.error()));
  }
  return {};
}

bool shim_core::refresh_ds_data() {
  auto result = poll_one();
  return result.has_value() && *result == inbound_dispatch_result::ds_state;
}

std::expected<shim_core::inbound_dispatch_result, shim_error> shim_core::poll_one() {
  std::lock_guard cache_lock{*cache_mutex_};
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further inbound"});
  }

  auto received = endpoint_.try_receive();
  if (!received.has_value()) {
    if (received.error().kind == tier1::tier1_transport_error_kind::no_message) {
      return inbound_dispatch_result::no_message;
    }
    return std::unexpected(wrap_receive_error(std::move(received.error())));
  }

  const auto& env = received->envelope;

  switch (env.kind) {
    case envelope_kind::boot_ack:
      connected_ = true;
      return inbound_dispatch_result::boot_ack;

    case envelope_kind::tick_boundary:
      if (env.payload_schema == schema_id::clock_state) {
        clock_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(clock_state));
        latest_clock_state_ = state;
        return inbound_dispatch_result::clock_state;
      }
      if (env.payload_schema == schema_id::power_state) {
        power_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(power_state));
        latest_power_state_ = state;
        return inbound_dispatch_result::power_state;
      }
      if (env.payload_schema == schema_id::ds_state) {
        ds_state state{};
        std::memcpy(&state, received->payload.data(), sizeof(ds_state));
        latest_ds_state_ = state;
        wake_new_data_event_handles();
        return inbound_dispatch_result::ds_state;
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
        return inbound_dispatch_result::can_frame_batch;
      }
      if (env.payload_schema == schema_id::can_status) {
        can_status state{};
        std::memcpy(&state, received->payload.data(), sizeof(can_status));
        latest_can_status_ = state;
        return inbound_dispatch_result::can_status;
      }
      if (env.payload_schema == schema_id::notifier_state) {
        // Variable-size: copy received->payload.size() bytes (active prefix),
        // not sizeof(notifier_state). The state{} zero-init covers the
        // 4-byte interior count→slots pad and per-slot trailing pad
        // (D-C6-VARIABLE-SIZE / D-C6-PADDING).
        notifier_state state{};
        std::memcpy(&state, received->payload.data(), received->payload.size());
        latest_notifier_state_ = state;
        return inbound_dispatch_result::notifier_state;
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
        {
          std::lock_guard lock{notifier_wait_state_->mutex};
          for (std::uint32_t i = 0; i < state.count; ++i) {
            notifier_wait_state_->pending_alarm_events.push_back(state.events[i]);
          }
        }
        notifier_wait_state_->cv.notify_all();
        return inbound_dispatch_result::notifier_alarm_batch;
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
        return inbound_dispatch_result::error_message_batch;
      }
      // D-C8-DEAD-BRANCH: at cycle 8 every per-tick payload schema is wired,
      // so this fall-through is unreachable from valid traffic. Kept as a
      // defensive forward-compat structural guard — a future schema added
      // to protocol_version.h and the validator's allowed set but not yet
      // wired here will fail loudly rather than silently discarded.
      return std::unexpected(
          shim_error{shim_error_kind::unsupported_payload_schema,
                     std::nullopt,
                     "payload_schema",
                     "shim does not yet handle this schema under tick_boundary"});

    case envelope_kind::shutdown:
      shutdown_observed_ = true;
      return inbound_dispatch_result::shutdown;

    default:
      return std::unexpected(shim_error{shim_error_kind::unsupported_envelope_kind,
                                        std::nullopt,
                                        "kind",
                                        "shim cycle 1 does not handle this envelope kind"});
  }
}

std::expected<void, shim_error> shim_core::send_can_frame_batch(const can_frame_batch& batch,
                                                                std::uint64_t sim_time_us) {
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

void shim_core::provide_new_data_event_handle(WPI_EventHandle handle) {
  if (handle == 0) {
    return;
  }
  if (std::ranges::find(new_data_event_handles_, handle) != new_data_event_handles_.end()) {
    return;
  }
  new_data_event_handles_.push_back(handle);
}

void shim_core::remove_new_data_event_handle(WPI_EventHandle handle) {
  if (handle == 0) {
    return;
  }
  std::erase(new_data_event_handles_, handle);
}

void shim_core::observe_user_program(enum user_program_observer_state state) noexcept {
  user_program_observer_state_ = state;
}

std::int64_t shim_core::record_usage_report(std::int32_t resource,
                                            std::int32_t instance_number,
                                            std::int32_t context,
                                            std::string_view feature) {
  usage_reports_.push_back(usage_report_record{
      resource,
      instance_number,
      context,
      std::string{feature},
  });
  return static_cast<std::int64_t>(usage_reports_.size());
}

std::int32_t shim_core::set_joystick_outputs(std::int32_t joystick_num,
                                             std::int64_t outputs,
                                             std::int32_t left_rumble,
                                             std::int32_t right_rumble) noexcept {
  if (joystick_num < 0 || joystick_num >= static_cast<std::int32_t>(kMaxJoysticks)) {
    return kHalHandleError;
  }
  joystick_outputs_[static_cast<std::size_t>(joystick_num)] =
      joystick_output_state{outputs, left_rumble, right_rumble};
  return kHalSuccess;
}

void shim_core::wake_new_data_event_handles() const {
  if (WPI_SetEvent == nullptr) {
    return;
  }
  for (const WPI_EventHandle handle : new_data_event_handles_) {
    WPI_SetEvent(handle);
  }
}

std::expected<void, shim_error> shim_core::send_notifier_state(const notifier_state& state,
                                                               std::uint64_t sim_time_us) {
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

std::expected<void, shim_error> shim_core::send_joystick_output_batch(
    const joystick_output_batch& batch, std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  auto sent = endpoint_.send(envelope_kind::tick_boundary,
                             schema_id::joystick_output_batch,
                             active_prefix_bytes(batch),
                             sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(wrap_send_error(std::move(sent.error())));
  }
  return {};
}

std::expected<void, shim_error> shim_core::send_user_program_observer_snapshot(
    const ::robosim::backend::user_program_observer_snapshot& snapshot,
    std::uint64_t sim_time_us) {
  if (shutdown_observed_) {
    return std::unexpected(shim_error{shim_error_kind::shutdown_already_observed,
                                      std::nullopt,
                                      "kind",
                                      "shim already observed shutdown; refusing further outbound"});
  }
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&snapshot);
  auto sent = endpoint_.send(envelope_kind::tick_boundary,
                             schema_id::user_program_observer_snapshot,
                             std::span<const std::uint8_t>{bytes, sizeof(snapshot)},
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

std::expected<void, shim_error> shim_core::flush_pending_errors(std::uint64_t sim_time_us) {
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
  std::copy_n(pending_error_messages_.begin(), pending_error_count_, batch.messages.begin());

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

std::expected<void, shim_error> shim_core::flush_pending_can_frames(std::uint64_t sim_time_us) {
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
  std::copy_n(pending_can_frames_.begin(), pending_can_frame_count_, batch.frames.begin());
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

std::uint32_t shim_core::open_can_stream_session(std::uint32_t message_id,
                                                 std::uint32_t message_id_mask,
                                                 std::uint32_t max_messages) {
  auto slot = std::ranges::find_if(can_stream_sessions_,
                                   [](const auto& session) { return !session.active; });
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

std::int32_t shim_core::read_can_stream_session(std::uint32_t handle,
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
                            slot->queued_frames.begin() + static_cast<std::ptrdiff_t>(count));
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

std::int32_t shim_core::find_notifier_slot(std::int32_t handle) const noexcept {
  if (handle == 0) {
    return -1;
  }
  for (std::size_t i = 0; i < notifier_records_.size(); ++i) {
    if (notifier_records_[i].active && notifier_records_[i].slot.handle == handle) {
      return static_cast<std::int32_t>(i);
    }
  }
  return -1;
}

std::int32_t shim_core::initialize_notifier() noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  auto slot =
      std::ranges::find_if(notifier_records_, [](const auto& record) { return !record.active; });
  if (slot == notifier_records_.end()) {
    return 0;
  }

  std::int32_t handle = next_notifier_handle_++;
  if (handle == 0) {
    handle = next_notifier_handle_++;
  }

  slot->active = true;
  slot->allocation_order = next_notifier_allocation_order_++;
  slot->stopped = false;
  slot->stop_wake_count = 0;
  slot->slot = notifier_slot{};
  slot->slot.handle = handle;
  return handle;
}

std::int32_t shim_core::set_notifier_name(std::int32_t handle, std::string_view name) noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  const std::int32_t index = find_notifier_slot(handle);
  if (index < 0) {
    return kHalHandleError;
  }

  auto& out = notifier_records_[static_cast<std::size_t>(index)].slot.name;
  out.fill('\0');
  const std::size_t bytes_to_copy = std::min(name.size(), out.size() - std::size_t{1});
  if (bytes_to_copy > 0) {
    std::memcpy(out.data(), name.data(), bytes_to_copy);
  }
  return kHalSuccess;
}

std::int32_t shim_core::update_notifier_alarm(std::int32_t handle,
                                              std::uint64_t trigger_time_us) noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  const std::int32_t index = find_notifier_slot(handle);
  if (index < 0) {
    return kHalHandleError;
  }

  auto& slot = notifier_records_[static_cast<std::size_t>(index)].slot;
  slot.trigger_time_us = trigger_time_us;
  slot.alarm_active = 1;
  slot.canceled = 0;
  notifier_records_[static_cast<std::size_t>(index)].stopped = false;
  return kHalSuccess;
}

std::int32_t shim_core::cancel_notifier_alarm(std::int32_t handle) noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  const std::int32_t index = find_notifier_slot(handle);
  if (index < 0) {
    return kHalHandleError;
  }

  auto& slot = notifier_records_[static_cast<std::size_t>(index)].slot;
  slot.trigger_time_us = 0;
  slot.alarm_active = 0;
  slot.canceled = 1;
  notifier_records_[static_cast<std::size_t>(index)].stopped = false;
  return kHalSuccess;
}

std::int32_t shim_core::stop_notifier(std::int32_t handle) noexcept {
  {
    std::lock_guard lock{notifier_wait_state_->mutex};
    const std::int32_t index = find_notifier_slot(handle);
    if (index < 0) {
      return kHalHandleError;
    }

    auto& record = notifier_records_[static_cast<std::size_t>(index)];
    record.slot.trigger_time_us = 0;
    record.slot.alarm_active = 0;
    record.slot.canceled = 1;
    record.stopped = true;
    ++record.stop_wake_count;
    std::erase_if(notifier_wait_state_->pending_alarm_events,
                  [handle](const notifier_alarm_event& event) { return event.handle == handle; });
  }
  notifier_wait_state_->cv.notify_all();
  return kHalSuccess;
}

void shim_core::clean_notifier(std::int32_t handle) noexcept {
  {
    std::lock_guard lock{notifier_wait_state_->mutex};
    const std::int32_t index = find_notifier_slot(handle);
    if (index < 0) {
      return;
    }
    std::erase_if(notifier_wait_state_->pending_alarm_events,
                  [handle](const notifier_alarm_event& event) { return event.handle == handle; });
    notifier_records_[static_cast<std::size_t>(index)] = notifier_record{};
  }
  notifier_wait_state_->cv.notify_all();
}

notifier_state shim_core::current_notifier_state() const noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  notifier_state state{};
  std::array<const notifier_record*, kMaxNotifiers> active{};
  std::size_t active_count = 0;
  for (const auto& record : notifier_records_) {
    if (record.active) {
      active[active_count] = &record;
      ++active_count;
    }
  }
  std::sort(active.begin(),
            active.begin() + static_cast<std::ptrdiff_t>(active_count),
            [](const notifier_record* lhs, const notifier_record* rhs) {
              return lhs->allocation_order < rhs->allocation_order;
            });

  state.count = static_cast<std::uint32_t>(active_count);
  for (std::size_t i = 0; i < active_count; ++i) {
    state.slots[i] = active[i]->slot;
  }
  return state;
}

std::expected<void, shim_error> shim_core::flush_notifier_state(std::uint64_t sim_time_us) {
  return send_notifier_state(current_notifier_state(), sim_time_us);
}

joystick_output_batch shim_core::current_joystick_output_batch() const noexcept {
  joystick_output_batch batch{};
  for (std::size_t joystick = 0; joystick < joystick_outputs_.size(); ++joystick) {
    const auto& stored = joystick_outputs_[joystick];
    if (!stored.has_value()) {
      continue;
    }
    auto& out = batch.outputs[batch.count];
    out.joystick_num = static_cast<std::int32_t>(joystick);
    out.outputs = stored->outputs;
    out.left_rumble = stored->left_rumble;
    out.right_rumble = stored->right_rumble;
    ++batch.count;
  }
  return batch;
}

std::expected<void, shim_error> shim_core::flush_joystick_outputs(std::uint64_t sim_time_us) {
  return send_joystick_output_batch(current_joystick_output_batch(), sim_time_us);
}

::robosim::backend::user_program_observer_snapshot
shim_core::current_user_program_observer_snapshot() const noexcept {
  ::robosim::backend::user_program_observer_snapshot snapshot{};
  switch (user_program_observer_state_) {
    case user_program_observer_state::none:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::none;
      break;
    case user_program_observer_state::starting:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::starting;
      break;
    case user_program_observer_state::disabled:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::disabled;
      break;
    case user_program_observer_state::autonomous:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::autonomous;
      break;
    case user_program_observer_state::teleop:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::teleop;
      break;
    case user_program_observer_state::test:
      snapshot.mode = ::robosim::backend::user_program_observer_mode::test;
      break;
  }
  return snapshot;
}

std::expected<void, shim_error> shim_core::flush_user_program_observer(
    std::uint64_t sim_time_us) {
  return send_user_program_observer_snapshot(current_user_program_observer_snapshot(), sim_time_us);
}

std::expected<schema_id, shim_error> shim_core::flush_next_driver_station_output(
    std::uint64_t sim_time_us) {
  switch (driver_station_output_flush_phase_) {
    case driver_station_output_flush_phase::user_program_observer: {
      auto flushed = flush_user_program_observer(sim_time_us);
      if (!flushed.has_value()) {
        return std::unexpected(flushed.error());
      }
      driver_station_output_flush_phase_ = driver_station_output_flush_phase::joystick_outputs;
      return schema_id::user_program_observer_snapshot;
    }
    case driver_station_output_flush_phase::joystick_outputs: {
      auto flushed = flush_joystick_outputs(sim_time_us);
      if (!flushed.has_value()) {
        return std::unexpected(flushed.error());
      }
      driver_station_output_flush_phase_ =
          driver_station_output_flush_phase::user_program_observer;
      return schema_id::joystick_output_batch;
    }
  }
  return std::unexpected(shim_error{shim_error_kind::unsupported_payload_schema,
                                    std::nullopt,
                                    "driver_station_output_flush_phase",
                                    "unknown Driver Station output flush phase"});
}

std::uint64_t shim_core::wait_for_notifier_alarm(std::int32_t handle, std::int32_t& status) {
  auto wait_state = notifier_wait_state_;
  std::unique_lock lock{wait_state->mutex};

  const auto remove_first_matching_event = [wait_state, handle]() -> std::optional<std::uint64_t> {
    auto found = std::ranges::find_if(
        wait_state->pending_alarm_events,
        [handle](const notifier_alarm_event& event) { return event.handle == handle; });
    if (found == wait_state->pending_alarm_events.end()) {
      return std::nullopt;
    }
    const std::uint64_t fired_at_us = found->fired_at_us;
    wait_state->pending_alarm_events.erase(found);
    return fired_at_us;
  };

  const std::int32_t initial_index = find_notifier_slot(handle);
  if (initial_index < 0 || wait_state->shutdown_wake) {
    status = kHalHandleError;
    return 0;
  }

  ++wait_state->total_waiters;
  ++wait_state->waiters_by_handle[handle];
  const auto unregister_waiter = [wait_state, handle]() {
    --wait_state->total_waiters;
    auto waiter_count = wait_state->waiters_by_handle.find(handle);
    if (waiter_count != wait_state->waiters_by_handle.end()) {
      --waiter_count->second;
      if (waiter_count->second == 0) {
        wait_state->waiters_by_handle.erase(waiter_count);
      }
    }
  };

  while (true) {
    if (wait_state->shutdown_wake) {
      unregister_waiter();
      status = kHalHandleError;
      return 0;
    }

    const std::int32_t index = find_notifier_slot(handle);
    if (index < 0) {
      unregister_waiter();
      status = kHalHandleError;
      return 0;
    }

    const auto& record = notifier_records_[static_cast<std::size_t>(index)];
    if (record.stopped) {
      unregister_waiter();
      status = kHalSuccess;
      return 0;
    }

    if (auto fired_at_us = remove_first_matching_event(); fired_at_us.has_value()) {
      unregister_waiter();
      status = kHalSuccess;
      return *fired_at_us;
    }

    wait_state->cv.wait(lock);
  }
}

void shim_core::prepare_for_hal_initialize() {
  std::lock_guard lock{notifier_wait_state_->mutex};
  notifier_wait_state_->shutdown_wake = false;
}

void shim_core::shutdown_hal_waits() {
  {
    std::lock_guard lock{notifier_wait_state_->mutex};
    notifier_wait_state_->shutdown_wake = true;
  }
  notifier_wait_state_->cv.notify_all();
}

std::uint32_t shim_core::pending_notifier_wait_count() const noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  return notifier_wait_state_->total_waiters;
}

std::uint32_t shim_core::pending_notifier_wait_count(std::int32_t handle) const noexcept {
  std::lock_guard lock{notifier_wait_state_->mutex};
  const auto found = notifier_wait_state_->waiters_by_handle.find(handle);
  return found == notifier_wait_state_->waiters_by_handle.end() ? 0u : found->second;
}

bool shim_core::is_connected() const {
  std::lock_guard cache_lock{*cache_mutex_};
  return connected_;
}

bool shim_core::is_shutting_down() const {
  return shutdown_observed_;
}

const std::optional<clock_state>& shim_core::latest_clock_state() const {
  return latest_clock_state_;
}

std::optional<clock_state> shim_core::latest_clock_state_snapshot() const {
  std::lock_guard cache_lock{*cache_mutex_};
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

user_program_observer_state shim_core::user_program_observer_state() const noexcept {
  return user_program_observer_state_;
}

std::span<const usage_report_record> shim_core::usage_reports() const noexcept {
  return usage_reports_;
}

std::optional<joystick_output_state> shim_core::joystick_outputs(
    std::int32_t joystick_num) const noexcept {
  if (joystick_num < 0 || joystick_num >= static_cast<std::int32_t>(kMaxJoysticks)) {
    return std::nullopt;
  }
  return joystick_outputs_[static_cast<std::size_t>(joystick_num)];
}

const boot_descriptor& shim_core::boot_descriptor_snapshot() const noexcept {
  return boot_descriptor_;
}

}  // namespace robosim::backend::shim
