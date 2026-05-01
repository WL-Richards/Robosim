#include "shim_host_driver.h"

#include "sync_envelope.h"

#include <cstddef>
#include <cstring>
#include <span>

namespace robosim::backend::shim {
namespace {

[[nodiscard]] std::span<const std::uint8_t> bytes_of_clock_state(const clock_state& state) {
  return {reinterpret_cast<const std::uint8_t*>(&state), sizeof(clock_state)};
}

[[nodiscard]] clock_state make_default_runtime_clock_state(std::uint64_t sim_time_us) {
  clock_state state{};
  state.sim_time_us = sim_time_us;
  state.system_active = 1;
  state.system_time_valid = 1;
  state.rsl_state = 1;
  return state;
}

[[nodiscard]] std::span<const std::uint8_t> active_prefix_bytes(
    const notifier_alarm_batch& batch) {
  const std::size_t active_size =
      offsetof(notifier_alarm_batch, events) +
      static_cast<std::size_t>(batch.count) * sizeof(notifier_alarm_event);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

}  // namespace

shim_host_driver::shim_host_driver(tier1::tier1_endpoint endpoint)
    : endpoint_(std::move(endpoint)) {}

std::expected<shim_host_driver, tier1::tier1_transport_error> shim_host_driver::make(
    tier1::tier1_shared_region& region) {
  auto endpoint = tier1::tier1_endpoint::make(region, direction::core_to_backend);
  if (!endpoint.has_value()) {
    return std::unexpected(std::move(endpoint.error()));
  }
  return shim_host_driver{std::move(*endpoint)};
}

std::expected<boot_descriptor, tier1::tier1_transport_error>
shim_host_driver::accept_boot_and_send_ack(std::uint64_t sim_time_us) {
  auto boot_msg = endpoint_.try_receive();
  if (!boot_msg.has_value()) {
    return std::unexpected(std::move(boot_msg.error()));
  }

  boot_descriptor descriptor{};
  std::memcpy(&descriptor, boot_msg->payload.data(), sizeof(boot_descriptor));

  auto ack = endpoint_.send(envelope_kind::boot_ack, schema_id::none, {}, sim_time_us);
  if (!ack.has_value()) {
    return std::unexpected(std::move(ack.error()));
  }
  return descriptor;
}

std::expected<void, tier1::tier1_transport_error> shim_host_driver::publish_clock_state(
    const clock_state& state, std::uint64_t sim_time_us) {
  return endpoint_.send(envelope_kind::tick_boundary,
                        schema_id::clock_state,
                        bytes_of_clock_state(state),
                        sim_time_us);
}

std::expected<void, tier1::tier1_transport_error> shim_host_driver::publish_notifier_alarm_batch(
    const notifier_alarm_batch& batch, std::uint64_t sim_time_us) {
  return endpoint_.send(envelope_kind::tick_boundary,
                        schema_id::notifier_alarm_batch,
                        active_prefix_bytes(batch),
                        sim_time_us);
}

std::expected<void, tier1::tier1_transport_error> shim_host_driver::publish_notifier_alarm(
    hal_handle handle, std::uint64_t fired_at_us, std::uint64_t sim_time_us) {
  notifier_alarm_batch batch{};
  batch.count = 1;
  batch.events[0].fired_at_us = fired_at_us;
  batch.events[0].handle = handle;
  return publish_notifier_alarm_batch(batch, sim_time_us);
}

std::expected<std::optional<notifier_state>, tier1::tier1_transport_error>
shim_host_driver::receive_notifier_state() {
  auto received = endpoint_.try_receive();
  if (!received.has_value()) {
    return std::unexpected(std::move(received.error()));
  }
  if (received->envelope.kind != envelope_kind::tick_boundary ||
      received->envelope.payload_schema != schema_id::notifier_state) {
    return std::optional<notifier_state>{};
  }

  notifier_state state{};
  std::memcpy(&state, received->payload.data(), received->payload.size());
  return state;
}

shim_host_runtime_driver::shim_host_runtime_driver(shim_host_driver driver)
    : driver_(std::move(driver)) {}

std::expected<shim_host_runtime_driver, tier1::tier1_transport_error>
shim_host_runtime_driver::make(tier1::tier1_shared_region& region) {
  auto driver = shim_host_driver::make(region);
  if (!driver.has_value()) {
    return std::unexpected(std::move(driver.error()));
  }
  return shim_host_runtime_driver{std::move(*driver)};
}

std::expected<boot_descriptor, tier1::tier1_transport_error>
shim_host_runtime_driver::accept_boot_and_send_ack(std::uint64_t sim_time_us) {
  return driver_.accept_boot_and_send_ack(sim_time_us);
}

std::expected<shim_host_runtime_robot_output, tier1::tier1_transport_error>
shim_host_runtime_driver::poll_robot_outputs() {
  auto received = driver_.receive_notifier_state();
  if (!received.has_value()) {
    return std::unexpected(std::move(received.error()));
  }
  if (!received->has_value()) {
    return shim_host_runtime_robot_output::ignored_message;
  }
  latest_notifier_state_ = **received;
  return shim_host_runtime_robot_output::notifier_state_received;
}

std::expected<shim_host_runtime_pump_result, tier1::tier1_transport_error>
shim_host_runtime_driver::pump_to(std::uint64_t sim_time_us) {
  if (!clock_published_for_time_us_.has_value() ||
      *clock_published_for_time_us_ != sim_time_us) {
    const auto sent =
        driver_.publish_clock_state(make_default_runtime_clock_state(sim_time_us), sim_time_us);
    if (!sent.has_value()) {
      return std::unexpected(std::move(sent.error()));
    }
    clock_published_for_time_us_ = sim_time_us;
    return shim_host_runtime_pump_result::clock_published;
  }

  notifier_alarm_batch due{};
  if (latest_notifier_state_.has_value()) {
    const std::uint32_t count = latest_notifier_state_->count;
    for (std::uint32_t i = 0; i < count && due.count < kMaxAlarmsPerBatch; ++i) {
      const auto& slot = latest_notifier_state_->slots[i];
      const auto trigger_key = std::pair{slot.handle, slot.trigger_time_us};
      if (slot.alarm_active == 0 || slot.canceled != 0 || slot.trigger_time_us > sim_time_us ||
          published_triggers_.contains(trigger_key)) {
        continue;
      }
      due.events[due.count].handle = slot.handle;
      due.events[due.count].fired_at_us = slot.trigger_time_us;
      ++due.count;
    }
  }

  if (due.count == 0) {
    return shim_host_runtime_pump_result::no_due_alarm;
  }

  const auto sent = driver_.publish_notifier_alarm_batch(due, sim_time_us);
  if (!sent.has_value()) {
    return std::unexpected(std::move(sent.error()));
  }
  for (std::uint32_t i = 0; i < due.count; ++i) {
    published_triggers_.emplace(due.events[i].handle, due.events[i].fired_at_us);
  }
  return shim_host_runtime_pump_result::notifier_alarm_published;
}

const std::optional<notifier_state>& shim_host_runtime_driver::latest_notifier_state()
    const noexcept {
  return latest_notifier_state_;
}

}  // namespace robosim::backend::shim
