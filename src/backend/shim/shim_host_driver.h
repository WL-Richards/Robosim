#pragma once

#include "boot_descriptor.h"
#include "clock_state.h"
#include "notifier_state.h"
#include "shared_memory_transport.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <set>
#include <utility>

namespace robosim::backend::shim {

/**
 * Host-side protocol driver for an installed in-process shim.
 *
 * The driver owns the core_to_backend Tier 1 endpoint. It only publishes real
 * protocol traffic; callers remain responsible for driving shim_core::poll().
 */
class shim_host_driver {
 public:
  /** Creates the host endpoint over an existing Tier 1 shared region. */
  [[nodiscard]] static std::expected<shim_host_driver, tier1::tier1_transport_error> make(
      tier1::tier1_shared_region& region);

  /**
   * Receives the shim's boot descriptor and sends boot_ack.
   *
   * Returns the descriptor observed on the wire. The installed shim becomes
   * connected only after the caller subsequently polls it.
   */
  [[nodiscard]] std::expected<boot_descriptor, tier1::tier1_transport_error>
  accept_boot_and_send_ack(std::uint64_t sim_time_us);

  /** Publishes a fixed-size clock snapshot at an explicit simulation time. */
  [[nodiscard]] std::expected<void, tier1::tier1_transport_error> publish_clock_state(
      const clock_state& state, std::uint64_t sim_time_us);

  /** Publishes a notifier alarm batch using active-prefix serialization. */
  [[nodiscard]] std::expected<void, tier1::tier1_transport_error> publish_notifier_alarm_batch(
      const notifier_alarm_batch& batch, std::uint64_t sim_time_us);

  /** Publishes one fired notifier alarm event. */
  [[nodiscard]] std::expected<void, tier1::tier1_transport_error> publish_notifier_alarm(
      hal_handle handle, std::uint64_t fired_at_us, std::uint64_t sim_time_us);

  /** Receives one robot-to-host notifier state snapshot, if that is the next message. */
  [[nodiscard]] std::expected<std::optional<notifier_state>, tier1::tier1_transport_error>
  receive_notifier_state();

  shim_host_driver(const shim_host_driver&) = delete;
  shim_host_driver& operator=(const shim_host_driver&) = delete;
  shim_host_driver(shim_host_driver&&) = default;
  shim_host_driver& operator=(shim_host_driver&&) = default;

 private:
  explicit shim_host_driver(tier1::tier1_endpoint endpoint);

  tier1::tier1_endpoint endpoint_;
};

enum class shim_host_runtime_robot_output {
  ignored_message,
  notifier_state_received,
};

enum class shim_host_runtime_pump_result {
  clock_published,
  notifier_alarm_published,
  no_due_alarm,
};

/**
 * Deterministic host-side runtime pump for clock ticks and notifier alarms.
 *
 * The pump wraps shim_host_driver and sends at most one host-to-robot protocol
 * message per pump_to() call, preserving Tier 1 lane backpressure semantics.
 */
class shim_host_runtime_driver {
 public:
  /** Creates the runtime over an existing Tier 1 shared region. */
  [[nodiscard]] static std::expected<shim_host_runtime_driver, tier1::tier1_transport_error> make(
      tier1::tier1_shared_region& region);

  /** Receives boot and sends boot_ack via the underlying host driver. */
  [[nodiscard]] std::expected<boot_descriptor, tier1::tier1_transport_error>
  accept_boot_and_send_ack(std::uint64_t sim_time_us);

  /** Receives one robot-to-host output message and caches notifier state. */
  [[nodiscard]] std::expected<shim_host_runtime_robot_output, tier1::tier1_transport_error>
  poll_robot_outputs();

  /** Publishes at most one host-to-robot clock/alarm message for sim_time_us. */
  [[nodiscard]] std::expected<shim_host_runtime_pump_result, tier1::tier1_transport_error>
  pump_to(std::uint64_t sim_time_us);

  /** Latest notifier state received from robot-to-host protocol traffic. */
  [[nodiscard]] const std::optional<notifier_state>& latest_notifier_state() const noexcept;

  shim_host_runtime_driver(const shim_host_runtime_driver&) = delete;
  shim_host_runtime_driver& operator=(const shim_host_runtime_driver&) = delete;
  shim_host_runtime_driver(shim_host_runtime_driver&&) = default;
  shim_host_runtime_driver& operator=(shim_host_runtime_driver&&) = default;

 private:
  explicit shim_host_runtime_driver(shim_host_driver driver);

  shim_host_driver driver_;
  std::optional<notifier_state> latest_notifier_state_;
  std::optional<std::uint64_t> clock_published_for_time_us_;
  std::set<std::pair<hal_handle, std::uint64_t>> published_triggers_;
};

}  // namespace robosim::backend::shim
