#pragma once

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_session.h"
#include "shared_memory_transport.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <string>

namespace robosim::backend::shim {

enum class shim_error_kind {
  send_failed,
  receive_failed,
  unsupported_envelope_kind,
  unsupported_payload_schema,
  shutdown_already_observed,
};

struct shim_error {
  shim_error_kind kind;
  std::optional<tier1::tier1_transport_error> transport_error;
  std::string offending_field_name;
  std::string message;

  bool operator==(const shim_error&) const = default;
};

// In-process backend-side orchestrator for the HAL <-> Sim Core
// protocol. Cycle 1 covers the boot handshake plus the inbound
// clock_state cache slot and the shutdown terminal receive path.
//
// The shim owns one tier1_endpoint constructed with
// direction::backend_to_core. Construction publishes one boot
// envelope carrying `desc`; from there the caller drives `poll()` to
// drain inbound traffic.
class shim_core {
 public:
  [[nodiscard]] static std::expected<shim_core, shim_error> make(
      tier1::tier1_endpoint endpoint,
      const boot_descriptor& desc,
      std::uint64_t sim_time_us);

  [[nodiscard]] std::expected<void, shim_error> poll();

  // Publishes `batch` as a tick_boundary envelope into the shim's
  // outbound backend_to_core lane. Sends exactly the active prefix
  // (offsetof(can_frame_batch, frames) + batch.count * sizeof(can_frame))
  // — NOT sizeof(can_frame_batch) — so the validator's count-vs-length
  // contract holds. Returns send_failed wrapping the underlying tier1
  // transport error if the lane is busy / in-progress, or
  // shutdown_already_observed once is_shutting_down() is true.
  [[nodiscard]] std::expected<void, shim_error> send_can_frame_batch(
      const can_frame_batch& batch, std::uint64_t sim_time_us);

  // Publishes `state` as a tick_boundary envelope into the shim's
  // outbound backend_to_core lane. Sends exactly the active prefix
  // (offsetof(notifier_state, slots) + state.count *
  // sizeof(notifier_slot)) — note the 8-byte header offset (not 4),
  // because notifier_slot has alignof 8. Same shutdown_already_observed
  // and send_failed semantics as send_can_frame_batch.
  [[nodiscard]] std::expected<void, shim_error> send_notifier_state(
      const notifier_state& state, std::uint64_t sim_time_us);

  // Publishes `batch` as a tick_boundary envelope into the shim's
  // outbound backend_to_core lane. Sends exactly the active prefix
  // (offsetof(error_message_batch, messages) + batch.count *
  // sizeof(error_message)). The 8-byte header offset comes from the
  // NAMED reserved_pad[4] field (not implicit padding); per
  // D-C8-PADDING-FREE the schema has zero implicit C++ padding.
  // Same shutdown_already_observed and send_failed semantics as the
  // other typed send methods.
  [[nodiscard]] std::expected<void, shim_error> send_error_message_batch(
      const error_message_batch& batch, std::uint64_t sim_time_us);

  [[nodiscard]] bool is_connected() const;
  [[nodiscard]] bool is_shutting_down() const;
  [[nodiscard]] const std::optional<clock_state>& latest_clock_state() const;
  [[nodiscard]] const std::optional<power_state>& latest_power_state() const;
  [[nodiscard]] const std::optional<ds_state>& latest_ds_state() const;
  [[nodiscard]] const std::optional<can_frame_batch>& latest_can_frame_batch() const;
  [[nodiscard]] const std::optional<can_status>& latest_can_status() const;
  [[nodiscard]] const std::optional<notifier_state>& latest_notifier_state() const;
  [[nodiscard]] const std::optional<notifier_alarm_batch>& latest_notifier_alarm_batch() const;
  [[nodiscard]] const std::optional<error_message_batch>& latest_error_message_batch() const;

  shim_core(const shim_core&) = delete;
  shim_core& operator=(const shim_core&) = delete;
  shim_core(shim_core&&) = default;
  shim_core& operator=(shim_core&&) = default;

 private:
  explicit shim_core(tier1::tier1_endpoint endpoint);

  tier1::tier1_endpoint endpoint_;
  bool connected_ = false;
  bool shutdown_observed_ = false;
  std::optional<clock_state> latest_clock_state_;
  std::optional<power_state> latest_power_state_;
  std::optional<ds_state> latest_ds_state_;
  std::optional<can_frame_batch> latest_can_frame_batch_;
  std::optional<can_status> latest_can_status_;
  std::optional<notifier_state> latest_notifier_state_;
  std::optional<notifier_alarm_batch> latest_notifier_alarm_batch_;
  std::optional<error_message_batch> latest_error_message_batch_;
};

}  // namespace robosim::backend::shim
