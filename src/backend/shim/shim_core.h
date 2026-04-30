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

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace robosim::backend::shim {

/** High-level shim orchestration failures. */
enum class shim_error_kind {
  send_failed,
  receive_failed,
  unsupported_envelope_kind,
  unsupported_payload_schema,
  shutdown_already_observed,
};

/** Shim failure with optional wrapped Tier 1 transport details. */
struct shim_error {
  shim_error_kind kind;
  std::optional<tier1::tier1_transport_error> transport_error;
  std::string offending_field_name;
  std::string message;

  bool operator==(const shim_error&) const = default;
};

/**
 * In-process backend-side orchestrator for the HAL <-> Sim Core protocol.
 *
 * shim_core owns a backend_to_core Tier 1 endpoint. make() immediately
 * publishes the boot descriptor; callers then drive poll() to accept boot_ack,
 * per-tick state snapshots, and shutdown. The C HAL ABI reads the latest
 * cached state through the process-global current() pointer.
 */
class shim_core {
 public:
  /** Creates a shim and sends the initial boot descriptor envelope. */
  [[nodiscard]] static std::expected<shim_core, shim_error> make(
      tier1::tier1_endpoint endpoint,
      const boot_descriptor& desc,
      std::uint64_t sim_time_us);

  /**
   * Installs the non-owning process-global shim pointer used by HAL_* exports.
   *
   * Callers install after make() succeeds and clear with nullptr before
   * destruction. v0 is single-threaded; hal_c.cpp owns the backing storage.
   */
  static void install_global(shim_core* shim) noexcept;

  /** Returns the non-owning process-global shim pointer used by HAL_* exports. */
  [[nodiscard]] static shim_core* current() noexcept;

  /** Drains at most one inbound message and updates connection/cache state. */
  [[nodiscard]] std::expected<void, shim_error> poll();

  /**
   * Publishes a CAN frame batch using active-prefix serialization.
   *
   * Fails with shutdown_already_observed after a shutdown envelope has arrived,
   * or send_failed when the Tier 1 outbound lane rejects the message.
   */
  [[nodiscard]] std::expected<void, shim_error> send_can_frame_batch(
      const can_frame_batch& batch, std::uint64_t sim_time_us);

  /** Publishes notifier state using its active-prefix serialization. */
  [[nodiscard]] std::expected<void, shim_error> send_notifier_state(
      const notifier_state& state, std::uint64_t sim_time_us);

  /** Publishes a HAL_SendError batch using active-prefix serialization. */
  [[nodiscard]] std::expected<void, shim_error> send_error_message_batch(
      const error_message_batch& batch, std::uint64_t sim_time_us);

  /** Enqueues one outbound HAL_SendError message, dropping it if the batch is full. */
  void enqueue_error(const error_message& msg) noexcept;

  /** Enqueues one outbound CAN TX frame, dropping it if the batch is full. */
  void enqueue_can_frame(const can_frame& frame) noexcept;

  /** Publishes and clears pending HAL_SendError messages, if any. */
  [[nodiscard]] std::expected<void, shim_error> flush_pending_errors(
      std::uint64_t sim_time_us);

  /** Publishes and clears pending CAN TX frames, if any. */
  [[nodiscard]] std::expected<void, shim_error> flush_pending_can_frames(
      std::uint64_t sim_time_us);

  /** Opens a filtered CAN RX stream session. Returns 0 if no slot is available. */
  [[nodiscard]] std::uint32_t open_can_stream_session(
      std::uint32_t message_id,
      std::uint32_t message_id_mask,
      std::uint32_t max_messages);

  /** Closes a CAN RX stream session handle if it is currently open. */
  void close_can_stream_session(std::uint32_t handle) noexcept;

  /** Reads and drains queued CAN RX stream frames into `messages`. */
  [[nodiscard]] std::int32_t read_can_stream_session(
      std::uint32_t handle,
      std::span<can_frame> messages,
      std::uint32_t& messages_read);

  /** Returns the currently buffered HAL_SendError messages. */
  [[nodiscard]] std::span<const error_message> pending_error_messages() const noexcept;

  /** Returns the currently buffered CAN TX frames. */
  [[nodiscard]] std::span<const can_frame> pending_can_frames() const noexcept;

  /** True after boot_ack has been accepted. */
  [[nodiscard]] bool is_connected() const;
  /** True after shutdown has been accepted. */
  [[nodiscard]] bool is_shutting_down() const;
  /** Latest cached clock_state, if one has been received. */
  [[nodiscard]] const std::optional<clock_state>& latest_clock_state() const;
  /** Latest cached power_state, if one has been received. */
  [[nodiscard]] const std::optional<power_state>& latest_power_state() const;
  /** Latest cached ds_state, if one has been received. */
  [[nodiscard]] const std::optional<ds_state>& latest_ds_state() const;
  /** Latest cached CAN frame batch, if one has been received. */
  [[nodiscard]] const std::optional<can_frame_batch>& latest_can_frame_batch() const;
  /** Latest cached CAN status, if one has been received. */
  [[nodiscard]] const std::optional<can_status>& latest_can_status() const;
  /** Latest cached notifier state, if one has been received. */
  [[nodiscard]] const std::optional<notifier_state>& latest_notifier_state() const;
  /** Latest cached notifier alarm batch, if one has been received. */
  [[nodiscard]] const std::optional<notifier_alarm_batch>& latest_notifier_alarm_batch() const;
  /** Latest cached error message batch, if one has been received. */
  [[nodiscard]] const std::optional<error_message_batch>& latest_error_message_batch() const;

  shim_core(const shim_core&) = delete;
  shim_core& operator=(const shim_core&) = delete;
  shim_core(shim_core&&) = default;
  shim_core& operator=(shim_core&&) = default;

 private:
  explicit shim_core(tier1::tier1_endpoint endpoint);

  void enqueue_can_rx_frame_for_streams(const can_frame& frame);

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
  std::array<error_message, kMaxErrorsPerBatch> pending_error_messages_{};
  std::uint32_t pending_error_count_ = 0;
  std::array<can_frame, kMaxCanFramesPerBatch> pending_can_frames_{};
  std::uint32_t pending_can_frame_count_ = 0;

  struct can_stream_session {
    bool active = false;
    std::uint32_t handle = 0;
    std::uint32_t message_id = 0;
    std::uint32_t message_id_mask = 0;
    std::uint32_t max_messages = 0;
    bool overrun = false;
    std::vector<can_frame> queued_frames;
  };

  static constexpr std::uint32_t kMaxCanStreamSessions = 32;
  std::array<can_stream_session, kMaxCanStreamSessions> can_stream_sessions_{};
  std::uint32_t next_can_stream_handle_ = 1;
};

}  // namespace robosim::backend::shim
