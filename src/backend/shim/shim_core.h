#pragma once

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "joystick_output.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_session.h"
#include "shared_memory_transport.h"
#include "user_program_observer.h"

#include <array>
#include <cstdint>
#include <expected>
#include <mutex>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

extern "C" {
/** Mirrors WPILib WPI_Handle from wpi/Synchronization.h. */
typedef unsigned int WPI_Handle;

/** Mirrors WPILib WPI_EventHandle from wpi/Synchronization.h. */
typedef WPI_Handle WPI_EventHandle;
}

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

/** Latest user-program observer state reported through the C HAL DS hooks. */
enum class user_program_observer_state {
  none,
  starting,
  disabled,
  autonomous,
  teleop,
  test,
};

/** Latest joystick output command reported through HAL_SetJoystickOutputs. */
struct joystick_output_state {
  std::int64_t outputs;
  std::int32_t left_rumble;
  std::int32_t right_rumble;

  bool operator==(const joystick_output_state&) const = default;
};

/** One HAL_Report usage entry observed from robot code. */
struct usage_report_record {
  std::int32_t resource;
  std::int32_t instance_number;
  std::int32_t context;
  std::string feature;

  bool operator==(const usage_report_record&) const = default;
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
  [[nodiscard]] static std::expected<shim_core, shim_error> make(tier1::tier1_endpoint endpoint,
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
   * Drains at most one inbound message for HAL_RefreshDSData.
   *
   * Returns true only when the accepted message was a Driver Station state
   * packet. Other valid inbound messages are still dispatched but return false.
   */
  [[nodiscard]] bool refresh_ds_data();

  /** Registers a WPI event handle for wakeup when DS data is accepted. */
  void provide_new_data_event_handle(WPI_EventHandle handle);

  /** Removes a previously registered WPI DS new-data event handle. */
  void remove_new_data_event_handle(WPI_EventHandle handle);

  /** Records the latest user-program observer state for host-side v0 use. */
  void observe_user_program(user_program_observer_state state) noexcept;

  /** Records one HAL_Report usage entry and returns its 1-based report index. */
  [[nodiscard]] std::int64_t record_usage_report(std::int32_t resource,
                                                 std::int32_t instance_number,
                                                 std::int32_t context,
                                                 std::string_view feature);

  /** Records the latest joystick outputs for one valid joystick slot. */
  [[nodiscard]] std::int32_t set_joystick_outputs(std::int32_t joystick_num,
                                                  std::int64_t outputs,
                                                  std::int32_t left_rumble,
                                                  std::int32_t right_rumble) noexcept;

  /**
   * Publishes a CAN frame batch using active-prefix serialization.
   *
   * Fails with shutdown_already_observed after a shutdown envelope has arrived,
   * or send_failed when the Tier 1 outbound lane rejects the message.
   */
  [[nodiscard]] std::expected<void, shim_error> send_can_frame_batch(const can_frame_batch& batch,
                                                                     std::uint64_t sim_time_us);

  /** Publishes notifier state using its active-prefix serialization. */
  [[nodiscard]] std::expected<void, shim_error> send_notifier_state(const notifier_state& state,
                                                                    std::uint64_t sim_time_us);

  /** Publishes a HAL_SendError batch using active-prefix serialization. */
  [[nodiscard]] std::expected<void, shim_error> send_error_message_batch(
      const error_message_batch& batch, std::uint64_t sim_time_us);

  /** Publishes joystick output/rumble state using active-prefix serialization. */
  [[nodiscard]] std::expected<void, shim_error> send_joystick_output_batch(
      const joystick_output_batch& batch, std::uint64_t sim_time_us);

  /** Publishes the latest user-program observer mode as one fixed-size snapshot. */
  [[nodiscard]] std::expected<void, shim_error> send_user_program_observer_snapshot(
      const ::robosim::backend::user_program_observer_snapshot& snapshot,
      std::uint64_t sim_time_us);

  /** Enqueues one outbound HAL_SendError message, dropping it if the batch is full. */
  void enqueue_error(const error_message& msg) noexcept;

  /** Enqueues one outbound CAN TX frame, dropping it if the batch is full. */
  void enqueue_can_frame(const can_frame& frame) noexcept;

  /** Publishes and clears pending HAL_SendError messages, if any. */
  [[nodiscard]] std::expected<void, shim_error> flush_pending_errors(std::uint64_t sim_time_us);

  /** Publishes and clears pending CAN TX frames, if any. */
  [[nodiscard]] std::expected<void, shim_error> flush_pending_can_frames(std::uint64_t sim_time_us);

  /** Opens a filtered CAN RX stream session. Returns 0 if no slot is available. */
  [[nodiscard]] std::uint32_t open_can_stream_session(std::uint32_t message_id,
                                                      std::uint32_t message_id_mask,
                                                      std::uint32_t max_messages);

  /** Closes a CAN RX stream session handle if it is currently open. */
  void close_can_stream_session(std::uint32_t handle) noexcept;

  /** Reads and drains queued CAN RX stream frames into `messages`. */
  [[nodiscard]] std::int32_t read_can_stream_session(std::uint32_t handle,
                                                     std::span<can_frame> messages,
                                                     std::uint32_t& messages_read);

  /** Allocates one HAL notifier handle, returning 0 when the table is full. */
  [[nodiscard]] std::int32_t initialize_notifier() noexcept;

  /** Updates the fixed-size name field for an active notifier handle. */
  [[nodiscard]] std::int32_t set_notifier_name(std::int32_t handle, std::string_view name) noexcept;

  /** Activates an alarm for an active notifier handle. */
  [[nodiscard]] std::int32_t update_notifier_alarm(std::int32_t handle,
                                                   std::uint64_t trigger_time_us) noexcept;

  /** Cancels the alarm for an active notifier handle. */
  [[nodiscard]] std::int32_t cancel_notifier_alarm(std::int32_t handle) noexcept;

  /** Stops the notifier's alarm without freeing its handle. */
  [[nodiscard]] std::int32_t stop_notifier(std::int32_t handle) noexcept;

  /** Frees an active notifier handle; invalid handles are ignored. */
  void clean_notifier(std::int32_t handle) noexcept;

  /** Returns a compact active-prefix snapshot of the live notifier table. */
  [[nodiscard]] notifier_state current_notifier_state() const noexcept;

  /** Publishes the current notifier table snapshot, including empty tables. */
  [[nodiscard]] std::expected<void, shim_error> flush_notifier_state(std::uint64_t sim_time_us);

  /** Returns a compact active-prefix snapshot of written joystick output slots. */
  [[nodiscard]] joystick_output_batch current_joystick_output_batch() const noexcept;

  /** Publishes the current joystick output snapshot, including empty snapshots. */
  [[nodiscard]] std::expected<void, shim_error> flush_joystick_outputs(
      std::uint64_t sim_time_us);

  /** Returns the latest observer state as a host-visible fixed-size snapshot. */
  [[nodiscard]] ::robosim::backend::user_program_observer_snapshot
  current_user_program_observer_snapshot() const noexcept;

  /** Publishes the current observer snapshot, including the default none mode. */
  [[nodiscard]] std::expected<void, shim_error> flush_user_program_observer(
      std::uint64_t sim_time_us);

  /**
   * Publishes the next Driver Station output-side snapshot.
   *
   * The pump emits at most one message per call, alternating
   * user_program_observer_snapshot then joystick_output_batch. It returns the
   * schema that was published and advances only after a successful send.
   */
  [[nodiscard]] std::expected<schema_id, shim_error> flush_next_driver_station_output(
      std::uint64_t sim_time_us);

  /** Waits for a fired alarm, stop wake, or clean wake for an active notifier. */
  [[nodiscard]] std::uint64_t wait_for_notifier_alarm(std::int32_t handle, std::int32_t& status);

  /** Clears shutdown wake state when the host reinitializes this installed shim. */
  void prepare_for_hal_initialize();

  /** Wakes pending HAL waits during process HAL shutdown. */
  void shutdown_hal_waits();

  /** Returns the total number of registered pending notifier waits. */
  [[nodiscard]] std::uint32_t pending_notifier_wait_count() const noexcept;

  /** Returns the pending notifier wait count for one handle. */
  [[nodiscard]] std::uint32_t pending_notifier_wait_count(std::int32_t handle) const noexcept;

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
  /** Thread-safe latest cached clock_state snapshot, if one has been received. */
  [[nodiscard]] std::optional<clock_state> latest_clock_state_snapshot() const;
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
  /** Latest user-program observer state reported by robot code. */
  [[nodiscard]] user_program_observer_state user_program_observer_state() const noexcept;
  /** Usage reports recorded through HAL_Report, in call order. */
  [[nodiscard]] std::span<const usage_report_record> usage_reports() const noexcept;
  /** Latest joystick output state for a valid slot, if one has been written. */
  [[nodiscard]] std::optional<joystick_output_state> joystick_outputs(
      std::int32_t joystick_num) const noexcept;
  /** Boot descriptor retained from successful construction. */
  [[nodiscard]] const boot_descriptor& boot_descriptor_snapshot() const noexcept;

  shim_core(const shim_core&) = delete;
  shim_core& operator=(const shim_core&) = delete;
  shim_core(shim_core&&) = default;
  shim_core& operator=(shim_core&&) = default;

 private:
  enum class inbound_dispatch_result {
    no_message,
    boot_ack,
    clock_state,
    power_state,
    ds_state,
    can_frame_batch,
    can_status,
    notifier_state,
    notifier_alarm_batch,
    error_message_batch,
    shutdown,
  };

  enum class driver_station_output_flush_phase {
    user_program_observer,
    joystick_outputs,
  };

  explicit shim_core(tier1::tier1_endpoint endpoint, const boot_descriptor& desc);

  [[nodiscard]] std::expected<inbound_dispatch_result, shim_error> poll_one();
  void wake_new_data_event_handles() const;
  void enqueue_can_rx_frame_for_streams(const can_frame& frame);
  [[nodiscard]] std::int32_t find_notifier_slot(std::int32_t handle) const noexcept;

  tier1::tier1_endpoint endpoint_;
  boot_descriptor boot_descriptor_{};
  bool connected_ = false;
  bool shutdown_observed_ = false;
  mutable std::shared_ptr<std::mutex> cache_mutex_ = std::make_shared<std::mutex>();
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
  enum user_program_observer_state user_program_observer_state_ =
      static_cast<enum user_program_observer_state>(0);
  std::vector<usage_report_record> usage_reports_;
  std::array<std::optional<joystick_output_state>, kMaxJoysticks> joystick_outputs_{};
  driver_station_output_flush_phase driver_station_output_flush_phase_ =
      driver_station_output_flush_phase::user_program_observer;

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
  std::vector<WPI_EventHandle> new_data_event_handles_;

  struct notifier_record {
    bool active = false;
    std::uint32_t allocation_order = 0;
    bool stopped = false;
    std::uint64_t stop_wake_count = 0;
    notifier_slot slot{};
  };

  struct notifier_wait_state;

  std::array<notifier_record, kMaxNotifiers> notifier_records_{};
  std::int32_t next_notifier_handle_ = 1;
  std::uint32_t next_notifier_allocation_order_ = 1;
  std::shared_ptr<notifier_wait_state> notifier_wait_state_;
};

}  // namespace robosim::backend::shim
