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
#include "sync_envelope.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace robosim::backend::tier1 {

enum class tier1_lane_state : std::uint32_t {
  empty = 0,
  writing = 1,
  full = 2,
  reading = 3,
};

enum class tier1_transport_error_kind {
  invalid_endpoint_direction,
  payload_too_large,
  lane_busy,
  no_message,
  lane_in_progress,
  session_rejected_envelope,
};

enum class tier1_mapping_error_kind {
  memfd_create_failed,
  ftruncate_failed,
  mmap_failed,
  dup_failed,
  invalid_fd,
  wrong_size,
};

struct tier1_transport_error {
  tier1_transport_error_kind kind;
  std::optional<backend::session_error> session_failure;
  std::string offending_field_name;
  std::string message;

  bool operator==(const tier1_transport_error&) const = default;
};

struct tier1_mapping_error {
  tier1_mapping_error_kind kind;
  int errno_value;
  std::string offending_field_name;
  std::string message;

  bool operator==(const tier1_mapping_error&) const = default;
};

inline constexpr std::size_t kTier1MaxPayloadBytes = std::max({
    sizeof(clock_state),
    sizeof(power_state),
    sizeof(ds_state),
    sizeof(can_frame_batch),
    sizeof(can_status),
    sizeof(notifier_state),
    sizeof(notifier_alarm_batch),
    sizeof(error_message_batch),
    sizeof(boot_descriptor),
});

struct tier1_lane {
  std::atomic<std::uint32_t> state{static_cast<std::uint32_t>(tier1_lane_state::empty)};
  sync_envelope envelope{};
  std::uint32_t payload_bytes = 0;
  std::array<std::uint8_t, kTier1MaxPayloadBytes> payload{};
};

struct tier1_shared_region {
  tier1_lane backend_to_core{};
  tier1_lane core_to_backend{};
};

struct tier1_message {
  sync_envelope envelope;
  std::vector<std::uint8_t> payload;

  bool operator==(const tier1_message&) const = default;
};

class unique_fd {
 public:
  unique_fd() = default;
  explicit unique_fd(int fd);
  ~unique_fd();

  unique_fd(const unique_fd&) = delete;
  unique_fd& operator=(const unique_fd&) = delete;
  unique_fd(unique_fd&& other) noexcept;
  unique_fd& operator=(unique_fd&& other) noexcept;

  [[nodiscard]] int get() const;
  [[nodiscard]] bool valid() const;
  [[nodiscard]] int release();

 private:
  void reset(int fd = -1) noexcept;

  int fd_ = -1;
};

class tier1_shared_mapping {
 public:
  tier1_shared_mapping() = default;
  ~tier1_shared_mapping();

  tier1_shared_mapping(const tier1_shared_mapping&) = delete;
  tier1_shared_mapping& operator=(const tier1_shared_mapping&) = delete;
  tier1_shared_mapping(tier1_shared_mapping&& other) noexcept;
  tier1_shared_mapping& operator=(tier1_shared_mapping&& other) noexcept;

  [[nodiscard]] static std::expected<tier1_shared_mapping, tier1_mapping_error> create();

  [[nodiscard]] static std::expected<tier1_shared_mapping, tier1_mapping_error> map_existing(
      const unique_fd& fd);

  [[nodiscard]] std::expected<unique_fd, tier1_mapping_error> duplicate_fd() const;

  [[nodiscard]] tier1_shared_region& region();
  [[nodiscard]] const tier1_shared_region& region() const;
  [[nodiscard]] std::size_t size_bytes() const;
  [[nodiscard]] int fd() const;
  [[nodiscard]] bool valid() const;

 private:
  tier1_shared_mapping(unique_fd fd, tier1_shared_region* region, std::size_t size_bytes);

  void reset() noexcept;

  unique_fd fd_;
  tier1_shared_region* region_ = nullptr;
  std::size_t size_bytes_ = 0;
};

class tier1_endpoint {
 public:
  [[nodiscard]] static std::expected<tier1_endpoint, tier1_transport_error> make(
      tier1_shared_region& region, direction local_direction);

  [[nodiscard]] std::expected<void, tier1_transport_error> send(
      envelope_kind kind,
      schema_id payload_schema,
      std::span<const std::uint8_t> payload,
      std::uint64_t sim_time_us);

  [[nodiscard]] std::expected<tier1_message, tier1_transport_error> try_receive();

 private:
  tier1_endpoint(tier1_shared_region& region, protocol_session session);

  [[nodiscard]] tier1_lane& outbound_lane();
  [[nodiscard]] tier1_lane& inbound_lane();

  tier1_shared_region* region_;
  protocol_session session_;
};

}  // namespace robosim::backend::tier1
