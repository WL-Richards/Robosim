#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::uint32_t kCanFlagFrameRemote = 0x80000000u;
inline constexpr std::uint32_t kCanFlagFrame11Bit  = 0x40000000u;

// Mirrors WPILib HAL_CANStreamMessage byte-for-byte (decision #11 /
// fork F3) so the future shim's HAL_CAN_ReadStreamSession path is a
// pointer cast.
//   message_id  at 0  (4 bytes; HAL flag bits in top 3 bits)
//   timestamp_us at 4 (4 bytes; matches HAL_CANStreamMessage::timeStamp)
//   data        at 8  (8 bytes)
//   data_size   at 16 (1 byte)
//   trailing pad at 17..19 (3 bytes; alignof 4)
struct can_frame {
  std::uint32_t message_id;
  std::uint32_t timestamp_us;
  std::array<std::uint8_t, 8> data;
  std::uint8_t data_size;

  bool operator==(const can_frame&) const = default;
};

static_assert(std::is_trivially_copyable_v<can_frame>);
static_assert(std::is_standard_layout_v<can_frame>);
static_assert(!std::is_polymorphic_v<can_frame>);
static_assert(std::is_aggregate_v<can_frame>);
static_assert(sizeof(can_frame) == 20);
static_assert(alignof(can_frame) == 4);

inline constexpr std::size_t kMaxCanFramesPerBatch = 64;

struct can_frame_batch {
  std::uint32_t count;
  std::array<can_frame, kMaxCanFramesPerBatch> frames;

  bool operator==(const can_frame_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<can_frame_batch>);
static_assert(std::is_standard_layout_v<can_frame_batch>);

// Active-prefix wire bytes: count word + count*element bytes (NOT
// sizeof(can_frame_batch)). The shim's outbound send path consumes
// this; the validator's expected_variable_payload_size mirrors the
// same arithmetic. Inline here so callers including this header get
// it without an extra include.
inline std::span<const std::uint8_t> active_prefix_bytes(
    const can_frame_batch& batch) {
  const std::size_t active_size =
      offsetof(can_frame_batch, frames) +
      static_cast<std::size_t>(batch.count) * sizeof(can_frame);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

}  // namespace robosim::backend
