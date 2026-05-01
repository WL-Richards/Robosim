#pragma once

#include "types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::size_t kErrorDetailsLen   = 1024;
inline constexpr std::size_t kErrorLocationLen  = 256;
inline constexpr std::size_t kErrorCallStackLen = 1024;

/** Truncation flag bit assignments for error_message::truncation_flags. */
inline constexpr std::uint8_t kErrorTruncDetails   = 0b0000'0001;
inline constexpr std::uint8_t kErrorTruncLocation  = 0b0000'0010;
inline constexpr std::uint8_t kErrorTruncCallStack = 0b0000'0100;

/**
 * Wire representation of one HAL_SendError call.
 *
 * severity mirrors HAL_SendError's HAL_Bool isError argument: 1 for error,
 * 0 for warning/info. The trailing pad after truncation_flags is named so the
 * schema has no implicit padding bytes in the serialized prefix.
 */
struct error_message {
  std::int32_t error_code;
  hal_bool severity;
  hal_bool is_lv_code;
  hal_bool print_msg;
  std::uint8_t truncation_flags;
  std::array<std::uint8_t, 3> reserved_pad;
  std::array<char, kErrorDetailsLen> details;
  std::array<char, kErrorLocationLen> location;
  std::array<char, kErrorCallStackLen> call_stack;

  bool operator==(const error_message&) const = default;
};

static_assert(std::is_trivially_copyable_v<error_message>);
static_assert(std::is_standard_layout_v<error_message>);
static_assert(!std::is_polymorphic_v<error_message>);
static_assert(std::is_aggregate_v<error_message>);
static_assert(sizeof(error_message) == 2324);
static_assert(alignof(error_message) == 4);

inline constexpr std::size_t kMaxErrorsPerBatch = 8;

/** Bounded batch of HAL_SendError messages using active-prefix serialization. */
struct error_message_batch {
  std::uint32_t count;
  std::array<std::uint8_t, 4> reserved_pad;  // pad to align 8 if any
                                             // batch element grows
  std::array<error_message, kMaxErrorsPerBatch> messages;

  bool operator==(const error_message_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<error_message_batch>);
static_assert(std::is_standard_layout_v<error_message_batch>);
static_assert(sizeof(error_message_batch) ==
              offsetof(error_message_batch, messages) +
                  kMaxErrorsPerBatch * sizeof(error_message),
              "error_message_batch is 8-byte header + 8 * 2324 = 18600 "
              "bytes; if this fires the layout shifted (named "
              "reserved_pad[4] grew or shrank, or sizeof(error_message) "
              "changed) and validator's offsetof-based active-prefix "
              "math plus shim cycle-8 dispatch arm need re-verifying.");
static_assert(sizeof(error_message_batch) == 18600);
static_assert(offsetof(error_message_batch, messages) == 8,
              "error_message_batch header is 4-byte count + named "
              "reserved_pad[4]; the production active_prefix_bytes "
              "overload below depends on this offset.");

/**
 * Returns the serialized active prefix for an error message batch.
 *
 * The prefix contains the 8-byte header and `count * sizeof(error_message)`;
 * unused batch slots are intentionally omitted.
 */
inline std::span<const std::uint8_t> active_prefix_bytes(
    const error_message_batch& batch) {
  const std::size_t active_size =
      offsetof(error_message_batch, messages) +
      static_cast<std::size_t>(batch.count) * sizeof(error_message);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

}  // namespace robosim::backend
