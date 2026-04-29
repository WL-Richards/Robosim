#pragma once

#include "types.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace robosim::backend {

inline constexpr std::size_t kErrorDetailsLen   = 1024;
inline constexpr std::size_t kErrorLocationLen  = 256;
inline constexpr std::size_t kErrorCallStackLen = 1024;

// Truncation flag bit assignments — pinned per L10. Mismatches are
// silent corruption.
inline constexpr std::uint8_t kErrorTruncDetails   = 0b0000'0001;
inline constexpr std::uint8_t kErrorTruncLocation  = 0b0000'0010;
inline constexpr std::uint8_t kErrorTruncCallStack = 0b0000'0100;

// Mirrors HAL_SendError parameters. severity is hal_bool (decision #22):
// 1 = error, 0 = warning/info, matching HAL_SendError's HAL_Bool isError.
//
// Field order chosen for zero interior pad: 4-byte primitives first,
// then 1-byte truncation_flags + 3 trailing pad to align 4, then
// fixed-size string buffers (alignof 1).
//   error_code        at 0    (4 bytes)
//   severity          at 4    (4 bytes)
//   is_lv_code        at 8    (4 bytes)
//   print_msg         at 12   (4 bytes)
//   truncation_flags  at 16   (1 byte; trailing pad 17..19 to align 4)
//   details           at 20   (1024 bytes)
//   location          at 1044 (256 bytes)
//   call_stack        at 1300 (1024 bytes)
// sizeof = 2324, alignof = 4. Trailing 3 bytes of named-field-pad
// after truncation_flags are explicit.
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

struct error_message_batch {
  std::uint32_t count;
  std::array<std::uint8_t, 4> reserved_pad;  // pad to align 8 if any
                                             // batch element grows
  std::array<error_message, kMaxErrorsPerBatch> messages;

  bool operator==(const error_message_batch&) const = default;
};

static_assert(std::is_trivially_copyable_v<error_message_batch>);
static_assert(std::is_standard_layout_v<error_message_batch>);

}  // namespace robosim::backend
