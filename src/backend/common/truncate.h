#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace robosim::backend {

// Copy src into dst, null-terminating at min(src.size(), dst.size() - 1).
// Returns true iff truncation occurred. Bytes in dst past the null
// terminator are not modified — caller is responsible for zero-init if a
// clean tail is required.
//
// Behavior is undefined if dst and src overlap (matches std::memcpy
// semantics; we use memcpy internally). If overlap-safe semantics are
// ever needed, switch to std::memmove and pin that explicitly — do not
// make it a per-call decision.
[[nodiscard]] bool copy_truncated(std::span<char> dst, std::string_view src);

// Copy src into dst (no null terminator — for byte buffers like
// HAL_MatchInfo::gameSpecificMessage that carry an explicit length
// field). Returns the number of bytes actually written.
[[nodiscard]] std::size_t copy_bytes_truncated(
    std::span<std::uint8_t> dst, std::span<const std::uint8_t> src);

}  // namespace robosim::backend
