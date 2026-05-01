#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace robosim::backend {

/**
 * Copies a string into a fixed-size HAL buffer and always null-terminates it.
 *
 * @return true when truncation occurred. Bytes after the terminator are left
 * unchanged; callers should zero-initialize the destination first when the
 * tail participates in equality or wire serialization.
 *
 * Behavior is undefined if `dst` and `src` overlap, matching std::memcpy.
 */
[[nodiscard]] bool copy_truncated(std::span<char> dst, std::string_view src);

/**
 * Copies raw bytes into a fixed-size HAL buffer without adding a terminator.
 *
 * Intended for fields such as HAL_MatchInfo::gameSpecificMessage that carry a
 * separate length. Returns the number of bytes written.
 */
[[nodiscard]] std::size_t copy_bytes_truncated(
    std::span<std::uint8_t> dst, std::span<const std::uint8_t> src);

}  // namespace robosim::backend
