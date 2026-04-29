#include "truncate.h"

#include <algorithm>
#include <cstring>

namespace robosim::backend {

bool copy_truncated(std::span<char> dst, std::string_view src) {
  if (dst.empty()) {
    // No room even for a null terminator. Truncation iff src had bytes.
    return !src.empty();
  }
  const std::size_t capacity = dst.size() - 1;
  const std::size_t copied = std::min(src.size(), capacity);
  if (copied > 0) {
    std::memcpy(dst.data(), src.data(), copied);
  }
  dst[copied] = '\0';
  return src.size() > capacity;
}

std::size_t copy_bytes_truncated(std::span<std::uint8_t> dst,
                                 std::span<const std::uint8_t> src) {
  const std::size_t copied = std::min(dst.size(), src.size());
  if (copied > 0) {
    std::memcpy(dst.data(), src.data(), copied);
  }
  return copied;
}

}  // namespace robosim::backend
