#pragma once

#include <cstdint>
#include <type_traits>

namespace robosim::backend {

/**
 * Mirrors WPILib HAL_Bool from hal/include/hal/Types.h byte-for-byte and
 * signedness-for-signedness.
 *
 * WPILib defines this as `typedef int32_t HAL_Bool`. Cycle 15 corrected a
 * previously unsigned alias; valid 0/1 wire values are unchanged, while the
 * C++ type contract now matches WPILib exactly.
 */
using hal_bool = std::int32_t;

/** Mirrors WPILib HAL_Handle. */
using hal_handle = std::int32_t;

static_assert(sizeof(hal_bool) == 4);
static_assert(std::is_same_v<hal_bool, std::int32_t>);
static_assert(std::is_same_v<hal_handle, std::int32_t>);

}  // namespace robosim::backend
