#pragma once

#include <cstdint>
#include <type_traits>

namespace robosim::backend {

// Mirrors WPILib HAL_Bool from hal/include/hal/Types.h. Width must match
// for the shim's memcpy contract (decision #10) to hold.
using hal_bool = std::uint32_t;

// Mirrors WPILib HAL_Handle.
using hal_handle = std::int32_t;

static_assert(sizeof(hal_bool) == 4);
static_assert(std::is_same_v<hal_bool, std::uint32_t>);
static_assert(std::is_same_v<hal_handle, std::int32_t>);

}  // namespace robosim::backend
