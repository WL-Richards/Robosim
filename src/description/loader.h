#pragma once

#include "error.h"
#include "schema.h"

#include <expected>
#include <filesystem>

namespace robosim::description {

[[nodiscard]] std::expected<robot_description, load_error> load_from_file(
    const std::filesystem::path& path);

}  // namespace robosim::description
