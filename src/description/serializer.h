#pragma once

// Robot description serializer.
// See tests/description/TEST_PLAN_VC.md "VC4 — serializer test plan".

#include "schema.h"

#include <expected>
#include <filesystem>
#include <string>

namespace robosim::description {

enum class save_error_kind {
  io_error,  // open / write / fsync / rename failed
};

struct save_error {
  save_error_kind kind;
  std::filesystem::path file_path;
  std::string message;

  bool operator==(const save_error&) const = default;
};

// Write `d` to `path` as canonical JSON per D-VC-6/8/9/10:
// - byte-stable for the same input,
// - keys in schema declaration order (nlohmann::ordered_json),
// - identity origins omitted from output,
// - float formatting via nlohmann::ordered_json::dump(2)
//   (nlohmann SHA pinned in CMakeLists.txt; guarded by S8).
[[nodiscard]] std::expected<void, save_error> save_to_file(
    const robot_description& d, const std::filesystem::path& path);

}  // namespace robosim::description
