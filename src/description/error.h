#pragma once

#include <filesystem>
#include <string>

namespace robosim::description {

enum class load_error_kind {
  file_not_found,
  parse_error,
  schema_error,
  domain_error,
  reference_error,
  conflict_error,
};

struct load_error {
  load_error_kind kind;
  std::filesystem::path file_path;
  std::string json_pointer;
  std::string message;

  bool operator==(const load_error&) const = default;
};

}  // namespace robosim::description
