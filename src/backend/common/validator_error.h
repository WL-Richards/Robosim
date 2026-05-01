#pragma once

#include <string>
#include <type_traits>

namespace robosim::backend {

/** Stateless protocol validation failure categories. */
enum class validate_error_kind {
  magic_mismatch,
  version_mismatch,
  unknown_schema_id,
  unknown_envelope_kind,
  unknown_direction,
  schema_payload_kind_mismatch,
  payload_size_mismatch,
  sequence_mismatch,
  direction_mismatch,
  unsupported_runtime,
};

/** Detailed validation failure with a stable field name for test assertions. */
struct validate_error {
  validate_error_kind kind;
  std::string offending_field_name;
  std::string message;

  bool operator==(const validate_error&) const = default;
};

}  // namespace robosim::backend
