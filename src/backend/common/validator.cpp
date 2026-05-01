#include "validator.h"

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "joystick_output.h"
#include "notifier_state.h"
#include "power_state.h"
#include "user_program_observer.h"

#include <cstring>
#include <format>

namespace robosim::backend {

namespace {

constexpr std::uint8_t schema_index(schema_id s) {
  return static_cast<std::uint8_t>(s);
}

[[nodiscard]] std::expected<void, validate_error>
fail(validate_error_kind kind, std::string field, std::string message) {
  return std::unexpected(
      validate_error{kind, std::move(field), std::move(message)});
}

// Fixed-size schema → expected payload_bytes. Returns 0 for variable-
// size batch schemas (caller checks those via the batch header).
constexpr std::uint32_t fixed_payload_size(schema_id s) {
  switch (s) {
    case schema_id::none:                 return 0;
    case schema_id::clock_state:          return sizeof(clock_state);
    case schema_id::power_state:          return sizeof(power_state);
    case schema_id::ds_state:             return sizeof(ds_state);
    case schema_id::can_status:           return sizeof(can_status);
    case schema_id::boot_descriptor:      return sizeof(boot_descriptor);
    case schema_id::user_program_observer_snapshot:
      return sizeof(user_program_observer_snapshot);
    // variable-size:
    case schema_id::can_frame_batch:
    case schema_id::notifier_state:
    case schema_id::notifier_alarm_batch:
    case schema_id::error_message_batch:
    case schema_id::joystick_output_batch:
      return 0;
  }
  return 0;
}

constexpr bool is_variable_size(schema_id s) {
  return s == schema_id::can_frame_batch
      || s == schema_id::notifier_state
      || s == schema_id::notifier_alarm_batch
      || s == schema_id::error_message_batch
      || s == schema_id::joystick_output_batch;
}

// For variable-size schemas: read the count from the start of the
// payload, then compute expected payload_bytes from it.
//
// Layout assumption (matches the in-tree batch struct definitions): the
// first 4 bytes of the payload are a uint32_t count. The validator
// only decodes this header field; element bodies are out of scope.
[[nodiscard]] std::expected<std::uint32_t, validate_error>
expected_variable_payload_size(schema_id s,
                               std::span<const std::uint8_t> payload) {
  if (payload.size() < sizeof(std::uint32_t)) {
    return std::unexpected(validate_error{
        validate_error_kind::payload_size_mismatch,
        "payload_bytes",
        "payload too small to contain batch count header"});
  }
  std::uint32_t count = 0;
  std::memcpy(&count, payload.data(), sizeof(count));

  std::uint32_t header_size = 0;
  std::uint32_t element_size = 0;
  std::uint32_t capacity = 0;
  switch (s) {
    case schema_id::can_frame_batch:
      header_size = static_cast<std::uint32_t>(
          offsetof(can_frame_batch, frames));
      element_size = sizeof(can_frame);
      capacity = static_cast<std::uint32_t>(kMaxCanFramesPerBatch);
      break;
    case schema_id::notifier_state:
      header_size = static_cast<std::uint32_t>(
          offsetof(notifier_state, slots));
      element_size = sizeof(notifier_slot);
      capacity = static_cast<std::uint32_t>(kMaxNotifiers);
      break;
    case schema_id::notifier_alarm_batch:
      header_size = static_cast<std::uint32_t>(
          offsetof(notifier_alarm_batch, events));
      element_size = sizeof(notifier_alarm_event);
      capacity = static_cast<std::uint32_t>(kMaxAlarmsPerBatch);
      break;
    case schema_id::error_message_batch:
      header_size = static_cast<std::uint32_t>(
          offsetof(error_message_batch, messages));
      element_size = sizeof(error_message);
      capacity = static_cast<std::uint32_t>(kMaxErrorsPerBatch);
      break;
    case schema_id::joystick_output_batch:
      header_size = static_cast<std::uint32_t>(
          offsetof(joystick_output_batch, outputs));
      element_size = sizeof(joystick_output_state);
      capacity = static_cast<std::uint32_t>(kMaxJoysticks);
      break;
    default:
      return 0u;  // not variable
  }
  if (count > capacity) {
    return std::unexpected(validate_error{
        validate_error_kind::payload_size_mismatch,
        "count",
        std::format("batch count {} exceeds capacity {}", count, capacity)});
  }
  return header_size + count * element_size;
}

[[nodiscard]] bool kind_allows_schema(envelope_kind k, schema_id s) {
  for (const auto& row : kPerKindAllowedSchemas) {
    if (row.kind == k) {
      const std::size_t idx = schema_index(s);
      if (idx >= row.allowed_by_schema_id.size()) return false;
      return row.allowed_by_schema_id[idx];
    }
  }
  return false;
}

}  // namespace

std::expected<void, validate_error>
validate_envelope(const sync_envelope& env,
                  direction expected_sender,
                  std::uint64_t expected_sequence,
                  std::span<const std::uint8_t> payload) {
  // Magic.
  if (env.magic != kProtocolMagic) {
    return fail(validate_error_kind::magic_mismatch, "magic",
                "envelope magic does not match kProtocolMagic");
  }

  // Protocol version (zero rejected as uninit guard, then mismatch).
  if (env.protocol_version == 0) {
    return fail(validate_error_kind::version_mismatch, "protocol_version",
                "protocol_version is zero (uninitialized envelope)");
  }
  if (env.protocol_version != kProtocolVersion) {
    return fail(validate_error_kind::version_mismatch, "protocol_version",
                std::format("protocol_version {} does not match "
                            "kProtocolVersion {}",
                            env.protocol_version, kProtocolVersion));
  }

  // Envelope kind closed enum (zero rejected; > 6 rejected).
  const auto kind_value = static_cast<std::uint16_t>(env.kind);
  if (env.kind == envelope_kind::reserved
      || kind_value > static_cast<std::uint16_t>(envelope_kind::shutdown)) {
    return fail(validate_error_kind::unknown_envelope_kind, "kind",
                std::format("envelope_kind {} is not in the closed set",
                            kind_value));
  }

  // Direction closed enum (zero rejected; > 2 rejected).
  const auto sender_value = static_cast<std::uint8_t>(env.sender);
  if (env.sender == direction::reserved
      || sender_value > static_cast<std::uint8_t>(direction::core_to_backend)) {
    return fail(validate_error_kind::unknown_direction, "sender",
                std::format("direction {} is not in the closed set",
                            sender_value));
  }

  // Direction matches expectation.
  if (env.sender != expected_sender) {
    return fail(validate_error_kind::direction_mismatch, "sender",
                "envelope sender does not match expected_sender");
  }

  // Schema id closed enum.
  const auto schema_value = static_cast<std::uint8_t>(env.payload_schema);
  if (schema_value > kSchemaIdMaxValid) {
    return fail(validate_error_kind::unknown_schema_id, "payload_schema",
                std::format("schema_id {} is not in the closed set",
                            schema_value));
  }

  // Sequence equality (no direction inferred from delta — the validator
  // is stateless; the future transport session owns the counter).
  if (env.sequence != expected_sequence) {
    return fail(validate_error_kind::sequence_mismatch, "sequence",
                std::format("sequence {} does not match expected {}",
                            env.sequence, expected_sequence));
  }

  // kind ↔ schema mapping.
  if (!kind_allows_schema(env.kind, env.payload_schema)) {
    return fail(validate_error_kind::schema_payload_kind_mismatch,
                "payload_schema",
                std::format("payload_schema {} not allowed for "
                            "envelope_kind {}",
                            schema_value, kind_value));
  }

  // payload_bytes size check.
  if (env.payload_schema == schema_id::none) {
    if (env.payload_bytes != 0) {
      return fail(validate_error_kind::schema_payload_kind_mismatch,
                  "payload_bytes",
                  "payload_schema is none but payload_bytes != 0");
    }
  } else if (is_variable_size(env.payload_schema)) {
    auto expected = expected_variable_payload_size(env.payload_schema,
                                                   payload);
    if (!expected.has_value()) return std::unexpected(expected.error());
    if (env.payload_bytes != *expected) {
      return fail(validate_error_kind::payload_size_mismatch,
                  "payload_bytes",
                  std::format("payload_bytes {} does not match "
                              "header-derived expected {}",
                              env.payload_bytes, *expected));
    }
  } else {
    const auto expected = fixed_payload_size(env.payload_schema);
    if (env.payload_bytes != expected) {
      return fail(validate_error_kind::payload_size_mismatch,
                  "payload_bytes",
                  std::format("payload_bytes {} does not match "
                              "fixed-schema expected {}",
                              env.payload_bytes, expected));
    }
  }

  return {};
}

}  // namespace robosim::backend
