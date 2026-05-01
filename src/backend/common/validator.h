#pragma once

#include "protocol_version.h"
#include "sync_envelope.h"
#include "validator_error.h"

#include <array>
#include <cstdint>
#include <expected>
#include <span>

namespace robosim::backend {

/**
 * One row in the closed envelope_kind -> payload_schema allowance table.
 *
 * The bool array is indexed by the underlying schema_id value. Adding a new
 * allowed kind/schema combination changes the protocol contract and requires a
 * kProtocolVersion bump.
 */
struct kind_schema_allowance {
  envelope_kind kind;
  std::array<bool, 12> allowed_by_schema_id;  // index = schema_id value
};

/** Shared current-protocol kind/schema allowance table used by tests, sessions, and transports. */
inline constexpr std::array<kind_schema_allowance, 6> kPerKindAllowedSchemas{{
    // boot → {boot_descriptor}
    {envelope_kind::boot,
     {false, false, false, false, false, false, false, false, false, true, false, false}},
    // boot_ack → {none} (with payload_bytes == 0)
    {envelope_kind::boot_ack,
     {true, false, false, false, false, false, false, false, false, false, false, false}},
    // tick_boundary → all per-tick-published schemas (closed; no
    //                 boot_descriptor, no none)
    {envelope_kind::tick_boundary,
     {false, true, true, true, true, true, true, true, true, false, true, true}},
    // on_demand_request → request set; error_message_batch excluded
    //                     (errors are pushed, not requested)
    {envelope_kind::on_demand_request,
     {false, true, true, true, true, true, true, true, false, false, false, false}},
    // on_demand_reply → reply set including error_message_batch (a
    //                   reply may carry a synchronous error result)
    {envelope_kind::on_demand_reply,
     {false, true, true, true, true, true, true, true, true, false, false, false}},
    // shutdown → {none}
    {envelope_kind::shutdown,
     {true, false, false, false, false, false, false, false, false, false, false, false}},
}};

/**
 * Validates a sync envelope without mutating any session state.
 *
 * @param env Envelope to check.
 * @param expected_sender Direction required for env.sender.
 * @param expected_sequence Exact sequence number required for env.sequence.
 * @param payload Payload bytes, required for variable-size schema checks.
 *
 * Fixed-size schemas are checked against sizeof(schema struct). Variable-size
 * batches decode only their count header and expected active-prefix length;
 * element bodies remain the receiver's responsibility after framing is valid.
 */
[[nodiscard]] std::expected<void, validate_error>
validate_envelope(const sync_envelope& env,
                  direction expected_sender,
                  std::uint64_t expected_sequence,
                  std::span<const std::uint8_t> payload = {});

}  // namespace robosim::backend
