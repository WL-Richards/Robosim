#pragma once

#include "protocol_version.h"
#include "sync_envelope.h"
#include "validator_error.h"

#include <array>
#include <cstdint>
#include <expected>
#include <span>

namespace robosim::backend {

// Closed envelope_kind → allowed-payload_schema mapping. Exposed so
// tests and the future transport-layer session can both read the same
// table. Adding a kind/schema combination is a kProtocolVersion bump
// (decision #15).
//
// Each row of this table corresponds to one envelope_kind; the bool[N]
// indexed by schema_id underlying value indicates allowance.
struct kind_schema_allowance {
  envelope_kind kind;
  std::array<bool, 10> allowed_by_schema_id;  // index = schema_id value
};

inline constexpr std::array<kind_schema_allowance, 6> kPerKindAllowedSchemas{{
    // boot → {boot_descriptor}
    {envelope_kind::boot,
     {false, false, false, false, false, false, false, false, false, true}},
    // boot_ack → {none} (with payload_bytes == 0)
    {envelope_kind::boot_ack,
     {true, false, false, false, false, false, false, false, false, false}},
    // tick_boundary → all per-tick-published schemas (closed; no
    //                 boot_descriptor, no none)
    {envelope_kind::tick_boundary,
     {false, true, true, true, true, true, true, true, true, false}},
    // on_demand_request → request set; error_message_batch excluded
    //                     (errors are pushed, not requested)
    {envelope_kind::on_demand_request,
     {false, true, true, true, true, true, true, true, false, false}},
    // on_demand_reply → reply set including error_message_batch (a
    //                   reply may carry a synchronous error result)
    {envelope_kind::on_demand_reply,
     {false, true, true, true, true, true, true, true, true, false}},
    // shutdown → {none}
    {envelope_kind::shutdown,
     {true, false, false, false, false, false, false, false, false, false}},
}};

// Pure stateless envelope validator. Caller provides the expected
// sender direction and the expected sequence number for that direction.
// Sequence-counter ownership lives in the future transport-cycle
// protocol_session (decision #19 / fork F4).
//
// Fixed-size schemas have known payload_bytes; the validator looks
// these up in a table built from sizeof(<struct>) at compile time.
//
// For variable-size batch schemas (can_frame_batch,
// notifier_alarm_batch, error_message_batch, notifier_state), the
// validator decodes the batch HEADER (the count field) from the
// payload prefix and checks payload_bytes consistency. It does NOT
// decode any element body — that remains the receiver's job once
// framing is validated. Crossing this line would silently take on
// transport-cycle scope (TEST_PLAN D17 boundary comment).
[[nodiscard]] std::expected<void, validate_error>
validate_envelope(const sync_envelope& env,
                  direction expected_sender,
                  std::uint64_t expected_sequence,
                  std::span<const std::uint8_t> payload = {});

}  // namespace robosim::backend
