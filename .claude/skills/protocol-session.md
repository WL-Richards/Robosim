---
name: protocol-session
description: Use when working on the stateful HAL <-> Sim Core protocol session wrapper that consumes the protocol schema validator, owns per-direction sequence counters, boot/boot_ack ordering, shutdown receive terminal state, and single-flight on-demand request/reply pairing. Does not cover transport bytes, shared memory, sockets, reconnect, HAL shim APIs, or LD_PRELOAD time.
---

# HAL <-> Sim Core protocol session

`protocol_session` is the Layer 2 stateful wrapper around the
schema-cycle's pure `validate_envelope` function. It is the first
consumer of the protocol schema, but it is still transport-agnostic:
no shared-memory atomics, socket framing, reconnect logic, HAL cache
mutation, or time interception live here.

## Scope

**In:**
- Endpoint direction setup: one session represents one endpoint, with a
  local direction and the opposite remote direction.
- Send-side sequence ownership. Successful outbound envelope builds
  consume `next_sequence_to_send`; failed builds do not.
- Receive-side sequence ownership. Accepted inbound envelopes consume
  `next_expected_receive_sequence`; validator or session-order failures
  do not.
- Session-order checks layered after stateless validation.
- Boot handshake ordering:
  - `boot` is backend -> core only.
  - `boot_ack` is core -> backend only.
  - Backend must send `boot` before normal outbound traffic.
  - Backend must receive `boot_ack` before normal inbound core traffic.
  - Core must receive `boot` before normal inbound backend traffic.
- Single-flight on-demand request/reply pairing.
- Terminal receive state after inbound `shutdown`.

**Out:**
- T1 shared-memory transport and atomics/seqlock.
- T2 socket framing, reconnect, and retry policy.
- HAL shim packing/unpacking and local cache mutation.
- Payload-body validation beyond what `validate_envelope` already
  checks.
- LD_PRELOAD time and sleep interception.
- Reconnect semantics. Duplicate boot/ack is rejected in this cycle.

## Public Surface

`src/backend/common/protocol_session.h`:

- `protocol_session::make(direction local_direction)`
- `local_direction()`, `remote_direction()`
- `next_sequence_to_send()`, `next_expected_receive_sequence()`
- `has_received_boot()`, `has_sent_boot()`
- `has_received_boot_ack()`, `has_sent_boot_ack()`
- `has_pending_on_demand_reply()`
- `has_received_shutdown()`
- `build_envelope(kind, schema, payload_bytes, sim_time_us)`
- `accept_envelope(env, payload)`

Errors use `session_error_kind` plus an optional embedded
`validate_error` when the stateless validator rejects the envelope.
Tests pin the typed error kind and offending field substring, not exact
message prose.

## Design Decisions

1. **Validator first, session order second.** Both build and accept
   paths run `validate_envelope` before mutating session state or
   applying handshake rules. This preserves the schema-cycle boundary
   and keeps framing errors distinct from state-machine errors.
2. **One endpoint per session.** A backend-local session sends
   `backend_to_core` and receives `core_to_backend`; a core-local
   session does the reverse.
3. **Handshake is direction-specific.** `boot` carries the backend's
   boot descriptor to the core. `boot_ack` is the core's compatibility
   acknowledgement to the backend.
4. **Rejected envelopes do not advance counters.** This is what lets a
   peer retry a corrected envelope at the same sequence.
5. **On-demand requests are single-flight in v0.** A second outbound
   `on_demand_request` is rejected while a reply is pending. Multiple
   outstanding request IDs belong to a later protocol version if needed.
6. **Sequence overflow is out of scope.** A v0 transport cannot produce
   2^64 frames in practice; adding overflow behavior now would be a
   speculative branch.

## Validation Status

Structural feature; no physical source data applies.

- Test plan was approved by the `test-reviewer` agent on 2026-04-29
  after three rounds.
- `tests/backend/common/protocol_session_test.cpp` adds 16 tests
  covering constructor direction mapping, send/receive sequence
  mutation, boot/boot_ack direction and ordering, validator-error
  mapping, on-demand request/reply pairing, and shutdown terminal
  receive state.
- Targeted verification run: `ctest --test-dir build -R
  ProtocolSession --output-on-failure`.

## Known Limits

- No reconnect or session reset path. Duplicate boot/ack is rejected.
- No body-level validation of `boot_descriptor.runtime`; the current
  schema validator checks envelope kind/schema/size only.
- No transport-level pairing ID for on-demand request/reply; v0 allows
  one outstanding request.

## Cross-references

- `.claude/skills/protocol-schema.md` — envelope, payload structs,
  stateless validator, and truncation helpers consumed by this session.
- `.claude/skills/layer-2-control-system-backend.md` — broader Layer 2
  backend status and remaining TDD cycles.
- `tests/backend/common/protocol_session_test.cpp` — approved session
  contract tests.
