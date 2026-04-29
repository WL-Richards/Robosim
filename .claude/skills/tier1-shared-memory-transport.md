---
name: tier1-shared-memory-transport
description: Use when working on the Tier 1 native shared-memory transport: the fixed two-lane shared region, nonblocking endpoint wrapper, lane-state semantics, payload capacity, protocol_session integration, and Linux memfd/mmap lifecycle wrapper. Does not cover blocking waits, cross-process stress tests, HAL cache mutation, or T2 sockets.
---

# Tier 1 Shared-Memory Transport

Tier 1 is the native backend transport between the in-process HAL shim
and the separate Sim Core process. The current implementation covers
the deterministic shared-region contract, the nonblocking endpoint
wrapper, and a Linux `memfd_create` / `mmap(MAP_SHARED)` lifecycle
wrapper for hosting the region in real OS-backed shared memory.

## Scope

**In:**
- `tier1_shared_region`: two fixed single-slot lanes, one
  backend-to-core and one core-to-backend.
- `tier1_lane`: public shared-memory ABI with atomic lane state,
  `sync_envelope`, payload byte count, and fixed inline payload storage.
- `tier1_endpoint`: nonblocking endpoint wrapper that owns a
  `protocol_session`, publishes outbound envelopes/payloads, and
  consumes inbound envelopes/payloads.
- Single-slot backpressure:
  - `empty` -> `writing` -> `full` for send.
  - `full` -> `reading` -> `empty` for successful receive.
- Failure atomicity: capacity, busy, in-progress, and session failures
  do not consume protocol-session state or clear evidence from the lane.
- Exact payload copy for fixed-size and variable-size schemas.
- `unique_fd`: move-only fd RAII helper.
- `tier1_shared_mapping`: Linux memfd-backed mapping sized exactly to
  `sizeof(tier1_shared_region)`.
- Peer mapping from duplicated fds. `map_existing(const unique_fd&)`
  duplicates the input fd; it never consumes or borrows caller-owned fd
  lifetime.

**Out:**
- POSIX named `shm_open`, permissions, global name discovery, and
  unlink policy.
- Blocking waits, futexes, condition variables, or notifications.
- Cross-process memory-ordering stress. The implementation uses
  acquire loads and release stores for lane state, but unit tests are
  deterministic single-thread tests.
- Multi-slot queues, overwrite/drop policy, or backpressure beyond
  `lane_busy`.
- HAL state cache mutation.
- Tier 2 sockets and reconnect.

## Public Surface

`src/backend/tier1/shared_memory_transport.h`:

- `tier1_lane_state`: `empty`, `writing`, `full`, `reading`.
- `tier1_transport_error_kind`: `invalid_endpoint_direction`,
  `payload_too_large`, `lane_busy`, `no_message`, `lane_in_progress`,
  `session_rejected_envelope`.
- `tier1_mapping_error_kind`: `memfd_create_failed`,
  `ftruncate_failed`, `mmap_failed`, `dup_failed`, `invalid_fd`,
  `wrong_size`.
- `kTier1MaxPayloadBytes`: exact max of current v0 full-capacity
  payload structs.
- `tier1_lane`: shared-region lane ABI.
- `tier1_shared_region`: two lanes.
- `tier1_message`: receiver-owned envelope + payload byte copy.
- `tier1_endpoint::make(region, local_direction)`.
- `tier1_endpoint::send(kind, schema, payload, sim_time_us)`.
- `tier1_endpoint::try_receive()`.
- `unique_fd`: move-only fd wrapper with `get`, `valid`, `release`.
- `tier1_shared_mapping::create()`: create exact-size memfd mapping.
- `tier1_shared_mapping::map_existing(fd)`: validate exact size, dup
  fd, map peer.
- `tier1_shared_mapping::duplicate_fd()`: duplicate mapping fd for a
  peer.

The shared region owns no dynamic memory. `tier1_message` may allocate
because it is an API-boundary copy returned to the caller, not shared
memory.

## Design Decisions

1. **Fixed inline payload capacity.** The capacity equals the largest
   current v0 payload struct, including full-capacity batches. No
   intentional headroom exists today; changing capacity is a reviewed
   ABI change.
2. **Single-slot lanes.** v0 uses one outstanding message per direction.
   A full lane rejects another send with `lane_busy`.
3. **Visible in-progress states.** `writing` and `reading` are public
   lane states. Receivers and senders reject them with
   `lane_in_progress` instead of reading or clobbering partial data.
4. **Lane/capacity checks happen before session mutation.** Failed
   capacity or lane-state checks do not burn protocol sequence numbers.
5. **Session validation happens before publication/clear.** Outbound
   envelopes are built through `protocol_session::build_envelope`.
   Inbound envelopes are accepted through `protocol_session::accept_envelope`.
6. **Failed receive preserves the lane.** If session validation rejects
   a full lane, the lane remains full for higher-level diagnostics or
   reset handling.
7. **Acquire/release lane state.** State observations use acquire loads;
   publishing `full` and clearing `empty` use release stores. Single-
   thread unit tests pin behavior, not the full cross-process memory
   model.
8. **Anonymous memfd lifecycle for v0.** The lifecycle wrapper uses
   Linux `memfd_create`, `ftruncate`, and `mmap(MAP_SHARED)`. This
   avoids global `/dev/shm` names in tests and keeps cleanup tied to fd
   lifetime. Named `shm_open` can wrap this shape later if a launcher
   needs discoverable names.
9. **Move-only ownership.** `unique_fd` and `tier1_shared_mapping` are
   non-copyable and move-only. Move assignment closes/unmaps the
   previous target before taking ownership.
10. **Exact-size peer mapping.** `map_existing` rejects fds whose
    `fstat` size differs from `sizeof(tier1_shared_region)` before
    calling `mmap`.

## Validation Status

Structural transport feature; no physical source data applies.

- Shared-region/endpoint test plan approved by the `test-reviewer`
  agent on 2026-04-29 after two rounds.
- `tests/backend/tier1/shared_memory_transport_test.cpp` adds 15 tests
  covering region initialization/layout, endpoint construction, lane
  direction, capacity boundaries, busy/in-progress states, exact payload
  copy, session-error mapping, receive failure atomicity, and the
  boot/boot_ack/tick round trip.
- OS-backed lifecycle test plan approved by the `test-reviewer` agent
  on 2026-04-29 after three rounds.
- `tests/backend/tier1/shared_memory_mapping_test.cpp` adds 9 tests
  covering memfd sizing, zero initialization, peer mapping visibility,
  endpoint compatibility across two mappings, invalid/wrong-size fd
  rejection, fd duplication, move-only `unique_fd`, move-only
  `tier1_shared_mapping`, and independent peer lifetime.
- Verification run: `ctest --test-dir build --output-on-failure` passed
  229 configured tests after the lifecycle cycle.

## Known Limits

- No blocking wait API yet.
- No cross-process stress test yet.
- No reconnect/reset semantics. Corrupt or rejected lanes remain for a
  later transport-lifecycle/reset cycle to handle.
- No named shared-memory discovery yet; current lifecycle uses anonymous
  memfd and explicit fd passing/duplication.
- No HAL shim integration yet.

## Cross-references

- `.claude/skills/protocol-schema.md` — payload/envelope byte contract.
- `.claude/skills/protocol-session.md` — stateful sequence and
  boot/ack ordering consumed by this endpoint.
- `.claude/skills/layer-2-control-system-backend.md` — broader Layer 2
  backend status and remaining TDD cycles.
- `tests/backend/tier1/shared_memory_transport_test.cpp` — approved T1
  shared-region/endpoint contract tests.
- `tests/backend/tier1/shared_memory_mapping_test.cpp` — approved
  OS-backed mapping/lifecycle tests.
