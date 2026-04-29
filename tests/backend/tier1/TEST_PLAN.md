# Tier 1 shared-memory transport — approved test plan

**Status:** `ready-to-implement` per the `test-reviewer` agent
(two review rounds, 2026-04-29).

The first cycle implements the deterministic in-process shared-region
foundation for Tier 1. The second cycle implements Linux
`memfd_create` / `ftruncate` / `mmap(MAP_SHARED)` lifecycle for hosting
the same region in real OS-backed shared memory. Neither cycle
implements blocking waits, cross-process stress tests, reconnect/reset,
HAL cache mutation, or Tier 2 sockets.

## Contract

- `tier1_shared_region` has two public ABI lanes:
  backend-to-core and core-to-backend.
- Each lane has atomic state, one `sync_envelope`, one payload byte
  count, and fixed inline payload storage.
- `kTier1MaxPayloadBytes` equals the largest current v0 full-capacity
  payload struct.
- Endpoints are nonblocking and use `protocol_session` for envelope
  construction and acceptance.
- Failed sends/receives do not mutate session state or clear evidence
  unless the message is successfully accepted.

## Approved Tests

1. Region starts with both lanes empty and fixed inline payload capacity.
2. Endpoint rejects reserved local direction.
3. Backend boot publishes into backend-to-core lane.
4. Oversize payload send fails without mutating session or lane.
5. Payload exactly at fixed capacity succeeds.
6. Full outbound lane rejects send without consuming sequence.
7. Writing/reading outbound lane rejects send without consuming sequence.
8. Invalid outbound envelope maps to endpoint session error and leaves
   lane empty.
9. Core receives backend boot and clears lane after successful accept.
10. Empty inbound lane returns `no_message`.
11. Writing/reading inbound lane rejects receive without clearing slot.
12. Session-rejected inbound envelope leaves lane full.
13. Boot, boot_ack, tick round trip succeeds across both lanes.
14. Variable-size batch payload copies exactly by header-derived size.
15. Corrupt payload metadata over capacity is rejected without clearing.

## Notes

Payload fixtures are schema-valid. Memory ordering is a code-review
requirement in this cycle: lane state observations use acquire loads,
and publishing/clearing states use release stores. Single-thread tests
do not prove cross-process memory ordering; that belongs to a later
stress/lifecycle cycle.

## OS-backed mapping lifecycle tests

Approved by `test-reviewer` after three rounds on 2026-04-29.

Contract:

- `tier1_shared_mapping::create()` creates a Linux memfd, sizes it
  exactly to `sizeof(tier1_shared_region)`, maps it `MAP_SHARED`, and
  exposes a zero-initialized region.
- `tier1_shared_mapping::map_existing(const unique_fd&)` validates fd
  size and duplicates the input fd for its own ownership.
- `unique_fd` and `tier1_shared_mapping` are move-only RAII types.
- Mapping tests are deterministic same-process tests; fork/exec,
  blocking waits, named shm, and cross-process memory-ordering stress
  remain out of scope.

Approved tests:

1. Create maps zero-initialized region of exact size.
2. Duplicate fd maps peer that observes the same region and owns its fd
   independently.
3. Endpoint boot/boot_ack/tick round trip works across two mappings.
4. Invalid fd is rejected before syscall errno.
5. Wrong-size fd is rejected before mmap.
6. `duplicate_fd` returns independent valid fd ownership.
7. `unique_fd` is move-only and transfers ownership.
8. `tier1_shared_mapping` is move-only and transfers mapping ownership.
9. Peer remains valid after owner mapping is destroyed.
