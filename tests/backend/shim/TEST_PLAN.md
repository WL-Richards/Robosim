# HAL shim core — approved test plan (cycle 1, revision 2)

**Status:** `ready-to-implement` per `test-reviewer` agent (two review
rounds, 2026-04-29). Round 2 verdict approved with two minor
implementation-time fixups already applied to this document (test-3
docstring cite, test-10 Step B explicit `sim_time_us` constant).

The reviewer's third proposed fixup (changing test 9's injected
envelope `sender` from `core_to_backend` to `backend_to_core`) is
**not applied**: it had the polarity backwards. A backend-side
`protocol_session` calls
`validate_envelope(env, remote_direction_, ...)` and `remote_direction_`
for the shim is `core_to_backend`. The injected envelope therefore
must carry `sender = core_to_backend` to pass framing validation and
hit the intended `expected_boot_ack_first` branch. The existing
tier1 test
`Tier1Endpoint.PreservesLaneWhenProtocolSessionRejectsInboundEnvelope`
(shared_memory_transport_test.cpp:358-379) confirms the convention:
the lane name (`backend_to_core` there) and the envelope's `sender`
field are the same direction.

This is the first slice of the HAL shim, the in-process Layer-2
component that the user's robot binary loads (eventually as
`libhal.so`). Cycle 1 is intentionally tiny: the boot handshake plus a
single inbound state slot (the FPGA clock published by sim core) and
the shutdown-terminal receive path. Everything else — power, DS, CAN,
notifier, error_message, on-demand request/reply, the C HAL surface,
LD_PRELOAD, and tier 2 — is deferred to its own future cycle.

## Why this scope

`protocol_session` and `tier1_endpoint` are both green. Nothing yet
*uses* them from the shim's side, which means we have no working
proof that the protocol stack survives end-to-end across a "user
process" / "sim core" boundary. The smallest cohesive feature that
exercises that path is:

- shim sends `boot` (`boot_descriptor` payload) into
  `backend_to_core`,
- shim receives `boot_ack` (zero payload) on `core_to_backend`,
- shim then receives a `tick_boundary` envelope carrying
  `clock_state` and exposes the latest value to a future caller
  (think: the future `HAL_GetFPGATime`),
- shim observes `shutdown` and refuses further inbound traffic.

That's enough to anchor the design seam (state cache + poll-driven
sync over an injected transport) without committing to the rest of
the schema or to any HAL function exports. Each subsequent cycle adds
one more slot/surface on top of the same seam.

## Contract under test

System under test: `robosim::backend::shim::shim_core`, a
single-threaded class that owns one `tier1::tier1_endpoint` (passed
by value at construction) and orchestrates the backend side of the
HAL ↔ sim-core protocol.

Public surface (proposed):

- `static std::expected<shim_core, shim_error> make(`
  `tier1::tier1_endpoint endpoint, const boot_descriptor& desc,`
  `std::uint64_t sim_time_us)`
  — Constructs the shim, immediately publishes a `boot` envelope
  carrying `desc` into the outbound lane via the endpoint. Returns
  the shim on success; returns the wrapped transport/session error
  on failure (lane busy, oversize, wrong-direction endpoint, etc.)
  without leaking partial state.
- `std::expected<void, shim_error> poll()`
  — Drains zero-or-one inbound message from the endpoint and applies
  it to local state. Single message per call by design (matches the
  single-slot lane). No-message is success with no state change.
- `bool is_connected() const` — true once a valid `boot_ack` has been
  received. Until then any inbound `tick_boundary` is a session-order
  error.
- `bool is_shutting_down() const` — true once a `shutdown` envelope
  has been observed.
- `std::optional<clock_state> latest_clock_state() const` —
  `nullopt` until at least one `tick_boundary` carrying
  `schema_id::clock_state` has been accepted; the most recent
  `clock_state` byte-copy thereafter.

Errors:

```cpp
enum class shim_error_kind {
  send_failed,
  receive_failed,
  unsupported_envelope_kind,   // valid framing, not handled this cycle
  unsupported_payload_schema,  // valid framing, not handled this cycle
  shutdown_already_observed,
};

struct shim_error {
  shim_error_kind kind;
  std::optional<tier1::tier1_transport_error> transport_error;
  std::string offending_field_name;
  std::string message;
  bool operator==(const shim_error&) const = default;
};
```

Out of scope for this cycle (each gets its own future cycle):

- Outbound traffic past the boot envelope (no `tick_boundary`,
  `on_demand_request`, or `shutdown` from the shim).
- All inbound schemas other than `clock_state` and the empty
  `boot_ack`/`shutdown`. Receiving e.g. a `power_state` envelope is
  rejected as `unsupported_payload_schema`; this is a deliberate
  cycle-1 limit, not the final design.
- Threading model. The shim is single-threaded; the caller drives
  `poll()` in a loop. No notifications, no condvars, no spin.
- The C HAL surface (`HAL_GetFPGATime` and the rest). This cycle
  exposes a C++ API only.
- Reset/reconnect after shutdown. Once shut down, this object is dead.

## Test fixtures and stubs

- The "core" peer is a real `tier1_endpoint` constructed with
  `direction::core_to_backend` over the same `tier1_shared_region`.
  Both the shim's `tier1_endpoint` and the test's are real; no
  hand-rolled fakes; the SUT (`shim_core`) is never mocked.
- Shared helper: `manually_fill_lane(tier1_lane&, const sync_envelope&,`
  ` std::span<const std::uint8_t>)` is **factored out of the existing
  `tests/backend/tier1/shared_memory_transport_test.cpp` into a new
  header `tests/backend/tier1/test_helpers.h`** as part of this cycle.
  Both the existing tier1 test and the new shim test consume it.
- Where the test reaches behind the endpoint's send path
  (`manually_fill_lane`), the access is at the **public ABI** of
  `tier1_shared_region` (the shared-region struct is the wire
  contract). Used in tests 9 and 13 only, where the test deliberately
  needs a state the sending peer cannot legitimately produce
  (out-of-order envelope; `writing` half-written state). Documented
  inline in each such test.
- Payload helpers `valid_boot_descriptor()` and
  `valid_clock_state(sim_time_us)` mirror the existing tier1-test
  fixtures; same fields and same byte layout.

## Determinism notes

Single-threaded same-process tests. No sleep, no real time, no thread
spawn, no RNG. Same inputs → byte-identical outputs (test 14 pins
this explicitly).

## Approved tests (proposed, revision 2)

### 1. `MakePublishesBootEnvelopeIntoBackendToCoreLane`

- **Layer / contract:** shim_core publishes the boot envelope on
  construction with the caller-supplied descriptor and sim_time_us.
- **Inputs:** fresh region; fresh shim wrapping the backend-side
  endpoint via `make(endpoint, desc, /*sim_time_us=*/123'456)`.
  `desc.runtime = roborio_2`, `desc.team_number = 971`, etc.
- **Expected outputs:**
  - `make` returns a valid shim.
  - `region.backend_to_core.state == full`.
  - The published envelope has `kind == boot`,
    `payload_schema == boot_descriptor`,
    `payload_bytes == sizeof(boot_descriptor)`,
    `sim_time_us == 123'456`,
    `sender == backend_to_core`,
    `sequence == 0`.
  - `std::memcmp(&region.backend_to_core.payload, &desc,
    sizeof(boot_descriptor)) == 0`.
- **Bug class:** missing-publish; wrong-direction sender; wrong
  schema; wrong payload bytes; hardcoded `sim_time_us` (the non-zero
  `t0` makes a "shim writes 0" bug fail loudly).

### 2. `MakeFailsWhenBackendLaneAlreadyFullAndDoesNotClobberLane`

- **Layer / contract:** `make()` failure-atomicity around boot send.
- **Inputs:**
  - Construct a fresh region.
  - Pre-fill `region.backend_to_core` via the shared
    `manually_fill_lane` helper with a sentinel envelope (boot/desc
    framing, sequence 0) and a 32-byte sentinel payload of `0xCC`,
    leaving the lane in state `full`.
  - Then construct a backend endpoint over that region and call
    `shim_core::make(endpoint, desc, 0)`.
- **Expected outputs:**
  - `make` returns
    `shim_error{shim_error_kind::send_failed, transport_error =`
    ` tier1_transport_error{kind = lane_busy, ...}, ...}`.
  - `region.backend_to_core.state == full` (unchanged).
  - The lane's payload bytes still byte-equal the `0xCC` sentinel
    (no shim has written into the lane).
- **Bug class:** `make` clobbering an existing lane; `make`
  succeeding despite a failed publish; partial-state leak.

### 3. `MakeFailsWhenGivenCoreToBackendEndpoint` (new)

- **Layer / contract:** shim must be wired with a `backend_to_core`
  endpoint; passing a `core_to_backend` one is a wiring bug that
  must surface loudly. Surfaces `boot_wrong_direction` from the
  `boot` branch of `protocol_session::validate_send_order`.
- **Inputs:** fresh region; endpoint constructed with
  `direction::core_to_backend`; call `shim_core::make(endpoint, desc,
  0)`.
- **Expected outputs:**
  - `make` returns
    `shim_error{shim_error_kind::send_failed, transport_error =`
    ` tier1_transport_error{kind = session_rejected_envelope,`
    ` session_error = {kind = boot_wrong_direction, ...}}, ...}`.
  - The "core" lane (`region.core_to_backend`) is not touched —
    `state == empty`. The `backend_to_core` lane is also untouched.
- **Bug class:** silent acceptance of a wrong-direction endpoint
  (would cause cross-process protocol deadlock at runtime).

### 4. `IsConnectedFalseAndClockStateNulloptUntilBootAck`

- **Layer / contract:** initial-state contract on observer methods.
- **Inputs:** shim post-`make` (boot already published via test-1
  setup); no `poll()` yet.
- **Expected outputs:**
  - `is_connected() == false`.
  - `is_shutting_down() == false`.
  - `latest_clock_state() == std::nullopt`.
- **Bug class:** shim treating itself as connected after only
  sending boot.

### 5. `PollOnEmptyInboundLaneSucceedsWithNoStateChange`

- **Layer / contract:** `poll()` is well-defined on an empty lane.
- **Inputs:** shim post-`make`; inbound `core_to_backend` lane empty.
- **Expected outputs:**
  - `auto r = shim.poll(); EXPECT_TRUE(r.has_value());`
  - `is_connected() == false` (unchanged).
  - `latest_clock_state() == std::nullopt` (unchanged).
  - The inbound lane state is still `empty`.
- **Bug class:** poll erroring on empty lane; poll spuriously
  flipping state.

### 6. `PollAcceptsBootAckAndFlipsToConnectedExactlyOnce`

- **Layer / contract:** `boot_ack` receipt flips `is_connected()`
  and consumes the lane.
- **Inputs:** shim post-`make`. Test peer's core-side endpoint sends
  a `boot_ack` (kind `boot_ack`, schema `none`, payload bytes 0,
  sequence 0).
- **Expected outputs after the first `poll()`:**
  - `poll()` returns `has_value() == true`.
  - `is_connected() == true`.
  - `is_shutting_down() == false`.
  - `latest_clock_state() == std::nullopt`.
  - `region.core_to_backend.state == empty` (lane drained).
- **Then a second `poll()` immediately:**
  - succeeds with no state change.
- **Bug class:** shim ignoring `boot_ack`; double-flip; failure to
  drain the lane after accepting.

### 7. `PollAfterConnectAcceptsClockStateAndCachesByteEqualValue`

- **Layer / contract:** post-connect, a `tick_boundary`/`clock_state`
  envelope updates the cached slot with full byte fidelity.
- **Inputs:**
  - Shim already connected (boot + boot_ack flow).
  - Core endpoint sends a `tick_boundary` envelope at sequence 1
    with payload schema `clock_state`. The payload struct is
    `valid_clock_state(/*sim_time_us=*/250'000)`, which sets
    `system_active=1`, `system_time_valid=1`, `rsl_state=1`, and
    leaves the rest zero.
- **Expected outputs after `poll()`:**
  - `poll().has_value() == true`.
  - `latest_clock_state().has_value() == true`.
  - `*latest_clock_state() == valid_clock_state(250'000)` using
    `clock_state::operator==` (default-generated) — a single
    full-struct equality, not field-by-field.
  - `region.core_to_backend.state == empty`.
- **Bug class:** payload corruption between transport and cache;
  cache populated for the wrong schema; cache populated only for
  fields the shim happens to read.

### 8. `PollLatestWinsForRepeatedClockStateUpdates`

- **Layer / contract:** cache replacement keeps only the most
  recent value.
- **Inputs:** shim connected. Two `tick_boundary`/`clock_state`
  envelopes applied via the core endpoint in order: first with
  `sim_time_us == 100'000` at sequence 1, then with
  `sim_time_us == 200'000` at sequence 2. A `poll()` is called
  between them and after the second.
- **Expected outputs:**
  - After first poll: `latest_clock_state()->sim_time_us == 100'000`.
  - After second poll: `latest_clock_state()->sim_time_us ==
    200'000`.
- **Bug class:** cache holds the first value forever; cache
  arithmetic-merges; cache mishandles single-slot replacement.

### 9. `PollRejectsTickBoundaryBeforeBootAckPreservingLane` (rewrite)

- **Layer / contract:** stated to be "protocol_session ordering
  honored end-to-end through the shim." This test verifies the
  shim's `poll()` surfaces an `expected_boot_ack_first` rejection
  from its own session, and that per the existing tier1 failure-
  atomicity rule the lane is preserved unchanged.
- **Setup (rewritten — test 8 of the prior revision was structurally
  impossible; see revision rationale below):**
  - Fresh region; backend endpoint; shim via `make` (boot published).
  - Core peer is **not** used to send `boot_ack`; instead, the test
    uses the shared `manually_fill_lane` helper to inject a
    well-framed envelope directly into `region.core_to_backend`:
    - `kind = tick_boundary`,
    - `payload_schema = clock_state`,
    - `payload_bytes = sizeof(clock_state)`,
    - `sequence = 0`,
    - `sender = core_to_backend`,
    - `sim_time_us = 100'000`,
    - payload bytes = `valid_clock_state(100'000)`.
  - **Why direct injection:** no peer can legitimately produce this
    state. A core-side `protocol_session` would refuse to build a
    `tick_boundary` before sending `boot_ack`. The bug class under
    test is "the *shim's* session correctly rejects an unexpected
    inbound that bypassed normal pairing" — that's a real failure
    mode in a multi-process system where the peer could be
    misbehaving or the lane could carry stale data after a crash.
- **Expected outputs:**
  - `poll()` returns
    `shim_error{shim_error_kind::receive_failed, transport_error =`
    ` tier1_transport_error{kind = session_rejected_envelope,`
    ` session_error = {kind = expected_boot_ack_first, ...}}, ...}`.
  - `region.core_to_backend.state == full` (lane preserved per
    tier1's failure-atomicity contract).
  - `region.core_to_backend.envelope` and `.payload_bytes` and
    `.payload` byte-equal the injected values (no clobber).
  - `is_connected() == false`.
  - `latest_clock_state() == std::nullopt`.
- **Bug class:** shim accepts state before the protocol agrees to
  it; shim hides the rejection by clearing the lane; shim silently
  reorders the protocol.

### 10. `PollRejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext`

- **Layer / contract:** cycle-1 explicit limit. Schemas other than
  `clock_state`, `none` (boot_ack/shutdown) are rejected loudly.
  After such a rejection, the session counter is *not* corrupted —
  a fresh valid envelope at the next-expected sequence still
  succeeds. This second half is what makes the loud-reject choice
  safe (no wedged session counter for the loosened cycle 2).
- **Inputs:**
  - Shim connected (boot + boot_ack consumed; expected receive
    sequence is now 1).
  - Step A: core endpoint sends a fully valid
    `tick_boundary`/`power_state` envelope at sequence 1 (a `power`
    payload of zeroes — framing is valid by the kind/schema table).
    Shim `poll()`.
  - Step B: core endpoint then sends a valid
    `tick_boundary`/`clock_state` envelope at sequence 2 with
    payload `valid_clock_state(/*sim_time_us=*/250'000)`. Shim
    `poll()`.
- **Expected outputs:**
  - After Step A:
    - `poll()` returns
      `shim_error{shim_error_kind::unsupported_payload_schema, ...}`
      with `offending_field_name == "payload_schema"`.
    - `is_connected() == true` (unchanged).
    - `latest_clock_state() == std::nullopt`.
    - `region.core_to_backend.state == empty` (the protocol_session
      accepted framing and advanced the receive counter; only the
      shim's post-session dispatch step refused).
  - After Step B:
    - `poll()` returns `has_value() == true`.
    - `*latest_clock_state() == valid_clock_state(250'000)` (full
      struct equality).
    - The fact that Step B succeeds proves the session counter
      advanced past the rejected `power_state` and Step B's
      sequence-2-as-next-expected is honored.
- **Bug class:** shim silently dropping schemas it can't yet handle
  (would mask cycle-2+ divergence); shim corrupting the session
  receive counter on dispatch reject (would wedge the channel).

### 11. `PollAcceptsShutdownAndTransitionsToShuttingDown`

- **Layer / contract:** inbound shutdown is terminal.
- **Inputs:** shim connected (boot + boot_ack). Core endpoint sends
  a `shutdown` envelope (kind `shutdown`, schema `none`, payload
  bytes 0, at the next expected sequence).
- **Expected outputs after `poll()`:**
  - `poll().has_value() == true`.
  - `is_shutting_down() == true`.
  - `is_connected() == true` (the shim doesn't go un-connected; it
    transitions to a separate terminal state).
  - `latest_clock_state()` unchanged from whatever it was before.
- **Bug class:** shutdown not propagated; shutdown processed as a
  regular envelope.

### 12. `PollAfterShutdownReturnsTerminalErrorAndIgnoresLane`

- **Layer / contract:** terminal-state stickiness.
- **Inputs:**
  - Shim post-shutdown (test 11 setup).
  - Core endpoint sends a fresh `tick_boundary`/`clock_state`
    envelope (it is allowed to — core's *send* side is unaffected
    by core having sent a `shutdown`).
- **Expected outputs:**
  - `poll()` returns
    `shim_error{shim_error_kind::shutdown_already_observed, ...}`.
  - `latest_clock_state()` unchanged from before this poll
    (so a future "shim leaked post-shutdown data into cache" bug
    fails).
- **Bug class:** shim ignoring shutdown semantics; shim continuing
  to drain the lane after shutdown; data leakage past shutdown.

### 13. `PollWrapsTransportInProgressErrorWithoutMutatingState`

- **Layer / contract:** the *new* behavior the shim adds on top of
  tier1's already-pinned `lane_in_progress` preservation is the
  error-wrapping into `shim_error`. The lane-preservation half is
  already owned by the tier1 test
  `RejectsInProgressInboundLaneStatesWithoutClearingTheSlot` and is
  not re-asserted here.
- **Inputs:** shim post-`make`. Test sets
  `region.core_to_backend.state` to
  `tier1_lane_state::writing` directly via the public atomic; lane
  envelope and payload are left zero.
- **Expected outputs:**
  - `poll()` returns
    `shim_error{shim_error_kind::receive_failed, transport_error =`
    ` tier1_transport_error{kind = lane_in_progress, ...}, ...}`.
  - `is_connected() == false` (unchanged).
  - `latest_clock_state() == std::nullopt` (unchanged).
- **Bug class:** shim swallows transport errors; shim reports a
  generic error that loses the underlying transport diagnostic.

### 14. `RepeatedRunsProduceByteIdenticalLatestClockState` (determinism)

- **Layer / contract:** non-negotiable #5 (deterministic and
  replayable). The shim is on the deterministic-replay path; the
  same input sequence must produce a byte-identical cache state
  across runs.
- **Inputs:** two independent setups (two regions, two shims, two
  core peers) running the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core sends `boot_ack` at seq 0.
  - shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 1 with
    `sim_time_us = 50'000`.
  - shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 2 with
    `sim_time_us = 100'000`.
  - shim `poll()`.
- **Expected outputs:**
  - For both setups: `latest_clock_state().has_value() == true`.
  - `*setup_a.latest_clock_state() == *setup_b.latest_clock_state()`
    (full struct byte-equality).
  - `setup_a.is_connected() == setup_b.is_connected() == true`.
- **Bug class:** any seam-level nondeterminism (e.g. shim including
  uninitialized stack bytes in its cache, shim reading wall-clock
  somewhere, padding bytes diverging because of ABI surprise).

## Cross-test invariants

- All tests construct one shim per test (no shared state across
  tests).
- All `sim_time_us` values are explicit constants. No wall-clock
  use.
- All payload comparisons use either full-struct `operator==` or
  `std::memcmp` over `sizeof(struct)` bytes.

## Stub disclosure

- The "core" peer is real `tier1_endpoint` + real `protocol_session`
  (the latter is owned inside the endpoint). No hand-rolled fakes.
- The SUT (`shim_core`) is never mocked.
- Tests 9, 13, and 2 reach behind the endpoint's send path to
  manipulate `region.<lane>` directly — only at the lane's public
  ABI (the shared-region struct is the wire contract). Those
  accesses are factored through the shared
  `tests/backend/tier1/test_helpers.h::manually_fill_lane`
  helper. Each test that uses direct access has an inline comment
  explaining why a real peer cannot produce the state under test.

## Revision rationale (round 1 → round 2)

- **Test 9 (was test 8 in r1):** rewritten. The original setup
  ("core endpoint `send`s a tick_boundary while shim hasn't yet
  polled boot_ack") cannot occur against the single-slot core_to_backend
  lane. The rewrite uses `manually_fill_lane` to inject the
  out-of-order envelope directly, which is the technique already
  used by `Tier1Endpoint.PreservesLaneWhenProtocolSessionRejectsInboundEnvelope`.
- **Test 3 (new):** added per `MakeFailsWhenGivenCoreToBackendEndpoint`
  finding (wiring-mistake bug class).
- **Test 10 (was test 9 in r1):** added the post-rejection-recovery
  half (Step B) so a future cycle adding `power_state` doesn't
  inherit a wedged session counter.
- **Test 13 (was test 12 in r1):** scoped to the wrapping behavior
  only; dropped the lane-payload-byte preservation re-checks that
  tier1 already owns.
- **Test 14 (new):** explicit determinism replay test per
  CLAUDE.md non-negotiable #5 and the tdd-workflow guidance on the
  determinism path.
- **Test 1:** non-zero `sim_time_us = 123'456` so a "shim hardcodes
  0" bug fails loudly.
- **Test 4 (was test 3) / 5 (was test 4):** corrected
  "returns void" phrasing to "returns has_value() == true".
- **Test 7 (was test 6):** explicitly states full-struct
  `operator==` byte-equality, not field-by-field.
- **Helper factoring:** `manually_fill_lane` is moved out of
  `tests/backend/tier1/shared_memory_transport_test.cpp` into a
  shared `tests/backend/tier1/test_helpers.h` as part of this cycle;
  the existing tier1 test is updated to consume it from there.
