# HAL ↔ Sim Core protocol schema — approved test plan

**Status:** **`ready-to-implement`** per the `test-reviewer` agent
(three review rounds, 2026-04-29). Implement the tests in this file
exactly as specified; if substantive deviation is needed during
implementation, re-run the test-reviewer cycle on the change rather
than drifting silently. See `.claude/skills/tdd-workflow.md` step 6.

Round-1 returned `not-ready` with forks F1–F4 + mechanical fixes;
v2 applied them. Round-2 returned `not-ready` with a punch list of
mechanical fixes (H7 layout bug, Q-table closure, L truncation
gaps, etc.); v3 applied them. Round-3 returned `ready-to-implement`
with two scrivener-level wording fixes that v4 applies (K1 sizeof
84→88, H8 inter-field-vs-trailing-pad framing, plus per-field
offsets in H8).

**Implementer's first action (next session):** read this document,
fetch and pin WPILib at `kWpilibParitySha` (round-3 reviewer noted
WPILib HAL is not yet in `build/_deps`; the schema cycle's
implementation step needs to add it via CMake FetchContent before
H1–H8 tests can compile against the parity mirrors), then write the
failing tests in this directory, then confirm they fail for the
*right* reason before writing the schema headers.

**Implements:** the next step on `docs/V0_PLAN.md` ("the protocol-
schema POD structs come next, but only after Phase B and a fresh
design discussion in a follow-up session"), and the protocol-schema
discipline pinned in `docs/ARCHITECTURE.md` "Protocol schema (HAL ↔
Sim Core)" and "HAL boundary protocol".

**Layer / skill:** Layer 2 — `.claude/skills/layer-2-control-system-backend.md`.
This feature is *just the schema* — fixed-size POD structs that both
T1 (shared memory) and T2 (socket) transport, plus pure helpers for
string truncation and a pure validator function. Out of scope: the
HAL shim, the sync layer, the LD_PRELOAD libc shim, and the Sim
Core's HAL implementations. Each is a separate TDD cycle.

**WPILib parity baseline:** All "matches WPILib byte-for-byte"
claims in this plan are against WPILib `main` at the SHA recorded
in the test source as `kWpilibParitySha`. If WPILib bumps and
changes a layout, the parity test fails loudly; the developer
either updates the mirror + bumps `kProtocolVersion`, or pins the
old SHA. Drift is a build break, never silent.

---

## Scope

**In:**
- A versioned, magic-prefixed `sync_envelope` used for every
  HAL ↔ Sim Core exchange.
- POD payload structs covering the v0 HAL surface
  (`docs/ARCHITECTURE.md` "v0 scope"):
  TimedRobot tick, Notifier, FPGA timestamp, DS protocol, CAN bus
  transport, power monitoring, error/log (`HAL_SendError`).
- A closed `schema_id` enum.
- A closed `runtime_type` enum (RoboRIO 2 only at v0).
- A closed `envelope_kind` enum.
- A closed `direction` enum.
- Compile-time invariants (`static_assert`) on every payload struct:
  trivially copyable, standard layout, non-polymorphic, aggregate.
- A run-time stateless validator for the envelope (magic, version,
  schema_id, sequence-equality, direction, kind→schema mapping).
- Pure truncation helpers (`copy_truncated`, `copy_bytes_truncated`)
  used to populate fixed-size string/byte buffers.
- An `operator== = default` on every payload struct.

**Out (future TDD cycles):**
- The HAL shim — produces/consumes these structs but is a separate
  feature.
- Sync mechanics (atomics + seqlock for T1, framing + reconnect for
  T2). The transport is wire-format-only at this stage.
- The LD_PRELOAD libc shim and JVM-time interception.
- DS UDP packet parsing — the Sim Core composes a digested
  `ds_state` from a recorded trace or relayed real DS; raw UDP
  bytes never cross this protocol (decision #11).
- Session state — sequence-counter ownership, boot/boot_ack state
  machine, T2 reconnection, "first message in session" enforcement.
  All session-aware checks (forward vs. backward sequence gap,
  out-of-order kind sequencing, on-demand request/reply pairing)
  fire from the future stateful `protocol_session` that wraps the
  validator (decision #2 / fork F4).
- HAL surfaces deferred per v0: DIO, PWM, Analog, Counter, SPI, I²C,
  USB, Relay, pneumatics. Adding any is a `kProtocolVersion` bump
  (decision #5).
- The `tools/rio-bench/` HAL-cost fixture.

---

## Public surface

`src/backend/common/`:

| Header                  | Contains |
|-------------------------|----------|
| `protocol_version.h`    | `kProtocolVersion`, `kProtocolMagic`, `schema_id` enum, `envelope_kind` enum, `direction` enum, `runtime_type` enum. |
| `types.h`               | `hal_bool` = `uint32_t` (mirrors WPILib `HAL_Bool`); `hal_handle` = `int32_t` (mirrors `HAL_Handle`). |
| `truncate.h`/`.cpp`     | `copy_truncated(std::span<char>, std::string_view) -> bool`; `copy_bytes_truncated(std::span<uint8_t>, std::span<const uint8_t>) -> std::size_t`. Pure helpers; no I/O. |
| `sync_envelope.h`       | `sync_envelope` POD header that prefixes every exchange. |
| `clock_state.h`         | `clock_state` — sim time, system_active, browned_out, system_time_valid, FPGA button, RSL, comms-disable count. **Per-tick fields only**; session-invariants live in `boot_descriptor` (fork F1). |
| `power_state.h`         | `power_state` — vin_v, vin_a, brownout_voltage_v. |
| `ds_state.h`            | `control_word`, `alliance_station`, `match_info`, `joystick_axes`, `joystick_buttons`, `joystick_povs`, `joystick_descriptor`, `ds_state` (bundles 6 joysticks + match info + control word + alliance + match-time). All WPILib-mirrored byte-for-byte. |
| `can_frame.h`           | `can_frame` — mirrors `HAL_CANStreamMessage` byte-for-byte (fork F3): `messageID`, `timeStamp`, `data[8]`, `dataSize`. 20 bytes. |
| `can_status.h`          | `can_status` — bus utilization, bus-off / tx-full / rx-error / tx-error counters. |
| `notifier_state.h`      | `notifier_slot`, `notifier_state` (fixed 32-slot array), `notifier_alarm_event`, `notifier_alarm_batch`. |
| `error_message.h`       | `error_message` — severity (hal_bool), error_code (int32), is_lv_code (hal_bool), details[1024], location[256], call_stack[1024], print_msg (hal_bool), truncation_flags (uint8). |
| `boot_descriptor.h`     | `boot_descriptor` — `runtime_type` (uint8) + `reserved3[3]` (zero-pad) + `team_number` (int32) + `vendor_capabilities` (uint32) + `wpilib_version[32]`. Field order in this table matches N1 exactly; both are the wire contract. |
| `validator.h`/`.cpp`    | `validate_envelope(const sync_envelope&, direction expected_sender, uint64_t expected_sequence) -> std::expected<void, validate_error>`. Pure stateless function. |
| `validator_error.h`     | `validate_error_kind` enum, `validate_error` struct with offending field name. |

```cpp
namespace robosim::backend {

inline constexpr uint16_t kProtocolVersion = 1;
inline constexpr std::array<char, 4> kProtocolMagic = {'R', 'S', 'S', '1'};

enum class envelope_kind : uint16_t {
  // closed; reserved=0 reserved-rejected to catch uninitialized structs
  reserved          = 0,
  boot              = 1,
  boot_ack          = 2,
  tick_boundary     = 3,
  on_demand_request = 4,
  on_demand_reply   = 5,
  shutdown          = 6,
};

enum class direction : uint8_t {
  reserved        = 0,  // rejected; uninitialized-struct guard
  backend_to_core = 1,
  core_to_backend = 2,
};

enum class schema_id : uint8_t {
  // closed at v0; adding one bumps kProtocolVersion
  none                  = 0,  // valid only when payload_bytes == 0
  clock_state           = 1,
  power_state           = 2,
  ds_state              = 3,
  can_frame_batch       = 4,
  can_status            = 5,
  notifier_state        = 6,
  notifier_alarm_batch  = 7,
  error_message_batch   = 8,
  boot_descriptor       = 9,
};

enum class runtime_type : uint8_t {
  reserved      = 0,  // rejected
  roborio_2     = 2,  // matches HAL_Runtime_RoboRIO2
  // SystemCore and others are v1+ additives
};

enum class validate_error_kind : uint8_t {
  magic_mismatch,
  version_mismatch,
  unknown_schema_id,
  unknown_envelope_kind,
  unknown_direction,
  schema_payload_kind_mismatch,  // payload_schema not allowed for this kind
  payload_size_mismatch,
  sequence_mismatch,             // sequence != expected; gap is sender-state
  direction_mismatch,
  unsupported_runtime,
};

struct sync_envelope {
  std::array<char, 4> magic;        // == kProtocolMagic
  uint16_t protocol_version;        // == kProtocolVersion
  envelope_kind kind;               // uint16_t backing
  uint64_t sequence;
  uint64_t sim_time_us;
  uint32_t payload_bytes;
  schema_id payload_schema;         // uint8_t backing
  direction sender;                 // uint8_t backing
  std::array<uint8_t, 2> reserved;  // zero-filled, validator does NOT enforce zero (forward-compat)
  bool operator==(const sync_envelope&) const = default;
};
static_assert(std::is_trivially_copyable_v<sync_envelope>);
static_assert(std::is_standard_layout_v<sync_envelope>);
static_assert(sizeof(sync_envelope) == 32);

[[nodiscard]] std::expected<void, validate_error>
validate_envelope(const sync_envelope& env,
                  direction expected_sender,
                  uint64_t expected_sequence);

[[nodiscard]] bool
copy_truncated(std::span<char> dst, std::string_view src);

[[nodiscard]] std::size_t
copy_bytes_truncated(std::span<uint8_t> dst, std::span<const uint8_t> src);

}  // namespace robosim::backend
```

The exact layout of the payload structs is pinned per-section in
the test plan below. The transport layer (future cycle) hands a
`sync_envelope` + raw-byte payload view to the receiver, which calls
the validator, then casts the payload to the type identified by
`payload_schema`.

---

## Decisions to pin

These are the design positions the schema takes. Each is testable
and tested below.

### 1. POD discipline is the contract, enforced at compile time

Every payload struct is `std::is_trivially_copyable_v<T> &&
std::is_standard_layout_v<T> && !std::is_polymorphic_v<T> &&
std::is_aggregate_v<T>` and is `static_assert`'d in its header. This
is what makes the protocol legal to `memcpy`. **Tests do not
re-assert this at runtime** — the static_assert *is* the test; a
runtime mirror adds zero coverage if the header is `#include`d. A
single build-only `compile_time_invariants_test.cpp` `#include`s
every payload header to ensure the asserts get checked.

### 2. Fixed-size only — no `std::string`, `std::vector`, `std::optional`, `std::variant`, raw pointers

Names are `char name[N]` arrays. Variable-count payloads use
`<event>_batch` structs with an explicit `count` and a fixed-size
array. CI grep step bans these types in `src/backend/common/`
(tracked TODO; the `is_trivially_copyable_v` static_assert already
catches `std::string`/`std::vector` indirectly).

### 3. Pinned little-endian on the wire

`static_assert(std::endian::native == std::endian::little)` at
`protocol_version.h`. v0 platforms are x86 host + ARM (qemu-user),
both little-endian. A future BE host adds a byte-swap layer at the
transport boundary, not the schema.

### 4. Magic bytes precede everything

4-byte magic `{'R', 'S', 'S', '1'}` at offset 0 of every envelope.
Cheap "is this the start of a frame?" check; without it, a partial
read silently parses bytes as `version + kind`.

### 5. Schema version handshake is fatal

Boot envelope from the backend declares `protocol_version`. Mismatch
against the Sim Core's compiled `kProtocolVersion` is rejected
loudly. **A `protocol_version == 0` is also rejected** as the
uninitialized-struct guard. The schema "grows organically" by
version-bumping per release.

### 6. Sequence numbers monotonic, gapless, per-direction, **owned by the future session**

Each direction starts at 0 on boot and increments by 1 per envelope.
Gaps are sync errors. **The validator is stateless** (fork F4): it
takes `expected_sequence` as a parameter and reports
`sequence_mismatch` on `actual != expected`. It does *not*
distinguish forward from backward gaps — that requires session state
the validator doesn't own. The future `protocol_session`
(transport-cycle scope) maintains the per-direction counter and
calls the validator with the right value.

### 7. Sim-time timestamps in `uint64_t` microseconds, anchored at sim boot = 0

Matches `HAL_GetFPGATime` units. The 32-bit FPGA hardware counter
wraps at ~71 minutes; `HAL_ExpandFPGATime` patches the upper 32
bits in software; we ship the expanded value. 10 kHz physics fits.

### 8. WPILib HAL constants mirrored exactly

`kMaxJoysticks = 6`, `kMaxJoystickAxes = 12`, `kMaxJoystickPOVs = 12`,
`kMaxJoystickButtons = 32`, `kJoystickNameLen = 256`,
`kEventNameLen = 64`, `kGameSpecificMessageLen = 64`. Cited from
`hal/include/hal/DriverStationTypes.h`.

### 9. Joystick + control-word + match-info + can_frame structs match WPILib byte-for-byte

`joystick_axes`, `joystick_buttons`, `joystick_povs`,
`joystick_descriptor`, `match_info`, `can_frame` mirror their
WPILib counterparts exactly: same field order, same types, same
sizes, same offsets. `control_word` packs the bits at the same
positions WPILib's bitfield emits on x86-64 + ARM Linux (verified
by a mirror-cast test).

**Why:** the future shim's `HAL_GetJoystickAxes(int joystick,
HAL_JoystickAxes* axes)` becomes `*axes = cache.joysticks[i].axes;`
or `std::memcpy(axes, &cache.joysticks[i].axes,
sizeof(HAL_JoystickAxes))` — one line, zero translation, zero
per-field bug surface.

### 10. DS state is digested, not raw UDP bytes

Sim Core composes a `ds_state` struct from the recorded trace or a
relayed real DS. Raw UDP packets never cross this protocol. v1+ may
add an optional `ds_raw_packet` schema_id alongside; v0 does not.

### 11. CAN frame layout = `HAL_CANStreamMessage` byte-for-byte (fork F3)

`can_frame`: `uint32_t messageID; uint32_t timeStamp; uint8_t
data[8]; uint8_t dataSize;` — same field order as
`HAL_CANStreamMessage`. 17 bytes raw, 20 bytes with trailing pad to
`alignof == 4`. Top-of-ID HAL flag bits (`HAL_CAN_IS_FRAME_REMOTE
= 0x80000000`, `HAL_CAN_IS_FRAME_11BIT = 0x40000000`) preserved.

**Why:** `HAL_CAN_ReadStreamSession` hands back `HAL_CANStreamMessage`
arrays directly; the future shim's read path becomes a pointer
cast. Send path (`HAL_CAN_SendMessage` takes individual params, not
a struct) is unaffected — the shim assembles a `can_frame` from the
params and ships it. **Send-side fidelity (the shim's
parameter-to-struct pack) is shim-cycle scope, not schema-cycle.**
The schema cycle pins the layout; the shim cycle pins the
faithfulness of the unpack.

### 12. Notifier alarms are events, not polled state

When the Sim Core's clock crosses an active notifier's
`trigger_time_us`, the next outbound envelope contains a
`notifier_alarm_event`. Shim's `HAL_WaitForNotifierAlarm` blocks
until the event arrives. Matches the FPGA-alarm-IRQ model.

### 13. Notifier table fixed at 32 active notifiers per backend

Real RIO FPGA has 8 alarm slots; 4× headroom for software
multiplexing. Exceeding 32 on `HAL_InitializeNotifier` is a
backend-startup error.

### 14. Error/log surface caps strings explicitly with a truncation flag (silent, fork F2 = A)

`HAL_SendError` becomes `error_message{ severity, error_code,
is_lv_code, details[1024], location[256], call_stack[1024],
print_msg, truncation_flags }`. **Truncation is performed by the
pure `copy_truncated` helper** (decision #20) shipping in this
cycle. Source > buffer-size → `dst[N-1] = '\0'`, returns `true`,
caller sets the truncation_flags bit. v1+ adds an
`error_message_continuation` schema_id if real callers exceed the
caps.

### 15. Schema ID enum is closed at v0; adding one bumps kProtocolVersion

The validator rejects `schema_id` values not in the closed enum.
**`schema_id::none` is valid only when `payload_bytes == 0`** —
otherwise `schema_payload_kind_mismatch`.

### 16. `runtime_type` is closed; v0 accepts only `roborio_2`

Boot descriptor carries `runtime_type`. `runtime_type::reserved`
(uninitialized) and any future-but-not-yet-supported value →
`unsupported_runtime`. v1+ adds `system_core` etc. with a
version bump.

### 17. `operator== = default` on every payload struct

Determinism comparisons across replays. Same pattern as the
description loader.

### 18. One header per HAL subsystem

Mirrors RIOEmulator's per-FPGA-subsystem decomposition.

### 19. Validator is fully stateless (fork F4)

The validator owns no session memory. Returns
`std::expected<void, validate_error>`. Stateful checks (sequence
gap direction, first-message-in-session, on-demand request/reply
pairing, T2 reconnection) ship in the future transport-cycle
`protocol_session` that wraps this validator.

### 20. String-truncation helpers ship in this cycle (fork F2 = A)

`copy_truncated(std::span<char> dst, std::string_view src) -> bool`
copies `min(src.size(), dst.size() - 1)` bytes, null-terminates at
that position, returns `true` iff truncation occurred. **Bytes in
`dst` past the null terminator are not modified** — caller is
responsible for zero-init if a clean tail is required (typical
flow: `dst{}` zero-initialized, then `copy_truncated`).

`copy_bytes_truncated(std::span<uint8_t> dst, std::span<const
uint8_t> src) -> std::size_t` copies `min(dst.size(), src.size())`
bytes, returns the count actually written. No null terminator (this
is for `HAL_MatchInfo::gameSpecificMessage`, which is a byte buffer
with an explicit length field).

**Overlap is undefined behavior**, matching `std::memcpy` semantics
(the helpers use `std::memcpy` internally). Callers must ensure
`dst` and `src` do not alias. If a future use case needs overlap-
safe semantics, swap to `std::memmove` and pin that explicitly —
don't make it a per-call decision.

Helpers are pure, no I/O, no allocation. Caller (the future shim)
combines `copy_truncated` with explicit `truncation_flags` bit
sets.

### 21. `HAL_MatchType` backing is `int32_t`

Pinned explicitly in the test (mirror struct + `static_assert`)
because C enum backing is implementation-defined. Verified against
WPILib at `kWpilibParitySha`.

### 22. `error_message::severity` is `hal_bool` (= `uint32_t`)

Mirrors `HAL_SendError`'s `HAL_Bool isError` parameter (1 = error,
0 = warning/info). Single bool, not a multi-level enum, because
WPILib's surface is binary. `error_code` carries the granular
severity-equivalent for users that want it.

### 23. Null-termination convention for fixed-buffer strings

All `char[N]` fields in payload structs are **null-terminated
within the buffer**: the last byte (`buf[N-1]`) is always `'\0'`
when the field has been populated through `copy_truncated`.
Default-constructed (zero-initialized) payload structs have all
bytes 0, which trivially satisfies this.

`uint8_t[N]` byte buffers (e.g. `match_info::gameSpecificMessage`)
are **not** null-terminated; they carry an explicit
`gameSpecificMessageSize` companion field giving the length.

### 24. Reserved bytes: zero on send, ignored on receive

`sync_envelope::reserved[2]` (and `boot_descriptor::reserved3[3]`,
`notifier_alarm_event::reserved_pad`, etc.): the sender writes
zero (verified by a test that `memset`-zeros an envelope, then
populates only named fields, and asserts `reserved == {0, 0}`); the
validator does **not** reject non-zero reserved bytes. This gives
forward-compat headroom without making old senders incompatible
with future receivers that might use those bytes.

**Forward-compat policy, stated explicitly** (round-2 reviewer
asked for this): older senders are not required to zero reserved
fields; receivers behaving as if they do is the cost of forward
compatibility. The risk is the inverse — a buggy sender writes
garbage into `reserved` and the receiver silently accepts it. We
accept that risk because: (a) C2-class tests catch correctly-
written senders that drift into reserved offsets (a struct rebase
that overlaps would fail C2); (b) when a future v1+ assigns
meaning to a previously-reserved byte, drift surfaces as a logic
bug at that point under the version bump (decision #5), not a
silent wire corruption today. The policy mirrors TCP and IP
header reserved fields; it is a standard forward-compat pattern.

---

## Pre-review shape questions and post-round-1 forks

Round 0 (pre-reviewer, with user, 2026-04-29):

- **Q1 — `sync_envelope` size.** Resolved: **32 bytes**.
- **Q2 — validator shape.** Resolved: **free function**.
- **Q3 — `error_message` truncation.** Resolved: **silent with flag**.
- **Q4 — `team_number` placement.** Round-0 resolution flipped by F1.

Round 1 (post-reviewer, with user, 2026-04-29):

- **F1 — `team_number` placement (flipped from Q4).** Resolved:
  **`team_number` lives in `boot_descriptor`, not per-tick
  `clock_state`.** Reviewer's argument: a session-invariant inside
  a per-tick struct is a silent-corruption attractor (same lesson
  as description-loader decision #8 generalized to data shape). The
  shim's "special case" cost is one line of caching at boot.
- **F2 — truncation-helper scope.** Resolved: **option A — schema
  cycle ships `copy_truncated` and `copy_bytes_truncated`**
  (decision #20). Pure, testable, lives with the layout.
- **F3 — CAN frame parity.** Resolved: **option A — `can_frame`
  mirrors `HAL_CANStreamMessage` byte-for-byte** (decision #11).
  20 bytes, WPILib field order. Receive path becomes a pointer
  cast.
- **F4 — validator stateless or not.** Resolved: **fully
  stateless** (decision #19). All session-aware checks move to the
  transport cycle.

---

## Test sections

GoogleTest. Test names follow `code-style.md`: suite is the SUT,
test name is the behavior. No fixtures bigger than necessary.

### A. POD discipline (compile-time)

- A1. `compile_time_invariants_test.cpp` — a build-only translation
  unit that `#include`s every payload header. Forces every header's
  `static_assert(std::is_trivially_copyable_v<T> &&
  std::is_standard_layout_v<T> && !std::is_polymorphic_v<T> &&
  std::is_aggregate_v<T>)` to fire. **No runtime mirror tests** —
  the static_assert is the contract.

- A2. `static_assert(sizeof(sync_envelope) == 32)` and
  `static_assert(alignof(sync_envelope) == 8)`. The wire-contract
  struct's literal layout is load-bearing and pinned.

- A3. **For our-design structs** (`sync_envelope`, `clock_state`,
  `power_state`, `can_status`, `error_message`, `boot_descriptor`,
  `notifier_slot`, `notifier_alarm_event`,
  `notifier_alarm_batch`-header, `<batch>` headers): a
  `padding_hygiene_test` enumerates all fields, computes
  `sizeof(T) - sum_of_field_sizes`, asserts == 0. **No interior
  pad, no trailing pad.** We ordered fields to make this hold.

- A4. **For WPILib-mirrored structs** (`joystick_axes`,
  `joystick_buttons`, `joystick_povs`, `joystick_descriptor`,
  `match_info`, `can_frame`): the test pins `sizeof`, `alignof`,
  every `offsetof(field)`, **and the explicit set of padding
  bytes** (e.g. `match_info` has 1 byte interior pad before
  `gameSpecificMessageSize` + 2 bytes trailing — pinned). Padding
  is intentional because we're mirroring WPILib's layout; we own
  the fact that it's there.

- A5. `hal_bool` typedef pin: `static_assert(sizeof(hal_bool) == 4)`,
  `static_assert(std::is_same_v<hal_bool, uint32_t>)`. Decision #10
  shim-memcpy story breaks if `hal_bool` ever becomes a 1-byte
  `bool`.

- A6. `hal_handle` typedef pin: `static_assert(std::is_same_v<
  hal_handle, int32_t>)`. Mirrors `HAL_Handle`.

### B. Endianness gate

- B1. `static_assert(std::endian::native == std::endian::little)` at
  `protocol_version.h`. CI fails-to-compile on a hypothetical BE
  host.

### C. sync_envelope structural

- C1. `offsetof` for every field: `magic == 0`, `protocol_version
  == 4`, `kind == 6`, `sequence == 8`, `sim_time_us == 16`,
  `payload_bytes == 24`, `payload_schema == 28`, `sender == 29`,
  `reserved == 30`. Pinned literal numbers — wire-contract.
- C2. **Reserved bytes round-trip-safe.** `sync_envelope env{};`
  — populate only named fields — `memcpy` to byte buffer,
  `memcpy` back, assert `env.reserved == {0, 0}`. Confirms
  default-zero plus that no field's write spills into `reserved`.
- C3. **Bit-identical equality**: two envelopes with bit-identical
  fields compare equal under `operator==`.

### D. Validator (envelope-level checks, stateless)

- D1. Wrong magic → `magic_mismatch`, message contains "magic".
- D2. `protocol_version != kProtocolVersion` → `version_mismatch`
  with offending value in message.
- D3. `protocol_version == 0` → `version_mismatch` (uninit guard).
- D4. `payload_schema` value > 9 (outside closed enum) →
  `unknown_schema_id`.
- D5. `kind` value 0 (`reserved`) → `unknown_envelope_kind` (uninit
  guard).
- D6. `kind` value > 6 → `unknown_envelope_kind`.
- D7. `sender` value 0 (`reserved`) → `unknown_direction` (uninit
  guard).
- D8. `sender` value > 2 → `unknown_direction`.
- D9. `sender == backend_to_core` and validator called with
  `expected_sender == core_to_backend` → `direction_mismatch`.
- D10. `sequence == expected_sequence` → ok.
- D11. `sequence != expected_sequence` (positive delta and negative
  delta tested as one parameterized test) → `sequence_mismatch`,
  message contains both expected and actual values. **Per fork F4,
  the validator does not distinguish forward from backward gap.**
- D12. `sequence == 0` with `expected_sequence == 0` → ok
  (boundary, fresh-session).
- D13. `sequence == UINT64_MAX` with `expected_sequence ==
  UINT64_MAX` → ok (boundary; wrap is not a v0 concern).
- D14. `payload_schema == none` with `payload_bytes == 0` → ok.
- D15. `payload_schema == none` with `payload_bytes != 0` →
  `schema_payload_kind_mismatch`.
- D16. `payload_schema == clock_state` with `payload_bytes !=
  sizeof(clock_state)` → `payload_size_mismatch`. Parameterized
  across every fixed-size schema (clock_state, power_state,
  ds_state, can_status, boot_descriptor).
- D17. **Variable-size batch payload sizes** (`can_frame_batch`,
  `notifier_alarm_batch`, `error_message_batch`, `notifier_state`):
  validator checks `payload_bytes == header_size + count *
  element_size` where `count` is read from the payload header —
  i.e., the validator inspects the first bytes of payload to find
  the count, then checks consistency with `payload_bytes`.
  Parameterized: each batch schema, valid count, count-too-large
  (capacity exceeded → `payload_size_mismatch`).

  **Test-source comment must explicitly state the boundary:** the
  validator decodes the batch *header* (the `count` field) but
  never decodes any element body. Element-body validation is the
  receiver's responsibility once the validator has confirmed the
  envelope/payload framing. Without this comment a future
  maintainer might extend the validator to range-check element
  contents and silently take on transport-cycle scope.

### E. Round-trip byte-copy contract

- E1. **One test per payload struct**: fill every byte of an
  instance with a unique non-zero value (`buf[i] = (i + 1) & 0xFF`)
  by writing into a buffer of `sizeof(T)` bytes, then `memcpy` into
  the struct, then `memcpy` the struct back into a second buffer,
  then XOR the two buffers byte-for-byte and assert all zeros.
  **Pad bytes are part of the unique fill** — they round-trip too,
  because `memcpy` of `sizeof(T)` bytes copies pad bytes verbatim.
  Guards against any path that mishandles a byte (alignment hole
  silently dropping data, unintended union, byte-swap-on-store).
  The unique-byte rule means a byte-swap or shift bug doesn't pass
  via accidental symmetry. (Round-2 reviewer asked for this
  clarification — silence on whether pads are filled was the issue;
  the answer is "yes, fill them too.")

### F. clock_state fields

- F1. `sim_time_us` round-trips at 0, 1, 1_000_000, UINT64_MAX.
- F2. `system_active`, `browned_out`, `system_time_valid`,
  `fpga_button_latched`, `rsl_state` — round-trip as `hal_bool`
  values 0 and 1.
- F3. `comms_disable_count` round-trips at 0 and UINT32_MAX.
- F4. `clock_state` does **not** contain `team_number` (per fork
  F1); a `static_assert` that "team_number" is not a member name
  catches a regression. Implemented as a SFINAE check or, more
  simply, a comment-pinned absence acknowledged by reviewer.

### G. power_state fields

- G1. `vin_v` round-trips at 0.0f, 7.0f, 12.6f, 13.5f. Test-source
  comment notes provenance ("RoboRIO 2 nominal operating range");
  this is structural, not a physical-model test, so no tolerance is
  asserted.
- G2. `vin_a` round-trips at 0.0f and 100.0f.
- G3. `power_state` default-constructs to all-zero. Test asserts
  `power_state{}.vin_v == 0.0f` — pins the zero-init contract that
  the rest of the protocol relies on.
- G4. `brownout_voltage_v` round-trips at 6.75f (the WPILib
  default). Round-trip pin only — defaults are the shim's concern.

### H. ds_state and WPILib parity (load-bearing)

WPILib mirrors are declared **locally in the test file** with the
exact field types from `DriverStationTypes.h@{kWpilibParitySha}`.
A test-source comment cites the SHA. When WPILib bumps and changes
a layout, the test fails; the developer either updates the mirror
+ bumps `kProtocolVersion`, or pins the older SHA.

- H1. `joystick_axes` byte parity. WPILib: `int16_t count; float
  axes[12]; uint8_t raw[12];` → 2 + 2 leading pad + 48 + 12 = 64,
  `alignof == 4`. Test pins `sizeof(joystick_axes) ==
  sizeof(HAL_JoystickAxes_mirror) == 64`,
  `offsetof(count) == 0`, `offsetof(axes) == 4`,
  `offsetof(raw) == 52`. Round-trip via `std::bit_cast` from our
  struct to WPILib mirror to bytes and back.
- H2. `joystick_buttons` byte parity. WPILib: `uint32_t buttons;
  uint8_t count;` → 4 + 1 + 3 trailing pad = 8, `alignof == 4`.
  Pin sizes and offsets.
- H3. `joystick_povs` byte parity. WPILib: `int16_t count; int16_t
  povs[12];` → 2 + 24 = 26, `alignof == 2`, **no trailing pad**.
  Test pins `sizeof == 26`. (Round-1 reviewer caught the previous
  draft's incorrect "28" — fixed.)
- H4. `joystick_descriptor` byte parity. WPILib: `uint8_t isXbox;
  uint8_t type; char name[256]; uint8_t axisCount; uint8_t
  axisTypes[12]; uint8_t buttonCount; uint8_t povCount;` → 1 + 1
  + 256 + 1 + 12 + 1 + 1 = 273, `alignof == 1`, no padding. Pin
  `sizeof == 273` and per-field offsets.
- H5. `control_word` bitfield mirror-cast. Test declares a local
  `HAL_ControlWord_mirror` (the WPILib bitfield struct), constructs
  it with one bit set at a time (`enabled = 1, autonomous = 0,
  ...`), `std::bit_cast`s it to `uint32_t`, asserts the bit
  position is what our `control_word` puts in the same bit
  position. Pins decision #9 — bitfield position parity, not just
  "we put bit 0 at bit 0." Test-source comment notes the ABI
  assumption: GCC and Clang on x86-64 + ARM Linux both lay
  bitfields out LSB-first. If this project is ever ported to MSVC
  or another compiler, the parity needs re-validating; the test
  will fail and the comment tells the future porter why.
- H6. `alliance_station` enum values: `kUnknown=0, kRed1=1,
  kRed2=2, kRed3=3, kBlue1=4, kBlue2=5, kBlue3=6` (per WPILib).
  Pin enum values; pin `static_assert(sizeof(alliance_station)
  == 4)` (default int backing, matches WPILib).
- H7. `match_info` byte parity. WPILib: `char eventName[64];
  HAL_MatchType matchType; uint16_t matchNumber; uint8_t
  replayNumber; uint8_t gameSpecificMessage[64]; uint16_t
  gameSpecificMessageSize;`. With `HAL_MatchType` backed by
  `int32_t` (decision #21, verified by mirror-cast),
  `alignof == 4`. Layout (verified by compiling WPILib's
  `HAL_MatchInfo` at `kWpilibParitySha` on x86-64 Linux GCC):
  - `eventName` at 0 (size 64).
  - `matchType` at 64 (size 4; naturally aligned because 64 % 4 == 0).
  - `matchNumber` at 68 (size 2).
  - `replayNumber` at 70 (size 1).
  - `gameSpecificMessage` at **71** (size 64; `uint8_t[64]` has
    alignof 1, so no pad needed before it).
  - **1 byte interior pad at offset 135** (between
    `gameSpecificMessage` end at 134 and the align-2
    `gameSpecificMessageSize` at 136).
  - `gameSpecificMessageSize` at 136 (size 2).
  - **2 bytes trailing pad at offsets 138..139** (struct alignof
    is 4 because of `matchType`).
  - Total `sizeof == 140`, `alignof == 4`.

  Round-2 reviewer caught the previous draft's incorrect placement
  (it had `gameSpecificMessage` at 72 with the interior pad at 71;
  in fact `uint8_t[]` aligns to 1 and the pad is at 135, not 71).
  Fixed.
- H8. `ds_state` total size pin and per-field offset pin. Field
  order chosen as: `joystick_axes[6]` first, then
  `joystick_buttons[6]`, `joystick_povs[6]`,
  `joystick_descriptor[6]`, `control_word`, `alliance_station`,
  `match_info`, `match_time_seconds`. C++ guarantees
  `sizeof(T[N]) == N * sizeof(T)` regardless of alignof.

  Sub-array sizes:
  - 6 × `joystick_axes` = 6 × 64 = 384.
  - 6 × `joystick_buttons` = 6 × 8 = 48.
  - 6 × `joystick_povs` = 6 × 26 = **156**. (Round-2 reviewer's
    catch — there is no per-element padding regardless of alignof.)
  - 6 × `joystick_descriptor` = 6 × 273 = 1638.

  **Per-field offsets pinned by the test** (verified on x86-64
  GCC, Round-3-reviewer-confirmed):
  - `joystick_axes[6]` at 0..384.
  - `joystick_buttons[6]` at 384..432.
  - `joystick_povs[6]` at 432..588.
  - `joystick_descriptor[6]` at 588..2226.
  - **2 bytes inter-field pad at 2226..2227** (compiler aligns
    `control_word` to 4-byte boundary; round-3 reviewer's catch
    that this is *inter-field*, not trailing).
  - `control_word` at 2228 (size 4).
  - `alliance_station` at 2232 (size 4).
  - `match_info` at 2236 (size 140).
  - `match_time_seconds` (double) at 2376 (size 8; naturally
    8-aligned because 2376 % 8 == 0).
  - `match_time_seconds` ends at 2384. **No trailing pad**:
    `match_time_seconds` at offset 2376 with size 8 already
    satisfies the struct's `alignof == 8` requirement, so the
    natural total size is 2384.

  Test pins the literal `sizeof(ds_state) == 2384`,
  `alignof == 8`, and every per-field offset listed above. A
  future field reorder that *accidentally* preserves total size
  but moves a sub-field still fails the per-field offset
  assertions. Padding-hygiene check confirms only the named
  inter-field pad (at 2226..2227) accounts for the gap between
  the named-field-size sum (2382) and `sizeof` (2384).
- H9. **DS strings populated via `copy_truncated`.** A per-string
  truncation test (`event_name`, joystick `name`,
  `gameSpecificMessage` via `copy_bytes_truncated`) belongs in
  Section L, not here. H9 only pins that `ds_state`'s default-
  constructed instance has all-zero string buffers (so the
  null-termination invariant holds without any helper call).

### I. can_frame structural (HAL_CANStreamMessage parity, fork F3)

- I1. WPILib: `uint32_t messageID; uint32_t timeStamp; uint8_t
  data[8]; uint8_t dataSize;`. Layout: messageID at 0, timeStamp
  at 4, data at 8, dataSize at 16, **3 bytes trailing pad at 17**,
  total `sizeof == 20`, `alignof == 4`. Pin offsets and sizeof,
  pin trailing pad as known and intentional.
- I2. `messageID` upper bits preserve `HAL_CAN_IS_FRAME_REMOTE
  (0x80000000)` and `HAL_CAN_IS_FRAME_11BIT (0x40000000)` after
  byte round-trip. Bottom 29 bits independently round-trip.
- I3. `dataSize` boundary: 0 and 8.
- I4. `dataSize > 8` is **out of scope of the schema** — neither
  the validator nor the struct enforces a range. The HAL shim /
  Sim Core enforce; documented in the test as an explicit non-
  guarantee.
- I5. `can_frame_batch`: header carries `count`; payload carries
  `frames[kMaxCanFramesPerBatch]` with `kMaxCanFramesPerBatch =
  64`. `count > 64` is a **validator** error
  (`payload_size_mismatch`, see D17).

### J. can_status structural

- J1. Field order matches `HAL_CAN_GetCANStatus` parameter order:
  `percent_bus_utilization (float)`, `bus_off_count (uint32)`,
  `tx_full_count (uint32)`, `receive_error_count (uint32)`,
  `transmit_error_count (uint32)`. Pin offsets.
- J2. `percent_bus_utilization` round-trips at 0.0f, 0.5f, 1.0f.

### K. notifier_state and notifier_alarm_event

- K1. `notifier_slot` field order chosen for zero interior pad:
  `trigger_time_us (uint64) at 0; handle (int32) at 8;
  alarm_active (hal_bool) at 12; canceled (hal_bool) at 16;
  name[64] at 20`. **Sum of named field sizes = 84; `sizeof ==
  88`; trailing pad = 4 bytes; `alignof == 8`.** Pin per-field
  offsets and the trailing-pad value. (Round-3 reviewer caught
  the previous draft's "sizeof == 84" wording — 84 is the named-
  field sum; 88 is the actual `sizeof` after compiler-inserted
  trailing pad to satisfy `alignof == 8`.)
- K2. `notifier_state`: header carries `count`; array of 32
  `notifier_slot`s. `count > 32` → validator
  `payload_size_mismatch` per D17.
- K3. `notifier_alarm_event` layout pinned to
  **`fired_at_us (uint64) at 0; handle (int32) at 8; reserved_pad
  (uint32) at 12`**. `sizeof == 16`, `alignof == 8`, no implicit
  pad — `reserved_pad` is a named field zero-filled on send,
  giving us forward-compat headroom (decision #24-style) without
  hidden alignment holes. Test pins these offsets and the explicit
  `reserved_pad`. (Round-2 reviewer caught that the previous draft
  said "test pins whichever order the implementation picks" —
  letting the implementer pick is letting the test grade itself.
  Fixed.)
- K4. `notifier_alarm_batch`: header carries `count`; array of 32
  events. `count > 32` → validator `payload_size_mismatch`.

### L. truncation helpers and error_message

The truncation helpers are pure functions (decision #20). Tested
directly here; downstream payload structs (error_message,
match_info, joystick_descriptor, boot_descriptor) consume them via
the future shim or test-side construction.

- L1. `copy_truncated(dst, src)` with `src.size() < dst.size()`:
  copies `src.size()` bytes; `dst[src.size()] == '\0'`; returns
  `false`. **Test pre-fills `dst` with `0xAB` before the call,**
  then asserts: bytes `dst[0..src.size()-1]` match `src`,
  `dst[src.size()] == '\0'`, and bytes `dst[src.size()+1..]` are
  still `0xAB` (helper does not zero them — caller's responsibility).
  Without the pre-fill assertion the "post-null bytes preserved"
  contract is unverified.
- L2. `copy_truncated(dst, src)` with `src.size() == dst.size() -
  1`: copies exactly that many bytes; `dst[size-1] == '\0'`;
  returns `false`.
- L3. `copy_truncated(dst, src)` with `src.size() == dst.size()`:
  copies `dst.size() - 1` bytes; `dst[size-1] == '\0'`; returns
  `true`.
- L4. `copy_truncated(dst, src)` with `src.size() > dst.size()`:
  same as L3.
- L5a. `copy_truncated` with `dst.size() == 0` and `src.size() > 0`:
  returns `true`, copies nothing (empty buffer can't even hold the
  null terminator; truncation occurred).
- L5b. `copy_truncated` with `dst.size() == 0` and `src.size() ==
  0`: returns `false`. Nothing was lost; the boundary case
  resolves to "no truncation by vacuity." Pinned deliberately
  rather than left to the implementer's coin flip.
- L6. `copy_truncated` with `src.size() == 0` and `dst.size() >
  0`: returns `false`, `dst[0] == '\0'`.
- L7. `copy_bytes_truncated(dst, src)` with `src.size() <
  dst.size()`: returns `src.size()`, copies `src.size()` bytes,
  bytes beyond stay at pre-call values (no auto-zero). Test
  pre-fills `dst` with `0xCD` and asserts the tail.
- L8. `copy_bytes_truncated` with `src.size() > dst.size()`:
  returns `dst.size()`, copies `dst.size()` bytes.
- L8b. `copy_bytes_truncated` with `src.size() == 0` and
  `dst.size() == 0`: returns `0`.
- L8c. `copy_bytes_truncated` with `src.size() == 0` and
  `dst.size() > 0`: returns `0`, `dst` unchanged from pre-call.
- L8d. `copy_bytes_truncated` with `src.size() > 0` and
  `dst.size() == 0`: returns `0`, copies nothing.
- L9. `error_message` populated via `copy_truncated` for each of
  `details`, `location`, `call_stack`. Test composes a long source
  string, calls the helper, sets the truncation flag bit, then
  byte-round-trips the populated `error_message`. Pins the
  end-to-end populate-and-ship flow.
- L10. `error_message::truncation_flags` bit assignments are
  defined and stable: bit 0 = details, bit 1 = location, bit 2 =
  call_stack. Pin the values; mismatches are silent corruption.

### M0. validate_error structural

- M0a. `static_assert(std::is_trivially_copyable_v<validate_error>)`,
  `static_assert(std::is_standard_layout_v<validate_error>)` —
  folded into A1's compile-time TU.
- M0b. Each test in section D that asserts on a `kind` value also
  asserts that `error.offending_field_name` is non-empty and
  contains the expected substring (e.g. D2's `version_mismatch`
  has `offending_field_name` containing `"protocol_version"`).
  This pins decision #19's claim that the field-name string is
  part of the contract; tests pin substring containment, never
  exact phrasing, to avoid pinning message style.
- M0c. `validate_envelope` returns an `std::expected<void,
  validate_error>` whose `has_value()` is `true` on success and
  `false` on every D-section error case. Implicit in every
  D-test, but explicitly asserted at least once for clarity.

### M. Closed-enum gates (validator + struct)

- M1. Validator accepts `schema_id` values 0..9; rejects 10..255
  with `unknown_schema_id`. Parameterized; covers the full
  `uint8_t` range.
- M2. Validator accepts `envelope_kind` values 1..6; rejects 0 and
  7..65535 with `unknown_envelope_kind`. Parameterized over the
  full `uint16_t` range (sampled, not exhaustive — sampled at
  every boundary).
- M3. Validator accepts `direction` values 1..2; rejects 0 and
  3..255 with `unknown_direction`.
- M4. `runtime_type::roborio_2 (= 2)` accepted in
  `boot_descriptor`. `runtime_type::reserved (= 0)` and any
  other value 1, 3..255 → `unsupported_runtime`. Parameterized.
- M5. Schema_id round-trip preserves the underlying `uint8_t`
  (no implicit promotion).

### N. boot_descriptor

- N1. Layout: `runtime_type (uint8) at 0; reserved3[3] at 1
  (zero-pad to align uint32); team_number (int32) at 4;
  vendor_capabilities (uint32) at 8; wpilib_version[32] at 12`.
  `sizeof == 44`, `alignof == 4`, no interior pad beyond the
  named `reserved3[3]`. Pin offsets.
- N2. `runtime_type != roborio_2` → `unsupported_runtime` (per
  M4).
- N3. `team_number` round-trips at 0, valid FRC range (1..9999),
  and `INT32_MAX` boundary.
- N4. `wpilib_version` populated via `copy_truncated`. L1–L4 cover
  the helper; N4 just pins that `wpilib_version` is consumed via
  the helper (test-side construction calls `copy_truncated`).

### O. Padding hygiene (no spurious bytes carry state)

- O1. **Renamed from "determinism" — what we're really pinning is
  that no struct has an unaccounted-for byte that could carry
  uninitialized state into a wire-byte stream.** For each payload
  struct: zero-initialize, populate every named field with a
  distinguishing value, `memcpy` to a buffer, recompute the buffer
  fully from the named fields' bytes, assert the buffer matches.
  Bytes outside named fields (the WPILib-mirrored interior pads in
  H7, the trailing pads stated in A4/K1) must be **zero** (because
  the source struct was zero-initialized). Catches: a future field
  insertion that creates a hidden alignment hole.

(O1 replaces the previous draft's O1+O2 — the two-runs-byte-equal
test was redundant with E1; the perturbation test is what we
actually want.)

### P. Hidden-state guards (compile-time only)

- P1. `static_assert(!std::is_polymorphic_v<T>)` for every
  payload (folded into A1's compile_time_invariants_test).
- P2. `static_assert(std::is_aggregate_v<T>)` for every payload
  (also folded into A1).
- P3. **Removed.** Originally a grep ban on `std::string` /
  `std::vector` / etc. — redundant with `is_trivially_copyable_v`
  (A1) for the relevant types. CI grep step is tracked as a
  follow-up TODO outside this test plan.

### Q. Envelope-kind ↔ payload-schema mapping (validator)

A parameterized test for the static rule "for envelope_kind X,
payload_schema must be in {allowed-set}". The mapping table is part
of the validator's public surface and is **fully closed** — no
"refined per shim cycle" punts (round-2 reviewer's catch).

The closed mapping (`kPerKindAllowedSchemas`):

| envelope_kind     | allowed payload_schema(s)                                                                                                                                                |
|-------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| boot              | `{boot_descriptor}`                                                                                                                                                      |
| boot_ack          | `{none}` (with `payload_bytes == 0`)                                                                                                                                     |
| tick_boundary     | `{clock_state, power_state, ds_state, can_status, can_frame_batch, notifier_state, notifier_alarm_batch, error_message_batch}` — every per-tick-published schema, closed |
| on_demand_request | `{clock_state, power_state, ds_state, can_status, can_frame_batch, notifier_state, notifier_alarm_batch}` — error_message_batch excluded (errors are pushed, not requested) |
| on_demand_reply   | `{clock_state, power_state, ds_state, can_status, can_frame_batch, notifier_state, notifier_alarm_batch, error_message_batch}` — same as tick_boundary; replies may carry a synchronous error result |
| shutdown          | `{none}` (with `payload_bytes == 0`)                                                                                                                                     |

`boot_descriptor` is legal **only** under `boot`; `none` is legal
only under `boot_ack` and `shutdown` (where `payload_bytes` must
also be 0). Future v1+ surface additions extend this table only via
`kProtocolVersion` bump.

- Q1. Boot with `payload_schema == boot_descriptor` and matching
  `payload_bytes == sizeof(boot_descriptor)` → ok.
- Q2. Boot with `payload_schema ∈ {everything else}` → parameterized
  rejection with `schema_payload_kind_mismatch`.
- Q3. boot_ack with `payload_bytes == 0` and `payload_schema ==
  none` → ok.
- Q4. boot_ack with `payload_bytes != 0` →
  `schema_payload_kind_mismatch` (regardless of payload_schema).
- Q5. shutdown with `payload_bytes == 0` and `payload_schema ==
  none` → ok.
- Q6. shutdown with non-empty payload → `schema_payload_kind_mismatch`.
- Q7. tick_boundary with `payload_schema ∈ kPerKindAllowedSchemas
  [tick_boundary]` and matching size → ok. Parameterized over the
  8 allowed schemas.
- Q8. tick_boundary with `payload_schema ∈ {boot_descriptor, none}`
  → `schema_payload_kind_mismatch`. (none-with-bytes=0 is the one
  edge: a tick_boundary with literally no payload is malformed.)
- Q9. on_demand_request with `payload_schema == error_message_batch`
  → `schema_payload_kind_mismatch` (errors are pushed, not
  requested — pinned per the table).
- Q10. on_demand_request and on_demand_reply with their respective
  allowed sets → ok. Parameterized.
- Q11. The "every (kind, schema) cross-product" exhaustive
  parameterized test: walk every `envelope_kind × schema_id` pair;
  if the pair is in `kPerKindAllowedSchemas`, validator returns
  ok; otherwise it returns `schema_payload_kind_mismatch`.
  Catches future drift between the table and the validator
  implementation.

(The previous draft's Q4 "first non-boot envelope in a session" and
Section R on_demand pairing are **transport-cycle scope** per fork
F4 and dropped from this plan.)

---

## Coverage gaps explicitly addressed in v2

Per round-1 reviewer findings:

- ✅ `payload_schema == none` only with `payload_bytes == 0` (D14/D15).
- ✅ `protocol_version == 0` rejected (D3).
- ✅ `direction` zero-value rejected (D7).
- ✅ `envelope_kind` zero-value rejected (D5).
- ✅ `runtime_type` closed-enum test (M4).
- ✅ `sequence` boundary at 0 and UINT64_MAX (D12, D13).
- ✅ `hal_bool` typedef pinned (A5).
- ✅ `HAL_MatchType` backing pinned (decision #21, H7).
- ✅ `error_message::severity` type pinned (decision #22).
- ✅ Null-termination convention pinned (decision #23, L9).
- ✅ Reserved bytes round-trip-safe (C2, decision #24).
- ✅ Truncation helper signatures and tests (L1–L8, decision #20).
- ✅ ds_state with zero connected joysticks: covered by zero-init
  contract (G3-equivalent for ds_state) and `joystick_descriptor.
  type == 0` sentinel mapping to `kHALJoystickType_Unknown`. Test
  H8 plus a default-construct test pin this.
- ✅ Q4-the-test (first-message-in-session) and R3 (per-direction
  counter) **moved** to transport cycle per F4.

---

## Determinism notes

This feature is structural, not stochastic. No RNG, no wall-clock,
no concurrency. Every test runs in <1 ms on a single thread.
`operator== = default` on every payload struct is the determinism
contract these tests enforce. Replay-determinism tests across full
envelope streams fire from the future transport cycle, not here.

---

## References used

- `hal/src/main/native/include/hal/HALBase.h` — `HAL_GetFPGATime`,
  `HAL_GetSystemActive`, `HAL_GetBrownedOut`,
  `HAL_GetSystemTimeValid`, `HAL_GetCommsDisableCount`,
  `HAL_GetFPGAButton`, `HAL_GetRSLState`, `HAL_RuntimeType`. Source
  of clock_state and boot_descriptor fields.
- `hal/src/main/native/include/hal/DriverStation.h` and
  `DriverStationTypes.h` — `HAL_ControlWord`,
  `HAL_AllianceStationID`, `HAL_MatchInfo`, `HAL_MatchType`,
  `HAL_JoystickAxes`, `HAL_JoystickPOVs`, `HAL_JoystickButtons`,
  `HAL_JoystickDescriptor`, `HAL_kMaxJoysticks`,
  `HAL_kMaxJoystickAxes`, `HAL_kMaxJoystickPOVs`,
  `kHALJoystickType_Unknown`. Source of ds_state byte layout
  (decisions #8, #9, #21).
- `hal/src/main/native/include/hal/Notifier.h` — notifier handle
  and alarm semantics (decision #12).
- `hal/src/main/native/include/hal/Power.h` — `HAL_GetVinVoltage`,
  `HAL_GetVinCurrent`, `HAL_GetBrownoutVoltage`. Source of
  power_state fields.
- `hal/src/main/native/include/hal/CAN.h` — `HAL_CAN_SendMessage`,
  `HAL_CAN_ReceiveMessage`, `HAL_CAN_GetCANStatus`,
  `HAL_CAN_ReadStreamSession`, `HAL_CANStreamMessage`,
  `HAL_CAN_IS_FRAME_REMOTE`, `HAL_CAN_IS_FRAME_11BIT`. Source of
  can_frame layout (decision #11 / fork F3) and can_status field
  order.
- `hal/src/main/native/include/hal/Types.h` — `HAL_Bool`,
  `HAL_Handle`, `HAL_NotifierHandle`. Source of the typedef pins
  (A5, A6).
- `hal/src/main/native/include/hal/Errors.h` — error code
  conventions referenced by `error_message.error_code`.
- `frcture` DS docs:
  `https://frcture.readthedocs.io/en/latest/driverstation/{ds_to_rio,
  rio_to_ds}.html`. Confirms decision #10 (digested DS, not raw).
- `frcture` FPGA registers:
  `https://frcture.readthedocs.io/en/latest/fpga/registers.html`.
  Confirms decision #7 (64-bit timestamp is shipped expanded).
- `WL-Richards/RIOEmulator` — per-FPGA-subsystem decomposition
  pattern (decision #18).
- `docs/ARCHITECTURE.md` — process model, HAL boundary protocol,
  protocol-schema discipline, v0 scope, language toolchain.
- `.claude/skills/layer-2-control-system-backend.md` — scope and
  boundary, "Carrying forward from prior work", v0 status.
- `.claude/skills/code-style.md` — POD discipline, no heap in hot
  loops, fixed sizes, snake_case, no exceptions in hot path,
  `std::expected`.
- `.claude/skills/tdd-workflow.md` — test plan format, reviewer
  gate.
- `tests/description/TEST_PLAN.md` — format precedent.
