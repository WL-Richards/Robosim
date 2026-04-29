---
name: protocol-schema
description: Use when working on the HAL ↔ Sim Core protocol schema (POD payload structs, sync envelope, validator), the closed protocol enums, the truncation helpers, or any code that produces or consumes wire bytes for the protocol. Covers the public API, the 24 pinned design decisions, the WPILib byte-parity discipline, error semantics, schema evolution via kProtocolVersion bumps, and what is explicitly out of scope (transport, sync mechanics, the HAL shim itself).
---

# HAL ↔ Sim Core protocol schema

The schema cycle is the foundation Layer 2 builds on. It pins the
byte-exact wire layout of every v0 HAL state element and event, the
closed envelope/schema/direction/runtime enums that the validator
gates on, and the pure helpers (validator, truncation) that the
future shim and transport will consume. This cycle ships **only the
schema**; the shim, the transport (T1 shmem, T2 socket), the
LD_PRELOAD libc shim, and the Sim Core's HAL implementations each
have their own future TDD cycle.

## Scope

**In:**
- A versioned, magic-prefixed `sync_envelope` (32 bytes, fixed
  layout) that prefixes every HAL ↔ Sim Core exchange.
- POD payload structs covering the v0 HAL surface: TimedRobot tick,
  Notifier, FPGA timestamp, DS protocol (control word, alliance,
  match phase, joystick), CAN bus transport, power monitoring,
  error/log (`HAL_SendError`).
- Closed enums: `envelope_kind`, `direction`, `schema_id`,
  `runtime_type`, `validate_error_kind`. Each has a `reserved = 0`
  uninitialized-struct guard where applicable.
- Compile-time invariants (`static_assert`) on every payload struct:
  trivially copyable, standard layout, non-polymorphic, aggregate,
  and pinned `sizeof` / `alignof` for the wire-contract envelope and
  WPILib-mirrored structs.
- A pure stateless `validate_envelope` function that gates on magic,
  version, schema_id closed-enum, envelope_kind closed-enum,
  direction closed-enum, sequence equality, kind ↔ schema mapping,
  and payload-size consistency (including header-derived counts for
  variable-size batches).
- Pure truncation helpers (`copy_truncated`, `copy_bytes_truncated`)
  used to populate fixed-size string and byte buffers.
- A `kPerKindAllowedSchemas` table exposed to the validator and to
  the future transport-layer session, fully closed (no
  shim-cycle-deferred punts).

**Out (each is a separate future TDD cycle):**
- The HAL shim — produces/consumes these structs but is a separate
  feature.
- Sync mechanics (atomics + seqlock for T1, framing + reconnect for
  T2). The transport is wire-format-only at this stage.
- Transport/session state beyond the in-process `protocol_session`:
  T2 reconnection, transport reset, "first message after reconnect",
  and request/reply IDs if v0 single-flight on-demand requests become
  insufficient. The validator is fully stateless (decision #19 / fork
  F4); session-aware checks that do not require transport bytes now
  live in `.claude/skills/protocol-session.md`.
- The LD_PRELOAD libc shim and JVM-time interception.
- DS UDP packet parsing — the Sim Core composes a digested
  `ds_state` from a recorded trace or relayed real DS; raw UDP bytes
  never cross this protocol (decision #10).
- HAL surfaces deferred per v0: DIO, PWM, Analog, Counter, SPI, I²C,
  USB, Relay, pneumatics. Adding any of these is a `kProtocolVersion`
  bump (decision #5/#15).
- The `tools/rio-bench/` HAL-cost fixture.
- Send-side fidelity (the future shim's parameter-to-struct pack for
  e.g. `HAL_CAN_SendMessage`) — schema-cycle pins the layout, shim-
  cycle pins the faithfulness of the unpack.

## Public surface

`src/backend/common/`:

| Header                    | Purpose                                                                              |
|---------------------------|--------------------------------------------------------------------------------------|
| `protocol_version.h`      | `kProtocolVersion`, `kProtocolMagic`, `schema_id`, `envelope_kind`, `direction`, `runtime_type` enums; little-endian `static_assert`. |
| `types.h`                 | `hal_bool` = `uint32_t`, `hal_handle` = `int32_t` (mirror WPILib `HAL_Bool`/`HAL_Handle`). |
| `truncate.h`/`.cpp`       | `copy_truncated`, `copy_bytes_truncated`. Pure, no I/O, no allocation, overlap = UB.  |
| `sync_envelope.h`         | 32-byte wire-contract envelope.                                                      |
| `clock_state.h`           | Per-tick sim time + system flags. **No team_number** (fork F1 — moved to boot).       |
| `power_state.h`           | vin_v, vin_a, brownout_voltage_v.                                                    |
| `ds_state.h`              | `control_word`, `alliance_station`, `match_type`, `match_info`, joystick types, `ds_state`. All WPILib-mirrored byte-for-byte. |
| `can_frame.h`             | `can_frame` mirrors `HAL_CANStreamMessage` byte-for-byte (fork F3).                  |
| `can_status.h`            | Field order matches `HAL_CAN_GetCANStatus` parameter order.                          |
| `notifier_state.h`        | `notifier_slot`, `notifier_state`, `notifier_alarm_event`, `notifier_alarm_batch`.   |
| `error_message.h`         | `error_message`, truncation-flag bit constants, `error_message_batch`.               |
| `boot_descriptor.h`       | Boot-envelope payload: `runtime_type`, `team_number`, `vendor_capabilities`, `wpilib_version`. |
| `validator.h`/`.cpp`      | Pure stateless `validate_envelope`. Owns the `kPerKindAllowedSchemas` table.         |
| `validator_error.h`       | `validate_error_kind` enum, `validate_error` struct.                                 |

WPILib byte-parity is baselined at `kWpilibParitySha`
(`1fd159938f65ee9d0b22c8a369030b68e0efa82c`, WPILib v2026.2.2),
recorded in `tests/backend/common/wpilib_mirror.h`. The mirrors are
local transcriptions of WPILib's HAL types; if WPILib bumps and
changes a layout, the parity tests fail and the developer either
re-transcribes + bumps `kProtocolVersion`, or pins the older SHA.
Drift is a build break, never silent.

## The 24 pinned decisions (canonical reference)

The full rationale lives in `tests/backend/common/TEST_PLAN.md`'s
"Decisions to pin" section; this skill index reproduces the headlines
so future work can quote the right number.

1. **POD discipline is the contract**, enforced by `static_assert` in
   every header. Build-only TU `tests/backend/common/compile_time_invariants_test.cpp`
   ensures the asserts get checked.
2. **Fixed-size only** — no `std::string`, `std::vector`,
   `std::optional`, `std::variant`, raw pointers. Variable-count
   payloads use `<event>_batch` structs with explicit `count`.
3. **Pinned little-endian** on the wire.
4. **Magic bytes** (`{'R','S','S','1'}`) at offset 0 of every envelope.
5. **Schema version handshake is fatal**, not best-effort. Zero
   rejected as uninit guard.
6. **Sequence numbers monotonic, gapless, per-direction**, owned by
   the future `protocol_session`. The validator is stateless and
   takes `expected_sequence` as a parameter.
7. **Sim-time timestamps in `uint64_t` microseconds**, anchored at
   sim boot = 0. Matches `HAL_GetFPGATime`.
8. **WPILib HAL constants mirrored exactly**: `kMaxJoysticks`,
   `kMaxJoystickAxes`, `kMaxJoystickPOVs`, `kJoystickNameLen`, etc.
9. **WPILib structs mirrored byte-for-byte**: joystick types,
   control_word bitfield positions, match_info, can_frame.
10. **DS state is digested, not raw UDP bytes.**
11. **CAN frame layout = `HAL_CANStreamMessage` byte-for-byte** (fork
    F3). Send path is shim-cycle scope.
12. **Notifier alarms are events**, not polled state.
13. **Notifier table fixed at 32**.
14. **Error/log truncation is silent with a flag bit** (fork F2 = A);
    truncation is performed by the pure `copy_truncated` helper.
15. **`schema_id` enum is closed at v0**; adding one bumps
    `kProtocolVersion`. `none` is valid only with `payload_bytes ==
    0`.
16. **`runtime_type` is closed; v0 accepts only `roborio_2`.**
17. **`operator== = default` on every payload struct.**
18. **One header per HAL subsystem** (RIOEmulator's per-FPGA-subsystem
    decomposition).
19. **Validator is fully stateless** (fork F4).
20. **String-truncation helpers ship in this cycle** (fork F2 = A).
    Overlap = UB (memcpy semantics).
21. **`HAL_MatchType` backing is `int32_t`** (verified by mirror cast).
22. **`error_message::severity` is `hal_bool`**, mirroring
    `HAL_SendError`'s `HAL_Bool isError`.
23. **Null-termination convention**: `char[N]` fields are null-
    terminated within the buffer; `uint8_t[N]` byte buffers are not
    (they carry an explicit length companion field).
24. **Reserved bytes: zero on send, ignored on receive**, matching
    TCP/IP header reserved-field forward-compat semantics.

## Decisions resolved by user forks

Recorded in TEST_PLAN.md "Pre-review shape questions and post-round-1
forks":

- **Q1.** `sync_envelope` size = **32 bytes** (no headroom; protocol-
  version bump is the loud signal).
- **Q2.** Validator shape = **free function** (stateless).
- **Q3.** `error_message` truncation = **silent with flag** (option
  A; spill is a v1+ additive if a real caller needs it).
- **Q4 / F1.** `team_number` placement = **`boot_descriptor`** (flipped
  from per-tick `clock_state` after round-1 reviewer's
  silent-corruption-attractor argument).
- **F2.** Truncation-helper scope = **schema cycle ships them**.
- **F3.** CAN frame parity = **mirror `HAL_CANStreamMessage`
  byte-for-byte**.
- **F4.** Validator = **fully stateless**; session-aware checks move
  to the future transport cycle.

## Error semantics

`validate_error` carries:
- `kind` — one of the ten `validate_error_kind` values
  (`magic_mismatch`, `version_mismatch`, `unknown_schema_id`,
  `unknown_envelope_kind`, `unknown_direction`,
  `schema_payload_kind_mismatch`, `payload_size_mismatch`,
  `sequence_mismatch`, `direction_mismatch`, `unsupported_runtime`).
  Tests pin on this, never on message phrasing.
- `offending_field_name` — substring-asserted by tests (e.g.
  `"protocol_version"`). Substring lets prose tweaks land without
  breaking tests; substring failure means the wrong field was
  reported.
- `message` — human-readable; tests do not pin its exact phrasing.

## Schema evolution

`kProtocolVersion` is pinned at the start of every envelope. The gate
is fatal at the validator (decision #5). When v2 lands:

- Bump `kProtocolVersion`.
- Add new `schema_id` values (additive; never reorder the existing
  closed enum).
- Add new `envelope_kind` values likewise.
- Add new fields to existing payload structs only if you understand
  the layout consequences — every additive forces a `kProtocolVersion`
  bump because the wire size changes.
- Update `kPerKindAllowedSchemas` if the kind ↔ schema mapping
  changes.
- Update WPILib `kWpilibParitySha` if the new HAL year requires it,
  and re-transcribe affected mirrors.

The deferred surfaces (DIO, PWM, Analog, Counter, SPI, I²C, USB,
Relay, pneumatics, `error_message_continuation` for spill, raw DS UDP
for FMS-bug repro) all land via this same protocol-version-bump
discipline.

## Validation status

Structural feature, not a physical model — no real-world quantity to
compare against. "Validation" here is:

- **106-test suite** in `tests/backend/common/{protocol_test,
  wpilib_parity_test}.cpp`, covering every section A–Q + M0 + O of
  the approved test plan.
- All 106 tests pass under `clang Debug`, `gcc Debug`, and
  `clang Debug + ASAN + UBSAN`.
- WPILib byte-parity baselined at v2026.2.2
  (`1fd159938f65ee9d0b22c8a369030b68e0efa82c`).

When changing the schema, the test plan is the source of truth. If a
test needs substantive change (new case, different assertion), re-run
the test-reviewer agent on the change before writing the new test
code. Drift between approved-plan and shipped-test defeats the gate.

## Known limits

- **No raw DS UDP fidelity.** `ds_state` is digested. v1+ may add an
  optional `ds_raw_packet` schema for FMS-bug reproduction.
- **`error_message` truncation is silent**, not spill. Long stack
  traces lose data past 1024 bytes; the truncation flag bit signals
  it.
- **Bitfield ABI assumption.** `control_word` parity assumes GCC and
  Clang on x86-64 + ARM Linux lay bitfields LSB-first. A future MSVC
  port would need re-validation; the H5 test will fail loudly and
  the porter must re-transcribe.
- **Validator decodes batch headers, not bodies.** Element-body
  validation is the receiver's job once the validator has confirmed
  framing. Tests pin this boundary explicitly (D17).
- **Reserved bytes ignored on receive.** Decision #24 documents the
  forward-compat policy; buggy senders that spam non-zero into
  reserved bytes are not caught here. Mitigation: C2-class tests
  catch correctly-written senders; protocol-version bumps surface
  meaning later.
- **WPILib parity SHA verification is manual today.** CI grep of
  `git ls-remote --exit-code` against the pinned SHA is a follow-up;
  the SHA constant in `wpilib_mirror.h` is the source of truth until
  then.

## Open follow-ups

- **CI grep step** verifying the WPILib parity SHA exists at
  `wpilibsuite/allwpilib`. Today it's a string in a header; CI should
  prove it's a real commit.
- **CI grep ban** on `std::string` / `std::vector` / `std::optional`
  / raw-pointer members in `src/backend/common/`. The
  `is_trivially_copyable_v` static_assert catches these indirectly,
  but a direct CI guard is cheap.
- **`error_message_continuation` schema** if a real caller exceeds
  1024-byte details/call_stack and the truncation_flags bit gets
  hit in production logs.
- **Raw-DS-UDP schema** when FMS-bug-reproduction work needs raw-byte
  fidelity.

## Cross-references

- `tests/backend/common/TEST_PLAN.md` — the approved test plan
  (test-reviewer ratified, 2026-04-29, three rounds).
- `tests/backend/common/wpilib_mirror.h` — the WPILib byte-parity
  baseline + `kWpilibParitySha`.
- `docs/V0_PLAN.md` — the v0 implementation plan; this cycle is the
  one called out as "the protocol-schema POD structs come next."
- `docs/ARCHITECTURE.md` — process model, HAL boundary protocol
  ("Protocol schema (HAL ↔ Sim Core)" sub-section), v0 scope,
  language toolchain.
- `.claude/skills/layer-2-control-system-backend.md` — the layer
  this cycle instantiates the foundation for; future shim and
  transport cycles will consume the schema and validator from
  here.
- `.claude/skills/protocol-session.md` — stateful validator wrapper
  that consumes this schema and owns sequence counters, boot/ack
  ordering, on-demand pairing, and shutdown receive state.
- `.claude/skills/code-style.md` — POD discipline, no heap in hot
  loops, `std::expected`, snake_case.
- `.claude/skills/tdd-workflow.md` — review-gated test plan
  workflow.
- `.claude/skills/robot-description-loader.md` — format precedent
  this cycle followed (typed `<error>_kind` enums,
  `operator== = default`, decisions enumerated, etc.).
