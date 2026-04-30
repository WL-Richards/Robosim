# Sim core status

The sim-core implementation track: scaffold, robot description loader,
HAL ↔ Sim Core protocol, Tier 1 shared-memory transport, and the
in-process HAL shim core. See `docs/V0_PLAN.md` for the plan and
`.claude/skills/` for per-feature actionable knowledge.

## Headline

- Full project baseline through cycle 24: **ctest 532/532 green** under
  `build`. Cycle 24 focused shim suite: **217/217 green**.
- HAL shim through **cycle 24** (power_state read surface closed +
  clock_state read surface fully closed — all 7 fields wired; first
  two C HAL write surfaces landed; CAN stream RX and CAN status read
  landed; Driver Station scalar and joystick reads landed): v0 outbound surface closed for the
  three semantically-meaningful outbound schemas; the eight inbound
  per-tick payload schemas are all wired with latest-wins replacement;
  C HAL ABI surfaces wired so far: **`HAL_GetFPGATime`** (cycle 12),
  **`HAL_GetVinVoltage`** (cycle 13), **`HAL_GetVinCurrent`** (cycle
  14), **`HAL_GetBrownedOut`** (cycle 15), **`HAL_GetBrownoutVoltage`**
  (cycle 16), **`HAL_GetSystemActive`** /
  **`HAL_GetSystemTimeValid`** / **`HAL_GetFPGAButton`** /
  **`HAL_GetRSLState`** (cycle 17), **`HAL_GetCommsDisableCount`**
  (cycle 18), **`HAL_SendError`** (cycle 19), and
  **`HAL_CAN_SendMessage`** (cycle 20), plus
  **`HAL_CAN_OpenStreamSession`** /
  **`HAL_CAN_ReadStreamSession`** /
  **`HAL_CAN_CloseStreamSession`** (cycle 21), and
  **`HAL_CAN_GetCANStatus`** (cycle 22), plus
  **`HAL_GetControlWord`** / **`HAL_GetAllianceStation`** /
  **`HAL_GetMatchTime`** (cycle 23), plus
  **`HAL_GetJoystickAxes`** / **`HAL_GetJoystickPOVs`** /
  **`HAL_GetJoystickButtons`** (cycle 24). The three
  power_state float readers widen float→double via
  D-C13-FLOAT-TO-DOUBLE-CAST. With cycle 16 the **power_state read
  surface is closed** — all three float fields wired.
  Cycle 15 also fixed a pre-existing HAL_Bool signedness parity bug
  (D-C15-HAL-BOOL-SIGNED-PARITY): `hal_bool` is now `int32_t`
  matching WPILib's `HAL_Bool` byte-for-byte AND signedness-for-
  signedness. `hal_c.h` exports `typedef int32_t HAL_Bool;` per
  D-C15-HAL-BOOL-TYPEDEF-IN-HAL-C-H. Plus the process-global
  `shim_core::install_global` / `shim_core::current` accessor (storage
  in `hal_c.cpp` per D-C12-STORAGE-IN-HAL-C-CPP).
- Layer 3/4/5 work has not started.

## Phase A — scaffold

Landed.

## Phase B — robot description loader

`descriptions/<robot>.json` loader implemented; **83 tests green**.
Schema v2 with kinematic-vs-visual frame distinction and the
`description::compose_origin` / `decompose_origin` algebraic-inverse
pair (URDF intrinsic-Z-Y′-X″, with a gimbal-pole branch that pins
yaw=0 and absorbs residual into roll while preserving the composed
transform). See `.claude/skills/robot-description-loader.md`.

## Layer 2 — HAL ↔ Sim Core protocol

### Protocol schema (foundation)
32-byte sync envelope, 9 closed payload schemas, stateless validator,
truncation helpers, WPILib byte-parity baselined at v2026.2.2.
**+106 tests** (suite total). See `tests/backend/common/TEST_PLAN.md`
and `.claude/skills/protocol-schema.md`.

### Protocol session (stateful wrapper)
Send/receive sequence counters, boot/boot_ack ordering, on-demand
request/reply pairing, shutdown terminal receive state. **+16 tests.**
See `.claude/skills/protocol-session.md`.

### Tier 1 shared-memory transport
Fixed two-lane region, nonblocking endpoint, protocol-session
integration, Linux memfd/mmap lifecycle. **+24 tests.**
See `.claude/skills/tier1-shared-memory-transport.md`.

## HAL shim core

In-process shim built up across cycles. See `.claude/skills/hal-shim.md`
and `tests/backend/shim/TEST_PLAN_CYCLE{2..11}.md`.

### Cycles 2–8 — inbound surface

Boot handshake plus inbound cache slots for every per-tick payload
schema: `clock_state`, `power_state`, `ds_state`, `can_frame_batch`,
`can_status`, `notifier_state`, `notifier_alarm_batch`,
`error_message_batch`. Each slot is independently maintained with
latest-wins replacement.

The four variable-size schemas (`can_frame_batch`, `notifier_state`,
`notifier_alarm_batch`, `error_message_batch`) use active-prefix
memcpy with zero-init-before-write so a shrinking batch leaves unused
element slots byte-zero. Zero-init also covers the 4-byte interior
`count → events` / `count → slots` implicit padding word in the two
notifier schemas. `error_message_batch` has zero implicit padding
because both `reserved_pad[4]` (between `count` and `messages`) and
`error_message::reserved_pad[3]` (after `truncation_flags`) are
*named* fields covered by defaulted `operator==`.

Other cycle-8 properties:
- Shutdown terminal receive path; single-threaded `poll()`-driven.
- The `unsupported_payload_schema` arm is unreachable from valid
  traffic but retained as a defensive forward-compat structural
  guard (D-C8-DEAD-BRANCH).
- Padding-byte determinism for `ds_state`, `can_frame_batch`,
  `notifier_state`, and `notifier_alarm_batch` pinned by direct
  `std::memcmp`. C8-6 also closed a pre-existing gap where the
  landed C7-6 implementation had silently omitted
  `notifier_alarm_batch` from the determinism replay despite its
  name "AllSevenSlots".

CAN RX queueing semantics are deferred per **D-C4-LATEST-WINS**
until the cycle that wires `HAL_CAN_ReadStreamSession`.

### Cycle 9 — first outbound-past-boot surface

`send_can_frame_batch(const can_frame_batch&, uint64_t sim_time_us)`
publishes a `can_frame_batch` payload as a `tick_boundary` envelope
into `backend_to_core`. Promotes the shim from
"send-only-once-at-construction" to a working duplex.

Design decisions introduced:
- **D-C9-NO-HELPER** — inline the active-prefix size computation
  (`offsetof(can_frame_batch, frames) + batch.count *
  sizeof(can_frame)`); outbound mirror of inbound's
  D-C4-VARIABLE-SIZE active-prefix discipline.
- **D-C9-SHUTDOWN-TERMINAL** — same shutdown-terminal short-circuit
  as `poll()` applies on the outbound path.
- **D-C9-TYPED-OUTBOUND** — typed-per-schema methods, so the five
  sim-authoritative schemas (`clock_state`, `power_state`, `ds_state`,
  `can_status`, `notifier_alarm_batch`) are intentionally absent
  from the outbound surface — wrong-direction publishing is
  impossible at the API boundary.
- **D-C9-NO-CONNECT-GATE** — outbound does not gate on
  `is_connected()`, matching the protocol's
  bidirectional-handshake-overlap allowance.
- **D-C9-WRAPPED-SEND-ERROR** — the shared `wrap_send_error` lambda
  was generalized from "transport rejected outbound boot envelope"
  to "transport rejected outbound envelope" so it serves both call
  sites.

**+10 tests** (the new `ShimCoreSend` suite plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundCanFrameBatch`).

### Cycle 10 — second outbound-meaningful schema

`send_notifier_state(const notifier_state&, uint64_t sim_time_us)`
publishes `notifier_state` as a `tick_boundary` envelope. The
schema's 8-byte header offset (vs cycle 9's 4-byte) and 132 implicit
padding bytes — the most demanding of any outbound v0 schema — made
it the natural follow-on.

Design decisions introduced:
- **D-C10-EXTRACT-ACTIVE-PREFIX** — cashes in cycle 9's deferred
  D-C9-NO-HELPER trigger by extracting `active_prefix_bytes`
  overloads into the schema headers
  (`src/backend/common/can_frame.h` for `can_frame_batch` and
  `src/backend/common/notifier_state.h` for `notifier_state`). Both
  `send_can_frame_batch` (refactored) and `send_notifier_state` (new)
  call the production helpers; the duplicated test-side overloads in
  `tests/backend/tier1/test_helpers.h` were deleted, and tests
  resolve to the production version via ADL.
- **D-C10-{INBOUND-INDEPENDENCE / NO-CONNECT-GATE /
  RECEIVE-COUNTER-INDEPENDENCE}-INHERITS** — formalizes the
  per-method-test trim convention. Cycle-9 cross-cutting contracts
  are session/transport-level and don't need re-verification per
  outbound schema.
- **D-C10-SHUTDOWN-TERMINAL-INHERITS** — only the shutdown
  short-circuit needs per-method coverage; pinned by C10-5.

**+6 tests** (5 in `ShimCoreSend` plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundNotifierState`).

### Cycle 11 — third (and final v0) outbound-meaningful schema

`send_error_message_batch(const error_message_batch&, uint64_t
sim_time_us)` publishes `error_message_batch` as a `tick_boundary`
envelope. Extends D-C10-EXTRACT-ACTIVE-PREFIX by adding the third
production `active_prefix_bytes` overload to
`src/backend/common/error_message.h` plus a
`static_assert(offsetof(error_message_batch, messages) == 8)` that
pins the named-`reserved_pad[4]`-derived header layout.

The schema is the only outbound v0 schema with **zero implicit C++
padding** (D-C8-PADDING-FREE inherited from cycle 8 — both
`reserved_pad` fields are NAMED), so cycle 11's memcmp companions
are byte-equivalent to `vector::operator==` but kept for cross-cycle
structural parity (resolution OQ-C11-MEMCMP-IN-DETERMINISM).

**No new design decisions** — every contract inherits from cycles 9
and 10, and the per-method-test trim convention applies unchanged.

With cycle 11, the **v0 outbound surface is closed** for the three
semantically-meaningful outbound schemas; the five sim-authoritative
schemas remain intentionally absent per D-C9-TYPED-OUTBOUND.

**+6 tests** (5 in `ShimCoreSend` plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutboundErrorMessageBatch`).

### Cycle 12 — first C HAL ABI surface

New TU `src/backend/shim/hal_c.{h,cpp}` carrying the project's first
`extern "C"` HAL symbol: `HAL_GetFPGATime(int32_t* status)` reading
from `latest_clock_state_->sim_time_us`. New process-global accessor
`shim_core::install_global` / `shim_core::current` (declarations on
`shim_core` in `shim_core.h`; definitions and TU-static
`g_installed_shim_` storage in `hal_c.cpp` per
**D-C12-STORAGE-IN-HAL-C-CPP**).

Status-code semantics (mirroring WPILib HAL/Errors.h):
- no shim installed → `*status = kHalHandleError`, return 0
  (**D-C12-NULL-SHIM-IS-HANDLE-ERROR**).
- shim + empty cache → `*status = kHalSuccess`, return 0. Matches
  WPILib's "always succeeds" contract and the sim's "sim_time
  starts at 0" model (**D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO**;
  the empty-cache 0-return is intentionally indistinguishable
  from a populated-cache `sim_time_us == 0`).
- shim + populated cache → `*status = kHalSuccess`, return cached
  `sim_time_us`. `*status` is overwritten on every call regardless
  of prior value (**D-C12-STATUS-WRITE-UNCONDITIONAL**).

NULL `status` is UB matching WPILib's header contract. Storage is
non-atomic (single-threaded v0); the threading cycle promotes to
`std::atomic<shim_core*>`.

**+8 tests** across two new suites (`ShimCoreCurrent` and
`HalGetFPGATime`). Test plan landed `ready-to-implement` after three
review rounds.

### Cycle 13 — HAL_GetVinVoltage

The second C HAL ABI surface. `extern "C" double
HAL_GetVinVoltage(int32_t* status)` reads `latest_power_state_->vin_v`
and widens float→double at the C ABI seam. Pure mirror of cycle 12's
shape applied to a different cache slot, with one new design decision:

- **D-C13-FLOAT-TO-DOUBLE-CAST.** WPILib's HAL_GetVinVoltage returns
  `double`; the schema's `power_state::vin_v` is `float`. The C ABI
  does an explicit `static_cast<double>(latest_power_state()->vin_v)`.
  IEEE-754 float→double widening is loss-free for every finite
  single-precision value (the double mantissa has 52 bits vs.
  float's 23; every representable float is also exactly representable
  as double). Future float-returning HAL_* surfaces inherit this
  decision.

All other contracts inherit unchanged from cycle 12 (D-C12-NULL-SHIM-
IS-HANDLE-ERROR, D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO per-function
variant for `power_state`, D-C12-STATUS-WRITE-UNCONDITIONAL,
D-C12-LATEST-WINS-READ).

**+4 tests** in the new `HalGetVinVoltage` suite (test count trimmed
from cycle 12's 8 because the install/clear and idempotency contracts
are session-level, pinned by cycle 12, and don't need re-verification
per HAL_* function — same trim convention as cycle 10 outbound
mirrors). Test plan landed `ready-to-implement` in a single review
round, no blockers, no required changes.

### Cycle 14 — HAL_GetVinCurrent

Pure mirror of cycle 13. `extern "C" double
HAL_GetVinCurrent(int32_t* status)` reads `latest_power_state_->vin_a`
and applies the same float→double widening at the C ABI seam. **No
new design decisions** — every contract inherits from cycles 12
and 13.

The cycle exists to validate that **D-C13-FLOAT-TO-DOUBLE-CAST
generalizes**: the same widening cast pattern works for a second
consumer reading a sibling field in the same cache slot. The
wrong-field bug class (reading `vin_v` instead of `vin_a`) is
per-function and gets per-function coverage.

**+4 tests** in the new `HalGetVinCurrent` suite. C14-3 fixture
values `7.5 / 42.25 / 6.5` are deliberately different from cycle
13's `12.5 / 99.25 / 6.75` to catch a "copy-pasted
HAL_GetVinVoltage's body without changing the field" bug. Test
plan landed `ready-to-implement` in a single review round; one
plan-text correction (a muddled parenthetical) addressed in
revision 2.

### Cycle 18 — HAL_GetCommsDisableCount

The final clock_state reader. `extern "C" int32_t
HAL_GetCommsDisableCount(int32_t* status)` reads
`latest_clock_state_->comms_disable_count` and casts at the C ABI
seam per **D-C18-UINT32-TO-INT32-CAST** (schema is `uint32_t`,
WPILib returns `int32_t`; cast is identity for values ≤ INT32_MAX
and C++20 modular wraparound for larger values). The schema's
unsigned type is preserved because its `UINT32_MAX` round-trip
boundary test is load-bearing for the wire-format contract; the
cast lives at the shim seam, not the schema.

**+5 tests** (one more than cycles 13/14/16's 4 because the
cast-wraparound seam warrants its own dedicated test):
- C18-1 / C18-2: standard null-shim and empty-cache paths.
- C18-3: in-range read pinning field-read correctness
  (`comms_disable_count = 42`, assert `n == 42`).
- C18-4: dedicated wraparound test
  (`comms_disable_count = 0xCAFEBABE > INT32_MAX`, assert
  `n == static_cast<int32_t>(std::uint32_t{0xCAFEBABEu})`).
- C18-5: latest-wins across two updates.

With cycle 18 the **clock_state read surface is fully closed**:
all seven fields are wired across cycles 12 (`HAL_GetFPGATime`),
15 (`HAL_GetBrownedOut`), 17 (4 HAL_Bool readers), and 18.

D-C18-UINT32-TO-INT32-CAST is endorsed as the standing decision
for any future `int32_t`-returning C HAL surface that reads a
`uint32_t` schema field. Test plan landed `ready-to-implement`
after two review rounds.

### Cycle 17 — clock_state HAL_Bool readers + helper lift (first bundled cycle)

The first **explicitly bundled cycle** in the C HAL ABI series.
Lifts a shared `clock_state_hal_bool_read(int32_t*, hal_bool
clock_state::*)` pointer-to-member helper in `hal_c.cpp`'s
`robosim::backend::shim::{anonymous}` namespace, refactors the
existing `HAL_GetBrownedOut` (cycle 15) to call the helper, and adds
**four new HAL_Bool readers** that share the same shape:

- `HAL_GetSystemActive` → `latest_clock_state_->system_active`
- `HAL_GetSystemTimeValid` → `latest_clock_state_->system_time_valid`
- `HAL_GetFPGAButton` → `latest_clock_state_->fpga_button_latched`
- `HAL_GetRSLState` → `latest_clock_state_->rsl_state`

Each new wrapper is a genuine 1-line `return
clock_state_hal_bool_read(status, &clock_state::field);` body. Two
file-scope `using` declarations (`using
robosim::backend::clock_state;` and `using
robosim::backend::shim::clock_state_hal_bool_read;`) bridge the
helper from the named namespace into `extern "C"`'s name lookup.

**Three new design decisions:**

- **D-C17-CLOCK-STATE-HAL-BOOL-HELPER.** The helper exists, lives
  in the named-namespace anonymous block (parallel to
  `g_installed_shim_`), encapsulates the cycle-12 null-shim /
  empty-cache / status-write / latest-wins contracts so the five
  callers inherit them without per-function code repetition.
- **D-C17-PER-WRAPPER-WRONG-FIELD-COVERAGE.** Each new wrapper
  gets one positive test that pins the correct member-pointer is
  passed. Null-shim / empty-cache / latest-wins are session-level
  via the helper and inherited from cycle-15's HalGetBrownedOut
  tests.
- **D-C17-BUNDLED-SCOPE.** Bundling multiple HAL_* surfaces into a
  single cycle is justified when (1) zero new design decisions per
  surface, (2) helper-lift trigger is real, (3) per-wrapper coverage
  is preserved, (4) bundling matches the natural granularity of the
  helper's scope. Explicitly NOT a precedent for outbound-buffering
  (HAL_SendError, HAL_CAN_SendMessage), struct-out-param
  (HAL_GetControlWord), or other "new mechanic" cycles.

**+4 tests** in four new GoogleTest suites
(`HalGetSystemActive` / `HalGetSystemTimeValid` /
`HalGetFPGAButton` / `HalGetRSLState`), one positive test each.
Cycle-15's existing `HalGetBrownedOut` 4-test suite verifies the
refactor preserves behavior. With cycle 17, the **clock_state
HAL_Bool reader subset is closed** (5 of 5 fields wired). Test plan
landed `ready-to-implement` after two review rounds; round-1
required spelling out the helper's placement explicitly (helper
inside `robosim::backend::shim::{anonymous}` plus file-scope
`using` declarations was picked as Option B).

### Cycle 16 — HAL_GetBrownoutVoltage

The fifth C HAL ABI surface and the **third float→double reader on
`power_state`**, completing the power_state read surface. `extern "C"
double HAL_GetBrownoutVoltage(int32_t* status)` reads
`latest_power_state_->brownout_voltage_v` (the brownout *threshold*,
not the live measurement) and applies D-C13-FLOAT-TO-DOUBLE-CAST.
**No new design decisions** — pure mirror of cycles 13/14.

**+4 tests** in the new `HalGetBrownoutVoltage` suite. C16-3 fixture
values `14.5 / 2.5 / 6.875` are deliberately distinct from cycle 13's
`12.5 / 99.25 / 6.75` and cycle 14's `7.5 / 42.25 / 6.5` to catch
"copy-pasted from a previous cycle without updating the field" bugs
across all three float readers. Test plan landed
`ready-to-implement` in a single review round; one required change
(explicit positional spelling of the helper call to prevent
default-arg drift) addressed in revision 2.

OQ-C16-LIFTING-A-HELPER (5th similar function in `hal_c.cpp`)
resolved as "still don't lift" — but reviewer noted that a
**scoped pointer-to-member helper** for the three power_state float
readers would be one parameter, not the four-template-param helper
the plan dismissed; this nuance is captured for future cycles when
a 6th or 7th similar function lands.

### Cycle 15 — HAL_Bool parity fix + HAL_GetBrownedOut

The fourth C HAL ABI surface and the **first HAL_Bool-returning
one**. `extern "C" HAL_Bool HAL_GetBrownedOut(int32_t* status)`
reads `latest_clock_state_->browned_out`. Two new design decisions:

- **D-C15-HAL-BOOL-SIGNED-PARITY.** Pre-existing bug fix: our
  `using hal_bool = std::uint32_t;` did not match WPILib's
  `typedef int32_t HAL_Bool;` despite the comment claiming parity.
  Cycle 15 changes the typedef to `int32_t`. Wire format is byte-
  identical for valid 0/1 values (the only values used in the
  codebase); the C++ type contract now matches WPILib byte-for-byte
  AND signedness-for-signedness. The fix is bundled with cycle 15
  rather than split into a parity-only cycle because HAL_GetBrownedOut
  is the first surface that exposes `hal_bool` at the C ABI seam,
  where signedness becomes part of the public contract.
- **D-C15-HAL-BOOL-TYPEDEF-IN-HAL-C-H.** Exports
  `typedef int32_t HAL_Bool;` inside `hal_c.h`'s `extern "C"`
  block. Robot code reading `hal_c.h` from a C TU sees the same
  type name as WPILib. Future bool-returning HAL_* surfaces
  (HAL_GetSystemActive, HAL_GetFPGAButton, HAL_GetRSLState, etc.)
  inherit.

**+4 new tests** in the `HalGetBrownedOut` suite plus the
existing `Types.HalBoolIsUint32` runtime test renamed and inverted
to `Types.HalBoolIsInt32`. C15-3 explicitly zeroes all four
sibling `hal_bool` fields in the fixture so wrong-field bugs
on `system_active` / `system_time_valid` / `fpga_button_latched` /
`rsl_state` are caught directly. C15-4 inverts those siblings
across two updates so latest-wins + wrong-field bugs are caught
jointly. Test plan landed `ready-to-implement` after two review
rounds.

### Cycle 19 — HAL_SendError + outbound error buffering

The first C HAL ABI write surface. `extern "C" int32_t
HAL_SendError(...)` constructs an `error_message` at the C seam,
handles NULL string pointers as empty, truncates strings with
`copy_truncated`, sets per-field truncation flags, and appends the
message to a per-`shim_core` pending buffer.

The buffer lives on `shim_core` as
`std::array<error_message, kMaxErrorsPerBatch>` plus an active count.
`enqueue_error` drops new messages silently after 8. The integrator
flushes explicitly at tick boundary with `flush_pending_errors`, which
builds a zero-initialized `error_message_batch` and sends it through
the existing cycle-11 `send_error_message_batch` path. Successful flush
clears the buffer; transport failure retains it for retry; empty flush
is a success no-op; post-shutdown flush returns
`shutdown_already_observed` without touching lane or buffer.

**+11 tests** across `HalSendError`, `ShimCoreEnqueueError`, and
`ShimCoreFlushPendingErrors`. The plan reached final narrow
`test-reviewer` verdict `ready` after three rounds; the important
round-2 fix was whole-batch `operator==` coverage so
`error_message_batch::reserved_pad[4]` is pinned zero on the wire.

### Cycle 20 — HAL_CAN_SendMessage + outbound CAN buffering

The second C HAL ABI write surface. `extern "C" void
HAL_CAN_SendMessage(...)` constructs a `can_frame` at the C seam,
preserves the caller's `messageID`, copies 0..8 payload bytes, and
appends the frame to a per-`shim_core` pending CAN buffer.

The buffer lives on `shim_core` as
`std::array<can_frame, kMaxCanFramesPerBatch>` plus an active count.
`enqueue_can_frame` drops new frames silently after 64. The integrator
flushes explicitly at tick boundary with `flush_pending_can_frames`,
which builds a zero-initialized `can_frame_batch`, stamps active frames
with `static_cast<uint32_t>(sim_time_us)`, and sends it through the
existing cycle-9 `send_can_frame_batch` path. Successful flush clears
the buffer; transport failure retains it for retry; empty flush is a
success no-op; post-shutdown flush returns `shutdown_already_observed`
without touching lane or buffer.

CAN-specific input rules are pinned at the C seam: `dataSize > 8`,
`data == nullptr && dataSize > 0`, and negative `periodMs` values other
than stop-repeat produce `HAL_ERR_CANSessionMux_InvalidBuffer` with no
enqueue. `data == nullptr && dataSize == 0` is a zero-length frame.
`periodMs >= 0` enqueues one immediate v0 frame; stop-repeat (`-1`)
succeeds without enqueueing.

**+15 tests** across `HalCanSendMessage`, `ShimCoreEnqueueCanFrame`,
and `ShimCoreFlushPendingCanFrames`. The plan reached
`ready-to-implement` after two `test-reviewer` rounds.

### Cycle 21 — HAL_CAN_ReadStreamSession + CAN RX queues

The CAN stream read surface. `HAL_CAN_OpenStreamSession` allocates a
per-`shim_core` nonzero stream handle with a message ID/mask filter and
caller-provided queue depth. `HAL_CAN_ReadStreamSession` drains queued
frames into `HAL_CANStreamMessage` output buffers, and
`HAL_CAN_CloseStreamSession` invalidates the handle.

Inbound `can_frame_batch` handling still updates `latest_can_frame_batch_`
exactly as cycle 4 specified, but now also appends matching frames to
each open stream's FIFO queue. Matching uses masked equality:
`(frame.message_id & mask) == (message_id & mask)`. A mask of `0`
matches all frames and preserves original IDs/flags in the output.
Frames received before a stream opens are not backfilled.

Per-stream overflow keeps the newest `maxMessages` frames, drops the
oldest, and reports `HAL_ERR_CANSessionMux_SessionOverrun` once on the
next read while returning the surviving frames. Valid empty reads return
`HAL_WARN_CANSessionMux_NoToken`; invalid/closed handles return
`HAL_ERR_CANSessionMux_NotAllowed`; null output buffers with nonzero
read count return `HAL_ERR_CANSessionMux_InvalidBuffer` without draining.

**+17 tests** across `HalCanOpenStreamSession`,
`HalCanReadStreamSession`, and `HalCanCloseStreamSession`. The plan
reached `ready-to-implement` after two `test-reviewer` rounds. The
reviewer noted that empty-read and overflow statuses are explicit v0 shim
decisions pending future RIO parity fixture validation.

### Cycle 22 — HAL_CAN_GetCANStatus

The CAN status read surface. `HAL_CAN_GetCANStatus` reads the latest
cached `can_status` and writes WPILib's five out-parameters in official
header order: `percentBusUtilization`, `busOffCount`, `txFullCount`,
`receiveErrorCount`, and `transmitErrorCount`.

No-shim calls set `kHalHandleError` and zero all five outputs.
Installed shim with empty `latest_can_status_` sets success, zeroes all
outputs, and does not materialize a cache value. Populated-cache reads
copy all five fields exactly. Latest-wins is tested with interleaved
poll/read/poll/read calls so the wrapper must read the current shim cache
on each call.

**+4 tests** in `HalCanGetCanStatus`. The plan reached
`ready-to-implement` after two `test-reviewer` rounds.

### Cycle 23 — Driver Station scalar reads

The first Driver Station C HAL read surface. `HAL_GetControlWord`,
`HAL_GetAllianceStation`, and `HAL_GetMatchTime` read from the latest
cached `ds_state`.

No-shim calls return `kHalHandleError`/default values. Installed shim
with empty `latest_ds_state_` returns success/default values and does
not materialize a cache value. Populated-cache reads return
`control.bits`, `station`, and `match_time_seconds` respectively.
`HAL_GetControlWord` clears reserved bits and maps all six named WPILib
control-word bits; the test suite uses one-hot coverage for each named
bit so transpositions cannot pass. Latest-wins is tested with
interleaved poll/read/poll/read calls across all three readers.

**+10 tests** across `HalGetControlWord`, `HalGetAllianceStation`,
`HalGetMatchTime`, and `HalDriverStationScalarReads`. The plan reached
`ready-to-implement` after three `test-reviewer` rounds.

### Cycle 24 — Driver Station joystick reads

The joystick Driver Station C HAL read surface. `HAL_GetJoystickAxes`,
`HAL_GetJoystickPOVs`, and `HAL_GetJoystickButtons` read from
`latest_ds_state_` and copy byte-compatible WPILib ABI structs.

No-shim calls return `kHalHandleError` and zero the full output struct.
Installed shim with empty `latest_ds_state_` returns success, zeroes the
full output struct, and does not materialize a cache value. Populated
valid-index reads copy the requested joystick slot byte-for-byte,
including padding and array suffix bytes. Invalid indices (`< 0` or
`>= 6`) are an explicit v0 decision: return `kHalSuccess` with
zero/default output. Latest-wins is tested across all three readers with
full byte equality.

**+14 tests** across `HalJoystickStructLayout`, `HalGetJoystickAxes`,
`HalGetJoystickPOVs`, `HalGetJoystickButtons`, and `HalJoystickReads`.
The plan reached `ready-to-implement` after two `test-reviewer` rounds.

## What's next

The C HAL ABI is the **largest remaining shim chunk** and is now in
progress — cycles 12–24 wired the first read surfaces, the global shim
accessor, the first two write surfaces (`HAL_SendError` and
`HAL_CAN_SendMessage`), CAN stream RX, CAN status reads, and DS scalar
and joystick reads. Each
remaining surface is its own TDD cycle:
- HAL_Initialize / HAL_Shutdown (initialize-once semantics).
- HAL_GetMatchInfo / descriptor/all-joystick aggregate DS reads.
- HAL_Notifier* (outbound buffer → `send_notifier_state`).

Other shim concerns:
- Shim-initiated `shutdown`.
- On-demand request/reply.
- Threading model.

Beyond the shim:
- T1 wait/reset/named-discovery work.
- T2 socket transport.
- LD_PRELOAD libc shim.
- Layer 3 / 4 / 5 work.
