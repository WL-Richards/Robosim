# Sim core status

The sim-core implementation track: scaffold, robot description loader,
HAL ↔ Sim Core protocol, Tier 1 shared-memory transport, and the
in-process HAL shim core. See `docs/V0_PLAN.md` for the plan and
`.claude/skills/` for per-feature actionable knowledge.

## Headline

- Full project baseline through cycle 46: **ctest 729/729 green** under
  `build`. Cycle 46 focused common suite: **131/131 green**. Cycle 46
  focused shim suite: **405/405 green**.
- HAL shim through **cycle 46** (power_state read surface closed +
  clock_state read surface fully closed — all 7 fields wired; first
  two C HAL write surfaces landed; CAN stream RX and CAN status read
  landed; Driver Station scalar and joystick reads landed; Notifier
  control-plane C HAL surface started; Driver Station joystick outputs
  publish through explicit protocol v2 snapshots; user-program observer
  state publishes through explicit protocol v3 snapshots; Driver Station
  output-side snapshots have a one-message pump boundary): v0 outbound
  surface closed for the three semantically-meaningful outbound schemas;
  cycle 37 adds `joystick_output_batch`; cycle 38 adds
  `user_program_observer_snapshot`; the eight inbound
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
  **`HAL_GetJoystickButtons`** (cycle 24), plus
  **`HAL_InitializeNotifier`** / **`HAL_SetNotifierName`** /
  **`HAL_UpdateNotifierAlarm`** / **`HAL_CancelNotifierAlarm`** /
  **`HAL_StopNotifier`** / **`HAL_CleanNotifier`** (cycle 25), plus
  **`HAL_SetNotifierThreadPriority`** (cycle 26), plus
  **`HAL_WaitForNotifierAlarm`** (cycle 27), plus
  **`HAL_Initialize`** / **`HAL_Shutdown`** (cycle 28), plus
  **`HAL_GetMatchInfo`** / **`HAL_GetJoystickDescriptor`** /
  **`HAL_GetJoystickIsXbox`** / **`HAL_GetJoystickType`** /
  **`HAL_GetJoystickName`** / **`HAL_GetJoystickAxisType`** (cycle
  29), plus **`HAL_GetAllJoystickData`** (cycle 30), plus
  **`HAL_CAN_ReceiveMessage`** (cycle 31), plus
  **`HAL_RefreshDSData`** (cycle 32), plus
  **`HAL_ProvideNewDataEventHandle`** /
  **`HAL_RemoveNewDataEventHandle`** (cycle 33), plus
  **`HAL_GetOutputsEnabled`** (cycle 34), plus
  **`HAL_ObserveUserProgramStarting`** /
  **`HAL_ObserveUserProgramDisabled`** /
  **`HAL_ObserveUserProgramAutonomous`** /
  **`HAL_ObserveUserProgramTeleop`** /
  **`HAL_ObserveUserProgramTest`** (cycle 35), plus
  **`HAL_SetJoystickOutputs`** (cycle 36), plus explicit
  **`joystick_output_batch` publication** (cycle 37), plus explicit
  **`user_program_observer_snapshot` publication** (cycle 38), plus the
  **Driver Station output pump** (cycle 39), plus
  **`HAL_GetRuntimeType`** (cycle 40), plus
  **`HAL_GetTeamNumber`** and retained boot metadata (cycle 41), plus
  **`HAL_GetFPGAVersion`** / **`HAL_GetFPGARevision`** (cycle 42), plus
  **`HAL_GetSerialNumber`**, **`HAL_GetPort`** /
  **`HAL_GetPortWithModule`**, **`HAL_GetLastError`** /
  **`HAL_GetErrorMessage`**, **`HAL_Report`**, and
  **`HAL_GetComments`** (cycles 42–46 dirty-worktree metadata/support
  follow-ons). The three
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
32-byte sync envelope, 11 closed payload schemas, stateless validator,
truncation helpers, WPILib byte-parity baselined at v2026.2.2.
Cycle 37 bumped `kProtocolVersion` to 2 for `joystick_output_batch`;
cycle 38 bumped it to 3 for `user_program_observer_snapshot`.
**131 tests** (suite total). See `tests/backend/common/TEST_PLAN.md`
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

### Cycle 25 — Notifier control plane

The first Notifier C HAL slice. `HAL_InitializeNotifier`,
`HAL_SetNotifierName`, `HAL_UpdateNotifierAlarm`,
`HAL_CancelNotifierAlarm`, `HAL_StopNotifier`, and
`HAL_CleanNotifier` now own per-shim notifier handles and publish
compact `notifier_state` snapshots through explicit
`flush_notifier_state(sim_time_us)`.

Handles are nonzero, monotonic, and not reused in v0; handle `0`,
unknown handles, and cleaned handles report `kHalHandleError` from
status-writing functions. The 32-slot table boundary is explicit: the
33rd active allocation returns handle `0` with `kHalHandleError`. Names
are zero-filled, null names become empty, and overlong names copy at
most 63 bytes to preserve a trailing NUL. Update activates an alarm;
cancel and stop clear `trigger_time_us`, set `alarm_active = 0`, and
mark `canceled = 1`. `HAL_CleanNotifier` is a no-op when no shim is
installed and removes live slots otherwise. Empty notifier-table flushes
publish a header-only snapshot rather than no-oping, so Sim Core can
observe that all notifiers were removed. `HAL_WaitForNotifierAlarm`
remained deferred to cycle 27.

**+18 tests** across `HalInitializeNotifier`, `HalSetNotifierName`,
`HalUpdateNotifierAlarm`, `HalCancelNotifierAlarm`, `HalStopNotifier`,
`HalCleanNotifier`, `ShimCoreFlushNotifierState`, and
`ShimCoreDeterminism`. The plan reached `ready-to-implement` after two
`test-reviewer` rounds.

### Cycle 26 — Notifier thread priority

`HAL_SetNotifierThreadPriority` is now present as a v0 deterministic
no-op. With no shim installed it returns false and writes
`kHalHandleError`; with a shim installed it accepts all `realTime` and
`priority` inputs, returns true, and writes `kHalSuccess`. The call does
not allocate notifiers, mutate `notifier_state`, or publish outbound
traffic. Real scheduler policy, priority validation, and threading
effects remain deferred to the future threading model.

**+5 tests** in `HalSetNotifierThreadPriority`. The plan reached
`ready-to-implement` in one `test-reviewer` round.

### Cycle 27 — Notifier wait

`HAL_WaitForNotifierAlarm` now has real v0 wake semantics instead of an
empty nonblocking placeholder. With no shim or an invalid/cleaned handle
it returns `0` and writes `kHalHandleError`. For a valid active handle
it waits until a matching inbound `notifier_alarm_batch` event is
available, `HAL_StopNotifier` wakes the handle, or `HAL_CleanNotifier`
invalidates it. Alarm events are appended during `shim_core::poll()` to
a per-shim FIFO wait queue, drained by matching handle without mutating
the existing latest-wins `latest_notifier_alarm_batch()` observer cache.
`HAL_StopNotifier` wakes pending waits with `0/kHalSuccess`;
`HAL_CleanNotifier` wakes them with `0/kHalHandleError`;
`HAL_CancelNotifierAlarm` updates control-plane state but does not wake
waits. Queue capacity/overflow and multi-waiter behavior remain
explicitly deferred.

Cycle 27 is the first shim cycle to add synchronized state around a HAL
wait path: notifier records and pending wait events are protected by the
per-shim notifier wait mutex/condition variable so concurrent
`poll()`/wait/stop/clean/cancel paths are data-race-free for this
surface.

**+10 tests** in `HalWaitForNotifierAlarm`. The plan reached
`ready-to-implement` after three `test-reviewer` rounds.

### Cycle 28 — HAL lifecycle

`HAL_Initialize(timeout, mode)` and `HAL_Shutdown()` are now wired as
the first process lifecycle C HAL surface. v0 keeps shim ownership with
the host harness: `HAL_Initialize` does not create transport or boot a
new `shim_core`; it returns false when no shim is installed and true
when `shim_core::install_global(&shim)` already points at a live shim.
`timeout` and `mode` are accepted but ignored in v0, including
`INT32_MIN` / `INT32_MAX`. Repeated and concurrent initialize calls are
idempotent read-only operations against an already installed shim.

`HAL_Shutdown` is a no-op without an installed shim. With an installed
shim it wakes pending HAL waits, then clears the process-global current
shim pointer without destroying or clearing the caller-owned shim
object. Pending `HAL_WaitForNotifierAlarm` calls wake with
`0/kHalHandleError` on shutdown. The host may reinstall the same shim
after shutdown and call `HAL_Initialize` again; cycle 28 intentionally
does not add persistent process poison state. A small diagnostic
`pending_notifier_wait_count` observer was added to make wait
registration deterministic in tests without sleeps.

**+9 tests** across `HalInitialize` and `HalShutdown`. The plan reached
`ready-to-implement` after two `test-reviewer` rounds.

### Cycle 29 — DS match info and joystick descriptors

The remaining per-slot Driver Station metadata reads now come from
`latest_ds_state_`. `HAL_GetMatchInfo` copies `ds_state::match`
byte-for-byte into `HAL_MatchInfo`; no installed shim returns
`kHalHandleError` and zeroes the output, while an installed shim with an
empty DS cache returns `kHalSuccess` and zeroes the output.

`HAL_GetJoystickDescriptor` copies valid joystick descriptor slots
byte-for-byte. Invalid joystick indices return `kHalSuccess` with a
zero/default descriptor, matching the cycle-24 invalid-index convention
and WPILib's documented descriptor default-fill behavior. The scalar
descriptor helpers `HAL_GetJoystickIsXbox`, `HAL_GetJoystickType`, and
`HAL_GetJoystickAxisType` have no status channel, so no-shim,
empty-cache, invalid joystick, and invalid axis paths return zero.
`HAL_GetJoystickName` fills `WPI_String { const char* str, size_t len }`
with heap-owned bytes copied from the fixed descriptor name field up to
the first NUL, or all 256 bytes if no NUL is present; unavailable or
empty names return `{nullptr, 0}`.

**+15 tests** across `HalGetMatchInfo`, `HalGetJoystickDescriptor`,
`HalJoystickDescriptorScalars`, `HalGetJoystickName`, and
`HalDriverStationMetadataReads`. The first test-reviewer pass was
`not-ready` for missing scalar-helper no-shim/empty/latest coverage; the
revised plan reached `ready-to-implement`.

### Cycle 30 — all joystick aggregate read

`HAL_GetAllJoystickData` now copies all six axes, POV, and button slots
from `latest_ds_state_` in one void C HAL call. No installed shim or an
installed shim with an empty DS cache zeroes all three output arrays.
Populated-cache reads copy every fixed ABI struct byte-for-byte and
match the per-slot Cycle 24 readers for the same cache.

**+5 tests** in `HalGetAllJoystickData`. The plan reached
`ready-to-implement` in the Cycle 29/30 reviewer follow-up.

### Cycle 31 — HAL_CAN_ReceiveMessage

`HAL_CAN_ReceiveMessage` is now wired against the older WPILib
`hal/CAN.h` signature already used by this repo's stream-session ABI:
`uint32_t* messageID`, `uint32_t messageIDMask`, `uint8_t* data`,
`uint8_t* dataSize`, `uint32_t* timeStamp`, `int32_t* status`. The
signature was re-confirmed against WPILib primary source before the
cycle because generated newer docs show a bus-ID based API.

v0 treats `messageID` as in/out: callers provide the requested ID in
`*messageID`, and successful reads overwrite it with the actual matched
frame ID. Matching uses the cycle-21 stream rule
`(frame.message_id & mask) == (requested_id & mask)`, including
mask-zero matching every active frame. The source is only the current
`latest_can_frame_batch_` active prefix: the call does not open, drain,
or backfill stream sessions, and it does not search stale older batches.
If multiple active frames match, it returns the first matching frame in
wire order.

No installed shim writes `kHalHandleError` and zeroes outputs. An
installed shim with null `data` writes `kHalCanInvalidBuffer`, zeroes
scalar outputs, and leaves receive state available for a later valid
read. Empty cache, empty latest batch, or no match writes
`kHalCanMessageNotFound` and zeroes all outputs; this intentionally uses
the receive-message error code rather than the stream-session
`kHalCanNoToken` warning.

**+11 tests** in `HalCanReceiveMessage`. The first test-reviewer pass
was `not-ready` for stale fallback, inactive-prefix, and internal-state
assertion gaps; the revised plan reached `ready-to-implement`.

### Cycle 32 — HAL_RefreshDSData

`HAL_RefreshDSData` is now wired as the first Driver Station runtime-loop
surface. No installed shim returns false. An installed shim attempts one
inbound receive/dispatch step through the same protocol path as `poll()`.
It returns true only when that accepted message is a `ds_state` packet; valid
non-DS messages still dispatch but return false. No-message, dispatch-error,
and shutdown paths also return false because the C ABI has no status
out-parameter.

The refresh path updates the same `latest_ds_state_` cache observed by the
existing DS getters. Tests pin that no-message and non-DS refresh calls
preserve getter-visible DS values, repeated DS refreshes are latest-wins, and
shutdown prevents later DS packets from surfacing through the public getters.

**+6 tests** in `HalRefreshDSData`. The first test-reviewer pass was
`not-ready` for internal-cache assertions, an infeasible multi-message queue
test, and weak shutdown observation; the revised plan reached
`ready-to-implement`.

### Cycle 33 — DS new-data event handles

`HAL_ProvideNewDataEventHandle` and `HAL_RemoveNewDataEventHandle` are
now wired for Driver Station new-data signaling. `WPI_Handle` and
`WPI_EventHandle` mirror WPILib as `unsigned int`, with `0` treated as
the invalid handle. Registrations live on the installed `shim_core`
object, are unique by handle, and are removed idempotently; no-shim,
zero-handle, and unknown-remove paths are no-ops.

Accepted `ds_state` packets wake registered handles through the WPI
signal-object seam by calling weak `WPI_SetEvent(handle)`. The wake is in
the shared inbound dispatch path, so both direct host-driven `poll()` and
`HAL_RefreshDSData()` signal handles when they accept DS data. Valid
non-DS packets, no-message refreshes, and post-shutdown paths do not
signal. The weak symbol keeps the shim linkable in non-WPI unit-test
hosts while still using the real WPI ABI symbol when present.

**+10 tests** in `HalDsNewDataEvents`. The first test-reviewer pass was
`not-ready` for ABI assertion gaps, mixed no-shim/invalid-handle
coverage, order-sensitive signal expectations, missing per-shim coverage,
and weak shutdown observation; the revised plan reached
`ready-to-implement`.

### Cycle 34 — HAL_GetOutputsEnabled

`HAL_GetOutputsEnabled` now mirrors WPILib sim's pinned expression:
the latest cached DS control word reports outputs enabled only when both
`kControlEnabled` and `kControlDsAttached` are set. No installed shim and
installed-shim empty DS cache both return false. E-stop does not add hidden
gating in v0; this surface follows the WPILib boolean expression exactly.

The result observes the same `latest_ds_state_` cache updated by direct
`poll()` and by `HAL_RefreshDSData`, so repeated DS updates are latest-wins.

**+5 tests** in `HalGetOutputsEnabled`.

### Cycle 35 — user-program observer calls

The five Driver Station user-program observer calls are now exported:
`HAL_ObserveUserProgramStarting`, `HAL_ObserveUserProgramDisabled`,
`HAL_ObserveUserProgramAutonomous`, `HAL_ObserveUserProgramTeleop`, and
`HAL_ObserveUserProgramTest`.

Because no protocol schema exists yet for publishing user-code observer
state to Sim Core, v0 records the latest call as a documented host-facing
`shim_core::user_program_observer_state()` value. The enum is per shim object
with `none`, `starting`, `disabled`, `autonomous`, `teleop`, and `test`
states. No-shim observer calls are no-ops, repeated calls are latest-wins,
observer calls do not publish into existing outbound schemas, and calls after
`HAL_Shutdown` are no-ops because shutdown detaches the process-global shim.

**+8 tests** in `HalObserveUserProgram`. The first test-reviewer pass was
`not-ready` for missing no-publish coverage, missing fresh-shim default-state
coverage, and not naming the state accessor as host-facing v0 API; the
revised plan reached `ready-to-implement`.

### Cycle 36 — HAL_SetJoystickOutputs

`HAL_SetJoystickOutputs` is now exported with the WPILib ABI:
`int32_t HAL_SetJoystickOutputs(int32_t, int64_t, int32_t, int32_t)`.
No installed shim returns `kHalHandleError`. Valid joystick indices are
`0..5`; invalid write indices return `kHalHandleError` and do not mutate
stored state.

Because no protocol schema exists yet for publishing joystick output and
rumble commands to Sim Core, v0 records the latest command per valid slot as
a documented host-facing `shim_core::joystick_outputs(joystickNum)` optional
`joystick_output_state`. Fresh shims and invalid accessor indices return
`std::nullopt`. Repeated writes are latest-wins per slot, state is per shim
object, raw signed C ABI values are preserved without clamping or narrowing,
and calls do not publish into existing outbound schemas. Calls after
`HAL_Shutdown` fail with `kHalHandleError` because shutdown detaches the
process-global shim.

**+11 tests** in `HalSetJoystickOutputs`. The first test-reviewer pass was
`not-ready` for missing invalid-accessor behavior and missing ABI signature
coverage; the revised plan reached `ready-to-implement`.

### Cycle 37 — joystick output protocol publication

Protocol v2 adds `src/backend/common/joystick_output.h` with
`joystick_output_state` and variable-size `joystick_output_batch`.
The new schema id is allowed only under `tick_boundary`; the validator
accepts exact active-prefix sizes for counts `0..6` and rejects
overflow, oversized, and undersized payloads.

The shim now exposes `send_joystick_output_batch`,
`current_joystick_output_batch`, and `flush_joystick_outputs`. The snapshot
is compacted in increasing joystick-number order, preserves the raw signed
values recorded by `HAL_SetJoystickOutputs`, publishes even a header-only
empty snapshot, and is not cleared by flush. `HAL_SetJoystickOutputs` remains
storage-only; publication happens only through explicit host flush/send
calls. Protocol-valid inbound joystick-output batches are rejected by shim
dispatch, and user-program observer state is not encoded in this schema.

**+21 common protocol tests** and **+9 shim tests**. The first test-reviewer
pass was `not-ready` for missing oversized active-prefix rejection coverage
and overbroad direction/shutdown wording; the revised plan reached
`ready-to-implement`.

### Cycle 38 — user-program observer protocol publication

Protocol v3 adds `src/backend/common/user_program_observer.h` with
`user_program_observer_mode` and fixed-size
`user_program_observer_snapshot`. The new schema id is allowed only under
`tick_boundary`; the validator accepts exactly `sizeof(snapshot)` payloads and
rejects truncated or oversized payloads.

The shim now exposes `send_user_program_observer_snapshot`,
`current_user_program_observer_snapshot`, and
`flush_user_program_observer`. The snapshot maps the latest cycle-35
per-shim observer state to the protocol enum and zero-initializes the named
reserved bytes. Fresh shims publish `mode == none`; flushing does not clear
storage, so repeated flushes republish latest state. Observer HAL calls remain
storage-only; publication happens only through explicit host flush/send calls.
Protocol-valid inbound observer snapshots are rejected by shim dispatch, and
joystick output state is not encoded in this schema.

**+4 net common protocol tests** and **+9 shim tests**. The test-reviewer pass
reached `ready-to-implement` before tests/code were written.

### Cycle 39 — Driver Station output pump

`shim_core::flush_next_driver_station_output(sim_time_us)` now provides a
host-facing one-message pump for DS output-side state. It coordinates the
existing cycle 38 observer snapshot and cycle 37 joystick-output snapshot
without adding a combined protocol schema or bumping `kProtocolVersion`.

The pump publishes at most one message per call because the Tier 1 outbound
lane carries one message at a time. The order is observer snapshot first,
joystick-output batch second, then repeat. It returns the schema that was
successfully published and advances phase only after a successful send; a
lane-busy or shutdown failure leaves the selected phase unchanged. Fresh shims
publish observer `none` followed by an empty joystick-output header, and the
underlying observer/joystick snapshots are not cleared by the pump.

**+7 shim tests**. The test-reviewer pass reached `ready-to-implement` with no
required changes.

### Cycle 40 — HAL runtime type

`HAL_GetRuntimeType()` now mirrors the WPILib HALBase C ABI and returns
`HAL_Runtime_RoboRIO2`. This is intentionally a C HAL metadata surface only:
it does not add or change protocol schemas, boot descriptors, or shim_core
state.

The C enum values are pinned separately from the robosim wire enum:
`HAL_Runtime_RoboRIO == 0`, `HAL_Runtime_RoboRIO2 == 1`, and
`HAL_Runtime_Simulation == 2`, while protocol `runtime_type::roborio_2`
remains its existing wire value. The call requires no installed shim, remains
stable after `HAL_Shutdown`, and does not publish outbound messages or poll
pending inbound messages.

**+4 shim tests**. The first test-reviewer pass required adding an inbound
message to prove no-poll behavior; the revised plan was implemented.

### Cycle 41 — HAL team number

`shim_core::make` now retains an exact copy of the successfully published
`boot_descriptor`, exposed through `boot_descriptor_snapshot()`. This gives
HALBase metadata calls a stable source of truth without reparsing the outbound
boot lane or changing any protocol schema.

`HAL_GetTeamNumber()` now mirrors the WPILib HALBase C ABI. With no installed
shim it returns `0`; with an installed shim it returns the retained
`boot_descriptor.team_number` before or after boot_ack, preserving the raw
`int32_t` value without clamping. The call does not publish, does not poll
pending inbound messages, and after `HAL_Shutdown` follows the detached
global-shim path while the caller-owned shim still retains boot metadata.

**+6 shim tests**. The test-reviewer pass reached `ready-to-implement` with no
required changes.

### Cycle 42 — HAL FPGA version/revision metadata

`HAL_GetFPGAVersion()` and `HAL_GetFPGARevision()` now mirror the WPILib
HALBase C ABI as a paired startup metadata slice. They do not add protocol
schemas, do not read per-tick inbound cache state, and do not change the boot
descriptor layout.

v0 returns deterministic shim constants with an installed shim:
`HAL_GetFPGAVersion()` returns `2026`, matching WPILib's documented FPGA
version competition-year convention, and `HAL_GetFPGARevision()` returns `0`
as explicit unknown/unmodeled revision metadata until a hardware profile source
exists. Both functions write `kHalSuccess` with an installed shim, including
before boot_ack or any inbound state, and write `kHalHandleError` plus return
0 with no installed shim. They do not publish, do not poll pending inbound
messages, and after `HAL_Shutdown` follow the detached no-shim path.

**+6 shim tests**. The test-reviewer pass reached `ready-to-implement` with no
required changes.

### Cycle 43 — HAL port handles

`HAL_GetPort()` and `HAL_GetPortWithModule()` mirror WPILib's HALBase port
handle value constructors. They are pure local handle encoders: no installed
shim is required, invalid module/channel values outside `0..254` return
`HAL_kInvalidHandle` (`0`), and installed-shim calls do not publish or poll.

### Cycle 44 — HAL status messages

`HAL_GetErrorMessage()` maps known shim status codes to stable static strings
and returns `"Unknown error status"` for unknown codes. `HAL_GetLastError()`
supports the `kHalUseLastError` sentinel by returning the last non-success
status written on the current thread. Successful status writes do not clear the
last-error value.

### Cycle 45 — HAL usage reporting

`HAL_Report()` mirrors the generated FRC usage-reporting ABI. With no installed
shim it returns `0`. With an installed shim it appends copied usage report
records to per-shim storage and returns a deterministic 1-based report index.
The function preserves raw signed fields, treats null feature strings as empty,
does not publish or poll, and follows the no-shim path after `HAL_Shutdown`.

### Cycle 46 — HAL comments metadata

`HAL_GetComments()` mirrors the WPILib HALBase comments metadata ABI. v0 reads
`PRETTY_HOSTNAME="..."` from `/etc/machine-info`, unescapes the common C-style
forms used by machine-info, caps the cached result to WPILib's 64-byte comments
buffer, and copies it into a heap-owned `WPI_String`.

The result is cached on first use, matching WPILib's one-time initialization
behavior. Missing files, absent keys, empty values, and malformed unterminated
quoted values return the empty string. Comments metadata does not require an
installed shim, does not publish, does not poll, and remains available after
`HAL_Shutdown` detaches the global shim pointer. Tests use a C++-namespace-only
path/cache override so they never read or mutate the host's real `/etc`.

**+10 shim tests**. The test-reviewer first requested the 64-byte cap,
deterministic test-hook lifecycle, and malformed-value coverage; the revised
plan reached `ready-to-implement`.

## What's next

The C HAL ABI is the **largest remaining shim chunk** and is now in
progress — cycles 12–46 wired the first read surfaces, the global shim
accessor, the first two write surfaces (`HAL_SendError` and
`HAL_CAN_SendMessage`), CAN one-shot/stream RX, CAN status reads, and DS
scalar and joystick/match/descriptor reads plus DS refresh/new-data
events, outputs-enabled, user-program observers, joystick outputs/rumble,
joystick output publication, user-program observer publication, DS-output
pump publication, `HAL_GetRuntimeType`, `HAL_GetTeamNumber`,
`HAL_GetFPGAVersion`, `HAL_GetFPGARevision`, `HAL_GetSerialNumber`,
`HAL_GetPort`, `HAL_GetPortWithModule`, `HAL_GetLastError`,
`HAL_GetErrorMessage`, `HAL_Report`, `HAL_GetComments`, plus the Notifier
control-plane/wait and lifecycle slices. Each
remaining surface is its own TDD cycle:
- Remaining device-family HAL surfaces such as DIO, PWM, relay, analog,
  compressor/pneumatics, power distribution, interrupts/counters/encoders, and
  serial/SPI/I2C as demanded by robot programs.
- Combined DS-output host-state message, if a host-facing aggregate becomes
  necessary.

Other shim concerns:
- Shim-initiated `shutdown`.
- On-demand request/reply.
- Broader threading model beyond the synchronized Notifier wait path.

Beyond the shim:
- T1 wait/reset/named-discovery work.
- T2 socket transport.
- LD_PRELOAD libc shim.
- Layer 3 / 4 / 5 work.
