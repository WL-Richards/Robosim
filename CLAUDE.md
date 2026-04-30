# robosim

High-fidelity, control-system-agnostic FRC robot simulator. Goal: run
unmodified robot binaries against an emulated control system and a
physically accurate world, faithfully enough to trust as a correctness
signal — sim/reality divergence is treated as a sim bug, not "good
enough."

## Before implementing in this repo

1. **Read the relevant skill(s)** in `.claude/skills/` for the area
   you're touching. Don't guess from filenames; load the skill.
2. **Tests are written first** and reviewed by the `test-reviewer`
   agent before any test code is written — see
   `.claude/skills/tdd-workflow.md`.
3. **Updating the skill is part of every feature change.** A change
   without a skill update is incomplete — see
   `.claude/skills/adding-a-feature.md`.

## Non-negotiables

1. **No shortcuts.** Stub-and-move-on is a tracked scaffold, not done.
2. **Robot code is unmodified.** Sim sits *underneath* the HAL.
3. **Control-system agnostic.** RoboRIO 2 is one backend, not the only.
4. **Vendor-faithful.** Match firmware behavior, including quirks, to
   stated firmware versions.
5. **Deterministic and replayable.** Same seed + inputs →
   byte-identical logs.
6. **Accuracy is measurable.** Every physical model has a stated error
   bound vs. a cited source.

## Index

- `docs/V0_PLAN.md` — entry point for the sim-core implementation
- `docs/VISUALIZER_V0_PLAN.md` — entry point for the visualizer
  subsystem (parallel work stream; `src/viz/`)
- `docs/ARCHITECTURE.md` — binding architecture (process, time,
  determinism, tiers, protocol, language, physics, robot description,
  visualizer)
- `docs/OPEN_QUESTIONS.md` — open and decided design questions
- `docs/REFERENCES.md` — external sources, datasheets, prior art
- `.claude/skills/` — per-feature actionable knowledge:
  - `code-style.md`, `tdd-workflow.md`, `adding-a-feature.md` — process
  - `layer-1-robot-code.md` … `layer-5-world-physics.md` — per-layer
  - `robot-description-loader.md` — `descriptions/<robot>.json` loader
  - `protocol-schema.md` — HAL ↔ Sim Core wire-format POD structs +
    pure stateless validator + truncation helpers (Layer 2 foundation)
  - `protocol-session.md` — stateful wrapper around the protocol
    validator: sequence counters, boot/ack ordering, on-demand pairing
  - `tier1-shared-memory-transport.md` — native shared-memory transport:
    fixed region, endpoint wrapper, memfd/mmap lifecycle
  - `hal-shim.md` — in-process HAL shim core: boot handshake,
    inbound state caches, dispatch by `(envelope_kind, schema_id)`
  - `visualizer.md` — `src/viz/` 3D viewer (Edit / Live / Replay)
  - `rio-bench.md` — `tools/rio-bench/` WPILib robot project that
    benchmarks RIO 2 HAL call costs and emits the CSV fixture the
    sim core's tier-1 backend will consume
- `.claude/agents/test-reviewer.md` — TDD test-design gatekeeper

## Status

Phase A scaffold landed. Phase B (the robot description loader) is
implemented and 83-test green. The HAL ↔ Sim Core protocol schema
(Layer 2 foundation: 32-byte sync envelope, 9 closed payload schemas,
stateless validator, truncation helpers, WPILib byte-parity baselined
at v2026.2.2) is implemented and 106-test green under clang/gcc Debug
+ ASan + UBSan. See `tests/backend/common/TEST_PLAN.md` and
`.claude/skills/protocol-schema.md`. The stateful protocol session
wrapper (send/receive sequence counters, boot/boot_ack ordering,
on-demand request/reply pairing, shutdown terminal receive state) is
implemented and adds 16 targeted tests; see
`.claude/skills/protocol-session.md`. The Tier 1 native shared-memory
transport (fixed two-lane region, nonblocking endpoint,
protocol-session integration, Linux memfd/mmap lifecycle) is
implemented and adds 24 targeted tests; see
`.claude/skills/tier1-shared-memory-transport.md`. The HAL shim core
through cycle 8 (boot handshake; inbound `clock_state`, `power_state`,
`ds_state`, `can_frame_batch`, `can_status`, `notifier_state`,
`notifier_alarm_batch`, and `error_message_batch` cache slots —
**all 8 per-tick payload schemas wired**, each independently
maintained with latest-wins replacement; the four variable-size
schemas (`can_frame_batch`, `notifier_state`, `notifier_alarm_batch`,
`error_message_batch`) use active-prefix memcpy with
zero-init-before-write so a shrinking batch leaves unused element
slots byte-zero, and zero-init also covers the 4-byte interior
`count → events`/`count → slots` implicit padding word in the two
notifier schemas (`error_message_batch` has zero implicit padding
because its `reserved_pad[4]` between `count` and `messages` plus
`error_message`'s `reserved_pad[3]` after `truncation_flags` are
*named* fields covered by defaulted `operator==`); shutdown terminal
receive path; single-threaded `poll()`-driven; the
`unsupported_payload_schema` arm is now unreachable from valid
traffic but retained as a defensive forward-compat structural guard
(D-C8-DEAD-BRANCH); padding-byte determinism for `ds_state`,
`can_frame_batch`, `notifier_state`, and `notifier_alarm_batch`
pinned by direct `std::memcmp` — cycle 8's C8-6 also closed a
pre-existing gap where the landed C7-6 implementation had silently
omitted `notifier_alarm_batch` from the determinism replay despite
its name "AllSevenSlots") is implemented and the suite covers 91
shim tests green (full project ctest 406/406) under both clang
Debug and GCC Debug + ASan + UBSan; see `.claude/skills/hal-shim.md`,
`tests/backend/shim/TEST_PLAN.md`,
`tests/backend/shim/TEST_PLAN_CYCLE2.md`,
`tests/backend/shim/TEST_PLAN_CYCLE3.md`,
`tests/backend/shim/TEST_PLAN_CYCLE4.md`,
`tests/backend/shim/TEST_PLAN_CYCLE5.md`,
`tests/backend/shim/TEST_PLAN_CYCLE6.md`,
`tests/backend/shim/TEST_PLAN_CYCLE7.md`, and
`tests/backend/shim/TEST_PLAN_CYCLE8.md`. The per-tick payload
schema set is now closed on the inbound side; cycle 9 (below)
opened the outbound side. CAN RX queueing semantics are deferred
per D-C4-LATEST-WINS until the cycle that wires
`HAL_CAN_ReadStreamSession`. T1 wait/reset/named-discovery
work, T2 socket transport, and LD_PRELOAD libc shim each
remain as future TDD cycles. Layer 3/4/5 work has not started.

Cycle 9 added the **first outbound-past-boot** surface,
promoting the shim from "send-only-once-at-construction" to a
working duplex: `send_can_frame_batch(const can_frame_batch&,
uint64_t sim_time_us)` publishes a `can_frame_batch` payload as
a `tick_boundary` envelope into `backend_to_core`. The
implementation inlines the active-prefix size computation
(`offsetof(can_frame_batch, frames) + batch.count *
sizeof(can_frame)`) per D-C9-NO-HELPER — outbound mirror of the
inbound D-C4-VARIABLE-SIZE active-prefix discipline. The same
shutdown-terminal short-circuit as `poll()` applies on the
outbound path (D-C9-SHUTDOWN-TERMINAL). The shim's outbound
API uses typed-per-schema methods (D-C9-TYPED-OUTBOUND), so the
five sim-authoritative schemas (`clock_state`, `power_state`,
`ds_state`, `can_status`, `notifier_alarm_batch`) are
intentionally absent from the outbound surface — wrong-direction
publishing is impossible at the API boundary. Outbound does not
gate on `is_connected()` (D-C9-NO-CONNECT-GATE), matching the
protocol's bidirectional-handshake-overlap allowance. The
shared `wrap_send_error` lambda was generalized from "transport
rejected outbound boot envelope" to "transport rejected outbound
envelope" so it serves both call sites (D-C9-WRAPPED-SEND-ERROR).
Cycle 9 added 10 new tests (the new `ShimCoreSend` suite plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdentical`
`OutboundCanFrameBatch`); the suite now covered 101 shim tests
green (full project ctest 416/416) under both clang Debug and
GCC Debug + ASan + UBSan. See `.claude/skills/hal-shim.md` and
`tests/backend/shim/TEST_PLAN_CYCLE9.md`.

Cycle 10 added the **second outbound-meaningful schema**:
`send_notifier_state(const notifier_state&, uint64_t sim_time_us)`
publishes `notifier_state` as a `tick_boundary` envelope into
`backend_to_core`. The schema's 8-byte header offset (vs cycle
9's 4-byte) and 132 implicit padding bytes (the most demanding
of any outbound v0 schema) made it the natural follow-on. Cycle
10 also **cashed in cycle 9's deferred D-C9-NO-HELPER trigger**
by extracting `active_prefix_bytes` overloads into the schema
headers (`src/backend/common/can_frame.h` for `can_frame_batch`
and `src/backend/common/notifier_state.h` for `notifier_state`)
per D-C10-EXTRACT-ACTIVE-PREFIX. Both `send_can_frame_batch`
(refactored) and `send_notifier_state` (new) call the production
helpers; the duplicated test-side overloads in
`tests/backend/tier1/test_helpers.h` for those two schemas were
deleted, and tests resolve to the production version via ADL.
Cycle 10 also formalized the per-method-test trim convention
(D-C10-{INBOUND-INDEPENDENCE / NO-CONNECT-GATE / RECEIVE-COUNTER-
INDEPENDENCE}-INHERITS): the test-reviewer explicitly endorsed
that cycle-9's cross-cutting contracts are session/transport-
level and don't need re-verification per outbound schema; only
the shutdown short-circuit needs per-method coverage
(D-C10-SHUTDOWN-TERMINAL-INHERITS, pinned by C10-5). Cycle 10
landed 6 new tests (5 in `ShimCoreSend` plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutbound`
`NotifierState`); the suite covered 107 shim tests green
(full project ctest 422/422) under both clang Debug and GCC
Debug + ASan + UBSan. See `.claude/skills/hal-shim.md` and
`tests/backend/shim/TEST_PLAN_CYCLE10.md`.

Cycle 11 added the **third (and final v0) outbound-meaningful
schema**: `send_error_message_batch(const error_message_batch&,
uint64_t sim_time_us)` publishes `error_message_batch` as a
`tick_boundary` envelope into `backend_to_core`. Cycle 11
extended D-C10-EXTRACT-ACTIVE-PREFIX by adding the third
production `active_prefix_bytes` overload to
`src/backend/common/error_message.h` plus a recommended
`static_assert(offsetof(error_message_batch, messages) == 8)`
that pins the named-`reserved_pad[4]`-derived header layout.
The schema is the only outbound v0 schema with **zero implicit
C++ padding** (D-C8-PADDING-FREE inherited from cycle 8 — both
`reserved_pad` fields are NAMED), so cycle 11's memcmp
companions are byte-equivalent to `vector::operator==` but
kept for cross-cycle structural parity per the test-reviewer's
OQ-C11-MEMCMP-IN-DETERMINISM resolution. Cycle 11 introduces
**no new design decisions** — every contract inherits from
cycles 9 and 10, and the per-method-test trim convention
applies unchanged. With cycle 11, the **v0 outbound surface is
closed** for the three semantically-meaningful outbound
schemas; the five sim-authoritative schemas remain
intentionally absent per D-C9-TYPED-OUTBOUND. Cycle 11 landed
6 new tests (5 in `ShimCoreSend` plus
`ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalOutbound`
`ErrorMessageBatch`); the suite now covers 113 shim tests green
(full project ctest 428/428) under both clang Debug and GCC
Debug + ASan + UBSan. See `.claude/skills/hal-shim.md` and
`tests/backend/shim/TEST_PLAN_CYCLE11.md`. The next major shim
work is shim-initiated `shutdown`, on-demand request/reply, and
the C HAL ABI surfaces (`HAL_GetFPGATime`, `HAL_CAN_SendMessage`,
`HAL_SendError`, `HAL_Notifier*`, etc.) — the largest remaining
chunk and the cycle that promotes `latest_can_frame_batch_`
from latest-wins to a queue per D-C4-LATEST-WINS.

In parallel, the visualizer subsystem (`src/viz/`,
`ROBOSIM_BUILD_VIZ=ON` opt-in) has Phase VA scaffold + Phase VB
read-side + Phase VC schema-v2/serializer + Phase VD gizmo persistence
all landed: scene-snapshot seam, edit-mode builder honoring v2
origins via the kinematic-vs-visual frame distinction (descendants
compose against parent kinematic, never parent visual), orbit camera,
ray-vs-primitive picking, OpenGL primitive renderer, ImGui panels,
ImGuizmo translate/rotate routed through `viz::apply_gizmo_target`
(pure logic) into the description's `joint.origin` /
`link.visual_origin`, `viz::edit_session` driving load/save/reload
with a dirty bit, File menu (Save / Save As / Reload-from-disk with
Discard-changes confirmation; Ctrl+S hotkey), and a working
`robosim-viz` CLI. Cylinder/arrow primitives anchor at the proximal
cap (TEST_PLAN rev 4) so a link's local origin lands at its
parent-joint attachment. `description::decompose_origin` provides the
algebraic inverse of `compose_origin` (URDF intrinsic-Z-Y′-X″, with
a gimbal-pole branch that pins yaw=0 and absorbs the residual into
roll while preserving the composed transform). 457/457 viz +
description tests green across the build matrix. v0 visualizer Edit
mode is functionally complete. See `.claude/skills/visualizer.md`,
`.claude/skills/robot-description-loader.md`, `tests/viz/TEST_PLAN.md`,
`tests/description/TEST_PLAN_VC.md`, and `tests/viz/TEST_PLAN_VD.md`.

Also in parallel, the `tools/rio-bench/` WPILib 2026 Java/Gradle
robot project (the v0 tooling deliverable that benchmarks RoboRIO
2 HAL call costs and emits the CSV fixture the sim core's tier-1
backend will consume) has all pure-Java logic implemented and
41/41 green under `./gradlew test` on JDK 17 with no HAL JNI
required. Test plan went through 4 review rounds with
`test-reviewer`; revision 4 + a P1 wording fix landed
`ready-to-implement`. HAL-bound code (`BenchmarkRunner`,
`CallBindings`, `RioFileSink`, `Robot.autonomousInit` wiring)
compiles against WPILib 2026.2.1 and is ready for the first
hardware run; the produced CSV is tagged `validated=false` until
the operator runs the sweep on a physical RIO 2 and commits the
result to `references/rio2-hal-costs/`. See
`.claude/skills/rio-bench.md` and
`tools/rio-bench/RioBenchmark/TEST_PLAN.md`.

Repo: `git@github.com:WL-Richards/Robosim.git`.
