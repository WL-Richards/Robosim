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
through cycle 5 (boot handshake; inbound `clock_state`, `power_state`,
`ds_state`, `can_frame_batch`, and `can_status` cache slots, each
independently maintained with latest-wins replacement;
`can_frame_batch` uses variable-size active-prefix memcpy with
zero-init-before-write so a shrinking batch leaves unused frame
slots byte-zero; shutdown terminal receive path; single-threaded
`poll()`-driven; loud reject for the three schemas not yet wired
with session-counter advance through dispatch; padding-byte
determinism for both `ds_state` and `can_frame_batch` pinned by
direct `std::memcmp`) is implemented and the suite covers 50 shim
tests; see `.claude/skills/hal-shim.md`,
`tests/backend/shim/TEST_PLAN.md`,
`tests/backend/shim/TEST_PLAN_CYCLE2.md`,
`tests/backend/shim/TEST_PLAN_CYCLE3.md`,
`tests/backend/shim/TEST_PLAN_CYCLE4.md`, and
`tests/backend/shim/TEST_PLAN_CYCLE5.md`. Cycle-6 (inbound
`notifier_state` — variable-size with `offsetof(notifier_state,
slots) == 8`) is the next TDD cycle. CAN RX queueing semantics are
deferred per D-C4-LATEST-WINS until the cycle that wires
`HAL_CAN_ReadStreamSession`. The remaining HAL shim work (additional
inbound schemas, outbound traffic past boot, exported C HAL ABI
surfaces, threading), T1 wait/reset/named-discovery work, T2 socket
transport, and LD_PRELOAD libc shim each remain as future TDD
cycles. Layer 3/4/5 work has not started.

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

Repo: `git@github.com:WL-Richards/Robosim.git`.
