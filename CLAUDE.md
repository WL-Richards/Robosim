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
7. **Comment as you go.** Public APIs, wire schemas, ABI boundaries, and
   non-obvious invariants should get Javadoc/Doxygen-style `/** ... */`
   comments. Keep them thorough enough for the next backend change to
   preserve layout, sequencing, and ownership contracts without reverse
   engineering tests.

## Index

Plans, architecture, references:

- `docs/V0_PLAN.md` — entry point for the sim-core implementation
- `docs/VISUALIZER_V0_PLAN.md` — entry point for the visualizer
  subsystem (parallel work stream; `src/viz/`)
- `docs/ARCHITECTURE.md` — binding architecture (process, time,
  determinism, tiers, protocol, language, physics, robot description,
  visualizer)
- `docs/OPEN_QUESTIONS.md` — open and decided design questions
- `docs/REFERENCES.md` — external sources, datasheets, prior art

Status (one doc per subsystem — keep these current):

- `docs/STATUS_SIM_CORE.md` — sim core, protocol, transport, HAL shim
- `docs/STATUS_VISUALIZER.md` — `src/viz/` editor / live / replay
- `docs/STATUS_RIO_BENCH.md` — `tools/rio-bench/` HAL cost benchmarking

Skills (`.claude/skills/` — actionable per-feature knowledge):

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
  benchmarks RIO 2 HAL call costs and emits the CSV fixture the sim
  core's tier-1 backend will consume

Agents:

- `.claude/agents/test-reviewer.md` — TDD test-design gatekeeper

## Status snapshot

For detail, read `docs/STATUS_*.md`. As of this revision:

- **Sim core** — Phase A scaffold + Phase B description loader +
  protocol schema/session + Tier 1 shared-memory transport + HAL shim
  through cycle 24 (v0 outbound surface closed; **power_state read
  surface closed** + **clock_state read surface closed**; C HAL ABI
  surfaces wired so far: **`HAL_GetFPGATime`** +
  **`HAL_GetVinVoltage`** + **`HAL_GetVinCurrent`** +
  **`HAL_GetBrownoutVoltage`** + **`HAL_GetCommsDisableCount`** +
  5 HAL_Bool readers (**`HAL_GetBrownedOut`**,
  **`HAL_GetSystemActive`**, **`HAL_GetSystemTimeValid`**,
  **`HAL_GetFPGAButton`**, **`HAL_GetRSLState`**) sharing a
  `clock_state_hal_bool_read` pointer-to-member helper; plus
  **`HAL_SendError`** with per-shim pending buffer and explicit
  `flush_pending_errors`; plus **`HAL_CAN_SendMessage`** with
  per-shim pending CAN frame buffer and explicit
  `flush_pending_can_frames`; plus **`HAL_CAN_OpenStreamSession`** /
  **`HAL_CAN_ReadStreamSession`** / **`HAL_CAN_CloseStreamSession`**
  with per-stream CAN RX queues; plus **`HAL_CAN_GetCANStatus`**;
  plus **`HAL_GetControlWord`** / **`HAL_GetAllianceStation`** /
  **`HAL_GetMatchTime`** from `latest_ds_state_`;
  plus **`HAL_GetJoystickAxes`** / **`HAL_GetJoystickPOVs`** /
  **`HAL_GetJoystickButtons`** from `latest_ds_state_`;
  plus the process-global accessor;
  cycle 15 fixed a pre-existing `hal_bool` signedness parity bug;
  cycle 18 pinned D-C18-UINT32-TO-INT32-CAST for the schema-vs-WPILib
  type seam; cycle 19 pinned the first C HAL write-surface buffering
  rules; cycle 20 pinned the CAN send buffering, invalid-buffer, and
  period subset rules; cycle 21 pinned CAN stream queueing, masked
  filtering, empty-read, overflow, and close-handle rules; cycle 22
  pinned CAN status out-parameter order and zero-default read behavior;
  cycle 23 pinned DS scalar zero/default semantics and
  `HAL_ControlWord` named-bit mapping; cycle 24 pinned joystick
  struct byte-copy, ABI layout, and invalid-index zero/default
  semantics).
  Full project baseline **ctest 532/532** green under `build`; cycle 24
  focused shim suite **217/217** green. Layer
  3/4/5 not started.
- **Visualizer** — Phases VA + VB + VC + VD all landed; v0 Edit mode
  functionally complete. **457/457** viz + description tests green.
- **rio-bench** — pure-Java logic **41/41** green; HAL-bound code
  compiles against WPILib 2026.2.1 and awaits a physical RIO 2 run.

Repo: `git@github.com:WL-Richards/Robosim.git`.
