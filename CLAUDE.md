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
    inbound state cache, dispatch by `(envelope_kind, schema_id)`
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
cycle 1 (boot handshake, inbound `clock_state` cache slot, shutdown
terminal receive path; single-threaded `poll()`-driven; cycle-1
explicit reject for the eight schemas not yet wired) is implemented
and adds 14 targeted tests; see `.claude/skills/hal-shim.md` and
`tests/backend/shim/TEST_PLAN.md`. Total suite is now 243 tests green
under both clang Debug and GCC Debug + ASan + UBSan. The remaining
HAL shim cycles (additional inbound schemas, outbound traffic past
boot, exported C HAL ABI surfaces, threading), T1
wait/reset/named-discovery work, T2 socket transport, and LD_PRELOAD
libc shim each remain as future TDD cycles. Layer 3/4/5 work has not
started.

In parallel, the visualizer subsystem (`src/viz/`,
`ROBOSIM_BUILD_VIZ=ON` opt-in) has Phase VA scaffold + Phase VB
read-side landed: scene-snapshot seam, edit-mode builder, orbit
camera, ray-vs-primitive picking, OpenGL primitive renderer, ImGui
panels, and a working `robosim-viz` CLI. Cylinder/arrow primitives
anchor at the proximal cap (TEST_PLAN rev 4) so a link's local
origin lands at its parent-joint attachment. ImGuizmo manipulation
of `world_from_local` is wired through main but writes only to the
in-memory snapshot — Phase VC (schema v1→v2 with `link.visual_origin`
/ `joint.origin` fields, plus `description::save_to_file`) has not
landed and is the next gating step before save round-trips. 49
viz-side tests green; see `.claude/skills/visualizer.md` and
`tests/viz/TEST_PLAN.md`.

Repo: `git@github.com:WL-Richards/Robosim.git`.
