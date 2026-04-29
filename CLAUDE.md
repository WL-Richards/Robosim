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

- `docs/V0_PLAN.md` — entry point for the first implementation session
- `docs/ARCHITECTURE.md` — binding architecture (process, time,
  determinism, tiers, protocol, language, physics, robot description)
- `docs/OPEN_QUESTIONS.md` — open and decided design questions
- `docs/REFERENCES.md` — external sources, datasheets, prior art
- `.claude/skills/` — per-feature actionable knowledge
- `.claude/agents/test-reviewer.md` — TDD test-design gatekeeper

## Status

Foundational design complete; all v0-gating questions decided. No
code yet. Start at `docs/V0_PLAN.md`.

Repo: `git@github.com:WL-Richards/Robosim.git` (not yet initialized
locally; first task in V0_PLAN).
