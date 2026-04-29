---
name: adding-a-feature
description: Use when adding a new feature, component, vendor device, motor model, sensor, mechanism, or backend to robosim. Covers the full process from design through skill update. Mandatory reading before opening a new feature branch or starting design work on something not already covered by an existing skill.
---

# Adding a feature

A feature in robosim is anything that adds new capability — a new motor
model, a new vendor device, a new sensor type, a new backend, a new game
piece, a new physics behavior. The process below applies uniformly.

## Process

### 1. Locate the feature on the layer map

Identify which layer(s) the feature touches. Read those layer skills.
If your feature touches more than one, read all relevant skills before
proceeding. Cross-layer features deserve extra design care.

### 2. Check for unresolved decisions

Open `docs/OPEN_QUESTIONS.md`. If your feature depends on or implies
a position on any open question, surface that. Either:

- Resolve the open question (write up the decision, update
  `docs/ARCHITECTURE.md`, mark the question decided), or
- Decline to start the feature until the question is resolved.

Do not silently take a position on an open question by implementing.

### 3. Find or create the feature's skill

A feature gets its own skill if any of the following are true:

- It exposes a non-trivial public surface that future implementers
  will interact with.
- It models a real-world device, motor, or sensor with citations and
  tolerances.
- It introduces a new backend, vendor, or game-piece type.
- It touches the determinism path.

If the feature is small enough to fit comfortably inside an existing
skill (e.g. a new helper inside an already-skilled module), extend
that skill rather than fragmenting.

A new skill goes in `.claude/skills/<feature-name>.md` with the
standard frontmatter:

```yaml
---
name: <feature-name>
description: When this skill applies — be specific so future agents
  load it for the right work.
---
```

### 4. Design tests first

Follow `tdd-workflow.md`. Draft the test plan, including:

- Citations for any physical constants involved.
- Tolerances and their justification.
- Determinism implications.

### 5. Submit tests for review

The `test-reviewer` agent must approve the test plan before you write
implementation. See `tdd-workflow.md` step 4.

### 6. Implement

Follow `code-style.md`. Implement minimum code to pass approved tests.
Cite sources for any constants in code comments. Keep functions small.

### 7. Validate

If the feature is a physical model: run the validation tests against
the cited source data. Record the measured error vs. expected. If
outside the stated tolerance, the feature is `unvalidated` until
either the model improves or the tolerance is justifiably widened
(with the reasoning recorded in the skill).

### 8. Update / write the skill

The skill is a working document. Capture:

- **Scope.** What this feature covers and explicitly does *not*.
- **Public surface.** The functions, types, or interfaces external
  callers should use. (Do not duplicate code into the skill — link or
  cite paths.)
- **Design decisions.** Why this shape vs. alternatives considered.
- **Citations.** Sources for any physical constants and behaviors.
- **Validation status.** What has been validated, against what data,
  to what tolerance. What remains unvalidated.
- **Known limits / gotchas.** Anything that bit you during
  implementation. Anything a future caller could trip on.
- **Open follow-ups.** TODOs that didn't block this feature but should
  be tracked.

### 9. Update cross-references

If the feature is referenced from other skills (e.g. a new motor model
referenced by Layer 4 in general), update those skills' index sections.

### 10. Update `docs/REFERENCES.md`

If you used a new external source (datasheet, paper, vendor doc, prior
project), add a one-liner there. Otherwise the source is invisible to
future work.

## What "complete" means

A feature is complete when **all** of the following hold:

- Tests are passing (and were approved by `test-reviewer` before
  implementation).
- `code-style.md` rules are followed.
- The feature's skill exists and is up to date.
- Cross-references are updated.
- Validation status is recorded (for physical models).
- New external sources are in `REFERENCES.md`.

A feature is **not** complete just because tests pass. Skipping the
skill update is the most common failure mode and the most expensive —
the next implementer pays the cost.

## When to push back on the scope of a feature

If the feature, as scoped, can't satisfy the project's
non-negotiables (`CLAUDE.md`), push back. For example:

- A motor model proposed without citations is not yet a feature; it's
  a placeholder. Either get the citations or scope down to "interface
  shape only," explicitly marked as a stub.
- A backend that depends on host wall-clock time breaks determinism.
  Push back; redesign for injected time.
- A vendor device modeled without firmware versioning will rot. Push
  back; add the version dimension.

Pushing back early is cheaper than ripping out a half-feature later.
