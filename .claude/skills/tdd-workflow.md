---
name: tdd-workflow
description: Use when starting any new feature, bug fix, or refactor in robosim — covers the strict test-driven development cycle the project requires, including the `test-reviewer` agent gate that proposed tests must pass before implementation begins.
---

# TDD workflow

Robosim is built test-first. Tests are designed *before* implementation
and reviewed *before* they're written. The cycle is non-optional.

## The cycle

```
  1. Decide what behavior to add or change
  2. Read the relevant layer skill(s) and ARCHITECTURE.md
  3. Draft a test plan (proposed tests, in pseudocode or sketch form)
  4. Submit to the `test-reviewer` agent
  5. Iterate until verdict is `ready-to-implement`
  6. Write the (failing) tests as approved
  7. Run them, confirm they fail for the expected reason
  8. Write the minimum code to make them pass
  9. Refactor under green
 10. Update the skill for the area with anything learned
```

Steps 4–5 are the unique part. Skipping them is not "moving fast" —
it's how we ship tests that pass against bad implementations.

## Step 3 — drafting the test plan

For each proposed test, write down:

- **Name** describing the behavior under test, in sentence form.
- **Layer / contract** being pinned (e.g. "Layer 4 — DCMotor model
  electrical equations" or "Layer 2 — HAL `HAL_GetCounter` contract").
- **Bug class** the test would catch — the concrete way the SUT could
  be wrong such that this test fails.
- **Inputs** and the **expected outputs**, with a citation if the
  expected value is a physical quantity (datasheet, vendor curve, bench
  fixture filename + commit).
- **Tolerance** if numerical, with justification.
- **Determinism notes** — anything stochastic must declare its seed.
- **Assumptions / stubs** — if the test depends on an interface stub,
  say so and explain what's still being meaningfully verified.

A bullet list is fine. The point is to make the design legible *before*
code exists.

## Step 4 — submitting to test-reviewer

Use the `test-reviewer` agent (defined in `.claude/agents/test-reviewer.md`).
Feed it:

- The test plan from step 3.
- A pointer to the layer skill(s) the work falls under.
- Any existing related tests in the area.
- The cited sources (link or path).

The agent returns a structured report with per-test verdicts. Possible
suite-level verdicts: `ready-to-implement` or `not-ready`.

If `not-ready`: address the findings and resubmit. Don't argue with the
reviewer in code review — argue in writing, where the disagreement is
recorded. If the reviewer is wrong about a specific point, push back
with reasoning; the reviewer's findings are advisory but the bar to
override is high (and the override goes in the test file as a comment
explaining why).

## Step 6 — writing the (failing) tests

Implement exactly what was approved. If during implementation you
realize a test needs a substantive change (different inputs, different
tolerance, a different assertion), pause and re-submit the changed
test. Drift between approved-plan and shipped-test defeats the gate.

## Step 7 — confirming failures

Each test must fail before its code exists, and fail for the reason you
expect. A test that passes when the SUT is unimplemented is broken
(common cause: asserting `>= 0` on something that defaults to 0).

## Step 8 — minimum code

Write the least code that makes the failing tests pass. No speculative
features, no extra error handling, no abstractions you don't need yet.
The next test will pull more code into existence.

## Step 9 — refactor under green

With tests passing, clean up. See `code-style.md` for what "clean" means
here. Tests stay green; if they break, you've changed behavior, not
just structure — back up.

## Step 10 — update the skill

Anything learned during the cycle that future implementers should know
goes into the relevant skill. Examples:

- A datasheet number that differs from a value in vendor SDK headers.
- A subtlety in how WPILib's HAL interprets a particular call.
- A solver instability discovered at certain operating points.
- A vendor firmware quirk discovered empirically.

If you noticed it once, the next person will hit it too. Capture it.

## What "good test" looks like here

The full criteria are enforced by `test-reviewer`. The short version:

- Tests behavior on the **public contract**, not implementation
  internals. A refactor that preserves behavior must not break tests.
- One test asserts one thing. Test names read like a sentence.
- Deterministic: no wall clock, no real network, no unseeded RNG, no
  `sleep()` for synchronization (use injected schedulers).
- Boundary cases (zero, max, negative, NaN, empty, full) covered by
  the suite, not just happy path.
- For physical models: cited source, stated tolerance, plausible
  operating point.
- For HAL/backend tests: pin the *contract*, not our implementation.
  Tests must pass against any correct backend implementation.
- For vendor models: state the firmware version the test targets.
- For determinism: at least one test in the deterministic-replay path
  asserts byte-identical state logs across two runs of the same scenario.

## Anti-patterns — avoid (the reviewer will reject)

- Asserting only that no exception was thrown.
- Asserting on log output as the primary check.
- Sleeping in tests.
- Mocking the system under test.
- Tests that exist only to raise coverage numbers.
- Tests written against implementation accidents (call order,
  internal state).

## Tooling (decided 2026-04-28)

- **Framework:** GoogleTest + GoogleMock.
- **Test naming:** `TEST(SuiteName, TestName)` where the suite is
  the system-under-test and the test name reads as a sentence about
  the behavior — e.g. `TEST(KrakenX60, StallsWhenLoadExceedsTorqueLimit)`.
- **Parameterized tests:** `INSTANTIATE_TEST_SUITE_P` for
  per-operating-point coverage of physical models. Each
  parameterization case carries a citation (datasheet, vendor
  curve, bench fixture row).
- **Fixtures:** keep minimal; tests should be readable top-to-bottom
  without jumping to setup.
- **CI gates** (must be green to merge): UBSan + ASan + TSan
  sanitizers, `clang-tidy`, GoogleTest suite, plus the test-reviewer
  agent's verdict on any new or modified test plan.
- **Tests live** in a parallel `tests/` tree mirroring the source
  layout (`src/motor/kraken_x60.cpp` → `tests/motor/kraken_x60_test.cpp`).
