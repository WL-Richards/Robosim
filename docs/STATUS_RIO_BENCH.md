# rio-bench status

WPILib 2026 Java/Gradle robot project under `tools/rio-bench/`. v0
tooling deliverable that benchmarks RoboRIO 2 HAL call costs and
emits the CSV fixture the sim core's tier-1 backend will consume.
See `.claude/skills/rio-bench.md`.

## Headline

- All pure-Java logic implemented; **41/41 green** under
  `./gradlew test` on JDK 17, no HAL JNI required.
- Test plan went through 4 review rounds with `test-reviewer`;
  revision 4 + a P1 wording fix landed `ready-to-implement`.
- HAL-bound code (`BenchmarkRunner`, `CallBindings`, `RioFileSink`,
  `Robot.autonomousInit` wiring) compiles against WPILib 2026.2.1
  and is ready for the first hardware run.
- `BenchmarkRunner` emits `[rio-bench] progress: NN%` lines to the
  DataLog at every 5% boundary across the full sweep so the operator
  can confirm forward progress on the DS console.

## What's next

The produced CSV is tagged `validated=false` until the operator runs
the sweep on a physical RIO 2 and commits the result to
`references/rio2-hal-costs/`.

## References

- `tools/rio-bench/RioBenchmark/TEST_PLAN.md`
