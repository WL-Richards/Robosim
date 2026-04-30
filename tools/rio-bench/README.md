# rio-bench

A WPILib 2026 Java/Gradle robot project that runs on a physical
RoboRIO 2 and measures HAL call costs. Output is a versioned fixture
file written to `/home/lvuser/rio-bench/<wpilib-version>-<rio-firmware>.csv`
on the RIO; the operator pulls it back to
`references/rio2-hal-costs/` in this repo. Consumed by the sim core's
tier-1 backend at startup.

**Not built by the top-level CMake.** This is a Gradle project. Build
and test it standalone:

```
cd RioBenchmark
JAVA_HOME=/path/to/wpilib/2026/jdk ./gradlew test       # 41 desktop unit tests
JAVA_HOME=/path/to/wpilib/2026/jdk ./gradlew deploy     # deploy to team 6443 RIO
```

## Operator procedure

1. `cd RioBenchmark && ./gradlew deploy`.
2. Connect the DS, enable autonomous. The sweep runs to completion
   in ~2 minutes; watch for `[rio-bench] DONE: file at <path>` in
   the console.
3. `scp lvuser@10.64.43.2:/home/lvuser/rio-bench/<file>.csv ../../references/rio2-hal-costs/`.
4. Edit the preamble: set `validated=true`. Commit.

## Documentation

- `.claude/skills/rio-bench.md` — full skill: scope, public surface,
  design decisions, validation status, known gotchas.
- `RioBenchmark/TEST_PLAN.md` — the reviewer-approved test plan.
- `docs/ARCHITECTURE.md` "Tooling deliverable" — the v0 spec this
  project satisfies.
