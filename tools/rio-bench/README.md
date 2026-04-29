# rio-bench

A WPILib robot project (Java + Gradle) that runs on a physical
RoboRIO 2 and measures HAL call costs. Output is a versioned fixture
file written to `references/rio2-hal-costs/<wpilib-version>-<rio-firmware>.csv`
and consumed by the sim core at startup.

**Not built by the top-level CMake.** This is a Gradle project; build
it standalone with `./gradlew build` from this directory once the
project skeleton lands. Until then this directory holds only the README.

See `docs/ARCHITECTURE.md` "v0 scope" → "Tooling deliverable" for the
full spec (mean + stddev + p99 + outlier rate per call class, multiple
operating points: idle-bus, saturated-bus, multi-thread contention).
