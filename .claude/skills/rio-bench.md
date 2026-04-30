---
name: rio-bench
description: Use when working on `tools/rio-bench/` — the WPILib 2026 Java/Gradle robot project that runs on a physical RoboRIO 2 and measures HAL call costs (mean / stddev / p99 / outlier rate per call class × operating point). Output is a versioned CSV fixture at `references/rio2-hal-costs/<wpilib-version>-<rio-firmware>.csv` that the sim core's tier-1 backend loads at startup to apply realistic HAL-call jitter without a real RIO. NOT built by the top-level CMake. Java tool, separate Gradle build. Hardware-only validation; unit-testable logic (statistics, CSV writer, sequencer, call registry) lives in pure-Java packages with JUnit 5 desktop tests.
---

# rio-bench

A WPILib 2026 robot project deployed to a physical RoboRIO 2 to
benchmark HAL call costs. Output is a versioned CSV fixture consumed
by the sim core's tier-1 backend (`docs/ARCHITECTURE.md` "Execution
tiers", `docs/OPEN_QUESTIONS.md` OQ-1 sub-decision 2). Until real
bench data exists the loader uses estimated values tagged
`unvalidated`.

This is a tooling deliverable, not a layer. It is the *only* way the
project gets honest RIO-2 HAL-call cost numbers; if it ships sloppy,
every tier-1 fidelity claim downstream is sloppy.

## Scope

**In scope:**
- Pure-Java statistics (mean, stddev, p99, outlier rate) over a series
  of microsecond timings.
- Pure-Java CSV writer producing the fixture format the sim core's
  loader will consume.
- Pure-Java sequencer that drives a fixed schedule of operating points
  (warmup → idle-bus → saturated-bus → multi-thread contention →
  done).
- Pure-Java call-class registry pinning the v0 set of HAL calls under
  measurement and their CSV labels.
- Hardware glue (`frc.robot.bench.runner`) that times each call class
  on the real RIO using `System.nanoTime()` and feeds samples into the
  sequencer.
- Auto-sweep entry from `autonomousInit` so the operator doesn't have
  to drive state changes from the DS.
- File output to `/home/lvuser/rio-bench/<wpilib-version>-<rio-firmware>.csv`
  on the RIO. The operator pulls it back to `references/rio2-hal-costs/`
  in the repo by hand.

**Out of scope (now):**
- The sim-core loader that consumes the CSV — that lives under
  `src/backend/tier1/` and is a future cycle.
- A wider HAL surface (DigitalIO, Analog, PWM, Counter, Encoder).
  v0 covers only what the shim already consumes; v1 widens.
- Cross-firmware sweeps. We capture the firmware version we ran
  against and that is the only datapoint the file claims to cover.
- Automated CSV transfer (FTP/SFTP/scp). Manual pull, with the path
  printed at the end of the run.

## Public surface

This is a deployed tool, not a library. The "public surface" callers
care about is the CSV format and the operator procedure.

### CSV format (v0)

Lines beginning with `#` are preamble, in order:

```
# rio-bench v0
# wpilib_version=<e.g. 2026.2.1>
# rio_firmware=<e.g. 9.0.0>
# rio_image=<e.g. 2026_v3.0>
# git_sha=<rio-bench source SHA>
# run_timestamp_iso8601=<UTC, fixed format>
# warmup_samples=<N>
# samples_per_block=<M>
# block_size=<K, calls per timing block>
# validated=<true|false>  // false = "unvalidated", set true only after sign-off
```

Then the header row:

```
call_class,operating_point,sample_count,mean_us,stddev_us,p99_us,outlier_rate
```

Then one row per `(call_class, operating_point)` pair, sorted by
`call_class` (lexicographic on label) then by `operating_point`
(`idle_bus` < `multi_thread` < `saturated_bus` lexicographic).
Decimal formatting is `Locale.ROOT` with fixed precision: `%.3f` for
microseconds, `%.6f` for outlier_rate.

### v0 call classes

| label                 | source         | notes                                                |
|-----------------------|----------------|------------------------------------------------------|
| `hal_get_fpga_time`   | `HALUtil`      | block-timed (K=100); sub-µs per call                 |
| `hal_notifier_wait`   | NotifierJNI    | timed across one wait period; uses a 1 ms scheduled alarm |
| `ds_state_read`       | `DriverStation`| `getMatchTime` + `isEnabled` block (K=10)            |
| `power_state_read`    | `RobotController`| `getBatteryVoltage` block (K=10)                   |
| `can_frame_read`      | TalonFX        | one `getRotorPosition` refresh                       |
| `can_frame_write`     | TalonFX        | one duty-cycle setpoint write                        |

`hal_notifier_wait`'s "cost" is the *jitter* on the 1 ms alarm, not
the wait itself. We measure `(actual_wake_us - target_wake_us)` per
sample.

### Operating points

| label           | how it's set up                                            |
|-----------------|------------------------------------------------------------|
| `idle_bus`      | no devices configured; CAN bus idle except DS heartbeat    |
| `saturated_bus` | 4 simulated TalonFX devices configured at 1 kHz status frames |
| `multi_thread`  | `idle_bus` setup + 3 worker threads on a `ThreadPoolExecutor` each looping on `hal_get_fpga_time` for the duration |

`saturated_bus` requires 4 physical TalonFX devices on the bus. If
fewer are present, the run logs a `# bus_devices=<N>` line in the
preamble and `validated=false`.

### Operator procedure

1. `cd tools/rio-bench/RioBenchmark && ./gradlew deploy` (team 6443).
2. Connect the DS, enable autonomous. The sweep runs to completion
   (~2 minutes). Disable when `[rio-bench] DONE: file at <path>`
   appears in the console.
3. `scp lvuser@10.64.43.2:/home/lvuser/rio-bench/<file>.csv references/rio2-hal-costs/`.
4. Edit the preamble: set `validated=true`. Commit.

## Design decisions

### D-RB-1 — measurement clock is `System.nanoTime()`, not `getFPGATime`.
Timing `HAL_GetFPGATime` with `HAL_GetFPGATime` is tautological
(measures zero overhead). `System.nanoTime()` bottoms out in
`clock_gettime(CLOCK_MONOTONIC)` on Linux — it's higher resolution
than the FPGA timestamp (1 ns vs 1 µs) and independent of the call
under test. The CSV reports microseconds because that's the unit the
sim core consumes; nano timings are scaled at sample time.

### D-RB-2 — block-timing for sub-µs calls.
Per-call timing is dominated by `nanoTime()` overhead at the µs
scale. We time blocks of K=100 invocations for `hal_get_fpga_time`
and divide. K=10 for the DS / power reads. K=1 for CAN frame
read/write (already 5–50 µs each). The block size is recorded in the
CSV preamble so future analysis can renormalize.

### D-RB-3 — outlier definition: `> mean + 5σ`.
Computed over the whole sample after warmup, including the tail.
This deliberately keeps the mean/stddev "honest" about real-time
jitter rather than reporting a sanitized robust mean. The
`outlier_rate` column is what the sim-core loader uses to model the
tail; consumers that want a clean mean can recompute by trimming
themselves.

### D-RB-4 — no GC / JIT compensation in v0.
A 1k-sample warmup loop triggers C2 JIT for the inner code path. We
do not pin GC pauses out of the sample window; they show up as
outliers, which is what the sim's tier-1 jitter model wants to see.
JVM flags (`-XX:+AlwaysPreTouch`) are passed via
`build.gradle` JVM args.

### D-RB-5 — CSV row order is deterministic.
Sorted by `(call_class, operating_point)` with fixed lexicographic
comparators. Preamble line order is fixed. Decimal formatting uses
`Locale.ROOT`. This makes diffs across runs meaningful — a row moving
means the schema changed, not the locale.

### D-RB-6 — auto-sweep, not DS-mode-per-point.
Operator drives a single autonomous-init entry. The sequencer
internally walks warmup → idle → saturated → multi-thread → done.
Less surface area for operator error; trade-off is no easy way to
re-run a single operating point without redeploying.

### D-RB-7 — pure-Java logic separated from HAL bindings.
`bench.stats`, `bench.csv`, `bench.sequencer`, `bench.calls` (the
registry, not the bindings) are unit-tested with JUnit 5 on desktop.
`bench.runner` and `bench.io` use real HAL calls and run on the RIO
only — no desktop unit tests for them; they're exercised by the
hardware run procedure.

## Validation status

**Pure-Java logic: validated.** The 41-test desktop suite runs
green under `./gradlew test` (JDK 17, no HAL JNI required). Test
plan went through 4 review rounds with the `test-reviewer` agent;
revision 4 + a P1 wording fix landed `ready-to-implement`.
Coverage:
- `StatisticsTest` (12) — mean / stddev / R-7 p99 / 5σ outlier
  rate, with edge cases (empty, single-sample, 1M-sample
  precision, block-timing division).
- `CsvWriterTest` (10) — preamble byte-equality with distinct
  per-field values, header position, sort order, HALF_UP
  rounding distinguished from HALF_EVEN at 1.0625, validated
  flag both polarities, optional bus_devices line, empty record
  list.
- `BenchmarkRecordTest` (3) — column order, comparator
  reflexivity / antisymmetry / primary-key dominance, value
  equality with explicit inequality cases.
- `SequencerTest` (10) — initial warmup state, off-by-one
  boundary checks at warmup/idle/saturated/multi_thread/done,
  per-class accounting, sample-bucket routing, unknown-class
  rejection, full Cartesian-product coverage.
- `OperatingPointTest` (2) — declaration-order pinning for P1,
  csvLabel table.
- `CallClassTest` (3) — six-member enum, csvLabel table,
  per-class blockSize.
- `BenchmarkPipelineTest` (1) — end-to-end Sequencer →
  Statistics → BenchmarkRecord → CsvWriter against a byte-equal
  fixture committed at `src/test/resources/pipeline_expected.csv`.

**HAL-bound code: built, untested.** `BenchmarkRunner`,
`CallBindings`, `RioFileSink` compile against WPILib 2026.2.1 but
have no desktop test path (they call HAL JNI). They will be
exercised the first time the project is deployed to a physical
RoboRIO 2; results land in `references/rio2-hal-costs/` per the
operator procedure above. Until then, the file the sim-core
loader will read is tagged `validated=false` (D-RB defaults).

## Known gotchas

- **The team number is in `.wpilib/wpilib_preferences.json`** — team
  6443. If the deploy target changes, that file is the source of
  truth, not `build.gradle`.
- **Java GC may run during measurement.** That is intentional — see
  D-RB-4 — but means the run-to-run variance is meaningfully nonzero
  even on the same hardware. Always commit the actual measured CSV;
  do not "average across runs" without recording each run.
- **`saturated_bus` needs real devices.** Without them, the operating
  point degrades to "idle-bus with 4 timeouts per cycle" and the
  numbers are not comparable to a real saturated bus. The run logs
  `bus_devices=<N>` and sets `validated=false` if the count is wrong.
- **CAN bench message ID is `0x1FFFFFFE`.** `CallBindings` uses this
  ID to provoke the HAL `FRCNetCommCANSessionMux*` calls without
  hitting any production device. If the bench is run on a bus where
  this ID is meaningful, the measurement is contaminated — change
  the constant in `CallBindings.java`.
- **`HAL_NOTIFIER_WAIT` measures `LockSupport.parkNanos(1ms)` jitter,
  not WPILib `Notifier`.** The implementation deliberately avoids
  the WPILib `Notifier` class because its callback model isn't
  amenable to per-call timing in a tight loop. Park-jitter on
  PREEMPT_RT is a reasonable proxy for FPGA-alarm IRQ jitter; if
  the sim core needs the real-Notifier number, the binding can be
  swapped without changing the schema.
- **Build precondition holds.** `./gradlew test` runs the full
  desktop suite without loading HAL JNI; `wpi.java.configureTestTasks(test)`
  in `build.gradle` did not interfere. If a future GradleRIO bump
  breaks this, the test-plan precondition section lists two
  remediations (set `includeDesktopSupport = true`, or remove the
  `configureTestTasks` call and configure manually).

## Open follow-ups

- Sim-core loader (Python or C++? Probably C++ in
  `src/backend/tier1/`). Future cycle.
- Cross-firmware comparator that diffs two CSVs and flags rows that
  shifted by >2σ. Future cycle, post-v1.
- REV bus operating point once REV is in scope (v1+).

## File layout

```
tools/rio-bench/RioBenchmark/
  build.gradle                    # GradleRIO 2026.2.1, JUnit 5 wired
  src/main/java/frc/robot/
    Main.java                     # stock
    Robot.java                    # autonomousInit triggers benchmark sweep
    RobotContainer.java           # stock (no commands)
    bench/
      stats/Statistics.java       # pure-Java; tested
      csv/CsvWriter.java          # pure-Java; tested
      csv/BenchmarkRecord.java    # pure-Java POD; tested
      sequencer/Sequencer.java    # pure-Java state machine; tested
      sequencer/OperatingPoint.java  # enum
      calls/CallClass.java        # enum + label/block-size table; tested
      runner/BenchmarkRunner.java # HAL-bound; not unit-tested
      runner/CallBindings.java    # one method per CallClass; HAL-bound
      io/RioFileSink.java         # writes /home/lvuser/rio-bench/<file>.csv
  src/test/java/frc/robot/bench/
    stats/StatisticsTest.java
    csv/CsvWriterTest.java
    csv/BenchmarkRecordTest.java
    sequencer/SequencerTest.java
    sequencer/OperatingPointTest.java
    calls/CallClassTest.java
    pipeline/BenchmarkPipelineTest.java
  src/test/resources/
    pipeline_expected.csv         # P1 byte-equal fixture
  TEST_PLAN.md                    # the test plan; reviewer-approved
```

## References

- `docs/ARCHITECTURE.md` lines 334–341 — the tooling-deliverable spec.
- `docs/OPEN_QUESTIONS.md` OQ-1 sub-decision 2 — tier hierarchy that
  motivates the table.
- `references/rio2-hal-costs/README.md` — sibling that documents how
  the file is consumed.
- `.claude/skills/hal-shim.md` — the HAL surfaces the v0 call set
  mirrors.
