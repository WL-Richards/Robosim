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
- File output to `rio-bench/<wpilib-version>-<rio-firmware>.csv`,
  resolved against the JVM cwd (which `frcRunRobot.sh` sets to
  `/home/lvuser` on a real RIO, so files land at
  `/home/lvuser/rio-bench/...` there; off-RIO they land under whichever
  directory the caller launched from)
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
call_class,operating_point,sample_count,mean_us,stddev_us,p50_us,p99_us,p999_us,outlier_rate
```

(Cycle B added the `p50_us` and `p999_us` columns adjacent to
`p99_us`. The CSV preamble version is unchanged at `# rio-bench v0` —
no loader exists yet to require a version bump; see `D-RB-8`.)

Then one row per `(call_class, operating_point)` pair, sorted by
`call_class` (lexicographic on label) then by `operating_point`
(`idle_bus` < `multi_thread` < `saturated_bus` <
`saturated_multi_thread` lexicographic). Decimal formatting is
`Locale.ROOT` with fixed precision: `%.3f` for the five microsecond
columns (`mean`, `stddev`, `p50`, `p99`, `p999`), `%.6f` for
`outlier_rate`.

### v0 call classes

| label                 | source         | notes                                                |
|-----------------------|----------------|------------------------------------------------------|
| `hal_get_fpga_time`   | `HALUtil`      | block-timed (K=100); sub-µs per call                 |
| `hal_notifier_wait`   | NotifierJNI    | timed across one wait period; uses a 1 ms scheduled alarm |
| `ds_state_read`       | `DriverStation`| `getMatchTime` + `isEnabled` block (K=10)            |
| `power_state_read`    | `RobotController`| `getBatteryVoltage` block (K=10)                   |
| `can_frame_read`      | `CANJNI`       | one `FRCNetCommCANSessionMuxReceiveMessage` to bench-reserved ID `0x1FFFFFFE` (no-match expected). HAL session-mux call cost only — does NOT include vendor-API overhead like `TalonFX.getRotorPosition().refresh()` (status-frame decoding, Phoenix6 caching, signal fusion). Real devices on the bus contribute as background traffic affecting the session mux's RX queue depth, not as the measured target. |
| `can_frame_write`     | `CANJNI`       | one `FRCNetCommCANSessionMuxSendMessage` to bench-reserved ID `0x1FFFFFFE` with `CAN_SEND_PERIOD_NO_REPEAT`. HAL send-call cost; no production device acks. Same scope caveat as `can_frame_read`. |

`hal_notifier_wait`'s "cost" is the *jitter* on the 1 ms alarm, not
the wait itself. We measure `(actual_wake_us - target_wake_us)` per
sample.

### Operating points

The operating-point → background-load mapping is centralized in the
pure-Java `bench.runner.WorkerSpec` table; the HAL-bound runner reads
`WorkerSpec.forPoint(phase)` to decide what to spawn. The mapping is
unit-tested by `WorkerSpecTest`. The two-dimensional table is
`(cpuWorkers ∈ {0, 3}) × (canSpamWorkers ∈ {0, 1, 4, 8})` — eight
operating points total.

| label                          | cpu workers | CAN spam workers | notes                                                                                                                              |
|--------------------------------|-------------|------------------|------------------------------------------------------------------------------------------------------------------------------------|
| `idle_bus`                     | 0           | 0                | quiescent baseline; CAN bus idle except DS heartbeat                                                                               |
| `saturated_bus`                | 0           | 1                | 1 worker loops on `FRCNetCommCANSessionMuxSendMessage` (ID `0x1FFFFFFD`) to software-saturate the bus; cycle A, see `D-RB-8`        |
| `multi_thread`                 | 3           | 0                | `idle_bus` setup + 3 worker threads each looping on `hal_get_fpga_time` for the duration                                            |
| `saturated_multi_thread`       | 3           | 1                | cross-product cell — both saturated bus AND CPU workers active; cycle A                                                             |
| `saturated_bus_4`              | 0           | 4                | heavier software-driven CAN load tier; cycle C                                                                                      |
| `saturated_bus_8`              | 0           | 8                | heaviest software-driven CAN load tier; cycle C                                                                                     |
| `saturated_bus_4_multi_thread` | 3           | 4                | cross-product of moderate spam tier with CPU workers; cycle C                                                                       |
| `saturated_bus_8_multi_thread` | 3           | 8                | cross-product of heaviest spam tier with CPU workers; cycle C                                                                       |

The `# bus_devices=<N>` preamble line records how many physical CAN
devices were alive on the bus during the sweep. Cycle C makes this
runtime-determined (via the `bench.runner.DeviceScan` helper that
probes TalonFX IDs at boot) rather than operator-configured. The
value is informational only — software-driven saturation makes
measurements reproducible regardless of the count, and the
`bus_devices` field exists so the sim core's tier-1 backend can
interpret per-row HAL-call cost in context (real-device baseline
traffic is whatever the scan found at run time).

### Operator procedure

1. `cd tools/rio-bench/RioBenchmark && ./gradlew deploy` (team 6443).
2. Connect the DS, enable autonomous. The sweep runs to completion
   (~2 minutes). The runner emits `[rio-bench] progress: NN%` lines
   to the DataLog at every 5% boundary across the full sweep
   (warmup + every operating point), so the operator can confirm
   forward progress without watching the timer. Disable when
   `[rio-bench] DONE: file at <path>` appears in the console.
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
`bench.runner` is split into a pure-Java seam (`WorkerSpec` —
operating-point → load-profile mapping; unit-tested) and HAL-bound
glue (`BenchmarkRunner.startWorkersFor`, `CallBindings` — call into
real HAL JNI; not unit-tested). `bench.io` is HAL-bound. Anything
HAL-bound runs on the RIO only and is exercised by the hardware
run procedure.

### D-RB-8 — bus saturation is software-driven, not operator-hardware-driven.
v0 defined `saturated_bus` as "4 physical TalonFX devices on the bus
at 1 kHz status frames" and gated `validated=false` on the actual
device count. Cycle A switches to a software-driven model: the
runner spawns a CAN-frame-spam worker that hammers
`FRCNetCommCANSessionMuxSendMessage` at message ID `0x1FFFFFFD`
(distinct from the bench measurement ID `0x1FFFFFFE`) for the
duration of any phase whose `WorkerSpec.canSpam()` is true.
Trade-off: saturation is now reproducible across runs without
requiring specific hardware, at the cost of a different load profile
than physically-present devices would produce (the spam worker
generates pure HAL-call traffic; real devices generate
status-frame-driven RX traffic that the spam worker does not). The
cycle-C device-count sweep will measure the hardware-driven case
separately; it does not replace this software-driven baseline.
**Loader version note.** The CSV preamble version is unchanged at
`# rio-bench v0` — no schema field is added or removed. The
sim-core loader (whichever cycle introduces it; cycle B is the
likely candidate) inherits the responsibility of deciding whether
to bump the version so loaders can tell hardware-saturated v0
files apart from software-saturated cycle-A-and-later files.
Cycle A defers that decision because no loader exists yet.

## Validation status

**Pure-Java logic: validated.** The 76-test desktop suite runs
green under `./gradlew test` (JDK 17, no HAL JNI required).
Coverage history:
- v0 plan (`TEST_PLAN.md`) went through 4 review rounds; revision 4
  + a P1 wording fix landed `ready-to-implement` at 41 tests.
- Cycle A plan (`TEST_PLAN_RB_A.md`) added `SATURATED_MULTI_THREAD`,
  the pure-Java `WorkerSpec` table, and software-driven bus
  saturation. Two review rounds; revision 2 landed
  `ready-to-implement`. Net delta: +7 tests; 41 → 48.
- Cycle B plan (`TEST_PLAN_RB_B.md`) added the `p50_us` and
  `p999_us` columns inline, lifted the R-7 helper to a
  package-private `Statistics.percentileNs` so it is testable at
  arbitrary `p`. Three review rounds; revision 3 landed
  `ready-to-implement`. Net delta: +6 tests; 48 → 54.
- Cycle C plan (`TEST_PLAN_RB_C.md`) parameterized the spam-worker
  count (`boolean canSpam` → `int canSpamWorkers`), added four
  new operating points (`SATURATED_BUS_4` / `_8` and their
  `_MULTI_THREAD` cross-products), and added a runtime
  `DeviceScan` helper that records the alive CAN device count
  in the preamble. Five review rounds; revision 5 landed
  `ready-to-implement`. Net delta: +22 tests (4 `WorkerSpecTest`,
  4 `SequencerTest`, 14 `DeviceScanTest`); 54 → 76. Multiple
  rewrites in place: all `OperatingPointTest`, all
  `SequencerTest` transition/done/Cartesian-product tests, and
  the `WorkerSpecTest` migration. Fixture regenerated to 48 rows.

Coverage by test class:
- `StatisticsTest` (17) — mean / stddev / 5σ outlier rate / R-7
  percentiles at p={0.5, 0.99, 0.999} via both `compute` wiring
  and the lifted `percentileNs` helper; edge cases (empty,
  single-sample, 1M-sample precision, small-N at p=0.999,
  block-timing division applied to all three percentiles).
- `CsvWriterTest` (10) — preamble byte-equality with distinct
  per-field values, header position (9-column form), sort
  order, HALF_UP rounding distinguished from HALF_EVEN at
  1.0625 across all five µs columns, validated flag both
  polarities, optional bus_devices line, empty record list.
- `BenchmarkRecordTest` (4) — column order (9-column row),
  comparator reflexivity / antisymmetry / primary-key
  dominance, value equality with explicit inequality cases,
  `Stats` record positional-constructor → accessor mapping
  pinned independently of `toCsvRow` (catches a Java field
  reorder that compiles but silently corrupts every row).
- `SequencerTest` (15) — initial warmup state, off-by-one
  boundary checks at all eight phase transitions
  (warmup → idle_bus → saturated_bus → multi_thread →
  saturated_multi_thread → saturated_bus_4 → saturated_bus_8
  → saturated_bus_4_multi_thread → saturated_bus_8_multi_thread
  → done), per-class accounting, sample-bucket routing,
  unknown-class rejection, full 6×8 Cartesian-product coverage.
- `OperatingPointTest` (3) — declaration-order pinning for P1
  (8-member enum, ordinals 0..7; first four preserved from
  cycle A), csvLabel table, lex-sort invariant for
  `CsvWriter`'s row order (8-entry list under cycle C).
- `WorkerSpecTest` (9) — per-cell `forPoint` mapping for each
  of the eight operating points, plus a cross-product
  distinctness invariant (set-of-WorkerSpec size = 8). Field
  type is `(int cpuWorkers, int canSpamWorkers)` after cycle
  C migrated from `boolean canSpam`; the per-cell tests pin
  the int values (`SATURATED_BUS_4 → (0, 4)`, etc.) so a
  clamping regression to cycle A's 0/1 semantic is caught.
- `DeviceScanTest` (14) — pure-Java cycle-C component pinning
  the runtime-scan record's invariants and factory:
  factory correctness on mixed/empty/full alive sets;
  factory rejection of null / negative-start / zero-length
  inputs; constructor rejection of unsorted IDs, duplicates,
  count mismatch, IDs above and below the probe range,
  inverted range, negative range start; structural equality
  (`List<Integer>` field type so `record`-derived `equals`
  is value-based — `int[]` would silently break it).
- `CallClassTest` (3) — six-member enum, csvLabel table,
  per-class blockSize.
- `BenchmarkPipelineTest` (1) — end-to-end Sequencer →
  Statistics → BenchmarkRecord → CsvWriter against a byte-equal
  fixture committed at `src/test/resources/pipeline_expected.csv`
  (cycle C: 48 rows × 9 columns; per-cell sample formula
  `v = ((6*p.ordinal() + c.ordinal()) + 1) * 1000` with
  spread `[v-1, v, v, v+1]` per cell — same as cycle B but
  the cell-index space grew from `[1, 24]` to `[1, 48]` as
  ordinals were added).

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
- **CAN bench message IDs are reserved across the bench.**
  `CallBindings` uses two IDs:
  - `0x1FFFFFFE` for `CAN_FRAME_READ` / `CAN_FRAME_WRITE`
    measurement (provokes the HAL `FRCNetCommCANSessionMux*`
    calls without hitting any production device);
  - `0x1FFFFFFD` for the `WorkerSpec` CAN-spam worker driving
    `SATURATED_BUS` / `SATURATED_MULTI_THREAD` saturation
    (cycle A; see `D-RB-8`).
  If the bench is run on a bus where either ID is meaningful,
  the measurement is contaminated — change the constants in
  `CallBindings.java`.
- **`bus_devices` preamble is informational under cycle A.** The
  line is still emitted when the runner records a device count,
  but it no longer gates `validated=false` and `saturated_bus`
  no longer requires hardware. Cycle C will reintroduce
  device-count-driven validity once it lands.
- **No `default` branch in `WorkerSpec.forPoint`.** The exhaustive
  switch expression over `OperatingPoint` is the compile-time
  guard that adding a fifth member without updating `WorkerSpec`
  fails the build. A `default` branch would silently swallow the
  missing case. The rule is mirrored as an inline source-code
  comment so reviewer turnover does not erode it.
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
  `src/backend/tier1/`). Future cycle. Whichever cycle introduces
  the loader inherits the version-bump decision flagged in
  `D-RB-8`.
- Notifier-jitter histogram sidecar. Cycle B added p50/p99/p999
  inline, which is a 3-point sketch of the
  `HAL_NOTIFIER_WAIT` jitter distribution; if the first hardware
  run shows multimodal jitter the percentiles obscure, add a
  parallel `<wpilib>-<rio>-notifier-hist.csv` sidecar with bucket
  counts. Deferred until measurements warrant.
- An `unsorted_input` test for `Statistics.compute` —
  pre-existing gap inherited from v0; every existing test
  submits already-sorted samples, so a refactor that drops the
  internal `Arrays.sort` step would pass the suite. Plan-level
  acknowledgement is in `TEST_PLAN_RB_B.md`. One-test fix.
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
      runner/CallBindings.java    # one method per CallClass + spamCanFrame() + probeTalonFxAlive(); HAL-bound
      runner/WorkerSpec.java      # pure-Java; OperatingPoint → (cpuWorkers, canSpamWorkers) table; tested
      runner/DeviceScan.java      # pure-Java; alive CAN device IDs from a HAL probe; tested
      io/RioFileSink.java         # writes rio-bench/<file>.csv (relative; cwd-resolved)
  src/test/java/frc/robot/bench/
    stats/StatisticsTest.java
    csv/CsvWriterTest.java
    csv/BenchmarkRecordTest.java
    sequencer/SequencerTest.java
    sequencer/OperatingPointTest.java
    runner/WorkerSpecTest.java
    calls/CallClassTest.java
    pipeline/BenchmarkPipelineTest.java
  src/test/resources/
    pipeline_expected.csv         # P1' byte-equal fixture (24 rows under cycle A)
  TEST_PLAN.md                    # v0 test plan; reviewer-approved
  TEST_PLAN_RB_A.md               # cycle-A delta plan; reviewer-approved
  TEST_PLAN_RB_B.md               # cycle-B delta plan; reviewer-approved
  TEST_PLAN_RB_C.md               # cycle-C delta plan; reviewer-approved
```

## References

- `docs/ARCHITECTURE.md` lines 334–341 — the tooling-deliverable spec.
- `docs/OPEN_QUESTIONS.md` OQ-1 sub-decision 2 — tier hierarchy that
  motivates the table.
- `references/rio2-hal-costs/README.md` — sibling that documents how
  the file is consumed.
- `.claude/skills/hal-shim.md` — the HAL surfaces the v0 call set
  mirrors.
