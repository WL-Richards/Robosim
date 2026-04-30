# rio-bench status

WPILib 2026 Java/Gradle robot project under `tools/rio-bench/`. v0
tooling deliverable that benchmarks RoboRIO 2 HAL call costs and
emits the CSV fixture the sim core's tier-1 backend will consume.
See `.claude/skills/rio-bench.md`.

## Headline

- Pure-Java logic: **76/76 green** under `./gradlew test` on JDK 17,
  no HAL JNI required (41/41 at v0 → 48/48 after cycle A → 54/54
  after cycle B → 76/76 after cycle C).
- v0, cycle-A, cycle-B, and cycle-C test plans all
  `ready-to-implement` per the `test-reviewer` agent (cycle C took
  five review rounds).
- HAL-bound code (`BenchmarkRunner`, `CallBindings`, `WorkerSpec`'s
  runner-side spawn logic, `RioFileSink`, `Robot.autonomousInit`
  wiring) compiles against WPILib 2026.2.1 and is ready for the
  first hardware run. Cycle C's HAL-bound device-scan
  (`CallBindings.probeTalonFxAlive`) does not yet exist — see
  "What's next."
- Sweep duration grew from ~2 min (v0/A/B; 24 cells) to ~5 min
  (cycle C; 48 cells). Operator procedure unchanged otherwise.
- `BenchmarkRunner` emits `[rio-bench] progress: NN%` lines to the
  DataLog at every 5% boundary across the full sweep so the operator
  can confirm forward progress on the DS console.

## Recent changes

**Cycle C** (post-v0 fidelity follow-up, this revision)

- `WorkerSpec` migrated from `(int cpuWorkers, boolean canSpam)`
  to `(int cpuWorkers, int canSpamWorkers)`. The cycle-A boolean
  was always 0 or 1 spam workers; integer generalizes for the
  load-tier sweep.
- Four new operating points added:
  - `SATURATED_BUS_4` `(0, 4)` and `SATURATED_BUS_8` `(0, 8)`
    — heavier software-driven CAN load tiers.
  - `SATURATED_BUS_4_MULTI_THREAD` `(3, 4)` and
    `SATURATED_BUS_8_MULTI_THREAD` `(3, 8)` — cross-products
    of the heavier tiers with CPU-contention workers.
  - Total: 8 operating points; CSV grows to 6×8 = 48 rows per
    sweep.
- New pure-Java component `bench.runner.DeviceScan`: a record
  + factory that consumes a HAL-bound CAN-device probe result
  (`boolean[]` of alive-by-index) and produces a structurally-
  comparable record `(int aliveCount, List<Integer> aliveIds,
  int probeRangeStartInclusive, int probeRangeEndInclusive)`.
  `List<Integer>` rather than `int[]` so the Java `record`'s
  derived `equals`/`hashCode` are value-based.
- Skill updated: cycle-C operating-points table (8 entries),
  validation status (76 tests), file layout (`DeviceScan.java`
  added).
- `pipeline_expected.csv` regenerated to 48 rows. The 24
  cycle-B cells are byte-identical (cell formula and ordinals
  preserved); 24 new rows added.

**Cycle B** (post-v0 fidelity follow-up, prior revision)

- CSV row schema gains two new percentile columns: `p50_us` and
  `p999_us`, adjacent to the existing `p99_us`. New header is
  `call_class,operating_point,sample_count,mean_us,stddev_us,p50_us,p99_us,p999_us,outlier_rate`.
- `Statistics.Stats` record widens to six fields:
  `(meanUs, stddevUs, p50Us, p99Us, p999Us, outlierRate)`.
- `Statistics`'s previously-private `quantileR7` helper is lifted
  to a package-private `percentileNs(long[] sorted, double p)` so
  it is unit-testable at arbitrary `p` values, not just
  transitively at `p=0.99` via `compute`.
- CSV preamble version is unchanged at `# rio-bench v0` (no loader
  exists yet to require a version bump; deferred per `D-RB-8`).
- No HAL-bound code change — `CallBindings.timeNotifierWait`
  already returned `(actual_wake - target_wake)` ns, so the new
  percentiles are derived from the same samples.
- `pipeline_expected.csv` regenerated (24 rows × 9 columns; sample
  formula now `((6*p + c) + 1) * 1000` with spread `[v-1, v, v,
  v+1]` per cell — see `TEST_PLAN_RB_B.md` `P1''` for the
  rationale).
- Skill updated: cycle-B coverage block under Validation Status,
  CSV-format header line, file layout reflects the cycle-B plan
  doc, and a histogram-sidecar follow-up was added under "Open
  follow-ups" (only worth implementing if the hardware run shows
  multimodal jitter the percentiles obscure).

**Cycle A** (post-v0 fidelity follow-up, prior revision)

- New `OperatingPoint`: `SATURATED_MULTI_THREAD` (ordinal 3,
  declared last so v0 ordinals are preserved). Models the realistic
  match condition where CAN bus load and worker-thread contention
  are concurrent, not isolated.
- New pure-Java `bench.runner.WorkerSpec` record + factory: maps
  each `OperatingPoint` to `(int cpuWorkers, boolean canSpam)`.
  Replaces the v0 `if (phase != MULTI_THREAD)` guard in
  `BenchmarkRunner.startWorkersFor`. Unit-tested by
  `WorkerSpecTest` (5 tests).
- Bus saturation is now software-driven (`D-RB-8`): a CAN-frame-
  spam worker on message ID `0x1FFFFFFD` runs during phases whose
  `WorkerSpec.canSpam()` is true. v0 required 4 physical TalonFX
  devices; cycle A drops that requirement.
- `pipeline_expected.csv` fixture regenerated: 18 → 24 rows. The
  18 v0 cells are byte-identical (verified by diff against v0
  fixture). The six new rows cover the `SATURATED_MULTI_THREAD`
  column for each call class.
- Skill updated with cycle-A operating-points table, `D-RB-8`
  decision entry, validation status, and the `WorkerSpec`-related
  gotchas.

## What's next

The produced CSV is tagged `validated=false` until the operator runs
the sweep on a physical RIO 2 and commits the result to
`references/rio2-hal-costs/`. Cycle C's hardware contract is the
same shape as A/B (operator deploys, runs autonomous, scp's the
file back), but two cycle-C HAL-bound pieces are still TODO before
the first hardware run:

- **`CallBindings.probeTalonFxAlive(int firstId, int lastId)`**
  — the HAL-bound counterpart to `DeviceScan.fromProbeResults`.
  Should call CTRE Phoenix6 `TalonFX(id).getVersion().isAlive()`
  (or equivalent) for each ID in the range and return a
  `boolean[]` for the factory to consume. Not unit-tested per
  `D-RB-7`.
- **`Robot.autonomousInit` wiring** to call the scan at boot,
  populate `RunMetadata.busDevices` from the scan count, and
  log alive IDs to the DataLog before the sweep starts.

Both are mechanical and small, but they're the bridge between
cycle C's tested pure-Java seam and the real CAN bus.

A post-cycle-C follow-up is recorded in the skill's "Open
follow-ups" — a hardware-driven calibration cycle (RB-cycle-D)
that compares software-spam load to real CTRE status-frame load,
bounding the fidelity gap of `D-RB-8`.

## References

- `tools/rio-bench/RioBenchmark/TEST_PLAN.md` — v0 test plan.
- `tools/rio-bench/RioBenchmark/TEST_PLAN_RB_A.md` — cycle-A test plan.
- `tools/rio-bench/RioBenchmark/TEST_PLAN_RB_B.md` — cycle-B test plan.
- `tools/rio-bench/RioBenchmark/TEST_PLAN_RB_C.md` — cycle-C test plan.
