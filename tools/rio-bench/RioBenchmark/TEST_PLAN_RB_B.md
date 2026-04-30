# rio-bench RB-cycle-B test plan

**Scope:** add `p50_us` and `p999_us` columns to every CSV row, in
addition to the existing `p99_us`. The motivation is tier-1
scheduler-jitter modeling: `HAL_NOTIFIER_WAIT`'s `(actual_wake -
target_wake)` distribution is the seed for the sim core's loop-
period jitter model, and a single percentile (p99) is too sparse a
sketch of that distribution to drive a stochastic process. p50 + p99
+ p999 gives a 3-point sketch — enough to characterize tail weight
without a full histogram.

**Status:** **draft, awaiting `test-reviewer` verdict**.

**Delta convention.** This plan only specifies tests that are
**new** or **modified** vs the v0 plan (`TEST_PLAN.md`) and the
cycle-A plan (`TEST_PLAN_RB_A.md`). Tests not mentioned here are
unchanged. Each entry that supersedes a prior test names the
predecessor's ID (e.g. "supersedes v0 `S5`").

## Context for the reviewer

### Cycle B's place in the post-v0 push

This is the second cycle of a three-cycle post-v0 fidelity push:

- **Cycle A** (landed): `SATURATED_MULTI_THREAD` cross-product
  operating point + software-driven CAN saturation
  (`D-RB-8`). 48-test suite green.
- **Cycle B** (this plan): notifier jitter percentiles —
  `p50_us`, `p999_us` columns added inline to every row.
- **Cycle C** (queued): CAN device-count sweep with runtime
  device scan.

### What changes implementation-wise

1. **`Statistics.Stats` record gains two fields.** Order:
   `(meanUs, stddevUs, p50Us, p99Us, p999Us, outlierRate)`. The
   `p99Us` field stays in its v0 position relative to mean/stddev
   so the reading order matches the column order in the CSV
   (low → high percentile, then outlier rate).

2. **`Statistics.compute` returns the extended `Stats`** with
   the three percentiles all derived from the same R-7
   interpolation as v0's `p99Us`. Internally, the existing
   private `quantileR7` helper is **lifted to a package-private
   static** `Statistics.percentileNs(long[] sortedSamples, double p)`
   so the test plan can pin its general behavior at any `p` (not
   just the three points the row carries). v0's existing
   p99-derivation tests continue to exercise the same code path
   via `compute`; the lifted helper gets new tests for `p ∈
   {0.0, 0.5, 0.999, 1.0}` boundary behavior.

3. **CSV header row gains two columns.** New header:
   `call_class,operating_point,sample_count,mean_us,stddev_us,p50_us,p99_us,p999_us,outlier_rate`.
   Column order is "low → high percentile, then outlier rate"
   so the file is read top-to-bottom in increasing order of
   what each column captures.

4. **`BenchmarkRecord.toCsvRow` emits two extra `%.3f` fields.**
   The `Comparator` is unchanged (still keyed on
   `(callClass.csvLabel, operatingPoint.csvLabel)`); the
   `equals`/`hashCode` derived by `record` automatically pick up
   the wider `Stats`.

5. **CSV preamble version is unchanged at `# rio-bench v0`.** No
   loader exists yet; the version-bump decision is deferred per
   `D-RB-8`. Existing v0 fixtures cannot round-trip through a
   cycle-B reader as-is, but no reader exists to break.

### What does NOT change

- Operating points, call classes, block sizes — unchanged from
  cycle A.
- Sequencer mechanics, `WorkerSpec` table, runner orchestration,
  CAN-spam worker — all unchanged.
- Outlier definition (5σ, D-RB-3), GC policy (D-RB-4), row sort
  order (D-RB-5), auto-sweep (D-RB-6), HAL/logic split (D-RB-7),
  software-driven saturation (D-RB-8) — all unchanged.
- The `HAL_NOTIFIER_WAIT` measurement itself: `CallBindings`
  already returns `Math.abs(actual_wake_ns - target_wake_ns)` per
  sample, which is exactly the jitter the new percentiles
  characterize. **No HAL-bound code change is required for
  cycle B.**

### What this cycle deliberately defers

- **Histogram sidecar.** The user's call: ship the percentile
  expansion now; add a sidecar `*-notifier-hist.csv` only if the
  hardware run shows multimodal jitter that the 3-point sketch
  obscures. Tracked in skill "Open follow-ups" once cycle B
  lands.
- **Per-call-class custom percentile sets.** Every row carries
  the same three percentiles. A future cycle could let each call
  class declare its own percentile set, but that's CSV-schema
  surgery for an unproven win.

## Suite-level concerns

- **Framework, determinism, no-HAL-imports, no-wall-clock** rules
  inherited from v0 plan.
- **`Stats` record field order is part of the test contract.**
  Multiple tests rely on the constructor argument order
  `(meanUs, stddevUs, p50Us, p99Us, p999Us, outlierRate)` and
  the corresponding column order in `toCsvRow`. A field-order
  shuffle that compiles successfully would silently swap values
  in the CSV. New test `R4` pins this.
- **Build precondition** (inherited from v0): `./gradlew test`
  must execute the desktop suite without loading HAL JNI.
- **Test-file naming.** Changes touch four test files plus the
  fixture:
  - `StatisticsTest` gains `S13`, `S14`, `S15`, `S16`, `S17`
    (general percentile + p50 + p999 + boundary).
  - `CsvWriterTest` rewrites `C1`, `C2`, `C3`, `C4` in place
    (header / row format expand).
  - `BenchmarkRecordTest` rewrites `R1` in place (toCsvRow row
    format) and gains `R4` (Stats-field-order pin).
  - `BenchmarkPipelineTest.P1'` is rewritten in place (fixture
    regenerated with two new columns per row, 24 rows × wider
    schema).
  - `pipeline_expected.csv` regenerated.

## Per-test specifications

### `StatisticsTest`

#### S13 — `percentile_ns_helper_returns_R7_value_for_canonical_dataset`
- **New.** Pins the lifted `Statistics.percentileNs(long[], double)`
  helper that `compute` now calls three times.
- Layer / contract: `bench.stats.Statistics.percentileNs` —
  R-7 interpolation, package-private.
- Bug class: a refactor that mistakenly uses nearest-rank instead
  of R-7 would pass the cycle-A `S5` (which checks `p99` via
  `compute`) only because both formulations happen to return the
  same value for v0's specific dataset. A direct test of the
  lifted helper at multiple `p` values catches this without
  depending on `compute`'s wiring.
- Inputs: sorted `long[]` `[10, 20, 30, 40, 50, 60, 70, 80, 90, 100]`
  ns; one assertion per `p` value:
  - `p = 0.0` → R-7: `h = (10-1)*0 + 1 = 1`, returns
    `sorted[0] = 10`.
  - `p = 0.5` → `h = 5.5`, fraction = 0.5, returns
    `sorted[4] + 0.5*(sorted[5]-sorted[4]) = 50 + 0.5*10 = 55`.
  - `p = 0.99` → `h = 9.91`, returns `90 + 0.91*10 = 99.1`
    (matches v0 `S5` derivation).
  - `p = 0.999` → `h = (10-1)*0.999 + 1 = 9.991`, returns
    `90 + 0.991*10 = 99.91`.
  - `p = 1.0` → `h = 10`, returns `sorted[9] = 100`.
- Tolerance: `< 1e-12`.
- Citation: Hyndman & Fan 1996 (R-7).

#### S14 — `compute_returns_p50_via_R7_on_known_dataset`
- **New.** Pins the wiring from `compute` → `percentileNs(., 0.5)`.
- Layer / contract: `Statistics.compute(samplesNs, blockSize).p50Us()`.
- Bug class: a wiring defect where `compute` calls
  `percentileNs(., 0.99)` for both p50 and p99 (or any other
  argument-collision). `S14` and the existing v0 `S5` together
  pin the three percentile arguments are distinct.
- Inputs: same dataset as `S13`, blockSize=1.
- Expected: `compute(...).p50Us() = 0.055` µs (= 55 ns / 1000).
- Tolerance: `< 1e-12`.

#### S15 — `compute_returns_p999_via_R7_on_known_dataset`
- **New.** Symmetric to `S14`.
- Layer / contract: `Statistics.compute(samplesNs, blockSize).p999Us()`.
- Bug class: wiring defect where `compute` passes the wrong `p`
  to `percentileNs` for the p999 column.
- Inputs: same dataset, blockSize=1.
- Expected: `compute(...).p999Us() = 0.09991` µs (= 99.91 ns / 1000).
- Tolerance: `< 1e-12`.

#### S16 — `compute_p50_and_p999_respect_block_size_division`
- **New.** Mirrors v0 `S9` (block-timing) but for the new percentiles.
- Layer / contract: per-call division applied to all three
  percentiles, not just p99.
- Bug class: a refactor that adds the new percentile fields but
  forgets to divide them by `blockSize` before returning. The cell
  would report block-totals (factor-of-100 too high for
  `HAL_GET_FPGA_TIME`) for p50/p999 while p99 looks correct —
  exactly the silent-corruption mode the v0 `S9` was written to
  catch for the original p99.
- Inputs: 10 samples each = 10000 ns, blockSize=100.
- Expected: `compute(...)` returns
  `p50Us = p99Us = p999Us = 0.1` (all three per-call values are
  equal because samples are identical).
- Tolerance: equality.

#### S17 — `compute_p999_well_defined_for_three_sample_input`
- **New.** Mirrors v0 `S6` (small-N edge case) for p999.
- Layer / contract: `compute` with `n = 3`.
- Bug class: a small-N branch that throws `IndexOutOfBounds` or
  returns NaN at `p = 0.999` because `(n-1)*0.999 + 1 = 2.998`
  is very close to the array's last index. v0 `S6` checked this
  for p99; cycle B's higher-percentile addition needs its own
  small-N pin because the floor/fraction arithmetic differs.
- Inputs: `[10, 20, 30]` ns, blockSize=1.
- Expected: `p999Us = 0.02998` µs. R-7 derivation:
  `h = (n-1)*p + 1 = 2*0.999 + 1 = 2.998`; floor = 2;
  fraction = 0.998; `lo = sorted[floor-1] = sorted[1] = 20`;
  `hi = sorted[floor] = sorted[2] = 30`;
  `p999 = lo + fraction*(hi - lo) = 20 + 0.998*10 = 29.98` ns.
  Per-call (blockSize=1) = 29.98 ns = `0.02998` µs.
- Tolerance: `< 1e-12`.
- The test also asserts no exception is thrown — small-N
  branches must not throw at `p = 0.999`. Note: v0 `S6` checks
  the same dataset at `p = 0.99` and expects `0.0298` µs;
  `S17` checks `p = 0.999` and expects `0.02998` µs. The two
  expected values look similar at a glance; do not confuse
  them.

> **Other v0 `StatisticsTest` cases** (`S1`–`S12`) are unchanged.
> They exercise mean / stddev / outlier-rate / R-7 p99 paths whose
> implementations are not touched by cycle B (the lift of
> `quantileR7` → `percentileNs` is structural; the v0 `p99Us`
> output remains byte-identical).

### `CsvWriterTest`

#### C1' — `writes_preamble_lines_with_correct_order_and_values`
- **Supersedes:** v0 `C1`. Header line widens from 7 columns to
  9 columns; preamble is unchanged.
- Layer / contract: `CsvWriter.write` preamble + header.
- Bug class: identical to v0 `C1` (preamble field-assignment
  swap, line ordering); plus the new bug class — a header that
  drops `p50_us` or `p999_us`, or inserts them in the wrong
  position relative to `p99_us`, which would silently change
  every downstream column index.
- Inputs: same `RunMetadata` fixture as v0 `C1`.
- Expected: the file's first 11 lines are identical to v0's
  expected first 11 lines for the preamble (10 lines starting
  with `# `), followed by the new header row at line 11:
  `call_class,operating_point,sample_count,mean_us,stddev_us,p50_us,p99_us,p999_us,outlier_rate\n`.

#### C2' — `writes_header_row_after_preamble`
- **Supersedes:** v0 `C2`. Header literal updated to the
  9-column form.
- Layer / contract: header position.
- Bug class: header drift would silently shift columns. With
  three percentile columns now, the bug class is wider — a
  reorder of the three percentiles or a swap with `outlier_rate`
  would still pass `C2` if `C2` only checked the column count.
  The literal-equality check still pins position-by-position.
- Inputs: empty record list, valid metadata.
- Expected: line after preamble is exactly the new header
  (literal above).

#### C3' — `sorts_records_by_call_class_then_operating_point`
- **Supersedes:** v0 `C3`. Sort key unchanged; per-row
  formatter widens. The point of the test is sort behavior, not
  row width; updating it preserves the original intent.
- Layer / contract: row sort comparator.
- Bug class: row order depending on insertion order (per D-RB-5).
- Inputs: 6 records spanning 3 call classes × 2 operating points
  (cycle B leaves the test's input deliberately scoped to two
  ops to keep the assertion compact; the four-op-point coverage
  lives in `P1'`'s 24-row fixture). Per-record `Stats` values
  are arbitrary fixed values **distinct per record** so a
  row-swap is visible in the byte-equal output.
- Expected: same 6 rows in `idle_bus < saturated_bus` order
  per the lex-sort invariant pinned by `O3`. Per-row content
  uses the new 9-column format; specific `Stats` values are
  inlined in the test.

#### C4' — `formats_microsecond_columns_with_locale_root_half_up`
- **Supersedes:** v0 `C4`. The HALF_UP-vs-HALF_EVEN distinction
  must hold for **all five** µs columns now (`mean`, `stddev`,
  `p50`, `p99`, `p999`), not just `mean`. v0's test asserted
  HALF_UP at `meanUs = 1.0625`; cycle B extends this to ensure
  the four other µs columns route through the same formatter.
- Layer / contract: `String.format(Locale.ROOT, "%.3f", x)` is
  applied to all five µs columns.
- Bug class: a refactor that hand-formats the new percentile
  columns with `BigDecimal.setScale(3, HALF_EVEN)` (banker's
  rounding) for one of the three percentile columns but uses
  the formatter for the others would produce a row where two
  cells round differently. v0 `C4` checks only `mean`; an
  HALF_EVEN bug in `p50` would slip through.
- Inputs: a record with each µs field set to `1.0625` µs (the
  HALF_UP-vs-HALF_EVEN witness from v0):
  `BenchmarkRecord(HAL_GET_FPGA_TIME, IDLE_BUS, 1,
   new Stats(1.0625, 1.0625, 1.0625, 1.0625, 1.0625, 0.0))`.
  `outlierRate=0.0` is set explicitly so the input record is
  fully specified; the test's concern is the five µs columns,
  and `outlierRate` flows through a different formatter
  (`%.6f`) so it does not interact with the HALF_UP assertion.
- Expected: the row contains `1.063` for **every** µs column
  (exactly five occurrences of `,1.063,` or trailing-`1.063,`
  in the row) — `1.062` appearing in any column would mean
  that column was HALF_EVEN-routed.

> **Other v0 `CsvWriterTest` cases** (`C5`–`C9`) are unchanged.
> `C5` (outlier_rate format) operates on the last column, whose
> position is preserved as the rightmost column. `C6`/`C6a`
> (validated flag), `C7`/`C8` (bus_devices), `C9` (empty record
> list) are all preamble or non-row tests.

### `BenchmarkRecordTest`

#### R1' — `to_csv_row_emits_documented_column_order`
- **Supersedes:** v0 `R1`. Row format widens.
- Layer / contract: `BenchmarkRecord.toCsvRow()` column order.
- Bug class: column shuffle. With the wider row, every pair
  of µs columns is a potential swap; pinning all six values
  distinct ensures any swap is byte-visible.
- Inputs: construct
  `BenchmarkRecord(HAL_GET_FPGA_TIME, IDLE_BUS, 7,
   new Stats(1.100, 2.200, 3.300, 4.400, 5.500, 0.006600))`.
  All six numerical values are mutually distinct so any swap
  between any two µs columns or between any µs column and
  `outlier_rate` is visible byte-by-byte. The five µs values
  are also distinct from any percentile-derived intermediate
  (no value coincidentally equals a max-sample or a sorted-
  index lookup).
- Expected: `toCsvRow()` returns exactly the string
  `hal_get_fpga_time,idle_bus,7,1.100,2.200,3.300,4.400,5.500,0.006600\n`.
- Tolerance: byte-equal.

#### R4 — `stats_record_field_order_is_pinned`
- **New.** Pins the `Stats` record's field order — the `record`
  syntax is positional, so a field reorder that compiles
  successfully would silently corrupt every downstream consumer.
- Layer / contract: `Statistics.Stats` constructor argument
  order.
- Bug class: a refactor that swaps `p50Us` and `p999Us` in the
  field declaration (because alphabetical sort puts `p50` before
  `p999`, but R-7-percentile order puts `p50` < `p99` < `p999`).
  The constructor would compile, every test that constructs
  `Stats` with positional args would silently get values in the
  wrong fields, and the row would emit `p999`'s value in the
  `p50` column. This is the "silently wrong" mode that field-
  named pins catch.
- Inputs: construct
  `new Stats(1.0, 2.0, 3.0, 4.0, 5.0, 0.06)` and read each
  accessor.
- Expected:
  - `meanUs() == 1.0`,
  - `stddevUs() == 2.0`,
  - `p50Us() == 3.0`,
  - `p99Us() == 4.0`,
  - `p999Us() == 5.0`,
  - `outlierRate() == 0.06`.
- Why this is not redundant with `R1'`: `R1'` checks the **CSV
  output column order**; `R4` checks the **Java field order**.
  A defect that swaps two fields in `Stats` and **also** swaps
  the corresponding `String.format` argument order in
  `toCsvRow` would pass `R1'` (because both ends of the swap
  cancel) but fail `R4`.

> **Other v0 `BenchmarkRecordTest` cases** (`R2` comparator,
> `R3` equals/hashCode) are unchanged. The `record`-derived
> `equals`/`hashCode` over the wider `Stats` automatically
> includes the new fields.

### `BenchmarkPipelineTest`

#### P1'' — `pipeline_produces_documented_csv_for_synthetic_run`
- **Supersedes:** cycle-A `P1'` (which itself superseded v0 `P1`).
  Same wiring, two more columns per row, fixture regenerated.
- Layer / contract: end-to-end Sequencer → Statistics →
  BenchmarkRecord → CsvWriter, 6×4=24 cells, 9-column rows.
- Bug class:
  - All cycle-A `P1'` bug classes (wiring, sort order, sequencer
    early termination), scaled to the wider row schema.
  - **New for cycle B:** a `compute` regression that fills
    `p50Us`/`p999Us` from the wrong intermediate. The
    synthetic-run dataset uses `4 identical samples` per cell,
    so all three percentiles equal the mean for each cell. A
    defect that returns the maximum sample for `p999` (instead
    of R-7-interpolated) would still produce the same value
    for this dataset. To distinguish, the synthetic samples per
    cell change to **non-identical**: cell `(c, p)` submits
    `[v - 1, v, v, v + 1]` ns where `v = (6*p.ordinal() + c.ordinal()) * 1000`.
    R-7 percentiles on this 4-sample distribution:
    - p50: `h = 3*0.5 + 1 = 2.5`, fraction 0.5, lo = sorted[1],
      hi = sorted[2]. For `[v-1, v, v, v+1]` sorted, lo = v,
      hi = v, so p50 = `v` (the median).
    - p99: `h = 3*0.99 + 1 = 3.97`, fraction 0.97, lo =
      sorted[2] = v, hi = sorted[3] = v+1, p99 =
      `v + 0.97 = v + 0.97` ns.
    - p999: `h = 3*0.999 + 1 = 3.997`, fraction 0.997, lo = v,
      hi = v+1, p999 = `v + 0.997` ns.
    - mean = `(v-1 + v + v + v+1)/4 = v` ns; stddev =
      `sqrt(((v-1-v)^2 + (v+1-v)^2)/3) = sqrt(2/3) ≈ 0.81650`
      ns. **Hmm, that's not zero, which means the byte-equal
      fixture would carry that stddev value too** — see below.
- Inputs:
  - `Sequencer` configured with all 6 v0 call classes,
    `warmupCount = 1`, `perPointCount = 4`.
  - For each `(c, p)` cell, submit `[v - 1, v, v, v + 1]` ns
    where **`v = ((6 * p.ordinal() + c.ordinal()) + 1) * 1000`**.
    The `+1` offset shifts the cell-index space to start at 1
    instead of 0, so the lowest cell (`HAL_GET_FPGA_TIME,
    IDLE_BUS`, `c=0, p=0`) produces samples `[999, 1000, 1000,
    1001]` instead of `[-1, 0, 0, 1]`. `Statistics.compute` has
    no non-negativity precondition (the v0 / cycle-A fixtures
    used `0` for that cell without trouble), but using
    physically-plausible positive nanosecond values avoids
    distracting a reviewer of the regenerated fixture.
    Submission order: round-robin `(round, callClass)` with
    rounds `0..3`; round 0 submits `v - 1`, round 1 submits
    `v`, round 2 submits `v`, round 3 submits `v + 1`. The
    Sequencer stores them in submission order;
    `Statistics.compute` sorts before quantile-ing.
  - `Statistics.compute(samples, c.blockSize())` per cell.
  - `RunMetadata` fixture identical to v0 `C1` / cycle-A `P1'`.
- Worked derivation, **one representative new row** (the same
  blockSize=1, ordinal-3 cell as cycle-A's `CAN_FRAME_WRITE,
  saturated_multi_thread`):
  - c=5, p=3, blockSize=1.
  - v = ((6*3 + 5) + 1) * 1000 = 24 * 1000 = 24000.
  - Samples (sorted): `[23999, 24000, 24000, 24001]` ns.
  - mean (ns) = 24000; per-call (block=1) = 24000 ns =
    `24.000` µs.
  - sample variance (ns²) = `((23999-24000)² + 0 + 0 +
    (24001-24000)²) / 3 = 2/3`; stddev = `sqrt(2/3) ≈
    0.816497` ns; per-call = `0.000816497` µs → `%.3f` =
    `0.001`.
  - p50 (R-7) at h=2.5: lo=sorted[1]=24000, hi=sorted[2]=24000,
    fraction=0.5; p50 = 24000 ns = `24.000` µs.
  - p99 (R-7) at h=3.97: lo=sorted[2]=24000, hi=sorted[3]=24001,
    fraction=0.97; p99 = 24000.97 ns → `24.001` µs.
  - p999 (R-7) at h=3.997: same lo/hi, fraction=0.997; p999 =
    24000.997 ns → `24.001` µs.
  - outlier_rate: stddev=0.816497 ns, threshold = mean + 5σ =
    24000 + 4.082 = 24004.08; max sample 24001 < threshold; no
    outliers. `outlier_rate = 0.000000`.
  - Expected row:
    `can_frame_write,saturated_multi_thread,4,24.000,0.001,24.000,24.001,24.001,0.000000`.
- **What P1'' does and does not catch.** The non-identical
  `[v-1, v, v, v+1]` samples distinguish max-sample (`v+1`) from
  R-7 p999 (`v+0.997`) at **nanosecond** precision, but at the
  CSV's `%.3f` µs formatting both round to the same value for
  every blockSize=1 cell (`v+0.97 ns ≈ v+0.001 µs ≈ v+0.997 ns`).
  Therefore P1'' does **not** catch a "p999 returns max-sample"
  defect via the byte-equal fixture. That defect class is caught
  at the unit level by `S13` / `S14` / `S15`, whose datasets
  produce R-7 values clearly distinct from any sample's value.
  P1''s role is wiring (cell magnitude is right, columns are in
  order, sort works); column-shuffle defects are caught by `R1'`,
  which uses six mutually-distinct values per row. The trio
  `S13/S14/S15 + R1' + P1''` together cover both math and
  wiring; trying to make P1'' subsume the unit-level percentile
  distinction would require samples spread far enough that
  v+0.97 and v+0.997 round differently, which contradicts the
  cell-index formula.
- **Inherited-cell invariant under cycle B is weaker than under
  cycle A.** Every row in the cycle-B fixture differs from the
  cycle-A fixture (two new columns added; additionally, the
  **twelve** blockSize=1 rows — three call classes
  (`hal_notifier_wait`, `can_frame_read`, `can_frame_write`)
  × four operating points — have their stddev column change
  from `0.000` to `0.001` because per-call stddev =
  sqrt(2/3) ns / 1 / 1000 ≈ 0.000816 µs → `%.3f` = `0.001`).
  The remaining **twelve** rows with blockSize ∈ {10, 100}
  (three call classes × four operating points) keep their
  stddev as `0.000` (the same sqrt(2/3) ns divided by 10 or
  100 rounds to `0.000` µs at `%.3f`). The diff against the
  cycle-A fixture shows modifications, not just additions.
  This is expected: cycle B is a CSV-schema change, not a
  row-additive cycle. Implementer is **not** asked to verify a
  byte-identical-inheritance property; the spot-check row
  above + the byte-equal fixture together pin correctness.
- **Fixture generation procedure.** Identical to cycle-A: run
  `P1''` against the implemented Sequencer + CsvWriter, capture
  output, commit. Spot-check the row above before commit.
- Expected: the full output string from `CsvWriter.write(...)`
  is byte-equal to the regenerated 24-row fixture.
- Tolerance: byte-equal.
- Determinism: every input is a deterministic function of
  `(call_class, operating_point, sample_index)`; no clocks; no
  random values; enum ordinals pinned.

## What this plan does NOT cover

- Histogram sidecar `*-notifier-hist.csv` — deferred per the
  user's call. Tracked as an open follow-up in the skill.
- Custom per-call-class percentile sets — deferred (no demand).
- Real-RIO measurements of the new percentile columns — picked
  up at hardware-validation time.
- CAN device-count sweep — RB-cycle-C.
- Sim-core loader / version-bump decision — explicitly deferred
  per `D-RB-8`.

### Pre-existing suite gap (acknowledged, not closed by cycle B)

- **No test submits an unsorted array to `Statistics.compute`.**
  Every existing test that calls `compute(long[], int)` does so
  with samples already in ascending order
  (`StatisticsTest.S5/S6/S9/S11/S12` use sorted-by-construction
  inputs). Cycle B's `S14/S15/S16/S17` continue this pattern.
  An implementation of `compute` that drops the internal
  `Arrays.sort` call before `percentileNs` would still pass the
  whole suite. The risk is "someone refactors `compute` and
  removes the sort," not a regression in the current code path.
  Cycle B does not close this gap because adding an unsorted-
  input test is outside the cycle's scope (the cycle is about
  *which* percentiles, not about input-shape robustness). A
  future cycle could close it by adding a single
  `compute_sorts_unsorted_input_before_percentile` test.

## Determinism summary

- All numerical inputs are constants.
- No test reads the wall clock, network, or filesystem (the
  fixture is loaded via classpath).
- `run_timestamp_iso8601` continues to be injected via
  `RunMetadata`.
- Run order independence: any subset of tests can run in any
  order without affecting outcomes.

## Citations

- Inherited from v0 plan: Hyndman & Fan 1996 (R-7); Wikipedia
  sample-stddev worked example; Java `Formatter` `%f` HALF_UP.
- New: none required — cycle B uses the same R-7 quantile
  definition as v0/cycle-A, just at additional `p` values.
