# rio-bench — approved test plan (v0, revision 4 + P1 wording fix)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
on revision 4, with one required editorial fix to P1 already
applied (the original "all per-call statistics fall on integer
multiples of 1 µs" claim was wrong — eight of the eighteen
cells are sub-integer µs values like 0.060 µs and 0.200 µs).
The corrected wording pins the actual property the suite relies
on: per-call-ns is an exact integer for every cell, so `%.3f`
formatting is unambiguous (no halfway-rounding cases). The
fixture file `pipeline_expected.csv` is authored against the
corrected derivation.

Revision 4 fixes:

- **Q1** — replaced `currentPoint() == OperatingPoint.WARMUP`
  with `isWarmingUp() == true`. Pinned the accessor convention
  in "Suite-level concerns / `OperatingPoint` enum contract":
  `Sequencer.isWarmingUp()` is the warmup-state predicate;
  `Sequencer.currentPoint()` is undefined during warmup
  (callers must guard).
- **Q2** — added the off-by-one boundary check the reviewer
  flagged as a coverage gap in revision 3 (gap #1). At 59
  round-robin warmup samples (9 full rounds + 5 of the 10th
  round) the sequencer must still report
  `isWarmingUp() == true`; at 60 it transitions. Submission
  changed from "alternating" to explicit round-robin to mirror
  Q3–Q5's discipline.

Revision 3 changelog (kept for context):

- **Q3 / Q4 / Q5** — specified round-robin interleaving for the
  599-sample partial run (99 complete rounds × 6 classes = 594
  samples, then 5 more samples one-per-class in declaration
  order = 599 total).
- **`fastForwardToPoint` helper** — defined explicit interleaving
  semantics: round-robin one sample per call class per round,
  walking warmup (if needed) then each operating point in order
  until `target` is reached.
- **P1** — inlined a worked derivation for two representative
  output rows so the byte-equal claim is reviewer-verifiable;
  reference to the full fixture file path added.
- **P1 / Q10 / OperatingPoint** — pinned the `OperatingPoint`
  enum: `WARMUP` is **not** an enum member (warmup is an internal
  `Sequencer` state); declaration order is `IDLE_BUS,
  SATURATED_BUS, MULTI_THREAD` (ordinals 0/1/2). P1's formula is
  rewritten to depend on this pinned declaration order; Q10's
  Cartesian product asserts against exactly these three values.
- **S8** — added the worked-out arithmetic showing that with the
  outlier included, mean ≈ 250 ns, stddev ≈ 3157 ns,
  mean+5σ ≈ 16037 ns; 100000 ns is unambiguously above and the
  999 normal samples are unambiguously below.
- **C1** — clarified that the preamble and header are separated
  by a single `\n` (no blank line); inlined expected string
  updated to match.
- **New tests added (reviewer suite-level gaps):**
  - New `OperatingPointTest` class with **O1**
    (`operating_point_enum_has_three_members_in_documented_order`,
    pinning the declaration order P1 depends on) and **O2**
    (`csv_label_matches_documented_table`, the `OperatingPoint`
    analogue of L2 — catches label drift before it propagates
    to C3 / Q10 / row content).
  - **C9** `empty_record_list_writes_preamble_and_header_only`
    — `CsvWriter.write(meta, emptyList)` must succeed and emit
    no data rows.

Revision 2's other findings were either already addressed in
revision 2 or are non-blocking (R2 antisymmetry framing,
S12 cleanup, etc.).

Revision 1 → revision 2 changelog (kept for context):

- **S5 / S6** — corrected p99 expected values (revision 1 used
  nearest-rank values while pinning R-7); chose a more
  diagnostic dataset for S5.
- **Q3 / Q4 / Q5 / Q6** — replaced "continue [prior test]" framing
  with a `fastForwardToPoint` helper so each test constructs and
  drives its own `Sequencer` independently.
- **C1** — specified distinct fixture values for every
  `RunMetadata` field so that a field-assignment swap produces a
  visible diff.
- **C3** — specified the exact insertion permutation and expected
  output order.
- **C4** — replaced "banker's round" with HALF\_UP per Java's
  `String.format(Locale.ROOT, "%.3f", x)`; added a fixture value
  that distinguishes HALF\_UP from HALF\_EVEN.
- **C5** — added the `outlier_rate = 0.0` rendering case.
- **C6** — added a companion test for `validated = true`.
- **R2** — added the `compare(a, a) == 0` assertion.
- **R3** — added the inequality assertion.
- **Q10** — strengthened to assert the full Cartesian product of
  (call_class × operating_point), not just count.
- **L1** — added the count assertion alongside membership.
- **New test class `BenchmarkPipelineTest`** — one end-to-end
  integration test exercising Sequencer → Statistics →
  BenchmarkRecord → CsvWriter, to catch wiring bugs no unit test
  would see (reviewer suite-level gap #6).
- **Build verification step** — added a precondition that
  `./gradlew test` must execute the desktop suite without
  requiring HAL JNI load; if `wpi.java.configureTestTasks(test)`
  blocks this, a separate sourceset excluding HAL configuration
  is the fix.

This is the unit-test plan for the pure-Java logic of `tools/rio-bench/`.
The hardware-validation step (deploy → run → diff CSV) is documented
as a manual procedure in `.claude/skills/rio-bench.md` "Operator
procedure"; that step is not in this plan because it cannot be
expressed as a unit test.

## Context for the reviewer

`tools/rio-bench/` is a WPILib 2026 Java/Gradle robot project, NOT
part of the top-level CMake build. It is deployed to a physical
RoboRIO 2, runs an autonomous-init benchmark sweep, and writes a CSV
fixture to `/home/lvuser/rio-bench/<wpilib-version>-<rio-firmware>.csv`.
The CSV is the *only* deliverable the sim-core's tier-1 backend will
consume; if the file shape is wrong, every downstream fidelity
claim drifts.

The skill at `.claude/skills/rio-bench.md` defines:
- the CSV format (preamble, header, row order, decimal formatting),
- the v0 call classes (`hal_get_fpga_time`, `hal_notifier_wait`,
  `ds_state_read`, `power_state_read`, `can_frame_read`,
  `can_frame_write`),
- the operating points (`idle_bus`, `saturated_bus`, `multi_thread`),
- the sequencer schedule (warmup → idle → saturated → multi_thread →
  done),
- the seven D-RB-* design decisions (clock choice, block timing,
  outlier definition, GC policy, row determinism, auto-sweep,
  HAL/logic separation).

The pure-Java packages under test are `bench.stats`, `bench.csv`,
`bench.sequencer`, `bench.calls`. The HAL-bound packages
`bench.runner` and `bench.io` are not in this plan — they have no
desktop unit tests because they call real HAL JNI.

## Suite-level concerns

- **Framework:** JUnit 5 (already wired in `build.gradle`).
- **Determinism:** every test uses fixed inputs. Statistics tests
  with random samples seed `java.util.Random` explicitly.
- **No HAL imports** in any file under `src/test/java/` — that's the
  hard line between "desktop-runnable" and "RIO-only."
- **No timestamp assertions** other than format. `run_timestamp_iso8601`
  is provided to the writer as an injected parameter (a `Clock` or
  fixed `Instant`), never read from `Instant.now()` inside the SUT.
- **`OperatingPoint` enum contract (pinned here so Q10 and P1
  can reference a single source of truth):**
  - The enum has exactly **three** members:
    `IDLE_BUS`, `SATURATED_BUS`, `MULTI_THREAD`. Declaration
    order is `IDLE_BUS, SATURATED_BUS, MULTI_THREAD` (ordinals
    0, 1, 2). The order is pinned because P1's deterministic
    sample formula depends on `ordinal()` and a reorder would
    silently change expected values.
  - **`WARMUP` is NOT an enum member.** Warmup is a distinct
    internal state of `Sequencer`. **Pinned accessor convention
    (so the tests can compile):**
    - `Sequencer.isWarmingUp() : boolean` — `true` while the
      sequencer is in the pre-IDLE_BUS warmup phase, `false`
      thereafter.
    - `Sequencer.currentPoint() : OperatingPoint` — returns the
      current measured operating point. Calling it during
      warmup is undefined behavior (callers must guard with
      `isWarmingUp()`); the tests in this plan only call
      `currentPoint()` after warmup ends.
    The `Sequencer.extractRecords()` output covers exactly the
    3-member enum, never a WARMUP bucket.
  - `csvLabel()` per member: `IDLE_BUS → "idle_bus"`,
    `SATURATED_BUS → "saturated_bus"`, `MULTI_THREAD → "multi_thread"`.
    Verified by L4. Lexicographic comparison on these labels
    gives `idle_bus < multi_thread < saturated_bus`, matching
    the skill's documented sort order.
- **Build precondition (must hold before tests are written):**
  `./gradlew test` must execute the desktop suite without
  requiring HAL JNI to load. The plan's premise (no HAL imports
  under `src/test/java/`) protects against import-time failures,
  but GradleRIO's `wpi.java.configureTestTasks(test)` may still
  configure a HAL-bearing classpath. If the first run of
  `./gradlew test` fails at JVM startup (HAL JNI not found), the
  fix is one of:
    1. set `includeDesktopSupport = true` in `build.gradle` so
       the desktop JNI is on the test classpath, or
    2. remove `wpi.java.configureTestTasks(test)` and configure
       the test task manually with no HAL dependencies.
  This must be verified before the implementer writes the first
  failing test. If neither option works the plan is invalidated
  and a different test-runner approach is needed.

## Per-test specifications

### `StatisticsTest`

System under test: `bench.stats.Statistics`, a pure-function class
that turns a `long[]` of nanosecond timings + a `samplesPerBlock` +
a `blockSize` into a `Stats` record (`meanUs`, `stddevUs`, `p99Us`,
`outlierRate`).

#### S1. `mean_of_constant_series_equals_constant`
- Layer / contract: pure stats — mean computation.
- Bug class: stride or off-by-one in the mean accumulator
  (sums everything but the first sample).
- Inputs: 1000 samples, each = 1000 ns; samplesPerBlock=1, blockSize=1.
- Expected: `meanUs = 1.0` exactly.
- Tolerance: equality on a double computed from integers (no
  rounding).
- Determinism: deterministic input.

#### S2. `stddev_of_constant_series_is_zero`
- Bug class: dividing by N when Bessel-corrected (N-1) is required,
  or vice versa, would still give zero on a constant series — so
  this test catches a more egregious bug: stddev returning anything
  nonzero on identical inputs (e.g. accumulator init bug).
- Inputs: 100 samples, each = 5000 ns.
- Expected: `stddevUs = 0.0`.
- Tolerance: `< 1e-12`.

#### S3. `mean_matches_textbook_for_known_dataset`
- Bug class: arithmetic error in mean accumulator (overflow,
  fence-post).
- Inputs: `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]` ns, blockSize=1.
- Expected: `meanUs = 0.0055`.
- Tolerance: `< 1e-9` µs.
- Citation: textbook arithmetic mean.

#### S4. `stddev_uses_sample_bessel_correction`
- Bug class: population (N) vs sample (N-1) divisor confusion.
  v0 pins **sample** stddev (Bessel-corrected) because the
  population we care about is the bench output as an estimator of
  RIO behavior, not the bench output itself.
- Inputs: `[2, 4, 4, 4, 5, 5, 7, 9]` ns, blockSize=1. Textbook example.
- Expected: `stddevUs ≈ 0.002138` (Bessel-corrected) — explicitly
  rejects the population value `0.002000`.
- Tolerance: `< 1e-9` µs.
- Citation: Wikipedia "Standard deviation" worked example.

#### S5. `p99_uses_linear_interpolation_between_ranked_samples`
- Bug class: nearest-rank vs linear-interpolation choice not pinned.
  We pin **linear interpolation** (R type-7, the default in NumPy
  and most stats packages).
- Inputs: 10 values `[10, 20, 30, 40, 50, 60, 70, 80, 90, 100]` ns,
  blockSize=1. (Chosen so R-7 and nearest-rank produce visibly
  different µs values.)
- Expected: `p99Us = 0.0991` exactly. R-7 derivation:
  `h = (10-1)*0.99 + 1 = 9.91`; `floor(h) = 9`; `fraction = 0.91`;
  `p99 = x[9] + 0.91*(x[10]-x[9]) = 90 + 0.91*10 = 99.1` ns →
  `0.0991` µs.
- This explicitly **rejects** the nearest-rank value `0.1000` µs
  (which would be returned by `ceil(0.99*N) = 10`-th sample = 100 ns).
- Tolerance: `< 1e-12`.
- Citation: Hyndman & Fan 1996 (R-7 / linear interpolation).

#### S6. `p99_of_three_samples_well_defined_under_R7`
- Bug class: small-N edge case yielding NaN or IndexOutOfBounds, OR
  silently falling back to nearest-rank for small N.
- Inputs: `[10, 20, 30]` ns, blockSize=1.
- Expected: `p99Us = 0.0298`. R-7 derivation:
  `h = (3-1)*0.99 + 1 = 2.98`; `floor(h) = 2`; `fraction = 0.98`;
  `p99 = x[2] + 0.98*(x[3]-x[2]) = 20 + 0.98*10 = 29.8` ns →
  `0.0298` µs.
- This explicitly **rejects** the nearest-rank value `0.030` µs
  (the top sample = 30 ns).
- The test also asserts no exception is thrown — small-N branches
  must not throw.
- Tolerance: `< 1e-12`.

#### S7. `outlier_rate_zero_for_pure_constant_series`
- Bug class: outlier branch firing when stddev=0 (would
  divide-by-zero or count every sample as an outlier).
- Inputs: 1000 constant samples.
- Expected: `outlierRate = 0.0`.
- Tolerance: equality.

#### S8. `outlier_rate_counts_samples_above_mean_plus_five_stddev`
- Bug class: wrong threshold (4σ vs 5σ), wrong direction
  (samples *below* mean-5σ counted), or fraction vs count confusion.
- Inputs: 1000 samples drawn from a fixed seed. 999 samples
  uniformly in `[100, 200]` ns + 1 sample at 100000 ns. Seed: 42.
- Expected: outlier_rate = `1/1000 = 0.001`.
- Threshold derivation (with the 100000 ns outlier included in
  mean / stddev — D-RB-3 says outliers are not excluded):
  - sum ≈ 999 × 150 + 100000 ≈ 249,850 ns; mean ≈ 249.85 ns.
  - variance contribution from 999 normal samples ≈
    999 × (var of uniform[100,200]) + 999 × (mean-shift)²
    ≈ 999 × 833 + 999 × (150 - 249.85)² ≈ 832k + 9.96M ≈ 10.79M.
  - variance contribution from outlier ≈ (100000 - 249.85)² ≈
    9.95 × 10⁹.
  - sample variance ≈ (10.79M + 9.95 × 10⁹) / 999 ≈ 9.97 × 10⁶.
  - sample stddev ≈ √9.97 × 10⁶ ≈ 3157 ns.
  - mean + 5σ ≈ 249.85 + 15,787 ≈ **16,037 ns**.
  - The outlier (100000 ns) is unambiguously above 16,037.
  - The 999 normal samples (max ≤ 200 ns) are unambiguously below.
  - Therefore exactly 1 outlier; outlier_rate = 1/1000 = 0.001.
- Tolerance: equality.
- Determinism: seed pinned.

#### S9. `block_timing_divides_block_total_by_block_size`
- Bug class: forgetting to divide by `blockSize`, reporting
  block-totals as per-call times.
- Inputs: 10 samples, each = 10000 ns; blockSize=100.
- Expected: `meanUs = 0.1` (per-call), not `10.0` (per-block).
- Tolerance: equality.

#### S10. `empty_series_throws_illegal_argument`
- Bug class: silently returning zero stats on empty input would
  let the runner write a "0.000, 0.000, 0.000, 0.000" row that
  looks like a real measurement.
- Inputs: `long[0]`, blockSize=1.
- Expected: `IllegalArgumentException` with message containing
  "empty".

#### S11. `single_sample_yields_zero_stddev_and_mean_equals_value`
- Bug class: single-sample case dividing by `(N-1)=0`.
- Inputs: `[7000]` ns, blockSize=1.
- Expected: `meanUs = 7.0`, `stddevUs = 0.0`,
  `p99Us = 7.0`, `outlierRate = 0.0`. No exception.

#### S12. `large_sample_count_does_not_lose_precision`
- Bug class: naïve `sum / n` mean accumulator overflowing / losing
  bits at 1e6 samples; or naïve `Σ(x-μ)²` two-pass introducing
  rounding drift.
- Inputs: 1_000_000 samples, all = 1234 ns.
- Expected: `meanUs = 1.234` exactly; `stddevUs = 0.0`.
- Tolerance: `< 1e-9` µs on mean.
- Note: the test passes for any precision-preserving algorithm
  (Welford's online or numerically-careful two-pass). It is meant
  to catch a naïve overflow or single-pass `Σx² - (Σx)²/n` form,
  which loses bits at this scale.

### `CsvWriterTest`

System under test: `bench.csv.CsvWriter`. Takes a `RunMetadata`
record (wpilib version, rio firmware, image, git SHA, run timestamp,
warmup/sample/block sizes, validated flag, optional bus_devices) and
a `List<BenchmarkRecord>`. Returns a `String` (full file contents).
Filesystem write happens in `bench.io.RioFileSink`, not here.

#### C1. `writes_preamble_lines_with_correct_order_and_values`
- Bug class: (a) row-order regression in the preamble breaking
  index-based parsers; (b) field-assignment swap in the format
  string (e.g. `wpilib_version` and `rio_firmware` swapped) that
  preserves order and line count but mislabels values.
- Inputs: a `RunMetadata` fixture with **distinct values per
  field** so an assignment swap produces a different output:
  - `wpilib_version = "2026.2.1"`
  - `rio_firmware = "9.0.0"`
  - `rio_image = "2026_v3.0"`
  - `git_sha = "abc123def456"`
  - `run_timestamp_iso8601 = "2026-04-29T20:00:00Z"` (injected)
  - `warmup_samples = 1000`
  - `samples_per_block = 10000`
  - `block_size = 100`
  - `validated = false`
  - `bus_devices = Optional.empty()`
- Expected: the file's first 11 lines are exactly (each line
  terminated by a single `\n`, no blank line between preamble
  and header):
  ```
  # rio-bench v0
  # wpilib_version=2026.2.1
  # rio_firmware=9.0.0
  # rio_image=2026_v3.0
  # git_sha=abc123def456
  # run_timestamp_iso8601=2026-04-29T20:00:00Z
  # warmup_samples=1000
  # samples_per_block=10000
  # block_size=100
  # validated=false
  call_class,operating_point,sample_count,mean_us,stddev_us,p99_us,outlier_rate
  ```
  i.e. line 11 is the header row, immediately following the
  preamble's final `# validated=false\n`.
- Tolerance: byte-equal.
- Determinism: `run_timestamp_iso8601` is injected via the
  `RunMetadata` constructor; the writer never reads `Instant.now()`.

#### C2. `writes_header_row_after_preamble`
- Bug class: header drift would silently shift columns.
- Inputs: empty record list, valid metadata.
- Expected: line after preamble is exactly
  `call_class,operating_point,sample_count,mean_us,stddev_us,p99_us,outlier_rate\n`.

#### C3. `sorts_records_by_call_class_then_operating_point`
- Bug class: row order depending on insertion order would make
  cross-run diffs unreadable. Pinned per D-RB-5.
- Inputs: 6 records spanning 3 call classes × 2 operating points,
  inserted in **reverse of expected output order** so a no-op sort
  fails the test:
  - insertion order:
    `(can_frame_write, saturated_bus)`,
    `(can_frame_write, idle_bus)`,
    `(can_frame_read, saturated_bus)`,
    `(can_frame_read, idle_bus)`,
    `(hal_get_fpga_time, saturated_bus)`,
    `(hal_get_fpga_time, idle_bus)`.
- Expected: output rows in this order:
  `(can_frame_read, idle_bus)`,
  `(can_frame_read, saturated_bus)`,
  `(can_frame_write, idle_bus)`,
  `(can_frame_write, saturated_bus)`,
  `(hal_get_fpga_time, idle_bus)`,
  `(hal_get_fpga_time, saturated_bus)`.
  Lexicographic on canonical labels for both keys, primary key
  first. Per-record stats fields are arbitrary fixed values
  (different per record) so a row-swap is visible byte-by-byte.

#### C4. `formats_microsecond_columns_with_locale_root_half_up`
- Bug class: (a) locale-dependent decimal separator (`,` in
  fr-FR) breaking the CSV — pinned per D-RB-5; (b) implementation
  using `BigDecimal.setScale(3, HALF_EVEN)` (banker's rounding)
  instead of Java's `String.format(Locale.ROOT, "%.3f", ...)`,
  which uses HALF_UP — would yield wrong values at exact halfway
  points and break loader-side numeric reproducibility.
- Inputs: two records used in turn:
  1. `meanUs = 1.2345678` (typical case).
  2. `meanUs = 1.0625` (= 17/16, exactly representable as a
     `double`; sits exactly halfway between `1.062` and `1.063`).
- Expected:
  1. row contains `1.235` (HALF_UP on `1.2345...` → up).
  2. row contains `1.063` — this **rejects** HALF_EVEN, which
     would round to `1.062` (last retained digit `2` is even).
- The implementation must use `String.format(Locale.ROOT, "%.3f", x)`
  (or any equivalent that matches Java's `Formatter` HALF_UP
  semantics for `%f`).

#### C5. `formats_outlier_rate_with_six_decimal_places`
- Bug class: (a) `outlier_rate=0.000001` rendered as `0.000`
  losing rare-tail signal; (b) a special-case branch
  `if (rate == 0) return "0"` that bypasses the formatter and
  emits a non-conforming value the loader can't parse.
- Inputs: two records used in turn:
  1. `outlierRate = 1e-6`.
  2. `outlierRate = 0.0`.
- Expected:
  1. row contains exactly `0.000001`.
  2. row contains exactly `0.000000` (not `0`, not `0.0`,
     not `0.000`).
- Tolerance: byte-equal substring match.

#### C6. `unvalidated_flag_renders_as_lowercase_false`
- Bug class: `Boolean.toString()` is fine on JDK but the future
  loader is C++ — pin the exact rendering.
- Inputs: `validated=false`.
- Expected: preamble contains `# validated=false` (lowercase, no
  surrounding whitespace).

#### C6a. `validated_flag_renders_as_lowercase_true`
- Bug class: a formatter that emits `True` (Python-style) or
  `TRUE` for the `true` case would silently break the C++ loader
  on validated CSVs — exactly the case the sim core relies on.
- Inputs: `validated=true`.
- Expected: preamble contains `# validated=true` (lowercase, no
  surrounding whitespace).

#### C7. `bus_devices_line_omitted_when_count_is_full`
- Bug class: always emitting a `bus_devices` line clutters the
  preamble; per the skill it appears only when `< expected`.
- Inputs: bus_devices field absent (Optional.empty).
- Expected: no `# bus_devices=` line in output.

#### C8. `bus_devices_line_present_when_count_is_short`
- Inputs: bus_devices=2 (expected 4).
- Expected: preamble contains `# bus_devices=2` exactly once.

#### C9. `empty_record_list_writes_preamble_and_header_only`
- Bug class: a defensive `if (records.isEmpty()) throw …` or
  `if (records.isEmpty()) return ""` branch that bypasses
  preamble emission would produce a degenerate file the loader
  would treat as malformed instead of "no measurements yet."
  Equally, a writer that emits a phantom data row from an
  uninitialized record buffer would corrupt downstream parsing.
- Inputs: same `RunMetadata` fixture as C1; record list = empty.
- Expected: output is exactly the C1 expected string (preamble +
  header line, both terminated by `\n`), with **no** data rows
  following. Total line count = 11. No exception.

System under test: `bench.csv.BenchmarkRecord`, an immutable POD
holding `(callClass, operatingPoint, sampleCount, stats)`.

#### R1. `to_csv_row_emits_documented_column_order`
- Bug class: column shuffle would silently break the loader.
- Inputs: a record with known values.
- Expected: `to_csv_row()` returns exactly
  `<call_class>,<operating_point>,<sample_count>,<mean_us>,<stddev_us>,<p99_us>,<outlier_rate>\n`
  with the documented formatters.

#### R2. `comparator_orders_by_call_class_then_operating_point`
- Bug class: the `Comparable`/`Comparator` contract drifting from
  the documented sort key would let CsvWriterTest C3 pass with the
  wrong order if the writer used `Collections.sort`.
- Inputs: triples of records.
- Expected:
  - `compare(a, a) == 0` for any record `a` (reflexivity, pins
    the equality contract).
  - For `{same call, different op}`: comparator returns sign of
    `op_a.compareTo(op_b)`.
  - For `{different call, any op}`: comparator returns sign of
    `call_a.compareTo(call_b)`, regardless of operating point
    (i.e. call-class is the primary key).
  - Antisymmetry: `compare(a, b) == -compare(b, a)` for at least
    one pair.

#### R3. `equals_is_value_based`
- Bug class: (a) identity-based equals would let collections
  silently duplicate records; (b) a trivially-`return true` equals
  would pass an equality-only test.
- Inputs:
  - record pair `(a, b)` with all fields identical.
  - record pair `(a, c)` differing only in `callClass`.
  - record pair `(a, d)` differing only in `operatingPoint`.
  - record pair `(a, e)` differing only in stats values.
- Expected:
  - `a.equals(b)` and `a.hashCode() == b.hashCode()`.
  - `!a.equals(c)`, `!a.equals(d)`, `!a.equals(e)`.

### `SequencerTest`

System under test: `bench.sequencer.Sequencer`. State machine:
configure with `(warmupCount, perPointCount, callClasses)`; submit
samples one at a time; query current operating point and
done-ness; on done, return all collected per-(call, point) sample
arrays.

#### Q1. `starts_in_warmup_state`
- Bug class: sequencer starting in IDLE_BUS, skipping warmup
  entirely.
- Inputs: freshly-constructed sequencer.
- Expected:
  - `isWarmingUp() == true`,
  - `isDone() == false`.
  - The plan deliberately does not assert `currentPoint()` here
    — per the `OperatingPoint` enum contract pinned in
    "Suite-level concerns", `currentPoint()` is undefined during
    warmup.

#### Q2. `transitions_warmup_to_idle_bus_after_warmup_count_samples_per_call`
- Bug class: (a) transitioning on total samples instead of
  per-call-class samples would short-circuit warmup for some
  classes (warmup is per-call-class); (b) off-by-one in the
  warmup counter would transition early or late.
- Inputs: warmupCount=10, perPointCount=100, 6 call classes.
  Submit warmup samples round-robin (one per class per round).
- Expected:
  - After 59 round-robin samples (9 complete rounds × 6 classes
    + 5 from the 10th round): `isWarmingUp() == true` (boundary
    check — five classes have hit 10, but the sixth has only 9,
    so the per-class warmup completion rule keeps the sequencer
    in warmup).
  - After 60 round-robin samples: `isWarmingUp() == false` and
    `currentPoint() == IDLE_BUS`.

> **Helper.** Q3–Q6 each construct their own `Sequencer` and
> independently advance to the test's starting state via a static
> helper:
> ```
> static void fastForwardToPoint(
>     Sequencer s,
>     OperatingPoint target,
>     List<CallClass> callClasses,
>     int warmupCount,
>     int perPointCount)
> ```
> **Interleaving:** the helper submits samples **round-robin —
> one sample per call class per round, in the order the
> `callClasses` list provides** — for the warmup phase (if
> `target` is past warmup) and then for each successive operating
> point until `s.currentPoint() == target`. Each warmup round
> contributes one sample to every class; the helper executes
> `warmupCount` rounds. Each measured-point round likewise
> contributes one sample to every class; the helper executes
> `perPointCount` rounds for each operating point earlier in
> declaration order than `target`. Sample value is a fixed constant
> (1000 ns) since the helper exists only to advance state, not to
> seed measurements.
>
> No shared mutable state between tests; no reliance on JUnit
> execution order.

> **Boundary-check distribution for Q3–Q5.** Each boundary check
> submits the per-point sample budget round-robin: 99 complete
> rounds × 6 call classes = 594 samples, then 5 more samples
> one-per-class in `callClasses` declaration order, totalling
> 599 samples. After the 599th sample, five classes have 100
> samples each but the sixth class (last in declaration order)
> has only 99 — so not all classes have reached `perPointCount`,
> and the sequencer must remain in the current operating point.
> The 600th sample is one more for that sixth class; at that
> point all six classes have reached 100 and the sequencer
> transitions. This pins the per-call-class transition rule
> independent of submission strategy.

#### Q3. `transitions_idle_to_saturated_after_per_point_samples`
- Bug class: premature transition (transitions before all call
  classes complete their per-point quota), or late transition.
- Inputs: fresh sequencer; `fastForwardToPoint(IDLE_BUS, ...)`;
  then submit 600 idle samples (100 × 6 call classes).
- Expected: after 600 samples, `currentPoint() == SATURATED_BUS`;
  after 599, still `IDLE_BUS` (boundary check).

#### Q4. `transitions_saturated_to_multi_thread_after_per_point_samples`
- Bug class: same as Q3 at the next boundary.
- Inputs: fresh sequencer; `fastForwardToPoint(SATURATED_BUS, ...)`;
  then submit 600 saturated samples.
- Expected: `currentPoint() == MULTI_THREAD` after 600;
  `currentPoint() == SATURATED_BUS` after 599 (boundary check).

#### Q5. `transitions_multi_thread_to_done_after_per_point_samples`
- Bug class: never-completing sequencer (a wrong off-by-one would
  trap the runner forever and the CSV would never be written).
- Inputs: fresh sequencer; `fastForwardToPoint(MULTI_THREAD, ...)`;
  then submit 600 multi-thread samples.
- Expected: `isDone() == true` after 600;
  `isDone() == false` and `currentPoint() == MULTI_THREAD` after
  599 (boundary check).

#### Q6. `done_state_rejects_further_samples`
- Bug class: silently accepting samples after done would inflate
  the last-active point's sample count and corrupt the multi-
  thread bucket.
- Inputs: fresh sequencer; `fastForwardToPoint` then submit the
  full multi-thread quota so `isDone() == true`; then submit one
  more sample for any call class.
- Expected: the extra-sample submission throws
  `IllegalStateException` and `extractRecords()` for the
  multi-thread point still has the documented per-point sample
  count, not one more.

#### Q7. `warmup_samples_are_discarded`
- Bug class: warmup samples leaking into the IDLE_BUS dataset
  would taint the first measured operating point with cold-start
  jitter.
- Inputs: warmupCount=2, perPointCount=3, 1 call class. Submit
  warmup samples = `[9999, 9999]`, then idle samples =
  `[10, 11, 12]`.
- Expected: the IDLE_BUS sample array for that call class is
  exactly `[10, 11, 12]`. The 9999s are gone.

#### Q8. `samples_routed_to_correct_call_class_bucket`
- Bug class: per-call-class accounting confused with per-operating-
  point accounting; samples landing in the wrong bucket.
- Inputs: 2 call classes (A, B), 1 sample of each per phase.
  warmupCount=1, perPointCount=1.
- Expected: A's IDLE_BUS array contains exactly the A idle sample;
  B's IDLE_BUS array contains exactly the B idle sample. No
  cross-contamination.

#### Q9. `submitting_unknown_call_class_throws`
- Bug class: silent acceptance of typo'd call-class labels would
  let a sample disappear into a bucket nobody reads.
- Inputs: sequencer configured with classes {A, B}; submit a
  sample for class C.
- Expected: `IllegalArgumentException`.

#### Q10. `produces_records_for_every_call_x_point_combination`
- Bug class: (a) a (call, point) pair receiving zero samples
  would render the CSV row as a missing line; the loader would
  have to guess. (b) a count-only check (`size() == 18`) would
  pass against an output where one pair is duplicated and
  another is missing.
- Inputs: full run with 6 classes × 3 points × perPointCount=1.
- Expected:
  - `extractRecords().size() == 18`.
  - The set
    `{ (r.callClass, r.operatingPoint) : r ∈ extractRecords() }`
    equals the full Cartesian product of `CallClass.values()`
    (6 elements) × `OperatingPoint.values()` (3 elements:
    `IDLE_BUS, SATURATED_BUS, MULTI_THREAD` — see "Suite-level
    concerns / `OperatingPoint` enum contract"; `WARMUP` is not
    in the enum and therefore cannot appear). No pair missing,
    no pair duplicated.

### `CallClassTest`

System under test: `bench.calls.CallClass` — enum + label / block-
size table. No HAL imports.

#### L1. `enum_lists_v0_call_classes_in_documented_set`
- Bug class: skill drift — adding/removing a call class without
  updating the skill would invalidate downstream expectations.
- Inputs: `CallClass.values()`.
- Expected:
  - `CallClass.values().length == 6`.
  - the set of values equals exactly the 6 v0 classes documented
    in `.claude/skills/rio-bench.md` "v0 call classes":
    `HAL_GET_FPGA_TIME`, `HAL_NOTIFIER_WAIT`, `DS_STATE_READ`,
    `POWER_STATE_READ`, `CAN_FRAME_READ`, `CAN_FRAME_WRITE`.
- An accidental addition fails count; an accidental rename fails
  membership; a removal fails both.

#### L2. `csv_label_is_lowercase_underscore_per_documented_table`
- Bug class: label drift between the skill's table and the code.
- Inputs: each CallClass.
- Expected: `csvLabel()` matches the skill's "label" column.

#### L3. `block_size_matches_documented_per_class_table`
- Bug class: silently forgetting D-RB-2 (sub-µs calls timed
  per-call would report dominated-by-clock-overhead numbers).
- Inputs: each CallClass.
- Expected:
  `HAL_GET_FPGA_TIME → 100`,
  `DS_STATE_READ → 10`,
  `POWER_STATE_READ → 10`,
  others → 1.

### `OperatingPointTest`

System under test: `bench.sequencer.OperatingPoint` — the enum
that names the three measured operating points and provides the
`csvLabel()` used in the CSV `operating_point` column.

#### O1. `operating_point_enum_has_three_members_in_documented_order`
- Layer / contract: `bench.sequencer.OperatingPoint` — enum
  declaration order pinned for P1.
- Bug class: declaration-order drift would silently change
  `ordinal()` values and break P1's deterministic sample
  formula.
- Inputs: `OperatingPoint.values()`.
- Expected:
  - `length == 3`.
  - `values()[0] == IDLE_BUS`,
    `values()[1] == SATURATED_BUS`,
    `values()[2] == MULTI_THREAD`.
  - There is no `WARMUP` member (asserted by enum size and
    membership).

#### O2. `csv_label_matches_documented_table`
- Layer / contract: `bench.sequencer.OperatingPoint` —
  `csvLabel()` strings.
- Bug class: label drift between the skill's table /
  "Suite-level concerns / `OperatingPoint` enum contract" and
  the enum's `csvLabel()`. A drift here propagates silently
  into C3 (sort order keyed on labels), Q10 (Cartesian product
  on labels), and CsvWriter row content; catching it at the
  enum source localizes the failure.
- Inputs: each `OperatingPoint` value.
- Expected:
  - `IDLE_BUS.csvLabel() == "idle_bus"`.
  - `SATURATED_BUS.csvLabel() == "saturated_bus"`.
  - `MULTI_THREAD.csvLabel() == "multi_thread"`.

### `BenchmarkPipelineTest`

System under test: the wiring of `Sequencer` → `Statistics` →
`BenchmarkRecord` → `CsvWriter`. This is the only test that
exercises all four pure-Java packages end-to-end. It exists to
catch wiring bugs (e.g., `Statistics.compute()` called with the
wrong block-size argument, or `BenchmarkRecord` constructed with
swapped fields) that no isolated unit test would surface.

#### P1. `pipeline_produces_documented_csv_for_synthetic_run`
- Layer / contract: integration of all four packages on the
  pure-Java pipeline.
- Bug class: a wiring defect where each unit passes its own
  unit tests but the composed output is wrong — e.g., per-call
  sample buckets fed to `Statistics.compute()` with `blockSize=1`
  when the call's documented `blockSize` is 100; or
  `BenchmarkRecord` constructed with `(operatingPoint, callClass)`
  swapped.
- Pinned `CallClass` declaration order (so `ordinal()` is
  reproducible):
  `HAL_GET_FPGA_TIME (0), HAL_NOTIFIER_WAIT (1), DS_STATE_READ (2),
  POWER_STATE_READ (3), CAN_FRAME_READ (4), CAN_FRAME_WRITE (5)`.
  Pinned `OperatingPoint` declaration order:
  `IDLE_BUS (0), SATURATED_BUS (1), MULTI_THREAD (2)` — per the
  "Suite-level concerns / `OperatingPoint` enum contract"
  section.
- Inputs:
  - `Sequencer` configured with all 6 v0 call classes,
    `warmupCount = 1`, `perPointCount = 4`. Submission via the
    same round-robin discipline `fastForwardToPoint` uses (one
    sample per class per round).
  - For each (call class `c`, operating point `p`), submit
    **4 identical samples** of value
    `s_ns(c, p) = (6 * p.ordinal() + c.ordinal()) * 1000`
    nanoseconds. The sample value is constant within each cell;
    the coefficient `6 * p.ordinal() + c.ordinal()` is the
    cell's index in the 18-cell flattened space. The
    per-call-ns value `(6p + c) * 1000 / blockSize[c]` is an
    exact integer for every (c, p) cell (since `1000 /
    blockSize[c] ∈ {10, 100, 1000}` for the v0 blockSizes), so
    the µs value `(per_call_ns / 1000)` has at most three
    significant decimal places and `%.3f` formatting is
    unambiguous (no halfway-rounding cases at the
    truncation boundary). Cells render at values like
    `0.060` µs, `0.200` µs, `1.500` µs, etc. — not all
    integer-µs.
  - `Statistics.compute(samples, blockSize)` is invoked per cell
    using each call class's documented `blockSize`
    (`HAL_GET_FPGA_TIME=100`, `DS_STATE_READ=POWER_STATE_READ=10`,
    others=1).
  - `RunMetadata` fixture identical to C1's (so the preamble
    matches C1's expected string byte-for-byte).
- Worked derivation, **two representative rows**:

  **Row (HAL_GET_FPGA_TIME, IDLE_BUS):** c=0, p=0, blockSize=100.
  Samples (ns): `[0, 0, 0, 0]`.
  - mean (ns) = 0; per-call mean = 0/100 = 0 ns = `0.000` µs.
  - sample stddev = 0 (identical samples); per-call = `0.000` µs.
  - p99 (R-7) = 0 ns = `0.000` µs.
  - outlier_rate = 0 (stddev=0 case → S7 contract → 0).
  - Expected row:
    `hal_get_fpga_time,idle_bus,4,0.000,0.000,0.000,0.000000`.

  **Row (CAN_FRAME_WRITE, MULTI_THREAD):** c=5, p=2, blockSize=1.
  Cell index = 6×2 + 5 = 17. Samples (ns):
  `[17000, 17000, 17000, 17000]`.
  - mean (ns) = 17000; per-call mean (block=1) = 17000 ns =
    `17.000` µs (exact integer / 1000 → exact double).
  - sample stddev = 0; per-call = `0.000` µs.
  - p99 (R-7 on identical samples) = 17000 ns = `17.000` µs.
  - outlier_rate = 0.
  - Expected row:
    `can_frame_write,multi_thread,4,17.000,0.000,17.000,0.000000`.

  These two rows are spot-checks; the full 18-row expected
  string (preamble + header + 18 sorted rows, byte-for-byte) is
  committed alongside the test at
  `src/test/resources/pipeline_expected.csv` and loaded via
  classpath in the test. The remaining 16 rows are derived
  identically: pick the cell's `(c, p)`, compute index `6p + c`,
  sample value `1000 * index` ns, divide by `blockSize` for
  per-call mean, stddev=0, p99=mean, outlier_rate=0. All values
  fall on exact-integer µs because `1000 / blockSize ∈ {10, 100,
  1000}` is integer for every v0 call class.

  Constant-per-cell samples deliberately avoid testing variance
  arithmetic (that's S2/S4/S12's job); P1 exists to catch
  wiring bugs, which manifest as wrong row labels or wrong
  magnitudes — both visible against the integer-µs fixture.

- Expected: the full output `String` from `CsvWriter.write(...)`
  is byte-equal to the fixture file's contents.
- Tolerance: byte-equal.
- Determinism: every input is a deterministic function of
  `(call_class, operating_point, sample_index)`; no clocks, no
  random values; enum ordinals pinned by declaration order.
- What this catches that the units don't:
  1. `Statistics.compute()` invoked with `samplesPerBlock`
     instead of `blockSize` (or vice versa) — would flip the
     `HAL_GET_FPGA_TIME` and `CAN_FRAME_WRITE` rows' magnitudes.
  2. Sequencer-bucket → BenchmarkRecord field-swap (call_class
     vs operating_point swapped on construction) — would swap
     row labels.
  3. `CsvWriter` emitting records in `extractRecords()` insertion
     order rather than sorted order, when the upstream sequencer
     happens to insert in sorted order — caught because the
     fixture is in sorted order, not insertion order.
  4. A regression where any one unit's constructor argument
     order silently changes — caught by the byte-equal check on
     stats values that depend on correct argument routing.

## What this plan does NOT cover

- The actual HAL call timings (run on hardware; RIO-only).
- The auto-sweep entry from `Robot.autonomousInit` (one method
  that wires `BenchmarkRunner` + `RioFileSink`; thin enough to
  exercise via the manual hardware procedure documented in
  `.claude/skills/rio-bench.md`).
- Worker-thread spawning for `multi_thread`. RIO-only — needs
  real `Thread`/`Executor` jitter under PREEMPT_RT to be
  meaningful.
- File output via `RioFileSink`. Path-construction is a
  `String.format` template that a unit test would only re-encode;
  the CSV body it writes is fully tested via `CsvWriterTest` and
  `BenchmarkPipelineTest`.

## Determinism summary

- All numerical tests use either constants or `Random(42)`.
- No test reads the wall clock, network, or filesystem.
- The CSV writer's `run_timestamp_iso8601` is injected via
  `RunMetadata`, never read from `Instant.now()` inside the SUT.
- Each test constructs its own SUT — Q3–Q6 use the
  `fastForwardToPoint` helper, not "continue [prior test]" state.
- Run order independence: any subset of tests can run in any
  order without affecting outcomes.

## Citations

- Hyndman, R.J. & Fan, Y. (1996). "Sample Quantiles in Statistical
  Packages." *The American Statistician* 50:361–365 — for R-7
  linear-interpolation choice in S5 / S6 / S11.
- Wikipedia, "Standard deviation," sample-stddev worked example —
  S4.
- Java `java.util.Formatter` documentation — `%f` uses HALF_UP
  rounding; pinned in C4.
- `.claude/skills/rio-bench.md` — for all label / order / format
  pinning across S, C, R, Q, L, P tests.
