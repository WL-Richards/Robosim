# rio-bench RB-cycle-A test plan

**Scope:** add a `SATURATED_MULTI_THREAD` operating point to the
sweep, and switch the existing `SATURATED_BUS` from operator-hardware
saturation to software-driven CAN saturation. This is the
"cross-product operating point" item in the post-v0 fidelity
follow-up: real robot code in a match sees CAN load and worker-thread
contention concurrently, but v0 only measured them in isolation.

**Status:** **draft, awaiting `test-reviewer` verdict**.

**Delta convention.** This plan only specifies tests that are **new**
or **modified** vs the v0 plan in `TEST_PLAN.md`. Tests not mentioned
here are unchanged. Each entry that supersedes a v0 test names the
v0 test ID (e.g. "supersedes v0 Q5") so the diff is reviewable.

## Context for the reviewer

`tools/rio-bench/` is a WPILib 2026 Java/Gradle robot project
deployed to a physical RoboRIO 2. v0 ships pure-Java logic
(41/41 green, `validated=false` on the CSV) and HAL-bound code that
compiles but has not run on hardware yet. The skill at
`.claude/skills/rio-bench.md` and `docs/STATUS_RIO_BENCH.md` describe
the current state.

This cycle is the first of a three-cycle post-v0 fidelity push
(A: cross-product point + software-driven saturation; B: notifier
jitter percentiles + histogram; C: CAN device-count sweep with
runtime device scan). Cycles B and C have separate test plans.

### What changes implementation-wise

1. **`OperatingPoint` enum gains `SATURATED_MULTI_THREAD`** as the
   fourth and last member (declaration order
   `IDLE_BUS, SATURATED_BUS, MULTI_THREAD, SATURATED_MULTI_THREAD`,
   ordinals 0/1/2/**3**). Existing ordinals 0/1/2 are preserved so
   v0's `BenchmarkPipelineTest.P1` cell-index formula
   `6 * p.ordinal() + c.ordinal()` still produces the v0 expected
   values for the first 18 cells.

2. **New pure-Java `WorkerSpec` table** at
   `bench.runner.WorkerSpec` (record + static factory) maps each
   `OperatingPoint` to `(int cpuWorkers, boolean canSpam)`. This is
   the contract the HAL-bound runner consumes; extracting it into a
   pure-Java record lets us pin the per-phase setup without touching
   HAL JNI in tests.

   | OperatingPoint            | cpuWorkers | canSpam |
   |---------------------------|------------|---------|
   | `IDLE_BUS`                | 0          | false   |
   | `SATURATED_BUS`           | 0          | **true**  (software-driven, was hardware-driven in v0) |
   | `MULTI_THREAD`            | 3          | false   |
   | `SATURATED_MULTI_THREAD`  | 3          | true    |

   **Implementation specifics.** `WorkerSpec` is a Java `record` so
   equality and hashCode are derived correctly from the two fields
   (the test suite relies on this in `W5` to assert spec-distinctness
   via `Set` membership). The factory `forPoint(OperatingPoint)` is
   an exhaustive switch *expression* with **no `default` branch**.
   The "no default branch" rule is enforced by an inline source-code
   comment in `WorkerSpec.java`, not just this plan, so the compiler-
   level exhaustiveness guarantee survives reviewer turnover — adding
   a fifth `OperatingPoint` becomes a compile error rather than a
   silent fall-through to a wrong default.

3. **`BenchmarkRunner.startWorkersFor`** is rewritten wholesale in
   terms of `WorkerSpec.forPoint(phase)`. Specifically: the v0
   `if (phase != OperatingPoint.MULTI_THREAD) return null;` guard
   at `BenchmarkRunner.java:135` is removed — it encodes the
   operating-point → worker mapping in two places. After cycle A,
   the only place that mapping lives is `WorkerSpec.forPoint`. Any
   remaining per-point `if`/`switch` ladder in `startWorkersFor`
   is a latent D-RB-7 violation and must be removed in the same
   commit. The CAN-spam worker calls
   `CANJNI.FRCNetCommCANSessionMuxSendMessage` to a non-bench ID
   (`0x1FFFFFFD`, distinct from the receive/send bench IDs) in a
   tight loop. HAL-bound; not unit-tested per `D-RB-7`.

4. **CSV row-sort order extends** to four operating points.
   Lexicographic comparison on `csvLabel`s gives
   `idle_bus < multi_thread < saturated_bus < saturated_multi_thread`,
   so each call class now produces four rows in that order
   (was three).

5. **`bus_devices` preamble line semantics shift.** v0 used the line
   to flag "fewer than expected physical devices, results
   `validated=false`." Under cycle A, saturation is software-driven
   so `bus_devices` no longer gates validity — it becomes purely
   informational. The runner change is HAL-bound; the writer's
   contract (Optional present → emit line; Optional empty → omit)
   does not change, so v0 tests `C7` and `C8` survive unchanged.
   Cycle C will reintroduce a meaningful `bus_devices` semantics for
   the device-count sweep.

6. **New design-decision entry `D-RB-8`** in the skill: bus
   saturation is software-driven, not operator-hardware-driven.
   Rationale + the trade-off vs v0's hardware-driven model is
   recorded there so future readers do not re-litigate. The
   skill entry must explicitly note that the sim-core loader
   (whichever cycle introduces it) inherits the responsibility of
   deciding whether to bump the CSV preamble version (currently
   `v0`) so loaders can tell hardware-saturated v0 files apart
   from software-saturated cycle-A-and-later files. Cycle A
   defers that decision because no loader exists yet; the next
   cycle that touches the loader (cycle B is the most likely)
   must answer it before merge.

7. **CSV preamble version unchanged at `# rio-bench v0`.** No column
   added, no column removed; the row at `operating_point=saturated_bus`
   measures a different load profile than a v0 file would, but no
   sim-core loader exists yet, so backward-compat is moot. Cycle B's
   percentile columns are the first version-bump candidate.

### What does NOT change

- Statistics (`bench.stats.Statistics`), CSV writer
  (`bench.csv.CsvWriter`), `BenchmarkRecord`, `CallClass`,
  `Sequencer` core mechanics, file output (`bench.io.RioFileSink`).
- Block sizes per call class.
- `outlier_rate` definition (D-RB-3).
- Decimal formatting / locale handling (D-RB-5).
- v0's `WARMUP` is still not an `OperatingPoint` member.

## Suite-level concerns

- **Framework, determinism, no-HAL-imports, no-wall-clock** rules
  inherited verbatim from v0 plan "Suite-level concerns."
- **`OperatingPoint` enum contract (cycle-A revision)** —
  supersedes the v0 contract pinned in the v0 plan:
  - Members: `IDLE_BUS`, `SATURATED_BUS`, `MULTI_THREAD`,
    `SATURATED_MULTI_THREAD`. Declaration order
    `IDLE_BUS, SATURATED_BUS, MULTI_THREAD, SATURATED_MULTI_THREAD`
    (ordinals 0, 1, 2, 3). The first three ordinals are preserved
    from v0 so the v0 `P1` cell-index formula is invariant for the
    cells it already covered.
  - `csvLabel()`: `IDLE_BUS → "idle_bus"`,
    `SATURATED_BUS → "saturated_bus"`,
    `MULTI_THREAD → "multi_thread"`,
    `SATURATED_MULTI_THREAD → "saturated_multi_thread"`.
    Lexicographic comparison on these labels gives
    `idle_bus < multi_thread < saturated_bus < saturated_multi_thread`,
    pinning the new four-row sort key for `CsvWriter`.
  - `Sequencer.isWarmingUp() / currentPoint()` accessor
    convention unchanged from v0.
- **`WorkerSpec` enum-completeness invariant** — the `WorkerSpec`
  factory in `bench.runner.WorkerSpec` uses an exhaustive
  switch-expression over `OperatingPoint`. The Java compiler enforces
  exhaustiveness at compile time as long as there is no `default`
  branch and no fall-through. The plan declares "no `default` branch
  in `WorkerSpec.forPoint`" as a binding implementation rule so a
  future fifth `OperatingPoint` cannot silently default to the
  "(0, false)" worker configuration.
- **Build precondition** (inherited from v0): `./gradlew test` must
  execute the desktop suite without loading HAL JNI. Cycle-A test
  files do not import any HAL package; this remains structurally
  protected.
- **Pipeline fixture regeneration.** The byte-equal fixture at
  `src/test/resources/pipeline_expected.csv` is regenerated for the
  6×4=24-cell space. The plan provides spot-check derivations for
  three new cells (one per blockSize bucket) below in `P1'`; the
  full regenerated fixture is committed alongside the test code.
- **Test-file naming.** New tests live alongside their v0 siblings:
  - `OperatingPointTest` gains `O3`.
  - New file: `src/test/java/frc/robot/bench/runner/WorkerSpecTest.java`.
  - `SequencerTest` gains `Q5b`; `Q5`, `Q6`, and `Q10` are
    rewritten in place. `Q6`'s rewrite is structural (terminal
    phase shifts from `MULTI_THREAD` to `SATURATED_MULTI_THREAD`),
    not just a relabel — without it, v0 `Q6` deadlocks the suite
    under cycle A because driving the sequencer to `MULTI_THREAD`
    + 600 samples no longer reaches `done` when there are four
    phases.
  - `BenchmarkPipelineTest.P1` is rewritten in place; the fixture
    file is overwritten.

## Per-test specifications

### `OperatingPointTest`

#### O1' — `operating_point_enum_has_four_members_in_documented_order`
- **Supersedes:** v0 `O1`
  (`operating_point_enum_has_three_members_in_documented_order`).
- Layer / contract: `bench.sequencer.OperatingPoint` — enum
  declaration order pinned for `P1'`.
- Bug class: declaration-order drift. Specifically:
  - inserting `SATURATED_MULTI_THREAD` anywhere other than last
    would shift existing ordinals 0/1/2 and silently change every
    expected value in `P1'`'s 18 v0-inherited cells;
  - inserting it correctly at index 3 but spelling
    `csvLabel()` differently would still fail at `O2'`, but
    declaration-order drift is the bug class this test catches.
- Inputs: `OperatingPoint.values()`.
- Expected:
  - `length == 4`.
  - `values()[0] == IDLE_BUS`, `values()[1] == SATURATED_BUS`,
    `values()[2] == MULTI_THREAD`,
    `values()[3] == SATURATED_MULTI_THREAD`.
  - There is no `WARMUP` member.

#### O2' — `csv_label_matches_documented_table`
- **Supersedes:** v0 `O2`. Identical structure; one new assertion
  for the new member.
- Layer / contract: `bench.sequencer.OperatingPoint.csvLabel()`.
- Bug class: csv-label drift between the skill's table and the enum
  body — would propagate silently into row sort order, the Cartesian
  product check in `Q10'`, and the byte-equal fixture in `P1'`.
- Inputs: each `OperatingPoint` value.
- Expected:
  - `IDLE_BUS.csvLabel() == "idle_bus"`,
  - `SATURATED_BUS.csvLabel() == "saturated_bus"`,
  - `MULTI_THREAD.csvLabel() == "multi_thread"`,
  - `SATURATED_MULTI_THREAD.csvLabel() == "saturated_multi_thread"`.

#### O3 — `csv_labels_lex_sort_in_documented_order`
- **New.**
- Layer / contract: row-sort key for `CsvWriter`, downstream of
  `OperatingPoint.csvLabel()`.
- Bug class: a future contributor renames a label (e.g.
  `multi_thread → mt_workers`) preserving uniqueness but changing
  lex order. The CSV row order would silently change without `O1'`
  or `O2'` failing — but `O3` would.
- Inputs: the four `csvLabel()` strings.
- Expected: when sorted lexicographically (Java's
  `String.compareTo`), they produce exactly
  `["idle_bus", "multi_thread", "saturated_bus",
  "saturated_multi_thread"]`.
- Why this is not redundant with `O2'` or `P1'`: `O2'` pins
  *value-by-value* labels; `O3` pins their *relative ordering*
  under the comparator `CsvWriter` actually uses. The argument
  that distinguishes `O3` from the byte-equal `P1'` fixture is
  the **fixture-regeneration property**: the
  `pipeline_expected.csv` fixture is regenerated alongside the
  implementation (per `P1'` "Fixture generation" below), so a
  buggy comparator produces a buggy fixture — both agree on the
  wrong order and `P1'` passes. `O3`'s expected ordering is
  hand-pinned in this plan document and is not regenerated, so
  it provides a sort-order check that survives whatever the
  implementation decides to do. `O3` is therefore the only test
  in cycle A that catches a comparator regression independent of
  any generated artifact.

### `WorkerSpecTest` (new test class)

System under test: `bench.runner.WorkerSpec` — a pure-Java record
`(int cpuWorkers, boolean canSpam)` with a static factory
`WorkerSpec forPoint(OperatingPoint p)` whose body is an exhaustive
switch expression over `OperatingPoint` (no `default` branch).

The record itself has no validation — `cpuWorkers` is a plain `int`
field; values are pinned per-OperatingPoint by the factory. There
are no constructor preconditions to test, only the factory mapping.

#### W1 — `forPoint_idle_bus_returns_no_workers`
- Layer / contract: `WorkerSpec.forPoint(IDLE_BUS)`.
- Bug class: returning a non-zero `cpuWorkers` or a `canSpam=true`
  for `IDLE_BUS` would corrupt the bench's quietest baseline.
  Specifically catches the `IDLE_BUS` arm of the switch being
  filled in with the same value as a copy-paste sibling.
- Inputs: `OperatingPoint.IDLE_BUS`.
- Expected: `WorkerSpec(0, false)`.

#### W2 — `forPoint_saturated_bus_returns_can_spam_only`
- Layer / contract: the cycle-A definition of `SATURATED_BUS`
  (software-driven, no CPU contention).
- Bug class: regressing to v0's `(0, false)` for `SATURATED_BUS`
  would make `SATURATED_BUS` measurements identical to `IDLE_BUS`
  measurements — i.e. silently undo the cycle-A semantic change.
  This is the test that catches "I bumped the enum but forgot to
  spawn the CAN-spam worker."
- Inputs: `OperatingPoint.SATURATED_BUS`.
- Expected: `WorkerSpec(0, true)`.

#### W3 — `forPoint_multi_thread_returns_three_cpu_workers_no_can_spam`
- Layer / contract: `WorkerSpec.forPoint(MULTI_THREAD)`; preserves
  v0's "3 cpu-contention worker threads" setup.
- Bug class: changing the cpu-worker count from 3 silently changes
  the load profile of every `MULTI_THREAD` measurement in the CSV.
  Also catches accidentally enabling `canSpam` here, which would
  conflate the `MULTI_THREAD` row with the `SATURATED_MULTI_THREAD`
  row.
- Inputs: `OperatingPoint.MULTI_THREAD`.
- Expected: `WorkerSpec(3, false)`.

#### W4 — `forPoint_saturated_multi_thread_returns_three_cpu_workers_and_can_spam`
- Layer / contract: the new cross-product point — both the cpu
  worker count and the CAN-spam flag must be on.
- Bug class: a partial-cell defect — e.g. spawning the cpu workers
  but forgetting `canSpam=true`, or vice versa. Either failure mode
  collapses `SATURATED_MULTI_THREAD` onto an existing row's load
  profile and erases the cycle-A fidelity gain. This is the most
  load-bearing test in `WorkerSpecTest`: it pins the *cross-product*
  property the cycle exists for.
- Inputs: `OperatingPoint.SATURATED_MULTI_THREAD`.
- Expected: `WorkerSpec(3, true)`.

#### W5 — `forPoint_returns_distinct_spec_for_every_operating_point`
- Layer / contract: `WorkerSpec.forPoint` produces four distinct
  `(cpuWorkers, canSpam)` tuples across the four `OperatingPoint`s
  — the cross-product property the cycle exists for.
- Bug class: a regression that collapses two operating points onto
  the same spec. Concretely:
  - regressing `SATURATED_BUS` to v0's `(0, false)` would make it
    equal `IDLE_BUS` — the most likely accidental defect, because
    "the v0 code worked";
  - making `SATURATED_MULTI_THREAD` equal `MULTI_THREAD` by
    forgetting `canSpam=true` would erase the cross-product cell
    the cycle exists to add.
  These bugs would also fail an individual `W1`–`W4`, but `W5`
  encodes the **cross-product intent** as an explicit suite-level
  invariant: a future contributor reading `WorkerSpecTest` sees
  "four distinct load profiles" as a documented property, not as
  the implication of four separately-asserted cells.
- Inputs: `OperatingPoint.values()`.
- Expected: collect `WorkerSpec.forPoint(p)` for each `p`; insert
  the four results into a `Set<WorkerSpec>`; assert the set has
  size 4. Because `WorkerSpec` is a Java `record`, equality is
  derived from the two fields and `Set` membership behaves
  correctly without a hand-written `equals`.
- Why this is not redundant with `W1`–`W4`: `W1`–`W4` answer
  "what is the spec for X?"; `W5` answers "are they all
  different?". Failure modes overlap, but the diagnostic intent
  is different. A reader scanning `WorkerSpecTest` for "what
  does cycle A change?" finds it in `W5`'s assertion much faster
  than by diffing four per-cell values against v0.

> **Note on coverage choice.** `WorkerSpec` is a flat 4-cell
> table. One test per cell (`W1`–`W4`) plus the cross-product
> distinctness assertion (`W5`) is exhaustive. There is no
> separate "every operating point is mapped" runtime test
> because the exhaustive switch expression enforces this at
> compile time (the "no `default` branch" rule is mirrored by an
> inline source-code comment in `WorkerSpec.java` so it survives
> reviewer turnover). Null-handling is not tested either —
> Java's switch-on-null-enum NPE is JVM behavior (JLS 14.11.1),
> not SUT logic, and asserting it would re-encode the language
> spec rather than test anything `WorkerSpec` actually controls.

### `SequencerTest`

#### Q5' — `transitions_multi_thread_to_saturated_multi_thread_after_per_point_samples`
- **Supersedes:** v0 `Q5`
  (`transitions_multi_thread_to_done_after_per_point_samples`).
  v0 `Q5` asserted that `MULTI_THREAD` was the terminal phase. Under
  cycle A it is not — there is one more phase after it.
- Layer / contract: `Sequencer.advancePhase()` correctly walks past
  `MULTI_THREAD` into `SATURATED_MULTI_THREAD` without prematurely
  marking the run done.
- Bug class: an off-by-one against `PHASES.length` (now 4, was 3)
  that still treats `MULTI_THREAD` as terminal — exactly the
  regression a copy-paste of the v0 sequencer logic would produce
  if `PHASES.length` is hard-coded somewhere instead of derived
  from `OperatingPoint.values()`.
- Inputs: fresh sequencer with `warmupCount=10`, `perPointCount=100`,
  all 6 call classes; `fastForwardToPoint(MULTI_THREAD, ...)`
  (helper interleaving rules unchanged from v0); then submit
  the per-point quota for `MULTI_THREAD`.
- Expected:
  - After 99 round-robin rounds × 6 classes + 5 partial samples
    (599 total): `currentPoint() == MULTI_THREAD`,
    `isDone() == false` (boundary check).
  - After the 600th sample for the sixth class:
    `currentPoint() == SATURATED_MULTI_THREAD`,
    `isDone() == false`. The sequencer must NOT be done at this
    boundary.

#### Q5b — `transitions_saturated_multi_thread_to_done_after_per_point_samples`
- **New.**
- Layer / contract: `Sequencer.advancePhase()` marks done after the
  fourth (and final) operating point's quota fills.
- Bug class: a sequencer that gets stuck in
  `SATURATED_MULTI_THREAD` because of a hard-coded `PHASES.length`
  comparison would never terminate the sweep, and the runner
  would never write the CSV. This is the v0 `Q5` "never-completing
  sequencer" hazard, now applied at the new terminal boundary.
- Inputs: fresh sequencer with `warmupCount=10`, `perPointCount=100`,
  all 6 call classes; `fastForwardToPoint(SATURATED_MULTI_THREAD, ...)`
  (helper now walks through one more phase than in v0); then submit
  600 saturated_multi_thread samples.
- Expected:
  - After 599 round-robin samples: `isDone() == false`,
    `currentPoint() == SATURATED_MULTI_THREAD` (boundary check).
  - After 600 round-robin samples: `isDone() == true`.

> **Helper note.** `fastForwardToPoint` in `SequencerTest` iterates
> `OperatingPoint.values()`; it has no hard-coded phase count, so
> it walks correctly through any number of phases. `Q5b`'s
> correctness therefore depends on `O1'` holding (four-member
> enum); if `O1'` fails, the test won't even compile because
> `SATURATED_MULTI_THREAD` is not a valid identifier. There is no
> separate "helper-walks-four-phases" test (an earlier draft had
> `Q5b-prereq` for this; review verdict was that any failure
> mode it would catch is already caught by `O1'` more
> diagnostically, so the test was test-of-tests overhead).
>
> **Implementer must add a one-line comment** near the
> `fastForwardToPoint(s, OperatingPoint.SATURATED_MULTI_THREAD, ...)`
> call in the Java source — e.g.
> `// O1' is the compile-time guard: SATURATED_MULTI_THREAD must exist as an enum member.`
> — so a future contributor doesn't re-add `Q5b-prereq` thinking
> the helper's four-phase walk is unverified.

#### Q6' — `done_state_rejects_further_samples`
- **Supersedes:** v0 `Q6`. The structural change (rather than
  label-only) is required because v0 `Q6` drives the sequencer to
  done by submitting 600 `MULTI_THREAD` samples after
  `fastForwardToPoint(MULTI_THREAD, ...)`. Under cycle A,
  `MULTI_THREAD` is no longer terminal; that submission pattern
  transitions to `SATURATED_MULTI_THREAD` instead of marking the
  run done. v0 `Q6` would deadlock at `assertTrue(s.isDone())`
  and the trailing `extractRecords()` would throw
  `IllegalStateException` because the run is not actually done.
- Layer / contract: `Sequencer.submitSample` rejects further
  samples after `isDone() == true`, AND the terminal
  operating-point bucket is not corrupted by the rejected sample.
- Bug class:
  - silently accepting samples after done would inflate the
    terminal point's sample count;
  - throwing `IllegalStateException` but mutating the bucket
    before the throw would also inflate the count;
  - regressing `done` to fire prematurely would let the rejection
    happen at the wrong boundary.
- Inputs: fresh sequencer with `warmupCount=10`, `perPointCount=100`,
  all 6 call classes;
  `fastForwardToPoint(SATURATED_MULTI_THREAD, ...)`; then submit
  the full SATURATED_MULTI_THREAD quota (100 round-robin rounds ×
  6 classes = 600 samples) so `isDone() == true`. Then submit one
  more sample **specifically for `HAL_GET_FPGA_TIME`**.
- Expected:
  - the extra-sample submission throws
    `IllegalStateException`;
  - `samplesFor(HAL_GET_FPGA_TIME, SATURATED_MULTI_THREAD).length`
    equals `100` — the terminal bucket count is exactly the
    documented per-point sample budget, not 101.
- The extra-sample's call class must match the bucket-protection
  assertion's call class. Submitting the extra sample for any
  *other* call class while asserting on `HAL_GET_FPGA_TIME`'s
  bucket would make the assertion trivially pass regardless of
  whether the sequencer accepted or rejected the sample, because
  the rejected (or wrongly-accepted) sample would land in a
  different bucket. Pinning both to `HAL_GET_FPGA_TIME` makes the
  bucket check non-trivial.
- Why the assertion targets `SATURATED_MULTI_THREAD` and not
  `MULTI_THREAD`: the terminal phase under cycle A is
  `SATURATED_MULTI_THREAD`. The rejected sample's
  bucket-protection assertion must target the bucket that the
  sequencer would have written to had the rejection not fired —
  i.e. the most-recently-active phase, the new terminal.

#### Q10' — `produces_records_for_every_call_x_point_combination`
- **Supersedes:** v0 `Q10`. Cardinality changes from 18 to 24; the
  Cartesian-product structure is otherwise identical.
- Layer / contract: `Sequencer.extractRecords()` covers exactly the
  6 × 4 = 24 (call_class, operating_point) pairs, each exactly
  once.
- Bug class: (a) a (call, point) pair receiving zero samples
  would render the CSV row as a missing line; (b) a count-only
  check would pass against duplicates + omissions.
- Inputs: full run with 6 classes × 4 points × `perPointCount=1`,
  no warmup. Round-robin submission, one sample per class per
  round, four rounds total.
- Expected:
  - `extractRecords().size() == 24`.
  - The set
    `{ (r.callClass, r.operatingPoint) : r ∈ extractRecords() }`
    equals the full Cartesian product
    `CallClass.values()` × `OperatingPoint.values()` (6 × 4 = 24
    elements). No pair missing, no pair duplicated.
- **Minimum implementation change.** v0 `Q10` has two hardcoded
  literals that drift with the enum and must both be updated:
  - `SequencerTest.java:180` — `for (int phase = 0; phase < 3; phase++)`
    becomes
    `for (int phase = 0; phase < OperatingPoint.values().length; phase++)`
    (or four explicit submission rounds since `perPointCount=1`).
    Without this change the sequencer is not done when
    `extractRecords()` is called and `extractRecords` throws
    `IllegalStateException`.
  - `SequencerTest.java:185` — `assertEquals(18, records.size())`
    becomes `assertEquals(24, records.size())`. Without this
    change the test fails the size assertion even after the loop
    bound is fixed.
  Both are loud failures rather than silent defects, but listing
  both is the complete instruction; missing the size literal would
  not change correctness but would cause a flag during
  implementation. Hardcoded counts that drift with the enum are
  exactly the regression class this rewrite exists to prevent.

> Other v0 `SequencerTest` cases (`Q1`, `Q2`, `Q3`, `Q4`, `Q7`,
> `Q8`, `Q9`) are unchanged — they exercise warmup / IDLE→
> SATURATED / SATURATED→MULTI_THREAD transitions whose semantics
> the new last-phase addition does not alter. (`Q6` is rewritten
> in place — see `Q6'` above.)

### `BenchmarkPipelineTest`

#### P1' — `pipeline_produces_documented_csv_for_synthetic_run`
- **Supersedes:** v0 `P1`. Same wiring, more cells.
- Layer / contract: integration of `Sequencer` → `Statistics` →
  `BenchmarkRecord` → `CsvWriter` over the cycle-A 6×4 cell space.
- Bug class: same as v0 `P1` (statistics-vs-block-size argument
  swap, BenchmarkRecord field swap, sort-order regression),
  scaled to the larger cell space. New bug class: a
  `WorkerSpec` regression that reorders the operating-point list
  the runner walks in such a way that samples land in the wrong
  bucket would shift the magnitudes of every cycle-A row;
  `P1'` catches this end-to-end.
- Pinned `OperatingPoint` declaration order: ordinals `0/1/2/3`
  for `IDLE_BUS / SATURATED_BUS / MULTI_THREAD /
  SATURATED_MULTI_THREAD` (per "Suite-level concerns / `OperatingPoint`
  enum contract (cycle-A revision)").
- Pinned `CallClass` declaration order: unchanged from v0.
- Inputs:
  - `Sequencer` configured with all 6 v0 call classes,
    `warmupCount = 1`, `perPointCount = 4`. Submission via
    round-robin discipline (one sample per class per round,
    in `CallClass.values()` order).
  - For each `(c, p)` cell, submit **4 identical samples** of
    value `s_ns(c, p) = (6 * p.ordinal() + c.ordinal()) * 1000` ns.
    Formula is identical to v0; only the cell space grows from
    18 cells (`p.ordinal() ∈ {0,1,2}`) to 24 cells
    (`p.ordinal() ∈ {0,1,2,3}`).
  - `Statistics.compute(samples, blockSize)` invoked per cell with
    each call class's documented `blockSize`.
  - `RunMetadata` fixture identical to v0 `P1` (and v0 `C1`) so the
    preamble matches v0's expected string byte-for-byte.
- Worked derivations, **three representative new rows** (one per
  blockSize bucket so each per-call-division formula is
  spot-checked):

  **Row (HAL_GET_FPGA_TIME, SATURATED_MULTI_THREAD):** c=0, p=3,
  blockSize=100. Cell index = 6×3 + 0 = 18. Samples (ns):
  `[18000, 18000, 18000, 18000]`.
  - block mean (ns) = 18000; per-call mean = 18000/100 = 180 ns =
    `0.180` µs.
  - sample stddev = 0; per-call = `0.000` µs.
  - p99 (R-7 on identical samples) = 18000/100 = `0.180` µs.
  - outlier_rate = 0 (stddev=0 → S7 contract).
  - Expected row:
    `hal_get_fpga_time,saturated_multi_thread,4,0.180,0.000,0.180,0.000000`.

  **Row (DS_STATE_READ, SATURATED_MULTI_THREAD):** c=2, p=3,
  blockSize=10. Cell index = 6×3 + 2 = 20. Samples (ns):
  `[20000, 20000, 20000, 20000]`.
  - per-call mean = 20000/10 = 2000 ns = `2.000` µs.
  - stddev = `0.000` µs; p99 = `2.000` µs; outlier_rate = 0.
  - Expected row:
    `ds_state_read,saturated_multi_thread,4,2.000,0.000,2.000,0.000000`.

  **Row (CAN_FRAME_WRITE, SATURATED_MULTI_THREAD):** c=5, p=3,
  blockSize=1. Cell index = 6×3 + 5 = 23. Samples (ns):
  `[23000, 23000, 23000, 23000]`.
  - per-call mean = 23000 ns = `23.000` µs.
  - stddev = `0.000` µs; p99 = `23.000` µs; outlier_rate = 0.
  - Expected row:
    `can_frame_write,saturated_multi_thread,4,23.000,0.000,23.000,0.000000`.

  All three rows show that `1000 / blockSize ∈ {10, 100, 1000}` is
  integer for every v0 call class, so per-call µs values fall on
  exact multiples of `0.001` µs and `%.3f` formatting is unambiguous
  (no halfway-rounding cases) — the v0 `P1` integer-µs property
  carries forward into the cycle-A rows.

- **All six new rows** (the `SATURATED_MULTI_THREAD` column of the
  6×4 grid), derived from the same `(6 * p.ordinal() + c.ordinal()) * 1000`
  formula with `p.ordinal()=3`. The three rows above are spot-
  checked manually; the other three follow the same arithmetic and
  are pinned here so a reviewer can verify the regenerated fixture
  end-to-end without recomputing:

  | call_class            | c | idx | sample_ns | blockSize | per_call_ns | µs       | expected fixture row                                                     |
  |-----------------------|---|-----|-----------|-----------|-------------|----------|--------------------------------------------------------------------------|
  | `hal_get_fpga_time`   | 0 | 18  | 18000     | 100       | 180         | `0.180`  | `hal_get_fpga_time,saturated_multi_thread,4,0.180,0.000,0.180,0.000000`  |
  | `hal_notifier_wait`   | 1 | 19  | 19000     | 1         | 19000       | `19.000` | `hal_notifier_wait,saturated_multi_thread,4,19.000,0.000,19.000,0.000000`|
  | `ds_state_read`       | 2 | 20  | 20000     | 10        | 2000        | `2.000`  | `ds_state_read,saturated_multi_thread,4,2.000,0.000,2.000,0.000000`      |
  | `power_state_read`    | 3 | 21  | 21000     | 10        | 2100        | `2.100`  | `power_state_read,saturated_multi_thread,4,2.100,0.000,2.100,0.000000`   |
  | `can_frame_read`      | 4 | 22  | 22000     | 1         | 22000       | `22.000` | `can_frame_read,saturated_multi_thread,4,22.000,0.000,22.000,0.000000`   |
  | `can_frame_write`     | 5 | 23  | 23000     | 1         | 23000       | `23.000` | `can_frame_write,saturated_multi_thread,4,23.000,0.000,23.000,0.000000`  |

  All six per-call-µs values are exact-integer × `0.001 µs`
  multiples (since `1000 / blockSize ∈ {10, 100, 1000}`), so the
  `%.3f` formatting has no halfway-rounding ambiguity — the
  hand-pinned values in this table are the exact byte sequences
  the regenerated fixture must contain for the
  `saturated_multi_thread` rows.

- **Inherited-cell invariant.** The 18 v0 cells in the regenerated
  `pipeline_expected.csv` must remain byte-identical to the v0
  fixture for the cells that v0 already measured (every
  `(call_class, operating_point)` pair where
  `operating_point ∈ {idle_bus, multi_thread, saturated_bus}`).
  This invariant is exactly what the ordinal-preservation
  decision (`SATURATED_MULTI_THREAD` declared last so the first
  three ordinals are unchanged) enables: same formula, same
  per-cell sample value, same row output. A diff between the
  v0 fixture and the cycle-A fixture should show only six new
  rows added (one per call class, all `saturated_multi_thread`)
  and zero modifications to existing rows. **Implementer must
  verify this diff before committing the regenerated fixture.**

- **Fixture generation procedure.** The 24-row
  `pipeline_expected.csv` is generated by running `P1'` against
  the implemented Sequencer + CsvWriter and committing the
  captured output. Three rows in the table above
  (`hal_get_fpga_time`, `ds_state_read`, `can_frame_write` — one
  per blockSize bucket: 100, 10, 1) are spot-checked manually
  before commit. The other three rows in the new column
  (`hal_notifier_wait`, `power_state_read`, `can_frame_read`)
  follow the same formula and are pinned in the table above; the
  byte-equal check itself catches any discrepancy in those rows.
  The 18 inherited rows are verified by the diff-against-v0-
  fixture step described in the inherited-cell invariant above.

- Full fixture: `src/test/resources/pipeline_expected.csv` is
  regenerated to 24 data rows + the v0 preamble + header. Sort
  order is per `csvLabel` lex on both keys, so within each
  call_class the four operating-point rows appear as
  `idle_bus, multi_thread, saturated_bus, saturated_multi_thread`.
- Expected: the full output string from `CsvWriter.write(...)` is
  byte-equal to the regenerated fixture.
- Tolerance: byte-equal.
- Determinism: every input is a deterministic function of
  `(call_class, operating_point, sample_index)`; no clocks; no
  random values; enum ordinals pinned.
- What this catches that the units don't:
  1. v0's listed wiring defects, scaled to the new cell space.
  2. A row-sort regression where the new `saturated_multi_thread`
     row appears before `saturated_bus` (e.g. via case-insensitive
     comparator mistake). The fixture pins the lex order
     `saturated_bus < saturated_multi_thread`.
  3. A `Sequencer` that closes after `MULTI_THREAD` (the v0
     terminal phase): `SATURATED_MULTI_THREAD` rows would all
     have `sample_count = 0` and the test would fail on every
     fourth row of every call class.
  4. A `WorkerSpec` regression that doesn't change measurements
     directly but does change the runner's operating-point
     iteration order — caught because the byte-equal fixture
     pins which sample value lands in which cell.

## What this plan does NOT cover

- The HAL-bound CAN-spam worker itself (`bench.runner` / HAL JNI)
  — covered by the manual hardware-deploy procedure inherited
  from v0; no desktop unit test path. Per `D-RB-7`, the WorkerSpec
  contract is the testable seam.
- Real-RIO measurements of the new operating point — picked up at
  hardware-validation time alongside the existing v0 sweep.
- Notifier-jitter percentiles, histogram, and p50/p999 columns —
  RB-cycle-B.
- Hardware device-count sweep — RB-cycle-C.
- The `bus_devices` preamble line gaining new (cycle-C) semantics
  — that change happens in cycle C's plan, not here.

## Determinism summary

- All numerical inputs are constants or pinned-seed `Random`.
- No test reads the wall clock, network, or filesystem (the
  fixture is loaded via classpath).
- `run_timestamp_iso8601` continues to be injected via
  `RunMetadata`, never read from `Instant.now()` inside the SUT.
- Every test constructs its own SUT; `fastForwardToPoint` is
  stateless (a static helper that takes a fresh `Sequencer`).
- Run order independence: any subset of tests can run in any order
  without affecting outcomes.

## Citations

- Inherited from v0 plan: Hyndman & Fan 1996 (R-7); Wikipedia
  sample-stddev worked example; Java `Formatter` `%f` HALF_UP.
- New: none required — cycle A introduces no new physical
  constants, statistical methods, or formatting rules. The new
  semantics for `SATURATED_BUS` (software-driven) is recorded in
  the skill as design decision `D-RB-8`.
