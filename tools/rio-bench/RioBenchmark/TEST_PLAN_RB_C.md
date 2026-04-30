# rio-bench RB-cycle-C test plan

**Scope:** add a software-driven CAN-bus-load sweep at three new
load tiers, plus a runtime device-scan that records the count and
IDs of physical CAN devices on the bus during the run. Together
these give the sim core a load-scaling curve (1/4/8 software spam
workers) for HAL-call cost vs bus utilization, plus a record of
what real-device baseline traffic was present.

**Status:** **draft, awaiting `test-reviewer` verdict**.

**Delta convention.** Same as cycles A/B: only new and modified
tests are specified here. Predecessors:

- v0 plan (`TEST_PLAN.md`)
- cycle-A plan (`TEST_PLAN_RB_A.md`)
- cycle-B plan (`TEST_PLAN_RB_B.md`)

## Context for the reviewer

### Cycle C's place in the post-v0 push

Third and final cycle of the post-v0 fidelity push:

- **Cycle A** (landed): `SATURATED_MULTI_THREAD` cross-product
  operating point + software-driven CAN saturation (`D-RB-8`).
- **Cycle B** (landed): `p50_us` and `p999_us` columns inline.
- **Cycle C** (this plan): software-driven device-count load
  tiers + runtime device-scan.

Suite is at 54 tests (41 v0 + 7 cycle-A + 6 cycle-B). Cycle C
adds further coverage; the final count is enumerated under
"Validation status" once the cycle lands.

### What cycle C adds

#### 1. `WorkerSpec` migration: `boolean canSpam` → `int canSpamWorkers`

Cycle A's `WorkerSpec` carried a `boolean canSpam` because the
spam-worker count was always 0 or 1. Cycle C parameterizes the
count: `(int cpuWorkers, int canSpamWorkers)`. The four cycle-A
mappings translate trivially:

| OperatingPoint              | cycle-A `(cpu, canSpam)` | cycle-C `(cpu, spam)` |
|-----------------------------|--------------------------|-----------------------|
| `IDLE_BUS`                  | `(0, false)`             | `(0, 0)`              |
| `SATURATED_BUS`             | `(0, true)`              | `(0, 1)`              |
| `MULTI_THREAD`              | `(3, false)`             | `(3, 0)`              |
| `SATURATED_MULTI_THREAD`    | `(3, true)`              | `(3, 1)`              |

Plus four new operating points:

| OperatingPoint                       | `(cpu, spam)` | csvLabel                          |
|--------------------------------------|---------------|-----------------------------------|
| `SATURATED_BUS_4`                    | `(0, 4)`      | `saturated_bus_4`                 |
| `SATURATED_BUS_8`                    | `(0, 8)`      | `saturated_bus_8`                 |
| `SATURATED_BUS_4_MULTI_THREAD`       | `(3, 4)`      | `saturated_bus_4_multi_thread`    |
| `SATURATED_BUS_8_MULTI_THREAD`       | `(3, 8)`      | `saturated_bus_8_multi_thread`    |

The `4` and `8` are software-spam-worker counts, not real-device
counts. The runtime scan records the real-device count separately
(see #2). Sim-core consumers compute total bus utilization as
"real-device baseline + software spam."

#### 2. Runtime device-scan + always-emitted `bus_devices` preamble

`bench.runner.DeviceScan` is a new pure-Java record + helper:

- `DeviceScan(int aliveCount, List<Integer> aliveIds, int probeRangeStartInclusive, int probeRangeEndInclusive)`
  — a defensive record (constructor-validated: list is sorted
  strictly ascending, values in range, count matches list size,
  range bounds non-negative and non-inverted). Field type is
  `List<Integer>` rather than `int[]` so the record's derived
  `equals`/`hashCode` use structural list equality; see
  "Suite-level concerns / `DeviceScan` field type" above.
- `static DeviceScan fromProbeResults(boolean[] aliveByIndex, int probeRangeStart)`
  — interprets a HAL-bound probe result. Caller passes a
  `boolean[]` of `aliveByIndex.length` results, where index `i`
  is "is device with ID `(probeRangeStart + i)` alive?". Returns
  a `DeviceScan` record with the alive-count and a sorted
  alive-IDs list.

The HAL-bound probe (`bench.runner.CallBindings.probeTalonFxAlive(int firstId, int lastId)`)
returns the `boolean[]` for `DeviceScan.fromProbeResults` to
consume. The HAL call itself is `new TalonFX(id).getVersion().isAlive()`
(or equivalent CTRE Phoenix6 API; pinned in the implementer's
follow-up); not unit-tested per `D-RB-7`.

The probe range is configurable in `BenchmarkRunner` (defaulting
to 0..62, the full CAN ID space). Operator can narrow it if a
known device range is sufficient — narrower range ⇒ shorter boot
time.

`RunMetadata.busDevices` becomes always-populated by the runner
(no longer `Optional.empty()` after a real run), with the value
sourced from the scan. The writer's behavior is unchanged: it
emits `# bus_devices=N` whenever the field is `Optional.of(N)`.
The cycle-A change that made this preamble informational stands;
cycle C just guarantees it's always present in real runs.

The runner also writes the alive-IDs list to the DataLog at boot
(`[rio-bench] device scan: alive_count=N, ids=[...]`) so the
operator can confirm the bus matches expectations before the
sweep starts.

#### 3. CSV row sort under cycle C

Lex sort on `csvLabel`s. With 8 operating points the per-call-class
row order is:

```
idle_bus
multi_thread
saturated_bus
saturated_bus_4
saturated_bus_4_multi_thread
saturated_bus_8
saturated_bus_8_multi_thread
saturated_multi_thread
```

(`saturated_bus_4_multi_thread < saturated_bus_8` because
`4` < `8` in lex; `saturated_bus_8_multi_thread <
saturated_multi_thread` because `8` < `m`.)

#### 4. CSV preamble version unchanged

Still `# rio-bench v0`. No column added or removed; only rows
gain. `D-RB-8`'s deferred version-bump decision is inherited.

### What does NOT change

- `Statistics`, `BenchmarkRecord`, `CsvWriter` (header line,
  formatters, sort comparator) — all unchanged from cycle B.
- Block sizes per call class — unchanged.
- Outlier definition, GC policy, decimal formatting,
  software-driven saturation per `D-RB-8` — all unchanged.
- `HAL_NOTIFIER_WAIT` measurement and the cycle-B percentile
  columns — unchanged.

### What this cycle deliberately defers

- **Hardware-driven calibration cycle.** Comparing
  software-spam-driven measurements to real-CTRE-status-frame
  measurements would bound the fidelity gap of `D-RB-8`. That's a
  separate cycle (call it RB-cycle-D, post-cycle-C). Cycle C
  doesn't claim to measure real-device traffic costs; it measures
  software-driven load-tier costs with the actual real-device
  baseline recorded for the sim core to interpret.
- **Per-call-class spam-count tuning.** Every operating point
  uses the same spam count regardless of which call class is
  being timed. A future cycle could let each call class declare
  its own tier set, but no demand justifies the schema surgery.

## Suite-level concerns

- **Framework, determinism, no-HAL-imports, no-wall-clock** rules
  inherited.
- **`OperatingPoint` enum contract (cycle-C revision).** Eight
  members, declaration order pinned for `BenchmarkPipelineTest`
  cell-index formula. Order:
  ```
  IDLE_BUS, SATURATED_BUS, MULTI_THREAD, SATURATED_MULTI_THREAD,
  SATURATED_BUS_4, SATURATED_BUS_8,
  SATURATED_BUS_4_MULTI_THREAD, SATURATED_BUS_8_MULTI_THREAD
  ```
  Ordinals `0..7`. The first four ordinals are preserved from
  cycle A so cycle-A's cell-index formula remains valid for those
  cells; the new members are appended in declaration order
  matching their `(cpu, spam)` grouping (no-cpu spam tiers first,
  then with-cpu cross-products).
- **`WorkerSpec` exhaustiveness invariant** continues — switch
  expression with no `default` branch; the rule is mirrored as
  an inline comment in `WorkerSpec.java`.
- **`DeviceScan` record-validation invariant.** The constructor
  enforces:
  - `aliveCount >= 0`
  - `aliveCount == aliveIds.size()`
  - `aliveIds` is sorted strictly ascending
  - every `id` in `aliveIds` is in `[probeRangeStart,
    probeRangeEnd]` inclusive
  - `probeRangeStart >= 0` (CAN ID space starts at 0; negative
    IDs are nonsense regardless of the probe range)
  - `probeRangeStart <= probeRangeEnd`
  These invariants are tested directly by `DeviceScanTest`
  `D6`–`D10` (constructor) and also exercised transitively by
  the factory tests `D1`–`D5`/`D11`.
- **`DeviceScan` field type for `aliveIds`: `List<Integer>` (not
  `int[]`).** Java `record` types synthesize `equals` /
  `hashCode` via per-field `Objects.equals`, which falls back
  to reference equality on arrays — not `Arrays.equals`. Using
  an `int[]` field would silently break structural equality
  for downstream callers (`Set` membership, map keys, equality
  asserts). `List<Integer>` gives the record correct derived
  equals/hashCode out of the box. The factory
  `fromProbeResults` returns
  `List.copyOf(aliveIdsCollectedAsList)` so the field is
  immutable; the constructor likewise wraps any caller-supplied
  list with `List.copyOf` (defensive copy) before assigning.
  Tested by `D12`.
- **Factory `DeviceScan.fromProbeResults` rejects zero-length
  probe arrays.** A zero-length `boolean[]` with start=N would
  imply `probeRangeEnd = N + 0 - 1 = N - 1`, violating
  `probeRangeStart <= probeRangeEnd`. Rather than letting the
  constructor throw a confusing "inverted range" error, the
  factory rejects zero-length input directly with a diagnostic
  message. Tested by `D11`.
- **CSV preamble row count is unchanged** at 10 preamble lines +
  1 header line = 11 lines before data rows. The runner now
  always populates `bus_devices`, but that field's emission is
  conditional on `Optional` presence in `CsvWriter`, and the
  empty-record-list test (`C9`, v0) still produces 11 lines when
  metadata's `busDevices` is `Optional.empty()` (synthetic test
  case). Tests that exercise the runner-populated case use a
  fixture with `busDevices = Optional.of(N)` and assert 12 lines
  + data rows, depending on whether records are present.
- **Pipeline fixture regeneration.** The fixture file at
  `src/test/resources/pipeline_expected.csv` is regenerated for
  the 6×8=48-cell space with the cycle-C cell formula (same
  shape as cycle B, `+1` offset preserved). The regenerated
  fixture's preamble continues to use `Optional.empty()` for
  `bus_devices` so the pipeline test does not depend on the scan
  result.
- **Test-file naming.** Changes touch:
  - `OperatingPointTest` rewritten in place (now 8 members,
    8 csvLabels, 8-element lex-sort order).
  - `WorkerSpecTest` rewritten in place (boolean → int, 4 new
    cell tests, distinctness expanded to 8).
  - `SequencerTest` `Q5'`, `Q5b`, `Q6'`, `Q10'` rewritten in
    place to walk 8 phases instead of 4. The boundary checks at
    every transition are preserved; the helper iterates
    `OperatingPoint.values()` and so picks up the new members
    automatically.
  - New file: `src/test/java/frc/robot/bench/runner/DeviceScanTest.java`.
  - `BenchmarkPipelineTest.P1''` rewritten in place; fixture
    regenerated.

## Per-test specifications

### `OperatingPointTest` (rewritten in place)

#### O1'' — `operating_point_enum_has_eight_members_in_documented_order`
- **Supersedes:** cycle-A `O1'`.
- Bug class caught: ordinal drift would shift cycle-A/B-inherited
  cells in the pipeline fixture.
- Inputs: `OperatingPoint.values()`.
- Expected:
  - `length == 8`.
  - `values()[0..3]` are unchanged from cycle A
    (`IDLE_BUS, SATURATED_BUS, MULTI_THREAD, SATURATED_MULTI_THREAD`).
  - `values()[4] == SATURATED_BUS_4`,
    `values()[5] == SATURATED_BUS_8`,
    `values()[6] == SATURATED_BUS_4_MULTI_THREAD`,
    `values()[7] == SATURATED_BUS_8_MULTI_THREAD`.

#### O2'' — `csv_label_matches_documented_table`
- **Supersedes:** cycle-A `O2'`. Adds four new label assertions.
- Inputs: each `OperatingPoint` value.
- Expected:
  - first four labels unchanged from cycle A;
  - `SATURATED_BUS_4.csvLabel() == "saturated_bus_4"`,
  - `SATURATED_BUS_8.csvLabel() == "saturated_bus_8"`,
  - `SATURATED_BUS_4_MULTI_THREAD.csvLabel() == "saturated_bus_4_multi_thread"`,
  - `SATURATED_BUS_8_MULTI_THREAD.csvLabel() == "saturated_bus_8_multi_thread"`.

#### O3' — `csv_labels_lex_sort_in_documented_order`
- **Supersedes:** cycle-A `O3`. The hand-pinned list grows to 8
  entries.
- Bug class caught: same as cycle-A `O3` (comparator regression
  surviving fixture regeneration).
- Inputs: the eight `csvLabel()` strings.
- Expected: when sorted lexicographically:
  ```
  idle_bus
  multi_thread
  saturated_bus
  saturated_bus_4
  saturated_bus_4_multi_thread
  saturated_bus_8
  saturated_bus_8_multi_thread
  saturated_multi_thread
  ```
  Verified by hand:
  - `i` < `m` < `s` (first three pinned).
  - Among `saturated_*`: prefix `saturated_` matches; then `b` <
    `m`, so `saturated_bus*` < `saturated_multi_thread`.
  - Within `saturated_bus_*`: `bus` then `_`; `saturated_bus` <
    `saturated_bus_*` (shorter is less since `_` follows after
    end-of-string in lex). So `saturated_bus` first.
  - Then `saturated_bus_4*` vs `saturated_bus_8*`: `4` < `8`, so
    all `_4*` before `_8*`.
  - Within `_4*`: `saturated_bus_4` (no suffix) < `saturated_bus_4_multi_thread`.
  - Same within `_8*`.
  - All these orderings are checked at the byte level; a future
    label that breaks them (e.g. inserting `saturated_bus_16` —
    `1` < `4` lex even though numerically 16 > 4 — would land
    between `saturated_bus` and `saturated_bus_4`; this test
    would catch it).

### `WorkerSpecTest` (rewritten in place; boolean → int migration)

System under test: `bench.runner.WorkerSpec`, now
`(int cpuWorkers, int canSpamWorkers)`.

#### W1' — `forPoint_idle_bus_returns_no_workers`
- **Supersedes:** cycle-A `W1`. Field type changes from
  `boolean canSpam` to `int canSpamWorkers`.
- Inputs: `OperatingPoint.IDLE_BUS`.
- Expected: `WorkerSpec(0, 0)`.

#### W2' — `forPoint_saturated_bus_returns_one_can_spam_worker`
- **Supersedes:** cycle-A `W2`. Documents that the cycle-A
  semantic `(0, true)` is now spelled `(0, 1)` (one spam worker)
  to be consistent with the cycle-C 4 / 8 tiers.
- Inputs: `OperatingPoint.SATURATED_BUS`.
- Expected: `WorkerSpec(0, 1)`.

#### W3 — `forPoint_multi_thread_returns_three_cpu_workers_no_can_spam`
- **Supersedes:** cycle-A `W3`. `canSpam=false` becomes
  `canSpamWorkers=0`.
- Inputs: `OperatingPoint.MULTI_THREAD`.
- Expected: `WorkerSpec(3, 0)`.

#### W4 — `forPoint_saturated_multi_thread_returns_three_cpu_workers_and_one_can_spam`
- **Supersedes:** cycle-A `W4`.
- Inputs: `OperatingPoint.SATURATED_MULTI_THREAD`.
- Expected: `WorkerSpec(3, 1)`.

#### W6 — `forPoint_saturated_bus_4_returns_four_can_spam_workers`
- **New.**
- Bug class: cycle-C-specific. A defect that mistakenly clamps
  `canSpamWorkers` to `0..1` (i.e. preserves cycle-A's boolean
  semantic) would silently return `(0, 1)` for this op-point and
  collapse it onto `SATURATED_BUS`. The four-spam-worker tier
  loses its distinguishing load profile.
- Inputs: `OperatingPoint.SATURATED_BUS_4`.
- Expected: `WorkerSpec(0, 4)`.

#### W7 — `forPoint_saturated_bus_8_returns_eight_can_spam_workers`
- **New.** Symmetric to `W6` at the heaviest tier.
- Inputs: `OperatingPoint.SATURATED_BUS_8`.
- Expected: `WorkerSpec(0, 8)`.

#### W8 — `forPoint_saturated_bus_4_multi_thread_returns_three_cpu_and_four_spam`
- **New.** Cross-product cell at the moderate tier.
- Bug class: a partial-cell defect (forgetting `cpuWorkers=3` or
  `canSpamWorkers=4`) would collapse the cell onto either
  `MULTI_THREAD` or `SATURATED_BUS_4` and erase the cross-product
  fidelity gain.
- Inputs: `OperatingPoint.SATURATED_BUS_4_MULTI_THREAD`.
- Expected: `WorkerSpec(3, 4)`.

#### W9 — `forPoint_saturated_bus_8_multi_thread_returns_three_cpu_and_eight_spam`
- **New.** Cross-product cell at the heaviest tier.
- Inputs: `OperatingPoint.SATURATED_BUS_8_MULTI_THREAD`.
- Expected: `WorkerSpec(3, 8)`.

#### W5' — `forPoint_returns_distinct_spec_for_every_operating_point`
- **Supersedes:** cycle-A `W5`. Set size grows from 4 to 8.
- Layer / contract: cross-product distinctness across all eight
  operating points.
- Bug class: any two operating points collapsing onto the same
  spec, e.g. a typo that returns `(0, 4)` for both
  `SATURATED_BUS_4` and `SATURATED_BUS_4_MULTI_THREAD`.
- Inputs: `OperatingPoint.values()`.
- Expected: `Set<WorkerSpec>` of size 8.
- Distinctness arithmetic check: the eight tuples are
  `(0,0), (0,1), (0,4), (0,8), (3,0), (3,1), (3,4), (3,8)`. All
  mutually distinct. ✓

> **Note on coverage.** The new W6/W7/W8/W9 each pin a single
> cell. The cross-product distinctness invariant (`W5'`) is the
> "big picture" check; the per-cell tests are the "specific
> values" checks. Overlap is intentional — `W5'` catches any
> defect that collapses two cells; per-cell tests localize the
> failure to the specific arm.

### `SequencerTest` (modified in place)

#### Q5'' — `transitions_multi_thread_to_saturated_multi_thread_after_per_point_samples`
- **Supersedes:** cycle-A `Q5'`. Implementation unchanged
  (multi_thread → saturated_multi_thread); only the surrounding
  test order changes because cycle C adds more phases after
  saturated_multi_thread.
- The plan note: when iterating `OperatingPoint.values()`, the
  `MULTI_THREAD → SATURATED_MULTI_THREAD` transition fires at
  the third advancement; cycle C does not change this. The test
  body is unchanged from cycle A — **except** the implementer
  must update the cycle-A diagnostic message on the after-600
  `assertFalse(s.isDone(), ...)` from "SATURATED_MULTI_THREAD is
  terminal phase" to a cycle-C-accurate string like "not done
  — SATURATED_BUS_4 and four more phases remain." Stale
  diagnostic messages mislead future debugging.

#### Q5b' — `transitions_saturated_multi_thread_to_saturated_bus_4_after_per_point_samples`
- **Supersedes:** cycle-A `Q5b` (which asserted that
  `SATURATED_MULTI_THREAD` is terminal). Under cycle C,
  `SATURATED_MULTI_THREAD` is no longer terminal — there are
  four more phases after it. The test now asserts the next
  transition fires.
- Bug class: a hardcoded `PHASES.length == 4` check from
  cycle A that did not get updated when cycle C added the four
  new op-points.
- Inputs: fresh sequencer with `warmupCount=10`, `perPointCount=100`,
  all 6 call classes; `fastForwardToPoint(SATURATED_MULTI_THREAD, ...)`;
  then submit 600 SATURATED_MULTI_THREAD samples.
- Expected:
  - after 599: `currentPoint() == SATURATED_MULTI_THREAD`,
    `isDone() == false`;
  - after 600: `currentPoint() == SATURATED_BUS_4`,
    `isDone() == false` (NOT done — three more phases remain).

#### Q5c — `transitions_saturated_bus_4_to_saturated_bus_8`
- **New.** Pins the intermediate boundary at `SATURATED_BUS_4 →
  SATURATED_BUS_8`.
- Setup: `Sequencer(warmupCount=10, perPointCount=100, ALL_CLASSES)`,
  then `fastForwardToPoint(SATURATED_BUS_4, ...)`. Then submit
  600 samples for the SATURATED_BUS_4 phase round-robin.
- Boundary-check pattern (identical to `Q5'` / `Q5b'`):
  - **After 599 round-robin samples:**
    `currentPoint() == SATURATED_BUS_4`, `isDone() == false`.
  - **After the 600th sample:**
    `currentPoint() == SATURATED_BUS_8` AND `isDone() == false`.

#### Q5d — `transitions_saturated_bus_8_to_saturated_bus_4_multi_thread`
- **New.** Pins the boundary at `SATURATED_BUS_8 →
  SATURATED_BUS_4_MULTI_THREAD`.
- Setup: `fastForwardToPoint(SATURATED_BUS_8, ...)`, then 600
  SATURATED_BUS_8 samples.
- Boundary-check pattern:
  - **After 599 samples:** `currentPoint() == SATURATED_BUS_8`,
    `isDone() == false`.
  - **After 600 samples:**
    `currentPoint() == SATURATED_BUS_4_MULTI_THREAD`,
    `isDone() == false`.

#### Q5e — `transitions_saturated_bus_4_multi_thread_to_saturated_bus_8_multi_thread`
- **New.** Pins the boundary at `SATURATED_BUS_4_MULTI_THREAD →
  SATURATED_BUS_8_MULTI_THREAD`.
- Setup: `fastForwardToPoint(SATURATED_BUS_4_MULTI_THREAD, ...)`,
  then 600 SATURATED_BUS_4_MULTI_THREAD samples.
- Boundary-check pattern:
  - **After 599 samples:**
    `currentPoint() == SATURATED_BUS_4_MULTI_THREAD`,
    `isDone() == false`.
  - **After 600 samples:**
    `currentPoint() == SATURATED_BUS_8_MULTI_THREAD`,
    `isDone() == false`.

> **Why the explicit `isDone() == false` matters at every
> intermediate boundary:** the assertion is not implied by the
> transition assertion. A premature-done regression at any
> intermediate boundary (e.g. a hardcoded `phaseIndex >= 6`
> check that fires at `SATURATED_BUS_4` for `Q5c`) would change
> `currentPoint()` to a sentinel and set `isDone() == true` at
> the same time; only the explicit `isDone() == false`
> assertion catches that.

#### Q5f — `transitions_saturated_bus_8_multi_thread_to_done`
- **New.** Pins the new terminal boundary (analogue of cycle-A's
  `Q5b` and v0's `Q5`).
- Setup: `Sequencer(warmupCount=10, perPointCount=100, ALL_CLASSES)`,
  then `fastForwardToPoint(SATURATED_BUS_8_MULTI_THREAD, ...)`.
  Then submit 600 samples for the SATURATED_BUS_8_MULTI_THREAD
  phase round-robin (one sample per call class per round, 100
  rounds × 6 classes = 600 samples).
- Boundary-check pattern:
  - **After 599 round-robin samples:**
    `currentPoint() == SATURATED_BUS_8_MULTI_THREAD`,
    `isDone() == false`.
  - **After the 600th sample:** `isDone() == true`. (Do not
    assert `currentPoint()` after `done == true`; per the v0
    accessor contract, `currentPoint()` is undefined in the
    done state.)

#### Q6'' — `done_state_rejects_further_samples`
- **Supersedes:** cycle-A `Q6'`. Terminal phase shifts from
  `SATURATED_MULTI_THREAD` to `SATURATED_BUS_8_MULTI_THREAD`.
- Setup: `Sequencer(warmupCount=10, perPointCount=100, ALL_CLASSES)`,
  then `fastForwardToPoint(SATURATED_BUS_8_MULTI_THREAD, ...)`,
  then submit the full SATURATED_BUS_8_MULTI_THREAD quota (100
  rounds × 6 classes = 600 samples) so `isDone() == true`. Then
  submit one more sample **specifically for `HAL_GET_FPGA_TIME`**
  (the call class must match the bucket-protection assertion;
  see cycle-A `Q6'` for the non-triviality rationale).
- Expected:
  - the extra-sample submission throws `IllegalStateException`;
  - `samplesFor(HAL_GET_FPGA_TIME, SATURATED_BUS_8_MULTI_THREAD).length == 100`
    — the terminal-bucket count is exactly the documented
    per-point sample budget, not 101.

#### Q10'' — `produces_records_for_every_call_x_point_combination`
- **Supersedes:** cycle-A `Q10'`. Cardinality grows from 24 to
  48.
- Inputs: full run with 6 classes × 8 points × `perPointCount=1`,
  no warmup.
- Expected: `extractRecords().size() == 48`. Cartesian product
  has 48 distinct `(call, point)` pairs.
- **Minimum implementation change** (mirrors cycle-A `Q10'`'s
  guidance): the sequencer-test body must use
  `OperatingPoint.values().length` for the loop bound and
  `ALL_CLASSES.size() * OperatingPoint.values().length` for the
  size literal — exactly the cycle-A form, which already
  generalizes correctly to 8 phases without modification.
  This is verification-by-rerun, not a code change.

> Other v0 / cycle-A `SequencerTest` cases (`Q1, Q2, Q3, Q4,
> Q7, Q8, Q9` and the `fastForwardToPoint` helper) are unchanged.

### `DeviceScanTest` (new test class)

System under test: `bench.runner.DeviceScan` — pure-Java record
+ static factory.

#### D1 — `from_probe_results_counts_alive_devices`
- Layer / contract: `DeviceScan.fromProbeResults(boolean[], int)`.
- Bug class: scan loses or duplicates an alive device, mis-counts
  the total, or skews the probe-range mapping.
- Inputs: `aliveByIndex = {false, true, false, true, true, false}`,
  `probeRangeStart = 10`. So devices at IDs 11, 13, 14 are alive;
  10, 12, 15 are dead. Range is `[10, 15]`.
- Expected:
  - `result.aliveCount() == 3`.
  - `result.aliveIds()` is `{11, 13, 14}` (sorted, no duplicates).
  - `result.probeRangeStartInclusive() == 10`.
  - `result.probeRangeEndInclusive() == 15` (start + length - 1).

#### D2 — `from_probe_results_handles_empty_alive_set`
- Bug class: a defect that throws on no-alive-devices would
  brick the runner if the operator forgets to wire any.
- Inputs: `aliveByIndex = {false, false, false}`, start=0.
- Expected: `aliveCount=0`, `aliveIds=[]`, range `[0, 2]`. No
  exception.

#### D3 — `from_probe_results_handles_full_alive_set`
- Bug class: off-by-one when every device is alive.
- Inputs: `aliveByIndex = {true, true, true}`, start=20.
- Expected: `aliveCount=3`, `aliveIds=[20, 21, 22]`, range
  `[20, 22]`.

#### D4 — `from_probe_results_rejects_null_input`
- Bug class: Java contract — JLS says `null` array param throws
  NPE on `.length`. Pin behavior so a future refactor with a
  silent default is caught.
- Inputs: `null`, start=0.
- Expected: `NullPointerException`.

#### D5 — `from_probe_results_rejects_negative_start`
- Bug class: negative probe-range start would produce nonsense
  device IDs (CAN ID space starts at 0). The factory delegates
  to the canonical constructor; the constructor's
  `probeRangeStart >= 0` invariant (see "Suite-level concerns /
  `DeviceScan` record-validation invariant") fires
  transitively. `D5` pins the factory-level behavior; `D10`
  pins the constructor-level invariant directly.
- Inputs: `aliveByIndex = {true}`, start = -1.
- Expected: `IllegalArgumentException`.

#### D6 — `record_constructor_rejects_unsorted_alive_ids`
- Bug class: a defect that constructs the record directly
  (bypassing `fromProbeResults`) with unsorted IDs would
  produce an invariant-violating record. The constructor's
  validation catches this.
- Inputs: `new DeviceScan(2, List.of(14, 11), 10, 15)`.
- Expected: `IllegalArgumentException`. As with `D11`, the
  exception's message is implementation-defined and is not
  pinned.

#### D6b — `record_constructor_rejects_duplicate_alive_ids`
- **New.** The "strictly ascending" invariant implies no
  duplicates, but `D6`'s `{14, 11}` input is both unsorted
  *and* duplicate-free, so it does not distinguish a strict
  (`<`) check from a non-strict (`<=`) check. An
  implementation that uses `<=` would accept a duplicated-ID
  input and produce a record violating the documented invariant.
- Bug class: non-strict ascending check.
- Inputs: `new DeviceScan(2, List.of(11, 11), 10, 15)`
  (sorted non-strictly; two equal elements).
- Expected: `IllegalArgumentException`.

#### D7 — `record_constructor_rejects_count_id_array_length_mismatch`
- Inputs: `new DeviceScan(2, List.of(11, 13, 14), 10, 15)`
  (3-element array, count=2).
- Expected: `IllegalArgumentException`.

#### D8 — `record_constructor_rejects_id_above_probe_range`
- Inputs: `new DeviceScan(1, List.of(20), 10, 15)`
  (id 20 is above range end 15).
- Expected: `IllegalArgumentException`.

#### D8b — `record_constructor_rejects_id_below_probe_range`
- **New.** The invariant says "every `id` in `aliveIds` is in
  `[probeRangeStart, probeRangeEnd]` inclusive" — that is two
  conditions. `D8` only verifies the upper bound; an
  implementation that checks `id <= probeRangeEnd` without
  `id >= probeRangeStart` would pass `D8` and silently accept a
  below-range ID. `D8b` closes this one-sided-bounds gap.
- Bug class: half-implemented range check (only one direction).
- Inputs: `new DeviceScan(1, List.of(9), 10, 15)` (id 9 is
  below range start 10).
- Expected: `IllegalArgumentException`.

#### D9 — `record_constructor_rejects_inverted_probe_range`
- Inputs: `new DeviceScan(0, List.of(), 15, 10)`
  (start > end).
- Expected: `IllegalArgumentException`.

#### D10 — `record_constructor_rejects_negative_probe_range_start`
- **New.** Pins the `probeRangeStart >= 0` constructor invariant
  directly (the factory-level `D5` only exercises this
  transitively). Without `D10`, a refactor that moves the
  validation from the constructor into the factory would
  silently accept direct constructor calls with negative starts,
  passing the suite while corrupting any caller that bypasses
  the factory.
- Inputs: `new DeviceScan(0, List.of(), -1, 5)`
  (negative start, otherwise consistent: count=0, empty array,
  start <= end).
- Expected: `IllegalArgumentException`.

#### D12 — `equal_field_values_yield_equal_records`
- **New.** Pins the structural-equality contract on the
  `DeviceScan` record. Java records derive `equals` and
  `hashCode` per-field via `Objects.equals`; for `int[]` this
  falls back to reference equality, which would break `Set`
  membership and any caller that compares scan results. The
  `aliveIds` field type is `List<Integer>` precisely so the
  derived equality is structural. Without this test, a future
  refactor that switches the field to `int[]` would silently
  break equality without any other test catching it.
- Bug class: a refactor that changes `aliveIds` from a
  list-typed field to an array-typed field, breaking structural
  equality.
- Inputs: two records constructed independently with identical
  field values:
  - `a = new DeviceScan(2, List.of(11, 14), 10, 15)`
  - `b = new DeviceScan(2, List.of(11, 14), 10, 15)`
  - `c = new DeviceScan(2, List.of(11, 13), 10, 15)` (one field
    differs).
- Expected:
  - `a.equals(b)` and `a.hashCode() == b.hashCode()`.
  - `!a.equals(c)`.

#### D11 — `from_probe_results_rejects_zero_length_array`
- **New.** Closes the boundary gap between `D2` (3-element
  all-false array) and the degenerate zero-length case. The
  factory rejects zero-length input directly — see "Suite-level
  concerns / Factory `DeviceScan.fromProbeResults` rejects
  zero-length probe arrays" for the rationale on rejecting at
  the factory boundary instead of letting the constructor's
  inverted-range error bubble up.
- Inputs: `aliveByIndex = new boolean[0]`, `probeRangeStart = 5`.
- Expected: `IllegalArgumentException`. The exception's message
  is implementation-defined and is not pinned by this test;
  asserting on message content would couple the test to internal
  formatting choices a future refactor may legitimately change.

> **Note on coverage choice.** `DeviceScan` is a defensive
> record; its constructor enforces the invariants that consumers
> rely on. `D6/D7/D8/D9/D10` test the constructor directly
> because they cover ways a future caller could bypass
> `fromProbeResults`. `D1/D2/D3` test the factory's correctness;
> `D4/D5/D11` test argument-handling. Together they pin the full
> type's contract.

### `BenchmarkPipelineTest`

#### P1''' — `pipeline_produces_documented_csv_for_synthetic_run`
- **Supersedes:** cycle-B `P1''`. Same wiring, more cells.
- Layer / contract: end-to-end Sequencer → Statistics →
  BenchmarkRecord → CsvWriter, 6×8=48 cells, 9-column rows.
- Bug class: all cycle-A/B `P1` bug classes (wiring, sort order,
  sequencer early termination), scaled to 8 operating points.
  New cycle-C bug class: a `WorkerSpec` migration regression
  that fails to spawn the correct number of workers for the new
  tiers — but this is HAL-bound and not caught at the unit
  level; `WorkerSpecTest` is the testable seam.
- Pinned `OperatingPoint` declaration order (cycle-C):
  `IDLE_BUS(0), SATURATED_BUS(1), MULTI_THREAD(2),
  SATURATED_MULTI_THREAD(3), SATURATED_BUS_4(4),
  SATURATED_BUS_8(5), SATURATED_BUS_4_MULTI_THREAD(6),
  SATURATED_BUS_8_MULTI_THREAD(7)`.
- Pinned `CallClass` declaration order: unchanged from v0.
- Inputs:
  - `Sequencer` configured with all 6 v0 call classes,
    `warmupCount = 1`, `perPointCount = 4`. Submission via
    round-robin discipline.
  - For each `(c, p)` cell, submit `[v - 1, v, v, v + 1]` ns
    where `v = ((6 * p.ordinal() + c.ordinal()) + 1) * 1000`.
    Cycle-B's `+1` offset preserved so the lowest cell stays
    positive. Cycle-C cell-index space is `[1, 48]`; sample
    values are `[1000, 48000]` ns range.
  - `Statistics.compute(samples, c.blockSize())` per cell.
  - `RunMetadata` fixture identical to cycle-B (and v0 `C1`).
- **Spot-check rows.** Three cells, one per blockSize bucket,
  one for each new cycle-C operating point family:

  **Row (HAL_GET_FPGA_TIME, SATURATED_BUS_4):** c=0, p=4,
  blockSize=100. Cell index = 6×4 + 0 + 1 = 25. v = 25000.
  - Block samples: `[24999, 25000, 25000, 25001]`.
  - mean = 25000 ns; per-call = 25000/100 = 250 ns = `0.250` µs.
  - stddev = sqrt(2/3)/100/1000 ≈ 0.0000082 µs → `%.3f` =
    `0.000`.
  - p50/p99/p999 all round to `0.250` (per-call differences are
    sub-0.001 µs at this blockSize).
  - outlier_rate = 0.
  - Expected row:
    `hal_get_fpga_time,saturated_bus_4,4,0.250,0.000,0.250,0.250,0.250,0.000000`.

  **Row (DS_STATE_READ, SATURATED_BUS_8):** c=2, p=5,
  blockSize=10. Cell index = 6×5 + 2 + 1 = 33. v = 33000.
  - Block samples: `[32999, 33000, 33000, 33001]`.
  - per-call mean = 33000/10 = 3300 ns = `3.300` µs.
  - stddev per-call ≈ 0.0000816 µs → `%.3f` = `0.000`.
  - p50/p99/p999 all round to `3.300`.
  - Expected row:
    `ds_state_read,saturated_bus_8,4,3.300,0.000,3.300,3.300,3.300,0.000000`.

  **Row (CAN_FRAME_WRITE, SATURATED_BUS_8_MULTI_THREAD):** c=5,
  p=7, blockSize=1. Cell index = 6×7 + 5 + 1 = 48. v = 48000.
  - Block samples: `[47999, 48000, 48000, 48001]`.
  - per-call mean = 48000 ns = `48.000` µs.
  - stddev per-call ≈ 0.000816 µs → `%.3f` = `0.001`.
  - p50 = 48000 → `48.000` µs.
  - p99 = 48000.97 ns → `48.001` µs.
  - p999 = 48000.997 ns → `48.001` µs.
  - outlier_rate = 0.
  - Expected row:
    `can_frame_write,saturated_bus_8_multi_thread,4,48.000,0.001,48.000,48.001,48.001,0.000000`.

- **Fixture generation procedure.** Identical to cycles A/B: run
  `P1'''` against the implemented Sequencer + CsvWriter, capture
  output, commit. Spot-check the three rows above before commit.
- **Cycle-A/B inheritance is partial.** The 24 cells from cycles
  A/B remain at cell indices `[1, 24]` (with the cycle-B `+1`
  offset preserved). The 24 new cycle-C cells are at indices
  `[25, 48]`. Diff against the cycle-B fixture should show
  exactly 24 added rows and zero modifications to the existing
  24. Implementer must verify this diff before commit.
- Expected: full output string from `CsvWriter.write(...)` is
  byte-equal to the regenerated 48-row fixture.
- Tolerance: byte-equal.
- Determinism: every input is a deterministic function of
  `(c, p, sample_index)`; no clocks; no random values; enum
  ordinals pinned.

## What this plan does NOT cover

- The HAL-bound device-scan (`bench.runner.CallBindings.probeTalonFxAlive`)
  — it calls real CTRE Phoenix6 API; the testable seam is
  `DeviceScan.fromProbeResults` which consumes the probe result.
- Real-RIO measurements at the new load tiers — picked up at
  hardware-validation time alongside cycle A/B.
- Hardware-driven calibration (real-CTRE-status-frame load vs
  software-spam load) — RB-cycle-D, post-cycle-C.
- Sweep-duration regression as op-point count grew from 3 (v0)
  to 4 (cycle A) to 8 (cycle C) — operator procedure now expects
  ~5 min instead of ~2 min; documented in the skill update.
- Sim-core loader / version-bump — deferred per `D-RB-8`.

### Pre-existing suite gap (acknowledged, not closed by cycle C)

- The unsorted-input gap from cycle B (no test submits an
  unsorted array to `Statistics.compute`) is unchanged.

## Skill updates required

Per CLAUDE.md, "Updating the skill is part of every feature
change." Cycle C touches enough surface that the
`.claude/skills/rio-bench.md` update is non-trivial. The
implementer must, in the same commit set as the cycle-C code:

1. **Operating points table.** Grow from 4 entries (cycle A) to
   8: the four cycle-A points plus `SATURATED_BUS_4`,
   `SATURATED_BUS_8`, `SATURATED_BUS_4_MULTI_THREAD`,
   `SATURATED_BUS_8_MULTI_THREAD`. Document each row's
   `(cpuWorkers, canSpamWorkers)` mapping.
2. **`WorkerSpec` field type change.** Update prose that names
   the field `canSpam` (boolean) to `canSpamWorkers` (int). The
   suite-level invariant "no `default` branch in
   `WorkerSpec.forPoint`" is unchanged; the per-arm body grows
   from 4 cases to 8.
3. **CSV row sort order.** The 8-entry lex-sorted list (per
   `O3'`) replaces cycle-A's 4-entry list. `D-RB-5`
   (deterministic row order) prose stays; only the example
   list grows.
4. **Operator procedure / sweep duration.** v0/cycle-A/cycle-B
   ran ~2 min for 24 cells; cycle C runs ~5 min for 48 cells.
   Update the "Operator procedure" section's sweep-duration
   estimate.
5. **`bus_devices` semantics.** Cycle A made the line
   informational (no longer gated `validated=false`). Cycle C
   guarantees the runner always populates it via the runtime
   scan. Update the "Operating points" preamble paragraph and
   the "Known gotchas" entry.
6. **New `bench.runner.DeviceScan` component** in the file
   layout block + a one-paragraph description in "Public
   surface" or a new subsection under "Design decisions" if
   `D-RB-9` is added (see #7).
7. **New `D-RB-9` design decision (recommended, not strictly
   required for this plan).** "Bus device count is recorded by
   runtime scan, not operator config." Cite the rationale
   (operator-config drift is the #1 cause of bad bench data;
   automatic scan removes the gap between claimed and actual
   bus state). One follow-up: cycle C's scan only covers
   TalonFX devices in a configurable ID range; a future cycle
   may extend to other CTRE/REV device types.
8. **Validation status.** Bump test count from 54 to 76
   (4 new W-tests + 4 new Q-tests + 14 new D-tests = 22 new;
   54 + 22 = 76). Note: `DeviceScanTest` is a new test class,
   so every test in it counts as new (D1-D11 + D6b + D8b + D12
   = 14). Rewrites in place do not change the count. Document
   cycle C as a "ready-to-implement" plan with five review
   rounds.
9. **Open follow-ups.** Add: hardware-driven calibration cycle
   (RB-cycle-D) comparing software-spam to real-CTRE-status-frame
   load.

This list is exhaustive; the implementer should not add or omit
items without re-running the test-reviewer.

## Determinism summary

- All numerical inputs are constants.
- No test reads the wall clock, network, or filesystem (the
  fixture is loaded via classpath; the device-scan tests use
  hand-constructed `boolean[]` inputs, never a real HAL probe).
- Run order independence: any subset of tests can run in any
  order without affecting outcomes.

## Citations

- Inherited from cycles A/B: Hyndman & Fan 1996 (R-7);
  Wikipedia sample-stddev worked example; Java `Formatter` `%f`
  HALF_UP.
- New: none required — cycle C is a sweep-extension cycle, not a
  numerical-method cycle.
