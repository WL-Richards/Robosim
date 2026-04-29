# HAL shim core — cycle 3 test plan (ds_state)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (two review rounds, 2026-04-29) with the rev-3
documentation correction applied. Cycle-3 production code and
tests landed and 29-shim-test green under both clang Debug and
GCC Debug + ASan + UBSan. One implementation-time correction
applied to the C3-R10 plan text (the `can_frame_batch` payload
must be a 4-byte count=0 header, not the full struct, because
the validator computes expected size from the variable-length
count); the correction is documented inline in the C3-R10
"Mechanical changes" bullet 1 above.

**Review history:**

- rev 1 → `not-ready`. Two blockers:
  - **C3-5 has_value() gating unspecified.** The five-step
    interleaved scenario describes per-step assertions that
    dereference `latest_clock_state()`, `latest_power_state()`,
    `latest_ds_state()` at multiple steps where one or more of
    those slots is expected to already be populated. The plan
    didn't specify whether the prerequisite `has_value()` checks
    at those steps should be `ASSERT_TRUE` or `EXPECT_TRUE`.
    With `EXPECT_TRUE`, a silent prior-step regression would
    leave the slot null and the subsequent dereference would be
    UB rather than a clean abort, *or* the "unchanged"
    assertion would pass trivially (still null = "still not
    changed"), producing a false green. Same no-shortcuts
    rationale that pinned C2-3b/c and C3-3b/c/d.
  - **C3-6 padding-byte determinism unpinned.** The plan
    argued that the zero-init fixture discipline plus byte-
    deterministic `std::memcpy` was enough to inherit the
    padding-byte determinism guarantee from the existing
    fixtures. The reviewer correctly noted that
    `ds_state::operator==` is field-by-field equality (the
    defaulted `operator==` recurses through named members; the
    C++ standard does not require it to compare padding bytes,
    and GCC/Clang do not generate padding-byte comparisons from
    a defaulted `operator==`). `ds_state` carries five interior
    padding bytes (2 at offsets 2226–2227 between
    `joystick_descriptors[5]` and `control`; 1 at
    `match_info`+135; 2 at `match_info`+138–139). A future
    refactor that constructs the cache entry through a non-zero-
    init path would leave garbage in those bytes, producing
    byte-non-identical logs even though `operator==` reads as
    equal — violating CLAUDE.md non-negotiable #5
    ("byte-identical logs"). The determinism test is the right
    place to pin this, and cycle 3 is the first cycle where it
    matters.
  - Plus one non-blocker documentation issue: the
    `valid_ds_state` rationale claimed a partial copy that
    "drops a prefix, suffix, or interior region" would fail
    byte-equality. The interior claim is technically false: a
    contrived bug that copies only the first 8 bytes and the
    last 156 bytes (skipping the 2220-byte zero-init interior
    between `joystick_axes_[0].axes[0]` and `control`) would
    not be caught because every byte in that interior is zero
    in both source and destination. The realistic
    "copies only N bytes" bug *is* caught for every practical
    value of N because every prefix and every suffix touches a
    sentinel field, but the prose was imprecise. Reviewer
    flagged it as documentation correctness, not a coverage
    gap requiring a new test.

- rev 2 → this revision. All rev-1 issues addressed:
  - **C3-5 per-step `has_value()` checks pinned to
    `ASSERT_TRUE`** before every dereference, with inline
    rationale citing the false-green / UB-on-null failure mode
    and the cycle-2/cycle-3 prior-art (C2-3b, C3-3b/c/d).
  - **C3-6 gains a `std::memcmp` assertion** on the
    `latest_ds_state()` comparison, immediately after the
    `operator==` assertion. Inline comment explains that
    `operator==` does not pin padding bytes and that the
    `memcmp` is the explicit byte-identical guarantee per
    CLAUDE.md non-negotiable #5 + D-C3-7. The clock_state and
    power_state slots do not need a `memcmp` companion (no
    padding bytes in either struct: `sizeof(clock_state) ==
    sizeof of fields packed` per its definition;
    `sizeof(power_state) == 12 == 3 × sizeof(float)`).
  - **Fixture rationale corrected** to "drops a prefix or
    suffix" rather than "drops a prefix, suffix, or interior
    region." A clarifying note explains that any realistic
    partial-copy bug touches at least one sentinel because the
    sentinels span offsets 0..2376 and a contrived
    interior-skip is not a realistic memcpy implementation.

**Implements:** the third TDD cycle of the HAL shim core, named in
`.claude/skills/hal-shim.md` "Out / Cycle 3+" and in
`.claude/skills/layer-2-control-system-backend.md` "HAL shim cycles
3+: additional inbound schemas (`ds_state` next, ...)".

This document is an **addendum** to the cycle-1 + cycle-2 plans
(`tests/backend/shim/TEST_PLAN.md` rev 2 ready-to-implement and
landed; `tests/backend/shim/TEST_PLAN_CYCLE2.md` rev 4
ready-to-implement and landed). Cycle 3 adds new contracts on top
of the cycle-2 surface, does not replace it. The cycle-1 + cycle-2
20 tests must continue to pass without behavior change; **one
cycle-1 test (test 10) is rewritten again** to migrate its probe
schema from `ds_state` (now supported) to `can_frame_batch` (the
next still-unsupported schema by `schema_id` numeric order).

---

## Why this cycle exists

Cycle 2 left the shim with two supported inbound state schemas
(`clock_state`, `power_state`) and one terminal-flag schema set
(`shutdown` / `boot_ack` / `none`). Every other inbound schema
under `tick_boundary` falls through to
`unsupported_payload_schema`. Cycle 3 promotes `ds_state` from
"rejected" to "accepted with its own latest-wins cache slot",
mirroring `clock_state` and `power_state`'s shape but with two
new wrinkles relative to cycle 2:

1. **The ds_state schema is large** (`sizeof(ds_state) == 2384`,
   `alignof == 8`) and contains nested aggregates (joystick arrays,
   match info, control word, alliance station). Cycle-2's
   `power_state` was a flat 12-byte three-`float` POD; the cycle-3
   byte-copy path stresses payload-size and alignment correctness
   in a way that cycle 2 did not.
2. **Three independent cache slots, not two.** The cross-slot
   independence test set must cover all four cross-population bug
   classes that arise once a third slot exists: ds-arm clobbers
   clock, ds-arm clobbers power, clock-arm clobbers ds, power-arm
   clobbers ds. C3-5 (the cycle-3 analogue of C2-5) is therefore a
   five-step interleaved scenario, not three.

The smallest cohesive feature is:

- **Accept** a `tick_boundary` envelope carrying `schema_id::ds_state`
  with `payload_bytes == sizeof(ds_state) == 2384` and byte-copy
  the payload into a new `latest_ds_state_` cache slot.
- **Expose** the new cache slot via
  `const std::optional<ds_state>& latest_ds_state() const` on
  `shim_core` (mirroring the reference-to-optional shape of
  `latest_clock_state()` and `latest_power_state()`).
- **Preserve** every existing cycle-1/2 contract verbatim.

The post-rejection counter recovery contract (cycle-1 test 10,
already rewritten in cycle 2 to probe `ds_state`) needs one more
mechanical migration: cycle 3 makes `ds_state` valid, so the
"still-unsupported probe" must shift to `can_frame_batch` (next by
`schema_id` enum value).

---

## Contract under test (additions only)

System under test is unchanged: `robosim::backend::shim::shim_core`.

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<ds_state>& latest_ds_state() const` —
  `nullopt` until at least one inbound `tick_boundary` envelope
  carrying `schema_id::ds_state` has been accepted; the most
  recent `ds_state` byte-copy thereafter. Latest-wins, no queue.
  Reference-to-optional shape mirrors `latest_clock_state()` and
  `latest_power_state()`.

### New error kinds

**None.** The fall-through dispatch
(`unsupported_payload_schema`, `unsupported_envelope_kind`) is
unchanged. No new `shim_error_kind` is introduced.

### Internal additions

- New private member `std::optional<ds_state> latest_ds_state_;`
  on `shim_core`, sibling to `latest_clock_state_` and
  `latest_power_state_`.
- New `case schema_id::ds_state:` arm in the `tick_boundary`
  dispatch in `shim_core::poll()`: byte-copy the
  `sizeof(ds_state)` bytes into `latest_ds_state_` and return
  success. Symmetric to the existing `clock_state` and
  `power_state` arms.

### Out of scope for this cycle

- Inbound schemas other than `ds_state` and the cycle-1+2 set.
  `can_frame_batch` is cycle 4; `can_status` is cycle 5; etc.
- Outbound `ds_state` — the shim is purely consumer.
- C HAL ABI surfaces (`HAL_GetControlWord`,
  `HAL_GetMatchInfo`, joystick query functions, etc.) that read
  from `latest_ds_state_`. Cycle 4+.
- Validity-domain checks on `ds_state` field values (e.g.
  rejecting an out-of-range `alliance_station`). The shim is a
  byte-copy cache; the producer (sim core) and consumers (future
  HAL surfaces) own field validation.

---

## Decisions pinned

- **D-C3-1.** `clock_state`, `power_state`, and `ds_state` caches
  are **independent storage**. None mutates any other. Pinned by
  C3-5 (cross-slot independence over the four new direction-pair
  bug classes). Inherits and extends D-C2-1.

  **Why this is a decision, not an obvious code shape:** a refactor
  that "deduplicates" the three slots into a single
  `std::variant<clock_state, power_state, ds_state>` (or a
  schema-tagged single slot, "we only need the latest *anything*")
  passes most other tests. C3-5 is the regression check.

- **D-C3-2.** Latest-wins, no queue. Inherits cycle-2 D-C2-2.

- **D-C3-3.** Schema-specific dispatch is **strictly post-session**.
  A pre-boot-ack `tick_boundary`/`ds_state` surfaces
  `expected_boot_ack_first` from the session, **not**
  `unsupported_payload_schema` from the dispatch and **not** a
  cache write. Pinned by C3-4. Inherits D-C2-3.

- **D-C3-4.** **Cycle-1 test 10's probe schema migrates from
  `ds_state` to `can_frame_batch`.** The migration is mechanical
  per the source-of-truth note in cycle-2 D-C2-4: "each cycle that
  adds a schema must rewrite this test to migrate the probe one
  schema along the cycle-3..cycle-N order
  (`ds_state` → `can_frame_batch` → `can_status` → … →
  `error_message_batch`)." `can_frame_batch` is the next
  per-tick-allowed schema by `schema_id` numeric order
  (`schema_id::can_frame_batch == 4`).

- **D-C3-5.** **No new `shim_error_kind`.** Inherits D-C2-5.

- **D-C3-6.** `latest_ds_state_` is **value-initialized as
  `std::nullopt`** at the same point in `shim_core`'s default-
  member-init that `latest_clock_state_` and `latest_power_state_`
  are. Pinned by C3-3a. Inherits the spirit of D-C2-6.

- **D-C3-7.** **Padding bytes in `ds_state` are part of the
  byte-copy contract.** `ds_state` has interior padding (the
  cycle-1 protocol-schema TEST_PLAN H8 traces it: 2 bytes between
  `joystick_descriptors[5]` and `control` due to alignof-4 on
  `control_word`; 1 byte interior pad inside `match_info` between
  `game_specific_message[63]` and `game_specific_message_size`;
  2 trailing bytes in `match_info` for alignof-4 from `match_type`
  resync). The shim's `std::memcpy` preserves padding bytes
  verbatim. **The test fixtures construct `ds_state{}`-zero-init
  values and only mutate explicit fields**, so padding bytes are
  zero in both source and round-tripped form, and the byte-equal
  assertions hold against `std::memcmp` and `operator==`. A
  fixture that wrote padding bytes non-deterministically (e.g. via
  `std::memset(&state, 0xCC, sizeof(state))` followed by partial
  field assignment) would expose padding non-determinism in
  `operator==`'s default field-by-field path; that variant is
  explicitly out of scope, and no cycle-3 test does that.

  This decision exists separately from D-C3-1/D-C3-2 because the
  bug class it guards (a refactor that copies fewer than
  `sizeof(ds_state)` bytes, e.g. one that skips padding "to be
  efficient") is unique to large-with-padding schemas and did not
  arise in cycle 2.

---

## Cross-cutting assertion shape (extends cycle 1 + 2)

Cycle-3 tests live in the same `ShimCoreMake` / `ShimCoreObservers`
/ `ShimCorePoll` / `ShimCoreDeterminism` GoogleTest suites as
cycle-1/2 tests, in `tests/backend/shim/shim_core_test.cpp`.
Naming and the "one assert per behavior" convention carry over.

Per cycle-1+2's pattern, all payload comparisons use either the
defaulted `ds_state::operator==` (full-struct equality, recursing
through every nested `joystick_axes` / `joystick_buttons` /
`joystick_povs` / `joystick_descriptor` / `match_info` /
`control_word`) or `std::memcmp(&a, &b, sizeof(ds_state)) == 0`
over the 2384 bytes. Tests do **not** field-by-field compare
individual struct members.

`ds_state::operator==` is bit-equal-as-`==` for every value the
tests produce because (a) all `float` and `double` fields are
exact constructor-supplied constants (no NaN, no rounding-residue,
no signed-zero ambiguity), (b) integer fields are byte-equal under
`==`, (c) array fields use `std::array::operator==` element-wise
which inherits the same properties. Padding bytes are zero in
fixture-constructed values per D-C3-7 above and survive the
`std::memcpy` round-trip byte-equally.

---

## Test fixtures (additions only)

Cycle 3 adds one inline helper to
`tests/backend/tier1/test_helpers.h`:

```cpp
inline ds_state valid_ds_state(std::int16_t joystick0_axis_count = 3,
                                float joystick0_axis_0_value     = 0.5f,
                                std::uint32_t control_bits       = kControlEnabled | kControlDsAttached,
                                alliance_station station         = alliance_station::red_2,
                                match_type type                  = match_type::qualification,
                                std::uint16_t match_number       = 42,
                                double match_time_seconds        = 12.5) {
  ds_state state{};  // zero-init — covers all padding bytes (D-C3-7)
                     // plus the five other-joystick slots, the
                     // event_name/game_specific_message arrays, etc.
  state.joystick_axes_[0].count = joystick0_axis_count;
  state.joystick_axes_[0].axes[0] = joystick0_axis_0_value;
  state.control.bits = control_bits;
  state.station = station;
  state.match.type = type;
  state.match.match_number = match_number;
  state.match_time_seconds = match_time_seconds;
  return state;
}
```

**Rationale for the touched fields:** the seven distinguished
fields span the full `ds_state` layout — joystick arrays at the
front (offset 0), control word in the middle (offset 2228 per
TEST_PLAN H8), alliance station at offset 2232, match_info
contents in the middle (offsets 2236+), and `match_time_seconds`
at the very end (offset 2376). A byte-copy bug that drops a
**prefix or suffix** of any practical length fails byte-equality
on at least one distinguished field, because every nonempty
prefix or suffix of the 2384-byte struct contains at least one
sentinel-bearing offset.

**Note on contrived interior-skip bugs:** a non-realistic bug
that copies the first 8 bytes plus the last 156 bytes (skipping
the 2220-byte zero-init interior between
`joystick_axes_[0].axes[0]` at offset 4 and `control` at offset
2228) would *not* be caught by these sentinels — every byte in
that interior is zero in both source and destination. Such a bug
is not a realistic `std::memcpy` implementation and the plan
does not target it. If a future refactor introduces a hand-
rolled copy loop that skips a region, that refactor lands with
its own positive test exercising the affected offsets.

The defaults are realistic FRC operating-point values (a Red-2
alliance station, qualification match #42, 12.5 seconds elapsed,
robot enabled and DS attached, joystick 0 reporting 3 axes with
axis 0 at +0.5).

**`ds_state{}` zero-init is intentional and load-bearing per
D-C3-7.** The aggregate-init zeroes every byte, including all
padding. Subsequent per-field assignment leaves padding zero. The
resulting in-memory layout is byte-identical across runs and
across compile configurations (clang Debug vs. GCC Debug + ASan +
UBSan, both pinned by C3-6).

**Inline comment to add at the helper definition:** a one-line
note that consumers compare via `ds_state::operator==`
(bit-equal-as-`==` on all float/double fields for the constants
this fixture produces; recursion through nested aggregates uses
the same mechanism), not `std::memcmp`, **except where a test
explicitly wants to assert padding-byte equality** (none in
cycle 3, but C3-6's determinism test could in principle drop
to `std::memcmp` if a future regression demands it; for now
`operator==` plus the zero-padding-byte invariant is sufficient).

**Implementer checklist:** adding `valid_ds_state` to
`test_helpers.h` requires that `#include "ds_state.h"` is already
present in that header. As of cycle-2 landing, `test_helpers.h`
includes `boot_descriptor.h`, `clock_state.h`, `power_state.h`,
`protocol_version.h`, `shared_memory_transport.h`, `sync_envelope.h`
— **`ds_state.h` is not yet included**. The cycle-3 implementer
must add `#include "ds_state.h"` in alphabetical order between
`clock_state.h` and `power_state.h`.

---

## Determinism notes

Single-threaded same-process tests. No new sources of
nondeterminism. C3-6 explicitly re-asserts byte-identical output
across two runs of an expanded scenario that includes
`clock_state`, `power_state`, *and* `ds_state` updates.

Padding-byte determinism on `ds_state` is pinned **directly** by
C3-6 via an explicit `std::memcmp(&*setup_a.latest_ds_state(),
&*setup_b.latest_ds_state(), sizeof(ds_state)) == 0` assertion,
in addition to the `operator==` assertion. `operator==` is
field-by-field equality and the C++ standard does not require it
to compare padding bytes (and GCC/Clang in practice do not), so
the `memcmp` is required to enforce the byte-identical
guarantee that CLAUDE.md non-negotiable #5 demands. The fixture's
zero-init pattern (`ds_state{}` followed by per-field
assignment) produces a byte-identical source struct on both
runs — every padding byte is zero — and `std::memcpy` is byte-
deterministic, so the post-cache struct is byte-identical and
the `memcmp` passes against a correct implementation. A future
regression that writes garbage into `ds_state{}`'s padding (e.g.
a refactor that constructs the cache value via some non-zero-
init path) would fail C3-6's `memcmp` assertion directly, even
though `operator==` would still pass.

`clock_state` and `power_state` do not need a parallel `memcmp`
companion: neither struct has padding bytes (`power_state` is
three packed `float`s with `sizeof == 12 == 3 × sizeof(float)`;
`clock_state` is a `uint64_t` followed by four `uint32_t`-shaped
booleans plus a `uint32_t` counter, all naturally aligned with
no interior or trailing pad). For those two slots, `operator==`
is byte-equal-to-`memcmp`, so the existing `operator==`
assertion in C3-6 is sufficient.

---

## Proposed tests (revision 3)

### C3-1. `ShimCorePoll.AcceptsDsStateAndCachesByteEqualValue`

- **Layer / contract:** Layer 2 HAL shim inbound dispatch;
  `latest_ds_state()` public accessor; D-C3-1 (cross-slot
  independence: at this single-arrival step, clock and power
  slots stay nullopt).
- **Bug class caught:** payload byte corruption in transit;
  shim writes `ds_state` payload into a non-`ds_state` slot;
  partial copy that drops a prefix, suffix, or interior region
  of the 2384-byte struct; `payload_bytes` mismatch silently
  truncates.
- **Inputs:**
  - Shim connected via `make_connected_shim` (boot + boot_ack
    consumed; expected receive sequence is now 1).
  - Core endpoint sends `tick_boundary`/`ds_state` at sequence 1
    with payload `valid_ds_state()` (default args) and
    `sim_time_us = 1'000`.
- **Expected outputs after `poll()`:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_ds_state().has_value())`.
  - `*shim.latest_ds_state() == valid_ds_state()` using the
    defaulted `operator==` (full-struct equality through every
    nested aggregate).
  - `region.core_to_backend.state == empty`.
  - `shim.latest_clock_state() == std::nullopt` (D-C3-1: clock
    slot untouched).
  - `shim.latest_power_state() == std::nullopt` (D-C3-1: power
    slot untouched).
- **Tolerance:** zero. No arithmetic on any field; copy is
  byte-exact.

### C3-2. `ShimCorePoll.LatestWinsForRepeatedDsStateUpdates`

- **Layer / contract:** D-C3-2 — cache replacement keeps only
  the most recent `ds_state`.
- **Bug class caught:** cache holds first value forever;
  field-wise update instead of struct-wise replace; shim
  partially-overwrites only fields that overlap with the second
  arrival.
- **Inputs:** shim connected. Two `tick_boundary`/`ds_state`
  envelopes via the core endpoint:
  - First at sequence 1: `valid_ds_state(/*joystick0_axis_count=*/2,
    /*joystick0_axis_0_value=*/0.25f, /*control_bits=*/kControlAutonomous,
    /*station=*/alliance_station::blue_1,
    /*type=*/match_type::practice,
    /*match_number=*/7,
    /*match_time_seconds=*/3.0)`.
  - Second at sequence 2: `valid_ds_state(/*joystick0_axis_count=*/4,
    /*joystick0_axis_0_value=*/-0.75f,
    /*control_bits=*/kControlEnabled | kControlTest,
    /*station=*/alliance_station::red_3,
    /*type=*/match_type::elimination,
    /*match_number=*/15,
    /*match_time_seconds=*/45.0)`.
  Every distinguished field differs between the two values.
- **Expected outputs:**
  - After first poll: `*shim.latest_ds_state()` equals the first
    fixture (full-struct equality).
  - After second poll: `*shim.latest_ds_state()` equals the second
    fixture (full-struct equality).
- **Bug class:** cache holds the first value forever; cache does
  field-wise update instead of struct-wise replace; cache only
  updates the first kilobyte of the struct (a "shim copies a
  prefix" bug).

### C3-3a. `ShimCoreObservers.DsStateNulloptBeforeAnyPoll`

- **Layer / contract:** D-C3-6 — `latest_ds_state_` value-
  initialized to `std::nullopt` at construction.
- **Suite placement:** `ShimCoreObservers`, the same suite as
  cycle-1 test 4 and cycle-2 C2-3a. Same precedent: tests post-
  `make` pre-`poll()` observer state without driving `poll()`.
- **Inputs:** shim post-`make` (boot already published); no
  `poll()` yet.
- **Expected outputs:**
  - `shim.latest_ds_state() == std::nullopt`.
- **Bug class:** `latest_ds_state_` default-init drifts to
  `ds_state{}` instead of `nullopt`; member ordering puts
  `latest_ds_state_` ahead of construction-time init.

### C3-3b. `ShimCorePoll.DsStateNulloptAfterBootAckIsAccepted`

- **Layer / contract:** D-C3-6 — `boot_ack` dispatch arm does
  not contaminate the ds-state cache slot.
- **Inputs:**
  - Shim post-`make`.
  - Core endpoint sends `boot_ack` (kind `boot_ack`, schema
    `none`, payload bytes 0, sequence 0).
  - Single `poll()`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`. **Required `ASSERT_TRUE`
    (not `EXPECT_TRUE`)** — pinned per cycle-2's convention; if
    `poll()` returned an error there is no point continuing.
  - `ASSERT_TRUE(shim.is_connected())`. **Required `ASSERT_TRUE`**
    — prerequisite check; if `boot_ack` processing silently
    failed, the primary assertion below would pass trivially
    (the dispatch arm was never reached), producing a false
    green. Same no-shortcuts (CLAUDE.md non-negotiable #1)
    rationale as C2-3b.
  - `shim.latest_ds_state() == std::nullopt`. (Primary
    assertion.)
- **Bug class:** `boot_ack` dispatch arm accidentally writes
  zero-init `ds_state` into the cache; dispatch table has a
  fall-through that populates ds on any accepted envelope.

### C3-3c. `ShimCorePoll.DsStateNulloptAfterClockStateIsAccepted`

- **Layer / contract:** D-C3-1 — independent storage, viewed
  from "the ds slot stays empty under unrelated `clock_state`
  traffic." Distinct bug class from C3-3d (`power_state`
  contamination); each dispatch arm is a different function in
  the table and can fail independently.
- **Inputs:**
  - Shim connected via `make_connected_shim`.
  - Core endpoint sends `tick_boundary`/`clock_state` at
    sequence 1 with `valid_clock_state(50'000)`.
  - Single `poll()`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_clock_state().has_value())`.
    Prerequisite — if `clock_state` processing silently failed,
    no traffic ever exercised cross-population to the ds slot
    and the primary assertion would pass trivially.
  - `shim.latest_ds_state() == std::nullopt`. (Primary
    assertion.)
- **Bug class:** `clock_state` dispatch arm accidentally cross-
  populates the ds slot (e.g. via a copy-paste error from the
  cycle-3 ds_state arm back into clock_state code); a shared
  union-typed cache that gets schema-tagged as `ds_state` after
  a `clock_state` arrival.

### C3-3d. `ShimCorePoll.DsStateNulloptAfterPowerStateIsAccepted`

- **Layer / contract:** D-C3-1 — independent storage, viewed
  from "the ds slot stays empty under unrelated `power_state`
  traffic." Distinct bug class from C3-3c.
- **Inputs:**
  - Shim connected via `make_connected_shim`.
  - Core endpoint sends `tick_boundary`/`power_state` at
    sequence 1 with `valid_power_state()`.
  - Single `poll()`.
- **Expected outputs:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_power_state().has_value())`.
    Prerequisite — same false-green rationale.
  - `shim.latest_ds_state() == std::nullopt`. (Primary
    assertion.)
- **Bug class:** `power_state` dispatch arm accidentally cross-
  populates the ds slot; a refactor that mistakenly routes
  `power_state` payload through the new ds path.

### C3-4. `ShimCorePoll.RejectsDsStateBeforeBootAckPreservingLane`

- **Layer / contract:** D-C3-3 — schema-specific dispatch is
  post-session. A pre-boot-ack
  `tick_boundary`/`ds_state` surfaces `expected_boot_ack_first`
  from the session, **not** `unsupported_payload_schema` from
  the dispatch.
- **Setup (mirrors cycle-1 test 9 and cycle-2 C2-4, schema
  swapped):**
  - Fresh region; backend endpoint; shim via `make` (boot
    published).
  - Core peer is **not** used to send `boot_ack`. Instead, the
    test uses `manually_fill_lane` to inject a well-framed
    envelope directly into `region.core_to_backend`:
    - `kind = tick_boundary`,
    - `payload_schema = ds_state`,
    - `payload_bytes = sizeof(ds_state) == 2384`,
    - `sequence = 0`,
    - `sender = core_to_backend`,
    - `sim_time_us = 100'000`,
    - payload bytes = `bytes_of(valid_ds_state())`.
  - **Why direct injection:** no peer can legitimately produce
    this state. A core-side `protocol_session` would refuse to
    build a `tick_boundary` before sending `boot_ack`. Same
    rationale as cycle-1 test 9 / cycle-2 C2-4.
- **Expected outputs:**
  - `poll()` returns
    `shim_error{shim_error_kind::receive_failed, transport_error =
    tier1_transport_error{kind = session_rejected_envelope,
    session_error = {kind = expected_boot_ack_first, ...}}, ...}`.
  - `region.core_to_backend.state == full` (lane preserved per
    tier1 failure-atomicity).
  - `region.core_to_backend.envelope` and `.payload_bytes` and
    `.payload` byte-equal the injected values (no clobber).
  - `shim.is_connected() == false`.
  - `shim.latest_clock_state() == std::nullopt`.
  - `shim.latest_power_state() == std::nullopt`.
  - `shim.latest_ds_state() == std::nullopt`.
- **Bug class:** schema-specific dispatch leaks before the
  session check (would surface `unsupported_payload_schema` or,
  worse, silently populate the cache); shim accepts ds_state
  ahead of the protocol agreement.

### C3-5. `ShimCorePoll.ClockPowerAndDsCachesAreIndependentlyMaintained`

- **Layer / contract:** D-C3-1 — independent cache storage
  across all three slots. The cycle-3 generalization of cycle-2
  C2-5 needs to cover four cross-population bug classes (each
  of the two non-SUT dispatch arms could potentially clobber any
  of the two non-self slots, and vice versa for the new ds_state
  arm — see "Why this cycle exists" item 2).
- **Setup:** shim connected via `make_connected_shim`. A
  five-step sequence interleaves all three schemas, asserting
  on all three slots at every step. Each step's `sim_time_us`
  is distinct so a "shim caches by reference and the source
  changes" bug surfaces.
  - **Step 1: clock_state arrival.** Core sends
    `tick_boundary`/`clock_state` at sequence 1 with
    `valid_clock_state(100'000)`.
  - **Step 2: power_state arrival.** Core sends
    `tick_boundary`/`power_state` at sequence 2 with
    `valid_power_state(12.5f, 2.0f, 6.8f)`.
  - **Step 3: ds_state arrival.** Core sends
    `tick_boundary`/`ds_state` at sequence 3 with
    `valid_ds_state()` (defaults).
  - **Step 4: clock_state arrival again.** Core sends
    `tick_boundary`/`clock_state` at sequence 4 with
    `valid_clock_state(200'000)`.
  - **Step 5: power_state arrival again.** Core sends
    `tick_boundary`/`power_state` at sequence 5 with
    `valid_power_state(13.5f, 5.0f, 6.5f)`.
- **Per-step assertion convention.** All `has_value()`
  prerequisite checks before any dereference of
  `latest_clock_state()`, `latest_power_state()`, or
  `latest_ds_state()` use **`ASSERT_TRUE`, not `EXPECT_TRUE`**.
  Same no-shortcuts (CLAUDE.md non-negotiable #1) rationale as
  C3-3b/c/d: a silent prior-step regression that left a slot
  null would either UB on dereference or trivially pass an
  "unchanged" assertion (still null = "still nullopt"),
  producing a false green. `ASSERT_TRUE` aborts and surfaces
  the real failure at the step where the regression first
  manifested. Each `poll()` itself is also gated with
  `ASSERT_TRUE(poll().has_value())`.
- **Expected outputs (after each `poll()`):**
  - **Step 1.** `ASSERT_TRUE(poll().has_value())`.
    `ASSERT_TRUE(shim.latest_clock_state().has_value())` —
    primary first-arrival check. Then
    `EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u)`.
    `EXPECT_FALSE(shim.latest_power_state().has_value())`.
    `EXPECT_FALSE(shim.latest_ds_state().has_value())`.
  - **Step 2.** `ASSERT_TRUE(poll().has_value())`.
    `ASSERT_TRUE(shim.latest_clock_state().has_value())` —
    prerequisite for the unchanged-clock dereference; aborts
    if step 1's clock write was silently dropped.
    `EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u)`
    (unchanged from step 1; *catches "power-arm clobbers
    clock-slot"*).
    `ASSERT_TRUE(shim.latest_power_state().has_value())` —
    primary new-arrival check.
    `EXPECT_EQ(*shim.latest_power_state(),
    valid_power_state(12.5f, 2.0f, 6.8f))`.
    `EXPECT_FALSE(shim.latest_ds_state().has_value())`.
  - **Step 3.** `ASSERT_TRUE(poll().has_value())`.
    `ASSERT_TRUE(shim.latest_clock_state().has_value())`.
    `EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 100'000u)`
    (unchanged; *catches "ds-arm clobbers clock-slot"*,
    D-C3-1).
    `ASSERT_TRUE(shim.latest_power_state().has_value())`.
    `EXPECT_EQ(*shim.latest_power_state(),
    valid_power_state(12.5f, 2.0f, 6.8f))` (unchanged;
    *catches "ds-arm clobbers power-slot"*, D-C3-1).
    `ASSERT_TRUE(shim.latest_ds_state().has_value())` —
    primary new-arrival check.
    `EXPECT_EQ(*shim.latest_ds_state(), valid_ds_state())`
    (full-struct equality).
  - **Step 4.** `ASSERT_TRUE(poll().has_value())`.
    `ASSERT_TRUE(shim.latest_clock_state().has_value())`.
    `EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u)`
    (updated; sanity check that the second clock arrival
    actually wrote).
    `ASSERT_TRUE(shim.latest_power_state().has_value())`.
    `EXPECT_EQ(*shim.latest_power_state(),
    valid_power_state(12.5f, 2.0f, 6.8f))` (unchanged).
    `ASSERT_TRUE(shim.latest_ds_state().has_value())`.
    `EXPECT_EQ(*shim.latest_ds_state(), valid_ds_state())`
    (unchanged; *catches "clock-arm clobbers ds-slot"*,
    D-C3-1).
  - **Step 5.** `ASSERT_TRUE(poll().has_value())`.
    `ASSERT_TRUE(shim.latest_clock_state().has_value())`.
    `EXPECT_EQ(shim.latest_clock_state()->sim_time_us, 200'000u)`
    (unchanged from step 4).
    `ASSERT_TRUE(shim.latest_power_state().has_value())`.
    `EXPECT_EQ(*shim.latest_power_state(),
    valid_power_state(13.5f, 5.0f, 6.5f))` (updated; sanity
    check on the second power arrival).
    `ASSERT_TRUE(shim.latest_ds_state().has_value())`.
    `EXPECT_EQ(*shim.latest_ds_state(), valid_ds_state())`
    (unchanged; *catches "power-arm clobbers ds-slot"*,
    D-C3-1).
- **Bug class:** any cross-slot dispatch error in any of the
  four directions enumerated above; shared single slot tagged
  by schema; union-typed cache; misread `schema_id` in the
  dispatch table.
- **Tolerance:** zero. All values are explicit constants.

### C3-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalLatestClockPowerAndDsState`

- **Layer / contract:** CLAUDE.md non-negotiable #5. Replaces
  cycle-2's
  `RepeatedRunsProduceByteIdenticalLatestPowerAndClockState`
  (C2-6) with a strict superset that also exercises the
  ds_state path. C2-6 is **dropped from the file** as part of
  this cycle (see "Test rewrites" below); same "no half-
  finished implementations" rationale that cycle 2 used to drop
  cycle-1 test 14.
- **Inputs:** two independent setups (two regions, two shims,
  two core peers) running the identical sequence:
  - `make` with the same `desc` and `sim_time_us = 0`.
  - Core sends `boot_ack` at seq 0; shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 1 with
    `valid_clock_state(50'000)`; shim `poll()`.
  - Core sends `tick_boundary`/`power_state` at seq 2 with
    `valid_power_state(12.5f, 2.0f, 6.8f)`; shim `poll()`.
  - Core sends `tick_boundary`/`ds_state` at seq 3 with
    `valid_ds_state()`; shim `poll()`.
  - Core sends `tick_boundary`/`clock_state` at seq 4 with
    `valid_clock_state(100'000)`; shim `poll()`.
- **Expected outputs:**
  - For both setups (each gated independently):
    `ASSERT_TRUE(setup_a.latest_clock_state().has_value())`,
    `ASSERT_TRUE(setup_a.latest_power_state().has_value())`,
    `ASSERT_TRUE(setup_a.latest_ds_state().has_value())`,
    `ASSERT_TRUE(setup_b.latest_clock_state().has_value())`,
    `ASSERT_TRUE(setup_b.latest_power_state().has_value())`,
    `ASSERT_TRUE(setup_b.latest_ds_state().has_value())`.
    **Required `ASSERT_TRUE` (not `EXPECT_TRUE`)** — same UB-on-
    null-optional rationale as C2-6.
  - `*setup_a.latest_clock_state() == *setup_b.latest_clock_state()`
    (full-struct equality, post-`ASSERT_TRUE` dereference).
  - `*setup_a.latest_power_state() == *setup_b.latest_power_state()`.
  - `*setup_a.latest_ds_state() == *setup_b.latest_ds_state()`
    (full-struct equality recursing through every nested
    aggregate).
  - `EXPECT_EQ(std::memcmp(&*setup_a.latest_ds_state(),`
    ` &*setup_b.latest_ds_state(), sizeof(ds_state)), 0)`.
    **Required separately from `operator==` above.** The
    defaulted `ds_state::operator==` is field-by-field equality
    that the C++ standard does not require to compare padding
    bytes (and GCC/Clang in practice do not). `ds_state`
    carries five interior padding bytes (2 at offsets 2226–2227
    between `joystick_descriptors[5]` and `control`; 1 at
    `match_info`+135; 2 at `match_info`+138–139 per the H8
    layout note). A future regression that constructs the
    cache entry through a non-zero-init path would leave
    garbage in those padding bytes, producing byte-non-
    identical logs while still passing `operator==`. The
    `std::memcmp` assertion pins the byte-identical guarantee
    that CLAUDE.md non-negotiable #5 ("byte-identical logs")
    requires. clock_state and power_state do not need a
    parallel `memcmp` companion: `power_state` is three packed
    `float`s with no padding (`sizeof == 12 == 3 ×
    sizeof(float)`); `clock_state` is similarly padding-free
    by its definition. ds_state is the first cycle-3
    introduction of a struct where the gap between
    `operator==` and byte-equality matters, and this is the
    cycle to pin it.
  - `EXPECT_TRUE(setup_a.is_connected())`,
    `EXPECT_TRUE(setup_b.is_connected())`. Structural soundness
    check; `EXPECT_TRUE` is fine because no dereference follows.
- **Bug class:** any seam-level nondeterminism in the new
  ds_state path (an `operator==` mismatch on padding bytes
  introduced by a fixture refactor — guarded by D-C3-7's
  zero-init contract); ABI mismatch on `ds_state` between
  compile configurations (alignof/sizeof divergence between
  clang Debug and GCC + ASan + UBSan would surface here as a
  byte-equality failure).

---

## Test rewrites (cycle-1+2 plan touched)

### C3-R10. `ShimCorePoll.RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext` (rewrite #2)

- **Source of change:** D-C3-4. Cycle 3 makes `ds_state`
  supported; cycle-2's rewrite of cycle-1 test 10 (which used
  `ds_state` as the probe) would now silently no-op the
  rejection contract.
- **Mechanical changes to `tests/backend/shim/shim_core_test.cpp`
  test 10** (all three are required; the inline comment update
  is called out explicitly because a literal find/replace on the
  schema_id alone would leave it stale):
  1. **Send line.** Change `schema_id::ds_state` →
     `schema_id::can_frame_batch`. **Implementation
     correction:** `can_frame_batch` is a *variable-size*
     schema; the validator computes
     `expected_payload_bytes == offsetof(can_frame_batch, frames) +
     count * sizeof(can_frame)` from the 4-byte `count` header at
     the start of the payload (per `validator.cpp`'s
     `expected_variable_payload_size`). Sending
     `bytes_of(can_frame_batch{})` (the full 1284-byte struct)
     is rejected by the framing validator because `count == 0`
     means `expected_payload_bytes == 4`. The test uses an
     explicit 4-byte zero-init buffer (the count=0 header) as the
     payload — minimal, framing-valid, and exercises the dispatch-
     reject contract identically to the original "any framing-
     valid payload of the right size" intent. The previous
     plan-text claim that "1284 bytes... fits in
     `kTier1MaxPayloadBytes`" was technically true but irrelevant
     — the validator rejects on the count↔length mismatch before
     checking lane capacity. The local variable becomes
     `empty_batch_header` (a `std::array<std::uint8_t, 4>{}`).
  2. **Inline comment at the call site** (currently the
     cycle-2-rewritten lines):
     ```
     // Step A — send a valid tick_boundary/ds_state. Framing is valid
     // (kPerKindAllowedSchemas allows ds_state under tick_boundary),
     // so the protocol_session accepts and advances next-expected to 2.
     // The shim's post-session dispatch refuses the schema. Cycle 3
     // will land ds_state and migrate this probe to the next
     // still-unsupported schema.
     ```
     becomes:
     ```
     // Step A — send a valid tick_boundary/can_frame_batch. Framing is
     // valid (kPerKindAllowedSchemas allows can_frame_batch under
     // tick_boundary), so the protocol_session accepts and advances
     // next-expected to 2. The shim's post-session dispatch refuses
     // the schema. Cycle 4 will land can_frame_batch and migrate this
     // probe to the next still-unsupported schema (can_status).
     ```
  3. **Block-comment header above the `TEST(...)` line.** Update
     the cumulative supported-schema set: "Schemas other than
     `clock_state`, `power_state`, `none`" becomes "Schemas
     other than `clock_state`, `power_state`, `ds_state`,
     `none`". And the cycle phrasing "cycle-2 limit" becomes
     "cycle-3 limit", with the trailing migration note updated
     to reference cycle 4 → `can_status`.
- **Bug class (unchanged):** shim silently dropping schemas it
  can't yet handle (would mask cycle-4+ divergence); shim
  corrupting the session receive counter on dispatch reject (the
  Step B `clock_state` send at the next sequence would fail).

### C3-R6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalLatestPowerAndClockState` (drop)

This cycle-2 determinism test is **dropped from the file** and
replaced by C3-6 (which is a strict superset: same boot_ack +
clock + power + clock scenario plus a ds_state arrival
interleaved at seq 3, with all three slots asserted byte-equal
across the two runs).

Dropping rather than keeping both is per `code-style.md`'s "no
half-finished implementations" rationale that cycle 2 itself
applied to cycle-1 test 14. Any failure mode C2-6 catches, C3-6
catches strictly harder. There is no credible failure mode
where C2-6 would surface a regression that C3-6 misses, given
that the three cache members are independent and C3-6 asserts
each independently.

---

## Cross-test invariants (additions)

- All cycle-3 tests construct one shim per test; no shared
  state across tests.
- All `sim_time_us`, joystick axis values, `match_number`,
  `match_time_seconds`, and other fixture parameters are
  explicit constants. No wall-clock, no derived-from-current-
  state, no PRNG.
- All `ds_state` payload comparisons use full-struct
  `operator==` (recursing through nested aggregates). Field-
  level checks (e.g.
  `latest_ds_state()->match.match_number == 42`) appear only
  as readability aids in C3-5's per-step assertions where the
  concern is "did one specific identifying value change," not
  "was the whole struct copied correctly."
- C3-3a/b/c/d each pin a single bug class around the ds-slot
  observer (default-init drift, boot_ack contamination,
  clock_state contamination, power_state contamination
  respectively). Where C3-3b/c/d reference observers on the
  *other* slots, the references are prerequisite checks (see
  rationale per-test), not the test's primary concern.
- C3-5 and C3-6 explicitly assert on **all three** cache slots
  at every step. A test that asserts on only some slots misses
  a cross-population bug class for the unasserted slot at that
  step.

## Stub disclosure (additions)

- The "core" peer is real `tier1_endpoint` + real
  `protocol_session` (no fakes), same as cycle 1 + 2.
- C3-4 reaches behind the endpoint's send path via
  `manually_fill_lane`, same as cycle-1 test 9 and cycle-2
  C2-4. The injection rationale is identical: no real peer can
  produce a pre-boot-ack `tick_boundary`/`ds_state`.
- No new mocks, no new stubs. The SUT (`shim_core`) is never
  mocked.

---

## What this plan does NOT cover

- **Outbound `ds_state`.** The shim never sends `ds_state` in
  cycle 3 (it is purely consumer). Outbound past boot is
  cycle 4+ at the earliest.
- **C HAL ABI surfaces** (`HAL_GetControlWord`,
  `HAL_GetMatchInfo`, `HAL_GetJoystickAxes`, etc.) that read
  from `latest_ds_state_`. Cycle 4+.
- **Validity-domain checks on `ds_state` field values** (e.g.
  rejecting an out-of-range `alliance_station`, a NaN
  `match_time_seconds`, or a `joystick_axes_[i].count` greater
  than `kMaxJoystickAxes`). The shim is a byte-copy cache;
  field-domain validation is the producer's responsibility (sim
  core) and the consumer's responsibility (the future C HAL
  surfaces), not the shim's.
- **Padding-byte memcmp invariance on `clock_state` and
  `power_state`.** Both structs are padding-free by their
  field layouts (`power_state` is three packed `float`s;
  `clock_state` is a `uint64_t` followed by four naturally-
  aligned `uint32_t`-shaped booleans and a `uint32_t`
  counter), so `operator==` already covers byte-equality for
  them. `ds_state` padding-byte determinism **is** pinned
  directly by C3-6's `std::memcmp` assertion (rev-2 addition);
  this exemption applies only to the two cycle-1/2 schemas.
- **Post-shutdown / post-`lane_in_progress` `latest_ds_state()`
  invariance.** Same as cycle-2's reasoning for the analogous
  power-slot question: the shutdown short-circuit is schema-
  agnostic and `lane_in_progress` returns before any dispatch,
  so the invariance is by construction. Cycle-1 tests 12 and
  13 already pin those paths schema-agnostically. Out of
  scope.

---

## Open questions

None for cycle 3. The shape mirrors cycle 2 closely enough that
the design space has already been committed. If reviewer disagrees
on a specific decision (D-C3-1 .. D-C3-7) the discussion reopens.
