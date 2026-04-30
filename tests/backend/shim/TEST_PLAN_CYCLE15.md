# HAL shim core — cycle 15 test plan (HAL_Bool parity + HAL_GetBrownedOut)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, two review rounds). Round-1 verdict was `not-ready`
with one required change (C15-3's misleading "hygiene" block) plus
a documentation-clarity note. Round-2 picked option (b) for the
C15-3 fix (neutralize all four sibling `hal_bool` fields to 0,
making C15-3 self-sufficient for wrong-field isolation) and added
the `extern "C"` placement note for the `HAL_Bool` typedef.
Round-2 verdict `ready-to-implement` with no remaining issues.
All three open questions resolved (bundle parity fix; rename-not-
delete; explicit post-helper assignment).

**Implements:** the fifteenth TDD cycle of the HAL shim core, the
**fourth C HAL ABI surface**: `extern "C" HAL_Bool
HAL_GetBrownedOut(int32_t* status)` reading from
`latest_clock_state_->browned_out`. Cycles 1–14 are landed and
129-shim-test green; full project ctest 444/444 green under both
clang Debug and GCC Debug + ASan + UBSan.

Cycle 15 also fixes a pre-existing **HAL_Bool signedness parity
bug**: our `using hal_bool = std::uint32_t;` does not match WPILib's
`typedef int32_t HAL_Bool;` — the two have the same width but
different signedness. The bug has shipped through cycles 1–14
because no surface yet exposed `hal_bool` as a *return value*; every
prior use was inside POD payload structs where the wire bytes for
valid 0/1 values are byte-identical between `uint32_t` and `int32_t`.
HAL_GetBrownedOut is the first cycle that exposes the type at the
C ABI seam, so it is the right cycle to fix the parity.

---

## Why this cycle exists

Two intertwined motivations:

1. **First HAL_Bool-returning C ABI surface.** WPILib's
   `HAL_GetBrownedOut(int32_t* status) -> HAL_Bool` returns a status-
   typed bool indicating whether the FPGA's brownout monitor has
   tripped. The shim reads `latest_clock_state_->browned_out` — a
   `hal_bool` field. The C ABI seam needs to expose this with the
   same WPILib-faithful type signature, which means
   `typedef int32_t HAL_Bool;` in `hal_c.h` and a
   `HAL_Bool HAL_GetBrownedOut(int32_t* status);` declaration.

2. **HAL_Bool signedness parity bug.** The cycle-1-era
   `src/backend/common/types.h` declared:
   ```cpp
   // Mirrors WPILib HAL_Bool from hal/include/hal/Types.h.
   using hal_bool = std::uint32_t;
   ```
   But WPILib's `hal/include/hal/Types.h` actually defines
   `typedef int32_t HAL_Bool;` (signed, not unsigned). Our test-side
   `tests/backend/common/wpilib_mirror.h` already pins
   `HAL_Bool = std::int32_t` correctly (the parity tests use it for
   wire-layout checks where signedness doesn't show up). Existing
   tests pass because for valid bool values 0 and 1 the bit pattern
   is identical between `uint32_t` and `int32_t`. The bug is purely
   a *type-name parity* issue that surfaces at the C ABI seam — the
   first time the shim *returns* `HAL_Bool` (cycle 15), the
   signedness becomes part of the public contract.

   Fixing parity is one line in `types.h` plus follow-on adjustments
   to the static_assert, one test (`Types.HalBoolIsUint32` →
   `Types.HalBoolIsInt32`), three TEST_PLAN.md references, two
   skill cross-references, and three comment-block updates in
   schema headers. Wire format is **byte-identical** for every
   value the codebase actually uses (always 0 or 1).

`HAL_GetBrownedOut` itself is structurally a pure mirror of cycle
12's `HAL_GetFPGATime` — same shape, different cache slot, different
return type. The 4-test pattern from cycles 13/14 carries over.

---

## Contract under test

### Production-side changes

`src/backend/common/types.h`:

```cpp
// Mirrors WPILib HAL_Bool from hal/include/hal/Types.h byte-for-byte
// AND signedness-for-signedness (cycle 15: signedness parity fix).
// Previously declared as uint32_t; the wire format is unchanged for
// valid 0/1 values, but the C++ type contract now matches WPILib.
using hal_bool = std::int32_t;

// ... existing hal_handle = std::int32_t ...

static_assert(sizeof(hal_bool) == 4);
static_assert(std::is_same_v<hal_bool, std::int32_t>);
```

`src/backend/shim/hal_c.h`:

```cpp
extern "C" {

// ... existing HAL_GetFPGATime, HAL_GetVinVoltage, HAL_GetVinCurrent ...

// Mirrors WPILib hal/include/hal/Types.h byte-for-byte AND
// signedness-for-signedness. WPILib defines this as
// `typedef int32_t HAL_Bool;`. Placed inside the `extern "C"`
// block (file-scope C linkage) so robot code including hal_c.h
// from a C TU sees the type as well; the typedef is a transparent
// alias for `int32_t` and has the same ABI.
typedef std::int32_t HAL_Bool;

// Mirrors WPILib HAL_GetBrownedOut from hal/include/hal/HALBase.h.
// Returns 1 if the FPGA's brownout monitor has tripped, 0 otherwise.
// For the shim, reads `latest_clock_state_->browned_out` directly
// (the cache field is already `hal_bool == HAL_Bool`).
//
// Status semantics mirror the other cycle-12+ HAL_Get* surfaces:
//   - shim_core::current() == nullptr: *status = kHalHandleError;
//     return 0.
//   - shim installed but latest_clock_state() == nullopt: *status =
//     kHalSuccess; return 0. Matches the model that the brownout
//     monitor is "not tripped" before the first clock_state arrives,
//     plus the WPILib contract that HAL_GetBrownedOut always
//     succeeds.
//   - shim installed and clock_state cached: *status = kHalSuccess;
//     return latest_clock_state()->browned_out.
HAL_Bool HAL_GetBrownedOut(std::int32_t* status);

}  // extern "C"
```

`src/backend/shim/hal_c.cpp`: definition of `HAL_GetBrownedOut`
follows the same 5-line shape as `HAL_GetFPGATime`, returning
`cached->browned_out` directly (no cast, no widening — the type
is already `HAL_Bool`).

### Comment-only updates (production)

- `src/backend/common/notifier_state.h` lines 21–22: comments say
  "hal_bool == uint32"; update to "hal_bool == int32".
- No changes needed to actual struct fields — they continue to use
  `hal_bool` and the byte layout is unchanged.

### Test-side changes

- `tests/backend/common/protocol_test.cpp` lines 95–98: rename
  `TEST(Types, HalBoolIsUint32)` to `TEST(Types, HalBoolIsInt32)`
  and change the `is_same_v<hal_bool, std::uint32_t>` assertion to
  `is_same_v<hal_bool, std::int32_t>`. The test still
  belt-and-suspenders the static_assert in `types.h`.
- `tests/backend/common/TEST_PLAN.md`: update three references
  (lines 97, 397–399 around decision #22, 506–508 around A5,
  990 in the validation summary) to reflect the int32_t typedef.
- `tests/backend/shim/shim_core_test.cpp`: add 4 new tests in a
  new `HalGetBrownedOut` suite (see "Proposed tests" below).

### Skill / docs updates

- `.claude/skills/protocol-schema.md` lines 74, 139–140: update the
  table entry for `types.h` and decision #22's "uint32_t" reference
  to "int32_t".
- `.claude/skills/hal-shim.md`: add cycle-15 entry to Validation
  Status, cross-references; update the description with the new
  HAL surface; pin the new design decision D-C15-HAL-BOOL-SIGNED-
  PARITY.
- `CLAUDE.md`: bump status snapshot to cycle 15 / 448 tests.
- `docs/STATUS_SIM_CORE.md`: add cycle-15 section, update headline.

### Out of scope (deferred to later cycles)

- HAL_GetBrownoutVoltage (`latest_power_state_->brownout_voltage_v`)
  — pure float→double mirror of cycles 13/14; no new design
  decisions; natural cycle-16 follow-on.
- HAL_GetSystemActive (reads `clock_state::system_active`) — second
  HAL_Bool reader, pure mirror of cycle 15; natural cycle-17.
- Other HAL_Bool-returning surfaces (HAL_GetSystemTimeValid,
  HAL_GetFPGAButton, HAL_GetRSLState) — sibling cycles.
- `HAL_GetControlWord` (struct out-param), `HAL_SendError`
  (outbound buffer), `HAL_CAN_SendMessage`, `HAL_Notifier*` — each
  introduces new mechanics worth its own cycle.
- Threading, LD_PRELOAD, NULL-status guard — same as cycles 12–14.

---

## Decisions pinned

### New: D-C15-HAL-BOOL-SIGNED-PARITY

`hal_bool` changes from `uint32_t` to `int32_t` to match WPILib's
`HAL_Bool = int32_t`. Wire format is byte-identical for every value
the codebase actually uses (0 and 1); the change is a C++ type
contract fix. Pinned by an updated static_assert in `types.h` and
the renamed `TEST(Types, HalBoolIsInt32)` runtime test.

**Rationale:** WPILib parity is a project non-negotiable
(CLAUDE.md #4 "vendor-faithful"). The shipped `using hal_bool =
std::uint32_t;` was a bug — the comment in the same file says
"Mirrors WPILib HAL_Bool" but the typedef does not actually mirror
the signedness. Cycle 15 closes the bug at the cycle that first
surfaces it (HAL_GetBrownedOut returns `HAL_Bool` at the C ABI
seam, where signedness becomes part of the public contract).

**Alternatives considered:**
- Leave `hal_bool` as `uint32_t` and cast at the C ABI seam:
  `static_cast<HAL_Bool>(cached->browned_out)`. **Rejected** — the
  whole point of the `hal_bool` typedef is "this is HAL_Bool"; if
  the cast is needed at every read, the typedef has no value.
- Add a separate `using hal_bool_signed = std::int32_t;` for the
  C ABI seam. **Rejected** — two typedefs for the same WPILib
  type doubles the mental cost.

### New: D-C15-HAL-BOOL-TYPEDEF-IN-HAL-C-H

`hal_c.h` exports `typedef std::int32_t HAL_Bool;` matching
WPILib's `hal/include/hal/Types.h` byte-for-byte AND name-for-name.
The C ABI signatures use `HAL_Bool`, not raw `int32_t`, for full
WPILib parity. Pinned at the cycle that first uses it.

**Rationale:** Robot code reading `hal_c.h` should see the same
type names as WPILib — anything else creates a "do I match WPILib
or do I match robosim's flavor of WPILib" question every time.
Future bool-returning HAL_* surfaces (HAL_GetSystemActive,
HAL_GetFPGAButton, HAL_GetRSLState, HAL_GetSystemTimeValid) inherit.

### Inherited unchanged from cycles 12–13

- D-C12-GLOBAL-ACCESSOR (session-level).
- D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function; pinned by C15-1).
- D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO per-function variant for
  `clock_state` (pinned by C15-2).
- D-C12-STATUS-WRITE-UNCONDITIONAL (per-function via sentinel).
- D-C12-LATEST-WINS-READ (per-function; pinned by C15-4).
- D-C13-FLOAT-TO-DOUBLE-CAST does **not** apply (HAL_Bool reader,
  not float reader).

---

## Test fixtures

No new helpers. Existing `valid_clock_state` covers most of the
surface; cycle 15 reads `browned_out` which the helper leaves
zero-init. Tests set `state.browned_out = 1;` after constructing
via the helper rather than expanding the helper's signature (avoids
adding params for a one-cycle need; mirrors how cycles 13/14 did
explicit field assignments past the helper's positional defaults).

`shim_global_install_guard` is reused unchanged from cycle 12.

---

## Determinism notes

`HAL_GetBrownedOut` is a pure read with the same shape as
`HAL_GetFPGATime`. Cycle 8's `latest_clock_state_` byte-identity
replay covers the cache contents. No new determinism class. No new
determinism test.

The hal_bool signedness fix does **not** change wire bytes for any
valid value — `uint32_t` and `int32_t` have identical bit patterns
for 0 and 1 (and for any value < 0x80000000), and the schema only
ever sends 0 or 1 in practice. Cycle 8's existing determinism
replay will continue to pass byte-for-byte across the type change.

---

## Proposed tests (revision 1)

### A pre-flight test for the parity fix

The existing `tests/backend/common/protocol_test.cpp` test
`Types.HalBoolIsUint32` is renamed and inverted as part of the
production code change. After the change, the test reads:

```cpp
TEST(Types, HalBoolIsInt32) {
  EXPECT_EQ(sizeof(hal_bool), 4u);
  EXPECT_TRUE((std::is_same_v<hal_bool, std::int32_t>));
}
```

This is a strict refactor of an existing test — same coverage
intent, corrected expected type. No new test added on the protocol
side.

### C15-1. `HalGetBrownedOut.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR (per-function);
  D-C12-STATUS-WRITE-UNCONDITIONAL (per-function).
- **Setup:**
  - `shim_core::install_global(nullptr);`
  - `ASSERT_EQ(shim_core::current(), nullptr);`
  - `int32_t status = 999;`
- **Action:** `HAL_Bool b = HAL_GetBrownedOut(&status);`
- **Expected:** `b == 0`; `status == kHalHandleError`.
- **Bug class:** null deref; status not written; wrong return type
  (a `bool` rather than `HAL_Bool` would not catch the signedness
  parity but compiler would convert; `uint32_t` return would be
  byte-equal to `int32_t` for 0 — caught by the static_assert,
  not the runtime test).

### C15-2. `HalGetBrownedOut.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO
  per-function variant for `clock_state`.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - `ASSERT_FALSE(shim.latest_clock_state().has_value());`
  - `shim_global_install_guard guard{shim};`
  - `int32_t status = 999;`
- **Action:** `HAL_Bool b = HAL_GetBrownedOut(&status);`
- **Expected:**
  - `b == 0`; `status == kHalSuccess`.
  - `shim.latest_clock_state().has_value() == false` after the call.
- **Bug class:** UB on disengaged optional; wrong status code;
  function returns `kHalHandleError` on empty cache (would surface
  boot-race state to robot code as a brownout error, which would
  likely brick the robot bootstrap).

### C15-3. `HalGetBrownedOut.WithCachedClockStateBrownedOutTrueReturnsOneAndSetsSuccessStatus`

- **Layer / contract:** end-to-end protocol-session →
  `latest_clock_state_` → C ABI integration with `browned_out=1`.
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - `auto state = valid_clock_state(/*sim_time_us=*/50'000);` then
    explicitly **neutralize all four sibling hal_bool fields to 0**
    so a wrong-field read on any of them returns 0 instead of the
    expected 1:
    ```cpp
    state.system_active       = 0;
    state.browned_out         = 1;  // distinguished value
    state.system_time_valid   = 0;
    state.fpga_button_latched = 0;
    state.rsl_state           = 0;
    ```
    This makes C15-3 **self-sufficient** for wrong-field isolation
    among `hal_bool` fields without relying on C15-4 (round-1
    reviewer required change to remove the previously-misleading
    "hygiene" prose that only zeroed one of four sibling fields).
  - Drive a `clock_state` envelope; poll; cache populated.
  - `int32_t status = 999;`
- **Action:** `HAL_Bool b = HAL_GetBrownedOut(&status);`
- **Expected:** `b == 1`; `status == kHalSuccess`.
- **Bug class:** function reads from a different cache slot
  (`latest_power_state_->vin_v` reinterpreted as bytes — would be
  a wildly wrong value); function reads the wrong `hal_bool`
  field within `clock_state` (C15-3's all-siblings-zero fixture
  now closes this directly: any of `system_active`,
  `system_time_valid`, `fpga_button_latched`, `rsl_state` would
  return 0 instead of the expected 1); function returns the cached
  value cast to `bool` and back to `HAL_Bool` (would still be 1 —
  moot).

### C15-4. `HalGetBrownedOut.LatestWinsAcrossTwoUpdatesIsolatesBrownedOutFromOtherHalBoolFields`

- **Layer / contract:** D-C12-LATEST-WINS-READ per-function variant
  for `latest_clock_state_->browned_out`. **Plus** wrong-field
  isolation: the two clock_state updates change `browned_out` AND
  the four sibling `hal_bool` fields in opposite directions, so a
  "shim reads `system_active` instead of `browned_out`" bug
  produces the wrong observable result.
- **Setup:**
  - Fresh region; shim made; install guard installed.
- **Action sequence:**
  1. Drive clock_state where `browned_out=0` BUT all four sibling
     hal_bool fields are 1: `state.sim_time_us = 50'000;
     state.system_active=1; state.browned_out=0;
     state.system_time_valid=1; state.fpga_button_latched=1;
     state.rsl_state=1;`. Poll, OK.
  2. `int32_t status1 = 999;
     HAL_Bool b1 = HAL_GetBrownedOut(&status1);`
     Expect `b1 == 0`, `status1 == kHalSuccess`. **A wrong-field
     read returns 1 here, failing the assertion.**
  3. Drive a second clock_state where `browned_out=1` BUT all four
     sibling hal_bool fields are 0: `state.sim_time_us = 100'000;
     state.system_active=0; state.browned_out=1;
     state.system_time_valid=0; state.fpga_button_latched=0;
     state.rsl_state=0;`. Poll, OK.
  4. `int32_t status2 = 888;
     HAL_Bool b2 = HAL_GetBrownedOut(&status2);`
     Expect `b2 == 1`, `status2 == kHalSuccess`. **A wrong-field
     read returns 0 here, failing the assertion.**
- **Bug class:** caches-on-first-call (would show `b2 == 0`);
  global-pointer snapshot at install time (same observable
  failure); function reads `system_active` instead of `browned_out`
  (b1 = 1 != expected 0); function reads `rsl_state` instead
  (same); function reads `fpga_button_latched` instead (same).
  This single test catches both the latest-wins bug class and
  the wrong-field bug class through the deliberate
  inversely-correlated fixture.

---

## Tests deliberately not added

- **Idempotency mirror.** Same trim as cycles 13/14; the per-function
  read path is structurally a copy of HAL_GetFPGATime and the bug
  class is pinned.
- **NULL status guard.** UB matching WPILib.
- **Cross-cutting cycle-12 mirrors.** Session-level; pinned once.
- **Determinism replay for HAL_GetBrownedOut.** Cycle 8 covers
  `latest_clock_state_` byte-identity.
- **`Types.HalBoolIsUint32` renamed-not-deleted.** The runtime test
  is a strict refactor of an existing test — same belt-and-suspenders
  intent, corrected expected type. There is no new test on the
  protocol side; the static_assert in `types.h` is the actual
  contract.
- **Test that `HAL_Bool == hal_bool` exact-type-equality.**
  Considered, not added — `HAL_Bool` is a C-side typedef in
  `extern "C"` block (file-scope `typedef int32_t HAL_Bool;`), and
  `hal_bool` is a C++ namespaced `using` declaration. Both alias
  `int32_t`; checking via `is_same_v` would assert the same fact
  twice. The static_asserts in `types.h` (`hal_bool` is `int32_t`)
  and the size/signedness check in C15-1's `HAL_Bool` return
  collectively pin the parity.

---

## Cross-test invariants

Same as cycles 12–14:
- Sentinel pattern (999/888) for status writes.
- C15-1 has no shim and no guard.
- C15-2/3/4 use `make_connected_shim` + `shim_global_install_guard`.
- C15-3 / C15-4 fixture values are bit-distinct from cycle 12's
  `clock_state` test fixtures so a copy-paste-with-wrong-field bug
  is caught.

---

## Implementation companion recommendations (non-test)

- The schema-wide `hal_bool` change is one line in `types.h`. After
  the change, rebuild the full project; any existing test that
  fails is a real regression to investigate.
- **Order of operations during implementation:**
  1. Change `using hal_bool = std::uint32_t;` → `std::int32_t;` and
     update the static_assert in `types.h`.
  2. Update `Types.HalBoolIsUint32` → `Types.HalBoolIsInt32` in
     `tests/backend/common/protocol_test.cpp`.
  3. Update comments in `notifier_state.h` (lines 21–22) and any
     other in-tree docstrings that mention "uint32" for hal_bool.
  4. Build the full project; run all tests under both clang and
     ASan; confirm 444/444 still green (parity fix should be
     wire-and-binary-compatible for valid 0/1 values).
  5. Add `typedef int32_t HAL_Bool;` and the `HAL_GetBrownedOut`
     declaration to `hal_c.h`.
  6. Add the broken-stub `HAL_GetBrownedOut` definition to
     `hal_c.cpp` (returns 0 unconditionally without writing status,
     same TDD discipline as cycles 12/13/14).
  7. Add the 4 cycle-15 tests; confirm they all fail.
  8. Replace the stub with the real 5-line implementation.
  9. Confirm 448/448 green under both build matrices.
- **TEST_PLAN.md and skill updates.** Three references in
  `tests/backend/common/TEST_PLAN.md` and two in
  `.claude/skills/protocol-schema.md` need to mention "int32_t"
  rather than "uint32_t" for hal_bool. These are
  documentation-only.

---

## Open questions

**OQ-C15-PARITY-FIX-FRAMING.** The plan bundles the schema-wide
`hal_bool` signedness change with the new HAL surface in one cycle.
An alternative is to split into cycle-15a (parity fix only) and
cycle-15b (HAL_GetBrownedOut). Plan picks bundling because the fix
is genuinely motivated by the surface (HAL_GetBrownedOut is the
first cycle that *exposes* the type at the C ABI), and a
parity-only cycle would have no new feature — violating the
"every cycle adds something" pattern. Reviewer welcome to argue
for splitting.

**OQ-C15-WRONG-FIELD-COVERAGE-IN-C15-3-OR-C15-4.** **Resolved at
round 1 in favor of "C15-3 self-sufficient + C15-4 covers the
latest-wins-plus-isolation joint contract."** The original plan's
"let C15-4 catch the wrong-field bug" approach was honest but the
prose mitigation in C15-3 misrepresented the scope of its
coverage. Round-1 reviewer required tightening C15-3's fixture so
all four sibling `hal_bool` fields are 0 while `browned_out = 1`,
making C15-3 catch wrong-field bugs directly. C15-4 still uses
inversely-correlated fixtures across the two updates (which has
two purposes: latest-wins itself, and double-checking wrong-field
isolation across an actual transition rather than a single read).
The cost is six explicit field assignments in C15-3 — well below
the threshold for a helper-param expansion (which OQ-C15-FIELD-
PARAM-ON-VALID-CLOCK-STATE explicitly rejected).

**OQ-C15-FIELD-PARAM-ON-VALID-CLOCK-STATE.** Should
`valid_clock_state` grow a `browned_out` parameter, or should the
test set `state.browned_out = 1;` after the helper call? Plan
picks "explicit field assignment" because (a) only one test in
this cycle needs the non-default value, (b) the helper's positional-
arg list would grow to 6+ params and lose readability, and (c)
the precedent from cycle 13's C13-3 fixture (which calls
`valid_power_state(7.5f, 99.25f, 6.5f)` explicitly rather than
adding a new helper signature) supports targeted member-access.
Reviewer welcome to argue for the helper expansion if multiple
future cycles will need the same access pattern.
