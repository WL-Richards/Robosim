# HAL shim core — cycle 12 test plan (C HAL ABI plumbing + HAL_GetFPGATime)

**Status:** **`ready-to-implement`** per `test-reviewer` agent
(2026-04-30, three review rounds). Round-1 verdict was `not-ready`
with three blockers (storage-location ambiguity; C12-2 internal
contradiction; `shim_global_install_guard` double-qualification) and
five non-blocking items (C12-1 / C12-4 explicit precondition wording;
C12-7 restructured to actually catch "caches-on-first-call";
D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO documentation note;
OQ-C12-STORAGE-LOCATION resolution). Round-2 had one residual blocker
(stale "storage in shim_core.cpp" sentence in D-C12-GLOBAL-ACCESSOR
contradicting the resolved OQ) plus two non-blocking items (revision
label drift; C12-3 setup-block silence on the pre-clear step).
Round-3 closed all of those. The plan is revision 3.

**Implements:** the twelfth TDD cycle of the HAL shim core, the
**first C HAL ABI surface**. Cycles 1–11 are landed and 113-shim-test
green; full project ctest 428/428 green under both clang Debug and
GCC Debug + ASan + UBSan. The shim's surface to date is a C++ API
(`shim_core::make` / `poll` / `send_*` / observers); cycle 12
introduces the **first `extern "C"` symbol** the user's robot binary
will eventually call (`HAL_GetFPGATime`), plus the process-global
shim-instance accessor that all subsequent C HAL surfaces will share.

The C HAL ABI is the **largest remaining shim chunk** — every
`HAL_*` symbol the WPILib HAL header declares ultimately needs an
implementation. Cycle 12 keeps scope narrow: introduce the new
translation unit, pin the global-accessor pattern, and ship the
**simplest** read function (HAL_GetFPGATime is a pure cache read
of `latest_clock_state()->sim_time_us`). Subsequent cycles
(HAL_GetVinVoltage, HAL_GetControlWord, HAL_SendError,
HAL_CAN_SendMessage, HAL_Notifier*, etc.) follow the same plumbing
without needing to re-litigate the pattern.

---

## Why this cycle exists

The C ABI is a hard **linguistic boundary**: robot code is unmodified
WPILib code that calls `HAL_GetFPGATime(&status)`, and at link/load
time those symbols must resolve to *our* shim, not to NI's
`libwpiHal.so`. v0's path to that resolution is LD_PRELOAD (per
ARCHITECTURE.md), but the LD_PRELOAD machinery is downstream — what
we need first is the C function bodies themselves, defined in a TU
that exports `extern "C"` symbols matching the WPILib HAL header
signatures byte-for-byte.

`HAL_GetFPGATime` is the natural starting point because:

1. **It is the highest-frequency HAL call in real robot code.** Every
   `Notifier`, every `WPI_TimedRobot::run()`, every PID loop reads it
   for time deltas. The `tools/rio-bench/` project uses it as the
   FPGA-timestamp source for measuring every other call's cost.
2. **It is a pure cache read.** No outbound buffering, no queue
   semantics, no notification — just `latest_clock_state_->sim_time_us`.
   This makes it the smallest test surface that meaningfully exercises
   the new C-ABI translation layer.
3. **`clock_state` is the first inbound schema (cycle 1).** It is
   guaranteed to exist in the shim's cache machinery; no upstream
   protocol-schema work is required.
4. **It pins the WPILib `int32_t* status` calling convention.**
   Every other HAL function follows the same pattern; getting it right
   here means subsequent cycles are mechanical.

---

## Contract under test

### New public surface

`src/backend/shim/shim_core.h` — additions only (the existing class
keeps its v1–v11 API):

```cpp
class shim_core {
 public:
  // ... existing make / poll / send_* / observers ...

  // Process-global accessor for the C HAL ABI. Robot code calls
  // exported HAL_* symbols (e.g. HAL_GetFPGATime) which look up the
  // installed shim_core* via current() and dispatch into its observers
  // / send methods. Caller installs after a successful shim_core::make
  // and clears (passes nullptr) before destruction; the pointer is
  // not owned. v0 is single-threaded; the storage is a non-atomic
  // file-static. The future threading cycle promotes to std::atomic.
  static void install_global(shim_core* shim) noexcept;
  [[nodiscard]] static shim_core* current() noexcept;
};
```

`src/backend/shim/hal_c.h` (new TU header):

```cpp
#pragma once

#include <cstdint>

namespace robosim::backend::shim {

// Status codes mirroring WPILib HAL/Errors.h. The constants we use
// in v0:
//   kHalSuccess     == HAL_SUCCESS       == 0
//   kHalHandleError == HAL_HANDLE_ERROR  == -1
// The full WPILib enumeration is much larger; we add constants here
// only as the surfaces that consume them land.
inline constexpr std::int32_t kHalSuccess     =  0;
inline constexpr std::int32_t kHalHandleError = -1;

}  // namespace robosim::backend::shim

extern "C" {

// Mirrors WPILib HAL_GetFPGATime from hal/include/hal/HALBase.h.
// Returns the FPGA timestamp in microseconds (the sim time published
// by the Sim Core via the most-recently-cached clock_state envelope).
//
// Status semantics:
//   - shim_core::current() == nullptr: *status = kHalHandleError;
//     return 0. Robot code that calls a HAL function before the
//     shim is installed sees a clear error code rather than UB.
//   - shim installed but latest_clock_state() == nullopt (boot has
//     happened but no clock_state has been received yet): *status =
//     kHalSuccess; return 0. Matches the sim's authoritative
//     "sim_time starts at 0" model and the WPILib contract that
//     HAL_GetFPGATime always succeeds.
//   - shim installed and clock_state cached: *status = kHalSuccess;
//     return latest_clock_state()->sim_time_us.
//
// Status pointer must not be NULL — UB if it is, matching WPILib's
// header contract (the function unconditionally writes through the
// pointer).
std::uint64_t HAL_GetFPGATime(std::int32_t* status);

}  // extern "C"
```

### New translation unit

`src/backend/shim/hal_c.cpp` — defines `HAL_GetFPGATime` and the
file-static `install_global` / `current` storage. The static lives in
this TU (not `shim_core.cpp`) because it is process-state owned by
the C HAL surface, not by the shim instance — a future
`HAL_Initialize` cycle will read the same pointer.

### Internal additions

- **Storage lives in `hal_c.cpp` as a TU-static `shim_core* g_installed_shim_
  = nullptr`** (per OQ-C12-STORAGE-LOCATION resolution at round 1: the
  storage is owned by the C HAL surface, not by any one shim instance).
  The `shim_core::install_global` / `shim_core::current` static methods
  are **declared** in `shim_core.h` (so callers see them on the class)
  but **defined** in `hal_c.cpp` (next to the storage they touch). This
  keeps method discoverability while putting bytes-of-state in the TU
  that owns them — directly preparing the future LD_PRELOAD split where
  `hal_c.cpp` compiles into `libhal.so` and `shim_core.cpp` stays
  separate.
- `shim_core.cpp` itself is **not modified** for storage; it gains no
  new file-scope state. The two new methods (`install_global`,
  `current`) are declared in `shim_core.h` and defined in `hal_c.cpp`.

### CMake

- `src/backend/shim/CMakeLists.txt` — add `hal_c.cpp` to the
  `robosim_backend_shim` STATIC sources. No new target.
- `tests/backend/shim/CMakeLists.txt` — no change; existing
  test executable picks up the new TU transitively.

### Out of scope (deferred to later cycles)

- `HAL_Initialize` / `HAL_Shutdown`. Future cycle owns the
  initialize-once / mode-and-timeout argument semantics.
- Every other HAL_* read/write function. Each is its own cycle:
  HAL_GetVinVoltage, HAL_GetVinCurrent, HAL_GetBrownedOut (read
  from `latest_power_state_`); HAL_GetControlWord, HAL_GetAlliance,
  HAL_GetMatchTime, HAL_GetJoystickAxes, etc. (read from
  `latest_ds_state_`); HAL_CAN_SendMessage (outbound buffer →
  send_can_frame_batch); HAL_SendError (outbound buffer →
  send_error_message_batch); HAL_Notifier* (outbound buffer →
  send_notifier_state). Each cycle adds the tests, the
  symbol(s), and any per-surface design decisions.
- Threading. `install_global` / `current` storage is non-atomic;
  v0 robot bootstrap is single-threaded. The threading cycle
  promotes the storage to `std::atomic<shim_core*>` with relaxed
  ordering on the read path.
- LD_PRELOAD wiring. The exported symbol exists; making the loader
  pick *our* libhal over NI's is downstream.
- NULL-status-pointer protection. WPILib HAL functions
  unconditionally write through `int32_t* status`; passing NULL is
  caller-side UB. We match.
- Multi-shim or multi-process scenarios. Single shim per process.
- Reset / reinstall after shutdown. After `is_shutting_down() == true`,
  the shim is dead; callers should `install_global(nullptr)` and the
  shim object is destroyed. Cycle 12 does NOT add a "HAL_GetFPGATime
  on a shutting-down shim" branch; the cache observer continues to
  return the last-cached `sim_time_us`, which is the correct behavior
  for "robot code reading the clock as it shuts down".

---

## Decisions pinned

### D-C12-GLOBAL-ACCESSOR

A single process-wide `shim_core*` slot is installed by the bootstrap
(or test harness) after `shim_core::make` succeeds, and cleared
before the shim object is destroyed. The C HAL functions read it via
`shim_core::current()`. Rationale: WPILib HAL is process-global; robot
code calls `HAL_*` symbols freely without passing a context handle.
Pinned by C12-1, C12-2, C12-3.

**Why a static member rather than a free function pair?** Discoverability
(`shim_core::current()` shows up in IDE autocomplete next to the
constructor); locality (the lifetime contract is tied to the class).
The storage itself lives in `hal_c.cpp` as a TU-static
(`static shim_core* g_installed_shim_ = nullptr`); the
`shim_core::install_global` / `shim_core::current` static methods are
**declared** in `shim_core.h` but **defined** in `hal_c.cpp` next to
the storage they touch — see OQ-C12-STORAGE-LOCATION resolution below.

**Why not std::atomic?** v0 is single-threaded; the storage is set
once by bootstrap before any HAL call and cleared at shutdown.
Promoting to `std::atomic<shim_core*>` is a one-line change in the
threading cycle.

### D-C12-NULL-SHIM-IS-HANDLE-ERROR

When `shim_core::current() == nullptr`, `HAL_GetFPGATime` writes
`*status = kHalHandleError` (-1) and returns 0. Pinned by C12-4.

**Why HAL_HANDLE_ERROR specifically?** WPILib's status taxonomy is
sparse; there is no "HAL not initialized" code. HAL_HANDLE_ERROR
(-1) is the catch-all error code used elsewhere in WPILib for
"resource lookup failed", and "no shim installed" is closest in
spirit. A future cycle that introduces HAL_Initialize may swap in
HAL_RESOURCE_ALREADY_ALLOCATED or similar; this is a documented
v0 choice.

### D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO

When the shim is installed but `latest_clock_state() == nullopt`,
`HAL_GetFPGATime` writes `*status = kHalSuccess` and returns 0.
Pinned by C12-5.

**Rationale:** The sim's authoritative model is "sim_time starts at
0 at boot". HAL_GetFPGATime's WPILib contract is "always succeeds"
(the FPGA register is always valid on a real RIO). Returning 0 with
success matches both. The alternative — returning HAL_HANDLE_ERROR
on empty cache — would surface a sim-internal state (the boot/sync
race window) to robot code as an error, which violates the layer-2
non-negotiable that "robot code is unmodified".

**Indistinguishability note (round-1 reviewer recommendation):** The
return value `0` from an empty-cache call is **byte-identical** to
the return value `0` from a populated cache where `sim_time_us == 0`
(e.g. immediately after boot when the first clock_state has arrived
but carries the boot timestamp). This is **by design**: robot code
must not be able to distinguish "I haven't seen a tick yet" from
"the sim is at time 0", because either way the correct robot-side
behavior (treat sim_time as 0) is the same. The shim does not need
to expose its own boot-race state to the C ABI surface.

### D-C12-LATEST-WINS-READ

`HAL_GetFPGATime` reads the cache observer (`latest_clock_state()`)
each call; the cycle-1 latest-wins discipline applies unchanged.
Repeated calls without an intervening `poll()` return the same
value (no clock advance between syncs). Pinned by C12-6.

### D-C12-STATUS-WRITE-UNCONDITIONAL

`HAL_GetFPGATime` writes `*status` on every code path (`null shim`,
`empty cache`, `cached clock_state`). It does NOT preserve the
caller's previous `*status` value. Pinned implicitly by C12-4
(checks `*status == kHalHandleError` after a previously-successful
call would have set 0) and C12-6 (success path overwrites).

**Why?** WPILib's HAL functions all write status unconditionally;
matching the contract means callers can pass an uninitialized
`int32_t status` and rely on the function to set it. The opposite
contract ("only write on error", à la POSIX `errno`) would surprise
WPILib robot code.

---

## Test fixtures

### New helper: `tests/backend/tier1/test_helpers.h`

Add a single RAII guard so test bodies don't leak the global state
into one another:

The guard lives in the `robosim::backend::shim` namespace (NOT
`robosim::backend::tier1::helpers`) so the existing
`namespace shim` block in `shim_core_test.cpp` resolves it without
extra using-decls. The cleanest place is a small additional
inline section near the top of `test_helpers.h` wrapped in
`namespace robosim::backend::shim { ... }` — the file already opens
multiple namespaces.

```cpp
// In tests/backend/tier1/test_helpers.h, alongside the existing
// helpers but in the shim namespace (forward-declares shim_core and
// the install_global static).
namespace robosim::backend::shim {
class shim_core;  // forward decl

// Cycle-12 helper. Installs `shim` as the C-HAL-ABI process-global
// for the duration of the guard's scope, restoring nullptr on
// destruction. Tests using HAL_GetFPGATime (or future HAL_* surfaces)
// construct one of these after make_connected_shim returns.
class shim_global_install_guard {
 public:
  explicit shim_global_install_guard(shim_core& shim);
  ~shim_global_install_guard();
  shim_global_install_guard(const shim_global_install_guard&) = delete;
  shim_global_install_guard& operator=(const shim_global_install_guard&) = delete;
};
}  // namespace robosim::backend::shim
```

(The bodies must call `shim_core::install_global` which is declared
in `shim_core.h`, so the implementation is in a `.cpp` somewhere or
the helper file must include `shim_core.h`. Simplest: include
`shim_core.h` in `test_helpers.h` or define the bodies inline after
including it. Implementer's choice; both are valid.)

The guard lives in `test_helpers.h` rather than the production header
because production callers (the LD_PRELOAD shim, `HAL_Initialize`)
have their own lifetime models; only tests need the strict
construct-and-restore RAII.

No other helper additions. Existing `valid_clock_state(sim_time_us)`,
`valid_boot_descriptor`, `make_connected_shim`, `manually_fill_lane`,
and `make_envelope` cover the fixture surface.

---

## Determinism notes

`HAL_GetFPGATime` is a pure read of an existing cache slot. The
cycle-8 `RepeatedRunsProduceByteIdenticalAllEightSlots` determinism
test already pins byte-identical content for `latest_clock_state_`
across two scenario runs. Cycle 12 adds **no new determinism test**
because:

- The only seam-level nondeterminism cycle 12 could introduce is in
  the `int32_t* status` write or the return value, both of which are
  scalar reads of fixed integer fields with no padding, no
  uninitialized-stack-bytes class, and no atomic-ordering question
  in single-threaded v0.
- Adding a determinism test for HAL_GetFPGATime would essentially
  re-test the cycle-8 clock_state determinism with one extra layer
  of indirection. The bug class is identical and already pinned.

If the threading cycle promotes the global to `std::atomic`, that
cycle's determinism story (memory-order acquire/release for cross-
thread visibility) gets its own test.

---

## Proposed tests (revision 3)

### C12-1. `ShimCoreCurrent.DefaultsToNullptrBeforeAnyInstallCall`

- **Layer / contract:** D-C12-GLOBAL-ACCESSOR initial state.
- **Setup:** `shim_core::install_global(nullptr);` as an unconditional
  precondition at the top of the test body. Per round-1 finding, this
  converts a silent ordering dependency on test-declaration sequence
  into a loud, self-documenting precondition — the test holds even
  if reshuffled within the suite.
- **Action:** `auto* p = shim_core::current();`
- **Expected:** `p == nullptr`.
- **Bug class:** implementer initializes the static to a non-null
  sentinel (`reinterpret_cast<shim_core*>(0xDEADBEEF)`); implementer
  reads from a different static than the one `install_global(nullptr)`
  wrote. The C++ standard guarantees zero-init for `static T* p;` at
  global / TU scope, so a "forgot to write `= nullptr`" bug is not
  what this test catches — it catches the sentinel-init class.

### C12-2. `ShimCoreCurrent.AfterInstallGlobalReturnsTheInstalledPointer`

- **Layer / contract:** D-C12-GLOBAL-ACCESSOR set path.
- **Setup:**
  - `shim_core::install_global(nullptr);` as an explicit precondition
    so the test is robust to any prior test having leaked a non-null
    pointer (round-1 finding: cross-test coupling otherwise).
  - `tier1_shared_region region{};`
  - `tier1_endpoint core;`
  - `auto shim = make_connected_shim(region, core);` — produces a
    valid `shim_core` we can take the address of. The shim being
    "connected" or not is irrelevant to the install/current pointer
    round-trip; we use the existing helper rather than reaching into
    `shim_core::make` directly.
- **Action:** `shim_core::install_global(&shim); auto* p = shim_core::current();`
- **Cleanup:** `shim_core::install_global(nullptr);` at the end of
  the test body so C12-3 / C12-4 see the null precondition cleanly.
  (The `shim_global_install_guard` is NOT used here because the test
  is *about* `install_global` itself; using the guard would be
  testing-the-test-fixture rather than testing the surface.)
- **Expected:** `p == &shim`.
- **Bug class:** implementer swaps the storage and the parameter;
  implementer reads from a different static than the one it wrote.

### C12-3. `ShimCoreCurrent.InstallGlobalNullptrClearsCurrent`

- **Layer / contract:** D-C12-GLOBAL-ACCESSOR clear path.
- **Setup:**
  - `shim_core::install_global(nullptr);` as an explicit pre-clear
    (mirrors C12-2's discipline — every test that touches the global
    starts from a known-null state, so the suite is order-independent).
  - `tier1_shared_region region{};`
  - `tier1_endpoint core;`
  - `auto shim = make_connected_shim(region, core);`
  - `shim_core::install_global(&shim);` — set non-null starting state.
- **Action:** `shim_core::install_global(nullptr); auto* p = shim_core::current();`
- **Cleanup:** none needed (the test's action IS the clear, and the
  post-action global state is null — exactly what downstream tests
  expect as their precondition).
- **Expected:** `p == nullptr`.
- **Bug class:** clear path is missing or guarded behind a check
  (e.g. `if (p) static_ = p;` would silently drop the nullptr write
  and leave a dangling pointer to the destroyed shim — a critical
  security/correctness bug that the threading cycle inherits).

### C12-4. `HalGetFPGATime.WithNoShimInstalledSetsStatusToHandleErrorAndReturnsZero`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR;
  D-C12-STATUS-WRITE-UNCONDITIONAL.
- **Setup:**
  - `shim_core::install_global(nullptr);` as an explicit precondition
    (round-1 hardening — the next assertion makes the precondition
    loud rather than silent).
  - `ASSERT_EQ(shim_core::current(), nullptr);` — verifies that the
    install path is actually clear before the call. If C12-2's
    cleanup is ever dropped, this fails loudly here rather than
    silently flipping C12-4 into a false success.
  - `int32_t status = 999;` — a sentinel distinct from both
    `kHalSuccess` (0) and `kHalHandleError` (-1) so the test catches
    a "function only writes status on the success path" bug.
- **Action:** `std::uint64_t t = HAL_GetFPGATime(&status);`
- **Expected:**
  - `t == 0`.
  - `status == kHalHandleError` (-1).
- **Bug class:** function dereferences the null shim pointer (UB,
  ASan-detectable); function returns garbage from a static; function
  forgets to overwrite the caller's pre-existing status (would leave
  `999` in place).

### C12-5. `HalGetFPGATime.WithShimInstalledButCacheEmptyReturnsZeroAndSetsSuccessStatus`

- **Layer / contract:** D-C12-NO-CLOCK-STATE-IS-SUCCESS-ZERO;
  D-C12-STATUS-WRITE-UNCONDITIONAL.
- **Setup:**
  - Fresh region; `auto shim = make_connected_shim(region, core);`
    (this completes boot+boot_ack but does NOT cache a clock_state —
    `make_connected_shim` only sends boot_ack, never a tick_boundary).
  - `ASSERT_FALSE(shim.latest_clock_state().has_value())`.
  - `shim_global_install_guard guard{shim};`.
  - `int32_t status = 999;`.
- **Action:** `std::uint64_t t = HAL_GetFPGATime(&status);`
- **Expected:**
  - `t == 0`.
  - `status == kHalSuccess` (0).
  - `shim.latest_clock_state().has_value() == false` after the call
    (HAL_GetFPGATime is a pure read; no side effect on the cache).
- **Bug class:** function returns garbage from `optional::value()`
  on a disengaged optional (UB); function returns
  `kHalHandleError` on empty cache (would surface sim-internal
  boot-race state to robot code).

### C12-6. `HalGetFPGATime.WithCachedClockStateReturnsItsSimTimeUsAndSetsSuccessStatus`

- **Layer / contract:** D-C12-LATEST-WINS-READ first read;
  end-to-end protocol-session → cache → C ABI integration.
- **Setup:**
  - Fresh region; `auto shim = make_connected_shim(region, core);`.
  - `shim_global_install_guard guard{shim};`.
  - Drive a clock_state inbound: core peer sends a `tick_boundary` /
    `clock_state` envelope at sim_time 12'345 via the existing
    transport API; `shim.poll()` returns OK; `latest_clock_state()`
    is populated.
  - **Bit-distinct value choice:** sim_time_us = 0xDEAD'BEEF'CAFE'BABE
    (a 64-bit value with no zero bytes; catches "function reads only
    low 32 bits" or "function reads off-by-N bytes" bugs that an
    ordinary small integer would not catch).
  - `int32_t status = 999;`.
- **Action:** `std::uint64_t t = HAL_GetFPGATime(&status);`
- **Expected:**
  - `t == 0xDEAD'BEEF'CAFE'BABE`.
  - `status == kHalSuccess`.
- **Bug class:** function reads the wrong cache slot (would read
  power_state's `vin_v` floats reinterpreted as bytes, etc.); function
  reads an aliased copy that drifted (e.g. shadow variable updated
  only on poll); function reads only the low 32 bits of sim_time_us;
  field-misalignment bug in the observer accessor.

### C12-7. `HalGetFPGATime.LatestWinsAcrossTwoUpdatesReturnsTheMostRecentSimTimeUs`

- **Layer / contract:** D-C12-LATEST-WINS-READ on a sequenced
  update; pins both "reads live shim state at call time" and
  "function does not cache its own copy on first read" by
  **interleaving HAL_GetFPGATime calls between the two polls**
  (round-1 finding: the original single-call-after-both-polls form
  did not actually catch the "caches on first call" bug class).
- **Setup:**
  - Fresh region; shim made; `shim_global_install_guard guard{shim};`.
- **Action sequence:**
  1. Drive clock_state(sim_time_us=100'000) via core→shim, poll, OK.
  2. `int32_t status1 = 999; auto t1 = HAL_GetFPGATime(&status1);`
  3. Drive clock_state(sim_time_us=200'000) via core→shim, poll, OK.
  4. `int32_t status2 = 888; auto t2 = HAL_GetFPGATime(&status2);`
- **Expected:**
  - After step 2: `t1 == 100'000`; `status1 == kHalSuccess`.
  - After step 4: `t2 == 200'000`; `status2 == kHalSuccess`.
- **Bug class:** HAL_GetFPGATime caches its own copy of sim_time_us
  on first read (would show `t2 == 100'000` because the cached
  first-call value never updates); HAL_GetFPGATime reads the
  first-written cache slot rather than the latest; the global pointer
  is captured as a snapshot at install time rather than dereferenced
  at call time (would show `t2 == 100'000` for the same reason);
  the call-time read path reads from a stale shadow of
  `latest_clock_state_`.

### C12-8. `HalGetFPGATime.MultipleCallsWithoutInterveningPollReturnTheSameValue`

- **Layer / contract:** HAL_GetFPGATime is idempotent with respect
  to non-poll-driven shim state changes (D-C12-LATEST-WINS-READ
  consequence: no internal side effect on the cache).
- **Setup:** identical to C12-6 (cache populated with sim_time_us =
  0xDEAD'BEEF'CAFE'BABE).
- **Action:**
  ```cpp
  int32_t status1 = 999, status2 = 888, status3 = 777;
  auto t1 = HAL_GetFPGATime(&status1);
  auto t2 = HAL_GetFPGATime(&status2);
  auto t3 = HAL_GetFPGATime(&status3);
  ```
- **Expected:**
  - `t1 == t2 && t2 == t3 && t1 == 0xDEAD'BEEF'CAFE'BABE`.
  - `status1 == status2 && status2 == status3 &&
     status1 == kHalSuccess`.
- **Bug class:** HAL_GetFPGATime mutates the shim's cache (would
  cause subsequent reads to return zero or stale data); function has
  internal state that drifts across calls (e.g. a "first-call-only"
  branch).

---

## Tests deliberately not added (and why)

- **Determinism replay across two runs.** Cycle 8 already pins
  `latest_clock_state_` byte-identity; the C ABI is a thin wrapper
  with no new nondeterminism class. Adding a HAL_GetFPGATime-specific
  determinism test would be coverage padding.
- **NULL status pointer guard.** WPILib's HAL_GetFPGATime
  unconditionally writes through `int32_t* status`; passing NULL is
  caller-side UB. Adding a "doesn't crash on NULL" test would tie
  us to a contract WPILib doesn't honor.
- **Concurrency / threading.** Single-threaded v0; the threading
  cycle owns its own test plan.
- **HAL_Initialize / HAL_Shutdown semantics.** Future cycle.
- **Re-installing a different shim.** The C12-2 / C12-3 set covers
  the install/clear paths. A "swap from shim A to shim B" test
  duplicates C12-2 with extra setup.
- **Cross-process global isolation.** v0 single-process; the
  shared-memory region is the cross-process boundary, not the C
  HAL global.
- **Post-shutdown HAL_GetFPGATime.** The cache observer continues
  to return the last-cached value after `shutdown` is observed,
  which is the correct behavior for robot code reading the clock
  as it shuts down. The cycle-1 shutdown semantic is on
  `poll()` and outbound `send_*`, not on cache-read observers.

---

## Cross-test invariants

- Tests live in `shim_core_test.cpp` under existing namespace and
  using-decls. No new test file.
- Tests that exercise HAL_* via the install path (C12-5 through
  C12-8) use `shim_global_install_guard` (RAII) so a failed
  assertion does not leak the global into the next test.
- Tests that exercise `install_global` / `current` directly (C12-1,
  C12-2, C12-3) **do not use the RAII guard**, because they are
  testing the install/current surface itself; using the guard would
  test-the-fixture rather than test-the-surface. They use explicit
  `shim_core::install_global(nullptr)` calls to pre-clear and
  post-clear the global so the suite is order-independent.
- Tests use `int32_t status = <sentinel>` (e.g. 999, 888, 777)
  rather than default-initialized status, so the "function failed
  to write status" bug class is detectable per call-site.
- **Shim usage per test:**
  - C12-1: no shim needed (pure null-state assertion).
  - C12-2 / C12-3: use `make_connected_shim` to obtain a valid
    `shim_core&` to take the address of and round-trip through the
    install/current accessors. The shim being "connected" or not is
    irrelevant; the helper is just the standard way to construct a
    valid shim_core in tests.
  - C12-4: no shim installed (the test point).
  - C12-5: shim made via `make_connected_shim` + RAII guard; cache
    is empty by precondition.
  - C12-6 / C12-7 / C12-8: shim made via `make_connected_shim` +
    RAII guard + inbound clock_state(s) driven via core peer.

---

## Implementation companion recommendations (non-test)

- **`hal_c.cpp`** is a **separate translation unit** from
  `shim_core.cpp`. Reasons: (a) the file-static `g_installed_shim_`
  storage is owned by the C HAL surface, not by any one shim
  instance — future cycles add HAL_Initialize / HAL_Shutdown that
  read the same static; (b) the `extern "C"` symbols are easier
  to audit when grouped in their own file; (c) the LD_PRELOAD
  cycle eventually compiles `hal_c.cpp` (and its siblings) into
  `libhal.so` while `shim_core.cpp` stays in
  `libshim_core.so` or similar — separating now avoids a refactor.
- **`install_global` / `current` as static methods on `shim_core`**
  rather than free functions in `robosim::backend::shim::`,
  **declared in `shim_core.h` but defined in `hal_c.cpp`**. The
  storage `static shim_core* g_installed_shim_` is a TU-static
  inside `hal_c.cpp`; the static methods are thin accessors over
  it. Co-locating definition with storage keeps the
  "who-touches-what" boundary clean.
- **Doc comment on `HAL_GetFPGATime`** mirroring the WPILib HAL
  header phrasing: "Reads the microsecond-resolution timer that
  starts when the FPGA is initialized." For our shim, this is the
  most-recently-cached `clock_state::sim_time_us`.
- **Header `hal_c.h`** does NOT include `shim_core.h` — only
  `<cstdint>` and the `extern "C"` declaration. The `.cpp` includes
  `shim_core.h` to access `current()`. This minimizes the header's
  include surface for the future LD_PRELOAD compilation.
- **Status code constants** (`kHalSuccess`, `kHalHandleError`)
  live in `hal_c.h` inside the `robosim::backend::shim` namespace
  so they are usable from C++ tests; the `extern "C"` block carries
  only the `HAL_*` declarations.

---

## Open questions

**OQ-C12-INSTALL-AS-METHOD-VS-FREE-FN.** Should `install_global` /
`current` be static methods on `shim_core`, or free functions in
`robosim::backend::shim`? The plan picks **static methods** for
discoverability and lifetime locality. Reviewer push-back welcome.

**OQ-C12-NO-CLOCK-STATE-DECISION.** Should `HAL_GetFPGATime` return
0 with `kHalSuccess` (the plan's choice) or `kHalHandleError` when
the cache is empty? Plan picks success because the sim-authoritative
model is "sim_time = 0 at boot" and WPILib's contract is
"HAL_GetFPGATime always succeeds". Reviewer welcome to argue for
error semantics.

**OQ-C12-NULL-STATUS-GUARD.** Should the function tolerate `status
== nullptr`? Plan picks UB-on-NULL to match WPILib. Reviewer welcome
to argue for a defensive guard (the cost is one branch per call;
the benefit is friendlier behavior under buggy robot code).

**OQ-C12-RAII-GUARD-LOCATION.** Should `shim_global_install_guard`
live in `tests/backend/tier1/test_helpers.h` (the plan's choice) or
in `src/backend/shim/shim_core.h`? Plan picks test-helpers because
production callers (LD_PRELOAD bootstrap, future HAL_Initialize) have
their own non-RAII lifetime models. Reviewer welcome to argue for
production-side placement.

**OQ-C12-STORAGE-LOCATION.** **Resolved at round 1 in favor of
`hal_c.cpp`.** Storage is owned by the C HAL surface (HAL_GetFPGATime
is its only current reader; future HAL_Initialize and other HAL_*
surfaces will share it). Method **declarations** stay on `shim_core`
in `shim_core.h` (for discoverability), but **definitions** live in
`hal_c.cpp` next to the storage they operate on. This matches the
LD_PRELOAD future where `hal_c.cpp` compiles into `libhal.so`.
