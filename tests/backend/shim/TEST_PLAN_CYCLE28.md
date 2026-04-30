# Cycle 28 test plan — HAL_Initialize / HAL_Shutdown lifecycle

Status: implemented. Test-reviewer verdicts: first draft `not-ready`
(shutdown wait wake could false-positive and repeated initialize did not catch
notifier allocator reset), second draft `ready-to-implement`.

## Scope

Cycle 28 adds the WPILib HALBase lifecycle C ABI surface:

```cpp
HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode);
void HAL_Shutdown(void);
```

The far goal is running an unmodified WPILib robot program. Real robot startup
calls `HAL_Initialize` before most other HAL calls, and shutdown must not leave
HAL-owned waits blocked. This cycle pins lifecycle behavior without making
`HAL_Initialize` own Tier 1 transport construction; in the current architecture
the host process still creates `shim_core` and installs it with
`shim_core::install_global`.

## v0 behavior decisions

- **D-C28-HOST-OWNS-SHIM.** `HAL_Initialize(timeout, mode)` does not create or
  boot a `shim_core`. It succeeds only if the host has already installed a shim
  with `shim_core::install_global(&shim)`.
- **D-C28-NO-SHIM-INIT-FAILS.** With no installed shim,
  `HAL_Initialize` returns false. There is no status out-param for this API.
- **D-C28-IDEMPOTENT-INIT.** With an installed shim, `HAL_Initialize` returns
  true. Repeated calls return true and do not re-send boot, mutate caches,
  allocate notifiers, publish outbound traffic, or reset existing shim state.
- **D-C28-IGNORE-TIMEOUT-MODE.** v0 accepts every `timeout` and `mode` value,
  including `INT32_MIN` and `INT32_MAX`, because this shim has no native HAL
  startup timeout or runtime mode model yet. The installed-shim/no-shim result
  is independent of those arguments.
- **D-C28-SHUTDOWN-DETACHES.** `HAL_Shutdown()` is a no-op with no installed
  shim. With an installed shim, it wakes HAL waiters owned by the shim and then
  clears the process-global current shim pointer. The shim object remains
  caller-owned; `HAL_Shutdown` does not destroy it.
- **D-C28-SHUTDOWN-WAKES-NOTIFIER-WAITS.** Pending
  `HAL_WaitForNotifierAlarm` calls wake with timestamp `0` and
  `kHalHandleError` when `HAL_Shutdown` detaches the shim. This prevents an
  unmodified robot program's Notifier thread from being stranded during process
  shutdown.
- **D-C28-WAITER-OBSERVER.** `shim_core` exposes a small diagnostic observer
  for pending notifier wait counts, both total and filtered by handle. This is
  not a HAL ABI surface; it gives tests and future harnesses a deterministic
  way to prove waiters are registered before exercising stop/clean/shutdown
  wake paths, without sleeps or scheduler-timing assertions.
- **D-C28-REINSTALL-AFTER-SHUTDOWN.** Because host code owns shim lifetime,
  tests may call `shim_core::install_global(&shim)` again after shutdown.
  `HAL_Initialize` then succeeds again against that installed shim. Cycle 28
  does not add a persistent "poisoned after shutdown" process state.
- **D-C28-INIT-REENTRANT-READ.** Concurrent `HAL_Initialize` calls with an
  already installed shim all return true and do not mutate shim state. Cycle 28
  does not make concurrent install/clear of the process-global pointer safe;
  broader global-accessor threading remains deferred.

## Proposed tests

### C28-1. `HalInitialize.WithNoShimInstalledReturnsFalse`

- Layer / contract: Layer 2 C HAL lifecycle no-shim behavior,
  D-C28-NO-SHIM-INIT-FAILS.
- Bug class caught: returning success without a usable shim, which would let an
  unmodified robot program proceed against disconnected HAL state.
- Inputs:
  - Clear `shim_core::install_global(nullptr)`.
  - Call `HAL_Initialize(500, 0)`.
- Expected:
  - Returns `0`.
  - `shim_core::current()` remains `nullptr`.
- Determinism: no transport traffic.

### C28-2. `HalInitialize.WithInstalledShimReturnsTrueAndDoesNotPublish`

- Layer / contract: D-C28-HOST-OWNS-SHIM and D-C28-IDEMPOTENT-INIT.
- Bug class caught: initialize tries to construct/send a second boot envelope,
  mutates shim caches, or clears the installed pointer.
- Inputs:
  - Create a connected shim and install it globally.
  - The connected-shim helper has already consumed the initial construction
    boot envelope and accepted `boot_ack`, leaving backend-to-core empty.
  - Ensure backend-to-core lane is empty.
  - Call `HAL_Initialize(500, 0)`.
- Expected:
  - Returns `1`.
  - `shim_core::current()` still points to the same shim.
  - Backend-to-core lane stays empty.
- Determinism: no wall-clock timeout.

### C28-3. `HalInitialize.RepeatedCallsAreIdempotentAndPreserveState`

- Layer / contract: D-C28-IDEMPOTENT-INIT.
- Bug class caught: repeated initialize resets notifier handle allocation,
  clears cached state, or reboots the protocol session.
- Inputs:
  - Create a connected shim and install it globally.
  - Poll one `clock_state`.
  - Allocate one notifier and set its name/alarm.
  - Save `current_notifier_state()` and `latest_clock_state()`.
  - Call `HAL_Initialize(0, 0)` twice.
- Expected:
  - Both calls return `1`.
  - Saved clock cache and notifier state remain byte-identical.
  - Allocating a second notifier after the repeated initialize calls returns a
    distinct monotonic handle greater than the first, and the original notifier
    remains present.
  - No outbound lane publish occurs.
- Determinism: fixed inputs, no timeout behavior.

### C28-4. `HalInitialize.AcceptsBoundaryTimeoutAndModeInputs`

- Layer / contract: D-C28-IGNORE-TIMEOUT-MODE.
- Bug class caught: accidental timeout/mode validation that rejects valid robot
  startup calls before this shim models those arguments.
- Inputs:
  - Installed shim.
  - Call `HAL_Initialize(INT32_MIN, INT32_MIN)` and
    `HAL_Initialize(INT32_MAX, INT32_MAX)`.
  - Clear global shim.
  - Call the same boundary inputs with no installed shim.
- Expected:
  - Installed calls return `1`.
  - No-shim calls return `0`.
  - Argument values do not affect the installed/no-shim result.
- Determinism: no waits or transport traffic.

### C28-5. `HalShutdown.WithNoShimInstalledIsNoOp`

- Layer / contract: D-C28-SHUTDOWN-DETACHES no-shim branch.
- Bug class caught: shutdown assumes a non-null shim and crashes or changes
  unrelated process state.
- Inputs:
  - Clear `shim_core::install_global(nullptr)`.
  - Call `HAL_Shutdown()` twice.
- Expected:
  - No crash.
  - `shim_core::current()` remains `nullptr`.
- Determinism: no transport traffic.

### C28-6. `HalShutdown.DetachesInstalledShimAndSubsequentHalReadsSeeNoShim`

- Layer / contract: D-C28-SHUTDOWN-DETACHES.
- Bug class caught: shutdown does not clear the C HAL global pointer, or clears
  it before existing HAL readers can observe no-shim behavior afterward.
- Inputs:
  - Create a connected shim, install globally, and call `HAL_Initialize`.
  - Poll one `clock_state` and allocate one notifier before shutdown.
  - Call `HAL_Shutdown()`.
  - Call `HAL_GetFPGATime(&status)` after shutdown.
- Expected:
  - `shim_core::current()` is `nullptr` after shutdown.
  - `HAL_GetFPGATime` returns `0` and writes `kHalHandleError`.
  - Direct inspection of the caller-owned `shim` still shows the previously
    cached `clock_state` and notifier slot, proving shutdown did not destroy or
    clear shim-owned state.
- Determinism: direct C HAL calls only.

### C28-7. `HalShutdown.WakesAllPendingNotifierWaitsWithHandleError`

- Layer / contract: D-C28-SHUTDOWN-WAKES-NOTIFIER-WAITS.
- Bug class caught: shutdown detaches the shim while a Notifier wait remains
  blocked forever, wakes only one waiter, or wakes with the stop-success value
  instead of shutdown error.
- Inputs:
  - Create a connected shim, install globally, and initialize.
  - Allocate two notifiers.
  - Start two waiter threads, one per handle, each calling
    `HAL_WaitForNotifierAlarm(handle, &status)`.
  - Use the public pending-wait observer to wait until it reports two total
    waiters and one waiter per handle. Use a bounded spin with
    `std::this_thread::yield()` only; no sleeps and no future-readiness timing.
  - Call `HAL_Shutdown()`.
- Expected:
  - Both waiters return timestamp `0`.
  - Both waiters write `kHalHandleError`.
  - `shim_core::current()` is `nullptr`.
  - Pending waiter count is zero after both waiters return.
- Determinism:
  - No sleeps. A bounded future wait is allowed only as a hang guard after
    shutdown.

### C28-8. `HalInitialize.CanSucceedAgainAfterHostReinstallsShim`

- Layer / contract: D-C28-REINSTALL-AFTER-SHUTDOWN.
- Bug class caught: shutdown leaves a persistent process poison flag that
  prevents a host-managed test or future harness from reinstalling a shim.
- Inputs:
  - Create a connected shim and install globally.
  - Call `HAL_Initialize`, then `HAL_Shutdown`.
  - Reinstall the same caller-owned shim with `shim_core::install_global(&shim)`.
  - Call `HAL_Initialize` again.
- Expected:
  - First initialize returns `1`.
  - Shutdown clears current pointer.
  - Reinstalled initialize returns `1`.
  - Current pointer is the reinstalled shim.
- Determinism: no transport traffic after initial connection.

### C28-9. `HalInitialize.ConcurrentCallsAllReturnTrueWithoutMutatingState`

- Layer / contract: D-C28-INIT-REENTRANT-READ.
- Bug class caught: initialize uses mutable one-shot process state, races on
  repeated init calls, or publishes during concurrent robot startup calls.
- Inputs:
  - Create a connected shim and install globally.
  - Poll one `clock_state` and allocate one notifier.
  - Launch several async tasks that all wait on a shared start future and then
    call `HAL_Initialize(500, 0)`.
  - Join every task.
- Expected:
  - Every call returns `1`.
  - `shim_core::current()` still points to the same shim.
  - Cached clock state and notifier state remain byte-identical.
  - Backend-to-core lane remains empty.
- Determinism:
  - No sleeps. Concurrency is limited to concurrent reads of an already
    installed global pointer and no mutable lifecycle state.

## Explicitly deferred

- Creating/owning `shim_core` inside `HAL_Initialize`.
- Parsing runtime configuration or shared-memory names from environment.
- Persistent process-wide initialized/shutdown state beyond the installed
  pointer.
- Gating every other HAL surface on successful `HAL_Initialize`; existing
  surfaces continue to use the installed-shim guard until a broader lifecycle
  compatibility cycle intentionally changes that contract.
- Concurrent `shim_core::install_global` calls or install/clear races.

## Review notes

Related files:

- `.claude/skills/hal-shim.md`
- `src/backend/shim/shim_core.{h,cpp}`
- `src/backend/shim/hal_c.{h,cpp}`
- `tests/backend/shim/shim_core_test.cpp`
