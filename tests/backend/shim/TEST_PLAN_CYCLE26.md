# HAL shim core — cycle 26 test plan (Notifier thread priority)

**Status:** implemented. Round-1 `test-reviewer` verdict was
`ready-to-implement`; no plan changes were required.

**Implements:** the twenty-sixth TDD cycle of the HAL shim core. Cycle
26 wires the remaining non-waiting Notifier C HAL function:

```c
HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime,
                                       int32_t priority,
                                       int32_t* status);
```

Official WPILib 2026.2.2 signatures are captured in
`docs/HAL_SHIM_NEXT_SURFACES_REPORT.md`. Cycle 25 intentionally deferred
this function because it is a scheduler/threading policy seam rather
than notifier handle state.

---

## Why this cycle exists

Cycle 25 added per-shim notifier handles and `notifier_state` snapshot
publication. WPILib robot code can also call
`HAL_SetNotifierThreadPriority` during notifier setup. The simulator v0
has no threading model, so there is no real OS thread priority to adjust.
This cycle adds the C ABI symbol and pins the v0 behavior explicitly
instead of leaving the function missing or inventing partial scheduler
semantics.

`HAL_WaitForNotifierAlarm` remains out of scope. It needs alarm-event
queueing from inbound `notifier_alarm_batch` and blocking/wake semantics
that deserve their own cycle.

---

## Contract under test

### New C HAL surface

`src/backend/shim/hal_c.h` adds:

```cpp
HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime,
                                       std::int32_t priority,
                                       std::int32_t* status);
```

### Behavior

- No installed shim: write `kHalHandleError`, return `0`.
- Installed shim: write `kHalSuccess`, return `1`.
- `realTime` and `priority` are accepted but ignored in v0. This
  includes `realTime == 0`, `realTime == 1`, other nonzero `HAL_Bool`
  values, and boundary `priority` values such as `INT32_MIN` and
  `INT32_MAX`.
- The call does not allocate notifiers, mutate existing notifier slots,
  publish `notifier_state`, touch inbound caches, or change any explicit
  flush behavior.

### Out of scope

- OS scheduler interaction, realtime policy, or priority validation.
- Thread creation or synchronization.
- `HAL_WaitForNotifierAlarm`.

---

## Decisions pinned

### New: D-C26-THREAD-PRIORITY-V0-NOOP

With a shim installed, `HAL_SetNotifierThreadPriority` is a deterministic
no-op that returns true and writes `kHalSuccess`. v0 has no thread to
prioritize and no authoritative WPILib/RIO fixture for priority bounds,
so rejecting values would be invented behavior. A future threading cycle
may replace this with scheduler-backed semantics and updated tests.

### New: D-C26-NO-STATE-MUTATION

The priority call is not part of `notifier_state`. It must not allocate,
clean, rename, update, cancel, stop, flush, or otherwise alter notifier
control-plane state.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: `status` is written on every call;
  null `status` remains UB.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim reports
  `kHalHandleError`.

---

## Proposed tests

### C26-1. `HalSetNotifierThreadPriority.WithNoShimInstalledReturnsFalseAndHandleError`

- **Layer / contract:** C HAL no-shim behavior for the Notifier priority
  surface.
- **Setup:** `shim_core::install_global(nullptr)`; status sentinel.
- **Action:** call `HAL_SetNotifierThreadPriority(1, 40, &status)`.
- **Expected:** returns `0`; `status == kHalHandleError`.
- **Bug class:** missing no-shim guard or falsely reports priority
  success without a backend.

### C26-2. `HalSetNotifierThreadPriority.WithInstalledShimReturnsTrueAndSuccess`

- **Layer / contract:** D-C26-THREAD-PRIORITY-V0-NOOP.
- **Setup:** connected shim installed globally; no notifiers allocated;
  assert current notifier snapshot is empty and outbound lane is empty.
- **Action:** call `HAL_SetNotifierThreadPriority(0, 0, &status)`.
- **Expected:** returns `1`; `status == kHalSuccess`; notifier snapshot
  remains empty; outbound lane remains empty.
- **Bug class:** installed path returns false, materializes notifier
  state, or auto-publishes.

### C26-3. `HalSetNotifierThreadPriority.DoesNotMutateExistingNotifierState`

- **Layer / contract:** D-C26-NO-STATE-MUTATION.
- **Setup:** connected shim installed globally; allocate two notifiers;
  name one, update the other, cancel one; capture
  `current_notifier_state()` active-prefix bytes and assert outbound
  lane is empty.
- **Action:** call `HAL_SetNotifierThreadPriority(1, 40, &status)`.
- **Expected:** returns `1`; `status == kHalSuccess`; active-prefix bytes
  of `current_notifier_state()` are unchanged; outbound lane remains
  empty.
- **Bug class:** priority call resets or rewrites notifier slots, clears
  cancellation flags, changes allocation order, or flushes.

### C26-4. `HalSetNotifierThreadPriority.AcceptsBoundaryPriorityInputsInV0`

- **Layer / contract:** D-C26-THREAD-PRIORITY-V0-NOOP boundary inputs.
- **Setup:** connected shim installed globally.
- **Action:** call with `(realTime=1, priority=INT32_MIN)`,
  `(realTime=1, priority=INT32_MAX)`, and
  `(realTime=-1, priority=-123)`, resetting status sentinel between
  calls.
- **Expected:** each call returns `1` and writes `kHalSuccess`.
- **Bug class:** accidental priority range validation or strict
  `HAL_Bool == 0/1` validation contradicting the v0 no-op decision.

### C26-5. `HalSetNotifierThreadPriority.OverwritesStatusOnRepeatedCalls`

- **Layer / contract:** inherited D-C12 status-write rule.
- **Setup:** connected shim installed globally.
- **Action:** call twice with different status sentinels and different
  input values.
- **Expected:** both calls return `1`; both status variables become
  `kHalSuccess`.
- **Bug class:** implementation writes status only on the first call or
  only for one input branch.
