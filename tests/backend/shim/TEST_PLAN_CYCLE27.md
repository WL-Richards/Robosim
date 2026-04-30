# Cycle 27 test plan — HAL_WaitForNotifierAlarm wake/drain

Status: implemented. Test-reviewer verdicts: first draft `not-ready`
(ordinary empty wait incorrectly returned `0/kHalSuccess`), second draft
`not-ready` (nondeterministic intermediate readiness assertions and missing
cancel-vs-stop behavior), third draft `ready-to-implement`.

## Scope

Cycle 27 adds the remaining Notifier C HAL surface:

```cpp
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status);
```

This is a Layer 2 HAL shim C ABI surface over the already-wired inbound
`notifier_alarm_batch` schema. The far goal is running an unmodified WPILib
robot program, so this cycle must not treat an ordinary "no alarm yet" wait as
`0/kHalSuccess`: in WPILib, `HAL_WaitForNotifierAlarm` waits until an alarm
fires or stop wakes it with timestamp 0.

## v0 behavior decisions

- **D-C27-BLOCKING-WAIT.** A valid active notifier wait blocks until one of
  three events happens:
  - a queued/polled alarm event for that handle is available;
  - `HAL_StopNotifier(handle, status)` wakes the notifier;
  - the handle is cleaned while the wait is pending.
- **D-C27-FIFO-QUEUE.** Inbound `notifier_alarm_batch` events are appended to a
  per-shim pending alarm queue during `shim_core::poll()`. Wait drains from
  that queue.
- **D-C27-HANDLE-FILTER.** `HAL_WaitForNotifierAlarm(handle, status)` returns
  the first queued event whose `event.handle == handle`. Events for other
  notifier handles remain queued and keep their relative order.
- **D-C27-STOP-WAKE.** `HAL_StopNotifier` wakes waits for that handle. A wait
  woken by stop returns `0` with `kHalSuccess`, matching the documented WPILib
  stop wake shape. Cycle 27 does not model a separate "canceled but not
  stopped" wake reason.
- **D-C27-CANCEL-DOES-NOT-WAKE.** `HAL_CancelNotifierAlarm` updates the
  notifier control-plane state but does not complete a pending wait. A later
  matching alarm event or stop/clean still completes the wait.
- **D-C27-CLEAN-WAKE.** `HAL_CleanNotifier` wakes waits for that handle. A wait
  woken by clean returns `0` with `kHalHandleError` because the handle is no
  longer active.
- **D-C27-LATEST-CACHE-UNCHANGED.** The existing
  `latest_notifier_alarm_batch()` observer remains latest-wins and
  byte-identical to the most recently polled batch. Draining the wait queue does
  not mutate the observer cache.
- **D-C27-INVALID-HANDLE.** No installed shim or an inactive/cleaned/zero handle
  writes `kHalHandleError` and returns `0` without draining any queued event.
- **D-C27-STATUS-WRITE.** `status` is overwritten on every return. Null
  `status` remains WPILib-style UB, matching prior C HAL surfaces.
- **D-C27-NO-CAPACITY-PIN.** Queue capacity and overflow policy are not pinned
  in Cycle 27. This avoids baking `kMaxAlarmsPerBatch` protocol-batch size into
  HAL wait behavior. A future stress cycle can add a deliberate capacity policy
  if needed.
- **D-C27-THREAD-SAFETY.** Cycle 27 is the first shim cycle where C HAL calls
  and `shim_core::poll()` may operate concurrently. Access to notifier records,
  stop/clean wake state, and pending alarm events must be synchronized so the
  wait, poll, stop, clean, and cancel paths are data-race-free.

## Proposed tests

### C27-1. `HalWaitForNotifierAlarm.WithNoShimInstalledReturnsZeroAndHandleError`

- Layer / contract: Layer 2 C HAL global-accessor/no-shim behavior.
- Bug class caught: missing no-shim guard, false success, or stale status.
- Inputs:
  - Clear `shim_core::install_global(nullptr)`.
  - Call `HAL_WaitForNotifierAlarm(1, &status)` with sentinel status.
- Expected:
  - Returns `0`.
  - Writes `kHalHandleError`.
- Determinism: no time, no threads.

### C27-2. `HalWaitForNotifierAlarm.ReturnsAlreadyQueuedMatchingAlarmWithoutBlocking`

- Layer / contract: D-C27-FIFO-QUEUE, D-C27-HANDLE-FILTER, and
  D-C27-STATUS-WRITE.
- Bug class caught: wait ignores queued events, returns unrelated handles, or
  leaves status stale.
- Inputs:
  - Connected shim installed globally.
  - Allocate two notifiers: `first` and `second`.
  - Poll one inbound `notifier_alarm_batch` with events:
    - `(fired_at_us=10'000, handle=second)`
    - `(fired_at_us=20'000, handle=first)`
    - `(fired_at_us=30'000, handle=first)`
  - Call wait for `first` twice, then for `second` once.
- Expected:
  - First wait for `first` returns `20'000`, status `kHalSuccess`.
  - Second wait for `first` returns `30'000`, status `kHalSuccess`.
  - Wait for `second` returns `10'000`, status `kHalSuccess`.
  - Each call pre-fills status with a distinct sentinel so overwrite is visible.
- Determinism: fixed event order, single-threaded poll before waits.

### C27-3. `HalWaitForNotifierAlarm.BlocksUntilMatchingAlarmIsPolled`

- Layer / contract: D-C27-BLOCKING-WAIT and D-C27-HANDLE-FILTER.
- Bug class caught: ordinary empty wait returns `0/kHalSuccess`, wait wakes for
  unrelated handles, or poll does not notify blocked waiters.
- Inputs:
  - Connected shim installed globally.
  - Allocate `first` and `second`.
  - Start one waiter thread calling `HAL_WaitForNotifierAlarm(first, &status)`.
  - Use a `std::latch`/promise only to start the waiter and main-thread actions
    in a deterministic order; do not assert intermediate scheduler readiness.
  - Poll a batch containing only an event for `second`.
  - Poll a batch containing `(fired_at_us=50'000, handle=first)`.
- Expected:
  - Waiter returns `50'000`.
  - Waiter status is `kHalSuccess`.
  - The `second` event remains available to a later wait for `second`.
- Determinism:
  - No wall-clock sleeps and no "future not ready yet" assertions. A bounded
    future wait is allowed only as a failure guard after the matching event is
    polled so a broken implementation cannot hang the suite forever.

### C27-4. `HalWaitForNotifierAlarm.AccumulatesAcrossMultiplePolledBatches`

- Layer / contract: D-C27-FIFO-QUEUE append semantics.
- Bug class caught: replacing the drain queue on every inbound batch, clearing
  unread events, or queueing only the latest batch.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Poll a batch with events at `100` and `200` for that handle.
  - Drain once; expected `100`.
  - Poll an empty `notifier_alarm_batch`.
  - Poll a second batch with events at `300` and `400` for that handle.
  - Drain the remaining three already-queued events.
- Expected:
  - Drain sequence after the second poll is `200`, `300`, `400`.
  - The empty inbound batch updates `latest_notifier_alarm_batch()` but does
    not clear the pending wait queue.
  - All calls write `kHalSuccess`.
- Determinism: fixed event order, no wall-clock.

### C27-5. `HalWaitForNotifierAlarm.DrainDoesNotMutateLatestAlarmBatchCache`

- Layer / contract: D-C27-LATEST-CACHE-UNCHANGED.
- Bug class caught: implementing the wait queue by destructively editing
  `latest_notifier_alarm_batch_`, or zeroing the observer cache after a wait.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Poll a batch with two events for that handle.
  - Save the byte image of `*shim.latest_notifier_alarm_batch()`.
  - Drain both events with `HAL_WaitForNotifierAlarm`.
- Expected:
  - Both waits return the two timestamps with `kHalSuccess`.
  - `latest_notifier_alarm_batch()` is still present and byte-identical to the
    saved batch over `sizeof(notifier_alarm_batch)`.
- Determinism: byte comparison over a zero-initialized schema.

### C27-6. `HalWaitForNotifierAlarm.InvalidOrCleanedHandleReturnsZeroHandleErrorWithoutDrainingOthers`

- Layer / contract: D-C27-INVALID-HANDLE and D-C27-CLEAN-WAKE stale-event
  handling.
- Bug class caught: accepting stale handles, returning queued events for a
  cleaned handle, draining before handle validation, or dropping unrelated
  queued events on invalid calls.
- Inputs:
  - Connected shim installed globally.
  - Allocate `live` and `cleaned` notifiers.
  - Poll a batch with:
    - `(fired_at_us=11'000, handle=cleaned)`
    - `(fired_at_us=22'000, handle=live)`
  - `HAL_CleanNotifier(cleaned)`.
  - Call wait with `0`, with `cleaned`, and with a never-allocated handle.
  - Then call wait with `live`.
- Expected:
  - The invalid calls return `0` and write `kHalHandleError`.
  - Waiting on `cleaned` does not return stale timestamp `11'000`.
  - The live call still returns `22'000` and writes `kHalSuccess`.
- Determinism: fixed handle lifecycle.

### C27-7. `HalWaitForNotifierAlarm.CancelDoesNotWakePendingWait`

- Layer / contract: D-C27-CANCEL-DOES-NOT-WAKE.
- Bug class caught: accidentally implementing cancel as stop, completing wait
  with `0`, or invalidating a still-active notifier.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Start one waiter thread calling `HAL_WaitForNotifierAlarm(handle, &status)`.
  - Use a `std::latch`/promise only to order the test actions; do not assert
    intermediate blocked/not-ready state.
  - Call `HAL_CancelNotifierAlarm(handle, &cancel_status)`.
  - Poll a batch containing `(fired_at_us=70'000, handle=handle)`.
- Expected:
  - Cancel writes `kHalSuccess`.
  - Waiter returns `70'000`, not `0`.
  - Waiter writes `kHalSuccess`.
  - Current notifier state remains the Cycle 25 canceled shape until the polled
    event is consumed; Cycle 27 does not require the inbound event to reactivate
    the control-plane slot.
- Determinism:
  - No sleeps and no intermediate readiness assertion. A bounded future wait is
    a failure guard only after the matching event is polled.

### C27-8. `HalWaitForNotifierAlarm.StopNotifierWakesPendingWaitWithZeroSuccess`

- Layer / contract: D-C27-STOP-WAKE.
- Bug class caught: stopped active handles treated as invalid, stop fails to
  notify blocked waiters, or stop returns a stale queued timestamp instead of
  the documented stop wake value.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Start one waiter thread calling `HAL_WaitForNotifierAlarm(handle, &status)`.
  - Use a `std::latch`/promise only to order the test actions; do not assert
    intermediate blocked/not-ready state.
  - Call `HAL_StopNotifier(handle, &stop_status)`.
- Expected:
  - Stop writes `kHalSuccess`.
  - Wait returns `0` and writes `kHalSuccess`.
  - Current notifier state still shows the stopped/canceled slot per Cycle 25.
- Determinism:
  - No sleeps and no intermediate readiness assertion. The waiter thread is
    joined via bounded future wait only as a failure guard.

### C27-9. `HalWaitForNotifierAlarm.CleanNotifierWakesPendingWaitWithHandleError`

- Layer / contract: D-C27-CLEAN-WAKE.
- Bug class caught: `HAL_CleanNotifier` leaves robot Notifier threads blocked,
  or a cleaned handle still reports success.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Start one waiter thread calling `HAL_WaitForNotifierAlarm(handle, &status)`.
  - Use a `std::latch`/promise only to order the test actions; do not assert
    intermediate blocked/not-ready state.
  - Call `HAL_CleanNotifier(handle)`.
- Expected:
  - Wait returns `0`.
  - Wait writes `kHalHandleError`.
  - `shim.current_notifier_state().count == 0`.
- Determinism:
  - No sleeps and no intermediate readiness assertion. Bounded future wait is a
    failure guard only.

### C27-10. `HalWaitForNotifierAlarm.OverwritesStatusOnRepeatedMixedReturns`

- Layer / contract: D-C27-STATUS-WRITE.
- Bug class caught: status only written on success, only written on error, or
  stale status surviving repeated calls.
- Inputs:
  - Connected shim installed globally.
  - Allocate one notifier.
  - Poll one alarm for the valid handle.
  - Call wait on the valid handle with sentinel status.
  - Call wait on invalid handle with a different sentinel status.
- Expected:
  - Valid call returns the queued timestamp, status `kHalSuccess`.
  - Invalid call returns `0`, status `kHalHandleError`.
- Determinism: valid call is pre-queued; invalid call does not block.

## Explicitly deferred

- Full Notifier scheduler generation of alarm events. Cycle 27 consumes
  already-valid inbound `notifier_alarm_batch` events from the Sim Core.
- Coupling waits to `HAL_SetNotifierThreadPriority`.
- Queue capacity/overflow policy.
- Multiple simultaneous waiters on the same handle. This cycle supports the
  single-waiter-per-notifier shape used by WPILib Notifier ownership; a future
  threading stress cycle can pin multi-waiter behavior if needed.

## Review notes

Related files:

- `.claude/skills/hal-shim.md`
- `src/backend/common/notifier_state.h`
- `src/backend/shim/shim_core.{h,cpp}`
- `src/backend/shim/hal_c.{h,cpp}`
- `tests/backend/shim/shim_core_test.cpp`
- `tests/backend/tier1/test_helpers.h`
