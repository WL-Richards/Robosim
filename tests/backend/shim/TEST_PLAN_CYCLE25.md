# HAL shim core — cycle 25 test plan (Notifier control plane)

**Status:** implemented. Round 1 returned `not-ready`: the reviewer
required explicit null-name coverage, invalid-handle coverage for
update/cancel/stop, no-shim behavior for the void `HAL_CleanNotifier`, a
pre-flush lane-empty assertion in the update test, and clearer wording
for snapshot flush failure semantics. Revision 2 resolved those findings
and received `ready-to-implement`.

**Implements:** the twenty-fifth TDD cycle of the HAL shim core. Cycle
25 starts the `HAL_Notifier*` surface with the smallest coherent slice:
the notifier control plane that owns handles and publishes outbound
`notifier_state` snapshots.

WPILib 2026.2.2 signatures from
`docs/HAL_SHIM_NEXT_SURFACES_REPORT.md`:

```c
HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status);
void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                         const char* name,
                         int32_t* status);
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status);
void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle);
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime,
                             int32_t* status);
void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status);
```

`HAL_SetNotifierThreadPriority` and `HAL_WaitForNotifierAlarm` are
explicitly deferred. The former is a scheduler/threading policy seam;
the latter needs alarm-event queueing and blocking/wake semantics from
inbound `notifier_alarm_batch`.

---

## Why this cycle exists

Cycle 6 added inbound `notifier_state`, cycle 7 added inbound
`notifier_alarm_batch`, and cycle 10 added the outbound
`send_notifier_state` typed publisher. Robot code still cannot create a
notifier or request an alarm through the C HAL ABI. This cycle bridges
that gap by adding per-shim notifier handles and an explicit
`flush_notifier_state(sim_time_us)` call that publishes the current
notifier table through the existing outbound `notifier_state` schema.

The main bug classes are handle `0` accidentally becoming valid, reused
or stale closed handles mutating live state, invalid handles reporting
success, missing active-prefix snapshot publication, stale caller names
or overlong names leaking past `kNotifierNameLen`, stop/cancel/update
flag drift, and full-table behavior.

---

## Contract under test

### New C HAL surface

`src/backend/shim/hal_c.h` adds:

```cpp
typedef std::int32_t HAL_NotifierHandle;

HAL_NotifierHandle HAL_InitializeNotifier(std::int32_t* status);
void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                         const char* name,
                         std::int32_t* status);
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle,
                      std::int32_t* status);
void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle);
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             std::uint64_t triggerTime,
                             std::int32_t* status);
void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             std::int32_t* status);
```

### New `shim_core` surface

```cpp
class shim_core {
 public:
  std::int32_t initialize_notifier() noexcept;
  std::int32_t set_notifier_name(std::int32_t handle,
                                 std::string_view name) noexcept;
  std::int32_t update_notifier_alarm(std::int32_t handle,
                                     std::uint64_t trigger_time_us) noexcept;
  std::int32_t cancel_notifier_alarm(std::int32_t handle) noexcept;
  std::int32_t stop_notifier(std::int32_t handle) noexcept;
  void clean_notifier(std::int32_t handle) noexcept;

  [[nodiscard]] std::expected<void, shim_error> flush_notifier_state(
      std::uint64_t sim_time_us);
  [[nodiscard]] notifier_state current_notifier_state() const noexcept;
};
```

`initialize_notifier` returns a nonzero handle on success, or `0` when
all `kMaxNotifiers == 32` slots are active. The C HAL wrapper maps that
full-table result to `kHalHandleError`.

`current_notifier_state()` returns a compact active-prefix snapshot of
the live notifier table in allocation order. `flush_notifier_state()`
publishes that snapshot through `send_notifier_state`. Empty flushes are
legal and send the 8-byte header-only `notifier_state` snapshot, because
an explicit "no active notifiers" state is meaningful to Sim Core after
the last notifier is cleaned.

### State mapping

Each live notifier maps to one `notifier_slot`:

- `handle`: the nonzero HAL notifier handle.
- `trigger_time_us`: last requested alarm trigger time; `0` after
  initialize, cancel, or stop.
- `alarm_active`: `1` after `HAL_UpdateNotifierAlarm`, `0` after
  initialize, cancel, stop, or clean.
- `canceled`: `0` after initialize or update, `1` after cancel or stop.
- `name`: zero-filled fixed `kNotifierNameLen` byte array. Null names
  become empty strings. Non-null names copy up to `kNotifierNameLen - 1`
  bytes and leave a trailing NUL when truncated.

### Out of scope

- `HAL_SetNotifierThreadPriority`.
- `HAL_WaitForNotifierAlarm`.
- Threading, blocking waits, condition variables, or wall-clock sleeps.
- Inbound alarm delivery from `notifier_alarm_batch` to waits.
- Reusing cleaned handles in v0. Handles are monotonically increasing
  nonzero integers; cleaned handles become invalid.
- Transport auto-flush from the C HAL function bodies. The integrator
  remains responsible for explicit flush cadence, matching cycles 19
  and 20.

---

## Decisions pinned

### New: D-C25-NOTIFIER-HANDLES-IN-SHIM-CORE

Notifier state lives on `shim_core`, not in `hal_c.cpp`. The C HAL seam
looks up `shim_core::current()` and delegates all handle/state mutation
to the installed shim. This matches the lifecycle decision for
`HAL_SendError` and CAN streams.

### New: D-C25-HANDLE-ZERO-INVALID

Notifier handle `0` is invalid. Successful `HAL_InitializeNotifier`
returns a nonzero handle. Invalid, zero, or cleaned handles return
`kHalHandleError` from status-writing C functions.

### New: D-C25-MONOTONIC-NO-REUSE

Cycle 25 does not reuse cleaned handles. Reuse is not needed for v0 and
would weaken stale-handle tests. If the 32-slot table has free capacity,
the next successful allocation gets the next nonzero handle.

### New: D-C25-FULL-TABLE-HANDLE-ERROR

When all 32 notifier slots are active, `HAL_InitializeNotifier` writes
`kHalHandleError` and returns `0`. This is the v0 behavior for the
common WPILib handle-allocation failure shape; no new notifier-specific
status constant is introduced.

### New: D-C25-EXPLICIT-NOTIFIER-FLUSH

C HAL notifier calls mutate the per-shim table only. The integrator
publishes the table by calling `flush_notifier_state(sim_time_us)`.
Unlike error/CAN TX empty flushes, notifier empty flush sends a
header-only `notifier_state` snapshot so Sim Core can observe that all
notifiers were removed.

### New: D-C25-NAME-COPY-TRUNCATES-WITH-NUL

Notifier names are copied into the fixed 64-byte schema field with
zero-fill before copy. Null name pointers are empty. Non-null names copy
at most 63 bytes, preserving one trailing NUL. No truncation status is
reported by v0 because the WPILib `HAL_SetNotifierName` signature has no
documented truncation status.

### New: D-C25-STOP-IS-CANCELLED-IN-STATE

`HAL_StopNotifier` marks the slot as not alarm-active, clears
`trigger_time_us`, and sets `canceled = 1`. The documented wait-wake
effect is deferred to the future wait cycle, but this state shape tells
Sim Core not to fire the alarm.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: status-writing functions write
  `*status` on every call; null `status` remains UB.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim reports
  `kHalHandleError`.
- D-C10 active-prefix `send_notifier_state` serialization and shutdown
  short-circuit.

---

## Proposed tests

### C25-1. `HalInitializeNotifier.WithNoShimInstalledReturnsZeroHandleAndHandleError`

- **Layer / contract:** C HAL notifier allocation; D-C12 no-shim
  behavior and D-C25-HANDLE-ZERO-INVALID.
- **Setup:** `shim_core::install_global(nullptr)`; status sentinel.
- **Action:** call `HAL_InitializeNotifier(&status)`.
- **Expected:** returns `0`; `status == kHalHandleError`.
- **Bug class:** no-shim path returns a plausible live handle or fails
  to write status.

### C25-2. `HalInitializeNotifier.WithInstalledShimReturnsNonzeroHandleAndSuccess`

- **Layer / contract:** D-C25-NOTIFIER-HANDLES-IN-SHIM-CORE.
- **Setup:** connected shim installed globally.
- **Action:** call `HAL_InitializeNotifier(&status)`.
- **Expected:** returns nonzero handle; `status == kHalSuccess`;
  `shim.current_notifier_state()` has `count == 1` with a slot carrying
  that handle, zero trigger, `alarm_active == 0`, `canceled == 0`, and
  zero-filled name.
- **Bug class:** handle allocated outside shim state, handle `0` treated
  as valid, or default slot flags wrong.

### C25-3. `HalInitializeNotifier.AllocatesDistinctHandlesUntilCapacityThenFails`

- **Layer / contract:** D-C25-FULL-TABLE-HANDLE-ERROR and
  `kMaxNotifiers` capacity.
- **Setup:** connected shim installed globally.
- **Action:** call `HAL_InitializeNotifier` 33 times.
- **Expected:** first 32 calls succeed with distinct nonzero handles;
  snapshot `count == 32`; 33rd returns `0` and writes
  `kHalHandleError`; snapshot remains the original 32 slots.
- **Bug class:** off-by-one capacity, duplicate handles, overflow write
  past `notifier_state::slots`, or failed allocation mutates state.

### C25-4. `HalSetNotifierName.UpdatesNameAndTruncatesOverlongInputWithTrailingNul`

- **Layer / contract:** D-C25-NAME-COPY-TRUNCATES-WITH-NUL.
- **Setup:** connected shim installed globally; allocate one notifier.
- **Action:** set a short name, inspect snapshot; then set an overlong
  deterministic 80-byte name and inspect snapshot again.
- **Expected:** both calls write `kHalSuccess`; short name bytes match
  and remaining bytes are zero; overlong name copies exactly 63 bytes,
  byte 63 is `'\0'`, and no stale bytes from the short name remain.
- **Bug class:** no zero-fill before copy, missing NUL terminator, or
  unbounded string copy into the fixed schema field.

### C25-5. `HalSetNotifierName.NullNameClearsPreviousNameToEmpty`

- **Layer / contract:** D-C25-NAME-COPY-TRUNCATES-WITH-NUL null-pointer
  variant.
- **Setup:** connected shim installed globally; allocate one notifier;
  set name `"previous"`.
- **Action:** call `HAL_SetNotifierName(handle, nullptr, &status)`.
- **Expected:** writes `kHalSuccess`; snapshot slot still exists and all
  64 `name` bytes are zero.
- **Bug class:** null name crashes, is treated as handle error, or
  leaves stale bytes from the previous name.

### C25-6. `HalSetNotifierName.InvalidHandleReportsHandleErrorAndDoesNotMutateState`

- **Layer / contract:** D-C25-HANDLE-ZERO-INVALID.
- **Setup:** connected shim installed globally; allocate one notifier
  and set name `"kept"`.
- **Action:** call `HAL_SetNotifierName(0, "bad", &status)` and
  `HAL_SetNotifierName(handle + 999, "bad", &status)`.
- **Expected:** each writes `kHalHandleError`; snapshot remains byte-
  equal to the pre-call snapshot.
- **Bug class:** invalid handle mutates slot 0, creates a new slot, or
  reports success.

### C25-7. `HalUpdateNotifierAlarm.ActivatesAlarmAndFlushPublishesActivePrefix`

- **Layer / contract:** C HAL update path plus D-C25-EXPLICIT-
  NOTIFIER-FLUSH.
- **Setup:** connected shim installed globally; allocate two notifiers;
  name them distinctly; update only the second to trigger time
  `1234567`.
- **Action:** call `shim.flush_notifier_state(50'000)` and receive the
  outbound envelope from the core endpoint.
- **Expected:** update writes `kHalSuccess`; outbound envelope is
  `tick_boundary/notifier_state`, sequence advances after boot, payload
  size is `offsetof(notifier_state, slots) + 2*sizeof(notifier_slot)`,
  and payload bytes equal `shim.current_notifier_state()` active prefix.
  The second slot has `trigger_time_us == 1234567`,
  `alarm_active == 1`, `canceled == 0`; the first remains inactive.
  Immediately before the explicit flush, the outbound lane is still
  empty, proving the C HAL calls did not auto-publish.
- **Bug class:** update not reflected in published state, wrong slot
  mutated, full-size struct sent instead of active prefix, or C HAL call
  auto-publishes unexpectedly before flush.

### C25-8. `HalUpdateNotifierAlarm.InvalidHandleReportsHandleErrorAndDoesNotMutateState`

- **Layer / contract:** D-C25-HANDLE-ZERO-INVALID for update.
- **Setup:** connected shim installed globally; allocate one notifier
  and update it to a known trigger; save snapshot bytes.
- **Action:** call `HAL_UpdateNotifierAlarm(0, 999, &status)` and
  `HAL_UpdateNotifierAlarm(handle + 999, 999, &status)`.
- **Expected:** each writes `kHalHandleError`; snapshot remains byte-
  equal to the pre-call snapshot.
- **Bug class:** invalid update mutates slot 0, creates a new slot, or
  reports success while losing the valid alarm.

### C25-9. `HalCancelNotifierAlarm.ClearsTriggerAndMarksCanceled`

- **Layer / contract:** `HAL_CancelNotifierAlarm` state mutation.
- **Setup:** connected shim installed globally; allocate a notifier;
  update alarm to a nonzero trigger.
- **Action:** call `HAL_CancelNotifierAlarm(handle, &status)`.
- **Expected:** `status == kHalSuccess`; snapshot slot keeps the handle
  but has `trigger_time_us == 0`, `alarm_active == 0`, and
  `canceled == 1`.
- **Bug class:** cancel leaves an alarm active, loses the slot, or does
  not communicate cancellation to Sim Core.

### C25-10. `HalCancelNotifierAlarm.InvalidHandleReportsHandleErrorAndDoesNotMutateState`

- **Layer / contract:** D-C25-HANDLE-ZERO-INVALID for cancel.
- **Setup:** connected shim installed globally; allocate one notifier
  and update it to a known trigger; save snapshot bytes.
- **Action:** call `HAL_CancelNotifierAlarm(0, &status)` and
  `HAL_CancelNotifierAlarm(handle + 999, &status)`.
- **Expected:** each writes `kHalHandleError`; snapshot remains byte-
  equal to the pre-call snapshot.
- **Bug class:** invalid cancel clears the wrong live alarm or reports
  success.

### C25-11. `HalStopNotifier.ClearsTriggerAndMarksCanceled`

- **Layer / contract:** D-C25-STOP-IS-CANCELLED-IN-STATE.
- **Setup:** connected shim installed globally; allocate a notifier;
  update alarm to a nonzero trigger.
- **Action:** call `HAL_StopNotifier(handle, &status)`.
- **Expected:** `status == kHalSuccess`; snapshot slot keeps the handle
  but has `trigger_time_us == 0`, `alarm_active == 0`, and
  `canceled == 1`.
- **Bug class:** stop leaves future alarms fireable or accidentally
  cleans the handle instead of stopping it.

### C25-12. `HalStopNotifier.InvalidHandleReportsHandleErrorAndDoesNotMutateState`

- **Layer / contract:** D-C25-HANDLE-ZERO-INVALID for stop.
- **Setup:** connected shim installed globally; allocate one notifier
  and update it to a known trigger; save snapshot bytes.
- **Action:** call `HAL_StopNotifier(0, &status)` and
  `HAL_StopNotifier(handle + 999, &status)`.
- **Expected:** each writes `kHalHandleError`; snapshot remains byte-
  equal to the pre-call snapshot.
- **Bug class:** invalid stop cancels the wrong live alarm, cleans the
  wrong handle, or reports success.

### C25-13. `HalCleanNotifier.WithNoShimInstalledIsNoOp`

- **Layer / contract:** void C HAL no-shim behavior for clean, matching
  `HAL_CAN_CloseStreamSession`.
- **Setup:** `shim_core::install_global(nullptr)`.
- **Action:** call `HAL_CleanNotifier(1234)`.
- **Expected:** call returns normally and `shim_core::current()` remains
  `nullptr`.
- **Bug class:** void clean dereferences a null global shim or creates
  process-global state.

### C25-14. `HalCleanNotifier.RemovesSlotAndStaleHandleCannotMutateNewState`

- **Layer / contract:** clean lifecycle and D-C25-MONOTONIC-NO-REUSE.
- **Setup:** connected shim installed globally; allocate two notifiers;
  set names `"first"` and `"second"`.
- **Action:** call `HAL_CleanNotifier(first_handle)`; then call
  `HAL_UpdateNotifierAlarm(first_handle, 999, &status)`; then allocate a
  third notifier.
- **Expected:** after clean, snapshot contains only the second notifier;
  stale update writes `kHalHandleError` and leaves snapshot unchanged;
  third allocation succeeds with a handle distinct from the cleaned
  first handle and appends a second active slot.
- **Bug class:** clean is a no-op, stale handles remain valid, or handle
  reuse makes stale callers affect new notifiers.

### C25-15. `ShimCoreFlushNotifierState.EmptyTablePublishesHeaderOnlySnapshot`

- **Layer / contract:** D-C25-EXPLICIT-NOTIFIER-FLUSH empty-state
  behavior.
- **Setup:** connected shim; no notifiers allocated.
- **Action:** call `shim.flush_notifier_state(75'000)` and receive from
  core.
- **Expected:** success; outbound envelope is
  `tick_boundary/notifier_state`, payload size is
  `offsetof(notifier_state, slots) == 8`, and payload bytes equal a
  zero-initialized `notifier_state{}` header prefix.
- **Bug class:** empty notifier table is treated as no-op, preventing
  Sim Core from observing that the table is empty.

### C25-16. `ShimCoreFlushNotifierState.TransportFailureRetainsNotifierStateForRetry`

- **Layer / contract:** snapshot flush failure semantics for the current
  notifier table.
- **Setup:** create shim but leave boot envelope occupying the outbound
  lane; allocate/update notifier through shim-core methods directly or
  after installing the global pointer.
- **Action:** call `flush_notifier_state`, then drain boot and call
  `flush_notifier_state` again.
- **Expected:** first call returns `shim_error_kind::send_failed` with
  `lane_busy` and does not clear/mutate notifier state; retry succeeds
  and publishes sequence `1` with the same active-prefix bytes.
- **Bug class:** failed flush drops notifier state or consumes outbound
  sequence.

### C25-17. `ShimCoreFlushNotifierState.PostShutdownIsRejectedWithoutTouchingLane`

- **Layer / contract:** D-C10 shutdown-terminal send behavior applied
  to the new flush wrapper.
- **Setup:** connected shim; allocate one notifier; poll inbound
  shutdown; assert outbound lane empty.
- **Action:** call `flush_notifier_state(80'000)`.
- **Expected:** returns `shim_error_kind::shutdown_already_observed`
  with no wrapped transport error; outbound lane remains empty;
  snapshot still contains the notifier for diagnostic/retry visibility.
- **Bug class:** flush publishes after terminal shutdown or clears state
  on a rejected send.

### C25-18. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalNotifierStateSnapshots`

- **Layer / contract:** deterministic replay for notifier control-plane
  state and outbound wire bytes.
- **Setup:** run two independent shim/core pairs through the same
  sequence: allocate two notifiers, set names, update one, cancel the
  other, flush, clean the first, flush again.
- **Action:** collect both outbound messages from each run.
- **Expected:** corresponding envelopes and payload vectors are equal;
  `std::memcmp` over each payload also returns zero. The second flush
  is a compact one-slot snapshot, proving cleaned slots are not leaked.
- **Bug class:** uninitialized padding in `notifier_slot`, stale cleaned
  slot bytes, nondeterministic handle assignment, or allocation-order
  drift.

---

## Reviewer prompts

- Is Cycle 25's control-plane-only slice small enough, or should
  `HAL_CleanNotifier` / `HAL_StopNotifier` be split out?
- Is header-only publication for an empty notifier table the right v0
  behavior, given notifier state is a snapshot rather than an append-only
  message buffer?
- Should full-table `HAL_InitializeNotifier` use `kHalHandleError`, or
  is there a more precise WPILib status constant that should be added
  before implementation?
