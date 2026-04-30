# HAL shim core - cycle 33 test plan (DS new-data event handles)

Status: implemented

Cycle 33 implements the Driver Station new-data event-handle registration
surface:

```cpp
void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle);
void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle);
```

Primary WPILib references:

- `hal/DriverStation.h` release docs, generated 2026-02-27:
  `HAL_ProvideNewDataEventHandle` "Adds an event handle to be signalled when
  new data arrives" and `HAL_RemoveNewDataEventHandle` "Removes the event
  handle from being signalled when new data arrives."
- `wpi/Synchronization.h` release docs: `WPI_Handle` is `unsigned int`, and
  `WPI_EventHandle` is a `WPI_Handle`; invalid handle is `0`.
- WPILib sim `DriverStation.cpp` stores handles in `wpi::EventVector` and
  calls `Wakeup()` when new Driver Station data arrives.

## Slice

This cycle only registers/removes DS new-data event handles and wakes them
when the shim accepts a new `ds_state` packet. It does not implement
`HAL_GetOutputsEnabled`, user-program observer calls, joystick outputs/rumble,
or a background receive thread. A host-driven `poll()` or
`HAL_RefreshDSData()` call is still what accepts inbound protocol messages in
v0.

## Decisions

- **D-C33-WPI-HANDLE-TYPE:** `WPI_Handle` and `WPI_EventHandle` are mirrored as
  `unsigned int`, matching WPILib's C ABI.
- **D-C33-WPI-SET-EVENT:** Signalling uses the real WPI signal-object seam:
  call `WPI_SetEvent(handle)` for each registered handle. The symbol is a weak
  external in this repo so unit tests and non-WPI hosts can link without
  wpiutil; if it is absent, signalling is a no-op.
- **D-C33-NO-SHIM:** Provide/remove with no installed shim are no-ops.
- **D-C33-INVALID-HANDLE:** Handle `0` is invalid and ignored on provide and
  remove.
- **D-C33-REGISTRATION:** Registration is per shim object. v0 stores unique
  nonzero handles; providing the same handle twice does not create duplicate
  wakeups. Removal removes the handle if present and is otherwise a no-op.
- **D-C33-WAKE-ON-DS:** Accepting a valid `tick_boundary` `ds_state` wakes all
  currently registered handles. This applies to both direct `shim_core::poll()`
  and the C `HAL_RefreshDSData()` path because they share the same dispatch
  function.
- **D-C33-NO-WAKE-NON-DS:** No-message, valid non-DS packets, invalid/removed
  handles, no-shim paths, and shutdown do not signal handles.
- **D-C33-SHUTDOWN:** After shutdown is observed, later inbound packets are not
  dispatched and do not signal handles.

## Proposed tests

### C33-1 - WPI event handle type matches WPILib unsigned handle ABI

- **Layer / contract:** C ABI type mirror for `WPI_Handle` and
  `WPI_EventHandle`.
- **Bug class caught:** signedness/width drift that would corrupt opaque WPI
  handles across the HAL boundary.
- **Inputs:** compile-time/static assertions in the test.
- **Expected:** `sizeof(WPI_Handle) == sizeof(unsigned int)`,
  `alignof(WPI_Handle) == alignof(unsigned int)`, `WPI_Handle` is unsigned,
  `WPI_EventHandle` is the same type as `WPI_Handle`, and
  `WPI_EventHandle{0}` is the invalid handle value.

### C33-2 - no-shim provide/remove does not leak into a later shim

- **Layer / contract:** Layer 2 no-shim behavior for status-less event
  registration.
- **Bug class caught:** null global shim dereference or storing global handles
  outside a shim object.
- **Inputs:** no installed shim; call provide/remove for handle `11`; then
  install a shim, inject a DS packet through refresh without re-providing
  handle `11`.
- **Expected:** refresh returns true; no `WPI_SetEvent(11)` call, and no signal
  calls at all, are observed.

### C33-3 - invalid zero handle is ignored on an installed shim

- **Layer / contract:** Layer 2 invalid WPI handle behavior.
- **Bug class caught:** treating invalid handle `0` as signalable.
- **Inputs:** installed shim; provide handle `0`; remove handle `0`; send one
  `ds_state`; call `HAL_RefreshDSData()`.
- **Expected:** refresh returns true and no `WPI_SetEvent` calls are observed.

### C33-4 - registered handles are signaled when refresh accepts DS data

- **Layer / contract:** public HAL event-handle behavior tied to
  `HAL_RefreshDSData`.
- **Bug class caught:** storing but never signaling handles, signaling before
  DS data is accepted, or signaling only one registered handle.
- **Inputs:** installed shim; provide handles `101` and `202`; send one
  `ds_state`; call `HAL_RefreshDSData()`.
- **Expected:** refresh returns true; exactly handles `101` and `202` are
  passed to `WPI_SetEvent`, asserted as an unordered exact multiset.

### C33-5 - direct poll accepting DS data signals registered handles

- **Layer / contract:** DS new-data means inbound `ds_state` acceptance, not
  only the C refresh wrapper.
- **Bug class caught:** wiring signaling only in `HAL_RefreshDSData` so a
  host-driven shim poll loop cannot wake WPILib waiters.
- **Inputs:** install a shim as the process-global, provide handle `303`, send
  one `ds_state`, and call `poll()` on that same shim object directly.
- **Expected:** poll succeeds and `WPI_SetEvent(303)` is observed once.

### C33-6 - non-DS refresh does not signal and preserves registration

- **Layer / contract:** only DS data arrivals wake DS new-data handles.
- **Bug class caught:** signaling on any valid inbound packet or clearing
  registrations after a non-DS packet.
- **Inputs:** installed shim; provide handle `404`; send `clock_state`; call
  `HAL_RefreshDSData()`; then send `ds_state`; call `HAL_RefreshDSData()`.
- **Expected:** first refresh returns false and the recorder is still empty at
  that checkpoint; second refresh returns true and the recorder then contains
  exactly one `404`.

### C33-7 - removing a handle prevents later wakeups without affecting others

- **Layer / contract:** remove unregisters one handle and is idempotent.
- **Bug class caught:** remove is ignored, removes all handles, or removed
  handles continue to wake.
- **Inputs:** installed shim; provide handles `501` and `502`; remove `501`;
  remove an unknown handle; send one `ds_state`; refresh.
- **Expected:** the unordered exact signal multiset contains one `502`, zero
  `501`, and no other handles.

### C33-8 - duplicate provide coalesces to one wakeup per DS packet

- **Layer / contract:** v0 unique-handle registration.
- **Bug class caught:** duplicate registration creates repeated `WPI_SetEvent`
  calls for the same DS packet.
- **Inputs:** installed shim; provide handle `606` twice; send one `ds_state`;
  refresh.
- **Expected:** exactly one `WPI_SetEvent(606)` call is observed.

### C33-9 - registrations are per shim object

- **Layer / contract:** event registration ownership is per installed
  `shim_core`, not process-global storage independent of the shim.
- **Bug class caught:** storing event handles in a global registry so a handle
  provided on shim A wakes when shim B receives DS data.
- **Inputs:** create shim A and install it; provide handle `808`. Create shim B
  and install shim B instead. Send one `ds_state` to shim B and refresh.
- **Expected:** refresh on shim B returns true and no `WPI_SetEvent(808)` call,
  and no signal calls at all, are observed.

### C33-10 - shutdown prevents post-shutdown event wakeups

- **Layer / contract:** event handles share the terminal shutdown behavior of
  the inbound dispatch path.
- **Bug class caught:** accepting or signaling post-shutdown DS packets.
- **Inputs:** installed shim; provide handle `707`; send shutdown and refresh;
  then plant a valid `ds_state` packet on the inbound lane and refresh again.
- **Expected:** shutdown refresh returns false; `shim.is_shutting_down()` is
  true before planting the DS packet; the recorder is empty after shutdown
  refresh and remains empty after the post-shutdown refresh.

## Deferred

- Real background receive thread that can wake WPILib without a host-driven
  `poll()` / `HAL_RefreshDSData()` call.
- `HAL_GetOutputsEnabled`.
- User-program observer calls.
- Joystick outputs/rumble.
