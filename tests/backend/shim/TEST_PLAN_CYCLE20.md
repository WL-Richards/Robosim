# HAL shim core — cycle 20 test plan (HAL_CAN_SendMessage + outbound buffering)

**Status:** implemented. Round-1 verdict was `not-ready`: the
buffering/flush tests were approved, but the reviewer rejected invented
permissive behavior for invalid CAN inputs. Revision 2 changed
`dataSize > 8` and `data == nullptr` with nonzero size to
`HAL_ERR_CANSessionMux_InvalidBuffer` / no enqueue, allows `nullptr`
only for `dataSize == 0`, and splits `periodMs > 0` from
`HAL_CAN_SEND_PERIOD_STOP_REPEATING`. Round-2 verdict was
`ready-to-implement`.

**Implements:** the twentieth TDD cycle of the HAL shim core, the
second C HAL ABI write surface and the CAN analogue of cycle 19's
explicit outbound buffering: `extern "C" void HAL_CAN_SendMessage(...)`
buffers robot-side CAN TX frames and `shim_core::flush_pending_can_frames`
publishes one `can_frame_batch` per tick via the existing cycle-9
`send_can_frame_batch`.

WPILib 2026.2.2 signature (from official generated docs for
`hal/CAN.h`):

```c
void HAL_CAN_SendMessage(uint32_t messageID, const uint8_t* data,
                         uint8_t dataSize, int32_t periodMs,
                         int32_t* status);
```

Cycle 20 intentionally follows the same high-level shape as cycle 19
(`HAL_SendError`) but does **not** blindly copy it: CAN has a larger
capacity (`kMaxCanFramesPerBatch == 64`), a fixed 8-byte data field,
a caller-provided `status*`, and WPILib's `periodMs` repeat/stop
semantic that our v0 wire schema cannot fully model yet.

---

## Why this cycle exists

Cycle 9 added the typed C++ outbound API:
`shim_core::send_can_frame_batch(const can_frame_batch&, uint64_t)`.
Cycle 20 wires the robot-facing C HAL seam that feeds it. Real robot
code calls `HAL_CAN_SendMessage` synchronously for individual CAN
frames; the shim accumulates those per-frame calls and the integrator
flushes once per tick.

This is the second outbound-buffering C ABI surface after
`HAL_SendError`. It should reuse cycle 19 decisions where they truly
apply:

- pending buffer lives on `shim_core`
- explicit caller-driven flush
- empty flush no-op
- successful flush clears
- transport failure retains
- post-shutdown flush short-circuits without touching buffer or lane
- overflow drops new messages

It also introduces CAN-specific seams future implementers will inherit:

- how to map `HAL_CAN_SendMessage` params into `can_frame`
- how to clamp or reject `dataSize`
- what to do with NULL `data`
- what to do with `periodMs`
- what timestamp goes in the outbound `can_frame::timestamp_us`

---

## Contract under test

### New public surface on `shim_core`

```cpp
class shim_core {
 public:
  // Cycle 20: outbound CAN buffering surface. Append `frame` to the
  // shim's pending CAN TX buffer. If the buffer is at capacity
  // (`kMaxCanFramesPerBatch == 64`), the new frame is dropped silently
  // (D-C20-OVERFLOW-DROP-INHERITS-C19). After shutdown, enqueue is
  // still accepted just like cycle 19's error buffer.
  void enqueue_can_frame(const can_frame& frame) noexcept;

  // Cycle 20: build a `can_frame_batch` from the pending buffer and
  // publish it via send_can_frame_batch (cycle 9). On success, clears
  // the buffer. On transport failure, keeps it for retry. Empty buffer
  // is a success no-op. After shutdown returns
  // shutdown_already_observed without touching buffer or lane.
  [[nodiscard]] std::expected<void, shim_error> flush_pending_can_frames(
      std::uint64_t sim_time_us);

  // Observer for the active pending CAN TX prefix.
  [[nodiscard]] std::span<const can_frame> pending_can_frames() const noexcept;
};
```

### New extern "C" surface

`src/backend/shim/hal_c.h`:

```cpp
extern "C" {

void HAL_CAN_SendMessage(std::uint32_t messageID,
                         const std::uint8_t* data,
                         std::uint8_t dataSize,
                         std::int32_t periodMs,
                         std::int32_t* status);

}  // extern "C"
```

### Internal additions

- `shim_core.h`: add private members
  `std::array<can_frame, kMaxCanFramesPerBatch> pending_can_frames_{};`
  and `std::uint32_t pending_can_frame_count_ = 0;`.
- `shim_core.cpp`: implementations of `enqueue_can_frame`,
  `flush_pending_can_frames`, and `pending_can_frames`.
- `hal_c.cpp`: `HAL_CAN_SendMessage` implementation. Looks up
  `shim_core::current()`, writes `*status` unconditionally, builds a
  zero-initialized `can_frame`, copies up to 8 bytes of payload data,
  sets `message_id`, `data_size`, and `timestamp_us` per decisions
  below, then calls `enqueue_can_frame`.

### Out of scope

- CAN RX queueing and `HAL_CAN_ReadStreamSession`. `latest_can_frame_batch_`
  remains latest-wins until that consumer lands.
- `HAL_CAN_GetCANStatus` (read surface from `latest_can_status_`).
- CAN API handle-based JNI surface (`HAL_InitializeCAN`,
  `HAL_WriteCANPacket`, etc.); this cycle targets the 2026 C HAL
  `HAL_CAN_SendMessage` surface in `hal/CAN.h`.
- Threading.
- Repeating-CAN scheduling beyond the explicit period decision below.

---

## Decisions pinned

### New: D-C20-BUFFER-IN-SHIM-CORE

The pending CAN TX buffer lives on `shim_core`, not in `hal_c.cpp`.
`HAL_CAN_SendMessage` looks up the installed shim and calls
`shim->enqueue_can_frame(frame)`. This mirrors D-C19-BUFFER-IN-SHIM-CORE
and keeps per-shim lifecycle clear.

### New: D-C20-OVERFLOW-DROP-INHERITS-C19

When `enqueue_can_frame` is called at `kMaxCanFramesPerBatch`, the new
frame is dropped silently. No rotation, no error return, no counter.
This mirrors D-C19-OVERFLOW-DROP for the same "current tick queue is
full" reason.

### New: D-C20-EXPLICIT-FLUSH-INHERITS-C19

`flush_pending_can_frames` is explicit and caller-driven. The intended
integrator loop has:

```cpp
robot_tick();                 // robot calls HAL_CAN_SendMessage
shim.flush_pending_can_frames(sim_time_us);
shim.flush_pending_errors(sim_time_us);
shim.poll();
```

Flush order between CAN and errors is integrator policy; each buffer's
method is independent. Empty flush no-ops, success clears, failure
retains, and post-shutdown short-circuits exactly like cycle 19.

### New: D-C20-SEAM-CONSTRUCTS-CAN-FRAME

The C ABI seam constructs a `can_frame` from WPILib parameters.
`shim_core::enqueue_can_frame` takes a fully-formed `can_frame` and
knows nothing about `periodMs`, NULL pointers, or WPILib's function
signature.

### New: D-C20-MESSAGE-ID-PRESERVED

`HAL_CAN_SendMessage` copies `messageID` byte-for-byte into
`can_frame::message_id`. The top flag bits
`HAL_CAN_IS_FRAME_REMOTE` and `HAL_CAN_IS_FRAME_11BIT` are preserved;
schema parity already pins their values in
`tests/backend/common/wpilib_parity_test.cpp`.

### New: D-C20-DATA-SIZE-RANGE

`dataSize` must be in the classic CAN range 0..8 documented by WPILib
2026.2.2 for `HAL_CAN_SendMessage`. If `dataSize > 8`, the shim writes
`HAL_ERR_CANSessionMux_InvalidBuffer` (`-44086`) to `*status` and does
not enqueue.

**Rationale:** the schema is classic-CAN-sized (`data[8]`) and the
official 2026.2.2 docs describe the C HAL send data as 0-8 bytes.
Clamping would hide an invalid caller and was rejected by the round-1
reviewer as uncited behavior.

### New: D-C20-NULL-DATA-AS-ZERO

If `data == nullptr` and `dataSize == 0`, enqueue a zero-length frame
with all data bytes zero from `can_frame{}` zero-init. If
`data == nullptr` and `dataSize > 0`, write
`HAL_ERR_CANSessionMux_InvalidBuffer` to `*status` and do not enqueue.

**Rationale:** unlike cycle 19's strings, CAN `data` is not documented
as optional when bytes are present. A null pointer is only safe and
meaningful when no bytes need copying.

### New: D-C20-TIMESTAMP-AT-FLUSH

Plan picks: `HAL_CAN_SendMessage` enqueues frames with
`timestamp_us == 0`; `flush_pending_can_frames(sim_time_us)` stamps
each copied outbound frame's `timestamp_us` to
`static_cast<std::uint32_t>(sim_time_us)` immediately before sending.

**Rationale:** the C HAL send call has no timestamp parameter, and v0
is single-threaded/no wall-clock. The flush boundary is the deterministic
tick time at which bytes enter the backend-to-core wire. The envelope
also carries full `uint64_t sim_time_us`; the per-frame field is
32-bit because it mirrors WPILib's `HAL_CANStreamMessage::timeStamp`.

Reviewer explicitly invited to challenge this. Alternative: leave
`timestamp_us == 0` and rely only on envelope time. Plan picks flush
stamping because it makes the outgoing `can_frame` self-consistent
with the tick that published it.

### New: D-C20-PERIODMS-DEFER-REPEAT-SCHEDULING

For `periodMs >= 0`, v0 enqueues exactly one immediate frame; repeat
scheduling for `periodMs > 0` is deferred to a future CAN periodic-send
cycle. For `periodMs == HAL_CAN_SEND_PERIOD_STOP_REPEATING` (`-1`),
v0 writes `kHalSuccess` and does not enqueue a data frame. Other
negative period values write `HAL_ERR_CANSessionMux_InvalidBuffer` and
do not enqueue.

**Rationale:** the current v0 wire schema carries concrete CAN frames,
not a repeat schedule, and the shim has no scheduler or monotonic clock
other than integrator-supplied tick times. Treating `periodMs > 0` as
"send one now" is a visible, deterministic subset that lets vendor code
that uses periodic CAN still produce observable traffic. Stop-repeat is
not a data send and must not enqueue; the future periodic-send cycle
will own cancellation semantics for an actual repeat table.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: `status` is written on every call;
  NULL `status` is UB matching the existing HAL_Get* convention.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim writes
  `kHalHandleError` and returns without enqueue.
- D-C20-CAN-INVALID-BUFFER-STATUS: `hal_c.h` adds
  `kHalCanInvalidBuffer = -44086`, mirroring
  `HAL_ERR_CANSessionMux_InvalidBuffer` from WPILib `hal/CAN.h`.
- D-C9 active-prefix send semantics: flush uses `send_can_frame_batch`.
- D-C19 buffer state machine: empty/success/failure/shutdown behavior.

---

## Proposed tests (revision 2)

### C20-1. `HalCanSendMessage.WithNoShimInstalledSetsStatusToHandleErrorAndDoesNotCrash`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR applied to
  `HAL_CAN_SendMessage`.
- **Setup:** `shim_core::install_global(nullptr)`.
- **Action:**
  ```cpp
  std::array<std::uint8_t, 8> data{1,2,3,4,5,6,7,8};
  std::int32_t status = 999;
  HAL_CAN_SendMessage(0x101, data.data(), 8, 0, &status);
  ```
- **Expected:** `status == kHalHandleError`.
- **Bug class:** null shim dereference; silent success with no installed
  shim.

### C20-2. `ShimCoreEnqueueCanFrame.AppendsFrameToPendingBuffer`

- **Layer / contract:** D-C20-BUFFER-IN-SHIM-CORE.
- **Setup:** fresh connected shim.
- **Action:** construct `can_frame frame = valid_can_frame(0x101, 0,
  4, 0xA0)` and call `shim.enqueue_can_frame(frame)`.
- **Expected:** pending span size is 1 and `pending_can_frames()[0] ==
  frame`.
- **Bug class:** no-op enqueue, wrong buffer, default frame append.

### C20-3. `HalCanSendMessage.WithShimInstalledEnqueuesConstructedFrame`

- **Layer / contract:** D-C20-SEAM-CONSTRUCTS-CAN-FRAME,
  D-C20-MESSAGE-ID-PRESERVED.
- **Setup:** fresh connected shim with global install guard.
- **Action:** call:
  ```cpp
  const std::array<std::uint8_t, 8> data{0xA0,0xA1,0xA2,0xA3,
                                         0xA4,0xA5,0xA6,0xA7};
  HAL_CAN_SendMessage(kCanFlagFrameRemote | kCanFlagFrame11Bit | 0x123,
                      data.data(), 8, 0, &status);
  ```
- **Expected:**
  - `status == kHalSuccess`.
  - pending size is 1.
  - pending frame equals `can_frame{message_id = flags|0x123,
    timestamp_us = 0, data = exact 8 bytes, data_size = 8}` via
    `operator==`.
- **Bug class:** flag bits stripped; wrong data copy; nonzero timestamp
  at enqueue instead of flush; status not overwritten.

### C20-4. `HalCanSendMessage.DataSizeAboveEightSetsInvalidBufferAndDoesNotEnqueue`

- **Layer / contract:** D-C20-DATA-SIZE-RANGE.
- **Setup:** fresh connected shim with install guard.
- **Action:** pass a 16-byte source buffer and `dataSize = 16`.
- **Expected:**
  - `status == kHalCanInvalidBuffer` (`-44086`).
  - pending frame span remains empty.
- **Bug class:** out-of-bounds write/read into schema; stores 16 in
  `data_size` causing downstream payload to claim impossible DLC;
  silently clamps invalid caller input.

### C20-5. `HalCanSendMessage.NullDataPointerWithZeroSizeEnqueuesZeroLengthFrame`

- **Layer / contract:** D-C20-NULL-DATA-AS-ZERO.
- **Setup:** fresh connected shim with install guard.
- **Action:** `HAL_CAN_SendMessage(0x555, nullptr, 0, 0, &status)`.
- **Expected:**
  - `status == kHalSuccess`.
  - pending frame has `message_id == 0x555`, `data_size == 0`,
    `data` all zero, and `timestamp_us == 0`.
- **Bug class:** null dereference when no bytes need copying; rejects a
  valid zero-length CAN frame.

### C20-5b. `HalCanSendMessage.NullDataPointerWithNonzeroSizeSetsInvalidBufferAndDoesNotEnqueue`

- **Layer / contract:** D-C20-NULL-DATA-AS-ZERO invalid branch.
- **Setup:** fresh connected shim with install guard.
- **Action:** `HAL_CAN_SendMessage(0x555, nullptr, 4, 0, &status)`.
- **Expected:**
  - `status == kHalCanInvalidBuffer` (`-44086`).
  - pending frame span remains empty.
- **Bug class:** null dereference; invents a zero payload from an
  invalid nonzero-size buffer.

### C20-6. `ShimCoreEnqueueCanFrame.OverflowDropsNewFramesAtCapacity`

- **Layer / contract:** D-C20-OVERFLOW-DROP-INHERITS-C19.
- **Setup:** fresh connected shim.
- **Action:** enqueue 64 frames with distinct `message_id` values
  1..64, then enqueue a 65th frame with `message_id = 999`.
- **Expected:** pending size remains 64, last frame has `message_id ==
  64`, and no pending frame has `message_id == 999`.
- **Bug class:** rotation, unbounded count growth past schema capacity.

### C20-7. `ShimCoreFlushPendingCanFrames.EmptyBufferIsSuccessNoOpWithoutTouchingLane`

- **Layer / contract:** D-C20-EXPLICIT-FLUSH-INHERITS-C19 empty arm.
- **Setup:** fresh connected shim, boot drained, backend lane empty.
- **Action:** `shim.flush_pending_can_frames(250'000)`.
- **Expected:** success, lane remains empty, pending span remains empty.
- **Bug class:** sends count-zero CAN batch wastefully; empty flush
  fails.

### C20-8. `ShimCoreFlushPendingCanFrames.PublishesAccumulatedFramesAsTickBoundaryEnvelope`

- **Layer / contract:** D-C20-TIMESTAMP-AT-FLUSH plus flush success
  state machine.
- **Setup:** fresh connected shim; enqueue 3 frames with
  `timestamp_us == 0` and distinct ids/data.
- **Action:** `shim.flush_pending_can_frames(500'000)`.
- **Expected:**
  - success.
  - core receives `tick_boundary` / `schema_id::can_frame_batch` /
    `sender == backend_to_core` / `sequence == 1` /
    `sim_time_us == 500'000`.
  - `payload_bytes == 64` (= 4 + 3*20).
  - memcpy payload into zero-initialized `can_frame_batch destination{}`
    and compare against `valid_can_frame_batch(expected_frames)` where
    each expected frame equals the enqueued frame except
    `timestamp_us == static_cast<uint32_t>(500'000)`.
  - pending buffer cleared.
- **Bug class:** no send, wrong schema/time/sequence, no clear on
  success, failure to stamp timestamps at flush, active-prefix size
  regression.

### C20-9. `ShimCoreFlushPendingCanFrames.PublishesFullCapacityBatchAtKMaxCanFramesPerBatchBoundary`

- **Layer / contract:** fixed-array active prefix → `can_frame_batch`
  construction at `count == 64`.
- **Setup:** fresh connected shim; enqueue exactly 64 distinct frames.
- **Action:** `shim.flush_pending_can_frames(750'000)`.
- **Expected:**
  - success.
  - received payload bytes are `4 + 64*20 == sizeof(can_frame_batch)`.
  - deserialized batch equals a `valid_can_frame_batch(all_64_frames)`
    reference with each timestamp stamped to 750'000.
  - pending buffer cleared.
- **Bug class:** off-by-one in count/copy bound; full-capacity
  active-prefix arithmetic bug.

### C20-10. `ShimCoreFlushPendingCanFrames.RetainsBufferOnTransportFailureAndPropagatesError`

- **Layer / contract:** D-C20-EXPLICIT-FLUSH-INHERITS-C19 failure arm.
- **Setup:** create shim via `make` and do not drain boot envelope, so
  backend_to_core lane is full. Enqueue 2 frames. Capture pending
  buffer and boot payload bytes.
- **Action:** `shim.flush_pending_can_frames(500'000)`.
- **Expected:**
  - failure `shim_error_kind::send_failed` wrapping
    `tier1_transport_error_kind::lane_busy`.
  - pending buffer retained byte-equal to pre-flush state.
  - backend lane still full and boot payload/envelope unchanged.
- **Bug class:** clears buffer on failed send; corrupts occupied lane.

### C20-11. `ShimCoreFlushPendingCanFrames.PostShutdownReturnsTerminalErrorWithoutTouchingBufferOrLane`

- **Layer / contract:** D-C20-EXPLICIT-FLUSH-INHERITS-C19 shutdown arm.
- **Setup:** fresh connected shim; receive shutdown and poll it; enqueue
  two frames after shutdown.
- **Action:** `shim.flush_pending_can_frames(500'000)`.
- **Expected:**
  - failure `shutdown_already_observed`.
  - `transport_error == nullopt`.
  - pending frames retained.
  - backend lane remains empty.
- **Bug class:** sends after shutdown; clears buffer post-shutdown.

### C20-12. `HalCanSendMessage.PositivePeriodMsEnqueuesOneImmediateFrameInV0`

- **Layer / contract:** D-C20-PERIODMS-DEFER-REPEAT-SCHEDULING.
- **Setup:** fresh connected shim with install guard.
- **Action:** call `HAL_CAN_SendMessage(0x321, data, 8, 20, &status)`.
- **Expected:** `status == kHalSuccess`, exactly one pending frame is
  enqueued, and its contents are the same as a `periodMs == 0` call
  would have produced.
- **Bug class:** periodMs accidentally used as frame timestamp or data
  length; v0 silently drops periodic CAN traffic.

### C20-13. `HalCanSendMessage.StopRepeatingPeriodDoesNotEnqueueADataFrame`

- **Layer / contract:** D-C20-PERIODMS-DEFER-REPEAT-SCHEDULING stop
  branch.
- **Setup:** fresh connected shim with install guard.
- **Action:** call `HAL_CAN_SendMessage(0x321, data, 8,
  HAL_CAN_SEND_PERIOD_STOP_REPEATING, &status)`.
- **Expected:** `status == kHalSuccess` and pending frame span remains
  empty.
- **Bug class:** treats stop-repeat as an ordinary send and emits an
  unintended data frame.

### C20-14. `HalCanSendMessage.OtherNegativePeriodSetsInvalidBufferAndDoesNotEnqueue`

- **Layer / contract:** D-C20-PERIODMS-DEFER-REPEAT-SCHEDULING invalid
  branch.
- **Setup:** fresh connected shim with install guard.
- **Action:** call `HAL_CAN_SendMessage(0x321, data, 8, -2, &status)`.
- **Expected:** `status == kHalCanInvalidBuffer` and pending frame span
  remains empty.
- **Bug class:** accepts undefined negative repeat periods as data
  sends.

---

## Tests deliberately not added

- A NULL `status` guard. Existing HAL_Get* surfaces treat NULL status
  as UB matching WPILib; this cycle inherits.
- A full periodic scheduler test. D-C20-PERIODMS-DEFER-REPEAT-SCHEDULING
  explicitly defers that behavior.
- CAN RX queueing tests. Owned by `HAL_CAN_ReadStreamSession`.
- Re-testing cycle-9 send-side sequence/active-prefix behavior beyond
  the flush path. C20-8/C20-9 cover the new batch construction; cycle 9
  owns `send_can_frame_batch` itself.
- A no-shim "does not enqueue" observer test. With no shim installed
  there is no per-shim observable buffer; status is the public contract.

---

## Open questions

**OQ-C20-DATA-SIZE-OVER-8.** Resolved after round 1: invalid buffer
status and no enqueue.

**OQ-C20-NULL-DATA.** Resolved after round 1: zero-size NULL is allowed;
nonzero-size NULL is invalid buffer / no enqueue.

**OQ-C20-TIMESTAMP.** Resolved by implementation: pending frames are
stamped at flush with `static_cast<uint32_t>(sim_time_us)`.

**OQ-C20-PERIODMS.** Resolved after round 1: positive period enqueues
one immediate frame and defers repeat scheduling; stop-repeat succeeds
without enqueue; other negative values are invalid buffer / no enqueue.
