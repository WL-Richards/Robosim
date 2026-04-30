# HAL shim core — cycle 21 test plan (HAL_CAN_ReadStreamSession + CAN RX queues)

**Status:** implemented. Round-1 verdict was `not-ready`: the reviewer
approved the overall stream direction but required the plan to resolve
HAL status choices, add a no-shim read test, add accumulation across
multiple inbound batches, make read success statuses/counts explicit,
and replace an unobservable close assertion with closed-handle plus
close-isolation coverage. Revision 2 resolved those findings and
received `ready-to-implement`.

**Implements:** the twenty-first TDD cycle of the HAL shim core. Cycle
21 wires the 2026 C HAL CAN stream surface:

```c
void HAL_CAN_OpenStreamSession(uint32_t* sessionHandle,
                               uint32_t messageID,
                               uint32_t messageIDMask,
                               uint32_t maxMessages,
                               int32_t* status);

void HAL_CAN_CloseStreamSession(uint32_t sessionHandle);

void HAL_CAN_ReadStreamSession(uint32_t sessionHandle,
                               struct HAL_CANStreamMessage* messages,
                               uint32_t messagesToRead,
                               uint32_t* messagesRead,
                               int32_t* status);
```

Official WPILib 2026.2.2 generated docs for `hal/CAN.h` define these
signatures and describe `HAL_CANStreamMessage` as:

- `uint32_t messageID`
- `uint32_t timeStamp`
- `uint8_t data[8]`
- `uint8_t dataSize`

Those fields are already mirrored byte-for-byte by
`robosim::backend::can_frame` (`message_id`, `timestamp_us`, `data`,
`data_size`) per the protocol schema cycle.

---

## Why this cycle exists

Cycle 4 intentionally left inbound `can_frame_batch` as latest-wins
because no C HAL consumer existed yet. Cycle 21 introduces that
consumer. CAN stream sessions are not a scalar latest-value read:
robot code opens a filtered stream, the shim must queue matching
frames as inbound batches arrive, and `HAL_CAN_ReadStreamSession`
drains queued frames in FIFO order.

This cycle keeps the scope to the 2026 stream API in `hal/CAN.h`:
`HAL_CAN_OpenStreamSession`, `HAL_CAN_ReadStreamSession`, and
`HAL_CAN_CloseStreamSession`.

---

## Contract under test

### New C HAL surface

`src/backend/shim/hal_c.h` adds:

```cpp
struct HAL_CANStreamMessage {
  std::uint32_t messageID;
  std::uint32_t timeStamp;
  std::uint8_t data[8];
  std::uint8_t dataSize;
};

void HAL_CAN_OpenStreamSession(std::uint32_t* sessionHandle,
                               std::uint32_t messageID,
                               std::uint32_t messageIDMask,
                               std::uint32_t maxMessages,
                               std::int32_t* status);

void HAL_CAN_CloseStreamSession(std::uint32_t sessionHandle);

void HAL_CAN_ReadStreamSession(std::uint32_t sessionHandle,
                               HAL_CANStreamMessage* messages,
                               std::uint32_t messagesToRead,
                               std::uint32_t* messagesRead,
                               std::int32_t* status);
```

### New shim behavior

- `shim_core::poll()` still updates `latest_can_frame_batch_` exactly
  as cycle 4 specified, but now also feeds every open CAN stream session
  with each frame in the accepted inbound active prefix.
- Each stream owns:
  - `handle`
  - `message_id`
  - `message_id_mask`
  - `max_messages`
  - FIFO queue of matching `can_frame`s
  - overrun flag
- A frame matches a stream when:

```cpp
(frame.message_id & stream.message_id_mask) ==
    (stream.message_id & stream.message_id_mask)
```

- Stream queues are per-session and independent. The same frame can be
  queued into multiple streams.
- `HAL_CAN_ReadStreamSession` drains up to
  `min(messagesToRead, queued_count)` frames from the selected stream,
  preserving FIFO order.

### C HAL status constants added by this cycle

`hal_c.h` adds the remaining CANSessionMux status constants used by
the stream surface:

```cpp
inline constexpr std::int32_t kHalCanMessageNotFound = -44087;
inline constexpr std::int32_t kHalCanNoToken = 44087;
inline constexpr std::int32_t kHalCanNotAllowed = -44088;
inline constexpr std::int32_t kHalCanNotInitialized = -44089;
inline constexpr std::int32_t kHalCanSessionOverrun = 44050;
```

The numeric values come from the WPILib 2026.2.2 generated
`hal/CAN.h` source. This cycle intentionally does not use
`kHalCanMessageNotFound` or `kHalCanNotInitialized`; they are defined
alongside the used constants so future CAN read cycles do not
re-transcribe the same block piecemeal.

### Out of scope

- `HAL_CAN_ReceiveMessage` one-shot latest read.
- `HAL_CAN_GetCANStatus`.
- Handle-based `hal/CANAPI.h` functions (`HAL_InitializeCAN`,
  `HAL_ReadCANPacketNew`, `HAL_ReadCANPacketLatest`, etc.).
- Threading.
- Wall-clock timeout/age behavior.

---

## Decisions pinned

### New: D-C21-STREAM-HANDLES-IN-SHIM-CORE

CAN stream sessions live on `shim_core`, not in `hal_c.cpp`, for the
same lifecycle reason as cycle 19/20 pending buffers. The C HAL seam
looks up `shim_core::current()` and delegates open/read/close to the
installed shim.

### New: D-C21-HANDLE-ZERO-INVALID

Handle value `0` is invalid, matching WPILib's `HAL_kInvalidHandle`.
The first successful v0 stream handle is nonzero. Closed handles become
invalid and are not readable.

### New: D-C21-FILTER-MASK-SEMANTICS

Stream filtering uses masked equality:
`(frame.message_id & messageIDMask) == (messageID & messageIDMask)`.
This is the standard "ID + mask" interpretation and lets a mask of
`0` match all frames. The full `message_id` is copied to the output
message; the filter does not rewrite the ID or flag bits.

### New: D-C21-QUEUE-ONLY-AFTER-OPEN

Opening a stream is not a replay operation. Frames received before the
stream opens remain visible only through the cycle-4
`latest_can_frame_batch_` cache; they are not backfilled into the new
stream queue.

### New: D-C21-FIFO-DRAIN

Reads drain queued frames in arrival order. If fewer frames are queued
than requested, the read returns the available prefix and empties the
queue. If more frames remain than requested, the unread suffix remains
queued for the next read.

### New: D-C21-ZERO-MAX-MESSAGES-INVALID

`HAL_CAN_OpenStreamSession(..., maxMessages = 0, ...)` writes
`*sessionHandle = 0`, `*status = HAL_ERR_CANSessionMux_InvalidBuffer`,
and does not create a stream. The WPILib docs define `maxMessages` as
"the maximum number of messages to stream"; zero capacity is unusable
for a stream and would make empty-vs-overrun behavior ambiguous. This
is v0 shim validation at the untrusted C seam, not a cited WPILib
hardware-behavior claim.

### New: D-C21-EMPTY-READ-NO-TOKEN

Reading a valid empty stream writes `*messagesRead = 0`, `*status =
HAL_WARN_CANSessionMux_NoToken` (`44087`), and leaves the
caller-provided `messages` array untouched. The exact native stream
empty-read status is not fully documented beyond the public
CANSessionMux status names; this v0 shim picks `NoToken` because it is
the only published warning-status name describing "no stream token /
no queued message" rather than a hard read failure. The Java CAN docs
also describe no-new-packet reads as a false/no-data result, not a
fatal condition. A future rio-bench/native-HAL fixture can tighten this
if the RIO reports a different status.

### New: D-C21-OVERFLOW-KEEPS-NEWEST

Each stream queue capacity is the `maxMessages` from
`HAL_CAN_OpenStreamSession`. If more matching frames arrive than fit,
the stream drops the oldest frames, keeps the newest `maxMessages`
frames in FIFO order, and latches an overrun flag. The next read returns
the available surviving frames and reports
`HAL_ERR_CANSessionMux_SessionOverrun` (`44050`); that read clears the
overrun flag. This is the v0 shim's deterministic version of the
Java-level `CANStreamOverflowException` description: "some messages
were lost" between reads, while the queue still contains the surviving
newest messages.

### New: D-C21-INVALID-HANDLE-NOT-ALLOWED

Reading an invalid, zero, or closed stream handle writes
`*messagesRead = 0`, `*status = HAL_ERR_CANSessionMux_NotAllowed`
(`-44088`), and leaves the caller-provided `messages` array untouched.
This cycle reserves `NotInitialized` for future whole-CAN-driver
initialization state; a bad per-stream handle is an operation not
allowed for that handle.

### New: D-C21-NULL-OUTPUT-BUFFER

For `HAL_CAN_ReadStreamSession`, `messages == nullptr` with
`messagesToRead > 0` writes `*messagesRead = 0`,
`*status = HAL_ERR_CANSessionMux_InvalidBuffer`, and does not drain the
queue. `messages == nullptr` with `messagesToRead == 0` is a zero-count
read and succeeds with `messagesRead == 0`.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR: C HAL seam uses `shim_core::current()`.
- D-C12-STATUS-WRITE-UNCONDITIONAL: `status` is written on every C HAL
  call; NULL `status` is UB matching existing HAL_* surfaces.
- D-C12-NULL-SHIM-IS-HANDLE-ERROR: no installed shim writes
  `kHalHandleError`.
- D-C4 active-prefix CAN batch ingestion and latest-wins cache update
  remain intact.

---

## Proposed tests

### C21-1. `HalCanOpenStreamSession.WithNoShimInstalledSetsHandleErrorAndInvalidHandle`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR applied to
  `HAL_CAN_OpenStreamSession`.
- **Setup:** `shim_core::install_global(nullptr)`.
- **Action:** call `HAL_CAN_OpenStreamSession(&handle, 0x100, 0x7FF, 4,
  &status)` with sentinels in `handle/status`.
- **Expected:** `status == kHalHandleError`, `handle == 0`.
- **Bug class:** no-shim path leaves a stale handle that later appears
  valid.

### C21-2. `HalCanOpenStreamSession.WithInstalledShimReturnsNonzeroHandle`

- **Layer / contract:** D-C21-STREAM-HANDLES-IN-SHIM-CORE,
  D-C21-HANDLE-ZERO-INVALID.
- **Setup:** connected shim installed globally.
- **Action:** open a stream for `messageID = 0x120`, `mask = 0x7FF`,
  `maxMessages = 4`.
- **Expected:** `status == kHalSuccess`, handle is nonzero.
- **Bug class:** C seam does not delegate to the installed shim or
  returns invalid handle 0 on success.

### C21-3. `HalCanOpenStreamSession.WithZeroMaxMessagesSetsInvalidBufferAndNoHandle`

- **Layer / contract:** D-C21-ZERO-MAX-MESSAGES-INVALID.
- **Setup:** connected shim installed globally.
- **Action:** open with `maxMessages = 0`.
- **Expected:** `status == kHalCanInvalidBuffer`, `handle == 0`.
- **Bug class:** creates a stream that can never hold a frame, leading
  to ambiguous empty-vs-overrun behavior.

### C21-4. `HalCanReadStreamSession.WithNoShimInstalledSetsHandleErrorAndDoesNotTouchMessages`

- **Layer / contract:** D-C12-NULL-SHIM-IS-HANDLE-ERROR applied to
  `HAL_CAN_ReadStreamSession`.
- **Setup:** `shim_core::install_global(nullptr)`; output message array
  filled with sentinel bytes; `messagesRead` sentinel.
- **Action:** `HAL_CAN_ReadStreamSession(1, messages, 1, &messagesRead,
  &status)`.
- **Expected:** `status == kHalHandleError`, `messagesRead == 0`,
  sentinel message bytes unchanged.
- **Bug class:** read path dereferences a null global shim or corrupts
  outputs on the no-shim failure path.

### C21-5. `HalCanReadStreamSession.WithInvalidHandleSetsNotAllowedAndDoesNotTouchMessages`

- **Layer / contract:** D-C21-HANDLE-ZERO-INVALID,
  D-C21-INVALID-HANDLE-NOT-ALLOWED.
- **Setup:** connected shim installed globally; output message array
  filled with sentinel bytes; `messagesRead` sentinel.
- **Action:** `HAL_CAN_ReadStreamSession(0, messages, 1, &messagesRead,
  &status)`.
- **Expected:** `status == kHalCanNotAllowed`, `messagesRead == 0`,
  sentinel message bytes unchanged.
- **Bug class:** invalid handle reads from stream 0 or corrupts caller
  output on failure.

### C21-6. `HalCanReadStreamSession.ValidEmptyStreamReturnsNoTokenAndLeavesMessagesUntouched`

- **Layer / contract:** D-C21-EMPTY-READ-NO-TOKEN.
- **Setup:** open a valid stream; output message array filled with
  sentinel bytes.
- **Action:** read one message before any matching inbound CAN batch.
- **Expected:** `status == kHalCanNoToken`, `messagesRead == 0`, output
  array unchanged.
- **Bug class:** empty read reports success, fabricates a zero frame, or
  clobbers output.

### C21-7. `HalCanReadStreamSession.ReturnsMatchingFramesInFifoOrderAndDrainsThem`

- **Layer / contract:** D-C21-FILTER-MASK-SEMANTICS, D-C21-FIFO-DRAIN.
- **Setup:** open stream for exact ID `0x120` with mask `0x7FF` and
  capacity 4.
- **Action:** inject one inbound `can_frame_batch` containing:
  `0x120`, nonmatching `0x121`, and `0x120` again with distinct
  timestamps/data; poll it; read two messages; read again.
- **Expected:** first read returns the two matching frames in arrival
  order with `status == kHalSuccess`, `messagesRead == 2`, and exact
  `messageID`, `timeStamp`, `data`, and `dataSize`; second read
  returns `kHalCanNoToken` and `messagesRead == 0`.
- **Bug class:** latest-wins instead of queue, filter ignored, output
  field copy mistakes, or read does not drain.

### C21-8. `HalCanReadStreamSession.PartialReadLeavesUnreadSuffixQueued`

- **Layer / contract:** D-C21-FIFO-DRAIN partial-read branch.
- **Setup:** open stream with capacity 4 and mask matching exact
  `0x555`.
- **Action:** inject three matching frames; read one; then read two.
- **Expected:** first read returns `status == kHalSuccess`,
  `messagesRead == 1`, and only frame 0; second read returns
  `status == kHalSuccess`, `messagesRead == 2`, and frames 1 and 2 in
  order.
- **Bug class:** read drains whole queue regardless of `messagesToRead`
  or reorders the suffix.

### C21-9. `HalCanReadStreamSession.AccumulatesFramesAcrossMultipleInboundBatches`

- **Layer / contract:** D-C21-FIFO-DRAIN across poll boundaries.
- **Setup:** open stream for exact `0x444` with capacity 4.
- **Action:** inject/poll a first batch containing frame A; inject/poll
  a second batch containing frames B and C; then read three messages.
- **Expected:** `status == kHalSuccess`, `messagesRead == 3`, output
  contains A, B, C in arrival order.
- **Bug class:** implementation clears each stream queue when a new
  inbound `can_frame_batch` arrives, so only the latest batch is
  readable.

### C21-10. `HalCanReadStreamSession.MaskZeroMatchesAllFramesWithoutRewritingIds`

- **Layer / contract:** D-C21-FILTER-MASK-SEMANTICS.
- **Setup:** open stream with `messageID = 0`, `messageIDMask = 0`,
  capacity 4.
- **Action:** inject frames with concrete IDs `0x101`,
  `kCanFlagFrameRemote | 0x202`, and
  `kCanFlagFrame11Bit | 0x303`.
- **Expected:** read returns every frame in arrival order, preserving
  each original `messageID`, with `status == kHalSuccess` and
  `messagesRead == 3`.
- **Bug class:** treats mask 0 as match-none or rewrites IDs to the
  filter ID.

### C21-11. `HalCanReadStreamSession.DoesNotBackfillFramesReceivedBeforeOpen`

- **Layer / contract:** D-C21-QUEUE-ONLY-AFTER-OPEN.
- **Setup:** connected shim not yet holding any stream.
- **Action:** inject/poll a matching `can_frame_batch`; then open a
  stream for that ID and read.
- **Expected:** `latest_can_frame_batch()` equals the exact injected
  batch (cycle 4 unchanged), but stream read returns `kHalCanNoToken`
  with `messagesRead == 0`.
- **Bug class:** opening a stream replays the latest cache as if it
  were queued stream data.

### C21-12. `HalCanReadStreamSession.MultipleStreamsReceiveIndependentCopies`

- **Layer / contract:** D-C21-STREAM-HANDLES-IN-SHIM-CORE,
  D-C21-FILTER-MASK-SEMANTICS.
- **Setup:** open stream A for exact `0x100`; open stream B with mask
  `0` (match all).
- **Action:** inject frames `0x100` and `0x200`; read A, then read B.
- **Expected:** A read returns `status == kHalSuccess`,
  `messagesRead == 1`, and only `0x100`; B read returns
  `status == kHalSuccess`, `messagesRead == 2`, and both frames.
  Reading A does not drain B.
- **Bug class:** single global CAN RX queue shared by all sessions or
  read drains frames from other streams.

### C21-13. `HalCanReadStreamSession.OverflowDropsOldestReportsSessionOverrunAndKeepsNewest`

- **Layer / contract:** D-C21-OVERFLOW-KEEPS-NEWEST.
- **Setup:** open stream for exact `0x321` with `maxMessages = 2`.
- **Action:** inject three matching frames A/B/C; read two messages.
- **Expected:** `status == kHalCanSessionOverrun`, `messagesRead == 2`,
  output contains B then C, and a subsequent read returns no-token.
- **Bug class:** unbounded growth, drops newest instead of oldest, or
  loses the overrun signal.

### C21-14. `HalCanReadStreamSession.NullMessagesWithNonzeroReadSetsInvalidBufferAndDoesNotDrain`

- **Layer / contract:** D-C21-NULL-OUTPUT-BUFFER.
- **Setup:** open stream and queue one matching frame.
- **Action:** read with `messages == nullptr`, `messagesToRead = 1`;
  then read again with a valid output array.
- **Expected:** first read sets `status == kHalCanInvalidBuffer`,
  `messagesRead == 0`; second read returns the originally queued frame.
- **Bug class:** null output pointer drains queued data or crashes.

### C21-15. `HalCanReadStreamSession.ZeroCountReadSucceedsForNullOrNonNullMessagesWithoutTouchingQueue`

- **Layer / contract:** D-C21-NULL-OUTPUT-BUFFER zero-count branch.
- **Setup:** open stream and queue one matching frame.
- **Action:** first read with `messages == nullptr`,
  `messagesToRead = 0`; second read with a non-null sentinel output
  array and `messagesToRead = 0`; third read one with a valid output
  array.
- **Expected:** both zero-count reads set `status == kHalSuccess`,
  `messagesRead == 0`, do not touch the non-null sentinel output, and
  do not drain. Third read returns the queued frame.
- **Bug class:** treats zero-count read as invalid buffer or drains data.

### C21-16. `HalCanCloseStreamSession.ClosedHandleIsUnreadable`

- **Layer / contract:** D-C21-HANDLE-ZERO-INVALID.
- **Setup:** open stream for exact `0x222`, then close it.
- **Action:** read using the closed handle with a sentinel output
  message.
- **Expected:** `status == kHalCanNotAllowed`, `messagesRead == 0`;
  output message sentinel bytes unchanged.
- **Bug class:** close is a no-op and closed handles remain readable.

### C21-17. `HalCanCloseStreamSession.ClosingOneStreamDoesNotAffectOtherStreams`

- **Layer / contract:** D-C21-STREAM-HANDLES-IN-SHIM-CORE,
  D-C21-HANDLE-ZERO-INVALID.
- **Setup:** open stream A for exact `0x100`; open stream B with mask
  `0` (match all); close stream A.
- **Action:** inject frames `0x100` and `0x200`; read stream B; read
  stream A.
- **Expected:** B read succeeds with `status == kHalSuccess`,
  `messagesRead == 2`, and both frames; A read reports
  `kHalCanNotAllowed` and `messagesRead == 0`.
- **Bug class:** close clears all stream sessions or corrupts another
  session's queue.

---

## Tests deliberately not added

- NULL `status`, NULL `sessionHandle`, or NULL `messagesRead` tests.
  Existing HAL_* surfaces treat required out-pointers as UB matching
  WPILib's unconditional write-through shape.
- Threaded open/read/close tests. The shim remains single-threaded v0.
- `HAL_CAN_ReceiveMessage`, `HAL_CAN_GetCANStatus`, and `CANAPI.h`
  handle-based read tests; those are separate surfaces.
- Testing stream-session capacity above this cycle's exercised values.
  The contract under test is the caller-provided per-session queue
  capacity and overflow behavior, not a project-wide maximum stream
  size.

---

## Reviewer round-1 resolutions

**OQ-C21-EMPTY-READ-STATUS.** Resolved in revision 2:
valid empty stream reads return `HAL_WARN_CANSessionMux_NoToken`.

**OQ-C21-OVERFLOW-STATUS-WITH-DATA.** Resolved in revision 2:
overflow keeps the newest frames, returns the surviving frames, reports
`HAL_ERR_CANSessionMux_SessionOverrun` once, and clears the flag on
that read.

**OQ-C21-INVALID-HANDLE-STATUS.** Resolved in revision 2:
invalid/closed stream handles return `HAL_ERR_CANSessionMux_NotAllowed`.
