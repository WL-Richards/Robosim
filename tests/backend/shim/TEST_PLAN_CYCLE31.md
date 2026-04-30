# HAL shim core — cycle 31 test plan (HAL_CAN_ReceiveMessage)

Status: implemented

Cycle 31 adds the one-shot CAN receive C HAL surface. The exact pinned
signature was reconfirmed against WPILib's primary `hal/CAN.h` source on
2026-04-30:

```cpp
void HAL_CAN_ReceiveMessage(uint32_t* messageID,
                            uint32_t messageIDMask,
                            uint8_t* data,
                            uint8_t* dataSize,
                            uint32_t* timeStamp,
                            int32_t* status);
```

The current repo already implements:

- `HAL_CAN_SendMessage` plus outbound pending CAN buffering (cycle 20).
- CAN stream open/read/close with masked FIFO queueing (cycle 21).
- `HAL_CAN_GetCANStatus` (cycle 22).
- `latest_can_frame_batch_` as a latest-wins cache populated by inbound
  `can_frame_batch` polling.

## Decisions

- **D-C31-SIGNATURE:** Use the older WPILib `HAL_CAN_ReceiveMessage`
  signature already aligned with this repo's stream-session ABI:
  `uint32_t* messageID`, `uint32_t messageIDMask`, `uint8_t* data`,
  `uint8_t* dataSize`, `uint32_t* timeStamp`, `int32_t* status`.
- **D-C31-MESSAGE-ID-INOUT:** `messageID` is an in/out pointer in v0.
  On entry, `*messageID` is the requested ID. On success, the shim writes the
  actual matched frame ID back to `*messageID`.
- **D-C31-MATCHING:** Matching uses the cycle-21 masked equality rule:
  `(frame.message_id & messageIDMask) == (requested_id & messageIDMask)`.
  A zero mask matches every frame.
- **D-C31-SOURCE:** This one-shot surface reads from
  `latest_can_frame_batch_`, not the stream-session queues. It does not open,
  close, backfill, or drain stream sessions.
- **D-C31-SELECTION:** If multiple frames in the latest batch match, return the
  first matching active-prefix frame in wire order.
- **D-C31-NO-SHIM:** No installed shim writes `kHalHandleError` and zeroes all
  writable outputs. No-shim takes precedence over `data == nullptr`, so the
  shim reports `kHalHandleError` before buffer validation when no shim is
  installed.
- **D-C31-EMPTY-OR-NO-MATCH:** Installed shim with no cached CAN frame batch,
  an empty cached batch, or no matching frame writes
  `kHalCanMessageNotFound` and zeroes all writable outputs. This uses the
  receive-message error code, not the stream-session valid-empty warning
  `kHalCanNoToken`.
- **D-C31-INVALID-BUFFER:** With an installed shim, `data == nullptr` is
  invalid because WPILib supplies an 8-byte output buffer. v0 writes
  `kHalCanInvalidBuffer`, zeroes `messageID`, `dataSize`, and `timeStamp`,
  and does not mutate or drain the receive state. Other output pointers are
  required non-null, matching existing status/out-param HAL seam conventions;
  tests do not pass them null.
- **D-C31-COPY:** On success, copy all 8 data bytes from `can_frame::data`, set
  `*dataSize` to `can_frame::data_size`, set `*timeStamp` to
  `can_frame::timestamp_us`, and set `*status = kHalSuccess`.
- **D-C31-LATEST-WINS:** Repeated inbound `can_frame_batch` polls replace the
  one-shot receive source; older batches are not searched after a later batch
  arrives.

## Proposed tests

### C31-1 — no shim reports handle error and zeroes outputs

- **Layer / contract:** Layer 2 C HAL no-shim behavior.
- **Bug class caught:** stale caller data leaks when no shim is installed.
- **Inputs:** no installed shim; `messageID = 0x123`, mask `0x7FF`,
  sentinel-filled `data[8]`, `dataSize = 99`, `timeStamp = 999`.
- **Expected:** status `kHalHandleError`; `messageID == 0`, all data bytes
  zero, `dataSize == 0`, `timeStamp == 0`.

### C31-2 — invalid data buffer reports invalid buffer without draining receive state

- **Layer / contract:** Layer 2 C HAL untrusted buffer validation.
- **Bug class caught:** null `data` dereference or accidentally consuming the
  one-shot receive source on a failed call.
- **Inputs:** installed shim with a populated latest CAN batch; call with
  `data == nullptr`, non-null `messageID`, `dataSize`, `timeStamp`, `status`.
- **Expected:** status `kHalCanInvalidBuffer`; scalar outputs zeroed. A
  subsequent valid `HAL_CAN_ReceiveMessage` for the same request returns the
  cached matching frame, proving the invalid-buffer call did not drain or
  mutate receive state.

### C31-3 — empty cache reports message-not-found and zeroes outputs

- **Layer / contract:** Layer 2 empty-cache CAN receive behavior.
- **Bug class caught:** treating no first CAN packet as success or no-token.
- **Inputs:** installed connected shim with no `can_frame_batch`; valid output
  buffers.
- **Expected:** status `kHalCanMessageNotFound`; all outputs zeroed; cache
  remains empty.

### C31-4 — empty latest batch reports message-not-found and zeroes outputs

- **Layer / contract:** Layer 2 active-prefix zero-count CAN receive behavior.
- **Bug class caught:** reading uninitialized frame slot 0 from an empty batch.
- **Inputs:** cached `can_frame_batch` with `count = 0`; request
  `messageID = 0`, `messageIDMask = 0`, which would match a zero/default
  inactive slot if the implementation scanned outside the active prefix.
- **Expected:** status `kHalCanMessageNotFound`; all outputs zeroed.

### C31-5 — exact mask success copies frame fields and overwrites message ID

- **Layer / contract:** Layer 2 one-shot CAN read success path.
- **Bug class caught:** not copying timestamp/data size/data bytes, or treating
  `messageID` as output-only instead of in/out.
- **Inputs:** cached batch with one matching frame `message_id = 0x123`,
  timestamp `4567`, `data_size = 5`, and distinct 8 data bytes; call with
  `messageID = 0x123`, mask `0x7FF`.
- **Expected:** status `kHalSuccess`; `messageID == 0x123`; data bytes,
  `dataSize`, and `timeStamp` match the frame.

### C31-6 — masked receive returns first matching active-prefix frame

- **Layer / contract:** Layer 2 masked equality and selection order.
- **Bug class caught:** unmasked comparison, returning a later matching frame,
  or selecting frames outside wire order.
- **Inputs:** latest batch active frames:
  1. `0x100` non-match for requested `0x120` mask `0x7F0`;
  2. `0x121` first match;
  3. `0x12A` later match with different data.
- **Expected:** status `kHalSuccess`; outputs match frame 2.

### C31-7 — zero mask returns the first active frame's actual ID

- **Layer / contract:** Cycle-21 mask-zero convention applied to one-shot
  receive.
- **Bug class caught:** treating zero mask as invalid or rewriting IDs through
  the requested value.
- **Inputs:** latest batch with first active frame `0x555`; call with
  `messageID = 0`, mask `0`.
- **Expected:** status `kHalSuccess`; `messageID == 0x555`; other outputs
  match the first frame.

### C31-8 — no matching frame reports message-not-found and zeroes outputs

- **Layer / contract:** Layer 2 one-shot miss behavior.
- **Bug class caught:** false positives from unmasked ID bits or stale output.
- **Inputs:** latest batch with active frames that do not satisfy the requested
  masked ID.
- **Expected:** status `kHalCanMessageNotFound`; all outputs zeroed.

### C31-9 — one-shot receive does not drain latest batch or stream sessions

- **Layer / contract:** separation between latest-cache one-shot reads and
  cycle-21 stream FIFO sessions.
- **Bug class caught:** reusing stream queues for one-shot reads, draining the
  latest cache, or consuming stream-session frames.
- **Inputs:** open a stream session for a matching ID; inject/poll one matching
  batch; call `HAL_CAN_ReceiveMessage` twice; then read the stream session.
- **Expected:** both one-shot calls return the same matching frame; stream read
  still returns the queued frame once; a second stream read returns
  `kHalCanNoToken`.

### C31-10 — latest-wins returns the newer matching batch

- **Layer / contract:** one-shot source is the current `latest_can_frame_batch_`.
- **Bug class caught:** retaining a stale matching payload after a newer
  matching poll.
- **Inputs:** inject first batch with matching ID/data A and read successfully;
  inject second batch with the same ID/data B and read again.
- **Expected:** first read returns A; second read returns B.

### C31-11 — latest-wins does not fall back to stale older matches

- **Layer / contract:** one-shot source is only the current
  `latest_can_frame_batch_`, not a history of prior batches.
- **Bug class caught:** searching stale older batches after a newer nonmatching
  or empty poll.
- **Inputs:** inject first batch with matching ID/data A and read successfully;
  inject second batch that is empty or contains only nonmatching active frames;
  read the original requested ID again.
- **Expected:** second read returns `kHalCanMessageNotFound` and zeroes all
  outputs rather than returning A.

## Deferred

- Newer 2027 bus-ID based `HAL_CAN_ReceiveMessage` and `HAL_CANMessage` /
  `HAL_CANReceiveMessage` signatures are not implemented in this cycle.
- Blocking receive, queue depth, and receive history beyond the latest
  `can_frame_batch` are intentionally not modeled for v0.
- Null `messageID`, `dataSize`, `timeStamp`, or `status` pointers remain UB
  like existing HAL read seams; v0 only validates `data == nullptr`.
