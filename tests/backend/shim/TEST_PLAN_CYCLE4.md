# HAL shim core — cycle 4 test plan (can_frame_batch)

**Status:** **landed**. `ready-to-implement` per `test-reviewer`
agent (2026-04-29, single review round; round-1 verdict approved
with two plan-text refinements applied). Cycle-4 production code
and tests landed and 40-shim-test green under both clang Debug
and GCC Debug + ASan + UBSan.

**Implements:** the fourth TDD cycle of the HAL shim core. Cycles 1–3
are landed and 29-shim-test green. Cycle 4 promotes
`can_frame_batch` from "fall-through unsupported_payload_schema" to
"accepted with its own latest-wins cache slot," mirroring cycles
1–3's pattern but with **three new wrinkles** unique to this cycle:

1. **First variable-size schema.** `clock_state` / `power_state` /
   `ds_state` are all fixed-size POD structs whose `payload_bytes`
   always equals `sizeof(struct)`. `can_frame_batch` is the first
   schema whose wire payload is the *active prefix* —
   `offsetof(can_frame_batch, frames) + count * sizeof(can_frame)`
   — not the full 1284-byte struct. The shim's byte-copy path
   must (a) zero-init the cache target, then (b) memcpy
   `received->payload.size()` bytes (which equals
   `env.payload_bytes`) into it, leaving the unused
   `frames[count..63]` slots zero.
2. **First schema with per-element padding.** Each `can_frame`
   carries 3 trailing padding bytes (offsets 17–19). The cycle-3
   D-C3-7 padding-byte determinism contract extends here: every
   frame's three trailing pad bytes are part of the wire byte
   stream and must round-trip byte-equally. C4-6 pins this with a
   direct `std::memcmp`, mirroring C3-6's approach for `ds_state`.
3. **Cache-replacement edge case the prior cycles couldn't have:
   *shrinking* batches.** A latest-wins replacement of an N-frame
   batch by an M-frame batch (M < N) must produce a cache value
   where `frames[M..63]` are byte-zero, not stale data from the
   prior batch. The naive "memcpy directly into the existing
   cache slot" implementation leaks stale frames. C4-2b catches
   this; it has no analogue in cycles 1–3 because fixed-size
   schemas have no "active prefix" to shrink.

This document is an **addendum** to the cycle-1/2/3 plans. The
cycle-1/2/3 29 tests must continue to pass without behavior
change; **one cycle-1 test (test 10) is rewritten again** to
migrate its probe schema from `can_frame_batch` (now supported) to
`can_status` (next still-unsupported by `schema_id` numeric order).

---

## Why this cycle exists

Cycle 3 left the shim with three supported inbound state schemas
(`clock_state`, `power_state`, `ds_state`) and `can_frame_batch`
unimplemented. The cycle-3 test 10 uses `can_frame_batch` as its
"unsupported probe." Cycle 4 promotes `can_frame_batch` to a
supported schema with its own latest-wins cache slot.

The smallest cohesive feature is:

- **Accept** a `tick_boundary` envelope carrying
  `schema_id::can_frame_batch` with `payload_bytes ==
  offsetof(can_frame_batch, frames) + count * sizeof(can_frame)`,
  zero-init a destination `can_frame_batch{}`, and byte-copy the
  active prefix into it.
- **Expose** the new cache slot via
  `const std::optional<can_frame_batch>& latest_can_frame_batch()
  const` on `shim_core`.
- **Preserve** every existing cycle-1/2/3 contract verbatim.

The probe migration to `can_status` is mechanical per the
cycle-3-established source-of-truth rule.

---

## Contract under test (additions only)

System under test is unchanged: `robosim::backend::shim::shim_core`.

### New public surface

`src/backend/shim/shim_core.h`:

- `const std::optional<can_frame_batch>& latest_can_frame_batch()
  const` — `nullopt` until at least one inbound `tick_boundary`
  envelope carrying `schema_id::can_frame_batch` has been
  accepted; the most recent zero-padded byte-copy thereafter.
  Latest-wins, no queue (D-C4-LATEST-WINS pinned below).

### New error kinds

**None.** The fall-through dispatch is unchanged.

### Internal additions

- New private member `std::optional<can_frame_batch>
  latest_can_frame_batch_;` on `shim_core`, sibling to the three
  existing cache slots.
- New `case schema_id::can_frame_batch:` arm in the
  `tick_boundary` dispatch in `shim_core::poll()`. The arm body
  differs from the fixed-size arms in one specific way:
  ```cpp
  case schema_id::can_frame_batch: {
    can_frame_batch state{};  // zero-init — covers per-frame
                              // padding and unused frames[count..63]
                              // (D-C4-VARIABLE-SIZE, D-C4-PADDING)
    std::memcpy(&state,
                received->payload.data(),
                received->payload.size());
    latest_can_frame_batch_ = state;
    return {};
  }
  ```
  The fixed-size arms hardcode `sizeof(struct)`; this arm uses
  `received->payload.size()` which equals
  `env.payload_bytes` and may be less than `sizeof(can_frame_batch)`.

### Out of scope for this cycle

- Inbound schemas other than `can_frame_batch` and the cycle-1/2/3
  set. `can_status` is cycle 5; `notifier_state` is cycle 6; etc.
- **CAN RX queue semantics.** The shim's existing design decision
  #7 (cycle 1) explicitly anticipates that some future schema may
  need queueing rather than latest-wins. `can_frame_batch` is the
  schema that decision was talking about. Cycle 4 nonetheless
  uses **latest-wins** (D-C4-LATEST-WINS pinned below); the queue
  contract is deferred to whichever future cycle wires the C HAL
  `HAL_CAN_ReadStreamSession` API. That cycle will replace
  `latest_can_frame_batch_` with a queue type and bring its own
  positive tests of the queue contract.
- C HAL ABI surfaces (`HAL_CAN_SendMessage`,
  `HAL_CAN_ReadStreamSession`, etc.).
- Outbound `can_frame_batch`. The shim is purely consumer in
  cycle 4.

---

## Decisions pinned

- **D-C4-1.** All four cache slots (`latest_clock_state_`,
  `latest_power_state_`, `latest_ds_state_`,
  `latest_can_frame_batch_`) are **independent storage**. None
  mutates any other. Inherits and extends D-C3-1. Pinned by C4-5
  (cross-slot independence over the six new direction-pair bug
  classes that arise once a fourth slot exists).

  The six new directions are:
  - cf-arm clobbers clock-slot
  - cf-arm clobbers power-slot
  - cf-arm clobbers ds-slot
  - clock-arm clobbers cf-slot
  - power-arm clobbers cf-slot
  - ds-arm clobbers cf-slot

  The six prior cycle-2/3 directions stay covered by C2-5 and
  C3-5, which remain in the file unchanged.

- **D-C4-LATEST-WINS.** The shim's `can_frame_batch` cache uses
  **latest-wins** semantics, the same as the three prior schemas.
  This is a **deferred decision** about queue semantics, not a
  permanent one. Cycle-1 D #7 anticipated that CAN RX may
  eventually need queueing; that need is driven by the C HAL
  consumer (`HAL_CAN_ReadStreamSession`), which has not landed.
  Implementing queueing now would either:
  - require guessing the queue depth, overflow policy, and ordering
    (FIFO vs. priority) without a real consumer to validate
    against — a `code-style.md` "no half-finished implementations"
    violation; or
  - require landing the C HAL consumer and the queue together,
    which is a much larger cycle than the shim alone.

  Latest-wins is correct for cycle 4's scope (the shim is purely
  a transport drain; latest-wins matches every other
  cached schema; no live consumer demands queueing yet) and the
  test suite explicitly pins it as latest-wins so a future
  refactor that adds queueing must come with its own positive
  contract test rather than silently changing semantics. Pinned
  by C4-2 (latest-wins replacement) and C4-2b (shrinking batches
  zero-fill the unused slots).

- **D-C4-2.** Latest-wins replacement is **always** preceded by a
  full `can_frame_batch{}` zero-init of the cache target before
  the memcpy. This guarantees that a shrinking batch (M-frame
  batch replacing an N-frame batch with M < N) leaves
  `frames[M..63]` byte-zero, not stale. Pinned by C4-2b. The
  alternative implementation (memcpy directly into the existing
  cache slot's storage without zero-init) silently leaks stale
  frames; the test catches this concretely.

- **D-C4-VARIABLE-SIZE.** The shim's `tick_boundary` dispatch
  arm for `can_frame_batch` reads `received->payload.size()`
  bytes (which equals `env.payload_bytes` after framing
  validation), **not** `sizeof(can_frame_batch)`. The
  `protocol_session` validator has already asserted that
  `payload_bytes == offsetof(can_frame_batch, frames) + count *
  sizeof(can_frame)` per the variable-size validator path
  (`validator.cpp::expected_variable_payload_size`). The shim
  trusts that contract. Pinned by C4-1 and C4-1b (single-frame
  and zero-frame cases).

- **D-C4-PADDING.** Each `can_frame` carries 3 trailing padding
  bytes (offsets 17–19 within the 20-byte frame). Those bytes are
  part of the wire byte stream and must round-trip byte-equally.
  The cache target's `can_frame_batch{}` zero-init covers them
  on the destination side; the test fixture's `valid_can_frame`
  helper uses `can_frame{}` zero-init plus per-field assignment
  on the source side, so padding is zero in both ends.
  C4-6 asserts this via direct `std::memcmp` over
  `sizeof(can_frame_batch)`-many bytes, mirroring C3-6's
  treatment of `ds_state` padding.

- **D-C4-3.** Schema-specific dispatch is **strictly post-session**.
  A pre-boot-ack `tick_boundary`/`can_frame_batch` surfaces
  `expected_boot_ack_first` from the session, **not**
  `unsupported_payload_schema` from the dispatch. Inherits
  D-C3-3. Pinned by C4-4.

- **D-C4-PROBE.** Cycle-1 test 10's probe schema migrates from
  `can_frame_batch` to `can_status` (mechanical migration per the
  cycle-3 source-of-truth rule). `can_status` is the next
  per-tick-allowed schema by `schema_id` numeric order
  (`schema_id::can_status == 5`).

- **D-C4-4.** No new `shim_error_kind`. Inherits D-C3-5.

- **D-C4-5.** `latest_can_frame_batch_` is **value-initialized as
  `std::nullopt`** at construction. Inherits D-C3-6.

---

## Cross-cutting assertion shape (extends cycles 1–3)

Cycle-4 tests live in the same `ShimCoreObservers` /
`ShimCorePoll` / `ShimCoreDeterminism` GoogleTest suites.
Naming and the "one assert per behavior" convention carry over.

Per the cycle-1/2/3 pattern:

- All payload comparisons use full-struct `operator==` for
  field-equality plus `std::memcmp` where padding-byte
  determinism matters (C4-6).
- All `has_value()` prerequisites before dereferences are
  `ASSERT_TRUE`, not `EXPECT_TRUE`.
- All scenarios use explicit-constant fixture values; no
  wall-clock, no PRNG.

`can_frame_batch::operator==` recurses through `frames` (a
`std::array`) which uses element-wise `can_frame::operator==`.
Each frame's `operator==` compares `message_id`,
`timestamp_us`, `data`, `data_size` — but **not** the 3 trailing
padding bytes. Field equality holds whenever the producer
constructed each frame via `can_frame{}` zero-init plus per-field
assignment (the fixture's discipline). Byte equality
(including padding) holds for the same reason and is asserted
explicitly only in C4-6.

---

## Test fixtures (additions only)

Cycle 4 adds two inline helpers to `tests/backend/tier1/test_helpers.h`:

```cpp
inline can_frame valid_can_frame(std::uint32_t message_id   = 0x123,
                                 std::uint32_t timestamp_us = 1000,
                                 std::uint8_t data_size     = 4,
                                 std::uint8_t fill_byte     = 0xAB) {
  can_frame f{};  // zero-init — covers the 3 trailing padding bytes
                  // (D-C4-PADDING) and any frames-array slots not
                  // explicitly populated.
  f.message_id = message_id;
  f.timestamp_us = timestamp_us;
  for (std::size_t i = 0; i < data_size && i < f.data.size(); ++i) {
    f.data[i] = static_cast<std::uint8_t>(fill_byte + i);
  }
  f.data_size = data_size;
  return f;
}

// Builds a can_frame_batch holding `frames.size()` elements, with
// the count field set accordingly and the unused frames[count..63]
// left as zero from the can_frame_batch{} zero-init. Span input
// avoids forcing a particular container choice on callers.
inline can_frame_batch valid_can_frame_batch(
    std::span<const can_frame> frames = {}) {
  can_frame_batch batch{};  // zero-init (D-C4-PADDING covers
                            // every per-frame padding region of
                            // every unused frames slot).
  batch.count = static_cast<std::uint32_t>(frames.size());
  for (std::size_t i = 0; i < frames.size() && i < batch.frames.size(); ++i) {
    batch.frames[i] = frames[i];
  }
  return batch;
}

// Returns a span of `payload_bytes` worth of bytes from the
// can_frame_batch — the active prefix only, not sizeof(struct).
// This is what the wire payload should be: the cycle-4 dispatch
// arm reads exactly this many bytes.
inline std::span<const std::uint8_t> active_prefix_bytes(
    const can_frame_batch& batch) {
  const auto active_size =
      offsetof(can_frame_batch, frames) +
      static_cast<std::size_t>(batch.count) * sizeof(can_frame);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}
```

**Rationale for the helper shape:**

- `valid_can_frame` defaults: `message_id = 0x123` (a non-zero
  11-bit ID), `timestamp_us = 1000`, `data_size = 4`,
  `fill_byte = 0xAB`. These produce a frame where every named
  field is non-zero and distinct from a default-constructed
  `can_frame{}` — so a "shim writes 0s" or "shim writes wrong
  field" bug fails byte-equality loudly. The first 4 bytes of
  `data` are `0xAB, 0xAC, 0xAD, 0xAE` (sequential to ensure
  byte-position bugs in the 8-byte data array are caught);
  bytes 4..7 of `data` are zero (data_size says they're
  unused).

- `valid_can_frame_batch` takes a `std::span<const can_frame>`
  rather than a varlist or initializer_list so callers can
  build vectors of any size without ceremony. The helper is
  tolerant of `frames.size() > 64` only by truncation; tests do
  not exercise that path (and the protocol validator would
  reject `count > kMaxCanFramesPerBatch` upstream).

- `active_prefix_bytes` returns a span over exactly the bytes
  that match the wire `payload_bytes`. Callers pass this span
  directly to `core.send(...)`. The helper exists because
  `bytes_of(batch)` (the existing `template <typename T>`
  helper) would return all 1284 bytes, which is wrong for a
  variable-size schema and would be rejected by the validator
  on the count↔length mismatch (the same trap that bit cycle
  3's first attempt at the test-10 rewrite).

**Implementer checklist:** the helpers reference `can_frame`,
`can_frame_batch`, `kMaxCanFramesPerBatch`, and `offsetof`. The
implementer must add `#include "can_frame.h"` to
`tests/backend/tier1/test_helpers.h` (alphabetically between
`boot_descriptor.h` and `clock_state.h`) and `#include <cstddef>`
for `offsetof` if it is not already transitively included. The
existing `<array>`, `<cstdint>`, `<cstring>`, `<span>`, `<vector>`
includes cover everything else.

---

## Determinism notes

Single-threaded same-process tests. No new sources of
nondeterminism. C4-6 explicitly re-asserts byte-identical
output across two runs of an expanded scenario that includes all
four schemas. Padding-byte determinism on `can_frame_batch` is
pinned **directly** by C4-6 via `std::memcmp` (mirroring C3-6's
treatment of `ds_state` padding); this is required because
`can_frame::operator==` is field-by-field and does not pin the 3
per-frame trailing pad bytes.

`clock_state` and `power_state` continue to need no `memcmp`
(neither has padding); `ds_state` and `can_frame_batch` both do.

---

## Proposed tests (revision 1)

### C4-1. `ShimCorePoll.AcceptsCanFrameBatchAndCachesByteEqualValue`

- **Layer / contract:** Layer 2 HAL shim inbound dispatch;
  `latest_can_frame_batch()` accessor; D-C4-1 (no cross-slot
  contamination at first arrival); D-C4-VARIABLE-SIZE
  (`payload.size()`-driven memcpy).
- **Bug class caught:** payload byte corruption in transit; wrong
  cache slot written; the dispatch arm uses
  `sizeof(can_frame_batch)` = 1284 instead of `payload.size()`,
  causing a buffer over-read or copying junk into the cache; the
  arm forgets to zero-init the destination, leaving `frames[count..63]`
  uninitialized.
- **Inputs:**
  - Shim connected via `make_connected_shim`.
  - Build a 3-frame batch:
    `valid_can_frame_batch(std::array{
        valid_can_frame(0x100, 1000, 4, 0xA0),
        valid_can_frame(0x200, 2000, 8, 0xB0),
        valid_can_frame(0x300, 3000, 0, 0xC0)})` — three distinct
    `message_id`s, three distinct `timestamp_us` values, three
    distinct `data_size` values (including the boundary case
    `data_size == 0` for frame 3, and the boundary case `data_size
    == 8` for frame 2).
  - Core endpoint sends `tick_boundary`/`can_frame_batch` at
    sequence 1 with payload `active_prefix_bytes(batch)` and
    `sim_time_us = 1'000`.
- **Expected outputs after `poll()`:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_can_frame_batch().has_value())`.
  - `*shim.latest_can_frame_batch() == batch` using the defaulted
    `operator==` (full-struct equality recursing through
    `frames`).
  - `EXPECT_FALSE(shim.latest_clock_state().has_value())`.
  - `EXPECT_FALSE(shim.latest_power_state().has_value())`.
  - `EXPECT_FALSE(shim.latest_ds_state().has_value())`.
  - `region.core_to_backend.state == empty`.
- **Tolerance:** zero. No arithmetic.

### C4-1b. `ShimCorePoll.AcceptsEmptyCanFrameBatch`

- **Layer / contract:** D-C4-VARIABLE-SIZE — the variable-size
  byte-copy path correctly handles `count == 0`. The wire
  `payload_bytes` is exactly `offsetof(can_frame_batch, frames) ==
  4` (the count field alone).
- **Bug class caught:** dispatch arm hardcodes a minimum
  `sizeof(can_frame_batch)` and rejects 4-byte payloads; the
  active-prefix memcpy reads zero bytes and silently leaves the
  cache as nullopt; the cache value's `count` field is mis-read
  as nonzero.
- **Inputs:**
  - Shim connected.
  - Empty batch: `valid_can_frame_batch()` (defaulted span; count
    = 0).
  - Core endpoint sends `tick_boundary`/`can_frame_batch` at
    sequence 1 with payload `active_prefix_bytes(batch)` (4 bytes
    of count=0) and `sim_time_us = 1'000`.
- **Expected outputs after `poll()`:**
  - `ASSERT_TRUE(poll().has_value())`.
  - `ASSERT_TRUE(shim.latest_can_frame_batch().has_value())`.
  - `shim.latest_can_frame_batch()->count == 0u`.
  - `*shim.latest_can_frame_batch() == valid_can_frame_batch()`
    (full-struct equality: count is 0 and frames is all-zero in
    both fixture and cache).
- **Bug class:** the empty-batch case is a real boundary case
  the larger fixture in C4-1 does not exercise. The protocol
  validator accepts `count == 0` (per `expected_variable_payload_size`'s
  bounds check); the shim must not reject it.

### C4-2. `ShimCorePoll.LatestWinsForRepeatedCanFrameBatchUpdates`

- **Layer / contract:** D-C4-LATEST-WINS — cache replacement is
  unconditional (no merge, no append), and consumers see the
  most recent batch as the entire cache content.
- **Bug class caught:** cache holds first batch forever; cache
  appends frames from the second batch to the first; cache
  field-merges across batches.
- **Inputs:** shim connected. Two `tick_boundary`/`can_frame_batch`
  envelopes:
  - First (sequence 1): batch with 2 frames using non-zero
    `message_id`s `0xAA` and `0xBB`.
  - Second (sequence 2): batch with 2 different frames using
    `message_id`s `0xCC` and `0xDD`.
- **Expected outputs:**
  - After first poll: cache holds the first batch (full-struct
    equality).
  - After second poll: cache holds the second batch (full-struct
    equality). The cache is *not* appending; it is replacing.
- **Bug class:** as above.

### C4-2b. `ShimCorePoll.ShrinkingBatchClearsTrailingFramesInCache`

- **Layer / contract:** D-C4-2 — the dispatch arm zero-inits the
  destination before memcpy, so a shrinking batch (M-frame batch
  replacing an N-frame batch with M < N) produces a cache value
  where `frames[M..63]` are byte-zero, not stale data from the
  prior batch.
- **Bug class caught:** the dispatch arm memcpys directly into
  `latest_can_frame_batch_`'s storage without first zero-initing,
  leaking the prior batch's frames. This is a real implementation
  trap because `std::optional<T>::operator=(T)` does not
  zero-init the held storage between assignments; an "efficient"
  implementation that aliases the storage would silently retain
  stale frames in `frames[M..63]`.
  This bug class has no analogue in cycles 1–3 because fixed-size
  schemas have no "shrinking active prefix" to leak.
- **Inputs:** shim connected.
  - Step 1: 5-frame batch at sequence 1 with all five frames
    distinct (e.g. `message_id` = `0x10, 0x20, 0x30, 0x40, 0x50`).
    Shim `poll()`.
  - Step 2: 2-frame batch at sequence 2 with two new distinct
    frames (e.g. `message_id` = `0xA0, 0xB0`).
    Shim `poll()`.
- **Expected outputs after step 2:**
  - `ASSERT_TRUE(shim.latest_can_frame_batch().has_value())`.
  - `shim.latest_can_frame_batch()->count == 2u`.
  - `shim.latest_can_frame_batch()->frames[0]` equals the first
    new frame (`message_id == 0xA0`).
  - `shim.latest_can_frame_batch()->frames[1]` equals the second
    new frame (`message_id == 0xB0`).
  - **`shim.latest_can_frame_batch()->frames[2] == can_frame{}`**
    (the prior batch's third frame, `message_id == 0x30`, must
    not survive). Asserted via `EXPECT_EQ` on the full
    `can_frame{}` default value, not by checking only
    `message_id`, so a partial-clear bug (e.g. zeroing only the
    first 4 bytes of each unused slot) is caught.
  - Same assertion for `frames[3]` and `frames[4]` against
    `can_frame{}`.
  - **Plus byte-level guard:**
    `EXPECT_EQ(std::memcmp(&shim.latest_can_frame_batch()->frames[2],
    &empty_frame, sizeof(can_frame)), 0)` where
    `const can_frame empty_frame{};` — pins that even the 3
    padding bytes of the unused slot are zero (a partial
    field-clear bug that left the prior frame's padding intact
    would survive `operator==` but fail this).
- **Bug class:** described inline. This is a single-test pin of
  D-C4-2; without it, an aliased-storage implementation passes
  every other test in the suite (including C4-2's latest-wins
  field-equality assertion, because the unused frames are not
  reached by `operator==`).

### C4-3a. `ShimCoreObservers.CanFrameBatchNulloptBeforeAnyPoll`

- **Layer / contract:** D-C4-5.
- **Suite placement:** `ShimCoreObservers` (cycle-2 / cycle-3
  precedent for "post-make pre-poll observer state" tests).
- **Inputs:** shim post-`make`; no `poll()` yet.
- **Expected outputs:** `shim.latest_can_frame_batch() ==
  std::nullopt`.
- **Bug class:** default-init drift to `can_frame_batch{}`
  instead of `nullopt`.

### C4-3b. `ShimCorePoll.CanFrameBatchNulloptAfterBootAckIsAccepted`

- **Layer / contract:** D-C4-5 + D-C4-1 — the `boot_ack` dispatch
  arm does not contaminate the cf slot.
- **Inputs / expected outputs:** mirror C3-3b's structure with
  the cf slot in place of the ds slot. `ASSERT_TRUE(poll().has_value())`,
  `ASSERT_TRUE(shim.is_connected())` (prerequisite with the
  cycle-2/3-established false-green rationale), then
  `EXPECT_FALSE(shim.latest_can_frame_batch().has_value())`.

### C4-3c. `ShimCorePoll.CanFrameBatchNulloptAfterClockStateIsAccepted`

- **Layer / contract:** D-C4-1 — `clock_state` dispatch does not
  cross-populate the cf slot.
- **Inputs / expected outputs:** mirror C3-3c with the cf slot
  in place of the ds slot. `ASSERT_TRUE(poll().has_value())`,
  `ASSERT_TRUE(shim.latest_clock_state().has_value())`
  (prerequisite), `EXPECT_FALSE(shim.latest_can_frame_batch().has_value())`.

### C4-3d. `ShimCorePoll.CanFrameBatchNulloptAfterPowerStateIsAccepted`

- **Layer / contract:** D-C4-1 — `power_state` dispatch does not
  cross-populate the cf slot.
- **Inputs / expected outputs:** mirror C3-3d with cf in place of
  ds.

### C4-3e. `ShimCorePoll.CanFrameBatchNulloptAfterDsStateIsAccepted`

- **Layer / contract:** D-C4-1 — `ds_state` dispatch (the
  cycle-3-introduced arm, immediately adjacent to the cycle-4
  cf arm in dispatch order) does not cross-populate the cf slot.
- **Inputs / expected outputs:** mirror the C3-3 family with the
  ds_state arrival driving the prerequisite check. This is the
  most likely failure-class for a bad cycle-4 refactor: a
  copy-paste from the new cf arm into the existing ds arm that
  accidentally writes the ds payload into both slots. The test
  pins the no-cross-population direction explicitly.

### C4-4. `ShimCorePoll.RejectsCanFrameBatchBeforeBootAckPreservingLane`

- **Layer / contract:** D-C4-3 — schema-specific dispatch is
  post-session.
- **Inputs / expected outputs:** mirror C3-4 with schema swapped
  to `can_frame_batch`. The injected envelope must use a
  variable-size payload; the test injects `count == 0` (the
  4-byte minimal active prefix) so the framing validator accepts
  the framing and the session rejects on order. No legitimate peer
  could produce a pre-boot-ack `tick_boundary`. The expected
  shim_error wraps `expected_boot_ack_first` from the session.
  All four cache slots remain `nullopt`; lane preserved unchanged.

### C4-5. `ShimCorePoll.AllFourCachesAreIndependentlyMaintained`

- **Layer / contract:** D-C4-1 — covers the **six new
  cross-population bug classes** introduced by the fourth slot
  (the cycle-2 and cycle-3 directions remain pinned by the
  existing C2-5 and C3-5 tests, which are not modified).
- **Bug class caught:** each of the six new directions
  enumerated under D-C4-1; shared single-slot or variant-typed
  cache; misread `schema_id`.
- **Setup:** shim connected via `make_connected_shim`. A
  seven-step sequence interleaves all four schemas with
  `ASSERT_TRUE` gates on every `has_value()` prerequisite before
  any dereference (per the cycle-3-established convention).

  - **Step 1: clock_state arrival.** `valid_clock_state(100'000)`.
    Assert clock populated; power, ds, cf all nullopt.
  - **Step 2: power_state arrival.** `valid_power_state(12.5f,
    2.0f, 6.8f)`. Assert clock unchanged (this re-pins the
    cycle-2 power→clock direction, but the primary new
    coverage starts at step 4); power populated; ds, cf still
    nullopt.
  - **Step 3: ds_state arrival.** `valid_ds_state()`. Assert
    clock and power unchanged; ds populated; cf still nullopt.
  - **Step 4: can_frame_batch arrival.** Concrete fixture
    (matching the style of all prior multi-step tests):
    ```
    const auto cf_first = valid_can_frame_batch(std::array{
        valid_can_frame(0x100, 1000, 4, 0xA0),
        valid_can_frame(0x200, 2000, 2, 0xB0)});
    ```
    Assert clock unchanged (***catches "cf-arm clobbers
    clock-slot"***); power unchanged (***catches "cf-arm
    clobbers power-slot"***); ds unchanged (***catches "cf-arm
    clobbers ds-slot"***); cf populated and equal to `cf_first`
    (full-struct `operator==`).
  - **Step 5: second clock_state arrival.**
    `valid_clock_state(200'000)`. Assert clock updated; power,
    ds unchanged; **cf unchanged** — i.e. `*shim.latest_can_frame_batch()
    == cf_first` (***catches "clock-arm clobbers cf-slot"***).
  - **Step 6: second power_state arrival.** `valid_power_state(13.5f,
    5.0f, 6.5f)`. Assert clock unchanged (from step 5); power
    updated; ds unchanged; **cf unchanged** — i.e.
    `*shim.latest_can_frame_batch() == cf_first` (***catches
    "power-arm clobbers cf-slot"***).
  - **Step 7: second ds_state arrival.** Concrete fixture:
    `valid_ds_state(/*joystick0_axis_count=*/4,
                    /*joystick0_axis_0_value=*/-0.25f,
                    /*control_bits=*/kControlEnabled | kControlAutonomous,
                    /*station=*/alliance_station::blue_2,
                    /*type=*/match_type::elimination,
                    /*match_number=*/99,
                    /*match_time_seconds=*/30.0)` (every field
    distinct from the step-3 `valid_ds_state()` defaults).
    Assert clock, power unchanged; ds updated to this fixture;
    **cf unchanged** — i.e. `*shim.latest_can_frame_batch() ==
    cf_first` (***catches "ds-arm clobbers cf-slot"***).
- **Expected outputs:** at every step, every slot's expected
  state is asserted (populated or nullopt; if populated, the
  expected value or "unchanged from prior step"). All
  `has_value()` checks before dereference are `ASSERT_TRUE`.
- **Tolerance:** zero.

### C4-6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalAllFourSlots`

- **Layer / contract:** CLAUDE.md non-negotiable #5. Strict
  superset of the cycle-3 C3-6 — same boot_ack + clock + power +
  ds + clock scenario, with a `can_frame_batch` arrival inserted
  between ds and the second clock. Replaces (drops) the cycle-3
  C3-6 in the file per the cycle-2/3-established "no
  half-finished implementations" precedent for strict-superset
  determinism tests.
- **Bug class caught:** seam-level nondeterminism in the
  `can_frame_batch` path; per-frame padding-byte divergence
  between compile configurations; shim's variable-size memcpy
  produces nondeterministic output for the same input.
- **Inputs:** two independent setups running the identical
  sequence:
  - boot_ack at seq 0; poll.
  - `tick_boundary`/`clock_state` at seq 1 with
    `valid_clock_state(50'000)`; poll.
  - `tick_boundary`/`power_state` at seq 2 with
    `valid_power_state(12.5f, 2.0f, 6.8f)`; poll.
  - `tick_boundary`/`ds_state` at seq 3 with `valid_ds_state()`;
    poll.
  - `tick_boundary`/`can_frame_batch` at seq 4 with a 2-frame
    batch; poll.
  - `tick_boundary`/`clock_state` at seq 5 with
    `valid_clock_state(100'000)`; poll.
- **Expected outputs:**
  - All eight `has_value()` gates are `ASSERT_TRUE` (both
    setups, all four slots).
  - `*setup_a.latest_clock_state() == *setup_b.latest_clock_state()`.
  - `*setup_a.latest_power_state() == *setup_b.latest_power_state()`.
  - `*setup_a.latest_ds_state() == *setup_b.latest_ds_state()`.
  - `*setup_a.latest_can_frame_batch() == *setup_b.latest_can_frame_batch()`.
  - **`std::memcmp` on `latest_ds_state()`** (D-C3-7 inheritance,
    pinned by C3-6 originally; carried into this superset).
  - **`std::memcmp` on `latest_can_frame_batch()`** (D-C4-PADDING) —
    new for cycle 4. Inline rationale: `can_frame::operator==`
    is field-by-field and does not pin the 3 trailing padding
    bytes per frame; `std::memcmp` over `sizeof(can_frame_batch)`
    bytes pins every padding byte across all 64 frame slots
    (active and unused) plus the 4-byte count header.
  - `EXPECT_TRUE(setup_a.is_connected())`,
    `EXPECT_TRUE(setup_b.is_connected())`.

---

## Test rewrites (cycle-1/3 plan touched)

### C4-R10. `ShimCorePoll.RejectsUnsupportedPayloadSchemaThenStillAcceptsValidNext` (rewrite #3)

- **Source of change:** D-C4-PROBE. Cycle 4 makes
  `can_frame_batch` supported; cycle-3's rewrite of test 10
  (which uses `can_frame_batch` as the probe) would now silently
  no-op the rejection contract.
- **Mechanical changes to `tests/backend/shim/shim_core_test.cpp`
  test 10:**
  1. **Send line.** Change `schema_id::can_frame_batch` →
     `schema_id::can_status`. The payload changes from the
     4-byte `count=0` header to `bytes_of(can_status{})`.
     `can_status` is fixed-size (per `validator.cpp`'s
     `fixed_payload_size`), so the existing `bytes_of`
     template helper works correctly. The local variable
     `empty_batch_header` becomes a fresh `can_status` named
     `cstatus`; the corresponding `core.send` payload changes
     to `bytes_of(cstatus)`. **Cycle 5 will land `can_status`
     and migrate this probe to the next still-unsupported
     schema (`notifier_state`).**
  2. **Inline comment at the call site.** The cycle-3 comment
     references `can_frame_batch` and the variable-size count
     mechanism. The cycle-4 rewrite simplifies it back to the
     fixed-size case (mirrors cycle 2's structure):
     ```
     // Step A — send a valid tick_boundary/can_status. Framing
     // is valid (kPerKindAllowedSchemas allows can_status under
     // tick_boundary), so the protocol_session accepts and
     // advances next-expected to 2. The shim's post-session
     // dispatch refuses the schema. Cycle 5 will land
     // can_status and migrate this probe to the next
     // still-unsupported schema (notifier_state).
     ```
  3. **Block-comment header above the `TEST(...)` line.**
     Update the cumulative supported-schema set: "Schemas
     other than `clock_state`, `power_state`, `ds_state`,
     `none`" becomes "Schemas other than `clock_state`,
     `power_state`, `ds_state`, `can_frame_batch`, `none`".
     Update "cycle-3 limit" to "cycle-4 limit" and the
     trailing migration note to reference cycle 5 →
     `notifier_state`.
- **Bug class (unchanged):** shim silently dropping schemas it
  can't yet handle (would mask cycle-5+ divergence); shim
  corrupting the session receive counter on dispatch reject
  (Step B's `clock_state` send at the next sequence would fail).

### C4-R6. `ShimCoreDeterminism.RepeatedRunsProduceByteIdenticalLatestClockPowerAndDsState` (drop)

This cycle-3 determinism test is **dropped from the file** and
replaced by C4-6 (which is a strict superset: the cycle-3
scenario plus a `can_frame_batch` arrival; same `ASSERT_TRUE`
gating; same `std::memcmp` on `ds_state` plus a new `std::memcmp`
on `can_frame_batch`). Same `code-style.md` "no half-finished
implementations" rationale that cycle 3 used to drop cycle-2's
C2-6.

---

## Cross-test invariants (additions)

- All cycle-4 tests construct one shim per test; no shared
  state.
- All `sim_time_us`, `message_id`, `timestamp_us`, `data_size`,
  and other fixture parameters are explicit constants.
- All `can_frame_batch` payload comparisons use full-struct
  `operator==`. Field-level checks
  (e.g. `latest_can_frame_batch()->count == 2u`) appear only as
  readability aids in C4-2b's per-frame assertions.
- C4-5 and C4-6 explicitly assert on **all four** cache slots at
  every step.
- The `active_prefix_bytes` helper is the **only** way cycle-4
  tests get a wire payload from a `can_frame_batch`. Using
  `bytes_of(batch)` would send 1284 bytes and fail framing
  validation (count↔length mismatch).

### C-N-3 family scaling — cutover plan

Each cycle adds one schema and consequently one new "post-X-poll,
new-slot stays empty" test per existing schema (cycle 4 adds 4
such tests: C4-3b/c/d/e on top of the existing default-init
C4-3a). The family count is therefore N + 1 at cycle N (cycle 4 =
5 tests; cycle 5 = 6; cycle 8 — when every schema is supported —
= 9 tests). This is acceptable through cycle 6 but starts to
feel mechanical at cycle 7+.

**Stated cutover:** when the family reaches **8 tests** (cycle 7,
adding `notifier_alarm_batch`), convert C-N-3b/c/d/... to a
single `INSTANTIATE_TEST_SUITE_P` parameterized over a list of
`(probe_schema_id, send_helper, prerequisite_observer)` tuples.
The C-N-3a default-init test stays separate (it has no
prerequisite-observer to parameterize over). Until then, the
linear growth is the right trade-off — explicit single-purpose
tests are easier to read and easier to fail-localize than a
parameterization.

A future cycle that adopts the parameterization brings its own
positive test of the parameterized harness alongside; this is
not a "free refactor" cycle.

## Stub disclosure (additions)

- The "core" peer is real `tier1_endpoint` + real
  `protocol_session` (no fakes), same as cycles 1–3.
- C4-4 reaches behind the endpoint's send path via
  `manually_fill_lane`, same rationale as the cycle-1/2/3
  pre-boot-ack tests.
- No new mocks. The SUT (`shim_core`) is never mocked.

---

## What this plan does NOT cover

- **CAN RX queueing semantics.** D-C4-LATEST-WINS pins
  latest-wins for cycle 4 and explicitly defers queue contracts
  to the cycle that wires `HAL_CAN_ReadStreamSession`.
- **Outbound `can_frame_batch`.** The shim is purely consumer in
  cycle 4. Outbound (CAN TX from the robot binary) is cycle 7+.
- **C HAL ABI surfaces** (`HAL_CAN_SendMessage`,
  `HAL_CAN_ReadStreamSession`, etc.).
- **Validity-domain checks on `can_frame` field values** (e.g.
  rejecting `data_size > 8`, `message_id` collisions). The
  protocol validator caps the active count at
  `kMaxCanFramesPerBatch`; per-frame field validation is the
  consumer's responsibility.
- **`count > kMaxCanFramesPerBatch` rejection.** Already enforced
  by the protocol validator's `expected_variable_payload_size`
  upstream of the shim; not the shim's responsibility.
- **Padding-byte memcmp on `clock_state` and `power_state`.**
  Same rationale as cycle 3: neither has padding bytes. Inherits
  the cycle-3 exemption.
- **Post-shutdown / post-`lane_in_progress` `latest_can_frame_batch()`
  invariance.** Same rationale as the cycle-2/3 analogues: those
  paths short-circuit before any dispatch, so the invariance is
  by construction. Cycle-1 tests 12 and 13 already pin those
  paths schema-agnostically.

---

## Open questions

None for cycle 4. The shape closely mirrors cycles 1–3. The one
genuinely new design call (latest-wins vs. queue) is pinned by
D-C4-LATEST-WINS as a *deferred* decision with the rationale
inline; the alternative landing is reserved for the cycle that
brings a live consumer.
