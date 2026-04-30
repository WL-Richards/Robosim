# HAL shim core — cycle 19 test plan (HAL_SendError + outbound buffering)

**Status:** implemented. Revision 3 received final narrow
`test-reviewer` verdict `ready` after two `not-ready` rounds.
Round-1 verdict was `not-ready` with two blockers
(C19-9 missing boot-envelope-bytes-unchanged assertion mirroring
C9-3; C19-8 underspecified deserialization mechanism for
wire-content verification) and two strongly-recommended changes
(C19-3 should use `operator==` against a reference message rather
than per-field assertions, to cover `reserved_pad` zero-init;
new C19-8b adding a full-8-message capacity test for the
flush-construction path). Round-2 addressed all four. Round-2
verdict was `not-ready` with one blocker: C19-8/C19-8b did not
assert `error_message_batch::reserved_pad[4]` stayed zero on the
wire. Round-3 addresses that by comparing deserialized batches
against `valid_error_message_batch(...)` references via defaulted
`operator==`, and fixes stale plan text called out by the
reviewer. Round-1 also ruled all four open questions:
OQ-C19-NO-SHIM-RETURN-VALUE
(`kHalHandleError`); OQ-C19-VECTOR-VS-ARRAY-FOR-BUFFER (switch to
`std::array<error_message, kMaxErrorsPerBatch> + uint32_t count`;
the array shape avoids heap allocation, expresses capacity
structurally, and simplifies `flush_pending_errors`);
OQ-C19-POST-SHUTDOWN-ENQUEUE-GATE (allow); OQ-C19-DROP-COUNTER
(defer). Round-2 adopts the array shape per the reviewer's
strongly-suggested OQ resolution. Test count grows from 10 to 11
with C19-8b.

**Implements:** the nineteenth TDD cycle of the HAL shim core, the
**first outbound-buffering surface** and the **eleventh C HAL ABI
function**: `extern "C" int32_t HAL_SendError(...)` taking 7
parameters and the supporting buffering layer on `shim_core`. This
cycle introduces a fundamentally new mechanic — robot code calls
HAL_SendError synchronously per error; the shim accumulates
messages in a per-shim buffer and the integrator flushes the buffer
explicitly at tick boundary via `flush_pending_errors`, which
publishes one `error_message_batch` envelope per flush via the
existing cycle-11 `send_error_message_batch`.

Cycles 1–18 are landed and 146-shim-test green; full project ctest
461/461 green under both clang Debug and GCC Debug + ASan + UBSan.

---

## Why this cycle exists

Every prior C HAL surface (cycles 12–18) was a **read** function
that returned a value derived from an inbound cache slot. Cycle 19
introduces the **first write surface**: HAL_SendError is robot
code's synchronous "log this error" call. Real WPILib forwards each
error over the network independently; our model batches up to
`kMaxErrorsPerBatch == 8` errors per tick into one
`error_message_batch` envelope (cycle 11 wired the outbound C++
API).

The buffering layer is the bridge between the synchronous robot-
side caller and the tick-paced wire protocol. It introduces
several design decisions that future outbound surfaces
(HAL_CAN_SendMessage, HAL_Notifier*) will inherit:

- Where does the buffer state live?
- When does the buffer flush?
- How does overflow get handled?
- How are caller-side variable-size payloads (strings) truncated to
  fit our fixed-size schema fields?
- What happens to the buffer on transport failure?
- What happens to the buffer on shutdown?

Pinning these for HAL_SendError sets the precedent. The design
decisions D-C19-* are explicitly worded so HAL_CAN_SendMessage and
HAL_Notifier* can inherit them without re-litigation.

WPILib signature (verified against `kWpilibParitySha`):

```c
int32_t HAL_SendError(HAL_Bool isError, int32_t errorCode, HAL_Bool isLVCode,
                      const char* details, const char* location,
                      const char* callStack, HAL_Bool printMsg);
```

Returns int32_t directly (NOT through a status pointer — different
calling convention from cycles 12–18's HAL_Get*). Returns 0 on
success, the error code on failure.

---

## Contract under test

### New public surface on `shim_core`

```cpp
class shim_core {
 public:
  // ... existing make / poll / send_* / observers ...

  // Cycle 19: outbound-buffering surface. Append `msg` to the shim's
  // pending-flush buffer. If the buffer is at capacity
  // (kMaxErrorsPerBatch == 8), the message is **dropped** silently
  // (D-C19-OVERFLOW-DROP). After shutdown the message is still
  // appended to the buffer (no observable side effect since
  // flush_pending_errors short-circuits post-shutdown — the buffer
  // becomes a write-only sink); future cycles may add a
  // post-shutdown enqueue gate. Single-threaded v0; the future
  // threading cycle adds locking.
  void enqueue_error(const error_message& msg) noexcept;

  // Cycle 19: build an error_message_batch from the pending buffer
  // and publish it via send_error_message_batch (cycle 11). On
  // success, clears the buffer (D-C19-FLUSH-CLEARS-ON-SUCCESS).
  // On transport failure, KEEPS the buffer for retry on the next
  // flush attempt (D-C19-FLUSH-RETAINS-ON-FAILURE). Empty buffer is
  // a success no-op without sending an envelope
  // (D-C19-EMPTY-FLUSH-NO-OP). After shutdown returns
  // shutdown_already_observed without touching the buffer or the
  // outbound lane (D-C19-FLUSH-SHUTDOWN-TERMINAL).
  [[nodiscard]] std::expected<void, shim_error> flush_pending_errors(
      std::uint64_t sim_time_us);

  // Cycle 19: observer for the pending-flush buffer. Returns a span
  // over the **active prefix** of the buffer (length
  // 0..kMaxErrorsPerBatch). Used by tests to verify enqueue / flush /
  // overflow behavior. Implementation:
  // `return {pending_error_messages_.data(), pending_error_count_};`
  [[nodiscard]] std::span<const error_message> pending_error_messages() const noexcept;
};
```

### New extern "C" surface

`src/backend/shim/hal_c.h`:

```cpp
extern "C" {

// ... existing ...

// Mirrors WPILib HAL_SendError from hal/include/hal/DriverStation.h.
// Synchronous robot-side log call. The shim translates the seven
// arguments into a fully-formed `error_message` (with copy_truncated
// applied to each string and truncation_flags set accordingly) and
// enqueues it on the installed shim's pending-flush buffer
// (D-C19-SEAM-CONSTRUCTS-ERROR-MESSAGE). The integrator drives
// `shim->flush_pending_errors(sim_time_us)` at tick boundary to
// actually send the accumulated messages as one
// `error_message_batch` envelope.
//
// Return: 0 on success (buffered); kHalHandleError if no shim is
// installed (uniform "shim not installed" signal across HAL
// surfaces per OQ-C19-NO-SHIM-RETURN-VALUE).
//
// NULL string pointers are treated as empty strings
// (D-C19-NULL-STRING-AS-EMPTY); the corresponding string field is
// left zero-init. Strings longer than the schema's fixed buffer
// are truncated via copy_truncated and the corresponding bit in
// truncation_flags is set.
std::int32_t HAL_SendError(HAL_Bool isError, std::int32_t errorCode,
                           HAL_Bool isLVCode, const char* details,
                           const char* location, const char* callStack,
                           HAL_Bool printMsg);

}  // extern "C"
```

### Internal additions

- `shim_core.h`: add private members
  `std::array<error_message, kMaxErrorsPerBatch> pending_error_messages_{};`
  and `std::uint32_t pending_error_count_ = 0;` (round-1 reviewer
  endorsed this shape over `std::vector` because it avoids heap
  allocation on the hot path, expresses the capacity bound
  structurally, and simplifies `flush_pending_errors`'s
  `error_message_batch` construction to a direct `std::copy` of
  the active prefix).
- `shim_core.cpp`: implementations of `enqueue_error`,
  `flush_pending_errors`, `pending_error_messages` observer.
- `hal_c.cpp`: `HAL_SendError` implementation. Uses
  `copy_truncated` from `truncate.h` for each of the three string
  fields. NULL pointers map to empty `std::string_view{}` (since
  `string_view` from a null `const char*` is UB in the
  `string_view(const char*)` constructor, the implementation must
  guard against NULL with `(p != nullptr) ? std::string_view{p} :
  std::string_view{};` per D-C19-NULL-STRING-AS-EMPTY).

### Out of scope

- Auto-flush on poll(). Plan picks **explicit-flush** per
  D-C19-EXPLICIT-FLUSH; the integrator's main loop has the structure
  `robot_tick() → flush_pending_errors() → poll()`.
- Multi-tick batching beyond `kMaxErrorsPerBatch`. Overflow drops
  in v0; future cycle may add an overflow counter for diagnostics.
- HAL_SendConsoleLine (separate WPILib function with different
  semantics). Future cycle.
- Threading. Single-threaded v0; the future threading cycle adds
  locking around the buffer.
- Message coalescing (e.g. duplicate-error suppression). v0 sends
  every distinct enqueue.
- A "you have unflushed errors" warning at destruction. Future
  cycle may add for diagnostics.
- Overflow-counter persistence in the schema. The wire format
  doesn't carry "you dropped N messages"; if needed a future
  schema cycle adds it.

---

## Decisions pinned

### New: D-C19-BUFFER-IN-SHIM-CORE

The pending-flush buffer is a private member of `shim_core` (not a
file-static in `hal_c.cpp`). The C ABI seam (`HAL_SendError`)
looks up the installed shim via `shim_core::current()` and calls
`shim->enqueue_error(msg)`. Tests pin this via
`shim.pending_error_messages()` observer.

**Rationale:** the buffer holds per-shim state. A file-static would
be process-global like `g_installed_shim_`, but the buffer's
lifecycle is tied to a specific shim instance (errors enqueued
before a shim is installed go nowhere; errors enqueued for shim A
should not flush through shim B). Putting it on `shim_core` makes
the lifecycle clear: when `shim_core` is destroyed, the buffer
goes with it.

### New: D-C19-OVERFLOW-DROP

When `enqueue_error` is called and the buffer is at
`kMaxErrorsPerBatch`, the new message is **dropped silently**. No
error code, no buffer rotation, no FIFO eviction. Tests pin this
via `pending_error_messages().size() == kMaxErrorsPerBatch` after
N+1 enqueues with N = `kMaxErrorsPerBatch`.

**Rationale:** simplest behavior. The buffer is the shim's queue
for the current tick; if it's full, the integrator hasn't flushed
yet and the next flush will get the messages it has. Dropping new
ones (rather than rotating) preserves the order of the messages
already enqueued.

**Future tightening:** if production reveals dropped messages are
a real problem, a future cycle adds an "overflow drop count" field
to `error_message_batch` (would require a schema bump) and emits
a synthetic "errors were dropped this tick" message at flush.

### New: D-C19-EXPLICIT-FLUSH

`flush_pending_errors` is **explicit, caller-driven**. Not auto-on-
poll, not on a timer. The integrator's main loop has the structure:

```cpp
while (running) {
  robot_tick();  // robot calls HAL_SendError synchronously, buffers up.
  shim.flush_pending_errors(sim_time_us);  // sends accumulated.
  shim.poll();  // processes inbound from sim core.
}
```

**Rationale:** explicit-flush gives the integrator control over
"when does the wire get touched". Auto-on-poll would couple inbound
cadence to outbound cadence in ways the cycle-1 design avoided
(`poll()` is nonblocking and side-effect-free for the wire). A
timer-based flush would introduce wall-clock dependence,
violating the determinism non-negotiable.

### New: D-C19-EMPTY-FLUSH-NO-OP

If `flush_pending_errors` is called with an empty buffer, it
returns success WITHOUT sending an envelope. The lane is not
touched, the session counter is not advanced.

**Rationale:** sending a zero-length `error_message_batch` would
work (cycle 11 verified empty batches) but is wasted bandwidth.
Empty-flush-no-op is the natural semantic for "no errors to send
this tick".

### New: D-C19-FLUSH-CLEARS-ON-SUCCESS

After `send_error_message_batch` returns success,
`flush_pending_errors` clears the pending buffer. Next call starts
fresh.

### New: D-C19-FLUSH-RETAINS-ON-FAILURE

If `send_error_message_batch` returns failure (e.g. `lane_busy`),
`flush_pending_errors` propagates the error AND **keeps the buffer
unchanged**. The integrator's next flush attempt retries with the
same accumulated messages plus any new ones added in the meantime
(subject to D-C19-OVERFLOW-DROP).

**Rationale:** `lane_busy` means SimCore hasn't drained the
previous tick's batch. The batch we wanted to send is still
unsent. Dropping our buffer would lose those errors entirely.
Retaining lets the next flush succeed once the lane drains. The
overflow-drop rule still applies, so a perpetually-full lane will
still drop new messages — but the existing 8 stay until they get
through.

### New: D-C19-FLUSH-SHUTDOWN-TERMINAL

After `is_shutting_down() == true`, `flush_pending_errors` returns
`shutdown_already_observed` immediately without touching the
buffer or the outbound lane. Mirrors cycle-9's
D-C9-SHUTDOWN-TERMINAL on the typed-send methods.

### New: D-C19-SEAM-CONSTRUCTS-ERROR-MESSAGE

The HAL_SendError C ABI seam (in `hal_c.cpp`) is responsible for
constructing the `error_message` from the seven WPILib parameters,
including string truncation via `copy_truncated`. `shim_core::enqueue_error`
takes a fully-formed `error_message` by const reference; it does
**not** know about WPILib's parameter list.

**Rationale:** keeps `shim_core` decoupled from WPILib's HAL surface.
Future C HAL surfaces that route to `enqueue_error` would
construct their own `error_message` at their own seam (though
HAL_SendError is the only such caller for v0).

### New: D-C19-NULL-STRING-AS-EMPTY

A NULL `const char*` parameter to HAL_SendError is treated as an
empty string. The corresponding `error_message` string field is
left zero-init (a single null terminator at offset 0). The
`truncation_flags` bit for that field is **not** set (an empty
string fits without truncation).

**Rationale:** WPILib's robot-side callers may pass NULL for
optional fields (e.g. an error with no callStack). Crashing the
robot via UB on `string_view(nullptr)` is unfaithful to the
"robot code is unmodified" non-negotiable. Treating NULL as empty
preserves the message's other fields and the error reaches the
sim core.

### Inherited unchanged

- D-C12-GLOBAL-ACCESSOR (HAL_SendError uses `shim_core::current()`).
- D-C12-NULL-SHIM-IS-HANDLE-ERROR (HAL_SendError without a shim
  returns `kHalHandleError` per OQ-C19-NO-SHIM-RETURN-VALUE).
- D-C15-HAL-BOOL-SIGNED-PARITY (the three HAL_Bool params).

### Resolved question on the no-shim return value

WPILib's HAL_SendError returns 0 on success, errorCode on failure.
The cycle-12 `kHalHandleError = -1` convention applied to HAL_Get*
functions where the return value is "the data". For HAL_SendError
the return value is "did this go through". Round-1 reviewer resolved
OQ-C19-NO-SHIM-RETURN-VALUE in favor of:

Return `kHalHandleError` (-1) when no shim is installed. It is the
universal "shim not installed" signal across HAL surfaces, regardless
of return-value semantic. The robot doesn't typically check
HAL_SendError's return; consistency with the rest of the C ABI surface
matters more.

---

## Test fixtures

No new helpers. Reuse `valid_error_message` from `test_helpers.h`,
`shim_global_install_guard`, and `make_connected_shim`.

---

## Determinism notes

The buffer is a fixed-capacity
`std::array<error_message, kMaxErrorsPerBatch>` plus an active
count. Its state evolves deterministically with caller actions.
`enqueue_error` always appends (or drops on overflow).
`flush_pending_errors` clears on success / retains on failure /
no-ops on empty. No threading, no RNG, no wall-clock. Cycle 8's
existing `error_message_batch` byte-identity replay covers the
wire-format determinism. No new determinism test in cycle 19.

---

## Proposed tests (revision 3)

### C19-1. `HalSendError.WithNoShimInstalledReturnsHandleError`

- **Layer / contract:** D-C19-NO-SHIM-RETURNS-HANDLE-ERROR (the
  HAL_SendError variant of D-C12-NULL-SHIM-IS-HANDLE-ERROR; per
  OQ-C19-NO-SHIM-RETURN-VALUE).
- **Setup:**
  - `shim_core::install_global(nullptr);`
  - `ASSERT_EQ(shim_core::current(), nullptr);`
- **Action:**
  ```cpp
  std::int32_t r = HAL_SendError(/*isError=*/1, /*errorCode=*/100,
                                 /*isLVCode=*/0, "details", "loc",
                                 "stack", /*printMsg=*/1);
  ```
- **Expected:** `r == kHalHandleError` (-1).
- **Bug class:** function dereferences null shim (UB); function
  returns 0 (would silently lose the error and confuse the robot).

### C19-2. `ShimCoreEnqueueError.AppendsMessageToPendingBuffer`

- **Layer / contract:** D-C19-BUFFER-IN-SHIM-CORE; the basic
  enqueue contract.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - `ASSERT_EQ(shim.pending_error_messages().size(), 0u);`
- **Action:**
  - Construct `error_message msg = valid_error_message(/*error_code=*/42, /*severity=*/1, /*is_lv_code=*/0, /*print_msg=*/1, /*truncation_flags=*/0, "details", "loc", "stack");`
  - `shim.enqueue_error(msg);`
- **Expected:**
  - `r == 0` (success).
  - `shim.pending_error_messages().size() == 1u`.
  - `shim.pending_error_messages()[0] == msg` (defaulted operator==
    covers every byte per D-C8-PADDING-FREE).
- **Bug class:** function does nothing; function appends a
  default-constructed message; function appends to the wrong
  buffer; function moves rather than copies (wouldn't break this
  test but is worth flagging).

### C19-3. `HalSendError.WithShimInstalledEnqueuesConstructedErrorMessage`

- **Layer / contract:** D-C19-SEAM-CONSTRUCTS-ERROR-MESSAGE;
  end-to-end HAL_SendError → enqueue_error.
- **Setup:**
  - Fresh region; shim made.
  - `shim_global_install_guard guard{shim};`
  - `ASSERT_EQ(shim.pending_error_messages().size(), 0u);`
- **Action:**
  ```cpp
  std::int32_t r = HAL_SendError(
      /*isError=*/1, /*errorCode=*/0xCAFE, /*isLVCode=*/0,
      "the details", "the location", "the call stack",
      /*printMsg=*/1);
  ```
- **Expected:**
  - `r == 0` (success).
  - `shim.pending_error_messages().size() == 1u`.
  - The buffered message satisfies `operator==` against a reference:
    ```cpp
    const auto expected = valid_error_message(
        /*error_code=*/0xCAFE, /*severity=*/1, /*is_lv_code=*/0,
        /*print_msg=*/1, /*truncation_flags=*/0,
        "the details", "the location", "the call stack");
    EXPECT_EQ(shim.pending_error_messages()[0], expected);
    ```
    Defaulted `error_message::operator==` covers every byte
    including `reserved_pad[3]` (per D-C8-PADDING-FREE), so this
    one assertion catches both per-field correctness AND
    `reserved_pad` zero-init in the buffered message — closing
    the round-1 reviewer's concern that per-field assertions
    alone could miss dirty reserved_pad bytes.
- **Bug class:** function reverses parameter order (e.g. swaps
  errorCode and isLVCode); function forgets to copy a string
  field; function doesn't zero-init the message before copying
  (would leave stack noise in the unused tail of each fixed-size
  buffer); function maps WPILib's HAL_Bool int32 to our hal_bool
  with a sign error.

### C19-4. `HalSendError.TruncatesLongStringsAndSetsTruncationFlags`

- **Layer / contract:** D-C19-NULL-STRING-AS-EMPTY's sibling: the
  truncation path. Pins that copy_truncated is invoked with the
  correct dest buffer for each field, and that the truncation
  flags are set per-field.
- **Setup:**
  - Fresh region; shim made; install guard installed.
  - Three long strings:
    - `details_long`: 2000 'A' chars (longer than `kErrorDetailsLen`
      = 1024).
    - `location_long`: 500 'B' chars (longer than
      `kErrorLocationLen` = 256).
    - `call_stack_long`: 2000 'C' chars (longer than
      `kErrorCallStackLen` = 1024).
- **Action:**
  ```cpp
  HAL_SendError(1, 0xBABE, 0, details_long.c_str(),
                location_long.c_str(), call_stack_long.c_str(), 1);
  ```
- **Expected:**
  - `shim.pending_error_messages().size() == 1u`.
  - The buffered message has:
    - `truncation_flags == kErrorTruncDetails | kErrorTruncLocation
       | kErrorTruncCallStack` (= `0b0000'0111` = 7).
    - `details` is filled with 1023 'A' chars + 1 null at index 1023.
    - `location` is filled with 255 'B' chars + 1 null at index 255.
    - `call_stack` is filled with 1023 'C' chars + 1 null at index 1023.
- **Bug class:** function uses the wrong dest size for one of the
  fields (e.g. pass `kErrorDetailsLen` to `location` slot); function
  forgets to set the truncation flag; function sets all three flags
  unconditionally even when no truncation occurred.

### C19-5. `HalSendError.NullStringPointersAreTreatedAsEmptyAndDoNotSetTruncationFlags`

- **Layer / contract:** D-C19-NULL-STRING-AS-EMPTY.
- **Setup:** fresh region; shim made; install guard installed.
- **Action:**
  ```cpp
  HAL_SendError(1, 0x1234, 0, /*details=*/nullptr,
                /*location=*/nullptr, /*callStack=*/nullptr, 1);
  ```
- **Expected:**
  - `r == 0` (success).
  - `shim.pending_error_messages().size() == 1u`.
  - The buffered message satisfies `operator==` against:
    ```cpp
    valid_error_message(/*error_code=*/0x1234, /*severity=*/1,
                        /*is_lv_code=*/0, /*print_msg=*/1,
                        /*truncation_flags=*/0, "", "", "")
    ```
    This covers `severity`, `is_lv_code`, `print_msg`,
    `reserved_pad[3]`, `truncation_flags == 0`, and all three
    zero-initialized string buffers. NULL is empty, not truncated.
- **Bug class:** function dereferences NULL (UB, ASan-detectable);
  function sets truncation_flags for NULL; function leaves the
  fields uninitialized (would leak stack noise).

### C19-6. `ShimCoreEnqueueError.OverflowDropsNewMessagesAtCapacity`

- **Layer / contract:** D-C19-OVERFLOW-DROP.
- **Setup:** fresh region; shim made; no install guard needed
  (this tests the shim_core API directly, not HAL_SendError).
- **Action:**
  - Enqueue 8 distinct messages (`error_code` values 1, 2, …, 8).
  - Capture `shim.pending_error_messages()` (should have 8 entries).
  - Enqueue a 9th distinct message (`error_code = 999`).
- **Expected:**
  - `shim.pending_error_messages().size() == 8u` (still — overflow
    dropped).
  - The buffer holds the FIRST 8 enqueues (error_code 1 through 8),
    NOT a rotation that includes 999.
  - Specifically: `shim.pending_error_messages()[7].error_code == 8`,
    and no entry has `error_code == 999`.
- **Bug class:** function rotates the buffer (would put 999 at
  index 7 and drop error_code 1); function silently grows the
  active count past capacity (would have size 9, breaking the
  fixed-batch wire-format invariant); function returns an error
  code from `enqueue_error` (we said it's `void`/`noexcept`).

### C19-7. `ShimCoreFlushPendingErrors.EmptyBufferIsSuccessNoOpWithoutTouchingLane`

- **Layer / contract:** D-C19-EMPTY-FLUSH-NO-OP.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - Verify boot envelope drained, lane empty:
    `ASSERT_EQ(region.backend_to_core.state.load(),
    static_cast<std::uint32_t>(tier1_lane_state::empty));`
  - `ASSERT_EQ(shim.pending_error_messages().size(), 0u);`
- **Action:**
  `auto r = shim.flush_pending_errors(/*sim_time_us=*/250'000);`
- **Expected:**
  - `r.has_value() == true` (success).
  - Lane still empty:
    `region.backend_to_core.state.load() ==
    static_cast<std::uint32_t>(tier1_lane_state::empty)`.
  - Buffer still empty:
    `shim.pending_error_messages().size() == 0u`.
- **Bug class:** function sends a count=0 envelope (would touch
  the lane and advance the session counter wastefully); function
  returns failure on empty buffer.

### C19-8. `ShimCoreFlushPendingErrors.PublishesAccumulatedMessagesAsTickBoundaryEnvelope`

- **Layer / contract:** D-C19-FLUSH-CLEARS-ON-SUCCESS plus
  end-to-end enqueue → flush → wire round-trip via
  `send_error_message_batch`.
- **Setup:**
  - Fresh region; shim made; core peer drained boot.
  - Enqueue 3 distinct messages
    (`error_code` 100, 200, 300; distinct `severity`,
    `is_lv_code`, `print_msg`, `truncation_flags`,
    and string fields per cycle-11's C11-1 fixture pattern).
- **Action:**
  `auto r = shim.flush_pending_errors(/*sim_time_us=*/500'000);`
- **Expected:**
  - `r.has_value() == true`.
  - Core peer's `try_receive()` returns a `tier1_message` with:
    - `envelope.kind == envelope_kind::tick_boundary`.
    - `envelope.payload_schema == schema_id::error_message_batch`.
    - `envelope.sender == direction::backend_to_core`.
    - `envelope.sequence == 1` (boot took 0).
    - `envelope.sim_time_us == 500'000`.
    - `envelope.payload_bytes == 6980` (= 8 + 3*2324).
  - **Deserialization mechanism (round-1 reviewer required
    clarification; round-2 reviewer required reserved-pad coverage):**
    `std::memcpy` the received `payload` bytes
    into a zero-initialized destination
    `error_message_batch destination{}` and then assert
    `destination == valid_error_message_batch({msg0, msg1, msg2})`
    via defaulted `error_message_batch::operator==`. That assertion
    covers `count`, `reserved_pad[4]`, all three active messages
    including their `reserved_pad[3]` fields, and the zero-initialized
    unused `messages[3..7]` tail in the destination. The memcpy length
    is `received->payload.size()` (i.e. 6980 — the active prefix);
    the zero-init of `destination` ensures the unused
    `messages[3..7]` tail is byte-zero, NOT garbage from the wire
    (which would only be a concern if the implementation
    incorrectly sent more than the active prefix — and that
    failure mode is already caught by the `payload_bytes == 6980`
    assertion above). The explicit `operator==` check is load-bearing
    for a bug where `flush_pending_errors` constructs
    `error_message_batch batch;` without zero-init, sets only
    `count` and `messages`, and leaks nondeterministic
    `reserved_pad[4]` bytes onto the wire.
  - Buffer cleared:
    `shim.pending_error_messages().size() == 0u`.
- **Bug class:** flush does not call `send_error_message_batch`;
  flush does not clear the buffer on success; flush clears the
  buffer when send fails; flush sends the wrong sim_time_us;
  flush includes garbage in the unused `messages[3..7]` tail
  (would fail wire-format byte-identity if cycle-11's
  active-prefix discipline isn't maintained — but cycle-11
  pinned this in production via `active_prefix_bytes`).

### C19-8b. `ShimCoreFlushPendingErrors.PublishesFullCapacityBatchAtKMaxErrorsPerBatchBoundary`

- **Layer / contract:** D-C19-FLUSH-CLEARS-ON-SUCCESS at the
  capacity boundary; pins that `flush_pending_errors`'s
  fixed-array active prefix → `error_message_batch` construction handles the
  `count == kMaxErrorsPerBatch == 8` case correctly. Round-1
  reviewer required this test because the new buffering layer
  builds the batch from a fixed-capacity array plus count and the maximum-capacity
  arithmetic (`8 + 8 * 2324 = 18600 == sizeof(error_message_batch)`)
  is the boundary where the active-prefix optimization equals
  the full struct size — a place where off-by-one bugs in the
  count vs. the array-fill could hide.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`; core peer
    drained boot.
  - Enqueue **exactly 8** distinct messages
    (`error_code` 1 through 8; per-message distinguishing fields
    consistent with cycle-11's C11-1 fixture pattern).
  - `ASSERT_EQ(shim.pending_error_messages().size(), 8u);`
- **Action:**
  `auto r = shim.flush_pending_errors(/*sim_time_us=*/750'000);`
- **Expected:**
  - `r.has_value() == true`.
  - Core peer's `try_receive()` returns a `tier1_message` with:
    - `envelope.payload_bytes == 18600` (= 8 + 8*2324, exactly
      `sizeof(error_message_batch)`; this is the only outbound
      v0 schema where the active prefix fills the whole struct).
    - `envelope.payload_schema == schema_id::error_message_batch`.
    - `envelope.sequence == 1`.
    - `envelope.sim_time_us == 750'000`.
  - Deserialization mechanism (same as C19-8): memcpy payload
    into `error_message_batch destination{}` and assert
    `destination == valid_error_message_batch(all_8_messages)` via
    defaulted `error_message_batch::operator==`. This covers
    `count`, the header `reserved_pad[4]`, and each
    `destination.messages[i] == msg_i` for `i` in `[0, 8)`.
  - Buffer cleared:
    `shim.pending_error_messages().size() == 0u`.
- **Bug class:** off-by-one in count assignment (would set
  `batch.count = 7` for 8-message buffer, sending a 7-message
  batch and dropping the 8th); off-by-one in the std::copy bound
  (would skip messages[7]); active-prefix arithmetic that special-
  cases the full-capacity case incorrectly (e.g. uses
  `sizeof(error_message_batch) + 8` instead of the offsetof-based
  formula).

### C19-9. `ShimCoreFlushPendingErrors.RetainsBufferOnTransportFailureAndPropagatesError`

- **Layer / contract:** D-C19-FLUSH-RETAINS-ON-FAILURE.
- **Setup:**
  - Fresh region; shim made via `make` (boot envelope occupies
    `region.backend_to_core` — lane is `full`). Use
    `valid_boot_descriptor()` so the boot bytes are deterministic
    and re-constructible for the post-action assertion.
  - Core peer does NOT drain the boot envelope. Lane remains busy.
  - Enqueue 2 messages.
  - Capture pre-flush buffer state for non-clobber assertion.
  - **Capture pre-flush boot-envelope payload bytes** for the
    lane-non-corruption assertion (round-1 reviewer required —
    mirrors C9-3's pattern for `send_can_frame_batch`):
    ```cpp
    const auto desc = valid_boot_descriptor();
    std::vector<std::uint8_t> boot_bytes_before(
        sizeof(boot_descriptor));
    std::memcpy(boot_bytes_before.data(), &desc,
                sizeof(boot_descriptor));
    ```
    (Equivalent: read the bytes directly out of
    `region.backend_to_core.payload` before the action.)
- **Action:**
  `auto r = shim.flush_pending_errors(/*sim_time_us=*/500'000);`
- **Expected:**
  - `r.has_value() == false`.
  - `r.error().kind == shim_error_kind::send_failed`.
  - `r.error().transport_error.has_value() == true` and
    `r.error().transport_error->kind ==
    tier1_transport_error_kind::lane_busy`.
  - Buffer retained: `shim.pending_error_messages().size() == 2u`,
    contents byte-equal to the captured pre-flush state.
  - Lane still `full`.
  - **Lane payload not corrupted (round-1 reviewer required):**
    `std::memcmp(region.backend_to_core.payload.data(),
    boot_bytes_before.data(), sizeof(boot_descriptor)) == 0` —
    proves the failed flush did NOT clobber the boot envelope
    mid-write before reporting the error. Without this assertion
    a "flush corrupted the lane mid-write then returned
    `lane_busy`" bug would silently pass.
- **Bug class:** flush clears the buffer on failure (would lose
  the 2 messages); flush rotates / partially clears the buffer;
  flush corrupts the lane payload mid-write before reporting
  failure (the new boot-bytes assertion catches this — was a real
  bug class in cycle-9's C9-3 round 1).

### C19-10. `ShimCoreFlushPendingErrors.PostShutdownReturnsTerminalErrorWithoutTouchingBufferOrLane`

- **Layer / contract:** D-C19-FLUSH-SHUTDOWN-TERMINAL. Mirror of
  cycle 9's `PostShutdownSendIsRejectedWithoutTouchingLane`.
- **Setup:**
  - Fresh region; shim made via `make_connected_shim`.
  - Core peer sends `shutdown` envelope at sequence 1; shim polls
    (`is_shutting_down() == true`).
  - Enqueue 2 messages (D-C19-OVERFLOW-DROP doesn't reject post-
    shutdown enqueue per D-C19's "no enqueue gate" decision).
  - `ASSERT_EQ(region.backend_to_core.state.load(),
    static_cast<std::uint32_t>(tier1_lane_state::empty));`
    (boot was drained by `make_connected_shim`).
- **Action:**
  `auto r = shim.flush_pending_errors(/*sim_time_us=*/500'000);`
- **Expected:**
  - `r.has_value() == false`.
  - `r.error().kind == shim_error_kind::shutdown_already_observed`.
  - `r.error().transport_error == std::nullopt` — proves the
    short-circuit fired BEFORE `endpoint_.send` was called.
  - Buffer retained:
    `shim.pending_error_messages().size() == 2u`.
  - Lane still empty.
- **Bug class:** flush calls `send_error_message_batch` after
  shutdown (would surface as `transport_error.has_value() == true`);
  flush clears the buffer post-shutdown (would lose the 2
  messages, which is fine functionally but means the next-tick
  attempt has nothing to retry).

---

## Tests deliberately not added

- **Idempotency mirror.** Shim_core's enqueue / flush APIs are
  state-mutating; "calling them twice produces the same result"
  is not a meaningful contract.
- **NULL status guard.** HAL_SendError doesn't take a status
  pointer.
- **Cross-cutting cycle-12 mirrors.** The session-level contracts
  (install / current / shim lookup) are exercised via C19-1 (no
  shim) and C19-3 (with shim) implicitly.
- **Determinism replay.** Cycle 8 covers `error_message_batch`
  byte-identity.
- **Threading** — single-threaded v0.
- **Auto-flush-on-poll.** Plan picks D-C19-EXPLICIT-FLUSH; no
  auto-flush behavior to test.
- **Multiple flush-then-enqueue cycles.** A combinator test that
  walks through enqueue → flush → enqueue → flush would exercise
  the buffer's state machine across boundaries, but each transition
  is already covered: empty-flush (C19-7), enqueue-then-flush
  (C19-8), flush-clears-on-success (C19-8 buffer assertion),
  enqueue-after-flush is a fresh enqueue (C19-2). Combining them
  catches no new bug class.
- **HAL_SendError return value on transport failure.** HAL_SendError
  ENQUEUES; it does not flush. The transport failure happens at
  flush time, not at HAL_SendError time. So HAL_SendError can
  always return 0 (success) when a shim is installed; failure to
  reach SimCore surfaces only when the integrator calls flush. This
  matches WPILib's "HAL_SendError is fire-and-forget at the C ABI
  level" semantic.

---

## Cross-test invariants

Same as cycles 12–18 plus:
- `valid_error_message` is the canonical fixture; pass explicit
  string args rather than relying on the helper's empty-string
  defaults when a test cares about field content.
- C19-2, C19-6 do NOT use the `shim_global_install_guard` because
  they exercise the shim_core API directly, not the C ABI.
- C19-1 / C19-3 / C19-4 / C19-5 use the install guard.
- C19-7 / C19-8 / C19-9 / C19-10 mix shim_core API and lane-state
  inspection but do NOT need the C HAL global, so they don't use
  the guard.

---

## Implementation companion recommendations

- **Order of operations:**
  1. Add the array-shape buffer state to `shim_core.h`:
     `std::array<error_message, kMaxErrorsPerBatch> pending_error_messages_{};`
     and `std::uint32_t pending_error_count_ = 0;`.
  2. Add `enqueue_error`, `pending_error_messages` observer, and
     `flush_pending_errors` to `shim_core.{h,cpp}`. `flush_pending_errors`
     calls the existing `send_error_message_batch` (cycle 11).
  3. Build; run cycle-1 through cycle-18 tests (still 461 green).
  4. Add `HAL_SendError` declaration to `hal_c.h`.
  5. Add `HAL_SendError` stub to `hal_c.cpp` (returns 0
     unconditionally without doing anything — broken by design).
  6. Add the 11 cycle-19 tests; confirm they fail.
  7. Replace the stub with the real implementation: lookup shim,
     construct `error_message` via `copy_truncated` for each string,
     handle NULL pointers as empty, set `truncation_flags`
     appropriately, call `enqueue_error`.
  8. Confirm 472/472 green under both matrices.
- **The `copy_truncated` semantics:** returns `bool` true if the
  string was truncated. The implementation should:
  ```cpp
  msg.truncation_flags = 0;
  if (copy_truncated(msg.details, details_view))
      msg.truncation_flags |= kErrorTruncDetails;
  if (copy_truncated(msg.location, location_view))
      msg.truncation_flags |= kErrorTruncLocation;
  if (copy_truncated(msg.call_stack, call_stack_view))
      msg.truncation_flags |= kErrorTruncCallStack;
  ```
- **NULL pointer handling:** before calling `copy_truncated`,
  convert `const char*` to `std::string_view` safely:
  ```cpp
  auto safe_view = [](const char* p) {
    return p ? std::string_view{p} : std::string_view{};
  };
  ```
- **The `error_message{}` zero-init is load-bearing.** The dest
  buffers must be zero before `copy_truncated` runs, so the unused
  tail past the truncation point stays zero. This also handles
  the NULL-string case (no copy, all zero).
- **`enqueue_error` body sketch (array shape):**
  ```cpp
  void shim_core::enqueue_error(const error_message& msg) noexcept {
    if (pending_error_count_ >= kMaxErrorsPerBatch) {
      return;  // D-C19-OVERFLOW-DROP
    }
    pending_error_messages_[pending_error_count_++] = msg;
  }
  ```
- **`pending_error_messages` observer body sketch:**
  ```cpp
  std::span<const error_message> shim_core::pending_error_messages()
      const noexcept {
    return {pending_error_messages_.data(), pending_error_count_};
  }
  ```
- **`flush_pending_errors` body sketch (array shape):**
  ```cpp
  std::expected<void, shim_error> shim_core::flush_pending_errors(
      std::uint64_t sim_time_us) {
    if (shutdown_observed_) {
      return std::unexpected(shim_error{
          shim_error_kind::shutdown_already_observed,
          std::nullopt, "kind",
          "shim already observed shutdown; refusing flush"});
    }
    if (pending_error_count_ == 0) {
      return {};  // D-C19-EMPTY-FLUSH-NO-OP
    }
    error_message_batch batch{};
    batch.count = pending_error_count_;
    std::copy(pending_error_messages_.begin(),
              pending_error_messages_.begin() + pending_error_count_,
              batch.messages.begin());
    auto sent = send_error_message_batch(batch, sim_time_us);
    if (!sent.has_value()) {
      return std::unexpected(std::move(sent.error()));  // D-C19-FLUSH-RETAINS-ON-FAILURE
    }
    pending_error_count_ = 0;  // D-C19-FLUSH-CLEARS-ON-SUCCESS
    // pending_error_messages_ tail past count_ is left at the last
    // values; the count gates which entries are observable.
    return {};
  }
  ```

---

## Open questions

**OQ-C19-NO-SHIM-RETURN-VALUE.** **Resolved at round 1 in favor of
`kHalHandleError` (-1).** Reviewer endorsed: uniform "shim not
installed" signal across the C HAL surface beats matching WPILib's
"errorCode on failure" convention since robot code rarely checks
HAL_SendError's return value.

**OQ-C19-VECTOR-VS-ARRAY-FOR-BUFFER.** **Resolved at round 1 in
favor of `std::array<error_message, kMaxErrorsPerBatch> + uint32_t
count`.** Reviewer's reasoning: (a) avoids heap allocation on the
hot path (HAL_SendError per error per tick); (b) expresses the
capacity bound structurally rather than through a runtime size
check; (c) simplifies `flush_pending_errors`'s
`error_message_batch` construction to `std::copy` over the active
prefix; (d) the 18 KB stack footprint per shim is acceptable for
a per-shim singleton. Round-2 adopts.

**OQ-C19-EXPLICIT-vs-AUTO-FLUSH.** **Resolved at round 1 in favor
of D-C19-EXPLICIT-FLUSH.** Reviewer endorsed: auto-on-poll would
break cycle 1's "poll is side-effect-free for the wire" design and
remove integrator control over tick composition (e.g. "send CAN
frames AND errors in the same tick"). The "integrator forgets to
flush" risk is real but symmetric with "integrator forgets to
poll" — addressable via future warning-at-destruction without
changing the contract.

**OQ-C19-POST-SHUTDOWN-ENQUEUE-GATE.** **Resolved at round 1 in
favor of allow.** Reviewer endorsed: rejecting post-shutdown
enqueue would require changing `enqueue_error`'s signature from
`void noexcept` to `expected<void, shim_error>`, propagating an
unexpected failure return back through HAL_SendError. Robot code
calling HAL_SendError during shutdown should not get an
unexpected failure; messages are simply lost.

**OQ-C19-DROP-COUNTER.** **Resolved at round 1 as defer.** Future
cycle adds an "overflow drop count" field to `error_message_batch`
if production reveals it's needed; would require a schema bump.
