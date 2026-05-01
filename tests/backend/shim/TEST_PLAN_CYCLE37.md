# HAL shim core - cycle 37 test plan (joystick output publication)

Status: implemented

Cycle 37 publishes the joystick output state recorded in cycle 36 through a
dedicated protocol schema and an explicit shim flush boundary. It does not
publish cycle 35 user-program observer state.

Primary references:

- WPILib 2026.2.2 release C++ docs for `HAL_SetJoystickOutputs` define the C
  HAL surface that produces joystick output/rumble commands.
- `tests/backend/shim/TEST_PLAN_CYCLE36.md` pins the existing per-shim storage:
  valid slots `0..5`, raw signed C ABI values preserved, repeated writes are
  latest-wins per slot, and `HAL_SetJoystickOutputs` itself does not publish.
- `.claude/skills/protocol-schema.md` requires a `kProtocolVersion` bump when a
  new `schema_id` is added.

## Slice decision

The smallest coherent Cycle 37 slice is a joystick-output-only outbound schema
plus host-facing shim snapshot/flush methods:

```cpp
struct joystick_output_state {
  std::int32_t joystick_num;
  std::array<std::uint8_t, 4> reserved_pad;
  std::int64_t outputs;
  std::int32_t left_rumble;
  std::int32_t right_rumble;
};

struct joystick_output_batch {
  std::uint32_t count;
  std::array<std::uint8_t, 4> reserved_pad;
  std::array<joystick_output_state, kMaxJoysticks> outputs;
};
```

## Decisions

- **D-C37-JOYSTICK-ONLY:** Cycle 37 includes joystick output/rumble commands
  only. User-program observer publication remains deferred until its own schema
  or a deliberate combined DS-output contract exists.
- **D-C37-PROTOCOL-V2:** Adding `schema_id::joystick_output_batch` changes the
  closed protocol schema set, so `kProtocolVersion` bumps from 1 to 2.
- **D-C37-TICK-PUSH:** `joystick_output_batch` is allowed by the validator
  under the `tick_boundary` kind only. The intended publisher is
  backend-to-core; a protocol-valid core-to-backend joystick output batch is
  still rejected by shim dispatch per D-C37-INBOUND-DEFENSIVE.
- **D-C37-ACTIVE-PREFIX:** `joystick_output_batch` is variable-size. The wire
  payload is `offsetof(joystick_output_batch, outputs) + count *
  sizeof(joystick_output_state)`. `count > kMaxJoysticks` is invalid.
- **D-C37-NAMED-PADDING:** The batch header pad and per-output pad are named
  `reserved_pad` fields. Senders zero-initialize them; the validator follows
  existing reserved-byte policy and ignores their contents on receive.
- **D-C37-SNAPSHOT:** `shim_core::current_joystick_output_batch()` returns a
  compact snapshot of all slots that have been written at least once.
- **D-C37-SORTED-SLOTS:** The snapshot active prefix is sorted by increasing
  joystick number, not call order. This makes repeated runs byte-identical and
  avoids encoding incidental write ordering into the host contract.
- **D-C37-EMPTY-SNAPSHOT:** A fresh shim snapshots to `count == 0`, and
  `flush_joystick_outputs(sim_time_us)` publishes that header-only empty
  snapshot. This is snapshot/state semantics, matching notifier state more than
  pending CAN/error queues.
- **D-C37-REPEATED-FLUSH:** Flushing does not clear joystick output storage.
  Repeated flushes publish the latest snapshot again with normal outbound
  sequence advancement.
- **D-C37-EXPLICIT-FLUSH:** `HAL_SetJoystickOutputs` remains storage-only.
  Publication happens only when host code calls `flush_joystick_outputs`.
- **D-C37-SHUTDOWN:** After the shim observes protocol shutdown,
  `send_joystick_output_batch` and `flush_joystick_outputs` return
  `shutdown_already_observed` without touching the outbound lane or advancing
  the send sequence.
- **D-C37-INBOUND-DEFENSIVE:** Even though the validator recognizes the new
  schema under `tick_boundary`, the HAL shim does not consume
  core-to-backend joystick output batches. If one arrives inbound, `poll()`
  fails with `unsupported_payload_schema` and does not mutate existing caches.

## Proposed tests

### C37-1 - joystick output schema has pinned POD layout and active-prefix bytes
- Layer / contract: Layer 2 protocol schema for joystick output publication.
- Bug class caught: implicit padding, wrong field order, or active-prefix math
  that would make host/shim disagree on wire bytes.
- Inputs: compile/runtime assertions on `joystick_output_state` and
  `joystick_output_batch`; zero-count and two-count batches.
- Expected: both structs are trivially copyable standard-layout aggregates;
  `joystick_output_state` is 24 bytes, alignof 8, offsets are
  `joystick_num=0`, `reserved_pad=4`, `outputs=8`, `left_rumble=16`,
  `right_rumble=20`; `joystick_output_batch` header offset is 8; active-prefix
  spans are 8 bytes for count 0 and 56 bytes for count 2.
- Tolerance / determinism: exact byte sizes; deterministic.

### C37-2 - validator accepts only joystick output batch active-prefix sizes
- Layer / contract: stateless protocol validator variable-size schema gate.
- Bug class caught: treating the new schema as fixed-size, accepting stale
  trailing slots after the active prefix, accepting truncated active prefixes,
  or allowing count beyond the six WPILib joystick slots.
- Inputs: `tick_boundary + joystick_output_batch` envelopes with counts `0`,
  `2`, `6`, and `7`; plus a `count == 2` payload whose `payload_bytes` is the
  full `sizeof(joystick_output_batch)` and a `count == 2` payload one byte
  smaller than the required active prefix.
- Expected: counts `0`, `2`, and `6` validate when `payload_bytes` matches the
  active prefix; count `7`, oversized `count == 2`, and undersized
  `count == 2` fail with `payload_size_mismatch`.
- Tolerance / determinism: exact sizes; deterministic.

### C37-3 - kind/schema mapping exposes joystick output batch only as tick push
- Layer / contract: protocol v2 closed kind/schema allowance table.
- Bug class caught: accidentally allowing joystick outputs as on-demand traffic
  or forgetting to recognize schema id 10 after bumping the enum.
- Inputs: validate `schema_id::joystick_output_batch` across every
  `envelope_kind`, and include it in the existing exhaustive cross-product.
- Expected: only `tick_boundary` accepts it; `kSchemaIdMaxValid == 10`;
  `kProtocolVersion == 2`.
- Tolerance / determinism: exact enum/table values; deterministic.

### C37-4 - send_joystick_output_batch publishes tick_boundary active-prefix wire bytes
- Layer / contract: shim outbound typed publisher.
- Bug class caught: publishing on the wrong schema id, using full struct size
  instead of active prefix, or stamping wrong sim time / sequence.
- Inputs: connected shim, drained boot lane, two-entry `joystick_output_batch`
  with nonzero raw values.
- Expected: core receives a backend-to-core `tick_boundary` envelope with
  `schema_id::joystick_output_batch`, the requested sim time, sequence 1, and
  payload bytes equal to `active_prefix_bytes(batch)`.
- Tolerance / determinism: exact bytes; deterministic.

### C37-5 - fresh shim flush publishes an empty joystick output snapshot
- Layer / contract: D-C37-EMPTY-SNAPSHOT host flush behavior.
- Bug class caught: treating an empty snapshot as a no-op pending queue and
  leaving the host unable to observe "no joystick outputs recorded".
- Inputs: connected shim, drained boot lane, no `HAL_SetJoystickOutputs` calls,
  call `flush_joystick_outputs(250000)`.
- Expected: flush succeeds; core receives a `joystick_output_batch` payload with
  `count == 0` and exactly the 8-byte header active prefix.
- Tolerance / determinism: exact bytes; deterministic.

### C37-6 - flush publishes latest compact snapshot sorted by joystick slot
- Layer / contract: D-C37-SNAPSHOT, D-C37-SORTED-SLOTS, D-C36-LATEST-WINS, and
  D-C36-RAW-VALUES.
- Bug class caught: preserving call order, publishing stale overwritten values,
  clearing other slots on update, swapping rumble fields, or narrowing the
  64-bit output bitmask.
- Inputs: install shim; set slot 5, set slot 1, set slot 5 again with different
  raw values; flush.
- Expected: payload count is 2; active entries are joystick 1 then joystick 5;
  joystick 5 contains the second slot-5 values; all raw signed values match
  the C ABI arguments.
- Tolerance / determinism: exact values; deterministic.

### C37-7 - HAL_SetJoystickOutputs does not publish until explicit flush
- Layer / contract: D-C37-EXPLICIT-FLUSH and cycle 36 storage behavior.
- Bug class caught: reintroducing direct publication from the C HAL call and
  making timing depend on robot-code call sites.
- Inputs: connected shim, drained boot lane, install shim, call
  `HAL_SetJoystickOutputs` for a valid slot, then inspect the core inbound
  lane before flushing.
- Expected: no outbound message is pending after the HAL call; a later flush
  publishes the stored state.
- Tolerance / determinism: exact lane state; deterministic.

### C37-8 - repeated flush republishes latest snapshot without clearing it
- Layer / contract: D-C37-REPEATED-FLUSH.
- Bug class caught: clearing output storage after the first flush or using an
  append-only queue instead of snapshot state.
- Inputs: set one joystick output; flush at `100000`; drain; flush again at
  `200000`.
- Expected: both flushes publish the same one-entry payload; the second
  envelope sequence advances by one and uses the second sim time.
- Tolerance / determinism: exact bytes and sequence; deterministic.

### C37-9 - post-shutdown joystick output publication is rejected without touching lane
- Layer / contract: D-C37-SHUTDOWN.
- Bug class caught: outbound sends after terminal shutdown.
- Inputs: connected shim observes a shutdown envelope; ensure outbound lane is
  empty; call both `send_joystick_output_batch` and `flush_joystick_outputs`.
- Expected: each returns `shutdown_already_observed`; no outbound message is
  pending.
- Tolerance / determinism: exact error kind and lane state; deterministic.

### C37-10 - protocol-valid inbound joystick output batch is rejected by the shim
- Layer / contract: D-C37-INBOUND-DEFENSIVE shim dispatch boundary.
- Bug class caught: accidentally treating host output commands as
  sim-authoritative inbound state or silently discarding a newly valid schema.
- Inputs: connected shim with a known cached `clock_state`; core sends a valid
  `tick_boundary + joystick_output_batch` inbound payload.
- Expected: `poll()` fails with `unsupported_payload_schema`; existing cached
  clock state remains unchanged.
- Tolerance / determinism: exact error kind and cached value; deterministic.

### C37-11 - observer calls are not included in joystick output publication
- Layer / contract: D-C37-JOYSTICK-ONLY / observer publication deferred.
- Bug class caught: smuggling user-program observer state through the joystick
  output schema or making joystick output payload depend on unrelated DS hooks.
- Inputs: call observer hooks and no joystick output writes, flush; then write
  one joystick output, call a different observer hook, flush again.
- Expected: first payload is empty; second payload contains only the joystick
  output entry. Observer state remains accessible through its cycle 35
  accessor but is not encoded in the batch.
- Tolerance / determinism: exact payload bytes; deterministic.

### C37-12 - repeated runs publish byte-identical joystick output batches
- Layer / contract: deterministic outbound replay for the new protocol schema.
- Bug class caught: uninitialized reserved bytes or call-order-dependent
  snapshot packing.
- Inputs: run the same scenario twice: set slots 3 and 0, overwrite slot 3,
  flush and capture the outbound payload bytes.
- Expected: the two payload byte vectors are identical and contain sorted slots
  0 then 3 with zeroed reserved pads.
- Tolerance / determinism: byte-for-byte equality; deterministic.

## Deferred

- Protocol publication of user-program observer state.
- A combined Driver Station output/observer host-state schema.
- Automatic 50 ms publication cadence.
- Rumble clamping or output-bit masking beyond preserving the C HAL values.
