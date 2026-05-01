# HAL shim core - cycle 43 test plan (HAL port handles)

Status: draft

Cycle 43 adds the HALBase port-handle constructors:

```cpp
HAL_PortHandle HAL_GetPort(std::int32_t channel);
HAL_PortHandle HAL_GetPortWithModule(std::int32_t module, std::int32_t channel);
```

These functions are pure C HAL value constructors. They do not allocate
resources, publish protocol messages, or require an installed shim.

## Decisions

- **D-C43-C-ABI-ONLY:** Cycle 43 adds only `HAL_GetPort` and
  `HAL_GetPortWithModule`. No protocol schema, protocol version, boot envelope
  layout, or host publication change is part of this slice.
- **D-C43-WPILIB-TYPES:** `HAL_Handle` mirrors WPILib as `std::int32_t`, and
  `HAL_PortHandle` is an alias of `HAL_Handle`.
- **D-C43-WPILIB-LAYOUT:** Port handles use WPILib's specialized layout:
  bits `0..7` channel, bits `8..15` module, bits `16..23` unused, bits
  `24..30` type, and bit `31` error. The port handle type value is `2`.
- **D-C43-DEFAULT-MODULE:** `HAL_GetPort(channel)` is equivalent to
  `HAL_GetPortWithModule(1, channel)`.
- **D-C43-UINT8-RANGE:** Inputs must fit the uint8_t channel/module fields.
  Values `< 0` or `>= 255` return `HAL_kInvalidHandle` (`0`) before narrowing.
- **D-C43-NO-SHIM-REQUIRED:** Port-handle construction does not require an
  installed shim and remains available after `HAL_Shutdown`.
- **D-C43-NO-PUBLISH-NO-POLL:** Port-handle construction is read-only process
  value construction. It must not publish outbound messages, poll inbound
  messages, or mutate cached sim state.

## Proposed tests

### C43-1 - HAL_GetPort signatures and typedefs
- Layer / contract: C HAL ABI and D-C43-WPILIB-TYPES.
- Bug class caught: wrong return type, unsigned handle type, missing
  `HAL_PortHandle`, or wrong argument order.
- Inputs: compile-time function pointer and typedef checks.
- Expected: `HAL_Handle` is `std::int32_t`; `HAL_PortHandle` aliases
  `HAL_Handle`; signatures match WPILib.
- Tolerance / determinism: compile-time exact type.

### C43-2 - HAL_GetPort uses module 1
- Layer / contract: D-C43-DEFAULT-MODULE and D-C43-WPILIB-LAYOUT.
- Bug class caught: encoding module 0, off-by-one channel encoding, or generic
  handle layout instead of port-specialized layout.
- Inputs: call `HAL_GetPort` with channels `0`, `1`, `42`, and `254`.
- Expected: each result equals `(2 << 24) | (1 << 8) | channel`.
- Tolerance / determinism: exact integer values.

### C43-3 - HAL_GetPortWithModule encodes both fields
- Layer / contract: D-C43-WPILIB-LAYOUT.
- Bug class caught: swapping module/channel, dropping module, or sign-extending
  byte fields.
- Inputs: call `HAL_GetPortWithModule` with `(0,0)`, `(1,0)`, `(7,3)`, and
  `(254,254)`.
- Expected: each result equals `(2 << 24) | (module << 8) | channel`.
- Tolerance / determinism: exact integer values.

### C43-4 - invalid inputs return invalid handle
- Layer / contract: D-C43-UINT8-RANGE.
- Bug class caught: narrowing negative or 255+ values into uint8_t fields,
  accepting out-of-range modules, or returning nonzero sentinel handles.
- Inputs: negative, `255`, `INT32_MIN`, and `INT32_MAX` boundary cases for
  channel and module.
- Expected: every invalid call returns `0`.
- Tolerance / determinism: exact integer value.

### C43-5 - installed port queries do not publish or poll
- Layer / contract: D-C43-NO-PUBLISH-NO-POLL.
- Bug class caught: accidentally tying value construction to shim state,
  sending protocol messages, or consuming pending inbound traffic.
- Inputs: create a shim, drain boot, queue a valid `clock_state` in the inbound
  core-to-backend lane, install the shim, call both port functions.
- Expected: returned handles match the WPILib layout; backend-to-core lane
  remains empty; core-to-backend lane remains full; latest clock/power/DS caches
  remain unset.
- Tolerance / determinism: exact handles, lane states, and cache emptiness.

### C43-6 - port handles remain available after shutdown detach
- Layer / contract: D-C43-NO-SHIM-REQUIRED.
- Bug class caught: stale global-shim dependency or post-shutdown failure.
- Inputs: connected shim; install it; call `HAL_GetPort`; call `HAL_Shutdown`;
  call both port functions again.
- Expected: global shim is null after shutdown and both calls still return the
  same deterministic encoded handles.
- Tolerance / determinism: exact integer values.

## Deferred

- Port-handle consumers such as DIO, PWM, Relay, Analog, SPI, I2C, or PCM/PH
  initialization surfaces.
- Helper APIs for decoding port handles outside tests.
- Generic HAL handle allocation/versioning for resource-owning handles.
