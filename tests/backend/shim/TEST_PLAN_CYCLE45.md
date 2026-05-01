# HAL shim core - cycle 45 test plan (HAL_Report)

Status: draft

Cycle 45 adds the generated FRC usage-reporting surface:

```cpp
std::int64_t HAL_Report(std::int32_t resource,
                        std::int32_t instanceNumber,
                        std::int32_t context,
                        const char* feature);
```

WPILib forwards this to NetworkCommunication usage reporting, treating a null
feature pointer as an empty string. The v0 shim records reports on the installed
`shim_core` for host-side inspection and does not add a protocol schema.

## Decisions

- **D-C45-C-ABI-ONLY:** Cycle 45 adds only `HAL_Report`. No protocol schema,
  protocol version, boot envelope layout, or host publication change is part of
  this slice.
- **D-C45-WPILIB-SIGNATURE:** The C ABI is
  `std::int64_t HAL_Report(std::int32_t resource, std::int32_t instanceNumber,
  std::int32_t context, const char* feature)`.
- **D-C45-NO-SHIM-NOOP:** With no installed shim, `HAL_Report` returns `0` and
  records nothing.
- **D-C45-PER-SHIM-LOG:** Installed shims append copied `usage_report_record`
  entries in call order and expose them through `shim_core::usage_reports()`.
- **D-C45-RETURN-INDEX:** Installed shims return a deterministic 1-based report
  index equal to the size of the per-shim report log after append.
- **D-C45-RAW-FIELDS:** The shim preserves raw signed `resource`,
  `instanceNumber`, and `context` values without enum validation, clamping, or
  narrowing.
- **D-C45-FEATURE-COPY:** Null feature pointers are stored as empty strings.
  Non-null feature strings are copied so later caller mutation cannot alter the
  retained report.
- **D-C45-NO-PUBLISH-NO-POLL:** `HAL_Report` is storage-only in v0. It must not
  publish outbound messages, poll inbound messages, or mutate cached sim state.
- **D-C45-SHUTDOWN-DETACH:** After `HAL_Shutdown`, calls follow the no-shim path
  and do not mutate the caller-owned shim's retained report log.

## Proposed tests

### C45-1 - HAL_Report signature
- Layer / contract: C HAL ABI and D-C45-WPILIB-SIGNATURE.
- Bug class caught: wrong return width, missing feature parameter, wrong
  argument order, or unsigned argument types.
- Inputs: compile-time function pointer check.
- Expected: signature is
  `std::int64_t (*)(std::int32_t, std::int32_t, std::int32_t, const char*)`.
- Tolerance / determinism: compile-time exact type.

### C45-2 - no installed shim returns zero
- Layer / contract: D-C45-NO-SHIM-NOOP.
- Bug class caught: dereferencing a null global shim, fabricating report
  indices, or writing process-global report state.
- Inputs: clear global shim; call `HAL_Report`.
- Expected: return value is `0`.
- Tolerance / determinism: exact integer value.

### C45-3 - installed shim records reports in call order
- Layer / contract: D-C45-PER-SHIM-LOG and D-C45-RETURN-INDEX.
- Bug class caught: not storing reports, replacing instead of appending,
  returning zero on installed shims, or changing field order.
- Inputs: connected installed shim; call `HAL_Report` twice with distinct
  values.
- Expected: return values are `1` and `2`; `usage_reports()` contains two exact
  records in call order.
- Tolerance / determinism: exact fields and return values.

### C45-4 - null feature is stored as empty string
- Layer / contract: D-C45-FEATURE-COPY.
- Bug class caught: null pointer dereference or using placeholder text for
  absent feature values.
- Inputs: installed shim; call `HAL_Report(..., nullptr)`.
- Expected: one report with `feature == ""`.
- Tolerance / determinism: exact string.

### C45-5 - raw fields and copied feature storage
- Layer / contract: D-C45-RAW-FIELDS and D-C45-FEATURE-COPY.
- Bug class caught: enum validation, clamping, narrowing, or retaining a
  borrowed feature pointer.
- Inputs: installed shim; call `HAL_Report` with `INT32_MIN`, `INT32_MAX`,
  negative context, and a mutable feature string; mutate the caller string
  afterward.
- Expected: retained report has the original integer values and original
  feature bytes.
- Tolerance / determinism: exact fields.

### C45-6 - installed report does not publish or poll
- Layer / contract: D-C45-NO-PUBLISH-NO-POLL.
- Bug class caught: shortcut behavior where usage reporting sends protocol
  messages or consumes pending inbound traffic.
- Inputs: create a shim, drain boot, queue a valid `clock_state` in the inbound
  core-to-backend lane, install the shim, call `HAL_Report`.
- Expected: report is recorded; backend-to-core lane remains empty;
  core-to-backend lane remains full; latest clock/power/DS caches remain unset.
- Tolerance / determinism: exact report count, lane states, and cache emptiness.

### C45-7 - shutdown detach prevents later mutation
- Layer / contract: D-C45-SHUTDOWN-DETACH.
- Bug class caught: stale global-shim mutation after shutdown.
- Inputs: connected shim; install it; call `HAL_Report`; call `HAL_Shutdown`;
  call `HAL_Report` again.
- Expected: first call returns `1`; post-shutdown call returns `0`; old shim
  still contains only the first report.
- Tolerance / determinism: exact count and string.

## Deferred

- Protocol publication for usage reports.
- Mirroring the full generated `HALUsageReporting::tResourceType` enum in the
  C shim header.
- NetworkCommunication-backed report indices or de-duplication semantics.
