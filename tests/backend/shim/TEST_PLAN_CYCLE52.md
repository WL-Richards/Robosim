# Cycle 52 test plan - DriverStation joystick refresh JNI adapters

Status: implemented.

Reviewer: approved as ready after adding explicit no-shim coverage and exact
export-symbol names.

## Context

Cycle 51 made the repo-owned non-HALSIM `libwpiHaljni.so` create and install a
real fallback shim for Java startup. Manual Java verification now advances past
`HAL.initialize` and stops at the first missing Driver Station refresh JNI
symbol:

```text
java.lang.UnsatisfiedLinkError:
  'int edu.wpi.first.hal.DriverStationJNI.getJoystickAxes(byte, float[])'
```

WPILib 2026.2.1 `DriverStation.refreshData()` immediately calls the four
per-port joystick refresh JNI methods for each of the six joystick slots:

- `DriverStationJNI.getJoystickAxes(byte, float[])`
- `DriverStationJNI.getJoystickAxesRaw(byte, int[])`
- `DriverStationJNI.getJoystickPOVs(byte, short[])`
- `DriverStationJNI.getJoystickButtons(byte, ByteBuffer)`

The C HAL readers behind this surface already exist from cycle 24 and read from
`latest_ds_state_`.

## Scope

In:

- Add the four joystick refresh JNI exports above.
- Adapt them to the existing C HAL readers:
  - `HAL_GetJoystickAxes`
  - `HAL_GetJoystickPOVs`
  - `HAL_GetJoystickButtons`
- Copy only the active count reported by the C HAL reader into the Java array.
- Convert raw axis bytes to Java `int[]` values in the range 0..255.
- Write joystick button count into the first byte of the provided direct
  `ByteBuffer` and return the button bitmask.
- Keep no-shim/default/invalid-index behavior inherited from the C HAL readers.
- Add a minimal local JNI test harness for array-region calls and direct-buffer
  address calls; no JVM is started in CTest.
- Update the JNI artifact export check to require the four new symbols.

Out:

- No `getAllJoystickData` JNI adapter.
- No descriptor/match/control-word JNI adapters.
- No string conversion, object field mutation, or Java exception handling.
- No DS state synthesis in the JNI layer.
- No claim that the Java robot process boots completely.

## v0 decisions

- **D-C52-JNI-DS-JOYSTICK-REFRESH-GROUP:** Cycle 52 lands the four per-port
  joystick refresh JNI adapters together because WPILib calls them as one
  contiguous loop in `DriverStation.refreshData()`.
- **D-C52-C-HAL-SEMANTICS-INHERIT:** JNI adapters do not reinterpret shim,
  cache, or invalid-index behavior; they call the existing C HAL readers and
  return/copy their zero/default results.
- **D-C52-ACTIVE-COUNT-COPY:** JNI adapters copy only `count` elements into the
  Java arrays. Inactive suffix contents belong to WPILib's reusable Java cache
  arrays and are not cleared by this JNI layer.
- **D-C52-RAW-AXIS-WIDEN:** `getJoystickAxesRaw(byte, int[])` widens each
  active `uint8_t` raw axis byte to a nonnegative Java `int`.
- **D-C52-BUTTON-DIRECT-BUFFER:** `getJoystickButtons(byte, ByteBuffer)` uses
  `GetDirectBufferAddress`; if the pointer is non-null, byte 0 receives the
  button count. The method returns the button bitmask either way.
- **D-C52-NULL-JNI-OUTPUTS:** A null JNI array/buffer pointer is treated as a
  no-copy destination while still returning the C HAL count/bitmask. This keeps
  the v0 adapters deterministic in direct unit tests without adding Java
  exception behavior.

## Tests

### C52-1 - artifact exports joystick refresh JNI symbols

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  the per-port joystick refresh methods.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include the four Cycle 52 JNI
  names:
  - `Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxes`
  - `Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxesRaw`
  - `Java_edu_wpi_first_hal_DriverStationJNI_getJoystickPOVs`
  - `Java_edu_wpi_first_hal_DriverStationJNI_getJoystickButtons`
- **Determinism notes:** Static artifact inspection only.

### C52-2 - `getJoystickAxes` copies active float axes and returns count

- **Layer / contract:** JNI adapter over C HAL joystick axes reader.
- **Bug class caught:** Adapter returns the wrong count, copies raw bytes instead
  of float axes, copies the wrong joystick slot, or clears/copies past active
  count.
- **Inputs:** Installed shim with cached `ds_state`; joystick slot 2 has count
  3 and distinct axis values in elements 0..3; fake JNI float array initialized
  to sentinels.
- **Expected outputs:** Return value is 3; Java float array elements 0..2 equal
  cached floats; element 3 remains the sentinel.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C52-3 - `getJoystickAxesRaw` widens active raw axis bytes

- **Layer / contract:** JNI adapter raw-axis conversion.
- **Bug class caught:** Signed-byte extension, copying float axes into the raw
  array, or copying inactive suffix elements.
- **Inputs:** Installed shim with cached `ds_state`; joystick slot 1 has count
  4 and raw bytes `{0, 1, 128, 255}`; fake JNI int array initialized to
  sentinels.
- **Expected outputs:** Return value is 4; Java int array elements 0..3 are
  `{0, 1, 128, 255}`; element 4 remains the sentinel.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C52-4 - `getJoystickPOVs` copies active POV values and returns count

- **Layer / contract:** JNI adapter over C HAL joystick POV reader.
- **Bug class caught:** Wrong element width, wrong slot, wrong count, or suffix
  clearing.
- **Inputs:** Installed shim with cached `ds_state`; joystick slot 5 has count
  2 and POV values `{90, -1}`; fake JNI short array initialized to sentinels.
- **Expected outputs:** Return value is 2; Java short array elements 0..1 equal
  cached values; element 2 remains the sentinel.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C52-5 - `getJoystickButtons` writes direct-buffer count and returns bitmask

- **Layer / contract:** JNI adapter over C HAL joystick buttons reader.
- **Bug class caught:** Count not written to Java's direct `ByteBuffer`, bitmask
  not returned, or count/bitmask swapped.
- **Inputs:** Installed shim with cached `ds_state`; joystick slot 4 has count
  9 and button mask `0xA5A50005`; fake direct buffer byte initialized to a
  sentinel.
- **Expected outputs:** Return value is `0xA5A50005` as Java `int`; direct
  buffer byte 0 is 9.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C52-6 - no-shim, default, and invalid-index paths inherit C HAL zero/default behavior

- **Layer / contract:** JNI adapters preserve existing C HAL no-cache and
  invalid-index behavior.
- **Bug class caught:** JNI layer treats C HAL defaults as errors, writes
  garbage, or handles signed Java `byte` indices inconsistently.
- **Inputs:** First clear global/JNI fallback state and call all four adapters
  for slot 0. Then install a shim with empty DS cache and call all four
  adapters for slot 0. Then install cached DS state and call all four adapters
  with Java byte `-1` and `6`.
- **Expected outputs:** Counts/bitmasks are 0; active arrays receive no writes;
  button direct-buffer byte is written as 0 when a direct buffer is provided.
- **Determinism notes:** In-process fake JNI env; Java `byte` signedness is
  represented by `jbyte`.

### C52-7 - null JNI destinations return values without copying

- **Layer / contract:** v0 direct adapter robustness.
- **Bug class caught:** Unit tests or future partial Java paths crash when the
  destination is null before Java exception behavior exists.
- **Inputs:** Installed shim with cached joystick state; call all four adapters
  with null Java array/buffer handles.
- **Expected outputs:** Counts/bitmask match the C HAL reader; no crash.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C52-8 - manual Java launch advances past joystick refresh JNI blockers

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** Direct Java command with `build/src/backend/shim` first in native
  paths.
- **Expected outputs:** Output no longer contains missing JNI errors for
  `getJoystickAxes`, `getJoystickAxesRaw`, `getJoystickPOVs`, or
  `getJoystickButtons`; the next missing JNI/runtime behavior drives Cycle 53.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts.
