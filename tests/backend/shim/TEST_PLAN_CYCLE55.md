# Cycle 55 test plan - DriverStation sendError JNI adapter

Status: implemented.

Reviewer: approved as ready.

## Context

Cycle 54 added `DriverStationJNI.nativeGetAllianceStation()` and manual Java
verification now reaches robot program startup:

```text
********** Robot program starting **********
```

The next launch blocker appears when WPILib tries to report NetworkTables
socket errors through Driver Station error reporting:

```text
java.lang.UnsatisfiedLinkError:
  'int edu.wpi.first.hal.DriverStationJNI.sendError(boolean, int, boolean,
  java.lang.String, java.lang.String, java.lang.String, boolean)'
```

The C HAL write surface already exists:

- `HAL_SendError(HAL_Bool isError, int32_t errorCode, HAL_Bool isLVCode,
  const char* details, const char* location, const char* callStack,
  HAL_Bool printMsg)`

It buffers `error_message` entries on the installed shim and preserves the
existing explicit `flush_pending_errors(sim_time_us)` boundary.

## Scope

In:

- Add JNI export `Java_edu_wpi_first_hal_DriverStationJNI_sendError`.
- Extend `jni_minimal.h` only enough to cover `GetStringUTFChars` and
  `ReleaseStringUTFChars` at their real JNI table positions.
- Convert the three Java `String` arguments to UTF-8 C strings and call
  `HAL_SendError`.
- Preserve existing C HAL no-shim behavior by returning `kHalHandleError`.
- Update the JNI artifact export check to require the new symbol.

Out:

- No automatic flush of pending errors from JNI; Java `sendError` is the same
  synchronous enqueue surface as C `HAL_SendError`.
- No change to `HAL_SendError` truncation, null-string, overflow, or outbound
  protocol publication semantics.
- No `DriverStationJNI.getMatchTime`, console APIs, outputs-active APIs, or
  descriptor/all-joystick aggregate JNI adapters.
- No attempt to suppress or solve the sandbox NetworkTables socket errors.

## v0 decisions

- **D-C55-ADAPTER-ONLY:** `DriverStationJNI.sendError(...)` is a JNI adapter
  over `HAL_SendError`; it does not inspect `shim_core` directly and does not
  publish protocol messages itself.
- **D-C55-EXPLICIT-FLUSH-BOUNDARY:** JNI error reporting only enqueues. The
  existing `flush_pending_errors(sim_time_us)` boundary remains the sole path
  to publish `error_message_batch`.
- **D-C55-JAVA-STRING-UTF:** Java strings are converted with
  `GetStringUTFChars` and released with `ReleaseStringUTFChars` after the
  `HAL_SendError` call. The existing C HAL copy/truncation rules mean the JNI
  adapter does not retain JVM-owned pointers.
- **D-C55-NULL-STRING-IS-EMPTY:** Null Java strings are passed as null C
  pointers, inheriting the existing C HAL rule that null strings become empty
  fields with no truncation flags.
- **D-C55-MISSING-JNI-FUNCTION-IS-NULL-STRING:** If a test/minimal JNI
  environment lacks `GetStringUTFChars`, the adapter still calls
  `HAL_SendError` with null string pointers. This keeps null/minimal test envs
  deterministic and avoids inventing a Java exception path in v0.
- **D-C55-BOOL-PARITY:** Java booleans map to `HAL_Bool` values `0` or `1`
  by testing `jboolean != 0`.

## Tests

### C55-1 - artifact exports sendError JNI symbol

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  `DriverStationJNI.sendError(...)`.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include
  `Java_edu_wpi_first_hal_DriverStationJNI_sendError`.
- **Determinism notes:** Static artifact inspection only.

### C55-2 - `sendError` converts Java strings and enqueues one error message

- **Layer / contract:** JNI adapter over C `HAL_SendError`.
- **Bug class caught:** Missing string conversion, wrong parameter order, wrong
  boolean mapping, or bypassing the C HAL buffering path.
- **Inputs:** Installed connected shim. Fake JNI env with three `jstring`
  objects containing `"detail text"`, `"location text"`, and `"stack text"`.
  Call `sendError(true, 1234, false, details, location, callStack, true)`.
- **Expected outputs:** Return `0`; shim pending error buffer has one
  `error_message` matching `error_code=1234`, severity/error `1`,
  `is_lv_code=0`, `print_msg=1`, and the three converted strings.
- **Determinism notes:** In-process fake JNI table, no real JVM.

### C55-3 - `sendError` releases each acquired Java string

- **Layer / contract:** JNI resource ownership.
- **Bug class caught:** Leaking UTF string handles or releasing the wrong
  `jstring`/pointer pairs.
- **Inputs:** Fake JNI env that records `GetStringUTFChars` and
  `ReleaseStringUTFChars` calls for three non-null strings.
- **Expected outputs:** Three acquired pointers are released exactly once, with
  matching original `jstring` handles, after the HAL call has copied them.
- **Determinism notes:** In-process fake JNI table.

### C55-4 - `sendError` null or minimal string paths inherit C HAL empty fields

- **Layer / contract:** JNI adapter null/default behavior over C HAL
  null-string semantics.
- **Bug class caught:** Dereferencing null strings/envs, skipping the HAL call,
  or inventing non-empty placeholder text.
- **Inputs:** Installed shim. Call `sendError(false, -7, true, nullptr,
  nullptr, nullptr, false)` with a null env; repeat with a fake env missing
  `GetStringUTFChars`.
- **Expected outputs:** Each call returns `0` and appends an `error_message`
  whose details/location/call_stack fields are empty, `error_code=-7`,
  `severity=0`, `is_lv_code=1`, and `print_msg=0`.
- **Determinism notes:** In-process.

### C55-5 - `sendError` no-shim path returns handle error and does not create fallback state

- **Layer / contract:** no-shim behavior remains inherited from C
  `HAL_SendError`; JNI fallback creation is limited to `HAL.initialize`.
- **Bug class caught:** JNI adapter silently creates a fallback shim, reports
  success without buffering, or returns a Java-specific placeholder value.
- **Inputs:** Reset JNI launch state and clear `shim_core::current()`. Call
  `sendError(true, 9, false, nullptr, nullptr, nullptr, true)`.
- **Expected outputs:** Return `kHalHandleError`; no JNI fallback state exists;
  no shim is installed.
- **Determinism notes:** In-process.

### C55-6 - manual Java launch advances past sendError blocker

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** Direct Java command with `build/src/backend/shim` first in native
  paths.
- **Expected outputs:** Output no longer contains missing JNI errors for
  `DriverStationJNI.sendError`. In the current sandbox run, the empty
  TimedRobot reaches startup, reports NetworkTables socket permission errors,
  and exits without a missing JNI symbol.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts and sandbox network permissions.
