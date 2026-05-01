# Cycle 53 test plan - DriverStation match-info and control-word JNI adapters

Status: implemented.

Reviewer: approved as ready after revising the string contract to match
`NewStringUTF` NUL-terminated v0 behavior and adding missing-function-pointer
null-safety coverage.

## Context

Cycle 52 added the four per-port joystick refresh JNI adapters. Manual Java
verification now gets through that loop in WPILib 2026.2.1
`DriverStation.refreshData()` and stops at:

```text
java.lang.UnsatisfiedLinkError:
  'int edu.wpi.first.hal.DriverStationJNI.getMatchInfo(edu.wpi.first.hal.MatchInfoData)'
```

The next Java line is `DriverStationJNI.getControlWord(m_controlWordCache)`,
which is a Java wrapper around native `DriverStationJNI.nativeGetControlWord()`.
The C HAL readers behind both surfaces already exist:

- `HAL_GetMatchInfo(HAL_MatchInfo*)`
- `HAL_GetControlWord(HAL_ControlWord*)`

## Scope

In:

- Add JNI export `Java_edu_wpi_first_hal_DriverStationJNI_getMatchInfo`.
- Add JNI export `Java_edu_wpi_first_hal_DriverStationJNI_nativeGetControlWord`.
- Adapt `getMatchInfo` to the existing C HAL reader and call Java
  `MatchInfoData.setData(String eventName, String gameSpecificMessage,
  int matchNumber, int replayNumber, int matchType)`.
- Adapt `nativeGetControlWord` to the existing C HAL reader and return the
  packed six-bit word used by WPILib Java's `DriverStationJNI.getControlWord`
  helper.
- Extend the minimal local JNI table to cover only the functions needed for
  `getMatchInfo`: `GetObjectClass`, `GetMethodID`, `NewStringUTF`, and
  `CallVoidMethod`.
- Update the JNI artifact export check to require both new symbols.

Out:

- No `nativeGetAllianceStation` JNI adapter.
- No `getMatchTime`, descriptor, all-joystick aggregate, or output-side JNI
  adapters.
- No Java field writes; `MatchInfoData` is populated through its public
  `setData` method just like the Java class is designed for JNI.
- No full Java process success claim.
- No Java exception handling beyond v0 no-op/null safety.

## v0 decisions

- **D-C53-REFRESH-PAIR:** Cycle 53 lands `getMatchInfo` and
  `nativeGetControlWord` together because they are adjacent non-joystick calls
  in `DriverStation.refreshData()` after the cycle 52 joystick loop.
- **D-C53-C-HAL-SEMANTICS-INHERIT:** JNI adapters inherit no-shim, empty-cache,
  and cached-state semantics from the existing C HAL readers. The JNI layer
  does not synthesize match data or control bits.
- **D-C53-MATCHINFO-SETDATA:** `getMatchInfo` populates Java by looking up and
  calling `MatchInfoData.setData(String, String, int, int, int)`.
- **D-C53-NEWSTRINGUTF-NUL-TERMINATED:** Event name uses bytes up to the first
  NUL in the fixed 64-byte event-name field. Game-specific message first clamps
  `gameSpecificMessageSize` to the fixed 64-byte field, then v0 passes a
  NUL-terminated scratch buffer to `NewStringUTF`, so bytes after the first NUL
  are not represented in the Java string. A future length-capable string path
  can revisit embedded-NUL preservation explicitly.
- **D-C53-CONTROLWORD-PACKING:** `nativeGetControlWord()` returns the raw
  six-bit `HAL_ControlWord` layout: enabled bit 0, autonomous bit 1, test bit 2,
  eStop bit 3, fmsAttached bit 4, dsAttached bit 5.
- **D-C53-NULL-JNI-MATCHINFO:** If `env`, the Java object, or any required JNI
  function pointer is null, `getMatchInfo` still returns the C HAL status but
  performs no Java mutation. This keeps direct unit tests deterministic until
  Java exception behavior is intentionally modeled.

## Tests

### C53-1 - artifact exports match-info and control-word JNI symbols

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  the next two Driver Station refresh JNI methods.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include:
  - `Java_edu_wpi_first_hal_DriverStationJNI_getMatchInfo`
  - `Java_edu_wpi_first_hal_DriverStationJNI_nativeGetControlWord`
- **Determinism notes:** Static artifact inspection only.

### C53-2 - `getMatchInfo` calls `MatchInfoData.setData` with cached match fields

- **Layer / contract:** JNI adapter over C HAL match-info reader.
- **Bug class caught:** Adapter skips Java mutation, calls the wrong method
  signature, swaps match-number/replay/type fields, or reads the wrong DS
  cache fields.
- **Inputs:** Installed shim with cached `ds_state.match`:
  event name `"PNW District"`, game-specific message `"ABC"`, match number 73,
  replay 2, match type elimination. Fake JNI environment captures
  `GetMethodID`, `NewStringUTF`, and `CallVoidMethod` arguments.
- **Expected outputs:** Return `kHalSuccess`; fake JNI sees method name
  `"setData"` and descriptor `"(Ljava/lang/String;Ljava/lang/String;III)V"`;
  captured Java arguments match `"PNW District"`, `"ABC"`, 73, 2, and the
  elimination enum value.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C53-3 - `getMatchInfo` uses NUL-terminated v0 Java strings

- **Layer / contract:** JNI string extraction over fixed-size HAL match fields.
- **Bug class caught:** Event name reads past first NUL, game-specific message
  ignores its explicit size before termination, or the adapter claims
  embedded-NUL preservation that `NewStringUTF` cannot provide.
- **Inputs:** Installed shim with event name bytes `"Event\0ignored"` in the
  fixed event buffer. Use a game-specific-message size of 8 with bytes
  `"ABC\0DEFG"` in the fixed game-specific-message buffer, and another case
  where the fixed game field starts with 64 non-NUL bytes while the explicit
  size is 3.
- **Expected outputs:** Captured event string is `"Event"`; captured
  game-specific message is `"ABC"` in the embedded-NUL case and exactly the
  first 3 bytes in the size-clamp case.
- **Determinism notes:** Fake `NewStringUTF` captures the NUL-terminated bytes
  passed to the JNI function; no JVM.

### C53-4 - `getMatchInfo` no-shim and empty-cache paths call `setData` with defaults

- **Layer / contract:** JNI adapter inherits C HAL default behavior.
- **Bug class caught:** Adapter treats C HAL handle error as a reason to skip
  Java default population, or leaves stale Java match data in empty/default
  paths.
- **Inputs:** First clear global/JNI fallback state and call `getMatchInfo`.
  Then install a shim with empty DS cache and call `getMatchInfo`.
- **Expected outputs:** No-shim returns `kHalHandleError`; empty cache returns
  `kHalSuccess`; both call `setData("", "", 0, 0, HAL_kMatchType_none)`.
- **Determinism notes:** In-process fake JNI env; no JVM.

### C53-5 - `getMatchInfo` null JNI destinations return status without mutation

- **Layer / contract:** v0 direct adapter robustness.
- **Bug class caught:** Direct tests or partial launch paths crash before Java
  exception behavior exists.
- **Inputs:** Installed shim with cached match info; call `getMatchInfo` with
  null env, null object, and a fake env with one required function pointer
  missing.
- **Expected outputs:** Return matches `HAL_GetMatchInfo` status; fake JNI
  captures no `setData` call.
- **Determinism notes:** In-process.

### C53-6 - `nativeGetControlWord` returns packed bits from cached DS state

- **Layer / contract:** JNI scalar adapter over C HAL control-word reader.
- **Bug class caught:** Bit order drift between C HAL and Java helper, or stale
  status/error handling changing the returned word.
- **Inputs:** Installed shim with cached control bits
  `enabled | autonomous | dsAttached`.
- **Expected outputs:** Return value is `0b100011`.
- **Determinism notes:** In-process.

### C53-7 - `nativeGetControlWord` default paths return zero

- **Layer / contract:** JNI adapter inherits C HAL no-shim/empty-cache default
  behavior.
- **Bug class caught:** JNI returns garbage on no-shim or empty-cache paths.
- **Inputs:** Clear global/JNI fallback state and call `nativeGetControlWord`;
  then install empty-cache shim and call again.
- **Expected outputs:** Both return 0.
- **Determinism notes:** In-process.

### C53-8 - manual Java launch advances past match-info/control-word blockers

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** Direct Java command with `build/src/backend/shim` first in native
  paths.
- **Expected outputs:** Output no longer contains missing JNI errors for
  `getMatchInfo` or `nativeGetControlWord`; the next missing JNI/runtime
  behavior drives Cycle 54.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts.
