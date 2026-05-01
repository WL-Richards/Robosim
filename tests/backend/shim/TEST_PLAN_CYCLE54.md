# Cycle 54 test plan - DriverStation alliance-station JNI adapter

Status: implemented.

Reviewer: approved as ready.

## Context

Cycle 53 added `DriverStationJNI.getMatchInfo(MatchInfoData)` and
`DriverStationJNI.nativeGetControlWord()`. Manual Java verification now advances
through those calls and stops during WPILib 2026.2.1 match-data logging inside
`DriverStation.refreshData()`:

```text
java.lang.UnsatisfiedLinkError:
  'int edu.wpi.first.hal.DriverStationJNI.nativeGetAllianceStation()'
```

The C HAL reader already exists:

- `HAL_GetAllianceStation(int32_t* status)`

The Java wrapper `DriverStationJNI.getAllianceStation()` maps native int values
0..6 to `AllianceStationID.Unknown`, `Red1`, `Red2`, `Red3`, `Blue1`, `Blue2`,
and `Blue3`.

## Scope

In:

- Add JNI export
  `Java_edu_wpi_first_hal_DriverStationJNI_nativeGetAllianceStation`.
- Adapt it directly to `HAL_GetAllianceStation`.
- Return the WPILib integer enum value from the C HAL reader.
- Preserve no-shim and empty-cache defaults as unknown station `0`.
- Update the JNI artifact export check to require the new symbol.

Out:

- No public Java object/enum construction; WPILib Java already wraps the native
  int.
- No `getMatchTime`, `getOutputsActive`, `sendError`, console, descriptor, or
  all-joystick aggregate JNI adapters.
- No DS state synthesis in JNI.
- No full Java process success claim.

## v0 decisions

- **D-C54-SCALAR-ADAPTER-ONLY:** `nativeGetAllianceStation()` is a pure scalar
  JNI adapter over the existing C HAL reader; it does not inspect `shim_core`
  directly.
- **D-C54-STATUS-DROPPED:** Java has no status out-param for this method.
  `kHalHandleError` from the no-shim C HAL path is intentionally collapsed to
  the returned unknown station value, matching WPILib's Java-facing shape.
- **D-C54-ENUM-PARITY:** Returned integers are the existing C HAL enum values:
  unknown `0`, red stations `1..3`, blue stations `4..6`.

## Tests

### C54-1 - artifact exports alliance-station JNI symbol

- **Layer / contract:** repo-owned non-HALSIM HAL JNI launch artifact.
- **Bug class caught:** Java launch still fails with `UnsatisfiedLinkError` for
  `DriverStationJNI.nativeGetAllianceStation()`.
- **Inputs:** CTest `nm -D --defined-only` verifier for `libwpiHaljni.so`.
- **Expected outputs:** Defined exported symbols include
  `Java_edu_wpi_first_hal_DriverStationJNI_nativeGetAllianceStation`.
- **Determinism notes:** Static artifact inspection only.

### C54-2 - `nativeGetAllianceStation` returns cached station enum value

- **Layer / contract:** JNI scalar adapter over C HAL alliance-station reader.
- **Bug class caught:** Wrong enum mapping, stale default, or adapter bypassing
  the C HAL reader.
- **Inputs:** Installed shim with cached `ds_state.station =
  alliance_station::blue_3`.
- **Expected outputs:** Return value is `HAL_AllianceStationID_kBlue3` / `6`.
- **Determinism notes:** In-process.

### C54-3 - `nativeGetAllianceStation` preserves all enum integer mappings

- **Layer / contract:** WPILib Java-facing station enum parity.
- **Bug class caught:** Off-by-one or red/blue ordering drift.
- **Inputs:** For each backend `alliance_station` value unknown/red_1/red_2/
  red_3/blue_1/blue_2/blue_3, inject a cached DS state and call JNI.
- **Expected outputs:** Returned ints are exactly `0, 1, 2, 3, 4, 5, 6`.
- **Determinism notes:** In-process table-driven test.

### C54-4 - `nativeGetAllianceStation` default paths return unknown

- **Layer / contract:** JNI adapter inherits C HAL no-shim and empty-cache
  defaults while dropping status.
- **Bug class caught:** Garbage return on no-shim or empty-cache paths, or
  leaking status codes through the Java-facing return value.
- **Inputs:** Clear global/JNI fallback state and call JNI. Then install a shim
  with empty DS cache and call again.
- **Expected outputs:** Both return `HAL_AllianceStationID_kUnknown` / `0`.
- **Determinism notes:** In-process.

### C54-5 - manual Java launch advances past alliance-station blocker

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** Direct Java command with `build/src/backend/shim` first in native
  paths.
- **Expected outputs:** Output no longer contains missing JNI errors for
  `nativeGetAllianceStation`; the next missing JNI/runtime behavior drives
  Cycle 55.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts.
