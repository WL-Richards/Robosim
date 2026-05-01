# Cycle 50 test plan — Non-HALSIM HAL JNI startup artifact

Status: implemented.

Reviewer: first pass rejected symbol-only coverage; second pass requested
`HAL.initialize` behavior; final revised plan approved as ready.

## Context

Cycle 49 rejected the WPILib desktop `libwpiHaljni.so` as a core shim launch
dependency because its undefined-symbol surface includes `HALSIM_*`. Cycle 50
starts the repo-owned replacement boundary: a minimal non-HALSIM
`libwpiHaljni.so` startup artifact that exports the native methods WPILib
`RobotBase.startRobot` calls before entering the robot loop.

Observed WPILib 2026.2.1 startup JNI symbol names:

- `Java_edu_wpi_first_hal_HAL_initialize`
- `Java_edu_wpi_first_hal_HAL_shutdown`
- `Java_edu_wpi_first_hal_HAL_hasMain`
- `Java_edu_wpi_first_hal_HAL_runMain`
- `Java_edu_wpi_first_hal_HAL_exitMain`
- `Java_edu_wpi_first_hal_HAL_terminate`
- `Java_edu_wpi_first_hal_DriverStationJNI_refreshDSData`
- `Java_edu_wpi_first_hal_DriverStationJNI_report`
- `Java_edu_wpi_first_hal_NotifierJNI_setHALThreadPriority`
- `Java_edu_wpi_first_hal_HALUtil_getHALRuntimeType`
- `Java_edu_wpi_first_hal_HALUtil_getFPGATime`

## Scope

In:

- Add shared target `robosim_wpi_hal_jni` that outputs exactly
  `libwpiHaljni.so`.
- Add `hal_jni.cpp` exporting the startup JNI symbols above.
- Keep JNI methods as thin adapters over already-implemented C HAL surfaces
  where possible.
- Return `false` from `HAL.hasMain()` so WPILib uses the direct Java
  `runRobot` path.
- Make `runMain`, `exitMain`, and `terminate` v0 no-ops.
- Avoid a CMake/JDK dependency by defining only minimal JNI scalar/pointer
  aliases needed for signatures. The slice does not dereference `JNIEnv`.
- Verify artifact name, exported symbols, non-HALSIM classifier acceptance,
  and callable adapter behavior.

Out:

- No `HALSIM_*` symbols or stubs.
- No full Java robot boot success claim.
- No JNI string conversion for `DriverStationJNI.report`; v0 passes a null
  feature pointer to `HAL_Report`.
- No full HALUtil, DriverStationJNI, NotifierJNI, or device-family JNI
  coverage.
- No Java object field writes or `JNIEnv` calls.

## v0 decisions

- **D-C50-JNI-ARTIFACT:** The core launch path owns a repo-built
  `libwpiHaljni.so` artifact instead of using WPILib's desktop-simulation JNI.
- **D-C50-NO-HALSIM-DEPS:** The artifact must pass the Cycle 49 classifier.
- **D-C50-STARTUP-SKELETON:** Export only pre-loop startup JNI symbols for
  now; missing later JNI symbols drive later cycles.
- **D-C50-HASMAIN-FALSE:** `HAL.hasMain()` returns false so
  `RobotBase.startRobot` calls the direct Java `runRobot` path.
- **D-C50-JNI-STRING-DEFERRED:** `DriverStationJNI.report(..., String)` does
  not decode the Java string in this slice; it delegates to `HAL_Report` with a
  null feature.

## Tests

### C50-1 — `libwpiHaljni.so` artifact exists at the CMake target path

- **Layer / contract:** Layer 2 HAL shim Java launch boundary.
- **Bug class caught:** Build produces the wrong filename or no JNI artifact,
  so Java `RuntimeLoader` cannot load `wpiHaljni` from the shim directory.
- **Inputs:** CTest CMake script with `$<TARGET_FILE:robosim_wpi_hal_jni>`.
- **Expected outputs:** File exists and basename is exactly
  `libwpiHaljni.so`.
- **Determinism notes:** Build artifact inspection only.
- **Assumptions / stubs:** Artifact existence, not launch success.

### C50-2 — startup JNI artifact exports the required RobotBase startup symbols

- **Layer / contract:** Java JNI ABI boundary for WPILib startup.
- **Bug class caught:** Java native resolution fails before reaching robot code
  because a pre-loop native method is absent or C++ mangled.
- **Inputs:** `nm -D --defined-only` on `robosim_wpi_hal_jni`, requiring the
  exact observed symbols listed in the context section.
- **Expected outputs:** All required symbols are defined dynamic symbols.
- **Determinism notes:** Artifact inspection only.
- **Assumptions / stubs:** Pins presence, not complete method semantics.

### C50-3 — startup JNI artifact is accepted by the non-HALSIM classifier

- **Layer / contract:** Cycle 49 non-HALSIM launch dependency policy.
- **Bug class caught:** JNI target accidentally links against desktop sim HAL
  or imports `HALSIM_*` while satisfying startup symbols.
- **Inputs:** Cycle 49 classifier run against
  `$<TARGET_FILE:robosim_wpi_hal_jni>` with CMake's `nm`.
- **Expected outputs:** Classifier exits zero.
- **Determinism notes:** Artifact inspection only.
- **Assumptions / stubs:** Passing the classifier means non-HALSIM-compatible,
  not launch complete.

### C50-4 — startup JNI methods expose the pinned v0 adapter semantics

- **Layer / contract:** Java JNI adapter behavior for pre-loop WPILib startup
  methods.
- **Bug class caught:** Library exports the right symbols but returns wrong
  control-flow values or does not delegate to C HAL surfaces.
- **Inputs:** Direct C++ calls to exported JNI functions with null
  `JNIEnv`/`jclass`/`jstring` placeholders.
- **Expected outputs:**
  - no-shim `HAL_initialize` returns JNI false.
  - installed-shim `HAL_initialize` returns JNI true.
  - `HAL_shutdown` detaches the installed global shim.
  - `HAL_hasMain` returns JNI false.
  - `HALUtil_getHALRuntimeType` equals `HAL_GetRuntimeType()`.
  - no-shim `HALUtil_getFPGATime` returns 0.
  - no-shim `NotifierJNI_setHALThreadPriority` returns JNI false.
  - installed-shim `DriverStationJNI_report` returns 1 and records one usage
    report with resource 29, instance 3, context 7, and empty feature.
- **Determinism notes:** Pure in-process C++ calls; no JVM, clock, or native
  loader.
- **Assumptions / stubs:** `JNIEnv`, `jclass`, and `jstring` are opaque
  placeholders because this slice intentionally does not decode strings or
  access Java objects.

### C50-5 — existing C HAL and shim behavior remain green

- **Layer / contract:** Existing C HAL behavior and shared `libwpiHal.so`
  artifact.
- **Bug class caught:** Adding JNI target perturbs core C HAL behavior or
  artifact boundaries.
- **Inputs:** Existing artifact checks and shim test binary.
- **Expected outputs:** Green.
- **Determinism notes:** Existing deterministic C++ and CMake/nm tests.

## Future cycle unlocked

The next launch cycle can put both repo-owned artifacts first in
`java.library.path` and let the first missing later JNI method or real `HAL_*`
runtime behavior drive the next slice.
