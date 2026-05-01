# Cycle 49 test plan — Non-HALSIM Java launch path classifier

Status: implemented.

Reviewer: approved as ready. The reviewer specifically noted that the
classifier should parse undefined-symbol rows as symbol tokens instead of
matching arbitrary substrings.

## Context

Cycle 47 added a replaceable `libwpiHal.so` shared artifact. Cycle 48 added
the minimal `hal::HandleBase` C++ ABI required by WPILib's desktop
`libwpiHaljni.so`; the direct empty TimedRobot launch then advanced to:

```text
undefined symbol: HALSIM_CancelRoboRioVInCurrentCallback
```

That is not a core-shim implementation request. It identifies the WPILib
desktop-simulation JNI artifact as HALSIM-dependent. Cycle 49 adds a narrow
launch-harness classifier so future work can reject that dependency shape
without adding HALSIM stubs to the core shim.

## Scope

In:

- Add a CMake classifier script for candidate `libwpiHaljni.so` undefined
  symbols.
- Accept candidates that have normal undefined `HAL_*` dependencies and no
  undefined `HALSIM_*` dependencies.
- Reject candidates that have any undefined `HALSIM_*` dependency and report
  the first HALSIM symbol.
- Cover classifier behavior with committed text fixtures, keeping CTest
  independent of the user's generated `tools/timed-robot-template` project and
  local WPILib install.
- Record the manual diagnostic command for checking the generated template's
  desktop JNI artifact.

Out:

- No `HALSIM_*` symbols or stubs.
- No robot process launch success claim.
- No Java launch in normal CTest.
- No device-family C HAL additions.

## v0 decisions

- **D-C49-HALSIM-DISQUALIFIES:** A candidate JNI artifact with undefined
  `HALSIM_*` symbols is not a valid core shim launch dependency.
- **D-C49-HAL-UNDEFINED-OK:** Undefined `HAL_*` symbols are expected for a JNI
  bridge that binds through `libwpiHal.so`.
- **D-C49-HERMETIC-CLASSIFIER:** CTest covers classifier behavior with text
  fixtures; real WPILib/template artifacts remain manual diagnostics due local
  install and untracked-project dependence.
- **D-C49-NO-LAUNCH-COMMAND-IN-CTEST:** No Java launch enters CTest until the
  repo owns all required artifacts or a generated fixture.

## Tests

### C49-1 — classifier rejects desktop JNI candidates that require HALSIM

- **Layer / contract:** Layer 2 HAL shim launch boundary; non-HALSIM dependency
  gate.
- **Bug class caught:** Future launch work treats WPILib desktop-simulation
  `libwpiHaljni.so` as acceptable and starts adding HALSIM stubs to core shim.
- **Inputs:** Text fixture containing undefined symbols `HAL_Initialize`,
  `HAL_GetRuntimeType`, `HALSIM_CancelRoboRioVInCurrentCallback`, and
  `HALSIM_RegisterRoboRioVInVoltageCallback`.
- **Expected outputs:** Classifier exits nonzero and includes
  `HALSIM_CancelRoboRioVInCurrentCallback` as the first blocker.
- **Determinism notes:** Static text fixture; no Java process, native loader,
  local WPILib install, or untracked template dependency.
- **Assumptions / stubs:** The fixture models `nm -D --undefined-only` row
  shape. It tests the classifier policy, not the real WPILib binary contents.

### C49-2 — classifier accepts non-HALSIM JNI candidates with normal HAL dependencies

- **Layer / contract:** Same launch-boundary classifier, positive path.
- **Bug class caught:** The classifier rejects every JNI candidate just because
  it has expected `HAL_*` undefined symbols.
- **Inputs:** Text fixture containing undefined symbols `HAL_Initialize`,
  `HAL_Shutdown`, `HAL_GetRuntimeType`, `HAL_RefreshDSData`, plus non-HAL C/C++
  runtime symbols.
- **Expected outputs:** Classifier exits zero and reports that the candidate is
  non-HALSIM-compatible.
- **Determinism notes:** Static text fixture.
- **Assumptions / stubs:** This does not prove such a JNI exists yet; it only
  prevents the classifier from encoding an impossible policy.

### C49-3 — current robosim `libwpiHal.so` still exports launch symbols

- **Layer / contract:** Existing Cycle 47/48 shared HAL launch artifact checks.
- **Bug class caught:** Classifier or harness additions disturb the existing
  replaceable `libwpiHal.so` boundary.
- **Inputs:** Existing `robosim_wpi_hal_artifact_exists` and
  `robosim_wpi_hal_exports_core_symbols` CTests.
- **Expected outputs:** Existing checks remain green.
- **Determinism notes:** Existing CMake/nm tests.

## Manual diagnostic

When `tools/timed-robot-template` exists and has been built, run:

```bash
cmake \
  -DROBOSIM_HAL_JNI_PATH=$PWD/tools/timed-robot-template/build/jni/release/libwpiHaljni.so \
  -DROBOSIM_HAL_JNI_NM=$(command -v nm) \
  -P tests/backend/shim/classify_hal_jni_candidate.cmake
```

The current WPILib desktop JNI artifact is expected to fail this classifier
with `HALSIM_CancelRoboRioVInCurrentCallback` as the first blocker.

## Future cycle unlocked

The next launch implementation should provide or select a non-HALSIM JNI /
launcher boundary so the empty TimedRobot reaches real robot-runtime `HAL_*`
calls through the shim. Adding HALSIM to the core shim remains out of scope.
