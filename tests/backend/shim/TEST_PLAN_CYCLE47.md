# Cycle 47 test plan — WPILib HAL shared launch artifact

Status: implemented.

Reviewer: first pass requested that C47-2 reject undefined `U` matches
from `nm -D`; the revised plan was approved as ready to implement.

## Context

Cycles 12-46 built the shim C HAL ABI inside the static
`robosim_backend_shim` library and proved those symbols through direct
GoogleTest calls. The empty WPILib Java `TimedRobot` template in
`tools/timed-robot-template` now builds and runs under stock desktop
simulation, but that path loads WPILib native shared libraries:

- `libwpiHaljni.so`
- `libwpiHal.so`
- `libhalsim_*.so`

The repo currently does not build a shared `libwpiHal.so` replacement,
so an unmodified Java robot has no realistic way to bind to the shim.
Cycle 47 makes that launch artifact explicit. It does **not** claim the
empty robot can boot through the shim yet; it creates and verifies the
first required dynamic-link boundary so Cycle 48 can run the actual
Java smoke and let the first missing HAL symbol or runtime behavior
drive the next slice.

## Scope

In:

- Add a shared-library target that produces a file named
  `libwpiHal.so` from the same shim C HAL implementation used by the
  existing static `robosim_backend_shim` target.
- Keep the existing static target for unit tests and current consumers.
- Add CTest coverage that verifies the shared artifact exists at its
  generator-resolved path.
- Add CTest coverage that verifies the shared artifact exports the
  implemented C HAL symbols required by the current core runtime
  surface.
- Document that the empty TimedRobot template is a stock-baseline
  fixture, not yet a shim-passing test.

Out:

- No attempt to satisfy every undefined `HAL_*` reference in
  `libwpiHaljni.so`.
- No Java robot process launch through the shim in this cycle.
- No Tier 2 socket transport, LD_PRELOAD libc time shim, or automatic
  host installation from `HAL_Initialize`.
- No new device-family HAL behavior.

## v0 decisions

- **D-C47-SHARED-NAME:** The shared target's output filename is exactly
  `libwpiHal.so`, matching WPILib's Linux HAL library basename. The CMake
  target name may be project-specific to avoid colliding with future
  imported targets, but the produced file must be replaceable on a
  native-library search path.
- **D-C47-SAME-SOURCES:** The shared artifact is built from the same
  `shim_core.cpp` and `hal_c.cpp` sources as `robosim_backend_shim`.
  Cycle 47 must not fork HAL behavior between static tests and the
  launch artifact.
- **D-C47-STATIC-RETAINED:** `robosim_backend_shim` remains a static
  library. Existing tests continue linking it directly.
- **D-C47-EXPORT-C-HAL:** The shared artifact must dynamically export
  representative implemented `extern "C"` HAL symbols across the
  startup, metadata, DS, Notifier, CAN, and output-side surfaces. This
  catches accidental hidden visibility, C++ mangling, wrong target
  source lists, or building an empty placeholder library.
- **D-C47-NO-FALSE-SMOKE:** Cycle 47 must not call the Java robot launch
  green just because stock WPILib simulation works. The stock template
  run remains a baseline sanity check; the shim-facing launch starts
  only after a replaceable HAL shared artifact exists.
- **D-C47-PIC-ONLY-WHERE-NEEDED:** If shared linking requires position
  independent code for backend libraries, enable PIC for the backend
  common/tier1/shim libraries involved in this artifact. Do not make a
  broad repo-wide CMake policy change.

## Proposed tests

### C47-1 — `libwpiHal.so` artifact exists at the CMake target path

- **Layer / contract:** Layer 2 HAL shim launch boundary; CMake target
  contract for an unmodified WPILib process to discover a HAL shared
  library by basename.
- **Bug class caught:** The shared target is missing, has the wrong
  output name, or points CTest at a stale/manual file instead of the
  build artifact.
- **Inputs:** CTest invokes a small CMake script with
  `-DROBOSIM_WPI_HAL_PATH=$<TARGET_FILE:robosim_wpi_hal>`.
- **Expected outputs:** The path exists and its filename is
  `libwpiHal.so`.
- **Determinism notes:** Pure filesystem check against the current build
  tree. No wall-clock waits, network, or WPILib install dependency.
- **Assumptions / stubs:** This verifies the launch artifact boundary,
  not Java runtime success.

### C47-2 — shared artifact exports core implemented HAL symbols

- **Layer / contract:** C HAL ABI dynamic symbol contract.
- **Bug class caught:** Symbols are C++-mangled, hidden, not linked into
  the shared target, or accidentally dropped when CMake source lists are
  refactored.
- **Inputs:** CTest invokes the same CMake script with
  `-DROBOSIM_WPI_HAL_NM=<CMAKE_NM>` and
  `-DROBOSIM_WPI_HAL_REQUIRED_SYMBOLS=...`.
- **Expected outputs:** Dynamic symbol table contains all required
  names exactly as **defined exported function symbols**. The verifier
  accepts normal defined text/function entries such as `T` or `W` (and
  toolchain-equivalent defined dynamic entries) and rejects undefined
  matches such as `U`; a name appearing only as an unresolved reference
  does not satisfy the test.
  - `HAL_Initialize`
  - `HAL_Shutdown`
  - `HAL_GetFPGATime`
  - `HAL_RefreshDSData`
  - `HAL_WaitForNotifierAlarm`
  - `HAL_CAN_SendMessage`
  - `HAL_GetMatchInfo`
  - `HAL_SetJoystickOutputs`
  - `HAL_ObserveUserProgramTeleop`
  - `HAL_GetRuntimeType`
  - `HAL_Report`
  - `HAL_GetComments`
- **Determinism notes:** Uses the compiler toolchain's `nm` configured
  by CMake. No process launch or ordering dependency.
- **Assumptions / stubs:** This is a representative export set, not an
  exhaustive WPILib HAL closure. The full missing-symbol census belongs
  to the first shim Java launch after this artifact exists.

### C47-3 — existing static shim tests still link and run

- **Layer / contract:** Existing C HAL behavior tested through
  `robosim_backend_shim`.
- **Bug class caught:** Refactoring CMake to support the shared target
  silently changes/removes the static target used by the existing shim
  suite.
- **Inputs:** Existing `robosim_backend_shim_test` target.
- **Expected outputs:** The existing shim suite remains green.
- **Determinism notes:** No new assertions; this is regression coverage
  for D-C47-STATIC-RETAINED.
- **Assumptions / stubs:** Existing test suite already pins the
  per-symbol behavior. Cycle 47 only ensures the CMake refactor does not
  strand those tests.

## Implementation sketch after approval

1. Refactor `src/backend/shim/CMakeLists.txt` so the static and shared
   shim libraries share one source list.
2. Add `robosim_wpi_hal` as a shared library with
   `OUTPUT_NAME wpiHal`.
3. Set targeted PIC properties if required by the shared link.
4. Add `tests/backend/shim/verify_wpi_hal_artifact.cmake`.
5. Add two CTest entries from `tests/backend/shim/CMakeLists.txt`:
   artifact/name check and dynamic export check.
6. Update `docs/STATUS_SIM_CORE.md` and `.claude/skills/hal-shim.md`
   with Cycle 47's boundary and explicit non-goals.
7. Verify focused CTest and full CTest.

## Future cycle unlocked

Cycle 48 can now attempt the actual unmodified empty `TimedRobot`
shim-facing launch by putting the built `libwpiHal.so` ahead of
WPILib's native HAL library and capturing the first real dynamic-link
or runtime failure. That failure, not speculation, should pick the next
HAL family or launch-layer fix.
