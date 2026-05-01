# Cycle 48 test plan — WPILib JNI launch advances past HandleBase ABI

Status: implemented.

Reviewer: first pass requested that the shared-artifact verifier accept
defined C++ ABI object symbols such as `V` RTTI/vtable entries instead of
function/text symbols only; the revised plan was approved as ready.

## Context

Cycle 47 added a replaceable `libwpiHal.so` shared artifact built from the
shim C HAL sources. A stock GradleRIO `simulateJava` run is not proof that
the shim HAL is loaded: GradleRIO's simulator child resets its native search
path to the template's extracted JNI directory and loads the stock
`libwpiHal.so`.

A direct Java launch of the empty `tools/timed-robot-template` jar with both
`LD_LIBRARY_PATH` and `-Djava.library.path` putting
`build/src/backend/shim/libwpiHal.so` first does exercise the shim artifact.
That launch currently fails before robot code starts:

```text
libwpiHaljni.so: undefined symbol: _ZTIN3hal10HandleBaseE
```

That symbol is C++ RTTI for `hal::HandleBase`. WPILib's `libwpiHaljni.so`
depends on a small set of non-C HAL C++ support symbols from `libwpiHal.so`
before the Java robot can bind through to the C HAL calls already implemented
by cycles 12-47. Cycle 48 adds the smallest support ABI surface needed to move
past the first blocker.

Primary source references:

- WPILib 2026.2.2 C++ docs for `hal/handles/HandlesInternal.h`, which defines
  `hal::HandleBase` as the base for HAL handles and lists the related handle
  helpers.
- The local WPILib 2026.2.1 native artifact
  `tools/timed-robot-template/build/jni/release/libwpiHal.so`, whose dynamic
  symbol table exports `hal::HandleBase` constructor, destructor, RTTI, vtable,
  `ResetHandles`, and `ResetGlobalHandles`.

## Scope

In:

- Add minimal WPILib-compatible C++ ABI exports for `hal::HandleBase`.
- Keep this support surface behaviorally inert for v0: no handle registry,
  no device-family allocation, no global reset side effects.
- Extend the shared-artifact CTest export check to require the exact
  `hal::HandleBase` dynamic symbols that blocked the direct Java launch.
- Add a focused direct-Java smoke command to the verification checklist:
  it is expected to fail later, but must no longer fail on
  `_ZTIN3hal10HandleBaseE`.

Out:

- No broad HAL handle-resource implementation.
- No `HALSIM_*` callback/state surface.
- No DIO/PWM/analog/device-family behavior.
- No GradleRIO `simulateJava` pass/fail assertion; Gradle's simulator task
  currently hides the shim by resetting the native path to the stock JNI
  extraction directory.
- No claim that the robot boots fully through the shim.

## v0 decisions

- **D-C48-CXX-SUPPORT-ABI:** The shim's `libwpiHal.so` must export the same
  `hal::HandleBase` dynamic symbol names that `libwpiHaljni.so` links against:
  constructor, destructor, RTTI, vtable, `ResetHandles`, and
  `ResetGlobalHandles`.
- **D-C48-INERT-HANDLEBASE:** `HandleBase` constructor/destructor and reset
  calls are no-ops in v0. The class only exists to satisfy WPILib JNI's
  support ABI until a real handle-resource implementation is demanded by
  device-family HAL cycles.
- **D-C48-SAME-SOURCES:** The support ABI source is included in the shared
  target and the static shim target source list, preserving Cycle 47's
  no-fork rule.
- **D-C48-DIRECT-JAVA-NOT-CTEST:** The direct Java launch is a manual focused
  verification step for this cycle rather than a normal CTest. It depends on
  the user's generated GradleRIO template and local WPILib/JDK install, both
  of which are outside the hermetic C++ test tree.

## Proposed tests

### C48-1 — `libwpiHal.so` exports WPILib `hal::HandleBase` support symbols

- **Layer / contract:** Layer 2 HAL shim launch boundary; dynamic C++ support
  ABI consumed by WPILib Java JNI.
- **Bug class caught:** The shared artifact still cannot satisfy
  `libwpiHaljni.so`'s first C++ support dependency, because RTTI/vtable or
  member functions are absent, hidden, or C++-ABI-mangled differently.
- **Inputs:** Extend `robosim_wpi_hal_exports_core_symbols` to include:
  - `_ZN3hal10HandleBaseC1Ev`
  - `_ZN3hal10HandleBaseC2Ev`
  - `_ZN3hal10HandleBaseD1Ev`
  - `_ZN3hal10HandleBaseD2Ev`
  - `_ZN3hal10HandleBase12ResetHandlesEv`
  - `_ZN3hal10HandleBase18ResetGlobalHandlesEv`
  - `_ZTIN3hal10HandleBaseE`
  - `_ZTSN3hal10HandleBaseE`
  - `_ZTVN3hal10HandleBaseE`
- **Expected outputs:** All symbols appear as defined dynamic symbols in the
  generated `libwpiHal.so`. The verifier must use `nm -D --defined-only` or
  an equivalent defined-symbol filter so it accepts both function/text symbols
  such as `T`/`W` and C++ ABI object symbols such as `V` for RTTI/typeinfo/
  vtables while still rejecting undefined-only matches.
- **Determinism notes:** Pure build artifact inspection through CMake's `nm`;
  no Java process, network, or wall-clock dependence.
- **Assumptions / stubs:** This pins ABI availability, not semantic handle
  allocation.

### C48-2 — direct empty TimedRobot launch advances past HandleBase RTTI failure

- **Layer / contract:** Shim-facing Java launch diagnostic.
- **Bug class caught:** We accidentally satisfy the CTest export list with the
  wrong artifact or fail to make the symbol visible to the actual Java/JNI
  loader path.
- **Inputs:** Manual verification command:

  ```bash
  timeout 15s env \
    JAVA_HOME=/home/will/wpilib/2026/jdk \
    PATH=/home/will/wpilib/2026/jdk/bin:$PATH \
    LD_LIBRARY_PATH=$PWD/build/src/backend/shim:$PWD/tools/timed-robot-template/build/jni/release \
    java \
      -Djava.library.path=$PWD/build/src/backend/shim:$PWD/tools/timed-robot-template/build/jni/release \
      -cp $PWD/tools/timed-robot-template/build/libs/timed-robot-template.jar \
      frc.robot.Main
  ```

- **Expected outputs:** The launch may still fail, but stderr must no longer
  contain `_ZTIN3hal10HandleBaseE`. If the new first failure is a `HALSIM_*`
  symbol, that is evidence this desktop-simulation JNI path is not the
  canonical robot-runtime path for the shim, not a request to implement
  HALSIM in the core HAL shim.
- **Determinism notes:** Manual diagnostic only. It is not added to CTest
  because it depends on the generated GradleRIO project and local WPILib
  install.
- **Assumptions / stubs:** The empty robot source remains unmodified.

### C48-3 — existing shim C HAL behavior remains green

- **Layer / contract:** Existing static `robosim_backend_shim` C HAL behavior.
- **Bug class caught:** Adding C++ support ABI to the shared/static source list
  perturbs existing C HAL tests or target wiring.
- **Inputs:** Existing `robosim_backend_shim_test` target.
- **Expected outputs:** The existing focused shim suite remains green.
- **Determinism notes:** Existing deterministic C++ tests.
- **Assumptions / stubs:** Existing suite owns per-HAL behavior; Cycle 48 only
  adds launch support ABI.

## Implementation sketch after approval

1. Add `hal_cpp_support.cpp` under `src/backend/shim/`.
2. Define a minimal polymorphic `hal::HandleBase` class with out-of-line
   constructor, virtual destructor, and no-op static reset functions.
3. Add the new source to the shared/static shim source list.
4. Extend the Cycle 47 export verifier so it accepts defined dynamic function
   and C++ ABI object symbols, then extend the symbol list with the
   `HandleBase` symbols.
5. Confirm the export test fails before implementation, then passes after.
6. Run focused shim tests and the full CTest suite.
7. Run the direct Java command and record the new first failure in
   `docs/STATUS_SIM_CORE.md` and `.claude/skills/hal-shim.md`.

## Future cycle unlocked

The direct Java launch's next first missing symbol should inform Cycle 49. If
the next blocker is `HALSIM_*`, Cycle 49 should pivot to a non-HALSIM robot
launch path or explicitly separate a desktop-simulation adapter from the real
shim path. If it is a specific `HAL_*` main/lifecycle symbol, that symbol
should be implemented before device-family surfaces.
