# Cycle 51 test plan — JNI startup fallback shim installation

Status: implemented.

Reviewer: approved as ready.

## Context

Cycle 50 added a repo-owned non-HALSIM `libwpiHaljni.so` startup artifact.
Manual Java verification now loads that artifact and fails at
`HAL.initialize` because no host launcher installed a `shim_core` instance.

The lower-level C HAL lifecycle contract remains important:
`HAL_Initialize` succeeds only when a shim has already been installed. Cycle 51
adds fallback creation at the JNI launch boundary, not inside C HAL.

## Scope

In:

- Add JNI-owned launch state in `hal_jni.cpp`.
- On `Java_edu_wpi_first_hal_HAL_initialize`, create and install a real
  `shim_core` only when `shim_core::current()` is null.
- Use the normal `shim_core::make` path with a default boot descriptor.
- Preserve the normal boot publication and do not synthesize boot_ack or
  connected state.
- Preserve preinstalled host shims: JNI initialize must not replace them.
- Make JNI shutdown destroy only JNI-owned fallback state; host-owned shims are
  detached but not destroyed.
- Add namespace-only test hooks for deterministic reset/observation.

Out:

- No sim-core process or shared-memory fd handoff.
- No boot_ack injection or fake connected state.
- No Java process success claim in CTest.
- No device-family HAL/JNI surfaces.

## v0 decisions

- **D-C51-JNI-OWNS-FALLBACK-SHIM:** JNI creates a real shim only when Java
  starts without a preinstalled host shim.
- **D-C51-C-HAL-INITIALIZE-UNCHANGED:** C `HAL_Initialize` no-shim behavior
  remains failure; only the JNI launch path performs fallback creation.
- **D-C51-BOOT-PUBLISHED-NOT-ACKED:** Fallback creation publishes the normal
  boot envelope and does not synthesize boot_ack/connected.
- **D-C51-HOST-SHIM-WINS:** A preinstalled host shim is never replaced by JNI
  fallback.
- **D-C51-JNI-SHUTDOWN-OWNERSHIP:** JNI shutdown destroys only JNI-owned
  fallback state; host-owned shims are only globally detached by existing
  `HAL_Shutdown` semantics.

## Tests

### C51-1 — JNI initialize with no installed shim creates and installs a real fallback shim

- **Layer / contract:** Java launch boundary above C HAL lifecycle.
- **Bug class caught:** Java launch still fails at `HAL.initialize`, or passes
  by returning true without a real shim.
- **Inputs:** Reset JNI launch state, clear global, call
  `Java_edu_wpi_first_hal_HAL_initialize(nullptr, nullptr, 500, 0)`.
- **Expected outputs:** Returns JNI true; `shim_core::current()` is non-null;
  observer hook reports fallback ownership; fallback boot descriptor is
  roboRIO2/team 0/vendor 0/version `2026.2.1`; fallback backend_to_core lane
  contains a valid boot envelope with byte-identical descriptor; current shim is
  not connected because no boot_ack was injected.
- **Determinism notes:** In-process static state reset hook; no Java process,
  clock, or network.
- **Assumptions / stubs:** Hooks are namespace-only and not exported as C ABI.

### C51-2 — C HAL Initialize no-shim behavior remains failure

- **Layer / contract:** Existing C HAL lifecycle contract.
- **Bug class caught:** Fallback accidentally moves into `HAL_Initialize` and
  masks missing host install for all C callers.
- **Inputs:** Reset JNI launch state, clear global, call `HAL_Initialize(500,
  0)`.
- **Expected outputs:** Returns 0 and `shim_core::current() == nullptr`.
- **Determinism notes:** In-process.

### C51-3 — JNI initialize does not replace preinstalled host shim

- **Layer / contract:** Host ownership precedence.
- **Bug class caught:** JNI fallback leaks/replaces the host-owned shim,
  publishes an extra boot envelope, or changes boot metadata.
- **Inputs:** Create and install a connected shim with non-default boot
  descriptor; reset JNI launch state first; call JNI initialize.
- **Expected outputs:** Returns true; `shim_core::current()` is the original
  host shim pointer; fallback observer reports no fallback owned; host shim
  boot metadata unchanged; no fallback boot envelope exists.
- **Determinism notes:** In-process.

### C51-4 — JNI shutdown destroys only JNI-owned fallback state

- **Layer / contract:** JNI fallback ownership/lifecycle.
- **Bug class caught:** Fallback pointer dangles after shutdown, or host-owned
  shim storage is destroyed incorrectly.
- **Inputs A:** Reset/clear, JNI initialize fallback, save pointer, JNI
  shutdown.
- **Expected A:** Global current is null; fallback observer reports no fallback
  owned and saved pointer is no longer current.
- **Inputs B:** Install host shim and call JNI shutdown.
- **Expected B:** Global current is null per existing `HAL_Shutdown` detach;
  host shim object remains usable and boot metadata is unchanged; fallback
  observer reports no fallback owned.
- **Determinism notes:** In-process.

### C51-5 — manual Java launch advances past failed initialize

- **Layer / contract:** Diagnostic Java launch smoke, outside normal CTest.
- **Inputs:** Direct Java command with `build/src/backend/shim` first in native
  paths.
- **Expected outputs:** Output no longer contains `Failed to initialize.
  Terminating`; later missing JNI/runtime behavior drives Cycle 52.
- **Determinism notes:** Manual because it depends on local JDK/template
  artifacts.
