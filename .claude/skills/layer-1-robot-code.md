---
name: layer-1-robot-code
description: Use when working on the user-facing robot code surface — how unmodified Java / C++ / Python WPILib robot projects are loaded, built, and bound to our HAL. Covers language-specific binding shims, vendor library loading, and the contract that the user's robot code must run with zero source changes.
---

# Layer 1 — Robot code

The user's WPILib project. Java (incl. Kotlin), C++, or Python (RobotPy).
**We do not modify user code.** The user's `Robot.<lang>` and dependencies
build the way they would for deployment.

## Scope

Layer 1 is the boundary, not the implementation. We don't write robot
code; we ensure the robot code can run on top of our backend. The actual
work in this layer is:

- **Binding shims** so each language's WPILib HAL calls reach our backend.
- **Build and packaging** support so a normal robot project can launch
  through our launcher.
- **Vendor library compatibility** so `Phoenix6.jar`, `REVLib.jar`, etc.
  load against our HAL like they would on a real robot.

## Public contract

The user runs:

```
robosim run --project <path-to-robot-project> [--config <robot-desc>]
```

…and their robot code begins executing as if deployed. The contract:

- Source files unchanged.
- `build.gradle` (or equivalent) unchanged in any robosim-specific way.
- Vendor SDKs unchanged.
- Behavior observable through HAL is the same as a real robot, within
  the validation tolerances of the layer 2–4 models below.

If a user has to add `if (RobotBase.isSimulation())` to make their code
work, we have failed the layer.

## Languages

### Java / Kotlin

WPILib Java compiles to `.jar` and links to native code through JNI
(`wpiHaljni`, `wpimathjni`, `cscorejnicvstatic`, …). On a real robot
these JNIs talk to the FPGA HAL; in our sim they talk to our backend.

- Backend supplies a `wpiHaljni` matching the WPILib API for the
  targeted year.
- Vendor JNIs (`Phoenix6Jni`, `REVLibJni`) likewise loaded from our
  vendor models.
- JVM is launched in-process under the backend or as a child process
  under control of the backend (decision tied to OQ-1).

### C++

WPILib C++ links to `wpilibc`, `hal`, `ntcore`, etc. Same rules as
Java but at the shared-library level: backend supplies our HAL .so /
.dll matching the WPILib API.

### Python (RobotPy)

RobotPy uses `pyfrc` and `robotpy-halsim` (or its successors) to bind
to the HAL via a Python C extension. We satisfy the same HAL surface
through the binding the user's RobotPy version expects.

## Vendor library loading

Phoenix 6, REVLib, and similar:

- Live as JARs (Java) and shared libs (C++/Python) shipped by the
  vendor.
- Their JNIs / native sides expect to call into HAL or directly into
  device drivers (CTRE has its own libraries).
- Our Layer 3 (vendor firmware models) implements the device side;
  Layer 1 just makes sure the right shared lib is on the load path.

`[OPEN]` Whether we ship our own vendor "stub" libs that route to
Layer 3, or intercept the real vendor libs at the JNI boundary,
depends on what each vendor's library actually does internally.
Per-vendor decision, captured in each vendor's own skill when added.

## Build and packaging

The launcher needs to:

- Detect project language (Java / C++ / Python) from project layout.
- Build the project if needed (delegating to GradleRIO / CMake / pip).
- Configure the load path so our HAL implementation, not the
  RoboRIO's, is found.
- Pass any user-supplied robot description / config through to Layer 2.

For Java, GradleRIO has a `simulateJava` task we can ride on rather
than reimplement; we'll likely supply our own classpath / native lib
overrides.

## Determinism implications

- The user's robot code may use `Math.random()`, system time, etc.
  These pass through Layer 2's syscall surface and must be
  intercepted for determinism.
- Filesystem and network access from user code (some teams log to
  files or expose web dashboards) must be sandboxed and recorded for
  replay. Decision in OQ-5.

## Hard parts and gotchas

- **WPILib API churn.** Each season changes signatures and adds HAL
  entries. We track upstream, version pinned per release.
- **Vendor library lifecycle.** Phoenix 5 / Phoenix 6 split is a real
  fork in behavior; some teams still use Phoenix 5. Our story per
  vendor is documented in that vendor's skill.
- **JIT and ahead-of-time variance.** The same Java code can behave
  differently under JIT warmup vs. AOT. For determinism on long runs,
  we may need to pin JVM flags.
- **Native library versioning.** WPILib native libs are tied to GLIBC
  versions on the RIO. Our host build picks its own ABI; mismatches
  here cause undebuggable load errors.

## Validation

A "Layer 1 works" demonstration is: a stock WPILib example project
(e.g. `TimedRobot` template) builds, launches under our runner, and
runs for N ticks producing the expected periodic-loop output, without
any source changes. This is a basic smoke test — it does not validate
fidelity of any device or physics, only that the boundary is wired.

## Files / paths (TBD)

To be filled when the project scaffold lands. Placeholder:

- `loader/` — language detection, build delegation, classpath/library
  configuration.
- `bindings/java/`, `bindings/cpp/`, `bindings/python/` — per-
  language adapters from user code into the backend's HAL surface.

## Cross-references

- `layer-2-control-system-backend.md` — the HAL we present.
- `layer-3-device-bus-vendor-firmware.md` — the vendor firmware side
  of vendor libs.
- `docs/OPEN_QUESTIONS.md` — OQ-1 (execution model), OQ-3 (language).
