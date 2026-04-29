# robosim

**A high-fidelity, control-system-agnostic FRC robot simulator.** Run unmodified robot binaries against an emulated control system and a physically-accurate world. Sim/reality divergence is treated as a sim bug, not "good enough."

---

## Why this exists

FRC simulation today is split across tools that each handle one slice well:

- **WPILib's built-in simulation** ships a software HAL backend so robot code can run unmodified on a dev machine, plus per-mechanism state-space models (`ArmSim`, `ElevatorSim`, `FlywheelSim`, `DifferentialDrivetrainSim`, ...) and integration hooks for vendor sim APIs. It's the right tool for unit-testing logic and tuning controllers in isolation — but the mechanism sims are independent rather than coupled in a unified rigid-body world, there's no CAN bus model, vendor behavior isn't pinned to a firmware version, and control-system timing (FPGA cycle, RT scheduling) isn't modeled.
- **Vendor simulation APIs** (CTRE Phoenix Sim, REV simulation hooks) are vendor-provided substitutes that the robot code calls into directly. Great for teams writing tests against a single vendor — but they're not a cross-vendor truth source, can't reproduce CAN bus saturation under heavy status-frame load, and the robot code has to know it's being simulated.

robosim sits *underneath* the HAL, models the CAN bus and vendor firmware as peers on the bus, and couples them to a rigid-body world driven by MuJoCo. The robot binary doesn't know it's in a simulator. The goal is for every output of the sim — joint angles, current draw, brownout state, sensor latency, log timing — to be defensible against a real robot under the same inputs, with a stated error bound.

This is a research-grade ambition with a long horizon. v0 is one mechanism on one motor. v1+ grows the device set and the demo robots from there.

---

## Non-negotiables

These are the project's identity. A change that violates one of these is rejected on principle.

1. **Robot code is unmodified.** The sim sits *underneath* the HAL — robot code never imports a sim package or links a sim library.
2. **Control-system agnostic.** RoboRIO 2 is one backend. System Core or future control hardware is another. The sim core doesn't know which is loaded.
3. **Vendor-faithful.** Match firmware behavior, including quirks, against a stated firmware version.
4. **Deterministic and replayable.** Same seed + inputs → byte-identical state logs.
5. **Accuracy is measurable.** Every physical model ships with a stated error bound vs. a cited source. Models that haven't met their bound are marked `unvalidated`, and that taint propagates downstream.

---

## Architecture at a glance

Two processes, talking over a versioned protocol.

```
┌─────────────────────────────────────────────────────────┐
│ Robot Code Process                                      │
│   (T1: x86 native; T2: ARM under qemu-user)             │
│ ┌───────────────────────────────┐                       │
│ │ User's robot code             │                       │
│ │ (Java/C++/Python, unmodified) │                       │
│ └─────────────┬─────────────────┘                       │
│               │ WPILib HAL calls                        │
│ ┌─────────────▼─────────────────┐                       │
│ │ HAL shim (libhal.so / wpiHal) │                       │
│ │   state cache + sync layer    │                       │
│ │   + LD_PRELOAD libc time      │                       │
│ └─────────────┬─────────────────┘                       │
└───────────────┼─────────────────────────────────────────┘
                │ state-mirror protocol
                │   T1: shared memory + atomic sync
                │   T2: socket across qemu boundary
                ▼
┌─────────────────────────────────────────────────────────┐
│ Sim Core Process                                        │
│   authoritative clock, CAN bus, vendor firmware,        │
│   electrical/motor/sensor models, MuJoCo world,         │
│   DS synthesis or trace replay, field & game plugin     │
└─────────────┬───────────────────────────────────────────┘
              │ WPILOG + NetworkTables
              ▼
   DS endpoint, visualizer, AdvantageScope log sinks
```

Inside the sim core, work is organized into five layers, each with a stable interface to the layer above:

| Layer | What it owns | v0 scope |
|---|---|---|
| **L1 — Robot code** | The user's unmodified binary | Unchanged |
| **L2 — Control-system backend** | HAL shim, state cache, sync layer, time interception | T1 (native) + T2 (`qemu-user`) |
| **L3 — Device bus + vendor firmware** | CAN arbitration, status frames, vendor closed loops | Phoenix 6: Talon FX, CANcoder, Pigeon 2 |
| **L4 — Physical models** | Lumped electromechanical models, sensor noise/quantization, electrical | Kraken X60, CANcoder, battery/brownout |
| **L5 — World physics** | Rigid-body dynamics, contact, joint coupling | MuJoCo binding, single-DOF arm world |

Time is owned exclusively by the sim core. Robot code's view of `nanoTime`, `clock_gettime`, FPGA timestamps, and periodic-loop edges is synthesized via the HAL protocol and an `LD_PRELOAD` libc shim. This is what makes deterministic replay free.

The full design — process model, protocol semantics, validation methodology, open questions — lives in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## Repo layout

```
robosim/
├── CMakeLists.txt              # Top-level build, dependency pinning
├── CLAUDE.md                   # Repo orientation for Claude Code sessions
│
├── docs/
│   ├── V0_PLAN.md              # First implementation session entry point
│   ├── ARCHITECTURE.md         # Binding architecture document
│   ├── OPEN_QUESTIONS.md       # Decided + open design questions
│   └── REFERENCES.md           # External sources, datasheets, prior art
│
├── src/
│   ├── backend/                # L2 — HAL shim + state cache + sync
│   │   ├── common/             #   protocol structs, tier-agnostic
│   │   ├── tier1/              #   native: shared-memory transport
│   │   └── tier2/              #   qemu-user: socket transport
│   ├── sim_core/               # Sim core process entry point
│   ├── bus/can/                # L3 — CAN bus arbitration model
│   ├── vendors/phoenix6/       # L3 — Talon FX, CANcoder, Pigeon 2
│   ├── models/
│   │   ├── electrical/         # L4 — battery, brownout, PDH
│   │   ├── motors/             # L4 — Kraken X60, ...
│   │   └── sensors/            # L4 — sensor models
│   ├── world/
│   │   ├── dynamics/           # L5 — engine-agnostic interface
│   │   └── engine/mujoco/      # L5 — MuJoCo binding
│   ├── description/            # JSON robot description loader
│   ├── time/                   # Sim clock interface
│   ├── rng/                    # Seed distribution
│   ├── log/                    # WPILOG state-stream emitter
│   └── util/                   # Units, det_math, error types
│
├── tests/                      # Mirrors src/ exactly
│
├── tools/
│   └── rio-bench/              # Java/Gradle WPILib project — measures
│                               #   real-RIO HAL call costs to calibrate
│                               #   T1 fidelity (separate build)
│
├── descriptions/
│   └── v0-arm.json             # Single-DOF arm robot description
│
├── references/
│   └── rio2-hal-costs/         # Versioned HAL cost fixture files
│
├── cmake/toolchains/
│   └── arm-rio.cmake           # ARM cross-compile for tier 2
│
└── .claude/
    ├── skills/                 # Per-feature actionable knowledge
    └── agents/                 # TDD test-reviewer agent
```

---

## Build

Requirements: CMake ≥ 3.24, Clang ≥ 18 or GCC ≥ 13, Ninja, and a network connection on first configure (`FetchContent` pulls dependencies pinned by commit SHA).

**Native (Tier 1) host build:**

```bash
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build
```

**Sanitizer builds** — multiple may be combined; TSan must be selected alone:

```bash
cmake -B build-asan -GNinja -DCMAKE_BUILD_TYPE=Debug \
      -DROBOSIM_ENABLE_ASAN=ON -DROBOSIM_ENABLE_UBSAN=ON
cmake --build build-asan
ctest --test-dir build-asan
```

**ARM cross-compile (Tier 2 backend)** — verifies the ARM HAL shim builds. Full execution requires `qemu-user` and is not yet wired up:

```bash
cmake -B build-arm -GNinja \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-rio.cmake
cmake --build build-arm
```

**Layer 5 (physics).** MuJoCo is a heavy dependency, gated behind `-DROBOSIM_BUILD_PHYSICS=ON` and `OFF` by default until Layer 5 work lands. The rest of the tree builds without it.

```bash
cmake -B build -GNinja -DROBOSIM_BUILD_PHYSICS=ON
cmake --build build
```

CI runs `{clang, gcc} × {none, asan, ubsan, tsan}` plus the ARM cross-compile job. All combinations must be green to merge.
