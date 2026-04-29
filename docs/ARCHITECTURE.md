# Architecture

Status: **draft / design phase**. Decisions marked `[OPEN]` are tracked in
`OPEN_QUESTIONS.md` and not yet binding.

## Goals (restated)

Run unmodified FRC robot binaries against an emulated control system and a
physically accurate world, with results that match real hardware closely
enough to trust as a correctness signal. Be agnostic to the control system
(RoboRIO 2 today, System Core / future hardware tomorrow), agnostic to the
season (game logic is a plugin), and agnostic to the user's robot code
language (Java/C++/Python).

## Process model

The simulator is **not** a single process. The HAL boundary is the
strongest abstraction line in the system, and crossing process
boundaries on it cleanly is what enables the four-tier model
(see "Execution tiers" below) and a future real-RIO HIL mode.

That said: in tier 1 (native), the **HAL implementation lives in-
process with the user's robot code**, loaded as a shared library —
the same way the real RIO's HAL is a shared library inside the
robot code's address space. The sim core is a separate process. The
HAL boundary is the IPC boundary, not the process boundary between
HAL and robot code.

Process layout:

```
┌─────────────────────────────────────────────────────────┐
│ Robot Code Process                                      │
│   (T1: x86 JVM/native; T2: ARM under qemu-user)         │
│ ┌───────────────────────────────┐                       │
│ │ User's robot code             │                       │
│ │ (Java/C++/Python, unmodified) │                       │
│ └─────────────┬─────────────────┘                       │
│               │ WPILib HAL calls                        │
│ ┌─────────────▼─────────────────┐                       │
│ │ HAL shim (libhal.so / wpiHal) │                       │
│ │   - state cache               │                       │
│ │   - sync layer                │                       │
│ │   - libc time interception    │                       │
│ │     (LD_PRELOAD)              │                       │
│ └─────────────┬─────────────────┘                       │
└───────────────┼─────────────────────────────────────────┘
                │ state-mirror protocol
                │   T1: shared memory + atomic sync
                │   T2: socket across qemu boundary
                ▼
┌─────────────────────────────────────────────────────────┐
│ Sim Core Process                                        │
│   - Authoritative clock                                 │
│   - Device bus (CAN) + vendor firmware models           │
│   - Electrical model (battery, PDP, brownout)           │
│   - Motor + sensor physical models                      │
│   - World physics (Layer 5)                             │
│   - DS state synthesis or trace replay                  │
│   - Field & game logic (plugin)                         │
└─────────────┬───────────────────────────────────────────┘
              │ state stream (WPILOG + NetworkTables)
              ▼
┌──────────────┐ ┌──────────────────┐ ┌────────────────────┐
│ DS endpoint  │ │ Visualizer       │ │ Log Sink           │
│ (real or     │ │ (3D, separate)   │ │ (WPILOG /          │
│  trace)      │ │                  │ │  AdvantageScope)   │
└──────────────┘ └──────────────────┘ └────────────────────┘
```

DS state is **not** owned by the Robot Code Process. The Sim Core
either synthesizes it from a recorded trace (deterministic mode) or
relays from a connected real DS (soft real-time mode), and pushes it
across the HAL protocol like any other state.

## Execution tiers

The HAL protocol is the same across tiers; only the implementation
shifts. **v0 ships tiers 1 and 2.**

- **Tier 1 — native.** User's robot code, JVM, and our HAL shim all
  built for x86 host. HAL ↔ Sim Core via shared memory + atomic sync.
  Daily development driver. Loop-overrun fidelity comes from a RIO
  HAL-cost table (see `tools/rio-bench/`) plus cgroups/CPU throttling.
  ARM ISA fidelity: none.
- **Tier 2 — user-mode QEMU.** Robot binary, JVM, WPILib native, and
  vendor libs are ARM, run under `qemu-user`. HAL shim is ARM,
  intercepts via LD_PRELOAD, speaks the same protocol over a socket
  to the x86 Sim Core. No kernel emulation, no NI Linux RT boot.
  Default fidelity validation tier — cheap enough to run on every
  test.
- **Tier 3 — full-system QEMU.** Adds NI Linux RT + PREEMPT_RT
  scheduling on top of tier 2's fidelity. RIOEmulator-style.
  Reserved for RT-scheduling-specific bugs and competition-gate
  validation. Built on demand, not in v0.
- **Tier 4 — HIL.** Real RIO over network, far future. Architecturally
  supported (same protocol) but not built.

## HAL boundary protocol

The HAL shim never talks to hardware. Instead, every HAL call reads
from or writes to a **local state cache** inside the shim, and the
cache exchanges deltas with the Sim Core at **tick-aligned sync
points** (plus on-demand sync for any HAL call that genuinely needs a
synchronous round-trip mid-tick — these are identified per call as
the surface grows).

Protocol shape:

- **Typed schema** describing every HAL state element (FPGA registers,
  IO state, counters, CAN buffers, status frames) plus an event
  stream (CAN frames in/out, interrupts, DS packets).
- **Read** — reads from local cache; returns instantly.
- **Write** — buffers to local cache.
- **Sync** — at tick boundaries (and on-demand): backend pushes
  accumulated writes + outgoing events, Sim Core advances time, pushes
  back state updates + incoming events. Every sync exchange is
  timestamped and logged — this is what makes replay free.
- **Schema versioning.** No upfront full-HAL schema document. v0
  schema covers v0 scope; subsequent releases add surface and bump
  the protocol version. Tests reference the version they were written
  against.

The schema and code are designed to look like a network protocol even
when implemented in-process. The local-fast-path is an implementation
detail, not the contract.

## Time

**Hard rule: the Sim Core owns the clock; nothing else has one.**
Robot code's view of time (`Timer.getFPGATimestamp()`, the periodic
loop tick, RT scheduling, `System.nanoTime`, `clock_gettime`,
`Thread.sleep`) is synthesized from this clock.

This rule cascades through more entry points than HAL alone. In tier
1 the HAL shim ships an LD_PRELOAD libc shim that intercepts
`clock_gettime`, `gettimeofday`, `nanosleep`, `clock_nanosleep`, etc.,
returning sim time. JVM intrinsics (`System.nanoTime`,
`System.currentTimeMillis`) bottom out in these syscalls and so see
sim time too. Tier 2 uses the same mechanism inside qemu-user.

Two run modes:

- **Deterministic** (default for tests / CI). No wall-clock anywhere.
  DS is either disabled, or replayed from a recorded trace. RNG
  seeded from a fixed root. Sim time advances as fast as compute
  allows. Same inputs → byte-identical state logs.
- **Soft real-time** (human-in-the-loop). Sim time advances at ~1×
  wall-clock with feedback control; real DS connected. Non-
  deterministic by construction. Useful for human driving and feeling
  latency.

In both modes, robot code observes sim time. Only the *rate* at which
sim time advances differs.

Default `java.util.Random` seeds from `System.nanoTime()`; controlling
time controls Random automatically. `SecureRandom` (which reads
`/dev/urandom` / `getrandom()`) is a separate leak deferred to v1+.
JVM internals (GC trigger heuristics, JIT compilation timing) are not
bit-deterministic in v0; higher-determinism modes (pinned GC, AOT)
come later if needed.

Inner-loop rates (target):

- Physics: 1 kHz (1 ms step). Configurable up to 10 kHz for stiff
  systems (e.g. high-gear-ratio elevators).
- Motor electrical model: same as physics, or 10× physics for stiff
  motor dynamics under PWM. `[OPEN]`
- CAN bus: 1 Mbit nominal, modeled as discrete frames with realistic
  inter-frame gaps and arbitration.
- Robot code main loop: 50 Hz (20 ms) like the real robot, but exact
  tick edge is controlled by the sync schedule.

## Determinism

Two runs with the same:

- Sim Core seed
- Robot binary
- DS input trace (or none, if disabled)
- Configuration (robot description, field, plugin set, vendor firmware
  versions)

…must produce byte-identical state logs. This is enforced by:

- Single authoritative clock; no `wall_clock_now()` in models.
- LD_PRELOAD shim intercepts time-querying syscalls in the Robot Code
  Process so JVM and user-code intrinsics see sim time.
- All RNG seeded from a single root; default Random seeds from
  intercepted `nanoTime()` and so falls out automatically.
- Ordered, timestamped message delivery on the device bus.
- Robot code IO that touches the OS (file, network, time) goes
  through the HAL protocol and is recorded/replayed.
- Every sync exchange between HAL shim and Sim Core is logged and
  replayable.

Non-deterministic mode is "soft real-time with a real DS." There is
no "deterministic with recorded inputs replayed back" mode separate
from the default — the deterministic mode *is* recorded-inputs.

## Control system backend interface

A "backend" is the bundle of (HAL shim + state cache + sync layer +
LD_PRELOAD libc shim + tier-specific transport). Backends differ
across tiers and across control systems (RoboRIO 2 today, System Core
later) but all implement the same HAL boundary protocol. The Sim
Core never knows which backend is loaded.

The contract the backend exposes to the Sim Core is the **protocol**
(see "HAL boundary protocol" above), not a Rust/C++ trait. From the
Sim Core's side, a backend is "a peer that speaks the protocol over
some transport." Switching control systems changes the backend
implementation; the Sim Core does not change.

The HAL shim itself is always loaded in-process with the user's robot
code, regardless of tier — that's a HAL ABI requirement, not a
choice. What differs across tiers is what's "in process" with what:

- T1: x86 user code + x86 HAL shim, sharing memory with x86 Sim Core.
- T2: ARM user code + ARM HAL shim under qemu-user, talking via
  socket to x86 Sim Core.
- T3: ARM user code + ARM HAL shim inside full-system QEMU running
  NI Linux RT, talking via socket to x86 Sim Core.
- T4: ARM user code + ARM HAL shim on a real RIO, talking via
  network to x86 Sim Core.

## Device bus interface

The device bus carries CAN frames, DIO/PWM signals, SPI/I²C transactions,
USB camera frames, and anything else a peripheral talks over. Vendor device
models register on the bus by (transport, address) and respond to traffic.

Key property: frames have **timestamps** and **arbitration order**. Two
devices both trying to send at the same instant resolve like real CAN. We
do not assume infinite bandwidth. CAN saturation under heavy status-frame
load is a real failure mode on FRC robots and we want to reproduce it.

## Vendor firmware models

Each device model owns:

- Its register / API surface as exposed to the vendor SDK on the robot side.
- Its closed-loop controller(s), e.g. Talon FX motion magic, Spark Max
  PIDF + smart-motion, with the same numerics and quirks as the real
  firmware.
- Its status frame schedule (configurable, like the real device).
- Its electrical/mechanical port — what it expects from the motor model
  underneath (voltage in, current out, rotor angle/velocity).

Models live in `vendors/<name>/` and are versioned per firmware revision.
A robot configured for Phoenix 6 firmware vX.Y.Z runs against the matching
model.

## Physical models

Two clean tiers:

1. **Lumped electromechanical models.** Per-motor: DC machine equations
   with measured constants from the manufacturer's curves, including
   thermal model, friction, and gearbox losses. Per-sensor: quantization,
   noise, latency.
2. **Rigid body world.** A 3D physics engine simulating the chassis,
   mechanisms, game pieces, and field. Forces from motors are applied as
   joint torques; reaction forces feed back into the motor model (this is
   what makes stalling and contact realistic).

These tiers communicate through a small, stable interface so we can swap
the physics engine without rewriting the motor code, and swap a motor model
without touching the world. `[OPEN]` Physics engine choice.

## Robot description

A robot is described declaratively in a single file (URDF-like, but
FRC-aware). It names every motor, sensor, and mechanism, wires them to
ports on the control system, and gives the rigid-body geometry/inertia.
This file is the source of truth — visualization, physics, and device
binding all read from it.

`[OPEN]` URDF + extensions vs. our own format vs. MuJoCo MJCF vs. SDF.

## Logging and observability

Every state variable produced by the Sim Core is logged in WPILOG format
so AdvantageScope works out of the box. NetworkTables is also exposed for
live viewers. The visualizer is a separate process consuming this stream;
nothing in the sim core depends on the visualizer being attached.

## Validation

We will not ship a model without a validation test. Validation is:

- A bench measurement on real hardware (motor under known load, encoder
  reading at known shaft angle, etc.) — or a vendor-published curve when
  bench access isn't available — captured in a fixture file.
- A sim test that runs the model under the same conditions and asserts
  the output is within a stated error bound.

When error bounds are not met, the model is marked `unvalidated` and any
downstream test that depends on it inherits that taint. We do not silently
ship inaccurate models.

## v0 scope (decided 2026-04-28)

The first concrete deliverable. Everything below is binding for v0;
post-v0 surface grows on demand.

**Tiers shipping in v0:** Tier 1 (native) and Tier 2 (user-mode QEMU).
T3 on demand; T4 architecturally supported.

**HAL surface:** TimedRobot periodic loop, Notifier, FPGA timestamp,
DS protocol (enable/disable, alliance, match phase, joystick), CAN
bus transport, power monitoring (battery voltage, brownout state),
error/log surface (`HAL_SendError`). Out of v0: DIO, PWM, Analog,
Counter, SPI, I²C, USB, Relay, pneumatics. Encoders read CAN-side
only.

**Vendor device set:** Talon FX (Kraken X60) + CANcoder + Pigeon 2,
all Phoenix 6 at the latest stable firmware version pinned at v0
ship. MVP scope per device — surface needed for v0 demo, not full
firmware reproduction. Vendor sim APIs (CTRE Phoenix Sim) used as
research aid only, never as test oracle. v1 adds REV.

**v0 demo robot:** Single-DOF arm. One Kraken X60 + one CANcoder for
joint position, closed-loop position control with gravity. Pigeon 2
in the device set but not exercised by the demo (loads cleanly,
verifies bus enumeration). Validates motor model, closed-loop, sensor
feedback, mechanism dynamics with constant-torque load. Will be
validated against a physical bench arm when bench access is
available.

**Tooling deliverable:** `tools/rio-bench/` — a WPILib robot project
that runs on a physical RIO 2, measures HAL call costs (mean +
stddev + p99 + outlier rate per call class, across multiple
operating points: idle-bus, saturated-bus, multi-thread contention),
and emits a versioned fixture file at
`references/rio2-hal-costs/<wpilib-version>-<rio-firmware>.csv`.
Until real bench data exists, estimated values are used and tagged
`unvalidated`.

**v0 implementation is now fully unblocked.** All five v0-gating
foundational questions (OQ-1, OQ-2, OQ-3, OQ-4, OQ-5) are decided.

## Robot description format (decided 2026-04-28)

**Format: JSON**, our own schema with FRC concepts as first-class
citizens. Transpiled to MJCF at load time so MuJoCo can consume the
physics portion; FRC-binding fields are consumed by Layers 2 and 3.

- **Parser:** `nlohmann/json` (header-only, MIT licensed, clean
  API), added via CMake FetchContent.
- **Layout:** `descriptions/<robot-name>.json`. Parser code at
  `src/description/`.
- **Schema grows organically** with version-bumps, same pattern as
  the HAL protocol. v0 schema covers what v0 needs (links, joints,
  motors, sensors).
- **One file per robot.** Sub-system reuse and shared modules
  (e.g. a "swerve module" definition reused across robots) are a
  v1+ feature.
- **Loading pipeline:**
  ```
  JSON file
    → JSON parse
    → in-memory RobotDescription struct
    → split:
       - geometry/joints/inertias/actuators → MJCF generation → MuJoCo
       - CAN IDs / firmware / controller-to-joint maps → Layers 2 & 3
  ```
- **First-class FRC concepts** that don't fit cleanly into MJCF and
  belong in our format: motor model selectors, controller / vendor
  bindings (CAN ID + firmware version), controller-to-joint
  mappings, drivetrain selector (tank/mecanum/swerve), sensor
  bindings, follower/leader relationships, status frame schedule
  overrides.

v0 single-DOF arm description sketch (illustrative; final schema
defined in `src/description/schema.h` at implementation time):

```json
{
  "schema_version": 1,
  "name": "v0-arm",
  "links": [
    { "name": "arm",
      "mass_kg": 2.0, "length_m": 0.5, "inertia_kgm2": 0.166 }
  ],
  "joints": [
    { "name": "shoulder", "type": "revolute",
      "parent": "world", "child": "arm",
      "axis": [0, 1, 0],
      "limit_lower_rad": -1.5708, "limit_upper_rad": 1.5708,
      "viscous_friction_nm_per_rad_per_s": 0.1 }
  ],
  "motors": [
    { "name": "shoulder_motor",
      "motor_model": "kraken_x60",
      "controller": "talon_fx",
      "controller_can_id": 1,
      "controller_firmware_version": "<pinned-at-v0-ship>",
      "joint": "shoulder",
      "gear_ratio": 100.0 }
  ],
  "sensors": [
    { "name": "shoulder_encoder",
      "sensor_model": "cancoder",
      "controller_can_id": 2,
      "controller_firmware_version": "<pinned-at-v0-ship>",
      "joint": "shoulder" }
  ]
}
```

A future authoring GUI / CAD-import tool is tracked as OQ-11
(deferred until the schema stabilizes).

## Physics engine (decided 2026-04-28)

**Engine: MuJoCo.** Apache 2.0 licensed, deterministic by design,
best-in-class for the stiff actuated-mechanism dynamics that
dominate FRC simulation work (high gear ratios on motors driving
joints). Clean C API; we wrap it in C++ behind a Layer 5 adapter.

Architectural rules:

- **Layer 5 is engine-agnostic at the interface.** Layer 3 and
  Layer 4 import nothing from MuJoCo. Layer 5 exposes a stable
  interface — apply force / torque, read pose / velocity / contact,
  step simulation — and the MuJoCo binding lives entirely behind
  that interface in `world/engine/mujoco/`.
- **Federation deferred.** If v2+ game-piece manipulation reveals
  MuJoCo's contact model as a real bottleneck, we federate (MuJoCo
  for robot internal dynamics, a contact-strong engine like Bullet
  for game-piece interactions) through the same engine-agnostic
  interface. v0 and v1 single-engine.
- **Determinism discipline.** MuJoCo's deterministic mode is the
  default. FP-determinism rules from the C++ toolchain apply at
  the binding boundary.

Inner-loop rate target: physics 1 kHz default, configurable up to
10 kHz for cases where mechanism stiffness demands it. MuJoCo's
implicit integrator handles up to that range comfortably.

## Language and toolchain (decided 2026-04-28)

**Language:** C++ throughout. Sim Core, HAL shim, LD_PRELOAD libc
shim, user-code launcher — all C++. Standard: C++20 minimum, C++23
where compiler support permits.

**Build system:** CMake. Cross-compilation for tier 2 (ARM) uses
WPILib's ARM cross-compiler via a CMake toolchain file committed to
the repo. Dependency management: CMake `FetchContent` with pinned
versions.

**Compiler:** Clang primary, GCC in the CI matrix. Codebase must
build clean on both. `-Wall -Wextra -Wpedantic -Werror` baseline.

**Determinism rules** (compiler doesn't help us; discipline does):

- No `std::chrono::system_clock` or `steady_clock` in sim core.
  Time goes through the sim-time interface. `clang-tidy` rule
  enforces this.
- No `std::random_device`, no default-seeded RNGs. All RNG seeded
  from the project root seed.
- FP determinism: FMA disabled where cross-platform byte-identity
  matters; ops requiring guaranteed determinism live in a
  `det_math` namespace with explicit pragmas.

**Memory safety strategy** (no borrow checker; lean on tooling):

- Smart pointers everywhere (`std::unique_ptr`, `std::shared_ptr`).
  No raw `new`/`delete` except inside explicitly-marked arena
  allocators.
- RAII for every resource.
- Sanitizers (UBSan + ASan + TSan) green in CI before any merge.
- `clang-tidy` with `cppcoreguidelines-*`, `bugprone-*`, `misc-*`
  enabled.
- No exceptions in sim-core hot paths. `std::expected`-style Result
  types for fallible ops; exceptions only at process boundaries.

**Style:** project-local `.clang-format` and `.clang-tidy` enforced
in CI. Naming is `snake_case` (matches WPILib HAL C-ABI naming so
there's no flip at the boundary). `.h` + `.cpp` split; C++20
modules deferred. Pimpl idiom only at the HAL shim's external ABI
boundary where ABI stability matters.

**Standard libraries:**

- Testing: **GoogleTest + GoogleMock**. Parameterized tests
  (`TYPED_TEST`, `INSTANTIATE_TEST_SUITE_P`) are the natural fit
  for "test this motor model at N operating points."
- Diagnostic logging: **spdlog**, with compile-time level filtering
  in hot loops, async sinks for non-hot paths.
- State logging: **WPILOG** binary format (separate from spdlog),
  emitted directly so AdvantageScope works out of the box.
- Concurrency: plain `std::thread` + `std::atomic` + condition
  variables for v0. No Boost.Asio / coroutines yet.

**Protocol schema (HAL ↔ Sim Core):**

- Source of truth: a single header of hand-written POD C++ structs.
- T1 (shared memory): structs `mmap`'d directly into a shared
  region; sync via atomics + a seqlock or simple mutex.
- T2 (socket): same structs, transmitted via `memcpy` with pinned
  little-endian. We control both ends, so no wire-format library
  yet. FlatBuffers / Cap'n Proto can be swapped in later if needed.

**Polyglot edges (intentional, minimal):**

- `tools/rio-bench/` — Java (it's a WPILib robot project; this is
  fixed by what runs on a real RIO).
- User-code launcher (loads our HAL shim alongside the user's JVM
  or native process) — C++.
- Build / dev / lint scripts — Python or shell. Kept minimal.
