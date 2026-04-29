---
name: layer-2-control-system-backend
description: Use when designing, implementing, or modifying the control-system backend (the runtime that the robot code thinks it's running on). Currently RoboRIO 2; future System Core and others must plug in cleanly. Covers HAL surface, RT scheduler, transports (CAN/DIO/PWM/SPI/I²C/Ethernet), Driver Station endpoint, and the backend interface itself.
---

# Layer 2 — Control system backend

The runtime that the user's robot code thinks it's running on. RoboRIO 2
is the first backend. The architecture must support adding System Core
and future hardware as additional backends without changing Layers 3+.

## Scope

The backend owns:

- **HAL surface** matching the targeted WPILib year (function-for-function).
- **RT scheduler / OS surface** — threads, sleeps, notifiers,
  interrupts, semaphores, FPGA timestamp.
- **Transports** — CAN, PWM, DIO, SPI, I²C, USB, Ethernet — all
  routing to / from the device bus.
- **Driver Station endpoint** — the FRC DS protocol, including
  enable/disable, alliance, match time, joystick state.
- **Power surface** — what the FPGA reports about battery voltage and
  brownout. The actual electrical state lives in Layer 4; the backend
  *publishes* it to robot code through HAL calls.

The backend does *not* own:

- Vendor device behavior (Layer 3).
- Motor or sensor physics (Layer 4).
- Robot or world dynamics (Layer 5).

## Public contract

The Sim Core invokes the backend through a stable interface:

- `init(robot_description, config) -> Backend`
- `boot()` — load and start the user's robot binary
- `step(dt)` — advance one tick of the backend's RT scheduler
- `device_bus()` — handle to transports
- `ds_input(packet)` / `ds_output() -> packet`
- `shutdown()`

The Sim Core never knows which backend is loaded. Switching from
RoboRIO 2 to System Core changes only the backend binary and config.

## Sub-systems

### HAL implementation

Implement the WPILib HAL contract (`hal/include/hal/*.h`) for the
pinned year. Most calls are state reads/writes; a non-trivial
minority — notifiers, interrupts, DMA, FPGA-side counters — need
careful timing modeling.

The HAL is **the contract we test against**. Tests pin behavior to
WPILib HAL semantics, not to our internal data structures. A test
that passes against our backend must also pass against any other
correct WPILib HAL implementation.

### RT scheduler / OS surface

Robot code uses:

- `Notifier` for fast loops (tied to FPGA alarms on real hardware).
- `Thread.sleep`, semaphores, locks.
- Periodic loop tick at 50 Hz (`TimedRobot`).
- Sometimes raw POSIX threads.

On a real RIO this is NI Linux RT with PREEMPT_RT. We need to model
scheduling latency, jitter, and priority well enough that race
conditions in user code reproduce. `[OPEN]` (OQ-1) — native scheduler
under sim time vs. emulated RT kernel under QEMU. Decision affects
*everything* in this sub-system.

### Transports

- **CAN** — frames out of the FPGA controller, through the bus
  (Layer 3) to vendor models. Multi-bus topology (CAN0 + CANivore
  CAN1+) is supported from day 1.
- **PWM** — output signals to motor controllers wired to PWM
  (Talon SR, Spark, Victor SP, etc.). Simple but timing matters.
- **DIO** — digital in/out, with interrupts and quadrature
  decoding (the latter is FPGA-side on real hardware).
- **SPI / I²C** — typical for IMUs, color sensors, LIDARs.
- **USB** — webcams (cscore), HID, occasional joystick.
- **Ethernet** — DS uplink, NetworkTables, vendor Ethernet devices
  (Limelight, PhotonVision co-processors, CANivore over Ethernet).

### Driver Station endpoint

UDP packet protocol matching the official DS. The backend exposes
this to the Sim Core, which can either:

- Forward to a real DS (for human-in-the-loop driving), or
- Synthesize from a recorded input trace (for deterministic replay
  and CI).

### Power surface

The FPGA on a real RIO publishes battery voltage, total current, and
brownout state. Our backend reads these from Layer 4's electrical
model and publishes them through HAL.

## Backend agnosticism

The interface above does not assume RoboRIO. When System Core lands:

- Different HAL surface — backend provides its own.
- Different transports — backend declares which transports it
  supports; vendor models on Layer 3 query that.
- Different RT model — backend hides it behind the same `step(dt)`
  interface.

The Sim Core, vendor models, physics, and world all stay the same.

## Carrying forward from prior work

The user has a prior project, `WL-Richards/RIOEmulator`, that booted
NI Linux RT under QEMU with replaced shim libraries
(`libNiFpga.so`, `libRoboRIO_FRC_ChipObject.so`,
`libFRC_NetworkCommunication.so`). It demonstrated that approach is
viable end-to-end (DS, NT, 50 Hz periodic via FPGA alarm IRQ 28),
which is real evidence for OQ-1's option B.

What we should reuse, conceptually:

- **File decomposition**: one shim file per FPGA subsystem
  (`tDIO_impl.cpp`, `tPWM_impl.cpp`, `tAlarm_impl.cpp`, …) — clean
  pattern, language-agnostic.
- **CAN router pattern**: decode 29-bit FRC arbitration ID, dispatch
  to per-device handlers. Fits cleanly with our device bus design.
- **Register-backing-store** abstraction for fake registers — testable.

What we explicitly do **not** carry forward:

- Hardcoded year/firmware in namespaces (`nFRC_2024_24_0_0`).
- Single-vendor coverage (TalonFX-only).
- Toy physics inside a stub (`talonfx_stub.c`).
- Host-bridged DS without record/replay (kills determinism).

These map to specific non-negotiables we won't violate again.

## Hard parts and gotchas

- **Notifier accuracy.** Lots of robot code uses `Notifier` for fast
  loops; jitter must match real hardware or controllers tuned in
  sim won't tune the same on a real bot.
- **FPGA-side behaviors** (DMA-driven encoder reads, hardware
  counters, PWM generation, alarm IRQs) bypass the CPU. We must
  expose them through HAL with the same observable timing.
- **HIL-readiness.** Designing the backend boundary so a real RIO
  can replace this process for hardware-in-the-loop testing is
  cheap *now* and costly later.
- **DS as a determinism boundary.** Live DS introduces wall-clock
  nondeterminism. Must be either off (replay mode), recorded, or
  explicitly marked nondeterministic.

## Validation

For each HAL function we implement, a contract test in
`tests/hal/<group>/`:

- Drives the HAL through the WPILib API.
- Asserts on observable outputs (return values, timing, side
  effects on transports).
- Independent of internal implementation — would pass against
  another correct backend.

For RT behavior:

- Notifier scheduling latency and jitter measured under load,
  compared against real-RIO baselines once we have them.

## v0 status (decided 2026-04-28; see `docs/ARCHITECTURE.md`)

OQ-1 is closed. v0 ships **Tier 1 (native)** and **Tier 2 (user-mode
QEMU)**, both implementing the same HAL boundary protocol. T3
(full-system QEMU) is on demand; T4 (HIL) is supported by the
protocol but not built.

The HAL boundary is a **state-mirror protocol with tick-aligned
sync** plus on-demand sync escape. Read/write hit a local cache in
the HAL shim; sync exchanges deltas with the Sim Core at tick
boundaries (and on demand). Every sync is recorded — replay falls
out automatically. Schema grows organically; protocol version bumps
per release.

Time discipline: Sim Core owns the clock. The HAL shim ships an
LD_PRELOAD libc shim intercepting `clock_gettime`, `gettimeofday`,
`nanosleep`, `clock_nanosleep`, etc. — JVM intrinsics route through
these and so see sim time. Two run modes: deterministic (default for
tests/CI) and soft real-time (HITL).

v0 HAL surface: TimedRobot periodic, Notifier, FPGA timestamp, DS
protocol, CAN bus, power monitoring, error/log. Out of v0: DIO, PWM,
Analog, Counter, SPI, I²C, USB, Relay, pneumatics.

**v0 deliverable: `tools/rio-bench/`** — WPILib robot project
deployed to a physical RIO 2. Measures HAL call costs (mean + stddev
+ p99 + outlier rate per call class; multiple operating points:
idle-bus, saturated-bus, multi-thread contention). Output:
`references/rio2-hal-costs/<wpilib-version>-<rio-firmware>.csv`.
Native tier loads it at startup for loop-overrun fidelity. Estimated
values until real-RIO access is available; user has stated access
will be available later.

## Cross-references

- `layer-1-robot-code.md` — what loads on top of us.
- `layer-3-device-bus-vendor-firmware.md` — what we route to.
- `layer-4-physical-models.md` — where battery / brownout state
  comes from.
- `docs/ARCHITECTURE.md` — process model, HAL boundary protocol,
  time discipline, v0 scope.
- `docs/REFERENCES.md` — frcture FPGA register reference, DS
  protocol notes.
