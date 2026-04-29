# Open Questions

Decisions not yet made. Each one is foundational — getting it wrong wastes
months — so we resolve them deliberately, with a writeup, before
implementation work that depends on them. Once resolved, the decision moves
to `ARCHITECTURE.md` and this file is updated.

Conventions:
- **OQ-N.** A numbered open question.
- **Status:** open / decided / deferred.
- **Owner:** who's responsible for driving the decision.
- **Blocks:** what work cannot start until this is decided.

## Status board

| OQ    | Question                          | Status   |
|-------|-----------------------------------|----------|
| OQ-1  | Robot code execution model        | decided  |
| OQ-2  | Physics engine                    | decided  |
| OQ-3  | Implementation language(s)        | decided  |
| OQ-4  | Robot description format          | decided  |
| OQ-5  | Determinism scope                 | decided  |
| OQ-6  | Vendor model fidelity strategy    | open     |
| OQ-7  | Visualizer                        | open     |
| OQ-8  | CAN bus topology fidelity         | open     |
| OQ-9  | Vision pipeline                   | deferred |
| OQ-10 | Test field for validation         | open     |
| OQ-11 | Authoring GUI / CAD integration   | deferred |

**v0 implementation is unblocked** — all v0-gating questions
(OQ-1–OQ-5) are decided. OQ-6/8/10 affect post-v0 work and can be
resolved as those layers come into focus. OQ-7/9/11 are explicitly
deferred. See `docs/V0_PLAN.md` for the implementation plan.

---

## OQ-1. Robot code execution model

**Status:** **decided** (2026-04-28). Binding resolution recorded in
`docs/ARCHITECTURE.md`. The original framing and sub-decision history
are preserved below for context.
**Blocks:** none — Layer 2 (control system backend) is now unblocked.

**The question.** How does the user's robot binary actually run?

**Options.**

A. **Native HAL replacement.** Build robot code for the host (linux-x86_64),
   link against our HAL implementation. This is what `wpilibj`'s sim mode
   does today. Fast iteration, easy debugging, cross-platform.
   *Cost:* misses RT scheduling fidelity; threading behavior differs from
   PREEMPT_RT; vendor JNIs need a sim variant; can't catch CPU-arch-
   dependent bugs.

B. **QEMU-emulated RoboRIO.** Run the actual NI RT image (or a
   RIO-compatible image) under QEMU emulating the Zynq Z-7020. Robot
   binary is the same `.deb` you'd deploy. Closest to reality.
   *Cost:* slow (probably 0.1×–0.3× wall clock without KVM, which we
   don't have for ARM-on-x86), tooling-heavy, hard to debug, image
   licensing questions for the NI distribution.

C. **Hybrid with fidelity dial.** Native by default for fast iteration;
   QEMU mode opt-in for fidelity validation. Backend interface designed so
   both modes look identical to Layer 3+.
   *Cost:* two backends to maintain; the cheap mode might be where bugs
   hide.

**Leaning.** C, with A as the day-1 deliverable. But this needs to be
decided before the backend interface is locked in, because the backend
process model differs (in-process JNI vs. separate QEMU process with a
network HAL).

**Existing evidence (B is feasible).** The user's prior project,
`WL-Richards/RIOEmulator`, implements option B end-to-end: boots NI
Linux RT under QEMU on emulated Zynq, replaces three NI shim libraries
(`libNiFpga.so`, `libRoboRIO_FRC_ChipObject.so`,
`libFRC_NetworkCommunication.so`) cross-compiled with WPILib's ARM
toolchain, runs unmodified robot code, drives DS + NetworkTables + a
50 Hz periodic loop via FPGA alarm IRQ 28. So feasibility of option B
is established. Cost confirmed: noticeable QEMU emulation overhead vs.
native, no determinism, single-vendor coverage in that prototype.

**To resolve:** prototype option A (native HAL replacement) at the same
"spin a motor 1 second" scope, measure latency/throughput, then decide
whether the C hybrid is worth the maintenance cost. If we go C, the
backend interface must be defined before either prototype solidifies.

### Sub-decisions made (2026-04-28)

These narrow the question; full closure pending sub-decisions 3–5.

**Sub-decision 1 — Architecture & boundary.**

- Native and QEMU are *siblings*, not competitors. Two implementations
  of one versioned, recorded HAL protocol. Layers 3–5 do not know which
  tier is loaded.
- Protocol shape: state-mirror with tick-aligned sync, plus on-demand
  sync escape hatch for HAL calls that genuinely need synchronous
  round-trip (identified per-call as we go).
- Schema grown organically, version-bumped per release. v0 schema
  covers v0 scope only.
- The sync layer is the recording boundary; replay and determinism
  fall out of recording every sync exchange.

**Sub-decision 2 — Tier hierarchy & v0 deliverable.**

Four-tier model:

- **Tier 1 — native.** Host x86 JVM + x86 HAL. Daily development
  driver. Loop-overrun fidelity via a RIO cost table + cgroups/CPU
  throttling. ARM ISA fidelity: none.
- **Tier 2 — user-mode QEMU.** Robot binary, JVM, WPILib native, and
  vendor libs run as ARM under `qemu-user`. No kernel emulation, no
  NI Linux RT boot. HAL shim is ARM, intercepts via LD_PRELOAD,
  speaks the same protocol to our x86 sim core. Default fidelity
  validation tier (cheap enough to run on every test).
- **Tier 3 — full-system QEMU.** Adds NI Linux RT + PREEMPT_RT
  scheduling on top of tier 2. RIOEmulator-style. Reserved for
  RT-scheduling-specific bugs and competition-gate validation.
- **Tier 4 — HIL.** Real RIO over network, far future.

**v0 ships tiers 1 and 2.** Tier 3 added on-demand. Tier 4 supported
architecturally (same protocol) but not built.

**Additional v0 deliverable: `tools/rio-bench/`.** A WPILib robot
project deployed to a physical RIO 2 that measures HAL call costs
(mean + stddev + p99 + outlier rate per call class, across multiple
operating points: idle-bus, saturated-bus, multi-thread contention).
Output is a versioned fixture file at
`references/rio2-hal-costs/<wpilib-version>-<rio-firmware>.csv`,
loaded at sim startup. **Until real bench data is available**,
estimated values are used (sourced from RIO 2 spec + reasonable
extrapolation), with the file tagged `unvalidated`. User has stated
real-RIO access will be available later.

**Sub-decision 3 — Vendor library strategy.**

- Build MVP vendor models from day 1, not ride-and-migrate. Vendor
  sim APIs (CTRE Phoenix Sim, REVLib sim) used as a *research aid*
  during model development, never as a test oracle.
- v0 device set: **Talon FX (Kraken X60) + CANcoder + Pigeon 2**, all
  on Phoenix 6. User has physical access to all three for bench
  validation.
- Phoenix 6 firmware version pinned to latest stable at project
  start (recorded with the model when implemented). Each device's
  skill captures the firmware version it covers; behavior known to
  differ across firmware versions is gated explicitly.
- v1 adds REV (Spark Max/Flex + NEO/NEO Vortex). Pneumatics, LEDs,
  vision are later.
- Tests pin against datasheet-cited curves and bench-measured
  behavior with stated tolerances. Vendor sim does not appear in
  test assertions.

**Sub-decision 4 — Time discipline.**

- **Hard rule:** sim core owns the clock; nothing else has one.
  Backend receives "advance to T+dt" and reports state changes.
- All time-querying entry points are intercepted in the backend, not
  just HAL. Tier 1 uses an LD_PRELOAD libc shim covering
  `clock_gettime`, `gettimeofday`, `nanosleep`, `clock_nanosleep`,
  etc.; JVM intrinsics (`System.nanoTime`, `System.currentTimeMillis`)
  bottom out in these syscalls and so see sim time too. Tier 2 uses
  the same mechanism inside qemu-user.
- Two run modes:
  - **Deterministic** (default for tests/CI): no wall-clock anywhere;
    DS disabled or replayed from recorded trace; RNG seeded from a
    fixed root; sim time advances as fast as compute allows.
  - **Soft real-time** (human-in-the-loop): sim time advances at ~1×
    wall-clock with feedback control; real DS connected;
    non-deterministic by construction.
- Default `java.util.Random` seeds from `System.nanoTime()`, so
  controlling time controls Random. `SecureRandom` deferred to v1+.
- JVM internals (GC trigger heuristics, JIT compilation timing) are
  not bit-deterministic even with sim time; v0 accepts this. Higher-
  determinism modes (pinned GC settings, AOT) come post-v0 if needed.

**Sub-decision 5 — v0 HAL surface and demo target.**

- v0 HAL surface, in scope: TimedRobot periodic loop, Notifier, FPGA
  timestamp, Driver Station protocol (enable/disable, alliance,
  match phase, joystick), CAN bus transport, power monitoring
  (battery voltage, brownout state), error/log surface
  (`HAL_SendError`).
- Out of scope until post-v0 demand: DIO, PWM, Analog, Counter, SPI,
  I²C, USB, Relay, Solenoids/pneumatics. Encoder reading in v0 is
  CAN-side only (Talon FX rotor feedback + CANcoder); no DIO
  quadrature.
- **v0 demo robot: single-DOF arm.** One Kraken X60 + one CANcoder
  for joint position, closed-loop position control with gravity.
  Pigeon 2 in v0 device set but not exercised by the demo (verifies
  it loads cleanly). Validates motor model, closed-loop, sensor
  feedback, mechanism dynamics with constant-torque load. Will be
  validated against a physical bench arm when bench access is
  available.
- Post-v0 the HAL surface grows on demand; each expansion bumps
  the protocol version per sub-decision 1.

---

## OQ-2. Physics engine

**Status:** **decided** (2026-04-28). Binding resolution recorded in
`docs/ARCHITECTURE.md` "Physics engine" section.
**Blocks:** none — Layer 5 is now unblocked.

**The question.** Which 3D rigid-body physics engine?

**Options.**

A. **Bullet.** Battle-tested, used by Gazebo. Mature contact handling.
   Older API, less great with stiff systems.

B. **Rapier.** Modern, written in Rust, deterministic across platforms
   (a big plus for replay). Less battle-tested for high-DOF articulated
   mechanisms.

C. **MuJoCo.** Best-in-class accuracy for actuated mechanisms (built for
   robotics research). Excellent stiff-system integration. Less great
   with arbitrary contact and game-piece manipulation.

D. **PhysX.** Excellent contact, used in Isaac Sim. Closed-source-ish,
   GPU-leaning, less embeddable.

E. **Custom.** Don't.

**Trade-off.** FRC has *both* high-DOF stiff actuated mechanisms (where
MuJoCo wins) *and* lots of complex contact (game pieces in intakes,
robots colliding) where MuJoCo is weaker. We may end up using MuJoCo for
chassis+mechanism dynamics and a separate, cheaper engine for game-piece
contact, federated through the world state. That's complex but might be
the only honest answer.

**Leaning.** Investigate MuJoCo first; fall back to Rapier if MuJoCo's
contact model can't handle game pieces well enough.

**To resolve:** prototype a swerve module on carpet and a game-piece-in-
intake test in each candidate; compare against video of real bots.

### Resolution (2026-04-28)

**Picked: MuJoCo** (option C). Apache 2.0 licensed (DeepMind, 2022+),
deterministic by design, best-in-class for stiff actuated mechanism
dynamics — which is most of what FRC sims model. Clean C API,
straightforward to wrap from C++.

Constraints captured:

- **Layer 5 stays engine-agnostic.** No MuJoCo headers in Layers 3 or
  4. Layer 5 exposes a stable interface (apply force/torque, read
  pose/velocity, query contacts) and the MuJoCo binding lives behind
  it.
- **Federation deferred to v2+.** If game-piece manipulation in v2+
  exposes MuJoCo's contact model as a real wall, we federate
  (MuJoCo for robot internal dynamics, Bullet or PhysX for
  game-piece contact) through Layer 5's engine-agnostic interface.
  Not a v0/v1 concern.
- **OQ-4 (robot description format) not auto-decided.** MJCF is
  attractive because it's MuJoCo-native, but FRC has concepts
  (CAN port bindings, vendor firmware versions, controller-to-joint
  mappings) that don't fit MJCF cleanly. Decided separately in OQ-4.

User has familiarity with Chaos / Jolt / ODE / Project Chrono.
MuJoCo learning curve is real but well-trodden in the robotics
community.

---

## OQ-3. Implementation language(s)

**Status:** **decided** (2026-04-28). Binding resolution recorded in
`docs/ARCHITECTURE.md` "Language and toolchain" section.
**Blocks:** none — Sim Core skeleton is now unblocked.

**Constraint.** HAL shim has to interop with WPILib native code (C++) and
JNI (Java). That part is C/C++.

**Options for Sim Core.**

A. **All C++.** One language; clean native interop with HAL. Painful for
   correctness work; slower to iterate.

B. **Rust core, C++ HAL shim.** Rust gives us memory safety and great
   determinism stories; FFI to C++ HAL shim is well-trodden. More moving
   parts.

C. **Rust everywhere, including HAL shim via cbindgen.** Cleanest Rust
   story; HAL shim becomes just a generated C header over a Rust
   implementation. Risk: WPILib HAL has nontrivial C semantics that may
   not map cleanly.

**Leaning.** B. Rust for sim core, motor models, physics adapters,
device bus, RT scheduler simulation. C++ for the HAL shim that talks to
WPILib. Python for tooling/scripts only.

**To resolve:** confirm that a Rust ↔ C++ HAL shim doesn't add
unacceptable jitter at 1 kHz tick rate.

### Resolution (2026-04-28)

**Picked: option A — all C++.** User is fluent in C++, multi-year
project, language familiarity is a real constraint that outweighs
the abstract memory-safety argument for Rust. Binding follow-on
choices captured in `docs/ARCHITECTURE.md` "Language and toolchain":

- C++20 minimum, C++23 where compilers support it.
- Build system: CMake.
- Compiler: Clang primary, GCC in CI matrix.
- Sanitizers (UBSan + ASan + TSan) and `clang-tidy` gating CI from
  day 1 — this is how we enforce the "no shortcuts" non-negotiable
  at the toolchain level, since C++ doesn't enforce it at the
  language level.
- Error handling: `std::expected`-style (Result types) in sim-core
  hot paths; exceptions only at process boundaries.
- Determinism rules: no `std::chrono::system_clock` /
  `steady_clock` in sim core; no `std::random_device` /
  default-seeded RNG; FP-determinism discipline (FMA disabled where
  cross-platform byte-identity matters; small `det_math` namespace
  for ops requiring it).
- Memory: smart pointers everywhere; no raw `new`/`delete` outside
  arena allocators; RAII enforced.
- Style: project `.clang-format` + `.clang-tidy` enforced in CI;
  `snake_case` to match WPILib HAL C-ABI naming at the boundary.
- Testing: GoogleTest + GoogleMock.
- Logging: spdlog for diagnostic; WPILOG for state stream.
- Deps: CMake FetchContent with pinned versions.
- IPC schema: hand-written POD C++ structs in a canonical header;
  T1 places them directly in shared memory; T2 transmits via
  `memcpy` over socket with pinned little-endian.
- Cross-compilation for tier 2 ARM via WPILib's ARM cross-compiler
  toolchain (the same one used in RIOEmulator).
- Polyglot edges: `tools/rio-bench/` is Java (must run on a RIO);
  user-code launcher is C++; build/dev scripts are Python or
  shell, kept minimal.

---

## OQ-4. Robot description format

**Status:** **decided** (2026-04-28). Binding resolution recorded in
`docs/ARCHITECTURE.md` "Robot description format" section.
**Blocks:** none — robot loading is now unblocked.

**Options.**

A. **URDF + FRC extensions.** ROS standard, lots of tooling, lots of
   pain (XML, no native swerve concept, no native motor-controller
   binding).

B. **MuJoCo MJCF.** If we use MuJoCo, this is its native format and very
   capable. Less general-purpose tooling.

C. **SDF (Gazebo).** Better than URDF in most ways, less ubiquitous.

D. **Custom.** TOML/YAML schema purpose-built for FRC: chassis kinematics
   selectors (tank/mecanum/swerve), motor port bindings, sensor
   bindings, mechanism trees.

**Leaning.** D, with import/export to URDF for visualization tooling
compatibility. FRC has too many specifics (port bindings, vendor IDs,
follower relationships) to ride on a generic format cleanly.

### Resolution (2026-04-28)

**Picked: option D — our own format, transpile to MJCF at load time.**
Format is **JSON**. Path (a) (MJCF + sidecar) was the runner-up; (c)
won because v1+ FRC concepts (swerve module declarations, follower-
leader relationships, vendor firmware versions) are first-class
citizens in our format and only generated artifacts in MJCF.

- **Format: JSON.** Parser: nlohmann/json (header-only, MIT, clean
  API), added to CMake FetchContent.
- **Schema grown organically with version-bumps**, same pattern as
  the HAL protocol from OQ-1. v0 schema covers the v0 arm.
- **One file per robot.** Sub-system reuse (a "shared swerve
  module" file) is a v1+ feature.
- **Layout:** `descriptions/<robot-name>.json`, parser at
  `src/description/`.
- **Loading pipeline:** JSON → in-memory `RobotDescription` struct
  → split into MJCF (geometry/joints/inertias/actuators handed to
  MuJoCo) and FRC-binding tables (CAN IDs / firmware versions /
  controller-to-joint maps consumed by Layers 2 and 3).

---

## OQ-5. Determinism scope

**Status:** **decided** (2026-04-28) as a side-effect of OQ-1
sub-decision 4 (time discipline). Resolution recorded in
`docs/ARCHITECTURE.md` "Time" and "Determinism" sections.

**Resolution summary.** Deterministic mode is "recorded inputs
replayed back" — there is no separate "determinism off with no replay"
mode. Soft real-time mode (with a real DS) is the only
non-deterministic mode. Replay is byte-identical; no floating-point-
reorder tolerance.

---

## OQ-6. Vendor model fidelity strategy

**Status:** open.
**Blocks:** vendor model implementations.

**Tiers, weakest to strongest.**

1. Match the public SDK API only — closed-loop math is "close enough."
2. Match SDK + status frame timings + known firmware constants.
3. Match SDK + frames + firmware controller numerics, validated against
   bench data.
4. Match all of the above + known firmware quirks (bug-for-bug).

**Question.** Where do we draw the line per device, and how do we get
data for tier 3+?

**Options for getting data.**
- Vendor sim APIs (CTRE Phoenix Sim, REVLib sim) — give us *some* signal
  but are themselves abstractions, not ground truth.
- Bench observation: instrument a real device, log register reads/writes
  + mechanical response, replay against our model.
- Datasheet + reverse engineering (legal grey area; needs a careful
  policy).

**Leaning.** Tier 3 minimum for any device used in match-critical paths
(drive motors, primary mechanism actuators). Bench observation is the
only honest path to tier 3+.

---

## OQ-7. Visualizer

**Status:** open. Lower priority — can defer.

**Options.**
A. AdvantageScope-only (it's already the de-facto FRC viewer; ride on it).
B. Custom 3D viewer (Bevy, three.js, Unity) for richer interaction.
C. Both: ship logs that work in AdvantageScope; build our own viewer for
   sim-specific things AdvantageScope can't show (e.g. game-piece flow,
   contact forces).

**Leaning.** C, but A is enough for v0.

---

## OQ-8. CAN bus topology fidelity

**Status:** open.
**Blocks:** device bus implementation.

**The question.** Do we model multiple CAN buses (RIO has CAN0; CANivore
adds CAN1+) explicitly with separate arbitration domains? How accurate
does CAN need to be — frame-by-frame with bit-stuffing latency, or
"frames arrive in order with realistic jitter"?

**Leaning.** Frame-by-frame with realistic gap and arbitration; ignore
bit-stuffing details unless validation reveals a need. Multiple buses
must be separate domains from day 1.

---

## OQ-9. Vision pipeline

**Status:** deferred until Layers 1–4 are working.

**The question.** How do we simulate a Limelight / PhotonVision
co-processor? Render the field, feed frames to the user's pipeline, get
detections back? Or stub the detections entirely with ground-truth-plus-
noise?

**Leaning.** Both modes. Stub-with-noise for fast iteration; full
rendered pipeline for validation.

---

## OQ-11. Authoring GUI / CAD integration

**Status:** **deferred** (post-v0). Tracked here so it isn't lost.

**Goal.** A GUI for authoring robot descriptions (the JSON files
decided in OQ-4), with CAD import for geometry and inertia
computation, and visual wiring for joints, motors, sensors, and CAN
bus topology. Likely also live-monitoring during simulation, though
that's a distinct surface from authoring.

**Why deferred.** v0 hand-writes one JSON file (the single-DOF arm).
v1+ may have multiple robot descriptions, at which point an
authoring tool starts paying for itself. Building the GUI before
the JSON schema is stable would be premature.

**Re-open trigger.** When (a) the description schema has stabilized
through several real robots, and (b) a contributor or user is
hitting friction hand-authoring JSON. At that point we make this
a real OQ with options (web UI, native Qt, integrate with existing
FRC tools, etc.) and a target.

## OQ-10. Test field for validation

**Status:** open. Practical question.

We need bench measurements for motor curves, sensor noise, mechanism
behavior. Do we:

A. Buy/build an instrumented test bench.
B. Partner with a team that has hardware and run instrumented sessions.
C. Rely entirely on vendor-published data.

**Leaning.** Mostly C for v0, with at least one motor + one swerve
module instrumented (A or B) by the time we claim "validated."
