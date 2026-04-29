---
name: layer-3-device-bus-vendor-firmware
description: Use when designing or modifying the device bus (CAN/DIO/PWM/SPI/I²C transport models) or any vendor device firmware model — Talon FX (Phoenix 6), Spark Max/Flex (REVLib), CANcoder, Pigeon 2, pneumatics controllers, CANdle, etc. Covers bus arbitration, status frame timing, vendor closed-loop controllers, firmware versioning, and the device-physics interface to Layer 4.
---

# Layer 3 — Device bus & vendor firmware

Everything the robot talks to *through* a transport. CAN motor
controllers, encoders, IMUs, pneumatics, LEDs, and (with caveats)
cameras and vision co-processors.

## Scope

Layer 3 owns:

- **Transport models** with realistic arbitration and timing.
  CAN is the big one: realistic frame gaps, multi-bus topology
  (CAN0, CANivore CAN1+), arbitration when multiple devices try to
  send simultaneously, behavior under saturation.
- **Vendor device firmware models** — for each device, the API
  surface the vendor SDK sees, the closed-loop controllers running
  inside the device, status frame schedules, faults and limits,
  persistent config.
- **Device → physical model port** — the standardized hand-off to
  Layer 4 (commanded voltage, applied torque, expected feedback).

## Public contract

### Bus

The bus exposes:

- `register(transport, address, handler)` — a device subscribes.
- `send(frame)` — backend or device sends; bus arbitrates.
- `recv(handler) -> Stream<Frame>` — handler receives frames
  addressed to it.

Frames carry timestamps. Two devices sending simultaneously resolve
deterministically (lower CAN ID wins, like real hardware). Bus
saturation produces realistic delays — this is a real failure mode
on FRC robots and we reproduce it.

### Vendor device models

Each device in `vendors/<name>/<device>/` exposes:

- A SDK-side facade (what the user-side vendor library sees) —
  registers, CAN frames, etc., matching the real device.
- A physics-side port (what Layer 4 sees) — the device delivers a
  voltage to its motor model, receives back the rotor angle/velocity
  and current draw.
- A configurable status-frame schedule.
- Persistent config that survives reboot the way real devices do.

### Firmware versioning

Every vendor model carries a firmware version. The robot's config
selects which firmware version is being modeled. Behavior known to
differ across firmware versions is gated on this — we do not collapse
it into a single "vendor X" behavior. Each device's skill records
which firmware versions it covers.

## Initial device targets (priority order)

1. **Talon FX** (Kraken X60, Falcon 500) on Phoenix 6.
2. **NEO / NEO Vortex** on Spark Max / Spark Flex (REVLib).
3. **CANcoder / Pigeon 2** (Phoenix 6).
4. **REV Through Bore Encoder** (absolute and quadrature paths).
5. **Pneumatics**: REV PH, CTRE PH, PCM.
6. **CANdle / Spark addressable LEDs.**
7. **Cameras**: USB webcam shim, Limelight, PhotonVision (these
   are co-processors, not just devices — see notes below).

Each becomes its own skill (`vendors/talon-fx-phoenix-6.md` etc.) at
the time of implementation. This skill covers the layer-level
contracts only.

## Sub-systems

### Closed-loop controllers

Real devices run their own controllers internally:

- Phoenix 6 motion magic, position/velocity PIDF, FOC commutation.
- Spark Max / Spark Flex PIDF + smart-motion.
- CANcoder fused / not-fused source selection.

These are **not** textbook reference implementations. Phoenix's FOC
commutation produces a different effective `Kt` curve from a naive DC
machine model. We model the actual firmware behavior, validated
against vendor data and bench measurements.

Each device's closed-loop runs at the device's loop rate (often 1 kHz
internally, sometimes faster). The bus only sees status frames at
configurable rates; the inner controller is independent.

### Status frames

Configurable rates and contents per device. The status-frame
schedule contributes to bus load and must be modeled accurately
because:

- Latency of feedback depends on it (a controller updated at 10 Hz
  will look slow even if the inner loop is fast).
- Bus saturation when many devices broadcast at high rates is a
  real-robot failure mode.

### Faults and limits

- Stator / supply current limits (Phoenix 6 enforces these in
  firmware).
- Soft limits (forward/reverse position).
- Hard limits via wired switches (also Layer 2 / DIO).
- Fused encoder fault propagation.
- Over-temperature derate (links to Layer 4 thermal model).

### Persistent config

What survives a power cycle vs. a reboot, what's in flash vs. RAM,
config CRC (Phoenix), factory defaults — modeled per device.

## Cameras / vision co-processors

Limelight and PhotonVision are entire computers running their own
software. Treating them as a "device on the bus" understates them.
Each gets its own skill when implemented. The minimum:

- Field renderer (ground-truth pose → simulated camera frame).
- The co-processor's pipeline runs on the simulated frames (or
  receives ground-truth-plus-noise in stub mode).
- Detections published over NetworkTables / Ethernet.

Decision deferred to OQ-9.

## Hard parts and gotchas

- **Closed firmware.** Vendors don't publish source. Fidelity comes
  from datasheets, vendor sim hooks, and bench observation. Some
  quirks are only knowable empirically — bench validation is the
  only honest path to high-tier fidelity.
- **CAN arbitration under load.** Easy to fake "infinite bandwidth"
  CAN; doing so hides bugs. Must arbitrate frame-by-frame.
- **Versioning matters.** Phoenix 6 vs. 5 is a fork. Within Phoenix
  6, firmware revisions change behavior. Single "Phoenix model" is a
  trap.
- **Vendor SDK quirks.** Some calls are async; some block; some have
  retries; some silently swallow errors. Match the SDK behavior
  precisely or robot code will behave differently in sim.

## Validation

Per-device tests in `tests/vendors/<name>/<device>/`:

- Closed-loop step response matched against vendor-published curves
  (or bench data when available), to a stated tolerance.
- Status frame timing histograms matched against captured CAN
  traces.
- Bus arbitration tests with synthetic traffic at known load levels.

A device without validation is `unvalidated` and so is anything
downstream that depends on it.

## Files / paths (TBD)

To be filled when scaffold lands. Placeholder:

- `bus/can/` — CAN bus model.
- `bus/dio/`, `bus/pwm/`, … — other transports.
- `vendors/<name>/<device>/` — per-device model + test fixtures.

## v0 status (decided 2026-04-28)

**v0 device set:** Talon FX (Kraken X60) + CANcoder + Pigeon 2,
all on Phoenix 6, firmware version pinned to latest stable at v0
ship. User has physical access to all three for bench validation.

**Strategy:** MVP from day 1, *not* ride-and-migrate against vendor
sim APIs. CTRE Phoenix Sim is consulted as a research aid during
model development (cross-check our outputs vs. theirs on identical
inputs; investigate disagreements) but never appears in test
assertions. Tests pin against datasheet curves and bench
measurements with stated tolerances.

**MVP per-device surface for v0** — only what the v0 demo robot
actually exercises (single-DOF arm using Kraken + CANcoder; Pigeon
loaded but idle):

- Talon FX: applied output, basic config (current limits, neutral
  mode, inversion), one closed-loop mode (position), status frame
  scheduler at configurable rates. Anything not exercised by the
  arm demo is `not-yet-modeled`.
- CANcoder: absolute position, configurable status frame rate,
  fused-source semantics scoped to what Phoenix 6 actually does for
  the v0 firmware.
- Pigeon 2: enumeration on the bus + minimum status frames so
  vendor library load-time checks pass. Yaw integration deferred
  until a demo exercises it.

REV (Spark Max/Flex + NEO/NEO Vortex) lands in v1. Pneumatics, LEDs,
vision are later. Each new device gets its own skill at
implementation time, capturing constants, citations, firmware
version, and validation status.

## Cross-references

- `layer-2-control-system-backend.md` — sources the transport
  traffic.
- `layer-4-physical-models.md` — receives commanded voltage, returns
  rotor state and current draw.
- `docs/REFERENCES.md` — vendor docs and datasheets.
- `docs/OPEN_QUESTIONS.md` — OQ-6 (vendor fidelity strategy),
  OQ-8 (CAN topology fidelity), OQ-9 (vision pipeline).
