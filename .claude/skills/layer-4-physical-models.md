---
name: layer-4-physical-models
description: Use when designing or modifying any physical model — battery, brownout, PDP/PDH, motor models (Kraken X60, Falcon 500, NEO, NEO Vortex, NEO 550, Minion, 775pro, CIM, etc.), gearboxes, sensor models (encoders, gyros, limit switches, TOF, color sensors), or wire harness electrical model. This is the layer where "almost identical to real life" is earned or lost — accuracy, citations, and validation tolerances matter most here.
---

# Layer 4 — Physical models

The continuous-time physics of motors, sensors, and the electrical
system. **This is the layer where the project's "no shortcuts" promise
gets earned.** Tests in this layer cite real sources, state tolerances,
and validate against published curves or bench measurements.

## Scope

Three sub-systems:

- **Electrical** — battery, PDP/PDH, wire harness resistance,
  brownout state machine.
- **Motors** — DC machine models for every motor type used in FRC,
  with gearboxes and thermal models.
- **Sensors** — quantization, noise, latency, bias, drift.

## Public contract

### Motor model interface

```
trait MotorModel:
  step(dt, applied_voltage_v, load_torque_nm) -> MotorState
  // returns rotor angle/velocity, current draw, temperature
```

Layer 3 (vendor device) supplies the applied voltage; Layer 5 (world)
supplies the load torque (through the joint connection); the motor
returns its electrical and mechanical state. Vendor closed-loop
controllers drive applied voltage on top of this model — they do not
*replace* the model.

### Battery / electrical interface

The battery sees aggregate current from all consumers; voltage sags
according to its model; the resulting bus voltage is what every
device actually sees. Brownout below threshold (typically 6.8 V)
cascades into HAL and disables outputs.

### Sensor interface

Sensors observe the underlying physical state and return a noisy,
quantized, latent measurement.

## Sub-systems

### Electrical

- **Battery.** 12 V nominal, modeled as Thévenin equivalent: open-
  circuit voltage as a function of state of charge, internal
  resistance. Both vary with SoC and temperature. Sustained current
  drains SoC.
- **PDP / PDH.** Per-channel current sensing with realistic noise
  and quantization, breaker trip curves (40 A, 30 A, 20 A — different
  trip times at different currents), total-current measurement.
- **Wire harness.** Lumped resistance per branch — small but
  non-zero. This is what couples motor stalls to brownout: a stalled
  Kraken pulls 366 A, drops the bus voltage through both battery
  internal resistance and harness resistance, and triggers brownout.
- **Brownout state machine.** Matches what the FPGA exposes to robot
  code — input voltage thresholds, time hysteresis, automatic motor
  disable, recovery.

### Motors

For each motor (Kraken X60, Falcon 500, NEO, NEO Vortex, NEO 550,
Minion, Bag, 775pro, CIM, miniCIM, RS-550):

- **DC machine equations** with manufacturer constants:
  - `V = I·R + L·dI/dt + Kv·ω`
  - `τ = Kt·I`
  - Stall torque, free speed, stall current, free current matched
    to published curves.
- **Gearbox** — ratio, efficiency curve (often not flat), output
  inertia. Backlash modeling is `[OPEN]`.
- **Thermal** — I²R heating, ambient cooling, thermal time constant,
  derating at high temperature, fault propagation back to the device
  model.
- **Brushed vs. brushless** — commutation losses on brushed; FOC
  commutation on Phoenix 6 brushless changes the effective `Kt`
  curve.

**No invented constants.** Every constant must cite a datasheet,
vendor curve, or bench measurement. Constants live in code with a
comment naming the source. Each motor type gets its own skill at
implementation time, capturing the citations.

### Sensors

- **Quadrature encoders** — counts per revolution, A/B/I lines, max
  edge rate, behavior near the rate limit (missed counts), index
  pulse handling.
- **Absolute encoders** — bit resolution, sample latency, wraparound,
  multiturn vs. single-turn.
- **CANcoder** — Phoenix 6 fused-source semantics included.
- **Pigeon 2 / NavX** — bias drift, noise spectrum, temperature
  dependence, integration drift over a match.
- **Limit switches** — debounce, polarity, disconnect handling.
- **TOF / photoelectric / color sensors** — response curves, FOV,
  noise, ambient-light effects.

## Hard parts and gotchas

- **Real motor curves don't match textbook DC machine equations
  exactly.** Phoenix 6 FOC commutation alters the effective torque
  constant. We model what the device *actually does*, not the
  idealized model.
- **Coupled brownout.** Motor stalls draw hundreds of amps, sag the
  battery, drop the logic rail, the RIO browns out, all motors
  disable, system recovers. The whole loop must close in sim. This
  is *the* test that proves the electrical and motor models are
  wired correctly.
- **Encoder edge cases.** Quantization at low velocity (you can't
  resolve 0.1 rad/s with a 1024 CPR encoder reading every 20 ms)
  and missed counts at high velocity are both common sources of
  real-robot bugs. Both must reproduce.
- **Stiffness.** High-gear-ratio mechanisms (50:1 elevator, swerve
  module steering) are stiff systems. Naive integrators explode;
  see solver notes in Layer 5.
- **Numerical units.** Mixing rad and rpm somewhere is the most
  predictable bug here. SI-only internally, conversions at the
  boundary, suffix every variable. See `code-style.md`.

## Validation

Validation is a hard requirement, not nice-to-have:

- For each motor model: a fixture file with the manufacturer's
  published torque/speed/current curve, plus a sim test that
  reproduces the curve to a stated tolerance (say ±5% — to be
  justified per motor when implemented).
- For each sensor model: noise/bias/latency characterization
  matched against datasheet specs or bench data.
- For the coupled electrical-motor brownout test: a stall-and-
  recover scenario with pinned voltage and current waveforms.

Models without validation are marked `unvalidated`. Anything
downstream inherits the taint. We do not silently ship inaccurate
models.

## Files / paths (TBD)

Placeholder:

- `models/electrical/` — battery, PDP, harness, brownout.
- `models/motors/<name>/` — per motor type, with fixture data.
- `models/sensors/<type>/` — per sensor.

Each motor and sensor gets its own skill at implementation time
(`models/kraken-x60.md`, etc.), capturing constants and citations
in detail.

## v0 status (decided 2026-04-28)

**Motor:** Kraken X60 (Talon FX). Single motor type for v0. Other
motors land as later layer skills.

**Sensors:** CANcoder (absolute, CAN), plus the Talon FX's built-in
rotor encoder (for closed-loop feedback through Phoenix 6). Pigeon 2
loaded on the bus but unused by the v0 arm demo. No DIO/quadrature
encoders in v0 — encoders are CAN-side only.

**Electrical:** battery + brownout state machine + power monitoring
required because the HAL surface includes power monitoring. PDH
channel-level current sensing is *not* in the v0 HAL surface, so the
electrical model needs to expose battery voltage and total current
to the HAL but per-channel breakdown is post-v0.

**Validation:** estimates allowed for v0 with `unvalidated` tags
until bench measurements come in. Real-RIO + real-device access is
expected. Each constant cited in code with the source it came from
(datasheet, vendor curve, estimate-with-rationale).

## Cross-references

- `layer-3-device-bus-vendor-firmware.md` — drives applied voltage
  via vendor closed-loop, reads sensor state, propagates faults.
- `layer-5-world-physics.md` — receives joint torques from motors,
  applies load to motors via reaction.
- `docs/REFERENCES.md` — datasheets, motor curves.
- `docs/OPEN_QUESTIONS.md` — OQ-10 (test field for validation).
