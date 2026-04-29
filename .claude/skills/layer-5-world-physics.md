---
name: layer-5-world-physics
description: Use when designing or modifying the rigid-body world simulation — chassis dynamics (tank/mecanum/swerve), articulated mechanisms (arms, elevators, turrets, intakes), game pieces, field elements, contact and friction, the per-season game logic plugin, and the robot-description format. Covers solver choice, integrator stability for stiff systems, contact models for game-piece manipulation, and field-render integration with Layer 3 vision.
---

# Layer 5 — World physics & field

Rigid-body simulation of the robot, mechanisms, game pieces, and field.
Bridges the gap between joint torques (from Layer 4 motors) and
observable robot behavior (pose, velocity, contact, scoring).

## Scope

- **Drivetrain** — tank, mecanum, swerve. Wheel-ground contact, slip,
  scrub, carpet vs. tile vs. diamond plate.
- **Articulated mechanisms** — arms, elevators, turrets, intakes,
  shooters. Joints with limits, friction, optional backlash.
- **Game pieces** — rigid bodies. Robot-piece contact and grip
  (intakes that depend on friction with a spinning roller).
- **Field** — floor, walls, scoring locations, AprilTags, lighting
  for vision.
- **Game logic plugin** — per-season scoring rules, match phases,
  penalties, link to FMS messages.
- **Robot description** — declarative file describing one team's
  robot (geometry, inertias, motor port bindings, sensor placements).

## Public contract

Layer 5 receives:

- **Joint commands** as torques and forces from Layer 4 motors,
  per-tick.
- **Field events** from the game plugin (e.g. game piece spawn,
  match phase change).

Layer 5 publishes:

- **Pose, velocity, acceleration** of every body — fed to sensors.
- **Contact events** — for game logic and for diagnostic stream.
- **Loaded torque** at every joint — fed back to motor models so
  stalls and reaction forces are real.

Sensor reads (encoders, gyros, vision) interpret this state through
their Layer 4 models.

## Sub-systems

### Drivetrain

- **Tank / skid steer** — friction model with longitudinal/lateral
  asymmetry. Scrub on turns.
- **Mecanum** — roller forces, less common but supported.
- **Swerve** — module steering and drive as separate joints, ground
  contact at each wheel. Module collisions and slip are real and
  affect odometry.
- **Carpet** — surface friction differs from tile/diamond plate.
  Carpet has direction-dependent friction (the nap matters at
  competition). `[OPEN]` whether nap is modeled in v0.

### Articulated mechanisms

- Joints: revolute, prismatic, fixed.
- Limits: hard stops with realistic compliance (don't pretend
  infinite stiffness).
- Friction at joints: viscous and Coulomb terms.
- Backlash: `[OPEN]`.
- Multi-stage elevators (cascading stages with bowden cables / belts)
  introduce coupled joints — modeled as kinematic constraints or
  full multibody depending on solver.

### Game pieces

- Rigid bodies with specified mass, inertia, contact properties.
- Robot-piece contact: friction-based grip, often via spinning
  rollers in intakes. Notoriously hard — generic contact engines
  handle this poorly. Custom contact handlers may be needed.
- Game-piece-piece contact: pile mechanics, ejection.

### Field

- Static geometry from FIRST's published STEP files where available.
- AprilTag positions and orientations matched to the season's
  layout.
- Lighting for vision rendering — uniform diffuse for v0, real
  lighting later.

### Game logic plugin

- Loaded per-season. Provides:
  - Scoring rules.
  - Match phase / state machine (auto, teleop, endgame, ...).
  - FMS message synthesis when running offline.
  - Per-game special events (e.g. coopertition, ranking).
- The simulator core is season-agnostic; only the plugin knows the
  current game.

### Robot description

A declarative file (TBD format — `[OPEN]` OQ-4) is the source of
truth. It binds:

- Bodies, joints, masses, inertias.
- Motor bindings (port, vendor model, motor type).
- Sensor bindings (port, sensor model, mounting frame).
- Drivetrain selector (tank/mecanum/swerve + parameters).

Visualization, physics, device binding, and validation all read from
this file. One source of truth — no duplicating geometry across
config files.

## Solver and integrator

The physics engine choice is `[OPEN]` (OQ-2). Constraints on the
choice from Layer 5's needs:

- Must integrate stiff systems stably (high gear ratios, hard
  stops). Implicit or semi-implicit integrators. Naive explicit
  Euler is unacceptable.
- Must support friction with anisotropy (carpet) and complex
  contact (game pieces). Bullet and PhysX have mature contact;
  MuJoCo has best actuated-mechanism accuracy; Rapier is modern
  and deterministic.
- Must be deterministic across platforms — same inputs and seed,
  same outputs, regardless of OS or CPU. This matters for replay.

Decision in OQ-2; until resolved, this skill stays engine-agnostic.

## Hard parts and gotchas

- **Swerve modules under contact with carpet.** Small slip, big
  consequences for odometry. Out-of-the-box physics engines model
  this poorly; expect to tune contact parameters carefully.
- **High gear ratios.** A 50:1 elevator joint is stiff. Solver
  explosions look like sudden NaNs or "the elevator launches into
  orbit." Validation tests must include stiff configurations.
- **Game-piece manipulation.** Intakes that grip via friction with
  a spinning roller are notoriously hard. May need custom contact
  handlers in v1; stub-grip with a friction proxy in v0.
- **Vision rendering.** Rendering the field for AprilTag /
  PhotonVision pipelines is a sub-project on its own. May use a
  headless renderer (Vulkan/OpenGL) driven from world state. See
  also `layer-3-device-bus-vendor-firmware.md` and OQ-9.
- **Contact stickiness vs. slip.** Robots being pushed (defense,
  alliance partner contact) reveal contact-model bugs that single-
  robot tests miss. Multi-body validation matters.

## Validation

- Drivetrain models tested against measured trajectories. Compare
  sim vs. recorded match data when available; otherwise vs. carefully
  characterized bench tests.
- Mechanism models tested against measured step responses (e.g.
  arm settles to a target angle in `t` seconds with overshoot
  matching real bot).
- Game-piece interactions: harder to validate quantitatively;
  qualitative video comparison is acceptable for v0 with a tracked
  TODO toward bench fixtures.

## Files / paths (TBD)

Placeholder:

- `world/dynamics/` — solver integration, joint and contact
  primitives.
- `world/drivetrain/` — drivetrain kinematics + dynamics.
- `world/mechanisms/` — generic articulated systems.
- `world/field/<season>/` — per-season field geometry & game logic
  plugin.
- `descriptions/<robot-name>.<ext>` — robot description files.

## v0 status (decided 2026-04-28)

**v0 demo robot: single-DOF arm.** One revolute joint, gravity-loaded,
driven by a Kraken X60 + CANcoder feedback (Layer 3/4) under
closed-loop position control on Phoenix 6.

What this means for v0 Layer 5:

- **In scope:** rigid-body single-link arm with mass, length,
  inertia, and gravity. Revolute joint with limits and viscous
  friction. Stable integrator for the gravity-loaded joint (this is
  the stiff-system test for our solver).
- **Out of scope:** chassis dynamics (no drivetrain demo in v0),
  contact / friction with the ground, game pieces, multi-body
  contact, vision rendering, field geometry, game logic plugin.
- **Validation target:** bench arm with measured mass, length, and
  CG. Sim arm should reproduce step response and gravity-balance
  position to a stated tolerance. Until bench data exists, validation
  is `unvalidated`.

**Physics engine: MuJoCo** (decided 2026-04-28; see
`docs/ARCHITECTURE.md` "Physics engine"). Apache 2.0,
deterministic, best-in-class for stiff actuated mechanisms. Lives
behind the Layer 5 engine-agnostic interface in
`world/engine/mujoco/`. No MuJoCo types appear in Layers 3 or 4.
Federation with a contact-strong engine for game pieces is a
v2+ concern, deferred until evidence demands it.

**Robot description: JSON, our own schema** (decided 2026-04-28;
see `docs/ARCHITECTURE.md` "Robot description format"). FRC
concepts (motor model selectors, vendor + firmware bindings,
controller-to-joint mappings, drivetrain selector, follower
relationships) are first-class. Geometry/joints/inertias/actuators
get transpiled to MJCF at load time and handed to MuJoCo.
nlohmann/json as the parser. v0 file:
`descriptions/v0-arm.json`. Authoring GUI / CAD-import tool
deferred (see OQ-11).

**Loader status (2026-04-29):** the description loader is implemented
and 83-test green; see `.claude/skills/robot-description-loader.md`
for the working knowledge. Layer 5's MJCF generator consumes the
`robot_description` struct (POD types in `src/description/schema.h`)
— *not* the raw JSON. The contract from the loader's side: every
geometry/joint/inertia/actuator field that v0 cares about is
present, type-checked, range-checked, and reference-resolved before
the struct is handed off. Layer 5 can rely on the struct being
internally consistent (no dangling references, no inverted joint
limits, no negative masses). The MJCF transpiler is a Layer 5
feature with its own skill and test plan when it lands.

## Cross-references

- `layer-4-physical-models.md` — torques in, sensor state out.
- `layer-3-device-bus-vendor-firmware.md` — vision co-processors
  receive rendered frames.
- `docs/ARCHITECTURE.md` — process model, time discipline, v0 scope.
- `docs/OPEN_QUESTIONS.md` — OQ-2 (physics engine), OQ-4 (robot
  description format), OQ-9 (vision pipeline).
