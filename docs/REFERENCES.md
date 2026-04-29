# References

Sources we draw on. Add anything you actually used, with one line on what
it gave us. Empty notes are useless — say *why* this is here.

## WPILib

- WPILib monorepo: <https://github.com/wpilibsuite/allwpilib>
  HAL headers, simulation HAL implementation, NetworkTables, WPILOG. Read
  `hal/src/main/native/include/hal/` for the surface our Layer 2 must
  implement.
- WPILib simulation docs: <https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/>
  Existing sim story; the bar we have to clear and the conventions we
  should not gratuitously break.
- AdvantageKit: <https://github.com/Mechanical-Advantage/AdvantageKit>
  IO-layer pattern lots of teams use; their sim approach is influential
  and worth studying. WPILOG + NT4 logging.
- AdvantageScope: <https://github.com/Mechanical-Advantage/AdvantageScope>
  Default log viewer for FRC; we should produce logs it understands.

## Vendor

- CTRE Phoenix 6 docs: <https://pro.docs.ctr-electronics.com/>
  API, control modes, status frames for Talon FX / CANcoder / Pigeon 2.
- CTRE Phoenix 6 simulation: <https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/>
  Their official sim hooks; we want to be at least as faithful as their
  own sim.
- REVLib: <https://docs.revrobotics.com/revlib>
  Spark Max / Spark Flex API, REV PH, Through Bore Encoder.
- REV motor data: <https://www.revrobotics.com/rev-21-1650/> (NEO),
  <https://www.revrobotics.com/rev-21-1652/> (NEO Vortex). Curves and
  constants live on the product pages.
- Kraken X60 datasheet: <https://store.ctr-electronics.com/announcing-kraken-x60/>
  Motor curves, electrical constants.

## Physics & motors

- "Modeling and Simulation of Electric Motors" (textbook references in
  the FRC Mechanical Advantage / WPILib docs). The DC machine model in
  `wpimath`'s `LinearSystemId.createDCMotorSystem` is the starting point
  but is intentionally simplified.
- Brian Douglas / Engineering with Rosie videos — practical control-
  systems pedagogy; useful for sanity checks.
- MuJoCo docs: <https://mujoco.readthedocs.io/>
- Rapier docs: <https://rapier.rs/>
- Bullet manual: <https://github.com/bulletphysics/bullet3>

## Prior art (FRC-specific simulation)

- Maple-Sim: <https://github.com/Shenzhen-Robotics-Alliance/maple-sim>
  Recent FRC physics sim integrating with WPILib; closest thing to what
  we're building. Study and learn from, but not the target.
- WPILib Romi simulation, Webots integration: existing routes for sim,
  generally lower fidelity than our goal.
- Choreo / PathPlanner trajectory tools: not sims, but their kinematic
  models inform what we need to produce.
- **WL-Richards/RIOEmulator:** <https://github.com/WL-Richards/RIOEmulator>
  The user's prior RoboRIO emulator. ~20k LOC C/C++/Python. Boots
  NI Linux RT under QEMU on emulated Zynq, replaces three NI shim libs
  cross-compiled with WPILib's ARM toolchain, runs unmodified robot
  code through DS + NT + 50 Hz periodic. Evidence that the QEMU+shim
  approach to OQ-1 option B is viable. **What to carry forward
  conceptually:** the per-FPGA-subsystem shim file decomposition
  (`tDIO_impl.cpp`, `tPWM_impl.cpp`, `tAlarm_impl.cpp`, …); the CAN
  router pattern (decode 29-bit FRC arbitration ID, dispatch to
  per-device handlers); a register-backing-store abstraction for
  testability. **What not to repeat:** hardcoded year/firmware in
  namespaces (`nFRC_2024_24_0_0`), single-vendor (TalonFX-only)
  coverage, toy stub physics in `talonfx_stub.c`, host-bridged DS
  with no record/replay (kills determinism), no plugin layer for
  alternative control systems.

## Hardware (RoboRIO 2)

- Xilinx Zynq-7020 TRM: SoC reference manual; needed if we go QEMU.
- NI Linux RT documentation; PREEMPT_RT specifics.
- FRC Driver Station protocol — partially documented at:
  <https://frcture.readthedocs.io/en/latest/driverstation/index.html>
  (community reverse-engineered; verify before relying). Direct
  reference for the v0 HAL DS surface.
- RoboRIO FPGA registers (community reverse-engineered) —
  <https://frcture.readthedocs.io/en/latest/fpga/registers.html>
  Register-level reference for the FPGA state surface our HAL shim
  exposes through the boundary protocol. Cross-check against
  WPILib HAL implementations and validate against bench measurements
  before relying on any specific register's behavior.

## CAN bus

- ISO 11898 (CAN spec) — for arbitration / framing details.
- CTRE CANivore docs — for the multi-bus topology FRC uses today.

## QEMU / emulation

- QEMU Zynq machine: <https://www.qemu.org/docs/master/system/arm/xlnx-versal-virt.html>
  Closest off-the-shelf target; verification of fit is part of OQ-1.

## Game / season

- Per-season game manuals (FIRST publishes annually). Field STEP files
  are usually released; we'll need them for visualization.
- AprilTag 36h11 family — FRC's tag dialect. <https://april.eecs.umich.edu/software/apriltag>

---

When you add something, prefer linking the canonical source (vendor docs,
repo) rather than secondary articles. Secondary sources go stale.
