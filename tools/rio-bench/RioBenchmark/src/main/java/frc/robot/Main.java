// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * WPILib entry point. Hands control to {@link RobotBase}, which constructs
 * a {@link Robot} and drives the periodic loop.
 */
public final class Main {
  private Main() {}

  /**
   * JVM entry. Forwarded args are unused — GradleRIO supplies them, but the
   * benchmark robot has no CLI flags of its own.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
