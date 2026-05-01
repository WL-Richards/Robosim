// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  double lastTimestamp = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    System.out.println("Robot Constructor Run!");
  }

  @Override
  public void robotInit() {
    System.out.println("Robot Init Run!");
  }

  @Override
  public void robotPeriodic() {
    double currentTimeStamp = Timer.getFPGATimestamp();
    System.out.println(currentTimeStamp-lastTimestamp);
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous Init Run!");
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("Autonomous Periodic Run!");
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop Init Run!");
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("Teleop Periodic Run!");
  }

  @Override
  public void disabledInit() {
    System.out.println("Disabled Init Run!");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    System.out.println("Test Init Run!");
  }

  @Override
  public void testPeriodic() {
    System.out.println("Test Periodic Run!");
  }

  @Override
  public void simulationInit() {
    System.out.println("Simulation Init Run!");
  }

  @Override
  public void simulationPeriodic() {
    System.out.println("Simulation Periodic Run!");
  }
}
