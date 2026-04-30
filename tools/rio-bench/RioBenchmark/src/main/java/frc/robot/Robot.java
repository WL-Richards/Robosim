// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.bench.csv.RunMetadata;
import frc.robot.bench.runner.BenchmarkRunner;

public class Robot extends TimedRobot {

  private Thread benchThread;

  public Robot() {
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    if (benchThread != null && benchThread.isAlive()) return;

    RunMetadata metadata = BenchmarkRunner.buildMetadata(WPILibVersion.Version, detectRioFirmware());
    BenchmarkRunner runner = new BenchmarkRunner(metadata);

    benchThread = new Thread(runner::runOnce, "rio-bench");
    benchThread.setDaemon(true);
    benchThread.start();
  }

  private static String detectRioFirmware() {
    return System.getenv().getOrDefault("RIO_FIRMWARE", "unknown");
  }
}
