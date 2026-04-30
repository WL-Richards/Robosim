// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.bench.csv.RunMetadata;
import frc.robot.bench.runner.BenchmarkRunner;

/**
 * TimedRobot harness for rio-bench.
 *
 * <p>The benchmark sweep is launched once per autonomous-enable on a daemon
 * worker thread, leaving the WPILib main loop free for the watchdog. Running
 * the sweep out of autonomous (rather than {@code robotInit}) means the
 * operator can disable, reposition the robot, and re-enable to gather another
 * run on the same boot — and crucially that the robot is in a known
 * field-legal state before any HAL calls are timed.
 */
public class Robot extends TimedRobot {

  private Thread benchThread;

  /**
   * Boots the WPILib data log so the bench can record progress and any
   * runtime errors to the same RIO-resident log the operator already
   * downloads after each run.
   */
  public Robot() {
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {}

  /**
   * Starts the benchmark sweep on its own daemon thread the first time
   * autonomous is enabled. Subsequent enables while a sweep is already
   * running are ignored — the runner itself enforces a one-shot guard, but
   * checking here avoids spawning and immediately discarding a thread.
   */
  @Override
  public void autonomousInit() {
    if (benchThread != null && benchThread.isAlive()) {
      return;
    }

    RunMetadata metadata = BenchmarkRunner.buildMetadata(WPILibVersion.Version, detectRioFirmware());
    BenchmarkRunner runner = new BenchmarkRunner(metadata);

    benchThread = new Thread(runner::runOnce, "rio-bench");
    benchThread.setDaemon(true);
    benchThread.start();
  }

  /**
   * Reads the RIO firmware version from the {@code RIO_FIRMWARE} env var
   * the deploy script sets. We do not query the FPGA directly because the
   * value is human-curated — it is one of the things being correlated with
   * the benchmark CSV by filename.
   */
  private static String detectRioFirmware() {
    return System.getenv().getOrDefault("RIO_FIRMWARE", "unknown");
  }
}
