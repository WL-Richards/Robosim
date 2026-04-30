package frc.robot.bench.runner;

import frc.robot.bench.sequencer.OperatingPoint;

/**
 * Per-{@link OperatingPoint} background-load configuration the
 * {@link BenchmarkRunner} consumes when entering a measured phase. Two
 * dimensions:
 *
 * <ul>
 *   <li>{@code cpuWorkers} — number of CPU-contention worker threads to
 *       spawn (each loops on a cheap HAL call; modeling logging /
 *       command-scheduler activity that competes with the bench thread).</li>
 *   <li>{@code canSpamWorkers} — number of CAN-frame-spam worker threads
 *       to spawn. Each loops on
 *       {@code FRCNetCommCANSessionMuxSendMessage} to a non-bench message
 *       ID, software-saturating the CAN bus regardless of physical device
 *       count. Cycle C generalized this from cycle A's boolean
 *       (0 or 1 workers) to an integer count to support load-tier sweeps
 *       at 1 / 4 / 8 spam workers.</li>
 * </ul>
 *
 * <p>This table separates the operating-point → load-profile mapping out of
 * {@link BenchmarkRunner} so it is unit-testable without spinning up HAL
 * JNI. The runner-side spawn logic itself remains HAL-bound and not
 * unit-tested per {@code D-RB-7} in {@code .claude/skills/rio-bench.md};
 * this record is the testable seam.
 *
 * <p>Equality and hashCode are derived from the two fields (Java {@code
 * record} default), which {@code WorkerSpecTest.W5'} relies on for
 * {@code Set}-membership distinctness checks.
 */
public record WorkerSpec(int cpuWorkers, int canSpamWorkers) {

  /**
   * Returns the load profile for the given operating point.
   *
   * <p><strong>Implementation rule.</strong> The body below is an exhaustive
   * switch <em>expression</em> with <strong>no {@code default} branch</strong>.
   * The Java compiler enforces exhaustiveness over the {@link OperatingPoint}
   * enum members; if a future contributor adds a ninth member without
   * updating this method, compilation fails. A {@code default} branch would
   * silently swallow the missing case and return a wrong-but-plausible spec.
   * Do not add one.
   *
   * @throws NullPointerException if {@code p} is {@code null} (Java
   *     switch-on-null-enum behavior, JLS 14.11.1).
   */
  public static WorkerSpec forPoint(OperatingPoint p) {
    return switch (p) {
      case IDLE_BUS -> new WorkerSpec(0, 0);
      case SATURATED_BUS -> new WorkerSpec(0, 1);
      case MULTI_THREAD -> new WorkerSpec(3, 0);
      case SATURATED_MULTI_THREAD -> new WorkerSpec(3, 1);
      case SATURATED_BUS_4 -> new WorkerSpec(0, 4);
      case SATURATED_BUS_8 -> new WorkerSpec(0, 8);
      case SATURATED_BUS_4_MULTI_THREAD -> new WorkerSpec(3, 4);
      case SATURATED_BUS_8_MULTI_THREAD -> new WorkerSpec(3, 8);
    };
  }
}
