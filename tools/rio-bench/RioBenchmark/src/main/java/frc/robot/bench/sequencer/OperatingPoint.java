package frc.robot.bench.sequencer;

/**
 * Bus / CPU contention scenarios the sweep cycles through. Together these
 * span the realistic envelope of HAL-call latencies a robot project will
 * see in match conditions:
 *
 * <ul>
 *   <li>{@link #IDLE_BUS} — quiescent baseline; nothing else hammering the HAL.</li>
 *   <li>{@link #SATURATED_BUS} — CAN bus loaded to model competition wiring.</li>
 *   <li>{@link #MULTI_THREAD} — additional worker threads contending for the
 *       HAL, modeling logging / command-scheduler activity.</li>
 * </ul>
 *
 * <p>The {@code csvLabel} is the stable string written into the output CSV;
 * downstream consumers (sim core, dashboards) match on it, so values here
 * are part of the v0 output contract.
 */
public enum OperatingPoint {
  IDLE_BUS("idle_bus"),
  SATURATED_BUS("saturated_bus"),
  MULTI_THREAD("multi_thread");

  private final String csvLabel;

  OperatingPoint(String csvLabel) {
    this.csvLabel = csvLabel;
  }

  /** @return the stable CSV label for this operating point (part of the v0 output contract). */
  public String csvLabel() {
    return csvLabel;
  }
}
