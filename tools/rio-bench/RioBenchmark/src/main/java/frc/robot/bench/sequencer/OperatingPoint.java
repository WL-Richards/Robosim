package frc.robot.bench.sequencer;

/**
 * Bus / CPU contention scenarios the sweep cycles through. Together these
 * span the realistic envelope of HAL-call latencies a robot project will
 * see in match conditions:
 *
 * <ul>
 *   <li>{@link #IDLE_BUS} — quiescent baseline; nothing else hammering the HAL.</li>
 *   <li>{@link #SATURATED_BUS} — CAN bus loaded to model competition wiring.
 *       Cycle-A onwards: software-driven (a CAN-frame-spam worker, see
 *       {@link frc.robot.bench.runner.WorkerSpec}); v0 was operator-hardware-
 *       driven (4 physical TalonFX devices). The label is preserved across
 *       the semantic shift so downstream consumers do not need to rename;
 *       see {@code D-RB-8} in {@code .claude/skills/rio-bench.md}.</li>
 *   <li>{@link #MULTI_THREAD} — additional worker threads contending for the
 *       HAL, modeling logging / command-scheduler activity.</li>
 *   <li>{@link #SATURATED_MULTI_THREAD} — both saturated bus AND worker
 *       threads, the realistic match-condition cross-product. New in
 *       cycle A.</li>
 *   <li>{@link #SATURATED_BUS_4} / {@link #SATURATED_BUS_8} — heavier
 *       software-driven CAN load tiers (4 and 8 spam workers). New in
 *       cycle C; gives the sim core a load-scaling curve.</li>
 *   <li>{@link #SATURATED_BUS_4_MULTI_THREAD} /
 *       {@link #SATURATED_BUS_8_MULTI_THREAD} — cross-products of the
 *       heavier load tiers with CPU-contention workers. New in cycle C.</li>
 * </ul>
 *
 * <p>Declaration order is fixed: {@code IDLE_BUS, SATURATED_BUS, MULTI_THREAD,
 * SATURATED_MULTI_THREAD, SATURATED_BUS_4, SATURATED_BUS_8,
 * SATURATED_BUS_4_MULTI_THREAD, SATURATED_BUS_8_MULTI_THREAD} (ordinals
 * 0–7). The first four ordinals are preserved from cycle A so existing
 * fixtures keyed off {@code ordinal()} (notably
 * {@code BenchmarkPipelineTest}'s synthetic-run formula) remain correct for
 * the cells they already covered.
 *
 * <p>The {@code csvLabel} is the stable string written into the output CSV;
 * downstream consumers (sim core, dashboards) match on it, so values here
 * are part of the v0 output contract.
 */
public enum OperatingPoint {
  IDLE_BUS("idle_bus"),
  SATURATED_BUS("saturated_bus"),
  MULTI_THREAD("multi_thread"),
  SATURATED_MULTI_THREAD("saturated_multi_thread"),
  SATURATED_BUS_4("saturated_bus_4"),
  SATURATED_BUS_8("saturated_bus_8"),
  SATURATED_BUS_4_MULTI_THREAD("saturated_bus_4_multi_thread"),
  SATURATED_BUS_8_MULTI_THREAD("saturated_bus_8_multi_thread");

  private final String csvLabel;

  OperatingPoint(String csvLabel) {
    this.csvLabel = csvLabel;
  }

  /** @return the stable CSV label for this operating point (part of the v0 output contract). */
  public String csvLabel() {
    return csvLabel;
  }
}
