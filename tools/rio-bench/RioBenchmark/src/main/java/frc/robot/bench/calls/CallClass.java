package frc.robot.bench.calls;

/**
 * Taxonomy of HAL call sites the bench measures. Each entry names a
 * distinct latency-class on the RIO; we do not enumerate every HAL
 * function, just the ones whose cost matters for tier-1 simulator
 * fidelity.
 *
 * <p>{@code blockSize} is the number of underlying HAL calls executed per
 * timing block in {@link frc.robot.bench.runner.CallBindings}. Block sizes
 * are chosen so the wall-clock window comfortably exceeds nanoTime
 * granularity on the RIO 2 (~250 ns): cheap calls run in 100-call blocks,
 * costly calls (CAN, notifier wait) run as 1-call blocks.
 *
 * <p>{@code csvLabel} is the stable string written into the output CSV
 * and is part of the v0 output contract.
 */
public enum CallClass {
  HAL_GET_FPGA_TIME("hal_get_fpga_time", 100),
  HAL_NOTIFIER_WAIT("hal_notifier_wait", 1),
  DS_STATE_READ("ds_state_read", 10),
  POWER_STATE_READ("power_state_read", 10),
  CAN_FRAME_READ("can_frame_read", 1),
  CAN_FRAME_WRITE("can_frame_write", 1);

  private final String csvLabel;
  private final int blockSize;

  CallClass(String csvLabel, int blockSize) {
    this.csvLabel = csvLabel;
    this.blockSize = blockSize;
  }

  /** @return the stable CSV label for this call class (part of the v0 output contract). */
  public String csvLabel() {
    return csvLabel;
  }

  /**
   * @return the number of underlying HAL invocations per timing block;
   *     statistics divide by this to report per-call latency
   */
  public int blockSize() {
    return blockSize;
  }
}
