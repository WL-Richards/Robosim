package frc.robot.bench.calls;

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

  public String csvLabel() {
    return csvLabel;
  }

  public int blockSize() {
    return blockSize;
  }
}
