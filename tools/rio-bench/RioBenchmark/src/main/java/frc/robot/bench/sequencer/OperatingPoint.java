package frc.robot.bench.sequencer;

public enum OperatingPoint {
  IDLE_BUS("idle_bus"),
  SATURATED_BUS("saturated_bus"),
  MULTI_THREAD("multi_thread");

  private final String csvLabel;

  OperatingPoint(String csvLabel) {
    this.csvLabel = csvLabel;
  }

  public String csvLabel() {
    return csvLabel;
  }
}
