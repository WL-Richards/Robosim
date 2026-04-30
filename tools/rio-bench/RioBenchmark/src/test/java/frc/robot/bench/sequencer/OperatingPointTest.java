package frc.robot.bench.sequencer;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class OperatingPointTest {

  @Test
  void operating_point_enum_has_three_members_in_documented_order() {
    OperatingPoint[] values = OperatingPoint.values();
    assertEquals(3, values.length);
    assertEquals(OperatingPoint.IDLE_BUS, values[0]);
    assertEquals(OperatingPoint.SATURATED_BUS, values[1]);
    assertEquals(OperatingPoint.MULTI_THREAD, values[2]);
  }

  @Test
  void csv_label_matches_documented_table() {
    assertEquals("idle_bus", OperatingPoint.IDLE_BUS.csvLabel());
    assertEquals("saturated_bus", OperatingPoint.SATURATED_BUS.csvLabel());
    assertEquals("multi_thread", OperatingPoint.MULTI_THREAD.csvLabel());
  }
}
