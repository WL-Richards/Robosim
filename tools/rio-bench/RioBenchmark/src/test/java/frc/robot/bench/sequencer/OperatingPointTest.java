package frc.robot.bench.sequencer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class OperatingPointTest {

  @Test
  void operating_point_enum_has_eight_members_in_documented_order() {
    OperatingPoint[] values = OperatingPoint.values();
    assertEquals(8, values.length);
    assertSame(OperatingPoint.IDLE_BUS, values[0]);
    assertSame(OperatingPoint.SATURATED_BUS, values[1]);
    assertSame(OperatingPoint.MULTI_THREAD, values[2]);
    assertSame(OperatingPoint.SATURATED_MULTI_THREAD, values[3]);
    assertSame(OperatingPoint.SATURATED_BUS_4, values[4]);
    assertSame(OperatingPoint.SATURATED_BUS_8, values[5]);
    assertSame(OperatingPoint.SATURATED_BUS_4_MULTI_THREAD, values[6]);
    assertSame(OperatingPoint.SATURATED_BUS_8_MULTI_THREAD, values[7]);
  }

  @Test
  void csv_label_matches_documented_table() {
    assertEquals("idle_bus", OperatingPoint.IDLE_BUS.csvLabel());
    assertEquals("saturated_bus", OperatingPoint.SATURATED_BUS.csvLabel());
    assertEquals("multi_thread", OperatingPoint.MULTI_THREAD.csvLabel());
    assertEquals(
        "saturated_multi_thread", OperatingPoint.SATURATED_MULTI_THREAD.csvLabel());
    assertEquals("saturated_bus_4", OperatingPoint.SATURATED_BUS_4.csvLabel());
    assertEquals("saturated_bus_8", OperatingPoint.SATURATED_BUS_8.csvLabel());
    assertEquals(
        "saturated_bus_4_multi_thread",
        OperatingPoint.SATURATED_BUS_4_MULTI_THREAD.csvLabel());
    assertEquals(
        "saturated_bus_8_multi_thread",
        OperatingPoint.SATURATED_BUS_8_MULTI_THREAD.csvLabel());
  }

  @Test
  void csv_labels_lex_sort_in_documented_order() {
    // Hand-pinned expected sort order. Independent of any generated
    // artifact (P1''' fixture is regenerated, so a buggy comparator
    // would produce a buggy fixture both agree on).
    List<String> labels =
        Arrays.stream(OperatingPoint.values()).map(OperatingPoint::csvLabel).sorted().toList();
    assertEquals(
        List.of(
            "idle_bus",
            "multi_thread",
            "saturated_bus",
            "saturated_bus_4",
            "saturated_bus_4_multi_thread",
            "saturated_bus_8",
            "saturated_bus_8_multi_thread",
            "saturated_multi_thread"),
        labels);
  }
}
