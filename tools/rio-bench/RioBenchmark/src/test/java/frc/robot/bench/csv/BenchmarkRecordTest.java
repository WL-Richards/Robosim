package frc.robot.bench.csv;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.stats.Statistics.Stats;
import org.junit.jupiter.api.Test;

class BenchmarkRecordTest {

  @Test
  void to_csv_row_emits_documented_column_order() {
    // R1': six mutually-distinct numerical values pin the column order.
    // Any swap between any two µs columns or between any µs column and
    // outlier_rate produces a different rendered string.
    Stats s = new Stats(1.100, 2.200, 3.300, 4.400, 5.500, 0.006600);
    BenchmarkRecord r =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 7, s);
    assertEquals(
        "hal_get_fpga_time,idle_bus,7,1.100,2.200,3.300,4.400,5.500,0.006600\n",
        r.toCsvRow());
  }

  @Test
  void comparator_orders_by_call_class_then_operating_point() {
    Stats s = new Stats(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    BenchmarkRecord a =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, s);
    BenchmarkRecord aSamePoint =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, s);
    BenchmarkRecord aOtherPoint =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.SATURATED_BUS, 1, s);
    BenchmarkRecord otherClassSamePoint =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s);
    BenchmarkRecord otherClassOtherPoint =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.SATURATED_BUS, 1, s);

    assertEquals(0, a.compareTo(aSamePoint));

    assertTrue(a.compareTo(aOtherPoint) < 0);
    assertTrue(aOtherPoint.compareTo(a) > 0);

    assertTrue(a.compareTo(otherClassSamePoint) < 0);
    assertTrue(otherClassSamePoint.compareTo(a) > 0);

    assertTrue(aOtherPoint.compareTo(otherClassSamePoint) < 0,
        "primary key (call_class) dominates: 'can_frame_read' < 'hal_get_fpga_time'");
    assertTrue(otherClassOtherPoint.compareTo(aOtherPoint) > 0,
        "primary key dominates regardless of operating point");
  }

  @Test
  void equals_is_value_based() {
    Stats s1 = new Stats(1.0, 2.0, 3.0, 4.0, 5.0, 0.5);
    Stats s2 = new Stats(1.0, 2.0, 3.0, 4.0, 5.0, 0.5);
    BenchmarkRecord a =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, s1);
    BenchmarkRecord b =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, s2);
    assertEquals(a, b);
    assertEquals(a.hashCode(), b.hashCode());

    BenchmarkRecord differentClass =
        new BenchmarkRecord(CallClass.CAN_FRAME_WRITE, OperatingPoint.IDLE_BUS, 1, s1);
    assertNotEquals(a, differentClass);

    BenchmarkRecord differentPoint =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.SATURATED_BUS, 1, s1);
    assertNotEquals(a, differentPoint);

    Stats sDifferent = new Stats(99.0, 2.0, 3.0, 4.0, 5.0, 0.5);
    BenchmarkRecord differentStats =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, sDifferent);
    assertNotEquals(a, differentStats);
  }

  @Test
  void stats_record_field_order_is_pinned() {
    // R4: pin the Stats record's positional constructor → accessor
    // mapping. A field-reorder in the record declaration that compiles
    // successfully would silently corrupt every CSV row downstream;
    // R1' alone would not catch it if toCsvRow's String.format arg order
    // were also swapped (the two swaps would cancel). R4 reads accessor
    // values directly, so it catches the Java-side reorder regardless of
    // toCsvRow.
    Stats s = new Stats(1.0, 2.0, 3.0, 4.0, 5.0, 0.06);
    assertEquals(1.0, s.meanUs(), 0.0);
    assertEquals(2.0, s.stddevUs(), 0.0);
    assertEquals(3.0, s.p50Us(), 0.0);
    assertEquals(4.0, s.p99Us(), 0.0);
    assertEquals(5.0, s.p999Us(), 0.0);
    assertEquals(0.06, s.outlierRate(), 0.0);
  }
}
