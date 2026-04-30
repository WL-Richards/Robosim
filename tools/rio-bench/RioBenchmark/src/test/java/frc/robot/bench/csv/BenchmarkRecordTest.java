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
    Stats s = new Stats(1.234, 0.567, 9.876, 0.001234);
    BenchmarkRecord r =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.SATURATED_BUS, 42, s);
    assertEquals("can_frame_read,saturated_bus,42,1.234,0.567,9.876,0.001234\n", r.toCsvRow());
  }

  @Test
  void comparator_orders_by_call_class_then_operating_point() {
    Stats s = new Stats(0.0, 0.0, 0.0, 0.0);
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
    Stats s1 = new Stats(1.0, 2.0, 3.0, 0.5);
    Stats s2 = new Stats(1.0, 2.0, 3.0, 0.5);
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

    Stats sDifferent = new Stats(99.0, 2.0, 3.0, 0.5);
    BenchmarkRecord differentStats =
        new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, sDifferent);
    assertNotEquals(a, differentStats);
  }
}
