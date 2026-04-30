package frc.robot.bench.sequencer;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class SequencerTest {

  private static final List<CallClass> ALL_CLASSES = List.of(CallClass.values());

  private static void fastForwardToPoint(
      Sequencer s,
      OperatingPoint target,
      List<CallClass> callClasses,
      int warmupCount,
      int perPointCount) {
    if (s.isWarmingUp()) {
      for (int round = 0; round < warmupCount; round++) {
        for (CallClass c : callClasses) s.submitSample(c, 1000L);
      }
    }
    for (OperatingPoint p : OperatingPoint.values()) {
      if (p == target) return;
      for (int round = 0; round < perPointCount; round++) {
        for (CallClass c : callClasses) s.submitSample(c, 1000L);
      }
    }
  }

  @Test
  void starts_in_warmup_state() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    assertTrue(s.isWarmingUp());
    assertFalse(s.isDone());
  }

  @Test
  void transitions_warmup_to_idle_bus_after_warmup_count_samples_per_call() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    for (int round = 0; round < 9; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertTrue(s.isWarmingUp(), "after 59 samples should still be warming up");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertFalse(s.isWarmingUp());
    assertEquals(OperatingPoint.IDLE_BUS, s.currentPoint());
  }

  @Test
  void transitions_idle_to_saturated_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.IDLE_BUS, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(OperatingPoint.IDLE_BUS, s.currentPoint(), "after 599: still IDLE_BUS");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(OperatingPoint.SATURATED_BUS, s.currentPoint(), "after 600: SATURATED_BUS");
  }

  @Test
  void transitions_saturated_to_multi_thread_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(OperatingPoint.SATURATED_BUS, s.currentPoint(), "after 599: still SATURATED_BUS");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(OperatingPoint.MULTI_THREAD, s.currentPoint(), "after 600: MULTI_THREAD");
  }

  @Test
  void transitions_multi_thread_to_done_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.MULTI_THREAD, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertFalse(s.isDone(), "after 599: not done");
    assertEquals(OperatingPoint.MULTI_THREAD, s.currentPoint());

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertTrue(s.isDone(), "after 600: done");
  }

  @Test
  void done_state_rejects_further_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.MULTI_THREAD, ALL_CLASSES, 10, 100);
    for (int round = 0; round < 100; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    assertTrue(s.isDone());

    assertThrows(
        IllegalStateException.class,
        () -> s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1000L));

    long[] mtSamples = s.samplesFor(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.MULTI_THREAD);
    assertEquals(100, mtSamples.length);
  }

  @Test
  void warmup_samples_are_discarded() {
    List<CallClass> oneClass = List.of(CallClass.HAL_GET_FPGA_TIME);
    Sequencer s = new Sequencer(2, 3, oneClass);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 9999L);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 9999L);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 10L);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 11L);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 12L);
    for (int i = 0; i < 3; i++) s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1000L);
    for (int i = 0; i < 3; i++) s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1000L);
    assertTrue(s.isDone());

    long[] idle = s.samplesFor(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS);
    assertArrayEquals(new long[] {10L, 11L, 12L}, idle);
  }

  @Test
  void samples_routed_to_correct_call_class_bucket() {
    List<CallClass> twoClasses =
        List.of(CallClass.HAL_GET_FPGA_TIME, CallClass.CAN_FRAME_READ);
    Sequencer s = new Sequencer(1, 1, twoClasses);
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1L);
    s.submitSample(CallClass.CAN_FRAME_READ, 1L);

    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 100L);
    s.submitSample(CallClass.CAN_FRAME_READ, 200L);

    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 300L);
    s.submitSample(CallClass.CAN_FRAME_READ, 400L);

    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 500L);
    s.submitSample(CallClass.CAN_FRAME_READ, 600L);
    assertTrue(s.isDone());

    assertArrayEquals(
        new long[] {100L},
        s.samplesFor(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS));
    assertArrayEquals(
        new long[] {200L},
        s.samplesFor(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS));
  }

  @Test
  void submitting_unknown_call_class_throws() {
    List<CallClass> twoClasses =
        List.of(CallClass.HAL_GET_FPGA_TIME, CallClass.CAN_FRAME_READ);
    Sequencer s = new Sequencer(1, 1, twoClasses);
    assertThrows(
        IllegalArgumentException.class,
        () -> s.submitSample(CallClass.CAN_FRAME_WRITE, 1000L));
  }

  @Test
  void produces_records_for_every_call_x_point_combination() {
    Sequencer s = new Sequencer(0, 1, ALL_CLASSES);
    for (int phase = 0; phase < 3; phase++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    assertTrue(s.isDone());

    List<BenchmarkRecord> records = s.extractRecords();
    assertEquals(18, records.size());

    Set<String> seen = new HashSet<>();
    for (BenchmarkRecord r : records) {
      seen.add(r.callClass() + "/" + r.operatingPoint());
    }
    Set<String> expected = new HashSet<>();
    for (CallClass c : ALL_CLASSES) {
      for (OperatingPoint p : OperatingPoint.values()) {
        expected.add(c + "/" + p);
      }
    }
    assertEquals(expected, seen);
  }
}
