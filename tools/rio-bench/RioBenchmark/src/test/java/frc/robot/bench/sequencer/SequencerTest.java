package frc.robot.bench.sequencer;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
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
  void transitions_multi_thread_to_saturated_multi_thread_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.MULTI_THREAD, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(OperatingPoint.MULTI_THREAD, s.currentPoint(), "after 599: still MULTI_THREAD");
    assertFalse(s.isDone(), "after 599: not done");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_MULTI_THREAD,
        s.currentPoint(),
        "after 600: SATURATED_MULTI_THREAD");
    assertFalse(
        s.isDone(),
        "after 600: not done — SATURATED_BUS_4 and four more phases remain under cycle C");
  }

  @Test
  void transitions_saturated_multi_thread_to_saturated_bus_4_after_per_point_samples() {
    // Cycle C: SATURATED_MULTI_THREAD is no longer terminal. A hardcoded
    // PHASES.length == 4 check from cycle A would silently treat it as
    // terminal; the !isDone() assertion at 600 catches that.
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_MULTI_THREAD, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_MULTI_THREAD,
        s.currentPoint(),
        "after 599: still SATURATED_MULTI_THREAD");
    assertFalse(s.isDone(), "after 599: not done");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_BUS_4, s.currentPoint(), "after 600: SATURATED_BUS_4");
    assertFalse(s.isDone(), "after 600: not done — three more phases remain");
  }

  @Test
  void transitions_saturated_bus_4_to_saturated_bus_8_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS_4, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(OperatingPoint.SATURATED_BUS_4, s.currentPoint(), "after 599: still SATURATED_BUS_4");
    assertFalse(s.isDone(), "after 599: not done");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_BUS_8, s.currentPoint(), "after 600: SATURATED_BUS_8");
    assertFalse(s.isDone(), "after 600: not done — two more phases remain");
  }

  @Test
  void transitions_saturated_bus_8_to_saturated_bus_4_multi_thread_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS_8, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(OperatingPoint.SATURATED_BUS_8, s.currentPoint(), "after 599: still SATURATED_BUS_8");
    assertFalse(s.isDone(), "after 599: not done");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_BUS_4_MULTI_THREAD,
        s.currentPoint(),
        "after 600: SATURATED_BUS_4_MULTI_THREAD");
    assertFalse(s.isDone(), "after 600: not done — one more phase remains");
  }

  @Test
  void transitions_saturated_bus_4_multi_thread_to_saturated_bus_8_multi_thread_after_per_point_samples() {
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS_4_MULTI_THREAD, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_BUS_4_MULTI_THREAD,
        s.currentPoint(),
        "after 599: still SATURATED_BUS_4_MULTI_THREAD");
    assertFalse(s.isDone(), "after 599: not done");

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertEquals(
        OperatingPoint.SATURATED_BUS_8_MULTI_THREAD,
        s.currentPoint(),
        "after 600: SATURATED_BUS_8_MULTI_THREAD");
    assertFalse(s.isDone(), "after 600: not done — terminal phase next");
  }

  @Test
  void transitions_saturated_bus_8_multi_thread_to_done_after_per_point_samples() {
    // Terminal-boundary test (analogue of cycle-A's Q5b at the new
    // terminal). currentPoint() is undefined after done per the v0
    // accessor contract, so we don't assert it.
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS_8_MULTI_THREAD, ALL_CLASSES, 10, 100);

    for (int round = 0; round < 99; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    for (int i = 0; i < 5; i++) s.submitSample(ALL_CLASSES.get(i), 1000L);
    assertFalse(s.isDone(), "after 599: not done");
    assertEquals(OperatingPoint.SATURATED_BUS_8_MULTI_THREAD, s.currentPoint());

    s.submitSample(ALL_CLASSES.get(5), 1000L);
    assertTrue(s.isDone(), "after 600: done");
  }

  @Test
  void done_state_rejects_further_samples() {
    // Q6'': terminal phase shifted from SATURATED_MULTI_THREAD (cycle A)
    // to SATURATED_BUS_8_MULTI_THREAD (cycle C). Bucket-protection
    // assertion targets HAL_GET_FPGA_TIME specifically — the rejected
    // sample must use the same call class so the post-rejection
    // bucket-count check is non-trivial.
    Sequencer s = new Sequencer(10, 100, ALL_CLASSES);
    fastForwardToPoint(s, OperatingPoint.SATURATED_BUS_8_MULTI_THREAD, ALL_CLASSES, 10, 100);
    for (int round = 0; round < 100; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    assertTrue(s.isDone());

    assertThrows(
        IllegalStateException.class,
        () -> s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1000L));

    long[] terminalBucket =
        s.samplesFor(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.SATURATED_BUS_8_MULTI_THREAD);
    assertEquals(100, terminalBucket.length);
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
    for (int p = 0; p < OperatingPoint.values().length - 1; p++) {
      for (int i = 0; i < 3; i++) s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1000L);
    }
    assertTrue(s.isDone());

    long[] idle = s.samplesFor(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS);
    assertArrayEquals(new long[] {10L, 11L, 12L}, idle);
  }

  @Test
  void samples_routed_to_correct_call_class_bucket() {
    List<CallClass> twoClasses =
        List.of(CallClass.HAL_GET_FPGA_TIME, CallClass.CAN_FRAME_READ);
    Sequencer s = new Sequencer(1, 1, twoClasses);
    // Walk: 1 warmup round + 1 round per operating point
    // (8 op-points under cycle C). Total 9 rounds × 2 classes = 18 submissions.
    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 1L);
    s.submitSample(CallClass.CAN_FRAME_READ, 1L);

    s.submitSample(CallClass.HAL_GET_FPGA_TIME, 100L);
    s.submitSample(CallClass.CAN_FRAME_READ, 200L);

    for (int round = 0; round < OperatingPoint.values().length - 1; round++) {
      s.submitSample(CallClass.HAL_GET_FPGA_TIME, 999L);
      s.submitSample(CallClass.CAN_FRAME_READ, 999L);
    }
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
    // Q10'': 6 × OperatingPoint.values().length = 48 records under cycle C.
    Sequencer s = new Sequencer(0, 1, ALL_CLASSES);
    for (int phase = 0; phase < OperatingPoint.values().length; phase++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1000L);
    }
    assertTrue(s.isDone());

    List<BenchmarkRecord> records = s.extractRecords();
    int expectedSize = ALL_CLASSES.size() * OperatingPoint.values().length;
    assertEquals(expectedSize, records.size());

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
