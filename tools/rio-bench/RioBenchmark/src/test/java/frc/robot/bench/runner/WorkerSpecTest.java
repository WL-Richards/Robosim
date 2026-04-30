package frc.robot.bench.runner;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.bench.sequencer.OperatingPoint;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class WorkerSpecTest {

  @Test
  void forPoint_idle_bus_returns_no_workers() {
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.IDLE_BUS);
    assertEquals(0, spec.cpuWorkers());
    assertEquals(0, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_bus_returns_one_can_spam_worker() {
    // Cycle-A semantics (canSpam=true) translates to 1 spam worker
    // under cycle C's int field. A regression to v0's hardware-driven
    // semantics would silently set this back to 0 and collapse onto
    // IDLE_BUS.
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_BUS);
    assertEquals(0, spec.cpuWorkers());
    assertEquals(1, spec.canSpamWorkers());
  }

  @Test
  void forPoint_multi_thread_returns_three_cpu_workers_no_can_spam() {
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.MULTI_THREAD);
    assertEquals(3, spec.cpuWorkers());
    assertEquals(0, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_multi_thread_returns_three_cpu_workers_and_one_can_spam() {
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_MULTI_THREAD);
    assertEquals(3, spec.cpuWorkers());
    assertEquals(1, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_bus_4_returns_four_can_spam_workers() {
    // A clamping defect that preserves cycle-A's boolean semantics
    // (canSpamWorkers ∈ {0, 1}) would silently return (0, 1) for this
    // op-point and collapse it onto SATURATED_BUS.
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_BUS_4);
    assertEquals(0, spec.cpuWorkers());
    assertEquals(4, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_bus_8_returns_eight_can_spam_workers() {
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_BUS_8);
    assertEquals(0, spec.cpuWorkers());
    assertEquals(8, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_bus_4_multi_thread_returns_three_cpu_and_four_spam() {
    // Cross-product cell. Forgetting cpuWorkers=3 collapses onto
    // SATURATED_BUS_4; forgetting canSpamWorkers=4 collapses onto
    // MULTI_THREAD.
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_BUS_4_MULTI_THREAD);
    assertEquals(3, spec.cpuWorkers());
    assertEquals(4, spec.canSpamWorkers());
  }

  @Test
  void forPoint_saturated_bus_8_multi_thread_returns_three_cpu_and_eight_spam() {
    WorkerSpec spec = WorkerSpec.forPoint(OperatingPoint.SATURATED_BUS_8_MULTI_THREAD);
    assertEquals(3, spec.cpuWorkers());
    assertEquals(8, spec.canSpamWorkers());
  }

  @Test
  void forPoint_returns_distinct_spec_for_every_operating_point() {
    // Cross-product distinctness: all 8 operating points must produce
    // 8 different load profiles. Set membership relies on Java record
    // equals/hashCode being field-derived (correct because both fields
    // are primitive int).
    Set<WorkerSpec> specs = new HashSet<>();
    for (OperatingPoint p : OperatingPoint.values()) {
      specs.add(WorkerSpec.forPoint(p));
    }
    assertEquals(
        OperatingPoint.values().length,
        specs.size(),
        "all "
            + OperatingPoint.values().length
            + " operating points must map to distinct WorkerSpecs; got "
            + Arrays.toString(specs.toArray()));
  }
}
