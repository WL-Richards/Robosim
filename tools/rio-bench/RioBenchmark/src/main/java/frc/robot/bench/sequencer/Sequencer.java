package frc.robot.bench.sequencer;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
import frc.robot.bench.stats.Statistics;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * State machine that walks a benchmark run through warmup → each operating
 * point in turn → done. Samples are collected per
 * (call class, operating point) pair. Warmup samples are counted but not
 * stored, so the JIT and CPU caches reach steady state before the recorded
 * portion of the sweep begins.
 *
 * <p>Not thread-safe. Designed for a single producer (the benchmark worker
 * thread).
 */
public final class Sequencer {
  private static final OperatingPoint[] PHASES = OperatingPoint.values();

  private final List<CallClass> callClasses;
  private final int warmupCount;
  private final int perPointCount;

  private int phaseIndex;
  private boolean done;
  private final Map<CallClass, Integer> phaseCounts;
  private final Map<CallClass, Map<OperatingPoint, long[]>> samples;
  private final Map<CallClass, Map<OperatingPoint, Integer>> sampleFill;

  /**
   * @param warmupCount samples per call class to discard before recording
   *     starts; 0 skips warmup
   * @param perPointCount samples per call class to record per operating point
   * @param callClasses the call classes the sweep will visit; must be non-empty
   * @throws IllegalArgumentException if {@code callClasses} is empty
   */
  public Sequencer(int warmupCount, int perPointCount, List<CallClass> callClasses) {
    if (callClasses.isEmpty()) {
      throw new IllegalArgumentException("callClasses must not be empty");
    }
    this.callClasses = List.copyOf(callClasses);
    this.warmupCount = warmupCount;
    this.perPointCount = perPointCount;

    // phaseIndex < 0 means "warming up"; if no warmup is requested we jump
    // straight to phase 0.
    this.phaseIndex = warmupCount == 0 ? 0 : -1;
    this.done = false;

    this.phaseCounts = new EnumMap<>(CallClass.class);
    for (CallClass c : callClasses) {
      phaseCounts.put(c, 0);
    }

    this.samples = new EnumMap<>(CallClass.class);
    this.sampleFill = new EnumMap<>(CallClass.class);
    for (CallClass c : callClasses) {
      EnumMap<OperatingPoint, long[]> perPoint = new EnumMap<>(OperatingPoint.class);
      EnumMap<OperatingPoint, Integer> perFill = new EnumMap<>(OperatingPoint.class);
      for (OperatingPoint p : PHASES) {
        perPoint.put(p, new long[perPointCount]);
        perFill.put(p, 0);
      }
      samples.put(c, perPoint);
      sampleFill.put(c, perFill);
    }
  }

  /** @return true while we are still in the warmup phase (samples discarded). */
  public boolean isWarmingUp() {
    return phaseIndex < 0 && !done;
  }

  /** @return true once every operating point has reached {@code perPointCount} samples for every call class. */
  public boolean isDone() {
    return done;
  }

  /**
   * @return the operating point currently being recorded
   * @throws IllegalStateException during warmup or after {@link #isDone()}
   */
  public OperatingPoint currentPoint() {
    if (isWarmingUp()) {
      throw new IllegalStateException("currentPoint() is undefined during warmup");
    }
    if (done) {
      throw new IllegalStateException("currentPoint() is undefined after done");
    }
    return PHASES[phaseIndex];
  }

  /**
   * Submits one timed block. During warmup the sample is counted but not
   * stored; during a recorded operating point it is stored under the
   * current phase. When every call class has reached the phase-completion
   * threshold the sequencer auto-advances.
   *
   * @param callClass the call class this block measured
   * @param sampleNs total nanoseconds for the block (per-call division
   *     happens later in the statistics stage)
   * @throws IllegalStateException if the sequencer is already done
   * @throws IllegalArgumentException if {@code callClass} was not declared
   *     at construction
   */
  public void submitSample(CallClass callClass, long sampleNs) {
    if (done) {
      throw new IllegalStateException("Sequencer is done; further samples are rejected");
    }

    Integer count = phaseCounts.get(callClass);
    if (count == null) {
      throw new IllegalArgumentException("Unknown call class: " + callClass);
    }

    if (isWarmingUp()) {
      int next = count + 1;
      phaseCounts.put(callClass, next);
      if (allReached(warmupCount)) {
        advancePhase();
      }
    } else {
      OperatingPoint point = PHASES[phaseIndex];
      Map<OperatingPoint, Integer> fillByPoint = sampleFill.get(callClass);
      int fill = fillByPoint.get(point);
      samples.get(callClass).get(point)[fill] = sampleNs;
      fillByPoint.put(point, fill + 1);

      int next = count + 1;
      phaseCounts.put(callClass, next);
      if (allReached(perPointCount)) {
        advancePhase();
      }
    }
  }

  /**
   * @return a defensive copy of the recorded samples for one
   *     (call class, operating point) pair
   * @throws IllegalStateException if the sweep is not yet complete
   */
  public long[] samplesFor(CallClass callClass, OperatingPoint operatingPoint) {
    if (!done) {
      throw new IllegalStateException("samplesFor() requires Sequencer to be done");
    }

    long[] full = samples.get(callClass).get(operatingPoint);
    int fill = sampleFill.get(callClass).get(operatingPoint);
    long[] out = new long[fill];
    System.arraycopy(full, 0, out, 0, fill);
    return out;
  }

  /**
   * Builds the per-(call class, operating point) {@link BenchmarkRecord}
   * list once the sweep is complete. Each record carries already-computed
   * statistics so the sink/serializer stage stays pure formatting.
   *
   * @throws IllegalStateException if the sweep is not yet complete
   */
  public List<BenchmarkRecord> extractRecords() {
    if (!done) {
      throw new IllegalStateException("extractRecords() requires Sequencer to be done");
    }

    List<BenchmarkRecord> out = new ArrayList<>();
    for (CallClass c : callClasses) {
      for (OperatingPoint p : PHASES) {
        long[] s = samplesFor(c, p);
        out.add(new BenchmarkRecord(c, p, s.length, Statistics.compute(s, c.blockSize())));
      }
    }
    return out;
  }

  /** @return true iff every call class's per-phase count has reached {@code threshold}. */
  private boolean allReached(int threshold) {
    for (int v : phaseCounts.values()) {
      if (v < threshold) {
        return false;
      }
    }
    return true;
  }

  /**
   * Resets per-call-class counts and moves to the next phase, or marks
   * the run done if no phases remain.
   */
  private void advancePhase() {
    phaseIndex++;
    for (CallClass c : callClasses) {
      phaseCounts.put(c, 0);
    }
    if (phaseIndex >= PHASES.length) {
      done = true;
    }
  }
}
