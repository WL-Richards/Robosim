package frc.robot.bench.sequencer;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
import frc.robot.bench.stats.Statistics;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

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

  public Sequencer(int warmupCount, int perPointCount, List<CallClass> callClasses) {
    if (callClasses.isEmpty()) {
      throw new IllegalArgumentException("callClasses must not be empty");
    }
    this.callClasses = List.copyOf(callClasses);
    this.warmupCount = warmupCount;
    this.perPointCount = perPointCount;
    this.phaseIndex = warmupCount == 0 ? 0 : -1;
    this.done = false;
    this.phaseCounts = new EnumMap<>(CallClass.class);
    for (CallClass c : callClasses) phaseCounts.put(c, 0);
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

  public boolean isWarmingUp() {
    return phaseIndex < 0 && !done;
  }

  public boolean isDone() {
    return done;
  }

  public OperatingPoint currentPoint() {
    if (isWarmingUp()) {
      throw new IllegalStateException("currentPoint() is undefined during warmup");
    }
    if (done) {
      throw new IllegalStateException("currentPoint() is undefined after done");
    }
    return PHASES[phaseIndex];
  }

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

  private boolean allReached(int threshold) {
    for (int v : phaseCounts.values()) {
      if (v < threshold) return false;
    }
    return true;
  }

  private void advancePhase() {
    phaseIndex++;
    for (CallClass c : callClasses) phaseCounts.put(c, 0);
    if (phaseIndex >= PHASES.length) {
      done = true;
    }
  }
}
