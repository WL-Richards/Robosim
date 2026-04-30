package frc.robot.bench.csv;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.stats.Statistics.Stats;
import java.util.Comparator;
import java.util.Locale;

public record BenchmarkRecord(
    CallClass callClass,
    OperatingPoint operatingPoint,
    int sampleCount,
    Stats stats)
    implements Comparable<BenchmarkRecord> {

  public static final Comparator<BenchmarkRecord> CSV_ORDER =
      Comparator.<BenchmarkRecord, String>comparing(r -> r.callClass.csvLabel())
          .thenComparing(r -> r.operatingPoint.csvLabel());

  @Override
  public int compareTo(BenchmarkRecord other) {
    return CSV_ORDER.compare(this, other);
  }

  public String toCsvRow() {
    return String.format(
        Locale.ROOT,
        "%s,%s,%d,%.3f,%.3f,%.3f,%.6f\n",
        callClass.csvLabel(),
        operatingPoint.csvLabel(),
        sampleCount,
        stats.meanUs(),
        stats.stddevUs(),
        stats.p99Us(),
        stats.outlierRate());
  }
}
