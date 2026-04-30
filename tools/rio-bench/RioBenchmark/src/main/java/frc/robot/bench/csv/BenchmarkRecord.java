package frc.robot.bench.csv;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.stats.Statistics.Stats;
import java.util.Comparator;
import java.util.Locale;

/**
 * One row of the output CSV — the result for a single
 * (call class, operating point) cell of the sweep.
 *
 * @param callClass the HAL call class this row reports on
 * @param operatingPoint the bus / CPU scenario active when these samples
 *     were taken
 * @param sampleCount the number of timing blocks that contributed to
 *     {@code stats}; typically equal to {@code samples_per_block} from the
 *     metadata, unless the sweep was truncated
 * @param stats per-call mean / stddev / p99 in microseconds plus the
 *     outlier rate
 */
public record BenchmarkRecord(
    CallClass callClass,
    OperatingPoint operatingPoint,
    int sampleCount,
    Stats stats)
    implements Comparable<BenchmarkRecord> {

  /**
   * Total ordering used both for the natural {@link #compareTo} and to
   * pre-sort rows before CSV emission. The order is (call class label,
   * operating point label), so the file is deterministic and diffable
   * across runs regardless of insertion order.
   */
  public static final Comparator<BenchmarkRecord> CSV_ORDER =
      Comparator.<BenchmarkRecord, String>comparing(r -> r.callClass.csvLabel())
          .thenComparing(r -> r.operatingPoint.csvLabel());

  @Override
  public int compareTo(BenchmarkRecord other) {
    return CSV_ORDER.compare(this, other);
  }

  /**
   * @return a single CSV row with a trailing newline, formatted in the
   *     {@code ROOT} locale so decimal separators do not depend on the
   *     RIO's locale settings
   */
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
