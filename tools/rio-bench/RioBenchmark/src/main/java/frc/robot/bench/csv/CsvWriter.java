package frc.robot.bench.csv;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Pure formatter for the rio-bench v0 CSV. Header lines start with
 * {@code "# "} and carry every field of {@link RunMetadata} as a
 * {@code key=value} pair so downstream tooling can recover the run
 * environment without an out-of-band manifest.
 */
public final class CsvWriter {
  private CsvWriter() {}

  /**
   * Renders metadata + records as the v0 CSV. Records are sorted by
   * {@link BenchmarkRecord#CSV_ORDER} so the byte-identical-output property
   * the project's determinism non-negotiable demands holds regardless of
   * insertion order.
   */
  public static String write(RunMetadata metadata, List<BenchmarkRecord> records) {
    StringBuilder out = new StringBuilder();

    out.append("# rio-bench v0\n");
    out.append("# wpilib_version=").append(metadata.wpilibVersion()).append('\n');
    out.append("# rio_firmware=").append(metadata.rioFirmware()).append('\n');
    out.append("# rio_image=").append(metadata.rioImage()).append('\n');
    out.append("# git_sha=").append(metadata.gitSha()).append('\n');
    out.append("# run_timestamp_iso8601=").append(metadata.runTimestampIso8601()).append('\n');
    out.append("# warmup_samples=").append(metadata.warmupSamples()).append('\n');
    out.append("# samples_per_block=").append(metadata.samplesPerBlock()).append('\n');
    out.append("# block_size=").append(metadata.blockSize()).append('\n');
    out.append("# validated=").append(metadata.validated() ? "true" : "false").append('\n');
    metadata
        .busDevices()
        .ifPresent(n -> out.append("# bus_devices=").append(n).append('\n'));

    out.append(
        "call_class,operating_point,sample_count,mean_us,stddev_us,p50_us,p99_us,p999_us,outlier_rate\n");

    List<BenchmarkRecord> sorted = new ArrayList<>(records);
    sorted.sort(BenchmarkRecord.CSV_ORDER);
    for (BenchmarkRecord r : sorted) {
      out.append(r.toCsvRow());
    }

    return out.toString();
  }

  /** Locale-stable mean-microseconds formatter; used by tests to assert column-level formatting. */
  static String formatMeanUs(double v) {
    return String.format(Locale.ROOT, "%.3f", v);
  }

  /** Locale-stable outlier-rate formatter; used by tests to assert column-level formatting. */
  static String formatOutlierRate(double v) {
    return String.format(Locale.ROOT, "%.6f", v);
  }
}
