package frc.robot.bench.stats;

import java.util.Arrays;

/**
 * Computes the per-cell summary statistics emitted in the CSV.
 *
 * <p>Mean and stddev use Welford's online algorithm (single pass,
 * numerically stable for the long tails HAL latency exhibits). Percentiles
 * (p50, p99, p999) all use the R-7 (linear-interpolation) quantile
 * definition, which matches numpy's default and pandas' {@code quantile()}
 * so cross-validation with notebook tooling agrees byte-for-byte.
 *
 * <p>Outliers are anything more than 5σ above the mean; the rate is
 * reported as part of the row so reviewers can flag cells where the
 * mean / p99 may have been pulled by a small number of pathological
 * samples.
 *
 * <p>Cycle B added p50 and p999 alongside the existing p99 — the tier-1
 * scheduler-jitter model needs more than a single tail point to reproduce
 * the {@code HAL_NOTIFIER_WAIT} jitter distribution. The R-7 helper was
 * lifted to {@link #percentileNs(long[], double)} so it is testable
 * directly at any {@code p}, not just transitively via {@link #compute}.
 */
public final class Statistics {
  private Statistics() {}

  /**
   * Per-call (not per-block) summary in microseconds plus the
   * dimensionless outlier rate.
   *
   * <p>Field order is part of the {@link
   * frc.robot.bench.csv.BenchmarkRecord#toCsvRow} contract — a swap
   * between {@code p50Us} and {@code p999Us} would compile but silently
   * corrupt every row of the output.
   *
   * @param meanUs Welford mean of per-call latency, in μs
   * @param stddevUs Welford sample stddev of per-call latency, in μs
   * @param p50Us 50th-percentile per-call latency (R-7), in μs
   * @param p99Us 99th-percentile per-call latency (R-7), in μs
   * @param p999Us 99.9th-percentile per-call latency (R-7), in μs
   * @param outlierRate fraction of raw block samples > mean + 5σ; in [0, 1]
   */
  public record Stats(
      double meanUs,
      double stddevUs,
      double p50Us,
      double p99Us,
      double p999Us,
      double outlierRate) {}

  /**
   * Folds {@code samplesNs} into a per-call {@link Stats} summary.
   *
   * @param samplesNs per-block elapsed times in nanoseconds
   * @param blockSize number of HAL calls each block contains; the returned
   *     stats are divided by this to report per-call latency
   * @throws IllegalArgumentException if {@code samplesNs} is empty
   */
  public static Stats compute(long[] samplesNs, int blockSize) {
    if (samplesNs.length == 0) {
      throw new IllegalArgumentException("samplesNs must not be empty");
    }
    int n = samplesNs.length;

    // Welford one-pass mean / variance.
    double mean = 0.0;
    double m2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = (double) samplesNs[i];
      double delta = x - mean;
      mean += delta / (i + 1);
      m2 += delta * (x - mean);
    }
    double sampleVarianceNs = (n > 1) ? (m2 / (n - 1)) : 0.0;
    double stddevNs = Math.sqrt(sampleVarianceNs);

    // R-7 quantile requires sorted input; clone so we do not mutate the caller's array.
    long[] sorted = samplesNs.clone();
    Arrays.sort(sorted);
    double p50Ns = percentileNs(sorted, 0.50);
    double p99Ns = percentileNs(sorted, 0.99);
    double p999Ns = percentileNs(sorted, 0.999);

    int outliers = 0;
    if (stddevNs > 0.0) {
      double thresholdNs = mean + 5.0 * stddevNs;
      for (long s : samplesNs) {
        if ((double) s > thresholdNs) {
          outliers++;
        }
      }
    }
    double outlierRate = (double) outliers / (double) n;

    double inv = 1.0 / blockSize;
    return new Stats(
        (mean * inv) / 1000.0,
        (stddevNs * inv) / 1000.0,
        (p50Ns * inv) / 1000.0,
        (p99Ns * inv) / 1000.0,
        (p999Ns * inv) / 1000.0,
        outlierRate);
  }

  /**
   * R-7 (linear-interpolation) quantile, matching numpy's default. Chosen
   * for cross-validation against notebook analysis tooling.
   *
   * <p>Package-private so {@code StatisticsTest} can pin behavior at
   * arbitrary {@code p} (S13 covers {@code p ∈ {0.0, 0.5, 0.99, 0.999, 1.0}}).
   * Cycle B lifted this from the previous private helper {@code quantileR7}.
   *
   * @param sorted ascending-sorted samples; {@code n >= 1}
   * @param p quantile in [0, 1]
   */
  static double percentileNs(long[] sorted, double p) {
    int n = sorted.length;
    if (n == 1) {
      return (double) sorted[0];
    }
    double h = (n - 1) * p + 1.0;
    int floorH = (int) Math.floor(h);
    double fraction = h - floorH;
    if (floorH >= n) {
      return (double) sorted[n - 1];
    }
    double lo = (double) sorted[floorH - 1];
    double hi = (floorH < n) ? (double) sorted[floorH] : lo;
    return lo + fraction * (hi - lo);
  }
}
