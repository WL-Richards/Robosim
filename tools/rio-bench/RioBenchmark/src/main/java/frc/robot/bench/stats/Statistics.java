package frc.robot.bench.stats;

import java.util.Arrays;

public final class Statistics {
  private Statistics() {}

  public record Stats(double meanUs, double stddevUs, double p99Us, double outlierRate) {}

  public static Stats compute(long[] samplesNs, int blockSize) {
    if (samplesNs.length == 0) {
      throw new IllegalArgumentException("samplesNs must not be empty");
    }
    int n = samplesNs.length;

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

    long[] sorted = samplesNs.clone();
    Arrays.sort(sorted);
    double p99Ns = quantileR7(sorted, 0.99);

    int outliers = 0;
    if (stddevNs > 0.0) {
      double thresholdNs = mean + 5.0 * stddevNs;
      for (long s : samplesNs) {
        if ((double) s > thresholdNs) outliers++;
      }
    }
    double outlierRate = (double) outliers / (double) n;

    double perCallMeanNs = mean / blockSize;
    double perCallStddevNs = stddevNs / blockSize;
    double perCallP99Ns = p99Ns / blockSize;

    return new Stats(
        perCallMeanNs / 1000.0,
        perCallStddevNs / 1000.0,
        perCallP99Ns / 1000.0,
        outlierRate);
  }

  private static double quantileR7(long[] sorted, double p) {
    int n = sorted.length;
    if (n == 1) return (double) sorted[0];
    double h = (n - 1) * p + 1.0;
    int floorH = (int) Math.floor(h);
    double fraction = h - floorH;
    if (floorH >= n) return (double) sorted[n - 1];
    double lo = (double) sorted[floorH - 1];
    double hi = (floorH < n) ? (double) sorted[floorH] : lo;
    return lo + fraction * (hi - lo);
  }
}
