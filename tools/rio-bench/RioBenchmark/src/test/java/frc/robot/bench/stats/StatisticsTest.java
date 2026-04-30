package frc.robot.bench.stats;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.bench.stats.Statistics.Stats;
import java.util.Random;
import org.junit.jupiter.api.Test;

class StatisticsTest {

  @Test
  void mean_of_constant_series_equals_constant() {
    long[] samples = new long[1000];
    for (int i = 0; i < samples.length; i++) samples[i] = 1000L;
    Stats s = Statistics.compute(samples, 1);
    assertEquals(1.0, s.meanUs(), 0.0);
  }

  @Test
  void stddev_of_constant_series_is_zero() {
    long[] samples = new long[100];
    for (int i = 0; i < samples.length; i++) samples[i] = 5000L;
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0, s.stddevUs(), 1e-12);
  }

  @Test
  void mean_matches_textbook_for_known_dataset() {
    long[] samples = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0055, s.meanUs(), 1e-9);
  }

  @Test
  void stddev_uses_sample_bessel_correction() {
    long[] samples = {2, 4, 4, 4, 5, 5, 7, 9};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0021380899, s.stddevUs(), 1e-9);
    assertTrue(
        Math.abs(s.stddevUs() - 0.002000) > 1e-6,
        "must reject the population (N divisor) value 0.002000");
  }

  @Test
  void p99_uses_linear_interpolation_between_ranked_samples() {
    long[] samples = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0991, s.p99Us(), 1e-12);
  }

  @Test
  void p99_of_three_samples_well_defined_under_R7() {
    long[] samples = {10, 20, 30};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0298, s.p99Us(), 1e-12);
  }

  @Test
  void outlier_rate_zero_for_pure_constant_series() {
    long[] samples = new long[1000];
    for (int i = 0; i < samples.length; i++) samples[i] = 12345L;
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.0, s.outlierRate(), 0.0);
  }

  @Test
  void outlier_rate_counts_samples_above_mean_plus_five_stddev() {
    Random rng = new Random(42);
    long[] samples = new long[1000];
    for (int i = 0; i < 999; i++) {
      samples[i] = 100L + (long) (rng.nextDouble() * 100.0);
    }
    samples[999] = 100000L;
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.001, s.outlierRate(), 0.0);
  }

  @Test
  void block_timing_divides_block_total_by_block_size() {
    long[] samples = new long[10];
    for (int i = 0; i < samples.length; i++) samples[i] = 10000L;
    Stats s = Statistics.compute(samples, 100);
    assertEquals(0.1, s.meanUs(), 1e-12);
  }

  @Test
  void empty_series_throws_illegal_argument() {
    IllegalArgumentException e =
        assertThrows(IllegalArgumentException.class, () -> Statistics.compute(new long[0], 1));
    assertTrue(e.getMessage().toLowerCase().contains("empty"));
  }

  @Test
  void single_sample_yields_zero_stddev_and_mean_equals_value() {
    long[] samples = {7000L};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(7.0, s.meanUs(), 0.0);
    assertEquals(0.0, s.stddevUs(), 0.0);
    assertEquals(7.0, s.p50Us(), 0.0);
    assertEquals(7.0, s.p99Us(), 0.0);
    assertEquals(7.0, s.p999Us(), 0.0);
    assertEquals(0.0, s.outlierRate(), 0.0);
  }

  @Test
  void large_sample_count_does_not_lose_precision() {
    long[] samples = new long[1_000_000];
    for (int i = 0; i < samples.length; i++) samples[i] = 1234L;
    Stats s = Statistics.compute(samples, 1);
    assertEquals(1.234, s.meanUs(), 1e-9);
    assertEquals(0.0, s.stddevUs(), 1e-9);
  }

  @Test
  void percentile_ns_helper_returns_R7_value_for_canonical_dataset() {
    // S13: directly pin the lifted percentileNs helper across boundary p
    // values. Catches a refactor that picks nearest-rank instead of R-7,
    // and an off-by-one at p=1.0 that would access sorted[n].
    long[] sorted = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    assertEquals(10.0, Statistics.percentileNs(sorted, 0.0), 1e-12);
    assertEquals(55.0, Statistics.percentileNs(sorted, 0.5), 1e-12);
    assertEquals(99.1, Statistics.percentileNs(sorted, 0.99), 1e-12);
    assertEquals(99.91, Statistics.percentileNs(sorted, 0.999), 1e-12);
    assertEquals(100.0, Statistics.percentileNs(sorted, 1.0), 1e-12);
  }

  @Test
  void compute_returns_p50_via_R7_on_known_dataset() {
    // S14: pins compute → percentileNs(., 0.5) wiring. Distinct from the
    // existing p99 test (S5) so an argument-collision defect (passing the
    // same p to both) is caught.
    long[] samples = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.055, s.p50Us(), 1e-12);
  }

  @Test
  void compute_returns_p999_via_R7_on_known_dataset() {
    // S15: pins compute → percentileNs(., 0.999) wiring.
    long[] samples = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.09991, s.p999Us(), 1e-12);
  }

  @Test
  void compute_p50_and_p999_respect_block_size_division() {
    // S16: per-call division must apply to all three percentiles, not
    // just p99. Identical samples force p50 = p99 = p999 = mean exactly.
    long[] samples = new long[10];
    for (int i = 0; i < samples.length; i++) samples[i] = 10000L;
    Stats s = Statistics.compute(samples, 100);
    assertEquals(0.1, s.p50Us(), 1e-12);
    assertEquals(0.1, s.p999Us(), 1e-12);
  }

  @Test
  void compute_p999_well_defined_for_three_sample_input() {
    // S17: small-N at p=0.999. Floor/fraction near the last index must
    // not throw or return NaN. v0 S6 covers the same dataset at p=0.99
    // and expects 0.0298; this expects 0.02998 (not the same value —
    // do not confuse the two).
    long[] samples = {10, 20, 30};
    Stats s = Statistics.compute(samples, 1);
    assertEquals(0.02998, s.p999Us(), 1e-12);
  }
}
