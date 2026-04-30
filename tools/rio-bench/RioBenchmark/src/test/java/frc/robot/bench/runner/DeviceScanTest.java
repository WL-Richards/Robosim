package frc.robot.bench.runner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class DeviceScanTest {

  @Test
  void from_probe_results_counts_alive_devices() {
    boolean[] alive = {false, true, false, true, true, false};
    DeviceScan scan = DeviceScan.fromProbeResults(alive, 10);
    assertEquals(3, scan.aliveCount());
    assertEquals(List.of(11, 13, 14), scan.aliveIds());
    assertEquals(10, scan.probeRangeStartInclusive());
    assertEquals(15, scan.probeRangeEndInclusive());
  }

  @Test
  void from_probe_results_handles_empty_alive_set() {
    boolean[] alive = {false, false, false};
    DeviceScan scan = DeviceScan.fromProbeResults(alive, 0);
    assertEquals(0, scan.aliveCount());
    assertEquals(List.of(), scan.aliveIds());
    assertEquals(0, scan.probeRangeStartInclusive());
    assertEquals(2, scan.probeRangeEndInclusive());
  }

  @Test
  void from_probe_results_handles_full_alive_set() {
    boolean[] alive = {true, true, true};
    DeviceScan scan = DeviceScan.fromProbeResults(alive, 20);
    assertEquals(3, scan.aliveCount());
    assertEquals(List.of(20, 21, 22), scan.aliveIds());
    assertEquals(20, scan.probeRangeStartInclusive());
    assertEquals(22, scan.probeRangeEndInclusive());
  }

  @Test
  void from_probe_results_rejects_null_input() {
    assertThrows(NullPointerException.class, () -> DeviceScan.fromProbeResults(null, 0));
  }

  @Test
  void from_probe_results_rejects_negative_start() {
    assertThrows(
        IllegalArgumentException.class,
        () -> DeviceScan.fromProbeResults(new boolean[] {true}, -1));
  }

  @Test
  void record_constructor_rejects_unsorted_alive_ids() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(2, List.of(14, 11), 10, 15));
  }

  @Test
  void record_constructor_rejects_duplicate_alive_ids() {
    // {11, 11} is sorted non-strictly but not strictly.
    // A `<=` ascending check would accept it; the documented
    // invariant requires strict (`<`).
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(2, List.of(11, 11), 10, 15));
  }

  @Test
  void record_constructor_rejects_count_id_array_length_mismatch() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(2, List.of(11, 13, 14), 10, 15));
  }

  @Test
  void record_constructor_rejects_id_above_probe_range() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(1, List.of(20), 10, 15));
  }

  @Test
  void record_constructor_rejects_id_below_probe_range() {
    // Catches half-implemented range check (only upper bound checked).
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(1, List.of(9), 10, 15));
  }

  @Test
  void record_constructor_rejects_inverted_probe_range() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(0, List.of(), 15, 10));
  }

  @Test
  void record_constructor_rejects_negative_probe_range_start() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new DeviceScan(0, List.of(), -1, 5));
  }

  @Test
  void from_probe_results_rejects_zero_length_array() {
    assertThrows(
        IllegalArgumentException.class,
        () -> DeviceScan.fromProbeResults(new boolean[0], 5));
  }

  @Test
  void equal_field_values_yield_equal_records() {
    // Pins structural equality. If aliveIds were ever changed to int[],
    // the record's derived equals would silently fall back to reference
    // equality and `a.equals(b)` would be false here.
    DeviceScan a = new DeviceScan(2, List.of(11, 14), 10, 15);
    DeviceScan b = new DeviceScan(2, List.of(11, 14), 10, 15);
    DeviceScan c = new DeviceScan(2, List.of(11, 13), 10, 15);
    assertEquals(a, b);
    assertEquals(a.hashCode(), b.hashCode());
    assertNotEquals(a, c);
  }
}
