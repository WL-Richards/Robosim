package frc.robot.bench.runner;

import java.util.ArrayList;
import java.util.List;

/**
 * Snapshot of which CAN device IDs were alive on the bus when the bench
 * sweep began. The sim core's tier-1 backend uses this baseline to
 * interpret per-row HAL-call cost in context: software-spam load tiers
 * sit on top of whatever real-device traffic was already present.
 *
 * <p>Cycle C records the count and ID list in the CSV preamble (count) +
 * DataLog (full ID list at boot). The pure-Java factory
 * {@link #fromProbeResults(boolean[], int)} consumes a HAL-bound probe
 * result; the HAL probe itself
 * ({@code CallBindings.probeTalonFxAlive(int, int)}) is not unit-tested
 * per {@code D-RB-7}.
 *
 * <p><strong>Field type for {@code aliveIds}: {@code List<Integer>} (not
 * {@code int[]}).</strong> Java {@code record} types synthesize {@code
 * equals} / {@code hashCode} via per-field {@code Objects.equals}, which
 * falls back to reference equality on arrays. Using {@code int[]} would
 * silently break structural equality for downstream consumers. {@code
 * List<Integer>} gives the record correct derived equality. The list is
 * immutable; both the canonical constructor and the factory wrap any
 * caller-supplied list with {@link List#copyOf(java.util.Collection)}
 * before assigning.
 *
 * @param aliveCount number of devices found alive; must equal
 *     {@code aliveIds.size()}; must be {@code >= 0}
 * @param aliveIds CAN device IDs found alive, sorted strictly ascending,
 *     each in {@code [probeRangeStartInclusive, probeRangeEndInclusive]}
 * @param probeRangeStartInclusive first CAN ID the probe scanned;
 *     must be {@code >= 0}
 * @param probeRangeEndInclusive last CAN ID the probe scanned;
 *     must be {@code >= probeRangeStartInclusive}
 */
public record DeviceScan(
    int aliveCount,
    List<Integer> aliveIds,
    int probeRangeStartInclusive,
    int probeRangeEndInclusive) {

  /**
   * Canonical constructor with invariant checking and a defensive copy
   * of {@code aliveIds}.
   *
   * @throws IllegalArgumentException if any documented invariant fails
   * @throws NullPointerException if {@code aliveIds} is null
   */
  public DeviceScan {
    if (aliveIds == null) {
      throw new NullPointerException("aliveIds must not be null");
    }
    if (aliveCount < 0) {
      throw new IllegalArgumentException("aliveCount must be >= 0; got " + aliveCount);
    }
    if (aliveCount != aliveIds.size()) {
      throw new IllegalArgumentException(
          "aliveCount (" + aliveCount + ") must equal aliveIds.size() (" + aliveIds.size() + ")");
    }
    if (probeRangeStartInclusive < 0) {
      throw new IllegalArgumentException(
          "probeRangeStartInclusive must be >= 0; got " + probeRangeStartInclusive);
    }
    if (probeRangeStartInclusive > probeRangeEndInclusive) {
      throw new IllegalArgumentException(
          "probeRangeStartInclusive ("
              + probeRangeStartInclusive
              + ") must be <= probeRangeEndInclusive ("
              + probeRangeEndInclusive
              + ")");
    }
    for (int i = 0; i < aliveIds.size(); i++) {
      int id = aliveIds.get(i);
      if (id < probeRangeStartInclusive || id > probeRangeEndInclusive) {
        throw new IllegalArgumentException(
            "aliveIds["
                + i
                + "]="
                + id
                + " is outside probe range ["
                + probeRangeStartInclusive
                + ", "
                + probeRangeEndInclusive
                + "]");
      }
      if (i > 0 && aliveIds.get(i - 1) >= id) {
        throw new IllegalArgumentException(
            "aliveIds must be sorted strictly ascending; got "
                + aliveIds.get(i - 1)
                + " followed by "
                + id);
      }
    }
    // Defensive copy: callers cannot mutate aliveIds after construction.
    aliveIds = List.copyOf(aliveIds);
  }

  /**
   * Builds a {@link DeviceScan} from a boolean array of probe results.
   * Index {@code i} of {@code aliveByIndex} is "is device with ID
   * {@code (probeRangeStart + i)} alive?".
   *
   * @throws IllegalArgumentException if {@code aliveByIndex} is empty
   *     (degenerate range) or {@code probeRangeStart < 0}
   * @throws NullPointerException if {@code aliveByIndex} is null
   */
  public static DeviceScan fromProbeResults(boolean[] aliveByIndex, int probeRangeStart) {
    if (aliveByIndex.length == 0) {
      throw new IllegalArgumentException(
          "aliveByIndex must not be empty (degenerate probe range)");
    }
    int probeRangeEnd = probeRangeStart + aliveByIndex.length - 1;
    List<Integer> ids = new ArrayList<>();
    for (int i = 0; i < aliveByIndex.length; i++) {
      if (aliveByIndex[i]) {
        ids.add(probeRangeStart + i);
      }
    }
    return new DeviceScan(ids.size(), ids, probeRangeStart, probeRangeEnd);
  }
}
