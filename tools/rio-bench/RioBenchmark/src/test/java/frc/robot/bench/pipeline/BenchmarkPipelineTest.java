package frc.robot.bench.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
import frc.robot.bench.csv.CsvWriter;
import frc.robot.bench.csv.RunMetadata;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.sequencer.Sequencer;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class BenchmarkPipelineTest {

  private static final List<CallClass> ALL_CLASSES = List.of(CallClass.values());
  private static final int WARMUP_COUNT = 1;
  private static final int PER_POINT_COUNT = 4;

  @Test
  void pipeline_produces_documented_csv_for_synthetic_run() throws Exception {
    Sequencer s = new Sequencer(WARMUP_COUNT, PER_POINT_COUNT, ALL_CLASSES);

    for (int round = 0; round < WARMUP_COUNT; round++) {
      for (CallClass c : ALL_CLASSES) s.submitSample(c, 1L);
    }
    // Cycle B P1'': v = ((6*p.ordinal() + c.ordinal()) + 1) * 1000.
    // The +1 offset keeps the lowest cell's `v - 1` sample non-negative
    // (samples are timing-style ns values, so negative is semantically
    // wrong even though Statistics.compute does not reject it).
    // For each cell, submit the four-sample spread [v-1, v, v, v+1] so
    // that p50, p99, p999 differ at nanosecond precision (per cycle-B
    // bug-class P1''-1: distinguishes p50 from p99 in the byte-equal
    // fixture for blockSize=1 cells).
    for (OperatingPoint p : OperatingPoint.values()) {
      for (int round = 0; round < PER_POINT_COUNT; round++) {
        for (CallClass c : ALL_CLASSES) {
          long valueNs = (long) ((6 * p.ordinal() + c.ordinal()) + 1) * 1000L;
          long sampleNs;
          switch (round) {
            case 0 -> sampleNs = valueNs - 1L;
            case 3 -> sampleNs = valueNs + 1L;
            default -> sampleNs = valueNs;
          }
          s.submitSample(c, sampleNs);
        }
      }
    }

    List<BenchmarkRecord> records = s.extractRecords();

    RunMetadata meta =
        new RunMetadata(
            "2026.2.1",
            "9.0.0",
            "2026_v3.0",
            "abc123def456",
            "2026-04-29T20:00:00Z",
            1000,
            10000,
            100,
            false,
            Optional.empty());

    String actual = CsvWriter.write(meta, records);

    String expected;
    try (InputStream in =
        BenchmarkPipelineTest.class.getResourceAsStream("/pipeline_expected.csv")) {
      assertNotNull(in, "fixture pipeline_expected.csv must be on the classpath");
      expected = new String(in.readAllBytes(), StandardCharsets.UTF_8);
    }

    assertEquals(expected, actual);
  }
}
