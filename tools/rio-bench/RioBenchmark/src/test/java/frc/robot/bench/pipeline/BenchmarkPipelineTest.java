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
    for (OperatingPoint p : OperatingPoint.values()) {
      long valueNs = (long) (6 * p.ordinal()) * 1000L;
      for (int round = 0; round < PER_POINT_COUNT; round++) {
        for (CallClass c : ALL_CLASSES) {
          long sampleNs = valueNs + (long) c.ordinal() * 1000L;
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
