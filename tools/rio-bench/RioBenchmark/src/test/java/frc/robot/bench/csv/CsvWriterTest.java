package frc.robot.bench.csv;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.bench.calls.CallClass;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.stats.Statistics.Stats;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class CsvWriterTest {

  private static RunMetadata defaultMetadata() {
    return new RunMetadata(
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
  }

  @Test
  void writes_preamble_lines_with_correct_order_and_values() {
    String out = CsvWriter.write(defaultMetadata(), List.of());
    String expected =
        """
        # rio-bench v0
        # wpilib_version=2026.2.1
        # rio_firmware=9.0.0
        # rio_image=2026_v3.0
        # git_sha=abc123def456
        # run_timestamp_iso8601=2026-04-29T20:00:00Z
        # warmup_samples=1000
        # samples_per_block=10000
        # block_size=100
        # validated=false
        call_class,operating_point,sample_count,mean_us,stddev_us,p99_us,outlier_rate
        """;
    assertEquals(expected, out);
  }

  @Test
  void writes_header_row_after_preamble() {
    String out = CsvWriter.write(defaultMetadata(), List.of());
    String[] lines = out.split("\n");
    assertEquals(
        "call_class,operating_point,sample_count,mean_us,stddev_us,p99_us,outlier_rate",
        lines[10]);
  }

  @Test
  void sorts_records_by_call_class_then_operating_point() {
    Stats s = new Stats(0.0, 0.0, 0.0, 0.0);
    List<BenchmarkRecord> records =
        List.of(
            new BenchmarkRecord(CallClass.CAN_FRAME_WRITE, OperatingPoint.SATURATED_BUS, 1, s),
            new BenchmarkRecord(CallClass.CAN_FRAME_WRITE, OperatingPoint.IDLE_BUS, 1, s),
            new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.SATURATED_BUS, 1, s),
            new BenchmarkRecord(CallClass.CAN_FRAME_READ, OperatingPoint.IDLE_BUS, 1, s),
            new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.SATURATED_BUS, 1, s),
            new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s));
    String out = CsvWriter.write(defaultMetadata(), records);
    String[] lines = out.split("\n");
    int header = 10;
    assertTrue(lines[header + 1].startsWith("can_frame_read,idle_bus,"));
    assertTrue(lines[header + 2].startsWith("can_frame_read,saturated_bus,"));
    assertTrue(lines[header + 3].startsWith("can_frame_write,idle_bus,"));
    assertTrue(lines[header + 4].startsWith("can_frame_write,saturated_bus,"));
    assertTrue(lines[header + 5].startsWith("hal_get_fpga_time,idle_bus,"));
    assertTrue(lines[header + 6].startsWith("hal_get_fpga_time,saturated_bus,"));
  }

  @Test
  void formats_microsecond_columns_with_locale_root_half_up() {
    Stats s1 = new Stats(1.2345678, 0.0, 0.0, 0.0);
    BenchmarkRecord r1 =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s1);
    String out1 = CsvWriter.write(defaultMetadata(), List.of(r1));
    assertTrue(out1.contains(",1.235,"), "expected '1.235' in output, got:\n" + out1);

    Stats s2 = new Stats(1.0625, 0.0, 0.0, 0.0);
    BenchmarkRecord r2 =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s2);
    String out2 = CsvWriter.write(defaultMetadata(), List.of(r2));
    assertTrue(out2.contains(",1.063,"), "expected '1.063' (HALF_UP), got:\n" + out2);
    assertFalse(out2.contains(",1.062,"), "must not produce '1.062' (HALF_EVEN)");
  }

  @Test
  void formats_outlier_rate_with_six_decimal_places() {
    Stats s1 = new Stats(0.0, 0.0, 0.0, 1e-6);
    BenchmarkRecord r1 =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s1);
    String out1 = CsvWriter.write(defaultMetadata(), List.of(r1));
    assertTrue(out1.contains(",0.000001\n"), "expected '0.000001' tail, got:\n" + out1);

    Stats s2 = new Stats(0.0, 0.0, 0.0, 0.0);
    BenchmarkRecord r2 =
        new BenchmarkRecord(CallClass.HAL_GET_FPGA_TIME, OperatingPoint.IDLE_BUS, 1, s2);
    String out2 = CsvWriter.write(defaultMetadata(), List.of(r2));
    assertTrue(out2.contains(",0.000000\n"), "expected '0.000000' tail, got:\n" + out2);
  }

  @Test
  void unvalidated_flag_renders_as_lowercase_false() {
    String out = CsvWriter.write(defaultMetadata(), List.of());
    assertTrue(out.contains("# validated=false\n"));
  }

  @Test
  void validated_flag_renders_as_lowercase_true() {
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
            true,
            Optional.empty());
    String out = CsvWriter.write(meta, List.of());
    assertTrue(out.contains("# validated=true\n"));
  }

  @Test
  void bus_devices_line_omitted_when_count_is_full() {
    String out = CsvWriter.write(defaultMetadata(), List.of());
    assertFalse(out.contains("# bus_devices="));
  }

  @Test
  void bus_devices_line_present_when_count_is_short() {
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
            Optional.of(2));
    String out = CsvWriter.write(meta, List.of());
    int first = out.indexOf("# bus_devices=2\n");
    int last = out.lastIndexOf("# bus_devices=2\n");
    assertTrue(first >= 0, "bus_devices line missing");
    assertEquals(first, last, "bus_devices line emitted more than once");
  }

  @Test
  void empty_record_list_writes_preamble_and_header_only() {
    String out = CsvWriter.write(defaultMetadata(), List.of());
    long lineCount = out.chars().filter(c -> c == '\n').count();
    assertEquals(11L, lineCount);
  }
}
