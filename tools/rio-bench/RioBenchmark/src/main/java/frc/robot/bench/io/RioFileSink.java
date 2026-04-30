package frc.robot.bench.io;

import frc.robot.bench.csv.RunMetadata;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public final class RioFileSink {
  private static final Path RIO_BENCH_DIR = Path.of("/home/lvuser/rio-bench");

  private RioFileSink() {}

  public static Path defaultPathFor(RunMetadata metadata) {
    String filename = metadata.wpilibVersion() + "-" + metadata.rioFirmware() + ".csv";
    return RIO_BENCH_DIR.resolve(filename);
  }

  public static void write(Path path, String csv) throws IOException {
    Files.createDirectories(path.getParent());
    Files.writeString(path, csv, StandardCharsets.UTF_8);
  }
}
