package frc.robot.bench.io;

import frc.robot.bench.csv.RunMetadata;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Writes the rio-bench CSV to local disk.
 *
 * <p>The output directory is the relative path {@code rio-bench/}, resolved
 * against the JVM's working directory. On a real RoboRIO the
 * {@code frcRunRobot.sh} launcher sets cwd to {@code /home/lvuser}, so
 * files still land at {@code /home/lvuser/rio-bench/} — which the operator
 * pull script expects and which survives a re-deploy. Off-RIO (desktop
 * sim, CI, dev machine) the same path lands under whichever directory the
 * caller launched from, instead of failing on a hardcoded absolute path
 * that nobody else owns.
 */
public final class RioFileSink {
  private static final Path RIO_BENCH_DIR = Path.of("rio-bench");

  private RioFileSink() {}

  /**
   * Builds the conventional output path for a run.
   *
   * <p>Filename convention is {@code <wpilib_version>-<rio_firmware>.csv},
   * so multiple runs on the same RIO/firmware combo overwrite each other
   * (intentional — the latest run is the one that matters) while runs
   * across different combos coexist in the same directory.
   */
  public static Path defaultPathFor(RunMetadata metadata) {
    String filename = metadata.wpilibVersion() + "-" + metadata.rioFirmware() + ".csv";
    return RIO_BENCH_DIR.resolve(filename);
  }

  /**
   * Writes {@code csv} to {@code path} as UTF-8, creating parent
   * directories as needed.
   */
  public static void write(Path path, String csv) throws IOException {
    Files.createDirectories(path.getParent());
    Files.writeString(path, csv, StandardCharsets.UTF_8);
  }
}
