package frc.robot.bench.csv;

import java.util.Optional;

/**
 * Header values written to the top of the rio-bench CSV. These describe
 * the run environment so a reader can correlate the measurements with a
 * specific RIO firmware / WPILib version / git revision and decide whether
 * the data is still authoritative.
 *
 * @param wpilibVersion {@code edu.wpi.first.wpilibj.util.WPILibVersion#Version}
 * @param rioFirmware human-curated RIO firmware string (env var on deploy);
 *     {@code "unknown"} if not set
 * @param rioImage human-curated RIO image identifier; reserved for a
 *     future deploy step
 * @param gitSha sim repo SHA at build time; {@code "unknown"} until the
 *     build stamps it
 * @param runTimestampIso8601 wall-clock instant the runner started, ISO-8601
 * @param warmupSamples warmup iterations per call class before recording
 *     started
 * @param samplesPerBlock recorded iterations per call class per operating
 *     point
 * @param blockSize underlying HAL invocations per timed block (taken from
 *     {@link frc.robot.bench.calls.CallClass#blockSize()} for the reference
 *     call class)
 * @param validated whether this run passed the post-run sanity checks;
 *     currently always false — the validation step is not yet wired
 * @param busDevices number of devices on the CAN bus during the sweep,
 *     when known
 */
public record RunMetadata(
    String wpilibVersion,
    String rioFirmware,
    String rioImage,
    String gitSha,
    String runTimestampIso8601,
    int warmupSamples,
    int samplesPerBlock,
    int blockSize,
    boolean validated,
    Optional<Integer> busDevices) {}
