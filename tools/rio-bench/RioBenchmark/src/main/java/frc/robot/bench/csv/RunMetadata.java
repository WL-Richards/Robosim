package frc.robot.bench.csv;

import java.util.Optional;

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
