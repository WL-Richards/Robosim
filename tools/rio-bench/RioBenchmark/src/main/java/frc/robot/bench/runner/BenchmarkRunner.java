package frc.robot.bench.runner;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.bench.calls.CallClass;
import frc.robot.bench.csv.BenchmarkRecord;
import frc.robot.bench.csv.CsvWriter;
import frc.robot.bench.csv.RunMetadata;
import frc.robot.bench.io.RioFileSink;
import frc.robot.bench.sequencer.OperatingPoint;
import frc.robot.bench.sequencer.Sequencer;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public final class BenchmarkRunner {
  private static final int WARMUP_COUNT = 1000;
  private static final int PER_POINT_COUNT = 10000;
  private static final int SAMPLES_PER_BLOCK = PER_POINT_COUNT;

  private static final List<CallClass> CALL_CLASSES = List.of(CallClass.values());

  private final RunMetadata metadata;
  private volatile boolean started;

  public BenchmarkRunner(RunMetadata metadata) {
    this.metadata = metadata;
  }

  public synchronized void runOnce() {
    if (started) return;
    started = true;

    DataLogManager.log("[rio-bench] starting sweep");

    Sequencer sequencer = new Sequencer(WARMUP_COUNT, PER_POINT_COUNT, CALL_CLASSES);
    CallBindings bindings = new CallBindings();

    for (int i = 0; i < WARMUP_COUNT; i++) {
      for (CallClass c : CALL_CLASSES) {
        long ns = bindings.timeBlockNs(c);
        sequencer.submitSample(c, ns);
      }
    }

    for (OperatingPoint phase : OperatingPoint.values()) {
      DataLogManager.log("[rio-bench] entering operating point: " + phase.csvLabel());
      AtomicBoolean stopWorkers = new AtomicBoolean(false);
      ExecutorService workers = startWorkersFor(phase, stopWorkers, bindings);
      try {
        for (int i = 0; i < PER_POINT_COUNT; i++) {
          for (CallClass c : CALL_CLASSES) {
            long ns = bindings.timeBlockNs(c);
            sequencer.submitSample(c, ns);
          }
        }
      } finally {
        stopWorkers.set(true);
        if (workers != null) {
          workers.shutdown();
          try {
            workers.awaitTermination(5, TimeUnit.SECONDS);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
        }
      }
    }

    List<BenchmarkRecord> records = sequencer.extractRecords();
    String csv = CsvWriter.write(metadata, records);
    Path outPath = RioFileSink.defaultPathFor(metadata);
    try {
      RioFileSink.write(outPath, csv);
      DataLogManager.log("[rio-bench] DONE: file at " + outPath);
    } catch (IOException e) {
      DataLogManager.log("[rio-bench] ERROR: " + e.getMessage());
    }
  }

  private ExecutorService startWorkersFor(
      OperatingPoint phase, AtomicBoolean stop, CallBindings bindings) {
    if (phase != OperatingPoint.MULTI_THREAD) return null;
    ExecutorService pool = Executors.newFixedThreadPool(3);
    for (int i = 0; i < 3; i++) {
      pool.submit(
          () -> {
            while (!stop.get()) bindings.timeBlockNs(CallClass.HAL_GET_FPGA_TIME);
          });
    }
    return pool;
  }

  public static RunMetadata buildMetadata(String wpilibVersion, String rioFirmware) {
    return new RunMetadata(
        wpilibVersion,
        rioFirmware,
        "unknown",
        gitShaFromManifestOrUnknown(),
        java.time.Instant.now().toString(),
        WARMUP_COUNT,
        SAMPLES_PER_BLOCK,
        CallClass.HAL_GET_FPGA_TIME.blockSize(),
        false,
        Optional.empty());
  }

  private static String gitShaFromManifestOrUnknown() {
    return "unknown";
  }
}
