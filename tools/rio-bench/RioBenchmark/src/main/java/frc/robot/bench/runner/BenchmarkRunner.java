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

/**
 * Drives one full benchmark sweep — warmup, then every operating point in
 * order — and writes the resulting CSV to the RIO. Designed to run on the
 * {@link frc.robot.Robot} autonomous worker thread; not safe for concurrent
 * {@link #runOnce()} callers.
 */
public final class BenchmarkRunner {
  /**
   * Iterations per call class before any samples are recorded; lets the JIT
   * and CPU caches reach steady state. Bumped from 1000 → 10000 after the
   * first hardware run showed `idle_bus` (the first measured phase) biased
   * high for `block_size=1` call classes (CAN_FRAME_*, HAL_NOTIFIER_WAIT) —
   * 1000 invocations is below C2 JIT's ~10k threshold, so those calls
   * weren't fully warm by the time recording started. 10000 invocations
   * past C2 + the notifier-wait warmup costs ~10s extra wall-time, which
   * is dominated by the rest of the sweep.
   */
  private static final int WARMUP_COUNT = 10000;

  /** Iterations per call class per operating point. Echoed as {@code samples_per_block} in the CSV header. */
  private static final int PER_POINT_COUNT = 10000;

  private static final int SAMPLES_PER_BLOCK = PER_POINT_COUNT;

  private static final List<CallClass> CALL_CLASSES = List.of(CallClass.values());

  private final RunMetadata metadata;
  private volatile boolean started;

  /**
   * @param metadata immutable header values written into the CSV preamble;
   *     typically built via {@link #buildMetadata(String, String)}
   */
  public BenchmarkRunner(RunMetadata metadata) {
    this.metadata = metadata;
  }

  /**
   * Runs the entire sweep exactly once. Subsequent calls are no-ops; the
   * one-shot guard means an operator who hits autonomous twice in a single
   * boot does not re-warm and overwrite the CSV mid-experiment.
   */
  public synchronized void runOnce() {
    if (started) {
      return;
    }
    started = true;

    DataLogManager.log("[rio-bench] starting sweep");

    Sequencer sequencer = new Sequencer(WARMUP_COUNT, PER_POINT_COUNT, CALL_CLASSES);
    CallBindings bindings = new CallBindings();

    // Progress tracking — one timing block per call class per iteration,
    // spanning warmup plus every operating point.
    final int totalBlocks =
        (WARMUP_COUNT + OperatingPoint.values().length * PER_POINT_COUNT) * CALL_CLASSES.size();
    int completedBlocks = 0;
    int nextProgressPct = 5;

    // Warmup phase: results are counted but discarded.
    for (int i = 0; i < WARMUP_COUNT; i++) {
      for (CallClass c : CALL_CLASSES) {
        long ns = bindings.timeBlockNs(c);
        sequencer.submitSample(c, ns);
        completedBlocks++;
        nextProgressPct = reportProgress(completedBlocks, totalBlocks, nextProgressPct);
      }
    }

    // Recorded phases: one operating point at a time, with any required
    // background workers running for the duration of that phase only.
    for (OperatingPoint phase : OperatingPoint.values()) {
      DataLogManager.log("[rio-bench] entering operating point: " + phase.csvLabel());

      AtomicBoolean stopWorkers = new AtomicBoolean(false);
      ExecutorService workers = startWorkersFor(phase, stopWorkers, bindings);

      try {
        for (int i = 0; i < PER_POINT_COUNT; i++) {
          for (CallClass c : CALL_CLASSES) {
            long ns = bindings.timeBlockNs(c);
            sequencer.submitSample(c, ns);
            completedBlocks++;
            nextProgressPct = reportProgress(completedBlocks, totalBlocks, nextProgressPct);
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

    // Emit CSV. Failures only log; the daemon thread exits either way.
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

  /**
   * Spins up background load matching {@code phase} per the
   * {@link WorkerSpec} table. The operating-point → load-profile mapping
   * lives in exactly one place ({@code WorkerSpec.forPoint}) so that
   * cycle-A's cross-product points stay consistent with cycle-B+ additions.
   *
   * <p>Worker threads:
   * <ul>
   *   <li>CPU-contention workers — each loops on a cheap HAL call, modeling
   *       the cross-thread contention a typical robot project sees from
   *       logging threads and command dispatch.</li>
   *   <li>CAN-spam worker (when {@code spec.canSpam()}) — loops on
   *       {@link CallBindings#spamCanFrame()} to software-saturate the
   *       bus regardless of operator hardware. This is the {@code D-RB-8}
   *       semantic shift: {@code SATURATED_BUS} is no longer
   *       operator-hardware-driven.</li>
   * </ul>
   *
   * @return null when {@code spec} requests no workers (e.g. {@code IDLE_BUS});
   *     the caller treats null as "nothing to shut down" in the
   *     {@code finally} block
   */
  private ExecutorService startWorkersFor(
      OperatingPoint phase, AtomicBoolean stop, CallBindings bindings) {
    WorkerSpec spec = WorkerSpec.forPoint(phase);
    int totalWorkers = spec.cpuWorkers() + spec.canSpamWorkers();
    if (totalWorkers == 0) {
      return null;
    }

    ExecutorService pool = Executors.newFixedThreadPool(totalWorkers);
    for (int i = 0; i < spec.cpuWorkers(); i++) {
      pool.submit(
          () -> {
            while (!stop.get()) {
              bindings.timeBlockNs(CallClass.HAL_GET_FPGA_TIME);
            }
          });
    }
    for (int i = 0; i < spec.canSpamWorkers(); i++) {
      pool.submit(
          () -> {
            while (!stop.get()) {
              bindings.spamCanFrame();
            }
          });
    }
    return pool;
  }

  /**
   * Builds the {@link RunMetadata} written into the CSV preamble. Static so
   * {@link frc.robot.Robot} can build it on the WPILib thread before handing
   * the runner off to the worker.
   *
   * @param wpilibVersion {@code edu.wpi.first.wpilibj.util.WPILibVersion#Version}
   * @param rioFirmware human-curated RIO firmware string (env var on deploy)
   */
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

  /**
   * Placeholder for reading the git SHA out of the deploy jar manifest.
   * Currently returns {@code "unknown"} — the build does not stamp the SHA
   * yet, and we would rather record an honest "unknown" than a stale value
   * carried over from a previous deploy.
   */
  private static String gitShaFromManifestOrUnknown() {
    return "unknown";
  }

  /**
   * Emits a progress line each time {@code completedBlocks / totalBlocks}
   * crosses the next 5%-multiple. The {@code while} loop covers the case
   * where a single submission advances through more than one threshold —
   * possible when {@code totalBlocks} is small (synthetic / debug runs).
   *
   * @return the next pct threshold to watch for, threaded back through
   *     the caller so progress state stays in {@link #runOnce()} locals
   */
  private static int reportProgress(int completedBlocks, int totalBlocks, int nextProgressPct) {
    int pct = (int) ((long) completedBlocks * 100L / totalBlocks);
    while (pct >= nextProgressPct && nextProgressPct <= 100) {
      DataLogManager.log("[rio-bench] progress: " + nextProgressPct + "%");
      nextProgressPct += 5;
    }
    return nextProgressPct;
  }
}
