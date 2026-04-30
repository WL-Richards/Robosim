package frc.robot.bench.runner;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.bench.calls.CallClass;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.concurrent.locks.LockSupport;

/**
 * Wraps each {@link CallClass} as a one-shot timed block. Each
 * {@code timeBlockNs} implementation runs the underlying HAL call
 * {@link CallClass#blockSize()} times inside a single
 * {@code System.nanoTime()} window — this amortizes the timestamping
 * overhead across the call cost we actually want to measure.
 *
 * <p>The {@code sink} field exists to defeat dead-code elimination: every
 * timed block writes its result into {@code sink} via XOR so the JIT cannot
 * recognize the loop body as side-effect-free and skip it.
 *
 * <p>Holds per-instance native buffers (for the CAN HAL calls) so the
 * timed sections never allocate.
 */
public final class CallBindings {
  /**
   * CAN message ID outside any production device range. Used to time the
   * HAL call cost itself; we expect the receive to find no match and the
   * send to either succeed silently or be ignored by every device.
   */
  private static final int CAN_BENCH_MESSAGE_ID = 0x1FFFFFFE;

  /**
   * CAN message ID used by {@link #spamCanFrame()} to software-saturate
   * the bus during {@code SATURATED_BUS} / {@code SATURATED_MULTI_THREAD}
   * phases. Distinct from {@link #CAN_BENCH_MESSAGE_ID} so spam traffic
   * cannot contaminate the timed receive/send measurements (which are
   * pinned to {@code CAN_BENCH_MESSAGE_ID}).
   */
  private static final int CAN_SPAM_MESSAGE_ID = 0x1FFFFFFD;

  private final IntBuffer canRxId =
      ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN).asIntBuffer();
  private final ByteBuffer canRxTimestamp =
      ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN);
  private final byte[] canTxPayload = new byte[8];
  private final byte[] canSpamPayload = new byte[8];

  private long sink;

  /**
   * Times one block for the requested call class.
   *
   * @return wall-time nanoseconds for the whole block (not per-call); the
   *     {@link frc.robot.bench.stats.Statistics} stage divides by
   *     {@link CallClass#blockSize()} when reporting per-call latency
   */
  public long timeBlockNs(CallClass cc) {
    return switch (cc) {
      case HAL_GET_FPGA_TIME -> timeFpgaTime();
      case HAL_NOTIFIER_WAIT -> timeNotifierWait();
      case DS_STATE_READ -> timeDsStateRead();
      case POWER_STATE_READ -> timePowerStateRead();
      case CAN_FRAME_READ -> timeCanFrameRead();
      case CAN_FRAME_WRITE -> timeCanFrameWrite();
    };
  }

  /**
   * 100-call block. {@code HAL_GetFPGATime} is one of the cheapest HAL
   * calls, so we need a large block to lift the signal above clock
   * granularity (~250 ns on the RIO 2).
   */
  private long timeFpgaTime() {
    long start = System.nanoTime();
    for (int i = 0; i < 100; i++) {
      sink ^= HALUtil.getFPGATime();
    }
    return System.nanoTime() - start;
  }

  /**
   * Single-shot 1 ms park. Reports the *jitter* (absolute deviation from
   * the requested wake instant) rather than total elapsed time, because
   * the 1 ms sleep itself is a constant we already know.
   */
  private long timeNotifierWait() {
    long targetNs = System.nanoTime() + 1_000_000L;
    LockSupport.parkNanos(1_000_000L);
    return Math.abs(System.nanoTime() - targetNs);
  }

  /**
   * 10-call block hitting both the bool and the double DS getters in each
   * iteration; together they exercise the typical "is enabled + match
   * time" pair a robot loop reads.
   */
  private long timeDsStateRead() {
    long start = System.nanoTime();
    for (int i = 0; i < 10; i++) {
      sink ^= DriverStation.isEnabled() ? 1L : 0L;
      sink ^= Double.doubleToLongBits(DriverStation.getMatchTime());
    }
    return System.nanoTime() - start;
  }

  /** 10-call block of battery-voltage reads — a representative power-state HAL surface. */
  private long timePowerStateRead() {
    long start = System.nanoTime();
    for (int i = 0; i < 10; i++) {
      sink ^= Double.doubleToLongBits(RobotController.getBatteryVoltage());
    }
    return System.nanoTime() - start;
  }

  /**
   * Times the CAN-receive HAL call once. The receive is expected to find
   * no frame matching the bench ID; the cost being measured is the round-
   * trip into the FRCNetComm session mux, not actual frame-handling work.
   */
  private long timeCanFrameRead() {
    canRxId.put(0, CAN_BENCH_MESSAGE_ID);
    long start = System.nanoTime();
    try {
      CANJNI.FRCNetCommCANSessionMuxReceiveMessage(
          canRxId, 0x1FFFFFFF, canRxTimestamp);
    } catch (RuntimeException ignored) {
      // No frame matching the bench ID — the HAL call cost is what we measure.
    }
    return System.nanoTime() - start;
  }

  /**
   * Times the CAN-send HAL call once. {@code CAN_SEND_PERIOD_NO_REPEAT}
   * disables HAL retries, so we measure a single transmit attempt
   * regardless of bus state.
   */
  private long timeCanFrameWrite() {
    long start = System.nanoTime();
    try {
      CANJNI.FRCNetCommCANSessionMuxSendMessage(
          CAN_BENCH_MESSAGE_ID, canTxPayload, CANJNI.CAN_SEND_PERIOD_NO_REPEAT);
    } catch (RuntimeException ignored) {
      // Send may report bus error if no devices ack; HAL call cost is measured.
    }
    return System.nanoTime() - start;
  }

  /**
   * Drives one CAN-frame transmission for software-driven bus saturation.
   * Called in a tight loop by the CAN-spam worker that {@code WorkerSpec}
   * spawns for {@code SATURATED_BUS} and {@code SATURATED_MULTI_THREAD}
   * phases. Uses a distinct message ID from {@link #timeCanFrameWrite()}
   * and {@link #timeCanFrameRead()} so spam traffic cannot land in the
   * receive queue used by the timed measurement.
   *
   * <p>Not timed; the runner does not feed this into the sequencer. The
   * goal is HAL-call traffic on the bus, not the cost of the call.
   */
  public void spamCanFrame() {
    try {
      CANJNI.FRCNetCommCANSessionMuxSendMessage(
          CAN_SPAM_MESSAGE_ID, canSpamPayload, CANJNI.CAN_SEND_PERIOD_NO_REPEAT);
    } catch (RuntimeException ignored) {
      // Bus-error / no-ack failures are expected during saturation. The
      // intent is to keep the HAL session-mux busy, not to land frames.
    }
  }

  /**
   * Exposes the JIT-defeat sink. Callers that want to be doubly sure the
   * loops were not optimized away can read this and write it somewhere
   * the JIT cannot see (for example, log output).
   */
  public long sinkValue() {
    return sink;
  }
}
