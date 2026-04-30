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

public final class CallBindings {
  // CAN message ID outside any production device range. Used to time the
  // HAL call cost itself; we expect the receive to find no match and the
  // send to either succeed silently or be ignored by every device.
  private static final int CAN_BENCH_MESSAGE_ID = 0x1FFFFFFE;

  private final IntBuffer canRxId =
      ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN).asIntBuffer();
  private final ByteBuffer canRxTimestamp =
      ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN);
  private final byte[] canTxPayload = new byte[8];

  private long sink;

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

  private long timeFpgaTime() {
    long start = System.nanoTime();
    for (int i = 0; i < 100; i++) sink ^= HALUtil.getFPGATime();
    return System.nanoTime() - start;
  }

  private long timeNotifierWait() {
    long targetNs = System.nanoTime() + 1_000_000L;
    LockSupport.parkNanos(1_000_000L);
    return Math.abs(System.nanoTime() - targetNs);
  }

  private long timeDsStateRead() {
    long start = System.nanoTime();
    for (int i = 0; i < 10; i++) {
      sink ^= DriverStation.isEnabled() ? 1L : 0L;
      sink ^= Double.doubleToLongBits(DriverStation.getMatchTime());
    }
    return System.nanoTime() - start;
  }

  private long timePowerStateRead() {
    long start = System.nanoTime();
    for (int i = 0; i < 10; i++) {
      sink ^= Double.doubleToLongBits(RobotController.getBatteryVoltage());
    }
    return System.nanoTime() - start;
  }

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

  public long sinkValue() {
    return sink;
  }
}
