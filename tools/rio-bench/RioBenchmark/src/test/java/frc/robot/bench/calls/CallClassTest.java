package frc.robot.bench.calls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EnumSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class CallClassTest {

  @Test
  void enum_lists_v0_call_classes_in_documented_set() {
    assertEquals(6, CallClass.values().length);
    Set<CallClass> expected =
        EnumSet.of(
            CallClass.HAL_GET_FPGA_TIME,
            CallClass.HAL_NOTIFIER_WAIT,
            CallClass.DS_STATE_READ,
            CallClass.POWER_STATE_READ,
            CallClass.CAN_FRAME_READ,
            CallClass.CAN_FRAME_WRITE);
    assertEquals(expected, EnumSet.allOf(CallClass.class));
  }

  @Test
  void csv_label_is_lowercase_underscore_per_documented_table() {
    assertEquals("hal_get_fpga_time", CallClass.HAL_GET_FPGA_TIME.csvLabel());
    assertEquals("hal_notifier_wait", CallClass.HAL_NOTIFIER_WAIT.csvLabel());
    assertEquals("ds_state_read", CallClass.DS_STATE_READ.csvLabel());
    assertEquals("power_state_read", CallClass.POWER_STATE_READ.csvLabel());
    assertEquals("can_frame_read", CallClass.CAN_FRAME_READ.csvLabel());
    assertEquals("can_frame_write", CallClass.CAN_FRAME_WRITE.csvLabel());
  }

  @Test
  void block_size_matches_documented_per_class_table() {
    assertEquals(100, CallClass.HAL_GET_FPGA_TIME.blockSize());
    assertEquals(10, CallClass.DS_STATE_READ.blockSize());
    assertEquals(10, CallClass.POWER_STATE_READ.blockSize());
    assertEquals(1, CallClass.HAL_NOTIFIER_WAIT.blockSize());
    assertEquals(1, CallClass.CAN_FRAME_READ.blockSize());
    assertEquals(1, CallClass.CAN_FRAME_WRITE.blockSize());
    assertTrue(CallClass.HAL_GET_FPGA_TIME.blockSize() > 0);
  }
}
