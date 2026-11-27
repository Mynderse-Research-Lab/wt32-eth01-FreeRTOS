/**
 * @file test_eip_axis_linear.cpp
 * @brief Process image and linear move host tests.
 */

#include "test_eip_axis_internal.h"

using namespace eip_axis_test;

void test_process_image_command_feedback(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  const Bytes cmd = {0x01, 0x02, 0x03};
  const Bytes fb = {0xAA, 0xBB};

  image.setCommand(cmd);
  TEST_ASSERT_TRUE(image.hasCommand());
  Bytes out_cmd;
  TEST_ASSERT_TRUE(image.getCommand(out_cmd));
  TEST_ASSERT_EQUAL_UINT32(cmd.size(), out_cmd.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(cmd.data(), out_cmd.data(), cmd.size());

  TEST_ASSERT_FALSE(image.feedbackFresh());
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(image.feedbackFresh());
  Bytes out_fb;
  TEST_ASSERT_TRUE(image.getFeedback(out_fb));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fb.data(), out_fb.data(), fb.size());
  image.consumeFeedbackFresh();
  TEST_ASSERT_FALSE(image.feedbackFresh());
}

void test_process_image_online(void) {
  eip::EipProcessImage image;
  TEST_ASSERT_FALSE(image.isOnline());
  
  const Bytes fb = {0xAA, 0xBB};
  image.setFeedback(fb);
  Bytes out_fb;
  
  // Offline: should return false even with data
  TEST_ASSERT_FALSE(image.getFeedback(out_fb));
  
  image.setOnline(true);
  TEST_ASSERT_TRUE(image.isOnline());
  // Online: should return true
  TEST_ASSERT_TRUE(image.getFeedback(out_fb));
  
  image.setOnline(false);
  TEST_ASSERT_FALSE(image.isOnline());
  // Offline again: should return false
  TEST_ASSERT_FALSE(image.getFeedback(out_fb));
}

void test_linear_x_belt_move_command(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT32(eip::k5100::kOutput104Size, cmd.size());
  TEST_ASSERT_EQUAL_INT32(12500, readLeI32(cmd, 16));
  // Preload: OperatingMode=Position, TravelMode=2, Absolute, StartMotion low.
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_EQUAL_UINT8(0, cmd[24]);  // NonCyclicMoveType Absolute
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);  // StartMotion edge after preload

  for (int i = 0; i < 5; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);  // Run
}

void test_linear_held_start_waits_for_release(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMmHeld(12.5f, 50.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  Bytes cmd;
  for (int i = 0; i < 20; ++i) {
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE(axis.isStartHeld());
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  }

  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_FALSE(axis.isStartHeld());
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
}

void test_linear_queued_move_waits_for_run_and_edge(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  // Nothing in flight: nothing to queue behind.
  TEST_ASSERT_FALSE(axis.queueHeldMm(2.0f, 40.0f, 0.0f, 0.0f));

  TEST_ASSERT_TRUE(axis.moveToMmHeld(12.5f, 50.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_TRUE(axis.queueHeldMm(2.0f, 40.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_FALSE(axis.queueHeldMm(3.0f, 40.0f, 0.0f, 0.0f));

  // The plunge edge is still high: the queued target is not on the wire yet.
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
  TEST_ASSERT_EQUAL_INT32(12500, cmdPositionPuu(cmd));

  // Moving toward 12.5 mm; once kStart ends the queued target goes out with SM=0.
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x04 | 0x08;
  for (int i = 0; i < 8; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  TEST_ASSERT_EQUAL_INT32(2000, cmdPositionPuu(cmd));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_TRUE(axis.isBusy());

  // The edge waits until the target has been on the wire for 3 RPI.
  TEST_ASSERT_FALSE(axis.releaseStartMotion());
  std::this_thread::sleep_for(std::chrono::milliseconds(7));
  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_FALSE(axis.isStartHeld());
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
  TEST_ASSERT_EQUAL_INT32(2000, cmdPositionPuu(cmd));
  TEST_ASSERT_TRUE(axis.isBusy());
}

void test_linear_offline_aborts_move(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));
  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  // Connection lost mid-move
  image.setOnline(false);
  axis.update(); // Should detect offline and abort immediately

  TEST_ASSERT_FALSE(axis.isBusy()); // Move phase reset to Idle
}

void test_linear_second_absolute_preloads_before_start_edge(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));
  pumpAbsoluteToRun(axis, image, 0);

  Bytes at = makeK5100Input154(12500, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 6; ++i) {
    image.setFeedback(at);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(5000, readLeI32(cmd, 16));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);

  Bytes fb = makeK5100Input154(12500, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
}

void test_linear_move_stays_busy_until_target(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 30; ++i) {
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "busy cleared before reaching target");
  }

  // AtReference + in band must be stable for several ticks before Hold.
  Bytes at_target = makeK5100Input154(12500, false, false, true);
  at_target[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(at_target);
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "need multi-tick stable arrival");
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "need multi-tick stable arrival");
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "should clear busy after stable AtReference");

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE_MESSAGE((cmd[1] & 0x04) == 0,
                           "Position Absolute must not StopMotion on arrival");
}

void test_linear_arrival_requires_stable_ticks(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));

  Bytes mid = makeK5100Input154(1000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(mid);
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes at = makeK5100Input154(5000, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Far-from-target feedback resets the stable arrival counter.
  Bytes glitch = makeK5100Input154(2000, false, false, false);
  glitch[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(glitch);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "counter reset by glitch");
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "counter reset by glitch");
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "clears after 3 stable ticks");
}
void test_axis_debounces_command_value_reached_jitter(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));

  Bytes mid = makeK5100Input154(1000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(mid);
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes at = makeK5100Input154(5000, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  
  Bytes at_jitter = makeK5100Input154(4999, false, false, false); // No AtReference bit
  at_jitter[9] |= 0x04 | 0x08;
  at_jitter[16] = 100; // Inject some velocity so band_stopped is FALSE

  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Jitter! The bit drops, even though we are in position band
  image.setFeedback(at_jitter);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Wait again
  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());
  axis.update();
  TEST_ASSERT_FALSE(axis.isBusy());
}

void test_axis_handles_drive_power_loss_while_network_online(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  
  TEST_ASSERT_TRUE(axis.moveToMm(10.0f, 50.0f, 0.0f, 0.0f));
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  // Simulate Drive Power Loss: Network stays online, but Class 1 reports a fault
  Bytes fault = makeK5100Input154(1000, true, false, false);
  image.setFeedback(fault);
  
  axis.update();
  // Axis enters kAbortStop state to command a stop, so it remains busy
  TEST_ASSERT_TRUE(axis.isBusy());
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}
void test_linear_soft_home_joint_frame(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  Bytes abs_pos = makeK5100Input154(500000, false, false, false);
  abs_pos[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(abs_pos);
  armAxisForMove(axis, image, 500000);

  axis.setCurrentPulses(0);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, axis.getCurrentMm());
  TEST_ASSERT_EQUAL_INT32(0, axis.getEncoderPulses());

  TEST_ASSERT_TRUE(axis.moveToMm(10.0f, 50.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(500000 + 10000, readLeI32(cmd, 16));
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
}

void test_linear_absolute_keeps_target_on_overshoot_feedback(void) {
  // Drive profiles the stop; host must not flip signed Speed jog refs.
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 52428.8;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(1.0f, 50.0f, 0.0f, 0.0f));

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes overshoot = makeK5100Input154(52429 + 100000, false, false, false);
  overshoot[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(overshoot);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(52429, readLeI32(cmd, 16));  // still Absolute target
  TEST_ASSERT_TRUE(readLeI32(cmd, 4) > 0);              // cruise magnitude
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
}
