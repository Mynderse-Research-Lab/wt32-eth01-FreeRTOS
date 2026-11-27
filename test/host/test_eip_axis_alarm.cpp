/**
 * @file test_eip_axis_alarm.cpp
 * @brief Linear alarm and stop host tests.
 */

#include "test_eip_axis_internal.h"

using namespace eip_axis_test;

void test_linear_move_does_not_stop_at_half_travel(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 30.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(150.0f, 200.0f, 3000.0f, 3000.0f));

  Bytes mid = makeK5100Input154(75000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 20; ++i) {
    image.setFeedback(mid);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "busy cleared at half travel");
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_TRUE_MESSAGE((cmd[1] & 0x04) == 0,
                             "StopMotion must not assert at rem≈D/2");
    TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
  }
}

void test_linear_z_ballscrew_scaling(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 4000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(2.0f, 10.0f, 0.0f, 0.0f));

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(8000, readLeI32(cmd, 16));
  TEST_ASSERT_TRUE(axis.pulsesPerMm() > 3999.0 && axis.pulsesPerMm() < 4001.0);
}

void test_linear_move_rejects_drive_limit_stop(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);

  // Fault + Stopped (isLikelyLimitStop) — move must be rejected.
  Bytes limit = makeK5100Input154(0, true, false, false);
  limit[9] |= 0x04 | 0x40;  // active + stopped (ready clear via fault)
  image.setFeedback(limit);
  TEST_ASSERT_FALSE(axis.moveToMm(10.0f, 20.0f, 0.0f, 0.0f));
}

void test_linear_feedback_and_alarm(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 5500);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, axis.getCurrentMm());
  // Sticky CommandInProgress alone must not look busy.
  Bytes cip = makeK5100Input154(5500, false, true, false);
  cip[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(cip);
  TEST_ASSERT_FALSE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.moveToMm(6.0f, 10.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setFeedback(makeK5100Input154(5500, true, false, true));
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}

void test_kinetix_a603_warning_decode(void) {
  char disp[8];
  eip::k5100::DriveCodeInfo info{};
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0603, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A603", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Invalid I/O"));

  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeK5100Input154(0, false, false, true, 0, 0x0603, true));
  TEST_ASSERT_FALSE(axis.isAlarmActive());  // warnings do not hard-fault the axis
  TEST_ASSERT_EQUAL_UINT16(0x0603, axis.getDriveWarningCode());

  char summary[192];
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(summary, sizeof(summary)));
  TEST_ASSERT_NOT_NULL(strstr(summary, "A603"));
}

void test_kinetix_a014_a015_helpers(void) {
  TEST_ASSERT_TRUE(eip::k5100::isWarningA014(0x0014));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA015(0x0015));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA014(0x0015));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA015(0x0014));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA014(0x0603));

  char disp[8];
  eip::k5100::DriveCodeInfo info{};
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0015, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A015", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Negative"));
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0014, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A014", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Positive"));
}

void test_kinetix_format_helpers_and_a603(void) {
  TEST_ASSERT_EQUAL_UINT16(0x0603, eip::k5100::codeDigits(0x1603));
  TEST_ASSERT_EQUAL_UINT16(0x0603, eip::k5100::codeDigits(0x0603));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA603(0x0603));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA603(0x1603));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA603(0x0014));

  char disp[8];
  eip::k5100::formatDisplayCode(disp, sizeof(disp), 'A', 0x0603);
  TEST_ASSERT_EQUAL_STRING("A603", disp);
  eip::k5100::formatDisplayCode(disp, sizeof(disp), 'E', 0x0602);
  TEST_ASSERT_EQUAL_STRING("E602", disp);

  char summary[192];
  TEST_ASSERT_FALSE(eip::k5100::formatDriveTripSummary(summary, sizeof(summary),
                                                       false, false, false, 0, 0));
  TEST_ASSERT_EQUAL_STRING("clear", summary);

  TEST_ASSERT_TRUE(eip::k5100::formatDriveTripSummary(
      summary, sizeof(summary), false, true, false, 0, 0x0603));
  TEST_ASSERT_NOT_NULL(strstr(summary, "A603"));
  TEST_ASSERT_NOT_NULL(strstr(summary, "WARN"));

  TEST_ASSERT_TRUE(eip::k5100::formatDriveTripSummary(
      summary, sizeof(summary), false, false, true, 0, 0));
  TEST_ASSERT_NOT_NULL(strstr(summary, "ConnectionFaulted"));
}

void test_linear_absolute_aborts_busy_on_a015(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(50.0f, 20.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  // A015 warning during Run must abort and clear busy (escape moves still OK).
  Bytes a015 = makeK5100Input154(1000, false, false, false, 0, 0x0015, true);
  a015[9] |= 0x04 | 0x08 | 0x20 | 0x40;  // active+ready+CIP + stopped
  image.setFeedback(a015);
  for (int i = 0; i < 40; ++i) {
    image.setFeedback(a015);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "A015 should clear Absolute busy");

  // Escape move must not be rejected solely for A015 warning.
  TEST_ASSERT_TRUE(axis.moveToMm(0.0f, 20.0f, 0.0f, 0.0f));

  // While still on A015, Absolute must not re-abort (creep/escape path).
  Bytes a015_run = makeK5100Input154(1000, false, false, false, 0, 0x0015, true);
  a015_run[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(a015_run);
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  for (int i = 0; i < 10; ++i) {
    image.setFeedback(a015_run);
    axis.update();
  }
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                           "escape while A015 still set must not re-abort");
}

void test_stop_motion_clears_busy_when_position_stable(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(100.0f, 20.0f, 0.0f, 0.0f));

  Bytes run_fb = makeK5100Input154(25000, false, false, false);
  run_fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(run_fb);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_TRUE(axis.isBusy());

  // Held position + stopped bit: kStopping clears busy after stable ticks.
  Bytes held = makeK5100Input154(25000, false, false, false);
  held[9] |= 0x04 | 0x08 | 0x20 | 0x40;
  for (int i = 0; i < 40; ++i) {
    image.setFeedback(held);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT8(0, static_cast<int8_t>(cmd[0]));  // OM=NotSpecified
  TEST_ASSERT_EQUAL_UINT8(10, cmd[26]);                     // TM=10
  TEST_ASSERT_TRUE((cmd[1] & 0x01) != 0);                   // servo_on
  TEST_ASSERT_TRUE((cmd[1] & 0x04) == 0);                   // StopMotion cleared
}

// Bench regression: during a 1 mm/s creep the drive asserts its `stopped` bit
// and reports near-zero speed. kStopping must keep isBusy() until position
// actually holds still.
void test_stop_motion_stays_busy_while_position_creeps(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(600.0f, 1.0f, 0.0f, 0.0f));

  Bytes run_fb = makeK5100Input154(0, false, false, false);
  run_fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(run_fb);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());

  // 10 PUU/tick ≈ 1 mm/s — same as the bench creep. Drive claims stopped.
  int32_t puu = 0;
  for (int i = 0; i < 50; ++i) {
    Bytes fb = makeK5100Input154(puu, false, false, false);
    fb[9] |= 0x04 | 0x08 | 0x20 | 0x40;
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "creeping feedback must keep kStopping busy");
    puu += 10;
  }
}

// Bench regression: the drive A603s and ignores the abort if it changes anything
// beyond the StopMotion bit. The settle image (OM=0 TM=10, zero decel) swapped
// mode mid-profile and asked for a decel-to-stop with no decel ramp.
// 1 s stop cap: position still changing and speed still high. Busy must
// survive the 200 ms stable window and clear on the 100th 100 Hz tick.
void test_stop_motion_times_out_while_still_moving(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(400.0f, 80.0f, 2000.0f, 100.0f));
  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.stopMotion());

  int32_t puu = 0;
  for (int i = 0; i < 99; ++i) {
    Bytes fb = makeK5100Input154(puu, false, false, false);
    fb[9] |= 0x04 | 0x08 | 0x20;  // active + ready + homed, not stopped
    writeLeI32(fb, 16, 100);      // 10.0 RPM, above the near-stop gate
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "StopMotion must stay busy until the 1 s cap");
    puu += 20;  // 0.02 mm/tick, above the 0.005 mm stable gate
  }
  Bytes fb = makeK5100Input154(puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  writeLeI32(fb, 16, 100);
  image.setFeedback(fb);
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(),
                            "100th tick must force hold while still moving");
}

// Servo-on with Active stuck at 0. The 400 ms settle must expire into hold
// and must not issue method 34.
void test_servo_arm_times_out_when_active_stays_clear(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x08;  // ready, Active clear
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(axis.enable());

  for (int i = 0; i < 10; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_FALSE_MESSAGE(axis.moveToMm(6.0f, 10.0f, 100.0f, 100.0f),
                            "move must wait out the servo-arm settle");

  bool saw_home = false;
  for (int i = 0; i < 50; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    if (cmd[0] == 3) saw_home = true;
  }
  TEST_ASSERT_FALSE(saw_home);
  // Holding copies feedback into the command target. Settle does not.
  Bytes later = makeK5100Input154(8000, false, false, false);
  later[9] |= 0x08;
  image.setFeedback(later);
  axis.update();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 8.0f, axis.getTargetMm(),
                                   "settle cap must leave the axis holding");
}

// HomedStatus stays 0, so arm issues method 34 and the 0.5 s wait expires
// without redefining joint zero.
void test_home34_times_out_when_homed_stays_clear(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x04 | 0x08;  // active + ready, HomedStatus clear
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(axis.enable());

  bool saw_method_34 = false;
  bool left_home = false;
  for (int i = 0; i < 80; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    if (cmd[0] == 3 && cmd[3] == 34) saw_method_34 = true;
    if (saw_method_34 && cmd[0] == 0) left_home = true;
  }
  TEST_ASSERT_TRUE_MESSAGE(saw_method_34, "unhomed arm must issue method 34");
  TEST_ASSERT_TRUE_MESSAGE(left_home, "Home34 wait must return to settle");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 5.0f, axis.getTargetMm(),
                                   "timeout must not zero the joint");
}

void test_move_while_disabled_does_not_arm_motion(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(fb);
  TEST_ASSERT_FALSE(axis.moveToMm(10.0f, 20.0f, 100.0f, 100.0f));
  TEST_ASSERT_FALSE(axis.isBusy());
  for (int i = 0; i < 80; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_NOT_EQUAL(3, cmd[0]);
  }
  TEST_ASSERT_FALSE(axis.isBusy());
}

void test_abort_image_differs_from_move_only_by_stop_bit(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  // Z mapping: A014 is joint max, so it lies ahead of this move toward +600.
  // A limit behind a move does not abort it (G-A13).
  axis.setJointMinWarningA015(true);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);

  TEST_ASSERT_TRUE(axis.moveToMm(600.0f, 1.0f, 0.0f, 0.0f));

  // Settle into Run and capture the accepted in-flight image.
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());
  Bytes run_cmd;
  TEST_ASSERT_TRUE(image.getCommand(run_cmd));
  TEST_ASSERT_TRUE((run_cmd[1] & 0x04) == 0);  // StopMotion clear while running

  // Rising A014 aborts mid-profile — the case that ran forever on the bench.
  Bytes a014 = makeK5100Input154(1100, false, false, false, 0, 0x0014, true);
  a014[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(a014);
  axis.update();

  Bytes stop_cmd;
  TEST_ASSERT_TRUE(image.getCommand(stop_cmd));
  TEST_ASSERT_EQUAL_UINT32(run_cmd.size(), stop_cmd.size());
  TEST_ASSERT_TRUE_MESSAGE((stop_cmd[1] & 0x04) != 0, "abort must set StopMotion");
  for (size_t i = 0; i < run_cmd.size(); ++i) {
    if (i == 1) continue;  // control bits carry the StopMotion edge
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(
        run_cmd[i], stop_cmd[i],
        "abort image must not change OM/TM/speed/accel/decel mid-profile");
  }
}
