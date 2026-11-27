/**
 * @file test_eip_axis_units.cpp
 * @brief Rotary units, timeout and scanner host tests.
 */

#include "test_eip_axis_internal.h"

using namespace eip_axis_test;

void test_hcs01_units_match_drive_scaling(void) {
    // S-0-0259 per the drive export (S-0-0044 = 2, S-0-0046 = -4): 0.0001 rpm.
    // 15 deg/s = 2.5 rpm. The old 0.0001 deg/min reading sent 9,000,000
    // (900 rpm, past the 600 rpm motor max: E2049, speed ignored).
    TEST_ASSERT_EQUAL_INT32(25000, Gantry::hcs01VelocityUnits(15.0));
    TEST_ASSERT_EQUAL_INT32(100000, Gantry::hcs01VelocityUnits(60.0));
    // Capped at S-0-0113, 600 rpm = 3600 deg/s.
    TEST_ASSERT_EQUAL_INT32(6000000, Gantry::hcs01VelocityUnits(3600.0));
    TEST_ASSERT_EQUAL_INT32(6000000, Gantry::hcs01VelocityUnits(10000.0));
    TEST_ASSERT_EQUAL_INT32(0, Gantry::hcs01VelocityUnits(-5.0));
    // S-0-0260 / S-0-0359: 0.001 rad/s2, no PUU factor.
    TEST_ASSERT_EQUAL_INT32(31416, Gantry::hcs01AccelUnits(1800.0));
    TEST_ASSERT_EQUAL_INT32(314159, Gantry::hcs01AccelUnits(18000.0));
    TEST_ASSERT_EQUAL_INT32(1000000, Gantry::hcs01AccelUnits(1.0e6));
}

// Home / bring-up orient pass accel 0 and fall back to the config defaults.
// Those must reach the drive in the same units as an explicit accel
// (was AXIS_THETA_ACCEL * PUU/deg = 180,000,000: F2028 after ~40 deg).
void test_rotary_default_accel_matches_explicit_accel(void) {
    constexpr double kPuuPerDeg = 10000.0;
    constexpr float kAccelDegS2 = 18000.0f;
    eip::EipProcessImage image;
    image.setOnline(true);
    // Same factory main.cpp uses for the live theta axis.
    const Gantry::EipRotaryAxisConfig cfg =
        Gantry::eipRotaryAxisConfigFromDeg(kPuuPerDeg, 3600.0, kAccelDegS2, kAccelDegS2);
    Gantry::GantryEipRotaryAxis axis(image, cfg);
    TEST_ASSERT_TRUE(axis.begin());
    image.setFeedback(makeHcs01Input102(0, false, false));
    TEST_ASSERT_TRUE(axis.enable());

    Bytes cmd;
    TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 15.0f, kAccelDegS2, kAccelDegS2));
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    const int32_t explicit_acc = readLeI32(cmd, 10);
    const int32_t explicit_dec = readLeI32(cmd, 14);
    TEST_ASSERT_TRUE(axis.stopMotion());

    TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 15.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_EQUAL_INT32(25000, readLeI32(cmd, 6));
    TEST_ASSERT_EQUAL_INT32_MESSAGE(explicit_acc, readLeI32(cmd, 10),
                                    "default accel must use drive units");
    TEST_ASSERT_EQUAL_INT32_MESSAGE(explicit_dec, readLeI32(cmd, 14),
                                    "default decel must use drive units");
}

void test_theta_move_timeout_scales_with_distance(void) {
    TEST_ASSERT_EQUAL_UINT32(Gantry::kThetaMoveTimeoutMinUs,
                             Gantry::thetaMoveTimeoutUs(10.0, 100.0));
    TEST_ASSERT_EQUAL_UINT32(Gantry::kThetaMoveTimeoutMinUs,
                             Gantry::thetaMoveTimeoutUs(300.0, 0.0));
    // 300 deg at the 15 deg/s home speed takes 20 s; allow twice that.
    TEST_ASSERT_EQUAL_UINT32(40000000u, Gantry::thetaMoveTimeoutUs(300.0, 15.0));
    TEST_ASSERT_EQUAL_UINT32(40000000u, Gantry::thetaMoveTimeoutUs(-300.0, 15.0));
}

// A slow move longer than 15 s must not time out at 15 s (home t from far
// away at 15 deg/s), but must still time out once it overruns.
void test_rotary_long_slow_move_outlives_fixed_timeout(void) {
    eip::EipProcessImage image;
    image.setOnline(true);
    Gantry::EipRotaryAxisConfig cfg;
    cfg.puu_per_deg = 100.0;
    Gantry::GantryEipRotaryAxis axis(image, cfg);
    TEST_ASSERT_TRUE(axis.begin());
    image.setFeedback(makeHcs01Input102(0, false, false));
    TEST_ASSERT_TRUE(axis.enable());
    // 170 deg at 5 deg/s: 34 s expected, 68 s timeout. 100 Hz updates.
    TEST_ASSERT_TRUE(axis.moveToDeg(170.0f, 5.0f, 0.0f, 0.0f));
    for (int i = 0; i < 1700; ++i) {
        image.setFeedback(makeHcs01Input102(1000, false, false));
        axis.update();
    }
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "17 s into a 34 s move must still be busy");
    for (int i = 0; i < 5500; ++i) {
        image.setFeedback(makeHcs01Input102(1000, false, false));
        axis.update();
    }
    TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "move must time out after 68 s");
}

void test_rotary_halt_clears_start_bit(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_FALSE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x8000, cw);
  TEST_ASSERT_BITS_LOW(0x2000, cw);
}

void test_rotary_alarm(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  image.setFeedback(makeHcs01Input102(0, true, true));
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}

void test_rotary_thetalim_cannot_expand_past_offset_envelope(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(1785127, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());

  axis.setAngleRange(-180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -180.0f, axis.getMinDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 180.0f, axis.getMaxDeg());

  axis.setAngleRange(-10.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -10.0f, axis.getMinDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f, axis.getMaxDeg());
}

void test_rotary_alarm_summary_and_cip_status(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  char buf[128] = {};
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("clear", buf);
  TEST_ASSERT_TRUE(axis.formatCipStatus(buf, sizeof(buf)));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "T->O live"));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "c1err=0"));

  image.setFeedback(makeHcs01Input102(0, true, true, 0x2057u));
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(buf, sizeof(buf)));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "F2057"));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "c1err=1"));
}

void test_axis_log_due_period(void) {
  int64_t last = 0;
  TEST_ASSERT_FALSE(Gantry::axisLogDue(0, 1000, &last));
  TEST_ASSERT_TRUE(Gantry::axisLogDue(10, 1000, &last));
  TEST_ASSERT_EQUAL(1000, static_cast<int>(last));
  TEST_ASSERT_FALSE(Gantry::axisLogDue(10, 50000, &last));
  TEST_ASSERT_TRUE(Gantry::axisLogDue(10, 101000, &last));
}

void test_scanner_process_image_exchange(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  udp.setEcho(true);
  eip::EipProcessImage image;

  const uint32_t handle = 0x44444444;
  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));

  eip::ScannerConfig scfg;
  scfg.target_ip = "192.168.1.100";
  eip::EipScanner scanner(tcp, udp, scfg);
  scanner.setProcessImage(&image);

  Gantry::EipLinearAxisConfig lcfg;
  lcfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, lcfg);
  TEST_ASSERT_TRUE(scanner.connect());
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  image.setOnline(true);
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(3.0f, 20.0f, 0.0f, 0.0f));

  Bytes sent_before;
  TEST_ASSERT_TRUE(image.getCommand(sent_before));

  Bytes io_out;
  TEST_ASSERT_TRUE(scanner.exchangeOnce(scanner.recvTimeoutMs(), io_out));
  TEST_ASSERT_TRUE(image.isOnline());
  TEST_ASSERT_TRUE(image.feedbackFresh());

  Bytes cmd_after;
  TEST_ASSERT_TRUE(image.getCommand(cmd_after));
  TEST_ASSERT_EQUAL_UINT32(sent_before.size(), cmd_after.size());

  scanner.disconnect();
  TEST_ASSERT_FALSE(image.isOnline());
}

void test_rotary_settles_within_hcs01_window_and_sub_window_move(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  // 1. Move 0 -> 45 deg: settle cleanly once drive reports reached=1 at 44.94 deg
  //    (within HCS01 S-0-0057 = 0.1 deg window, no stall waiting for 0.01 deg).
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 1440.0f, 7200.0f, 7200.0f));
  for (int i = 0; i < 18; ++i) {
    axis.update();
  }
  image.setFeedback(makeHcs01Input102(200000, false, false));
  axis.update();
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(449400, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  // 2. Sub-window move (89.994 -> 90.000 deg): reached stays 1, must finish
  //    promptly instead of hanging until the 15 s timeout.
  image.setFeedback(makeHcs01Input102(899940, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 30.0f, 1800.0f, 1800.0f));
  for (int i = 0; i < 24; ++i) {
    image.setFeedback(makeHcs01Input102(899940, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  // 3. Stiction standstill at 0.15 deg short of target (reached=0, standstill=1):
  //    stays busy at 0.60 deg away AND while still creeping at 30 deg/s (0.30 deg/tick),
  //    then settles cleanly once truly stationary inside 0.30 deg for 50 ms.
  auto makeStandstill = [](int32_t puu) {
    Bytes frame;
    eip::ByteWriter w(frame);
    w.u16(0xC80Eu);  // ready=3 AF, in_standstill=1, command_value_reached=0
    w.i32(puu);
    w.i32(0);
    w.u32(0);
    return frame;
  };
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(-45.0f, 1000.0f, 3600.0f, 3600.0f));
  for (int i = 0; i < 18; ++i) {
    axis.update();
  }
  // Stationary at -44.40 deg (0.60 deg away): stays busy.
  image.setFeedback(makeStandstill(-444000));
  for (int i = 0; i < 8; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  // Passing through [-44.75 .. -45.25] at 0.10 deg/tick (slow bring-up cruise): stays busy.
  for (int i = 0; i < 6; ++i) {
    image.setFeedback(makeStandstill(-447500 - i * 1000));
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  // Stationary in stiction at -44.85 deg (0.15 deg away): settles after 5 stable ticks.
  image.setFeedback(makeStandstill(-448500));
  for (int i = 0; i < 6; ++i) {
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
}
