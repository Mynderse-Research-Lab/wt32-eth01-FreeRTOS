/**
 * @file test_eip_axis_rotary.cpp
 * @brief Rotary move and home host tests.
 */

#include "test_eip_axis_internal.h"

using namespace eip_axis_test;

void test_rotary_enable_holds_feedback_and_keeps_halt(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(9000, false, true));
  TEST_ASSERT_TRUE(axis.enable());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw0 = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x0020, cw0);  // C0500 / clear_errors
  TEST_ASSERT_BITS_HIGH(0x0002, cw0);  // stay OM
  TEST_ASSERT_BITS_LOW(0x8000, cw0);   // Drive ON after fault-clear
  TEST_ASSERT_EQUAL_INT32(9000, readLeI32(cmd, 2));
  TEST_ASSERT_FALSE(axis.isBusy());

  for (int i = 0; i < 12; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x8000, cw);  // Drive ON
  TEST_ASSERT_BITS_HIGH(0x4000, cw);  // Drive Enable
  TEST_ASSERT_BITS_LOW(0x2000, cw);   // Halt active — do not start
  TEST_ASSERT_BITS_LOW(0x0020, cw);   // clear_errors released

  TEST_ASSERT_TRUE(axis.disable());
  Bytes off;
  TEST_ASSERT_TRUE(image.getCommand(off));
  TEST_ASSERT_EQUAL_INT32(9000, readLeI32(off, 2));
  const uint16_t off_cw = readLeU16(off, 0);
  TEST_ASSERT_BITS_HIGH(0x0002, off_cw);  // OM bit1 — must not enter A0050
  TEST_ASSERT_BITS_LOW(0xA000, off_cw);   // Drive ON + Halt off
}

void test_rotary_arst_while_disabled_stays_drive_off(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(1785000, true, true, 0x4009u));
  TEST_ASSERT_TRUE(axis.clearAlarm());
  TEST_ASSERT_FALSE(axis.isEnabled());
  for (int i = 0; i < 12; ++i) {
    axis.update();
  }
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x0002, cw);  // OM
  TEST_ASSERT_BITS_LOW(0x8000, cw);   // Drive ON must stay 0
  TEST_ASSERT_BITS_LOW(0x0020, cw);   // C0500 pulse released
}

void test_rotary_move_and_feedback(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(0, false, false));
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));

  Bytes preload;
  TEST_ASSERT_TRUE(image.getCommand(preload));
  TEST_ASSERT_EQUAL_UINT32(eip::hcs01::kOutput101Size, preload.size());
  TEST_ASSERT_EQUAL_INT32(4500, readLeI32(preload, 2));
  TEST_ASSERT_BITS_LOW(0x2000, readLeU16(preload, 0));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 16; ++i) {
    axis.update();
  }
  Bytes started;
  TEST_ASSERT_TRUE(image.getCommand(started));
  TEST_ASSERT_EQUAL_INT32(4500, readLeI32(started, 2));
  TEST_ASSERT_BITS_HIGH(0x2000, readLeU16(started, 0));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, axis.getCurrentDeg());
  TEST_ASSERT_FALSE(axis.isBusy());
}

// Class 1 stays up, ready never reaches 3. After the 1 s AH cap the hold
// command stops tracking feedback.
void test_theta_ah_times_out_when_ready_stays_below_operation(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  auto frame = [](int32_t puu) {
    eip::hcs01::Hcs01StatusWord st;
    st.ready = eip::hcs01::ReadyForOperation::kReadyAb;
    st.operating_mode_ack = eip::hcs01::OperatingModeAck::kOperatingMode;
    eip::hcs01::Hcs01PositioningActual actual;
    actual.status = st;
    actual.position_feedback = puu;
    Bytes out;
    eip::ByteWriter w(out);
    w.u16(actual.status.encode());
    w.i32(actual.position_feedback);
    w.i32(0);
    w.u32(0);
    return out;
  };

  image.setFeedback(frame(1000));
  TEST_ASSERT_TRUE(axis.enable());
  for (int i = 0; i < 160; ++i) {
    image.setFeedback(frame(1000));
    axis.update();
  }
  Bytes held;
  TEST_ASSERT_TRUE(image.getCommand(held));
  TEST_ASSERT_EQUAL_INT32(1000, readLeI32(held, 2));

  for (int i = 0; i < 5; ++i) {
    image.setFeedback(frame(8000));
    axis.update();
  }
  Bytes after;
  TEST_ASSERT_TRUE(image.getCommand(after));
  TEST_ASSERT_EQUAL_INT32_MESSAGE(1000, readLeI32(after, 2),
                                  "AH cap must stop tracking feedback");
}

void test_rotary_offline_aborts_move(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 4; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setOnline(false);
  axis.update();
  TEST_ASSERT_FALSE(axis.isBusy());
}
// The theta trace reads the move stamp and feedback from another task.
void test_rotary_move_stamp_and_trace_sample(void) {
  HostTest::setUs(5000000);
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  image.setFeedback(makeHcs01Input102(100000, false, true));

  Gantry::RotaryMoveStamp st;
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_EQUAL_UINT32(0, st.seq);

  // velocity_feedback 100 = 0.01 rpm = 0.06 deg/s.
  Gantry::RotaryTraceSample smp;
  TEST_ASSERT_TRUE(axis.sampleTrace(smp));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.0f, static_cast<float>(smp.deg));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.06f, smp.vel_deg_s);
  TEST_ASSERT_EQUAL_HEX16(0xC410, smp.status);

  TEST_ASSERT_TRUE(axis.moveToDeg(-35.0f, 1980.0f, 9900.0f, 9900.0f));
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_EQUAL_UINT32(1, st.seq);
  TEST_ASSERT_EQUAL_INT64(5000000, st.cmd_us);
  TEST_ASSERT_EQUAL_INT64(0, st.start_us);
  TEST_ASSERT_EQUAL_INT64(0, st.done_us);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Gantry::RotaryMoveEnd::kNone),
                          static_cast<uint8_t>(st.end));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, st.start_deg);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -35.0f, st.target_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1980.0f, st.vel_deg_s);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 9900.0f, st.acc_deg_s2);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 9900.0f, st.dec_deg_s2);

  for (int i = 0; i < 16; ++i) {
    HostTest::advanceMs(1);
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_TRUE(st.start_us > st.cmd_us);
  TEST_ASSERT_EQUAL_INT64(0, st.done_us);

  for (int i = 0; i < 4; ++i) {
    HostTest::advanceMs(1);
    image.setFeedback(makeHcs01Input102(-350000, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_TRUE(st.done_us > st.start_us);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Gantry::RotaryMoveEnd::kExact),
                          static_cast<uint8_t>(st.end));

  // A stop while idle keeps the finished move's end; a stop mid-move is an abort.
  const int64_t done_us = st.done_us;
  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_EQUAL_INT64(done_us, st.done_us);
  TEST_ASSERT_TRUE(axis.moveToDeg(0.0f, 1980.0f, 9900.0f, 9900.0f));
  HostTest::advanceMs(1);
  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_TRUE(axis.getMoveStamp(st));
  TEST_ASSERT_EQUAL_UINT32(2, st.seq);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Gantry::RotaryMoveEnd::kAbort),
                          static_cast<uint8_t>(st.end));
  HostTest::resetClock();
}

void test_rotary_consecutive_move_ignores_stale_in_pos(void) {
  // Live test_cycle I: H leaves bit4=1 at ~179°, then 179→0 must stay busy
  // until the new target, not halt at the previous pose.
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  for (int i = 0; i < 16; ++i) {
    axis.update();
  }
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, axis.getCurrentDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(0.0f, 3600.0f, 18000.0f, 18000.0f));
  TEST_ASSERT_TRUE(axis.isBusy());
  for (int i = 0; i < 16; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "stale in-pos at previous target cleared busy");
  }
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.0f, axis.getCurrentDeg());

  image.setFeedback(makeHcs01Input102(2000, false, false));
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 16; ++i) {
    image.setFeedback(makeHcs01Input102(0, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, axis.getCurrentDeg());
}

void test_rotary_unaligned_home_offsets_and_clamps_to_drive_travel(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(1785127, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 178.5127f, axis.getDriveAbsDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 178.5127f, axis.getCurrentDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 36000.0f, axis.getMaxDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -36000.0f, axis.getMinDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(180.0f, 30.0f, 180.0f, 180.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const int32_t cmd_puu = readLeI32(cmd, 2);
  TEST_ASSERT_EQUAL_INT32(1800000, cmd_puu);
}

void test_rotary_aligned_home_is_drive_identity(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, axis.getDriveAbsDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, axis.getCurrentDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 36000.0f, axis.getMaxDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -36000.0f, axis.getMinDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(180.0f, 30.0f, 180.0f, 180.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const int32_t cmd_puu = readLeI32(cmd, 2);
  TEST_ASSERT_EQUAL_INT32(1800000, cmd_puu);
}

void test_rotary_unaligned_small_delta_keeps_offset_command(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(9000, false, true));
  TEST_ASSERT_TRUE(axis.hasLiveFeedback());
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, axis.getCurrentDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(10.0f, 30.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(1000, readLeI32(cmd, 2));
}

void test_rotary_set_puu_per_deg(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_FALSE(axis.setPuuPerDeg(0.0));
  TEST_ASSERT_FALSE(axis.setPuuPerDeg(-1.0));
  TEST_ASSERT_TRUE(axis.setPuuPerDeg(200.0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 200.0f, static_cast<float>(axis.pulsesPerDeg()));
  TEST_ASSERT_TRUE(axis.moveToDeg(10.0f, 5.0f, 20.0f, 30.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(2000, readLeI32(cmd, 2));
  // 5 deg/s; velocity does not scale with PUU/deg (S-0-0259 is 0.0001 rpm).
  TEST_ASSERT_EQUAL_INT32(8333, readLeI32(cmd, 6));
  TEST_ASSERT_EQUAL_INT32(349, readLeI32(cmd, 10));
  TEST_ASSERT_EQUAL_INT32(524, readLeI32(cmd, 14));
}
