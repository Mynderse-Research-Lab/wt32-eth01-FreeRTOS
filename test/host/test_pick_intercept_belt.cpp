/**
 * @file test_pick_intercept_belt.cpp
 * @brief Belt clock host tests.
 */

#include "test_pick_intercept_internal.h"

using namespace pick_intercept_test;

// ---------------------------------------------------------------------------
// Belt clock
// ---------------------------------------------------------------------------

void test_position_reanchors_on_displacement(void) {
  PickIntercept::BeltClock belt;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                    static_cast<int>(PickIntercept::noteConveyor(
                        belt, frameAt(1000000, 1000000, 200.0f, 1.0f, 1))));
  const double mid = PickIntercept::beltPositionMm(belt, 1500000);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1100.0f, static_cast<float>(mid));

  // 1 s later the encoder says +500 mm, not the 200 mm the old speed would add.
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                    static_cast<int>(PickIntercept::noteConveyor(
                        belt, frameAt(2000000, 2000000, 200.0f, 1.5f, 2))));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1500.0f, static_cast<float>(belt.displacement_mm));
  const double later = PickIntercept::beltPositionMm(belt, 2500000);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1600.0f, static_cast<float>(later));
}

void test_timestamp_wrap_is_not_a_restart(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 0xFFFFF000u, 1000.0f, 0.0f, 1));
  const auto note =
      PickIntercept::noteConveyor(belt, frameAt(1000 + 8192, 0x1000u, 500.0f, 0.01f, 2));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_TRUE(belt.conveyor_us > 0xFFFFF000LL);
}

void test_sequence_wrap_is_not_a_restart(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 1.0f, 255));
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 1.01f, 0));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1010.0f, static_cast<float>(belt.displacement_mm));
}

void test_displacement_backward_restarts_and_bumps_epoch(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 2.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(0, belt.epoch);
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 0.1f, 11));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Restart), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 100.0f, static_cast<float>(belt.displacement_mm));
  TEST_ASSERT_EQUAL_UINT32(1, belt.epoch);
}

void test_sequence_reset_restarts(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 1.0f, 40));
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 1.01f, 0));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Restart), static_cast<int>(note));
}

void test_min_delay_anchors_at_conveyor_time(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000000, 1000000, 200.0f, 0.0f, 1));
  TEST_ASSERT_EQUAL_INT64(0, belt.offset_us);
  // This frame sat in a queue for 750 ms: gantry time jumped more than conveyor time.
  PickIntercept::noteConveyor(belt, frameAt(2000000, 1250000, 200.0f, 0.05f, 2));
  TEST_ASSERT_EQUAL_INT64(0, belt.offset_us);
  TEST_ASSERT_EQUAL_INT64(1250000, belt.anchor_gantry_us);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 50.0f, static_cast<float>(belt.displacement_mm));
  // Anchored at receipt this would read 50 mm; the belt has really moved on.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 200.0f,
                           static_cast<float>(PickIntercept::beltPositionMm(belt, 2000000)));
}

void test_stale_after_two_missed_frames(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000000, 1000000, 200.0f, 0.0f, 1));
  TEST_ASSERT_FALSE(PickIntercept::beltStale(belt, 1000000 + 100000));
  TEST_ASSERT_TRUE(PickIntercept::beltStale(belt, 1000000 + PickIntercept::kStaleAfterUs + 1));
}

void test_capture_subtracts_latency_until_displacement_is_valid(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(2000000, 2000000, 200.0f, 1.0f, 1));
  const double from_latency =
      PickIntercept::capturePositionMm(belt, 2000000, 100000, false, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 980.0f, static_cast<float>(from_latency));
  const double from_flag =
      PickIntercept::capturePositionMm(belt, 2000000, 100000, true, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 500.0f, static_cast<float>(from_flag));
}

void test_flat_displacement_falls_back_to_speed(void) {
  PickIntercept::BeltClock belt;
  int64_t t = 1000000;
  PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, 1));
  PickIntercept::BeltNote note = PickIntercept::BeltNote::Anchored;
  for (uint8_t seq = 2; seq <= 4; ++seq) {
    t += 250000;
    note = PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, seq));
    if (seq < 4) {
      TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                        static_cast<int>(note));
    }
  }
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Fallback), static_cast<int>(note));
  TEST_ASSERT_TRUE(belt.fallback);
  // Three flat frames at 200 mm/s = 150 mm integrated; the position is continuous.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1150.0f, static_cast<float>(belt.displacement_mm));
  t += 250000;
  note = PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, 5));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1200.0f, static_cast<float>(belt.displacement_mm));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 700.0f, static_cast<float>(PickIntercept::capturePositionMm(
                                               belt, t, 0, true, 0.5f)));
}
