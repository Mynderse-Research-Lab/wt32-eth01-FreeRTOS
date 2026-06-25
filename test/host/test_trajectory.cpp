// Host unit tests for the trapezoidal trajectory planner. Promoted from the
// on-target selftest in src/basic_tests.cpp.

#include "unity.h"

#include "GantryTrajectory.h"

void setUp(void) {}
void tearDown(void) {}

namespace {
// start=0, target=100, max_speed=40, accel=80, decel=80 (matches basic_tests).
Gantry::TrapezoidalProfile makeProfile() {
  return Gantry::TrajectoryPlanner::calculateProfile(0.0f, 100.0f, 40.0f, 80.0f, 80.0f);
}
}  // namespace

static void test_profile_valid(void) {
  Gantry::TrapezoidalProfile profile = makeProfile();
  TEST_ASSERT_TRUE(profile.valid);
}

static void test_total_time_positive(void) {
  Gantry::TrapezoidalProfile profile = makeProfile();
  TEST_ASSERT_TRUE(profile.total_time > 0.0f);
}

static void test_interpolate_at_start_is_start(void) {
  Gantry::TrapezoidalProfile profile = makeProfile();
  float pos = Gantry::TrajectoryPlanner::interpolate(profile, 0.0f, 100.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, pos);
}

static void test_interpolate_at_end_is_target(void) {
  Gantry::TrapezoidalProfile profile = makeProfile();
  float pos =
      Gantry::TrajectoryPlanner::interpolate(profile, 0.0f, 100.0f, profile.total_time);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, pos);
}

// Zero-distance moves must produce an invalid profile (guard in calculateProfile).
static void test_zero_distance_profile_invalid(void) {
  Gantry::TrapezoidalProfile profile =
      Gantry::TrajectoryPlanner::calculateProfile(50.0f, 50.0f, 40.0f, 80.0f, 80.0f);
  TEST_ASSERT_FALSE(profile.valid);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_profile_valid);
  RUN_TEST(test_total_time_positive);
  RUN_TEST(test_interpolate_at_start_is_start);
  RUN_TEST(test_interpolate_at_end_is_target);
  RUN_TEST(test_zero_distance_profile_invalid);
  return UNITY_END();
}
