// Host checks that derived drivetrain macros stay pinned to datasheet defaults.

#include "unity.h"

#include "axis_drivetrain_params.h"

void setUp(void) {}
void tearDown(void) {}

static void test_x_pulses_per_mm(void) {
  const double expected =
      (static_cast<double>(AXIS_X_ENCODER_PPR) *
       static_cast<double>(AXIS_X_MOTOR_REDUCER_RATIO)) /
      static_cast<double>(AXIS_X_LEAD_MM_PER_REV);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, static_cast<float>(expected),
                           static_cast<float>(AXIS_X_PULSES_PER_MM));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 52428.8f,
                           static_cast<float>(AXIS_X_PULSES_PER_MM));
}

static void test_z_pulses_per_mm(void) {
  const double expected =
      (static_cast<double>(AXIS_Z_ENCODER_PPR) *
       static_cast<double>(AXIS_Z_MOTOR_REDUCER_RATIO)) /
      static_cast<double>(AXIS_Z_LEAD_MM_PER_REV);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, static_cast<float>(expected),
                           static_cast<float>(AXIS_Z_PULSES_PER_MM));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 104857.6f,
                           static_cast<float>(AXIS_Z_PULSES_PER_MM));
}

static void test_theta_pulses_per_deg(void) {
  const double expected =
      (static_cast<double>(AXIS_THETA_ENCODER_PPR) *
       static_cast<double>(AXIS_THETA_MOTOR_REDUCER_RATIO) *
       static_cast<double>(AXIS_THETA_OUTPUT_GEAR_RATIO)) /
      360.0;
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, static_cast<float>(expected),
                           static_cast<float>(AXIS_THETA_PULSES_PER_DEG));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 100.0f,
                           static_cast<float>(AXIS_THETA_PULSES_PER_DEG));
}

static void test_z_critical_rpm_speed_cap_is_1000(void) {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 1000.0f,
                           static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S));
  const double derived =
      (static_cast<double>(AXIS_Z_CRITICAL_RPM) / 60.0) *
      static_cast<double>(AXIS_Z_LEAD_MM_PER_REV);
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-3f, static_cast<float>(derived),
      static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S));
}

static void test_default_hard_joint_limits(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, AXIS_X_HARD_LIMIT_MIN_MM);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 550.0f, AXIS_X_HARD_LIMIT_MAX_MM);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, AXIS_Z_HARD_LIMIT_MIN_MM);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, AXIS_Z_HARD_LIMIT_MAX_MM);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -180.0f, AXIS_THETA_HARD_LIMIT_MIN_DEG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, AXIS_THETA_HARD_LIMIT_MAX_DEG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -180.0f, AXIS_THETA_DRIVE_ABS_MIN_DEG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, AXIS_THETA_DRIVE_ABS_MAX_DEG);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_x_pulses_per_mm);
  RUN_TEST(test_z_pulses_per_mm);
  RUN_TEST(test_theta_pulses_per_deg);
  RUN_TEST(test_z_critical_rpm_speed_cap_is_1000);
  RUN_TEST(test_default_hard_joint_limits);
  return UNITY_END();
}
