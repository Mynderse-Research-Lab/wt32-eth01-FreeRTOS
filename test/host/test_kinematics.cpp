// Host unit tests for Gantry forward/inverse kinematics and joint-limit
// validation. Promoted from the on-target selftest in src/basic_tests.cpp so
// the core math is regression-tested in CI without hardware.

#include "unity.h"

#include "GantryConfig.h"
#include "GantryKinematics.h"

void setUp(void) {}
void tearDown(void) {}

namespace {
Gantry::KinematicParameters makeParams() {
  Gantry::KinematicParameters params;
  params.theta_x_offset_mm  = -55.0f;
  params.z_axis_y_offset_mm = 80.0f;
  return params;
}
}  // namespace

// Forward kinematics: joint space -> workspace pose.
static void test_forward_x_includes_theta_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  // pose.x = joint.x + theta_x_offset_mm = 100 + (-55) = 45
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 45.0f, pose.x);
}

static void test_forward_y_fixed_from_z_axis_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 80.0f, pose.y);
}

static void test_forward_z_adds_datum_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 25.0f + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM, pose.z);
}

static void test_forward_theta_passthrough(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30.0f, pose.theta);
}

// Inverse kinematics round-trip (pose.y is dropped - no Y actuator).
static void test_inverse_round_trip(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::KinematicParameters params = makeParams();
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, params);
  Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, joints.x, inv.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, joints.z, inv.z);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, joints.theta, inv.theta);
}

// Joint-limit validation.
static void test_validate_inside_limits(void) {
  Gantry::JointLimits limits;
  limits.x_min = 0.0f;
  limits.x_max = 200.0f;
  limits.z_min = 0.0f;
  limits.z_max = 100.0f;
  limits.theta_min = -90.0f;
  limits.theta_max = 90.0f;
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  TEST_ASSERT_TRUE(Gantry::Kinematics::validate(joints, limits));
}

static void test_validate_outside_x_limits(void) {
  Gantry::JointLimits limits;
  limits.x_min = 0.0f;
  limits.x_max = 200.0f;
  limits.z_min = 0.0f;
  limits.z_max = 100.0f;
  limits.theta_min = -90.0f;
  limits.theta_max = 90.0f;
  Gantry::JointConfig joints(250.0f, 25.0f, 30.0f);
  TEST_ASSERT_FALSE(Gantry::Kinematics::validate(joints, limits));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_forward_x_includes_theta_offset);
  RUN_TEST(test_forward_y_fixed_from_z_axis_offset);
  RUN_TEST(test_forward_z_adds_datum_offset);
  RUN_TEST(test_forward_theta_passthrough);
  RUN_TEST(test_inverse_round_trip);
  RUN_TEST(test_validate_inside_limits);
  RUN_TEST(test_validate_outside_x_limits);
  return UNITY_END();
}
