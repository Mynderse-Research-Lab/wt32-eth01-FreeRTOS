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

Gantry::JointLimits makeLimits() {
  Gantry::JointLimits limits;
  limits.x_min = 0.0f;
  limits.x_max = 200.0f;
  limits.z_min = 0.0f;
  limits.z_max = 100.0f;
  limits.theta_min = -90.0f;
  limits.theta_max = 90.0f;
  return limits;
}

// Host builds use GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM = 0.0f unless overridden.
constexpr float kZDatum = GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM;
}  // namespace

// ---------------------------------------------------------------------------
// Forward kinematics — pin exact values (not just "within")
// ---------------------------------------------------------------------------
static void test_forward_x_includes_theta_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(45.0f, pose.x);  // 100 + (-55)
}

static void test_forward_y_fixed_from_z_axis_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(80.0f, pose.y);
}

static void test_forward_z_adds_datum_offset(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(25.0f + kZDatum, pose.z);
}

static void test_forward_theta_passthrough(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(30.0f, pose.theta);
}

static void test_forward_zero_joints(void) {
  Gantry::JointConfig joints(0.0f, 0.0f, 0.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(-55.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(80.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f + kZDatum, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

static void test_forward_negative_theta_and_x(void) {
  Gantry::JointConfig joints(-10.0f, 5.0f, -45.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, makeParams());
  TEST_ASSERT_EQUAL_FLOAT(-65.0f, pose.x);  // -10 + (-55)
  TEST_ASSERT_EQUAL_FLOAT(5.0f + kZDatum, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(-45.0f, pose.theta);
}

static void test_forward_y_independent_of_joints(void) {
  Gantry::KinematicParameters params = makeParams();
  Gantry::EndEffectorPose a =
      Gantry::Kinematics::forward(Gantry::JointConfig(0.0f, 0.0f, 0.0f), params);
  Gantry::EndEffectorPose b =
      Gantry::Kinematics::forward(Gantry::JointConfig(180.0f, 90.0f, 60.0f), params);
  TEST_ASSERT_EQUAL_FLOAT(a.y, b.y);
  TEST_ASSERT_EQUAL_FLOAT(80.0f, a.y);
}

// ---------------------------------------------------------------------------
// Inverse + round-trip variants
// ---------------------------------------------------------------------------
static void test_inverse_round_trip(void) {
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::KinematicParameters params = makeParams();
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, params);
  Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
  TEST_ASSERT_EQUAL_FLOAT(joints.x, inv.x);
  TEST_ASSERT_EQUAL_FLOAT(joints.z, inv.z);
  TEST_ASSERT_EQUAL_FLOAT(joints.theta, inv.theta);
}

static void test_inverse_drops_pose_y(void) {
  Gantry::KinematicParameters params = makeParams();
  Gantry::EndEffectorPose pose(45.0f, 999.0f, 25.0f + kZDatum, 30.0f);
  Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
  // pose.y must not affect joints (no Y actuator)
  TEST_ASSERT_EQUAL_FLOAT(100.0f, inv.x);  // 45 - (-55)
  TEST_ASSERT_EQUAL_FLOAT(25.0f, inv.z);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, inv.theta);
}

static void test_inverse_round_trip_table(void) {
  const Gantry::JointConfig cases[] = {
      Gantry::JointConfig(0.0f, 0.0f, 0.0f),
      Gantry::JointConfig(200.0f, 100.0f, -90.0f),
      Gantry::JointConfig(12.5f, 7.25f, 15.0f),
      Gantry::JointConfig(-5.0f, 50.0f, 89.9f),
  };
  Gantry::KinematicParameters params = makeParams();
  for (const Gantry::JointConfig& joints : cases) {
    Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, params);
    Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, joints.x, inv.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, joints.z, inv.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, joints.theta, inv.theta);
  }
}

static void test_forward_inverse_with_alt_offsets(void) {
  Gantry::KinematicParameters params;
  params.theta_x_offset_mm = 12.0f;
  params.z_axis_y_offset_mm = -40.0f;
  Gantry::JointConfig joints(50.0f, 10.0f, -20.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, params);
  TEST_ASSERT_EQUAL_FLOAT(62.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(-40.0f, pose.y);
  Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
  TEST_ASSERT_EQUAL_FLOAT(50.0f, inv.x);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, inv.z);
  TEST_ASSERT_EQUAL_FLOAT(-20.0f, inv.theta);
}

// ---------------------------------------------------------------------------
// Joint-limit validation — inclusive bounds + each axis
// ---------------------------------------------------------------------------
static void test_validate_inside_limits(void) {
  TEST_ASSERT_TRUE(
      Gantry::Kinematics::validate(Gantry::JointConfig(100.0f, 25.0f, 30.0f),
                                   makeLimits()));
}

static void test_validate_at_exact_bounds(void) {
  Gantry::JointLimits limits = makeLimits();
  TEST_ASSERT_TRUE(
      Gantry::Kinematics::validate(Gantry::JointConfig(0.0f, 0.0f, -90.0f), limits));
  TEST_ASSERT_TRUE(
      Gantry::Kinematics::validate(Gantry::JointConfig(200.0f, 100.0f, 90.0f),
                                   limits));
}

static void test_validate_outside_x_limits(void) {
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(250.0f, 25.0f, 30.0f),
                                   makeLimits()));
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(-0.1f, 25.0f, 30.0f),
                                   makeLimits()));
}

static void test_validate_outside_z_limits(void) {
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(100.0f, -1.0f, 0.0f),
                                   makeLimits()));
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(100.0f, 100.1f, 0.0f),
                                   makeLimits()));
}

static void test_validate_outside_theta_limits(void) {
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(100.0f, 25.0f, -90.1f),
                                   makeLimits()));
  TEST_ASSERT_FALSE(
      Gantry::Kinematics::validate(Gantry::JointConfig(100.0f, 25.0f, 90.1f),
                                   makeLimits()));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_forward_x_includes_theta_offset);
  RUN_TEST(test_forward_y_fixed_from_z_axis_offset);
  RUN_TEST(test_forward_z_adds_datum_offset);
  RUN_TEST(test_forward_theta_passthrough);
  RUN_TEST(test_forward_zero_joints);
  RUN_TEST(test_forward_negative_theta_and_x);
  RUN_TEST(test_forward_y_independent_of_joints);
  RUN_TEST(test_inverse_round_trip);
  RUN_TEST(test_inverse_drops_pose_y);
  RUN_TEST(test_inverse_round_trip_table);
  RUN_TEST(test_forward_inverse_with_alt_offsets);
  RUN_TEST(test_validate_inside_limits);
  RUN_TEST(test_validate_at_exact_bounds);
  RUN_TEST(test_validate_outside_x_limits);
  RUN_TEST(test_validate_outside_z_limits);
  RUN_TEST(test_validate_outside_theta_limits);
  return UNITY_END();
}
