#include "basic_tests.h"

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "esp_log.h"

#include <math.h>

namespace {
static const char *TAG = "BasicTests";

bool nearlyEqual(float a, float b, float eps = 1e-3f) {
  return fabsf(a - b) <= eps;
}
}  // namespace

BasicTestSummary runBasicTests() {
  BasicTestSummary summary{0, 0};

  auto check = [&](bool condition, const char *name) {
    if (condition) {
      summary.passed++;
      ESP_LOGI(TAG, "PASS: %s", name);
    } else {
      summary.failed++;
      ESP_LOGE(TAG, "FAIL: %s", name);
    }
  };

  // Coordinate frame (pose): +X across belt (x_home = x_max), +Y downstream,
  // +Z up from belt (z=0 belt, z=z_max retracted), theta 0 deg along +X,
  // +90 deg along +Y (pose.theta = joint.theta + 90 deg).
  Gantry::KinematicParameters params;
  params.theta_x_offset_mm  = -55.0f;
  params.z_axis_y_offset_mm =  80.0f;
  params.x_stroke_mm        = 550.0f;
  params.z_stroke_mm        = 150.0f;

  // Forward kinematics sanity.
  // JointConfig(x_mm, z_mm, theta_deg); pose carries x/y(=along-belt)/z/theta.
  Gantry::JointConfig joints(100.0f, 25.0f, 30.0f);
  Gantry::EndEffectorPose pose = Gantry::Kinematics::forward(joints, params);
  check(nearlyEqual(pose.x, 505.0f), "forward: x = x_stroke - (joint.x + theta_x_offset)");
  check(nearlyEqual(pose.y, 80.0f), "forward: y fixed from z_axis_y_offset_mm");
  check(nearlyEqual(pose.z, 125.0f), "forward: pose.z = z_stroke - joint.z");
  check(nearlyEqual(pose.theta, 120.0f), "forward: pose.theta = joint.theta + 90");

  // Inverse kinematics round-trip (pose.y is dropped - no Y actuator).
  Gantry::JointConfig inv = Gantry::Kinematics::inverse(pose, params);
  check(nearlyEqual(inv.x, joints.x), "inverse: x round-trip");
  check(nearlyEqual(inv.z, joints.z), "inverse: z round-trip");
  check(nearlyEqual(inv.theta, joints.theta), "inverse: theta round-trip");

  // Limit validation sanity. JointLimits has x/z/theta (no y).
  Gantry::JointLimits limits;
  limits.x_min = 0.0f;
  limits.x_max = 200.0f;
  limits.z_min = 0.0f;
  limits.z_max = 100.0f;
  limits.theta_min = -90.0f;
  limits.theta_max = 90.0f;
  check(Gantry::Kinematics::validate(joints, limits), "validate: inside limits");
  check(!Gantry::Kinematics::validate(Gantry::JointConfig(250.0f, 25.0f, 30.0f), limits),
        "validate: outside x limits");

  // Trajectory profile and interpolation sanity.
  Gantry::TrapezoidalProfile profile =
      Gantry::TrajectoryPlanner::calculateProfile(0.0f, 100.0f, 40.0f, 80.0f, 80.0f);
  check(profile.valid, "trajectory: profile valid");
  check(profile.total_time > 0.0f, "trajectory: total_time positive");
  check(nearlyEqual(Gantry::TrajectoryPlanner::interpolate(profile, 0.0f, 100.0f, 0.0f), 0.0f),
        "trajectory: interpolate at t=0 is start");
  check(nearlyEqual(
            Gantry::TrajectoryPlanner::interpolate(profile, 0.0f, 100.0f, profile.total_time),
            100.0f),
        "trajectory: interpolate at end is target");

  ESP_LOGI(TAG, "Basic tests complete: passed=%d failed=%d", summary.passed, summary.failed);
  return summary;
}
