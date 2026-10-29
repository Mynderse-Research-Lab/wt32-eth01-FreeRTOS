/**
 * @file GantryKinematics.cpp
 * @brief Forward and inverse kinematics implementation.
 * @version 2.1.0
 *
 * Coordinate convention (transformed frame):
 *   New +X = horizontal traverse across belt (inverted: x_home = x_max).
 *   New +Y = along-belt (no actuator; +Y = downstream, unchanged).
 *   New +Z = vertical height above conveyor (inverted: z=0 conveyor, z=z_max retracted).
 *   Theta  = 0° along New +X, +90° along +Y (from new +X to unchanged +Y).
 */

#include "GantryKinematics.h"

namespace Gantry {

EndEffectorPose Kinematics::forward(const JointConfig& joints,
                                     const KinematicParameters& params) {
    EndEffectorPose pose;

    // Invert X: x_home = x_max, so when joint.x = 0, pose.x = x_max
    pose.x     = params.x_stroke_mm - (joints.x + params.theta_x_offset_mm);
    pose.y     = params.z_axis_y_offset_mm;
    // Invert Z: z_home = z_max (complete retraction), z=0 is conveyor surface
    pose.z     = params.z_stroke_mm - joints.z;
    // Theta: 0° is along New +X, +90° is along +Y
    pose.theta = 90.0f - joints.theta;

    return pose;
}

JointConfig Kinematics::inverse(const EndEffectorPose& pose,
                                 const KinematicParameters& params) {
    JointConfig joints;

    // Invert X: joint.x = x_max - pose.x - theta_x_offset
    joints.x     = params.x_stroke_mm - pose.x - params.theta_x_offset_mm;
    // Invert Z: joint.z = z_max - pose.z
    joints.z     = params.z_stroke_mm - pose.z;
    // Theta: joint.theta = 90° - pose.theta
    joints.theta = 90.0f - pose.theta;

    return joints;
}

bool Kinematics::validate(const JointConfig& joints, const JointLimits& limits) {
    return limits.isValid(joints);
}

} // namespace Gantry
