/**
 * @file GantryKinematics.cpp
 * @brief Forward and inverse kinematics implementation.
 * @version 2.0.0
 *
 * Coordinate convention:
 *   X = horizontal traverse, Y = along-belt (no actuator; -Y = downstream),
 *   Z = vertical (+Z = up). pose.z = joint.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 *   (TCP height above physical belt/bed; see axis_drivetrain_params.h).
 *   Theta = rotation about Z.
 */

#include "GantryKinematics.h"

namespace Gantry {

EndEffectorPose Kinematics::forward(const JointConfig& joints,
                                     const KinematicParameters& params) {
    EndEffectorPose pose;

    pose.x     = joints.x + params.theta_x_offset_mm;
    pose.y     = params.z_axis_y_offset_mm;
    pose.z     = joints.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM;
    pose.theta = joints.theta;

    return pose;
}

JointConfig Kinematics::inverse(const EndEffectorPose& pose,
                                 const KinematicParameters& params) {
    JointConfig joints;

    joints.x     = pose.x - params.theta_x_offset_mm;
    joints.z     = pose.z - GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM;
    joints.theta = pose.theta;

    return joints;
}

bool Kinematics::validate(const JointConfig& joints, const JointLimits& limits) {
    return limits.isValid(joints);
}

} // namespace Gantry
