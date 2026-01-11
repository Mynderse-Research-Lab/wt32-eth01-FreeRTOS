/**
 * @file GantryKinematics.cpp
 * @brief Forward and inverse kinematics implementation
 * @version 1.0.0
 */

#include "GantryKinematics.h"

namespace Gantry {

EndEffectorPose Kinematics::forward(const JointConfig& joints, 
                                     const KinematicParameters& params) {
    EndEffectorPose pose;
    
    // For this gantry configuration:
    // - X position: X-axis position + Theta X offset (theta rotates around Y, doesn't change X)
    // - Y position: Y-axis position (direct)
    // - Z position: Constant (Y-axis Z offset)
    // - Theta orientation: Direct from joint
    
    pose.x = joints.x + params.theta_x_offset_mm;
    pose.y = joints.y;
    pose.z = params.y_axis_z_offset_mm;
    pose.theta = joints.theta;
    
    return pose;
}

JointConfig Kinematics::inverse(const EndEffectorPose& pose,
                                 const KinematicParameters& params) {
    JointConfig joints;
    
    // Inverse kinematics for this gantry configuration:
    // - X-axis: End-effector X - Theta X offset
    // - Y-axis: End-effector Y (direct)
    // - Theta: End-effector theta (direct)
    
    joints.x = pose.x - params.theta_x_offset_mm;
    joints.y = pose.y;
    joints.theta = pose.theta;
    
    return joints;
}

bool Kinematics::validate(const JointConfig& joints, const JointLimits& limits) {
    return limits.isValid(joints);
}

} // namespace Gantry
