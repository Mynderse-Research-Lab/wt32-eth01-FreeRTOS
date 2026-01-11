/**
 * @file GantryKinematics.h
 * @brief Forward and inverse kinematics for 3-axis gantry
 * @version 1.0.0
 * 
 * This module provides kinematic functions for converting between:
 * - Joint space (x, y, theta) and Cartesian space (x, y, z, theta)
 * - Forward kinematics: Joint space -> Cartesian space
 * - Inverse kinematics: Cartesian space -> Joint space
 */

#ifndef GANTRY_KINEMATICS_H
#define GANTRY_KINEMATICS_H

#include "GantryConfig.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

namespace Gantry {

/**
 * @class Kinematics
 * @brief Forward and inverse kinematics for 3-axis gantry
 */
class Kinematics {
public:
    /**
     * @brief Forward kinematics: Joint space -> Cartesian space
     * 
     * Calculates the end-effector pose from joint positions.
     * For this gantry configuration:
     * - X and Y positions are directly from joints (orthogonal prismatic joints)
     * - Z position is constant (from kinematic parameters)
     * - Theta orientation is directly from joint
     * 
     * @param joints Joint configuration
     * @param params Kinematic parameters
     * @return End-effector pose in cartesian coordinates
     */
    static EndEffectorPose forward(const JointConfig& joints, 
                                    const KinematicParameters& params);
    
    /**
     * @brief Inverse kinematics: Cartesian space -> Joint space
     * 
     * Calculates the required joint positions from end-effector pose.
     * For this gantry configuration, the solution is straightforward
     * since X and Y are independent prismatic joints.
     * 
     * @param pose End-effector pose
     * @param params Kinematic parameters
     * @return Joint configuration
     */
    static JointConfig inverse(const EndEffectorPose& pose,
                                const KinematicParameters& params);
    
    /**
     * @brief Validate joint configuration against limits
     * 
     * @param joints Joint configuration to validate
     * @param limits Joint limits
     * @return true if valid, false otherwise
     */
    static bool validate(const JointConfig& joints, const JointLimits& limits);
};

} // namespace Gantry

#endif // GANTRY_KINEMATICS_H
