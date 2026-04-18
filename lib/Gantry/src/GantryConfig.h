/**
 * @file GantryConfig.h
 * @brief Configuration structures for 3-axis gantry system
 * @version 1.0.0
 * 
 * This module defines data structures for:
 * - Joint space configuration (x, y, theta)
 * - End-effector pose (cartesian coordinates)
 * - Joint limits and kinematic parameters
 * - Complete gantry configuration
 */

#ifndef GANTRY_CONFIG_H
#define GANTRY_CONFIG_H

#include <stdint.h>
#include "axis_drivetrain_params.h"

namespace Gantry {

/**
 * @struct JointConfig
 * @brief Joint space configuration (internal representation)
 * 
 * Represents the configuration of the gantry in joint space:
 * - X-axis: Horizontal prismatic joint (right-to-left)
 * - Y-axis: Vertical prismatic joint (down-to-up)
 * - Theta: Rotational joint at end of Y-axis
 */
struct JointConfig {
    float x;      // X-axis position (mm) - horizontal (right-to-left)
    float y;      // Y-axis position (mm) - vertical (down-to-up)
    float theta;  // Theta angle (degrees) - rotation around Y-axis
    
    // Constructors
    JointConfig() : x(0.0f), y(0.0f), theta(0.0f) {}
    
    JointConfig(float x_val, float y_val, float theta_val) 
        : x(x_val), y(y_val), theta(theta_val) {}
    
    // Operators
    JointConfig operator+(const JointConfig& other) const {
        return JointConfig(x + other.x, y + other.y, theta + other.theta);
    }
    
    JointConfig operator-(const JointConfig& other) const {
        return JointConfig(x - other.x, y - other.y, theta - other.theta);
    }
    
    JointConfig operator*(float scale) const {
        return JointConfig(x * scale, y * scale, theta * scale);
    }
};

/**
 * @struct JointLimits
 * @brief Joint limits for validation
 */
struct JointLimits {
    float x_min, x_max;
    float y_min, y_max;
    float theta_min, theta_max;
    
    JointLimits() 
        : x_min(0.0f), x_max(0.0f),
          y_min(0.0f), y_max(0.0f),
          theta_min(-90.0f), theta_max(90.0f) {}
    
    /**
     * @brief Check if joint configuration is within limits
     * @param config Joint configuration to validate
     * @return true if valid, false otherwise
     */
    bool isValid(const JointConfig& config) const {
        return (config.x >= x_min && config.x <= x_max) &&
               (config.y >= y_min && config.y <= y_max) &&
               (config.theta >= theta_min && config.theta <= theta_max);
    }
};

/**
 * @struct EndEffectorPose
 * @brief End-effector pose in workspace/cartesian coordinates
 */
struct EndEffectorPose {
    float x, y, z;    // Position (mm)
    float theta;      // Orientation (degrees)
    
    EndEffectorPose() : x(0.0f), y(0.0f), z(0.0f), theta(0.0f) {}
    
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val)
        : x(x_val), y(y_val), z(z_val), theta(theta_val) {}
};

/**
 * @struct KinematicParameters
 * @brief Mechanical parameters for kinematics
 * 
 * These parameters define the mechanical layout of the gantry system
 */
struct KinematicParameters {
    float y_axis_z_offset_mm;    // Y-axis Z offset from X (default: 80mm)
    float theta_x_offset_mm;     // Theta X offset from Y vertical (default: -55mm)
    float gripper_y_offset_mm;   // Gripper Y offset (default: 385mm)
    float gripper_z_offset_mm;   // Gripper Z offset (default: 80mm)

    KinematicParameters()
        : y_axis_z_offset_mm(GANTRY_Y_AXIS_Z_OFFSET_MM),
          theta_x_offset_mm(GANTRY_THETA_X_OFFSET_MM),
          gripper_y_offset_mm(GANTRY_GRIPPER_Y_OFFSET_MM),
          gripper_z_offset_mm(GANTRY_GRIPPER_Z_OFFSET_MM) {}
};

/**
 * @struct GantryConfig
 * @brief Complete gantry configuration
 */
struct GantryConfig {
    JointLimits limits;
    KinematicParameters kinematic_params;
    float workspace_origin_offset_mm;
    
    GantryConfig() : workspace_origin_offset_mm(0.0f) {}
};

} // namespace Gantry

#endif // GANTRY_CONFIG_H
