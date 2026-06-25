/**
 * @file GantryConfig.h
 * @brief Configuration structures for the 3-axis gantry system.
 * @version 2.0.0
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   - X: horizontal traverse along the gantry beam (across the conveyor belt).
 *        Actuator: belt linear stage.
 *   - Y: along-belt direction; the gantry has NO Y actuator. Conveyor
 *        downstream is the -Y direction. Used only by the pick scheduler and
 *        the MQTT bridge to describe battery positions on the belt.
 *   - Z: vertical (gantry descent axis). +Z = up; joint Z=0 is the homing
 *        datum (limit switch), not necessarily the belt touch plane — see
 *        GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM in axis_drivetrain_params.h.
 *        Safe-retracted / top-home is at Z = Z_max.
 *   - Theta: rotation about the vertical (Z) axis.
 *
 * The vertical actuator was historically named "Y"; references to "Y axis"
 * elsewhere in old code/docs refer to what is now the Z axis.
 *
 * This module defines data structures for:
 *   - Joint space configuration (x, z, theta)
 *   - End-effector pose in cartesian coordinates (x, y, z, theta)
 *   - Joint limits and kinematic parameters
 *   - Complete gantry configuration
 */

#ifndef GANTRY_CONFIG_H
#define GANTRY_CONFIG_H

#include <stdint.h>
#include "axis_drivetrain_params.h"

namespace Gantry {

/**
 * @struct JointConfig
 * @brief Joint space configuration (internal representation).
 *
 * The gantry has TWO prismatic joints and ONE rotary joint:
 *   - x:     X-axis position (mm) - horizontal traverse along the beam
 *            (across the conveyor; +X by hardware convention).
 *   - z:     Z-axis position (mm) - vertical; +Z = up; Z=0 is the homing datum
 *            (see GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM for physical bed plane).
 *   - theta: Rotation around the vertical (Z) axis (degrees).
 *
 * There is intentionally no JointConfig::y member - the conveyor along-belt
 * (Y) direction has no gantry actuator; battery Y positions live in the pick
 * scheduler / MQTT bridge data structures, not in JointConfig.
 */
struct JointConfig {
    float x;      // X-axis position (mm) - horizontal traverse
    float z;      // Z-axis position (mm) - vertical, +Z = up; 0 = homing datum
    float theta;  // Theta angle (degrees) - rotation around Z

    JointConfig() : x(0.0f), z(0.0f), theta(0.0f) {}

    JointConfig(float x_val, float z_val, float theta_val)
        : x(x_val), z(z_val), theta(theta_val) {}

    JointConfig operator+(const JointConfig& other) const {
        return JointConfig(x + other.x, z + other.z, theta + other.theta);
    }

    JointConfig operator-(const JointConfig& other) const {
        return JointConfig(x - other.x, z - other.z, theta - other.theta);
    }

    JointConfig operator*(float scale) const {
        return JointConfig(x * scale, z * scale, theta * scale);
    }
};

/**
 * @struct JointLimits
 * @brief Joint limits for validation.
 *
 * Y has no actuator and therefore no joint-limit pair; Y travel constraints
 * (if any) live in the pick scheduler's workspace model, not here.
 */
struct JointLimits {
    float x_min, x_max;
    float z_min, z_max;
    float theta_min, theta_max;

    JointLimits()
        : x_min(0.0f), x_max(0.0f),
          z_min(0.0f), z_max(0.0f),
          theta_min(-90.0f), theta_max(90.0f) {}

    /**
     * @brief Check if joint configuration is within limits.
     */
    bool isValid(const JointConfig& config) const {
        return (config.x >= x_min && config.x <= x_max) &&
               (config.z >= z_min && config.z <= z_max) &&
               (config.theta >= theta_min && config.theta <= theta_max);
    }
};

/**
 * @struct EndEffectorPose
 * @brief End-effector pose in workspace/cartesian coordinates.
 *
 * All four spatial fields are explicit:
 *   - x:     horizontal (across belt, mm).
 *   - y:     along-belt (mm); -y is downstream. The gantry itself does not
 *            traverse in y; this field carries the gantry's fixed along-belt
 *            placement plus any along-belt offsets through theta + gripper.
 *   - z:     vertical (mm); +z = up. Workspace TCP height above the physical
 *            belt/bed: z_pose = z_joint + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 *            (macro in axis_drivetrain_params.h; default 0).
 *   - theta: rotation about z (deg).
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
 * @brief Mechanical parameters for kinematics.
 *
 * Field meaning in the new X / Y(=along-belt) / Z(=up) frame:
 *   - z_axis_y_offset_mm : along-belt offset of the Z column from the X beam
 *                          datum (formerly y_axis_z_offset_mm).
 *   - theta_x_offset_mm  : X offset of the theta module from the Z column.
 *   - gripper_x_offset_mm: X offset of the gripper TCP from the theta module
 *                          (formerly gripper_y_offset_mm).
 *   - gripper_z_offset_mm: Z offset of the gripper TCP from the theta module
 *                          mounting flange.
 *
 * Only z_axis_y_offset_mm and theta_x_offset_mm are consumed by the simple
 * forward/inverse implementation in GantryKinematics; the gripper_* offsets
 * are retained so callers can carry full TCP geometry through KinematicParameters
 * once the inverse model becomes pose-aware of the gripper.
 */
struct KinematicParameters {
    float z_axis_y_offset_mm;    // Along-belt offset of Z column from X beam.
    float theta_x_offset_mm;     // Theta X offset from Z column.
    float gripper_x_offset_mm;   // Gripper X offset.
    float gripper_z_offset_mm;   // Gripper Z offset.

    KinematicParameters()
        : z_axis_y_offset_mm(GANTRY_Z_AXIS_Y_OFFSET_MM),
          theta_x_offset_mm(GANTRY_THETA_X_OFFSET_MM),
          gripper_x_offset_mm(GANTRY_GRIPPER_X_OFFSET_MM),
          gripper_z_offset_mm(GANTRY_GRIPPER_Z_OFFSET_MM) {}
};

/**
 * @struct GantryConfig
 * @brief Complete gantry configuration.
 */
struct GantryConfig {
    JointLimits limits;
    KinematicParameters kinematic_params;
    float workspace_origin_offset_mm;

    GantryConfig() : workspace_origin_offset_mm(0.0f) {}
};

} // namespace Gantry

#endif // GANTRY_CONFIG_H
