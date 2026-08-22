/**
 * @file GantryKinematics.h
 * @brief Forward and inverse kinematics for the 3-axis gantry.
 * @version 2.0.0
 *
 * Coordinate convention (firmware-wide):
 *   - X: horizontal traverse along the gantry beam.
 *   - Y: along-belt direction; +Y is conveyor downstream. The gantry has no
 *        Y actuator, so JointConfig has no y member; the workspace pose Y
 *        field is filled from kinematic parameters (z_axis_y_offset_mm).
 *   - Z: vertical (+Z = down / toward belt). Joint z = A015 retract datum;
 *        pose.z = joint.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 *        (see axis_drivetrain_params.h).
 *   - Theta: rotation about Z (right-handed about +Z).
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
 * @brief Forward and inverse kinematics for the 3-axis gantry.
 */
class Kinematics {
public:
    /**
     * @brief Forward kinematics: joint space -> cartesian (workspace) space.
     *
     * For this gantry configuration:
     *   - pose.x = joint.x + theta_x_offset (theta rotates about Z, doesn't
     *              change pose.x in the simple TCP model)
     *   - pose.y = z_axis_y_offset_mm (along-belt placement is fixed)
     *   - pose.z = joint.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM (+Z = down)
     *   - pose.theta = joint.theta
     */
    static EndEffectorPose forward(const JointConfig& joints,
                                    const KinematicParameters& params);

    /**
     * @brief Inverse kinematics: cartesian (workspace) space -> joint space.
     *
     * Drops pose.y (no Y actuator) and pose-z offsets, leaving the two
     * prismatic joints and the theta angle.
     */
    static JointConfig inverse(const EndEffectorPose& pose,
                                const KinematicParameters& params);

    /**
     * @brief Validate joint configuration against limits.
     */
    static bool validate(const JointConfig& joints, const JointLimits& limits);
};

} // namespace Gantry

#endif // GANTRY_KINEMATICS_H
