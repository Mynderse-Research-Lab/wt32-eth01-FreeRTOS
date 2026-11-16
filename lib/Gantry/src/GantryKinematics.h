/**
 * @file GantryKinematics.h
 * @brief Forward and inverse kinematics for the 3-axis gantry.
 * @version 2.0.0
 *
 * Coordinate convention (firmware-wide):
 *   - X: horizontal traverse along the gantry beam (inverted: x_home = x_max).
 *   - Y: along-belt direction; +Y is conveyor downstream (unchanged).
 *   - Z: vertical height above conveyor (inverted: z=0 conveyor, z=z_max complete retraction).
 *   - Theta: 0° along New +X, +90° along +Y (from new +X to unchanged +Y).
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

/// Theta joint to pose angle. The joint turns about joint +Z (down) and its 0° holds the
/// pads along +Y; the pose angle turns about +Z up from +X. pose = 90° - joint.
inline float thetaPoseFromJointDeg(float joint_deg) { return 90.0f - joint_deg; }
/// Inverse of thetaPoseFromJointDeg: joint = 90° - pose.
inline float thetaJointFromPoseDeg(float pose_deg) { return 90.0f - pose_deg; }

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
     *   - pose.x = params.x_stroke_mm - (joint.x + theta_x_offset) (x_home = x_max)
     *   - pose.y = z_axis_y_offset_mm (along-belt placement is fixed)
     *   - pose.z = params.z_stroke_mm - joint.z (z=0 conveyor, z=z_max complete retraction)
     *   - pose.theta = 90° - joint.theta (0° along New +X, +90° along +Y)
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
