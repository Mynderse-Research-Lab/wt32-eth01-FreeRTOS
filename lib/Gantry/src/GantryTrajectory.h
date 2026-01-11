/**
 * @file GantryTrajectory.h
 * @brief Trajectory planning for 3-axis gantry
 * @version 1.0.0
 * 
 * This module provides trajectory planning functions:
 * - Trapezoidal velocity profiles
 * - Waypoint management (future)
 * - Multi-segment trajectories (future)
 */

#ifndef GANTRY_TRAJECTORY_H
#define GANTRY_TRAJECTORY_H

#include "GantryConfig.h"
#include <stdint.h>
#include <cmath>

namespace Gantry {

/**
 * @struct TrapezoidalProfile
 * @brief Trapezoidal velocity profile parameters
 * 
 * Represents a trapezoidal velocity profile with acceleration,
 * cruise, and deceleration phases.
 */
struct TrapezoidalProfile {
    float t_accel;      // Acceleration time (s)
    float t_cruise;     // Cruise time (s)
    float t_decel;      // Deceleration time (s)
    float total_time;   // Total time (s)
    float max_speed;    // Maximum speed reached
    bool valid;         // Profile validity flag
    
    TrapezoidalProfile() 
        : t_accel(0.0f), t_cruise(0.0f), t_decel(0.0f), 
          total_time(0.0f), max_speed(0.0f), valid(false) {}
};

/**
 * @struct Waypoint
 * @brief Single waypoint in a trajectory
 */
struct Waypoint {
    JointConfig position;
    float speed_mm_per_s;
    float acceleration_mm_per_s2;
    float deceleration_mm_per_s2;
    
    Waypoint() 
        : speed_mm_per_s(200.0f), 
          acceleration_mm_per_s2(1000.0f),
          deceleration_mm_per_s2(1000.0f) {}
};

/**
 * @class TrajectoryPlanner
 * @brief Simple trajectory planner for point-to-point motion
 */
class TrajectoryPlanner {
public:
    /**
     * @brief Calculate trapezoidal velocity profile
     * 
     * Calculates the timing parameters for a trapezoidal velocity profile
     * between two positions. Handles both trapezoidal (reaches max speed)
     * and triangular (doesn't reach max speed) profiles.
     * 
     * @param start Start position
     * @param target Target position
     * @param max_speed Maximum speed
     * @param acceleration Acceleration rate
     * @param deceleration Deceleration rate
     * @return Trapezoidal profile with timing parameters
     */
    static TrapezoidalProfile calculateProfile(float start, float target,
                                               float max_speed,
                                               float acceleration,
                                               float deceleration);
    
    /**
     * @brief Interpolate position from trapezoidal profile
     * 
     * Calculates the position at a given time using the trapezoidal profile.
     * 
     * @param profile Trapezoidal profile
     * @param start Start position
     * @param target Target position
     * @param elapsed_s Elapsed time (seconds)
     * @return Interpolated position
     */
    static float interpolate(const TrapezoidalProfile& profile,
                            float start, float target, float elapsed_s);
};

} // namespace Gantry

#endif // GANTRY_TRAJECTORY_H
