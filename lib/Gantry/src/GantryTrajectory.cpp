#include "GantryTrajectory.h"

#include <cmath>

namespace Gantry {

TrapezoidalProfile TrajectoryPlanner::calculateProfile(float start, float target,
                                                       float max_speed,
                                                       float acceleration,
                                                       float deceleration) {
    TrapezoidalProfile profile;
    
    // Check for finite inputs and positive dynamics
    if (!std::isfinite(start) || !std::isfinite(target) ||
        !std::isfinite(max_speed) || !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
        max_speed <= 1e-6f || acceleration <= 1e-6f || deceleration <= 1e-6f) {
        profile.valid = false;
        return profile;
    }

    // Calculate distance
    float distance = target - start;
    float abs_distance = (distance < 0) ? -distance : distance;
    
    // Handle zero distance
    if (abs_distance < 1e-6f) {
        profile.valid = false;
        return profile;
    }
    
    // Calculate time to accelerate to max speed
    float t_accel = max_speed / acceleration;
    float t_decel = max_speed / deceleration;
    
    // Calculate distance during acceleration and deceleration
    float dist_accel = 0.5f * acceleration * t_accel * t_accel;
    float dist_decel = 0.5f * deceleration * t_decel * t_decel;
    
    // Check if we reach max speed (trapezoidal) or not (triangular)
    if (dist_accel + dist_decel > abs_distance) {
        // Triangular profile - won't reach max speed
        // Scale down to fit distance
        float scale = sqrtf(abs_distance / (dist_accel + dist_decel));
        t_accel *= scale;
        t_decel *= scale;
        dist_accel = 0.5f * acceleration * t_accel * t_accel;
        dist_decel = abs_distance - dist_accel;
        profile.max_speed = acceleration * t_accel;
        profile.t_cruise = 0.0f;
    } else {
        // Trapezoidal profile - reaches max speed
        profile.max_speed = max_speed;
        float dist_cruise = abs_distance - dist_accel - dist_decel;
        profile.t_cruise = dist_cruise / max_speed;
    }
    
    profile.t_accel = t_accel;
    profile.t_decel = t_decel;
    profile.total_time = profile.t_accel + profile.t_cruise + profile.t_decel;
    profile.valid = true;
    
    return profile;
}

float TrajectoryPlanner::interpolate(const TrapezoidalProfile& profile,
                                     float start, float target, float elapsed_s) {
    if (!profile.valid || elapsed_s <= 0.0f) {
        return start;
    }
    
    if (elapsed_s >= profile.total_time) {
        return target;
    }
    
    float distance = target - start;
    float direction = (distance < 0) ? -1.0f : 1.0f;
    
    float current_pos = start;
    
    if (elapsed_s < profile.t_accel && profile.t_accel > 1e-6f) {
        // Acceleration phase: s = 0.5 * a * t²
        float t = elapsed_s;
        float dist = 0.5f * (profile.max_speed / profile.t_accel) * t * t;
        current_pos += direction * dist;
    } else if (elapsed_s < profile.t_accel + profile.t_cruise) {
        // Cruise phase: s = s_accel + v * (t - t_accel)
        float dist_accel = (profile.t_accel > 1e-6f) ? (0.5f * profile.max_speed * profile.t_accel) : 0.0f;
        float t_cruise = elapsed_s - profile.t_accel;
        float dist_cruise = profile.max_speed * t_cruise;
        current_pos += direction * (dist_accel + dist_cruise);
    } else if (profile.t_decel > 1e-6f) {
        // Deceleration phase: s = s_start + s_accel + s_cruise + (v*t_decel - 0.5*a*t²)
        float dist_accel = (profile.t_accel > 1e-6f) ? (0.5f * profile.max_speed * profile.t_accel) : 0.0f;
        float dist_cruise = profile.max_speed * profile.t_cruise;
        float t_decel = elapsed_s - profile.t_accel - profile.t_cruise;
        float decel_rate = profile.max_speed / profile.t_decel;
        float dist_decel = profile.max_speed * t_decel - 0.5f * decel_rate * t_decel * t_decel;
        current_pos += direction * (dist_accel + dist_cruise + dist_decel);
    } else {
        current_pos = target;
    }
    
    return current_pos;
}

} // namespace Gantry
