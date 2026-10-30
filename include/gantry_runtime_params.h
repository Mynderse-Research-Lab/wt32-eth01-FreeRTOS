/**
 * @file gantry_runtime_params.h
 * @brief Maps stored parameters onto what the Gantry library applies.
 *
 * The Gantry library has no dependency on AppConfig. main.cpp calls this at boot
 * and the console calls it after every 'cfg' change, so a stored parameter is the
 * only value in force at runtime. The macros in axis_drivetrain_params.h only
 * seed AppConfig's compile-time defaults.
 */

#ifndef GANTRY_RUNTIME_PARAMS_H
#define GANTRY_RUNTIME_PARAMS_H

#include "AppConfigData.h"
#include "GantryUtils.h"

inline Gantry::RuntimeParams gantryRuntimeParams(const Config::AppConfigData& c) {
    Gantry::RuntimeParams p;
    p.z_speed_cap_mm_s =
        (static_cast<float>(c.axis_z_critical_rpm) / 60.0f) * c.axis_z_lead_mm_per_rev;
    p.theta_max_speed_deg_s = c.axis_theta_max_speed_deg_per_s;
    p.theta_accel_deg_s2 = c.axis_theta_accel_deg_per_s2;
    p.theta_decel_deg_s2 = c.axis_theta_decel_deg_per_s2;
    p.gripper_open_ms = static_cast<uint32_t>(c.gantry_gripper_open_time_ms);
    p.gripper_close_ms = static_cast<uint32_t>(c.gantry_gripper_close_time_ms);
    p.x_hard_span_mm = c.axis_x_hard_limit_max_mm - c.axis_x_hard_limit_min_mm;
    p.home_speed_mm_s = c.eip_home_cal_speed_mm_s;
    p.home_accel_mm_s2 = c.eip_home_cal_accel_mm_s2;
    p.home_theta_speed_deg_s = c.eip_home_theta_speed_deg_s;
    p.creep_speed_mm_s = c.eip_creep_speed_mm_s;
    p.z_axis_y_offset_mm = c.gantry_z_axis_y_offset_mm;
    p.theta_x_offset_mm = c.gantry_theta_x_offset_mm;
    p.gripper_x_offset_mm = c.gantry_gripper_x_offset_mm;
    p.gripper_z_offset_mm = c.gantry_gripper_z_offset_mm;
    return p;
}

#endif  // GANTRY_RUNTIME_PARAMS_H
