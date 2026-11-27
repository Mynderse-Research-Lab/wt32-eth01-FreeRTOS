/**
 * @file AppConfigDefaultsMotion.cpp
 * @brief Compile-time defaults, sections 1-5: motion profile, axes, geometry, gripper.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

void AppConfig::loadMotionDefaults() {
    // 1. Motion Profile
#if defined(CONFIG_GANTRY_DEFAULT_SPEED_MM_PER_S)
    data_.gantry_default_speed_mm_per_s = CONFIG_GANTRY_DEFAULT_SPEED_MM_PER_S;
#else
    data_.gantry_default_speed_mm_per_s = 50;
#endif

#if defined(CONFIG_GANTRY_DEFAULT_ACCEL_MM_PER_S2)
    data_.gantry_default_accel_mm_per_s2 = CONFIG_GANTRY_DEFAULT_ACCEL_MM_PER_S2;
#else
    data_.gantry_default_accel_mm_per_s2 = 3000;
#endif

#if defined(CONFIG_GANTRY_DEFAULT_DECEL_MM_PER_S2)
    data_.gantry_default_decel_mm_per_s2 = CONFIG_GANTRY_DEFAULT_DECEL_MM_PER_S2;
#else
    data_.gantry_default_decel_mm_per_s2 = 3000;
#endif

#if defined(CONFIG_GANTRY_DEFAULT_SPEED_DEG_PER_S)
    data_.gantry_default_speed_deg_per_s = CONFIG_GANTRY_DEFAULT_SPEED_DEG_PER_S;
#else
    data_.gantry_default_speed_deg_per_s = 30;
#endif

#if defined(CONFIG_GANTRY_DEFAULT_ACCEL_DEG_PER_S2)
    data_.gantry_default_accel_deg_per_s2 = CONFIG_GANTRY_DEFAULT_ACCEL_DEG_PER_S2;
#else
    data_.gantry_default_accel_deg_per_s2 = 180;
#endif

#if defined(CONFIG_GANTRY_DEFAULT_DECEL_DEG_PER_S2)
    data_.gantry_default_decel_deg_per_s2 = CONFIG_GANTRY_DEFAULT_DECEL_DEG_PER_S2;
#else
    data_.gantry_default_decel_deg_per_s2 = 180;
#endif

#if defined(CONFIG_GANTRY_PATH_MAX_SPEED_MM_PER_S)
    data_.gantry_path_max_speed_mm_per_s = CONFIG_GANTRY_PATH_MAX_SPEED_MM_PER_S;
#else
    data_.gantry_path_max_speed_mm_per_s = 3333;
#endif

#if defined(CONFIG_GANTRY_PATH_MAX_ACCEL_MM_PER_S2)
    data_.gantry_path_max_accel_mm_per_s2 = CONFIG_GANTRY_PATH_MAX_ACCEL_MM_PER_S2;
#else
    data_.gantry_path_max_accel_mm_per_s2 = 20000;
#endif

#if defined(CONFIG_GANTRY_PATH_MAX_DECEL_MM_PER_S2)
    data_.gantry_path_max_decel_mm_per_s2 = CONFIG_GANTRY_PATH_MAX_DECEL_MM_PER_S2;
#else
    data_.gantry_path_max_decel_mm_per_s2 = 20000;
#endif

#if defined(CONFIG_GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2)
    data_.gantry_eip_assumed_stop_decel_mm_s2 = CONFIG_GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2;
#else
    data_.gantry_eip_assumed_stop_decel_mm_s2 = 300;
#endif

    // 2. X Axis
#if defined(CONFIG_AXIS_X_LEAD_MM_PER_REV)
    data_.axis_x_lead_mm_per_rev = (float)CONFIG_AXIS_X_LEAD_MM_PER_REV;
#else
    data_.axis_x_lead_mm_per_rev = 200.0f;
#endif

#if defined(CONFIG_AXIS_X_HARD_LIMIT_MIN_MM)
    data_.axis_x_hard_limit_min_mm = (float)CONFIG_AXIS_X_HARD_LIMIT_MIN_MM;
#else
    data_.axis_x_hard_limit_min_mm = 0.0f;
#endif

#if defined(CONFIG_AXIS_X_HARD_LIMIT_MAX_MM)
    data_.axis_x_hard_limit_max_mm = (float)CONFIG_AXIS_X_HARD_LIMIT_MAX_MM;
#else
    data_.axis_x_hard_limit_max_mm = 550.0f;
#endif

#if defined(CONFIG_AXIS_X_MAX_SPEED_MM_PER_S)
    data_.axis_x_max_speed_mm_per_s = (float)CONFIG_AXIS_X_MAX_SPEED_MM_PER_S;
#else
    data_.axis_x_max_speed_mm_per_s = 3333.3f;
#endif

#if defined(CONFIG_AXIS_X_ACCEL_MM_PER_S2)
    data_.axis_x_accel_mm_per_s2 = (float)CONFIG_AXIS_X_ACCEL_MM_PER_S2;
#else
    data_.axis_x_accel_mm_per_s2 = 20000.0f;
#endif

#if defined(CONFIG_AXIS_X_DECEL_MM_PER_S2)
    data_.axis_x_decel_mm_per_s2 = (float)CONFIG_AXIS_X_DECEL_MM_PER_S2;
#else
    data_.axis_x_decel_mm_per_s2 = 20000.0f;
#endif

#if defined(CONFIG_AXIS_X_POSITION_TOLERANCE_MM)
    data_.axis_x_position_tolerance_mm = (float)CONFIG_AXIS_X_POSITION_TOLERANCE_MM;
#else
    data_.axis_x_position_tolerance_mm = 0.08f;
#endif

#if defined(CONFIG_AXIS_X_ENCODER_PPR)
    data_.axis_x_encoder_ppr = CONFIG_AXIS_X_ENCODER_PPR;
#else
    data_.axis_x_encoder_ppr = 2097152;
#endif

#if defined(CONFIG_AXIS_X_MOTOR_REDUCER_RATIO)
    data_.axis_x_motor_reducer_ratio = (float)CONFIG_AXIS_X_MOTOR_REDUCER_RATIO;
#else
    data_.axis_x_motor_reducer_ratio = 5.0f;
#endif

#if defined(CONFIG_AXIS_X_HOMING_SPEED_PPS)
    data_.axis_x_homing_speed_pps = CONFIG_AXIS_X_HOMING_SPEED_PPS;
#else
    data_.axis_x_homing_speed_pps = 8000;
#endif

    // 3. Z Axis
#if defined(CONFIG_AXIS_Z_LEAD_MM_PER_REV)
    data_.axis_z_lead_mm_per_rev = (float)CONFIG_AXIS_Z_LEAD_MM_PER_REV;
#else
    data_.axis_z_lead_mm_per_rev = 20.0f;
#endif

#if defined(CONFIG_AXIS_Z_CRITICAL_RPM)
    data_.axis_z_critical_rpm = CONFIG_AXIS_Z_CRITICAL_RPM;
#else
    data_.axis_z_critical_rpm = 3000;
#endif

#if defined(CONFIG_AXIS_Z_HARD_LIMIT_MIN_MM)
    data_.axis_z_hard_limit_min_mm = (float)CONFIG_AXIS_Z_HARD_LIMIT_MIN_MM;
#else
    data_.axis_z_hard_limit_min_mm = 0.0f;
#endif

#if defined(CONFIG_AXIS_Z_HARD_LIMIT_MAX_MM)
    data_.axis_z_hard_limit_max_mm = (float)CONFIG_AXIS_Z_HARD_LIMIT_MAX_MM;
#else
    data_.axis_z_hard_limit_max_mm = 150.0f;
#endif

#if defined(CONFIG_AXIS_Z_MAX_SPEED_MM_PER_S)
    data_.axis_z_max_speed_mm_per_s = (float)CONFIG_AXIS_Z_MAX_SPEED_MM_PER_S;
#else
    data_.axis_z_max_speed_mm_per_s = 500.0f;
#endif

#if defined(CONFIG_AXIS_Z_ACCEL_MM_PER_S2)
    data_.axis_z_accel_mm_per_s2 = (float)CONFIG_AXIS_Z_ACCEL_MM_PER_S2;
#else
    data_.axis_z_accel_mm_per_s2 = 5000.0f;
#endif

#if defined(CONFIG_AXIS_Z_DECEL_MM_PER_S2)
    data_.axis_z_decel_mm_per_s2 = (float)CONFIG_AXIS_Z_DECEL_MM_PER_S2;
#else
    data_.axis_z_decel_mm_per_s2 = 5000.0f;
#endif

#if defined(CONFIG_AXIS_Z_POSITION_TOLERANCE_MM)
    data_.axis_z_position_tolerance_mm = (float)CONFIG_AXIS_Z_POSITION_TOLERANCE_MM;
#else
    data_.axis_z_position_tolerance_mm = 0.03f;
#endif

#if defined(CONFIG_AXIS_Z_ENCODER_PPR)
    data_.axis_z_encoder_ppr = CONFIG_AXIS_Z_ENCODER_PPR;
#else
    data_.axis_z_encoder_ppr = 2097152;
#endif

#if defined(CONFIG_AXIS_Z_MOTOR_REDUCER_RATIO)
    data_.axis_z_motor_reducer_ratio = (float)CONFIG_AXIS_Z_MOTOR_REDUCER_RATIO;
#else
    data_.axis_z_motor_reducer_ratio = 1.0f;
#endif

#if defined(CONFIG_AXIS_Z_HAS_MOTOR_BRAKE)
    data_.axis_z_has_motor_brake = true;
#else
    data_.axis_z_has_motor_brake = false;
#endif

#if defined(CONFIG_AXIS_Z_HOMING_SPEED_PPS)
    data_.axis_z_homing_speed_pps = CONFIG_AXIS_Z_HOMING_SPEED_PPS;
#else
    data_.axis_z_homing_speed_pps = 8000;
#endif

#if defined(CONFIG_GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM)
    data_.gantry_z_datum_offset_above_bed_mm = (float)CONFIG_GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM;
#else
    data_.gantry_z_datum_offset_above_bed_mm = 0.0f;
#endif

    // 4. Theta Axis
#if defined(CONFIG_AXIS_THETA_OUTPUT_GEAR_RATIO)
    data_.axis_theta_output_gear_ratio = (float)CONFIG_AXIS_THETA_OUTPUT_GEAR_RATIO;
#else
    data_.axis_theta_output_gear_ratio = 1.0f;
#endif

#if defined(CONFIG_AXIS_THETA_HARD_LIMIT_MIN_DEG)
    data_.axis_theta_hard_limit_min_deg = (float)CONFIG_AXIS_THETA_HARD_LIMIT_MIN_DEG;
#else
    data_.axis_theta_hard_limit_min_deg = -180.0f;
#endif

#if defined(CONFIG_AXIS_THETA_HARD_LIMIT_MAX_DEG)
    data_.axis_theta_hard_limit_max_deg = (float)CONFIG_AXIS_THETA_HARD_LIMIT_MAX_DEG;
#else
    data_.axis_theta_hard_limit_max_deg = 180.0f;
#endif

#if defined(CONFIG_AXIS_THETA_MAX_SPEED_DEG_PER_S)
    data_.axis_theta_max_speed_deg_per_s = (float)CONFIG_AXIS_THETA_MAX_SPEED_DEG_PER_S;
#else
    data_.axis_theta_max_speed_deg_per_s = 360.0f;
#endif

#if defined(CONFIG_AXIS_THETA_ACCEL_DEG_PER_S2)
    data_.axis_theta_accel_deg_per_s2 = (float)CONFIG_AXIS_THETA_ACCEL_DEG_PER_S2;
#else
    data_.axis_theta_accel_deg_per_s2 = 1800.0f;
#endif

#if defined(CONFIG_AXIS_THETA_DECEL_DEG_PER_S2)
    data_.axis_theta_decel_deg_per_s2 = (float)CONFIG_AXIS_THETA_DECEL_DEG_PER_S2;
#else
    data_.axis_theta_decel_deg_per_s2 = 1800.0f;
#endif

#if defined(CONFIG_AXIS_THETA_POSITION_TOLERANCE_DEG)
    data_.axis_theta_position_tolerance_deg = (float)CONFIG_AXIS_THETA_POSITION_TOLERANCE_DEG;
#else
    data_.axis_theta_position_tolerance_deg = 0.01f;
#endif

#if defined(CONFIG_AXIS_THETA_ENCODER_PPR)
    data_.axis_theta_encoder_ppr = CONFIG_AXIS_THETA_ENCODER_PPR;
#else
    data_.axis_theta_encoder_ppr = 36000;
#endif

#if defined(CONFIG_AXIS_THETA_MOTOR_REDUCER_RATIO)
    data_.axis_theta_motor_reducer_ratio = (float)CONFIG_AXIS_THETA_MOTOR_REDUCER_RATIO;
#else
    data_.axis_theta_motor_reducer_ratio = 1.0f;
#endif

#if defined(CONFIG_AXIS_THETA_HOMING_SPEED_PPS)
    data_.axis_theta_homing_speed_pps = CONFIG_AXIS_THETA_HOMING_SPEED_PPS;
#else
    data_.axis_theta_homing_speed_pps = 6000;
#endif

#if defined(CONFIG_GANTRY_THETA_SEQUENTIAL)
    data_.gantry_theta_sequential = true;
#else
    data_.gantry_theta_sequential = false;
#endif

    // 5. Geometry & Gripper
#if defined(CONFIG_GANTRY_Z_AXIS_Y_OFFSET_MM)
    data_.gantry_z_axis_y_offset_mm = (float)CONFIG_GANTRY_Z_AXIS_Y_OFFSET_MM;
#else
    data_.gantry_z_axis_y_offset_mm = 80.0f;
#endif

#if defined(CONFIG_GANTRY_THETA_X_OFFSET_MM)
    data_.gantry_theta_x_offset_mm = (float)CONFIG_GANTRY_THETA_X_OFFSET_MM;
#else
    data_.gantry_theta_x_offset_mm = -55.0f;
#endif

#if defined(CONFIG_GANTRY_GRIPPER_X_OFFSET_MM)
    data_.gantry_gripper_x_offset_mm = (float)CONFIG_GANTRY_GRIPPER_X_OFFSET_MM;
#else
    data_.gantry_gripper_x_offset_mm = 385.0f;
#endif

#if defined(CONFIG_GANTRY_GRIPPER_Z_OFFSET_MM)
    data_.gantry_gripper_z_offset_mm = (float)CONFIG_GANTRY_GRIPPER_Z_OFFSET_MM;
#else
    data_.gantry_gripper_z_offset_mm = 80.0f;
#endif

#if defined(CONFIG_GANTRY_SAFE_Z_HEIGHT_MM)
    data_.gantry_safe_z_height_mm = (float)CONFIG_GANTRY_SAFE_Z_HEIGHT_MM;
#else
    data_.gantry_safe_z_height_mm = 48.4f;
#endif

    // Bin-column X. Independent of the bring-up park (GANTRY_CAL_X_PARK_MM).
    data_.gantry_cal_x_park_mm = 10.0f;

#if defined(CONFIG_GANTRY_CAL_Z_PARK_MM)
    data_.gantry_cal_z_park_mm = (float)CONFIG_GANTRY_CAL_Z_PARK_MM;
#else
    data_.gantry_cal_z_park_mm = 90.8f;
#endif

#if defined(CONFIG_GANTRY_CAL_THETA_PARK_DEG)
    data_.gantry_cal_theta_park_deg = (float)CONFIG_GANTRY_CAL_THETA_PARK_DEG;
#else
    data_.gantry_cal_theta_park_deg = 90.0f;
#endif

#if defined(CONFIG_GANTRY_CONVEYOR_COLLISION_X_MIN_MM)
    data_.gantry_conveyor_collision_x_min_mm = (float)CONFIG_GANTRY_CONVEYOR_COLLISION_X_MIN_MM;
#else
    data_.gantry_conveyor_collision_x_min_mm = 95.0f;
#endif

#if defined(CONFIG_GANTRY_CONVEYOR_COLLISION_Z_MIN_MM)
    data_.gantry_conveyor_collision_z_min_mm = (float)CONFIG_GANTRY_CONVEYOR_COLLISION_Z_MIN_MM;
#else
    data_.gantry_conveyor_collision_z_min_mm = 115.0f;
#endif

#if defined(CONFIG_GANTRY_GRIPPER_OPEN_TIME_MS)
    data_.gantry_gripper_open_time_ms = CONFIG_GANTRY_GRIPPER_OPEN_TIME_MS;
#else
    data_.gantry_gripper_open_time_ms = 190;
#endif

#if defined(CONFIG_GANTRY_GRIPPER_CLOSE_TIME_MS)
    data_.gantry_gripper_close_time_ms = CONFIG_GANTRY_GRIPPER_CLOSE_TIME_MS;
#else
    data_.gantry_gripper_close_time_ms = 190;
#endif
}

} // namespace Config
