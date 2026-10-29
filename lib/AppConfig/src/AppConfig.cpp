#include "AppConfig.h"
#include "MemoryStorage.h"
#include "NvsStorage.h"
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

namespace {

struct PickClassKey {
  const char* jaw;
  const char* len;
  const char* height;
};

// NVS keys are at most 15 characters. Old cv_jaw1_ms / cell1_* keys are not read.
constexpr int kPickClassSlots = 10;
const PickClassKey kPickClassKeys[kPickClassSlots] = {
    {"cv_jaw_aaa_ms", "cell_aaa_len_mm", "cell_aaa_h_mm"},
    {"cv_jaw_aa_ms", "cell_aa_len_mm", "cell_aa_h_mm"},
    {"cv_jaw_d_ms", "cell_d_len_mm", "cell_d_h_mm"},
    {"cv_jaw_c_ms", "cell_c_len_mm", "cell_c_h_mm"},
    {"cv_jaw_9v_ms", "cell_9v_len_mm", "cell_9v_h_mm"},
    {"cv_jaw_cr123_ms", "cell_cr123_len", "cell_cr123_h_mm"},
    {"cv_jaw_18650_ms", "cell_18650_len", "cell_18650_h_mm"},
    {"cv_jaw_21700_ms", "cell_21700_len", "cell_21700_h_mm"},
    {"cv_jaw_26650_ms", "cell_26650_len", "cell_26650_h_mm"},
    {"cv_jaw_32650_ms", "cell_32650_len", "cell_32650_h_mm"},
};

int pickKeySlot(const char* key, int field) {
  for (int i = 0; i < kPickClassSlots; ++i) {
    const char* name = field == 0 ? kPickClassKeys[i].jaw
                       : field == 1 ? kPickClassKeys[i].len
                                    : kPickClassKeys[i].height;
    if (std::strcmp(key, name) == 0) {
      return i;
    }
  }
  return -1;
}

}  // namespace

namespace Config {

AppConfig& AppConfig::instance() {
    static AppConfig s_instance;
    return s_instance;
}

AppConfig::AppConfig() {
    loadDefaults();
}

void AppConfig::loadDefaults() {
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
    data_.gantry_safe_z_height_mm = 35.7f;
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

    // 6. Conveyor Intercept
#if defined(CONFIG_CONVEYOR_Y_CAM_MM)
    data_.conveyor_y_cam_mm = (float)CONFIG_CONVEYOR_Y_CAM_MM;
#else
    data_.conveyor_y_cam_mm = -711.0f;
#endif

#if defined(CONFIG_CONVEYOR_Y_PICK_MM)
    data_.conveyor_y_pick_mm = (float)CONFIG_CONVEYOR_Y_PICK_MM;
#else
    data_.conveyor_y_pick_mm = 0.0f;
#endif

#if defined(CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM)
    data_.conveyor_x_across_to_gantry_x_offset_mm = (float)CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM;
#else
    data_.conveyor_x_across_to_gantry_x_offset_mm = 245.0f;
#endif

#if defined(CONFIG_CONVEYOR_BELT_WIDTH_MM)
    data_.conveyor_belt_width_mm = (float)CONFIG_CONVEYOR_BELT_WIDTH_MM;
#else
    data_.conveyor_belt_width_mm = 300.0f;
#endif
    data_.conveyor_belt_near_x_mm = 225.0f;
    data_.conveyor_belt_far_x_mm = 500.0f;
    data_.conveyor_frame_near_x_mm = 183.0f;
    data_.conveyor_frame_far_x_mm = 500.0f;
    data_.test_battery_x_mm = 382.5f;

#if defined(CONFIG_CONVEYOR_Z_PICK_JOINT_MM)
    data_.conveyor_z_pick_joint_mm = (float)CONFIG_CONVEYOR_Z_PICK_JOINT_MM;
#else
    data_.conveyor_z_pick_joint_mm = 105.4f;
#endif

#if defined(CONFIG_PICK_DEFAULT_BELT_SPEED_MM_S)
    data_.pick_default_belt_speed_mm_s = (float)CONFIG_PICK_DEFAULT_BELT_SPEED_MM_S;
#else
    data_.pick_default_belt_speed_mm_s = 1524.0f;
#endif

#if defined(CONFIG_TAU_MIN_US)
    data_.tau_min_us = CONFIG_TAU_MIN_US;
#else
    data_.tau_min_us = 100000;
#endif

#if defined(CONFIG_TAU_MAX_US)
    data_.tau_max_us = CONFIG_TAU_MAX_US;
#else
    data_.tau_max_us = 5000000;
#endif

    data_.pick_vision_latency_ms = 40;
    data_.pick_valve_ms = 30;
    data_.pick_valve_on_ms = 8;
    data_.pick_valve_off_ms = 17;
    data_.pick_trim_von_ms = 0;
    data_.pick_trim_voff_ms = 0;
    data_.pick_trim_close_ms = 0;
    data_.pick_trim_z_ms = 0;
    data_.pick_trim_grip_ms = 0;
    data_.pick_trim_hold_ms = 0;
    data_.pick_offset_plunge_ms = 0;
    data_.pick_offset_close_ms = 0;
    data_.pick_offset_retract_ms = 0;
    data_.eip_w5500_spi_khz = 10000;
    data_.display_tft_enable = 1;
    data_.eip_txwr_split = 0;
    data_.eip_spi_dma = 0;
    data_.log_uart_async = 1;
    data_.eip_scope_gpio = -1;
    data_.pick_bench_enable = false;
    for (int i = 0; i < kPickClassSlots; ++i) {
      data_.pick_jaw_ms[i] = 0;
    }
    data_.pick_jaw_offset_deg = 0.0f;
    data_.pick_dedupe_mm = 40.0f;
    const float len_mm[kPickClassSlots] = {44.5f, 50.5f, 61.5f, 50.0f, 48.5f,
                                            34.5f, 65.0f, 70.0f, 65.0f, 65.0f};
    const float height_mm[kPickClassSlots] = {10.5f, 14.5f, 34.2f, 26.2f, 26.5f,
                                               17.0f, 18.3f, 21.2f, 26.5f, 32.0f};
    for (int i = 0; i < kPickClassSlots; ++i) {
      data_.pick_cell_len_mm[i] = len_mm[i];
      data_.pick_cell_h_mm[i] = height_mm[i];
    }
    data_.pick_finger_open_mm = 80.0f;
    data_.pick_finger_thick_mm = 10.0f;
    data_.pick_finger_width_mm = 50.0f;
    data_.pick_jaw_speed_mm_s = 210.0f;
    data_.pick_guard_ms = 15;
    data_.pick_dropin_enable = false;
    data_.pick_drop_any_theta = false;
    data_.pick_stage_x_mm = 320.0f;
    data_.pick_speed_mm_s = 2000;
    data_.pick_accel_mm_s2 = 3000;
    data_.pick_theta_speed_deg_s = 360.0f;
    data_.pick_theta_accel_deg_s2 = 1800.0f;

    // 7. Homing & Calibration
#if defined(CONFIG_EIP_HOME_CAL_SPEED_MM_S)
    data_.eip_home_cal_speed_mm_s = (float)CONFIG_EIP_HOME_CAL_SPEED_MM_S;
#else
    data_.eip_home_cal_speed_mm_s = 50.0f;
#endif

#if defined(CONFIG_EIP_HOME_THETA_SPEED_DEG_S)
    data_.eip_home_theta_speed_deg_s = (float)CONFIG_EIP_HOME_THETA_SPEED_DEG_S;
#else
    data_.eip_home_theta_speed_deg_s = 15.0f;
#endif

#if defined(CONFIG_EIP_HOME_CAL_ACCEL_MM_S2)
    data_.eip_home_cal_accel_mm_s2 = (float)CONFIG_EIP_HOME_CAL_ACCEL_MM_S2;
#else
    data_.eip_home_cal_accel_mm_s2 = 2000.0f;
#endif

#if defined(CONFIG_EIP_CREEP_SPEED_MM_S)
    data_.eip_creep_speed_mm_s = (float)CONFIG_EIP_CREEP_SPEED_MM_S;
#else
    data_.eip_creep_speed_mm_s = 1.0f;
#endif

#if defined(CONFIG_GANTRY_CALIBRATION_TIMEOUT_MS)
    data_.gantry_calibration_timeout_ms = CONFIG_GANTRY_CALIBRATION_TIMEOUT_MS;
#else
    data_.gantry_calibration_timeout_ms = 20000;
#endif

#if defined(CONFIG_GANTRY_TRAVEL_MEASUREMENT_TIMEOUT_MS)
    data_.gantry_travel_measurement_timeout_ms = CONFIG_GANTRY_TRAVEL_MEASUREMENT_TIMEOUT_MS;
#else
    data_.gantry_travel_measurement_timeout_ms = 20000;
#endif

    // 8. Network (LAN8720)
#if defined(CONFIG_ETH_USE_STATIC_IP)
    data_.eth_use_static_ip = true;
#else
    data_.eth_use_static_ip = false;
#endif

#if defined(CONFIG_ETH_STATIC_IP)
    std::strncpy(data_.eth_static_ip, CONFIG_ETH_STATIC_IP, sizeof(data_.eth_static_ip) - 1);
#else
    std::strncpy(data_.eth_static_ip, "192.168.1.100", sizeof(data_.eth_static_ip) - 1);
#endif

#if defined(CONFIG_ETH_STATIC_GATEWAY)
    std::strncpy(data_.eth_static_gateway, CONFIG_ETH_STATIC_GATEWAY, sizeof(data_.eth_static_gateway) - 1);
#else
    std::strncpy(data_.eth_static_gateway, "192.168.1.5", sizeof(data_.eth_static_gateway) - 1);
#endif

#if defined(CONFIG_ETH_STATIC_NETMASK)
    std::strncpy(data_.eth_static_netmask, CONFIG_ETH_STATIC_NETMASK, sizeof(data_.eth_static_netmask) - 1);
#else
    std::strncpy(data_.eth_static_netmask, "255.255.255.0", sizeof(data_.eth_static_netmask) - 1);
#endif

#if defined(CONFIG_ETH_IP_WAIT_TIMEOUT_MS)
    data_.eth_ip_wait_timeout_ms = CONFIG_ETH_IP_WAIT_TIMEOUT_MS;
#else
    data_.eth_ip_wait_timeout_ms = 1500;
#endif

    // 9. Display
    data_.display_orientation = 0; // 0 = Landscape (320x240)
    data_.display_brightness = 100; // 100%
    data_.display_backlight_timeout_s = 10;

    // 10. Console Security (Developer)
#if defined(CONFIG_CONSOLE_TCP_AUTH_ENABLE)
    data_.console_tcp_auth_enable = true;
#else
    data_.console_tcp_auth_enable = false;
#endif

#if defined(CONFIG_CONSOLE_TCP_PASSWORD)
    std::strncpy(data_.console_tcp_password, CONFIG_CONSOLE_TCP_PASSWORD, sizeof(data_.console_tcp_password) - 1);
#else
    std::strncpy(data_.console_tcp_password, "LTU_1932", sizeof(data_.console_tcp_password) - 1);
#endif

#if defined(CONFIG_CONSOLE_TCP_AUTH_MAX_TRIES)
    data_.console_tcp_auth_max_tries = CONFIG_CONSOLE_TCP_AUTH_MAX_TRIES;
#else
    data_.console_tcp_auth_max_tries = 3;
#endif

#if defined(CONFIG_CONSOLE_TCP_AUTH_REMEMBER_S)
    data_.console_tcp_auth_remember_s = CONFIG_CONSOLE_TCP_AUTH_REMEMBER_S;
#else
    data_.console_tcp_auth_remember_s = 60;
#endif

#if defined(CONFIG_CONSOLE_TCP_AUTH_REMEMBER_MAX)
    data_.console_tcp_auth_remember_max = CONFIG_CONSOLE_TCP_AUTH_REMEMBER_MAX;
#else
    data_.console_tcp_auth_remember_max = 4;
#endif

    // 11. Console Options (Developer)
#if defined(CONFIG_CONSOLE_TCP_ENABLE)
    data_.console_tcp_enable = true;
#else
    data_.console_tcp_enable = false;
#endif

#if defined(CONFIG_CONSOLE_UART_ENABLE)
    data_.console_uart_enable = true;
#else
    data_.console_uart_enable = false;
#endif

#if defined(CONFIG_CONSOLE_TCP_PORT)
    data_.console_tcp_port = CONFIG_CONSOLE_TCP_PORT;
#else
    data_.console_tcp_port = 2323;
#endif

#if defined(CONFIG_CONSOLE_TCP_LOG_ENABLE)
    data_.console_tcp_log_enable = true;
#else
    data_.console_tcp_log_enable = false;
#endif

#if defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_ERR)
    data_.console_tcp_log_level = 0;
#elif defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_WARN)
    data_.console_tcp_log_level = 1;
#elif defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_DBUG)
    data_.console_tcp_log_level = 3;
#else
    data_.console_tcp_log_level = 2; // INFO
#endif

#if defined(CONFIG_OTA_SERVER_PORT)
    data_.ota_server_port = CONFIG_OTA_SERVER_PORT;
#else
    data_.ota_server_port = 8032;
#endif

    // 12. EtherNet/IP (Developer)
#if defined(CONFIG_EIP_SCANNER_ENABLED)
    data_.eip_scanner_enabled = true;
#else
    data_.eip_scanner_enabled = false;
#endif

#if defined(CONFIG_EIP_AXIS_X)
    data_.eip_axis_x = true;
#else
    data_.eip_axis_x = false;
#endif

#if defined(CONFIG_EIP_AXIS_Z)
    data_.eip_axis_z = true;
#else
    data_.eip_axis_z = false;
#endif

#if defined(CONFIG_EIP_AXIS_THETA)
    data_.eip_axis_theta = true;
#else
    data_.eip_axis_theta = false;
#endif

#if defined(CONFIG_EIP_TARGET_IP_X)
    std::strncpy(data_.eip_target_ip_x, CONFIG_EIP_TARGET_IP_X, sizeof(data_.eip_target_ip_x) - 1);
#else
    std::strncpy(data_.eip_target_ip_x, "192.168.1.20", sizeof(data_.eip_target_ip_x) - 1);
#endif

#if defined(CONFIG_EIP_TARGET_IP_Z)
    std::strncpy(data_.eip_target_ip_z, CONFIG_EIP_TARGET_IP_Z, sizeof(data_.eip_target_ip_z) - 1);
#else
    std::strncpy(data_.eip_target_ip_z, "192.168.1.21", sizeof(data_.eip_target_ip_z) - 1);
#endif

#if defined(CONFIG_EIP_TARGET_IP_THETA)
    std::strncpy(data_.eip_target_ip_theta, CONFIG_EIP_TARGET_IP_THETA, sizeof(data_.eip_target_ip_theta) - 1);
#else
    std::strncpy(data_.eip_target_ip_theta, "192.168.1.23", sizeof(data_.eip_target_ip_theta) - 1);
#endif

#if defined(CONFIG_EIP_W5500_IP)
    std::strncpy(data_.eip_w5500_ip, CONFIG_EIP_W5500_IP, sizeof(data_.eip_w5500_ip) - 1);
#else
    std::strncpy(data_.eip_w5500_ip, "192.168.1.10", sizeof(data_.eip_w5500_ip) - 1);
#endif

#if defined(CONFIG_EIP_W5500_SUBNET)
    std::strncpy(data_.eip_w5500_subnet, CONFIG_EIP_W5500_SUBNET, sizeof(data_.eip_w5500_subnet) - 1);
#else
    std::strncpy(data_.eip_w5500_subnet, "255.255.255.0", sizeof(data_.eip_w5500_subnet) - 1);
#endif

#if defined(CONFIG_EIP_W5500_GATEWAY)
    std::strncpy(data_.eip_w5500_gateway, CONFIG_EIP_W5500_GATEWAY, sizeof(data_.eip_w5500_gateway) - 1);
#else
    std::strncpy(data_.eip_w5500_gateway, "192.168.1.1", sizeof(data_.eip_w5500_gateway) - 1);
#endif

#if defined(CONFIG_EIP_W5500_SPI_HZ)
    data_.eip_w5500_spi_hz = CONFIG_EIP_W5500_SPI_HZ;
#else
    data_.eip_w5500_spi_hz = 20000000;
#endif

#if defined(CONFIG_EIP_AXIS_X_PUU_PER_MM)
    data_.eip_axis_x_puu_per_mm = (float)CONFIG_EIP_AXIS_X_PUU_PER_MM;
#else
    data_.eip_axis_x_puu_per_mm = 52428.8f;
#endif

#if defined(CONFIG_EIP_AXIS_Z_PUU_PER_MM)
    data_.eip_axis_z_puu_per_mm = (float)CONFIG_EIP_AXIS_Z_PUU_PER_MM;
#else
    data_.eip_axis_z_puu_per_mm = 104857.6f;
#endif

#if defined(CONFIG_EIP_AXIS_THETA_PUU_PER_DEG)
    data_.eip_axis_theta_puu_per_deg = (float)CONFIG_EIP_AXIS_THETA_PUU_PER_DEG;
#else
    data_.eip_axis_theta_puu_per_deg = 10000.0f;
#endif

#if defined(CONFIG_EIP_X_RPI_US)
    data_.eip_x_rpi_us = CONFIG_EIP_X_RPI_US;
#else
    data_.eip_x_rpi_us = 2000;
#endif

#if defined(CONFIG_EIP_THETA_RPI_US)
    data_.eip_theta_rpi_us = CONFIG_EIP_THETA_RPI_US;
#else
    data_.eip_theta_rpi_us = 2000;
#endif

#if defined(CONFIG_EIP_ENDSTOP_FROM_MCP)
    data_.eip_endstop_source = 1;
#else
    data_.eip_endstop_source = 0;
#endif

#if defined(CONFIG_AXIS_THETA_DRIVE_ABS_MIN_DEG)
    data_.axis_theta_drive_abs_min_deg = (float)CONFIG_AXIS_THETA_DRIVE_ABS_MIN_DEG;
#else
    data_.axis_theta_drive_abs_min_deg = -180.0f;
#endif

#if defined(CONFIG_AXIS_THETA_DRIVE_ABS_MAX_DEG)
    data_.axis_theta_drive_abs_max_deg = (float)CONFIG_AXIS_THETA_DRIVE_ABS_MAX_DEG;
#else
    data_.axis_theta_drive_abs_max_deg = 180.0f;
#endif

#if defined(CONFIG_EIP_HCS01_ENG_IP)
    std::strncpy(data_.hcs01_eng_ip, CONFIG_EIP_HCS01_ENG_IP, sizeof(data_.hcs01_eng_ip) - 1);
#else
    std::strncpy(data_.hcs01_eng_ip, "192.168.1.22", sizeof(data_.hcs01_eng_ip) - 1);
#endif

    // 13. Developer Mode Password
#if defined(CONFIG_DEV_MODE_PASSWORD)
    std::strncpy(data_.dev_mode_password, CONFIG_DEV_MODE_PASSWORD, sizeof(data_.dev_mode_password) - 1);
#else
    std::strncpy(data_.dev_mode_password, "DEV_2026", sizeof(data_.dev_mode_password) - 1);
#endif
}

bool AppConfig::init(std::unique_ptr<IConfigStorage> storage) {
    if (storage) {
        storage_ = std::move(storage);
    } else {
#if defined(ESP_PLATFORM)
        storage_ = std::make_unique<NvsStorage>("gantry_cfg");
#else
        storage_ = std::make_unique<MemoryStorage>();
#endif
    }
    if (!storage_->init()) {
        return false;
    }
    return load();
}

bool AppConfig::load() {
    if (!storage_) return false;

    // Load every parameter from storage if present
    storage_->getInt32("spd_mm_s", data_.gantry_default_speed_mm_per_s);
    storage_->getInt32("acc_mm_s2", data_.gantry_default_accel_mm_per_s2);
    storage_->getInt32("dec_mm_s2", data_.gantry_default_decel_mm_per_s2);
    storage_->getInt32("th_spd_deg_s", data_.gantry_default_speed_deg_per_s);
    storage_->getInt32("th_acc_deg_s2", data_.gantry_default_accel_deg_per_s2);
    storage_->getInt32("th_dec_deg_s2", data_.gantry_default_decel_deg_per_s2);
    storage_->getInt32("ceil_spd_mm_s", data_.gantry_path_max_speed_mm_per_s);
    storage_->getInt32("ceil_acc_mm_s2", data_.gantry_path_max_accel_mm_per_s2);
    storage_->getInt32("ceil_dec_mm_s2", data_.gantry_path_max_decel_mm_per_s2);
    storage_->getInt32("eip_stop_dec", data_.gantry_eip_assumed_stop_decel_mm_s2);

    storage_->getFloat("x_lead_mm", data_.axis_x_lead_mm_per_rev);
    storage_->getFloat("x_lim_min_mm", data_.axis_x_hard_limit_min_mm);
    storage_->getFloat("x_lim_max_mm", data_.axis_x_hard_limit_max_mm);
    storage_->getFloat("x_max_spd_mm_s", data_.axis_x_max_speed_mm_per_s);
    storage_->getFloat("x_acc_mm_s2", data_.axis_x_accel_mm_per_s2);
    storage_->getFloat("x_dec_mm_s2", data_.axis_x_decel_mm_per_s2);
    storage_->getFloat("x_tol_mm", data_.axis_x_position_tolerance_mm);
    storage_->getInt32("x_enc_ppr", data_.axis_x_encoder_ppr);
    storage_->getFloat("x_gear_ratio", data_.axis_x_motor_reducer_ratio);
    storage_->getInt32("x_home_pps", data_.axis_x_homing_speed_pps);

    storage_->getFloat("z_lead_mm", data_.axis_z_lead_mm_per_rev);
    storage_->getInt32("z_crit_rpm", data_.axis_z_critical_rpm);
    storage_->getFloat("z_lim_min_mm", data_.axis_z_hard_limit_min_mm);
    storage_->getFloat("z_lim_max_mm", data_.axis_z_hard_limit_max_mm);
    storage_->getFloat("z_max_spd_mm_s", data_.axis_z_max_speed_mm_per_s);
    storage_->getFloat("z_acc_mm_s2", data_.axis_z_accel_mm_per_s2);
    storage_->getFloat("z_dec_mm_s2", data_.axis_z_decel_mm_per_s2);
    storage_->getFloat("z_tol_mm", data_.axis_z_position_tolerance_mm);
    storage_->getInt32("z_enc_ppr", data_.axis_z_encoder_ppr);
    storage_->getFloat("z_gear_ratio", data_.axis_z_motor_reducer_ratio);
    storage_->getBool("z_has_brake", data_.axis_z_has_motor_brake);
    storage_->getInt32("z_home_pps", data_.axis_z_homing_speed_pps);
    storage_->getFloat("z_datum_mm", data_.gantry_z_datum_offset_above_bed_mm);

    storage_->getFloat("th_gear_ratio", data_.axis_theta_output_gear_ratio);
    storage_->getFloat("th_lim_min_deg", data_.axis_theta_hard_limit_min_deg);
    storage_->getFloat("th_lim_max_deg", data_.axis_theta_hard_limit_max_deg);
    storage_->getFloat("th_spd_max", data_.axis_theta_max_speed_deg_per_s);
    storage_->getFloat("th_acc_deg", data_.axis_theta_accel_deg_per_s2);
    storage_->getFloat("th_dec_deg", data_.axis_theta_decel_deg_per_s2);
    storage_->getFloat("th_tol_deg", data_.axis_theta_position_tolerance_deg);
    storage_->getInt32("th_enc_ppr", data_.axis_theta_encoder_ppr);
    storage_->getFloat("th_reducer", data_.axis_theta_motor_reducer_ratio);
    storage_->getInt32("th_home_pps", data_.axis_theta_homing_speed_pps);
    storage_->getBool("th_sequential", data_.gantry_theta_sequential);

    storage_->getFloat("geom_z_y_mm", data_.gantry_z_axis_y_offset_mm);
    storage_->getFloat("geom_th_x_mm", data_.gantry_theta_x_offset_mm);
    storage_->getFloat("geom_grp_x_mm", data_.gantry_gripper_x_offset_mm);
    storage_->getFloat("geom_grp_z_mm", data_.gantry_gripper_z_offset_mm);
    storage_->getFloat("geom_safe_z_mm", data_.gantry_safe_z_height_mm);
    storage_->getFloat("geom_cal_x_mm", data_.gantry_cal_x_park_mm);
    storage_->getFloat("geom_cal_z_mm", data_.gantry_cal_z_park_mm);
    storage_->getFloat("geom_cal_th_deg", data_.gantry_cal_theta_park_deg);
    storage_->getFloat("col_x_min_mm", data_.gantry_conveyor_collision_x_min_mm);
    storage_->getFloat("col_z_min_mm", data_.gantry_conveyor_collision_z_min_mm);
    storage_->getInt32("grp_open_ms", data_.gantry_gripper_open_time_ms);
    storage_->getInt32("grp_close_ms", data_.gantry_gripper_close_time_ms);

    storage_->getFloat("cv_cam_y_mm", data_.conveyor_y_cam_mm);
    storage_->getFloat("cv_pick_y_mm", data_.conveyor_y_pick_mm);
    storage_->getFloat("cv_x_off_mm", data_.conveyor_x_across_to_gantry_x_offset_mm);
    storage_->getFloat("cv_belt_w_mm", data_.conveyor_belt_width_mm);
    storage_->getFloat("cv_belt_near_mm", data_.conveyor_belt_near_x_mm);
    storage_->getFloat("cv_belt_far_mm", data_.conveyor_belt_far_x_mm);
    storage_->getFloat("cv_cnv_near_mm", data_.conveyor_frame_near_x_mm);
    storage_->getFloat("cv_cnv_far_mm", data_.conveyor_frame_far_x_mm);
    storage_->getFloat("batt_x", data_.test_battery_x_mm);
    storage_->getFloat("cv_z_pick_mm", data_.conveyor_z_pick_joint_mm);
    storage_->getFloat("cv_belt_spd", data_.pick_default_belt_speed_mm_s);
    storage_->getInt32("cv_tau_min_us", data_.tau_min_us);
    storage_->getInt32("cv_tau_max_us", data_.tau_max_us);
    storage_->getInt32("cv_vis_ms", data_.pick_vision_latency_ms);
    storage_->getInt32("cv_valve_ms", data_.pick_valve_ms);
    storage_->getInt32("pk_von_ms", data_.pick_valve_on_ms);
    storage_->getInt32("pk_voff_ms", data_.pick_valve_off_ms);
    storage_->getInt32("pk_tr_von", data_.pick_trim_von_ms);
    storage_->getInt32("pk_tr_voff", data_.pick_trim_voff_ms);
    storage_->getInt32("pk_tr_cls", data_.pick_trim_close_ms);
    storage_->getInt32("pk_tr_z", data_.pick_trim_z_ms);
    storage_->getInt32("pk_tr_grp", data_.pick_trim_grip_ms);
    storage_->getInt32("pk_tr_hld", data_.pick_trim_hold_ms);
    storage_->getInt32("pk_off_plunge", data_.pick_offset_plunge_ms);
    storage_->getInt32("pk_off_close", data_.pick_offset_close_ms);
    storage_->getInt32("pk_off_retract", data_.pick_offset_retract_ms);
    storage_->getInt32("eip_spi_khz", data_.eip_w5500_spi_khz);
    storage_->getInt32("disp_tft_en", data_.display_tft_enable);
    storage_->getInt32("eip_txwr_split", data_.eip_txwr_split);
    storage_->getInt32("eip_spi_dma", data_.eip_spi_dma);
    storage_->getInt32("log_uart_async", data_.log_uart_async);
    storage_->getInt32("eip_scope_gpio", data_.eip_scope_gpio);
    storage_->getBool("pk_bench", data_.pick_bench_enable);
    for (int i = 0; i < kPickClassSlots; ++i) {
      storage_->getInt32(kPickClassKeys[i].jaw, data_.pick_jaw_ms[i]);
    }
    storage_->getFloat("cv_jaw_deg", data_.pick_jaw_offset_deg);
    storage_->getFloat("cv_dedupe_mm", data_.pick_dedupe_mm);
    for (int i = 0; i < kPickClassSlots; ++i) {
      storage_->getFloat(kPickClassKeys[i].len, data_.pick_cell_len_mm[i]);
      storage_->getFloat(kPickClassKeys[i].height, data_.pick_cell_h_mm[i]);
    }
    storage_->getFloat("fin_open_mm", data_.pick_finger_open_mm);
    storage_->getFloat("fin_wid_mm", data_.pick_finger_thick_mm);
    storage_->getFloat("fin_len_mm", data_.pick_finger_width_mm);
    storage_->getFloat("fin_spd_mm_s", data_.pick_jaw_speed_mm_s);
    storage_->getInt32("pk_guard_ms", data_.pick_guard_ms);
    storage_->getBool("pk_dropin", data_.pick_dropin_enable);
    storage_->getBool("pk_drop_any_th", data_.pick_drop_any_theta);
    storage_->getFloat("pk_stage_x_mm", data_.pick_stage_x_mm);
    storage_->getInt32("pk_spd_mm_s", data_.pick_speed_mm_s);
    storage_->getInt32("pk_acc_mm_s2", data_.pick_accel_mm_s2);
    storage_->getFloat("pk_th_spd", data_.pick_theta_speed_deg_s);
    storage_->getFloat("pk_th_acc", data_.pick_theta_accel_deg_s2);

    storage_->getFloat("cal_spd_mm_s", data_.eip_home_cal_speed_mm_s);
    storage_->getFloat("cal_th_spd_deg", data_.eip_home_theta_speed_deg_s);
    storage_->getFloat("cal_acc_mm_s2", data_.eip_home_cal_accel_mm_s2);
    storage_->getFloat("cal_creep_mm_s", data_.eip_creep_speed_mm_s);
    storage_->getInt32("cal_timeout_ms", data_.gantry_calibration_timeout_ms);
    storage_->getInt32("trv_timeout_ms", data_.gantry_travel_measurement_timeout_ms);

    storage_->getBool("eth_static", data_.eth_use_static_ip);
    storage_->getString("eth_ip", data_.eth_static_ip, sizeof(data_.eth_static_ip));
    storage_->getString("eth_gw", data_.eth_static_gateway, sizeof(data_.eth_static_gateway));
    storage_->getString("eth_netmask", data_.eth_static_netmask, sizeof(data_.eth_static_netmask));
    storage_->getInt32("eth_timeout_ms", data_.eth_ip_wait_timeout_ms);

    storage_->getInt32("disp_orient", data_.display_orientation);
    storage_->getInt32("disp_bright", data_.display_brightness);
    storage_->getInt32("disp_bl_to_s", data_.display_backlight_timeout_s);

    storage_->getBool("con_auth", data_.console_tcp_auth_enable);
    storage_->getString("con_pw", data_.console_tcp_password, sizeof(data_.console_tcp_password));
    storage_->getInt32("con_tries", data_.console_tcp_auth_max_tries);
    storage_->getInt32("con_ttl_s", data_.console_tcp_auth_remember_s);
    storage_->getInt32("con_max_peers", data_.console_tcp_auth_remember_max);

    storage_->getBool("con_tcp_en", data_.console_tcp_enable);
    storage_->getBool("con_uart_en", data_.console_uart_enable);
    storage_->getInt32("con_port", data_.console_tcp_port);
    storage_->getBool("con_log_en", data_.console_tcp_log_enable);
    storage_->getInt32("con_log_lvl", data_.console_tcp_log_level);
    storage_->getInt32("ota_port", data_.ota_server_port);

    storage_->getBool("eip_en", data_.eip_scanner_enabled);
    storage_->getBool("eip_x_en", data_.eip_axis_x);
    storage_->getBool("eip_z_en", data_.eip_axis_z);
    storage_->getBool("eip_th_en", data_.eip_axis_theta);
    storage_->getString("eip_ip_x", data_.eip_target_ip_x, sizeof(data_.eip_target_ip_x));
    storage_->getString("eip_ip_z", data_.eip_target_ip_z, sizeof(data_.eip_target_ip_z));
    storage_->getString("eip_ip_th", data_.eip_target_ip_theta, sizeof(data_.eip_target_ip_theta));
    storage_->getString("w5500_ip", data_.eip_w5500_ip, sizeof(data_.eip_w5500_ip));
    storage_->getString("w5500_sub", data_.eip_w5500_subnet, sizeof(data_.eip_w5500_subnet));
    storage_->getString("w5500_gw", data_.eip_w5500_gateway, sizeof(data_.eip_w5500_gateway));
    storage_->getInt32("w5500_spi_hz", data_.eip_w5500_spi_hz);
    storage_->getFloat("eip_x_puu_mm", data_.eip_axis_x_puu_per_mm);
    storage_->getFloat("eip_z_puu_mm", data_.eip_axis_z_puu_per_mm);
    storage_->getFloat("eip_th_puu_deg", data_.eip_axis_theta_puu_per_deg);
    storage_->getInt32("eip_x_rpi_us", data_.eip_x_rpi_us);
    storage_->getInt32("eip_th_rpi_us", data_.eip_theta_rpi_us);
    storage_->getInt32("eip_endstop_src", data_.eip_endstop_source);
    storage_->getFloat("th_abs_min_deg", data_.axis_theta_drive_abs_min_deg);
    storage_->getFloat("th_abs_max_deg", data_.axis_theta_drive_abs_max_deg);
    storage_->getString("hcs01_eng_ip", data_.hcs01_eng_ip, sizeof(data_.hcs01_eng_ip));

    storage_->getString("dev_pw", data_.dev_mode_password, sizeof(data_.dev_mode_password));

    return true;
}

bool AppConfig::save() {
    if (!storage_) return false;

    storage_->setInt32("spd_mm_s", data_.gantry_default_speed_mm_per_s);
    storage_->setInt32("acc_mm_s2", data_.gantry_default_accel_mm_per_s2);
    storage_->setInt32("dec_mm_s2", data_.gantry_default_decel_mm_per_s2);
    storage_->setInt32("th_spd_deg_s", data_.gantry_default_speed_deg_per_s);
    storage_->setInt32("th_acc_deg_s2", data_.gantry_default_accel_deg_per_s2);
    storage_->setInt32("th_dec_deg_s2", data_.gantry_default_decel_deg_per_s2);
    storage_->setInt32("ceil_spd_mm_s", data_.gantry_path_max_speed_mm_per_s);
    storage_->setInt32("ceil_acc_mm_s2", data_.gantry_path_max_accel_mm_per_s2);
    storage_->setInt32("ceil_dec_mm_s2", data_.gantry_path_max_decel_mm_per_s2);
    storage_->setInt32("eip_stop_dec", data_.gantry_eip_assumed_stop_decel_mm_s2);

    storage_->setFloat("x_lead_mm", data_.axis_x_lead_mm_per_rev);
    storage_->setFloat("x_lim_min_mm", data_.axis_x_hard_limit_min_mm);
    storage_->setFloat("x_lim_max_mm", data_.axis_x_hard_limit_max_mm);
    storage_->setFloat("x_max_spd_mm_s", data_.axis_x_max_speed_mm_per_s);
    storage_->setFloat("x_acc_mm_s2", data_.axis_x_accel_mm_per_s2);
    storage_->setFloat("x_dec_mm_s2", data_.axis_x_decel_mm_per_s2);
    storage_->setFloat("x_tol_mm", data_.axis_x_position_tolerance_mm);
    storage_->setInt32("x_enc_ppr", data_.axis_x_encoder_ppr);
    storage_->setFloat("x_gear_ratio", data_.axis_x_motor_reducer_ratio);
    storage_->setInt32("x_home_pps", data_.axis_x_homing_speed_pps);

    storage_->setFloat("z_lead_mm", data_.axis_z_lead_mm_per_rev);
    storage_->setInt32("z_crit_rpm", data_.axis_z_critical_rpm);
    storage_->setFloat("z_lim_min_mm", data_.axis_z_hard_limit_min_mm);
    storage_->setFloat("z_lim_max_mm", data_.axis_z_hard_limit_max_mm);
    storage_->setFloat("z_max_spd_mm_s", data_.axis_z_max_speed_mm_per_s);
    storage_->setFloat("z_acc_mm_s2", data_.axis_z_accel_mm_per_s2);
    storage_->setFloat("z_dec_mm_s2", data_.axis_z_decel_mm_per_s2);
    storage_->setFloat("z_tol_mm", data_.axis_z_position_tolerance_mm);
    storage_->setInt32("z_enc_ppr", data_.axis_z_encoder_ppr);
    storage_->setFloat("z_gear_ratio", data_.axis_z_motor_reducer_ratio);
    storage_->setBool("z_has_brake", data_.axis_z_has_motor_brake);
    storage_->setInt32("z_home_pps", data_.axis_z_homing_speed_pps);
    storage_->setFloat("z_datum_mm", data_.gantry_z_datum_offset_above_bed_mm);

    storage_->setFloat("th_gear_ratio", data_.axis_theta_output_gear_ratio);
    storage_->setFloat("th_lim_min_deg", data_.axis_theta_hard_limit_min_deg);
    storage_->setFloat("th_lim_max_deg", data_.axis_theta_hard_limit_max_deg);
    storage_->setFloat("th_spd_max", data_.axis_theta_max_speed_deg_per_s);
    storage_->setFloat("th_acc_deg", data_.axis_theta_accel_deg_per_s2);
    storage_->setFloat("th_dec_deg", data_.axis_theta_decel_deg_per_s2);
    storage_->setFloat("th_tol_deg", data_.axis_theta_position_tolerance_deg);
    storage_->setInt32("th_enc_ppr", data_.axis_theta_encoder_ppr);
    storage_->setFloat("th_reducer", data_.axis_theta_motor_reducer_ratio);
    storage_->setInt32("th_home_pps", data_.axis_theta_homing_speed_pps);
    storage_->setBool("th_sequential", data_.gantry_theta_sequential);

    storage_->setFloat("geom_z_y_mm", data_.gantry_z_axis_y_offset_mm);
    storage_->setFloat("geom_th_x_mm", data_.gantry_theta_x_offset_mm);
    storage_->setFloat("geom_grp_x_mm", data_.gantry_gripper_x_offset_mm);
    storage_->setFloat("geom_grp_z_mm", data_.gantry_gripper_z_offset_mm);
    storage_->setFloat("geom_safe_z_mm", data_.gantry_safe_z_height_mm);
    storage_->setFloat("geom_cal_x_mm", data_.gantry_cal_x_park_mm);
    storage_->setFloat("geom_cal_z_mm", data_.gantry_cal_z_park_mm);
    storage_->setFloat("geom_cal_th_deg", data_.gantry_cal_theta_park_deg);
    storage_->setFloat("col_x_min_mm", data_.gantry_conveyor_collision_x_min_mm);
    storage_->setFloat("col_z_min_mm", data_.gantry_conveyor_collision_z_min_mm);
    storage_->setInt32("grp_open_ms", data_.gantry_gripper_open_time_ms);
    storage_->setInt32("grp_close_ms", data_.gantry_gripper_close_time_ms);

    storage_->setFloat("cv_cam_y_mm", data_.conveyor_y_cam_mm);
    storage_->setFloat("cv_pick_y_mm", data_.conveyor_y_pick_mm);
    storage_->setFloat("cv_x_off_mm", data_.conveyor_x_across_to_gantry_x_offset_mm);
    storage_->setFloat("cv_belt_w_mm", data_.conveyor_belt_width_mm);
    storage_->setFloat("cv_belt_near_mm", data_.conveyor_belt_near_x_mm);
    storage_->setFloat("cv_belt_far_mm", data_.conveyor_belt_far_x_mm);
    storage_->setFloat("cv_cnv_near_mm", data_.conveyor_frame_near_x_mm);
    storage_->setFloat("cv_cnv_far_mm", data_.conveyor_frame_far_x_mm);
    storage_->setFloat("batt_x", data_.test_battery_x_mm);
    storage_->setFloat("cv_z_pick_mm", data_.conveyor_z_pick_joint_mm);
    storage_->setFloat("cv_belt_spd", data_.pick_default_belt_speed_mm_s);
    storage_->setInt32("cv_tau_min_us", data_.tau_min_us);
    storage_->setInt32("cv_tau_max_us", data_.tau_max_us);
    storage_->setInt32("cv_vis_ms", data_.pick_vision_latency_ms);
    storage_->setInt32("cv_valve_ms", data_.pick_valve_ms);
    storage_->setInt32("pk_von_ms", data_.pick_valve_on_ms);
    storage_->setInt32("pk_voff_ms", data_.pick_valve_off_ms);
    storage_->setInt32("pk_tr_von", data_.pick_trim_von_ms);
    storage_->setInt32("pk_tr_voff", data_.pick_trim_voff_ms);
    storage_->setInt32("pk_tr_cls", data_.pick_trim_close_ms);
    storage_->setInt32("pk_tr_z", data_.pick_trim_z_ms);
    storage_->setInt32("pk_tr_grp", data_.pick_trim_grip_ms);
    storage_->setInt32("pk_tr_hld", data_.pick_trim_hold_ms);
    storage_->setInt32("pk_off_plunge", data_.pick_offset_plunge_ms);
    storage_->setInt32("pk_off_close", data_.pick_offset_close_ms);
    storage_->setInt32("pk_off_retract", data_.pick_offset_retract_ms);
    storage_->setInt32("eip_spi_khz", data_.eip_w5500_spi_khz);
    storage_->setInt32("disp_tft_en", data_.display_tft_enable);
    storage_->setInt32("eip_txwr_split", data_.eip_txwr_split);
    storage_->setInt32("eip_spi_dma", data_.eip_spi_dma);
    storage_->setInt32("log_uart_async", data_.log_uart_async);
    storage_->setInt32("eip_scope_gpio", data_.eip_scope_gpio);
    storage_->setBool("pk_bench", data_.pick_bench_enable);
    for (int i = 0; i < kPickClassSlots; ++i) {
      storage_->setInt32(kPickClassKeys[i].jaw, data_.pick_jaw_ms[i]);
    }
    storage_->setFloat("cv_jaw_deg", data_.pick_jaw_offset_deg);
    storage_->setFloat("cv_dedupe_mm", data_.pick_dedupe_mm);
    for (int i = 0; i < kPickClassSlots; ++i) {
      storage_->setFloat(kPickClassKeys[i].len, data_.pick_cell_len_mm[i]);
      storage_->setFloat(kPickClassKeys[i].height, data_.pick_cell_h_mm[i]);
    }
    storage_->setFloat("fin_open_mm", data_.pick_finger_open_mm);
    storage_->setFloat("fin_wid_mm", data_.pick_finger_thick_mm);
    storage_->setFloat("fin_len_mm", data_.pick_finger_width_mm);
    storage_->setFloat("fin_spd_mm_s", data_.pick_jaw_speed_mm_s);
    storage_->setInt32("pk_guard_ms", data_.pick_guard_ms);
    storage_->setBool("pk_dropin", data_.pick_dropin_enable);
    storage_->setBool("pk_drop_any_th", data_.pick_drop_any_theta);
    storage_->setFloat("pk_stage_x_mm", data_.pick_stage_x_mm);
    storage_->setInt32("pk_spd_mm_s", data_.pick_speed_mm_s);
    storage_->setInt32("pk_acc_mm_s2", data_.pick_accel_mm_s2);
    storage_->setFloat("pk_th_spd", data_.pick_theta_speed_deg_s);
    storage_->setFloat("pk_th_acc", data_.pick_theta_accel_deg_s2);

    storage_->setFloat("cal_spd_mm_s", data_.eip_home_cal_speed_mm_s);
    storage_->setFloat("cal_th_spd_deg", data_.eip_home_theta_speed_deg_s);
    storage_->setFloat("cal_acc_mm_s2", data_.eip_home_cal_accel_mm_s2);
    storage_->setFloat("cal_creep_mm_s", data_.eip_creep_speed_mm_s);
    storage_->setInt32("cal_timeout_ms", data_.gantry_calibration_timeout_ms);
    storage_->setInt32("trv_timeout_ms", data_.gantry_travel_measurement_timeout_ms);

    storage_->setBool("eth_static", data_.eth_use_static_ip);
    storage_->setString("eth_ip", data_.eth_static_ip);
    storage_->setString("eth_gw", data_.eth_static_gateway);
    storage_->setString("eth_netmask", data_.eth_static_netmask);
    storage_->setInt32("eth_timeout_ms", data_.eth_ip_wait_timeout_ms);

    storage_->setInt32("disp_orient", data_.display_orientation);
    storage_->setInt32("disp_bright", data_.display_brightness);
    storage_->setInt32("disp_bl_to_s", data_.display_backlight_timeout_s);

    storage_->setBool("con_auth", data_.console_tcp_auth_enable);
    storage_->setString("con_pw", data_.console_tcp_password);
    storage_->setInt32("con_tries", data_.console_tcp_auth_max_tries);
    storage_->setInt32("con_ttl_s", data_.console_tcp_auth_remember_s);
    storage_->setInt32("con_max_peers", data_.console_tcp_auth_remember_max);

    storage_->setBool("con_tcp_en", data_.console_tcp_enable);
    storage_->setBool("con_uart_en", data_.console_uart_enable);
    storage_->setInt32("con_port", data_.console_tcp_port);
    storage_->setBool("con_log_en", data_.console_tcp_log_enable);
    storage_->setInt32("con_log_lvl", data_.console_tcp_log_level);
    storage_->setInt32("ota_port", data_.ota_server_port);

    storage_->setBool("eip_en", data_.eip_scanner_enabled);
    storage_->setBool("eip_x_en", data_.eip_axis_x);
    storage_->setBool("eip_z_en", data_.eip_axis_z);
    storage_->setBool("eip_th_en", data_.eip_axis_theta);
    storage_->setString("eip_ip_x", data_.eip_target_ip_x);
    storage_->setString("eip_ip_z", data_.eip_target_ip_z);
    storage_->setString("eip_ip_th", data_.eip_target_ip_theta);
    storage_->setString("w5500_ip", data_.eip_w5500_ip);
    storage_->setString("w5500_sub", data_.eip_w5500_subnet);
    storage_->setString("w5500_gw", data_.eip_w5500_gateway);
    storage_->setInt32("w5500_spi_hz", data_.eip_w5500_spi_hz);
    storage_->setFloat("eip_x_puu_mm", data_.eip_axis_x_puu_per_mm);
    storage_->setFloat("eip_z_puu_mm", data_.eip_axis_z_puu_per_mm);
    storage_->setFloat("eip_th_puu_deg", data_.eip_axis_theta_puu_per_deg);
    storage_->setInt32("eip_x_rpi_us", data_.eip_x_rpi_us);
    storage_->setInt32("eip_th_rpi_us", data_.eip_theta_rpi_us);
    storage_->setInt32("eip_endstop_src", data_.eip_endstop_source);
    storage_->setFloat("th_abs_min_deg", data_.axis_theta_drive_abs_min_deg);
    storage_->setFloat("th_abs_max_deg", data_.axis_theta_drive_abs_max_deg);
    storage_->setString("hcs01_eng_ip", data_.hcs01_eng_ip);

    storage_->setString("dev_pw", data_.dev_mode_password);

    return storage_->commit();
}

bool AppConfig::factoryReset() {
    if (!storage_) return false;
    storage_->eraseAll();
    loadDefaults();
    return true;
}

bool AppConfig::unlockDeveloperMode(const char* password) {
    if (password == nullptr) return false;
    if (std::strcmp(password, data_.dev_mode_password) == 0) {
        dev_unlocked_ = true;
        return true;
    }
    return false;
}

bool AppConfig::changeDeveloperPassword(const char* old_pw, const char* new_pw) {
    if (!old_pw || !new_pw || std::strlen(new_pw) == 0) return false;
    if (std::strcmp(old_pw, data_.dev_mode_password) != 0) return false;

    std::strncpy(data_.dev_mode_password, new_pw, sizeof(data_.dev_mode_password) - 1);
    data_.dev_mode_password[sizeof(data_.dev_mode_password) - 1] = '\0';
    if (storage_) {
        storage_->setString("dev_pw", data_.dev_mode_password);
        storage_->commit();
    }
    return true;
}

bool AppConfig::getParamInt(const char* key, int32_t& out_val) const {
    const ParamSchema* s = findParamSchema(key);
    if (!s || (s->type != ParamType::INT && s->type != ParamType::CHOICE)) return false;

    if (std::strcmp(key, "spd_mm_s") == 0) out_val = data_.gantry_default_speed_mm_per_s;
    else if (std::strcmp(key, "acc_mm_s2") == 0) out_val = data_.gantry_default_accel_mm_per_s2;
    else if (std::strcmp(key, "dec_mm_s2") == 0) out_val = data_.gantry_default_decel_mm_per_s2;
    else if (std::strcmp(key, "th_spd_deg_s") == 0) out_val = data_.gantry_default_speed_deg_per_s;
    else if (std::strcmp(key, "th_acc_deg_s2") == 0) out_val = data_.gantry_default_accel_deg_per_s2;
    else if (std::strcmp(key, "th_dec_deg_s2") == 0) out_val = data_.gantry_default_decel_deg_per_s2;
    else if (std::strcmp(key, "ceil_spd_mm_s") == 0) out_val = data_.gantry_path_max_speed_mm_per_s;
    else if (std::strcmp(key, "ceil_acc_mm_s2") == 0) out_val = data_.gantry_path_max_accel_mm_per_s2;
    else if (std::strcmp(key, "ceil_dec_mm_s2") == 0) out_val = data_.gantry_path_max_decel_mm_per_s2;
    else if (std::strcmp(key, "eip_stop_dec") == 0) out_val = data_.gantry_eip_assumed_stop_decel_mm_s2;
    else if (std::strcmp(key, "x_enc_ppr") == 0) out_val = data_.axis_x_encoder_ppr;
    else if (std::strcmp(key, "x_home_pps") == 0) out_val = data_.axis_x_homing_speed_pps;
    else if (std::strcmp(key, "z_crit_rpm") == 0) out_val = data_.axis_z_critical_rpm;
    else if (std::strcmp(key, "z_enc_ppr") == 0) out_val = data_.axis_z_encoder_ppr;
    else if (std::strcmp(key, "z_home_pps") == 0) out_val = data_.axis_z_homing_speed_pps;
    else if (std::strcmp(key, "th_enc_ppr") == 0) out_val = data_.axis_theta_encoder_ppr;
    else if (std::strcmp(key, "th_home_pps") == 0) out_val = data_.axis_theta_homing_speed_pps;
    else if (std::strcmp(key, "grp_open_ms") == 0) out_val = data_.gantry_gripper_open_time_ms;
    else if (std::strcmp(key, "grp_close_ms") == 0) out_val = data_.gantry_gripper_close_time_ms;
    else if (std::strcmp(key, "cv_tau_min_us") == 0) out_val = data_.tau_min_us;
    else if (std::strcmp(key, "cv_tau_max_us") == 0) out_val = data_.tau_max_us;
    else if (std::strcmp(key, "cv_vis_ms") == 0) out_val = data_.pick_vision_latency_ms;
    else if (std::strcmp(key, "cv_valve_ms") == 0) out_val = data_.pick_valve_ms;
    else if (std::strcmp(key, "pk_von_ms") == 0) out_val = data_.pick_valve_on_ms;
    else if (std::strcmp(key, "pk_voff_ms") == 0) out_val = data_.pick_valve_off_ms;
    else if (std::strcmp(key, "pk_tr_von") == 0) out_val = data_.pick_trim_von_ms;
    else if (std::strcmp(key, "pk_tr_voff") == 0) out_val = data_.pick_trim_voff_ms;
    else if (std::strcmp(key, "pk_tr_cls") == 0) out_val = data_.pick_trim_close_ms;
    else if (std::strcmp(key, "pk_tr_z") == 0) out_val = data_.pick_trim_z_ms;
    else if (std::strcmp(key, "pk_tr_grp") == 0) out_val = data_.pick_trim_grip_ms;
    else if (std::strcmp(key, "pk_tr_hld") == 0) out_val = data_.pick_trim_hold_ms;
    else if (std::strcmp(key, "pk_off_plunge") == 0) out_val = data_.pick_offset_plunge_ms;
    else if (std::strcmp(key, "pk_off_close") == 0) out_val = data_.pick_offset_close_ms;
    else if (std::strcmp(key, "pk_off_retract") == 0) out_val = data_.pick_offset_retract_ms;
    else if (std::strcmp(key, "eip_spi_khz") == 0) out_val = data_.eip_w5500_spi_khz;
    else if (std::strcmp(key, "disp_tft_en") == 0) out_val = data_.display_tft_enable;
    else if (std::strcmp(key, "eip_txwr_split") == 0) out_val = data_.eip_txwr_split;
    else if (std::strcmp(key, "eip_spi_dma") == 0) out_val = data_.eip_spi_dma;
    else if (std::strcmp(key, "log_uart_async") == 0) out_val = data_.log_uart_async;
    else if (std::strcmp(key, "eip_scope_gpio") == 0) out_val = data_.eip_scope_gpio;
    else if (const int jaw = pickKeySlot(key, 0); jaw >= 0) out_val = data_.pick_jaw_ms[jaw];
    else if (std::strcmp(key, "pk_guard_ms") == 0) out_val = data_.pick_guard_ms;
    else if (std::strcmp(key, "pk_spd_mm_s") == 0) out_val = data_.pick_speed_mm_s;
    else if (std::strcmp(key, "pk_acc_mm_s2") == 0) out_val = data_.pick_accel_mm_s2;
    else if (std::strcmp(key, "cal_timeout_ms") == 0) out_val = data_.gantry_calibration_timeout_ms;
    else if (std::strcmp(key, "trv_timeout_ms") == 0) out_val = data_.gantry_travel_measurement_timeout_ms;
    else if (std::strcmp(key, "eth_timeout_ms") == 0) out_val = data_.eth_ip_wait_timeout_ms;
    else if (std::strcmp(key, "disp_orient") == 0) out_val = data_.display_orientation;
    else if (std::strcmp(key, "disp_bright") == 0) out_val = data_.display_brightness;
    else if (std::strcmp(key, "disp_bl_to_s") == 0) out_val = data_.display_backlight_timeout_s;
    else if (std::strcmp(key, "con_tries") == 0) out_val = data_.console_tcp_auth_max_tries;
    else if (std::strcmp(key, "con_ttl_s") == 0) out_val = data_.console_tcp_auth_remember_s;
    else if (std::strcmp(key, "con_max_peers") == 0) out_val = data_.console_tcp_auth_remember_max;
    else if (std::strcmp(key, "con_port") == 0) out_val = data_.console_tcp_port;
    else if (std::strcmp(key, "con_log_lvl") == 0) out_val = data_.console_tcp_log_level;
    else if (std::strcmp(key, "ota_port") == 0) out_val = data_.ota_server_port;
    else if (std::strcmp(key, "w5500_spi_hz") == 0) out_val = data_.eip_w5500_spi_hz;
    else if (std::strcmp(key, "eip_x_rpi_us") == 0) out_val = data_.eip_x_rpi_us;
    else if (std::strcmp(key, "eip_th_rpi_us") == 0) out_val = data_.eip_theta_rpi_us;
    else if (std::strcmp(key, "eip_endstop_src") == 0) out_val = data_.eip_endstop_source;
    else return false;

    return true;
}

bool AppConfig::setParamInt(const char* key, int32_t val, bool& out_reboot_required) {
    const ParamSchema* s = findParamSchema(key);
    if (!s || (s->type != ParamType::INT && s->type != ParamType::CHOICE)) return false;

    if (val < (int32_t)s->min_val || val > (int32_t)s->max_val) return false;

    out_reboot_required = s->reboot_required;

    if (std::strcmp(key, "spd_mm_s") == 0) data_.gantry_default_speed_mm_per_s = val;
    else if (std::strcmp(key, "acc_mm_s2") == 0) data_.gantry_default_accel_mm_per_s2 = val;
    else if (std::strcmp(key, "dec_mm_s2") == 0) data_.gantry_default_decel_mm_per_s2 = val;
    else if (std::strcmp(key, "th_spd_deg_s") == 0) data_.gantry_default_speed_deg_per_s = val;
    else if (std::strcmp(key, "th_acc_deg_s2") == 0) data_.gantry_default_accel_deg_per_s2 = val;
    else if (std::strcmp(key, "th_dec_deg_s2") == 0) data_.gantry_default_decel_deg_per_s2 = val;
    else if (std::strcmp(key, "ceil_spd_mm_s") == 0) data_.gantry_path_max_speed_mm_per_s = val;
    else if (std::strcmp(key, "ceil_acc_mm_s2") == 0) data_.gantry_path_max_accel_mm_per_s2 = val;
    else if (std::strcmp(key, "ceil_dec_mm_s2") == 0) data_.gantry_path_max_decel_mm_per_s2 = val;
    else if (std::strcmp(key, "eip_stop_dec") == 0) data_.gantry_eip_assumed_stop_decel_mm_s2 = val;
    else if (std::strcmp(key, "x_enc_ppr") == 0) data_.axis_x_encoder_ppr = val;
    else if (std::strcmp(key, "x_home_pps") == 0) data_.axis_x_homing_speed_pps = val;
    else if (std::strcmp(key, "z_crit_rpm") == 0) data_.axis_z_critical_rpm = val;
    else if (std::strcmp(key, "z_enc_ppr") == 0) data_.axis_z_encoder_ppr = val;
    else if (std::strcmp(key, "z_home_pps") == 0) data_.axis_z_homing_speed_pps = val;
    else if (std::strcmp(key, "th_enc_ppr") == 0) data_.axis_theta_encoder_ppr = val;
    else if (std::strcmp(key, "th_home_pps") == 0) data_.axis_theta_homing_speed_pps = val;
    else if (std::strcmp(key, "grp_open_ms") == 0) data_.gantry_gripper_open_time_ms = val;
    else if (std::strcmp(key, "grp_close_ms") == 0) data_.gantry_gripper_close_time_ms = val;
    else if (std::strcmp(key, "cv_tau_min_us") == 0) data_.tau_min_us = val;
    else if (std::strcmp(key, "cv_tau_max_us") == 0) data_.tau_max_us = val;
    else if (std::strcmp(key, "cv_vis_ms") == 0) data_.pick_vision_latency_ms = val;
    else if (std::strcmp(key, "cv_valve_ms") == 0) data_.pick_valve_ms = val;
    else if (std::strcmp(key, "pk_von_ms") == 0) data_.pick_valve_on_ms = val;
    else if (std::strcmp(key, "pk_voff_ms") == 0) data_.pick_valve_off_ms = val;
    else if (std::strcmp(key, "pk_tr_von") == 0) data_.pick_trim_von_ms = val;
    else if (std::strcmp(key, "pk_tr_voff") == 0) data_.pick_trim_voff_ms = val;
    else if (std::strcmp(key, "pk_tr_cls") == 0) data_.pick_trim_close_ms = val;
    else if (std::strcmp(key, "pk_tr_z") == 0) data_.pick_trim_z_ms = val;
    else if (std::strcmp(key, "pk_tr_grp") == 0) data_.pick_trim_grip_ms = val;
    else if (std::strcmp(key, "pk_tr_hld") == 0) data_.pick_trim_hold_ms = val;
    else if (std::strcmp(key, "pk_off_plunge") == 0) data_.pick_offset_plunge_ms = val;
    else if (std::strcmp(key, "pk_off_close") == 0) data_.pick_offset_close_ms = val;
    else if (std::strcmp(key, "pk_off_retract") == 0) data_.pick_offset_retract_ms = val;
    else if (std::strcmp(key, "eip_spi_khz") == 0) data_.eip_w5500_spi_khz = val;
    else if (std::strcmp(key, "disp_tft_en") == 0) data_.display_tft_enable = val;
    else if (std::strcmp(key, "eip_txwr_split") == 0) data_.eip_txwr_split = val;
    else if (std::strcmp(key, "eip_spi_dma") == 0) data_.eip_spi_dma = val;
    else if (std::strcmp(key, "log_uart_async") == 0) data_.log_uart_async = val;
    else if (std::strcmp(key, "eip_scope_gpio") == 0) data_.eip_scope_gpio = val;
    else if (const int jaw = pickKeySlot(key, 0); jaw >= 0) data_.pick_jaw_ms[jaw] = val;
    else if (std::strcmp(key, "pk_guard_ms") == 0) data_.pick_guard_ms = val;
    else if (std::strcmp(key, "pk_spd_mm_s") == 0) data_.pick_speed_mm_s = val;
    else if (std::strcmp(key, "pk_acc_mm_s2") == 0) data_.pick_accel_mm_s2 = val;
    else if (std::strcmp(key, "cal_timeout_ms") == 0) data_.gantry_calibration_timeout_ms = val;
    else if (std::strcmp(key, "trv_timeout_ms") == 0) data_.gantry_travel_measurement_timeout_ms = val;
    else if (std::strcmp(key, "eth_timeout_ms") == 0) data_.eth_ip_wait_timeout_ms = val;
    else if (std::strcmp(key, "disp_orient") == 0) data_.display_orientation = val;
    else if (std::strcmp(key, "disp_bright") == 0) data_.display_brightness = val;
    else if (std::strcmp(key, "disp_bl_to_s") == 0) data_.display_backlight_timeout_s = val;
    else if (std::strcmp(key, "con_tries") == 0) data_.console_tcp_auth_max_tries = val;
    else if (std::strcmp(key, "con_ttl_s") == 0) data_.console_tcp_auth_remember_s = val;
    else if (std::strcmp(key, "con_max_peers") == 0) data_.console_tcp_auth_remember_max = val;
    else if (std::strcmp(key, "con_port") == 0) data_.console_tcp_port = val;
    else if (std::strcmp(key, "con_log_lvl") == 0) data_.console_tcp_log_level = val;
    else if (std::strcmp(key, "ota_port") == 0) data_.ota_server_port = val;
    else if (std::strcmp(key, "w5500_spi_hz") == 0) data_.eip_w5500_spi_hz = val;
    else if (std::strcmp(key, "eip_x_rpi_us") == 0) data_.eip_x_rpi_us = val;
    else if (std::strcmp(key, "eip_th_rpi_us") == 0) data_.eip_theta_rpi_us = val;
    else if (std::strcmp(key, "eip_endstop_src") == 0) data_.eip_endstop_source = val;
    else return false;

    if (storage_) {
        storage_->setInt32(key, val);
        storage_->commit();
    }
    return true;
}

bool AppConfig::getParamFloat(const char* key, float& out_val) const {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::FLOAT) return false;

    if (std::strcmp(key, "x_lead_mm") == 0) out_val = data_.axis_x_lead_mm_per_rev;
    else if (std::strcmp(key, "x_lim_min_mm") == 0) out_val = data_.axis_x_hard_limit_min_mm;
    else if (std::strcmp(key, "x_lim_max_mm") == 0) out_val = data_.axis_x_hard_limit_max_mm;
    else if (std::strcmp(key, "x_max_spd_mm_s") == 0) out_val = data_.axis_x_max_speed_mm_per_s;
    else if (std::strcmp(key, "x_acc_mm_s2") == 0) out_val = data_.axis_x_accel_mm_per_s2;
    else if (std::strcmp(key, "x_dec_mm_s2") == 0) out_val = data_.axis_x_decel_mm_per_s2;
    else if (std::strcmp(key, "x_tol_mm") == 0) out_val = data_.axis_x_position_tolerance_mm;
    else if (std::strcmp(key, "x_gear_ratio") == 0) out_val = data_.axis_x_motor_reducer_ratio;
    else if (std::strcmp(key, "z_lead_mm") == 0) out_val = data_.axis_z_lead_mm_per_rev;
    else if (std::strcmp(key, "z_lim_min_mm") == 0) out_val = data_.axis_z_hard_limit_min_mm;
    else if (std::strcmp(key, "z_lim_max_mm") == 0) out_val = data_.axis_z_hard_limit_max_mm;
    else if (std::strcmp(key, "z_max_spd_mm_s") == 0) out_val = data_.axis_z_max_speed_mm_per_s;
    else if (std::strcmp(key, "z_acc_mm_s2") == 0) out_val = data_.axis_z_accel_mm_per_s2;
    else if (std::strcmp(key, "z_dec_mm_s2") == 0) out_val = data_.axis_z_decel_mm_per_s2;
    else if (std::strcmp(key, "z_tol_mm") == 0) out_val = data_.axis_z_position_tolerance_mm;
    else if (std::strcmp(key, "z_gear_ratio") == 0) out_val = data_.axis_z_motor_reducer_ratio;
    else if (std::strcmp(key, "z_datum_mm") == 0) out_val = data_.gantry_z_datum_offset_above_bed_mm;
    else if (std::strcmp(key, "th_gear_ratio") == 0) out_val = data_.axis_theta_output_gear_ratio;
    else if (std::strcmp(key, "th_lim_min_deg") == 0) out_val = data_.axis_theta_hard_limit_min_deg;
    else if (std::strcmp(key, "th_lim_max_deg") == 0) out_val = data_.axis_theta_hard_limit_max_deg;
    else if (std::strcmp(key, "th_spd_max") == 0) out_val = data_.axis_theta_max_speed_deg_per_s;
    else if (std::strcmp(key, "th_acc_deg") == 0) out_val = data_.axis_theta_accel_deg_per_s2;
    else if (std::strcmp(key, "th_dec_deg") == 0) out_val = data_.axis_theta_decel_deg_per_s2;
    else if (std::strcmp(key, "th_tol_deg") == 0) out_val = data_.axis_theta_position_tolerance_deg;
    else if (std::strcmp(key, "th_reducer") == 0) out_val = data_.axis_theta_motor_reducer_ratio;
    else if (std::strcmp(key, "geom_z_y_mm") == 0) out_val = data_.gantry_z_axis_y_offset_mm;
    else if (std::strcmp(key, "geom_th_x_mm") == 0) out_val = data_.gantry_theta_x_offset_mm;
    else if (std::strcmp(key, "geom_grp_x_mm") == 0) out_val = data_.gantry_gripper_x_offset_mm;
    else if (std::strcmp(key, "geom_grp_z_mm") == 0) out_val = data_.gantry_gripper_z_offset_mm;
    else if (std::strcmp(key, "geom_safe_z_mm") == 0) out_val = data_.gantry_safe_z_height_mm;
    else if (std::strcmp(key, "geom_cal_x_mm") == 0) out_val = data_.gantry_cal_x_park_mm;
    else if (std::strcmp(key, "geom_cal_z_mm") == 0) out_val = data_.gantry_cal_z_park_mm;
    else if (std::strcmp(key, "geom_cal_th_deg") == 0) out_val = data_.gantry_cal_theta_park_deg;
    else if (std::strcmp(key, "col_x_min_mm") == 0) out_val = data_.gantry_conveyor_collision_x_min_mm;
    else if (std::strcmp(key, "col_z_min_mm") == 0) out_val = data_.gantry_conveyor_collision_z_min_mm;
    else if (std::strcmp(key, "cv_cam_y_mm") == 0) out_val = data_.conveyor_y_cam_mm;
    else if (std::strcmp(key, "cv_pick_y_mm") == 0) out_val = data_.conveyor_y_pick_mm;
    else if (std::strcmp(key, "cv_x_off_mm") == 0) out_val = data_.conveyor_x_across_to_gantry_x_offset_mm;
    else if (std::strcmp(key, "cv_belt_w_mm") == 0) out_val = data_.conveyor_belt_width_mm;
    else if (std::strcmp(key, "cv_belt_near_mm") == 0) out_val = data_.conveyor_belt_near_x_mm;
    else if (std::strcmp(key, "cv_belt_far_mm") == 0) out_val = data_.conveyor_belt_far_x_mm;
    else if (std::strcmp(key, "cv_cnv_near_mm") == 0) out_val = data_.conveyor_frame_near_x_mm;
    else if (std::strcmp(key, "cv_cnv_far_mm") == 0) out_val = data_.conveyor_frame_far_x_mm;
    else if (std::strcmp(key, "batt_x") == 0) out_val = data_.test_battery_x_mm;
    else if (std::strcmp(key, "cv_z_pick_mm") == 0) out_val = data_.conveyor_z_pick_joint_mm;
    else if (std::strcmp(key, "cv_belt_spd") == 0) out_val = data_.pick_default_belt_speed_mm_s;
    else if (std::strcmp(key, "cv_jaw_deg") == 0) out_val = data_.pick_jaw_offset_deg;
    else if (std::strcmp(key, "cv_dedupe_mm") == 0) out_val = data_.pick_dedupe_mm;
    else if (const int len = pickKeySlot(key, 1); len >= 0) out_val = data_.pick_cell_len_mm[len];
    else if (const int height = pickKeySlot(key, 2); height >= 0) out_val = data_.pick_cell_h_mm[height];
    else if (std::strcmp(key, "fin_open_mm") == 0) out_val = data_.pick_finger_open_mm;
    else if (std::strcmp(key, "fin_wid_mm") == 0) out_val = data_.pick_finger_thick_mm;
    else if (std::strcmp(key, "fin_len_mm") == 0) out_val = data_.pick_finger_width_mm;
    else if (std::strcmp(key, "fin_spd_mm_s") == 0) out_val = data_.pick_jaw_speed_mm_s;
    else if (std::strcmp(key, "pk_stage_x_mm") == 0) out_val = data_.pick_stage_x_mm;
    else if (std::strcmp(key, "pk_th_spd") == 0) out_val = data_.pick_theta_speed_deg_s;
    else if (std::strcmp(key, "pk_th_acc") == 0) out_val = data_.pick_theta_accel_deg_s2;
    else if (std::strcmp(key, "cal_spd_mm_s") == 0) out_val = data_.eip_home_cal_speed_mm_s;
    else if (std::strcmp(key, "cal_th_spd_deg") == 0) out_val = data_.eip_home_theta_speed_deg_s;
    else if (std::strcmp(key, "cal_acc_mm_s2") == 0) out_val = data_.eip_home_cal_accel_mm_s2;
    else if (std::strcmp(key, "cal_creep_mm_s") == 0) out_val = data_.eip_creep_speed_mm_s;
    else if (std::strcmp(key, "eip_x_puu_mm") == 0) out_val = data_.eip_axis_x_puu_per_mm;
    else if (std::strcmp(key, "eip_z_puu_mm") == 0) out_val = data_.eip_axis_z_puu_per_mm;
    else if (std::strcmp(key, "eip_th_puu_deg") == 0) out_val = data_.eip_axis_theta_puu_per_deg;
    else if (std::strcmp(key, "th_abs_min_deg") == 0) out_val = data_.axis_theta_drive_abs_min_deg;
    else if (std::strcmp(key, "th_abs_max_deg") == 0) out_val = data_.axis_theta_drive_abs_max_deg;
    else return false;

    return true;
}

bool AppConfig::setParamFloat(const char* key, float val, bool& out_reboot_required) {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::FLOAT) return false;

    if (val < s->min_val || val > s->max_val) return false;

    out_reboot_required = s->reboot_required;

    if (std::strcmp(key, "x_lead_mm") == 0) data_.axis_x_lead_mm_per_rev = val;
    else if (std::strcmp(key, "x_lim_min_mm") == 0) data_.axis_x_hard_limit_min_mm = val;
    else if (std::strcmp(key, "x_lim_max_mm") == 0) data_.axis_x_hard_limit_max_mm = val;
    else if (std::strcmp(key, "x_max_spd_mm_s") == 0) data_.axis_x_max_speed_mm_per_s = val;
    else if (std::strcmp(key, "x_acc_mm_s2") == 0) data_.axis_x_accel_mm_per_s2 = val;
    else if (std::strcmp(key, "x_dec_mm_s2") == 0) data_.axis_x_decel_mm_per_s2 = val;
    else if (std::strcmp(key, "x_tol_mm") == 0) data_.axis_x_position_tolerance_mm = val;
    else if (std::strcmp(key, "x_gear_ratio") == 0) data_.axis_x_motor_reducer_ratio = val;
    else if (std::strcmp(key, "z_lead_mm") == 0) data_.axis_z_lead_mm_per_rev = val;
    else if (std::strcmp(key, "z_lim_min_mm") == 0) data_.axis_z_hard_limit_min_mm = val;
    else if (std::strcmp(key, "z_lim_max_mm") == 0) data_.axis_z_hard_limit_max_mm = val;
    else if (std::strcmp(key, "z_max_spd_mm_s") == 0) data_.axis_z_max_speed_mm_per_s = val;
    else if (std::strcmp(key, "z_acc_mm_s2") == 0) data_.axis_z_accel_mm_per_s2 = val;
    else if (std::strcmp(key, "z_dec_mm_s2") == 0) data_.axis_z_decel_mm_per_s2 = val;
    else if (std::strcmp(key, "z_tol_mm") == 0) data_.axis_z_position_tolerance_mm = val;
    else if (std::strcmp(key, "z_gear_ratio") == 0) data_.axis_z_motor_reducer_ratio = val;
    else if (std::strcmp(key, "z_datum_mm") == 0) data_.gantry_z_datum_offset_above_bed_mm = val;
    else if (std::strcmp(key, "th_gear_ratio") == 0) data_.axis_theta_output_gear_ratio = val;
    else if (std::strcmp(key, "th_lim_min_deg") == 0) data_.axis_theta_hard_limit_min_deg = val;
    else if (std::strcmp(key, "th_lim_max_deg") == 0) data_.axis_theta_hard_limit_max_deg = val;
    else if (std::strcmp(key, "th_spd_max") == 0) data_.axis_theta_max_speed_deg_per_s = val;
    else if (std::strcmp(key, "th_acc_deg") == 0) data_.axis_theta_accel_deg_per_s2 = val;
    else if (std::strcmp(key, "th_dec_deg") == 0) data_.axis_theta_decel_deg_per_s2 = val;
    else if (std::strcmp(key, "th_tol_deg") == 0) data_.axis_theta_position_tolerance_deg = val;
    else if (std::strcmp(key, "th_reducer") == 0) data_.axis_theta_motor_reducer_ratio = val;
    else if (std::strcmp(key, "geom_z_y_mm") == 0) data_.gantry_z_axis_y_offset_mm = val;
    else if (std::strcmp(key, "geom_th_x_mm") == 0) data_.gantry_theta_x_offset_mm = val;
    else if (std::strcmp(key, "geom_grp_x_mm") == 0) data_.gantry_gripper_x_offset_mm = val;
    else if (std::strcmp(key, "geom_grp_z_mm") == 0) data_.gantry_gripper_z_offset_mm = val;
    else if (std::strcmp(key, "geom_safe_z_mm") == 0) data_.gantry_safe_z_height_mm = val;
    else if (std::strcmp(key, "geom_cal_x_mm") == 0) data_.gantry_cal_x_park_mm = val;
    else if (std::strcmp(key, "geom_cal_z_mm") == 0) data_.gantry_cal_z_park_mm = val;
    else if (std::strcmp(key, "geom_cal_th_deg") == 0) data_.gantry_cal_theta_park_deg = val;
    else if (std::strcmp(key, "col_x_min_mm") == 0) data_.gantry_conveyor_collision_x_min_mm = val;
    else if (std::strcmp(key, "col_z_min_mm") == 0) data_.gantry_conveyor_collision_z_min_mm = val;
    else if (std::strcmp(key, "cv_cam_y_mm") == 0) data_.conveyor_y_cam_mm = val;
    else if (std::strcmp(key, "cv_pick_y_mm") == 0) data_.conveyor_y_pick_mm = val;
    else if (std::strcmp(key, "cv_x_off_mm") == 0) data_.conveyor_x_across_to_gantry_x_offset_mm = val;
    else if (std::strcmp(key, "cv_belt_w_mm") == 0) data_.conveyor_belt_width_mm = val;
    else if (std::strcmp(key, "cv_belt_near_mm") == 0) data_.conveyor_belt_near_x_mm = val;
    else if (std::strcmp(key, "cv_belt_far_mm") == 0) data_.conveyor_belt_far_x_mm = val;
    else if (std::strcmp(key, "cv_cnv_near_mm") == 0) data_.conveyor_frame_near_x_mm = val;
    else if (std::strcmp(key, "cv_cnv_far_mm") == 0) data_.conveyor_frame_far_x_mm = val;
    else if (std::strcmp(key, "batt_x") == 0) data_.test_battery_x_mm = val;
    else if (std::strcmp(key, "cv_z_pick_mm") == 0) data_.conveyor_z_pick_joint_mm = val;
    else if (std::strcmp(key, "cv_belt_spd") == 0) data_.pick_default_belt_speed_mm_s = val;
    else if (std::strcmp(key, "cv_jaw_deg") == 0) data_.pick_jaw_offset_deg = val;
    else if (std::strcmp(key, "cv_dedupe_mm") == 0) data_.pick_dedupe_mm = val;
    else if (const int len = pickKeySlot(key, 1); len >= 0) data_.pick_cell_len_mm[len] = val;
    else if (const int height = pickKeySlot(key, 2); height >= 0) data_.pick_cell_h_mm[height] = val;
    else if (std::strcmp(key, "fin_open_mm") == 0) data_.pick_finger_open_mm = val;
    else if (std::strcmp(key, "fin_wid_mm") == 0) data_.pick_finger_thick_mm = val;
    else if (std::strcmp(key, "fin_len_mm") == 0) data_.pick_finger_width_mm = val;
    else if (std::strcmp(key, "fin_spd_mm_s") == 0) data_.pick_jaw_speed_mm_s = val;
    else if (std::strcmp(key, "pk_stage_x_mm") == 0) data_.pick_stage_x_mm = val;
    else if (std::strcmp(key, "pk_th_spd") == 0) data_.pick_theta_speed_deg_s = val;
    else if (std::strcmp(key, "pk_th_acc") == 0) data_.pick_theta_accel_deg_s2 = val;
    else if (std::strcmp(key, "cal_spd_mm_s") == 0) data_.eip_home_cal_speed_mm_s = val;
    else if (std::strcmp(key, "cal_th_spd_deg") == 0) data_.eip_home_theta_speed_deg_s = val;
    else if (std::strcmp(key, "cal_acc_mm_s2") == 0) data_.eip_home_cal_accel_mm_s2 = val;
    else if (std::strcmp(key, "cal_creep_mm_s") == 0) data_.eip_creep_speed_mm_s = val;
    else if (std::strcmp(key, "eip_x_puu_mm") == 0) data_.eip_axis_x_puu_per_mm = val;
    else if (std::strcmp(key, "eip_z_puu_mm") == 0) data_.eip_axis_z_puu_per_mm = val;
    else if (std::strcmp(key, "eip_th_puu_deg") == 0) data_.eip_axis_theta_puu_per_deg = val;
    else if (std::strcmp(key, "th_abs_min_deg") == 0) data_.axis_theta_drive_abs_min_deg = val;
    else if (std::strcmp(key, "th_abs_max_deg") == 0) data_.axis_theta_drive_abs_max_deg = val;
    else return false;

    if (storage_) {
        storage_->setFloat(key, val);
        storage_->commit();
    }
    return true;
}

bool AppConfig::getParamBool(const char* key, bool& out_val) const {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::BOOL) return false;

    if (std::strcmp(key, "z_has_brake") == 0) out_val = data_.axis_z_has_motor_brake;
    else if (std::strcmp(key, "th_sequential") == 0) out_val = data_.gantry_theta_sequential;
    else if (std::strcmp(key, "eth_static") == 0) out_val = data_.eth_use_static_ip;
    else if (std::strcmp(key, "con_auth") == 0) out_val = data_.console_tcp_auth_enable;
    else if (std::strcmp(key, "con_tcp_en") == 0) out_val = data_.console_tcp_enable;
    else if (std::strcmp(key, "con_uart_en") == 0) out_val = data_.console_uart_enable;
    else if (std::strcmp(key, "con_log_en") == 0) out_val = data_.console_tcp_log_enable;
    else if (std::strcmp(key, "eip_en") == 0) out_val = data_.eip_scanner_enabled;
    else if (std::strcmp(key, "eip_x_en") == 0) out_val = data_.eip_axis_x;
    else if (std::strcmp(key, "eip_z_en") == 0) out_val = data_.eip_axis_z;
    else if (std::strcmp(key, "eip_th_en") == 0) out_val = data_.eip_axis_theta;
    else if (std::strcmp(key, "pk_dropin") == 0) out_val = data_.pick_dropin_enable;
    else if (std::strcmp(key, "pk_bench") == 0) out_val = data_.pick_bench_enable;
    else if (std::strcmp(key, "pk_drop_any_th") == 0) out_val = data_.pick_drop_any_theta;
    else return false;

    return true;
}

bool AppConfig::setParamBool(const char* key, bool val, bool& out_reboot_required) {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::BOOL) return false;

    out_reboot_required = s->reboot_required;

    if (std::strcmp(key, "z_has_brake") == 0) data_.axis_z_has_motor_brake = val;
    else if (std::strcmp(key, "th_sequential") == 0) data_.gantry_theta_sequential = val;
    else if (std::strcmp(key, "eth_static") == 0) data_.eth_use_static_ip = val;
    else if (std::strcmp(key, "con_auth") == 0) data_.console_tcp_auth_enable = val;
    else if (std::strcmp(key, "con_tcp_en") == 0) data_.console_tcp_enable = val;
    else if (std::strcmp(key, "con_uart_en") == 0) data_.console_uart_enable = val;
    else if (std::strcmp(key, "con_log_en") == 0) data_.console_tcp_log_enable = val;
    else if (std::strcmp(key, "eip_en") == 0) data_.eip_scanner_enabled = val;
    else if (std::strcmp(key, "eip_x_en") == 0) data_.eip_axis_x = val;
    else if (std::strcmp(key, "eip_z_en") == 0) data_.eip_axis_z = val;
    else if (std::strcmp(key, "eip_th_en") == 0) data_.eip_axis_theta = val;
    else if (std::strcmp(key, "pk_dropin") == 0) data_.pick_dropin_enable = val;
    else if (std::strcmp(key, "pk_bench") == 0) data_.pick_bench_enable = val;
    else if (std::strcmp(key, "pk_drop_any_th") == 0) data_.pick_drop_any_theta = val;
    else return false;

    if (storage_) {
        storage_->setBool(key, val);
        storage_->commit();
    }
    return true;
}

bool AppConfig::getParamString(const char* key, char* out_buf, size_t max_len) const {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::STRING || !out_buf || max_len == 0) return false;

    const char* src = nullptr;
    if (std::strcmp(key, "eth_ip") == 0) src = data_.eth_static_ip;
    else if (std::strcmp(key, "eth_gw") == 0) src = data_.eth_static_gateway;
    else if (std::strcmp(key, "eth_netmask") == 0) src = data_.eth_static_netmask;
    else if (std::strcmp(key, "con_pw") == 0) src = data_.console_tcp_password;
    else if (std::strcmp(key, "eip_ip_x") == 0) src = data_.eip_target_ip_x;
    else if (std::strcmp(key, "eip_ip_z") == 0) src = data_.eip_target_ip_z;
    else if (std::strcmp(key, "eip_ip_th") == 0) src = data_.eip_target_ip_theta;
    else if (std::strcmp(key, "w5500_ip") == 0) src = data_.eip_w5500_ip;
    else if (std::strcmp(key, "w5500_sub") == 0) src = data_.eip_w5500_subnet;
    else if (std::strcmp(key, "w5500_gw") == 0) src = data_.eip_w5500_gateway;
    else if (std::strcmp(key, "hcs01_eng_ip") == 0) src = data_.hcs01_eng_ip;
    else return false;

    std::strncpy(out_buf, src, max_len - 1);
    out_buf[max_len - 1] = '\0';
    return true;
}

bool AppConfig::setParamString(const char* key, const char* val, bool& out_reboot_required) {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::STRING || !val) return false;

    out_reboot_required = s->reboot_required;

    char* dst = nullptr;
    size_t cap = 0;
    if (std::strcmp(key, "eth_ip") == 0) { dst = data_.eth_static_ip; cap = sizeof(data_.eth_static_ip); }
    else if (std::strcmp(key, "eth_gw") == 0) { dst = data_.eth_static_gateway; cap = sizeof(data_.eth_static_gateway); }
    else if (std::strcmp(key, "eth_netmask") == 0) { dst = data_.eth_static_netmask; cap = sizeof(data_.eth_static_netmask); }
    else if (std::strcmp(key, "con_pw") == 0) { dst = data_.console_tcp_password; cap = sizeof(data_.console_tcp_password); }
    else if (std::strcmp(key, "eip_ip_x") == 0) { dst = data_.eip_target_ip_x; cap = sizeof(data_.eip_target_ip_x); }
    else if (std::strcmp(key, "eip_ip_z") == 0) { dst = data_.eip_target_ip_z; cap = sizeof(data_.eip_target_ip_z); }
    else if (std::strcmp(key, "eip_ip_th") == 0) { dst = data_.eip_target_ip_theta; cap = sizeof(data_.eip_target_ip_theta); }
    else if (std::strcmp(key, "w5500_ip") == 0) { dst = data_.eip_w5500_ip; cap = sizeof(data_.eip_w5500_ip); }
    else if (std::strcmp(key, "w5500_sub") == 0) { dst = data_.eip_w5500_subnet; cap = sizeof(data_.eip_w5500_subnet); }
    else if (std::strcmp(key, "w5500_gw") == 0) { dst = data_.eip_w5500_gateway; cap = sizeof(data_.eip_w5500_gateway); }
    else if (std::strcmp(key, "hcs01_eng_ip") == 0) { dst = data_.hcs01_eng_ip; cap = sizeof(data_.hcs01_eng_ip); }
    else return false;

    std::strncpy(dst, val, cap - 1);
    dst[cap - 1] = '\0';

    if (storage_) {
        storage_->setString(key, dst);
        storage_->commit();
    }
    return true;
}

namespace {

bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    while (*a != '\0' && *b != '\0') {
        if (std::tolower(static_cast<unsigned char>(*a)) !=
            std::tolower(static_cast<unsigned char>(*b))) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

const char* skipSpace(const char* s) {
    while (s != nullptr && *s != '\0' &&
           std::isspace(static_cast<unsigned char>(*s))) {
        ++s;
    }
    return s;
}

bool parseBoolText(const char* text, bool& out) {
    if (equalsIgnoreCase(text, "1") || equalsIgnoreCase(text, "on") ||
        equalsIgnoreCase(text, "true") || equalsIgnoreCase(text, "yes")) {
        out = true;
        return true;
    }
    if (equalsIgnoreCase(text, "0") || equalsIgnoreCase(text, "off") ||
        equalsIgnoreCase(text, "false") || equalsIgnoreCase(text, "no")) {
        out = false;
        return true;
    }
    return false;
}

bool consumedNumber(const char* end) {
    end = skipSpace(end);
    return end != nullptr && *end == '\0';
}

} // namespace

bool AppConfig::isParamVisible(const char* key) const {
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }
    if (schema->access == AccessLevel::DEVELOPER && !dev_unlocked_) {
        return false;
    }
    return true;
}

bool AppConfig::formatParamValue(const char* key, char* out_buf, size_t max_len) const {
    if (out_buf == nullptr || max_len == 0 || !isParamVisible(key)) {
        return false;
    }
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }

    if (schema->type == ParamType::INT) {
        int32_t value = 0;
        if (!getParamInt(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%ld", static_cast<long>(value));
        return true;
    }
    if (schema->type == ParamType::FLOAT) {
        float value = 0.0f;
        if (!getParamFloat(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%.2f", static_cast<double>(value));
        return true;
    }
    if (schema->type == ParamType::BOOL) {
        bool value = false;
        if (!getParamBool(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%s", value ? "ON" : "OFF");
        return true;
    }
    if (schema->type == ParamType::STRING) {
        return getParamString(key, out_buf, max_len);
    }
    if (schema->type == ParamType::CHOICE) {
        int32_t value = 0;
        if (!getParamInt(key, value)) {
            return false;
        }
        if (value >= 0 && value < static_cast<int32_t>(schema->choices.size())) {
            std::snprintf(out_buf, max_len, "%s", schema->choices[static_cast<size_t>(value)]);
        } else {
            std::snprintf(out_buf, max_len, "%ld", static_cast<long>(value));
        }
        return true;
    }
    return false;
}

bool AppConfig::applyParamText(const char* key, const char* text, bool& out_reboot_required) {
    if (!isParamVisible(key) || text == nullptr) {
        return false;
    }
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }
    const char* body = skipSpace(text);
    if (body == nullptr || *body == '\0') {
        return false;
    }

    if (schema->type == ParamType::BOOL) {
        bool value = false;
        if (!parseBoolText(body, value)) {
            return false;
        }
        return setParamBool(key, value, out_reboot_required);
    }
    if (schema->type == ParamType::INT) {
        char* end = nullptr;
        long parsed = std::strtol(body, &end, 10);
        if (!consumedNumber(end)) {
            return false;
        }
        return setParamInt(key, static_cast<int32_t>(parsed), out_reboot_required);
    }
    if (schema->type == ParamType::CHOICE) {
        char* end = nullptr;
        long parsed = std::strtol(body, &end, 10);
        if (consumedNumber(end) && end != body) {
            return setParamInt(key, static_cast<int32_t>(parsed), out_reboot_required);
        }
        for (size_t i = 0; i < schema->choices.size(); ++i) {
            if (equalsIgnoreCase(body, schema->choices[i])) {
                return setParamInt(key, static_cast<int32_t>(i), out_reboot_required);
            }
        }
        return false;
    }
    if (schema->type == ParamType::FLOAT) {
        char* end = nullptr;
        float parsed = std::strtof(body, &end);
        if (!consumedNumber(end)) {
            return false;
        }
        return setParamFloat(key, parsed, out_reboot_required);
    }
    if (schema->type == ParamType::STRING) {
        return setParamString(key, body, out_reboot_required);
    }
    return false;
}

} // namespace Config
