#ifndef APP_CONFIG_DATA_H
#define APP_CONFIG_DATA_H

#include <cstdint>

namespace Config {

struct AppConfigData {
    // 1. Motion Profile
    int32_t gantry_default_speed_mm_per_s = 50;
    int32_t gantry_default_accel_mm_per_s2 = 3000;
    int32_t gantry_default_decel_mm_per_s2 = 3000;
    int32_t gantry_default_speed_deg_per_s = 30;
    int32_t gantry_default_accel_deg_per_s2 = 180;
    int32_t gantry_default_decel_deg_per_s2 = 180;
    int32_t gantry_path_max_speed_mm_per_s = 500;
    int32_t gantry_path_max_accel_mm_per_s2 = 3000;
    int32_t gantry_path_max_decel_mm_per_s2 = 3000;
    int32_t gantry_eip_assumed_stop_decel_mm_s2 = 300;

    // 2. X Axis
    float axis_x_lead_mm_per_rev = 200.0f;
    float axis_x_hard_limit_min_mm = 0.0f;
    float axis_x_hard_limit_max_mm = 550.0f;
    float axis_x_max_speed_mm_per_s = 500.0f;
    float axis_x_accel_mm_per_s2 = 3000.0f;
    float axis_x_decel_mm_per_s2 = 3000.0f;
    float axis_x_position_tolerance_mm = 0.08f;
    int32_t axis_x_encoder_ppr = 2097152;
    float axis_x_motor_reducer_ratio = 5.0f;
    int32_t axis_x_homing_speed_pps = 8000;

    // 3. Z Axis
    float axis_z_lead_mm_per_rev = 20.0f;
    int32_t axis_z_critical_rpm = 3000;
    float axis_z_hard_limit_min_mm = 0.0f;
    float axis_z_hard_limit_max_mm = 150.0f;
    float axis_z_max_speed_mm_per_s = 500.0f;
    float axis_z_accel_mm_per_s2 = 5000.0f;
    float axis_z_decel_mm_per_s2 = 5000.0f;
    float axis_z_position_tolerance_mm = 0.03f;
    int32_t axis_z_encoder_ppr = 2097152;
    float axis_z_motor_reducer_ratio = 1.0f;
    bool axis_z_has_motor_brake = false;
    int32_t axis_z_homing_speed_pps = 8000;
    float gantry_z_datum_offset_above_bed_mm = 0.0f;

    // 4. Theta Axis
    float axis_theta_output_gear_ratio = 1.0f;
    float axis_theta_hard_limit_min_deg = -180.0f;
    float axis_theta_hard_limit_max_deg = 180.0f;
    float axis_theta_max_speed_deg_per_s = 360.0f;
    float axis_theta_accel_deg_per_s2 = 1800.0f;
    float axis_theta_decel_deg_per_s2 = 1800.0f;
    float axis_theta_position_tolerance_deg = 0.01f;
    int32_t axis_theta_encoder_ppr = 36000;
    float axis_theta_motor_reducer_ratio = 1.0f;
    int32_t axis_theta_homing_speed_pps = 6000;
    bool gantry_theta_sequential = false;

    // 5. Geometry & Gripper
    float gantry_z_axis_y_offset_mm = 80.0f;
    float gantry_theta_x_offset_mm = -55.0f;
    float gantry_gripper_x_offset_mm = 385.0f;
    float gantry_gripper_z_offset_mm = 80.0f;
    float gantry_safe_z_height_mm = 35.7f;
    float gantry_cal_x_park_mm = 35.0f;
    float gantry_conveyor_collision_x_min_mm = 95.0f;
    float gantry_conveyor_collision_z_min_mm = 115.0f;
    int32_t gantry_gripper_open_time_ms = 190;
    int32_t gantry_gripper_close_time_ms = 150;

    // 6. Conveyor Intercept
    float conveyor_y_cam_mm = 336.55f;
    float conveyor_y_pick_mm = 1016.0f;
    float conveyor_x_across_to_gantry_x_offset_mm = 0.0f;
    float conveyor_z_pick_joint_mm = 140.0f;
    float pick_default_belt_speed_mm_s = 1524.0f;
    int32_t tau_min_us = 100000;
    int32_t tau_max_us = 5000000;

    // 7. Homing & Calibration
    float eip_home_cal_speed_mm_s = 50.0f;
    float eip_home_theta_speed_deg_s = 15.0f;
    float eip_home_cal_accel_mm_s2 = 2000.0f;
    float eip_creep_speed_mm_s = 1.0f;
    int32_t gantry_calibration_timeout_ms = 30000;
    int32_t gantry_travel_measurement_timeout_ms = 90000;

    // 8. Network (LAN8720)
    bool eth_use_static_ip = true;
    char eth_static_ip[32] = "192.168.1.100";
    char eth_static_gateway[32] = "192.168.1.5";
    char eth_static_netmask[32] = "255.255.255.0";
    int32_t eth_ip_wait_timeout_ms = 3000;

    // 9. Display
    int32_t display_orientation = 0; // 0 = Landscape, 1 = Portrait
    int32_t display_brightness = 100; // 0..100 % (PWM brightness)
    int32_t display_backlight_timeout_s = 30;

    // 10. Console Security (Developer)
    bool console_tcp_auth_enable = true;
    char console_tcp_password[32] = "LTU_1932";
    int32_t console_tcp_auth_max_tries = 3;
    int32_t console_tcp_auth_remember_s = 600;
    int32_t console_tcp_auth_remember_max = 4;

    // 11. Console Options (Developer)
    bool console_tcp_enable = true;
    bool console_uart_enable = false;
    int32_t console_tcp_port = 2323;
    bool console_tcp_log_enable = true;
    int32_t console_tcp_log_level = 2; // 0=ERR, 1=WARN, 2=INFO, 3=DBUG
    int32_t ota_server_port = 8032;

    // 12. EtherNet/IP (Developer)
    bool eip_scanner_enabled = false;
    bool eip_axis_x = true;
    bool eip_axis_z = true;
    bool eip_axis_theta = false;
    char eip_target_ip_x[32] = "192.168.1.20";
    char eip_target_ip_z[32] = "192.168.1.21";
    char eip_target_ip_theta[32] = "192.168.1.23";
    char eip_w5500_ip[32] = "192.168.1.10";
    char eip_w5500_subnet[32] = "255.255.255.0";
    char eip_w5500_gateway[32] = "192.168.1.1";
    int32_t eip_w5500_spi_hz = 20000000;
    float eip_axis_x_puu_per_mm = 52428.8f;
    float eip_axis_z_puu_per_mm = 104857.6f;
    float eip_axis_theta_puu_per_deg = 10000.0f;
    int32_t eip_x_rpi_us = 2000;
    int32_t eip_theta_rpi_us = 2000;
    int32_t eip_endstop_source = 0; // 0=Drive, 1=MCP23S17
    float axis_theta_drive_abs_min_deg = -180.0f;
    float axis_theta_drive_abs_max_deg = 180.0f;
    char hcs01_eng_ip[32] = "192.168.1.22";

    // 13. Developer Mode Password
    char dev_mode_password[32] = "DEV_2026";
};

} // namespace Config

#endif // APP_CONFIG_DATA_H
