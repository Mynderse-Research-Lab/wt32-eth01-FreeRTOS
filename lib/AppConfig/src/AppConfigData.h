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
    int32_t gantry_path_max_speed_mm_per_s = 3333;
    int32_t gantry_path_max_accel_mm_per_s2 = 20000;
    int32_t gantry_path_max_decel_mm_per_s2 = 20000;
    int32_t gantry_eip_assumed_stop_decel_mm_s2 = 300;

    // 2. X Axis
    float axis_x_lead_mm_per_rev = 200.0f;
    float axis_x_hard_limit_min_mm = 0.0f;
    float axis_x_hard_limit_max_mm = 550.0f;
    float axis_x_max_speed_mm_per_s = 3333.3f;
    float axis_x_accel_mm_per_s2 = 20000.0f;
    float axis_x_decel_mm_per_s2 = 20000.0f;
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
    float gantry_safe_z_height_mm = 48.4f;
    float gantry_cal_x_park_mm = 10.0f;
    float gantry_cal_z_park_mm = 90.8f;
    float gantry_cal_theta_park_deg = 90.0f;
    float gantry_conveyor_collision_x_min_mm = 95.0f;
    float gantry_conveyor_collision_z_min_mm = 115.0f;
    int32_t gantry_gripper_open_time_ms = 190;
    int32_t gantry_gripper_close_time_ms = 190;

    // 6. Conveyor Intercept
    float conveyor_y_cam_mm = -711.0f;
    float conveyor_y_pick_mm = 0.0f;
    // Vision x is from the camera. Gantry X = x + this. Belt center is the
    // collision edge (95 mm) plus half the 300 mm belt (150 mm).
    float conveyor_x_across_to_gantry_x_offset_mm = 245.0f;
    float conveyor_belt_width_mm = 300.0f;
    // Belt edges (beltcal near/far), conveyor frame edges (beltcal cnvyr
    // near/far) and the test battery (beltcal batt), all joint X as `move` uses.
    float conveyor_belt_near_x_mm = 225.0f;
    float conveyor_belt_far_x_mm = 500.0f;
    float conveyor_frame_near_x_mm = 183.0f;
    float conveyor_frame_far_x_mm = 500.0f;
    // Test battery position, joint X (beltcal batt).
    float test_battery_x_mm = 382.5f;
    int32_t pick_vision_latency_ms = 40;
    int32_t pick_valve_ms = 30;
    int32_t pick_valve_on_ms = 8;
    int32_t pick_valve_off_ms = 17;
    int32_t pick_trim_von_ms = 0;
    int32_t pick_trim_voff_ms = 0;
    int32_t pick_trim_close_ms = 0;
    int32_t pick_trim_z_ms = 0;
    int32_t pick_trim_grip_ms = 0;
    int32_t pick_trim_hold_ms = 0;
    int32_t pick_offset_plunge_ms = 0;
    int32_t pick_offset_close_ms = 0;
    // 1: vision detections take x_across from batt_x (fixed trigger-sensor frames).
    int32_t vision_x_from_batt = 0;
    int32_t pick_offset_retract_ms = 0;
    // Class 1 dropout experiments (applied at boot): W5500 SPI2 clock and
    // whether the TFT UI runs (SPI3 display traffic on/off).
    int32_t eip_w5500_spi_khz = 10000;
    int32_t display_tft_enable = 1;
    int32_t eip_txwr_split = 0;
    // 0 (default): W5500 SPI2 without DMA. Logic-analyzer capture 2026-10-20:
    // with DMA, a 4-byte (TXDATA) transfer right after a DMA transfer put the
    // previous DMA buffer's first bytes on MOSI (SEND became a stale TX_WR
    // write), which caused the lost/misplaced W5500 writes and Class 1 drops.
    int32_t eip_spi_dma = 0;
    int32_t log_uart_async = 1;  // buffered UART0 log output (driver TX ring)
    int32_t eip_scope_gpio = -1; // Class 1 exchange marker GPIO (-1 off; 17 with TFT off)
    bool pick_bench_enable = false;
    int32_t pick_jaw_ms[10] = {};  // 0 = derive from jaw travel. One slot per pick class.
    float pick_jaw_offset_deg = 90.0f;
    float pick_dedupe_mm = 40.0f;
    float pick_default_belt_speed_mm_s = 1524.0f;
    // Length and height labels for classes 1–10. Not a vision fallback.
    float pick_cell_len_mm[10] = {44.5f, 50.5f, 61.5f, 50.0f, 48.5f, 34.5f, 65.0f, 70.0f, 65.0f, 65.0f};
    float pick_cell_h_mm[10] = {10.5f, 14.5f, 34.2f, 26.2f, 26.5f, 17.0f, 18.3f, 21.2f, 26.5f, 32.0f};
    float pick_finger_open_mm = 80.0f;
    float pick_finger_thick_mm = 10.0f;
    float pick_finger_width_mm = 50.0f;
    float pick_jaw_speed_mm_s = 210.0f;
    int32_t pick_guard_ms = 15;
    bool pick_dropin_enable = false;
    bool pick_drop_any_theta = false;
    float pick_stage_x_mm = 320.0f;
    int32_t tau_min_us = 100000;
    int32_t tau_max_us = 5000000;

    // 7. Homing & Calibration
    float eip_home_cal_speed_mm_s = 50.0f;
    float eip_home_theta_speed_deg_s = 15.0f;
    float eip_home_cal_accel_mm_s2 = 2000.0f;
    float eip_creep_speed_mm_s = 1.0f;
    int32_t gantry_calibration_timeout_ms = 20000;
    int32_t gantry_travel_measurement_timeout_ms = 20000;

    // 8. Network (LAN8720)
    bool eth_use_static_ip = true;
    char eth_static_ip[32] = "192.168.1.100";
    char eth_static_gateway[32] = "192.168.1.5";
    char eth_static_netmask[32] = "255.255.255.0";
    int32_t eth_ip_wait_timeout_ms = 1500;

    // 9. Display
    int32_t display_orientation = 0; // 0 = Landscape, 1 = Portrait
    int32_t display_brightness = 100; // 0..100 % (PWM brightness)
    int32_t display_backlight_timeout_s = 10;

    // 10. Console Security (Developer)
    bool console_tcp_auth_enable = true;
    char console_tcp_password[32] = "LTU_1932";
    int32_t console_tcp_auth_max_tries = 3;
    int32_t console_tcp_auth_remember_s = 60;
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
