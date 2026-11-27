/**
 * @file AppConfigDefaultsCell.cpp
 * @brief Compile-time defaults, sections 6-13: conveyor, homing, network, display, console, EtherNet/IP, developer password.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

void AppConfig::loadCellDefaults() {
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

#if defined(CONFIG_PICK_DEFAULT_BELT_SPEED_MM_S)
    data_.pick_default_belt_speed_mm_s = (float)CONFIG_PICK_DEFAULT_BELT_SPEED_MM_S;
#else
    data_.pick_default_belt_speed_mm_s = 1524.0f;
#endif

    data_.pick_vision_latency_ms = 40;
    data_.pick_valve_ms = 30;
    data_.pick_valve_on_ms = 8;
    data_.pick_z_plunge_dead_ms = 30;
    data_.pick_z_retract_dead_ms = 22;
    data_.pick_offset_plunge_ms = 0;
    data_.pick_offset_close_ms = 0;
    data_.vision_x_from_batt = 0;
    data_.pick_offset_retract_ms = 0;
    data_.eip_w5500_spi_khz = 10000;
    data_.display_tft_enable = 1;
    data_.eip_txwr_split = 0;
    data_.eip_spi_dma = 0;
    data_.log_uart_async = 1;
    data_.eip_scope_gpio = -1;
    for (int i = 0; i < kPickClassSlots; ++i) {
      data_.pick_jaw_ms[i] = 0;
    }
    data_.pick_jaw_offset_deg = 0.0f;
    data_.pick_dedupe_mm = 40.0f;
    data_.pick_finger_open_mm = 80.0f;
    data_.pick_finger_thick_mm = 10.0f;
    data_.pick_finger_width_mm = 50.0f;
    data_.pick_jaw_speed_mm_s = 210.0f;
    data_.pick_guard_ms = 15;
    data_.pick_dropin_enable = false;
    data_.pick_drop_any_theta = false;
    data_.pick_stage_x_mm = 320.0f;

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

#if defined(CONFIG_NTP_SERVER)
    std::strncpy(data_.ntp_server, CONFIG_NTP_SERVER, sizeof(data_.ntp_server) - 1);
#else
    std::strncpy(data_.ntp_server, "10.42.0.1", sizeof(data_.ntp_server) - 1);
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

} // namespace Config
