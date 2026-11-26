/**
 * @file AppConfigAccess.cpp
 * @brief Typed get and set of one parameter by key.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

namespace {

int pickJawSlot(const char* key) {
  for (int i = 0; i < kPickClassSlots; ++i) {
    if (std::strcmp(key, kPickJawKeys[i]) == 0) {
      return i;
    }
  }
  return -1;
}

}  // namespace

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
    else if (std::strcmp(key, "vis_lat_ms") == 0) out_val = data_.pick_vision_latency_ms;
    else if (std::strcmp(key, "grp_valve_ms") == 0) out_val = data_.pick_valve_ms;
    else if (std::strcmp(key, "pk_von_ms") == 0) out_val = data_.pick_valve_on_ms;
    else if (std::strcmp(key, "pk_z_dead_ms") == 0) out_val = data_.pick_z_plunge_dead_ms;
    else if (std::strcmp(key, "pk_z_ret_ms") == 0) out_val = data_.pick_z_retract_dead_ms;
    else if (std::strcmp(key, "pk_off_plunge") == 0) out_val = data_.pick_offset_plunge_ms;
    else if (std::strcmp(key, "pk_off_close") == 0) out_val = data_.pick_offset_close_ms;
    else if (std::strcmp(key, "vis_batt_x") == 0) out_val = data_.vision_x_from_batt;
    else if (std::strcmp(key, "pk_off_retract") == 0) out_val = data_.pick_offset_retract_ms;
    else if (std::strcmp(key, "eip_spi_khz") == 0) out_val = data_.eip_w5500_spi_khz;
    else if (std::strcmp(key, "disp_tft_en") == 0) out_val = data_.display_tft_enable;
    else if (std::strcmp(key, "eip_txwr_split") == 0) out_val = data_.eip_txwr_split;
    else if (std::strcmp(key, "eip_spi_dma") == 0) out_val = data_.eip_spi_dma;
    else if (std::strcmp(key, "log_uart_async") == 0) out_val = data_.log_uart_async;
    else if (std::strcmp(key, "eip_scope_gpio") == 0) out_val = data_.eip_scope_gpio;
    else if (const int jaw = pickJawSlot(key); jaw >= 0) out_val = data_.pick_jaw_ms[jaw];
    else if (std::strcmp(key, "pk_guard_ms") == 0) out_val = data_.pick_guard_ms;
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
    else if (std::strcmp(key, "vis_lat_ms") == 0) data_.pick_vision_latency_ms = val;
    else if (std::strcmp(key, "grp_valve_ms") == 0) data_.pick_valve_ms = val;
    else if (std::strcmp(key, "pk_von_ms") == 0) data_.pick_valve_on_ms = val;
    else if (std::strcmp(key, "pk_z_dead_ms") == 0) data_.pick_z_plunge_dead_ms = val;
    else if (std::strcmp(key, "pk_z_ret_ms") == 0) data_.pick_z_retract_dead_ms = val;
    else if (std::strcmp(key, "pk_off_plunge") == 0) data_.pick_offset_plunge_ms = val;
    else if (std::strcmp(key, "pk_off_close") == 0) data_.pick_offset_close_ms = val;
    else if (std::strcmp(key, "vis_batt_x") == 0) data_.vision_x_from_batt = val;
    else if (std::strcmp(key, "pk_off_retract") == 0) data_.pick_offset_retract_ms = val;
    else if (std::strcmp(key, "eip_spi_khz") == 0) data_.eip_w5500_spi_khz = val;
    else if (std::strcmp(key, "disp_tft_en") == 0) data_.display_tft_enable = val;
    else if (std::strcmp(key, "eip_txwr_split") == 0) data_.eip_txwr_split = val;
    else if (std::strcmp(key, "eip_spi_dma") == 0) data_.eip_spi_dma = val;
    else if (std::strcmp(key, "log_uart_async") == 0) data_.log_uart_async = val;
    else if (std::strcmp(key, "eip_scope_gpio") == 0) data_.eip_scope_gpio = val;
    else if (const int jaw = pickJawSlot(key); jaw >= 0) data_.pick_jaw_ms[jaw] = val;
    else if (std::strcmp(key, "pk_guard_ms") == 0) data_.pick_guard_ms = val;
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

    recordChange(key);
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
    else if (std::strcmp(key, "vis_cam_y_mm") == 0) out_val = data_.conveyor_y_cam_mm;
    else if (std::strcmp(key, "cv_pick_y_mm") == 0) out_val = data_.conveyor_y_pick_mm;
    else if (std::strcmp(key, "vis_x_off_mm") == 0) out_val = data_.conveyor_x_across_to_gantry_x_offset_mm;
    else if (std::strcmp(key, "cv_belt_w_mm") == 0) out_val = data_.conveyor_belt_width_mm;
    else if (std::strcmp(key, "cv_belt_near_mm") == 0) out_val = data_.conveyor_belt_near_x_mm;
    else if (std::strcmp(key, "cv_belt_far_mm") == 0) out_val = data_.conveyor_belt_far_x_mm;
    else if (std::strcmp(key, "cv_cnv_near_mm") == 0) out_val = data_.conveyor_frame_near_x_mm;
    else if (std::strcmp(key, "cv_cnv_far_mm") == 0) out_val = data_.conveyor_frame_far_x_mm;
    else if (std::strcmp(key, "batt_x") == 0) out_val = data_.test_battery_x_mm;
    else if (std::strcmp(key, "cv_belt_spd") == 0) out_val = data_.pick_default_belt_speed_mm_s;
    else if (std::strcmp(key, "jaw_off_deg") == 0) out_val = data_.pick_jaw_offset_deg;
    else if (std::strcmp(key, "vis_dedupe_mm") == 0) out_val = data_.pick_dedupe_mm;
    else if (std::strcmp(key, "fin_open_mm") == 0) out_val = data_.pick_finger_open_mm;
    else if (std::strcmp(key, "fin_wid_mm") == 0) out_val = data_.pick_finger_thick_mm;
    else if (std::strcmp(key, "fin_len_mm") == 0) out_val = data_.pick_finger_width_mm;
    else if (std::strcmp(key, "fin_spd_mm_s") == 0) out_val = data_.pick_jaw_speed_mm_s;
    else if (std::strcmp(key, "pk_stage_x_mm") == 0) out_val = data_.pick_stage_x_mm;
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
    else if (std::strcmp(key, "vis_cam_y_mm") == 0) data_.conveyor_y_cam_mm = val;
    else if (std::strcmp(key, "cv_pick_y_mm") == 0) data_.conveyor_y_pick_mm = val;
    else if (std::strcmp(key, "vis_x_off_mm") == 0) data_.conveyor_x_across_to_gantry_x_offset_mm = val;
    else if (std::strcmp(key, "cv_belt_w_mm") == 0) data_.conveyor_belt_width_mm = val;
    else if (std::strcmp(key, "cv_belt_near_mm") == 0) data_.conveyor_belt_near_x_mm = val;
    else if (std::strcmp(key, "cv_belt_far_mm") == 0) data_.conveyor_belt_far_x_mm = val;
    else if (std::strcmp(key, "cv_cnv_near_mm") == 0) data_.conveyor_frame_near_x_mm = val;
    else if (std::strcmp(key, "cv_cnv_far_mm") == 0) data_.conveyor_frame_far_x_mm = val;
    else if (std::strcmp(key, "batt_x") == 0) data_.test_battery_x_mm = val;
    else if (std::strcmp(key, "cv_belt_spd") == 0) data_.pick_default_belt_speed_mm_s = val;
    else if (std::strcmp(key, "jaw_off_deg") == 0) data_.pick_jaw_offset_deg = val;
    else if (std::strcmp(key, "vis_dedupe_mm") == 0) data_.pick_dedupe_mm = val;
    else if (std::strcmp(key, "fin_open_mm") == 0) data_.pick_finger_open_mm = val;
    else if (std::strcmp(key, "fin_wid_mm") == 0) data_.pick_finger_thick_mm = val;
    else if (std::strcmp(key, "fin_len_mm") == 0) data_.pick_finger_width_mm = val;
    else if (std::strcmp(key, "fin_spd_mm_s") == 0) data_.pick_jaw_speed_mm_s = val;
    else if (std::strcmp(key, "pk_stage_x_mm") == 0) data_.pick_stage_x_mm = val;
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

    recordChange(key);
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
    else if (std::strcmp(key, "pk_drop_any_th") == 0) data_.pick_drop_any_theta = val;
    else return false;

    recordChange(key);
    return true;
}

bool AppConfig::getParamString(const char* key, char* out_buf, size_t max_len) const {
    const ParamSchema* s = findParamSchema(key);
    if (!s || s->type != ParamType::STRING || !out_buf || max_len == 0) return false;

    const char* src = nullptr;
    if (std::strcmp(key, "eth_ip") == 0) src = data_.eth_static_ip;
    else if (std::strcmp(key, "eth_gw") == 0) src = data_.eth_static_gateway;
    else if (std::strcmp(key, "ntp_server") == 0) src = data_.ntp_server;
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
    else if (std::strcmp(key, "ntp_server") == 0) { dst = data_.ntp_server; cap = sizeof(data_.ntp_server); }
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

    recordChange(key);
    return true;
}

} // namespace Config
