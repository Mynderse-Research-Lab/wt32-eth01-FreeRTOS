/**
 * @file AppConfigParamIo.cpp
 * @brief Reading and writing every stored key through IParamIo.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

namespace {

// A renamed key is read under its old name first, so a set stored by older
// firmware keeps its value; the new name wins when both are present.
class RenamedKeys : public IParamIo {
public:
    explicit RenamedKeys(IParamIo& inner) : inner_(inner) {}

    bool getInt32(const char* key, int32_t& out_val) override {
        const bool old = readOld(key, [&](const char* k) { return inner_.getInt32(k, out_val); });
        return inner_.getInt32(key, out_val) || old;
    }
    bool getFloat(const char* key, float& out_val) override {
        const bool old = readOld(key, [&](const char* k) { return inner_.getFloat(k, out_val); });
        return inner_.getFloat(key, out_val) || old;
    }
    bool getBool(const char* key, bool& out_val) override {
        const bool old = readOld(key, [&](const char* k) { return inner_.getBool(k, out_val); });
        return inner_.getBool(key, out_val) || old;
    }
    bool getString(const char* key, char* out_buf, size_t max_len) override {
        const bool old =
            readOld(key, [&](const char* k) { return inner_.getString(k, out_buf, max_len); });
        return inner_.getString(key, out_buf, max_len) || old;
    }

    bool setInt32(const char*, int32_t) override { return false; }
    bool setFloat(const char*, float) override { return false; }
    bool setBool(const char*, bool) override { return false; }
    bool setString(const char*, const char*) override { return false; }

private:
    template <typename Read>
    static bool readOld(const char* key, Read read) {
        const char* old = legacyParamKey(key);
        return old != nullptr && read(old);
    }

    IParamIo& inner_;
};

}  // namespace

void AppConfig::readParams(IParamIo& store) {
    RenamedKeys io(store);
    io.getInt32("spd_mm_s", data_.gantry_default_speed_mm_per_s);
    io.getInt32("acc_mm_s2", data_.gantry_default_accel_mm_per_s2);
    io.getInt32("dec_mm_s2", data_.gantry_default_decel_mm_per_s2);
    io.getInt32("th_spd_deg_s", data_.gantry_default_speed_deg_per_s);
    io.getInt32("th_acc_deg_s2", data_.gantry_default_accel_deg_per_s2);
    io.getInt32("th_dec_deg_s2", data_.gantry_default_decel_deg_per_s2);
    io.getInt32("ceil_spd_mm_s", data_.gantry_path_max_speed_mm_per_s);
    io.getInt32("ceil_acc_mm_s2", data_.gantry_path_max_accel_mm_per_s2);
    io.getInt32("ceil_dec_mm_s2", data_.gantry_path_max_decel_mm_per_s2);
    io.getInt32("eip_stop_dec", data_.gantry_eip_assumed_stop_decel_mm_s2);

    io.getFloat("x_lead_mm", data_.axis_x_lead_mm_per_rev);
    io.getFloat("x_lim_min_mm", data_.axis_x_hard_limit_min_mm);
    io.getFloat("x_lim_max_mm", data_.axis_x_hard_limit_max_mm);
    io.getFloat("x_max_spd_mm_s", data_.axis_x_max_speed_mm_per_s);
    io.getFloat("x_acc_mm_s2", data_.axis_x_accel_mm_per_s2);
    io.getFloat("x_dec_mm_s2", data_.axis_x_decel_mm_per_s2);
    io.getFloat("x_tol_mm", data_.axis_x_position_tolerance_mm);
    io.getInt32("x_enc_ppr", data_.axis_x_encoder_ppr);
    io.getFloat("x_gear_ratio", data_.axis_x_motor_reducer_ratio);
    io.getInt32("x_home_pps", data_.axis_x_homing_speed_pps);

    io.getFloat("z_lead_mm", data_.axis_z_lead_mm_per_rev);
    io.getInt32("z_crit_rpm", data_.axis_z_critical_rpm);
    io.getFloat("z_lim_min_mm", data_.axis_z_hard_limit_min_mm);
    io.getFloat("z_lim_max_mm", data_.axis_z_hard_limit_max_mm);
    io.getFloat("z_max_spd_mm_s", data_.axis_z_max_speed_mm_per_s);
    io.getFloat("z_acc_mm_s2", data_.axis_z_accel_mm_per_s2);
    io.getFloat("z_dec_mm_s2", data_.axis_z_decel_mm_per_s2);
    io.getFloat("z_tol_mm", data_.axis_z_position_tolerance_mm);
    io.getInt32("z_enc_ppr", data_.axis_z_encoder_ppr);
    io.getFloat("z_gear_ratio", data_.axis_z_motor_reducer_ratio);
    io.getBool("z_has_brake", data_.axis_z_has_motor_brake);
    io.getInt32("z_home_pps", data_.axis_z_homing_speed_pps);
    io.getFloat("z_datum_mm", data_.gantry_z_datum_offset_above_bed_mm);

    io.getFloat("th_gear_ratio", data_.axis_theta_output_gear_ratio);
    io.getFloat("th_lim_min_deg", data_.axis_theta_hard_limit_min_deg);
    io.getFloat("th_lim_max_deg", data_.axis_theta_hard_limit_max_deg);
    io.getFloat("th_spd_max", data_.axis_theta_max_speed_deg_per_s);
    io.getFloat("th_acc_deg", data_.axis_theta_accel_deg_per_s2);
    io.getFloat("th_dec_deg", data_.axis_theta_decel_deg_per_s2);
    io.getFloat("th_tol_deg", data_.axis_theta_position_tolerance_deg);
    io.getInt32("th_enc_ppr", data_.axis_theta_encoder_ppr);
    io.getFloat("th_reducer", data_.axis_theta_motor_reducer_ratio);
    io.getInt32("th_home_pps", data_.axis_theta_homing_speed_pps);
    io.getBool("th_sequential", data_.gantry_theta_sequential);

    io.getFloat("geom_z_y_mm", data_.gantry_z_axis_y_offset_mm);
    io.getFloat("geom_th_x_mm", data_.gantry_theta_x_offset_mm);
    io.getFloat("geom_grp_x_mm", data_.gantry_gripper_x_offset_mm);
    io.getFloat("geom_grp_z_mm", data_.gantry_gripper_z_offset_mm);
    io.getFloat("geom_safe_z_mm", data_.gantry_safe_z_height_mm);
    io.getFloat("geom_cal_x_mm", data_.gantry_cal_x_park_mm);
    io.getFloat("geom_cal_z_mm", data_.gantry_cal_z_park_mm);
    io.getFloat("geom_cal_th_deg", data_.gantry_cal_theta_park_deg);
    io.getFloat("col_x_min_mm", data_.gantry_conveyor_collision_x_min_mm);
    io.getFloat("col_z_min_mm", data_.gantry_conveyor_collision_z_min_mm);
    io.getInt32("grp_open_ms", data_.gantry_gripper_open_time_ms);
    io.getInt32("grp_close_ms", data_.gantry_gripper_close_time_ms);

    io.getFloat("vis_cam_y_mm", data_.conveyor_y_cam_mm);
    io.getFloat("cv_pick_y_mm", data_.conveyor_y_pick_mm);
    io.getFloat("vis_x_off_mm", data_.conveyor_x_across_to_gantry_x_offset_mm);
    io.getFloat("cv_belt_w_mm", data_.conveyor_belt_width_mm);
    io.getFloat("cv_belt_near_mm", data_.conveyor_belt_near_x_mm);
    io.getFloat("cv_belt_far_mm", data_.conveyor_belt_far_x_mm);
    io.getFloat("cv_cnv_near_mm", data_.conveyor_frame_near_x_mm);
    io.getFloat("cv_cnv_far_mm", data_.conveyor_frame_far_x_mm);
    io.getFloat("batt_x", data_.test_battery_x_mm);
    io.getFloat("cv_belt_spd", data_.pick_default_belt_speed_mm_s);
    io.getInt32("vis_lat_ms", data_.pick_vision_latency_ms);
    io.getInt32("grp_valve_ms", data_.pick_valve_ms);
    io.getInt32("pk_von_ms", data_.pick_valve_on_ms);
    io.getInt32("pk_z_dead_ms", data_.pick_z_plunge_dead_ms);
    io.getInt32("pk_z_ret_ms", data_.pick_z_retract_dead_ms);
    io.getInt32("pk_off_plunge", data_.pick_offset_plunge_ms);
    io.getInt32("pk_off_close", data_.pick_offset_close_ms);
    io.getInt32("vis_batt_x", data_.vision_x_from_batt);
    io.getInt32("pk_off_retract", data_.pick_offset_retract_ms);
    io.getInt32("eip_spi_khz", data_.eip_w5500_spi_khz);
    io.getInt32("disp_tft_en", data_.display_tft_enable);
    io.getInt32("eip_txwr_split", data_.eip_txwr_split);
    io.getInt32("eip_spi_dma", data_.eip_spi_dma);
    io.getInt32("log_uart_async", data_.log_uart_async);
    io.getInt32("eip_scope_gpio", data_.eip_scope_gpio);
    for (int i = 0; i < kPickClassSlots; ++i) {
      io.getInt32(kPickJawKeys[i], data_.pick_jaw_ms[i]);
    }
    io.getFloat("jaw_off_deg", data_.pick_jaw_offset_deg);
    io.getFloat("vis_dedupe_mm", data_.pick_dedupe_mm);
    for (int i = 0; i < kPickClassSlots; ++i) {
    }
    io.getFloat("fin_open_mm", data_.pick_finger_open_mm);
    io.getFloat("fin_wid_mm", data_.pick_finger_thick_mm);
    io.getFloat("fin_len_mm", data_.pick_finger_width_mm);
    io.getFloat("fin_spd_mm_s", data_.pick_jaw_speed_mm_s);
    io.getInt32("pk_guard_ms", data_.pick_guard_ms);
    io.getBool("pk_dropin", data_.pick_dropin_enable);
    io.getBool("pk_drop_any_th", data_.pick_drop_any_theta);
    io.getFloat("pk_stage_x_mm", data_.pick_stage_x_mm);

    io.getFloat("cal_spd_mm_s", data_.eip_home_cal_speed_mm_s);
    io.getFloat("cal_th_spd_deg", data_.eip_home_theta_speed_deg_s);
    io.getFloat("cal_acc_mm_s2", data_.eip_home_cal_accel_mm_s2);
    io.getFloat("cal_creep_mm_s", data_.eip_creep_speed_mm_s);
    io.getInt32("cal_timeout_ms", data_.gantry_calibration_timeout_ms);
    io.getInt32("trv_timeout_ms", data_.gantry_travel_measurement_timeout_ms);

    io.getBool("eth_static", data_.eth_use_static_ip);
    io.getString("eth_ip", data_.eth_static_ip, sizeof(data_.eth_static_ip));
    io.getString("eth_gw", data_.eth_static_gateway, sizeof(data_.eth_static_gateway));
    io.getString("ntp_server", data_.ntp_server, sizeof(data_.ntp_server));
    io.getString("eth_netmask", data_.eth_static_netmask, sizeof(data_.eth_static_netmask));
    io.getInt32("eth_timeout_ms", data_.eth_ip_wait_timeout_ms);

    io.getInt32("disp_orient", data_.display_orientation);
    io.getInt32("disp_bright", data_.display_brightness);
    io.getInt32("disp_bl_to_s", data_.display_backlight_timeout_s);

    io.getBool("con_auth", data_.console_tcp_auth_enable);
    io.getString("con_pw", data_.console_tcp_password, sizeof(data_.console_tcp_password));
    io.getInt32("con_tries", data_.console_tcp_auth_max_tries);
    io.getInt32("con_ttl_s", data_.console_tcp_auth_remember_s);
    io.getInt32("con_max_peers", data_.console_tcp_auth_remember_max);

    io.getBool("con_tcp_en", data_.console_tcp_enable);
    io.getBool("con_uart_en", data_.console_uart_enable);
    io.getInt32("con_port", data_.console_tcp_port);
    io.getBool("con_log_en", data_.console_tcp_log_enable);
    io.getInt32("con_log_lvl", data_.console_tcp_log_level);
    io.getInt32("ota_port", data_.ota_server_port);

    io.getBool("eip_en", data_.eip_scanner_enabled);
    io.getBool("eip_x_en", data_.eip_axis_x);
    io.getBool("eip_z_en", data_.eip_axis_z);
    io.getBool("eip_th_en", data_.eip_axis_theta);
    io.getString("eip_ip_x", data_.eip_target_ip_x, sizeof(data_.eip_target_ip_x));
    io.getString("eip_ip_z", data_.eip_target_ip_z, sizeof(data_.eip_target_ip_z));
    io.getString("eip_ip_th", data_.eip_target_ip_theta, sizeof(data_.eip_target_ip_theta));
    io.getString("w5500_ip", data_.eip_w5500_ip, sizeof(data_.eip_w5500_ip));
    io.getString("w5500_sub", data_.eip_w5500_subnet, sizeof(data_.eip_w5500_subnet));
    io.getString("w5500_gw", data_.eip_w5500_gateway, sizeof(data_.eip_w5500_gateway));
    io.getInt32("w5500_spi_hz", data_.eip_w5500_spi_hz);
    io.getFloat("eip_x_puu_mm", data_.eip_axis_x_puu_per_mm);
    io.getFloat("eip_z_puu_mm", data_.eip_axis_z_puu_per_mm);
    io.getFloat("eip_th_puu_deg", data_.eip_axis_theta_puu_per_deg);
    io.getInt32("eip_x_rpi_us", data_.eip_x_rpi_us);
    io.getInt32("eip_th_rpi_us", data_.eip_theta_rpi_us);
    io.getInt32("eip_endstop_src", data_.eip_endstop_source);
    io.getFloat("th_abs_min_deg", data_.axis_theta_drive_abs_min_deg);
    io.getFloat("th_abs_max_deg", data_.axis_theta_drive_abs_max_deg);
    io.getString("hcs01_eng_ip", data_.hcs01_eng_ip, sizeof(data_.hcs01_eng_ip));

    io.getString("dev_pw", data_.dev_mode_password, sizeof(data_.dev_mode_password));
}

void AppConfig::writeParams(IParamIo& io) const {
    io.setInt32("spd_mm_s", data_.gantry_default_speed_mm_per_s);
    io.setInt32("acc_mm_s2", data_.gantry_default_accel_mm_per_s2);
    io.setInt32("dec_mm_s2", data_.gantry_default_decel_mm_per_s2);
    io.setInt32("th_spd_deg_s", data_.gantry_default_speed_deg_per_s);
    io.setInt32("th_acc_deg_s2", data_.gantry_default_accel_deg_per_s2);
    io.setInt32("th_dec_deg_s2", data_.gantry_default_decel_deg_per_s2);
    io.setInt32("ceil_spd_mm_s", data_.gantry_path_max_speed_mm_per_s);
    io.setInt32("ceil_acc_mm_s2", data_.gantry_path_max_accel_mm_per_s2);
    io.setInt32("ceil_dec_mm_s2", data_.gantry_path_max_decel_mm_per_s2);
    io.setInt32("eip_stop_dec", data_.gantry_eip_assumed_stop_decel_mm_s2);

    io.setFloat("x_lead_mm", data_.axis_x_lead_mm_per_rev);
    io.setFloat("x_lim_min_mm", data_.axis_x_hard_limit_min_mm);
    io.setFloat("x_lim_max_mm", data_.axis_x_hard_limit_max_mm);
    io.setFloat("x_max_spd_mm_s", data_.axis_x_max_speed_mm_per_s);
    io.setFloat("x_acc_mm_s2", data_.axis_x_accel_mm_per_s2);
    io.setFloat("x_dec_mm_s2", data_.axis_x_decel_mm_per_s2);
    io.setFloat("x_tol_mm", data_.axis_x_position_tolerance_mm);
    io.setInt32("x_enc_ppr", data_.axis_x_encoder_ppr);
    io.setFloat("x_gear_ratio", data_.axis_x_motor_reducer_ratio);
    io.setInt32("x_home_pps", data_.axis_x_homing_speed_pps);

    io.setFloat("z_lead_mm", data_.axis_z_lead_mm_per_rev);
    io.setInt32("z_crit_rpm", data_.axis_z_critical_rpm);
    io.setFloat("z_lim_min_mm", data_.axis_z_hard_limit_min_mm);
    io.setFloat("z_lim_max_mm", data_.axis_z_hard_limit_max_mm);
    io.setFloat("z_max_spd_mm_s", data_.axis_z_max_speed_mm_per_s);
    io.setFloat("z_acc_mm_s2", data_.axis_z_accel_mm_per_s2);
    io.setFloat("z_dec_mm_s2", data_.axis_z_decel_mm_per_s2);
    io.setFloat("z_tol_mm", data_.axis_z_position_tolerance_mm);
    io.setInt32("z_enc_ppr", data_.axis_z_encoder_ppr);
    io.setFloat("z_gear_ratio", data_.axis_z_motor_reducer_ratio);
    io.setBool("z_has_brake", data_.axis_z_has_motor_brake);
    io.setInt32("z_home_pps", data_.axis_z_homing_speed_pps);
    io.setFloat("z_datum_mm", data_.gantry_z_datum_offset_above_bed_mm);

    io.setFloat("th_gear_ratio", data_.axis_theta_output_gear_ratio);
    io.setFloat("th_lim_min_deg", data_.axis_theta_hard_limit_min_deg);
    io.setFloat("th_lim_max_deg", data_.axis_theta_hard_limit_max_deg);
    io.setFloat("th_spd_max", data_.axis_theta_max_speed_deg_per_s);
    io.setFloat("th_acc_deg", data_.axis_theta_accel_deg_per_s2);
    io.setFloat("th_dec_deg", data_.axis_theta_decel_deg_per_s2);
    io.setFloat("th_tol_deg", data_.axis_theta_position_tolerance_deg);
    io.setInt32("th_enc_ppr", data_.axis_theta_encoder_ppr);
    io.setFloat("th_reducer", data_.axis_theta_motor_reducer_ratio);
    io.setInt32("th_home_pps", data_.axis_theta_homing_speed_pps);
    io.setBool("th_sequential", data_.gantry_theta_sequential);

    io.setFloat("geom_z_y_mm", data_.gantry_z_axis_y_offset_mm);
    io.setFloat("geom_th_x_mm", data_.gantry_theta_x_offset_mm);
    io.setFloat("geom_grp_x_mm", data_.gantry_gripper_x_offset_mm);
    io.setFloat("geom_grp_z_mm", data_.gantry_gripper_z_offset_mm);
    io.setFloat("geom_safe_z_mm", data_.gantry_safe_z_height_mm);
    io.setFloat("geom_cal_x_mm", data_.gantry_cal_x_park_mm);
    io.setFloat("geom_cal_z_mm", data_.gantry_cal_z_park_mm);
    io.setFloat("geom_cal_th_deg", data_.gantry_cal_theta_park_deg);
    io.setFloat("col_x_min_mm", data_.gantry_conveyor_collision_x_min_mm);
    io.setFloat("col_z_min_mm", data_.gantry_conveyor_collision_z_min_mm);
    io.setInt32("grp_open_ms", data_.gantry_gripper_open_time_ms);
    io.setInt32("grp_close_ms", data_.gantry_gripper_close_time_ms);

    io.setFloat("vis_cam_y_mm", data_.conveyor_y_cam_mm);
    io.setFloat("cv_pick_y_mm", data_.conveyor_y_pick_mm);
    io.setFloat("vis_x_off_mm", data_.conveyor_x_across_to_gantry_x_offset_mm);
    io.setFloat("cv_belt_w_mm", data_.conveyor_belt_width_mm);
    io.setFloat("cv_belt_near_mm", data_.conveyor_belt_near_x_mm);
    io.setFloat("cv_belt_far_mm", data_.conveyor_belt_far_x_mm);
    io.setFloat("cv_cnv_near_mm", data_.conveyor_frame_near_x_mm);
    io.setFloat("cv_cnv_far_mm", data_.conveyor_frame_far_x_mm);
    io.setFloat("batt_x", data_.test_battery_x_mm);
    io.setFloat("cv_belt_spd", data_.pick_default_belt_speed_mm_s);
    io.setInt32("vis_lat_ms", data_.pick_vision_latency_ms);
    io.setInt32("grp_valve_ms", data_.pick_valve_ms);
    io.setInt32("pk_von_ms", data_.pick_valve_on_ms);
    io.setInt32("pk_z_dead_ms", data_.pick_z_plunge_dead_ms);
    io.setInt32("pk_z_ret_ms", data_.pick_z_retract_dead_ms);
    io.setInt32("pk_off_plunge", data_.pick_offset_plunge_ms);
    io.setInt32("pk_off_close", data_.pick_offset_close_ms);
    io.setInt32("vis_batt_x", data_.vision_x_from_batt);
    io.setInt32("pk_off_retract", data_.pick_offset_retract_ms);
    io.setInt32("eip_spi_khz", data_.eip_w5500_spi_khz);
    io.setInt32("disp_tft_en", data_.display_tft_enable);
    io.setInt32("eip_txwr_split", data_.eip_txwr_split);
    io.setInt32("eip_spi_dma", data_.eip_spi_dma);
    io.setInt32("log_uart_async", data_.log_uart_async);
    io.setInt32("eip_scope_gpio", data_.eip_scope_gpio);
    for (int i = 0; i < kPickClassSlots; ++i) {
      io.setInt32(kPickJawKeys[i], data_.pick_jaw_ms[i]);
    }
    io.setFloat("jaw_off_deg", data_.pick_jaw_offset_deg);
    io.setFloat("vis_dedupe_mm", data_.pick_dedupe_mm);
    for (int i = 0; i < kPickClassSlots; ++i) {
    }
    io.setFloat("fin_open_mm", data_.pick_finger_open_mm);
    io.setFloat("fin_wid_mm", data_.pick_finger_thick_mm);
    io.setFloat("fin_len_mm", data_.pick_finger_width_mm);
    io.setFloat("fin_spd_mm_s", data_.pick_jaw_speed_mm_s);
    io.setInt32("pk_guard_ms", data_.pick_guard_ms);
    io.setBool("pk_dropin", data_.pick_dropin_enable);
    io.setBool("pk_drop_any_th", data_.pick_drop_any_theta);
    io.setFloat("pk_stage_x_mm", data_.pick_stage_x_mm);

    io.setFloat("cal_spd_mm_s", data_.eip_home_cal_speed_mm_s);
    io.setFloat("cal_th_spd_deg", data_.eip_home_theta_speed_deg_s);
    io.setFloat("cal_acc_mm_s2", data_.eip_home_cal_accel_mm_s2);
    io.setFloat("cal_creep_mm_s", data_.eip_creep_speed_mm_s);
    io.setInt32("cal_timeout_ms", data_.gantry_calibration_timeout_ms);
    io.setInt32("trv_timeout_ms", data_.gantry_travel_measurement_timeout_ms);

    io.setBool("eth_static", data_.eth_use_static_ip);
    io.setString("eth_ip", data_.eth_static_ip);
    io.setString("eth_gw", data_.eth_static_gateway);
    io.setString("ntp_server", data_.ntp_server);
    io.setString("eth_netmask", data_.eth_static_netmask);
    io.setInt32("eth_timeout_ms", data_.eth_ip_wait_timeout_ms);

    io.setInt32("disp_orient", data_.display_orientation);
    io.setInt32("disp_bright", data_.display_brightness);
    io.setInt32("disp_bl_to_s", data_.display_backlight_timeout_s);

    io.setBool("con_auth", data_.console_tcp_auth_enable);
    io.setString("con_pw", data_.console_tcp_password);
    io.setInt32("con_tries", data_.console_tcp_auth_max_tries);
    io.setInt32("con_ttl_s", data_.console_tcp_auth_remember_s);
    io.setInt32("con_max_peers", data_.console_tcp_auth_remember_max);

    io.setBool("con_tcp_en", data_.console_tcp_enable);
    io.setBool("con_uart_en", data_.console_uart_enable);
    io.setInt32("con_port", data_.console_tcp_port);
    io.setBool("con_log_en", data_.console_tcp_log_enable);
    io.setInt32("con_log_lvl", data_.console_tcp_log_level);
    io.setInt32("ota_port", data_.ota_server_port);

    io.setBool("eip_en", data_.eip_scanner_enabled);
    io.setBool("eip_x_en", data_.eip_axis_x);
    io.setBool("eip_z_en", data_.eip_axis_z);
    io.setBool("eip_th_en", data_.eip_axis_theta);
    io.setString("eip_ip_x", data_.eip_target_ip_x);
    io.setString("eip_ip_z", data_.eip_target_ip_z);
    io.setString("eip_ip_th", data_.eip_target_ip_theta);
    io.setString("w5500_ip", data_.eip_w5500_ip);
    io.setString("w5500_sub", data_.eip_w5500_subnet);
    io.setString("w5500_gw", data_.eip_w5500_gateway);
    io.setInt32("w5500_spi_hz", data_.eip_w5500_spi_hz);
    io.setFloat("eip_x_puu_mm", data_.eip_axis_x_puu_per_mm);
    io.setFloat("eip_z_puu_mm", data_.eip_axis_z_puu_per_mm);
    io.setFloat("eip_th_puu_deg", data_.eip_axis_theta_puu_per_deg);
    io.setInt32("eip_x_rpi_us", data_.eip_x_rpi_us);
    io.setInt32("eip_th_rpi_us", data_.eip_theta_rpi_us);
    io.setInt32("eip_endstop_src", data_.eip_endstop_source);
    io.setFloat("th_abs_min_deg", data_.axis_theta_drive_abs_min_deg);
    io.setFloat("th_abs_max_deg", data_.axis_theta_drive_abs_max_deg);
    io.setString("hcs01_eng_ip", data_.hcs01_eng_ip);

    io.setString("dev_pw", data_.dev_mode_password);
}

} // namespace Config
