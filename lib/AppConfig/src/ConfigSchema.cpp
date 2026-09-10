#include "ConfigSchema.h"
#include <cstring>

namespace Config {

static const std::vector<ParamSchema> s_schemas = {
    // ========================================================================
    // 1. Motion Profile (End-User)
    // ========================================================================
    {"spd_mm_s",      "Default Speed",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 2000.0f, 5.0f, "mm/s", {}},
    {"acc_mm_s2",     "Default Accel",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"dec_mm_s2",     "Default Decel",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"th_spd_deg_s",  "Theta Speed",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 7200.0f, 5.0f, "deg/s", {}},
    {"th_acc_deg_s2", "Theta Accel",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 18000.0f, 10.0f, "deg/s2", {}},
    {"th_dec_deg_s2", "Theta Decel",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 18000.0f, 10.0f, "deg/s2", {}},
    {"ceil_spd_mm_s", "Speed Ceiling",    "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 2000.0f, 10.0f, "mm/s", {}},
    {"ceil_acc_mm_s2","Accel Ceiling",    "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"ceil_dec_mm_s2","Decel Ceiling",    "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"eip_stop_dec",  "EIP Stop Decel",    "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 50.0f, 5000.0f, 10.0f, "mm/s2", {}},

    // ========================================================================
    // 2. X Axis (End-User)
    // ========================================================================
    {"x_lead_mm",     "Lead / Rev",        "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  1.0f, 1000.0f, 1.0f, "mm", {}},
    {"x_lim_min_mm",  "Hard Limit Min",    "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  -2000.0f, 2000.0f, 1.0f, "mm", {}},
    {"x_lim_max_mm",  "Hard Limit Max",    "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  0.0f, 5000.0f, 1.0f, "mm", {}},
    {"x_max_spd_mm_s","Max Speed Clamp",   "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 5000.0f, 10.0f, "mm/s", {}},
    {"x_acc_mm_s2",   "Accel Envelope",    "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"x_dec_mm_s2",   "Decel Envelope",    "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"x_tol_mm",      "Pos Tolerance",     "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 0.001f, 10.0f, 0.01f, "mm", {}},
    {"x_enc_ppr",     "Encoder PPR",       "X Axis",         ParamType::INT,   AccessLevel::END_USER, true,  1.0f, 16777216.0f, 1000.0f, "pls", {}},
    {"x_gear_ratio",  "Reducer Ratio",     "X Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  0.1f, 100.0f, 0.1f, "", {}},
    {"x_home_pps",    "Homing Speed",      "X Axis",         ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 100000.0f, 500.0f, "pps", {}},

    // ========================================================================
    // 3. Z Axis (End-User)
    // ========================================================================
    {"z_lead_mm",     "Lead / Rev",        "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  1.0f, 100.0f, 1.0f, "mm", {}},
    {"z_crit_rpm",    "Critical RPM",      "Z Axis",         ParamType::INT,   AccessLevel::END_USER, true,  100.0f, 10000.0f, 100.0f, "rpm", {}},
    {"z_lim_min_mm",  "Hard Limit Min",    "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  -2000.0f, 2000.0f, 1.0f, "mm", {}},
    {"z_lim_max_mm",  "Hard Limit Max",    "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  0.0f, 5000.0f, 1.0f, "mm", {}},
    {"z_max_spd_mm_s","Max Speed Clamp",   "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 5000.0f, 10.0f, "mm/s", {}},
    {"z_acc_mm_s2",   "Accel Envelope",    "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"z_dec_mm_s2",   "Decel Envelope",    "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"z_tol_mm",      "Pos Tolerance",     "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, false, 0.001f, 10.0f, 0.005f, "mm", {}},
    {"z_enc_ppr",     "Encoder PPR",       "Z Axis",         ParamType::INT,   AccessLevel::END_USER, true,  1.0f, 16777216.0f, 1000.0f, "pls", {}},
    {"z_gear_ratio",  "Reducer Ratio",     "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  0.1f, 100.0f, 0.1f, "", {}},
    {"z_has_brake",   "Has Motor Brake",   "Z Axis",         ParamType::BOOL,  AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"z_home_pps",    "Homing Speed",      "Z Axis",         ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 100000.0f, 500.0f, "pps", {}},
    {"z_datum_mm",    "Z Datum Offset",    "Z Axis",         ParamType::FLOAT, AccessLevel::END_USER, true,  -500.0f, 500.0f, 1.0f, "mm", {}},

    // ========================================================================
    // 4. Theta Axis (End-User)
    // ========================================================================
    {"th_gear_ratio", "Output Gear Ratio", "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, true,  0.1f, 100.0f, 0.1f, "", {}},
    {"th_lim_min_deg","Hard Limit Min",    "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, true,  -360.0f, 360.0f, 5.0f, "deg", {}},
    {"th_lim_max_deg","Hard Limit Max",    "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, true,  -360.0f, 360.0f, 5.0f, "deg", {}},
    {"th_spd_max",    "Max Speed",         "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 7200.0f, 10.0f, "deg/s", {}},
    {"th_acc_deg",    "Accel Envelope",    "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, false, 10.0f, 50000.0f, 100.0f, "deg/s2", {}},
    {"th_dec_deg",    "Decel Envelope",    "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, false, 10.0f, 50000.0f, 100.0f, "deg/s2", {}},
    {"th_tol_deg",    "Pos Tolerance",     "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, false, 0.001f, 5.0f, 0.005f, "deg", {}},
    {"th_enc_ppr",    "Encoder PPR",       "Theta Axis",     ParamType::INT,   AccessLevel::END_USER, true,  1.0f, 16777216.0f, 1000.0f, "pls", {}},
    {"th_reducer",    "Reducer Ratio",     "Theta Axis",     ParamType::FLOAT, AccessLevel::END_USER, true,  0.1f, 100.0f, 0.1f, "", {}},
    {"th_home_pps",   "Homing Speed",      "Theta Axis",     ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 100000.0f, 500.0f, "pps", {}},
    {"th_sequential", "Sequential Move",   "Theta Axis",     ParamType::BOOL,  AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {}},

    // ========================================================================
    // 5. Geometry & Gripper (End-User)
    // ========================================================================
    {"geom_z_y_mm",   "Z-Col Y Offset",    "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_th_x_mm",  "Theta X Offset",    "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_grp_x_mm", "Gripper TCP X",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_grp_z_mm", "Gripper TCP Z",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_safe_z_mm","Safe Z Margin",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  1.0f, 150.0f, 1.0f, "mm", {}},
    {"geom_cal_x_mm", "Cal X Park",        "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 500.0f, 1.0f, "mm", {}},
    {"col_x_min_mm",  "Collision X Min",   "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 550.0f, 1.0f, "mm", {}},
    {"col_z_min_mm",  "Collision Z Min",   "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 150.0f, 1.0f, "mm", {}},
    {"grp_open_ms",   "Gripper Open Time", "Geometry",       ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 5000.0f, 10.0f, "ms", {}},
    {"grp_close_ms",  "Gripper Close Time","Geometry",       ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 5000.0f, 10.0f, "ms", {}},

    // ========================================================================
    // 6. Conveyor Intercept (End-User)
    // ========================================================================
    {"cv_cam_y_mm",   "Camera Y Pos",      "Conveyor",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 5000.0f, 1.0f, "mm", {}},
    {"cv_pick_y_mm",  "Pick Y Pos",        "Conveyor",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 5000.0f, 1.0f, "mm", {}},
    {"cv_x_off_mm",   "X Across Offset",   "Conveyor",       ParamType::FLOAT, AccessLevel::END_USER, false, -500.0f, 500.0f, 0.5f, "mm", {}},
    {"cv_z_pick_mm",  "Z Pick Joint Pos",  "Conveyor",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 150.0f, 1.0f, "mm", {}},
    {"cv_belt_spd",   "Default Belt Spd",  "Conveyor",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 5000.0f, 10.0f, "mm/s", {}},
    {"cv_tau_min_us", "Tau Min",           "Conveyor",       ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 10000000.0f, 10000.0f, "us", {}},
    {"cv_tau_max_us", "Tau Max",           "Conveyor",       ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 30000000.0f, 50000.0f, "us", {}},

    // ========================================================================
    // 7. Homing & Calibration (End-User)
    // ========================================================================
    {"cal_spd_mm_s",  "Search Speed",      "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 200.0f, 1.0f, "mm/s", {}},
    {"cal_th_spd_deg","Theta Search Spd",  "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 360.0f, 1.0f, "deg/s", {}},
    {"cal_acc_mm_s2", "Home/Cal Accel",    "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 50.0f, 10000.0f, 50.0f, "mm/s2", {}},
    {"cal_creep_mm_s","Creep Speed",       "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 0.1f, 20.0f, 0.1f, "mm/s", {}},
    {"cal_timeout_ms","Cal Timeout",       "Homing & Cal",   ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 120000.0f, 1000.0f, "ms", {}},
    {"trv_timeout_ms","Travel Timeout",    "Homing & Cal",   ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 300000.0f, 5000.0f, "ms", {}},

    // ========================================================================
    // 8. Network (LAN8720) (End-User)
    // ========================================================================
    {"eth_static",    "Use Static IP",     "Network",        ParamType::BOOL,   AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eth_ip",        "Static IP",         "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_gw",        "Static Gateway",    "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_netmask",   "Static Netmask",    "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_timeout_ms","Link Timeout",      "Network",        ParamType::INT,    AccessLevel::END_USER, true,  100.0f, 60000.0f, 500.0f, "ms", {}},

    // ========================================================================
    // 9. Display (End-User)
    // ========================================================================
    {"disp_orient",   "Orientation",       "Display",        ParamType::CHOICE, AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {"Landscape", "Portrait"}},
    {"disp_bright",   "Brightness",        "Display",        ParamType::INT,    AccessLevel::END_USER, false, 0.0f, 100.0f, 5.0f, "%", {}},
    {"disp_bl_to_s",  "Backlight Timeout", "Display",        ParamType::INT,    AccessLevel::END_USER, false, 0.0f, 3600.0f, 5.0f, "s", {}},

    // ========================================================================
    // 10. Console Security (Developer)
    // ========================================================================
    {"con_auth",      "TCP Auth Enable",   "Console Sec",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_pw",        "TCP Password",      "Console Sec",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"con_tries",     "Max Auth Tries",    "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 1.0f, 10.0f, 1.0f, "", {}},
    {"con_ttl_s",     "Remember TTL",      "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 0.0f, 86400.0f, 60.0f, "s", {}},
    {"con_max_peers", "Max Remembered",    "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 1.0f, 16.0f, 1.0f, "", {}},

    // ========================================================================
    // 11. Console Options (Developer)
    // ========================================================================
    {"con_tcp_en",    "TCP Console En",    "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_uart_en",   "UART Console En",   "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_port",      "TCP Listen Port",   "Console Opt",    ParamType::INT,    AccessLevel::DEVELOPER, true,  1.0f, 65535.0f, 1.0f, "", {}},
    {"con_log_en",    "TCP Log Stream",    "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, false, 0.0f, 1.0f, 1.0f, "", {}},
    {"con_log_lvl",   "Min Log Level",     "Console Opt",    ParamType::CHOICE, AccessLevel::DEVELOPER, false, 0.0f, 3.0f, 1.0f, "", {"ERR", "WARN", "INFO", "DBUG"}},
    {"ota_port",      "OTA Server Port",   "Console Opt",    ParamType::INT,    AccessLevel::DEVELOPER, true,  1.0f, 65535.0f, 1.0f, "", {}},

    // ========================================================================
    // 12. EtherNet/IP (Developer)
    // ========================================================================
    {"eip_en",        "EIP Scanner Enable","EtherNet/IP",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_x_en",      "X Axis EIP Enable", "EtherNet/IP",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_z_en",      "Z Axis EIP Enable", "EtherNet/IP",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_th_en",     "Theta EIP Enable",  "EtherNet/IP",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_ip_x",      "X Drive IP",        "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eip_ip_z",      "Z Drive IP",        "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eip_ip_th",     "Theta Drive IP",    "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"w5500_ip",      "W5500 Source IP",   "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"w5500_sub",     "W5500 Subnet Mask", "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"w5500_gw",      "W5500 Gateway",     "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"w5500_spi_hz",  "W5500 SPI Clock",   "EtherNet/IP",    ParamType::INT,    AccessLevel::DEVELOPER, true,  1000000.0f, 80000000.0f, 1000000.0f, "Hz", {}},
    {"eip_x_puu_mm",  "X PUU / mm",        "EtherNet/IP",    ParamType::FLOAT, AccessLevel::DEVELOPER, true,  1.0f, 1000000.0f, 1.0f, "", {}},
    {"eip_z_puu_mm",  "Z PUU / mm",        "EtherNet/IP",    ParamType::FLOAT, AccessLevel::DEVELOPER, true,  1.0f, 1000000.0f, 1.0f, "", {}},
    {"eip_th_puu_deg","Theta PUU / deg",   "EtherNet/IP",    ParamType::FLOAT, AccessLevel::DEVELOPER, true,  1.0f, 1000000.0f, 1.0f, "", {}},
    {"eip_x_rpi_us",  "X/Z RPI",           "EtherNet/IP",    ParamType::INT,    AccessLevel::DEVELOPER, true,  500.0f, 50000.0f, 500.0f, "us", {}},
    {"eip_th_rpi_us", "Theta RPI",         "EtherNet/IP",    ParamType::INT,    AccessLevel::DEVELOPER, true,  500.0f, 50000.0f, 500.0f, "us", {}},
    {"eip_endstop_src","Endstop Source",   "EtherNet/IP",    ParamType::CHOICE, AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {"Drive Inputs", "MCP23S17"}},
    {"th_abs_min_deg","Theta Abs Min",     "EtherNet/IP",    ParamType::FLOAT, AccessLevel::DEVELOPER, true,  -360.0f, 360.0f, 5.0f, "deg", {}},
    {"th_abs_max_deg","Theta Abs Max",     "EtherNet/IP",    ParamType::FLOAT, AccessLevel::DEVELOPER, true,  -360.0f, 360.0f, 5.0f, "deg", {}},
    {"hcs01_eng_ip",  "HCS01 Eng IP",      "EtherNet/IP",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
};

const std::vector<ParamSchema>& getParamSchemas() {
    return s_schemas;
}

const ParamSchema* findParamSchema(const char* key) {
    if (key == nullptr) return nullptr;
    for (const auto& s : s_schemas) {
        if (std::strcmp(s.key, key) == 0) {
            return &s;
        }
    }
    return nullptr;
}

} // namespace Config
