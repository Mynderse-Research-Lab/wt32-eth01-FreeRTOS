#include "ConfigSchema.h"
#include <cstring>

namespace Config {

static const std::vector<ParamSchema> s_schemas = {
    // ========================================================================
    // 1. Motion Profile (End-User)
    // ========================================================================
    {"spd_mm_s",      "Default Speed",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 3333.0f, 5.0f, "mm/s", {}},
    {"acc_mm_s2",     "Default Accel",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"dec_mm_s2",     "Default Decel",     "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 100.0f, 50000.0f, 100.0f, "mm/s2", {}},
    {"th_spd_deg_s",  "Theta Speed",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 7200.0f, 5.0f, "deg/s", {}},
    {"th_acc_deg_s2", "Theta Accel",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 18000.0f, 10.0f, "deg/s2", {}},
    {"th_dec_deg_s2", "Theta Decel",       "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 10.0f, 18000.0f, 10.0f, "deg/s2", {}},
    {"ceil_spd_mm_s", "Speed Ceiling",    "Motion Profile", ParamType::INT,   AccessLevel::END_USER, false, 1.0f, 3333.0f, 10.0f, "mm/s", {}},
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
    // 5. Geometry (End-User)
    // ========================================================================
    {"geom_z_y_mm",   "Z-Col Y Offset",    "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_th_x_mm",  "Theta X Offset",    "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_grp_x_mm", "Gripper TCP X",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_grp_z_mm", "Gripper TCP Z",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  -1000.0f, 1000.0f, 1.0f, "mm", {}},
    {"geom_safe_z_mm","Safe Z Margin",     "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, true,  1.0f, 150.0f, 1.0f, "mm", {}},
    {"geom_cal_x_mm", "Bin Drop X",        "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 500.0f, 1.0f, "mm", {}},
    {"geom_cal_z_mm", "Bin Drop Z",        "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 150.0f, 0.1f, "mm", {}},
    {"geom_cal_th_deg","Bin Drop Theta",   "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, -180.0f, 180.0f, 1.0f, "deg", {}},
    {"col_x_min_mm",  "Collision X Min",   "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 550.0f, 1.0f, "mm", {}},
    {"col_z_min_mm",  "Collision Z Min",   "Geometry",       ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 150.0f, 1.0f, "mm", {}},

    // ========================================================================
    // 6. Conveyor: belt and frame, in gantry coordinates (End-User)
    // ========================================================================
    {"cv_pick_y_mm",   "Pick Y Pos",         "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 5000.0f, 1.0f, "mm", {},
     "World Y of the jaw line (+Y downstream). A cell arrives when the belt has carried it from camera Y + y_bat_mm to here."},
    {"cv_belt_w_mm",   "Belt Width",         "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, 50.0f, 800.0f, 1.0f, "mm", {},
     "Belt width. 'beltcal apply' stores it from the two belt edges; the intercept does not read it."},
    {"cv_belt_near_mm", "Belt Near Edge X",   "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, -100.0f, 600.0f, 0.5f, "mm", {},
     "Joint X of the belt's near edge ('beltcal near'). 'beltcal apply' takes the width and centre from the two edges."},
    {"cv_belt_far_mm", "Belt Far Edge X",    "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, -100.0f, 600.0f, 0.5f, "mm", {},
     "Joint X of the belt's far edge ('beltcal far')."},
    {"cv_cnv_near_mm", "Conveyor Near X",    "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 600.0f, 0.5f, "mm", {},
     "Joint X of the conveyor frame's near edge ('beltcal cnvyr near'). 'beltcal apply' copies it to col_x_min_mm."},
    {"cv_cnv_far_mm",  "Conveyor Far X",     "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 600.0f, 0.5f, "mm", {},
     "Joint X of the conveyor frame's far edge ('beltcal cnvyr far'). Shown by 'beltcal'; no motion reads it."},
    {"cv_belt_spd",    "Default Belt Spd",   "Conveyor", ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 5000.0f, 10.0f, "mm/s", {},
     "Belt speed 'pickcover' uses when no conveyor frame is current."},

    // ========================================================================
    // 7. Vision: detections into gantry coordinates (End-User)
    // ========================================================================
    {"vis_cam_y_mm",   "Camera Y Pos",       "Vision",   ParamType::FLOAT, AccessLevel::END_USER, false, -5000.0f, 5000.0f, 1.0f, "mm", {},
     "World Y of the camera's optical centre (+Y downstream). A detection's world Y is this plus y_bat_mm. 'pickcal cam mark' sets it."},
    {"vis_x_off_mm",   "Camera X",           "Vision",   ParamType::FLOAT, AccessLevel::END_USER, false, -500.0f, 500.0f, 0.5f, "mm", {},
     "Gantry pose X of vision x = 0: pose X = x_across_mm + this. 'beltcal apply center' sets it to the belt centre."},
    {"vis_lat_ms",     "Vision Latency",     "Vision",   ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Exposure-to-receipt time. A detection without a valid capture displacement is placed on the belt at receipt minus this."},
    {"vis_dedupe_mm",  "Detect Dedupe",      "Vision",   ParamType::FLOAT, AccessLevel::END_USER, false, 5.0f, 200.0f, 1.0f, "mm", {},
     "Detections with the same item_id, or closer than this both along and across the belt, are one cell. Under 5 mm means 40."},
    {"vis_batt_x",     "Vision X = batt_x",  "Vision",   ParamType::INT, AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {},
     "1: each vision frame takes x_across from batt_x and y_bat from the trigger sensor position (fixed trigger-sensor frames)."},
    {"batt_x",         "Test Battery X",     "Vision",   ParamType::FLOAT, AccessLevel::END_USER, false, -100.0f, 600.0f, 0.5f, "mm", {},
     "Joint X of the test battery ('beltcal batt'). With vis_batt_x = 1 every vision frame takes its X from here."},

    // ========================================================================
    // 8. Gripper: jaws, fingers and valve (End-User)
    // ========================================================================
    {"jaw_off_deg",    "Jaw Angle Offset",   "Gripper",  ParamType::FLOAT, AccessLevel::END_USER, false, -180.0f, 180.0f, 1.0f, "deg", {},
     "Pad direction relative to the cell's long axis, added before the theta joint conversion. 0: pads along the long axis (side grip)."},
    {"jaw_aaa_ms",     "Jaw Contact AAA",    "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for AAA in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_aa_ms",      "Jaw Contact AA",     "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for AA in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_d_ms",       "Jaw Contact D",      "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for D in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_c_ms",       "Jaw Contact C",      "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for C in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_9v_ms",      "Jaw Contact 9V",     "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for 9V in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_cr123_ms",   "Jaw Contact CR123A", "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for CR123A in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_18650_ms",   "Jaw Contact 18650",  "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for 18650 in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_21700_ms",   "Jaw Contact 21700",  "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for 21700 in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_26650_ms",   "Jaw Contact 26650",  "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for 26650 in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"jaw_32650_ms",   "Jaw Contact 32650",  "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Jaw close time for 32650 in the strike timing. 0: side gap / fin_spd_mm_s."},
    {"grp_valve_ms",   "Grip Valve Delay",   "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 0.0f, 500.0f, 5.0f, "ms", {},
     "Valve-to-jaw-motion delay in the strike timing: the grip command leads contact by this plus the jaw close time."},
    {"grp_open_ms",    "Gripper Open Time",  "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 10.0f, 5000.0f, 10.0f, "ms", {},
     "Time the gripper takes to open. The bin drop waits this long, and the post-pick cycle estimate includes it."},
    {"grp_close_ms",   "Gripper Close Time", "Gripper",  ParamType::INT, AccessLevel::END_USER, false, 10.0f, 5000.0f, 10.0f, "ms", {},
     "Time the gripper takes to close. The post-pick cycle estimate and the console 'grip' wait use it."},
    {"fin_open_mm",    "Finger Opening",     "Gripper",  ParamType::FLOAT, AccessLevel::END_USER, false, 10.0f, 200.0f, 0.5f, "mm", {},
     "Gap between the open fingers. Wider detections are rejected; the side gap and lane clearance use it."},
    {"fin_wid_mm",     "Finger Width",       "Gripper",  ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 40.0f, 0.5f, "mm", {},
     "Finger size along the closing direction (the finger thickness; field pick_finger_thick_mm). Sets the lane clearance."},
    {"fin_len_mm",     "Finger Length",      "Gripper",  ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 120.0f, 0.5f, "mm", {},
     "Finger size across the closing direction (field pick_finger_width_mm). Sets the finger rectangles in the clearance test."},
    {"fin_spd_mm_s",   "Jaw Speed",          "Gripper",  ParamType::FLOAT, AccessLevel::END_USER, false, 10.0f, 2000.0f, 5.0f, "mm/s", {},
     "Speed of each jaw while closing (SCHUNK KGG 100-80: 40 mm in 0.19 s at 6 bar). Sets the close time when the class time is 0."},


    // ========================================================================
    // 9. Pick: strike timing and cycle (End-User)
    // ========================================================================
    {"pk_von_ms",      "Valve On",           "Pick",     ParamType::INT, AccessLevel::END_USER, false, 0.0f, 100.0f, 1.0f, "ms", {},
     "Valve delay the strike planner allows before the jaws move (strike_plan valve_delay_s)."},
    {"pk_z_dead_ms",   "Z Plunge Dead",      "Pick",     ParamType::INT, AccessLevel::END_USER, false, 0.0f, 100.0f, 1.0f, "ms", {},
     "Measured Kinetix Z start deadtime from StartMotion edge to plunge motion (strike_plan z_plunge_dead_s)."},
    {"pk_z_ret_ms",    "Z Retract Dead",     "Pick",     ParamType::INT, AccessLevel::END_USER, false, 0.0f, 100.0f, 1.0f, "ms", {},
     "Measured Kinetix Z retract deadtime from queued retract edge to upward reversal (strike_plan z_retract_dead_s)."},
    // Strike plan offsets (strike_plan.h). Each moves only its own event.
    {"pk_off_plunge",  "Z Plunge Offset",    "Pick",     ParamType::INT, AccessLevel::END_USER, false, -100.0f, 100.0f, 1.0f, "ms", {},
     "Moves the strike plan's Z plunge by this many ms; the other events stay where they are."},
    {"pk_off_close",   "Grip Close Offset",  "Pick",     ParamType::INT, AccessLevel::END_USER, false, -100.0f, 100.0f, 1.0f, "ms", {},
     "Moves the strike plan's grip close by this many ms; the other events stay where they are."},
    {"pk_off_retract", "Z Retract Offset",   "Pick",     ParamType::INT, AccessLevel::END_USER, false, -100.0f, 100.0f, 1.0f, "ms", {},
     "Moves the strike plan's Z retract by this many ms; the other events stay where they are."},
    {"pk_guard_ms",    "Timing Guard",       "Pick",     ParamType::INT, AccessLevel::END_USER, false, 0.0f, 200.0f, 1.0f, "ms", {},
     "Z must reach grasp depth this long before jaw contact; the late-Z window check adds it too."},
    {"pk_dropin",      "Drop-in Strike",     "Pick",     ParamType::BOOL, AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {},
     "1 allows late-Z strikes, where Z is released after the upstream finger clears the cell. 0 skips them ('late Z off')."},
    {"pk_drop_any_th", "Bin Any Theta",      "Pick",     ParamType::BOOL, AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {},
     "1 drops in the bin at the pick theta; 0 turns to geom_cal_th_deg first."},
    {"pk_stage_x_mm",  "Staging X",          "Pick",     ParamType::FLOAT, AccessLevel::END_USER, false, 0.0f, 550.0f, 1.0f, "mm", {},
     "Joint X where the gantry waits between picks, at SAFE_Z with theta 0."},

    // ========================================================================
    // 10. Homing & Calibration (End-User)
    // ========================================================================
    {"cal_spd_mm_s",  "Search Speed",      "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 200.0f, 1.0f, "mm/s", {}},
    {"cal_th_spd_deg","Theta Search Spd",  "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 1.0f, 360.0f, 1.0f, "deg/s", {}},
    {"cal_acc_mm_s2", "Home/Cal Accel",    "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 50.0f, 10000.0f, 50.0f, "mm/s2", {}},
    {"cal_creep_mm_s","Creep Speed",       "Homing & Cal",   ParamType::FLOAT, AccessLevel::END_USER, false, 0.1f, 20.0f, 0.1f, "mm/s", {}},
    {"cal_timeout_ms","Cal Timeout",       "Homing & Cal",   ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 120000.0f, 1000.0f, "ms", {}},
    {"trv_timeout_ms","Travel Timeout",    "Homing & Cal",   ParamType::INT,   AccessLevel::END_USER, false, 1000.0f, 300000.0f, 5000.0f, "ms", {}},

    // ========================================================================
    // 11. Network (LAN8720) (End-User)
    // ========================================================================
    {"eth_static",    "Use Static IP",     "Network",        ParamType::BOOL,   AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eth_ip",        "Static IP",         "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_gw",        "Static Gateway",    "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"ntp_server",    "NTP Server",        "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_netmask",   "Static Netmask",    "Network",        ParamType::STRING, AccessLevel::END_USER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"eth_timeout_ms","Link Timeout",      "Network",        ParamType::INT,    AccessLevel::END_USER, true,  100.0f, 60000.0f, 500.0f, "ms", {}},

    // ========================================================================
    // 12. Display (End-User)
    // ========================================================================
    {"disp_orient",   "Orientation",       "Display",        ParamType::CHOICE, AccessLevel::END_USER, false, 0.0f, 1.0f, 1.0f, "", {"Landscape", "Portrait"}},
    {"disp_bright",   "Brightness",        "Display",        ParamType::INT,    AccessLevel::END_USER, false, 0.0f, 100.0f, 5.0f, "%", {}},
    {"disp_bl_to_s",  "Backlight Timeout", "Display",        ParamType::INT,    AccessLevel::END_USER, false, 0.0f, 3600.0f, 5.0f, "s", {}},
    {"disp_tft_en",   "TFT UI Enabled",    "Display",        ParamType::INT,    AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},

    // ========================================================================
    // 13. Console Security (Developer)
    // ========================================================================
    {"con_auth",      "TCP Auth Enable",   "Console Sec",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_pw",        "TCP Password",      "Console Sec",    ParamType::STRING, AccessLevel::DEVELOPER, true,  0.0f, 0.0f, 0.0f, "", {}},
    {"con_tries",     "Max Auth Tries",    "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 1.0f, 10.0f, 1.0f, "", {}},
    {"con_ttl_s",     "Remember TTL",      "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 0.0f, 86400.0f, 60.0f, "s", {}},
    {"con_max_peers", "Max Remembered",    "Console Sec",    ParamType::INT,    AccessLevel::DEVELOPER, false, 1.0f, 16.0f, 1.0f, "", {}},

    // ========================================================================
    // 14. Console Options (Developer)
    // ========================================================================
    {"con_tcp_en",    "TCP Console En",    "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_uart_en",   "UART Console En",   "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"con_port",      "TCP Listen Port",   "Console Opt",    ParamType::INT,    AccessLevel::DEVELOPER, true,  1.0f, 65535.0f, 1.0f, "", {}},
    {"con_log_en",    "TCP Log Stream",    "Console Opt",    ParamType::BOOL,   AccessLevel::DEVELOPER, false, 0.0f, 1.0f, 1.0f, "", {}},
    {"con_log_lvl",   "Min Log Level",     "Console Opt",    ParamType::CHOICE, AccessLevel::DEVELOPER, false, 0.0f, 3.0f, 1.0f, "", {"ERR", "WARN", "INFO", "DBUG"}},
    {"ota_port",      "OTA Server Port",   "Console Opt",    ParamType::INT,    AccessLevel::DEVELOPER, true,  1.0f, 65535.0f, 1.0f, "", {}},

    // ========================================================================
    // 15. EtherNet/IP (Developer)
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
    {"eip_spi_khz",   "W5500 SPI Clock",   "EtherNet/IP",    ParamType::INT,    AccessLevel::END_USER, true,  1000.0f, 20000.0f, 1000.0f, "kHz", {}},
    {"eip_txwr_split","Split TX_WR Write", "EtherNet/IP",    ParamType::INT,    AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_spi_dma",   "W5500 SPI DMA",     "EtherNet/IP",    ParamType::INT,    AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"log_uart_async","Buffered Log UART", "Console Opt",    ParamType::INT,    AccessLevel::END_USER, true,  0.0f, 1.0f, 1.0f, "", {}},
    {"eip_scope_gpio","Class 1 Scope GPIO","EtherNet/IP",    ParamType::INT,    AccessLevel::END_USER, true,  -1.0f, 39.0f, 1.0f, "", {}},
};

// Stored sets and parameter files from before 2026-11-16 use the old names.
// AppConfig reads both and writes only the new one.
static const std::vector<RenamedParamKey> s_renamed = {
    {"cv_cam_y_mm", "vis_cam_y_mm"},
    {"cv_x_off_mm", "vis_x_off_mm"},
    {"cv_vis_ms", "vis_lat_ms"},
    {"cv_dedupe_mm", "vis_dedupe_mm"},
    {"cv_jaw_deg", "jaw_off_deg"},
    {"cv_valve_ms", "grp_valve_ms"},
    {"cv_jaw_aaa_ms", "jaw_aaa_ms"},
    {"cv_jaw_aa_ms", "jaw_aa_ms"},
    {"cv_jaw_d_ms", "jaw_d_ms"},
    {"cv_jaw_c_ms", "jaw_c_ms"},
    {"cv_jaw_9v_ms", "jaw_9v_ms"},
    {"cv_jaw_cr123_ms", "jaw_cr123_ms"},
    {"cv_jaw_18650_ms", "jaw_18650_ms"},
    {"cv_jaw_21700_ms", "jaw_21700_ms"},
    {"cv_jaw_26650_ms", "jaw_26650_ms"},
    {"cv_jaw_32650_ms", "jaw_32650_ms"},
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

const std::vector<RenamedParamKey>& getRenamedParamKeys() {
    return s_renamed;
}

const char* renamedParamKey(const char* old_key) {
    if (old_key == nullptr) return nullptr;
    for (const auto& r : s_renamed) {
        if (std::strcmp(r.old_key, old_key) == 0) return r.new_key;
    }
    return nullptr;
}

const char* legacyParamKey(const char* new_key) {
    if (new_key == nullptr) return nullptr;
    for (const auto& r : s_renamed) {
        if (std::strcmp(r.new_key, new_key) == 0) return r.old_key;
    }
    return nullptr;
}

} // namespace Config
