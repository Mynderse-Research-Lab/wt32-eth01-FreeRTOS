/**
 * @file test_app_config_values.cpp
 * @brief AppConfig value and bounds host tests.
 */

#include "test_app_config_internal.h"

using namespace app_config_test;

void test_default_values(void) {
    const auto& d = AppConfig::instance().data();
    TEST_ASSERT_EQUAL_INT(50, d.gantry_default_speed_mm_per_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f, d.axis_x_lead_mm_per_rev);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 550.0f, d.axis_x_hard_limit_max_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, d.axis_z_lead_mm_per_rev);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.0f, d.axis_z_hard_limit_max_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -180.0f, d.axis_theta_hard_limit_min_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 180.0f, d.axis_theta_hard_limit_max_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 95.0f, d.gantry_conveyor_collision_x_min_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 245.0f, d.conveyor_x_across_to_gantry_x_offset_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f, d.conveyor_belt_width_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 225.0f, d.conveyor_belt_near_x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 500.0f, d.conveyor_belt_far_x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 183.0f, d.conveyor_frame_near_x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 500.0f, d.conveyor_frame_far_x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 382.5f, d.test_battery_x_mm);
    TEST_ASSERT_EQUAL_INT32(0, d.vision_x_from_batt);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 115.0f, d.gantry_conveyor_collision_z_min_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, d.gantry_cal_x_park_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.8f, d.gantry_cal_z_park_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, d.gantry_cal_theta_park_deg);
    TEST_ASSERT_EQUAL_STRING("192.168.1.100", d.eth_static_ip);
    TEST_ASSERT_EQUAL_STRING("DEV_2026", d.dev_mode_password);
}

void test_set_get_int_with_bounds(void) {
    bool reboot = false;
    // Valid speed change
    bool ok = AppConfig::instance().setParamInt("spd_mm_s", 120, reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL_INT(120, AppConfig::instance().data().gantry_default_speed_mm_per_s);

    int32_t val = 0;
    ok = AppConfig::instance().getParamInt("spd_mm_s", val);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(120, val);

    // Accept 90% X physical speed (3000) and reject out of bounds (< 1 or > 3333)
    ok = AppConfig::instance().setParamInt("spd_mm_s", 3000, reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(3000, AppConfig::instance().data().gantry_default_speed_mm_per_s);
    ok = AppConfig::instance().setParamInt("spd_mm_s", 120, reboot);
    TEST_ASSERT_TRUE(ok);
    ok = AppConfig::instance().setParamInt("spd_mm_s", 0, reboot);
    TEST_ASSERT_FALSE(ok);
    ok = AppConfig::instance().setParamInt("spd_mm_s", 3500, reboot);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(120, AppConfig::instance().data().gantry_default_speed_mm_per_s);
}

void test_set_get_float_reboot_flag(void) {
    bool reboot = false;
    // x_lead_mm has reboot_required = true
    bool ok = AppConfig::instance().setParamFloat("x_lead_mm", 250.0f, reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(reboot);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.0f, AppConfig::instance().data().axis_x_lead_mm_per_rev);
}

void test_set_get_bool_and_string(void) {
    bool reboot = false;
    bool ok = AppConfig::instance().setParamBool("z_has_brake", true, reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(reboot);
    TEST_ASSERT_TRUE(AppConfig::instance().data().axis_z_has_motor_brake);

    ok = AppConfig::instance().setParamString("eth_ip", "192.168.1.150", reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(reboot);
    TEST_ASSERT_EQUAL_STRING("192.168.1.150", AppConfig::instance().data().eth_static_ip);
}

void test_developer_mode_password_lifecycle(void) {
    TEST_ASSERT_FALSE(AppConfig::instance().isDeveloperModeUnlocked());

    // Wrong password fails
    TEST_ASSERT_FALSE(AppConfig::instance().unlockDeveloperMode("WRONG_PW"));
    TEST_ASSERT_FALSE(AppConfig::instance().isDeveloperModeUnlocked());

    // Correct password succeeds
    TEST_ASSERT_TRUE(AppConfig::instance().unlockDeveloperMode("DEV_2026"));
    TEST_ASSERT_TRUE(AppConfig::instance().isDeveloperModeUnlocked());

    AppConfig::instance().lockDeveloperMode();
    TEST_ASSERT_FALSE(AppConfig::instance().isDeveloperModeUnlocked());

    // Change password with wrong old password fails
    TEST_ASSERT_FALSE(AppConfig::instance().changeDeveloperPassword("BAD_OLD", "NEW_2027"));

    // Change password with correct old password succeeds
    TEST_ASSERT_TRUE(AppConfig::instance().changeDeveloperPassword("DEV_2026", "NEW_2027"));

    // Old password no longer works
    TEST_ASSERT_FALSE(AppConfig::instance().unlockDeveloperMode("DEV_2026"));
    // New password works
    TEST_ASSERT_TRUE(AppConfig::instance().unlockDeveloperMode("NEW_2027"));
}

void test_factory_reset(void) {
    bool reboot = false;
    AppConfig::instance().setParamInt("spd_mm_s", 300, reboot);
    AppConfig::instance().setParamString("eth_ip", "10.0.0.99", reboot);
    TEST_ASSERT_EQUAL_INT(300, AppConfig::instance().data().gantry_default_speed_mm_per_s);
    TEST_ASSERT_EQUAL_STRING("10.0.0.99", AppConfig::instance().data().eth_static_ip);

    TEST_ASSERT_TRUE(AppConfig::instance().factoryReset());
    TEST_ASSERT_EQUAL_INT(50, AppConfig::instance().data().gantry_default_speed_mm_per_s);
    TEST_ASSERT_EQUAL_STRING("192.168.1.100", AppConfig::instance().data().eth_static_ip);
}

void test_drop_pose_and_text_apply(void) {
    bool reboot = false;
    bool ok = AppConfig::instance().applyParamText("geom_cal_z_mm", "91.5", reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 91.5f, AppConfig::instance().data().gantry_cal_z_park_mm);

    ok = AppConfig::instance().applyParamText("geom_cal_z_mm", "200", reboot);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 91.5f, AppConfig::instance().data().gantry_cal_z_park_mm);

    ok = AppConfig::instance().applyParamText("geom_cal_th_deg", "90", reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, AppConfig::instance().data().gantry_cal_theta_park_deg);

    ok = AppConfig::instance().applyParamText("disp_orient", "portrait", reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(1, AppConfig::instance().data().display_orientation);

    char shown[32];
    TEST_ASSERT_TRUE(AppConfig::instance().formatParamValue("geom_cal_z_mm", shown, sizeof(shown)));
    TEST_ASSERT_EQUAL_STRING("91.50", shown);

    TEST_ASSERT_FALSE(AppConfig::instance().isParamVisible("eip_ip_x"));
    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("eip_ip_x", "192.168.1.20", reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().unlockDeveloperMode(
        AppConfig::instance().data().dev_mode_password));
    TEST_ASSERT_TRUE(AppConfig::instance().isParamVisible("eip_ip_x"));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("z_has_brake", "on", reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().data().axis_z_has_motor_brake);
}

// Strike plan offsets: end-user, live (no reboot), whole ms within +/- 100.
void test_strike_offsets_apply_live_and_persist(void) {
    const auto& d = AppConfig::instance().data();
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_retract_ms);
    TEST_ASSERT_EQUAL_INT32(30, d.pick_z_plunge_dead_ms);
    TEST_ASSERT_EQUAL_INT32(22, d.pick_z_retract_dead_ms);

    bool reboot = true;
    TEST_ASSERT_TRUE(AppConfig::instance().isParamVisible("pk_off_plunge"));
    TEST_ASSERT_TRUE(AppConfig::instance().isParamVisible("pk_z_dead_ms"));
    TEST_ASSERT_TRUE(AppConfig::instance().isParamVisible("pk_z_ret_ms"));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_plunge", "-12", reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_close", "7", reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_retract", "100", reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_z_dead_ms", "31", reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_z_ret_ms", "25", reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL_INT32(-12, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);
    TEST_ASSERT_EQUAL_INT32(31, d.pick_z_plunge_dead_ms);
    TEST_ASSERT_EQUAL_INT32(25, d.pick_z_retract_dead_ms);

    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_off_retract", "101", reboot));
    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_off_close", "2.5", reboot));
    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_z_dead_ms", "-1", reboot));
    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_z_ret_ms", "101", reboot));
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(31, d.pick_z_plunge_dead_ms);
    TEST_ASSERT_EQUAL_INT32(25, d.pick_z_retract_dead_ms);

    char shown[16];
    TEST_ASSERT_TRUE(AppConfig::instance().formatParamValue("pk_off_plunge", shown, sizeof(shown)));
    TEST_ASSERT_EQUAL_STRING("-12", shown);
    TEST_ASSERT_TRUE(AppConfig::instance().formatParamValue("pk_z_dead_ms", shown, sizeof(shown)));
    TEST_ASSERT_EQUAL_STRING("31", shown);

    // Saved values come back after a reload from the same storage.
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_TRUE(AppConfig::instance().load());
    TEST_ASSERT_EQUAL_INT32(-12, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);
    TEST_ASSERT_EQUAL_INT32(31, d.pick_z_plunge_dead_ms);
    TEST_ASSERT_EQUAL_INT32(25, d.pick_z_retract_dead_ms);
}

void test_first_boot_stores_a_full_set(void) {
    const ConfigSetInfo set = AppConfig::instance().activeSet();
    TEST_ASSERT_TRUE(set.valid);
    TEST_ASSERT_EQUAL_UINT32(1, set.seq);
    TEST_ASSERT_TRUE(set.param_count >= getParamSchemas().size());
}
