/**
 * @file test_app_config_store.cpp
 * @brief AppConfig persistence host tests.
 */

#include "test_app_config_internal.h"

using namespace app_config_test;

// A set must fit a small NVS partition, and each key needs its own record.
void test_set_fits_and_key_hashes_are_unique(void) {
    size_t len = kConfigSetMaxBytes;
    std::vector<uint8_t> blob(len);
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", blob.data(), len));
    TEST_ASSERT_TRUE(len < 1900);

    ConfigSetHeader header;
    TEST_ASSERT_TRUE(parseConfigSet(blob.data(), len, header));
    ConfigSetReader reader(blob.data() + sizeof(header), header.payload_len);
    int32_t value = 0;
    TEST_ASSERT_TRUE(reader.getInt32("spd_mm_s", value));
    TEST_ASSERT_EQUAL_INT32(50, value);

    std::set<uint32_t> seen;
    size_t schema_keys = 0;
    for (const auto& schema : getParamSchemas()) {
        seen.insert(configKeyHash(schema.key));
        ++schema_keys;
    }
    TEST_ASSERT_EQUAL_size_t(schema_keys, seen.size());
}

void test_change_stays_in_ram_until_saved(void) {
    set("spd_mm_s", 750);
    TEST_ASSERT_EQUAL_INT(750, speed());
    TEST_ASSERT_EQUAL_INT(1, AppConfig::instance().pendingChanges());
    TEST_ASSERT_EQUAL_UINT32(0, SharedStorage::flash_writes);

    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_INT(0, AppConfig::instance().pendingChanges());
    TEST_ASSERT_EQUAL_UINT32(2, activeSeq());
    TEST_ASSERT_EQUAL_UINT32(1, SharedStorage::flash_writes);
    boot();
    TEST_ASSERT_EQUAL_INT(750, speed());
}

void test_reboot_discards_unsaved_changes(void) {
    set("spd_mm_s", 750);
    boot();
    TEST_ASSERT_EQUAL_INT(50, speed());
    TEST_ASSERT_EQUAL_INT(0, AppConfig::instance().pendingChanges());
}

void test_no_flash_write_without_a_real_change(void) {
    boot();
    boot();
    TEST_ASSERT_EQUAL_UINT32(1, activeSeq());

    const int before = speed();
    set("spd_mm_s", before + 10);
    set("spd_mm_s", before);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_UINT32(1, activeSeq());
    TEST_ASSERT_EQUAL_UINT32(0, SharedStorage::flash_writes);
}

void test_save_stamps_the_set(void) {
    set("spd_mm_s", 321);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_TRUE(AppConfig::instance().activeSet().unix_s > 1700000000u);
}

void test_damaged_set_is_kept_until_an_operator_saves(void) {
    set("spd_mm_s", 111);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    size_t len = kConfigSetMaxBytes;
    std::vector<uint8_t> blob(len);
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", blob.data(), len));
    blob[len - 1] ^= 0xFF;
    TEST_ASSERT_TRUE(s_nvs->setBlob("cfg_set", blob.data(), len));

    boot();
    TEST_ASSERT_TRUE(AppConfig::instance().activeSet().damaged);
    TEST_ASSERT_EQUAL_INT(50, speed());
    std::vector<uint8_t> after(kConfigSetMaxBytes);
    size_t after_len = after.size();
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", after.data(), after_len));
    TEST_ASSERT_EQUAL_MEMORY(blob.data(), after.data(), len);

    set("spd_mm_s", 222);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    boot();
    TEST_ASSERT_FALSE(AppConfig::instance().activeSet().damaged);
    TEST_ASSERT_EQUAL_INT(222, speed());
}

// G-F7: a read error is not an empty store. The set is kept as damaged, the
// machine runs on defaults, and nothing is written until an operator saves.
void test_unreadable_set_is_kept_not_overwritten(void) {
    set("spd_mm_s", 111);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    size_t len = kConfigSetMaxBytes;
    std::vector<uint8_t> before(len);
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", before.data(), len));

    SharedStorage::fail_blob_reads = true;
    const uint32_t writes = SharedStorage::flash_writes;
    boot();
    TEST_ASSERT_TRUE(AppConfig::instance().activeSet().damaged);
    TEST_ASSERT_EQUAL_INT(50, speed());
    TEST_ASSERT_EQUAL_UINT32(writes, SharedStorage::flash_writes);

    SharedStorage::fail_blob_reads = false;
    std::vector<uint8_t> after(kConfigSetMaxBytes);
    size_t after_len = after.size();
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", after.data(), after_len));
    TEST_ASSERT_EQUAL_UINT32(len, after_len);
    TEST_ASSERT_EQUAL_MEMORY(before.data(), after.data(), len);

    boot();
    TEST_ASSERT_FALSE(AppConfig::instance().activeSet().damaged);
    TEST_ASSERT_EQUAL_INT(111, speed());
}

void test_older_keys_move_into_the_first_set(void) {
    powerOn();
    s_nvs->setInt32("spd_mm_s", 200);
    s_nvs->setFloat("geom_safe_z_mm", 48.4f);
    s_nvs->setInt32("cfg_chg_unix", 1750000000);
    bootOnly();
    TEST_ASSERT_EQUAL_INT(200, speed());
    TEST_ASSERT_EQUAL_UINT32(1, activeSeq());
    TEST_ASSERT_EQUAL_UINT32(1750000000u, AppConfig::instance().activeSet().unix_s);

    // Still there for a rollback until the boot is confirmed.
    int32_t raw = 0;
    TEST_ASSERT_TRUE(s_nvs->getInt32("spd_mm_s", raw));
    AppConfig::instance().commitPending();
    TEST_ASSERT_FALSE(s_nvs->getInt32("spd_mm_s", raw));
    TEST_ASSERT_FALSE(s_nvs->getInt32("cfg_chg_unix", raw));

    boot();
    TEST_ASSERT_EQUAL_INT(200, speed());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 48.4f, AppConfig::instance().data().gantry_safe_z_height_mm);
}

void test_crash_before_confirmation_loses_nothing(void) {
    s_nvs->setInt32("spd_mm_s", 640);
    bootOnly();
    TEST_ASSERT_EQUAL_UINT32(2, activeSeq());
    boot();
    TEST_ASSERT_EQUAL_INT(640, speed());
    TEST_ASSERT_EQUAL_UINT32(2, activeSeq());
}

void test_failed_save_keeps_the_changes_in_ram(void) {
    set("spd_mm_s", 640);
    SharedStorage::fail_blobs = true;
    TEST_ASSERT_FALSE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_INT(640, speed());
    TEST_ASSERT_EQUAL_INT(1, AppConfig::instance().pendingChanges());

    SharedStorage::fail_blobs = false;
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_INT(0, AppConfig::instance().pendingChanges());
}

void test_factory_reset_only_replaces_the_set_when_saved(void) {
    set("spd_mm_s", 750);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());

    TEST_ASSERT_TRUE(AppConfig::instance().factoryReset());
    TEST_ASSERT_EQUAL_INT(50, speed());
    TEST_ASSERT_EQUAL_INT(1, AppConfig::instance().pendingChanges());
    boot();
    TEST_ASSERT_EQUAL_INT(750, speed());

    TEST_ASSERT_TRUE(AppConfig::instance().factoryReset());
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    boot();
    TEST_ASSERT_EQUAL_INT(50, speed());
}

void test_save_request_goes_through_the_handler(void) {
    TEST_ASSERT_FALSE(AppConfig::instance().requestSave());
    int calls = 0;
    AppConfig::instance().setSaveHandler([&calls]() {
        ++calls;
        return true;
    });
    TEST_ASSERT_TRUE(AppConfig::instance().requestSave());
    TEST_ASSERT_EQUAL_INT(1, calls);
    AppConfig::instance().setSaveHandler(nullptr);
}

void test_gantry_runtime_params_follow_the_stored_values(void) {
    bool reboot = false;
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("th_spd_max", 900.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt("z_crit_rpm", 2700, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("z_lead_mm", 10.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt("grp_close_ms", 210, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("cal_spd_mm_s", 50.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("z_acc_mm_s2", 5500.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("z_dec_mm_s2", 5400.0f, reboot));

    const Gantry::RuntimeParams p = gantryRuntimeParams(AppConfig::instance().data());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 900.0f, p.theta_max_speed_deg_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 450.0f, p.z_speed_cap_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5500.0f, p.z_accel_cap_mm_s2);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5400.0f, p.z_decel_cap_mm_s2);
    TEST_ASSERT_EQUAL_UINT32(210, p.gripper_close_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, p.home_speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 550.0f, p.x_hard_span_mm);
}
