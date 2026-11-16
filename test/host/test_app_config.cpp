#include "unity.h"
#include "AppConfig.h"
#include "MemoryStorage.h"
#include "ConfigSchema.h"
#include "gantry_runtime_params.h"

#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <vector>

using namespace Config;

namespace {

// Survives AppConfig::init(), the way NVS survives a reboot.
class SharedStorage : public IConfigStorage {
public:
    explicit SharedStorage(MemoryStorage& mem) : mem_(mem) {}
    bool init() override { return true; }
    bool getInt32(const char* k, int32_t& v) override { return mem_.getInt32(k, v); }
    bool setInt32(const char* k, int32_t v) override {
        ++flash_writes;
        return mem_.setInt32(k, v);
    }
    bool getFloat(const char* k, float& v) override { return mem_.getFloat(k, v); }
    bool setFloat(const char* k, float v) override { return mem_.setFloat(k, v); }
    bool getBool(const char* k, bool& v) override { return mem_.getBool(k, v); }
    bool setBool(const char* k, bool v) override { return mem_.setBool(k, v); }
    bool getString(const char* k, char* b, size_t n) override { return mem_.getString(k, b, n); }
    bool setString(const char* k, const char* v) override { return mem_.setString(k, v); }
    bool getBlob(const char* k, void* b, size_t& n) override { return mem_.getBlob(k, b, n); }
    bool setBlob(const char* k, const void* d, size_t n) override {
        if (fail_blobs) {
            return false;
        }
        ++flash_writes;
        return mem_.setBlob(k, d, n);
    }
    bool eraseKey(const char* k) override { return mem_.eraseKey(k); }
    bool commit() override { return true; }

    static bool fail_blobs;
    static uint32_t flash_writes;

private:
    MemoryStorage& mem_;
};

bool SharedStorage::fail_blobs = false;
uint32_t SharedStorage::flash_writes = 0;
MemoryStorage* s_nvs = nullptr;

void powerOn() {
    delete s_nvs;
    s_nvs = new MemoryStorage();
    SharedStorage::fail_blobs = false;
}

// Boot without the confirmation that drops merged keys.
void bootOnly() {
    AppConfig::instance().init(std::make_unique<SharedStorage>(*s_nvs));
}

void boot() {
    bootOnly();
    AppConfig::instance().commitPending();
}

void set(const char* key, int32_t val) {
    bool reboot = false;
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt(key, val, reboot));
}

int speed() {
    return AppConfig::instance().data().gantry_default_speed_mm_per_s;
}

uint32_t activeSeq() {
    return AppConfig::instance().activeSet().seq;
}

}  // namespace

void setUp(void) {
    powerOn();
    boot();
    SharedStorage::flash_writes = 0;
    AppConfig::instance().lockDeveloperMode();
}

void tearDown(void) {}

static void test_default_values(void) {
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

static void test_set_get_int_with_bounds(void) {
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

    // Reject out of bounds (< 1 or > 2000)
    ok = AppConfig::instance().setParamInt("spd_mm_s", 0, reboot);
    TEST_ASSERT_FALSE(ok);
    ok = AppConfig::instance().setParamInt("spd_mm_s", 2500, reboot);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(120, AppConfig::instance().data().gantry_default_speed_mm_per_s);
}

static void test_set_get_float_reboot_flag(void) {
    bool reboot = false;
    // x_lead_mm has reboot_required = true
    bool ok = AppConfig::instance().setParamFloat("x_lead_mm", 250.0f, reboot);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(reboot);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.0f, AppConfig::instance().data().axis_x_lead_mm_per_rev);
}

static void test_set_get_bool_and_string(void) {
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

static void test_developer_mode_password_lifecycle(void) {
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

static void test_factory_reset(void) {
    bool reboot = false;
    AppConfig::instance().setParamInt("spd_mm_s", 300, reboot);
    AppConfig::instance().setParamString("eth_ip", "10.0.0.99", reboot);
    TEST_ASSERT_EQUAL_INT(300, AppConfig::instance().data().gantry_default_speed_mm_per_s);
    TEST_ASSERT_EQUAL_STRING("10.0.0.99", AppConfig::instance().data().eth_static_ip);

    TEST_ASSERT_TRUE(AppConfig::instance().factoryReset());
    TEST_ASSERT_EQUAL_INT(50, AppConfig::instance().data().gantry_default_speed_mm_per_s);
    TEST_ASSERT_EQUAL_STRING("192.168.1.100", AppConfig::instance().data().eth_static_ip);
}

static void test_drop_pose_and_text_apply(void) {
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
static void test_strike_offsets_apply_live_and_persist(void) {
    const auto& d = AppConfig::instance().data();
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(0, d.pick_offset_retract_ms);

    bool reboot = true;
    TEST_ASSERT_TRUE(AppConfig::instance().isParamVisible("pk_off_plunge"));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_plunge", "-12", reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_close", "7", reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().applyParamText("pk_off_retract", "100", reboot));
    TEST_ASSERT_EQUAL_INT32(-12, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);

    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_off_retract", "101", reboot));
    TEST_ASSERT_FALSE(AppConfig::instance().applyParamText("pk_off_close", "2.5", reboot));
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);

    char shown[16];
    TEST_ASSERT_TRUE(AppConfig::instance().formatParamValue("pk_off_plunge", shown, sizeof(shown)));
    TEST_ASSERT_EQUAL_STRING("-12", shown);

    // Saved values come back after a reload from the same storage.
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_TRUE(AppConfig::instance().load());
    TEST_ASSERT_EQUAL_INT32(-12, d.pick_offset_plunge_ms);
    TEST_ASSERT_EQUAL_INT32(7, d.pick_offset_close_ms);
    TEST_ASSERT_EQUAL_INT32(100, d.pick_offset_retract_ms);
}

static void test_first_boot_stores_a_full_set(void) {
    const ConfigSetInfo set = AppConfig::instance().activeSet();
    TEST_ASSERT_TRUE(set.valid);
    TEST_ASSERT_EQUAL_UINT32(1, set.seq);
    TEST_ASSERT_TRUE(set.param_count > 170);
}

// A set must fit a small NVS partition, and each key needs its own record.
static void test_set_fits_and_key_hashes_are_unique(void) {
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

static void test_change_stays_in_ram_until_saved(void) {
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

static void test_reboot_discards_unsaved_changes(void) {
    set("spd_mm_s", 750);
    boot();
    TEST_ASSERT_EQUAL_INT(50, speed());
    TEST_ASSERT_EQUAL_INT(0, AppConfig::instance().pendingChanges());
}

static void test_no_flash_write_without_a_real_change(void) {
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

static void test_save_stamps_the_set(void) {
    set("spd_mm_s", 321);
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_TRUE(AppConfig::instance().activeSet().unix_s > 1700000000u);
}

static void test_damaged_set_is_kept_until_an_operator_saves(void) {
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

static void test_older_keys_move_into_the_first_set(void) {
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

static void test_crash_before_confirmation_loses_nothing(void) {
    s_nvs->setInt32("spd_mm_s", 640);
    bootOnly();
    TEST_ASSERT_EQUAL_UINT32(2, activeSeq());
    boot();
    TEST_ASSERT_EQUAL_INT(640, speed());
    TEST_ASSERT_EQUAL_UINT32(2, activeSeq());
}

static void test_failed_save_keeps_the_changes_in_ram(void) {
    set("spd_mm_s", 640);
    SharedStorage::fail_blobs = true;
    TEST_ASSERT_FALSE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_INT(640, speed());
    TEST_ASSERT_EQUAL_INT(1, AppConfig::instance().pendingChanges());

    SharedStorage::fail_blobs = false;
    TEST_ASSERT_TRUE(AppConfig::instance().saveSet());
    TEST_ASSERT_EQUAL_INT(0, AppConfig::instance().pendingChanges());
}

static void test_factory_reset_only_replaces_the_set_when_saved(void) {
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

static void test_save_request_goes_through_the_handler(void) {
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

static void test_gantry_runtime_params_follow_the_stored_values(void) {
    bool reboot = false;
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("th_spd_max", 900.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt("z_crit_rpm", 2700, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("z_lead_mm", 10.0f, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt("grp_close_ms", 210, reboot));
    TEST_ASSERT_TRUE(AppConfig::instance().setParamFloat("cal_spd_mm_s", 50.0f, reboot));

    const Gantry::RuntimeParams p = gantryRuntimeParams(AppConfig::instance().data());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 900.0f, p.theta_max_speed_deg_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 450.0f, p.z_speed_cap_mm_s);
    TEST_ASSERT_EQUAL_UINT32(210, p.gripper_close_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, p.home_speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 550.0f, p.x_hard_span_mm);
}

// The parameter file reloads through applyParamText, so each exported value
// has to be accepted back.
static void test_every_value_reloads_from_its_text(void) {
    TEST_ASSERT_TRUE(AppConfig::instance().unlockDeveloperMode("DEV_2026"));
    for (const auto& schema : getParamSchemas()) {
        char text[40];
        TEST_ASSERT_TRUE_MESSAGE(AppConfig::instance().formatParamValue(schema.key, text, sizeof(text)),
                                 schema.key);
        // Host defaults can sit outside a schema range; the gantry rejects those.
        const bool numeric = schema.type == ParamType::INT || schema.type == ParamType::FLOAT;
        if (text[0] == '\0' ||
            (numeric && (std::atof(text) < schema.min_val || std::atof(text) > schema.max_val))) {
            continue;
        }
        bool reboot = false;
        TEST_ASSERT_TRUE_MESSAGE(AppConfig::instance().applyParamText(schema.key, text, reboot),
                                 schema.key);
    }
}

static void test_schema_lookup(void) {
    const auto& schemas = getParamSchemas();
    TEST_ASSERT_TRUE(schemas.size() > 50);

    const ParamSchema* drop_z = findParamSchema("geom_cal_z_mm");
    TEST_ASSERT_NOT_NULL(drop_z);
    TEST_ASSERT_EQUAL_STRING("Bin Drop Z", drop_z->label);
    TEST_ASSERT_EQUAL(AccessLevel::END_USER, drop_z->access);
    TEST_ASSERT_FALSE(drop_z->reboot_required);

    const ParamSchema* drop_th = findParamSchema("geom_cal_th_deg");
    TEST_ASSERT_NOT_NULL(drop_th);
    TEST_ASSERT_EQUAL_STRING("Bin Drop Theta", drop_th->label);

    const ParamSchema* s = findParamSchema("col_x_min_mm");
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL_STRING("Collision X Min", s->label);
    TEST_ASSERT_EQUAL(AccessLevel::END_USER, s->access);

    const ParamSchema* s_eip = findParamSchema("w5500_ip");
    TEST_ASSERT_NOT_NULL(s_eip);
    TEST_ASSERT_EQUAL(AccessLevel::DEVELOPER, s_eip->access);
    TEST_ASSERT_TRUE(s_eip->reboot_required);
}

// A set written by firmware from before the 2026-11-16 rename.
static void storeSetWithOldNames(void) {
    ConfigSetWriter writer;
    writer.setFloat("cv_jaw_deg", 12.5f);
    writer.setFloat("cv_cam_y_mm", -650.0f);
    writer.setInt32("cv_jaw_9v_ms", 55);
    writer.setInt32("spd_mm_s", 75);
    const std::vector<uint8_t>& payload = writer.payload();
    ConfigSetHeader header;
    header.seq = 7;
    header.payload_len = static_cast<uint16_t>(payload.size());
    header.payload_hash = configHash(payload.data(), payload.size());
    header.param_count = writer.count();
    std::vector<uint8_t> blob(sizeof(header) + payload.size());
    std::memcpy(blob.data(), &header, sizeof(header));
    std::memcpy(blob.data() + sizeof(header), payload.data(), payload.size());
    TEST_ASSERT_TRUE(s_nvs->setBlob("cfg_set", blob.data(), blob.size()));
}

static void test_renamed_keys_keep_their_stored_values(void) {
    powerOn();
    storeSetWithOldNames();
    boot();
    const AppConfigData& d = AppConfig::instance().data();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.5f, d.pick_jaw_offset_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -650.0f, d.conveyor_y_cam_mm);
    TEST_ASSERT_EQUAL_INT32(55, d.pick_jaw_ms[4]);
    TEST_ASSERT_EQUAL_INT(75, speed());

    // Boot rewrites the set under the new names only.
    size_t len = kConfigSetMaxBytes;
    std::vector<uint8_t> blob(len);
    TEST_ASSERT_TRUE(s_nvs->getBlob("cfg_set", blob.data(), len));
    ConfigSetHeader header;
    TEST_ASSERT_TRUE(parseConfigSet(blob.data(), len, header));
    ConfigSetReader reader(blob.data() + sizeof(header), header.payload_len);
    float value = 0.0f;
    TEST_ASSERT_TRUE(reader.getFloat("jaw_off_deg", value));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.5f, value);
    TEST_ASSERT_FALSE(reader.getFloat("cv_jaw_deg", value));

    boot();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.5f, AppConfig::instance().data().pick_jaw_offset_deg);
}

static void test_old_per_key_values_move_to_the_new_names(void) {
    powerOn();
    s_nvs->setFloat("cv_x_off_mm", 307.5f);
    bootOnly();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 307.5f,
                             AppConfig::instance().data().conveyor_x_across_to_gantry_x_offset_mm);
    AppConfig::instance().commitPending();
    float raw = 0.0f;
    TEST_ASSERT_FALSE(s_nvs->getFloat("cv_x_off_mm", raw));
    boot();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 307.5f,
                             AppConfig::instance().data().conveyor_x_across_to_gantry_x_offset_mm);
}

static void test_rename_table_points_at_live_keys(void) {
    std::set<uint32_t> hashes;
    for (const auto& schema : getParamSchemas()) {
        hashes.insert(configKeyHash(schema.key));
    }
    for (const auto& r : getRenamedParamKeys()) {
        TEST_ASSERT_NULL_MESSAGE(findParamSchema(r.old_key), r.old_key);
        TEST_ASSERT_NOT_NULL_MESSAGE(findParamSchema(r.new_key), r.new_key);
        TEST_ASSERT_EQUAL_STRING(r.new_key, renamedParamKey(r.old_key));
        TEST_ASSERT_EQUAL_STRING(r.old_key, legacyParamKey(r.new_key));
        // An old name must not read another parameter's record.
        TEST_ASSERT_TRUE_MESSAGE(hashes.insert(configKeyHash(r.old_key)).second, r.old_key);
    }
    TEST_ASSERT_NULL(renamedParamKey("spd_mm_s"));
}

static void test_pick_sections_are_described(void) {
    const char* sections[] = {"Conveyor", "Vision", "Gripper", "Cells", "Pick"};
    int described = 0;
    for (const auto& schema : getParamSchemas()) {
        TEST_ASSERT_TRUE_MESSAGE(std::strlen(schema.key) <= 15, schema.key);
        for (const char* section : sections) {
            if (std::strcmp(schema.section, section) == 0) {
                TEST_ASSERT_NOT_NULL_MESSAGE(schema.description, schema.key);
                TEST_ASSERT_TRUE_MESSAGE(std::strlen(schema.description) > 10, schema.key);
                ++described;
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(69, described);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_default_values);
    RUN_TEST(test_set_get_int_with_bounds);
    RUN_TEST(test_set_get_float_reboot_flag);
    RUN_TEST(test_set_get_bool_and_string);
    RUN_TEST(test_developer_mode_password_lifecycle);
    RUN_TEST(test_factory_reset);
    RUN_TEST(test_drop_pose_and_text_apply);
    RUN_TEST(test_strike_offsets_apply_live_and_persist);
    RUN_TEST(test_first_boot_stores_a_full_set);
    RUN_TEST(test_set_fits_and_key_hashes_are_unique);
    RUN_TEST(test_change_stays_in_ram_until_saved);
    RUN_TEST(test_reboot_discards_unsaved_changes);
    RUN_TEST(test_no_flash_write_without_a_real_change);
    RUN_TEST(test_save_stamps_the_set);
    RUN_TEST(test_damaged_set_is_kept_until_an_operator_saves);
    RUN_TEST(test_older_keys_move_into_the_first_set);
    RUN_TEST(test_crash_before_confirmation_loses_nothing);
    RUN_TEST(test_failed_save_keeps_the_changes_in_ram);
    RUN_TEST(test_factory_reset_only_replaces_the_set_when_saved);
    RUN_TEST(test_save_request_goes_through_the_handler);
    RUN_TEST(test_gantry_runtime_params_follow_the_stored_values);
    RUN_TEST(test_every_value_reloads_from_its_text);
    RUN_TEST(test_schema_lookup);
    RUN_TEST(test_renamed_keys_keep_their_stored_values);
    RUN_TEST(test_old_per_key_values_move_to_the_new_names);
    RUN_TEST(test_rename_table_points_at_live_keys);
    RUN_TEST(test_pick_sections_are_described);
    return UNITY_END();
}
