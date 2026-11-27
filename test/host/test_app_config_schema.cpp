/**
 * @file test_app_config_schema.cpp
 * @brief AppConfig schema and rename host tests.
 */

#include "test_app_config_internal.h"

using namespace app_config_test;

// The parameter file reloads through applyParamText, so each exported value
// has to be accepted back.
void test_every_value_reloads_from_its_text(void) {
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

void test_schema_lookup(void) {
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

void test_renamed_keys_keep_their_stored_values(void) {
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

void test_old_per_key_values_move_to_the_new_names(void) {
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

void test_rename_table_points_at_live_keys(void) {
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

void test_pick_sections_are_described(void) {
    const char* sections[] = {"Conveyor", "Vision", "Gripper", "Pick"};
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
    TEST_ASSERT_EQUAL_INT(41, described);
}
