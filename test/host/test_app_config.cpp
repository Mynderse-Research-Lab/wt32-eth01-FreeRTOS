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

#include "test_app_config_internal.h"
using namespace app_config_test;

void setUp(void) {
    powerOn();
    boot();
    SharedStorage::flash_writes = 0;
    AppConfig::instance().lockDeveloperMode();
}

void tearDown(void) {}

void test_default_values(void);
void test_set_get_int_with_bounds(void);
void test_set_get_float_reboot_flag(void);
void test_set_get_bool_and_string(void);
void test_developer_mode_password_lifecycle(void);
void test_factory_reset(void);
void test_drop_pose_and_text_apply(void);
void test_strike_offsets_apply_live_and_persist(void);
void test_first_boot_stores_a_full_set(void);
void test_set_fits_and_key_hashes_are_unique(void);
void test_change_stays_in_ram_until_saved(void);
void test_reboot_discards_unsaved_changes(void);
void test_no_flash_write_without_a_real_change(void);
void test_save_stamps_the_set(void);
void test_damaged_set_is_kept_until_an_operator_saves(void);
void test_unreadable_set_is_kept_not_overwritten(void);
void test_older_keys_move_into_the_first_set(void);
void test_crash_before_confirmation_loses_nothing(void);
void test_failed_save_keeps_the_changes_in_ram(void);
void test_factory_reset_only_replaces_the_set_when_saved(void);
void test_save_request_goes_through_the_handler(void);
void test_gantry_runtime_params_follow_the_stored_values(void);
void test_every_value_reloads_from_its_text(void);
void test_schema_lookup(void);
void test_renamed_keys_keep_their_stored_values(void);
void test_old_per_key_values_move_to_the_new_names(void);
void test_rename_table_points_at_live_keys(void);
void test_pick_sections_are_described(void);

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
    RUN_TEST(test_unreadable_set_is_kept_not_overwritten);
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
