/**
 * @file test_gantry_orchestration.cpp
 * @brief Host tests for Gantry path SM, SAFE_Z interlocks, and theta window.
 */

#include "unity.h"

#ifndef CONFIG_GANTRY_THETA_SEQUENTIAL
#define CONFIG_GANTRY_THETA_SEQUENTIAL 0
#endif

#include "FakeAxes.h"
#include "Gantry.h"
#include "GantryPathProfile.h"
#include "axis_drivetrain_params.h"
#include "gpio_expander.h"
#include "host_test_clock.h"
#include "pick_intercept.h"

#include <memory>

using Gantry::DrivePositionRef;
using Gantry::EndEffectorPose;
using Gantry::GantryError;
using Gantry::JointConfig;
using Gantry::Test::FakeLinearAxis;
using Gantry::Test::FakeRotaryAxis;

#include "test_gantry_orchestration_internal.h"
using namespace gantry_orch_test;

void setUp(void) { HostTest::resetClock(); }
void tearDown(void) {}

void test_move_not_initialized(void);
void test_move_not_enabled(void);
void test_move_not_calibrated(void);
void test_workspace_cal_clears_on_drive_lost(void);
void test_workspace_cal_clears_on_theta_lost(void);
void test_drive_fault_stop_does_not_rezero_x(void);
void test_move_invalid_position(void);
void test_orchestrator_bubbles_up_axis_rejection(void);
void test_move_already_moving(void);
void test_theta_refused_no_in_band_segment(void);
void test_theta_only_while_deep_refused(void);
void test_theta_only_to_joint_limits_in_band(void);
void test_theta_only_return_to_zero_stays_busy(void);
void test_theta_does_not_start_before_25_percent(void);
void test_theta_starts_at_25_percent_window_speed(void);
void test_descent_held_until_theta_idle(void);
void test_stop_clears_pending_theta(void);
void test_retract_starts_theta_after_band_entry(void);
void test_theta_start_fraction_zero_starts_with_x(void);
void test_theta_start_fraction_caps_at_move_theta_speed(void);
void test_pick_theta_commands_a_tenth_while_waiting(void);
void test_pick_theta_truncates_below_a_tenth(void);
void test_pick_theta_quantizes_to_a_tenth(void);
void test_pick_theta_runs_with_x_below_half_degree(void);
void test_console_still_ignores_a_fifth_of_a_degree(void);
void test_theta_refused_below_band_during_wait(void);
void test_budget_matches_timed_fake_pre_position(void);
void test_fail_path_off_target(void);
void test_deferred_x_arms_once_in_band(void);
void test_in_band_xz_together(void);
void test_three_segment_outbound(void);
void test_above_to_above_arms_x_at_band_entry(void);
void test_pnp_gripper_then_retract(void);
void test_bringup_requires_drive_managed(void);
void test_bringup_z_minus_then_timeout(void);
void test_bringup_phase_order_to_x_home(void);
void test_bringup_full_sequence_with_theta_90_orient(void);
void test_bringup_fails_when_theta_orient_faults(void);
void test_bringup_fails_when_theta_orient_times_out(void);
void test_drives_online_requires_every_link(void);
void test_inactive_servo_axis_reports_first_off_servo(void);
void test_theta_settled_at_requires_idle_no_alarm_and_position(void);
void test_sequential_theta_after_in_band_path(void);
void test_sequential_theta_gated_when_path_ends_deep(void);
void test_z_cap_on_z_only_segment(void);
void test_z_cap_on_combined_segment_keeps_it_straight(void);
void test_retract_to_safe_z_while_theta_alarmed(void);
void test_retract_stops_a_move_in_progress(void);
void test_grip_reports_write_failure(void);
void test_grip_fails_when_latch_does_not_confirm(void);
void test_cross_axis_fault_stops_orchestrator(void);
void test_z_accel_cap_preserves_full_x_accel(void);
void test_retract_to_safe_z_noop_inside_traverse_band(void);
void test_chain_move_after_retract_refused_while_preload_held(void);
void test_chain_move_after_retract_arms_x_and_theta_on_band_entry_then_descends(void);
void test_theta_full_speed_skips_window_throttling(void);
void test_bin_retract_pick_move_arms_x_before_z_completes(void);

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_move_not_initialized);
    RUN_TEST(test_move_not_enabled);
    RUN_TEST(test_move_not_calibrated);
    RUN_TEST(test_workspace_cal_clears_on_drive_lost);
    RUN_TEST(test_workspace_cal_clears_on_theta_lost);
    RUN_TEST(test_drive_fault_stop_does_not_rezero_x);
    RUN_TEST(test_move_invalid_position);
    RUN_TEST(test_orchestrator_bubbles_up_axis_rejection);
    RUN_TEST(test_move_already_moving);
    RUN_TEST(test_theta_refused_no_in_band_segment);
    RUN_TEST(test_theta_only_while_deep_refused);
    RUN_TEST(test_theta_only_to_joint_limits_in_band);
    RUN_TEST(test_theta_only_return_to_zero_stays_busy);
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_theta_does_not_start_before_25_percent);
    RUN_TEST(test_theta_starts_at_25_percent_window_speed);
    RUN_TEST(test_descent_held_until_theta_idle);
    RUN_TEST(test_stop_clears_pending_theta);
    RUN_TEST(test_retract_starts_theta_after_band_entry);
    RUN_TEST(test_theta_start_fraction_zero_starts_with_x);
    RUN_TEST(test_theta_start_fraction_caps_at_move_theta_speed);
    RUN_TEST(test_pick_theta_commands_a_tenth_while_waiting);
    RUN_TEST(test_pick_theta_truncates_below_a_tenth);
    RUN_TEST(test_pick_theta_quantizes_to_a_tenth);
    RUN_TEST(test_pick_theta_runs_with_x_below_half_degree);
    RUN_TEST(test_console_still_ignores_a_fifth_of_a_degree);
    RUN_TEST(test_theta_refused_below_band_during_wait);
    RUN_TEST(test_budget_matches_timed_fake_pre_position);
#endif
    RUN_TEST(test_fail_path_off_target);
    RUN_TEST(test_deferred_x_arms_once_in_band);
    RUN_TEST(test_in_band_xz_together);
    RUN_TEST(test_three_segment_outbound);
    RUN_TEST(test_above_to_above_arms_x_at_band_entry);
    RUN_TEST(test_pnp_gripper_then_retract);
    RUN_TEST(test_bringup_requires_drive_managed);
    RUN_TEST(test_bringup_z_minus_then_timeout);
    RUN_TEST(test_bringup_phase_order_to_x_home);
    RUN_TEST(test_bringup_full_sequence_with_theta_90_orient);
    RUN_TEST(test_bringup_fails_when_theta_orient_faults);
    RUN_TEST(test_bringup_fails_when_theta_orient_times_out);
    RUN_TEST(test_theta_settled_at_requires_idle_no_alarm_and_position);
    RUN_TEST(test_drives_online_requires_every_link);
    RUN_TEST(test_inactive_servo_axis_reports_first_off_servo);
#if CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_sequential_theta_after_in_band_path);
    RUN_TEST(test_sequential_theta_gated_when_path_ends_deep);
#endif
    RUN_TEST(test_z_cap_on_z_only_segment);
    RUN_TEST(test_z_cap_on_combined_segment_keeps_it_straight);
    RUN_TEST(test_z_accel_cap_preserves_full_x_accel);
    RUN_TEST(test_retract_to_safe_z_while_theta_alarmed);
    RUN_TEST(test_retract_to_safe_z_noop_inside_traverse_band);
    RUN_TEST(test_chain_move_after_retract_refused_while_preload_held);
    RUN_TEST(test_chain_move_after_retract_arms_x_and_theta_on_band_entry_then_descends);
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_theta_full_speed_skips_window_throttling);
#endif
    RUN_TEST(test_bin_retract_pick_move_arms_x_before_z_completes);
    RUN_TEST(test_retract_stops_a_move_in_progress);
    RUN_TEST(test_grip_reports_write_failure);
    RUN_TEST(test_grip_fails_when_latch_does_not_confirm);
    RUN_TEST(test_cross_axis_fault_stops_orchestrator);
    return UNITY_END();
}
