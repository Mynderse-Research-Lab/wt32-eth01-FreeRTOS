// Host unit tests for EipProcessImage, GantryEipLinearAxis, GantryEipRotaryAxis,
// and scanner/axis process-image integration.

#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "CipMessageRouter.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipTransport.h"
#include "GantryEipLinearAxis.h"
#include "GantryEipRotaryAxis.h"
#include "GantryThetaOrigin.h"
#include "GantryUtils.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"
#include "host_test_clock.h"
#include "KinetixFaultCodes.h"
#include "axis_drivetrain_params.h"

void setUp(void) {}
void tearDown(void) {}

using eip::Bytes;

#include "test_eip_axis_internal.h"
using namespace eip_axis_test;

void test_process_image_command_feedback(void);
void test_process_image_online(void);
void test_linear_x_belt_move_command(void);
void test_linear_held_start_waits_for_release(void);
void test_linear_queued_move_waits_for_run_and_edge(void);
void test_linear_offline_aborts_move(void);
void test_linear_second_absolute_preloads_before_start_edge(void);
void test_linear_move_stays_busy_until_target(void);
void test_linear_arrival_requires_stable_ticks(void);
void test_axis_debounces_command_value_reached_jitter(void);
void test_axis_handles_drive_power_loss_while_network_online(void);
void test_linear_soft_home_joint_frame(void);
void test_linear_absolute_keeps_target_on_overshoot_feedback(void);
void test_linear_move_does_not_stop_at_half_travel(void);
void test_linear_z_ballscrew_scaling(void);
void test_linear_move_rejects_drive_limit_stop(void);
void test_linear_feedback_and_alarm(void);
void test_kinetix_a603_warning_decode(void);
void test_kinetix_a014_a015_helpers(void);
void test_kinetix_format_helpers_and_a603(void);
void test_linear_absolute_aborts_busy_on_a015(void);
void test_stop_motion_clears_busy_when_position_stable(void);
void test_stop_motion_stays_busy_while_position_creeps(void);
void test_stop_motion_times_out_while_still_moving(void);
void test_servo_arm_times_out_when_active_stays_clear(void);
void test_home34_times_out_when_homed_stays_clear(void);
void test_move_while_disabled_does_not_arm_motion(void);
void test_abort_image_differs_from_move_only_by_stop_bit(void);
void test_rotary_enable_holds_feedback_and_keeps_halt(void);
void test_rotary_arst_while_disabled_stays_drive_off(void);
void test_rotary_move_and_feedback(void);
void test_theta_ah_times_out_when_ready_stays_below_operation(void);
void test_rotary_offline_aborts_move(void);
void test_rotary_move_stamp_and_trace_sample(void);
void test_rotary_consecutive_move_ignores_stale_in_pos(void);
void test_rotary_unaligned_home_offsets_and_clamps_to_drive_travel(void);
void test_rotary_aligned_home_is_drive_identity(void);
void test_rotary_unaligned_small_delta_keeps_offset_command(void);
void test_rotary_set_puu_per_deg(void);
void test_hcs01_units_match_drive_scaling(void);
void test_rotary_default_accel_matches_explicit_accel(void);
void test_theta_move_timeout_scales_with_distance(void);
void test_rotary_long_slow_move_outlives_fixed_timeout(void);
void test_rotary_halt_clears_start_bit(void);
void test_rotary_alarm(void);
void test_rotary_thetalim_cannot_expand_past_offset_envelope(void);
void test_rotary_alarm_summary_and_cip_status(void);
void test_axis_log_due_period(void);
void test_scanner_process_image_exchange(void);
void test_rotary_settles_within_hcs01_window_and_sub_window_move(void);

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_process_image_command_feedback);
  RUN_TEST(test_process_image_online);
  RUN_TEST(test_linear_x_belt_move_command);
  RUN_TEST(test_linear_held_start_waits_for_release);
  RUN_TEST(test_linear_queued_move_waits_for_run_and_edge);
  RUN_TEST(test_linear_offline_aborts_move);
  RUN_TEST(test_linear_second_absolute_preloads_before_start_edge);
  RUN_TEST(test_linear_move_stays_busy_until_target);
  RUN_TEST(test_linear_arrival_requires_stable_ticks);
  RUN_TEST(test_axis_debounces_command_value_reached_jitter);
  RUN_TEST(test_axis_handles_drive_power_loss_while_network_online);
  RUN_TEST(test_linear_soft_home_joint_frame);
  RUN_TEST(test_linear_absolute_keeps_target_on_overshoot_feedback);
  RUN_TEST(test_linear_move_does_not_stop_at_half_travel);
  RUN_TEST(test_linear_z_ballscrew_scaling);
  RUN_TEST(test_linear_move_rejects_drive_limit_stop);
  RUN_TEST(test_linear_feedback_and_alarm);
  RUN_TEST(test_kinetix_a603_warning_decode);
  RUN_TEST(test_kinetix_a014_a015_helpers);
  RUN_TEST(test_kinetix_format_helpers_and_a603);
  RUN_TEST(test_linear_absolute_aborts_busy_on_a015);
  RUN_TEST(test_stop_motion_clears_busy_when_position_stable);
  RUN_TEST(test_stop_motion_stays_busy_while_position_creeps);
  RUN_TEST(test_stop_motion_times_out_while_still_moving);
  RUN_TEST(test_servo_arm_times_out_when_active_stays_clear);
  RUN_TEST(test_home34_times_out_when_homed_stays_clear);
  RUN_TEST(test_move_while_disabled_does_not_arm_motion);
  RUN_TEST(test_abort_image_differs_from_move_only_by_stop_bit);
  RUN_TEST(test_rotary_enable_holds_feedback_and_keeps_halt);
  RUN_TEST(test_rotary_arst_while_disabled_stays_drive_off);
  RUN_TEST(test_rotary_move_and_feedback);
  RUN_TEST(test_hcs01_units_match_drive_scaling);
  RUN_TEST(test_rotary_default_accel_matches_explicit_accel);
  RUN_TEST(test_theta_move_timeout_scales_with_distance);
  RUN_TEST(test_rotary_long_slow_move_outlives_fixed_timeout);
  RUN_TEST(test_theta_ah_times_out_when_ready_stays_below_operation);
  RUN_TEST(test_rotary_offline_aborts_move);
  RUN_TEST(test_rotary_consecutive_move_ignores_stale_in_pos);
  RUN_TEST(test_rotary_settles_within_hcs01_window_and_sub_window_move);
  RUN_TEST(test_rotary_unaligned_home_offsets_and_clamps_to_drive_travel);
  RUN_TEST(test_rotary_aligned_home_is_drive_identity);
  RUN_TEST(test_rotary_unaligned_small_delta_keeps_offset_command);
  RUN_TEST(test_rotary_set_puu_per_deg);
  RUN_TEST(test_rotary_halt_clears_start_bit);
  RUN_TEST(test_rotary_alarm);
  RUN_TEST(test_rotary_thetalim_cannot_expand_past_offset_envelope);
  RUN_TEST(test_rotary_alarm_summary_and_cip_status);
  RUN_TEST(test_rotary_move_stamp_and_trace_sample);
  RUN_TEST(test_axis_log_due_period);
  RUN_TEST(test_scanner_process_image_exchange);
  return UNITY_END();
}
