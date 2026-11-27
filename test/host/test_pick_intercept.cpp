// Host tests for belt tracking, side-grip geometry, strike timing, and path-budget parity.

#include "unity.h"

#include "pick_intercept.h"

#include <cmath>

void setUp(void) {}
void tearDown(void) {}


#include "test_pick_intercept_internal.h"

void test_position_reanchors_on_displacement(void);
void test_timestamp_wrap_is_not_a_restart(void);
void test_sequence_wrap_is_not_a_restart(void);
void test_displacement_backward_restarts_and_bumps_epoch(void);
void test_sequence_reset_restarts(void);
void test_min_delay_anchors_at_conveyor_time(void);
void test_stale_after_two_missed_frames(void);
void test_capture_subtracts_latency_until_displacement_is_valid(void);
void test_flat_displacement_falls_back_to_speed(void);
void test_side_jaw_sweep_is_perpendicular_and_short(void);
void test_full_width_is_a_pick_and_grasp_is_half_height(void);
void test_command_theta_quantizes_to_a_tenth(void);
void test_side_jaw_is_linear_and_rejects_when_nothing_fits(void);
void test_lane_band_is_about_29_deg_for_18650(void);
void test_lane_band_is_narrower_for_21700(void);
void test_dropin_window_boundary_against_belt_speed(void);
void test_grip_lead_from_jaw_travel_and_contact_speed(void);
void test_dropin_timeline_never_overlaps_a_finger_before_crossing(void);
void test_separating_axis_matches_a_known_overlap(void);
void test_slow_approach_unless_a_later_part_is_at_risk(void);
void test_arrival_uses_the_gantry_settle_tolerances(void);
void test_collision_envelope_blocks_deep_z_over_the_conveyor(void);
void test_profile_end_matches_time_along(void);
void test_in_band_budget_matches_path_segment(void);
void test_retract_into_band_defers_x(void);
void test_z_cap_slows_combined_segments(void);
void test_theta_budget_counts_rotary_phases_from_move_start(void);
void test_theta_budget_uses_linear_delta(void);
void test_hover_is_budgeted_before_the_dropin_tail(void);
void test_jaw_is_frozen_once_z_leaves_the_band(void);
void test_lane_strike_slack_and_dropin_release(void);
void test_retime_follows_a_speed_change(void);
void test_epoch_rejects_parts_and_strikes_from_before_a_restart(void);
void test_arrival_is_camera_relative(void);
void test_zero_speed_does_not_invent_an_arrival(void);
void test_neighbour_crossing_a_finger_delays_the_fingers(void);
void test_tracker_dedupes_and_edf_takes_the_earliest(void);
void test_tracker_matches_item_id_and_remembers_taken_cells(void);
void test_development_profiles_grasp_from_the_object_top(void);
void test_vision_contract_rejects_each_missing_field(void);
void test_follower_pace_uses_the_post_bin_pose(void);
void test_jaw_override_survives_a_supplied_width(void);
void test_contact_lead_pulls_depth_ahead_of_center_arrival(void);
void test_neighbour_occupancy_covers_the_retract(void);
void test_budget_uses_z_accel_cap_separately_from_x(void);
void test_retime_strike_events_preserves_ballistic_invariants(void);
void test_plunge_overdue_guard_blocks_late_release(void);
void test_budget_skips_theta_when_already_within_arrival_tolerance(void);
void test_budget_theta_full_speed_uses_theta_cap(void);

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_position_reanchors_on_displacement);
  RUN_TEST(test_timestamp_wrap_is_not_a_restart);
  RUN_TEST(test_sequence_wrap_is_not_a_restart);
  RUN_TEST(test_displacement_backward_restarts_and_bumps_epoch);
  RUN_TEST(test_sequence_reset_restarts);
  RUN_TEST(test_min_delay_anchors_at_conveyor_time);
  RUN_TEST(test_stale_after_two_missed_frames);
  RUN_TEST(test_capture_subtracts_latency_until_displacement_is_valid);
  RUN_TEST(test_flat_displacement_falls_back_to_speed);
  RUN_TEST(test_full_width_is_a_pick_and_grasp_is_half_height);
  RUN_TEST(test_command_theta_quantizes_to_a_tenth);
  RUN_TEST(test_side_jaw_sweep_is_perpendicular_and_short);
  RUN_TEST(test_side_jaw_is_linear_and_rejects_when_nothing_fits);
  RUN_TEST(test_lane_band_is_about_29_deg_for_18650);
  RUN_TEST(test_lane_band_is_narrower_for_21700);
  RUN_TEST(test_dropin_window_boundary_against_belt_speed);
  RUN_TEST(test_grip_lead_from_jaw_travel_and_contact_speed);
  RUN_TEST(test_dropin_timeline_never_overlaps_a_finger_before_crossing);
  RUN_TEST(test_separating_axis_matches_a_known_overlap);
  RUN_TEST(test_slow_approach_unless_a_later_part_is_at_risk);
  RUN_TEST(test_arrival_uses_the_gantry_settle_tolerances);
  RUN_TEST(test_collision_envelope_blocks_deep_z_over_the_conveyor);
  RUN_TEST(test_profile_end_matches_time_along);
  RUN_TEST(test_in_band_budget_matches_path_segment);
  RUN_TEST(test_retract_into_band_defers_x);
  RUN_TEST(test_z_cap_slows_combined_segments);
  RUN_TEST(test_budget_uses_z_accel_cap_separately_from_x);
  RUN_TEST(test_theta_budget_counts_rotary_phases_from_move_start);
  RUN_TEST(test_theta_budget_uses_linear_delta);
  RUN_TEST(test_hover_is_budgeted_before_the_dropin_tail);
  RUN_TEST(test_jaw_is_frozen_once_z_leaves_the_band);
  RUN_TEST(test_lane_strike_slack_and_dropin_release);
  RUN_TEST(test_retime_follows_a_speed_change);
  RUN_TEST(test_retime_strike_events_preserves_ballistic_invariants);
  RUN_TEST(test_plunge_overdue_guard_blocks_late_release);
  RUN_TEST(test_budget_skips_theta_when_already_within_arrival_tolerance);
  RUN_TEST(test_budget_theta_full_speed_uses_theta_cap);
  RUN_TEST(test_epoch_rejects_parts_and_strikes_from_before_a_restart);
  RUN_TEST(test_arrival_is_camera_relative);
  RUN_TEST(test_zero_speed_does_not_invent_an_arrival);
  RUN_TEST(test_neighbour_crossing_a_finger_delays_the_fingers);
  RUN_TEST(test_development_profiles_grasp_from_the_object_top);
  RUN_TEST(test_vision_contract_rejects_each_missing_field);
  RUN_TEST(test_follower_pace_uses_the_post_bin_pose);
  RUN_TEST(test_jaw_override_survives_a_supplied_width);
  RUN_TEST(test_contact_lead_pulls_depth_ahead_of_center_arrival);
  RUN_TEST(test_neighbour_occupancy_covers_the_retract);
  RUN_TEST(test_tracker_dedupes_and_edf_takes_the_earliest);
  RUN_TEST(test_tracker_matches_item_id_and_remembers_taken_cells);
  return UNITY_END();
}
