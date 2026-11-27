/**
 * @file test_pick_intercept_tracker.cpp
 * @brief Neighbour and tracker host tests.
 */

#include "test_pick_intercept_internal.h"

using namespace pick_intercept_test;

// ---------------------------------------------------------------------------
// Neighbours and tracker
// ---------------------------------------------------------------------------

void test_neighbour_crossing_a_finger_delays_the_fingers(void) {
  const PickIntercept::FingerGeom f;
  PickIntercept::Occupant occ;
  occ.dx_mm = 44.0f;
  occ.remain_mm = 100.0;
  occ.cell_deg = 90.0f;
  occ.cell = cell18650();
  // Jaw across X (jaw_deg = 0°). A 50 mm finger is clear of this occupant at 0.7875 s (200 mm/s).
  TEST_ASSERT_FLOAT_WITHIN(1.0f, PickIntercept::kWaitS,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.0f, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.7875f,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.8f,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.8f, 2.0f));
  occ.dx_mm = 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.0f, 2.0f));
}

void test_tracker_dedupes_and_edf_takes_the_earliest(void) {
  PickIntercept::PartTracker tracker;
  PickIntercept::TrackedPart early;
  early.item_id = 1;
  early.x_across_mm = 10.0f;
  early.y_bat_mm = -400.0f;
  early.s_capture_mm = 0.0;
  early.s_arrive_mm = 400.0;
  PickIntercept::TrackedPart later = early;
  later.item_id = 2;
  later.x_across_mm = 80.0f;
  later.s_arrive_mm = 700.0;
  PickIntercept::TrackedPart dup = early;
  dup.item_id = 3;

  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Added),
                    static_cast<int>(PickIntercept::trackAdd(tracker, later, 0.0, 40.0f, true, 0)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Added),
                    static_cast<int>(PickIntercept::trackAdd(tracker, early, 0.0, 40.0f, true, 0)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Duplicate),
                    static_cast<int>(PickIntercept::trackAdd(tracker, dup, 0.0, 40.0f, true, 0)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Infeasible),
                    static_cast<int>(PickIntercept::trackAdd(tracker, later, 0.0, 40.0f, false, 0)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Outside),
                    static_cast<int>(PickIntercept::trackAdd(tracker, early, 500.0, 40.0f, true, 0)));

  PickIntercept::TrackedPart copy[4];
  const int n = PickIntercept::trackCopy(tracker, copy, 4);
  TEST_ASSERT_EQUAL_INT(2, n);
  const PickIntercept::TrackedPart* best = &copy[0];
  for (int i = 1; i < n; ++i) {
    if (copy[i].s_arrive_mm < best->s_arrive_mm) {
      best = &copy[i];
    }
  }
  TEST_ASSERT_EQUAL_UINT32(1, best->item_id);
  PickIntercept::trackRemove(tracker, best->token);
  TEST_ASSERT_EQUAL_INT(1, tracker.count);
  TEST_ASSERT_EQUAL_UINT32(2, tracker.parts[0].item_id);

  PickIntercept::trackDropDownstream(tracker, 700.0 + PickIntercept::kDownstreamMm + 1.0);
  TEST_ASSERT_EQUAL_INT(0, tracker.count);
}

void test_tracker_matches_item_id_and_remembers_taken_cells(void) {
  PickIntercept::PartTracker tracker;
  PickIntercept::TrackedPart part;
  part.item_id = 7;
  part.x_across_mm = 10.0f;
  part.theta_deg = 20.0f;
  part.s_arrive_mm = 400.0;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Added),
                    static_cast<int>(PickIntercept::trackAdd(tracker, part, 0.0, 40.0f, true, 0)));
  // Same item_id far away in position is still the same cell; the repeat refreshes it.
  PickIntercept::TrackedPart again = part;
  again.s_arrive_mm = 460.0;
  again.theta_deg = 25.0f;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Duplicate),
                    static_cast<int>(PickIntercept::trackAdd(tracker, again, 0.0, 40.0f, true, 0)));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 460.0f, static_cast<float>(tracker.parts[0].s_arrive_mm));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 25.0f, tracker.parts[0].theta_deg);

  PickIntercept::trackNoteTaken(tracker, tracker.parts[0]);
  PickIntercept::trackRemove(tracker, tracker.parts[0].token);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Taken),
                    static_cast<int>(PickIntercept::trackAdd(tracker, again, 0.0, 40.0f, true, 0)));
  PickIntercept::TrackedPart unnamed = again;
  unnamed.item_id = 0;
  unnamed.s_arrive_mm = 470.0;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Taken),
                    static_cast<int>(PickIntercept::trackAdd(tracker, unnamed, 0.0, 40.0f, true, 0)));
  PickIntercept::trackClear(tracker);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Added),
                    static_cast<int>(PickIntercept::trackAdd(tracker, again, 0.0, 40.0f, true, 0)));
}

void test_development_profiles_grasp_from_the_object_top(void) {
  const float belt = 115.0f;
  for (uint8_t cls = 1; cls <= PickIntercept::kKnownClassCount; ++cls) {
    const PickIntercept::DevProfile* profile = PickIntercept::devProfile(cls);
    TEST_ASSERT_NOT_NULL(profile);
    const PickIntercept::CellGeom cell = PickIntercept::devCell(cls);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, profile->length_mm, cell.length_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, profile->width_mm, cell.width_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, profile->height_from_belt_mm, cell.height_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, PickIntercept::graspJointZMm(belt, cell.height_mm),
                             profile->grasp_joint_z_mm);
    TEST_ASSERT_TRUE(cell.width_mm > 0.0f && cell.width_mm <= 80.0f);
    if (cls <= PickIntercept::kClassCount) {
      TEST_ASSERT_NULL(PickIntercept::classPickReject(cls));
    } else {
      TEST_ASSERT_EQUAL_STRING("pack not targeted", PickIntercept::classPickReject(cls));
    }
  }
  TEST_ASSERT_EQUAL_STRING("AAA", PickIntercept::devProfile(1)->name);
  TEST_ASSERT_EQUAL_STRING("9V", PickIntercept::devProfile(5)->name);
  TEST_ASSERT_EQUAL_STRING("laptop", PickIntercept::devProfile(11)->name);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 114.0f, PickIntercept::devProfile(12)->length_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 25.0f, PickIntercept::devProfile(12)->height_from_belt_mm);
  TEST_ASSERT_EQUAL_STRING("powertool", PickIntercept::devProfile(13)->name);
  const float bench_stroke_mm = 105.5f;
  TEST_ASSERT_TRUE(PickIntercept::devProfile(1)->grasp_joint_z_mm > bench_stroke_mm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(2)->grasp_joint_z_mm > bench_stroke_mm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(6)->grasp_joint_z_mm > bench_stroke_mm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(7)->grasp_joint_z_mm > bench_stroke_mm);
  TEST_ASSERT_NULL(PickIntercept::devProfile(0));
  TEST_ASSERT_NULL(PickIntercept::devProfile(14));
  TEST_ASSERT_EQUAL_STRING("class", PickIntercept::classPickReject(14));
}

void test_vision_contract_rejects_each_missing_field(void) {
  PickIntercept::VisionSample sample;
  sample.item_id = 7;
  sample.battery_class = 1;
  sample.capture_valid = true;
  sample.x_across_mm = 10.0f;
  sample.y_bat_mm = -20.0f;
  sample.theta_deg = 15.0f;
  const PickIntercept::DevProfile* profile = PickIntercept::devProfile(1);
  sample.width_mm = profile->width_mm;
  sample.length_mm = profile->length_mm;
  sample.height_from_belt_mm = profile->height_from_belt_mm;
  TEST_ASSERT_NULL(PickIntercept::visionContractReject(sample, 80.0f));

  sample.item_id = 0;
  TEST_ASSERT_EQUAL_STRING("item id", PickIntercept::visionContractReject(sample, 80.0f));
  sample.item_id = 7;
  sample.battery_class = 14;
  TEST_ASSERT_EQUAL_STRING("class", PickIntercept::visionContractReject(sample, 80.0f));
  sample.battery_class = 11;
  sample.width_mm = PickIntercept::devProfile(11)->width_mm;
  sample.length_mm = PickIntercept::devProfile(11)->length_mm;
  sample.height_from_belt_mm = PickIntercept::devProfile(11)->height_from_belt_mm;
  TEST_ASSERT_NULL(PickIntercept::visionContractReject(sample, 80.0f));
  sample.battery_class = 1;
  sample.capture_valid = false;
  TEST_ASSERT_EQUAL_STRING("capture displacement",
                           PickIntercept::visionContractReject(sample, 80.0f));
  sample.capture_valid = true;
  sample.theta_deg = 180.0f;
  TEST_ASSERT_EQUAL_STRING("theta", PickIntercept::visionContractReject(sample, 80.0f));
  sample.theta_deg = 15.0f;
  sample.x_across_mm = std::nanf("");
  TEST_ASSERT_EQUAL_STRING("pose", PickIntercept::visionContractReject(sample, 80.0f));
  sample.x_across_mm = 10.0f;
  sample.width_mm = 0.0f;
  TEST_ASSERT_EQUAL_STRING("width", PickIntercept::visionContractReject(sample, 80.0f));
  sample.width_mm = profile->width_mm;
  sample.length_mm = 0.0f;
  TEST_ASSERT_EQUAL_STRING("length", PickIntercept::visionContractReject(sample, 80.0f));
  sample.length_mm = profile->length_mm;
  sample.height_from_belt_mm = 0.0f;
  TEST_ASSERT_EQUAL_STRING("height", PickIntercept::visionContractReject(sample, 80.0f));
}

void test_follower_pace_uses_the_post_bin_pose(void) {
  const auto cycle = PickIntercept::postBinCycle(100.0f, 200.0f, 90.0f, 10.0f, 35.7f, 90.8f, 90.0f,
                                                 0.19f, 0.19f, limits(750.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f,
                           cycle.grip_dwell_s + cycle.retract_s + cycle.transfer_s +
                               cycle.descend_s + cycle.open_dwell_s + cycle.bin_retract_s,
                           cycle.total_s);
  TEST_ASSERT_TRUE(cycle.transfer_s > 0.2f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, cycle.descend_s, cycle.bin_retract_s);

  const float ready = 1.0f + cycle.total_s;
  const float slow = 0.4f;
  const float fast = 0.2f;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::FollowerPace::Slow),
                    static_cast<int>(PickIntercept::followerPace(ready, ready + slow, slow, fast)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::FollowerPace::Fast),
                    static_cast<int>(PickIntercept::followerPace(ready, ready + fast, slow, fast)));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::FollowerPace::Skip),
                    static_cast<int>(
                        PickIntercept::followerPace(ready, ready + fast - 0.01f, slow, fast)));
}

void test_jaw_override_survives_a_supplied_width(void) {
  auto t = timingFor(PickIntercept::Clearance::EarlyZ, 20.0f, 0.0f);
  t.jaw_override_s = 0.080f;
  const auto leads = PickIntercept::strikeLeads(t, 400.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.100f, leads.grip_lead_s);
  t.jaw_override_s = 0.0f;
  const auto derived = PickIntercept::strikeLeads(t, 400.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.020f + 20.0f / 210.0f, derived.grip_lead_s);
}

void test_contact_lead_pulls_depth_ahead_of_center_arrival(void) {
  const auto m = limits();
  const auto budget =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 20.0f, 35.7f, 35.7f, 50.0f, 0.0f, m);
  PickIntercept::GripGeometry lane;
  lane.mode = PickIntercept::Clearance::EarlyZ;
  lane.side_gap_mm = 30.0f;
  lane.along_factor = 1.0f;
  auto t = timingFor(PickIntercept::Clearance::EarlyZ, 30.0f, 1.0f);
  const float lead = PickIntercept::beltContactLeadS(30.0f, 300.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, lead);
  const auto ok = PickIntercept::planPick(2.0f, 300.0f, budget, lane, t);
  TEST_ASSERT_TRUE(ok.feasible);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f - lead - t.guard_s - ok.z_done_s, ok.slack_s);
  const auto late = PickIntercept::planPick(ok.z_done_s + lead + t.guard_s - 0.001f, 300.0f, budget,
                                            lane, t);
  TEST_ASSERT_FALSE(late.feasible);
  TEST_ASSERT_EQUAL_STRING("late", late.reason);
}

void test_neighbour_occupancy_covers_the_retract(void) {
  const PickIntercept::FingerGeom f;
  PickIntercept::Occupant occ;
  occ.dx_mm = 44.0f;
  occ.remain_mm = 200.0;
  occ.cell_deg = 90.0f;
  occ.cell = cell18650();
  // A grip-only horizon ends before the occupant reaches the finger.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.0f, 0.4f));
  // Extending the horizon through the retract waits until the finger is clear.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.2875f,
                           PickIntercept::fingersFreeFromS(&occ, 1, 0.0f, f, 200.0f, 0.0f, 2.0f));
}

void test_budget_uses_z_accel_cap_separately_from_x(void) {
  auto m = limits(3000.0f, 18000.0f, 900.0f);
  m.z_accel = 5000.0f;
  m.z_decel = 5000.0f;
  m.theta_cap = 3240.0f;
  m.theta_accel = 16200.0f;
  m.theta_decel = 16200.0f;

  const auto budget =
      PickIntercept::budgetPick(0.0f, 48.4f, 0.0f, 137.5f, 48.4f, 48.4f, 119.0f, 0.0f, m);
  const float expected_x =
      PickIntercept::profileSeconds(137.5f, 3000.0f, 18000.0f, 18000.0f) +
      PickIntercept::kPreloadS + PickIntercept::kSettleS;
  const float expected_z =
      PickIntercept::profileSeconds(119.0f - 48.4f, 900.0f, 5000.0f, 5000.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected_x, budget.linear_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected_z, budget.z_motion_s);

  const auto cyc =
      PickIntercept::postBinCycle(119.0f, 382.5f, 0.0f, 10.0f, 48.4f, 90.8f, 90.0f, 0.0f, 0.19f, m);
  const float expected_retract =
      PickIntercept::profileSeconds(119.0f - 48.4f, 900.0f, 5000.0f, 5000.0f) +
      PickIntercept::kPreloadS + PickIntercept::kSettleS;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected_retract, cyc.retract_s);
}

void test_retime_strike_events_preserves_ballistic_invariants(void) {
  PickIntercept::ArmedStrikeTimes st;
  st.z_pending = true;
  st.grip_pending = true;
  st.retract_pending = true;
  st.z_rel_us = -268000;
  st.close_rel_us = -164000;
  st.retract_rel_us = -22000;
  st.min_z_to_retract_us = 246000;  // 30 + 238 - 22 ms
  const int64_t close_to_retract = st.retract_rel_us - st.close_rel_us;  // 142000 us

  // 1. Pre-Z: all three track arrive_us together.
  PickIntercept::retimeStrikeEvents(st, 1000000);
  TEST_ASSERT_EQUAL_INT64(732000, st.z_at_us);
  TEST_ASSERT_EQUAL_INT64(836000, st.grip_at_us);
  TEST_ASSERT_EQUAL_INT64(978000, st.retract_at_us);

  // 2. Z fires at 732500 us. Belt slows (+20 ms arrive shift) before grip fires:
  //    retract shifts +20 ms and grip shifts with it so retract - grip stays constant.
  st.z_pending = false;
  st.z_fired_us = 732500;
  PickIntercept::retimeStrikeEvents(st, 1020000);
  TEST_ASSERT_EQUAL_INT64(732000, st.z_at_us);
  TEST_ASSERT_EQUAL_INT64(998000, st.retract_at_us);
  TEST_ASSERT_EQUAL_INT64(998000 - close_to_retract, st.grip_at_us);

  // 3. Belt speeds up (-30 ms arrive shift) while Z is in flight and grip still pending:
  //    retract clamps to z_fired + min_z_to_retract (no mid-air reversal), grip follows.
  PickIntercept::retimeStrikeEvents(st, 970000);
  TEST_ASSERT_EQUAL_INT64(732500 + 246000, st.retract_at_us);
  TEST_ASSERT_EQUAL_INT64(st.retract_at_us - close_to_retract, st.grip_at_us);

  // 4. Grip fires late at 860000 us; belt jitter (-25 ms or +25 ms) cannot shrink
  //    or stretch the ballistic close->retract duration.
  st.grip_pending = false;
  st.grip_fired_us = 860000;
  PickIntercept::retimeStrikeEvents(st, 945000);
  TEST_ASSERT_EQUAL_INT64(860000 + close_to_retract, st.retract_at_us);
  PickIntercept::retimeStrikeEvents(st, 1050000);
  TEST_ASSERT_EQUAL_INT64(860000 + close_to_retract, st.retract_at_us);
}

void test_plunge_overdue_guard_blocks_late_release(void) {
  constexpr int64_t kMaxLate = 15000;
  TEST_ASSERT_FALSE(PickIntercept::isPlungeOverdue(1000000, 1050000, kMaxLate));
  TEST_ASSERT_FALSE(PickIntercept::isPlungeOverdue(1015000, 1000000, kMaxLate));
  TEST_ASSERT_TRUE(PickIntercept::isPlungeOverdue(1015001, 1000000, kMaxLate));
  TEST_ASSERT_TRUE(PickIntercept::isPlungeOverdue(1060000, 1000000, kMaxLate));
}

void test_budget_skips_theta_when_already_within_arrival_tolerance(void) {
  const auto m = limits();
  // Theta settled at -44.89 deg vs -45.00 deg (0.11 deg <= kArriveEpsDeg=1.0):
  // runPick will not command another theta move, so approach_s must stay 0.
  const auto settled =
      PickIntercept::budgetPick(382.5f, 48.4f, -44.89f, 382.5f, 48.4f, 48.4f, 119.0f, -45.0f, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, settled.theta_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, settled.approach_s);

  // Beyond kArriveEpsDeg (2.0 deg), theta start + profile + settle is budgeted.
  const auto needs_turn =
      PickIntercept::budgetPick(382.5f, 48.4f, -43.0f, 382.5f, 48.4f, 48.4f, 119.0f, -45.0f, m);
  TEST_ASSERT_TRUE(needs_turn.theta_s > PickIntercept::kThetaStartS);
  TEST_ASSERT_TRUE(needs_turn.approach_s >= needs_turn.theta_s + PickIntercept::kSettleS);
}

void test_budget_theta_full_speed_uses_theta_cap(void) {
  auto m_win = limits();
  m_win.speed = 1330.0f;
  m_win.theta_cap = 1000.0f;
  m_win.theta_accel = 3600.0f;
  m_win.theta_decel = 3600.0f;
  m_win.theta_full_speed = false;
  auto m_full = m_win;
  m_full.theta_full_speed = true;

  const auto b_win =
      PickIntercept::budgetPick(10.0f, 48.4f, 90.0f, 382.5f, 48.4f, 48.4f, 119.0f, 0.0f, m_win);
  const auto b_full =
      PickIntercept::budgetPick(10.0f, 48.4f, 90.0f, 382.5f, 48.4f, 48.4f, 119.0f, 0.0f, m_full);
  const auto w = Gantry::Path::planThetaWindow(372.5f, m_win.speed, 90.0f, m_win.theta_cap, 0.0f);
  const float expected_win = PickIntercept::kTickS + PickIntercept::kThetaStartS +
                             PickIntercept::profileSeconds(90.0f, w.speed_deg_per_s,
                                                           m_win.theta_accel, m_win.theta_decel);
  const float expected_full = PickIntercept::kTickS + PickIntercept::kThetaStartS +
                              PickIntercept::profileSeconds(90.0f, m_full.theta_cap,
                                                            m_full.theta_accel, m_full.theta_decel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected_win, b_win.theta_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected_full, b_full.theta_s);
  TEST_ASSERT_TRUE(b_full.theta_s < b_win.theta_s);
}
