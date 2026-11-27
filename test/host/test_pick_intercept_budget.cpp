/**
 * @file test_pick_intercept_budget.cpp
 * @brief Budget, schedule and retime host tests.
 */

#include "test_pick_intercept_internal.h"

using namespace pick_intercept_test;

// ---------------------------------------------------------------------------
// Budget and schedule
// ---------------------------------------------------------------------------

void test_profile_end_matches_time_along(void) {
  const float length = 70.0f;
  const float speed = 1000.0f;
  const float accel = 3000.0f;
  const float total = PickIntercept::profileSeconds(length, speed, accel, accel);
  const float along =
      PickIntercept::timeAlongProfile(length, length, speed, accel, accel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, total, along);
  TEST_ASSERT_TRUE(total > 0.2f && total < 0.5f);
}

void test_in_band_budget_matches_path_segment(void) {
  const auto m = limits();
  const auto budget =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 200.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  Gantry::Path::PathSegment segs[3];
  const size_t n = Gantry::Path::planSegments(0.0f, 35.7f, 200.0f, 35.7f, 35.7f, segs);
  TEST_ASSERT_EQUAL_UINT(1, n);
  TEST_ASSERT_TRUE(segs[0].move_x);
  TEST_ASSERT_FALSE(segs[0].move_z);
  const float expect = PickIntercept::profileSeconds(200.0f, m.speed, m.accel, m.decel) +
                       PickIntercept::kPreloadS + PickIntercept::kSettleS;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expect, budget.approach_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2000.0f, budget.approach_speed_mm_s);
  const float z_motion = PickIntercept::profileSeconds(105.4f - 35.7f, 1000.0f, m.accel, m.decel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, z_motion, budget.z_motion_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, budget.descend_speed_mm_s);
  TEST_ASSERT_TRUE(budget.z_limited);
}

void test_retract_into_band_defers_x(void) {
  const auto m = limits(1000.0f);
  const auto budget =
      PickIntercept::budgetPick(0.0f, 80.0f, 0.0f, 100.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  const float t_z = PickIntercept::profileSeconds(80.0f - 35.7f, 1000.0f, m.accel, m.decel) +
                    PickIntercept::kPreloadS;
  const float t_x =
      PickIntercept::profileSeconds(100.0f, 1000.0f, m.accel, m.decel) + PickIntercept::kPreloadS;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, t_z + t_x + PickIntercept::kSettleS, budget.approach_s);
}

void test_z_cap_slows_combined_segments(void) {
  const auto m = limits(2000.0f, 3000.0f, 1000.0f);
  // Down 60 mm while moving 40 mm across: |dz|/L = 0.83, so V drops to 1202 mm/s.
  const float v = Gantry::Path::zCappedPathSpeed(m.speed, 40.0f, -60.0f, m.z_cap);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f * std::hypot(40.0f, 60.0f) / 60.0f, v);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2000.0f, Gantry::Path::zCappedPathSpeed(m.speed, 40.0f, 0.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, Gantry::Path::zCappedPathSpeed(m.speed, 0.0f, 30.0f, 1000.0f));
  const Gantry::Path::PathSegment z_only{0.0f, 60.0f, false, true};
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f,
      PickIntercept::profileSeconds(60.0f, 1000.0f, m.accel, m.decel) + PickIntercept::kPreloadS +
          PickIntercept::kSettleS,
      PickIntercept::segmentWallSeconds(0.0f, 0.0f, z_only, 35.7f, m.speed, m.accel, m.decel,
                                        m.z_cap));
}

void test_theta_budget_counts_rotary_phases_from_move_start(void) {
  const auto m = limits();
  const float len = 200.0f;
  const float dth = 90.0f;
  const auto plain =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, len, 35.7f, 35.7f, 105.4f, 0.0f, m);
  const auto turned =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, len, 35.7f, 35.7f, 105.4f, dth, m);
  const auto w = Gantry::Path::planThetaWindow(len, m.speed, dth, m.theta_cap, 0.0f);
  TEST_ASSERT_TRUE(w.runnable);
  const float theta_end = PickIntercept::kTickS + PickIntercept::kThetaStartS +
                          PickIntercept::profileSeconds(dth, w.speed_deg_per_s, m.theta_accel,
                                                        m.theta_decel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, theta_end, turned.theta_s);
  const float expect = plain.linear_s > theta_end + PickIntercept::kSettleS
                           ? plain.linear_s
                           : theta_end + PickIntercept::kSettleS;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expect, turned.approach_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, plain.z_motion_s, turned.z_motion_s);

  // Starting below the band, theta counts from the first in-band instant.
  const auto low =
      PickIntercept::budgetPick(0.0f, 80.0f, 0.0f, len, 35.7f, 35.7f, 105.4f, dth, m);
  const float t_band = PickIntercept::timeAlongProfile(80.0f - 35.7f, 80.0f - 35.7f, 1000.0f,
                                                       m.accel, m.decel) +
                       PickIntercept::kPreloadS;
  TEST_ASSERT_TRUE(low.theta_s > t_band + PickIntercept::kThetaStartS);
}

void test_theta_budget_uses_linear_delta(void) {
  const auto m = limits();
  const auto short_turn =
      PickIntercept::budgetPick(0.0f, 35.7f, 170.0f, 10.0f, 35.7f, 35.7f, 105.4f, 90.0f, m);
  const auto long_turn =
      PickIntercept::budgetPick(0.0f, 35.7f, 170.0f, 10.0f, 35.7f, 35.7f, 105.4f, -90.0f, m);
  TEST_ASSERT_TRUE(long_turn.theta_s > short_turn.theta_s + 0.3f);
}

void test_hover_is_budgeted_before_the_dropin_tail(void) {
  const auto m = limits();
  const auto lane =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 100.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  const auto drop =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 100.0f, 35.7f, 90.0f, 105.4f, 0.0f, m);
  const float hover = PickIntercept::profileSeconds(90.0f - 35.7f, 1000.0f, m.accel, m.decel) +
                      PickIntercept::kPreloadS + PickIntercept::kSettleS;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, hover, drop.hover_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, lane.approach_s + hover, drop.approach_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f,
                           PickIntercept::profileSeconds(15.4f, 1000.0f, m.accel, m.decel),
                           drop.z_motion_s);
}

void test_jaw_is_frozen_once_z_leaves_the_band(void) {
  const auto m = limits();
  // At the hover (out of band) with the jaw committed: no X, no theta in the budget.
  const auto at_hover =
      PickIntercept::budgetPick(100.0f, 90.0f, 170.0f, 100.0f, 90.0f, 90.0f, 105.4f, 170.0f, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, at_hover.approach_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, at_hover.theta_s);
  // A refined angle is checked against the frozen jaw, not re-chosen.
  const PickIntercept::CellGeom cell = cell18650();
  const PickIntercept::FingerGeom f;
  const auto first = PickIntercept::classifyClearance(80.0f, 170.0f, cell, f, true);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::EarlyZ), static_cast<int>(first.mode));
  const auto refined = PickIntercept::classifyClearance(10.0f, 170.0f, cell, f, true);
  TEST_ASSERT_TRUE(refined.mode != PickIntercept::Clearance::EarlyZ);
}

void test_lane_strike_slack_and_dropin_release(void) {
  const auto m = limits();
  const auto budget =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 100.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  PickIntercept::GripGeometry lane;
  lane.mode = PickIntercept::Clearance::EarlyZ;
  lane.side_gap_mm = 30.85f;
  lane.along_factor = 0.0f;
  const auto t = timingFor(PickIntercept::Clearance::EarlyZ, 30.85f, 0.0f);
  const auto ok = PickIntercept::planPick(1.0f, 300.0f, budget, lane, t);
  TEST_ASSERT_TRUE(ok.feasible);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, budget.approach_s + PickIntercept::kPreloadS, ok.z_start_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f - t.guard_s - ok.z_done_s, ok.slack_s);
  const auto late = PickIntercept::planPick(ok.z_done_s, 300.0f, budget, lane, t);
  TEST_ASSERT_FALSE(late.feasible);
  TEST_ASSERT_EQUAL_STRING("late", late.reason);

  const auto short_z =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 100.0f, 35.7f, 100.0f, 105.0f, 0.0f, m);
  PickIntercept::GripGeometry drop = lane;
  drop.mode = PickIntercept::Clearance::LateZ;
  drop.along_factor = 1.0f;
  auto td = timingFor(PickIntercept::Clearance::LateZ, 30.85f, 1.0f);
  td.z_tail_s = short_z.z_motion_s;
  td.z_top_s = 0.25f * short_z.z_motion_s;
  const auto leads = PickIntercept::strikeLeads(td, 300.0f);
  const float contact_lead = PickIntercept::beltContactLeadS(30.85f, 300.0f, 1.0f);
  const auto d = PickIntercept::planPick(2.0f, 300.0f, short_z, drop, td);
  TEST_ASSERT_TRUE(leads.ok);
  TEST_ASSERT_TRUE(d.feasible);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f - contact_lead - leads.release_lead_s, d.z_start_s);
  TEST_ASSERT_TRUE(d.z_done_s <= 2.0f - contact_lead - td.guard_s + 1e-4f);
  td.z_tail_s = 0.02f;
  const auto too_late = PickIntercept::planPick(1.0f, 300.0f, budget, drop, td);
  TEST_ASSERT_FALSE(too_late.feasible);
  TEST_ASSERT_TRUE(too_late.z_done_s > 1.0f - td.guard_s);
  const auto stopped = PickIntercept::planPick(PickIntercept::kWaitS, 0.0f, budget, lane, t);
  TEST_ASSERT_FALSE(stopped.feasible);
}

// ---------------------------------------------------------------------------
// Re-timing, epochs, arrival
// ---------------------------------------------------------------------------

void test_retime_follows_a_speed_change(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000000, 1000000, 200.0f, 0.0f, 1));
  auto t = timingFor(PickIntercept::Clearance::EarlyZ, 30.85f, 0.0f);
  t.s_arrive_mm = 1000.0;
  const auto first = PickIntercept::retimePick(t, belt, 1000000);
  TEST_ASSERT_TRUE(first.ok);
  TEST_ASSERT_INT64_WITHIN(10, 1000000 + 5000000, first.arrive_us);
  const auto leads = PickIntercept::strikeLeads(t, 200.0f);
  TEST_ASSERT_INT64_WITHIN(10, first.arrive_us - static_cast<int64_t>(leads.grip_lead_s * 1.0e6f),
                           first.grip_us);
  PickIntercept::noteConveyor(belt, frameAt(1250000, 1250000, 400.0f, 0.05f, 2));
  const auto second = PickIntercept::retimePick(t, belt, 1250000);
  TEST_ASSERT_TRUE(second.ok);
  TEST_ASSERT_INT64_WITHIN(10, 1250000 + 2375000, second.arrive_us);
  PickIntercept::noteConveyor(belt, frameAt(1500000, 1500000, 0.0f, 0.05f, 3));
  const auto stopped = PickIntercept::retimePick(t, belt, 1500000);
  TEST_ASSERT_FALSE(stopped.ok);
  TEST_ASSERT_EQUAL_STRING("belt stopped", stopped.reason);
  const auto stale =
      PickIntercept::retimePick(t, belt, 1500000 + PickIntercept::kStaleAfterUs + 1);
  TEST_ASSERT_FALSE(stale.ok);
}

void test_epoch_rejects_parts_and_strikes_from_before_a_restart(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 2.0f, 10));
  auto t = timingFor(PickIntercept::Clearance::EarlyZ, 30.85f, 0.0f);
  t.epoch = belt.epoch;
  t.s_arrive_mm = 2500.0;
  PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 0.1f, 11));
  const auto r = PickIntercept::retimePick(t, belt, 2000);
  TEST_ASSERT_FALSE(r.ok);
  TEST_ASSERT_EQUAL_STRING("conveyor restart", r.reason);

  PickIntercept::PartTracker tracker;
  PickIntercept::TrackedPart old;
  old.epoch = 0;
  old.s_arrive_mm = 300.0;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::TrackAdd::Epoch),
                    static_cast<int>(PickIntercept::trackAdd(tracker, old, 100.0, 40.0f, true,
                                                             belt.epoch)));
}

void test_arrival_is_camera_relative(void) {
  // Cell at the camera centre, camera 711 mm upstream of the jaw plane.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1711.0f,
                           static_cast<float>(PickIntercept::arriveBeltMm(1000.0, 0.0f, -711.0f, 0.0f)));
  // 11 mm downstream of the camera centre arrives 11 mm sooner.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1700.0f,
                           static_cast<float>(PickIntercept::arriveBeltMm(1000.0, 0.0f, -711.0f, 11.0f)));
}

void test_zero_speed_does_not_invent_an_arrival(void) {
  const float wait = PickIntercept::secondsToPlane(711.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, PickIntercept::kWaitS, wait);
  const float late = PickIntercept::secondsToPlane(-1.0f, 200.0f);
  TEST_ASSERT_TRUE(late < 0.0f);
}
