/**
 * @file test_pick_intercept_geometry.cpp
 * @brief Side-grip geometry host tests.
 */

#include "test_pick_intercept_internal.h"

using namespace pick_intercept_test;

// ---------------------------------------------------------------------------
// Side-grip geometry
// ---------------------------------------------------------------------------

void test_side_jaw_sweep_is_perpendicular_and_short(void) {
  for (int deg = -90; deg <= 90; deg += 5) {
    const float cell = static_cast<float>(deg);
    const PickIntercept::JawChoice jaw =
        PickIntercept::chooseSideJaw(0.0f, cell, 0.0f, -180.0f, 180.0f);
    TEST_ASSERT_TRUE(jaw.ok);
    // Pads along the long axis: pose = cell (mod 180), joint = pose - 90°.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, std::remainder(jaw.pose_deg - cell, 180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, jaw.pose_deg - jaw.joint_deg);
    TEST_ASSERT_TRUE(jaw.travel_deg <= 90.0f + 1e-3f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, std::fabs(jaw.joint_deg), jaw.travel_deg);
    const PickIntercept::JawChoice offset =
        PickIntercept::chooseSideJaw(0.0f, cell, 10.0f, -180.0f, 180.0f);
    TEST_ASSERT_TRUE(offset.ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
                             std::remainder(offset.pose_deg - cell - 10.0f, 180.0f));
  }
}

void test_full_width_is_a_pick_and_grasp_is_half_height(void) {
  const PickIntercept::FingerGeom f;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 95.0f, PickIntercept::graspJointZMm(115.0f, 40.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.4f, PickIntercept::graspJointZMm(115.0f, 80.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 115.0f, PickIntercept::graspJointZMm(115.0f, 0.0f));

  const auto sized = PickIntercept::applyVisionSize(cell18650(), 80.0f, 120.0f,
                                                    40.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, sized.length_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 40.0f, sized.height_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 18.3f, sized.class_cross_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 95.0f, PickIntercept::graspJointZMm(115.0f, sized.height_mm));
  TEST_ASSERT_TRUE(std::fabs(PickIntercept::graspJointZMm(115.0f, sized.length_mm) - 95.0f) > 1.0f);

  PickIntercept::CellGeom full = sized;
  full.length_mm = 120.0f;
  const auto jaw = PickIntercept::chooseSideJaw(0.0f, 0.0f, 0.0f, -180.0f, 180.0f);
  const auto fit = PickIntercept::classifyClearance(
      0.0f, PickIntercept::closeAxisDeg(jaw.pose_deg), full, f, false);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::Skip),
                    static_cast<int>(fit.mode));
  TEST_ASSERT_EQUAL_STRING("late Z off", fit.reason);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, fit.side_gap_mm);
  TEST_ASSERT_TRUE(fit.x_clearance_mm < PickIntercept::kLaneMarginMm);
  const auto leads = PickIntercept::strikeLeads(
      timingFor(PickIntercept::Clearance::EarlyZ, 0.0f, 0.0f), 400.0f);
  TEST_ASSERT_TRUE(leads.ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, leads.close_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.020f, leads.grip_lead_s);

  const float close_40 = PickIntercept::jawCloseTravelMm(f.open_mm, 40.0f) / f.jaw_mm_s;
  const float close_60 = PickIntercept::jawCloseTravelMm(f.open_mm, 60.0f) / f.jaw_mm_s;
  TEST_ASSERT_TRUE(close_60 < close_40);
  PickIntercept::CellGeom mid = full;
  mid.width_mm = 40.0f;
  PickIntercept::CellGeom wider = full;
  wider.width_mm = 60.0f;
  const auto g40 = PickIntercept::classifyClearance(
      0.0f, PickIntercept::closeAxisDeg(jaw.pose_deg), mid, f, true);
  const auto g60 = PickIntercept::classifyClearance(
      0.0f, PickIntercept::closeAxisDeg(jaw.pose_deg), wider, f, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, g40.side_gap_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, g60.side_gap_mm);
  const auto slow = PickIntercept::strikeLeads(
      timingFor(PickIntercept::Clearance::EarlyZ, g40.side_gap_mm, 0.0f), 200.0f);
  const auto fast_close = PickIntercept::strikeLeads(
      timingFor(PickIntercept::Clearance::EarlyZ, g60.side_gap_mm, 0.0f), 200.0f);
  TEST_ASSERT_TRUE(fast_close.close_s < slow.close_s);

  TEST_ASSERT_EQUAL_STRING("width",
                           PickIntercept::visionSizeReject(81.0f, 120.0f, 40.0f, 80.0f));
  TEST_ASSERT_EQUAL_STRING("width", PickIntercept::visionSizeReject(0.0f, 0.0f, 0.0f, 80.0f));
  TEST_ASSERT_NULL(PickIntercept::visionSizeReject(80.0f, 120.0f, 40.0f, 80.0f));
  const auto across_width = PickIntercept::gripAcrossWidth(65.0f, 32.0f, 30.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 32.0f, across_width.width_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 65.0f, across_width.length_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, across_width.theta_deg);
  TEST_ASSERT_NULL(PickIntercept::visionSizeReject(65.0f, 32.0f, 32.0f, 80.0f));
  PickIntercept::CellGeom over = full;
  over.width_mm = 81.0f;
  const auto bad = PickIntercept::classifyClearance(
      0.0f, PickIntercept::closeAxisDeg(jaw.pose_deg), over, f, true);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::Skip),
                    static_cast<int>(bad.mode));
  TEST_ASSERT_EQUAL_STRING("cell wider than jaws", bad.reason);
}

void test_command_theta_quantizes_to_a_tenth(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, Gantry::Path::commandThetaDeg(0.0f, 0.05f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, Gantry::Path::commandThetaDeg(0.0f, 0.14f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, Gantry::Path::commandThetaDeg(0.0f, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.1f, Gantry::Path::commandThetaDeg(0.0f, 10.14f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, Gantry::Path::commandThetaDeg(10.0f, 10.06f));
}

void test_side_jaw_is_linear_and_rejects_when_nothing_fits(void) {
  // Cell along the belt (90°): joint 0° or ±180°.
  const auto from_plus = PickIntercept::chooseSideJaw(170.0f, 90.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_TRUE(from_plus.ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 180.0f, from_plus.joint_deg);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, from_plus.travel_deg);
  const auto from_minus = PickIntercept::chooseSideJaw(-170.0f, 90.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -180.0f, from_minus.joint_deg);
  // Cell 5° gives joints -85° and 95°. From 175° the linear move to -85° is 260°; 95° is 80°.
  const auto no_wrap = PickIntercept::chooseSideJaw(175.0f, 5.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 95.0f, no_wrap.joint_deg);
  // Cell across the belt (0°) needs joint ±90°, outside 0..60.
  const auto none = PickIntercept::chooseSideJaw(0.0f, 0.0f, 0.0f, 0.0f, 60.0f);
  TEST_ASSERT_FALSE(none.ok);
}

void test_lane_band_is_about_29_deg_for_18650(void) {
  const PickIntercept::CellGeom cell = cell18650();
  const PickIntercept::FingerGeom f;
  int early = 0;
  for (int deg = -90; deg <= 90; deg += 5) {
    const float c = static_cast<float>(deg);
    const auto jaw = PickIntercept::chooseSideJaw(0.0f, c, 0.0f, -180.0f, 180.0f);
    const auto g = PickIntercept::classifyClearance(
        c, PickIntercept::closeAxisDeg(jaw.pose_deg), cell, f, true);
    // 50 mm fingers. Along the belt (90°) the cell sits between the pads. Across the belt (0°) it does not.
    if (std::fabs(std::fabs(c) - 90.0f) < 1.0f) {
      TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::EarlyZ),
                        static_cast<int>(g.mode));
    }
    if (std::fabs(c) <= 45.0f) {
      TEST_ASSERT_TRUE(g.mode != PickIntercept::Clearance::EarlyZ);
    }
    if (g.mode == PickIntercept::Clearance::EarlyZ) {
      ++early;
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f * (f.open_mm - cell.height_mm), g.side_gap_mm);
  }
  TEST_ASSERT_TRUE(early >= 3);
  TEST_ASSERT_TRUE(PickIntercept::xLaneClearanceMm(90.0f, 180.0f, cell, f) >=
                   PickIntercept::kLaneMarginMm);
  TEST_ASSERT_TRUE(PickIntercept::xLaneClearanceMm(0.0f, 90.0f, cell, f) <
                   PickIntercept::kLaneMarginMm);
  const auto along = PickIntercept::classifyClearance(90.0f, 180.0f, cell, f, false);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::EarlyZ),
                    static_cast<int>(along.mode));
  const auto off = PickIntercept::classifyClearance(0.0f, 90.0f, cell, f, false);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::Skip), static_cast<int>(off.mode));
  TEST_ASSERT_EQUAL_STRING("late Z off", off.reason);
  const auto wide =
      PickIntercept::classifyClearance(0.0f, 0.0f, PickIntercept::CellGeom{90.0f, 18.3f}, f, true);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::Skip), static_cast<int>(wide.mode));
  TEST_ASSERT_EQUAL_STRING("cell wider than jaws", wide.reason);
}

void test_lane_band_is_narrower_for_21700(void) {
  const PickIntercept::FingerGeom f;
  const float gap_18650 =
      PickIntercept::xLaneClearanceMm(-27.0f, 63.0f, cell18650(), f);
  const float gap_21700 =
      PickIntercept::xLaneClearanceMm(-27.0f, 63.0f, cell21700(), f);
  TEST_ASSERT_TRUE(gap_21700 < gap_18650);
}

void test_dropin_window_boundary_against_belt_speed(void) {
  // Across the belt: the jaw axis is along Y, so the whole belt speed closes the gap.
  const float gap = 30.85f;
  const auto t = timingFor(PickIntercept::Clearance::LateZ, gap, 1.0f);
  // W = 2g/v must cover 2*(tail - top + guard) = 0.130 s -> v <= 474 mm/s.
  const auto slow = PickIntercept::strikeLeads(t, 400.0f);
  TEST_ASSERT_TRUE(slow.ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * gap / 400.0f, slow.window_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, t.guard_s + t.z_tail_s, slow.release_lead_s);
  const auto fast = PickIntercept::strikeLeads(t, 500.0f);
  TEST_ASSERT_FALSE(fast.ok);
  TEST_ASSERT_EQUAL_STRING("too fast for this angle", fast.reason);
  // A lane strike does not care about the window.
  const auto lane = PickIntercept::strikeLeads(
      timingFor(PickIntercept::Clearance::EarlyZ, gap, 0.2f), 800.0f);
  TEST_ASSERT_TRUE(lane.ok);
}

void test_grip_lead_from_jaw_travel_and_contact_speed(void) {
  const float gap = 30.85f;
  auto t = timingFor(PickIntercept::Clearance::LateZ, gap, 1.0f);
  const auto slow = PickIntercept::strikeLeads(t, 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.020f + gap / 210.0f, slow.grip_lead_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 210.0f, slow.contact_mm_s);
  // Faster than the jaws along their axis: the downstream finger meets the cell.
  const auto fast = PickIntercept::strikeLeads(t, 500.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.020f + gap / 500.0f, fast.grip_lead_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 710.0f, fast.contact_mm_s);
  t.jaw_override_s = 0.080f;
  const auto over = PickIntercept::strikeLeads(t, 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.100f, over.grip_lead_s);
}

void test_dropin_timeline_never_overlaps_a_finger_before_crossing(void) {
  const PickIntercept::CellGeom cell = cell18650();
  const PickIntercept::FingerGeom f;
  const float v = 400.0f;
  // Away from ±90° (along the belt): these across-belt orientations need the late Z tail.
  for (int deg = -90; deg <= 90; deg += 15) {
    if (std::fabs(static_cast<float>(deg)) > 45.0f) {
      continue;
    }
    const float c = static_cast<float>(deg);
    const float jaw = c + 90.0f;
    const auto g = PickIntercept::classifyClearance(c, jaw, cell, f, true);
    TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::LateZ),
                      static_cast<int>(g.mode));
    auto t = timingFor(PickIntercept::Clearance::LateZ, g.side_gap_mm, g.along_factor);
    const auto leads = PickIntercept::strikeLeads(t, v);
    TEST_ASSERT_TRUE(leads.ok);
    PickIntercept::Rect fin[2];
    PickIntercept::fingerRects(0.0f, 0.0f, jaw, f, fin);
    // Fingertips pass the cell top at release + z_top and stay down past contact.
    const float t_below = -leads.release_lead_s + t.z_top_s;
    const float t_end = 0.5f * leads.window_s - t.guard_s;
    for (float s = t_below; s <= t_end; s += 0.001f) {
      const auto r = PickIntercept::cellRect(0.0f, v * s, c, cell);
      TEST_ASSERT_FALSE(PickIntercept::rectsOverlap(r, fin[0]));
      TEST_ASSERT_FALSE(PickIntercept::rectsOverlap(r, fin[1]));
    }
  }
  // Sanity: across the belt (0° is New +X), just before the window the cell is under the upstream finger.
  PickIntercept::Rect fin[2];
  PickIntercept::fingerRects(0.0f, 0.0f, 90.0f, f, fin);
  const auto under = PickIntercept::cellRect(0.0f, -(30.85f + 4.0f), 0.0f, cell);
  TEST_ASSERT_TRUE(PickIntercept::rectsOverlap(under, fin[0]) ||
                   PickIntercept::rectsOverlap(under, fin[1]));
}

void test_separating_axis_matches_a_known_overlap(void) {
  const PickIntercept::Rect on{0.0f, 0.0f, 10.0f, 5.0f, 0.0f};
  const PickIntercept::Rect turned{0.0f, 0.0f, 10.0f, 5.0f, 30.0f};
  TEST_ASSERT_TRUE(PickIntercept::rectsOverlap(on, turned));
  const PickIntercept::Rect clear{80.0f, 0.0f, 4.0f, 4.0f, 0.0f};
  TEST_ASSERT_FALSE(PickIntercept::rectsOverlap(on, clear));
}

void test_slow_approach_unless_a_later_part_is_at_risk(void) {
  TEST_ASSERT_TRUE(PickIntercept::laterPartNeedsFastApproach(1.0f, 1.2f, 0.5f));
  TEST_ASSERT_FALSE(PickIntercept::laterPartNeedsFastApproach(1.0f, 2.0f, 0.5f));
  TEST_ASSERT_FALSE(PickIntercept::laterPartNeedsFastApproach(1.0f, -0.1f, 0.5f));

  const float slow_mm_s = 750.0f;
  const float z_cap_mm_s = 950.0f;
  auto m = limits(slow_mm_s, 3000.0f, z_cap_mm_s);
  m.theta_cap = 90.0f;
  const auto budget =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 200.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, slow_mm_s, budget.approach_speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, z_cap_mm_s, budget.descend_speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-3f, PickIntercept::profileSeconds(105.4f - 35.7f, z_cap_mm_s, m.accel, m.decel),
      budget.z_motion_s);
  const float at_cap = PickIntercept::profileSeconds(200.0f, z_cap_mm_s, m.accel, m.decel);
  const float at_approach = PickIntercept::profileSeconds(200.0f, slow_mm_s, m.accel, m.decel);
  TEST_ASSERT_TRUE(at_cap < at_approach);
}

void test_arrival_uses_the_gantry_settle_tolerances(void) {
  // Theta tolerates AF->AH coast past S-0-0057 (0.10 deg), up to 1.0 deg.
  TEST_ASSERT_TRUE(PickIntercept::jointsArrived(10.0f, 20.0f, 90.0f, 10.0f, 20.0f, 90.15f));
  TEST_ASSERT_TRUE(PickIntercept::jointsArrived(10.0f, 20.0f, 90.0f, 10.0f, 20.0f, 90.9f));
  TEST_ASSERT_FALSE(PickIntercept::jointsArrived(10.0f, 20.0f, 90.0f, 10.0f, 20.0f, 91.1f));
  TEST_ASSERT_TRUE(PickIntercept::jointsArrived(10.9f, 20.0f, 90.0f, 10.0f, 20.0f, 90.0f));
  TEST_ASSERT_FALSE(PickIntercept::jointsArrived(11.1f, 20.0f, 90.0f, 10.0f, 20.0f, 90.0f));
}

void test_collision_envelope_blocks_deep_z_over_the_conveyor(void) {
  TEST_ASSERT_TRUE(PickIntercept::inConveyorCollision(120.0f, 120.0f, 95.0f, 115.0f));
  TEST_ASSERT_FALSE(PickIntercept::inConveyorCollision(120.0f, 105.4f, 95.0f, 115.0f));
  TEST_ASSERT_FALSE(PickIntercept::inConveyorCollision(10.0f, 140.0f, 95.0f, 115.0f));
}
