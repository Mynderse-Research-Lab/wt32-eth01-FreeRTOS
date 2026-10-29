// Host tests for belt tracking, side-grip geometry, strike timing, and path-budget parity.

#include "unity.h"

#include "pick_intercept.h"

#include <cmath>

void setUp(void) {}
void tearDown(void) {}

static PickIntercept::CellGeom cell18650() {
  return PickIntercept::CellGeom{65.0f, 18.3f, 18.3f};
}

static PickIntercept::CellGeom cell21700() {
  return PickIntercept::CellGeom{70.0f, 21.2f, 21.2f};
}

static PickIntercept::ConveyorFrame frameAt(int64_t gantry_us, uint32_t ts, float speed,
                                            float disp_m, uint8_t seq) {
  PickIntercept::ConveyorFrame frame;
  frame.gantry_us = gantry_us;
  frame.timestamp_us = ts;
  frame.speed_mm_s = speed;
  frame.displacement_m = disp_m;
  frame.sequence = seq;
  return frame;
}

static PickIntercept::MotionLimits limits(float speed = 2000.0f, float accel = 3000.0f,
                                          float z_cap = 1000.0f) {
  PickIntercept::MotionLimits m;
  m.speed = speed;
  m.accel = accel;
  m.decel = accel;
  m.theta_cap = 360.0f;
  m.theta_accel = 1800.0f;
  m.theta_decel = 1800.0f;
  m.z_cap = z_cap;
  m.theta_start_frac = 0.0f;
  return m;
}

static PickIntercept::StrikeTiming timingFor(PickIntercept::Clearance mode, float side_gap,
                                             float along) {
  PickIntercept::StrikeTiming t;
  t.mode = mode;
  t.side_gap_mm = side_gap;
  t.along_factor = along;
  t.jaw_mm_s = 210.0f;
  t.valve_s = 0.020f;
  t.guard_s = 0.015f;
  t.z_top_s = 0.050f;
  t.z_tail_s = 0.100f;
  return t;
}

// ---------------------------------------------------------------------------
// Belt clock
// ---------------------------------------------------------------------------

static void test_position_reanchors_on_displacement(void) {
  PickIntercept::BeltClock belt;
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                    static_cast<int>(PickIntercept::noteConveyor(
                        belt, frameAt(1000000, 1000000, 200.0f, 1.0f, 1))));
  const double mid = PickIntercept::beltPositionMm(belt, 1500000);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1100.0f, static_cast<float>(mid));

  // 1 s later the encoder says +500 mm, not the 200 mm the old speed would add.
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                    static_cast<int>(PickIntercept::noteConveyor(
                        belt, frameAt(2000000, 2000000, 200.0f, 1.5f, 2))));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1500.0f, static_cast<float>(belt.displacement_mm));
  const double later = PickIntercept::beltPositionMm(belt, 2500000);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1600.0f, static_cast<float>(later));
}

static void test_timestamp_wrap_is_not_a_restart(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 0xFFFFF000u, 1000.0f, 0.0f, 1));
  const auto note =
      PickIntercept::noteConveyor(belt, frameAt(1000 + 8192, 0x1000u, 500.0f, 0.01f, 2));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_TRUE(belt.conveyor_us > 0xFFFFF000LL);
}

static void test_sequence_wrap_is_not_a_restart(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 1.0f, 255));
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 1.01f, 0));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1010.0f, static_cast<float>(belt.displacement_mm));
}

static void test_displacement_backward_restarts_and_bumps_epoch(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 2.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(0, belt.epoch);
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 0.1f, 11));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Restart), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 100.0f, static_cast<float>(belt.displacement_mm));
  TEST_ASSERT_EQUAL_UINT32(1, belt.epoch);
}

static void test_sequence_reset_restarts(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000, 1000, 200.0f, 1.0f, 40));
  const auto note = PickIntercept::noteConveyor(belt, frameAt(2000, 2000, 200.0f, 1.01f, 0));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Restart), static_cast<int>(note));
}

static void test_min_delay_anchors_at_conveyor_time(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000000, 1000000, 200.0f, 0.0f, 1));
  TEST_ASSERT_EQUAL_INT64(0, belt.offset_us);
  // This frame sat in a queue for 750 ms: gantry time jumped more than conveyor time.
  PickIntercept::noteConveyor(belt, frameAt(2000000, 1250000, 200.0f, 0.05f, 2));
  TEST_ASSERT_EQUAL_INT64(0, belt.offset_us);
  TEST_ASSERT_EQUAL_INT64(1250000, belt.anchor_gantry_us);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 50.0f, static_cast<float>(belt.displacement_mm));
  // Anchored at receipt this would read 50 mm; the belt has really moved on.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 200.0f,
                           static_cast<float>(PickIntercept::beltPositionMm(belt, 2000000)));
}

static void test_stale_after_two_missed_frames(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(1000000, 1000000, 200.0f, 0.0f, 1));
  TEST_ASSERT_FALSE(PickIntercept::beltStale(belt, 1000000 + 100000));
  TEST_ASSERT_TRUE(PickIntercept::beltStale(belt, 1000000 + PickIntercept::kStaleAfterUs + 1));
}

static void test_capture_subtracts_latency_until_displacement_is_valid(void) {
  PickIntercept::BeltClock belt;
  PickIntercept::noteConveyor(belt, frameAt(2000000, 2000000, 200.0f, 1.0f, 1));
  const double from_latency =
      PickIntercept::capturePositionMm(belt, 2000000, 100000, false, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 980.0f, static_cast<float>(from_latency));
  const double from_flag =
      PickIntercept::capturePositionMm(belt, 2000000, 100000, true, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 500.0f, static_cast<float>(from_flag));
}

static void test_flat_displacement_falls_back_to_speed(void) {
  PickIntercept::BeltClock belt;
  int64_t t = 1000000;
  PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, 1));
  PickIntercept::BeltNote note = PickIntercept::BeltNote::Anchored;
  for (uint8_t seq = 2; seq <= 4; ++seq) {
    t += 250000;
    note = PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, seq));
    if (seq < 4) {
      TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored),
                        static_cast<int>(note));
    }
  }
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Fallback), static_cast<int>(note));
  TEST_ASSERT_TRUE(belt.fallback);
  // Three flat frames at 200 mm/s = 150 mm integrated; the position is continuous.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1150.0f, static_cast<float>(belt.displacement_mm));
  t += 250000;
  note = PickIntercept::noteConveyor(belt, frameAt(t, static_cast<uint32_t>(t), 200.0f, 1.0f, 5));
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::BeltNote::Anchored), static_cast<int>(note));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1200.0f, static_cast<float>(belt.displacement_mm));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 700.0f, static_cast<float>(PickIntercept::capturePositionMm(
                                               belt, t, 0, true, 0.5f)));
}

// ---------------------------------------------------------------------------
// Side-grip geometry
// ---------------------------------------------------------------------------

static void test_side_jaw_sweep_is_perpendicular_and_short(void) {
  for (int deg = -90; deg <= 90; deg += 5) {
    const float cell = static_cast<float>(deg);
    const PickIntercept::JawChoice jaw =
        PickIntercept::chooseSideJaw(0.0f, cell, 0.0f, -180.0f, 180.0f);
    TEST_ASSERT_TRUE(jaw.ok);
    const float rel = std::remainder(jaw.joint_deg - cell, 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, rel);
    TEST_ASSERT_TRUE(jaw.travel_deg <= 90.0f + 1e-3f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, std::fabs(jaw.joint_deg), jaw.travel_deg);
    const PickIntercept::JawChoice offset =
        PickIntercept::chooseSideJaw(0.0f, cell, 10.0f, -180.0f, 180.0f);
    TEST_ASSERT_TRUE(offset.ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
                             std::remainder(offset.joint_deg - cell - 10.0f, 180.0f));
  }
}

static void test_full_width_is_a_pick_and_grasp_is_half_height(void) {
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
      0.0f, PickIntercept::closeAxisDeg(jaw.joint_deg), full, f, false);
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
      0.0f, PickIntercept::closeAxisDeg(jaw.joint_deg), mid, f, true);
  const auto g60 = PickIntercept::classifyClearance(
      0.0f, PickIntercept::closeAxisDeg(jaw.joint_deg), wider, f, true);
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
      0.0f, PickIntercept::closeAxisDeg(jaw.joint_deg), over, f, true);
  TEST_ASSERT_EQUAL(static_cast<int>(PickIntercept::Clearance::Skip),
                    static_cast<int>(bad.mode));
  TEST_ASSERT_EQUAL_STRING("cell wider than jaws", bad.reason);
}

static void test_command_theta_quantizes_to_a_tenth(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, Gantry::Path::commandThetaDeg(0.0f, 0.05f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, Gantry::Path::commandThetaDeg(0.0f, 0.14f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, Gantry::Path::commandThetaDeg(0.0f, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.1f, Gantry::Path::commandThetaDeg(0.0f, 10.14f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, Gantry::Path::commandThetaDeg(10.0f, 10.06f));
}

static void test_side_jaw_is_linear_and_rejects_when_nothing_fits(void) {
  const auto from_plus = PickIntercept::chooseSideJaw(170.0f, 0.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_TRUE(from_plus.ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 180.0f, from_plus.joint_deg);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, from_plus.travel_deg);
  const auto from_minus = PickIntercept::chooseSideJaw(-170.0f, 0.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -180.0f, from_minus.joint_deg);
  // 175 to -85 is a linear 260°. The equivalent joint at 95° is only 80° away.
  const auto no_wrap = PickIntercept::chooseSideJaw(175.0f, -85.0f, 0.0f, -180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 95.0f, no_wrap.joint_deg);
  const auto none = PickIntercept::chooseSideJaw(0.0f, 90.0f, 0.0f, 0.0f, 60.0f);
  TEST_ASSERT_FALSE(none.ok);
}

static void test_lane_band_is_about_29_deg_for_18650(void) {
  const PickIntercept::CellGeom cell = cell18650();
  const PickIntercept::FingerGeom f;
  int early = 0;
  for (int deg = -90; deg <= 90; deg += 5) {
    const float c = static_cast<float>(deg);
    const auto jaw = PickIntercept::chooseSideJaw(0.0f, c, 0.0f, -180.0f, 180.0f);
    const auto g = PickIntercept::classifyClearance(
        c, PickIntercept::closeAxisDeg(jaw.joint_deg), cell, f, true);
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

static void test_lane_band_is_narrower_for_21700(void) {
  const PickIntercept::FingerGeom f;
  const float gap_18650 =
      PickIntercept::xLaneClearanceMm(-27.0f, 63.0f, cell18650(), f);
  const float gap_21700 =
      PickIntercept::xLaneClearanceMm(-27.0f, 63.0f, cell21700(), f);
  TEST_ASSERT_TRUE(gap_21700 < gap_18650);
}

static void test_dropin_window_boundary_against_belt_speed(void) {
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

static void test_grip_lead_from_jaw_travel_and_contact_speed(void) {
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

static void test_dropin_timeline_never_overlaps_a_finger_before_crossing(void) {
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

static void test_separating_axis_matches_a_known_overlap(void) {
  const PickIntercept::Rect on{0.0f, 0.0f, 10.0f, 5.0f, 0.0f};
  const PickIntercept::Rect turned{0.0f, 0.0f, 10.0f, 5.0f, 30.0f};
  TEST_ASSERT_TRUE(PickIntercept::rectsOverlap(on, turned));
  const PickIntercept::Rect clear{80.0f, 0.0f, 4.0f, 4.0f, 0.0f};
  TEST_ASSERT_FALSE(PickIntercept::rectsOverlap(on, clear));
}

static void test_slow_approach_unless_a_later_part_is_at_risk(void) {
  const auto slow = PickIntercept::approachPair(false);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 750.0f, slow.x_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, slow.theta_deg_s);
  TEST_ASSERT_FALSE(slow.fast);
  const auto fast = PickIntercept::approachPair(true);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2000.0f, fast.x_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 270.0f, fast.theta_deg_s);
  TEST_ASSERT_TRUE(fast.fast);
  TEST_ASSERT_TRUE(PickIntercept::laterPartNeedsFastApproach(1.0f, 1.2f, 0.5f));
  TEST_ASSERT_FALSE(PickIntercept::laterPartNeedsFastApproach(1.0f, 2.0f, 0.5f));
  TEST_ASSERT_FALSE(PickIntercept::laterPartNeedsFastApproach(1.0f, -0.1f, 0.5f));

  auto m = limits(PickIntercept::kApproachSlowMmS, 3000.0f, PickIntercept::kZCriticalMmS);
  m.theta_cap = PickIntercept::kApproachSlowDegS;
  const auto budget =
      PickIntercept::budgetPick(0.0f, 35.7f, 0.0f, 200.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 750.0f, budget.approach_speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 950.0f, budget.descend_speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-3f, PickIntercept::profileSeconds(105.4f - 35.7f, PickIntercept::kZCriticalMmS, m.accel, m.decel),
      budget.z_motion_s);
  const float at_cap = PickIntercept::profileSeconds(200.0f, PickIntercept::kZCriticalMmS, m.accel, m.decel);
  const float at_approach =
      PickIntercept::profileSeconds(200.0f, PickIntercept::kApproachSlowMmS, m.accel, m.decel);
  TEST_ASSERT_TRUE(at_cap < at_approach);
}

static void test_collision_envelope_blocks_deep_z_over_the_conveyor(void) {
  TEST_ASSERT_TRUE(PickIntercept::inConveyorCollision(120.0f, 120.0f, 95.0f, 115.0f));
  TEST_ASSERT_FALSE(PickIntercept::inConveyorCollision(120.0f, 105.4f, 95.0f, 115.0f));
  TEST_ASSERT_FALSE(PickIntercept::inConveyorCollision(10.0f, 140.0f, 95.0f, 115.0f));
}

// ---------------------------------------------------------------------------
// Budget and schedule
// ---------------------------------------------------------------------------

static void test_profile_end_matches_time_along(void) {
  const float length = 70.0f;
  const float speed = 1000.0f;
  const float accel = 3000.0f;
  const float total = PickIntercept::profileSeconds(length, speed, accel, accel);
  const float along =
      PickIntercept::timeAlongProfile(length, length, speed, accel, accel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, total, along);
  TEST_ASSERT_TRUE(total > 0.2f && total < 0.5f);
}

static void test_in_band_budget_matches_path_segment(void) {
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

static void test_retract_into_band_defers_x(void) {
  const auto m = limits(1000.0f);
  const auto budget =
      PickIntercept::budgetPick(0.0f, 80.0f, 0.0f, 100.0f, 35.7f, 35.7f, 105.4f, 0.0f, m);
  const float t_z = PickIntercept::profileSeconds(80.0f - 35.7f, 1000.0f, m.accel, m.decel) +
                    PickIntercept::kPreloadS;
  const float t_x =
      PickIntercept::profileSeconds(100.0f, 1000.0f, m.accel, m.decel) + PickIntercept::kPreloadS;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, t_z + t_x + PickIntercept::kSettleS, budget.approach_s);
}

static void test_z_cap_slows_combined_segments(void) {
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

static void test_theta_budget_counts_rotary_phases_from_move_start(void) {
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

static void test_theta_budget_uses_linear_delta(void) {
  const auto m = limits();
  const auto short_turn =
      PickIntercept::budgetPick(0.0f, 35.7f, 170.0f, 10.0f, 35.7f, 35.7f, 105.4f, 90.0f, m);
  const auto long_turn =
      PickIntercept::budgetPick(0.0f, 35.7f, 170.0f, 10.0f, 35.7f, 35.7f, 105.4f, -90.0f, m);
  TEST_ASSERT_TRUE(long_turn.theta_s > short_turn.theta_s + 0.3f);
}

static void test_hover_is_budgeted_before_the_dropin_tail(void) {
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

static void test_jaw_is_frozen_once_z_leaves_the_band(void) {
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

static void test_lane_strike_slack_and_dropin_release(void) {
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

static void test_retime_follows_a_speed_change(void) {
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

static void test_epoch_rejects_parts_and_strikes_from_before_a_restart(void) {
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

static void test_arrival_is_camera_relative(void) {
  // Cell at the camera centre, camera 711 mm upstream of the jaw plane.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1711.0f,
                           static_cast<float>(PickIntercept::arriveBeltMm(1000.0, 0.0f, -711.0f, 0.0f)));
  // 11 mm downstream of the camera centre arrives 11 mm sooner.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1700.0f,
                           static_cast<float>(PickIntercept::arriveBeltMm(1000.0, 0.0f, -711.0f, 11.0f)));
}

static void test_zero_speed_does_not_invent_an_arrival(void) {
  const float wait = PickIntercept::secondsToPlane(711.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, PickIntercept::kWaitS, wait);
  const float late = PickIntercept::secondsToPlane(-1.0f, 200.0f);
  TEST_ASSERT_TRUE(late < 0.0f);
}

// ---------------------------------------------------------------------------
// Neighbours and tracker
// ---------------------------------------------------------------------------

static void test_neighbour_crossing_a_finger_delays_the_fingers(void) {
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

static void test_tracker_dedupes_and_edf_takes_the_earliest(void) {
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

static void test_tracker_matches_item_id_and_remembers_taken_cells(void) {
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

static void test_development_profiles_grasp_from_the_object_top(void) {
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
  TEST_ASSERT_TRUE(PickIntercept::devProfile(1)->grasp_joint_z_mm > PickIntercept::kMeasuredStrokeMm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(2)->grasp_joint_z_mm > PickIntercept::kMeasuredStrokeMm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(6)->grasp_joint_z_mm > PickIntercept::kMeasuredStrokeMm);
  TEST_ASSERT_TRUE(PickIntercept::devProfile(7)->grasp_joint_z_mm > PickIntercept::kMeasuredStrokeMm);
  TEST_ASSERT_NULL(PickIntercept::devProfile(0));
  TEST_ASSERT_NULL(PickIntercept::devProfile(14));
  TEST_ASSERT_EQUAL_STRING("class", PickIntercept::classPickReject(14));
}

static void test_vision_contract_rejects_each_missing_field(void) {
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

static void test_follower_pace_uses_the_post_bin_pose(void) {
  const auto cycle = PickIntercept::postBinCycle(100.0f, 200.0f, 90.0f, 10.0f, 35.7f, 90.8f, 90.0f,
                                                 0.19f, 0.19f, 1000.0f, 3000.0f, 3000.0f, 1800.0f,
                                                 1800.0f);
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

static void test_jaw_override_survives_a_supplied_width(void) {
  auto t = timingFor(PickIntercept::Clearance::EarlyZ, 20.0f, 0.0f);
  t.jaw_override_s = 0.080f;
  const auto leads = PickIntercept::strikeLeads(t, 400.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.100f, leads.grip_lead_s);
  t.jaw_override_s = 0.0f;
  const auto derived = PickIntercept::strikeLeads(t, 400.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.020f + 20.0f / 210.0f, derived.grip_lead_s);
}

static void test_contact_lead_pulls_depth_ahead_of_center_arrival(void) {
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

static void test_neighbour_occupancy_covers_the_retract(void) {
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
  RUN_TEST(test_collision_envelope_blocks_deep_z_over_the_conveyor);
  RUN_TEST(test_profile_end_matches_time_along);
  RUN_TEST(test_in_band_budget_matches_path_segment);
  RUN_TEST(test_retract_into_band_defers_x);
  RUN_TEST(test_z_cap_slows_combined_segments);
  RUN_TEST(test_theta_budget_counts_rotary_phases_from_move_start);
  RUN_TEST(test_theta_budget_uses_linear_delta);
  RUN_TEST(test_hover_is_budgeted_before_the_dropin_tail);
  RUN_TEST(test_jaw_is_frozen_once_z_leaves_the_band);
  RUN_TEST(test_lane_strike_slack_and_dropin_release);
  RUN_TEST(test_retime_follows_a_speed_change);
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
