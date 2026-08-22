/**
 * @file test_path_profile.cpp
 * @brief Host tests for 2-D X-Z path profile decompose + segment planning.
 */

#include "unity.h"

#include "GantryPathProfile.h"
#include "GantryThetaOrigin.h"

#include <cmath>

using Gantry::Path::AxisComponents;
using Gantry::Path::PathProfile;
using Gantry::Path::PathSegment;
using Gantry::Path::bandCeilingFromZMinus;
using Gantry::Path::decompose;
using Gantry::Path::kAxisEpsMm;
using Gantry::Path::planSegments;
using Gantry::Path::planThetaWindow;
using Gantry::Path::segmentEndpointsReached;
using Gantry::Path::zInTraverseBand;
using Gantry::Path::kThetaEndFrac;
using Gantry::Path::kThetaStartFrac;
using Gantry::Path::ThetaWindow;

void setUp(void) {}
void tearDown(void) {}

static void test_decompose_pure_x(void) {
  const PathProfile p{100.0f, 3000.0f, 3000.0f};
  const AxisComponents c = decompose(50.0f, 0.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, c.x_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3000.0f, c.x_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3000.0f, c.x_decel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.z_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.z_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.z_decel);
}

static void test_decompose_pure_z(void) {
  const PathProfile p{100.0f, 3000.0f, 2000.0f};
  const AxisComponents c = decompose(0.0f, -40.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.x_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, c.z_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3000.0f, c.z_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, c.z_decel);
}

static void test_decompose_45_degree_resultant(void) {
  const PathProfile p{100.0f, 3000.0f, 3000.0f};
  const AxisComponents c = decompose(30.0f, 30.0f, p);
  const float inv_sqrt2 = 1.0f / std::sqrt(2.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f * inv_sqrt2, c.x_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f * inv_sqrt2, c.z_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 3000.0f * inv_sqrt2, c.x_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 3000.0f * inv_sqrt2, c.z_accel);

  const float v_res = std::hypot(c.x_speed, c.z_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, v_res);
}

static void test_decompose_components_never_exceed_resultant(void) {
  const PathProfile p{80.0f, 2500.0f, 1800.0f};
  const AxisComponents c = decompose(120.0f, 40.0f, p);
  TEST_ASSERT_TRUE(c.x_speed <= p.speed_mm_per_s + 0.01f);
  TEST_ASSERT_TRUE(c.z_speed <= p.speed_mm_per_s + 0.01f);
  TEST_ASSERT_TRUE(c.x_accel <= p.accel_mm_per_s2 + 0.01f);
  TEST_ASSERT_TRUE(c.z_accel <= p.accel_mm_per_s2 + 0.01f);
  TEST_ASSERT_TRUE(c.x_decel <= p.decel_mm_per_s2 + 0.01f);
  TEST_ASSERT_TRUE(c.z_decel <= p.decel_mm_per_s2 + 0.01f);
}

static void test_decompose_zero_profile_preserved(void) {
  const PathProfile p{50.0f, 0.0f, 0.0f};
  const AxisComponents c = decompose(10.0f, 10.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.x_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.z_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.x_decel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, c.z_decel);
  TEST_ASSERT_TRUE(c.x_speed > 0.0f);
  TEST_ASSERT_TRUE(c.z_speed > 0.0f);
}

static void assert_x_endpoints_in_band(const PathSegment* segs, size_t n,
                                       float ceiling_z) {
  for (size_t i = 0; i < n; ++i) {
    if (segs[i].move_x) {
      TEST_ASSERT_TRUE_MESSAGE(
          segs[i].z_mm <= ceiling_z + kAxisEpsMm,
          "X Absolute endpoint must be in the SAFE_Z band");
    }
  }
}

static void test_plan_pure_z(void) {
  PathSegment out[3];
  const size_t n = planSegments(100.0f, 60.0f, 100.0f, 10.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_FALSE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, out[0].z_mm);
}

static void test_plan_both_in_band(void) {
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 10.0f, 200.0f, 20.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out[0].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_plan_start_above_end_in_band(void) {
  // No via at SAFE_Z: one combined segment; X starts when Z is in-band.
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 130.0f, 200.0f, 20.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out[0].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_plan_retract_home_no_safe_z_via(void) {
  PathSegment out[3];
  const size_t n = planSegments(350.0f, 130.0f, 0.0f, 0.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out[0].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_plan_start_in_band_end_above(void) {
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 10.0f, 200.0f, 130.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(2u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_FALSE(out[1].move_x);
  TEST_ASSERT_TRUE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 130.0f, out[1].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_plan_outbound_from_origin(void) {
  PathSegment out[3];
  const size_t n = planSegments(0.0f, 0.0f, 350.0f, 130.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(2u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 350.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_FALSE(out[1].move_x);
  TEST_ASSERT_TRUE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 130.0f, out[1].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_plan_both_above_band(void) {
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 100.0f, 200.0f, 120.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(3u, n);
  TEST_ASSERT_FALSE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_TRUE(out[1].move_x);
  TEST_ASSERT_FALSE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[1].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[1].z_mm);
  TEST_ASSERT_FALSE(out[2].move_x);
  TEST_ASSERT_TRUE(out[2].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, out[2].z_mm);
  assert_x_endpoints_in_band(out, n, 30.0f);
}

static void test_band_ceiling_from_zminus(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, bandCeilingFromZMinus(0.0f, 30.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, bandCeilingFromZMinus(10.0f, 30.0f));
}

static void test_z_in_traverse_band(void) {
  const float ceiling_z = 30.0f;
  TEST_ASSERT_TRUE(zInTraverseBand(0.0f, ceiling_z));
  TEST_ASSERT_TRUE(zInTraverseBand(30.0f, ceiling_z));
  TEST_ASSERT_TRUE(zInTraverseBand(30.5f, ceiling_z));  // within 0.5 mm eps
  TEST_ASSERT_FALSE(zInTraverseBand(31.0f, ceiling_z));
  TEST_ASSERT_FALSE(zInTraverseBand(148.0f, ceiling_z));
}

static void test_segment_endpoints_reached(void) {
  PathSegment seg{35.0f, 30.0f, true, false};
  TEST_ASSERT_TRUE(segmentEndpointsReached(35.0f, 30.0f, seg, 1.0f));
  TEST_ASSERT_TRUE(segmentEndpointsReached(35.4f, 0.0f, seg, 1.0f));  // Z not armed
  TEST_ASSERT_FALSE(segmentEndpointsReached(0.0f, 30.0f, seg, 1.0f));  // X miss
  PathSegment both{35.0f, 0.0f, true, true};
  TEST_ASSERT_FALSE(segmentEndpointsReached(35.0f, 30.0f, both, 1.0f));
  TEST_ASSERT_TRUE(segmentEndpointsReached(35.0f, 0.2f, both, 1.0f));
}

static void test_plan_test_cycle_legs(void) {
  PathSegment out[3];
  const float ceiling = 30.0f;
  size_t n = planSegments(35.0f, 30.0f, 220.0f, 8.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  n = planSegments(220.0f, 8.0f, 220.0f, 130.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_FALSE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  n = planSegments(220.0f, 130.0f, 0.0f, 0.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  n = planSegments(0.0f, 0.0f, 350.0f, 130.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(2u, n);
  n = planSegments(350.0f, 130.0f, 80.0f, 110.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(3u, n);
  n = planSegments(80.0f, 110.0f, 0.0f, 0.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  // G-I are theta-only at (0,0): X-Z planner is a no-op.
  n = planSegments(0.0f, 0.0f, 0.0f, 0.0f, ceiling, out);
  TEST_ASSERT_EQUAL_UINT(0u, n);
}

static void test_plan_noop(void) {
  PathSegment out[3];
  const size_t n = planSegments(35.0f, 40.0f, 35.0f, 40.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(0u, n);
}

static void test_theta_window_nominal_speed(void) {
  // L=100 mm, V=100 mm/s → T_seg=1 s; window 0.5 s; |dTheta|=90 → 180 deg/s.
  const ThetaWindow w = planThetaWindow(100.0f, 100.0f, 90.0f, 3600.0f);
  TEST_ASSERT_TRUE(w.runnable);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, kThetaStartFrac, w.start_frac);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, w.speed_deg_per_s);
  TEST_ASSERT_FALSE(w.speed_clamped);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.5f, kThetaEndFrac - kThetaStartFrac);
}

static void test_theta_window_zero_delta_not_runnable(void) {
  const ThetaWindow w = planThetaWindow(100.0f, 100.0f, 0.0f, 3600.0f);
  TEST_ASSERT_FALSE(w.runnable);
  TEST_ASSERT_FALSE(w.speed_clamped);
}

static void test_theta_window_clamped_to_cap(void) {
  // Same T_seg=1 s, 90 deg needs 180 deg/s; cap 10 → clamped.
  const ThetaWindow w = planThetaWindow(100.0f, 100.0f, 90.0f, 10.0f);
  TEST_ASSERT_TRUE(w.runnable);
  TEST_ASSERT_TRUE(w.speed_clamped);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, w.speed_deg_per_s);
}

static void test_theta_window_hard_limit_stroke(void) {
  // Full firmware envelope (180 deg) on an A-like in-band traverse at 50 mm/s
  // stays under the 3600 deg/s kinematic cap.
  const float a_len = std::hypot(220.0f - 35.0f, 8.0f - 30.0f);
  const ThetaWindow w = planThetaWindow(a_len, 50.0f, 180.0f, 3600.0f);
  TEST_ASSERT_TRUE(w.runnable);
  TEST_ASSERT_FALSE(w.speed_clamped);
  TEST_ASSERT_TRUE(w.speed_deg_per_s > 0.0f);
  TEST_ASSERT_TRUE(w.speed_deg_per_s < 3600.0f);
}

static void test_decompose_at_axis_eps_boundary(void) {
  const PathProfile p{100.0f, 3000.0f, 3000.0f};
  // L == kAxisEpsMm: scaleComponent returns the full magnitude (no split).
  const AxisComponents c = decompose(kAxisEpsMm, 0.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, c.x_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, c.z_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 3000.0f, c.x_accel);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 3000.0f, c.z_accel);

  const AxisComponents below = decompose(kAxisEpsMm * 0.5f, 0.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, below.x_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, below.z_speed);
}

static void test_plan_straddle_ceiling_within_eps(void) {
  // Start 0.02 mm above the 30 mm ceiling, end 0.02 mm below; |dZ| < eps.
  // Planner must treat this as in-band (start_above is false).
  PathSegment out[3];
  const size_t n = planSegments(0.0f, 30.02f, 100.0f, 29.98f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_FALSE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, out[0].x_mm);
}

static void test_theta_origin_aligned_uses_drive_identity(void) {
  const auto p = Gantry::ThetaOrigin::plan(0.05f, -180.0f, 180.0f, -180.0f,
                                           180.0f);
  TEST_ASSERT_TRUE(p.aligned);
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.zero_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -179.5f, p.joint_min_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 179.5f, p.joint_max_deg);
}

static void test_theta_origin_unaligned_shrinks_positive_travel(void) {
  const auto p = Gantry::ThetaOrigin::plan(178.5127f, -180.0f, 180.0f, -180.0f,
                                           180.0f);
  TEST_ASSERT_FALSE(p.aligned);
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 178.5127f, p.zero_deg);
  TEST_ASSERT_TRUE(p.joint_max_deg < 2.0f);
  TEST_ASSERT_TRUE(p.joint_max_deg > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -180.0f, p.joint_min_deg);
}

static void test_theta_window_no_divide_by_zero(void) {
  const ThetaWindow zlen = planThetaWindow(0.0f, 100.0f, 90.0f, 3600.0f);
  TEST_ASSERT_FALSE(zlen.runnable);
  const ThetaWindow zspd = planThetaWindow(100.0f, 0.0f, 90.0f, 3600.0f);
  TEST_ASSERT_FALSE(zspd.runnable);
  const ThetaWindow neg = planThetaWindow(100.0f, -50.0f, 90.0f, 3600.0f);
  TEST_ASSERT_FALSE(neg.runnable);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_decompose_pure_x);
  RUN_TEST(test_decompose_pure_z);
  RUN_TEST(test_decompose_45_degree_resultant);
  RUN_TEST(test_decompose_components_never_exceed_resultant);
  RUN_TEST(test_decompose_zero_profile_preserved);
  RUN_TEST(test_decompose_at_axis_eps_boundary);
  RUN_TEST(test_plan_straddle_ceiling_within_eps);
  RUN_TEST(test_plan_pure_z);
  RUN_TEST(test_plan_both_in_band);
  RUN_TEST(test_plan_start_above_end_in_band);
  RUN_TEST(test_plan_retract_home_no_safe_z_via);
  RUN_TEST(test_plan_start_in_band_end_above);
  RUN_TEST(test_plan_outbound_from_origin);
  RUN_TEST(test_plan_both_above_band);
  RUN_TEST(test_band_ceiling_from_zminus);
  RUN_TEST(test_z_in_traverse_band);
  RUN_TEST(test_segment_endpoints_reached);
  RUN_TEST(test_plan_noop);
  RUN_TEST(test_plan_test_cycle_legs);
  RUN_TEST(test_theta_window_nominal_speed);
  RUN_TEST(test_theta_window_zero_delta_not_runnable);
  RUN_TEST(test_theta_window_clamped_to_cap);
  RUN_TEST(test_theta_window_hard_limit_stroke);
  RUN_TEST(test_theta_origin_aligned_uses_drive_identity);
  RUN_TEST(test_theta_origin_unaligned_shrinks_positive_travel);
  RUN_TEST(test_theta_window_no_divide_by_zero);
  return UNITY_END();
}
