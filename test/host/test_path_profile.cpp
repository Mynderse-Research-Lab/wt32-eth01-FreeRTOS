/**
 * @file test_path_profile.cpp
 * @brief Host tests for 2-D X-Z path profile decompose + segment planning.
 */

#include "unity.h"

#include "GantryPathProfile.h"

#include <cmath>

using Gantry::Path::AxisComponents;
using Gantry::Path::PathProfile;
using Gantry::Path::PathSegment;
using Gantry::Path::bandCeilingFromZMinus;
using Gantry::Path::decompose;
using Gantry::Path::kAxisEpsMm;
using Gantry::Path::planSegments;
using Gantry::Path::segmentEndpointsReached;
using Gantry::Path::zInTraverseBand;

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

static void assert_coord_endpoints_in_bottom_band(const PathSegment* segs,
                                                  size_t n, float ceiling_z,
                                                  float z0) {
  float prev_z = z0;
  for (size_t i = 0; i < n; ++i) {
    if (segs[i].move_x && segs[i].move_z) {
      TEST_ASSERT_TRUE(prev_z <= ceiling_z + kAxisEpsMm);
      TEST_ASSERT_TRUE(segs[i].z_mm <= ceiling_z + kAxisEpsMm);
    }
    if (segs[i].move_x) {
      TEST_ASSERT_TRUE(segs[i].z_mm <= ceiling_z + kAxisEpsMm);
    }
    prev_z = segs[i].z_mm;
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
  // ceiling=30. z0=10, z1=20 → one coordinated.
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 10.0f, 200.0f, 20.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(1u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out[0].z_mm);
  assert_coord_endpoints_in_bottom_band(out, n, 30.0f, 10.0f);
}

static void test_plan_start_above_end_in_band(void) {
  // z0=130, z1=20, C=30 → descend then coordinated.
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 130.0f, 200.0f, 20.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(2u, n);
  TEST_ASSERT_FALSE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_TRUE(out[1].move_x);
  TEST_ASSERT_TRUE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out[1].z_mm);
  assert_coord_endpoints_in_bottom_band(out, n, 30.0f, 130.0f);
}

static void test_plan_start_in_band_end_above(void) {
  // z0=10, z1=130, C=30 → coordinated to C then ascend.
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 10.0f, 200.0f, 130.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(2u, n);
  TEST_ASSERT_TRUE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_FALSE(out[1].move_x);
  TEST_ASSERT_TRUE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 130.0f, out[1].z_mm);
  assert_coord_endpoints_in_bottom_band(out, n, 30.0f, 10.0f);
}

static void test_plan_both_above_band(void) {
  // z0=100, z1=120, C=30 → descend to C, X at C, ascend.
  PathSegment out[3];
  const size_t n = planSegments(10.0f, 100.0f, 200.0f, 120.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(3u, n);
  TEST_ASSERT_FALSE(out[0].move_x);
  TEST_ASSERT_TRUE(out[0].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out[0].z_mm);
  TEST_ASSERT_TRUE(out[1].move_x);
  TEST_ASSERT_FALSE(out[1].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out[1].x_mm);
  TEST_ASSERT_FALSE(out[2].move_x);
  TEST_ASSERT_TRUE(out[2].move_z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, out[2].z_mm);
  assert_coord_endpoints_in_bottom_band(out, n, 30.0f, 100.0f);
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

static void test_plan_noop(void) {
  PathSegment out[3];
  const size_t n = planSegments(35.0f, 40.0f, 35.0f, 40.0f, 30.0f, out);
  TEST_ASSERT_EQUAL_UINT(0u, n);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_decompose_pure_x);
  RUN_TEST(test_decompose_pure_z);
  RUN_TEST(test_decompose_45_degree_resultant);
  RUN_TEST(test_decompose_components_never_exceed_resultant);
  RUN_TEST(test_decompose_zero_profile_preserved);
  RUN_TEST(test_plan_pure_z);
  RUN_TEST(test_plan_both_in_band);
  RUN_TEST(test_plan_start_above_end_in_band);
  RUN_TEST(test_plan_start_in_band_end_above);
  RUN_TEST(test_plan_both_above_band);
  RUN_TEST(test_band_ceiling_from_zminus);
  RUN_TEST(test_z_in_traverse_band);
  RUN_TEST(test_segment_endpoints_reached);
  RUN_TEST(test_plan_noop);
  return UNITY_END();
}
