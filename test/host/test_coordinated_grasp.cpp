#include "coordinated_grasp.h"

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

void setUp(void) {}
void tearDown(void) {}

static const double kClassL[10] = {44.5, 50.5, 61.5, 50.0, 48.5, 34.5, 65.0, 70.0, 65.0, 65.0};
static const double kClassW[10] = {10.5, 14.5, 34.2, 26.2, 17.5, 17.0, 18.3, 21.2, 26.5, 32.0};
static const double kClassH[10] = {10.5, 14.5, 34.2, 26.2, 26.5, 17.0, 18.3, 21.2, 26.5, 32.0};

static void test_zero_angle_drag_is_zero(void) {
  CoordinatedGrasp::Body body;
  body.length_mm = 65.0;
  body.width_mm = 32.0;
  body.height_mm = 32.0;
  body.theta_deg = 0.0;
  body.speed_mm_s = 200.0;
  const auto plan = CoordinatedGrasp::solve(body);
  TEST_ASSERT_EQUAL(static_cast<int>(CoordinatedGrasp::Status::AnalyticClear),
                    static_cast<int>(plan.status));
  const double remain_s = 0.5 / (40.0 / 0.190);
  TEST_ASSERT_FLOAT_WITHIN(0.02, 200.0 * remain_s, plan.drag_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, plan.t_grasp_s, plan.t_second_s);
  body.theta_deg = -30.0;
  const auto neg = CoordinatedGrasp::solve(body);
  body.theta_deg = 30.0;
  const auto pos = CoordinatedGrasp::solve(body);
  TEST_ASSERT_EQUAL(static_cast<int>(pos.status), static_cast<int>(neg.status));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, pos.drag_mm, neg.drag_mm);
}

static void test_short_cell_is_invalid(void) {
  CoordinatedGrasp::Body body{44.5, 10.5, 10.5, 10.0, 100.0, 10.0, 50.0};
  const auto plan = CoordinatedGrasp::solve(body);
  TEST_ASSERT_EQUAL(static_cast<int>(CoordinatedGrasp::Status::InvalidInput),
                    static_cast<int>(plan.status));
}

static void test_retract_follows_second_contact(void) {
  CoordinatedGrasp::Body body{65.0, 32.0, 32.0, 0.0, 20.0, 10.0, 50.0};
  const auto plan = CoordinatedGrasp::solve(body);
  CoordinatedGrasp::Arm arm;
  CoordinatedGrasp::armPair(arm, plan, 1000000, 1);
  TEST_ASSERT_TRUE(arm.armed);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, plan.t_grasp_s, plan.t_second_s);
  TEST_ASSERT_EQUAL(
      1000000 + static_cast<int>(std::llround(plan.t_grasp_s * 1.0e6)),
      static_cast<int>(arm.retract_at_us));
  TEST_ASSERT_FALSE(CoordinatedGrasp::fireDue(arm, 2, arm.close_at_us, false));
  TEST_ASSERT_TRUE(CoordinatedGrasp::fireDue(arm, 1, arm.close_at_us, false));
  TEST_ASSERT_TRUE(arm.close_fired);
  TEST_ASSERT_FALSE(arm.close_pending);
}

static void test_contact_assisted_is_not_armed(void) {
  CoordinatedGrasp::Body body{65.0, 32.0, 32.0, 30.0, 1500.0, 10.0, 50.0};
  const auto plan = CoordinatedGrasp::solve(body);
  TEST_ASSERT_EQUAL(static_cast<int>(CoordinatedGrasp::Status::ContactAssisted),
                    static_cast<int>(plan.status));
  CoordinatedGrasp::Arm arm;
  CoordinatedGrasp::armPair(arm, plan, 0, 1);
  TEST_ASSERT_FALSE(arm.armed);
}

static void test_grid_matches_frozen_vectors(void) {
  std::ifstream in(COORDINATED_JSON, std::ios::binary);
  TEST_ASSERT_TRUE(in.good());
  std::stringstream buffer;
  buffer << in.rdbuf();
  const std::string data = buffer.str();
  const char* cursor = data.c_str();
  int mismatches = 0;
  for (int cls = 0; cls < 10; ++cls) {
    cursor = std::strstr(cursor, "\"rows\":");
    TEST_ASSERT_NOT_NULL(cursor);
    cursor = std::strchr(cursor, '[');
    TEST_ASSERT_NOT_NULL(cursor);
    ++cursor;
    int index = 0;
    while (*cursor && *cursor != ']') {
      while (*cursor && *cursor != '[' && *cursor != ']') {
        ++cursor;
      }
      if (*cursor != '[') {
        break;
      }
      ++cursor;
      int field[7];
      for (int f = 0; f < 7; ++f) {
        field[f] = std::atoi(cursor);
        cursor = std::strchr(cursor, f == 6 ? ']' : ',');
        TEST_ASSERT_NOT_NULL(cursor);
        ++cursor;
      }
      const int ang_i = index / 75;
      const int spd_i = index % 75;
      CoordinatedGrasp::Body body;
      body.length_mm = kClassL[cls];
      body.width_mm = kClassW[cls];
      body.height_mm = kClassH[cls];
      body.theta_deg = ang_i * 0.1;
      body.speed_mm_s = 20.0 + spd_i * 20.0;
      const auto plan = CoordinatedGrasp::solve(body);
      if (static_cast<int>(plan.status) != field[0]) {
        ++mismatches;
        if (mismatches == 1) {
          char msg[160];
          std::snprintf(msg, sizeof(msg), "class %d ang %.1f spd %.0f got %d want %d", cls + 1,
                        body.theta_deg, body.speed_mm_s, static_cast<int>(plan.status), field[0]);
          TEST_FAIL_MESSAGE(msg);
        }
      }
      if (field[0] <= 2) {
        const double drag = field[1] / 100.0;
        const double t1 = field[4] / 10000.0;
        const double t2 = field[5] / 10000.0;
        if (std::fabs(plan.drag_mm - drag) > 0.05 || std::fabs(plan.t_first_s - t1) > 2.5e-4 ||
            std::fabs(plan.t_second_s - t2) > 2.5e-4) {
          ++mismatches;
          if (mismatches == 1) {
            TEST_FAIL_MESSAGE("event mismatch");
          }
        }
      }
      ++index;
    }
    TEST_ASSERT_EQUAL(901 * 75, index);
  }
  TEST_ASSERT_EQUAL(0, mismatches);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_zero_angle_drag_is_zero);
  RUN_TEST(test_short_cell_is_invalid);
  RUN_TEST(test_retract_follows_second_contact);
  RUN_TEST(test_contact_assisted_is_not_armed);
  RUN_TEST(test_grid_matches_frozen_vectors);
  return UNITY_END();
}
