/**
 * @file test_eip_limit_sequence.cpp
 * @brief Host tests for EIP drive-managed home/cal phase helpers.
 */

#include "unity.h"

#include "GantryEipLimitSequence.h"

using Gantry::EipLimit::CreepOutcome;
using Gantry::EipLimit::SeekOutcome;
using Gantry::EipLimit::evaluateCreep;
using Gantry::EipLimit::evaluateSeek;

void setUp(void) {}
void tearDown(void) {}

static void test_seek_deltas_joint_min_max(void) {
  TEST_ASSERT_TRUE(Gantry::EipLimit::homeSeekDeltaMm() < 0.0f);
  TEST_ASSERT_TRUE(Gantry::EipLimit::homeCreepDeltaMm() > 0.0f);
  TEST_ASSERT_TRUE(Gantry::EipLimit::calSeekDeltaMm() > 0.0f);
  TEST_ASSERT_TRUE(Gantry::EipLimit::calCreepDeltaMm() < 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(Gantry::EipLimit::kSeekTravelMm,
                          -Gantry::EipLimit::homeSeekDeltaMm());
}

static void test_home_seek_outcomes(void) {
  TEST_ASSERT_EQUAL_INT((int)SeekOutcome::kTripped,
                        (int)evaluateSeek(/*warn=*/true, /*busy=*/true,
                                          /*sawBusy=*/true));
  TEST_ASSERT_EQUAL_INT((int)SeekOutcome::kWait,
                        (int)evaluateSeek(false, true, false));
  TEST_ASSERT_EQUAL_INT((int)SeekOutcome::kStartMove,
                        (int)evaluateSeek(false, false, false));
  TEST_ASSERT_EQUAL_INT((int)SeekOutcome::kFailNoTrip,
                        (int)evaluateSeek(false, false, true));
}

static void test_creep_outcomes(void) {
  TEST_ASSERT_EQUAL_INT((int)CreepOutcome::kCleared,
                        (int)evaluateCreep(/*warn=*/false, /*busy=*/true));
  TEST_ASSERT_EQUAL_INT((int)CreepOutcome::kWait,
                        (int)evaluateCreep(true, true));
  TEST_ASSERT_EQUAL_INT((int)CreepOutcome::kStartMove,
                        (int)evaluateCreep(true, false));
}

static void test_axis_letter(void) {
  TEST_ASSERT_EQUAL_STRING(
      "X", Gantry::EipLimit::axisLetter(Gantry::EipLimit::AxisRole::kX));
  TEST_ASSERT_EQUAL_STRING(
      "Z", Gantry::EipLimit::axisLetter(Gantry::EipLimit::AxisRole::kZ));
}

static void test_warning_labels_x_z(void) {
  using Gantry::EipLimit::AxisRole;
  using Gantry::EipLimit::jointMinIsA015;
  using Gantry::EipLimit::maxWarningLabel;
  using Gantry::EipLimit::minWarningLabel;
  TEST_ASSERT_EQUAL_STRING("A014/PL", minWarningLabel(AxisRole::kX));
  TEST_ASSERT_EQUAL_STRING("A015/NL", maxWarningLabel(AxisRole::kX));
  TEST_ASSERT_FALSE(jointMinIsA015(AxisRole::kX));
  TEST_ASSERT_EQUAL_STRING("A015/NL", minWarningLabel(AxisRole::kZ));
  TEST_ASSERT_EQUAL_STRING("A014/PL", maxWarningLabel(AxisRole::kZ));
  TEST_ASSERT_TRUE(jointMinIsA015(AxisRole::kZ));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_seek_deltas_joint_min_max);
  RUN_TEST(test_home_seek_outcomes);
  RUN_TEST(test_creep_outcomes);
  RUN_TEST(test_axis_letter);
  RUN_TEST(test_warning_labels_x_z);
  return UNITY_END();
}
