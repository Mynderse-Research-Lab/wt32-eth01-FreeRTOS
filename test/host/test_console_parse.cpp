// Host tests for UART console argument parsing (gantry_console_parse.h).

#include "unity.h"

#include "gantry_console_parse.h"

void setUp(void) {}
void tearDown(void) {}

using GantryConsole::AccelParse;
using GantryConsole::AxisToken;
using GantryConsole::LinearUnitMode;
using GantryConsole::MoveParse;
using GantryConsole::PuuCalParse;
using GantryConsole::SpeedParse;
using GantryConsole::ThetaLimParse;
using GantryConsole::applyRangeLimitU32;
using GantryConsole::convertMmToSelected;
using GantryConsole::convertSelectedToMm;
using GantryConsole::kMinAccelDegPerS2;
using GantryConsole::kMinAccelMmPerS2;
using GantryConsole::kMinSpeedMmPerS;
using GantryConsole::kMmPerInch;
using GantryConsole::linearUnitLabel;
using GantryConsole::parseAccelCommand;
using GantryConsole::parseAxisToken;
using GantryConsole::parseLinearUnit;
using GantryConsole::parseMoveCommand;
using GantryConsole::parsePuuCalCommand;
using GantryConsole::parseSpeedCommand;
using GantryConsole::parseThetaLimCommand;
using GantryConsole::suggestPuuScale;

static void test_axis_token_x_z_t_and_theta_alias(void) {
  AxisToken tok = AxisToken::X;
  TEST_ASSERT_TRUE(parseAxisToken("x", tok));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(AxisToken::X), static_cast<int>(tok));
  TEST_ASSERT_TRUE(parseAxisToken("z", tok));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(AxisToken::Z), static_cast<int>(tok));
  TEST_ASSERT_TRUE(parseAxisToken("t", tok));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(AxisToken::THETA), static_cast<int>(tok));
  TEST_ASSERT_TRUE(parseAxisToken("theta", tok));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(AxisToken::THETA), static_cast<int>(tok));
}

static void test_axis_token_rejects_bad_and_null(void) {
  AxisToken tok = AxisToken::X;
  TEST_ASSERT_FALSE(parseAxisToken("y", tok));
  TEST_ASSERT_FALSE(parseAxisToken("X", tok));
  TEST_ASSERT_FALSE(parseAxisToken("", tok));
  TEST_ASSERT_FALSE(parseAxisToken(nullptr, tok));
}

static void test_linear_unit_round_trip(void) {
  TEST_ASSERT_EQUAL_FLOAT(25.4f, convertSelectedToMm(1.0f, LinearUnitMode::INCH));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 1.0f,
                           convertMmToSelected(kMmPerInch, LinearUnitMode::INCH));
  TEST_ASSERT_EQUAL_FLOAT(12.5f, convertSelectedToMm(12.5f, LinearUnitMode::MM));
  TEST_ASSERT_EQUAL_FLOAT(12.5f, convertMmToSelected(12.5f, LinearUnitMode::MM));
  TEST_ASSERT_EQUAL_STRING("mm", linearUnitLabel(LinearUnitMode::MM));
  TEST_ASSERT_EQUAL_STRING("in", linearUnitLabel(LinearUnitMode::INCH));
}

static void test_parse_linear_unit_tokens(void) {
  LinearUnitMode mode = LinearUnitMode::MM;
  TEST_ASSERT_TRUE(parseLinearUnit("mm", mode));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(LinearUnitMode::MM), static_cast<int>(mode));
  TEST_ASSERT_TRUE(parseLinearUnit("in", mode));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(LinearUnitMode::INCH), static_cast<int>(mode));
  TEST_ASSERT_TRUE(parseLinearUnit("inch", mode));
  TEST_ASSERT_TRUE(parseLinearUnit("inches", mode));
  TEST_ASSERT_FALSE(parseLinearUnit("cm", mode));
  TEST_ASSERT_FALSE(parseLinearUnit(nullptr, mode));
}

static void test_range_limit_clamp_and_disabled(void) {
  TEST_ASSERT_EQUAL_UINT32(1u, applyRangeLimitU32(0u, kMinSpeedMmPerS, 500u, true));
  TEST_ASSERT_EQUAL_UINT32(500u, applyRangeLimitU32(9999u, kMinSpeedMmPerS, 500u, true));
  TEST_ASSERT_EQUAL_UINT32(50u, applyRangeLimitU32(50u, kMinSpeedMmPerS, 500u, true));
  TEST_ASSERT_EQUAL_UINT32(0u, applyRangeLimitU32(0u, kMinSpeedMmPerS, 500u, false));
  TEST_ASSERT_EQUAL_UINT32(kMinAccelMmPerS2,
                           applyRangeLimitU32(1u, kMinAccelMmPerS2, 3000u, true));
  TEST_ASSERT_EQUAL_UINT32(kMinAccelDegPerS2,
                           applyRangeLimitU32(0u, kMinAccelDegPerS2, 18000u, true));
}

static void test_parse_speed_ok_and_malformed(void) {
  SpeedParse p = parseSpeedCommand("speed 50 30");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_TRUE(p.has_deg);
  TEST_ASSERT_EQUAL_INT(50, p.speed_mm);
  TEST_ASSERT_EQUAL_INT(30, p.speed_deg);

  p = parseSpeedCommand("speed 80");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_FALSE(p.has_deg);

  TEST_ASSERT_FALSE(parseSpeedCommand("speed 0").ok);
  TEST_ASSERT_FALSE(parseSpeedCommand("speed").ok);
  TEST_ASSERT_FALSE(parseSpeedCommand("speed -1").ok);
}

static void test_parse_accel_ok_and_malformed(void) {
  AccelParse p = parseAccelCommand("accel 3000 2500 180 90");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL_INT(4, p.n);
  TEST_ASSERT_EQUAL_INT(3000, p.accel);
  TEST_ASSERT_EQUAL_INT(90, p.decel_deg);

  p = parseAccelCommand("accel 2000");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL_INT(1, p.n);

  TEST_ASSERT_FALSE(parseAccelCommand("accel").ok);
  TEST_ASSERT_FALSE(parseAccelCommand("accel 0").ok);
  TEST_ASSERT_FALSE(parseAccelCommand("accel 100 0").ok);
  TEST_ASSERT_FALSE(parseAccelCommand("accel 100 100 0").ok);
}

static void test_parse_move_arg_count(void) {
  MoveParse p = parseMoveCommand("move 10 20 30");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, p.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, p.z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, p.theta);

  TEST_ASSERT_FALSE(parseMoveCommand("move 10 20").ok);
  TEST_ASSERT_FALSE(parseMoveCommand("move").ok);
}

static void test_parse_thetalim(void) {
  ThetaLimParse p = parseThetaLimCommand("thetalim -90 90");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, p.min_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, p.max_deg);
  TEST_ASSERT_FALSE(parseThetaLimCommand("thetalim 90 90").ok);
  TEST_ASSERT_FALSE(parseThetaLimCommand("thetalim 10").ok);
}

static void test_puucal_parse_and_suggest_formula(void) {
  PuuCalParse p = parsePuuCalCommand("puucal x 100 80");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL_INT('x', p.axis);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, p.commanded);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 80.0f, p.measured);

  p = parsePuuCalCommand("puucal T 10 8");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL_INT('t', p.axis);

  TEST_ASSERT_FALSE(parsePuuCalCommand("puucal y 1 1").ok);
  TEST_ASSERT_FALSE(parsePuuCalCommand("puucal x 0 1").ok);
  TEST_ASSERT_FALSE(parsePuuCalCommand("puucal x 1").ok);

  double suggested = 0.0;
  TEST_ASSERT_TRUE(suggestPuuScale(100.0, 10.0f, 8.0f, suggested));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 125.0f, static_cast<float>(suggested));
  TEST_ASSERT_FALSE(suggestPuuScale(0.0, 10.0f, 8.0f, suggested));
  TEST_ASSERT_FALSE(suggestPuuScale(100.0, 0.0f, 8.0f, suggested));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_axis_token_x_z_t_and_theta_alias);
  RUN_TEST(test_axis_token_rejects_bad_and_null);
  RUN_TEST(test_linear_unit_round_trip);
  RUN_TEST(test_parse_linear_unit_tokens);
  RUN_TEST(test_range_limit_clamp_and_disabled);
  RUN_TEST(test_parse_speed_ok_and_malformed);
  RUN_TEST(test_parse_accel_ok_and_malformed);
  RUN_TEST(test_parse_move_arg_count);
  RUN_TEST(test_parse_thetalim);
  RUN_TEST(test_puucal_parse_and_suggest_formula);
  return UNITY_END();
}
