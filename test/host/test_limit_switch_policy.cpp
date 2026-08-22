// Host tests for limit-switch debounce/pin encoding and end-effector routing.

#include "unity.h"

#include "GantryEndEffectorPolicy.h"
#include "GantryLimitSwitchPolicy.h"

void setUp(void) {}
void tearDown(void) {}

using Gantry::EndEffectorPolicy::PinDomain;
using Gantry::EndEffectorPolicy::classifyPin;
using Gantry::EndEffectorPolicy::outputLevel;
using Gantry::LimitPolicy::Debounce;
using Gantry::LimitPolicy::coerceDebounceCycles;
using Gantry::LimitPolicy::debounceSample;
using Gantry::LimitPolicy::forceSample;
using Gantry::LimitPolicy::isEncodedDirectPin;
using Gantry::LimitPolicy::isMcpLogicalPin;
using Gantry::LimitPolicy::rawLevelIsActive;
using Gantry::LimitPolicy::resolveDirectGpioPin;

static void test_pin_encoding_mcp_vs_direct(void) {
  TEST_ASSERT_TRUE(isMcpLogicalPin(0));
  TEST_ASSERT_TRUE(isMcpLogicalPin(15));
  TEST_ASSERT_FALSE(isMcpLogicalPin(GPIO_DIRECT_PIN_BASE));
  TEST_ASSERT_FALSE(isMcpLogicalPin(-1));

  TEST_ASSERT_TRUE(isEncodedDirectPin(GPIO_EXPANDER_DIRECT_FLAG | 5));
  TEST_ASSERT_EQUAL_INT(5, resolveDirectGpioPin(GPIO_EXPANDER_DIRECT_FLAG | 5));
  TEST_ASSERT_EQUAL_INT(22, resolveDirectGpioPin(22));
  TEST_ASSERT_EQUAL_INT(-1, resolveDirectGpioPin(3));
  TEST_ASSERT_EQUAL_INT(-1, resolveDirectGpioPin(-1));
}

static void test_debounce_cycles_zero_coerced_to_one(void) {
  TEST_ASSERT_EQUAL_UINT8(1, coerceDebounceCycles(0));
  TEST_ASSERT_EQUAL_UINT8(6, coerceDebounceCycles(6));
}

static void test_raw_level_active_low_and_high(void) {
  TEST_ASSERT_TRUE(rawLevelIsActive(0, true));
  TEST_ASSERT_FALSE(rawLevelIsActive(1, true));
  TEST_ASSERT_TRUE(rawLevelIsActive(1, false));
  TEST_ASSERT_FALSE(rawLevelIsActive(0, false));
}

static void test_force_sample_seeds_stable_state(void) {
  Debounce d{};
  forceSample(d, true, 6);
  TEST_ASSERT_TRUE(d.sample_state);
  TEST_ASSERT_TRUE(d.stable_state);
  TEST_ASSERT_EQUAL_UINT8(6, d.stable_count);
}

static void test_flap_below_n_rejected(void) {
  Debounce d{};
  forceSample(d, false, 3);
  debounceSample(d, true, 3);
  debounceSample(d, false, 3);
  TEST_ASSERT_FALSE(d.stable_state);
  TEST_ASSERT_FALSE(d.sample_state);
  TEST_ASSERT_EQUAL_UINT8(1, d.stable_count);
}

static void test_acceptance_at_n_consecutive(void) {
  Debounce d{};
  forceSample(d, false, 3);
  debounceSample(d, true, 3);
  TEST_ASSERT_FALSE(d.stable_state);
  debounceSample(d, true, 3);
  TEST_ASSERT_FALSE(d.stable_state);
  debounceSample(d, true, 3);
  TEST_ASSERT_TRUE(d.stable_state);
  TEST_ASSERT_TRUE(d.sample_state);
  TEST_ASSERT_EQUAL_UINT8(3, d.stable_count);
}

static void test_end_effector_output_level_both_polarities(void) {
  TEST_ASSERT_EQUAL_UINT8(1, outputLevel(true, true));
  TEST_ASSERT_EQUAL_UINT8(0, outputLevel(false, true));
  TEST_ASSERT_EQUAL_UINT8(0, outputLevel(true, false));
  TEST_ASSERT_EQUAL_UINT8(1, outputLevel(false, false));
}

static void test_end_effector_pin_domains(void) {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(PinDomain::kMcp),
                        static_cast<int>(classifyPin(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(PinDomain::kDirectGpio),
                        static_cast<int>(classifyPin(22)));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(PinDomain::kDirectGpio),
      static_cast<int>(classifyPin(GPIO_EXPANDER_DIRECT_FLAG | 4)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(PinDomain::kInvalid),
                        static_cast<int>(classifyPin(-1)));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_pin_encoding_mcp_vs_direct);
  RUN_TEST(test_debounce_cycles_zero_coerced_to_one);
  RUN_TEST(test_raw_level_active_low_and_high);
  RUN_TEST(test_force_sample_seeds_stable_state);
  RUN_TEST(test_flap_below_n_rejected);
  RUN_TEST(test_acceptance_at_n_consecutive);
  RUN_TEST(test_end_effector_output_level_both_polarities);
  RUN_TEST(test_end_effector_pin_domains);
  return UNITY_END();
}
