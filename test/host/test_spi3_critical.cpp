// Host tests for Spi3Bus Class 1 critical-section counter.

#include "unity.h"

#include "Spi3BusCritical.h"

void setUp(void) {}
void tearDown(void) {}

static void test_nested_enter_exit(void) {
  spi3::Class1Critical c;
  TEST_ASSERT_FALSE(c.active());
  TEST_ASSERT_EQUAL_INT(0, c.load());

  c.enter();
  TEST_ASSERT_TRUE(c.active());
  TEST_ASSERT_EQUAL_INT(1, c.load());

  c.enter();
  TEST_ASSERT_TRUE(c.active());
  TEST_ASSERT_EQUAL_INT(2, c.load());

  c.exit();
  TEST_ASSERT_TRUE(c.active());
  TEST_ASSERT_EQUAL_INT(1, c.load());

  c.exit();
  TEST_ASSERT_FALSE(c.active());
  TEST_ASSERT_EQUAL_INT(0, c.load());
}

static void test_underflow_clamped_to_zero(void) {
  spi3::Class1Critical c;
  c.exit();
  TEST_ASSERT_EQUAL_INT(0, c.load());
  TEST_ASSERT_FALSE(c.active());
  c.exit();
  TEST_ASSERT_EQUAL_INT(0, c.load());
}

static void test_reset_clears_active(void) {
  spi3::Class1Critical c;
  c.enter();
  c.enter();
  c.reset();
  TEST_ASSERT_FALSE(c.active());
  TEST_ASSERT_EQUAL_INT(0, c.load());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_nested_enter_exit);
  RUN_TEST(test_underflow_clamped_to_zero);
  RUN_TEST(test_reset_clears_active);
  return UNITY_END();
}
