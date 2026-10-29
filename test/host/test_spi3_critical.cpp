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

// enter()/exit() report idle<->active edges; the target signals the SPI3
// "Class 1 idle" event exactly on the reported exit edge.
static void test_transitions_reported_only_on_edges(void) {
  spi3::Class1Critical c;
  TEST_ASSERT_TRUE(c.enter());   // idle -> active
  TEST_ASSERT_FALSE(c.enter());  // nested
  TEST_ASSERT_FALSE(c.exit());   // still active
  TEST_ASSERT_TRUE(c.exit());    // active -> idle
  TEST_ASSERT_FALSE(c.active());
}

static void test_unmatched_exit_reports_idle(void) {
  spi3::Class1Critical c;
  // An unmatched exit must still report idle so waiters are never stranded.
  TEST_ASSERT_TRUE(c.exit());
  TEST_ASSERT_EQUAL_INT(0, c.load());
  TEST_ASSERT_TRUE(c.enter());
  TEST_ASSERT_TRUE(c.exit());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_nested_enter_exit);
  RUN_TEST(test_underflow_clamped_to_zero);
  RUN_TEST(test_reset_clears_active);
  RUN_TEST(test_transitions_reported_only_on_edges);
  RUN_TEST(test_unmatched_exit_reports_idle);
  return UNITY_END();
}
