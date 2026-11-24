/**
 * @file test_w5500_deferred_log.cpp
 * @brief Host tests for the W5500 deferred log (G-B13): order, truncation,
 *        drop-on-full and recovery after a drain.
 */

#include "W5500DeferredLog.h"
#include "unity.h"

#include <cstring>
#include <string>

using namespace w5500;

void setUp(void) { resetDeferredLog(); }
void tearDown(void) {}

void test_lines_come_out_in_order(void) {
    TEST_ASSERT_TRUE(deferLog("A", 'W', 10, "first %d", 1));
    TEST_ASSERT_TRUE(deferLog("B", 'I', 20, "second %s", "x"));
    DeferredLogLine line;
    TEST_ASSERT_TRUE(popDeferredLog(line));
    TEST_ASSERT_EQUAL_STRING("A", line.tag);
    TEST_ASSERT_EQUAL_CHAR('W', line.level);
    TEST_ASSERT_EQUAL_UINT32(10, line.at_ms);
    TEST_ASSERT_EQUAL_STRING("first 1", line.text);
    TEST_ASSERT_TRUE(popDeferredLog(line));
    TEST_ASSERT_EQUAL_STRING("second x", line.text);
    TEST_ASSERT_FALSE(popDeferredLog(line));
}

void test_long_line_is_truncated(void) {
    const std::string longText(400, 'y');
    TEST_ASSERT_TRUE(deferLog("T", 'I', 0, "%s", longText.c_str()));
    DeferredLogLine line;
    TEST_ASSERT_TRUE(popDeferredLog(line));
    TEST_ASSERT_EQUAL_size_t(kDeferredLogLineLen - 1, std::strlen(line.text));
}

void test_full_queue_drops_and_counts(void) {
    for (size_t i = 0; i < kDeferredLogLines; ++i) {
        TEST_ASSERT_TRUE(deferLog("T", 'I', 0, "line %u", static_cast<unsigned>(i)));
    }
    TEST_ASSERT_FALSE(deferLog("T", 'I', 0, "one too many"));
    TEST_ASSERT_FALSE(deferLog("T", 'I', 0, "two too many"));
    TEST_ASSERT_EQUAL_UINT32(2, deferredLogDrops());
    DeferredLogLine line;
    TEST_ASSERT_TRUE(popDeferredLog(line));
    TEST_ASSERT_EQUAL_STRING("line 0", line.text);  // the oldest lines are kept
    TEST_ASSERT_TRUE(deferLog("T", 'I', 0, "fits again"));
    size_t n = 0;
    while (popDeferredLog(line)) {
        ++n;
    }
    TEST_ASSERT_EQUAL_size_t(kDeferredLogLines, n);
    TEST_ASSERT_EQUAL_STRING("fits again", line.text);
}

void test_wraps_around_many_times(void) {
    DeferredLogLine line;
    for (unsigned i = 0; i < 5 * kDeferredLogLines; ++i) {
        TEST_ASSERT_TRUE(deferLog("T", 'E', i, "n=%u", i));
        TEST_ASSERT_TRUE(popDeferredLog(line));
        TEST_ASSERT_EQUAL_UINT32(i, line.at_ms);
    }
    TEST_ASSERT_EQUAL_UINT32(0, deferredLogDrops());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_lines_come_out_in_order);
    RUN_TEST(test_long_line_is_truncated);
    RUN_TEST(test_full_queue_drops_and_counts);
    RUN_TEST(test_wraps_around_many_times);
    return UNITY_END();
}
