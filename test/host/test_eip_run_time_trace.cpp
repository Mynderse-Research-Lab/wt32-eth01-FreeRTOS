// Host tests for the per-task CPU time report behind the Class 1 gap
// diagnostics (G-B6).

#include "EipRunTimeTrace.h"
#include "unity.h"

#include <initializer_list>

using eip::RunTimeHistory;
using eip::RunTimeSnapshot;
using eip::TaskRunDelta;

void setUp() {}
void tearDown() {}

static RunTimeSnapshot snapshot(uint32_t t_us, std::initializer_list<eip::TaskRunTime> tasks) {
    RunTimeSnapshot s;
    s.t_us = t_us;
    for (const eip::TaskRunTime& t : tasks) {
        s.tasks[s.count++] = t;
    }
    return s;
}

static void test_history_finds_the_newest_snapshot_before_a_time() {
    static RunTimeHistory h;
    for (uint32_t t = 0; t < 12; ++t) {
        h.push(snapshot(t * 20000, {}));
    }
    TEST_ASSERT_EQUAL_UINT32(eip::kRunTimeHistoryDepth, h.size());
    const RunTimeSnapshot* s = h.atOrBefore(175000);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL_UINT32(160000, s->t_us);
    // Older than everything kept: snapshots 0..3 were overwritten.
    TEST_ASSERT_NULL(h.atOrBefore(70000));
}

static void test_history_survives_the_microsecond_wrap() {
    static RunTimeHistory h;
    h.push(snapshot(0xFFFFB1E0u, {}));  // 2^32 - 20000
    h.push(snapshot(0x00000000u, {}));
    const RunTimeSnapshot* s = h.atOrBefore(0xFFFFFF00u);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFB1E0u, s->t_us);
}

// The G-B6 question: what ran on core 1 while the scanner did not.
static void test_diff_ranks_tasks_by_cpu_time() {
    const RunTimeSnapshot before = snapshot(0, {{1, 1000, 1}, {2, 5000, 1}, {3, 700, 0}});
    const RunTimeSnapshot after = snapshot(80000, {{1, 61000, 1}, {2, 9000, 1}, {3, 700, 0}});
    TaskRunDelta out[8];
    const size_t n = eip::diffRunTimes(before, after, out, 8);
    TEST_ASSERT_EQUAL_UINT32(2, n);  // task 3 used no time
    TEST_ASSERT_EQUAL_UINT32(1, out[0].task_number);
    TEST_ASSERT_EQUAL_UINT32(60000, out[0].delta_us);
    TEST_ASSERT_EQUAL_INT8(1, out[0].core);
    TEST_ASSERT_EQUAL_UINT32(2, out[1].task_number);
    TEST_ASSERT_EQUAL_UINT32(4000, out[1].delta_us);
}

static void test_diff_counts_a_new_task_from_zero() {
    const RunTimeSnapshot before = snapshot(0, {{1, 1000, 0}});
    const RunTimeSnapshot after = snapshot(20000, {{1, 1500, 0}, {9, 3000, 1}});
    TaskRunDelta out[4];
    TEST_ASSERT_EQUAL_UINT32(2, eip::diffRunTimes(before, after, out, 4));
    TEST_ASSERT_EQUAL_UINT32(9, out[0].task_number);
    TEST_ASSERT_EQUAL_UINT32(3000, out[0].delta_us);
}

static void test_diff_keeps_the_largest_when_the_output_is_short() {
    const RunTimeSnapshot before = snapshot(0, {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}});
    const RunTimeSnapshot after = snapshot(20000, {{1, 100, 0}, {2, 900, 0}, {3, 500, 0}});
    TaskRunDelta out[2];
    TEST_ASSERT_EQUAL_UINT32(2, eip::diffRunTimes(before, after, out, 2));
    TEST_ASSERT_EQUAL_UINT32(2, out[0].task_number);
    TEST_ASSERT_EQUAL_UINT32(3, out[1].task_number);
}

static void test_diff_survives_a_counter_wrap() {
    const RunTimeSnapshot before = snapshot(0, {{1, 0xFFFFFF00u, 1}});
    const RunTimeSnapshot after = snapshot(20000, {{1, 0x00000100u, 1}});
    TaskRunDelta out[1];
    TEST_ASSERT_EQUAL_UINT32(1, eip::diffRunTimes(before, after, out, 1));
    TEST_ASSERT_EQUAL_UINT32(0x200, out[0].delta_us);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_history_finds_the_newest_snapshot_before_a_time);
    RUN_TEST(test_history_survives_the_microsecond_wrap);
    RUN_TEST(test_diff_ranks_tasks_by_cpu_time);
    RUN_TEST(test_diff_counts_a_new_task_from_zero);
    RUN_TEST(test_diff_keeps_the_largest_when_the_output_is_short);
    RUN_TEST(test_diff_survives_a_counter_wrap);
    return UNITY_END();
}
