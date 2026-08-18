#include "unity.h"

#include "EipClass1Timing.h"
#include "EipReliabilityStats.h"
#include "Kinetix5100Assembly.h"
#include "W5500Poll.h"

void setUp(void) {}
void tearDown(void) {}

static void test_busy_poll_us_immediate_success(void) {
  int calls = 0;
  const bool ok = w5500::busyPollUs(5000, [&]() {
    ++calls;
    return true;
  });
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT(1, calls);
}

static void test_busy_poll_us_timeout(void) {
  const bool ok = w5500::busyPollUs(200, []() { return false; });
  TEST_ASSERT_FALSE(ok);
}

static void test_sendok_timeout_constants(void) {
  TEST_ASSERT_EQUAL_UINT32(2000u, w5500::kUdpSendOkTimeoutUs);
  TEST_ASSERT_EQUAL_UINT32(10000u, w5500::kUdpDestChangeSendOkTimeoutUs);
  TEST_ASSERT_EQUAL_UINT32(500000u, w5500::kTcpSendOkTimeoutUs);
  TEST_ASSERT_EQUAL_UINT32(50000u, w5500::kBusyPollYieldAboveUs);
  TEST_ASSERT_EQUAL_UINT32(2000u, w5500::kUdpRecvCmdTimeoutUs);
}

static void test_busy_poll_tight_timeout(void) {
  const bool ok = w5500::busyPollUsTight(200, []() { return false; });
  TEST_ASSERT_FALSE(ok);
}

static void test_soft_miss_warn_gate(void) {
  uint32_t last = 0;
  TEST_ASSERT_TRUE(eip::shouldWarnSoftMiss(last, 1000, 1000));
  TEST_ASSERT_FALSE(eip::shouldWarnSoftMiss(last, 1500, 1000));
  TEST_ASSERT_TRUE(eip::shouldWarnSoftMiss(last, 2500, 1000));
}

static void test_reliability_counters(void) {
  eip::reliabilityStats().reset();
  eip::reliabilityStats().noteSoftMiss();
  eip::reliabilityStats().noteSoftMiss();
  eip::reliabilityStats().noteSendOkFail();
  eip::reliabilityStats().noteChipRecover();
  eip::reliabilityStats().noteReconnect();
  const eip::ReliabilitySnapshot s = eip::reliabilityStats().snapshot();
  TEST_ASSERT_EQUAL_UINT32(2, s.soft_miss);
  TEST_ASSERT_EQUAL_UINT32(1, s.sendok_fail);
  TEST_ASSERT_EQUAL_UINT32(1, s.chip_recover);
  TEST_ASSERT_EQUAL_UINT32(1, s.reconnect);
}

static void test_likely_limit_stop_heuristic(void) {
  eip::k5100::InputAssembly154 fb{};
  TEST_ASSERT_FALSE(fb.isLikelyLimitStop());
  fb.fault = true;
  TEST_ASSERT_FALSE(fb.isLikelyLimitStop());
  fb.stopped = true;
  TEST_ASSERT_TRUE(fb.isLikelyLimitStop());
  TEST_ASSERT_TRUE(fb.hasDriveFault());
  TEST_ASSERT_FALSE(fb.isReady());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_busy_poll_us_immediate_success);
  RUN_TEST(test_busy_poll_us_timeout);
  RUN_TEST(test_sendok_timeout_constants);
  RUN_TEST(test_busy_poll_tight_timeout);
  RUN_TEST(test_soft_miss_warn_gate);
  RUN_TEST(test_reliability_counters);
  RUN_TEST(test_likely_limit_stop_heuristic);
  return UNITY_END();
}
