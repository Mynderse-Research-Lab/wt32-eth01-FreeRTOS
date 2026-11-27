/**
 * @file test_w5500_socket_fastpath_tcp.cpp
 * @brief TCP send-nowait host tests.
 */

#include "test_w5500_socket_fastpath_internal.h"

#include "W5500Poll.h"

using namespace w5500_fastpath_test;

void test_tcp_send_nowait_skips_sendok_poll(void) {
    CountingHal hal;
    hal.sn_sr[0] = SOCK_ESTABLISHED;
    const uint8_t frame[24] = {};
    TEST_ASSERT_EQUAL_INT(24, w5500::socketSendNoWait(hal, 0, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_HEX8(Sn_CR_SEND, hal.last_cr_written);
    TEST_ASSERT_TRUE(hal.lastTxnIsCommandWrite());
    TEST_ASSERT_EQUAL_UINT32(0, hal.countCrIrPolls());
}

void test_tcp_send_nowait_reports_the_previous_failure(void) {
    CountingHal hal;
    hal.sn_sr[0] = SOCK_ESTABLISHED;
    hal.sn_ir = Sn_IR_TIMEOUT;
    const uint8_t frame[24] = {};
    TEST_ASSERT_EQUAL_INT(-1, w5500::socketSendNoWait(hal, 0, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_HEX8(0, hal.sn_ir);
    // The flag is cleared, so the next one goes out.
    TEST_ASSERT_EQUAL_INT(24, w5500::socketSendNoWait(hal, 0, frame, sizeof(frame)));
}

void test_tcp_send_nowait_needs_an_established_socket(void) {
    CountingHal hal;
    hal.sn_sr[0] = SOCK_CLOSE_WAIT;
    const uint8_t frame[24] = {};
    TEST_ASSERT_EQUAL_INT(-1, w5500::socketSendNoWait(hal, 0, frame, sizeof(frame)));
    TEST_ASSERT_NOT_EQUAL(Sn_CR_SEND, hal.last_cr_written);
}

// A NOP's SENDOK left in Sn_IR must not confirm the next explicit send.
void test_tcp_send_ignores_a_stale_sendok(void) {
    CountingHal hal;
    hal.sn_sr[0] = SOCK_ESTABLISHED;
    hal.sn_ir = Sn_IR_SENDOK;
    const uint8_t frame[24] = {};
    TEST_ASSERT_EQUAL_INT(-1, w5500::socketSend(hal, 0, frame, sizeof(frame)));
}

// G-B6: reading the statistics must not need the bus mutex. With the old
// take() this deadlocked here; on the gantry it held off Class 1 for 30 ms.
void test_bus_stats_take_does_not_need_the_bus_mutex(void) {
    w5500::BusMutex m;
    m.lock();
    const w5500::BusHoldStats first = m.take();
    // unlock() records a hold only when it is longer than the window's
    // longest so far (0 after take()), and the host clock counts whole
    // microseconds. A faster lock/unlock measured 0 us and left the holder
    // empty, so hold until the clock has ticked.
    const int64_t t0 = w5500::pollNowUs();
    while (w5500::pollNowUs() == t0) {
    }
    m.unlock();
    (void)first;
    const w5500::BusHoldStats s = m.take();
    TEST_ASSERT_GREATER_THAN_UINT32(0, s.max_hold_us);
    TEST_ASSERT_EQUAL_STRING("host", s.holder);  // the hold just released
}
