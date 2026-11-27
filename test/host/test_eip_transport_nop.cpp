/**
 * @file test_eip_transport_nop.cpp
 * @brief Encapsulation timeout host tests.
 */

#include "test_eip_transport_rigs.h"

using namespace eip_transport_test;

void test_open_reads_the_timeout_and_spaces_nops(void) {
    KeptAliveAxis ax(makeGetAttributeMrReply(0, Bytes{60, 0}));  // 60 s
    TEST_ASSERT_TRUE(ax.scanner->axisConnected(0));
    TEST_ASSERT_EQUAL_UINT32(15000, ax.scanner->axisEncapNopIntervalMs(0));

    const int64_t now = eip::class1NowUs();
    ax.scanner->serviceEncapKeepalive(now);
    TEST_ASSERT_EQUAL_UINT32(0, ax.nopsSent());

    const int64_t due = now + 15000LL * 1000;
    ax.scanner->serviceEncapKeepalive(due);
    TEST_ASSERT_EQUAL_UINT32(1, ax.nopsSent());
    ax.scanner->serviceEncapKeepalive(due + 1000);
    TEST_ASSERT_EQUAL_UINT32(1, ax.nopsSent());
    ax.scanner->serviceEncapKeepalive(due + 15000LL * 1000);
    TEST_ASSERT_EQUAL_UINT32(2, ax.nopsSent());
}

void test_unreadable_timeout_assumes_the_default(void) {
    KeptAliveAxis ax(makeGetAttributeMrReply(0x14, Bytes{}));  // attribute not supported
    TEST_ASSERT_TRUE(ax.scanner->axisConnected(0));
    TEST_ASSERT_EQUAL_UINT32(30000, ax.scanner->axisEncapNopIntervalMs(0));
}

void test_disabled_timeout_sends_no_nop(void) {
    KeptAliveAxis ax(makeGetAttributeMrReply(0, Bytes{0, 0}));
    ax.scanner->serviceEncapKeepalive(eip::class1NowUs() + 3600LL * 1000 * 1000);
    TEST_ASSERT_EQUAL_UINT32(0, ax.nopsSent());
}

void test_failed_nop_is_logged_once_and_not_recovered(void) {
    KeptAliveAxis ax(makeGetAttributeMrReply(0, Bytes{120, 0}));
    const size_t connects = ax.tcp.connects();
    ax.tcp.failSendsUntilReconnect();
    const int64_t due = eip::class1NowUs() + 30000LL * 1000;

    ax.scanner->serviceEncapKeepalive(due);
    TEST_ASSERT_EQUAL_UINT32(1, ax.scanner->pendingLogCount());
    ax.scanner->serviceEncapKeepalive(due + 30000LL * 1000);
    TEST_ASSERT_EQUAL_UINT32(1, ax.scanner->pendingLogCount());
    TEST_ASSERT_EQUAL_UINT32(connects, ax.tcp.connects());
    TEST_ASSERT_TRUE(ax.scanner->axisConnected(0));
}

