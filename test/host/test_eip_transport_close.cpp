/**
 * @file test_eip_transport_close.cpp
 * @brief Graceful close and class 1 timing host tests.
 */

#include "test_eip_transport_rigs.h"

using namespace eip_transport_test;

void test_graceful_close_uses_a_live_session(void) {
  ClosableAxis ax;
  ax.tcp.enqueueResponse(makeSendRRDataEncapReply(0x11110001, makeForwardCloseMrReply()));
  TEST_ASSERT_TRUE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_EQUAL_UINT32(1, ax.tcp.connects());
  TEST_ASSERT_FALSE(ax.scanner->axisConnected(0));
}

void test_graceful_close_rebuilds_a_dropped_session(void) {
  ClosableAxis ax;
  const size_t sent_before = ax.tcp.allSent().size();
  ax.tcp.dropByPeer();
  ax.tcp.enqueueResponse(makeRegisterSessionReply(0x22220002));
  ax.tcp.enqueueResponse(makeSendRRDataEncapReply(0x22220002, makeForwardCloseMrReply()));

  TEST_ASSERT_TRUE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_EQUAL_UINT32(2, ax.tcp.connects());
  // The new session's RegisterSession, then the ForwardClose on it.
  TEST_ASSERT_TRUE(ax.tcp.allSent().size() >= sent_before + 2);
  const Bytes& close_frame = ax.tcp.allSent()[sent_before + 1];
  TEST_ASSERT_EQUAL_HEX8(0x6F, close_frame[0]);
  TEST_ASSERT_EQUAL_HEX8(0x02, close_frame[4]);
  TEST_ASSERT_EQUAL_HEX8(0x00, close_frame[5]);
  TEST_ASSERT_EQUAL_HEX8(0x22, close_frame[6]);
  TEST_ASSERT_FALSE(ax.scanner->axisConnected(0));
}

// The live failure: the socket still looked connected when the send failed.
void test_graceful_close_retries_on_a_new_session_when_the_send_fails(void) {
  ClosableAxis ax;
  ax.tcp.failSendsUntilReconnect();
  ax.tcp.enqueueResponse(makeRegisterSessionReply(0x22220002));
  ax.tcp.enqueueResponse(makeSendRRDataEncapReply(0x22220002, makeForwardCloseMrReply()));

  TEST_ASSERT_TRUE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_EQUAL_UINT32(2, ax.tcp.connects());
  TEST_ASSERT_FALSE(ax.scanner->axisConnected(0));
}

// Connected through the close, or the keepalive helper would stop its O->T.
void test_graceful_close_keeps_the_axis_connected_until_closed(void) {
  ClosableAxis ax;
  ax.tcp.dropByPeer();
  ax.tcp.enqueueResponse(makeRegisterSessionReply(0x22220002));
  ax.tcp.enqueueResponse(makeSendRRDataEncapReply(0x22220002, makeForwardCloseMrReply()));
  size_t sends = 0;
  size_t while_connected = 0;
  eip::EipMultiScanner* scanner = ax.scanner.get();
  ax.tcp.onSend([&]() {
    ++sends;
    if (scanner->axisConnected(0)) ++while_connected;
  });

  TEST_ASSERT_TRUE(ax.scanner->gracefulCloseAxis(0));
  // RegisterSession and ForwardClose; UnRegisterSession follows the close.
  TEST_ASSERT_TRUE(sends >= 2);
  TEST_ASSERT_EQUAL_UINT32(sends - 1, while_connected);
}

void test_graceful_close_reports_a_failed_reconnect(void) {
  ClosableAxis ax;
  ax.tcp.dropByPeer();
  ax.tcp.failConnect();
  TEST_ASSERT_FALSE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_FALSE(ax.scanner->axisConnected(0));
  TEST_ASSERT_FALSE(ax.image.isOnline());
}

void test_graceful_close_ignores_a_closed_axis(void) {
  ClosableAxis ax;
  ax.tcp.enqueueResponse(makeSendRRDataEncapReply(0x11110001, makeForwardCloseMrReply()));
  TEST_ASSERT_TRUE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_FALSE(ax.scanner->gracefulCloseAxis(0));
  TEST_ASSERT_FALSE(ax.scanner->gracefulCloseAxis(5));
}

void test_multi_scanner_dual_demux(void) {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeUdpEndpoint udp;

  eip::ForwardOpenReply fo0 = makeSampleForwardOpenReply();
  fo0.ot_connection_id = 0x10000001;
  fo0.to_connection_id = 0x20000001;
  fo0.connection_serial = 0x0001;

  eip::ForwardOpenReply fo1 = makeSampleForwardOpenReply();
  fo1.ot_connection_id = 0x10000002;
  fo1.to_connection_id = 0x20000002;
  fo1.connection_serial = 0x0002;

  const uint32_t h0 = 0x11110001;
  const uint32_t h1 = 0x11110002;
  tcp0.enqueueResponse(makeRegisterSessionReply(h0));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardOpenMrReply(fo0)));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardCloseMrReply()));
  tcp0.enqueueResponse(makeRegisterSessionReply(0));

  tcp1.enqueueResponse(makeRegisterSessionReply(h1));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardOpenMrReply(fo1)));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardCloseMrReply()));
  tcp1.enqueueResponse(makeRegisterSessionReply(0));

  eip::ScannerConfig cfg0 = makeTestScannerConfig();
  cfg0.target_ip = "192.168.1.20";
  cfg0.connection_serial = 0x0001;
  cfg0.ot_connection_id = 0x10000001;
  cfg0.connection_timeout_multiplier = 7;

  eip::ScannerConfig cfg1 = makeTestScannerConfig();
  cfg1.target_ip = "192.168.1.21";
  cfg1.connection_serial = 0x0002;
  cfg1.ot_connection_id = 0x10000002;
  cfg1.connection_timeout_multiplier = 7;

  eip::EipProcessImage img0;
  eip::EipProcessImage img1;
  eip::MultiAxisSlot slots[2] = {{cfg0, &img0}, {cfg1, &img1}};
  eip::ITcpClient* tcps[2] = {&tcp0, &tcp1};

  eip::EipMultiScanner scanner(tcps, 2, udp, slots);
  TEST_ASSERT_TRUE(scanner.openAxis(0));
  TEST_ASSERT_TRUE(scanner.bindSharedUdp());
  TEST_ASSERT_TRUE(scanner.openAxis(1));
  TEST_ASSERT_TRUE(scanner.axisConnected(0));
  TEST_ASSERT_TRUE(scanner.axisConnected(1));

  const Bytes assy0(52, 0xA0);
  const Bytes assy1(52, 0xB1);
  // T->O frames carry the T->O connection ID in the sequenced address.
  udp.enqueueResponse(
      eip::buildClass1OutputCpf(fo0.to_connection_id, 1, 1, assy0, false));
  udp.enqueueResponse(
      eip::buildClass1OutputCpf(fo1.to_connection_id, 1, 1, assy1, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(50)));
  TEST_ASSERT_TRUE(img0.isOnline());
  TEST_ASSERT_TRUE(img1.isOnline());

  Bytes fb0;
  Bytes fb1;
  TEST_ASSERT_TRUE(img0.getFeedback(fb0));
  TEST_ASSERT_TRUE(img1.getFeedback(fb1));
  TEST_ASSERT_EQUAL_HEX8(0xA0, fb0[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB1, fb1[0]);

  scanner.disconnect();
  TEST_ASSERT_FALSE(img0.isOnline());
  TEST_ASSERT_FALSE(img1.isOnline());
}

void test_class1_rpi_remainder_and_miss_policy(void) {
  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderMs(5, 5));
  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderMs(5, 8));
  TEST_ASSERT_EQUAL_UINT32(3, eip::rpiRemainderMs(5, 2));
  TEST_ASSERT_EQUAL_UINT32(1, eip::rpiRemainderMs(0, 0));

  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderUs(1000, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderUs(1000, 1500));
  TEST_ASSERT_EQUAL_UINT32(250, eip::rpiRemainderUs(1000, 750));
  TEST_ASSERT_EQUAL_UINT32(1000, eip::rpiRemainderUs(0, 0));

  // Tick-aligned Class 1 pacing helpers (FreeRTOS 1 ms tick).
  TEST_ASSERT_EQUAL_UINT32(2, eip::class1PaceTicks(2000, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1RpiFractionUs(2000, 1000));
  TEST_ASSERT_EQUAL_UINT32(1, eip::class1PaceTicks(1500, 1000));
  TEST_ASSERT_EQUAL_UINT32(500, eip::class1RpiFractionUs(1500, 1000));
  TEST_ASSERT_EQUAL_UINT32(1, eip::class1PaceTicks(500, 1000));  // clamped
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1RpiFractionUs(500, 1000));
  TEST_ASSERT_EQUAL_UINT32(1, eip::class1PaceTicks(0, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1RpiFractionUs(0, 1000));

  // Overrun: catch up 1..7, yield+reset at 8. Firmware zeros streak on pdTRUE.
  {
    uint32_t streak = 0;
    for (uint32_t i = 1; i <= 7; ++i) {
      TEST_ASSERT_EQUAL(eip::Class1OverrunAction::kCatchUp,
                        eip::class1OverrunAction(streak, 8));
      TEST_ASSERT_EQUAL_UINT32(i, streak);
    }
    TEST_ASSERT_EQUAL(eip::Class1OverrunAction::kYieldTick,
                      eip::class1OverrunAction(streak, 8));
    TEST_ASSERT_EQUAL_UINT32(0, streak);
  }
  {
    uint32_t streak = 0;
    for (int i = 0; i < 16; ++i) {
      TEST_ASSERT_EQUAL(eip::Class1OverrunAction::kCatchUp,
                        eip::class1OverrunAction(streak, 0));
    }
    TEST_ASSERT_EQUAL_UINT32(16, streak);
  }

  TEST_ASSERT_FALSE(eip::shouldTeardownAfterInputMisses(0));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterInputMisses(1));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(2));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(3));

  // Default teardown window is 10x RPI (20 ms @ 2 ms), below the drive's
  // 16x RPI CIP timeout (multiplier code 2).
  TEST_ASSERT_EQUAL_UINT32(10u, eip::kClass1StaleTeardownRpiMultiples);
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(0, 2000));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(6000, 2000));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(19999, 2000));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(20000, 2000));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(1, 0));

  // Stale gate stays disarmed until O->T has been sent and N*RPI elapsed.
  TEST_ASSERT_FALSE(eip::class1StaleGateArmed(0, 2000, false));
  TEST_ASSERT_FALSE(eip::class1StaleGateArmed(10000, 2000, false));
  TEST_ASSERT_FALSE(eip::class1StaleGateArmed(5999, 2000, true));
  TEST_ASSERT_TRUE(eip::class1StaleGateArmed(6000, 2000, true));
  TEST_ASSERT_TRUE(eip::class1StaleGateArmed(1, 0, true));

  TEST_ASSERT_EQUAL_UINT32(0xC0A80114u, eip::parseIpv4Host("192.168.1.20"));
  TEST_ASSERT_EQUAL_UINT32(0u, eip::parseIpv4Host(""));
  TEST_ASSERT_EQUAL_UINT32(0u, eip::parseIpv4Host("192.168.1"));
  TEST_ASSERT_EQUAL_UINT32(0u, eip::parseIpv4Host(nullptr));

  uint32_t last_warn = 0;
  TEST_ASSERT_TRUE(eip::shouldWarnSoftMiss(last_warn, 100, 1000));
  TEST_ASSERT_FALSE(eip::shouldWarnSoftMiss(last_warn, 500, 1000));
  TEST_ASSERT_TRUE(eip::shouldWarnSoftMiss(last_warn, 1200, 1000));
}

// Exact boundary / rpi_us==0 / 32-bit overflow cases for the stale gate.
void test_stale_teardown_boundary(void) {
  // Boundary: age == 3 * RPI tears down; one less does not.
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(5999, 2000, 3));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(6000, 2000, 3));

  // rpi_us == 0: any positive age is stale; age 0 is not.
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(0, 0));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(1, 0));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(UINT32_MAX, 0));

  // Large RPI that would wrap a 32-bit product (0x60000000 * 3).
  const uint32_t big_rpi = 0x60000000u;
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(0, big_rpi, 3));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(UINT32_MAX, big_rpi, 3));
}

void test_class1_timing_stats_ring_and_cmd_to_start(void) {
  eip::class1TimingStats().reset();
  for (uint32_t i = 1; i <= 10; ++i) {
    eip::class1TimingStats().recordExchangeUs(i * 100);
  }
  const eip::Class1TimingSnapshot ex = eip::class1TimingStats().exchange();
  TEST_ASSERT_EQUAL_UINT32(10, ex.count);
  TEST_ASSERT_EQUAL_UINT32(100, ex.min_us);
  TEST_ASSERT_EQUAL_UINT32(1000, ex.max_us);
  TEST_ASSERT_TRUE(ex.p50_us >= 100 && ex.p50_us <= 1000);
  TEST_ASSERT_TRUE(ex.p99_us >= ex.p50_us);

  eip::class1TimingStats().reset();
  const int64_t t0 = eip::class1NowUs();
  eip::class1TimingStats().noteAbsoluteStartMotionPublished(t0);
  uint8_t assy[40] = {};
  assy[1] = 0x10;  // StartMotion
  eip::class1TimingStats().noteOtAssemblySent(assy, sizeof(assy), t0 + 250);
  const eip::Class1TimingSnapshot cs = eip::class1TimingStats().cmdToStart();
  TEST_ASSERT_EQUAL_UINT32(1, cs.count);
  TEST_ASSERT_EQUAL_UINT32(250, cs.min_us);
  TEST_ASSERT_EQUAL_UINT32(250, cs.p50_us);
}

void test_class1_timing_drain_ring_and_pace_counters(void) {
  eip::class1TimingStats().reset();
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1TimingStats().toDrain().count);
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1TimingStats().paceOverrunCount());
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1TimingStats().paceYieldCount());

  for (uint32_t i = 1; i <= 5; ++i) {
    eip::class1TimingStats().recordToDrainUs(i * 50);
  }
  const eip::Class1TimingSnapshot dr = eip::class1TimingStats().toDrain();
  TEST_ASSERT_EQUAL_UINT32(5, dr.count);
  TEST_ASSERT_EQUAL_UINT32(50, dr.min_us);
  TEST_ASSERT_EQUAL_UINT32(250, dr.max_us);

  eip::class1TimingStats().notePaceOverrun();
  eip::class1TimingStats().notePaceOverrun();
  eip::class1TimingStats().notePaceYield();
  TEST_ASSERT_EQUAL_UINT32(2, eip::class1TimingStats().paceOverrunCount());
  TEST_ASSERT_EQUAL_UINT32(1, eip::class1TimingStats().paceYieldCount());

  eip::class1TimingStats().reset();
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1TimingStats().paceOverrunCount());
  TEST_ASSERT_EQUAL_UINT32(0, eip::class1TimingStats().toDrain().count);
}

void test_multi_scanner_input_miss_status(void) {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeUdpEndpoint udp;

  eip::ForwardOpenReply fo0 = makeSampleForwardOpenReply();
  fo0.ot_connection_id = 0x10000001;
  fo0.to_connection_id = 0x20000001;
  fo0.connection_serial = 0x0001;
  fo0.to_api_us = 5000;

  eip::ForwardOpenReply fo1 = makeSampleForwardOpenReply();
  fo1.ot_connection_id = 0x10000002;
  fo1.to_connection_id = 0x20000002;
  fo1.connection_serial = 0x0002;
  fo1.to_api_us = 5000;

  const uint32_t h0 = 0x11110001;
  const uint32_t h1 = 0x11110002;
  tcp0.enqueueResponse(makeRegisterSessionReply(h0));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardOpenMrReply(fo0)));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardCloseMrReply()));
  tcp0.enqueueResponse(makeRegisterSessionReply(0));

  tcp1.enqueueResponse(makeRegisterSessionReply(h1));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardOpenMrReply(fo1)));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardCloseMrReply()));
  tcp1.enqueueResponse(makeRegisterSessionReply(0));

  eip::ScannerConfig cfg0 = makeTestScannerConfig();
  cfg0.target_ip = "192.168.1.20";
  cfg0.connection_serial = 0x0001;
  cfg0.ot_connection_id = 0x10000001;
  cfg0.to_connection_id = 0x20000001;
  cfg0.connection_timeout_multiplier = 7;

  eip::ScannerConfig cfg1 = makeTestScannerConfig();
  cfg1.target_ip = "192.168.1.21";
  cfg1.connection_serial = 0x0002;
  cfg1.ot_connection_id = 0x10000002;
  cfg1.to_connection_id = 0x20000002;
  cfg1.connection_timeout_multiplier = 7;

  eip::MultiAxisSlot slots[2] = {{cfg0, nullptr}, {cfg1, nullptr}};
  eip::ITcpClient* tcps[2] = {&tcp0, &tcp1};

  eip::EipMultiScanner scanner(tcps, 2, udp, slots);
  TEST_ASSERT_TRUE(scanner.openAxis(0));
  TEST_ASSERT_TRUE(scanner.bindSharedUdp());
  TEST_ASSERT_TRUE(scanner.openAxis(1));

  // FO does not stamp feedback — age is unknown until cyclic start.
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, scanner.axisFeedbackAgeUs(0));
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, scanner.axisFeedbackAgeUs(1));

  // Produce-then-drain: empty RX is OK during the post-cyclic grace window.
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));
  TEST_ASSERT_FALSE(scanner.axisReceivedLastCycle(0));
  TEST_ASSERT_FALSE(scanner.axisReceivedLastCycle(1));

  // Grace ends at 3x RPI (@5000 us = 15 ms) and reseeds the stale clock.
  // Real starvation needs another 10x RPI (50 ms) after that → miss.
  // Sleeps are sized well clear of ~15 ms host timer granularity.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // Well inside the 50 ms window: the old 3x RPI policy would already have
  // flagged this; the 10x policy must not.
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));
  (void)scanner.flushDeferredLogs();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kInputMiss),
                        static_cast<int>(scanner.exchangeOnce(0)));
  // The teardown diagnostic is queued for the task to print after the
  // critical section, never emitted from inside exchangeOnce.
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(scanner.pendingLogCount()));
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(scanner.flushDeferredLogs()));
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(scanner.pendingLogCount()));

  scanner.disconnect();
}

// Regression: axis0 FO→axis1 FO gap must not look like T->O starvation on the
// first cyclic exchange (bench: ~70 ms FO gap vs 3x2000 us = 6 ms).
void test_fo_delay_before_first_exchange_not_stale(void) {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeUdpEndpoint udp;

  eip::ForwardOpenReply fo0 = makeSampleForwardOpenReply();
  fo0.ot_connection_id = 0x10000001;
  fo0.to_connection_id = 0x20000001;
  fo0.connection_serial = 0x0001;
  fo0.to_api_us = 2000;

  eip::ForwardOpenReply fo1 = makeSampleForwardOpenReply();
  fo1.ot_connection_id = 0x10000002;
  fo1.to_connection_id = 0x20000002;
  fo1.connection_serial = 0x0002;
  fo1.to_api_us = 2000;

  const uint32_t h0 = 0x11110001;
  const uint32_t h1 = 0x11110002;
  tcp0.enqueueResponse(makeRegisterSessionReply(h0));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardOpenMrReply(fo0)));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardCloseMrReply()));
  tcp0.enqueueResponse(makeRegisterSessionReply(0));

  tcp1.enqueueResponse(makeRegisterSessionReply(h1));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardOpenMrReply(fo1)));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardCloseMrReply()));
  tcp1.enqueueResponse(makeRegisterSessionReply(0));

  eip::ScannerConfig cfg0 = makeTestScannerConfig();
  cfg0.target_ip = "192.168.1.20";
  cfg0.connection_serial = 0x0001;
  cfg0.ot_connection_id = 0x10000001;
  cfg0.to_connection_id = 0x20000001;
  cfg0.connection_timeout_multiplier = 7;

  eip::ScannerConfig cfg1 = makeTestScannerConfig();
  cfg1.target_ip = "192.168.1.21";
  cfg1.connection_serial = 0x0002;
  cfg1.ot_connection_id = 0x10000002;
  cfg1.to_connection_id = 0x20000002;
  cfg1.connection_timeout_multiplier = 7;

  eip::MultiAxisSlot slots[2] = {{cfg0, nullptr}, {cfg1, nullptr}};
  eip::ITcpClient* tcps[2] = {&tcp0, &tcp1};

  eip::EipMultiScanner scanner(tcps, 2, udp, slots);
  TEST_ASSERT_TRUE(scanner.openAxis(0));
  TEST_ASSERT_TRUE(scanner.bindSharedUdp());
  // Simulate peer FO / bind cost that previously aged axis0 past 3x RPI.
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  TEST_ASSERT_TRUE(scanner.openAxis(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(15));

  // First cyclic exchange must not spuriously return kInputMiss.
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));

  scanner.disconnect();
}

