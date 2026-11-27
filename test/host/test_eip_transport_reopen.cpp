/**
 * @file test_eip_transport_reopen.cpp
 * @brief Reopen, batch drain and encapsulation host tests.
 */

#include "test_eip_transport_rigs.h"

using namespace eip_transport_test;

void test_axis_reopen_backoff_policy(void) {
  TEST_ASSERT_EQUAL_UINT32(500000u, eip::kHcs01ReopenAfterStaleUs);
  TEST_ASSERT_EQUAL_UINT32(1000u, eip::axisReopenNextBackoffMs(0));
  TEST_ASSERT_EQUAL_UINT32(2000u, eip::axisReopenNextBackoffMs(1000));
  TEST_ASSERT_EQUAL_UINT32(8000u, eip::axisReopenNextBackoffMs(4000));
  TEST_ASSERT_EQUAL_UINT32(10000u, eip::axisReopenNextBackoffMs(8000));
  TEST_ASSERT_EQUAL_UINT32(10000u, eip::axisReopenNextBackoffMs(10000));
  TEST_ASSERT_EQUAL_UINT32(10000u, eip::axisReopenNextBackoffMs(0x90000000u));
}

// Regression (bench log 2026-10-13): HCS01 went silent after one T->O and was
// never re-opened while X/Z kept running. It must be re-opened on its own.
void test_hcs01_silent_axis_is_reopened_alone(void) {
  TriConnected d(true);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(eip::EipMultiScanner::kMaxAxes),
                           static_cast<uint32_t>(d.scanner->nextAxisToReopen()));
  d.starveThetaPastReopenThreshold();

  TEST_ASSERT_TRUE(d.scanner->axisReopenPending(2));
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(d.scanner->nextAxisToReopen()));
  TEST_ASSERT_TRUE(d.img0.isOnline());
  TEST_ASSERT_TRUE(d.img1.isOnline());
  TEST_ASSERT_FALSE(d.img2.isOnline());
  (void)d.scanner->flushDeferredLogs();

  d.scanner->prepareAxisReopen(2);
  TEST_ASSERT_FALSE(d.scanner->axisConnected(2));
  TEST_ASSERT_TRUE(d.scanner->axisConnected(0));
  const size_t x_tcp_frames_before = d.tcp0.allSent().size();
  const size_t z_tcp_frames_before = d.tcp1.allSent().size();
  TEST_ASSERT_TRUE(d.scanner->reopenAxis(2));

  // Old connection was closed before the new ForwardOpen (0x4E = FC service).
  TEST_ASSERT_TRUE(d.tcp2.sentContainsCipService(0x4E));
  TEST_ASSERT_TRUE(d.scanner->axisConnected(2));
  TEST_ASSERT_FALSE(d.scanner->axisReopenPending(2));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(eip::EipMultiScanner::kMaxAxes),
                           static_cast<uint32_t>(d.scanner->nextAxisToReopen()));
  // X/Z were never touched: still connected, no explicit traffic to them.
  TEST_ASSERT_TRUE(d.scanner->axisConnected(0));
  TEST_ASSERT_TRUE(d.scanner->axisConnected(1));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(x_tcp_frames_before),
                           static_cast<uint32_t>(d.tcp0.allSent().size()));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(z_tcp_frames_before),
                           static_cast<uint32_t>(d.tcp1.allSent().size()));

  // Theta answers again on the new connection and comes back online.
  d.feedKinetix();
  d.udp.enqueueResponse(eip::buildClass1OutputCpf(d.fo[2].to_connection_id, 1, 1, d.th, false));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(2));
  TEST_ASSERT_TRUE(d.img2.isOnline());
  TEST_ASSERT_TRUE(d.scanner->flushDeferredLogs() >= 1u);  // reopen OK + first T->O

  d.scanner->disconnect();
}

void test_hcs01_reopen_failure_backs_off(void) {
  TriConnected d(false);
  d.starveThetaPastReopenThreshold();
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(d.scanner->nextAxisToReopen()));
  (void)d.scanner->flushDeferredLogs();

  d.scanner->prepareAxisReopen(2);
  TEST_ASSERT_FALSE(d.scanner->reopenAxis(2));
  TEST_ASSERT_FALSE(d.scanner->axisConnected(2));
  TEST_ASSERT_TRUE(d.scanner->axisReopenPending(2));
  // Not retried immediately: next attempt waits the initial backoff.
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(eip::EipMultiScanner::kMaxAxes),
                           static_cast<uint32_t>(d.scanner->nextAxisToReopen()));
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(d.scanner->flushDeferredLogs()));

  // X/Z keep exchanging normally with theta idle.
  d.feedKinetix();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_TRUE(d.img0.isOnline());
  TEST_ASSERT_TRUE(d.img1.isOnline());

  d.scanner->disconnect();
  TEST_ASSERT_FALSE(d.scanner->axisReopenPending(2));
}

void test_drain_ignores_duplicate_cid(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);

  const Bytes first(52, 0x11);
  const Bytes second(52, 0x22);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, first, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 2, 2, second, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_FALSE(d.scanner->axisReceivedLastCycle(1));

  Bytes fb0;
  TEST_ASSERT_TRUE(d.img0.getFeedback(fb0));
  // First matching CID wins; duplicate in the same cycle is ignored.
  TEST_ASSERT_EQUAL_HEX8(0x11, fb0[0]);

  d.scanner->disconnect();
}

void test_drain_ignores_unknown_cid(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);

  const Bytes foreign(52, 0xEE);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(0xDEADBEEF, 1, 1, foreign, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_FALSE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_FALSE(d.scanner->axisReceivedLastCycle(1));
  TEST_ASSERT_FALSE(d.img0.isOnline());
  TEST_ASSERT_FALSE(d.img1.isOnline());

  d.scanner->disconnect();
}

void test_drain_stops_when_all_axes_got(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);

  const Bytes assy0(52, 0xA0);
  const Bytes assy1(52, 0xB1);
  const Bytes extra(52, 0xEE);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy0, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo1.to_connection_id, 1, 1, assy1, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 2, 2, extra, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(d.udp.recvCallCount()));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(1));

  Bytes fb0;
  TEST_ASSERT_TRUE(d.img0.getFeedback(fb0));
  TEST_ASSERT_EQUAL_HEX8(0xA0, fb0[0]);

  d.scanner->disconnect();
}

void test_drain_is_bounded(void) {
  DualConnected d(20000, false);
  const Bytes assy(52, 0x55);
  d.udp.setFloodFrame(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(
      static_cast<uint32_t>(eip::EipMultiScanner::kMaxDrainPerCycle),
      static_cast<uint32_t>(d.udp.recvCallCount()));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));

  d.scanner->disconnect();
}

// exchangeOnce must time the drain so eiptiming can show it.
void test_exchange_records_drain_sample(void) {
  DualConnected d(20000, false);
  d.udp.setEmptyReturnsZero(true);
  eip::class1TimingStats().reset();

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(1, eip::class1TimingStats().toDrain().count);

  d.scanner->disconnect();
}

// One batched burst carries both axes plus a duplicate; first CID wins and no
// second drain call is needed.
void test_batch_drain_applies_both_axes_in_one_call(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);
  d.udp.setBatchMode(true);

  const Bytes first(52, 0x11);
  const Bytes dup(52, 0x22);
  const Bytes assy1(52, 0xB1);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, first, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 2, 2, dup, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo1.to_connection_id, 1, 1, assy1, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(d.udp.batchCallCount()));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(d.udp.recvCallCount()));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(1));

  Bytes fb0;
  Bytes fb1;
  TEST_ASSERT_TRUE(d.img0.getFeedback(fb0));
  TEST_ASSERT_TRUE(d.img1.getFeedback(fb1));
  TEST_ASSERT_EQUAL_HEX8(0x11, fb0[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB1, fb1[0]);

  d.scanner->disconnect();
}

// Batched frames are still capped at kMaxDrainPerCycle per cycle.
void test_batch_drain_is_bounded(void) {
  DualConnected d(20000, false);
  d.udp.setBatchMode(true);
  const Bytes assy(52, 0x55);
  d.udp.setFloodFrame(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(
      static_cast<uint32_t>(eip::EipMultiScanner::kMaxDrainPerCycle),
      static_cast<uint32_t>(d.udp.batchDatagramCount()));
  TEST_ASSERT_TRUE(d.udp.batchCallCount() <
                   eip::EipMultiScanner::kMaxDrainPerCycle);
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));

  d.scanner->disconnect();
}

void test_feedback_age_resets_on_fresh_data(void) {
  DualConnected d(20000, false);
  d.udp.setEmptyReturnsZero(true);

  // FO must not invent a feedback timestamp.
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, d.scanner->axisFeedbackAgeUs(0));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  const uint32_t age_seeded = d.scanner->axisFeedbackAgeUs(0);
  TEST_ASSERT_TRUE(age_seeded < 5000);

  const Bytes assy(52, 0x42);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy, false));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  const uint32_t age_fresh = d.scanner->axisFeedbackAgeUs(0);
  TEST_ASSERT_TRUE(age_fresh < age_seeded || age_fresh < 2000);
  TEST_ASSERT_TRUE(age_fresh < 5000);

  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  const uint32_t age_starved = d.scanner->axisFeedbackAgeUs(0);
  TEST_ASSERT_TRUE(age_starved > age_fresh);
  TEST_ASSERT_TRUE(age_starved >= 2000);

  d.scanner->disconnect();
}

void test_exchange_reports_send_failure(void) {
  DualConnected d(20000, false);
  d.udp.setEmptyReturnsZero(true);
  d.udp.setFailSend(true);

  const size_t recv_before = d.udp.recvCallCount();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOutputSendFailed),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(recv_before),
                           static_cast<uint32_t>(d.udp.recvCallCount()));

  d.udp.setFailSend(false);
  d.scanner->disconnect();
}

void test_cip_sequence_number_wraparound(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x12345678;
  cfg.ot_include_run_idle_header = false;
  io.setConfig(cfg);
  
  // CIP sequence is a uint16_t, so it wraps at 0xFFFF.
  // We force the underlying uint16_t to wrap by pushing 0x10000 frames.
  // To avoid huge loops in unit tests, we'll just verify the EipIoConnection
  // wraps cleanly. Let's just pump 65537 frames. It's fast on host.
  const Bytes dummy_assembly = {0x01, 0x02, 0x03};
  Bytes last_frame;
  for (uint32_t i = 0; i < 65537; ++i) {
    last_frame = io.buildOutputFrame(dummy_assembly);
  }
  
  // Encap sequence should be 65537 (0x00010001), CIP sequence should wrap to 1
  TEST_ASSERT_EQUAL_UINT32(65537, io.encapSequence());
  TEST_ASSERT_EQUAL_UINT16(1, io.cipSequence());
  
  // And the frame should contain CIP sequence 1
  // Sequence number is at byte 10,11 in the CPF payload usually.
  // Let's decode it safely using the helper.
  uint32_t cid = 0;
  Bytes out_assy;
  TEST_ASSERT_TRUE(eip::parseClass1InputCpf(last_frame, cid, out_assy, false));
  TEST_ASSERT_EQUAL_UINT32(0x12345678, cid);
  TEST_ASSERT_EQUAL_UINT32(dummy_assembly.size(), out_assy.size());
}

void test_session_nop_frame(void) {
    FakeTcpClient tcp;
    tcp.enqueueResponse(makeRegisterSessionReply(0x0A0B0C0D));
    eip::EipSession session(tcp);
    TEST_ASSERT_TRUE(session.registerSession());

    TEST_ASSERT_TRUE(session.sendNop());
    const Bytes& f = tcp.lastSent();
    TEST_ASSERT_EQUAL_UINT32(24, f.size());
    const uint8_t head[8] = {0x00, 0x00, 0x00, 0x00, 0x0D, 0x0C, 0x0B, 0x0A};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(head, f.data(), sizeof(head));
    for (size_t i = 8; i < 12; ++i) {
        TEST_ASSERT_EQUAL_HEX8(0, f[i]);  // status
    }
    for (size_t i = 20; i < 24; ++i) {
        TEST_ASSERT_EQUAL_HEX8(0, f[i]);  // options
    }
}

void test_session_nop_needs_a_session(void) {
    FakeTcpClient tcp;
    eip::EipSession session(tcp);
    TEST_ASSERT_FALSE(session.sendNop());
    TEST_ASSERT_EQUAL_UINT32(0, tcp.allSent().size());
}

void test_encap_nop_interval_policy(void) {
    TEST_ASSERT_EQUAL_UINT32(30000, eip::encapNopIntervalMs(true, 120));
    TEST_ASSERT_EQUAL_UINT32(30000, eip::encapNopIntervalMs(false, 0));
    TEST_ASSERT_EQUAL_UINT32(0, eip::encapNopIntervalMs(true, 0));
    TEST_ASSERT_EQUAL_UINT32(250, eip::encapNopIntervalMs(true, 1));
    TEST_ASSERT_EQUAL_UINT32(900000, eip::encapNopIntervalMs(true, 3600));
}

void test_encap_inactivity_timeout_request(void) {
    // Get_Attribute_Single, 3-word path: class 0xF5, instance 1, attribute 13.
    const Bytes expected = {0x0E, 0x03, 0x20, 0xF5, 0x24, 0x01, 0x30, 0x0D};
    const Bytes req = eip::buildEncapInactivityTimeoutGetRequest();
    TEST_ASSERT_EQUAL_UINT32(expected.size(), req.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), req.data(), expected.size());
}

