/**
 * @file test_eip_transport_drain.cpp
 * @brief Class 1 drain host tests.
 */

#include "test_eip_transport_rigs.h"

using namespace eip_transport_test;

void test_drain_applies_multiple_datagrams_one_cycle(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);

  const Bytes assy0(52, 0xA0);
  const Bytes assy1(52, 0xB1);
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy0, false));
  d.udp.enqueueResponse(
      eip::buildClass1OutputCpf(d.fo1.to_connection_id, 1, 1, assy1, false));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(1));
  TEST_ASSERT_TRUE(d.img0.isOnline());
  TEST_ASSERT_TRUE(d.img1.isOnline());

  Bytes fb0;
  Bytes fb1;
  TEST_ASSERT_TRUE(d.img0.getFeedback(fb0));
  TEST_ASSERT_TRUE(d.img1.getFeedback(fb1));
  TEST_ASSERT_EQUAL_HEX8(0xA0, fb0[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB1, fb1[0]);

  d.scanner->disconnect();
}

// Regression (bench log 2026-10-13): priming drained T->O, then
// beginCyclicExchange reset the per-cycle flag and "first T->O" was logged a
// second time from inside the Class 1 critical section.
void test_first_to_reported_once_across_cyclic_start(void) {
  DualConnected d;
  d.udp.setEmptyReturnsZero(true);
  const Bytes assy(52, 0x11);

  d.udp.enqueueResponse(eip::buildClass1OutputCpf(d.fo0.to_connection_id, 1, 1, assy, false));
  d.udp.enqueueResponse(eip::buildClass1OutputCpf(d.fo1.to_connection_id, 1, 1, assy, false));
  (void)d.scanner->drainBufferedInputs();
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(d.scanner->pendingLogCount()));
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(d.scanner->flushDeferredLogs()));

  d.scanner->beginCyclicExchange();
  d.udp.enqueueResponse(eip::buildClass1OutputCpf(d.fo0.to_connection_id, 2, 2, assy, false));
  d.udp.enqueueResponse(eip::buildClass1OutputCpf(d.fo1.to_connection_id, 2, 2, assy, false));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(d.scanner->exchangeOnce(0)));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(0));
  TEST_ASSERT_TRUE(d.scanner->axisReceivedLastCycle(1));
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(d.scanner->pendingLogCount()));

  d.scanner->disconnect();
}

void test_class1_deferred_log_fifo_and_overflow(void) {
  eip::Class1DeferredLog q;
  eip::Class1LogEntry e;
  TEST_ASSERT_FALSE(q.pop(e));

  TEST_ASSERT_TRUE(q.push(eip::Class1LogEvent::kHcs01Stale, 2, 15788));
  TEST_ASSERT_TRUE(q.push(eip::Class1LogEvent::kStaleTeardown, 0, 6569, 2000));
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(q.size()));

  TEST_ASSERT_TRUE(q.pop(e));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::Class1LogEvent::kHcs01Stale),
                        static_cast<int>(e.event));
  TEST_ASSERT_EQUAL_UINT8(2, e.axis);
  TEST_ASSERT_EQUAL_UINT32(15788u, e.a);
  TEST_ASSERT_TRUE(q.pop(e));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::Class1LogEvent::kStaleTeardown),
                        static_cast<int>(e.event));
  TEST_ASSERT_EQUAL_UINT32(2000u, e.b);

  // Full queue drops (never blocks) and counts the drops once.
  for (size_t i = 0; i < eip::Class1DeferredLog::kCapacity; ++i) {
    TEST_ASSERT_TRUE(q.push(eip::Class1LogEvent::kHcs01OtFailed, 2));
  }
  TEST_ASSERT_FALSE(q.push(eip::Class1LogEvent::kHcs01OtFailed, 2));
  TEST_ASSERT_FALSE(q.push(eip::Class1LogEvent::kHcs01OtFailed, 2));
  TEST_ASSERT_EQUAL_UINT32(2u, q.takeDropped());
  TEST_ASSERT_EQUAL_UINT32(0u, q.takeDropped());
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(eip::Class1DeferredLog::kCapacity),
                           static_cast<uint32_t>(q.size()));
  q.clear();
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(q.size()));
}

void test_scanner_policy_cip_timeout_multiplier(void) {
  // Code 2 => drive timeout 16x RPI, above our 10x RPI stale teardown.
  TEST_ASSERT_EQUAL_UINT8(2, eip::kClass1ConnectionTimeoutMultiplier);
  eip::KinetixScannerArgs ka;
  ka.ip = "192.168.1.20";
  TEST_ASSERT_EQUAL_UINT8(2, eip::makeKinetixScannerConfig(ka).connection_timeout_multiplier);
  TEST_ASSERT_EQUAL_UINT8(2, eip::makeThetaScannerConfig().connection_timeout_multiplier);
  const uint32_t drive_timeout_rpi = 4u << eip::kClass1ConnectionTimeoutMultiplier;
  TEST_ASSERT_TRUE(drive_timeout_rpi > eip::kClass1StaleTeardownRpiMultiples);
}

void test_hcs01_stale_does_not_teardown_kinetix(void) {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeTcpClient tcp2;
  FakeUdpEndpoint udp;
  eip::EipProcessImage img0;
  eip::EipProcessImage img1;
  eip::EipProcessImage img2;

  eip::ForwardOpenReply fo0 = makeSampleForwardOpenReply();
  fo0.ot_connection_id = 0x10000001;
  fo0.to_connection_id = 0x20000001;
  fo0.connection_serial = 0x0001;
  fo0.to_api_us = 1000;

  eip::ForwardOpenReply fo1 = makeSampleForwardOpenReply();
  fo1.ot_connection_id = 0x10000002;
  fo1.to_connection_id = 0x20000002;
  fo1.connection_serial = 0x0002;
  fo1.to_api_us = 1000;

  eip::ForwardOpenReply fo2 = makeSampleForwardOpenReply();
  fo2.ot_connection_id = 0x10000003;
  fo2.to_connection_id = 0x20000003;
  fo2.connection_serial = 0x0003;
  fo2.to_api_us = 1000;

  const uint32_t h0 = 0x11110001;
  const uint32_t h1 = 0x11110002;
  const uint32_t h2 = 0x11110003;
  tcp0.enqueueResponse(makeRegisterSessionReply(h0));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardOpenMrReply(fo0)));
  tcp0.enqueueResponse(makeSendRRDataEncapReply(h0, makeForwardCloseMrReply()));
  tcp0.enqueueResponse(makeRegisterSessionReply(0));
  tcp1.enqueueResponse(makeRegisterSessionReply(h1));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardOpenMrReply(fo1)));
  tcp1.enqueueResponse(makeSendRRDataEncapReply(h1, makeForwardCloseMrReply()));
  tcp1.enqueueResponse(makeRegisterSessionReply(0));
  tcp2.enqueueResponse(makeRegisterSessionReply(h2));
  tcp2.enqueueResponse(makeSendRRDataEncapReply(h2, makeForwardOpenMrReply(fo2)));
  tcp2.enqueueResponse(makeSendRRDataEncapReply(h2, makeForwardCloseMrReply()));
  tcp2.enqueueResponse(makeRegisterSessionReply(0));

  eip::ScannerConfig cfg0 = makeTestScannerConfig();
  cfg0.target_ip = "192.168.1.20";
  cfg0.connection_serial = 0x0001;
  cfg0.ot_connection_id = 0x10000001;
  cfg0.to_connection_id = 0x20000001;

  eip::ScannerConfig cfg1 = makeTestScannerConfig();
  cfg1.target_ip = "192.168.1.21";
  cfg1.connection_serial = 0x0002;
  cfg1.ot_connection_id = 0x10000002;
  cfg1.to_connection_id = 0x20000002;

  eip::ScannerConfig cfg2 = makeTestScannerConfig();
  cfg2.target_ip = "192.168.1.23";
  cfg2.drive_family = eip::ScannerConfig::DriveFamily::kHcs01;
  cfg2.config_assembly_instance = 0;
  cfg2.ot_assembly_instance = 101;
  cfg2.to_assembly_instance = 102;
  cfg2.ot_assembly_size = 18;
  cfg2.to_assembly_size = 14;
  cfg2.connection_serial = 0x0003;
  cfg2.ot_connection_id = 0x10000003;
  cfg2.to_connection_id = 0x20000003;

  eip::MultiAxisSlot slots[3] = {{cfg0, &img0}, {cfg1, &img1}, {cfg2, &img2}};
  eip::ITcpClient* tcps[3] = {&tcp0, &tcp1, &tcp2};
  eip::EipMultiScanner scanner(tcps, 3, udp, slots);
  TEST_ASSERT_TRUE(scanner.openAxis(0));
  TEST_ASSERT_TRUE(scanner.bindSharedUdp());
  TEST_ASSERT_TRUE(scanner.openAxis(1));
  TEST_ASSERT_TRUE(scanner.openAxis(2));

  udp.setEmptyReturnsZero(true);
  const Bytes kx(52, 0xA0);
  const Bytes kz(52, 0xB1);
  auto feedKinetix = [&]() {
    udp.enqueueResponse(
        eip::buildClass1OutputCpf(fo0.to_connection_id, 1, 1, kx, false));
    udp.enqueueResponse(
        eip::buildClass1OutputCpf(fo1.to_connection_id, 1, 1, kz, false));
  };

  feedKinetix();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));

  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  feedKinetix();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));

  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  feedKinetix();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));
  TEST_ASSERT_TRUE(img0.isOnline());
  TEST_ASSERT_TRUE(img1.isOnline());
  TEST_ASSERT_FALSE(img2.isOnline());

  scanner.disconnect();
}

