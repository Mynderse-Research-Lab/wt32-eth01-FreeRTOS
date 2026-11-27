/**
 * @file test_eip_transport_rigs.h
 * @brief Multi-axis rigs for the EIP transport host tests.
 */

#pragma once

#include "test_eip_transport_internal.h"

namespace eip_transport_test {

struct DualConnected {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeUdpEndpoint udp;
  eip::EipProcessImage img0;
  eip::EipProcessImage img1;
  eip::ForwardOpenReply fo0;
  eip::ForwardOpenReply fo1;
  eip::ScannerConfig cfg0;
  eip::ScannerConfig cfg1;
  eip::MultiAxisSlot slots[2];
  eip::ITcpClient* tcps[2];
  std::unique_ptr<eip::EipMultiScanner> scanner;

  DualConnected(uint32_t to_api_us = 20000, bool with_images = true) {
    fo0 = makeSampleForwardOpenReply();
    fo0.ot_connection_id = 0x10000001;
    fo0.to_connection_id = 0x20000001;
    fo0.connection_serial = 0x0001;
    fo0.to_api_us = to_api_us;

    fo1 = makeSampleForwardOpenReply();
    fo1.ot_connection_id = 0x10000002;
    fo1.to_connection_id = 0x20000002;
    fo1.connection_serial = 0x0002;
    fo1.to_api_us = to_api_us;

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

    cfg0 = makeTestScannerConfig();
    cfg0.target_ip = "192.168.1.20";
    cfg0.connection_serial = 0x0001;
    cfg0.ot_connection_id = 0x10000001;
    cfg0.to_connection_id = 0x20000001;
    cfg0.connection_timeout_multiplier = 7;

    cfg1 = makeTestScannerConfig();
    cfg1.target_ip = "192.168.1.21";
    cfg1.connection_serial = 0x0002;
    cfg1.ot_connection_id = 0x10000002;
    cfg1.to_connection_id = 0x20000002;
    cfg1.connection_timeout_multiplier = 7;

    slots[0] = {cfg0, with_images ? &img0 : nullptr};
    slots[1] = {cfg1, with_images ? &img1 : nullptr};
    tcps[0] = &tcp0;
    tcps[1] = &tcp1;
    scanner = std::make_unique<eip::EipMultiScanner>(tcps, 2, udp, slots);
    TEST_ASSERT_TRUE(scanner->openAxis(0));
    TEST_ASSERT_TRUE(scanner->bindSharedUdp());
    TEST_ASSERT_TRUE(scanner->openAxis(1));
  }
};


}  // namespace

namespace eip_transport_test {

// X/Z Kinetix + theta HCS01 on 1 ms RPI. `theta_reopen_ok` queues the TCP
// replies a successful theta reopen consumes (FC, RegisterSession, FO).
// UnRegisterSession has no reply, so none is queued for it.
struct TriConnected {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeTcpClient tcp2;
  FakeUdpEndpoint udp;
  eip::EipProcessImage img0;
  eip::EipProcessImage img1;
  eip::EipProcessImage img2;
  eip::ForwardOpenReply fo[3];
  eip::MultiAxisSlot slots[3];
  eip::ITcpClient* tcps[3];
  std::unique_ptr<eip::EipMultiScanner> scanner;
  const Bytes kx = Bytes(52, 0xA0);
  const Bytes kz = Bytes(52, 0xB1);
  const Bytes th = Bytes(14, 0xC2);

  explicit TriConnected(bool theta_reopen_ok) {
    FakeTcpClient* t[3] = {&tcp0, &tcp1, &tcp2};
    for (uint32_t i = 0; i < 3; ++i) {
      fo[i] = makeSampleForwardOpenReply();
      fo[i].ot_connection_id = 0x10000001u + i;
      fo[i].to_connection_id = 0x20000001u + i;
      fo[i].connection_serial = static_cast<uint16_t>(i + 1);
      fo[i].to_api_us = 1000;
      const uint32_t h = 0x11110001u + i;
      t[i]->enqueueResponse(makeRegisterSessionReply(h));
      t[i]->enqueueResponse(makeSendRRDataEncapReply(h, makeForwardOpenMrReply(fo[i])));
    }
    const uint32_t h2 = 0x11110003u;
    if (theta_reopen_ok) {
      const uint32_t h2b = 0x22220003u;
      tcp2.enqueueResponse(makeSendRRDataEncapReply(h2, makeForwardCloseMrReply()));
      tcp2.enqueueResponse(makeRegisterSessionReply(h2b));
      tcp2.enqueueResponse(makeSendRRDataEncapReply(h2b, makeForwardOpenMrReply(fo[2])));
      tcp2.enqueueResponse(makeSendRRDataEncapReply(h2b, makeForwardCloseMrReply()));
    } else {
      // FC answered, then RegisterSession gets no reply -> reopen fails.
      tcp2.enqueueResponse(makeSendRRDataEncapReply(h2, makeForwardCloseMrReply()));
    }
    tcp0.enqueueResponse(makeSendRRDataEncapReply(0x11110001u, makeForwardCloseMrReply()));
    tcp1.enqueueResponse(makeSendRRDataEncapReply(0x11110002u, makeForwardCloseMrReply()));

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

    slots[0] = {cfg0, &img0};
    slots[1] = {cfg1, &img1};
    slots[2] = {cfg2, &img2};
    tcps[0] = &tcp0;
    tcps[1] = &tcp1;
    tcps[2] = &tcp2;
    scanner = std::make_unique<eip::EipMultiScanner>(tcps, 3, udp, slots);
    TEST_ASSERT_TRUE(scanner->openAxis(0));
    TEST_ASSERT_TRUE(scanner->bindSharedUdp());
    TEST_ASSERT_TRUE(scanner->openAxis(1));
    TEST_ASSERT_TRUE(scanner->openAxis(2));
    udp.setEmptyReturnsZero(true);
  }

  void feedKinetix() {
    udp.enqueueResponse(eip::buildClass1OutputCpf(fo[0].to_connection_id, 1, 1, kx, false));
    udp.enqueueResponse(eip::buildClass1OutputCpf(fo[1].to_connection_id, 1, 1, kz, false));
  }

  // Run until theta (never answering) has been silent past the reopen threshold
  // while X/Z keep answering every exchange.
  void starveThetaPastReopenThreshold() {
    feedKinetix();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                          static_cast<int>(scanner->exchangeOnce(0)));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // past 3x grace
    feedKinetix();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                          static_cast<int>(scanner->exchangeOnce(0)));
    std::this_thread::sleep_for(
        std::chrono::microseconds(eip::kHcs01ReopenAfterStaleUs + 20000));
    feedKinetix();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                          static_cast<int>(scanner->exchangeOnce(0)));
  }
};


}  // namespace

namespace eip_transport_test {

inline
Bytes makeGetAttributeMrReply(uint8_t general_status, const Bytes& data) {
    Bytes cip;
    eip::ByteWriter w(cip);
    w.u8(static_cast<uint8_t>(eip::CipService::kGetAttributeSingle) |
         eip::kCipReplyServiceMask);
    w.u8(0);
    w.u8(general_status);
    w.u8(0);
    w.bytes(data);
    return cip;
}

// One axis whose drive answers attribute 13 with `reply` before Forward Open.
struct KeptAliveAxis {
    FakeTcpClient tcp;
    FakeUdpEndpoint udp;
    std::unique_ptr<eip::EipMultiScanner> scanner;

    explicit KeptAliveAxis(const Bytes& attr13_reply) {
        const uint32_t handle = 0x33330003;
        tcp.enqueueResponse(makeRegisterSessionReply(handle));
        tcp.enqueueResponse(makeSendRRDataEncapReply(handle, attr13_reply));
        tcp.enqueueResponse(
            makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));
        eip::ScannerConfig cfg = makeTestScannerConfig();
        cfg.read_encap_inactivity_timeout = true;
        eip::MultiAxisSlot slots[1] = {{cfg, nullptr}};
        eip::ITcpClient* tcps[1] = {&tcp};
        scanner = std::make_unique<eip::EipMultiScanner>(tcps, 1, udp, slots);
        TEST_ASSERT_TRUE(scanner->openAxis(0));
    }

    size_t nopsSent() const {
        size_t n = 0;
        for (const Bytes& f : tcp.allSent()) {
            if (f.size() == 24 && f[0] == 0x00 && f[1] == 0x00) {
                ++n;
            }
        }
        return n;
    }
};


}  // namespace

