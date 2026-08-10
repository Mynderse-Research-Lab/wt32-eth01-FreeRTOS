#include "unity.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include "CipMessageRouter.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipIoConnection.h"
#include "EipMultiScanner.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipSession.h"
#include "EipTransport.h"

void setUp(void) {}
void tearDown(void) {}

using eip::Bytes;

namespace {

class FakeTcpClient : public eip::ITcpClient {
 public:
  void enqueueResponse(Bytes response) {
    responses_.push_back(std::move(response));
  }
  const Bytes& lastSent() const { return last_sent_; }
  const std::vector<Bytes>& allSent() const { return all_sent_; }

  bool sentContainsCipService(uint8_t service) const {
    for (const Bytes& frame : all_sent_) {
      for (size_t i = 0; i + 1 < frame.size(); ++i) {
        if (frame[i] == service) return true;
      }
    }
    return false;
  }

  bool connect(const char*, uint16_t) override { return true; }
  ssize_t send(const uint8_t* data, size_t len) override {
    last_sent_.assign(data, data + len);
    all_sent_.push_back(last_sent_);
    return static_cast<ssize_t>(len);
  }
  ssize_t recv(uint8_t* buf, size_t max_len, uint32_t) override {
    if (response_index_ >= responses_.size()) return -1;
    const Bytes& r = responses_[response_index_++];
    const size_t n = std::min(max_len, r.size());
    std::memcpy(buf, r.data(), n);
    return static_cast<ssize_t>(n);
  }
  void close() override {}
  bool isConnected() const override { return true; }

 private:
  std::vector<Bytes> responses_;
  size_t response_index_ = 0;
  Bytes last_sent_;
  std::vector<Bytes> all_sent_;
};

class FakeUdpEndpoint : public eip::IUdpEndpoint {
 public:
  void enqueueResponse(Bytes response) {
    responses_.push_back(std::move(response));
  }
  void setEcho(bool echo) { echo_ = echo; }
  const Bytes& lastSent() const { return last_sent_; }

  bool bind(uint16_t, uint32_t = 0) override { return true; }
  ssize_t sendTo(const uint8_t* data, size_t len, uint32_t, uint16_t) override {
    last_sent_.assign(data, data + len);
    if (echo_) echo_frame_.assign(data, data + len);
    return static_cast<ssize_t>(len);
  }
  ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t) override {
    if (echo_ && !echo_frame_.empty()) {
      const size_t n = std::min(max_len, echo_frame_.size());
      std::memcpy(buf, echo_frame_.data(), n);
      echo_frame_.clear();
      return static_cast<ssize_t>(n);
    }
    if (response_index_ >= responses_.size()) return -1;
    const Bytes& r = responses_[response_index_++];
    const size_t n = std::min(max_len, r.size());
    std::memcpy(buf, r.data(), n);
    return static_cast<ssize_t>(n);
  }
  void close() override {}

 private:
  std::vector<Bytes> responses_;
  size_t response_index_ = 0;
  Bytes last_sent_;
  bool echo_ = false;
  Bytes echo_frame_;
};

Bytes makeRegisterSessionReply(uint32_t handle) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = handle;
  h.status = 0;
  const Bytes data = {0x01, 0x00, 0x00, 0x00};
  return eip::encodeEncapsulation(h, data);
}

Bytes makeSendRRDataEncapReply(uint32_t session_handle, const Bytes& cip_reply) {
  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = session_handle;
  rh.status = 0;
  const Bytes payload = eip::encodeSendRRDataPayload(cip_reply);
  return eip::encodeEncapsulation(rh, payload);
}

Bytes makeForwardOpenMrReply(const eip::ForwardOpenReply& fo) {
  Bytes data;
  eip::ByteWriter w(data);
  w.u32(fo.ot_connection_id);
  w.u32(fo.to_connection_id);
  w.u16(fo.connection_serial);
  w.u16(fo.originator_vendor_id);
  w.u32(fo.originator_serial);
  w.u32(fo.ot_api_us);
  w.u32(fo.to_api_us);
  w.u8(0);
  w.u8(0);

  Bytes cip;
  eip::ByteWriter cw(cip);
  cw.u8(static_cast<uint8_t>(eip::CipService::kForwardOpen) |
        eip::kCipReplyServiceMask);
  cw.u8(0);
  cw.u8(0);
  cw.u8(0);
  cw.bytes(data);
  return cip;
}

Bytes makeForwardCloseMrReply() {
  Bytes cip;
  eip::ByteWriter cw(cip);
  cw.u8(static_cast<uint8_t>(eip::CipService::kForwardClose) |
        eip::kCipReplyServiceMask);
  cw.u8(0);
  cw.u8(0);
  cw.u8(0);
  cw.u32(0);  // connection ID
  cw.u16(0);  // connection serial
  cw.u16(0);  // vendor
  cw.u32(0);  // originator serial
  return cip;
}

eip::ForwardOpenReply makeSampleForwardOpenReply() {
  eip::ForwardOpenReply fo;
  fo.ot_connection_id = 0x10000001;
  fo.to_connection_id = 0x20000002;
  fo.connection_serial = 0x0005;
  fo.originator_vendor_id = 0x004D;
  fo.originator_serial = 0xCAFEF00D;
  fo.ot_api_us = 20000;
  fo.to_api_us = 20000;
  return fo;
}

eip::ScannerConfig makeTestScannerConfig() {
  eip::ScannerConfig cfg;
  cfg.target_ip = "192.168.1.100";
  cfg.connection_serial = 0x0005;
  cfg.originator_serial = 0xCAFEF00D;
  cfg.ot_connection_id = 0x10000001;
  return cfg;
}

}  // namespace

static void test_session_register(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x0A0B0C0D));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());
  TEST_ASSERT_EQUAL_HEX32(0x0A0B0C0D, session.sessionHandle());

  TEST_ASSERT_EQUAL_HEX8(0x65, tcp.lastSent()[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, tcp.lastSent()[1]);
}

static void test_class1_cpf_bytes(void) {
  const Bytes assembly = {0xDE, 0xAD};
  Bytes cpf = eip::buildClass1OutputCpf(0x10000001, 7, 42, assembly, true);

  const Bytes expected = {
      0x02, 0x00,
      0x02, 0x80, 0x08, 0x00,
      0x01, 0x00, 0x00, 0x10,
      0x07, 0x00, 0x00, 0x00,
      0xB1, 0x00, 0x08, 0x00,
      0x2A, 0x00,
      0x01, 0x00, 0x00, 0x00,
      0xDE, 0xAD};
  TEST_ASSERT_EQUAL_UINT32(expected.size(), cpf.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), cpf.data(), expected.size());
}

static void test_io_connection_output_frame(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x20000002;
  cfg.session_handle = 0x12345678;
  cfg.ot_include_run_idle_header = false;
  cfg.to_include_run_idle_header = false;
  io.setConfig(cfg);

  const Bytes assembly = {0x11, 0x22, 0x33};
  Bytes frame = io.buildOutputFrame(assembly);

  // Class 1 implicit O->T is a raw CPF UDP payload (no encapsulation header).
  // Layout: 2 item count + 12 sequenced address + 4 item header + 2 seq + 3 data = 23.
  TEST_ASSERT_EQUAL_UINT32(23u, frame.size());
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[0]);  // item count (little-endian)
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[2]);  // sequenced address type 0x8002
  TEST_ASSERT_EQUAL_HEX8(0x80, frame[3]);
  // bytes 4-5: sequenced address item length = 8
  TEST_ASSERT_EQUAL_HEX8(0x08, frame[4]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[5]);
  // bytes 6-9: connection ID 0x20000002 (little-endian)
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[6]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[7]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[8]);
  TEST_ASSERT_EQUAL_HEX8(0x20, frame[9]);
  // bytes 10-13: encapsulation sequence 1
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[10]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[11]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[12]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[13]);
  // bytes 14-15: connected data type 0x00B1
  TEST_ASSERT_EQUAL_HEX8(0xB1, frame[14]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[15]);
  // bytes 16-17: item length = 5
  TEST_ASSERT_EQUAL_HEX8(0x05, frame[16]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[17]);
  // bytes 18-19: CIP sequence count
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[18]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[19]);
  // bytes 20-22: assembly data
  TEST_ASSERT_EQUAL_HEX8(0x11, frame[20]);
  TEST_ASSERT_EQUAL_HEX8(0x22, frame[21]);
  TEST_ASSERT_EQUAL_HEX8(0x33, frame[22]);
  // Validate the produced CPF is also self-parsable.
  {
    Bytes parsed;
    TEST_ASSERT_TRUE(io.parseInputFrame(frame, parsed));
    TEST_ASSERT_EQUAL_UINT32(assembly.size(), parsed.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), parsed.data(), parsed.size());
  }
  TEST_ASSERT_EQUAL_UINT16(1, io.encapSequence());
  TEST_ASSERT_EQUAL_UINT16(1, io.cipSequence());
}

static void test_io_connection_parse_roundtrip(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x10000001;
  cfg.session_handle = 0xABCDEF01;
  cfg.ot_include_run_idle_header = true;
  cfg.to_include_run_idle_header = true;
  io.setConfig(cfg);

  const Bytes assembly_in = {0xAA, 0xBB};
  Bytes frame = io.buildOutputFrame(assembly_in);

  Bytes parsed;
  TEST_ASSERT_TRUE(io.parseInputFrame(frame, parsed));
  TEST_ASSERT_EQUAL_UINT32(assembly_in.size(), parsed.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly_in.data(), parsed.data(), parsed.size());
}

static void test_io_connection_parse_t_to_o_cpf(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x10000001;
  cfg.to_include_run_idle_header = false;
  io.setConfig(cfg);

  // Raw T->O CPF: 1 sequenced address item + 1 connected data item.
  const Bytes assembly = {0x10, 0x11, 0x12, 0x13};
  Bytes cpf;
  eip::ByteWriter w(cpf);
  w.u16(2);                            // item count
  w.u16(0x8002);                       // sequenced address type
  w.u16(8);                            // length
  w.u32(0x20000002);                   // connection ID
  w.u32(1);                            // sequence
  w.u16(0x00B1);                       // connected data type
  w.u16(static_cast<uint16_t>(2 + assembly.size()));  // length
  w.u16(1);                            // CIP sequence count
  w.bytes(assembly);

  Bytes parsed;
  TEST_ASSERT_TRUE(io.parseInputFrame(cpf, parsed));
  TEST_ASSERT_EQUAL_UINT32(assembly.size(), parsed.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), parsed.data(), parsed.size());
}

static void test_session_explicit_roundtrip(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x11111111));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());

  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = 0x11111111;
  rh.status = 0;
  const Bytes cip_reply = {0x8E, 0x00, 0x00, 0x00};
  Bytes payload = eip::encodeSendRRDataPayload(cip_reply);
  tcp.enqueueResponse(eip::encodeEncapsulation(rh, payload));

  const Bytes cip_req = {0x0E, 0x03, 0x20, 0x04, 0x24, 0x9A, 0x30, 0x03};
  Bytes cip_resp;
  TEST_ASSERT_TRUE(session.sendExplicit(cip_req, cip_resp));
  TEST_ASSERT_EQUAL_UINT32(cip_reply.size(), cip_resp.size());
}

static void test_scanner_connect_forward_open(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  const uint32_t handle = 0x11111111;
  const eip::ForwardOpenReply fo = makeSampleForwardOpenReply();

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(fo)));

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());
  TEST_ASSERT_EQUAL(eip::EipScanner::State::kConnected, scanner.state());
  TEST_ASSERT_EQUAL_HEX32(fo.ot_connection_id, scanner.openReply().ot_connection_id);
  TEST_ASSERT_EQUAL_HEX32(fo.to_connection_id, scanner.openReply().to_connection_id);
  TEST_ASSERT_TRUE(tcp.sentContainsCipService(
      static_cast<uint8_t>(eip::CipService::kForwardOpen)));
  scanner.disconnect();
}

static void test_scanner_full_lifecycle(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  udp.setEcho(true);
  const uint32_t handle = 0x22222222;
  const eip::ForwardOpenReply fo = makeSampleForwardOpenReply();

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(fo)));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardCloseMrReply()));
  tcp.enqueueResponse(makeRegisterSessionReply(0));  // UnRegisterSession reply

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());

  Bytes input;
  TEST_ASSERT_TRUE(scanner.exchangeOnce(scanner.recvTimeoutMs(), input));
  TEST_ASSERT_GREATER_THAN(0, input.size());

  scanner.disconnect();
  TEST_ASSERT_EQUAL(eip::EipScanner::State::kIdle, scanner.state());
  TEST_ASSERT_TRUE(tcp.sentContainsCipService(
      static_cast<uint8_t>(eip::CipService::kForwardClose)));
}

static void test_scanner_recv_timeout(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  const uint32_t handle = 0x33333333;

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(makeSendRRDataEncapReply(
      handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());

  Bytes input;
  TEST_ASSERT_FALSE(scanner.exchangeOnce(1, input));

  scanner.disconnect();
}

static void test_parse_class1_input_cpf_demux_id(void) {
  const Bytes assembly = {0x01, 0x02, 0x03, 0x04};
  const Bytes frame =
      eip::buildClass1OutputCpf(0xAABBCCDD, 9, 3, assembly, false);

  uint32_t cid = 0;
  Bytes out;
  TEST_ASSERT_TRUE(eip::parseClass1InputCpf(frame, cid, out, false));
  TEST_ASSERT_EQUAL_HEX32(0xAABBCCDD, cid);
  TEST_ASSERT_EQUAL_UINT32(assembly.size(), out.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), out.data(), out.size());
}

static void test_multi_scanner_dual_demux(void) {
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

static void test_class1_rpi_remainder_and_miss_policy(void) {
  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderMs(5, 5));
  TEST_ASSERT_EQUAL_UINT32(0, eip::rpiRemainderMs(5, 8));
  TEST_ASSERT_EQUAL_UINT32(3, eip::rpiRemainderMs(5, 2));
  TEST_ASSERT_EQUAL_UINT32(1, eip::rpiRemainderMs(0, 0));

  TEST_ASSERT_FALSE(eip::shouldTeardownAfterInputMisses(0));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterInputMisses(2));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(3));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(4));
}

static void test_multi_scanner_input_miss_status(void) {
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

  // No T->O enqueued → input miss (O->T send still succeeds on FakeUdp).
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kInputMiss),
                        static_cast<int>(scanner.exchangeOnce(5)));

  scanner.disconnect();
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_session_register);
  RUN_TEST(test_class1_cpf_bytes);
  RUN_TEST(test_io_connection_output_frame);
  RUN_TEST(test_io_connection_parse_roundtrip);
  RUN_TEST(test_io_connection_parse_t_to_o_cpf);
  RUN_TEST(test_session_explicit_roundtrip);
  RUN_TEST(test_scanner_connect_forward_open);
  RUN_TEST(test_scanner_full_lifecycle);
  RUN_TEST(test_scanner_recv_timeout);
  RUN_TEST(test_parse_class1_input_cpf_demux_id);
  RUN_TEST(test_multi_scanner_dual_demux);
  RUN_TEST(test_class1_rpi_remainder_and_miss_policy);
  RUN_TEST(test_multi_scanner_input_miss_status);
  return UNITY_END();
}
