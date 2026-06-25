// Host unit tests for EtherNet/IP transport scaffolding (session + Class 1 IO).
// Uses fake TCP/UDP clients; proves framing is self-consistent (not bench-validated).

#include "unity.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipIoConnection.h"
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

  bool connect(const char*, uint16_t) override { return true; }
  ssize_t send(const uint8_t* data, size_t len) override {
    last_sent_.assign(data, data + len);
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
};

class FakeUdpEndpoint : public eip::IUdpEndpoint {
 public:
  void enqueueResponse(Bytes response) {
    responses_.push_back(std::move(response));
  }
  const Bytes& lastSent() const { return last_sent_; }

  bool bind(uint16_t) override { return true; }
  ssize_t sendTo(const uint8_t* data, size_t len, const char*, uint16_t) override {
    last_sent_.assign(data, data + len);
    return static_cast<ssize_t>(len);
  }
  ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t) override {
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
};

Bytes makeRegisterSessionReply(uint32_t handle) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = handle;
  h.status = 0;
  const Bytes data = {0x01, 0x00, 0x00, 0x00};
  return eip::encodeEncapsulation(h, data);
}

}  // namespace

static void test_session_register(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x0A0B0C0D));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());
  TEST_ASSERT_EQUAL_HEX32(0x0A0B0C0D, session.sessionHandle());

  // First bytes of sent frame: RegisterSession command 0x0065 LE.
  TEST_ASSERT_EQUAL_HEX8(0x65, tcp.lastSent()[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, tcp.lastSent()[1]);
}

static void test_class1_cpf_bytes(void) {
  const Bytes assembly = {0xDE, 0xAD};
  Bytes cpf = eip::buildClass1OutputCpf(0x10000001, 7, 42, assembly, true);

  // count=2; seq addr 0x8002 len 8; conn data 0x00B1 len 8 (2+4+2)
  const Bytes expected = {
      0x02, 0x00,
      0x02, 0x80, 0x08, 0x00,
      0x01, 0x00, 0x00, 0x10,  // connection_id
      0x07, 0x00, 0x00, 0x00,  // encap_seq
      0xB1, 0x00, 0x08, 0x00,
      0x2A, 0x00,              // cip_seq 42
      0x01, 0x00, 0x00, 0x00,  // Run/Idle PROVISIONAL
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
  cfg.include_run_idle_header = false;
  io.setConfig(cfg);

  const Bytes assembly = {0x11, 0x22, 0x33};
  Bytes frame = io.buildOutputFrame(assembly);

  TEST_ASSERT_GREATER_THAN(eip::kEncapHeaderSize, frame.size());
  TEST_ASSERT_EQUAL_HEX8(0x70, frame[0]);  // SendUnitData
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[1]);
  TEST_ASSERT_EQUAL_HEX32(0x12345678,
                          static_cast<uint32_t>(frame[4]) |
                              (static_cast<uint32_t>(frame[5]) << 8) |
                              (static_cast<uint32_t>(frame[6]) << 16) |
                              (static_cast<uint32_t>(frame[7]) << 24));
  TEST_ASSERT_EQUAL_UINT16(1, io.encapSequence());
  TEST_ASSERT_EQUAL_UINT16(1, io.cipSequence());
}

static void test_io_connection_parse_roundtrip(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x10000001;
  cfg.session_handle = 0xABCDEF01;
  cfg.include_run_idle_header = true;
  io.setConfig(cfg);

  const Bytes assembly_in = {0xAA, 0xBB};
  Bytes frame = io.buildOutputFrame(assembly_in);

  Bytes parsed;
  TEST_ASSERT_TRUE(io.parseInputFrame(frame, parsed));
  TEST_ASSERT_EQUAL_UINT32(assembly_in.size(), parsed.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly_in.data(), parsed.data(), parsed.size());
}

static void test_session_explicit_roundtrip(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x11111111));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());

  // Build SendRRData reply with empty CIP response.
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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_session_register);
  RUN_TEST(test_class1_cpf_bytes);
  RUN_TEST(test_io_connection_output_frame);
  RUN_TEST(test_io_connection_parse_roundtrip);
  RUN_TEST(test_session_explicit_roundtrip);
  return UNITY_END();
}
