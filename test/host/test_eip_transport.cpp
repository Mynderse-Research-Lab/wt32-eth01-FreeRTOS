#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "CipMessageRouter.h"
#include "EipClass1Timing.h"
#include "EipClass1TimingStats.h"
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
  void setEmptyReturnsZero(bool v) { empty_returns_zero_ = v; }
  void setFailSend(bool v) { fail_send_ = v; }
  void setFloodFrame(Bytes frame) {
    flood_ = true;
    flood_frame_ = std::move(frame);
  }
  // Batch mode packs every pending frame into one recvBatch call, the way the
  // W5500 endpoint drains the RX FIFO in a single burst.
  void setBatchMode(bool v) { batch_ = v; }
  const Bytes& lastSent() const { return last_sent_; }
  size_t recvCallCount() const { return recv_calls_; }
  size_t batchCallCount() const { return batch_calls_; }
  size_t batchDatagramCount() const { return batch_datagrams_; }

  bool bind(uint16_t, uint32_t = 0) override { return true; }
  ssize_t sendTo(const uint8_t* data, size_t len, uint32_t, uint16_t) override {
    if (fail_send_) return -1;
    last_sent_.assign(data, data + len);
    if (echo_) echo_frame_.assign(data, data + len);
    return static_cast<ssize_t>(len);
  }
  ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t) override {
    ++recv_calls_;
    if (flood_) {
      const size_t n = std::min(max_len, flood_frame_.size());
      std::memcpy(buf, flood_frame_.data(), n);
      return static_cast<ssize_t>(n);
    }
    if (echo_ && !echo_frame_.empty()) {
      const size_t n = std::min(max_len, echo_frame_.size());
      std::memcpy(buf, echo_frame_.data(), n);
      echo_frame_.clear();
      return static_cast<ssize_t>(n);
    }
    if (response_index_ >= responses_.size()) {
      return empty_returns_zero_ ? 0 : -1;
    }
    const Bytes& r = responses_[response_index_++];
    const size_t n = std::min(max_len, r.size());
    std::memcpy(buf, r.data(), n);
    return static_cast<ssize_t>(n);
  }
  size_t recvBatch(uint8_t* buf, size_t buf_len, eip::UdpDatagramView* views,
                   size_t max_views) override {
    if (!batch_) {
      return eip::IUdpEndpoint::recvBatch(buf, buf_len, views, max_views);
    }
    ++batch_calls_;
    size_t n = 0;
    size_t off = 0;
    while (n < max_views) {
      const Bytes* frame = nullptr;
      if (flood_) {
        frame = &flood_frame_;
      } else if (response_index_ < responses_.size()) {
        frame = &responses_[response_index_++];
      } else {
        break;
      }
      if (off + frame->size() > buf_len) break;
      std::memcpy(buf + off, frame->data(), frame->size());
      views[n].data = buf + off;
      views[n].len = frame->size();
      off += frame->size();
      ++n;
    }
    batch_datagrams_ += n;
    return n;
  }
  void close() override {}

 private:
  std::vector<Bytes> responses_;
  size_t response_index_ = 0;
  size_t recv_calls_ = 0;
  size_t batch_calls_ = 0;
  size_t batch_datagrams_ = 0;
  Bytes last_sent_;
  bool echo_ = false;
  bool empty_returns_zero_ = false;
  bool fail_send_ = false;
  bool flood_ = false;
  bool batch_ = false;
  Bytes echo_frame_;
  Bytes flood_frame_;
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

static void test_parse_class1_input_cpf_view(void) {
  const Bytes assembly = {0x11, 0x22, 0x33, 0x44, 0x55};
  const Bytes frame =
      eip::buildClass1OutputCpf(0xDEADBEEF, 5, 7, assembly, false);

  uint32_t cid_copy = 0;
  Bytes out_copy;
  TEST_ASSERT_TRUE(
      eip::parseClass1InputCpf(frame, cid_copy, out_copy, false));

  uint32_t cid_view = 0;
  const uint8_t* assy = nullptr;
  size_t assy_len = 0;
  TEST_ASSERT_TRUE(eip::parseClass1InputCpfView(
      frame.data(), frame.size(), cid_view, assy, assy_len, false));
  TEST_ASSERT_EQUAL_HEX32(cid_copy, cid_view);
  TEST_ASSERT_EQUAL_UINT32(out_copy.size(), assy_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(out_copy.data(), assy, assy_len);
  // View pointer must lie inside the input frame buffer.
  TEST_ASSERT_TRUE(assy >= frame.data());
  TEST_ASSERT_TRUE(assy + assy_len <= frame.data() + frame.size());

  // Truncated frame
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(frame.data(), 3, cid_view,
                                                 assy, assy_len, false));

  // Bad item length (claims more bytes than remain)
  Bytes bad = frame;
  // Sequenced-address item length at bytes 4-5; inflate past end.
  bad[4] = 0xFF;
  bad[5] = 0x00;
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(bad.data(), bad.size(),
                                                 cid_view, assy, assy_len,
                                                 false));

  // No connected-data item: only sequenced address
  Bytes addr_only;
  {
    eip::ByteWriter w(addr_only);
    w.u16(1);  // one item
    w.u16(0x8002);
    w.u16(8);
    w.u32(0x12345678);
    w.u32(1);
  }
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(
      addr_only.data(), addr_only.size(), cid_view, assy, assy_len, false));
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
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterInputMisses(2));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(3));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterInputMisses(4));

  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(0, 2000));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(1999, 2000));
  TEST_ASSERT_FALSE(eip::shouldTeardownAfterStaleUs(5999, 2000));
  TEST_ASSERT_TRUE(eip::shouldTeardownAfterStaleUs(6000, 2000));
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
static void test_stale_teardown_boundary(void) {
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

static void test_class1_timing_stats_ring_and_cmd_to_start(void) {
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

static void test_class1_timing_drain_ring_and_pace_counters(void) {
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

static void test_multi_scanner_input_miss_status(void) {
  FakeTcpClient tcp0;
  FakeTcpClient tcp1;
  FakeUdpEndpoint udp;

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

  // Grace ends at 3x RPI (@1000 us = 3 ms) and reseeds the stale clock.
  // Real starvation needs another 3x RPI after that → miss.
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kOk),
                        static_cast<int>(scanner.exchangeOnce(0)));
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(eip::ExchangeStatus::kInputMiss),
                        static_cast<int>(scanner.exchangeOnce(0)));

  scanner.disconnect();
}

// Regression: axis0 FO→axis1 FO gap must not look like T->O starvation on the
// first cyclic exchange (bench: ~70 ms FO gap vs 3x2000 us = 6 ms).
static void test_fo_delay_before_first_exchange_not_stale(void) {
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

namespace {

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

static void test_drain_applies_multiple_datagrams_one_cycle(void) {
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

static void test_hcs01_stale_does_not_teardown_kinetix(void) {
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

static void test_drain_ignores_duplicate_cid(void) {
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

static void test_drain_ignores_unknown_cid(void) {
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

static void test_drain_stops_when_all_axes_got(void) {
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

static void test_drain_is_bounded(void) {
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
static void test_exchange_records_drain_sample(void) {
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
static void test_batch_drain_applies_both_axes_in_one_call(void) {
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
static void test_batch_drain_is_bounded(void) {
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

static void test_feedback_age_resets_on_fresh_data(void) {
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

static void test_exchange_reports_send_failure(void) {
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
  RUN_TEST(test_parse_class1_input_cpf_view);
  RUN_TEST(test_multi_scanner_dual_demux);
  RUN_TEST(test_class1_rpi_remainder_and_miss_policy);
  RUN_TEST(test_stale_teardown_boundary);
  RUN_TEST(test_class1_timing_stats_ring_and_cmd_to_start);
  RUN_TEST(test_class1_timing_drain_ring_and_pace_counters);
  RUN_TEST(test_multi_scanner_input_miss_status);
  RUN_TEST(test_fo_delay_before_first_exchange_not_stale);
  RUN_TEST(test_drain_applies_multiple_datagrams_one_cycle);
  RUN_TEST(test_hcs01_stale_does_not_teardown_kinetix);
  RUN_TEST(test_drain_ignores_duplicate_cid);
  RUN_TEST(test_drain_ignores_unknown_cid);
  RUN_TEST(test_drain_stops_when_all_axes_got);
  RUN_TEST(test_drain_is_bounded);
  RUN_TEST(test_exchange_records_drain_sample);
  RUN_TEST(test_batch_drain_applies_both_axes_in_one_call);
  RUN_TEST(test_batch_drain_is_bounded);
  RUN_TEST(test_feedback_age_resets_on_fresh_data);
  RUN_TEST(test_exchange_reports_send_failure);
  return UNITY_END();
}
