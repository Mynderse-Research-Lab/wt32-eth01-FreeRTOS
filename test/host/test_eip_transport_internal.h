/**
 * @file test_eip_transport_internal.h
 * @brief Session and I/O fakes for the EIP transport host tests.
 */

#pragma once

#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "CipMessageRouter.h"
#include "CipTcpIpInterface.h"
#include "EipClass1DeferredLog.h"
#include "EipClass1Timing.h"
#include "EipClass1TimingStats.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipIoConnection.h"
#include "EipMultiScanner.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipScannerPolicy.h"
#include "EipSession.h"
#include "EipTransport.h"

using eip::Bytes;

namespace eip_transport_test {

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

  // The peer sends FIN: the socket stops being usable until reconnected.
  void dropByPeer() { connected_ = false; }
  // The socket still reads connected, but nothing can be sent on it.
  void failSendsUntilReconnect() { fail_sends_ = true; }
  void failConnect() { fail_connect_ = true; }
  void onSend(std::function<void()> hook) { on_send_ = std::move(hook); }
  size_t connects() const { return connects_; }

  bool connect(const char*, uint16_t) override {
    ++connects_;
    connected_ = !fail_connect_;
    fail_sends_ = false;
    return connected_;
  }
  ssize_t send(const uint8_t* data, size_t len) override {
    if (!connected_ || fail_sends_) return -1;
    if (on_send_) on_send_();
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
  bool isConnected() const override { return connected_; }

 private:
  std::vector<Bytes> responses_;
  size_t response_index_ = 0;
  Bytes last_sent_;
  std::vector<Bytes> all_sent_;
  bool connected_ = true;
  bool fail_connect_ = false;
  bool fail_sends_ = false;
  std::function<void()> on_send_;
  size_t connects_ = 0;
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

inline
Bytes makeRegisterSessionReply(uint32_t handle) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = handle;
  h.status = 0;
  const Bytes data = {0x01, 0x00, 0x00, 0x00};
  return eip::encodeEncapsulation(h, data);
}

inline
Bytes makeSendRRDataEncapReply(uint32_t session_handle, const Bytes& cip_reply) {
  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = session_handle;
  rh.status = 0;
  const Bytes payload = eip::encodeSendRRDataPayload(cip_reply);
  return eip::encodeEncapsulation(rh, payload);
}

inline
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

inline
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

inline
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

inline
eip::ScannerConfig makeTestScannerConfig() {
  eip::ScannerConfig cfg;
  cfg.target_ip = "192.168.1.100";
  cfg.connection_serial = 0x0005;
  cfg.originator_serial = 0xCAFEF00D;
  cfg.ot_connection_id = 0x10000001;
  return cfg;
}

// One axis opened through a fake drive, for the planned-close tests.
struct ClosableAxis {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  eip::EipProcessImage image;
  std::unique_ptr<eip::EipMultiScanner> scanner;

  ClosableAxis() {
    const uint32_t handle = 0x11110001;
    tcp.enqueueResponse(makeRegisterSessionReply(handle));
    tcp.enqueueResponse(
        makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));
    eip::ScannerConfig cfg = makeTestScannerConfig();
    cfg.target_ip = "192.168.1.20";
    eip::MultiAxisSlot slots[1] = {{cfg, &image}};
    eip::ITcpClient* tcps[1] = {&tcp};
    scanner = std::make_unique<eip::EipMultiScanner>(tcps, 1, udp, slots);
    TEST_ASSERT_TRUE(scanner->openAxis(0));
    TEST_ASSERT_TRUE(scanner->bindSharedUdp());
    TEST_ASSERT_TRUE(scanner->axisConnected(0));
  }
};

}  // namespace

