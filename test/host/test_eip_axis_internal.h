/**
 * @file test_eip_axis_internal.h
 * @brief Shared fakes and axis helpers for the EIP axis host tests.
 */

#pragma once

#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "CipMessageRouter.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipTransport.h"
#include "GantryEipLinearAxis.h"
#include "GantryEipRotaryAxis.h"
#include "GantryThetaOrigin.h"
#include "GantryUtils.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"
#include "host_test_clock.h"
#include "KinetixFaultCodes.h"

using eip::Bytes;

namespace eip_axis_test {

class FakeTcpClient : public eip::ITcpClient {
 public:
  void enqueueResponse(Bytes response) {
    responses_.push_back(std::move(response));
  }
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
  void setEcho(bool echo) { echo_ = echo; }
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
    return -1;
  }
  void close() override {}

 private:
  bool echo_ = false;
  Bytes last_sent_;
  Bytes echo_frame_;
};

inline
Bytes makeRegisterSessionReply(uint32_t handle) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = handle;
  h.status = 0;
  return eip::encodeEncapsulation(h, Bytes{0x01, 0x00, 0x00, 0x00});
}

inline
Bytes makeSendRRDataEncapReply(uint32_t session_handle, const Bytes& cip_reply) {
  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = session_handle;
  rh.status = 0;
  return eip::encodeEncapsulation(rh, eip::encodeSendRRDataPayload(cip_reply));
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
Bytes makeK5100Input154(int32_t position, bool fault, bool cmd_in_progress,
                        bool at_reference, uint16_t fault_code = 0,
                        uint16_t warning_code = 0, bool warning_present = false) {
  Bytes frame(eip::k5100::kInput154Size, 0);
  if (fault) frame[8] |= 0x02;
  if (warning_present) frame[9] |= 0x02;
  if (cmd_in_progress) frame[9] |= 0x10;
  if (at_reference) frame[9] |= 0x80;
  frame[20] = static_cast<uint8_t>(fault_code & 0xFF);
  frame[21] = static_cast<uint8_t>((fault_code >> 8) & 0xFF);
  frame[22] = static_cast<uint8_t>(warning_code & 0xFF);
  frame[23] = static_cast<uint8_t>((warning_code >> 8) & 0xFF);
  frame[24] = static_cast<uint8_t>(position & 0xFF);
  frame[25] = static_cast<uint8_t>((position >> 8) & 0xFF);
  frame[26] = static_cast<uint8_t>((position >> 16) & 0xFF);
  frame[27] = static_cast<uint8_t>((position >> 24) & 0xFF);
  return frame;
}

inline
Bytes makeHcs01Input102(int32_t position_puu, bool class1_error,
                        bool command_reached, uint32_t diagnostic = 0) {
  eip::hcs01::Hcs01PositioningActual actual;
  actual.status = eip::hcs01::Hcs01StatusWord::decode(0xC410);
  if (class1_error) {
    actual.status.class1_error = true;
    actual.status.ready = eip::hcs01::ReadyForOperation::kNotReady;
  }
  if (!command_reached) {
    actual.status.command_value_reached = false;
  }
  actual.position_feedback = position_puu;
  actual.velocity_feedback = 100;
  actual.diagnostic_message = diagnostic;

  Bytes frame;
  eip::ByteWriter w(frame);
  w.u16(actual.status.encode());
  w.i32(actual.position_feedback);
  w.i32(actual.velocity_feedback);
  w.u32(actual.diagnostic_message);
  return frame;
}

inline
void writeLeI32(Bytes& data, size_t offset, int32_t value) {
  const uint32_t u = static_cast<uint32_t>(value);
  data[offset] = static_cast<uint8_t>(u & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  data[offset + 2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  data[offset + 3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

inline
int32_t readLeI32(const Bytes& data, size_t offset) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(data[offset]) |
      (static_cast<uint32_t>(data[offset + 1]) << 8) |
      (static_cast<uint32_t>(data[offset + 2]) << 16) |
      (static_cast<uint32_t>(data[offset + 3]) << 24));
}

inline
uint16_t readLeU16(const Bytes& data, size_t offset) {
  return static_cast<uint16_t>(data[offset] |
                               (static_cast<uint16_t>(data[offset + 1]) << 8));
}
inline
void armAxisForMove(Gantry::GantryEipLinearAxis& axis,
                           eip::EipProcessImage& image, int32_t position_puu) {
  Bytes fb = makeK5100Input154(position_puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;  // active + ready + HomedStatus
  image.setFeedback(fb);
  // enable(): ServoLow + ServoSettle; HomedStatus skips Home34 unlock.
  for (int i = 0; i < 50; ++i) {
    axis.update();
  }
}

// kPreload=4 + kStart=5 @ 100 Hz. Extra ticks keep tests off the phase edge.
inline
void pumpAbsoluteToRun(Gantry::GantryEipLinearAxis& axis,
                              eip::EipProcessImage& image, int32_t position_puu) {
  Bytes fb = makeK5100Input154(position_puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
}

inline
int32_t cmdPositionPuu(const Bytes& cmd) {
  int32_t v = 0;
  std::memcpy(&v, &cmd[16], sizeof(v));  // Position reference, bytes 16..19 LE
  return v;
}

}  // namespace

