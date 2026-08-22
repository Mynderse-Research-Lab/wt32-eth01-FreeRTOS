// EtherNet/IP explicit messaging session over TCP 44818.
//
// Manages RegisterSession / SendRRData / UnRegisterSession. The CIP request
// bytes are built by the encoding layer; this class handles the TCP framing.

#ifndef ETHERNET_IP_EIP_SESSION_H
#define ETHERNET_IP_EIP_SESSION_H

#include <array>
#include <cstdint>

#include "EipByteBuffer.h"
#include "EipTransport.h"

namespace eip {

class EipSession {
 public:
  static constexpr uint16_t kDefaultPort = 44818;
  static constexpr size_t kMaxFrameSize = 512;

  explicit EipSession(ITcpClient& tcp);

  bool registerSession();
  bool sendExplicit(const Bytes& cip_request, Bytes& out_cip_response,
                    uint16_t timeout_ms = 5000);
  bool unregisterSession();

  uint32_t sessionHandle() const { return session_handle_; }
  bool isRegistered() const { return session_handle_ != 0; }

 private:
  bool transact(const Bytes& request, Bytes& out_response, uint16_t timeout_ms);

  ITcpClient& tcp_;
  uint32_t session_handle_ = 0;
  std::array<uint8_t, 8> sender_context_{};
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SESSION_H
