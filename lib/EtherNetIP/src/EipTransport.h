// Socket abstractions for the EtherNet/IP transport layer.
//
// Pure interfaces with no ESP-IDF dependency. Firmware provides the W5500
// implementation in EipSocketW5500.cpp; host tests use fakes.

#ifndef ETHERNET_IP_EIP_TRANSPORT_H
#define ETHERNET_IP_EIP_TRANSPORT_H

#include <cstddef>
#include <cstdint>

// ssize_t is POSIX; provide a fallback for environments that lack it.
#ifndef _SSIZE_T_DEFINED
#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
#include <sys/types.h>
#endif
#endif

namespace eip {

/// Parse dotted-decimal IPv4 to host-order uint32 ((a<<24)|(b<<16)|(c<<8)|d).
/// Returns 0 on failure (also rejects 0.0.0.0).
inline uint32_t parseIpv4Host(const char* host) {
  if (host == nullptr) {
    return 0;
  }
  unsigned int a = 0, b = 0, c = 0, d = 0;
  // Manual parse — Class 1 must never call sscanf on the hot path; this helper
  // is for FO / config only.
  const char* p = host;
  auto read_octet = [&p](unsigned int& out) -> bool {
    if (*p < '0' || *p > '9') {
      return false;
    }
    unsigned int v = 0;
    while (*p >= '0' && *p <= '9') {
      v = v * 10u + static_cast<unsigned int>(*p - '0');
      if (v > 255u) {
        return false;
      }
      ++p;
    }
    out = v;
    return true;
  };
  if (!read_octet(a) || *p++ != '.') {
    return 0;
  }
  if (!read_octet(b) || *p++ != '.') {
    return 0;
  }
  if (!read_octet(c) || *p++ != '.') {
    return 0;
  }
  if (!read_octet(d) || *p != '\0') {
    return 0;
  }
  return (static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(b) << 16) |
         (static_cast<uint32_t>(c) << 8) | static_cast<uint32_t>(d);
}

class ITcpClient {
 public:
  virtual ~ITcpClient() = default;
  virtual bool connect(const char* host, uint16_t port) = 0;
  virtual ssize_t send(const uint8_t* data, size_t len) = 0;
  virtual ssize_t recv(uint8_t* buf, size_t max_len, uint32_t timeout_ms) = 0;
  virtual void close() = 0;
  virtual bool isConnected() const = 0;
};

class IUdpEndpoint {
 public:
  virtual ~IUdpEndpoint() = default;
  // multicast_connection_id: when non-zero, join the Class 1 T->O multicast
  // group derived from this connection ID (W5500 Sn_MR_MULTI).
  virtual bool bind(uint16_t port, uint32_t multicast_connection_id = 0) = 0;
  /// Class 1 O->T: dest_ip_host is pre-parsed host-order IPv4 (never a string).
  virtual ssize_t sendTo(const uint8_t* data, size_t len, uint32_t dest_ip_host,
                         uint16_t port) = 0;
  virtual ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t timeout_ms) = 0;
  virtual void close() = 0;
};

// Shared Ethernet link state (firmware adapts MqttBridge::EthernetLink).
class ILinkStatus {
 public:
  virtual ~ILinkStatus() = default;
  virtual bool isUp() const = 0;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_TRANSPORT_H
