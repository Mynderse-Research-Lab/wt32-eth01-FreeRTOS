// Socket abstractions for the EtherNet/IP transport layer.
//
// Pure interfaces with no ESP-IDF dependency. Firmware provides lwIP
// implementations in EipSocketEspIdf.cpp; host tests use fakes.

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
  virtual ssize_t sendTo(const uint8_t* data, size_t len, const char* host,
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
