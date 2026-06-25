// Socket abstractions for the EtherNet/IP transport layer.
//
// Pure interfaces with no ESP-IDF dependency. Firmware provides lwIP
// implementations in EipSocketEspIdf.cpp; host tests use fakes.

#ifndef ETHERNET_IP_EIP_TRANSPORT_H
#define ETHERNET_IP_EIP_TRANSPORT_H

#include <cstddef>
#include <cstdint>

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
  virtual bool bind(uint16_t port) = 0;
  virtual ssize_t sendTo(const uint8_t* data, size_t len, const char* host,
                         uint16_t port) = 0;
  virtual ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t timeout_ms) = 0;
  virtual void close() = 0;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_TRANSPORT_H
