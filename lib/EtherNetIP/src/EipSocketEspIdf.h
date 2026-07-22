// lwIP socket implementations of ITcpClient / IUdpEndpoint (ESP-IDF only).

#ifndef ETHERNET_IP_EIP_SOCKET_ESPIDF_H
#define ETHERNET_IP_EIP_SOCKET_ESPIDF_H

#include "EipTransport.h"

namespace eip {

class EipSocketTcpClient : public ITcpClient {
 public:
  EipSocketTcpClient();
  ~EipSocketTcpClient() override;

  bool connect(const char* host, uint16_t port) override;
  ssize_t send(const uint8_t* data, size_t len) override;
  ssize_t recv(uint8_t* buf, size_t max_len, uint32_t timeout_ms) override;
  void close() override;
  bool isConnected() const override;

 private:
  int fd_ = -1;
};

class EipSocketUdpEndpoint : public IUdpEndpoint {
 public:
  EipSocketUdpEndpoint();
  ~EipSocketUdpEndpoint() override;

  bool bind(uint16_t port, uint32_t multicast_connection_id = 0) override;
  ssize_t sendTo(const uint8_t* data, size_t len, const char* host,
                 uint16_t port) override;
  ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t timeout_ms) override;
  void close() override;

 private:
  int fd_ = -1;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SOCKET_ESPIDF_H
