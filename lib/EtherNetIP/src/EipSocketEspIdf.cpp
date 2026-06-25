#include "EipSocketEspIdf.h"

#include <cstring>

#include "lwip/sockets.h"

namespace eip {

EipSocketTcpClient::EipSocketTcpClient() = default;

EipSocketTcpClient::~EipSocketTcpClient() { close(); }

bool EipSocketTcpClient::connect(const char* host, uint16_t port) {
  close();
  fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd_ < 0) return false;

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
    close();
    return false;
  }

  if (::connect(fd_, reinterpret_cast<struct sockaddr*>(&addr),
                sizeof(addr)) != 0) {
    close();
    return false;
  }
  return true;
}

ssize_t EipSocketTcpClient::send(const uint8_t* data, size_t len) {
  if (fd_ < 0) return -1;
  return ::send(fd_, data, len, 0);
}

ssize_t EipSocketTcpClient::recv(uint8_t* buf, size_t max_len,
                                 uint32_t timeout_ms) {
  if (fd_ < 0) return -1;

  struct timeval tv {};
  tv.tv_sec = static_cast<int>(timeout_ms / 1000);
  tv.tv_usec = static_cast<int>((timeout_ms % 1000) * 1000);
  (void)setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return ::recv(fd_, buf, max_len, 0);
}

void EipSocketTcpClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool EipSocketTcpClient::isConnected() const { return fd_ >= 0; }

EipSocketUdpEndpoint::EipSocketUdpEndpoint() = default;

EipSocketUdpEndpoint::~EipSocketUdpEndpoint() { close(); }

bool EipSocketUdpEndpoint::bind(uint16_t port) {
  close();
  fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd_ < 0) return false;

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) !=
      0) {
    close();
    return false;
  }
  return true;
}

ssize_t EipSocketUdpEndpoint::sendTo(const uint8_t* data, size_t len,
                                     const char* host, uint16_t port) {
  if (fd_ < 0) return -1;
  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) return -1;
  return ::sendto(fd_, data, len, 0, reinterpret_cast<struct sockaddr*>(&addr),
                  sizeof(addr));
}

ssize_t EipSocketUdpEndpoint::recvFrom(uint8_t* buf, size_t max_len,
                                       uint32_t timeout_ms) {
  if (fd_ < 0) return -1;
  struct timeval tv {};
  tv.tv_sec = static_cast<int>(timeout_ms / 1000);
  tv.tv_usec = static_cast<int>((timeout_ms % 1000) * 1000);
  (void)setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  return ::recvfrom(fd_, buf, max_len, 0, nullptr, nullptr);
}

void EipSocketUdpEndpoint::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace eip
