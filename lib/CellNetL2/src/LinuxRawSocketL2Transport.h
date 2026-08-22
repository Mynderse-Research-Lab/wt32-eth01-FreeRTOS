/**
 * @file LinuxRawSocketL2Transport.h
 * @brief Linux POSIX Raw Packet Socket (AF_PACKET / SOCK_RAW) transport driver.
 *
 * Enables running CellNetL2 directly on Linux workstations, Ubuntu IPCs,
 * and Raspberry Pi SBCs over any standard Ethernet NIC (e.g. eth0, enp3s0).
 */

#ifndef LINUX_RAW_SOCKET_L2_TRANSPORT_H
#define LINUX_RAW_SOCKET_L2_TRANSPORT_H

#ifndef ESP_PLATFORM

#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

namespace CellNet {

class LinuxRawSocketL2Transport : public IL2Transport {
 public:
  explicit LinuxRawSocketL2Transport(std::string iface_name = "eth0")
      : iface_name_(std::move(iface_name)) {}

  ~LinuxRawSocketL2Transport() override {
    running_ = false;
    if (sock_fd_ >= 0) {
      close(sock_fd_);
      sock_fd_ = -1;
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

  /**
   * @brief Open raw packet socket and bind to the network interface.
   * @return true if successfully opened and bound, false on error.
   */
  bool begin() {
    // Open raw Layer-2 packet socket filtered for EtherType 0x88B5
    sock_fd_ = socket(AF_PACKET, SOCK_RAW, htons(CELL_NET_L2_ETHERTYPE));
    if (sock_fd_ < 0) {
      std::cerr << "[LinuxL2] socket(AF_PACKET) failed: " << strerror(errno)
                << " (Did you run with CAP_NET_RAW or sudo?)" << std::endl;
      return false;
    }

    // Query network interface index
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[LinuxL2] ioctl(SIOCGIFINDEX) failed for interface '"
                << iface_name_ << "': " << strerror(errno) << std::endl;
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    if_index_ = ifr.ifr_ifindex;

    // Query hardware MAC address
    if (ioctl(sock_fd_, SIOCGIFHWADDR, &ifr) == 0) {
      std::memcpy(mac_, ifr.ifr_hwaddr.sa_data, 6);
    }

    // Bind socket strictly to this interface
    struct sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = if_index_;
    sll.sll_protocol = htons(CELL_NET_L2_ETHERTYPE);
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {
      std::cerr << "[LinuxL2] bind() to " << iface_name_
                << " failed: " << strerror(errno) << std::endl;
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }

    // Start background RX worker thread
    running_ = true;
    rx_thread_ = std::thread(&LinuxRawSocketL2Transport::rxWorker, this);
    return true;
  }

  bool sendFrame(const uint8_t* data, size_t length) override {
    if (sock_fd_ < 0 || data == nullptr || length == 0) {
      return false;
    }

    struct sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = if_index_;
    sll.sll_halen = 6;
    std::memcpy(sll.sll_addr, data, 6);  // Dest MAC from Ethernet header

    const ssize_t sent = sendto(sock_fd_, data, length, 0,
                                reinterpret_cast<struct sockaddr*>(&sll),
                                sizeof(sll));
    return (sent == static_cast<ssize_t>(length));
  }

  bool getMacAddress(uint8_t mac_out[6]) const override {
    if (mac_out == nullptr) {
      return false;
    }
    std::memcpy(mac_out, mac_, 6);
    return true;
  }

  bool isLinkUp() const override { return (sock_fd_ >= 0); }

  void setRxCallback(RxFrameCallback callback) override {
    rx_callback_ = std::move(callback);
  }

 private:
  void rxWorker() {
    uint8_t buf[2048];
    while (running_) {
      const ssize_t bytes =
          recvfrom(sock_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (bytes > 0 && rx_callback_) {
        rx_callback_(buf, static_cast<size_t>(bytes));
      }
    }
  }

  std::string iface_name_;
  int sock_fd_{-1};
  int if_index_{-1};
  uint8_t mac_[6]{};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  RxFrameCallback rx_callback_{nullptr};
};

}  // namespace CellNet

#endif  // !ESP_PLATFORM

#endif  // LINUX_RAW_SOCKET_L2_TRANSPORT_H
