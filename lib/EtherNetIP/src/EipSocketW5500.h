// EtherNet/IP transport over W5500 hardware TCP/UDP sockets.
//
// Thin adapter implementing ITcpClient and IUdpEndpoint. Contains ZERO
// W5500 register reads or writes. All I/O delegates to w5500::socket*().
//
// Uses the W5500 hardware TCP/IP offload engine — no lwIP involvement.
// The W5500 SPI bus is mutex-guarded internally by W5500Socket.cpp.

#ifndef ETHERNET_IP_EIP_SOCKET_W5500_H
#define ETHERNET_IP_EIP_SOCKET_W5500_H

#include "EipTransport.h"

namespace w5500 { class W5500Hal; }

namespace eip {

class EipSocketW5500Tcp : public ITcpClient {
public:
    explicit EipSocketW5500Tcp(w5500::W5500Hal& hal);

    bool connect(const char* host, uint16_t port) override;
    void close() override;
    ssize_t send(const uint8_t* data, size_t len) override;
    ssize_t recv(uint8_t* buf, size_t max_len, uint32_t timeout_ms) override;
    bool isConnected() const override;

private:
    w5500::W5500Hal& hal_;
    int sock_ = -1;       // W5500 socket number (-1 = not open)
    bool connected_ = false;
    uint32_t defaultTimeoutMs_ = 5000;
};

class EipSocketW5500Udp : public IUdpEndpoint {
public:
    explicit EipSocketW5500Udp(w5500::W5500Hal& hal);

    bool bind(uint16_t port, uint32_t multicast_connection_id = 0) override;
    void close() override;
    ssize_t sendTo(const uint8_t* data, size_t len, uint32_t dest_ip_host,
                   uint16_t port) override;
    ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t timeout_ms) override;
    size_t recvBatch(uint8_t* buf, size_t buf_len, UdpDatagramView* views,
                     size_t max_views) override;

private:
    static constexpr size_t kMaxP2pTx = 3;
    // Per-call batch ceiling; the caller loops until the RX FIFO is empty.
    static constexpr size_t kMaxRecvBatch = 16;

    int p2pTxSockFor(uint32_t dest_ip_host);
    void closeP2pTx();

    w5500::W5500Hal& hal_;
    int rx_sock_ = -1;   // Class 1 T->O listener (port 2222, optional MULTI)
    int tx_sock_ = -1;   // Multicast O->T only; P2P uses p2p_tx_sock_[]
    int p2p_tx_sock_[kMaxP2pTx];
    uint32_t p2p_tx_dest_[kMaxP2pTx];
    size_t p2p_tx_count_ = 0;
    bool p2p_split_tx_ = false;  // P2P: RX on 2222, one TX socket per dest IP
    bool bound_ = false;
    uint32_t defaultTimeoutMs_ = 20000;  // cyclic I/O timeout
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SOCKET_W5500_H
