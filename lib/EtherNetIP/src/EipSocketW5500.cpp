// EtherNet/IP transport over W5500 hardware sockets.
//
// Thin delegation to w5500::socket*() functions. Zero W5500 register access.

#include "EipSocketW5500.h"

#include "EipConnectionManager.h"
#include "EipIoConnection.h"
#include "W5500Socket.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif

#include <cstring>

namespace eip {

// ==========================================================================
// EipSocketW5500Tcp
// ==========================================================================

EipSocketW5500Tcp::EipSocketW5500Tcp(w5500::W5500Hal& hal)
    : hal_(hal) {}

bool EipSocketW5500Tcp::connect(const char* host, uint16_t port) {
    close();

    uint32_t ip = parseIpv4Host(host);
    if (ip == 0) return false;

    int sock = w5500::socketOpen(hal_, w5500::SocketMode::kTcp);
    if (sock < 0) {
#ifdef ESP_PLATFORM
        ESP_LOGE("EipW5500", "TCP socketOpen failed (no free W5500 socket)");
#endif
        return false;
    }
    sock_ = sock;

#ifdef ESP_PLATFORM
    ESP_LOGI("EipW5500", "TCP open sock=%d connecting to %s:%u (timeout %u ms)",
             sock_, host, port, static_cast<unsigned>(defaultTimeoutMs_));
#endif

    bool ok = w5500::socketConnect(hal_, static_cast<uint8_t>(sock_), ip, port,
                                   defaultTimeoutMs_);
    if (!ok) {
        w5500::socketClose(hal_, static_cast<uint8_t>(sock_));
        sock_ = -1;
        return false;
    }

    connected_ = true;
    return true;
}

void EipSocketW5500Tcp::close() {
    if (sock_ < 0) return;
    w5500::socketClose(hal_, static_cast<uint8_t>(sock_));
    sock_ = -1;
    connected_ = false;
}

ssize_t EipSocketW5500Tcp::send(const uint8_t* data, size_t len) {
    if (sock_ < 0 || !connected_) return -1;
    return static_cast<ssize_t>(
        w5500::socketSend(hal_, static_cast<uint8_t>(sock_), data, len));
}

ssize_t EipSocketW5500Tcp::recv(uint8_t* buf, size_t max_len, uint32_t timeout_ms) {
    if (sock_ < 0) return -1;

    int n = w5500::socketRecv(hal_, static_cast<uint8_t>(sock_),
                               buf, max_len, timeout_ms);
    if (n <= 0) {
        // Check if socket disconnected
        if (!w5500::socketIsConnected(hal_, static_cast<uint8_t>(sock_))) {
            connected_ = false;
        }
        return n;
    }
    return static_cast<ssize_t>(n);
}

bool EipSocketW5500Tcp::isConnected() const {
    return (sock_ >= 0) && connected_;
}

// ==========================================================================
// EipSocketW5500Udp
// ==========================================================================

EipSocketW5500Udp::EipSocketW5500Udp(w5500::W5500Hal& hal)
    : hal_(hal) {}

bool EipSocketW5500Udp::bind(uint16_t port, uint32_t multicast_connection_id) {
    close();

    uint32_t mcast_ip = 0;
    if (multicast_connection_id != 0) {
        mcast_ip = multicastIpFromConnectionId(multicast_connection_id);
    }

    if (mcast_ip == 0) {
        // Point-to-point T->O: a single UDP socket on port 2222 handles both
        // O->T send and T->O receive. No IGMP/multicast ambiguity.
        int sock = w5500::socketOpen(hal_, w5500::SocketMode::kUdp, port, 0);
        if (sock < 0) {
#ifdef ESP_PLATFORM
            ESP_LOGE("EipW5500", "UDP P2P open failed port=%u", port);
#endif
            return false;
        }
        rx_sock_ = sock;
        tx_sock_ = sock;
        bound_ = true;
#ifdef ESP_PLATFORM
        ESP_LOGI("EipW5500", "UDP P2P sock=%d port=%u", sock, port);
#endif
        return true;
    }

    // Multicast T->O: split sockets so O->T sends cannot disturb IGMP membership.
    // RX socket listens on the multicast group; TX socket sends from port 2222.
    const int rx = w5500::socketOpen(hal_, w5500::SocketMode::kUdp, port, mcast_ip);
    if (rx < 0) {
#ifdef ESP_PLATFORM
        ESP_LOGE("EipW5500", "UDP RX open failed port=%u mcast_cid=0x%08lX",
                 port, static_cast<unsigned long>(multicast_connection_id));
#endif
        return false;
    }

    const int tx = w5500::socketOpen(hal_, w5500::SocketMode::kUdp,
                                     EipIoConnection::kDefaultUdpPort, 0);
    if (tx < 0) {
        w5500::socketClose(hal_, static_cast<uint8_t>(rx));
#ifdef ESP_PLATFORM
        ESP_LOGE("EipW5500", "UDP TX open failed (no free socket)");
#endif
        return false;
    }

    rx_sock_ = rx;
    tx_sock_ = tx;
    bound_ = true;
#ifdef ESP_PLATFORM
    ESP_LOGI("EipW5500",
             "UDP IO rx_sock=%d port=%u mcast_cid=0x%08lX tx_sock=%d port=%u",
             rx_sock_, port, static_cast<unsigned long>(multicast_connection_id),
             tx_sock_, EipIoConnection::kDefaultUdpPort);
#endif
    return true;
}

void EipSocketW5500Udp::close() {
    if (tx_sock_ >= 0 && tx_sock_ != rx_sock_) {
        w5500::socketClose(hal_, static_cast<uint8_t>(tx_sock_));
    }
    if (rx_sock_ >= 0) {
        w5500::socketClose(hal_, static_cast<uint8_t>(rx_sock_));
    }
    tx_sock_ = -1;
    rx_sock_ = -1;
    bound_ = false;
}

ssize_t EipSocketW5500Udp::sendTo(const uint8_t* data, size_t len,
                                   uint32_t dest_ip_host, uint16_t port) {
    if (tx_sock_ < 0 || !bound_) return -1;
    if (dest_ip_host == 0) return -1;

    const ssize_t sent = static_cast<ssize_t>(
        w5500::socketSendTo(hal_, static_cast<uint8_t>(tx_sock_),
                             data, len, dest_ip_host, port));
#ifdef ESP_PLATFORM
    if (sent != static_cast<ssize_t>(len)) {
        ESP_LOGW("EipW5500",
                 "UDP sendTo %u.%u.%u.%u:%u failed (%d of %u bytes) tx_sock=%d",
                 static_cast<unsigned>((dest_ip_host >> 24) & 0xFF),
                 static_cast<unsigned>((dest_ip_host >> 16) & 0xFF),
                 static_cast<unsigned>((dest_ip_host >> 8) & 0xFF),
                 static_cast<unsigned>(dest_ip_host & 0xFF),
                 port, static_cast<int>(sent), static_cast<unsigned>(len),
                 tx_sock_);
    }
#endif
    return sent;
}

ssize_t EipSocketW5500Udp::recvFrom(uint8_t* buf, size_t max_len,
                                     uint32_t timeout_ms) {
    if (rx_sock_ < 0) return -1;

    int n;
    if (timeout_ms == 0) {
        n = w5500::socketRecvFromNonBlocking(hal_, static_cast<uint8_t>(rx_sock_),
                                             buf, max_len);
    } else {
        n = w5500::socketRecvFrom(hal_, static_cast<uint8_t>(rx_sock_), buf,
                                  max_len, timeout_ms);
#ifdef ESP_PLATFORM
        if (n <= 0) {
            const int avail =
                w5500::socketRxAvailable(hal_, static_cast<uint8_t>(rx_sock_));
            if (avail > 0) {
                ESP_LOGW("EipW5500", "recvFrom timeout rx_sock=%d RX_RSR=%d",
                         rx_sock_, avail);
            }
        }
#endif
    }
    return static_cast<ssize_t>(n);
}

}  // namespace eip
