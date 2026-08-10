// W5500 hardware TCP/UDP socket implementation.
//
// All SPI register I/O goes through W5500Hal. No direct W5500::readReg calls.
// Mutex-guarded for concurrent access from multiple FreeRTOS tasks.

#include "W5500Socket.h"

#include "W5500.h"  // register/status/command constants
#include "W5500BusMutex.h"
#include "W5500Poll.h"

#include <cstring>
#include <functional>
#include <mutex>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define W5500_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define W5500_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#define W5500_LOGE(...) ESP_LOGE(TAG, __VA_ARGS__)
#else
#include <chrono>
#include <thread>
#define W5500_LOGI(...) ((void)0)
#define W5500_LOGW(...) ((void)0)
#define W5500_LOGE(...) ((void)0)
#endif

namespace w5500 {

static const char* TAG = "W5500Sock";

namespace {

constexpr uint8_t kCommonBlock    = 0x00;
constexpr uint32_t kDefaultTimeoutMs = 5000;
constexpr uint32_t kCommandTimeoutMs  = 100;
constexpr size_t   kUdpHeaderSize     = 8;

static uint16_t g_next_tcp_source_port = 49152;

struct UdpMcastListen {
    uint32_t ip = 0;
    uint16_t port = 0;
};
static UdpMcastListen g_udp_mcast_listen[8] = {};

uint16_t nextTcpSourcePort() {
    const uint16_t port = g_next_tcp_source_port++;
    if (g_next_tcp_source_port < 49152) {
        g_next_tcp_source_port = 49152;
    }
    return port;
}

uint8_t readSocketReg(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    return hal.readReg(kBlockSocketReg(sock), addr);
}

void writeSocketReg(W5500Hal& hal, uint8_t sock, uint16_t addr, uint8_t val) {
    hal.writeReg(kBlockSocketReg(sock), addr, val);
}

uint16_t readSocketReg16(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    return hal.readReg16(kBlockSocketReg(sock), addr);
}

uint16_t readSocketSizeReg16Stable(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    uint16_t a = readSocketReg16(hal, sock, addr);
    for (int i = 0; i < 8; ++i) {
        uint16_t b = readSocketReg16(hal, sock, addr);
        if (a == b) return a;
        a = b;
    }
    return a;
}

void writeSocketReg16(W5500Hal& hal, uint8_t sock, uint16_t addr, uint16_t val) {
    hal.writeReg16(kBlockSocketReg(sock), addr, val);
}

bool waitCommandComplete(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
    return busyPollMs(timeoutMs, [&]() {
        return readSocketReg(hal, sock, Sn_CR) == 0;
    });
}

bool issueCommand(W5500Hal& hal, uint8_t sock, uint8_t cmd) {
    writeSocketReg(hal, sock, Sn_CR, cmd);
    return waitCommandComplete(hal, sock, kCommandTimeoutMs);
}

int findFreeSocket(W5500Hal& hal) {
    for (uint8_t s = 0; s < 8; ++s) {
        if (readSocketReg(hal, s, Sn_SR) == SOCK_CLOSED) {
            return static_cast<int>(s);
        }
    }
    return -1;
}

void writeSocketDIPR(W5500Hal& hal, uint8_t sock, uint32_t ip) {
    // Contiguous Sn_DIPR[4] - one burst instead of four writeReg calls.
    uint8_t data[4] = {
        static_cast<uint8_t>((ip >> 24) & 0xFF),
        static_cast<uint8_t>((ip >> 16) & 0xFF),
        static_cast<uint8_t>((ip >> 8) & 0xFF),
        static_cast<uint8_t>(ip & 0xFF)
    };
    hal.writeBuf(kBlockSocketReg(sock), Sn_DIPR, data, 4);
}

void writeSocketDest(W5500Hal& hal, uint8_t sock, uint32_t ip, uint16_t port) {
    // Sn_DIPR (0x0C..0x0F) + Sn_DPORT (0x10..0x11) are contiguous - one burst.
    uint8_t data[6] = {
        static_cast<uint8_t>((ip >> 24) & 0xFF),
        static_cast<uint8_t>((ip >> 16) & 0xFF),
        static_cast<uint8_t>((ip >> 8) & 0xFF),
        static_cast<uint8_t>(ip & 0xFF),
        static_cast<uint8_t>((port >> 8) & 0xFF),
        static_cast<uint8_t>(port & 0xFF),
    };
    hal.writeBuf(kBlockSocketReg(sock), Sn_DIPR, data, 6);
}

struct UdpDestCache {
    uint32_t ip = 0;
    uint16_t port = 0;
    bool valid = false;
};
static UdpDestCache g_udp_dest_cache[8] = {};

void cacheSocketDest(uint8_t sock, uint32_t ip, uint16_t port) {
    if (sock < 8) {
        g_udp_dest_cache[sock] = {ip, port, true};
    }
}

void clearSocketDestCache(uint8_t sock) {
    if (sock < 8) {
        g_udp_dest_cache[sock] = {};
    }
}

bool socketDestCached(uint8_t sock, uint32_t ip, uint16_t port) {
    if (sock >= 8) return false;
    const UdpDestCache& c = g_udp_dest_cache[sock];
    return c.valid && c.ip == ip && c.port == port;
}

void restoreUdpMulticastDest(W5500Hal& hal, uint8_t sock) {
    const UdpMcastListen& m = g_udp_mcast_listen[sock];
    if (m.ip == 0) return;
    // Skip SPI when the socket already points at the multicast listen dest.
    if (socketDestCached(sock, m.ip, m.port)) return;
    writeSocketDest(hal, sock, m.ip, m.port);
    cacheSocketDest(sock, m.ip, m.port);
}

bool waitSendComplete(W5500Hal& hal, uint8_t sock, uint32_t timeout_us) {
    // Tight spin: Class 1 UDP SENDOK is typically microseconds; taskYIELD at
    // 1000 Hz was costing ~1 ms per axis (~3 ms dual O->T).
    return busyPollUsTight(timeout_us, [&]() {
        std::lock_guard<std::mutex> lock(spiBusMutex());
        uint8_t ir = readSocketReg(hal, sock, Sn_IR);
        if (ir & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON)) {
            return true;
        }
        return false;
    });
}

bool waitCommandCompleteUnlocked(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
    const uint32_t timeout_us =
        (timeoutMs > (UINT32_MAX / 1000u)) ? UINT32_MAX : (timeoutMs * 1000u);
    // Cap tight CR wait - SEND clears CR quickly; do not burn seconds.
    const uint32_t capped =
        (timeout_us > kUdpSendOkTimeoutUs) ? kUdpSendOkTimeoutUs : timeout_us;
    return busyPollUsTight(capped, [&]() {
        std::lock_guard<std::mutex> lock(spiBusMutex());
        return readSocketReg(hal, sock, Sn_CR) == 0;
    });
}

void clearSendInterrupts(W5500Hal& hal, uint8_t sock) {
    uint8_t ir = readSocketReg(hal, sock, Sn_IR);
    if (ir != 0) {
        writeSocketReg(hal, sock, Sn_IR, ir);
    }
}

const char* socketStatusName(uint8_t sr) {
    switch (sr) {
        case SOCK_CLOSED:      return "CLOSED";
        case SOCK_INIT:        return "INIT";
        case SOCK_LISTEN:      return "LISTEN";
        case SOCK_SYNSENT:     return "SYNSENT";
        case SOCK_SYNRECV:     return "SYNRECV";
        case SOCK_ESTABLISHED: return "ESTABLISHED";
        case SOCK_CLOSE_WAIT:  return "CLOSE_WAIT";
        case SOCK_UDP:         return "UDP";
        default:               return "UNKNOWN";
    }
}

int readUdpDatagramUnlocked(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                            size_t maxLen) {
    uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
    if (avail < kUdpHeaderSize) {
        return 0;
    }

    uint16_t rxRd = readSocketReg16(hal, sock, Sn_RX_RD);

    uint8_t hdr[kUdpHeaderSize];
    hal.readBuf(kBlockSocketRxBuf(sock), rxRd, hdr, kUdpHeaderSize);

    const uint16_t payloadLen =
        static_cast<uint16_t>((static_cast<uint16_t>(hdr[6]) << 8) | hdr[7]);
    const uint16_t packetBytes =
        static_cast<uint16_t>(kUdpHeaderSize + payloadLen);
    if (payloadLen == 0 || packetBytes > avail) {
        W5500_LOGW(
            "RecvFrom: sock=%d bad UDP hdr len=%u avail=%u - discarding %u", sock,
            payloadLen, avail, avail);
        rxRd = static_cast<uint16_t>(rxRd + avail);
        writeSocketReg16(hal, sock, Sn_RX_RD, rxRd);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
        waitCommandComplete(hal, sock, kCommandTimeoutMs);
        return 0;
    }

    uint16_t toRead = (static_cast<size_t>(payloadLen) < maxLen)
                          ? payloadLen
                          : static_cast<uint16_t>(maxLen);

    hal.readBuf(kBlockSocketRxBuf(sock),
                static_cast<uint16_t>(rxRd + kUdpHeaderSize), buf, toRead);

    rxRd = static_cast<uint16_t>(rxRd + packetBytes);
    writeSocketReg16(hal, sock, Sn_RX_RD, rxRd);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
    waitCommandComplete(hal, sock, kCommandTimeoutMs);
    return static_cast<int>(toRead);
}

}  // namespace

uint16_t nextEphemeralPort() {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    return nextTcpSourcePort();
}

// --- Public API --------------------------------------------------------------

int socketOpen(W5500Hal& hal, SocketMode mode, uint16_t localPort,
               uint32_t udpMulticastListenIp) {
    std::lock_guard<std::mutex> lock(spiBusMutex());

    int sock = findFreeSocket(hal);
    if (sock < 0) return -1;

    uint8_t sockU8 = static_cast<uint8_t>(sock);

    if (mode == SocketMode::kTcp && localPort == 0) {
        localPort = nextTcpSourcePort();
    }
    if (mode == SocketMode::kUdp && localPort == 0) {
        localPort = nextTcpSourcePort();
    }

    if (localPort != 0) {
        writeSocketReg16(hal, sockU8, Sn_PORT, localPort);
    }

    uint8_t mr = static_cast<uint8_t>(mode);
    if (mode == SocketMode::kUdp && udpMulticastListenIp != 0) {
        mr |= Sn_MR_MULTI;
        writeSocketDest(hal, sockU8, udpMulticastListenIp, localPort);
        cacheSocketDest(sockU8, udpMulticastListenIp, localPort);
        g_udp_mcast_listen[sockU8] = {udpMulticastListenIp, localPort};
    } else {
        g_udp_mcast_listen[sockU8] = {};
        clearSocketDestCache(sockU8);
    }

    writeSocketReg(hal, sockU8, Sn_MR, mr);

    if (!issueCommand(hal, sockU8, Sn_CR_OPEN)) {
        return -1;
    }

    if (mode == SocketMode::kUdp) {
        if (!busyPollMs(500, [&]() {
            const uint8_t sr = readSocketReg(hal, sockU8, Sn_SR);
            return (sr == SOCK_UDP || sr == SOCK_CLOSED);
        })) {
            W5500_LOGW( "UDP open: sock=%d timed out waiting for SOCK_UDP", sock);
            return -1;
        }
        const uint8_t status = readSocketReg(hal, sockU8, Sn_SR);
        if (status != SOCK_UDP) {
            W5500_LOGW( "UDP open: sock=%d final Sr=0x%02X (%s)",
                     sock, status, socketStatusName(status));
            return -1;
        }
#ifdef ESP_PLATFORM
        if (udpMulticastListenIp != 0) {
            W5500_LOGI("UDP multicast sock=%d port=%u group=%u.%u.%u.%u",
                     sock, localPort,
                     (udpMulticastListenIp >> 24) & 0xFF,
                     (udpMulticastListenIp >> 16) & 0xFF,
                     (udpMulticastListenIp >> 8) & 0xFF,
                     udpMulticastListenIp & 0xFF);
        } else {
            W5500_LOGI("UDP sock=%d bound port=%u", sock, localPort);
        }
#endif
        return sock;
    }

    uint8_t status = readSocketReg(hal, sockU8, Sn_SR);
    if (status == SOCK_CLOSED) {
        return -1;
    }

    return sock;
}

void socketClose(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    g_udp_mcast_listen[sock] = {};
    clearSocketDestCache(sock);
    issueCommand(hal, sock, Sn_CR_CLOSE);
    writeSocketReg(hal, sock, Sn_MR, Sn_MR_CLOSE);
    issueCommand(hal, sock, Sn_CR_CLOSE);
    busyPollMs(200, [&]() {
        return readSocketReg(hal, sock, Sn_SR) == SOCK_CLOSED;
    });
}

bool socketConnect(W5500Hal& hal, uint8_t sock, uint32_t ip, uint16_t port,
                   uint32_t timeoutMs) {
    uint16_t sourcePort = 0;
    {
        // Hold SPI only for setup + CONNECT + Sn_CR clear (ms). Do NOT hold
        // across the ESTABLISHED poll - that starved Class 1 for up to 5s.
        std::lock_guard<std::mutex> lock(spiBusMutex());

        sourcePort = readSocketReg16(hal, sock, Sn_PORT);
        if (sourcePort == 0) {
            sourcePort = nextTcpSourcePort();
            writeSocketReg16(hal, sock, Sn_PORT, sourcePort);
        }

        const uint8_t sr0 = readSocketReg(hal, sock, Sn_SR);
        if (sr0 != SOCK_INIT) {
            W5500_LOGW( "Connect: sock=%d not INIT (Sr=0x%02X %s) - abort",
                     sock, sr0, socketStatusName(sr0));
            return false;
        }

        // Sn_CR must be idle before CONNECT or the command is dropped.
        if (!waitCommandComplete(hal, sock, kCommandTimeoutMs)) {
            W5500_LOGE("Connect: Sn_CR busy before CONNECT sock=%d", sock);
            return false;
        }

        writeSocketDest(hal, sock, ip, port);
        cacheSocketDest(sock, ip, port);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_CONNECT);
        if (!waitCommandComplete(hal, sock, kCommandTimeoutMs)) {
            W5500_LOGE("Connect: CONNECT cmd timed out on socket %d src_port=%u",
                     sock, sourcePort);
            return false;
        }
    }

    // Poll Sn_SR without holding SPI (Class 1 / other sockets must proceed).
    // INIT during ARP is normal on W5500; SYNSENT once ARP succeeds; CLOSED
    // on ARP/TCP failure.
    uint32_t elapsed = 0;
    constexpr uint32_t kDiagIntervalMs = 1000;
    uint32_t nextDiag = kDiagIntervalMs;
    uint8_t finalSr = SOCK_CLOSED;

    if (!busyPollMs(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(spiBusMutex());
        uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        finalSr = sr;

#ifdef ESP_PLATFORM
        if (elapsed >= nextDiag) {
            W5500_LOGI("Connect diag sock=%d src_port=%u sr=0x%02X (%s) elapsed=%lu ms",
                     sock, sourcePort, sr, socketStatusName(sr),
                     static_cast<unsigned long>(elapsed));
            nextDiag += kDiagIntervalMs;
        }
#endif
        ++elapsed;

        return (sr == SOCK_ESTABLISHED || sr == SOCK_CLOSED ||
                sr == SOCK_CLOSE_WAIT);
    })) {
        W5500_LOGW( "Connect: timeout sock=%d src_port=%u final Sr=0x%02X (%s) elapsed=%lu ms",
                 sock, sourcePort, finalSr, socketStatusName(finalSr),
                 static_cast<unsigned long>(elapsed));
        return false;
    }

    if (finalSr != SOCK_ESTABLISHED) {
        uint8_t ir = 0;
        {
            std::lock_guard<std::mutex> lock(spiBusMutex());
            ir = readSocketReg(hal, sock, Sn_IR);
            clearSendInterrupts(hal, sock);
        }
        W5500_LOGW(
                 "Connect: failed sock=%d src_port=%u Sr=0x%02X (%s) IR=0x%02X "
                 "(CLOSED after INIT usually = ARP/TCP fail - check drive link/"
                 "E602 clear, daisy-chain)",
                 sock, sourcePort, finalSr, socketStatusName(finalSr), ir);
        return false;
    }

    W5500_LOGI("Connect: sock=%d src_port=%u final Sr=0x%02X (%s)",
             sock, sourcePort, finalSr, socketStatusName(finalSr));
    return true;
}

bool socketListen(W5500Hal& hal, uint8_t sock, uint16_t port) {
    std::lock_guard<std::mutex> lock(spiBusMutex());

    writeSocketReg16(hal, sock, Sn_PORT, port);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_LISTEN);
    return waitCommandComplete(hal, sock, kCommandTimeoutMs);
}

int socketSend(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    {
        std::lock_guard<std::mutex> lock(spiBusMutex());

        uint16_t freeSize = readSocketSizeReg16Stable(hal, sock, Sn_TX_FSR);
        if (freeSize < static_cast<uint16_t>(len)) {
            W5500_LOGW( "Send: sock=%d need %u bytes, TX free %u",
                     sock, static_cast<unsigned>(len), freeSize);
            return -1;
        }

        uint16_t txWr = readSocketReg16(hal, sock, Sn_TX_WR);
        hal.writeBuf(kBlockSocketTxBuf(sock), txWr, data, static_cast<uint16_t>(len));

        txWr += static_cast<uint16_t>(len);
        writeSocketReg16(hal, sock, Sn_TX_WR, txWr);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);
    }

    if (!waitCommandCompleteUnlocked(hal, sock, kCommandTimeoutMs)) {
        W5500_LOGW( "Send: sock=%d SEND cmd timeout", sock);
        return -1;
    }

    if (!waitSendComplete(hal, sock, kTcpSendOkTimeoutUs)) {
        W5500_LOGW( "Send: sock=%d SENDOK timeout", sock);
        return -1;
    }

    std::lock_guard<std::mutex> lock(spiBusMutex());
    uint8_t ir = readSocketReg(hal, sock, Sn_IR);
    if (ir & Sn_IR_SENDOK) {
        clearSendInterrupts(hal, sock);
        return static_cast<int>(len);
    }
    if (ir & Sn_IR_TIMEOUT) {
        W5500_LOGW( "Send: sock=%d TX timeout (ARP/link)", sock);
    } else if (ir & Sn_IR_DISCON) {
        W5500_LOGW( "Send: sock=%d disconnected during send", sock);
    }
    clearSendInterrupts(hal, sock);
    return -1;
}

int socketRecv(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
               uint32_t timeoutMs) {
    // Poll without holding the SPI mutex for the whole timeout (other sockets
    // must keep exchanging Class 1 frames).
    bool dataReady = busyPollMs(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(spiBusMutex());
        uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
        if (avail > 0) return true;
        uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        return (sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT);
    });
    (void)dataReady;

    std::lock_guard<std::mutex> lock(spiBusMutex());

    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    if (sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT) return -1;

    uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
    if (avail == 0) return 0;  // timeout, no data

    uint16_t toRead = (static_cast<size_t>(avail) < maxLen) ? avail : static_cast<uint16_t>(maxLen);
    uint16_t rxRd = readSocketReg16(hal, sock, Sn_RX_RD);

    hal.readBuf(kBlockSocketRxBuf(sock), rxRd, buf, toRead);

    rxRd += toRead;
    writeSocketReg16(hal, sock, Sn_RX_RD, rxRd);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
    waitCommandComplete(hal, sock, kCommandTimeoutMs);

    return static_cast<int>(toRead);
}

int socketSendTo(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len,
                 uint32_t destIp, uint16_t destPort) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    // Hold SPI for the whole UDP TX + SENDOK. Avoids per-poll mutex churn and
    // keeps Class 1 O->T on a tight path (SPI3 is deferred during exchange).
    std::lock_guard<std::mutex> lock(spiBusMutex());

    // Class 1 destinations are fixed per axis - skip DIPR/DPORT when cached.
    if (!socketDestCached(sock, destIp, destPort)) {
        writeSocketDest(hal, sock, destIp, destPort);
        cacheSocketDest(sock, destIp, destPort);
    }

    // No Sn_TX_FSR / Sn_SR on the fast path: frames are tiny vs TX buffer and
    // every prior send waited for SENDOK. Check Sn_SR only on failure.

    uint16_t txWr = readSocketReg16(hal, sock, Sn_TX_WR);
    hal.writeBuf(kBlockSocketTxBuf(sock), txWr, data, static_cast<uint16_t>(len));

    txWr += static_cast<uint16_t>(len);
    writeSocketReg16(hal, sock, Sn_TX_WR, txWr);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);

    // Sn_CR (0x01) and Sn_IR (0x02) are adjacent - one 2-byte poll covers both.
    const int64_t deadline = pollNowUs() + static_cast<int64_t>(kUdpSendOkTimeoutUs);
    uint8_t cr_ir[2] = {0xFF, 0};
    bool send_done = false;
    while (pollNowUs() < deadline) {
        hal.readBuf(kBlockSocketReg(sock), Sn_CR, cr_ir, 2);
        if (cr_ir[0] == 0 &&
            (cr_ir[1] & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON))) {
            send_done = true;
            break;
        }
    }
    if (!send_done) {
        // Distinguish cmd stuck vs SENDOK miss for diagnostics.
        if (cr_ir[0] != 0) {
            W5500_LOGW( "SendTo: sock=%d SEND cmd timeout", sock);
        } else {
            W5500_LOGW( "SendTo: sock=%d SENDOK timeout", sock);
        }
        const uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        if (sr != SOCK_UDP) {
            W5500_LOGW( "SendTo: sock=%d not UDP (Sr=0x%02X)", sock, sr);
        }
        return -1;
    }

    if (cr_ir[1] & Sn_IR_SENDOK) {
        clearSendInterrupts(hal, sock);
        restoreUdpMulticastDest(hal, sock);
        return static_cast<int>(len);
    }
    clearSendInterrupts(hal, sock);
    W5500_LOGW( "SendTo: sock=%d no SENDOK (IR=0x%02X)", sock, cr_ir[1]);
    return -1;
}

int socketRecvFrom(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
                   uint32_t timeoutMs) {
    // Do not hold spiBusMutex() across the wait - Class 1 O->T sendTo on the
    // same (or another) socket must proceed while we wait for T->O.
    const uint32_t timeout_us =
        (timeoutMs > (UINT32_MAX / 1000u)) ? UINT32_MAX : (timeoutMs * 1000u);
    bool dataReady = false;
    if (timeout_us <= kBusyPollYieldAboveUs) {
        dataReady = busyPollUsTight(timeout_us, [&]() {
            std::lock_guard<std::mutex> lock(spiBusMutex());
            return readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR) >=
                   kUdpHeaderSize;
        });
    } else {
        dataReady = busyPollUs(timeout_us, [&]() {
            std::lock_guard<std::mutex> lock(spiBusMutex());
            return readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR) >=
                   kUdpHeaderSize;
        });
    }

    std::lock_guard<std::mutex> lock(spiBusMutex());
    if (!dataReady) {
        return 0;
    }
    return readUdpDatagramUnlocked(hal, sock, buf, maxLen);
}

int socketRecvFromNonBlocking(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                              size_t maxLen) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    return readUdpDatagramUnlocked(hal, sock, buf, maxLen);
}

bool socketBind(W5500Hal& hal, uint8_t sock, uint16_t port) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    writeSocketReg16(hal, sock, Sn_PORT, port);
    return true;
}

bool socketIsConnected(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr == SOCK_ESTABLISHED);
}

bool socketIsOpen(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr != SOCK_CLOSED);
}

int socketRxAvailable(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    return static_cast<int>(readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR));
}

uint8_t socketStatus(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    return readSocketReg(hal, sock, Sn_SR);
}

}  // namespace w5500
