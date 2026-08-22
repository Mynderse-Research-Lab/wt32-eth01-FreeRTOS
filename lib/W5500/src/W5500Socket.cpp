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

// Deferred Class 1 SENDOK: SEND was issued and its completion is owed at the
// head of the next send on that socket (a full RPI later).
static bool g_udp_send_pending[8] = {};

// Sn_TX_WR is host-owned; the chip never moves it. Track it in software so the
// Class 1 send path does not pay a read per frame.
struct TxWrCache {
    uint16_t wr = 0;
    bool valid = false;
};
static TxWrCache g_tx_wr_cache[8] = {};

void cacheSocketDest(uint8_t sock, uint32_t ip, uint16_t port) {
    if (sock < 8) {
        g_udp_dest_cache[sock] = {ip, port, true};
    }
}

void clearSocketDestCache(uint8_t sock) {
    if (sock < 8) {
        g_udp_dest_cache[sock] = {};
        g_udp_send_pending[sock] = false;
        g_tx_wr_cache[sock] = {};
    }
}

void invalidateSocketTxWr(uint8_t sock) {
    if (sock < 8) {
        g_tx_wr_cache[sock] = {};
    }
}

uint16_t readSocketTxWr(W5500Hal& hal, uint8_t sock) {
    if (sock < 8 && g_tx_wr_cache[sock].valid) {
        return g_tx_wr_cache[sock].wr;
    }
    const uint16_t wr = readSocketReg16(hal, sock, Sn_TX_WR);
    if (sock < 8) {
        g_tx_wr_cache[sock] = {wr, true};
    }
    return wr;
}

void noteSocketTxWr(uint8_t sock, uint16_t wr) {
    if (sock < 8) {
        g_tx_wr_cache[sock] = {wr, true};
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

// Sn_IR is write-1-to-clear and every caller already holds the polled value,
// so no read-back is needed on the Class 1 hot path.
void clearSendInterrupts(W5500Hal& hal, uint8_t sock, uint8_t ir) {
    if (ir != 0) {
        writeSocketReg(hal, sock, Sn_IR, ir);
    }
}

// Confirm a SEND that was left unconfirmed by the previous deferred call on
// this socket. Returns false when that send reported TIMEOUT/DISCON (or never
// completed) so the caller can escalate one cycle late.
bool confirmDeferredSend(W5500Hal& hal, uint8_t sock) {
    if (sock >= 8 || !g_udp_send_pending[sock]) return true;
    g_udp_send_pending[sock] = false;

    uint8_t cr_ir[2] = {0xFF, 0};
    hal.readBuf(kBlockSocketReg(sock), Sn_CR, cr_ir, 2);
    bool send_done =
        (cr_ir[0] == 0 &&
         (cr_ir[1] & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON)) != 0);
    if (!send_done) {
        // A full RPI has passed, so this is rare; fall back to the same
        // bounded spin the blocking path uses.
        const int64_t deadline =
            pollNowUs() + static_cast<int64_t>(kUdpSendOkTimeoutUs);
        while (pollNowUs() < deadline) {
            hal.readBuf(kBlockSocketReg(sock), Sn_CR, cr_ir, 2);
            if (cr_ir[0] == 0 &&
                (cr_ir[1] & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON)) != 0) {
                send_done = true;
                break;
            }
        }
    }
    if (!send_done) {
        W5500_LOGW( "SendTo: sock=%d deferred SENDOK timeout", sock);
        return false;
    }
    clearSendInterrupts(hal, sock, cr_ir[1]);
    if (cr_ir[1] & Sn_IR_SENDOK) return true;
    W5500_LOGW( "SendTo: sock=%d deferred send failed (IR=0x%02X)", sock,
             cr_ir[1]);
    return false;
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

void readRxRsrAndRd(W5500Hal& hal, uint8_t sock, uint16_t& rsr, uint16_t& rd) {
    // Sn_RX_RSR (0x26) and Sn_RX_RD (0x28) are adjacent — one 4-byte burst.
    uint8_t b[4] = {};
    hal.readBuf(kBlockSocketReg(sock), Sn_RX_RSR, b, 4);
    rsr = static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
    rd = static_cast<uint16_t>((static_cast<uint16_t>(b[2]) << 8) | b[3]);
}

// Settled Sn_RX_RSR / Sn_RX_RD pair. The chip advances RSR asynchronously, so
// re-read until two bursts agree. False when no full UDP header is queued.
bool readSettledRxPointers(W5500Hal& hal, uint8_t sock, uint16_t& avail,
                           uint16_t& rxRd) {
    readRxRsrAndRd(hal, sock, avail, rxRd);
    if (avail < kUdpHeaderSize) return false;
    for (int i = 0; i < 8; ++i) {
        uint16_t avail2 = 0;
        uint16_t rd2 = 0;
        readRxRsrAndRd(hal, sock, avail2, rd2);
        if (avail == avail2) {
            rxRd = rd2;
            break;
        }
        avail = avail2;
        rxRd = rd2;
    }
    return avail >= kUdpHeaderSize;
}

bool waitUdpRecvCommand(W5500Hal& hal, uint8_t sock) {
    return busyPollUsTight(kUdpRecvCmdTimeoutUs, [&]() {
        return readSocketReg(hal, sock, Sn_CR) == 0;
    });
}

void completeUdpRecv(W5500Hal& hal, uint8_t sock, uint16_t new_rd) {
    writeSocketReg16(hal, sock, Sn_RX_RD, new_rd);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
    (void)waitUdpRecvCommand(hal, sock);
}

int readUdpDatagramUnlocked(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                            size_t maxLen) {
    // Class 1 drain peeks up to this many RX bytes in one SPI burst (header +
    // payload, or two queued Class 1 frames). Larger datagrams fall back.
    constexpr uint16_t kUdpFastBurstMax = 8 + 256;

    uint16_t avail = 0;
    uint16_t rxRd = 0;
    if (!readSettledRxPointers(hal, sock, avail, rxRd)) {
        return 0;
    }

    const uint16_t peek =
        (avail < kUdpFastBurstMax) ? avail : kUdpFastBurstMax;
    uint8_t scratch[kUdpFastBurstMax];
    hal.readBuf(kBlockSocketRxBuf(sock), rxRd, scratch, peek);

    const uint16_t payloadLen =
        static_cast<uint16_t>((static_cast<uint16_t>(scratch[6]) << 8) |
                              scratch[7]);
    const uint16_t packetBytes =
        static_cast<uint16_t>(kUdpHeaderSize + payloadLen);
    if (payloadLen == 0 || packetBytes > avail) {
        W5500_LOGW(
            "RecvFrom: sock=%d bad UDP hdr len=%u avail=%u - discarding %u", sock,
            payloadLen, avail, avail);
        completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + avail));
        return 0;
    }

    const uint16_t toRead = (static_cast<size_t>(payloadLen) < maxLen)
                                ? payloadLen
                                : static_cast<uint16_t>(maxLen);
    if (packetBytes <= peek) {
        if (toRead > 0 && buf != nullptr) {
            std::memcpy(buf, scratch + kUdpHeaderSize, toRead);
        }
    } else {
        hal.readBuf(kBlockSocketRxBuf(sock),
                    static_cast<uint16_t>(rxRd + kUdpHeaderSize), buf, toRead);
    }

    completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + packetBytes));
    return static_cast<int>(toRead);
}

size_t readUdpBatchUnlocked(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                            size_t bufLen, UdpDatagram* out, size_t maxOut) {
    if (buf == nullptr || out == nullptr || maxOut == 0 ||
        bufLen <= kUdpHeaderSize) {
        return 0;
    }
    if (bufLen > 0xFFFFu) bufLen = 0xFFFFu;

    uint16_t avail = 0;
    uint16_t rxRd = 0;
    if (!readSettledRxPointers(hal, sock, avail, rxRd)) {
        return 0;
    }

    const uint16_t burst =
        (static_cast<size_t>(avail) < bufLen) ? avail
                                              : static_cast<uint16_t>(bufLen);
    hal.readBuf(kBlockSocketRxBuf(sock), rxRd, buf, burst);

    size_t n = 0;
    uint16_t off = 0;
    bool malformed = false;
    while (n < maxOut && static_cast<size_t>(off) + kUdpHeaderSize <= burst) {
        const uint16_t payloadLen = static_cast<uint16_t>(
            (static_cast<uint16_t>(buf[off + 6]) << 8) | buf[off + 7]);
        const uint16_t packetBytes =
            static_cast<uint16_t>(kUdpHeaderSize + payloadLen);
        if (payloadLen == 0) {
            malformed = true;
            break;
        }
        if (static_cast<uint32_t>(off) + packetBytes > burst) {
            // Only a real overclaim is malformed; a burst clipped by bufLen
            // just leaves the rest of the datagram queued.
            malformed = (burst == avail);
            break;
        }
        out[n].data = buf + off + kUdpHeaderSize;
        out[n].len = payloadLen;
        ++n;
        off = static_cast<uint16_t>(off + packetBytes);
    }

    if (n > 0) {
        completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + off));
        return n;
    }

    if (!malformed && burst > kUdpHeaderSize) {
        // Head datagram is larger than the batch scratch: hand back what fits
        // and consume the whole packet so the FIFO cannot stall.
        const uint16_t payloadLen = static_cast<uint16_t>(
            (static_cast<uint16_t>(buf[6]) << 8) | buf[7]);
        const uint16_t packetBytes =
            static_cast<uint16_t>(kUdpHeaderSize + payloadLen);
        if (payloadLen > 0 && packetBytes <= avail) {
            out[0].data = buf + kUdpHeaderSize;
            out[0].len = static_cast<uint16_t>(burst - kUdpHeaderSize);
            completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + packetBytes));
            return 1;
        }
    }

    W5500_LOGW( "RecvBatch: sock=%d bad UDP hdr avail=%u - discarding %u", sock,
             avail, avail);
    completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + avail));
    return 0;
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
    // OPEN resets the chip-side FIFO pointers; drop any inherited software
    // state so recover() cannot resume on a stale Sn_TX_WR or SEND.
    invalidateSocketTxWr(sockU8);
    g_udp_send_pending[sockU8] = false;

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
        (void)readSocketTxWr(hal, sockU8);
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

    (void)readSocketTxWr(hal, sockU8);
    return sock;
}

void socketResetSoftwareState() {
    for (uint8_t s = 0; s < 8; ++s) {
        g_udp_mcast_listen[s] = {};
        clearSocketDestCache(s);
    }
}

void socketClose(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    g_udp_mcast_listen[sock] = {};
    clearSocketDestCache(sock);
    invalidateSocketTxWr(sock);
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
            clearSendInterrupts(hal, sock, ir);
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
        noteSocketTxWr(sock, txWr);
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
        clearSendInterrupts(hal, sock, ir);
        return static_cast<int>(len);
    }
    if (ir & Sn_IR_TIMEOUT) {
        W5500_LOGW( "Send: sock=%d TX timeout (ARP/link)", sock);
    } else if (ir & Sn_IR_DISCON) {
        W5500_LOGW( "Send: sock=%d disconnected during send", sock);
    }
    clearSendInterrupts(hal, sock, ir);
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
                 uint32_t destIp, uint16_t destPort, bool deferSendOk) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    // Hold SPI for the whole UDP TX + SENDOK. Avoids per-poll mutex churn and
    // keeps Class 1 O->T on a tight path (SPI3 is deferred during exchange).
    std::lock_guard<std::mutex> lock(spiBusMutex());

    if (!confirmDeferredSend(hal, sock)) {
        return -1;
    }

    // Class 1 destinations are fixed per axis - skip DIPR/DPORT when cached.
    // One UDP socket serves X/Z/theta, so dest changes every O->T.
    const bool dest_cached = socketDestCached(sock, destIp, destPort);
    if (!dest_cached) {
        writeSocketDest(hal, sock, destIp, destPort);
        cacheSocketDest(sock, destIp, destPort);
    }

    // No Sn_TX_FSR / Sn_SR on the fast path: frames are tiny vs TX buffer and
    // every prior send waited for SENDOK. Check Sn_SR only on failure.

    uint16_t txWr = readSocketTxWr(hal, sock);
    hal.writeBuf(kBlockSocketTxBuf(sock), txWr, data, static_cast<uint16_t>(len));

    txWr += static_cast<uint16_t>(len);
    writeSocketReg16(hal, sock, Sn_TX_WR, txWr);
    noteSocketTxWr(sock, txWr);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);

    // Cached dest means no ARP, and each Class 1 axis owns its TX socket and
    // sends once per RPI - so SENDOK can be confirmed at the head of the next
    // send instead of spinning here. A dest change still waits (ARP).
    if (deferSendOk && dest_cached && sock < 8) {
        g_udp_send_pending[sock] = true;
        return static_cast<int>(len);
    }

    // Sn_CR (0x01) and Sn_IR (0x02) are adjacent - one 2-byte poll covers both.
    const uint32_t wait_us = dest_cached ? kUdpSendOkTimeoutUs
                                         : kUdpDestChangeSendOkTimeoutUs;
    const int64_t deadline = pollNowUs() + static_cast<int64_t>(wait_us);
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
        clearSendInterrupts(hal, sock, cr_ir[1]);
        restoreUdpMulticastDest(hal, sock);
        return static_cast<int>(len);
    }
    clearSendInterrupts(hal, sock, cr_ir[1]);
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

size_t socketRecvFromBatch(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                           size_t bufLen, UdpDatagram* out, size_t maxOut) {
    std::lock_guard<std::mutex> lock(spiBusMutex());
    return readUdpBatchUnlocked(hal, sock, buf, bufLen, out, maxOut);
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
