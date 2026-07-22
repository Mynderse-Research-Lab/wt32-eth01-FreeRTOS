// W5500 hardware TCP/UDP socket implementation.
//
// All SPI register I/O goes through W5500Hal. No direct W5500::readReg calls.
// Mutex-guarded for concurrent access from multiple FreeRTOS tasks.

#include "W5500Socket.h"

#include "W5500.h"  // register/status/command constants

#include <cstring>
#include <functional>
#include <mutex>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
// Host test stubs — the test replaces these with its own timing
#include <chrono>
#include <thread>
#endif

namespace w5500 {

static const char* TAG = "W5500Sock";

namespace {

// --- Internal helpers --------------------------------------------------------

constexpr uint8_t kCommonBlock    = 0x00;
constexpr uint32_t kDefaultTimeoutMs = 5000;
constexpr uint32_t kCommandTimeoutMs  = 100;
constexpr uint32_t kSendOkTimeoutMs   = 500;
constexpr size_t   kUdpHeaderSize     = 8;  // destIP(4) + destPort(2) + len(2)

static std::mutex g_spi_mutex;
static uint16_t g_next_tcp_source_port = 49152;

// Multicast UDP sockets: Sn_DIPR/Sn_DPORT must be restored after SEND
// (sendTo overwrites them with the unicast O->T destination).
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

// W5500 datasheet: Sn_RX_RSR / Sn_TX_FSR must be read until two consecutive
// values match — a single read can return a transient/zero value.
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

// Wait for a socket command to complete (Sn_CR clears to 0).
bool waitCommandComplete(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
#ifndef ESP_PLATFORM
    // Host: simple spin with sleep
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (readSocketReg(hal, sock, Sn_CR) == 0) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return false;
#else
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (xTaskGetTickCount() < deadline) {
        if (readSocketReg(hal, sock, Sn_CR) == 0) return true;
        vTaskDelay(1);
    }
    return false;
#endif
}

// Issue a socket command and wait for it to complete.
bool issueCommand(W5500Hal& hal, uint8_t sock, uint8_t cmd) {
    writeSocketReg(hal, sock, Sn_CR, cmd);
    return waitCommandComplete(hal, sock, kCommandTimeoutMs);
}

// Find first free socket (Sn_SR == SOCK_CLOSED).
int findFreeSocket(W5500Hal& hal) {
    for (uint8_t s = 0; s < 8; ++s) {
        if (readSocketReg(hal, s, Sn_SR) == SOCK_CLOSED) {
            return static_cast<int>(s);
        }
    }
    return -1;
}

// Busy-poll with yield. Returns false on timeout.
bool busyPollYield(uint32_t timeoutMs, const std::function<bool()>& check) {
#ifndef ESP_PLATFORM
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (check()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
#else
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (xTaskGetTickCount() < deadline) {
        if (check()) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
#endif
}

// Write an IP address (uint32_t in network byte order) to Sn_DIPR (4 bytes).
void writeSocketDIPR(W5500Hal& hal, uint8_t sock, uint32_t ip) {
    uint8_t data[4] = {
        static_cast<uint8_t>((ip >> 24) & 0xFF),
        static_cast<uint8_t>((ip >> 16) & 0xFF),
        static_cast<uint8_t>((ip >> 8) & 0xFF),
        static_cast<uint8_t>(ip & 0xFF)
    };
    // Sn_DIPR is at offset 0x000C, 4 bytes
    for (int i = 0; i < 4; ++i) {
        hal.writeReg(kBlockSocketReg(sock), static_cast<uint16_t>(Sn_DIPR + i), data[i]);
    }
}

void restoreUdpMulticastDest(W5500Hal& hal, uint8_t sock) {
    const UdpMcastListen& m = g_udp_mcast_listen[sock];
    if (m.ip == 0) return;
    writeSocketDIPR(hal, sock, m.ip);
    writeSocketReg16(hal, sock, Sn_DPORT, m.port);
}

// Wait for SEND_OK (or TIMEOUT/DISCON) after Sn_CR_SEND.
// Caller must NOT hold g_spi_mutex — polls with short locked reads so Class 1
// on other sockets can keep exchanging.
bool waitSendComplete(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
    return busyPollYield(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(g_spi_mutex);
        uint8_t ir = readSocketReg(hal, sock, Sn_IR);
        if (ir & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON)) {
            return true;
        }
        return false;
    });
}

// Sn_CR clear wait without holding the SPI mutex across yields.
bool waitCommandCompleteUnlocked(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
    return busyPollYield(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(g_spi_mutex);
        return readSocketReg(hal, sock, Sn_CR) == 0;
    });
}

// Clear socket interrupt flags we care about after send.
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

}  // namespace

uint16_t nextEphemeralPort() {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    return nextTcpSourcePort();
}

// --- Public API --------------------------------------------------------------

int socketOpen(W5500Hal& hal, SocketMode mode, uint16_t localPort,
               uint32_t udpMulticastListenIp) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);

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
        writeSocketDIPR(hal, sockU8, udpMulticastListenIp);
        writeSocketReg16(hal, sockU8, Sn_DPORT, localPort);
        g_udp_mcast_listen[sockU8] = {udpMulticastListenIp, localPort};
    } else {
        g_udp_mcast_listen[sockU8] = {};
    }

    writeSocketReg(hal, sockU8, Sn_MR, mr);

    if (!issueCommand(hal, sockU8, Sn_CR_OPEN)) {
        return -1;
    }

    if (mode == SocketMode::kUdp) {
        if (!busyPollYield(500, [&]() {
            const uint8_t sr = readSocketReg(hal, sockU8, Sn_SR);
            return (sr == SOCK_UDP || sr == SOCK_CLOSED);
        })) {
            ESP_LOGW(TAG, "UDP open: sock=%d timed out waiting for SOCK_UDP", sock);
            return -1;
        }
        const uint8_t status = readSocketReg(hal, sockU8, Sn_SR);
        if (status != SOCK_UDP) {
            ESP_LOGW(TAG, "UDP open: sock=%d final Sr=0x%02X (%s)",
                     sock, status, socketStatusName(status));
            return -1;
        }
#ifdef ESP_PLATFORM
        if (udpMulticastListenIp != 0) {
            ESP_LOGI(TAG, "UDP multicast sock=%d port=%u group=%u.%u.%u.%u",
                     sock, localPort,
                     (udpMulticastListenIp >> 24) & 0xFF,
                     (udpMulticastListenIp >> 16) & 0xFF,
                     (udpMulticastListenIp >> 8) & 0xFF,
                     udpMulticastListenIp & 0xFF);
        } else {
            ESP_LOGI(TAG, "UDP sock=%d bound port=%u", sock, localPort);
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
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    g_udp_mcast_listen[sock] = {};
    issueCommand(hal, sock, Sn_CR_CLOSE);
    writeSocketReg(hal, sock, Sn_MR, Sn_MR_CLOSE);
    issueCommand(hal, sock, Sn_CR_CLOSE);
    busyPollYield(200, [&]() {
        return readSocketReg(hal, sock, Sn_SR) == SOCK_CLOSED;
    });
}

bool socketConnect(W5500Hal& hal, uint8_t sock, uint32_t ip, uint16_t port,
                   uint32_t timeoutMs) {
    uint16_t sourcePort = 0;
    {
        // Hold SPI only for setup + CONNECT + Sn_CR clear (ms). Do NOT hold
        // across the ESTABLISHED poll — that starved Class 1 for up to 5s.
        std::lock_guard<std::mutex> lock(g_spi_mutex);

        sourcePort = readSocketReg16(hal, sock, Sn_PORT);
        if (sourcePort == 0) {
            sourcePort = nextTcpSourcePort();
            writeSocketReg16(hal, sock, Sn_PORT, sourcePort);
        }

        const uint8_t sr0 = readSocketReg(hal, sock, Sn_SR);
        if (sr0 != SOCK_INIT) {
            ESP_LOGW(TAG, "Connect: sock=%d not INIT (Sr=0x%02X %s) — abort",
                     sock, sr0, socketStatusName(sr0));
            return false;
        }

        // Sn_CR must be idle before CONNECT or the command is dropped.
        if (!waitCommandComplete(hal, sock, kCommandTimeoutMs)) {
            ESP_LOGE(TAG, "Connect: Sn_CR busy before CONNECT sock=%d", sock);
            return false;
        }

        writeSocketDIPR(hal, sock, ip);
        writeSocketReg16(hal, sock, Sn_DPORT, port);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_CONNECT);
        if (!waitCommandComplete(hal, sock, kCommandTimeoutMs)) {
            ESP_LOGE(TAG, "Connect: CONNECT cmd timed out on socket %d src_port=%u",
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

    if (!busyPollYield(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(g_spi_mutex);
        uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        finalSr = sr;

#ifdef ESP_PLATFORM
        if (elapsed >= nextDiag) {
            ESP_LOGI(TAG, "Connect diag sock=%d src_port=%u sr=0x%02X (%s) elapsed=%lu ms",
                     sock, sourcePort, sr, socketStatusName(sr),
                     static_cast<unsigned long>(elapsed));
            nextDiag += kDiagIntervalMs;
        }
#endif
        ++elapsed;

        return (sr == SOCK_ESTABLISHED || sr == SOCK_CLOSED ||
                sr == SOCK_CLOSE_WAIT);
    })) {
        ESP_LOGW(TAG, "Connect: timeout sock=%d src_port=%u final Sr=0x%02X (%s) elapsed=%lu ms",
                 sock, sourcePort, finalSr, socketStatusName(finalSr),
                 static_cast<unsigned long>(elapsed));
        return false;
    }

    if (finalSr != SOCK_ESTABLISHED) {
        uint8_t ir = 0;
        {
            std::lock_guard<std::mutex> lock(g_spi_mutex);
            ir = readSocketReg(hal, sock, Sn_IR);
            clearSendInterrupts(hal, sock);
        }
        ESP_LOGW(TAG,
                 "Connect: failed sock=%d src_port=%u Sr=0x%02X (%s) IR=0x%02X "
                 "(CLOSED after INIT usually = ARP/TCP fail — check drive link/"
                 "E602 clear, daisy-chain)",
                 sock, sourcePort, finalSr, socketStatusName(finalSr), ir);
        return false;
    }

    ESP_LOGI(TAG, "Connect: sock=%d src_port=%u final Sr=0x%02X (%s)",
             sock, sourcePort, finalSr, socketStatusName(finalSr));
    return true;
}

bool socketListen(W5500Hal& hal, uint8_t sock, uint16_t port) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);

    writeSocketReg16(hal, sock, Sn_PORT, port);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_LISTEN);
    return waitCommandComplete(hal, sock, kCommandTimeoutMs);
}

int socketSend(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    {
        std::lock_guard<std::mutex> lock(g_spi_mutex);

        uint16_t freeSize = readSocketSizeReg16Stable(hal, sock, Sn_TX_FSR);
        if (freeSize < static_cast<uint16_t>(len)) {
            ESP_LOGW(TAG, "Send: sock=%d need %u bytes, TX free %u",
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
        ESP_LOGW(TAG, "Send: sock=%d SEND cmd timeout", sock);
        return -1;
    }

    if (!waitSendComplete(hal, sock, kSendOkTimeoutMs)) {
        ESP_LOGW(TAG, "Send: sock=%d SENDOK timeout", sock);
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_spi_mutex);
    uint8_t ir = readSocketReg(hal, sock, Sn_IR);
    if (ir & Sn_IR_SENDOK) {
        clearSendInterrupts(hal, sock);
        return static_cast<int>(len);
    }
    if (ir & Sn_IR_TIMEOUT) {
        ESP_LOGW(TAG, "Send: sock=%d TX timeout (ARP/link)", sock);
    } else if (ir & Sn_IR_DISCON) {
        ESP_LOGW(TAG, "Send: sock=%d disconnected during send", sock);
    }
    clearSendInterrupts(hal, sock);
    return -1;
}

int socketRecv(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
               uint32_t timeoutMs) {
    // Poll without holding the SPI mutex for the whole timeout (other sockets
    // must keep exchanging Class 1 frames).
    bool dataReady = busyPollYield(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(g_spi_mutex);
        uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
        if (avail > 0) return true;
        uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        return (sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT);
    });
    (void)dataReady;

    std::lock_guard<std::mutex> lock(g_spi_mutex);

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

    {
        std::lock_guard<std::mutex> lock(g_spi_mutex);

        const uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        if (sr != SOCK_UDP) {
            ESP_LOGW(TAG, "SendTo: sock=%d not UDP (Sr=0x%02X)", sock, sr);
            return -1;
        }

        writeSocketDIPR(hal, sock, destIp);
        writeSocketReg16(hal, sock, Sn_DPORT, destPort);

        uint16_t freeSize = readSocketSizeReg16Stable(hal, sock, Sn_TX_FSR);
        if (freeSize < static_cast<uint16_t>(len)) {
            ESP_LOGW(TAG, "SendTo: sock=%d need %u TX free %u",
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
        ESP_LOGW(TAG, "SendTo: sock=%d SEND cmd timeout", sock);
        return -1;
    }

    if (!waitSendComplete(hal, sock, kSendOkTimeoutMs)) {
        ESP_LOGW(TAG, "SendTo: sock=%d SENDOK timeout", sock);
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_spi_mutex);
    uint8_t ir = readSocketReg(hal, sock, Sn_IR);
    if (ir & Sn_IR_SENDOK) {
        clearSendInterrupts(hal, sock);
        restoreUdpMulticastDest(hal, sock);
        return static_cast<int>(len);
    }
    clearSendInterrupts(hal, sock);
    ESP_LOGW(TAG, "SendTo: sock=%d no SENDOK (IR=0x%02X)", sock, ir);
    return -1;
}

int socketRecvFrom(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
                   uint32_t timeoutMs) {
    // Do not hold g_spi_mutex across the wait — Class 1 O->T sendTo on the
    // same (or another) socket must proceed while we wait for T->O.
    bool dataReady = busyPollYield(timeoutMs, [&]() {
        std::lock_guard<std::mutex> lock(g_spi_mutex);
        return readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR) >= kUdpHeaderSize;
    });

    std::lock_guard<std::mutex> lock(g_spi_mutex);

    uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
    if (!dataReady || avail < kUdpHeaderSize) {
#ifdef ESP_PLATFORM
        if (avail > 0) {
            ESP_LOGW(TAG, "RecvFrom: sock=%d timeout with RX_RSR=%u (need %u hdr)",
                     sock, avail, static_cast<unsigned>(kUdpHeaderSize));
        }
#endif
        return 0;
    }

    uint16_t rxRd = readSocketReg16(hal, sock, Sn_RX_RD);

    uint8_t hdr[kUdpHeaderSize];
    hal.readBuf(kBlockSocketRxBuf(sock), rxRd, hdr, kUdpHeaderSize);

    // W5500 UDP RX header: peer IP (4) + peer port (2) + payload length (2 BE).
    const uint16_t payloadLen =
        static_cast<uint16_t>((static_cast<uint16_t>(hdr[6]) << 8) | hdr[7]);
    const uint16_t packetBytes = static_cast<uint16_t>(kUdpHeaderSize + payloadLen);
    if (payloadLen == 0 || packetBytes > avail) {
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG,
                 "RecvFrom: sock=%d bad UDP hdr len=%u avail=%u — discarding %u",
                 sock, payloadLen, avail, avail);
#endif
        // Drain whatever is reported so the socket cannot wedge.
        rxRd = static_cast<uint16_t>(rxRd + avail);
        writeSocketReg16(hal, sock, Sn_RX_RD, rxRd);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
        waitCommandComplete(hal, sock, kCommandTimeoutMs);
        return 0;
    }

    uint16_t toRead =
        (static_cast<size_t>(payloadLen) < maxLen) ? payloadLen
                                                   : static_cast<uint16_t>(maxLen);

    hal.readBuf(kBlockSocketRxBuf(sock),
                static_cast<uint16_t>(rxRd + kUdpHeaderSize), buf, toRead);

    // Always advance past the full datagram (header + declared payload), even
    // if the caller buffer truncated the copy.
    rxRd = static_cast<uint16_t>(rxRd + packetBytes);
    writeSocketReg16(hal, sock, Sn_RX_RD, rxRd);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_RECV);
    waitCommandComplete(hal, sock, kCommandTimeoutMs);

    return static_cast<int>(toRead);
}

bool socketBind(W5500Hal& hal, uint8_t sock, uint16_t port) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    writeSocketReg16(hal, sock, Sn_PORT, port);
    return true;
}

bool socketIsConnected(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr == SOCK_ESTABLISHED);
}

bool socketIsOpen(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr != SOCK_CLOSED);
}

int socketRxAvailable(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    return static_cast<int>(readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR));
}

uint8_t socketStatus(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<std::mutex> lock(g_spi_mutex);
    return readSocketReg(hal, sock, Sn_SR);
}

}  // namespace w5500
