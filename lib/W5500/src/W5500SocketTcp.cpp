/**
 * @file W5500SocketTcp.cpp
 * @brief W5500 TCP open, connect, listen, send and receive.
 */

#include "W5500Socket.h"

#include "W5500.h"
#include "W5500BusMutex.h"
#include "W5500DeferredLog.h"
#include "W5500Diag.h"
#include "W5500Poll.h"
#include "W5500SocketInternal.h"

#include <cstring>
#include <functional>
#include <mutex>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <chrono>
#include <thread>
#endif

namespace w5500 {

namespace {

constexpr uint32_t kCommandTimeoutMs  = 10;
static uint16_t g_next_tcp_source_port = 49152;

uint16_t nextTcpSourcePort() {
    const uint16_t port = g_next_tcp_source_port++;
    if (g_next_tcp_source_port < 49152) {
        g_next_tcp_source_port = 49152;
    }
    return port;
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

}  // namespace

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

namespace {

bool waitSendComplete(W5500Hal& hal, uint8_t sock, uint32_t timeout_us) {
    // Tight spin: Class 1 UDP SENDOK is typically microseconds; taskYIELD at
    // 1000 Hz was costing ~1 ms per axis (~3 ms dual O->T).
    return busyPollUsTight(timeout_us, [&]() {
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
        uint8_t ir = readSocketReg(hal, sock, Sn_IR);
        if (ir & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON)) {
            return true;
        }
        return false;
    });
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
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    return nextTcpSourcePort();
}

// --- Public API --------------------------------------------------------------

int socketOpen(W5500Hal& hal, SocketMode mode, uint16_t localPort,
               uint32_t udpMulticastListenIp) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

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
                     static_cast<unsigned>((udpMulticastListenIp >> 24) & 0xFF),
                     static_cast<unsigned>((udpMulticastListenIp >> 16) & 0xFF),
                     static_cast<unsigned>((udpMulticastListenIp >> 8) & 0xFF),
                     static_cast<unsigned>(udpMulticastListenIp & 0xFF));
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

SoftTxState socketSoftTxState() {
    return softTxState(0);
}

void socketClose(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
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
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

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

    // Poll Sn_SR without holding SPI between reads (Class 1 / other sockets
    // must proceed). INIT during ARP is normal on W5500; SYNSENT once ARP
    // succeeds; CLOSED on ARP/TCP failure. Sleep between reads so tasks below
    // the scanner priority are not starved for the whole connect timeout.
    uint32_t elapsed = 0;
    constexpr uint32_t kDiagIntervalMs = 1000;
    uint32_t nextDiag = kDiagIntervalMs;
    uint8_t finalSr = SOCK_CLOSED;

    if (!sleepPollMs(timeoutMs, kConnectPollIntervalMs, [&](uint32_t elapsedMs) {
        uint8_t sr = 0;
        {
            std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
            sr = readSocketReg(hal, sock, Sn_SR);
        }
        finalSr = sr;
        elapsed = elapsedMs;

#ifdef ESP_PLATFORM
        if (elapsed >= nextDiag) {
            W5500_LOGI("Connect diag sock=%d src_port=%u sr=0x%02X (%s) elapsed=%lu ms",
                     sock, sourcePort, sr, socketStatusName(sr),
                     static_cast<unsigned long>(elapsed));
            nextDiag += kDiagIntervalMs;
        }
#endif

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
            std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
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
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

    writeSocketReg16(hal, sock, Sn_PORT, port);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_LISTEN);
    return waitCommandComplete(hal, sock, kCommandTimeoutMs);
}

int socketSend(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    {
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

        uint16_t freeSize = readSocketSizeReg16Stable(hal, sock, Sn_TX_FSR);
        if (freeSize < static_cast<uint16_t>(len)) {
            W5500_LOGW( "Send: sock=%d need %u bytes, TX free %u",
                     sock, static_cast<unsigned>(len), freeSize);
            return -1;
        }

        // A socketSendNoWait leaves its SENDOK set; it must not pass for ours.
        const uint8_t stale = readSocketReg(hal, sock, Sn_IR) & Sn_IR_SENDOK;
        clearSendInterrupts(hal, sock, stale);

        uint16_t txWr = readSocketReg16(hal, sock, Sn_TX_WR);
        hal.writeBuf(kBlockSocketTxBuf(sock), txWr, data, static_cast<uint16_t>(len));

        txWr += static_cast<uint16_t>(len);
        writeTxWr(hal, sock, txWr);
        noteSocketTxWr(sock, txWr);
        writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);
        noteSocketSend(sock);
    }

    if (!waitCommandCompleteUnlocked(hal, sock, kCommandTimeoutMs)) {
        W5500_LOGW( "Send: sock=%d SEND cmd timeout", sock);
        return -1;
    }

    if (!waitSendComplete(hal, sock, kTcpSendOkTimeoutUs)) {
        W5500_LOGW( "Send: sock=%d SENDOK timeout", sock);
        return -1;
    }

    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
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

int socketSendNoWait(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

    // The previous call's outcome is still in Sn_IR.
    const uint8_t ir = readSocketReg(hal, sock, Sn_IR) &
                       (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON);
    clearSendInterrupts(hal, sock, ir);
    if ((ir & (Sn_IR_TIMEOUT | Sn_IR_DISCON)) != 0) {
        return -1;
    }
    if (readSocketReg(hal, sock, Sn_SR) != SOCK_ESTABLISHED) {
        return -1;
    }
    if (readSocketSizeReg16Stable(hal, sock, Sn_TX_FSR) < static_cast<uint16_t>(len)) {
        return -1;
    }

    uint16_t txWr = readSocketReg16(hal, sock, Sn_TX_WR);
    hal.writeBuf(kBlockSocketTxBuf(sock), txWr, data, static_cast<uint16_t>(len));
    txWr += static_cast<uint16_t>(len);
    writeTxWr(hal, sock, txWr);
    noteSocketTxWr(sock, txWr);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);
    noteSocketSend(sock);
    return static_cast<int>(len);
}

int socketRecv(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
               uint32_t timeoutMs) {
    // Poll without holding the SPI mutex for the whole timeout (other sockets
    // must keep exchanging Class 1 frames).
    bool dataReady = busyPollMs(timeoutMs, [&]() {
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
        uint16_t avail = readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR);
        if (avail > 0) return true;
        uint8_t sr = readSocketReg(hal, sock, Sn_SR);
        return (sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT);
    });
    (void)dataReady;

    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

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


bool socketBind(W5500Hal& hal, uint8_t sock, uint16_t port) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    writeSocketReg16(hal, sock, Sn_PORT, port);
    return true;
}

bool socketIsConnected(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr == SOCK_ESTABLISHED);
}

bool socketIsOpen(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    uint8_t sr = readSocketReg(hal, sock, Sn_SR);
    return (sr != SOCK_CLOSED);
}

int socketRxAvailable(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    return static_cast<int>(readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR));
}

uint8_t socketStatus(W5500Hal& hal, uint8_t sock) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    return readSocketReg(hal, sock, Sn_SR);
}

}  // namespace w5500
