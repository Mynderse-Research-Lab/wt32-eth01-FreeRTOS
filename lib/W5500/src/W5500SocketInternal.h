/**
 * @file W5500SocketInternal.h
 * @brief Register helpers and the socket state shared by the TCP and UDP files.
 *
 * Private to this library. The helpers are inline so the Class 1 UDP path
 * still sees their bodies.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "W5500.h"
#include "W5500Diag.h"
#include "W5500Hal.h"

#ifdef ESP_PLATFORM
// Socket calls run inside the W5500 bus mutex and on the Class 1 path, so
// they never print directly: a line is queued and EipLog prints it (G-B13).
#define W5500_LOGI(...) ::w5500::deferLog(TAG, 'I', esp_log_timestamp(), __VA_ARGS__)
#define W5500_LOGW(...) ::w5500::deferLog(TAG, 'W', esp_log_timestamp(), __VA_ARGS__)
#define W5500_LOGE(...) ::w5500::deferLog(TAG, 'E', esp_log_timestamp(), __VA_ARGS__)
#else
#define W5500_LOGI(...) ((void)0)
#define W5500_LOGW(...) ((void)0)
#define W5500_LOGE(...) ((void)0)
#endif

namespace w5500 {

static const char* TAG = "W5500Sock";

struct UdpMcastListen {
    uint32_t ip = 0;
    uint16_t port = 0;
};
extern UdpMcastListen g_udp_mcast_listen[8];
extern bool g_udp_send_pending[8];

void writeTxWr(W5500Hal& hal, uint8_t sock, uint16_t txWr);
void cacheSocketDest(uint8_t sock, uint32_t ip, uint16_t port);
void clearSocketDestCache(uint8_t sock);
void invalidateSocketTxWr(uint8_t sock);
uint16_t readSocketTxWr(W5500Hal& hal, uint8_t sock);
void noteSocketTxWr(uint8_t sock, uint16_t wr);
void noteSocketSend(uint8_t sock);
bool waitCommandCompleteUnlocked(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs);
SoftTxState softTxState(uint8_t extraPending);

constexpr size_t   kUdpHeaderSize     = 8;

inline
uint8_t readSocketReg(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    return hal.readReg(kBlockSocketReg(sock), addr);
}

inline
void writeSocketReg(W5500Hal& hal, uint8_t sock, uint16_t addr, uint8_t val) {
    hal.writeReg(kBlockSocketReg(sock), addr, val);
}

inline
uint16_t readSocketReg16(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    return hal.readReg16(kBlockSocketReg(sock), addr);
}

inline
uint16_t readSocketSizeReg16Stable(W5500Hal& hal, uint8_t sock, uint16_t addr) {
    uint16_t a = readSocketReg16(hal, sock, addr);
    for (int i = 0; i < 8; ++i) {
        uint16_t b = readSocketReg16(hal, sock, addr);
        if (a == b) return a;
        a = b;
    }
    return a;
}

inline
void writeSocketReg16(W5500Hal& hal, uint8_t sock, uint16_t addr, uint16_t val) {
    hal.writeReg16(kBlockSocketReg(sock), addr, val);
}

inline
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

// Sn_IR is write-1-to-clear and every caller already holds the polled value,
// so no read-back is needed on the Class 1 hot path.
inline
void clearSendInterrupts(W5500Hal& hal, uint8_t sock, uint8_t ir) {
    if (ir != 0) {
        writeSocketReg(hal, sock, Sn_IR, ir);
    }
}

inline
void readRxRsrAndRd(W5500Hal& hal, uint8_t sock, uint16_t& rsr, uint16_t& rd) {
    // Sn_RX_RSR (0x26) and Sn_RX_RD (0x28) are adjacent — one 4-byte burst.
    uint8_t b[4] = {};
    hal.readBuf(kBlockSocketReg(sock), Sn_RX_RSR, b, 4);
    rsr = static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
    rd = static_cast<uint16_t>((static_cast<uint16_t>(b[2]) << 8) | b[3]);
}

// Settled Sn_RX_RSR / Sn_RX_RD pair. The chip advances RSR asynchronously, so
// re-read until two bursts agree. False when no full UDP header is queued.
inline
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

}  // namespace w5500
