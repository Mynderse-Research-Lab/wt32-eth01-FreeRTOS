// W5500 hardware TCP/UDP socket implementation.
//
// All SPI register I/O goes through W5500Hal. No direct W5500::readReg calls.
// Mutex-guarded for concurrent access from multiple FreeRTOS tasks.

#include "W5500Socket.h"

#include "W5500.h"  // register/status/command constants
#include "W5500BusMutex.h"
#include "W5500DeferredLog.h"
#include "W5500Diag.h"
#include "W5500Poll.h"

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

#include "W5500SocketInternal.h"

namespace w5500 {


namespace {

constexpr uint8_t kCommonBlock    = 0x00;
constexpr uint32_t kDefaultTimeoutMs = 500;
// cfg eip_txwr_split (dropout experiment): Sn_TX_WR as two 1-byte writes.
// Each is a 4-byte SPI frame (TXDATA, no DMA) instead of one 5-byte DMA frame;
// on 2026-10-20 the 5-byte TX_WR write was the one landing wrong or lost.
bool g_txwr_split = false;

struct UdpDestCache {
    uint32_t ip = 0;
    uint16_t port = 0;
    bool valid = false;
};
static UdpDestCache g_udp_dest_cache[8] = {};

// Sn_TX_WR is host-owned; the chip never moves it. Track it in software so the
// Class 1 send path does not pay a read per frame.
struct TxWrCache {
    uint16_t wr = 0;
    bool valid = false;
};
static TxWrCache g_tx_wr_cache[8] = {};

// Time each socket last issued SEND (diagnostics only; see W5500Diag).
static int64_t g_last_send_us[8] = {};
}  // namespace

UdpMcastListen g_udp_mcast_listen[8] = {};
// Deferred Class 1 SENDOK: SEND was issued and its completion is owed at the
// head of the next send on that socket (a full RPI later).
bool g_udp_send_pending[8] = {};







void writeTxWr(W5500Hal& hal, uint8_t sock, uint16_t txWr) {
    if (g_txwr_split) {
        writeSocketReg(hal, sock, Sn_TX_WR, static_cast<uint8_t>(txWr >> 8));
        writeSocketReg(hal, sock, static_cast<uint16_t>(Sn_TX_WR + 1),
                       static_cast<uint8_t>(txWr & 0xFF));
        return;
    }
    writeSocketReg16(hal, sock, Sn_TX_WR, txWr);
}





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
        g_last_send_us[sock] = 0;
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

void noteSocketSend(uint8_t sock) {
    if (sock < 8) {
        g_last_send_us[sock] = pollNowUs();
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


bool waitCommandCompleteUnlocked(W5500Hal& hal, uint8_t sock, uint32_t timeoutMs) {
    const uint32_t timeout_us =
        (timeoutMs > (UINT32_MAX / 1000u)) ? UINT32_MAX : (timeoutMs * 1000u);
    // Cap tight CR wait - SEND clears CR quickly; do not burn seconds.
    const uint32_t capped =
        (timeout_us > kUdpSendOkTimeoutUs) ? kUdpSendOkTimeoutUs : timeout_us;
    return busyPollUsTight(capped, [&]() {
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
        return readSocketReg(hal, sock, Sn_CR) == 0;
    });
}


// Software TX state for the failure snapshot. extraPending marks a socket
// whose pending flag was already consumed by the failing confirm.
SoftTxState softTxState(uint8_t extraPending) {
    SoftTxState st;
    st.pending_mask = extraPending;
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t bit = static_cast<uint8_t>(1u << i);
        if (g_udp_send_pending[i]) st.pending_mask = static_cast<uint8_t>(st.pending_mask | bit);
        if (g_tx_wr_cache[i].valid) {
            st.tx_wr[i] = g_tx_wr_cache[i].wr;
            st.tx_wr_valid = static_cast<uint8_t>(st.tx_wr_valid | bit);
        }
        st.last_send_us[i] = g_last_send_us[i];
        if (g_udp_dest_cache[i].valid) {
            st.dest_ip[i] = g_udp_dest_cache[i].ip;
            st.dest_port[i] = g_udp_dest_cache[i].port;
            st.dest_valid = static_cast<uint8_t>(st.dest_valid | bit);
        }
    }
    return st;
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
        W5500_LOGW( "SendTo: sock=%d deferred SENDOK timeout (CR=0x%02X IR=0x%02X)",
                 sock, cr_ir[0], cr_ir[1]);
        captureFailureSnapshot(hal, sock, FailReason::kDeferredSendOkTimeout,
                               softTxState(static_cast<uint8_t>(1u << sock)));
        return false;
    }
    if (cr_ir[1] & Sn_IR_SENDOK) {
        clearSendInterrupts(hal, sock, cr_ir[1]);
        return true;
    }
    W5500_LOGW( "SendTo: sock=%d deferred send failed (IR=0x%02X)", sock,
             cr_ir[1]);
    // Snapshot before the clear so Sn_IR still shows what the chip reported.
    captureFailureSnapshot(hal, sock, FailReason::kDeferredSendFailed,
                           softTxState(static_cast<uint8_t>(1u << sock)));
    clearSendInterrupts(hal, sock, cr_ir[1]);
    return false;
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
    // 32 bits: a length of 0xFFF8 or more must not wrap past the checks.
    const uint32_t packetBytes = kUdpHeaderSize + static_cast<uint32_t>(payloadLen);
    if (payloadLen == 0 || packetBytes > avail) {
        W5500_LOGW(
            "RecvFrom: sock=%d bad UDP hdr len=%u avail=%u - discarding %u", sock,
            payloadLen, avail, avail);
        noteRxHeaderBad();
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
        // 32 bits: a length of 0xFFF8 or more must not wrap past the checks.
        const uint32_t packetBytes =
            kUdpHeaderSize + static_cast<uint32_t>(payloadLen);
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
        const uint32_t packetBytes =
            kUdpHeaderSize + static_cast<uint32_t>(payloadLen);
        if (payloadLen > 0 && packetBytes <= avail) {
            out[0].data = buf + kUdpHeaderSize;
            out[0].len = static_cast<uint16_t>(burst - kUdpHeaderSize);
            completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + packetBytes));
            return 1;
        }
    }

    // Log the head header bytes: a corrupt SPI read shows as junk here while
    // avail still adds up to whole drive frames.
    W5500_LOGW( "RecvBatch: sock=%d bad UDP hdr avail=%u rd=0x%04X hdr=%02X%02X%02X%02X "
             "%02X%02X %02X%02X - discarding %u", sock, avail, rxRd, buf[0], buf[1],
             buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], avail);
    noteRxHeaderBad();
    completeUdpRecv(hal, sock, static_cast<uint16_t>(rxRd + avail));
    return 0;
}


void socketSetTxWrSplit(bool split) {
    g_txwr_split = split;
}

int socketSendTo(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len,
                 uint32_t destIp, uint16_t destPort, bool deferSendOk) {
    if (len == 0) return 0;
    if (len > 0xFFFF) return -1;

    // Hold SPI for the whole UDP TX + SENDOK. Avoids per-poll mutex churn and
    // keeps Class 1 O->T on a tight path (SPI3 is deferred during exchange).
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());

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
    writeTxWr(hal, sock, txWr);
    noteSocketTxWr(sock, txWr);
    writeSocketReg(hal, sock, Sn_CR, Sn_CR_SEND);
    noteSocketSend(sock);

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
        captureFailureSnapshot(hal, sock,
                               (cr_ir[0] != 0) ? FailReason::kSendCmdTimeout
                                               : FailReason::kSendOkTimeout,
                               softTxState(0));
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
            std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
            return readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR) >=
                   kUdpHeaderSize;
        });
    } else {
        dataReady = busyPollUs(timeout_us, [&]() {
            std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
            return readSocketSizeReg16Stable(hal, sock, Sn_RX_RSR) >=
                   kUdpHeaderSize;
        });
    }

    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    if (!dataReady) {
        return 0;
    }
    return readUdpDatagramUnlocked(hal, sock, buf, maxLen);
}

int socketRecvFromNonBlocking(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                              size_t maxLen) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    return readUdpDatagramUnlocked(hal, sock, buf, maxLen);
}

size_t socketRecvFromBatch(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                           size_t bufLen, UdpDatagram* out, size_t maxOut) {
    std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
    return readUdpBatchUnlocked(hal, sock, buf, bufLen, out, maxOut);
}


}  // namespace w5500
