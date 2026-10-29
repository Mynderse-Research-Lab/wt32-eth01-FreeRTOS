// W5500 fault diagnostics (see W5500Diag.h).

#include "W5500Diag.h"

#include "W5500.h"
#include "W5500BusMutex.h"
#include "W5500Poll.h"

#include <cstdio>
#include <cstring>
#include <mutex>

namespace w5500 {

namespace {

constexpr uint16_t kRegIr = 0x0015;   // common Interrupt Register
constexpr uint16_t kRegImr = 0x0016;  // common Interrupt Mask (then SIR, SIMR, RTR, RCR)
constexpr uint8_t kExpectedVersion = 0x04;
constexpr uint16_t kSnDhar = 0x0006;  // socket destination MAC (then DIPR, DPORT)

std::mutex& diagMutex() {
    static std::mutex m;
    return m;
}

FailureSnapshot g_snapshot;
bool g_snapshot_valid = false;

uint8_t g_expected_mac[6] = {};
uint8_t g_expected_ip[4] = {};
bool g_expected_valid = false;

uint8_t g_exp_rx_kb[8] = {};
uint8_t g_exp_tx_kb[8] = {};
bool g_exp_buf_valid = false;

SpiCheckStats g_stats;

uint16_t be16(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

void captureFailureSnapshot(W5500Hal& hal, uint8_t sock, FailReason reason,
                            const SoftTxState& soft) {
    {
        std::lock_guard<std::mutex> lock(diagMutex());
        if (g_snapshot_valid) return;
    }

    FailureSnapshot snap;
    snap.t_us = pollNowUs();
    snap.sock = sock;
    snap.reason = reason;
    snap.soft = soft;
    snap.version = hal.readReg(kBlockCommon(), REG_VERSIONR);
    snap.phycfgr = hal.readReg(kBlockCommon(), REG_PHYCFGR);
    snap.ir = hal.readReg(kBlockCommon(), kRegIr);
    snap.sir = hal.readReg(kBlockCommon(), REG_SIR);
    hal.readBuf(kBlockCommon(), REG_SHAR, snap.mac, 6);
    hal.readBuf(kBlockCommon(), REG_SIPR, snap.ip, 4);
    snap.mr = hal.readReg(kBlockCommon(), REG_MR);
    {
        // IMR(0x16) SIR(0x17) SIMR(0x18) RTR(0x19-0x1A) RCR(0x1B): one burst.
        uint8_t c[6] = {};
        hal.readBuf(kBlockCommon(), kRegImr, c, 6);
        snap.imr = c[0];
        snap.simr = c[2];
        snap.rtr = be16(&c[3]);
        snap.rcr = c[5];
    }
    for (uint8_t i = 0; i < 8; ++i) {
        // Sn_MR..Sn_SR are adjacent: one 4-byte burst.
        uint8_t head[4] = {};
        hal.readBuf(kBlockSocketReg(i), Sn_MR, head, 4);
        // Sn_TX_FSR..Sn_RX_RSR (0x20..0x27) in one 8-byte burst.
        uint8_t ptr[8] = {};
        hal.readBuf(kBlockSocketReg(i), Sn_TX_FSR, ptr, 8);
        SocketRegs& r = snap.s[i];
        r.mr = head[0];
        r.cr = head[1];
        r.ir = head[2];
        r.sr = head[3];
        r.tx_fsr = be16(&ptr[0]);
        r.tx_rd = be16(&ptr[2]);
        r.tx_wr = be16(&ptr[4]);
        r.rx_rsr = be16(&ptr[6]);
        uint8_t port[2] = {};
        hal.readBuf(kBlockSocketReg(i), Sn_PORT, port, 2);
        r.port = be16(port);
        // Sn_RXBUF_SIZE (0x1E) and Sn_TXBUF_SIZE (0x1F) are adjacent.
        uint8_t sz[2] = {};
        hal.readBuf(kBlockSocketReg(i), Sn_RXBUF_SIZE, sz, 2);
        r.rxbuf_kb = sz[0];
        r.txbuf_kb = sz[1];
        // Sn_DHAR (0x06..0x0B), Sn_DIPR (0x0C..0x0F), Sn_DPORT (0x10..0x11).
        uint8_t dst[12] = {};
        hal.readBuf(kBlockSocketReg(i), kSnDhar, dst, 12);
        std::memcpy(r.dhar, dst, 6);
        std::memcpy(r.dipr, dst + 6, 4);
        r.dport = be16(dst + 10);
    }

    std::lock_guard<std::mutex> lock(diagMutex());
    snap.buf_expected = g_exp_buf_valid ? 1 : 0;
    std::memcpy(snap.exp_rx_kb, g_exp_rx_kb, 8);
    std::memcpy(snap.exp_tx_kb, g_exp_tx_kb, 8);
    if (!g_snapshot_valid) {
        g_snapshot = snap;
        g_snapshot_valid = true;
    }
}

bool takeFailureSnapshot(FailureSnapshot& out) {
    std::lock_guard<std::mutex> lock(diagMutex());
    if (!g_snapshot_valid) return false;
    out = g_snapshot;
    g_snapshot_valid = false;
    return true;
}

const char* failReasonName(FailReason reason) {
    switch (reason) {
        case FailReason::kDeferredSendOkTimeout: return "deferred SENDOK timeout";
        case FailReason::kDeferredSendFailed:    return "deferred send failed";
        case FailReason::kSendCmdTimeout:        return "SEND cmd timeout";
        case FailReason::kSendOkTimeout:         return "SENDOK timeout";
        case FailReason::kInputStale:            return "T->O stale";
        case FailReason::kLinkDown:              return "link down";
    }
    return "unknown";
}

size_t formatPhy(uint8_t phycfgr, char* buf, size_t len) {
    if (buf == nullptr || len == 0) return 0;
    // PHYCFGR bit0 LNK, bit1 SPD (1=100M), bit2 DPX (1=full).
    const int n = std::snprintf(buf, len, "lnk=%u %s %s",
                                static_cast<unsigned>(phycfgr & 0x01),
                                (phycfgr & 0x02) ? "100M" : "10M",
                                (phycfgr & 0x04) ? "FD" : "HD");
    if (n < 0) return 0;
    return (static_cast<size_t>(n) < len) ? static_cast<size_t>(n) : len - 1;
}

size_t formatSnapshotCommon(const FailureSnapshot& snap, char* buf, size_t len) {
    if (buf == nullptr || len == 0) return 0;
    char phy[24];
    formatPhy(snap.phycfgr, phy, sizeof(phy));
    const int n = std::snprintf(
        buf, len,
        "sock=%u %s t=%lld us ver=0x%02X phy=0x%02X (%s) ir=0x%02X "
        "sir=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X ip=%u.%u.%u.%u "
        "pending=0x%02X cmr=0x%02X imr=0x%02X simr=0x%02X rtr=%u rcr=%u",
        static_cast<unsigned>(snap.sock), failReasonName(snap.reason),
        static_cast<long long>(snap.t_us), snap.version, snap.phycfgr, phy,
        snap.ir, snap.sir,
        snap.mac[0], snap.mac[1], snap.mac[2], snap.mac[3], snap.mac[4],
        snap.mac[5], snap.ip[0], snap.ip[1], snap.ip[2], snap.ip[3],
        snap.soft.pending_mask, snap.mr, snap.imr, snap.simr,
        static_cast<unsigned>(snap.rtr), static_cast<unsigned>(snap.rcr));
    if (n < 0) return 0;
    return (static_cast<size_t>(n) < len) ? static_cast<size_t>(n) : len - 1;
}

size_t formatSnapshotSocket(const FailureSnapshot& snap, uint8_t sock,
                            char* buf, size_t len) {
    if (buf == nullptr || len == 0 || sock >= 8) return 0;
    const SocketRegs& r = snap.s[sock];
    // Firmware's cached Sn_TX_WR next to the chip's: a mismatch means a TX_WR
    // write never landed. last_send is how long before the snapshot SEND went.
    // Destination as the chip holds it; flagged when it differs from the
    // UDP destination the firmware cached (written once per socket).
    char dst[96];
    const bool dst_known = (snap.soft.dest_valid & (1u << sock)) != 0;
    uint8_t raw[6];
    std::memcpy(raw, r.dipr, 4);
    raw[4] = static_cast<uint8_t>(r.dport >> 8);
    raw[5] = static_cast<uint8_t>(r.dport & 0xFF);
    const bool dst_changed =
        dst_known && !destMatches(raw, snap.soft.dest_ip[sock], snap.soft.dest_port[sock]);
    std::snprintf(dst, sizeof(dst),
                  " dst=%u.%u.%u.%u:%u dmac=%02X:%02X:%02X:%02X:%02X:%02X%s",
                  r.dipr[0], r.dipr[1], r.dipr[2], r.dipr[3],
                  static_cast<unsigned>(r.dport), r.dhar[0], r.dhar[1], r.dhar[2],
                  r.dhar[3], r.dhar[4], r.dhar[5], dst_changed ? "(DEST CHANGED)" : "");
    char sw[80] = "";
    if (snap.soft.tx_wr_valid & (1u << sock)) {
        const int64_t ago_us = (snap.soft.last_send_us[sock] > 0)
                                   ? snap.t_us - snap.soft.last_send_us[sock]
                                   : -1;
        std::snprintf(sw, sizeof(sw), " sw_wr=0x%04X%s last_send=%lld us",
                      static_cast<unsigned>(snap.soft.tx_wr[sock]),
                      (snap.soft.tx_wr[sock] != r.tx_wr) ? "(MISMATCH)" : "",
                      static_cast<long long>(ago_us));
    }
    const bool buf_changed =
        snap.buf_expected &&
        (r.rxbuf_kb != snap.exp_rx_kb[sock] || r.txbuf_kb != snap.exp_tx_kb[sock]);
    const int n = std::snprintf(
        buf, len,
        "s%u mr=0x%02X cr=0x%02X ir=0x%02X sr=0x%02X tx_fsr=%u tx_rd=0x%04X "
        "tx_wr=0x%04X rx_rsr=%u%s port=%u buf=%u/%uK%s%s%s",
        static_cast<unsigned>(sock), r.mr, r.cr, r.ir, r.sr,
        static_cast<unsigned>(r.tx_fsr), static_cast<unsigned>(r.tx_rd),
        static_cast<unsigned>(r.tx_wr), static_cast<unsigned>(r.rx_rsr), sw,
        static_cast<unsigned>(r.port), static_cast<unsigned>(r.rxbuf_kb),
        static_cast<unsigned>(r.txbuf_kb), buf_changed ? "(CHANGED)" : "",
        (snap.soft.pending_mask & (1u << sock)) ? " pending" : "", dst);
    if (n < 0) return 0;
    return (static_cast<size_t>(n) < len) ? static_cast<size_t>(n) : len - 1;
}

bool destMatches(const uint8_t raw[6], uint32_t ip, uint16_t port) {
    const uint32_t got_ip = (static_cast<uint32_t>(raw[0]) << 24) |
                            (static_cast<uint32_t>(raw[1]) << 16) |
                            (static_cast<uint32_t>(raw[2]) << 8) | raw[3];
    return got_ip == ip && be16(raw + 4) == port;
}

size_t checkSocketDests(W5500Hal& hal, const SoftTxState& soft,
                        DestMismatch* out, size_t max_out) {
    size_t n = 0;
    for (uint8_t i = 0; i < 8 && n < max_out; ++i) {
        if ((soft.dest_valid & (1u << i)) == 0) continue;
        uint8_t raw[6] = {};
        {
            std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
            hal.readBuf(kBlockSocketReg(i), Sn_DIPR, raw, 6);
        }
        if (destMatches(raw, soft.dest_ip[i], soft.dest_port[i])) continue;
        DestMismatch& m = out[n++];
        m.sock = i;
        std::memcpy(m.raw, raw, 6);
        m.exp_ip = soft.dest_ip[i];
        m.exp_port = soft.dest_port[i];
    }
    return n;
}

void setExpectedBufferSizes(const uint8_t rx_kb[8], const uint8_t tx_kb[8]) {
    std::lock_guard<std::mutex> lock(diagMutex());
    std::memcpy(g_exp_rx_kb, rx_kb, 8);
    std::memcpy(g_exp_tx_kb, tx_kb, 8);
    g_exp_buf_valid = true;
}

void setExpectedIdentity(const uint8_t mac[6], const uint8_t ip[4]) {
    std::lock_guard<std::mutex> lock(diagMutex());
    std::memcpy(g_expected_mac, mac, 6);
    std::memcpy(g_expected_ip, ip, 4);
    g_expected_valid = true;
}

SpiCheckResult spiIntegrityCheck(W5500Hal& hal) {
    SpiCheckResult res;
    {
        std::lock_guard<w5500::BusMutex> lock(spiBusMutex());
        res.version = hal.readReg(kBlockCommon(), REG_VERSIONR);
        hal.readBuf(kBlockCommon(), REG_SHAR, res.mac, 6);
        hal.readBuf(kBlockCommon(), REG_SIPR, res.ip, 4);
        res.phycfgr = hal.readReg(kBlockCommon(), REG_PHYCFGR);
    }

    std::lock_guard<std::mutex> lock(diagMutex());
    ++g_stats.checks;
    if (res.version != kExpectedVersion) {
        ++g_stats.version_bad;
        res.ok = false;
    }
    if (g_expected_valid) {
        if (std::memcmp(res.mac, g_expected_mac, 6) != 0) {
            ++g_stats.mac_bad;
            res.ok = false;
        }
        if (std::memcmp(res.ip, g_expected_ip, 4) != 0) {
            ++g_stats.ip_bad;
            res.ok = false;
        }
    }
    if (!res.ok) {
        g_stats.last_bad_us = pollNowUs();
    }
    return res;
}

void noteRxHeaderBad() {
    std::lock_guard<std::mutex> lock(diagMutex());
    ++g_stats.rx_hdr_bad;
    g_stats.last_bad_us = pollNowUs();
}

SpiCheckStats spiCheckStats() {
    std::lock_guard<std::mutex> lock(diagMutex());
    return g_stats;
}

void resetSpiCheckStats() {
    std::lock_guard<std::mutex> lock(diagMutex());
    g_stats = SpiCheckStats{};
    g_snapshot_valid = false;
}

}  // namespace w5500
