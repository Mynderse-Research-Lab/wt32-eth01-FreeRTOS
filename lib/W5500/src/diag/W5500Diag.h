// W5500 fault diagnostics for the Class 1 dropout investigation.
//
// Read-only: nothing here changes socket state or recovers anything.
//  - Failure snapshot: the full chip register state captured at the first
//    failed Class 1 send, before the scanner tears the link down.
//  - SPI integrity check: periodic read-back of registers whose value is
//    known (VERSIONR, SHAR, SIPR). A mismatch means a corrupted SPI read.

#ifndef W5500_DIAG_H
#define W5500_DIAG_H

#include <cstddef>
#include <cstdint>

#include "W5500Hal.h"

namespace w5500 {

enum class FailReason : uint8_t {
    kDeferredSendOkTimeout = 1,  // previous SEND never completed
    kDeferredSendFailed    = 2,  // previous SEND reported TIMEOUT/DISCON
    kSendCmdTimeout        = 3,  // Sn_CR never cleared after SEND
    kSendOkTimeout         = 4,  // blocking SENDOK wait expired
    kInputStale            = 5,  // T->O feedback stale (no send failed)
    kLinkDown              = 6,  // PHYCFGR LNK went to 0
};

struct SocketRegs {
    uint8_t mr = 0;
    uint8_t cr = 0;
    uint8_t ir = 0;
    uint8_t sr = 0;
    uint16_t tx_fsr = 0;
    uint16_t tx_rd = 0;
    uint16_t tx_wr = 0;
    uint16_t rx_rsr = 0;
    uint16_t port = 0;      // Sn_PORT
    uint8_t rxbuf_kb = 0;   // Sn_RXBUF_SIZE
    uint8_t txbuf_kb = 0;   // Sn_TXBUF_SIZE
    uint8_t dhar[6] = {};   // Sn_DHAR (0x06..0x0B)
    uint8_t dipr[4] = {};   // Sn_DIPR (0x0C..0x0F)
    uint16_t dport = 0;     // Sn_DPORT (0x10..0x11)
};

// Host-side (software) view of the Class 1 TX path, owned by W5500Socket.
// Lets the snapshot compare the chip's Sn_TX_WR with what the firmware wrote.
struct SoftTxState {
    uint16_t tx_wr[8] = {};       // cached Sn_TX_WR (last value written)
    uint8_t tx_wr_valid = 0;      // bit per socket: cache valid
    uint8_t pending_mask = 0;     // bit per socket: deferred SENDOK owed
    int64_t last_send_us[8] = {}; // time SEND was issued (0 = never)
    // UDP destination the firmware wrote (written once, then cached): the
    // chip's Sn_DIPR/Sn_DPORT must still hold it.
    uint32_t dest_ip[8] = {};
    uint16_t dest_port[8] = {};
    uint8_t dest_valid = 0;       // bit per socket
};

struct FailureSnapshot {
    int64_t t_us = 0;
    uint8_t sock = 0;
    FailReason reason = FailReason::kDeferredSendOkTimeout;
    uint8_t version = 0;
    uint8_t phycfgr = 0;
    uint8_t ir = 0;
    uint8_t sir = 0;
    uint8_t mac[6] = {};
    uint8_t ip[4] = {};
    // Configuration registers: if a write was misdirected, these change.
    uint8_t mr = 0;         // common Mode Register
    uint8_t imr = 0;
    uint8_t simr = 0;
    uint16_t rtr = 0;       // retry time (100 us units)
    uint8_t rcr = 0;        // retry count
    uint8_t buf_expected = 0;  // 1 when expected buffer sizes are known
    uint8_t exp_rx_kb[8] = {};
    uint8_t exp_tx_kb[8] = {};
    SoftTxState soft;
    SocketRegs s[8];
};

// Capture the chip state. Caller holds spiBusMutex(). Only the first failure
// is kept until takeFailureSnapshot() consumes it.
void captureFailureSnapshot(W5500Hal& hal, uint8_t sock, FailReason reason,
                            const SoftTxState& soft);
bool takeFailureSnapshot(FailureSnapshot& out);

const char* failReasonName(FailReason reason);

// PHYCFGR as "lnk=1 100M FD" (pure).
size_t formatPhy(uint8_t phycfgr, char* buf, size_t len);

// One-line renderings for the log (pure; host-tested).
size_t formatSnapshotCommon(const FailureSnapshot& snap, char* buf, size_t len);
size_t formatSnapshotSocket(const FailureSnapshot& snap, uint8_t sock,
                            char* buf, size_t len);

// Sn_DIPR[4] + Sn_DPORT[2] as read from the chip vs the cached destination (pure).
bool destMatches(const uint8_t raw[6], uint32_t ip, uint16_t port);

// A Class 1 socket whose Sn_DIPR/Sn_DPORT no longer holds the cached value.
struct DestMismatch {
    uint8_t sock = 0xFF;
    uint8_t raw[6] = {};    // DIPR[4], DPORT[2] as read
    uint32_t exp_ip = 0;
    uint16_t exp_port = 0;
};

// Read Sn_DIPR/Sn_DPORT of every socket with a cached UDP destination and
// return the mismatches (up to max_out). Takes spiBusMutex(). Read-only.
size_t checkSocketDests(W5500Hal& hal, const SoftTxState& soft,
                        DestMismatch* out, size_t max_out);

struct SpiCheckStats {
    uint32_t checks = 0;
    uint32_t version_bad = 0;
    uint32_t mac_bad = 0;
    uint32_t ip_bad = 0;
    uint32_t rx_hdr_bad = 0;
    int64_t last_bad_us = 0;
};

struct SpiCheckResult {
    bool ok = true;
    uint8_t version = 0;
    uint8_t mac[6] = {};
    uint8_t ip[4] = {};
    uint8_t phycfgr = 0;  // read with the check; caller tracks link-mode changes
};

// Socket buffer sizes written at init/recover; the snapshot flags a socket
// whose Sn_RXBUF_SIZE / Sn_TXBUF_SIZE no longer matches.
void setExpectedBufferSizes(const uint8_t rx_kb[8], const uint8_t tx_kb[8]);

// Values written at init/recover; the check compares SHAR/SIPR against them.
void setExpectedIdentity(const uint8_t mac[6], const uint8_t ip[4]);

// Read VERSIONR, SHAR, SIPR (and PHYCFGR) and compare. Takes spiBusMutex().
SpiCheckResult spiIntegrityCheck(W5500Hal& hal);

// Count a UDP RX header that failed validation (corrupt read or bad queue).
void noteRxHeaderBad();

SpiCheckStats spiCheckStats();
void resetSpiCheckStats();

}  // namespace w5500

#endif  // W5500_DIAG_H
