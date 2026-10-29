// Host tests for W5500 dropout diagnostics: failure snapshot capture and
// formatting, and the SPI read-back integrity check.

#include "W5500.h"
#include "W5500Diag.h"
#include "W5500Hal.h"
#include "unity.h"

#include <cstring>
#include <string>

namespace {

// Flat register file: 32 blocks x 64 bytes covers common + socket registers.
class RegFileHal : public w5500::W5500Hal {
public:
    uint8_t mem[32][64] = {};

    uint8_t readReg(uint8_t block, uint16_t addr) override {
        return mem[block][addr];
    }
    void writeReg(uint8_t block, uint16_t addr, uint8_t value) override {
        mem[block][addr] = value;
    }
    uint16_t readReg16(uint8_t block, uint16_t addr) override {
        return static_cast<uint16_t>((mem[block][addr] << 8) | mem[block][addr + 1]);
    }
    void writeReg16(uint8_t block, uint16_t addr, uint16_t value) override {
        mem[block][addr] = static_cast<uint8_t>(value >> 8);
        mem[block][addr + 1] = static_cast<uint8_t>(value & 0xFF);
    }
    void readBuf(uint8_t block, uint16_t addr, uint8_t* buf, uint16_t len) override {
        std::memcpy(buf, &mem[block][addr], len);
    }
    void writeBuf(uint8_t block, uint16_t addr, const uint8_t* buf,
                  uint16_t len) override {
        std::memcpy(&mem[block][addr], buf, len);
    }
};

const uint8_t kMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t kIp[4] = {192, 168, 1, 10};

w5500::SoftTxState pendingOnly(uint8_t mask) {
    w5500::SoftTxState soft;
    soft.pending_mask = mask;
    return soft;
}

void loadHealthyChip(RegFileHal& hal) {
    hal.mem[0][REG_VERSIONR] = 0x04;
    hal.mem[0][REG_PHYCFGR] = 0x07;  // link up, 100M, full duplex
    std::memcpy(&hal.mem[0][REG_SHAR], kMac, 6);
    std::memcpy(&hal.mem[0][REG_SIPR], kIp, 4);
}

}  // namespace

void setUp() {
    w5500::resetSpiCheckStats();
}

void tearDown() {}

void test_snapshot_captures_socket_registers() {
    RegFileHal hal;
    loadHealthyChip(hal);
    const uint8_t s5 = kBlockSocketReg(5);
    hal.mem[s5][Sn_MR] = 0x02;
    hal.mem[s5][Sn_CR] = 0x00;
    hal.mem[s5][Sn_IR] = 0x00;
    hal.mem[s5][Sn_SR] = 0x22;
    hal.writeReg16(s5, Sn_TX_FSR, 0x07C0);
    hal.writeReg16(s5, Sn_TX_RD, 0x1200);
    hal.writeReg16(s5, Sn_TX_WR, 0x1240);
    const uint8_t s0 = kBlockSocketReg(0);
    hal.mem[s0][Sn_IR] = 0x02;  // DISCON
    hal.mem[s0][Sn_SR] = 0x1C;  // CLOSE_WAIT

    w5500::captureFailureSnapshot(hal, 5, w5500::FailReason::kDeferredSendOkTimeout,
                                  pendingOnly(0x64));
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT8(5, snap.sock);
    TEST_ASSERT_EQUAL_UINT8(0x04, snap.version);
    TEST_ASSERT_EQUAL_UINT8(0x07, snap.phycfgr);
    TEST_ASSERT_EQUAL_UINT8(0x22, snap.s[5].sr);
    TEST_ASSERT_EQUAL_UINT16(0x07C0, snap.s[5].tx_fsr);
    TEST_ASSERT_EQUAL_UINT16(0x1200, snap.s[5].tx_rd);
    TEST_ASSERT_EQUAL_UINT16(0x1240, snap.s[5].tx_wr);
    TEST_ASSERT_EQUAL_UINT8(0x1C, snap.s[0].sr);
    TEST_ASSERT_EQUAL_UINT8(0x02, snap.s[0].ir);

    // Consumed: a second take returns nothing.
    TEST_ASSERT_FALSE(w5500::takeFailureSnapshot(snap));
}

void test_snapshot_keeps_first_failure() {
    RegFileHal hal;
    loadHealthyChip(hal);
    w5500::captureFailureSnapshot(hal, 5, w5500::FailReason::kDeferredSendOkTimeout,
                                  pendingOnly(0));
    w5500::captureFailureSnapshot(hal, 2, w5500::FailReason::kSendOkTimeout, pendingOnly(0));
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT8(5, snap.sock);
    TEST_ASSERT_EQUAL(static_cast<int>(w5500::FailReason::kDeferredSendOkTimeout),
                      static_cast<int>(snap.reason));
}

void test_snapshot_format_lines() {
    RegFileHal hal;
    loadHealthyChip(hal);
    const uint8_t s5 = kBlockSocketReg(5);
    hal.mem[s5][Sn_SR] = 0x22;
    w5500::captureFailureSnapshot(hal, 5, w5500::FailReason::kDeferredSendOkTimeout,
                                  pendingOnly(0x20));
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));

    char line[256];
    w5500::formatSnapshotCommon(snap, line, sizeof(line));
    const std::string common(line);
    TEST_ASSERT_TRUE(common.find("sock=5 deferred SENDOK timeout") != std::string::npos);
    TEST_ASSERT_TRUE(common.find("lnk=1 100M FD") != std::string::npos);
    TEST_ASSERT_TRUE(common.find("mac=02:00:00:00:00:01") != std::string::npos);
    TEST_ASSERT_TRUE(common.find("ip=192.168.1.10") != std::string::npos);
    TEST_ASSERT_TRUE(common.find("pending=0x20") != std::string::npos);

    w5500::formatSnapshotSocket(snap, 5, line, sizeof(line));
    const std::string sock(line);
    TEST_ASSERT_TRUE(sock.find("s5 mr=0x00 cr=0x00 ir=0x00 sr=0x22") != std::string::npos);
    TEST_ASSERT_TRUE(sock.find(" pending") != std::string::npos);

    w5500::formatSnapshotSocket(snap, 4, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("pending") == std::string::npos);
}

void test_snapshot_flags_lost_tx_wr_write() {
    RegFileHal hal;
    loadHealthyChip(hal);
    const uint8_t s2 = kBlockSocketReg(2);
    // Chip says the buffer is empty (rd == wr), firmware wrote 64 bytes more.
    hal.writeReg16(s2, Sn_TX_RD, 0x0880);
    hal.writeReg16(s2, Sn_TX_WR, 0x0880);
    w5500::SoftTxState soft;
    soft.tx_wr[2] = 0x08C0;
    soft.tx_wr_valid = 0x04;
    soft.pending_mask = 0x04;
    w5500::captureFailureSnapshot(hal, 2, w5500::FailReason::kDeferredSendOkTimeout, soft);
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));

    char line[256];
    w5500::formatSnapshotSocket(snap, 2, line, sizeof(line));
    const std::string s(line);
    TEST_ASSERT_TRUE(s.find("sw_wr=0x08C0(MISMATCH)") != std::string::npos);

    // A matching cache prints no mismatch; sockets without a cache print none.
    snap.soft.tx_wr[2] = 0x0880;
    w5500::formatSnapshotSocket(snap, 2, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("MISMATCH") == std::string::npos);
    w5500::formatSnapshotSocket(snap, 3, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("sw_wr") == std::string::npos);
}

// 2026-10-20: stuck sockets read Sn_TX_FSR = 1 - pending. The snapshot now
// reads the buffer-size registers so an overwritten size shows as CHANGED.
void test_snapshot_flags_changed_buffer_size() {
    RegFileHal hal;
    loadHealthyChip(hal);
    const uint8_t rx[8] = {4, 2, 2, 2, 1, 1, 1, 1};
    const uint8_t tx[8] = {4, 2, 2, 2, 1, 1, 1, 1};
    w5500::setExpectedBufferSizes(rx, tx);
    for (uint8_t i = 0; i < 8; ++i) {
        hal.mem[kBlockSocketReg(i)][Sn_RXBUF_SIZE] = rx[i];
        hal.mem[kBlockSocketReg(i)][Sn_TXBUF_SIZE] = tx[i];
    }
    hal.mem[kBlockSocketReg(6)][Sn_TXBUF_SIZE] = 0;  // overwritten
    hal.writeReg16(kBlockSocketReg(6), Sn_PORT, 49157);
    hal.mem[0][REG_MR] = 0x00;
    hal.writeReg16(0, 0x0019, 2000);  // RTR
    hal.mem[0][0x001B] = 8;           // RCR
    w5500::captureFailureSnapshot(hal, 6, w5500::FailReason::kDeferredSendOkTimeout,
                                  pendingOnly(0x40));
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT8(0, snap.s[6].txbuf_kb);
    TEST_ASSERT_EQUAL_UINT16(2000, snap.rtr);
    TEST_ASSERT_EQUAL_UINT8(8, snap.rcr);

    char line[320];
    w5500::formatSnapshotSocket(snap, 6, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("port=49157 buf=1/0K(CHANGED)") != std::string::npos);
    w5500::formatSnapshotSocket(snap, 5, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("buf=1/1K") != std::string::npos);
    TEST_ASSERT_TRUE(std::string(line).find("CHANGED") == std::string::npos);
    w5500::formatSnapshotCommon(snap, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("rtr=2000 rcr=8") != std::string::npos);
}

// 2026-10-20 no-DMA run: drives went silent while every SEND completed. The
// destination is written once and cached, so a write landing in Sn_DIPR /
// Sn_DPORT would redirect every later frame; the snapshot and the periodic
// check compare it with the cache.
void test_snapshot_flags_changed_destination() {
    RegFileHal hal;
    loadHealthyChip(hal);
    const uint8_t s5 = kBlockSocketReg(5);
    const uint8_t s6 = kBlockSocketReg(6);
    const uint8_t good[6] = {192, 168, 1, 21, 0x08, 0xAE};  // :2222
    std::memcpy(&hal.mem[s5][Sn_DIPR], good, 6);
    const uint8_t bad[6] = {192, 168, 0xB2, 0x00, 0x08, 0xAE};
    std::memcpy(&hal.mem[s6][Sn_DIPR], bad, 6);
    const uint8_t dmac[6] = {0x00, 0x1D, 0x9C, 0x01, 0x02, 0x03};
    std::memcpy(&hal.mem[s5][0x06], dmac, 6);

    w5500::SoftTxState soft;
    soft.dest_ip[5] = 0xC0A80115;  // 192.168.1.21
    soft.dest_port[5] = 2222;
    soft.dest_ip[6] = 0xC0A80117;  // 192.168.1.23
    soft.dest_port[6] = 2222;
    soft.dest_valid = 0x60;

    TEST_ASSERT_TRUE(w5500::destMatches(good, 0xC0A80115, 2222));
    TEST_ASSERT_FALSE(w5500::destMatches(good, 0xC0A80115, 2223));
    TEST_ASSERT_FALSE(w5500::destMatches(bad, 0xC0A80117, 2222));

    w5500::DestMismatch m[8];
    TEST_ASSERT_EQUAL_UINT32(1, w5500::checkSocketDests(hal, soft, m, 8));
    TEST_ASSERT_EQUAL_UINT8(6, m[0].sock);
    TEST_ASSERT_EQUAL_UINT8(0xB2, m[0].raw[2]);
    TEST_ASSERT_EQUAL_UINT32(0xC0A80117, m[0].exp_ip);

    w5500::captureFailureSnapshot(hal, 0xFF, w5500::FailReason::kInputStale, soft);
    w5500::FailureSnapshot snap;
    TEST_ASSERT_TRUE(w5500::takeFailureSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT16(2222, snap.s[5].dport);
    char line[320];
    w5500::formatSnapshotSocket(snap, 5, line, sizeof(line));
    std::string l5(line);
    TEST_ASSERT_TRUE(l5.find("dst=192.168.1.21:2222 dmac=00:1D:9C:01:02:03") != std::string::npos);
    TEST_ASSERT_TRUE(l5.find("DEST CHANGED") == std::string::npos);
    w5500::formatSnapshotSocket(snap, 6, line, sizeof(line));
    TEST_ASSERT_TRUE(std::string(line).find("dst=192.168.178.0:2222") != std::string::npos);
    TEST_ASSERT_TRUE(std::string(line).find("(DEST CHANGED)") != std::string::npos);
}

void test_format_phy() {
    char buf[24];
    w5500::formatPhy(0xBF, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("lnk=1 100M FD", buf);
    w5500::formatPhy(0xB9, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("lnk=1 10M HD", buf);
}

void test_integrity_check_reports_phy() {
    RegFileHal hal;
    loadHealthyChip(hal);
    TEST_ASSERT_EQUAL_UINT8(0x07, w5500::spiIntegrityCheck(hal).phycfgr);
}

void test_format_truncates_safely() {
    w5500::FailureSnapshot snap;
    char tiny[16];
    const size_t n = w5500::formatSnapshotCommon(snap, tiny, sizeof(tiny));
    TEST_ASSERT_EQUAL_UINT32(sizeof(tiny) - 1, static_cast<uint32_t>(n));
    TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(tiny[sizeof(tiny) - 1]));
}

void test_integrity_check_passes_on_healthy_chip() {
    RegFileHal hal;
    loadHealthyChip(hal);
    w5500::setExpectedIdentity(kMac, kIp);
    const w5500::SpiCheckResult res = w5500::spiIntegrityCheck(hal);
    TEST_ASSERT_TRUE(res.ok);
    const w5500::SpiCheckStats st = w5500::spiCheckStats();
    TEST_ASSERT_EQUAL_UINT32(1, st.checks);
    TEST_ASSERT_EQUAL_UINT32(0, st.version_bad + st.mac_bad + st.ip_bad);
}

void test_integrity_check_counts_each_mismatch() {
    RegFileHal hal;
    loadHealthyChip(hal);
    w5500::setExpectedIdentity(kMac, kIp);

    hal.mem[0][REG_SIPR + 3] = 11;  // one corrupted IP byte
    w5500::SpiCheckResult res = w5500::spiIntegrityCheck(hal);
    TEST_ASSERT_FALSE(res.ok);
    TEST_ASSERT_EQUAL_UINT8(11, res.ip[3]);

    hal.mem[0][REG_SIPR + 3] = 10;
    hal.mem[0][REG_VERSIONR] = 0xFF;  // MISO stuck high
    res = w5500::spiIntegrityCheck(hal);
    TEST_ASSERT_FALSE(res.ok);

    const w5500::SpiCheckStats st = w5500::spiCheckStats();
    TEST_ASSERT_EQUAL_UINT32(2, st.checks);
    TEST_ASSERT_EQUAL_UINT32(1, st.ip_bad);
    TEST_ASSERT_EQUAL_UINT32(1, st.version_bad);
    TEST_ASSERT_EQUAL_UINT32(0, st.mac_bad);
}

void test_rx_header_counter() {
    w5500::noteRxHeaderBad();
    w5500::noteRxHeaderBad();
    TEST_ASSERT_EQUAL_UINT32(2, w5500::spiCheckStats().rx_hdr_bad);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_snapshot_captures_socket_registers);
    RUN_TEST(test_snapshot_keeps_first_failure);
    RUN_TEST(test_snapshot_format_lines);
    RUN_TEST(test_snapshot_flags_lost_tx_wr_write);
    RUN_TEST(test_snapshot_flags_changed_buffer_size);
    RUN_TEST(test_snapshot_flags_changed_destination);
    RUN_TEST(test_format_phy);
    RUN_TEST(test_integrity_check_reports_phy);
    RUN_TEST(test_format_truncates_safely);
    RUN_TEST(test_integrity_check_passes_on_healthy_chip);
    RUN_TEST(test_integrity_check_counts_each_mismatch);
    RUN_TEST(test_rx_header_counter);
    return UNITY_END();
}
