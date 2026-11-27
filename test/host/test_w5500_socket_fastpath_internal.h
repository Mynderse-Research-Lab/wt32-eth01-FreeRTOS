/**
 * @file test_w5500_socket_fastpath_internal.h
 * @brief Counting SPI stand-in for the W5500 socket fast-path host tests.
 */

#pragma once

#include "W5500.h"
#include "W5500Hal.h"
#include "W5500BusMutex.h"
#include "W5500Socket.h"
#include "unity.h"

#include <chrono>
#include <cstring>
#include <vector>

namespace w5500_fastpath_test {

struct Txn {
  enum class Kind { kReg, kReg16, kBuf };
  Kind kind;
  uint8_t block = 0;
  uint16_t addr = 0;
  uint16_t len = 0;
  bool is_write = false;
};

class CountingHal : public w5500::W5500Hal {
 public:
  uint8_t sn_sr[8] = {};  // SOCK_CLOSED — findFreeSocket / close poll
  uint8_t sn_mr[8] = {};
  uint8_t sn_cr = 0;
  uint8_t sn_ir = 0;
  uint16_t tx_wr = 0x100;
  uint16_t tx_fsr = 0xFFFF;
  uint16_t rx_rd = 0;
  uint16_t rx_wr = 0;
  std::vector<uint8_t> rx_mem;

  // Merged CR+IR poll (Sn_CR burst of 2). Default: CR cleared + SENDOK.
  uint8_t poll_ir = Sn_IR_SENDOK;
  bool poll_stuck = false;  // never report done (SENDOK timeout path)
  size_t poll_pending_reads = 0;  // report "not done" for this many polls

  std::vector<Txn> txns;
  uint8_t last_dest[6] = {};
  uint8_t last_cr_written = 0;
  size_t sn_sr_reads = 0;
  size_t recv_cmd_writes = 0;

  uint16_t rxRsr() const {
    return static_cast<uint16_t>(rx_wr - rx_rd);
  }

  void enqueueUdpDatagram(const uint8_t* payload, uint16_t payload_len,
                          uint32_t src_ip = 0, uint16_t src_port = 0) {
    const uint16_t at = rx_wr;
    const size_t need =
        static_cast<size_t>(static_cast<uint16_t>(at + 8u + payload_len)) + 1u;
    if (rx_mem.size() < need) {
      rx_mem.resize(need, 0);
    }
    rx_mem[at + 0] = static_cast<uint8_t>((src_ip >> 24) & 0xFF);
    rx_mem[at + 1] = static_cast<uint8_t>((src_ip >> 16) & 0xFF);
    rx_mem[at + 2] = static_cast<uint8_t>((src_ip >> 8) & 0xFF);
    rx_mem[at + 3] = static_cast<uint8_t>(src_ip & 0xFF);
    rx_mem[at + 4] = static_cast<uint8_t>((src_port >> 8) & 0xFF);
    rx_mem[at + 5] = static_cast<uint8_t>(src_port & 0xFF);
    rx_mem[at + 6] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
    rx_mem[at + 7] = static_cast<uint8_t>(payload_len & 0xFF);
    if (payload_len > 0 && payload != nullptr) {
      std::memcpy(rx_mem.data() + at + 8, payload, payload_len);
    }
    rx_wr = static_cast<uint16_t>(rx_wr + 8u + payload_len);
  }

  // Bad header: declared length that fails packetBytes > avail or is zero.
  void enqueueRawRx(const uint8_t* bytes, uint16_t len) {
    const uint16_t at = rx_wr;
    const size_t need = static_cast<size_t>(at) + len;
    if (rx_mem.size() < need) {
      rx_mem.resize(need, 0);
    }
    std::memcpy(rx_mem.data() + at, bytes, len);
    rx_wr = static_cast<uint16_t>(rx_wr + len);
  }

  uint8_t sockFromBlock(uint8_t block) const {
    if (block >= 1 && ((block - 1) % 4) == 0) {
      return static_cast<uint8_t>((block - 1) / 4);
    }
    return 0xFF;
  }

  bool isSocketRegBlock(uint8_t block) const {
    return sockFromBlock(block) < 8;
  }

  bool isRxBufBlock(uint8_t block) const {
    return block >= 3 && ((block - 3) % 4) == 0;
  }

  uint8_t readReg(uint8_t block, uint16_t addr) override {
    txns.push_back({Txn::Kind::kReg, block, addr, 1, false});
    const uint8_t sock = sockFromBlock(block);
    if (sock < 8) {
      if (addr == Sn_CR) return sn_cr;
      if (addr == Sn_IR) return sn_ir;
      if (addr == Sn_SR) {
        ++sn_sr_reads;
        return sn_sr[sock];
      }
      if (addr == Sn_MR) return sn_mr[sock];
    }
    return 0;
  }

  void writeReg(uint8_t block, uint16_t addr, uint8_t value) override {
    txns.push_back({Txn::Kind::kReg, block, addr, 1, true});
    const uint8_t sock = sockFromBlock(block);
    if (sock >= 8) return;

    if (addr == Sn_MR) {
      sn_mr[sock] = value;
      return;
    }
    if (addr == Sn_IR) {
      sn_ir = static_cast<uint8_t>(sn_ir & ~value);
      return;
    }
    if (addr != Sn_CR) return;

    last_cr_written = value;
    if (value == Sn_CR_OPEN) {
      if ((sn_mr[sock] & 0x0F) == Sn_MR_UDP) {
        sn_sr[sock] = SOCK_UDP;
      } else if ((sn_mr[sock] & 0x0F) == Sn_MR_TCP) {
        sn_sr[sock] = SOCK_INIT;
      }
    } else if (value == Sn_CR_CLOSE) {
      sn_sr[sock] = SOCK_CLOSED;
    } else if (value == Sn_CR_RECV) {
      ++recv_cmd_writes;
    }
    sn_cr = 0;
  }

  uint16_t readReg16(uint8_t block, uint16_t addr) override {
    txns.push_back({Txn::Kind::kReg16, block, addr, 2, false});
    if (!isSocketRegBlock(block)) return 0;
    if (addr == Sn_TX_WR) return tx_wr;
    if (addr == Sn_TX_FSR) return tx_fsr;
    if (addr == Sn_RX_RSR) return rxRsr();
    if (addr == Sn_RX_RD) return rx_rd;
    return 0;
  }

  void writeReg16(uint8_t block, uint16_t addr, uint16_t value) override {
    txns.push_back({Txn::Kind::kReg16, block, addr, 2, true});
    if (!isSocketRegBlock(block)) return;
    if (addr == Sn_TX_WR) {
      tx_wr = value;
    } else if (addr == Sn_RX_RD) {
      rx_rd = value;
    }
  }

  void readBuf(uint8_t block, uint16_t addr, uint8_t* buf,
               uint16_t len) override {
    txns.push_back({Txn::Kind::kBuf, block, addr, len, false});
    std::memset(buf, 0, len);

    if (isSocketRegBlock(block) && addr == Sn_CR && len >= 2) {
      if (poll_stuck || poll_pending_reads > 0) {
        if (poll_pending_reads > 0) --poll_pending_reads;
        buf[0] = 0;
        buf[1] = 0;
      } else {
        buf[0] = 0;
        buf[1] = poll_ir;
        sn_ir = poll_ir;
      }
      return;
    }

    if (isSocketRegBlock(block) && addr == Sn_RX_RSR && len >= 2) {
      const uint16_t rsr = rxRsr();
      buf[0] = static_cast<uint8_t>(rsr >> 8);
      buf[1] = static_cast<uint8_t>(rsr & 0xFF);
      if (len >= 4) {
        buf[2] = static_cast<uint8_t>(rx_rd >> 8);
        buf[3] = static_cast<uint8_t>(rx_rd & 0xFF);
      }
      return;
    }

    if (isRxBufBlock(block)) {
      for (uint16_t i = 0; i < len; ++i) {
        const uint16_t a = static_cast<uint16_t>(addr + i);
        buf[i] = (a < rx_mem.size()) ? rx_mem[a] : 0;
      }
    }
  }

  void writeBuf(uint8_t block, uint16_t addr, const uint8_t* data,
                uint16_t len) override {
    txns.push_back({Txn::Kind::kBuf, block, addr, len, true});
    if (isSocketRegBlock(block) && addr == Sn_DIPR && len >= 4 && data != nullptr) {
      const size_t n = (len > 6) ? 6u : static_cast<size_t>(len);
      std::memcpy(last_dest, data, n);
    }
  }

  size_t countDestWrites() const {
    size_t n = 0;
    const uint8_t blk = kBlockSocketReg(0);
    for (const Txn& t : txns) {
      if (t.is_write && t.kind == Txn::Kind::kBuf && t.block == blk &&
          t.addr == Sn_DIPR && t.len == 6) {
        ++n;
      }
    }
    return n;
  }

  size_t countTxnWrites() const {
    size_t n = 0;
    for (const Txn& t : txns) {
      if (t.is_write) ++n;
    }
    return n;
  }

  size_t countRxBufReads() const {
    size_t n = 0;
    for (const Txn& t : txns) {
      if (!t.is_write && t.kind == Txn::Kind::kBuf && isRxBufBlock(t.block)) {
        ++n;
      }
    }
    return n;
  }

  size_t countRsrRdBursts() const {
    size_t n = 0;
    const uint8_t blk = kBlockSocketReg(0);
    for (const Txn& t : txns) {
      if (!t.is_write && t.kind == Txn::Kind::kBuf && t.block == blk &&
          t.addr == Sn_RX_RSR && t.len == 4) {
        ++n;
      }
    }
    return n;
  }

  // Merged Sn_CR + Sn_IR SENDOK poll bursts.
  size_t countCrIrPolls() const {
    size_t n = 0;
    const uint8_t blk = kBlockSocketReg(0);
    for (const Txn& t : txns) {
      if (!t.is_write && t.kind == Txn::Kind::kBuf && t.block == blk &&
          t.addr == Sn_CR && t.len == 2) {
        ++n;
      }
    }
    return n;
  }

  size_t countTxWrReads() const {
    size_t n = 0;
    const uint8_t blk = kBlockSocketReg(0);
    for (const Txn& t : txns) {
      if (!t.is_write && t.kind == Txn::Kind::kReg16 && t.block == blk &&
          t.addr == Sn_TX_WR) {
        ++n;
      }
    }
    return n;
  }

  size_t countIrReads() const {
    size_t n = 0;
    const uint8_t blk = kBlockSocketReg(0);
    for (const Txn& t : txns) {
      if (!t.is_write && t.kind == Txn::Kind::kReg && t.block == blk &&
          t.addr == Sn_IR) {
        ++n;
      }
    }
    return n;
  }

  bool lastTxnIsCommandWrite() const {
    if (txns.empty()) return false;
    const Txn& t = txns.back();
    return t.is_write && t.kind == Txn::Kind::kReg && t.addr == Sn_CR;
  }
};

}  // namespace

