// Host tests for Class 1 UDP send fast-path (dest cache + transaction diet)
// and non-blocking UDP recv (header strip / drain).

#include "W5500.h"
#include "W5500Hal.h"
#include "W5500Socket.h"
#include "unity.h"

#include <cstring>
#include <vector>

namespace {

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

  std::vector<Txn> txns;
  uint8_t last_dest[6] = {};
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
      if (poll_stuck) {
        buf[0] = 0;
        buf[1] = 0;
      } else {
        buf[0] = 0;
        buf[1] = poll_ir;
        sn_ir = poll_ir;
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
};

}  // namespace

void setUp(void) {}
void tearDown(void) {}

static void test_dipr_dport_contiguous(void) {
  TEST_ASSERT_EQUAL_UINT16(0x000C, Sn_DIPR);
  TEST_ASSERT_EQUAL_UINT16(0x0010, Sn_DPORT);
  TEST_ASSERT_EQUAL_UINT16(Sn_DIPR + 4, Sn_DPORT);
}

static void test_sendto_caches_dest(void) {
  CountingHal hal;
  const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const uint32_t ip = 0xC0A80114;  // 192.168.1.20
  const uint16_t port = 2222;

  w5500::socketClose(hal, 0);  // clear any leftover dest cache
  hal.txns.clear();

  const int n1 =
      w5500::socketSendTo(hal, 0, payload, sizeof(payload), ip, port);
  TEST_ASSERT_EQUAL_INT(8, n1);
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countDestWrites()));
  const size_t first_txns = hal.txns.size();
  TEST_ASSERT_TRUE(first_txns <= 12);  // diet target ~6 plus clear IR

  hal.txns.clear();
  const int n2 =
      w5500::socketSendTo(hal, 0, payload, sizeof(payload), ip, port);
  TEST_ASSERT_EQUAL_INT(8, n2);
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countDestWrites()));
  TEST_ASSERT_TRUE(hal.txns.size() < first_txns);
}

static void test_sendto_rewrites_on_dest_change(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {9, 9, 9, 9};
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80114, 2222));
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80115, 2222));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countDestWrites()));
}

static void test_sendto_restores_multicast_dest(void) {
  CountingHal hal;
  const uint32_t mcast = 0xE000012A;  // 224.0.1.42
  const uint16_t local_port = 2222;
  const int sock =
      w5500::socketOpen(hal, w5500::SocketMode::kUdp, local_port, mcast);
  TEST_ASSERT_EQUAL_INT(0, sock);

  const uint8_t payload[4] = {1, 2, 3, 4};
  const uint32_t axis_ip = 0xC0A80114;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, axis_ip, local_port));

  // After SENDOK, DIPR/DPORT must be restored to the multicast listen group.
  TEST_ASSERT_EQUAL_HEX8(0xE0, hal.last_dest[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, hal.last_dest[1]);
  TEST_ASSERT_EQUAL_HEX8(0x01, hal.last_dest[2]);
  TEST_ASSERT_EQUAL_HEX8(0x2A, hal.last_dest[3]);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>((local_port >> 8) & 0xFF),
                         hal.last_dest[4]);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(local_port & 0xFF),
                         hal.last_dest[5]);

  w5500::socketClose(hal, 0);
}

static void test_sendto_sendok_timeout_returns_error(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  hal.poll_stuck = true;
  hal.sn_sr_reads = 0;
  const uint8_t payload[2] = {0xAA, 0xBB};
  const int n =
      w5500::socketSendTo(hal, 0, payload, 2, 0xC0A80114, 2222);
  TEST_ASSERT_EQUAL_INT(-1, n);
  // Sn_SR is diagnostics-only on the failure path.
  TEST_ASSERT_TRUE(hal.sn_sr_reads >= 1);
}

static void test_sendto_ir_timeout_and_discon(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[2] = {1, 2};

  hal.poll_ir = Sn_IR_TIMEOUT;
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal, 0, payload, 2, 0xC0A80114, 2222));

  w5500::socketClose(hal, 0);
  CountingHal hal2;
  w5500::socketClose(hal2, 0);
  hal2.poll_ir = Sn_IR_DISCON;
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal2, 0, payload, 2, 0xC0A80115, 2222));
}

static void test_sendto_rejects_bad_length(void) {
  CountingHal hal;
  const uint8_t payload[4] = {1, 2, 3, 4};
  const size_t before = hal.txns.size();
  TEST_ASSERT_EQUAL_INT(0, w5500::socketSendTo(hal, 0, payload, 0, 0x1, 1));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(before),
                           static_cast<uint32_t>(hal.txns.size()));

  std::vector<uint8_t> big(0x10000, 0x5A);
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal, 0, big.data(), big.size(), 0x1, 1));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(before),
                           static_cast<uint32_t>(hal.txns.size()));
}

static void test_socket_close_clears_dest_cache(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {9, 9, 9, 9};
  const uint32_t ip = 0xC0A80114;
  TEST_ASSERT_EQUAL_INT(4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222));
  hal.txns.clear();
  // Cache hit — no DIPR rewrite.
  TEST_ASSERT_EQUAL_INT(4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countDestWrites()));

  w5500::socketClose(hal, 0);
  TEST_ASSERT_EQUAL_UINT8(SOCK_CLOSED, hal.sn_sr[0]);

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countDestWrites()));
}

static void test_recv_nonblocking_empty_returns_zero(void) {
  CountingHal hal;
  uint8_t buf[16];
  hal.recv_cmd_writes = 0;
  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.recv_cmd_writes));
}

static void test_recv_nonblocking_strips_header(void) {
  CountingHal hal;
  const uint8_t payload[] = {0x10, 0x20, 0x30, 0x40};
  hal.enqueueUdpDatagram(payload, sizeof(payload), 0xC0A80114, 2222);
  const uint16_t rd0 = hal.rx_rd;

  uint8_t out[16] = {};
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out, sizeof(payload));
  TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(rd0 + 8 + sizeof(payload)),
                           hal.rx_rd);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
  TEST_ASSERT_TRUE(hal.recv_cmd_writes >= 1);
}

static void test_recv_nonblocking_truncates_but_advances(void) {
  CountingHal hal;
  const uint8_t payload[] = {1, 2, 3, 4, 5, 6, 7, 8};
  hal.enqueueUdpDatagram(payload, sizeof(payload));
  const uint16_t rd0 = hal.rx_rd;

  uint8_t out[3] = {};
  TEST_ASSERT_EQUAL_INT(3, w5500::socketRecvFromNonBlocking(hal, 0, out, 3));
  TEST_ASSERT_EQUAL_HEX8(1, out[0]);
  TEST_ASSERT_EQUAL_HEX8(2, out[1]);
  TEST_ASSERT_EQUAL_HEX8(3, out[2]);
  // Full packet consumed so the next read stays aligned.
  TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(rd0 + 8 + sizeof(payload)),
                           hal.rx_rd);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
}

static void test_recv_nonblocking_bad_header_drains(void) {
  CountingHal hal;
  // Declared payload length 0 — drain entire RSR.
  const uint8_t bad0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  hal.enqueueRawRx(bad0, 8);
  uint8_t out[16];
  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());

  // Declared length beyond available bytes.
  CountingHal hal2;
  const uint8_t bad1[8] = {0, 0, 0, 0, 0, 0, 0x00, 0x20};  // len=32, avail=8
  hal2.enqueueRawRx(bad1, 8);
  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal2, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT16(0, hal2.rxRsr());
}

static void test_recv_nonblocking_back_to_back(void) {
  CountingHal hal;
  const uint8_t a[] = {0xA1, 0xA2};
  const uint8_t b[] = {0xB1, 0xB2, 0xB3};
  hal.enqueueUdpDatagram(a, sizeof(a));
  hal.enqueueUdpDatagram(b, sizeof(b));

  uint8_t out[8] = {};
  TEST_ASSERT_EQUAL_INT(2, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8(0xA1, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0xA2, out[1]);

  TEST_ASSERT_EQUAL_INT(3, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8(0xB1, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB2, out[1]);
  TEST_ASSERT_EQUAL_HEX8(0xB3, out[2]);

  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_dipr_dport_contiguous);
  RUN_TEST(test_sendto_caches_dest);
  RUN_TEST(test_sendto_rewrites_on_dest_change);
  RUN_TEST(test_sendto_restores_multicast_dest);
  RUN_TEST(test_sendto_sendok_timeout_returns_error);
  RUN_TEST(test_sendto_ir_timeout_and_discon);
  RUN_TEST(test_sendto_rejects_bad_length);
  RUN_TEST(test_socket_close_clears_dest_cache);
  RUN_TEST(test_recv_nonblocking_empty_returns_zero);
  RUN_TEST(test_recv_nonblocking_strips_header);
  RUN_TEST(test_recv_nonblocking_truncates_but_advances);
  RUN_TEST(test_recv_nonblocking_bad_header_drains);
  RUN_TEST(test_recv_nonblocking_back_to_back);
  return UNITY_END();
}
