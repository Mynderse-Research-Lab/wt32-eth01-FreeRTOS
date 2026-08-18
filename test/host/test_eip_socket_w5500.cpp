// Host tests for EipSocketW5500 TCP/UDP over a mock W5500 HAL.

#include "EipIoConnection.h"
#include "EipSocketW5500.h"
#include "W5500.h"
#include "W5500Hal.h"
#include "unity.h"

#include <cstring>
#include <vector>

namespace {

class CountingHal : public w5500::W5500Hal {
 public:
  uint8_t sn_sr[8] = {};
  uint8_t sn_mr[8] = {};
  uint8_t sn_cr = 0;
  uint8_t sn_ir = 0;
  uint16_t tx_wr = 0x100;
  uint16_t tx_fsr = 0xFFFF;
  uint16_t rx_rd = 0;
  uint16_t rx_wr = 0;
  uint8_t poll_ir = Sn_IR_SENDOK;
  bool connect_fail = false;
  size_t open_cmds = 0;
  size_t close_cmds = 0;
  std::vector<uint8_t> rx_mem;

  uint16_t rxRsr() const { return static_cast<uint16_t>(rx_wr - rx_rd); }

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

  uint8_t sockFromBlock(uint8_t block) const {
    if (block >= 1 && ((block - 1) % 4) == 0) {
      return static_cast<uint8_t>((block - 1) / 4);
    }
    return 0xFF;
  }

  bool isSocketRegBlock(uint8_t block) const { return sockFromBlock(block) < 8; }

  bool isRxBufBlock(uint8_t block) const {
    return block >= 3 && ((block - 3) % 4) == 0;
  }

  size_t countUdp() const {
    size_t n = 0;
    for (uint8_t s = 0; s < 8; ++s) {
      if (sn_sr[s] == SOCK_UDP) {
        ++n;
      }
    }
    return n;
  }

  size_t countNotClosed() const {
    size_t n = 0;
    for (uint8_t s = 0; s < 8; ++s) {
      if (sn_sr[s] != SOCK_CLOSED) {
        ++n;
      }
    }
    return n;
  }

  uint8_t readReg(uint8_t block, uint16_t addr) override {
    const uint8_t sock = sockFromBlock(block);
    if (sock < 8) {
      if (addr == Sn_CR) return sn_cr;
      if (addr == Sn_IR) return sn_ir;
      if (addr == Sn_SR) return sn_sr[sock];
      if (addr == Sn_MR) return sn_mr[sock];
    }
    return 0;
  }

  void writeReg(uint8_t block, uint16_t addr, uint8_t value) override {
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
      ++open_cmds;
      if ((sn_mr[sock] & 0x0F) == Sn_MR_UDP) {
        sn_sr[sock] = SOCK_UDP;
      } else if ((sn_mr[sock] & 0x0F) == Sn_MR_TCP) {
        sn_sr[sock] = SOCK_INIT;
      }
    } else if (value == Sn_CR_CLOSE) {
      ++close_cmds;
      sn_sr[sock] = SOCK_CLOSED;
    } else if (value == Sn_CR_CONNECT) {
      sn_sr[sock] = connect_fail ? SOCK_CLOSED : SOCK_ESTABLISHED;
    }
    sn_cr = 0;
  }

  uint16_t readReg16(uint8_t block, uint16_t addr) override {
    if (!isSocketRegBlock(block)) return 0;
    if (addr == Sn_TX_WR) return tx_wr;
    if (addr == Sn_TX_FSR) return tx_fsr;
    if (addr == Sn_RX_RSR) return rxRsr();
    if (addr == Sn_RX_RD) return rx_rd;
    return 0;
  }

  void writeReg16(uint8_t block, uint16_t addr, uint16_t value) override {
    if (!isSocketRegBlock(block)) return;
    if (addr == Sn_TX_WR) {
      tx_wr = value;
    } else if (addr == Sn_RX_RD) {
      rx_rd = value;
    }
  }

  void readBuf(uint8_t block, uint16_t addr, uint8_t* buf,
               uint16_t len) override {
    std::memset(buf, 0, len);
    if (isSocketRegBlock(block) && addr == Sn_CR && len >= 2) {
      buf[0] = 0;
      buf[1] = poll_ir;
      sn_ir = poll_ir;
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

  void writeBuf(uint8_t, uint16_t, const uint8_t*, uint16_t) override {}
};

}  // namespace

void setUp(void) {}
void tearDown(void) {}

static void test_udp_p2p_rx_then_tx_per_dest(void) {
  CountingHal hal;
  eip::EipSocketW5500Udp udp(hal);
  TEST_ASSERT_TRUE(udp.bind(eip::EipIoConnection::kDefaultUdpPort, 0));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.open_cmds));

  const uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL_INT(
      4, udp.sendTo(payload, sizeof(payload), 0xC0A80114u, 2222));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_INT(
      4, udp.sendTo(payload, sizeof(payload), 0xC0A80114u, 2222));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_INT(
      4, udp.sendTo(payload, sizeof(payload), 0xC0A80115u, 2222));
  TEST_ASSERT_EQUAL_UINT32(3, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_INT(
      4, udp.sendTo(payload, sizeof(payload), 0xC0A80117u, 2222));
  TEST_ASSERT_EQUAL_UINT32(4, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_INT(
      -1, udp.sendTo(payload, sizeof(payload), 0xC0A80118u, 2222));
  TEST_ASSERT_EQUAL_UINT32(4, static_cast<uint32_t>(hal.countUdp()));

  udp.close();
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countUdp()));
}

static void test_udp_multicast_split_bind(void) {
  CountingHal hal;
  eip::EipSocketW5500Udp udp(hal);
  TEST_ASSERT_TRUE(udp.bind(eip::EipIoConnection::kDefaultUdpPort, 0x20000003u));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.countUdp()));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.open_cmds));
  udp.close();
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countUdp()));
}

static void test_tcp_connect_fails_when_no_free_socket(void) {
  CountingHal hal;
  for (uint8_t s = 0; s < 8; ++s) {
    hal.sn_sr[s] = SOCK_UDP;
  }
  eip::EipSocketW5500Tcp tcp(hal);
  TEST_ASSERT_FALSE(tcp.connect("192.168.1.20", 44818));
  TEST_ASSERT_FALSE(tcp.isConnected());
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.open_cmds));
}

static void test_tcp_connect_failure_closes_socket(void) {
  CountingHal hal;
  hal.connect_fail = true;
  eip::EipSocketW5500Tcp tcp(hal);
  TEST_ASSERT_FALSE(tcp.connect("192.168.1.20", 44818));
  TEST_ASSERT_FALSE(tcp.isConnected());
  TEST_ASSERT_TRUE(hal.open_cmds >= 1);
  TEST_ASSERT_TRUE(hal.close_cmds >= 1);
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countNotClosed()));
}

static void test_sendto_zero_dest_guard(void) {
  CountingHal hal;
  eip::EipSocketW5500Udp udp(hal);
  TEST_ASSERT_TRUE(udp.bind(2222, 0));
  const uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL_INT(-1, udp.sendTo(payload, sizeof(payload), 0, 2222));
  TEST_ASSERT_EQUAL_INT(
      4, udp.sendTo(payload, sizeof(payload), 0xC0A80114u, 2222));
}

static void test_recvfrom_nonblocking_vs_blocking(void) {
  CountingHal hal;
  eip::EipSocketW5500Udp udp(hal);
  TEST_ASSERT_TRUE(udp.bind(2222, 0));

  uint8_t buf[16] = {};
  TEST_ASSERT_EQUAL_INT(0, udp.recvFrom(buf, sizeof(buf), 0));

  const uint8_t payload[3] = {9, 8, 7};
  hal.enqueueUdpDatagram(payload, 3, 0xC0A80114u, 2222);
  TEST_ASSERT_EQUAL_INT(3, udp.recvFrom(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_UINT8(9, buf[0]);
  TEST_ASSERT_EQUAL_UINT8(8, buf[1]);
  TEST_ASSERT_EQUAL_UINT8(7, buf[2]);

  TEST_ASSERT_EQUAL_INT(0, udp.recvFrom(buf, sizeof(buf), 1));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_udp_p2p_rx_then_tx_per_dest);
  RUN_TEST(test_udp_multicast_split_bind);
  RUN_TEST(test_tcp_connect_fails_when_no_free_socket);
  RUN_TEST(test_tcp_connect_failure_closes_socket);
  RUN_TEST(test_sendto_zero_dest_guard);
  RUN_TEST(test_recvfrom_nonblocking_vs_blocking);
  return UNITY_END();
}
