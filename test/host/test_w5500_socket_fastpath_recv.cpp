/**
 * @file test_w5500_socket_fastpath_recv.cpp
 * @brief UDP receive fast-path host tests.
 */

#include "test_w5500_socket_fastpath_internal.h"

using namespace w5500_fastpath_test;

void test_rx_rsr_rd_contiguous(void) {
  TEST_ASSERT_EQUAL_UINT16(0x0026, Sn_RX_RSR);
  TEST_ASSERT_EQUAL_UINT16(0x0028, Sn_RX_RD);
  TEST_ASSERT_EQUAL_UINT16(Sn_RX_RSR + 2, Sn_RX_RD);
}

void test_recv_nonblocking_empty_returns_zero(void) {
  CountingHal hal;
  uint8_t buf[16];
  hal.recv_cmd_writes = 0;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.recv_cmd_writes));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countRxBufReads()));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countRsrRdBursts()));
}

void test_recv_nonblocking_strips_header(void) {
  CountingHal hal;
  const uint8_t payload[] = {0x10, 0x20, 0x30, 0x40};
  hal.enqueueUdpDatagram(payload, sizeof(payload), 0xC0A80114, 2222);
  const uint16_t rd0 = hal.rx_rd;

  uint8_t out[16] = {};
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out, sizeof(payload));
  TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(rd0 + 8 + sizeof(payload)),
                           hal.rx_rd);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
  TEST_ASSERT_TRUE(hal.recv_cmd_writes >= 1);
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countRxBufReads()));
  TEST_ASSERT_TRUE(hal.countRsrRdBursts() >= 1);
}

void test_recv_nonblocking_truncates_but_advances(void) {
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

void test_recv_nonblocking_bad_header_drains(void) {
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

void test_recv_nonblocking_back_to_back(void) {
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

void test_recv_nonblocking_large_payload_second_read(void) {
  CountingHal hal;
  std::vector<uint8_t> payload(300, 0x5A);
  payload[0] = 0x11;
  payload[299] = 0x22;
  hal.enqueueUdpDatagram(payload.data(), static_cast<uint16_t>(payload.size()));

  uint8_t out[300] = {};
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      300, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8(0x11, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x22, out[299]);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.countRxBufReads()));
}

// Two queued datagrams: one RSR/RD pass, one payload burst, one RECV.
void test_recv_batch_two_datagrams_one_burst(void) {
  CountingHal hal;
  const uint8_t a[] = {0xA1, 0xA2};
  const uint8_t b[] = {0xB1, 0xB2, 0xB3};
  hal.enqueueUdpDatagram(a, sizeof(a));
  hal.enqueueUdpDatagram(b, sizeof(b));

  uint8_t buf[128] = {};
  w5500::UdpDatagram dg[4] = {};
  hal.recv_cmd_writes = 0;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_UINT32(
      2, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 4)));

  TEST_ASSERT_EQUAL_UINT16(2, dg[0].len);
  TEST_ASSERT_EQUAL_HEX8(0xA1, dg[0].data[0]);
  TEST_ASSERT_EQUAL_HEX8(0xA2, dg[0].data[1]);
  TEST_ASSERT_EQUAL_UINT16(3, dg[1].len);
  TEST_ASSERT_EQUAL_HEX8(0xB1, dg[1].data[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB3, dg[1].data[2]);
  // Views point into the caller's burst buffer.
  TEST_ASSERT_TRUE(dg[0].data == buf + 8);

  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countRxBufReads()));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(hal.countRsrRdBursts()));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.recv_cmd_writes));
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());

  TEST_ASSERT_EQUAL_UINT32(
      0, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 4)));
}

void test_recv_batch_empty_makes_no_recv(void) {
  CountingHal hal;
  uint8_t buf[64] = {};
  w5500::UdpDatagram dg[2] = {};
  hal.recv_cmd_writes = 0;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_UINT32(
      0, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 2)));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.recv_cmd_writes));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countRxBufReads()));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countRsrRdBursts()));
}

// maxOut caps the batch; the surplus stays queued and RX_RD advances only over
// what was handed back.
void test_recv_batch_respects_max_out(void) {
  CountingHal hal;
  const uint8_t a[] = {0xA1, 0xA2};
  const uint8_t b[] = {0xB1, 0xB2, 0xB3};
  hal.enqueueUdpDatagram(a, sizeof(a));
  hal.enqueueUdpDatagram(b, sizeof(b));

  uint8_t buf[128] = {};
  w5500::UdpDatagram dg[2] = {};
  TEST_ASSERT_EQUAL_UINT32(
      1, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 1)));
  TEST_ASSERT_EQUAL_UINT16(2, dg[0].len);
  TEST_ASSERT_EQUAL_UINT16(11, hal.rxRsr());  // 8 + 3 still queued

  TEST_ASSERT_EQUAL_UINT32(
      1, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 1)));
  TEST_ASSERT_EQUAL_UINT16(3, dg[0].len);
  TEST_ASSERT_EQUAL_HEX8(0xB1, dg[0].data[0]);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
}

void test_recv_batch_bad_header_drains(void) {
  CountingHal hal;
  const uint8_t bad0[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // declared length 0
  hal.enqueueRawRx(bad0, 8);
  uint8_t buf[64] = {};
  w5500::UdpDatagram dg[2] = {};
  TEST_ASSERT_EQUAL_UINT32(
      0, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 2)));
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());

  CountingHal hal2;
  const uint8_t bad1[8] = {0, 0, 0, 0, 0, 0, 0x00, 0x20};  // len=32, avail=8
  hal2.enqueueRawRx(bad1, 8);
  TEST_ASSERT_EQUAL_UINT32(
      0, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal2, 0, buf, sizeof(buf), dg, 2)));
  TEST_ASSERT_EQUAL_UINT16(0, hal2.rxRsr());
}

// MISO stuck high: every RX byte and Sn_RX_RSR read as 0xFF (G-B15).
static void fillRxStuckHigh(CountingHal& hal) {
  hal.rx_mem.assign(0x10000, 0xFF);
  hal.rx_rd = 0;
  hal.rx_wr = 0xFFFF;
}

// One header declaring payload_len, then 64 bytes, so avail is 72.
static void enqueueHeaderWithLength(CountingHal& hal, uint16_t payload_len) {
  uint8_t raw[72];
  std::memset(raw, 0xA5, sizeof(raw));
  raw[6] = static_cast<uint8_t>(payload_len >> 8);
  raw[7] = static_cast<uint8_t>(payload_len & 0xFF);
  hal.enqueueRawRx(raw, sizeof(raw));
}

void test_recv_batch_rejects_stuck_high_rx(void) {
  CountingHal hal;
  fillRxStuckHigh(hal);
  uint8_t buf[512] = {};
  w5500::UdpDatagram dg[16] = {};
  TEST_ASSERT_EQUAL_UINT32(
      0, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 16)));
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
}

void test_recv_batch_rejects_lengths_that_wrap_16_bits(void) {
  for (uint32_t len = 0xFFF8; len <= 0xFFFF; ++len) {
    CountingHal hal;
    enqueueHeaderWithLength(hal, static_cast<uint16_t>(len));
    uint8_t buf[128] = {};
    w5500::UdpDatagram dg[16] = {};
    TEST_ASSERT_EQUAL_UINT32(
        0, static_cast<uint32_t>(
               w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 16)));
    TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
  }
}

void test_recv_nonblocking_rejects_stuck_high_rx(void) {
  CountingHal hal;
  fillRxStuckHigh(hal);
  uint8_t out[300] = {};  // larger than the 264-byte peek scratch
  TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
}

void test_recv_nonblocking_rejects_lengths_that_wrap_16_bits(void) {
  for (uint32_t len = 0xFFF8; len <= 0xFFFF; ++len) {
    CountingHal hal;
    enqueueHeaderWithLength(hal, static_cast<uint16_t>(len));
    uint8_t out[300] = {};
    TEST_ASSERT_EQUAL_INT(0, w5500::socketRecvFromNonBlocking(hal, 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
  }
}

// A datagram larger than the batch buffer must not stall the FIFO.
void test_recv_batch_oversized_datagram_advances(void) {
  CountingHal hal;
  std::vector<uint8_t> payload(300, 0x5A);
  payload[0] = 0x11;
  hal.enqueueUdpDatagram(payload.data(), static_cast<uint16_t>(payload.size()));
  const uint8_t tail[] = {0xC1, 0xC2};
  hal.enqueueUdpDatagram(tail, sizeof(tail));

  uint8_t buf[64] = {};
  w5500::UdpDatagram dg[4] = {};
  TEST_ASSERT_EQUAL_UINT32(
      1, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 4)));
  TEST_ASSERT_EQUAL_UINT16(sizeof(buf) - 8, dg[0].len);
  TEST_ASSERT_EQUAL_HEX8(0x11, dg[0].data[0]);

  // The oversized frame was consumed whole, so the next one is readable.
  TEST_ASSERT_EQUAL_UINT32(
      1, static_cast<uint32_t>(
             w5500::socketRecvFromBatch(hal, 0, buf, sizeof(buf), dg, 4)));
  TEST_ASSERT_EQUAL_UINT16(2, dg[0].len);
  TEST_ASSERT_EQUAL_HEX8(0xC1, dg[0].data[0]);
  TEST_ASSERT_EQUAL_UINT16(0, hal.rxRsr());
}

void test_w5500_sram_pointer_wraparound(void) {
  CountingHal hal;
  // Initialize tx_wr to 0xFFF0 (near 16-bit wraparound)
  hal.tx_wr = 0xFFF0;
  
  // Use socketOpen to read tx_wr into software state
  w5500::socketOpen(hal, w5500::SocketMode::kUdp, 2222, 0);
  
  const uint8_t data[32] = {0};
  int ret = w5500::socketSendTo(hal, 0, data, sizeof(data), 0x01020304, 1234, false);
  TEST_ASSERT_EQUAL_INT(sizeof(data), ret);
  
  // We wrote 32 bytes starting at 0xFFF0. The new tx_wr should be 0xFFF0 + 32 = 0x0010 (16).
  // Check the recorded transactions.
  bool found_tx_wr = false;
  for (const auto& txn : hal.txns) {
    if (txn.kind == Txn::Kind::kReg16 && txn.addr == 0x0024 && txn.is_write) {
      found_tx_wr = true;
    }
  }
  TEST_ASSERT_TRUE(found_tx_wr);
  // We expect tx_wr to have wrapped around to 16 (0x0010). (0xFFF0 + 32 bytes)
  TEST_ASSERT_EQUAL_UINT16(0x0010, hal.tx_wr);
}
