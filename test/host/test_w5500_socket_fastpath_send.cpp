/**
 * @file test_w5500_socket_fastpath_send.cpp
 * @brief UDP send fast-path host tests.
 */

#include "test_w5500_socket_fastpath_internal.h"

using namespace w5500_fastpath_test;

void test_dipr_dport_contiguous(void) {
  TEST_ASSERT_EQUAL_UINT16(0x000C, Sn_DIPR);
  TEST_ASSERT_EQUAL_UINT16(0x0010, Sn_DPORT);
  TEST_ASSERT_EQUAL_UINT16(Sn_DIPR + 4, Sn_DPORT);
}

void test_sendto_caches_dest(void) {
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

void test_sendto_rewrites_on_dest_change(void) {
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

void test_sendto_restores_multicast_dest(void) {
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

void test_sendto_sendok_timeout_returns_error(void) {
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

void test_sendto_ir_timeout_and_discon(void) {
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

void test_sendto_rejects_bad_length(void) {
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

void test_w5500_handles_spi_corruption(void) {
  CountingHal hal;
  // If MISO is disconnected or corrupted to all 1s:
  for (int i = 0; i < 8; ++i) {
    hal.sn_sr[i] = 0xFF;
  }
  
  // socketOpen should fail because Sn_SR reads 0xFF on all sockets (none CLOSED)
  TEST_ASSERT_EQUAL_INT(-1, w5500::socketOpen(hal, w5500::SocketMode::kUdp, 2222, 0));
  
  // socketConnect should fail because Sn_SR reads 0xFF (not SOCK_INIT)
  TEST_ASSERT_FALSE(w5500::socketConnect(hal, 0, 0xC0A80114, 2222, 100));
}

void test_socket_close_clears_dest_cache(void) {
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

// Deferred mode: a cached-dest send issues SEND and returns without polling.
void test_sendto_deferred_skips_sendok_poll(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const uint32_t ip = 0xC0A80114;

  // First send changes the dest, so it still waits for SENDOK.
  TEST_ASSERT_EQUAL_INT(
      8, w5500::socketSendTo(hal, 0, payload, 8, ip, 2222, true));
  TEST_ASSERT_TRUE(hal.countCrIrPolls() >= 1);

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      8, w5500::socketSendTo(hal, 0, payload, 8, ip, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countCrIrPolls()));
  TEST_ASSERT_TRUE(hal.lastTxnIsCommandWrite());
  TEST_ASSERT_EQUAL_HEX8(Sn_CR_SEND, hal.last_cr_written);

  w5500::socketClose(hal, 0);
}

// The owed SENDOK is confirmed by exactly one poll at the head of the next send.
void test_sendto_deferred_confirms_on_next_send(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {1, 2, 3, 4};
  const uint32_t ip = 0xC0A80114;

  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countCrIrPolls()));
  // Write-only IR clear: the poll already carried the IR value.
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countIrReads()));
  TEST_ASSERT_TRUE(hal.lastTxnIsCommandWrite());
  TEST_ASSERT_EQUAL_HEX8(Sn_CR_SEND, hal.last_cr_written);

  w5500::socketClose(hal, 0);
}

// SENDOK not yet set on the deferred confirm: bounded spin, still succeeds.
void test_sendto_deferred_confirm_spins_when_not_ready(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {5, 6, 7, 8};
  const uint32_t ip = 0xC0A80114;

  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));

  hal.poll_pending_reads = 3;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(4, static_cast<uint32_t>(hal.countCrIrPolls()));

  w5500::socketClose(hal, 0);
}

// TIMEOUT / DISCON discovered on the deferred confirm fails the next send.
void test_sendto_deferred_confirm_reports_failure(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {1, 1, 1, 1};
  const uint32_t ip = 0xC0A80114;

  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));

  hal.poll_ir = Sn_IR_TIMEOUT;
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));

  // The failed confirm must not leave the socket owing another one.
  hal.poll_ir = Sn_IR_SENDOK;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countCrIrPolls()));

  w5500::socketClose(hal, 0);

  CountingHal hal2;
  w5500::socketClose(hal2, 0);
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal2, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal2, 0, payload, 4, ip, 2222, true));
  hal2.poll_ir = Sn_IR_DISCON;
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal2, 0, payload, 4, ip, 2222, true));
  w5500::socketClose(hal2, 0);
}

// A dest change keeps the blocking ARP-length SENDOK wait even in deferred mode.
void test_sendto_deferred_dest_change_still_blocks(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {9, 9, 9, 9};
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80114, 2222, true));

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80115, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countDestWrites()));
  TEST_ASSERT_TRUE(hal.countCrIrPolls() >= 1);

  // Nothing was deferred, so the following send owes no confirmation.
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80115, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countCrIrPolls()));

  // Dest change budgets the 10 ms ARP wait, not the 2 ms cached-dest wait.
  w5500::socketClose(hal, 0);
  hal.poll_stuck = true;
  const auto t0 = std::chrono::steady_clock::now();
  TEST_ASSERT_EQUAL_INT(
      -1, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80116, 2222, true));
  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - t0)
                              .count();
  TEST_ASSERT_TRUE(elapsed_us >= 5000);

  hal.poll_stuck = false;
  w5500::socketClose(hal, 0);
}

// Sn_TX_WR is host-owned: read once at OPEN, then tracked in software.
void test_tx_wr_read_once_after_open(void) {
  CountingHal hal;
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(0, w5500::socketOpen(hal, w5500::SocketMode::kUdp, 2222, 0));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countTxWrReads()));
  const uint8_t payload[4] = {1, 2, 3, 4};
  const uint16_t wr0 = hal.tx_wr;

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80114, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countTxWrReads()));
  TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(wr0 + 4), hal.tx_wr);

  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80114, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countTxWrReads()));
  TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(wr0 + 8), hal.tx_wr);

  // Re-open invalidates then re-seeds so a chip reset cannot be inherited.
  w5500::socketClose(hal, 0);
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(0, w5500::socketOpen(hal, w5500::SocketMode::kUdp, 2222, 0));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countTxWrReads()));
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, 0xC0A80114, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(hal.countTxWrReads()));

  w5500::socketClose(hal, 0);
}

// Close / recover-style reset must drop a deferred SENDOK so the next send
// does not confirm a SEND the chip no longer has.
void test_close_and_reset_clear_deferred_pending(void) {
  CountingHal hal;
  w5500::socketClose(hal, 0);
  const uint8_t payload[4] = {1, 2, 3, 4};
  const uint32_t ip = 0xC0A80114;

  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));

  w5500::socketClose(hal, 0);
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  // Dest cache was dropped, so this send waits for SENDOK once. A leftover
  // deferred confirm would add a second CR+IR poll.
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countCrIrPolls()));

  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  w5500::socketResetSoftwareState();
  hal.txns.clear();
  TEST_ASSERT_EQUAL_INT(
      4, w5500::socketSendTo(hal, 0, payload, 4, ip, 2222, true));
  TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(hal.countCrIrPolls()));

  w5500::socketClose(hal, 0);
}
