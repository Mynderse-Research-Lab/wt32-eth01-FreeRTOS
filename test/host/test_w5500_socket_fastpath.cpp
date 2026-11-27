// Host tests for Class 1 UDP send fast-path (dest cache + transaction diet)
// and non-blocking UDP recv (header strip / drain).

#include "W5500.h"
#include "W5500Hal.h"
#include "W5500BusMutex.h"
#include "W5500Socket.h"
#include "unity.h"

#include <chrono>
#include <cstring>
#include <vector>

#include "test_w5500_socket_fastpath_internal.h"
using namespace w5500_fastpath_test;

void setUp(void) {}
void tearDown(void) {}

void test_dipr_dport_contiguous(void);
void test_sendto_caches_dest(void);
void test_sendto_rewrites_on_dest_change(void);
void test_sendto_restores_multicast_dest(void);
void test_sendto_sendok_timeout_returns_error(void);
void test_sendto_ir_timeout_and_discon(void);
void test_sendto_rejects_bad_length(void);
void test_w5500_handles_spi_corruption(void);
void test_socket_close_clears_dest_cache(void);
void test_sendto_deferred_skips_sendok_poll(void);
void test_sendto_deferred_confirms_on_next_send(void);
void test_sendto_deferred_confirm_spins_when_not_ready(void);
void test_sendto_deferred_confirm_reports_failure(void);
void test_sendto_deferred_dest_change_still_blocks(void);
void test_tx_wr_read_once_after_open(void);
void test_close_and_reset_clear_deferred_pending(void);
void test_rx_rsr_rd_contiguous(void);
void test_recv_nonblocking_empty_returns_zero(void);
void test_recv_nonblocking_strips_header(void);
void test_recv_nonblocking_truncates_but_advances(void);
void test_recv_nonblocking_bad_header_drains(void);
void test_recv_nonblocking_back_to_back(void);
void test_recv_nonblocking_large_payload_second_read(void);
void test_recv_batch_two_datagrams_one_burst(void);
void test_recv_batch_empty_makes_no_recv(void);
void test_recv_batch_respects_max_out(void);
void test_recv_batch_bad_header_drains(void);
void test_recv_batch_rejects_stuck_high_rx(void);
void test_recv_batch_rejects_lengths_that_wrap_16_bits(void);
void test_recv_nonblocking_rejects_stuck_high_rx(void);
void test_recv_nonblocking_rejects_lengths_that_wrap_16_bits(void);
void test_recv_batch_oversized_datagram_advances(void);
void test_w5500_sram_pointer_wraparound(void);
void test_tcp_send_nowait_skips_sendok_poll(void);
void test_tcp_send_nowait_reports_the_previous_failure(void);
void test_tcp_send_nowait_needs_an_established_socket(void);
void test_tcp_send_ignores_a_stale_sendok(void);
void test_bus_stats_take_does_not_need_the_bus_mutex(void);

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_bus_stats_take_does_not_need_the_bus_mutex);
  RUN_TEST(test_tcp_send_nowait_skips_sendok_poll);
  RUN_TEST(test_tcp_send_nowait_reports_the_previous_failure);
  RUN_TEST(test_tcp_send_nowait_needs_an_established_socket);
  RUN_TEST(test_tcp_send_ignores_a_stale_sendok);
  RUN_TEST(test_dipr_dport_contiguous);
  RUN_TEST(test_rx_rsr_rd_contiguous);
  RUN_TEST(test_sendto_caches_dest);
  RUN_TEST(test_sendto_rewrites_on_dest_change);
  RUN_TEST(test_sendto_restores_multicast_dest);
  RUN_TEST(test_sendto_sendok_timeout_returns_error);
  RUN_TEST(test_sendto_ir_timeout_and_discon);
  RUN_TEST(test_sendto_rejects_bad_length);
  RUN_TEST(test_socket_close_clears_dest_cache);
  RUN_TEST(test_sendto_deferred_skips_sendok_poll);
  RUN_TEST(test_sendto_deferred_confirms_on_next_send);
  RUN_TEST(test_sendto_deferred_confirm_spins_when_not_ready);
  RUN_TEST(test_sendto_deferred_confirm_reports_failure);
  RUN_TEST(test_sendto_deferred_dest_change_still_blocks);
  RUN_TEST(test_tx_wr_read_once_after_open);
  RUN_TEST(test_close_and_reset_clear_deferred_pending);
  RUN_TEST(test_recv_nonblocking_empty_returns_zero);
  RUN_TEST(test_recv_nonblocking_strips_header);
  RUN_TEST(test_recv_nonblocking_truncates_but_advances);
  RUN_TEST(test_recv_nonblocking_bad_header_drains);
  RUN_TEST(test_recv_nonblocking_back_to_back);
  RUN_TEST(test_recv_nonblocking_large_payload_second_read);
  RUN_TEST(test_recv_batch_two_datagrams_one_burst);
  RUN_TEST(test_recv_batch_empty_makes_no_recv);
  RUN_TEST(test_recv_batch_respects_max_out);
  RUN_TEST(test_recv_batch_bad_header_drains);
  RUN_TEST(test_recv_batch_oversized_datagram_advances);
  RUN_TEST(test_recv_batch_rejects_stuck_high_rx);
  RUN_TEST(test_recv_batch_rejects_lengths_that_wrap_16_bits);
  RUN_TEST(test_recv_nonblocking_rejects_stuck_high_rx);
  RUN_TEST(test_recv_nonblocking_rejects_lengths_that_wrap_16_bits);
  RUN_TEST(test_w5500_sram_pointer_wraparound);
  RUN_TEST(test_w5500_handles_spi_corruption);
  return UNITY_END();
}
