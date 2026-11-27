#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "CipMessageRouter.h"
#include "CipTcpIpInterface.h"
#include "EipClass1DeferredLog.h"
#include "EipClass1Timing.h"
#include "EipClass1TimingStats.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipIoConnection.h"
#include "EipMultiScanner.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipScannerPolicy.h"
#include "EipSession.h"
#include "EipTransport.h"

void setUp(void) {}
void tearDown(void) {}

using eip::Bytes;

void test_session_register(void);
void test_class1_cpf_bytes(void);
void test_io_connection_output_frame(void);
void test_io_connection_parse_roundtrip(void);
void test_io_connection_parse_t_to_o_cpf(void);
void test_session_explicit_roundtrip(void);
void test_scanner_connect_forward_open(void);
void test_scanner_full_lifecycle(void);
void test_scanner_recv_timeout(void);
void test_parse_class1_input_cpf_demux_id(void);
void test_parse_class1_input_cpf_view(void);
void test_graceful_close_uses_a_live_session(void);
void test_graceful_close_rebuilds_a_dropped_session(void);
void test_graceful_close_retries_on_a_new_session_when_the_send_fails(void);
void test_graceful_close_keeps_the_axis_connected_until_closed(void);
void test_graceful_close_reports_a_failed_reconnect(void);
void test_graceful_close_ignores_a_closed_axis(void);
void test_multi_scanner_dual_demux(void);
void test_class1_rpi_remainder_and_miss_policy(void);
void test_stale_teardown_boundary(void);
void test_class1_timing_stats_ring_and_cmd_to_start(void);
void test_class1_timing_drain_ring_and_pace_counters(void);
void test_multi_scanner_input_miss_status(void);
void test_fo_delay_before_first_exchange_not_stale(void);
void test_drain_applies_multiple_datagrams_one_cycle(void);
void test_first_to_reported_once_across_cyclic_start(void);
void test_class1_deferred_log_fifo_and_overflow(void);
void test_scanner_policy_cip_timeout_multiplier(void);
void test_hcs01_stale_does_not_teardown_kinetix(void);
void test_axis_reopen_backoff_policy(void);
void test_hcs01_silent_axis_is_reopened_alone(void);
void test_hcs01_reopen_failure_backs_off(void);
void test_drain_ignores_duplicate_cid(void);
void test_drain_ignores_unknown_cid(void);
void test_drain_stops_when_all_axes_got(void);
void test_drain_is_bounded(void);
void test_exchange_records_drain_sample(void);
void test_batch_drain_applies_both_axes_in_one_call(void);
void test_batch_drain_is_bounded(void);
void test_feedback_age_resets_on_fresh_data(void);
void test_exchange_reports_send_failure(void);
void test_cip_sequence_number_wraparound(void);
void test_session_nop_frame(void);
void test_session_nop_needs_a_session(void);
void test_encap_nop_interval_policy(void);
void test_encap_inactivity_timeout_request(void);
void test_open_reads_the_timeout_and_spaces_nops(void);
void test_unreadable_timeout_assumes_the_default(void);
void test_disabled_timeout_sends_no_nop(void);
void test_failed_nop_is_logged_once_and_not_recovered(void);

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_session_nop_frame);
  RUN_TEST(test_session_nop_needs_a_session);
  RUN_TEST(test_encap_nop_interval_policy);
  RUN_TEST(test_encap_inactivity_timeout_request);
  RUN_TEST(test_open_reads_the_timeout_and_spaces_nops);
  RUN_TEST(test_unreadable_timeout_assumes_the_default);
  RUN_TEST(test_disabled_timeout_sends_no_nop);
  RUN_TEST(test_failed_nop_is_logged_once_and_not_recovered);
  RUN_TEST(test_session_register);
  RUN_TEST(test_class1_cpf_bytes);
  RUN_TEST(test_io_connection_output_frame);
  RUN_TEST(test_io_connection_parse_roundtrip);
  RUN_TEST(test_io_connection_parse_t_to_o_cpf);
  RUN_TEST(test_session_explicit_roundtrip);
  RUN_TEST(test_scanner_connect_forward_open);
  RUN_TEST(test_scanner_full_lifecycle);
  RUN_TEST(test_scanner_recv_timeout);
  RUN_TEST(test_parse_class1_input_cpf_demux_id);
  RUN_TEST(test_parse_class1_input_cpf_view);
  RUN_TEST(test_multi_scanner_dual_demux);
  RUN_TEST(test_class1_rpi_remainder_and_miss_policy);
  RUN_TEST(test_stale_teardown_boundary);
  RUN_TEST(test_class1_timing_stats_ring_and_cmd_to_start);
  RUN_TEST(test_class1_timing_drain_ring_and_pace_counters);
  RUN_TEST(test_multi_scanner_input_miss_status);
  RUN_TEST(test_fo_delay_before_first_exchange_not_stale);
  RUN_TEST(test_drain_applies_multiple_datagrams_one_cycle);
  RUN_TEST(test_first_to_reported_once_across_cyclic_start);
  RUN_TEST(test_class1_deferred_log_fifo_and_overflow);
  RUN_TEST(test_scanner_policy_cip_timeout_multiplier);
  RUN_TEST(test_hcs01_stale_does_not_teardown_kinetix);
  RUN_TEST(test_axis_reopen_backoff_policy);
  RUN_TEST(test_hcs01_silent_axis_is_reopened_alone);
  RUN_TEST(test_hcs01_reopen_failure_backs_off);
  RUN_TEST(test_drain_ignores_duplicate_cid);
  RUN_TEST(test_drain_ignores_unknown_cid);
  RUN_TEST(test_drain_stops_when_all_axes_got);
  RUN_TEST(test_drain_is_bounded);
  RUN_TEST(test_exchange_records_drain_sample);
  RUN_TEST(test_batch_drain_applies_both_axes_in_one_call);
  RUN_TEST(test_batch_drain_is_bounded);
  RUN_TEST(test_feedback_age_resets_on_fresh_data);
  RUN_TEST(test_exchange_reports_send_failure);
  RUN_TEST(test_cip_sequence_number_wraparound);
  RUN_TEST(test_graceful_close_uses_a_live_session);
  RUN_TEST(test_graceful_close_rebuilds_a_dropped_session);
  RUN_TEST(test_graceful_close_retries_on_a_new_session_when_the_send_fails);
  RUN_TEST(test_graceful_close_keeps_the_axis_connected_until_closed);
  RUN_TEST(test_graceful_close_reports_a_failed_reconnect);
  RUN_TEST(test_graceful_close_ignores_a_closed_axis);
  return UNITY_END();
}
