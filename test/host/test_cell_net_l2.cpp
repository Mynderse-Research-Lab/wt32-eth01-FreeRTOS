/**
 * @file test_cell_net_l2.cpp
 * @brief Host unit tests for CellNetL2 framing, validation, and node dispatching.
 */

#include "CellNetL2.h"
#include "CellNetL2Framing.h"
#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"
#include "unity.h"

#include <cstring>
#include <deque>
#include <vector>

void setUp(void) {}
void tearDown(void) {}

void test_framing_expected_lengths(void);
void test_framing_heartbeat_roundtrip(void);
void test_framing_vision_detect_roundtrip(void);
void test_framing_old_padded_vision_frame_ignores_capture(void);
void test_framing_conveyor_speed_roundtrip(void);
void test_framing_gantry_status_roundtrip(void);
void test_framing_cell_command_roundtrip(void);
void test_framing_reject_invalid_ethertype(void);
void test_framing_reject_invalid_version(void);
void test_framing_reject_truncated_frames(void);
void test_framing_cell_command_accepts_ethernet_padding(void);
void test_node_rx_dispatch_callbacks(void);
void test_node_sequence_loss_detection(void);
void test_node_tx_methods(void);
void test_framing_ota_sizes(void);
void test_framing_ota_data_roundtrip_and_limits(void);
void test_framing_ota_fixed_roundtrips(void);

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_framing_expected_lengths);
  RUN_TEST(test_framing_heartbeat_roundtrip);
  RUN_TEST(test_framing_vision_detect_roundtrip);
  RUN_TEST(test_framing_old_padded_vision_frame_ignores_capture);
  RUN_TEST(test_framing_conveyor_speed_roundtrip);
  RUN_TEST(test_framing_gantry_status_roundtrip);
  RUN_TEST(test_framing_cell_command_roundtrip);
  RUN_TEST(test_framing_reject_invalid_ethertype);
  RUN_TEST(test_framing_reject_invalid_version);
  RUN_TEST(test_framing_reject_truncated_frames);
  RUN_TEST(test_framing_cell_command_accepts_ethernet_padding);
  RUN_TEST(test_node_rx_dispatch_callbacks);
  RUN_TEST(test_node_sequence_loss_detection);
  RUN_TEST(test_node_tx_methods);
  RUN_TEST(test_framing_ota_sizes);
  RUN_TEST(test_framing_ota_data_roundtrip_and_limits);
  RUN_TEST(test_framing_ota_fixed_roundtrips);
  return UNITY_END();
}
