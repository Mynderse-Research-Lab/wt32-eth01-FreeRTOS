/**
 * @file test_cell_net_l2_framing.cpp
 * @brief CellNet L2 framing host tests.
 */

#include "CellNetL2.h"
#include "CellNetL2Framing.h"
#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"
#include "unity.h"

#include <cstring>
#include <deque>
#include <vector>

void test_framing_expected_lengths(void) {
  TEST_ASSERT_EQUAL_UINT32(sizeof(L2HeartbeatFrame),
                           CellNet::CellNetL2Framing::expectedFrameLength(
                               CellMsgType::HEARTBEAT));
  TEST_ASSERT_EQUAL_UINT32(sizeof(L2VisionDetectFrame),
                           CellNet::CellNetL2Framing::expectedFrameLength(
                               CellMsgType::VISION_DETECT));
  TEST_ASSERT_EQUAL_UINT32(sizeof(L2ConveyorSpeedFrame),
                           CellNet::CellNetL2Framing::expectedFrameLength(
                               CellMsgType::CONVEYOR_SPEED));
  TEST_ASSERT_EQUAL_UINT32(sizeof(L2GantryStatusFrame),
                           CellNet::CellNetL2Framing::expectedFrameLength(
                               CellMsgType::GANTRY_STATUS));
  TEST_ASSERT_EQUAL_UINT32(sizeof(L2CellCommandFrame),
                           CellNet::CellNetL2Framing::expectedFrameLength(
                               CellMsgType::CELL_COMMAND));
}

void test_framing_heartbeat_roundtrip(void) {
  const uint8_t src_mac[6] = {0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E};
  L2HeartbeatPayload payload{};
  payload.uptime_ms = 123456;
  payload.status_flags = 0xABCD;

  const auto bytes = CellNet::CellNetL2Framing::buildHeartbeatFrame(
      src_mac, CellNodeId::SUPERVISOR, 42, 987654, payload);

  TEST_ASSERT_EQUAL_UINT32(sizeof(L2HeartbeatFrame), bytes.size());

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(parsed.valid);
  TEST_ASSERT_EQUAL_UINT8(CELL_NET_L2_VERSION, parsed.cell.version);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::HEARTBEAT),
                          parsed.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellNodeId::SUPERVISOR),
                          parsed.cell.sender_id);
  TEST_ASSERT_EQUAL_UINT8(42, parsed.cell.sequence);
  TEST_ASSERT_EQUAL_UINT32(987654, parsed.cell.timestamp_us_low);
  TEST_ASSERT_EQUAL_UINT32(123456, parsed.payload.heartbeat.uptime_ms);
  TEST_ASSERT_EQUAL_UINT16(0xABCD, parsed.payload.heartbeat.status_flags);
}

void test_framing_vision_detect_roundtrip(void) {
  const uint8_t src_mac[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
  L2VisionDetectPayload payload{};
  payload.item_id = 999;
  payload.x_across_mm = 145.5f;
  payload.y_bat_mm = 12.25f;
  payload.theta_deg = -45.0f;
  payload.battery_class = 4;
  payload.reserved[0] = kVisionCaptureDispValid;
  payload.capture_displacement_m = 1.25f;
  payload.width_mm = 75.0f;
  payload.length_mm = 120.0f;
  payload.height_from_belt_mm = 80.0f;

  const auto bytes = CellNet::CellNetL2Framing::buildVisionDetectFrame(
      src_mac, CellNodeId::VISION, 101, 555123, payload);

  TEST_ASSERT_EQUAL_UINT32(sizeof(L2VisionDetectFrame), bytes.size());

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(parsed.valid);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::VISION_DETECT),
                          parsed.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellNodeId::VISION),
                          parsed.cell.sender_id);
  TEST_ASSERT_EQUAL_UINT32(999, parsed.payload.vision_detect.item_id);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 145.5f,
                           parsed.payload.vision_detect.x_across_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 12.25f,
                           parsed.payload.vision_detect.y_bat_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -45.0f,
                           parsed.payload.vision_detect.theta_deg);
  TEST_ASSERT_EQUAL_UINT8(4, parsed.payload.vision_detect.battery_class);
  TEST_ASSERT_EQUAL_UINT8(kVisionCaptureDispValid, parsed.payload.vision_detect.reserved[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.25f, parsed.payload.vision_detect.capture_displacement_m);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 75.0f, parsed.payload.vision_detect.width_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, parsed.payload.vision_detect.length_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 80.0f, parsed.payload.vision_detect.height_from_belt_mm);
}

void test_framing_old_padded_vision_frame_ignores_capture(void) {
  // Pre-extension payload is 20 bytes. A 60-byte padded frame still parses;
  // reserved[0] is clear, so the trailing float is not a capture displacement.
  std::vector<uint8_t> frame(60, 0);
  frame[12] = 0x88;
  frame[13] = 0xB5;
  frame[14] = CELL_NET_L2_VERSION;
  frame[15] = static_cast<uint8_t>(CellMsgType::VISION_DETECT);
  frame[16] = static_cast<uint8_t>(CellNodeId::VISION);
  const uint32_t item = 42;
  std::memcpy(frame.data() + 22, &item, sizeof(item));

  CellNet::ParsedFrame parsed{};
  TEST_ASSERT_TRUE(CellNet::CellNetL2Framing::parseFrame(frame.data(), frame.size(), parsed));
  TEST_ASSERT_EQUAL_UINT32(42, parsed.payload.vision_detect.item_id);
  TEST_ASSERT_EQUAL_UINT8(0, parsed.payload.vision_detect.reserved[0] & kVisionCaptureDispValid);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, parsed.payload.vision_detect.capture_displacement_m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, parsed.payload.vision_detect.width_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, parsed.payload.vision_detect.length_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, parsed.payload.vision_detect.height_from_belt_mm);
  TEST_ASSERT_TRUE(sizeof(L2VisionDetectFrame) <= 60);

  TEST_ASSERT_FALSE(CellNet::CellNetL2Framing::parseFrame(frame.data(), 42, parsed));
}

void test_framing_conveyor_speed_roundtrip(void) {
  const uint8_t src_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
  L2ConveyorSpeedPayload payload{};
  payload.speed_mm_s = 250.75f;
  payload.displacement_m = 18.520f;
  payload.raw_encoder_cnt = 92600;

  const auto bytes = CellNet::CellNetL2Framing::buildConveyorSpeedFrame(
      src_mac, CellNodeId::CONVEYOR, 77, 1000200, payload);

  TEST_ASSERT_EQUAL_UINT32(sizeof(L2ConveyorSpeedFrame), bytes.size());

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(parsed.valid);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::CONVEYOR_SPEED),
                          parsed.cell.msg_type);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.75f,
                           parsed.payload.conveyor_speed.speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 18.520f,
                           parsed.payload.conveyor_speed.displacement_m);
  TEST_ASSERT_EQUAL_INT32(92600,
                          parsed.payload.conveyor_speed.raw_encoder_cnt);
}

void test_framing_gantry_status_roundtrip(void) {
  const uint8_t src_mac[6] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x03};
  L2GantryStatusPayload payload{};
  payload.motion_state = 2;
  payload.active_slot = 1;
  payload.fault_flags = 0;
  payload.x_pos_mm = 210.0f;
  payload.z_pos_mm = 35.7f;
  payload.theta_deg = 90.0f;
  payload.last_cycle_time_ms = 824.5f;

  const auto bytes = CellNet::CellNetL2Framing::buildGantryStatusFrame(
      src_mac, CellNodeId::GANTRY, 88, 2000300, payload);

  TEST_ASSERT_EQUAL_UINT32(sizeof(L2GantryStatusFrame), bytes.size());

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(parsed.valid);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::GANTRY_STATUS),
                          parsed.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(2, parsed.payload.gantry_status.motion_state);
  TEST_ASSERT_EQUAL_UINT8(1, parsed.payload.gantry_status.active_slot);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 210.0f,
                           parsed.payload.gantry_status.x_pos_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 35.7f,
                           parsed.payload.gantry_status.z_pos_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f,
                           parsed.payload.gantry_status.theta_deg);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 824.5f,
                           parsed.payload.gantry_status.last_cycle_time_ms);
}

void test_framing_cell_command_roundtrip(void) {
  const uint8_t src_mac[6] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x04};
  L2CellCommandPayload payload{};
  payload.command_id = 5;  // PARK
  payload.param_u8 = 1;
  payload.param_u16 = 300;
  payload.param_float = 35.7f;

  const auto bytes = CellNet::CellNetL2Framing::buildCellCommandFrame(
      src_mac, CellNodeId::SUPERVISOR, 12, 3000400, payload);

  TEST_ASSERT_EQUAL_UINT32(sizeof(L2CellCommandFrame), bytes.size());

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(parsed.valid);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::CELL_COMMAND),
                          parsed.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(5, parsed.payload.cell_command.command_id);
  TEST_ASSERT_EQUAL_UINT8(1, parsed.payload.cell_command.param_u8);
  TEST_ASSERT_EQUAL_UINT16(300, parsed.payload.cell_command.param_u16);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 35.7f,
                           parsed.payload.cell_command.param_float);
}

void test_framing_reject_invalid_ethertype(void) {
  L2HeartbeatFrame frame{};
  frame.eth.ethertype = 0x0800;  // IPv4, not 0x88B5
  frame.cell.version = CELL_NET_L2_VERSION;
  frame.cell.msg_type = static_cast<uint8_t>(CellMsgType::HEARTBEAT);

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(
      reinterpret_cast<const uint8_t*>(&frame), sizeof(frame), parsed);
  TEST_ASSERT_FALSE(ok);
  TEST_ASSERT_FALSE(parsed.valid);
}

void test_framing_reject_invalid_version(void) {
  const uint8_t src_mac[6] = {0};
  L2HeartbeatPayload payload{};
  auto bytes = CellNet::CellNetL2Framing::buildHeartbeatFrame(
      src_mac, CellNodeId::GANTRY, 1, 100, payload);

  // Corrupt version field (byte 14)
  bytes[14] = 0x02;

  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size(), parsed);
  TEST_ASSERT_FALSE(ok);
}

void test_framing_reject_truncated_frames(void) {
  const uint8_t src_mac[6] = {0};
  L2HeartbeatPayload payload{};
  auto bytes = CellNet::CellNetL2Framing::buildHeartbeatFrame(
      src_mac, CellNodeId::GANTRY, 1, 100, payload);

  // Truncate by 1 byte
  CellNet::ParsedFrame parsed{};
  const bool ok = CellNet::CellNetL2Framing::parseFrame(bytes.data(),
                                                        bytes.size() - 1, parsed);
  TEST_ASSERT_FALSE(ok);
}

void test_framing_cell_command_accepts_ethernet_padding(void) {
  const uint8_t src_mac[6] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x04};
  L2CellCommandPayload payload{};
  payload.command_id = 3;
  auto bytes = CellNet::CellNetL2Framing::buildCellCommandFrame(
      src_mac, CellNodeId::SUPERVISOR, 7, 1000, payload);

  CellNet::ParsedFrame exact{};
  TEST_ASSERT_TRUE(CellNet::CellNetL2Framing::parseFrame(bytes.data(), bytes.size(), exact));
  TEST_ASSERT_EQUAL_UINT8(3, exact.payload.cell_command.command_id);

  bytes.resize(CellNet::CellNetL2Framing::kMinEthFrameLen, 0);
  CellNet::ParsedFrame padded{};
  TEST_ASSERT_TRUE(CellNet::CellNetL2Framing::parseFrame(bytes.data(), bytes.size(), padded));
  TEST_ASSERT_EQUAL_UINT8(exact.payload.cell_command.command_id,
                          padded.payload.cell_command.command_id);

  bytes.resize(CellNet::CellNetL2Framing::kMaxPaddedRxLen + 1, 0);
  CellNet::ParsedFrame oversized{};
  TEST_ASSERT_FALSE(CellNet::CellNetL2Framing::parseFrame(bytes.data(), bytes.size(), oversized));
}
