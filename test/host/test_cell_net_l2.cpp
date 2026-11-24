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

namespace {

class FakeL2Transport : public CellNet::IL2Transport {
 public:
  uint8_t mac[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
  bool link_up = true;
  std::deque<std::vector<uint8_t>> sent_frames;
  CellNet::RxFrameCallback rx_cb = nullptr;

  bool sendFrame(const uint8_t* data, size_t length) override {
    if (!link_up || data == nullptr || length == 0) {
      return false;
    }
    sent_frames.emplace_back(data, data + length);
    return true;
  }

  bool getMacAddress(uint8_t mac_out[6]) const override {
    if (mac_out == nullptr) return false;
    std::memcpy(mac_out, mac, 6);
    return true;
  }

  bool isLinkUp() const override { return link_up; }

  void setRxCallback(CellNet::RxFrameCallback callback) override {
    rx_cb = std::move(callback);
  }

  void injectRxFrame(const uint8_t* data, size_t length) {
    if (rx_cb) {
      rx_cb(data, length);
    }
  }
};

}  // namespace

static void test_framing_expected_lengths(void) {
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

static void test_framing_heartbeat_roundtrip(void) {
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

static void test_framing_vision_detect_roundtrip(void) {
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

static void test_framing_old_padded_vision_frame_ignores_capture(void) {
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

static void test_framing_conveyor_speed_roundtrip(void) {
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

static void test_framing_gantry_status_roundtrip(void) {
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

static void test_framing_cell_command_roundtrip(void) {
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

static void test_framing_reject_invalid_ethertype(void) {
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

static void test_framing_reject_invalid_version(void) {
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

static void test_framing_reject_truncated_frames(void) {
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

static void test_framing_cell_command_accepts_ethernet_padding(void) {
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

static void test_node_rx_dispatch_callbacks(void) {
  FakeL2Transport transport;
  CellNet::CellNetL2Node node(transport, CellNodeId::GANTRY);
  TEST_ASSERT_TRUE(node.begin());

  bool heartbeat_received = false;
  bool conveyor_received = false;
  bool vision_received = false;
  bool command_received = false;

  node.onHeartbeat([&](const L2HeartbeatPayload& payload, const L2CellHeader& hdr) {
    heartbeat_received = true;
    TEST_ASSERT_EQUAL_UINT32(777, payload.uptime_ms);
  });

  node.onConveyorSpeed([&](const L2ConveyorSpeedPayload& payload, const L2CellHeader& hdr) {
    conveyor_received = true;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.0f, payload.speed_mm_s);
  });

  node.onVisionDetect([&](const L2VisionDetectPayload& payload, const L2CellHeader& hdr) {
    vision_received = true;
    TEST_ASSERT_EQUAL_UINT32(55, payload.item_id);
  });

  node.onCellCommand([&](const L2CellCommandPayload& payload, const L2CellHeader& hdr) {
    command_received = true;
    TEST_ASSERT_EQUAL_UINT8(1, payload.command_id);
  });

  // Inject Heartbeat
  L2HeartbeatPayload hb{};
  hb.uptime_ms = 777;
  const auto hb_bytes = CellNet::CellNetL2Framing::buildHeartbeatFrame(
      transport.mac, CellNodeId::SUPERVISOR, 1, 1000, hb);
  transport.injectRxFrame(hb_bytes.data(), hb_bytes.size());
  TEST_ASSERT_TRUE(heartbeat_received);

  // Inject Conveyor Speed
  L2ConveyorSpeedPayload conv{};
  conv.speed_mm_s = 150.0f;
  const auto conv_bytes = CellNet::CellNetL2Framing::buildConveyorSpeedFrame(
      transport.mac, CellNodeId::CONVEYOR, 1, 1010, conv);
  transport.injectRxFrame(conv_bytes.data(), conv_bytes.size());
  TEST_ASSERT_TRUE(conveyor_received);

  // Inject Vision Detect
  L2VisionDetectPayload vis{};
  vis.item_id = 55;
  const auto vis_bytes = CellNet::CellNetL2Framing::buildVisionDetectFrame(
      transport.mac, CellNodeId::VISION, 1, 1020, vis);
  transport.injectRxFrame(vis_bytes.data(), vis_bytes.size());
  TEST_ASSERT_TRUE(vision_received);

  // Inject Cell Command
  L2CellCommandPayload cmd{};
  cmd.command_id = 1;
  const auto cmd_bytes = CellNet::CellNetL2Framing::buildCellCommandFrame(
      transport.mac, CellNodeId::SUPERVISOR, 2, 1030, cmd);
  transport.injectRxFrame(cmd_bytes.data(), cmd_bytes.size());
  TEST_ASSERT_TRUE(command_received);

  const auto stats = node.getStats();
  TEST_ASSERT_EQUAL_UINT32(4, stats.rx_frames);
  TEST_ASSERT_EQUAL_UINT32(0, stats.rx_invalid_frames);
  TEST_ASSERT_EQUAL_UINT32(0, stats.rx_sequence_drops);
}

static void test_node_sequence_loss_detection(void) {
  FakeL2Transport transport;
  CellNet::CellNetL2Node node(transport, CellNodeId::GANTRY);
  TEST_ASSERT_TRUE(node.begin());

  L2ConveyorSpeedPayload conv{};
  conv.speed_mm_s = 100.0f;

  // Send Seq 10
  const auto f1 = CellNet::CellNetL2Framing::buildConveyorSpeedFrame(
      transport.mac, CellNodeId::CONVEYOR, 10, 1000, conv);
  transport.injectRxFrame(f1.data(), f1.size());

  // Send Seq 14 (lost 11, 12, 13 -> 3 drops)
  const auto f2 = CellNet::CellNetL2Framing::buildConveyorSpeedFrame(
      transport.mac, CellNodeId::CONVEYOR, 14, 1040, conv);
  transport.injectRxFrame(f2.data(), f2.size());

  const auto stats = node.getStats();
  TEST_ASSERT_EQUAL_UINT32(2, stats.rx_frames);
  TEST_ASSERT_EQUAL_UINT32(3, stats.rx_sequence_drops);
}

static void test_node_tx_methods(void) {
  FakeL2Transport transport;
  CellNet::CellNetL2Node node(transport, CellNodeId::GANTRY);
  TEST_ASSERT_TRUE(node.begin());

  TEST_ASSERT_TRUE(node.sendHeartbeat(1000, 0));
  TEST_ASSERT_TRUE(node.sendGantryStatus(1, 0, 10.0f, 20.0f, 0.0f, 150.0f, 0));
  TEST_ASSERT_TRUE(node.sendConveyorSpeed(150.0f, 12.0f, 5000));
  TEST_ASSERT_TRUE(node.sendVisionDetect(101, 50.0f, 10.0f, 0.0f, 1));
  TEST_ASSERT_TRUE(node.sendCellCommand(1, 0, 0, 0.0f));

  const auto stats = node.getStats();
  TEST_ASSERT_EQUAL_UINT32(5, stats.tx_frames);
  TEST_ASSERT_EQUAL_UINT32(5, transport.sent_frames.size());
  for (const auto& frame : transport.sent_frames) {
    TEST_ASSERT_EQUAL_UINT32(CellNet::CellNetL2Framing::kMinEthFrameLen, frame.size());
  }
}

static void test_framing_ota_sizes(void) {
  using F = CellNet::CellNetL2Framing;
  TEST_ASSERT_EQUAL_size_t(34, F::expectedFrameLength(CellMsgType::OTA_BEGIN));
  TEST_ASSERT_EQUAL_size_t(30, F::expectedFrameLength(CellMsgType::OTA_DATA));
  TEST_ASSERT_EQUAL_size_t(44, F::expectedFrameLength(CellMsgType::OTA_STATUS));
  TEST_ASSERT_EQUAL_size_t(30, F::expectedFrameLength(CellMsgType::OTA_END));
  TEST_ASSERT_EQUAL_size_t(38, F::expectedFrameLength(CellMsgType::OTA_RESULT));
}

static void test_framing_ota_data_roundtrip_and_limits(void) {
  using F = CellNet::CellNetL2Framing;
  const uint8_t src[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  const uint8_t dst[6] = {0xa8, 0x03, 0x2a, 0x22, 0xbf, 0x8f};
  uint8_t data[478];
  for (size_t i = 0; i < sizeof(data); ++i) {
    data[i] = static_cast<uint8_t>(i * 7u);
  }
  L2OtaDataHeader h{};
  h.session = 0xBEEF;
  h.block = 205;
  h.frag = 8;
  h.flags = 1;
  h.data_len = 456;
  auto bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 200, 5, h, data, dst);
  TEST_ASSERT_EQUAL_size_t(486, bytes.size());  // variant B frame
  CellNet::ParsedFrame p{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), p));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::OTA_DATA), p.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(200, p.cell.sequence);
  TEST_ASSERT_EQUAL_HEX16(0xBEEF, p.payload.ota_data.session);
  TEST_ASSERT_EQUAL_UINT16(205, p.payload.ota_data.block);
  TEST_ASSERT_EQUAL_UINT8(8, p.payload.ota_data.frag);
  TEST_ASSERT_EQUAL_UINT8(1, p.payload.ota_data.flags);
  TEST_ASSERT_EQUAL_UINT16(456, p.payload.ota_data.data_len);
  TEST_ASSERT_NOT_NULL(p.ota_data);
  TEST_ASSERT_EQUAL_MEMORY(data, p.ota_data, 456);

  // A short last fragment is padded on the wire; data_len still governs.
  h.data_len = 4;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame small{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), small));
  TEST_ASSERT_EQUAL_UINT16(4, small.payload.ota_data.data_len);

  // data_len larger than the frame carries: rejected.
  h.data_len = 40;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(bytes.size() - 1);
  CellNet::ParsedFrame trunc{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), trunc));

  // data_len 0: rejected.
  h.data_len = 0;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame empty{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), empty));

  // 508 bytes is the cap for OTA_DATA; 509 is rejected.
  h.data_len = 478;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  TEST_ASSERT_EQUAL_size_t(F::kMaxOtaDataFrameLen, bytes.size());
  CellNet::ParsedFrame cap{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), cap));
  bytes.push_back(0);
  CellNet::ParsedFrame over{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), over));
}

static void test_framing_ota_fixed_roundtrips(void) {
  using F = CellNet::CellNetL2Framing;
  const uint8_t src[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};

  L2OtaBeginPayload b{};
  b.session = 7;
  b.block_size = 4096;
  b.frag_data = 456;
  b.image_size = 843360;
  auto bytes = F::buildOtaBeginFrame(src, CellNodeId::SUPERVISOR, 1, 2, b);
  CellNet::ParsedFrame pb{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pb));
  TEST_ASSERT_EQUAL_UINT32(843360, pb.payload.ota_begin.image_size);
  TEST_ASSERT_EQUAL_UINT16(456, pb.payload.ota_begin.frag_data);

  L2OtaStatusPayload st{};
  st.session = 7;
  st.block = 12;
  st.status = 3;
  st.processed_seq = 250;
  st.missing[0] = 0x08;
  bytes = F::buildOtaStatusFrame(src, CellNodeId::GANTRY, 1, 2, st);
  CellNet::ParsedFrame ps{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), ps));
  TEST_ASSERT_EQUAL_UINT8(250, ps.payload.ota_status.processed_seq);
  TEST_ASSERT_EQUAL_HEX8(0x08, ps.payload.ota_status.missing[0]);

  L2OtaEndPayload e{};
  e.session = 7;
  e.image_size = 843360;
  bytes = F::buildOtaEndFrame(src, CellNodeId::SUPERVISOR, 1, 2, e);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame pe{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pe));
  TEST_ASSERT_EQUAL_UINT32(843360, pe.payload.ota_end.image_size);

  L2OtaResultPayload r{};
  r.session = 7;
  r.status = 4;
  r.erase_ms = 2788;
  r.emac_missed = 0;
  r.emac_overflow = 1;
  bytes = F::buildOtaResultFrame(src, CellNodeId::GANTRY, 1, 2, r);
  CellNet::ParsedFrame pr{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pr));
  TEST_ASSERT_EQUAL_UINT16(2788, pr.payload.ota_result.erase_ms);
  TEST_ASSERT_EQUAL_UINT16(1, pr.payload.ota_result.emac_overflow);

  // The 64-byte cap still applies to every type but OTA_DATA.
  bytes.resize(F::kMaxPaddedRxLen + 1, 0);
  CellNet::ParsedFrame big{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), big));
}

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
