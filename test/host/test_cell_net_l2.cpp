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
  payload.battery_class = 2;

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
  TEST_ASSERT_EQUAL_UINT8(2, parsed.payload.vision_detect.battery_class);
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
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_framing_expected_lengths);
  RUN_TEST(test_framing_heartbeat_roundtrip);
  RUN_TEST(test_framing_vision_detect_roundtrip);
  RUN_TEST(test_framing_conveyor_speed_roundtrip);
  RUN_TEST(test_framing_gantry_status_roundtrip);
  RUN_TEST(test_framing_cell_command_roundtrip);
  RUN_TEST(test_framing_reject_invalid_ethertype);
  RUN_TEST(test_framing_reject_invalid_version);
  RUN_TEST(test_framing_reject_truncated_frames);
  RUN_TEST(test_node_rx_dispatch_callbacks);
  RUN_TEST(test_node_sequence_loss_detection);
  RUN_TEST(test_node_tx_methods);
  return UNITY_END();
}
