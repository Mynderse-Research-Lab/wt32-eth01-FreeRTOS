/**
 * @file test_cell_net_l2_node.cpp
 * @brief CellNet L2 node dispatch host tests.
 */

#include "CellNetL2.h"
#include "CellNetL2Framing.h"
#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"
#include "unity.h"

#include <cstring>
#include <deque>
#include <vector>

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

void test_node_rx_dispatch_callbacks(void) {
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

void test_node_sequence_loss_detection(void) {
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

void test_node_tx_methods(void) {
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
