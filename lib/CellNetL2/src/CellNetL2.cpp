/**
 * @file CellNetL2.cpp
 * @brief High-speed, reusable OSI Layer-2 Communication Node implementation.
 */

#include "CellNetL2.h"

#include <chrono>
#include <cstring>

#ifdef ESP_PLATFORM
#include "esp_timer.h"
#endif

namespace CellNet {

namespace {

uint32_t getCurrentTimestampUs() {
#ifdef ESP_PLATFORM
  return static_cast<uint32_t>(esp_timer_get_time() & 0xFFFFFFFF);
#else
  const auto now = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch())
          .count() &
      0xFFFFFFFF);
#endif
}

}  // namespace

CellNetL2Node::CellNetL2Node(IL2Transport& transport, CellNodeId node_id)
    : transport_(transport), node_id_(node_id) {}

bool CellNetL2Node::begin() {
  if (!transport_.getMacAddress(self_mac_)) {
    // If MAC retrieval fails, use zeros or continue gracefully
    std::memset(self_mac_, 0, sizeof(self_mac_));
  }

  // Register the incoming frame listener on the transport
  transport_.setRxCallback(
      [this](const uint8_t* buffer, size_t length) {
        processIncomingFrame(buffer, length);
      });

  initialized_ = true;
  return true;
}

bool CellNetL2Node::isReady() const {
  return initialized_ && transport_.isLinkUp();
}

bool CellNetL2Node::sendHeartbeat(uint32_t uptime_ms, uint16_t status_flags,
                                  const uint8_t dest_mac[6]) {
  if (!initialized_) {
    return false;
  }

  L2HeartbeatPayload payload{};
  payload.uptime_ms = uptime_ms;
  payload.status_flags = status_flags;

  const uint32_t ts_us = getCurrentTimestampUs();
  const auto frame = CellNetL2Framing::buildHeartbeatFrame(
      self_mac_, node_id_, sequence_++, ts_us, payload, dest_mac);

  const bool ok = transport_.sendFrame(frame.data(), frame.size());
  if (ok) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_frames++;
  }
  return ok;
}

bool CellNetL2Node::sendVisionDetect(uint32_t item_id, float x_across_mm,
                                     float y_bat_mm, float theta_deg,
                                     uint8_t battery_class,
                                     const uint8_t dest_mac[6]) {
  if (!initialized_) {
    return false;
  }

  L2VisionDetectPayload payload{};
  payload.item_id = item_id;
  payload.x_across_mm = x_across_mm;
  payload.y_bat_mm = y_bat_mm;
  payload.theta_deg = theta_deg;
  payload.battery_class = battery_class;

  const uint32_t ts_us = getCurrentTimestampUs();
  const auto frame = CellNetL2Framing::buildVisionDetectFrame(
      self_mac_, node_id_, sequence_++, ts_us, payload, dest_mac);

  const bool ok = transport_.sendFrame(frame.data(), frame.size());
  if (ok) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_frames++;
  }
  return ok;
}

bool CellNetL2Node::sendConveyorSpeed(float speed_mm_s, float displacement_m,
                                      int32_t raw_encoder_cnt,
                                      const uint8_t dest_mac[6]) {
  if (!initialized_) {
    return false;
  }

  L2ConveyorSpeedPayload payload{};
  payload.speed_mm_s = speed_mm_s;
  payload.displacement_m = displacement_m;
  payload.raw_encoder_cnt = raw_encoder_cnt;

  const uint32_t ts_us = getCurrentTimestampUs();
  const auto frame = CellNetL2Framing::buildConveyorSpeedFrame(
      self_mac_, node_id_, sequence_++, ts_us, payload, dest_mac);

  const bool ok = transport_.sendFrame(frame.data(), frame.size());
  if (ok) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_frames++;
  }
  return ok;
}

bool CellNetL2Node::sendGantryStatus(uint8_t motion_state, uint8_t active_slot,
                                     float x_pos_mm, float z_pos_mm,
                                     float theta_deg, float last_cycle_time_ms,
                                     uint16_t fault_flags,
                                     const uint8_t dest_mac[6]) {
  if (!initialized_) {
    return false;
  }

  L2GantryStatusPayload payload{};
  payload.motion_state = motion_state;
  payload.active_slot = active_slot;
  payload.fault_flags = fault_flags;
  payload.x_pos_mm = x_pos_mm;
  payload.z_pos_mm = z_pos_mm;
  payload.theta_deg = theta_deg;
  payload.last_cycle_time_ms = last_cycle_time_ms;

  const uint32_t ts_us = getCurrentTimestampUs();
  const auto frame = CellNetL2Framing::buildGantryStatusFrame(
      self_mac_, node_id_, sequence_++, ts_us, payload, dest_mac);

  const bool ok = transport_.sendFrame(frame.data(), frame.size());
  if (ok) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_frames++;
  }
  return ok;
}

bool CellNetL2Node::sendCellCommand(uint8_t command_id, uint8_t param_u8,
                                    uint16_t param_u16, float param_float,
                                    const uint8_t dest_mac[6]) {
  if (!initialized_) {
    return false;
  }

  L2CellCommandPayload payload{};
  payload.command_id = command_id;
  payload.param_u8 = param_u8;
  payload.param_u16 = param_u16;
  payload.param_float = param_float;

  const uint32_t ts_us = getCurrentTimestampUs();
  const auto frame = CellNetL2Framing::buildCellCommandFrame(
      self_mac_, node_id_, sequence_++, ts_us, payload, dest_mac);

  const bool ok = transport_.sendFrame(frame.data(), frame.size());
  if (ok) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_frames++;
  }
  return ok;
}

bool CellNetL2Node::processIncomingFrame(const uint8_t* buffer, size_t length) {
  ParsedFrame parsed{};
  if (!CellNetL2Framing::parseFrame(buffer, length, parsed)) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_invalid_frames++;
    return false;
  }

  // Sequence drop tracking per sender node
  const uint8_t sender = parsed.cell.sender_id;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_frames++;
    if (sender_seen_[sender]) {
      const auto expected_seq =
          static_cast<uint8_t>((last_sender_seq_[sender] + 1) & 0xFF);
      if (parsed.cell.sequence != expected_seq) {
        const uint8_t lost = (parsed.cell.sequence - expected_seq) & 0xFF;
        stats_.rx_sequence_drops += lost;
      }
    } else {
      sender_seen_[sender] = true;
    }
    last_sender_seq_[sender] = parsed.cell.sequence;
  }

  // Dispatch to registered type handler
  const auto type = static_cast<CellMsgType>(parsed.cell.msg_type);
  switch (type) {
    case CellMsgType::HEARTBEAT:
      if (heartbeat_cb_) {
        heartbeat_cb_(parsed.payload.heartbeat, parsed.cell);
      }
      break;
    case CellMsgType::VISION_DETECT:
      if (vision_cb_) {
        vision_cb_(parsed.payload.vision_detect, parsed.cell);
      }
      break;
    case CellMsgType::CONVEYOR_SPEED:
      if (conveyor_cb_) {
        conveyor_cb_(parsed.payload.conveyor_speed, parsed.cell);
      }
      break;
    case CellMsgType::GANTRY_STATUS:
      if (gantry_cb_) {
        gantry_cb_(parsed.payload.gantry_status, parsed.cell);
      }
      break;
    case CellMsgType::CELL_COMMAND:
      if (cmd_cb_) {
        cmd_cb_(parsed.payload.cell_command, parsed.cell);
      }
      break;
    default:
      return false;
  }

  return true;
}

CellNetStats CellNetL2Node::getStats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void CellNetL2Node::resetStats() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_ = {};
  std::memset(sender_seen_, 0, sizeof(sender_seen_));
  std::memset(last_sender_seq_, 0, sizeof(last_sender_seq_));
}

}  // namespace CellNet
