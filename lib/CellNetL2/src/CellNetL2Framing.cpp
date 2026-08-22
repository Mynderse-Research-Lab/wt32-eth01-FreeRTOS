/**
 * @file CellNetL2Framing.cpp
 * @brief Freestanding C++17 wire encoding, decoding, and validation implementation.
 */

#include "CellNetL2Framing.h"

#include <cstring>

namespace CellNet {

namespace {

inline void setMac(uint8_t dest[6], const uint8_t src[6]) {
  if (src != nullptr) {
    std::memcpy(dest, src, 6);
  } else {
    std::memset(dest, 0xFF, 6);  // Default to Broadcast (FF:FF:FF:FF:FF:FF)
  }
}

inline void setEthHeader(L2EthernetHeader& eth, const uint8_t src_mac[6],
                         const uint8_t dest_mac[6]) {
  setMac(eth.dest_mac, dest_mac);
  std::memcpy(eth.src_mac, src_mac, 6);
  // EtherType 0x88B5 stored Big-Endian (Network Byte Order: byte 12 = 0x88, byte 13 = 0xB5)
  const uint16_t ethertype_host = CELL_NET_L2_ETHERTYPE;
  eth.ethertype = static_cast<uint16_t>(((ethertype_host & 0x00FFu) << 8) |
                                        ((ethertype_host & 0xFF00u) >> 8));
}

inline void setCellHeader(L2CellHeader& cell, CellMsgType msg_type,
                          CellNodeId sender_id, uint8_t sequence,
                          uint32_t timestamp_us) {
  cell.version = CELL_NET_L2_VERSION;
  cell.msg_type = static_cast<uint8_t>(msg_type);
  cell.sender_id = static_cast<uint8_t>(sender_id);
  cell.sequence = sequence;
  cell.timestamp_us_low = timestamp_us;
}

}  // namespace

size_t CellNetL2Framing::expectedFrameLength(CellMsgType type) {
  constexpr size_t kHdrLen = sizeof(L2EthernetHeader) + sizeof(L2CellHeader);
  switch (type) {
    case CellMsgType::HEARTBEAT:
      return sizeof(L2HeartbeatFrame);
    case CellMsgType::VISION_DETECT:
      return sizeof(L2VisionDetectFrame);
    case CellMsgType::CONVEYOR_SPEED:
      return sizeof(L2ConveyorSpeedFrame);
    case CellMsgType::GANTRY_STATUS:
      return sizeof(L2GantryStatusFrame);
    case CellMsgType::CELL_COMMAND:
      return sizeof(L2CellCommandFrame);
    default:
      return 0;
  }
}

bool CellNetL2Framing::parseFrame(const uint8_t* buffer, size_t length,
                                  ParsedFrame& out) {
  out.valid = false;
  if (buffer == nullptr) {
    return false;
  }

  constexpr size_t kMinHdrLen =
      sizeof(L2EthernetHeader) + sizeof(L2CellHeader);
  if (length < kMinHdrLen) {
    return false;
  }

  // 1. Verify EtherType (0x88B5 Big-Endian on wire at bytes 12-13)
  const uint16_t raw_ethertype =
      (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
  if (raw_ethertype != CELL_NET_L2_ETHERTYPE) {
    return false;
  }

  // 2. Read Ethernet and Cell headers
  std::memcpy(&out.eth, buffer, sizeof(L2EthernetHeader));
  std::memcpy(&out.cell, buffer + sizeof(L2EthernetHeader), sizeof(L2CellHeader));

  // 3. Verify Version
  if (out.cell.version != CELL_NET_L2_VERSION) {
    return false;
  }

  // 4. Verify Exact Message Length
  const auto msg_type = static_cast<CellMsgType>(out.cell.msg_type);
  const size_t expected_len = expectedFrameLength(msg_type);
  if (expected_len == 0 || length != expected_len) {
    return false;
  }

  // 5. Extract Payload
  const uint8_t* payload_ptr = buffer + kMinHdrLen;
  const size_t payload_len = length - kMinHdrLen;

  switch (msg_type) {
    case CellMsgType::HEARTBEAT:
      std::memcpy(&out.payload.heartbeat, payload_ptr, payload_len);
      break;
    case CellMsgType::VISION_DETECT:
      std::memcpy(&out.payload.vision_detect, payload_ptr, payload_len);
      break;
    case CellMsgType::CONVEYOR_SPEED:
      std::memcpy(&out.payload.conveyor_speed, payload_ptr, payload_len);
      break;
    case CellMsgType::GANTRY_STATUS:
      std::memcpy(&out.payload.gantry_status, payload_ptr, payload_len);
      break;
    case CellMsgType::CELL_COMMAND:
      std::memcpy(&out.payload.cell_command, payload_ptr, payload_len);
      break;
    default:
      return false;
  }

  out.valid = true;
  return true;
}

std::vector<uint8_t> CellNetL2Framing::buildHeartbeatFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2HeartbeatPayload& payload,
    const uint8_t dest_mac[6]) {
  L2HeartbeatFrame frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, CellMsgType::HEARTBEAT, sender_id, sequence, timestamp_us);
  frame.payload = payload;

  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

std::vector<uint8_t> CellNetL2Framing::buildVisionDetectFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2VisionDetectPayload& payload,
    const uint8_t dest_mac[6]) {
  L2VisionDetectFrame frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, CellMsgType::VISION_DETECT, sender_id, sequence, timestamp_us);
  frame.payload = payload;

  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

std::vector<uint8_t> CellNetL2Framing::buildConveyorSpeedFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2ConveyorSpeedPayload& payload,
    const uint8_t dest_mac[6]) {
  L2ConveyorSpeedFrame frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, CellMsgType::CONVEYOR_SPEED, sender_id, sequence, timestamp_us);
  frame.payload = payload;

  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

std::vector<uint8_t> CellNetL2Framing::buildGantryStatusFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2GantryStatusPayload& payload,
    const uint8_t dest_mac[6]) {
  L2GantryStatusFrame frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, CellMsgType::GANTRY_STATUS, sender_id, sequence, timestamp_us);
  frame.payload = payload;

  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

std::vector<uint8_t> CellNetL2Framing::buildCellCommandFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2CellCommandPayload& payload,
    const uint8_t dest_mac[6]) {
  L2CellCommandFrame frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, CellMsgType::CELL_COMMAND, sender_id, sequence, timestamp_us);
  frame.payload = payload;

  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

}  // namespace CellNet
