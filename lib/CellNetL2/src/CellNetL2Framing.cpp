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
    case CellMsgType::OTA_BEGIN:
      return sizeof(L2OtaBeginFrame);
    case CellMsgType::OTA_DATA:
      return sizeof(L2OtaDataFrameHeader);  // minimum: data follows
    case CellMsgType::OTA_STATUS:
      return sizeof(L2OtaStatusFrame);
    case CellMsgType::OTA_END:
      return sizeof(L2OtaEndFrame);
    case CellMsgType::OTA_RESULT:
      return sizeof(L2OtaResultFrame);
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

  // 4. Verify Message Length (Ethernet minimum-size padding may follow the payload)
  const auto msg_type = static_cast<CellMsgType>(out.cell.msg_type);
  const size_t expected_len = expectedFrameLength(msg_type);
  const size_t max_len =
      (msg_type == CellMsgType::OTA_DATA) ? kMaxOtaDataFrameLen : kMaxPaddedRxLen;
  if (expected_len == 0 || length < expected_len || length > max_len) {
    return false;
  }

  // 5. Extract Payload
  const uint8_t* payload_ptr = buffer + kMinHdrLen;
  const size_t payload_len = expected_len - kMinHdrLen;

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
    case CellMsgType::OTA_BEGIN:
      std::memcpy(&out.payload.ota_begin, payload_ptr, payload_len);
      break;
    case CellMsgType::OTA_DATA:
      std::memcpy(&out.payload.ota_data, payload_ptr, payload_len);
      if (out.payload.ota_data.data_len == 0 ||
          expected_len + out.payload.ota_data.data_len > length) {
        return false;
      }
      out.ota_data = buffer + expected_len;
      break;
    case CellMsgType::OTA_STATUS:
      std::memcpy(&out.payload.ota_status, payload_ptr, payload_len);
      break;
    case CellMsgType::OTA_END:
      std::memcpy(&out.payload.ota_end, payload_ptr, payload_len);
      break;
    case CellMsgType::OTA_RESULT:
      std::memcpy(&out.payload.ota_result, payload_ptr, payload_len);
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

namespace {

template <typename FrameT, typename PayloadT>
std::vector<uint8_t> buildFixed(CellMsgType type, const uint8_t src_mac[6],
                                CellNodeId sender_id, uint8_t sequence,
                                uint32_t timestamp_us, const PayloadT& payload,
                                const uint8_t dest_mac[6]) {
  FrameT frame{};
  setEthHeader(frame.eth, src_mac, dest_mac);
  setCellHeader(frame.cell, type, sender_id, sequence, timestamp_us);
  frame.payload = payload;
  const auto* ptr = reinterpret_cast<const uint8_t*>(&frame);
  return std::vector<uint8_t>(ptr, ptr + sizeof(frame));
}

}  // namespace

std::vector<uint8_t> CellNetL2Framing::buildOtaBeginFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2OtaBeginPayload& payload, const uint8_t dest_mac[6]) {
  return buildFixed<L2OtaBeginFrame>(CellMsgType::OTA_BEGIN, src_mac, sender_id, sequence,
                                     timestamp_us, payload, dest_mac);
}

std::vector<uint8_t> CellNetL2Framing::buildOtaDataFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2OtaDataHeader& header, const uint8_t* data,
    const uint8_t dest_mac[6]) {
  std::vector<uint8_t> frame = buildFixed<L2OtaDataFrameHeader>(
      CellMsgType::OTA_DATA, src_mac, sender_id, sequence, timestamp_us, header, dest_mac);
  if (data != nullptr && header.data_len > 0) {
    frame.insert(frame.end(), data, data + header.data_len);
  }
  return frame;
}

std::vector<uint8_t> CellNetL2Framing::buildOtaStatusFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2OtaStatusPayload& payload, const uint8_t dest_mac[6]) {
  return buildFixed<L2OtaStatusFrame>(CellMsgType::OTA_STATUS, src_mac, sender_id, sequence,
                                      timestamp_us, payload, dest_mac);
}

std::vector<uint8_t> CellNetL2Framing::buildOtaEndFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2OtaEndPayload& payload, const uint8_t dest_mac[6]) {
  return buildFixed<L2OtaEndFrame>(CellMsgType::OTA_END, src_mac, sender_id, sequence,
                                   timestamp_us, payload, dest_mac);
}

std::vector<uint8_t> CellNetL2Framing::buildOtaResultFrame(
    const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
    uint32_t timestamp_us, const L2OtaResultPayload& payload, const uint8_t dest_mac[6]) {
  return buildFixed<L2OtaResultFrame>(CellMsgType::OTA_RESULT, src_mac, sender_id, sequence,
                                      timestamp_us, payload, dest_mac);
}

}  // namespace CellNet
