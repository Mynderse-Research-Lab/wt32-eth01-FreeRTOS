/**
 * @file CellNetL2Framing.h
 * @brief Freestanding C++17 wire encoding, decoding, and validation for Layer-2 frames.
 *
 * Implements byte-exact serialization and parsing for EtherType 0x88B5 without
 * framework or OS dependencies.
 */

#ifndef CELL_NET_L2_FRAMING_H
#define CELL_NET_L2_FRAMING_H

#include "cell_net_l2_protocol.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace CellNet {

/**
 * @brief Parsed Layer-2 frame container.
 */
struct ParsedFrame {
  L2EthernetHeader eth{};
  L2CellHeader cell{};
  union {
    L2HeartbeatPayload heartbeat;
    L2VisionDetectPayload vision_detect;
    L2ConveyorSpeedPayload conveyor_speed;
    L2GantryStatusPayload gantry_status;
    L2CellCommandPayload cell_command;
  } payload{};
  bool valid{false};
};

class CellNetL2Framing {
 public:
  /// Minimum Ethernet frame length on the wire, excluding the 4-byte FCS.
  static constexpr size_t kMinEthFrameLen = 60;
  /// Largest accepted receive length: a minimum frame plus 4 bytes, so a receiver that
  /// keeps the FCS still passes. ESP-IDF strips the FCS before the L2 input hook
  /// (esp_eth_mac_esp_dma.c), so on the WT32 nodes those 4 bytes can carry payload.
  static constexpr size_t kMaxPaddedRxLen = 64;

  /**
   * @brief Retrieve expected total frame length (Ethernet + Cell + Payload) for a message type.
   * @param type Message type enum.
   * @return Size in bytes, or 0 if unknown type.
   */
  static size_t expectedFrameLength(CellMsgType type);

  /**
   * @brief Parse and validate a raw Ethernet frame.
   * @param buffer Raw frame bytes.
   * @param length Total frame length in bytes.
   * @param out Reference to ParsedFrame structure to fill.
   * @return true if frame is valid EtherType 0x88B5, matching version, and its length is
   *         between the message size and kMaxPaddedRxLen (trailing padding is ignored).
   */
  static bool parseFrame(const uint8_t* buffer, size_t length, ParsedFrame& out);

  /**
   * @brief Build a binary Heartbeat frame (Type 0x00).
   */
  static std::vector<uint8_t> buildHeartbeatFrame(
      const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
      uint32_t timestamp_us, const L2HeartbeatPayload& payload,
      const uint8_t dest_mac[6] = nullptr);

  /**
   * @brief Build a binary Vision Detection frame (Type 0x01).
   */
  static std::vector<uint8_t> buildVisionDetectFrame(
      const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
      uint32_t timestamp_us, const L2VisionDetectPayload& payload,
      const uint8_t dest_mac[6] = nullptr);

  /**
   * @brief Build a binary Conveyor Telemetry frame (Type 0x02).
   */
  static std::vector<uint8_t> buildConveyorSpeedFrame(
      const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
      uint32_t timestamp_us, const L2ConveyorSpeedPayload& payload,
      const uint8_t dest_mac[6] = nullptr);

  /**
   * @brief Build a binary Gantry Status frame (Type 0x03).
   */
  static std::vector<uint8_t> buildGantryStatusFrame(
      const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
      uint32_t timestamp_us, const L2GantryStatusPayload& payload,
      const uint8_t dest_mac[6] = nullptr);

  /**
   * @brief Build a binary Cell Command frame (Type 0x04).
   */
  static std::vector<uint8_t> buildCellCommandFrame(
      const uint8_t src_mac[6], CellNodeId sender_id, uint8_t sequence,
      uint32_t timestamp_us, const L2CellCommandPayload& payload,
      const uint8_t dest_mac[6] = nullptr);
};

}  // namespace CellNet

#endif  // CELL_NET_L2_FRAMING_H
