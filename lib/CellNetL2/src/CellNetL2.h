/**
 * @file CellNetL2.h
 * @brief High-speed, reusable OSI Layer-2 Communication Node for cell subsystems.
 *
 * Implements subsystem-agnostic transmission, callback dispatching, and link
 * statistics over the abstract IL2Transport interface.
 */

#ifndef CELL_NET_L2_H
#define CELL_NET_L2_H

#include "CellNetL2Framing.h"
#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>

namespace CellNet {

using HeartbeatCallback =
    std::function<void(const L2HeartbeatPayload& payload, const L2CellHeader& header)>;
using VisionDetectCallback =
    std::function<void(const L2VisionDetectPayload& payload, const L2CellHeader& header)>;
using ConveyorSpeedCallback =
    std::function<void(const L2ConveyorSpeedPayload& payload, const L2CellHeader& header)>;
using GantryStatusCallback =
    std::function<void(const L2GantryStatusPayload& payload, const L2CellHeader& header)>;
using CellCommandCallback =
    std::function<void(const L2CellCommandPayload& payload, const L2CellHeader& header)>;

struct CellNetStats {
  uint32_t tx_frames = 0;
  uint32_t rx_frames = 0;
  uint32_t rx_invalid_frames = 0;
  uint32_t rx_sequence_drops = 0;
};

class CellNetL2Node {
 public:
  explicit CellNetL2Node(IL2Transport& transport,
                         CellNodeId node_id = CellNodeId::UNKNOWN);
  ~CellNetL2Node() = default;

  // Non-copyable
  CellNetL2Node(const CellNetL2Node&) = delete;
  CellNetL2Node& operator=(const CellNetL2Node&) = delete;

  /**
   * @brief Initialize Layer-2 node, bind RX callbacks, and query local MAC.
   */
  bool begin();

  /**
   * @brief Check if transport link is up and node is ready.
   */
  bool isReady() const;

  CellNodeId getNodeId() const { return node_id_; }
  void setNodeId(CellNodeId node_id) { node_id_ = node_id; }

  // --------------------------------------------------------------------------
  // Message Transmission Methods (Subsystem-Agnostic)
  // --------------------------------------------------------------------------

  bool sendHeartbeat(uint32_t uptime_ms, uint16_t status_flags = 0,
                     const uint8_t dest_mac[6] = nullptr);

  bool sendVisionDetect(uint32_t item_id, float x_across_mm, float y_bat_mm,
                        float theta_deg, uint8_t battery_class,
                        const uint8_t dest_mac[6] = nullptr);

  bool sendConveyorSpeed(float speed_mm_s, float displacement_m,
                         int32_t raw_encoder_cnt,
                         const uint8_t dest_mac[6] = nullptr);

  bool sendGantryStatus(uint8_t motion_state, uint8_t active_slot,
                        float x_pos_mm, float z_pos_mm, float theta_deg,
                        float last_cycle_time_ms, uint16_t fault_flags = 0,
                        const uint8_t dest_mac[6] = nullptr);

  bool sendCellCommand(uint8_t command_id, uint8_t param_u8 = 0,
                       uint16_t param_u16 = 0, float param_float = 0.0f,
                       const uint8_t dest_mac[6] = nullptr);

  // --------------------------------------------------------------------------
  // Callback Handlers Registration
  // --------------------------------------------------------------------------

  void onHeartbeat(HeartbeatCallback cb) { heartbeat_cb_ = std::move(cb); }
  void onVisionDetect(VisionDetectCallback cb) { vision_cb_ = std::move(cb); }
  void onConveyorSpeed(ConveyorSpeedCallback cb) { conveyor_cb_ = std::move(cb); }
  void onGantryStatus(GantryStatusCallback cb) { gantry_cb_ = std::move(cb); }
  void onCellCommand(CellCommandCallback cb) { cmd_cb_ = std::move(cb); }

  // --------------------------------------------------------------------------
  // Reception & Diagnostics
  // --------------------------------------------------------------------------

  /**
   * @brief Process an incoming raw Ethernet frame.
   * @return true if frame was a valid 0x88B5 message and dispatched.
   */
  bool processIncomingFrame(const uint8_t* buffer, size_t length);

  CellNetStats getStats() const;
  void resetStats();

 private:
  IL2Transport& transport_;
  CellNodeId node_id_;
  uint8_t self_mac_[6]{};
  uint8_t sequence_{0};
  bool initialized_{false};

  mutable std::mutex stats_mutex_;
  CellNetStats stats_{};

  // Track expected sequence numbers per sender node (256 slots)
  uint8_t last_sender_seq_[256]{};
  bool sender_seen_[256]{};

  HeartbeatCallback heartbeat_cb_{nullptr};
  VisionDetectCallback vision_cb_{nullptr};
  ConveyorSpeedCallback conveyor_cb_{nullptr};
  GantryStatusCallback gantry_cb_{nullptr};
  CellCommandCallback cmd_cb_{nullptr};
};

}  // namespace CellNet

#endif  // CELL_NET_L2_H
