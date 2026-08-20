/**
 * @file CellNetL2.h
 * @brief High-speed OSI Layer-2 Communication Manager for WT32-ETH01.
 */

#ifndef CELL_NET_L2_H
#define CELL_NET_L2_H

#include "cell_net_l2_protocol.h"
#include "esp_err.h"
#include "esp_eth_driver.h"

#include <cstdint>

typedef void (*CellNetVisionCallback)(const L2VisionDetectPayload &payload, uint32_t timestamp_us);
typedef void (*CellNetConveyorCallback)(const L2ConveyorSpeedPayload &payload, uint32_t timestamp_us);
typedef void (*CellNetCommandCallback)(const L2CellCommandPayload &payload);

class CellNetL2 {
 public:
  static CellNetL2 &instance();

  /**
   * @brief Initialize Layer-2 transceiver on top of the active Ethernet EMAC handle.
   * @param eth_handle The active LAN8720 esp_eth driver handle.
   * @param self_node_id Node identifier for this device (e.g. CellNodeId::GANTRY).
   */
  esp_err_t begin(esp_eth_handle_t eth_handle, CellNodeId self_node_id);

  /**
   * @brief Check if Layer-2 link is initialized.
   */
  bool isReady() const { return ready_; }

  /**
   * @brief Register callbacks for incoming peer messages.
   */
  void setVisionCallback(CellNetVisionCallback cb) { vision_cb_ = cb; }
  void setConveyorCallback(CellNetConveyorCallback cb) { conveyor_cb_ = cb; }
  void setCommandCallback(CellNetCommandCallback cb) { cmd_cb_ = cb; }

  /**
   * @brief Send Gantry status telemetry frame over Layer-2 broadcast.
   */
  esp_err_t sendGantryStatus(uint8_t motion_state, uint8_t active_slot,
                             float x_mm, float z_mm, float theta_deg,
                             float cycle_ms, uint16_t fault_flags = 0);

  /**
   * @brief Handle raw incoming Ethernet MAC frame (called from input interceptor).
   */
  bool processIncomingFrame(const uint8_t *buffer, uint32_t length);

 private:
  CellNetL2() = default;
  ~CellNetL2() = default;

  esp_eth_handle_t eth_handle_ = nullptr;
  CellNodeId self_node_id_ = CellNodeId::GANTRY;
  uint8_t self_mac_[6] = {};
  uint8_t sequence_ = 0;
  bool ready_ = false;

  CellNetVisionCallback vision_cb_ = nullptr;
  CellNetConveyorCallback conveyor_cb_ = nullptr;
  CellNetCommandCallback cmd_cb_ = nullptr;
};

#endif  // CELL_NET_L2_H
