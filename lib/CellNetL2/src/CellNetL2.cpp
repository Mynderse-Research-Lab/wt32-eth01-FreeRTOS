/**
 * @file CellNetL2.cpp
 * @brief High-speed OSI Layer-2 Communication Manager implementation.
 */

#include "CellNetL2.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "lwip/def.h"

#include <cstring>

static const char *TAG = "CellNetL2";

static esp_err_t (*s_original_eth_input)(esp_eth_handle_t, uint8_t *, uint32_t, void *) = nullptr;

static esp_err_t cell_net_l2_rx_hook(esp_eth_handle_t hdl, uint8_t *buffer, uint32_t length, void *priv) {
  if (buffer != nullptr && length >= sizeof(L2EthernetHeader) + sizeof(L2CellHeader)) {
    uint16_t ethertype = (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
    if (ethertype == CELL_NET_L2_ETHERTYPE) {
      if (CellNetL2::instance().processIncomingFrame(buffer, length)) {
        // Intercepted and handled at Layer 2: free or return OK without passing to LwIP
        free(buffer);
        return ESP_OK;
      }
    }
  }

  if (s_original_eth_input != nullptr) {
    return s_original_eth_input(hdl, buffer, length, priv);
  }
  return ESP_OK;
}

CellNetL2 &CellNetL2::instance() {
  static CellNetL2 s_instance;
  return s_instance;
}

esp_err_t CellNetL2::begin(esp_eth_handle_t eth_handle, CellNodeId self_node_id) {
  if (eth_handle == nullptr) {
    ESP_LOGE(TAG, "Cannot begin CellNetL2 with null Ethernet handle");
    return ESP_ERR_INVALID_ARG;
  }

  eth_handle_ = eth_handle;
  self_node_id_ = self_node_id;

  esp_err_t err = esp_read_mac(self_mac_, ESP_MAC_ETH);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Could not query MAC address from EFUSE / Ethernet");
  }

  // Hook raw Layer-2 input filter
  err = esp_eth_update_input_path(eth_handle_, cell_net_l2_rx_hook, &s_original_eth_input);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to hook Ethernet input path for Layer-2 frames: %d", (int)err);
    return err;
  }

  ready_ = true;
  ESP_LOGI(TAG, "CellNet Layer-2 transceiver initialized (Node ID 0x%02X, EtherType 0x%04X, MAC %02X:%02X:%02X:%02X:%02X:%02X)",
           static_cast<uint8_t>(self_node_id_), CELL_NET_L2_ETHERTYPE,
           self_mac_[0], self_mac_[1], self_mac_[2], self_mac_[3], self_mac_[4], self_mac_[5]);

  return ESP_OK;
}

esp_err_t CellNetL2::sendGantryStatus(uint8_t motion_state, uint8_t active_slot,
                                     float x_mm, float z_mm, float theta_deg,
                                     float cycle_ms, uint16_t fault_flags) {
  if (!ready_ || eth_handle_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  L2GantryStatusFrame frame;
  // Destination: Broadcast
  memset(frame.eth.dest_mac, 0xFF, 6);
  memcpy(frame.eth.src_mac, self_mac_, 6);
  frame.eth.ethertype = htons(CELL_NET_L2_ETHERTYPE);

  frame.cell.version = CELL_NET_L2_VERSION;
  frame.cell.msg_type = static_cast<uint8_t>(CellMsgType::GANTRY_STATUS);
  frame.cell.sender_id = static_cast<uint8_t>(self_node_id_);
  frame.cell.sequence = sequence_++;
  frame.cell.timestamp_us_low = static_cast<uint32_t>(esp_timer_get_time() & 0xFFFFFFFF);

  frame.payload.motion_state = motion_state;
  frame.payload.active_slot = active_slot;
  frame.payload.fault_flags = fault_flags;
  frame.payload.x_pos_mm = x_mm;
  frame.payload.z_pos_mm = z_mm;
  frame.payload.theta_deg = theta_deg;
  frame.payload.last_cycle_time_ms = cycle_ms;

  esp_err_t err = esp_eth_transmit(eth_handle_, &frame, sizeof(frame));
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "esp_eth_transmit failed: %d", (int)err);
  }
  return err;
}

bool CellNetL2::processIncomingFrame(const uint8_t *buffer, uint32_t length) {
  if (buffer == nullptr || length < sizeof(L2EthernetHeader) + sizeof(L2CellHeader)) {
    return false;
  }

  const auto *cell_hdr = reinterpret_cast<const L2CellHeader *>(buffer + sizeof(L2EthernetHeader));
  if (cell_hdr->version != CELL_NET_L2_VERSION) {
    return false;
  }

  const uint8_t *payload_ptr = buffer + sizeof(L2EthernetHeader) + sizeof(L2CellHeader);
  uint32_t payload_len = length - (sizeof(L2EthernetHeader) + sizeof(L2CellHeader));

  switch (static_cast<CellMsgType>(cell_hdr->msg_type)) {
    case CellMsgType::VISION_DETECT: {
      if (payload_len >= sizeof(L2VisionDetectPayload) && vision_cb_ != nullptr) {
        L2VisionDetectPayload payload;
        memcpy(&payload, payload_ptr, sizeof(payload));
        vision_cb_(payload, cell_hdr->timestamp_us_low);
        return true;
      }
      break;
    }
    case CellMsgType::CONVEYOR_SPEED: {
      if (payload_len >= sizeof(L2ConveyorSpeedPayload) && conveyor_cb_ != nullptr) {
        L2ConveyorSpeedPayload payload;
        memcpy(&payload, payload_ptr, sizeof(payload));
        conveyor_cb_(payload, cell_hdr->timestamp_us_low);
        return true;
      }
      break;
    }
    case CellMsgType::CELL_COMMAND: {
      if (payload_len >= sizeof(L2CellCommandPayload) && cmd_cb_ != nullptr) {
        L2CellCommandPayload payload;
        memcpy(&payload, payload_ptr, sizeof(payload));
        cmd_cb_(payload);
        return true;
      }
      break;
    }
    default:
      break;
  }

  return false;
}
