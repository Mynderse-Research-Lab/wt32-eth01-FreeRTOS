/**
 * @file EspEthL2Transport.cpp
 * @brief ESP-IDF LAN8720 Ethernet MAC transport driver implementation.
 */

#ifdef ESP_PLATFORM

#include "EspEthL2Transport.h"
#include "cell_net_l2_protocol.h"

#include "esp_log.h"
#include "esp_mac.h"

#include <cstdlib>
#include <cstring>

static const char* TAG = "EspEthL2";

namespace CellNet {

static esp_err_t (*s_original_eth_input)(esp_eth_handle_t, uint8_t*, uint32_t,
                                         void*) = nullptr;
static EspEthL2Transport* s_active_instance = nullptr;

esp_err_t EspEthL2Transport::rxHook(esp_eth_handle_t hdl, uint8_t* buffer,
                                    uint32_t length, void* priv) {
  if (buffer != nullptr &&
      length >= sizeof(L2EthernetHeader) + sizeof(L2CellHeader)) {
    const uint16_t ethertype =
        (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
    if (ethertype == CELL_NET_L2_ETHERTYPE) {
      if (s_active_instance != nullptr &&
          s_active_instance->rx_callback_ != nullptr) {
        s_active_instance->rx_callback_(buffer, length);
      }
      // Intercepted at Layer 2: free buffer and do not pass to LwIP
      std::free(buffer);
      return ESP_OK;
    }
  }

  if (s_original_eth_input != nullptr) {
    return s_original_eth_input(hdl, buffer, length, priv);
  }
  return ESP_OK;
}

EspEthL2Transport::EspEthL2Transport(esp_eth_handle_t eth_handle)
    : eth_handle_(eth_handle) {
  s_active_instance = this;
}

esp_err_t EspEthL2Transport::attachEthHandle(esp_eth_handle_t eth_handle) {
  if (eth_handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  eth_handle_ = eth_handle;
  s_active_instance = this;

  if (!hooked_) {
    esp_err_t err = esp_eth_update_input_path(eth_handle_, rxHook,
                                              &s_original_eth_input);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to hook Ethernet input path: %d", (int)err);
      return err;
    }
    hooked_ = true;
    ESP_LOGI(TAG, "Attached Layer-2 RX filter hook (EtherType 0x%04X)",
             CELL_NET_L2_ETHERTYPE);
  }

  return ESP_OK;
}

bool EspEthL2Transport::sendFrame(const uint8_t* data, size_t length) {
  if (eth_handle_ == nullptr || data == nullptr || length == 0) {
    return false;
  }

  esp_err_t err = esp_eth_transmit(eth_handle_,
                                   const_cast<uint8_t*>(data), length);
  return (err == ESP_OK);
}

bool EspEthL2Transport::getMacAddress(uint8_t mac_out[6]) const {
  if (mac_out == nullptr) {
    return false;
  }
  return (esp_read_mac(mac_out, ESP_MAC_ETH) == ESP_OK);
}

bool EspEthL2Transport::isLinkUp() const {
  return (eth_handle_ != nullptr);
}

void EspEthL2Transport::setRxCallback(RxFrameCallback callback) {
  rx_callback_ = std::move(callback);
}

}  // namespace CellNet

#endif  // ESP_PLATFORM
