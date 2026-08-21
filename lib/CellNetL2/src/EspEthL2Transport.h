/**
 * @file EspEthL2Transport.h
 * @brief ESP-IDF LAN8720 Ethernet MAC transport driver for Layer-2 frames.
 */

#ifndef ESP_ETH_L2_TRANSPORT_H
#define ESP_ETH_L2_TRANSPORT_H

#ifdef ESP_PLATFORM

#include "IL2Transport.h"
#include "esp_err.h"
#include "esp_eth_driver.h"

namespace CellNet {

class EspEthL2Transport : public IL2Transport {
 public:
  explicit EspEthL2Transport(esp_eth_handle_t eth_handle = nullptr);
  ~EspEthL2Transport() override = default;

  /**
   * @brief Bind the transport to an initialized ESP-IDF Ethernet handle.
   */
  esp_err_t attachEthHandle(esp_eth_handle_t eth_handle);

  // IL2Transport interface overrides
  bool sendFrame(const uint8_t* data, size_t length) override;
  bool getMacAddress(uint8_t mac_out[6]) const override;
  bool isLinkUp() const override;
  void setRxCallback(RxFrameCallback callback) override;

 private:
  esp_eth_handle_t eth_handle_{nullptr};
  RxFrameCallback rx_callback_{nullptr};
  bool hooked_{false};

  static esp_err_t rxHook(esp_eth_handle_t hdl, uint8_t* buffer,
                          uint32_t length, void* priv);
};

}  // namespace CellNet

#endif  // ESP_PLATFORM

#endif  // ESP_ETH_L2_TRANSPORT_H
