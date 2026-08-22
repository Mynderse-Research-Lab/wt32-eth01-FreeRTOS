#ifndef ETHERNET_LINK_H
#define ETHERNET_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"

#include <atomic>

namespace Network {

class EthernetLink {
public:
    EthernetLink() = default;
    ~EthernetLink();

    // WT32-ETH01: GPIO16 enables the LAN8720 external 50 MHz crystal (REFCLK
    // into GPIO0). Call early in boot so the oscillator is up before EMAC
    // MDIO probe; RJ45 link LEDs stay dark while this pin is low.
    static bool enablePhyOscillator();

    bool start();
    bool waitForUp(uint32_t timeout_ms);
    bool isUp() const;
    esp_eth_handle_t getEthHandle() const { return eth_handle_; }

private:
    static void handleEthEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
    static void handleIpEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);

    bool started_ = false;
    std::atomic<bool> link_up_{false};
    std::atomic<bool> phy_link_connected_{false};
    bool handlers_registered_ = false;
    bool configureNetwork();
    bool hasUsableIp() const;
    esp_eth_handle_t eth_handle_ = nullptr;
    esp_eth_netif_glue_handle_t eth_glue_ = nullptr;
    esp_netif_t* netif_ = nullptr;
};

}  // namespace Network

#endif  // ETHERNET_LINK_H
