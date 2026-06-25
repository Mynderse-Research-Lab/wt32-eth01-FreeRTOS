#ifndef MQTT_BRIDGE_ETHERNET_LINK_H
#define MQTT_BRIDGE_ETHERNET_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"

namespace MqttBridge {

class EthernetLink {
public:
    EthernetLink() = default;
    ~EthernetLink();

    bool start();
    bool waitForUp(uint32_t timeout_ms);
    bool isUp() const;

private:
    static void handleEthEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
    static void handleIpEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);

    bool started_ = false;
    bool link_up_ = false;
    bool phy_link_connected_ = false;
    bool handlers_registered_ = false;
    bool configureNetwork();
    bool hasUsableIp() const;
    esp_eth_handle_t eth_handle_ = nullptr;
    esp_eth_netif_glue_handle_t eth_glue_ = nullptr;
    esp_netif_t* netif_ = nullptr;
};

}  // namespace MqttBridge

#endif  // MQTT_BRIDGE_ETHERNET_LINK_H
