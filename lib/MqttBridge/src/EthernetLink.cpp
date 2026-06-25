#include "EthernetLink.h"

#include "ethernet_app_config.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth_phy_lan87xx.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace MqttBridge {

static const char* TAG = "EthernetLink";
// WT32-ETH01 internal LAN8720 (see WT32_ETH01_PINOUT.md).
static constexpr int ETH_PHY_POWER_GPIO = 16;  // PHY supply enable, not a strap pin
static constexpr int ETH_PHY_ADDR = 1;

static bool enablePhyPower() {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ETH_PHY_POWER_GPIO);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PHY power gpio_config failed: %s", esp_err_to_name(err));
        return false;
    }
    err = gpio_set_level(static_cast<gpio_num_t>(ETH_PHY_POWER_GPIO), 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PHY power enable failed: %s", esp_err_to_name(err));
        return false;
    }
    // LAN8720 needs settle time after rail comes up.
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "PHY power enabled on GPIO%d", ETH_PHY_POWER_GPIO);
    return true;
}

bool EthernetLink::hasUsableIp() const {
    if (netif_ == nullptr) {
        return false;
    }
    esp_netif_ip_info_t info = {};
    if (esp_netif_get_ip_info(netif_, &info) != ESP_OK) {
        return false;
    }
    return info.ip.addr != 0;
}

bool EthernetLink::configureNetwork() {
    if (netif_ == nullptr) {
        return false;
    }

#if ETH_USE_STATIC_IP
    esp_err_t err = esp_netif_dhcpc_stop(netif_);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED) {
        ESP_LOGE(TAG, "esp_netif_dhcpc_stop failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = ESP_IP4TOADDR(ETH_STATIC_IP_OCTET_1,
                                    ETH_STATIC_IP_OCTET_2,
                                    ETH_STATIC_IP_OCTET_3,
                                    ETH_STATIC_IP_OCTET_4);
    ip_info.gw.addr = ESP_IP4TOADDR(ETH_STATIC_GW_OCTET_1,
                                    ETH_STATIC_GW_OCTET_2,
                                    ETH_STATIC_GW_OCTET_3,
                                    ETH_STATIC_GW_OCTET_4);
    ip_info.netmask.addr = ESP_IP4TOADDR(ETH_STATIC_NETMASK_OCTET_1,
                                         ETH_STATIC_NETMASK_OCTET_2,
                                         ETH_STATIC_NETMASK_OCTET_3,
                                         ETH_STATIC_NETMASK_OCTET_4);

    err = esp_netif_set_ip_info(netif_, &ip_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_set_ip_info failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG,
             "Static IP configured (dumb switch / no DHCP): " IPSTR " gw " IPSTR " mask " IPSTR,
             IP2STR(&ip_info.ip),
             IP2STR(&ip_info.gw),
             IP2STR(&ip_info.netmask));
    return true;
#else
    esp_err_t err = esp_netif_dhcpc_start(netif_);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGE(TAG, "esp_netif_dhcpc_start failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "DHCP client started (requires a router or DHCP server on the LAN)");
    return true;
#endif
}

EthernetLink::~EthernetLink() {
    if (eth_handle_ != nullptr) {
        (void)esp_eth_stop(eth_handle_);
    }
    if (handlers_registered_) {
        (void)esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetLink::handleEthEvent);
        (void)esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &EthernetLink::handleIpEvent);
        handlers_registered_ = false;
    }
    if (eth_glue_ != nullptr) {
        (void)esp_eth_del_netif_glue(eth_glue_);
        eth_glue_ = nullptr;
    }
    if (eth_handle_ != nullptr) {
        (void)esp_eth_driver_uninstall(eth_handle_);
        eth_handle_ = nullptr;
    }
    if (netif_ != nullptr) {
        esp_netif_destroy(netif_);
        netif_ = nullptr;
    }
}

bool EthernetLink::start() {
    if (started_) {
        return true;
    }

    // Phase-1 bring-up: initialize networking primitives and keep the link
    // state explicit. LAN8720 driver install/start is the next step.
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }

    if (!enablePhyPower()) {
        return false;
    }

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    netif_ = esp_netif_new(&cfg);
    if (netif_ == nullptr) {
        ESP_LOGE(TAG, "esp_netif_new(ETH) failed");
        return false;
    }

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    // GPIO16 on WT32 is PHY power, not a dedicated reset line.
    phy_config.reset_gpio_num = -1;

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t* phy = esp_eth_phy_new_lan87xx(&phy_config);
    if (mac == nullptr || phy == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate MAC/PHY drivers");
        if (mac != nullptr) {
            mac->del(mac);
        }
        if (phy != nullptr) {
            phy->del(phy);
        }
        return false;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    err = esp_eth_driver_install(&eth_config, &eth_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    eth_glue_ = esp_eth_new_netif_glue(eth_handle_);
    if (eth_glue_ == nullptr) {
        ESP_LOGE(TAG, "esp_eth_new_netif_glue failed");
        return false;
    }

    err = esp_netif_attach(netif_, eth_glue_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_attach failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_netif_set_default_netif(netif_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_set_default_netif failed: %s", esp_err_to_name(err));
        return false;
    }

    if (!configureNetwork()) {
        return false;
    }

    err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetLink::handleEthEvent, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ETH event register failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &EthernetLink::handleIpEvent, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IP event register failed: %s", esp_err_to_name(err));
        return false;
    }
    handlers_registered_ = true;

    err = esp_eth_start(eth_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(err));
        return false;
    }

    started_ = true;
    link_up_ = false;
    ESP_LOGI(TAG, "LAN8720 Ethernet started (PHY_ADDR=%d, RMII CLK out GPIO17 per sdkconfig)",
             ETH_PHY_ADDR);
    return true;
}

bool EthernetLink::waitForUp(uint32_t timeout_ms) {
    if (!started_) {
        return false;
    }
    const TickType_t step = pdMS_TO_TICKS(50);
    const TickType_t max_wait = pdMS_TO_TICKS(timeout_ms);
    TickType_t waited = 0;
    while (!link_up_ && waited < max_wait) {
#if ETH_USE_STATIC_IP
        if (phy_link_connected_ && hasUsableIp()) {
            link_up_ = true;
            ESP_LOGI(TAG, "Static IP ready while PHY link is up");
            break;
        }
#endif
        vTaskDelay(step);
        waited += step;
    }
    if (!link_up_) {
        ESP_LOGE(TAG,
                 "No IP within %lu ms (PHY link=%s). On a dumb switch use static IP; "
                 "check cable, link LED, and that the broker PC is on the same subnet.",
                 static_cast<unsigned long>(timeout_ms),
                 phy_link_connected_ ? "up" : "down");
    }
    return link_up_;
}

bool EthernetLink::isUp() const {
    return link_up_;
}

void EthernetLink::handleEthEvent(void* handler_args, esp_event_base_t, int32_t event_id, void*) {
    auto* self = static_cast<EthernetLink*>(handler_args);
    if (self == nullptr) {
        return;
    }
    if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        self->phy_link_connected_ = false;
#if !ETH_USE_STATIC_IP
        self->link_up_ = false;
#endif
        ESP_LOGW(TAG, "Ethernet PHY link down");
    } else if (event_id == ETHERNET_EVENT_CONNECTED) {
        self->phy_link_connected_ = true;
        ESP_LOGI(TAG, "Ethernet PHY link up");
#if ETH_USE_STATIC_IP
        if (self->hasUsableIp()) {
            self->link_up_ = true;
            ESP_LOGI(TAG, "Static IP active on link-up");
        }
#endif
    }
}

void EthernetLink::handleIpEvent(void* handler_args, esp_event_base_t, int32_t event_id, void* event_data) {
    auto* self = static_cast<EthernetLink*>(handler_args);
    if (self == nullptr || event_id != IP_EVENT_ETH_GOT_IP || event_data == nullptr) {
        return;
    }
    auto* event = static_cast<ip_event_got_ip_t*>(event_data);
    self->link_up_ = true;
#if ETH_USE_STATIC_IP
    ESP_LOGI(TAG, "Ethernet got IP (static): " IPSTR, IP2STR(&event->ip_info.ip));
#else
    ESP_LOGI(TAG, "Ethernet got IP (DHCP): " IPSTR, IP2STR(&event->ip_info.ip));
#endif
}

}  // namespace MqttBridge
