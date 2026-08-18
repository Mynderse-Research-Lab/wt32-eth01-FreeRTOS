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
// WT32-ETH01: GPIO16 gates the LAN8720 external crystal (not a PHY reset line).
// Vendor / ESPHome use PHY address 1 (strapped on-module).
static constexpr gpio_num_t ETH_PHY_OSC_GPIO = GPIO_NUM_16;
// Prefer auto-scan; WT32 modules are usually strapped to address 1.
static constexpr int32_t ETH_PHY_ADDR = ESP_ETH_PHY_ADDR_AUTO;
static constexpr uint32_t ETH_PHY_OSC_SETTLE_MS = 1000;

static void probeLan8720ViaMdio(esp_eth_mac_t* mac) {
    if (mac == nullptr || mac->read_phy_reg == nullptr) {
        return;
    }
    // PHYIDR1/PHYIDR2 = regs 2/3. LAN8720 vendor ID is 0x0007 / 0xC0Fx when alive.
    int found = 0;
    for (uint32_t addr = 0; addr < 32; ++addr) {
        uint32_t id1 = 0;
        uint32_t id2 = 0;
        if (mac->read_phy_reg(mac, addr, 2, &id1) != ESP_OK) {
            continue;
        }
        if (id1 == 0x0000u || id1 == 0xffffu) {
            continue;
        }
        (void)mac->read_phy_reg(mac, addr, 3, &id2);
        ESP_LOGI(TAG, "MDIO probe: addr=%lu PHYIDR1=0x%04lx PHYIDR2=0x%04lx",
                 static_cast<unsigned long>(addr),
                 static_cast<unsigned long>(id1),
                 static_cast<unsigned long>(id2));
        ++found;
    }
    if (found == 0) {
        ESP_LOGW(TAG,
                 "MDIO probe: no PHY ID at any address — LAN8720 not responding "
                 "(crystal on GPIO0 / PHY dead / MDC23-MDIO18). GPIO16 high alone is not enough.");
    }
}

bool EthernetLink::enablePhyOscillator() {
    // WT32-ETH01: GPIO16 enables the LAN8720 50 MHz crystal.
    // On ESP32 + IDF v6, GPIO16 is NOT an RTCIO (rtc_io_num_map[16] == -1).
    // Do not pass this pin as phy reset_gpio_num — the IDF driver would pulse
    // it low and kill the crystal during init (power-up timeout).

    // Already driven high (e.g. early app_main call) — do not gpio_reset_pin
    // (that briefly floats the pad and drops the oscillator).
    if (gpio_get_level(ETH_PHY_OSC_GPIO) == 1) {
        (void)gpio_set_direction(ETH_PHY_OSC_GPIO, GPIO_MODE_INPUT_OUTPUT);
        (void)gpio_set_level(ETH_PHY_OSC_GPIO, 1);
        return true;
    }

    gpio_reset_pin(ETH_PHY_OSC_GPIO);

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ETH_PHY_OSC_GPIO);
    io.mode = GPIO_MODE_INPUT_OUTPUT;  // allow readback of driven level
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PHY osc gpio_config failed: %s", esp_err_to_name(err));
        return false;
    }
    (void)gpio_set_drive_capability(ETH_PHY_OSC_GPIO, GPIO_DRIVE_CAP_3);
    (void)gpio_set_pull_mode(ETH_PHY_OSC_GPIO, GPIO_FLOATING);

    err = gpio_set_level(ETH_PHY_OSC_GPIO, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PHY osc gpio_set_level(1) failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ETH_PHY_OSC_SETTLE_MS));
    const int level = gpio_get_level(ETH_PHY_OSC_GPIO);
    if (level != 1) {
        ESP_LOGW(TAG,
                 "PHY osc GPIO%d readback=%d after drive-high — check short/panel "
                 "on ETH enable; RJ45 LEDs may stay dark",
                 static_cast<int>(ETH_PHY_OSC_GPIO), level);
    } else {
        ESP_LOGI(TAG,
                 "PHY osc enabled on GPIO%d (settle %lu ms)",
                 static_cast<int>(ETH_PHY_OSC_GPIO),
                 static_cast<unsigned long>(ETH_PHY_OSC_SETTLE_MS));
    }
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

    esp_ip4_addr_t ip = {};
    esp_ip4_addr_t gw = {};
    esp_ip4_addr_t mask = {};
    if (esp_netif_str_to_ip4(ETH_STATIC_IP, &ip) != ESP_OK ||
        esp_netif_str_to_ip4(ETH_STATIC_GATEWAY, &gw) != ESP_OK ||
        esp_netif_str_to_ip4(ETH_STATIC_NETMASK, &mask) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid static IP/gw/netmask string (check menuconfig)");
        return false;
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = ip.addr;
    ip_info.gw.addr = gw.addr;
    ip_info.netmask.addr = mask.addr;

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

    if (!enablePhyOscillator()) {
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
    // Crystal enable is handled only by enablePhyOscillator(). Do NOT set
    // reset_gpio_num=16 — IDF pulses that pin low during hw reset and then
    // BMCR power-up times out (MDIO sees no live PHY).
    phy_config.reset_gpio_num = -1;
    phy_config.reset_timeout_ms = 2000;

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    // Explicit WT32 wiring (matches IDF esp32 default; do not use CLK_OUT on 17 —
    // GPIO17 is W5500 MOSI).
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = 0;

    // Extra settle after other bring-up (W5500/SPI) before first MDIO access.
    vTaskDelay(pdMS_TO_TICKS(200));
    (void)gpio_set_level(ETH_PHY_OSC_GPIO, 1);

    ESP_LOGI(TAG,
             "ETH bring-up: reset_gpio=-1 phy_addr=auto osc_gpio=%d level=%d",
             static_cast<int>(ETH_PHY_OSC_GPIO), gpio_get_level(ETH_PHY_OSC_GPIO));

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

    probeLan8720ViaMdio(mac);

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
    ESP_LOGI(TAG, "LAN8720 Ethernet started (phy_addr=auto, RMII CLK_EXT_IN GPIO0)");
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
