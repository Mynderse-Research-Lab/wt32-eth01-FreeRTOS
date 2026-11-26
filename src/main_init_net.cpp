/**
 * @file main_init_net.cpp
 * @brief Boot stage for the plant LAN (LAN8720): the TCP console, the OTA
 *        server, CellNet L2 and the NTP client, once the link is up.
 */

#include "main_internal.h"

#include "AppConfig.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "ethernet_app_config.h"
#include "gantry_net_console.h"
#include "gantry_ota.h"

bool bootStartNetwork(Gantry::Gantry& gantry, GantryTestConsoleConfig& consoleCfg,
                      Network::EthernetLink& ethernetLink,
                      CellNet::EspEthL2Transport& l2Transport,
                      CellNet::CellNetL2Node& l2Node) {
    bool ethUp = false;
    if (ethernetLink.start() && ethernetLink.waitForUp(ETH_IP_WAIT_TIMEOUT_MS)) {
        ethUp = true;
        gantryNetConsoleStart(&consoleCfg);
        ESP_LOGI(TAG, "Net console listening on TCP %d (plant / LAN8720)",
                 CONSOLE_TCP_PORT);
        gantryOtaStartServer(&gantry, 8032, CONSOLE_TCP_PASSWORD);
        ESP_LOGI(TAG, "OTA server listening on TCP 8032 (plant / LAN8720)");

        // Initialize High-Speed OSI Layer-2 Cell Network transceiver
        if (l2Transport.attachEthHandle(ethernetLink.getEthHandle(), ethernetLink.getNetif()) == ESP_OK &&
            l2Node.begin()) {
            ESP_LOGI(TAG, "CellNet OSI Layer-2 bus ACTIVE (EtherType 0x%04X, Node 0x%02X)",
                     CELL_NET_L2_ETHERTYPE, static_cast<uint8_t>(CellNodeId::GANTRY));
        } else {
            ESP_LOGW(TAG, "CellNet Layer-2 initialization failed");
        }
    } else {
#if CONSOLE_UART_ENABLE
        ESP_LOGW(TAG, "LAN8720 not up — UART console only; net console skipped");
#else
        ESP_LOGW(TAG, "LAN8720 not up — net console skipped (no UART fallback)");
#endif
    }

    if (ethUp) {
        esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(
            Config::AppConfig::instance().data().ntp_server);
        if (esp_netif_sntp_init(&sntp) != ESP_OK) {
            ESP_LOGW(TAG, "NTP client did not start; stored sets will show no clock");
        }
    }
    return ethUp;
}
