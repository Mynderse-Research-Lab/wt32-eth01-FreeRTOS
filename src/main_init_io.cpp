/**
 * @file main_init_io.cpp
 * @brief Boot stages before motion: stored parameters and the log UART,
 *        then the LAN8720 crystal, the SPI3 bus, the MCP23S17 and the TFT.
 */

#include "main_internal.h"

#include "AppConfig.h"
#include "EthernetLink.h"
#include "MCP23S17.h"
#include "Spi3Bus.h"
#include "SpiDisplay.h"
#include "UiManager.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "gantry_app_constants.h"
#include "gpio_expander.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

void bootInitConfig() {
    // Initialize NVS storage for runtime application parameters
    // Stored parameters only revert on an operator command, so unreadable NVS
    // is never erased here. 'cfg factory' erases it and starts a new set.
    const esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_OK) {
        Config::AppConfig::instance().init();
    }
    const Config::ConfigSetInfo active_set = Config::AppConfig::instance().activeSet();
    if (active_set.damaged) {
        ESP_LOGE(TAG, "Stored parameter set is damaged: running on compile-time defaults. "
                      "It is kept until 'cfg factory' or a parameter file reload");
    } else if (active_set.valid) {
        ESP_LOGI(TAG, "Parameter set #%lu loaded (%u parameters)",
                 static_cast<unsigned long>(active_set.seq),
                 static_cast<unsigned>(active_set.param_count));
    } else {
        ESP_LOGE(TAG, "Parameter set not stored (NVS %s): running on compile-time defaults, "
                      "nothing persists. 'cfg factory' erases NVS and starts over",
                 esp_err_to_name(nvs_err));
    }

    // cfg log_uart_async (reboot): without a UART driver every log line
    // busy-waits on UART0 at 115200 (~22 ms per 250-char line) while holding
    // stdout. On 2026-10-20 that stalled the Class 1 task 20-40 ms whenever it
    // logged behind a Gantry log burst, and X/theta timed out. With the driver
    // a line is copied into the TX ring and the UART drains it in the
    // background; a full ring still blocks, so it is sized for long bursts.
    if (Config::AppConfig::instance().data().log_uart_async != 0) {
        constexpr int kLogUartTxBuf = 16384;
        const esp_err_t uerr = uart_driver_install(
            static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM), 256, kLogUartTxBuf,
            0, nullptr, 0);
        if (uerr == ESP_OK) {
            uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
            ESP_LOGW(TAG, "Log UART buffered (%d B TX ring, cfg log_uart_async=1)",
                     kLogUartTxBuf);
        } else {
            ESP_LOGW(TAG, "Log UART driver install failed: %s", esp_err_to_name(uerr));
        }
    }
}

void bootInitIo() {
    // Enable WT32 LAN8720 crystal (GPIO16) before other bring-up so REFCLK and
    // RJ45 LEDs can come up; EthernetLink::start() will re-assert if needed.
    if (!Network::EthernetLink::enablePhyOscillator()) {
        ESP_LOGW(TAG, "LAN8720 crystal enable failed — plant ETH / TCP :2323 may be unavailable");
    }

    // Diagnostic electrical line probe for SPI3 lines (CS=GPIO2, MISO=GPIO36)
    {
        gpio_config_t t_cs = {};
        t_cs.pin_bit_mask = (1ULL << SPI3_CS_MCP_GPIO);
        t_cs.mode = GPIO_MODE_INPUT_OUTPUT;
        t_cs.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&t_cs);
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 1);
        int cs_mcp_hi = gpio_get_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 0);
        int cs_mcp_lo = gpio_get_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 1);
        gpio_reset_pin(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));

        gpio_config_t t_miso = {};
        t_miso.pin_bit_mask = (1ULL << SPI3_MISO_GPIO);
        t_miso.mode = GPIO_MODE_INPUT;
        gpio_config(&t_miso);
        int miso36_lvl = gpio_get_level(static_cast<gpio_num_t>(SPI3_MISO_GPIO));
        gpio_reset_pin(static_cast<gpio_num_t>(SPI3_MISO_GPIO));

        ESP_LOGI(TAG, "SPI3 line probe: CS_MCP(GPIO%d) 1->%d 0->%d | MISO(GPIO%d) level=%d",
                 SPI3_CS_MCP_GPIO, cs_mcp_hi, cs_mcp_lo, SPI3_MISO_GPIO, miso36_lvl);
    }

    // SPI3 shared bus (MCP default client) + MCP23S17 Field/UI + ST7789 TFT
    if (!spi3::init()) {
        ESP_LOGW(TAG, "SPI3 bus init failed — continuing without MCP/TFT");
    } else {
        mcp23s17_config_t mcpCfg = {};
        mcpCfg.spi_host = spi3::host();
        mcpCfg.cs_pin = static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO);
        mcpCfg.miso_pin = static_cast<gpio_num_t>(SPI3_MISO_GPIO);
        mcpCfg.mosi_pin = static_cast<gpio_num_t>(SPI3_MOSI_GPIO);
        mcpCfg.sclk_pin = static_cast<gpio_num_t>(SPI3_SCLK_GPIO);
        mcpCfg.device_address = MCP23S17_HW_ADDR;
        mcpCfg.clock_speed_hz = SPI3_MCP_CLOCK_HZ;
        mcpCfg.skip_bus_init = true;
        if (!gpio_expander_init(&mcpCfg)) {
            ESP_LOGW(TAG, "MCP23S17 init failed — Field I/O unavailable");
        } else if (gpio_expander_configure_field_and_ui() != ESP_OK) {
            ESP_LOGW(TAG, "MCP Field/UI pin configure failed");
        } else {
            display::SpiDisplayConfig dispCfg = {};
            dispCfg.mcp = gpio_expander_get_mcp_handle();
            dispCfg.mcp_cs_pin = -1; // Shared hardware CS via ESP32 GPIO2
            dispCfg.esp_cs_pin = SPI3_CS_TFT_GPIO;
            dispCfg.mcp_dc_pin = MCP_TFT_DC;
            dispCfg.mcp_res_pin = MCP_TFT_RES;
            dispCfg.mcp_blk_pin = -1; // Moved to ESP32 for native PWM
            dispCfg.esp_blk_pin = TFT_BLK_GPIO;
            dispCfg.clock_hz = SPI3_TFT_CLOCK_HZ;
            // disp_tft_en=0: Class 1 dropout experiment - no TFT traffic on
            // SPI3 (MCP Field I/O and the gripper still run).
            if (Config::AppConfig::instance().data().display_tft_enable == 0) {
                // The TFT driver normally parks its CS; without it GPIO32
                // floats and the panel can sit selected on the shared SPI3
                // bus (MISO contention with the MCP23S17). Deselect it here.
                gpio_config_t tft_cs = {};
                tft_cs.pin_bit_mask = (1ULL << SPI3_CS_TFT_GPIO);
                tft_cs.mode = GPIO_MODE_OUTPUT;
                gpio_config(&tft_cs);
                gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_TFT_GPIO), 1);
                ESP_LOGW(TAG, "TFT UI disabled (cfg disp_tft_en=0) - no display traffic on SPI3; TFT CS held high");
            } else if (!display::UiManager::instance().init(dispCfg)) {
                ESP_LOGW(TAG, "UiManager ST7789 display begin failed");
            }
        }
    }
}
