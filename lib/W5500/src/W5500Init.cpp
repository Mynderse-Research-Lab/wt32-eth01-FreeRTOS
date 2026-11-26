/**
 * @file W5500Init.cpp
 * @brief W5500 reset, init, recover and deinit.
 */

#include "W5500.h"
#include "W5500Internal.h"
#include "W5500BusMutex.h"
#include "W5500Diag.h"
#include "W5500Socket.h"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <mutex>


// ==========================================================================
// Constructor / Destructor
// ==========================================================================

W5500::~W5500() {
    deinit();
}

// ==========================================================================
// Hardware reset
// ==========================================================================

void W5500::hardReset() {
    if (cfg_.rst_set_level != nullptr) {
        cfg_.rst_set_level(cfg_.rst_ctx, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        cfg_.rst_set_level(cfg_.rst_ctx, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        ESP_LOGI(TAG, "Hardware reset complete (external RST callback)");
        return;
    }

    if (cfg_.rst_gpio < 0) {
        // Use software reset
        writeReg(kBlockCommon(), REG_MR, MR_RST);
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << cfg_.rst_gpio);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Hold RSTn low >= 500 us (use 10 ms)
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst_gpio), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst_gpio), 1);

    // Wait >= 50 ms before SPI communication (use 100 ms)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Hardware reset complete (GPIO%d)", cfg_.rst_gpio);
}

// ==========================================================================
// init / deinit
// ==========================================================================

bool W5500::init(const W5500Config& cfg) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    cfg_ = cfg;

    // --- SPI bus init ---
    spi_bus_config_t busCfg = {};
    busCfg.mosi_io_num     = cfg_.mosi_gpio;
    busCfg.miso_io_num     = cfg_.miso_gpio;
    busCfg.sclk_io_num     = cfg_.sclk_gpio;
    busCfg.quadwp_io_num   = -1;
    busCfg.quadhd_io_num   = -1;
    busCfg.max_transfer_sz = cfg_.use_dma ? (kMaxFrameSize + kHeaderSize) : kNoDmaMaxTransfer;

    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(cfg_.spi_host),
                                       &busCfg, cfg_.use_dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }
    if (!cfg_.use_dma) {
        ESP_LOGW(TAG, "SPI bus without DMA: bursts split into %u-byte frames",
                 (unsigned)kNoDmaChunkPayload);
    }

    // --- SPI device ---
    // Cap: ESP-IDF v6 full-duplex GPIO-matrix limit is < 80/3 Hz (~26.67 MHz).
    static constexpr int kMaxFullDuplexHz = 20000000;
    int sclkHz = cfg_.sclk_hz;
    if (sclkHz > kMaxFullDuplexHz) {
        ESP_LOGW(TAG, "SPI clock %d Hz capped to %d Hz (GPIO-matrix full-duplex)",
                 sclkHz, kMaxFullDuplexHz);
        sclkHz = kMaxFullDuplexHz;
        cfg_.sclk_hz = sclkHz;
    }

    spi_device_interface_config_t devCfg = {};
    devCfg.mode           = 0;          // SPI Mode 0 (CPOL=0, CPHA=0)
    devCfg.clock_speed_hz = sclkHz;
    // VDM (OM=00) needs CS edges per frame; cs_gpio < 0 is only for FDM experiments.
    devCfg.spics_io_num   = (cfg_.cs_gpio >= 0) ? cfg_.cs_gpio : -1;
    devCfg.queue_size     = 1;
    devCfg.flags          = 0;
    devCfg.cs_ena_pretrans = 1;
    devCfg.cs_ena_posttrans = 1;

    err = spi_bus_add_device(static_cast<spi_host_device_t>(cfg_.spi_host),
                             &devCfg, reinterpret_cast<spi_device_handle_t*>(&spiHandle_));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        // Do not spi_bus_free here: a failed add can leave the bus lock
        // inconsistent and assert in spi_bus_deinit_lock (IDF v6).
        return false;
    }

    // --- Reset chip ---
    hardReset();

    if (!configureAfterReset()) {
        deinit();
        return false;
    }

    initialized_ = true;
    if (cfg_.cs_gpio >= 0) {
        ESP_LOGI(TAG, "W5500 initialized (SPI%d, CS=GPIO%d, %d Hz)",
                 cfg_.spi_host, cfg_.cs_gpio, cfg_.sclk_hz);
    } else {
        ESP_LOGI(TAG, "W5500 initialized (SPI%d, CS=hardwired, %d Hz)",
                 cfg_.spi_host, cfg_.sclk_hz);
    }
    return true;
}

bool W5500::configureAfterReset() {
    uint8_t ver = 0;
    for (int retry = 0; retry < 10; ++retry) {
        ver = readReg(kBlockCommon(), REG_VERSIONR);
        if (ver == 0x04) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Fallback: try software reset if external reset line didn't bring chip up
    if (ver != 0x04) {
        ESP_LOGW(TAG, "W5500 version read 0x%02X; attempting software reset fallback...", ver);
        writeReg(kBlockCommon(), REG_MR, MR_RST);
        vTaskDelay(pdMS_TO_TICKS(50));
        for (int retry = 0; retry < 5; ++retry) {
            ver = readReg(kBlockCommon(), REG_VERSIONR);
            if (ver == 0x04) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // Multirate fallback: if current clock failed, try stepped-down clock rates (5 MHz, 2 MHz, 1 MHz)
    if (ver != 0x04) {
        const int fallback_rates[] = {5000000, 2000000, 1000000};
        for (int fb_hz : fallback_rates) {
            if (fb_hz >= cfg_.sclk_hz) {
                continue;
            }
            ESP_LOGW(TAG, "W5500 version 0x%02X at %d Hz; retrying at fallback %d Hz...",
                     ver, cfg_.sclk_hz, fb_hz);
            spi_bus_remove_device(static_cast<spi_device_handle_t>(spiHandle_));
            spiHandle_ = nullptr;

            spi_device_interface_config_t devCfg = {};
            devCfg.mode           = 0;
            devCfg.clock_speed_hz = fb_hz;
            devCfg.spics_io_num   = (cfg_.cs_gpio >= 0) ? cfg_.cs_gpio : -1;
            devCfg.queue_size     = 1;
            devCfg.flags          = 0;
            devCfg.cs_ena_pretrans = 1;
            devCfg.cs_ena_posttrans = 1;

            esp_err_t add_err = spi_bus_add_device(static_cast<spi_host_device_t>(cfg_.spi_host),
                                                   &devCfg, reinterpret_cast<spi_device_handle_t*>(&spiHandle_));
            if (add_err != ESP_OK) {
                ESP_LOGE(TAG, "SPI device re-add failed at %d Hz: %s", fb_hz, esp_err_to_name(add_err));
                break;
            }
            cfg_.sclk_hz = fb_hz;
            writeReg(kBlockCommon(), REG_MR, MR_RST);
            vTaskDelay(pdMS_TO_TICKS(50));
            ver = readReg(kBlockCommon(), REG_VERSIONR);
            if (ver == 0x04) {
                ESP_LOGI(TAG, "W5500 communication established at %d Hz!", fb_hz);
                break;
            }
        }
    }

    if (ver != 0x04) {
        ESP_LOGE(TAG, "Unexpected W5500 version: 0x%02X (expected 0x04)", ver);
        return false;
    }
    ESP_LOGI(TAG, "W5500 version 0x%02X confirmed", ver);

    // Write a valid MAC to SHAR for hardware TCP/UDP sockets (ARP needs it).
    // Locally-administered unicast: 02:00:00:00:00:01 (unique on the daisy-chain).
    uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    {
        writeBuf(kBlockCommon(), REG_SHAR, mac, 6);
        ESP_LOGI(TAG, "W5500 MAC set: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

#if CONFIG_EIP_SCANNER_ENABLED
    {
        uint8_t ip[4]   = {0, 0, 0, 0};
        uint8_t sub[4]  = {0, 0, 0, 0};
        uint8_t gw[4]   = {0, 0, 0, 0};

        std::sscanf(CONFIG_EIP_W5500_IP,      "%hhu.%hhu.%hhu.%hhu",
                    &ip[0], &ip[1], &ip[2], &ip[3]);
        std::sscanf(CONFIG_EIP_W5500_SUBNET,  "%hhu.%hhu.%hhu.%hhu",
                    &sub[0], &sub[1], &sub[2], &sub[3]);
        std::sscanf(CONFIG_EIP_W5500_GATEWAY, "%hhu.%hhu.%hhu.%hhu",
                    &gw[0], &gw[1], &gw[2], &gw[3]);

        writeBuf(kBlockCommon(), REG_SIPR, ip, 4);
        writeBuf(kBlockCommon(), REG_SUBR, sub, 4);
        writeBuf(kBlockCommon(), REG_GAR, gw, 4);
        // Reference for the periodic SPI read-back check (W5500Diag).
        w5500::setExpectedIdentity(mac, ip);

        ESP_LOGI(TAG, "W5500 IP: %u.%u.%u.%u / %u.%u.%u.%u gw %u.%u.%u.%u",
                 ip[0], ip[1], ip[2], ip[3],
                 sub[0], sub[1], sub[2], sub[3],
                 gw[0], gw[1], gw[2], gw[3]);
    }
#endif

    // Bias buffers toward Class 1 UDP (prefer sock0 after reset): 4 KB RX/TX.
    // FO TCP peers get 2 KB; remaining sockets 1 KB. Sum RX=4+2+2+2+1+1+1+1=14.
    static const uint8_t kRxKb[8] = {4, 2, 2, 2, 1, 1, 1, 1};
    static const uint8_t kTxKb[8] = {4, 2, 2, 2, 1, 1, 1, 1};
    w5500::setExpectedBufferSizes(kRxKb, kTxKb);  // W5500Diag snapshot reference
    for (uint8_t sock = 0; sock < 8; ++sock) {
        writeReg(kBlockSocketReg(sock), Sn_RXBUF_SIZE, kRxKb[sock]);
        writeReg(kBlockSocketReg(sock), Sn_TXBUF_SIZE, kTxKb[sock]);
    }

    for (uint8_t sock = 0; sock < 8; ++sock) {
        writeReg(kBlockSocketReg(sock), Sn_CR, Sn_CR_CLOSE);
    }
    return true;
}

bool W5500::recover() {
    if (!initialized_ || spiHandle_ == nullptr) {
        ESP_LOGE(TAG, "recover: not initialized");
        return false;
    }

    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());

    ESP_LOGW(TAG, "W5500 recover: closing sockets + hard reset");
    for (uint8_t sock = 0; sock < 8; ++sock) {
        writeReg(kBlockSocketReg(sock), Sn_CR, Sn_CR_CLOSE);
    }
    w5500::socketResetSoftwareState();

    hardReset();

    if (!configureAfterReset()) {
        ESP_LOGE(TAG, "W5500 recover failed (VERSIONR)");
        return false;
    }

    ESP_LOGI(TAG, "W5500 recover complete");
    return true;
}

void W5500::deinit() {
    if (!initialized_) {
        return;
    }

    releaseBus();
    closeSocket(0);

    if (spiHandle_ != nullptr) {
        spi_bus_remove_device(reinterpret_cast<spi_device_handle_t>(spiHandle_));
        spiHandle_ = nullptr;
    }

    spi_bus_free(static_cast<spi_host_device_t>(cfg_.spi_host));
    initialized_ = false;
    ESP_LOGI(TAG, "W5500 deinitialized");
}
