#include "W5500.h"
#include "W5500BusMutex.h"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <cstring>
#include <mutex>

static const char* TAG = "W5500";

static constexpr uint16_t kMaxFrameSize = 1536;
static constexpr uint16_t kHeaderSize   = 3;
static constexpr uint16_t kMaxBurstPayload = 2048;
static constexpr uint16_t kSpiScratchSize = kHeaderSize + kMaxBurstPayload;

// Static DMA-capable SPI burst scratch (no per-call heap under Class 1 RPI).
#ifdef ESP_PLATFORM
DMA_ATTR static uint8_t g_spi_tx_scratch[kSpiScratchSize];
DMA_ATTR static uint8_t g_spi_rx_scratch[kSpiScratchSize];
#else
static uint8_t g_spi_tx_scratch[kSpiScratchSize];
static uint8_t g_spi_rx_scratch[kSpiScratchSize];
#endif

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
        vTaskDelay(pdMS_TO_TICKS(1));
        cfg_.rst_set_level(cfg_.rst_ctx, 1);
        vTaskDelay(pdMS_TO_TICKS(55));
        ESP_LOGI(TAG, "Hardware reset complete (external RST callback)");
        return;
    }

    if (cfg_.rst_gpio < 0) {
        // Use software reset
        writeReg(kBlockCommon(), REG_MR, MR_RST);
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << cfg_.rst_gpio);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Hold RSTn low >= 500 us
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst_gpio), 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst_gpio), 1);

    // Wait >= 50 ms before SPI communication
    vTaskDelay(pdMS_TO_TICKS(55));
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
    busCfg.max_transfer_sz = kMaxFrameSize + kHeaderSize;

    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(cfg_.spi_host),
                                       &busCfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
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
    uint8_t ver = getVersion();
    if (ver != 0x04) {
        ESP_LOGE(TAG, "Unexpected W5500 version: 0x%02X (expected 0x04)", ver);
        return false;
    }
    ESP_LOGI(TAG, "W5500 version 0x%02X confirmed", ver);

    // Write a valid MAC to SHAR for hardware TCP/UDP sockets (ARP needs it).
    // Locally-administered unicast: 02:00:00:00:00:01 (unique on the daisy-chain).
    {
        uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
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

    std::lock_guard<std::mutex> lock(w5500::spiBusMutex());

    ESP_LOGW(TAG, "W5500 recover: closing sockets + hard reset");
    for (uint8_t sock = 0; sock < 8; ++sock) {
        writeReg(kBlockSocketReg(sock), Sn_CR, Sn_CR_CLOSE);
    }

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

// ==========================================================================
// Register I/O
// ==========================================================================

uint8_t W5500::readReg(uint8_t blockSelect, uint16_t addr) {
    uint8_t txBuf[4] = {
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        makeCtrlByte(blockSelect, kRwRead),
        0x00  // dummy byte to clock out the response
    };
    uint8_t rxBuf[4] = {};

    spiTransfer(txBuf, rxBuf, sizeof(txBuf));
    return rxBuf[3];
}

void W5500::writeReg(uint8_t blockSelect, uint16_t addr, uint8_t value) {
    uint8_t txBuf[4] = {
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        makeCtrlByte(blockSelect, kRwWrite),
        value
    };
    spiTransfer(txBuf, nullptr, sizeof(txBuf));
}

uint16_t W5500::readReg16(uint8_t blockSelect, uint16_t addr) {
    uint8_t txBuf[5] = {
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        makeCtrlByte(blockSelect, kRwRead),
        0x00, 0x00
    };
    uint8_t rxBuf[5] = {};
    spiTransfer(txBuf, rxBuf, sizeof(txBuf));
    return static_cast<uint16_t>((rxBuf[3] << 8) | rxBuf[4]);
}

void W5500::writeReg16(uint8_t blockSelect, uint16_t addr, uint16_t value) {
    uint8_t txBuf[5] = {
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        makeCtrlByte(blockSelect, kRwWrite),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    spiTransfer(txBuf, nullptr, sizeof(txBuf));
}

void W5500::readBuf(uint8_t blockSelect, uint16_t addr, uint8_t* buf, uint16_t len) {
    if (len > kMaxBurstPayload) {
        ESP_LOGE(TAG, "readBuf: len %u > %u", (unsigned)len, (unsigned)kMaxBurstPayload);
        return;
    }
    const uint16_t totalLen = static_cast<uint16_t>(kHeaderSize + len);
    g_spi_tx_scratch[0] = static_cast<uint8_t>(addr >> 8);
    g_spi_tx_scratch[1] = static_cast<uint8_t>(addr & 0xFF);
    g_spi_tx_scratch[2] = makeCtrlByte(blockSelect, kRwRead);
    std::memset(g_spi_tx_scratch + kHeaderSize, 0x00, len);

    spiTransfer(g_spi_tx_scratch, g_spi_rx_scratch, totalLen);
    std::memcpy(buf, g_spi_rx_scratch + kHeaderSize, len);
}

void W5500::writeBuf(uint8_t blockSelect, uint16_t addr, const uint8_t* buf, uint16_t len) {
    if (len > kMaxBurstPayload) {
        ESP_LOGE(TAG, "writeBuf: len %u > %u", (unsigned)len, (unsigned)kMaxBurstPayload);
        return;
    }
    const uint16_t totalLen = static_cast<uint16_t>(kHeaderSize + len);
    g_spi_tx_scratch[0] = static_cast<uint8_t>(addr >> 8);
    g_spi_tx_scratch[1] = static_cast<uint8_t>(addr & 0xFF);
    g_spi_tx_scratch[2] = makeCtrlByte(blockSelect, kRwWrite);
    std::memcpy(g_spi_tx_scratch + kHeaderSize, buf, len);

    spiTransfer(g_spi_tx_scratch, nullptr, totalLen);
}

// ==========================================================================
// Socket register wrappers
// ==========================================================================

uint8_t W5500::readSocketReg(uint8_t sock, uint16_t addr) {
    return readReg(kBlockSocketReg(sock), addr);
}

void W5500::writeSocketReg(uint8_t sock, uint16_t addr, uint8_t value) {
    writeReg(kBlockSocketReg(sock), addr, value);
}

uint16_t W5500::readSocketReg16(uint8_t sock, uint16_t addr) {
    return readReg16(kBlockSocketReg(sock), addr);
}

void W5500::writeSocketReg16(uint8_t sock, uint16_t addr, uint16_t value) {
    writeReg16(kBlockSocketReg(sock), addr, value);
}

// ==========================================================================
// MACRAW Socket Operations
// ==========================================================================

bool W5500::openMacRaw(uint8_t sock) {
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }

    // Close first to get to a known state
    closeSocket(sock);

    // Set MACRAW mode
    writeSocketReg(sock, Sn_MR, Sn_MR_MACRAW);

    // Issue OPEN command
    writeSocketReg(sock, Sn_CR, Sn_CR_OPEN);

    if (!waitForCommand(sock, 100)) {
        ESP_LOGE(TAG, "MACRAW OPEN command timed out for socket %d", sock);
        return false;
    }

    uint8_t status = readSocketReg(sock, Sn_SR);
    if (status != SOCK_MACRAW) {
        ESP_LOGE(TAG, "Socket %d status 0x%02X, expected SOCK_MACRAW (0x%02X)",
                 sock, status, SOCK_MACRAW);
        return false;
    }

    ESP_LOGI(TAG, "Socket %d opened in MACRAW mode", sock);
    return true;
}

void W5500::closeSocket(uint8_t sock) {
    if (!initialized_) return;

    writeSocketReg(sock, Sn_CR, Sn_CR_CLOSE);
    waitForCommand(sock, 100);
}

// ==========================================================================
// Frame I/O
// ==========================================================================

int W5500::sendFrame(const uint8_t* data, uint16_t len) {
    if (!initialized_) return -1;
    if (len > kMaxFrameSize) return -1;

    const uint8_t sock = 0;

    // Wait for TX buffer space
    uint32_t deadline = esp_timer_get_time() + 100000;  // 100 ms timeout
    while (esp_timer_get_time() < deadline) {
        uint16_t freeSize = readSocketReg16(sock, Sn_TX_FSR);
        if (freeSize >= len) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint16_t freeSize = readSocketReg16(sock, Sn_TX_FSR);
    if (freeSize < len) {
        ESP_LOGE(TAG, "TX buffer full: need %d, have %d", len, freeSize);
        return -1;
    }

    // Write frame to TX buffer
    writeBuf(kBlockSocketTxBuf(sock), 0, data, len);

    // Update TX write pointer
    uint16_t txWr = readSocketReg16(sock, Sn_TX_WR);
    txWr += len;
    writeSocketReg16(sock, Sn_TX_WR, txWr);

    // Issue SEND command
    writeSocketReg(sock, Sn_CR, Sn_CR_SEND);

    // Wait for SENDOK interrupt (or timeout)
    uint32_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < 100000) {
        uint8_t ir = readSocketReg(sock, Sn_IR);
        if (ir & Sn_IR_SENDOK) {
            writeSocketReg(sock, Sn_IR, Sn_IR_SENDOK);  // Clear interrupt
            return len;
        }
        if (ir & Sn_IR_TIMEOUT) {
            writeSocketReg(sock, Sn_IR, Sn_IR_TIMEOUT);
            ESP_LOGE(TAG, "TX timeout (ARP or link issue)");
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGE(TAG, "SEND timed out");
    return -1;
}

int W5500::recvFrame(uint8_t* buf, uint16_t bufSize) {
    if (!initialized_) return 0;

    const uint8_t sock = 0;
    uint16_t rxSize = readSocketReg16(sock, Sn_RX_RSR);

    if (rxSize == 0) return 0;

    // MACRAW RX data starts with 2-byte packet length
    if (rxSize < 2) {
        // Malformed, discard
        writeSocketReg(sock, Sn_CR, Sn_CR_RECV);
        return 0;
    }

    uint8_t lenBuf[2];
    readBuf(kBlockSocketRxBuf(sock), 0, lenBuf, 2);
    writeSocketReg(sock, Sn_CR, Sn_CR_RECV);

    uint16_t frameLen = static_cast<uint16_t>((lenBuf[0] << 8) | lenBuf[1]) - 2;
    if (frameLen > bufSize) {
        // Frame too large for buffer, discard the rest
        ESP_LOGW(TAG, "RX frame too large: %d > %d, discarding", frameLen, bufSize);

        // Discard remaining bytes
        uint8_t discard[64];
        uint16_t remaining = frameLen;
        while (remaining > 0) {
            uint16_t chunk = (remaining > sizeof(discard)) ? sizeof(discard) : remaining;
            readBuf(kBlockSocketRxBuf(sock), 0, discard, chunk);
            writeSocketReg(sock, Sn_CR, Sn_CR_RECV);
            remaining -= chunk;
        }
        return 0;
    }

    if (frameLen > 0) {
        readBuf(kBlockSocketRxBuf(sock), 0, buf, frameLen);
        writeSocketReg(sock, Sn_CR, Sn_CR_RECV);
    }

    return frameLen;
}

// ==========================================================================
// Utility
// ==========================================================================

uint8_t W5500::getVersion() {
    return readReg(kBlockCommon(), REG_VERSIONR);
}

void W5500::getMacAddress(uint8_t mac[6]) {
    readBuf(kBlockCommon(), REG_SHAR, mac, 6);
}

bool W5500::isLinkUp() {
    // PHYCFGR register is at offset 0x002E in common block
    // Bit 0 = LNK (link status)
    uint8_t phyCfgr = readReg(kBlockCommon(), 0x002E);
    return (phyCfgr & 0x01) != 0;
}

// ==========================================================================
// Private helpers
// ==========================================================================

bool W5500::waitForCommand(uint8_t sock, uint32_t timeoutMs) {
    uint32_t start = esp_timer_get_time();
    const uint32_t timeoutUs = timeoutMs * 1000;

    while ((esp_timer_get_time() - start) < timeoutUs) {
        uint8_t cr = readSocketReg(sock, Sn_CR);
        if (cr == 0x00) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool W5500::acquireBus() {
#ifdef ESP_PLATFORM
    if (spiHandle_ == nullptr || !initialized_) {
        return false;
    }
    if (bus_acquired_) {
        return true;
    }
    const esp_err_t err = spi_device_acquire_bus(
        reinterpret_cast<spi_device_handle_t>(spiHandle_), portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI acquire_bus failed: %s", esp_err_to_name(err));
        return false;
    }
    bus_acquired_ = true;
    return true;
#else
    bus_acquired_ = true;
    return true;
#endif
}

void W5500::releaseBus() {
#ifdef ESP_PLATFORM
    if (spiHandle_ == nullptr || !bus_acquired_) {
        bus_acquired_ = false;
        return;
    }
    spi_device_release_bus(reinterpret_cast<spi_device_handle_t>(spiHandle_));
#endif
    bus_acquired_ = false;
}

void W5500::spiTransfer(const uint8_t* txData, uint8_t* rxData, size_t len) {
#ifdef ESP_PLATFORM
    if (len == 0 || txData == nullptr) {
        return;
    }

    spi_transaction_t trans = {};
    trans.length = len * 8;  // in bits
    trans.rxlength = (rxData != nullptr) ? (len * 8) : 0;

    // ≤4 bytes: use TXDATA/RXDATA — no DMA priv-buffer malloc (Class 1 hot path).
    if (len <= 4) {
        trans.flags = SPI_TRANS_USE_TXDATA;
        std::memcpy(trans.tx_data, txData, len);
        if (rxData != nullptr) {
            trans.flags |= SPI_TRANS_USE_RXDATA;
        }
        const esp_err_t err = spi_device_polling_transmit(
            reinterpret_cast<spi_device_handle_t>(spiHandle_), &trans);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(err));
            return;
        }
        if (rxData != nullptr) {
            std::memcpy(rxData, trans.rx_data, len);
        }
        return;
    }

    // Larger transfers: ensure DMA-capable buffers (scratch is DMA_ATTR).
    const uint8_t* tx = txData;
    uint8_t* rx = rxData;
    if (tx != g_spi_tx_scratch) {
        if (len > kSpiScratchSize) {
            ESP_LOGE(TAG, "SPI TX len %u > scratch", static_cast<unsigned>(len));
            return;
        }
        std::memcpy(g_spi_tx_scratch, txData, len);
        tx = g_spi_tx_scratch;
    }
    if (rx != nullptr && rx != g_spi_rx_scratch) {
        if (len > kSpiScratchSize) {
            ESP_LOGE(TAG, "SPI RX len %u > scratch", static_cast<unsigned>(len));
            return;
        }
        rx = g_spi_rx_scratch;
    }
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;

    // Polling transmit: Class 1 already busy-waits on SENDOK/CR; the interrupt
    // path (queue + ISR + semaphore) cost ~40-60 us per tiny register access.
    const esp_err_t err = spi_device_polling_transmit(
        reinterpret_cast<spi_device_handle_t>(spiHandle_), &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(err));
        return;
    }
    if (rxData != nullptr && rxData != g_spi_rx_scratch && rx == g_spi_rx_scratch) {
        std::memcpy(rxData, g_spi_rx_scratch, len);
    }
#else
    (void)txData;
    (void)rxData;
    (void)len;
#endif
}
