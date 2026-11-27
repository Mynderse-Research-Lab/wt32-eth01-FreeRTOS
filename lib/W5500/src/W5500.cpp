#include "W5500.h"
#include "W5500Internal.h"
#include "W5500BusMutex.h"
#include "W5500Diag.h"
#include "W5500Socket.h"

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
    if (!cfg_.use_dma && len > kNoDmaChunkPayload) {
        // Each piece is a complete VDM frame (own header, own CS cycle).
        for (uint16_t off = 0; off < len; off = static_cast<uint16_t>(off + kNoDmaChunkPayload)) {
            const uint16_t n = (len - off > kNoDmaChunkPayload)
                                   ? kNoDmaChunkPayload
                                   : static_cast<uint16_t>(len - off);
            readBuf(blockSelect, static_cast<uint16_t>(addr + off), buf + off, n);
        }
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
    if (!cfg_.use_dma && len > kNoDmaChunkPayload) {
        for (uint16_t off = 0; off < len; off = static_cast<uint16_t>(off + kNoDmaChunkPayload)) {
            const uint16_t n = (len - off > kNoDmaChunkPayload)
                                   ? kNoDmaChunkPayload
                                   : static_cast<uint16_t>(len - off);
            writeBuf(blockSelect, static_cast<uint16_t>(addr + off), buf + off, n);
        }
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

    if (!waitForCommand(sock, 10)) {
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
    waitForCommand(sock, 10);
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
    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
    return readReg(kBlockCommon(), REG_VERSIONR);
}

void W5500::getMacAddress(uint8_t mac[6]) {
    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
    readBuf(kBlockCommon(), REG_SHAR, mac, 6);
}

bool W5500::isLinkUp() {
    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
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

// acquire/release take spiBusMutex(): every W5500 transaction runs under it,
// and ESP-IDF rejects (acquire) or asserts (release) when another task has a
// polling transaction in flight on this device - e.g. a console explicit
// message (ethlink) while the Class 1 scanner holds the bus.
bool W5500::acquireBus() {
#ifdef ESP_PLATFORM
    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
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
    std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
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
