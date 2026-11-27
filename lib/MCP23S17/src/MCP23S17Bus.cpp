/**
 * @file MCP23S17Bus.cpp
 * @brief MCP23S17 lock, holder logging and register access.
 *
 * One of the MCP23S17 driver files; MCP23S17Internal.h holds what they share.
 */

#include "MCP23S17Internal.h"

#include <string.h>
#include "freertos/task.h"
#include "esp_timer.h"
#include "Spi3Bus.h"

static constexpr TickType_t kMcpSpiLockTimeoutTicks = pdMS_TO_TICKS(8);
static constexpr uint32_t kMcpClass1WaitMs = 8;
// Re-checks if Class 1 re-entered between the idle wake-up and the mutex.
static constexpr int kMcpClass1RetryAttempts = 4;

// Why the last mcp23s17_lock() failed (for the error log).
static const char* s_lock_fail = "";

static inline void mcp23s17_note_holder(mcp23s17_handle_t handle) {
    handle->held_since_us = esp_timer_get_time();
    const char *name = pcTaskGetName(NULL);
    strncpy(handle->holder, name != NULL ? name : "?", sizeof(handle->holder) - 1);
    handle->holder[sizeof(handle->holder) - 1] = '\0';
}

// Who holds spi_mutex now and for how long, logged when a lock times out.
static void mcp23s17_log_holder(mcp23s17_handle_t handle) {
    const int64_t since = handle->held_since_us;
    if (since != 0) {
        ESP_LOGE(TAG, "  spi_mutex held by %s for %lld us so far", handle->holder,
                 (long long)(esp_timer_get_time() - since));
    }
}

bool mcp23s17_lock(mcp23s17_handle_t handle) {
    if (handle == NULL || handle->spi_mutex == NULL) {
        s_lock_fail = "no handle";
        return false;
    }
    if (handle->owns_bus) {
        if (xSemaphoreTake(handle->spi_mutex, kMcpSpiLockTimeoutTicks) == pdTRUE) {
            mcp23s17_note_holder(handle);
            return true;
        }
        s_lock_fail = "mutex timeout";
        return false;
    }

    // Shared SPI3: defer while a Class 1 exchange is in its critical section.
    // Block on the exit edge (no mutex held) instead of per-tick polling,
    // which aliased with the tick-aligned 2 ms exchange and timed out.
    for (int attempt = 0; attempt < kMcpClass1RetryAttempts; ++attempt) {
        if (!spi3_wait_class1_idle(kMcpClass1WaitMs)) {
            s_lock_fail = "Class 1 idle wait timeout";
            return false;
        }
        if (xSemaphoreTake(handle->spi_mutex, kMcpSpiLockTimeoutTicks) != pdTRUE) {
            s_lock_fail = "mutex timeout";
            return false;
        }
        if (!spi3_class1_critical_active()) {
            mcp23s17_note_holder(handle);
            return true;
        }
        xSemaphoreGive(handle->spi_mutex);
    }
    s_lock_fail = "Class 1 re-entered on every retry";
    return false;
}

// Holds past the 8 ms waiter timeout are logged, at most once a second: at
// 4 ms the log ran ~6 lines a second and was load in itself (G-D3).
static constexpr int64_t kMcpLongHoldUs = 20000;
static constexpr int64_t kMcpHoldLogEveryUs = 1000000;
static int64_t s_hold_log_us = 0;

void mcp23s17_unlock(mcp23s17_handle_t handle) {
    if (handle != NULL && handle->spi_mutex != NULL) {
        const int64_t now = esp_timer_get_time();
        const int64_t held = now - handle->held_since_us;
        handle->held_since_us = 0;
        xSemaphoreGive(handle->spi_mutex);
        if (held > kMcpLongHoldUs && now - s_hold_log_us >= kMcpHoldLogEveryUs) {
            s_hold_log_us = now;
            ESP_LOGW(TAG, "spi_mutex held %lld us by %s", (long long)held, pcTaskGetName(NULL));
        }
    }
}

esp_err_t mcp23s17_write_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.length = 24;  // 3 bytes * 8 bits
    trans.tx_data[0] = MCP23S17_WRITE_OPCODE | (handle->device_address << 1);
    trans.tx_data[1] = reg;
    trans.tx_data[2] = value;

    esp_err_t ret = spi_device_polling_transmit(handle->spi_device, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t mcp23s17_read_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value) {
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans.length = 24;  // 3 bytes * 8 bits
    trans.tx_data[0] = MCP23S17_READ_OPCODE | (handle->device_address << 1);
    trans.tx_data[1] = reg;
    trans.tx_data[2] = 0;  // Dummy byte

    esp_err_t ret = spi_device_polling_transmit(handle->spi_device, &trans);
    if (ret == ESP_OK) {
        *value = trans.rx_data[2];
    } else {
        ESP_LOGE(TAG, "Read register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t mcp23s17_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
    if (!mcp23s17_lock(handle)) {
        ESP_LOGE(TAG, "SPI mutex lock failed for write reg 0x%02X (%s)", reg, s_lock_fail);
        mcp23s17_log_holder(handle);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, value);
    mcp23s17_unlock(handle);
    return ret;
}

esp_err_t mcp23s17_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value) {
    if (!mcp23s17_lock(handle)) {
        ESP_LOGE(TAG, "SPI mutex lock failed for read reg 0x%02X (%s)", reg, s_lock_fail);
        mcp23s17_log_holder(handle);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp23s17_read_register_unlocked(handle, reg, value);
    mcp23s17_unlock(handle);
    return ret;
}

esp_err_t mcp23s17_debug_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t *value) {
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return mcp23s17_read_register(handle, reg, value);
}

esp_err_t mcp23s17_debug_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return mcp23s17_write_register(handle, reg, value);
}
