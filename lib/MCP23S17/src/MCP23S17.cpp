/**
 * @file MCP23S17.cpp
 * @brief MCP23S17 SPI GPIO Expander Implementation
 */

#include "MCP23S17.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

// MCP23S17 Register addresses
#define MCP23S17_IODIRA   0x00  // I/O Direction Port A
#define MCP23S17_IODIRB   0x01  // I/O Direction Port B
#define MCP23S17_IPOLA    0x02  // Input Polarity Port A
#define MCP23S17_IPOLB    0x03  // Input Polarity Port B
#define MCP23S17_GPINTENA 0x04  // Interrupt Enable Port A
#define MCP23S17_GPINTENB 0x05  // Interrupt Enable Port B
#define MCP23S17_DEFVALA  0x06  // Default Compare Port A
#define MCP23S17_DEFVALB  0x07  // Default Compare Port B
#define MCP23S17_INTCONA  0x08  // Interrupt Control Port A
#define MCP23S17_INTCONB  0x09  // Interrupt Control Port B
#define MCP23S17_IOCON    0x0A  // I/O Configuration (shared)
#define MCP23S17_IOCON_INIT 0x08  // HAEN; mcp23s17_verify_config() expects it
#define MCP23S17_GPPUA    0x0C  // Pull-up Resistor Port A
#define MCP23S17_GPPUB    0x0D  // Pull-up Resistor Port B
#define MCP23S17_INTFA    0x0E  // Interrupt Flag Port A
#define MCP23S17_INTFB    0x0F  // Interrupt Flag Port B
#define MCP23S17_INTCAPA  0x10  // Interrupt Capture Port A
#define MCP23S17_INTCAPB  0x11  // Interrupt Capture Port B
#define MCP23S17_GPIOA    0x12  // GPIO Port A
#define MCP23S17_GPIOB    0x13  // GPIO Port B
#define MCP23S17_OLATA    0x14  // Output Latch Port A
#define MCP23S17_OLATB    0x15  // Output Latch Port B

// MCP23S17 SPI opcode
#define MCP23S17_WRITE_OPCODE 0x40
#define MCP23S17_READ_OPCODE  0x41

struct mcp23s17_handle {
    spi_device_handle_t spi_device;
    spi_host_device_t spi_host;
    SemaphoreHandle_t spi_mutex;
    uint8_t device_address;
    uint8_t port_a_dir;
    uint8_t port_b_dir;
    uint8_t port_a_output;
    uint8_t port_b_output;
    uint8_t port_a_pullup;
    uint8_t port_b_pullup;
    bool owns_bus;
    // Current holder of spi_mutex, for the timeout log (G-D3).
    volatile int64_t held_since_us;
    char holder[16];
};

static const char *TAG = "MCP23S17";

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

static inline bool mcp23s17_lock(mcp23s17_handle_t handle) {
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

static inline void mcp23s17_unlock(mcp23s17_handle_t handle) {
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

static esp_err_t mcp23s17_write_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
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

static esp_err_t mcp23s17_read_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value) {
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

static esp_err_t mcp23s17_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
    if (!mcp23s17_lock(handle)) {
        ESP_LOGE(TAG, "SPI mutex lock failed for write reg 0x%02X (%s)", reg, s_lock_fail);
        mcp23s17_log_holder(handle);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, value);
    mcp23s17_unlock(handle);
    return ret;
}

static esp_err_t mcp23s17_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value) {
    if (!mcp23s17_lock(handle)) {
        ESP_LOGE(TAG, "SPI mutex lock failed for read reg 0x%02X (%s)", reg, s_lock_fail);
        mcp23s17_log_holder(handle);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp23s17_read_register_unlocked(handle, reg, value);
    mcp23s17_unlock(handle);
    return ret;
}

mcp23s17_handle_t mcp23s17_init(const mcp23s17_config_t* config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    // Allocate handle
    mcp23s17_handle_t handle = (mcp23s17_handle_t)malloc(sizeof(struct mcp23s17_handle));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return NULL;
    }
    memset(handle, 0, sizeof(struct mcp23s17_handle));
    handle->device_address = config->device_address & 0x07;
    handle->spi_host = config->spi_host;
    handle->owns_bus = !config->skip_bus_init;
    handle->spi_mutex = xSemaphoreCreateMutex();
    if (handle->spi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        free(handle);
        return NULL;
    }

    if (handle->owns_bus) {
        spi_bus_config_t bus_cfg = {};
        bus_cfg.miso_io_num = config->miso_pin;
        bus_cfg.mosi_io_num = config->mosi_pin;
        bus_cfg.sclk_io_num = config->sclk_pin;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = 32;

        esp_err_t bus_ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (bus_ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(bus_ret));
            vSemaphoreDelete(handle->spi_mutex);
            free(handle);
            return NULL;
        }
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = config->clock_speed_hz > 0 ? config->clock_speed_hz : 1000000;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = config->cs_pin;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = 0;
    dev_cfg.cs_ena_pretrans = 2;
    dev_cfg.cs_ena_posttrans = 2;
    dev_cfg.pre_cb = NULL;

    esp_err_t ret = spi_bus_add_device(config->spi_host, &dev_cfg, &handle->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        if (handle->owns_bus) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(handle->spi_mutex);
        free(handle);
        return NULL;
    }

    // Use the configured hardware address (A2..A0 strapping). No auto-detect:
    // it looked for the power-on IODIRA value 0xFF, but after a software reset
    // the chip is still powered and keeps its configured IODIRA, while an empty
    // address reads a floating MISO that can come back 0xFF - the scan then
    // picked a non-existent address and every access failed until a power
    // cycle (2026-10-21 capture: opcodes 0x42/0x43 with A2..A0 = 0).
    handle->device_address = config->device_address & 0x07;
    {
        uint8_t iodira = 0;
        if (mcp23s17_read_register_unlocked(handle, MCP23S17_IODIRA, &iodira) == ESP_OK) {
            ESP_LOGI(TAG, "MCP23S17 at address 0x%02X: IODIRA=0x%02X (%s)",
                     0x20 | handle->device_address, iodira,
                     iodira == 0xFF ? "power-on default" : "kept through software reset");
        }
    }

    // Configure IOCON: enable HAEN (bit 3 = 0x08) so hardware address pins are active
    ret = mcp23s17_write_register(handle, MCP23S17_IOCON, MCP23S17_IOCON_INIT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IOCON init failed");
        mcp23s17_deinit(handle);
        return NULL;
    }

    handle->port_a_dir = 0xFF;
    handle->port_b_dir = 0xFF;
    mcp23s17_write_register(handle, MCP23S17_IODIRA, 0xFF);
    mcp23s17_write_register(handle, MCP23S17_IODIRB, 0xFF);
    mcp23s17_write_register(handle, MCP23S17_GPPUA, 0x00);
    mcp23s17_write_register(handle, MCP23S17_GPPUB, 0x00);

    handle->port_a_output = 0x00;
    handle->port_b_output = 0x00;
    handle->port_a_pullup = 0x00;
    handle->port_b_pullup = 0x00;

    // Verify SPI communication with MCP23S17 by reading back IODIRA
    uint8_t check_dir = 0;
    ret = mcp23s17_read_register(handle, MCP23S17_IODIRA, &check_dir);
    if (ret != ESP_OK || check_dir != 0xFF) {
        ESP_LOGE(TAG, "MCP23S17 communication check failed (IODIRA read 0x%02X, expected 0xFF)", check_dir);
    } else {
        ESP_LOGI(TAG, "MCP23S17 communication verified (IODIRA=0x%02X)", check_dir);
    }

    ESP_LOGI(TAG, "MCP23S17 initialized (address: 0x%02X, shared_bus=%d)",
             0x20 | handle->device_address, config->skip_bus_init ? 1 : 0);
    return handle;
}

void mcp23s17_deinit(mcp23s17_handle_t handle) {
    if (handle == NULL) return;

    if (handle->spi_device) {
        spi_bus_remove_device(handle->spi_device);
        handle->spi_device = NULL;
    }
    if (handle->owns_bus) {
        spi_bus_free(handle->spi_host);
    }
    if (handle->spi_mutex) {
        vSemaphoreDelete(handle->spi_mutex);
        handle->spi_mutex = NULL;
    }
    free(handle);
}

esp_err_t mcp23s17_set_pin_direction(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool is_output) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mcp23s17_lock(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    uint8_t* dir_cache = (pin < 8) ? &handle->port_a_dir : &handle->port_b_dir;

    if (is_output) {
        *dir_cache &= ~(1 << bit);
    } else {
        *dir_cache |= (1 << bit);
    }

    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, *dir_cache);
    mcp23s17_unlock(handle);
    return ret;
}

esp_err_t mcp23s17_set_pin_pullup(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool enable) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mcp23s17_lock(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    uint8_t* pullup_cache = (pin < 8) ? &handle->port_a_pullup : &handle->port_b_pullup;

    if (enable) {
        *pullup_cache |= (1 << bit);
    } else {
        *pullup_cache &= ~(1 << bit);
    }

    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, *pullup_cache);
    mcp23s17_unlock(handle);
    return ret;
}

esp_err_t mcp23s17_write_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin, uint8_t level) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mcp23s17_lock(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_OLATA : MCP23S17_OLATB;
    uint8_t* output_cache = (pin < 8) ? &handle->port_a_output : &handle->port_b_output;

    if (level) {
        *output_cache |= (1 << bit);
    } else {
        *output_cache &= ~(1 << bit);
    }

    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, *output_cache);
    mcp23s17_unlock(handle);
    return ret;
}

esp_err_t mcp23s17_read_latch(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t* value) {
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_OLATA : MCP23S17_OLATB;
    return mcp23s17_read_register(handle, reg, value);
}

esp_err_t mcp23s17_verify_config(mcp23s17_handle_t handle, uint8_t* iocon,
                                 uint8_t* iodira, uint8_t* iodirb) {
    if (handle == NULL || iocon == NULL || iodira == NULL || iodirb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = mcp23s17_read_register(handle, MCP23S17_IOCON, iocon);
    if (ret == ESP_OK) ret = mcp23s17_read_register(handle, MCP23S17_IODIRA, iodira);
    if (ret == ESP_OK) ret = mcp23s17_read_register(handle, MCP23S17_IODIRB, iodirb);
    if (ret != ESP_OK) {
        return ret;
    }
    if (*iocon != MCP23S17_IOCON_INIT || *iodira != handle->port_a_dir ||
        *iodirb != handle->port_b_dir) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

uint8_t mcp23s17_read_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin) {
    if (handle == NULL || pin > 15) {
        return 0;
    }

    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t value = 0;
    
    if (mcp23s17_read_register(handle, reg, &value) != ESP_OK) {
        return 0;
    }

    uint8_t bit = pin & 0x07;
    return (value >> bit) & 0x01;
}

esp_err_t mcp23s17_write_port(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t value) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mcp23s17_lock(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_OLATA : MCP23S17_OLATB;
    if (port == MCP23S17_PORT_A) {
        handle->port_a_output = value;
    } else {
        handle->port_b_output = value;
    }

    esp_err_t ret = mcp23s17_write_register_unlocked(handle, reg, value);
    mcp23s17_unlock(handle);
    return ret;
}

uint8_t mcp23s17_read_port(mcp23s17_handle_t handle, mcp23s17_port_t port) {
    if (handle == NULL) {
        return 0;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t value = 0;
    
    if (mcp23s17_read_register(handle, reg, &value) != ESP_OK) {
        return 0;
    }

    return value;
}

esp_err_t mcp23s17_set_pin_interrupt(mcp23s17_handle_t handle, mcp23s17_pin_t pin, 
                                      bool enable, bool trigger_on_rising) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mcp23s17_lock(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bit = pin & 0x07;
    uint8_t inten_reg = (pin < 8) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    uint8_t intcon_reg = (pin < 8) ? MCP23S17_INTCONA : MCP23S17_INTCONB;
    uint8_t defval_reg = (pin < 8) ? MCP23S17_DEFVALA : MCP23S17_DEFVALB;
    esp_err_t ret;

    // Enable/disable interrupt
    uint8_t inten_val = 0;
    ret = mcp23s17_read_register_unlocked(handle, inten_reg, &inten_val);
    if (ret != ESP_OK) {
        mcp23s17_unlock(handle);
        return ret;
    }
    if (enable) {
        inten_val |= (1 << bit);
    } else {
        inten_val &= ~(1 << bit);
    }
    ret = mcp23s17_write_register_unlocked(handle, inten_reg, inten_val);
    if (ret != ESP_OK) {
        mcp23s17_unlock(handle);
        return ret;
    }

    // Configure interrupt trigger (compare to DEFVAL)
    uint8_t intcon_val = 0;
    ret = mcp23s17_read_register_unlocked(handle, intcon_reg, &intcon_val);
    if (ret != ESP_OK) {
        mcp23s17_unlock(handle);
        return ret;
    }
    intcon_val |= (1 << bit);
    ret = mcp23s17_write_register_unlocked(handle, intcon_reg, intcon_val);
    if (ret != ESP_OK) {
        mcp23s17_unlock(handle);
        return ret;
    }

    // Set default value for comparison
    uint8_t defval_val = 0;
    ret = mcp23s17_read_register_unlocked(handle, defval_reg, &defval_val);
    if (ret != ESP_OK) {
        mcp23s17_unlock(handle);
        return ret;
    }
    if (trigger_on_rising) {
        defval_val &= ~(1 << bit);  // Trigger when pin goes HIGH (was LOW)
    } else {
        defval_val |= (1 << bit);   // Trigger when pin goes LOW (was HIGH)
    }
    ret = mcp23s17_write_register_unlocked(handle, defval_reg, defval_val);
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
