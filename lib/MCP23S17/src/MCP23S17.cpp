/**
 * @file MCP23S17.cpp
 * @brief MCP23S17 SPI GPIO Expander Implementation
 */

#include "MCP23S17.h"
#include "MCP23S17Internal.h"
#include <string.h>

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
