/**
 * @file gpio_expander.c
 * @brief GPIO Expander Implementation
 */

#include "gpio_expander.h"
#include "MCP23S17.h"
#include "esp_log.h"

static const char *TAG = "GPIOExpander";

static mcp23s17_handle_t g_mcp_handle = NULL;

static bool is_mcp_pin(int pin) {
    return (pin >= 0 && pin < GPIO_DIRECT_PIN_BASE);
}

bool gpio_expander_init(const mcp23s17_config_t* mcp_config) {
    if (mcp_config == NULL) {
        ESP_LOGE(TAG, "MCP23S17 config is NULL");
        return false;
    }

    g_mcp_handle = mcp23s17_init(mcp_config);
    if (g_mcp_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MCP23S17");
        return false;
    }

    ESP_LOGI(TAG, "GPIO expander initialized");
    return true;
}

void gpio_expander_deinit(void) {
    if (g_mcp_handle != NULL) {
        mcp23s17_deinit(g_mcp_handle);
        g_mcp_handle = NULL;
    }
}

esp_err_t gpio_expander_set_direction(int pin, bool is_output) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_set_pin_direction(g_mcp_handle, (mcp23s17_pin_t)pin, is_output);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t gpio_expander_set_pullup(int pin, bool enable) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_set_pin_pullup(g_mcp_handle, (mcp23s17_pin_t)pin, enable);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t gpio_expander_write(int pin, uint8_t level) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_write_pin(g_mcp_handle, (mcp23s17_pin_t)pin, level);
    }
    return ESP_ERR_INVALID_ARG;
}

uint8_t gpio_expander_read(int pin) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return 0;
        }
        return mcp23s17_read_pin(g_mcp_handle, (mcp23s17_pin_t)pin);
    }
    return 0;
}

mcp23s17_handle_t gpio_expander_get_mcp_handle(void) {
    return g_mcp_handle;
}
