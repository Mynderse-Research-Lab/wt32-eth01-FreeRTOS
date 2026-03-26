/**
 * @file gpio_expander.c
 * @brief GPIO Expander Implementation
 */

#include "gpio_expander.h"
#include "MCP23S17.h"
#include "esp_log.h"

static const char *TAG = "GPIOExpander";

static mcp23s17_handle_t g_mcp_handle = NULL;

static bool is_mcp_pin(uint8_t pin) {
    return (pin <= GPIO_EXPANDER_PIN_MAX);
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

esp_err_t gpio_expander_set_direction(uint8_t pin, bool is_output) {
    if (!is_mcp_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_mcp_handle == NULL) {
        ESP_LOGE(TAG, "MCP23S17 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return mcp23s17_set_pin_direction(g_mcp_handle, pin, is_output);
}

esp_err_t gpio_expander_set_pullup(uint8_t pin, bool enable) {
    if (!is_mcp_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_mcp_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return mcp23s17_set_pin_pullup(g_mcp_handle, pin, enable);
}

esp_err_t gpio_expander_write(uint8_t pin, uint8_t level) {
    if (!is_mcp_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_mcp_handle == NULL) {
        ESP_LOGE(TAG, "MCP23S17 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return mcp23s17_write_pin(g_mcp_handle, pin, level);
}

uint8_t gpio_expander_read(uint8_t pin) {
    if (!is_mcp_pin(pin)) {
        return 0;
    }
    if (g_mcp_handle == NULL) {
        ESP_LOGE(TAG, "MCP23S17 not initialized");
        return 0;
    }
    return mcp23s17_read_pin(g_mcp_handle, pin);
}

mcp23s17_handle_t gpio_expander_get_mcp_handle(void) {
    return g_mcp_handle;
}
