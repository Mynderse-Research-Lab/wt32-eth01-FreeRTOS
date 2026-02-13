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
    // MCP pins are meaningful only when an MCP device is initialized.
    // In direct mode (no MCP attached), all pins are treated as direct GPIO.
    return (g_mcp_handle != NULL) && (pin < GPIO_DIRECT_PIN_BASE);
}

bool gpio_expander_init(const mcp23s17_config_t* mcp_config) {
    if (mcp_config == NULL) {
        ESP_LOGW(TAG, "MCP23S17 config is NULL - using direct GPIO only");
        return true;
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
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_set_pin_direction(g_mcp_handle, pin, is_output);
    } else {
        // Direct GPIO pin (raw GPIO number, no offset encoding)
        gpio_num_t gpio_num = (gpio_num_t)pin;
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << gpio_num);
        io_conf.mode = is_output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        return gpio_config(&io_conf);
    }
}

esp_err_t gpio_expander_set_pullup(uint8_t pin, bool enable) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_set_pin_pullup(g_mcp_handle, pin, enable);
    } else {
        // Direct GPIO pin - configure pull-up
        gpio_num_t gpio_num = (gpio_num_t)pin;
        gpio_set_pull_mode(gpio_num, enable ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
        return ESP_OK;
    }
}

esp_err_t gpio_expander_write(uint8_t pin, uint8_t level) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_write_pin(g_mcp_handle, pin, level);
    } else {
        // Direct GPIO pin (raw GPIO number, no offset encoding)
        gpio_num_t gpio_num = (gpio_num_t)pin;
        gpio_set_level(gpio_num, level);
        return ESP_OK;
    }
}

uint8_t gpio_expander_read(uint8_t pin) {
    if (is_mcp_pin(pin)) {
        // MCP23S17 pin
        if (g_mcp_handle == NULL) {
            ESP_LOGE(TAG, "MCP23S17 not initialized");
            return 0;
        }
        return mcp23s17_read_pin(g_mcp_handle, pin);
    } else {
        // Direct GPIO pin (raw GPIO number, no offset encoding)
        gpio_num_t gpio_num = (gpio_num_t)pin;
        return gpio_get_level(gpio_num);
    }
}

mcp23s17_handle_t gpio_expander_get_mcp_handle(void) {
    return g_mcp_handle;
}
