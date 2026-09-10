/**
 * @file gpio_expander.c
 * @brief GPIO Expander Implementation (MCP23S17)
 */

#include "gpio_expander.h"
#include "gantry_app_constants.h"
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
        if (g_mcp_handle == NULL) {
            return ESP_ERR_INVALID_STATE;
        }
        return mcp23s17_set_pin_pullup(g_mcp_handle, (mcp23s17_pin_t)pin, enable);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t gpio_expander_write(int pin, uint8_t level) {
    if (is_mcp_pin(pin)) {
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

uint8_t gpio_expander_read_port_b(void) {
    if (g_mcp_handle == NULL) {
        return 0xFF;
    }
    return mcp23s17_read_port(g_mcp_handle, MCP23S17_PORT_B);
}

esp_err_t gpio_expander_configure_field_and_ui(void) {
    if (g_mcp_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Port A: DOUT0–3 outputs (LOW), DIN0–3 inputs
    for (int i = MCP_FIELD_DOUT0; i <= MCP_FIELD_DOUT3; ++i) {
        esp_err_t e = mcp23s17_set_pin_direction(g_mcp_handle, (mcp23s17_pin_t)i, true);
        if (e != ESP_OK) return e;
        e = mcp23s17_write_pin(g_mcp_handle, (mcp23s17_pin_t)i, 0);
        if (e != ESP_OK) return e;
    }
    for (int i = MCP_FIELD_DIN0; i <= MCP_FIELD_DIN3; ++i) {
        esp_err_t e = mcp23s17_set_pin_direction(g_mcp_handle, (mcp23s17_pin_t)i, false);
        if (e != ESP_OK) return e;
    }

    // Port B inputs with pull-ups: KO (PB0), PUSH (PB1), ENC_B (PB2), ENC_A (PB3)
    for (int i = MCP_UI_ENC_KO; i <= MCP_UI_ENC_A; ++i) {
        esp_err_t e = mcp23s17_set_pin_direction(g_mcp_handle, (mcp23s17_pin_t)i, false);
        if (e != ESP_OK) return e;
        e = mcp23s17_set_pin_pullup(g_mcp_handle, (mcp23s17_pin_t)i, true);
        if (e != ESP_OK) return e;
    }

    // Port B outputs: BLK (PB4), DC (PB5), RES (PB6), W5500_RST (PB7).
    // Initialize BLK=HIGH (backlight ON), DC=LOW (command mode), RES=HIGH (run mode), W5500_RST=HIGH.
    const int tft_outs[] = {MCP_TFT_BLK, MCP_TFT_DC, MCP_TFT_RES, MCP_W5500_RST};
    for (unsigned i = 0; i < sizeof(tft_outs) / sizeof(tft_outs[0]); ++i) {
        if (tft_outs[i] < 0) continue;
        esp_err_t e =
            mcp23s17_set_pin_direction(g_mcp_handle, (mcp23s17_pin_t)tft_outs[i], true);
        if (e != ESP_OK) return e;
        uint8_t level = (tft_outs[i] == MCP_TFT_DC) ? 0 : 1;
        e = mcp23s17_write_pin(g_mcp_handle, (mcp23s17_pin_t)tft_outs[i], level);
        if (e != ESP_OK) return e;
    }

    ESP_LOGI(TAG,
             "MCP Port A Field x8 (DOUT0=gripper); Port B: KO/PUSH/ENC(PB0..3) BLK(PB4) DC(PB5) RES(PB6) W5500_RST(PB7)");
    return ESP_OK;
}

void gpio_expander_w5500_rst_set_level(void* ctx, int level) {
    (void)ctx;
    if (g_mcp_handle == NULL) {
        return;
    }
    (void)mcp23s17_write_pin(g_mcp_handle, (mcp23s17_pin_t)MCP_W5500_RST,
                             level ? 1 : 0);
}

esp_err_t field_dout_set(unsigned channel, bool level) {
    if (channel >= FIELD_24V_DOUT_COUNT || g_mcp_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return mcp23s17_write_pin(g_mcp_handle,
                              (mcp23s17_pin_t)(MCP_FIELD_DOUT0 + (int)channel),
                              level ? 1 : 0);
}

bool field_din_get(unsigned channel) {
    if (channel >= FIELD_24V_DIN_COUNT || g_mcp_handle == NULL) {
        return false;
    }
    return mcp23s17_read_pin(g_mcp_handle,
                             (mcp23s17_pin_t)(MCP_FIELD_DIN0 + (int)channel)) != 0;
}
