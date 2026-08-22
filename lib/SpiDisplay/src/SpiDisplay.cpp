#include "SpiDisplay.h"

#include "Spi3Bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace display {
namespace {
const char* TAG = "SpiDisplay";
}  // namespace

bool SpiDisplay::begin(const SpiDisplayConfig& cfg) {
    cfg_ = cfg;
    ready_ = false;

    if (!spi3::isReady() || cfg_.mcp == nullptr || cfg_.mcp_cs_pin < 0) {
        ESP_LOGW(TAG, "SpiDisplay begin skipped (SPI3/MCP/CS not ready)");
        return false;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = cfg_.clock_hz > 0 ? cfg_.clock_hz : 20000000;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = -1;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = 0;

    esp_err_t err = spi_bus_add_device(spi3::host(), &dev_cfg, &tft_dev_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TFT spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    err = spi3::withMcp([this]() -> esp_err_t {
        auto setOut = [this](int pin, uint8_t level) -> esp_err_t {
            if (pin < 0) return ESP_OK;
            esp_err_t e = mcp23s17_set_pin_direction(
                cfg_.mcp, static_cast<mcp23s17_pin_t>(pin), true);
            if (e != ESP_OK) return e;
            return mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(pin),
                                      level);
        };
        esp_err_t e = setOut(cfg_.mcp_cs_pin, 1);
        if (e != ESP_OK) return e;
        e = setOut(cfg_.mcp_res_pin, 0);
        if (e != ESP_OK) return e;
        e = setOut(cfg_.mcp_dc_pin, 0);
        if (e != ESP_OK) return e;
        e = setOut(cfg_.mcp_blk_pin, 1);
        if (e != ESP_OK) return e;
        return ESP_OK;
    });
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MCP TFT control init failed: %s", esp_err_to_name(err));
        spi_bus_remove_device(tft_dev_);
        tft_dev_ = nullptr;
        return false;
    }

    if (cfg_.mcp_res_pin >= 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)spi3::withMcp([this]() -> esp_err_t {
            return mcp23s17_write_pin(
                cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_res_pin), 1);
        });
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ready_ = true;
    ESP_LOGI(TAG, "SpiDisplay stub ready (MCP CS pin %d)", cfg_.mcp_cs_pin);
    return true;
}

bool SpiDisplay::refreshStub() {
    if (!ready_ || tft_dev_ == nullptr) {
        return false;
    }

    esp_err_t err = spi3::withTft([this]() -> esp_err_t {
        esp_err_t e = mcp23s17_write_pin(
            cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_cs_pin), 0);
        if (e != ESP_OK) return e;
        if (cfg_.mcp_dc_pin >= 0) {
            e = mcp23s17_write_pin(cfg_.mcp,
                                   static_cast<mcp23s17_pin_t>(cfg_.mcp_dc_pin),
                                   0);
            if (e != ESP_OK) {
                (void)mcp23s17_write_pin(
                    cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_cs_pin), 1);
                return e;
            }
        }
        uint8_t byte = 0x00;
        spi_transaction_t t = {};
        t.length = 8;
        t.tx_buffer = &byte;
        e = spi_device_transmit(tft_dev_, &t);
        (void)mcp23s17_write_pin(
            cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_cs_pin), 1);
        return e;
    });

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "refreshStub failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

}  // namespace display
