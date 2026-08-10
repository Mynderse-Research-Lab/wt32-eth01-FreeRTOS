#include "Spi3Bus.h"

#include "gantry_app_constants.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <atomic>

namespace spi3 {
namespace {

const char* TAG = "Spi3Bus";
SemaphoreHandle_t g_mutex = nullptr;
bool g_ready = false;
std::atomic<int> g_class1_critical{0};

esp_err_t waitClass1Clear(TickType_t timeout_ticks) {
    const TickType_t start = xTaskGetTickCount();
    while (g_class1_critical.load(std::memory_order_acquire) > 0) {
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            ESP_LOGW(TAG, "SPI3 deferred: Class 1 critical timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

esp_err_t withLocked(const std::function<esp_err_t()>& fn) {
    if (!g_ready || g_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (waitClass1Clear(pdMS_TO_TICKS(20)) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGE(TAG, "SPI3 mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    if (g_class1_critical.load(std::memory_order_acquire) > 0) {
        xSemaphoreGive(g_mutex);
        if (waitClass1Clear(pdMS_TO_TICKS(20)) != ESP_OK) {
            return ESP_ERR_TIMEOUT;
        }
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
    esp_err_t err = fn();
    xSemaphoreGive(g_mutex);
    return err;
}

}  // namespace

bool init() {
    if (g_ready) {
        return true;
    }

    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == nullptr) {
        ESP_LOGE(TAG, "mutex create failed");
        return false;
    }

    gpio_config_t cs_cfg = {};
    cs_cfg.pin_bit_mask = (1ULL << SPI3_CS_MCP_GPIO);
    cs_cfg.mode = GPIO_MODE_OUTPUT;
    cs_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cs_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cs_cfg.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&cs_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "CS_MCP gpio_config failed");
        vSemaphoreDelete(g_mutex);
        g_mutex = nullptr;
        return false;
    }
    gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 1);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = SPI3_MISO_GPIO;
    bus_cfg.mosi_io_num = SPI3_MOSI_GPIO;
    bus_cfg.sclk_io_num = SPI3_SCLK_GPIO;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4096;

    esp_err_t err = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(g_mutex);
        g_mutex = nullptr;
        return false;
    }

    g_ready = true;
    ESP_LOGI(TAG,
             "SPI3 ready (SCLK=%d MOSI=%d MISO=%d CS_MCP=%d); TFT CS on MCP; "
             "Class1 deferral enabled",
             SPI3_SCLK_GPIO, SPI3_MOSI_GPIO, SPI3_MISO_GPIO, SPI3_CS_MCP_GPIO);
    return true;
}

void deinit() {
    if (!g_ready) {
        return;
    }
    spi_bus_free(SPI3_HOST);
    if (g_mutex) {
        vSemaphoreDelete(g_mutex);
        g_mutex = nullptr;
    }
    g_ready = false;
}

bool isReady() { return g_ready; }

spi_host_device_t host() { return SPI3_HOST; }

SemaphoreHandle_t mutex() { return g_mutex; }

void class1CriticalEnter() {
    g_class1_critical.fetch_add(1, std::memory_order_acq_rel);
}

void class1CriticalExit() {
    int v = g_class1_critical.fetch_sub(1, std::memory_order_acq_rel);
    if (v <= 0) {
        g_class1_critical.store(0, std::memory_order_release);
    }
}

bool class1CriticalActive() {
    return g_class1_critical.load(std::memory_order_acquire) > 0;
}

esp_err_t withMcp(const std::function<esp_err_t()>& fn) { return withLocked(fn); }

esp_err_t withTft(const std::function<esp_err_t()>& fn) { return withLocked(fn); }

}  // namespace spi3

extern "C" void spi3_class1_critical_enter(void) { spi3::class1CriticalEnter(); }
extern "C" void spi3_class1_critical_exit(void) { spi3::class1CriticalExit(); }
