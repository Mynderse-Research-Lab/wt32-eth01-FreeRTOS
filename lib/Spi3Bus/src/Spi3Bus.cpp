#include "Spi3Bus.h"
#include "Spi3BusCritical.h"

#include "gantry_app_constants.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

namespace spi3 {
namespace {

const char* TAG = "Spi3Bus";
SemaphoreHandle_t g_mutex = nullptr;
bool g_ready = false;
Class1Critical g_class1;

// "Class 1 idle" event bit, set on the last exit and cleared on the first
// enter. Polling active() once per tick aliased with the tick-aligned 2 ms
// exchange: when an exchange ran past 1 ms the waiter saw "active" at every
// tick boundary and timed out after 8 ms although SPI was mostly free.
constexpr EventBits_t kClass1IdleBit = BIT0;
StaticEventGroup_t g_idle_events_buf;
EventGroupHandle_t g_idle_events = nullptr;

// Total budget for Class 1 to go idle before an SPI3 access gives up.
constexpr uint32_t kClass1WaitMs = 8;
// Re-checks if Class 1 re-entered between our wake-up and taking the mutex.
constexpr int kClass1RetryAttempts = 4;
constexpr uint32_t kMutexTimeoutMs = 50;

esp_err_t waitClass1Clear(TickType_t timeout_ticks) {
    if (!g_class1.active()) {
        return ESP_OK;
    }
    const TickType_t start = xTaskGetTickCount();
    while (true) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks) {
            return g_class1.active() ? ESP_ERR_TIMEOUT : ESP_OK;
        }
        if (g_idle_events == nullptr) {
            vTaskDelay(1);  // init() not run yet: old polling fallback
        } else {
            const EventBits_t bits = xEventGroupWaitBits(
                g_idle_events, kClass1IdleBit, pdFALSE, pdTRUE, timeout_ticks - elapsed);
            if (!g_class1.active()) {
                return ESP_OK;
            }
            if ((bits & kClass1IdleBit) != 0) {
                // Bit left set by an exit that raced a new enter. Clear it, then
                // re-check so a real exit landing in between is not lost.
                xEventGroupClearBits(g_idle_events, kClass1IdleBit);
                if (!g_class1.active()) {
                    xEventGroupSetBits(g_idle_events, kClass1IdleBit);
                    return ESP_OK;
                }
            }
        }
        if (!g_class1.active()) {
            return ESP_OK;
        }
    }
}

esp_err_t withLocked(const std::function<esp_err_t()>& fn) {
    if (!g_ready || g_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const TickType_t start = xTaskGetTickCount();
    const TickType_t budget = pdMS_TO_TICKS(kClass1WaitMs);
    for (int attempt = 0; attempt < kClass1RetryAttempts; ++attempt) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        const TickType_t remaining = (elapsed < budget) ? (budget - elapsed) : 0;
        if (waitClass1Clear(remaining) != ESP_OK) {
            break;
        }
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) != pdTRUE) {
            ESP_LOGE(TAG, "SPI3 mutex timeout");
            return ESP_ERR_TIMEOUT;
        }
        if (!g_class1.active()) {
            const esp_err_t err = fn();
            xSemaphoreGive(g_mutex);
            return err;
        }
        // Class 1 started again between the wake-up and the mutex: back off.
        xSemaphoreGive(g_mutex);
    }
    ESP_LOGW(TAG, "SPI3 deferred: Class 1 critical timeout");
    return ESP_ERR_TIMEOUT;
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
    if (g_idle_events == nullptr) {
        // Static storage: never freed, so enter/exit may signal it at any time.
        g_idle_events = xEventGroupCreateStatic(&g_idle_events_buf);
        if (!g_class1.active()) {
            xEventGroupSetBits(g_idle_events, kClass1IdleBit);
        }
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
             "SPI3 ready (SCLK=%d MOSI=%d MISO=%d CS_MCP=%d); TFT CS=%d; "
             "Class1 deferral enabled",
             SPI3_SCLK_GPIO, SPI3_MOSI_GPIO, SPI3_MISO_GPIO, SPI3_CS_MCP_GPIO, SPI3_CS_TFT_GPIO);
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
    if (g_class1.enter() && g_idle_events != nullptr) {
        xEventGroupClearBits(g_idle_events, kClass1IdleBit);
    }
}

void class1CriticalExit() {
    if (g_class1.exit() && g_idle_events != nullptr) {
        xEventGroupSetBits(g_idle_events, kClass1IdleBit);
    }
}

bool class1CriticalActive() { return g_class1.active(); }

bool waitClass1Idle(uint32_t timeout_ms) {
    return waitClass1Clear(pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

esp_err_t withMcp(const std::function<esp_err_t()>& fn) { return withLocked(fn); }

esp_err_t withTft(const std::function<esp_err_t()>& fn) { return withLocked(fn); }

}  // namespace spi3

extern "C" void spi3_class1_critical_enter(void) { spi3::class1CriticalEnter(); }
extern "C" void spi3_class1_critical_exit(void) { spi3::class1CriticalExit(); }
extern "C" bool spi3_class1_critical_active(void) { return spi3::class1CriticalActive(); }
extern "C" bool spi3_wait_class1_idle(uint32_t timeout_ms) {
    return spi3::waitClass1Idle(timeout_ms);
}
