#include "pick_scheduler.h"

#include "MqttBridge.h"
#include "MqttBridgeTypes.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <cstdio>

static const char* TAG = "PickScheduler";

void pickSchedulerTask(void* param) {
    auto* cfg = static_cast<PickSchedulerTaskConfig*>(param);
    if (cfg == nullptr || cfg->bridge == nullptr || cfg->gantry == nullptr) {
        ESP_LOGE(TAG, "Invalid pick scheduler config");
        vTaskDelete(nullptr);
        return;
    }

    QueueHandle_t pick_queue = cfg->bridge->pickQueue();
    if (pick_queue == nullptr) {
        ESP_LOGE(TAG, "Pick queue not available");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "Pick scheduler started (plan-only; no motion yet)");

    while (true) {
        MqttBridge::PickFrame frame = {};
        if (xQueueReceive(pick_queue, &frame, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG,
                 "Pick frame: X=%.3f Z=%.3f THETA=%.3f t_epoch_us=%llu ETA_epoch=%llu",
                 frame.x_mm,
                 frame.z_mm,
                 frame.theta_deg,
                 static_cast<unsigned long long>(frame.t_epoch_us),
                 static_cast<unsigned long long>(frame.eta_epoch_us));

        char status_json[320] = {0};
        std::snprintf(status_json,
                      sizeof(status_json),
                      "{\"state\":\"SKIP\",\"reason\":\"pick_motion_not_wired\","
                      "\"X\":%.3f,\"Z\":%.3f,\"THETA\":%.3f,"
                      "\"t_epoch_us\":%llu,\"ETA_epoch\":%llu}",
                      frame.x_mm,
                      frame.z_mm,
                      frame.theta_deg,
                      static_cast<unsigned long long>(frame.t_epoch_us),
                      static_cast<unsigned long long>(frame.eta_epoch_us));
        if (!cfg->bridge->publishStatusJson(status_json)) {
            ESP_LOGW(TAG, "Failed to publish pick status");
        }
    }
}
