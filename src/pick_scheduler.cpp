/**
 * @file pick_scheduler.cpp
 * @brief Dynamic intercept pick scheduler implementation using CellNetL2.
 */

#include "pick_scheduler.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "PickScheduler";

static QueueHandle_t s_pick_queue = nullptr;
static float s_live_belt_speed_mm_s = 1524.0f;

void pickSchedulerTask(void *param) {
  auto *cfg = static_cast<PickSchedulerTaskConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Invalid pick scheduler config");
    vTaskDelete(nullptr);
    return;
  }

  s_pick_queue = xQueueCreate(16, sizeof(L2VisionDetectPayload));
  if (s_pick_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create pick queue");
    vTaskDelete(nullptr);
    return;
  }

  if (cfg->net_l2 != nullptr) {
    cfg->net_l2->onVisionDetect([](const L2VisionDetectPayload &payload, const L2CellHeader &) {
      if (s_pick_queue != nullptr) {
        xQueueSend(s_pick_queue, &payload, 0);
      }
    });
    cfg->net_l2->onConveyorSpeed([](const L2ConveyorSpeedPayload &payload, const L2CellHeader &) {
      s_live_belt_speed_mm_s = payload.speed_mm_s;
    });
  }

  ESP_LOGI(TAG, "Pick scheduler started with OSI Layer-2 link");

  while (true) {
    L2VisionDetectPayload detect = {};
    if (xQueueReceive(s_pick_queue, &detect, pdMS_TO_TICKS(200)) != pdTRUE) {
      continue;
    }

    ESP_LOGI(TAG,
             "L2 Pick Target: Item=%lu X=%.2f mm Y=%.2f mm Theta=%.2f deg (v_belt=%.1f mm/s)",
             (unsigned long)detect.item_id, detect.x_across_mm, detect.y_bat_mm,
             detect.theta_deg, (double)s_live_belt_speed_mm_s);

    int64_t start_time_us = esp_timer_get_time();

    // Execute 3-axis motion to pick pose
    Gantry::GantryError err = cfg->gantry->moveTo(
        Gantry::EndEffectorPose(detect.x_across_mm, 0.0f, 0.0f, detect.theta_deg),
        100, 180);

    if (err != Gantry::GantryError::OK) {
      ESP_LOGW(TAG, "Pick trajectory rejected (error %d) — aborting", static_cast<int>(err));
      continue;
    }

    while (cfg->gantry->isBusy()) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Actuate gripper
    ESP_LOGI(TAG, "Gantry at pick pose. Actuating pneumatic gripper...");
    vTaskDelay(pdMS_TO_TICKS(150));

    float cycle_time_ms = static_cast<float>(esp_timer_get_time() - start_time_us) / 1000.0f;

    // Publish status over Layer-2
    if (cfg->net_l2 != nullptr) {
      cfg->net_l2->sendGantryStatus(
          0, 0, detect.x_across_mm, 0.0f, detect.theta_deg, cycle_time_ms);
    }
  }
}
