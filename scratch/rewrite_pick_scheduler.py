import re

with open("src/pick_scheduler.cpp", "r") as f:
    content = f.read()

replacement = """
struct PickTask {
  L2VisionDetectPayload detect;
  int64_t received_time_us;
};

static QueueHandle_t s_pick_queue = nullptr;
static float s_live_belt_speed_mm_s = 1524.0f;

void pickSchedulerTask(void *param) {
  auto *cfg = static_cast<PickSchedulerTaskConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Invalid pick scheduler config");
    vTaskDelete(nullptr);
    return;
  }

  s_live_belt_speed_mm_s = Config::AppConfig::instance().data().pick_default_belt_speed_mm_s;

  s_pick_queue = xQueueCreate(16, sizeof(PickTask));
  if (s_pick_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create pick queue");
    vTaskDelete(nullptr);
    return;
  }

  if (cfg->net_l2 != nullptr) {
    cfg->net_l2->onVisionDetect([](const L2VisionDetectPayload &payload, const L2CellHeader &hdr) {
      if (s_pick_queue != nullptr) {
        PickTask task = {payload, esp_timer_get_time()};
        xQueueSend(s_pick_queue, &task, 0);
      }
    });
    cfg->net_l2->onConveyorSpeed([](const L2ConveyorSpeedPayload &payload, const L2CellHeader &) {
      s_live_belt_speed_mm_s = payload.speed_mm_s;
    });
  }

  ESP_LOGI(TAG, "Pick scheduler started with OSI Layer-2 link");

  while (true) {
    PickTask task = {};
    if (xQueueReceive(s_pick_queue, &task, pdMS_TO_TICKS(200)) != pdTRUE) {
      continue;
    }

    const auto& detect = task.detect;
    ESP_LOGI(TAG,
             "L2 Pick Target: Item=%lu X=%.2f mm Y=%.2f mm Theta=%.2f deg (v_belt=%.1f mm/s)",
             (unsigned long)detect.item_id, detect.x_across_mm, detect.y_bat_mm,
             detect.theta_deg, (double)s_live_belt_speed_mm_s);

    int64_t start_time_us = esp_timer_get_time();
    const auto& cdata = Config::AppConfig::instance().data();

    // 1. Feasibility check
    float D_mm = cdata.conveyor_y_pick_mm - detect.y_bat_mm;
    if (D_mm <= 0.0f) {
      ESP_LOGW(TAG, "Battery past pick plane, skipping.");
      continue;
    }
    
    float t_pick_local_s = D_mm / s_live_belt_speed_mm_s;
    int64_t t_pick_local_us = task.received_time_us + (int64_t)(t_pick_local_s * 1000000.0f);
    float safe_z = cdata.gantry_safe_z_height_mm;
    float pick_z = cdata.conveyor_z_pick_joint_mm;
    float target_x = cdata.conveyor_x_across_to_gantry_x_offset_mm + detect.x_across_mm;

    // 2. APPROACH (X, Theta, safe Z)
    Gantry::GantryError err = cfg->gantry->moveTo(
        Gantry::EndEffectorPose(target_x, 0.0f, safe_z, detect.theta_deg),
        cdata.gantry_default_speed_mm_per_s, cdata.gantry_default_speed_deg_per_s);
    if (err != Gantry::GantryError::OK) {
      ESP_LOGW(TAG, "APPROACH rejected (error %d)", static_cast<int>(err));
      continue;
    }
    while (cfg->gantry->isBusy()) { vTaskDelay(pdMS_TO_TICKS(10)); }

    // 3. WAIT_DEADLINE
    int64_t wait_us = t_pick_local_us - esp_timer_get_time() - (int64_t)(cdata.gantry_gripper_close_time_ms * 1000);
    if (wait_us > 0) {
      vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
    }

    // 4. DESCEND
    err = cfg->gantry->moveTo(
        Gantry::EndEffectorPose(target_x, 0.0f, pick_z, detect.theta_deg),
        cdata.gantry_default_speed_mm_per_s, cdata.gantry_default_speed_deg_per_s);
    if (err == Gantry::GantryError::OK) {
      while (cfg->gantry->isBusy()) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }

    // 5. GRIP
    ESP_LOGI(TAG, "Gantry at pick pose. Actuating pneumatic gripper...");
    cfg->gantry->grip(true);
    vTaskDelay(pdMS_TO_TICKS(cdata.gantry_gripper_close_time_ms));

    // 6. RETRACT
    err = cfg->gantry->moveTo(
        Gantry::EndEffectorPose(target_x, 0.0f, safe_z, detect.theta_deg),
        cdata.gantry_default_speed_mm_per_s, cdata.gantry_default_speed_deg_per_s);
    if (err == Gantry::GantryError::OK) {
      while (cfg->gantry->isBusy()) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }

    // 7. TRANSFER & RELEASE (Park position for now)
    err = cfg->gantry->moveTo(
        Gantry::EndEffectorPose(cdata.gantry_cal_x_park_mm, 0.0f, safe_z, 0.0f),
        cdata.gantry_default_speed_mm_per_s, cdata.gantry_default_speed_deg_per_s);
    if (err == Gantry::GantryError::OK) {
      while (cfg->gantry->isBusy()) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }
    
    cfg->gantry->grip(false);
    vTaskDelay(pdMS_TO_TICKS(cdata.gantry_gripper_open_time_ms));

    float cycle_time_ms = static_cast<float>(esp_timer_get_time() - start_time_us) / 1000.0f;

    // Publish status over Layer-2
    if (cfg->net_l2 != nullptr) {
      // 0 = IDLE
      cfg->net_l2->sendGantryStatus(
          0, 0, target_x, safe_z, 0.0f, cycle_time_ms);
    }
  }
}
"""

content = re.sub(r'static QueueHandle_t s_pick_queue = nullptr;.*?^}$', replacement, content, flags=re.MULTILINE | re.DOTALL)

with open("src/pick_scheduler.cpp", "w") as f:
    f.write(content)
