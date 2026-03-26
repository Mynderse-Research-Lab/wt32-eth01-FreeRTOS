\
// main.c  (WT32-ETH02 – draft baseline)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motion_config.h"

MotionCommand g_motion_command = {0};
MotionProfile g_motion_profile = {0};
SystemStatus  g_system_status  = {0};

void supervisory_layer_task(void *pvParameters);
void subordinate_layer_task(void *pvParameters);

void app_main(void)
{
    ESP_LOGI("MAIN", "Starting WT32-ETH02 motion baseline (draft)");

    xTaskCreatePinnedToCore(
        supervisory_layer_task,
        "Supervisory",
        4096,
        NULL,
        3,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        subordinate_layer_task,
        "Subordinate",
        4096,
        NULL,
        4,
        NULL,
        1
    );

    vTaskDelay(pdMS_TO_TICKS(1000));
    g_motion_command.start_pos[AXIS_X] = 0.0;
    g_motion_command.target_pos[AXIS_X] = 1000.0;
    g_motion_command.v_start = 1000.0;
    g_motion_command.v_peak  = 5000.0;
    g_motion_command.accel_segments = 16;
    g_motion_command.valid = true;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
