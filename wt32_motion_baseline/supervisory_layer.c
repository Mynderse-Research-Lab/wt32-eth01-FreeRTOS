\
// supervisory_layer.c  (Core 0 – draft)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motion_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SUPERVISORY";

static void generate_dummy_profile(const MotionCommand *cmd, MotionProfile *profile)
{
    memset(profile, 0, sizeof(MotionProfile));

    for (int axis = 0; axis < NUM_AXES; ++axis) {
        double dist = cmd->target_pos[axis] - cmd->start_pos[axis];
        uint32_t total_steps = (uint32_t)(dist);

        uint32_t per_seg = (NUM_SEGMENTS > 0) ? total_steps / NUM_SEGMENTS : 0;
        uint32_t remainder = (NUM_SEGMENTS > 0) ? total_steps % NUM_SEGMENTS : 0;

        for (int seg = 0; seg < NUM_SEGMENTS; ++seg) {
            uint32_t steps = per_seg + (seg < (int)remainder ? 1U : 0U);
            profile->segment_steps[axis][seg] = steps;
            profile->segment_delay[axis][seg] = 1000; // placeholder ticks
        }
        profile->active_segments[axis] = NUM_SEGMENTS;
    }

    profile->ready = true;
    profile->executing = false;
    profile->completed = false;
}

void supervisory_layer_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Supervisory task started on core %d", xPortGetCoreID());

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (g_motion_command.valid) {
            MotionCommand cmd_local;
            memcpy(&cmd_local, &g_motion_command, sizeof(MotionCommand));
            g_motion_command.valid = false;

            generate_dummy_profile(&cmd_local, &g_motion_profile);
            ESP_LOGI(TAG, "Dummy profile generated");
        }
    }
}
