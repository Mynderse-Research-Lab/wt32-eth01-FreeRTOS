// supervisory_layer.cpp  (Core 0 – draft)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motion_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SUPERVISORY";

// Enable verbose debug output
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
  #define DEBUG_LOG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
  #define DEBUG_LOGD(fmt, ...) ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
  #define DEBUG_LOGW(fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#else
  #define DEBUG_LOG(fmt, ...)
  #define DEBUG_LOGD(fmt, ...)
  #define DEBUG_LOGW(fmt, ...)
#endif

// Debug counters
static uint32_t profiles_generated = 0;
static uint32_t commands_received = 0;

static void generate_dummy_profile(const MotionCommand *cmd, MotionProfile *profile)
{
    DEBUG_LOG("=== Generating motion profile ===");
    
    memset(profile, 0, sizeof(MotionProfile));

    uint32_t total_steps_all_axes = 0;

    for (int axis = 0; axis < NUM_AXES; ++axis) {
        double dist = cmd->target_pos[axis] - cmd->start_pos[axis];
        uint32_t total_steps = (uint32_t)(dist > 0 ? dist : -dist);  // abs value

        DEBUG_LOG("Axis %d: start=%.2f, target=%.2f, dist=%.2f, steps=%lu",
                  axis, cmd->start_pos[axis], cmd->target_pos[axis], dist, total_steps);

        uint32_t per_seg = (NUM_SEGMENTS > 0) ? total_steps / NUM_SEGMENTS : 0;
        uint32_t remainder = (NUM_SEGMENTS > 0) ? total_steps % NUM_SEGMENTS : 0;

        DEBUG_LOGD("Axis %d: per_seg=%lu, remainder=%lu", axis, per_seg, remainder);

        for (int seg = 0; seg < NUM_SEGMENTS; ++seg) {
            uint32_t steps = per_seg + (seg < (int)remainder ? 1U : 0U);
            profile->segment_steps[axis][seg] = steps;
            profile->segment_delay[axis][seg] = 1000; // placeholder ticks (1000 us)
            total_steps_all_axes += steps;
        }
        profile->active_segments[axis] = NUM_SEGMENTS;
        
        DEBUG_LOG("Axis %d: active_segments=%d, total scheduled=%lu", 
                  axis, profile->active_segments[axis], total_steps);
    }

    profile->ready = true;
    profile->executing = false;
    profile->completed = false;
    
    profiles_generated++;
    
    DEBUG_LOG("Profile generation complete: total_steps=%lu, profile_count=%lu",
              total_steps_all_axes, profiles_generated);
    
    // Log velocity parameters
    DEBUG_LOG("Velocity params: v_start=%.1f, v_peak=%.1f, accel_segments=%d",
              cmd->v_start, cmd->v_peak, cmd->accel_segments);
}

void supervisory_layer_task(void *pvParameters)
{
    (void)pvParameters;
    
    ESP_LOGI(TAG, "Supervisory task started on core %d", xPortGetCoreID());
    DEBUG_LOG("Task stack high water mark: %d bytes", uxTaskGetStackHighWaterMark(NULL) * 4);

    // Track time for periodic status reports
    unsigned long lastStatusTime = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        // Periodic status report every 10 seconds
        unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - lastStatusTime > 10000) {
            lastStatusTime = now;
            DEBUG_LOGD("Status: commands_received=%lu, profiles_generated=%lu",
                       commands_received, profiles_generated);
            DEBUG_LOGD("Profile state: ready=%d, executing=%d, completed=%d",
                       g_motion_profile.ready, g_motion_profile.executing, 
                       g_motion_profile.completed);
            DEBUG_LOGD("Stack high water mark: %d bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }

        if (g_motion_command.valid) {
            commands_received++;
            DEBUG_LOG("=== Motion command #%lu received ===", commands_received);
            
            // Log command details
            DEBUG_LOG("Command valid flag: %d", g_motion_command.valid);
            for (int axis = 0; axis < NUM_AXES; ++axis) {
                if (g_motion_command.target_pos[axis] != g_motion_command.start_pos[axis]) {
                    DEBUG_LOG("Axis %d: %.2f -> %.2f", axis,
                              g_motion_command.start_pos[axis],
                              g_motion_command.target_pos[axis]);
                }
            }
            
            MotionCommand cmd_local;
            memcpy(&cmd_local, &g_motion_command, sizeof(MotionCommand));
            g_motion_command.valid = false;
            DEBUG_LOG("Command copied, valid flag cleared");

            // Check if subordinate is still executing
            if (g_motion_profile.executing) {
                DEBUG_LOGW("Previous profile still executing! Waiting...");
                while (g_motion_profile.executing) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                DEBUG_LOG("Previous profile completed, proceeding with new profile");
            }

            generate_dummy_profile(&cmd_local, &g_motion_profile);
            ESP_LOGI(TAG, "Dummy profile generated");
        }
    }
}
