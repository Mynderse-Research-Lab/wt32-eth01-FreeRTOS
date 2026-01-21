// subordinate_layer.cpp  (Core 1 – draft)
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motion_config.h"
#include "rom/ets_sys.h"
#include <Arduino.h>

static const char *TAG = "SUBORDINATE";

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

#define STEP_PIN_X GPIO_NUM_12
#define STEP_PIN_Y GPIO_NUM_13
#define STEP_PIN_TH GPIO_NUM_14

static void init_step_pins(void) {
  DEBUG_LOG("Initializing step pins: X=%d, Y=%d, TH=%d", STEP_PIN_X, STEP_PIN_Y,
            STEP_PIN_TH);

  gpio_config_t cfg = {.pin_bit_mask = (1ULL << STEP_PIN_X) |
                                       (1ULL << STEP_PIN_Y) |
                                       (1ULL << STEP_PIN_TH),
                       .mode = GPIO_MODE_OUTPUT,
                       .pull_up_en = GPIO_PULLUP_DISABLE,
                       .pull_down_en = GPIO_PULLDOWN_DISABLE,
                       .intr_type = GPIO_INTR_DISABLE};

  esp_err_t result = gpio_config(&cfg);
  if (result != ESP_OK) {
    DEBUG_LOGW("GPIO config failed with error: %d", result);
  } else {
    DEBUG_LOG("GPIO config successful");
  }

  gpio_set_level(STEP_PIN_X, 0);
  gpio_set_level(STEP_PIN_Y, 0);
  gpio_set_level(STEP_PIN_TH, 0);

  DEBUG_LOG("Step pins initialized to LOW");
}

static inline void step_pulse(gpio_num_t pin) {
  gpio_set_level(pin, 1);
  gpio_set_level(pin, 0);
}

#define AXIS_TH AXIS_THETA

// Debug counters
static uint32_t total_steps_executed = 0;
static uint32_t profile_executions = 0;

void subordinate_layer_task(void *pvParameters) {
  (void)pvParameters;
  init_step_pins();

  ESP_LOGI(TAG, "Subordinate task started on core %d", xPortGetCoreID());
  DEBUG_LOG("Task stack high water mark: %d bytes",
            uxTaskGetStackHighWaterMark(NULL) * 4);

  for (;;) {
    if (!g_motion_profile.ready || g_motion_profile.executing) {
      // Only log periodically to avoid spam
      static uint32_t wait_count = 0;
      wait_count++;
      if (wait_count % 200 == 0) { // Log every ~1 second (200 * 5ms)
        DEBUG_LOGD("Waiting for profile... ready=%d, executing=%d",
                   g_motion_profile.ready, g_motion_profile.executing);
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    profile_executions++;
    DEBUG_LOG("=== Starting profile execution #%lu ===", profile_executions);
    DEBUG_LOG("Stack high water mark: %d bytes",
              uxTaskGetStackHighWaterMark(NULL) * 4);

    g_motion_profile.executing = true;
    g_motion_profile.completed = false;

    for (int axis = 0; axis < NUM_AXES; ++axis) {
      g_system_status.current_step[axis] = 0;
      g_system_status.current_segment[axis] = 0;
    }

    uint32_t profile_total_steps = 0;
    unsigned long start_time = millis();

    for (int seg = 0; seg < NUM_SEGMENTS; ++seg) {
      // Log segment progress periodically
      if (seg % 16 == 0) {
        DEBUG_LOG("Processing segment %d/%d", seg, NUM_SEGMENTS);
      }

      for (int axis = 0; axis < NUM_AXES; ++axis) {
        uint32_t steps = g_motion_profile.segment_steps[axis][seg];
        uint32_t delay_ticks = g_motion_profile.segment_delay[axis][seg];

        // Log first segment details for each axis
        if (seg == 0 && steps > 0) {
          DEBUG_LOG("Axis %d, Seg 0: steps=%lu, delay=%lu us", axis, steps,
                    delay_ticks);
        }

        for (uint32_t s = 0; s < steps; ++s) {
          switch (axis) {
          case AXIS_X:
            step_pulse(STEP_PIN_X);
            break;
          case AXIS_Y:
            step_pulse(STEP_PIN_Y);
            break;
          case AXIS_TH:
            step_pulse(STEP_PIN_TH);
            break;
          }
          g_system_status.current_step[axis]++;
          g_system_status.current_segment[axis] = seg;
          profile_total_steps++;
          total_steps_executed++;
          ets_delay_us(delay_ticks); // simple delay for prototype
        }
      }
    }

    unsigned long elapsed_time = millis() - start_time;

    g_motion_profile.executing = false;
    g_motion_profile.completed = true;

    ESP_LOGI(TAG, "Profile execution complete");
    DEBUG_LOG("Execution stats: steps=%lu, time=%lu ms, rate=%.1f steps/s",
              profile_total_steps, elapsed_time,
              elapsed_time > 0
                  ? (float)profile_total_steps / elapsed_time * 1000
                  : 0);
    DEBUG_LOG("Total steps executed (all time): %lu", total_steps_executed);

    // Log final position for each axis
    for (int axis = 0; axis < NUM_AXES; ++axis) {
      DEBUG_LOG("Axis %d final position: step %lu, segment %d", axis,
                g_system_status.current_step[axis],
                g_system_status.current_segment[axis]);
    }
  }
}
