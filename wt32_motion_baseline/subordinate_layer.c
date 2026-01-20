\
// subordinate_layer.c  (Core 1 – draft)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "motion_config.h"
#include "esp_log.h"

static const char *TAG = "SUBORDINATE";

#define STEP_PIN_X   GPIO_NUM_12
#define STEP_PIN_Y   GPIO_NUM_13
#define STEP_PIN_TH  GPIO_NUM_14

static void init_step_pins(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << STEP_PIN_X) |
                        (1ULL << STEP_PIN_Y) |
                        (1ULL << STEP_PIN_TH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(STEP_PIN_X, 0);
    gpio_set_level(STEP_PIN_Y, 0);
    gpio_set_level(STEP_PIN_TH, 0);
}

static inline void step_pulse(gpio_num_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}

void subordinate_layer_task(void *pvParameters)
{
    (void)pvParameters;
    init_step_pins();
    ESP_LOGI(TAG, "Subordinate task started on core %d", xPortGetCoreID());

    for (;;) {
        if (!g_motion_profile.ready || g_motion_profile.executing) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        g_motion_profile.executing = true;
        g_motion_profile.completed = false;

        for (int axis = 0; axis < NUM_AXES; ++axis) {
            g_system_status.current_step[axis] = 0;
            g_system_status.current_segment[axis] = 0;
        }

        for (int seg = 0; seg < NUM_SEGMENTS; ++seg) {
            for (int axis = 0; axis < NUM_AXES; ++axis) {
                uint32_t steps = g_motion_profile.segment_steps[axis][seg];
                uint32_t delay_ticks = g_motion_profile.segment_delay[axis][seg];

                for (uint32_t s = 0; s < steps; ++s) {
                    switch (axis) {
                        case AXIS_X: step_pulse(STEP_PIN_X); break;
                        case AXIS_Y: step_pulse(STEP_PIN_Y); break;
                        case AXIS_TH: step_pulse(STEP_PIN_TH); break;
                    }
                    g_system_status.current_step[axis]++;
                    g_system_status.current_segment[axis] = seg;
                    ets_delay_us(delay_ticks); // simple delay for prototype
                }
            }
        }

        g_motion_profile.executing = false;
        g_motion_profile.completed = true;
        ESP_LOGI(TAG, "Profile execution complete");
    }
}
