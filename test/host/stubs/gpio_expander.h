#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_DIRECT_PIN_BASE        0x10
#define GPIO_EXPANDER_DIRECT_FLAG   0x100
#define GPIO_EXPANDER_DIRECT_MASK   0x0FF
#define GPIO_EXPANDER_DIRECT_PIN(gpio_num) \
  (GPIO_EXPANDER_DIRECT_FLAG | ((int)(gpio_num) & GPIO_EXPANDER_DIRECT_MASK))

static inline esp_err_t gpio_expander_set_direction(int, bool) { return ESP_OK; }
static inline esp_err_t gpio_expander_set_pullup(int, bool) { return ESP_OK; }
static inline esp_err_t gpio_expander_write(int, uint8_t) { return ESP_OK; }
static inline uint8_t gpio_expander_read(int) { return 1; }

#ifdef __cplusplus
}
#endif
