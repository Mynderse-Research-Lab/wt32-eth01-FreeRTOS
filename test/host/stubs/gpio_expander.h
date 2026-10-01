#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
namespace HostTest {
// Tests set this to make MCP output writes fail.
inline esp_err_t gpio_expander_write_result = ESP_OK;
}
#define HOST_GPIO_EXPANDER_WRITE_RESULT HostTest::gpio_expander_write_result
#else
#define HOST_GPIO_EXPANDER_WRITE_RESULT ESP_OK
#endif

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
static inline esp_err_t gpio_expander_write(int, uint8_t) { return HOST_GPIO_EXPANDER_WRITE_RESULT; }
static inline uint8_t gpio_expander_read(int) { return 1; }

#ifdef __cplusplus
}
#endif
