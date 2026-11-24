#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
namespace HostTest {
// Tests set this to make MCP output writes fail.
inline esp_err_t gpio_expander_write_result = ESP_OK;
// Latch read-back: follows the last write unless a test pins it (0/1),
// e.g. 0 to model a silent MCP23S17 that reads every register as 0x00.
inline uint8_t gpio_expander_last_level = 0;
inline int gpio_expander_latch_override = -1;
}
#define HOST_GPIO_EXPANDER_WRITE_RESULT HostTest::gpio_expander_write_result
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
#ifdef __cplusplus
static inline esp_err_t gpio_expander_write(int, uint8_t level) {
    if (HostTest::gpio_expander_write_result == ESP_OK) HostTest::gpio_expander_last_level = level;
    return HOST_GPIO_EXPANDER_WRITE_RESULT;
}
static inline esp_err_t gpio_expander_read_latch(int, uint8_t* level) {
    *level = (HostTest::gpio_expander_latch_override >= 0)
                 ? (uint8_t)HostTest::gpio_expander_latch_override
                 : HostTest::gpio_expander_last_level;
    return ESP_OK;
}
#else
static inline esp_err_t gpio_expander_write(int, uint8_t) { return ESP_OK; }
static inline esp_err_t gpio_expander_read_latch(int, uint8_t* level) { *level = 0; return ESP_OK; }
#endif
static inline uint8_t gpio_expander_read(int) { return 1; }

#ifdef __cplusplus
}
#endif
