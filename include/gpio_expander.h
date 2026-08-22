/**
 * @file gpio_expander.h
 * @brief GPIO Expander Abstraction Layer for MCP23S17
 */

#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mcp23s17_config_t mcp23s17_config_t;
typedef struct mcp23s17_handle* mcp23s17_handle_t;

#define GPIO_DIRECT_PIN_BASE        0x10
#define GPIO_EXPANDER_DIRECT_FLAG   0x100
#define GPIO_EXPANDER_DIRECT_MASK   0x0FF
#define GPIO_EXPANDER_DIRECT_PIN(gpio_num) \
  (GPIO_EXPANDER_DIRECT_FLAG | ((int)(gpio_num) & GPIO_EXPANDER_DIRECT_MASK))

bool gpio_expander_init(const mcp23s17_config_t* mcp_config);
void gpio_expander_deinit(void);
esp_err_t gpio_expander_set_direction(int pin, bool is_output);
esp_err_t gpio_expander_set_pullup(int pin, bool enable);
esp_err_t gpio_expander_write(int pin, uint8_t level);
uint8_t gpio_expander_read(int pin);
mcp23s17_handle_t gpio_expander_get_mcp_handle(void);

esp_err_t gpio_expander_configure_field_and_ui(void);
esp_err_t field_dout_set(unsigned channel, bool level);
bool field_din_get(unsigned channel);

/** W5500 RSTn via MCP PB7 — for W5500Config.rst_set_level */
void gpio_expander_w5500_rst_set_level(void* ctx, int level);

#ifdef __cplusplus
}
#endif

#endif // GPIO_EXPANDER_H
