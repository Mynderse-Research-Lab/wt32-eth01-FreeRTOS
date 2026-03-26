/**
 * @file gpio_expander.h
 * @brief GPIO Expander Abstraction Layer for MCP23S17
 * @version 1.0.0
 * 
 * Provides GPIO abstraction for MCP23S17 GPIO expander pins only.
 */

#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations to avoid pulling MCP23S17 SPI internals into this public header.
typedef struct mcp23s17_config_t mcp23s17_config_t;
typedef struct mcp23s17_handle* mcp23s17_handle_t;

// MCP23S17 logical pin mapping for this module: 0..15 only.
#define GPIO_EXPANDER_PIN_BASE 0x00
#define GPIO_EXPANDER_PIN_MAX  0x0F

/**
 * @brief Initialize GPIO expander system
 * 
 * @param mcp_config MCP23S17 configuration (must be non-NULL)
 * @return true on success
 */
bool gpio_expander_init(const mcp23s17_config_t* mcp_config);

/**
 * @brief Deinitialize GPIO expander system
 */
void gpio_expander_deinit(void);

/**
 * @brief Configure pin as input or output
 * 
 * @param pin Logical MCP23S17 pin number (0-15)
 * @param is_output true for output, false for input
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_set_direction(uint8_t pin, bool is_output);

/**
 * @brief Set pin pull-up resistor
 * 
 * @param pin Logical MCP23S17 pin number (0-15)
 * @param enable true to enable pull-up
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_set_pullup(uint8_t pin, bool enable);

/**
 * @brief Write pin level
 * 
 * @param pin Logical MCP23S17 pin number (0-15)
 * @param level Pin level (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_write(uint8_t pin, uint8_t level);

/**
 * @brief Read pin level
 * 
 * @param pin Logical MCP23S17 pin number (0-15)
 * @return Pin level (0 or 1)
 */
uint8_t gpio_expander_read(uint8_t pin);

/**
 * @brief Get MCP23S17 handle (for advanced operations)
 * 
 * @return MCP23S17 handle or NULL
 */
mcp23s17_handle_t gpio_expander_get_mcp_handle(void);

#ifdef __cplusplus
}
#endif

#endif // GPIO_EXPANDER_H
