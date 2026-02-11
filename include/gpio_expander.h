/**
 * @file gpio_expander.h
 * @brief GPIO Expander Abstraction Layer for MCP23S17
 * @version 1.0.0
 * 
 * Provides GPIO abstraction that works with both direct ESP32 GPIO
 * and MCP23S17 GPIO expander. Maps logical pin numbers to physical pins.
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

// Pin mapping: Logical pin numbers mapped to MCP23S17 or direct GPIO
// MCP23S17 pins: 0-15 (Port A: 0-7, Port B: 8-15)
// Direct GPIO: 16-255 (raw ESP32 GPIO numbers, no offset encoding)

#define GPIO_EXPANDER_PIN_BASE 0x00  // MCP23S17 pins start at 0
#define GPIO_DIRECT_PIN_BASE   0x10  // Values >= 16 are treated as direct GPIO

// Pin type flags
#define PIN_TYPE_MCP23S17  0x00
#define PIN_TYPE_DIRECT    0x10

/**
 * @brief Initialize GPIO expander system
 * 
 * @param mcp_config MCP23S17 configuration (NULL to disable MCP23S17)
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
 * @param pin Logical pin number (0-15 for MCP23S17, 16+ for direct GPIO)
 * @param is_output true for output, false for input
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_set_direction(uint8_t pin, bool is_output);

/**
 * @brief Set pin pull-up resistor
 * 
 * @param pin Logical pin number
 * @param enable true to enable pull-up
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_set_pullup(uint8_t pin, bool enable);

/**
 * @brief Write pin level
 * 
 * @param pin Logical pin number
 * @param level Pin level (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_write(uint8_t pin, uint8_t level);

/**
 * @brief Read pin level
 * 
 * @param pin Logical pin number
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
