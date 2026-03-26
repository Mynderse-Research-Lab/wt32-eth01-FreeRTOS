/**
 * @file MCP23S17.h
 * @brief MCP23S17 SPI GPIO Expander Driver for ESP-IDF
 * @version 1.0.0
 * 
 * Provides GPIO abstraction for MCP23S17 16-bit SPI GPIO expander.
 * Supports both direct GPIO and MCP23S17 GPIO operations.
 */

#ifndef MCP23S17_H
#define MCP23S17_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MCP23S17 GPIO port (A or B)
 */
typedef enum {
    MCP23S17_PORT_A = 0,
    MCP23S17_PORT_B = 1
} mcp23s17_port_t;

/**
 * @brief MCP23S17 pin number (0-15, where 0-7 = Port A, 8-15 = Port B)
 */
typedef uint8_t mcp23s17_pin_t;

/**
 * @brief MCP23S17 handle
 */
typedef struct mcp23s17_handle* mcp23s17_handle_t;

/**
 * @brief MCP23S17 configuration structure
 */
typedef struct mcp23s17_config_t {
    spi_host_device_t spi_host;      // SPI host (SPI2_HOST or SPI3_HOST)
    gpio_num_t cs_pin;               // Chip select pin
    gpio_num_t miso_pin;             // MISO pin
    gpio_num_t mosi_pin;             // MOSI pin
    gpio_num_t sclk_pin;             // SCLK pin
    uint8_t device_address;          // Device address (0x20-0x27, default 0x20)
    uint32_t clock_speed_hz;         // SPI clock speed (default 10MHz)
} mcp23s17_config_t;

/**
 * @brief Initialize MCP23S17 GPIO expander
 * 
 * @param config Configuration structure
 * @return Handle on success, NULL on failure
 */
mcp23s17_handle_t mcp23s17_init(const mcp23s17_config_t* config);

/**
 * @brief Deinitialize MCP23S17 GPIO expander
 * 
 * @param handle MCP23S17 handle
 */
void mcp23s17_deinit(mcp23s17_handle_t handle);

/**
 * @brief Set pin direction (input/output)
 * 
 * @param handle MCP23S17 handle
 * @param pin Pin number (0-15)
 * @param is_output true for output, false for input
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_set_pin_direction(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool is_output);

/**
 * @brief Set pin pull-up resistor
 * 
 * @param handle MCP23S17 handle
 * @param pin Pin number (0-15)
 * @param enable true to enable pull-up
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_set_pin_pullup(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool enable);

/**
 * @brief Write pin level
 * 
 * @param handle MCP23S17 handle
 * @param pin Pin number (0-15)
 * @param level Pin level (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_write_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin, uint8_t level);

/**
 * @brief Read pin level
 * 
 * @param handle MCP23S17 handle
 * @param pin Pin number (0-15)
 * @return Pin level (0 or 1)
 */
uint8_t mcp23s17_read_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin);

/**
 * @brief Write port (all 8 pins of port A or B)
 * 
 * @param handle MCP23S17 handle
 * @param port Port (A or B)
 * @param value 8-bit value
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_write_port(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t value);

/**
 * @brief Read port (all 8 pins of port A or B)
 * 
 * @param handle MCP23S17 handle
 * @param port Port (A or B)
 * @return 8-bit port value
 */
uint8_t mcp23s17_read_port(mcp23s17_handle_t handle, mcp23s17_port_t port);

/**
 * @brief Configure pin interrupt
 * 
 * @param handle MCP23S17 handle
 * @param pin Pin number (0-15)
 * @param enable true to enable interrupt
 * @param trigger_on_rising true for rising edge, false for falling edge
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_set_pin_interrupt(mcp23s17_handle_t handle, mcp23s17_pin_t pin, 
                                      bool enable, bool trigger_on_rising);

/**
 * @brief Raw register read helper for diagnostics.
 *
 * @param handle MCP23S17 handle
 * @param reg Register address
 * @param value Output value
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_debug_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t *value);

/**
 * @brief Raw register write helper for diagnostics.
 *
 * @param handle MCP23S17 handle
 * @param reg Register address
 * @param value Value to write
 * @return ESP_OK on success
 */
esp_err_t mcp23s17_debug_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // MCP23S17_H
