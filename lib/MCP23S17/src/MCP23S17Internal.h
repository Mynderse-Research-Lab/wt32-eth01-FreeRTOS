/**
 * @file MCP23S17Internal.h
 * @brief Handle layout, register addresses, and the lock and register
 *        helpers shared by the MCP23S17 driver files.
 *
 * Private to this library; include it only from those files.
 */
#pragma once

#include "MCP23S17.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// MCP23S17 Register addresses
#define MCP23S17_IODIRA   0x00  // I/O Direction Port A
#define MCP23S17_IODIRB   0x01  // I/O Direction Port B
#define MCP23S17_IPOLA    0x02  // Input Polarity Port A
#define MCP23S17_IPOLB    0x03  // Input Polarity Port B
#define MCP23S17_GPINTENA 0x04  // Interrupt Enable Port A
#define MCP23S17_GPINTENB 0x05  // Interrupt Enable Port B
#define MCP23S17_DEFVALA  0x06  // Default Compare Port A
#define MCP23S17_DEFVALB  0x07  // Default Compare Port B
#define MCP23S17_INTCONA  0x08  // Interrupt Control Port A
#define MCP23S17_INTCONB  0x09  // Interrupt Control Port B
#define MCP23S17_IOCON    0x0A  // I/O Configuration (shared)
#define MCP23S17_IOCON_INIT 0x08  // HAEN; mcp23s17_verify_config() expects it
#define MCP23S17_GPPUA    0x0C  // Pull-up Resistor Port A
#define MCP23S17_GPPUB    0x0D  // Pull-up Resistor Port B
#define MCP23S17_INTFA    0x0E  // Interrupt Flag Port A
#define MCP23S17_INTFB    0x0F  // Interrupt Flag Port B
#define MCP23S17_INTCAPA  0x10  // Interrupt Capture Port A
#define MCP23S17_INTCAPB  0x11  // Interrupt Capture Port B
#define MCP23S17_GPIOA    0x12  // GPIO Port A
#define MCP23S17_GPIOB    0x13  // GPIO Port B
#define MCP23S17_OLATA    0x14  // Output Latch Port A
#define MCP23S17_OLATB    0x15  // Output Latch Port B

// MCP23S17 SPI opcode
#define MCP23S17_WRITE_OPCODE 0x40
#define MCP23S17_READ_OPCODE  0x41

struct mcp23s17_handle {
    spi_device_handle_t spi_device;
    spi_host_device_t spi_host;
    SemaphoreHandle_t spi_mutex;
    uint8_t device_address;
    uint8_t port_a_dir;
    uint8_t port_b_dir;
    uint8_t port_a_output;
    uint8_t port_b_output;
    uint8_t port_a_pullup;
    uint8_t port_b_pullup;
    bool owns_bus;
    // Current holder of spi_mutex, for the timeout log (G-D3).
    volatile int64_t held_since_us;
    char holder[16];
};

[[maybe_unused]] static const char *TAG = "MCP23S17";

bool mcp23s17_lock(mcp23s17_handle_t handle);
void mcp23s17_unlock(mcp23s17_handle_t handle);
esp_err_t mcp23s17_write_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t value);
esp_err_t mcp23s17_read_register_unlocked(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value);
esp_err_t mcp23s17_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value);
esp_err_t mcp23s17_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value);
