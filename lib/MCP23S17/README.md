# MCP23S17 SPI GPIO Expander Library

ESP-IDF driver for MCP23S17 16-bit SPI GPIO expander.

## Features

- SPI communication interface
- 16 GPIO pins (Port A: 0-7, Port B: 8-15)
- Input/output configuration
- Pull-up resistor control
- Port-level read/write operations
- Interrupt support (configured but not fully implemented)

## Pin Mapping

- **Port A (Pins 0-7)**: General purpose I/O
- **Port B (Pins 8-15)**: General purpose I/O

## Usage

```c
#include "MCP23S17.h"

// Configure MCP23S17
mcp23s17_config_t config = {};
config.spi_host = SPI2_HOST;
config.cs_pin = GPIO_NUM_5;
config.miso_pin = GPIO_NUM_19;
config.mosi_pin = GPIO_NUM_23;
config.sclk_pin = GPIO_NUM_18;
config.device_address = 0x00;  // A0=A1=A2=GND
config.clock_speed_hz = 10000000;

// Initialize
mcp23s17_handle_t handle = mcp23s17_init(&config);

// Configure pin as output
mcp23s17_set_pin_direction(handle, 0, true);  // Pin 0 = output

// Write pin
mcp23s17_write_pin(handle, 0, 1);  // Set pin 0 HIGH

// Configure pin as input with pull-up
mcp23s17_set_pin_direction(handle, 3, false);  // Pin 3 = input
mcp23s17_set_pin_pullup(handle, 3, true);

// Read pin
uint8_t level = mcp23s17_read_pin(handle, 3);
```

## Hardware Connections

| MCP23S17 Pin | ESP32 Pin | Description |
|--------------|-----------|-------------|
| VDD | 3.3V | Power supply |
| VSS | GND | Ground |
| RESET | 3.3V | Reset (tie high) |
| CS | GPIO 5 | Chip select |
| SCK | GPIO 18 | SPI clock |
| SI (MOSI) | GPIO 23 | SPI data in |
| SO (MISO) | GPIO 19 | SPI data out |
| A0, A1, A2 | GND | Address pins (device address 0x20) |
| INTA, INTB | (Optional) | Interrupt outputs |

## Device Address

Device address is determined by A0, A1, A2 pins:
- All GND: 0x20 (default)
- A0=VDD, A1=A2=GND: 0x21
- etc.

Set `device_address` in config to match hardware (0x00-0x07, representing lower 3 bits).
