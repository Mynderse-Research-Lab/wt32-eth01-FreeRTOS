# MCP23S17 Integration and Direct GPIO Test Mode

## Overview

The application supports two X-axis IO modes:

- `APP_USE_MCP23S17=1`: Extended IO through MCP23S17.
- `APP_USE_MCP23S17=0`: Temporary direct WT32 GPIO test mode (no MCP hardware connected).

In both modes:

- Limit switches are required for any active X-axis.
- Encoder pins stay on direct ESP32 GPIO.
- Theta servo PWM stays on direct ESP32 GPIO.

## Active Modes

### MCP Mode (`APP_USE_MCP23S17=1`)

SPI wiring:

- CS: GPIO 5
- MISO: GPIO 19
- MOSI: GPIO 23
- SCLK: GPIO 18

X-axis signals:

- Pulse: `PIN_PULSE` (direct GPIO 32, LEDC)
- DIR / ENABLE / status and control lines: MCP pins
- Limits: MCP `PIN_LIMIT_MIN` / `PIN_LIMIT_MAX`

### Temporary Direct Mode (`APP_USE_MCP23S17=0`)

Direct WT32 mapping is aligned to `SDF08NK8X-Driver-library` test app:

- X Pulse: GPIO 2
- X DIR: GPIO 12
- X ENABLE: GPIO 15
- X ALM input: GPIO 17
- X Limit MIN: GPIO 33
- X Limit MAX: GPIO 5
- X ALM reset: disabled (`-1`) in this mode

Additional direct pins still used:

- Encoder A: GPIO 35
- Encoder B: GPIO 36
- Theta PWM: GPIO 13

## GPIO Abstraction Behavior

`gpio_expander` now behaves as:

- MCP initialized: pins `0..15` are MCP pins, others are direct GPIO.
- MCP not initialized (`gpio_expander_init(NULL)`): all pins are treated as direct GPIO.

This is required so direct-mode mappings like GPIO `2`, `5`, `12`, and `15` work correctly.

## Runtime Verification

Use serial console commands:

- `pins`: prints active runtime pin mapping used by `main.cpp`.
- `limits`: reads and prints active limit switch states.

The `pins` command prints from runtime-configured values (not hardcoded display constants).

## Pinout Spreadsheets

- Main mapping: `pinout.csv`
- Temporary direct-mode mapping: `pinout_temporary_direct_gpio.csv`
