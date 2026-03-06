# MCP23S17 Integration

## Overview

This application uses an MCP23S17 SPI GPIO expander for all expansion-card digital IO.
The MCP23S17 is required at runtime.

Key design notes:

- Limit switches are required for any active X-axis.
- Encoder pins stay on direct ESP32 GPIO.
- Theta servo PWM stays on direct ESP32 GPIO.

## Wiring (SPI)

- CS: GPIO 5
- MISO: GPIO 19
- MOSI: GPIO 23
- SCLK: GPIO 18

## X-axis signals

- Pulse: `PIN_PULSE` (direct GPIO 32, LEDC)
- DIR / ENABLE / status and control lines: MCP pins
- Limits: MCP `PIN_LIMIT_MIN` / `PIN_LIMIT_MAX`

## GPIO Abstraction Behavior

`gpio_expander` now behaves as:

- Pins `0..15` are MCP pins (require MCP23S17 init).
- Pins `>= 16` are treated as direct ESP32 GPIO numbers.

## Runtime Verification

Use serial console commands:

- `pins`: prints active runtime pin mapping used by `main.cpp`.
- `limits`: reads and prints active limit switch states.

The `pins` command prints from runtime-configured values (not hardcoded display constants).

## Pinout Spreadsheets

- Main mapping: `pinout.csv`
