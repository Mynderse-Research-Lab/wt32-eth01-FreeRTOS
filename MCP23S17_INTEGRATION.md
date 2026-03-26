# MCP23S17 Integration

## Overview

This application runs in **MCP23S17 mode only** (`APP_USE_MCP23S17=1`).

- X/Y axis digital control and status IO is routed through MCP23S17.
- End-effector (gripper) IO is routed through MCP23S17.
- Only hardware-peripheral pins remain direct on WT32:
  - X/Y encoder A/B on PCNT-capable direct GPIO.
  - X/Y pulse output and Theta PWM output on direct LEDC/PWM GPIO.

> Note: MCP23S17 cannot provide ESP32 hardware PCNT/LEDC peripherals, so encoder and PWM-class signals must remain on direct ESP32 GPIO.

## SPI wiring (WT32-ETH01 -> MCP23S17)

| Signal | WT32-ETH01 GPIO | MCP23S17 Pin | Direction | Notes |
|---|---:|---|---|---|
| CS | 5 | CS | Output | SPI chip select |
| MISO | 4 | SO | Input | Data from MCP23S17 to ESP32 |
| MOSI | 12 | SI | Output | Data from ESP32 to MCP23S17 (strapping pin: must stay LOW during reset) |
| SCLK | 14 | SCK | Output | SPI clock |

## MCP23S17 pin assignments

### Port A (GPA0..GPA7 / logical 0..7)

| Logical Pin | MCP Pin | Symbol | Direction | Function |
|---:|---|---|---|---|
| 0 | GPA0 | `PIN_X_DIR` | Output | X-axis direction |
| 1 | GPA1 | `PIN_X_ENABLE` | Output | X-axis enable |
| 2 | GPA2 | `PIN_X_LIMIT_MIN` | Input (pull-up) | X home limit (active-low) |
| 3 | GPA3 | `PIN_X_LIMIT_MAX` | Input (pull-up) | X end limit (active-low) |
| 4 | GPA4 | `PIN_X_ALARM_STATUS` | Input (pull-up) | X alarm input |
| 5 | GPA5 | `PIN_X_ALARM_RESET` | Output | X alarm reset output |
| 6 | GPA6 | *(available)* | - | Available for future assignment |
| 7 | GPA7 | `PIN_GRIPPER` | Output | End-effector/gripper control |

### Port B (GPB0..GPB7 / logical 8..15)

| Logical Pin | MCP Pin | Symbol | Direction | Function |
|---:|---|---|---|---|
| 8 | GPB0 | `PIN_Y_DIR` | Output | Y-axis direction |
| 9 | GPB1 | `PIN_Y_ENABLE` | Output | Y-axis enable |
| 10 | GPB2 | `PIN_Y_LIMIT_MIN` | Input (pull-up) | Y home limit (active-low) |
| 11 | GPB3 | `PIN_Y_LIMIT_MAX` | Input (pull-up) | Y end limit (active-low) |
| 12 | GPB4 | `PIN_Y_ALARM_STATUS` | Input (pull-up) | Y alarm input |
| 13 | GPB5 | `PIN_Y_ALARM_RESET` | Output | Y alarm reset output |
| 14 | GPB6 | `PIN_THETA_LIMIT_MIN` | Input (pull-up) | Theta minimum limit switch (active-low) |
| 15 | GPB7 | `PIN_THETA_LIMIT_MAX` | Input (pull-up) | Theta maximum limit switch (active-low) |

## Direct WT32 GPIO pins

| Symbol | WT32-ETH01 GPIO | Peripheral | Direction | Function |
|---|---:|---|---|---|
| `PIN_X_PULSE` | 32 | LEDC (ch 0) | Output | X pulse output |
| `PIN_Y_PULSE` | 33 | LEDC (ch 1 reserved) | Output | Y pulse output |
| `PIN_X_ENC_A` | 35 | PCNT (unit 0) | Input | X encoder A |
| `PIN_X_ENC_B` | 36 | PCNT (unit 0) | Input | X encoder B |
| `PIN_Y_ENC_A` | 39 | PCNT (unit 1) | Input | Y encoder A |
| `PIN_Y_ENC_B` | 2 | PCNT (unit 1) | Input | Y encoder B (strapping pin: keep LOW/floating at reset) |
| `PIN_THETA_PWM` | 15 | LEDC (ch 2) | Output | Theta PWM (strapping pin: keep HIGH at reset for normal boot logs) |

> X and Y are now configured with identical SDF08NK8X servo-driver paths: each axis uses dedicated pulse (LEDC), encoder (PCNT), alarm input/reset, and MCP23S17 digital control/status lines.

## GPIO Abstraction Behavior

`gpio_expander` behavior:

- pins `0..15` are MCP23S17 pins.
- pins `>=16` are direct ESP32 GPIO numbers.

## Runtime Verification

Use serial console commands:

- `pins`: prints active runtime pin mapping used by `main.cpp`.
- `limits`: reads and prints active limit switch states.

The `pins` command prints from runtime-configured values (not hardcoded display constants).

## Pinout Spreadsheets

- Main mapping: `pinout.csv`
