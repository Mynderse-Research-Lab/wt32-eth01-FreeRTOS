# MCP23S17 Integration

## Overview

This application uses an **MCP23S17** 16-bit SPI GPIO expander as the sole expansion-card IO interface. The MCP23S17 **must be present and initialized at runtime**; there is no direct-GPIO fallback mode.

Libraries (`Gantry`, `SDF08NK8X`, `MCP23S17`) are **pin-number agnostic** — they receive all pin assignments through config structs at startup. Only `main.cpp` and `gantry_app_constants.h` define the physical pin mapping.

## SPI wiring (ESP32 → MCP23S17)

> **WT32-ETH01 note:** GPIO 18, 19, 23 are internally wired to the LAN8720A Ethernet PHY
> and are **not available** for user IO. SPI is routed to free, exposed GPIOs instead.
> See `WT32_ETH01_PINOUT.md` for the full board pin map.

| Signal | ESP32 GPIO | MCP23S17 Pin | Boot note |
|--------|-----------|--------------|-----------|
| MISO   | **4**     | SO           | No constraints |
| MOSI   | **15**    | SI           | Must be HIGH at boot (default pull-up OK) |
| SCLK   | **14**    | SCK          | No constraints |
| CS     | **5**     | CS           | No constraints |

**Device address:** `0x00` (A0=A1=A2=GND)
**SPI clock:** 10 MHz

## MCP23S17 pin assignments (0–15)

### Port A (pins 0–7)

| MCP Pin | Symbol         | Direction    | Function                         |
|---------|----------------|-------------|----------------------------------|
| 0       | *(reserved)*   | —           | —                                |
| 1       | `PIN_DIR`      | Output      | X-axis direction (Servo SIGN/DIR)|
| 2       | `PIN_ENABLE`   | Output      | X-axis servo enable (SON)        |
| 3       | `PIN_LIMIT_MIN`| Input (pull-up) | X-axis home limit switch (active low) |
| 4       | `PIN_LIMIT_MAX`| Input (pull-up) | X-axis end limit switch (active low)  |
| 5       | `PIN_Y_STEP`   | Output      | Y-axis stepper pulse             |
| 6       | `PIN_Y_DIR`    | Output      | Y-axis stepper direction         |
| 7       | `PIN_Y_ENABLE` | Output      | Y-axis stepper enable            |

### Port B (pins 8–15)

| MCP Pin | Symbol              | Direction       | Function                              |
|---------|---------------------|-----------------|---------------------------------------|
| 8       | `PIN_GRIPPER`       | Output          | End-effector gripper control           |
| 9       | `PIN_LED`           | Output          | Status LED                             |
| 10      | `PIN_X_POS_REACHED` | Input (pull-up) | X-axis position reached (Drive OUT1)   |
| 11      | `PIN_X_BRAKE_STATUS`| Input (pull-up) | X-axis brake status (Drive OUT2)       |
| 12      | `PIN_X_ALARM_STATUS`| Input (pull-up) | X-axis alarm status (Drive OUT3)       |
| 13      | `PIN_X_ALARM_RESET` | Output          | X-axis alarm reset (Drive IN1/ARST)    |
| 14      | `PIN_X_CWCCW_PROHIB`| Output          | X-axis CW/CCW prohibition (Drive IN3)  |
| 15      | `PIN_X_PULSE_INHIB` | Output          | X-axis pulse inhibit (Drive IN5/INH)   |

## Direct ESP32 GPIO pins

These signals **cannot** go through the MCP23S17 because they require hardware peripherals (LEDC, PCNT):

| Symbol         | ESP32 GPIO | Direction | Function                       | Boot note |
|----------------|-----------|-----------|--------------------------------|-----------|
| `PIN_PULSE`    | 32        | Output    | X-axis pulse (LEDC hardware)   | No constraints |
| `PIN_ENC_A`    | 35        | Input     | X-axis encoder A (PCNT)        | Input-only; no pull resistors |
| `PIN_ENC_B`    | 36        | Input     | X-axis encoder B (PCNT)        | Input-only; no pull resistors |
| `PIN_THETA_PWM`| **2**     | Output    | Theta servo PWM (LEDC)         | Floats at boot = normal boot OK |

## GPIO Expander Abstraction (`gpio_expander`)

The `gpio_expander` layer provides a unified read/write API:

- **Pins 0–15** → routed to MCP23S17 via SPI.
- **Pins ≥ 16** → treated as direct ESP32 GPIO numbers.

`gpio_expander_init()` requires a **non-NULL** `mcp23s17_config_t*`. There is no fallback "direct-only" mode. If MCP23S17 initialization fails, the system logs an error and GPIO operations on pins 0–15 will fail.

## Runtime Verification

Use serial console commands:

- `pins` — prints the active runtime pin mapping used by `main.cpp`.
- `limits` — reads and prints active limit switch states via `gpio_expander_read()`.

The `pins` command prints from runtime-configured values (not hardcoded display constants).

## Reference

- Pin defines: `include/gantry_app_constants.h`
- Full pinout spreadsheet: `pinout.csv`
- MCP23S17 driver: `lib/MCP23S17/src/`
- GPIO expander abstraction: `src/gpio_expander.c`, `include/gpio_expander.h`
