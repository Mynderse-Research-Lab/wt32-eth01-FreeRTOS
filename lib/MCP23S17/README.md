# MCP23S17 SPI GPIO Expander Library

ESP-IDF driver for MCP23S17 16-bit SPI GPIO expander.

On WT32-ETH01 this part shares **SPI3** with the Amazon SPI TFT. Prefer
`skip_bus_init=true` after `spi3::init()` so the bus is owned by `lib/Spi3Bus`.

## WT32 pin map (SPI3)

| Signal | GPIO | Notes |
|--------|------|--------|
| SCLK | 14 | Shared with TFT |
| MOSI | 4 | Shared with TFT |
| MISO | 36 | MCP readback; TFT ignore |
| CS_MCP | 2 | Idle HIGH; default SPI3 client |
| TFT CS | MCP PB2 | Software CS (not ESP GPIO) |
| KO | MCP PB6 | Module key input |
| W5500 RST | MCP PB7 | Active-low; external pull-up |

Do **not** use GPIO 18/19/23 — those are LAN8720 RMII.

## Channel map (`device_address=0`, A0–A2=GND → 0x20)

| MCP | Function |
|-----|----------|
| PA0–PA3 | Field 24 V DOUT0–3 (DOUT0 = gripper) |
| PA4–PA7 | Field 24 V DIN0–3 |
| PB0 / PB1 | TFT DC / RES |
| PB2 | TFT CS (software CS on SPI3; idle HIGH) |
| PB3–PB5 | Encoder A / B / PUSH |
| PB6 | KO (module key input) |
| PB7 | W5500 RSTn (active-low; external pull-up) |

TFT BLK is hardwired ON (+5V); not MCP-driven.

## Usage (shared SPI3)

```c
#include "Spi3Bus.h"
#include "MCP23S17.h"
#include "gpio_expander.h"
#include "gantry_app_constants.h"

spi3::init();

mcp23s17_config_t config = {};
config.spi_host = spi3::host();
config.cs_pin = (gpio_num_t)SPI3_CS_MCP_GPIO;
config.device_address = MCP23S17_HW_ADDR;
config.clock_speed_hz = SPI3_MCP_CLOCK_HZ;
config.skip_bus_init = true;

gpio_expander_init(&config);
gpio_expander_configure_field_and_ui();
field_dout_set(0, true);
bool din0 = field_din_get(0);
```

## Hardware Connections

| MCP23S17 Pin | ESP32 Pin | Description |
|--------------|-----------|-------------|
| VDD | 3.3V | Power supply |
| VSS | GND | Ground |
| RESET | 3.3V | Reset (tie high) |
| CS | GPIO 2 | Chip select |
| SCK | GPIO 14 | SPI clock |
| SI (MOSI) | GPIO 4 | SPI data in |
| SO (MISO) | GPIO 36 | SPI data out |
| A0, A1, A2 | GND | Address 0x20 |
