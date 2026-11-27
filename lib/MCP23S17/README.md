# MCP23S17

C driver for the MCP23S17 16-bit SPI GPIO expander on SPI3. It caches IODIR, GPPU and OLAT, and serialises register access with a per-handle mutex.

On this board the bus is owned by `Spi3Bus`. Callers pass `skip_bus_init = true` after `spi3::init()`.

## Maturity

Runs on the gantry. G-D3 and G-D6 are open and concern this driver.

## Dependencies

ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_common`, `log`, `freertos`, and `Spi3Bus`.

## Consumers

`GpioExpander` is the field and UI wrapper (`gpio_expander_init`). `Gantry` drives the gripper through that wrapper. `SpiDisplay` reads port B for the encoder and buttons and writes DC and RES.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-D3, G-D6.

## Pin map

From `include/gantry_app_constants.h:40–71`. The header comment in `MCP23S17.h` does not restate this map.

| Signal | Where |
|---|---|
| SCLK, MOSI, MISO | GPIO14, GPIO4, GPIO36 |
| MCP chip select | GPIO2, idle high |
| TFT chip select | GPIO32 (ESP GPIO, not an MCP pin) |
| TFT backlight | GPIO17 |
| PA0–PA3 | Field 24 V DOUT0–3. DOUT0 is the gripper |
| PA4–PA7 | Field 24 V DIN0–3 |
| PB0 | Module KO |
| PB1 | Encoder push |
| PB2 | Encoder B |
| PB3 | Encoder A |
| PB4 | Unused (backlight moved to GPIO17) |
| PB5 | TFT D/C |
| PB6 | TFT RES |
| PB7 | W5500 RSTn, active low |

Hardware address is `MCP23S17_HW_ADDR` (0). A0, A1 and A2 are tied to ground.
