# MCP23S17

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) › MCP23S17*

**Read this when** you need to know what MCP23S17 does, where it sits and who
uses it.

C driver for the MCP23S17 16-bit SPI GPIO expander on SPI3. It caches IODIR,
GPPU and OLAT, and serialises register access with a per-handle mutex.

On this board the bus is owned by `Spi3Bus`. Callers pass `skip_bus_init = true`
after `spi3::init()`.

## Maturity

Runs on the gantry. G-D3 and G-D6 are open and concern this driver.

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_common`, `log`, `freertos`,
and `Spi3Bus`.

## Consumers

`GpioExpander` is the field and UI wrapper (`gpio_expander_init`). `Gantry`
drives the gripper through that wrapper. `SpiDisplay` reads port B for the
encoder and buttons and writes DC and RES.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-D3, G-D6.

## Pin map

The pin map is the `SPI3_*`, `MCP_*` and `TFT_*` constants in
`include/gantry_app_constants.h:40–71`, one comment per pin. DOUT0 (PA0) is the
gripper valve and PB7 is the W5500 reset, active low.

Hardware address is `MCP23S17_HW_ADDR` (0). A0, A1 and A2 are tied to ground.

## See also

-   **Down:** [MCP23S17 reference](REFERENCE.md), the facts: API, timing, keys,
    errors.
-   **Up:** [Libraries](/lib/README.md).
