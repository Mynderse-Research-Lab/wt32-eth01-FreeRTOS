# GpioExpander

Wraps one MCP23S17 for the field outputs and inputs, the panel encoder, the TFT control lines, and the W5500 reset pin. Pins below `GPIO_DIRECT_PIN_BASE` (16) are chip pins. Any other pin is rejected.

## Maturity

Matured. G-D3 and G-D6 do not concern this wrapper. G-D3 is the SPI3 priority ceiling for the MCP23S17 lock. G-D6 is the MCP23S17 driver (`MCP23S17.cpp`), the second SPI3 lock in `Spi3Bus`, and the console `mcp_reg` command.

## Dependencies

`MCP23S17`, ESP-IDF `log` and `esp_common`. Pin numbers and field-channel counts come from `include/gantry_app_constants.h`.

## Consumers

`Gantry` (end effector and limit switches) requires this component. `bootInitIo` in `src/main_init_io.cpp` initialises it and passes the handle to the display; `app_main` calls `bootInitIo`. The test console and the pick scheduler call it from `src/`.

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests do not compile this file. `test/host/stubs/gpio_expander.h` stands in for the header.

## Open items

None for this wrapper. G-D3 and G-D6 stay on MCP23S17 and Spi3Bus.
