# Spi3Bus

Owns SPI3. The MCP23S17 (chip select GPIO2) and the ST7789 (chip select GPIO32) share this bus. The W5500 is on SPI2 and does not use this component.

## Maturity

Runs on the gantry. G-D3 and G-D6 are open and concern this bus.

## Dependencies

ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `log` and `freertos`. Pin numbers and clock rates come from `include/gantry_app_constants.h`.

## Consumers

`bootInitIo` calls `spi3::init`. `MCP23S17` and `SpiDisplay` take the bus. The EtherNet/IP scanner calls `spi3_class1_critical_enter` and `spi3_class1_critical_exit` around a Class 1 exchange.

## Build and test

Registered from `idf/CMakeLists.txt`. Host test: `test_spi3_critical` (the critical-section counter). The SPI driver itself is not host-tested.

## Open items

G-D3, G-D6.
