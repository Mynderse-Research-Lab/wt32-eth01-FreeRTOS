*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › W5500*

# W5500

**Read this when** you need to know what W5500 does, where it sits and who uses it.

Driver for the WIZnet W5500 on SPI2, the bus that carries EtherNet/IP Class 1. It initialises and recovers the chip, runs the hardware TCP and UDP sockets, and times the one bus mutex. `W5500Hal` is the register interface the socket code and the host tests share.

## Maturity

Runs on the gantry. G-B15, the UDP length field, was fixed and closed on 2026-11-27. G-B16, G-B17 and G-B18 are open. G-B5 covers the address, which comes from Kconfig.

## Dependencies

ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_timer`, `log` and `freertos`.

## Consumers

`bootInitMotion` in `src/main_init_motion.cpp` constructs the chip and applies the stored keys `eip_spi_khz`, `eip_spi_dma` and `eip_txwr_split` before `init`. `EtherNetIP` is the socket and Class 1 caller. Reset is MCP PB7 (`W5500_RST_GPIO` is passed as -1).

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests: `test_w5500_spi`, `test_w5500_diag`, `test_w5500_poll`, `test_w5500_socket_fastpath`, `test_w5500_deferred_log`, and `test_eip_socket_w5500` through the EtherNetIP adapter.

## Open items

G-B5, G-B16, G-B17, G-B18.

## Next

- **Down:** [W5500 reference](REFERENCE.md), the facts: API, timing, keys, errors.
- **Up:** [Libraries](../README.md).
