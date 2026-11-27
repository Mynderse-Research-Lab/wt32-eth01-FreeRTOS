# W5500

Driver for the WIZnet W5500 on SPI2, the bus that carries EtherNet/IP Class 1. It initialises and recovers the chip, runs the hardware TCP and UDP sockets, and times the one bus mutex. `W5500Hal` is the register interface the socket code and the host tests share.

## Maturity

Runs on the gantry. G-B15, the UDP length field, was fixed and closed on 2026-11-27. G-B16, G-B17 and G-B18 are open. G-B5 covers the address, which comes from Kconfig.

## Dependencies

ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_timer`, `log` and `freertos`.

## Consumers

`bootInitMotion` in `src/main_init_motion.cpp` constructs the chip and applies `eip_w5500_spi_khz`, `eip_spi_dma` and `eip_txwr_split` before `init`. `EtherNetIP` is the socket and Class 1 caller. Reset is MCP PB7 (`W5500_RST_GPIO` is passed as -1).

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests: `test_w5500_spi`, `test_w5500_diag`, `test_w5500_poll`, `test_w5500_socket_fastpath`, `test_w5500_deferred_log`.

## Open items

G-B5, G-B15, G-B16, G-B17, G-B18.
