# EthernetLink

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
EthernetLink*

**Read this when** you need to know what EthernetLink does, where it sits and
who uses it.

Brings up the LAN8720 on the ESP32 EMAC: RMII, external 50 MHz clock into GPIO0,
crystal enable on GPIO16. It applies a static IP or DHCP, and records link and
IP events for the `boottime` console command.

This link carries CellNet L2, the TCP console and TCP OTA. EtherNet/IP stays on
the W5500.

## Maturity

Runs on the gantry. G-E7 is open and concerns link state. The stored `eth_ip`
key has no reader (G-F2).

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

ESP-IDF `esp_eth`, `esp_netif`, `esp_event`, `freertos`, `log`,
`espressif__lan87xx` and `nvs_flash`. `esp_driver_gpio` and `esp_timer` are
private. Addresses come from `include/ethernet_app_config.h`.

## Consumers

`bootInitIo` in `src/main/main_init_io.cpp` calls `enablePhyOscillator`.
`bootStartNetwork` in `src/main/main_init_net.cpp` calls `start` and
`waitForUp`. The TCP console, TCP OTA server and CellNet node start only after
that wait. `boottime` reads `ethernetBootTimes()` from
`src/console/gantry_console_commands.cpp`.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-E7, G-F2.

## See also

-   **Down:** [EthernetLink reference](REFERENCE.md), the facts: API, timing,
    keys, errors.
-   **Up:** [Libraries](/lib/README.md).
