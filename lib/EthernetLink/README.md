# EthernetLink

Brings up the LAN8720 on the ESP32 EMAC: RMII, external 50 MHz clock into GPIO0, crystal enable on GPIO16. It applies a static IP or DHCP, and records link and IP events for the `boottime` console command.

This link carries CellNet L2, the TCP console and TCP OTA. EtherNet/IP stays on the W5500.

## Maturity

Runs on the gantry. G-E7 is open and concerns link state. G-B5 covers the stored `eth_ip` key, which this component does not read.

## Dependencies

ESP-IDF `esp_eth`, `esp_netif`, `esp_event`, `freertos`, `log`, `espressif__lan87xx` and `nvs_flash`. GPIO and `esp_timer` are private. Addresses come from `include/ethernet_app_config.h`.

## Consumers

`bootInitIo` in `src/main_init_io.cpp` calls `enablePhyOscillator`. `bootStartNetwork` in `src/main_init_net.cpp` calls `start` and `waitForUp`. The TCP console, TCP OTA server and CellNet node start only after that wait. `boottime` reads `ethernetBootTimes()` from `src/gantry_console_commands.cpp`.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-E7, G-B5.
