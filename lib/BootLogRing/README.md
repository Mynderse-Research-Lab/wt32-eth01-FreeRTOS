# BootLogRing

Keeps the first log output after reset in RAM. The LAN console connects only after boot, so start-up logs would otherwise exist only on UART0.

## Maturity

Matured. No open item in `docs/OPEN_ITEMS.md` concerns this component.

## Dependencies

ESP-IDF `log` and `freertos`. The capture hook is `esp_log_set_vprintf`.

## Consumers

`app_main` calls `bootLogRingInstall` before any other `esp_log_set_vprintf` user (`src/main.cpp`). The test console command `bootlog` reads the ring (`src/gantry_console_commands.cpp`).

## Build and test

Registered from `idf/CMakeLists.txt`. The main component lists `BootLogRing` in `REQUIRES`. There is no host test: the hook uses the ESP-IDF log lock and a FreeRTOS critical section.

## Open items

None.
