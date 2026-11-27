# BootLogRing

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
BootLogRing*

**Read this when** you need to know what BootLogRing does, where it sits and who
uses it.

Keeps the first log output after reset in RAM. The LAN console connects only
after boot, so start-up logs would otherwise exist only on UART0.

## Role

Diagnostic support library. Only the `bootlog` console command reads it; motion,
picks and the Class 1 scanner do not depend on it. It is installed first in
`app_main` and stays in the log path for the whole uptime; the cost is in
[Task and timing](REFERENCE.md#task-and-timing). Removing it means deleting
`bootLogRingInstall` from `app_main`, the `bootlog` command, its help line and
include, and `BootLogRing` from both CMake lists.

## Maturity

Matured. No open item in `docs/OPEN_ITEMS.md` concerns this component.

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

ESP-IDF `log` and `freertos`. The capture hook is `esp_log_set_vprintf`.

## Consumers

`app_main` calls `bootLogRingInstall` before any other `esp_log_set_vprintf`
user (`src/main.cpp`). The test console command `bootlog` reads the ring
(`src/gantry_console_commands.cpp`).

## Build and test

Registered from `idf/CMakeLists.txt`. The main component lists `BootLogRing` in
`REQUIRES`. There is no host test: the hook uses the ESP-IDF log lock and a
FreeRTOS critical section.

## Open items

None.

## See also

-   **Down:** [BootLogRing reference](REFERENCE.md), the facts: API, timing,
    keys, errors.
-   **Up:** [Libraries](/lib/README.md).
