# GantryOta

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
GantryOta*

**Read this when** you need to know what GantryOta does, where it sits and who
uses it.

Writes a firmware image into the passive OTA slot, with the motors off. Two ways
in: a TCP server on the LAN8720, and a reboot into CellNet OTA-only mode (N-2).

## Maturity

Matured, and it runs on the gantry. The gantry side of N-2 is done; N-2 stays
open until the supervisor branch `feat/cellnet-ota` is pushed.

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

`Gantry` for the busy check and `disable()`. `CellNetL2` for framing,
`OtaReceiver` and `EspEthL2Transport`. `EthernetLink` for the PHY in OTA-only
mode. ESP-IDF `app_update`, `esp_app_format`, `esp_partition`, `esp_eth`,
`esp_system`, `esp_timer`, `freertos`, `log`, `lwip`, `hal` and `soc`.

## Consumers

`app_main` calls `gantryOtaCellNetModeRequested` and, when set,
`gantryOtaRunCellNetMode`, and later `gantryOtaConfirmBootValid`
(`src/main/main.cpp`). `bootStartNetwork` in `src/main/main_init_net.cpp` calls
`gantryOtaStartServer`. The pick scheduler calls `gantryOtaRebootToCellNetMode`
(`src/pick_scheduler/pick_scheduler.cpp`). The console calls
`gantryOtaPrintStatus`, `gantryOtaFlashStallTest` and `gantryResetReasonName`.

## Build and test

Registered from `idf/CMakeLists.txt`. The main component lists `GantryOta` in
`REQUIRES`. There is no host test of these files. The receiver they drive is
`test_cellnet_ota_receiver`.

## Open items

N-2 (gantry side done). N-1 records gantry OTA as that item's done part. G-E7 is
the link wait in `src/main/main_init_net.cpp` that decides whether the TCP
server starts. C-2 is the conveyor.

## See also

-   **Down:** [GantryOta reference](REFERENCE.md), the facts: API, timing, keys,
    errors.
-   **Up:** [Libraries](/lib/README.md).
