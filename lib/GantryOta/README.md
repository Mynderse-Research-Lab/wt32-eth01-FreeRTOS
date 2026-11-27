# GantryOta

Writes a firmware image into the passive OTA slot, with the motors off. Two ways in: a TCP server on the LAN8720, and a reboot into CellNet OTA-only mode (N-2).

## Maturity

Matured. N-2 is done, and the feature runs on the gantry.

## Dependencies

`Gantry` for the busy check and `disable()`. `CellNetL2` for framing, `OtaReceiver` and `EspEthL2Transport`. `EthernetLink` for the PHY in OTA-only mode. ESP-IDF `app_update`, `esp_app_format`, `esp_partition`, `esp_eth`, `esp_system`, `esp_timer`, `freertos`, `log`, `lwip`, `hal` and `soc`.

## Consumers

`app_main` calls `gantryOtaCellNetModeRequested` and, when set, `gantryOtaRunCellNetMode`, and later `gantryOtaConfirmBootValid` (`src/main.cpp`). `bootStartNetwork` in `src/main_init_net.cpp` calls `gantryOtaStartServer`. The pick scheduler calls `gantryOtaRebootToCellNetMode` (`src/pick_scheduler.cpp`). The console calls `gantryOtaPrintStatus`, `gantryOtaFlashStallTest` and `gantryResetReasonName`.

## Build and test

Registered from `idf/CMakeLists.txt`. The main component lists `GantryOta` in `REQUIRES`. There is no host test of these files. The receiver they drive is `test_cellnet_ota_receiver`.

## Open items

N-2 is done. N-1 records gantry OTA as that item's done part. G-E7 is the link wait in `src/main_init_net.cpp` that decides whether the TCP server starts. C-2 is the conveyor.
