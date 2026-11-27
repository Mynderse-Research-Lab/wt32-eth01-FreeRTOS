# CellNetL2

Raw Ethernet (EtherType `0x88B5`) framing, dispatch and transports for the cell nodes, plus the CellNet firmware-update receiver and the Linux sender. No domain logic.

## Maturity

The gantry runs the node, the ESP32 transport and the OTA receiver. N-1 and N-2 are open. C-3 is the conveyor copy of the same frames.

## Dependencies

ESP-IDF `esp_eth`, `esp_netif`, `esp_timer`, `log` and `freertos`. Message structs are `include/cell_net_l2_protocol.h`.

## What is built

The firmware component compiles `CellNetL2Framing.cpp`, `CellNetL2.cpp`, `CellNetOtaReceiver.cpp` and `EspEthL2Transport.cpp`.

`CellNetOtaSender.cpp` and `LinuxRawSocketL2Transport.h` are Linux-only. The sender header is empty unless `__linux__` is defined. They are not in this component's `SRCS`, and they are not in the host-test build.

## Consumers

`app_main` constructs `EspEthL2Transport` and `CellNetL2Node` for `CellNodeId::GANTRY`. `bootStartNetwork` attaches the Ethernet handle after the link is up. The OTA-only boot path uses `CellNetOtaReceiver` from `lib/GantryOta`.

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests: `test_cell_net_l2` and `test_cellnet_ota_receiver`. The sender has no host test.

## Open items

N-1, N-2, C-3.

## Documents

- Wire specification: [lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md)
- Firmware update: [lib/CellNetL2/CELLNET_OTA_PROTOCOL.md](CELLNET_OTA_PROTOCOL.md)
