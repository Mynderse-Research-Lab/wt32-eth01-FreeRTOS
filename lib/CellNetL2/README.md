# CellNetL2

Raw-Ethernet (EtherType `0x88B5`) framing, dispatch and transports for the cell nodes, plus
the CellNet firmware-update receiver and sender. No domain logic.

- Wire specification, payloads, library use and diagnostics:
  [docs/CELL_NET_L2_COMMUNICATION_GUIDE.md](../../docs/CELL_NET_L2_COMMUNICATION_GUIDE.md)
- Firmware update over CellNet: [docs/CELLNET_OTA_PROTOCOL.md](../../docs/CELLNET_OTA_PROTOCOL.md)
- Message structs and IDs: `include/cell_net_l2_protocol.h`

| File | Contents |
| :--- | :--- |
| `src/CellNetL2Framing.*` | Frame builders and `parseFrame` (no OS dependencies) |
| `src/CellNetL2.*` | `CellNetL2Node`: send methods, callbacks, statistics |
| `src/IL2Transport.h` | Transport interface |
| `src/EspEthL2Transport.*` | ESP-IDF transport: Ethernet input hook, non-`0x88B5` frames go on to lwIP |
| `src/LinuxRawSocketL2Transport.h` | Linux `AF_PACKET` transport |
| `src/CellNetOtaReceiver.*`, `src/CellNetOtaSender.*` | Firmware update over CellNet |

Host tests: `test/host/test_cell_net_l2.cpp` and `test/host/test_cellnet_ota_receiver.cpp`.
