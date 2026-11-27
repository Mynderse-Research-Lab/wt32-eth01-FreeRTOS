# CellNetL2 reference

Public headers: `lib/CellNetL2/src/CellNetL2.h`, `CellNetL2Framing.h`, `IL2Transport.h`, `EspEthL2Transport.h`, `CellNetOtaReceiver.h`. On Linux also `CellNetOtaSender.h` and `LinuxRawSocketL2Transport.h`. Wire structs: `include/cell_net_l2_protocol.h`.

The byte layout, message IDs and the OTA exchange are the two documents linked from the README. This page does not restate them.

## Node API

`CellNetL2Node` (`CellNetL2.h`):

```c
bool begin();
bool isReady() const;
bool sendHeartbeat(uint32_t uptime_ms, uint16_t status_flags = 0, const uint8_t dest_mac[6] = nullptr);
bool sendVisionDetect(...);   // millimetres, degrees, class, optional capture displacement
bool sendConveyorSpeed(float speed_mm_s, float displacement_m, int32_t raw_encoder_cnt, ...);
bool sendGantryStatus(...);   // positions in mm and degrees
bool sendCellCommand(uint8_t command_id, uint8_t param_u8 = 0, uint16_t param_u16 = 0, float param_float = 0.0f, ...);
bool processIncomingFrame(const uint8_t* buffer, size_t length);
CellNetStats getStats() const;
```

`CellNetStats` is `tx_frames`, `rx_frames`, `rx_invalid_frames`, `rx_sequence_drops`.

Callbacks (`onHeartbeat`, `onVisionDetect`, `onConveyorSpeed`, `onGantryStatus`, `onCellCommand`) are invoked from `processIncomingFrame`. On the gantry that runs inside `EspEthL2Transport::rxHook`, installed with `esp_eth_update_input_path` (`EspEthL2Transport.cpp:65`), on the `emac_rx` task. A callback that blocks or logs holds that task.

`sequence_` is a plain `uint8_t` (`CellNetL2.h`). Sends from more than one task can duplicate it.

## Time

`L2CellHeader::timestamp_us_low` is a `uint32_t` (`include/cell_net_l2_protocol.h:72`). It wraps every 2^32 microseconds, about 71.6 minutes. There is no time-of-day message. N-1.

## OTA receiver

`CellNetOtaReceiver` (`CellNetOtaReceiver.h`):

| Constant | Value |
|---|---|
| `kBeginDeadlineMs` | 30,000 ms from boot to a valid `OTA_BEGIN` |
| `kProgressTimeoutMs` | 2,000 ms without a completed block or `OTA_END` |

`expired(now_ms)` is those two deadlines. The gantry task that calls it is `lib/GantryOta`, not this component. The protocol document is the state machine.

## Linux sender

`runOtaSender` (`CellNetOtaSender.h:87`) blocks until the update ends. `SenderResult::code` is 0 ok, 1 setup, 2 protocol, 3 no result, 4 `emac_missed != 0`, 5 reboot not confirmed (`CellNetOtaSender.h:61`). Phases reported to the progress callback: `discover`, `reboot`, `begin`, `blocks`, `end`, `confirm`.

The Linux transport receives its own outgoing frames, and its destructor closes the socket before joining the receive thread. N-2.

## Errors

The `send*` calls return false before `begin` and when the transport does not send the frame. `processIncomingFrame` returns false for a frame that does not parse and counts it in `rx_invalid_frames`. `isReady` is false until `begin` and while the transport reports the link down.

## AppConfig

None.

## Console

None registered by this component.

## Usage

```cpp
// src/main.cpp
static CellNet::EspEthL2Transport l2Transport;
static CellNet::CellNetL2Node l2Node(l2Transport, CellNodeId::GANTRY);
// src/main_init_net.cpp, after the LAN8720 link is up
if (l2Transport.attachEthHandle(ethernetLink.getEthHandle(), ethernetLink.getNetif()) == ESP_OK &&
    l2Node.begin()) {
    // node active
}
```

## Documents

`lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md`, `lib/CellNetL2/CELLNET_OTA_PROTOCOL.md`. Open items: N-1, N-2, C-3 in `docs/OPEN_ITEMS.md`.
