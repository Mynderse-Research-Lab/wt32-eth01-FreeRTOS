# CellNetL2 reference

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
[CellNetL2](README.md) › Reference*

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key
or a return value.

Public headers: `lib/CellNetL2/src/CellNetL2.h`, `CellNetL2Framing.h`,
`IL2Transport.h`, `EspEthL2Transport.h`, `CellNetOtaReceiver.h`. On Linux also
`CellNetOtaSender.h` and `LinuxRawSocketL2Transport.h`. Wire structs:
`include/cell_net_l2_protocol.h`.

The byte layout, message IDs and the OTA exchange are the two documents linked
from the README. This page does not restate them.

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

`CellNetStats` is `tx_frames`, `rx_frames`, `rx_invalid_frames`,
`rx_sequence_drops`.

Callbacks (`onHeartbeat`, `onVisionDetect`, `onConveyorSpeed`, `onGantryStatus`,
`onCellCommand`) are invoked from `processIncomingFrame`. On the gantry that
runs inside `EspEthL2Transport::rxHook`, installed with
`esp_eth_update_input_path` (`EspEthL2Transport.cpp:65`), on the `emac_rx` task.
A callback that blocks or logs holds that task.

`sequence_` is a plain `uint8_t` (`CellNetL2.h`). Sends from more than one task
can duplicate it.

## Time

`L2CellHeader::timestamp_us_low` is a `uint32_t`
(`include/cell_net_l2_protocol.h:72`). It wraps every 2^32 microseconds, about
71.6 minutes. There is no time-of-day message. N-1.

## OTA receiver

`CellNetOtaReceiver` (`CellNetOtaReceiver.h`):

| Constant | Value |
|---|---|
| `kBeginDeadlineMs` | 30,000 ms from boot to a valid `OTA_BEGIN` |
| `kProgressTimeoutMs` | 2,000 ms without a completed block or `OTA_END` |

`expired(now_ms)` is those two deadlines. The gantry task that calls it is
`lib/GantryOta`, not this component. The protocol document is the state machine.

## Linux sender

`runOtaSender` (`CellNetOtaSender.h:87`) blocks until the update ends.
`SenderResult::code` is 0 ok, 1 setup, 2 protocol, 3 no result, 4
`emac_missed != 0`, 5 reboot not confirmed (`CellNetOtaSender.h:61`). Phases
reported to the progress callback: `discover`, `reboot`, `begin`, `blocks`,
`end`, `confirm`.

The Linux transport receives its own outgoing frames, and its destructor closes
the socket before joining the receive thread. N-2.

## Errors

The `send*` calls return false before `begin` and when the transport does not
send the frame. `processIncomingFrame` returns false for a frame that does not
parse and counts it in `rx_invalid_frames`. `isReady` is false until `begin` and
while the transport reports the link down.

## AppConfig

None.

## Console

None registered by this component.

## Usage

```cpp
// src/main/main.cpp
static CellNet::EspEthL2Transport l2Transport;
static CellNet::CellNetL2Node l2Node(l2Transport, CellNodeId::GANTRY);
// src/main/main_init_net.cpp, after the LAN8720 link is up
if (l2Transport.attachEthHandle(ethernetLink.getEthHandle(), ethernetLink.getNetif()) == ESP_OK &&
    l2Node.begin()) {
    // node active
}
```

## Known issues

Found in the library review of 2026-11-26 (at `1caa98c`) and not tracked as open
items. Not re-checked since; confirm against the code before acting.

| # | Category | Location | Finding | Evidence |
|---|---|---|---|---|
| N1 | risk | `EspEthL2Transport.cpp:65, 73` | The receive hook and the undersized-frame register bit depend on the IDF version and may be lost if EthernetLink stops and restarts the driver | Suggested: check the hook and the bit after `esp_eth_stop`/`start` |
| N2 | bug, diagnostic | `CellNetL2.cpp:219-225` | A duplicate or reordered frame adds 255 to `rx_sequence_drops` | Confirmed |
| N3 | risk | `CellNetL2.cpp:79` and the other senders | `sequence_++` from several tasks can produce duplicate sequence numbers, which trigger N2 at the receiver | Confirmed |
| N4 | risk | `CellNetL2.h:96-100`; `EspEthL2Transport.cpp:102-104` | Callback assignments race with dispatch in `emac_rx` if they happen after the hook is live; boot order keeps them apart today | Suggested |
| N7 | test gap | `CellNetOtaSender.cpp`, `CellNetOtaSenderPhases.cpp`, `CellNetOtaSenderReport.cpp` | The sender has no host test and is not compiled in CI. A loopback test against `OtaReceiver` needs an interface for `Link` | Confirmed |
| N8 | docs | `IL2Transport.h:48-50`; `cell_net_l2_protocol.h:5`; `CellNetL2.cpp:33-34` | `isLinkUp` claims link state but means only that a handle exists; an unmeasured "sub-50-microsecond" claim; a comment on undersized frames that contradicts the transport | Confirmed |

## Documents

`lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md`,
`lib/CellNetL2/CELLNET_OTA_PROTOCOL.md`. Open items: N-1, N-2, C-3 in
`docs/OPEN_ITEMS.md`.

## See also

-   **Up:** [CellNetL2 README](README.md), [Libraries](/lib/README.md).
