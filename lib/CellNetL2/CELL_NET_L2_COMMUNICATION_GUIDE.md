*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › [CellNetL2](README.md) › Wire guide*

# CellNet Layer-2 Communication Guide

**Read this when** you implement or debug a node that talks CellNet L2.

**Status:** Canonical wire specification for CellNet  
**EtherType:** `0x88B5`  
**Source of truth:** `include/cell_net_l2_protocol.h` (the supervisor vendors a copy)  
**Companion Docs:** [CELLNET_OTA_PROTOCOL.md](CELLNET_OTA_PROTOCOL.md), [GANTRY_FIRMWARE.md](../../GANTRY_FIRMWARE.md), [INDEX.md](../../docs/INDEX.md)

---

## 1. Scope

`lib/CellNetL2` packs, sends, validates and dispatches fixed-size frames over raw Ethernet
between the cell nodes on the closed cell switch. It holds no domain logic: coordinate
transforms, belt tracking and motion stay in the applications (`pick_scheduler.cpp` on the
gantry).

| Node | ID | Platform |
| :--- | :--- | :--- |
| Vision | `0x01` | Ubuntu x86_64 |
| Conveyor | `0x02` | WT32-ETH01 (separate firmware, own copy of the header) |
| Gantry | `0x03` | WT32-ETH01 (`EspEthL2Transport`) |
| Supervisor | `0x04` | Raspberry Pi ARM64 (`LinuxRawSocketL2Transport`) |

The protocol has no checksum of its own; it relies on the Ethernet FCS checked by the MAC.

---

## 2. Framing

```
┌───────────────────────────┬──────────────────────────────────────────────────────┬─────────────┐
│  Ethernet Header (14 B)   │                  Cell Header (8 B)                   │   Payload   │
├───────────┬───────────────┼─────────┬─────────┬──────────┬──────────┬───────────┼─────────────┤
│ Dest MAC  │  Source MAC   │ Version │ MsgType │ SenderID │ Sequence │ Timestamp │ Typed, fixed│
│ (6 B)     │  (6 B)        │ (0x01)  │ (1 B)   │ (1 B)    │ (1 B)    │ (4 B, µs) │ size        │
└───────────┴───────────────┴─────────┴─────────┴──────────┴──────────┴───────────┴─────────────┘
```

- All structs are `#pragma pack(push, 1)`. The EtherType is big-endian on the wire; header
  and payload fields are copied in host byte order (`memcpy`), and every current node is
  little-endian.
- `dest_mac` defaults to broadcast; every `send*` method takes an optional unicast MAC.
- `sequence` is one rolling counter per sending node, shared by all message types.
- `timestamp_us_low` is the low 32 bits of the sender's microsecond clock
  (`esp_timer_get_time()` on ESP32, `steady_clock` on Linux).
- **Padding:** `CellNetL2Node` zero-pads every frame to the 60-byte Ethernet minimum.
  `EspEthL2Transport` also tells the EMAC to forward undersized frames, so a peer that does
  not pad still gets through.

### Receive rules (`CellNetL2Framing::parseFrame`)

A frame is accepted when the EtherType is `0x88B5`, `version == 0x01`, the type is known,
and its length is at least the message size and at most 64 bytes (`kMaxPaddedRxLen`:
60 bytes plus a kept FCS). `OTA_DATA` alone may be up to 508 bytes. Bytes past the message
size are ignored. Anything else counts as `rx_invalid_frames`.

### Message types

| Type | ID | Direction | Frame |
| :--- | :--- | :--- | :--- |
| `HEARTBEAT` | `0x00` | Any → broadcast | 30 B |
| `VISION_DETECT` | `0x01` | Vision → gantry, supervisor | 58 B |
| `CONVEYOR_SPEED` | `0x02` | Conveyor → gantry, supervisor | 34 B |
| `GANTRY_STATUS` | `0x03` | Gantry → supervisor | 40 B |
| `CELL_COMMAND` | `0x04` | Supervisor → gantry, conveyor | 30 B |
| `OTA_BEGIN` … `OTA_RESULT` | `0x05`–`0x09` | Supervisor ↔ node in OTA mode | see [CELLNET_OTA_PROTOCOL.md](CELLNET_OTA_PROTOCOL.md) |

`CellNetL2Node` dispatches types `0x00`–`0x04`. The OTA types are parsed by the same framing
code but handled by the OTA-mode receive path (`lib/GantryOta/src/gantry_cellnet_ota.cpp` with
`CellNetOtaReceiver`) and by `CellNetOtaSender` on the supervisor.

### Payloads

#### `HEARTBEAT` (0x00)
```cpp
struct L2HeartbeatPayload {
  uint32_t uptime_ms;
  uint16_t status_flags;
  uint8_t reserved[2];
};
```

#### `VISION_DETECT` (0x01, payload 36 bytes)
```cpp
struct L2VisionDetectPayload {
  uint32_t item_id;              // 0
  float x_across_mm;             // 4  centroid across the belt
  float y_bat_mm;                // 8  camera-relative, +Y downstream
  float theta_deg;               // 12 long axis: 0° along +X, 90° along +Y
  uint8_t battery_class;         // 16 1–10 pick, 11–13 pack occupant
  uint8_t reserved[3];           // 17 bit0 required: capture_displacement_m is valid
  float capture_displacement_m;  // 20 metres at exposure; the bit must be set
  float width_mm;                // 24 jaws close across this; required, (0, 80]
  float length_mm;               // 28 in-plane long axis; required
  float height_from_belt_mm;     // 32 belt surface to the top; required
};
```

`y_bat_mm` is measured from the camera center at the capture instant; the gantry adds
`vis_cam_y_mm` to get world Y (+Y downstream).

**Frames.** The vision frame is right-handed: +X across the belt, +Y downstream, +Z up,
z = 0 on the belt. `theta_deg` is the cell long axis in that frame: 0° along +X, 90° along
+Y, positive about +Z, in `[-180°, 180°)`. The gantry joint frame is rotated 180° about Y:
\(X_g = -X_v\), \(Y_g = Y_v\), \(Z_g = -Z_v\) (down), and the belt surface is the
calibrated joint \(z_{\max}\). The gantry pose frame (`GantryKinematics`) keeps the vision
axes, so detections enter it with offsets only (`vis_x_off_mm`, `vis_cam_y_mm`).

**Theta joint.** The joint holds the pads along +Y at 0°, and a positive joint turns them
counter-clockwise seen from above, about \(+Z_v\) (checked on the Pi camera on 2026-11-16: at
+45° the +X finger moved downstream). So
\(\theta_{\mathrm{joint}} = \theta_{\mathrm{pose}} - 90^\circ\). For a side grip the pads run
along the long axis, \(\theta_{\mathrm{pose}} = \mathrm{fold}(\theta) +\) `jaw_off_deg`
(0° by default), which gives
\[
\theta_{\mathrm{joint}} = \mathrm{fold}(\theta) + \texttt{jaw\_off\_deg} - 90^\circ + k \cdot 180^\circ ,
\]
with \(k\) choosing the in-range joint with the least travel (`chooseSideJaw`). A cell along
the belt (\(\theta = 90^\circ\)) is gripped at joint 0°, one across the belt at ±90°.
`item_id` is nonzero and stays the same across repeated observations of one object in a
conveyor epoch. `battery_class` `1`–`10` is a pick: AAA, AA, D, C, 9V, CR123A, 18650,
21700, 26650, 32650. `11`–`13` is a pack occupant that is tracked and not picked: laptop,
Li-ion pack, power-tool. It selects the tunable jaw-time slot for a pick class and does
not supply a size. `width_mm`, `length_mm`,
and `height_from_belt_mm` are all required. Width is in `(0, 80]` mm. `reserved[0]` bit 0
must be set and `capture_displacement_m` is the conveyor displacement at exposure. A missing
field is rejected. Pick envelopes, length × width × height above the belt: AAA
`44.5 × 10.5 × 10.5`, AA `50.5 × 14.5 × 14.5`, D `61.5 × 34.2 × 34.2`, C `50 × 26.2 × 26.2`,
9V `48.5 × 17.5 × 26.5`, CR123A `34.5 × 17 × 17`, 18650 `65 × 18.3 × 18.3`, 21700
`70 × 21.2 × 21.2`, 26650 `65 × 26.5 × 26.5`, 32650 `65 × 32 × 32`. Packs kept in view:
laptop `205 × 50 × 22`, Li-ion pack `114 × 66 × 25`, power-tool `120 × 75 × 80`. Grasp height is \(\min(h/2, 25.4\,\mathrm{mm})\)
down from the object top. The frame is 58 bytes, so a 60-byte padded frame still parses.
Detections are handed to the pick task through a queue, so the Ethernet receive task never
runs the planner.

#### `CONVEYOR_SPEED` (0x02)
```cpp
struct L2ConveyorSpeedPayload {
  float speed_mm_s;
  float displacement_m;    // cumulative belt travel, metres
  int32_t raw_encoder_cnt; // 500-PPR PCNT value
};
```

`displacement_m` is the belt position. `speed_mm_s` only projects that sample until the next report. The two fields and the frame timestamp are one sample. The grasp deadlines are target travels on that anchor.

#### `GANTRY_STATUS` (0x03)
```cpp
struct L2GantryStatusPayload {
  uint8_t motion_state;      // 0=IDLE, 1=HOMING, 2=INTERCEPTING, 3=GRIP_COMMANDED (hold unverified), 4=FAULT
  uint8_t active_slot;       // 0=ota_0, 1=ota_1
  uint16_t fault_flags;
  float x_pos_mm;
  float z_pos_mm;
  float theta_deg;
  float last_cycle_time_ms;
};
```

#### `CELL_COMMAND` (0x04)
```cpp
struct L2CellCommandPayload {
  uint8_t command_id;  // 1=START, 2=STOP, 3=PAUSE, 4=CLEAR_FAULT, 5=PARK,
                       // 6=SET_GRIPPER (param_u8=0|1), 7=REBOOT_TO_OTA (param_u16=OTA session)
  uint8_t param_u8;
  uint16_t param_u16;
  float param_float;
};
```

---

## 3. Using the library

The API is in `lib/CellNetL2/src/CellNetL2.h`: one `send*` method and one `on*` callback per
message type `0x00`–`0x04`, plus `begin()`, `isReady()`, `getStats()` and `resetStats()`.

**ESP-IDF.** Once the LAN8720 is up, call
`EspEthL2Transport::attachEthHandle(eth_handle, netif)` and then `CellNetL2Node::begin()`
(`src/main_init_net.cpp`). The transport hooks the Ethernet input path: `0x88B5` frames go to CellNet,
every other frame goes on to lwIP, so the TCP console and OTA server share the port. Callbacks
run in the EMAC receive task; keep them short and hand work to a queue.
`EspEthL2Transport::isLinkUp()` only reports that a handle is attached, not the PHY link.

**Linux.** Use `lib/CellNetL2/src/LinuxRawSocketL2Transport.h` (an `AF_PACKET` socket bound
to one interface and to `0x88B5`). Callbacks run on its receive thread. Compile
`CellNetL2.cpp` and `CellNetL2Framing.cpp` with `-I include -I lib/CellNetL2/src`, C++17,
`-lpthread`. The binary needs `CAP_NET_RAW` (`sudo setcap cap_net_raw+ep <binary>`) or root.

---

## 4. Diagnostics

`getStats()` returns `tx_frames`, `rx_frames`, `rx_invalid_frames` and `rx_sequence_drops`.

| Symptom | Cause |
| :--- | :--- |
| `rx_invalid_frames` rises | Version mismatch, unknown type, or a length outside the receive rules (section 2). Both transports already filter on EtherType. |
| `rx_sequence_drops` rises | Frames lost between sender and receiver, or a sender restarted: its counter starts again at 0 and the gap counts as drops. |
| `sendFrame()` returns false | No handle attached (ESP32) or the socket is closed (Linux), or the driver refused the frame. |
| Linux `socket(AF_PACKET)` fails with `EPERM` | Missing `CAP_NET_RAW`. |

---

## 5. Host tests

`test/host/test_cell_net_l2.cpp` covers frame sizes, round trips for every type (OTA
included), rejection of a wrong EtherType, a wrong version and truncated frames, padded
frames, callback dispatch and sequence-loss counting. The OTA receiver has its own
`test_cellnet_ota_receiver.cpp`. Run them with the repository testing gate (`CONTRIBUTING.md`).

## Next

- **Up:** [CellNetL2 README](README.md).
