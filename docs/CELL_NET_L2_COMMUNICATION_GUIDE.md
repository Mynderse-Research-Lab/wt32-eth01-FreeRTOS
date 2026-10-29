# CellNet Layer-2 Communication Library Guide & Deployment Manual

**Status:** Canonical communication specification & deployment manual  
**EtherType:** `0x88B5` (IEEE 802.3 Local / Real-Time Protocol)  
**Target Systems:** WT32-ETH01 (ESP32), Linux x86_64 (Ubuntu IPC), Linux ARM64 (Raspberry Pi)  
**Companion Docs:** [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md), [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md), [INDEX.md](INDEX.md)

---

## 1. Overview & Architectural Scope

The **`CellNetL2`** library provides high-speed, deterministic, and hardware-independent **OSI Layer-2 communication** across all battery sorting cell subsystems over raw Ethernet (`0x88B5`).

```
┌──────────────────────────────────────────────────────────────────────────────────────────┐
│                   CLOSED CELL NETWORK (Layer-2 Switch / Unmanaged 100BASE-TX)            │
│                                                                                          │
│   ┌────────────────────┐   ┌────────────────────┐   ┌────────────────────────────────┐   │
│   │   Ubuntu IPC       │   │  Conveyor WT32     │   │     Raspberry Pi               │   │
│   │ (Vision Detection) │   │ (500-PPR Tracking) │   │ (Supervisor / Cell Gate)       │   │
│   │  Node ID: 0x01     │   │  Node ID: 0x02     │   │  Node ID: 0x04                 │   │
│   └─────────┬──────────┘   └─────────┬──────────┘   └───────────────┬────────────────┘   │
│             │                        │                              │                    │
│             ▼                        ▼                              ▼                    │
│   ═══════════════════════════ UNMANAGED SWITCH (100BASE-TX) ══════════════════════════   │
│                                      ▲                                                   │
│                                      │                                                   │
│                            ┌─────────┴──────────┐                                        │
│                            │    Gantry WT32     │                                        │
│                            │  (Motion Control)  │                                        │
│                            │   Node ID: 0x03    │                                        │
│                            └────────────────────┘                                        │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```

### Strict Boundary: Pure Data Transport
The sole responsibility of `lib/CellNetL2` is to **reliably package, transmit, validate, and dispatch raw messages between nodes**. 
- The library contains **zero domain calculations, kinematics, or trajectory algorithms**.
- All coordinate transformations, conveyor dead-reckoning, and motion math remain strictly inside application layers (e.g. `pick_scheduler.cpp` on Gantry, belt tracker on Conveyor, image classifier on Vision).

---

## 2. Data Integrity & Transport Guarantees

Because industrial automation environments experience significant electromagnetic interference (EMI) from servo drives and variable frequency drives, `CellNetL2` implements four independent hardware and software data integrity mechanisms:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 1. Hardware CRC-32 (FCS)  ──► Auto-drops any frame with bitflips in silicon     │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 2. DMA Ring Buffers       ──► Packets queued separately in RAM (no overwrite)   │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 3. Exact Length Checks    ──► Rejects malformed, truncated, or oversized frames │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 4. #pragma pack(push, 1)  ──► Guarantees byte alignment across CPU architectures│
└─────────────────────────────────────────────────────────────────────────────────┘
```

1. **Hardware CRC-32 (FCS)**: Every frame is verified by the receiving Ethernet MAC silicon. Bit-flipped or electrically corrupted packets are dropped before reaching RAM.
2. **DMA Ring Buffer Queues**: Multi-node simultaneous transmissions land in independent RAM descriptors without overwriting or race conditions.
3. **Exact Length Validation**: The library strictly verifies that `length == sizeof(ExpectedFrame)` before reading any payload bytes.
4. **1-Byte Struct Packing (`#pragma pack(push, 1)`)**: Eliminates compiler padding differences between 32-bit Xtensa (ESP32), 64-bit ARM (Raspberry Pi), and 64-bit x86 (Ubuntu).
5. **Independent Sequence Counters (`0..255`)**: Tracks packet delivery per sender ID to immediately detect dropped or out-of-order frames.
6. **Zero-LwIP Overhead**: Bypasses the TCP/IP stack entirely, achieving $< 50\text{ }\mu\text{s}$ transit and dispatch latency.

---

## 3. Protocol Framing & Wire Specification

Every frame transmitted over the network has the following byte structure:

```
┌───────────────────────────┬──────────────────────────────────────────┬────────────────────────┐
│  Ethernet Header (14 B)   │           Cell Header (8 B)              │   Payload (Variable)   │
├───────────┬───────────────┼──────────┬──────────┬──────────┬─────────┼────────────────────────┤
│ Dest MAC  │   Source MAC  │ Version  │ MsgType  │ SenderID │ Seq No. │  Typed Payload Data    │
│ (6 Bytes) │   (6 Bytes)   │ (0x01)   │ (0x00..4)│ (0x01..4)│ (0..255)│  (Speed / Detect / etc)│
└───────────┴───────────────┴──────────┴──────────┴──────────┴─────────┴────────────────────────┘
```

### Common Headers

```cpp
#pragma pack(push, 1)

struct L2EthernetHeader {
  uint8_t dest_mac[6]; // Broadcast FF:FF:FF:FF:FF:FF or targeted Unicast
  uint8_t src_mac[6];  // Local NIC hardware MAC address
  uint16_t ethertype;  // Big-Endian 0x88B5 (Network Byte Order: 0x88, 0xB5)
};

struct L2CellHeader {
  uint8_t version;            // CELL_NET_L2_VERSION (0x01)
  uint8_t msg_type;           // CellMsgType (0x00..0x04)
  uint8_t sender_id;          // CellNodeId (0x01..0x04)
  uint8_t sequence;           // Rolling sequence counter (0..255)
  uint32_t timestamp_us_low;  // Microsecond timestamp at sampling instant
};
```

### Message Types & Payloads

#### 1. Heartbeat (`MsgType::HEARTBEAT = 0x00`, Total Size: 30 Bytes)
```cpp
struct L2HeartbeatPayload {
  uint32_t uptime_ms;     // Node uptime in milliseconds
  uint16_t status_flags;  // Subsystem status bitmask
  uint8_t reserved[2];
};
```

#### 2. Vision Detection (`MsgType::VISION_DETECT = 0x01`, frame 58 bytes, payload 36)
```cpp
struct L2VisionDetectPayload {
  uint32_t item_id;              // 0
  float x_across_mm;             // 4  centroid across the belt
  float y_bat_mm;                // 8  camera-relative, +Y downstream
  float theta_deg;               // 12 long axis: 0° along +Y, + toward −X
  uint8_t battery_class;         // 16 1–10 pick, 11–13 pack occupant
  uint8_t reserved[3];           // 17 bit0 required: capture_displacement_m is valid
  float capture_displacement_m;  // 20 metres at exposure; the bit must be set
  float width_mm;                // 24 jaws close across this; required, (0, 80]
  float length_mm;               // 28 in-plane long axis; required
  float height_from_belt_mm;     // 32 belt surface to the top; required
};
```

`y_bat_mm` is measured from the camera center at the capture instant; the gantry adds
`cv_cam_y_mm` to get world Y (+Y downstream). `theta_deg` is the cell long axis: 0° is
parallel to +Y and positive rotation about +Z runs from +Y toward −X, in `[-180°, 180°)`.
The gantry folds it to ±90° and grips the width faces. The theta joint matches that long axis: 0° holds the pads parallel to +Y, and open/close is perpendicular to the pads.
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

#### 3. Conveyor Telemetry (`MsgType::CONVEYOR_SPEED = 0x02`, Total Size: 34 Bytes)
```cpp
struct L2ConveyorSpeedPayload {
  float speed_mm_s;        // Real-time linear belt speed
  float displacement_m;    // Cumulative belt travel since startup (meters)
  int32_t raw_encoder_cnt; // 500-PPR hardware pulse counter value
};
```

`displacement_m` is the belt position. `speed_mm_s` only projects that sample until the next report. The two fields and the frame timestamp are one sample. The grasp deadlines are target travels on that anchor.

#### 4. Gantry Status (`MsgType::GANTRY_STATUS = 0x03`, Total Size: 40 Bytes)
```cpp
struct L2GantryStatusPayload {
  uint8_t motion_state;      // 0=IDLE, 1=HOMING, 2=INTERCEPTING, 3=GRIPPING, 4=FAULT
  uint8_t active_slot;       // 0=ota_0, 1=ota_1
  uint16_t fault_flags;      // Bitmask of active drive warnings
  float x_pos_mm;            // Current X position (mm)
  float z_pos_mm;            // Current Z position (mm)
  float theta_deg;           // Current Theta angle (deg)
  float last_cycle_time_ms;  // Duration of last completed pick cycle
};
```

#### 5. Cell Supervisor Command (`MsgType::CELL_COMMAND = 0x04`, Total Size: 30 Bytes)
```cpp
struct L2CellCommandPayload {
  uint8_t command_id; // 1=START, 2=STOP, 3=PAUSE, 4=CLEAR_FAULT, 5=PARK
  uint8_t param_u8;
  uint16_t param_u16;
  float param_float;
};

#pragma pack(pop)
```

---

## 4. Embedded Deployment Guide (ESP32 / FreeRTOS / ESP-IDF)

### Step 1: Component Inclusion
In your component's `CMakeLists.txt` or `idf_component_register`:
```cmake
idf_component_register(
    SRCS "main.cpp"
    REQUIRES CellNetL2 EthernetLink esp_eth
)
```

### Step 2: Driver & Node Initialization
In your firmware's `app_main()` or initialization task:
```cpp
#include "CellNetL2.h"
#include "EspEthL2Transport.h"
#include "EthernetLink.h"
#include "esp_log.h"

static const char* TAG = "ConveyorApp";

// 1. Static driver and node instantiation
static Network::EthernetLink s_eth_link;
static CellNet::EspEthL2Transport s_l2_transport;
static CellNet::CellNetL2Node s_l2_node(s_l2_transport, CellNodeId::CONVEYOR);

void app_main() {
    // 2. Initialize LAN8720 RMII Ethernet
    s_eth_link.start();
    if (s_eth_link.waitForUp(5000)) {
        // 3. Attach Layer-2 filter hook to the active EMAC handle
        if (s_l2_transport.attachEthHandle(s_eth_link.getEthHandle()) == ESP_OK &&
            s_l2_node.begin()) {
            ESP_LOGI(TAG, "CellNet Layer-2 node online (Node 0x02, EtherType 0x88B5)");
        }
    }

    // 4. Register incoming command listener
    s_l2_node.onCellCommand([](const L2CellCommandPayload &cmd, const L2CellHeader &hdr) {
        ESP_LOGI(TAG, "Received Supervisor Command ID: %d", cmd.command_id);
        if (cmd.command_id == 1) {
            // Start belt motor
        } else if (cmd.command_id == 2) {
            // Stop belt motor
        }
    });

    // 5. Start periodic telemetry broadcast task
    xTaskCreatePinnedToCore(telemetryTask, "ConvTx", 4096, nullptr, 5, nullptr, 0);
}
```

### Step 3: Periodic Telemetry Transmission
```cpp
void telemetryTask(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        float speed_mm_s = readHardwareEncoderSpeed();
        float displacement_m = readCumulativeDisplacement();
        int32_t pulses = readRawPcntPulses();

        // Broadcasts 34-byte packet over wire
        s_l2_node.sendConveyorSpeed(speed_mm_s, displacement_m, pulses);

        // Fixed 100.0 Hz deterministic cadence (10.0 ms period)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
```

---

## 5. Unix / Linux Deployment Guide (Ubuntu x86_64 / Raspberry Pi)

On Linux systems (e.g. Ubuntu Vision Classifier on `x86_64` or Raspberry Pi Supervisor on `aarch64`), communication uses standard **Linux POSIX Raw Sockets (`AF_PACKET` / `SOCK_RAW`)**.

### Step 1: Linux Transport Implementation
Create `LinuxRawSocketL2Transport.h`:
```cpp
#pragma once

#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

namespace CellNet {

class LinuxRawSocketL2Transport : public IL2Transport {
 public:
  explicit LinuxRawSocketL2Transport(const std::string& iface_name)
      : iface_name_(iface_name) {}

  ~LinuxRawSocketL2Transport() override {
    running_ = false;
    if (sock_fd_ >= 0) {
      close(sock_fd_);
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

  bool begin() {
    sock_fd_ = socket(AF_PACKET, SOCK_RAW, htons(CELL_NET_L2_ETHERTYPE));
    if (sock_fd_ < 0) {
      std::cerr << "[LinuxL2] socket(AF_PACKET) failed: " << strerror(errno) << std::endl;
      return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[LinuxL2] ioctl(SIOCGIFINDEX) failed: " << strerror(errno) << std::endl;
      return false;
    }
    if_index_ = ifr.ifr_ifindex;

    if (ioctl(sock_fd_, SIOCGIFHWADDR, &ifr) == 0) {
      std::memcpy(mac_, ifr.ifr_hwaddr.sa_data, 6);
    }

    struct sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = if_index_;
    sll.sll_protocol = htons(CELL_NET_L2_ETHERTYPE);
    if (bind(sock_fd_, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
      std::cerr << "[LinuxL2] bind() failed: " << strerror(errno) << std::endl;
      return false;
    }

    running_ = true;
    rx_thread_ = std::thread(&LinuxRawSocketL2Transport::rxWorker, this);
    return true;
  }

  bool sendFrame(const uint8_t* data, size_t length) override {
    if (sock_fd_ < 0 || data == nullptr || length == 0) return false;

    struct sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = if_index_;
    sll.sll_halen = 6;
    std::memcpy(sll.sll_addr, data, 6); // Dest MAC from Ethernet header

    ssize_t sent = sendto(sock_fd_, data, length, 0,
                          (struct sockaddr*)&sll, sizeof(sll));
    return (sent == static_cast<ssize_t>(length));
  }

  bool getMacAddress(uint8_t mac_out[6]) const override {
    if (mac_out == nullptr) return false;
    std::memcpy(mac_out, mac_, 6);
    return true;
  }

  bool isLinkUp() const override { return (sock_fd_ >= 0); }

  void setRxCallback(RxFrameCallback callback) override {
    rx_callback_ = std::move(callback);
  }

 private:
  void rxWorker() {
    uint8_t buf[2048];
    while (running_) {
      ssize_t bytes = recvfrom(sock_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (bytes > 0 && rx_callback_) {
        rx_callback_(buf, static_cast<size_t>(bytes));
      }
    }
  }

  std::string iface_name_;
  int sock_fd_{-1};
  int if_index_{-1};
  uint8_t mac_[6]{};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  RxFrameCallback rx_callback_{nullptr};
};

} // namespace CellNet
```

### Step 2: Linux Application Implementation (Ubuntu Vision Node)
```cpp
#include "CellNetL2.h"
#include "LinuxRawSocketL2Transport.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 1. Bind to dedicated closed cell interface "eth0"
    CellNet::LinuxRawSocketL2Transport transport("eth0");
    if (!transport.begin()) {
        std::cerr << "Failed to open raw Ethernet transport. Check permissions!\n";
        return 1;
    }

    CellNet::CellNetL2Node node(transport, CellNodeId::VISION);
    node.begin();

    std::cout << "Vision Node Active on eth0 (Node ID 0x01)\n";

    // 2. Listen to live belt speed
    node.onConveyorSpeed([](const L2ConveyorSpeedPayload &conv, const L2CellHeader &hdr) {
        // Sync conveyor belt velocity with camera frame-rate
    });

    // 3. Camera capture loop
    uint32_t item_counter = 1000;
    while (true) {
        // Mock camera inference detection:
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        float x_across_mm = 120.5f;
        float y_along_mm = 0.0f;
        float theta_deg = 45.0f;
        uint8_t battery_class = 8; // 21700

        node.sendVisionDetect(item_counter++, x_across_mm, y_along_mm, theta_deg, battery_class);
        std::cout << "Broadcast detection: Item #" << (item_counter - 1) << std::endl;
    }

    return 0;
}
```

### Step 3: Linux CMake Build Configuration (`CMakeLists.txt`)

```cmake
cmake_minimum_required(VERSION 3.16)
project(vision_node CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/CellNetL2/src
)

add_executable(vision_node
    main.cpp
    lib/CellNetL2/src/CellNetL2Framing.cpp
    lib/CellNetL2/src/CellNetL2.cpp
)

target_link_libraries(vision_node PRIVATE pthread)
```

### Step 4: Compiling and Running in Linux Terminal

```bash
# 1. Build binary
mkdir -p build && cd build
cmake ..
make -j4

# 2. Grant raw socket permission (runs without sudo)
sudo setcap cap_net_raw+ep ./vision_node

# 3. Execute on target interface
./vision_node eth0
```

#### One-Liner Compilation (Without CMake via g++)

```bash
g++ -std=c++17 -I./include -I./lib/CellNetL2/src \
    main.cpp \
    lib/CellNetL2/src/CellNetL2Framing.cpp \
    lib/CellNetL2/src/CellNetL2.cpp \
    -lpthread -o vision_node

sudo setcap cap_net_raw+ep ./vision_node
./vision_node eth0
```

---

## 6. Diagnostics, Counters & Link Monitoring

Every `CellNetL2Node` instance maintains atomic telemetry statistics:

```cpp
CellNetStats stats = node.getStats();
printf("TX Frames: %lu\n", stats.tx_frames);
printf("RX Frames: %lu\n", stats.rx_frames);
printf("Corrupted / Invalid Frames: %lu\n", stats.rx_invalid_frames);
printf("Sequence Drops: %lu\n", stats.rx_sequence_drops);
```

### Troubleshooting Checklist

| Symptom | Probable Cause | Corrective Action |
| :--- | :--- | :--- |
| `rx_invalid_frames` increments | Stray non-0x88B5 packets or truncated payloads | Confirm only closed cell devices are plugged into the switch; check cable integrity. |
| `rx_sequence_drops` increments | CPU starvation on receiver or switch buffer overflow | Confirm receiver task priority; verify 100BASE-TX full-duplex auto-negotiation. |
| `sendFrame()` returns false | Physical cable unplugged or invalid EMAC handle | Verify Ethernet link status via `ethernetLink.isUp()` or `s_transport.isLinkUp()`. |
| Linux `socket(AF_PACKET)` error `EPERM` | Missing raw socket permissions | Run `sudo setcap cap_net_raw+ep <binary>` or run as root. |

---

## 7. Host Unit Testing

Native host regression tests run without ESP-IDF or physical hardware:

```powershell
cmake -S test/host -B build/host && cmake --build build/host && ctest --test-dir build/host --output-on-failure
```

The test suite validates:
- Byte-exact encoding for all 5 message types.
- Strict rejection of invalid EtherTypes, version mismatches, and truncated packets.
- Simulated multi-node dispatching and packet drop counter tracking via `FakeL2Transport`.

---

## 8. Programmer's Quick Reference & API Cheat-Sheet

### Summary of Class APIs

```cpp
namespace CellNet {

// ----------------------------------------------------------------------------
// 1. High-Level Communication Node
// ----------------------------------------------------------------------------
class CellNetL2Node {
 public:
  explicit CellNetL2Node(IL2Transport& transport, CellNodeId node_id = CellNodeId::UNKNOWN);

  bool begin();
  bool isReady() const;
  CellNodeId getNodeId() const;
  void setNodeId(CellNodeId node_id);

  // Senders (Returns true on successful MAC/PHY transmission)
  bool sendHeartbeat(uint32_t uptime_ms, uint16_t status_flags = 0, const uint8_t dest_mac[6] = nullptr);
  bool sendVisionDetect(uint32_t item_id, float x_across_mm, float y_bat_mm, float theta_deg, uint8_t battery_class, float capture_displacement_m = 0, bool capture_disp_valid = false, float width_mm = 0, float length_mm = 0, float height_from_belt_mm = 0, const uint8_t dest_mac[6] = nullptr);
  bool sendConveyorSpeed(float speed_mm_s, float displacement_m, int32_t raw_encoder_cnt, const uint8_t dest_mac[6] = nullptr);
  bool sendGantryStatus(uint8_t motion_state, uint8_t active_slot, float x_pos_mm, float z_pos_mm, float theta_deg, float last_cycle_time_ms, uint16_t fault_flags = 0, const uint8_t dest_mac[6] = nullptr);
  bool sendCellCommand(uint8_t command_id, uint8_t param_u8 = 0, uint16_t param_u16 = 0, float param_float = 0.0f, const uint8_t dest_mac[6] = nullptr);

  // Callback Handlers (Synchronously dispatched upon frame arrival)
  void onHeartbeat(std::function<void(const L2HeartbeatPayload&, const L2CellHeader&)> cb);
  void onVisionDetect(std::function<void(const L2VisionDetectPayload&, const L2CellHeader&)> cb);
  void onConveyorSpeed(std::function<void(const L2ConveyorSpeedPayload&, const L2CellHeader&)> cb);
  void onGantryStatus(std::function<void(const L2GantryStatusPayload&, const L2CellHeader&)> cb);
  void onCellCommand(std::function<void(const L2CellCommandPayload&, const L2CellHeader&)> cb);

  // Diagnostics
  CellNetStats getStats() const;
  void resetStats();
};

// ----------------------------------------------------------------------------
// 2. Pure Freestanding Wire Framing & Validation Engine
// ----------------------------------------------------------------------------
class CellNetL2Framing {
 public:
  static size_t expectedFrameLength(CellMsgType type);
  static bool parseFrame(const uint8_t* buffer, size_t length, ParsedFrame& out);
  static std::vector<uint8_t> buildHeartbeatFrame(...);
  static std::vector<uint8_t> buildVisionDetectFrame(...);
  static std::vector<uint8_t> buildConveyorSpeedFrame(...);
  static std::vector<uint8_t> buildGantryStatusFrame(...);
  static std::vector<uint8_t> buildCellCommandFrame(...);
};

// ----------------------------------------------------------------------------
// 3. Hardware Transport Abstraction
// ----------------------------------------------------------------------------
class IL2Transport {
 public:
  virtual ~IL2Transport() = default;
  virtual bool sendFrame(const uint8_t* data, size_t length) = 0;
  virtual bool getMacAddress(uint8_t mac_out[6]) const = 0;
  virtual bool isLinkUp() const = 0;
  virtual void setRxCallback(std::function<void(const uint8_t*, size_t)> callback) = 0;
};

} // namespace CellNet
```

