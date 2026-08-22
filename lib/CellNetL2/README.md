# CellNetL2 — Reusable OSI Layer-2 Real-Time Communication Library

`CellNetL2` is a high-speed, deterministic, and hardware-independent **OSI Layer-2 communication library** operating directly over raw Ethernet with **EtherType `0x88B5`** (IEEE 802.3 Local / Real-Time).

This library provides the unified, low-latency communication backbone across all battery sorting automation subsystems:
- **Vision IPC (Ubuntu x86_64 / Linux)**: Node ID `0x01` (`VISION`)
- **Conveyor Belt Tracker (WT32-ETH01 / ESP32)**: Node ID `0x02` (`CONVEYOR`)
- **Gantry Motion Controller (WT32-ETH01 / ESP32)**: Node ID `0x03` (`GANTRY`)
- **Supervisor & HMI (Raspberry Pi ARM64 / Linux)**: Node ID `0x04` (`SUPERVISOR`)

---

## 1. Architectural Scope & Principles

### Strict Separation of Concerns: Pure Data Transport
The sole responsibility of `lib/CellNetL2` is to **reliably package, transmit, validate, and dispatch raw messages between nodes**.
- **No Domain Math**: The library contains **zero domain calculations, kinematics, or trajectory algorithms**.
- **Application Responsibility**: Coordinate transformations, spatial dead-reckoning, and motion trajectories remain strictly inside application layers (e.g. `pick_scheduler.cpp` on Gantry, belt tracker on Conveyor, classifier on Vision).

### Data Integrity & Safety Model
1. **Hardware CRC-32 (FCS)**: Handled in silicon by the Ethernet MAC; bit-flipped or electrically corrupted packets are dropped before reaching RAM.
2. **DMA Ring Buffer Queues**: Multi-node simultaneous transmissions land in independent RAM descriptors without overwriting or race conditions.
3. **Exact Length Validation**: The library strictly verifies that `length == sizeof(ExpectedFrame)` before reading any payload bytes.
4. **1-Byte Struct Packing (`#pragma pack(push, 1)`)**: Eliminates compiler padding differences between 32-bit Xtensa (ESP32), 64-bit ARM (Raspberry Pi), and 64-bit x86 (Ubuntu).
5. **Independent Sequence Counters (`0..255`)**: Tracks packet delivery per sender ID to immediately detect dropped or out-of-order frames.
6. **Zero-LwIP Overhead**: Raw EtherType `0x88B5` frames bypass the LwIP TCP/IP stack for $< 50\text{ }\mu\text{s}$ transit and dispatch latency.

---

## 2. Directory Structure

```
lib/CellNetL2/
├── CMakeLists.txt              # ESP-IDF component registration
├── README.md                   # This documentation manual
└── src/
    ├── IL2Transport.h          # Abstract C++17 transport interface
    ├── CellNetL2Framing.h      # Freestanding wire encoders, decoders, and validation
    ├── CellNetL2Framing.cpp    # Framing implementation
    ├── CellNetL2.h             # Subsystem-agnostic Node API & Callback Dispatcher
    ├── CellNetL2.cpp           # Node implementation & sequence drop tracker
    ├── EspEthL2Transport.h     # ESP-IDF LAN8720 EMAC DMA transport driver
    └── EspEthL2Transport.cpp   # EMAC filter hook implementation
```

---

## 3. Wire Protocol & Frame Layout

All frames use the standard 14-byte IEEE 802.3 header, followed by the 8-byte Cell Protocol header, followed by a typed payload:

```
┌───────────────────────────┬──────────────────────────────────────────┬────────────────────────┐
│  Ethernet Header (14 B)   │           Cell Header (8 B)              │   Payload (Variable)   │
├───────────┬───────────────┼──────────┬──────────┬──────────┬─────────┼────────────────────────┤
│ Dest MAC  │   Source MAC  │ Version  │ MsgType  │ SenderID │ Seq No. │  Typed Payload Data    │
│ (6 Bytes) │   (6 Bytes)   │ (0x01)   │ (0x00..4)│ (0x01..4)│ (0..255)│  (Speed / Detect / etc)│
└───────────┴───────────────┴──────────┴──────────┴──────────┴─────────┴────────────────────────┘
```

### Supported Message Types

| Msg Type | ID | Direction | Frame Size | Primary Payload Fields |
| :--- | :--- | :--- | :--- | :--- |
| **`HEARTBEAT`** | `0x00` | Any $\rightarrow$ Broadcast | 30 Bytes | `uptime_ms`, `status_flags` |
| **`VISION_DETECT`** | `0x01` | Vision $\rightarrow$ Gantry / Supervisor | 38 Bytes | `item_id`, `x_across_mm`, `y_bat_mm`, `theta_deg`, `battery_class` |
| **`CONVEYOR_SPEED`** | `0x02` | Conveyor $\rightarrow$ Gantry / Supervisor | 34 Bytes | `speed_mm_s`, `displacement_m`, `raw_encoder_cnt` |
| **`GANTRY_STATUS`** | `0x03` | Gantry $\rightarrow$ Supervisor | 40 Bytes | `motion_state`, `active_slot`, `x_pos_mm`, `z_pos_mm`, `theta_deg`, `cycle_ms`, `fault_flags` |
| **`CELL_COMMAND`** | `0x04` | Supervisor $\rightarrow$ Gantry / Conveyor | 30 Bytes | `command_id` (Start/Stop/Park), `param_u8`, `param_u16`, `param_float` |

---

## 4. Deployment Guides

### A. Embedded Deployment (ESP32 / WT32-ETH01 / FreeRTOS)

```cpp
#include "CellNetL2.h"
#include "EspEthL2Transport.h"
#include "EthernetLink.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 1. Static driver and node instantiation
static Network::EthernetLink s_eth_link;
static CellNet::EspEthL2Transport s_l2_transport;
static CellNet::CellNetL2Node s_l2_node(s_l2_transport, CellNodeId::CONVEYOR);

void telemetryTask(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        float speed = readBeltSpeed();
        float displacement = readDisplacement();
        int32_t pulses = readPcntHardwarePulses();

        // Broadcast 34-byte packet over wire
        s_l2_node.sendConveyorSpeed(speed, displacement, pulses);

        // Fixed 100.0 Hz deterministic cadence (10.0 ms period)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

void app_main() {
    s_eth_link.start();
    if (s_eth_link.waitForUp(5000)) {
        // Attach Layer-2 filter hook to the active EMAC handle
        s_l2_transport.attachEthHandle(s_eth_link.getEthHandle());
        s_l2_node.begin();
    }

    // Register incoming command listener
    s_l2_node.onCellCommand([](const L2CellCommandPayload &cmd, const L2CellHeader &hdr) {
        if (cmd.command_id == 1) startBelt();
        else if (cmd.command_id == 2) stopBelt();
    });

    xTaskCreatePinnedToCore(telemetryTask, "ConvTx", 4096, nullptr, 5, nullptr, 0);
}
```

---

### B. Unix / Linux Deployment (Ubuntu x86_64 / Raspberry Pi ARM64)

On Linux systems, use POSIX raw packet sockets (`AF_PACKET` / `SOCK_RAW`) bound to the dedicated closed-cell NIC (e.g. `eth0`):

```cpp
#include "CellNetL2.h"
#include "LinuxRawSocketL2Transport.h" // Standard Linux AF_PACKET transport
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

    // 2. Listen to live belt speed
    node.onConveyorSpeed([](const L2ConveyorSpeedPayload &conv, const L2CellHeader &hdr) {
        // Sync conveyor belt velocity with camera frame-rate
    });

    // 3. Camera capture loop
    uint32_t item_id = 1000;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        float x_across_mm = 120.5f;
        float y_along_mm = 0.0f;
        float theta_deg = 45.0f;
        uint8_t battery_class = 2; // 21700

        node.sendVisionDetect(item_id++, x_across_mm, y_along_mm, theta_deg, battery_class);
    }

    return 0;
}
```

#### Building on Linux with CMake (`CMakeLists.txt`)

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

#### Compiling and Running in Linux Terminal

```bash
# 1. Build executable
mkdir -p build && cd build
cmake ..
make -j4

# 2. Grant raw socket permission (no sudo required to execute)
sudo setcap cap_net_raw+ep ./vision_node

# 3. Run on dedicated network interface (e.g. eth0)
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

## 5. Programmer's Reference

### 5.1 Namespaces & Headers
- **Root Namespace:** `CellNet`
- **Primary Header:** `#include "CellNetL2.h"`
- **Framing & Low-Level Header:** `#include "CellNetL2Framing.h"`
- **Protocol Enums & Structs:** `#include "cell_net_l2_protocol.h"`
- **ESP-IDF Driver:** `#include "EspEthL2Transport.h"` (ESP32 only)
- **Linux Driver:** `#include "LinuxRawSocketL2Transport.h"` (Linux / POSIX only)

---

### 5.2 Protocol Constants & Enumerations

```cpp
constexpr uint16_t CELL_NET_L2_ETHERTYPE = 0x88B5; // IEEE 802.3 Local / Real-Time
constexpr uint8_t  CELL_NET_L2_VERSION   = 0x01;   // Current Protocol Version

enum class CellNodeId : uint8_t {
  UNKNOWN    = 0x00,
  VISION     = 0x01, // Ubuntu IPC Camera Classifier
  CONVEYOR   = 0x02, // WT32 Belt Tracker (500 PPR encoder)
  GANTRY     = 0x03, // WT32 Multi-Axis Motion Controller
  SUPERVISOR = 0x04, // Raspberry Pi Cell Coordinator & Web HMI
  BROADCAST  = 0xFF,
};

enum class CellMsgType : uint8_t {
  HEARTBEAT      = 0x00, // Periodic health and uptime beacon
  VISION_DETECT  = 0x01, // Vision -> Gantry / Supervisor object coordinates
  CONVEYOR_SPEED = 0x02, // Conveyor -> Gantry / Supervisor belt velocity
  GANTRY_STATUS  = 0x03, // Gantry -> Supervisor motion telemetry
  CELL_COMMAND   = 0x04, // Supervisor -> Gantry / Conveyor cell execution commands
};
```

---

### 5.3 Core Class: `CellNet::CellNetL2Node`

The main orchestrator class that manages frame transmission, callback dispatching, and link statistics over any `IL2Transport` implementation.

#### Constructor & Lifecycle
```cpp
explicit CellNetL2Node(IL2Transport& transport, CellNodeId node_id = CellNodeId::UNKNOWN);
```
- **`transport`**: Reference to the underlying transport driver instance (`EspEthL2Transport`, `LinuxRawSocketL2Transport`, or `FakeL2Transport`).
- **`node_id`**: Subsystem node ID (e.g. `CellNodeId::GANTRY`).

```cpp
bool begin();
```
- Binds the receive callback on the transport and retrieves the local NIC MAC address.
- **Returns**: `true` on success, `false` on failure.

```cpp
bool isReady() const;
```
- **Returns**: `true` if the transport is initialized and the physical link is up.

```cpp
CellNodeId getNodeId() const;
void setNodeId(CellNodeId node_id);
```
- Getter and setter for the local node ID.

---

#### Message Transmission Methods
All send methods return `bool` (`true` if successfully pushed to the hardware transport, `false` if link is down or buffer is invalid).

```cpp
bool sendHeartbeat(uint32_t uptime_ms, uint16_t status_flags = 0, const uint8_t dest_mac[6] = nullptr);
```
- Broadcasts a 30-byte `HEARTBEAT` frame with node uptime and status flags.

```cpp
bool sendVisionDetect(uint32_t item_id, float x_across_mm, float y_bat_mm, float theta_deg, uint8_t battery_class, const uint8_t dest_mac[6] = nullptr);
```
- Broadcasts a 38-byte `VISION_DETECT` frame containing detected battery coordinates and form-factor classification.

```cpp
bool sendConveyorSpeed(float speed_mm_s, float displacement_m, int32_t raw_encoder_cnt, const uint8_t dest_mac[6] = nullptr);
```
- Broadcasts a 34-byte `CONVEYOR_SPEED` frame containing instantaneous velocity, cumulative belt travel in meters, and raw 500-PPR hardware pulse counts.

```cpp
bool sendGantryStatus(uint8_t motion_state, uint8_t active_slot, float x_pos_mm, float z_pos_mm, float theta_deg, float last_cycle_time_ms, uint16_t fault_flags = 0, const uint8_t dest_mac[6] = nullptr);
```
- Broadcasts a 40-byte `GANTRY_STATUS` frame containing the active Cartesian/rotary pose, cycle time, and drive warning bitmasks.

```cpp
bool sendCellCommand(uint8_t command_id, uint8_t param_u8 = 0, uint16_t param_u16 = 0, float param_float = 0.0f, const uint8_t dest_mac[6] = nullptr);
```
- Broadcasts a 30-byte `CELL_COMMAND` frame from Supervisor to coordinate cell start/stop/pause states.

> **Note on `dest_mac`**: Passing `nullptr` (default) transmits to the global Broadcast address `FF:FF:FF:FF:FF:FF`. Passing a 6-byte array sends a targeted Unicast frame.

---

#### Callback Registration Methods
Register lambda or function pointer handlers for incoming peer messages. Callbacks are dispatched synchronously from the transport RX thread/task.

```cpp
void onHeartbeat(std::function<void(const L2HeartbeatPayload& payload, const L2CellHeader& header)> cb);
void onVisionDetect(std::function<void(const L2VisionDetectPayload& payload, const L2CellHeader& header)> cb);
void onConveyorSpeed(std::function<void(const L2ConveyorSpeedPayload& payload, const L2CellHeader& header)> cb);
void onGantryStatus(std::function<void(const L2GantryStatusPayload& payload, const L2CellHeader& header)> cb);
void onCellCommand(std::function<void(const L2CellCommandPayload& payload, const L2CellHeader& header)> cb);
```

---

#### Diagnostics & Link Health
```cpp
struct CellNetStats {
  uint32_t tx_frames;          // Total successfully transmitted frames
  uint32_t rx_frames;          // Total successfully parsed & dispatched frames
  uint32_t rx_invalid_frames;  // Frames rejected due to corrupt EtherType/Version/Length
  uint32_t rx_sequence_drops;  // Total missing sequence frames detected across senders
};

CellNetStats getStats() const;
void resetStats();
```

---

### 5.4 Low-Level Framing Class: `CellNet::CellNetL2Framing`

Provides static, freestanding C++17 serialization and parsing functions:

```cpp
struct ParsedFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  union {
    L2HeartbeatPayload heartbeat;
    L2VisionDetectPayload vision_detect;
    L2ConveyorSpeedPayload conveyor_speed;
    L2GantryStatusPayload gantry_status;
    L2CellCommandPayload cell_command;
  } payload;
  bool valid;
};

static size_t expectedFrameLength(CellMsgType type);
static bool parseFrame(const uint8_t* buffer, size_t length, ParsedFrame& out);
static std::vector<uint8_t> buildHeartbeatFrame(...);
static std::vector<uint8_t> buildVisionDetectFrame(...);
static std::vector<uint8_t> buildConveyorSpeedFrame(...);
static std::vector<uint8_t> buildGantryStatusFrame(...);
static std::vector<uint8_t> buildCellCommandFrame(...);
```

---

### 5.5 Abstract Transport Interface: `CellNet::IL2Transport`

```cpp
class IL2Transport {
 public:
  virtual ~IL2Transport() = default;
  virtual bool sendFrame(const uint8_t* data, size_t length) = 0;
  virtual bool getMacAddress(uint8_t mac_out[6]) const = 0;
  virtual bool isLinkUp() const = 0;
  virtual void setRxCallback(std::function<void(const uint8_t* buffer, size_t length)> callback) = 0;
};
```

---

## 6. Host Unit Testing

Run the native Unity regression test suite on your development workstation:

```powershell
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host -R test_cell_net_l2 --output-on-failure
```

