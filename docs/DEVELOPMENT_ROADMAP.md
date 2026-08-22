# Development Roadmap: Phase 5 & Phase 6

This document outlines the systematic plan to move development forward, addressing structural technical debt before executing the migration to a Zenoh-DDS networking stack. 

*Note: The physical pneumatic end-effector (gripper) is not yet assembled onto the gantry. The hardware currently only supports X, Z, and Theta axis interpolation. The scheduler logic will reflect this hardware constraint.*

---

## Phase 5: Architecture Decoupling & Motion Wiring (Pre-Zenoh)

Before replacing the communications layer, the following underlying structural issues are being resolved to ensure the new network stack has a stable foundation.

### 5.1 Extract the Network Layer (**Done ✅**)
- **Goal**: Decouple the LAN8720 Ethernet and LwIP IP-acquisition logic from the MQTT client.
- **Status**: **Completed**. `EthernetLink` was extracted into its own standalone system component under `lib/EthernetLink` with no MQTT dependencies.

### 5.2 Wire the Pick Scheduler to Physical Motion (**Active Target 🎯**)
- **Goal**: Connect incoming `CellNetL2` / telemetry targets in `pick_scheduler.cpp` to live `Gantry::moveTo()` commands.
- **Action**:
  1. Map the incoming pick target payload coordinates to the gantry trajectory interface.
  2. Because the **pneumatic end-effector is missing**, the grip() action will be strictly mocked (e.g., a software delay or bypassed entirely) while the X, Z, and Theta trajectory sequences run live.
  3. Validate that continuous physical motion does not cause EtherNet/IP pacing overruns during network telemetry bursts.

### 5.3 Static Memory Allocation for Telemetry (**Done ✅**)
- **Goal**: Prevent heap fragmentation and dynamic string serialization overhead at high frequency.
- **Status**: **Completed**. Implemented `lib/CellNetL2` using `#pragma pack(1)` binary structs over raw Ethernet (`0x88B5`), completely bypassing JSON heap allocations.

### 5.4 Fault State Propagation
- **Goal**: Prevent the controller from blindingly accepting targets if a drive faults.
- **Action**: Implement a background query for `gantry->getFaults()`. If a hardware alarm (e.g., HCS01 / Kinetix following error) is detected, transition the `pick_scheduler` into a `FAULT` state, lock out further motion commands, and publish an abort status to the network.

---

## Phase 6: Zenoh-DDS Integration

Once the scheduler is safely moving the machine and decoupled from MQTT, the network transport migration will begin.

### 6.1 Component Setup
- Pull the zenoh-pico C99 library into the ESP-IDF build as a standard component under lib/ZenohBridge.
- Implement the ZenohBridge C++ wrapper to handle the UDP/TCP peer connection lifecycle, managing the z_check() and z_lease() loops in a dedicated low-priority FreeRTOS task to prevent interfering with the Class 1 scanner.

### 6.2 Key Expression Mapping
Translate the current MQTT topics to Zenoh key expressions:
- **Subscribe**: gantry/pick/cmd, gantry/control/cmd
- **Publish**: gantry/status, gantry/telemetry
- Switch the payload format to Compact CDR (binary) for maximum throughput.

### 6.3 Deprecate MQTT
- Re-route the PickScheduler to pull frames directly from the ZenohBridge callback queue.
- Remove lib/MqttBridge, mosquitto broker dependencies, and associated Kconfig variables.

### 6.4 Supervisory Validation
- Provide host-side tools (zenoh_test_publisher.py) to stream targets.
- Validate round-trip commanding using zenoh-bridge-dds to prove ROS 2 compatibility from the vision system to the embedded controller.
