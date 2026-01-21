# Gantry System - Test Plan with Wiring

## Date: 2025-01-27 (Updated)
## Hardware: WT32-ETH01 + SDF08NK8X Servo Driver
## Software: Phase 1 Complete (Enhanced Kinematics API) + Phase 2.1 Complete (Waypoint Structures)

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Complete Wiring Diagram](#complete-wiring-diagram)
3. [Pre-Test Setup](#pre-test-setup)
4. [Test Procedures](#test-procedures)
5. [Test Cases](#test-cases)
6. [Expected Results](#expected-results)
7. [Troubleshooting](#troubleshooting)

---

## Hardware Overview

### System Components

1. **WT32-ETH01 Development Board**
   - ESP32-D0WD dual-core processor
   - Built-in Ethernet (LAN8720 PHY)
   - 4MB Flash, 320KB RAM
   - Operating Voltage: 3.3V

2. **SDF08NK8X Servo Driver (Bergerda)**
   - Pulse/Direction control mode
   - Encoder feedback support
   - CN1 connector for I/O

3. **X-Axis Servo Motor**
   - Connected to SDF08NK8X driver
   - Quadrature encoder (typically 2500 PPR)
   - Ball-screw mechanism (40mm pitch)

4. **Limit Switches (2x)**
   - Home limit switch (PIN_LIMIT_MIN)
   - End limit switch (PIN_LIMIT_MAX)
   - NO (Normally Open) type with pullup resistors

5. **Network Connection**
   - Ethernet cable to router/switch
   - Network: 192.168.2.x
   - MQTT broker required
k
---

## Complete Wiring Diagram

### ESP32 WT32-ETH01 Pin Assignments

| ESP32 GPIO | Signal | Direction | Description | Driver Pin |
|------------|--------|-----------|-------------|------------|
| GPIO 2 | PULSE | OUTPUT | Pulse signal to servo driver | CN1 Pin 18 |
| GPIO 4 | DIR | OUTPUT | Direction signal to servo driver | CN1 Pin 19 |
| GPIO 12 | ENABLE | OUTPUT | Servo enable (SON) | CN1 Pin 21 (IN0) |
| GPIO 35 | ENC_A | INPUT | Encoder A+ signal | CN1 Pin 23 |
| GPIO 36 | ENC_B | INPUT | Encoder B+ signal | CN1 Pin 24 |
| GPIO 33 | GRIPPER_OUT | OUTPUT | Gripper output (STUB - no hardware) | Not connected |
| GPIO 25 | GRIPPER_IN | INPUT (PU) | Gripper input/feedback (STUB - no hardware) | Not connected |
| GPIO 14 | LIMIT_MIN | INPUT (PU) | Home limit switch | External switch |
| GPIO 32 | LIMIT_MAX | INPUT (PU) | End limit switch | External switch |
| GPIO 15 | - | - | LED blink task (optional) | - |
| GPIO 16 | ETH_POWER | - | Ethernet PHY power | Internal |
| GPIO 17 | ETH_CLK | - | Ethernet clock output | Internal |
| GPIO 18 | ETH_MDIO | - | Ethernet MDIO | Internal |
| GPIO 23 | ETH_MDC | - | Ethernet MDC | Internal |

**Note**: GPIO 35 and 36 are INPUT ONLY pins (cannot be configured as outputs)

### SDF08NK8X Servo Driver Connections

| Driver CN1 Pin | Signal Name | ESP32 GPIO | Description |
|----------------|-------------|------------|-------------|
| Pin 18 | PULS | GPIO 2 | Pulse input |
| Pin 19 | SIGN | GPIO 4 | Direction input |
| Pin 21 (IN0) | SON | GPIO 12 | Servo enable |
| Pin 23 | A+ | GPIO 35 | Encoder A+ |
| Pin 24 | B+ | GPIO 36 | Encoder B+ |
| Pin 1 (OUT1) | POS | (Optional) | Position reached (not used) |
| Pin 14 (OUT2) | BRK | (Optional) | Brake release (not used) |
| Pin 15 (OUT3) | ALM | (Optional) | Alarm signal (not used) |

### Limit Switch Connections

```
Limit Switch (Home)          ESP32
─────────────────           ──────
   COM ──────────────────── GND
   NO  ──────────────────── GPIO 14 (with internal pullup)
   NC  ──────────────────── (not connected)

Limit Switch (End)
─────────────────
   COM ──────────────────── GND
   NO  ──────────────────── GPIO 32 (with internal pullup)
   NC  ──────────────────── (not connected)
```

**Note**: Both limit switches use ESP32 internal pullup resistors (configured in software)
When switch is open: GPIO reads HIGH (3.3V)
When switch is closed: GPIO reads LOW (0V)

### Gripper Connections (STUB - No Hardware)

**Note**: The gripper is currently stubbed. No actual hardware is connected.
Pins are configured for future use or simulation/testing purposes only.

```
Gripper Output (STUB)        ESP32
─────────────────────       ──────
   (Not connected) ──────── GPIO 33 (OUTPUT)
   
Gripper Input (STUB)         ESP32
─────────────────────       ──────
   (Not connected) ──────── GPIO 25 (INPUT with pullup)
```

**Implementation Notes**:
- GPIO 33 (GRIPPER_OUT): Output pin for gripper control signal
  - HIGH = gripper closed (active)
  - LOW = gripper open (inactive)
  - Pin state is updated by `grip()` method but no hardware is connected
  
- GPIO 25 (GRIPPER_IN): Input pin for gripper feedback/status
  - Configured with internal pullup
  - Currently not read (stub returns last set state)
  - Reserved for future hardware feedback implementation

**Current Behavior**:
- `grip(true)` sets GPIO 33 HIGH and updates internal state
- `grip(false)` sets GPIO 33 LOW and updates internal state
- `getGripperStatus()` returns the last set state (not read from hardware)

### Power Connections

```
Power Supply 1: 5V/12V/24V (for Servo Driver + Motor)
────────────────────────────────────────────────────
  +V ──────────────────────── SDF08NK8X VCC
  GND ─────────────────────── SDF08NK8X GND
  GND ─────────────────────── ESP32 GND (common ground)

Power Supply 2: 3.3V or 5V (for WT32-ETH01)
───────────────────────────────────────────
  +V ──────────────────────── WT32-ETH01 VIN (5V) or 3.3V
  GND ─────────────────────── WT32-ETH01 GND

**IMPORTANT**: Common GND required between all systems
```

### Ethernet Connection

```
Router/Switch              WT32-ETH01
────────────               ──────────
  RJ45 ──────────────────── Ethernet Port
  (CAT5/CAT6 cable)
  
Network Configuration:
  IP: 192.168.2.2
  Gateway: 192.168.2.1
  Subnet: 255.255.255.0
```

---

## Pre-Test Setup

### 1. Hardware Assembly

- [ ] Solder/connect all wires according to wiring diagram
- [ ] Verify all connections are secure
- [ ] Check for short circuits (especially power rails)
- [ ] Verify common ground connection between all systems
- [ ] Ensure power supplies are appropriate voltage/current rating
- [ ] Connect Ethernet cable to router/switch

### 2. Servo Driver Configuration

Configure SDF08NK8X driver parameters (if accessible via driver panel/keypad):

- [ ] PN04 = 0 (Position control mode)
- [ ] PN08 = 0 (Pulse/Direction mode)
- [ ] PN09/PN10: Set electronic gear ratio (if needed)
- [ ] Verify encoder PPR setting matches motor (typically 2500)
- [ ] Verify motor parameters are correctly set

### 3. Software Preparation

- [ ] Upload firmware to WT32-ETH01 via PlatformIO
- [ ] Verify upload successful (check serial output)
- [ ] Confirm no compilation errors
- [ ] Verify firmware version and features

### 4. Network Setup

- [ ] Ensure MQTT broker is running (192.168.2.1:1883)
- [ ] Verify network connectivity (ping test)
- [ ] Configure MQTT client (if needed) to subscribe/publish
- [ ] Test MQTT connection

### 5. Safety Checks

- [ ] Verify all power supplies are OFF
- [ ] Check mechanical clearances (no obstructions)
- [ ] Verify limit switches are properly mounted
- [ ] Ensure emergency stop is accessible
- [ ] Verify motor is properly secured
- [ ] Check that encoder cable is not tangled

---

## Test Procedures

### Procedure 1: Basic Initialization Test

**Objective**: Verify all components initialize correctly

**Steps**:
1. Connect USB cable to WT32-ETH01
2. Open serial monitor (115200 baud)
3. Power on system
4. Observe serial output

**Expected Serial Output**:
```
[    10][Core1][System] Booting...
[  4095][Core1][Gantry] Task Started
ServoDriver: LEDC channel initialized
ServoDriver: Encoder PCNT initialized
ServoDriver: Initialized successfully
[  4106][Core1][Gantry] Initialized
[  4096][Core1][System] Tasks created.
ServoDriver: Enabled
```

**Pass Criteria**: All initialization messages appear, no errors

---

### Procedure 2: Network Connectivity Test

**Objective**: Verify Ethernet and MQTT connectivity

**Steps**:
1. Power on system
2. Wait for Ethernet link (check serial output)
3. Verify IP address assignment
4. Test MQTT connection

**Expected Serial Output**:
```
[  1000][Core1][Ethernet] Link Up
[  2000][Core1][Ethernet] IP: 192.168.2.2
[  2500][Core1][MQTT] Connected. Session: 0
```

**MQTT Test** (using MQTT client):
- Subscribe to: `lab/outbox`
- Publish to: `lab/gantry/cmd`
- Expected: Receive "online" message on `lab/outbox`

**Pass Criteria**: Ethernet connects, IP assigned, MQTT connected

---

### Procedure 3: Encoder Feedback Test

**Objective**: Verify encoder reading is functional

**Steps**:
1. System initialized and enabled
2. Manually rotate motor (if possible) or command small movement
3. Monitor encoder position via serial or MQTT

**MQTT Command** (send to `lab/gantry/cmd`):
```
x:0,y:0,t:0,s:1000
```
This will not move if already at 0,0,0

**MQTT Monitor** (subscribe to `lab/gantry`):
Expected: `X:0 Y:0 Th:0` (position updates every 5 seconds)

**Pass Criteria**: Encoder position is readable and updates

---

### Procedure 4: Limit Switch Test

**Objective**: Verify limit switches are read correctly

**Steps**:
1. System initialized
2. Manually activate home limit switch (GPIO 14 to GND)
3. Manually activate end limit switch (GPIO 32 to GND)
4. Verify readings (via software test or serial output)

**Note**: Current implementation has stub limit switch handlers
For full testing, limit switch state should be monitored

**Pass Criteria**: Limit switches can be read (when implemented)

---

### Procedure 5: Basic Motion Test (Existing API)

**Objective**: Test existing moveTo() functionality

**MQTT Command** (send to `lab/gantry/cmd`):
```
x:1000,y:50,t:0,s:5000
```

**Expected Behavior**:
1. Command received and queued
2. Gantry starts moving
3. Motion completes
4. Gripper sequence executes (stub - pin state updated, no hardware)
5. Position updates via MQTT

**Monitor via Serial**:
```
[  5000][Core1][MQTT] Msg Received: x:1000,y:50,t:0,s:5000
[  5001][Core1][Gantry] Moving: X=1000 Y=50 T=0
[  5010][Core1][Gantry] Gripping...
[  5020][Core1][Gantry] Cmd Done
```

**Pass Criteria**: Command executes, motion occurs (if implemented), no errors

---

### Procedure 6: Phase 1.2 API Test - Kinematics Methods

**Objective**: Test enhanced kinematics API (requires code modification for testing)

**Note**: These methods require explicit API calls in test code.
Current main.cpp does not call these methods directly.

**Test Code Example** (for reference - requires code modification):
```cpp
// After gantry initialization:
Gantry::JointConfig joint;
joint.x = 100.0f;
joint.y = 50.0f;
joint.theta = 45.0f;

// Test forward kinematics
Gantry::EndEffectorPose pose = gantry.forwardKinematics(joint);
// Verify pose values match expected (based on kinematic parameters)

// Test inverse kinematics (round-trip)
Gantry::JointConfig joint2 = gantry.inverseKinematics(pose);
// Verify: joint2.x ≈ joint.x, joint2.y ≈ joint.y, joint2.theta ≈ joint.theta

// Test getCurrentJointConfig
Gantry::JointConfig currentJoint = gantry.getCurrentJointConfig();
// Verify: currentJoint matches current encoder/Y/Theta positions

// Test getTargetJointConfig
Gantry::JointConfig targetJoint = gantry.getTargetJointConfig();
// Verify: targetJoint matches target positions

// Test getCurrentEndEffectorPose
Gantry::EndEffectorPose currentPose = gantry.getCurrentEndEffectorPose();
// Verify: currentPose matches forwardKinematics(currentJoint)

// Test getTargetEndEffectorPose
Gantry::EndEffectorPose targetPose = gantry.getTargetEndEffectorPose();
// Verify: targetPose matches forwardKinematics(targetJoint)
```

**Test Cases**:
1. Forward kinematics with zero joint config → Should return pose at origin
2. Forward kinematics with known values → Verify pose matches expected
3. Inverse kinematics round-trip → Verify forward(inverse(pose)) ≈ pose
4. Get current/target configurations → Verify values match actual state

**Pass Criteria**: 
- Forward/inverse kinematics produce expected results
- Round-trip accuracy within tolerance
- Getter methods return correct values

---

### Procedure 7: Phase 1.3 API Test - Move Methods with New Structures

**Objective**: Test moveTo() overloads with JointConfig and EndEffectorPose

**Note**: These methods require explicit API calls in test code.

**Test Code Snippet** (for reference):
```cpp
// Test JointConfig moveTo
Gantry::JointConfig joint;
joint.x = 100.0f;
joint.y = 50.0f;
joint.theta = 0.0f;
GantryError err = gantry.moveTo(joint, 50, 30);

// Test EndEffectorPose moveTo
Gantry::EndEffectorPose pose;
pose.x = 100.0f;
pose.y = 50.0f;
pose.z = 80.0f;
pose.theta = 0.0f;
GantryError err2 = gantry.moveTo(pose, 50, 30);
```

**Pass Criteria**: 
- Methods compile and execute
- Joint limits are validated
- Error codes returned correctly
- Motion occurs (if motion control implemented)

---

### Procedure 8: Error Handling Test

**Objective**: Test error handling and edge cases

**Test Cases**:
1. Move before initialization → Should return NOT_INITIALIZED
2. Move before enable → Should return MOTOR_NOT_ENABLED
3. Move while already moving → Should return ALREADY_MOVING
4. Move to invalid position (out of limits) → Should return INVALID_POSITION
5. Disconnect encoder → Should handle gracefully
6. Network disconnect → Should handle reconnection

**Pass Criteria**: Appropriate error codes returned, no crashes

---

### Procedure 9: Phase 2.1 API Test - Waypoint and WaypointQueue

**Objective**: Test Waypoint and WaypointQueue structures

**Note**: These structures require explicit API calls in test code.
Current main.cpp does not use these structures directly.

**Test Code Example** (for reference - requires code modification):
```cpp
// Test Waypoint creation
Gantry::EndEffectorPose pose;
pose.x = 100.0f;
pose.y = 50.0f;
pose.z = 80.0f;
pose.theta = 45.0f;

Gantry::Waypoint wp1(pose);
wp1.speed_mm_per_s = 100;
wp1.speed_deg_per_s = 60;
// Verify: wp1.pose == pose, wp1.speed_mm_per_s == 100

Gantry::Waypoint wp2;  // Default constructor
// Verify: wp2.pose is default, wp2.speed_mm_per_s == 50 (default)

// Test WaypointQueue
Gantry::WaypointQueue<8> queue;  // Queue with 8 waypoints

// Test empty queue
// Verify: queue.empty() == true
// Verify: queue.size() == 0
// Verify: queue.full() == false

// Test push operations
bool result1 = queue.push(wp1);
// Verify: result1 == true
// Verify: queue.size() == 1
// Verify: queue.empty() == false

queue.push(wp2);
// Verify: queue.size() == 2

// Test pop operations
Gantry::Waypoint popped;
bool result2 = queue.pop(popped);
// Verify: result2 == true
// Verify: popped.pose == wp1.pose (FIFO order)
// Verify: queue.size() == 1

queue.pop(popped);
// Verify: popped.pose == wp2.pose
// Verify: queue.size() == 0
// Verify: queue.empty() == true

// Test full queue
for (int i = 0; i < 8; i++) {
    Gantry::Waypoint wp;
    queue.push(wp);
}
// Verify: queue.full() == true
// Verify: queue.size() == 8

// Test push to full queue
Gantry::Waypoint extra;
bool result3 = queue.push(extra);
// Verify: result3 == false (queue full)

// Test clear
queue.clear();
// Verify: queue.empty() == true
// Verify: queue.size() == 0
// Verify: queue.full() == false
```

**Test Cases**:
1. Waypoint default constructor → Verify default values
2. Waypoint constructor with pose → Verify pose and default speeds
3. WaypointQueue push/pop → Verify FIFO order
4. WaypointQueue empty/full states → Verify state flags
5. WaypointQueue capacity → Verify queue limits
6. WaypointQueue clear → Verify queue is cleared

**Pass Criteria**: 
- Waypoint structures created correctly
- WaypointQueue operations work correctly (FIFO)
- Queue states (empty/full) are correct
- No memory leaks or crashes

---

## Test Cases

### TC-001: System Boot and Initialization
- **Priority**: CRITICAL
- **Status**: ✅ PASSED (from verification report)
- **Result**: All components initialize correctly

### TC-002: Ethernet Connection
- **Priority**: HIGH
- **Status**: PENDING
- **Requires**: Network infrastructure

### TC-003: MQTT Connectivity
- **Priority**: HIGH
- **Status**: PENDING
- **Requires**: MQTT broker

### TC-004: Encoder Reading
- **Priority**: HIGH
- **Status**: PENDING
- **Requires**: Motor with encoder

### TC-005: Limit Switch Reading
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Limit switches connected

### TC-006: Basic Motion (Existing API)
- **Priority**: HIGH
- **Status**: PENDING
- **Requires**: Complete hardware setup

### TC-007: Phase 1.2 - Forward Kinematics
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification

### TC-008: Phase 1.2 - Inverse Kinematics
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification

### TC-009: Phase 1.3 - MoveTo with JointConfig
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification

### TC-010: Phase 1.3 - MoveTo with EndEffectorPose
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification

### TC-011: Error Handling
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Complete system

### TC-012: Gripper Control (STUB)
- **Priority**: LOW
- **Status**: PENDING (Stub Implementation)
- **Description**: Test gripper control methods (stub - no hardware)
- **Note**: Gripper is stubbed. Output pin (GPIO 33) state is updated but no hardware is connected.
- **Test**: Verify grip() method updates pin state and getGripperStatus() returns correct state

### TC-013: Phase 1.2 - Get Current/Target Joint Config
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test getCurrentJointConfig() and getTargetJointConfig()

### TC-014: Phase 1.2 - Get Current/Target End-Effector Pose
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test getCurrentEndEffectorPose() and getTargetEndEffectorPose()

### TC-015: Phase 1.3 - MoveTo Error Handling
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test error codes (NOT_INITIALIZED, MOTOR_NOT_ENABLED, ALREADY_MOVING, INVALID_POSITION)

### TC-016: Phase 2.1 - Waypoint Structure
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test Waypoint struct creation and initialization

### TC-017: Phase 2.1 - WaypointQueue Operations
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test WaypointQueue push, pop, size, empty, full, clear

### TC-018: Phase 2.1 - WaypointQueue Full/Empty Edge Cases
- **Priority**: MEDIUM
- **Status**: PENDING
- **Requires**: Test code modification
- **Description**: Test queue behavior at capacity and when empty

---

## Expected Results

### Successful Test Indicators

1. **Initialization**: All components initialize without errors
2. **Network**: Ethernet connects, IP assigned, MQTT connected
3. **Encoder**: Position readings are stable and update correctly
4. **Motion**: Commands are accepted and executed (when implemented)
5. **Error Handling**: Appropriate error codes returned for invalid operations
6. **Stability**: System runs continuously without crashes

### Failure Indicators

1. **Initialization Errors**: Missing initialization messages, error messages
2. **Network Errors**: No IP assignment, MQTT connection failures
3. **Encoder Errors**: Zero or erratic position readings
4. **Motion Errors**: Commands ignored, unexpected behavior
5. **Crashes**: System resets, watchdog timeouts
6. **Hardware Errors**: Smoke, overheating, non-functional components

---

## Troubleshooting

### Problem: System does not boot

**Symptoms**: No serial output, device not recognized

**Solutions**:
1. Check USB cable connection
2. Verify power supply is connected and adequate
3. Check for short circuits
4. Try different USB port
5. Verify COM port in platformio.ini

### Problem: ServoDriver initialization fails

**Symptoms**: "ServoDriver: Init Failed!" message

**Solutions**:
1. Verify GPIO pin connections
2. Check that PULSE, DIR, ENABLE pins are correct
3. Verify encoder connections (if enabled)
4. Check servo driver power supply
5. Verify driver is not in alarm state

### Problem: Encoder reading is zero or incorrect

**Symptoms**: Encoder position doesn't change or is always zero

**Solutions**:
1. Verify encoder A+ and B+ connections (GPIO 35, 36)
2. Check encoder power supply
3. Verify encoder PPR setting matches motor
4. Test encoder with oscilloscope (if available)
5. Verify PCNT unit is not conflicting with other peripherals

### Problem: Network does not connect

**Symptoms**: No IP address assigned, Ethernet link down

**Solutions**:
1. Verify Ethernet cable is connected
2. Check router/switch is powered and operational
3. Verify network configuration (IP, gateway, subnet)
4. Check for IP conflicts (multiple devices with same IP)
5. Try different Ethernet cable
6. Verify ETH_PHY_POWER pin (GPIO 16) connection

### Problem: MQTT does not connect

**Symptoms**: MQTT connection timeout, "Disconnected" messages

**Solutions**:
1. Verify MQTT broker is running
2. Check broker IP address (192.168.2.1)
3. Verify broker port (1883)
4. Check firewall settings
5. Verify network connectivity (ping test)
6. Check MQTT broker logs for errors

### Problem: Motion does not occur

**Symptoms**: Commands accepted but no movement

**Solutions**:
1. Verify servo driver is enabled
2. Check motor power supply
3. Verify pulse and direction signals at driver
4. Check for mechanical obstructions
5. Verify limit switches are not triggered
6. Check servo driver alarm status
7. Verify motor is properly connected to driver

### Problem: Limit switches not working

**Symptoms**: Limit switch state not detected

**Solutions**:
1. Verify GPIO connections (GPIO 14, 32)
2. Check limit switch wiring (COM to GND, NO to GPIO)
3. Test limit switch continuity with multimeter
4. Verify pullup resistors are enabled (internal)
5. Check that limit switch handler is implemented (currently stub)

### Problem: System crashes or resets

**Symptoms**: Unexpected resets, watchdog timeouts

**Solutions**:
1. Check stack sizes for FreeRTOS tasks
2. Verify memory usage (should be < 15% RAM)
3. Check for infinite loops or blocking operations
4. Verify all tasks are yielding (vTaskDelay calls)
5. Check for memory leaks
6. Monitor serial output for error messages

---

## Safety Warnings

⚠️ **IMPORTANT SAFETY CONSIDERATIONS**:

1. **Power Supplies**: Ensure all power supplies are properly rated and fused
2. **Grounding**: Always connect common ground between all systems
3. **Mechanical Safety**: Ensure mechanical system is properly secured before testing
4. **Emergency Stop**: Have emergency stop accessible during all tests
5. **High Voltage**: Servo drivers may operate at high voltage - follow manufacturer guidelines
6. **Moving Parts**: Keep clear of moving parts during operation
7. **Hot Surfaces**: Servo drivers and motors may become hot during operation
8. **Software Limits**: Implement software limits in addition to hardware limit switches

---

## Test Environment Checklist

- [ ] WT32-ETH01 development board
- [ ] USB cable for programming/debugging
- [ ] SDF08NK8X servo driver
- [ ] Servo motor with encoder
- [ ] Power supplies (3.3V/5V for ESP32, motor voltage for driver)
- [ ] Limit switches (2x)
- [ ] Gripper pins configured (GPIO 33 output, GPIO 25 input) - STUB, no hardware needed
- [ ] Ethernet cable
- [ ] Router/switch with MQTT broker
- [ ] Multimeter (for troubleshooting)
- [ ] Oscilloscope (optional, for signal verification)
- [ ] Wire and connectors
- [ ] Safety equipment (emergency stop, etc.)

---

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-01-27 | 1.0 | Initial test plan with wiring diagram | Development Team |
| 2025-01-27 | 1.1 | Added test cases for Phase 1.2, 1.3, and 2.1 features | Development Team |

---

## Notes

1. **Phase 1 Features**: Phase 1.2 and 1.3 features are compiled but not directly tested by main.cpp. Explicit test code would be needed to test these features.

2. **Phase 2.1 Features**: Waypoint and WaypointQueue structures are available but not used by main.cpp. These require explicit code modifications to test.

3. **MQTT Interface**: Current MQTT interface (main.cpp) uses the legacy API (moveTo with int32_t parameters). New APIs (JointConfig, EndEffectorPose, Waypoint) are not exposed via MQTT yet.

4. **Testing New APIs**: To test Phase 1.2, 1.3, and 2.1 features, either:
   - Modify main.cpp to include test code
   - Create a separate test sketch
   - Add MQTT commands to expose new APIs (future enhancement)

5. **Backward Compatibility**: All new features maintain backward compatibility. Existing code continues to work unchanged.

2. **Stub Implementations**: Some features (homing, calibration, motion control) are currently stub implementations. These should be tested when fully implemented.

3. **Network Dependency**: Several tests require network infrastructure. These can be skipped if network is not available.

4. **Hardware Dependency**: Full testing requires complete hardware setup. Individual components can be tested separately.

5. **Safety First**: Always follow safety guidelines and manufacturer recommendations when working with servo systems.
