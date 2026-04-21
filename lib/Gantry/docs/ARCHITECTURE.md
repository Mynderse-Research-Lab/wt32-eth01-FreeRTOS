# Gantry Library Architecture

**Version:** 2.0.0
**Last Updated:** Apr 2026

Complete architecture documentation for the Gantry library.

> **Canonical layered flow.** The control-and-feedback tree, signal routing table, and layered invariants are maintained in [`ARCHITECTURE_FLOW.md`](ARCHITECTURE_FLOW.md). That file is the single source of truth; read it first. The sections below describe the Gantry library's **internal** architecture — the module breakdown, state machine, kinematics, and memory layout — and assume the reader is already familiar with the layered flow.
>
> **Driver refactor (2026-04, still current).** Every axis runs on top of the generic `PulseMotor` library. The pre-2026 `GantryAxisStepper` (step/dir Y) and `GantryRotaryServo` (PWM-hobby-servo Theta) classes were replaced by polymorphic interfaces:
>
> - `GantryLinearAxis` (mm domain) — implemented by `GantryPulseMotorLinearAxis` for ballscrew / belt / rack-pinion drivetrains.
> - `GantryRotaryAxis` (deg domain) — implemented by `GantryPulseMotorRotaryAxis` for rotary-direct drivetrains.
>
> If any section below still mentions `GantryAxisStepper` or `GantryRotaryServo` as concrete types, treat `Gantry.h` and the `GantryLinearAxis.h` / `GantryRotaryAxis.h` interface headers as authoritative.

---

## Table of Contents

- [System Overview](#system-overview)
- [Module Structure](#module-structure)
- [Data Flow](#data-flow)
- [Coordinate Systems](#coordinate-systems)
- [Motion Planning](#motion-planning)
- [State Machine](#state-machine)
- [Safety Systems](#safety-systems)
- [Memory Management](#memory-management)

---

## System Overview

The Gantry library provides a modular, extensible architecture for controlling a 3-axis gantry robot system. It exposes a single public class, `Gantry::Gantry`, which owns three sibling children — the axis wrappers, the end-effector, and the limit switches — and hides every hardware detail (pulse generation, encoder counting, MCP23S17 traffic) behind that one entry point.

### High-Level Architecture

```mermaid
flowchart TD
  App["Application Layer<br/>moveTo(), home(), calibrate(), grip()"]
  Gantry["Gantry::Gantry<br/>sequential motion, state machine,<br/>kinematics, safety"]
  AxisX["GantryPulseMotorLinearAxis (X)"]
  AxisY["GantryPulseMotorLinearAxis (Y)"]
  AxisT["GantryPulseMotorRotaryAxis (Theta)"]
  EE["GantryEndEffector<br/>(digital gripper)"]
  LSX["GantryLimitSwitch × 2<br/>(X min, X max — debounced)"]
  HW["Pulse + direction outputs (LEDC)<br/>Encoder inputs (PCNT)<br/>Gripper / DIR / EN / ALM via MCP23S17"]

  %% ----- downstream (control) -----
  App --> Gantry
  Gantry --> AxisX
  Gantry --> AxisY
  Gantry --> AxisT
  Gantry --> EE
  Gantry --> LSX
  AxisX --> HW
  AxisY --> HW
  AxisT --> HW
  EE --> HW
  LSX --> HW

  %% ----- upstream (feedback) -----
  HW -- "fb: encoder / alarm / limit" --> AxisX
  HW -- "fb" --> AxisY
  HW -- "fb" --> AxisT
  HW -- "fb: limit state" --> LSX
  AxisX -- "fb: mm, state" --> Gantry
  AxisY -- "fb: mm, state" --> Gantry
  AxisT -- "fb: deg, state" --> Gantry
  LSX   -- "fb: limit" --> Gantry
  Gantry -- "fb: status" --> App
```

The `fb:`-prefixed edges are upstream feedback. They flow back through the same ownership tree they came down, giving `Gantry` a consistent read-only view of axis position, limit state, and alarm status without the application layer having to reach past it. `GantryEndEffector` has no feedback edge because the gripper is a digital output with no sensed state in this revision. The full hardware-layer tree (`PulseMotorDriver → LEDC / PCNT / gpio_expander → MCP23S17`) is shown in [`ARCHITECTURE_FLOW.md`](ARCHITECTURE_FLOW.md) §1; this diagram is deliberately abridged to focus on what the Gantry library itself owns.

---

## Module Structure

### Core Modules

#### 1. `Gantry` (Main Class)
**File:** `Gantry.h/cpp`

**Responsibilities:**
- Unified API for all axes
- Sequential motion planning
- State machine execution
- Coordinate transformations
- Error handling

**Key Components:**
- Motion state machine
- Target position storage
- Sequential motion logic

#### 2. `GantryConfig`
**File:** `GantryConfig.h/cpp`

**Responsibilities:**
- Data structure definitions
- Joint space representation
- Workspace representation
- Configuration parameters

**Key Structures:**
- `JointConfig`: Joint space coordinates
- `EndEffectorPose`: Workspace coordinates
- `JointLimits`: Limit validation
- `KinematicParameters`: Mechanical parameters

#### 3. `GantryKinematics`
**File:** `GantryKinematics.h/cpp`

**Responsibilities:**
- Forward kinematics (joint → workspace)
- Inverse kinematics (workspace → joint)
- Joint limit validation

**Key Methods:**
- `forward()`: Joint space → Workspace
- `inverse()`: Workspace → Joint space
- `validate()`: Limit checking

#### 4. `GantryTrajectory`
**File:** `GantryTrajectory.h/cpp`

**Responsibilities:**
- Trapezoidal velocity profiles
- Trajectory planning
- Waypoint management (future)

**Key Components:**
- `TrapezoidalProfile`: Motion profile parameters
- `TrajectoryPlanner`: Profile calculation

#### 5. `GantryAxisStepper`
**File:** `GantryAxisStepper.h/cpp`

**Responsibilities:**
- Y-axis stepper motor control
- Step/dir signal generation
- Trapezoidal motion profiles
- Limit checking

**Key Features:**
- Cooperative update loop
- Acceleration/deceleration control
- Position tracking

#### 6. `GantryRotaryServo`
**File:** `GantryRotaryServo.h/cpp`

**Responsibilities:**
- Theta-axis PWM servo control
- Angle-to-pulse conversion
- Limit enforcement

**Key Features:**
- ESP32 LEDC support
- Standard Servo library fallback
- Configurable pulse ranges

#### 7. `GantryEndEffector`
**File:** `GantryEndEffector.h/cpp`

**Responsibilities:**
- Gripper control
- Digital output management
- Active high/low support

**Key Features:**
- Simple on/off control
- Configurable polarity

#### 8. `GantryUtils`
**File:** `GantryUtils.h`

**Responsibilities:**
- Constants definition
- Helper macros
- Common utilities

**Key Components:**
- `Constants` namespace: Default values
- Validation macros: Code simplification

---

## Data Flow

### Motion Command Flow

```
User Code
  │
  ├─► moveTo(JointConfig)
  │     │
  │     ├─► Validate limits
  │     ├─► Store targets
  │     └─► startSequentialMotion()
  │           │
  │           ├─► Determine motion sequence
  │           ├─► Set gripper target state
  │           └─► Initialize state machine
  │
  └─► update() (called frequently)
        │
        ├─► processSequentialMotion()
        │     │
        │     ├─► Y_DESCENDING: Move Y down
        │     ├─► GRIPPER_ACTUATING: Wait for gripper
        │     ├─► Y_RETRACTING: Move Y up
        │     └─► X_MOVING: Move X to target
        │
        └─► updateAxisPositions()
              │
              ├─► axisY_.update()  (stepper)
              └─► Update current positions
```

### Sequential Motion Sequence

```
┌─────────────────────────────────────────────────────┐
│  Sequential Motion State Machine                   │
└─────────────────────────────────────────────────────┘

START
  │
  ├─► Check current Y position
  │
  ├─► [Current Y < Safe Height]
  │     │
  │     └─► Y_RETRACTING → Move Y to safe height
  │           │
  │           └─► [Complete] → Check if need to descend
  │
  ├─► [Target Y < Current Y]
  │     │
  │     └─► Y_DESCENDING → Move Y to target
  │           │
  │           └─► [Complete] → GRIPPER_ACTUATING
  │                 │
  │                 └─► Close/Open gripper (100ms)
  │                       │
  │                       └─► Y_RETRACTING → Move Y to safe height
  │                             │
  │                             └─► [Complete] → X_MOVING
  │
  └─► [Y at safe height]
        │
        └─► X_MOVING → Move X to target
              │
              └─► [Complete] → IDLE

Theta moves independently throughout sequence
```

---

## Coordinate Systems

### Joint Space

**Definition:** Internal representation using joint positions

- **X**: Horizontal position (mm) - 0 = home, positive = right-to-left
- **Y**: Vertical position (mm) - 0 = fully retracted, positive = down-to-up
- **Theta**: Rotation angle (degrees) - 0 = neutral, ±90° = limits

**Example:**
```cpp
JointConfig joint;
joint.x = 200.0f;    // 200mm from home
joint.y = 50.0f;     // 50mm extended
joint.theta = 45.0f; // 45° rotation
```

### Workspace Space (End-Effector)

**Definition:** Cartesian coordinates of end-effector tip

- **X**: Horizontal position (mm) - includes theta offset
- **Y**: Vertical position (mm) - direct from Y joint
- **Z**: Height position (mm) - constant offset (80mm default)
- **Theta**: Orientation (degrees) - direct from joint

**Transformation:**
```
X_workspace = X_joint + theta_x_offset (-55mm)
Y_workspace = Y_joint
Z_workspace = y_axis_z_offset (80mm)
Theta_workspace = Theta_joint
```

**Example:**
```cpp
EndEffectorPose pose;
pose.x = 145.0f;     // 200 - 55
pose.y = 50.0f;      // Direct
pose.z = 80.0f;      // Constant
pose.theta = 45.0f;  // Direct
```

### Coordinate Transformations

#### Forward Kinematics
```cpp
EndEffectorPose forward(const JointConfig& joints, 
                       const KinematicParameters& params) {
    EndEffectorPose pose;
    pose.x = joints.x + params.theta_x_offset_mm;
    pose.y = joints.y;
    pose.z = params.y_axis_z_offset_mm;
    pose.theta = joints.theta;
    return pose;
}
```

#### Inverse Kinematics
```cpp
JointConfig inverse(const EndEffectorPose& pose,
                    const KinematicParameters& params) {
    JointConfig joints;
    joints.x = pose.x - params.theta_x_offset_mm;
    joints.y = pose.y;
    joints.theta = pose.theta;
    return joints;
}
```

---

## Motion Planning

### Sequential Motion Planning

The library implements a sequential motion planner that ensures safe operation:

1. **Y-axis Descent** (if target Y < current Y)
   - Descends to target Y position
   - Uses configured speed/accel/decel

2. **Gripper Actuation**
   - Automatically determines action:
     - Descending → Close gripper (picking)
     - Ascending → Open gripper (placing)
   - Waits for actuation time (100ms default)

3. **Y-axis Retraction**
   - Retracts to safe height
   - Prevents collision during X movement

4. **X-axis Movement**
   - Moves to target X position
   - Only occurs when Y is at safe height

5. **Theta Movement**
   - Moves independently
   - Can occur anytime during sequence

### Motion Profiles

#### Trapezoidal Profile (Y-axis)

```
Velocity
  ^
  |     ┌─────┐
  |    ╱       ╲
  |   ╱         ╲
  |  ╱           ╲
  | ╱             ╲
  └─────────────────► Time
   Accel  Cruise  Decel
```

**Parameters:**
- Maximum speed (mm/s)
- Acceleration (mm/s²)
- Deceleration (mm/s²)

#### X-axis Motion (SDF08NK8X)

X-axis uses the SDF08NK8X driver's built-in motion profiles:
- Trapezoidal velocity profiles
- Configurable acceleration/deceleration
- Encoder feedback for position control

---

## State Machine

### Motion States

```cpp
enum class MotionState {
    IDLE,              // No motion in progress
    Y_DESCENDING,      // Y-axis moving down to target
    GRIPPER_ACTUATING, // Gripper opening/closing
    Y_RETRACTING,      // Y-axis retracting to safe height
    X_MOVING,          // X-axis moving to target
    THETA_MOVING       // Theta axis moving (independent)
};
```

### State Transitions

```
IDLE
  │
  ├─► [Start motion] → Y_DESCENDING or Y_RETRACTING or X_MOVING
  │
Y_DESCENDING
  │
  └─► [Y reached target] → GRIPPER_ACTUATING
        │
        └─► [Gripper complete] → Y_RETRACTING
              │
              └─► [Y at safe height] → X_MOVING
                    │
                    └─► [X reached target] → IDLE

Y_RETRACTING
  │
  ├─► [Need to descend] → Y_DESCENDING
  └─► [At safe height] → X_MOVING
```

### State Machine Execution

The state machine is executed in `update()`:

```cpp
void Gantry::update() {
    // Check alarms
    if (axisX_.isAlarmActive()) {
        stopAllMotion();
        return;
    }
    
    // Process state machine
    if (motionState_ != MotionState::IDLE) {
        processSequentialMotion();
    }
    
    // Update axis positions
    updateAxisPositions();
}
```

---

## Safety Systems

### Limit Switch Handling

**Architecture:** Limit switches are handled by actuator libraries, not the Gantry class.

- **X-axis**: SDF08NK8X driver handles limit switch debouncing and safety
- **Y-axis**: GantryAxisStepper validates limits before movement
- **Gantry class**: Only calls actuator library methods

**Benefits:**
- Separation of concerns
- Consistent debouncing across axes
- Reduced code duplication

### Alarm Monitoring

```cpp
bool Gantry::isAlarmActive() const {
    return initialized_ && axisX_.isAlarmActive();
}
```

**Alarm Sources:**
- X-axis driver alarms (SDF08NK8X)
- Motion timeouts
- Limit switch violations

**Alarm Response:**
- Stop all motion
- Disable motors
- Reset state machine to IDLE

### Motion Validation

Before starting motion:
1. Check initialization state
2. Check motor enable state
3. Check if already moving
4. Validate joint limits
5. Check alarm status

---

## Memory Management

### Memory Layout

```
┌─────────────────────────────────────┐
│  Gantry Class Instance              │
│  • Axis drivers: ~2-5 KB            │
│  • State machine: ~0.5 KB           │
│  • Configuration: ~0.5 KB           │
│  • Position tracking: ~0.2 KB       │
│  Total: ~3-6 KB                      │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Stack Allocations (temporary)       │
│  • JointConfig: 12 bytes            │
│  • EndEffectorPose: 16 bytes        │
│  • Calculations: <100 bytes         │
└─────────────────────────────────────┘
```

### Memory Optimization

- **No dynamic allocation**: All structures are stack-allocated
- **Template-based queues**: Compile-time size determination
- **Efficient data structures**: Minimal overhead

### RAM Usage Estimate

| Component | RAM Usage |
|-----------|-----------|
| Gantry class | ~3-6 KB |
| Stack (max) | ~200 bytes |
| **Total** | **~5-10 KB** |

Well within ESP32's 320KB RAM limit.

---

## Design Patterns

### 1. Strategy Pattern

Each axis type uses a different driver strategy:
- X-axis: SDF08NK8X driver
- Y-axis: GantryAxisStepper
- Theta: GantryRotaryServo

### 2. State Pattern

Sequential motion uses a state machine pattern:
- States: IDLE, Y_DESCENDING, GRIPPER_ACTUATING, etc.
- Transitions: Based on motion completion

### 3. Template Method Pattern

Common operations follow a template:
- Validate → Configure → Execute → Update

### 4. Facade Pattern

Gantry class provides a unified interface:
- Hides complexity of individual axis drivers
- Provides simple, consistent API

---

## Extension Points

### Adding New Axis Types

1. Create new driver class (e.g., `GantryAxisLinear`)
2. Implement required interface:
   - `begin()`, `enable()`, `disable()`
   - `isConfigured()`, `isBusy()`
   - `moveTo()`, `update()`
3. Add to Gantry class:
   - Member variable
   - Configuration methods
   - Integration in sequential motion

### Adding New Motion Profiles

1. Extend `GantryTrajectory` module
2. Add new profile type (e.g., S-curve)
3. Integrate into motion planning

### Adding New Safety Features

1. Extend alarm monitoring
2. Add new validation checks
3. Integrate into state machine

---

## Performance Considerations

### Update Rate

- **Recommended**: 10-100 Hz
- **Minimum**: 10 Hz (for smooth motion)
- **Maximum**: Limited by CPU and motion calculations

### Timing Constraints

- **Motion planning**: <1ms per update
- **Kinematics**: <100μs per calculation
- **State machine**: <10μs per update

### Optimization Strategies

1. **Cooperative updates**: Non-blocking operations
2. **Efficient calculations**: Minimal floating-point operations
3. **State caching**: Avoid redundant calculations

---

**Last Updated:** Feb 10th 2026  
**Version:** 1.0.0

