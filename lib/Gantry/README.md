# Gantry Library

**Version:** 1.0.0 (In Development)  
**Platform:** ESP32 (WT32-ETH01)  
**Framework:** ESP-IDF (FreeRTOS)  

A comprehensive multi-axis gantry control library for ESP32 with 3-DoF support (X, Y, Theta) designed for pick-and-place applications with conveyor synchronization.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
- [Documentation](#documentation)
- [Development Status](#development-status)
- [Installation](#installation)
- [Examples](#examples)
- [API Reference](#api-reference)
- [Configuration](#configuration)
- [Thread Safety](#thread-safety)
- [License](#license)

---

## Overview

The Gantry library provides a complete motion control system for a 3-axis gantry robot. All three motion axes are driven by the generic `PulseMotor` library (see `lib/PulseMotor/`), so any mix of pulse-train driver hardware can be used. Per-axis drivetrain topology is configured independently via a `DrivetrainType` enum:

- **X-axis**: Horizontal linear joint. Default deployment: Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt actuator (200 mm/rev).
- **Y-axis**: Vertical linear joint. Default deployment: Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew actuator (20 mm pitch).
- **Theta-axis**: Rotational joint. Default deployment: custom pulse-train driver + SCHUNK ERD 04-40-D-H-N miniature rotary module.
- **End-effector**: Digital output; default deployment is a SCHUNK KGG 100-80 pneumatic gripper.

The library implements sequential motion planning, forward/inverse kinematics, trajectory planning, and safety features suitable for industrial automation applications.

---

## Features

### ✅ Implemented Features

- **Multi-axis Control**
  - X / Y / Theta: PulseMotor driver (pulse+direction), each with independent `DrivetrainConfig` (ballscrew, belt, rack-pinion, or rotary-direct).
  - Encoder feedback per axis via PCNT (when wired).
  - Trapezoidal velocity profiles in the driver, 5 ms ramp callback.
  - End-effector: digital output gripper control (e.g. SCHUNK KGG 100-80).

- **Sequential Motion Planning**
  - Automatic Y-axis descent → gripper actuation → Y-axis retraction → X-axis movement
  - Safe height management for collision avoidance
  - Non-blocking state machine execution

- **Kinematics**
  - Forward kinematics (joint space → workspace)
  - Inverse kinematics (workspace → joint space)
  - Joint limit validation
  - Workspace coordinate system

- **Safety Features**
  - Limit switch handling (delegated to actuator libraries)
  - Alarm monitoring
  - Motion state validation
  - Emergency stop support

- **Homing & Calibration**
  - X-axis homing sequence
  - Automatic axis length calibration
  - Position reference management

- **FreeRTOS Compatible**
  - Non-blocking operations
  - Cooperative update loop
  - Task-safe design patterns

### 🚧 In Progress

- Trajectory waypoint queue execution
- Enhanced error recovery
- Performance optimization
- Extended documentation

### 📋 Planned Features

- S-curve motion profiles
- Collision detection
- Advanced trajectory planning
- Configuration persistence (EEPROM)
- Network status reporting (MQTT)

---

## Quick Start

### Basic Setup

```cpp
#include "Gantry.h"
#include "PulseMotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

// Configure each axis's driver + drivetrain independently.
// (Typically these come from include/axis_pulse_motor_params.h and
//  include/axis_drivetrain_params.h in the full application; values shown
//  inline here for clarity.)
PulseMotor::DriverConfig xDrv;
xDrv.pulse_pin      = X_PULSE_GPIO;
xDrv.dir_pin        = X_DIR_PIN;
xDrv.enable_pin     = X_ENABLE_PIN;
xDrv.encoder_a_pin  = X_ENC_A_GPIO;
xDrv.encoder_b_pin  = X_ENC_B_GPIO;
xDrv.encoder_ppr    = 10000;
xDrv.enable_encoder_feedback = true;
xDrv.pulse_mode     = PulseMotor::PulseMode::PULSE_DIRECTION;

PulseMotor::DrivetrainConfig xDt;
xDt.type                 = PulseMotor::DRIVETRAIN_BELT;   // Beta 100-ZRS
xDt.belt_lead_mm_per_rev = 200.0f;
xDt.encoder_ppr          = xDrv.encoder_ppr;
xDt.motor_reducer_ratio  = 1.0f;

// Repeat for yDrv/yDt (ballscrew) and tDrv/tDt (rotary-direct).

Gantry::Gantry gantry(xDrv, xDt, yDrv, yDt, tDrv, tDt, GRIPPER_PIN);
gantry.setLimitPins(MIN_LIMIT_PIN, MAX_LIMIT_PIN);
gantry.setYAxisLimits(0.0f, 200.0f);  // 0-200mm travel
gantry.setThetaLimits(-180.0f, 180.0f);  // soft angular limits

// Initialize
gantry.begin();
gantry.enable();

// Home X-axis
gantry.home();

// Move to position (joint space)
Gantry::JointConfig target;
target.x = 100.0f;    // 100mm
target.y = 50.0f;     // 50mm
target.theta = 45.0f; // 45 degrees

gantry.moveTo(target, 50, 30);  // 50 mm/s, 30 deg/s

// Update task (call frequently)
void gantryUpdateTask(void *pvParameters) {
    while (1) {
        gantry.update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz update rate
    }
}

void app_main(void) {
    // Create update task
    xTaskCreate(gantryUpdateTask, "GantryUpdate", 4096, NULL, 5, NULL);
    
    // Other initialization...
}
```

### Sequential Motion Example

```cpp
// The library automatically sequences:
// 1. Y-axis descends to target Y
// 2. Gripper actuates (close for picking, open for placing)
// 3. Y-axis retracts to safe height
// 4. X-axis moves to target X
// 5. Theta moves independently

Gantry::JointConfig pickPos;
pickPos.x = 200.0f;
pickPos.y = 30.0f;   // Low position for picking
pickPos.theta = 0.0f;

gantry.moveTo(pickPos, 50, 30);
while (gantry.isBusy()) {
    gantry.update();
    delay(10);
}

// Now move to place position
Gantry::JointConfig placePos;
placePos.x = 400.0f;
placePos.y = 30.0f;
placePos.theta = 90.0f;

gantry.moveTo(placePos, 50, 30);
```

---

## Architecture

The library is organized into modular components:

```
Gantry Library
├── Gantry.h/cpp                         - Main gantry control class
├── GantryConfig.h/cpp                   - Configuration structures (JointConfig, EndEffectorPose, KinematicParameters)
├── GantryKinematics.h/cpp               - Forward/inverse kinematics
├── GantryTrajectory.h/cpp               - Trajectory planning
├── GantryLinearAxis.h                   - Abstract linear-axis interface (mm domain)
├── GantryRotaryAxis.h                   - Abstract rotary-axis interface (deg domain)
├── GantryPulseMotorLinearAxis.h/cpp     - Linear-axis implementation on PulseMotor
├── GantryPulseMotorRotaryAxis.h/cpp     - Rotary-axis implementation on PulseMotor
├── GantryEndEffector.h/cpp              - Gripper control
├── GantryLimitSwitch.h/cpp              - Debounced limit switch input
└── GantryUtils.h                        - Constants and macros
```

### Key Design Principles

1. **Modularity**: Each axis type has its own driver class
2. **Separation of Concerns**: Kinematics, trajectory, and control are separate
3. **Reusability**: Helper functions eliminate code duplication
4. **Safety First**: Limit switches and alarms handled by actuator libraries
5. **FreeRTOS Friendly**: Non-blocking, cooperative design

---

## Documentation

- **[API Reference](docs/API_REFERENCE.md)** - Complete API documentation
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System architecture and design
- **[Configuration Guide](docs/CONFIGURATION_GUIDE.md)** - Setup and tuning
- **[Examples](docs/EXAMPLES.md)** - Code examples and tutorials
- **[Development Journal](docs/DEVELOPMENT_JOURNAL.md)** - Chronological engineering journal
- **[Changelog](CHANGELOG.md)** - Version history

---

## Development Status

**Current Version:** 1.0.0 (In Development)

### Implementation Status

| Component | Status | Notes |
|-----------|--------|-------|
| X-axis (PulseMotor + belt) | ✅ Complete | Kinetix 5100 + SCHUNK Beta 100-ZRS |
| Y-axis (PulseMotor + ballscrew) | ✅ Complete | Kinetix 5100 + SCHUNK Beta 80-SRS |
| Theta-axis (PulseMotor + rotary-direct) | ✅ Complete | Custom driver + SCHUNK ERD 04-40-D-H-N |
| End-effector | ✅ Complete | Digital output control |
| Sequential Motion | ✅ Complete | Y→Gripper→Y→X sequence |
| Kinematics | ✅ Complete | Forward/inverse kinematics |
| Homing | ✅ Complete | X-axis homing sequence |
| Calibration | ✅ Complete | Axis length measurement |
| Trajectory Planning | 🟡 Partial | Basic waypoint support |
| Error Recovery | 🟡 Partial | Basic alarm handling |
| Documentation | 🟡 In Progress | Comprehensive docs being written |

**Legend:**
- ✅ Complete and tested
- 🟡 Partial implementation
- 🔴 Not started

See [CHANGELOG.md](CHANGELOG.md) and [DEVELOPMENT_JOURNAL.md](docs/DEVELOPMENT_JOURNAL.md) for the history behind these statuses.

---

## Installation

### ESP-IDF Component

The Gantry library is designed as an ESP-IDF component. Add it to your project:

1. **Add as Component**: Copy `lib/Gantry` to your ESP-IDF project's `components/` directory, or add it as a submodule.

2. **Update CMakeLists.txt**: Add the component to your main `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        gantry
        pulsemotor
        freertos
        driver
        esp_timer
)
```

3. **Component CMakeLists.txt**: Ensure `lib/Gantry/CMakeLists.txt` registers the component properly:

```cmake
idf_component_register(
    SRCS 
        "src/Gantry.cpp"
        "src/GantryConfig.cpp"
        "src/GantryKinematics.cpp"
        "src/GantryTrajectory.cpp"
        "src/GantryPulseMotorLinearAxis.cpp"
        "src/GantryPulseMotorRotaryAxis.cpp"
        "src/GantryEndEffector.cpp"
        "src/GantryLimitSwitch.cpp"
    INCLUDE_DIRS 
        "src"
    REQUIRES
        pulsemotor
        freertos
        driver
        esp_timer
)
```

### Dependencies

- **PulseMotor**: Generic pulse+direction motor driver for all three axes (`lib/PulseMotor/`).
- **FreeRTOS**: Included in ESP-IDF.
- **ESP-IDF Driver APIs**: GPIO, LEDC, PCNT (included in ESP-IDF).
- **ESP-IDF Timer**: `esp_timer` for timing functions.

---

## Examples

See the [Examples Guide](docs/EXAMPLES.md) for complete examples:

- **Basic Motion**: Simple point-to-point movement
- **Homing Sequence**: X-axis homing and calibration
- **Sequential Motion**: Pick-and-place sequence
- **FreeRTOS Integration**: Multi-task usage
- **Kinematics**: Forward/inverse kinematics usage

---

## API Reference

### Main Class: `Gantry::Gantry`

```cpp
// Construction
Gantry(const PulseMotor::DriverConfig&     xDrv,
       const PulseMotor::DrivetrainConfig& xDt,
       const PulseMotor::DriverConfig&     yDrv,
       const PulseMotor::DrivetrainConfig& yDt,
       const PulseMotor::DriverConfig&     tDrv,
       const PulseMotor::DrivetrainConfig& tDt,
       int gripperPin);

// Initialization
bool begin();
void enable();
void disable();

// Configuration
void setLimitPins(int xMinPin, int xMaxPin);
void setYAxisLimits(float minMm, float maxMm);
void setThetaLimits(float minDeg, float maxDeg);
void setJointLimits(float xMin, float xMax,
                    float yMin, float yMax,
                    float thetaMin, float thetaMax);
void setSafeYHeight(float safeHeight_mm);

// Motion Control
GantryError moveTo(const JointConfig& joint, ...);
GantryError moveTo(const EndEffectorPose& pose, ...);
bool isBusy() const;
void update();  // Call frequently in loop

// Homing & Calibration
void home();
int calibrate();

// Gripper Control
void grip(bool active);

// Status & Information
int getCurrentY() const;
int getCurrentTheta() const;
bool isAlarmActive() const;
```

See [API_REFERENCE.md](docs/API_REFERENCE.md) for complete documentation.

---

## Configuration

### Per-axis configuration

Each axis is configured before construction via a `PulseMotor::DriverConfig` (electrical) and a `PulseMotor::DrivetrainConfig` (mechanical). The recommended source of values is the pair of parameter headers `include/axis_pulse_motor_params.h` and `include/axis_drivetrain_params.h`; see `src/main.cpp` for the helper functions that translate them.

```cpp
// X-axis example (SCHUNK Beta 100-ZRS belt via Kinetix 5100)
PulseMotor::DriverConfig xDrv;
xDrv.pulse_pin       = PIN_X_PULSE_EXP;
xDrv.dir_pin         = PIN_X_DIR;
xDrv.enable_pin      = PIN_X_ENABLE;
xDrv.alarm_reset_pin = PIN_X_ALARM_RESET;
xDrv.alarm_pin       = PIN_X_ALARM_STATUS;
xDrv.encoder_a_pin   = PIN_X_ENC_A;
xDrv.encoder_b_pin   = PIN_X_ENC_B;
xDrv.encoder_ppr     = AXIS_X_ENCODER_PPR;
xDrv.max_pulse_freq  = AXIS_X_MAX_PULSE_FREQ_HZ;
xDrv.pulse_mode      = PulseMotor::PulseMode::PULSE_DIRECTION;
xDrv.enable_encoder_feedback = true;
xDrv.homing_speed_pps = AXIS_X_HOMING_SPEED_PPS;

PulseMotor::DrivetrainConfig xDt;
xDt.type                 = PulseMotor::DRIVETRAIN_BELT;
xDt.belt_lead_mm_per_rev = AXIS_X_BELT_LEAD_MM_PER_REV;
xDt.encoder_ppr          = xDrv.encoder_ppr;
xDt.motor_reducer_ratio  = AXIS_X_MOTOR_REDUCER_RATIO;
```

Runtime-tunable knobs on the Gantry instance are limited to soft limits (`setYAxisLimits`, `setThetaLimits`, `setJointLimits`), limit-switch wiring (`setLimitPins`), safe-Y (`setSafeYHeight`), and homing speed (`setHomingSpeed`).

### Safe Height Configuration

```cpp
gantry.setSafeYHeight(150.0f);  // Safe height for X-axis travel (mm)
```

---

## Thread Safety

⚠️ **NOT thread-safe by default**

All methods must be called from:
- Single FreeRTOS task (recommended), OR
- Multiple tasks with mutex protection

### Recommended Usage Pattern

```cpp
#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Single task approach (recommended)
void gantryTask(void *pvParameters) {
    Gantry::Gantry* gantry = (Gantry::Gantry*)pvParameters;
    
    // Initialize gantry in task
    gantry->begin();
    gantry->enable();
    gantry->home();
    
    while (gantry->isBusy()) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Main update loop
    while (1) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz update rate
    }
}

// In app_main():
void app_main(void) {
    // Configure and create gantry instance
    PulseMotor::DriverConfig     xDrv, yDrv, tDrv;
    PulseMotor::DrivetrainConfig xDt,  yDt,  tDt;
    // ... populate each from include/axis_pulse_motor_params.h
    //     and include/axis_drivetrain_params.h ...

    static Gantry::Gantry gantry(xDrv, xDt, yDrv, yDt, tDrv, tDt, GRIPPER_PIN);
    // ... configure axes ...
    
    // Create gantry task
    xTaskCreate(gantryTask, "Gantry", 4096, &gantry, 5, NULL);
    
    // Other initialization...
}
```

---

## Coordinate Systems

### Joint Space

- **X**: Horizontal position (mm) - right-to-left
- **Y**: Vertical position (mm) - down-to-up  
- **Theta**: Rotation angle (degrees) - around Y-axis

### Workspace Space (End-Effector)

- **X, Y, Z**: Position (mm) in workspace coordinates
- **Theta**: Orientation (degrees)

### Transformations

- Forward: `JointConfig` → `EndEffectorPose`
- Inverse: `EndEffectorPose` → `JointConfig`

See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed coordinate system documentation.

---

## Error Handling

The library uses `GantryError` enum for error reporting:

```cpp
enum class GantryError {
    OK,                      // Success
    NOT_INITIALIZED,         // Gantry not initialized
    MOTOR_NOT_ENABLED,       // Motors not enabled
    ALREADY_MOVING,          // Motion in progress
    INVALID_POSITION,        // Position out of range
    INVALID_PARAMETER,       // Invalid parameter
    TIMEOUT,                 // Operation timeout
    LIMIT_SWITCH_FAILED,     // Limit switch error
    CALIBRATION_FAILED,      // Calibration failed
    CONVERSION_ERROR         // Unit conversion error
};
```

Always check return values:

```cpp
GantryError result = gantry.moveTo(target, 50, 30);
if (result != GantryError::OK) {
    Serial.printf("Move failed: %d\n", (int)result);
}
```

---

## Memory Usage

**Estimated RAM Usage:** ~5-10 KB

| Component | RAM Usage |
|-----------|-----------|
| Gantry class instance | ~2 KB |
| Configuration structures | ~0.5 KB |
| Motion state machine | ~0.5 KB |
| Axis drivers | ~2-5 KB |
| **Total** | **~5-10 KB** |

Well within ESP32's 320KB RAM limit.

---

## Performance

- **Update Rate**: 10-100 Hz recommended (call `update()` frequently)
- **Motion Planning**: <1ms per update
- **Kinematics**: <100μs per calculation
- **Sequential Motion**: State machine overhead <1%

---

## Troubleshooting

### Common Issues

1. **Motion not starting**
   - Check `isBusy()` returns false
   - Verify motors are enabled (`enable()`)
   - Check alarm status (`isAlarmActive()`)

2. **Y-axis not moving**
   - Verify Y-axis pins configured (`setYAxisPins()`)
   - Check steps-per-mm setting
   - Verify limits are set correctly

3. **Sequential motion stuck**
   - Check safe Y height configuration
   - Verify gripper actuation time
   - Monitor motion state in debugger

4. **Limit switch issues**
   - Limit switches handled by actuator libraries
   - Check PulseMotor driver configuration (`axis_pulse_motor_params.h`)
   - Verify pin connections

---

## Contributing

This library is currently in active development. Known bugs and in-progress items are tracked in the root-level `PROGRAMMING_REFERENCE.md` (§12 "Known Bugs & Gotchas") and in `lib/Gantry/CHANGELOG.md`.

---

## License

MIT License - see [LICENSE](LICENSE) file for details.

---

## Support

For issues, questions, or contributions:
- Check the root-level `PROGRAMMING_REFERENCE.md` (§12) for known issues
- Review [API_REFERENCE.md](docs/API_REFERENCE.md) for API details
- See [EXAMPLES.md](docs/EXAMPLES.md) for usage examples

---

**Last Updated:** Feb 10th 2026  
**Version:** 1.0.0 (In Development)

