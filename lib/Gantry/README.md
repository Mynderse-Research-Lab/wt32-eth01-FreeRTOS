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

The Gantry library provides a complete motion control system for a 3-axis gantry robot:

- **X-axis**: Horizontal prismatic joint (ball-screw driven via SDF08NK8X servo driver)
- **Y-axis**: Vertical prismatic joint (step/dir stepper motor)
- **Theta-axis**: Rotational joint (PWM servo motor, inline rotary)
- **End-effector**: Digital output gripper control

The library implements sequential motion planning, forward/inverse kinematics, trajectory planning, and safety features suitable for industrial automation applications.

---

## Features

### ✅ Implemented Features

- **Multi-axis Control**
  - X-axis: SDF08NK8X servo driver with encoder feedback
  - Y-axis: Step/dir stepper motor with trapezoidal velocity profiles
  - Theta-axis: PWM servo control (ESP32 LEDC or standard Servo library)
  - End-effector: Digital output gripper control

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

// Configure X-axis servo driver
BergerdaServo::DriverConfig xConfig;
xConfig.encoder_ppr = 6000;
xConfig.step_pin = 32;
xConfig.dir_pin = 33;
// ... configure other pins ...

// Create gantry instance
Gantry::Gantry gantry(xConfig, GRIPPER_PIN);

// Configure axes
gantry.setLimitPins(MIN_LIMIT_PIN, MAX_LIMIT_PIN);
gantry.setYAxisPins(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
gantry.setYAxisStepsPerMm(200.0f);  // 200 steps/mm
gantry.setYAxisLimits(0.0f, 200.0f);  // 0-200mm travel
gantry.setThetaServo(THETA_PWM_PIN, 0);

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
├── Gantry.h/cpp          - Main gantry control class
├── GantryConfig.h/cpp    - Configuration structures
├── GantryKinematics.h/cpp - Forward/inverse kinematics
├── GantryTrajectory.h/cpp - Trajectory planning
├── GantryAxisStepper.h/cpp - Y-axis stepper driver
├── GantryRotaryServo.h/cpp - Theta-axis servo driver
├── GantryEndEffector.h/cpp - Gripper control
└── GantryUtils.h         - Constants and macros
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
| X-axis (SDF08NK8X) | ✅ Complete | Full integration with servo driver |
| Y-axis (Stepper) | ✅ Complete | Step/dir with accel/decel profiles |
| Theta-axis (Servo) | ✅ Complete | PWM servo control |
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
        sdf08nk8x
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
        "src/GantryAxisStepper.cpp"
        "src/GantryRotaryServo.cpp"
        "src/GantryEndEffector.cpp"
    INCLUDE_DIRS 
        "src"
    REQUIRES
        sdf08nk8x
        freertos
        driver
        esp_timer
)
```

### Dependencies

- **SDF08NK8X**: Servo driver component for X-axis (ESP-IDF component)
- **FreeRTOS**: Included in ESP-IDF
- **ESP-IDF Driver APIs**: GPIO, LEDC, PCNT (included in ESP-IDF)
- **ESP-IDF Timer**: `esp_timer` for timing functions

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
Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin);

// Initialization
bool begin();
void enable();
void disable();

// Configuration
void setLimitPins(int xMinPin, int xMaxPin);
void setYAxisPins(int stepPin, int dirPin, int enablePin = -1, ...);
void setYAxisStepsPerMm(float stepsPerMm);
void setYAxisLimits(float minMm, float maxMm);
void setThetaServo(int pwmPin, int pwmChannel = 0);
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

### X-Axis Configuration

Configure via `BergerdaServo::DriverConfig`:

```cpp
BergerdaServo::DriverConfig xConfig;
xConfig.encoder_ppr = 6000;
xConfig.step_pin = 32;
xConfig.dir_pin = 33;
xConfig.enable_pin = 25;
xConfig.homing_speed_pps = 6000;
// ... more configuration ...
```

### Y-Axis Configuration

```cpp
gantry.setYAxisPins(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
gantry.setYAxisStepsPerMm(200.0f);  // Steps per millimeter
gantry.setYAxisLimits(0.0f, 200.0f);  // Min/max travel
gantry.setYAxisMotionLimits(100.0f, 500.0f, 500.0f);  // Max speed, accel, decel
```

### Theta-Axis Configuration

```cpp
gantry.setThetaServo(THETA_PWM_PIN, PWM_CHANNEL);
gantry.setThetaLimits(-90.0f, 90.0f);  // Angle range
gantry.setThetaPulseRange(1000, 2000);  // Pulse width range (μs)
```

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
    BergerdaServo::DriverConfig xConfig;
    // ... configure xConfig ...
    
    static Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
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
   - Check SDF08NK8X driver configuration
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

