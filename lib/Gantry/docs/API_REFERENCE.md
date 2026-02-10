# Gantry Library API Reference

**Version:** 1.0.0  
**Last Updated:** 2025-01-XX

Complete API documentation for the Gantry library.

---

## Table of Contents

- [Main Classes](#main-classes)
- [Data Structures](#data-structures)
- [Enumerations](#enumerations)
- [Constants](#constants)
- [API Methods](#api-methods)
- [Usage Examples](#usage-examples)

---

## Main Classes

### `Gantry::Gantry`

Main gantry control class providing unified interface for multi-axis motion control.

**Location:** `Gantry.h`

**Namespace:** `Gantry`

---

## Data Structures

### `JointConfig`

Joint space configuration (internal representation).

```cpp
struct JointConfig {
    float x;      // X-axis position (mm) - horizontal (right-to-left)
    float y;      // Y-axis position (mm) - vertical (down-to-up)
    float theta;  // Theta angle (degrees) - rotation around Y-axis
    
    // Constructors
    JointConfig();
    JointConfig(float x_val, float y_val, float theta_val);
    
    // Operators
    JointConfig operator+(const JointConfig& other) const;
    JointConfig operator-(const JointConfig& other) const;
    JointConfig operator*(float scale) const;
};
```

**Example:**
```cpp
Gantry::JointConfig joint;
joint.x = 100.0f;
joint.y = 50.0f;
joint.theta = 45.0f;
```

### `EndEffectorPose`

End-effector pose in workspace/cartesian coordinates.

```cpp
struct EndEffectorPose {
    float x, y, z;    // Position (mm)
    float theta;      // Orientation (degrees)
    
    EndEffectorPose();
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val);
};
```

**Example:**
```cpp
Gantry::EndEffectorPose pose;
pose.x = 200.0f;
pose.y = 100.0f;
pose.z = 80.0f;  // Constant Z offset
pose.theta = 90.0f;
```

### `GantryStatus`

Complete status snapshot of gantry system.

```cpp
struct GantryStatus {
    // Position (current)
    int32_t currentX_mm;
    int32_t currentY_mm;
    int32_t currentTheta_deg;
    
    // Target positions
    int32_t targetX_mm;
    int32_t targetY_mm;
    int32_t targetTheta_deg;
    
    // Motion state
    bool isBusy;
    bool xMoving;
    bool yMoving;
    bool thetaMoving;
    
    // System state
    bool initialized;
    bool enabled;
    bool gripperActive;
    bool alarmActive;
    
    // Configuration
    int32_t axisLength_mm;
    int32_t workspaceOriginOffset_mm;
    
    // Timestamp
    uint32_t lastUpdate_ms;
};
```

### `JointLimits`

Joint limits for validation.

```cpp
struct JointLimits {
    float x_min, x_max;
    float y_min, y_max;
    float theta_min, theta_max;
    
    bool isValid(const JointConfig& config) const;
};
```

### `KinematicParameters`

Mechanical parameters for kinematics calculations.

```cpp
struct KinematicParameters {
    float y_axis_z_offset_mm;           // Default: 80mm
    float theta_x_offset_mm;            // Default: -55mm
    float gripper_y_offset_mm;          // Default: 385mm
    float gripper_z_offset_mm;          // Default: 80mm
    float x_axis_ball_screw_pitch_mm;   // Default: 40mm
};
```

---

## Enumerations

### `GantryError`

Error codes for Gantry operations.

```cpp
enum class GantryError {
    OK,                      // Operation successful
    NOT_INITIALIZED,         // Gantry not initialized
    MOTOR_NOT_ENABLED,       // Motor not enabled
    ALREADY_MOVING,          // Motion already in progress
    INVALID_POSITION,        // Position out of valid range
    INVALID_PARAMETER,       // Invalid parameter (speed, etc.)
    TIMEOUT,                 // Operation timed out
    LIMIT_SWITCH_FAILED,     // Limit switch not triggered
    CALIBRATION_FAILED,      // Calibration failed
    CONVERSION_ERROR         // Unit conversion error
};
```

### `HomingStatus`

Status of homing operation.

```cpp
enum class HomingStatus {
    IDLE,        // Not homing
    IN_PROGRESS, // Homing in progress
    COMPLETE,    // Homing completed successfully
    FAILED       // Homing failed
};
```

---

## Constants

Defined in `GantryUtils.h`:

```cpp
namespace Gantry::Constants {
    constexpr float DEFAULT_STEPS_PER_REV = 6000.0f;
    constexpr float DEFAULT_PULSES_PER_MM = 150.0f;
    constexpr float DEFAULT_SAFE_Y_HEIGHT_MM = 150.0f;
    constexpr uint32_t DEFAULT_HOMING_SPEED_PPS = 6000;
    constexpr uint32_t DEFAULT_SPEED_MM_PER_S = 50;
    constexpr uint32_t DEFAULT_SPEED_DEG_PER_S = 30;
    constexpr uint32_t GRIPPER_ACTUATE_TIME_MS = 100;
    constexpr uint32_t CALIBRATION_TIMEOUT_MS = 30000;
    constexpr uint32_t TRAVEL_MEASUREMENT_TIMEOUT_MS = 90000;
}
```

---

## API Methods

### Construction & Initialization

#### `Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin)`

Constructs a new Gantry object.

**Parameters:**
- `xConfig`: Configuration for X-axis servo driver (SDF08NK8X)
- `gripperPin`: GPIO pin for gripper control (-1 to disable)

**Example:**
```cpp
BergerdaServo::DriverConfig xConfig;
// ... configure xConfig ...
Gantry::Gantry gantry(xConfig, 26);
```

#### `bool begin()`

Initializes the gantry system. Must be called before use.

**Returns:** `true` if successful, `false` on failure

**Example:**
```cpp
if (!gantry.begin()) {
    Serial.println("Gantry initialization failed!");
    return;
}
```

#### `void enable()`

Enables all axes. Motors are enabled and ready for motion.

**Note:** Must call `begin()` first.

#### `void disable()`

Disables all axes and stops any motion in progress.

---

### Configuration

#### `void setLimitPins(int xMinPin, int xMaxPin)`

Sets limit switch pins for X-axis.

**Parameters:**
- `xMinPin`: Minimum limit pin (home position)
- `xMaxPin`: Maximum limit pin (end position)

**Note:** Call before `begin()`.

#### `void setYAxisPins(int stepPin, int dirPin, int enablePin = -1, bool invertDir = false, bool enableActiveLow = true)`

Configures Y-axis stepper motor pins.

**Parameters:**
- `stepPin`: Step pulse pin
- `dirPin`: Direction pin
- `enablePin`: Enable pin (-1 to disable)
- `invertDir`: Invert direction signal
- `enableActiveLow`: Enable pin active low (true) or high (false)

#### `void setYAxisStepsPerMm(float stepsPerMm)`

Sets Y-axis steps-per-millimeter conversion.

**Parameters:**
- `stepsPerMm`: Steps per millimeter (e.g., 200 for 200 steps/mm)

#### `void setYAxisLimits(float minMm, float maxMm)`

Sets Y-axis travel limits.

**Parameters:**
- `minMm`: Minimum Y position (mm)
- `maxMm`: Maximum Y position (mm)

#### `void setYAxisMotionLimits(float maxSpeedMmPerS, float accelMmPerS2, float decelMmPerS2)`

Sets Y-axis motion limits.

**Parameters:**
- `maxSpeedMmPerS`: Maximum speed (mm/s)
- `accelMmPerS2`: Acceleration (mm/s²)
- `decelMmPerS2`: Deceleration (mm/s²)

#### `void setThetaServo(int pwmPin, int pwmChannel = 0)`

Configures theta-axis servo PWM pin.

**Parameters:**
- `pwmPin`: PWM output pin
- `pwmChannel`: PWM channel (ESP32 LEDC channel, 0-15)

#### `void setThetaLimits(float minDeg, float maxDeg)`

Sets theta-axis angular limits.

**Parameters:**
- `minDeg`: Minimum angle (degrees)
- `maxDeg`: Maximum angle (degrees)

#### `void setThetaPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs)`

Sets theta servo pulse width range.

**Parameters:**
- `minPulseUs`: Minimum pulse width (microseconds)
- `maxPulseUs`: Maximum pulse width (microseconds)

#### `void setEndEffectorPin(int pin, bool activeHigh = true)`

Configures end-effector (gripper) pin.

**Parameters:**
- `pin`: GPIO pin for gripper control
- `activeHigh`: Active high (true) or low (false)

#### `void setSafeYHeight(float safeHeight_mm)`

Sets safe Y height for X-axis travel.

**Parameters:**
- `safeHeight_mm`: Safe height in millimeters (default: 150mm)

---

### Motion Control

#### `GantryError moveTo(const JointConfig& joint, uint32_t speed_mm_per_s = 50, uint32_t speed_deg_per_s = 30, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0)`

Moves to target joint configuration.

**Parameters:**
- `joint`: Target joint configuration
- `speed_mm_per_s`: Motion speed for X/Y axes (mm/s)
- `speed_deg_per_s`: Motion speed for theta (deg/s)
- `acceleration_mm_per_s2`: Acceleration (0 = use default)
- `deceleration_mm_per_s2`: Deceleration (0 = use default)

**Returns:** `GantryError` code

**Motion Sequence:**
1. Y-axis descends to target Y (if needed)
2. Gripper actuates (close for picking, open for placing)
3. Y-axis retracts to safe height
4. X-axis moves to target X
5. Theta moves independently

**Example:**
```cpp
Gantry::JointConfig target;
target.x = 200.0f;
target.y = 50.0f;
target.theta = 45.0f;

GantryError result = gantry.moveTo(target, 50, 30);
if (result != GantryError::OK) {
    Serial.printf("Move failed: %d\n", (int)result);
}
```

#### `GantryError moveTo(const EndEffectorPose& pose, uint32_t speed_mm_per_s = 50, uint32_t speed_deg_per_s = 30, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0)`

Moves to target end-effector pose (uses inverse kinematics).

**Parameters:**
- `pose`: Target end-effector pose
- `speed_mm_per_s`: Motion speed for X/Y axes (mm/s)
- `speed_deg_per_s`: Motion speed for theta (deg/s)
- `acceleration_mm_per_s2`: Acceleration (0 = use default)
- `deceleration_mm_per_s2`: Deceleration (0 = use default)

**Returns:** `GantryError` code

**Example:**
```cpp
Gantry::EndEffectorPose target;
target.x = 200.0f;
target.y = 100.0f;
target.z = 80.0f;
target.theta = 90.0f;

gantry.moveTo(target, 50, 30);
```

#### `void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed)`

Legacy moveTo method (deprecated, use JointConfig version).

**Parameters:**
- `x`: Target X position (mm)
- `y`: Target Y position (mm)
- `theta`: Target theta angle (degrees)
- `speed`: Speed in pulses per second (for X axis)

#### `bool isBusy() const`

Checks if gantry is currently moving.

**Returns:** `true` if any axis is moving or motion state machine is active

**Example:**
```cpp
gantry.moveTo(target, 50, 30);
while (gantry.isBusy()) {
    gantry.update();
    delay(10);
}
```

#### `void update()`

Update function - must be called frequently in main loop.

**Note:** Call at 10-100 Hz for proper motion control.

**Example:**
```cpp
void loop() {
    gantry.update();
    delay(10);  // ~100 Hz update rate
}
```

---

### Homing & Calibration

#### `void home()`

Homes the X-axis to minimum limit switch.

**Note:** Requires limit pins configured and motors enabled.

**Example:**
```cpp
gantry.enable();
gantry.home();
while (gantry.isBusy()) {
    gantry.update();
    delay(10);
}
```

#### `int calibrate()`

Calibrates X-axis length by measuring travel from MIN to MAX limit.

**Returns:** Axis length in mm, or 0 on failure

**Example:**
```cpp
int axisLength = gantry.calibrate();
if (axisLength > 0) {
    Serial.printf("Axis length: %d mm\n", axisLength);
} else {
    Serial.println("Calibration failed!");
}
```

---

### Gripper Control

#### `void grip(bool active)`

Controls the gripper (end-effector).

**Parameters:**
- `active`: `true` to close gripper, `false` to open

**Example:**
```cpp
gantry.grip(true);   // Close gripper
delay(100);
gantry.grip(false);  // Open gripper
```

---

### Status & Information

#### `int getXEncoder() const`

Gets X-axis encoder position.

**Returns:** Encoder position in pulses

#### `int getCurrentY() const`

Gets current Y position.

**Returns:** Current Y position in mm

#### `int getCurrentTheta() const`

Gets current theta angle.

**Returns:** Current theta angle in degrees

#### `bool isAlarmActive() const`

Checks if alarm condition is active.

**Returns:** `true` if alarm is active

#### `void setHomingSpeed(uint32_t speed_pps)`

Sets homing speed for X-axis.

**Parameters:**
- `speed_pps`: Homing speed in pulses per second

---

### Kinematics

#### `EndEffectorPose forwardKinematics(const JointConfig& joint) const`

Forward kinematics: Joint space → Workspace.

**Parameters:**
- `joint`: Joint configuration

**Returns:** End-effector pose in workspace coordinates

**Example:**
```cpp
Gantry::JointConfig joint;
joint.x = 100.0f;
joint.y = 50.0f;
joint.theta = 45.0f;

Gantry::EndEffectorPose pose = gantry.forwardKinematics(joint);
Serial.printf("End-effector: x=%.1f y=%.1f z=%.1f theta=%.1f\n",
              pose.x, pose.y, pose.z, pose.theta);
```

#### `JointConfig inverseKinematics(const EndEffectorPose& pose) const`

Inverse kinematics: Workspace → Joint space.

**Parameters:**
- `pose`: End-effector pose

**Returns:** Required joint configuration

**Example:**
```cpp
Gantry::EndEffectorPose pose;
pose.x = 200.0f;
pose.y = 100.0f;
pose.z = 80.0f;
pose.theta = 90.0f;

Gantry::JointConfig joint = gantry.inverseKinematics(pose);
Serial.printf("Joint: x=%.1f y=%.1f theta=%.1f\n",
              joint.x, joint.y, joint.theta);
```

#### `JointConfig getCurrentJointConfig() const`

Gets current joint configuration.

**Returns:** Current joint positions

#### `JointConfig getTargetJointConfig() const`

Gets target joint configuration.

**Returns:** Target joint positions

#### `EndEffectorPose getCurrentEndEffectorPose() const`

Gets current end-effector pose.

**Returns:** Current end-effector pose

#### `EndEffectorPose getTargetEndEffectorPose() const`

Gets target end-effector pose.

**Returns:** Target end-effector pose

---

### Configuration Accessors

#### `void setStepsPerRevolution(float steps_per_rev)`

Sets steps per motor revolution for X-axis.

**Parameters:**
- `steps_per_rev`: Steps per revolution (default: 6000)

#### `float getStepsPerRevolution() const`

Gets steps per motor revolution.

**Returns:** Steps per revolution

#### `float getPulsesPerMm() const`

Gets pulses per millimeter for X-axis.

**Returns:** Pulses per millimeter

---

## Usage Examples

### Complete Setup Example

```cpp
#include "Gantry.h"

// Pin definitions
#define X_STEP_PIN 32
#define X_DIR_PIN 33
#define X_ENABLE_PIN 25
#define X_MIN_LIMIT 35
#define X_MAX_LIMIT 36
#define Y_STEP_PIN 26
#define Y_DIR_PIN 27
#define Y_ENABLE_PIN 14
#define THETA_PWM_PIN 13
#define GRIPPER_PIN 12

Gantry::Gantry gantry(BergerdaServo::DriverConfig(), GRIPPER_PIN);

void setup() {
    Serial.begin(115200);
    
    // Configure X-axis (via driver config)
    BergerdaServo::DriverConfig xConfig;
    xConfig.step_pin = X_STEP_PIN;
    xConfig.dir_pin = X_DIR_PIN;
    xConfig.enable_pin = X_ENABLE_PIN;
    xConfig.encoder_ppr = 6000;
    // ... more X-axis config ...
    
    // Create gantry with X config
    gantry = Gantry::Gantry(xConfig, GRIPPER_PIN);
    
    // Configure limit switches
    gantry.setLimitPins(X_MIN_LIMIT, X_MAX_LIMIT);
    
    // Configure Y-axis
    gantry.setYAxisPins(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
    gantry.setYAxisStepsPerMm(200.0f);
    gantry.setYAxisLimits(0.0f, 200.0f);
    gantry.setYAxisMotionLimits(100.0f, 500.0f, 500.0f);
    
    // Configure theta-axis
    gantry.setThetaServo(THETA_PWM_PIN, 0);
    gantry.setThetaLimits(-90.0f, 90.0f);
    
    // Set safe height
    gantry.setSafeYHeight(150.0f);
    
    // Initialize
    if (!gantry.begin()) {
        Serial.println("Initialization failed!");
        return;
    }
    
    gantry.enable();
    
    // Home X-axis
    gantry.home();
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
    }
    
    Serial.println("Gantry ready!");
}

void loop() {
    gantry.update();
    delay(10);
}
```

### Pick-and-Place Sequence

```cpp
void pickAndPlace(float pickX, float pickY, float placeX, float placeY) {
    // Move to pick position
    Gantry::JointConfig pickPos;
    pickPos.x = pickX;
    pickPos.y = pickY;  // Low position
    pickPos.theta = 0.0f;
    
    gantry.moveTo(pickPos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
    }
    
    // Gripper closes automatically during sequential motion
    
    // Move to place position
    Gantry::JointConfig placePos;
    placePos.x = placeX;
    placePos.y = placeY;  // Low position
    placePos.theta = 90.0f;
    
    gantry.moveTo(placePos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
    }
    
    // Gripper opens automatically during sequential motion
}
```

---

## Error Handling

Always check return values:

```cpp
GantryError result = gantry.moveTo(target, 50, 30);
switch (result) {
    case GantryError::OK:
        Serial.println("Move successful");
        break;
    case GantryError::NOT_INITIALIZED:
        Serial.println("Gantry not initialized");
        break;
    case GantryError::MOTOR_NOT_ENABLED:
        Serial.println("Motors not enabled");
        break;
    case GantryError::ALREADY_MOVING:
        Serial.println("Motion already in progress");
        break;
    case GantryError::INVALID_POSITION:
        Serial.println("Position out of range");
        break;
    default:
        Serial.printf("Unknown error: %d\n", (int)result);
        break;
}
```

---

## Thread Safety Notes

⚠️ **NOT thread-safe by default**

- All methods must be called from a single FreeRTOS task, OR
- Use mutex protection for multi-task access
- `update()` must be called from the same task context

---

**Last Updated:** 2025-01-XX  
**Version:** 1.0.0
