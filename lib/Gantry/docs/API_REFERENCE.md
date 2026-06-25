# Gantry Library API Reference

**Version:** 2.1.0
**Last Updated:** May 2026

Complete API documentation for the Gantry library.

> **Axis convention (2026-05):** `X` is the horizontal across-belt axis, `Z` is the vertical descent axis (ballscrew; `+Z` = up; **joint** `z` = homing datum; **`pose.z`** above bed uses `GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM`), `Theta` rotates the end-effector about `Z`. The conveyor downstream direction is `-Y` in the world frame; the gantry has no Y joint. Joint space is `(x, z, theta)`; workspace pose is `(x, y, z, theta)` where `y` is the fixed along-belt offset of the carriage. Earlier revisions of this document spoke of a "Y joint" being the vertical axis — those names have been renamed throughout. Legacy stubs `setYAxisPins`, `setYAxisStepsPerMm`, `setYAxisMotionLimits`, `setThetaServo`, `setThetaPulseRange`, and any reference to `BergerdaServo::DriverConfig` predate the 2026 refactor and no longer exist in the source tree.
>
> ```cpp
> Gantry(const PulseMotor::DriverConfig&     xDrv,
>        const PulseMotor::DrivetrainConfig& xDt,
>        const PulseMotor::DriverConfig&     zDrv,
>        const PulseMotor::DrivetrainConfig& zDt,
>        const PulseMotor::DriverConfig&     tDrv,
>        const PulseMotor::DrivetrainConfig& tDt,
>        int gripperPin);
> ```

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

Joint space configuration (internal representation). The gantry has three joints — across-belt linear, vertical linear, and rotational. **There is no `y` field.**

```cpp
struct JointConfig {
    float x;      // Across-belt position (mm); horizontal
    float z;      // Vertical position (mm); +Z = up; joint z = homing datum
    float theta;  // End-effector rotation (degrees) about Z

    JointConfig();
    JointConfig(float x_val, float z_val, float theta_val);

    JointConfig operator+(const JointConfig& other) const;
    JointConfig operator-(const JointConfig& other) const;
    JointConfig operator*(float scale) const;
};
```

**Example:**
```cpp
Gantry::JointConfig joint;
joint.x = 100.0f;
joint.z = 50.0f;
joint.theta = 45.0f;
```

### `EndEffectorPose`

End-effector pose in workspace coordinates. `x` is across-belt, `y` is along-belt (`-Y` is conveyor downstream), `z` is vertical (`+Z` up; **`pose.z`** = TCP height above physical bed via `GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM`). The `y` field of the pose comes from the fixed `GANTRY_Z_AXIS_Y_OFFSET_MM` — there is no Y joint to actuate it.

```cpp
struct EndEffectorPose {
    float x, y, z;    // Position (mm) — (across-belt, along-belt, vertical)
    float theta;      // Orientation (degrees)

    EndEffectorPose();
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val);
};
```

**Example:**
```cpp
Gantry::EndEffectorPose pose;
pose.x = 200.0f;
pose.y = -80.0f;   // Along-belt (negative downstream of the gantry origin)
pose.z = 80.0f;
pose.theta = 90.0f;
```

### `GantryStatus`

Complete status snapshot of gantry system.

```cpp
struct GantryStatus {
    // Position (current)
    int32_t currentX_mm;
    int32_t currentZ_mm;
    int32_t currentTheta_deg;

    // Target positions
    int32_t targetX_mm;
    int32_t targetZ_mm;
    int32_t targetTheta_deg;

    // Motion state
    bool isBusy;
    bool xMoving;
    bool zMoving;
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
    float z_min, z_max;
    float theta_min, theta_max;

    bool isValid(const JointConfig& config) const;
};
```

### `KinematicParameters`

Mechanical parameters for kinematics calculations.

```cpp
struct KinematicParameters {
    float z_axis_y_offset_mm;           // Z-axis carriage offset along belt (Y), default 80
    float theta_x_offset_mm;            // Default: -55 mm
    float gripper_x_offset_mm;          // Gripper offset across belt (X), default 385
    float gripper_z_offset_mm;          // Gripper offset vertical (Z), default 80
    float x_axis_ball_screw_pitch_mm;   // Default: 40 mm
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
    constexpr float DEFAULT_SAFE_Z_HEIGHT_MM = 150.0f;
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

#### `Gantry(xDrv, xDt, zDrv, zDt, tDrv, tDt, gripperPin)`

Constructs a new Gantry object from three `PulseMotor::DriverConfig` + `DrivetrainConfig` pairs (one per axis) plus a digital gripper pin.

**Parameters:**
- `xDrv` / `xDt`: Electrical + mechanical config for the across-belt X axis (typical: belt drivetrain).
- `zDrv` / `zDt`: Electrical + mechanical config for the vertical Z axis (typical: ballscrew).
- `tDrv` / `tDt`: Electrical + mechanical config for the Theta axis (rotary-direct).
- `gripperPin`: Pin for digital gripper control (`-1` to disable).

See `src/main.cpp` (`makeXDriverConfig` / `makeZDriverConfig` / `makeThetaDriverConfig` and their drivetrain counterparts) for the canonical population from `include/axis_pulse_motor_params.h` and `include/axis_drivetrain_params.h`.

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

#### `void setZAxisLimits(float minMm, float maxMm)`

Sets Z-axis travel limits (`+Z = up`; limits apply to **joint** `z` / homing datum).

**Parameters:**
- `minMm`: Minimum Z position (mm). Typical: `0` (at belt).
- `maxMm`: Maximum Z position (mm). Typical: `AXIS_Z_HARD_LIMIT_MAX_MM` (e.g. 150 for the Beta 80-SRS).

#### `void setThetaLimits(float minDeg, float maxDeg)`

Sets theta-axis angular limits.

**Parameters:**
- `minDeg`: Minimum angle (degrees)
- `maxDeg`: Maximum angle (degrees)

#### `void setJointLimits(float xMin, float xMax, float zMin, float zMax, float thetaMin, float thetaMax)`

Sets joint-space soft limits used by `moveTo(JointConfig)`.

#### `void setEndEffectorPin(int pin, bool activeHigh = true)`

Configures end-effector (gripper) pin.

**Parameters:**
- `pin`: GPIO pin for gripper control
- `activeHigh`: Active high (true) or low (false)

#### `void setSafeZHeight(float safeHeight_mm)`

Sets safe Z height for X-axis travel (the carriage retracts to this Z before any X traverse).

**Parameters:**
- `safeHeight_mm`: Safe height in millimeters above the belt (default: 150 mm).

---

### Motion Control

#### `GantryError moveTo(const JointConfig& joint, uint32_t speed_mm_per_s = 50, uint32_t speed_deg_per_s = 30, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0)`

Moves to target joint configuration.

**Parameters:**
- `joint`: Target joint configuration (`x`, `z`, `theta`)
- `speed_mm_per_s`: Motion speed for X/Z axes (mm/s)
- `speed_deg_per_s`: Motion speed for theta (deg/s)
- `acceleration_mm_per_s2`: Acceleration (0 = use default)
- `deceleration_mm_per_s2`: Deceleration (0 = use default)

**Returns:** `GantryError` code

**Motion Sequence:**
1. Z-axis descends to target Z (toward belt, since `+Z = up`)
2. Gripper actuates (close for picking, open for placing)
3. Z-axis retracts to safe height
4. X-axis traverses to target X
5. Theta moves independently

**Example:**
```cpp
Gantry::JointConfig target;
target.x = 200.0f;
target.z = 50.0f;     // 50 mm above the belt
target.theta = 45.0f;

GantryError result = gantry.moveTo(target, 50, 30);
if (result != GantryError::OK) {
    Serial.printf("Move failed: %d\n", (int)result);
}
```

#### `GantryError moveTo(const EndEffectorPose& pose, uint32_t speed_mm_per_s = 50, uint32_t speed_deg_per_s = 30, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0)`

Moves to target end-effector pose (uses inverse kinematics).

**Parameters:**
- `pose`: Target end-effector pose. The pose `y` field must match `params.z_axis_y_offset_mm` (there is no Y joint to satisfy other values).
- `speed_mm_per_s`: Motion speed for X/Z axes (mm/s)
- `speed_deg_per_s`: Motion speed for theta (deg/s)
- `acceleration_mm_per_s2`: Acceleration (0 = use default)
- `deceleration_mm_per_s2`: Deceleration (0 = use default)

**Returns:** `GantryError` code

**Example:**
```cpp
Gantry::EndEffectorPose target;
target.x = 200.0f;
target.y = -80.0f;     // Must equal the fixed Z-axis along-belt offset
target.z = 80.0f;
target.theta = 90.0f;

gantry.moveTo(target, 50, 30);
```

#### `void moveTo(int32_t x, int32_t z, int32_t theta, uint32_t speed)`

Legacy integer-mm `moveTo`. Prefer the `JointConfig` form for new code.

**Parameters:**
- `x`: Target X position (mm)
- `z`: Target Z position (mm; `+Z = up`)
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

#### `int getCurrentZ() const`

Gets current Z position.

**Returns:** Current **joint** Z position in mm (`+Z` = up; homing datum at `0`).

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
joint.z = 50.0f;
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
pose.y = -80.0f;   // along-belt; must match params.z_axis_y_offset_mm
pose.z = 80.0f;
pose.theta = 90.0f;

Gantry::JointConfig joint = gantry.inverseKinematics(pose);
Serial.printf("Joint: x=%.1f z=%.1f theta=%.1f\n",
              joint.x, joint.z, joint.theta);
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

See `src/main.cpp` for the canonical production setup. The skeleton below shows the three-axis construction path under the current API:

```cpp
#include "Gantry.h"
#include "PulseMotor.h"

// Build per-axis configs from include/axis_pulse_motor_params.h
// and include/axis_drivetrain_params.h (helpers in src/main.cpp):
PulseMotor::DriverConfig     xDrv = makeXDriverConfig();
PulseMotor::DrivetrainConfig xDt  = makeXDrivetrainConfig();
PulseMotor::DriverConfig     zDrv = makeZDriverConfig();
PulseMotor::DrivetrainConfig zDt  = makeZDrivetrainConfig();
PulseMotor::DriverConfig     tDrv = makeThetaDriverConfig();
PulseMotor::DrivetrainConfig tDt  = makeThetaDrivetrainConfig();

Gantry::Gantry::preparePinsForBoot(xDrv, zDrv, tDrv, PIN_GRIPPER);
static Gantry::Gantry gantry(xDrv, xDt, zDrv, zDt, tDrv, tDt, PIN_GRIPPER);

gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);
gantry.setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM, AXIS_X_HARD_LIMIT_MAX_MM,
                      AXIS_Z_HARD_LIMIT_MIN_MM, AXIS_Z_HARD_LIMIT_MAX_MM,
                      AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
gantry.setZAxisLimits(AXIS_Z_HARD_LIMIT_MIN_MM, AXIS_Z_HARD_LIMIT_MAX_MM);
gantry.setThetaLimits(AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
gantry.setSafeZHeight(GANTRY_SAFE_Z_HEIGHT_MM);

if (!gantry.begin()) { /* report and abort */ }
gantry.enable();
gantry.home();  // X homing
```

### Pick-and-Place Sequence

```cpp
void pickAndPlace(float pickX, float pickZ, float placeX, float placeZ) {
    // Move to pick position (descend toward belt)
    Gantry::JointConfig pickPos;
    pickPos.x = pickX;
    pickPos.z = pickZ;       // low Z = close to the belt
    pickPos.theta = 0.0f;

    gantry.moveTo(pickPos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Gripper closes automatically during the Z↓→grip→Z↑ phase of sequential motion.

    Gantry::JointConfig placePos;
    placePos.x = placeX;
    placePos.z = placeZ;
    placePos.theta = 90.0f;

    gantry.moveTo(placePos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
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

**Last Updated:** Feb 10th 2026  
**Version:** 1.0.0

