# Gantry Library Configuration Guide

**Version:** 1.0.0  
**Last Updated:** 2025-01-XX

Complete configuration guide for the Gantry library.

---

## Table of Contents

- [Quick Configuration](#quick-configuration)
- [X-Axis Configuration](#x-axis-configuration)
- [Y-Axis Configuration](#y-axis-configuration)
- [Theta-Axis Configuration](#theta-axis-configuration)
- [End-Effector Configuration](#end-effector-configuration)
- [Kinematic Parameters](#kinematic-parameters)
- [Motion Parameters](#motion-parameters)
- [Safety Configuration](#safety-configuration)
- [Tuning Guide](#tuning-guide)

---

## Quick Configuration

### Minimal Setup

```cpp
#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

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

void app_main(void) {
    // Configure X-axis driver
    BergerdaServo::DriverConfig xConfig;
    xConfig.step_pin = X_STEP_PIN;
    xConfig.dir_pin = X_DIR_PIN;
    xConfig.enable_pin = X_ENABLE_PIN;
    xConfig.encoder_ppr = 6000;
    xConfig.homing_speed_pps = 6000;
    
    // Create gantry
    static Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
    
    // Configure axes
    gantry.setLimitPins(X_MIN_LIMIT, X_MAX_LIMIT);
    gantry.setYAxisPins(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
    gantry.setYAxisStepsPerMm(200.0f);
    gantry.setYAxisLimits(0.0f, 200.0f);
    gantry.setThetaServo(THETA_PWM_PIN, 0);
    
    // Create gantry update task
    xTaskCreate([](void* pvParams) {
        Gantry::Gantry* gantry = (Gantry::Gantry*)pvParams;
        gantry->begin();
        gantry->enable();
        while (1) {
            gantry->update();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "GantryUpdate", 4096, &gantry, 5, NULL);
    
    // Other initialization...
}
```

---

## X-Axis Configuration

### SDF08NK8X Driver Configuration

The X-axis uses the SDF08NK8X servo driver library. Configure via `BergerdaServo::DriverConfig`:

```cpp
BergerdaServo::DriverConfig xConfig;

// Pin Configuration
xConfig.step_pin = 32;        // Step pulse pin
xConfig.dir_pin = 33;         // Direction pin
xConfig.enable_pin = 25;      // Enable pin (-1 to disable)
xConfig.encoder_pin_a = 18;   // Encoder A pin
xConfig.encoder_pin_b = 19;   // Encoder B pin

// Motor Parameters
xConfig.encoder_ppr = 6000;   // Encoder pulses per revolution
xConfig.microsteps = 1;       // Microstepping (1, 2, 4, 8, etc.)

// Motion Parameters
xConfig.max_speed_pps = 10000;      // Maximum speed (pulses/sec)
xConfig.max_accel_pps2 = 5000;      // Maximum acceleration
xConfig.homing_speed_pps = 6000;    // Homing speed

// Limit Switches (handled by driver)
xConfig.limit_min_pin = 35;   // Minimum limit pin
xConfig.limit_max_pin = 36;   // Maximum limit pin

// Create gantry with X config
Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
```

### X-Axis Steps Per Revolution

Calculate based on your motor and ball screw:

```cpp
// Formula: steps_per_rev = encoder_ppr * microsteps
// Example: 6000 encoder PPR * 1 microstep = 6000 steps/rev

gantry.setStepsPerRevolution(6000.0f);
```

### X-Axis Ball Screw Pitch

Set the ball screw pitch for position conversion:

```cpp
// Default: 40mm per revolution
// This affects pulses-per-mm calculation:
// pulses_per_mm = steps_per_rev / pitch_mm
// Example: 6000 / 40 = 150 pulses/mm

// Configure via kinematic parameters (if needed)
Gantry::KinematicParameters params;
params.x_axis_ball_screw_pitch_mm = 40.0f;
```

---

## Y-Axis Configuration

### Pin Configuration

```cpp
gantry.setYAxisPins(
    Y_STEP_PIN,      // Step pulse pin (required)
    Y_DIR_PIN,       // Direction pin (required)
    Y_ENABLE_PIN,    // Enable pin (optional, -1 to disable)
    false,           // Invert direction (optional, default: false)
    true             // Enable active low (optional, default: true)
);
```

### Steps Per Millimeter

Calculate based on your motor and mechanical setup:

```cpp
// Formula: steps_per_mm = (steps_per_rev * microsteps) / (pitch_mm * gear_ratio)
// Example: (200 steps/rev * 16 microsteps) / (2mm pitch * 1) = 1600 steps/mm

gantry.setYAxisStepsPerMm(1600.0f);
```

**Common Values:**
- Direct drive, 1.8° stepper, 2mm pitch: 100 steps/mm
- Direct drive, 1.8° stepper, 5mm pitch: 40 steps/mm
- With microstepping (16x): Multiply by 16

### Travel Limits

```cpp
// Set Y-axis travel limits (mm)
gantry.setYAxisLimits(
    0.0f,    // Minimum Y position (mm)
    200.0f   // Maximum Y position (mm)
);

// If limits are 0,0, limits are disabled
```

### Motion Limits

```cpp
// Set Y-axis motion limits
gantry.setYAxisMotionLimits(
    100.0f,  // Maximum speed (mm/s)
    500.0f,  // Acceleration (mm/s²)
    500.0f   // Deceleration (mm/s²)
);
```

**Recommended Values:**
- **Speed**: Start with 50-100 mm/s, increase gradually
- **Acceleration**: Start with 200-500 mm/s²
- **Deceleration**: Usually same as acceleration

### Step Pulse Width

```cpp
// Configure step pulse width (microseconds)
// Default: 2μs (minimum for most drivers)
// Increase if driver requires longer pulses

axisY_.setStepPulseWidthUs(5);  // 5μs pulse width
```

---

## Theta-Axis Configuration

### Pin Configuration

```cpp
// ESP32: Use LEDC channels (0-15)
gantry.setThetaServo(
    THETA_PWM_PIN,  // PWM output pin
    0               // LEDC channel (0-15 for ESP32)
);
```

### Angle Range

```cpp
// Set theta angular limits (degrees)
gantry.setThetaLimits(
    -90.0f,  // Minimum angle (degrees)
    90.0f    // Maximum angle (degrees)
);
```

### Pulse Width Range

```cpp
// Set servo pulse width range (microseconds)
// Standard servos: 1000-2000μs (1-2ms)
// Extended range: 500-2500μs

gantry.setThetaPulseRange(
    1000,  // Minimum pulse width (μs)
    2000   // Maximum pulse width (μs)
);
```

**Common Values:**
- Standard servo: 1000-2000μs (±90°)
- Extended servo: 500-2500μs (±180°)
- Custom servo: Check datasheet

---

## End-Effector Configuration

### Pin Configuration

```cpp
// Configure gripper pin
gantry.setEndEffectorPin(
    GRIPPER_PIN,  // GPIO pin
    true          // Active high (true) or low (false)
);
```

**Common Configurations:**
- **Active High**: Pin HIGH = gripper closed
- **Active Low**: Pin LOW = gripper closed (with pull-up)

### Gripper Actuation Time

Default: 100ms (defined in `GantryUtils.h`)

For different gripper types:
- Pneumatic: 50-100ms
- Vacuum: 30-50ms
- Solenoid: 20-50ms

Adjust if needed (modify `GRIPPER_ACTUATE_TIME_MS` constant).

---

## Kinematic Parameters

### Default Values

```cpp
struct KinematicParameters {
    float y_axis_z_offset_mm = 80.0f;           // Y-axis Z offset
    float theta_x_offset_mm = -55.0f;           // Theta X offset
    float gripper_y_offset_mm = 385.0f;         // Gripper Y offset
    float gripper_z_offset_mm = 80.0f;          // Gripper Z offset
    float x_axis_ball_screw_pitch_mm = 40.0f;   // Ball screw pitch
};
```

### Customizing Kinematic Parameters

If your gantry has different mechanical dimensions:

```cpp
// Access kinematic parameters (if exposed in future version)
// Currently uses defaults, customization planned
```

**Measurement Guide:**
1. Measure Y-axis Z offset from X-axis rail
2. Measure theta rotation center X offset
3. Measure gripper offsets from theta center
4. Measure ball screw pitch

---

## Motion Parameters

### Safe Y Height

Height at which Y-axis retracts before X-axis movement:

```cpp
// Set safe height (mm)
// Default: 150mm
gantry.setSafeYHeight(150.0f);
```

**Recommendation:**
- Set to highest expected object height + clearance
- Typical: 100-200mm depending on application

### Default Speeds

```cpp
// Default motion speeds (if not specified)
uint32_t speed_mm_per_s = 50;   // X/Y axes (mm/s)
uint32_t speed_deg_per_s = 30;  // Theta axis (deg/s)
```

**Recommended Values:**
- **X-axis**: 50-200 mm/s (depends on ball screw)
- **Y-axis**: 50-100 mm/s (depends on stepper)
- **Theta**: 30-90 deg/s (depends on servo)

### Acceleration/Deceleration

```cpp
// Specify acceleration/deceleration (mm/s²)
// 0 = use default (speed / 2)

gantry.moveTo(target, 
              50,    // speed (mm/s)
              30,    // theta speed (deg/s)
              500,   // acceleration (mm/s²)
              500);  // deceleration (mm/s²)
```

**Recommendation:**
- Start conservative: 200-500 mm/s²
- Increase gradually while monitoring vibration
- Match acceleration to deceleration for smooth motion

---

## Safety Configuration

### Limit Switches

Limit switches are configured in the actuator libraries:

**X-axis (SDF08NK8X):**
```cpp
xConfig.limit_min_pin = 35;  // Home position
xConfig.limit_max_pin = 36;  // End position
```

**Y-axis:**
```cpp
// Limits enforced via setYAxisLimits()
// No hardware limit switches required (software limits)
```

### Alarm Monitoring

```cpp
// Check alarm status
if (gantry.isAlarmActive()) {
    Serial.println("Alarm detected!");
    // Handle alarm
}
```

**Alarm Sources:**
- X-axis driver alarms
- Motion timeouts
- Limit switch violations

### Emergency Stop

```cpp
// Emergency stop
gantry.disable();  // Stops all motion and disables motors
```

---

## Tuning Guide

### Step 1: Basic Configuration

1. Configure all pins
2. Set steps-per-mm for Y-axis
3. Set basic limits
4. Initialize and enable

### Step 2: Test Individual Axes

**X-axis:**
```cpp
// Test X-axis homing
gantry.home();
while (gantry.isBusy()) {
    gantry.update();
    delay(10);
}
```

**Y-axis:**
```cpp
// Test Y-axis movement
Gantry::JointConfig test;
test.x = 0.0f;
test.y = 50.0f;  // Move Y to 50mm
test.theta = 0.0f;
gantry.moveTo(test, 50, 30);
```

**Theta-axis:**
```cpp
// Test theta rotation
test.theta = 45.0f;
gantry.moveTo(test, 50, 30);
```

### Step 3: Tune Motion Parameters

**Start Conservative:**
```cpp
// Low speeds, moderate acceleration
gantry.moveTo(target, 30, 20, 200, 200);
```

**Gradually Increase:**
```cpp
// Increase speed 10% at a time
gantry.moveTo(target, 50, 30, 400, 400);
gantry.moveTo(target, 70, 40, 600, 600);
```

**Monitor:**
- Vibration
- Missed steps
- Mechanical stress
- Position accuracy

### Step 4: Tune Sequential Motion

**Adjust Safe Height:**
```cpp
// Set based on your application
gantry.setSafeYHeight(150.0f);  // Adjust as needed
```

**Test Pick-and-Place:**
```cpp
// Test full sequence
Gantry::JointConfig pickPos(200.0f, 30.0f, 0.0f);
gantry.moveTo(pickPos, 50, 30);
// ... wait for completion ...

Gantry::JointConfig placePos(400.0f, 30.0f, 90.0f);
gantry.moveTo(placePos, 50, 30);
```

### Step 5: Fine-Tune

**Optimize Speeds:**
- Find maximum reliable speed for each axis
- Balance speed vs. accuracy
- Consider mechanical limitations

**Optimize Acceleration:**
- Reduce vibration
- Minimize settling time
- Prevent missed steps

**Optimize Safe Height:**
- Minimize unnecessary Y movement
- Ensure collision-free X travel
- Balance speed vs. safety

---

## Configuration Checklist

### Initial Setup

- [ ] X-axis pins configured
- [ ] Y-axis pins configured
- [ ] Theta-axis pin configured
- [ ] Gripper pin configured
- [ ] Limit switches configured (X-axis)
- [ ] Steps-per-mm calculated (Y-axis)
- [ ] Limits set (all axes)
- [ ] Motion limits set (Y-axis)
- [ ] Safe height configured

### Calibration

- [ ] X-axis homed
- [ ] X-axis calibrated (axis length measured)
- [ ] Y-axis zero position verified
- [ ] Theta zero position verified
- [ ] Gripper open/close verified

### Testing

- [ ] Individual axis movement tested
- [ ] Sequential motion tested
- [ ] Pick-and-place sequence tested
- [ ] Limits verified
- [ ] Alarm handling tested

---

## Troubleshooting

### Y-axis Not Moving

1. Check pin configuration
2. Verify steps-per-mm setting
3. Check limits (may be at limit)
4. Verify enable pin (if used)
5. Check motion limits (speed/accel)

### Sequential Motion Issues

1. Check safe height setting
2. Verify gripper actuation time
3. Monitor motion state
4. Check for alarms

### Position Accuracy Issues

1. Verify steps-per-mm calculation
2. Check for missed steps
3. Verify encoder feedback (X-axis)
4. Check mechanical backlash

---

**Last Updated:** 2025-01-XX  
**Version:** 1.0.0
