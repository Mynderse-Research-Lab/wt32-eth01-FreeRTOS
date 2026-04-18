# Gantry Library Configuration Guide

**Version:** 1.0.0  
**Last Updated:** Feb 10th 2026

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
    // Populate the three DriverConfigs (pins, encoder PPR, inversion, LEDC bits)
    PulseMotor::DriverConfig xConfig, yConfig, thetaConfig;
    xConfig.pulse_pin   = PIN_X_PULSE;
    xConfig.dir_pin     = PIN_X_DIR;
    xConfig.enable_pin  = PIN_X_ENABLE;
    xConfig.encoder_ppr = AXIS_X_ENCODER_PPR;
    // ... fill yConfig and thetaConfig in the same shape ...

    static Gantry::Gantry gantry(xConfig, yConfig, thetaConfig, GRIPPER_PIN);

    // X-axis soft limits
    gantry.setLimitPins(X_MIN_LIMIT, X_MAX_LIMIT);

    // Per-axis drivetrain conversion
    PulseMotor::DrivetrainConfig xDt;
    xDt.type                 = PulseMotor::DrivetrainType::BELT;
    xDt.belt_lead_mm_per_rev = AXIS_X_BELT_LEAD_MM_PER_REV;
    xDt.encoder_ppr          = AXIS_X_ENCODER_PPR;
    xDt.motor_reducer_ratio  = AXIS_X_MOTOR_REDUCER_RATIO;
    gantry.setXDrivetrain(xDt);
    // ... setYDrivetrain(...) and setThetaDrivetrain(...) ...

    gantry.setJointLimits(AXIS_X_TRAVEL_MIN_MM, AXIS_X_TRAVEL_MAX_MM,
                          AXIS_Y_TRAVEL_MIN_MM, AXIS_Y_TRAVEL_MAX_MM,
                          AXIS_THETA_TRAVEL_MIN_DEG, AXIS_THETA_TRAVEL_MAX_DEG);
    gantry.setSafeYHeight(GANTRY_SAFE_Y_HEIGHT_MM);

    xTaskCreate([](void* pvParams) {
        Gantry::Gantry* g = (Gantry::Gantry*)pvParams;
        g->begin();
        g->enable();
        while (1) {
            g->update();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "GantryUpdate", 4096, &gantry, 5, NULL);
}
```

---

## X / Y / Theta Axis Configuration

All three axes use the same driver library (`PulseMotor::PulseMotorDriver`).
Each axis needs **two** configurations:

1. **`PulseMotor::DriverConfig`** — electrical side (pins, encoder PPR,
   pulse-frequency cap, inversion, LEDC bits). Fed into the `Gantry`
   constructor.
2. **`PulseMotor::DrivetrainConfig`** — mechanical side (drivetrain type +
   lead/pitch, encoder PPR, motor reducer ratio). Installed via
   `setXDrivetrain()`, `setYDrivetrain()`, `setThetaDrivetrain()`.

### Example: X-axis (belt)

```cpp
PulseMotor::DriverConfig xConfig;
xConfig.pulse_pin       = PIN_X_PULSE;
xConfig.dir_pin         = PIN_X_DIR;
xConfig.enable_pin      = PIN_X_ENABLE;
xConfig.alarm_pin       = PIN_X_ALARM_STATUS;
xConfig.alarm_reset_pin = PIN_X_ALARM_RESET;
xConfig.encoder_a_pin   = PIN_X_ENC_A;
xConfig.encoder_b_pin   = PIN_X_ENC_B;
xConfig.enable_encoder_feedback = true;
xConfig.encoder_ppr     = AXIS_X_ENCODER_PPR;
xConfig.max_pulse_freq  = AXIS_X_MAX_PULSE_FREQ_HZ;
xConfig.invert_dir_pin  = AXIS_X_INVERT_DIR;

PulseMotor::DrivetrainConfig xDt;
xDt.type                 = PulseMotor::DrivetrainType::BELT;
xDt.belt_lead_mm_per_rev = AXIS_X_BELT_LEAD_MM_PER_REV;
xDt.encoder_ppr          = AXIS_X_ENCODER_PPR;
xDt.motor_reducer_ratio  = AXIS_X_MOTOR_REDUCER_RATIO;
// ... later, after constructing the Gantry ...
gantry.setXDrivetrain(xDt);
```

### Example: Y-axis (ballscrew)

```cpp
PulseMotor::DrivetrainConfig yDt;
yDt.type                = PulseMotor::DrivetrainType::BALLSCREW;
yDt.lead_mm             = AXIS_Y_BALLSCREW_LEAD_MM;
yDt.encoder_ppr         = AXIS_Y_ENCODER_PPR;
yDt.motor_reducer_ratio = AXIS_Y_MOTOR_REDUCER_RATIO;
gantry.setYDrivetrain(yDt);

gantry.setYAxisLimits(AXIS_Y_TRAVEL_MIN_MM, AXIS_Y_TRAVEL_MAX_MM);
```

### Example: Theta-axis (rotary direct)

```cpp
PulseMotor::DrivetrainConfig thetaDt;
thetaDt.type                = PulseMotor::DrivetrainType::ROTARY_DIRECT;
thetaDt.output_gear_ratio   = AXIS_THETA_OUTPUT_GEAR_RATIO;
thetaDt.encoder_ppr         = AXIS_THETA_ENCODER_PPR;
thetaDt.motor_reducer_ratio = AXIS_THETA_MOTOR_REDUCER_RATIO;
gantry.setThetaDrivetrain(thetaDt);

gantry.setThetaLimits(AXIS_THETA_TRAVEL_MIN_DEG, AXIS_THETA_TRAVEL_MAX_DEG);
```

### Per-move speed / accel / decel

Motion limits are passed per move through the `moveTo(JointConfig, ...)`
arguments. The defaults live in `include/axis_drivetrain_params.h`:

```c
#define AXIS_X_MAX_SPEED_MM_PER_S 500.0f
#define AXIS_X_ACCEL_MM_PER_S2   3000.0f
#define AXIS_X_DECEL_MM_PER_S2   3000.0f
```

Call `gantry.moveTo(target, speed_mm_per_s, speed_deg_per_s, accel, decel)`
to override them per command.

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
    float y_axis_z_offset_mm = 80.0f;    // Y-axis Z offset
    float theta_x_offset_mm = -55.0f;    // Theta X offset
    float gripper_y_offset_mm = 385.0f;  // Gripper Y offset
    float gripper_z_offset_mm = 80.0f;   // Gripper Z offset
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

**X-axis (PulseMotor):**
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

**Last Updated:** Feb 10th 2026  
**Version:** 1.0.0

