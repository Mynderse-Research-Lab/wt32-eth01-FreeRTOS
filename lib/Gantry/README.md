# Gantry Library

**Version:** 2.0.0
**Platform:** ESP32 (WT32-ETH01)
**Framework:** ESP-IDF (FreeRTOS) / Arduino-ESP32

Multi-axis gantry control library for ESP32. Three pulse-train axes (X, Y, Theta) driven through `PulseMotor::PulseMotorDriver`, with safe-height motion sequencing, kinematics, and trajectory planning for pick-and-place applications.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
- [Documentation](#documentation)
- [Installation](#installation)
- [API Reference](#api-reference)
- [Configuration](#configuration)
- [Thread Safety](#thread-safety)
- [Coordinate Systems](#coordinate-systems)
- [Error Handling](#error-handling)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

Three axes, one driver family:

- **X axis** — linear (belt or ballscrew), driven by `PulseMotor::PulseMotorDriver`.
- **Y axis** — linear (belt or ballscrew), driven by `PulseMotor::PulseMotorDriver`.
- **Theta axis** — rotary-direct, driven by `PulseMotor::PulseMotorDriver`.
- **End-effector** — digital output (pneumatic gripper).

Per-axis mm↔pulse (and deg↔pulse for Theta) conversion is supplied by
`PulseMotor::DrivetrainConfig` records installed via `setXDrivetrain()`,
`setYDrivetrain()`, `setThetaDrivetrain()`.

---

## Features

- **Three pulse-train axes** under a single driver family (`PulseMotor::PulseMotorDriver`) — no stepper or PWM-hobby-servo fallbacks.
- **Sequential motion planning** — Y descends to target → gripper actuates → Y retracts to safe height → X translates.
- **Kinematics** — forward/inverse mapping between joint space and workspace, plus joint-limit validation.
- **Homing + calibration** for X-axis using MIN/MAX limit switches.
- **Alarm propagation** from every configured driver and a unified `clearAlarm()`.
- **Non-blocking FreeRTOS-compatible design.**

---

## Quick Start

```cpp
#include "Gantry.h"
#include "axis_pulse_motor_params.h"
#include "axis_drivetrain_params.h"

// ---- 1) Build three PulseMotor::DriverConfig objects (X, Y, Theta) ----
PulseMotor::DriverConfig xConfig;
xConfig.pulse_pin        = PIN_X_PULSE_EXP;
xConfig.dir_pin          = PIN_X_DIR;
xConfig.enable_pin       = PIN_X_ENABLE;
xConfig.alarm_pin        = PIN_X_ALARM_STATUS;
xConfig.alarm_reset_pin  = PIN_X_ALARM_RESET;
xConfig.encoder_a_pin    = PIN_X_ENC_A;
xConfig.encoder_b_pin    = PIN_X_ENC_B;
xConfig.enable_encoder_feedback = true;
xConfig.encoder_ppr      = AXIS_X_ENCODER_PPR;
xConfig.max_pulse_freq   = AXIS_X_MAX_PULSE_FREQ_HZ;
xConfig.invert_dir_pin   = AXIS_X_INVERT_DIR;
// ... populate yConfig and thetaConfig the same way.

// ---- 2) Construct the Gantry with all three drivers ----
static Gantry::Gantry gantry(xConfig, yConfig, thetaConfig, PIN_GRIPPER);

// ---- 3) Install per-axis drivetrains (mm/deg <-> pulse scaling) ----
PulseMotor::DrivetrainConfig xDt;
xDt.type                  = PulseMotor::DrivetrainType::BELT;
xDt.belt_lead_mm_per_rev  = AXIS_X_BELT_LEAD_MM_PER_REV;
xDt.encoder_ppr           = AXIS_X_ENCODER_PPR;
xDt.motor_reducer_ratio   = AXIS_X_MOTOR_REDUCER_RATIO;
gantry.setXDrivetrain(xDt);

PulseMotor::DrivetrainConfig yDt;
yDt.type                  = PulseMotor::DrivetrainType::BALLSCREW;
yDt.lead_mm               = AXIS_Y_BALLSCREW_LEAD_MM;
yDt.encoder_ppr           = AXIS_Y_ENCODER_PPR;
yDt.motor_reducer_ratio   = AXIS_Y_MOTOR_REDUCER_RATIO;
gantry.setYDrivetrain(yDt);

PulseMotor::DrivetrainConfig thetaDt;
thetaDt.type              = PulseMotor::DrivetrainType::ROTARY_DIRECT;
thetaDt.output_gear_ratio = AXIS_THETA_OUTPUT_GEAR_RATIO;
thetaDt.encoder_ppr       = AXIS_THETA_ENCODER_PPR;
thetaDt.motor_reducer_ratio = AXIS_THETA_MOTOR_REDUCER_RATIO;
gantry.setThetaDrivetrain(thetaDt);

// ---- 4) Limits + safe height ----
gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);
gantry.setJointLimits(AXIS_X_TRAVEL_MIN_MM, AXIS_X_TRAVEL_MAX_MM,
                      AXIS_Y_TRAVEL_MIN_MM, AXIS_Y_TRAVEL_MAX_MM,
                      AXIS_THETA_TRAVEL_MIN_DEG, AXIS_THETA_TRAVEL_MAX_DEG);
gantry.setSafeYHeight(GANTRY_SAFE_Y_HEIGHT_MM);

// ---- 5) Bring up ----
gantry.begin();
gantry.enable();
gantry.home();

// ---- 6) Move ----
Gantry::JointConfig target;
target.x     = 100.0f;
target.y     =  50.0f;
target.theta =  45.0f;
gantry.moveTo(target, 50, 30);          // 50 mm/s, 30 deg/s

// ---- 7) Update task ----
void gantryUpdateTask(void *pv) {
    auto *g = static_cast<Gantry::Gantry *>(pv);
    while (1) {
        g->update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}
```

---

## Architecture

```
Gantry Library
├── Gantry.h/cpp             — Main gantry control class + sequential state machine
├── GantryConfig.h           — Configuration structs (JointConfig, JointLimits, KinematicParameters, ...)
├── GantryKinematics.h/cpp   — Forward/inverse kinematics + validation
├── GantryTrajectory.h/cpp   — Trapezoidal profile helpers
├── GantryEndEffector.h/cpp  — Digital gripper
├── GantryLimitSwitch.h/cpp  — Debounced limit input
└── GantryUtils.h            — Constants + assert macros
```

### Design principles

1. **One driver family** — all three axes run through `PulseMotor::PulseMotorDriver`.
2. **Explicit drivetrain metadata** — mm/deg↔pulse conversion lives in `PulseMotor::DrivetrainConfig`, one per axis.
3. **Gantry owns limit switches** — via `GantryLimitSwitch`. The driver's own `limit_min_pin`/`limit_max_pin` fields are left at `-1`.
4. **FreeRTOS-friendly** — non-blocking `update()` loop, per-driver ramp timer on `esp_timer`.

### Sequential motion state machine

```
IDLE
  ↓ startSequentialMotion()
Y_DESCENDING          ← Y moves down if target < current
  ↓
GRIPPER_ACTUATING     ← grip open/close; timing from GANTRY_GRIPPER_*_TIME_MS
  ↓
Y_RETRACTING          ← Y returns to safeYHeight_mm_ before X is allowed
  ↓
X_MOVING              ← axisX_.moveRelative(...)
  ↓
THETA_MOVING          ← runs in parallel; does not block the others
  ↓
IDLE
```

---

## Documentation

- **[API Reference](docs/API_REFERENCE.md)** — detailed function-by-function documentation.
- **[Architecture Guide](docs/ARCHITECTURE.md)** — internals, state machine, PCNT/LEDC wiring.
- **[Configuration Guide](docs/CONFIGURATION_GUIDE.md)** — tuning and commissioning.
- **[Examples](docs/EXAMPLES.md)** — code snippets for common tasks.
- **[Development Journal](docs/DEVELOPMENT_JOURNAL.md)** — chronological engineering history.

---

## Installation

### PlatformIO

`library.json` lists `PulseMotor` as the only first-party dependency. PlatformIO picks up both libraries from `lib/` automatically when the project is built.

### ESP-IDF component

If adopting as an ESP-IDF component, add a `CMakeLists.txt` next to `src/` that registers the sources in `lib/Gantry/src/` and declares `PulseMotor` as a required component.

### Runtime dependencies

- **PulseMotor** (`lib/PulseMotor`) — pulse-train driver used by all three axes.
- **MCP23S17** (`lib/MCP23S17`) + the application-level `gpio_expander` shim — when driver pins are routed through the MCP expander.
- **FreeRTOS** — included in ESP-IDF / Arduino-ESP32.
- **ESP-IDF driver APIs** — `driver/gpio`, `driver/ledc`, `driver/pulse_cnt`, `esp_timer`.

---

## API Reference

### Main class: `Gantry::Gantry`

```cpp
// Construction
Gantry(const PulseMotor::DriverConfig &xConfig, int gripperPin);
Gantry(const PulseMotor::DriverConfig &xConfig,
       const PulseMotor::DriverConfig &yConfig,
       int gripperPin);
Gantry(const PulseMotor::DriverConfig &xConfig,
       const PulseMotor::DriverConfig &yConfig,
       const PulseMotor::DriverConfig &thetaConfig,
       int gripperPin);

// Initialization
bool begin();
void enable();
void disable();

// Drivetrain + limits
void setLimitPins(int xMinPin, int xMaxPin);
void setXDrivetrain(const PulseMotor::DrivetrainConfig&);
void setYDrivetrain(const PulseMotor::DrivetrainConfig&);
void setThetaDrivetrain(const PulseMotor::DrivetrainConfig&);
void setYAxisLimits(float minMm, float maxMm);
void setThetaLimits(float minDeg, float maxDeg);
void setJointLimits(float xMin, float xMax,
                    float yMin, float yMax,
                    float thetaMin, float thetaMax);
void setSafeYHeight(float safeHeight_mm);
void setEndEffectorPin(int pin, bool activeHigh = true);
void setHomingSpeed(uint32_t speed_pps);

// Motion
void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed_mm_per_s);
GantryError moveTo(const JointConfig& joint,
                   uint32_t speed_mm_per_s = 50,
                   uint32_t speed_deg_per_s = 30,
                   uint32_t acceleration_mm_per_s2 = 0,
                   uint32_t deceleration_mm_per_s2 = 0);
GantryError moveTo(const EndEffectorPose& pose,
                   uint32_t speed_mm_per_s = 50,
                   uint32_t speed_deg_per_s = 30,
                   uint32_t acceleration_mm_per_s2 = 0,
                   uint32_t deceleration_mm_per_s2 = 0);
bool isBusy() const;
void update();                 // call at ~100 Hz

// Homing / calibration
void home();
int  calibrate();
void requestAbort();
bool isAbortRequested() const;

// Gripper
void grip(bool active);

// Alarms
bool isAlarmActive() const;
bool clearAlarm();

// State queries
int     getXEncoder() const;
int     getXEncoderRaw() const;
int32_t getXCommandedPulses() const;
float   getXCommandedMm() const;
float   getXEncoderMm() const;
int     getCurrentY() const;
int     getCurrentTheta() const;

JointConfig     getCurrentJointConfig() const;
JointConfig     getTargetJointConfig() const;
EndEffectorPose getCurrentEndEffectorPose() const;
EndEffectorPose getTargetEndEffectorPose() const;

EndEffectorPose forwardKinematics(const JointConfig&) const;
JointConfig     inverseKinematics(const EndEffectorPose&) const;

// Scaling (derived from installed DrivetrainConfigs)
double xPulsesPerMm() const;
double yPulsesPerMm() const;
double thetaPulsesPerDeg() const;
```

See `docs/API_REFERENCE.md` for full function contracts and examples.

---

## Configuration

### Axis drivetrains

Each axis needs both a `PulseMotor::DriverConfig` (electrical side: pins,
encoder, LEDC) and a `PulseMotor::DrivetrainConfig` (mechanical side: lead,
reducer, encoder PPR). The application fills these from
`include/axis_pulse_motor_params.h` and `include/axis_drivetrain_params.h`
respectively.

```cpp
PulseMotor::DrivetrainConfig xDt;
xDt.type                  = PulseMotor::DrivetrainType::BELT;
xDt.belt_lead_mm_per_rev  = AXIS_X_BELT_LEAD_MM_PER_REV;
xDt.encoder_ppr           = AXIS_X_ENCODER_PPR;
xDt.motor_reducer_ratio   = AXIS_X_MOTOR_REDUCER_RATIO;
gantry.setXDrivetrain(xDt);
```

### Joint limits and safe height

```cpp
gantry.setJointLimits(0.0f, 550.0f, 0.0f, 150.0f, -180.0f, 180.0f);
gantry.setSafeYHeight(GANTRY_SAFE_Y_HEIGHT_MM);
```

### Gripper timing

Open/close times live in `axis_drivetrain_params.h` as
`GANTRY_GRIPPER_OPEN_TIME_MS` and `GANTRY_GRIPPER_CLOSE_TIME_MS` and are
consumed automatically by the sequential state machine.

---

## Thread Safety

`Gantry` is **not** internally thread-safe. Call `update()` and every motion /
status method from a single FreeRTOS task, or provide an external mutex.

Each `PulseMotor::PulseMotorDriver` has its own optional mutex gated by
`PULSE_MOTOR_USE_FREERTOS`. Its ramp-timer ISR (5 ms) is the only context that
touches driver state besides the caller.

Recommended pattern:

```cpp
void gantryTask(void *pv) {
    auto *g = static_cast<Gantry::Gantry *>(pv);
    g->begin();
    g->enable();
    g->home();
    while (1) {
        g->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## Coordinate Systems

### Joint space

- **X** — horizontal position (mm).
- **Y** — vertical position (mm).
- **Theta** — rotation angle (degrees) around Y.

### Workspace (end-effector)

- **X, Y, Z** — position (mm).
- **Theta** — orientation (degrees).

Forward / inverse mapping is via `forwardKinematics()` / `inverseKinematics()`.

---

## Error Handling

```cpp
enum class GantryError {
    OK,
    NOT_INITIALIZED,
    MOTOR_NOT_ENABLED,
    ALREADY_MOVING,
    INVALID_POSITION,
    INVALID_PARAMETER,
    TIMEOUT,
    LIMIT_SWITCH_FAILED,
    CALIBRATION_FAILED,
    CONVERSION_ERROR
};
```

Always check the return value of `moveTo(JointConfig)` / `moveTo(EndEffectorPose)`.

---

## Troubleshooting

| Symptom | First checks |
|---|---|
| `moveTo` returns `NOT_INITIALIZED` | Did `gantry.begin()` succeed? |
| `moveTo` returns `INVALID_POSITION` | Do the target values lie inside `setJointLimits()` bounds? |
| X or Y not moving, no errors | Each axis has a matching `DrivetrainConfig`? `encoder_ppr` and `lead_mm` / `belt_lead_mm_per_rev` nonzero? |
| Alarm keeps firing | Call `gantry.clearAlarm()`; verify the drive's alarm-reset wiring (`alarm_reset_pin`). |
| Sequential motion stuck | Confirm `safeYHeight_mm_` is reachable and the Y drivetrain is installed. |

---

## License

MIT — see `LICENSE`.
