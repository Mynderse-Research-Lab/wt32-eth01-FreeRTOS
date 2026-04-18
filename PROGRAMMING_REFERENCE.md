# Programming Reference — WT32-ETH01 Gantry Controller

**Target:** ESP32 (WT32-ETH01)
**Framework:** ESP-IDF v5.x (Arduino-ESP32 v3 as a component)

This document is the single-source programming reference for the firmware. It covers the build and flash flow, the FreeRTOS entry point and task layout, the public C/C++ APIs of every library, the serial console command surface, and the current known bugs. For *conceptual* overviews ("what does this library do, why does it exist"), see `LIBRARIES_OVERVIEW.md`.

---

## Table of Contents

1. [Build & Flash](#1-build--flash)
2. [Application Entry Point & Task Layout](#2-application-entry-point--task-layout)
3. [Pin Map](#3-pin-map)
4. [Public API: `Gantry` library](#4-public-api-gantry-library)
5. [Public API: `PulseMotor` driver](#5-public-api-pulsemotor-driver)
6. [Public API: `MCP23S17` SPI GPIO expander](#6-public-api-mcp23s17-spi-gpio-expander)
7. [Public API: `gpio_expander` abstraction](#7-public-api-gpio_expander-abstraction)
8. [Serial Console Commands](#8-serial-console-commands)
9. [Application-Level Constants](#9-application-level-constants)
10. [Diagnostic Compile-Time Toggles](#10-diagnostic-compile-time-toggles)
11. [Known Bugs & Gotchas](#11-known-bugs--gotchas)

---

## 1. Build & Flash

### 1.1 Toolchain

| Tool | Version |
|---|---|
| ESP-IDF | 5.x (tested on v5.1.x and v6.0) |
| Target chip | `esp32` |
| Python | bundled with ESP-IDF (do **not** mix with `scoop`/system Python, see §11.2) |
| `arduino-esp32` | `^3.0.0` (via `idf_component.yml`) |
| `CONFIG_FREERTOS_HZ` | must be `1000` |

### 1.2 Project layout that matters at build time

```
idf/                          # ESP-IDF project root (gitignored)
├── CMakeLists.txt            # minimal: project(wt32_eth01_base)
├── sdkconfig                 # generated; committed as sdkconfig.old snapshot
├── sdkconfig.defaults        # only CONFIG_FREERTOS_HZ=1000
├── partitions.csv
└── main/
    ├── CMakeLists.txt        # compiles all sources from ../../src, ../../lib/*/src
    └── idf_component.yml     # arduino-esp32 dependency

src/                          # application sources
├── main.cpp                  # app_main(), MCP init, task creation
├── gantry_test_console.cpp   # interactive serial console (FreeRTOS task)
├── gpio_expander.c           # C shim: MCP vs direct GPIO routing
└── basic_tests.cpp           # runBasicTests() — kinematics self-test

include/                      # project-wide headers
├── gantry_app_constants.h    # pin map + motion defaults
└── gpio_expander.h           # MCP/direct pin API

lib/
├── Gantry/                   # 3-axis gantry controller
├── PulseMotor/               # Generic pulse+direction motor driver (X, Y, Theta)
└── MCP23S17/                 # SPI GPIO expander driver
```

`idf/main/CMakeLists.txt` compiles application + library sources **directly** — there are no separate ESP-IDF custom components under `idf/components/`. If you see `Failed to resolve component 'PulseMotor'` (or similar), the `main/CMakeLists.txt` was regenerated with component dependencies and needs to be reverted to compile-from-lib-tree mode (see §11.3 below).

### 1.3 Build

```powershell
. "$env:USERPROFILE\esp-idf\export.ps1"
cd E:\Projects\wt32-eth01-base\idf
idf.py set-target esp32          # first time only
idf.py build
```

Output firmware: `idf/build/wt32_eth01_base.bin`.

### 1.4 Flash & monitor

```powershell
idf.py -p COM3 flash monitor     # replace COM3 with your port
```

Exit monitor: `Ctrl+]`.

If upload fails: hold **BOOT**, tap **EN/RESET**, release **BOOT** as "Connecting…" appears. Retry at lower baud with `-b 115200`.

### 1.5 Clean build

```powershell
idf.py fullclean
idf.py build
```

Use `fullclean` after changing `sdkconfig.defaults` or after any ESP-IDF or Arduino-ESP32 version change.

---

## 2. Application Entry Point & Task Layout

### 2.1 `app_main()`

Defined in `src/main.cpp`. Startup order:

1. **`initMcp23s17()`** — brings up SPI2 bus, creates the MCP23S17 handle (mutex included), sets every MCP pin's direction / pull-up / initial level. All control/limit/alarm pins flow through this.
2. **`gpio_config` for `PIN_Y_PULSE`** — direct ESP32 GPIO (LEDC will take over later).
3. **Build three `(PulseMotor::DriverConfig, PulseMotor::DrivetrainConfig)` pairs** via the per-axis `makeXDriverConfig()` / `makeXDrivetrainConfig()` etc. helpers. Each pair consumes the per-axis macros from `include/axis_pulse_motor_params.h` and `include/axis_drivetrain_params.h`.
4. **Construct `Gantry::Gantry gantry(xDrv, xDt, yDrv, yDt, tDrv, tDt, PIN_GRIPPER)`** as a `static`.
5. **`gantry.setLimitPins()`, `setJointLimits()`, `setYAxisLimits()`, `setThetaLimits()`, `setSafeYHeight()`** — lightweight configuration that the driver/drivetrain configs did not already cover.
6. **`gantry.begin()`** — initialises X, Y, and Theta PulseMotor drivers plus the end-effector (MCP pin).
7. **`gantry.enable()`** — sets SON on all three axis drivers.
8. **Create FreeRTOS tasks.**

### 2.2 FreeRTOS tasks

| Task | Core | Priority | Stack | Purpose |
|---|---|---|---|---|
| `GantryUpdate` | 1 | 5 | 4096 B | Calls `gantry.update()` at 100 Hz. Drives stepper pulses, limit debounce, sequential motion state machine. |
| `SerialCmd` | 0 | 1 | 4096 B | `gantry_test_console` — line-buffered stdin reader, command dispatcher, LIVE POS telemetry. |

Task stack/priority/core constants live in `include/gantry_app_constants.h`:

```c
#define GANTRY_UPDATE_TASK_STACK    4096
#define GANTRY_UPDATE_TASK_PRIORITY 5
#define GANTRY_UPDATE_TASK_CORE     1
#define CONSOLE_TASK_STACK          4096
#define CONSOLE_TASK_PRIORITY       1
#define CONSOLE_TASK_CORE           0
```

The `app_main` task calls `vTaskDelete(NULL)` after task creation — the two FreeRTOS tasks own the runtime.

### 2.3 Thread safety notes

- `lib/MCP23S17/src/MCP23S17.cpp` holds a **FreeRTOS mutex per device**; all `read_register`/`write_register` paths take it. Concurrent access from `GantryUpdate` (reading limit/alarm pins) and `SerialCmd` (register dumps, `grip`) is safe.
- `lib/PulseMotor/src/PulseMotor.h` has optional `PULSE_MOTOR_USE_FREERTOS` (default `1`) which adds a `SemaphoreHandle_t mutex_` around state. The legacy `SDF08NK8X_USE_FREERTOS` macro is honoured as an alias for source compatibility with older build flags.
- The motion profile is driven by an `esp_timer` callback (`rampTimerCallback`) running at 5 ms intervals. That ISR must remain short and must not block on heap or non-ISR-safe FreeRTOS APIs (see header comment in `PulseMotor.h`).

---

## 3. Pin Map

Source of truth: `include/gantry_app_constants.h`. `pinout.csv` mirrors the same values as a spreadsheet-friendly artifact.

### 3.1 Direct ESP32 GPIO (WT32-ETH01)

| Signal | GPIO | Peripheral | Notes |
|---|---|---|---|
| `MCP23S17_SPI_CS_PIN` | 15 | SPI2 CS | Strap pin; safe at reset. |
| `MCP23S17_SPI_MISO_PIN` | 35 | SPI2 MISO | Input-only. |
| `MCP23S17_SPI_MOSI_PIN` | 12 | SPI2 MOSI | Strap pin — must be LOW at reset. |
| `MCP23S17_SPI_SCLK_PIN` | 5 | SPI2 SCK | |
| `PIN_X_PULSE` | 14 | LEDC ch0 | X-axis pulse-train output (belt-drive linear axis). |
| `PIN_Y_PULSE` | 2 | LEDC ch1 | Y-axis pulse-train output (ballscrew linear axis). Strap pin — must be LOW/floating at reset. |
| `PIN_THETA_PULSE` (alias `PIN_THETA_PWM`) | 0 | LEDC ch2 | Theta pulse-train output (rotary-direct axis). Strap pin — must be HIGH at reset. |
| `PIN_X_ENC_A` | 4 | PCNT unit 0 | Encoder A+. General-purpose header pin. |
| `PIN_X_ENC_B` | 36 | PCNT unit 0 | Encoder B+. Input-only. |
| `PIN_Y_ENC_A` | 39 | PCNT unit 1 | Encoder A+. Input-only. |
| `PIN_Y_ENC_B` | 32 | PCNT unit 1 | Encoder B+. |

MCP SPI bus runs at **1 MHz** (`MCP23S17_SPI_CLOCK_HZ_WORKING`).

### 3.2 MCP23S17 pin map

| Signal | MCP pin | Port/bit | Direction | Notes |
|---|---|---|---|---|
| `PIN_X_DIR` | 0 | A.0 | Out | Servo SIGN/DIR |
| `PIN_X_ENABLE` | 1 | A.1 | Out | Servo SON |
| `PIN_X_LIMIT_MIN` | 2 | A.2 | In pull-up | Active-low |
| `PIN_X_LIMIT_MAX` | 3 | A.3 | In pull-up | Active-low |
| `PIN_X_ALARM_STATUS` | 4 | A.4 | In pull-up | Active-low alarm |
| `PIN_X_ALARM_RESET` | 5 | A.5 | Out | ARST pulse |
| *(available)* | 6 | A.6 | — | `PA6_AVAILABLE` |
| `PIN_GRIPPER` | 7 | A.7 | Out | Digital end-effector |
| `PIN_Y_DIR` | 8 | B.0 | Out | |
| `PIN_Y_ENABLE` | 9 | B.1 | Out | |
| `PIN_Y_LIMIT_MIN` | 10 | B.2 | In pull-up | Active-low |
| `PIN_Y_LIMIT_MAX` | 11 | B.3 | In pull-up | Active-low |
| `PIN_Y_ALARM_STATUS` | 12 | B.4 | In pull-up | Active-low |
| `PIN_Y_ALARM_RESET` | 13 | B.5 | Out | |
| `PIN_THETA_DIR` | 14 | B.6 | Out | Theta direction line. Previously `PIN_THETA_LIMIT_MIN` in the PWM-hobby-servo build. |
| `PIN_THETA_ENABLE` | 15 | B.7 | Out | Theta enable / SON. Previously `PIN_THETA_LIMIT_MAX`. |

### 3.3 Pin-identifier encoding

A single `int` pin field can carry two kinds of pins:

| Range | Meaning |
|---|---|
| `0..15` | MCP logical pin. |
| `>= GPIO_DIRECT_PIN_BASE (0x10)` | Direct ESP32 GPIO number, e.g. `14` → GPIO14 on WT32. |
| `GPIO_EXPANDER_DIRECT_PIN(n)` → `0x100 \| n` | Explicitly-flagged direct GPIO, used when the raw number would overlap MCP pin space (e.g. the Y-pulse at GPIO2 needs the flag so it isn't interpreted as MCP pin 2). |
| `-1` | Not routed / disabled. |

Helpers in `include/gpio_expander.h`:

```c
#define GPIO_DIRECT_PIN_BASE        0x10
#define GPIO_EXPANDER_DIRECT_FLAG   0x100
#define GPIO_EXPANDER_DIRECT_MASK   0x0FF
#define GPIO_EXPANDER_DIRECT_PIN(gpio_num) \
  (GPIO_EXPANDER_DIRECT_FLAG | ((int)(gpio_num) & GPIO_EXPANDER_DIRECT_MASK))
```

Consumers (e.g. `GantryEndEffector`) branch on `isMcpLogicalPin(pin)` vs `isEncodedDirectPin(pin)` to pick the right API.

---

## 4. Public API: `Gantry` library

Namespace: `Gantry`. All types below live in it unless noted.

### 4.1 Core types

```cpp
enum class GantryError {
    OK, NOT_INITIALIZED, MOTOR_NOT_ENABLED, ALREADY_MOVING,
    INVALID_POSITION, INVALID_PARAMETER, TIMEOUT,
    LIMIT_SWITCH_FAILED, CALIBRATION_FAILED, CONVERSION_ERROR
};

enum class HomingStatus { IDLE, IN_PROGRESS, COMPLETE, FAILED };

struct JointConfig {        // joint space (mm, mm, deg)
    float x, y, theta;
    JointConfig operator+(const JointConfig&) const;
    JointConfig operator-(const JointConfig&) const;
    JointConfig operator*(float) const;
};

struct JointLimits {
    float x_min, x_max, y_min, y_max, theta_min, theta_max;
    bool isValid(const JointConfig&) const;
};

struct EndEffectorPose {    // workspace coordinates (mm, mm, mm, deg)
    float x, y, z, theta;
};

struct KinematicParameters {
    float y_axis_z_offset_mm;           // default 80
    float theta_x_offset_mm;            // default -55
    float gripper_y_offset_mm;          // default 385
    float gripper_z_offset_mm;          // default 80
    float x_axis_ball_screw_pitch_mm;   // default 40
};

struct GantryStatus { /* position, targets, flags, timestamps */ };

struct Waypoint {
    EndEffectorPose pose;
    uint32_t speed_mm_per_s, speed_deg_per_s;
    uint32_t acceleration_mm_per_s2, deceleration_mm_per_s2;
};

template<size_t MAX_WAYPOINTS = 16>
class WaypointQueue { /* push/pop/size/empty/full/clear */ };
```

### 4.2 `Gantry::Gantry`

Construction:

```cpp
Gantry(const PulseMotor::DriverConfig&     xDrv,
       const PulseMotor::DrivetrainConfig& xDt,
       const PulseMotor::DriverConfig&     yDrv,
       const PulseMotor::DrivetrainConfig& yDt,
       const PulseMotor::DriverConfig&     tDrv,
       const PulseMotor::DrivetrainConfig& tDt,
       int gripperPin);
```

The constructor factories a concrete `GantryPulseMotorLinearAxis` for X and Y (the `xDt.type` / `yDt.type` must be a linear drivetrain — `DRIVETRAIN_BALLSCREW`, `DRIVETRAIN_BELT`, or `DRIVETRAIN_RACKPINION`) and a `GantryPulseMotorRotaryAxis` for Theta (`tDt.type` must be `DRIVETRAIN_ROTARY_DIRECT`). If a type/axis pair is mis-configured the corresponding `unique_ptr` is null and `begin()` returns `false` for X (the required axis) or logs a warning for Y/Theta.

#### Configuration (call before `begin()`)

| Method | Purpose |
|---|---|
| `void setLimitPins(int xMin, int xMax)` | X-axis limit switch pins (MCP or direct). |
| `void setYAxisLimits(float minMm, float maxMm)` | Y travel envelope. |
| `void setThetaLimits(float minDeg, float maxDeg)` | Theta angular limits (also propagates to `GantryPulseMotorRotaryAxis`). |
| `void setJointLimits(float xMin, float xMax, float yMin, float yMax, float thetaMin, float thetaMax)` | Joint validation envelope used by `moveTo(JointConfig)`. |
| `void setEndEffectorPin(int pin, bool activeHigh = true)` | Override constructor gripper pin. |
| `void setSafeYHeight(float mm)` | Safe retraction height used before X travel. Default 150 mm. |
| `void setHomingSpeed(uint32_t pps)` | Homing speed (pulses/s); mutates the X driver's `DriverConfig.homing_speed_pps`. |
| `void setStepsPerRevolution(float steps)` | Legacy shim used by diagnostics. Per-axis scaling lives in `DrivetrainConfig`. |

Per-axis mm<->pulse scaling, motion caps, and inversion flags come from the two parameter headers (`include/axis_pulse_motor_params.h` and `include/axis_drivetrain_params.h`) and are baked into the `DriverConfig` / `DrivetrainConfig` passed to the constructor. There is no separate runtime setter for them.

#### Lifecycle

```cpp
bool begin();          // initialize all axes; returns false on any failure
void enable();         // enable X servo + Y stepper
void disable();        // disable both (motors freewheel)
bool isEnabled() const;
void update();         // call from GantryUpdate task @100 Hz
```

#### Motion

```cpp
void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed_pps);
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

void home();             // X-axis to MIN limit
int  calibrate();        // measure axis length (mm), or 0 on failure

void requestAbort();     // force-stop any motion / homing / calibration
bool isAbortRequested() const;

bool isBusy() const;
bool isAlarmActive() const;
bool clearAlarm();       // pulse ARST on configured drives

void grip(bool active);
```

#### State queries

```cpp
int   getXEncoder() const;          // encoder counts (filtered)
int   getXEncoderRaw() const;       // raw hardware counter
int32_t getXCommandedPulses() const;
float getXCommandedMm() const;
float getXEncoderMm() const;
int   getCurrentY() const;          // mm (integer)
int   getCurrentTheta() const;      // degrees (integer)

JointConfig       getCurrentJointConfig() const;
JointConfig       getTargetJointConfig() const;
EndEffectorPose   getCurrentEndEffectorPose() const;
EndEffectorPose   getTargetEndEffectorPose() const;

EndEffectorPose   forwardKinematics(const JointConfig&) const;
JointConfig       inverseKinematics(const EndEffectorPose&) const;

float getStepsPerRevolution() const;
float getPulsesPerMm() const;
```

### 4.3 `Gantry::Kinematics` (static helpers)

```cpp
static EndEffectorPose forward(const JointConfig&, const KinematicParameters&);
static JointConfig     inverse(const EndEffectorPose&, const KinematicParameters&);
static bool            validate(const JointConfig&, const JointLimits&);
```

### 4.4 `Gantry::TrajectoryPlanner` (static helpers)

```cpp
struct TrapezoidalProfile {
    float t_accel, t_cruise, t_decel, total_time, max_speed;
    bool valid;
};

static TrapezoidalProfile calculateProfile(float start, float target,
                                           float max_speed,
                                           float acceleration,
                                           float deceleration);
static float interpolate(const TrapezoidalProfile&,
                         float start, float target, float elapsed_s);
```

### 4.5 Axis interfaces and implementations (normally used through `Gantry`)

- `Gantry::GantryLinearAxis` — abstract interface for any linear (mm-domain) axis. Methods: `begin`, `enable`/`disable`/`isEnabled`, `moveToMm` / `moveRelativeMm` / `stopMotion`, `getCurrentMm` / `getTargetMm` / `isBusy`, and pulse-domain accessors `moveToPulses` / `getCurrentPulses` / `getEncoderPulses` / `setCurrentPulses` used by the homing/calibration paths.
- `Gantry::GantryRotaryAxis` — abstract interface for any rotary (deg-domain) axis. Methods: `begin`, `enable`/`disable`/`isEnabled`, `moveToDeg`, `stopMotion`, `getCurrentDeg`, `setAngleRange`.
- `Gantry::GantryPulseMotorLinearAxis` — concrete linear-axis implementation backed by `PulseMotor::PulseMotorDriver` + a `DrivetrainConfig`. Handles mm↔pulse conversion for ballscrew / belt / rack-pinion drivetrains. Exposes `driver()` for `Gantry`-internal access (e.g. `setHomingSpeed`).
- `Gantry::GantryPulseMotorRotaryAxis` — concrete rotary-axis implementation on top of `PulseMotor::PulseMotorDriver`. Handles deg↔pulse conversion for `DRIVETRAIN_ROTARY_DIRECT`.
- `Gantry::GantryEndEffector` — digital gripper. `configurePin`, `begin`, `setActive`, `isActive`, `getPin`. **Correctly routes pins `<16` through `gpio_expander_*`, pins `>=16` (or direct-flagged) through Arduino `pinMode`/`digitalWrite`.**
- `Gantry::GantryLimitSwitch` — debounced input. `configure(pin, activeLow=true, enablePullup=true, debounceCycles=6)`, `begin`, `update(force=false)`, `isActive`.

### 4.6 Constants (`Gantry::Constants`)

```cpp
constexpr float    DEFAULT_STEPS_PER_REV          = 6000.0f;
constexpr float    DEFAULT_PULSES_PER_MM          = 150.0f;
constexpr float    DEFAULT_SAFE_Y_HEIGHT_MM       = 150.0f;
constexpr uint32_t DEFAULT_HOMING_SPEED_PPS       = 6000;
constexpr uint32_t DEFAULT_SPEED_MM_PER_S         = 50;
constexpr uint32_t DEFAULT_SPEED_DEG_PER_S        = 30;
constexpr uint32_t GRIPPER_ACTUATE_TIME_MS        = 100;  // legacy default; SCHUNK KGG 100-80 uses GANTRY_GRIPPER_OPEN_TIME_MS / GANTRY_GRIPPER_CLOSE_TIME_MS from axis_drivetrain_params.h
constexpr uint32_t CALIBRATION_TIMEOUT_MS         = 30000;
constexpr uint32_t TRAVEL_MEASUREMENT_TIMEOUT_MS  = 90000;
```

---

## 5. Public API: `PulseMotor` driver

Namespace: `PulseMotor`. Header: `lib/PulseMotor/src/PulseMotor.h`.

### 5.1 Types

```cpp
enum class PulseMode : uint8_t {
    PULSE_DIRECTION = 0,  // standard 2-wire pulse+direction
    CW_CCW          = 1,  // dual-pulse (not yet supported)
    QUADRATURE      = 2   // A/B quadrature (not yet supported)
};

enum DrivetrainType : uint8_t {
    DRIVETRAIN_BALLSCREW     = 1,
    DRIVETRAIN_BELT          = 2,
    DRIVETRAIN_RACKPINION    = 3,
    DRIVETRAIN_ROTARY_DIRECT = 4
};

struct DriverConfig {
    // ---- Named pin fields (replace the SDF08 slot arrays) ----
    int pulse_pin, dir_pin, enable_pin;
    int alarm_reset_pin, brake_pin, inhibit_pin;
    int in_position_pin, brake_status_pin, alarm_pin;
    int encoder_a_pin, encoder_b_pin, encoder_z_pin;
    int limit_min_pin, limit_max_pin;   // typically -1 when the Gantry layer owns limits

    PulseMode pulse_mode;               // default PULSE_DIRECTION
    uint32_t  max_pulse_freq;           // Hz
    uint32_t  encoder_ppr;
    double    gear_numerator, gear_denominator;

    int     ledc_channel;
    uint8_t ledc_resolution;            // forced to 1-bit at runtime
    int     pcnt_unit;

    bool     enable_encoder_feedback;
    bool     enable_closed_loop_control;
    bool     invert_output_logic;
    bool     invert_dir_pin;
    bool     home_on_boot;

    uint32_t homing_speed_pps;
    uint8_t  limit_debounce_cycles;
    uint16_t limit_sample_interval_ms;
    bool     limit_log_changes;
};

struct DrivetrainConfig {
    DrivetrainType type;
    // Ballscrew
    float    ballscrew_lead_mm;
    uint32_t ballscrew_critical_rpm;
    // Belt
    float    belt_lead_mm_per_rev;      // primary
    uint32_t belt_pulley_teeth;
    float    belt_pitch_mm;
    // Rack and pinion
    float    pinion_pitch_diameter_mm;
    // Rotary direct
    float    output_gear_ratio;
    // Common
    uint32_t encoder_ppr;
    float    motor_reducer_ratio;
};

double pulsesPerMm(const DrivetrainConfig&);  // header-inline helper
double pulsesPerDeg(const DrivetrainConfig&); // header-inline helper

struct DriveStatus {
    bool     servo_enabled, position_reached, brake_released, alarm_active;
    uint32_t current_position;
    int32_t  encoder_position, position_error;
    uint32_t current_speed, last_update_ms;
};

using AlarmCallback           = void (*)(const char* alarm_code);
using PositionReachedCallback = void (*)(uint32_t position);
using StatusUpdateCallback    = void (*)(const DriveStatus&);
```

### 5.2 `PulseMotor::PulseMotorDriver`

```cpp
explicit PulseMotorDriver(const DriverConfig& config);
~PulseMotorDriver();
bool initialize();

bool enable();
bool disable();
bool isEnabled() const;
bool isMotionActive() const;

bool moveToPosition(uint32_t target, uint32_t max_speed = 10000,
                    uint32_t accel = 5000, uint32_t decel = 5000);
bool moveRelative (int64_t  delta,  uint32_t max_speed = 10000,
                    uint32_t accel = 5000, uint32_t decel = 5000);
bool stopMotion   (uint32_t decel = 50000);
bool eStop();

// Driver-side homing / travel measurement are stubbed out by design: the
// Gantry layer owns that responsibility so limit-switch ownership lives in
// one place.

DriveStatus getStatus() const;
uint32_t    getPosition() const;
void        setPosition(uint32_t position);
uint32_t    getSpeed() const;
bool        isAlarmActive() const;
bool        clearAlarm();

int32_t getEncoderPosition() const;
void    resetEncoderPosition();

void setAlarmCallback          (AlarmCallback);
void setPositionReachedCallback(PositionReachedCallback);
void setStatusUpdateCallback   (StatusUpdateCallback);

const DriverConfig& getConfig() const;
void                setConfig(const DriverConfig&);

static String getVersion();
```

Key implementation details:

- Pulse output via **LEDC** (channel from `DriverConfig::ledc_channel`, runtime-forced to 1-bit resolution to keep `div_param` within range at low frequencies).
- Encoder via **`pulse_cnt`** (ESP-IDF v5+ API), with a 64-bit `encoder_accumulator_` on top of the 16-bit hardware counter.
- `esp_timer_handle_t ramp_timer_` fires `rampTimerCallback` every 5 ms to update the trapezoidal profile.
- All public mutating methods take `mutex_` when `PULSE_MOTOR_USE_FREERTOS=1` (the default). The legacy `SDF08NK8X_USE_FREERTOS` macro is honoured as an alias for build-flag compatibility.

---

## 6. Public API: `MCP23S17` SPI GPIO expander

Header: `lib/MCP23S17/src/MCP23S17.h`. C API.

### 6.1 Types

```c
typedef enum { MCP23S17_PORT_A = 0, MCP23S17_PORT_B = 1 } mcp23s17_port_t;
typedef uint8_t mcp23s17_pin_t;                 // 0..15
typedef struct mcp23s17_handle* mcp23s17_handle_t;

typedef struct mcp23s17_config_t {
    spi_host_device_t spi_host;
    gpio_num_t        cs_pin, miso_pin, mosi_pin, sclk_pin;
    uint8_t           device_address;      // 0x00..0x07 (addr pins)
    uint32_t          clock_speed_hz;      // e.g. 1000000
} mcp23s17_config_t;
```

### 6.2 Functions

```c
mcp23s17_handle_t mcp23s17_init  (const mcp23s17_config_t*);
void              mcp23s17_deinit(mcp23s17_handle_t);

esp_err_t mcp23s17_set_pin_direction(mcp23s17_handle_t, mcp23s17_pin_t, bool is_output);
esp_err_t mcp23s17_set_pin_pullup   (mcp23s17_handle_t, mcp23s17_pin_t, bool enable);
esp_err_t mcp23s17_write_pin        (mcp23s17_handle_t, mcp23s17_pin_t, uint8_t level);
uint8_t   mcp23s17_read_pin         (mcp23s17_handle_t, mcp23s17_pin_t);

esp_err_t mcp23s17_write_port(mcp23s17_handle_t, mcp23s17_port_t, uint8_t value);
uint8_t   mcp23s17_read_port (mcp23s17_handle_t, mcp23s17_port_t);

esp_err_t mcp23s17_set_pin_interrupt(mcp23s17_handle_t, mcp23s17_pin_t,
                                     bool enable, bool trigger_on_rising);

/* Debug helpers — raw register access, also mutex-protected. */
esp_err_t mcp23s17_debug_read_register (mcp23s17_handle_t, uint8_t reg, uint8_t* value);
esp_err_t mcp23s17_debug_write_register(mcp23s17_handle_t, uint8_t reg, uint8_t  value);
```

Every call above takes the handle's internal `SemaphoreHandle_t spi_mutex` around `spi_device_transmit`. Safe to call from multiple FreeRTOS tasks.

---

## 7. Public API: `gpio_expander` abstraction

Header: `include/gpio_expander.h`. Thin C shim over `MCP23S17` with one global handle.

```c
bool      gpio_expander_init          (const mcp23s17_config_t*);
void      gpio_expander_deinit        (void);

esp_err_t gpio_expander_set_direction (int pin, bool is_output);
esp_err_t gpio_expander_set_pullup    (int pin, bool enable);
esp_err_t gpio_expander_write         (int pin, uint8_t level);
uint8_t   gpio_expander_read          (int pin);

mcp23s17_handle_t gpio_expander_get_mcp_handle(void);
```

Behavior of `pin` argument:

| `pin` | Routing |
|---|---|
| `0..15` | MCP23S17 pin. |
| anything else | Returns `ESP_ERR_INVALID_ARG` — this layer is **MCP-only**. Direct GPIO must be driven via `driver/gpio.h` / `pinMode`/`digitalWrite`. |

`GantryEndEffector` and similar consumers decode the `GPIO_EXPANDER_DIRECT_PIN(...)` encoded form themselves before choosing which API to call.

---

## 8. Serial Console Commands

Entry: `gantryTestConsoleTask`, created in `src/main.cpp` with `GantryTestConsoleConfig` (defined in `src/gantry_test_console.h`).

The console reads from stdin (USB serial via `idf.py monitor` or equivalent) with `;`, `\r`, or `\n` as line delimiters.

### 8.1 Always-available commands

| Command | Description |
|---|---|
| `help` / `?` | Print command list. |
| `status` | Dump `Gantry Status` (positions, busy flag, enabled, alarm, calibrated length, workspace offset). |
| `limits` | Read current X MIN / MAX limit switches. |
| `pins` | Print active pin configuration (direct vs MCP, LEDC channels, PCNT units). |
| `gpio_drive g v` | Drive a **direct** ESP32 GPIO (by GPIO number) to `0` or `1`. |
| `enable` | `gantry.enable()`. |
| `disable` | `gantry.disable()`. |
| `home` | Run X-axis homing (blocks on hardware; run async via task internally). |
| `calibrate` | Measure X-axis length and set X max. |
| `units <mm\|in>` | Select unit of subsequent `speed`, `accel`, `move` inputs and printouts. |
| `speed <v> [deg/s]` | Set move speed in selected linear units/s (optional Theta deg/s). |
| `accel <a> [d]` | Set accel (and optional decel) in selected linear units/s². Zero is rejected. |
| `rangelimit <0\|1>` | Enable/disable speed+accel range clamps. |
| `move <x> <y> <t>` | Move to absolute `(x, y, theta)`. Requires `home` **and** `calibrate` to have run this session. See §11.1 — move now drives the X axis correctly; encoder feedback is a separate hardware concern. |
| `grip <0\|1>` | `gantry.grip()`. 1 = closed, 0 = open. |
| `stop` | `requestAbort()` + `disable()`. |
| `alarmreset` / `arst` | `gantry.clearAlarm()` — pulses ARST on any configured drive. |
| `selftest` | Run `runBasicTests()` — kinematics/trajectory math self-check. |

### 8.2 MCP diagnostic commands (compiled in when `MCP_DEBUG_CMDS=1`, on by default)

| Command | Description |
|---|---|
| `mcp_pin_mode <p> <m>` | Force MCP pin `p` (0..15) to mode `inpu`/`in` (input), `out0` (output low), or `out1` (output high). |
| `mcp_dump <a\|b>` | Dump IOCON, direction, pull-up, OLAT, and GPIO registers for the selected port. |
| `mcp_reg <r\|w> <reg> [value]` | Raw register read/write. `reg`/`value` accept hex (`0x12`) or decimal. |

### 8.3 Passive output

- `LIVE POS: x_cmd=... x_enc=... y=... theta=...` — logged every 100 ms while motion is active, every 1 s while idle.
- `CTRL FLIP: <name> <old>-><new>` — printed when any monitored control pin (alarm, limit, enable) changes state.

---

## 9. Application-Level Constants

Application configuration is split across three headers, each with a single responsibility. All changes land by editing one of these files and rebuilding; there are no runtime setters for these values.

### 9.1 `include/gantry_app_constants.h` — pin map + peripheral allocation + task parameters

Board-specific wiring only. No mechanical or electrical tuning.

```c
// MCP23S17 SPI bus
#define MCP23S17_SPI_CS_PIN               15
#define MCP23S17_SPI_MISO_PIN             35
#define MCP23S17_SPI_MOSI_PIN             12
#define MCP23S17_SPI_SCLK_PIN              5
#define MCP23S17_SPI_CLOCK_HZ_WORKING  1000000

// MCP logical pin assignments (Port A = 0..7, Port B = 8..15)
// X axis: PIN_X_DIR, PIN_X_ENABLE, PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX,
//         PIN_X_ALARM_STATUS, PIN_X_ALARM_RESET, PIN_GRIPPER
// Y axis: PIN_Y_DIR, PIN_Y_ENABLE, PIN_Y_LIMIT_MIN, PIN_Y_LIMIT_MAX,
//         PIN_Y_ALARM_STATUS, PIN_Y_ALARM_RESET
// Theta:  PIN_THETA_DIR (14), PIN_THETA_ENABLE (15)

// Direct ESP32 GPIO (LEDC / PCNT)
#define PIN_X_PULSE          14
#define PIN_Y_PULSE           2
#define PIN_THETA_PULSE       0       // legacy alias PIN_THETA_PWM
#define PIN_X_ENC_A           4
#define PIN_X_ENC_B          36
#define PIN_Y_ENC_A          39
#define PIN_Y_ENC_B          32
#define PIN_THETA_ENC_A      -1       // not wired in this hardware revision
#define PIN_THETA_ENC_B      -1

// Peripheral channel allocation
#define X_PULSE_LEDC_CHANNEL        0
#define Y_PULSE_LEDC_CHANNEL        1
#define THETA_PULSE_LEDC_CHANNEL    2     // legacy alias THETA_PWM_LEDC_CHANNEL
#define X_ENCODER_PCNT_UNIT         0
#define Y_ENCODER_PCNT_UNIT         1
#define THETA_ENCODER_PCNT_UNIT     2

// MCP defaults
#define MCP23S17_DEVICE_ADDRESS       0x00
#define MCP23S17_CLOCK_HZ         10000000
#define MCP_DEBUG_CMDS                   1

// FreeRTOS tasks
#define GANTRY_UPDATE_TASK_STACK    4096
#define GANTRY_UPDATE_TASK_PRIORITY    5
#define GANTRY_UPDATE_TASK_CORE        1
#define CONSOLE_TASK_STACK          4096
#define CONSOLE_TASK_PRIORITY          1
#define CONSOLE_TASK_CORE              0
```

### 9.2 `include/axis_pulse_motor_params.h` — per-axis electrical tuning

Motor + driver + optional gearbox parameters. One block per axis (X / Y / Theta):

```c
#define AXIS_X_ENCODER_PPR                 10000u
#define AXIS_X_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_X_GEAR_NUMERATOR                 1.0
#define AXIS_X_GEAR_DENOMINATOR               1.0
#define AXIS_X_MOTOR_REDUCER_RATIO            1.0f
#define AXIS_X_INVERT_DIR                        0
#define AXIS_X_INVERT_OUTPUT_LOGIC               1
#define AXIS_X_LEDC_RESOLUTION_BITS              2u
#define AXIS_X_HOMING_SPEED_PPS              8000u
#define AXIS_X_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_X_LIMIT_SAMPLE_INTERVAL_MS         3u
// (analogous AXIS_Y_* and AXIS_THETA_* blocks follow)
```

### 9.3 `include/axis_drivetrain_params.h` — per-axis mechanical tuning

Drivetrain topology + hard-limit envelope + motion caps + position tolerance + gripper timing + kinematic offsets. Each axis picks a `DrivetrainType` independently. Macros here are drivetrain-AGNOSTIC: for any linear axis the kinematic constant is `AXIS_*_LEAD_MM_PER_REV` regardless of whether the topology is belt, ballscrew, or rack-pinion. `AXIS_*_HARD_LIMIT_MIN/MAX_*` is the mechanical envelope the axis must never exceed (for Theta — whose hardware is unlimited — this is the firmware cable-clamp and acts as the effective hard envelope). **Soft limits are not defined here**; they are derived on boot from the homing / calibration sweep (see `Gantry::calibrate()` and the console homing task) and may be tighter than the hard envelope.

```c
// X: SCHUNK Beta 100-ZRS belt actuator
#define AXIS_X_DRIVETRAIN             DT_BELT
#define AXIS_X_LEAD_MM_PER_REV        200.0f      // datasheet "stroke per round"
#define AXIS_X_HARD_LIMIT_MAX_MM      550.0f      // mechanical stroke
#define AXIS_X_POSITION_TOLERANCE_MM    0.08f     // datasheet repeatability

// Y: SCHUNK Beta 80-SRS ballscrew actuator
#define AXIS_Y_DRIVETRAIN             DT_BALLSCREW
#define AXIS_Y_LEAD_MM_PER_REV         20.0f      // screw pitch
#define AXIS_Y_CRITICAL_RPM            3000u      // whip-speed ceiling
#define AXIS_Y_HARD_LIMIT_MAX_MM      150.0f
#define AXIS_Y_POSITION_TOLERANCE_MM    0.03f

// Theta: SCHUNK ERD 04-40-D-H-N rotary module
#define AXIS_THETA_DRIVETRAIN             DT_ROTARY_DIRECT
#define AXIS_THETA_OUTPUT_GEAR_RATIO        1.0f
#define AXIS_THETA_HARD_LIMIT_MIN_DEG    -180.0f  // firmware cable-clamp
#define AXIS_THETA_HARD_LIMIT_MAX_DEG     180.0f
#define AXIS_THETA_MAX_SPEED_DEG_PER_S   3600.0f  // 600 rpm cap
#define AXIS_THETA_POSITION_TOLERANCE_DEG   0.01f

// Gantry-level
#define GANTRY_SAFE_Y_HEIGHT_MM         150.0f
#define GANTRY_GRIPPER_OPEN_TIME_MS     190u      // SCHUNK KGG 100-80
#define GANTRY_GRIPPER_CLOSE_TIME_MS    150u
// Kinematic offsets: GANTRY_Y_AXIS_Z_OFFSET_MM, GANTRY_THETA_X_OFFSET_MM,
//                    GANTRY_GRIPPER_Y_OFFSET_MM, GANTRY_GRIPPER_Z_OFFSET_MM

// Derived helpers (macro form, for preprocessor-only contexts):
//   AXIS_X_PULSES_PER_MM, AXIS_Y_PULSES_PER_MM, AXIS_THETA_PULSES_PER_DEG
//   AXIS_Y_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S
```

Runtime code generally prefers `PulseMotor::pulsesPerMm(dt)` / `PulseMotor::pulsesPerDeg(dt)` applied to a `DrivetrainConfig` rather than the macros — the macros are mainly for diagnostics and unit tests.

`axis_drivetrain_params.h` also reserves a commented-out block for Trap Move values (reflected inertia, required torque, cycle-time target) that will be populated once the SCHUNK Trap Move PDFs are obtained; see `driver_datasheets_and_calculations/INDEX.md`.

#### 9.3.1 Geometry freeze gate

The kinematic offsets in `axis_drivetrain_params.h` — `GANTRY_Y_AXIS_Z_OFFSET_MM`, `GANTRY_THETA_X_OFFSET_MM`, `GANTRY_GRIPPER_Y_OFFSET_MM`, `GANTRY_GRIPPER_Z_OFFSET_MM`, `GANTRY_SAFE_Y_HEIGHT_MM` — are **development-rig placeholders**. The header emits a one-shot compile-time `#warning` from `src/main.cpp` as a reminder to update them against the frozen production design before deployment.

Deployment attestation procedure:

1. Measure the five offsets against the frozen CAD drawings, or on the as-built assembly with a calibration fixture / CMM.
2. Overwrite the five macros in `axis_drivetrain_params.h` with the measured values (do not set them at runtime; this header is consumed at compile time and also feeds `AXIS_*_PULSES_PER_MM` / `AXIS_Y_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S`).
3. Define `GANTRY_GEOMETRY_FROZEN` as a compile definition to silence the reminder and attest that the geometry has been frozen. Typical wiring in an ESP-IDF deployment component:

   ```cmake
   target_compile_definitions(${COMPONENT_LIB} PUBLIC GANTRY_GEOMETRY_FROZEN)
   ```

   or, one-shot on the command line:

   ```sh
   idf.py -C idf build -DCMAKE_CXX_FLAGS=-DGANTRY_GEOMETRY_FROZEN
   ```

A second, weaker silencer — `GANTRY_SUPPRESS_GEOMETRY_WARNING` — bypasses the reminder WITHOUT claiming the geometry has been frozen. Use it only for CI or bring-up builds that are knowingly running with dev-rig geometry.

The gate is implemented so that only the `src/main.cpp` translation unit emits the reminder (via `#define AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS` ahead of the include chain). This keeps the build log clean of duplicate copies from the dozen-or-so other TUs that transitively pull the header in.

---

## 10. Diagnostic Compile-Time Toggles

Inherited from the reset-loop investigation. All default to disabled in the production build. Pass to `idf.py build` via `-D...=1`.

| Flag | Effect |
|---|---|
| `DIAG_SKIP_ALL_MCP_INIT_OUTPUT_WRITES` | Skip all post-init MCP output writes. |
| `DIAG_SKIP_MCP_OUTPUT_DIRECTION_CONFIG` | Skip output-direction setup. |
| `DIAG_SKIP_MCP_INPUT_DIRECTION_CONFIG` | Skip input-direction setup. |
| `DIAG_SKIP_MCP_INPUT_PULLUP_CONFIG` | Skip input pull-up setup. |
| `DIAG_MCP_SPI_CLOCK_HZ_OVERRIDE=<hz>` | Override SPI clock (default 1 MHz). |
| `DIAG_HALT_AFTER_MCP_INIT=1` | Halt after MCP setup (isolates downstream bring-up). |
| `DIAG_STARTUP_HALT_PHASE=<0..5>` | 1: after direct pulse GPIO; 2: after driver wiring; 3: before `begin`; 4: after `begin`; 5: after `enable`. |
| `DIAG_SKIP_DIRECT_PULSE_GPIO_CONFIG=1` | Skip X/Y pulse-pin `gpio_config`. |
| `GANTRY_DIAG_SKIP_AXIS_X_INIT=1` | Skip X init inside `Gantry::begin()`. |
| `GANTRY_DIAG_SKIP_AXIS_Y_INIT=1` | Skip Y init inside `Gantry::begin()`. |
| `GANTRY_DIAG_SKIP_THETA_INIT=1` | Skip Theta init inside `Gantry::begin()`. |

Full rationale: `RESET_LOOP_DIAGNOSTICS.md`.

---

## 11. Known Bugs & Gotchas

### 11.1 X-axis encoder feedback (history and current state)

**Motion command path — resolved.** The original "`move` accepts the command but motion never starts" symptom (`x_cmd` flat after `OK Move started`) is no longer reproducible on the consolidated `main`. The state machine exercises the full path: `startSequentialMotion` → `Y_DESCENDING` → `X_MOVING` → `startXAxisMotion` → `axisX_.moveRelative(...)`, and `LIVE POS` shows `x_cmd` ramping smoothly to the commanded target.

**Encoder feedback pin — fixed in firmware.** An earlier hardware trace showed `x_enc=0.00 mm` across the entire ramp while `x_cmd` advanced normally, and the `[X_MOVE] ... encoder=0 ...` log confirmed PCNT was seeing no counts. Root cause was `PIN_X_ENC_A` being mapped to **GPIO34**, which `WT32_ETH01_PINOUT.md` lists as *"Not routed to header"* — i.e. no physical pad existed to wire the encoder A+ signal to. GPIO34 was itself a remap away from the original `35`, which had shared an electrical trace with `MCP23S17_SPI_MISO_PIN`. Firmware now maps `PIN_X_ENC_A` to **GPIO4**, a general-purpose header pin with no strap, Ethernet, or peripheral-allocation conflict; PCNT routes any GPIO through the input matrix (see `WT32_ETH01_PINOUT.md` §"Design notes"). After re-wiring encoder A+ from whatever pad was previously in use to the GPIO4 header pin and rebuilding, `x_enc` should track `x_cmd` under motion. `PIN_X_ENC_B` remains on GPIO36 (unaffected, exposed on the right header).

**Code pointers (still useful for diagnosis):**

- `src/gantry_test_console.cpp` — `move` handler around the `g_homeCompletedThisSession && g_calibratedThisSession` gate.
- `lib/Gantry/src/Gantry.cpp:280-310` — `moveTo(JointConfig, ...)` validates via `Kinematics::validate(joint, config_.limits)` then `startSequentialMotion()`.
- `lib/Gantry/src/Gantry.cpp:795-980` — state machine: `startSequentialMotion` → `Y_DESCENDING`/`X_MOVING` → `startXAxisMotion` → `axisX_.moveRelative(...)`.
- `lib/Gantry/src/Gantry.cpp:923-978` — `startXAxisMotion` emits `[X_MOVE] current=... tracked=... driver=... encoder=... target=... delta=... speed=... accel=... decel=...` once per move; the `encoder=` field is the first place to look when chasing zero-feedback issues.

### 11.2 ESP-IDF component cache corruption

The attached build log shows:

```
idf_component_tools.hash_tools.errors.ChecksumsInvalidJson:
  Invalid checksums file:
  E:\Projects\wt32-eth01-base\idf\managed_components\espressif__libsodium\CHECKSUMS.json
```

The file was written empty (JSON decoder sees `Expecting value: line 1 column 1`). Repair steps:

```powershell
cd E:\Projects\wt32-eth01-base\idf
Remove-Item -Recurse -Force .\managed_components
Remove-Item -Force .\dependencies.lock -ErrorAction SilentlyContinue
idf.py fullclean
idf.py reconfigure
idf.py build
```

The IDF Component Manager will re-download every managed component. Ensure ESP-IDF's bundled Python (`C:\Espressif\tools\python\v6.0\venv\Scripts\python.exe`) is the one in use — the traceback shows it fell through to `scoop\apps\python313` once, which can also corrupt the cache on Windows.

### 11.3 `idf/` is gitignored — state lives only locally

If you wipe `idf/` or clone fresh, you must regenerate four files before `idf.py build`:

- `idf/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_eth01_base)
```

- `idf/main/CMakeLists.txt` — lists the application sources plus the `lib/` source trees directly (no separate ESP-IDF components). Base it on the existing checked-in copy from a working tree.
- `idf/main/idf_component.yml` — must declare `espressif/arduino-esp32` and any other managed components.
- `idf/sdkconfig.defaults` — board/feature defaults for the WT32-ETH01.

Do not regenerate `main/CMakeLists.txt` with a `REQUIRES PulseMotor MCP23S17 Gantry` line; those aren't separate components. If that happens, `idf.py reconfigure` fails with `Failed to resolve component 'PulseMotor'`. Revert to the compile-from-lib-tree layout (sources listed directly via `SRC_DIRS` / `INCLUDE_DIRS`).

### 11.4 Strap pins and boot stability

`GPIO0` (Theta PWM), `GPIO2` (Y pulse), and `GPIO12` (MCP MOSI) are ESP32 strap pins. They are safe **only** because:

- GPIO0 is pulled HIGH at reset by external hardware and the firmware drives it via LEDC only after boot.
- GPIO2 is LOW at reset and the firmware keeps it low (`initDirectOutputs` sets it 0) before any pulse activity.
- GPIO12 stays LOW at reset because MCP MOSI is idle until SPI is initialized.

Any rework that drives these pins externally during reset will brick the boot.

### 11.5 `home` + `calibrate` are required after every startup before `move` is allowed

`gantry_test_console.cpp` gates `move` on `g_homeCompletedThisSession && g_calibratedThisSession`. This is by design, not a bug — report as `ERROR: Move blocked. Run 'home' then 'calibrate' after every startup.`
