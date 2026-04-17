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
5. [Public API: `SDF08NK8X` servo driver](#5-public-api-sdf08nk8x-servo-driver)
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
├── SDF08NK8X/                # Bergerda servo driver (X-axis)
└── MCP23S17/                 # SPI GPIO expander driver
```

`idf/main/CMakeLists.txt` compiles application + library sources **directly** — there are no separate ESP-IDF custom components under `idf/components/`. If you see `Failed to resolve component 'SDF08NK8X'` (or similar), the `main/CMakeLists.txt` was regenerated with component dependencies and needs to be reverted to compile-from-lib-tree mode (see §11.3 below).

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
3. **Construct `BergerdaServo::DriverConfig xConfig`** with pulse on `PIN_X_PULSE` (direct, LEDC ch0), DIR/EN/ARST/ALM on MCP pins, encoder A/B on direct GPIO via PCNT unit 0. `xConfig.invert_dir_pin = true`.
4. **Construct `Gantry::Gantry gantry(xConfig, PIN_GRIPPER)`** as a `static` (lives for program lifetime).
5. **`gantry.setLimitPins()`, `setYAxisPins()`, `setYAxisStepsPerMm()`, `setYAxisLimits()`, `setYAxisMotionLimits()`, `setThetaServo()`, `setThetaLimits()`, `setThetaPulseRange()`, `setSafeYHeight()`** — full axis configuration.
6. **`gantry.begin()`** — initializes X (SDF08NK8X), Y (stepper), Theta (LEDC servo), end-effector (MCP pin).
7. **`gantry.enable()`** — sets SON for X, enables Y stepper.
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
- `lib/SDF08NK8X/src/SDF08NK8X.h` has optional `SDF08NK8X_USE_FREERTOS` (default `1`) which adds a `SemaphoreHandle_t mutex_` around state.
- The motion profile is driven by an `esp_timer` callback (`rampTimerCallback`) running at 5 ms intervals. That ISR must remain short and must not block on heap or non-ISR-safe FreeRTOS APIs (see header comment in `SDF08NK8X.h`).

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
| `PIN_X_PULSE` | 14 | LEDC ch0 | X-axis step pulse output. |
| `PIN_Y_PULSE` | 2 | LEDC ch1 | Y-axis step pulse output. Strap pin — must be LOW/floating at reset. |
| `PIN_THETA_PWM` | 0 | LEDC ch2 | Theta servo PWM. Strap pin — must be HIGH at reset. |
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
| `PIN_THETA_LIMIT_MIN` | 14 | B.6 | In pull-up | Active-low |
| `PIN_THETA_LIMIT_MAX` | 15 | B.7 | In pull-up | Active-low |

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
Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin);
Gantry(const BergerdaServo::DriverConfig &xConfig,
       const BergerdaServo::DriverConfig &yConfig, int gripperPin);
```

#### Configuration (call before `begin()`)

| Method | Purpose |
|---|---|
| `void setLimitPins(int xMin, int xMax)` | X-axis limit switch pins (MCP or direct). |
| `void setYAxisPins(int step, int dir, int enable = -1, bool invertDir = false, bool enableActiveLow = true)` | Y stepper wiring. |
| `void setYAxisStepsPerMm(float stepsPerMm)` | Y scaling. |
| `void setYAxisLimits(float minMm, float maxMm)` | Y travel envelope. |
| `void setYAxisMotionLimits(float maxSpeedMmPerS, float accelMmPerS2, float decelMmPerS2)` | Y speed profile caps. |
| `void setThetaServo(int pwmPin, int pwmChannel = 0)` | Theta LEDC PWM pin/channel. |
| `void setThetaLimits(float minDeg, float maxDeg)` | Theta angular limits. |
| `void setThetaPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs)` | Pulse window for the analog servo. |
| `void setJointLimits(float xMin, float xMax, float yMin, float yMax, float thetaMin, float thetaMax)` | Joint validation envelope used by `moveTo(JointConfig)`. |
| `void setEndEffectorPin(int pin, bool activeHigh = true)` | Override constructor gripper pin. |
| `void setSafeYHeight(float mm)` | Safe retraction height used before X travel. Default 150 mm. |
| `void setHomingSpeed(uint32_t pps)` | Homing speed (pulses/s). |
| `void setStepsPerRevolution(float steps)` | Default 6000; drives `getPulsesPerMm()`. |

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

### 4.5 Sub-axis classes (normally used through `Gantry`)

- `Gantry::GantryAxisStepper` — cooperative step/dir stepper (Y-axis). `configurePins`, `setStepsPerMm`, `setLimits`, `setMotionLimits`, `begin`, `enable`/`disable`, `moveToMm`, `stop`, `update`, `getCurrentMm`, `getCurrentSteps`.
- `Gantry::GantryRotaryServo` — LEDC-based PWM servo (Theta). `configurePin`, `setAngleRange`, `setPulseRange`, `begin`, `moveToDeg`, `getCurrentDeg`.
- `Gantry::GantryEndEffector` — digital gripper. `configurePin`, `begin`, `setActive`, `isActive`, `getPin`. **Correctly routes pins `<16` through `gpio_expander_*`, pins `>=16` (or direct-flagged) through Arduino `pinMode`/`digitalWrite`.**
- `Gantry::GantryLimitSwitch` — debounced input. `configure(pin, activeLow=true, enablePullup=true, debounceCycles=6)`, `begin`, `update(force=false)`, `isActive`. (Default raised from 3 → 6 cycles; a `debounceCycles` of `0` is silently clamped to `1`.)

### 4.6 Constants (`Gantry::Constants`)

```cpp
constexpr float    DEFAULT_STEPS_PER_REV          = 6000.0f;
constexpr float    DEFAULT_PULSES_PER_MM          = 150.0f;
constexpr float    DEFAULT_SAFE_Y_HEIGHT_MM       = 150.0f;
constexpr uint32_t DEFAULT_HOMING_SPEED_PPS       = 6000;
constexpr uint32_t DEFAULT_SPEED_MM_PER_S         = 50;
constexpr uint32_t DEFAULT_SPEED_DEG_PER_S        = 30;
constexpr uint32_t GRIPPER_ACTUATE_TIME_MS        = 100;
constexpr uint32_t CALIBRATION_TIMEOUT_MS         = 30000;
constexpr uint32_t TRAVEL_MEASUREMENT_TIMEOUT_MS  = 90000;
```

---

## 5. Public API: `SDF08NK8X` servo driver

Namespace: `BergerdaServo`. Header: `lib/SDF08NK8X/src/SDF08NK8X.h`.

### 5.1 Types

```cpp
enum class PulseMode   : uint8_t { PULSE_DIRECTION = 0, CW_CCW = 1, QUADRATURE = 2 };
enum class ControlMode : uint8_t { POSITION = 0, SPEED = 1, TORQUE = 2, JOG = 3 };

struct DriverConfig {
    // Pin indices (see header doxygen for slot semantics):
    int input_pin_nos[6];     // [0]POS_reached [1]BRAKE [2]ALM [3]A+ [4]B+ [5]Z+
    int output_pin_nos[8];    // [0]SON [1]ARST [2]CW/CCW [3]CLE [4]INH [5]GEARI [6]PULS [7]SIGN

    PulseMode    pulse_mode;      // default PULSE_DIRECTION
    ControlMode  control_mode;    // default POSITION

    uint32_t max_pulse_freq;      // Hz, default 600 000
    uint32_t encoder_ppr;         // default 12 000
    double   gear_numerator;      // default 1.0
    double   gear_denominator;    // default 1.0

    // LEDC pulse generation
    int     ledc_channel;         // default 0
    int     ledc_pulse_pin;       // default -1 (falls back to output_pin_nos[6])
    uint8_t ledc_resolution;      // default 2 bits (for high freq)

    bool enable_encoder_feedback;   // default false
    bool enable_closed_loop_control;// default false
    bool invert_output_logic;       // default true (LOW = logical HIGH)
    bool invert_dir_pin;            // default false
    int  pcnt_unit;                 // app-level id; hardware allocated dynamically
    bool home_on_boot;              // default HOME_ON_BOOT (1)
    uint32_t homing_speed_pps;      // default 6000

    uint8_t  limit_debounce_cycles;     // default 10
    uint16_t limit_sample_interval_ms;  // default 3
    bool     limit_log_changes;         // default true
    int      limit_min_pin;             // default -1
    int      limit_max_pin;             // default -1
};

struct DriveStatus {
    bool     servo_enabled, position_reached, brake_released, alarm_active;
    uint32_t current_position;
    int32_t  encoder_position, position_error;
    uint32_t current_speed, last_update_ms;
};

struct MotionProfile {
    enum Phase : uint8_t { IDLE, ACCEL, CRUISE, DECEL };
    // (accumulators, phase, direction, timestamps — internal)
};

using AlarmCallback           = void (*)(const char* alarm_code);
using PositionReachedCallback = void (*)(uint32_t position);
using StatusUpdateCallback    = void (*)(const DriveStatus&);
```

### 5.2 `BergerdaServo::ServoDriver`

```cpp
explicit ServoDriver(const DriverConfig& config);
~ServoDriver();
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

bool     startHoming(uint32_t speed = 10000);
bool     isHoming() const;
bool     measureTravelDistance(uint32_t speed = 10000,
                               uint32_t accel = 10000,
                               uint32_t decel = 10000,
                               uint32_t timeout_ms = 60000);
bool     hasTravelDistance() const;
uint32_t getTravelDistance() const;

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
String              getConfigStatus() const;
String              getDriverInfo()   const;

static String getVersion();
```

Key implementation details:

- Pulse output via **LEDC** (default ch0, 2-bit resolution to allow very high freq).
- Encoder via **`pulse_cnt`** (ESP-IDF v5+ API), with 64-bit `encoder_accumulator_` to survive 16-bit wrap.
- `esp_timer_handle_t ramp_timer_` fires `rampTimerCallback` every 5 ms to update the trapezoidal profile.
- All public methods take `mutex_` when `SDF08NK8X_USE_FREERTOS=1` (default).

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

All in `include/gantry_app_constants.h`.

```c
// Hardware peripheral channels
#define X_PULSE_LEDC_CHANNEL      0
#define Y_PULSE_LEDC_CHANNEL      1
#define THETA_PWM_LEDC_CHANNEL    2
#define X_ENCODER_PCNT_UNIT       0
#define Y_ENCODER_PCNT_UNIT       1

// MCP defaults
#define MCP23S17_DEVICE_ADDRESS         0x00
#define MCP23S17_CLOCK_HZ               10000000
#define MCP23S17_SPI_CLOCK_HZ_WORKING    1000000   // actually used
#define MCP_DEBUG_CMDS                  1

// Motion defaults
#define GANTRY_HOMING_SPEED_PPS    6000
#define GANTRY_ENCODER_PPR         6000
#define GANTRY_MAX_PULSE_FREQ      10000
#define GANTRY_X_MIN_MM            0.0f
#define GANTRY_X_MAX_MM            200.0f
#define GANTRY_Y_STEPS_PER_MM      200.0f
#define GANTRY_Y_MIN_MM            0.0f
#define GANTRY_Y_MAX_MM            200.0f
#define GANTRY_Y_MAX_SPEED_MMPS    100.0f
#define GANTRY_Y_ACCEL_MMPS2       500.0f
#define GANTRY_Y_DECEL_MMPS2       500.0f
#define GANTRY_THETA_MIN_DEG      -90.0f
#define GANTRY_THETA_MAX_DEG       90.0f
#define GANTRY_THETA_MIN_PULSE_US  1000
#define GANTRY_THETA_MAX_PULSE_US  2000
#define GANTRY_SAFE_Y_MM           150.0f
```

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

Do not regenerate `main/CMakeLists.txt` with a `REQUIRES SDF08NK8X MCP23S17 Gantry` line; those aren't separate components. If that happens, `idf.py reconfigure` fails with `Failed to resolve component 'SDF08NK8X'`. Revert to the compile-from-lib-tree layout (sources listed directly via `SRC_DIRS` / `INCLUDE_DIRS`).

### 11.4 Strap pins and boot stability

`GPIO0` (Theta PWM), `GPIO2` (Y pulse), and `GPIO12` (MCP MOSI) are ESP32 strap pins. They are safe **only** because:

- GPIO0 is pulled HIGH at reset by external hardware and the firmware drives it via LEDC only after boot.
- GPIO2 is LOW at reset and the firmware keeps it low (`initDirectOutputs` sets it 0) before any pulse activity.
- GPIO12 stays LOW at reset because MCP MOSI is idle until SPI is initialized.

Any rework that drives these pins externally during reset will brick the boot.

### 11.5 `home` + `calibrate` are required after every startup before `move` is allowed

`gantry_test_console.cpp` gates `move` on `g_homeCompletedThisSession && g_calibratedThisSession`. This is by design, not a bug — report as `ERROR: Move blocked. Run 'home' then 'calibrate' after every startup.`
