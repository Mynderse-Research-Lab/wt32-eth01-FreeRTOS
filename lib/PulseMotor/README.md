# PulseMotor

Generic pulse+direction motor driver library for ESP32 (Arduino core via PlatformIO).

PulseMotor is hardware-agnostic: any motor driver whose command interface is a pulse train plus a direction line can be driven with this library. The driver is pulse-domain only — unit conversion (mm, degrees) lives in the consumer, typically a `DrivetrainConfig` carried by a gantry-style wrapper.

## Verified target hardware

| Driver                                      | Motor / actuator               | Configuration notes                                                  |
|---------------------------------------------|--------------------------------|----------------------------------------------------------------------|
| Bergerda **SDF08NK8X** servo driver         | Any Bergerda F-series motor    | PTI mode, CN1 pulse+direction. See `driver_datasheets_and_calculations/SDF08NK8X_manual_V3.0_2024.pdf`. |
| Allen-Bradley **Kinetix 5100** servo driver | Allen-Bradley Kinetix motor    | PTI pulse+direction step mode. See `driver_datasheets_and_calculations/AB_Kinetix5100_user_manual.pdf`. |
| Custom pulse-train driver                   | SCHUNK **ERD 04-40-D-H-N** rotary module | Custom driver consumes HIPERFACE internally and exposes pulse+dir to the ESP32. |

Add more rows as new targets are commissioned. The library itself does not care about branding.

## Features

- Deterministic trapezoidal velocity profile (accel/cruise/decel) driven by `esp_timer` + LEDC.
- Hardware quadrature encoder feedback via PCNT with 64-bit software accumulation.
- Open-loop (command-domain) and closed-loop (encoder-corrected) position control.
- Smooth stopping: `stopMotion(deceleration)` ramps down before latching.
- Named pin fields — no SDF08-era slot-index encoding.
- FreeRTOS mutex wraps public mutating calls when `PULSE_MOTOR_USE_FREERTOS=1` (default).
- Arduino ESP32 core v2 and v3 supported.

## Pin configuration (named fields)

```cpp
PulseMotor::DriverConfig cfg;

cfg.pulse_pin        = PIN_PULSE;     // LEDC output, must be a direct ESP32 GPIO
cfg.dir_pin          = PIN_DIR;
cfg.enable_pin       = PIN_ENABLE;
cfg.alarm_pin        = PIN_ALARM;     // input (active-low)
cfg.alarm_reset_pin  = PIN_ARST;      // output (pulsed)
cfg.encoder_a_pin    = PIN_ENC_A;
cfg.encoder_b_pin    = PIN_ENC_B;
cfg.limit_min_pin    = -1;            // -1 if the gantry layer owns limits
cfg.limit_max_pin    = -1;

cfg.pulse_mode       = PulseMotor::PulseMode::PULSE_DIRECTION;
cfg.encoder_ppr      = 10000;
cfg.max_pulse_freq   = 200000;
cfg.invert_dir_pin   = false;
cfg.invert_output_logic = true;
cfg.enable_encoder_feedback = true;
```

Pins may be native ESP32 GPIOs (plain numbers), MCP23S17 logical pins through the shared `gpio_expander` module, or `GPIO_EXPANDER_DIRECT_PIN()`-encoded direct pins.

## Drivetrain configuration (optional, for unit conversion)

```cpp
PulseMotor::DrivetrainConfig dt;
dt.type                   = PulseMotor::DRIVETRAIN_BALLSCREW;
dt.ballscrew_lead_mm      = 20.0f;   // mm per screw revolution
dt.encoder_ppr            = cfg.encoder_ppr;
dt.motor_reducer_ratio    = 1.0f;

const double ppm = PulseMotor::pulsesPerMm(dt);   // pulses per mm of linear travel
```

For rotary axes (direct drive with optional reducer):

```cpp
PulseMotor::DrivetrainConfig dt;
dt.type                   = PulseMotor::DRIVETRAIN_ROTARY_DIRECT;
dt.output_gear_ratio      = 1.0f;    // output_rev / motor_rev
dt.encoder_ppr            = cfg.encoder_ppr;
dt.motor_reducer_ratio    = 1.0f;

const double ppd = PulseMotor::pulsesPerDeg(dt);  // pulses per degree
```

The driver itself never consumes `DrivetrainConfig`; the struct is a companion data object for consumers.

## Basic usage

```cpp
#include "PulseMotor.h"

PulseMotor::DriverConfig cfg;
// ... configure named pin fields ...

PulseMotor::PulseMotorDriver driver(cfg);

void setup() {
    if (!driver.initialize()) { /* init failed */ return; }
    driver.enable();
    driver.moveToPosition(10000, /*speed_pps*/ 8000,
                          /*accel_pps2*/ 4000, /*decel_pps2*/ 4000);
}

void loop() {
    if (driver.isMotionActive()) return;
    // Motion complete.
}
```

## Thread safety

All public mutating methods take an internal FreeRTOS mutex when `PULSE_MOTOR_USE_FREERTOS=1` (the default). `getStatus()` / `getPosition()` / `isMotionActive()` are const and lock-free.

## Build flags

| Macro                         | Default | Meaning                                                                 |
|-------------------------------|---------|-------------------------------------------------------------------------|
| `PULSE_MOTOR_USE_FREERTOS`    | `1`     | Wrap mutating public methods in a FreeRTOS mutex.                        |
| `PULSE_MOTOR_HOME_ON_BOOT`    | `1`     | Default value for `DriverConfig::home_on_boot`.                          |
| `PULSE_MOTOR_DEBUG`           | `1`     | Enable `ESP_LOGD` debug output.                                          |
| `SDF08NK8X_USE_FREERTOS`      | (none)  | **Legacy alias**: if set by a caller, overrides `PULSE_MOTOR_USE_FREERTOS`. |

## History

This library was originally authored as `SDF08NK8X` for the Bergerda SDF-08-N-K-8X driver. It was generalized to `PulseMotor` once the project committed to driving all three gantry axes (X, Y, Theta) with pulse-train servo/rotary drivers that share the same command-interface shape.
