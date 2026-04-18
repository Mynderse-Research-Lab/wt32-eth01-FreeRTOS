# Gantry Development Journal

**Version:** 2.0.0
**Last Updated:** Apr 17, 2026

This journal captures the full development path of the current Gantry library, including architecture moves, behavior changes, safety fixes, console/diagnostic additions, and branch/merge decisions.

---

## 0) 2026-04: PulseMotor unification refactor

- Generalised `lib/SDF08NK8X` into `lib/PulseMotor`.
  - Namespace `BergerdaServo` -> `PulseMotor`; class `ServoDriver` -> `PulseMotorDriver`.
  - Replaced the SDF08-specific pin slot arrays (`input_pin_nos[6]`, `output_pin_nos[8]`) with named fields on `DriverConfig` (`pulse_pin`, `dir_pin`, `enable_pin`, `alarm_pin`, `alarm_reset_pin`, `encoder_a_pin`, `encoder_b_pin`, ...).
  - Removed the dead `ControlMode` enum (the library only ever supported `POSITION`).
  - Introduced `PulseMotor::DrivetrainConfig` and `PulseMotor::DrivetrainType` (`DRIVETRAIN_BALLSCREW`, `DRIVETRAIN_BELT`, `DRIVETRAIN_RACKPINION`, `DRIVETRAIN_ROTARY_DIRECT`) with header-inline `pulsesPerMm` / `pulsesPerDeg` helpers. The driver itself stays strictly pulse-domain; unit conversion lives in the consumer.
- Restructured the Gantry library around two abstract axis interfaces:
  - `GantryLinearAxis` (mm) -> `GantryPulseMotorLinearAxis` (backs ballscrew / belt / rack-pinion).
  - `GantryRotaryAxis` (deg) -> `GantryPulseMotorRotaryAxis` (backs rotary-direct).
  - `Gantry::Gantry` now holds `std::unique_ptr<GantryLinearAxis>` for X and Y and `std::unique_ptr<GantryRotaryAxis>` for Theta. Concrete classes are factoried at construction time from the per-axis `DrivetrainType`.
- Retired `GantryAxisStepper` (cooperative step/dir stepper) and `GantryRotaryServo` (50 Hz PWM hobby servo). All production axes are now pulse-train.
- Split configuration across three headers:
  - `include/gantry_app_constants.h` narrowed to pin map + peripheral allocation + task parameters.
  - `include/axis_pulse_motor_params.h` (new) owns electrical tuning (encoder PPR, pulse bandwidth, gear ratios, inversion flags, homing, debounce) per axis.
  - `include/axis_drivetrain_params.h` (new) owns mechanical tuning (drivetrain type + type-specific fields, travel envelope, motion caps, position tolerance, gripper timing, kinematic offsets). Includes derived helpers and a reserved slot for Trap-Move sizing values.
- Updated `src/main.cpp` to build three `(DriverConfig, DrivetrainConfig)` pairs via per-axis helper functions that consume the two new headers.
- Rewrote `examples/BasicDriverTest/` and `examples/Move100Steps/` against the new API.
- Added `driver_datasheets_and_calculations/INDEX.md` listing present and pending hardware datasheets. Relocated the SDF08 datasheet to a normalised filename.
- Build flag `SDF08NK8X_USE_FREERTOS` is aliased by `PULSE_MOTOR_USE_FREERTOS` so legacy `platformio.ini` flags do not silently flip the threading model during the rename.

Verified target stacks after the refactor:

| Axis  | Drivetrain        | Driver                      | Actuator                    |
|-------|-------------------|-----------------------------|-----------------------------|
| X     | DRIVETRAIN_BELT   | Allen-Bradley Kinetix 5100  | SCHUNK Beta 100-ZRS         |
| Y     | DRIVETRAIN_BALLSCREW | Allen-Bradley Kinetix 5100 | SCHUNK Beta 80-SRS          |
| Theta | DRIVETRAIN_ROTARY_DIRECT | Custom pulse-train driver | SCHUNK ERD 04-40-D-H-N |

---

## 1) Major Architecture Changes

- Migrated the application to a FreeRTOS-style structure.
- Split serial/test console logic out of `main.cpp` into `src/gantry_test_console.cpp`.
- Moved pin and app constants into `include/gantry_app_constants.h`.
- Added dual hardware mode support:
  - MCP23S17 IO expander mode.
  - Direct WT32 GPIO mode for test bring-up without MCP23S17.
- Added reusable limit-switch object in Gantry library:
  - `lib/Gantry/src/GantryLimitSwitch.h`
  - `lib/Gantry/src/GantryLimitSwitch.cpp`
- Shifted limit ownership from driver-internal logic to Gantry-level logic for reuse across axes.

---

## 2) Safety, Alarm, and Motion Behavior Updates

- Alarm behavior:
  - Homing and calibration are blocked when alarm is active.
  - Gantry alarm status is driven by driver status state (not raw app-level pin interpretation).
- Motion-stop behavior:
  - Added force-stop behavior when Gantry detects motion-path failures.
  - Expanded stop-on-failure handling beyond X to Y/Theta command paths.
- Calibration robustness:
  - Added MIN-release watchdog after starting MIN->MAX travel.
  - Added explicit warning log when MIN-release watchdog aborts calibration.
- Startup flow:
  - Homing + calibration session gating retained.
  - Move commands remain blocked until startup sequence succeeds.

---

## 3) Console and Runtime Diagnostics

- Added/extended console commands and feedback:
  - `pins`, `limits`, `status`, `alarmreset`/`arst`, `speed`, `accel`, `selftest`.
- Added control-variable flip monitor with debounce and realtime output.
- Added live motion telemetry while busy:
  - `LIVE POS: x_cmd=..., x_enc=..., y=..., theta=...`
- `status` formatting kept stable per operator request; live telemetry carries expanded dual-X visibility.

---

## 4) Pinout and Test Wiring Evolution

- Temporary direct test mapping aligned to SDF08 driver test branch:
  - Pulse/Dir/Enable/ALM/ARST wired directly to WT32 pins.
- ARST test pin enabled and moved to GPIO32.
- Limit switches required in active axis modes and monitored in runtime diagnostics.
- Added spreadsheet-friendly pinout artifacts:
  - `pinout.csv`

---

## 5) Build/Tooling and IDF Integration

- Standardized project indexing/build flow around ESP-IDF and compile database.
- Removed PlatformIO-centric index assumptions from active setup.
- Added/updated IDF component registration and include wiring for:
  - Gantry
  - SDF08NK8X
  - MCP23S17
- Kept build validation loop with `idf.py build` after each substantive behavior change.

---

## 6) Driver-Library Branch Operations

- Converted driver serial prints to ESP logging style.
- Removed driver-side limit-switch instance/debounce handling in driver branch.
- Merged driver-branch changes back into Gantry branch line.
- Preserved app-level visibility of driver logs through test console workflow.

---

## 7) Current Known Operational Signals

- If `x_cmd` changes while `x_enc` stays flat:
  - Command generation is active, but encoder feedback is not changing.
  - This indicates feedback path, encoder mode, or physical wiring/config mismatch.
- If calibration starts at MIN and quickly fails:
  - Check MIN release behavior and direction polarity.
  - Use live telemetry + control flips + `limits` command to diagnose release timing.

---

## 8) Remaining Cleanup Opportunities

- Consider replacing fixed MIN-release timeout with distance-based criterion.
- Continue reducing non-essential debug noise in normal mode while retaining deep diagnostics in debug mode.
- Migrate legacy PCNT usage warnings to new pulse counter API when practical.

---

## 9) Related Files

- Core application:
  - `src/main.cpp`
  - `src/gantry_test_console.cpp`
- Core Gantry library:
  - `lib/Gantry/src/Gantry.cpp`
  - `lib/Gantry/src/Gantry.h`
  - `lib/Gantry/src/GantryLimitSwitch.cpp`
  - `lib/Gantry/src/GantryLimitSwitch.h`
- Driver integration:
  - `lib/SDF08NK8X/src/SDF08NK8X.cpp`
  - `lib/SDF08NK8X/src/SDF08NK8X.h`
- Integration docs:
  - `PROGRAMMING_REFERENCE.md` (root)
  - `LIBRARIES_OVERVIEW.md` (root)
  - `lib/Gantry/docs/API_REFERENCE.md`
  - `lib/Gantry/docs/ARCHITECTURE.md`

