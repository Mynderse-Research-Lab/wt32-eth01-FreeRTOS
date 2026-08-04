# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Removed
- PulseMotor adapters and `lib/PulseMotor` (production motion is EtherNet/IP only).
- Blocking `stopAndWaitForPhysicalStop` / `cancelAbsoluteToFeedback` / speed-bit
  `isAxisPhysicallyStopped` — clear-edge home/cal now uses the tick-based
  `kStopping` position-delta gate alone.

## [2.1.0] - 2026-05-13

### Changed (breaking)
- Coordinate-frame refactor: the vertical axis is now **`Z`** (`+Z = up`; **joint** `z` = homing datum at the limit-defined zero). Across-belt is `X`; along-belt is `Y` with `-Y` downstream (world coordinate only; the gantry has no Y joint). Joint space is `(x, z, theta)`.
- `Gantry` constructor parameter rename: `yDrv/yDt → zDrv/zDt`.
- Public API renames:
  - `homeY() → homeZ()`, `calibrateY() → calibrateZ()`.
  - `getCurrentY() → getCurrentZ()`.
  - `setYAxisLimits(...) → setZAxisLimits(...)`.
  - `setSafeYHeight(...) → setSafeZHeight(...)`.
  - `setJointLimits(xMin, xMax, yMin, yMax, ...) → setJointLimits(xMin, xMax, zMin, zMax, ...)`.
  - `moveTo(int32_t x, int32_t y, int32_t theta, ...) → moveTo(int32_t x, int32_t z, int32_t theta, ...)`.
- Struct renames:
  - `JointConfig::y → JointConfig::z`.
  - `JointLimits::y_min/max → JointLimits::z_min/max`.
  - `KinematicParameters::y_axis_z_offset_mm → z_axis_y_offset_mm`.
  - `KinematicParameters::gripper_y_offset_mm → gripper_x_offset_mm`.
  - `GantryStatus::currentY_mm/targetY_mm/yMoving → currentZ_mm/targetZ_mm/zMoving`.
- Internal renames:
  - `axisY_ → axisZ_`, `moveYAxisTo(...) → moveZAxisTo(...)`.
  - `MotionState::Y_DESCENDING/Y_RETRACTING → Z_DESCENDING/Z_RETRACTING`.
  - `Constants::DEFAULT_SAFE_Y_HEIGHT_MM → DEFAULT_SAFE_Z_HEIGHT_MM`.
- Application macro renames in `include/`:
  - `PIN_Y_* → PIN_Z_*` (DIR, ENABLE, LIMIT_MIN/MAX, ALARM_STATUS/RESET, PULSE, ENC_A/B).
  - `AXIS_Y_* → AXIS_Z_*` (every per-axis macro).
  - `Y_PULSE_LEDC_CHANNEL → Z_PULSE_LEDC_CHANNEL`, `Y_ENCODER_PCNT_UNIT → Z_ENCODER_PCNT_UNIT`.
  - `GANTRY_SAFE_Y_HEIGHT_MM → GANTRY_SAFE_Z_HEIGHT_MM`.
  - `GANTRY_Y_AXIS_Z_OFFSET_MM → GANTRY_Z_AXIS_Y_OFFSET_MM`.
  - `GANTRY_GRIPPER_Y_OFFSET_MM → GANTRY_GRIPPER_X_OFFSET_MM`.
  - `GANTRY_DIAG_SKIP_AXIS_Y_INIT → GANTRY_DIAG_SKIP_AXIS_Z_INIT`.
- Kinematics: `forward()` sets `pose.y = params.z_axis_y_offset_mm` (the gantry has no Y joint). At 2.1.0 release `pose.z` equalled `joint.z`; **2.1.1** adds `GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM` for bed-referenced `pose.z` (default `0` preserves that behaviour).
- Console tokens: `home y|t` and `calibrate y|t` are renamed to `home z|t` and `calibrate z|t`; `move <x> <y> <t>` is renamed to `move <x> <z> <t>`. `pins` output prints the Z LEDC channel.

### Notes
- The previous "Y" axis already used a `+Y = up` semantic, so the rename is lexical at the firmware level — no scalar values need re-signing. Only the conveyor downstream convention (`-Y`) was introduced as a new world-coordinate concept, used only by the MQTT bridge and Pickup-Algo SRS.
- Documentation updated in lockstep: `PROGRAMMING_REFERENCE.md`, `LIBRARIES_OVERVIEW.md`, `RESET_LOOP_DIAGNOSTICS.md`, `MQTT_comms_subsys.md`, `Pickup_algo_and_MQTTBridge_SRS.md`, and `lib/Gantry/docs/*`.

## [2.1.1] - 2026-05-13

### Changed
- **Z datum vs physical belt/bed:** added compile-time `GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM` in `include/axis_drivetrain_params.h` (default `0` mm). `Kinematics::forward()` sets `pose.z = joint.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM`; `inverse()` subtracts it. Joint `z` remains the homing datum; non-zero values express that the limit-defined zero is offset from the belt/bed touch plane.

## [1.0.0] - 2024-XX-XX

### Added
- Initial library skeleton with modular design
- `GantryConfig` module for configuration structures:
  - `JointConfig` - Joint space representation (x, y, theta)
  - `EndEffectorPose` - Cartesian space representation
  - `JointLimits` - Joint limits validation
  - `KinematicParameters` - Mechanical parameters
  - `GantryConfig` - Complete configuration
- `GantryKinematics` module for forward/inverse kinematics:
  - Forward kinematics (joint space -> cartesian space)
  - Inverse kinematics (cartesian space -> joint space)
  - Joint validation functions
- `GantryTrajectory` module for trajectory planning:
  - Trapezoidal velocity profiles
  - Position interpolation
  - Waypoint structures (for future use)
- PlatformIO library configuration (`library.json`)
- License file (MIT)
- Documentation structure

### Notes
- Library skeleton created for modular gantry control system
- Designed for WT32-ETH01 (ESP32 with 4MB flash, ~320KB RAM)
- Target memory usage: <10KB RAM
- Optimized for embedded systems
