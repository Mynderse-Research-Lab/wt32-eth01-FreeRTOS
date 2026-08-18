# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Console `test_theta_path`: combined in-band X+Z+theta using a live
  `thetalim`-safe dθ (25–75% window). Requires enable + bring-up first.
- HCS01 eng CLI: `travel --yes`, `save --yes` (C2200), `verify-origin`.
- `arst` while motors disabled pulses HCS01 C0500 then `makeDriveOff`
  (does not Drive ON / WaitAf).
- Console `test_cycle`: enable, EIP bring-up (home+cal), path legs A–F
  at the live console `speed`/`accel`/`decel`, then theta-only G–I at
  `(0,0)` to live `thetalim` min/max and back to 0 at the kinematic
  speed/accel caps. Runs in a worker task so `stop` still works.
  Segment-count checks use commanded waypoints so a sub-mm X residual
  does not fail a Z-only leg.
- Theta (HCS01) production path: CIP target `192.168.1.23` (FKM), assemblies
  101/102 sized 18/14, HIPERFACE soft-home / modulo calibrate, console
  `home t` / `calibrate t` / theta-only `move` gates.
- Console theta knobs matching X/Z: `speed … [deg/s]`, `accel … [ta] [td]`,
  `puu t`, `puucal t`, `thetalim`; `move`/`test_cycle` pass deg/s² (not mm/s²).
- Theta Class 1 T→O demuxes on shared UDP 2222; O→T uses one W5500 UDP
  socket per dest (DIPR cached). FO is config 0 / 101 / 102, O→T **24** /
  T→O **16**, Run/Idle bytes on, bit 8 clear, RPI 2000 µs. Theta FO failure
  is non-fatal (X/Z stay up).
- Three-axis UDP: HoldKA/prime drain T→O; HCS01-only T→O stale does not
  chip-recover X/Z.

### Fixed
- Class 1 cadence: cached-dest O→T defers SENDOK to the next send; T→O
  drain is one RX burst of datagram views; `Sn_TX_WR` is software-cached;
  `Sn_IR` clear is write-only. `eiptiming` prints drain plus pace
  overrun/yield. Live `0x9c6c0` 2026-08-18 12:45 (n=128, 6 s settle):
  exchange p99 **938 GO**; cycle min/p50/p99 **1342/1999/2659**;
  ot_send p99 **452**; drain p99 **421**; `pace overrun=23 yield=2`;
  `soft_miss=0`. Cadence not collapsed onto 2000 µs; reliability clean.
- Class 1 overrun pacing no longer reseeds `last_wake` after
  `xTaskDelayUntil` pdFALSE (that added 1 ms every cycle and stretched
  period to ~3 ms at RPI 2000 µs). Catch-up keeps the period grid; yield
  one tick every 8 consecutive overruns so IDLE1 can still feed the TWDT.
- Class 1 T→O drain: one SPI burst for Sn_RX_RSR+Sn_RX_RD and the UDP
  datagram; tight Sn_CR RECV wait; stop once every connected axis has a
  packet this cycle (cuts exchange after per-dest O→T).
- Absolute PTP preloads the new Position (StartMotion=0) then raises the
  StartMotion edge, matching Home34. Fast-path SM=1 in the same packet as a
  new target was dropped by Kinetix (Z retract 130→30 sat until timeout;
  retry after StopMotion ran).
- HCS01 halt is bit13 1→0 (`makeDriveHalt`); rotary busy is a motion-active
  flag. X enable failure hard-disables Z and Theta.

### Changed
- Theta `home t` aligns joint to drive abs when `|S-0-0051|≤2°` (C0300 at
  mechanical home). Unaligned HIPERFACE (~178°) keeps an offset so the
  cable-neutral pose stays joint 0, but thetalim shrinks to remaining
  drive travel (avoids F2057). `thetalim` cannot widen past that captured
  envelope. ERD04 travel overlay: S-0-0278=36000, S-0-0049=+180,
  S-0-0050=−180 (surgical HTTP `hcs01_eng.py travel`, not a full .par load).
- Home / calibrate / bring-up seek, park, and SAFE_Z return locked at
  **100 mm/s** and **2000 mm/s²** (not console `speed`/`accel`). Creep
  toward switch-clear stays 1 mm/s.
- Joint 0 is the sample at min-switch disable (limit warning deassert) on X
  and Z home/bring-up — no extra offset. Path: in-band X+Z together; SAFE_Z
  interlocks X and theta (not a via). Theta starts at 25% and is scheduled
  to finish by 75% of the in-band traverse; descent below the band waits if
  theta is still turning. Above the band, Z-only.
- World frame: X unchanged; **+Y conveyor downstream**; **+Z down** (toward
  belt). Joint Z=0 / A015 is retracted; A014 is toward the belt. Y and Z
  flipped together (right-handed). PnP descend/grip follows increasing Z.

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
- The previous "Y" axis already used a `+Y = up` semantic, so the rename is lexical at the firmware level — no scalar values need re-signing. Only the conveyor downstream convention (`-Y`) was introduced as a new world-coordinate concept, used by the MQTT bridge / pick path.
- Documentation updated in lockstep: `PROGRAMMING_REFERENCE.md`, `LIBRARIES_OVERVIEW.md`, `RESET_LOOP_DIAGNOSTICS.md`, `MQTT_comms_subsys.md`, and `lib/Gantry/docs/*`.

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
