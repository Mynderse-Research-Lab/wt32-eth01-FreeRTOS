# PulseMotor Library

Generic pulse-train motor control interface for ESP32-based motion systems.

## Purpose

`PulseMotor` provides a driver-facing configuration model with named pins and
drivetrain metadata so application code can be reused across pulse-train
drivers (SDF08NK8X, Kinetix 5100 pulse-input mode, and custom pulse-train
drivers).

## Main Types

- `PulseMotor::DriverConfig`: named electrical pin assignments and driver tuning.
- `PulseMotor::PulseMotorDriver`: pulse-domain motion API.
- `PulseMotor::DrivetrainType` and `PulseMotor::DrivetrainConfig`: mechanical
  conversion metadata used by higher-level controllers.

## Notes

- The driver API remains in pulse units (`moveToPosition`, `moveRelative`).
- Mechanical conversions (`pulses/mm`, `pulses/deg`) are exposed as helpers and
  are intended for the Gantry layer.
